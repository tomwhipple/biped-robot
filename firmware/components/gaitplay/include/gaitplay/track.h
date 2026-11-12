// On-board gait track (#297 B2): a queue of time-stamped leg keyframes the host appends WHILE it plays,
// interpolated at the 50 Hz tick, with the gait phase that drives the stance trim. Pure: no hardware, no
// clock, no globals -- the CLI feeds it frames and asks for the targets at a track time.
//
// Why a track: the robot has no 50 Hz joint-target path from the host (the UDP link carries policy
// commands; a CLI pose costs ~100 ms), and a continuous sway needs a target every tick. The host plans one
// or two steps ahead (the sway gait: experiments/roll-balance/sway.py with the stepper's IK), uploads them
// as keyframes and keeps appending; the firmware plays them on its own clock, immune to Wi-Fi jitter.
//
// Frames. Each has a time on the track's own timeline (ms, strictly increasing), the 12 leg joint goals in
// servo ticks (absolute, in kLegJoints order), the gait phase phi (0..2pi, the policy convention: the LEFT
// foot swings while sin(phi) > 0) and flags: kLift (the gait lifts a foot in this frame's half cycle: the
// trim runs), kStepEnd (both feet down, a step is complete: a stop may end here), kRest (NOT on the
// timeline: the centred two-feet stand for the step that ends at this frame's time; sent right after its
// kStepEnd frame with ddd = 0, kept apart as the stop target for that step end).
// Interpolation: Catmull-Rom on the ticks between the bracketing frames (the clamped neighbours at the
// ends), linear on the unwrapped phase.
//
// Stop. `stop()` (or the queue running dry) lets the track run to the next kStepEnd frame, drops what is
// queued after it, then moves to the last kRest pose over rest_ms (min-jerk) and holds: done. A queue that
// runs dry with no kStepEnd ahead holds the last frame (the host must append whole steps). Nothing ever
// returns to a pre-lift pose.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace gaitplay {

constexpr int kJoints = 12;
// the order of the 12 goals in a frame (bus joint names)
inline constexpr const char* kLegJoints[kJoints] = {
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"};
constexpr int kCap = 256;           // queued frames (25 s at 10 Hz keyframes)

enum Flags : uint8_t { kLift = 1, kStepEnd = 2, kRest = 4 };

struct Frame {
    uint32_t t_ms = 0;
    int16_t q[kJoints] = {};
    float phi = 0.0f;               // rad, 0..2pi
    uint8_t flags = 0;
};

// One frame from its compact text form (the CLI's `track add`), 42 hex digits:
//   ddd pp f qqq x12      ddd = ms after the previous frame (0x000..0xFFF), pp = phi * 256 / 2pi,
//                         f = flags, qqq = goal ticks 0..4095 per joint (kLegJoints order)
// `prev_t` is the previous frame's time (the first frame of a track: its ddd is its own time).
// false on a malformed token.
bool parseFrame(const char* hex, uint32_t prev_t, Frame* out);
// The inverse (for tests and the host twin): writes 42 chars + NUL.
void formatFrame(const Frame& f, uint32_t prev_t, char* out43);

enum class State : uint8_t { kIdle = 0, kPlaying, kStopping, kResting, kDone };

struct Sample {
    int32_t q[kJoints];
    float phi;          // rad, unwrapped within the bracket, then wrapped to 0..2pi
    bool lift;          // the bracketing frames' kLift (the earlier one's)
    bool step_end;      // a kStepEnd frame was passed since the last sample
};

class Track {
  public:
    void clear();
    // Append (to a playing track too). false: full, out of order (t <= the last queued), or after a stop.
    bool add(const Frame& f);
    int queued() const { return n_; }
    // The k-th queued frame on the timeline (0 = the next / current bracket start), nullptr past the end.
    const Frame* peek(int k) const { return (k >= 0 && k < n_) ? &at(k) : nullptr; }
    uint32_t lastTime() const { return n_ ? buf_[idx(n_ - 1)].t_ms : last_t_; }
    // Start at track time = the first frame's time. Needs >= 2 frames on the timeline.
    bool start();
    // Stop at the next step end, then rest (see above). Idempotent.
    void stop();
    // The targets at track time t_ms (monotone between calls; the caller's clock). Frames before the
    // bracket are dropped (their slots are free for appends). Returns the state after the sample.
    State sample(uint32_t t_ms, Sample* s);
    State state() const { return state_; }
    bool haveRest() const { return have_rest_; }
    uint32_t restMs() const { return rest_ms_; }
    void setRestMs(uint32_t ms) { rest_ms_ = ms < 200 ? 200 : ms > 5000 ? 5000 : ms; }
    uint32_t startTime() const { return t_start_; }
    uint32_t steps() const { return steps_; }             // kStepEnd frames passed

  private:
    int idx(int k) const { return (head_ + k) % kCap; }
    const Frame& at(int k) const { return buf_[idx(k)]; }
    void beginRest(const int32_t* from, uint32_t t_ms);
    Frame buf_[kCap];
    int head_ = 0, n_ = 0;
    uint32_t last_t_ = 0;
    State state_ = State::kIdle;
    bool stop_req_ = false;
    int stop_at_ = -1;              // queue index of the step end the stop ends at (-1 = none yet)
    static constexpr int kRestCap = 16;
    Frame rests_[kRestCap];         // rest poses by their step end's time (a ring, oldest overwritten)
    int rest_n_ = 0, rest_head_ = 0;
    const Frame* restFor(uint32_t t_step_end) const;
    Frame rest_{};
    bool have_rest_ = false;
    uint32_t rest_ms_ = 1000, rest_t0_ = 0, t_start_ = 0, steps_ = 0;
    int32_t rest_from_[kJoints] = {};
    int32_t last_q_[kJoints] = {};
    float last_phi_ = 0.0f;
    Frame prev_{}, prev2_{};        // the last two frames the clock has passed (interpolation anchors)
    bool have_prev_ = false, have_prev2_ = false;
};

// The stance trim's input from the gait phase (one mechanism for the track and a policy's gait clock).
// Half cycle u = (phi mod pi) / pi; the swing side is L while sin(phi) > 0 (phi in 0..pi), R otherwise.
// Single support = the middle half of each half cycle, u in [0.25, 0.75]; within it the raise is the first
// 35 % (progress 0 -> 1) and the lowering starts at 65 %.
struct Support {
    char stance;        // 'L' / 'R' in single support, 0 in double support or with no lift
    float progress;     // 0..1 over the raise; 1 after it
    bool lowering;      // the last 35 % of the single support
};
Support supportFromPhase(float phi, bool lift);

}  // namespace gaitplay
