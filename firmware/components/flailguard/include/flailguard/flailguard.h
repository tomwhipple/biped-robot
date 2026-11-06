// Flail guard: "the policy is doing something clearly wrong" -- stop it and
// HOLD, before the robot is a fall.
//
// 2026-11-01 21:39 UTC, the first armed policy stand (robot_legs_walk14_s1_
// s128r24, tethered `run`, zero command): ~3.3 s of a clean stand, then the
// policy's targets began to chatter and swing -- hip rolls to -24/+22 deg,
// knees to -35..-95 deg, feet kicked high -- and nothing on the robot stopped
// it. The fall guard (fallguard.h, 45 deg / 200 ms) only fires once the torso
// is going over, and the robot never got there on its own: Tom had to grab it
// and TILT it to trip the guard. The control loop itself was healthy (0 late
// ticks). The policy was the fault, and nothing judged the policy.
//
// This module judges it, every armed tick, on four criteria. Any one trips:
//
//   pose      at the plain-stand command (and after the 1 s takeover ramp),
//             any joint's WRITTEN target more than its limit from the trained
//             stand pose, continuously for pose_hold_ms. The written target is
//             the shaped one (obs::CommandShaper), so single-tick chatter does
//             not count here -- `chatter` below owns that.
//   rate      the torso's angular-rate magnitude |gyro|, low-passed (EMA,
//             rate_tau_s), above rate_rad_s. A stand is nearly still; a robot
//             thrashing its legs is not.
//   tilt      torso tilt past tilt_deg, continuously for tilt_hold_ms: well
//             before the fall guard's 45 deg, while holding still has a chance.
//   chatter   the policy's RAW targets (before the shaper) changing fast: the
//             mean over joints of |d target / dt|, low-passed (EMA,
//             chatter_tau_s), above chatter_rad_s. Sign-flipping
//             +/-15..80 deg per tick is what the recording shows from ~3.2 s.
//
// pose (and only pose) is gated on the stand command: under a walk command the
// joints are SUPPOSED to leave the stand pose. rate, tilt and chatter apply to
// every command.
//
// THE LIMITS (kLimits), tuned on the recording of that failure
// (hw_sessions/2026-11-01/flight_20261101T213631Z_policy_stand1; the first 4 s
// are frozen into host/vectors/flail_vectors.h by tools/gen_flail_vectors.py,
// and host/test_flailguard.cpp asserts no criterion trips in the clean first
// 3 s and the guard trips by 3.5 s):
//
//   pose      15 deg every joint (poseLimitDeg), 300 ms
//   rate      1.5 rad/s, EMA tau 0.2 s     (peak 1.07 in the clean 3 s)
//   tilt      20 deg, 200 ms               (peak 3.7 deg in the clean 3 s)
//   chatter   7.0 rad/s mean (= 8 deg per 20 ms tick), EMA tau 0.1 s
//             (peak 3.9 rad/s in the clean 3 s; the sim's stand runs at an
//             action rate ~0.08/tick RMS, i.e. ~4 deg/tick)
//
// What the CALLER does on a trip (ctrl_task.cpp): stop the policy, freeze
// every servo's goal at its present position with torque ON (a stiff hold,
// NOT limp -- going limp is the fall guard's job, past 45 deg), disarm to
// bench with ArmResult kDisarmedFlail, and hand over to the bench, which homes
// the robot once it is upright and still (cli.cpp flailTick).
//
// Pure: no hardware, no clock (dt arrives with every call), no globals --
// host/test_flailguard.cpp asserts the properties. ctrl_task owns the one
// instance (core 1).
#pragma once
#include <stdint.h>

namespace flailguard {

constexpr int kMaxJoints = 17;   // == obs::kNumBusJoints; the policy drives <= this

enum class Trip : uint8_t {
    kNone = 0,
    kPose = 1,
    kRate = 2,
    kTilt = 3,
    kChatter = 4,
};

// Criteria, as a mask (Limits::criteria). The host test runs each alone to
// report its own trip time on the recording.
constexpr uint8_t kCritPose = 1u << 0;
constexpr uint8_t kCritRate = 1u << 1;
constexpr uint8_t kCritTilt = 1u << 2;
constexpr uint8_t kCritChatter = 1u << 3;
constexpr uint8_t kCritAll = kCritPose | kCritRate | kCritTilt | kCritChatter;

struct Limits {
    uint32_t pose_hold_ms;     // any joint past its pose limit this long
    float rate_rad_s;          // EMA |gyro| above this
    float rate_tau_s;
    float tilt_deg;            // tilt above this ...
    uint32_t tilt_hold_ms;     // ... this long
    float chatter_rad_s;       // EMA of mean |d raw target / dt| above this
    float chatter_tau_s;
    uint8_t criteria;          // kCrit* mask
    // rate's limit while WALKING (Input::walking): Tom 2026-11-05, for a 5-step walk.
    // Every one-cycle stride on the robot tripped 1.5 at the step-onset kick (EMA
    // 1.50 - 1.61 rad/s); in sim the deployed walker peaks 1.6 - 1.85 while walking
    // and does not fall. A walking trip still recovers (WalkRecovery), never holds.
    // 2.5 -> 3.0 (Tom, same evening): the 5-step run reached ~3 steps, then tripped at 2.57.
    float rate_walk_rad_s = 3.0f;
};

constexpr Limits kLimits = {300u, 1.5f, 0.2f, 20.0f, 200u, 7.0f, 0.1f, kCritAll};

// The pose limit for a joint, by its name (obs_spec kJointNames), degrees.
// One table, by joint kind; unknown names get the default. 15 deg everywhere
// today -- the table exists so a joint that legitimately moves more at a
// stand can be given room without touching the others.
float poseLimitDeg(const char* joint_name);

const char* tripName(Trip t);

struct Input {
    int n = 0;                       // joints in the arrays below (<= kMaxJoints)
    const float* target = nullptr;   // rad: what the loop writes this tick
    const float* raw = nullptr;      // rad: the policy's raw target (pre-shaper)
    const float* stand = nullptr;    // rad: the trained stand pose
    const float* pose_limit = nullptr;   // rad per joint
    bool stand_cmd = false;          // the command is the plain stand
    bool walking = false;            // a walk/step command now or within WalkRecovery's
                                     // recent_ms, and no recovery engaged: rate uses
                                     // Limits::rate_walk_rad_s
    bool pose_armed = false;         // takeover ramp finished
    bool imu_valid = false;          // fresh, finite attitude this tick
    const float* up = nullptr;       // torso z axis in the world (imu::Sample::up)
    const float* gyro = nullptr;     // body rates, rad/s
};

class Detector {
  public:
    explicit Detector(const Limits& limits = kLimits) : L_(limits) {}

    // A fresh run (the arm handover): every run, filter and the latch.
    void reset();

    // One control tick, dt_ms after the previous one. Returns the criterion
    // on the tick the guard trips, kNone otherwise -- and kNone on every tick
    // after, until reset(): one event per run, the caller acts once.
    Trip update(const Input& in, uint32_t dt_ms);

    bool tripped() const { return trip_ != Trip::kNone; }
    Trip trip() const { return trip_; }
    // The tripping criterion's value at the trip (deg for pose/tilt, rad/s
    // for rate/chatter) and, for pose, the joint index (else -1).
    float tripValue() const { return trip_value_; }
    int tripJoint() const { return trip_joint_; }
    uint32_t ticks() const { return ticks_; }           // ticks since reset

    // Live filters and runs, and the peaks since reset (for `flail` status:
    // after a clean stand, how close did each criterion come?).
    float rateEma() const { return rate_ema_; }
    float chatterEma() const { return chatter_ema_; }
    float peakRate() const { return peak_rate_; }
    float peakChatter() const { return peak_chatter_; }
    float peakPoseDeg() const { return peak_pose_deg_; }   // worst |target - stand|, gated like pose
    float peakTiltDeg() const { return peak_tilt_deg_; }
    const Limits& limits() const { return L_; }

  private:
    Limits L_;
    Trip trip_ = Trip::kNone;
    float trip_value_ = 0.0f;
    int trip_joint_ = -1;
    uint32_t ticks_ = 0;
    uint32_t pose_run_ms_ = 0, tilt_run_ms_ = 0;
    bool pose_cand_ = false, tilt_cand_ = false;
    float rate_ema_ = 0.0f, chatter_ema_ = 0.0f;
    float prev_raw_[kMaxJoints] = {};
    bool have_prev_ = false;
    float peak_rate_ = 0.0f, peak_chatter_ = 0.0f, peak_pose_deg_ = 0.0f, peak_tilt_deg_ = 0.0f;
};

// WalkRecovery: what a trip DURING A WALK does (Tom 2026-11-05: "modify the flail
// guard to just disable the action policy but let the IMU loop keep running").
//
// 2026-11-05 02:51Z, stride1_03 on the robot: the guard tripped on rate (EMA
// 1.5 rad/s) 0.87 s after the step command -- 0.2 s after the one-cycle command had
// already returned to the stand -- froze the joints, and the robot fell 0.8 s later.
// In sim, holding the joints at the trip turns nearly every tripped stride / walk into
// a fall, while the policy left in control rides out the 1.6 - 2.5 rad/s peaks with 0
// falls (3 policies x 6 probes x 8 seeds, the measured-sole plant, refit actuator).
//
// So: a trip within recent_ms of a MOVING command (any channel off the plain stand)
// does NOT hold. The policy keeps running -- it is the IMU-in-the-loop controller --
// but its command is taken to the plain stand (over ramp_ms, 0 = at once), latched
// for the rest of the run: it finishes the step and balances to a stand, as in every
// walk -> stop it trained on. The guard pauses for grace_ms (its rate filter is still
// past the limit, and the stop itself moves the body), then judges afresh at the
// stand; a second trip, or any trip with no recent moving command, holds and
// disarms exactly as before. Once per run. The fall guard (fallguard.h) is separate
// and unchanged: past 45 deg it still cuts torque.
//
// Pure logic: the caller feeds it one tick at a time (host/test_flailguard.cpp).
class WalkRecovery {
  public:
    struct Config {
        uint32_t recent_ms = 2000;   // a trip this soon after a moving command recovers
        uint32_t grace_ms = 2000;    // the guard does not judge for this long after
        uint32_t ramp_ms = 300;      // the command reaches the stand over this (0: at once);
                                     // sim 11-05: 0 falls either way, the ramp has the lower
                                     // post-trip goal acceleration (median 0.66 vs 0.76 of the stride's)
    };
    WalkRecovery() : C_(Config{}) {}
    explicit WalkRecovery(const Config& c) : C_(c) {}

    void reset();                    // fresh run (the arm handover)
    // Every tick, BEFORE the command reaches the policy: does the link's command
    // (after pinFrozen) ask for motion? dt_ms since the previous tick.
    void tick(bool moving_cmd, uint32_t dt_ms);
    // The detector tripped this tick. true: recovering (the stand override is on,
    // write the policy's targets as usual); false: hold and disarm, as before.
    bool onTrip();

    bool standOverride() const { return active_; }
    // 0..1: how far the command fed to the policy has been taken toward the stand.
    float overrideFrac() const;
    // false during the grace window: the caller skips Detector::update.
    bool judging() const { return !active_ || since_ms_ >= C_.grace_ms; }
    // true once, on the first tick after the grace window: the caller resets the
    // Detector so it judges the stand from fresh filters.
    bool graceJustEnded();
    // A walk/step command now or within recent_ms, with no recovery engaged: the
    // flail guard judges rate against Limits::rate_walk_rad_s.
    bool walkingRecent() const {
        return !active_ && quiet_ms_ != UINT32_MAX && quiet_ms_ <= C_.recent_ms;
    }
    const Config& config() const { return C_; }

  private:
    Config C_;
    uint32_t quiet_ms_ = UINT32_MAX;   // since the last moving command (MAX: none this run)
    bool active_ = false;
    uint32_t since_ms_ = 0;
    bool grace_reported_ = false;
};

}  // namespace flailguard
