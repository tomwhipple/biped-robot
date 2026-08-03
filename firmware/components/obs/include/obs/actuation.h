// Action <-> joint angle <-> servo ticks.
//
// Three coordinate systems meet here and mixing them up is the classic
// deploy-day failure:
//   action   [-1, 1] per joint, what the policy emits
//   angle    radians in the SIM's sign convention, what the MJCF joints use
//   steps    0..4095 encoder ticks in the SERVO's own frame
//
// The action -> angle map must match walker_env._action_to_ctrl() for the
// deployed run's action_map (loco_v5t: "full" -- piecewise-linear, action 0 ==
// the standing pose, full travel on each side). The angle -> steps map is
// per-servo calibration (docs/wiring.md bring-up step 4: "Set Middle Position"
// at the CAD-neutral pose), which is why it takes a Calibration rather than
// being baked in.
#pragma once
#include <stdint.h>

#include "obs/obs_spec.h"

namespace obs {

// Per-servo calibration, restored from NVS at boot. Defaults are what a
// freshly "Set Middle Position"-ed, correctly-oriented servo gives you.
struct Calibration {
    int32_t zero_steps[kNumJoints];   // encoder value at joint angle 0
    int8_t dir[kNumJoints];           // +1 or -1: servo CW vs. joint positive
    Calibration() {
        for (int i = 0; i < kNumJoints; ++i) {
            zero_steps[i] = 2048;     // scsbus::kCenterSteps
            dir[i] = 1;
        }
    }
};

// walker_env._action_to_ctrl(), action_map == "full":
//   span = action >= 0 ? (hi - default) : (default - lo)
//   angle = default + span * clamp(action, -1, 1)
void actionToAngles(const float* action, float* angle_rad);

// Inverse, used to seed prev_action after a torque re-engage so the policy is
// not handed a discontinuity.
void anglesToAction(const float* angle_rad, float* action);

// Angle <-> encoder ticks. 4096 ticks per revolution.
int32_t angleToSteps(int joint, float rad, const Calibration& cal);

// angleToSteps without the policy-range clamp: converts any angle through
// the calibration, bounded only by the encoder's 0..4095. For callers that
// clamp against a DIFFERENT range first (the CLI bench clamps to the
// measured mechanical envelope, main/mech_envelope.h). The act path must
// keep using angleToSteps -- the kJointLo/Hi clamp there is SIL-pinned.
int32_t angleToStepsRaw(int joint, float rad, const Calibration& cal);
float stepsToAngle(int joint, int32_t steps, const Calibration& cal);

// The servo reports speed in steps/s (register 58). Whether the policy is fed
// this or a finite difference of positions is an open sys-ID question
// (docs/firmware-design.md section 8) -- the sim uses true qvel and ToddlerBot
// finite-differences. Both paths are provided; the control task picks one.
float stepsPerSecToRadPerSec(int joint, int32_t steps_per_s,
                             const Calibration& cal);

// -- command shaping ---------------------------------------------------
// The policy emits a new position target every 20 ms; written raw (speed 0 ==
// unlimited) the servo slams at max speed to each one, arrives mid-tick and
// stops dead -- a 50 Hz velocity square wave on every joint. The shaper turns
// that staircase into a C2 trajectory: three cascaded first-order lags, all
// poles at pole_hz. Each stage adds one derivative of smoothness, so the
// trajectory the servos are asked to follow has continuous velocity AND
// acceleration (finite jerk). Each stage is the exact zero-order-hold
// discretization, so it is stable for any pole/dt; all poles are real, so a
// step never overshoots; and the output never leaves the hull of its inputs,
// so a shaped target cannot exceed a joint's range if the raw ones don't.
//
// Default pole: 10 Hz, picked by closed-loop referee sweep (2026-08-02,
// loco_v8foot, 8 seeds/scenario through the SIL stack): stand_10s 8/8 and
// line_1m 8/8 at every pole tried; goal_home python 7/8 vs SIL raw 5/8,
// 16 Hz 7/8, 12 Hz 6/8, 10 Hz 6/8, 8 Hz 4/8. So 10 Hz is the strongest
// smoothing that stays inside the referee's parity budget -- 8 Hz measurably
// costs the policy task performance. Runtime-tunable (CLI `shape`); 0 means
// shaping off (the legacy raw path).
inline constexpr float kShaperPoleHz = 10.0f;

class CommandShaper {
  public:
    explicit CommandShaper(float pole_hz = kShaperPoleHz) {
        setPole(pole_hz, kControlDt);
    }
    // Recompute the per-stage gain: k = 1 - exp(-2*pi*pole_hz*dt).
    void setPole(float pole_hz, float dt);
    float poleHz() const { return pole_hz_; }
    // Seed every stage. Callers seed from MEASURED joint angles on a torque
    // (re)engage so the first shaped target can never be a jump.
    void reset(const float* angle_rad);
    // Forget the seed; the next update() self-seeds from its target. Called
    // when torque is released or the bus is handed away, so a re-engage
    // reseeds from wherever the joints actually are by then.
    void invalidate() { primed_ = false; }
    bool primed() const { return primed_; }
    // One control tick: raw targets in, shaped targets out (in-place safe).
    void update(const float* target_rad, float* out_rad);

  private:
    float y1_[kNumJoints] = {};
    float y2_[kNumJoints] = {};
    float y3_[kNumJoints] = {};
    float k_ = 1.0f;
    float pole_hz_ = 0.0f;
    bool primed_ = false;
};

// The goal speed (register 46, steps/s) written alongside a shaped target:
// enough to close the gap from the MEASURED position to the target within one
// tick, with headroom. While tracking, that is the trajectory's own speed --
// no max-speed slam -- and under a disturbance the error term grows it back
// toward the old slam behaviour, so smoothing never costs stiffness. The
// floor keeps the field nonzero (0 means UNLIMITED to the servo); the ceiling
// is the servo's own kMaxGoalSpeed (static_asserted where scsbus is visible).
inline constexpr float kGoalSpeedHeadroom = 1.25f;
inline constexpr int32_t kGoalSpeedFloor = 50;     // steps/s
inline constexpr int32_t kGoalSpeedMax = 3400;     // == scsbus::kMaxGoalSpeed
uint16_t goalSpeedSteps(int32_t goal_steps, int32_t present_steps);

// First-order low-pass finite-difference velocity, for the other branch of
// that question. alpha = 1.0 is a raw difference.
class VelocityEstimator {
  public:
    explicit VelocityEstimator(float alpha = 0.5f) : alpha_(alpha) {}
    void reset(const float* angle_rad);
    void update(const float* angle_rad, float dt, float* out_rad_s);

  private:
    float prev_[kNumJoints] = {};
    float vel_[kNumJoints] = {};
    float alpha_;
    bool primed_ = false;
};

}  // namespace obs
