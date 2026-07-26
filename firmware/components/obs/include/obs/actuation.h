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
float stepsToAngle(int joint, int32_t steps, const Calibration& cal);

// The servo reports speed in steps/s (register 58). Whether the policy is fed
// this or a finite difference of positions is an open sys-ID question
// (docs/firmware-design.md section 8) -- the sim uses true qvel and ToddlerBot
// finite-differences. Both paths are provided; the control task picks one.
float stepsPerSecToRadPerSec(int joint, int32_t steps_per_s,
                             const Calibration& cal);

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
