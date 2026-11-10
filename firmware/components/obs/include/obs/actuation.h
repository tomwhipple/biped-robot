// Action <-> joint angle <-> servo ticks.
//
// Three coordinate systems meet here and mixing them up is the classic
// deploy-day failure:
//   action   [-1, 1] per joint, what the policy emits
//   angle    radians in the SIM's sign convention, what the MJCF joints use
//   steps    0..4095 encoder ticks in the SERVO's own frame
//
// The action -> angle map must match walker_env._action_to_ctrl() for the
// deployed run's action_map ("full", the only one implemented -- piecewise-linear, action 0 ==
// the standing pose, full travel on each side). The angle -> steps map is
// per-servo calibration (docs/wiring.md bring-up step 4: "Set Middle Position"
// at the CAD-neutral pose), which is why it takes a Calibration rather than
// being baked in.
#pragma once
#include <stdint.h>

#include "obs/bus_map.h"
#include "obs/obs_spec.h"

namespace obs {

// Per-servo calibration, restored from NVS at boot, indexed by BUS joint
// (obs/bus_map.h, bus index order), so it covers every servo the
// robot carries, not just the ones the policy drives. Defaults are what a
// freshly "Set Middle Position"-ed, correctly-oriented servo gives you.
//
// `fitted` says which bus joints this robot actually has: the legs-only
// robot carries its twelve leg joints and not the neck or arms. The control loop and the bench
// commands read and write fitted servos only -- a SYNC READ that waits on a
// servo which is not there costs its whole timeout every tick. `cal` sets it
// (a joint that answers `cal zero` is fitted; `cal fit` sets it by hand).
struct Calibration {
    int32_t zero_steps[kNumBusJoints];   // encoder value at joint angle 0
    int8_t dir[kNumBusJoints];           // +1 or -1: servo CW vs. joint positive
    uint8_t fitted[kNumBusJoints];       // 1 = this servo is on the robot
    Calibration() {
        for (int b = 0; b < kNumBusJoints; ++b) {
            zero_steps[b] = 2048;     // scsbus::kCenterSteps
            dir[b] = 1;
            fitted[b] = 1;            // expect every servo until told otherwise
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

// Angle <-> encoder ticks, 4096 ticks per revolution, for POLICY joint j
// (obs_spec order): the calibration row is policyToBus(j)'s. angleToSteps
// clamps to the policy range kJointLo/Hi first -- that clamp is SIL-pinned.
int32_t angleToSteps(int joint, float rad, const Calibration& cal);
float stepsToAngle(int joint, int32_t steps, const Calibration& cal);

// The same maps for BUS joint b, without any joint-range clamp: bounded only
// by the encoder's 0..4095. For callers that clamp against a DIFFERENT range
// first (the CLI bench clamps to the measured mechanical envelope,
// main/mech_envelope.h) or that report where a joint is.
int32_t busAngleToStepsRaw(int bus, float rad, const Calibration& cal);
float busStepsToAngle(int bus, int32_t steps, const Calibration& cal);

// The servo reports speed in steps/s (register 58). Whether the policy is fed
// this or a finite difference of positions is an open sys-ID question
// (docs/firmware-design.md section 8) -- the sim uses true qvel and ToddlerBot
// finite-differences. Both paths are provided; the control task picks one.
// Policy joint j, like angleToSteps.
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

// -- motion limiter ------------------------------------------------------
// The shaper smooths the policy's staircase, but it does not bound it: a big
// target step still comes out as a fast shaped move, and the fast STS3215s
// (~240-275 deg/s at full duty) then stop dead at the goal. On 2026-11-03 the
// first walk step of walk18_shp10sq kicked the swing leg that way and the
// reaction spun the robot on its stance foot. The limiter sits between the
// shaped targets and the goal ticks and bounds what goes to the servo, per
// joint, every tick:
//   |velocity| <= vmax, |acceleration| <= amax, and BRAKING LOOK-AHEAD: the
//   velocity toward the target never exceeds the speed from which amax
//   braking stops exactly at it (limitBrakeSpeed, the discrete-time stopping
//   curve). So a joint arrives at a target that holds still with zero
//   velocity and never passes it; a target that moves back toward the joint
//   faster than amax allows is the only way to overshoot, by at most the
//   v^2 / (2 amax) the bound costs.
//   jmax (optional, 0 = off): an S-curve by a boxcar (moving average) of
//   `taps` = ceil(2 amax / (jmax dt)) ticks on the limited trajectory. The
//   output is a convex mix of limited positions, so it keeps every bound
//   above (vmax, amax, no overshoot) and its jerk is <= 2 amax / (taps dt)
//   <= jmax; it delays the trajectory by (taps - 1) / 2 ticks.
// State: position and velocity per joint (+ the boxcar history), seeded
// from MEASURED q with zero velocity on a torque (re)engage, exactly where
// the shaper seeds. Float, deterministic, no transcendental beyond sqrtf:
// sim/fw_limit.py and env_mjx run the same law, pinned by
// tests/test_fw_limit.py against this code through the SIL library.
// Off at boot (vmax 0); `limit` sets it live.
inline constexpr int kLimitMaxTaps = 16;
// Proposed settings (`limit on`; sim sweep 2026-11-03,
// experiments/policy/motion-limiter): vmax below the servo's ~240 deg/s
// loaded top speed so the limiter, not the servo's saturation, sets the
// profile.
inline constexpr float kLimitVmaxDeg = 200.0f;      // deg/s
inline constexpr float kLimitAmaxDeg = 2000.0f;     // deg/s^2
inline constexpr float kLimitJmaxDeg = 0.0f;        // deg/s^3, 0 = no jerk limit
inline constexpr float kDegToRad = 0.017453292519943295f;

// The largest speed toward a target `dist` rad away (dist >= 0) from which a
// joint can still stop exactly on it while decelerating by at most amax*dt
// per tick, with position integrated as p += v*dt. With u = amax*dt^2 and
// m = dist/u: k = floor((sqrt(1 + 8m) - 1) / 2) whole braking ticks, and
// v = amax*dt * (k + (m - k(k+1)/2) / (k+1)). Piecewise linear between the
// triangular numbers, continuous, -> sqrt(2 amax dist) for large dist, and
// dist/dt (land in one tick) for dist <= u.
float limitBrakeSpeed(float dist, float amax, float dt);
// Boxcar length for a jerk limit: 1 when jmax <= 0, else
// ceil(2 amax / (jmax dt)), capped at kLimitMaxTaps.
int limitTaps(float amax, float jmax, float dt);

class MotionLimiter {
  public:
    // rad/s, rad/s^2, rad/s^3. vmax <= 0 or amax <= 0 turns it off. Live:
    // state carries over a change (a taps change re-reads the history,
    // which always holds kLimitMaxTaps ticks).
    void configure(float vmax, float amax, float jmax, float dt);
    bool enabled() const { return vmax_ > 0.0f && amax_ > 0.0f; }
    int taps() const { return taps_; }
    // Seed every joint at angle_rad, at rest.
    void reset(const float* angle_rad);
    void invalidate() { primed_ = false; }
    bool primed() const { return primed_; }
    // One control tick: shaped targets in, limited targets and their
    // velocity (rad/s) out. In-place safe (target == out_rad).
    void update(const float* target_rad, float* out_rad, float* out_vel);

  private:
    float p_[kNumJoints] = {};
    float v_[kNumJoints] = {};
    float hp_[kLimitMaxTaps][kNumJoints] = {};   // boxcar history, ring
    float hv_[kLimitMaxTaps][kNumJoints] = {};
    int head_ = 0;
    float vmax_ = 0.0f, amax_ = 0.0f, jmax_ = 0.0f;
    float dt_ = kControlDt, dv_ = 0.0f;
    int taps_ = 1;
    bool primed_ = false;
};

// The goal speed written with a LIMITED target: the limiter's own speed,
// x kGoalSpeedHeadroom, floor..max -- so the servo slews along the limited
// profile between the 50 Hz goals (reaching each goal just before the next)
// instead of chasing it at whatever speed the measured gap asks for.
uint16_t limitGoalSpeedSteps(float vel_rad_s);

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
