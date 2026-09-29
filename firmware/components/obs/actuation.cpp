#include "obs/actuation.h"

#include <math.h>

namespace obs {
namespace {
constexpr float kTwoPi = 6.283185307179586f;
constexpr float kStepsPerRev = 4096.0f;
constexpr float kRadPerStep = kTwoPi / kStepsPerRev;
constexpr int32_t kMaxSteps = 4095;   // ST3215 encoder full scale

float clamp1(float x) { return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x); }
}  // namespace

void actionToAngles(const float* action, float* angle_rad) {
    for (int i = 0; i < kNumJoints; ++i) {
        const float a = clamp1(action[i]);
        const float span = (a >= 0.0f) ? (kJointHi[i] - kJointDefault[i])
                                       : (kJointDefault[i] - kJointLo[i]);
        angle_rad[i] = kJointDefault[i] + span * a;
    }
}

void anglesToAction(const float* angle_rad, float* action) {
    for (int i = 0; i < kNumJoints; ++i) {
        const float d = angle_rad[i] - kJointDefault[i];
        const float span = (d >= 0.0f) ? (kJointHi[i] - kJointDefault[i])
                                       : (kJointDefault[i] - kJointLo[i]);
        action[i] = (span > 1e-9f) ? clamp1(d / span) : 0.0f;
    }
}

int32_t busAngleToStepsRaw(int bus, float rad, const Calibration& cal) {
    const float ticks = rad / kRadPerStep * static_cast<float>(cal.dir[bus]);
    int32_t steps = cal.zero_steps[bus] + static_cast<int32_t>(lrintf(ticks));
    // Steps-domain guard. The caller bounds the ANGLE against whichever range
    // applies to it, but a wrong zero_steps (miscalibration, or NVS restored
    // from a different build of the robot) still shifts the result off the
    // encoder's 0..4095 range -- and a target the servo cannot reach is a
    // horn parked against a hard stop drawing stall current. Clamping is the
    // safe failure: the joint sits at its extreme instead of cooking.
    if (steps < 0) steps = 0;
    if (steps > kMaxSteps) steps = kMaxSteps;
    return steps;
}

int32_t angleToSteps(int joint, float rad, const Calibration& cal) {
    // Clamp to the joint's MJCF range before it reaches the servo: the policy
    // is clamped there too, and a target outside the mechanical range is how
    // you stall a horn against a printed part.
    if (rad < kJointLo[joint]) rad = kJointLo[joint];
    if (rad > kJointHi[joint]) rad = kJointHi[joint];
    return busAngleToStepsRaw(policyToBus(joint), rad, cal);
}

float busStepsToAngle(int bus, int32_t steps, const Calibration& cal) {
    const int32_t d = steps - cal.zero_steps[bus];
    return static_cast<float>(d) * kRadPerStep *
           static_cast<float>(cal.dir[bus]);
}

float stepsToAngle(int joint, int32_t steps, const Calibration& cal) {
    return busStepsToAngle(policyToBus(joint), steps, cal);
}

float stepsPerSecToRadPerSec(int joint, int32_t steps_per_s,
                             const Calibration& cal) {
    return static_cast<float>(steps_per_s) * kRadPerStep *
           static_cast<float>(cal.dir[policyToBus(joint)]);
}

void CommandShaper::setPole(float pole_hz, float dt) {
    pole_hz_ = pole_hz;
    if (pole_hz <= 0.0f || dt <= 0.0f) {
        k_ = 1.0f;                   // degenerate: pass-through
        return;
    }
    k_ = 1.0f - expf(-kTwoPi * pole_hz * dt);
    if (k_ > 1.0f) k_ = 1.0f;
    if (k_ < 1e-6f) k_ = 1e-6f;
}

void CommandShaper::reset(const float* angle_rad) {
    for (int i = 0; i < kNumJoints; ++i) {
        y1_[i] = angle_rad[i];
        y2_[i] = angle_rad[i];
        y3_[i] = angle_rad[i];
    }
    primed_ = true;
}

void CommandShaper::update(const float* target_rad, float* out_rad) {
    if (!primed_) reset(target_rad);   // defensive; callers seed from measured
    for (int i = 0; i < kNumJoints; ++i) {
        // Cascade order matters: each stage sees this tick's upstream output.
        y1_[i] += k_ * (target_rad[i] - y1_[i]);
        y2_[i] += k_ * (y1_[i] - y2_[i]);
        y3_[i] += k_ * (y2_[i] - y3_[i]);
        out_rad[i] = y3_[i];
    }
}

uint16_t goalSpeedSteps(int32_t goal_steps, int32_t present_steps) {
    const int32_t err =
        goal_steps >= present_steps ? goal_steps - present_steps
                                    : present_steps - goal_steps;
    float sps = static_cast<float>(err) / kControlDt * kGoalSpeedHeadroom;
    if (sps < static_cast<float>(kGoalSpeedFloor)) {
        sps = static_cast<float>(kGoalSpeedFloor);
    }
    if (sps > static_cast<float>(kGoalSpeedMax)) {
        sps = static_cast<float>(kGoalSpeedMax);
    }
    return static_cast<uint16_t>(lrintf(sps));
}

void VelocityEstimator::reset(const float* angle_rad) {
    for (int i = 0; i < kNumJoints; ++i) {
        prev_[i] = angle_rad[i];
        vel_[i] = 0.0f;
    }
    primed_ = true;
}

void VelocityEstimator::update(const float* angle_rad, float dt,
                               float* out_rad_s) {
    if (!primed_ || dt <= 0.0f) {
        reset(angle_rad);
        for (int i = 0; i < kNumJoints; ++i) out_rad_s[i] = 0.0f;
        return;
    }
    for (int i = 0; i < kNumJoints; ++i) {
        const float raw = (angle_rad[i] - prev_[i]) / dt;
        vel_[i] += alpha_ * (raw - vel_[i]);
        prev_[i] = angle_rad[i];
        out_rad_s[i] = vel_[i];
    }
}

}  // namespace obs
