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

int32_t angleToSteps(int joint, float rad, const Calibration& cal) {
    // Clamp to the joint's MJCF range before it reaches the servo: the policy
    // is clamped there too, and a target outside the mechanical range is how
    // you stall a horn against a printed part.
    if (rad < kJointLo[joint]) rad = kJointLo[joint];
    if (rad > kJointHi[joint]) rad = kJointHi[joint];
    const float ticks = rad / kRadPerStep * static_cast<float>(cal.dir[joint]);
    int32_t steps = cal.zero_steps[joint] + static_cast<int32_t>(lrintf(ticks));
    // Second guard, in the steps domain. The clamp above bounds the ANGLE, but
    // a wrong zero_steps (miscalibration, or NVS restored from a different
    // build of the robot) still shifts the result off the encoder's 0..4095
    // range -- and a target the servo cannot reach is a horn parked against a
    // hard stop drawing stall current. Clamping is the safe failure: the joint
    // sits at its extreme instead of cooking.
    if (steps < 0) steps = 0;
    if (steps > kMaxSteps) steps = kMaxSteps;
    return steps;
}

float stepsToAngle(int joint, int32_t steps, const Calibration& cal) {
    const int32_t d = steps - cal.zero_steps[joint];
    return static_cast<float>(d) * kRadPerStep *
           static_cast<float>(cal.dir[joint]);
}

float stepsPerSecToRadPerSec(int joint, int32_t steps_per_s,
                             const Calibration& cal) {
    return static_cast<float>(steps_per_s) * kRadPerStep *
           static_cast<float>(cal.dir[joint]);
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
