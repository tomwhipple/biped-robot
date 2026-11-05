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

float limitBrakeSpeed(float dist, float amax, float dt) {
    if (!(dist > 0.0f) || !(amax > 0.0f) || !(dt > 0.0f)) return 0.0f;
    const float u = amax * dt * dt;
    const float m = dist / u;
    float k = floorf(0.5f * (sqrtf(1.0f + 8.0f * m) - 1.0f));
    if (k < 0.0f) k = 0.0f;
    const float rho = (m - 0.5f * k * (k + 1.0f)) / (k + 1.0f);
    return amax * dt * (k + rho);
}

int limitTaps(float amax, float jmax, float dt) {
    if (!(jmax > 0.0f) || !(amax > 0.0f) || !(dt > 0.0f)) return 1;
    // 1e-3: an exact ratio (2 amax == n jmax dt) is n taps, not n + 1, in
    // float as in double (sim/fw_limit.py uses the same slack)
    const float n = ceilf(2.0f * amax / (jmax * dt) - 1e-3f);
    if (n < 1.0f) return 1;
    if (n > static_cast<float>(kLimitMaxTaps)) return kLimitMaxTaps;
    return static_cast<int>(n);
}

void MotionLimiter::configure(float vmax, float amax, float jmax, float dt) {
    vmax_ = vmax > 0.0f ? vmax : 0.0f;
    amax_ = amax > 0.0f ? amax : 0.0f;
    jmax_ = jmax > 0.0f ? jmax : 0.0f;
    dt_ = dt > 0.0f ? dt : kControlDt;
    dv_ = amax_ * dt_;
    taps_ = limitTaps(amax_, jmax_, dt_);
}

void MotionLimiter::reset(const float* angle_rad) {
    for (int i = 0; i < kNumJoints; ++i) {
        p_[i] = angle_rad[i];
        v_[i] = 0.0f;
        for (int t = 0; t < kLimitMaxTaps; ++t) {
            hp_[t][i] = angle_rad[i];
            hv_[t][i] = 0.0f;
        }
    }
    head_ = 0;
    primed_ = true;
}

void MotionLimiter::update(const float* target_rad, float* out_rad,
                           float* out_vel) {
    if (!primed_) reset(target_rad);   // defensive; callers seed from measured
    head_ = (head_ + 1) % kLimitMaxTaps;
    for (int i = 0; i < kNumJoints; ++i) {
        const float e = target_rad[i] - p_[i];
        float vb = limitBrakeSpeed(fabsf(e), amax_, dt_);
        if (vb > vmax_) vb = vmax_;
        const float vd = e > 0.0f ? vb : (e < 0.0f ? -vb : 0.0f);
        float dv = vd - v_[i];
        if (dv > dv_) dv = dv_;
        if (dv < -dv_) dv = -dv_;
        v_[i] += dv;
        p_[i] += v_[i] * dt_;
        hp_[head_][i] = p_[i];
        hv_[head_][i] = v_[i];
    }
    if (taps_ <= 1) {
        for (int i = 0; i < kNumJoints; ++i) {
            out_rad[i] = p_[i];
            out_vel[i] = v_[i];
        }
        return;
    }
    // boxcar over the newest taps_ ticks, newest first (the order
    // sim/fw_limit.py sums in)
    const float inv = 1.0f / static_cast<float>(taps_);
    for (int i = 0; i < kNumJoints; ++i) {
        float sp = 0.0f, sv = 0.0f;
        for (int t = 0; t < taps_; ++t) {
            const int r = (head_ - t + kLimitMaxTaps) % kLimitMaxTaps;
            sp += hp_[r][i];
            sv += hv_[r][i];
        }
        out_rad[i] = sp * inv;
        out_vel[i] = sv * inv;
    }
}

uint16_t limitGoalSpeedSteps(float vel_rad_s) {
    float sps = fabsf(vel_rad_s) / kRadPerStep * kGoalSpeedHeadroom;
    if (!(sps >= static_cast<float>(kGoalSpeedFloor))) {   // NaN -> floor
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
