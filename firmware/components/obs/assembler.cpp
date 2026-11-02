#include "obs/assembler.h"

#include <math.h>
#include <string.h>

namespace obs {

void assembleFrame(const Inputs& in, float* out) {
    memcpy(out + kOffQ, in.q, sizeof(float) * kNumJoints);
    memcpy(out + kOffDq, in.dq, sizeof(float) * kNumJoints);
    memcpy(out + kOffUp, in.up, sizeof(float) * 3);
    // Torso linear velocity: no sensor exists, and the deployed policy was
    // trained with imu_obs=1, i.e. having seen exactly zeros here.
    out[kOffLinVel + 0] = 0.0f;
    out[kOffLinVel + 1] = 0.0f;
    out[kOffLinVel + 2] = 0.0f;
    memcpy(out + kOffGyro, in.gyro, sizeof(float) * 3);
    memcpy(out + kOffPrevAction, in.prev_action, sizeof(float) * kActDim);
    out[kOffHeight] = 0.0f;          // likewise: no height sensor
    out[kOffPhase + 0] = sinf(in.phase);
    out[kOffPhase + 1] = cosf(in.phase);
    memcpy(out + kOffCmd, in.cmd, sizeof(float) * kNumCmd);
    static_assert(kOffCmd + kNumCmd == kFrameDim, "frame layout mismatch");
}

void History::fill(const float* frame) {
    for (int i = 0; i < kHistLen - 1; ++i) {
        memcpy(prev_[i], frame, sizeof(float) * kFrameDim);
    }
}

void History::build(const float* frame, float* obs) const {
    memcpy(obs, frame, sizeof(float) * kFrameDim);
    for (int i = 0; i < kHistLen - 1; ++i) {
        memcpy(obs + (i + 1) * kFrameDim, prev_[i], sizeof(float) * kFrameDim);
    }
}

void History::push(const float* frame) {
    for (int i = kHistLen - 2; i > 0; --i) {
        memcpy(prev_[i], prev_[i - 1], sizeof(float) * kFrameDim);
    }
    if (kHistLen > 1) memcpy(prev_[0], frame, sizeof(float) * kFrameDim);
}

void GaitClock::advance(float dt) {
    // walker_env: (phase + 2*pi*dt*freq + pi) % (2*pi) - pi
    const float two_pi = 6.283185307179586f;
    float p = phase_ + two_pi * dt * freq_ + 3.141592653589793f;
    p = fmodf(p, two_pi);
    if (p < 0.0f) p += two_pi;      // C fmod keeps the sign; Python % does not
    phase_ = p - 3.141592653589793f;
}

float GaitClock::speedScale(float v_planar) {
    // env_mjx.speed_clock_scale: clip(sqrt(|v| / REF), lo, hi); walker_env
    // mirrors it with max(v, 0) under the sqrt. v_planar is >= 0 here.
    if (!kSpeedClock) return 1.0f;
    const float v = v_planar < 0.0f ? 0.0f : v_planar;
    float s = sqrtf(v / kSpeedClockRef);
    if (s < kSpeedClockLo) s = kSpeedClockLo;
    if (s > kSpeedClockHi) s = kSpeedClockHi;
    return s;
}

void GaitClock::advanceSpeedClock(float dt, float v_planar) {
    advance(dt * speedScale(v_planar));   // freq * scale == same step at dt * scale
}

namespace {
constexpr float kTwoPi = 6.283185307179586f;
constexpr float kPi = 3.141592653589793f;
// The same wrap advance() applies: into [-pi, pi).
float wrapPi(float p) {
    p = fmodf(p + kPi, kTwoPi);
    if (p < 0.0f) p += kTwoPi;
    return p - kPi;
}
}  // namespace

bool GaitClock::coastTo(float dt, float target) {
    const float t = wrapPi(target);
    const float step = kTwoPi * dt * freq_;
    float to_go = fmodf(t - phase_, kTwoPi);      // forward distance, [0, 2pi)
    if (to_go < 0.0f) to_go += kTwoPi;
    // <= one step to go -- or a hair PAST the target (float rounding of the
    // wrap), which must not send the clock round a whole cycle
    if (to_go <= step || to_go >= kTwoPi - 1e-4f) {
        phase_ = t;
        return true;
    }
    advance(dt);
    return false;
}

bool plainStand(const float* cmd) {
    const float vy = kNumCmd > 1 ? cmd[1] : 0.0f;
    const float lift = kNumCmd > 4 ? cmd[4] : 0.0f;
    return !(fabsf(cmd[0]) > 0.05f || fabsf(vy) > 0.05f || fabsf(cmd[2]) > 0.05f ||
             fabsf(lift) >= 0.5f);
}

void tickGaitClock(GaitClock& clk, const float* cmd, StandClock mode,
                   float stand_phase) {
    const float vy = kNumCmd > 1 ? cmd[1] : 0.0f;
    const float v_planar = sqrtf(cmd[0] * cmd[0] + vy * vy);
    if (!plainStand(cmd)) {
        clk.advanceSpeedClock(kControlDt, v_planar);
        return;
    }
    if (kClockStandFreeze) return;                // trained with the clock held
    switch (mode) {
        case StandClock::kFreeze:
            return;
        case StandClock::kCoast:
            (void)clk.coastTo(kControlDt * GaitClock::speedScale(v_planar), stand_phase);
            return;
        case StandClock::kRun:
        default:
            clk.advanceSpeedClock(kControlDt, v_planar);
            return;
    }
}

}  // namespace obs
