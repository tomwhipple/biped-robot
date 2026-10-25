#include "balance/roll_pi.h"

#include <math.h>

namespace balance {

const char* validate(const Config& c) {
    if (!(c.kp >= 0.0f && c.kp <= 10.0f)) return "kp must be 0..10";
    if (!(c.ki >= 0.0f && c.ki <= 10.0f)) return "ki must be 0..10";
    if (!(c.kd >= 0.0f && c.kd <= 1.0f)) return "kd must be 0..1";
    if (!(fabsf(c.ka) <= 3.0f)) return "ankle mix must be within +-3";
    if (!(fabsf(c.kh) <= 3.0f)) return "hip mix must be within +-3";
    if (!(c.clamp_rad > 0.0f && c.clamp_rad <= 0.26180f + 1e-5f)) return "clamp must be 0..15 deg";
    if (!(c.abort_rad >= 0.05236f && c.abort_rad <= 0.34907f)) return "abort must be 3..20 deg";
    return nullptr;
}

bool rollFromUp(const float up[3], float* roll_rad) {
    if (!isfinite(up[0]) || !isfinite(up[1]) || !isfinite(up[2])) return false;
    if (up[2] < 0.5f) return false;
    *roll_rad = atan2f(-up[1], up[2]);
    return true;
}

void RollBalance::engage(float roll_ref_rad) {
    ref_ = roll_ref_rad;
    err_ = 0.0f;
    i_ = 0.0f;
    u_ = 0.0f;
    rate_ = 0.0f;
    state_ = State::kActive;
}

void RollBalance::disengage() {
    state_ = State::kOff;
    err_ = 0.0f;
    i_ = 0.0f;
    u_ = 0.0f;
    rate_ = 0.0f;
}

State RollBalance::update(float roll_rad, float dt_s, float roll_rate_rad_s) {
    if (state_ != State::kActive) return state_;
    if (!isfinite(roll_rate_rad_s)) {
        state_ = State::kFrozenImu;
        return state_;
    }
    rate_ = roll_rate_rad_s;
    const float err = roll_rad - ref_;
    if (!isfinite(err) || fabsf(err) > cfg_.abort_rad) {
        state_ = State::kFrozenAbort;
        err_ = err;
        return state_;
    }
    err_ = err;
    const float unsat = cfg_.kp * err + cfg_.ki * i_ + cfg_.kd * roll_rate_rad_s;
    float u = unsat;
    if (u > cfg_.clamp_rad) u = cfg_.clamp_rad;
    if (u < -cfg_.clamp_rad) u = -cfg_.clamp_rad;
    const bool pushing_into_clamp = (u != unsat) && ((err > 0.0f) == (u > 0.0f));
    if (!pushing_into_clamp) i_ += err * dt_s;
    u_ = u;
    return state_;
}

void RollBalance::imuFault() {
    if (state_ == State::kActive) state_ = State::kFrozenImu;
}

const char* stateName(State s) {
    switch (s) {
        case State::kOff: return "off";
        case State::kActive: return "ACTIVE";
        case State::kFrozenAbort: return "FROZEN (roll error past the abort limit)";
        case State::kFrozenImu: return "FROZEN (IMU read failed)";
    }
    return "?";
}

}  // namespace balance
