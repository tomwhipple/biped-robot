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
    if (!(c.abort_decay_s >= 0.05f && c.abort_decay_s <= 2.0f)) return "abort decay must be 0.05..2 s";
    if (!(c.abort_land_s >= 0.2f && c.abort_land_s <= 5.0f)) return "abort land must be 0.2..5 s";
    return nullptr;
}

static float minJerk(float t) {
    if (!(t > 0.0f)) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    const float t2 = t * t, t3 = t2 * t;
    const float v = 10.0f * t3 - 15.0f * t3 * t + 6.0f * t3 * t2;
    return v > 1.0f ? 1.0f : v;      // float rounding can overshoot a hair near t = 1
}

void abortProfile(const Config& c, float since_s, float* u_scale, float* land_frac) {
    *u_scale = 1.0f - minJerk(since_s / c.abort_decay_s);
    *land_frac = minJerk((since_s - c.abort_decay_s) / c.abort_land_s);
}

int32_t landTarget(int32_t from, int32_t start, float land_frac) {
    if (!(land_frac > 0.0f)) return from;
    if (land_frac >= 1.0f) return start;
    return from + static_cast<int32_t>(lroundf(land_frac * static_cast<float>(start - from)));
}

float RollBalance::landFraction() const {
    if (!aborted()) return 0.0f;
    float us, lf;
    abortProfile(cfg_, since_, &us, &lf);
    return lf;
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
    since_ = u_abort_ = err_abort_ = 0.0f;
    state_ = State::kActive;
}

void RollBalance::disengage() {
    state_ = State::kOff;
    err_ = 0.0f;
    i_ = 0.0f;
    u_ = 0.0f;
    rate_ = 0.0f;
    since_ = u_abort_ = err_abort_ = 0.0f;
}

bool RollBalance::abort(State why) {
    if (state_ != State::kActive) return false;
    state_ = (why == State::kAbortImu) ? State::kAbortImu : State::kAbortTip;
    u_abort_ = u_;
    err_abort_ = err_;
    since_ = 0.0f;
    return true;
}

void RollBalance::abortTick(float dt_s) {
    if (!aborted()) return;
    if (isfinite(dt_s) && dt_s > 0.0f) since_ += dt_s;
    float us, lf;
    abortProfile(cfg_, since_, &us, &lf);
    u_ = u_abort_ * us;
}

State RollBalance::update(float roll_rad, float dt_s, float roll_rate_rad_s) {
    if (aborted()) {
        abortTick(dt_s);                       // no decisions: the sequence is open loop
        return state_;
    }
    if (state_ != State::kActive) return state_;
    if (!isfinite(roll_rate_rad_s)) {
        abort(State::kAbortImu);
        return state_;
    }
    rate_ = roll_rate_rad_s;
    const float err = roll_rad - ref_;
    if (!isfinite(err) || fabsf(err) > cfg_.abort_rad) {
        if (isfinite(err)) err_ = err;         // else the last finite error is the one worth reporting
        abort(State::kAbortTip);
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
    abort(State::kAbortImu);
}

const char* stateName(State s) {
    switch (s) {
        case State::kOff: return "off";
        case State::kActive: return "ACTIVE";
        case State::kAbortTip: return "ABORTED (roll error past the abort limit)";
        case State::kAbortImu: return "ABORTED (IMU read failed)";
    }
    return "?";
}

const char* abortPhase(const RollBalance& b) {
    if (!b.aborted()) return "";
    if (b.landed()) return "landed";
    return b.abortSince() < b.config().abort_decay_s ? "fading" : "landing";
}

}  // namespace balance
