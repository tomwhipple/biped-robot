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
    // the CLI converts degrees with /57.29578f: allow float rounding at both ends (3.0 and 20.0 must pass)
    if (!(c.abort_rad >= 0.05236f - 1e-5f && c.abort_rad <= 0.34907f + 1e-5f)) return "abort must be 3..20 deg";
    if (!(c.abort_decay_s >= 0.05f && c.abort_decay_s <= 2.0f)) return "abort decay must be 0.05..2 s";
    if (!(c.abort_land_s >= 0.2f && c.abort_land_s <= 5.0f)) return "abort land must be 0.2..5 s";
    if (!(c.arm_start_rad >= 0.0f && c.arm_start_rad <= 0.34907f)) return "arm start must be 0..20 deg";
    if (!(c.i_lim_rad >= 0.0f && c.i_lim_rad <= 0.26180f + 1e-5f)) return "I limit must be 0..15 deg";
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

bool continues(State s, char active_side, char new_side, const bool* was_on,
               const bool* now_on, int n) {
    if ((s != State::kActive && s != State::kArmed) || active_side != new_side) return false;
    for (int b = 0; b < n; ++b) {
        if (now_on[b] && !was_on[b]) return false;
    }
    return true;
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

void RollBalance::arm(float roll_ref_rad) {
    engage(roll_ref_rad);
    state_ = State::kArmed;
}

bool RollBalance::start(float roll_ref_rad) {
    if (state_ != State::kArmed) return false;
    engage(roll_ref_rad);
    return true;
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
    if (state_ != State::kActive && state_ != State::kArmed) return false;
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
    if (state_ != State::kActive && state_ != State::kArmed) return state_;
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
    if (state_ == State::kArmed) {
        // watching only: u stays 0, I stays 0 -- unless the lean is real (see ARMED)
        if (!(cfg_.arm_start_rad > 0.0f && fabsf(err) > cfg_.arm_start_rad)) return state_;
        engage(roll_rad);                      // reference here; the PI acts from the next tick
        rate_ = roll_rate_rad_s;
        return state_;
    }
    const float unsat = cfg_.kp * err + cfg_.ki * i_ + cfg_.kd * roll_rate_rad_s;
    float u = unsat;
    if (u > cfg_.clamp_rad) u = cfg_.clamp_rad;
    if (u < -cfg_.clamp_rad) u = -cfg_.clamp_rad;
    const bool pushing_into_clamp = (u != unsat) && ((err > 0.0f) == (u > 0.0f));
    if (!pushing_into_clamp) i_ += err * dt_s;
    if (cfg_.i_lim_rad > 0.0f && cfg_.ki > 0.0f) {
        // the integral's share of u stays within +-i_lim_rad (it limits the next tick's u)
        const float lim = cfg_.i_lim_rad / cfg_.ki;
        if (i_ > lim) i_ = lim;
        if (i_ < -lim) i_ = -lim;
    }
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
        case State::kArmed: return "ARMED (PI starts when the move ends; abort live)";
    }
    return "?";
}

const char* abortPhase(const RollBalance& b) {
    if (!b.aborted()) return "";
    if (b.landed()) return "landed";
    return b.abortSince() < b.config().abort_decay_s ? "fading" : "landing";
}

}  // namespace balance
