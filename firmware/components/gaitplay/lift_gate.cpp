#include "gaitplay/lift_gate.h"

#include <math.h>

namespace gaitplay {

const char* validateGate(const LiftGateConfig& c) {
    if (!(c.lvl_rad >= 0.00873f && c.lvl_rad <= 0.34907f)) return "lvl must be 0.5-20 deg";
    if (!(c.rate_rad_s >= 0.01745f && c.rate_rad_s <= 3.4907f)) return "rate must be 1-200 deg/s";
    if (c.dwell_ms > 2000) return "dwell must be 0-2000 ms";
    if (c.timeout_ms < c.dwell_ms + 100 || c.timeout_ms > 30000) return "timeout must be dwell + 100 ms .. 30000 ms";
    if (!(c.ref_alpha >= 0.0f && c.ref_alpha <= 1.0f)) return "ref_alpha must be 0-1";
    return nullptr;
}

void LiftGate::begin() {
    st_ = GateStats();
    start_ = 0.0f;
    ref_[0] = ref_[1] = 0.0f;
    have_start_ = false;
    have_ref_[0] = have_ref_[1] = false;
    holding_ = quiet_ = false;
    held_ms_ = hold_t0_ = quiet_t0_ = last_hold_ms_ = last_gate_t_ = t_gate_ = 0;
    side_ = 0;
    seen_gate_ = false;
}

float LiftGate::ref(char side) const {
    const int k = side == 'R' ? 1 : 0;
    if (have_ref_[k]) return ref_[k];
    return have_start_ ? start_ : NAN;
}

bool LiftGate::good(float roll, float rate, char side, float* dev) const {
    const float r = ref(side);
    const float d = roll - r;
    *dev = d;
    if (!isfinite(roll) || !isfinite(rate) || !isfinite(r)) return false;
    return fabsf(d) <= cfg_.lvl_rad && fabsf(rate) <= cfg_.rate_rad_s;
}

void LiftGate::accept(char side, float roll) {
    const int k = side == 'R' ? 1 : 0;
    if (!have_ref_[k]) {
        ref_[k] = roll;
        have_ref_[k] = true;
    } else {
        ref_[k] += cfg_.ref_alpha * (roll - ref_[k]);
    }
}

GateOut LiftGate::tick(const GateIn& in) {
    GateOut out;
    if (!have_start_ && isfinite(in.roll_rad)) {
        start_ = in.roll_rad;
        have_start_ = true;
    }
    // the track time with the holds so far; while holding, the clock is parked at the gate frame
    uint32_t t = in.t_raw_ms > held_ms_ ? in.t_raw_ms - held_ms_ : 0;
    out.t_ms = t;
    out.rate_rad_s = in.rate_rad_s;
    if (holding_) {
        out.side = side_;
        float dev = 0.0f;
        const bool ok = good(in.roll_rad, in.rate_rad_s, side_, &dev) && cfg_.enabled;
        out.dev_rad = dev;
        if (ok) {
            if (!quiet_) {
                quiet_ = true;
                quiet_t0_ = in.wall_ms;
            }
        } else {
            quiet_ = false;
        }
        const uint32_t dur = in.wall_ms - hold_t0_;
        held_ms_ = in.t_raw_ms > t_gate_ ? in.t_raw_ms - t_gate_ : 0;
        out.t_ms = t_gate_;
        if (!cfg_.enabled || (quiet_ && in.wall_ms - quiet_t0_ >= cfg_.dwell_ms)) {
            holding_ = false;
            out.ev = GateEvt::kHoldEnd;
            out.hold_ms = dur;
            st_.held_ms += dur;
            last_hold_ms_ = dur;
            if (cfg_.enabled) accept(side_, in.roll_rad);
        } else if (dur >= cfg_.timeout_ms) {
            holding_ = false;
            out.ev = GateEvt::kTimeout;
            out.hold_ms = dur;
            st_.held_ms += dur;
            ++st_.timeouts;
            last_hold_ms_ = dur;
        } else {
            out.holding = true;
        }
        return out;
    }
    if (!cfg_.enabled || !in.have_gate) return out;
    if (seen_gate_ && in.t_gate_ms <= last_gate_t_) return out;       // already judged
    if (in.t_gate_ms > t) return out;                                   // not reached yet
    seen_gate_ = true;
    last_gate_t_ = in.t_gate_ms;
    out.side = in.side;
    ++st_.gates;
    float dev = 0.0f;
    const bool ok = good(in.roll_rad, in.rate_rad_s, in.side, &dev);
    out.dev_rad = dev;
    if (ok) {
        accept(in.side, in.roll_rad);
        last_hold_ms_ = 0;
        out.ev = GateEvt::kPass;
        return out;
    }
    holding_ = true;
    quiet_ = false;
    side_ = in.side;
    t_gate_ = in.t_gate_ms;
    hold_t0_ = in.wall_ms;
    held_ms_ = in.t_raw_ms > t_gate_ ? in.t_raw_ms - t_gate_ : 0;
    ++st_.holds;
    out.ev = GateEvt::kHoldStart;
    out.holding = true;
    out.t_ms = t_gate_;
    return out;
}

}  // namespace gaitplay
