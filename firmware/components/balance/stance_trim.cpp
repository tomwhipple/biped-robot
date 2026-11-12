#include "balance/stance_trim.h"

#include <math.h>

namespace balance {

const char* validateTrim(const TrimConfig& c) {
    if (!(c.kp >= 0.0f && c.kp <= 2.0f)) return "kp must be 0..2";
    if (!(c.ki >= 0.0f && c.ki <= 5.0f)) return "ki must be 0..5 /s";
    // the CLI converts degrees with /57.29578f: allow float rounding at the ends
    if (!(c.lim_rad > 0.0f && c.lim_rad <= 0.34907f + 1e-5f)) return "lim must be 0..20 deg";
    if (!(c.t_lim_rad >= 0.0f && c.t_lim_rad <= c.lim_rad + 1e-5f)) return "tlim must be 0..lim";
    if (!(c.fade_s >= 0.05f && c.fade_s <= 5.0f)) return "fade must be 0.05..5 s";
    if (!(c.abort_rad >= 0.05236f - 1e-5f && c.abort_rad <= 0.52360f + 1e-5f)) return "abort must be 3..30 deg";
    if (!(c.learn_rad >= 0.00873f - 1e-5f && c.learn_rad <= 0.52360f + 1e-5f)) return "learn must be 0.5..30 deg";
    if (!(c.keep_rad >= c.learn_rad - 1e-5f && c.keep_rad <= 0.52360f + 1e-5f)) return "keep must be learn..30 deg";
    return nullptr;
}

static float clipf(float x, float lim) {
    if (x > lim) return lim;
    if (x < -lim) return -lim;
    return x;
}

void StanceTrim::begin(char side, float roll_ref_rad) {
    side_ = (side == 'R') ? 'R' : 'L';
    ref_ = roll_ref_rad;
    err_ = 0.0f;
    hike_ = 0.0f;
    hike_frozen_ = 0.0f;
    since_ = 0.0f;
    err_abort_ = 0.0f;
    state_ = TrimState::kClosed;
    t_begin_ = t_[side_ == 'R' ? 1 : 0];
    peak_ = 0.0f;
    clamped_ = false;
}

// A chain is over (release, abort, stop): commit its T, or put it back if the chain was not clean.
void StanceTrim::endChain(bool aborting) {
    if (side_ != 'L' && side_ != 'R') return;
    reverted_ = aborting || clamped_ || peak_ > cfg_.keep_rad;
    if (reverted_) t_[side_ == 'R' ? 1 : 0] = t_begin_;
}

float StanceTrim::fadeLeft() const {
    if (state_ != TrimState::kFade && !aborted()) return 1.0f;
    const float k = 1.0f - since_ / cfg_.fade_s;
    return k > 0.0f ? k : 0.0f;
}

TrimState StanceTrim::update(float roll_rad, float dt_s, float ramp) {
    if (!(isfinite(dt_s) && dt_s > 0.0f)) dt_s = 0.0f;
    switch (state_) {
        case TrimState::kOff:
            return state_;
        case TrimState::kHold:
            if (isfinite(roll_rad)) err_ = roll_rad - ref_;
            return state_;
        case TrimState::kFade:
        case TrimState::kAbortTip:
        case TrimState::kAbortImu: {
            since_ += dt_s;
            if (isfinite(roll_rad)) err_ = roll_rad - ref_;
            const float k = fadeLeft();
            hike_ = hike_frozen_ * k;
            if (k <= 1e-4f) {     // float time sums land a hair short of fade_s
                hike_ = 0.0f;
                state_ = TrimState::kOff;
                side_ = 0;
            }
            return state_;
        }
        case TrimState::kClosed:
            break;
    }
    if (!isfinite(roll_rad)) {
        imuFault();
        return state_;
    }
    const float e = roll_rad - ref_;
    err_ = e;
    if (fabsf(e) > peak_) peak_ = fabsf(e);
    if (fabsf(e) > cfg_.abort_rad) {
        err_abort_ = e;
        hike_frozen_ = hike_;
        since_ = 0.0f;
        endChain(true);
        state_ = TrimState::kAbortTip;
        return state_;
    }
    float r = isfinite(ramp) ? ramp : 1.0f;
    if (r < 0.0f) r = 0.0f;
    if (r > 1.0f) r = 1.0f;
    const int k = side_ == 'R' ? 1 : 0;
    const float s = sign();
    const float unsat = r * t_[k] + cfg_.kp * s * e;
    if (fabsf(unsat) >= cfg_.lim_rad) clamped_ = true;
    else if (fabsf(e) <= cfg_.learn_rad) t_[k] = clipf(t_[k] + cfg_.ki * s * e * dt_s, cfg_.t_lim_rad);
    hike_ = clipf(unsat, cfg_.lim_rad);
    return state_;
}

void StanceTrim::hold() {
    if (state_ == TrimState::kClosed) state_ = TrimState::kHold;
}

void StanceTrim::resume() {
    if (state_ == TrimState::kHold) state_ = TrimState::kClosed;
}

void StanceTrim::release() {
    if (state_ == TrimState::kClosed || state_ == TrimState::kHold) {
        hike_frozen_ = hike_;
        since_ = 0.0f;
        endChain(false);
        state_ = TrimState::kFade;
    }
}

void StanceTrim::imuFault() {
    if (state_ != TrimState::kClosed && state_ != TrimState::kHold) return;
    err_abort_ = err_;
    hike_frozen_ = hike_;
    since_ = 0.0f;
    endChain(true);
    state_ = TrimState::kAbortImu;
}

void StanceTrim::stop() {
    if (state_ == TrimState::kClosed || state_ == TrimState::kHold) endChain(false);
    state_ = TrimState::kOff;
    side_ = 0;
    hike_ = hike_frozen_ = 0.0f;
    err_ = 0.0f;
    since_ = 0.0f;
}

bool StanceTrim::setLearned(char side, float t_rad) {
    if ((side != 'L' && side != 'R') || !isfinite(t_rad)) return false;
    t_[side == 'R' ? 1 : 0] = clipf(t_rad, cfg_.t_lim_rad);
    return true;
}

bool TrimDriver::step(const SupportIn& in, float roll_rad, float dt_s, StepReport* done) {
    bool ended = false;
    // the residual's own fade
    if (res_side_ != 0) {
        res_since_s_ += (isfinite(dt_s) && dt_s > 0.0f) ? dt_s : 0.0f;
        const float k = res_left_s_ > 0.0f ? 1.0f - res_since_s_ / res_left_s_ : 0.0f;
        if (k <= 1e-4f) {
            res_side_ = 0;
            res_hip_ = 0.0f;
        } else {
            res_hip_ = res_hip0_ * k;
        }
    }
    if (cur_ != 0 && in.stance != cur_) {
        // the single support is over (double support, or the other foot): fade, report
        trim_.release();
        rep_.aborted = rep_.aborted || trim_.aborted();
        rep_.reverted = trim_.lastReverted();
        if (done != nullptr) *done = rep_;
        ended = true;
        cur_ = 0;
    }
    if (in.stance != 0 && cur_ == 0) {
        if (!isfinite(roll_rad)) {
            trim_.update(roll_rad, dt_s, 1.0f);        // a fading chain goes on; no new chain without attitude
            return ended;
        }
        if (trim_.engaged()) {
            // the last correction is still fading: it finishes on its own hip (a residual still running
            // from before is replaced -- two handovers inside one fade time)
            res_side_ = trim_.side();
            res_hip0_ = res_hip_ = trim_.hipJoint();
            res_left_s_ = trim_.config().fade_s * trim_.fadeLeft();
            res_since_s_ = 0.0f;
            trim_.release();
        }
        trim_.begin(in.stance, roll_rad);
        cur_ = in.stance;
        rep_ = StepReport();
        rep_.stance = in.stance;
        mid_taken_ = false;
    }
    if (cur_ != 0) {
        if (in.lowering) trim_.hold();
        trim_.update(roll_rad, dt_s, in.progress);
        const float s = cur_ == 'R' ? -1.0f : 1.0f;
        const float lean = s * trim_.err();              // + = toward the swing foot
        if (lean > rep_.lean_in) rep_.lean_in = lean;
        if (-lean > rep_.lean_out) rep_.lean_out = -lean;
        if (!mid_taken_ && in.progress >= 1.0f) {
            mid_taken_ = true;
            rep_.e_mid = trim_.err();
            rep_.t_mid = trim_.learned(cur_);
            rep_.hike_mid = trim_.hike();
        }
        if (trim_.aborted()) rep_.aborted = true;
    } else {
        trim_.update(roll_rad, dt_s, 1.0f);              // the fade of the last chain
    }
    return ended;
}

const char* trimStateName(TrimState s) {
    switch (s) {
        case TrimState::kOff: return "off";
        case TrimState::kClosed: return "CLOSED";
        case TrimState::kHold: return "HOLD (frozen through the lowering)";
        case TrimState::kFade: return "FADE (foot down: the correction goes to 0)";
        case TrimState::kAbortTip: return "ABORTED (roll error past the abort limit): frozen, fading";
        case TrimState::kAbortImu: return "ABORTED (IMU read failed): frozen, fading";
    }
    return "?";
}

}  // namespace balance
