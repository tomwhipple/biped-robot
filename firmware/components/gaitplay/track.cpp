#include "gaitplay/track.h"

#include <math.h>
#include <string.h>

namespace gaitplay {

namespace {

constexpr float kTwoPi = 6.28318531f;
constexpr float kPi = 3.14159265f;

int hexv(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool hexn(const char* s, int n, uint32_t* out) {
    uint32_t v = 0;
    for (int i = 0; i < n; ++i) {
        const int d = hexv(s[i]);
        if (d < 0) return false;
        v = (v << 4) | static_cast<uint32_t>(d);
    }
    *out = v;
    return true;
}

float wrap2pi(float x) {
    x = fmodf(x, kTwoPi);
    return x < 0.0f ? x + kTwoPi : x;
}

float minJerk(float u) {
    if (!(u > 0.0f)) return 0.0f;
    if (u >= 1.0f) return 1.0f;
    return u * u * u * (10.0f - 15.0f * u + 6.0f * u * u);
}

// Catmull-Rom through p1 -> p2 at u in [0, 1], tangents from the neighbours
float catmull(float p0, float p1, float p2, float p3, float u) {
    const float u2 = u * u, u3 = u2 * u;
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * u + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * u2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * u3);
}

}  // namespace

bool parseFrame(const char* hex, uint32_t prev_t, Frame* out) {
    if (hex == nullptr || strlen(hex) != 42) return false;
    uint32_t dt = 0, ph = 0, fl = 0;
    if (!hexn(hex, 3, &dt) || !hexn(hex + 3, 2, &ph) || !hexn(hex + 5, 1, &fl)) return false;
    Frame f;
    for (int j = 0; j < kJoints; ++j) {
        uint32_t q = 0;
        if (!hexn(hex + 6 + 3 * j, 3, &q)) return false;
        f.q[j] = static_cast<int16_t>(q);
    }
    f.t_ms = prev_t + dt;
    f.phi = static_cast<float>(ph) * kTwoPi / 256.0f;
    f.flags = static_cast<uint8_t>(fl);
    *out = f;
    return true;
}

void formatFrame(const Frame& f, uint32_t prev_t, char* out43) {
    static const char* const kHex = "0123456789abcdef";
    uint32_t dt = f.t_ms - prev_t;
    if (dt > 0xFFF) dt = 0xFFF;
    const uint32_t ph = static_cast<uint32_t>(lroundf(wrap2pi(f.phi) * 256.0f / kTwoPi)) & 0xFF;
    uint32_t vals[3 + kJoints];
    vals[0] = dt;
    vals[1] = ph;
    vals[2] = f.flags & 0xF;
    int k = 0;
    auto put = [&](uint32_t v, int n) {
        for (int i = n - 1; i >= 0; --i) out43[k++] = kHex[(v >> (4 * i)) & 0xF];
    };
    put(vals[0], 3);
    put(vals[1], 2);
    put(vals[2], 1);
    for (int j = 0; j < kJoints; ++j) {
        int32_t q = f.q[j];
        if (q < 0) q = 0;
        if (q > 4095) q = 4095;
        put(static_cast<uint32_t>(q), 3);
    }
    out43[k] = 0;
}

void Track::clear() {
    head_ = n_ = 0;
    last_t_ = 0;
    state_ = State::kIdle;
    stop_req_ = false;
    stop_at_ = -1;
    rest_n_ = rest_head_ = 0;
    have_rest_ = false;
    steps_ = 0;
    have_prev_ = have_prev2_ = false;
}

bool Track::add(const Frame& f) {
    if (state_ == State::kResting || state_ == State::kDone || stop_req_) return false;
    if (f.flags & kRest) {
        rests_[(rest_head_ + rest_n_) % kRestCap] = f;
        if (rest_n_ < kRestCap) ++rest_n_;
        else rest_head_ = (rest_head_ + 1) % kRestCap;
        return true;
    }
    if (n_ >= kCap) return false;
    if ((n_ > 0 || state_ != State::kIdle) && f.t_ms <= lastTime()) return false;
    buf_[idx(n_)] = f;
    ++n_;
    last_t_ = f.t_ms;
    return true;
}

const Frame* Track::restFor(uint32_t t_step_end) const {
    const Frame* best = nullptr;
    for (int k = 0; k < rest_n_; ++k) {
        const Frame& r = rests_[(rest_head_ + k) % kRestCap];
        if (r.t_ms <= t_step_end && (best == nullptr || r.t_ms >= best->t_ms)) best = &r;
    }
    return best;
}

bool Track::start() {
    if (state_ != State::kIdle || n_ < 2) return false;
    state_ = State::kPlaying;
    t_start_ = at(0).t_ms;
    for (int j = 0; j < kJoints; ++j) last_q_[j] = at(0).q[j];
    last_phi_ = at(0).phi;
    return true;
}

void Track::stop() {
    if (state_ == State::kPlaying) {
        stop_req_ = true;
        state_ = State::kStopping;
    }
}

void Track::beginRest(const int32_t* from, uint32_t t_ms) {
    for (int j = 0; j < kJoints; ++j) rest_from_[j] = from[j];
    rest_t0_ = t_ms;
    n_ = 0;
    state_ = have_rest_ ? State::kResting : State::kDone;
}

State Track::sample(uint32_t t_ms, Sample* s) {
    s->step_end = false;
    s->lift = false;
    if (state_ == State::kIdle) {
        for (int j = 0; j < kJoints; ++j) s->q[j] = last_q_[j];
        s->phi = last_phi_;
        return state_;
    }
    if (state_ == State::kResting || state_ == State::kDone) {
        const float u = state_ == State::kDone ? 1.0f
                                               : minJerk(static_cast<float>(t_ms - rest_t0_) /
                                                         static_cast<float>(rest_ms_));
        for (int j = 0; j < kJoints; ++j) {
            const float to = have_rest_ ? static_cast<float>(rest_.q[j]) : static_cast<float>(rest_from_[j]);
            s->q[j] = static_cast<int32_t>(lroundf(static_cast<float>(rest_from_[j]) +
                                                  u * (to - static_cast<float>(rest_from_[j]))));
            last_q_[j] = s->q[j];
        }
        s->phi = last_phi_;
        if (state_ == State::kResting && u >= 1.0f) state_ = State::kDone;
        return state_;
    }
    // playing / stopping: pass the frames the clock has reached; a step end passed is counted, and ends a
    // stop (or a queue that has run dry there)
    bool holding = false;
    while (n_ >= 1 && at(0).t_ms <= t_ms) {
        const Frame a = at(0);
        if (a.flags & kStepEnd) {
            ++steps_;
            s->step_end = true;
            if (stop_req_ || n_ == 1) {
                int32_t from[kJoints];
                for (int j = 0; j < kJoints; ++j) from[j] = a.q[j];
                const Frame* r = restFor(a.t_ms);
                have_rest_ = r != nullptr;
                if (have_rest_) rest_ = *r;
                last_phi_ = a.phi;
                beginRest(from, t_ms);
                const bool se = s->step_end;
                const State st = sample(t_ms, s);
                s->step_end = se;
                return st;
            }
        }
        if (n_ == 1) {                  // the last frame, mid-step: hold it (the host appends whole steps)
            holding = true;
            break;
        }
        prev2_ = prev_;
        have_prev2_ = have_prev_;
        prev_ = a;
        have_prev_ = true;
        head_ = (head_ + 1) % kCap;
        --n_;
    }
    const Frame& b = at(0);
    if (holding || !have_prev_) {
        for (int j = 0; j < kJoints; ++j) {
            s->q[j] = b.q[j];
            last_q_[j] = s->q[j];
        }
        s->phi = b.phi;
        s->lift = false;
        last_phi_ = s->phi;
        return state_;
    }
    const Frame& p1 = prev_;
    const Frame& p0 = have_prev2_ ? prev2_ : prev_;
    const Frame& p3 = n_ >= 2 ? at(1) : b;
    const float span = static_cast<float>(b.t_ms - p1.t_ms);
    // a clock that is not monotone is not rewound: u stays within the bracket
    float u = (span > 0.0f && t_ms > p1.t_ms) ? static_cast<float>(t_ms - p1.t_ms) / span : 0.0f;
    if (u > 1.0f) u = 1.0f;
    for (int j = 0; j < kJoints; ++j) {
        s->q[j] = static_cast<int32_t>(lroundf(catmull(static_cast<float>(p0.q[j]), static_cast<float>(p1.q[j]),
                                                       static_cast<float>(b.q[j]), static_cast<float>(p3.q[j]), u)));
        last_q_[j] = s->q[j];
    }
    float d = b.phi - p1.phi;
    if (d < -kPi) d += kTwoPi;
    if (d > kPi) d -= kTwoPi;
    s->phi = wrap2pi(p1.phi + u * d);
    s->lift = (p1.flags & kLift) != 0;
    last_phi_ = s->phi;
    return state_;
}

bool Track::gateAhead(uint32_t t_ms, uint32_t* t_gate, char* stance) const {
    for (int k = 0; k < n_; ++k) {
        const Frame& f = at(k);
        if (f.t_ms > t_ms) break;
        if (f.flags & kGate) {
            *t_gate = f.t_ms;
            *stance = liftStance(f.phi);
            return true;
        }
    }
    return false;
}

State Track::abortToRest(uint32_t t_ms) {
    if (state_ != State::kPlaying && state_ != State::kStopping) return state_;
    const Frame* r = restFor(t_ms);
    have_rest_ = r != nullptr;
    if (have_rest_) rest_ = *r;
    int32_t from[kJoints];
    for (int j = 0; j < kJoints; ++j) from[j] = last_q_[j];
    beginRest(from, t_ms);
    return state_;
}

char liftStance(float phi) {
    if (!isfinite(phi)) return 0;
    return wrap2pi(phi) < kPi ? 'R' : 'L';
}

Support supportFromPhase(float phi, bool lift) {
    Support out{0, 0.0f, false};
    if (!lift || !isfinite(phi)) return out;
    const float p = wrap2pi(phi);
    const bool left_swings = p < kPi;
    const float u = (left_swings ? p : p - kPi) / kPi;        // 0..1 within the half cycle
    if (u < 0.25f || u > 0.75f) return out;
    const float w = (u - 0.25f) / 0.5f;                        // 0..1 over the single support
    out.stance = left_swings ? 'R' : 'L';
    out.progress = w >= 0.35f ? 1.0f : w / 0.35f;
    out.lowering = w >= 0.65f;
    return out;
}

bool clockWalking(const float* cmd, int ncmd) {
    const auto at = [cmd, ncmd](int i) { return i < ncmd ? fabsf(cmd[i]) : 0.0f; };
    return at(0) > 0.05f || at(1) > 0.05f || at(2) > 0.05f || at(4) > 0.5f;
}

}  // namespace gaitplay
