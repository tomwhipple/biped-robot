#include "fallguard/fallguard.h"

#include <math.h>

namespace fallguard {

float tiltDeg(const float up[3]) {
    float z = up[2];
    if (!(z <= 1.0f)) z = 1.0f;      // also catches NaN: report 0, callers gate on validity anyway
    if (z < -1.0f) z = -1.0f;
    return acosf(z) * 57.29578f;
}

const char* motionName(Motion m) { return m == Motion::kPolicy ? "policy" : "bench"; }

const char* eventName(Event e) {
    switch (e) {
        case Event::kNone: return "none";
        case Event::kFell: return "fell";
        case Event::kPickedUp: return "picked up";
        case Event::kCleared: return "cleared";
    }
    return "?";
}

static bool finite3(const float v[3]) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }

Event Detector::update(Motion m, bool valid, const float up[3], const float gyro[3], uint32_t now_ms) {
    if (!valid || up == nullptr || gyro == nullptr || !finite3(up) || !finite3(gyro)) {
        fall_cand_ = up_cand_ = false;
        return Event::kNone;
    }
    if (!fallen_) {
        const Limits L = limits(m);
        if (!(up[2] < L.fall_up_z)) {
            fall_cand_ = false;
            return Event::kNone;
        }
        if (!fall_cand_) {
            fall_cand_ = true;
            fall_since_ = now_ms;
        }
        if (static_cast<uint32_t>(now_ms - fall_since_) < L.debounce_ms) return Event::kNone;
        fallen_ = true;
        fell_in_ = m;
        fall_tilt_deg_ = tiltDeg(up);
        ++falls_;
        fall_cand_ = up_cand_ = false;
        return Event::kFell;
    }
    const float w = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    if (!(up[2] > kPickupUpZ && w < kPickupGyroRadS)) {
        up_cand_ = false;
        return Event::kNone;
    }
    if (!up_cand_) {
        up_cand_ = true;
        up_since_ = now_ms;
    }
    if (static_cast<uint32_t>(now_ms - up_since_) < kPickupHoldMs) return Event::kNone;
    up_cand_ = false;          // one event per hold; the caller clears (or not: autohome off)
    up_since_ = now_ms;
    return Event::kPickedUp;
}

void Detector::latch(Motion m, float tilt_deg) {
    if (fallen_) return;
    fallen_ = true;
    fell_in_ = m;
    fall_tilt_deg_ = tilt_deg;
    ++falls_;
    fall_cand_ = up_cand_ = false;
}

void Detector::clear() {
    fallen_ = false;
    fall_cand_ = up_cand_ = false;
}

uint32_t Detector::candidateMs(uint32_t now_ms) const {
    if (!fallen_) return fall_cand_ ? static_cast<uint32_t>(now_ms - fall_since_) : 0u;
    return up_cand_ ? static_cast<uint32_t>(now_ms - up_since_) : 0u;
}

}  // namespace fallguard
