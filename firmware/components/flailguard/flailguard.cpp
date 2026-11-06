#include "flailguard/flailguard.h"

#include <math.h>
#include <string.h>

namespace flailguard {

namespace {

constexpr float kRadToDeg = 57.29578f;

bool finite3(const float* v) {
    return v != nullptr && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

bool endsWith(const char* s, const char* tail) {
    const size_t n = strlen(s), m = strlen(tail);
    return n >= m && strcmp(s + n - m, tail) == 0;
}

// One EMA step with the exact zero-order-hold gain for this dt.
float ema(float y, float x, float tau_s, uint32_t dt_ms) {
    if (!(tau_s > 0.0f)) return x;
    const float a = 1.0f - expf(-(static_cast<float>(dt_ms) / 1000.0f) / tau_s);
    return y + a * (x - y);
}

}  // namespace

float poseLimitDeg(const char* joint_name) {
    // By joint KIND (the suffix after L_/R_), so both legs and both policy
    // specs (10- and 12-joint) share one row each. Order matters only where
    // one suffix ends another ("ankle_roll" before "ankle").
    struct Row {
        const char* kind;
        float deg;
    };
    static constexpr Row kTable[] = {
        {"hip_yaw", 15.0f},  {"hip_roll", 15.0f}, {"hip_pitch", 15.0f},
        {"knee", 15.0f},     {"ankle_roll", 15.0f}, {"ankle", 15.0f},
    };
    constexpr float kDefault = 15.0f;
    if (joint_name == nullptr) return kDefault;
    for (const Row& r : kTable) {
        if (endsWith(joint_name, r.kind)) return r.deg;
    }
    return kDefault;
}

const char* tripName(Trip t) {
    switch (t) {
        case Trip::kNone: return "none";
        case Trip::kPose: return "pose";
        case Trip::kRate: return "rate";
        case Trip::kTilt: return "tilt";
        case Trip::kChatter: return "chatter";
    }
    return "?";
}

void Detector::reset() {
    trip_ = Trip::kNone;
    trip_value_ = 0.0f;
    trip_joint_ = -1;
    ticks_ = 0;
    pose_run_ms_ = tilt_run_ms_ = 0;
    pose_cand_ = tilt_cand_ = false;
    rate_ema_ = chatter_ema_ = 0.0f;
    have_prev_ = false;
    peak_rate_ = peak_chatter_ = peak_pose_deg_ = peak_tilt_deg_ = 0.0f;
}

Trip Detector::update(const Input& in, uint32_t dt_ms) {
    if (tripped()) return Trip::kNone;
    ++ticks_;
    const int n = in.n < 0 ? 0 : (in.n > kMaxJoints ? kMaxJoints : in.n);

    // -- tilt + rate: the torso, from the IMU ------------------------------
    // No fresh attitude, no decision: a gap breaks the tilt run (like the
    // fall guard) and leaves the rate filter where it was.
    Trip hit = Trip::kNone;
    float hit_value = 0.0f;
    int hit_joint = -1;
    if (in.imu_valid && finite3(in.up) && finite3(in.gyro)) {
        float z = in.up[2];
        if (z > 1.0f) z = 1.0f;
        if (z < -1.0f) z = -1.0f;
        const float tilt = acosf(z) * kRadToDeg;
        if (tilt > peak_tilt_deg_) peak_tilt_deg_ = tilt;
        if (tilt > L_.tilt_deg) {
            if (tilt_cand_) {
                tilt_run_ms_ += dt_ms;
            } else {
                tilt_cand_ = true;
                tilt_run_ms_ = 0;
            }
            if ((L_.criteria & kCritTilt) && tilt_run_ms_ >= L_.tilt_hold_ms) {
                hit = Trip::kTilt;
                hit_value = tilt;
            }
        } else {
            tilt_cand_ = false;
            tilt_run_ms_ = 0;
        }
        const float w = sqrtf(in.gyro[0] * in.gyro[0] + in.gyro[1] * in.gyro[1] +
                              in.gyro[2] * in.gyro[2]);
        rate_ema_ = ema(rate_ema_, w, L_.rate_tau_s, dt_ms);
        if (rate_ema_ > peak_rate_) peak_rate_ = rate_ema_;
        if (hit == Trip::kNone && (L_.criteria & kCritRate) && rate_ema_ > L_.rate_rad_s) {
            hit = Trip::kRate;
            hit_value = rate_ema_;
        }
    } else {
        tilt_cand_ = false;
        tilt_run_ms_ = 0;
    }

    // -- chatter: the raw policy targets' rate -----------------------------
    if (in.raw != nullptr && n > 0 && dt_ms > 0) {
        bool ok = true;
        for (int j = 0; j < n; ++j) ok = ok && isfinite(in.raw[j]);
        if (!ok) {
            have_prev_ = false;            // a hole restarts the difference
        } else {
            if (have_prev_) {
                float sum = 0.0f;
                for (int j = 0; j < n; ++j) sum += fabsf(in.raw[j] - prev_raw_[j]);
                const float rate = sum / static_cast<float>(n) /
                                   (static_cast<float>(dt_ms) / 1000.0f);
                chatter_ema_ = ema(chatter_ema_, rate, L_.chatter_tau_s, dt_ms);
                if (chatter_ema_ > peak_chatter_) peak_chatter_ = chatter_ema_;
                if (hit == Trip::kNone && (L_.criteria & kCritChatter) &&
                    chatter_ema_ > L_.chatter_rad_s) {
                    hit = Trip::kChatter;
                    hit_value = chatter_ema_;
                }
            }
            for (int j = 0; j < n; ++j) prev_raw_[j] = in.raw[j];
            have_prev_ = true;
        }
    }

    // -- pose: the written targets vs the stand, at the stand command ------
    if (in.stand_cmd && in.pose_armed && in.target != nullptr && in.stand != nullptr &&
        in.pose_limit != nullptr && n > 0) {
        bool over = false;
        float worst_ratio = 0.0f, worst_deg = 0.0f;
        int worst_j = -1;
        for (int j = 0; j < n; ++j) {
            const float dev = fabsf(in.target[j] - in.stand[j]);
            if (!isfinite(dev)) continue;
            const float deg = dev * kRadToDeg;
            if (deg > peak_pose_deg_) peak_pose_deg_ = deg;
            const float lim = in.pose_limit[j];
            if (!(lim > 0.0f)) continue;
            const float ratio = dev / lim;
            if (ratio > 1.0f) over = true;
            if (ratio > worst_ratio) {
                worst_ratio = ratio;
                worst_deg = deg;
                worst_j = j;
            }
        }
        if (over) {
            if (pose_cand_) {
                pose_run_ms_ += dt_ms;
            } else {
                pose_cand_ = true;
                pose_run_ms_ = 0;
            }
            if (hit == Trip::kNone && (L_.criteria & kCritPose) &&
                pose_run_ms_ >= L_.pose_hold_ms) {
                hit = Trip::kPose;
                hit_value = worst_deg;
                hit_joint = worst_j;
            }
        } else {
            pose_cand_ = false;
            pose_run_ms_ = 0;
        }
    } else {
        pose_cand_ = false;
        pose_run_ms_ = 0;
    }

    if (hit != Trip::kNone) {
        trip_ = hit;
        trip_value_ = hit_value;
        trip_joint_ = hit_joint;
    }
    return hit;
}

// -- WalkRecovery ------------------------------------------------------------

void WalkRecovery::reset() {
    quiet_ms_ = UINT32_MAX;
    active_ = false;
    since_ms_ = 0;
    grace_reported_ = false;
}

void WalkRecovery::tick(bool moving_cmd, uint32_t dt_ms) {
    if (moving_cmd) {
        quiet_ms_ = 0;
    } else if (quiet_ms_ != UINT32_MAX) {
        quiet_ms_ = (UINT32_MAX - quiet_ms_ > dt_ms) ? quiet_ms_ + dt_ms : UINT32_MAX - 1;
    }
    if (active_ && since_ms_ < UINT32_MAX - dt_ms) since_ms_ += dt_ms;
}

bool WalkRecovery::onTrip() {
    if (active_) return false;                       // once per run: a second trip holds
    if (quiet_ms_ == UINT32_MAX || quiet_ms_ > C_.recent_ms) return false;   // a stand trip holds
    active_ = true;
    since_ms_ = 0;
    grace_reported_ = false;
    return true;
}

float WalkRecovery::overrideFrac() const {
    if (!active_) return 0.0f;
    if (C_.ramp_ms == 0) return 1.0f;
    const float f = static_cast<float>(since_ms_) / static_cast<float>(C_.ramp_ms);
    return f < 1.0f ? f : 1.0f;
}

bool WalkRecovery::graceJustEnded() {
    if (!active_ || grace_reported_ || since_ms_ < C_.grace_ms) return false;
    grace_reported_ = true;
    return true;
}

}  // namespace flailguard
