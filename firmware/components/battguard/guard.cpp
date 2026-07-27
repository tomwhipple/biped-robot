#include "battguard/guard.h"

namespace battguard {

void Guard::update(uint8_t dv, float now_ms) {
    if (dv != 0) last_dv_ = dv;

    // -- latched: nothing un-does this ------------------------------------
    // The pack voltage RISES the moment torque drops -- an unloaded flat 3S
    // rebounds well above 10.5 V within seconds. So a guard that cleared on
    // recovery would sit the robot down, watch the rail recover, stand it back
    // up, and repeat until the cells are ruined. Latching is the whole point.
    if (level_ == Level::kSafe) {
        crouch_ = kCrouchFloor;
        return;
    }

    if (level_ == Level::kLanding) {
        const float t = now_ms - land_started_ms_;
        if (t >= land_ms_) {
            level_ = Level::kSafe;
            crouch_ = kCrouchFloor;
            return;
        }
        float u = t / land_ms_;
        if (u < 0.0f) u = 0.0f;          // a backwards clock holds full height
        crouch_ = 1.0f - (1.0f - kCrouchFloor) * u;
        return;
    }

    // -- not tripped yet ---------------------------------------------------
    if (dv == 0) return;                 // no reading: hold, do not count

    if (dv <= land_dv_) {
        ++below_land_;
    } else {
        below_land_ = 0;
    }
    if (dv <= warn_dv_) {
        ++below_warn_;
    } else {
        below_warn_ = 0;
    }

    if (below_land_ >= confirm_) {
        level_ = Level::kLanding;
        land_started_ms_ = now_ms;
        crouch_ = 1.0f;
        return;
    }
    level_ = (below_warn_ >= confirm_) ? Level::kWarn : Level::kOk;
}

void Guard::reset() {
    level_ = Level::kOk;
    crouch_ = 1.0f;
    land_started_ms_ = 0.0f;
    below_warn_ = 0;
    below_land_ = 0;
}

}  // namespace battguard
