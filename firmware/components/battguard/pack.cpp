#include "battguard/pack.h"

namespace battguard {

uint8_t inputDv(uint8_t servo_dv, uint16_t pack_mv) {
    const uint8_t pack_dv = pack_mv >= kPackFloorMv ? dvFromMv(pack_mv) : uint8_t{0};
    if (servo_dv == 0) return pack_dv;
    if (pack_dv == 0) return servo_dv;
    return pack_dv < servo_dv ? pack_dv : servo_dv;
}

bool PackSense::due(uint32_t now_ms) const {
    // Signed difference: wrap-safe for intervals under 24 days.
    return !tried_ || static_cast<int32_t>(now_ms - next_ms_) >= 0;
}

void PackSense::ok(uint16_t mv, uint32_t now_ms) {
    if (mv > kPackMaxMv) {
        fail(now_ms);
        return;
    }
    tried_ = true;
    ++reads_;
    have_ = true;
    at_ms_ = now_ms;
    last_mv_ = mv;
    fails_in_row_ = 0;
    next_ms_ = now_ms + kPeriodMs;
}

void PackSense::fail(uint32_t now_ms) {
    tried_ = true;
    ++failures_;
    if (fails_in_row_ < kMissAfter) ++fails_in_row_;
    next_ms_ = now_ms + (missing() ? kRetryMs : kPeriodMs);
}

uint16_t PackSense::packMv(uint32_t now_ms) const {
    if (!have_ || now_ms - at_ms_ > kStaleMs) return 0;
    return last_mv_;
}

}  // namespace battguard
