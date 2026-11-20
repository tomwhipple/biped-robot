// The pack's own voltage sense: the General Driver's INA219, and how it is
// combined with the servos' register 62 for battguard (issue #109).
//
// The INA219 (U2, 0x42 on the board's I2C bus) sits across R11 (0.01 Ohm) on
// DC_IN -> VIN, in front of the board's 5 V buck only (docs/wiring.md, "What
// the INA219 measures"). Its BUS VOLTAGE register is VIN to ground: the pack,
// behind the reverse-polarity FET, SW1 and the shunt, a few tens of mV under
// DC_IN at the logic's ~1 A. None of the servo current flows through R11, so
// the reading does not depend on the servo bus: it is there benched, with
// torque released and, on power lite, with the servo rail off.
//
// Only the bus voltage register is read. It needs no calibration (that is for
// current and power), and the power-on configuration (0x399F: 32 V range,
// shunt and bus continuous) converts it every 532 us. The firmware writes the
// chip only to put it back in that configuration if its mode does not convert
// the bus continuously (busContinuous() false), because a chip left in
// power-down or triggered mode would hand back one old reading forever.
//
// Pure, like Guard: no I2C, no clock. The hardware half is
// main/pack_sense.cpp; host/test_battguard.cpp asserts what is here.
#pragma once
#include <stdint.h>

namespace battguard {

// -- the INA219 (TI SBOS448) ------------------------------------------------
namespace ina219 {
constexpr uint8_t kAddr = 0x42;            // board.h kPowerAddr
constexpr uint8_t kRegConfig = 0x00;
constexpr uint8_t kRegBus = 0x02;
constexpr uint16_t kConfigReset = 0x8000;  // RST: every register to its default
constexpr uint16_t kConfigDefault = 0x399F;

// Registers are big-endian on the wire.
constexpr uint16_t reg16(const uint8_t b[2]) {
    return static_cast<uint16_t>((b[0] << 8) | b[1]);
}
// Bus voltage register: bits 15..3 are the reading, 4 mV a count (bit 1
// CNVR and bit 0 OVF concern the power and current registers).
constexpr uint16_t busMv(uint16_t reg) {
    return static_cast<uint16_t>((reg >> 3) * 4u);
}
// MODE (bits 2..0) 110 or 111: the bus voltage converts continuously.
constexpr bool busContinuous(uint16_t config) { return (config & 0x7u) >= 6u; }
}  // namespace ina219

// A reading above this is not a 3S pack (12.6 V full, the board's silkscreen
// limit is 12.6 V and the INA219 reads up to 32 V): treated as a failed read.
constexpr uint16_t kPackMaxMv = 16000;
// A reading below this is no pack: the board is on USB, which powers the
// logic and not DC_IN. 6.0 V is 2.0 V a cell, a pack that is already ruined,
// and about the STS3215's own floor; the board's 5 V buck does not run there.
// Below it the reading goes out in the beacon but never into the guard.
constexpr uint16_t kPackFloorMv = 6000;

// Millivolts -> the guard's decivolts. TRUNCATES, so the guard never sees the
// pack higher than measured (10.59 V is 105: warn), and saturates at 255.
constexpr uint8_t dvFromMv(uint16_t mv) {
    return mv / 100u > 255u ? uint8_t{255} : static_cast<uint8_t>(mv / 100u);
}

// The guard's input for one tick, from both senses. `servo_dv` is register 62
// this tick (0: no servo answered); `pack_mv` the INA219's fresh reading (0:
// none). Either alone, or the LOWER of the two: the guard can then trip no
// later than it would on the servos alone, and losing either sense does not
// blind it. A pack reading under kPackFloorMv counts as none.
uint8_t inputDv(uint8_t servo_dv, uint16_t pack_mv);

// When to read the chip, and whether the last reading still counts.
//
// The I2C bus is shared with the 250 Hz IMU sampler (one mutex, imu/qmi8658.h
// devRead), so the read is kept rare and bounded: kPeriodMs while the chip
// answers; after kMissAfter failures in a row it is called missing and tried
// once every kRetryMs, so a board without the chip, or one that NAKs, costs
// one short NAK every two seconds. A reading older than kStaleMs is no
// reading: packMv() returns 0, "unknown", never the last value.
class PackSense {
  public:
    static constexpr uint32_t kPeriodMs = 100;   // 10 Hz, the beacon's rate
    static constexpr uint32_t kRetryMs = 2000;
    static constexpr int kMissAfter = 3;
    static constexpr uint32_t kStaleMs = 1000;

    // A read is due (wrap-safe uint32 milliseconds).
    bool due(uint32_t now_ms) const;
    // The outcome of the read due() asked for. A good read must be inside
    // kPackMaxMv; ok() takes anything above as a failure.
    void ok(uint16_t mv, uint32_t now_ms);
    void fail(uint32_t now_ms);

    // The last good reading if it is fresh, else 0.
    uint16_t packMv(uint32_t now_ms) const;
    // kMissAfter failures in a row (or none ever answered after that many).
    bool missing() const { return fails_in_row_ >= kMissAfter; }
    uint32_t reads() const { return reads_; }
    uint32_t failures() const { return failures_; }
    uint16_t lastMv() const { return last_mv_; }

  private:
    bool tried_ = false;
    uint32_t next_ms_ = 0;
    bool have_ = false;
    uint32_t at_ms_ = 0;
    uint16_t last_mv_ = 0;
    int fails_in_row_ = 0;
    uint32_t reads_ = 0;
    uint32_t failures_ = 0;
};

}  // namespace battguard
