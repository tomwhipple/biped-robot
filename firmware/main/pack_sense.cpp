#include "pack_sense.h"

#include "battguard/pack.h"
#include "board.h"
#include "esp_timer.h"
#include "imu/qmi8658.h"
#include "shared.h"

namespace robot {
namespace {

namespace ina = battguard::ina219;
static_assert(board::kPowerAddr == ina::kAddr, "board.h and battguard/pack.h name one chip");

// Each wait (the shared lock, then the transfer) gives up after this. A good read
// takes ~0.15 ms; 5 ms is a held bus, not a slow chip.
constexpr int kI2cTimeoutMs = 5;

// Housekeeping only (pack_sense.h).
battguard::PackSense g_sense;
void* g_dev = nullptr;          // i2c_master_dev_handle_t on boardI2c()'s bus
uint16_t g_config = 0;
bool g_config_reset = false;

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// The device handle is made once and kept: a read that fails (a NAK, a missing
// chip) keeps it and tries again, so nothing is allocated per retry. Only
// packSenseDetach() (before `imu reinit` deletes the bus) removes it.
//
// The configuration is checked on the first good contact, and again after any
// failure (a chip that came back from a reset or a brown-out of its own): if it
// does not convert the bus continuously, it is reset to its defaults.
bool g_config_ok = false;

bool checkConfig() {
    uint8_t b[2] = {0, 0};
    if (!imu::Qmi8658Imu::devRead(g_dev, ina::kRegConfig, b, 2, kI2cTimeoutMs, kI2cTimeoutMs))
        return false;
    g_config = ina::reg16(b);
    g_config_reset = !ina::busContinuous(g_config);
    if (g_config_reset) {
        const uint8_t rst[3] = {ina::kRegConfig, static_cast<uint8_t>(ina::kConfigReset >> 8),
                                static_cast<uint8_t>(ina::kConfigReset & 0xFF)};
        if (!imu::Qmi8658Imu::devRawWrite(g_dev, rst, sizeof rst, kI2cTimeoutMs, kI2cTimeoutMs))
            return false;
    }
    g_config_ok = true;
    return true;
}

}  // namespace

bool packSenseDetach() {
    if (g_dev == nullptr) return true;
    // Under the bus lock, checked: on a refusal the handle is still registered,
    // so it is kept (and the caller must not delete the bus).
    if (!imu::Qmi8658Imu::rmDevice(g_dev)) return false;
    g_dev = nullptr;
    g_config_ok = false;
    return true;
}

void packSenseTick() {
    const uint32_t now = nowMs();
    if (g_sense.due(now)) {
        if (g_dev == nullptr) {
            g_dev = boardI2c().addDevice(ina::kAddr, static_cast<uint32_t>(board::kI2cHz));
        }
        uint8_t b[2] = {0, 0};
        const bool ok = g_dev != nullptr && (g_config_ok || checkConfig()) &&
                        imu::Qmi8658Imu::devRead(g_dev, ina::kRegBus, b, 2, kI2cTimeoutMs,
                                                 kI2cTimeoutMs);
        const uint32_t reads_before = g_sense.reads();
        if (ok) {
            g_sense.ok(ina::busMv(ina::reg16(b)), now);   // > 16 V counts as a failure
        } else {
            g_sense.fail(now);
            g_config_ok = false;
        }
        const uint16_t mv = g_sense.packMv(now);
        g_pack_mv.store(mv);
        // Only a reading PackSense took counts as fresh (after the value: shared.h).
        if (g_sense.reads() != reads_before && mv != 0) g_pack_reads.fetch_add(1);
        return;
    }
    // Between reads: a reading that has gone stale stops being reported.
    if (g_pack_mv.load() != 0 && g_sense.packMv(now) == 0) g_pack_mv.store(0);
}

PackSenseStatus packSenseStatus() {
    const uint32_t now = nowMs();
    PackSenseStatus s;
    s.attached = g_dev != nullptr;
    s.missing = g_sense.missing();
    s.mv = g_sense.packMv(now);
    s.last_mv = g_sense.lastMv();
    s.reads = g_sense.reads();
    s.failures = g_sense.failures();
    s.config = g_config;
    s.config_reset = g_config_reset;
    return s;
}

}  // namespace robot
