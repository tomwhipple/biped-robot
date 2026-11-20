#include "pack_sense.h"

#include "battguard/pack.h"
#include "board.h"
#include "driver/i2c_master.h"
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

// Register the chip on the board's bus and check it converts the bus voltage
// continuously; if not (it never is after a power-on reset), reset it to its
// defaults. False leaves no handle behind.
bool attach() {
    g_dev = boardI2c().addDevice(ina::kAddr, static_cast<uint32_t>(board::kI2cHz));
    if (g_dev == nullptr) return false;
    uint8_t b[2] = {0, 0};
    bool ok = imu::Qmi8658Imu::devRead(g_dev, ina::kRegConfig, b, 2, kI2cTimeoutMs,
                                       kI2cTimeoutMs);
    if (ok) {
        g_config = ina::reg16(b);
        g_config_reset = !ina::busContinuous(g_config);
        if (g_config_reset) {
            const uint8_t rst[3] = {ina::kRegConfig,
                                    static_cast<uint8_t>(ina::kConfigReset >> 8),
                                    static_cast<uint8_t>(ina::kConfigReset & 0xFF)};
            ok = imu::Qmi8658Imu::devRawWrite(g_dev, rst, sizeof rst, kI2cTimeoutMs,
                                              kI2cTimeoutMs);
        }
    }
    if (!ok) packSenseDetach();
    return ok;
}

}  // namespace

void packSenseDetach() {
    if (g_dev != nullptr) {
        i2c_master_bus_rm_device(static_cast<i2c_master_dev_handle_t>(g_dev));
        g_dev = nullptr;
    }
}

void packSenseTick() {
    const uint32_t now = nowMs();
    if (g_sense.due(now)) {
        uint8_t b[2] = {0, 0};
        // Attach on the first try and after any failure: a chip that came back
        // (or a bus re-made by `imu reinit`) gets its configuration checked again.
        const bool ok = (g_dev != nullptr || attach()) &&
                        imu::Qmi8658Imu::devRead(g_dev, ina::kRegBus, b, 2, kI2cTimeoutMs,
                                                 kI2cTimeoutMs);
        if (ok) {
            g_sense.ok(ina::busMv(ina::reg16(b)), now);
        } else {
            g_sense.fail(now);
            packSenseDetach();
        }
        const uint16_t mv = g_sense.packMv(now);
        g_pack_mv.store(mv);
        if (ok && mv != 0) g_pack_reads.fetch_add(1);   // after the value: shared.h
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
