// The pack voltage from the General Driver's INA219 (0x42), the hardware half of
// battguard/pack.h (issue #109).
//
// HOUSEKEEPING TASK ONLY: packSenseTick() runs in its loop, and every CLI command
// (including `imu reinit`, which tears the I2C bus down) runs on the same task, so
// the device handle needs no lock of its own. It publishes shared.h g_pack_mv and
// g_pack_reads for ctrl and the beacon.
//
// The cost, bounded (docs/firmware-design.md, "The pack sense"):
//   * never on core 1: the control loop reads two atomics;
//   * one register read (bus voltage, 2 bytes) every 100 ms: about 0.15 ms on the
//     400 kHz bus, under the IMU sampler's mutex, which the 250 Hz sampler then waits
//     out at most once in 25 samples, inside its 4 ms period;
//   * the lock wait and the transfer each time out at kI2cTimeoutMs, so a held bus
//     stalls housekeeping by at most ~2x that per attempt;
//   * a chip that NAKs or is missing is "missing" after 3 failures and is then tried
//     once every 2 s: one NAK (tens of us) every 2 s, on the same device handle (it
//     is made once; only `imu reinit` removes it). The reading meanwhile is 0,
//     "unknown", and the guard holds on the servos alone.
#pragma once
#include <stdint.h>

namespace robot {

void packSenseTick();
// Drop the device handle before the bus it lives on is deleted (imuReinitLive),
// under the bus lock. False: the driver refused and the handle is kept, so the
// bus must not be deleted. The next tick attaches again.
bool packSenseDetach();

struct PackSenseStatus {
    bool attached = false;
    bool missing = false;
    uint16_t mv = 0;          // fresh reading, 0 = none
    uint16_t last_mv = 0;     // the last good one, however old
    uint32_t reads = 0;
    uint32_t failures = 0;
    uint16_t config = 0;      // the configuration register as found at attach
    bool config_reset = false;  // ... and whether it had to be reset to convert
};
PackSenseStatus packSenseStatus();

}  // namespace robot
