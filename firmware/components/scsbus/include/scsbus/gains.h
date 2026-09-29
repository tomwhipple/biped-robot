// Position-loop gains (registers 21 P, 22 D): the guarded EEPROM write and
// the read-back check that gates arming.
//
// Plan B (DESIGN.md section 4, docs/servo-map.md section 4) raises P on the
// roll and knee servos. Those values live in each servo's EEPROM, and a
// factory reset or a swapped spare silently comes back at P = 32 -- a robot
// that then falls at its first crossover. So the gains are (a) written only
// through writePositionGains(), which refuses unless the servo reports torque
// OFF and verifies what it wrote, and (b) read back and compared with the
// compiled expected table (main/servo_gains.h) at boot and before every arm,
// by checkPositionGains().
//
// Neither function ever writes a goal position, touches torque, or writes a
// register other than 55 (the EEPROM lock), 21 and 22. A gain write with
// torque off does not move a servo; AGENTS.md still treats every bus write as
// a potential motion command, hence the torque gate in front of it.
//
// Pure: everything goes through scsbus::Bus, so the host tests drive both
// against a scripted port (host/test_scsbus.cpp).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "scsbus/bus.h"

namespace scsbus {

struct PositionGains {
    uint8_t p;   // register 21
    uint8_t d;   // register 22
};

// The STS3215 factory values (memory table v3.7). The prototype runs them.
constexpr PositionGains kFactoryGains{32, 32};

// One 2-byte READ at register 21 (P, then D).
Status readPositionGains(Bus& bus, uint8_t id, PositionGains& out);

enum class GainWrite : uint8_t {
    kOk = 0,         // written, locked, read back equal
    kBadArg,         // broadcast / out-of-range id, or P == 0
    kTorqueUnknown,  // torque state unreadable: refused, nothing written
    kTorqueOn,       // torque is ON: refused, nothing written
    kUnlockFailed,   // EEPROM unlock not acknowledged (lock re-sent)
    kWriteFailed,    // gain write not acknowledged (lock re-sent)
    kLockFailed,     // gains written, but the re-lock not acknowledged
    kReadbackFailed, // written and locked; the read-back did not answer
    kMismatch,       // written and locked; the read-back differs
};

const char* gainWriteName(GainWrite r);

// The guarded write, on ONE servo:
//   1. read register 40 (torque enable); refuse unless it answers 0;
//   2. unlock the EEPROM (55 <- 0);
//   3. write P and D in one 2-byte WRITE at 21;
//   4. lock the EEPROM (55 <- 1) -- attempted on every path past step 2, so a
//      failure never leaves the EEPROM unlocked;
//   5. `settle()` (the ESP side waits for the EEPROM commit, as `id` does;
//      nullptr on the host);
//   6. read 21/22 back and compare.
// `readback` (optional) receives what step 6 read.
GainWrite writePositionGains(Bus& bus, uint8_t id, PositionGains g,
                             PositionGains* readback = nullptr,
                             void (*settle)() = nullptr);

// The arm gate: one row per servo that must be checked.
struct GainExpect {
    uint8_t id;
    PositionGains gains;
};
struct GainCheck {
    uint8_t id;
    bool answered;        // the 2-byte read at 21 came back
    PositionGains read;   // valid when answered
    bool ok;              // answered AND read == expected
};

// Read registers 21/22 from every servo in `expect` and compare. Fills
// `out[0..n)` and returns the number of servos that are NOT ok (a servo that
// does not answer counts: an unreadable gain is not a verified one).
size_t checkPositionGains(Bus& bus, const GainExpect* expect, size_t n,
                          GainCheck* out);

}  // namespace scsbus
