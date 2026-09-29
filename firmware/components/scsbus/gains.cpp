#include "scsbus/gains.h"

namespace scsbus {

Status readPositionGains(Bus& bus, uint8_t id, PositionGains& out) {
    uint8_t b[2] = {0, 0};
    const Status st = bus.read(id, kRegPosP, 2, b);
    if (st == Status::kOk) {
        out.p = b[0];
        out.d = b[1];
    }
    return st;
}

const char* gainWriteName(GainWrite r) {
    switch (r) {
        case GainWrite::kOk: return "ok";
        case GainWrite::kBadArg: return "bad-arg";
        case GainWrite::kTorqueUnknown: return "torque-unknown";
        case GainWrite::kTorqueOn: return "torque-on";
        case GainWrite::kUnlockFailed: return "unlock-failed";
        case GainWrite::kWriteFailed: return "write-failed";
        case GainWrite::kLockFailed: return "lock-failed";
        case GainWrite::kReadbackFailed: return "readback-failed";
        case GainWrite::kMismatch: return "mismatch";
    }
    return "?";
}

GainWrite writePositionGains(Bus& bus, uint8_t id, PositionGains g,
                             PositionGains* readback, void (*settle)()) {
    // One servo, by name: a broadcast gain write would re-tune every joint
    // on the bus at once and nothing could read it back.
    if (id > kMaxId) return GainWrite::kBadArg;
    // P = 0 is a servo that holds nothing under load: not a gain, a limp
    // joint with torque "on". Refused rather than written.
    if (g.p == 0) return GainWrite::kBadArg;

    // The torque gate. An unreadable state is a refusal, never a guess --
    // the same rule torqueGate() applies to goal writes in the CLI.
    uint8_t torque = 0;
    if (bus.readU8(id, kRegTorqueEnable, torque) != Status::kOk) {
        return GainWrite::kTorqueUnknown;
    }
    if (torque != 0) return GainWrite::kTorqueOn;

    if (bus.unlockEeprom(id) != Status::kOk) {
        bus.lockEeprom(id);          // best effort: never leave it unlocked
        return GainWrite::kUnlockFailed;
    }
    const uint8_t pd[2] = {g.p, g.d};
    if (bus.write(id, kRegPosP, pd, 2) != Status::kOk) {
        bus.lockEeprom(id);
        return GainWrite::kWriteFailed;
    }
    if (bus.lockEeprom(id) != Status::kOk) return GainWrite::kLockFailed;
    if (settle) settle();

    PositionGains got{0, 0};
    if (readPositionGains(bus, id, got) != Status::kOk) {
        return GainWrite::kReadbackFailed;
    }
    if (readback) *readback = got;
    return (got.p == g.p && got.d == g.d) ? GainWrite::kOk
                                          : GainWrite::kMismatch;
}

size_t checkPositionGains(Bus& bus, const GainExpect* expect, size_t n,
                          GainCheck* out) {
    size_t bad = 0;
    for (size_t i = 0; i < n; ++i) {
        GainCheck& c = out[i];
        c.id = expect[i].id;
        c.read = PositionGains{0, 0};
        c.answered = readPositionGains(bus, c.id, c.read) == Status::kOk;
        c.ok = c.answered && c.read.p == expect[i].gains.p &&
               c.read.d == expect[i].gains.d;
        if (!c.ok) ++bad;
    }
    return bad;
}

}  // namespace scsbus
