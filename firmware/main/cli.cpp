#include "cli.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obs/actuation.h"
#include "obs/obs_spec.h"
#include "policy/mlp.h"
#include "scsbus/bus.h"
#include "cal_store.h"
#include "mech_envelope.h"
#include "shared.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace cli {
namespace {

char g_line[128];      // one owner (the housekeeping task), no heap

// Settle time between writing an ID and re-checking it.
//
// A servo assigned ID 10 through the vendor web UI came back as ID 1 on
// 2026-07-26; the same assignment through this path survived a power cycle.
// That is n=1 each way and the mechanism is NOT established -- a too-early
// power cut, a repeated "Set New ID" against a stale listID, and the vendor's
// LockEprom() addressing the new ID rather than the old all fit the evidence,
// and the last of those is what this code does too. So do not read this delay
// as a diagnosis. The verify below is the part that actually earns its keep:
// whatever the cause, a lost ID stops being silent.
constexpr int kEepromCommitMs = 2000;

const char* statusName(scsbus::Status s) {
    switch (s) {
        case scsbus::Status::kOk: return "ok";
        case scsbus::Status::kTimeout: return "timeout";
        case scsbus::Status::kBadReply: return "bad-reply";
        case scsbus::Status::kWrongId: return "wrong-id";
        case scsbus::Status::kTxFail: return "tx-fail";
        case scsbus::Status::kBadArg: return "bad-arg";
    }
    return "?";
}

void say(Sink out, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void say(Sink out, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_line, sizeof g_line, fmt, ap);
    va_end(ap);
    out(g_line);
}

// The bus is only ours while the control loop is benched. Refusing here is
// what stops a curious `move` from fighting the policy for the same joint.
scsbus::Bus* claimBus(Sink out) {
    if (robot::g_mode_request.load() != robot::Mode::kBench ||
        robot::g_ctrl_owns_bus.load()) {
        out("busy: the control loop owns the bus -- run `bench` first\r\n");
        return nullptr;
    }
    if (!robot::g_bus) {
        out("no bus\r\n");
        return nullptr;
    }
    return robot::g_bus;
}

// Split on spaces in place. Returns argc.
int split(char* s, char** argv, int max) {
    int argc = 0;
    while (*s && argc < max) {
        while (*s == ' ' || *s == '\t') ++s;
        if (!*s) break;
        argv[argc++] = s;
        while (*s && *s != ' ' && *s != '\t') ++s;
        if (*s) *s++ = 0;
    }
    return argc;
}

long num(const char* s, long fallback = -1) {
    char* end = nullptr;
    const long v = strtol(s, &end, 0);
    return (end && end != s) ? v : fallback;
}

// -- commands --------------------------------------------------------------

void cmdScan(Sink out) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    out("scanning IDs 0-253 ...\r\n");
    int found = 0;
    for (int id = 0; id <= scsbus::kMaxId; ++id) {
        if (!bus->ping(static_cast<uint8_t>(id))) continue;
        ++found;
        int32_t pos = 0;
        uint8_t volt = 0, flags = 0;
        bus->readPosition(static_cast<uint8_t>(id), pos);
        bus->readVoltageDeciVolts(static_cast<uint8_t>(id), volt);
        bus->readStatus(static_cast<uint8_t>(id), flags);
        // Name it if it is one of ours, so a mis-numbered servo is obvious.
        const char* role = "-";
        for (int j = 0; j < obs::kNumJoints; ++j) {
            if (obs::kServoId[j] == id) role = obs::kJointNames[j];
        }
        say(out, "  id %3d  pos %5ld  %4.1f V  err 0x%02X  %s\r\n", id,
            static_cast<long>(pos), volt / 10.0, static_cast<unsigned>(flags),
            role);
    }
    say(out, "%d servo(s)\r\n", found);
}

void cmdPing(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc >= 2) {
        const long id = num(argv[1]);
        say(out, "id %ld: %s\r\n", id,
            bus->ping(static_cast<uint8_t>(id)) ? "present" : "no reply");
        return;
    }
    // No argument: ping exactly the IDs the policy expects, in action order.
    int missing = 0;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const bool ok = bus->ping(obs::kServoId[j]);
        if (!ok) ++missing;
        say(out, "  %-12s id %2d  %s\r\n", obs::kJointNames[j],
            obs::kServoId[j], ok ? "ok" : "MISSING");
    }
    say(out, "%d of %d present\r\n", obs::kNumJoints - missing,
        obs::kNumJoints);
}

void cmdId(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 3) { out("usage: id <old> <new>\r\n"); return; }
    const long a = num(argv[1]), b = num(argv[2]);
    if (a < 0 || a > scsbus::kMaxId || b < 0 || b > scsbus::kMaxId) {
        out("ids must be 0-253\r\n");
        return;
    }
    if (bus->ping(static_cast<uint8_t>(b)) && a != b) {
        say(out, "refusing: id %ld already answers -- two servos with the "
                 "same id will not enumerate\r\n", b);
        return;
    }
    const scsbus::Status st = bus->setId(static_cast<uint8_t>(a),
                                         static_cast<uint8_t>(b));
    if (st != scsbus::Status::kOk) {
        say(out, "id %ld -> %ld: %s\r\n", a, b, statusName(st));
        return;
    }
    // An ID write that reports ok has still only reached the EEPROM cell, not
    // committed it. Cutting power inside that window loses the change silently
    // -- exactly how a servo set through the vendor web UI came back as ID 1
    // on 2026-07-26. Hold, then prove the servo answers to the new ID before
    // saying anything reassuring, so nobody unplugs on the strength of "ok".
    vTaskDelay(pdMS_TO_TICKS(kEepromCommitMs));
    if (!bus->ping(static_cast<uint8_t>(b))) {
        say(out, "id %ld -> %ld: WROTE BUT DID NOT VERIFY -- rescan before "
                 "trusting it\r\n", a, b);
        return;
    }
    say(out, "id %ld -> %ld: ok, verified after commit -- safe to power down\r\n",
        a, b);
}

void cmdPos(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 2) { out("usage: pos <id>\r\n"); return; }
    const long id = num(argv[1]);
    scsbus::Feedback f{};
    const scsbus::Status st = bus->readFeedback(static_cast<uint8_t>(id), f);
    if (st != scsbus::Status::kOk) {
        say(out, "id %ld: %s\r\n", id, statusName(st));
        return;
    }
    say(out, "id %ld  pos %5ld  spd %5ld  load %4ld  %4.1f V  %u C  "
             "err 0x%02X\r\n",
        id, static_cast<long>(f.position), static_cast<long>(f.speed),
        static_cast<long>(f.load), f.voltage_dv / 10.0,
        static_cast<unsigned>(f.temperature_c),
        static_cast<unsigned>(f.status));
}

// Refuse a goal write while torque is off (or unreadable). STS servos
// AUTO-ENABLE torque when a goal position arrives, so "torque is off, this
// is inert" is FALSE -- that assumption drove every joint through its hard
// stop and broke both feet off on 2026-08-02. Motion is now an explicit
// two-step: `torque`, then the move.
bool torqueGate(scsbus::Bus* bus, Sink out, uint8_t id) {
    uint8_t on = 0;
    const scsbus::Status st = bus->readU8(id, scsbus::kRegTorqueEnable, on);
    if (st != scsbus::Status::kOk) {
        say(out, "id %d: cannot read torque state (%s) -- refusing to write "
            "a goal blind\r\n", id, statusName(st));
        return false;
    }
    if (on == 0) {
        say(out, "id %d: torque is OFF, and a goal write would auto-enable "
            "it and MOVE the joint. `torque %d` first if you mean it.\r\n",
            id, id);
        return false;
    }
    return true;
}

// Clamp bench ticks to the joint's CALIBRATED mechanical envelope, when the
// target servo is one of the policy's. Raw 0..4095 is the encoder's range,
// not the mechanism's -- the difference is a horn driving a printed part
// past its stop. The envelope (mech_envelope.h) is the measured mechanical
// truth, deliberately separate from the plant's policy range: the bench can
// e.g. hyperextend a knee to its measured -95..+95 even though policies
// train in -95..+5 (split introduced 2026-08-03 after the clamp cut short
// a manual ROM session at the plant's limits).
int32_t clampToJointRange(uint8_t id, int32_t ticks, Sink out) {
    for (int j = 0; j < obs::kNumJoints; ++j) {
        if (robot::servoIds()[j] != id) continue;
        const obs::Calibration& cal = robot::calibration();
        const int32_t a = obs::angleToStepsRaw(j, robot::kMechLo[j], cal);
        const int32_t b = obs::angleToStepsRaw(j, robot::kMechHi[j], cal);
        const int32_t lo = a < b ? a : b, hi = a < b ? b : a;
        if (ticks < lo || ticks > hi) {
            const int32_t c = ticks < lo ? lo : hi;
            say(out, "id %d: %ld is outside joint %d's range [%ld..%ld]%s, "
                "clamped to %ld\r\n", id, static_cast<long>(ticks), j,
                static_cast<long>(lo), static_cast<long>(hi),
                robot::g_cal_from_nvs ? "" : " (UNCALIBRATED defaults!)",
                static_cast<long>(c));
            return c;
        }
        return ticks;
    }
    return ticks;               // not a policy servo: encoder range only
}

void cmdMove(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 3) { out("usage: move <id> <ticks> [ms]\r\n"); return; }
    const long id = num(argv[1]);
    const long ticks = num(argv[2], 2048);
    const long ms = argc >= 4 ? num(argv[3], 0) : 0;
    // Optional 5th arg: goal SPEED in steps/s (register 46). Without it the
    // servo slews at maximum and the goal-time argument is effectively
    // ignored -- measured 2026-07-26 at ~3000 steps/s for a "9000 ms" move.
    // A commanded speed is what makes a slow constant-velocity sweep (and so
    // a friction measurement) possible at all.
    const long spd = argc >= 5 ? num(argv[4], 0) : 0;
    if (ticks < 0 || ticks > 4095) {
        out("ticks must be 0-4095 (2048 == middle)\r\n");
        return;
    }
    if (!torqueGate(bus, out, static_cast<uint8_t>(id))) return;
    const uint8_t ids[1] = {static_cast<uint8_t>(id)};
    const int32_t tgt[1] = {clampToJointRange(static_cast<uint8_t>(id),
                                              static_cast<int32_t>(ticks),
                                              out)};
    const scsbus::Status st = bus->syncWritePositions(
        ids, tgt, 1, static_cast<uint16_t>(ms),
        static_cast<uint16_t>(spd), 0);
    say(out, "move id %ld -> %ld ticks, %ld ms, spd %ld: %s\r\n", id, ticks,
        ms, spd, statusName(st));
}

void cmdPose(Sink out, int argc, char** argv) {
    // The whole pose in ONE broadcast SYNC WRITE -- the same frame the
    // control loop uses, so all ten servos latch their targets together
    // instead of rippling through ten `move`s. Targets are ticks in JOINT
    // order (obs_spec kJointNames: L_hip_yaw..L_ankle, R_hip_yaw..R_ankle);
    // the servo-map permutation and its errata are applied here, exactly as
    // in the act path.
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 1 + obs::kNumJoints) {
        out("usage: pose <t0..t9 ticks, joint order> [steps/s]\r\n");
        out("       joints: L yaw,roll,pitch,knee,ankle then R same\r\n");
        return;
    }
    int32_t tgt[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const long t = num(argv[1 + i]);
        if (t < 0 || t > 4095) {
            say(out, "arg %d: ticks must be 0-4095 (2048 == middle)\r\n",
                i + 1);
            return;
        }
        tgt[i] = clampToJointRange(robot::servoIds()[i],
                                   static_cast<int32_t>(t), out);
    }
    // Gate on every servo BEFORE the first byte hits the wire: this is one
    // broadcast frame, so there is no such thing as moving only the safe
    // ones.
    for (int i = 0; i < obs::kNumJoints; ++i) {
        if (!torqueGate(bus, out, robot::servoIds()[i])) return;
    }
    // Bench default is a GENTLE sweep (~0.9 rad/s), not the servo's max --
    // a hand-typed pose with a typo shouldn't snap a limb across its range.
    // An explicit 0 asks for unlimited, same convention as `move`.
    const long spd = argc >= 2 + obs::kNumJoints
                         ? num(argv[1 + obs::kNumJoints], 600) : 600;
    const scsbus::Status st = bus->syncWritePositions(
        robot::servoIds(), tgt, obs::kNumJoints, 0,
        static_cast<uint16_t>(spd), 0);
    say(out, "pose -> %d joints, spd %ld: %s\r\n", obs::kNumJoints, spd,
        statusName(st));
}

void cmdShape(Sink out, int argc, char** argv) {
    // Live-tunable on purpose: a single atomic float with one writer (here)
    // and one reader (ctrl), per the shared.h rules -- so the pole can be
    // swept while the robot walks and the jerk change felt directly.
    if (argc >= 2) {
        const float hz = strtof(argv[1], nullptr);
        if (hz < 0.0f || hz > 25.0f) {
            out("hz must be 0 (off) .. 25 (Nyquist at the 50 Hz tick)\r\n");
            return;
        }
        robot::g_shape_hz.store(hz);
    }
    const float hz = robot::g_shape_hz.load();
    if (hz > 0.0f) {
        say(out, "shape: C2 pole %.2f Hz (3 cascaded lags) + per-joint goal "
            "speed\r\n", static_cast<double>(hz));
    } else {
        out("shape: off (raw targets, max-speed slew)\r\n");
    }
}

void cmdTorque(Sink out, int argc, char** argv, bool on) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    const bool one = argc >= 2;
    const uint8_t id = one ? static_cast<uint8_t>(num(argv[1]))
                           : scsbus::kBroadcastId;
    const scsbus::Status st = bus->torqueEnable(id, on);
    say(out, "%s %s: %s\r\n", on ? "torque" : "release",
        one ? argv[1] : "all", statusName(st));
}

void cmdMiddle(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 2) { out("usage: middle <id>\r\n"); return; }
    const long id = num(argv[1]);
    const scsbus::Status st = bus->setMiddle(static_cast<uint8_t>(id));
    say(out, "id %ld: current position latched as 2048 (%s). Torque is now "
             "OFF -- re-`torque` before loading the joint.\r\n",
        id, statusName(st));
}

void cmdVolt(Sink out) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    // The board has NO voltage divider to an ADC (see main/board.h): the only
    // pack-voltage sense is each servo's own register 62, at 0.1 V.
    int n = 0;
    int sum = 0;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        uint8_t dv = 0;
        if (bus->readVoltageDeciVolts(obs::kServoId[j], dv) ==
            scsbus::Status::kOk) {
            sum += dv;
            ++n;
        }
    }
    if (!n) { out("no servo answered\r\n"); return; }
    const double v = sum / 10.0 / n;
    say(out, "bus %.1f V (mean of %d servos, 0.1 V resolution)%s\r\n", v, n,
        v * 10.0 <= battguard::kWarn3S
            ? "  *** LAND THE ROBOT (3S floor) ***" : "");
}

// The pack guard. Read-only unless `batt reset` -- see shared.h battGuard()
// for why the write half is bench-only.
void cmdBatt(Sink out, int argc, char** argv) {
    battguard::Guard& g = robot::battGuard();
    if (argc >= 2 && !strcmp(argv[1], "reset")) {
        // Bench-mode gate, same rule as the mutating `cal` subcommands: this
        // WRITES state the control task owns, and only in bench mode is ctrl
        // guaranteed to return before touching it. Checked directly rather
        // than via claimBus() because this needs the mode, not the bus.
        if (robot::g_mode_request.load() != robot::Mode::kBench ||
            robot::g_ctrl_owns_bus.load()) {
            out("busy: the control loop owns the guard -- `bench` first\r\n");
            return;
        }
        g.reset();
        out("pack guard cleared -- do this ONLY after swapping in a fresh "
            "pack\r\n");
        return;
    }
    const char* lvl = "?";
    switch (g.level()) {
        case battguard::Level::kOk: lvl = "ok"; break;
        case battguard::Level::kWarn: lvl = "WARN (land it)"; break;
        case battguard::Level::kLanding: lvl = "LANDING (crouching)"; break;
        case battguard::Level::kSafe: lvl = "SAFE (torque off, latched)"; break;
    }
    say(out, "pack %s -- last %.1f V, warn %.1f V, land %.1f V\r\n", lvl,
        g.lastDv() / 10.0, g.warnDv() / 10.0, g.landDv() / 10.0);
    say(out, "  %d/%d consecutive ticks below the land line, crouch cmd "
             "%.2f\r\n",
        g.belowTicks(), battguard::kConfirmTicks,
        static_cast<double>(g.crouch()));
    if (g.latched()) {
        out("  LATCHED. Swap the pack, then `batt reset` (bench mode) or "
            "reboot.\r\n");
    }
    if (g.lastDv() == 0) {
        out("  (no reading yet -- voltage arrives on servo feedback, so the "
            "loop must have run)\r\n");
    }
}

void cmdMode(Sink out, bool run) {
    // No valid as-built calibration, no run. The default Calibration (zero
    // 2048, dir +1) is exactly the raw-middle pose that broke the feet on
    // 2026-08-02, and a stored blob measured under a different servo map is
    // invalidated at load (cal_store.h v2) for the same reason.
    if (run && !robot::g_cal_from_nvs) {
        out("REFUSED: no as-built calibration in NVS (missing, or "
            "invalidated by a servo-map change). Run the `cal` workflow "
            "first -- driving the loop on 2048-defaults twists the robot.\r\n");
        return;
    }
    robot::g_mode_request.store(run ? robot::Mode::kRun : robot::Mode::kBench);
    out(run ? "control loop armed -- bus handed to core 1\r\n"
            : "benched -- control loop released torque and gave up the bus\r\n");
}


// -- calibration -----------------------------------------------------------
//
// Two mechanisms exist and they are NOT interchangeable; docs/bringup-day1.md
// records the split:
//   `middle <id>`  writes the SERVO's own offset (reg 40 <- 128). Coarse zero,
//                  lives in the servo's EEPROM, travels with the servo, and
//                  survives reflashing the board.
//   `cal ...`      writes the FIRMWARE's offset (NVS). Fine trim on top, plus
//                  the per-joint direction sign, which the servo cannot
//                  express at all.
// Use `middle` first at the CAD-neutral pose, then `cal zero` to absorb the
// residual the horn splines cannot -- the ST3215 horn seats on discrete teeth,
// so a purely mechanical zero is never exact.
void cmdCal(Sink out, int argc, char** argv) {
    obs::Calibration& cal = robot::calibration();
    if (argc < 2 || !strcmp(argv[1], "show")) {
        for (int j = 0; j < obs::kNumJoints; ++j) {
            say(out, "  %-12s id %2d  zero %4ld  dir %+d\r\n",
                obs::kJointNames[j], obs::kServoId[j],
                static_cast<long>(cal.zero_steps[j]), cal.dir[j]);
        }
        say(out, "%s\r\n", robot::g_cal_from_nvs
                                ? "loaded from NVS at boot"
                                : "DEFAULTS -- nothing stored (cal save)");
        return;
    }
    if (!strcmp(argv[1], "zero")) {
        scsbus::Bus* bus = claimBus(out);
        if (!bus) return;
        // Latch wherever the joints are RIGHT NOW as angle zero. Only correct
        // when the robot is physically held at the CAD-neutral pose.
        const long only = argc >= 3 ? num(argv[2], -1) : -1;
        int done = 0;
        for (int j = 0; j < obs::kNumJoints; ++j) {
            if (only >= 0 && j != only) continue;
            int32_t pos = 0;
            if (bus->readPosition(obs::kServoId[j], pos) !=
                scsbus::Status::kOk) {
                say(out, "  %-12s NO REPLY -- left unchanged\r\n",
                    obs::kJointNames[j]);
                continue;
            }
            cal.zero_steps[j] = pos;
            ++done;
            say(out, "  %-12s zero <- %ld\r\n", obs::kJointNames[j],
                static_cast<long>(pos));
        }
        say(out, "%d joint(s) zeroed -- NOT saved yet, run `cal save`\r\n",
            done);
        return;
    }
    if (!strcmp(argv[1], "dir") && argc >= 4) {
        if (!claimBus(out)) return;           // writer-vs-ctrl-reader guard
        const long j = num(argv[2], -1), d = num(argv[3], 0);
        if (j < 0 || j >= obs::kNumJoints || (d != 1 && d != -1)) {
            out("usage: cal dir <joint 0-9> <1|-1>\r\n");
            return;
        }
        cal.dir[j] = static_cast<int8_t>(d);
        say(out, "%s dir <- %+ld -- run `cal save`\r\n",
            obs::kJointNames[j], d);
        return;
    }
    if (!strcmp(argv[1], "set") && argc >= 5) {
        // Type a row of docs/servo-map.md's as-built table straight in --
        // recovery path when the robot cannot be posed (e.g. broken parts).
        if (!claimBus(out)) return;           // writer-vs-ctrl-reader guard
        const long j = num(argv[2], -1);
        const long z = num(argv[3], -1);
        const long d = num(argv[4], 0);
        if (j < 0 || j >= obs::kNumJoints || z < 0 || z > 4095 ||
            (d != 1 && d != -1)) {
            out("usage: cal set <joint 0-9> <zero 0-4095> <1|-1>\r\n");
            return;
        }
        cal.zero_steps[j] = static_cast<int32_t>(z);
        cal.dir[j] = static_cast<int8_t>(d);
        say(out, "%s zero <- %ld dir <- %+ld -- run `cal save`\r\n",
            obs::kJointNames[j], z, d);
        return;
    }
    if (!strcmp(argv[1], "migrate")) {
        // Normally unnecessary -- boot auto-migrates a v1 blob that matches
        // the compiled as-built table (asbuilt_cal.h). This is the manual
        // path for everything else: a v1 blob with UNKNOWN values loads into
        // the live cal only, and a human decides before `cal save`.
        if (!claimBus(out)) return;
        obs::Calibration old;
        if (!robot::calLoadV1(old)) {
            out("no readable v1 blob in NVS (already v2, empty, or corrupt)\r\n");
            return;
        }
        cal = old;
        if (robot::calIsAsBuilt(cal)) {
            const bool ok = robot::calSave(cal);
            robot::g_cal_from_nvs = ok;
            say(out, "v1 blob matches the as-built table -- saved as v2: "
                "%s\r\n", ok ? "ok" : "FAILED");
            return;
        }
        robot::g_cal_from_nvs = false;        // not blessed until saved as v2
        for (int j = 0; j < obs::kNumJoints; ++j) {
            say(out, "  %-12s id %2d  zero %4ld  dir %+d\r\n",
                obs::kJointNames[j], obs::kServoId[j],
                static_cast<long>(cal.zero_steps[j]), cal.dir[j]);
        }
        out("v1 blob does NOT match asbuilt_cal.h -- loaded into the LIVE\r\n"
            "cal only. Verify against docs/servo-map.md, then `cal save`,\r\n"
            "or recalibrate.\r\n");
        return;
    }
    if (!strcmp(argv[1], "save")) {
        const bool ok = robot::calSave(cal);
        robot::g_cal_from_nvs = ok;
        say(out, "cal save: %s\r\n", ok ? "ok" : "FAILED");
        return;
    }
    if (!strcmp(argv[1], "load")) {
        if (!claimBus(out)) return;
        obs::Calibration fresh;
        const bool ok = robot::calLoad(fresh);
        if (ok) { cal = fresh; robot::g_cal_from_nvs = true; }
        say(out, "cal load: %s\r\n", ok ? "ok" : "nothing stored");
        return;
    }
    if (!strcmp(argv[1], "reset")) {
        if (!claimBus(out)) return;
        cal = obs::Calibration();
        robot::calErase();
        robot::g_cal_from_nvs = false;
        out("cal reset to defaults (zero 2048, dir +1) and erased\r\n");
        return;
    }
    out("usage: cal [show] | zero [joint] | dir <joint> <1|-1> | set <joint> <zero> <1|-1>\r\n       | migrate | save | load | reset\r\n");
}

void cmdStat(Sink out) {
    say(out, "ticks %lu  late %lu (%u%%)  worst %lu us  state %u  "
             "servo_err 0x%04X\r\n",
        static_cast<unsigned long>(robot::g_telemetry.ticks.load()),
        static_cast<unsigned long>(robot::g_telemetry.overruns.load()),
        static_cast<unsigned>(robot::g_telemetry.loop_late_pct.load()),
        static_cast<unsigned long>(robot::g_telemetry.worst_tick_us.load()),
        static_cast<unsigned>(robot::g_telemetry.state.load()),
        static_cast<unsigned>(robot::g_telemetry.servo_err.load()));
    say(out, "  phases us: read %lu  imu %lu  obs %lu  net %lu  write %lu  "
             "other %lu\r\n",
        static_cast<unsigned long>(robot::g_telemetry.us_read.load()),
        static_cast<unsigned long>(robot::g_telemetry.us_imu.load()),
        static_cast<unsigned long>(robot::g_telemetry.us_obs.load()),
        static_cast<unsigned long>(robot::g_telemetry.us_net.load()),
        static_cast<unsigned long>(robot::g_telemetry.us_write.load()),
        static_cast<unsigned long>(robot::g_telemetry.us_other.load()));
    say(out, "policy: %s, %d joints, obs %d, run %s\r\n",
        policy::kWeightsArePlaceholder ? "PLACEHOLDER WEIGHTS" : "exported",
        obs::kNumJoints, obs::kObsDim, obs::kRunName);
}

}  // namespace

void banner(Sink out) {
    out("\r\nbimo firmware v1 -- bench mode (nothing moves until `run`)\r\n");
    out("  scan                 ping IDs 0-253, report position/voltage/faults\r\n");
    out("  ping [id]            no id = check exactly the policy's servos\r\n");
    out("  id <old> <new>       assign a servo ID (EEPROM, one servo on the bus)\r\n");
    out("  pos <id>             position, speed, load, voltage, temp, faults\r\n");
    out("  move <id> <ticks> [ms] [steps/s]   2048 == middle, 4096 ticks/rev\r\n");
    out("  pose <t0..t9> [steps/s]   all 10 targets, ONE frame (joint order)\r\n");
    out("  release [id] | torque [id]   no id = broadcast\r\n");
    out("  middle <id>          latch the current angle as 2048, torque off\r\n");
    out("  volt                 pack voltage, read off the servos (no board ADC)\r\n");
    out("  batt [reset]         under-voltage guard state; reset after a pack swap\r\n");
    out("  cal [show|zero|dir|set|migrate|save|load|reset]   zero + dir (NVS)\r\n");
    out("  shape [hz]           C2 command-shaping pole; 0 = off (raw/jerky)\r\n");
    out("  run | bench          hand the bus to / take it back from the loop\r\n");
    out("  stat                 tick timing and fault counters\r\n");
}

void execute(const char* line, Sink out) {
    char buf[96];
    strncpy(buf, line, sizeof buf - 1);
    buf[sizeof buf - 1] = 0;
    // 13 tokens: `pose` + 10 joint targets + [steps/s] + one spare.
    char* argv[13];
    const int argc = split(buf, argv, 13);
    if (argc == 0) return;
    const char* c = argv[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) banner(out);
    else if (!strcmp(c, "scan")) cmdScan(out);
    else if (!strcmp(c, "ping")) cmdPing(out, argc, argv);
    else if (!strcmp(c, "id")) cmdId(out, argc, argv);
    else if (!strcmp(c, "pos")) cmdPos(out, argc, argv);
    else if (!strcmp(c, "move")) cmdMove(out, argc, argv);
    else if (!strcmp(c, "release")) cmdTorque(out, argc, argv, false);
    else if (!strcmp(c, "torque")) cmdTorque(out, argc, argv, true);
    else if (!strcmp(c, "middle")) cmdMiddle(out, argc, argv);
    else if (!strcmp(c, "pose")) cmdPose(out, argc, argv);
    else if (!strcmp(c, "shape")) cmdShape(out, argc, argv);
    else if (!strcmp(c, "volt")) cmdVolt(out);
    else if (!strcmp(c, "batt")) cmdBatt(out, argc, argv);
    else if (!strcmp(c, "run")) cmdMode(out, true);
    else if (!strcmp(c, "bench")) cmdMode(out, false);
    else if (!strcmp(c, "cal")) cmdCal(out, argc, argv);
    else if (!strcmp(c, "stat")) cmdStat(out);
    else out("? (try `help`)\r\n");
}

}  // namespace cli
