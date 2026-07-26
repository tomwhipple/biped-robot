#include "cli.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obs/actuation.h"
#include "obs/obs_spec.h"
#include "policy/mlp.h"
#include "scsbus/bus.h"
#include "shared.h"

namespace cli {
namespace {

char g_line[128];      // one owner (the housekeeping task), no heap

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
    say(out, "id %ld -> %ld: %s\r\n", a, b, statusName(st));
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

void cmdMove(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 3) { out("usage: move <id> <ticks> [ms]\r\n"); return; }
    const long id = num(argv[1]);
    const long ticks = num(argv[2], 2048);
    const long ms = argc >= 4 ? num(argv[3], 0) : 0;
    if (ticks < 0 || ticks > 4095) {
        out("ticks must be 0-4095 (2048 == middle)\r\n");
        return;
    }
    const uint8_t ids[1] = {static_cast<uint8_t>(id)};
    const int32_t tgt[1] = {static_cast<int32_t>(ticks)};
    const scsbus::Status st = bus->syncWritePositions(
        ids, tgt, 1, static_cast<uint16_t>(ms), 0, 0);
    say(out, "move id %ld -> %ld ticks over %ld ms: %s\r\n", id, ticks, ms,
        statusName(st));
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
        v <= 10.5 ? "  *** LAND THE ROBOT (3S floor) ***" : "");
}

void cmdMode(Sink out, bool run) {
    robot::g_mode_request.store(run ? robot::Mode::kRun : robot::Mode::kBench);
    out(run ? "control loop armed -- bus handed to core 1\r\n"
            : "benched -- control loop released torque and gave up the bus\r\n");
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
    out("  move <id> <ticks> [ms]   2048 == middle, 4096 ticks per revolution\r\n");
    out("  release [id] | torque [id]   no id = broadcast\r\n");
    out("  middle <id>          latch the current angle as 2048, torque off\r\n");
    out("  volt                 pack voltage, read off the servos (no board ADC)\r\n");
    out("  run | bench          hand the bus to / take it back from the loop\r\n");
    out("  stat                 tick timing and fault counters\r\n");
}

void execute(const char* line, Sink out) {
    char buf[96];
    strncpy(buf, line, sizeof buf - 1);
    buf[sizeof buf - 1] = 0;
    char* argv[8];
    const int argc = split(buf, argv, 8);
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
    else if (!strcmp(c, "volt")) cmdVolt(out);
    else if (!strcmp(c, "run")) cmdMode(out, true);
    else if (!strcmp(c, "bench")) cmdMode(out, false);
    else if (!strcmp(c, "stat")) cmdStat(out);
    else out("? (try `help`)\r\n");
}

}  // namespace cli
