#include "cli.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obs/actuation.h"
#include "obs/obs_spec.h"
#include "policy/mlp.h"
#include "scsbus/bus.h"
#include "board.h"
#include "cal_store.h"
#include "imu_sampler.h"
#include "mech_envelope.h"
#include "shared.h"
#include "wifi_link.h"
#include "timesync.h"

#include "esp_timer.h"
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

// `home`'s slew rate, steps/s (~0.45 rad/s). Half `pose`'s bench default:
// see cmdHome for why this move in particular is the slow one.
constexpr long kHomeStepsPerSec = 300;

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
    // Optional 6th arg: ACC register value (100 steps/s^2 per LSB). The
    // loop and every prior bench write sent 0; the 2026-08-31 FRF measured
    // a ~10 rad/s^2 tracking cap at gait amplitudes, so this exists to
    // probe what the servo's accel profile actually does with the value.
    const long acc = argc >= 6 ? num(argv[5], 0) : 0;
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
        static_cast<uint16_t>(spd), static_cast<uint8_t>(acc & 0xFF));
    say(out, "move id %ld -> %ld ticks, %ld ms, spd %ld, acc %ld: %s\r\n", id,
        ticks, ms, spd, acc, statusName(st));
}

// Smooth-pose streamer state (poseTick). Housekeeping task only.
static bool s_traj_active = false;
static TickType_t s_traj_t0 = 0, s_traj_last = 0;
static long s_traj_ms = 0;
static float s_traj_headroom = 1.0f;    // servo speed = (gap to the next sub-target) / tick x this
static uint8_t s_traj_acc = 0;          // servo acceleration register (100 steps/s^2 per LSB)
static int32_t s_traj_bigstep = 1;      // write at once when a joint has this many ticks to go
static long s_traj_minint_ms = 20;      // ... else wait this long between writes (>= 1 tick)
// ZV input shaping (z<tenths of Hz>): the profile is the average of itself
// and a copy delayed by half the period of the structural mode, so the
// vibration the second half excites cancels the first half's. 0 = off.
static long s_traj_zv_ms = 0;
static float s_traj_floor = 15.0f;      // f<steps/s>: minimum servo speed near the ends
// Servo trace (T<id> token, `trace` to print): during the stream and for
// kTraceTailMs after it, poseTick reads one servo's position/speed/load every
// tick -- the leg-side instrument for end-of-motion shake (Tom 2026-09-03:
// "you were going to instrument this for hard numbers"; the pelvis IMU
// cannot see the legs).
constexpr int kTraceMax = 400;
constexpr long kTraceTailMs = 1200;
static uint8_t s_trace_id = 0;
static int16_t s_trace_pos[kTraceMax], s_trace_spd[kTraceMax], s_trace_load[kTraceMax];
static uint16_t s_trace_ms[kTraceMax];
static int s_trace_n = 0;
static bool s_trace_on = false;
static TickType_t s_trace_until = 0;
static int32_t s_traj_start[obs::kNumJoints];
static int32_t s_traj_tgt[obs::kNumJoints];
static int32_t s_traj_sent[obs::kNumJoints];     // last goal written per joint
static uint16_t s_traj_spd[obs::kNumJoints];     // ... and its speed
static TickType_t s_traj_tsent[obs::kNumJoints]; // ... and when

// `reg <id> <addr> [1|2]` -- READ a servo register (EEPROM or RAM), never
// write. Added 2026-09-03 to inspect PosP/PosD/PosI (21-23), dead zones
// (26/27) and acceleration (41) while chasing bench oscillation.
void cmdReg(Sink out, int argc, char** argv) {
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return;
    if (argc < 3) { out("usage: reg <id> <addr> [1|2]   (read-only)\r\n"); return; }
    const long id = num(argv[1]);
    const long addr = num(argv[2]);
    const long n = argc >= 4 ? num(argv[3], 1) : 1;
    if (id < 0 || id > 253 || addr < 0 || addr > 255) { out("bad id/addr\r\n"); return; }
    if (n == 2) {
        uint16_t v = 0;
        const scsbus::Status st = bus->readU16(static_cast<uint8_t>(id),
                                               static_cast<uint8_t>(addr), v);
        if (st != scsbus::Status::kOk) { say(out, "id %ld reg %ld: %s\r\n", id, addr, statusName(st)); return; }
        say(out, "id %ld reg %ld = %u (0x%04X)\r\n", id, addr, static_cast<unsigned>(v), static_cast<unsigned>(v));
    } else {
        uint8_t v = 0;
        const scsbus::Status st = bus->readU8(static_cast<uint8_t>(id),
                                              static_cast<uint8_t>(addr), v);
        if (st != scsbus::Status::kOk) { say(out, "id %ld reg %ld: %s\r\n", id, addr, statusName(st)); return; }
        say(out, "id %ld reg %ld = %u (0x%02X)\r\n", id, addr, static_cast<unsigned>(v), static_cast<unsigned>(v));
    }
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
    // UNISON mode (2026-09-03, Tom: "the ankle/hip servos move, then the
    // knees catch up -- all three should move in unison to stay balanced"):
    // a trailing `t<ms>` gives every servo its OWN speed from its own
    // distance, so all ten arrive together, instead of one speed for all.
    const char* last = argc >= 2 + obs::kNumJoints ? argv[1 + obs::kNumJoints]
                                                    : nullptr;
    // DEFAULT IS SMOOTH (Tom 2026-09-03: "if a position is commanded, the
    // controller should take it there smoothly"): with no mode argument the
    // duration comes from the longest move at 300 steps/s (>= 1 s). A
    // trailing number keeps the legacy constant-speed slew, t<ms> the
    // unison mode, s<ms> an explicit smooth duration.
    const bool smooth = (last == nullptr) || last[0] == 's';
    if (smooth) {
        long ms = 0;
        if (last) {
            ms = num(last + 1, 3000);
        } else {
            int32_t worst = 0;
            for (int i = 0; i < obs::kNumJoints; ++i) {
                int32_t cur = tgt[i];
                if (bus->readPosition(robot::servoIds()[i], cur) == scsbus::Status::kOk) {
                    const int32_t d = cur > tgt[i] ? cur - tgt[i] : tgt[i] - cur;
                    if (d > worst) worst = d;
                }
            }
            ms = (worst * 1000L) / 300L;
            if (ms < 1000) ms = 1000;
        }
        if (ms < 200 || ms > 20000) {
            out("pose s<ms>: duration must be 200-20000 ms\r\n");
            return;
        }
        for (int i = 0; i < obs::kNumJoints; ++i) {
            int32_t cur = tgt[i];
            if (bus->readPosition(robot::servoIds()[i], cur) !=
                scsbus::Status::kOk) {
                say(out, "id %u did not answer -- no smooth pose\r\n",
                    robot::servoIds()[i]);
                return;
            }
            s_traj_start[i] = cur;
            s_traj_tgt[i] = tgt[i];
            s_traj_sent[i] = cur;
            s_traj_spd[i] = 15;
            s_traj_tsent[i] = xTaskGetTickCount();
        }
        // optional tuning tokens after s<ms>: h<pct> speed headroom (servo
        // speed = profile speed x pct/100; <100 keeps the servo running
        // continuously behind the stream instead of stop-starting at 50 Hz),
        // a<n> servo acceleration register (100 steps/s^2 per LSB, 0 = max).
        s_traj_headroom = 1.0f;
        s_traj_acc = 0;
        s_traj_bigstep = 1;
        s_traj_minint_ms = 20;
        s_traj_zv_ms = 0;
        s_traj_floor = 15.0f;
        s_trace_id = 0;
        for (int k = 1 + obs::kNumJoints; k < argc; ++k) {
            if (argv[k][0] == 's') continue;
            if (argv[k][0] == 'h') {
                long pct = num(argv[k] + 1, 90);
                if (pct < 30) pct = 30;
                if (pct > 300) pct = 300;
                s_traj_headroom = static_cast<float>(pct) / 100.0f;
            } else if (argv[k][0] == 'a') {
                long a = num(argv[k] + 1, 0);
                if (a < 0) a = 0;
                if (a > 254) a = 254;
                s_traj_acc = static_cast<uint8_t>(a);
            } else if (argv[k][0] == 'b') {          // b<ticks>: big-step threshold
                long b = num(argv[k] + 1, 4);
                if (b < 1) b = 1;
                if (b > 64) b = 64;
                s_traj_bigstep = static_cast<int32_t>(b);
            } else if (argv[k][0] == 'T') {          // T<id>: trace this servo
                const long id = num(argv[k] + 1, 0);
                if (id >= 1 && id <= 253) s_trace_id = static_cast<uint8_t>(id);
            } else if (argv[k][0] == 'f') {          // f<steps/s>: speed floor
                const long f = num(argv[k] + 1, 15);
                if (f >= 5 && f <= 500) s_traj_floor = static_cast<float>(f);
            } else if (argv[k][0] == 'z') {          // z<tenths Hz>: ZV shaper at that mode
                const long t = num(argv[k] + 1, 25);
                if (t >= 5 && t <= 200) s_traj_zv_ms = 5000L / t;   // half period, ms
            } else if (argv[k][0] == 'i') {          // i<ms>: min interval between small writes
                long iv = num(argv[k] + 1, 100);
                if (iv < 20) iv = 20;
                if (iv > 1000) iv = 1000;
                s_traj_minint_ms = iv;
            }
        }
        s_traj_ms = ms;
        s_traj_t0 = xTaskGetTickCount();
        s_traj_last = s_traj_t0;
        s_traj_active = true;
        s_trace_n = 0;
        s_trace_on = s_trace_id != 0;
        s_trace_until = s_traj_t0 + pdMS_TO_TICKS(ms + s_traj_zv_ms + kTraceTailMs);
        say(out, "pose -> %d joints, SMOOTH minimum-jerk over %ld ms (speed x%.2f, "
                 "acc %u, step %ld ticks / %ld ms, ZV %ld ms): streaming\r\n",
            obs::kNumJoints, ms, static_cast<double>(s_traj_headroom),
            static_cast<unsigned>(s_traj_acc), static_cast<long>(s_traj_bigstep),
            s_traj_minint_ms, s_traj_zv_ms);
        return;
    }
    if (last && last[0] == 't') {
        const long ms = num(last + 1, 2000);
        if (ms < 100 || ms > 20000) {
            out("pose t<ms>: duration must be 100-20000 ms\r\n");
            return;
        }
        uint16_t spds[obs::kNumJoints];
        long lo = 100000, hi = 0;
        for (int i = 0; i < obs::kNumJoints; ++i) {
            int32_t cur = tgt[i];
            if (bus->readPosition(robot::servoIds()[i], cur) !=
                scsbus::Status::kOk) {
                say(out, "id %u did not answer -- no unison pose\r\n",
                    robot::servoIds()[i]);
                return;
            }
            const long d = cur > tgt[i] ? cur - tgt[i] : tgt[i] - cur;
            long v = (d * 1000L) / ms;
            if (v < 10) v = 10;          // 0 means unlimited on the wire
            if (v > 3400) v = 3400;
            spds[i] = static_cast<uint16_t>(v);
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        const scsbus::Status st = bus->syncWritePositions(
            robot::servoIds(), tgt, spds, obs::kNumJoints, 0);
        say(out, "pose -> %d joints in UNISON over %ld ms (speeds %ld..%ld "
                 "steps/s): %s\r\n",
            obs::kNumJoints, ms, lo, hi, statusName(st));
        return;
    }
    const long spd = last ? num(last, 600) : 600;
    const scsbus::Status st = bus->syncWritePositions(
        robot::servoIds(), tgt, obs::kNumJoints, 0,
        static_cast<uint16_t>(spd), 0);
    say(out, "pose -> %d joints, spd %ld: %s\r\n", obs::kNumJoints, spd,
        statusName(st));
}

// -- reset the servos ------------------------------------------------------
// `home`: every joint to its CALIBRATED ZERO -- which on this robot is the
// standing pose (asbuilt_cal.h: kJointDefault is all zeros, so each joint's
// zero_steps IS where it stands) -- in one gentle broadcast frame.
//
// This is the one bench command that deliberately turns torque ON before it
// writes a goal, and that needs saying plainly, because everything else here
// refuses to (torqueGate above, and the feet that paid for it on 2026-08-02).
// The difference is intent. torqueGate exists to stop a goal write from
// SILENTLY becoming motion; this command IS the motion, asked for by name, by
// an operator who wants a heap of robot back on its feet. The states it is
// most useful in -- E-stopped, fallen, benched after a run -- are exactly the
// states where torque is off and a plain `pose` would be refused.
//
// So the safety is bought elsewhere, three ways:
//   * NO CALIBRATION, NO HOME. Default zeros are 2048 everywhere, the
//     raw-middle pose that broke the feet. Same gate as `run` (cmdMode).
//   * The targets are clamped to the measured mechanical envelope, like
//     every other bench write (clampToJointRange).
//   * It slews SLOWLY -- 300 steps/s, ~0.45 rad/s, half of `pose`'s already
//     gentle bench default -- because the robot may be lying in a pose the
//     move has to walk out of, and because a slow limb is one a hand can
//     catch.
// Torque is left ON at the end, holding the pose: a robot that homed and
// then went limp has only fallen over more tidily.
// Returns the verdict, because over the radio `out` is a tether nobody is
// holding: linkHome puts what this returns into the beacon. Every early exit
// below is a REASON, not a silence.
// Pending readback after a home: the slew deadline and the targets written.
// Touched only by the housekeeping task (typed `home`, linkHome and
// homeVerify all run there), so plain statics.
static bool s_home_pending = false;
static TickType_t s_home_deadline = 0;
static int32_t s_home_tgt[obs::kNumJoints];
constexpr int32_t kHomeTolTicks = 12;      // ~1 deg: the servo's own deadband is ~3

linkproto::ArmResult homeAll(Sink out, long spd) {
    s_home_pending = false;
    scsbus::Bus* bus = claimBus(out);
    if (!bus) return linkproto::ArmResult::kHomeBusFailed;
    if (!robot::g_cal_from_nvs) {
        out("REFUSED: no as-built calibration in NVS, so \"zero\" here means "
            "2048 on every joint -- the raw-middle pose that broke the feet, "
            "not a stand. Run the `cal` workflow first.\r\n");
        return linkproto::ArmResult::kHomeNoCal;
    }
    // The pack guard outranks the link everywhere else (ctrl_task's safety
    // block), so it outranks this too: a flat pack is exactly when powering
    // ten servos to hold a stand is the wrong answer, and unlike the E-stop
    // this latch is the ROBOT's, not the operator's, to clear.
    if (robot::battGuard().torqueMustRelease()) {
        out("REFUSED: the pack under-voltage guard has torque latched off. "
            "Swap the pack, then `batt reset` -- see `batt`.\r\n");
        return linkproto::ArmResult::kHomeLowBatt;
    }
    const obs::Calibration& cal = robot::calibration();
    int32_t tgt[obs::kNumJoints];
    for (int j = 0; j < obs::kNumJoints; ++j) {
        tgt[j] = clampToJointRange(robot::servoIds()[j], cal.zero_steps[j],
                                   out);
    }
    // Torque first, then the goal -- the explicit two-step, just with both
    // halves on one line because that is what was asked for. Broadcast, so
    // there is no window in which half the robot is holding and half is not.
    const scsbus::Status ts = bus->torqueEnable(scsbus::kBroadcastId, true);
    if (ts != scsbus::Status::kOk) {
        say(out, "torque enable failed (%s) -- not writing goals\r\n",
            statusName(ts));
        return linkproto::ArmResult::kHomeBusFailed;
    }
    // How far is the longest move? Sets the readback deadline: the write is
    // a broadcast with no reply, and on 2026-09-02 one "ok" moved nothing,
    // so the verdict now waits for the joints to actually get there.
    int32_t worst = 0;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        int32_t cur = 0;
        if (bus->readPosition(robot::servoIds()[j], cur) == scsbus::Status::kOk) {
            const int32_t d = cur > tgt[j] ? cur - tgt[j] : tgt[j] - cur;
            if (d > worst) worst = d;
        } else {
            worst = 4096;             // unknown start: allow the full slew
        }
    }
    // The move itself is the smooth streamer (poseTick): a minimum-jerk
    // profile no faster than the old constant `spd` on the longest joint,
    // and never shorter than 1.5 s. The old constant-speed home rang the
    // structure at 1.0 rad/s of torso rate from a 20 deg crouch (2026-09-03);
    // the streamed pose family measured <= 0.05.
    long dur_ms = spd > 0 ? (worst * 1000L) / spd : 1500L;
    if (dur_ms < 2500L) dur_ms = 2500L;      // gentle: the crouch runs rang least at >= 3 s
    if (dur_ms > 8000L) dur_ms = 8000L;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        int32_t cur = tgt[j];
        if (bus->readPosition(robot::servoIds()[j], cur) != scsbus::Status::kOk) {
            say(out, "home: id %u did not answer -- no move\r\n",
                robot::servoIds()[j]);
            return linkproto::ArmResult::kHomeBusFailed;
        }
        s_traj_start[j] = cur;
        s_traj_tgt[j] = tgt[j];
        s_traj_sent[j] = cur;
        s_traj_spd[j] = 15;
        s_traj_tsent[j] = xTaskGetTickCount();
    }
    s_traj_ms = dur_ms;
    s_traj_headroom = 1.0f;
    s_traj_acc = 0;
    s_traj_bigstep = 1;
    s_traj_minint_ms = 20;
    s_traj_zv_ms = 0;
    s_traj_floor = 15.0f;
    s_traj_t0 = xTaskGetTickCount();
    s_traj_last = s_traj_t0;
    s_traj_active = true;
    const scsbus::Status st = scsbus::Status::kOk;
    say(out, "home -> %d joints to their calibrated zero, SMOOTH over %ld ms "
             "(worst move %ld ticks): streaming\r\n",
        obs::kNumJoints, dur_ms, static_cast<long>(worst));
    const long slew_ms = dur_ms + 700L;
    memcpy(s_home_tgt, tgt, sizeof s_home_tgt);
    s_home_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(slew_ms);
    s_home_pending = true;
    say(out, "  reading back in %ld ms\r\n", slew_ms);
    return linkproto::ArmResult::kHomePending;
}


// The tethered `home`. 0 steps/s asks for the servo's maximum, same
// convention as `move` and `pose`.
void cmdHome(Sink out, int argc, char** argv) {
    homeAll(out, argc >= 2 ? num(argv[1], kHomeStepsPerSec)
                           : kHomeStepsPerSec);
}

void cmdObsFreeze(Sink out, int argc, char** argv) {
    // obsfreeze [none|up|gyro|imu|dq ...] -- bench diagnostic, not persistent.
    if (argc >= 2) {
        uint32_t fz = 0;
        for (int i = 1; i < argc; ++i) {
            if (!strcmp(argv[i], "none")) fz = 0;
            else if (!strcmp(argv[i], "up")) fz |= robot::kObsFreezeUp;
            else if (!strcmp(argv[i], "gyro")) fz |= robot::kObsFreezeGyro;
            else if (!strcmp(argv[i], "imu")) fz |= robot::kObsFreezeUp | robot::kObsFreezeGyro;
            else if (!strcmp(argv[i], "dq")) fz |= robot::kObsFreezeDq;
            else { say(out, "obsfreeze: unknown '%s' (none|up|gyro|imu|dq)\r\n", argv[i]); return; }
        }
        robot::g_obs_freeze.store(fz);
    }
    const uint32_t fz = robot::g_obs_freeze.load();
    say(out, "obsfreeze: up %s  gyro %s  dq %s  (policy obs only; beacon + guards see real values)\r\n",
        (fz & robot::kObsFreezeUp) ? "FROZEN(0,0,1)" : "live",
        (fz & robot::kObsFreezeGyro) ? "FROZEN(0)" : "live",
        (fz & robot::kObsFreezeDq) ? "FROZEN(0)" : "live");
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
        // Publish the verdict BEFORE printing it. Over the radio this atomic
        // is the only way the refusal reaches the operator -- `out` is the
        // tether, and a wireless arm has nobody watching it (shared.h
        // g_arm_result; the beacon in wifi_link.cpp carries it).
        robot::g_arm_result.store(
            static_cast<uint8_t>(linkproto::ArmResult::kRefusedNoCal));
        out("REFUSED: no as-built calibration in NVS (missing, or "
            "invalidated by a servo-map change). Run the `cal` workflow "
            "first -- driving the loop on 2048-defaults twists the robot.\r\n");
        return;
    }
    robot::g_arm_result.store(
        static_cast<uint8_t>(linkproto::ArmResult::kAccepted));
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
    say(out, "  imu sampler: %lu samples, %lu failures (%d ms period, tick-averaged gyro)\r\n",
        static_cast<unsigned long>(robot::imuSampleTotal()),
        static_cast<unsigned long>(robot::imuSampleFailures()), robot::kImuPeriodMs);
    say(out, "policy: %s, %d joints, obs %d, run %s\r\n",
        policy::kWeightsArePlaceholder ? "PLACEHOLDER WEIGHTS" : "exported",
        obs::kNumJoints, obs::kObsDim, obs::kRunName);
}

// -- obsdump ---------------------------------------------------------------
//
// The instrument for the one layer SIL cannot cover: the numbers the real
// sensors put into obs::Inputs. See obs_dump.h. This command only sets the
// mode -- the housekeeping streamer owns the UART writes and the 1.5 KB line
// buffer, so there is exactly one of each.
const char* dumpModeName(robot::ObsDumpMode m) {
    switch (m) {
        case robot::ObsDumpMode::kOff: return "off";
        case robot::ObsDumpMode::kStream: return "on";
        case robot::ObsDumpMode::kOnce: return "once";
    }
    return "?";
}

void cmdObsDump(Sink out, int argc, char** argv) {
    const bool armed = robot::g_mode_request.load() == robot::Mode::kRun;

    if (argc < 2 || !strcmp(argv[1], "show")) {
        say(out, "obsdump %s  published %lu records  loop %s\r\n",
            dumpModeName(robot::g_obs_dump.mode()),
            static_cast<unsigned long>(robot::g_obs_dump.sequence()),
            armed ? "armed" : "BENCHED");
        say(out, "  %d columns per record, ~%d Hz, prefix OBS,\r\n",
            robot::kObsDumpCols, robot::kObsDumpHz);
        out("  usage: obsdump on | off | once\r\n");
        return;
    }

    if (!strcmp(argv[1], "off")) {
        robot::g_obs_dump.setMode(robot::ObsDumpMode::kOff);
        out("obsdump off\r\n");
        return;
    }

    const bool once = !strcmp(argv[1], "once");
    if (once || !strcmp(argv[1], "on")) {
        if (!armed) {
            // Not a policy choice -- there is genuinely nothing to dump. In
            // bench mode ctrl_task warms the attitude filter and then
            // `continue`s: no servo read, no assembleFrame, no observation.
            out("obsdump: the loop is BENCHED, so it never builds an\r\n"
                "  observation -- ctrl returns before the servos are read.\r\n"
                "  `run` first (with the robot supported), then `obsdump on`.\r\n");
            return;
        }
        robot::g_obs_dump.setMode(once ? robot::ObsDumpMode::kOnce
                                       : robot::ObsDumpMode::kStream);
        // Silent on purpose: the next thing on this UART is the OBSHDR line
        // the streamer emits, so a host tool sees the column names first.
        return;
    }

    out("usage: obsdump on | off | once\r\n");
}

}  // namespace

static void traceTick(scsbus::Bus* bus, TickType_t now) {
    if (!s_trace_on) return;
    if (now >= s_trace_until || s_trace_n >= kTraceMax) { s_trace_on = false; return; }
    scsbus::Feedback f{};
    if (bus->readFeedback(s_trace_id, f) != scsbus::Status::kOk) return;
    s_trace_ms[s_trace_n] = static_cast<uint16_t>(pdTICKS_TO_MS(now - s_traj_t0));
    s_trace_pos[s_trace_n] = static_cast<int16_t>(f.position);
    s_trace_spd[s_trace_n] = static_cast<int16_t>(f.speed);
    s_trace_load[s_trace_n] = static_cast<int16_t>(f.load);
    ++s_trace_n;
}

void traceDump(Sink out) {
    say(out, "trace id %u: %d samples (ms pos spd load)\r\n",
        static_cast<unsigned>(s_trace_id), s_trace_n);
    for (int k = 0; k < s_trace_n; ++k) {
        say(out, "T %u %d %d %d\r\n", static_cast<unsigned>(s_trace_ms[k]),
            static_cast<int>(s_trace_pos[k]), static_cast<int>(s_trace_spd[k]),
            static_cast<int>(s_trace_load[k]));
    }
    out("trace end\r\n");
}

void poseTick(Sink out) {
    if (!s_traj_active) {
        // trace tail after the stream: keep sampling until s_trace_until
        if (s_trace_on) {
            const TickType_t now = xTaskGetTickCount();
            if (now - s_traj_last >= pdMS_TO_TICKS(20)) {
                s_traj_last = now;
                scsbus::Bus* bus = claimBus(out);
                if (bus) traceTick(bus, now);
            }
        }
        return;
    }
    if (robot::g_ctrl_owns_bus.load() ||
        robot::g_mode_request.load() == robot::Mode::kRun) {
        s_traj_active = false;
        out("smooth pose cancelled -- the loop was armed\r\n");
        return;
    }
    const TickType_t now = xTaskGetTickCount();
    if (now - s_traj_last < pdMS_TO_TICKS(20)) return;     // ~50 Hz evaluation
    s_traj_last = now;
    scsbus::Bus* bus = claimBus(out);
    if (!bus) { s_traj_active = false; return; }
    const long el = static_cast<long>(pdTICKS_TO_MS(now - s_traj_t0));
    // quintic minimum-jerk: s = 10t^3 - 15t^4 + 6t^5, optionally ZV-shaped
    auto quintic = [](float t) {
        if (t <= 0.0f) return 0.0f;
        if (t >= 1.0f) return 1.0f;
        const float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return 10.0f * t3 - 15.0f * t4 + 6.0f * t5;
    };
    const long total_ms = s_traj_ms + s_traj_zv_ms;
    float tau = static_cast<float>(el) / static_cast<float>(total_ms);
    if (tau > 1.0f) tau = 1.0f;
    float sc;
    if (s_traj_zv_ms > 0) {
        const float a = static_cast<float>(el) / static_cast<float>(s_traj_ms);
        const float b = static_cast<float>(el - s_traj_zv_ms) / static_cast<float>(s_traj_ms);
        sc = 0.5f * quintic(a) + 0.5f * quintic(b);
    } else {
        sc = quintic(static_cast<float>(el) / static_cast<float>(s_traj_ms));
    }
    // Write rule (Tom 2026-09-03, "make sure we don't have single ticks that
    // are too close together in time"): a joint gets a new goal when it has
    // >= kBigStep ticks to move, or >= 1 tick AND >= kMinIntervalMs since its
    // last write, or at the very end. Its speed is the distance over the
    // interval it will take -- the servo GLIDES between updates instead of
    // being poked one tick every 20 ms against its dead zone.
    const int32_t kBigStep = s_traj_bigstep;
    const long kMinIntervalMs = s_traj_minint_ms;
    int32_t steps[obs::kNumJoints];
    uint16_t spds[obs::kNumJoints];
    bool changed = false;
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const float d = static_cast<float>(s_traj_tgt[i] - s_traj_start[i]);
        const int32_t want = tau >= 1.0f
            ? s_traj_tgt[i]
            : s_traj_start[i] + static_cast<int32_t>(lroundf(sc * d));
        const int32_t dstep = want - s_traj_sent[i];
        const int32_t mag = dstep < 0 ? -dstep : dstep;
        const long since_ms = static_cast<long>(pdTICKS_TO_MS(now - s_traj_tsent[i]));
        const bool go = (mag >= kBigStep) ||
                        (mag >= 1 && since_ms >= kMinIntervalMs) ||
                        (tau >= 1.0f && mag >= 1);
        if (!go) {
            steps[i] = s_traj_sent[i];
            spds[i] = s_traj_spd[i];
            continue;
        }
        changed = true;
        // CLOSED-LOOP speed (2026-09-03, knee trace): the speed that closes
        // the gap from the servo's MEASURED position to this sub-target within
        // one tick. Open-loop "profile speed x 0.9" let lag accumulate to ~10 %
        // of the move, which the servo then crept off at the floor speed for
        // a second after the profile ended -- the wobbly finish.
        int32_t actual = s_traj_sent[i];
        bus->readPosition(robot::servoIds()[i], actual);
        const int32_t gap = want - actual;
        const int32_t gapmag = gap < 0 ? -gap : gap;
        const long dt_ms = since_ms < 20 ? 20 : since_ms;
        float v = static_cast<float>(gapmag) * 1000.0f / static_cast<float>(dt_ms) *
                  s_traj_headroom;
        if (v < s_traj_floor) v = s_traj_floor;
        if (v > 3400.0f) v = 3400.0f;
        steps[i] = want;
        spds[i] = static_cast<uint16_t>(v);
        s_traj_sent[i] = want;
        s_traj_spd[i] = spds[i];
        s_traj_tsent[i] = now;
    }
    if (changed) {
        bus->syncWritePositions(robot::servoIds(), steps, spds, obs::kNumJoints,
                                s_traj_acc);
    }
    traceTick(bus, now);
    if (tau >= 1.0f) {
        s_traj_active = false;
        say(out, "smooth pose done (%ld ms)\r\n", el);
    }
}

void homeVerify(Sink out) {
    if (!s_home_pending) return;
    if (xTaskGetTickCount() < s_home_deadline) return;
    s_home_pending = false;
    if (robot::g_ctrl_owns_bus.load() ||
        robot::g_mode_request.load() == robot::Mode::kRun) {
        // Armed in the meantime: that request's verdict already replaced
        // ours, and the bus is not the CLI's to read. Say so, store nothing.
        out("home readback skipped -- the loop was armed before the slew "
            "finished\r\n");
        return;
    }
    scsbus::Bus* bus = claimBus(out);
    if (!bus) {
        robot::g_arm_result.store(
            static_cast<uint8_t>(linkproto::ArmResult::kHomeNotReached));
        return;
    }
    int off = 0;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        int32_t cur = 0;
        const uint8_t id = robot::servoIds()[j];
        if (bus->readPosition(id, cur) != scsbus::Status::kOk) {
            say(out, "home readback: id %u did not answer\r\n", id);
            ++off;
            continue;
        }
        const int32_t d = cur - s_home_tgt[j];
        if (d > kHomeTolTicks || d < -kHomeTolTicks) {
            say(out, "home readback: %s id %u at %ld, zero %ld (%+ld ticks)\r\n",
                obs::kJointNames[j], id, static_cast<long>(cur),
                static_cast<long>(s_home_tgt[j]), static_cast<long>(d));
            ++off;
        }
    }
    if (off == 0) {
        out("home readback: all 10 joints at their zeros -- torque is ON and "
            "HOLDING the stand; `release` to let go, or arm to walk\r\n");
        robot::g_arm_result.store(
            static_cast<uint8_t>(linkproto::ArmResult::kDisarmedHome));
    } else {
        say(out, "home readback: %d joint(s) OFF their zeros -- the reset did "
            "not take; check `scan`, torque, pack\r\n", off);
        robot::g_arm_result.store(
            static_cast<uint8_t>(linkproto::ArmResult::kHomeNotReached));
    }
}

// -- imu -------------------------------------------------------------------
//
// This is the bring-up tool, and its most important subcommand is `raw`.
// The board stands vertical and transverse in the pelvis recess, so the
// mounting rotation is ~90 degrees rather than a trim, and no amount of
// reading the schematic tells you reliably which physical axis the chip calls
// X. Tilting the board by hand while watching `imu raw` does.
void cmdImu(Sink out, int argc, char** argv) {
    imu::Qmi8658Imu* dev = robot::onboardImu();

    if (argc >= 2 && !strcmp(argv[1], "scan")) {
        // Bus scan works even with no IMU: that is the point when the part is
        // NOT answering and you need to know whether anything is out there.
        static imu::Qmi8658Imu probe_dev(imu::Qmi8658Imu::Config{
            board::kI2cSda, board::kI2cScl,
            static_cast<uint32_t>(board::kI2cHz), board::kImuAddr, false, 8,
            1024});
        imu::Qmi8658Imu* d = dev ? dev : &probe_dev;
        uint8_t found[16];
        const int n = d->scanBus(found, 16);
        if (!n) {
            say(out, "i2c GPIO %d/%d @ %d Hz: NOTHING answered\r\n",
                static_cast<int>(board::kI2cSda),
                static_cast<int>(board::kI2cScl), board::kI2cHz);
            out("  the board's own sensors live on this bus, so an empty scan\r\n"
                "  means the bus is wrong or held, not that a part is missing\r\n");
            return;
        }
        say(out, "i2c GPIO %d/%d @ %d Hz: %d device(s)\r\n",
            static_cast<int>(board::kI2cSda), static_cast<int>(board::kI2cScl),
            board::kI2cHz, n);
        for (int i = 0; i < n; ++i) {
            const char* who = "?";
            switch (found[i]) {
                case 0x6B: case 0x6A: who = "QMI8658C  6-axis IMU"; break;
                case 0x0C: who = "AK09918C  magnetometer (unused)"; break;
                case 0x77: who = "BMP280    barometer (unused)"; break;
                case 0x42: who = "INA219    power monitor (unused)"; break;
                case 0x4A: case 0x4B: who = "BNO085    (P1 upgrade path)"; break;
                default: break;
            }
            say(out, "  0x%02X  %s\r\n", found[i], who);
        }
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "reinit")) {
        // bench-only: recover a hung bus and re-init the QMI without a
        // flash. Must sit ABOVE the stub check: a flash that resets the board
        // mid-read leaves SDA held and boot falls back to the stub. If it comes back, the sampler task keeps reading it (it
        // holds the same object); if we booted on the stub, say so.
        out(robot::imuReinitLive() ? "imu reinit: QMI8658C is back\r\n"
                            : "imu reinit: still not answering (power-cycle the board)\r\n");
        return;
    }
    if (!dev) {
        out("imu: running on the STUB -- the QMI8658C did not answer at boot\r\n"
            "  `imu scan` to see what is on the bus\r\n");
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "gscale")) {
        // per-axis gyro scale (SENSOR frame). `imu gscale` prints; three
        // numbers set + save to NVS. Measured with tools/imu_scale_check.py.
        if (argc >= 5) {
            float gs[3];
            for (int i = 0; i < 3; ++i) gs[i] = static_cast<float>(atof(argv[2 + i]));
            for (int i = 0; i < 3; ++i) {
                if (!(gs[i] > 0.5f && gs[i] < 2.0f)) { out("gscale: each factor must be in (0.5, 2)\r\n"); return; }
            }
            dev->setGyroScale(gs);
            out(robot::imuGyroScaleSave(gs) ? "gscale saved to NVS\r\n" : "gscale: NVS save FAILED\r\n");
        }
        float cur[3];
        dev->gyroScale(cur);
        say(out, "gyro scale (sensor frame): x %.4f y %.4f z %.4f\r\n",
            static_cast<double>(cur[0]), static_cast<double>(cur[1]), static_cast<double>(cur[2]));
        return;
    }
    if (argc >= 2 && !strcmp(argv[1], "avg")) {
        if (argc >= 3) robot::imuSetAverage(!strcmp(argv[2], "on"));
        say(out, "imu avg: %s (gyro handed to the loop is the %s)\r\n",
            robot::imuAverage() ? "on" : "off",
            robot::imuAverage() ? "tick average of ~5 samples" : "latest 4 ms sample");
        return;
    }
    if (argc >= 2 && !strcmp(argv[1], "ring")) {
        const int ms = argc >= 3 ? static_cast<int>(num(argv[2], 2000)) : 2000;
        robot::RingStats r;
        if (!robot::imuRing(ms, r)) { out("imu ring: not enough samples\r\n"); return; }
        say(out, "imu ring: last %d ms, %d samples @ %d ms; horizontal gyro peak %.3f rad/s, "
                 "RMS first/last quarter %.4f/%.4f, ~%.1f Hz\r\n",
            ms, r.n, robot::kImuPeriodMs, static_cast<double>(r.peak),
            static_cast<double>(r.rms_first), static_cast<double>(r.rms_last),
            static_cast<double>(r.f_hz));
        say(out, "  integral over the window (deg, body frame): x %.2f y %.2f z %.2f; mean rate (rad/s) %.4f %.4f %.4f\r\n",
            static_cast<double>(r.integ[0] * 57.2958f), static_cast<double>(r.integ[1] * 57.2958f),
            static_cast<double>(r.integ[2] * 57.2958f), static_cast<double>(r.mean[0]),
            static_cast<double>(r.mean[1]), static_cast<double>(r.mean[2]));
        out("  envelope (mrad/s per 50 ms):");
        for (int i = 0; i < r.nbins; ++i) say(out, " %d", static_cast<int>(r.env[i] * 1000.0f + 0.5f));
        out("\r\n");
        return;
    }
    if (argc >= 2 && !strcmp(argv[1], "raw")) {
        int n = (argc >= 3) ? atoi(argv[2]) : 1;
        if (n < 1) n = 1;
        if (n > 200) n = 200;
        out("sensor frame, mount NOT applied -- accel m/s2, gyro rad/s\r\n");
        for (int i = 0; i < n; ++i) {
            float a[3], g[3];
            if (!dev->readRaw(a, g)) { out("read failed\r\n"); return; }
            const float mag = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            say(out, "a % 7.3f % 7.3f % 7.3f  |a| %5.3f   g % 7.4f % 7.4f % 7.4f\r\n",
                static_cast<double>(a[0]), static_cast<double>(a[1]),
                static_cast<double>(a[2]), static_cast<double>(mag),
                static_cast<double>(g[0]), static_cast<double>(g[1]),
                static_cast<double>(g[2]));
            if (n > 1) vTaskDelay(pdMS_TO_TICKS(100));
        }
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "bias")) {
        if (robot::g_mode_request.load() != robot::Mode::kBench ||
            robot::g_ctrl_owns_bus.load()) {
            out("busy: `bench` first -- bias calibration writes driver state\r\n");
            return;
        }
        // Bias is orientation-dependent (MEMS g-sensitivity) and temperature-
        // dependent, so it does NOT transfer from the bench to the robot.
        // Measured on this board: calibrating flat on a desk and then bolting
        // it upright into the pelvis left 0.0187 rad/s on the sensor's y --
        // which the mount maps onto body z, i.e. straight into yaw, the one
        // axis gravity cannot correct. That was ~1 deg/s of heading drift,
        // 29 deg over a 30 s run. Recalibrating in place cut it to 0.07 deg/s.
        const imu::Mount& cur = dev->mount();
        if (cur.w == 1.0f && cur.x == 0.0f && cur.y == 0.0f && cur.z == 0.0f) {
            out("  NOTE: mount is still identity. Bias does not survive being\r\n"
                "  moved -- set the mount, then redo this in the robot's\r\n"
                "  STANDING pose, or the yaw will drift.\r\n");
        }
        out("hold the robot STILL for 2 s, in the pose it will RUN in...\r\n");
        float b[3], s[3];
        if (!dev->calibrateBias(b, s)) { out("calibration failed\r\n"); return; }
        say(out, "bias  % .5f % .5f % .5f rad/s\r\n",
            static_cast<double>(b[0]), static_cast<double>(b[1]),
            static_cast<double>(b[2]));
        say(out, "spread % .5f % .5f % .5f rad/s\r\n",
            static_cast<double>(s[0]), static_cast<double>(s[1]),
            static_cast<double>(s[2]));
        // Yaw has no gravity reference, so whatever bias survives here
        // integrates straight into heading error -- and `up` is
        // yaw-dependent. This is the number that decides whether the
        // horizontal channel is worth anything after 30 s.
        const float worst = fmaxf(fmaxf(fabsf(b[0]), fabsf(b[1])), fabsf(b[2]));
        say(out, "residual yaw drift ~%.1f deg after 30 s\r\n",
            static_cast<double>(fabsf(b[2]) * 30.0f * 57.2958f));
        // The threshold is set from the part's MEASURED noise floor on this
        // board, not from a guess. At 500 Hz ODR with the on-chip low-pass
        // off, a motionless board shows ~0.1-0.3 rad/s peak-to-peak, so a
        // tighter gate would fire every time and teach everyone to ignore it.
        // Real handling shows up well above this.
        if (s[0] > 0.6f || s[1] > 0.6f || s[2] > 0.6f) {
            out("  *** spread is large -- the robot MOVED. Redo it still. ***\r\n");
        }
        // The number that actually decides whether this calibration is worth
        // anything is the drift line above: bias stderr is sigma/sqrt(N), and
        // it lands around 3 mrad/s, i.e. a few degrees of heading per run.
        (void)worst;

        const imu::Mount& m = dev->mount();
        const float mq[4] = {m.w, m.x, m.y, m.z};
        if (robot::imuCalSave(b, mq)) {
            robot::g_imu_cal_from_nvs = true;
            out("saved to NVS -- survives a power cycle\r\n");
        } else {
            out("NVS SAVE FAILED -- this bias dies at the next reset\r\n");
        }
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "mount")) {
        if (argc < 6) {
            const imu::Mount& m = dev->mount();
            say(out, "mount % .5f % .5f % .5f % .5f (w x y z)\r\n",
                static_cast<double>(m.w), static_cast<double>(m.x),
                static_cast<double>(m.y), static_cast<double>(m.z));
            out("usage: imu mount <w> <x> <y> <z>   (sensor -> body)\r\n");
            out("  the board stands vertical/transverse in the pelvis recess,\r\n");
            out("  so the nominal from CAD is ~90 deg about y: 0.7071 0 0.7071 0\r\n");
            return;
        }
        float q[4];
        for (int i = 0; i < 4; ++i) q[i] = strtof(argv[2 + i], nullptr);
        const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (n < 0.5f) { out("not a rotation (zero quaternion)\r\n"); return; }
        for (int i = 0; i < 4; ++i) q[i] /= n;
        imu::Mount m;
        m.w = q[0]; m.x = q[1]; m.y = q[2]; m.z = q[3];
        dev->setMount(m);
        dev->realign();          // the old estimate was in the old body frame
        float bias[3];
        dev->bias(bias);
        if (robot::imuCalSave(bias, q)) {
            robot::g_imu_cal_from_nvs = true;
            say(out, "mount % .5f % .5f % .5f % .5f saved\r\n",
                static_cast<double>(q[0]), static_cast<double>(q[1]),
                static_cast<double>(q[2]), static_cast<double>(q[3]));
        } else {
            out("NVS SAVE FAILED\r\n");
        }
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "ae")) {
        imu::Qmi8658Imu::AeDiag d;
        if (!dev->aeDiagnose(d)) { out("diagnose failed\r\n"); return; }
        say(out, "ctrl write %s   ctrl6 0x%02X (want 0x80)   ctrl7 0x%02X "
                 "(want 0x0B)   status0 0x%02X\r\n",
            d.ctrl_written ? "ok" : "FAILED", d.ctrl6, d.ctrl7, d.status0);
        out("dQ ");
        for (int i = 0; i < 8; ++i) say(out, "%02X ", d.dq_bytes[i]);
        out("\r\ndV ");
        for (int i = 0; i < 6; ++i) say(out, "%02X ", d.dv_bytes[i]);
        // Non-zero is NOT the same as valid, and the difference is the whole
        // point of this command. Decode with the documented scalings and let
        // the numbers speak: a stationary board must give dQ ~ (1,0,0,0) and
        // |dV| ~ g*dt, i.e. a few tenths of a m/s.
        float dq[4], dv[3], n = 0.0f;
        for (int i = 0; i < 4; ++i) {
            const int16_t r = static_cast<int16_t>(
                static_cast<uint16_t>(d.dq_bytes[i * 2]) |
                (static_cast<uint16_t>(d.dq_bytes[i * 2 + 1]) << 8));
            dq[i] = static_cast<float>(r) / 16384.0f;
            n += dq[i] * dq[i];
        }
        n = sqrtf(n);
        for (int i = 0; i < 3; ++i) {
            const int16_t r = static_cast<int16_t>(
                static_cast<uint16_t>(d.dv_bytes[i * 2]) |
                (static_cast<uint16_t>(d.dv_bytes[i * 2 + 1]) << 8));
            dv[i] = static_cast<float>(r) / 1024.0f;
        }
        say(out, "\r\ndQ Q14 % .4f % .4f % .4f % .4f  |dQ| %.4f (want 1.0)\r\n",
            static_cast<double>(dq[0]), static_cast<double>(dq[1]),
            static_cast<double>(dq[2]), static_cast<double>(dq[3]),
            static_cast<double>(n));
        say(out, "dV m/s % .3f % .3f % .3f  (want ~0.2 at rest)\r\n",
            static_cast<double>(dv[0]), static_cast<double>(dv[1]),
            static_cast<double>(dv[2]));
        if (!d.any_nonzero) {
            out("verdict: registers all zero -- AE produced nothing\r\n");
        } else if (n > 0.9f && n < 1.1f) {
            out("verdict: VALID unit quaternion -- AE works, use it\r\n");
        } else {
            out("verdict: non-zero but NOT a unit quaternion under the\r\n"
                "  documented Q14 scaling. The register contents do not mean\r\n"
                "  what rev 0.6 says they mean on this silicon (rev 0x7C vs\r\n"
                "  the 0x79 documented). Raw path stands.\r\n");
        }
        return;
    }

    if (argc >= 2 && !strcmp(argv[1], "forget")) {
        say(out, "%s\r\n", robot::imuCalErase() ? "erased" : "erase failed");
        robot::g_imu_cal_from_nvs = false;
        return;
    }

    // Default: status.
    imu::Qmi8658Imu::Probe p;
    dev->probe(p);
    say(out, "driver   %s\r\n", dev->name());
    say(out, "who_am_i 0x%02X (expect 0x05)   rev 0x%02X   addr 0x%02X\r\n",
        p.who_am_i, p.revision, p.addr_found);

    imu::Sample s{};
    dev->read(s);
    say(out, "up       % .4f % .4f % .4f   (body z, yaw stripped = sim framezaxis at yaw 0)\r\n",
        static_cast<double>(s.up[0]), static_cast<double>(s.up[1]),
        static_cast<double>(s.up[2]));
    say(out, "gyro     % .4f % .4f % .4f rad/s\r\n",
        static_cast<double>(s.gyro[0]), static_cast<double>(s.gyro[1]),
        static_cast<double>(s.gyro[2]));
    float b[3];
    dev->bias(b);
    say(out, "bias     % .5f % .5f % .5f rad/s%s\r\n",
        static_cast<double>(b[0]), static_cast<double>(b[1]),
        static_cast<double>(b[2]),
        robot::g_imu_cal_from_nvs ? "   (from NVS)"
                                  : "   *** NOT CALIBRATED: run `imu bias` ***");
    const imu::Mount& mt = dev->mount();
    say(out, "mount    % .4f % .4f % .4f % .4f%s\r\n",
        static_cast<double>(mt.w), static_cast<double>(mt.x),
        static_cast<double>(mt.y), static_cast<double>(mt.z),
        (mt.w == 1.0f && mt.x == 0.0f && mt.y == 0.0f && mt.z == 0.0f)
            ? "   (identity -- not yet measured on the robot)" : "");
    say(out, "levelled %s   accel-reject streak %d\r\n",
        dev->fusion().levelled() ? "yes" : "NO (gyro only)",
        dev->fusion().rejectStreak());

    // The filter's own state, so a mount argument can be settled by looking
    // rather than by reasoning about what the fusion "should" be doing.
    // projgrav is what the accelerometer sees in the BODY frame; `up` is its
    // transpose-partner. For a forward lean projgrav.x must go NEGATIVE and
    // up.x POSITIVE -- if they do not have opposite signs, the mount is wrong.
    float q[4];
    dev->fusion().quat(q);
    float pg[3];
    imu::projectedGravityFromQuaternion(q[0], q[1], q[2], q[3], pg);
    say(out, "quat     % .4f % .4f % .4f % .4f\r\n",
        static_cast<double>(q[0]), static_cast<double>(q[1]),
        static_cast<double>(q[2]), static_cast<double>(q[3]));
    say(out, "projgrav % .4f % .4f % .4f   (world +z in BODY frame)\r\n",
        static_cast<double>(pg[0]), static_cast<double>(pg[1]),
        static_cast<double>(pg[2]));

    // Measure what this actually costs the tick. firmware-design section 4
    // budgeted ~1.0 ms for the BNO085's SH-2 reports and never got to check
    // it, because no IMU was ever fitted. This one is fitted.
    const int64_t t0 = esp_timer_get_time();
    constexpr int kN = 50;
    for (int i = 0; i < kN; ++i) {
        imu::Sample tmp{};
        dev->read(tmp);
    }
    const int64_t t1 = esp_timer_get_time();
    say(out, "read     %.3f ms mean of %d (I2C burst + fusion), 20 ms tick\r\n",
        static_cast<double>(t1 - t0) / (kN * 1000.0), kN);
}

// -- ntp -------------------------------------------------------------------
//
// The robot's wall clock (timesync.h): is it synced, to whom, how long ago,
// and by how much did the last reply move it. Logging is silenced after boot
// (app_main), so this line is the only place SNTP can be seen from -- and the
// thing to type after a flash before trusting a single `t_us` in a log.
void cmdNtp(Sink out, int argc, char**) {
    if (argc >= 2) { out("usage: ntp   (status only; server comes from DHCP)\r\n"); return; }
    robot::TimeStatus ts;
    robot::timeGetStatus(ts);
    if (!ts.started) {
        out("ntp: not started -- no DHCP lease yet (`wifi`)\r\n");
        return;
    }
    if (!ts.synced) {
        say(out, "ntp: UNSYNCED, waiting for a reply from `%s` / `%s`; "
                 "telemetry t_us = 0\r\n",
            ts.server0[0] ? ts.server0 : "(no DHCP server)", ts.server1);
        return;
    }
    const uint64_t s = ts.now_us / 1000000ull;
    const unsigned us = static_cast<unsigned>(ts.now_us % 1000000ull);
    // Broken down by hand: newlib's gmtime is fine but strftime drags in a
    // locale table, and this is one line.
    const uint64_t days = s / 86400ull;
    const unsigned sod = static_cast<unsigned>(s % 86400ull);
    // Civil-from-days (Howard Hinnant), valid for the years this robot lives.
    const int64_t z = static_cast<int64_t>(days) + 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t y = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;
    say(out, "ntp: synced  %04lld-%02u-%02uT%02u:%02u:%02u.%06uZ  "
             "(t_us %llu)\r\n",
        static_cast<long long>(m <= 2 ? y + 1 : y), m, d, sod / 3600,
        (sod / 60) % 60, sod % 60, us,
        static_cast<unsigned long long>(ts.now_us));
    say(out, "  server `%s` (dhcp), fallback `%s`; %lu replies, last %lld s "
             "ago, moved the clock %ld ms\r\n",
        ts.server0[0] ? ts.server0 : "(none)", ts.server1,
        static_cast<unsigned long>(ts.syncs),
        static_cast<long long>(ts.since_sync_us / 1000000),
        static_cast<long>(ts.last_step_ms));
}

// -- wifi ------------------------------------------------------------------
//
// Provisioning and status for the UDP link (wifi_link.h). Credential writes
// are bench-only for the same reason `cal save` is: an NVS commit is a flash
// write, a flash write stalls both cores' cache, and a stalled control tick
// past 2x budget trips the torque release.
void cmdWifi(Sink out, int argc, char** argv) {
    if (argc >= 2 && !strcmp(argv[1], "clear")) {
        if (robot::g_mode_request.load() != robot::Mode::kBench) {
            out("busy: NVS writes stall the control tick -- `bench` first\r\n");
            return;
        }
        say(out, "%s\r\n", robot::wifiCredsErase()
                               ? "credentials erased (radio stays up until "
                                 "reboot)"
                               : "erase failed");
        return;
    }
    if (argc >= 3) {
        if (robot::g_mode_request.load() != robot::Mode::kBench) {
            out("busy: NVS writes stall the control tick -- `bench` first\r\n");
            return;
        }
        if (!robot::wifiCredsSave(argv[1], argv[2])) {
            out("NVS save failed\r\n");
            return;
        }
        say(out, "saved; joining `%s` ...\r\n", argv[1]);
        if (!robot::wifiApply(argv[1], argv[2])) {
            out("driver not up -- credentials stored, reboot to join\r\n");
        }
        return;
    }
    if (argc >= 2) {
        out("usage: wifi | wifi <ssid> <psk> | wifi clear   (no spaces in "
            "either)\r\n");
        return;
    }

    robot::WifiStatus w;
    robot::wifiGetStatus(w);
    if (!w.driver_up) { out("wifi: driver not up (init failed at boot)\r\n"); return; }
    if (!w.have_creds) {
        out("wifi: no credentials -- `wifi <ssid> <psk>` (bench mode)\r\n");
        return;
    }
    say(out, "wifi: %s `%s`", w.connected ? "CONNECTED to" : "joining", w.ssid);
    if (w.connected) {
        say(out, "  ip %lu.%lu.%lu.%lu  rssi %d dBm",
            static_cast<unsigned long>((w.ip >> 24) & 0xFF),
            static_cast<unsigned long>((w.ip >> 16) & 0xFF),
            static_cast<unsigned long>((w.ip >> 8) & 0xFF),
            static_cast<unsigned long>(w.ip & 0xFF), w.rssi);
    }
    out("\r\n");
    // tlm tx counts both lengths; `pose` is how many were the 40 B mirror-mode
    // frame, and whether the current commander is asking for it (FLAG_POSE).
    // The bench check in docs/mirror-mode.md reads this line: with mirror
    // OFF the pose count must not move.
    say(out, "  cmd rx %lu  bad %lu  tlm tx %lu (pose %lu, %s)  disconnects %lu\r\n",
        static_cast<unsigned long>(w.rx_frames),
        static_cast<unsigned long>(w.rx_bad),
        static_cast<unsigned long>(w.tx_tlm),
        static_cast<unsigned long>(w.tx_tlm_ext),
        w.pose_wanted ? "asked" : "not asked",
        static_cast<unsigned long>(w.disconnects));
    if (w.peer_ip) {
        say(out, "  commander %lu.%lu.%lu.%lu\r\n",
            static_cast<unsigned long>((w.peer_ip >> 24) & 0xFF),
            static_cast<unsigned long>((w.peer_ip >> 16) & 0xFF),
            static_cast<unsigned long>((w.peer_ip >> 8) & 0xFF),
            static_cast<unsigned long>(w.peer_ip & 0xFF));
    } else {
        out("  commander: none yet (nothing received on 4210)\r\n");
    }
}

void banner(Sink out) {
    out("\r\nbimo firmware v1 -- bench mode (nothing moves until `run`)\r\n");
    out("  scan                 ping IDs 0-253, report position/voltage/faults\r\n");
    out("  ping [id]            no id = check exactly the policy's servos\r\n");
    out("  id <old> <new>       assign a servo ID (EEPROM, one servo on the bus)\r\n");
    out("  pos <id>             position, speed, load, voltage, temp, faults\r\n");
    out("  move <id> <ticks> [ms] [steps/s]   2048 == middle, 4096 ticks/rev\r\n");
    out("  pose <t0..t9> [s<ms>|t<ms>|steps/s]  all 10 targets (joint order);\r\n"
        "                       default = SMOOTH min-jerk (auto duration); s<ms> sets it,\r\n"
        "                       tokens h<pct> a<acc> b<ticks> i<ms>; t<ms> unison; N = const speed\r\n");
    out("  reg <id> <addr> [1|2]  READ a servo register (PID 21-23, deadzone 26/27, acc 41)\r\n");
    out("  home [steps/s]       every joint to its calibrated zero (the stand),\r\n");
    out("                       torque ON and holding -- works after a fall\r\n");
    out("  release [id] | torque [id]   no id = broadcast\r\n");
    out("  middle <id>          latch the current angle as 2048, torque off\r\n");
    out("  volt                 pack voltage, read off the servos (no board ADC)\r\n");
    out("  batt [reset]         under-voltage guard state; reset after a pack swap\r\n");
    out("  cal [show|zero|dir|set|migrate|save|load|reset]   zero + dir (NVS)\r\n");
    out("  shape [hz]           C2 command-shaping pole; 0 = off (raw/jerky)\r\n");
    out("  obsfreeze [none|up|gyro|imu|dq]  bench: freeze policy-obs parts at nominal\r\n");
    out("  run | bench          hand the bus to / take it back from the loop\r\n");
    out("  imu [scan|raw [n]|ring [ms]|avg [on|off]|gscale [x y z]|reinit|bias|mount|forget]\r\n");
    out("  wifi [<ssid> <psk>|clear]   UDP link status / credentials (NVS)\r\n");
    out("  ntp                  wall clock: SNTP sync state (server from DHCP)\r\n");
    out("  stat                 tick timing and fault counters\r\n");
    out("  obsdump [on|off|once]   stream the policy's observation as CSV\r\n");
}

void linkMode(bool run, Sink out) {
    out(run ? "link: arm -> " : "link: disarm -> ");
    cmdMode(out, run);
}

void linkHome(Sink out) {
    out("link: reset servos -> ");
    // The move needs the bus, and the CLI only owns the bus while the loop is
    // benched -- so end the run first. This is a disarm in every respect,
    // including that re-arming afterwards is a fresh, deliberate ARM edge.
    robot::g_mode_request.store(robot::Mode::kBench);
    // ctrl hands the bus back on its next tick (20 ms). Wait for it rather
    // than failing the operator's request into claimBus's "busy:".
    for (int i = 0; i < 40 && robot::g_ctrl_owns_bus.load(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // Publish the VERDICT, not the intention. This atomic is the only way the
    // outcome reaches an operator who is holding a laptop and not a tether,
    // and a reset that quietly did nothing -- no calibration, a latched pack,
    // a servo that did not answer -- must say which, or it looks exactly like
    // a dead button.
    robot::g_arm_result.store(
        static_cast<uint8_t>(homeAll(out, kHomeStepsPerSec)));
}

void execute(const char* line, Sink out) {
    char buf[96];
    strncpy(buf, line, sizeof buf - 1);
    buf[sizeof buf - 1] = 0;
    // 24 tokens: `pose` + 10 targets + mode + up to a dozen tuning tokens
    // (h/a/b/i/f/z/T). It was 13 until 2026-09-03, which silently dropped
    // every tuning token after the first -- three "A/B/C" bench comparisons
    // were run on identical settings before this was found.
    char* argv[24];
    const int argc = split(buf, argv, 24);
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
    else if (!strcmp(c, "reg")) cmdReg(out, argc, argv);
    else if (!strcmp(c, "trace")) traceDump(out);
    else if (!strcmp(c, "home")) cmdHome(out, argc, argv);
    else if (!strcmp(c, "shape")) cmdShape(out, argc, argv);
    else if (!strcmp(c, "obsfreeze")) cmdObsFreeze(out, argc, argv);
    else if (!strcmp(c, "volt")) cmdVolt(out);
    else if (!strcmp(c, "batt")) cmdBatt(out, argc, argv);
    else if (!strcmp(c, "run")) cmdMode(out, true);
    else if (!strcmp(c, "bench")) cmdMode(out, false);
    else if (!strcmp(c, "cal")) cmdCal(out, argc, argv);
    else if (!strcmp(c, "imu")) cmdImu(out, argc, argv);
    else if (!strcmp(c, "wifi")) cmdWifi(out, argc, argv);
    else if (!strcmp(c, "ntp")) cmdNtp(out, argc, argv);
    else if (!strcmp(c, "stat")) cmdStat(out);
    else if (!strcmp(c, "obsdump")) cmdObsDump(out, argc, argv);
    else out("? (try `help`)\r\n");
}

}  // namespace cli
