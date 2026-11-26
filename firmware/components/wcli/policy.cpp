// The Wi-Fi CLI allowlist. See wcli/wcli.h (classify) and docs/wifi-cli.md
// ("What is refused over Wi-Fi") -- keep that table and this file in step.
//
// Default DENY. Every command word the CLI knows is named below, either
// allowed (possibly only some of its forms) or refused with the reason; any
// other word is refused as unknown. So a command added to cli.cpp later is
// tether-only until it is added here on purpose.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wcli/wcli.h"

namespace wcli {
namespace {

constexpr int kMaxArgs = 8;   // only the first few tokens decide anything

struct Tokens {
    char buf[kMaxLine + 1] = {};
    const char* argv[kMaxArgs] = {};
    int argc = 0;    // tokens seen (capped at kMaxArgs + 1: "more than")
};

void tokenize(const char* line, Tokens& t) {
    strncpy(t.buf, line, sizeof t.buf - 1);
    t.buf[sizeof t.buf - 1] = 0;
    t.argc = 0;
    char* s = t.buf;
    while (*s) {
        while (*s == ' ' || *s == '\t') ++s;
        if (!*s) break;
        if (t.argc < kMaxArgs) t.argv[t.argc] = s;
        if (t.argc <= kMaxArgs) ++t.argc;
        while (*s && *s != ' ' && *s != '\t') ++s;
        if (*s) *s++ = 0;
    }
}

bool is(const char* a, const char* b) { return strcmp(a, b) == 0; }

bool oneOf(const char* w, const char* const* set) {
    for (; *set != nullptr; ++set) {
        if (is(w, *set)) return true;
    }
    return false;
}

Decision allow() { return Decision{Verdict::kAllow, ""}; }
Decision tether(const char* why) { return Decision{Verdict::kRefuseTether, why}; }

// Allowed with any arguments while benched. Each one already runs behind the
// same gates on the tether (claimBus: bench mode and the bus handed back;
// torqueGate; the mechanical-envelope clamp; balLanding) -- the line goes
// through the very same cli::execute().
const char* const kBenchAny[] = {
    "help", "?", "scan", "ping", "pos", "move", "release", "torque", "pose",
    "balance", "reg", "sread", "trace", "home", "shape", "limit", "obsfreeze", "volt",
    "stat", "ntp", "bno", "rec", "bench",
    // controller settings for the NEXT run, like shape / limit / obsfreeze (2026-11-06: the robot
    // runs on battery with no tether, and standclock resets to `run` at every boot)
    "standclock", "heading",
    // the stance-hip trim (#297): status, gains and learned T are RAM only, like `balance`
    "trim",
    // the gait track (#297 B2): frames, play / stop / halt; RAM only, behind the same gates as `pose`
    "track",
    // the track's lift gate (#297): thresholds and on / off, RAM only, like `trim`
    "liftgate", nullptr};

}  // namespace

Decision classify(const char* line, bool armed) {
    for (const char* p = line; *p; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if ((c < 0x20 && c != '\t') || c > 0x7E) {
            return Decision{Verdict::kRefuseBadLine,
                            "control or non-ASCII bytes in the line"};
        }
    }
    if (strlen(line) > kMaxLine) {
        return Decision{Verdict::kRefuseBadLine, "line too long"};
    }
    Tokens t;
    tokenize(line, t);
    if (t.argc == 0) return allow();   // an empty line does nothing anywhere
    const char* c = t.argv[0];
    const int argc = t.argc;
    const char* sub = argc >= 2 ? t.argv[1] : "";

    if (armed) {
        // Armed, the bus and the robot are the control loop's. Only pure
        // status reads that touch neither are allowed; everything else is
        // the commander's business (or the tether's).
        const bool status =
            is(c, "help") || is(c, "?") || is(c, "stat") || is(c, "ntp") ||
            (argc == 1 && (is(c, "wifi") || is(c, "batt") || is(c, "rec") ||
                           is(c, "cal") || is(c, "balance") || is(c, "wcli"))) ||
            (argc == 2 && is(c, "rec") && is(sub, "status")) ||
            (argc == 2 && is(c, "cal") && is(sub, "show"));
        if (status) return allow();
        return Decision{Verdict::kRefuseArmed,
                        "the control loop is armed and the Wi-Fi CLI is "
                        "bench-only"};
    }

    if (oneOf(c, kBenchAny)) return allow();

    // -- allowed in some forms only ------------------------------------------
    if (is(c, "batt")) {
        if (argc == 1) return allow();
        return tether("`batt reset` clears the pack-guard latch (swap the pack "
                      "at the robot, then reset it on the tether, or reboot); "
                      "`batt pack` changes what the guard reads");
    }
    if (is(c, "gains")) {
        if (argc <= 2) return allow();   // the table, or one servo's read
        return tether("`gains <id> <P> <D>` writes servo EEPROM");
    }
    if (is(c, "cal")) {
        if (argc == 1 || (argc == 2 && is(sub, "show"))) return allow();
        return tether("`cal` changes the calibration (live or NVS)");
    }
    if (is(c, "wifi")) {
        if (argc == 1) return allow();
        return tether("`wifi <ssid> <psk>` / `wifi clear` write the Wi-Fi "
                      "credentials (NVS)");
    }
    if (is(c, "wcli")) {
        if (argc == 1) return allow();
        return tether("`wcli key` manages the Wi-Fi CLI key itself");
    }
    if (is(c, "obsdump")) {
        if (argc == 1 || is(sub, "show") || is(sub, "off")) return allow();
        return tether("the observation dump streams to the tether UART only");
    }
    if (is(c, "stream")) {
        if (argc == 2 && (is(sub, "status") || is(sub, "dump"))) return allow();
        return tether("`stream` drives a servo with no torque gate and no "
                      "envelope clamp (a bench experiment tool)");
    }
    if (is(c, "imu")) {
        // `time`: the 50-read timing (refused by the firmware while the bench
        // loop plays: it runs on that loop's task, #297)
        static const char* const kImuRead[] = {"scan", "raw", "ring", "avg",
                                               "ae", "reinit", "time", nullptr};
        if (argc == 1 || oneOf(sub, kImuRead)) return allow();
        if (is(sub, "gscale")) {
            if (argc < 5) return allow();
            return tether("`imu gscale <x> <y> <z>` saves the gyro scale to NVS");
        }
        if (is(sub, "mount")) {
            if (argc < 6) return allow();
            return tether("`imu mount <w> <x> <y> <z>` saves the IMU mount to NVS");
        }
        if (is(sub, "bias")) {
            // the online stand-still bias: status and on / off are RAM; `save` writes NVS
            if (argc >= 3 && is(t.argv[2], "online") && (argc == 3 || (argc == 4 && !is(t.argv[3], "save")))) {
                return allow();
            }
            return tether("`imu bias` calibrates and saves the gyro bias (NVS)");
        }
        if (is(sub, "forget")) {
            return tether("`imu forget` erases the IMU calibration (NVS)");
        }
        return Decision{Verdict::kRefuseUnknown, "unknown `imu` form"};
    }

    if (is(c, "rail")) {
        // Power lite's servo rail (Tom 2026-11-21: "turning it on via wifi should be
        // allowed"). Status, on and off only; every interlock is behind cli::execute()
        // and railguard, which do not know the transport.
        if (argc == 1 || (argc == 2 && (is(sub, "on") || is(sub, "off")))) return allow();
        if (argc == 2 && is(sub, "clear")) {
            return tether("`rail clear` acknowledges a STUCK RAIL fault (check K1 "
                          "at the robot first)");
        }
        if (argc == 2 && is(sub, "panic")) {
            return tether("`rail panic` aborts the firmware on purpose (the "
                          "fail-safe's bench check)");
        }
        return Decision{Verdict::kRefuseUnknown, "unknown `rail` form"};
    }
    if (is(c, "fall")) {
        if (argc == 1) return allow();
        return tether("`fall on|off|autohome|clear` switches the fall guard (a "
                      "safety latch)");
    }
    if (is(c, "flail")) {
        if (argc == 1) return allow();
        return tether("`flail on|off` switches the armed loop's flail guard (a "
                      "safety latch)");
    }
    if (is(c, "sweep")) {
        if (argc == 2 && is(sub, "stop")) return allow();
        return tether("`sweep` is a bench identification experiment (roll "
                      "sweep), run from the tether");
    }
    if (is(c, "servocfg")) {
        if (argc == 1) return allow();   // read and verify regs 85/81 only
        return tether("`servocfg fast|factory` rewrites the STS3215 response "
                      "registers (servo RAM)");
    }
    if (is(c, "regw")) {
        return tether("`regw` writes a servo register (an actuator "
                      "experiment tool)");
    }
    if (is(c, "stepresp")) {
        return tether("`stepresp` is a bench identification experiment (one "
                      "servo's step response), run from the tether");
    }

    // -- never over Wi-Fi --------------------------------------------------------
    if (is(c, "id")) return tether("`id` assigns a servo ID (servo EEPROM)");
    if (is(c, "middle")) {
        return tether("`middle` re-zeroes a servo (servo EEPROM)");
    }
    if (is(c, "run")) {
        return tether("arming is the commander's ARM edge (or the tether's "
                      "`run`), never a Wi-Fi CLI line");
    }
    return Decision{Verdict::kRefuseUnknown, "not on the Wi-Fi allowlist"};
}

const char* benchLoopRefusal(const char* line) {
    Tokens t;
    tokenize(line, t);
    if (t.argc == 0) return nullptr;
    const char* c = t.argv[0];
    const int argc = t.argc;
    const char* sub = argc >= 2 ? t.argv[1] : "";

    // The loop's own commands: status, add, steps, stop, halt -- RAM only.
    if (is(c, "track")) return nullptr;
    // Stop / disarm / safety: never refused. One bus write at most
    // (`release`: a broadcast torque-off), or a RAM latch.
    static const char* const kSafety[] = {"release", "bench", "fall", "flail",
                                          "home", nullptr};
    if (oneOf(c, kSafety)) return nullptr;
    if ((is(c, "stream") || is(c, "sweep")) && is(sub, "stop")) return nullptr;
    if (is(c, "stream") && argc == 2 && is(sub, "status")) return nullptr;
    if (is(c, "rail")) {
        if (argc == 1 || is(sub, "off") || is(sub, "panic") || is(sub, "clear")) return nullptr;
        return "`rail on` waits for the rail to come up (20+10+5 ms polls)";
    }
    // Take the bench loop over (`pose` refuses itself while a track plays).
    if (is(c, "balance") && is(sub, "log")) {
        return "`balance log` prints the whole balance log (one line per 40 ms sample)";
    }
    if (is(c, "pose") || is(c, "balance")) return nullptr;
    // `trim off` / `balance off` fade; every `trim` form is RAM.
    // RAM settings and status prints with no bus, I2C or NVS: well under 1 ms
    // of work, the print queued (Wi-Fi ring 6.4 KB; tether 512 B buffer).
    static const char* const kRam[] = {"trim", "liftgate", "heading", "stat", "ntp",
                                       "shape", "limit", "standclock", "obsfreeze",
                                       nullptr};
    if (oneOf(c, kRam)) return nullptr;
    // One or two bus transactions: <= 3 ms each (the bus's reply timeout).
    // `torque` is one (broadcast) write; `move` a torque check and one write.
    if (is(c, "reg") || is(c, "pos") || is(c, "torque") || is(c, "move")) return nullptr;
    if (is(c, "ping")) {
        if (argc >= 2) return nullptr;
        return "`ping` with no id pings every fitted servo (3 ms per missing one)";
    }
    if (is(c, "imu")) {
        if (argc == 1) return nullptr;                     // the sampler's latest sample
        if (is(sub, "ring") || is(sub, "avg")) return nullptr;
        if (is(sub, "raw")) {
            if (argc == 2 || atoi(t.argv[2]) <= 1) return nullptr;   // one read, ~2 ms
            return "`imu raw <n>` waits 100 ms between samples";
        }
        if (is(sub, "gscale")) {
            if (argc < 5) return nullptr;
            return "`imu gscale <x> <y> <z>` writes NVS (flash erase: tens of ms)";
        }
        if (is(sub, "mount")) {
            if (argc < 6) return nullptr;
            return "`imu mount <w> <x> <y> <z>` writes NVS (flash erase: tens of ms)";
        }
        if (is(sub, "bias")) {
            if (argc >= 3 && is(t.argv[2], "online") && !(argc >= 4 && is(t.argv[3], "save"))) return nullptr;
            return "`imu bias` calibrates for seconds / writes NVS";
        }
        if (is(sub, "time")) return "`imu time` reads the IMU 50 times (~115 ms)";
        if (is(sub, "ae")) return "`imu ae` reads the IMU for seconds";
        if (is(sub, "scan")) return "`imu scan` probes all 127 I2C addresses";
        if (is(sub, "reinit")) return "`imu reinit` recovers and re-inits the I2C bus";
        if (is(sub, "forget")) return "`imu forget` erases NVS";
        return "unknown `imu` form";
    }
    if (is(c, "batt")) {
        if (argc == 1 || is(sub, "pack")) return nullptr;
        return "`batt reset` waits 60 ms for the control task";
    }
    if (is(c, "rec")) {
        if (argc == 1 || is(sub, "status")) return nullptr;
        return "`rec on|off|idle|<ip>` saves the recorder's config to NVS (from the tether, benched)";
    }
    if (is(c, "obsdump")) {
        if (is(sub, "off")) return nullptr;
        return "`obsdump` prints a CSV line every tick on this task";
    }
    if (is(c, "wifi") || is(c, "wcli")) {
        if (argc == 1) return nullptr;
        return "it writes NVS (flash erase: tens of ms)";
    }

    if (is(c, "help") || is(c, "?")) return "the help text is ~4.4 KB (~340 ms on the 115200-baud tether)";
    if (is(c, "scan")) return "`scan` pings IDs 0-253 (3 ms per missing one: ~0.8 s)";
    if (is(c, "trace")) return "the trace dump prints up to 400 lines (~8 KB)";
    if (is(c, "cal")) return "`cal` prints 17 joints (~0.9 KB: ~30 ms on the tether), reads every servo or writes NVS";
    if (is(c, "gains")) return "`gains` reads P/D of every servo (or writes EEPROM: 2 s)";
    if (is(c, "servocfg")) return "`servocfg` reads/writes the response registers of every servo";
    if (is(c, "volt")) return "`volt` reads the INA219 and every servo's voltage";
    if (is(c, "run")) return "arming re-reads every servo's gains and settles";
    if (is(c, "nettime")) return "`nettime` runs the policy forward pass n times";
    if (is(c, "bno")) return "`bno` waits on the BNO085 (5-25 ms polls)";
    if (is(c, "sread") || is(c, "stepresp")) return "a servo identification run (tens of ms to seconds)";
    if (is(c, "id") || is(c, "middle") || is(c, "regw")) return "it writes servo EEPROM (2 s commit wait)";
    if (is(c, "stream")) return "`stream` starts a servo stream task / dumps its whole log";
    if (is(c, "sweep")) return "`sweep` starts a roll identification sweep on this task";
    return "not known to return within 10 ms";
}

void redactForLog(const char* line, char* out, size_t cap) {
    if (cap == 0) return;
    Tokens t;
    tokenize(line, t);
    const char* keep = nullptr;   // how much of the line survives, as text
    if (t.argc >= 3 && is(t.argv[0], "wcli") && is(t.argv[1], "key") &&
        !is(t.argv[2], "clear")) {
        keep = "wcli key <redacted>";
    }
    if (keep != nullptr) {
        snprintf(out, cap, "%s", keep);
        return;
    }
    if (t.argc >= 3 && is(t.argv[0], "wifi")) {
        // The SSID is broadcast anyway; the PSK is the secret.
        snprintf(out, cap, "wifi %s <redacted>", t.argv[1]);
        return;
    }
    snprintf(out, cap, "%s", line);
}

size_t formatRefusal(const Decision& d, const char* line, char* out,
                     size_t cap) {
    if (cap == 0) return 0;
    // The command word only (never echo arguments: they may be long, and
    // a bad line may be what got it refused).
    char word[24] = {};
    size_t i = 0;
    while (*line == ' ' || *line == '\t') ++line;
    while (line[i] > 0x20 && line[i] < 0x7F && i < sizeof word - 1) {
        word[i] = line[i];
        ++i;
    }
    int n = 0;
    switch (d.verdict) {
        case Verdict::kAllow:
            out[0] = 0;
            return 0;
        case Verdict::kRefuseArmed:
            n = snprintf(out, cap, "refused over Wi-Fi: %s (disarm from the "
                         "commander, or use the tether)\r\n", d.why);
            break;
        case Verdict::kRefuseTether:
            n = snprintf(out, cap, "refused over Wi-Fi (tether only): %s\r\n",
                         d.why);
            break;
        case Verdict::kRefuseUnknown:
            n = snprintf(out, cap, "refused over Wi-Fi: `%s` -- %s (tether "
                         "only, or a typo: `help`)\r\n", word, d.why);
            break;
        case Verdict::kRefuseBadLine:
            n = snprintf(out, cap, "refused over Wi-Fi: %s\r\n", d.why);
            break;
    }
    if (n < 0) {
        out[0] = 0;
        return 0;
    }
    return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

}  // namespace wcli
