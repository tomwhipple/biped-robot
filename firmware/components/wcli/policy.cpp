// The Wi-Fi CLI allowlist. See wcli/wcli.h (classify) and docs/wifi-cli.md
// ("What is refused over Wi-Fi") -- keep that table and this file in step.
//
// Default DENY. Every command word the CLI knows is named below, either
// allowed (possibly only some of its forms) or refused with the reason; any
// other word is refused as unknown. So a command added to cli.cpp later is
// tether-only until it is added here on purpose.
#include <stdio.h>
#include <string.h>

#include "wcli/wcli.h"

namespace wcli {
namespace {

constexpr int kMaxArgs = 8;   // only the first few tokens decide anything

struct Tokens {
    char buf[kMaxLine + 1];
    const char* argv[kMaxArgs];
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
    "balance", "reg", "sread", "trace", "home", "shape", "obsfreeze", "volt",
    "stat", "ntp", "bno", "rec", "bench", nullptr};

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
        return tether("`batt reset` clears the pack-guard latch: swap the pack "
                      "at the robot, then reset it on the tether (or reboot)");
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
        static const char* const kImuRead[] = {"scan", "raw", "ring", "avg",
                                               "ae", "reinit", nullptr};
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
            return tether("`imu bias` calibrates and saves the gyro bias (NVS)");
        }
        if (is(sub, "forget")) {
            return tether("`imu forget` erases the IMU calibration (NVS)");
        }
        return Decision{Verdict::kRefuseUnknown, "unknown `imu` form"};
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
