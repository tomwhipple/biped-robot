// bimo console: single-keystroke control of the robot over the WiFi link.
//
//   make -C firmware/host tui
//   firmware/host/build/bimo_tui --host 192.168.2.90
//
// Every byte this sends is produced by firmware/components/linkproto -- the
// same translation units the ESP32 decodes with -- so the client and the
// robot cannot disagree about the protocol. The Python link/ tools are the
// reference and the twins; this is the operator's seat.
//
// Keys:
//   a          arm / disarm (ARM bit edge -> the robot's `run` / `bench`)
//   up / down  walk forward / backward while held
//   left/right turn left (+wz, CCW) / right (-wz) while held
//   space, s   stand (also clears a latched E-stop)
//   e          E-STOP: latching torque release on the robot
//   q          quit: disarms first, so a closed console leaves a limp robot
//
// "While held" on a terminal: there is no key-up event, only auto-repeat. A
// motion key starts (or extends) a hold that expires --hold-ms after the LAST
// repeat, so the robot stops that long after the key is released. The hold
// must outlast the terminal's initial repeat delay or the robot stutters at
// the start of every press; the measured delay is shown on screen so
// --hold-ms can be set from a number rather than a guess. Every pending key
// is drained each tick -- a queue of stale repeats would otherwise keep a
// released key "held".
//
// The robot is authoritative about its mode: if it reports BENCH while this
// console believes it armed (arm refused for want of a calibration, or the
// robot rebooted), the console drops back to disarmed and says so.
#include <arpa/inet.h>
#include <errno.h>
#include <ncurses.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "linkproto/protocol.h"

namespace {

using namespace linkproto;

constexpr double kDefaultHoldMs = 700.0;   // > any common initial repeat delay
constexpr double kDefaultVx = 0.4;         // eval_precision line_1m's speed
constexpr double kDefaultWz = 0.5;         // eval_commands validates +-0.5
constexpr double kArmSyncMs = 1500.0;      // BENCH this long after arming -> refused
constexpr double kTlmLostMs = 1000.0;      // no beacon this long -> "link lost"

enum class Motion { kNone, kForward, kBackward, kLeft, kRight };

const char* motionName(Motion m) {
    switch (m) {
        case Motion::kNone: return "stand";
        case Motion::kForward: return "FORWARD";
        case Motion::kBackward: return "BACKWARD";
        case Motion::kLeft: return "TURN LEFT";
        case Motion::kRight: return "TURN RIGHT";
    }
    return "?";
}

const char* stateName(LinkState s) {
    switch (s) {
        case LinkState::kLive: return "LIVE";
        case LinkState::kStand: return "STAND";
        case LinkState::kRelax: return "RELAX (torque off)";
        case LinkState::kEstop: return "ESTOP (latched)";
        case LinkState::kLowBattLand: return "VLAND (flat pack, crouching)";
        case LinkState::kLowBattSafe: return "VSAFE (flat pack, torque off)";
        case LinkState::kFallen: return "FALLEN (torque off)";
        case LinkState::kBench: return "BENCH (loop not armed)";
    }
    return "?";
}

double nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 +
           static_cast<double>(ts.tv_nsec) / 1e6;
}

struct Options {
    const char* host = nullptr;
    int cmd_port = kCmdPort;
    int tlm_port = kTlmPort;
    double hold_ms = kDefaultHoldMs;
    double vx = kDefaultVx;
    double wz = kDefaultWz;
    double rate_hz = static_cast<double>(kSendHz);
};

void usage(const char* argv0) {
    fprintf(stderr,
            "usage: %s --host IP [--cmd-port %d] [--tlm-port %d]\n"
            "          [--hold-ms %.0f] [--vx %.2f] [--wz %.2f] [--rate %.0f]\n",
            argv0, kCmdPort, kTlmPort, kDefaultHoldMs, kDefaultVx, kDefaultWz,
            static_cast<double>(kSendHz));
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* v = (i + 1 < argc) ? argv[i + 1] : nullptr;
        if (!strcmp(a, "--host") && v) { o.host = v; ++i; }
        else if (!strcmp(a, "--cmd-port") && v) { o.cmd_port = atoi(v); ++i; }
        else if (!strcmp(a, "--tlm-port") && v) { o.tlm_port = atoi(v); ++i; }
        else if (!strcmp(a, "--hold-ms") && v) { o.hold_ms = atof(v); ++i; }
        else if (!strcmp(a, "--vx") && v) { o.vx = atof(v); ++i; }
        else if (!strcmp(a, "--wz") && v) { o.wz = atof(v); ++i; }
        else if (!strcmp(a, "--rate") && v) { o.rate_hz = atof(v); ++i; }
        else return false;
    }
    return o.host != nullptr && o.hold_ms > 0.0 && o.rate_hz > 0.0;
}

// -- the link ---------------------------------------------------------------
struct Link {
    int tx = -1, rx = -1;
    sockaddr_in to = {};
    uint32_t seq = 0;
    uint32_t sent = 0;

    Telemetry tlm = {};
    bool have_tlm = false;
    double tlm_at_ms = -1.0;
    uint32_t tlm_count = 0, tlm_bad = 0;
    // Beacon rate: frames in the last second, updated once a second.
    uint32_t rate_window = 0;
    double rate_at_ms = 0.0;
    double rate_hz = 0.0;

    bool open(const Options& o) {
        tx = socket(AF_INET, SOCK_DGRAM, 0);
        rx = socket(AF_INET, SOCK_DGRAM, 0);
        if (tx < 0 || rx < 0) return false;
        to.sin_family = AF_INET;
        to.sin_port = htons(static_cast<uint16_t>(o.cmd_port));
        if (inet_pton(AF_INET, o.host, &to.sin_addr) != 1) return false;
        int one = 1;
        setsockopt(rx, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in bind_addr = {};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port = htons(static_cast<uint16_t>(o.tlm_port));
        if (bind(rx, reinterpret_cast<sockaddr*>(&bind_addr),
                 sizeof bind_addr) < 0) {
            return false;
        }
        timeval tv = {0, 0};
        setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        return true;
    }

    void send(float vx, float wz, uint8_t flags) {
        uint8_t wire[kCmdLen];
        encodeCommand(wire, seq++, vx, wz, flags);
        if (sendto(tx, wire, kCmdLen, 0, reinterpret_cast<sockaddr*>(&to),
                   sizeof to) == static_cast<ssize_t>(kCmdLen)) {
            ++sent;
        }
    }

    // Drain to the freshest beacon; never queue up.
    void poll(double now_ms) {
        uint8_t buf[64];
        for (;;) {
            const ssize_t n = recv(rx, buf, sizeof buf, MSG_DONTWAIT);
            if (n < 0) break;
            Telemetry t = {};
            if (decodeTelemetry(buf, static_cast<size_t>(n), t) != Err::kOk) {
                ++tlm_bad;
                continue;
            }
            tlm = t;
            have_tlm = true;
            tlm_at_ms = now_ms;
            ++tlm_count;
            ++rate_window;
        }
        if (now_ms - rate_at_ms >= 1000.0) {
            rate_hz = static_cast<double>(rate_window) * 1000.0 /
                      (now_ms - rate_at_ms);
            rate_window = 0;
            rate_at_ms = now_ms;
        }
    }

    double tlmAgeMs(double now_ms) const {
        return have_tlm ? now_ms - tlm_at_ms : -1.0;
    }
};

// -- the operator's intent --------------------------------------------------
struct Console {
    bool armed = false;
    bool estop = false;
    Motion motion = Motion::kNone;
    double held_until_ms = 0.0;
    double armed_at_ms = 0.0;
    char note[160] = "";

    // Key-repeat measurement, so --hold-ms is set from evidence.
    bool key_down = false;
    double press_at_ms = 0.0, last_key_ms = 0.0;
    double initial_delay_ms = -1.0, repeat_gap_ms = -1.0;

    void say(const char* s) {
        strncpy(note, s, sizeof note - 1);
        note[sizeof note - 1] = 0;
    }

    void motionKey(Motion m, double now_ms, double hold_ms) {
        if (!armed) { say("not armed -- press [a] first"); return; }
        if (estop) { say("E-STOP latched -- [space] to stand, then move"); return; }
        if (key_down && m == motion) {
            const double gap = now_ms - last_key_ms;
            if (initial_delay_ms < 0.0) initial_delay_ms = now_ms - press_at_ms;
            else repeat_gap_ms = gap;
        } else {
            press_at_ms = now_ms;
            initial_delay_ms = -1.0;
        }
        key_down = true;
        last_key_ms = now_ms;
        motion = m;
        held_until_ms = now_ms + hold_ms;
    }

    void stand(const char* why) {
        motion = Motion::kNone;
        key_down = false;
        if (estop) { estop = false; say("E-stop cleared: standing"); }
        else if (why) say(why);
    }

    void tick(double now_ms) {
        if (motion != Motion::kNone && now_ms >= held_until_ms) {
            motion = Motion::kNone;
            key_down = false;
        }
    }

    // What goes on the wire this tick.
    void frame(const Options& o, float& vx, float& wz, uint8_t& flags) const {
        vx = 0.0f;
        wz = 0.0f;
        flags = 0;
        if (armed) flags |= kFlagArm;
        if (estop) { flags |= kFlagEstop; return; }
        if (!armed || motion == Motion::kNone) return;
        flags |= kFlagEnable;
        switch (motion) {
            case Motion::kForward: vx = static_cast<float>(o.vx); break;
            case Motion::kBackward: vx = -static_cast<float>(o.vx); break;
            case Motion::kLeft: wz = static_cast<float>(o.wz); break;
            case Motion::kRight: wz = -static_cast<float>(o.wz); break;
            case Motion::kNone: break;
        }
    }
};

// -- drawing ----------------------------------------------------------------
void flagsText(uint8_t f, char* out, size_t cap) {
    snprintf(out, cap, "%s%s%s%s", (f & kFlagArm) ? "ARM " : "",
             (f & kFlagEnable) ? "ENABLE " : "", (f & kFlagEstop) ? "ESTOP " : "",
             f ? "" : "(none)");
}

void draw(const Options& o, const Link& link, const Console& c, double now_ms,
          float vx, float wz, uint8_t flags) {
    erase();
    int row = 0;
    mvprintw(row++, 0, "bimo console -> %s:%d   telemetry :%d   %.0f Hz   [q] quit",
             o.host, o.cmd_port, o.tlm_port, o.rate_hz);
    mvhline(row++, 0, ACS_HLINE, 78);

    const int L = 0, R = 38;
    const int top = row;
    mvprintw(row++, L, "CLIENT");
    mvprintw(row++, L, "arm      %s", c.armed ? "ARMED" : "disarmed");
    mvprintw(row++, L, "e-stop   %s", c.estop ? "LATCHED" : "--");
    if (c.motion != Motion::kNone) {
        mvprintw(row++, L, "motion   %-10s (hold %.2f s left)", motionName(c.motion),
                 (c.held_until_ms - now_ms) / 1000.0);
    } else {
        mvprintw(row++, L, "motion   stand");
    }
    char ft[40];
    flagsText(flags, ft, sizeof ft);
    mvprintw(row++, L, "sending  vx %+.2f  wz %+.2f", static_cast<double>(vx),
             static_cast<double>(wz));
    mvprintw(row++, L, "flags    %s", ft);
    mvprintw(row++, L, "seq      %u  (%u sent)", link.seq, link.sent);

    row = top;
    const double age = link.tlmAgeMs(now_ms);
    if (!link.have_tlm) {
        mvprintw(row++, R, "ROBOT   (no telemetry yet)");
    } else if (age > kTlmLostMs) {
        mvprintw(row++, R, "ROBOT   LINK LOST  (last beacon %.1f s ago)", age / 1000.0);
    } else {
        mvprintw(row++, R, "ROBOT   beacon %.0f ms ago, %.1f Hz", age, link.rate_hz);
    }
    if (link.have_tlm) {
        const Telemetry& t = link.tlm;
        mvprintw(row++, R, "state    %s", stateName(t.state));
        mvprintw(row++, R, "vbat     %.2f V", static_cast<double>(t.vbat_v));
        mvprintw(row++, R, "up_z     %.2f", static_cast<double>(t.up_z));
        mvprintw(row++, R, "vx_est   %+.2f m/s   wz_est %+.2f rad/s",
                 static_cast<double>(t.vx_est), static_cast<double>(t.wz_est));
        if (t.servo_err) {
            char bits[16];
            for (int b = 7; b >= 0; --b) bits[7 - b] = (t.servo_err >> b) & 1 ? '1' : '0';
            bits[8] = 0;
            mvprintw(row++, R, "servo    FAULT %s (bit i = ID i+1)", bits);
        } else {
            mvprintw(row++, R, "servo    ok");
        }
        mvprintw(row++, R, "loop     %u%% late ticks", t.loop_late_pct);
        if (t.state == LinkState::kBench) {
            // Benched, seq_echo is the diagnostic byte, not a sequence -- so
            // do not draw a "lag" computed from it. Showing the raw byte too
            // keeps a screenshot diagnosable against protocol.h.
            mvprintw(row++, R, "diag     0x%02X  beacons %u bad %u",
                     telemetryDiag(t), link.tlm_count, link.tlm_bad);
        } else {
            mvprintw(row++, R, "seq_echo %u  (lag %ld)  beacons %u bad %u",
                     t.seq_echo,
                     static_cast<long>(link.seq) - 1 -
                         static_cast<long>(t.seq_echo),
                     link.tlm_count, link.tlm_bad);
        }
    }

    row = top + 9;
    mvhline(row++, 0, ACS_HLINE, 78);
    mvprintw(row++, 0, "[a] arm/disarm   [up/down] walk %.2f m/s   [left/right] turn %.2f rad/s",
             o.vx, o.wz);
    mvprintw(row++, 0, "[space] stand    [e] E-STOP        hold timeout %.0f ms", o.hold_ms);
    if (c.initial_delay_ms >= 0.0) {
        mvprintw(row++, 0, "key repeat: initial delay %.0f ms, repeat every %.0f ms%s",
                 c.initial_delay_ms, c.repeat_gap_ms,
                 c.initial_delay_ms >= o.hold_ms
                     ? "  -- LONGER than hold: raise --hold-ms" : "");
    } else {
        mvprintw(row++, 0, "key repeat: not measured yet (hold a motion key)");
    }
    // The robot's own answer to "why is nothing happening?". Drawn whenever
    // the beacon says BENCH, not only after an arm attempt: a console opened
    // on an uncalibrated robot should read "arming will be REFUSED" before
    // anyone presses [a], instead of after a silent 1.5 s.
    if (link.have_tlm && link.tlm.state == LinkState::kBench &&
        link.tlmAgeMs(now_ms) < kTlmLostMs) {
        mvprintw(row++, 0, "BENCH: %s", diagReason(telemetryDiag(link.tlm)));
    }
    if (c.note[0]) mvprintw(row++, 0, "%s", c.note);
    refresh();
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        usage(argv[0]);
        return 2;
    }
    Link link;
    if (!link.open(o)) {
        fprintf(stderr, "link: %s (host %s, tlm port %d)\n", strerror(errno),
                o.host, o.tlm_port);
        return 1;
    }

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    set_escdelay(25);
    curs_set(0);
    // keypad() puts the terminal in application-cursor mode, where terminfo
    // expects arrows as ESC O A..D -- but plenty of terminals (and anything
    // relaying keys through a pty) still send ESC [ A..D. Unrecognised, that
    // arrives as three separate keys, the last of which is a letter; hence
    // both encodings are registered and no command is bound to an uppercase
    // letter, so a split sequence can never fire one.
    define_key("\033[A", KEY_UP);
    define_key("\033[B", KEY_DOWN);
    define_key("\033[D", KEY_LEFT);
    define_key("\033[C", KEY_RIGHT);
    define_key("\033OA", KEY_UP);
    define_key("\033OB", KEY_DOWN);
    define_key("\033OD", KEY_LEFT);
    define_key("\033OC", KEY_RIGHT);

    Console c;
    const double period_ms = 1000.0 / o.rate_hz;
    double next_ms = nowMs();
    bool quit = false;
    while (!quit) {
        const double now_ms = nowMs();

        // Drain EVERY pending key: repeats must not queue up behind a slow
        // tick, or a released key stays "held" for as long as the backlog.
        for (int ch; (ch = getch()) != ERR;) {
            switch (ch) {
                case 'q': quit = true; break;
                case 'a':
                    if (c.armed) {
                        c.armed = false;
                        c.stand(nullptr);
                        c.say("disarm sent: robot benches, torque off");
                    } else {
                        c.armed = true;
                        c.estop = false;
                        c.armed_at_ms = now_ms;
                        c.say("arm sent: waiting for the robot to leave BENCH");
                    }
                    break;
                case 'e':
                    c.estop = true;
                    c.motion = Motion::kNone;
                    c.key_down = false;
                    c.say("E-STOP sent (latched until [space])");
                    break;
                case ' ': case 's': c.stand("stand"); break;
                case KEY_UP: c.motionKey(Motion::kForward, now_ms, o.hold_ms); break;
                case KEY_DOWN: c.motionKey(Motion::kBackward, now_ms, o.hold_ms); break;
                case KEY_LEFT: c.motionKey(Motion::kLeft, now_ms, o.hold_ms); break;
                case KEY_RIGHT: c.motionKey(Motion::kRight, now_ms, o.hold_ms); break;
                default: break;
            }
        }
        c.tick(now_ms);

        link.poll(now_ms);
        // The robot decides its mode. Armed here but BENCH there, after the
        // edge has had time to land: refused (no calibration in NVS) or the
        // robot rebooted under us. Either way, this console is wrong.
        if (c.armed && link.have_tlm && link.tlmAgeMs(now_ms) < kTlmLostMs &&
            link.tlm.state == LinkState::kBench &&
            now_ms - c.armed_at_ms > kArmSyncMs) {
            c.armed = false;
            c.stand(nullptr);
            // The robot now says WHY (protocol.h kDiag*), so quote it instead
            // of guessing at "no calibration?" the way this line used to.
            char msg[sizeof c.note];
            snprintf(msg, sizeof msg, "robot stayed BENCH after arm: %s",
                     diagReason(telemetryDiag(link.tlm)));
            c.say(msg);
        }

        float vx, wz;
        uint8_t flags;
        c.frame(o, vx, wz, flags);
        link.send(vx, wz, flags);
        draw(o, link, c, now_ms, vx, wz, flags);

        next_ms += period_ms;
        const double sleep_ms = next_ms - nowMs();
        if (sleep_ms > 0.0) {
            usleep(static_cast<useconds_t>(sleep_ms * 1000.0));
        } else {
            next_ms = nowMs();          // fell behind: resync, do not burst
        }
    }

    // Leave a limp robot, not a standing one nobody is watching: drop ARM (the
    // 1 -> 0 edge benches it) and repeat, since any single datagram may vanish.
    for (int i = 0; i < 10; ++i) {
        link.send(0.0f, 0.0f, 0);
        usleep(20 * 1000);
    }
    endwin();
    printf("bimo console: quit; sent disarm (%u frames total)\n", link.sent);
    return 0;
}
