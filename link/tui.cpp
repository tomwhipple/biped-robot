// bimo console: single-keystroke control of the robot over the WiFi link.
//
//   make -C firmware/host tui
//   firmware/host/build/bimo_tui --host 192.168.2.90
//
// Every byte this sends is produced by firmware/components/linkproto -- the
// same translation units the ESP32 decodes with -- so the client and the
// robot cannot disagree about the protocol. The Python link/ tools are the
// reference and the twins; this is the operator's seat over SSH. The windowed
// seat is link/gui.cpp, and both sit on link/client.h so they cannot disagree
// with each other either.
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
// released key "held". (bimo_gui has none of this: a window gets real
// key-release events, so its dead-man is exact.)
//
// The robot is authoritative about its mode: if it reports BENCH while this
// console believes it armed (arm refused for want of a calibration, or the
// robot rebooted), the console drops back to disarmed and says so.
#include <errno.h>
#include <ncurses.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "client.h"
#include "linkproto/protocol.h"

namespace {

using namespace linkproto;
using bimo::Ext;
using bimo::Intent;
using bimo::Link;
using bimo::Speeds;
using bimo::nowMs;

constexpr double kDefaultHoldMs = 700.0;   // > any common initial repeat delay

struct Options {
    const char* host = nullptr;
    int cmd_port = kCmdPort;
    int tlm_port = kTlmPort;
    double hold_ms = kDefaultHoldMs;
    Speeds speeds;
    double rate_hz = static_cast<double>(kSendHz);
};

void usage(const char* argv0) {
    fprintf(stderr,
            "usage: %s --host IP [--cmd-port %d] [--tlm-port %d]\n"
            "          [--hold-ms %.0f] [--vx %.2f] [--wz %.2f] [--rate %.0f]\n",
            argv0, kCmdPort, kTlmPort, kDefaultHoldMs, bimo::kDefaultVx,
            bimo::kDefaultWz, static_cast<double>(kSendHz));
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* v = (i + 1 < argc) ? argv[i + 1] : nullptr;
        if (!strcmp(a, "--host") && v) { o.host = v; ++i; }
        else if (!strcmp(a, "--cmd-port") && v) { o.cmd_port = atoi(v); ++i; }
        else if (!strcmp(a, "--tlm-port") && v) { o.tlm_port = atoi(v); ++i; }
        else if (!strcmp(a, "--hold-ms") && v) { o.hold_ms = atof(v); ++i; }
        else if (!strcmp(a, "--vx") && v) { o.speeds.vx = static_cast<float>(atof(v)); ++i; }
        else if (!strcmp(a, "--wz") && v) { o.speeds.wz = static_cast<float>(atof(v)); ++i; }
        else if (!strcmp(a, "--rate") && v) { o.rate_hz = atof(v); ++i; }
        else return false;
    }
    return o.host != nullptr && o.hold_ms > 0.0 && o.rate_hz > 0.0;
}

// -- the terminal's fake dead-man -------------------------------------------
// One axis at a time (a terminal delivers one key at a time), expiring
// --hold-ms after the last auto-repeat.
struct Hold {
    int axis = -1;                       // -1 = standing
    double until_ms = 0.0;
    bool key_down = false;
    double press_at_ms = 0.0, last_key_ms = 0.0;
    double initial_delay_ms = -1.0, repeat_gap_ms = -1.0;

    void key(Intent& in, int ax, double now_ms, double hold_ms) {
        in.releaseAll();
        if (!in.press(ax, now_ms)) {     // refused: not armed, or E-stopped
            axis = -1;
            key_down = false;
            return;
        }
        if (key_down && ax == axis) {
            const double gap = now_ms - last_key_ms;
            if (initial_delay_ms < 0.0) initial_delay_ms = now_ms - press_at_ms;
            else repeat_gap_ms = gap;
        } else {
            press_at_ms = now_ms;
            initial_delay_ms = -1.0;
        }
        key_down = true;
        last_key_ms = now_ms;
        axis = ax;
        until_ms = now_ms + hold_ms;
    }

    void tick(Intent& in, double now_ms) {
        if (axis >= 0 && now_ms >= until_ms) {
            in.releaseAll();
            axis = -1;
            key_down = false;
        }
    }

    void clear() {
        axis = -1;
        key_down = false;
    }
};

// -- drawing ----------------------------------------------------------------
void flagsText(uint8_t f, char* out, size_t cap) {
    snprintf(out, cap, "%s%s%s%s", (f & kFlagArm) ? "ARM " : "",
             (f & kFlagEnable) ? "ENABLE " : "", (f & kFlagEstop) ? "ESTOP " : "",
             f ? "" : "(none)");
}

void draw(const Options& o, const Link& link, const Intent& in, const Hold& hold,
          double now_ms, float vx, float wz, uint8_t flags) {
    erase();
    int row = 0;
    mvprintw(row++, 0, "bimo console -> %s:%d   telemetry :%d   %.0f Hz   [q] quit",
             o.host, o.cmd_port, o.tlm_port, link.rate_hz);
    mvhline(row++, 0, ACS_HLINE, 78);

    const int L = 0, R = 38;
    const int top = row;
    mvprintw(row++, L, "CLIENT");
    mvprintw(row++, L, "arm      %s", in.armed ? "ARMED" : "disarmed");
    mvprintw(row++, L, "e-stop   %s", in.estop ? "LATCHED" : "--");
    if (hold.axis >= 0) {
        mvprintw(row++, L, "motion   %-10s (hold %.2f s left)",
                 bimo::axisName(hold.axis), (hold.until_ms - now_ms) / 1000.0);
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
    } else if (age > bimo::kTlmLostMs) {
        mvprintw(row++, R, "ROBOT   LINK LOST  (last beacon %.1f s ago)", age / 1000.0);
    } else {
        mvprintw(row++, R, "ROBOT   beacon %.0f ms ago, %.1f Hz", age, link.rate_hz);
    }
    if (link.have_tlm) {
        const Telemetry& t = link.tlm;
        mvprintw(row++, R, "state    %s", bimo::stateName(t.state));
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
                     t.seq_echo, link.lag(), link.tlm_count, link.tlm_bad);
        }
    }

    row = top + 9;
    mvhline(row++, 0, ACS_HLINE, 78);
    mvprintw(row++, 0, "[a] arm/disarm   [up/down] walk %.2f m/s   [left/right] turn %.2f rad/s",
             static_cast<double>(o.speeds.vx), static_cast<double>(o.speeds.wz));
    mvprintw(row++, 0, "[space] stand    [e] E-STOP        hold timeout %.0f ms", o.hold_ms);
    if (hold.initial_delay_ms >= 0.0) {
        mvprintw(row++, 0, "key repeat: initial delay %.0f ms, repeat every %.0f ms%s",
                 hold.initial_delay_ms, hold.repeat_gap_ms,
                 hold.initial_delay_ms >= o.hold_ms
                     ? "  -- LONGER than hold: raise --hold-ms" : "");
    } else {
        mvprintw(row++, 0, "key repeat: not measured yet (hold a motion key)");
    }
    // The robot's own answer to "why is nothing happening?". Drawn whenever
    // the beacon says BENCH, not only after an arm attempt: a console opened
    // on an uncalibrated robot should read "arming will be REFUSED" before
    // anyone presses [a], instead of after a silent 1.5 s.
    if (link.have_tlm && link.tlm.state == LinkState::kBench &&
        !link.linkLost(now_ms)) {
        mvprintw(row++, 0, "BENCH: %s", diagReason(telemetryDiag(link.tlm)));
    }
    if (in.note[0]) mvprintw(row++, 0, "%s", in.note);
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
    if (!link.open(o.host, o.cmd_port, o.tlm_port)) {
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

    Intent in;
    Hold hold;
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
                    hold.clear();
                    in.toggleArm(now_ms);
                    break;
                case 'e':
                    hold.clear();
                    in.fireEstop();
                    break;
                case ' ': case 's':
                    hold.clear();
                    in.stand("stand");
                    break;
                case KEY_UP: hold.key(in, bimo::kAxForward, now_ms, o.hold_ms); break;
                case KEY_DOWN: hold.key(in, bimo::kAxBack, now_ms, o.hold_ms); break;
                // Left/right TURN on the terminal: with one key at a time and
                // no key-up, yaw is the more useful of the two. The window
                // client puts strafe here and turn on PgUp/PgDn.
                case KEY_LEFT: hold.key(in, bimo::kAxTurnLeft, now_ms, o.hold_ms); break;
                case KEY_RIGHT: hold.key(in, bimo::kAxTurnRight, now_ms, o.hold_ms); break;
                default: break;
            }
        }
        hold.tick(in, now_ms);

        link.poll(now_ms);
        // The robot decides its mode. Armed here but BENCH there, after the
        // edge has had time to land: refused (no calibration in NVS) or the
        // robot rebooted under us. Either way, this console is wrong.
        if (in.syncToRobot(link, now_ms)) hold.clear();

        float vx, vy, wz;
        bimo::Ext ext;
        uint8_t flags;
        bimo::sendIntent(link, in, o.speeds, vx, vy, wz, ext, flags);
        (void)vy;                        // the terminal client never strafes
        draw(o, link, in, hold, now_ms, vx, wz, flags);

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
