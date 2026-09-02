// bimo_gui: the windowed operator's console.
//
//   brew install glfw
//   make -C firmware/host deps gui
//   firmware/host/build/bimo_gui --host 192.168.2.90     # the robot
//   firmware/host/build/bimo_gui --host 127.0.0.1        # the sim; press Start
//
// Same wire, same core, same robot as link/tui.cpp: everything about arming,
// the E-stop latch and the frame that goes out at 20 Hz lives in
// link/client.h, so the two consoles cannot disagree. What a window buys, and
// a terminal cannot:
//
//   * A REAL dead-man. A terminal has no key-up event, so bimo_tui fakes one
//     with a --hold-ms timeout tuned against the auto-repeat delay. Here a
//     released key stops the robot on the very next frame -- and so does
//     clicking away from the window, which is the one new way a window can
//     get an operator into trouble, so it is handled explicitly.
//   * The EXTENDED channels. crouch / lift / foot_dx / foot_dz are sliders;
//     no keyboard reaches them. Off their defaults the frame becomes 24 B,
//     which is shown on screen, because firmware older than 2026-08-31 drops
//     a long frame by length.
//   * Telemetry as a SHAPE. vbat, up_z and the command-vs-estimate pairs are
//     30-second strip charts rather than a number the screen overwrites 20
//     times a second.
//   * The sim, IN the window. sim/sil_twin.py runs the real firmware control
//     stack at 1x real time and serves its render as MJPEG; the SIM panel
//     starts it and draws it, so a policy can be flown before it is flown.
//
// --headless runs the identical link loop with no window, taking `press
// forward` / `release forward` / `set crouch 0.8` on stdin. That is what
// tests/test_gui_e2e.py drives: no display, no pty, and it exercises the real
// key-release path rather than a simulation of auto-repeat.
#include <errno.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "client.h"
#include "jpeg.h"
#include "keymap.h"
#include "linkproto/protocol.h"
#include "mjpeg.h"
#include "recorder.h"
#include "simproc.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
// gl3.h first, and GLFW told not to pull gl.h in behind it: including both is
// a warning, and warnings are errors here.
#include <OpenGL/gl3.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace {

using namespace linkproto;
using bimo::Ext;
using bimo::Intent;
using bimo::Keymap;
using bimo::Link;
using bimo::Speeds;
using bimo::nowMs;

constexpr int kAxCountLocal = bimo::kAxCount;
constexpr int kChart = 300;                // 30 s of a 10 Hz beacon
constexpr int kDefaultStreamPort = 8645;   // sim/sil_twin.py's own default
// A stream with nothing new for this long is stalled, and must look stalled.
constexpr double kStreamStaleMs = 2000.0;

struct Ring {
    float v[kChart] = {0.0f};
    int off = 0, n = 0;
    void push(float x) {
        v[off] = x;
        off = (off + 1) % kChart;
        if (n < kChart) ++n;
    }
};

struct Options {
    const char* host = nullptr;
    int cmd_port = kCmdPort;
    int tlm_port = kTlmPort;
    int stream_port = kDefaultStreamPort;
    Speeds speeds;
    double rate_hz = static_cast<double>(kSendHz);
    const char* keymap_path = nullptr;
    const char* sim_view = nullptr;      // explicit URL; else host:stream_port
    const char* repo = nullptr;
    const char* sessions = nullptr;      // default <repo>/hw_sessions
    const char* run_name = nullptr;      // preselected sim run
    bool mirror = false;                 // start in mirror mode
    int sim_cmd_port = 0;                // 0 -> cmd_port + 100
    bool headless = false;
    bool record = true;                  // --no-record to opt out
    bool autostream = false;             // --sim-view given: connect at boot
    const char* screenshot = nullptr;    // write a PNG of the window, then go
    double screenshot_after = 6.0;
};

void usage(const char* argv0) {
    fprintf(stderr,
            "usage: %s --host IP [--cmd-port %d] [--tlm-port %d]\n"
            "          [--vx %.2f] [--vy %.2f] [--wz %.2f] [--rate %.0f]\n"
            "          [--keymap FILE] [--sim-view [URL]] [--stream-port %d]\n"
            "          [--run-name NAME] [--repo DIR] [--sessions DIR]\n"
            "          [--headless] [--no-record] [--sim-cmd-port N]\n"
            "          [--mirror]  drive the robot, start a sim, ghost it\n"
            "          [--screenshot FILE [--screenshot-after S]]\n",
            argv0, kCmdPort, kTlmPort, bimo::kDefaultVx, bimo::kDefaultVy,
            bimo::kDefaultWz, static_cast<double>(kSendHz), kDefaultStreamPort);
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* v = (i + 1 < argc) ? argv[i + 1] : nullptr;
        const bool has_val = v != nullptr && v[0] != '-';
        if (!strcmp(a, "--host") && v) { o.host = v; ++i; }
        else if (!strcmp(a, "--cmd-port") && v) { o.cmd_port = atoi(v); ++i; }
        else if (!strcmp(a, "--tlm-port") && v) { o.tlm_port = atoi(v); ++i; }
        else if (!strcmp(a, "--stream-port") && v) { o.stream_port = atoi(v); ++i; }
        else if (!strcmp(a, "--vx") && v) { o.speeds.vx = static_cast<float>(atof(v)); ++i; }
        else if (!strcmp(a, "--vy") && v) { o.speeds.vy = static_cast<float>(atof(v)); ++i; }
        else if (!strcmp(a, "--wz") && v) { o.speeds.wz = static_cast<float>(atof(v)); ++i; }
        else if (!strcmp(a, "--rate") && v) { o.rate_hz = atof(v); ++i; }
        else if (!strcmp(a, "--keymap") && v) { o.keymap_path = v; ++i; }
        else if (!strcmp(a, "--repo") && v) { o.repo = v; ++i; }
        else if (!strcmp(a, "--sessions") && v) { o.sessions = v; ++i; }
        else if (!strcmp(a, "--run-name") && v) { o.run_name = v; ++i; }
        else if (!strcmp(a, "--headless")) { o.headless = true; }
        else if (!strcmp(a, "--no-record")) { o.record = false; }
        else if (!strcmp(a, "--mirror")) { o.mirror = true; }
        else if (!strcmp(a, "--sim-cmd-port") && v) { o.sim_cmd_port = atoi(v); ++i; }
        else if (!strcmp(a, "--screenshot") && v) { o.screenshot = v; ++i; }
        else if (!strcmp(a, "--screenshot-after") && v) {
            o.screenshot_after = atof(v); ++i;
        }
        else if (!strcmp(a, "--sim-view")) {
            o.autostream = true;
            if (has_val) { o.sim_view = v; ++i; }
        }
        else return false;
    }
    return o.host != nullptr && o.rate_hz > 0.0;
}

// Everything the two front ends share.
struct App {
    Options o;
    Link link;
    Intent in;
    Keymap keys;
    bimo::Recorder rec;
    bimo::MjpegStream stream;
    bimo::SimProc sim;
    bimo::RunList runs;

    char host_port[64] = "";
    char repo[512] = ".";
    char sessions[512] = "hw_sessions";
    char keymap_path[512] = "";
    char sim_url[512] = "";
    char status[320] = "";
    int run_sel = 0;
    // Default OFF: the console should come up looking like the robot does on
    // the bench at power-on -- standing on the ground, benched, torque off.
    // --hang is the test-stand plant (torso welded, feet free), which is a
    // deliberate diagnostic, not a resting state.
    bool sim_hang = false;
    // Mirror mode: the console drives the ROBOT, the same command is relayed
    // to the sim, and the robot's observed joint angles are pushed to the sim
    // to draw as a ghost. docs/mirror-mode.md.
    bool mirror = false;
    int sim_cmd_port = 0, sim_tlm_port = 0;
    double last_pose_ms = 0.0;
    uint32_t poses_sent = 0;

    // what went on the wire this tick
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    Ext ext_sent;
    uint8_t flags = 0;

    // Everything the console says, on disk. On-screen text scrolls away or
    // is simply not being looked at, which is how a sil_twin that died on its
    // first line stayed invisible to both the operator and the author.
    FILE* logf = nullptr;
    char last_note[192] = "";
    char last_status[320] = "";
    char last_sim_err[192] = "";
    char last_stream_err[192] = "";
    int logged_sim_lines = 0;

    Ring c_vbat, c_upz, c_vx_cmd, c_vx_est, c_wz_cmd, c_wz_est, c_late;
    uint32_t charted = 0;
    bool quit = false;

    // Two independent sources for the same six axes, resolved once per frame
    // AFTER the UI is built (applyHeld). Holding a button with the mouse and
    // holding the key must mean exactly the same thing; before this they did
    // not, because pumpKeys ran at the top of the frame and released whatever
    // the previous frame's button had set.
    bool key_held[kAxCountLocal] = {false, false, false, false, false, false};
    bool mouse_held[kAxCountLocal] = {false, false, false, false, false, false};

    // window-only
    GLuint tex = 0;
    int tex_w = 0, tex_h = 0;
    uint64_t frame_seq = 0;
    int rebind = -1;                     // action awaiting a key
    bool show_keymap = false;
};

void logEvent(App& a, const char* kind, const char* fmt, ...) {
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    time_t now = time(nullptr);
    tm utc;
    gmtime_r(&now, &utc);
    char stamp[32];
    strftime(stamp, sizeof stamp, "%Y-%m-%dT%H:%M:%SZ", &utc);
    if (a.logf != nullptr) {
        fprintf(a.logf, "%s %-6s %s\n", stamp, kind, msg);
        fflush(a.logf);            // a crash must not cost the last line
    }
    fprintf(stderr, "%s %-6s %s\n", stamp, kind, msg);
}

void openLog(App& a) {
    mkdir(a.sessions, 0755);
    char path[600];
    snprintf(path, sizeof path, "%s/bimo_gui.log", a.sessions);
    a.logf = fopen(path, "a");
    if (a.logf == nullptr) {
        fprintf(stderr, "bimo_gui: cannot append to %s\n", path);
    }
}

// Anything the console has started saying since the last tick goes to the
// log. Done by diffing the message fields rather than at each call site, so a
// message added later is logged without anyone remembering to.
void pumpLog(App& a) {
    if (strcmp(a.last_note, a.in.note) != 0) {
        snprintf(a.last_note, sizeof a.last_note, "%s", a.in.note);
        if (a.last_note[0]) logEvent(a, "note", "%s", a.last_note);
    }
    if (strcmp(a.last_status, a.status) != 0) {
        snprintf(a.last_status, sizeof a.last_status, "%s", a.status);
        if (a.last_status[0]) logEvent(a, "status", "%s", a.last_status);
    }
    if (strcmp(a.last_sim_err, a.sim.err) != 0) {
        snprintf(a.last_sim_err, sizeof a.last_sim_err, "%s", a.sim.err);
        if (a.last_sim_err[0]) logEvent(a, "SIM", "%s", a.last_sim_err);
    }
    char serr[192];
    a.stream.error(serr, sizeof serr);
    if (strcmp(a.last_stream_err, serr) != 0) {
        snprintf(a.last_stream_err, sizeof a.last_stream_err, "%s", serr);
        if (a.last_stream_err[0]) logEvent(a, "stream", "%s", a.last_stream_err);
    }
    // The child's own stdout -- its last words when it dies are in here.
    const int have = a.sim.lines();
    const int first = a.sim.log_n - have;
    if (a.logged_sim_lines < first) a.logged_sim_lines = first;
    while (a.logged_sim_lines < a.sim.log_n) {
        logEvent(a, "sim", "%s", a.sim.line(a.logged_sim_lines - first));
        ++a.logged_sim_lines;
    }
}

// One tick of the link, identical windowed or headless.
void tick(App& a, double now_ms) {
    a.link.poll(now_ms);
    a.in.syncToRobot(a.link, now_ms);
    bimo::sendIntent(a.link, a.in, a.o.speeds, a.vx, a.vy, a.wz, a.ext_sent,
                     a.flags);
    a.rec.command(now_ms, a.link.seq - 1, a.vx, a.vy, a.wz, a.ext_sent, a.flags,
                  a.link.last_len);
    // Chart on each new beacon, so the x axis is the robot's clock and not
    // ours: 10 Hz * 300 samples = the last 30 seconds.
    if (a.link.tlm_count != a.charted) {
        a.charted = a.link.tlm_count;
        const Telemetry& t = a.link.tlm;
        a.c_vbat.push(t.vbat_v);
        a.c_upz.push(t.up_z);
        a.c_vx_cmd.push(a.vx);
        a.c_vx_est.push(t.vx_est);
        a.c_wz_cmd.push(a.wz);
        a.c_wz_est.push(t.wz_est);
        a.c_late.push(static_cast<float>(t.loop_late_pct));
        a.rec.telemetry(now_ms, t);
        // Mirror: hand the robot's measured pose to the sim, once per beacon,
        // so the ghost is drawn from the same 10 Hz the robot reports at.
        if (a.mirror && t.n_joints == linkproto::kNumJoints && a.sim.running()) {
            char line[256];
            int at = snprintf(line, sizeof line, "pose");
            for (size_t j = 0; j < linkproto::kNumJoints; ++j) {
                at += snprintf(line + at, sizeof line - static_cast<size_t>(at),
                               " %.4f", static_cast<double>(t.joints[j]));
            }
            if (a.sim.tell(line)) {
                ++a.poses_sent;
                a.last_pose_ms = now_ms;
            }
        }
    }
    a.rec.maybeFlush(now_ms);
    a.sim.poll();
    pumpLog(a);
}

void toggleRecord(App& a, double now_ms) {
    if (a.rec.active()) {
        snprintf(a.status, sizeof a.status, "recorded %u cmd / %u tlm -> %s",
                 a.rec.n_cmd, a.rec.n_tlm, a.rec.path);
        a.rec.stop();
        return;
    }
    char err[192] = "";
    if (!a.rec.start(a.sessions, a.o.host, now_ms, err, sizeof err)) {
        snprintf(a.status, sizeof a.status, "%s", err);
        return;
    }
    snprintf(a.status, sizeof a.status, "recording -> %s", a.rec.path);
}

// Where to watch the sim. --sim-view wins. Otherwise: a sim THIS console
// started is a LOCAL child -- simproc.cpp forks it here -- so its stream is on
// 127.0.0.1 whatever --host says, and in mirror mode --host is the robot.
// Deriving the URL from --host there watches port %d on the robot, which
// nothing serves: the panel connects forever and the operator sees an empty
// frame with mirror apparently on. A stream this console did NOT start is the
// other case, and there --host is exactly the right guess.
void simViewUrl(const App& a, char* out, size_t cap, bool ours) {
    if (a.o.sim_view != nullptr) {
        snprintf(out, cap, "%s", a.o.sim_view);
        return;
    }
    snprintf(out, cap, "http://%s:%d/", ours ? "127.0.0.1" : a.o.host,
             a.o.stream_port);
}

void startSim(App& a) {
    if (a.runs.n == 0) {
        snprintf(a.status, sizeof a.status,
                 "no runnable sim runs under %s/sim/runs (each needs a "
                 "config.json AND sim/sil/weights/<run>.silw.json)", a.repo);
        return;
    }
    // In mirror mode the robot owns the primary ports, so the sim gets its
    // own; otherwise it listens where the console is already pointed.
    const int cp = a.mirror ? a.sim_cmd_port : a.o.cmd_port;
    const int tp = a.mirror ? a.sim_tlm_port : a.o.tlm_port;
    if (!a.sim.start(a.repo, a.runs.name[a.run_sel], a.sim_hang,
                     a.o.stream_port, cp, tp)) {
        snprintf(a.status, sizeof a.status, "%s", a.sim.err);
        return;
    }
    simViewUrl(a, a.sim_url, sizeof a.sim_url, true);
    a.stream.start(a.sim_url);
    snprintf(a.status, sizeof a.status, "sim %s starting; view %s",
             a.runs.name[a.run_sel], a.sim_url);
}

void stopSim(App& a) {
    a.stream.stop();
    a.sim.stop();
}

// Mirror is a MODE, not a button press: it changes where commands go and what
// the robot is asked to beacon, so it says so plainly and does not pretend a
// running sim moved ports underneath it.
void setMirror(App& a, bool on) {
    a.mirror = on;
    a.in.want_pose = on;
    if (on) {
        a.link.setMirror("127.0.0.1", a.sim_cmd_port);
        snprintf(a.status, sizeof a.status,
                 "MIRROR: driving %s, relaying to the sim on :%d, asking for "
                 "joint angles", a.o.host, a.sim_cmd_port);
    } else {
        a.link.clearMirror();
        snprintf(a.status, sizeof a.status, "mirror off");
    }
}

// Actions that are not a held axis.
void fireAction(App& a, int action, double now_ms) {
    switch (action) {
        case bimo::kActArm: a.in.toggleArm(now_ms); break;
        case bimo::kActStand: a.in.stand("stand"); break;
        case bimo::kActEstop: a.in.fireEstop(); break;
        case bimo::kActQuit: a.quit = true; break;
        case bimo::kActRecord: toggleRecord(a, now_ms); break;
        case bimo::kActResetExt:
            a.in.ext.reset();
            a.in.say("extended channels back to defaults (14 B frames)");
            break;
        default: break;
    }
}

// -- headless ---------------------------------------------------------------
void headlessStatus(const App& a, double now_ms, double t0_ms) {
    char fl[40];
    snprintf(fl, sizeof fl, "%s%s%s", (a.flags & kFlagArm) ? "ARM " : "",
             (a.flags & kFlagEnable) ? "ENABLE " : "",
             (a.flags & kFlagEstop) ? "ESTOP " : "");
    printf("[%7.2fs] arm %-8s motion %-12s send vx %+.2f vy %+.2f wz %+.2f "
           "crouch %.2f len %zu flags %s",
           (now_ms - t0_ms) / 1000.0, a.in.armed ? "ARMED" : "disarmed",
           bimo::motionText(a.in), static_cast<double>(a.vx),
           static_cast<double>(a.vy), static_cast<double>(a.wz),
           static_cast<double>(a.ext_sent.crouch), a.link.last_len,
           fl[0] ? fl : "(none)");
    if (a.link.have_tlm && !a.link.linkLost(now_ms)) {
        const Telemetry& t = a.link.tlm;
        printf(" | robot %s  %.2f V  up_z %.2f  vx_est %+.2f",
               bimo::stateName(t.state), static_cast<double>(t.vbat_v),
               static_cast<double>(t.up_z), static_cast<double>(t.vx_est));
        if (t.state == LinkState::kBench) {
            printf("  BENCH: %s", diagReason(telemetryDiag(t)));
        }
    } else {
        printf(" | robot (no telemetry)");
    }
    printf("\n");
    fflush(stdout);
}

void headlessLine(App& a, char* line, double now_ms) {
    char* verb = strtok(line, " \t");
    if (verb == nullptr) return;
    char* arg = strtok(nullptr, " \t");
    char* arg2 = strtok(nullptr, " \t");
    if (!strcmp(verb, "press") || !strcmp(verb, "release")) {
        if (arg == nullptr) return;
        const int action = bimo::actionFromName(arg);
        if (action < 0 || action >= bimo::kMotionActions) {
            printf("note: `%s` is not a motion action\n", arg);
            return;
        }
        if (!strcmp(verb, "press")) a.in.press(action, now_ms);
        else a.in.release(action);
    } else if (!strcmp(verb, "sim")) {
        // The Start button, scriptable. Worth having for its own sake: the
        // spawn path is the one part of this console a test cannot otherwise
        // reach, and it is exactly the part that broke silently.
        if (arg != nullptr && !strcmp(arg, "start")) {
            if (arg2 != nullptr) {
                const int i = a.runs.indexOf(arg2);
                if (i >= 0) a.run_sel = i;
                else printf("note: no run `%s`\n", arg2);
            }
            startSim(a);
            printf("note: %s\n", a.status);
        } else if (arg != nullptr && !strcmp(arg, "stop")) {
            stopSim(a);
        } else if (arg != nullptr && !strcmp(arg, "mirror")) {
            setMirror(a, !(arg2 != nullptr && !strcmp(arg2, "off")));
            printf("note: %s\n", a.status);
        } else if (arg != nullptr && !strcmp(arg, "reset")) {
            printf("note: %s\n", a.sim.tell("reset") ? "sim reset sent"
                                                     : "no sim listening");
        } else if (arg != nullptr && !strcmp(arg, "hang")) {
            a.sim_hang = !a.sim_hang;
            printf("note: hang %s\n", a.sim_hang ? "on" : "off");
        } else {
            printf("note: sim start|stop|reset|hang|mirror [off]\n");
        }
        fflush(stdout);
    } else if (!strcmp(verb, "set")) {
        if (arg == nullptr || arg2 == nullptr) return;
        const float v = static_cast<float>(atof(arg2));
        if (!strcmp(arg, "crouch")) a.in.ext.crouch = v;
        else if (!strcmp(arg, "lift")) a.in.ext.lift = v;
        else if (!strcmp(arg, "foot_dx")) a.in.ext.foot_dx = v;
        else if (!strcmp(arg, "foot_dz")) a.in.ext.foot_dz = v;
        else { printf("note: no channel `%s`\n", arg); return; }
        a.in.ext.clamp();
    } else {
        const int action = bimo::actionFromName(verb);
        if (action >= bimo::kMotionActions) fireAction(a, action, now_ms);
        else printf("note: unknown command `%s`\n", verb);
    }
}

// Line commands on stdin, drained without blocking. Both front ends read
// them: headless because that is its only input, and the window because a
// figure worth putting in the docs has to be reproducible from a command
// line rather than from someone's hands (docs/control-channel.md).
// Returns false on EOF -- a console whose operator went away must leave, and
// leaving disarms.
bool pumpStdin(App& a, double now_ms) {
    static char line[256];
    static size_t len = 0;
    for (;;) {
        fd_set r;
        FD_ZERO(&r);
        FD_SET(STDIN_FILENO, &r);
        timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &r, nullptr, nullptr, &tv) <= 0) return true;
        char c;
        const ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) return false;
        if (c == '\n') {
            line[len] = 0;
            headlessLine(a, line, now_ms);
            len = 0;
        } else if (len + 1 < sizeof line) {
            line[len++] = c;
        }
    }
}

int runHeadless(App& a) {
    const double period_ms = 1000.0 / a.o.rate_hz;
    const double t0_ms = nowMs();
    double next_ms = t0_ms, next_status = t0_ms;
    char last_note[sizeof a.in.note] = "";
    printf("bimo_gui --headless -> %s:%d (telemetry :%d)\n", a.o.host,
           a.o.cmd_port, a.o.tlm_port);
    fflush(stdout);
    while (!a.quit) {
        const double now_ms = nowMs();

        if (!pumpStdin(a, now_ms)) a.quit = true;   // EOF: leave, disarming

        tick(a, now_ms);
        if (strcmp(last_note, a.in.note) != 0) {
            snprintf(last_note, sizeof last_note, "%s", a.in.note);
            if (last_note[0]) {
                printf("note: %s\n", last_note);
                fflush(stdout);
            }
        }
        if (now_ms >= next_status) {
            headlessStatus(a, now_ms, t0_ms);
            next_status = now_ms + 500.0;
        }
        next_ms += period_ms;
        const double sleep_ms = next_ms - nowMs();
        if (sleep_ms > 0.0) usleep(static_cast<useconds_t>(sleep_ms * 1000.0));
        else next_ms = nowMs();
    }
    return 0;
}

// -- the window -------------------------------------------------------------
// The keymap keeps its own key codes so `make test` needs no GLFW (see
// keymap.h); this is the only place that knows about the toolkit's enum.
int toKeyCode(ImGuiKey k) {
    if (k >= ImGuiKey_A && k <= ImGuiKey_Z) {
        return 'a' + static_cast<int>(k) - static_cast<int>(ImGuiKey_A);
    }
    if (k >= ImGuiKey_0 && k <= ImGuiKey_9) {
        return '0' + static_cast<int>(k) - static_cast<int>(ImGuiKey_0);
    }
    if (k >= ImGuiKey_F1 && k <= ImGuiKey_F12) {
        return bimo::kKeyF1 + static_cast<int>(k) - static_cast<int>(ImGuiKey_F1);
    }
    switch (k) {
        case ImGuiKey_Space: return bimo::kKeySpace;
        case ImGuiKey_Enter: return bimo::kKeyEnter;
        case ImGuiKey_Tab: return bimo::kKeyTab;
        case ImGuiKey_Escape: return bimo::kKeyEscape;
        case ImGuiKey_Backspace: return bimo::kKeyBackspace;
        case ImGuiKey_UpArrow: return bimo::kKeyUp;
        case ImGuiKey_DownArrow: return bimo::kKeyDown;
        case ImGuiKey_LeftArrow: return bimo::kKeyLeft;
        case ImGuiKey_RightArrow: return bimo::kKeyRight;
        case ImGuiKey_PageUp: return bimo::kKeyPageUp;
        case ImGuiKey_PageDown: return bimo::kKeyPageDown;
        case ImGuiKey_Home: return bimo::kKeyHome;
        case ImGuiKey_End: return bimo::kKeyEnd;
        case ImGuiKey_Insert: return bimo::kKeyInsert;
        case ImGuiKey_Delete: return bimo::kKeyDelete;
        default: return bimo::kKeyNone;
    }
}

ImGuiKey fromKeyCode(int c) {
    if (c >= 'a' && c <= 'z') {
        return static_cast<ImGuiKey>(static_cast<int>(ImGuiKey_A) + c - 'a');
    }
    if (c >= '0' && c <= '9') {
        return static_cast<ImGuiKey>(static_cast<int>(ImGuiKey_0) + c - '0');
    }
    if (c >= bimo::kKeyF1 && c <= bimo::kKeyF12) {
        return static_cast<ImGuiKey>(static_cast<int>(ImGuiKey_F1) + c -
                                     bimo::kKeyF1);
    }
    switch (c) {
        case bimo::kKeySpace: return ImGuiKey_Space;
        case bimo::kKeyEnter: return ImGuiKey_Enter;
        case bimo::kKeyTab: return ImGuiKey_Tab;
        case bimo::kKeyEscape: return ImGuiKey_Escape;
        case bimo::kKeyBackspace: return ImGuiKey_Backspace;
        case bimo::kKeyUp: return ImGuiKey_UpArrow;
        case bimo::kKeyDown: return ImGuiKey_DownArrow;
        case bimo::kKeyLeft: return ImGuiKey_LeftArrow;
        case bimo::kKeyRight: return ImGuiKey_RightArrow;
        case bimo::kKeyPageUp: return ImGuiKey_PageUp;
        case bimo::kKeyPageDown: return ImGuiKey_PageDown;
        case bimo::kKeyHome: return ImGuiKey_Home;
        case bimo::kKeyEnd: return ImGuiKey_End;
        case bimo::kKeyInsert: return ImGuiKey_Insert;
        case bimo::kKeyDelete: return ImGuiKey_Delete;
        default: return ImGuiKey_None;
    }
}

ImVec4 stateColour(LinkState s) {
    switch (s) {
        case LinkState::kLive: return ImVec4(0.20f, 0.75f, 0.30f, 1.0f);
        case LinkState::kStand: return ImVec4(0.85f, 0.65f, 0.15f, 1.0f);
        case LinkState::kBench: return ImVec4(0.55f, 0.60f, 0.70f, 1.0f);
        case LinkState::kRelax: return ImVec4(0.85f, 0.45f, 0.15f, 1.0f);
        default: return ImVec4(0.85f, 0.20f, 0.20f, 1.0f);   // estop/fall/batt
    }
}

// Held for as long as the mouse is down on it -- the same dead-man as the
// key, so the on-screen controls are not a second-class way to drive. The
// result feeds App::mouse_held, resolved with the keyboard in applyHeld().
bool heldButton(const char* label, const ImVec2& size) {
    ImGui::Button(label, size);
    return ImGui::IsItemActive();
}

// Keyboard OR mouse, once per frame, after both have had their say.
void applyHeld(App& a, double now_ms) {
    for (int ax = 0; ax < kAxCountLocal; ++ax) {
        if (a.key_held[ax] || a.mouse_held[ax]) a.in.press(ax, now_ms);
        else a.in.release(ax);
    }
}

void labelFor(const App& a, int action, char* out, size_t cap,
              const char* text) {
    const char* kn = bimo::keyName(a.keys.key[action]);
    if (kn[0]) snprintf(out, cap, "%s [%s]", text, kn);
    else snprintf(out, cap, "%s", text);
}

// Every numeric field is FIXED WIDTH, and the right-hand block starts at an
// absolute x rather than after the left one. This is the line an operator
// checks to decide the link is healthy, and it sits directly above E-STOP:
// at 20 Hz, a `seq` crossing 1000 or a beacon age going 99 -> 100 ms would
// otherwise slide everything to its right and make the whole line shimmer.
// Text that moves has to be re-read instead of glanced at.
void drawHeader(App& a, double now_ms) {
    const float rx = 300.0f;              // where the robot half always starts
    ImGui::Text("-> %-21s tx %2.0f Hz", a.host_port, a.o.rate_hz);
    ImGui::SameLine(rx);
    if (!a.link.have_tlm) {
        ImGui::TextDisabled("<- :%-5d no telemetry yet", a.o.tlm_port);
    } else if (a.link.linkLost(now_ms)) {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.2f, 1.0f),
                           "<- :%-5d LINK LOST  %6.1f s ago", a.o.tlm_port,
                           a.link.tlmAgeMs(now_ms) / 1000.0);
    } else {
        // Ages and counters are clamped to their field, so a long session or a
        // stalled beacon cannot widen the line either.
        const double age = a.link.tlmAgeMs(now_ms);
        const long lag = a.link.tlm.state == LinkState::kBench ? 0 : a.link.lag();
        ImGui::Text("<- :%-5d beacon %4.0f ms %5.1f Hz   seq %8u  lag %4ld  "
                    "bad %4u",
                    a.o.tlm_port, age > 9999.0 ? 9999.0 : age,
                    a.link.rate_hz > 999.9 ? 999.9 : a.link.rate_hz,
                    a.link.seq % 100000000u,
                    lag > 9999 ? 9999 : (lag < -999 ? -999 : lag),
                    a.link.tlm_bad % 10000u);
    }

    char lbl[64];
    labelFor(a, bimo::kActArm, lbl, sizeof lbl,
             a.in.armed ? "DISARM" : "ARM");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.toggleArm(now_ms);
    ImGui::SameLine();
    labelFor(a, bimo::kActStand, lbl, sizeof lbl, "STAND");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.stand("stand");
    ImGui::SameLine();
    // All three states, or ImGui falls back to its default blue the instant
    // the mouse crosses the button -- and a blue E-STOP is a lie.
    const ImVec4 red = a.in.estop ? ImVec4(0.80f, 0.10f, 0.10f, 1.0f)
                                  : ImVec4(0.55f, 0.12f, 0.12f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, red);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(red.x + 0.15f, red.y, red.z, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          ImVec4(red.x + 0.25f, red.y, red.z, 1.0f));
    labelFor(a, bimo::kActEstop, lbl, sizeof lbl,
             a.in.estop ? "E-STOP LATCHED" : "E-STOP");
    if (ImGui::Button(lbl, ImVec2(240, 34))) a.in.fireEstop();
    ImGui::PopStyleColor(3);

    ImGui::SameLine(0.0f, 32.0f);
    if (a.link.have_tlm && !a.link.linkLost(now_ms)) {
        const LinkState s = a.link.tlm.state;
        ImGui::PushStyleColor(ImGuiCol_Button, stateColour(s));
        ImGui::Button(bimo::stateName(s), ImVec2(250, 34));
        ImGui::PopStyleColor();
    } else {
        ImGui::BeginDisabled();
        ImGui::Button("(no beacon)", ImVec2(250, 34));
        ImGui::EndDisabled();
    }

    // The robot's own answer to "why is nothing happening?", drawn whenever
    // the beacon says BENCH -- not only after an arm attempt. An uncalibrated
    // robot is diagnosable on sight (docs/control-channel.md, 2026-08-30).
    if (a.link.have_tlm && !a.link.linkLost(now_ms) &&
        a.link.tlm.state == LinkState::kBench) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "BENCH: %s",
                           diagReason(telemetryDiag(a.link.tlm)));
    }
    if (a.in.note[0]) ImGui::TextUnformatted(a.in.note);
    if (a.status[0]) ImGui::TextDisabled("%s", a.status);
}

void drawSim(App& a, double now_ms, float panel_w) {
    ImGui::SeparatorText("SIM");
    bool mir = a.mirror;
    if (ImGui::Checkbox("mirror the robot", &mir)) setMirror(a, mir);
    ImGui::SameLine();
    if (a.mirror) {
        const bool fed = a.link.have_tlm &&
                         a.link.tlm.n_joints == linkproto::kNumJoints;
        if (fed) {
            ImGui::TextColored(ImVec4(0.25f, 0.8f, 0.35f, 1.0f),
                               "ghost live -- %u poses", a.poses_sent);
        } else {
            // A benched loop measures nothing, so it publishes no pose. Say
            // which of the two it is rather than leaving "no ghost" ambiguous.
            const bool benched = a.link.have_tlm &&
                                 a.link.tlm.state == LinkState::kBench;
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "%s",
                               benched
                                   ? "asked -- but a BENCHED robot measures "
                                     "nothing; the ghost appears on arm"
                                   : "asked for joint angles; none yet");
        }
    } else {
        ImGui::TextDisabled("drive the robot, sim follows as a ghost");
    }
    char serr[192];
    a.stream.error(serr, sizeof serr);
    const bool external = !a.sim.running() && a.stream.frames() > 0;
    // A child that died must SAY so, loudly, and outrank the stream's patient
    // "waiting for a stream at ...". Not doing that is how a sil_twin that
    // exited on its first line (macOS has no EGL) looked exactly like a sim
    // that was merely slow to start -- for both the operator and the person
    // who wrote this.
    const bool sim_failed = !a.sim.running() && a.sim.err[0] != 0;

    if (a.sim.running()) {
        // Ours: say what sil_twin itself reported it was running, rather than
        // reading it back off the controls.
        ImGui::TextUnformatted(a.sim.ident[0] ? a.sim.ident : "starting ...");
        if (ImGui::Button("Stop")) stopSim(a);
        ImGui::SameLine();
        // Re-rolls the episode in place. Restarting the process costs ~30 s of
        // MuJoCo and policy loading -- long enough that nobody would.
        if (ImGui::Button("Reset")) {
            if (a.sim.tell("reset")) {
                a.in.stand("sim episode reset");
                snprintf(a.status, sizeof a.status, "sim reset");
            } else {
                snprintf(a.status, sizeof a.status, "the sim is not listening");
            }
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", a.sim_url);
    } else if (external) {
        // A stream this console did not start. The controls below would
        // describe a different sim than the one rendering, which is worse
        // than no controls at all -- so they say so first.
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "external stream -- not started by this console");
        ImGui::TextDisabled("%s  (what it runs is not carried by the stream)",
                            a.sim_url);
        if (ImGui::Button("replace with...")) startSim(a);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(panel_w * 0.45f);
        if (a.runs.n > 0 && ImGui::BeginCombo("##run", a.runs.name[a.run_sel])) {
            for (int i = 0; i < a.runs.n; ++i) {
                if (ImGui::Selectable(a.runs.name[i], i == a.run_sel)) {
                    a.run_sel = i;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::Checkbox("hang", &a.sim_hang);
    } else {
        ImGui::SetNextItemWidth(panel_w * 0.55f);
        if (a.runs.n > 0) {
            if (ImGui::BeginCombo("policy", a.runs.name[a.run_sel])) {
                for (int i = 0; i < a.runs.n; ++i) {
                    if (ImGui::Selectable(a.runs.name[i], i == a.run_sel)) {
                        a.run_sel = i;
                    }
                }
                ImGui::EndCombo();
            }
        } else {
            ImGui::TextDisabled("no runs with SIL weights under %s/sim/runs",
                                a.repo);
        }
        ImGui::TextDisabled("the trained brain the sim will run "
                            "(sim/runs/, needs exported SIL weights)");
        ImGui::Checkbox("hang: weld the torso, feet free (test stand)",
                        &a.sim_hang);
        ImGui::SameLine();
        if (ImGui::Button("Start")) startSim(a);
    }

    // The frame, or an honest reason there is no frame.
    const float w = panel_w - 24.0f;
    const float h = a.tex_h > 0 ? w * static_cast<float>(a.tex_h) /
                                      static_cast<float>(a.tex_w)
                                : w * 0.7f;
    if (a.tex != 0) {
        ImGui::Image(static_cast<ImTextureID>(a.tex), ImVec2(w, h));
        const double age = now_ms - a.stream.lastFrameMs();
        if (age > kStreamStaleMs) {
            // A frozen frame that looks live is the failure tools/cam_live.py
            // had to fix; say so over the picture rather than let it lie.
            ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.25f, 1.0f),
                               "STALE  no frame for %.1f s", age / 1000.0);
        } else {
            ImGui::TextDisabled("%u frames", a.stream.frames());
        }
    } else {
        ImGui::Dummy(ImVec2(w, h * 0.4f));
        if (sim_failed) {
            ImGui::TextColored(ImVec4(0.95f, 0.25f, 0.20f, 1.0f),
                               "the sim %s", a.sim.err);
            ImGui::TextDisabled("its last words are in the log below");
        } else if (serr[0]) {
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f), "%s", serr);
        } else if (a.stream.running()) {
            ImGui::TextDisabled("connecting to %s ...", a.sim_url);
        } else {
            ImGui::TextDisabled("no stream -- press Start, or pass --sim-view");
        }
        ImGui::Dummy(ImVec2(w, h * 0.4f));
    }

    // Opened for you when it failed: the reason is always in there.
    if (sim_failed) ImGui::SetNextItemOpen(true, ImGuiCond_Always);
    if (a.sim.lines() > 0 && ImGui::CollapsingHeader("sil_twin log")) {
        ImGui::BeginChild("simlog", ImVec2(0, 110), ImGuiChildFlags_Borders);
        for (int i = 0; i < a.sim.lines(); ++i) {
            ImGui::TextUnformatted(a.sim.line(i));
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }
}

void drawDrive(App& a) {
    for (int i = 0; i < kAxCountLocal; ++i) a.mouse_held[i] = false;
    ImGui::SeparatorText("DRIVE");
    ImGui::SameLine();
    ImGui::Text("on the wire: %zu B", a.link.last_len);
    if (a.link.last_len == kCmdLenExt) {
        ImGui::SameLine();
        // Old firmware drops a long frame BY LENGTH, so the symptom of an
        // out-of-date robot is that the sliders make it go quiet.
        ImGui::TextDisabled("(extended frame: needs firmware >= 2026-08-31)");
    }

    const ImVec2 sz(104, 40);
    const float x0 = ImGui::GetCursorPosX() + 40.0f;
    char lbl[64];

    ImGui::SetCursorPosX(x0 + sz.x + 8.0f);
    labelFor(a, bimo::kActForward, lbl, sizeof lbl, "fwd");
    if (heldButton(lbl, sz)) a.mouse_held[bimo::kAxForward] = true;

    labelFor(a, bimo::kActStrafeLeft, lbl, sizeof lbl, "<-");
    ImGui::SetCursorPosX(x0);
    if (heldButton(lbl, sz)) a.mouse_held[bimo::kAxLeft] = true;
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActBack, lbl, sizeof lbl, "back");
    if (heldButton(lbl, sz)) a.mouse_held[bimo::kAxBack] = true;
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActStrafeRight, lbl, sizeof lbl, "->");
    if (heldButton(lbl, sz)) a.mouse_held[bimo::kAxRight] = true;
    ImGui::SameLine(0.0f, 24.0f);
    ImGui::BeginGroup();
    ImGui::Text("vx  %+.2f m/s", static_cast<double>(a.vx));
    ImGui::Text("vy  %+.2f m/s", static_cast<double>(a.vy));
    ImGui::Text("wz  %+.2f rad/s", static_cast<double>(a.wz));
    ImGui::EndGroup();

    ImGui::SetCursorPosX(x0);
    labelFor(a, bimo::kActTurnLeft, lbl, sizeof lbl, "turn L");
    if (heldButton(lbl, ImVec2(160, 34))) a.mouse_held[bimo::kAxTurnLeft] = true;
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActTurnRight, lbl, sizeof lbl, "turn R");
    if (heldButton(lbl, ImVec2(160, 34))) a.mouse_held[bimo::kAxTurnRight] = true;

    ImGui::SeparatorText("EXTENDED CHANNELS");
    ImGui::SameLine();
    labelFor(a, bimo::kActResetExt, lbl, sizeof lbl, "reset -> 14 B");
    if (ImGui::Button(lbl)) {
        a.in.ext.reset();
        a.in.say("extended channels back to defaults (14 B frames)");
    }
    bool changed = false;
    changed |= ImGui::SliderFloat("crouch", &a.in.ext.crouch, kCrouchMin, 1.0f,
                                  "%.2f");
    int lift = a.in.ext.lift < -0.5f ? 0 : (a.in.ext.lift > 0.5f ? 2 : 1);
    ImGui::TextUnformatted("lift  ");
    ImGui::SameLine();
    changed |= ImGui::RadioButton("left##lift", &lift, 0);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("none##lift", &lift, 1);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("right##lift", &lift, 2);
    a.in.ext.lift = static_cast<float>(lift - 1);
    changed |= ImGui::SliderFloat("foot_dx", &a.in.ext.foot_dx, -kFootDMax,
                                  kFootDMax, "%+.3f m");
    changed |= ImGui::SliderFloat("foot_dz", &a.in.ext.foot_dz, -kFootDMax,
                                  kFootDMax, "%+.3f m");
    if (changed) a.in.ext.clamp();
}

void plot(const char* label, const Ring& r, float lo, float hi,
          const char* fmt, float last) {
    char overlay[64];
    snprintf(overlay, sizeof overlay, fmt, static_cast<double>(last));
    // Only the samples actually taken. Plotting the whole ring while it fills
    // draws the untouched zeros as a step down to the floor, which reads as a
    // battery that just collapsed.
    const int cnt = r.n;
    const int off = (r.n < kChart) ? 0 : r.off;
    ImGui::PlotLines(label, r.v, cnt, off, overlay, lo, hi,
                     ImVec2(-260.0f, 42.0f));
}

void drawTelemetry(App& a, double now_ms) {
    ImGui::SeparatorText("TELEMETRY  (last 30 s)");
    if (!a.link.have_tlm) {
        ImGui::TextDisabled("nothing to plot until the robot beacons");
        return;
    }
    const Telemetry& t = a.link.tlm;
    ImGui::SameLine();
    if (t.servo_err) {
        char bits[16];
        for (int b = 7; b >= 0; --b) bits[7 - b] = (t.servo_err >> b) & 1 ? '1' : '0';
        bits[8] = 0;
        ImGui::TextColored(ImVec4(0.95f, 0.25f, 0.20f, 1.0f),
                           "servo FAULT %s (bit i = ID i+1)", bits);
    } else {
        ImGui::TextDisabled("servo ok    loop %u%% late ticks", t.loop_late_pct);
    }
    plot("vbat", a.c_vbat, 9.0f, 13.0f, "%.2f V",
         t.vbat_v);
    plot("up_z", a.c_upz, 0.0f, 1.05f, "%.2f", t.up_z);
    plot("vx cmd", a.c_vx_cmd, -1.1f, 1.1f, "%+.2f", a.vx);
    plot("vx est", a.c_vx_est, -1.1f, 1.1f, "%+.2f", t.vx_est);
    plot("wz cmd", a.c_wz_cmd, -1.1f, 1.1f, "%+.2f", a.wz);
    plot("wz est", a.c_wz_est, -1.1f, 1.1f, "%+.2f", t.wz_est);
    (void)now_ms;
}

void drawSession(App& a, double now_ms) {
    ImGui::Separator();
    char lbl[64];
    labelFor(a, bimo::kActRecord, lbl, sizeof lbl,
             a.rec.active() ? "stop recording" : "record");
    if (ImGui::Button(lbl)) toggleRecord(a, now_ms);
    ImGui::SameLine();
    if (a.rec.active()) {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f),
                           "REC  %s   %u cmd - %u tlm", a.rec.path, a.rec.n_cmd,
                           a.rec.n_tlm);
    } else {
        ImGui::TextDisabled("not recording (--no-record was passed) -> %s",
                            a.sessions);
    }
    ImGui::SameLine();
    if (ImGui::Button(a.show_keymap ? "hide keys" : "keys")) {
        a.show_keymap = !a.show_keymap;
        a.rebind = -1;
    }
}

void drawKeymap(App& a) {
    ImGui::SeparatorText("KEYMAP");
    ImGui::TextDisabled("%s -- click a binding, then press a key",
                        a.keymap_path[0] ? a.keymap_path : "(no file)");
    for (int i = 0; i < bimo::kActCount; ++i) {
        if (i % 2 != 0) ImGui::SameLine(340.0f);
        ImGui::Text("%-18s", bimo::actionLabel(i));
        ImGui::SameLine();
        const char* kn = bimo::keyName(a.keys.key[i]);
        char btn[64];
        snprintf(btn, sizeof btn, "%-10s##bind%d",
                 a.rebind == i ? "press a key" : (kn[0] ? kn : "none"), i);
        if (ImGui::Button(btn, ImVec2(120, 0))) a.rebind = (a.rebind == i) ? -1 : i;
    }
    if (ImGui::Button("restore defaults")) {
        a.keys.reset();
        a.rebind = -1;
    }
    ImGui::SameLine();
    if (ImGui::Button("save")) {
        char err[192] = "";
        if (a.keys.save(a.keymap_path, err, sizeof err)) {
            snprintf(a.status, sizeof a.status, "keymap saved -> %s",
                     a.keymap_path);
        } else {
            snprintf(a.status, sizeof a.status, "%s", err);
        }
        a.rebind = -1;
    }
}

// Keys -> intent. Polled per frame rather than driven by callbacks, so "held"
// is literally "the key is down right now": there is no way for a release to
// be missed and leave the robot walking.
void pumpKeys(App& a, GLFWwindow* win, double now_ms) {
    ImGuiIO& io = ImGui::GetIO();
    const bool focused = glfwGetWindowAttrib(win, GLFW_FOCUSED) != 0;

    if (a.rebind >= 0) {
        for (int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; ++k) {
            const ImGuiKey key = static_cast<ImGuiKey>(k);
            if (!ImGui::IsKeyPressed(key, false)) continue;
            const int code = toKeyCode(key);
            if (code == bimo::kKeyNone) continue;
            a.keys.bind(a.rebind, code);
            snprintf(a.status, sizeof a.status, "%s -> %s",
                     bimo::actionLabel(a.rebind), bimo::keyName(code));
            a.rebind = -1;
            break;
        }
        for (int i = 0; i < kAxCountLocal; ++i) a.key_held[i] = false;
        return;
    }

    // The window lost focus (or a text field has the keyboard) with a key
    // held: a window can do that, and a terminal cannot. Treat it as every
    // key up, this instant.
    if (!focused || io.WantCaptureKeyboard) {
        for (int i = 0; i < kAxCountLocal; ++i) a.key_held[i] = false;
        return;
    }

    for (int act = 0; act < bimo::kMotionActions; ++act) {
        const ImGuiKey key = fromKeyCode(a.keys.key[act]);
        a.key_held[act] = key != ImGuiKey_None && ImGui::IsKeyDown(key);
    }
    for (int act = bimo::kMotionActions; act < bimo::kActCount; ++act) {
        const ImGuiKey key = fromKeyCode(a.keys.key[act]);
        if (key != ImGuiKey_None && ImGui::IsKeyPressed(key, false)) {
            fireAction(a, act, now_ms);
        }
    }
}

void refreshTexture(App& a) {
    uint8_t* jpeg = nullptr;
    size_t len = 0;
    if (!a.stream.latest(&a.frame_seq, &jpeg, &len)) return;
    int w = 0, h = 0;
    unsigned char* px = bimo::decodeJpeg(jpeg, len, &w, &h);
    bimo::MjpegStream::freeFrame(jpeg);
    if (px == nullptr) return;
    if (a.tex == 0) {
        glGenTextures(1, &a.tex);
        glBindTexture(GL_TEXTURE_2D, a.tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    } else {
        glBindTexture(GL_TEXTURE_2D, a.tex);
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 px);
    a.tex_w = w;
    a.tex_h = h;
    bimo::freePixels(px);
}

// The console photographs its own back buffer. Deliberately not
// `screencapture`: that needs Screen Recording rights, which is a strange
// thing for a robot console to ask for, and it cannot run unattended. Reading
// our own framebuffer needs nothing, and makes the figure in the docs
// regenerable from a command line like every other render in this repo.
bool grabWindow(const char* path, int w, int h) {
    if (w <= 0 || h <= 0) return false;
    const size_t stride = static_cast<size_t>(w) * 4;
    unsigned char* px = static_cast<unsigned char*>(
        malloc(stride * static_cast<size_t>(h)));
    if (px == nullptr) return false;
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px);
    // GL's origin is bottom-left and a PNG's is top-left.
    unsigned char* row = static_cast<unsigned char*>(malloc(stride));
    if (row == nullptr) {
        free(px);
        return false;
    }
    for (int y = 0; y < h / 2; ++y) {
        unsigned char* top = px + static_cast<size_t>(y) * stride;
        unsigned char* bot = px + static_cast<size_t>(h - 1 - y) * stride;
        memcpy(row, top, stride);
        memcpy(top, bot, stride);
        memcpy(bot, row, stride);
    }
    free(row);
    const bool ok = bimo::writePng(path, w, h, px);
    free(px);
    return ok;
}

int runWindow(App& a) {
    glfwSetErrorCallback(nullptr);
    if (!glfwInit()) {
        logEvent(a, "FATAL", "cannot open a window (glfwInit failed); "
                             "--headless drives the same link without one");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    char title[128];
    snprintf(title, sizeof title, "bimo_gui  ->  %s", a.o.host);
    GLFWwindow* win = glfwCreateWindow(1280, 860, title, nullptr, nullptr);
    if (win == nullptr) {
        glfwTerminate();
        logEvent(a, "FATAL", "cannot create a window");
        return 1;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;   // no stray imgui.ini in the repo
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    const double period_ms = 1000.0 / a.o.rate_hz;
    const double t0_ms = nowMs();
    double next_link_ms = t0_ms;
    bool shot_taken = false;

    while (!a.quit && !glfwWindowShouldClose(win)) {
        glfwPollEvents();
        const double now_ms = nowMs();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        pumpKeys(a, win, now_ms);
        if (!isatty(STDIN_FILENO)) pumpStdin(a, now_ms);   // scripted figures
        refreshTexture(a);

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("bimo", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoBringToFrontOnFocus);

        drawHeader(a, now_ms);
        ImGui::Separator();

        const float sim_w = vp->WorkSize.x * 0.42f;
        const float mid_h = vp->WorkSize.y * 0.46f;
        ImGui::BeginChild("sim", ImVec2(sim_w, mid_h));
        drawSim(a, now_ms, sim_w);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("drive", ImVec2(0, mid_h));
        drawDrive(a);
        ImGui::EndChild();

        if (a.show_keymap) {
            drawKeymap(a);
        } else {
            drawTelemetry(a, now_ms);
        }
        drawSession(a, now_ms);

        ImGui::End();

        // Only now are BOTH inputs for this frame known: the keyboard from
        // pumpKeys and the mouse from the buttons just drawn. Resolve them,
        // then let the link have its 20 Hz tick -- which must keep its own
        // cadence regardless of how fast this draws.
        applyHeld(a, now_ms);
        if (now_ms >= next_link_ms) {
            tick(a, now_ms);
            next_link_ms += period_ms;
            if (next_link_ms < now_ms) next_link_ms = now_ms;
        }

        ImGui::Render();
        int fb_w, fb_h;
        glfwGetFramebufferSize(win, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Before the swap: this is the frame that was just drawn.
        if (a.o.screenshot != nullptr && !shot_taken &&
            now_ms - t0_ms >= a.o.screenshot_after * 1000.0) {
            shot_taken = true;
            if (grabWindow(a.o.screenshot, fb_w, fb_h)) {
                printf("bimo_gui: wrote %s (%dx%d)\n", a.o.screenshot, fb_w,
                       fb_h);
            } else {
                logEvent(a, "ERROR", "could not write %s", a.o.screenshot);
            }
            fflush(stdout);
            a.quit = true;
        }
        glfwSwapBuffers(win);
    }

    // A window closed with the mouse must disarm exactly like [q] does; that
    // happens in main(), after this returns.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // A console that dies takes the link with it and never sends the disarm,
    // so nothing routine is allowed to kill it. Writing to the sim child's
    // stdin after that child has died raises SIGPIPE, whose default action is
    // exactly that -- on a robot that may be armed. Ignore it here and check
    // write() for EPIPE at the one place that writes (SimProc::tell).
    signal(SIGPIPE, SIG_IGN);
    Options o;
    if (!parse(argc, argv, o)) {
        usage(argv[0]);
        return 2;
    }
    App a;
    a.o = o;
    snprintf(a.host_port, sizeof a.host_port, "%s:%d", o.host, o.cmd_port);
    if (!a.link.open(o.host, o.cmd_port, o.tlm_port)) {
        fprintf(stderr, "bimo_gui: link: %s (host %s, tlm port %d)\n",
                strerror(errno), o.host, o.tlm_port);
        return 1;                      // before the log exists: stderr only
    }
    if (o.repo != nullptr) {
        snprintf(a.repo, sizeof a.repo, "%s", o.repo);
    } else {
        bimo::guessRepoRoot(argv[0], a.repo, sizeof a.repo);
    }
    if (o.sessions != nullptr) {
        snprintf(a.sessions, sizeof a.sessions, "%s", o.sessions);
    } else {
        snprintf(a.sessions, sizeof a.sessions, "%s/hw_sessions", a.repo);
    }
    if (o.keymap_path != nullptr) {
        snprintf(a.keymap_path, sizeof a.keymap_path, "%s", o.keymap_path);
    } else {
        bimo::defaultKeymapPath(a.keymap_path, sizeof a.keymap_path);
    }
    openLog(a);
    logEvent(a, "start", "bimo_gui -> %s (telemetry :%d), repo %s",
             a.host_port, o.tlm_port, a.repo);
    char err[192] = "";
    if (a.keymap_path[0] && !a.keys.load(a.keymap_path, err, sizeof err)) {
        logEvent(a, "FATAL", "keymap: %s", err);
        return 2;
    }
    a.sim_cmd_port = o.sim_cmd_port ? o.sim_cmd_port : o.cmd_port + 100;
    a.sim_tlm_port = a.sim_cmd_port + 1;
    a.runs.scan(a.repo);
    logEvent(a, "runs", "%d policy run(s) with SIL weights under %s/sim/runs",
             a.runs.n, a.repo);
    if (o.run_name != nullptr) {
        const int i = a.runs.indexOf(o.run_name);
        if (i >= 0) a.run_sel = i;
    }
    if (o.autostream) {
        simViewUrl(a, a.sim_url, sizeof a.sim_url, false);
        a.stream.start(a.sim_url);
    }
    // Record by default. A take you forgot to arm the recorder for is a take
    // you cannot compare against a sim eval afterwards, and the frames are
    // cheap: ~90 bytes each at 20 Hz, so an hour of driving is a few MB.
    // --no-record opts out.
    if (o.record) toggleRecord(a, bimo::nowMs());
    // --mirror is a mode AND a subject. Mirroring nothing is a console that
    // asks the robot for joint angles and then draws them nowhere: the pose
    // relay is gated on a running child (tick()), the extended beacon is not
    // recorded, and the relayed commands go to a port no one is bound to. So
    // the flag starts the sim it means to mirror against -- the --run-name
    // run, or the first one with SIL weights -- the same as pressing Start.
    // --sim-view is the operator saying they already have one; leave it be.
    if (o.mirror) {
        setMirror(a, true);
        logEvent(a, "mirror", "%s", a.status);
        if (!o.autostream) {
            startSim(a);
            logEvent(a, "mirror", "%s", a.status);
            printf("note: %s\n", a.status);
            fflush(stdout);
        }
    }

    const int rc = a.o.headless ? runHeadless(a) : runWindow(a);

    // Leave a limp robot, not a standing one nobody is watching: drop ARM (the
    // 1 -> 0 edge benches it) and repeat, since any single datagram may vanish.
    for (int i = 0; i < 10; ++i) {
        a.link.send(0.0f, 0.0f, 0);
        usleep(20 * 1000);
    }
    if (a.rec.active()) {
        logEvent(a, "rec", "%u cmd / %u tlm -> %s", a.rec.n_cmd, a.rec.n_tlm,
                 a.rec.path);
        a.rec.stop();
    }
    stopSim(a);
    a.link.close();
    logEvent(a, "quit", "sent disarm (%u frames total)", a.link.sent);
    if (a.logf != nullptr) fclose(a.logf);
    return rc;
}
