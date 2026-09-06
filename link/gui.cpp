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
// What the send-rate slider may reach. The floor is the real constraint: the
// robot's watchdog calls a link stale after kStaleMs (250 ms) and decays to
// the trained stand, so anything at or below 4 Hz would make the robot stand
// on its own while the console believed it was driving. 5 Hz keeps a whole
// period of margin; the ceiling is well past any useful cadence.
const double kRateMin = 5.0;
const double kRateMax = 100.0;

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
    // OBSERVER: watch a robot somebody else is driving. See the block comment
    // above `readonly` in link/client.h -- no transmit socket is opened.
    bool readonly = false;
    // Whether --host was really given, as opposed to the placeholder a
    // watcher is allowed to start with. Only a real one can be taken control
    // of; see Link::has_dest.
    bool host_given = false;
    const char* watch = nullptr;         // HOST[:PORT] to fan telemetry out to
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
            "          [--readonly] START watching; no transmit socket. The\n"
            "                       TAKE CONTROL button (or [t]) opens one\n"
            "          [--watch HOST[:PORT]]  relay each beacon to a watcher\n"
            "          [--screenshot FILE [--screenshot-after S]]\n"
            "\n"
            "  --readonly needs a beacon to watch, and the robot beacons only\n"
            "  to whoever commanded it last. So the DRIVING console forwards:\n"
            "    driver:   bimo_gui --host ROBOT --watch 192.168.2.30\n"
            "    watcher:  bimo_gui --readonly --host ROBOT\n",
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
        else if (!strcmp(a, "--readonly")) { o.readonly = true; }
        else if (!strcmp(a, "--watch") && v) { o.watch = v; ++i; }
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
    // --host is where commands GO, so an observer does not need one -- but it
    // is still the best guess for where the sim stream is, so it stays
    // accepted. Only its being REQUIRED is lifted.
    o.host_given = o.host != nullptr;
    if (o.host == nullptr && o.readonly) o.host = "0.0.0.0";
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

    // host_port is a display string ("host:port"); sized to hold a 63-byte
    // host (ed_host's bound) plus ":port".
    char host_port[80] = "";
    char repo[512] = ".";
    // sessions defaults to <repo>/hw_sessions, so it must hold repo (512)
    // plus the 12-byte suffix.
    char sessions[544] = "hw_sessions";
    char keymap_path[512] = "";
    char sim_url[512] = "";
    // status is a one-line message shown in the window and logged. It embeds
    // 512-byte paths (repo, sessions, keymap, sim_url, rec.path), so it is
    // sized to hold the largest such message -- repo + sessions + the fixed
    // text -- rather than truncate a long path (gcc -Wformat-truncation).
    char status[1152] = "";
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
    char last_status[1152] = "";
    char last_sim_err[544] = "";
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

    // -- SETTINGS: every command-line flag, editable in the window ----------
    // Edited into buffers and APPLIED on a button, not bound live to the
    // fields they set. Typing "192.168.2.9" on the way to ".90" would
    // otherwise relink to a robot that does not exist, halfway through the
    // keystroke -- and the ones that rebind a socket or restart a stream have
    // to happen once, deliberately, not per character. The live-safe ones
    // (speeds, rate) are bound directly and say so.
    bool show_settings = false;
    char ed_host[64] = "";
    int ed_cmd_port = 0, ed_tlm_port = 0;
    char ed_watch[80] = "";
    bool ed_watch_on = false;
    char ed_sim_view[512] = "";
    int ed_stream_port = 0, ed_sim_cmd_port = 0;
    char ed_repo[512] = "";
    // Matches a.sessions (repo + "/hw_sessions"), so the round-trip copy
    // cannot truncate.
    char ed_sessions[544] = "";
    char ed_keymap[512] = "";
    char ed_shot[512] = "shot.png";
    bool shot_now = false;               // the render loop grabs on next frame

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
    a.in.checkHome(a.link, now_ms);
    bimo::sendIntent(a.link, a.in, a.o.speeds, a.vx, a.vy, a.wz, a.ext_sent,
                     a.flags);
    // Command rows are what this console PUT ON THE WIRE. An observer put
    // nothing there, so it records telemetry only: a take with 20 Hz of
    // invented zero-velocity commands beside the driver's real motion would
    // read, later, as a robot that moved while being told to stand.
    if (!a.link.readonly) {
        a.rec.command(now_ms, a.link.seq - 1, a.vx, a.vy, a.wz, a.ext_sent,
                      a.flags, a.link.last_len);
    }
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
            // The up vector rides on the same line, and only when the robot
            // actually sent one: appended, never defaulted. A viewer handed
            // (0,0,1) it was not told would draw a fallen robot standing.
            if (t.have_att) {
                snprintf(line + at, sizeof line - static_cast<size_t>(at),
                         " %.4f %.4f %.4f", static_cast<double>(t.up_x),
                         static_cast<double>(t.up_y),
                         static_cast<double>(t.up_z));
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
    // own; otherwise it listens where the console is already pointed. An
    // observer is in the first case whether or not --mirror was given: it is
    // bound to the beacon port to hear the real robot, so a sim beaconing
    // there too would interleave a second robot's telemetry into the same
    // charts.
    const bool own_ports = a.mirror || a.link.readonly;
    const int cp = own_ports ? a.sim_cmd_port : a.o.cmd_port;
    const int tp = own_ports ? a.sim_tlm_port : a.o.tlm_port;
    // Mirroring => a VIEWER. Not a second robot running beside the real one:
    // a picture of the real one, posed from its own telemetry.
    if (!a.sim.start(a.repo, a.runs.name[a.run_sel], a.sim_hang, a.mirror,
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
    a.in.want_att = on;
    if (on && a.link.readonly) {
        // The observer's mirror: no command is relayed (there is no transmit
        // socket) and kFlagPose is never asked for (that too would be a frame
        // on the wire). What still works is the half that matters -- joint
        // angles arriving in the relayed beacon are pushed to the sim and
        // drawn, so the watcher sees the robot MOVE in 3D rather than as
        // numbers. That depends on the DRIVER having asked for pose, which is
        // exactly what `bimo_gui --mirror` on the driving side does, so say
        // so rather than leave an empty ghost looking like a broken sim.
        snprintf(a.status, sizeof a.status,
                 "MIRROR (observer): drawing the robot's measured joints in "
                 "the sim. Needs the driving console to be in mirror mode too "
                 "-- only then does the beacon carry joint angles.");
    } else if (on) {
        // No command relay any more (Tom, 2026-09-03): mirror mode is for
        // SEEING the robot, not for running a second one beside it. Relaying
        // the command was what made the sim a simulation -- it stepped its own
        // physics from our vx/wz and drifted away from the robot in silence,
        // and the "ghost" was a 60/40 blend of the two, so the picture was
        // part measurement and part guess with no way to tell which.
        // Now the sim is a mannequin: it is posed from the robot's measured
        // joints and nothing else.
        snprintf(a.status, sizeof a.status,
                 "MIRROR: the sim is a VIEWER -- posed from %s's measured "
                 "joints, no physics, no policy. Asking for joint angles.",
                 a.o.host);
    } else {
        snprintf(a.status, sizeof a.status, "mirror off");
    }
}

// WATCHING <-> DRIVING. The socket is what changes; everything else follows.
void toggleControl(App& a, double now_ms) {
    if (!a.link.readonly) {
        a.link.goReadonly();
        a.in.readonly = true;
        a.in.releaseAll();
        // NOT disarmed on the way out -- see Link::goReadonly. The arm level
        // this console was holding is simply no longer this console's to
        // hold; whoever is still driving holds theirs, and if nobody is, the
        // robot's watchdog stands it in kStaleMs.
        snprintf(a.status, sizeof a.status,
                 "WATCHING: transmit socket closed. The robot is left exactly "
                 "as it is -- no parting disarm, so a console still driving "
                 "it keeps it.");
        logEvent(a, "readonly", "%s", a.status);
        setMirror(a, a.mirror);          // the mode's meaning changed under it
        return;
    }
    if (!a.link.takeControl()) {
        snprintf(a.status, sizeof a.status,
                 "cannot take control: started without --host, so there is no "
                 "robot address to command. (The beacon's sender is the "
                 "console forwarding it, not the robot.)");
        return;
    }
    a.in.adopt(a.link, now_ms);
    snprintf(a.status, sizeof a.status, "DRIVING %s -- this console is now a "
             "commander, and the robot beacons here instead", a.o.host);
    logEvent(a, "control", "%s | %s", a.status, a.in.note);
    setMirror(a, a.mirror);
}

// Actions that are not a held axis.
void fireAction(App& a, int action, double now_ms) {
    switch (action) {
        case bimo::kActArm: a.in.toggleArm(now_ms); break;
        case bimo::kActStand: a.in.stand("stand"); break;
        case bimo::kActEstop: a.in.fireEstop(); break;
        case bimo::kActHome: a.in.requestHome(now_ms); break;
        case bimo::kActQuit: a.quit = true; break;
        case bimo::kActRecord: toggleRecord(a, now_ms); break;
        case bimo::kActControl: toggleControl(a, now_ms); break;
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
    snprintf(fl, sizeof fl, "%s%s%s%s", (a.flags & kFlagArm) ? "ARM " : "",
             (a.flags & kFlagEnable) ? "ENABLE " : "",
             (a.flags & kFlagEstop) ? "ESTOP " : "",
             (a.flags & kFlagHome) ? "HOME " : "");
    if (a.link.readonly) {
        // No send half at all: there is nothing to report about it, and
        // printing zeros would read as "commanding a stop", which is the one
        // thing an observer must never look like it is doing.
        printf("[%7.2fs] OBSERVER  watched %5u beacon(s) %5.1f Hz",
               (now_ms - t0_ms) / 1000.0, a.link.tlm_count,
               a.link.rate_hz > 999.9 ? 999.9 : a.link.rate_hz);
    } else {
    printf("[%7.2fs] arm %-8s motion %-12s send vx %+.2f vy %+.2f wz %+.2f "
           "crouch %.2f len %zu flags %s",
           (now_ms - t0_ms) / 1000.0, a.in.armed ? "ARMED" : "disarmed",
           bimo::motionText(a.in), static_cast<double>(a.vx),
           static_cast<double>(a.vy), static_cast<double>(a.wz),
           static_cast<double>(a.ext_sent.crouch), a.link.last_len,
           fl[0] ? fl : "(none)");
    }
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
    } else if (!strcmp(verb, "panel")) {
        // Which pane the middle of the window shows. Exists for the same
        // reason `sim start` does: a figure worth putting in the docs has to
        // be reproducible from a command line rather than from someone's
        // hands, and the settings panel is now one of the things worth
        // showing.
        if (arg != nullptr && !strcmp(arg, "settings")) {
            a.show_settings = true;
            a.show_keymap = false;
        } else if (arg != nullptr && !strcmp(arg, "keys")) {
            a.show_keymap = true;
            a.show_settings = false;
        } else if (arg != nullptr && !strcmp(arg, "telemetry")) {
            a.show_keymap = a.show_settings = false;
        } else {
            printf("note: panel settings|keys|telemetry\n");
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
    if (a.link.readonly) {
        printf("bimo_gui --headless --readonly: OBSERVER, no transmit socket; "
               "listening for beacons on :%d\n", a.o.tlm_port);
    } else {
        printf("bimo_gui --headless -> %s:%d (telemetry :%d)\n", a.o.host,
               a.o.cmd_port, a.o.tlm_port);
    }
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
    if (a.link.readonly) {
        // Same column, same width, so the line does not move between modes --
        // and amber, because "you cannot stop this robot" is a standing
        // condition to be aware of, not an error and not normal running.
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "-- %-21s 0 Hz  ", "OBSERVER: watching only");
    } else {
        ImGui::Text("-> %-21s tx %2.0f Hz", a.host_port, a.o.rate_hz);
    }
    ImGui::SameLine(rx);
    if (!a.link.have_tlm) {
        if (a.link.readonly) {
            // The one failure a watcher will actually hit, named precisely.
            // "no telemetry yet" would send an operator hunting a dead radio;
            // the radio is fine, nobody is forwarding to them.
            ImGui::TextDisabled("<- :%-5d nothing forwarded yet -- the "
                                "driving console needs --watch", a.o.tlm_port);
        } else {
            ImGui::TextDisabled("<- :%-5d no telemetry yet", a.o.tlm_port);
        }
    } else if (a.link.linkLost(now_ms)) {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.2f, 1.0f),
                           "<- :%-5d LINK LOST  %6.1f s ago", a.o.tlm_port,
                           a.link.tlmAgeMs(now_ms) / 1000.0);
    } else {
        // Ages and counters are clamped to their field, so a long session or a
        // stalled beacon cannot widen the line either.
        const double age = a.link.tlmAgeMs(now_ms);
        const long lag = a.link.tlm.state == LinkState::kBench ? 0 : a.link.lag();
        if (a.link.readonly) {
            // seq and lag both measure OUR frames against the robot's echo.
            // A watcher has sent none, so lag would be a large negative
            // number that looks like a fault. Report what it does know: how
            // many relayed beacons it has actually seen.
            ImGui::Text("<- :%-5d beacon %4.0f ms %5.1f Hz   seen %8u"
                        "            bad %4u",
                        a.o.tlm_port, age > 9999.0 ? 9999.0 : age,
                        a.link.rate_hz > 999.9 ? 999.9 : a.link.rate_hz,
                        a.link.tlm_count % 100000000u,
                        a.link.tlm_bad % 10000u);
        } else {
        ImGui::Text("<- :%-5d beacon %4.0f ms %5.1f Hz   seq %8u  lag %4ld  "
                    "bad %4u",
                    a.o.tlm_port, age > 9999.0 ? 9999.0 : age,
                    a.link.rate_hz > 999.9 ? 999.9 : a.link.rate_hz,
                    a.link.seq % 100000000u,
                    lag > 9999 ? 9999 : (lag < -999 ? -999 : lag),
                    a.link.tlm_bad % 10000u);
        }
    }

    char lbl[64];
    // WATCH ONLY / TAKE CONTROL. Above the controls it governs, because it is
    // the switch that decides whether any of them mean anything -- and read
    // before them for the same reason. Amber in both directions: handing a
    // robot between two consoles is consequential whichever way it goes, and
    // neither direction is an emergency.
    const bool ro = a.link.readonly;
    const ImVec4 amb(0.62f, 0.42f, 0.06f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, amb);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(amb.x + 0.15f, amb.y + 0.10f, amb.z, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          ImVec4(amb.x + 0.25f, amb.y + 0.18f, amb.z, 1.0f));
    labelFor(a, bimo::kActControl, lbl, sizeof lbl,
             ro ? "TAKE CONTROL" : "WATCH ONLY");
    // Greyed, not hidden, when there is nowhere to send: the operator should
    // see the button they are looking for and be told why it cannot work.
    const bool can_take = !ro || a.link.has_dest;
    if (!can_take) ImGui::BeginDisabled();
    if (ImGui::Button(lbl, ImVec2(240, 30))) toggleControl(a, now_ms);
    if (!can_take) ImGui::EndDisabled();
    ImGui::PopStyleColor(3);
    const bool live = a.link.have_tlm && !a.link.linkLost(now_ms) &&
                      a.link.tlm.state != LinkState::kBench;
    // The full consequence goes in a tooltip, not on the row. It is three
    // sentences and the header is the one part of this window that must never
    // reflow -- but it is also exactly what an operator wants before clicking,
    // so it is one hover away rather than in the docs.
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(
            !can_take
                ? "Started without --host, so this console has no robot\n"
                  "address to command. The beacon's sender is whichever\n"
                  "console is FORWARDING it, not the robot, so it cannot\n"
                  "be guessed from the traffic either. Restart with --host."
            : ro ? (live
                ? "Become the commander of this robot.\n\n"
                  "The robot beacons to whoever commanded LAST, so the\n"
                  "console driving it stops hearing anything at all.\n\n"
                  "It is RUNNING: its arm level is adopted, so the handover\n"
                  "puts no edge on the wire and moves nothing. It keeps\n"
                  "standing until you press a key."
                : "Become the commander of this robot.\n\n"
                  "The robot beacons to whoever commanded LAST, so the\n"
                  "console driving it stops hearing anything at all.\n\n"
                  "Nothing is running, so this console starts disarmed.")
            : "Stop commanding and just watch.\n\n"
              "The transmit socket is closed -- not merely ignored.\n"
              "No parting disarm is sent: a console still driving this\n"
              "robot keeps it, and with nobody driving, the robot's own\n"
              "watchdog stands it after 250 ms.");
    }
    ImGui::SameLine(0.0f, 12.0f);
    if (!can_take) {
        ImGui::TextDisabled("no --host: nothing to command");
    } else if (ro) {
        ImGui::TextDisabled("commands %s; the console driving it goes dark%s",
                            a.o.host,
                            live ? " (adopts its arm level, so it keeps "
                                   "standing)"
                                 : " (starts disarmed)");
    } else {
        ImGui::TextDisabled("stop commanding; the robot is left exactly as "
                            "it is");
    }

    // Every control from here to the end of the row commands the robot. An
    // observer gets them greyed rather than hidden: the operator should be
    // able to see the console is the one they know, and that it is the MODE
    // and not a missing feature that stops them. Clicking through the disable
    // is impossible, and Intent::refuseReadonly is the backstop if it were.
    if (a.link.readonly) ImGui::BeginDisabled();
    labelFor(a, bimo::kActArm, lbl, sizeof lbl,
             a.in.armed ? "DISARM" : "ARM");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.toggleArm(now_ms);
    ImGui::SameLine();
    labelFor(a, bimo::kActStand, lbl, sizeof lbl, "STAND");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.stand("stand");
    ImGui::SameLine();
    // All three states, or ImGui falls back to its default blue the instant
    // the mouse crosses the button -- and a blue E-STOP is a lie.
    // Grey, not red, for a watcher. ImGui's disable dims a colour by alpha,
    // and a dimmed red is still unmistakably an E-STOP -- which is precisely
    // the button that must not look available when it is not. Draining the
    // colour is what makes "you cannot stop this robot from here" legible at
    // a glance, which is the only speed that matters for this control.
    const ImVec4 red = a.link.readonly  ? ImVec4(0.26f, 0.26f, 0.28f, 1.0f)
                     : a.in.estop    ? ImVec4(0.80f, 0.10f, 0.10f, 1.0f)
                                     : ImVec4(0.55f, 0.12f, 0.12f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, red);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(red.x + 0.15f, red.y, red.z, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          ImVec4(red.x + 0.25f, red.y, red.z, 1.0f));
    labelFor(a, bimo::kActEstop, lbl, sizeof lbl,
             a.link.readonly ? "E-STOP (not yours)"
                          : (a.in.estop ? "E-STOP LATCHED" : "E-STOP"));
    if (ImGui::Button(lbl, ImVec2(240, 34))) a.in.fireEstop();
    ImGui::PopStyleColor(3);
    // The robot's own state is NOT a control, and must not grey out with the
    // controls: it is the single thing a watcher is here to read.
    if (a.link.readonly) ImGui::EndDisabled();

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

    // RESET SERVOS. On its own row, below the row that STOPS things, because
    // it is the one button here that MOVES the robot from a standstill --
    // and it is the only one that still works once the E-stop or the fall
    // latch has (correctly) locked everything else out. Amber, not red: this
    // is not an emergency control, it is the way out of one.
    if (a.link.readonly) ImGui::BeginDisabled();
    const ImVec4 amber = a.link.readonly ? ImVec4(0.26f, 0.26f, 0.28f, 1.0f)
                                      : ImVec4(0.62f, 0.42f, 0.06f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, amber);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(amber.x + 0.15f, amber.y + 0.10f, amber.z,
                                 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          ImVec4(amber.x + 0.25f, amber.y + 0.18f, amber.z,
                                 1.0f));
    labelFor(a, bimo::kActHome, lbl, sizeof lbl, "RESET SERVOS");
    if (ImGui::Button(lbl, ImVec2(240, 34))) a.in.requestHome(now_ms);
    ImGui::PopStyleColor(3);
    if (a.link.readonly) ImGui::EndDisabled();
    ImGui::SameLine(0.0f, 12.0f);
    if (a.link.readonly) {
        ImGui::TextDisabled(
            "watching only: no transmit socket exists. The session driving "
            "the robot owns ARM, STAND, E-STOP and RESET.");
    } else if (a.in.home_frames > 0) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "asking (%d frames)...", a.in.home_frames);
    } else {
        ImGui::TextDisabled(
            "all joints -> calibrated zero (the stand), slowly, torque ON. "
            "Benches the loop; no arm needed.");
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
    // A watcher mirrors the robot's MEASURED pose only -- it relays no
    // command, because it has no socket to relay one on. Same checkbox, and
    // the half that survives is the half a watcher wants: the robot moving,
    // in 3D, instead of as six numbers.
    if (ImGui::Checkbox(a.link.readonly ? "ghost the robot" : "mirror the robot",
                        &mir)) {
        setMirror(a, mir);
    }
    ImGui::SameLine();
    if (a.mirror) {
        // The picture is only as live as the pose feed. When the feed stops,
        // the viewer holds the LAST pose -- which is correct (it is still the
        // last known state) and dangerous to leave unlabelled, because a held
        // pose and a live one look identical. So the age is on screen
        // whenever it is not moving. This is the whole complaint the mode was
        // rebuilt for: a picture that keeps looking live after it stops being
        // a measurement.
        // Three independent things can be missing, and they have different
        // fixes: joints in the beacon, a sim to draw them in, and a pose
        // recent enough to still be true. Saying which one it is beats one
        // "no ghost" that sends the operator looking in the wrong place.
        const bool fed = a.link.have_tlm &&
                         a.link.tlm.n_joints == linkproto::kNumJoints;
        const bool drawing = a.sim.running() && a.poses_sent > 0;
        const double pose_age = drawing ? now_ms - a.last_pose_ms : -1.0;
        if (fed && drawing && pose_age < 1000.0) {
            ImGui::TextColored(ImVec4(0.25f, 0.8f, 0.35f, 1.0f),
                               "live -- %u poses%s", a.poses_sent,
                               a.link.tlm.have_att ? " + attitude" : "");
        } else if (drawing) {
            // The picture is still up and no longer a measurement. Age on
            // screen, always: a held pose and a live one look identical.
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                               "FROZEN: showing the last pose, %.1f s old",
                               pose_age / 1000.0);
        } else if (fed) {
            // The half that WAS reporting "none yet" while joint angles were
            // arriving perfectly well -- there was simply nothing to draw
            // them in.
            ImGui::TextColored(ImVec4(0.25f, 0.8f, 0.35f, 1.0f),
                               "joint angles%s arriving -- press Start to "
                               "draw them",
                               a.link.tlm.have_att ? " + attitude" : "");
        } else if (a.link.readonly) {
            // The watcher cannot fix this itself: asking for joint angles is
            // a frame on the wire (kFlagPose), and it sends none. Name the
            // console that CAN, or this reads as a broken sim.
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                               "no joint angles in the beacon -- the DRIVING "
                               "console must be in mirror mode too");
        } else {
            // A benched loop measures nothing, so it publishes no pose. Say
            // which of the two it is rather than leaving "no ghost" ambiguous.
            const bool benched = a.link.have_tlm &&
                                 a.link.tlm.state == LinkState::kBench;
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "%s",
                               benched
                                   ? "asked -- but a BENCHED robot measures "
                                     "nothing; the pose appears on arm"
                                   : "asked for joint angles; none yet");
        }
    } else if (a.link.readonly) {
        ImGui::TextDisabled("draw the robot's measured joints in the sim");
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
    ImGui::SeparatorText(a.link.readonly ? "DRIVE (observer: disabled)" : "DRIVE");
    ImGui::SameLine();
    if (a.link.readonly) {
        // Not "0 B", which reads as an empty frame going out 20 times a
        // second. Nothing is on the wire because there is no wire.
        ImGui::TextDisabled("nothing on the wire");
    } else {
        ImGui::Text("on the wire: %zu B", a.link.last_len);
    }
    // The whole panel, sliders included: crouch/lift/foot_* are commands too,
    // and a slider that moves while the robot does not is the same lie the
    // E-STOP would be.
    if (a.link.readonly) ImGui::BeginDisabled();
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
    if (a.link.readonly) ImGui::EndDisabled();
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
    // The `cmd` traces are THIS console's own command, charted beside the
    // robot's estimate so the operator can see the robot answer them. An
    // observer never commanded anything, so its cmd traces are a flat zero
    // that sits under a moving `est` and reads as the one fault an operator
    // most wants to catch -- a robot moving while being told to stand. The
    // driver's actual command is simply not in the beacon (the robot echoes
    // seq, not velocity), so there is nothing honest to draw: say that, and
    // plot only what the robot itself reported.
    if (a.link.readonly) {
        plot("vx est", a.c_vx_est, -1.1f, 1.1f, "%+.2f", t.vx_est);
        plot("wz est", a.c_wz_est, -1.1f, 1.1f, "%+.2f", t.wz_est);
        ImGui::TextDisabled(
            "no cmd traces: the beacon carries what the robot MEASURED, not "
            "what the driving console asked for.");
    } else {
        plot("vx cmd", a.c_vx_cmd, -1.1f, 1.1f, "%+.2f", a.vx);
        plot("vx est", a.c_vx_est, -1.1f, 1.1f, "%+.2f", t.vx_est);
        plot("wz cmd", a.c_wz_cmd, -1.1f, 1.1f, "%+.2f", a.wz);
        plot("wz est", a.c_wz_est, -1.1f, 1.1f, "%+.2f", t.wz_est);
    }
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
        a.show_settings = false;
        a.rebind = -1;
    }
    ImGui::SameLine();
    if (ImGui::Button(a.show_settings ? "hide settings" : "settings")) {
        a.show_settings = !a.show_settings;
        a.show_keymap = false;
        a.rebind = -1;
    }
}

// -- SETTINGS ---------------------------------------------------------------
// Everything the command line can set, set from the window instead. The flags
// stay: a console you can start already pointed at the right robot is what
// scripts and `--screenshot` figures need. But an operator should not have to
// restart -- and lose the link, the recording and the sim child -- to try the
// same console against the twin, raise the walk speed, or start forwarding to
// a watcher who has just asked to see it.

void applyLink(App& a, double now_ms) {
    const bool observe = a.link.readonly;
    // Disarm the robot we are LEAVING, if this console is what is holding it
    // up. Unlike releasing control (where somebody else keeps flying it),
    // re-pointing means nobody is left commanding this one, so the explicit
    // 1 -> 0 edge is better than waiting out the watchdog -- and repeated,
    // since any single datagram may vanish.
    if (!observe && a.in.armed) {
        a.in.disarm();
        for (int i = 0; i < 5; ++i) {
            a.link.send(0.0f, 0.0f, 0);
            usleep(10 * 1000);
        }
    }
    a.in.releaseAll();
    a.in.armed = false;
    a.in.estop = false;
    a.in.home_frames = 0;
    if (!a.link.relink(a.ed_host, a.ed_cmd_port, a.ed_tlm_port, observe)) {
        snprintf(a.status, sizeof a.status,
                 "relink to %s:%d failed: %s (the old link is closed; fix the "
                 "address and apply again)", a.ed_host, a.ed_cmd_port,
                 strerror(errno));
        logEvent(a, "ERROR", "%s", a.status);
        return;
    }
    a.o.host = a.ed_host;              // the buffer outlives every use of it
    a.o.cmd_port = a.ed_cmd_port;
    a.o.tlm_port = a.ed_tlm_port;
    a.o.host_given = true;
    snprintf(a.host_port, sizeof a.host_port, "%s:%d", a.o.host, a.o.cmd_port);
    // The relay was pointed at the old link's socket; re-apply it, and let
    // mirror re-derive its ports too.
    if (a.ed_watch_on) {
        char wh[64] = "";
        int wp = a.o.tlm_port;
        if (bimo::splitHostPort(a.ed_watch, wh, sizeof wh, wp)) {
            a.link.setWatch(wh, wp);
        }
    }
    setMirror(a, a.mirror);
    snprintf(a.status, sizeof a.status,
             "%s %s:%d (telemetry :%d) -- disarmed by the move",
             observe ? "watching" : "driving", a.o.host, a.o.cmd_port,
             a.o.tlm_port);
    logEvent(a, "relink", "%s", a.status);
    (void)now_ms;
}

void applyWatch(App& a) {
    if (!a.ed_watch_on) {
        a.link.clearWatch();
        snprintf(a.status, sizeof a.status, "beacon relay off");
        return;
    }
    char wh[64] = "";
    int wp = a.o.tlm_port;
    if (!bimo::splitHostPort(a.ed_watch, wh, sizeof wh, wp) ||
        !a.link.setWatch(wh, wp)) {
        a.ed_watch_on = false;
        snprintf(a.status, sizeof a.status,
                 "--watch %s: expected HOST[:PORT] with a dotted-quad host",
                 a.ed_watch);
        return;
    }
    snprintf(a.status, sizeof a.status, "relaying every beacon to %s:%d%s", wh,
             wp, a.link.readonly ? " -- once this console takes control" : "");
    logEvent(a, "watch", "%s", a.status);
}

void applyStream(App& a) {
    a.o.stream_port = a.ed_stream_port;
    a.o.sim_view = a.ed_sim_view[0] ? a.ed_sim_view : nullptr;
    simViewUrl(a, a.sim_url, sizeof a.sim_url, a.sim.running());
    a.stream.stop();
    a.stream.start(a.sim_url);
    snprintf(a.status, sizeof a.status, "stream -> %s", a.sim_url);
}

void applyPaths(App& a) {
    snprintf(a.repo, sizeof a.repo, "%s", a.ed_repo);
    snprintf(a.sessions, sizeof a.sessions, "%s", a.ed_sessions);
    a.runs.scan(a.repo);
    if (a.run_sel >= a.runs.n) a.run_sel = 0;
    snprintf(a.status, sizeof a.status,
             "repo %s -- %d policy run(s) with SIL weights; sessions -> %s",
             a.repo, a.runs.n, a.sessions);
}

void drawSettings(App& a, double now_ms) {
    ImGui::SeparatorText("SETTINGS");
    // Scrolls: there are more flags than rows on a laptop screen, and a panel
    // that silently ends at the bottom of the window hides exactly the
    // settings nobody remembers exist. -34 keeps the session footer -- the
    // record button -- on screen underneath.
    ImGui::BeginChild("settings_scroll",
                      ImVec2(0.0f, ImGui::GetContentRegionAvail().y - 34.0f));
    ImGui::TextDisabled("every command-line flag, live. Fields that rebind a "
                        "socket or restart a stream apply on their button; "
                        "speeds and rate take effect as you drag.");

    ImGui::SeparatorText("link  (--host / --cmd-port / --tlm-port)");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::InputText("host", a.ed_host, sizeof a.ed_host);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("cmd port", &a.ed_cmd_port);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("tlm port", &a.ed_tlm_port);
    const bool moved = strcmp(a.ed_host, a.o.host) != 0 ||
                       a.ed_cmd_port != a.o.cmd_port ||
                       a.ed_tlm_port != a.o.tlm_port;
    if (!moved) ImGui::BeginDisabled();
    if (ImGui::Button("apply & reconnect", ImVec2(180, 0))) {
        applyLink(a, now_ms);
    }
    if (!moved) ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled(moved ? "disarms first, then re-points this console "
                                "(it stays %s)"
                              : "currently %s -- edit a field to re-point",
                        a.link.readonly ? "a watcher" : "a driver");

    ImGui::SeparatorText("speeds  (--vx / --vy / --wz / --rate)");
    ImGui::SetNextItemWidth(300.0f);
    ImGui::SliderFloat("vx  m/s", &a.o.speeds.vx, 0.0f, kVMax, "%.2f");
    ImGui::SetNextItemWidth(300.0f);
    ImGui::SliderFloat("vy  m/s (strafe)", &a.o.speeds.vy, 0.0f, kVyMax,
                       "%.2f");
    ImGui::SetNextItemWidth(300.0f);
    ImGui::SliderFloat("wz  rad/s", &a.o.speeds.wz, 0.0f, kWMax, "%.2f");
    ImGui::SetNextItemWidth(300.0f);
    // Bounded well inside anything the watchdog calls stale (kStaleMs = 250
    // ms): a rate slider that could starve the robot's own dead-man would be
    // a way to make it stand by fiddling with a setting.
    ImGui::SliderScalar("send Hz", ImGuiDataType_Double, &a.o.rate_hz,
                        &kRateMin, &kRateMax, "%.0f");
    ImGui::TextDisabled("the trained envelope caps these: v <= %.1f m/s, "
                        "vy <= %.1f, w <= %.1f rad/s",
                        static_cast<double>(kVMax), static_cast<double>(kVyMax),
                        static_cast<double>(kWMax));

    ImGui::SeparatorText("telemetry relay  (--watch)");
    if (ImGui::Checkbox("forward every beacon to", &a.ed_watch_on)) {
        applyWatch(a);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::InputText("HOST[:PORT]##watch", a.ed_watch, sizeof a.ed_watch,
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
        applyWatch(a);
    }
    ImGui::SameLine();
    if (ImGui::Button("apply##watch")) applyWatch(a);
    ImGui::TextDisabled("a `--readonly` console cannot hear the robot on its "
                        "own: this is how the driver feeds it.  relayed: %u",
                        a.link.watched);

    ImGui::SeparatorText("sim  (--sim-view / --stream-port / --sim-cmd-port)");
    ImGui::SetNextItemWidth(360.0f);
    ImGui::InputText("view URL (blank = host:port)", a.ed_sim_view,
                     sizeof a.ed_sim_view);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("stream port", &a.ed_stream_port);
    ImGui::SameLine();
    if (ImGui::Button("connect##stream")) applyStream(a);
    ImGui::SameLine();
    if (ImGui::Button("disconnect##stream")) {
        a.stream.stop();
        snprintf(a.status, sizeof a.status, "stream disconnected");
    }
    ImGui::SetNextItemWidth(120.0f);
    if (a.sim.running()) ImGui::BeginDisabled();
    if (ImGui::InputInt("sim cmd port", &a.ed_sim_cmd_port)) {
        a.sim_cmd_port = a.ed_sim_cmd_port;
        a.sim_tlm_port = a.ed_sim_cmd_port + 1;
    }
    if (a.sim.running()) ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled(a.sim.running()
                            ? "stop the sim to move its ports"
                            : "the sim's own ports; telemetry is cmd + 1");

    ImGui::SeparatorText("paths  (--repo / --sessions / --keymap)");
    ImGui::SetNextItemWidth(420.0f);
    ImGui::InputText("repo", a.ed_repo, sizeof a.ed_repo);
    ImGui::SetNextItemWidth(420.0f);
    ImGui::InputText("sessions", a.ed_sessions, sizeof a.ed_sessions);
    if (ImGui::Button("apply & rescan runs", ImVec2(180, 0))) applyPaths(a);
    ImGui::SameLine();
    ImGui::TextDisabled("%d run(s) found", a.runs.n);
    ImGui::SetNextItemWidth(420.0f);
    ImGui::InputText("keymap", a.ed_keymap, sizeof a.ed_keymap);
    if (ImGui::Button("load##keymap", ImVec2(180, 0))) {
        char err[192] = "";
        snprintf(a.keymap_path, sizeof a.keymap_path, "%s", a.ed_keymap);
        if (a.keys.load(a.keymap_path, err, sizeof err)) {
            snprintf(a.status, sizeof a.status, "keymap <- %s", a.keymap_path);
        } else {
            snprintf(a.status, sizeof a.status, "%s", err);
        }
    }

    ImGui::SeparatorText("screenshot  (--screenshot)");
    ImGui::SetNextItemWidth(420.0f);
    ImGui::InputText("file", a.ed_shot, sizeof a.ed_shot);
    if (ImGui::Button("grab now", ImVec2(180, 0))) a.shot_now = true;
    ImGui::SameLine();
    ImGui::TextDisabled("writes a PNG of this window on the next frame");

    ImGui::SeparatorText("this session");
    ImGui::TextDisabled(
        "--headless is the one flag with no button: it chooses whether there "
        "is a window at all, and this is the window. --readonly has one -- "
        "it is the TAKE CONTROL / WATCH ONLY button at the top.");
    ImGui::EndChild();
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
        // An observer holds no axis, ever. Intent::press would refuse anyway,
        // but it would refuse ONCE PER FRAME, and a status line reprinting the
        // same refusal at 60 Hz is how a leaning-on-the-keyboard operator
        // loses the beacon-lost message underneath it.
        a.key_held[act] = !a.link.readonly && key != ImGuiKey_None &&
                          ImGui::IsKeyDown(key);
    }
    for (int act = bimo::kMotionActions; act < bimo::kActCount; ++act) {
        const ImGuiKey key = fromKeyCode(a.keys.key[act]);
        // Quit and the recorder are the console's OWN controls -- neither
        // touches the robot -- so a watcher keeps them. Everything else is a
        // command, and stays inert with the buttons that fire it.
        if (a.link.readonly && act != bimo::kActQuit &&
            act != bimo::kActRecord && act != bimo::kActControl) {
            continue;
        }
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
        } else if (a.show_settings) {
            drawSettings(a, now_ms);
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
            // Read per tick, not captured at startup: the send rate is a
            // slider now, and a cadence fixed at boot would ignore it.
            next_link_ms += 1000.0 / a.o.rate_hz;
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
        // The settings panel's "grab now". Same frame, same grab -- but it
        // does NOT quit: --screenshot exists to produce a figure and leave,
        // this one is an operator taking a picture of a session they are
        // still in the middle of.
        if (a.shot_now) {
            a.shot_now = false;
            if (grabWindow(a.ed_shot, fb_w, fb_h)) {
                snprintf(a.status, sizeof a.status, "wrote %s (%dx%d)",
                         a.ed_shot, fb_w, fb_h);
            } else {
                snprintf(a.status, sizeof a.status, "could not write %s",
                         a.ed_shot);
            }
            logEvent(a, "shot", "%s", a.status);
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
    // --readonly is the STARTING mode, not a permanent one: the console can
    // be handed the robot with the TAKE CONTROL button, which is why the
    // address is resolved either way. From here on the live truth is
    // link.readonly (equivalently, tx < 0), never o.readonly.
    if (o.readonly) {
        if (!a.link.listen(o.tlm_port, o.host_given ? o.host : nullptr,
                           o.cmd_port)) {
            fprintf(stderr, "bimo_gui: listen: %s (tlm port %d, host %s)\n",
                    strerror(errno), o.tlm_port,
                    o.host_given ? o.host : "(none)");
            return 1;
        }
        a.in.readonly = true;
    } else if (!a.link.open(o.host, o.cmd_port, o.tlm_port)) {
        fprintf(stderr, "bimo_gui: link: %s (host %s, tlm port %d)\n",
                strerror(errno), o.host, o.tlm_port);
        return 1;                      // before the log exists: stderr only
    }
    // --watch is set up in BOTH modes now that the mode can change. The relay
    // needs a transmit socket, so it simply does nothing while watching
    // (Link::poll checks tx) and starts working the moment this console takes
    // control -- which is what a watcher-turned-driver wants, since the
    // console it took the robot from is now the one in the dark.
    if (o.watch != nullptr) {
        char wh[64] = "";
        int wp = o.tlm_port;           // the port a stock watcher listens on
        if (!bimo::splitHostPort(o.watch, wh, sizeof wh, wp) ||
            !a.link.setWatch(wh, wp)) {
            fprintf(stderr, "bimo_gui: --watch %s: expected HOST[:PORT] with "
                            "a dotted-quad host\n", o.watch);
            return 2;
        }
        snprintf(a.status, sizeof a.status, "relaying every beacon to %s:%d%s",
                 wh, wp, o.readonly ? " once this console takes control" : "");
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
    if (o.readonly) {
        logEvent(a, "readonly",
                 "OBSERVER: no transmit socket. Waiting for beacons on :%d "
                 "-- the driving console must forward them (--watch). "
                 "TAKE CONTROL [t] %s.", o.tlm_port,
                 a.link.has_dest ? "will command it" : "unavailable: no --host");
    }
    if (a.link.watch_on) logEvent(a, "watch", "%s", a.status);
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

    // The settings panel edits copies; seed them from what the command line
    // actually produced, so it opens describing THIS session rather than the
    // defaults, and "apply" with nothing touched is a no-op.
    snprintf(a.ed_host, sizeof a.ed_host, "%s", a.o.host);
    a.ed_cmd_port = a.o.cmd_port;
    a.ed_tlm_port = a.o.tlm_port;
    a.ed_stream_port = a.o.stream_port;
    a.ed_sim_cmd_port = a.sim_cmd_port;
    a.ed_watch_on = a.link.watch_on;
    if (o.watch != nullptr) {
        snprintf(a.ed_watch, sizeof a.ed_watch, "%s", o.watch);
    }
    if (o.sim_view != nullptr) {
        snprintf(a.ed_sim_view, sizeof a.ed_sim_view, "%s", o.sim_view);
    }
    snprintf(a.ed_repo, sizeof a.ed_repo, "%s", a.repo);
    snprintf(a.ed_sessions, sizeof a.ed_sessions, "%s", a.sessions);
    snprintf(a.ed_keymap, sizeof a.ed_keymap, "%s", a.keymap_path);
    if (o.screenshot != nullptr) {
        snprintf(a.ed_shot, sizeof a.ed_shot, "%s", o.screenshot);
    }

    const int rc = a.o.headless ? runHeadless(a) : runWindow(a);

    // Leave a limp robot, not a standing one nobody is watching: drop ARM (the
    // 1 -> 0 edge benches it) and repeat, since any single datagram may vanish.
    // An observer never armed anything and must not disarm what the DRIVER
    // armed -- the whole mode is that closing this window changes nothing on
    // the robot. Link::emit would drop these anyway; not composing them is
    // what says why.
    if (!a.link.readonly) {
        for (int i = 0; i < 10; ++i) {
            a.link.send(0.0f, 0.0f, 0);
            usleep(20 * 1000);
        }
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
