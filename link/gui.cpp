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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
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
    bool headless = false;
    bool autostream = false;             // --sim-view given: connect at boot
};

void usage(const char* argv0) {
    fprintf(stderr,
            "usage: %s --host IP [--cmd-port %d] [--tlm-port %d]\n"
            "          [--vx %.2f] [--vy %.2f] [--wz %.2f] [--rate %.0f]\n"
            "          [--keymap FILE] [--sim-view [URL]] [--stream-port %d]\n"
            "          [--run-name NAME] [--repo DIR] [--sessions DIR]\n"
            "          [--headless]\n",
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

    char repo[512] = ".";
    char sessions[512] = "hw_sessions";
    char keymap_path[512] = "";
    char sim_url[512] = "";
    char status[320] = "";
    int run_sel = 0;
    bool sim_hang = false;

    // what went on the wire this tick
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    Ext ext_sent;
    uint8_t flags = 0;

    Ring c_vbat, c_upz, c_vx_cmd, c_vx_est, c_wz_cmd, c_wz_est, c_late;
    uint32_t charted = 0;
    bool quit = false;

    // window-only
    GLuint tex = 0;
    int tex_w = 0, tex_h = 0;
    uint64_t frame_seq = 0;
    int rebind = -1;                     // action awaiting a key
    bool show_keymap = false;
};

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
    }
    a.rec.maybeFlush(now_ms);
    a.sim.poll();
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

void simViewUrl(const App& a, char* out, size_t cap) {
    if (a.o.sim_view != nullptr) {
        snprintf(out, cap, "%s", a.o.sim_view);
        return;
    }
    snprintf(out, cap, "http://%s:%d/", a.o.host, a.o.stream_port);
}

void startSim(App& a) {
    if (a.runs.n == 0) {
        snprintf(a.status, sizeof a.status,
                 "no runnable sim runs under %s/sim/runs (each needs a "
                 "config.json AND sim/sil/weights/<run>.silw.json)", a.repo);
        return;
    }
    if (!a.sim.start(a.repo, a.runs.name[a.run_sel], a.sim_hang,
                     a.o.stream_port, a.o.cmd_port, a.o.tlm_port)) {
        snprintf(a.status, sizeof a.status, "%s", a.sim.err);
        return;
    }
    simViewUrl(a, a.sim_url, sizeof a.sim_url);
    a.stream.start(a.sim_url);
    snprintf(a.status, sizeof a.status, "sim %s starting; view %s",
             a.runs.name[a.run_sel], a.sim_url);
}

void stopSim(App& a) {
    a.stream.stop();
    a.sim.stop();
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

int runHeadless(App& a) {
    const double period_ms = 1000.0 / a.o.rate_hz;
    const double t0_ms = nowMs();
    double next_ms = t0_ms, next_status = t0_ms;
    char line[256];
    size_t len = 0;
    char last_note[sizeof a.in.note] = "";
    printf("bimo_gui --headless -> %s:%d (telemetry :%d)\n", a.o.host,
           a.o.cmd_port, a.o.tlm_port);
    fflush(stdout);
    while (!a.quit) {
        const double now_ms = nowMs();

        for (;;) {
            fd_set r;
            FD_ZERO(&r);
            FD_SET(STDIN_FILENO, &r);
            timeval tv = {0, 0};
            if (select(STDIN_FILENO + 1, &r, nullptr, nullptr, &tv) <= 0) break;
            char c;
            const ssize_t n = read(STDIN_FILENO, &c, 1);
            if (n <= 0) {          // stdin closed: leave, disarming on the way
                a.quit = true;
                break;
            }
            if (c == '\n') {
                line[len] = 0;
                headlessLine(a, line, now_ms);
                len = 0;
            } else if (len + 1 < sizeof line) {
                line[len++] = c;
            }
        }

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

// A button that is "held" for as long as the mouse is down on it -- the same
// dead-man as the key, so the on-screen controls are not a second-class way
// to drive.
bool heldButton(const char* label, const ImVec2& size) {
    ImGui::Button(label, size);
    return ImGui::IsItemActive();
}

void labelFor(const App& a, int action, char* out, size_t cap,
              const char* text) {
    const char* kn = bimo::keyName(a.keys.key[action]);
    if (kn[0]) snprintf(out, cap, "%s [%s]", text, kn);
    else snprintf(out, cap, "%s", text);
}

void drawHeader(App& a, double now_ms) {
    ImGui::Text("-> %s:%d   tx %.0f Hz", a.o.host, a.o.cmd_port, a.o.rate_hz);
    ImGui::SameLine(0.0f, 24.0f);
    if (!a.link.have_tlm) {
        ImGui::TextDisabled("<- :%d  no telemetry yet", a.o.tlm_port);
    } else if (a.link.linkLost(now_ms)) {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.2f, 1.0f),
                           "<- :%d  LINK LOST (%.1f s)", a.o.tlm_port,
                           a.link.tlmAgeMs(now_ms) / 1000.0);
    } else {
        ImGui::Text("<- :%d  beacon %.0f ms - %.1f Hz   seq %u  lag %ld  bad %u",
                    a.o.tlm_port, a.link.tlmAgeMs(now_ms), a.link.rate_hz,
                    a.link.seq,
                    a.link.tlm.state == LinkState::kBench ? 0 : a.link.lag(),
                    a.link.tlm_bad);
    }

    char lbl[64];
    labelFor(a, bimo::kActArm, lbl, sizeof lbl,
             a.in.armed ? "DISARM" : "ARM");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.toggleArm(now_ms);
    ImGui::SameLine();
    labelFor(a, bimo::kActStand, lbl, sizeof lbl, "STAND");
    if (ImGui::Button(lbl, ImVec2(140, 34))) a.in.stand("stand");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button,
                          a.in.estop ? ImVec4(0.75f, 0.10f, 0.10f, 1.0f)
                                     : ImVec4(0.55f, 0.12f, 0.12f, 1.0f));
    labelFor(a, bimo::kActEstop, lbl, sizeof lbl,
             a.in.estop ? "E-STOP LATCHED" : "E-STOP");
    if (ImGui::Button(lbl, ImVec2(220, 34))) a.in.fireEstop();
    ImGui::PopStyleColor();

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
    ImGui::SetNextItemWidth(panel_w - 90.0f);
    if (a.runs.n > 0) {
        if (ImGui::BeginCombo("run", a.runs.name[a.run_sel])) {
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
    ImGui::Checkbox("hang (test stand)", &a.sim_hang);
    ImGui::SameLine();
    if (a.sim.running()) {
        if (ImGui::Button("Stop")) stopSim(a);
    } else {
        if (ImGui::Button("Start")) startSim(a);
    }

    // The frame, or an honest reason there is no frame.
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float w = panel_w - 16.0f;
    const float h = a.tex_h > 0 ? w * static_cast<float>(a.tex_h) /
                                      static_cast<float>(a.tex_w)
                                : w * 0.75f;
    if (a.tex != 0) {
        ImGui::Image(static_cast<ImTextureID>(a.tex), ImVec2(w, h));
        const double age = now_ms - a.stream.lastFrameMs();
        if (age > kStreamStaleMs) {
            // A frozen frame that looks live is the failure tools/cam_live.py
            // had to fix; say so over the picture rather than let it lie.
            ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.25f, 1.0f),
                               "STALE  no frame for %.1f s", age / 1000.0);
        } else {
            ImGui::Text("%s   %u frames", a.sim_url, a.stream.frames());
        }
    } else {
        ImGui::Dummy(ImVec2(w, h * 0.35f));
        char serr[192];
        a.stream.error(serr, sizeof serr);
        if (serr[0]) {
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f), "%s", serr);
        } else if (a.stream.running()) {
            ImGui::TextDisabled("connecting to %s ...", a.sim_url);
        } else {
            ImGui::TextDisabled("no stream -- press Start, or pass --sim-view");
        }
        ImGui::Dummy(ImVec2(w, h * 0.35f));
    }
    (void)avail;

    if (a.sim.lines() > 0 && ImGui::CollapsingHeader("sil_twin log")) {
        ImGui::BeginChild("simlog", ImVec2(0, 120), ImGuiChildFlags_Borders);
        for (int i = 0; i < a.sim.lines(); ++i) {
            ImGui::TextUnformatted(a.sim.line(i));
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }
}

void drawDrive(App& a, double now_ms) {
    ImGui::SeparatorText("DRIVE");
    ImGui::SameLine();
    ImGui::Text("on the wire: %zu B", a.link.last_len);
    if (a.link.last_len == kCmdLenExt) {
        ImGui::SameLine();
        // Old firmware drops a long frame BY LENGTH, so the symptom of an
        // out-of-date robot is that the sliders make it go quiet.
        ImGui::TextDisabled("(extended frame: needs firmware >= 2026-08-31)");
    }

    const ImVec2 sz(78, 40);
    const float x0 = ImGui::GetCursorPosX() + 40.0f;
    char lbl[64];

    ImGui::SetCursorPosX(x0 + sz.x + 8.0f);
    labelFor(a, bimo::kActForward, lbl, sizeof lbl, "fwd");
    if (heldButton(lbl, sz)) a.in.press(bimo::kAxForward, now_ms);

    labelFor(a, bimo::kActStrafeLeft, lbl, sizeof lbl, "<-");
    ImGui::SetCursorPosX(x0);
    if (heldButton(lbl, sz)) a.in.press(bimo::kAxLeft, now_ms);
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActBack, lbl, sizeof lbl, "back");
    if (heldButton(lbl, sz)) a.in.press(bimo::kAxBack, now_ms);
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActStrafeRight, lbl, sizeof lbl, "->");
    if (heldButton(lbl, sz)) a.in.press(bimo::kAxRight, now_ms);
    ImGui::SameLine(0.0f, 24.0f);
    ImGui::BeginGroup();
    ImGui::Text("vx  %+.2f m/s", static_cast<double>(a.vx));
    ImGui::Text("vy  %+.2f m/s", static_cast<double>(a.vy));
    ImGui::Text("wz  %+.2f rad/s", static_cast<double>(a.wz));
    ImGui::EndGroup();

    ImGui::SetCursorPosX(x0);
    labelFor(a, bimo::kActTurnLeft, lbl, sizeof lbl, "turn L");
    if (heldButton(lbl, ImVec2(118, 34))) a.in.press(bimo::kAxTurnLeft, now_ms);
    ImGui::SameLine(0.0f, 8.0f);
    labelFor(a, bimo::kActTurnRight, lbl, sizeof lbl, "turn R");
    if (heldButton(lbl, ImVec2(118, 34))) a.in.press(bimo::kAxTurnRight, now_ms);

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
    ImGui::PlotLines(label, r.v, kChart, r.off, overlay, lo, hi,
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
        ImGui::TextDisabled("not recording (-> %s)", a.sessions);
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
        a.in.releaseAll();
        return;
    }

    // The window lost focus (or a text field has the keyboard) with a key
    // held: a window can do that, and a terminal cannot. Treat it as every
    // key up, this instant.
    if (!focused || io.WantCaptureKeyboard) {
        a.in.releaseAll();
        return;
    }

    for (int act = 0; act < bimo::kMotionActions; ++act) {
        const ImGuiKey key = fromKeyCode(a.keys.key[act]);
        if (key == ImGuiKey_None) continue;
        if (ImGui::IsKeyDown(key)) a.in.press(act, now_ms);
        else a.in.release(act);
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

int runWindow(App& a) {
    glfwSetErrorCallback(nullptr);
    if (!glfwInit()) {
        fprintf(stderr, "cannot open a window (glfwInit failed). "
                        "--headless drives the same link without one.\n");
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
        fprintf(stderr, "cannot create a window\n");
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
    double next_link_ms = nowMs();

    while (!a.quit && !glfwWindowShouldClose(win)) {
        glfwPollEvents();
        const double now_ms = nowMs();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        pumpKeys(a, win, now_ms);

        // The link runs at its own 20 Hz regardless of the frame rate: the
        // robot's command stream must not depend on how fast this draws.
        if (now_ms >= next_link_ms) {
            tick(a, now_ms);
            next_link_ms += period_ms;
            if (next_link_ms < now_ms) next_link_ms = now_ms;
        }
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
        drawDrive(a, now_ms);
        ImGui::EndChild();

        if (a.show_keymap) {
            drawKeymap(a);
        } else {
            drawTelemetry(a, now_ms);
        }
        drawSession(a, now_ms);

        ImGui::End();

        ImGui::Render();
        int fb_w, fb_h;
        glfwGetFramebufferSize(win, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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
    Options o;
    if (!parse(argc, argv, o)) {
        usage(argv[0]);
        return 2;
    }
    App a;
    a.o = o;
    if (!a.link.open(o.host, o.cmd_port, o.tlm_port)) {
        fprintf(stderr, "link: %s (host %s, tlm port %d)\n", strerror(errno),
                o.host, o.tlm_port);
        return 1;
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
    char err[192] = "";
    if (a.keymap_path[0] && !a.keys.load(a.keymap_path, err, sizeof err)) {
        fprintf(stderr, "keymap: %s\n", err);
        return 2;
    }
    a.runs.scan(a.repo);
    if (o.run_name != nullptr) {
        const int i = a.runs.indexOf(o.run_name);
        if (i >= 0) a.run_sel = i;
    }
    if (o.autostream) {
        simViewUrl(a, a.sim_url, sizeof a.sim_url);
        a.stream.start(a.sim_url);
    }

    const int rc = a.o.headless ? runHeadless(a) : runWindow(a);

    // Leave a limp robot, not a standing one nobody is watching: drop ARM (the
    // 1 -> 0 edge benches it) and repeat, since any single datagram may vanish.
    for (int i = 0; i < 10; ++i) {
        a.link.send(0.0f, 0.0f, 0);
        usleep(20 * 1000);
    }
    if (a.rec.active()) a.rec.stop();
    stopSim(a);
    a.link.close();
    printf("bimo_gui: quit; sent disarm (%u frames total)\n", a.link.sent);
    return rc;
}
