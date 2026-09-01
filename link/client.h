// The half of an operator's console that is not a screen.
//
// Sockets, the operator's intent, the arm-sync rule, and the decision about
// what actually goes on the wire -- shared by both clients:
//
//   link/tui.cpp   ncurses, over SSH, one keystroke at a time
//   link/gui.cpp   Dear ImGui, a window, true key-release and sliders
//
// Splitting it this way is the same argument the protocol itself makes: two
// copies of a rule drift the moment one side is reworked, and "what does ARM
// mean, and when does the console give up on it" is exactly the kind of rule
// that must not have two answers. Every byte still comes from
// firmware/components/linkproto, so neither console can disagree with the
// robot either.
//
// No STL and no exceptions: this compiles under the firmware's own flags
// (-fno-exceptions -fno-rtti -Werror -Wconversion), so the console and the
// robot are built from the same rules as well as the same sources.
#ifndef BIMO_LINK_CLIENT_H_
#define BIMO_LINK_CLIENT_H_

#include <netinet/in.h>
#include <stddef.h>
#include <stdint.h>

#include "linkproto/protocol.h"

namespace bimo {

using linkproto::Command;
using linkproto::LinkState;
using linkproto::Telemetry;

// BENCH this long after arming -> the robot refused, or rebooted under us.
constexpr double kArmSyncMs = 1500.0;
// No beacon this long -> "link lost".
constexpr double kTlmLostMs = 1000.0;

constexpr double kDefaultVx = 0.4;    // eval_precision line_1m's speed
constexpr double kDefaultVy = 0.2;    // inside kVyMax (0.3), the sway draw
constexpr double kDefaultWz = 0.5;    // eval_commands validates +-0.5

double nowMs();

const char* stateName(LinkState s);

// -- what a key can hold ----------------------------------------------------
// Six axes rather than one "motion", because a window has real key-release
// and can therefore hold two at once (forward + turn = an arc). The ncurses
// console still sets exactly one at a time; same struct, poorer input device.
enum Axis {
    kAxForward = 0,
    kAxBack,
    kAxLeft,        // strafe: +vy, body-frame left
    kAxRight,       // strafe: -vy
    kAxTurnLeft,    // +wz, CCW
    kAxTurnRight,   // -wz
    kAxCount,
};

const char* axisName(int axis);

// Per-axis magnitudes, from the command line.
struct Speeds {
    float vx = static_cast<float>(kDefaultVx);
    float vy = static_cast<float>(kDefaultVy);
    float wz = static_cast<float>(kDefaultWz);
};

// The five extended channels, at the trained defaults a 14 B frame decodes to.
struct Ext {
    float crouch = 1.0f;
    float lift = 0.0f;
    float foot_dx = 0.0f;
    float foot_dz = 0.0f;

    bool atDefaults() const;   // compared on the MILLI-ROUNDED ints, as the
                               // wire does -- see Link::sendFull()
    void reset();
    // Snap into the trained draw (linkproto::clampExtToEnvelope), so a slider
    // cannot ask for a posture training never produced.
    void clamp();
};

// -- the link ---------------------------------------------------------------
struct Link {
    int tx = -1, rx = -1;
    sockaddr_in to = {};
    uint32_t seq = 0;
    uint32_t sent = 0;
    size_t last_len = 0;          // 14 or 24: what the UI reports

    Telemetry tlm = {};
    bool have_tlm = false;
    double tlm_at_ms = -1.0;
    uint32_t tlm_count = 0, tlm_bad = 0;
    // Beacon rate: frames in the last second, updated once a second.
    uint32_t rate_window = 0;
    double rate_at_ms = 0.0;
    double rate_hz = 0.0;

    bool open(const char* host, int cmd_port, int tlm_port);
    void close();

    // vx/wz only -- the classic 14 B frame.
    void send(float vx, float wz, uint8_t flags);
    // The full intent. Emits the SHORT frame whenever the extras are at their
    // defaults, exactly as link/protocol.py:encode_command does, so a console
    // that never touches a slider is byte-identical to one that cannot.
    void sendFull(float vx, float vy, float wz, const Ext& ext, uint8_t flags);

    // Drain to the freshest beacon; never queue up.
    void poll(double now_ms);
    double tlmAgeMs(double now_ms) const;
    bool linkLost(double now_ms) const;
    long lag() const;
};

// -- the operator's intent --------------------------------------------------
struct Intent {
    bool armed = false;
    bool estop = false;
    bool held[kAxCount] = {false, false, false, false, false, false};
    Ext ext;
    double armed_at_ms = 0.0;
    char note[192] = "";

    void say(const char* s);
    void sayf2(const char* fmt, const char* a);

    bool moving() const;
    void releaseAll();            // focus loss, stand, e-stop: every key up

    // Returns false (and says why) when the press is refused.
    bool press(int axis, double now_ms);
    void release(int axis);

    void arm(double now_ms);
    void disarm();
    void toggleArm(double now_ms);

    void stand(const char* why);
    void fireEstop();

    // What goes on the wire this tick.
    void frame(const Speeds& s, float& vx, float& vy, float& wz, Ext& ext_out,
               uint8_t& flags) const;

    // The robot is authoritative: armed here but BENCH there, after the edge
    // has had time to land, means the arm was refused or the robot rebooted.
    // Returns true if it just gave up (and fills `note` with the robot's own
    // reason).
    bool syncToRobot(const Link& link, double now_ms);
};

// A short human name for the current motion, for the status line.
const char* motionText(const Intent& in);

// Send `intent` once, handing back exactly what went on the wire so the
// console can draw it. One place decides the frame, so the two consoles
// cannot put different bytes on the wire for the same keys.
void sendIntent(Link& link, const Intent& in, const Speeds& s, float& vx,
                float& vy, float& wz, Ext& ext_out, uint8_t& flags);

}  // namespace bimo

#endif  // BIMO_LINK_CLIENT_H_
