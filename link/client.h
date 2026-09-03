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

// How many consecutive frames carry a "reset the servos" request.
//
// The robot acts on the RISING EDGE of kFlagHome (linkproto::HomeLatch), and
// this is UDP: a single flagged frame is a request that a dropped packet
// silently cancels. Holding the level for 8 frames -- 400 ms at kSendHz --
// makes the request survive a burst loss, and costs nothing, because an edge
// is idempotent: the robot homes on the first flagged frame it receives and
// ignores the seven behind it. The level must also DROP again, or the next
// press would make no edge at all.
constexpr int kHomeFrames = 8;

constexpr double kDefaultVx = 0.4;    // eval_precision line_1m's speed
constexpr double kDefaultVy = 0.2;    // inside kVyMax (0.3), the sway draw
constexpr double kDefaultWz = 0.5;    // eval_commands validates +-0.5

double nowMs();

const char* stateName(LinkState s);

// "192.168.2.30" or "192.168.2.30:9101" -> address and port, for --watch.
// `port` is left UNTOUCHED when the spec carries none, so the caller's
// default -- the telemetry port it is already bound to, which is what a
// watcher on a stock build will be listening on -- survives. Returns false on
// an empty host, a port that is not a number, or one outside 1..65535.
bool splitHostPort(const char* spec, char* host, size_t cap, int& port);

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

    // Mirror mode: the same bytes also go to a second address -- the sim,
    // running the same command beside the robot. One encode, two sendto's, so
    // the sim cannot be commanded differently from the robot by construction.
    sockaddr_in mirror_to = {};
    bool mirror_on = false;

    // OBSERVER mode: this link has no transmit socket at all, so it cannot
    // put a byte on the wire. Not a policy the UI enforces -- a policy the
    // file descriptor enforces.
    //
    // Toggleable (2026-09-03), and the invariant survives the toggle: the
    // socket is CLOSED on the way into readonly and CREATED on the way out,
    // so `readonly` and `tx < 0` are never briefly out of step. A flag that
    // merely guarded a live socket would be a weaker promise than this.
    bool readonly = false;
    // Whether `to` holds a real destination -- i.e. a --host was given. A
    // watcher started without one has nowhere to take control TO, and must
    // say so rather than offer a button that cannot work. The beacon's source
    // address is NOT a fallback: it is the forwarding driver, not the robot.
    bool has_dest = false;

    // Telemetry fan-out, the observer's other half. The robot beacons to
    // WHOEVER COMMANDED LAST and to nobody else (firmware/main/wifi_link.cpp,
    // the Commander snapshot), so a second console cannot simply listen in:
    // there is nothing addressed to it, and the only way to become the
    // destination is to send a command -- which both steals the beacon from
    // the real driver and commands the robot. So the DRIVER forwards instead.
    // Every beacon it accepts is re-sent byte for byte to the watcher, from
    // the socket it already commands with, which is why the observer can be
    // wholly silent. Verbatim on purpose: a relay that re-encoded would let
    // the two consoles disagree about what the robot said.
    sockaddr_in watch_to = {};
    bool watch_on = false;
    uint32_t watched = 0;

    bool open(const char* host, int cmd_port, int tlm_port);
    // The observer's open(): bind the beacon port, create NO tx socket, and
    // latch `readonly`. Every send path below is a no-op afterwards, and
    // would fail on tx == -1 even if one were not. `host` may be null -- it
    // is only remembered, as the destination a later takeControl() uses.
    bool listen(int tlm_port, const char* host, int cmd_port);

    // -- the toggle ---------------------------------------------------------
    // Stop commanding: close the transmit socket. Deliberately sends NO
    // parting disarm, unlike quitting. The robot is being watched precisely
    // because somebody else is flying it, and THEIR frames hold the arm level
    // high; a disarm here would be a 1 -> 0 edge that benches the robot
    // mid-stride out from under the console that owns it. With nobody else
    // driving, going quiet is what the robot's own watchdog is for (kStaleMs
    // -> stand, kRelaxMs -> torque off) -- a better failsafe than anything
    // this end could send.
    void goReadonly();
    // Start commanding. False when no --host was given: see has_dest.
    bool takeControl();

    // Re-point the console at a different robot (or at the sim) without
    // restarting it, in the given mode. Closes both sockets and re-opens
    // them, so it is `open`/`listen` and not a patch to a live one -- the
    // bind is the part that has to change, and a half-moved link is worse
    // than a closed one. The per-target counters are reset with it: `seq`,
    // `sent` and the beacon tallies all describe a conversation with ONE
    // robot, and carrying them across would make `lag` meaningless in
    // exactly the way the observer's own header had to avoid.
    bool relink(const char* host, int cmd_port, int tlm_port, bool observe);
    bool setMirror(const char* host, int port);
    void clearMirror() { mirror_on = false; }
    bool setWatch(const char* host, int port);
    void clearWatch() { watch_on = false; }
    void close();

    // vx/wz only -- the classic 14 B frame.
    void send(float vx, float wz, uint8_t flags);
    // The full intent. Emits the SHORT frame whenever the extras are at their
    // defaults, exactly as link/protocol.py:encode_command does, so a console
    // that never touches a slider is byte-identical to one that cannot.
    void sendFull(float vx, float vy, float wz, const Ext& ext, uint8_t flags);

    void emit(const uint8_t* wire, size_t n);

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
    // OBSERVER: every control here refuses, and SAYS it refused.
    //
    // The silence is already guaranteed one layer down (Link::listen leaves
    // tx == -1), so this is not what makes the mode safe -- it is what makes
    // it HONEST. A console whose E-STOP darkens on click while nothing goes
    // out is worse than one with no E-STOP at all: it invites an operator to
    // press it in the second that matters and believe the robot was told.
    // Refusing out loud is the only answer that cannot be misread, and it
    // lives here rather than in either front end so the window and the
    // terminal cannot come to different conclusions about what a watcher may
    // do (the same argument client.h opens with).
    bool readonly = false;
    // "Beacon joint angles too" (kFlagPose). Independent of arming: it asks
    // for telemetry, it does not command anything.
    bool want_pose = false;
    // Frames left to spend asking for a servo reset (kFlagHome). See
    // kHomeFrames; sendIntent spends one per frame.
    int home_frames = 0;
    // When the last reset was asked for, or -1 once it has been answered
    // for. See checkHome: a request nobody answers must not look like a
    // request that worked.
    double home_at_ms = -1.0;
    bool held[kAxCount] = {false, false, false, false, false, false};
    Ext ext;
    double armed_at_ms = 0.0;
    char note[192] = "";

    void say(const char* s);
    void sayf2(const char* fmt, const char* a);

    // True (and says so) when this console only watches. Every control below
    // that would reach the robot asks this first.
    bool refuseReadonly();

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

    // "Reset the servos": every joint to its calibrated zero, the standing
    // pose. Deliberately NOT gated on `armed` or on the E-stop -- the states
    // this is for are exactly the ones where nothing else may move the robot
    // (E-stopped, fall-latched, benched after a run). The robot benches to do
    // it, so this disarms here too, and re-arming stays a deliberate act.
    void requestHome(double now_ms);

    // The robot is authoritative here too. A reset it HEARD produces one of
    // the kFlagHome verdicts in the BENCH beacon (linkproto::isHomeResult) --
    // it worked, or it says why. Silence after the edge has had time to land
    // means the request never arrived, or arrived at a firmware that predates
    // the bit and ignored it. Both look identical from the operator's chair
    // -- a button that does nothing -- so say so rather than let them hunt a
    // dead radio or a seized servo. Returns true if it just said it.
    bool checkHome(const Link& link, double now_ms);

    // What goes on the wire this tick.
    void frame(const Speeds& s, float& vx, float& vy, float& wz, Ext& ext_out,
               uint8_t& flags) const;

    // Take over from whoever is driving, WITHOUT moving the robot.
    //
    // This is the one genuinely dangerous moment in the observer feature, and
    // the danger is not obvious. linkproto::ArmLatch is a SINGLE latch on the
    // robot fed by every decoded frame in arrival order, whoever sent it --
    // it tracks a level, not a per-sender session. So a console that starts
    // commanding a LIVE robot with ARM low does not "begin disarmed": it puts
    // a 1 -> 0 edge on the wire, and the robot benches with torque off and
    // falls over, mid-stride, as the direct result of someone clicking a
    // button labelled "take control".
    //
    // So the handover ADOPTS the level the robot is already reporting, and is
    // a no-op on the wire by construction: armed if the beacon says the loop
    // is running, E-stopped if it says latched, and no axis held either way,
    // so the robot keeps standing until an actual key is pressed. With no
    // fresh beacon there is nothing to adopt and nothing safe to assume, so
    // it starts disarmed -- benching a robot we cannot see is the right
    // answer to not knowing.
    void adopt(const Link& link, double now_ms);

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
// `in` is non-const because a home request is spent as it is sent: the
// kFlagHome level lasts kHomeFrames frames and this is what counts them down.
void sendIntent(Link& link, Intent& in, const Speeds& s, float& vx,
                float& vy, float& wz, Ext& ext_out, uint8_t& flags);

}  // namespace bimo

#endif  // BIMO_LINK_CLIENT_H_
