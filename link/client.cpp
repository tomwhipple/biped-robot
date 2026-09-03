#include "client.h"

#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace bimo {
namespace {

using namespace linkproto;

// The wire quantises every channel to milli-units, so "is this slider at its
// default" has to be asked in the units the robot will actually see. Asking
// in floats would emit a 24 B frame for a slider sitting 1e-7 off 1.0 -- a
// frame that firmware older than 2026-08-31 drops by length.
int milli(float x) {
    const double v = nearbyint(static_cast<double>(x) * 1000.0);
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int>(v);
}

}  // namespace

double nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) * 1000.0 +
           static_cast<double>(ts.tv_nsec) / 1e6;
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

bool splitHostPort(const char* spec, char* host, size_t cap, int& port) {
    if (spec == nullptr || cap == 0) return false;
    const char* colon = strrchr(spec, ':');
    const size_t hlen = colon != nullptr ? static_cast<size_t>(colon - spec)
                                         : strlen(spec);
    if (hlen == 0 || hlen + 1 > cap) return false;
    memcpy(host, spec, hlen);
    host[hlen] = 0;
    if (colon == nullptr) return true;           // no port: keep the default
    const char* p = colon + 1;
    if (*p == 0) return false;
    int v = 0;
    for (; *p != 0; ++p) {
        if (*p < '0' || *p > '9') return false;
        v = v * 10 + (*p - '0');
        if (v > 65535) return false;
    }
    if (v == 0) return false;
    port = v;
    return true;
}

const char* axisName(int axis) {
    switch (axis) {
        case kAxForward: return "FORWARD";
        case kAxBack: return "BACKWARD";
        case kAxLeft: return "STRAFE LEFT";
        case kAxRight: return "STRAFE RIGHT";
        case kAxTurnLeft: return "TURN LEFT";
        case kAxTurnRight: return "TURN RIGHT";
        default: return "?";
    }
}

// -- Ext --------------------------------------------------------------------
bool Ext::atDefaults() const {
    return milli(crouch) == 1000 && milli(lift) == 0 && milli(foot_dx) == 0 &&
           milli(foot_dz) == 0;
}

void Ext::clamp() {
    Command cmd = {};
    cmd.crouch = crouch;
    cmd.lift = lift;
    cmd.foot_dx = foot_dx;
    cmd.foot_dz = foot_dz;
    clampExtToEnvelope(cmd);
    crouch = cmd.crouch;
    lift = cmd.lift;
    foot_dx = cmd.foot_dx;
    foot_dz = cmd.foot_dz;
}

void Ext::reset() {
    crouch = 1.0f;
    lift = 0.0f;
    foot_dx = 0.0f;
    foot_dz = 0.0f;
}

// -- Link -------------------------------------------------------------------
bool Link::open(const char* host, int cmd_port, int tlm_port) {
    readonly = false;
    tx = socket(AF_INET, SOCK_DGRAM, 0);
    rx = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx < 0 || rx < 0) return false;
    to.sin_family = AF_INET;
    to.sin_port = htons(static_cast<uint16_t>(cmd_port));
    if (inet_pton(AF_INET, host, &to.sin_addr) != 1) return false;
    has_dest = true;
    int one = 1;
    setsockopt(rx, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(static_cast<uint16_t>(tlm_port));
    if (bind(rx, reinterpret_cast<sockaddr*>(&bind_addr), sizeof bind_addr) < 0) {
        return false;
    }
    timeval tv = {0, 0};
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return true;
}

// The observer's open(). Deliberately NOT `open(nullptr, ...)`: a mode that
// cannot transmit should not share a constructor with one that can, and the
// difference an operator's safety rests on should be visible at the call site.
bool Link::listen(int tlm_port, const char* host, int cmd_port) {
    readonly = true;
    tx = -1;                      // no transmit socket exists yet
    // Resolved but not connected: `to` is where takeControl() would send, and
    // resolving it now means a bad --host is a startup error rather than a
    // button that fails at the moment it is finally needed.
    if (host != nullptr) {
        to.sin_family = AF_INET;
        to.sin_port = htons(static_cast<uint16_t>(cmd_port));
        if (inet_pton(AF_INET, host, &to.sin_addr) != 1) return false;
        has_dest = true;
    }
    rx = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx < 0) return false;
    int one = 1;
    // SO_REUSEADDR because the whole point is to sit alongside something
    // else: a driver on this same machine, or a previous observer still
    // closing. It costs nothing here -- an observer has nothing to lose by
    // sharing the port, and gains the ability to be started and restarted
    // under a running session.
    setsockopt(rx, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(static_cast<uint16_t>(tlm_port));
    if (bind(rx, reinterpret_cast<sockaddr*>(&bind_addr), sizeof bind_addr) < 0) {
        return false;
    }
    timeval tv = {0, 0};
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    return true;
}

void Link::goReadonly() {
    if (tx >= 0) ::close(tx);
    tx = -1;
    readonly = true;
    // Zeroed so the status line cannot go on reporting the size of the last
    // frame from a console that is no longer sending any.
    last_len = 0;
}

bool Link::takeControl() {
    if (!has_dest) return false;
    if (tx < 0) {
        tx = socket(AF_INET, SOCK_DGRAM, 0);
        if (tx < 0) return false;
    }
    // Last, and only once the socket is real: readonly must never read false
    // while there is still nothing to send on.
    readonly = false;
    return true;
}

bool Link::relink(const char* host, int cmd_port, int tlm_port, bool observe) {
    close();
    has_dest = false;
    seq = sent = 0;
    last_len = 0;
    have_tlm = false;
    tlm_at_ms = -1.0;
    tlm_count = tlm_bad = watched = 0;
    rate_window = 0;
    rate_hz = 0.0;
    rate_at_ms = 0.0;
    if (observe) return listen(tlm_port, host, cmd_port);
    return open(host, cmd_port, tlm_port);
}

bool Link::setWatch(const char* host, int port) {
    watch_to.sin_family = AF_INET;
    watch_to.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &watch_to.sin_addr) != 1) {
        watch_on = false;
        return false;
    }
    watch_on = true;
    return true;
}

void Link::close() {
    if (tx >= 0) ::close(tx);
    if (rx >= 0) ::close(rx);
    tx = rx = -1;
}

// One encode, one destination.
void Link::emit(const uint8_t* wire, size_t n) {
    // The observer's last line: tx is already -1, so this is belt as well as
    // braces -- but it also keeps `last_len` and `sent` HONEST. A watcher's
    // status line must read "0 B on the wire", not the size of a frame it
    // composed and threw away.
    if (readonly) return;
    last_len = n;
    if (sendto(tx, wire, n, 0, reinterpret_cast<sockaddr*>(&to), sizeof to) ==
        static_cast<ssize_t>(n)) {
        ++sent;
    }
}

void Link::send(float vx, float wz, uint8_t flags) {
    if (readonly) return;                 // see sendFull()
    uint8_t wire[kCmdLen];
    emit(wire, encodeCommand(wire, seq++, vx, wz, flags));
}

void Link::sendFull(float vx, float vy, float wz, const Ext& ext,
                    uint8_t flags) {
    // An observer returns BEFORE `seq` moves, not merely before the sendto.
    // seq is the number the header reports and lag() measures the robot's
    // echo against, so a console that has sent nothing must still be at zero
    // -- otherwise a watcher shows a rising sequence and a lag of minus
    // several hundred, which reads exactly like a robot that has stopped
    // answering. emit() refuses too; that is the backstop, this is the rule.
    if (readonly) return;
    // The short frame whenever the extras are at their trained defaults: old
    // firmware drops a 24 B frame by length, so a console that is not asking
    // for anything extended must not look like one that is. This is the rule
    // link/protocol.py:encode_command implements; encodeCommandExt() alone
    // always emits 24 B.
    if (milli(vy) == 0 && ext.atDefaults()) {
        send(vx, wz, flags);
        return;
    }
    Command cmd = {};
    cmd.seq = seq++;
    cmd.vx = vx;
    cmd.wz = wz;
    cmd.flags = flags;
    cmd.vy = vy;
    cmd.crouch = ext.crouch;
    cmd.lift = ext.lift;
    cmd.foot_dx = ext.foot_dx;
    cmd.foot_dz = ext.foot_dz;
    uint8_t wire[kCmdLenExt];
    emit(wire, encodeCommandExt(wire, cmd));
}

void Link::poll(double now_ms) {
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
        // Forward to the watcher, VERBATIM and only once decoded. Verbatim so
        // the observer runs the same decodeTelemetry against the same bytes
        // the robot signed -- a relay that re-encoded would put a frame on the
        // wire the robot never sent, and the CRC would stop meaning anything.
        // Only once decoded so stray traffic on an open port is dropped here
        // rather than forwarded on to be dropped there.
        if (watch_on && tx >= 0) {
            if (sendto(tx, buf, static_cast<size_t>(n), 0,
                       reinterpret_cast<sockaddr*>(&watch_to),
                       sizeof watch_to) == n) {
                ++watched;
            }
        }
    }
    if (now_ms - rate_at_ms >= 1000.0) {
        rate_hz = static_cast<double>(rate_window) * 1000.0 /
                  (now_ms - rate_at_ms);
        rate_window = 0;
        rate_at_ms = now_ms;
    }
}

double Link::tlmAgeMs(double now_ms) const {
    return have_tlm ? now_ms - tlm_at_ms : -1.0;
}

bool Link::linkLost(double now_ms) const {
    return !have_tlm || tlmAgeMs(now_ms) > kTlmLostMs;
}

long Link::lag() const {
    return static_cast<long>(seq) - 1 - static_cast<long>(tlm.seq_echo);
}

// -- Intent -----------------------------------------------------------------
void Intent::say(const char* s) {
    strncpy(note, s, sizeof note - 1);
    note[sizeof note - 1] = 0;
}

void Intent::sayf2(const char* fmt, const char* a) {
    snprintf(note, sizeof note, fmt, a);
}

bool Intent::moving() const {
    for (int i = 0; i < kAxCount; ++i) {
        if (held[i]) return true;
    }
    return false;
}

void Intent::releaseAll() {
    for (int i = 0; i < kAxCount; ++i) held[i] = false;
}

// The one refusal, worded once. Every observer control routes through it, so
// the console cannot say "watching only" in one place and something vaguer in
// another -- and it names the way out, because an operator who reaches for a
// control on a moving robot needs to be told what WOULD work, not only that
// this did not.
bool Intent::refuseReadonly() {
    if (!readonly) return false;
    say("OBSERVER: this console only watches -- it has no transmit socket. "
        "The session driving the robot owns every control; restart without "
        "--readonly to take over.");
    return true;
}

bool Intent::press(int axis, double now_ms) {
    (void)now_ms;
    if (refuseReadonly()) return false;
    if (!armed) {
        say("not armed -- press [a] first");
        return false;
    }
    if (estop) {
        say("E-STOP latched -- [space] to stand, then move");
        return false;
    }
    if (axis < 0 || axis >= kAxCount) return false;
    held[axis] = true;
    return true;
}

void Intent::release(int axis) {
    if (axis < 0 || axis >= kAxCount) return;
    held[axis] = false;
}

void Intent::arm(double now_ms) {
    if (refuseReadonly()) return;
    armed = true;
    estop = false;
    armed_at_ms = now_ms;
    say("arm sent: waiting for the robot to leave BENCH");
}

void Intent::disarm() {
    if (refuseReadonly()) return;
    armed = false;
    releaseAll();
    estop = false;
    say("disarm sent: robot benches, torque off");
}

void Intent::toggleArm(double now_ms) {
    if (armed) {
        disarm();
    } else {
        arm(now_ms);
    }
}

void Intent::stand(const char* why) {
    releaseAll();
    if (refuseReadonly()) return;
    if (estop) {
        estop = false;
        say("E-stop cleared: standing");
    } else if (why) {
        say(why);
    }
}

void Intent::requestHome(double now_ms) {
    if (refuseReadonly()) return;
    home_frames = kHomeFrames;
    home_at_ms = now_ms;
    // The robot benches to run the move (the CLI half of the firmware owns
    // the servo bus), so drop ARM here too rather than let the console go on
    // claiming a run the robot has ended. A latched E-stop is NOT cleared:
    // homing answers "the robot is on the floor", not "carry on".
    armed = false;
    releaseAll();
    say("reset servos: robot benches, joints slew to the stand, torque HOLDS");
}

void Intent::fireEstop() {
    // The one refusal that is genuinely uncomfortable, and still the right
    // one: see Intent::readonly in client.h. A latched-looking E-STOP that
    // sent nothing would be a worse failure than this message.
    if (refuseReadonly()) return;
    estop = true;
    releaseAll();
    say("E-STOP sent (latched until [space])");
}

void Intent::adopt(const Link& link, double now_ms) {
    readonly = false;
    releaseAll();
    home_frames = 0;
    home_at_ms = -1.0;
    const bool fresh = link.have_tlm && !link.linkLost(now_ms);
    const LinkState st = link.tlm.state;
    // "Armed" is the loop RUNNING, which is every state except kBench --
    // including kFallen and the low-battery pair, where torque is off but the
    // loop still holds the arm level. Reading only kLive here would drop the
    // level on a robot that had merely tripped its fall latch, benching it.
    armed = fresh && st != LinkState::kBench;
    estop = fresh && st == LinkState::kEstop;
    // The grace period starts now: syncToRobot gives an arm kArmSyncMs to be
    // reflected, and an adopted level has had no edge of its own to wait on.
    armed_at_ms = now_ms;
    if (!fresh) {
        say("took control with NO beacon: starting disarmed, which benches "
            "the robot if it was running. Nothing else is safe without "
            "knowing what it is doing.");
    } else if (armed) {
        say("took control of a RUNNING robot: holding its arm level so the "
            "handover moves nothing. It stands until you press a key.");
    } else {
        say("took control: the robot is benched, so this console is "
            "disarmed too. Press [a] to arm.");
    }
}

void Intent::frame(const Speeds& s, float& vx, float& vy, float& wz,
                   Ext& ext_out, uint8_t& flags) const {
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;
    ext_out.reset();
    flags = 0;
    if (want_pose) flags |= kFlagPose;    // a request, not a command
    if (want_att) flags |= kFlagAtt;      // likewise
    // Before every early return below, and gated on nothing: a servo reset
    // must reach the robot from the states that refuse to move -- E-stopped,
    // disarmed, fall-latched. That is the whole reason it is its own bit.
    if (home_frames > 0) flags |= kFlagHome;
    if (armed) flags |= kFlagArm;
    if (estop) {
        flags |= kFlagEstop;
        return;                           // kFlagPose already set above
    }
    if (!armed) return;
    const bool extras = !ext.atDefaults();
    if (!moving() && !extras) return;
    // ENABLE gates the extended channels too (linkproto::Watchdog::accept), so
    // a crouch held at zero velocity still needs it -- a commanded posture is
    // a command, not a stand.
    flags |= kFlagEnable;
    if (held[kAxForward]) vx += s.vx;
    if (held[kAxBack]) vx -= s.vx;
    if (held[kAxLeft]) vy += s.vy;
    if (held[kAxRight]) vy -= s.vy;
    if (held[kAxTurnLeft]) wz += s.wz;
    if (held[kAxTurnRight]) wz -= s.wz;
    ext_out = ext;
}

bool Intent::checkHome(const Link& link, double now_ms) {
    if (home_at_ms < 0.0) return false;
    if (link.linkLost(now_ms)) return false;   // we know nothing; say nothing
    if (now_ms - home_at_ms <= kArmSyncMs) return false;
    home_at_ms = -1.0;                         // asked and answered, either way
    if (link.tlm.state == LinkState::kBench &&
        isHomeResult(diagArmResult(telemetryDiag(link.tlm)))) {
        return false;      // answered -- the BENCH line already says HOW
    }
    say("no answer to the reset: this robot's firmware may predate the "
        "request (re-flash it), or the frames never arrived");
    return true;
}

bool Intent::syncToRobot(const Link& link, double now_ms) {
    if (!armed || link.linkLost(now_ms)) return false;
    if (link.tlm.state != LinkState::kBench) return false;
    if (now_ms - armed_at_ms <= kArmSyncMs) return false;
    armed = false;
    releaseAll();
    estop = false;
    // The robot now says WHY (protocol.h kDiag*), so quote it instead of
    // guessing at "no calibration?" the way this used to.
    sayf2("robot stayed BENCH after arm: %s",
          diagReason(telemetryDiag(link.tlm)));
    return true;
}

const char* motionText(const Intent& in) {
    for (int i = 0; i < kAxCount; ++i) {
        if (in.held[i]) return axisName(i);
    }
    return "stand";
}

void sendIntent(Link& link, Intent& in, const Speeds& s, float& vx,
                float& vy, float& wz, Ext& ext_out, uint8_t& flags) {
    // The frame is computed even for an observer -- both front ends draw
    // vx/vy/wz and the flags from these outputs -- and then declined one layer
    // down, in Link::sendFull. One gate, at the socket, rather than a second
    // copy of the rule here.
    in.frame(s, vx, vy, wz, ext_out, flags);
    link.sendFull(vx, vy, wz, ext_out, flags);
    // Spent on the way out, not on the way in: the count is frames actually
    // PUT ON THE WIRE, so a request cannot expire in a console that is not
    // sending (a window that lost focus, a paused test).
    if (in.home_frames > 0) --in.home_frames;
}

}  // namespace bimo
