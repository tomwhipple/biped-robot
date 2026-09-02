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
    tx = socket(AF_INET, SOCK_DGRAM, 0);
    rx = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx < 0 || rx < 0) return false;
    to.sin_family = AF_INET;
    to.sin_port = htons(static_cast<uint16_t>(cmd_port));
    if (inet_pton(AF_INET, host, &to.sin_addr) != 1) return false;
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

bool Link::setMirror(const char* host, int port) {
    mirror_to.sin_family = AF_INET;
    mirror_to.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &mirror_to.sin_addr) != 1) {
        mirror_on = false;
        return false;
    }
    mirror_on = true;
    return true;
}

void Link::close() {
    if (tx >= 0) ::close(tx);
    if (rx >= 0) ::close(rx);
    tx = rx = -1;
}

// One encode, up to two destinations.
void Link::emit(const uint8_t* wire, size_t n) {
    last_len = n;
    if (sendto(tx, wire, n, 0, reinterpret_cast<sockaddr*>(&to), sizeof to) ==
        static_cast<ssize_t>(n)) {
        ++sent;
    }
    if (mirror_on) {
        sendto(tx, wire, n, 0, reinterpret_cast<sockaddr*>(&mirror_to),
               sizeof mirror_to);
    }
}

void Link::send(float vx, float wz, uint8_t flags) {
    uint8_t wire[kCmdLen];
    emit(wire, encodeCommand(wire, seq++, vx, wz, flags));
}

void Link::sendFull(float vx, float vy, float wz, const Ext& ext,
                    uint8_t flags) {
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

bool Intent::press(int axis, double now_ms) {
    (void)now_ms;
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
    armed = true;
    estop = false;
    armed_at_ms = now_ms;
    say("arm sent: waiting for the robot to leave BENCH");
}

void Intent::disarm() {
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
    if (estop) {
        estop = false;
        say("E-stop cleared: standing");
    } else if (why) {
        say(why);
    }
}

void Intent::requestHome() {
    home_frames = kHomeFrames;
    // The robot benches to run the move (the CLI half of the firmware owns
    // the servo bus), so drop ARM here too rather than let the console go on
    // claiming a run the robot has ended. A latched E-stop is NOT cleared:
    // homing answers "the robot is on the floor", not "carry on".
    armed = false;
    releaseAll();
    say("reset servos: robot benches, joints slew to the stand, torque HOLDS");
}

void Intent::fireEstop() {
    estop = true;
    releaseAll();
    say("E-STOP sent (latched until [space])");
}

void Intent::frame(const Speeds& s, float& vx, float& vy, float& wz,
                   Ext& ext_out, uint8_t& flags) const {
    vx = 0.0f;
    vy = 0.0f;
    wz = 0.0f;
    ext_out.reset();
    flags = 0;
    if (want_pose) flags |= kFlagPose;    // a request, not a command
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
    in.frame(s, vx, vy, wz, ext_out, flags);
    link.sendFull(vx, vy, wz, ext_out, flags);
    // Spent on the way out, not on the way in: the count is frames actually
    // PUT ON THE WIRE, so a request cannot expire in a console that is not
    // sending (a window that lost focus, a paused test).
    if (in.home_frames > 0) --in.home_frames;
}

}  // namespace bimo
