// C++ port of link/protocol.py -- the laptop <-> robot command link.
//
// link/protocol.py is the REFERENCE. This file conforms to it byte for byte,
// never the other way round, and tools/gen_protocol_vectors.py freezes the
// Python side's output into host/vectors/protocol_vectors.h so a divergence is
// a failing test rather than a mystery on the bench. See docs/control-channel.md.
//
// Everything here is pure: no sockets, no clock (the Watchdog takes now_ms as
// an argument exactly like the Python one), so it runs on the host under
// sanitizers.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace linkproto {

// -- framing ---------------------------------------------------------------
constexpr uint8_t kMagicCmd[2] = {'B', 'M'};   // laptop -> robot
constexpr uint8_t kMagicTlm[2] = {'B', 'T'};   // robot -> laptop
constexpr uint8_t kVersion = 1;
constexpr size_t kCmdLen = 14;
// Extended command frame (2026-08-31): the same 12-byte prefix, then five
// more int16 milli-channels (vy, crouch, lift, foot_dx, foot_dz) completing
// walker_env.set_command()'s 7-wide ext_cmd vector. LENGTH selects the
// layout, no version bump: a 14 B frame decodes with the extras at their
// trained defaults (0, 1.0, 0, 0, 0), so every pre-extension sender keeps
// working. protocol.py is the reference for the sender-side rule (emit the
// short frame whenever the extras are at defaults).
constexpr size_t kCmdLenExt = 24;
constexpr size_t kTlmLen = 20;
// Extended TELEMETRY (2026-09-01): the same 18-byte body, then one int16
// milli-radian per joint, then the CRC. For mirror mode -- driving the real
// robot while a sim follows its observed pose.
//
// This could NOT be done the way the extended command frame was. There the
// robot is the decoder and was taught both lengths; here the robot is the
// SENDER and every client rejects an unexpected length, so a robot that
// simply began beaconing 40 B would blind every existing commander at once.
// Hence kFlagPose: the long frame is REQUESTED, never volunteered. See
// docs/mirror-mode.md.
constexpr size_t kNumJoints = 10;   // obs_spec order, NOT servo-id order
constexpr size_t kTlmLenExt = kTlmLen + 2 * kNumJoints;

constexpr uint16_t kCmdPort = 4210;
constexpr uint16_t kTlmPort = 4211;

// -- command flags ---------------------------------------------------------
constexpr uint8_t kFlagEnable = 1u << 0;   // 0 = stand still regardless of vx/wz
constexpr uint8_t kFlagEstop = 1u << 1;    // latching torque release
constexpr uint8_t kFlagArm = 1u << 2;      // operator wants the loop armed; ArmLatch
// "Beacon joint angles too" (kTlmLenExt). A level, not an edge: stop asking
// and the very next beacon is classic again.
constexpr uint8_t kFlagPose = 1u << 3;
// "Put every joint back at its calibrated zero -- the standing pose -- now."
// (2026-09-02) A RECOVERY action rather than a command, and the difference is
// the whole point: it is honoured while the loop is BENCHED, while the FALL
// latch is tripped and while the E-stop is latched -- exactly the states in
// which every other channel on this wire correctly refuses to move anything,
// and exactly the states an operator is in when the robot is a heap on the
// floor and needs to be stood back up before it can be armed again.
//
// It does not arm and cannot be used to walk: the robot BENCHES first, the
// CLI half of the firmware owns the bus for the move, and the joints slew at
// the bench's gentle speed (cli.cpp cmdHome). Re-arming stays as deliberate
// as it ever was -- a fresh ARM edge.
//
// Acted on by its RISING EDGE (HomeLatch), like kFlagArm and for the same
// two reasons: a level would re-issue the move every frame at 20 Hz, and a
// client that reboots with the bit set must not move a robot nobody is
// watching.
constexpr uint8_t kFlagHome = 1u << 4;

// -- timing ----------------------------------------------------------------
constexpr float kSendHz = 20.0f;
constexpr float kStaleMs = 250.0f;         // -> decay to the trained stand
constexpr float kRelaxMs = 5000.0f;        // -> torque off entirely
constexpr uint32_t kSeqResyncGap = 1000;   // bigger backwards jump = restart

// -- trained command envelope ----------------------------------------------
// MUST track walker_env's cmd_v_range / cmd_w_range for the deployed policy.
// The band (0, kVMinWalk) is a hole in the training distribution, not a slow
// walk, so clampToEnvelope() snaps it to a stand.
constexpr float kVMinWalk = 0.3f;
constexpr float kVMax = 1.0f;
constexpr float kWMax = 1.0f;
// extended-channel envelope, from the walker_env training draws (protocol.py
// VY_MAX / CROUCH_MIN / FOOT_D_MAX -- keep in step):
constexpr float kVyMax = 0.3f;      // lateral sway amplitude
constexpr float kCrouchMin = 0.6f;  // crouch_range[0], the battguard floor
constexpr float kFootDMax = 0.05f;  // traj_radius hi: foot offsets, m

enum class Err : uint8_t {
    kOk = 0,
    kBadLength,
    kBadMagic,
    kBadVersion,
    kBadCrc,
    kBadState,       // telemetry only: link-state byte out of range
};

enum class LinkState : uint8_t {
    // Order matters: this is the on-the-wire encoding, and it is
    // list(LinkState) order in protocol.py.
    kLive = 0,
    kStand = 1,
    kRelax = 2,
    kEstop = 3,
    // Pack under-voltage, latched by battguard::Guard. Unlike every state
    // above, these are decided by the ROBOT and no command clears them.
    kLowBattLand = 4,   // crouching down under control on a flat pack
    kLowBattSafe = 5,   // crouch finished, torque off, and it stays off
    // Torso down (up_z below kFallUpZ, debounced), robot-latched: torque off
    // so a downed robot does not grind its servos against the floor. A fall
    // DISARMS the robot (ctrl_task): the state is reported for the fall tick,
    // then the loop benches. Re-arming is deliberate -- a fresh wireless ARM
    // edge or a tethered `run` -- never automatic.
    kFallen = 6,
    // Control loop benched: CLI owns the bus, the link commands nothing,
    // torque is off. Boot state. Left via an ArmLatch edge or the tethered
    // `run`.
    kBench = 7,
    kMaxState = kBench,
};

// -- bench diagnostics (2026-08-30) ----------------------------------------
// Why the loop is NOT armed, carried to the operator over the radio.
//
// The refusal used to exist only as a line of UART text, so a robot that
// declined a wireless ARM just sat in BENCH saying nothing and the only way
// to find out why was to plug the tether. This byte is that answer on the
// wire. It rides in `seq_echo` and ONLY while the reported state is kBench:
// seq_echo means "the last command seq the loop APPLIED", and a benched loop
// applies nothing, so in exactly that state the field carries no information
// to destroy. No version bump, no length change -- every commander built
// before this still decodes every frame (docs/control-channel.md).
//
// Bit layout, APPEND-ONLY like the LinkState enum: reserved bits are sent
// zero and a client must ignore the ones it does not know.
constexpr uint8_t kDiagRun = 1u << 0;      // mode is kRun (0 = benched)
constexpr uint8_t kDiagCalOk = 1u << 1;    // as-built calibration came from NVS
// bits 2-3 reserved, sent zero
constexpr uint8_t kDiagArmShift = 4;       // bits 4-7: the ArmResult below
constexpr uint8_t kDiagArmMask = 0xF0;

// What became of the last mode request -- a wireless ARM edge or a typed
// `run`/`bench`. APPEND ONLY: a future refusal reason takes the next value,
// and a client that does not know it must fall back to a generic message
// rather than mis-report an older reason.
enum class ArmResult : uint8_t {
    kNone = 0,           // nothing has asked for a mode change yet
    kAccepted = 1,       // the last request was honoured (arm or disarm)
    kRefusedNoCal = 2,   // arm refused: no as-built calibration in NVS
    kDisarmedFall = 3,   // the FALL latch tripped: run over, robot disarmed
    kDisarmedHome = 4,   // a kFlagHome request benched the loop and homed it
    // ... and the three ways that request can fail. They exist for the same
    // reason kRefusedNoCal does: a reset that silently does nothing sends an
    // operator to look for a broken button, a dead radio or a seized servo,
    // when the robot knew the answer all along (docs/control-channel.md,
    // "Saying WHY, over the radio").
    kHomeNoCal = 5,      // reset REFUSED: no as-built calibration in NVS
    kHomeLowBatt = 6,    // reset REFUSED: the pack guard has torque latched off
    kHomeBusFailed = 7,  // reset FAILED: the servo bus did not accept it
};

// True for every verdict a kFlagHome request can produce. A console that
// asked for a reset uses this to tell "the robot answered me" from "the robot
// is talking about something else" -- an old firmware that never heard of the
// request still reports kAccepted for the disarm that rode in with it.
inline bool isHomeResult(ArmResult r) {
    return r == ArmResult::kDisarmedHome || r == ArmResult::kHomeNoCal ||
           r == ArmResult::kHomeLowBatt || r == ArmResult::kHomeBusFailed;
}

uint8_t packDiag(bool run, bool cal_ok, ArmResult result);
ArmResult diagArmResult(uint8_t diag);
// One line an operator can act on. Never null, and safe for a diag byte from
// a firmware newer than this client.
const char* diagReason(uint8_t diag);

// -- fall detection (robot-side, ctrl_task) --------------------------------
// The threshold is walker_env's own fall_up_z: the sim terminates an episode
// below it, so past this line the policy is operating outside anything it
// trained on and holding torque only feeds the fall. Debounced over ticks
// because a footfall transient dips up_z briefly; 200 ms of "down" is a
// torso on the floor, not a wobble.
constexpr float kFallUpZ = 0.4f;
constexpr float kFallDebounceMs = 200.0f;
// The firmware latch no longer auto-clears (a fall DISARMS the robot,
// ctrl_task.cpp); these two remain for the python twin's Watchdog and any
// host-side "is it righted yet" display.
constexpr float kUprightUpZ = 0.7f;     // hysteresis: righted is > this ...
constexpr float kUprightMs = 2000.0f;   // ... held this long, + ENABLE off

struct Command {
    uint32_t seq;
    float vx;        // body-frame forward velocity, m/s
    float wz;        // yaw rate, rad/s
    uint8_t flags;
    // extended channels (kCmdLenExt frames); a classic frame decodes to
    // exactly these defaults -- walker_env ext_cmd layout beyond vx/wz
    float vy = 0.0f;       // lateral velocity, m/s
    float crouch = 1.0f;   // stance-height fraction command (cmd[3])
    float lift = 0.0f;     // swing-foot selector: -1 left, +1 right
    float foot_dx = 0.0f;  // swing-foot target x offset, m
    float foot_dz = 0.0f;  // swing-foot target extra height, m

    bool enabled() const { return (flags & kFlagEnable) != 0; }
    bool estop() const { return (flags & kFlagEstop) != 0; }
    bool arm() const { return (flags & kFlagArm) != 0; }
    bool pose() const { return (flags & kFlagPose) != 0; }
    bool home() const { return (flags & kFlagHome) != 0; }
};

struct Telemetry {
    uint32_t seq_echo;
    LinkState state;
    float vbat_v;
    float up_z;          // torso up-vector z; 1.0 = perfectly upright
    float vx_est;
    float wz_est;
    uint8_t servo_err;   // bitmask, bit i = servo ID i+1 faulted
    uint8_t loop_late_pct;
    // Measured joint angles, radians, obs_spec order. n_joints is 0 on a
    // classic 20 B frame -- which is every frame, unless a commander asked
    // with kFlagPose.
    uint8_t n_joints = 0;
    float joints[kNumJoints] = {0};
};

// The bench diagnostic byte this frame carries, or 0 when it carries none.
// Reading seq_echo directly would be a bug waiting to happen: the byte is
// only meaningful while the state is kBench.
uint8_t telemetryDiag(const Telemetry& t);

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
// crc16Ccitt("123456789") == 0x29B1 -- the standard check value, asserted in
// the host tests so this port is diffed against a number, not against
// "whatever Python said".
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// Snap a raw command into the region the policy was actually trained on.
// Enforced robot-side because the robot is what eats a bad command.
void clampToEnvelope(float vx, float wz, float& out_vx, float& out_wz);

// The extended channels, snapped into their trained draws (in place).
void clampExtToEnvelope(Command& pkt);

// Encode into `out` (kCmdLen / kTlmLen bytes). Returns bytes written.
size_t encodeCommand(uint8_t* out, uint32_t seq, float vx, float wz,
                     uint8_t flags);
// Extended frame: always kCmdLenExt bytes (senders wanting the short-frame
// default rule use protocol.py; firmware-side encoding exists for the twins
// and tests).
size_t encodeCommandExt(uint8_t* out, const Command& cmd);
// Accepts kCmdLen (extras -> defaults) and kCmdLenExt frames.
Err decodeCommand(const uint8_t* buf, size_t len, Command& out);

// Emits kTlmLenExt when t.n_joints == kNumJoints, else kTlmLen. `out` must
// therefore have room for kTlmLenExt whenever joints are set. Returns the
// bytes written -- USE IT: sending kTlmLen of a long frame truncates it and
// every CRC downstream fails.
size_t encodeTelemetry(uint8_t* out, const Telemetry& t);
// Accepts kTlmLen (n_joints -> 0) and kTlmLenExt frames.
Err decodeTelemetry(const uint8_t* buf, size_t len, Telemetry& out);

}  // namespace linkproto
