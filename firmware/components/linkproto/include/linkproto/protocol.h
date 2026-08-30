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
constexpr size_t kTlmLen = 20;

constexpr uint16_t kCmdPort = 4210;
constexpr uint16_t kTlmPort = 4211;

// -- command flags ---------------------------------------------------------
constexpr uint8_t kFlagEnable = 1u << 0;   // 0 = stand still regardless of vx/wz
constexpr uint8_t kFlagEstop = 1u << 1;    // latching torque release
constexpr uint8_t kFlagArm = 1u << 2;      // operator wants the loop armed; ArmLatch

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
    // so a downed robot does not grind its servos against the floor. Clears
    // without a reboot: upright for kUprightMs plus a frame with ENABLE off.
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
};

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
constexpr float kUprightUpZ = 0.7f;     // hysteresis: righted is > this ...
constexpr float kUprightMs = 2000.0f;   // ... held this long, + ENABLE off

struct Command {
    uint32_t seq;
    float vx;        // body-frame forward velocity, m/s
    float wz;        // yaw rate, rad/s
    uint8_t flags;

    bool enabled() const { return (flags & kFlagEnable) != 0; }
    bool estop() const { return (flags & kFlagEstop) != 0; }
    bool arm() const { return (flags & kFlagArm) != 0; }
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

// Encode into `out` (kCmdLen / kTlmLen bytes). Returns bytes written.
size_t encodeCommand(uint8_t* out, uint32_t seq, float vx, float wz,
                     uint8_t flags);
Err decodeCommand(const uint8_t* buf, size_t len, Command& out);

size_t encodeTelemetry(uint8_t* out, const Telemetry& t);
Err decodeTelemetry(const uint8_t* buf, size_t len, Telemetry& out);

}  // namespace linkproto
