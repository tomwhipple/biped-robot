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
};

struct Command {
    uint32_t seq;
    float vx;        // body-frame forward velocity, m/s
    float wz;        // yaw rate, rad/s
    uint8_t flags;

    bool enabled() const { return (flags & kFlagEnable) != 0; }
    bool estop() const { return (flags & kFlagEstop) != 0; }
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
