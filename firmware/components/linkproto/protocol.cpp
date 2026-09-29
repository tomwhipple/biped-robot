#include "linkproto/protocol.h"

#include <math.h>

namespace linkproto {
namespace {

// protocol.py's _milli(): int(round(x * 1000.0)) saturated into int16.
//
// Python's round() is round-half-to-EVEN, and so is C's nearbyint() under the
// default FE_TONEAREST rounding mode -- lround() would be half-away-from-zero
// and would disagree on exact .0005 inputs. The golden vectors include such a
// case on purpose.
int16_t milli(float x) {
    const double v = nearbyint(static_cast<double>(x) * 1000.0);
    if (v <= -32768.0) return -32768;
    if (v >= 32767.0) return 32767;
    return static_cast<int16_t>(v);
}

inline void put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
inline uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint32_t get32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
inline void put64(uint8_t* p, uint64_t v) {
    for (size_t i = 0; i < 8; ++i) {
        p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
    }
}
inline uint64_t get64(const uint8_t* p) {
    uint64_t v = 0;
    for (size_t i = 0; i < 8; ++i) v |= static_cast<uint64_t>(p[i]) << (8 * i);
    return v;
}

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ (data[i] << 8));
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void clampToEnvelope(float vx, float wz, float& out_vx, float& out_wz) {
    if (wz > kWMax) wz = kWMax;
    if (wz < -kWMax) wz = -kWMax;
    out_wz = wz;
    if (fabsf(vx) < kVMinWalk) {
        // The untrained band. Snap to a stand rather than interpolate into it;
        // a turn-in-place (vx=0, wz!=0) IS trained, so wz survives.
        out_vx = 0.0f;
        return;
    }
    if (vx > kVMax) vx = kVMax;
    if (vx < -kVMax) vx = -kVMax;
    out_vx = vx;
}

void clampExtToEnvelope(Command& pkt) {
    if (pkt.vy > kVyMax) pkt.vy = kVyMax;
    if (pkt.vy < -kVyMax) pkt.vy = -kVyMax;
    if (pkt.crouch > 1.0f) pkt.crouch = 1.0f;
    if (pkt.crouch < kCrouchMin) pkt.crouch = kCrouchMin;
    if (pkt.lift > 1.0f) pkt.lift = 1.0f;
    if (pkt.lift < -1.0f) pkt.lift = -1.0f;
    if (pkt.foot_dx > kFootDMax) pkt.foot_dx = kFootDMax;
    if (pkt.foot_dx < -kFootDMax) pkt.foot_dx = -kFootDMax;
    if (pkt.foot_dz > kFootDMax) pkt.foot_dz = kFootDMax;
    if (pkt.foot_dz < -kFootDMax) pkt.foot_dz = -kFootDMax;
}

uint8_t packDiag(bool run, bool cal_ok, ArmResult result) {
    uint8_t d = 0;
    // cppcheck-suppress badBitmaskCheck -- building a bitmask from zero is the
    // idiom; the `d |` form keeps each flag's contribution explicit.
    if (run) d = static_cast<uint8_t>(d | kDiagRun);
    if (cal_ok) d = static_cast<uint8_t>(d | kDiagCalOk);
    return static_cast<uint8_t>(
        d | ((static_cast<uint8_t>(result) << kDiagArmShift) & kDiagArmMask));
}

ArmResult diagArmResult(uint8_t diag) {
    return static_cast<ArmResult>((diag & kDiagArmMask) >> kDiagArmShift);
}

uint8_t telemetryDiag(const Telemetry& t) {
    // Only kBench frames carry it; anywhere else seq_echo is a real sequence
    // number and its low byte means nothing at all.
    return t.state == LinkState::kBench
               ? static_cast<uint8_t>(t.seq_echo & 0xFFu)
               : 0u;
}

const char* diagReason(uint8_t diag) {
    switch (diagArmResult(diag)) {
        case ArmResult::kNone:
            return (diag & kDiagCalOk)
                       ? "no arm requested yet -- calibrated, ready to arm"
                       : "no arm requested yet -- and there is NO calibration "
                         "in NVS, so arming will be REFUSED";
        case ArmResult::kAccepted:
            return (diag & kDiagRun) ? "armed -- the control loop is running"
                                     : "disarmed on request -- press arm to run";
        case ArmResult::kRefusedNoCal:
            return "arm REFUSED -- no as-built calibration in NVS (run `cal`)";
        case ArmResult::kDisarmedFall:
            return "disarmed -- FALL latch tripped; re-arm deliberately (ARM "
                   "edge or run)";
        case ArmResult::kDisarmedHome:
            return "disarmed -- servos reset to the standing pose (hips rolled "
                   "out/back for play) and torque RELEASED; arm to walk";
        case ArmResult::kHomeNoCal:
            return "servo reset REFUSED -- no as-built calibration in NVS, so "
                   "\"zero\" is not a stand (run `cal`)";
        case ArmResult::kHomeLowBatt:
            return "servo reset REFUSED -- pack under-voltage latch; swap the "
                   "pack, then `batt reset`";
        case ArmResult::kHomeBusFailed:
            return "servo reset FAILED -- the servo bus did not accept it "
                   "(check pack, wiring, `scan`)";
        case ArmResult::kHomePending:
            return "servo reset written -- joints slewing to the stand, "
                   "readback pending";
        case ArmResult::kHomeNotReached:
            return "servo reset FAILED -- readback found joints OFF their "
                   "zeros (see the tether for which; `scan`)";
        case ArmResult::kRefusedGains:
            return "arm REFUSED -- a servo's position-loop gains (reg 21/22) "
                   "are not the expected values, or did not read back (run "
                   "`gains`)";
    }
    // A refusal reason this client is too old to name. Say so rather than
    // guess: the enum is append-only, so an unknown value is a NEWER robot.
    return "arm REFUSED -- reason unknown to this client (newer firmware)";
}

size_t encodeCommand(uint8_t* out, uint32_t seq, float vx, float wz,
                     uint8_t flags) {
    out[0] = kMagicCmd[0];
    out[1] = kMagicCmd[1];
    out[2] = kVersion;
    out[3] = flags;
    put32(out + 4, seq);
    put16(out + 8, static_cast<uint16_t>(milli(vx)));
    put16(out + 10, static_cast<uint16_t>(milli(wz)));
    put16(out + 12, crc16Ccitt(out, 12));
    return kCmdLen;
}

size_t encodeCommandExt(uint8_t* out, const Command& cmd) {
    out[0] = kMagicCmd[0];
    out[1] = kMagicCmd[1];
    out[2] = kVersion;
    out[3] = cmd.flags;
    put32(out + 4, cmd.seq);
    put16(out + 8, static_cast<uint16_t>(milli(cmd.vx)));
    put16(out + 10, static_cast<uint16_t>(milli(cmd.wz)));
    put16(out + 12, static_cast<uint16_t>(milli(cmd.vy)));
    put16(out + 14, static_cast<uint16_t>(milli(cmd.crouch)));
    put16(out + 16, static_cast<uint16_t>(milli(cmd.lift)));
    put16(out + 18, static_cast<uint16_t>(milli(cmd.foot_dx)));
    put16(out + 20, static_cast<uint16_t>(milli(cmd.foot_dz)));
    put16(out + 22, crc16Ccitt(out, 22));
    return kCmdLenExt;
}

Err decodeCommand(const uint8_t* buf, size_t len, Command& out) {
    if (len != kCmdLen && len != kCmdLenExt) return Err::kBadLength;
    if (buf[0] != kMagicCmd[0] || buf[1] != kMagicCmd[1]) return Err::kBadMagic;
    if (buf[2] != kVersion) return Err::kBadVersion;
    if (get16(buf + len - 2) != crc16Ccitt(buf, len - 2)) return Err::kBadCrc;
    out.flags = buf[3];
    out.seq = get32(buf + 4);
    out.vx = static_cast<int16_t>(get16(buf + 8)) / 1000.0f;
    out.wz = static_cast<int16_t>(get16(buf + 10)) / 1000.0f;
    if (len == kCmdLenExt) {
        out.vy = static_cast<int16_t>(get16(buf + 12)) / 1000.0f;
        out.crouch = static_cast<int16_t>(get16(buf + 14)) / 1000.0f;
        out.lift = static_cast<int16_t>(get16(buf + 16)) / 1000.0f;
        out.foot_dx = static_cast<int16_t>(get16(buf + 18)) / 1000.0f;
        out.foot_dz = static_cast<int16_t>(get16(buf + 20)) / 1000.0f;
    } else {
        out.vy = 0.0f;
        out.crouch = 1.0f;
        out.lift = 0.0f;
        out.foot_dx = 0.0f;
        out.foot_dz = 0.0f;
    }
    return Err::kOk;
}

size_t encodeTelemetry(uint8_t* out, const Telemetry& t) {
    out[0] = kMagicTlm[0];
    out[1] = kMagicTlm[1];
    out[2] = kVersion;
    out[3] = static_cast<uint8_t>(t.state);
    put32(out + 4, t.seq_echo);
    // protocol.py's _milli_u16(): round-half-even into uint16, the same
    // rounding milli() uses. It ROUNDS rather than truncates because vbat_v
    // arrives here as a float32 of an integer millivolt count (link_task.cpp:
    // vbat_mv / 1000.0f), and float32(11400/1000) is 11.399999619 V --
    // truncating that reported 11399 mV for every 11.4 V pack and disagreed
    // with the Python reference's double (issue #54).
    double mv = nearbyint(static_cast<double>(t.vbat_v) * 1000.0);
    if (mv < 0.0) mv = 0.0;
    if (mv > 65535.0) mv = 65535.0;
    put16(out + 8, static_cast<uint16_t>(mv));
    put16(out + 10, static_cast<uint16_t>(milli(t.up_z)));
    put16(out + 12, static_cast<uint16_t>(milli(t.vx_est)));
    put16(out + 14, static_cast<uint16_t>(milli(t.wz_est)));
    put32(out + 16, t.servo_err);
    out[20] = t.loop_late_pct;
    put64(out + kTlmTimeOff, t.t_us);
    // Blocks in a FIXED order and of fixed size, so the length alone still
    // names the layout: [base 21][t_us 8][joints 34?][att 4?][crc 2].
    size_t at = kTlmTimeOff + 8;
    if (t.n_joints == kNumJoints) {
        for (size_t i = 0; i < kNumJoints; ++i) {
            put16(out + at + 2 * i, static_cast<uint16_t>(milli(t.joints[i])));
        }
        at += 2 * kNumJoints;
    }
    if (t.have_att) {
        put16(out + at, static_cast<uint16_t>(milli(t.up_x)));
        put16(out + at + 2, static_cast<uint16_t>(milli(t.up_y)));
        at += kTlmAttBytes;
    }
    put16(out + at, crc16Ccitt(out, at));
    return at + 2;
}

Err decodeTelemetry(const uint8_t* buf, size_t len, Telemetry& out) {
    // Length selects the layout, exactly as protocol.py.
    const bool has_joints = (len == kTlmLenExt || len == kTlmLenExtAtt);
    const bool has_att = (len == kTlmLenAtt || len == kTlmLenExtAtt);
    if (len != kTlmLen && !has_joints && !has_att) return Err::kBadLength;
    if (buf[0] != kMagicTlm[0] || buf[1] != kMagicTlm[1]) return Err::kBadMagic;
    if (buf[2] != kVersion) return Err::kBadVersion;
    const size_t body = len - 2;
    if (get16(buf + body) != crc16Ccitt(buf, body)) return Err::kBadCrc;
    if (buf[3] > static_cast<uint8_t>(LinkState::kMaxState)) {
        return Err::kBadState;
    }
    out.state = static_cast<LinkState>(buf[3]);
    out.seq_echo = get32(buf + 4);
    out.vbat_v = get16(buf + 8) / 1000.0f;
    out.up_z = static_cast<int16_t>(get16(buf + 10)) / 1000.0f;
    out.vx_est = static_cast<int16_t>(get16(buf + 12)) / 1000.0f;
    out.wz_est = static_cast<int16_t>(get16(buf + 14)) / 1000.0f;
    out.servo_err = get32(buf + 16);
    out.loop_late_pct = buf[20];
    size_t at = kTlmTimeOff;
    out.t_us = get64(buf + at);
    at += 8;
    out.n_joints = 0;
    for (size_t i = 0; i < kNumJoints; ++i) out.joints[i] = 0.0f;
    out.have_att = 0;
    out.up_x = 0.0f;
    out.up_y = 0.0f;
    if (has_joints) {
        out.n_joints = static_cast<uint8_t>(kNumJoints);
        for (size_t i = 0; i < kNumJoints; ++i) {
            out.joints[i] =
                static_cast<int16_t>(get16(buf + at + 2 * i)) / 1000.0f;
        }
        at += 2 * kNumJoints;
    }
    if (has_att) {
        out.have_att = 1;
        out.up_x = static_cast<int16_t>(get16(buf + at)) / 1000.0f;
        out.up_y = static_cast<int16_t>(get16(buf + at + 2)) / 1000.0f;
    }
    return Err::kOk;
}

}  // namespace linkproto
