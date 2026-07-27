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

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
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

Err decodeCommand(const uint8_t* buf, size_t len, Command& out) {
    if (len != kCmdLen) return Err::kBadLength;
    if (buf[0] != kMagicCmd[0] || buf[1] != kMagicCmd[1]) return Err::kBadMagic;
    if (buf[2] != kVersion) return Err::kBadVersion;
    if (get16(buf + 12) != crc16Ccitt(buf, 12)) return Err::kBadCrc;
    out.flags = buf[3];
    out.seq = get32(buf + 4);
    out.vx = static_cast<int16_t>(get16(buf + 8)) / 1000.0f;
    out.wz = static_cast<int16_t>(get16(buf + 10)) / 1000.0f;
    return Err::kOk;
}

size_t encodeTelemetry(uint8_t* out, const Telemetry& t) {
    out[0] = kMagicTlm[0];
    out[1] = kMagicTlm[1];
    out[2] = kVersion;
    out[3] = static_cast<uint8_t>(t.state);
    put32(out + 4, t.seq_echo);
    // NOTE: protocol.py uses int(vbat_v * 1000) here -- truncation, not the
    // round-half-even of _milli(). Matching that exactly matters for vectors.
    double mv = static_cast<double>(t.vbat_v) * 1000.0;
    mv = trunc(mv);
    if (mv < 0.0) mv = 0.0;
    if (mv > 65535.0) mv = 65535.0;
    put16(out + 8, static_cast<uint16_t>(mv));
    put16(out + 10, static_cast<uint16_t>(milli(t.up_z)));
    put16(out + 12, static_cast<uint16_t>(milli(t.vx_est)));
    put16(out + 14, static_cast<uint16_t>(milli(t.wz_est)));
    out[16] = t.servo_err;
    out[17] = t.loop_late_pct;
    put16(out + 18, crc16Ccitt(out, 18));
    return kTlmLen;
}

Err decodeTelemetry(const uint8_t* buf, size_t len, Telemetry& out) {
    if (len != kTlmLen) return Err::kBadLength;
    if (buf[0] != kMagicTlm[0] || buf[1] != kMagicTlm[1]) return Err::kBadMagic;
    if (buf[2] != kVersion) return Err::kBadVersion;
    if (get16(buf + 18) != crc16Ccitt(buf, 18)) return Err::kBadCrc;
    if (buf[3] > static_cast<uint8_t>(LinkState::kMaxState)) {
        return Err::kBadState;
    }
    out.state = static_cast<LinkState>(buf[3]);
    out.seq_echo = get32(buf + 4);
    out.vbat_v = get16(buf + 8) / 1000.0f;
    out.up_z = static_cast<int16_t>(get16(buf + 10)) / 1000.0f;
    out.vx_est = static_cast<int16_t>(get16(buf + 12)) / 1000.0f;
    out.wz_est = static_cast<int16_t>(get16(buf + 14)) / 1000.0f;
    out.servo_err = buf[16];
    out.loop_late_pct = buf[17];
    return Err::kOk;
}

}  // namespace linkproto
