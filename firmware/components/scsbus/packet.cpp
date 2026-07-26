#include "scsbus/packet.h"

namespace scsbus {
namespace {

// Common envelope: FF FF ID LEN INST <params> CHK. `nparams` excludes the
// instruction byte. Returns the total length or 0 if it will not fit.
size_t frame(uint8_t* out, size_t cap, uint8_t id, Inst inst,
             const uint8_t* params, size_t nparams) {
    const size_t total = 6 + nparams;
    if (cap < total || nparams > 250) return 0;
    out[0] = kHeader;
    out[1] = kHeader;
    out[2] = id;
    out[3] = static_cast<uint8_t>(nparams + 2);
    out[4] = static_cast<uint8_t>(inst);
    for (size_t i = 0; i < nparams; ++i) out[5 + i] = params[i];
    // Checksum covers ID .. last param, i.e. from out[2] for LEN+1 bytes.
    out[total - 1] = checksum(out + 2, nparams + 3);
    return total;
}

}  // namespace

uint8_t checksum(const uint8_t* buf, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += buf[i];
    return static_cast<uint8_t>(~sum & 0xFF);
}

size_t buildPing(uint8_t* out, size_t cap, uint8_t id) {
    return frame(out, cap, id, Inst::kPing, nullptr, 0);
}

size_t buildRead(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                 uint8_t n) {
    const uint8_t p[2] = {addr, n};
    return frame(out, cap, id, Inst::kRead, p, 2);
}

size_t buildWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                  const uint8_t* data, uint8_t n) {
    uint8_t p[64];
    if (static_cast<size_t>(n) + 1 > sizeof p) return 0;
    p[0] = addr;
    for (uint8_t i = 0; i < n; ++i) p[1 + i] = data[i];
    return frame(out, cap, id, Inst::kWrite, p, static_cast<size_t>(n) + 1);
}

size_t buildWrite8(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                   uint8_t value) {
    return buildWrite(out, cap, id, addr, &value, 1);
}

size_t buildWrite16(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                    uint16_t value) {
    uint8_t d[2];
    wrU16(d, value);
    return buildWrite(out, cap, id, addr, d, 2);
}

size_t buildRegWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                     const uint8_t* data, uint8_t n) {
    uint8_t p[64];
    if (static_cast<size_t>(n) + 1 > sizeof p) return 0;
    p[0] = addr;
    for (uint8_t i = 0; i < n; ++i) p[1 + i] = data[i];
    return frame(out, cap, id, Inst::kRegWrite, p, static_cast<size_t>(n) + 1);
}

size_t buildAction(uint8_t* out, size_t cap) {
    return frame(out, cap, kBroadcastId, Inst::kAction, nullptr, 0);
}

size_t buildReset(uint8_t* out, size_t cap, uint8_t id) {
    return frame(out, cap, id, Inst::kReset, nullptr, 0);
}

size_t buildSyncRead(uint8_t* out, size_t cap, uint8_t addr, uint8_t n,
                     const uint8_t* ids, size_t nids) {
    uint8_t p[2 + kMaxServos];
    if (nids == 0 || nids > kMaxServos) return 0;
    p[0] = addr;
    p[1] = n;
    for (size_t i = 0; i < nids; ++i) p[2 + i] = ids[i];
    return frame(out, cap, kBroadcastId, Inst::kSyncRead, p, nids + 2);
}

size_t buildSyncWrite(uint8_t* out, size_t cap, uint8_t addr, uint8_t stride,
                      const uint8_t* ids, const uint8_t* data, size_t nids) {
    uint8_t p[2 + kMaxServos * 9];
    if (nids == 0 || nids > kMaxServos || stride == 0 || stride > 8) return 0;
    p[0] = addr;
    p[1] = stride;
    size_t k = 2;
    for (size_t i = 0; i < nids; ++i) {
        p[k++] = ids[i];
        for (uint8_t j = 0; j < stride; ++j) p[k++] = data[i * stride + j];
    }
    return frame(out, cap, kBroadcastId, Inst::kSyncWrite, p, k);
}

void packPosEx(uint8_t* out7, int32_t pos_steps, uint16_t time_ms,
               uint16_t speed_steps_s, uint8_t accel) {
    out7[0] = accel;
    wrU16(out7 + 1, toSignMag(pos_steps, 15));
    wrU16(out7 + 3, time_ms);
    wrU16(out7 + 5, speed_steps_s);
}

ParseResult parseResponse(const uint8_t* buf, size_t len, Response& out) {
    // Resynchronise: the bus is half-duplex and shared, so a reply can be
    // preceded by an echo tail or line noise. Scan for FF FF.
    size_t i = 0;
    while (i + 1 < len && !(buf[i] == kHeader && buf[i + 1] == kHeader)) ++i;
    if (i + 1 >= len) {
        // Keep the trailing byte if it might be the first header byte.
        out.consumed = (len && buf[len - 1] == kHeader) ? len - 1 : len;
        return len == 0 ? ParseResult::kNeedMore : ParseResult::kNoHeader;
    }
    // A run of FFs: the real header is the last two, so step to them.
    while (i + 2 < len && buf[i + 2] == kHeader) ++i;

    if (i + 4 > len) { out.consumed = i; return ParseResult::kNeedMore; }
    const uint8_t id = buf[i + 2];
    const uint8_t plen = buf[i + 3];
    if (plen < 2) { out.consumed = i + 2; return ParseResult::kBadLength; }
    const size_t total = static_cast<size_t>(plen) + 4;
    if (i + total > len) { out.consumed = i; return ParseResult::kNeedMore; }
    if (checksum(buf + i + 2, static_cast<size_t>(plen) + 1) !=
        buf[i + total - 1]) {
        out.consumed = i + 2;      // skip past this header and try again
        return ParseResult::kBadChecksum;
    }
    out.id = id;
    out.error = buf[i + 4];
    out.params = buf + i + 5;
    out.nparams = static_cast<uint8_t>(plen - 2);
    out.consumed = i + total;
    return ParseResult::kOk;
}

bool decodeFeedback(const uint8_t* p, size_t n, Feedback& out) {
    if (n < kFeedbackLen) return false;
    out.position = signMag(rdU16(p + 0), 15);
    out.speed = signMag(rdU16(p + 2), 15);
    out.load = signMag(rdU16(p + 4), 10);
    out.voltage_dv = p[6];
    out.temperature_c = p[7];
    out.status = p[9];           // reg 65, i.e. offset 9 from reg 56
    out.moving = p[10];
    out.current = signMag(rdU16(p + 13), 15);   // reg 69
    return true;
}

}  // namespace scsbus
