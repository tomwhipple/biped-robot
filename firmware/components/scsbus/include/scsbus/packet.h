// Feetech SCS/STS packet codec -- pure, no I/O, host-testable.
//
// Every function writes into a caller-supplied buffer and returns the number
// of bytes written (0 on "won't fit" / bad arguments). Nothing allocates.
//
// Wire format (protocol manual section 1.2):
//   request   FF FF  ID  LEN  INST  PARAM...  CHK
//   response  FF FF  ID  LEN  ERR   PARAM...  CHK
// LEN counts everything after it except the checksum, i.e. nparams + 2.
// CHK = ~(ID + LEN + INST + sum(PARAM)) & 0xFF.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "scsbus/registers.h"

namespace scsbus {

// Worst case we build: a SYNC WRITE of kMaxServos joints x 7 payload bytes
// (accel, pos, time, speed) plus the ID byte each, plus the 8-byte envelope.
constexpr size_t kMaxServos = 16;
constexpr size_t kMaxPacket = 8 + kMaxServos * 8 + 8;

// The 15-byte contiguous feedback block at kRegPresentPosition.
constexpr size_t kFeedbackLen = 15;

uint8_t checksum(const uint8_t* buf, size_t len);

// -- request builders ------------------------------------------------------
// All return the encoded length, or 0 if `cap` is too small.
size_t buildPing(uint8_t* out, size_t cap, uint8_t id);
size_t buildRead(uint8_t* out, size_t cap, uint8_t id, uint8_t addr, uint8_t n);
size_t buildWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                  const uint8_t* data, uint8_t n);
size_t buildWrite8(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                   uint8_t value);
size_t buildWrite16(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                    uint16_t value);
size_t buildRegWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t addr,
                     const uint8_t* data, uint8_t n);
size_t buildAction(uint8_t* out, size_t cap);
size_t buildReset(uint8_t* out, size_t cap, uint8_t id);

// SYNC READ `n` bytes at `addr` from each of `ids`. Servos answer in the order
// listed, one ordinary status packet each.
size_t buildSyncRead(uint8_t* out, size_t cap, uint8_t addr, uint8_t n,
                     const uint8_t* ids, size_t nids);

// SYNC WRITE: `stride` bytes per servo, `data` laid out servo-major
// (data[i*stride .. ] belongs to ids[i]). Broadcast, so nobody replies.
size_t buildSyncWrite(uint8_t* out, size_t cap, uint8_t addr, uint8_t stride,
                      const uint8_t* ids, const uint8_t* data, size_t nids);

// The vendor library's WritePosEx(): one 7-byte write starting at
// kRegAcceleration -- accel, goal position, goal time, goal speed. Packing it
// here (rather than at the call site) keeps the sync-write payload layout in
// exactly one place.
constexpr uint8_t kPosExStride = 7;
void packPosEx(uint8_t* out7, int32_t pos_steps, uint16_t time_ms,
               uint16_t speed_steps_s, uint8_t accel);

// -- response parsing ------------------------------------------------------
enum class ParseResult : uint8_t {
    kOk = 0,
    kNeedMore,     // a prefix of a valid packet; read more bytes and retry
    kBadChecksum,
    kBadLength,
    kNoHeader,     // no FF FF anywhere in the buffer
};

struct Response {
    uint8_t id;
    uint8_t error;        // ErrorBit mask, straight from the wire
    const uint8_t* params;
    uint8_t nparams;
    size_t consumed;      // bytes of the input buffer this packet occupied,
                          // counting any garbage skipped before the header
};

// Find and validate the first status packet in `buf`. On kOk, `out.params`
// points into `buf` (no copy), and the caller should drop `out.consumed`
// bytes. On kBadChecksum / kBadLength, `out.consumed` says how much to drop
// to resynchronise past the bad header.
ParseResult parseResponse(const uint8_t* buf, size_t len, Response& out);

// Little-endian 16-bit field accessors (STS/SMS byte order).
inline uint16_t rdU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
inline void wrU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
}

// -- feedback block decode -------------------------------------------------
struct Feedback {
    int32_t position;      // steps, signed
    int32_t speed;         // steps/s, signed
    int32_t load;          // 0.1 % duty, signed
    uint8_t voltage_dv;    // 0.1 V
    uint8_t temperature_c;
    uint8_t status;        // ErrorBit mask
    uint8_t moving;
    int32_t current;       // 6.5 mA units, signed
};

// Decode the 15-byte block read from kRegPresentPosition (regs 56..70).
bool decodeFeedback(const uint8_t* p, size_t n, Feedback& out);

}  // namespace scsbus
