// Transaction layer over the Feetech packet codec.
//
// Everything the control loop and the bring-up CLI need, expressed as
// request/response transactions on an abstract Port. The Port is the only
// hardware-touching piece, so this whole class is exercised on the host
// against a scripted FakePort (host/test_scsbus.cpp).
//
// Timing note (docs/wiring.md section "Servo bus"): the bus is 1 Mbaud
// half-duplex with the direction switching done in hardware on the Waveshare
// board -- there is no direction-enable GPIO to drive. That means we can never
// see our own transmission, and it also means a reply can begin arriving
// microseconds after our last stop bit; the Port implementation owns that
// timing, not this class.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "scsbus/packet.h"

namespace scsbus {

// The one hardware seam. An implementation wraps a UART.
class Port {
  public:
    virtual ~Port() = default;
    // Send exactly `len` bytes. Returns false on a driver error.
    virtual bool write(const uint8_t* data, size_t len) = 0;
    // Read up to `cap` bytes, blocking at most `timeout_us`. Returns the
    // count (0 on timeout).
    virtual size_t read(uint8_t* out, size_t cap, uint32_t timeout_us) = 0;
    // Discard anything already buffered (called before each transaction so a
    // late reply to a previous request cannot be mistaken for this one's).
    virtual void flushInput() = 0;
    // Monotonic microseconds, for the reply deadline.
    virtual uint64_t nowUs() const = 0;
};

enum class Status : uint8_t {
    kOk = 0,
    kTimeout,       // no reply inside the deadline
    kBadReply,      // checksum/length garbage that never resynchronised
    kWrongId,       // a reply, but from somebody else
    kTxFail,
    kBadArg,
};

class Bus {
  public:
    // `reply_timeout_us` bounds a single transaction. At 1 Mbaud a 15-byte
    // reply is ~150 us on the wire; the servo's own turnaround (reg 7, default
    // 0) plus scheduling slop dominates. 3 ms is generous and still leaves the
    // 20 ms tick intact if one servo is missing.
    explicit Bus(Port& port, uint32_t reply_timeout_us = 3000)
        : port_(port), reply_timeout_us_(reply_timeout_us) {}

    // -- discovery ---------------------------------------------------------
    bool ping(uint8_t id);

    // -- raw register access ----------------------------------------------
    Status read(uint8_t id, uint8_t addr, uint8_t n, uint8_t* out,
                uint8_t* err = nullptr);
    Status readU8(uint8_t id, uint8_t addr, uint8_t& out);
    Status readU16(uint8_t id, uint8_t addr, uint16_t& out);
    Status write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n);
    Status writeU8(uint8_t id, uint8_t addr, uint8_t value);

    // -- the 50 Hz pair ----------------------------------------------------
    // One SYNC READ of the feedback block for `n` servos. `ok[i]` reports
    // whether servo i answered; the transaction as a whole still returns kOk
    // if at least one did, because a single dead servo must not blind the
    // control loop to the other nine.
    Status syncReadFeedback(const uint8_t* ids, size_t n, Feedback* out,
                            bool* ok);
    // One SYNC WRITE of position targets. Broadcast: no replies, no waiting.
    Status syncWritePositions(const uint8_t* ids, const int32_t* steps,
                              size_t n, uint16_t time_ms = 0,
                              uint16_t speed = 0, uint8_t accel = 0);
    // Same frame, but with a PER-SERVO goal speed (register 46) -- the
    // control loop's command-shaping path streams the speed that reaches each
    // shaped target by the next tick, instead of the max-speed slam a zero
    // speed field means. Nonzero speeds only: 0 is "unlimited" on the wire,
    // which is exactly what this overload exists to avoid.
    Status syncWritePositions(const uint8_t* ids, const int32_t* steps,
                              const uint16_t* speeds, size_t n,
                              uint8_t accel = 0);

    // -- torque ------------------------------------------------------------
    // id == kBroadcastId releases/engages everything in one frame, which is
    // what the link watchdog's RELAX and ESTOP paths use (control-channel.md).
    Status torqueEnable(uint8_t id, bool on);

    // -- bring-up / calibration -------------------------------------------
    Status unlockEeprom(uint8_t id);          // reg 55 <- 0
    Status lockEeprom(uint8_t id);            // reg 55 <- 1
    // Change a servo's ID. Unlocks EEPROM, writes reg 5, re-locks against the
    // NEW id -- getting that order wrong is why hand-rolled ID changes leave
    // servos with an unlocked EEPROM.
    Status setId(uint8_t old_id, uint8_t new_id);
    // "Set Middle Position": latch the current shaft angle as 2048 by writing
    // 128 to the torque-enable register. Leaves torque OFF afterwards, which
    // is the state you want while assembling.
    Status setMiddle(uint8_t id);

    // -- individual reads --------------------------------------------------
    Status readPosition(uint8_t id, int32_t& steps);
    Status readVoltageDeciVolts(uint8_t id, uint8_t& dv);
    Status readStatus(uint8_t id, uint8_t& flags);
    Status readFeedback(uint8_t id, Feedback& out);

  private:
    // Send `tx`, then collect exactly one status packet. `expect_id` of
    // kBroadcastId accepts whatever answers first.
    Status transact(const uint8_t* tx, size_t txlen, uint8_t expect_id,
                    Response& out, uint8_t* store, size_t store_cap);

    Port& port_;
    uint32_t reply_timeout_us_;
    uint8_t tx_[kMaxPacket] = {};  // no heap: one scratch buffer, one owner
    uint8_t rx_[kMaxPacket] = {};
};

}  // namespace scsbus
