#include "scsbus/bus.h"

namespace scsbus {

Status Bus::transact(const uint8_t* tx, size_t txlen, uint8_t expect_id,
                     Response& out, uint8_t* store, size_t store_cap) {
    if (txlen == 0) return Status::kBadArg;
    port_.flushInput();
    if (!port_.write(tx, txlen)) return Status::kTxFail;

    const uint64_t deadline = port_.nowUs() + reply_timeout_us_;
    size_t have = 0;
    for (;;) {
        Response r;
        const ParseResult pr = parseResponse(store, have, r);
        if (pr == ParseResult::kOk) {
            if (expect_id != kBroadcastId && r.id != expect_id) {
                // Somebody else's reply (or our own echo on a mis-wired bus).
                // Drop it and keep looking inside the same deadline.
                const size_t drop = r.consumed;
                for (size_t i = drop; i < have; ++i) store[i - drop] = store[i];
                have -= drop;
                if (port_.nowUs() >= deadline) return Status::kWrongId;
                continue;
            }
            out = r;
            return Status::kOk;
        }
        if (pr == ParseResult::kBadChecksum || pr == ParseResult::kBadLength ||
            pr == ParseResult::kNoHeader) {
            const size_t drop = r.consumed ? r.consumed : have;
            for (size_t i = drop; i < have; ++i) store[i - drop] = store[i];
            have -= drop;
        }
        const uint64_t now = port_.nowUs();
        if (now >= deadline) return have ? Status::kBadReply : Status::kTimeout;
        if (have >= store_cap) return Status::kBadReply;
        const size_t got = port_.read(store + have, store_cap - have,
                                      static_cast<uint32_t>(deadline - now));
        if (got == 0 && port_.nowUs() >= deadline) {
            return have ? Status::kBadReply : Status::kTimeout;
        }
        have += got;
    }
}

bool Bus::ping(uint8_t id) {
    const size_t n = buildPing(tx_, sizeof tx_, id);
    Response r;
    return transact(tx_, n, id, r, rx_, sizeof rx_) == Status::kOk;
}

Status Bus::read(uint8_t id, uint8_t addr, uint8_t n, uint8_t* out,
                 uint8_t* err) {
    if (n == 0 || id == kBroadcastId) return Status::kBadArg;
    const size_t txn = buildRead(tx_, sizeof tx_, id, addr, n);
    Response r;
    const Status st = transact(tx_, txn, id, r, rx_, sizeof rx_);
    if (st != Status::kOk) return st;
    if (r.nparams < n) return Status::kBadReply;
    for (uint8_t i = 0; i < n; ++i) out[i] = r.params[i];
    if (err) *err = r.error;
    return Status::kOk;
}

Status Bus::readU8(uint8_t id, uint8_t addr, uint8_t& out) {
    return read(id, addr, 1, &out);
}

Status Bus::readU16(uint8_t id, uint8_t addr, uint16_t& out) {
    uint8_t b[2];
    const Status st = read(id, addr, 2, b);
    if (st == Status::kOk) out = rdU16(b);
    return st;
}

Status Bus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n) {
    const size_t txn = buildWrite(tx_, sizeof tx_, id, addr, data, n);
    if (txn == 0) return Status::kBadArg;
    if (id == kBroadcastId) {
        // Broadcast writes get no reply by definition -- fire and forget.
        return port_.write(tx_, txn) ? Status::kOk : Status::kTxFail;
    }
    Response r;
    return transact(tx_, txn, id, r, rx_, sizeof rx_);
}

Status Bus::writeU8(uint8_t id, uint8_t addr, uint8_t value) {
    return write(id, addr, &value, 1);
}

Status Bus::writeU16(uint8_t id, uint8_t addr, uint16_t value) {
    uint8_t d[2];
    wrU16(d, value);
    return write(id, addr, d, 2);
}

Status Bus::syncReadFeedback(const uint8_t* ids, size_t n, Feedback* out,
                             bool* ok) {
    if (n == 0 || n > kMaxServos) return Status::kBadArg;
    const size_t txn = buildSyncRead(tx_, sizeof tx_, kRegPresentPosition,
                                     kFeedbackLen, ids, n);
    if (txn == 0) return Status::kBadArg;
    for (size_t i = 0; i < n; ++i) ok[i] = false;

    port_.flushInput();
    if (!port_.write(tx_, txn)) return Status::kTxFail;

    // One deadline for the whole burst: the servos answer back-to-back in the
    // order we listed them, so this is ~n * (reply time) not n * timeout.
    const uint64_t deadline =
        port_.nowUs() + reply_timeout_us_ * static_cast<uint32_t>(n);
    size_t have = 0, answered = 0;
    while (answered < n) {
        Response r;
        const ParseResult pr = parseResponse(rx_, have, r);
        if (pr == ParseResult::kOk) {
            for (size_t i = 0; i < n; ++i) {
                if (ids[i] == r.id && !ok[i]) {
                    if (decodeFeedback(r.params, r.nparams, out[i])) {
                        // The response header's ERROR byte and reg 65 carry
                        // the same bits; OR them so a fault reported only in
                        // the header is not lost.
                        out[i].status |= r.error;
                        ok[i] = true;
                        ++answered;
                    }
                    break;
                }
            }
        }
        if (pr != ParseResult::kNeedMore) {
            const size_t drop = r.consumed ? r.consumed : have;
            for (size_t i = drop; i < have; ++i) rx_[i - drop] = rx_[i];
            have -= drop;
            if (pr == ParseResult::kOk) continue;
        }
        const uint64_t now = port_.nowUs();
        if (now >= deadline) break;
        if (have >= sizeof rx_) break;
        // Ask for exactly the bytes still outstanding, not the whole buffer.
        // uart_read_bytes() returns when it has the requested count OR the
        // timeout expires -- so requesting more than will ever arrive turns
        // every read into a full-timeout sleep. That was the whole of the
        // 29.9 ms measured in a 20 ms tick on 2026-07-26: the ten replies
        // landed in ~1.4 ms and we then blocked on an already-full buffer.
        const size_t outstanding = (n - answered) * kSyncReplyLen;
        size_t want = outstanding > have ? outstanding - have : 1;
        const size_t space = sizeof rx_ - have;
        if (want > space) want = space;
        have += port_.read(rx_ + have, want,
                           static_cast<uint32_t>(deadline - now));
    }
    return answered ? Status::kOk : Status::kTimeout;
}

Status Bus::syncWritePositions(const uint8_t* ids, const int32_t* steps,
                               size_t n, uint16_t time_ms, uint16_t speed,
                               uint8_t accel) {
    if (n == 0 || n > kMaxServos) return Status::kBadArg;
    uint8_t payload[kMaxServos * kPosExStride];
    for (size_t i = 0; i < n; ++i) {
        packPosEx(payload + i * kPosExStride, steps[i], time_ms, speed, accel);
    }
    const size_t txn = buildSyncWrite(tx_, sizeof tx_, kRegAcceleration,
                                      kPosExStride, ids, payload, n);
    if (txn == 0) return Status::kBadArg;
    return port_.write(tx_, txn) ? Status::kOk : Status::kTxFail;
}

Status Bus::syncWritePositions(const uint8_t* ids, const int32_t* steps,
                               const uint16_t* speeds, size_t n,
                               uint8_t accel) {
    if (n == 0 || n > kMaxServos || !speeds) return Status::kBadArg;
    uint8_t payload[kMaxServos * kPosExStride];
    for (size_t i = 0; i < n; ++i) {
        packPosEx(payload + i * kPosExStride, steps[i], /*time_ms=*/0,
                  speeds[i], accel);
    }
    const size_t txn = buildSyncWrite(tx_, sizeof tx_, kRegAcceleration,
                                      kPosExStride, ids, payload, n);
    if (txn == 0) return Status::kBadArg;
    return port_.write(tx_, txn) ? Status::kOk : Status::kTxFail;
}

Status Bus::torqueEnable(uint8_t id, bool on) {
    return writeU8(id, kRegTorqueEnable, on ? 1 : 0);
}

Status Bus::unlockEeprom(uint8_t id) { return writeU8(id, kRegEepromLock, 0); }
Status Bus::lockEeprom(uint8_t id) { return writeU8(id, kRegEepromLock, 1); }

Status Bus::setId(uint8_t old_id, uint8_t new_id) {
    if (new_id > kMaxId) return Status::kBadArg;
    Status st = unlockEeprom(old_id);
    if (st != Status::kOk) return st;
    st = writeU8(old_id, kRegId, new_id);
    if (st != Status::kOk) {
        // Leaving the EEPROM unlocked is worse than a failed rename; try to
        // put the lock back on whichever ID still answers.
        lockEeprom(old_id);
        return st;
    }
    return lockEeprom(new_id);      // it answers to the NEW id from here on
}

Status Bus::setMiddle(uint8_t id) {
    return writeU8(id, kRegTorqueEnable, kTorqueCalibrateMiddle);
}

Status Bus::setPositionCorrection(uint8_t id, int32_t steps) {
    if (steps < -2047 || steps > 2047) return Status::kBadArg;
    Status st = unlockEeprom(id);
    if (st != Status::kOk) return st;
    st = writeU16(id, kRegPosCorrection, toSignMag(steps, 11));
    const Status lock = lockEeprom(id);
    return st != Status::kOk ? st : lock;
}

Status Bus::readPosition(uint8_t id, int32_t& steps) {
    uint16_t raw = 0;
    const Status st = readU16(id, kRegPresentPosition, raw);
    if (st == Status::kOk) steps = signMag(raw, 15);
    return st;
}

Status Bus::readVoltageDeciVolts(uint8_t id, uint8_t& dv) {
    return readU8(id, kRegPresentVoltage, dv);
}

Status Bus::readStatus(uint8_t id, uint8_t& flags) {
    return readU8(id, kRegStatus, flags);
}

Status Bus::readFeedback(uint8_t id, Feedback& out) {
    uint8_t b[kFeedbackLen];
    uint8_t err = 0;
    const Status st = read(id, kRegPresentPosition, kFeedbackLen, b, &err);
    if (st != Status::kOk) return st;
    if (!decodeFeedback(b, sizeof b, out)) return Status::kBadReply;
    out.status |= err;
    return Status::kOk;
}

}  // namespace scsbus
