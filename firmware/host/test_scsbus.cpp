// Feetech SCS/STS packet codec + transaction layer, on the host.
//
// The golden packets are the WORKED EXAMPLES from Feetech's own communication
// protocol manual (the PDF Waveshare links from the ST3215 wiki page), so this
// is diffed against the vendor's bytes rather than against our own encoder.
// The Bus tests drive a scripted FakePort -- no hardware, no serial port.
#include <deque>
#include <vector>

#include "../main/servo_gains.h"
#include "scsbus/bus.h"
#include "scsbus/gains.h"
#include "scsbus/packet.h"
#include "test_util.h"

using namespace scsbus;

namespace {

// -- a Port that answers from a script -------------------------------------
class FakePort : public Port {
  public:
    std::vector<uint8_t> tx;
    // Replies are queued as whole packets, and read() hands back one packet at
    // a time -- a real bus does not deliver the next servo's answer before we
    // have asked for it, and a port that dumped the whole script in one read
    // would hide exactly the resynchronisation bugs this is here to catch.
    std::deque<std::vector<uint8_t>> rx;
    uint64_t t_us = 0;
    int flushes = 0;
    bool fail_write = false;

    bool write(const uint8_t* d, size_t n) override {
        if (fail_write) return false;
        tx.insert(tx.end(), d, d + n);
        t_us += n * 10;                 // ~10 us/byte at 1 Mbaud
        return true;
    }
    size_t read(uint8_t* o, size_t cap, uint32_t timeout_us) override {
        if (rx.empty()) {
            t_us += timeout_us;         // the caller's deadline expires here
            return 0;
        }
        std::vector<uint8_t>& head = rx.front();
        const size_t k = cap < head.size() ? cap : head.size();
        for (size_t i = 0; i < k; ++i) o[i] = head[i];
        head.erase(head.begin(), head.begin() + static_cast<long>(k));
        if (head.empty()) rx.pop_front();
        t_us += k * 10;
        return k;
    }
    // NOTE: deliberately does not drop the script. The real port flushes the
    // UART FIFO; here the queued reply IS the servo's answer to the write that
    // is about to happen.
    void flushInput() override { ++flushes; }
    uint64_t nowUs() const override { return t_us; }

    void queueStatus(uint8_t id, uint8_t err, std::vector<uint8_t> params) {
        std::vector<uint8_t> p;
        p.push_back(id);
        p.push_back(static_cast<uint8_t>(params.size() + 2));
        p.push_back(err);
        for (uint8_t x : params) p.push_back(x);
        std::vector<uint8_t> pkt{0xFF, 0xFF};
        pkt.insert(pkt.end(), p.begin(), p.end());
        pkt.push_back(checksum(p.data(), p.size()));
        rx.push_back(pkt);
    }
};

// -- vendor manual worked examples ----------------------------------------
void testVendorPackets() {
    uint8_t buf[kMaxPacket];

    const uint8_t ping[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
    CHECK_EQ(buildPing(buf, sizeof buf, 1), sizeof ping);
    CHECK_BYTES(buf, ping, sizeof ping);

    // READ 2 bytes at 0x38 (present position) from ID 1.
    const uint8_t rd[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
    CHECK_EQ(buildRead(buf, sizeof buf, 1, kRegPresentPosition, 2), sizeof rd);
    CHECK_BYTES(buf, rd, sizeof rd);

    // Broadcast "set ID to 1" (reg 5).
    const uint8_t sid[] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x05, 0x01, 0xF4};
    CHECK_EQ(buildWrite8(buf, sizeof buf, kBroadcastId, kRegId, 1), sizeof sid);
    CHECK_BYTES(buf, sid, sizeof sid);

    // Goal position 2048, time 0, speed 1000 -- a 6-byte write at reg 42.
    const uint8_t gp[] = {0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A, 0x00, 0x08,
                          0x00, 0x00, 0xE8, 0x03, 0xD5};
    uint8_t payload[6];
    wrU16(payload + 0, 2048);
    wrU16(payload + 2, 0);
    wrU16(payload + 4, 1000);
    CHECK_EQ(buildWrite(buf, sizeof buf, 1, kRegGoalPosition, payload, 6),
             sizeof gp);
    CHECK_BYTES(buf, gp, sizeof gp);

    const uint8_t act[] = {0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA};
    CHECK_EQ(buildAction(buf, sizeof buf), sizeof act);
    CHECK_BYTES(buf, act, sizeof act);

    // SYNC READ 8 bytes at 0x38 from IDs 1 and 2.
    const uint8_t sr[] = {0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38,
                          0x08, 0x01, 0x02, 0x36};
    const uint8_t ids2[] = {1, 2};
    CHECK_EQ(buildSyncRead(buf, sizeof buf, kRegPresentPosition, 8, ids2, 2),
             sizeof sr);
    CHECK_BYTES(buf, sr, sizeof sr);
}

void testSyncWriteShape() {
    // Length must be (L+1)*N + 4 per the manual; check the framing arithmetic
    // rather than a hand-copied byte string.
    uint8_t buf[kMaxPacket];
    const uint8_t ids[] = {1, 2, 3};
    const int32_t pos[] = {2048, 1024, 3072};
    uint8_t payload[3 * kPosExStride];
    for (int i = 0; i < 3; ++i) {
        packPosEx(payload + i * kPosExStride, pos[i], 0, 0, 0);
    }
    const size_t n = buildSyncWrite(buf, sizeof buf, kRegAcceleration,
                                    kPosExStride, ids, payload, 3);
    CHECK_EQ(n, 4 + (kPosExStride + 1) * 3 + 4);
    CHECK_EQ(buf[2], kBroadcastId);
    CHECK_EQ(buf[3], (kPosExStride + 1) * 3 + 4);
    CHECK_EQ(buf[4], static_cast<uint8_t>(Inst::kSyncWrite));
    CHECK_EQ(buf[5], kRegAcceleration);
    CHECK_EQ(buf[6], kPosExStride);
    CHECK_EQ(buf[7], 1);
    CHECK_EQ(buf[8], 0);                        // accel
    CHECK_EQ(rdU16(buf + 9), 2048);             // little-endian goal position
    CHECK_EQ(buf[n - 1], checksum(buf + 2, n - 3));
}

void testSignMagnitude() {
    // Feetech is sign-magnitude, NOT two's complement. Position/speed use bit
    // 15, load bit 10, the EEPROM position correction bit 11.
    CHECK_EQ(signMag(0x0800, 15), 2048);
    CHECK_EQ(signMag(0x8800, 15), -2048);
    CHECK_EQ(signMag(0x0000, 15), 0);
    CHECK_EQ(signMag(0x8000, 15), 0);           // "negative zero"
    CHECK_EQ(signMag(0x0064, 10), 100);
    CHECK_EQ(signMag(0x0464, 10), -100);
    CHECK_EQ(toSignMag(-2048, 15), 0x8800);
    CHECK_EQ(toSignMag(2048, 15), 0x0800);
    CHECK_EQ(toSignMag(-100, 11), 0x0864);
    for (int32_t v = -2000; v <= 2000; v += 137) {
        CHECK_EQ(signMag(toSignMag(v, 15), 15), v);
    }
}

void testParseResponse() {
    // A clean status packet: ID 5, no error, two parameter bytes.
    uint8_t body[] = {0x05, 0x04, 0x00, 0x34, 0x12};
    uint8_t pkt[8] = {0xFF, 0xFF, 0, 0, 0, 0, 0, 0};
    memcpy(pkt + 2, body, sizeof body);
    pkt[7] = checksum(body, sizeof body);
    Response r{};
    CHECK_EQ(static_cast<int>(parseResponse(pkt, sizeof pkt, r)),
             static_cast<int>(ParseResult::kOk));
    CHECK_EQ(r.id, 5);
    CHECK_EQ(r.error, 0);
    CHECK_EQ(r.nparams, 2);
    CHECK_EQ(rdU16(r.params), 0x1234);
    CHECK_EQ(r.consumed, sizeof pkt);

    // A partial packet asks for more rather than guessing.
    CHECK_EQ(static_cast<int>(parseResponse(pkt, 5, r)),
             static_cast<int>(ParseResult::kNeedMore));

    // Leading garbage is skipped (half-duplex bus: echoes and noise happen).
    uint8_t noisy[11] = {0x00, 0x7F, 0xAA};
    memcpy(noisy + 3, pkt, sizeof pkt);
    CHECK_EQ(static_cast<int>(parseResponse(noisy, sizeof noisy, r)),
             static_cast<int>(ParseResult::kOk));
    CHECK_EQ(r.id, 5);
    CHECK_EQ(r.consumed, sizeof noisy);

    // A run of FFs before the packet must not be mistaken for the header.
    uint8_t runs[12] = {0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(runs + 4, pkt + 2, 6);
    CHECK_EQ(static_cast<int>(parseResponse(runs, 10, r)),
             static_cast<int>(ParseResult::kOk));
    CHECK_EQ(r.id, 5);

    // A corrupted checksum is reported, and consumed says how to resync.
    uint8_t bad[8];
    memcpy(bad, pkt, sizeof pkt);
    bad[7] ^= 0xFF;
    CHECK_EQ(static_cast<int>(parseResponse(bad, sizeof bad, r)),
             static_cast<int>(ParseResult::kBadChecksum));
    CHECK(r.consumed > 0);
}

void testDecodeFeedback() {
    // regs 56..70: pos 1234, speed -300, load 55, 11.7 V, 41 C, overload flag,
    // moving, current 200.
    uint8_t b[kFeedbackLen] = {};
    wrU16(b + 0, toSignMag(1234, 15));
    wrU16(b + 2, toSignMag(-300, 15));
    wrU16(b + 4, toSignMag(55, 10));
    b[6] = 117;
    b[7] = 41;
    b[9] = kErrOverload;
    b[10] = 1;
    wrU16(b + 13, toSignMag(200, 15));
    Feedback f{};
    CHECK(decodeFeedback(b, sizeof b, f));
    CHECK_EQ(f.position, 1234);
    CHECK_EQ(f.speed, -300);
    CHECK_EQ(f.load, 55);
    CHECK_NEAR(f.voltage_dv * kVoltsPerLsb, 11.7, 1e-3);
    CHECK_EQ(f.temperature_c, 41);
    CHECK_EQ(f.status, kErrOverload);
    CHECK_EQ(f.moving, 1);
    CHECK_EQ(f.current, 200);
    CHECK(!decodeFeedback(b, 4, f));
}

// -- transaction layer -----------------------------------------------------
void testBusPingAndRead() {
    FakePort p;
    Bus bus(p);
    p.queueStatus(3, 0, {});
    CHECK(bus.ping(3));
    CHECK_EQ(p.tx.size(), 6u);
    CHECK_EQ(p.tx[2], 3);

    // No reply at all -> ping fails and does not hang.
    p.tx.clear();
    CHECK(!bus.ping(4));

    // Present position 1500.
    p.queueStatus(1, 0, {static_cast<uint8_t>(toSignMag(1500, 15) & 0xFF),
                         static_cast<uint8_t>(toSignMag(1500, 15) >> 8)});
    int32_t pos = 0;
    CHECK_EQ(static_cast<int>(bus.readPosition(1, pos)),
             static_cast<int>(Status::kOk));
    CHECK_EQ(pos, 1500);

    // A reply from the wrong servo is not accepted as ours.
    p.queueStatus(9, 0, {0x00, 0x00});
    CHECK(bus.readPosition(1, pos) != Status::kOk);
}

void testBusTorqueBroadcast() {
    // The link watchdog's RELAX/ESTOP path: one broadcast frame, no replies,
    // no waiting (control-channel.md: "RELAX/ESTOP -> torque-enable register
    // off on IDs 1-8").
    FakePort p;
    Bus bus(p);
    CHECK_EQ(static_cast<int>(bus.torqueEnable(kBroadcastId, false)),
             static_cast<int>(Status::kOk));
    CHECK_EQ(p.tx.size(), 8u);
    CHECK_EQ(p.tx[2], kBroadcastId);
    CHECK_EQ(p.tx[5], kRegTorqueEnable);
    CHECK_EQ(p.tx[6], 0);
    CHECK_EQ(p.rx.size(), 0u);
}

void testBusSyncWritePerServoSpeed() {
    // The command-shaping path: one broadcast frame, per-servo goal speed in
    // the reg-46 slot, goal time zero, no replies.
    FakePort p;
    Bus bus(p);
    const uint8_t ids[] = {1, 2, 3};
    const int32_t pos[] = {2048, 1024, 3072};
    const uint16_t spd[] = {150, 3400, 50};
    CHECK_EQ(static_cast<int>(bus.syncWritePositions(ids, pos, spd, 3)),
             static_cast<int>(Status::kOk));
    CHECK_EQ(p.tx.size(), 4u + (kPosExStride + 1) * 3 + 4u);
    CHECK_EQ(p.tx[2], kBroadcastId);
    CHECK_EQ(p.tx[4], static_cast<uint8_t>(Inst::kSyncWrite));
    CHECK_EQ(p.tx[5], kRegAcceleration);
    CHECK_EQ(p.tx[6], kPosExStride);
    for (int i = 0; i < 3; ++i) {
        const uint8_t* e = p.tx.data() + 7 + i * (kPosExStride + 1);
        CHECK_EQ(e[0], ids[i]);
        CHECK_EQ(e[1], 0);                          // accel: not commanded
        CHECK_EQ(rdU16(e + 2), toSignMag(pos[i], 15));
        CHECK_EQ(rdU16(e + 4), 0);                  // goal time unused
        CHECK_EQ(rdU16(e + 6), spd[i]);             // the per-servo speed
    }
    CHECK_EQ(p.rx.size(), 0u);                      // broadcast: no replies

    // A null speed array is a caller bug, refused before the wire.
    CHECK_EQ(static_cast<int>(bus.syncWritePositions(ids, pos, nullptr, 3)),
             static_cast<int>(Status::kBadArg));
}

void testBusSetIdLocksTheNewId() {
    // Unlock old, write reg 5, lock NEW -- getting the last step wrong leaves
    // a servo with an unlocked EEPROM that nothing will notice until it
    // forgets its calibration.
    FakePort p;
    Bus bus(p);
    p.queueStatus(2, 0, {});     // reply to unlock
    p.queueStatus(2, 0, {});     // reply to the ID write
    p.queueStatus(7, 0, {});     // reply to lock, from the NEW id
    CHECK_EQ(static_cast<int>(bus.setId(2, 7)), static_cast<int>(Status::kOk));
    // three 8-byte writes
    CHECK_EQ(p.tx.size(), 24u);
    CHECK_EQ(p.tx[2], 2);   CHECK_EQ(p.tx[5], kRegEepromLock); CHECK_EQ(p.tx[6], 0);
    CHECK_EQ(p.tx[10], 2);  CHECK_EQ(p.tx[13], kRegId);        CHECK_EQ(p.tx[14], 7);
    CHECK_EQ(p.tx[18], 7);  CHECK_EQ(p.tx[21], kRegEepromLock);CHECK_EQ(p.tx[22], 1);

    CHECK_EQ(static_cast<int>(bus.setId(1, 254)),
             static_cast<int>(Status::kBadArg));
}

void testBusSetMiddle() {
    FakePort p;
    Bus bus(p);
    p.queueStatus(4, 0, {});
    CHECK_EQ(static_cast<int>(bus.setMiddle(4)), static_cast<int>(Status::kOk));
    CHECK_EQ(p.tx[5], kRegTorqueEnable);
    CHECK_EQ(p.tx[6], kTorqueCalibrateMiddle);
}

void testBusSyncReadFeedback() {
    FakePort p;
    Bus bus(p);
    const uint8_t ids[3] = {1, 2, 3};
    // Servo 2 is unplugged: it never answers. The other two must still be
    // usable -- one dead joint cannot blind the control loop.
    for (uint8_t id : {uint8_t{1}, uint8_t{3}}) {
        std::vector<uint8_t> b(kFeedbackLen, 0);
        const uint16_t pos = toSignMag(100 * static_cast<int32_t>(id), 15);
        b[0] = static_cast<uint8_t>(pos & 0xFF);
        b[1] = static_cast<uint8_t>(pos >> 8);
        b[6] = 118;
        p.queueStatus(id, 0, b);
    }
    Feedback fb[3];
    bool ok[3];
    CHECK_EQ(static_cast<int>(bus.syncReadFeedback(ids, 3, fb, ok)),
             static_cast<int>(Status::kOk));
    CHECK(ok[0] && !ok[1] && ok[2]);
    CHECK_EQ(fb[0].position, 100);
    CHECK_EQ(fb[2].position, 300);
    CHECK_EQ(fb[0].voltage_dv, 118);
    // The request was one broadcast SYNC READ, not three reads.
    CHECK_EQ(p.tx[2], kBroadcastId);
    CHECK_EQ(p.tx[4], static_cast<uint8_t>(Inst::kSyncRead));

    // Nobody home at all: reported, not hung.
    Bus bus2(p);
    CHECK_EQ(static_cast<int>(bus2.syncReadFeedback(ids, 3, fb, ok)),
             static_cast<int>(Status::kTimeout));
}

void testBusErrorFlagsSurface() {
    FakePort p;
    Bus bus(p);
    std::vector<uint8_t> b(kFeedbackLen, 0);
    b[9] = kErrTemperature;                     // reg 65
    p.queueStatus(6, kErrVoltage, b);           // ... and the header's ERROR
    Feedback f{};
    CHECK_EQ(static_cast<int>(bus.readFeedback(6, f)),
             static_cast<int>(Status::kOk));
    CHECK_EQ(f.status, kErrTemperature | kErrVoltage);
}

void testTxFailureIsReported() {
    FakePort p;
    p.fail_write = true;
    Bus bus(p);
    CHECK_EQ(static_cast<int>(bus.writeU8(1, kRegTorqueEnable, 1)),
             static_cast<int>(Status::kTxFail));
}

// -- position-loop gains (scsbus/gains.h) ------------------------------------
// Each request the guarded write sends, decoded from the tx stream: the
// instruction, the register it addresses and the first data byte. Lets the
// tests say "exactly these writes, in this order, and NO goal write" rather
// than comparing offsets by hand.
struct Req { uint8_t id, inst, addr, b0, n; };
std::vector<Req> requests(const std::vector<uint8_t>& tx) {
    std::vector<Req> out;
    size_t i = 0;
    while (i + 5 < tx.size()) {
        if (tx[i] != 0xFF || tx[i + 1] != 0xFF) { ++i; continue; }
        const uint8_t len = tx[i + 3];
        Req r{tx[i + 2], tx[i + 4], tx[i + 5],
              len >= 4 ? tx[i + 6] : uint8_t{0},
              static_cast<uint8_t>(len >= 3 ? len - 3 : 0)};
        out.push_back(r);
        i += 4u + len;
    }
    return out;
}

bool touchesGoalOrTorqueWrite(const std::vector<Req>& rs) {
    for (const Req& r : rs) {
        if (r.inst != static_cast<uint8_t>(Inst::kWrite) &&
            r.inst != static_cast<uint8_t>(Inst::kSyncWrite) &&
            r.inst != static_cast<uint8_t>(Inst::kRegWrite)) {
            continue;
        }
        if (r.addr != kRegEepromLock && r.addr != kRegPosP) return true;
    }
    return false;
}

void testGainWriteHappyPath() {
    FakePort p;
    Bus bus(p);
    p.queueStatus(3, 0, {0});          // reg 40: torque OFF
    p.queueStatus(3, 0, {});           // unlock ack
    p.queueStatus(3, 0, {});           // 21/22 write ack
    p.queueStatus(3, 0, {});           // lock ack
    p.queueStatus(3, 0, {128, 64});    // read-back
    int settled = 0;
    static int* s_settled = nullptr;
    s_settled = &settled;
    PositionGains rb{0, 0};
    const GainWrite r = writePositionGains(
        bus, 3, PositionGains{128, 64}, &rb, [] { ++*s_settled; });
    CHECK_EQ(static_cast<int>(r), static_cast<int>(GainWrite::kOk));
    CHECK_EQ(rb.p, 128);
    CHECK_EQ(rb.d, 64);
    CHECK_EQ(settled, 1);
    const std::vector<Req> rs = requests(p.tx);
    CHECK_EQ(rs.size(), 5u);
    if (rs.size() == 5) {
        // read torque, unlock, write P+D together, lock, read P+D back
        CHECK_EQ(rs[0].inst, static_cast<uint8_t>(Inst::kRead));
        CHECK_EQ(rs[0].addr, kRegTorqueEnable);
        CHECK_EQ(rs[1].inst, static_cast<uint8_t>(Inst::kWrite));
        CHECK_EQ(rs[1].addr, kRegEepromLock);
        CHECK_EQ(rs[1].b0, 0);
        CHECK_EQ(rs[2].inst, static_cast<uint8_t>(Inst::kWrite));
        CHECK_EQ(rs[2].addr, kRegPosP);
        CHECK_EQ(rs[2].n, 2);
        CHECK_EQ(rs[2].b0, 128);
        CHECK_EQ(rs[3].addr, kRegEepromLock);
        CHECK_EQ(rs[3].b0, 1);
        CHECK_EQ(rs[4].inst, static_cast<uint8_t>(Inst::kRead));
        CHECK_EQ(rs[4].addr, kRegPosP);
        for (const Req& q : rs) CHECK_EQ(q.id, 3);   // never broadcast
    }
    CHECK(!touchesGoalOrTorqueWrite(rs));
}

void testGainWriteRefusesWithTorqueOn() {
    // Torque ON: refused after the one read. Nothing written -- not the
    // lock, not a gain, and above all not a goal.
    FakePort p;
    Bus bus(p);
    p.queueStatus(5, 0, {1});
    CHECK_EQ(static_cast<int>(writePositionGains(bus, 5, PositionGains{96, 32})),
             static_cast<int>(GainWrite::kTorqueOn));
    const std::vector<Req> rs = requests(p.tx);
    CHECK_EQ(rs.size(), 1u);
    if (!rs.empty()) CHECK_EQ(rs[0].inst, static_cast<uint8_t>(Inst::kRead));

    // Torque state unreadable (servo silent): refused the same way.
    FakePort q;
    Bus bus2(q);
    CHECK_EQ(static_cast<int>(writePositionGains(bus2, 5, PositionGains{96, 32})),
             static_cast<int>(GainWrite::kTorqueUnknown));
    CHECK_EQ(requests(q.tx).size(), 1u);
}

void testGainWriteRejectsBadArgsBeforeTheWire() {
    FakePort p;
    Bus bus(p);
    CHECK_EQ(static_cast<int>(writePositionGains(bus, kBroadcastId,
                                                 PositionGains{64, 32})),
             static_cast<int>(GainWrite::kBadArg));
    CHECK_EQ(static_cast<int>(writePositionGains(bus, 1, PositionGains{0, 32})),
             static_cast<int>(GainWrite::kBadArg));
    CHECK(p.tx.empty());
}

void testGainWriteRelocksOnFailure() {
    // The gain write is not acknowledged: the EEPROM must still be locked
    // again, and the verdict must say it failed.
    FakePort p;
    Bus bus(p);
    p.queueStatus(2, 0, {0});          // torque off
    p.queueStatus(2, 0, {});           // unlock ack
    // (no ack to the gain write)
    const GainWrite r = writePositionGains(bus, 2, PositionGains{64, 40});
    CHECK_EQ(static_cast<int>(r), static_cast<int>(GainWrite::kWriteFailed));
    const std::vector<Req> rs = requests(p.tx);
    CHECK(!rs.empty());
    if (!rs.empty()) {
        CHECK_EQ(rs.back().addr, kRegEepromLock);
        CHECK_EQ(rs.back().b0, 1);
    }
    CHECK(!touchesGoalOrTorqueWrite(rs));
}

void testGainWriteDetectsAMismatchedReadback() {
    FakePort p;
    Bus bus(p);
    p.queueStatus(4, 0, {0});
    p.queueStatus(4, 0, {});
    p.queueStatus(4, 0, {});
    p.queueStatus(4, 0, {});
    p.queueStatus(4, 0, {32, 32});     // the write did not take
    PositionGains rb{0, 0};
    CHECK_EQ(static_cast<int>(writePositionGains(bus, 4, PositionGains{128, 64},
                                                 &rb)),
             static_cast<int>(GainWrite::kMismatch));
    CHECK_EQ(rb.p, 32);
}

void testGainCheck() {
    FakePort p;
    Bus bus(p);
    const GainExpect want[3] = {{1, {32, 32}}, {2, {128, 64}}, {3, {32, 32}}};
    p.queueStatus(1, 0, {32, 32});     // as expected
    p.queueStatus(2, 0, {32, 32});     // a factory-reset spare: P = 32
    // servo 3 silent
    GainCheck got[3];
    CHECK_EQ(checkPositionGains(bus, want, 3, got), 2u);
    CHECK(got[0].answered && got[0].ok);
    CHECK(got[1].answered && !got[1].ok);
    CHECK_EQ(got[1].read.p, 32);
    CHECK(!got[2].answered && !got[2].ok);
    // A check is reads only.
    for (const Req& r : requests(p.tx)) {
        CHECK_EQ(r.inst, static_cast<uint8_t>(Inst::kRead));
        CHECK_EQ(r.addr, kRegPosP);
        CHECK_EQ(r.b0, 2);             // one READ of two bytes (21, 22)
    }

    FakePort q;
    Bus bus2(q);
    q.queueStatus(1, 0, {32, 32});
    q.queueStatus(2, 0, {128, 64});
    q.queueStatus(3, 0, {32, 32});
    CHECK_EQ(checkPositionGains(bus2, want, 3, got), 0u);
}

// The compiled expected-gain table (main/servo_gains.h; its static_asserts
// -- every bus servo has a row, P raised exactly where the deployed policy
// trained stiffer -- are compiled here too): one row per bus servo, and
// factory P/D on every row until #73 measures the Plan B values.
void testExpectedGainTable() {
    CHECK_EQ(robot::kNumExpectedGains, static_cast<size_t>(obs::kNumBusJoints));
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        const GainExpect* e = robot::expectedGainsFor(obs::kBusServoId[b]);
        CHECK(e != nullptr);
        if (!e) continue;
        CHECK_EQ(e->gains.p, kFactoryGains.p);
        CHECK_EQ(e->gains.d, kFactoryGains.d);
    }
    CHECK(robot::expectedGainsFor(0) == nullptr);
    CHECK(robot::expectedGainsFor(18) == nullptr);
}

}  // namespace

int main() {
    testVendorPackets();
    testSyncWriteShape();
    testSignMagnitude();
    testParseResponse();
    testDecodeFeedback();
    testBusPingAndRead();
    testBusTorqueBroadcast();
    testBusSyncWritePerServoSpeed();
    testBusSetIdLocksTheNewId();
    testBusSetMiddle();
    testBusSyncReadFeedback();
    testBusErrorFlagsSurface();
    testTxFailureIsReported();
    testGainWriteHappyPath();
    testGainWriteRefusesWithTorqueOn();
    testGainWriteRejectsBadArgsBeforeTheWire();
    testGainWriteRelocksOnFailure();
    testGainWriteDetectsAMismatchedReadback();
    testGainCheck();
    testExpectedGainTable();
    return testutil::report("scsbus");
}
