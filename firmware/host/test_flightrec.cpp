// flightrec -- the flight recorder's wire format and its producer ring.
//
// The vectors (vectors/flightrec_vectors.h) are generated from
// tools/flightrec.py, the decoder the collector on mira uses: each case is a
// record built field by field plus the bytes Python encoded for it. Encoding
// the former here must give exactly the latter -- that is what keeps the
// robot and the collector speaking one format.
#include "flightrec/record.h"

#include <math.h>
#include <string.h>

#include "test_util.h"
#include "vectors/flightrec_vectors.h"

namespace fv = flightrec_vectors;
using flightrec::Header;

namespace {

template <size_t N>
void checkBytes(const uint8_t* got, size_t n, const uint8_t (&want)[N]) {
    CHECK_EQ(n, N);
    if (n == N) CHECK(memcmp(got, want, N) == 0);
    for (size_t i = 0; i < N && i < n; ++i) {
        if (got[i] != want[i]) {
            fprintf(stderr, "  first difference at byte %zu: 0x%02X vs 0x%02X\n", i,
                    got[i], want[i]);
            break;
        }
    }
}

template <size_t N>
void tickCase(void (*build)(Header&, flightrec::Tick&), const uint8_t (&wire)[N]) {
    Header h;
    flightrec::Tick t;
    build(h, t);
    uint8_t buf[flightrec::kMaxPacket];
    const size_t n = flightrec::encodeTick(buf, sizeof buf, h, t);
    checkBytes(buf, n, wire);
    CHECK_EQ(n, flightrec::kHeaderLen + flightrec::kTickFixedLen +
                    flightrec::kJointLen * t.njoint);

    // decode -> every field back
    Header h2;
    flightrec::Tick t2;
    CHECK(flightrec::decodeTick(wire, N, h2, t2));
    CHECK_EQ(h2.seq, h.seq);
    CHECK_EQ(h2.t_utc_us, h.t_utc_us);
    CHECK_EQ(h2.drop_net, h.drop_net);
    CHECK_EQ(t2.njoint, t.njoint);
    CHECK_EQ(static_cast<int>(t2.source), static_cast<int>(t.source));
    CHECK_EQ(t2.fb_off_us, t.fb_off_us);
    CHECK(t2.quat[1] == t.quat[1] && t2.up[2] == t.up[2] && t2.pitch == t.pitch);
    CHECK(t2.bal.u == t.bal.u && t2.bal.clamps == t.bal.clamps);
    CHECK_EQ(t2.bal.ankle_ticks, t.bal.ankle_ticks);
    CHECK(t2.ctrl.cmd[5] == t.ctrl.cmd[5] && t2.ctrl.phase == t.ctrl.phase);
    CHECK_EQ(t2.ctrl.home_stage, t.ctrl.home_stage);
    for (int k = 0; k < t.njoint; ++k) {
        CHECK_EQ(t2.joint[k].pos, t.joint[k].pos);
        CHECK_EQ(t2.joint[k].nominal, t.joint[k].nominal);
        CHECK_EQ(t2.joint[k].goal_speed, t.joint[k].goal_speed);
        CHECK_EQ(t2.joint[k].current, t.joint[k].current);
    }
    // re-encode what was decoded: identical bytes
    uint8_t again[flightrec::kMaxPacket];
    checkBytes(again, flightrec::encodeTick(again, sizeof again, h2, t2), wire);
}

template <size_t N>
void eventCase(void (*build)(Header&, flightrec::Event&), const uint8_t (&wire)[N]) {
    Header h;
    flightrec::Event e;
    build(h, e);
    uint8_t buf[flightrec::kMaxPacket];
    checkBytes(buf, flightrec::encodeEvent(buf, sizeof buf, h, e), wire);
    Header h2;
    flightrec::Event e2;
    CHECK(flightrec::decodeEvent(wire, N, h2, e2));
    CHECK_EQ(e2.len, e.len);
    CHECK(memcmp(e2.text, e.text, e.len) == 0);
    CHECK_EQ(e2.a, e.a);
    CHECK_EQ(e2.b, e.b);
}

void testVectors() {
    tickCase(fv::tick12_bal, fv::tick12_bal_wire);
    tickCase(fv::tick17_ctrl, fv::tick17_ctrl_wire);
    tickCase(fv::tick0_idle, fv::tick0_idle_wire);
    eventCase(fv::ev_cmd, fv::ev_cmd_wire);
    eventCase(fv::ev_bal, fv::ev_bal_wire);
    eventCase(fv::ev_out, fv::ev_out_wire);

    Header h;
    flightrec::Hello m;
    fv::hello(h, m);
    uint8_t buf[flightrec::kMaxPacket];
    const size_t n = flightrec::encodeHello(buf, sizeof buf, h, m);
    checkBytes(buf, n, fv::hello_wire);
    CHECK_EQ(n, flightrec::kHelloLen);
    Header h2;
    flightrec::Hello m2;
    CHECK(flightrec::decodeHello(fv::hello_wire, sizeof fv::hello_wire, h2, m2));
    CHECK(strcmp(m2.fw, m.fw) == 0);
    CHECK_EQ(m2.us_other, m.us_other);
    CHECK_EQ(m2.zero[16], m.zero[16]);
    CHECK_EQ(m2.dir[1], -1);
}

void testHeaderAndRejects() {
    Header h;
    flightrec::Tick t;
    fv::tick12_bal(h, t);
    uint8_t buf[flightrec::kMaxPacket];
    const size_t n = flightrec::encodeTick(buf, sizeof buf, h, t);
    Header hh;
    CHECK(flightrec::decodeHeader(buf, n, hh));
    CHECK(hh.type == flightrec::Type::kTick);
    CHECK_EQ(hh.boot_id, 0xA1B2C3D4u);

    flightrec::Tick t2;
    // short by one byte, long by one byte: both refused
    CHECK(!flightrec::decodeTick(buf, n - 1, hh, t2));
    uint8_t longer[flightrec::kMaxPacket + 1];
    memcpy(longer, buf, n);
    longer[n] = 0;
    CHECK(!flightrec::decodeTick(longer, n + 1, hh, t2));
    // wrong magic / version
    uint8_t bad[flightrec::kMaxPacket];
    memcpy(bad, buf, n);
    bad[0] = 'X';
    CHECK(!flightrec::decodeHeader(bad, n, hh));
    memcpy(bad, buf, n);
    bad[2] = flightrec::kVersion + 1;
    CHECK(!flightrec::decodeHeader(bad, n, hh));
    // a tick is not an event
    flightrec::Event e;
    CHECK(!flightrec::decodeEvent(buf, n, hh, e));
    // a buffer too small to encode into: 0, nothing claimed to fit
    CHECK_EQ(flightrec::encodeTick(buf, n - 1, h, t), 0);
    t.njoint = flightrec::kMaxJoints + 1;
    CHECK_EQ(flightrec::encodeTick(buf, sizeof buf, h, t), 0);
}

void testMaxSizes() {
    Header h;
    flightrec::Tick t;
    t.njoint = flightrec::kMaxJoints;
    uint8_t buf[flightrec::kMaxPacket];
    CHECK_EQ(flightrec::encodeTick(buf, sizeof buf, h, t), flightrec::kTickMaxLen);
    flightrec::Event e;
    char longline[400];
    memset(longline, 'x', sizeof longline - 1);
    longline[sizeof longline - 1] = 0;
    flightrec::setText(e, longline);
    CHECK_EQ(e.len, flightrec::kMaxText);
    CHECK_EQ(flightrec::encodeEvent(buf, sizeof buf, h, e), flightrec::kEventMaxLen);
}

void testText() {
    flightrec::Event e;
    flightrec::setText(e, "home readback: all 12 joints at their zeros\r\n");
    CHECK_EQ(e.len, 43);
    CHECK(e.text[e.len - 1] == 's');
    flightrec::setText(e, nullptr);
    CHECK_EQ(e.len, 0);
    flightrec::setText(e, "\r\n");
    CHECK_EQ(e.len, 0);
}

void testSat16() {
    CHECK_EQ(flightrec::sat16(40000), 32767);
    CHECK_EQ(flightrec::sat16(-40000), -32768);
    CHECK_EQ(flightrec::sat16(-7), -7);
}

void testRollPitch() {
    // 20 deg roll about x, then 10 deg pitch about y: q = qy * qx (ZYX, yaw 0)
    const float r = 20.0f * 0.0174532925f, p = 10.0f * 0.0174532925f;
    const float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    const float q[4] = {cp * cr, cp * sr, sp * cr, -sp * sr};
    float roll = 0, pitch = 0;
    flightrec::rollPitchFromQuat(q, &roll, &pitch);
    CHECK_NEAR(roll, r, 1e-5);
    CHECK_NEAR(pitch, p, 1e-5);
    const float level[4] = {1, 0, 0, 0};
    flightrec::rollPitchFromQuat(level, &roll, &pitch);
    CHECK_NEAR(roll, 0, 1e-7);
    CHECK_NEAR(pitch, 0, 1e-7);
}

void testRing() {
    static flightrec::Ring<4> ring;
    size_t len = 0;
    CHECK(ring.peek(&len) == nullptr);
    // fill: four claims succeed, the fifth is refused (the producer drops)
    for (int i = 0; i < 4; ++i) {
        uint8_t* s = ring.claim();
        CHECK(s != nullptr);
        if (s) {
            s[0] = static_cast<uint8_t>(10 + i);
            ring.publish(1 + static_cast<size_t>(i));
        }
    }
    CHECK(ring.claim() == nullptr);
    CHECK_EQ(ring.size(), 4);
    // FIFO order, lengths kept
    for (int i = 0; i < 2; ++i) {
        const uint8_t* p = ring.peek(&len);
        CHECK(p != nullptr);
        if (p) {
            CHECK_EQ(p[0], 10 + i);
            CHECK_EQ(len, 1 + i);
        }
        ring.pop();
    }
    // abandoned claim (publish 0) does not advance
    CHECK(ring.claim() != nullptr);
    ring.publish(0);
    CHECK_EQ(ring.size(), 2);
    // wrap the indices well past N
    for (int i = 0; i < 1000; ++i) {
        uint8_t* s = ring.claim();
        if (!s) {
            ring.pop();
            s = ring.claim();
        }
        CHECK(s != nullptr);
        if (s) {
            s[0] = static_cast<uint8_t>(i & 0xFF);
            ring.publish(1);
        }
    }
    int last = -1;
    while (const uint8_t* p = ring.peek(&len)) {
        last = p[0];
        ring.pop();
    }
    CHECK_EQ(last, 999 & 0xFF);
    CHECK_EQ(ring.size(), 0);
}

}  // namespace

int main() {
    testVectors();
    testHeaderAndRejects();
    testMaxSizes();
    testText();
    testSat16();
    testRollPitch();
    testRing();
    return testutil::report("flightrec");
}
