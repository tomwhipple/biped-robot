// The C++ link protocol vs. link/protocol.py's own output.
//
// Every vector in protocol_vectors.h was produced by running the Python
// reference; nothing here was typed by hand except the CCITT-FALSE check
// value, which is the one number the whole chain can be diffed against.
#include <stdint.h>

#include "linkproto/framing.h"
#include "linkproto/protocol.h"
#include "linkproto/watchdog.h"
#include "test_util.h"
#include "vectors/protocol_vectors.h"

using namespace linkproto;
namespace V = protocol_vectors;

namespace {

const char* errName(Err e) {
    switch (e) {
        case Err::kOk: return "kOk";
        case Err::kBadLength: return "kBadLength";
        case Err::kBadMagic: return "kBadMagic";
        case Err::kBadVersion: return "kBadVersion";
        case Err::kBadCrc: return "kBadCrc";
        case Err::kBadState: return "kBadState";
    }
    return "?";
}

const char* stateName(LinkState s) {
    switch (s) {
        case LinkState::kLive: return "kLive";
        case LinkState::kStand: return "kStand";
        case LinkState::kRelax: return "kRelax";
        case LinkState::kEstop: return "kEstop";
    }
    return "?";
}

void testCrc() {
    // docs/control-channel.md: "so a C port can be diffed against a number
    // rather than against 'whatever Python said'."
    CHECK_EQ(crc16Ccitt(reinterpret_cast<const uint8_t*>("123456789"), 9),
             0x29B1);
    for (size_t i = 0; i < V::kNumCrc; ++i) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(V::kCrc[i].text);
        CHECK_EQ(crc16Ccitt(p, V::kCrc[i].len), V::kCrc[i].crc);
    }
}

void testCommandFrames() {
    for (size_t i = 0; i < V::kNumCmd; ++i) {
        const auto& c = V::kCmd[i];
        uint8_t wire[kCmdLen];
        CHECK_EQ(encodeCommand(wire, c.seq, c.vx, c.wz, c.flags), kCmdLen);
        CHECK_BYTES(wire, c.wire, kCmdLen);

        Command got{};
        CHECK_EQ(static_cast<int>(decodeCommand(c.wire, kCmdLen, got)),
                 static_cast<int>(Err::kOk));
        CHECK_EQ(got.seq, c.seq);
        CHECK_EQ(got.flags, c.flags);
        // Milli-unit quantisation is exactly representable in float, so this
        // is an equality, not an epsilon.
        CHECK_NEAR(got.vx, c.dec_vx, 0.0);
        CHECK_NEAR(got.wz, c.dec_wz, 0.0);
    }
}

void testBadFrames() {
    for (size_t i = 0; i < V::kNumBadCmd; ++i) {
        const auto& b = V::kBadCmd[i];
        Command out{};
        const Err e = decodeCommand(b.wire, b.len, out);
        ++testutil::g_checks;
        if (strcmp(errName(e), b.err) != 0) {
            char msg[128];
            snprintf(msg, sizeof msg, "bad frame %zu: got %s want %s", i,
                     errName(e), b.err);
            testutil::fail(__FILE__, __LINE__, msg);
        }
    }
}

void testEverySingleBitFlipIsCaught() {
    // tests/test_protocol.py:test_crc_catches_every_single_bit_flip, ported.
    uint8_t good[kCmdLen];
    encodeCommand(good, 3, 0.7f, -0.2f, kFlagEnable);
    int caught = 0, tried = 0;
    for (size_t byte = 0; byte + 2 < kCmdLen; ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t bad[kCmdLen];
            memcpy(bad, good, kCmdLen);
            bad[byte] ^= static_cast<uint8_t>(1u << bit);
            Command out{};
            ++tried;
            if (decodeCommand(bad, kCmdLen, out) != Err::kOk) ++caught;
        }
    }
    CHECK_EQ(caught, tried);
    CHECK_EQ(tried, 96);
}

void testTelemetryFrames() {
    for (size_t i = 0; i < V::kNumTlm; ++i) {
        const auto& c = V::kTlm[i];
        Telemetry t{};
        t.seq_echo = c.seq;
        t.state = static_cast<LinkState>(c.state);
        t.vbat_v = c.vbat;
        t.up_z = c.up_z;
        t.vx_est = c.vx;
        t.wz_est = c.wz;
        t.servo_err = c.servo_err;
        t.loop_late_pct = c.late;
        uint8_t wire[kTlmLen];
        CHECK_EQ(encodeTelemetry(wire, t), kTlmLen);
        CHECK_BYTES(wire, c.wire, kTlmLen);

        Telemetry back{};
        CHECK_EQ(static_cast<int>(decodeTelemetry(c.wire, kTlmLen, back)),
                 static_cast<int>(Err::kOk));
        CHECK_EQ(back.seq_echo, c.seq);
        CHECK_EQ(static_cast<int>(back.state), c.state);
        CHECK_EQ(back.servo_err, c.servo_err);
        CHECK_EQ(back.loop_late_pct, c.late);
        CHECK_NEAR(back.up_z, c.up_z, 1e-3);
    }
    // Crossing the two sockets must not decode (both live on one host).
    uint8_t cmd[kCmdLen + 6] = {};
    encodeCommand(cmd, 1, 0.0f, 0.0f, 0);
    Telemetry out{};
    CHECK(decodeTelemetry(cmd, kTlmLen, out) != Err::kOk);
}

void testClampToEnvelope() {
    for (size_t i = 0; i < V::kNumClamp; ++i) {
        const auto& c = V::kClamp[i];
        float vx = 0.0f, wz = 0.0f;
        clampToEnvelope(c.vx, c.wz, vx, wz);
        CHECK_NEAR(vx, c.out_vx, 1e-6);
        CHECK_NEAR(wz, c.out_wz, 1e-6);
    }
}

void testWatchdog() {
    Watchdog dog;
    CHECK(dog.valid());
    CHECK(!Watchdog(500.0f, 100.0f).valid());    // 0 < stale < relax

    for (size_t i = 0; i < V::kNumWatchdog; ++i) {
        const auto& s = V::kWatchdog[i];
        if (s.op == 0) {
            Command pkt{s.seq, s.vx, s.wz, s.flags};
            const bool ok = dog.accept(pkt, s.now_ms);
            CHECK_EQ(static_cast<int>(ok), s.accepted);
        } else if (s.op == 1) {
            const LinkState st = dog.state(s.now_ms);
            ++testutil::g_checks;
            if (strcmp(stateName(st), s.state) != 0) {
                char msg[160];
                snprintf(msg, sizeof msg,
                         "watchdog step %zu at %.1f ms: got %s want %s", i,
                         static_cast<double>(s.now_ms), stateName(st),
                         s.state);
                testutil::fail(__FILE__, __LINE__, msg);
            }
        } else {
            float vx = 0.0f, wz = 0.0f;
            dog.command(s.now_ms, vx, wz);
            CHECK_NEAR(vx, s.out_vx, 1e-6);
            CHECK_NEAR(wz, s.out_wz, 1e-6);
        }
        CHECK_EQ(dog.rejected(), s.rejected);
    }
}

void testEndToEnd() {
    // docs/control-channel.md's port note 4, in miniature.
    Watchdog dog;
    uint8_t wire[kCmdLen];
    encodeCommand(wire, 1, 0.75f, -0.5f, kFlagEnable);
    Command pkt{};
    CHECK_EQ(static_cast<int>(decodeCommand(wire, kCmdLen, pkt)),
             static_cast<int>(Err::kOk));
    CHECK(dog.accept(pkt, 0.0f));
    float vx = 0.0f, wz = 0.0f;
    dog.command(0.0f, vx, wz);
    CHECK_NEAR(vx, 0.75, 1e-3);
    CHECK_NEAR(wz, -0.5, 1e-3);
}

void testDemux() {
    // The UART0 stream carries CLI lines and binary frames on one wire.
    Demux d;
    const char* line = "scan\n";
    Demux::Event ev = Demux::Event::kNone;
    for (const char* p = line; *p; ++p) ev = d.push(static_cast<uint8_t>(*p));
    CHECK_EQ(static_cast<int>(ev), static_cast<int>(Demux::Event::kLine));
    CHECK(strcmp(d.line(), "scan") == 0);

    uint8_t wire[kCmdLen];
    encodeCommand(wire, 12, 0.6f, 0.1f, kFlagEnable);
    ev = Demux::Event::kNone;
    for (size_t i = 0; i < kCmdLen; ++i) ev = d.push(wire[i]);
    CHECK_EQ(static_cast<int>(ev), static_cast<int>(Demux::Event::kFrame));
    Command pkt{};
    CHECK_EQ(static_cast<int>(decodeCommand(d.frame(), kCmdLen, pkt)),
             static_cast<int>(Err::kOk));
    CHECK_EQ(pkt.seq, 12);

    // A line that merely starts with 'B' is still a line.
    const char* b = "Bus\r\n";
    ev = Demux::Event::kNone;
    for (const char* p = b; *p; ++p) {
        const Demux::Event e = d.push(static_cast<uint8_t>(*p));
        if (e != Demux::Event::kNone) ev = e;
    }
    CHECK_EQ(static_cast<int>(ev), static_cast<int>(Demux::Event::kLine));
    CHECK(strcmp(d.line(), "Bus") == 0);

    // A frame arriving right after a line, with no separator, still parses.
    d.reset();
    for (const char* p = "volt\n"; *p; ++p) d.push(static_cast<uint8_t>(*p));
    ev = Demux::Event::kNone;
    for (size_t i = 0; i < kCmdLen; ++i) ev = d.push(wire[i]);
    CHECK_EQ(static_cast<int>(ev), static_cast<int>(Demux::Event::kFrame));

    // Overlong garbage does not run off the end of the buffer.
    d.reset();
    ev = Demux::Event::kNone;
    for (int i = 0; i < 400; ++i) ev = d.push('x');
    CHECK(ev == Demux::Event::kOverflow || ev == Demux::Event::kNone);
}

}  // namespace

int main() {
    testCrc();
    testCommandFrames();
    testBadFrames();
    testEverySingleBitFlipIsCaught();
    testTelemetryFrames();
    testClampToEnvelope();
    testWatchdog();
    testEndToEnd();
    testDemux();
    return testutil::report("protocol");
}
