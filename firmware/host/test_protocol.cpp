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
        case LinkState::kLowBattLand: return "kLowBattLand";
        case LinkState::kLowBattSafe: return "kLowBattSafe";
        case LinkState::kFallen: return "kFallen";
        case LinkState::kBench: return "kBench";
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

void testExtendedCommandFrames() {
    for (size_t i = 0; i < V::kNumCmdExt; ++i) {
        const auto& c = V::kCmdExt[i];
        // decode (the python wire image; 14 B short frames included --
        // extras must come back at the trained defaults)
        Command got{};
        CHECK_EQ(static_cast<int>(decodeCommand(c.wire, c.wire_len, got)),
                 static_cast<int>(Err::kOk));
        CHECK_EQ(got.seq, c.seq);
        CHECK_EQ(got.flags, c.flags);
        CHECK_NEAR(got.vx, c.dec[0], 0.0);
        CHECK_NEAR(got.wz, c.dec[1], 0.0);
        CHECK_NEAR(got.vy, c.dec[2], 0.0);
        CHECK_NEAR(got.crouch, c.dec[3], 0.0);
        CHECK_NEAR(got.lift, c.dec[4], 0.0);
        CHECK_NEAR(got.foot_dx, c.dec[5], 0.0);
        CHECK_NEAR(got.foot_dz, c.dec[6], 0.0);

        // encode: the firmware's ext encoder must reproduce python's long
        // frames byte for byte (python emits SHORT frames when the extras
        // are at defaults -- those roundtrip through decode only)
        if (c.wire_len == kCmdLenExt) {
            Command src{};
            src.seq = c.seq;
            src.flags = c.flags;
            src.vx = c.vx;
            src.wz = c.wz;
            src.vy = c.vy;
            src.crouch = c.crouch;
            src.lift = c.lift;
            src.foot_dx = c.fdx;
            src.foot_dz = c.fdz;
            uint8_t wire[kCmdLenExt];
            CHECK_EQ(encodeCommandExt(wire, src), kCmdLenExt);
            CHECK_BYTES(wire, c.wire, kCmdLenExt);
        }

        // envelope clamp of the decoded frame, against the python reference
        Command cl = got;
        clampExtToEnvelope(cl);
        CHECK_NEAR(cl.vy, c.clamped[0], 0.0);
        CHECK_NEAR(cl.crouch, c.clamped[1], 0.0);
        CHECK_NEAR(cl.lift, c.clamped[2], 0.0);
        CHECK_NEAR(cl.foot_dx, c.clamped[3], 0.0);
        CHECK_NEAR(cl.foot_dz, c.clamped[4], 0.0);
    }
}

void testWatchdogExtChannels() {
    // Not vector-driven: the ext channels ride the same accept/decay path as
    // vx/wz, so a short script checks storage, ENABLE gating and stale decay.
    Watchdog dog;
    Command pkt{};
    pkt.seq = 1;
    pkt.flags = kFlagEnable;
    pkt.vx = 0.4f;
    pkt.lift = -1.0f;
    pkt.foot_dx = 0.03f;
    pkt.crouch = 0.8f;
    CHECK_EQ(dog.accept(pkt, 0.0f), true);
    float c7[7];
    dog.commandExt(1.0f, c7);
    CHECK_NEAR(c7[0], 0.4f, 0.0);
    CHECK_NEAR(c7[3], 0.8f, 0.0);
    CHECK_NEAR(c7[4], -1.0f, 0.0);
    CHECK_NEAR(c7[5], 0.03f, 0.0);
    // stale -> the trained stand command (crouch back to 1.0)
    dog.commandExt(kStaleMs + 2.0f, c7);
    CHECK_NEAR(c7[0], 0.0f, 0.0);
    CHECK_NEAR(c7[3], 1.0f, 0.0);
    CHECK_NEAR(c7[4], 0.0f, 0.0);
    // ENABLE off zeroes the extras like vx/wz
    pkt.seq = 2;
    pkt.flags = 0;
    CHECK_EQ(dog.accept(pkt, kStaleMs + 3.0f), true);
    dog.commandExt(kStaleMs + 4.0f, c7);
    CHECK_NEAR(c7[4], 0.0f, 0.0);
    CHECK_NEAR(c7[3], 1.0f, 0.0);
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
        // seq_echo's low byte is the bench diagnostic in kBench frames and
        // nothing at all anywhere else. Python's Telemetry.diag froze the
        // expected value; this is the same rule, transcribed.
        CHECK_EQ(telemetryDiag(back), c.diag);
    }
    // Crossing the two sockets must not decode (both live on one host).
    uint8_t cmd[kCmdLen + 6] = {};
    encodeCommand(cmd, 1, 0.0f, 0.0f, 0);
    Telemetry out{};
    CHECK(decodeTelemetry(cmd, kTlmLen, out) != Err::kOk);
}

void checkReason(uint8_t diag, const char* want, size_t i) {
    const char* got = diagReason(diag);
    ++testutil::g_checks;
    if (strcmp(got, want) != 0) {
        char msg[512];
        snprintf(msg, sizeof msg,
                 "diag case %zu (0x%02X): got \"%s\" want \"%s\"", i, diag,
                 got, want);
        testutil::fail(__FILE__, __LINE__, msg);
    }
}

void testBenchDiagnostics() {
    // The whole point of this byte is that the operator reads its SENTENCE,
    // so the sentences are golden too -- two copies of a message drift the
    // moment one side is reworded.
    for (size_t i = 0; i < V::kNumDiag; ++i) {
        const auto& c = V::kDiag[i];
        const uint8_t packed = packDiag(c.run != 0, c.cal_ok != 0,
                                        static_cast<ArmResult>(c.result));
        CHECK_EQ(packed, c.packed);
        CHECK_EQ(static_cast<int>(diagArmResult(packed)), c.result);
        CHECK_EQ((packed & kDiagRun) != 0, c.run != 0);
        CHECK_EQ((packed & kDiagCalOk) != 0, c.cal_ok != 0);
        checkReason(packed, c.reason, i);
    }
    for (size_t i = 0; i < V::kNumDiagRaw; ++i) {
        const auto& c = V::kDiagRaw[i];
        checkReason(c.diag, c.reason, i);
        // Reserved bits 2-3 must not leak into the result nibble, and an
        // ArmResult this build has never heard of must stay unrecognised
        // rather than aliasing onto kNone/kAccepted/kRefusedNoCal.
        const int known =
            static_cast<int>(diagArmResult(c.diag)) <=
            static_cast<int>(ArmResult::kRefusedNoCal);
        CHECK_EQ(known, c.known);
    }
    // Boot state: nothing asked, nothing refused, nothing calibrated.
    CHECK_EQ(packDiag(false, false, ArmResult::kNone), 0);
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

void runArmScript(const V::ArmStep* steps, size_t n) {
    ArmLatch latch;
    for (size_t i = 0; i < n; ++i) {
        bool want = false;
        const bool edge = latch.update(steps[i].flags, want);
        const int verdict = edge ? static_cast<int>(want) : -1;
        ++testutil::g_checks;
        if (verdict != steps[i].verdict) {
            char msg[128];
            snprintf(msg, sizeof msg, "arm step %zu flags 0x%02X: got %d want %d",
                     i, steps[i].flags, verdict, steps[i].verdict);
            testutil::fail(__FILE__, __LINE__, msg);
        }
        CHECK(latch.haveLevel());
        CHECK_EQ(static_cast<int>(latch.level()),
                 static_cast<int>((steps[i].flags & kFlagArm) != 0));
    }
}

void testArmLatch() {
    ArmLatch fresh;
    CHECK(!fresh.haveLevel());
    runArmScript(V::kArm, V::kNumArm);
    runArmScript(V::kArmHeld, V::kNumArmHeld);
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

// Extended telemetry (mirror mode): the classic body plus 10 joint angles,
// diffed against the bytes link/protocol.py produced. The frame is REQUESTED
// with kFlagPose and never volunteered -- see docs/mirror-mode.md for why
// telemetry could not copy the command frame's length-selection trick.
void testExtendedTelemetryFrames() {
    for (size_t c = 0; c < V::kNumTlmExt; ++c) {
        const V::TlmExtCase& k = V::kTlmExt[c];
        Telemetry t{};
        t.seq_echo = k.seq;
        t.state = static_cast<LinkState>(k.state);
        t.vbat_v = k.vbat;
        t.up_z = k.up_z;
        t.vx_est = k.vx;
        t.wz_est = k.wz;
        t.servo_err = k.servo_err;
        t.loop_late_pct = k.late;
        t.n_joints = static_cast<uint8_t>(kNumJoints);
        for (size_t i = 0; i < kNumJoints; ++i) t.joints[i] = k.joints[i];

        uint8_t wire[kTlmLenExt];
        const size_t n = encodeTelemetry(wire, t);
        CHECK_EQ(n, kTlmLenExt);
        CHECK_BYTES(wire, k.wire, kTlmLenExt);

        Telemetry back{};
        CHECK(decodeTelemetry(k.wire, kTlmLenExt, back) == Err::kOk);
        CHECK_EQ(back.n_joints, kNumJoints);
        for (size_t i = 0; i < kNumJoints; ++i) {
            // Milli-radian quantisation, and the saturating cases clamp.
            const float want = k.joints[i] > 32.767f    ? 32.767f
                             : k.joints[i] < -32.768f   ? -32.768f
                                                        : k.joints[i];
            CHECK_NEAR(back.joints[i], want, 0.001f);
        }
    }
}

// The property every existing commander depends on: not asking changes
// nothing. A Telemetry with no joints must encode to the classic 20 B frame,
// byte for byte, and a classic frame must decode with no joints.
void testClassicTelemetryIsUntouched() {
    for (size_t c = 0; c < V::kNumTlm; ++c) {
        const V::TlmCase& k = V::kTlm[c];
        Telemetry t{};
        t.seq_echo = k.seq;
        t.state = static_cast<LinkState>(k.state);
        t.vbat_v = k.vbat;
        t.up_z = k.up_z;
        t.vx_est = k.vx;
        t.wz_est = k.wz;
        t.servo_err = k.servo_err;
        t.loop_late_pct = k.late;
        t.n_joints = 0;                       // did not ask
        uint8_t wire[kTlmLenExt];
        CHECK_EQ(encodeTelemetry(wire, t), kTlmLen);
        CHECK_BYTES(wire, k.wire, kTlmLen);

        Telemetry back{};
        CHECK(decodeTelemetry(k.wire, kTlmLen, back) == Err::kOk);
        CHECK_EQ(back.n_joints, 0);
    }
    // A length between the two is still refused.
    Telemetry junk{};
    uint8_t buf[kTlmLenExt] = {0};
    CHECK(decodeTelemetry(buf, kTlmLen + 1, junk) == Err::kBadLength);
    CHECK(decodeTelemetry(buf, kTlmLenExt - 1, junk) == Err::kBadLength);
}

// kFlagPose is a level on the wire and must not disturb the other three.
void testPoseFlag() {
    uint8_t wire[kCmdLen];
    encodeCommand(wire, 1, 0.0f, 0.0f,
                  static_cast<uint8_t>(kFlagArm | kFlagPose));
    Command c{};
    CHECK(decodeCommand(wire, kCmdLen, c) == Err::kOk);
    CHECK(c.pose());
    CHECK(c.arm());
    CHECK(!c.enabled());
    CHECK(!c.estop());

    encodeCommand(wire, 2, 0.0f, 0.0f, kFlagArm);
    CHECK(decodeCommand(wire, kCmdLen, c) == Err::kOk);
    CHECK(!c.pose());
}

// kFlagHome is the "reset the servos" request: a bit that must ride
// alongside every other flag without disturbing one, because the states it
// is FOR are the states the other bits are describing (E-stopped, benched).
void testHomeFlag() {
    uint8_t wire[kCmdLen];
    encodeCommand(wire, 1, 0.0f, 0.0f,
                  static_cast<uint8_t>(kFlagEstop | kFlagHome | kFlagPose));
    Command c{};
    CHECK(decodeCommand(wire, kCmdLen, c) == Err::kOk);
    CHECK(c.home());
    CHECK(c.estop());
    CHECK(c.pose());
    CHECK(!c.arm());
    CHECK(!c.enabled());

    encodeCommand(wire, 2, 0.0f, 0.0f, kFlagEstop);
    CHECK(decodeCommand(wire, kCmdLen, c) == Err::kOk);
    CHECK(!c.home());
}

// The edge rule, which is the whole safety argument for a bit that MOVES a
// robot from a standstill.
void testHomeLatch() {
    HomeLatch h;
    // A client that boots with the bit already set moves nothing: same
    // first-frame rule as ArmLatch, and here it matters more -- the request
    // re-poses ten joints, and nobody is necessarily watching.
    CHECK(!h.update(kFlagHome));
    CHECK(!h.update(kFlagHome));

    HomeLatch g;
    CHECK(!g.update(0));                       // first frame: level low, seen
    CHECK(g.update(kFlagHome));                // 0 -> 1: the one event
    CHECK(!g.update(kFlagHome));               // held: idempotent, no re-home
    CHECK(!g.update(kFlagHome));
    CHECK(!g.update(0));                       // 1 -> 0 is not an "un-home"
    CHECK(g.update(kFlagHome));                // ... and the next press works

    // Deaf to every other bit: an arm, an e-stop and a pose request in the
    // same frames must not fake an edge.
    HomeLatch q;
    CHECK(!q.update(kFlagArm));
    CHECK(!q.update(static_cast<uint8_t>(kFlagArm | kFlagEnable)));
    CHECK(!q.update(static_cast<uint8_t>(kFlagEstop | kFlagPose)));
    CHECK(q.update(static_cast<uint8_t>(kFlagEstop | kFlagHome)));
}

// -- the attitude block (2026-09-03) ----------------------------------------
// up_z alone is how FAR from upright, never which way. All four lengths must
// round-trip, and -- the part that actually matters -- adding the block must
// not have changed a single byte of the two frames that existed before it,
// because the robot is the SENDER and an unexpected length is a dropped frame
// at every client already built.
void testAttitudeIsAdditive() {
    Telemetry t = {};
    t.state = LinkState::kLive;
    t.seq_echo = 4242;
    t.vbat_v = 11.4f;
    t.up_z = 0.887f;
    t.vx_est = 0.4f;
    t.wz_est = -0.25f;
    t.loop_late_pct = 3;

    uint8_t plain[kTlmLenMax] = {0};
    const size_t n_plain = encodeTelemetry(plain, t);
    CHECK(n_plain == kTlmLen);

    // Same telemetry, now asking for attitude: 4 bytes longer, and the first
    // 18 bytes IDENTICAL. Only the CRC moves.
    t.have_att = 1;
    t.up_x = 0.450f;
    t.up_y = -0.100f;
    uint8_t att[kTlmLenMax] = {0};
    const size_t n_att = encodeTelemetry(att, t);
    CHECK(n_att == kTlmLenAtt);
    CHECK(memcmp(plain, att, 18) == 0);

    Telemetry got = {};
    CHECK(decodeTelemetry(att, n_att, got) == Err::kOk);
    CHECK(got.have_att == 1);
    CHECK_NEAR(got.up_x, 0.450, 1e-3);
    CHECK_NEAR(got.up_y, -0.100, 1e-3);
    CHECK_NEAR(got.up_z, 0.887, 1e-3);
    CHECK(got.n_joints == 0);

    // With joints as well: the longest frame, joints BEFORE attitude, and the
    // 38-byte prefix still byte-identical to the joints-only frame.
    for (size_t i = 0; i < kNumJoints; ++i) {
        t.joints[i] = 0.1f * static_cast<float>(i) - 0.4f;
    }
    t.n_joints = static_cast<uint8_t>(kNumJoints);
    uint8_t both[kTlmLenMax] = {0};
    const size_t n_both = encodeTelemetry(both, t);
    CHECK(n_both == kTlmLenExtAtt);

    t.have_att = 0;
    uint8_t pose_only[kTlmLenMax] = {0};
    const size_t n_pose = encodeTelemetry(pose_only, t);
    CHECK(n_pose == kTlmLenExt);
    CHECK(memcmp(pose_only, both, kTlmLenExt - 2) == 0);   // all but the CRC

    Telemetry g2 = {};
    CHECK(decodeTelemetry(both, n_both, g2) == Err::kOk);
    CHECK(g2.n_joints == static_cast<uint8_t>(kNumJoints));
    CHECK(g2.have_att == 1);
    CHECK_NEAR(g2.up_x, 0.450, 1e-3);
    CHECK_NEAR(g2.up_y, -0.100, 1e-3);
    for (size_t i = 0; i < kNumJoints; ++i) {
        CHECK_NEAR(g2.joints[i], 0.1 * static_cast<double>(i) - 0.4, 1e-3);
    }

    // A frame that was not asked for attitude must report NONE, not zeros
    // that read as "perfectly upright".
    Telemetry g3 = {};
    CHECK(decodeTelemetry(plain, n_plain, g3) == Err::kOk);
    CHECK(g3.have_att == 0);

    // Lengths between the valid ones are still refused.
    Telemetry g4 = {};
    CHECK(decodeTelemetry(att, kTlmLenAtt - 1, g4) == Err::kBadLength);
    CHECK(decodeTelemetry(both, 42, g4) == Err::kBadLength);

    // A flipped bit in the new block must fail the CRC -- the block rides
    // inside it precisely because the CRC is length-agnostic.
    uint8_t bad[kTlmLenMax];
    memcpy(bad, both, n_both);
    bad[kTlmLenExt] = static_cast<uint8_t>(bad[kTlmLenExt] ^ 0x01);
    CHECK(decodeTelemetry(bad, n_both, g4) == Err::kBadCrc);
}

// The two implementations pinned to the SAME literal, so neither can drift
// onto its own private idea of the frame. Produced by link/protocol.py and
// asserted identically in tests/test_protocol.py -- this is the house rule
// that the port is diffed against a number, not against whatever Python said.
void testAttitudeFrameMatchesPython() {
    Telemetry t = {};
    t.state = LinkState::kLive;
    t.seq_echo = 4242;
    // 11.5 exactly, not 11.4: the encoder TRUNCATES millivolts to match
    // protocol.py, and 11.4f is 11.39999961 as a float, so it would truncate
    // to 11399 here and 11400 there -- a difference in the vbat field, not in
    // anything this test is about. (On the wire vbat is always an integer
    // millivolt from vbat_dv * 100, so the robot never hits this.)
    t.vbat_v = 11.5f;
    t.up_z = 0.887f;
    t.vx_est = 0.4f;
    t.wz_est = -0.25f;
    t.loop_late_pct = 3;
    t.have_att = 1;
    t.up_x = 0.450f;
    t.up_y = -0.100f;
    const uint8_t want[] = {
        0x42, 0x54, 0x01, 0x00, 0x92, 0x10, 0x00, 0x00, 0xec, 0x2c, 0x77, 0x03,
        0x90, 0x01, 0x06, 0xff, 0x00, 0x03, 0xc2, 0x01, 0x9c, 0xff, 0x72, 0xf7
    };
    uint8_t got[kTlmLenMax] = {0};
    const size_t n = encodeTelemetry(got, t);
    CHECK(n == sizeof want);
    for (size_t i = 0; i < sizeof want; ++i) CHECK(got[i] == want[i]);
}

// kFlagAtt is a request, like kFlagPose: its own bit, disturbing no other.
void testAttFlagIsItsOwnBit() {
    uint8_t wire[kCmdLen];
    Command c = {};
    encodeCommand(wire, 1, 0.0f, 0.0f, kFlagAtt);
    CHECK(decodeCommand(wire, kCmdLen, c) == Err::kOk);
    CHECK(c.att());
    CHECK(!c.pose() && !c.arm() && !c.estop() && !c.enabled() && !c.home());

    Command c2 = {};
    encodeCommand(wire, 2, 0.0f, 0.0f,
                  static_cast<uint8_t>(kFlagPose | kFlagAtt));
    CHECK(decodeCommand(wire, kCmdLen, c2) == Err::kOk);
    CHECK(c2.att() && c2.pose());
}

int main() {
    testAttitudeIsAdditive();
    testAttitudeFrameMatchesPython();
    testAttFlagIsItsOwnBit();
    testCrc();
    testCommandFrames();
    testExtendedCommandFrames();
    testWatchdogExtChannels();
    testBadFrames();
    testEverySingleBitFlipIsCaught();
    testTelemetryFrames();
    testExtendedTelemetryFrames();
    testClassicTelemetryIsUntouched();
    testPoseFlag();
    testHomeFlag();
    testHomeLatch();
    testBenchDiagnostics();
    testClampToEnvelope();
    testWatchdog();
    testArmLatch();
    testEndToEnd();
    testDemux();
    return testutil::report("protocol");
}
