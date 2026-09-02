// The console core, tested where it can be: link/client.cpp's intent and
// frame-length rule, link/keymap.cpp's file format, link/mjpeg.cpp's stream
// parser.
//
// These are the parts of an operator's console that have no screen in them,
// which is exactly why they were split out of tui.cpp: a rule about when ARM
// is given up on, or when a 24 B frame is emitted, is testable prose, and a
// window is not. Everything here runs under ASan/UBSan in `make test`.
//
// The stream parser gets the adversarial half of the attention: it is the one
// piece of the console that eats bytes from the network.
#include "client.h"
#include "keymap.h"
#include "mjpeg.h"
#include "test_util.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

using namespace bimo;

namespace {

// -- the frame-length rule --------------------------------------------------
// link/protocol.py:encode_command emits the SHORT frame whenever the extras
// round to their defaults. Old firmware drops a long frame BY LENGTH, so a
// console that is not asking for anything extended must not look like one
// that is -- and the comparison has to happen on the milli-rounded ints, or a
// slider resting 1e-7 off 1.0 silently goes long.
void test_frame_length_rule() {
    Ext e;
    CHECK(e.atDefaults());

    e.crouch = 1.0f + 1e-6f;                  // below half a milli-unit
    CHECK(e.atDefaults());
    e.crouch = 0.9994f;                       // rounds to 999: a real change
    CHECK(!e.atDefaults());
    e.reset();
    CHECK(e.atDefaults());

    e.foot_dx = 0.0004f;                      // rounds to 0
    CHECK(e.atDefaults());
    e.foot_dx = 0.0006f;                      // rounds to 1
    CHECK(!e.atDefaults());
    e.reset();

    e.lift = 1.0f;
    CHECK(!e.atDefaults());
    e.reset();
    e.foot_dz = -0.01f;
    CHECK(!e.atDefaults());
}

void test_ext_clamped_to_the_trained_draw() {
    Ext e;
    e.crouch = 0.1f;                          // below crouch_range[0]
    e.lift = 5.0f;
    e.foot_dx = 1.0f;
    e.foot_dz = -1.0f;
    e.clamp();
    CHECK_NEAR(e.crouch, linkproto::kCrouchMin, 1e-6f);
    CHECK_NEAR(e.lift, 1.0f, 1e-6f);
    CHECK_NEAR(e.foot_dx, linkproto::kFootDMax, 1e-6f);
    CHECK_NEAR(e.foot_dz, -linkproto::kFootDMax, 1e-6f);
}

// -- the intent -------------------------------------------------------------
void test_disarmed_refuses_motion() {
    Intent in;
    Speeds s;
    CHECK(!in.press(kAxForward, 0.0));
    CHECK(!in.moving());
    CHECK(strstr(in.note, "not armed") != nullptr);

    float vx, vy, wz;
    Ext ext;
    uint8_t flags;
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK_EQ(flags, 0);                       // no ARM, no ENABLE
}

// A window has real key-release, so this is the whole dead-man: press sets,
// release clears, and the very next frame is a stand. No timeout anywhere.
void test_release_stands_immediately() {
    Intent in;
    Speeds s;
    in.arm(0.0);
    CHECK(in.press(kAxForward, 0.0));
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK_NEAR(vx, s.vx, 1e-6f);
    CHECK((flags & linkproto::kFlagEnable) != 0);

    in.release(kAxForward);
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK_NEAR(vx, 0.0f, 1e-6f);
    CHECK((flags & linkproto::kFlagEnable) == 0);
    CHECK((flags & linkproto::kFlagArm) != 0);    // still armed, just standing
}

// Two axes at once is the other thing a terminal cannot do.
void test_axes_sum() {
    Intent in;
    Speeds s;
    in.arm(0.0);
    in.press(kAxForward, 0.0);
    in.press(kAxTurnLeft, 0.0);
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK_NEAR(vx, s.vx, 1e-6f);
    CHECK_NEAR(wz, s.wz, 1e-6f);

    in.press(kAxLeft, 0.0);
    in.press(kAxRight, 0.0);                  // opposing strafes cancel
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK_NEAR(vy, 0.0f, 1e-6f);
}

void test_estop_latches_until_stand() {
    Intent in;
    Speeds s;
    in.arm(0.0);
    in.press(kAxForward, 0.0);
    in.fireEstop();
    CHECK(!in.moving());                      // the press is dropped, not held
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK((flags & linkproto::kFlagEstop) != 0);
    CHECK((flags & linkproto::kFlagEnable) == 0);

    CHECK(!in.press(kAxForward, 0.0));        // latched: motion is refused
    in.stand(nullptr);
    CHECK(!in.estop);
    CHECK(in.press(kAxForward, 0.0));
}

// A crouch at zero velocity still needs ENABLE: linkproto::Watchdog gates the
// extended channels on it too, so without this the slider does nothing.
void test_extras_alone_set_enable() {
    Intent in;
    Speeds s;
    in.arm(0.0);
    in.ext.crouch = 0.8f;
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK((flags & linkproto::kFlagEnable) != 0);
    CHECK_NEAR(vx, 0.0f, 1e-6f);
    CHECK_NEAR(ext.crouch, 0.8f, 1e-6f);
}

// The robot is authoritative: armed here but BENCH there, once the edge has
// had time to land, means the arm was refused or the robot rebooted.
void test_arm_sync_gives_up_and_quotes_the_robot() {
    Intent in;
    Link link;
    link.have_tlm = true;
    link.tlm.state = LinkState::kBench;
    link.tlm.seq_echo = linkproto::packDiag(false, false,
                                            linkproto::ArmResult::kRefusedNoCal);
    link.tlm_at_ms = 0.0;

    in.arm(0.0);
    CHECK(!in.syncToRobot(link, 100.0));      // too early to conclude anything
    CHECK(in.armed);

    link.tlm_at_ms = kArmSyncMs + 100.0;      // a fresh beacon, still BENCH
    CHECK(in.syncToRobot(link, kArmSyncMs + 100.0));
    CHECK(!in.armed);
    CHECK(strstr(in.note, "stayed BENCH") != nullptr);
    CHECK(strstr(in.note, "calibration") != nullptr);
}

void test_arm_sync_ignores_a_dead_link() {
    Intent in;
    Link link;
    link.have_tlm = true;
    link.tlm.state = LinkState::kBench;
    link.tlm_at_ms = 0.0;
    in.arm(0.0);
    // Beacon older than kTlmLostMs: the console knows nothing, so it must not
    // conclude the arm was refused.
    CHECK(!in.syncToRobot(link, 10000.0));
    CHECK(in.armed);
}

// -- reset the servos -------------------------------------------------------
// The point of the request is that it survives the states in which every
// other channel on this wire correctly refuses to move anything.
void test_home_is_sent_disarmed_and_estopped() {
    Speeds s;
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;

    Intent cold;                              // the boot state: disarmed
    cold.requestHome();
    cold.frame(s, vx, vy, wz, ext, flags);
    CHECK((flags & linkproto::kFlagHome) != 0);
    CHECK((flags & linkproto::kFlagArm) == 0);      // it does not arm ...
    CHECK((flags & linkproto::kFlagEnable) == 0);   // ... and commands nothing

    // frame() returns EARLY once the E-stop is latched, so the home bit has
    // to be set before that return -- otherwise the one console button that
    // can stand a fallen robot up is dead in exactly the state it is for.
    Intent in;
    in.arm(0.0);
    in.press(kAxForward, 0.0);
    in.fireEstop();
    in.requestHome();
    CHECK(!in.armed);                  // the robot benches to home; so do we
    CHECK(!in.moving());
    in.frame(s, vx, vy, wz, ext, flags);
    CHECK((flags & linkproto::kFlagHome) != 0);
    CHECK((flags & linkproto::kFlagEstop) != 0);    // still latched, honestly
    CHECK((flags & linkproto::kFlagEnable) == 0);
    CHECK_NEAR(vx, 0.0f, 1e-6f);
}

// The level is held for kHomeFrames and then DROPS: the robot acts on the
// rising edge, so a level that never falls would make the second press a
// no-op, and one that falls too soon can be lost with a single UDP packet.
void test_home_level_is_held_then_released() {
    Intent in;
    Speeds s;
    Link link;                         // unopened: sendto fails, frames count
    float vx, vy, wz;
    Ext ext;
    uint8_t flags;

    in.requestHome();
    int flagged = 0;
    for (int i = 0; i < kHomeFrames + 5; ++i) {
        sendIntent(link, in, s, vx, vy, wz, ext, flags);
        if (flags & linkproto::kFlagHome) ++flagged;
    }
    CHECK_EQ(flagged, kHomeFrames);
    CHECK_EQ(in.home_frames, 0);

    in.requestHome();                  // a fresh press makes a fresh edge
    sendIntent(link, in, s, vx, vy, wz, ext, flags);
    CHECK((flags & linkproto::kFlagHome) != 0);
}

// -- the keymap -------------------------------------------------------------
void test_keymap_names_round_trip() {
    for (int a = 0; a < kActCount; ++a) {
        CHECK_EQ(actionFromName(actionName(a)), a);
    }
    const int keys[] = {'a', 'z', '0', '9', kKeySpace, kKeyPageDown, kKeyF12,
                        kKeyUp, kKeyDelete};
    for (size_t i = 0; i < sizeof keys / sizeof keys[0]; ++i) {
        CHECK_EQ(keyFromName(keyName(keys[i])), keys[i]);
    }
    CHECK_EQ(keyFromName("A"), 'a');          // case-folded
    CHECK_EQ(keyFromName("pageup"), kKeyPageUp);
    CHECK_EQ(keyFromName("nonsense"), kKeyNone);
}

// One key can only mean one thing: a duplicate that silently shadows is worse
// than one that visibly steals.
void test_keymap_bind_is_exclusive() {
    Keymap k;
    CHECK_EQ(k.actionFor(kKeyUp), kActForward);
    k.bind(kActEstop, kKeyUp);
    CHECK_EQ(k.actionFor(kKeyUp), kActEstop);
    CHECK_EQ(k.key[kActForward], kKeyNone);
}

void test_keymap_file_round_trip() {
    char path[] = "/tmp/bimo_keymap_testXXXXXX";
    const int fd = mkstemp(path);
    CHECK(fd >= 0);
    ::close(fd);

    Keymap a;
    a.bind(kActForward, 'w');
    a.bind(kActBack, 's');
    a.bind(kActEstop, kKeyF5);
    char err[192] = "";
    CHECK(a.save(path, err, sizeof err));

    Keymap b;
    CHECK(b.load(path, err, sizeof err));
    for (int i = 0; i < kActCount; ++i) CHECK_EQ(b.key[i], a.key[i]);
    unlink(path);

    // A missing file is not an error: it means "use the defaults".
    Keymap c;
    CHECK(c.load("/tmp/bimo_keymap_definitely_absent", err, sizeof err));
    CHECK_EQ(c.key[kActForward], kKeyUp);
}

// A bad line is refused loudly. A silently ignored binding is a key that does
// nothing on a robot that is already moving.
void test_keymap_rejects_a_bad_file() {
    char path[] = "/tmp/bimo_keymap_badXXXXXX";
    const int fd = mkstemp(path);
    CHECK(fd >= 0);
    FILE* f = fdopen(fd, "w");
    fprintf(f, "# a comment\n\nforward = Up\nnonsense = Up\n");
    fclose(f);

    Keymap k;
    char err[192] = "";
    CHECK(!k.load(path, err, sizeof err));
    CHECK(strstr(err, "unknown action") != nullptr);

    f = fopen(path, "w");
    fprintf(f, "forward = Sideways\n");
    fclose(f);
    CHECK(!k.load(path, err, sizeof err));
    CHECK(strstr(err, "unknown key") != nullptr);

    f = fopen(path, "w");
    fprintf(f, "forward Up\n");
    fclose(f);
    CHECK(!k.load(path, err, sizeof err));
    CHECK(strstr(err, "expected") != nullptr);
    unlink(path);
}

// -- the stream parser ------------------------------------------------------
const char kHttp[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=siltwinframe\r\n\r\n";

size_t part(char* out, size_t cap, const char* body, size_t n) {
    const int h = snprintf(out, cap,
                           "--siltwinframe\r\nContent-Type: image/jpeg\r\n"
                           "Content-Length: %zu\r\n\r\n", n);
    memcpy(out + h, body, n);
    memcpy(out + h + n, "\r\n", 2);
    return static_cast<size_t>(h) + n + 2;
}

void test_parser_reads_parts() {
    MjpegParser p;
    char buf[512];
    size_t n = 0;
    memcpy(buf, kHttp, sizeof kHttp - 1);
    n = sizeof kHttp - 1;
    n += part(buf + n, sizeof buf - n, "JPEGONE", 7);
    n += part(buf + n, sizeof buf - n, "TWO", 3);

    p.feed(reinterpret_cast<const uint8_t*>(buf), n);
    const uint8_t* f = nullptr;
    size_t len = 0;
    CHECK(p.take(&f, &len));
    CHECK_EQ(len, 7u);
    CHECK(memcmp(f, "JPEGONE", 7) == 0);
    // Both parts arrived in one read: the second must surface without waiting
    // for more bytes.
    CHECK(p.take(&f, &len));
    CHECK_EQ(len, 3u);
    CHECK(memcmp(f, "TWO", 3) == 0);
    CHECK(!p.take(&f, &len));
    CHECK(!p.overflowed());
}

// The network splits wherever it likes, including inside a header and inside
// a body. One byte at a time is the worst case, so test that.
void test_parser_survives_split_reads() {
    char buf[512];
    size_t n = sizeof kHttp - 1;
    memcpy(buf, kHttp, n);
    n += part(buf + n, sizeof buf - n, "SPLITME!", 8);

    MjpegParser p;
    int got = 0;
    for (size_t i = 0; i < n; ++i) {
        p.feed(reinterpret_cast<const uint8_t*>(buf) + i, 1);
        const uint8_t* f = nullptr;
        size_t len = 0;
        while (p.take(&f, &len)) {
            CHECK_EQ(len, 8u);
            CHECK(memcmp(f, "SPLITME!", 8) == 0);
            ++got;
        }
    }
    CHECK_EQ(got, 1);
    CHECK(!p.overflowed());
}

// A part that stops half way (the producer died mid-frame) must yield
// nothing, rather than a half-decoded image the operator would read as live.
void test_parser_holds_a_truncated_frame() {
    char buf[512];
    size_t n = sizeof kHttp - 1;
    memcpy(buf, kHttp, n);
    n += part(buf + n, sizeof buf - n, "COMPLETE", 8);
    const size_t cut = n - 4;                 // chop the tail off the body

    MjpegParser p;
    p.feed(reinterpret_cast<const uint8_t*>(buf), cut);
    const uint8_t* f = nullptr;
    size_t len = 0;
    // The header block promised 8 bytes and fewer arrived: nothing comes out.
    CHECK(!p.take(&f, &len));
    CHECK(!p.overflowed());

    // ... and the moment the rest turns up, the whole frame does.
    p.feed(reinterpret_cast<const uint8_t*>(buf) + cut, n - cut);
    CHECK(p.take(&f, &len));
    CHECK_EQ(len, 8u);
    CHECK(memcmp(f, "COMPLETE", 8) == 0);
}

// A header block that never ends is not a header block; refuse it rather than
// buffer the machine to death.
void test_parser_refuses_endless_headers() {
    MjpegParser p;
    char junk[8192];
    memset(junk, 'x', sizeof junk);
    for (int i = 0; i < 16 && !p.overflowed(); ++i) {
        p.feed(reinterpret_cast<const uint8_t*>(junk), sizeof junk);
    }
    CHECK(p.overflowed());
}

}  // namespace

int main() {
    test_frame_length_rule();
    test_ext_clamped_to_the_trained_draw();
    test_disarmed_refuses_motion();
    test_release_stands_immediately();
    test_axes_sum();
    test_estop_latches_until_stand();
    test_extras_alone_set_enable();
    test_arm_sync_gives_up_and_quotes_the_robot();
    test_arm_sync_ignores_a_dead_link();
    test_home_is_sent_disarmed_and_estopped();
    test_home_level_is_held_then_released();
    test_keymap_names_round_trip();
    test_keymap_bind_is_exclusive();
    test_keymap_file_round_trip();
    test_keymap_rejects_a_bad_file();
    test_parser_reads_parts();
    test_parser_survives_split_reads();
    test_parser_holds_a_truncated_frame();
    test_parser_refuses_endless_headers();
    return testutil::report("client");
}
