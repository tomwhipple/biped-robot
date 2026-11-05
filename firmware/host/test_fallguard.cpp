// fallguard::Detector -- the ONE fall detector both control paths run
// (ctrl_task.cpp with Motion::kPolicy, cli.cpp fallTick with Motion::kBench).
//
// The properties a downed robot depends on, asserted:
//   * the limits: 45 deg / 200 ms under the policy, 30 deg / 100 ms on the
//     bench, and the sim's episode end (linkproto::kFallUpZ, 66 deg) is NOT
//     what the hardware uses any more
//   * a short dip does not trip it; a gap in the attitude neither trips nor
//     clears anything
//   * recovery needs upright AND still, continuously, for 2 s
//   * an armed-loop fall handed to the bench Detector (latch()) recovers by
//     the same pick-up rule -> autohome (the cross-core hand-over, modelled)
#include "fallguard/fallguard.h"

#include <math.h>

#include "linkproto/protocol.h"
#include "test_util.h"

using fallguard::Detector;
using fallguard::Event;
using fallguard::Motion;

namespace {

constexpr uint32_t kDt = 20;   // ms per tick (ctrl 50 Hz; the bench house loop is 10-20 ms)

// up vector of a torso tilted `deg` about the body x axis (a roll)
void upAt(float deg, float up[3]) {
    const float r = deg / 57.29578f;
    up[0] = 0.0f;
    up[1] = -sinf(r);
    up[2] = cosf(r);
}

const float kStill[3] = {0.0f, 0.0f, 0.0f};

// Feed `ms` of a constant tilt; returns the first non-kNone event and the time it came
// (or kNone). `now` advances.
Event feed(Detector& d, Motion m, float deg, uint32_t ms, uint32_t& now, uint32_t* at = nullptr,
           const float* gyro = kStill, bool valid = true) {
    float up[3];
    upAt(deg, up);
    Event first = Event::kNone;
    for (uint32_t t = 0; t < ms; t += kDt) {
        const Event e = d.update(m, valid, up, gyro, now);
        if (e != Event::kNone && first == Event::kNone) {
            first = e;
            if (at) *at = now;
        }
        now += kDt;
    }
    return first;
}

void testLimitsTable() {
    CHECK_NEAR(fallguard::kPolicyLimits.fall_up_z, cos(45.0 / 57.29578), 1e-4);
    CHECK_EQ(fallguard::kPolicyLimits.debounce_ms, 200u);
    CHECK_NEAR(fallguard::kBenchLimits.fall_up_z, cos(30.0 / 57.29578), 1e-4);
    CHECK_EQ(fallguard::kBenchLimits.debounce_ms, 100u);
    CHECK_NEAR(fallguard::kPickupUpZ, cos(fallguard::kPickupTiltDeg / 57.29578), 1e-4);
    // the hardware's armed line is tighter than the sim's episode end, which stays 0.4
    CHECK_NEAR(linkproto::kFallUpZ, 0.4, 1e-6);
    CHECK(fallguard::kPolicyLimits.fall_up_z > linkproto::kFallUpZ);
    CHECK(fallguard::kBenchLimits.fall_up_z > fallguard::kPolicyLimits.fall_up_z);
    CHECK(fallguard::limits(Motion::kPolicy).debounce_ms == 200u);
    CHECK(fallguard::limits(Motion::kBench).debounce_ms == 100u);
    float up[3];
    upAt(45.0f, up);
    CHECK_NEAR(fallguard::tiltDeg(up), 45.0, 1e-3);
    upAt(0.0f, up);
    CHECK_NEAR(fallguard::tiltDeg(up), 0.0, 1e-3);
}

void testPolicyTripsAt45After200ms() {
    Detector d;
    uint32_t now = 1000, at = 0;
    // 40 deg forever: inside the policy line (it would trip the bench one)
    CHECK(feed(d, Motion::kPolicy, 40.0f, 5000, now) == Event::kNone);
    CHECK(!d.fallen());
    // 50 deg: trips exactly once the run has lasted 200 ms
    const uint32_t t0 = now;
    CHECK(feed(d, Motion::kPolicy, 50.0f, 400, now, &at) == Event::kFell);
    CHECK(d.fallen());
    CHECK_EQ(at - t0, 200u);
    CHECK(d.fellIn() == Motion::kPolicy);
    CHECK_NEAR(d.fallTiltDeg(), 50.0, 0.01);
    CHECK_EQ(d.falls(), 1u);
    // latched: further tilt is no new event
    CHECK(feed(d, Motion::kPolicy, 80.0f, 1000, now) == Event::kNone);
    CHECK_EQ(d.falls(), 1u);
}

void testOldSimLineNoLongerNeeded() {
    // 60 deg (above kFallUpZ's 66) used to be "still walking" for the armed loop; now it is down.
    Detector d;
    uint32_t now = 0;
    CHECK(feed(d, Motion::kPolicy, 60.0f, 300, now) == Event::kFell);
}

void testBenchTripsAt30After100ms() {
    Detector d;
    uint32_t now = 0, at = 0;
    CHECK(feed(d, Motion::kBench, 28.0f, 3000, now) == Event::kNone);
    const uint32_t t0 = now;
    CHECK(feed(d, Motion::kBench, 35.0f, 300, now, &at) == Event::kFell);
    CHECK_EQ(at - t0, 100u);
    CHECK(d.fellIn() == Motion::kBench);
}

void testShortDipDoesNotTrip() {
    Detector d;
    uint32_t now = 0;
    for (int k = 0; k < 50; ++k) {
        CHECK(feed(d, Motion::kPolicy, 60.0f, 180, now) == Event::kNone);   // 9 ticks down
        CHECK(feed(d, Motion::kPolicy, 5.0f, 20, now) == Event::kNone);     // one tick up resets
    }
    CHECK(!d.fallen());
    Detector b;
    CHECK(feed(b, Motion::kBench, 40.0f, 80, now) == Event::kNone);
    CHECK(feed(b, Motion::kBench, 5.0f, 20, now) == Event::kNone);
    CHECK(feed(b, Motion::kBench, 40.0f, 80, now) == Event::kNone);
    CHECK(!b.fallen());
}

void testInvalidSampleNeitherTripsNorCompletes() {
    Detector d;
    uint32_t now = 0;
    // 160 ms down, then a gap, then 160 ms down: never 200 ms continuous
    CHECK(feed(d, Motion::kPolicy, 70.0f, 160, now) == Event::kNone);
    CHECK(feed(d, Motion::kPolicy, 70.0f, 100, now, nullptr, kStill, false) == Event::kNone);
    CHECK(feed(d, Motion::kPolicy, 70.0f, 160, now) == Event::kNone);
    CHECK(!d.fallen());
    // non-finite attitude counts as invalid
    float up[3] = {NAN, 0.0f, 0.1f};
    for (int k = 0; k < 50; ++k) {
        now += kDt;
        CHECK(d.update(Motion::kPolicy, true, up, kStill, now) == Event::kNone);
    }
    CHECK(!d.fallen());
    // and while fallen, a gap breaks the pick-up hold
    CHECK(feed(d, Motion::kPolicy, 70.0f, 300, now) == Event::kFell);
    CHECK(feed(d, Motion::kBench, 0.0f, 1900, now) == Event::kNone);
    CHECK(feed(d, Motion::kBench, 0.0f, 40, now, nullptr, kStill, false) == Event::kNone);
    CHECK(feed(d, Motion::kBench, 0.0f, 1900, now) == Event::kNone);
    uint32_t at = 0;
    const uint32_t t0 = now;
    CHECK(feed(d, Motion::kBench, 0.0f, 400, now, &at) == Event::kPickedUp);
    CHECK(at - t0 < 200u);
}

void testPickupNeedsUprightAndStillFor2s() {
    Detector d;
    uint32_t now = 0, at = 0;
    CHECK(feed(d, Motion::kBench, 90.0f, 500, now) == Event::kFell);
    // lying there: nothing
    CHECK(feed(d, Motion::kBench, 90.0f, 5000, now) == Event::kNone);
    // carried upright but swinging (|w| 0.5 rad/s): nothing
    const float swing[3] = {0.3f, 0.4f, 0.0f};
    CHECK(feed(d, Motion::kBench, 3.0f, 5000, now, nullptr, swing) == Event::kNone);
    // still but 15 deg off upright: nothing
    CHECK(feed(d, Motion::kBench, 15.0f, 5000, now) == Event::kNone);
    // upright 1.9 s, a wobble, upright again: the hold restarts
    CHECK(feed(d, Motion::kBench, 4.0f, 1900, now) == Event::kNone);
    CHECK(feed(d, Motion::kBench, 4.0f, 20, now, nullptr, swing) == Event::kNone);
    const uint32_t t0 = now;
    CHECK(feed(d, Motion::kBench, 4.0f, 2500, now, &at) == Event::kPickedUp);
    CHECK_EQ(at - t0, 2000u);
    CHECK(d.fallen());          // the caller clears once the home has started
    d.clear();
    CHECK(!d.fallen());
    CHECK(feed(d, Motion::kBench, 4.0f, 3000, now) == Event::kNone);
}

void testPickupRepeatsWhileNotCleared() {
    // autohome off: the caller ignores kPickedUp, the latch stays, and it comes at most every 2 s
    Detector d;
    uint32_t now = 0;
    CHECK(feed(d, Motion::kBench, 90.0f, 200, now) == Event::kFell);
    int n = 0;
    float up[3];
    upAt(0.0f, up);
    for (int k = 0; k < 500; ++k) n += d.update(Motion::kBench, true, up, kStill, now += kDt) == Event::kPickedUp;
    CHECK(n >= 4 && n <= 5);    // 10 s / 2 s
    CHECK(d.fallen());
}

void testClockWrap() {
    Detector d;
    uint32_t now = 0xFFFFFFFFu - 50u;
    CHECK(feed(d, Motion::kPolicy, 70.0f, 400, now) == Event::kFell);
}

// The cross-core hand-over, modelled: ctrl's Detector (kPolicy) trips, ctrl
// stores the tilt and bumps the counter (shared.h g_armed_falls) and benches
// the loop; housekeeping sees the new count, latches ITS Detector, and the
// bench pick-up rule then fires the autohome -- with nothing re-armed.
void testArmedFallHandsOverToBenchAutohome() {
    Detector ctrl, bench;
    uint32_t armed_falls = 0, seen = 0;
    float handed_tilt = 0.0f;
    bool mode_run = true, torque_on = true, homed = false;
    uint32_t now = 0;
    float up[3];
    // walking at 10 deg, then over to 55 deg
    for (int k = 0; k < 100; ++k) {
        const float deg = k < 50 ? 10.0f : 55.0f;
        upAt(deg, up);
        if (mode_run && !ctrl.fallen()) {
            if (ctrl.update(Motion::kPolicy, true, up, kStill, now) == Event::kFell) {
                handed_tilt = ctrl.fallTiltDeg();
                ++armed_falls;
                mode_run = false;           // g_mode_request = kBench
            }
        }
        if (ctrl.fallen()) torque_on = false;   // limp this very tick
        // housekeeping: the hand-over first (before the mode gate), then the bench Detector
        if (armed_falls != seen) {
            seen = armed_falls;
            bench.latch(Motion::kPolicy, handed_tilt);
        }
        if (!mode_run) {
            const Event e = bench.update(Motion::kBench, true, up, kStill, now);
            CHECK(e != Event::kFell);       // already latched: no second "fall"
        }
        now += kDt;
    }
    CHECK(!mode_run);
    CHECK(!torque_on);
    CHECK(bench.fallen());
    CHECK(bench.fellIn() == Motion::kPolicy);
    CHECK_NEAR(bench.fallTiltDeg(), 55.0, 0.01);
    CHECK_EQ(bench.falls(), 1u);
    // lying 3 s, then picked up and held upright and still
    for (int k = 0; k < 150; ++k) {
        upAt(85.0f, up);
        now += kDt;
        CHECK(bench.update(Motion::kBench, true, up, kStill, now) == Event::kNone);
    }
    for (int k = 0; k < 150 && !homed; ++k) {
        upAt(2.0f, up);
        now += kDt;
        if (bench.update(Motion::kBench, true, up, kStill, now) == Event::kPickedUp) {
            homed = true;                   // cli.cpp: homeAll(), then clear()
            bench.clear();
        }
    }
    CHECK(homed);
    CHECK(!bench.fallen());
    CHECK(!mode_run);                       // the recovery homes; it never arms
    // latch() is idempotent: a second hand-over while latched keeps the first numbers
    Detector b2;
    b2.latch(Motion::kPolicy, 50.0f);
    b2.latch(Motion::kBench, 80.0f);
    CHECK(b2.fellIn() == Motion::kPolicy);
    CHECK_NEAR(b2.fallTiltDeg(), 50.0, 1e-6);
    CHECK_EQ(b2.falls(), 1u);
}

void testCandidateMs() {
    Detector d;
    uint32_t now = 0;
    CHECK_EQ(d.candidateMs(now), 0u);
    feed(d, Motion::kPolicy, 50.0f, 100, now);
    CHECK_EQ(d.candidateMs(now), 100u);
    CHECK(fallguard::eventName(Event::kFell)[0] == 'f');
    CHECK(fallguard::motionName(Motion::kPolicy)[0] == 'p');
}

}  // namespace

int main() {
    testLimitsTable();
    testPolicyTripsAt45After200ms();
    testOldSimLineNoLongerNeeded();
    testBenchTripsAt30After100ms();
    testShortDipDoesNotTrip();
    testInvalidSampleNeitherTripsNorCompletes();
    testPickupNeedsUprightAndStillFor2s();
    testPickupRepeatsWhileNotCleared();
    testClockWrap();
    testArmedFallHandsOverToBenchAutohome();
    testCandidateMs();
    return testutil::report("fallguard");
}
