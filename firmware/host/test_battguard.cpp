// battguard::Guard -- the pack under-voltage supervisor.
//
// Every property here is one the robot's battery depends on, so they are
// asserted rather than reasoned about. The two that matter most:
//
//   * a millisecond sag must NOT trip it (6.8 A gait transients are normal)
//   * a recovery after it HAS tripped must not un-trip it (the rail rebounds
//     as soon as torque drops, which is exactly when it must stay down)
#include "battguard/guard.h"

#include "test_util.h"

using battguard::Guard;
using battguard::Level;

namespace {

constexpr float kDt = 20.0f;   // ms per control tick, 50 Hz

// Feed `n` ticks of a steady reading, advancing the injected clock.
void feed(Guard& g, uint8_t dv, int n, float& now) {
    for (int i = 0; i < n; ++i) {
        g.update(dv, now);
        now += kDt;
    }
}

void testHealthyPackIsSilent() {
    Guard g;
    CHECK(g.valid());
    float now = 0.0f;
    feed(g, 118, 500, now);                    // 11.8 V for 10 s
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
    CHECK(!g.torqueMustRelease());
    CHECK_NEAR(g.crouch(), 1.0, 1e-6);
    CHECK_EQ(g.lastDv(), 118);
}

void testWarnAssertsAndClears() {
    Guard g;
    float now = 0.0f;
    feed(g, 104, battguard::kConfirmTicks - 1, now);
    CHECK(g.level() == Level::kOk);            // not yet confirmed
    feed(g, 104, 1, now);
    CHECK(g.level() == Level::kWarn);          // 10.4 V, confirmed

    // Warn is a live indicator, not a latch: a pack that recovers (a lighter
    // gait, a fresh cell settling) goes quiet again.
    feed(g, 112, 5, now);
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
}

void testWarnBoundaryIsInclusive() {
    // docs/wiring.md says "land the robot BY 10.5 V", and cli.cpp's `volt`
    // warns at v <= 10.5. Exactly 10.5 must warn, not sit one count short.
    Guard g;
    float now = 0.0f;
    feed(g, 105, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kWarn);
}

void testTransientSagDoesNotTrip() {
    Guard g;
    float now = 0.0f;
    feed(g, 114, 100, now);
    // A gait transient: well below the land line, but only for a few ticks.
    for (int burst = 0; burst < 20; ++burst) {
        feed(g, 92, battguard::kConfirmTicks - 1, now);   // 9.2 V, ~0.48 s
        feed(g, 114, 10, now);                            // rail recovers
    }
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
    CHECK_EQ(g.belowTicks(), 0);
}

void testSustainedLowTripsAndRamps() {
    Guard g;
    float now = 0.0f;
    feed(g, 114, 50, now);
    feed(g, 98, battguard::kConfirmTicks - 1, now);
    // 9.8 V is below BOTH lines, so both counters advance together and neither
    // has confirmed yet: a pack that falls off a cliff never shows a warn
    // phase, it goes straight to landing on the confirming tick. The warn
    // phase is for the normal case -- a pack sagging past 10.5 V on its way
    // down, which testWarnAssertsAndClears covers.
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());

    feed(g, 98, 1, now);                        // the confirming tick
    CHECK(g.level() == Level::kLanding);
    CHECK(g.latched());
    CHECK(!g.torqueMustRelease());              // still under control
    CHECK_NEAR(g.crouch(), 1.0, 1e-6);          // ramp starts at full height

    const float t0 = now;
    // Half way through the ramp: half way between 1.0 and the crouch floor.
    while (now < t0 + battguard::kLandMs / 2.0f) feed(g, 98, 1, now);
    CHECK(g.level() == Level::kLanding);
    CHECK_NEAR(g.crouch(), 1.0 - (1.0 - battguard::kCrouchFloor) * 0.5, 0.02);

    // Monotone, and never below the trained floor.
    float prev = g.crouch();
    while (now < t0 + battguard::kLandMs + 4 * kDt) {
        feed(g, 98, 1, now);
        CHECK(g.crouch() <= prev + 1e-6f);
        CHECK(g.crouch() >= battguard::kCrouchFloor - 1e-6f);
        prev = g.crouch();
    }
    CHECK(g.level() == Level::kSafe);
    CHECK(g.torqueMustRelease());
    CHECK_NEAR(g.crouch(), battguard::kCrouchFloor, 1e-6);
}

void testLatchSurvivesRecovery() {
    // The property the whole class exists for. Torque drops, the unloaded pack
    // rebounds past 11 V, and the robot must STAY down.
    Guard g;
    float now = 0.0f;
    feed(g, 98, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kLanding);
    feed(g, 126, 5000, now);                    // 12.6 V for 100 s
    CHECK(g.level() == Level::kSafe);
    CHECK(g.torqueMustRelease());
    CHECK(g.latched());
}

void testNoReadingIsNotZeroVolts() {
    // dv == 0 is "no servo answered this tick". Treating it as 0 V would sit
    // the robot down on the first dropped bus frame.
    Guard g;
    float now = 0.0f;
    feed(g, 0, 1000, now);
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
    CHECK_EQ(g.lastDv(), 0);

    // And a dropout mid-discharge neither counts nor clears the count.
    Guard h;
    now = 0.0f;
    feed(h, 98, battguard::kConfirmTicks - 1, now);
    const int before = h.belowTicks();
    feed(h, 0, 50, now);
    CHECK_EQ(h.belowTicks(), before);
    CHECK(!h.latched());
    feed(h, 98, 1, now);
    CHECK(h.level() == Level::kLanding);
}

void testResetClearsTheLatch() {
    Guard g;
    float now = 0.0f;
    feed(g, 90, 5000, now);
    CHECK(g.level() == Level::kSafe);
    g.reset();
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
    CHECK(!g.torqueMustRelease());
    CHECK_NEAR(g.crouch(), 1.0, 1e-6);
    CHECK_EQ(g.lastDv(), 90);                   // the reading itself is history

    // ... and a still-flat pack trips it straight back.
    feed(g, 90, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kLanding);
}

void test2SThresholds() {
    Guard g(battguard::kWarn2S, battguard::kLand2S);
    CHECK(g.valid());
    float now = 0.0f;
    feed(g, 74, 200, now);                      // 7.4 V nominal 2S
    CHECK(g.level() == Level::kOk);
    feed(g, 69, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kWarn);
    feed(g, 65, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kLanding);
}

void testInvalidConfig() {
    CHECK(!Guard(99, 105).valid());              // land above warn
    CHECK(!Guard(105, 99, 0).valid());           // no confirmation at all
    CHECK(!Guard(105, 99, 25, 0.0f).valid());    // zero-length landing
    CHECK(Guard(105, 105).valid());              // equal is legal
}

}  // namespace

int main() {
    testHealthyPackIsSilent();
    testWarnAssertsAndClears();
    testWarnBoundaryIsInclusive();
    testTransientSagDoesNotTrip();
    testSustainedLowTripsAndRamps();
    testLatchSurvivesRecovery();
    testNoReadingIsNotZeroVolts();
    testResetClearsTheLatch();
    test2SThresholds();
    testInvalidConfig();
    return testutil::report("battguard");
}
