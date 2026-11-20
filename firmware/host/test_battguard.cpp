// battguard::Guard -- the pack under-voltage supervisor.
//
// Every property here is one the robot's battery depends on, so they are
// asserted rather than reasoned about. The two that matter most:
//
//   * a millisecond sag must NOT trip it (6.8 A gait transients are normal)
//   * a recovery after it HAS tripped must not un-trip it (the rail rebounds
//     as soon as torque drops, which is exactly when it must stay down)
#include "battguard/guard.h"
#include "battguard/pack.h"

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

// -- the pack sense (battguard/pack.h; issue #109) ---------------------------
namespace {

// The INA219's bus register, decoded as the datasheet's Figure 27 (TI SBOS448):
// bits 15..3, 4 mV a count, CNVR/OVF ignored.
void testIna219BusDecode() {
    namespace ina = battguard::ina219;
    CHECK_EQ(ina::busMv(0x0000), 0);
    CHECK_EQ(ina::busMv(0x0007), 0);                   // only CNVR, OVF, reserved
    CHECK_EQ(ina::busMv(static_cast<uint16_t>(3150u << 3)), 12600);   // 12.6 V
    CHECK_EQ(ina::busMv(static_cast<uint16_t>((2775u << 3) | 0x3)), 11100);
    CHECK_EQ(ina::busMv(0xFFF8), 32764);               // full scale: 8191 counts
    const uint8_t be[2] = {0x5D, 0x4A};                // MSB first on the wire
    CHECK_EQ(ina::reg16(be), 0x5D4A);
    CHECK_EQ(ina::busMv(ina::reg16(be)), (0x5D4A >> 3) * 4);
    // Only a mode that converts the bus continuously is left alone.
    CHECK(ina::busContinuous(ina::kConfigDefault));    // 0x399F: MODE 111
    CHECK(ina::busContinuous(0x399E));                 // 110: bus continuous
    CHECK(!ina::busContinuous(0x3998));                // 000: power-down
    CHECK(!ina::busContinuous(0x399A));                // 010: bus triggered
    CHECK(!ina::busContinuous(0x399B));                // 011: triggered
    CHECK(!ina::busContinuous(0x399D));                // 101: shunt only
}

void testDvFromMvTruncates() {
    CHECK_EQ(battguard::dvFromMv(10599), 105);   // never reads the pack higher
    CHECK_EQ(battguard::dvFromMv(10500), 105);
    CHECK_EQ(battguard::dvFromMv(9999), 99);
    CHECK_EQ(battguard::dvFromMv(0), 0);
    CHECK_EQ(battguard::dvFromMv(65535), 255);   // saturates
}

// Either sense alone, or the lower of the two; a pack reading under the floor
// (the board on USB) is no reading, never a flat pack.
void testInputDvTakesTheLower() {
    using battguard::inputDv;
    CHECK_EQ(inputDv(0, 0), 0);                  // no data at all
    CHECK_EQ(inputDv(118, 0), 118);              // servos only: today's guard
    CHECK_EQ(inputDv(0, 11840), 118);            // the bus is down: the INA219 guards
    CHECK_EQ(inputDv(117, 11840), 117);          // under load the servos read lower
    CHECK_EQ(inputDv(118, 9950), 99);            // the pack reads lower: it wins
    CHECK_EQ(inputDv(118, battguard::kPackFloorMv - 1), 118);   // USB: ignored
    CHECK_EQ(inputDv(0, 4400), 0);
    CHECK_EQ(inputDv(0, battguard::kPackFloorMv), 60);
}

// The bus fails, the INA219 keeps guarding: the same trip, on the same ticks,
// as the servos would have given.
void testGuardTripsOnThePackAlone() {
    Guard g;
    float now = 0.0f;
    for (int i = 0; i < battguard::kConfirmTicks; ++i) {
        g.update(battguard::inputDv(0, 9850), now);
        now += kDt;
    }
    CHECK(g.level() == Level::kLanding);
    CHECK_EQ(g.lastDv(), 98);
}

void testPackSenseSchedule() {
    battguard::PackSense s;
    using P = battguard::PackSense;
    CHECK(s.due(0));                             // the first read is due at once
    CHECK_EQ(s.packMv(0), 0);                    // and nothing is known yet
    s.ok(11840, 0);
    CHECK_EQ(s.packMv(0), 11840);
    CHECK(!s.due(P::kPeriodMs - 1));
    CHECK(s.due(P::kPeriodMs));
    // Fresh up to kStaleMs, unknown after: never the last value forever.
    CHECK_EQ(s.packMv(P::kStaleMs), 11840);
    CHECK_EQ(s.packMv(P::kStaleMs + 1), 0);

    // A failure keeps the 10 Hz retry until kMissAfter in a row ...
    uint32_t t = 100;
    for (int i = 0; i < P::kMissAfter - 1; ++i) {
        s.fail(t);
        CHECK(!s.missing());
        CHECK(s.due(t + P::kPeriodMs));
        t += P::kPeriodMs;
    }
    s.fail(t);
    CHECK(s.missing());
    // ... then one try every kRetryMs: a chip that NAKs costs nearly nothing.
    CHECK(!s.due(t + P::kRetryMs - 1));
    CHECK(s.due(t + P::kRetryMs));
    CHECK_EQ(s.failures(), static_cast<uint32_t>(P::kMissAfter));
    // The last good value has gone stale by now, and says so.
    CHECK_EQ(s.packMv(t + P::kRetryMs), 0);
    // One good read is back to 10 Hz.
    s.ok(11800, t + P::kRetryMs);
    CHECK(!s.missing());
    CHECK_EQ(s.packMv(t + P::kRetryMs), 11800);
    CHECK(s.due(t + P::kRetryMs + P::kPeriodMs));
}

// A reading no 3S pack can give is a failed read, not a pack voltage.
void testPackSenseRefusesTheImplausible() {
    battguard::PackSense s;
    s.ok(battguard::kPackMaxMv + 4, 0);
    CHECK_EQ(s.packMv(0), 0);
    CHECK_EQ(s.failures(), 1u);
    CHECK_EQ(s.reads(), 0u);
    s.ok(battguard::kPackMaxMv, 100);
    CHECK_EQ(s.packMv(100), battguard::kPackMaxMv);
}

// Benched there is no crouch to ride: a confirmed trip is SAFE on that very
// tick, so the latch's refusals (rail, home, autohome test torqueMustRelease)
// hold from the trip, with no 1.5 s landing window in which the robot could move.
void testBenchTripGoesStraightToSafe() {
    Guard g;
    float now = 0.0f;
    for (int i = 0; i < battguard::kConfirmTicks - 1; ++i) {
        g.benchUpdate(battguard::inputDv(0, 9800), now);
        now += kDt;
    }
    CHECK(!g.latched());                        // not confirmed yet
    g.benchUpdate(battguard::inputDv(0, 9800), now);
    CHECK(g.level() == Level::kSafe);
    CHECK(g.latched());
    CHECK(g.torqueMustRelease());
    // ... and it stays latched when the bench pack rebounds.
    now += kDt;
    for (int i = 0; i < 200; ++i) {
        g.benchUpdate(battguard::inputDv(0, 12000), now);
        now += kDt;
    }
    CHECK(g.torqueMustRelease());
}

// An armed trip disarmed mid-landing finishes at once on the bench.
void testArmedLandingFinishesOnTheBench() {
    Guard g;
    float now = 0.0f;
    feed(g, 98, battguard::kConfirmTicks, now);
    CHECK(g.level() == Level::kLanding);
    CHECK(!g.torqueMustRelease());
    g.benchUpdate(0, now);                      // no reading: still finishes
    CHECK(g.level() == Level::kSafe);
    CHECK(g.torqueMustRelease());
}

// Warn is still a live indicator on the bench, not a latch.
void testBenchWarnClears() {
    Guard g;
    float now = 0.0f;
    for (int i = 0; i < battguard::kConfirmTicks; ++i) {
        g.benchUpdate(battguard::inputDv(0, 10450), now);
        now += kDt;
    }
    CHECK(g.level() == Level::kWarn);
    g.benchUpdate(battguard::inputDv(0, 11500), now);
    CHECK(g.level() == Level::kOk);
    CHECK(!g.latched());
}

// The INA219 is truncated to tenths: it warns below 10.6 V and lands below
// 10.0 V (9.999 V is 99 dV), never above. Pinned so the docs' numbers hold.
void testInaTruncationLines() {
    CHECK_EQ(battguard::inputDv(0, 9999), battguard::kLand3S);
    CHECK(battguard::inputDv(0, 10000) > battguard::kLand3S);
    CHECK_EQ(battguard::inputDv(0, 10599), battguard::kWarn3S);
    CHECK(battguard::inputDv(0, 10600) > battguard::kWarn3S);
}

// The clock wraps (uint32 ms, 49.7 days): still due on time, still fresh.
void testPackSenseWraps() {
    battguard::PackSense s;
    const uint32_t t0 = 0xFFFFFFC0u;
    s.ok(11100, t0);
    CHECK(!s.due(t0 + 50));
    CHECK(s.due(t0 + battguard::PackSense::kPeriodMs));   // past the wrap
    CHECK_EQ(s.packMv(t0 + 500), 11100);
    CHECK_EQ(s.packMv(t0 + 1001), 0);
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
    testIna219BusDecode();
    testDvFromMvTruncates();
    testInputDvTakesTheLower();
    testGuardTripsOnThePackAlone();
    testPackSenseSchedule();
    testPackSenseRefusesTheImplausible();
    testPackSenseWraps();
    testBenchTripGoesStraightToSafe();
    testArmedLandingFinishesOnTheBench();
    testBenchWarnClears();
    testInaTruncationLines();
    return testutil::report("battguard");
}
