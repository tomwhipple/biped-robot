// railguard::Guard -- the power-lite servo rail's state machine (issue #218).
//
// The properties the robot depends on, asserted:
//   * SERVO_EN is low until something deliberately asks, and the bus stays shut
//     until RAIL_V has risen and the servos have had time to boot
//   * no arm while ESTOP_N is high or was never seen low; no rail in time drops
//     SERVO_EN and says so
//   * every drop path takes SERVO_EN low on that call
//   * the E-stop latch: back to RUN powers nothing, and re-arming needs a
//     disabled command after the toggle is back AND a fresh request
//   * a welded K1 raises the stuck fault, at boot too; a normal disarm NEVER
//     does, over the whole range of capacitance, draw and a dead LED2
#include "railguard/railguard.h"

#include <math.h>
#include <stdlib.h>

#include "test_util.h"

using railguard::DropWhy;
using railguard::Fault;
using railguard::Guard;
using railguard::Sample;
using railguard::State;
using railguard::Verdict;

namespace {

constexpr uint32_t kDt = 20;   // ms: ctrl's tick

Sample at(uint32_t now, bool estop_high, uint16_t rail_mv) {
    Sample s;
    s.now_ms = now;
    s.estop_high = estop_high;
    s.rail_valid = true;
    s.rail_mv = rail_mv;
    return s;
}

// `ms` of a constant input; `now` advances.
void feed(Guard& g, uint32_t& now, uint32_t ms, bool estop_high, uint16_t rail_mv) {
    for (uint32_t t = 0; t < ms; t += kDt) {
        g.update(at(now, estop_high, rail_mv));
        now += kDt;
    }
}

// A guard past its boot samples: the toggle at RUN, the rail at 0.
Guard booted(uint32_t& now) {
    Guard g;
    feed(g, now, 200, false, 0);
    CHECK(!g.estopPressed());
    return g;
}

// ... and with the rail up.
Guard railOn(uint32_t& now, uint16_t mv = 12000) {
    Guard g = booted(now);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    feed(g, now, railguard::kServoBootMs + 100, false, mv);
    CHECK(g.busUp());
    return g;
}

void testDividerMatchesTheBoard() {
    // README: 2.21 V at 12.6 V, 1.58 V at 9.0 V.
    CHECK(abs(static_cast<int>(railguard::railMvFromPinMv(2210)) - 12600) < 30);
    CHECK(abs(static_cast<int>(railguard::railMvFromPinMv(1580)) - 9000) < 30);
    // The line, at the pin: 1404 mV, inside the ADC's good range at 11 dB, and the
    // top of the pack is too (150 to 2450 mV).
    CHECK(railguard::kRailOnMv * 10u / railguard::kDividerX10 > 150u);
    CHECK(12600u * 10u / railguard::kDividerX10 < 2450u);
}

void testBootIsOff() {
    Guard g;
    CHECK(!g.servoEn());
    CHECK(!g.busUp());
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kNoReading));
    CHECK(!g.servoEn());
    uint32_t now = 0;
    feed(g, now, 2000, false, 0);
    CHECK(!g.servoEn());   // nobody asked
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
    CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(DropWhy::kNone));
}

void testArmOnlyWithTheEstopReleased() {
    // Before ESTOP_N has been seen low for long enough, it is pressed.
    Guard g;
    uint32_t now = 0;
    g.update(at(now, false, 0));
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kEstopPressed));
    CHECK(!g.servoEn());
    // A floating or unplugged lead reads high: stop, and never armable.
    Guard h;
    now = 0;
    feed(h, now, 3000, true, 0);
    CHECK_EQ(static_cast<int>(h.requestOn()), static_cast<int>(Verdict::kEstopPressed));
    CHECK(!h.servoEn());
    CHECK(h.estopLatched());
}

void testArmSequence() {
    uint32_t now = 0;
    Guard g = booted(now);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    CHECK(g.servoEn());
    CHECK(!g.busUp());
    CHECK_EQ(static_cast<int>(g.state()), static_cast<int>(State::kRising));
    // K1 has not closed yet.
    feed(g, now, 40, false, 0);
    CHECK(!g.busUp());
    // One reading above the line is not enough ...
    g.update(at(now, false, 12000));
    now += kDt;
    CHECK_EQ(static_cast<int>(g.state()), static_cast<int>(State::kRising));
    // ... two is, and the servos then get their boot time before any bus frame.
    g.update(at(now, false, 12000));
    now += kDt;
    CHECK_EQ(static_cast<int>(g.state()), static_cast<int>(State::kSettling));
    CHECK(g.servoEn());
    CHECK(!g.busUp());
    feed(g, now, railguard::kServoBootMs - kDt, false, 12000);
    CHECK(!g.busUp());
    feed(g, now, 2 * kDt, false, 12000);
    CHECK(g.busUp());
    // A second request on a live rail changes nothing.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    CHECK(g.busUp());
}

void testRiseTimeoutDropsAndReports() {
    uint32_t now = 0;
    Guard g = booted(now);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    feed(g, now, railguard::kRiseTimeoutMs - 2 * kDt, false, 3000);
    CHECK(g.servoEn());
    feed(g, now, 3 * kDt, false, 3000);
    CHECK(!g.servoEn());
    CHECK(!g.busUp());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kRiseTimeout));
    CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(DropWhy::kRiseTimeout));
    // The next deliberate arm retries, and clears it.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    CHECK(g.servoEn());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
}

void testRiseIsNotReadFromAMissingReading() {
    uint32_t now = 0;
    Guard g = booted(now);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    for (uint32_t t = 0; t <= railguard::kRiseTimeoutMs; t += kDt) {
        Sample s = at(now, false, 12000);
        s.rail_valid = false;           // rail_mv means nothing
        g.update(s);
        now += kDt;
    }
    CHECK(!g.servoEn());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kRiseTimeout));
    // No reading, no arm.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kNoReading));
}

void testEveryDropPathDropsAtOnce() {
    const DropWhy paths[] = {DropWhy::kDisarm,    DropWhy::kOperatorStop, DropWhy::kLinkRelax,
                             DropWhy::kLinkEstop, DropWhy::kBattSafe,     DropWhy::kFall};
    for (const DropWhy why : paths) {
        uint32_t now = 0;
        Guard g = railOn(now);
        g.drop(why);
        CHECK(!g.servoEn());
        CHECK(!g.busUp());
        CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(why));
        // None of these latches the way the E-stop does: the rail falls, and the
        // next deliberate arm is allowed.
        feed(g, now, 100, false, 0);
        CHECK(!g.servoEn());
        CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
        CHECK(g.servoEn());
    }
    // ... during the rise and the settle as well.
    {
        uint32_t now = 0;
        Guard g = booted(now);
        g.requestOn();
        g.drop(DropWhy::kFall);
        CHECK(!g.servoEn());
        g = booted(now);
        g.requestOn();
        feed(g, now, 100, false, 12000);
        CHECK_EQ(static_cast<int>(g.state()), static_cast<int>(State::kSettling));
        g.drop(DropWhy::kOperatorStop);
        CHECK(!g.servoEn());
    }
    // The first reason stands: the bench handover's disarm that follows a fall's
    // limp must not rewrite why the rail went.
    uint32_t now = 0;
    Guard g = railOn(now);
    g.drop(DropWhy::kFall);
    g.drop(DropWhy::kDisarm);
    CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(DropWhy::kFall));
}

void testLimpReasons() {
    using railguard::limpWhy;
    CHECK_EQ(static_cast<int>(limpWhy(true, true, true)), static_cast<int>(DropWhy::kBattSafe));
    CHECK_EQ(static_cast<int>(limpWhy(false, true, true)), static_cast<int>(DropWhy::kFall));
    CHECK_EQ(static_cast<int>(limpWhy(false, false, true)), static_cast<int>(DropWhy::kLinkRelax));
    CHECK_EQ(static_cast<int>(limpWhy(false, false, false)), static_cast<int>(DropWhy::kLinkEstop));
}

void testEstopDropsAndLatches() {
    uint32_t now = 0;
    Guard g = railOn(now);
    // One high sample is noise.
    g.update(at(now, true, 12000));
    now += kDt;
    CHECK(g.busUp());
    CHECK(!g.estopLatched());
    g.update(at(now, false, 12000));
    now += kDt;
    CHECK(g.busUp());
    // Two in a row is a press: within one tick of the second.
    g.update(at(now, true, 12000));
    now += kDt;
    g.update(at(now, true, 3000));
    CHECK(!g.servoEn());
    CHECK(!g.busUp());
    CHECK(g.estopLatched());
    CHECK(g.estopPressed());
    CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(DropWhy::kEstopInput));
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
}

void testDeliberateRearm() {
    uint32_t now = 0;
    Guard g = railOn(now);
    feed(g, now, 100, true, 0);
    CHECK(g.estopLatched());

    // A disabled command while it is still pressed clears nothing ...
    g.disabledCommand();
    CHECK(g.estopLatched());
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kEstopPressed));

    // ... back to RUN: the rail stays off, however long, and an arm is refused.
    feed(g, now, 5000, false, 0);
    CHECK(!g.estopPressed());
    CHECK(g.estopLatched());
    CHECK(!g.servoEn());
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kEstopLatched));
    CHECK(!g.servoEn());

    // Only just back at RUN does not count as back.
    Guard h = railOn(now);
    feed(h, now, 100, true, 0);
    h.update(at(now, false, 0));
    now += kDt;
    h.disabledCommand();
    CHECK(h.estopLatched());

    // The disabled command clears the latch and powers nothing ...
    g.disabledCommand();
    CHECK(!g.estopLatched());
    feed(g, now, 1000, false, 0);
    CHECK(!g.servoEn());
    // ... the deliberate request does.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    CHECK(g.servoEn());

    // A press and release that a benched robot sits through latches too.
    Guard b = booted(now);
    feed(b, now, 60, true, 0);
    feed(b, now, 200, false, 0);
    CHECK(b.estopLatched());
    CHECK_EQ(static_cast<int>(b.requestOn()), static_cast<int>(Verdict::kEstopLatched));
}

void testRailLost() {
    uint32_t now = 0;
    Guard g = railOn(now);
    // A 12 A get-up at a 9 V pack keeps SERVO+ at 8.66 V or more: nowhere near.
    feed(g, now, 5000, false, 8660);
    CHECK(g.busUp());
    // A dip shorter than the debounce is not a loss.
    feed(g, now, railguard::kRailLostMs - 2 * kDt, false, 3000);
    CHECK(g.busUp());
    feed(g, now, 100, false, 11000);
    CHECK(g.busUp());
    // F1 blows, or the BMS trips.
    feed(g, now, railguard::kRailLostMs + 2 * kDt, false, 200);
    CHECK(!g.servoEn());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kRailLost));
    CHECK_EQ(static_cast<int>(g.lastDrop()), static_cast<int>(DropWhy::kRailLost));
    // Not a welded contact, and the next arm may retry.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
}

// The rail after K1 opens: C through the divider and, while it conducts, R8 + LED2,
// plus a constant servo draw. Integrated rather than closed-form so the draw fits.
struct Decay {
    double c_f, led_ohm, draw_a, v;
    uint16_t step(double dt_s) {
        const double i_div = v / 57000.0;
        const double i_led = (led_ohm > 0.0 && v > 1.9) ? (v - 1.9) / led_ohm : 0.0;
        const double i_srv = v > 1.0 ? draw_a : 0.0;
        v -= (i_div + i_led + i_srv) * dt_s / c_f;
        if (v < 0.0) v = 0.0;
        return static_cast<uint16_t>(v * 1000.0);
    }
};

// A repeatable ADC: the ESP32's is noisy, tens of counts at the pin, so +/-120 mV here.
uint16_t noisy(uint16_t mv, uint32_t& lfsr) {
    lfsr = lfsr * 1664525u + 1013904223u;
    const int n = static_cast<int>((lfsr >> 16) % 241u) - 120;
    const int out = static_cast<int>(mv) + n;
    return static_cast<uint16_t>(out < 0 ? 0 : out);
}

// A disarm; returns how long until the rail crossed the line.
uint32_t disarmDecay(Decay d, bool noise, Guard& g, uint32_t& now) {
    g.drop(DropWhy::kDisarm);
    uint32_t lfsr = 12345u;
    uint32_t below_at = 0;
    for (uint32_t t = 0; t < 200000u; t += kDt) {
        uint16_t mv = d.step(static_cast<double>(kDt) / 1000.0);
        if (noise) mv = noisy(mv, lfsr);
        g.update(at(now, false, mv));
        now += kDt;
        if (d.v * 1000.0 < railguard::kRailOnMv && below_at == 0) below_at = t + kDt;
        if (below_at != 0 && t > below_at + 2000u) break;
    }
    return below_at;
}

void testNormalDisarmIsNeverStuck() {
    // {C, the LED branch (0 = open), servo draw}: every row of railguard.h's table,
    // from a full pack, with and without ADC noise.
    const Decay rows[] = {
        {3.2e-3, 4700.0, 0.24, 12.6},   // 8 servos at 30 mA idle
        {1.2e-3, 4700.0, 0.0, 12.6},    // no servos, C1 +20 %
        {0.8e-3, 4700.0, 0.0, 12.6},    // C1 -20 %
        {3.2e-3, 4700.0, 0.0, 12.6},    // the simulated servo capacitance, no draw
        {1.2e-3, 0.0, 0.0, 12.6},       // LED2 open
        {3.2e-3, 0.0, 0.0, 12.6},       // LED2 open and no draw: the pathological row
    };
    const double bound_s[] = {0.1, 3.5, 3.0, 8.5, 35.0, 90.0};
    int i = 0;
    for (const Decay& d : rows) {
        for (int noise = 0; noise < 2; ++noise) {
            uint32_t now = 0;
            Guard g = railOn(now);
            const uint32_t took = disarmDecay(d, noise != 0, g, now);
            CHECK(took > 0);
            CHECK(static_cast<double>(took) / 1000.0 < bound_s[i]);
            CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
            // Below the line: K1 proved it opened, and the next arm may go.
            CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
        }
        ++i;
    }
}

void testEstopPressDecayIsNotStuck() {
    uint32_t now = 0;
    Guard g = railOn(now);
    Decay d{3.2e-3, 4700.0, 0.0, 12.6};
    uint32_t lfsr = 99u;
    for (uint32_t t = 0; t < 12000u; t += kDt) {
        g.update(at(now, true, noisy(d.step(0.02), lfsr)));
        now += kDt;
    }
    CHECK(g.estopLatched());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
}

void testWeldedContact() {
    // K1 welded: the rail stays at the pack through the disarm.
    uint32_t now = 0;
    Guard g = railOn(now);
    g.drop(DropWhy::kDisarm);
    uint32_t lfsr = 7u;
    uint32_t t = 0;
    for (; t < 60000u && g.fault() == Fault::kNone; t += kDt) {
        g.update(at(now, false, noisy(12000, lfsr)));
        now += kDt;
    }
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kStuck));
    CHECK(t >= railguard::kStuckMs);
    CHECK(t < railguard::kStuckMs + 2000u);   // noise lowers the reference once or twice

    // An arm is refused, and the fault outlives the rail falling.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kStuckFault));
    CHECK(!g.servoEn());
    // `rail clear` is refused while the rail is still up ...
    CHECK(!g.clearFault());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kStuck));
    feed(g, now, 1000, false, 0);
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kStuck));
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kStuckFault));
    // ... and allowed once it has fallen.
    CHECK(g.clearFault());
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
}

void testWeldedWithTheEstopPressed() {
    // The case the check exists for: the E-stop no longer cuts the rail.
    uint32_t now = 0;
    Guard g = railOn(now);
    feed(g, now, railguard::kStuckMs + 200, true, 12000);
    CHECK(!g.servoEn());
    CHECK(g.estopLatched());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kStuck));
}

void testWeldedAtBoot() {
    Guard g;
    uint32_t now = 0;
    feed(g, now, railguard::kStuckMs + 200, false, 11800);
    CHECK(!g.servoEn());
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kStuck));
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kStuckFault));
}

void testRebootMidRunIsNotWelded() {
    // A panic reboot leaves the rail falling at the first sample.
    Guard g;
    uint32_t now = 0;
    Decay d{1.2e-3, 4700.0, 0.0, 12.6};
    for (uint32_t t = 0; t < 12000u; t += kDt) {
        g.update(at(now, false, d.step(0.02)));
        now += kDt;
    }
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
    CHECK(!g.servoEn());
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
}

void testArmWaitsForTheRailToFall() {
    uint32_t now = 0;
    Guard g = railOn(now);
    g.drop(DropWhy::kDisarm);
    feed(g, now, kDt, false, 11000);
    // Still charged: above-the-line would prove nothing about this closing.
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kRailStillUp));
    CHECK(!g.servoEn());
    feed(g, now, kDt, false, 6000);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
}

void testMissingReadingsHoldTheCheck() {
    uint32_t now = 0;
    Guard g = railOn(now);
    g.drop(DropWhy::kDisarm);
    for (uint32_t t = 0; t < railguard::kStuckMs * 2u; t += kDt) {
        Sample s = at(now, false, 12000);
        s.rail_valid = false;
        g.update(s);
        now += kDt;
    }
    CHECK_EQ(static_cast<int>(g.fault()), static_cast<int>(Fault::kNone));
    // A dead ADC on a live rail must not read as a lost rail either.
    Guard h = railOn(now);
    for (uint32_t t = 0; t < 2000u; t += kDt) {
        Sample s = at(now, false, 0);
        s.rail_valid = false;
        h.update(s);
        now += kDt;
    }
    CHECK(h.busUp());
    // The E-stop still works without it.
    feed(h, now, 60, true, 0);
    CHECK(!h.servoEn());
}

void testClockWrap() {
    uint32_t now = 0xFFFFFF00u;
    Guard g;
    feed(g, now, 200, false, 0);
    CHECK_EQ(static_cast<int>(g.requestOn()), static_cast<int>(Verdict::kOk));
    feed(g, now, railguard::kServoBootMs + 100, false, 12000);
    CHECK(g.busUp());
    now = 0xFFFFFF80u;
    Guard h;
    feed(h, now, railguard::kStuckMs + 100, false, 12000);
    CHECK_EQ(static_cast<int>(h.fault()), static_cast<int>(Fault::kStuck));
}

void testNames() {
    CHECK(strcmp(railguard::stateName(State::kOn), "ON") == 0);
    CHECK(strcmp(railguard::dropName(DropWhy::kEstopInput), "E-stop pressed") == 0);
    CHECK(strstr(railguard::faultName(Fault::kStuck), "welded") != nullptr);
    CHECK(strstr(railguard::verdictName(Verdict::kEstopLatched), "disarm") != nullptr);
    for (int i = 0; i <= 3; ++i) CHECK(railguard::stateName(static_cast<State>(i))[0] != '?');
    for (int i = 0; i <= 9; ++i) CHECK(railguard::dropName(static_cast<DropWhy>(i))[0] != '?');
    for (int i = 0; i <= 3; ++i) CHECK(railguard::faultName(static_cast<Fault>(i))[0] != '?');
    for (int i = 0; i <= 5; ++i) CHECK(railguard::verdictName(static_cast<Verdict>(i))[0] != '?');
}

}  // namespace

int main() {
    testDividerMatchesTheBoard();
    testBootIsOff();
    testArmOnlyWithTheEstopReleased();
    testArmSequence();
    testRiseTimeoutDropsAndReports();
    testRiseIsNotReadFromAMissingReading();
    testEveryDropPathDropsAtOnce();
    testLimpReasons();
    testEstopDropsAndLatches();
    testDeliberateRearm();
    testRailLost();
    testNormalDisarmIsNeverStuck();
    testEstopPressDecayIsNotStuck();
    testWeldedContact();
    testWeldedWithTheEstopPressed();
    testWeldedAtBoot();
    testRebootMidRunIsNotWelded();
    testArmWaitsForTheRailToFall();
    testMissingReadingsHoldTheCheck();
    testClockWrap();
    testNames();
    return testutil::report("test_railguard");
}
