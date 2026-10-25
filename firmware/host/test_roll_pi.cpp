// balance::RollBalance -- the stance-leg roll PI behind `pose ... B<L|R>`.
//
// The golden rows come from the sim twin, experiments/one-leg-stand/balance.py
// (`balance.py` with no arguments prints them): RollPI(kp 1.0, ki 0.5,
// clamp 6 deg) at dt 0.02. The rest are the properties the bench depends on.
#include "balance/roll_pi.h"

#include <math.h>

#include "test_util.h"

using balance::Config;
using balance::RollBalance;
using balance::State;

namespace {

constexpr float kDeg = 0.017453293f;

Config golden() {
    Config c;
    c.kp = 1.0f; c.ki = 0.5f; c.kd = 0.0f; c.ka = 1.0f; c.kh = 0.0f;
    c.clamp_rad = 6.0f * kDeg; c.abort_rad = 30.0f * kDeg;
    return c;
}

void testMatchesSimTwin() {
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    struct Row { float err, u, i; };
    const Row rows[] = {
        {0.02f, 0.020000f, 0.000400f},  {0.05f, 0.050200f, 0.001400f},
        {0.20f, 0.104720f, 0.001400f},  {0.20f, 0.104720f, 0.001400f},
        {0.20f, 0.104720f, 0.001400f},  {-0.10f, -0.099300f, -0.000600f},
        {-0.10f, -0.100300f, -0.002600f},
    };
    for (const Row& r : rows) {
        b.update(r.err, 0.02f);
        CHECK_NEAR(b.u(), r.u, 2e-5);
        CHECK_NEAR(b.integral(), r.i, 2e-5);
    }
}

Config goldenKd() {
    Config c = golden();
    c.kd = 0.1f;
    return c;
}

// Rows from balance.py (the second block it prints): RollPI(1.0, 0.5, 6 deg, kd 0.1).
void testGyroMatchesSimTwin() {
    RollBalance b;
    b.configure(goldenKd());
    b.engage(0.0f);
    struct Row { float err, rate, u, i; };
    const Row rows[] = {
        {0.02f, 0.0f, 0.020000f, 0.000400f},  {0.02f, 0.5f, 0.070200f, 0.000800f},
        {0.02f, -0.5f, -0.029600f, 0.001200f}, {0.0f, 0.3f, 0.030600f, 0.001200f},
        {0.1f, 0.4f, 0.104720f, 0.001200f},   {0.1f, 0.4f, 0.104720f, 0.001200f},
        {-0.05f, -2.0f, -0.104720f, 0.001200f},
    };
    for (const Row& r : rows) {
        b.update(r.err, 0.02f, r.rate);
        CHECK_NEAR(b.u(), r.u, 2e-5);
        CHECK_NEAR(b.integral(), r.i, 2e-5);
    }
}

void testRateOnlyResponseAndSign() {
    // Zero error, the roll rate alone: u has the rate's sign (same as kp*err, so a tip
    // toward + roll that is still accelerating gets the + correction), scaled by kd,
    // and the integral does not move (err is 0).
    RollBalance b;
    b.configure(goldenKd());
    b.engage(0.0f);
    b.update(0.0f, 0.02f, 0.3f);
    CHECK_NEAR(b.u(), 0.03, 1e-7);
    CHECK_NEAR(b.integral(), 0.0, 1e-9);
    CHECK_NEAR(b.rate(), 0.3, 1e-7);
    b.update(0.0f, 0.02f, -0.3f);
    CHECK_NEAR(b.u(), -0.03, 1e-7);
    // kd 0 ignores the rate entirely; the 2-argument call is the PI alone
    RollBalance p, q;
    p.configure(golden()); q.configure(golden());
    p.engage(0.0f); q.engage(0.0f);
    p.update(0.03f, 0.02f, 5.0f);
    q.update(0.03f, 0.02f);
    CHECK_NEAR(p.u(), q.u(), 1e-9);
}

void testRateAndClampInteraction() {
    // The clamp applies to the SUM, and a rate that saturates the output freezes the
    // integrator when the error is on the same side (no windup behind the D term).
    RollBalance b;
    Config c = goldenKd();
    b.configure(c);
    b.engage(0.0f);
    for (int i = 0; i < 100; ++i) b.update(0.02f, 0.02f, 3.0f);   // kd*w = 0.3 rad >> clamp
    CHECK_NEAR(b.u(), c.clamp_rad, 1e-6);
    CHECK(b.integral() < 0.01f);
    // rate reverses: the output leaves the clamp on the next tick
    b.update(0.02f, 0.02f, -3.0f);
    CHECK(b.u() < 0.0f);
    // opposite-sign rate does not freeze the integral when unsaturated
    RollBalance d;
    d.configure(c);
    d.engage(0.0f);
    d.update(0.05f, 0.02f, -0.1f);
    CHECK_NEAR(d.integral(), 0.05 * 0.02, 1e-7);
}

void testNonFiniteRateFreezes() {
    RollBalance b;
    b.configure(goldenKd());
    b.engage(0.0f);
    b.update(0.05f, 0.02f, 0.1f);
    const float u = b.u();
    CHECK(b.update(0.06f, 0.02f, NAN) == State::kFrozenImu);
    CHECK_NEAR(b.u(), u, 1e-9);
    RollBalance c;
    c.configure(goldenKd());
    c.engage(0.0f);
    c.update(0.05f, 0.02f, 0.1f);
    CHECK(c.update(0.06f, 0.02f, INFINITY) == State::kFrozenImu);
    // the abort limit still wins on a finite rate
    RollBalance d;
    d.configure(goldenKd());
    d.engage(0.0f);
    CHECK(d.update(0.7f, 0.02f, 0.1f) == State::kFrozenAbort);
}

void testMirrorSymmetryWithRate() {
    RollBalance l, r;
    l.configure(goldenKd()); r.configure(goldenKd());
    l.engage(0.0f); r.engage(0.0f);
    const float e[] = {0.01f, 0.03f, 0.05f, 0.02f, -0.01f};
    const float w[] = {0.1f, 0.4f, 0.2f, -0.3f, -0.1f};
    for (int k = 0; k < 5; ++k) {
        l.update(e[k], 0.02f, w[k]);
        r.update(-e[k], 0.02f, -w[k]);
        CHECK_NEAR(l.u(), -r.u(), 1e-9);
    }
}

void testReferenceIsCapturedNotAbsolute() {
    // The IMU's absolute roll zero is unknown: only the change since engage matters.
    RollBalance a, b;
    a.configure(golden()); b.configure(golden());
    a.engage(0.0f);  a.update(0.03f, 0.02f);
    b.engage(0.40f); b.update(0.43f, 0.02f);     // same lean from a different zero
    CHECK_NEAR(a.u(), b.u(), 1e-6);
    CHECK_NEAR(a.err(), 0.03, 1e-6);
    // engaged with no change: no correction at all
    RollBalance c;
    c.configure(golden());
    c.engage(0.25f);
    c.update(0.25f, 0.02f);
    CHECK_NEAR(c.u(), 0.0, 1e-9);
}

void testOutputClampAndAntiWindup() {
    RollBalance b;
    Config c = golden();
    c.abort_rad = 40.0f * kDeg;
    b.configure(c);
    b.engage(0.0f);
    for (int i = 0; i < 500; ++i) b.update(0.2f, 0.02f);     // 11 deg held for 10 s
    CHECK_NEAR(b.u(), c.clamp_rad, 1e-6);
    CHECK(b.integral() < 0.01f);                  // did not wind up while saturated
    // the error reverses: the output leaves the clamp at once, not after unwinding
    b.update(-0.05f, 0.02f);
    CHECK(b.u() < 0.0f);
    // symmetric on the other side
    RollBalance n;
    n.configure(c);
    n.engage(0.0f);
    for (int i = 0; i < 500; ++i) n.update(-0.2f, 0.02f);
    CHECK_NEAR(n.u(), -c.clamp_rad, 1e-6);
}

void testIntegralRemovesSteadyError() {
    RollBalance b;
    Config c;                       // defaults
    b.configure(c);
    b.engage(0.0f);
    b.update(0.02f, 0.02f);
    const float u0 = b.u();
    for (int i = 0; i < 100; ++i) b.update(0.02f, 0.02f);    // a held 1.1 deg error
    CHECK(b.u() > u0);              // the I term keeps pushing while the error persists
}

void testOutputMix() {
    RollBalance b;
    Config c = golden();
    c.ka = 1.0f; c.kh = -0.5f;
    b.configure(c);
    b.engage(0.0f);
    b.update(0.04f, 0.02f);
    CHECK_NEAR(b.ankle(), b.u(), 1e-9);
    CHECK_NEAR(b.hip(), -0.5 * b.u(), 1e-9);
}

void testAbortFreezesAtLastCorrection() {
    RollBalance b;
    b.configure(golden());                          // abort 30 deg
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    const float u = b.u();
    CHECK(b.state() == State::kActive);
    CHECK(b.update(0.7f, 0.02f) == State::kFrozenAbort);     // 40 deg
    CHECK_NEAR(b.u(), u, 1e-9);                     // held, not zeroed, not extrapolated
    b.update(0.02f, 0.02f);                         // back inside the limit: stays frozen
    CHECK(b.state() == State::kFrozenAbort);
    CHECK_NEAR(b.u(), u, 1e-9);
    CHECK(b.engaged());
}

void testImuFaultFreezes() {
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    const float u = b.u();
    b.imuFault();
    CHECK(b.state() == State::kFrozenImu);
    b.update(0.01f, 0.02f);
    CHECK_NEAR(b.u(), u, 1e-9);
    // a fault on an idle controller does not arm it
    RollBalance idle;
    idle.imuFault();
    CHECK(idle.state() == State::kOff);
}

void testNanFreezes() {
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    const float u = b.u();
    CHECK(b.update(NAN, 0.02f) == State::kFrozenAbort);
    CHECK_NEAR(b.u(), u, 1e-9);
}

void testDisengageZeroes() {
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    b.disengage();
    CHECK(b.state() == State::kOff);
    CHECK_NEAR(b.u(), 0.0, 1e-12);
    CHECK_NEAR(b.integral(), 0.0, 1e-12);
    b.update(0.05f, 0.02f);                          // ignored while off
    CHECK_NEAR(b.u(), 0.0, 1e-12);
    b.engage(0.1f);                                  // a re-engage starts clean
    CHECK_NEAR(b.u(), 0.0, 1e-12);
    CHECK_NEAR(b.ref(), 0.1, 1e-6);
}

void testMirrorSymmetry() {
    // Same code serves either stance: u is odd in the error.
    RollBalance l, r;
    l.configure(golden()); r.configure(golden());
    l.engage(0.0f); r.engage(0.0f);
    const float seq[] = {0.01f, 0.03f, 0.05f, 0.02f, -0.01f};
    for (float e : seq) {
        l.update(e, 0.02f);
        r.update(-e, 0.02f);
        CHECK_NEAR(l.u(), -r.u(), 1e-9);
        CHECK_NEAR(l.hip(), -r.hip(), 1e-9);
    }
}

void testRollFromUp() {
    float roll = 9.0f;
    const float level[3] = {0.0f, 0.0f, 1.0f};
    CHECK(balance::rollFromUp(level, &roll));
    CHECK_NEAR(roll, 0.0, 1e-7);
    // imu.h: roll 20 deg gives up = (0, -0.342, 0.940); + = toward the robot's right
    const float r20[3] = {0.0f, -0.34202f, 0.93969f};
    CHECK(balance::rollFromUp(r20, &roll));
    CHECK_NEAR(roll, 20.0 * kDeg, 1e-4);
    const float l10[3] = {0.0f, 0.17365f, 0.98481f};
    CHECK(balance::rollFromUp(l10, &roll));
    CHECK_NEAR(roll, -10.0 * kDeg, 1e-4);
    const float fallen[3] = {0.0f, -0.9f, 0.3f};
    CHECK(!balance::rollFromUp(fallen, &roll));
    const float bad[3] = {0.0f, NAN, 1.0f};
    CHECK(!balance::rollFromUp(bad, &roll));
}

void testDefaultsAreSimChosen() {
    // experiments/one-leg-stand/2026-10-25-gyro-damping-sim.md
    const Config c;
    CHECK_NEAR(c.kp, 0.3, 1e-6);
    CHECK_NEAR(c.ki, 1.0, 1e-6);
    CHECK_NEAR(c.kd, 0.03, 1e-6);
    CHECK_NEAR(c.ka, 1.0, 1e-6);
    CHECK_NEAR(c.kh, -1.0, 1e-6);
}

void testValidate() {
    CHECK(balance::validate(Config{}) == nullptr);
    Config c;
    c.kp = -1.0f;            CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.ki = 50.0f;       CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.kd = -0.1f;       CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.kd = 2.0f;        CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.kd = NAN;         CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.ka = 9.0f;        CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.kh = -9.0f;       CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.clamp_rad = 0.0f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.clamp_rad = 0.5f; CHECK(balance::validate(c) != nullptr);
    // what the CLI produces for `balance set ... 10` and `... 15` (deg / 57.29578f)
    c = Config{}; c.clamp_rad = 10.0f / 57.29578f; CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.clamp_rad = 15.0f / 57.29578f; CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.clamp_rad = 15.5f / 57.29578f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_rad = 0.01f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.kp = NAN;         CHECK(balance::validate(c) != nullptr);
}

}  // namespace

int main() {
    testMatchesSimTwin();
    testGyroMatchesSimTwin();
    testRateOnlyResponseAndSign();
    testRateAndClampInteraction();
    testNonFiniteRateFreezes();
    testMirrorSymmetryWithRate();
    testReferenceIsCapturedNotAbsolute();
    testOutputClampAndAntiWindup();
    testIntegralRemovesSteadyError();
    testOutputMix();
    testAbortFreezesAtLastCorrection();
    testImuFaultFreezes();
    testNanFreezes();
    testDisengageZeroes();
    testMirrorSymmetry();
    testRollFromUp();
    testDefaultsAreSimChosen();
    testValidate();
    return testutil::report("roll_pi");
}
