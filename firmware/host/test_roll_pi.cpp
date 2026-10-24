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
    c.kp = 1.0f; c.ki = 0.5f; c.ka = 1.0f; c.kh = 0.0f;
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

void testValidate() {
    CHECK(balance::validate(Config{}) == nullptr);
    Config c;
    c.kp = -1.0f;            CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.ki = 50.0f;       CHECK(balance::validate(c) != nullptr);
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
    testValidate();
    return testutil::report("roll_pi");
}
