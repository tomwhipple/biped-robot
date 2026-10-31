// balance::RollBalance -- the stance-leg roll PI behind `pose ... B<L|R>`.
//
// The golden rows come from the sim twin, experiments/one-leg-stand/balance.py
// (`balance.py` with no arguments prints them): RollPI(kp 1.0, ki 0.5,
// clamp 6 deg) at dt 0.02. The rest are the properties the bench depends on.
#include "balance/roll_pi.h"

#include <math.h>
#include <string.h>

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

void testNonFiniteRateAborts() {
    RollBalance b;
    b.configure(goldenKd());
    b.engage(0.0f);
    b.update(0.05f, 0.02f, 0.1f);
    const float u = b.u();
    CHECK(b.update(0.06f, 0.02f, NAN) == State::kAbortImu);
    CHECK_NEAR(b.u(), u, 1e-9);
    RollBalance c;
    c.configure(goldenKd());
    c.engage(0.0f);
    c.update(0.05f, 0.02f, 0.1f);
    CHECK(c.update(0.06f, 0.02f, INFINITY) == State::kAbortImu);
    // the abort limit still wins on a finite rate
    RollBalance d;
    d.configure(goldenKd());
    d.engage(0.0f);
    CHECK(d.update(0.7f, 0.02f, 0.1f) == State::kAbortTip);
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

// -- the abort sequence (experiments/one-leg-stand/balance_abort.py, SEQ policy) ----------------

void testAbortProfileMatchesSimTwin() {
    // balance_abort.py golden: decay 0.3 s, land 1.0 s
    const Config c;
    struct Row { float t, us, lf; };
    const Row rows[] = {
        {0.0f, 1.000000f, 0.000000f}, {0.1f, 0.790123f, 0.000000f}, {0.3f, 0.000000f, 0.000000f},
        {0.5f, 0.000000f, 0.057920f}, {1.0f, 0.000000f, 0.836920f}, {1.3f, 0.000000f, 1.000000f},
        {2.0f, 0.000000f, 1.000000f},
    };
    for (const Row& r : rows) {
        float us = -1.0f, lf = -1.0f;
        balance::abortProfile(c, r.t, &us, &lf);
        CHECK_NEAR(us, r.us, 2e-5);
        CHECK_NEAR(lf, r.lf, 2e-5);
    }
    CHECK_NEAR(c.abort_decay_s, 0.3, 1e-6);
    CHECK_NEAR(c.abort_land_s, 1.0, 1e-6);
}

// Run an aborted controller at 50 Hz for `secs`; returns u's trace in `u` (size n).
void runAbort(RollBalance& b, int n, float* u, float* land) {
    for (int k = 0; k < n; ++k) {
        b.abortTick(0.02f);
        u[k] = b.u();
        land[k] = b.landFraction();
    }
}

void checkSequence(RollBalance& b, float u0) {
    // fades monotonically to exactly 0 within abort_decay_s, lands only after that, reaches 1
    float u[100], land[100];
    runAbort(b, 100, u, land);                       // 2 s
    float prev = u0;
    for (int k = 0; k < 100; ++k) {
        const float t = 0.02f * static_cast<float>(k + 1);
        CHECK(fabsf(u[k]) <= fabsf(prev) + 1e-9f);   // never grows
        CHECK(u[k] * u0 >= -1e-12f);                 // never changes sign
        prev = u[k];
        if (t >= 0.3f + 1e-4f) CHECK_NEAR(u[k], 0.0, 1e-9);
        if (fabsf(u[k]) > 1e-9f) CHECK_NEAR(land[k], 0.0, 1e-9);     // no landing while u is still on
        if (k > 0) CHECK(land[k] >= land[k - 1]);
        if (t >= 1.3f + 1e-4f) CHECK_NEAR(land[k], 1.0, 1e-9);
    }
    CHECK(b.landed());
    CHECK(b.engaged());                              // torque is never released: still engaged, not kOff
    CHECK(b.state() != State::kOff);
}

void testAbortOnTipBothSides() {
    // outward and inward are the same code: the error's sign only sets the sign of the frozen u
    for (int side = -1; side <= 1; side += 2) {
        RollBalance b;
        b.configure(golden());                       // abort 30 deg
        b.engage(0.0f);
        b.update(0.05f * static_cast<float>(side), 0.02f);
        const float u = b.u();
        CHECK(b.state() == State::kActive);
        CHECK(!b.aborted());
        CHECK(b.update(0.7f * static_cast<float>(side), 0.02f) == State::kAbortTip);   // 40 deg
        CHECK(b.aborted());
        CHECK_NEAR(b.u(), u, 1e-9);                  // the abort tick itself still holds u
        CHECK_NEAR(b.abortU(), u, 1e-9);
        CHECK_NEAR(b.abortErr(), 0.7 * side, 1e-6);
        CHECK_NEAR(b.landFraction(), 0.0, 1e-9);
        // the roll coming back inside the limit (or a wild value) changes nothing: no decisions
        b.update(0.01f, 0.02f);
        b.update(-0.7f * static_cast<float>(side), 0.02f, 9.0f);
        CHECK(b.state() == State::kAbortTip);
        CHECK_NEAR(b.abortU(), u, 1e-9);
        CHECK_NEAR(b.abortErr(), 0.7 * side, 1e-6);
        RollBalance c = b;                           // continue from after those two ticks
        checkSequence(c, u);
    }
}

void testAbortOnImuFault() {
    // a failed attitude read, and a non-finite rate: no attitude is needed to run the sequence
    RollBalance a;
    a.configure(golden());
    a.engage(0.0f);
    a.update(0.05f, 0.02f);
    const float u = a.u();
    a.imuFault();
    CHECK(a.state() == State::kAbortImu);
    CHECK_NEAR(a.abortU(), u, 1e-9);
    CHECK(!a.landed());
    checkSequence(a, u);
    RollBalance r;
    r.configure(golden());
    r.engage(0.0f);
    r.update(0.05f, 0.02f);
    CHECK(r.update(0.05f, 0.02f, NAN) == State::kAbortImu);
    CHECK_NEAR(r.abortU(), r.u(), 1e-9);
    // update() with garbage after the fault only advances the sequence
    for (int k = 0; k < 100; ++k) r.update(NAN, 0.02f, NAN);
    CHECK(r.state() == State::kAbortImu);
    CHECK(r.landed());
    CHECK_NEAR(r.u(), 0.0, 1e-9);
    // a fault on an idle controller does not arm it
    RollBalance idle;
    idle.imuFault();
    CHECK(idle.state() == State::kOff);
    CHECK(!idle.abort(State::kAbortTip));
    CHECK(idle.state() == State::kOff);
}

void testAbortIsIdempotent() {
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    const float u = b.u();
    CHECK(b.abort(State::kAbortTip));
    for (int k = 0; k < 10; ++k) b.abortTick(0.02f);
    const float since = b.abortSince();
    const float ucur = b.u();
    CHECK(since > 0.15f);
    // a second abort of either cause: refused, nothing restarts (the sequence does not rewind)
    CHECK(!b.abort(State::kAbortTip));
    CHECK(!b.abort(State::kAbortImu));
    b.imuFault();
    b.update(0.9f, 0.02f, NAN);
    CHECK(b.state() == State::kAbortTip);            // the first cause stands
    CHECK_NEAR(b.abortU(), u, 1e-9);
    CHECK_NEAR(b.abortSince(), since + 0.02, 1e-6);  // only the update's tick advanced it
    CHECK(fabsf(b.u()) <= fabsf(ucur) + 1e-9f);
    // abortTick on a non-aborted controller is a no-op
    RollBalance live;
    live.configure(golden());
    live.engage(0.0f);
    live.update(0.05f, 0.02f);
    const float ul = live.u();
    live.abortTick(1.0f);
    CHECK(live.state() == State::kActive);
    CHECK_NEAR(live.u(), ul, 1e-9);
    // disengage ends it; a re-engage starts a fresh active loop
    b.disengage();
    CHECK(b.state() == State::kOff);
    CHECK(!b.engaged());
    b.engage(0.0f);
    CHECK(b.state() == State::kActive);
    CHECK_NEAR(b.abortSince(), 0.0, 1e-12);
    CHECK_NEAR(b.u(), 0.0, 1e-12);
}

void testLandingReachesStartPose() {
    // what poseTick does per joint: landTarget(from, start, landFraction) every 20 ms
    RollBalance b;
    b.configure(golden());
    b.engage(0.0f);
    b.update(0.05f, 0.02f);
    b.update(0.7f, 0.02f);
    const int32_t from[3] = {2140, 1900, 2048};      // nominal targets at the abort (lift pose)
    const int32_t start[3] = {2100, 2000, 2048};     // where the B stream began
    int32_t last[3] = {from[0], from[1], from[2]};
    int ticks = 0;
    while (!b.landed() && ticks < 500) {
        b.abortTick(0.02f);
        ++ticks;
        for (int j = 0; j < 3; ++j) {
            const int32_t w = balance::landTarget(from[j], start[j], b.landFraction());
            // monotone from `from` toward `start`, never past it
            const int32_t dir = start[j] >= from[j] ? 1 : -1;
            CHECK((w - last[j]) * dir >= 0);
            CHECK((start[j] - w) * dir >= 0);
            last[j] = w;
        }
    }
    CHECK(b.landed());
    CHECK(ticks >= 64 && ticks <= 66);              // 0.3 + 1.0 s at 50 Hz
    for (int j = 0; j < 3; ++j) CHECK(balance::landTarget(from[j], start[j], b.landFraction()) == start[j]);
    CHECK(last[0] == start[0] && last[1] == start[1] && last[2] == start[2]);
    // the correction is gone when it lands: the joints sit exactly on the pre-lift pose
    CHECK_NEAR(b.ankle(), 0.0, 1e-12);
    CHECK_NEAR(b.hip(), 0.0, 1e-12);
    // before the correction is gone the pose has not moved at all
    CHECK(balance::landTarget(from[0], start[0], 0.0f) == from[0]);
    // already at the start pose: stays there
    CHECK(balance::landTarget(2100, 2100, 0.5f) == 2100);
}

void testContinues() {
    using balance::continues;
    const bool all[4] = {true, true, true, true};
    const bool some[4] = {true, false, true, true};
    // same side, still active, same joints: the chain goes on (I and u kept)
    CHECK(continues(State::kActive, 'L', 'L', all, all, 4));
    CHECK(continues(State::kActive, 'R', 'R', all, all, 4));
    CHECK(continues(State::kArmed, 'L', 'L', all, all, 4));      // a new move before the lift ended
    CHECK(!continues(State::kArmed, 'L', 'R', all, all, 4));
    // a subset of the stream is fine; a joint the stream was not moving is not
    CHECK(continues(State::kActive, 'L', 'L', all, some, 4));
    CHECK(!continues(State::kActive, 'L', 'L', some, all, 4));
    // the other stance, an idle controller or one already aborting: fresh engage
    CHECK(!continues(State::kActive, 'L', 'R', all, all, 4));
    CHECK(!continues(State::kOff, 0, 'L', all, all, 4));
    CHECK(!continues(State::kAbortTip, 'L', 'L', all, all, 4));
    CHECK(!continues(State::kAbortImu, 'R', 'R', all, all, 4));
}

void testArmedWatchesOnlyThenStarts() {
    RollBalance b;
    b.configure(golden());
    b.arm(0.01f);
    CHECK(b.state() == State::kArmed);
    CHECK(b.engaged() && b.armed() && !b.aborted());
    // the lift's false tilt: no correction, no integration
    for (int i = 0; i < 100; ++i) b.update(0.01f + 0.06f, 0.02f, 0.1f);
    CHECK_NEAR(b.u(), 0.0, 1e-12);
    CHECK_NEAR(b.integral(), 0.0, 1e-12);
    CHECK_NEAR(b.err(), 0.06, 1e-6);                // reported against the pre-lift roll
    CHECK(b.state() == State::kArmed);
    // the move ends: the reference is taken there and the PI runs from zero
    CHECK(b.start(0.07f));
    CHECK(b.state() == State::kActive);
    CHECK_NEAR(b.ref(), 0.07, 1e-6);
    CHECK_NEAR(b.u(), 0.0, 1e-12);
    b.update(0.07f, 0.02f);
    CHECK_NEAR(b.u(), 0.0, 1e-9);                   // sitting at the new reference: nothing to do
    b.update(0.09f, 0.02f);
    CHECK(b.u() > 0.0f);
    // start() only from ARMED
    CHECK(!b.start(0.0f));
    CHECK_NEAR(b.ref(), 0.07, 1e-6);
    RollBalance off;
    CHECK(!off.start(0.0f));
    CHECK(off.state() == State::kOff);
}

void testArmedStartsOnARealLean() {
    // both sides: a lean past arm_start_rad starts the PI there, the next tick corrects
    const float sides[2] = {1.0f, -1.0f};
    for (float s : sides) {
        RollBalance b;
        Config c = golden();
        c.arm_start_rad = 4.0f * kDeg;
        b.configure(c);
        b.arm(0.0f);
        b.update(s * 3.9f * kDeg, 0.02f);
        CHECK(b.armed());
        CHECK_NEAR(b.u(), 0.0, 1e-12);
        CHECK(b.update(s * 4.2f * kDeg, 0.02f) == State::kActive);
        CHECK_NEAR(b.ref(), s * 4.2f * kDeg, 1e-6);
        CHECK_NEAR(b.u(), 0.0, 1e-12);
        CHECK_NEAR(b.integral(), 0.0, 1e-12);
        b.update(s * 5.0f * kDeg, 0.02f);
        CHECK(b.u() * s > 0.0f);                    // pushes back against the lean
        CHECK(!b.start(0.0f));                      // already running: the move end does not re-reference
    }
    // 0 = only at the move end
    RollBalance n;
    Config c = golden();
    c.arm_start_rad = 0.0f;
    n.configure(c);
    n.arm(0.0f);
    n.update(0.3f, 0.02f);
    CHECK(n.armed());
    // must stay below the abort
    c = Config{}; c.arm_start_rad = 0.4f;   CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.arm_start_rad = -0.01f; CHECK(balance::validate(c) != nullptr);
    CHECK(balance::validate(Config{}) == nullptr);
}

void testArmedAborts() {
    // tip: past the limit from the pre-lift roll, both sides, u was 0
    const float sides[2] = {1.0f, -1.0f};
    for (float s : sides) {
        RollBalance b;
        Config c = golden();                        // abort 30 deg
        c.arm_start_rad = 0.0f;                     // stays armed up to the abort
        b.configure(c);
        b.arm(0.0f);
        b.update(s * 0.3f, 0.02f);
        CHECK(b.armed());
        CHECK(b.update(s * 0.6f, 0.02f) == State::kAbortTip);
        CHECK_NEAR(b.abortU(), 0.0, 1e-12);
        CHECK(!b.start(0.0f));                      // an aborted loop never starts
        int n = 0;
        while (!b.landed() && n < 500) { b.abortTick(0.02f); ++n; }
        CHECK(b.landed());
    }
    RollBalance i;
    i.configure(golden());
    i.arm(0.0f);
    i.imuFault();
    CHECK(i.state() == State::kAbortImu);
    RollBalance r;
    r.configure(golden());
    r.arm(0.0f);
    CHECK(r.update(0.0f, 0.02f, NAN) == State::kAbortImu);
}

void testAbortPhaseNames() {
    RollBalance b;
    b.configure(golden());
    CHECK(strcmp(balance::abortPhase(b), "") == 0);
    b.engage(0.0f);
    CHECK(strcmp(balance::abortPhase(b), "") == 0);
    b.update(0.7f, 0.02f);
    CHECK(strcmp(balance::abortPhase(b), "fading") == 0);
    for (int k = 0; k < 17; ++k) b.abortTick(0.02f);
    CHECK(strcmp(balance::abortPhase(b), "landing") == 0);
    for (int k = 0; k < 70; ++k) b.abortTick(0.02f);
    CHECK(strcmp(balance::abortPhase(b), "landed") == 0);
    CHECK(strstr(balance::stateName(State::kAbortTip), "ABORT") != nullptr);
    CHECK(strstr(balance::stateName(State::kAbortImu), "IMU") != nullptr);
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

// Rows from balance.py (the third block it prints): RollPI(kp 0.3, ki 1.0, clamp 10 deg,
// i_lim 3 deg) at dt 0.5 -- the integral stops at i_lim/ki and unwinds from there at once.
void testIntegralLimitMatchesSimTwin() {
    Config c;
    c.kp = 0.3f; c.ki = 1.0f; c.kd = 0.0f; c.ka = 1.0f; c.kh = -1.0f;
    c.clamp_rad = 10.0f * kDeg; c.abort_rad = 20.0f * kDeg; c.i_lim_rad = 3.0f * kDeg;
    RollBalance b;
    b.configure(c);
    b.engage(0.0f);
    struct Row { float err, u, i; };
    const Row rows[] = {
        {0.05f, 0.015000f, 0.025000f},  {0.05f, 0.040000f, 0.050000f},
        {0.05f, 0.065000f, 0.052360f},  {0.05f, 0.067360f, 0.052360f},
        {0.05f, 0.067360f, 0.052360f},  {0.05f, 0.067360f, 0.052360f},
        {-0.02f, 0.046360f, 0.042360f}, {-0.02f, 0.036360f, 0.032360f},
    };
    for (const Row& r : rows) {
        b.update(r.err, 0.5f);
        CHECK_NEAR(b.u(), r.u, 2e-5);
        CHECK_NEAR(b.integral(), r.i, 2e-5);
    }
}

// The limit is symmetric, scales with ki, and 0 turns it off (the pre-10-30 loop).
void testIntegralLimitProperties() {
    const float sgns[] = {1.0f, -1.0f};
    for (float sgn : sgns) {
        Config c;                                     // defaults: kp 0.3 ki 1.0, I limit 3 deg
        c.ki = 2.0f;
        RollBalance b;
        b.configure(c);
        b.engage(0.0f);
        for (int k = 0; k < 400; ++k) b.update(sgn * 2.0f * kDeg, 0.025f);
        CHECK_NEAR(b.integral() * c.ki, sgn * c.i_lim_rad, 1e-6);
        CHECK_NEAR(b.u(), sgn * (c.kp * 2.0f * kDeg + c.i_lim_rad), 1e-5);
        c.i_lim_rad = 0.0f;                           // off: the integral runs to the output clamp
        RollBalance f;
        f.configure(c);
        f.engage(0.0f);
        for (int k = 0; k < 400; ++k) f.update(sgn * 2.0f * kDeg, 0.025f);
        CHECK_NEAR(f.u(), sgn * c.clamp_rad, 1e-6);
    }
}

void testDefaultsAreSimChosen() {
    // experiments/one-leg-stand/2026-10-25-gyro-damping-sim.md
    const Config c;
    CHECK_NEAR(c.kp, 0.3, 1e-6);
    CHECK_NEAR(c.ki, 1.0, 1e-6);
    CHECK_NEAR(c.kd, 0.03, 1e-6);
    CHECK_NEAR(c.ka, 1.0, 1e-6);
    CHECK_NEAR(c.kh, -1.0, 1e-6);
    CHECK_NEAR(c.i_lim_rad, 3.0 * kDeg, 1e-5);       // 2026-10-30-sim-fit.md
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
    // what the CLI produces for `balance set ... <abort> 3` and `... 20` (deg / 57.29578f): the ends pass
    c = Config{}; c.abort_rad = 3.0f / 57.29578f; CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.abort_rad = 20.0f / 57.29578f; CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.abort_rad = 2.9f / 57.29578f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_decay_s = 0.0f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_decay_s = 3.0f; CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_land_s = 0.1f;  CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_land_s = NAN;   CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.abort_land_s = 0.4f;  CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.kp = NAN;         CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.i_lim_rad = 0.0f;           CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.i_lim_rad = 15.0f * kDeg;   CHECK(balance::validate(c) == nullptr);
    c = Config{}; c.i_lim_rad = -0.01f;         CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.i_lim_rad = 16.0f * kDeg;   CHECK(balance::validate(c) != nullptr);
    c = Config{}; c.i_lim_rad = NAN;            CHECK(balance::validate(c) != nullptr);
}

}  // namespace

int main() {
    testMatchesSimTwin();
    testGyroMatchesSimTwin();
    testRateOnlyResponseAndSign();
    testRateAndClampInteraction();
    testNonFiniteRateAborts();
    testMirrorSymmetryWithRate();
    testReferenceIsCapturedNotAbsolute();
    testOutputClampAndAntiWindup();
    testIntegralRemovesSteadyError();
    testOutputMix();
    testAbortProfileMatchesSimTwin();
    testAbortOnTipBothSides();
    testAbortOnImuFault();
    testAbortIsIdempotent();
    testLandingReachesStartPose();
    testAbortPhaseNames();
    testContinues();
    testArmedWatchesOnlyThenStarts();
    testArmedAborts();
    testArmedStartsOnARealLean();
    testDisengageZeroes();
    testMirrorSymmetry();
    testRollFromUp();
    testIntegralLimitMatchesSimTwin();
    testIntegralLimitProperties();
    testDefaultsAreSimChosen();
    testValidate();
    return testutil::report("roll_pi");
}
