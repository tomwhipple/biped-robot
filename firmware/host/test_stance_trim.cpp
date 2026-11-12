// balance::StanceTrim -- the stance-hip roll trim behind `pose ... S<L|R>` (#297 step 2).
//
// The golden rows come from the Python twin, sim/stance_trim.py (`python sim/stance_trim.py` prints
// them): default gains (kp 0.25, ki 1.0, lim 15, tlim 14, fade 0.6 s, abort 15 deg), dt 0.02. The rest
// are the properties the bench and the stepper depend on.
#include "balance/stance_trim.h"

#include <math.h>
#include <string.h>

#include "test_util.h"

using balance::StanceTrim;
using balance::TrimConfig;
using balance::TrimState;

namespace {

constexpr float kDeg = 0.017453293f;

struct Row {
    const char* ev;
    float roll, ramp;
    int state;
    float hike, t;
};

const Row kGolden[] = {
        {"u", 0.010000f, 0.000000f, 5, 0.000000f, 0.000000f},
        {"u", 0.012000f, 0.100000f, 5, 0.000500f, 0.000040f},
        {"u", 0.014000f, 0.200000f, 5, 0.001008f, 0.000120f},
        {"u", 0.016000f, 0.300000f, 5, 0.001536f, 0.000240f},
        {"u", 0.018000f, 0.400000f, 5, 0.002096f, 0.000400f},
        {"u", 0.020000f, 0.500000f, 5, 0.002700f, 0.000600f},
        {"u", 0.022000f, 0.600000f, 5, 0.003360f, 0.000840f},
        {"u", 0.024000f, 0.700000f, 5, 0.004088f, 0.001120f},
        {"u", 0.026000f, 0.800000f, 5, 0.004896f, 0.001440f},
        {"u", 0.028000f, 0.900000f, 5, 0.005796f, 0.001800f},
        {"u", 0.030000f, 1.000000f, 5, 0.006800f, 0.002200f},
        {"u", 0.032000f, 1.000000f, 5, 0.007700f, 0.002640f},
        {"u", 0.034000f, 1.000000f, 5, 0.008640f, 0.003120f},
        {"u", 0.036000f, 1.000000f, 5, 0.009620f, 0.003640f},
        {"u", 0.038000f, 1.000000f, 5, 0.010640f, 0.004200f},
        {"hold", 0.060000f, 1.000000f, 6, 0.010640f, 0.004200f},
        {"u", 0.070000f, 1.000000f, 6, 0.010640f, 0.004200f},
        {"u", 0.070000f, 1.000000f, 6, 0.010640f, 0.004200f},
        {"u", 0.070000f, 1.000000f, 6, 0.010640f, 0.004200f},
        {"release", 0.060000f, 1.000000f, 7, 0.010285f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.009931f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.009576f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.009221f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.008867f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.008512f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.008157f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.007803f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.007448f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.007093f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.006739f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.006384f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.006029f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.005675f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.005320f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.004965f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.004611f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.004256f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.003901f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.003547f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.003192f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.002837f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.002483f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.002128f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.001773f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.001419f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.001064f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.000709f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 7, 0.000355f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 0, 0.000000f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 0, 0.000000f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 0, 0.000000f, 0.004200f},
        {"u", 0.050000f, 1.000000f, 0, 0.000000f, 0.004200f},
        {"u", -0.020000f, 0.500000f, 5, 0.026180f, 0.052360f},
        {"u", -0.050000f, 1.000000f, 5, 0.059860f, 0.052960f},
        {"u", -0.080000f, 1.000000f, 5, 0.067960f, 0.054160f},
        {"u", -0.110000f, 1.000000f, 5, 0.076660f, 0.055960f},
        {"u", -0.140000f, 1.000000f, 5, 0.085960f, 0.055960f},
        {"u", -0.170000f, 1.000000f, 5, 0.093460f, 0.055960f},
        {"u", -0.400000f, 1.000000f, 8, 0.093460f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.090345f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.087229f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.084114f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.080999f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.077883f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.074768f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.071653f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.068537f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.065422f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.062307f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.059191f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.056076f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.052961f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.049845f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.046730f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.043615f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.040499f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.037384f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.034269f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.031153f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.028038f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.024923f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.021807f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.018692f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.015577f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.012461f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.009346f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.006231f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 8, 0.003115f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 0, 0.000000f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 0, 0.000000f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 0, 0.000000f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 0, 0.000000f, 0.052360f},
        {"u", -0.400000f, 1.000000f, 0, 0.000000f, 0.052360f},
        {"u", -0.000000f, 1.000000f, 5, 0.052360f, 0.052360f},
        {"u", -0.016000f, 1.000000f, 5, 0.056360f, 0.052680f},
        {"u", -0.032000f, 1.000000f, 5, 0.060680f, 0.053320f},
        {"u", -0.048000f, 1.000000f, 5, 0.065320f, 0.054280f},
        {"u", -0.064000f, 1.000000f, 5, 0.070280f, 0.055560f},
        {"u", -0.080000f, 1.000000f, 5, 0.075560f, 0.057160f},
        {"u", -0.096000f, 1.000000f, 5, 0.081160f, 0.059080f},
        {"u", -0.112000f, 1.000000f, 5, 0.087080f, 0.059080f},
        {"u", -0.128000f, 1.000000f, 5, 0.091080f, 0.059080f},
        {"u", -0.144000f, 1.000000f, 5, 0.095080f, 0.059080f},
        {"release", -0.100000f, 1.000000f, 7, 0.091911f, 0.052360f},
};

void testMatchesSimTwin() {
    StanceTrim tr;
    tr.begin('L', 0.01f);
    int block = 0;
    bool second = false;
    for (const Row& r : kGolden) {
        if (r.state == 5 && tr.state() == TrimState::kOff) {
            ++block;
            if (block == 1) {
                // the twin's second block: R stance with a learned 3 deg trim, then an abort (T reverts)
                CHECK(tr.setLearned('R', 3.0f * kDeg));
                tr.begin('R', -0.02f);
                second = true;
            } else {
                // the third: a 9 deg lean released without an abort (learning stops past 6, T reverts)
                tr.begin('R', 0.0f);
            }
        }
        if (!strcmp(r.ev, "hold")) tr.hold();
        if (!strcmp(r.ev, "release")) tr.release();
        tr.update(r.roll, 0.02f, r.ramp);
        CHECK_EQ(static_cast<int>(tr.state()), r.state);
        CHECK_NEAR(tr.hike(), r.hike, 2e-5);
        CHECK_NEAR(tr.learned(second ? 'R' : 'L'), r.t, 2e-5);
    }
    CHECK(second);
    CHECK_EQ(block, 2);
    CHECK(tr.lastReverted());
}

void testSignsBothSides() {
    // L stance, leaning toward the lifted R foot (roll +): hike up, the L hip roll joint goes +
    StanceTrim l;
    l.begin('L', 0.0f);
    l.update(0.05f, 0.02f, 1.0f);
    CHECK(l.hike() > 0.0f);
    CHECK_NEAR(l.hipJoint(), l.hike(), 1e-9);
    CHECK(l.learned('L') > 0.0f);
    // R stance, leaning toward the lifted L foot (roll -): hike up, the R hip roll joint goes -
    StanceTrim r;
    r.begin('R', 0.0f);
    r.update(-0.05f, 0.02f, 1.0f);
    CHECK(r.hike() > 0.0f);
    CHECK_NEAR(r.hipJoint(), -r.hike(), 1e-9);
    CHECK(r.learned('R') > 0.0f);
    // mirror: the same lean toward the swing foot gives the same hike on either side
    CHECK_NEAR(l.hike(), r.hike(), 1e-7);
    CHECK_NEAR(l.learned('L'), r.learned('R'), 1e-7);
}

void testReferenceIsCaptured() {
    StanceTrim a, b;
    a.begin('L', 0.0f);
    b.begin('L', 0.1f);
    a.update(0.02f, 0.02f, 1.0f);
    b.update(0.12f, 0.02f, 1.0f);
    CHECK_NEAR(a.hike(), b.hike(), 1e-7);
    CHECK_NEAR(a.err(), b.err(), 1e-7);
}

void testRampAppliesLearnedTrim() {
    StanceTrim tr;
    CHECK(tr.setLearned('L', 8.0f * kDeg));
    tr.begin('L', 0.0f);
    tr.update(0.0f, 0.02f, 0.0f);
    CHECK_NEAR(tr.hike(), 0.0f, 1e-9);
    tr.update(0.0f, 0.02f, 0.5f);
    CHECK_NEAR(tr.hike(), 4.0f * kDeg, 1e-6);
    tr.update(0.0f, 0.02f, 1.0f);
    CHECK_NEAR(tr.hike(), 8.0f * kDeg, 1e-6);
    // the learned trim is kept across chains, per side
    tr.release();
    for (int k = 0; k < 40; ++k) tr.update(0.0f, 0.02f, 1.0f);
    CHECK(tr.state() == TrimState::kOff);
    CHECK_NEAR(tr.learned('L'), 8.0f * kDeg, 1e-6);
    CHECK_NEAR(tr.learned('R'), 0.0f, 1e-9);
}

void testClampAndAntiWindup() {
    StanceTrim tr;
    tr.begin('L', 0.0f);
    for (int k = 0; k < 10; ++k) tr.update(0.08f, 0.02f, 1.0f);    // 4.6 deg: inside learn_lim
    const float t1 = tr.learned('L');
    CHECK(tr.hike() <= tr.config().lim_rad + 1e-7f);
    TrimConfig c;
    c.kp = 2.0f;
    StanceTrim big;
    big.configure(c);
    big.begin('L', 0.0f);
    big.update(0.2f, 0.02f, 1.0f);      // kp e = 0.4 > lim 0.2618: saturated
    CHECK_NEAR(big.hike(), c.lim_rad, 1e-7);
    CHECK_NEAR(big.learned('L'), 0.0f, 1e-9);    // no integration while saturated
    CHECK(t1 > 0.0f);
}

void testLearnedLimit() {
    StanceTrim tr;
    CHECK(tr.setLearned('L', 1.0f));
    CHECK_NEAR(tr.learned('L'), tr.config().t_lim_rad, 1e-7);
    CHECK(!tr.setLearned('X', 0.1f));
    CHECK(!tr.setLearned('R', NAN));
    tr.clearLearned();
    CHECK_NEAR(tr.learned('L'), 0.0f, 1e-9);
}

void testHoldFreezesAndReleaseFades() {
    StanceTrim tr;
    tr.begin('R', 0.0f);
    for (int k = 0; k < 10; ++k) tr.update(-0.05f, 0.02f, 1.0f);
    const float h = tr.hike(), t = tr.learned('R');
    tr.hold();
    CHECK(tr.state() == TrimState::kHold);
    for (int k = 0; k < 10; ++k) tr.update(-0.15f, 0.02f, 1.0f);   // a touch-down jolt is not answered
    CHECK_NEAR(tr.hike(), h, 1e-9);
    CHECK_NEAR(tr.learned('R'), t, 1e-9);
    tr.resume();
    CHECK(tr.state() == TrimState::kClosed);
    tr.hold();
    tr.release();
    CHECK(tr.state() == TrimState::kFade);
    tr.resume();
    CHECK(tr.state() == TrimState::kFade);       // a fade is not resumed: a new chain begins instead
    tr.update(0.0f, 0.3f, 1.0f);
    CHECK_NEAR(tr.hike(), 0.5f * h, 1e-6);
    tr.update(0.0f, 0.31f, 1.0f);
    CHECK(tr.state() == TrimState::kOff);
    CHECK_NEAR(tr.hike(), 0.0f, 1e-9);
    CHECK_EQ(tr.side(), 0);
}

void testAbortFreezesAndFadesNoLanding() {
    StanceTrim tr;
    tr.begin('L', 0.0f);
    for (int k = 0; k < 5; ++k) tr.update(0.05f, 0.02f, 1.0f);
    const float h = tr.hike(), t = tr.learned('L');
    tr.update(0.3f, 0.02f, 1.0f);       // 17 deg > 15: abort, frozen at the last hike
    CHECK(tr.state() == TrimState::kAbortTip);
    CHECK(tr.aborted());
    CHECK_NEAR(tr.hike(), h, 1e-9);
    CHECK_NEAR(tr.abortErr(), 0.3f, 1e-6);
    CHECK(t > 0.0f);
    CHECK_NEAR(tr.learned('L'), 0.0f, 1e-9);        // the abort puts T back where the chain found it
    for (int k = 0; k < 15; ++k) tr.update(0.4f, 0.02f, 1.0f);
    CHECK_NEAR(tr.hike(), 0.5f * h, 1e-5);
    for (int k = 0; k < 15; ++k) tr.update(0.4f, 0.02f, 1.0f);
    CHECK(tr.state() == TrimState::kOff);
    // a release or hold during the abort changes nothing
    StanceTrim a;
    a.begin('L', 0.0f);
    a.update(0.3f, 0.02f, 1.0f);
    a.release();
    a.hold();
    CHECK(a.state() == TrimState::kAbortTip);
}

void testImuFault() {
    StanceTrim tr;
    tr.begin('L', 0.0f);
    tr.update(0.05f, 0.02f, 1.0f);
    const float h = tr.hike();
    tr.update(NAN, 0.02f, 1.0f);
    CHECK(tr.state() == TrimState::kAbortImu);
    CHECK_NEAR(tr.hike(), h, 1e-9);
    StanceTrim off;
    off.imuFault();
    CHECK(off.state() == TrimState::kOff);
}

void testStopAndValidate() {
    StanceTrim tr;
    tr.begin('L', 0.0f);
    tr.update(0.05f, 0.02f, 1.0f);
    tr.stop();
    CHECK(!tr.engaged());
    CHECK_NEAR(tr.hike(), 0.0f, 1e-9);
    TrimConfig c;
    CHECK(balance::validateTrim(c) == nullptr);
    c.kp = 3.0f;
    CHECK(balance::validateTrim(c) != nullptr);
    c = TrimConfig();
    c.t_lim_rad = c.lim_rad * 1.5f;
    CHECK(balance::validateTrim(c) != nullptr);
    c = TrimConfig();
    c.fade_s = 0.0f;
    CHECK(balance::validateTrim(c) != nullptr);
    c = TrimConfig();
    c.abort_rad = 3.0f / 57.29578f;      // the CLI's own conversion must pass at the bound
    CHECK(balance::validateTrim(c) == nullptr);
    CHECK(strcmp(balance::trimStateName(TrimState::kClosed), "CLOSED") == 0);
}

// The driver behind the gait track and a policy's gait clock: single support begins a chain with the
// roll then as the reference, the raise ramps T in, the lowering holds, double support fades and reports.
void testDriverChainAndReport() {
    StanceTrim tr;
    balance::TrimDriver drv(tr);
    balance::StepReport rep;
    balance::SupportIn ds;                      // double support
    CHECK(!drv.step(ds, 0.01f, 0.02f, &rep));
    CHECK(tr.state() == TrimState::kOff);
    balance::SupportIn ss;
    ss.stance = 'L';
    ss.progress = 0.0f;
    CHECK(drv.begins(ss));
    CHECK(!drv.step(ss, 0.02f, 0.02f, &rep));
    CHECK(tr.state() == TrimState::kClosed);
    CHECK_EQ(tr.side(), 'L');
    CHECK_NEAR(tr.ref(), 0.02f, 1e-9);
    CHECK(!drv.begins(ss));
    // the raise: lean toward the swing (R) foot = + roll on L stance, then the end of the raise
    for (int k = 1; k <= 10; ++k) {
        ss.progress = static_cast<float>(k) / 10.0f;
        drv.step(ss, 0.02f + 0.004f * static_cast<float>(k), 0.02f, &rep);
    }
    const float e_mid = tr.err(), h_mid = tr.hike(), t_mid = tr.learned('L');
    ss.progress = 1.0f;
    drv.step(ss, 0.02f - 0.03f, 0.02f, &rep);   // a lean to the stance side after the mid point
    ss.lowering = true;
    drv.step(ss, 0.0f, 0.02f, &rep);
    CHECK(tr.state() == TrimState::kHold);
    CHECK(drv.step(ds, 0.0f, 0.02f, &rep));     // the single support is over: report
    CHECK(tr.state() == TrimState::kFade);
    CHECK_EQ(rep.stance, 'L');
    CHECK_NEAR(rep.lean_in, 0.04f, 1e-6);
    CHECK_NEAR(rep.lean_out, 0.03f, 1e-6);
    CHECK_NEAR(rep.e_mid, e_mid, 1e-9);
    CHECK_NEAR(rep.hike_mid, h_mid, 1e-9);
    CHECK_NEAR(rep.t_mid, t_mid, 1e-9);
    CHECK(!rep.aborted);
    for (int k = 0; k < 40; ++k) drv.step(ds, 0.0f, 0.02f, &rep);
    CHECK(tr.state() == TrimState::kOff);       // the fade runs out in double support
}

void testDriverNoAttitudeNoChain() {
    StanceTrim tr;
    balance::TrimDriver drv(tr);
    balance::StepReport rep;
    balance::SupportIn ss;
    ss.stance = 'R';
    CHECK(!drv.step(ss, NAN, 0.02f, &rep));
    CHECK(tr.state() == TrimState::kOff);        // no reference, no chain
    CHECK(!drv.step(ss, 0.01f, 0.02f, &rep));    // the attitude is back mid support: the chain begins
    CHECK(tr.state() == TrimState::kClosed);
    CHECK_EQ(tr.side(), 'R');
    drv.step(ss, NAN, 0.02f, &rep);
    CHECK(tr.state() == TrimState::kAbortImu);   // lost inside a chain: the IMU abort
    balance::SupportIn ds;
    CHECK(drv.step(ds, 0.0f, 0.02f, &rep));
    CHECK(rep.aborted);
}

// The provisional T (hw 2026-11-12 shuttle): learning stops past learn_lim, and a chain that was not
// clean leaves T as it found it; a clean one commits what it learned.
void testProvisionalT() {
    StanceTrim tr;
    CHECK(tr.setLearned('L', 5.0f * kDeg));
    // clean: a 3 deg lean, released -> T kept
    tr.begin('L', 0.0f);
    for (int k = 0; k < 20; ++k) tr.update(3.0f * kDeg, 0.02f, 1.0f);
    const float t_clean = tr.learned('L');
    CHECK(t_clean > 5.0f * kDeg);
    tr.release();
    CHECK(!tr.lastReverted());
    CHECK_NEAR(tr.learned('L'), t_clean, 1e-9);
    for (int k = 0; k < 40; ++k) tr.update(0.0f, 0.02f, 1.0f);
    CHECK(tr.state() == TrimState::kOff);
    // past learn_lim: no learning at all, and past keep_lim: reverted at the release
    tr.begin('L', 0.0f);
    for (int k = 0; k < 20; ++k) tr.update(7.0f * kDeg, 0.02f, 1.0f);
    CHECK_NEAR(tr.learned('L'), t_clean, 1e-9);     // 7 deg > learn 6: T does not move
    tr.begin('L', 0.0f);                            // (a fresh chain: begin replaces)
    for (int k = 0; k < 10; ++k) tr.update(5.0f * kDeg, 0.02f, 1.0f);
    const float t_mid = tr.learned('L');
    CHECK(t_mid > t_clean);                         // learning below learn_lim
    for (int k = 0; k < 5; ++k) tr.update(9.0f * kDeg, 0.02f, 1.0f);   // the lean runs on to 9 deg
    tr.release();
    CHECK(tr.lastReverted());
    CHECK_NEAR(tr.learned('L'), t_clean, 1e-9);     // what the bad chain learned is dropped
    CHECK_NEAR(tr.peakErr(), 9.0f * kDeg, 1e-6);
    // the hike at its clamp: reverted even with a small peak
    StanceTrim sat;
    TrimConfig c;
    c.lim_rad = 4.0f * kDeg;
    c.t_lim_rad = 4.0f * kDeg;
    c.learn_rad = 3.0f * kDeg;
    c.keep_rad = 7.0f * kDeg;
    sat.configure(c);
    CHECK(sat.setLearned('R', 3.5f * kDeg));
    sat.begin('R', 0.0f);
    for (int k = 0; k < 10; ++k) sat.update(-2.5f * kDeg, 0.02f, 1.0f);   // 3.5 + 0.6 > 4: clamped
    sat.release();
    CHECK(sat.lastReverted());
    CHECK_NEAR(sat.learned('R'), 3.5f * kDeg, 1e-7);
    // validate
    TrimConfig v;
    v.keep_rad = v.learn_rad * 0.5f;
    CHECK(balance::validateTrim(v) != nullptr);
}

// The handover at a fast gait: the last chain's fade continues on its own hip as a residual while the
// new chain ramps in -- no step in the total correction at the new single support's begin.
void testDriverResidualHandover() {
    StanceTrim tr;
    balance::TrimDriver drv(tr);
    balance::StepReport rep;
    CHECK(tr.setLearned('L', 6.0f * kDeg));
    balance::SupportIn l, ds, r;
    l.stance = 'L';
    r.stance = 'R';
    r.progress = 0.0f;
    for (int k = 0; k < 20; ++k) drv.step(l, 0.0f, 0.02f, &rep);       // L chain, hike = 6 deg
    CHECK_NEAR(tr.hike(), 6.0f * kDeg, 1e-6);
    for (int k = 0; k < 10; ++k) drv.step(ds, 0.0f, 0.02f, &rep);      // 0.2 s of a 0.6 s fade
    const float before = tr.hipJoint();
    CHECK(before > 0.0f && before < 6.0f * kDeg);
    drv.step(r, 0.0f, 0.02f, &rep);                                    // R begins: hand over
    CHECK_EQ(drv.residualSide(), 'L');
    CHECK_EQ(tr.side(), 'R');
    // the L hip keeps its correction (it fades from here), it is not dropped to 0
    CHECK_NEAR(drv.residualHip(), before, 1e-9);                      // continuous: the same value
    for (int k = 0; k < 30; ++k) drv.step(r, 0.0f, 0.02f, &rep);
    CHECK_EQ(drv.residualSide(), 0);                                   // done within the fade left
    CHECK_NEAR(drv.residualHip(), 0.0f, 1e-9);
}

void testDriverSideToSide() {
    // straight from one stance to the other (no double support tick): report the first, begin the second
    StanceTrim tr;
    balance::TrimDriver drv(tr);
    balance::StepReport rep;
    balance::SupportIn l, r;
    l.stance = 'L';
    r.stance = 'R';
    drv.step(l, 0.0f, 0.02f, &rep);
    CHECK(drv.begins(r));
    CHECK(drv.step(r, 0.01f, 0.02f, &rep));
    CHECK_EQ(rep.stance, 'L');
    CHECK_EQ(tr.side(), 'R');
    CHECK(tr.state() == TrimState::kClosed);
    CHECK_NEAR(tr.ref(), 0.01f, 1e-9);
}

}  // namespace

int main() {
    testMatchesSimTwin();
    testSignsBothSides();
    testReferenceIsCaptured();
    testRampAppliesLearnedTrim();
    testClampAndAntiWindup();
    testLearnedLimit();
    testHoldFreezesAndReleaseFades();
    testAbortFreezesAndFadesNoLanding();
    testImuFault();
    testStopAndValidate();
    testDriverChainAndReport();
    testDriverNoAttitudeNoChain();
    testDriverSideToSide();
    testProvisionalT();
    testDriverResidualHandover();
    return testutil::report("stance_trim");
}
