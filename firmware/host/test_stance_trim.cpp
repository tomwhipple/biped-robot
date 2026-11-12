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
        {"u", -0.140000f, 1.000000f, 5, 0.085960f, 0.058360f},
        {"u", -0.170000f, 1.000000f, 5, 0.095860f, 0.061360f},
        {"u", -0.400000f, 1.000000f, 8, 0.095860f, 0.061360f},
        {"u", -0.400000f, 1.000000f, 8, 0.092665f, 0.061360f},
        {"u", -0.400000f, 1.000000f, 8, 0.089469f, 0.061360f},
        {"u", -0.400000f, 1.000000f, 8, 0.086274f, 0.061360f},
};

void testMatchesSimTwin() {
    StanceTrim tr;
    tr.begin('L', 0.01f);
    bool second = false;
    for (const Row& r : kGolden) {
        if (!second && r.state == 5 && tr.state() == TrimState::kOff) {
            // the twin's second block: R stance with a learned 3 deg trim
            CHECK(tr.setLearned('R', 3.0f * kDeg));
            tr.begin('R', -0.02f);
            second = true;
        }
        if (!strcmp(r.ev, "hold")) tr.hold();
        if (!strcmp(r.ev, "release")) tr.release();
        tr.update(r.roll, 0.02f, r.ramp);
        CHECK_EQ(static_cast<int>(tr.state()), r.state);
        CHECK_NEAR(tr.hike(), r.hike, 2e-5);
        CHECK_NEAR(tr.learned(second ? 'R' : 'L'), r.t, 2e-5);
    }
    CHECK(second);
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
    for (int k = 0; k < 10; ++k) tr.update(0.25f, 0.02f, 1.0f);
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
    CHECK_NEAR(tr.learned('L'), t, 1e-9);           // the abort tick does not learn
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
    return testutil::report("stance_trim");
}
