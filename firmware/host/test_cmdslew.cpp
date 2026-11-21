// The operator command slew (components/cmdslew, issue #266), pinned to sim/cmd_kernel.py.
//
// The trajectory below is cmd_kernel.golden(): 120 ticks x (vx, vy, wz), float32, printed %.9g.
// The C++ law is the same float32 arithmetic, so it must match BIT FOR BIT (no tolerance).
// tests/test_cmdslew_golden.py asserts this table, the rates, the tick and the target steps equal
// cmd_kernel's, so drift on either side fails a test.
#include "cmdslew/cmdslew.h"

#include <stdint.h>
#include <string.h>

#include "test_util.h"

using cmdslew::CommandSlew;
using cmdslew::Rates;

namespace {

// GOLDEN_RATES, GOLDEN_DT, GOLDEN_STEPS (sim/cmd_kernel.py).
constexpr float kGoldenRates[3] = {0.5f, 0.4f, 1.5f};
constexpr float kGoldenDt = 0.02f;
struct GoldenStep {
    int tick;
    float tgt[3];
};
constexpr GoldenStep kGoldenSteps[] = {
    {0, {0.24f, -0.25f, 0.8f}},
    {20, {-0.4f, 0.0f, -1.0f}},
    {55, {0.0f, 0.0f, 0.0f}},
    {90, {0.013f, 0.0f, 0.0f}},
};
constexpr int kGoldenTicks = 120;
// BEGIN GOLDEN TABLE
constexpr float kGolden[kGoldenTicks][3] = {
    {0.00999999978f, -0.00800000038f, 0.0299999993f},
    {0.0199999996f, -0.0160000008f, 0.0599999987f},
    {0.0299999993f, -0.0240000002f, 0.0899999961f},
    {0.0399999991f, -0.0320000015f, 0.119999997f},
    {0.049999997f, -0.0400000028f, 0.149999991f},
    {0.0599999949f, -0.0480000041f, 0.179999992f},
    {0.0699999928f, -0.0560000055f, 0.209999993f},
    {0.0799999908f, -0.064000003f, 0.239999995f},
    {0.0899999887f, -0.0720000044f, 0.269999981f},
    {0.0999999866f, -0.0800000057f, 0.299999982f},
    {0.109999985f, -0.088000007f, 0.329999983f},
    {0.119999982f, -0.0960000083f, 0.359999985f},
    {0.12999998f, -0.10400001f, 0.389999986f},
    {0.139999986f, -0.112000011f, 0.419999987f},
    {0.149999991f, -0.120000012f, 0.449999988f},
    {0.159999996f, -0.128000006f, 0.479999989f},
    {0.170000002f, -0.136000007f, 0.50999999f},
    {0.180000007f, -0.144000009f, 0.539999962f},
    {0.190000013f, -0.15200001f, 0.569999933f},
    {0.200000018f, -0.160000011f, 0.599999905f},
    {0.190000013f, -0.15200001f, 0.569999933f},
    {0.180000007f, -0.144000009f, 0.539999962f},
    {0.170000002f, -0.136000007f, 0.50999999f},
    {0.159999996f, -0.128000006f, 0.479999989f},
    {0.149999991f, -0.120000005f, 0.449999988f},
    {0.139999986f, -0.112000003f, 0.419999987f},
    {0.12999998f, -0.104000002f, 0.389999986f},
    {0.119999982f, -0.0960000008f, 0.359999985f},
    {0.109999985f, -0.0879999995f, 0.329999983f},
    {0.0999999866f, -0.0799999982f, 0.299999982f},
    {0.0899999887f, -0.0719999969f, 0.269999981f},
    {0.0799999908f, -0.0639999956f, 0.23999998f},
    {0.0699999928f, -0.0559999943f, 0.209999979f},
    {0.0599999949f, -0.047999993f, 0.179999977f},
    {0.049999997f, -0.0399999917f, 0.149999976f},
    {0.0399999991f, -0.0319999903f, 0.119999975f},
    {0.0299999993f, -0.023999989f, 0.0899999738f},
    {0.0199999996f, -0.0159999877f, 0.0599999726f},
    {0.00999999978f, -0.00799998734f, 0.0299999733f},
    {0.0f, 0.0f, -2.60770321e-08f},
    {-0.00999999978f, 0.0f, -0.0300000254f},
    {-0.0199999996f, 0.0f, -0.0600000247f},
    {-0.0299999993f, 0.0f, -0.0900000259f},
    {-0.0399999991f, 0.0f, -0.120000027f},
    {-0.049999997f, 0.0f, -0.150000021f},
    {-0.0599999949f, 0.0f, -0.180000022f},
    {-0.0699999928f, 0.0f, -0.210000023f},
    {-0.0799999908f, 0.0f, -0.240000024f},
    {-0.0899999887f, 0.0f, -0.270000011f},
    {-0.0999999866f, 0.0f, -0.300000012f},
    {-0.109999985f, 0.0f, -0.330000013f},
    {-0.119999982f, 0.0f, -0.360000014f},
    {-0.12999998f, 0.0f, -0.390000015f},
    {-0.139999986f, 0.0f, -0.420000017f},
    {-0.149999991f, 0.0f, -0.450000018f},
    {-0.139999986f, 0.0f, -0.420000017f},
    {-0.12999998f, 0.0f, -0.390000015f},
    {-0.119999982f, 0.0f, -0.360000014f},
    {-0.109999985f, 0.0f, -0.330000013f},
    {-0.0999999866f, 0.0f, -0.300000012f},
    {-0.0899999887f, 0.0f, -0.270000011f},
    {-0.0799999908f, 0.0f, -0.24000001f},
    {-0.0699999928f, 0.0f, -0.210000008f},
    {-0.0599999949f, 0.0f, -0.180000007f},
    {-0.049999997f, 0.0f, -0.150000006f},
    {-0.0399999991f, 0.0f, -0.120000005f},
    {-0.0299999993f, 0.0f, -0.0900000036f},
    {-0.0199999996f, 0.0f, -0.0600000024f},
    {-0.00999999978f, 0.0f, -0.0300000031f},
    {0.0f, 0.0f, -3.7252903e-09f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.00999999978f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
    {0.0130000003f, 0.0f, 0.0f},
};
// END GOLDEN TABLE

uint32_t bitsOf(float f) {
    uint32_t u = 0;
    memcpy(&u, &f, sizeof u);
    return u;
}
bool bitEq(float a, float b) { return bitsOf(a) == bitsOf(b); }

void testGolden() {
    CommandSlew s;
    s.configure(Rates{kGoldenRates[0], kGoldenRates[1], kGoldenRates[2]}, kGoldenDt);
    s.reset();
    CHECK(s.enabled());
    float tgt[3] = {0.0f, 0.0f, 0.0f};
    int bad = 0;
    for (int k = 0; k < kGoldenTicks; ++k) {
        for (const GoldenStep& st : kGoldenSteps) {
            if (st.tick == k) memcpy(tgt, st.tgt, sizeof tgt);
        }
        s.step(tgt[0], tgt[1], tgt[2]);
        if (!bitEq(s.vx(), kGolden[k][0]) || !bitEq(s.vy(), kGolden[k][1]) ||
            !bitEq(s.wz(), kGolden[k][2])) {
            ++bad;
            fprintf(stderr, "tick %d: got %.9g %.9g %.9g want %.9g %.9g %.9g\n", k, s.vx(), s.vy(),
                    s.wz(), kGolden[k][0], kGolden[k][1], kGolden[k][2]);
        }
    }
    CHECK_EQ(bad, 0);
}

void testDisabledPassesThrough() {
    CommandSlew s;
    CHECK(!s.enabled());
    s.configure(Rates{}, 0.02f);
    CHECK(!s.enabled());
    s.step(0.3f, -0.2f, 0.9f);
    CHECK(bitEq(s.vx(), 0.3f) && bitEq(s.vy(), -0.2f) && bitEq(s.wz(), 0.9f));
    s.step(-1.0f, 1.0f, -1.0f);
    CHECK(bitEq(s.vx(), -1.0f) && bitEq(s.vy(), 1.0f) && bitEq(s.wz(), -1.0f));
}

void testZeroRateChannelPassesThrough() {
    CommandSlew s;
    s.configure(Rates{0.5f, 0.0f, 1.5f}, 0.02f);
    CHECK(s.enabled());
    s.step(0.24f, -0.25f, 0.8f);
    CHECK(bitEq(s.vx(), 0.5f * 0.02f));   // ramping
    CHECK(bitEq(s.vy(), -0.25f));         // snapped
    CHECK(bitEq(s.wz(), 1.5f * 0.02f));
    CommandSlew t;
    t.configure(Rates{0.0f, 0.4f, 0.0f}, 0.02f);
    t.step(0.24f, -0.25f, 0.8f);
    CHECK(bitEq(t.vx(), 0.24f));
    CHECK(bitEq(t.vy(), -(0.4f * 0.02f)));
    CHECK(bitEq(t.wz(), 0.8f));
}

void testResetAndLanding() {
    CommandSlew s;
    s.configure(Rates{0.5f, 0.4f, 1.5f}, 0.02f);
    s.reset();
    s.step(0.24f, 0.0f, 0.0f);
    CHECK(s.vx() > 0.0f && s.vx() < 0.24f);
    s.reset();                      // the stand on a re-arm
    CHECK(s.vx() == 0.0f && s.vy() == 0.0f && s.wz() == 0.0f);
    s.reset(0.1f, -0.1f, 0.5f);
    CHECK(s.vx() == 0.1f && s.vy() == -0.1f && s.wz() == 0.5f);
    // Lands exactly on the target, then stays.
    for (int k = 0; k < 200; ++k) s.step(0.24f, 0.0f, 0.0f);
    CHECK(bitEq(s.vx(), 0.24f) && s.vy() == 0.0f && s.wz() == 0.0f);
    // A target inside one step is taken at once.
    s.reset();
    s.step(0.009f, 0.0f, 0.0f);
    CHECK(bitEq(s.vx(), 0.009f));
}

}  // namespace

int main() {
    testGolden();
    testDisabledPassesThrough();
    testZeroRateChannelPassesThrough();
    testResetAndLanding();
    return testutil::report("test_cmdslew");
}
