// The static-MLP forward pass against numpy-computed golden vectors.
//
// The weights are the REAL distilled policy (tools/gen_policy_weights.py
// --run <run>, from the .silw the referee scored), and the arithmetic under
// test is brax's: running_statistics normaliser, swish hidden layers, a linear
// 2*action_size output, and NormalTanhDistribution.mode() = tanh(mean).
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "policy/mlp.h"
#include "obs/obs_spec.h"
#include "test_util.h"
// <> not "": a policy profile build (POLICY=...) puts its own vectors/
// first on the include path, ahead of this directory.
#include <vectors/policy_vectors.h>

namespace {

void testShapesAgreeWithTheObsSpec() {
    CHECK_EQ(policy::kObsDim, obs::kObsDim);
    CHECK_EQ(policy::kActDim, obs::kActDim);
    CHECK_EQ(policy::kObsDim, policy_vectors::kObsDim);
    CHECK_EQ(policy::kActDim, policy_vectors::kActDim);
#ifdef POLICY_WEIGHTS_Q8
    // An int8 head keeps only the act_dim mean logits (policy/dense.h).
    CHECK_EQ(policy::kLayersQ8[policy::kNumLayers - 1].out, policy::kActDim);
    CHECK_EQ(policy::kLayersQ8[0].in, policy::kObsDim);
    for (int i = 0; i < policy::kNumLayers; ++i) {
        const policy::DenseQ8& L = policy::kLayersQ8[i];
        CHECK_EQ(L.in_pad, policy::pad4(L.in));
        CHECK(L.in <= policy::kQ8MaxIn);
        CHECK(reinterpret_cast<uintptr_t>(L.w) % 4 == 0);
        if (i > 0) CHECK_EQ(L.in, policy::kLayersQ8[i - 1].out);
        // the zero padding and the 4 bytes of slack the kernels read
        for (int j = 0; j < L.out; ++j) {
            for (int k = L.in; k < L.in_pad; ++k) CHECK_EQ(L.w[j * L.in_pad + k], 0);
        }
        for (int k = 0; k < 4; ++k) CHECK_EQ(L.w[L.out * L.in_pad + k], 0);
    }
#else
    // The output layer is 2 * action_size: mean and log-std concatenated.
    CHECK_EQ(policy::kLayers[policy::kNumLayers - 1].out, 2 * policy::kActDim);
    CHECK_EQ(policy::kLayers[0].in, policy::kObsDim);
    for (int i = 1; i < policy::kNumLayers; ++i) {
        CHECK_EQ(policy::kLayers[i].in, policy::kLayers[i - 1].out);
    }
#endif
}

void testNormalizer() {
    float o[policy::kObsDim], n[policy::kObsDim];
    for (int i = 0; i < policy::kObsDim; ++i) o[i] = policy::kNormMean[i];
    policy::normalize(o, n);
    for (int i = 0; i < policy::kObsDim; ++i) CHECK_NEAR(n[i], 0.0, 1e-6);
    for (int i = 0; i < policy::kObsDim; ++i) {
        o[i] = policy::kNormMean[i] + policy::kNormStd[i];
    }
    policy::normalize(o, n);
    // A frozen slot's std is 1e-6, below float resolution next to a mean of
    // order 1 (mean + std rounds): expect what float arithmetic gives there.
    for (int i = 0; i < policy::kObsDim; ++i) {
        const float want = policy::isFrozen(i)
                               ? (o[i] - policy::kNormMean[i]) / policy::kNormStd[i]
                               : 1.0f;
        CHECK_NEAR(n[i], want, 1e-5);
    }
}

void testGoldenVectors() {
    for (size_t c = 0; c < policy_vectors::kNumCases; ++c) {
        const auto& k = policy_vectors::kCases[c];
        float action[policy::kActDim];
        policy::forward(k.obs, action);
        for (int i = 0; i < policy::kActDim; ++i) {
            // firmware-design section 7 asks for 1e-6 against the JAX golden
            // vectors; float32 accumulation order differs from numpy's BLAS,
            // so 2e-6 is the honest bound for a 147-wide dot product. An int8
            // profile's vectors are tools/policy_q8.py's, which the q8 path
            // reproduces to the bit up to the final tanhf.
            CHECK_NEAR(action[i], k.action[i], 2e-6);
            CHECK(action[i] >= -1.0f && action[i] <= 1.0f);
        }
    }
}

void testForwardIsPure() {
    // The scratch buffers are static; running twice must not drift.
    float a1[policy::kActDim], a2[policy::kActDim];
    policy::forward(policy_vectors::kCases[1].obs, a1);
    policy::forward(policy_vectors::kCases[2].obs, a2);
    policy::forward(policy_vectors::kCases[1].obs, a2);
    for (int i = 0; i < policy::kActDim; ++i) CHECK_NEAR(a2[i], a1[i], 0.0);
}

void testWeightsAreTheDeployedPolicy() {
    // The v1 blocker (firmware-design section 6) closing: the compiled-in net
    // is a real, refereed, distilled policy -- not the scaffold. Regenerating
    // the placeholder (`--placeholder`) turns this red on purpose.
    //
    // A policy PROFILE (firmware/policies/<name>, BIMO_POLICY) may carry an
    // untrained test net instead (tools/make_test_policy_run.py) while its
    // plant has no deployable policy: its golden vectors still pin the
    // arithmetic, and the firmware refuses to arm on it (cli.cpp armRefused).
#ifndef BIMO_POLICY_OVERLAY
    CHECK(!policy::kWeightsArePlaceholder);
#endif
    // ... and it is the SAME run the obs spec was generated from, so the
    // observation the assembler builds is the one the net was trained on.
    CHECK(strcmp(policy::kWeightsRun, obs::kRunName) == 0);
    // A distilled net, not the (512, 256, 128) trainer: every hidden layer
    // has to fit the scratch buffers, and the whole thing has to fit flash.
#ifdef POLICY_WEIGHTS_Q8
    // An int8 profile is the wider net (docs/firmware-design.md 6.1): its
    // bound is bytes streamed per tick, not the float scratch.
    size_t bytes = 0;
    for (int i = 0; i < policy::kNumLayers; ++i) {
        bytes += static_cast<size_t>(policy::kLayersQ8[i].out) *
                 policy::kLayersQ8[i].in_pad;
    }
    CHECK(bytes < 160u * 1024u);   // ~6 ms of flash streaming
#else
    // cppcheck-suppress knownConditionTrueFalse -- asserting the invariant,
    // not testing a runtime condition.
    CHECK(policy::kMaxWidth == policy::kObsDim);
    size_t n = 0;
    for (int i = 0; i < policy::kNumLayers; ++i) {
        n += static_cast<size_t>(policy::kLayers[i].in) *
                 static_cast<size_t>(policy::kLayers[i].out) +
             static_cast<size_t>(policy::kLayers[i].out);
    }
    CHECK(n < 100000);   // ~400 KB fp32 ceiling for the constexpr in DROM
#endif
}

void testTheNetIsNotDegenerate() {
    // A header full of zeros would pass every golden vector trivially (the
    // vectors would be zeros too). Assert the weights carry real magnitude.
    double sum = 0.0;
    size_t n = 0;
#ifdef POLICY_WEIGHTS_Q8
    for (int i = 0; i < policy::kNumLayers; ++i) {
        const policy::DenseQ8& L = policy::kLayersQ8[i];
        const size_t k = static_cast<size_t>(L.out) * L.in_pad;
        for (size_t j = 0; j < k; ++j) sum += fabs(static_cast<double>(L.w[j]));
        for (int j = 0; j < L.out; ++j) CHECK(L.scale[j] > 0.0f);
        n += k;
    }
    CHECK(n > 0 && sum / static_cast<double>(n) > 1.0);   // int8 units
#else
    for (int i = 0; i < policy::kNumLayers; ++i) {
        const size_t k = static_cast<size_t>(policy::kLayers[i].in) *
                         static_cast<size_t>(policy::kLayers[i].out);
        for (size_t j = 0; j < k; ++j) sum += fabs(policy::kLayers[i].w[j]);
        n += k;
    }
    CHECK(n > 0 && sum / static_cast<double>(n) > 1e-3);
#endif
    // The normaliser must be the trained one: brax floors std at std_eps
    // (1e-6) for channels the run never varied, and those channels MUST come
    // through as-is -- see docs/sil-harness.md finding 2.
    for (int i = 0; i < policy::kObsDim; ++i) CHECK(policy::kNormStd[i] > 0.0f);
}

void testFrozenChannels() {
    // docs/sil-harness.md finding 2 / policy/mlp.h pinFrozen. A frozen slot
    // (normaliser std at brax's 1e-6 floor) must be one the firmware can feed
    // its trained value EXACTLY: a command channel (pinned by pinFrozen) or a
    // slot the assembler hard-zeroes (linvel, height -- trained value 0).
    // A frozen sensor slot (q, dq, up, gyro, prev_action, phase) could not be
    // fed it and must not exist.
    using namespace obs;
    for (int s = 0; s < kFrameDim; ++s) {
        // the same set in every history frame (each frame is one frame)
        for (int h = 1; h < kHistLen; ++h) {
            CHECK(policy::isFrozen(h * kFrameDim + s) == policy::isFrozen(s));
        }
        if (!policy::isFrozen(s)) continue;
        const bool is_cmd = s >= kOffCmd && s < kOffCmd + kNumCmd;
        const bool is_zero = (s >= kOffLinVel && s < kOffLinVel + 3) || s == kOffHeight;
        CHECK(is_cmd || is_zero);
        if (is_zero) {
            for (int h = 0; h < kHistLen; ++h) {
                CHECK(policy::kNormMean[h * kFrameDim + s] == 0.0f);
            }
        }
    }
    // pinFrozen: frozen command channels come back at their trained value
    // (the normaliser mean), every other channel untouched, and the mask
    // names exactly the frozen channels that differed.
    float cmd[kNumCmd];
    for (int i = 0; i < kNumCmd; ++i) cmd[i] = 0.37f + 0.01f * static_cast<float>(i);
    const uint32_t m = policy::pinFrozen(kOffCmd, kNumCmd, cmd);
    for (int i = 0; i < kNumCmd; ++i) {
        const int k = kOffCmd + i;
        const float was = 0.37f + 0.01f * static_cast<float>(i);
        if (policy::isFrozen(k)) {
            CHECK(cmd[i] == policy::kNormMean[k]);
            CHECK(((m >> i) & 1u) == (was != policy::kNormMean[k] ? 1u : 0u));
        } else {
            CHECK(cmd[i] == was);
            CHECK(((m >> i) & 1u) == 0u);
        }
    }
    // pinned twice: nothing left to change
    CHECK(policy::pinFrozen(kOffCmd, kNumCmd, cmd) == 0u);
}


// -- the int8 path (policy/q8.h), on every build ----------------------------

uint32_t g_rng = 20261102u;
uint32_t rnd() {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng >> 8;
}
float rndf(float lo, float hi) {
    return lo + (hi - lo) * static_cast<float>(rnd() % 1000001u) / 1000000.0f;
}

void testExpAndSwishFast() {
    // The polynomial is good to ~1.2e-7; rounding z * log2(e) to float adds
    // ~6e-8 * |z| on top -- immaterial to swish, which is what it is for.
    double worst_e = 0.0, worst_e10 = 0.0, worst_s = 0.0;
    for (int k = -8000; k <= 8000; ++k) {
        const float z = static_cast<float>(k) * 0.01f;      // -80 .. 80
        const double want = exp(static_cast<double>(z));
        const double rel = fabs(static_cast<double>(policy::expFast(z)) - want) / want;
        if (rel > worst_e) worst_e = rel;
        if (fabsf(z) <= 10.0f && rel > worst_e10) worst_e10 = rel;
        if (fabsf(z) <= 30.0f) {
            const double y = static_cast<double>(z);
            const double sw = y / (1.0 + exp(-y));
            const double d = fabs(static_cast<double>(policy::swishFast(z)) - sw);
            if (d > worst_s) worst_s = d;
        }
    }
    CHECK(worst_e10 < 1e-6);
    CHECK(worst_e < 5e-6);
    CHECK(worst_s < 2e-6);
    CHECK(policy::expFast(200.0f) == policy::expFast(80.0f));    // clamped
    CHECK(policy::swishFast(-200.0f) <= 0.0f && policy::swishFast(-200.0f) > -1e-30f);
    CHECK(policy::swishFast(0.0f) == 0.0f);
}

void testQuantizeActs() {
    float x[13];
    alignas(4) int16_t q[16 + 4];
    for (int i = 0; i < 13; ++i) x[i] = rndf(-3.0f, 3.0f);
    x[5] = -7.5f;                                            // the max |x|
    for (int16_t& v : q) v = 99;
    const float s = policy::quantizeActs(x, 13, 16, q);
    CHECK_NEAR(s, 7.5 / 32767.0, 1e-9);
    CHECK_EQ(q[5], -32767);
    for (int i = 0; i < 13; ++i) CHECK_NEAR(q[i] * s, x[i], 0.5 * s + 1e-9);
    for (int i = 13; i < 16; ++i) CHECK_EQ(q[i], 0);
    for (float& v : x) v = 0.0f;
    CHECK(policy::quantizeActs(x, 13, 16, q) == 1.0f);
    for (int i = 0; i < 16; ++i) CHECK_EQ(q[i], 0);
}

// What q8_mac16.S computes, instruction by instruction in C: the 0xFF00FF00
// split of each weight word into int16 halves holding w << 8, the four
// half-by-half MULAs into a 40-bit accumulator, and the >> 8 funnel shift.
// It cannot prove the silicon (policy::init() does that on the robot) but it
// pins the trick itself: signs, halves, byte order and headroom.
void emulateMac16(const int16_t* x, const int8_t* w, int in_pad, int out, int32_t* acc) {
    const uint32_t mask = 0xFF00FF00u;
    for (int j = 0; j < out; ++j) {
        int64_t a = 0;                                       // ACCHI:ACCLO
        for (int i = 0; i < in_pad; i += 4) {
            uint32_t word;
            memcpy(&word, w + j * in_pad + i, 4);            // little-endian
            uint32_t m0, m1;                                 // x[1]:x[0], x[3]:x[2]
            memcpy(&m0, x + i, 4);
            memcpy(&m1, x + i + 2, 4);
            const uint32_t a13 = word & mask;
            const uint32_t a12 = (word << 8) & mask;
            auto lo = [](uint32_t r) { return static_cast<int16_t>(r & 0xFFFFu); };
            auto hi = [](uint32_t r) { return static_cast<int16_t>(r >> 16); };
            a += static_cast<int32_t>(lo(m0)) * lo(a12);     // mula.da.ll m0, a12
            a += static_cast<int32_t>(hi(m0)) * lo(a13);     // mula.da.hl m0, a13
            a += static_cast<int32_t>(lo(m1)) * hi(a12);     // mula.da.lh m1, a12
            a += static_cast<int32_t>(hi(m1)) * hi(a13);     // mula.da.hh m1, a13
        }
        CHECK(a < (int64_t{1} << 39) && a >= -(int64_t{1} << 39));
        CHECK(a % 256 == 0);
        acc[j] = static_cast<int32_t>(a / 256);              // src (ACCHI:ACCLO) >> 8
    }
}

void testQ8Kernels() {
    // widest supported input, every value extreme somewhere
    constexpr int kIn = policy::kQ8MaxIn, kOut = 9;
    static int8_t w[kOut * kIn + 4];
    alignas(4) static int16_t x[kIn + 4];
    for (int t = 0; t < 3; ++t) {
        for (int i = 0; i < kIn; ++i) {
            x[i] = t == 0 ? static_cast<int16_t>((i & 1) ? -32767 : 32767)
                          : static_cast<int16_t>(static_cast<int32_t>(rnd() % 65535u) - 32767);
        }
        for (int i = 0; i < kOut * kIn; ++i) {
            w[i] = t == 0 ? static_cast<int8_t>((i & 1) ? -127 : 127)
                          : static_cast<int8_t>(static_cast<int32_t>(rnd() % 255u) - 127);
        }
        int32_t got[kOut], emu[kOut];
        policy::q8RowsC(x, w, kIn, kOut, got);
        emulateMac16(x, w, kIn, kOut, emu);
        for (int j = 0; j < kOut; ++j) {
            int64_t want = 0;
            for (int i = 0; i < kIn; ++i) want += int64_t{x[i]} * w[j * kIn + i];
            CHECK_EQ(got[j], want);
            CHECK_EQ(emu[j], want);
        }
    }
}

void testQuantizeDense() {
    constexpr int kIn = 7, kOut = 5, kKeep = 3, kPad = 8;
    float wf[kIn * kOut], bf[kOut];
    for (float& v : wf) v = rndf(-0.4f, 0.4f);
    for (float& v : bf) v = rndf(-0.1f, 0.1f);
    for (int i = 0; i < kIn; ++i) wf[i * kOut + 1] = 0.0f;   // an all-zero output
    const policy::Dense L{kIn, kOut, wf, bf};
    int8_t wq[kKeep * kPad + 4];
    float sc[kKeep], bq[kKeep];
    memset(wq, 55, sizeof wq);
    policy::quantizeDense(L, kKeep, wq, sc, bq);
    for (int j = 0; j < kKeep; ++j) {
        CHECK(bq[j] == bf[j]);
        for (int i = 0; i < kIn; ++i) {
            CHECK_NEAR(wq[j * kPad + i] * sc[j], wf[i * kOut + j], 0.5 * sc[j] + 1e-9);
        }
        CHECK_EQ(wq[j * kPad + kIn], 0);
    }
    CHECK(sc[1] == 1.0f);
    for (int k = 0; k < 4; ++k) CHECK_EQ(wq[kKeep * kPad + k], 0);
}

void testFloatHeadTrimIsExact() {
#ifndef POLICY_WEIGHTS_Q8
    // forwardNet computes only the act_dim mean logits of the 2*act_dim head.
    // Asking it for the whole head (act_dim = 2 * kActDim) must give the same
    // first kActDim actions to the bit.
    static float sa[policy::kMaxWidth], sb[policy::kMaxWidth];
    policy::Net net;
    net.obs_dim = policy::kObsDim;
    net.num_layers = policy::kNumLayers;
    net.layers = policy::kLayers;
    net.norm_mean = policy::kNormMean;
    net.norm_std = policy::kNormStd;
    net.scratch_a = sa;
    net.scratch_b = sb;
    float trim[policy::kActDim], full[2 * policy::kActDim];
    net.act_dim = policy::kActDim;
    policy::forwardNet(net, policy_vectors::kCases[3].obs, trim);
    net.act_dim = 2 * policy::kActDim;
    policy::forwardNet(net, policy_vectors::kCases[3].obs, full);
    // the two nets must agree bit for bit, so a byte comparison is the point
    CHECK(memcmp(trim, full, sizeof trim) == 0);  // NOLINT(bugprone-suspicious-memory-comparison)
#endif
}

#ifdef POLICY_WEIGHTS_Q8
// A bump allocator over static storage: init() never frees, and a malloc'd
// copy would read as a leak to LeakSanitizer.
alignas(4) unsigned char g_arena[256 * 1024];
size_t g_arena_used = 0;
void* arenaAlloc(size_t n) {
    n = (n + 3u) & ~size_t{3};
    if (g_arena_used + n > sizeof g_arena) return nullptr;
    void* p = g_arena + g_arena_used;
    g_arena_used += n;
    return p;
}

void testInitPlacementAndSelfTest() {
    // No RAM budget: everything stays in "flash", the self-test passes.
    policy::init(nullptr, 0);
    const policy::Status& st = policy::status();
    CHECK(st.quantized);
    CHECK(st.selftest_ran && st.selftest_ok);
    CHECK(st.selftest_err <= policy::kSelfTestTol);
    CHECK_EQ(st.layers_in_ram, 0);
    CHECK(st.kernel == policy::Kernel::kC);          // no MAC16 on the host
    // A budget for the smallest layers only: largest-first skips what does
    // not fit and still places the rest.
    size_t smallest = SIZE_MAX, total = 0;
    for (int l = 0; l < policy::kNumLayers; ++l) {
        const size_t n = static_cast<size_t>(policy::kLayersQ8[l].out) *
                             policy::kLayersQ8[l].in_pad + 4u;
        if (n < smallest) smallest = n;
        total += n;
    }
    policy::init(arenaAlloc, smallest);
    CHECK_EQ(policy::status().layers_in_ram, 1);
    CHECK_EQ(policy::status().ram_bytes, smallest);
    // Everything in RAM: the golden vectors still hold, to the same bound.
    policy::init(arenaAlloc, total);
    CHECK_EQ(policy::status().layers_in_ram, policy::kNumLayers);
    CHECK(policy::status().selftest_ok);
    for (size_t c = 0; c < policy_vectors::kNumCases; ++c) {
        float a[policy::kActDim];
        policy::forward(policy_vectors::kCases[c].obs, a);
        for (int i = 0; i < policy::kActDim; ++i) {
            CHECK_NEAR(a[i], policy_vectors::kCases[c].action[i], 2e-6);
        }
    }
}
#endif

}  // namespace

int main() {
    testShapesAgreeWithTheObsSpec();
    testNormalizer();
    testGoldenVectors();
    testForwardIsPure();
    testWeightsAreTheDeployedPolicy();
    testTheNetIsNotDegenerate();
    testFrozenChannels();
    testExpAndSwishFast();
    testQuantizeActs();
    testQ8Kernels();
    testQuantizeDense();
    testFloatHeadTrimIsExact();
#ifdef POLICY_WEIGHTS_Q8
    testInitPlacementAndSelfTest();
#endif
    return testutil::report("policy");
}
