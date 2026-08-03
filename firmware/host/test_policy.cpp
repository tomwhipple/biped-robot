// The static-MLP forward pass against numpy-computed golden vectors.
//
// The weights are the REAL distilled policy (tools/gen_policy_weights.py
// --run <run>, from the .silw the referee scored), and the arithmetic under
// test is brax's: running_statistics normaliser, swish hidden layers, a linear
// 2*action_size output, and NormalTanhDistribution.mode() = tanh(mean).
#include <math.h>
#include <string.h>

#include "policy/mlp.h"
#include "obs/obs_spec.h"
#include "test_util.h"
#include "vectors/policy_vectors.h"

namespace {

void testShapesAgreeWithTheObsSpec() {
    CHECK_EQ(policy::kObsDim, obs::kObsDim);
    CHECK_EQ(policy::kActDim, obs::kActDim);
    CHECK_EQ(policy::kObsDim, policy_vectors::kObsDim);
    CHECK_EQ(policy::kActDim, policy_vectors::kActDim);
    // The output layer is 2 * action_size: mean and log-std concatenated.
    CHECK_EQ(policy::kLayers[policy::kNumLayers - 1].out, 2 * policy::kActDim);
    CHECK_EQ(policy::kLayers[0].in, policy::kObsDim);
    for (int i = 1; i < policy::kNumLayers; ++i) {
        CHECK_EQ(policy::kLayers[i].in, policy::kLayers[i - 1].out);
    }
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
    for (int i = 0; i < policy::kObsDim; ++i) CHECK_NEAR(n[i], 1.0, 1e-5);
}

void testGoldenVectors() {
    for (size_t c = 0; c < policy_vectors::kNumCases; ++c) {
        const auto& k = policy_vectors::kCases[c];
        float action[policy::kActDim];
        policy::forward(k.obs, action);
        for (int i = 0; i < policy::kActDim; ++i) {
            // firmware-design section 7 asks for 1e-6 against the JAX golden
            // vectors; float32 accumulation order differs from numpy's BLAS,
            // so 2e-6 is the honest bound for a 147-wide dot product.
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
    CHECK(!policy::kWeightsArePlaceholder);
    // ... and it is the SAME run the obs spec was generated from, so the
    // observation the assembler builds is the one the net was trained on.
    CHECK(strcmp(policy::kWeightsRun, obs::kRunName) == 0);
    // A distilled net, not the (512, 256, 128) trainer: every hidden layer
    // has to fit the scratch buffers, and the whole thing has to fit flash.
    CHECK(policy::kMaxWidth == policy::kObsDim);
    size_t n = 0;
    for (int i = 0; i < policy::kNumLayers; ++i) {
        n += static_cast<size_t>(policy::kLayers[i].in) *
                 static_cast<size_t>(policy::kLayers[i].out) +
             static_cast<size_t>(policy::kLayers[i].out);
    }
    CHECK(n < 100000);   // ~400 KB fp32 ceiling for the constexpr in DROM
}

void testTheNetIsNotDegenerate() {
    // A header full of zeros would pass every golden vector trivially (the
    // vectors would be zeros too). Assert the weights carry real magnitude.
    double sum = 0.0;
    size_t n = 0;
    for (int i = 0; i < policy::kNumLayers; ++i) {
        const size_t k = static_cast<size_t>(policy::kLayers[i].in) *
                         static_cast<size_t>(policy::kLayers[i].out);
        for (size_t j = 0; j < k; ++j) sum += fabs(policy::kLayers[i].w[j]);
        n += k;
    }
    CHECK(n > 0 && sum / static_cast<double>(n) > 1e-3);
    // The normaliser must be the trained one: brax floors std at std_eps
    // (1e-6) for channels the run never varied, and those channels MUST come
    // through as-is -- see docs/sil-harness.md finding 2.
    for (int i = 0; i < policy::kObsDim; ++i) CHECK(policy::kNormStd[i] > 0.0f);
}

}  // namespace

int main() {
    testShapesAgreeWithTheObsSpec();
    testNormalizer();
    testGoldenVectors();
    testForwardIsPure();
    testWeightsAreTheDeployedPolicy();
    testTheNetIsNotDegenerate();
    return testutil::report("policy");
}
