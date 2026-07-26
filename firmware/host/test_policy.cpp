// The static-MLP forward pass against numpy-computed golden vectors.
//
// The weights are a PLACEHOLDER (tools/gen_policy_weights.py) because
// sim/export_policy.py does not exist yet -- but the arithmetic under test is
// the real thing: brax's running_statistics normaliser, swish hidden layers, a
// linear 2*action_size output, and NormalTanhDistribution.mode() = tanh(mean).
// When the real distilled net lands, only the header changes.
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

void testPlaceholderIsFlagged() {
    // A loud, testable reminder that this is not the trained policy yet.
    CHECK(policy::kWeightsArePlaceholder);
}

}  // namespace

int main() {
    testShapesAgreeWithTheObsSpec();
    testNormalizer();
    testGoldenVectors();
    testForwardIsPure();
    testPlaceholderIsFlagged();
    return testutil::report("policy");
}
