// Static-MLP forward pass for the deployed brax/PPO policy.
//
// What the training side actually produces (verified against
// sim/runs/loco_v5t/params.pkl, 2026-07-26):
//
//   * observations are normalised by brax's running_statistics:
//       x = (obs - mean) / std,  element-wise over the FULL stacked obs
//   * hidden layers are flax Dense with kernel shape (in, out), row-major,
//     activation = linen.swish  (x * sigmoid(x))  -- brax's make_ppo_networks
//     default. NOTE: docs/firmware-design.md section 5 said "tanh/identity";
//     that is wrong and the changelog records the correction.
//   * the output layer is linear and 2 * action_size wide: the first half is
//     the distribution mean, the second half the (state-independent) log-std
//   * deterministic inference is NormalTanhDistribution.mode() = tanh(mean)
//
// So: normalize -> swish MLP -> take the first kActDim logits -> tanh.
//
// The weights come from a generated header. In v1 that header holds a small
// PLACEHOLDER net at the correct input/output width -- sim/export_policy.py
// does not exist yet (firmware/README.md, "v2 seams"). The golden-vector test
// harness is real: tools/gen_policy_weights.py writes both the weights and the
// numpy-computed obs->action pairs the host test checks to 1e-6.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "policy/dense.h"
#include "policy/weights.h"

namespace policy {

// Run the deployed policy. `obs` is kObsDim wide, `action` is kActDim wide and
// comes back in [-1, 1].
void forward(const float* obs, float* action);

// Normalise only -- exposed so the host tests can check the normaliser
// separately from the layers when a golden vector disagrees.
void normalize(const float* obs, float* out);

}  // namespace policy
