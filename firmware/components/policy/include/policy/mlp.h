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
// The weights come from a generated header holding the DEPLOYED policy: the
// training net distilled to a width that fits flash (firmware-design section
// 6, sim/mjx/distill_student.py), exported to a .silw by
// tools/export_policy_weights.py, refereed on both scorecard columns, and
// compiled in by tools/gen_policy_weights.py --run <run>, which writes the
// numpy-computed obs->action pairs the host test checks alongside it.
// `policy::kWeightsArePlaceholder` is false and test_policy asserts it, so the
// pre-distillation scaffold cannot come back unnoticed.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "policy/dense.h"
#include "policy/weights.h"

namespace policy {

// The same network with its weights supplied by the caller instead of by the
// generated header.
//
// Why this exists: the SIL harness (docs/sil-harness.md) has to run an
// EXPORTED policy -- a .silw blob read at runtime -- through this exact
// arithmetic. Without it the host library would have to reimplement the
// normaliser, the swish layers and the tanh mode, which is precisely the class
// of bug ("swish-vs-tanh in the MLP") the harness exists to catch. So the
// weights become a parameter and forward() below is a thin binding of the
// generated header to it; the on-target path is unchanged, operation for
// operation.
struct Net {
    int obs_dim = 0;
    int act_dim = 0;
    int num_layers = 0;
    const Dense* layers = nullptr;    // num_layers of them, in order
    const float* norm_mean = nullptr;   // obs_dim wide
    const float* norm_std = nullptr;    // obs_dim wide
    // Caller-owned ping-pong scratch, each at least max(obs_dim, widest
    // layer output) floats. No allocation happens inside the forward pass.
    float* scratch_a = nullptr;
    float* scratch_b = nullptr;
};

void forwardNet(const Net& net, const float* obs, float* action);

// Run the deployed policy. `obs` is kObsDim wide, `action` is kActDim wide and
// comes back in [-1, 1].
void forward(const float* obs, float* action);

// Normalise only -- exposed so the host tests can check the normaliser
// separately from the layers when a golden vector disagrees.
void normalize(const float* obs, float* out);

}  // namespace policy
