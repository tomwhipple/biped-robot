// Static-MLP forward pass for the deployed brax/PPO policy.
//
// What the training side actually produces (verified against
// sim/runs/loco_v5t/params.pkl, 2026-07-26):
//
//   * observations are normalised by brax's running_statistics:
//       x = (obs - mean) / std,  element-wise over the FULL stacked obs
//   * hidden layers are flax Dense with kernel shape (in, out), row-major,
//     activation = linen.swish  (x * sigmoid(x))  -- brax's make_ppo_networks
//     default (docs/firmware-design.md section 5.2). Only the final mode is a
//     tanh; the hidden layers are not.
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

// -- frozen channels (docs/sil-harness.md finding 2) -------------------------
// brax floors the normaliser std at 1e-6, so an obs slot training never varied
// turns any deviation d into d * 1e6 at the first layer: the net has never seen
// that input move, and its first-layer weights for it are untrained noise
// scaled by 1e6. The only value it is defined at is the one training fed it,
// which the normaliser recorded exactly as that slot's mean.
//
// A slot is FROZEN when its std is at or below kFrozenStdMax (the floor is
// 1e-6; every channel training varied sits orders of magnitude above).
inline constexpr float kFrozenStdMax = 1e-5f;

// Pin v[0..n), the obs slots off .. off+n-1 of the NEWEST frame, to their
// trained value wherever the net's normaliser says that slot is frozen.
// Returns the bitmask of entries that were not already exactly that value.
// The control loop runs every command channel through this, so a policy that
// trained with a constant crouch (cmd[3] = 1) or no lift / foot channels
// (cmd[4..6] = 0) is fed exactly that, whatever the link or the pack guard
// asks for -- such a policy cannot crouch or lift on command anyway.
uint32_t pinFrozenNet(const Net& net, int off, int n, float* v);
// The same for the compiled-in net (policy/weights.h).
uint32_t pinFrozen(int off, int n, float* v);
// Is obs slot i of the compiled-in net frozen?
bool isFrozen(int i);

}  // namespace policy
