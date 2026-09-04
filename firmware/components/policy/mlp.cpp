#include "policy/mlp.h"

#include <math.h>

namespace policy {
namespace {

// No heap after init: two ping-pong scratch buffers sized to the widest layer.
// Single owner (the control task), so no locking either.
float buf_a[kMaxWidth];
float buf_b[kMaxWidth];

inline float swish(float x) { return x / (1.0f + expf(-x)); }

void dense(const Dense& L, const float* x, float* y, bool activate) {
    for (uint16_t j = 0; j < L.out; ++j) y[j] = L.b[j];
    for (uint16_t i = 0; i < L.in; ++i) {
        const float xi = x[i];
        if (xi == 0.0f) continue;
        const float* row = L.w + static_cast<size_t>(i) * L.out;
        for (uint16_t j = 0; j < L.out; ++j) y[j] += xi * row[j];
    }
    if (activate) {
        for (uint16_t j = 0; j < L.out; ++j) y[j] = swish(y[j]);
    }
}

}  // namespace

void normalize(const float* obs, float* out) {
    for (int i = 0; i < kObsDim; ++i) {
        out[i] = (obs[i] - kNormMean[i]) / kNormStd[i];
    }
}

void forwardNet(const Net& net, const float* obs, float* action) {
    for (int i = 0; i < net.obs_dim; ++i) {
        net.scratch_a[i] = (obs[i] - net.norm_mean[i]) / net.norm_std[i];
    }
    // Ping-pong scratch buffers: `src` is read-only to `dense()` but the
    // buffers themselves are mutable, so both pointers are plain float*.
    float* src = net.scratch_a;
    float* dst = net.scratch_b;
    for (int l = 0; l < net.num_layers; ++l) {
        dense(net.layers[l], src, dst, l + 1 < net.num_layers);
        float* tmp = src;
        src = dst;
        dst = tmp;
    }
    // `src` now holds 2 * act_dim logits: [mean | log_std]. Deterministic
    // inference is the tanh-normal's mode, i.e. tanh(mean); the std half is
    // only used while training.
    for (int i = 0; i < net.act_dim; ++i) action[i] = tanhf(src[i]);
}

void forward(const float* obs, float* action) {
    // cppcheck-suppress knownConditionTrueFalse -- a compile-time guarantee,
    // not a runtime check: the scratch buffer is sized to the widest layer.
    static_assert(kObsDim <= kMaxWidth, "scratch buffer too small for obs");
    Net net;
    net.obs_dim = kObsDim;
    net.act_dim = kActDim;
    net.num_layers = kNumLayers;
    net.layers = kLayers;
    net.norm_mean = kNormMean;
    net.norm_std = kNormStd;
    net.scratch_a = buf_a;
    net.scratch_b = buf_b;
    forwardNet(net, obs, action);
}

}  // namespace policy
