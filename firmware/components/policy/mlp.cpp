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

void forward(const float* obs, float* action) {
    static_assert(kObsDim <= kMaxWidth, "scratch buffer too small for obs");
    normalize(obs, buf_a);

    const float* src = buf_a;
    float* dst = buf_b;
    for (int l = 0; l < kNumLayers; ++l) {
        dense(kLayers[l], src, dst, l + 1 < kNumLayers);
        const float* tmp = src;
        src = dst;
        dst = const_cast<float*>(tmp);
    }
    // `src` now holds 2 * kActDim logits: [mean | log_std]. Deterministic
    // inference is the tanh-normal's mode, i.e. tanh(mean); the std half is
    // only used while training.
    for (int i = 0; i < kActDim; ++i) action[i] = tanhf(src[i]);
}

}  // namespace policy
