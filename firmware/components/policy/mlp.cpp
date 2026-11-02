#include "policy/mlp.h"

#include <math.h>
#include <string.h>

namespace policy {
namespace {

#ifdef POLICY_WEIGHTS_Q8
// Q8 scratch, sized to the widest layer; 4-byte alignment for the int16
// activations is what the MAC16 kernel's word loads need.
float q_xf[scratchQ8Floats(kMaxWidth)];
alignas(4) int16_t q_xq[scratchQ8Int16(kMaxWidth)];
int32_t q_acc[scratchQ8Int32(kMaxWidth)];
// The layer table forward() runs: kLayersQ8 with each layer's pointers
// re-aimed at its RAM copy when init() made one.
DenseQ8 g_layers[kNumLayers] = {};
bool g_layers_bound = false;
#else
// No heap after init: two ping-pong scratch buffers sized to the widest layer.
// Single owner (the control task), so no locking either.
float buf_a[kMaxWidth];
float buf_b[kMaxWidth];
#endif

Status g_status;

inline float swish(float x) { return x / (1.0f + expf(-x)); }

// y[j] for j < n_out of a layer whose kernel rows are L.out wide.
void dense(const Dense& L, const float* x, float* y, bool activate, int n_out) {
    for (int j = 0; j < n_out; ++j) y[j] = L.b[j];
    for (uint16_t i = 0; i < L.in; ++i) {
        const float xi = x[i];
        if (xi == 0.0f) continue;
        const float* row = L.w + static_cast<size_t>(i) * L.out;
        for (int j = 0; j < n_out; ++j) y[j] += xi * row[j];
    }
    if (activate) {
        for (int j = 0; j < n_out; ++j) y[j] = swish(y[j]);
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
        const bool last = l + 1 == net.num_layers;
        // The head is 2 * act_dim logits, [mean | log_std]; inference only
        // reads the mean half, so only that half is computed. Each output is
        // the same sum in the same order either way: bit-identical.
        const int n_out = last && net.act_dim > 0 && net.act_dim < net.layers[l].out
                              ? net.act_dim
                              : net.layers[l].out;
        dense(net.layers[l], src, dst, !last, n_out);
        float* tmp = src;
        src = dst;
        dst = tmp;
    }
    // Deterministic inference is the tanh-normal's mode, i.e. tanh(mean).
    for (int i = 0; i < net.act_dim; ++i) action[i] = tanhf(src[i]);
}

uint32_t pinFrozenNet(const Net& net, int off, int n, float* v) {
    uint32_t changed = 0;
    for (int i = 0; i < n && i < 32; ++i) {
        const int k = off + i;
        if (k < 0 || k >= net.obs_dim) continue;
        if (!(net.norm_std[k] <= kFrozenStdMax)) continue;
        if (v[i] != net.norm_mean[k]) changed |= 1u << i;
        v[i] = net.norm_mean[k];
    }
    return changed;
}

uint32_t pinFrozen(int off, int n, float* v) {
    Net net;
    net.obs_dim = kObsDim;
    net.norm_mean = kNormMean;
    net.norm_std = kNormStd;
    return pinFrozenNet(net, off, n, v);
}

bool isFrozen(int i) {
    return i >= 0 && i < kObsDim && kNormStd[i] <= kFrozenStdMax;
}

#ifdef POLICY_WEIGHTS_Q8

namespace {

size_t layerWeightBytes(const DenseQ8& L) {
    return static_cast<size_t>(L.out) * L.in_pad + 4u;
}

void bindLayers() {
    if (g_layers_bound) return;
    for (int l = 0; l < kNumLayers; ++l) g_layers[l] = kLayersQ8[l];
    g_layers_bound = true;
}

NetQ8 compiledNet(const DenseQ8* layers) {
    NetQ8 net;
    net.obs_dim = kObsDim;
    net.act_dim = kActDim;
    net.num_layers = kNumLayers;
    net.layers = layers;
    net.norm_mean = kNormMean;
    net.norm_std = kNormStd;
    net.xf = q_xf;
    net.xq = q_xq;
    net.acc = q_acc;
    return net;
}

}  // namespace

void forward(const float* obs, float* action) {
    bindLayers();
    forwardNetQ8(compiledNet(g_layers), obs, action, g_status.kernel);
}

void init(void* (*alloc)(size_t), size_t dram_budget) {
    bindLayers();
    Status st;
    st.quantized = true;
    st.num_layers = kNumLayers;
    for (int l = 0; l < kNumLayers; ++l) {
        st.weight_bytes += static_cast<uint32_t>(
            layerWeightBytes(kLayersQ8[l]) + 8u * kLayersQ8[l].out);
    }

    // Placement: largest layer first while the budget lasts. Each byte in RAM
    // is a byte not streamed from flash, so bytes moved is what counts.
    if (alloc) {
        bool done[kNumLayers] = {};
        for (int pass = 0; pass < kNumLayers; ++pass) {
            int best = -1;
            for (int l = 0; l < kNumLayers; ++l) {
                if (done[l]) continue;
                if (best < 0 ||
                    layerWeightBytes(kLayersQ8[l]) > layerWeightBytes(kLayersQ8[best])) {
                    best = l;
                }
            }
            done[best] = true;
            const size_t n = layerWeightBytes(kLayersQ8[best]);
            if (st.ram_bytes + n > dram_budget) continue;
            auto* w = static_cast<int8_t*>(alloc(n));
            if (!w) continue;
            memcpy(w, kLayersQ8[best].w, n);
            g_layers[best].w = w;
            st.ram_bytes += static_cast<uint32_t>(n);
            ++st.layers_in_ram;
        }
    }

    // Self-test: C against the generator's numpy action, then MAC16 against C.
    float a_c[kActDim];
    forwardNetQ8(compiledNet(g_layers), kSelfTestObs, a_c, Kernel::kC);
    st.selftest_ran = true;
    for (int i = 0; i < kActDim; ++i) {
        const float e = fabsf(a_c[i] - kSelfTestAction[i]);
        if (!(e <= st.selftest_err)) st.selftest_err = e;   // NaN sticks
    }
    st.selftest_ok = st.selftest_err <= kSelfTestTol;
    st.kernel = Kernel::kC;
    if (haveMac16()) {
        float a_m[kActDim];
        forwardNetQ8(compiledNet(g_layers), kSelfTestObs, a_m, Kernel::kMac16);
        st.mac16_checked = true;
        st.mac16_matches = memcmp(a_m, a_c, sizeof a_c) == 0;
        if (st.mac16_matches) st.kernel = Kernel::kMac16;
    }
    g_status = st;
}

#else  // float profile

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

void init(void* (*alloc)(size_t), size_t dram_budget) {
    (void)alloc;
    (void)dram_budget;
    Status st;
    st.num_layers = kNumLayers;
    for (int l = 0; l < kNumLayers; ++l) {
        st.weight_bytes += static_cast<uint32_t>(
            4u * (static_cast<size_t>(kLayers[l].in) * kLayers[l].out + kLayers[l].out));
    }
    g_status = st;
}

#endif

const Status& status() { return g_status; }

}  // namespace policy
