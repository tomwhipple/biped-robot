// The int8 forward pass (policy/q8.h). Built with -ffp-contract=off (the
// component's CMakeLists.txt; the host build never fuses at its default
// flags): every float statement below is one IEEE operation, in the order
// tools/policy_q8.py performs it.
#include "policy/q8.h"

#include <math.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define Q8_HOT IRAM_ATTR
#else
#define Q8_HOT
#endif

namespace policy {
namespace {

constexpr float kLog2e = 1.44269504088896341f;
constexpr float kExpClamp = 80.0f;
// 2^f = e^(f ln2), Taylor coefficients ln2^k / k!, k = 0..6.
constexpr float kC0 = 1.0f;
constexpr float kC1 = 0.693147180559945f;
constexpr float kC2 = 0.240226506959101f;
constexpr float kC3 = 0.0555041086648216f;
constexpr float kC4 = 0.00961812910762848f;
constexpr float kC5 = 0.00133335581464284f;
constexpr float kC6 = 0.000154035303933816f;

// Round half away from zero, in float: (v >= 0) ? (int)(v + 0.5) : -(int)(0.5 - v).
// Callers keep |v| far inside int32; NaN maps to 0 rather than to UB.
Q8_HOT inline int32_t roundHalfAway(float v) {
    if (v >= 0.0f) {
        const float t = v + 0.5f;
        return static_cast<int32_t>(t);
    }
    if (!(v < 0.0f)) return 0;
    const float t = 0.5f - v;
    return -static_cast<int32_t>(t);
}

Q8_HOT inline float pow2i(int32_t n) {      // exact 2^n, |n| <= 126
    const uint32_t bits = static_cast<uint32_t>(n + 127) << 23;
    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

}  // namespace

const char* kernelName(Kernel k) {
    switch (k) {
        case Kernel::kC: return "c";
        case Kernel::kMac16: return "mac16";
    }
    return "?";
}

bool haveMac16() {
#if defined(__XTENSA__)
    return true;
#else
    return false;
#endif
}

Q8_HOT float expFast(float z) {
    if (z > kExpClamp) z = kExpClamp;
    if (z < -kExpClamp) z = -kExpClamp;
    const float t = z * kLog2e;
    const int32_t n = roundHalfAway(t);
    const float f = t - static_cast<float>(n);
    float p = kC6;
    p = p * f;
    p = p + kC5;
    p = p * f;
    p = p + kC4;
    p = p * f;
    p = p + kC3;
    p = p * f;
    p = p + kC2;
    p = p * f;
    p = p + kC1;
    p = p * f;
    p = p + kC0;
    return p * pow2i(n);
}

Q8_HOT float swishFast(float y) {
    const float e = expFast(-y);
    const float d = 1.0f + e;
    return y / d;
}

Q8_HOT float quantizeActs(const float* x, int n, int n_pad, int16_t* xq) {
    float m = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float a = fabsf(x[i]);
        if (a > m) m = a;
    }
    if (!(m > 0.0f) || !(m <= 3.0e38f)) {       // all zero, or inf
        for (int i = 0; i < n_pad; ++i) xq[i] = 0;
        return 1.0f;
    }
    const float inv = static_cast<float>(kQ8ActMax) / m;
    const float s = m / static_cast<float>(kQ8ActMax);
    for (int i = 0; i < n; ++i) {
        const float v = x[i] * inv;
        int32_t q = roundHalfAway(v);
        if (q > kQ8ActMax) q = kQ8ActMax;
        if (q < -kQ8ActMax) q = -kQ8ActMax;
        xq[i] = static_cast<int16_t>(q);
    }
    for (int i = n; i < n_pad; ++i) xq[i] = 0;
    return s;
}

Q8_HOT void q8RowsC(const int16_t* x, const int8_t* w, int in_pad, int out,
                    int32_t* acc) {
    for (int j = 0; j < out; ++j) {
        const int8_t* r = w + static_cast<size_t>(j) * static_cast<size_t>(in_pad);
        int32_t a = 0;
        for (int i = 0; i < in_pad; i += 4) {
            a += static_cast<int32_t>(x[i]) * r[i] +
                 static_cast<int32_t>(x[i + 1]) * r[i + 1] +
                 static_cast<int32_t>(x[i + 2]) * r[i + 2] +
                 static_cast<int32_t>(x[i + 3]) * r[i + 3];
        }
        acc[j] = a;
    }
}

void quantizeDense(const Dense& L, int out_keep, int8_t* wq, float* scale,
                   float* b) {
    const int in = L.in;
    const int in_pad = pad4(in);
    for (int j = 0; j < out_keep; ++j) {
        float m = 0.0f;
        for (int i = 0; i < in; ++i) {
            const float a = fabsf(L.w[static_cast<size_t>(i) * L.out + static_cast<size_t>(j)]);
            if (a > m) m = a;
        }
        const float s = m > 0.0f ? m / static_cast<float>(kQ8WeightMax) : 1.0f;
        int8_t* row = wq + static_cast<size_t>(j) * static_cast<size_t>(in_pad);
        for (int i = 0; i < in; ++i) {
            const float v = L.w[static_cast<size_t>(i) * L.out + static_cast<size_t>(j)] / s;
            int32_t q = roundHalfAway(v);
            if (q > kQ8WeightMax) q = kQ8WeightMax;
            if (q < -kQ8WeightMax) q = -kQ8WeightMax;
            row[i] = static_cast<int8_t>(q);
        }
        for (int i = in; i < in_pad; ++i) row[i] = 0;
        scale[j] = s;
        b[j] = L.b[j];
    }
    memset(wq + static_cast<size_t>(out_keep) * static_cast<size_t>(in_pad), 0, 4);
}

Q8_HOT void forwardNetQ8(const NetQ8& net, const float* obs, float* action,
                         Kernel k) {
    for (int i = 0; i < net.obs_dim; ++i) {
        const float d = obs[i] - net.norm_mean[i];
        net.xf[i] = d / net.norm_std[i];
    }
    for (int l = 0; l < net.num_layers; ++l) {
        const DenseQ8& L = net.layers[l];
        const bool last = l + 1 == net.num_layers;
        const float sx = quantizeActs(net.xf, L.in, L.in_pad, net.xq);
#if defined(__XTENSA__)
        if (k == Kernel::kMac16) {
            q8RowsMac16(net.xq, L.w, L.in_pad, L.out, net.acc);
        } else {
            q8RowsC(net.xq, L.w, L.in_pad, L.out, net.acc);
        }
#else
        (void)k;
        q8RowsC(net.xq, L.w, L.in_pad, L.out, net.acc);
#endif
        for (int j = 0; j < L.out; ++j) {
            const float c = sx * L.scale[j];
            float y = static_cast<float>(net.acc[j]) * c;
            y = y + L.b[j];
            net.xf[j] = last ? y : swishFast(y);
        }
    }
    for (int i = 0; i < net.act_dim; ++i) action[i] = tanhf(net.xf[i]);
}

}  // namespace policy
