// The int8 forward pass of a QUANTIZED policy profile.
//
// Why it exists (docs/firmware-design.md section 6.1): the ESP32 streams a
// float net's weights out of SPI flash on every tick -- 32 KB of flash cache
// against a 160 KB (128,128) net -- and that streaming, not the FPU, is most
// of the 7.8 ms the v6legs net costs. A (256,256) float net is 445 KB: ~22 ms,
// more than the whole 20 ms tick. int8 weights are a quarter of the bytes
// (108 KB for (256,256), small enough to stream in ~4 ms), and the integer
// dot product runs on the LX6's MAC16 unit at ~2.5 cycles per MAC.
//
// The arithmetic, which tools/policy_q8.py mirrors operation for operation:
//
//   x      = (obs - mean) / std                 as the float net
//   per layer:
//     s_x, xq = quantizeActs(x)                 per-tensor int16, dynamic
//     acc_j   = sum_i xq_i * w_ji               exact int32
//     y_j     = float(acc_j) * (s_x * scale_j) + b_j
//     x       = swishFast(y)                    hidden layers only
//   action = tanh(y[0..act_dim))                the head keeps only the mean
//
// The integer sum is exact, so the C kernel and the MAC16 kernel agree to the
// bit (policy::init() checks that on the robot before trusting MAC16). The
// float steps are single IEEE operations in a fixed order; q8.cpp is built
// with -ffp-contract=off so no fused multiply-add can reorder a rounding.
// Host, target and numpy then agree to the last bit up to the final tanhf.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "policy/dense.h"

namespace policy {

inline constexpr int kQ8MaxIn = 512;      // int32 accumulator headroom
inline constexpr int16_t kQ8ActMax = 32767;
inline constexpr int8_t kQ8WeightMax = 127;

inline constexpr int pad4(int n) { return (n + 3) & ~3; }

enum class Kernel : uint8_t {
    kC = 0,        // portable C, every build
    kMac16 = 1,    // Xtensa MAC16 (ESP32), q8_mac16.S
};

const char* kernelName(Kernel k);
// True where q8RowsMac16 exists (an Xtensa build with the MAC16 option).
bool haveMac16();

// e^z, |z| clamped to 80: range reduction to 2^n * 2^f, |f| <= 0.5, and a
// degree-6 polynomial for 2^f (~1.2e-7 relative; rounding z * log2(e) to
// float adds ~6e-8 * |z|: 5e-7 at |z| = 10). Reproducible bit for bit on
// every IEEE single-precision target, unlike libm's expf -- and ~5x cheaper
// on the ESP32, where expf is a call chain through __ieee754_expf, finitef
// and __divsf3.
float expFast(float z);
// y / (1 + expFast(-y)) -- brax's swish to ~1e-6 absolute.
float swishFast(float y);

// x[0..n) -> xq[0..n), zero-padded to n_pad; returns the scale s_x with
// x ~= s_x * xq. All-zero x -> scale 1, xq all 0.
float quantizeActs(const float* x, int n, int n_pad, int16_t* xq);

// acc[j] = sum_{i < in_pad} x[i] * w[j * in_pad + i], j < out. in_pad is a
// multiple of 4 and at most kQ8MaxIn. x is 4-byte aligned with 4 bytes of
// readable slack past x[in_pad - 1]; so is w past its last row.
void q8RowsC(const int16_t* x, const int8_t* w, int in_pad, int out,
             int32_t* acc);
#if defined(__XTENSA__)
extern "C" void q8RowsMac16(const int16_t* x, const int8_t* w, int in_pad,
                            int out, int32_t* acc);
#endif

// Round-to-nearest int8 quantization of a float flax Dense (in, out), keeping
// the first out_keep outputs: the run-time path for timing a FLOAT profile's
// net as int8 (`nettime`). The deployed int8 headers are quantized offline
// with error feedback (tools/policy_q8.py, GPTQ), which this does not repeat.
// wq: out_keep * pad4(in) + 4 bytes; scale, b: out_keep floats.
void quantizeDense(const Dense& L, int out_keep, int8_t* wq, float* scale,
                   float* b);

struct NetQ8 {
    int obs_dim = 0;
    int act_dim = 0;
    int num_layers = 0;
    const DenseQ8* layers = nullptr;
    const float* norm_mean = nullptr;
    const float* norm_std = nullptr;
    // Caller-owned scratch, sized by scratchQ8*(): float activations, the
    // int16 activations (+slack, 4-byte aligned), the int32 accumulators.
    float* xf = nullptr;
    int16_t* xq = nullptr;
    int32_t* acc = nullptr;
};

// Scratch sizes for a net whose widest input or output is `max_width`.
inline constexpr int scratchQ8Floats(int max_width) { return max_width; }
inline constexpr int scratchQ8Int16(int max_width) { return pad4(max_width) + 4; }
inline constexpr int scratchQ8Int32(int max_width) { return max_width; }

void forwardNetQ8(const NetQ8& net, const float* obs, float* action,
                  Kernel k = Kernel::kC);

}  // namespace policy
