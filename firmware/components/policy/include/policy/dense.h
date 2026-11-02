// One flax Dense layer, as exported.
//
// `w` is row-major (in, out) exactly as JAX/flax stores the kernel, so
//   y[j] = b[j] + sum_i x[i] * w[i * out + j]
// and the exporter never has to transpose (a transpose is one more place for
// a deploy-day mistake to hide).
#pragma once
#include <stdint.h>

namespace policy {

struct Dense {
    uint16_t in;
    uint16_t out;
    const float* w;
    const float* b;
};

// One layer of a QUANTIZED profile (tools/gen_policy_weights.py --quant int8,
// policy/q8.h). Unlike Dense, `w` is OUTPUT-major -- row j is output j's
// int8 weights over the inputs, zero-padded from `in` to `in_pad` (a multiple
// of 4) -- because the int8 kernels are dot products, one output at a time.
// The generator writes it that way and the golden vectors pin it: a transpose
// mistake cannot pass them. The array carries 4 bytes of slack past the last
// row (the MAC16 kernel's weight prefetch reads one word ahead).
//
//   y[j] = b[j] + scale[j] * s_x * sum_i xq[i] * w[j * in_pad + i]
//
// `out` is the number of outputs KEPT: the head stores only the act_dim mean
// logits, not the log-std half inference never reads.
struct DenseQ8 {
    uint16_t in;
    uint16_t in_pad;
    uint16_t out;
    const int8_t* w;       // out * in_pad (+4 slack)
    const float* scale;    // out: per-output-channel weight scale
    const float* b;        // out
};

}  // namespace policy
