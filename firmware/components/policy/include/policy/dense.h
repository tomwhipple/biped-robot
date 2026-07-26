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

}  // namespace policy
