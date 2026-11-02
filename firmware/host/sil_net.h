// Runtime loaders for the two files sil_init() takes: an exported policy
// (.silw) and a calibration. Host-only; nothing here is compiled for the
// ESP32, where both live in flash/NVS instead.
//
// Split out of sil_lib.cpp so the golden-vector test can drive an exported net
// directly, without going through a whole control tick.
#pragma once
#include <string>
#include <vector>

#include "obs/actuation.h"
#include "policy/mlp.h"

namespace sil {

// The .silw format (docs/sil-harness.md, "Weight export", as realised by
// tools/export_policy_weights.py):
//
//   <run>.silw        an optional 64-byte binary header --
//                       magic "SILW", u32 version, u32 header_bytes,
//                       u32 obs_dim, u32 act_dim, u32 num_layers,
//                       u32 num_sizes, char activation[16],
//                       u32 layer_sizes[num_sizes]
//                     followed by little-endian f32:
//                       norm_mean (obs_dim), norm_std (obs_dim),
//                       W0 (in0*out0, row-major (in,out) as flax stores it),
//                       b0 (out0), W1, b1, ...
//   <run>.silw.json   {"layer_sizes":[...], "activation":"swish",
//                      "obs_dim":147, "act_dim":10,
//                      "normalizer":{"mean":[...], "std":[...]},
//                      "blob":{"header_bytes":64, "order":[...], ...}}
//
// The header is self-describing, so a headed blob loads with no sidecar at
// all; when a sidecar is present its dims, activation, blob length and
// normalizer are cross-checked against the bytes and a disagreement is an
// error, not a preference. A HEADERLESS blob (the plainest reading of the
// design doc) also loads: then the sidecar is required, and whether the
// normalizer leads the floats is decided by the blob's length.
//
// On an int8 profile build (policy/q8.h) the int8 weights were rounded
// offline with calibration data a .silw does not carry (tools/policy_q8.py,
// GPTQ), so they cannot be rebuilt from one. Loading the .silw of the
// compiled-in run (same run name and layer widths) therefore switches
// forward() to the compiled-in int8 net -- what the robot runs; the float
// layers stay reachable through net(). Any other .silw runs float, exactly
// as on a float build, and quant() says which.
//
// The sidecar is looked for at <path>.json first, then <path minus
// extension>.json; passing the sidecar itself as `path` also works.
class LoadedNet {
  public:
    // Bind to the weights compiled into the firmware (policy/weights.h).
    void loadBuiltin();
    // Read an exported net. False -> err explains which file and why.
    bool load(const char* weights_path, std::string& err);

    const policy::Net& net() const { return net_; }
    int obsDim() const { return net_.obs_dim; }
    int actDim() const { return net_.act_dim; }
    bool builtin() const { return builtin_; }
    const std::string& source() const { return source_; }
    const std::string& activation() const { return activation_; }
    const std::vector<int>& layerSizes() const { return sizes_; }

    // Convenience: normalise + forward, using this net's storage -- on an
    // int8 profile build, the compiled-in int8 net when that is what was
    // loaded (see load()).
    void forward(const float* obs, float* action) const;
    // "int8" when forward() runs the compiled-in int8 net, else "none".
    const char* quant() const { return q8_ ? "int8" : "none"; }

  private:
    void bind();

    bool builtin_ = false;
    std::string source_ = "(none)";
    std::string activation_ = "swish";
    std::vector<int> sizes_;               // layer_sizes, num_layers + 1 long
    std::vector<float> params_;            // kernels + biases, contiguous
    std::vector<float> mean_, std_;
    std::vector<policy::Dense> layers_;
    mutable std::vector<float> scratch_a_, scratch_b_;
    policy::Net net_;
    bool q8_ = false;
#ifdef POLICY_WEIGHTS_Q8
    mutable std::vector<float> q_xf_;
    mutable std::vector<int16_t> q_xq_;
    mutable std::vector<int32_t> q_acc_;
    policy::NetQ8 netq8_;
    void bindQ8();
#endif
};

// JSON calibration: an array of {bus_id, zero_steps, dir}, or an object with
// that array under "joints"/"calibration"/"servos". Every bus ID in
// obs::kServoId must appear exactly once -- a partial file is rejected rather
// than silently left at the nominal 2048.
bool loadCalibration(const char* path, obs::Calibration& cal, std::string& err);

}  // namespace sil
