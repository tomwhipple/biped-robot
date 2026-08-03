#!/usr/bin/env python3
"""Generate the firmware's policy weights header + forward-pass golden vectors.

Two modes, one header contract:

  REAL (the deployed artifact)
      .venv/bin/python tools/gen_policy_weights.py --run <run>
      .venv/bin/python tools/gen_policy_weights.py --silw path/to/x.silw

    Reads the exported .silw blob written by tools/export_policy_weights.py
    (params.pkl -> little-endian f32 + self-describing SILW header, with a
    JSON sidecar cross-checked when present) and compiles it into
    firmware/components/policy/include/policy/weights.h with
    kWeightsArePlaceholder = false.  The net that goes on the robot must be
    small enough to live in flash as a constexpr -- firmware-design section 6
    says distill the (512, 256, 128) training net to ~(128, 128); see
    sim/mjx/distill_student.py.  The referee gates the artifact BEFORE it is
    compiled in: both scorecard columns, python and --sil, on the run whose
    .silw this reads.

  PLACEHOLDER (the pre-distillation scaffold, kept for regeneration/CI)
      .venv/bin/python tools/gen_policy_weights.py --placeholder

    A pseudo-random net at the real input/output widths.  It exists so the
    forward pass and its test harness were real code before any real weights
    existed; kWeightsArePlaceholder is true and test_policy asserts it.

Either way the golden vectors in firmware/host/vectors/policy_vectors.h are
computed HERE in numpy with the exact brax semantics the firmware reproduces:

    x       = (obs - mean) / std              running_statistics.normalize
    h_i     = swish(h_{i-1} @ W_i + b_i)      linen.swish, NOT tanh
    logits  = h_n @ W_out + b_out             linear, 2 * action_size wide
    action  = tanh(logits[:action_size])      NormalTanhDistribution.mode()

Kernel layout is flax's (in, out) row-major, copied verbatim, so neither the
export nor the firmware ever has to transpose.

Real-mode obs cases are built as raw = mean + std * z from a sampled NORMALIZED
z (the same construction tools/export_policy_weights.py uses for its brax
goldens): sampling raw obs directly would put every channel wildly out of
distribution, and the frozen channels -- normalizer std pinned at the 1e-6
std_eps floor, see docs/sil-harness.md finding 2 -- would land 1e5 sigma out.
"""
import argparse
import hashlib
import json
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

WEIGHTS_OUT = os.path.join(ROOT, "firmware", "components", "policy", "include",
                           "policy", "weights.h")
VEC_OUT = os.path.join(ROOT, "firmware", "host", "vectors",
                       "policy_vectors.h")

SPEC_H = os.path.join(ROOT, "firmware", "components", "obs", "include", "obs",
                      "obs_spec.h")
SILW_DIR = os.path.join(ROOT, "sim", "sil", "weights")

MAGIC = b"SILW"

# Placeholder hidden sizes. Small on purpose: a (512, 256, 128) net would be a
# megabyte of pseudo-random source for a scaffold nobody flashes.
HIDDEN = (32, 32)


def spec_dims():
    """Read kObsDim / kActDim out of the generated obs spec so the two headers
    cannot disagree."""
    txt = open(SPEC_H).read()
    out = {}
    for key in ("kObsDim", "kActDim"):
        for line in txt.splitlines():
            if line.startswith("inline constexpr int %s " % key):
                out[key] = int(line.split("=")[1].strip().rstrip(";"))
    if len(out) != 2:
        raise SystemExit("could not read dims from %s -- run "
                         "tools/gen_obs_spec.py first" % SPEC_H)
    return out["kObsDim"], out["kActDim"]


def spec_run_name():
    for line in open(SPEC_H):
        if line.startswith("inline constexpr const char* kRunName"):
            return line.split('"')[1]
    return None


def fl(x):
    s = "%.9g" % float(x)
    if not any(c in s for c in ".eEn"):
        s += ".0"
    return s + "f"


def carr(name, v, per_line=8):
    v = np.ravel(v)
    lines = ["inline constexpr float %s[%d] = {" % (name, v.size)]
    for i in range(0, v.size, per_line):
        lines.append("    " + ", ".join(fl(x) for x in v[i:i + per_line]) + ",")
    lines.append("};")
    return lines


def swish(x):
    x = np.asarray(x, dtype=np.float32)
    return (x / (1.0 + np.exp(-x, dtype=np.float32))).astype(np.float32)


def forward(mean, std, layers, obs, act_dim):
    """The reference float32 forward pass; the firmware must match this."""
    x = ((np.asarray(obs, dtype=np.float32) - mean) / std).astype(np.float32)
    for i, (w, b) in enumerate(layers):
        x = (x @ w + b).astype(np.float32)
        if i + 1 < len(layers):
            x = swish(x)
    return np.tanh(x[:act_dim]).astype(np.float32)


# ---------------------------------------------------------------------
#  .silw reader (mirror of tools/export_policy_weights.py's writer and of
#  the C reader in firmware/host/sil_net.cpp: header, then norm_mean,
#  norm_std, then (W_i, b_i) in application order, all little-endian f32)
# ---------------------------------------------------------------------
def read_silw(path):
    blob = open(path, "rb").read()
    if blob[:4] != MAGIC:
        raise SystemExit("%s: not a SILW blob (this tool wants the headed "
                         "format, not --raw)" % path)
    (version, header_bytes, obs_dim, act_dim, n_layers,
     n_sizes) = struct.unpack("<6I", blob[4:28])
    if version != 1:
        raise SystemExit("%s: SILW version %d, this tool knows 1"
                         % (path, version))
    act = blob[28:28 + 16].split(b"\0")[0].decode()
    if act != "swish":
        raise SystemExit("%s: activation %r -- the firmware MLP is swish only"
                         % (path, act))
    sizes = list(struct.unpack("<%dI" % n_sizes,
                               blob[28 + 16:28 + 16 + 4 * n_sizes]))
    if 28 + 16 + 4 * n_sizes != header_bytes:
        raise SystemExit("%s: header_bytes %d disagrees with its own fields"
                         % (path, header_bytes))
    if len(sizes) != n_layers + 1 or sizes[0] != obs_dim:
        raise SystemExit("%s: layer sizes %s do not describe %d layers from "
                         "%d inputs" % (path, sizes, n_layers, obs_dim))
    if sizes[-1] != 2 * act_dim:
        raise SystemExit("%s: output width %d is not 2 * act_dim (%d) -- the "
                         "brax policy head is mean || log-std"
                         % (path, sizes[-1], act_dim))

    f = np.frombuffer(blob, dtype="<f4", offset=header_bytes)
    need = 2 * obs_dim + sum(sizes[i] * sizes[i + 1] + sizes[i + 1]
                             for i in range(n_layers))
    if f.size != need:
        raise SystemExit("%s: %d floats after the header, expected %d"
                         % (path, f.size, need))
    at = 0

    def take(n, shape=None):
        nonlocal at
        v = np.array(f[at:at + n], dtype=np.float32)
        at += n
        return v.reshape(shape) if shape else v

    mean = take(obs_dim)
    std = take(obs_dim)
    layers = []
    for i in range(n_layers):
        w = take(sizes[i] * sizes[i + 1], (sizes[i], sizes[i + 1]))
        layers.append((w, take(sizes[i + 1])))
    if np.any(std <= 0.0) or not np.all(np.isfinite(mean)):
        raise SystemExit("%s: normalizer is not usable" % path)

    info = dict(sizes=sizes, obs_dim=obs_dim, act_dim=act_dim,
                sha256=hashlib.sha256(blob).hexdigest(), bytes=len(blob),
                run=os.path.basename(path)[:-len(".silw")])

    # cross-check the sidecar when it is there (the C reader does the same)
    side = path + ".json"
    if os.path.exists(side):
        with open(side) as fh:
            sc = json.load(fh)
        for key, got in (("obs_dim", obs_dim), ("act_dim", act_dim),
                         ("layer_sizes", sizes), ("activation", "swish")):
            if key in sc and sc[key] != got:
                raise SystemExit("%s: sidecar %s=%r but blob says %r"
                                 % (side, key, sc[key], got))
        if sc.get("blob", {}).get("sha256") not in (None, info["sha256"]):
            raise SystemExit("%s: sidecar sha256 does not match the blob "
                             "-- re-run tools/export_policy_weights.py" % side)
        info["run"] = sc.get("run", info["run"])
    return mean, std, layers, info


# ---------------------------------------------------------------------
def placeholder_net(obs_dim, act_dim):
    rng = np.random.default_rng(20260726)
    # Plausible-looking normaliser: non-unit scales so a firmware that forgets
    # to normalise fails loudly rather than "nearly passing".
    mean = rng.uniform(-0.5, 0.5, obs_dim).astype(np.float32)
    std = rng.uniform(0.2, 1.8, obs_dim).astype(np.float32)
    sizes = [obs_dim] + list(HIDDEN) + [2 * act_dim]
    layers = []
    for i in range(len(sizes) - 1):
        lim = np.sqrt(3.0 / sizes[i])          # lecun-uniform, brax's default
        w = rng.uniform(-lim, lim, (sizes[i], sizes[i + 1])).astype(np.float32)
        b = rng.uniform(-0.1, 0.1, sizes[i + 1]).astype(np.float32)
        layers.append((w, b))
    return mean, std, layers, dict(sizes=sizes, obs_dim=obs_dim,
                                   act_dim=act_dim, run=None)


def obs_cases(mean, std, obs_dim, placeholder, n=8, seed=20260803):
    """The raw obs the golden vectors are taken at."""
    rng = np.random.default_rng(seed if not placeholder else 20260726)
    if placeholder:
        # unchanged from the pre-distillation scaffold: raw draws, no
        # normalizer to respect
        return [np.zeros(obs_dim, np.float32) if k == 0
                else rng.normal(0.0, 1.0, obs_dim).astype(np.float32) * (1 + k)
                for k in range(6)]
    out = [np.zeros(obs_dim, np.float32),                # all-zero raw obs
           mean.copy()]                                  # z = 0
    for k in range(n - 2):
        z = (np.ones(obs_dim, np.float32) if k == 0
             else rng.normal(0.0, 0.25 + 0.5 * (k % 5), obs_dim
                             ).astype(np.float32))
        out.append((mean + std * z).astype(np.float32))
    return out


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--run", default=None,
                   help="run name; reads sim/sil/weights/<run>.silw")
    g.add_argument("--silw", default=None, help="explicit .silw path")
    g.add_argument("--placeholder", action="store_true",
                   help="regenerate the pseudo-random scaffold instead")
    ap.add_argument("--cases", type=int, default=8)
    args = ap.parse_args()

    if not (args.run or args.silw or args.placeholder):
        raise SystemExit(
            "pick a mode: --run <run> / --silw <path> for the real distilled "
            "net, or --placeholder for the scaffold.  This tool used to "
            "default to the placeholder; it does not any more, because the "
            "deployed header is now real weights and an argument-less run "
            "would silently un-deploy the policy.")

    obs_dim, act_dim = spec_dims()
    placeholder = bool(args.placeholder)
    if placeholder:
        mean, std, layers, info = placeholder_net(obs_dim, act_dim)
    else:
        path = args.silw or os.path.join(SILW_DIR, "%s.silw" % args.run)
        if not os.path.exists(path):
            raise SystemExit(
                "%s does not exist -- export it first:\n  JAX_PLATFORMS=cpu "
                ".venv/bin/python tools/export_policy_weights.py --run %s "
                "--no-golden" % (path, args.run or "<run>"))
        mean, std, layers, info = read_silw(path)
        if info["obs_dim"] != obs_dim or info["act_dim"] != act_dim:
            raise SystemExit(
                "obs_spec.h says %d obs / %d act but %s holds %d / %d -- "
                "regenerate tools/gen_obs_spec.py --run <deployed run>"
                % (obs_dim, act_dim, os.path.basename(path), info["obs_dim"],
                   info["act_dim"]))
        run_name = spec_run_name()
        if run_name and info["run"] and run_name != info["run"]:
            print("  WARNING: obs_spec.h kRunName is %r but these weights are "
                  "%r -- regenerate the obs spec for the deployed run"
                  % (run_name, info["run"]), file=sys.stderr)

    sizes = info["sizes"]
    cases = [(o, forward(mean, std, layers, o, act_dim))
             for o in obs_cases(mean, std, obs_dim, placeholder, args.cases)]

    # -- weights header ----------------------------------------------------
    L = []
    w = L.append
    w("// GENERATED by tools/gen_policy_weights.py -- do not edit.")
    w("//")
    if placeholder:
        w("// *** PLACEHOLDER WEIGHTS -- NOT THE TRAINED POLICY. ***")
        w("// Correct input/output widths and correct brax semantics (swish")
        w("// hidden layers, 2*act_dim linear output, tanh mode), pseudo-")
        w("// random numbers.  Regenerate the real header with")
        w("//   tools/gen_policy_weights.py --run <deployed run>")
    else:
        w("// Run       : %s" % info["run"])
        w("// Source    : sim/sil/weights/%s.silw (sha256 %s)"
          % (info["run"], info["sha256"][:16]))
        w("// Net       : %s, swish hidden, linear 2*act_dim head, tanh mode"
          % " -> ".join(str(s) for s in sizes))
        w("// Normalizer: brax running_statistics, shipped with the weights")
        w("//             (the firmware net consumes NORMALIZED obs).")
        w("//")
        w("// Distilled from the training net by sim/mjx/distill_student.py")
        w("// (firmware-design section 6) and refereed on both columns --")
        w("// eval_precision.py and eval_precision.py --sil -- before this")
        w("// header was written.  constexpr, so it lives in flash DROM.")
    w("#pragma once")
    w("#include <stddef.h>")
    w("#include <stdint.h>")
    w("")
    w('#include "policy/dense.h"')
    w("")
    w("namespace policy {")
    w("")
    w("inline constexpr bool kWeightsArePlaceholder = %s;"
      % ("true" if placeholder else "false"))
    w('inline constexpr const char* kWeightsRun = "%s";'
      % ("__placeholder__" if placeholder else info["run"]))
    w("inline constexpr int kObsDim = %d;" % obs_dim)
    w("inline constexpr int kActDim = %d;" % act_dim)
    w("inline constexpr int kNumLayers = %d;" % len(layers))
    w("inline constexpr int kMaxWidth = %d;" % max(sizes))
    w("")
    w("// running_statistics.normalize: x = (obs - mean) / std")
    L += carr("kNormMean", mean)
    L += carr("kNormStd", std)
    w("")
    for i, (wt, b) in enumerate(layers):
        w("// layer %d: (%d, %d), flax kernel order (in, out) row-major"
          % (i, wt.shape[0], wt.shape[1]))
        L += carr("kW%d" % i, wt, per_line=8)
        L += carr("kB%d" % i, b, per_line=8)
        w("")
    w("inline constexpr Dense kLayers[kNumLayers] = {")
    for i, (wt, b) in enumerate(layers):
        w("    {%d, %d, kW%d, kB%d}," % (wt.shape[0], wt.shape[1], i, i))
    w("};")
    w("")
    w("}  // namespace policy")
    w("")
    os.makedirs(os.path.dirname(WEIGHTS_OUT), exist_ok=True)
    with open(WEIGHTS_OUT, "w") as f:
        f.write("\n".join(L))

    # -- golden vectors ----------------------------------------------------
    V = []
    v = V.append
    v("// GENERATED by tools/gen_policy_weights.py -- do not edit.")
    v("// numpy-computed obs -> action through the exact brax semantics, for")
    v("// the net in policy/weights.h (%s)."
      % ("PLACEHOLDER" if placeholder else info["run"]))
    if not placeholder:
        v("// obs = mean + std * z for a sampled normalized z, so every case")
        v("// sits where the policy actually lives.")
    v("#pragma once")
    v("#include <stddef.h>")
    v("")
    v("namespace policy_vectors {")
    v("")
    v("inline constexpr int kObsDim = %d;" % obs_dim)
    v("inline constexpr int kActDim = %d;" % act_dim)
    v("")
    v("struct Case { float obs[kObsDim]; float action[kActDim]; };")
    v("inline const Case kCases[] = {")
    for o, a in cases:
        v("    {{" + ", ".join(fl(x) for x in o) + "},")
        v("     {" + ", ".join(fl(x) for x in a) + "}},")
    v("};")
    v("inline constexpr size_t kNumCases = sizeof kCases / sizeof kCases[0];")
    v("")
    v("}  // namespace policy_vectors")
    v("")
    os.makedirs(os.path.dirname(VEC_OUT), exist_ok=True)
    with open(VEC_OUT, "w") as f:
        f.write("\n".join(V))

    nparams = sum(w_.size + b.size for w_, b in layers)
    print("%s net: %s (%d weights + %d normalizer floats, %.1f KB fp32)"
          % ("placeholder" if placeholder else info["run"],
             " -> ".join(str(s) for s in sizes), nparams, 2 * obs_dim,
             (nparams + 2 * obs_dim) * 4 / 1024.0))
    print("wrote %s (%.0f KB source)"
          % (WEIGHTS_OUT, os.path.getsize(WEIGHTS_OUT) / 1024.0))
    print("wrote %s (%d cases)" % (VEC_OUT, len(cases)))


if __name__ == "__main__":
    main()
