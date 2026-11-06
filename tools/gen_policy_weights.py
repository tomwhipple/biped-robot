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

QUANTIZED (--quant int8, a policy profile for a net too big to stream as
float; docs/firmware-design.md section 6.1)
      .venv/bin/python tools/gen_policy_weights.py --run <run> \
          --profile <name> --quant int8

    The header carries int8 OUTPUT-major kernels with per-output float
    scales (policy::DenseQ8, POLICY_WEIGHTS_Q8), quantized offline with GPTQ
    error feedback on CPU-plant rollouts of the float net
    (tools/policy_q8.py), the head cut to its act_dim mean logits, and one
    self-test case the firmware checks at boot.  The golden vectors are the
    numpy mirror of the firmware's int8 arithmetic (policy_q8.forward_q8),
    which the firmware reproduces to the bit up to the final tanhf; how far
    that lands from the float net is reported here and pinned by
    test_sil_golden against brax.  A profile that is already int8 stays int8
    when regenerated without --quant.

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
sys.path.insert(0, HERE)
import policy_profile                                  # noqa: E402
import policy_q8                                       # noqa: E402

# The deployed set; --profile <name> reads and writes firmware/policies/<name>/
# instead (tools/policy_profile.py). main() rebinds these three.
WEIGHTS_OUT = policy_profile.files()["weights_h"]
VEC_OUT = policy_profile.files()["policy_vectors_h"]
SPEC_H = policy_profile.files()["spec_h"]
RUNS = os.path.join(ROOT, "sim", "runs")
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


def iarr(name, v, per_line=24):
    v = np.ravel(v)
    lines = ["alignas(4) inline constexpr int8_t %s[%d] = {" % (name, v.size)]
    for i in range(0, v.size, per_line):
        lines.append("    " + ",".join(str(int(x)) for x in v[i:i + per_line])
                     + ",")
    lines.append("};")
    return lines


def q8_lines(qlayers, selftest):
    """The int8 half of a quantized weights.h (policy::DenseQ8)."""
    L = []
    for i, (wq, scale, b, n_in) in enumerate(qlayers):
        k, in_pad = wq.shape
        L.append("// layer %d: int8 (%d out, %d in -> %d padded), OUTPUT-major,"
                 " + 4 bytes slack" % (i, k, n_in, in_pad))
        L += iarr("kQW%d" % i, np.concatenate([wq.ravel(),
                                               np.zeros(4, np.int8)]))
        L += carr("kQS%d" % i, scale)
        L += carr("kQB%d" % i, b)
        L.append("")
    L.append("inline constexpr DenseQ8 kLayersQ8[kNumLayers] = {")
    for i, (wq, scale, b, n_in) in enumerate(qlayers):
        L.append("    {%d, %d, %d, kQW%d, kQS%d, kQB%d},"
                 % (n_in, wq.shape[1], wq.shape[0], i, i, i))
    L.append("};")
    L.append("")
    L.append("// policy::init()'s boot check: this raw obs (a calibration-rollout")
    L.append("// state) must give this action (tools/policy_q8.py forward_q8).")
    L += carr("kSelfTestObs", selftest[0])
    L += carr("kSelfTestAction", selftest[1])
    L.append("")
    return L


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--run", default=None,
                   help="run name; reads sim/sil/weights/<run>.silw")
    g.add_argument("--silw", default=None, help="explicit .silw path")
    g.add_argument("--placeholder", action="store_true",
                   help="regenerate the pseudo-random scaffold instead")
    ap.add_argument("--cases", type=int, default=8)
    ap.add_argument("--profile", default=None,
                    help="policy profile <name>: read its obs spec and write "
                         "its weights under firmware/policies/<name>/ "
                         "(tools/policy_profile.py)")
    ap.add_argument("--quant", choices=("none", "int8"), default=None,
                    help="int8: a quantized profile (policy/q8.h). Default: "
                         "whatever the target header already is")
    args = ap.parse_args()
    global SPEC_H, WEIGHTS_OUT, VEC_OUT
    profile = policy_profile.check(args.profile)
    out = policy_profile.files(profile)
    SPEC_H, WEIGHTS_OUT, VEC_OUT = (out["spec_h"], out["weights_h"],
                                    out["policy_vectors_h"])

    if not (args.run or args.silw or args.placeholder):
        raise SystemExit(
            "pick a mode: --run <run> / --silw <path> for the real distilled "
            "net, or --placeholder for the scaffold.  There is no default: "
            "the deployed header is real weights, and an argument-less run "
            "would silently un-deploy the policy.")

    quant = args.quant
    if quant is None:
        was_q8 = (os.path.exists(WEIGHTS_OUT)
                  and "#define POLICY_WEIGHTS_Q8" in open(WEIGHTS_OUT).read())
        quant = "int8" if was_q8 else "none"
        if was_q8:
            print("  %s is int8: regenerating it int8 (--quant none to "
                  "change that)" % os.path.relpath(WEIGHTS_OUT, ROOT))
    if quant == "int8" and (args.placeholder or profile is None):
        raise SystemExit("--quant int8 is for a policy PROFILE's real net "
                         "(--profile <name> --run <run>): the deployed set "
                         "and the placeholder stay float")

    obs_dim, act_dim = spec_dims()
    placeholder = bool(args.placeholder)
    test_net = None
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
        # An UNTRAINED net made to test the firmware path
        # (tools/make_test_policy_run.py) compiles as a placeholder: the
        # firmware refuses to arm on it (cli.cpp armRefused).
        cfg_path = os.path.join(RUNS, info["run"] or "", "config.json")
        if info["run"] and os.path.exists(cfg_path):
            with open(cfg_path) as fh:
                test_net = json.load(fh).get("test_net")

    sizes = info["sizes"]
    qlayers = None
    selftest = None
    if quant == "int8":
        print("  int8: calibration rollouts of the float net on the CPU "
              "plant ...")
        cal = policy_q8.calibration_obs(ROOT, info["run"], mean, std, layers,
                                        act_dim)
        qlayers = policy_q8.gptq_quantize_net(mean, std, layers, act_dim, cal)
        a_f = policy_q8.float_forward(mean, std, layers, cal, act_dim)
        a_q = np.array([policy_q8.forward_q8(mean, std, qlayers, o, act_dim)
                        for o in cal])
        e = np.abs(a_q - a_f)
        print("  int8 vs float on %d calibration obs: action err max %.4f "
              "p99 %.4f rms %.4f" % (len(cal), e.max(),
                                     np.percentile(e, 99),
                                     np.sqrt(np.mean(e ** 2))))
        so = cal[len(cal) // 3]
        selftest = (so, policy_q8.forward_q8(mean, std, qlayers, so, act_dim))
        fwd = lambda o: policy_q8.forward_q8(mean, std, qlayers, o, act_dim)
    else:
        fwd = lambda o: forward(mean, std, layers, o, act_dim)
    cases = [(o, fwd(o))
             for o in obs_cases(mean, std, obs_dim, placeholder, args.cases)]
    if qlayers is not None:
        for k, (o, a) in enumerate(cases):
            z = float(np.max(np.abs((o - mean) / std)))
            e = float(np.max(np.abs(a - forward(mean, std, layers, o,
                                                 act_dim))))
            print("  int8 vs float, golden case %d (max |z| %.3g): %.4f%s"
                  % (k, z, e, "  <- a frozen channel 1e6 sigma out: no "
                     "int8 rounding matches float there, and the firmware "
                     "never feeds it (pinFrozen)" if z > 1e3 else ""))

    # -- weights header ----------------------------------------------------
    L = []
    w = L.append
    w("// GENERATED by tools/gen_policy_weights.py -- do not edit.")
    w("//")
    if test_net is not None:
        w("// *** UNTRAINED TEST NET -- NOT A POLICY. ***")
        w("// %s: tools/make_test_policy_run.py, seed %s. Built at the real"
          % (info["run"], test_net.get("seed")))
        w("// widths through the real exporter so the firmware path is")
        w("// golden-tested end to end; kWeightsArePlaceholder is true, and")
        w("// the firmware refuses to arm on it.")
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
        if test_net is None:
            w("// Distilled from the training net by sim/mjx/distill_student.py")
            w("// (firmware-design section 6) and refereed on both columns --")
            w("// eval_precision.py and eval_precision.py --sil -- before this")
            w("// header was written.  constexpr, so it lives in flash DROM.")
        else:
            w("// constexpr, so it lives in flash DROM.")
    if qlayers is not None:
        w("//")
        w("// INT8 (policy/q8.h): output-major int8 kernels, per-output float")
        w("// scales, GPTQ-rounded on CPU-plant rollouts (tools/policy_q8.py);")
        w("// the head keeps its %d mean logits only. policy::init() copies"
          % act_dim)
        w("// what fits into RAM and checks kSelfTest* before `run` may arm.")
    w("#pragma once")
    L.extend(policy_profile.guard_lines(profile, "weights.h"))
    w("#include <stddef.h>")
    w("#include <stdint.h>")
    w("")
    w('#include "policy/dense.h"')
    w("")
    w("namespace policy {")
    w("")
    w("inline constexpr bool kWeightsArePlaceholder = %s;"
      % ("true" if placeholder or test_net is not None else "false"))
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
    if qlayers is not None:
        L += q8_lines(qlayers, selftest)
        layers_out = []
    else:
        layers_out = layers
    for i, (wt, b) in enumerate(layers_out):
        w("// layer %d: (%d, %d), flax kernel order (in, out) row-major"
          % (i, wt.shape[0], wt.shape[1]))
        L += carr("kW%d" % i, wt, per_line=8)
        L += carr("kB%d" % i, b, per_line=8)
        w("")
    if qlayers is None:
        w("inline constexpr Dense kLayers[kNumLayers] = {")
        for i, (wt, b) in enumerate(layers):
            w("    {%d, %d, kW%d, kB%d}," % (wt.shape[0], wt.shape[1], i, i))
        w("};")
        w("")
    w("}  // namespace policy")
    w("")
    if qlayers is not None:
        # the macro the firmware's #ifdefs key on, ahead of the namespace
        at = L.index("namespace policy {")
        L[at:at] = ["// policy/q8.h: this net runs the int8 forward pass.",
                    "#define POLICY_WEIGHTS_Q8 1", ""]
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
    V.extend(policy_profile.guard_lines(profile, "policy_vectors.h"))
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

    if qlayers is not None:
        nb = sum(q.size + 8 * q.shape[0] + 4 for q, _, _, _ in qlayers)
        print("int8 net: %.1f KB of weights+scales+biases (vs %.1f KB fp32)"
              % (nb / 1024.0, sum(w_.size + b.size for w_, b in layers)
                 * 4 / 1024.0))
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
