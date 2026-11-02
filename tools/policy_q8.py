"""The int8 policy forward pass, in numpy, operation for operation as the
firmware runs it (firmware/components/policy/q8.cpp).

A QUANTIZED policy profile (tools/gen_policy_weights.py --quant int8) carries
its dense layers as int8 weights with one float scale per output channel, and
runs every layer as

    s_x, xq = quantize_acts(x)            per-tensor int16, dynamic
    acc_j   = sum_i xq_i * wq_ji          exact int32 (C loop or the MAC16
                                          kernel -- bit-identical)
    y_j     = float(acc_j) * (s_x * s_j) + b_j
    x'      = swish_fast(y)               every hidden layer

with only the first act_dim outputs of the head computed (the log-std half is
never used at inference) and action = tanh(head).

Every float operation below is a single float32 op in the order the C code
does them, and the firmware compiles q8.cpp with -ffp-contract=off, so this
mirror, the host build and the ESP32 agree to the last bit up to the final
tanhf (libm vs numpy, ~1 ulp). That is why swish_fast is a polynomial here and
not expf: libm's expf is not reproducible across platforms, a polynomial is.
"""
import numpy as np

F = np.float32
QMAX_W = 127
QMAX_X = 32767
LOG2E = F(1.44269504088896341)
# 2^f = e^(f ln2) for |f| <= 0.5: Taylor to degree 6, truncation error
# (0.347)^7 / 7! ~ 1.2e-7 relative -- at float32 resolution.
EXP2_C = [F(1.0), F(0.693147180559945), F(0.240226506959101),
          F(0.0555041086648216), F(0.00961812910762848),
          F(0.00133335581464284), F(0.000154035303933816)]
EXP_CLAMP = F(80.0)


def _round_half_away(v):
    """(v >= 0) ? (int)(v + 0.5f) : -(int)(0.5f - v), in float32."""
    v = np.asarray(v, dtype=F)
    pos = np.trunc((v + F(0.5)).astype(F))
    neg = -np.trunc((F(0.5) - v).astype(F))
    return np.where(v >= F(0), pos, neg).astype(np.int64)


def pad4(n):
    return (n + 3) & ~3


def quantize_dense(w, b, out_keep=None):
    """flax kernel (in, out) -> int8 (out_keep, pad4(in)) output-major rows,
    float32 per-row scale, float32 bias. out_keep < out drops the trailing
    outputs (the head's log-std half)."""
    w = np.asarray(w, dtype=F)
    n_in, n_out = w.shape
    k = n_out if out_keep is None else out_keep
    wt = np.ascontiguousarray(w[:, :k].T)                  # (k, in)
    maxabs = np.max(np.abs(wt), axis=1).astype(F)
    scale = np.where(maxabs > F(0), (maxabs / F(QMAX_W)).astype(F),
                     F(1.0)).astype(F)
    q = _round_half_away((wt / scale[:, None]).astype(F))
    q = np.clip(q, -QMAX_W, QMAX_W).astype(np.int8)
    qp = np.zeros((k, pad4(n_in)), np.int8)
    qp[:, :n_in] = q
    return qp, scale, np.asarray(b, dtype=F)[:k].copy()


def quantize_acts(x):
    """float32 vector -> (scale, int16 values)."""
    x = np.asarray(x, dtype=F)
    m = F(np.max(np.abs(x))) if x.size else F(0)
    if not (m > F(0)) or not (m <= F(3.0e38)):        # all zero, or inf
        return F(1.0), np.zeros(x.shape, np.int64)
    inv = F(F(QMAX_X) / m)
    s = F(m / F(QMAX_X))
    q = _round_half_away((x * inv).astype(F))
    return s, np.clip(q, -QMAX_X, QMAX_X)


def exp_fast(z):
    """e^z for z in float32, z clamped to +-80 (the C swishFast's exp)."""
    z = np.clip(np.asarray(z, dtype=F), -EXP_CLAMP, EXP_CLAMP).astype(F)
    t = (z * LOG2E).astype(F)
    n = _round_half_away(t)
    f = (t - n.astype(F)).astype(F)
    p = EXP2_C[6]
    for c in EXP2_C[5::-1]:
        p = (p * f).astype(F)
        p = (p + c).astype(F)
    return (p * np.ldexp(F(1.0), n.astype(np.int32)).astype(F)).astype(F)


def swish_fast(y):
    y = np.asarray(y, dtype=F)
    e = exp_fast(-y)
    d = (F(1.0) + e).astype(F)
    return (y / d).astype(F)


def quantize_net(layers, act_dim):
    """[(w, b)] float layers -> [(wq, scale, b, in)], head cut to act_dim."""
    out = []
    for i, (w, b) in enumerate(layers):
        keep = act_dim if i + 1 == len(layers) else None
        wq, s, bb = quantize_dense(w, b, keep)
        out.append((wq, s, bb, w.shape[0]))
    return out


def forward_q8(mean, std, qlayers, obs, act_dim):
    """The firmware's quantized forward pass (policy::forwardNetQ8)."""
    x = ((np.asarray(obs, dtype=F) - mean) / std).astype(F)
    for i, (wq, s, b, n_in) in enumerate(qlayers):
        sx, xq = quantize_acts(x[:n_in])
        acc = wq[:, :n_in].astype(np.int64) @ xq.astype(np.int64)
        assert np.all(np.abs(acc) < 2 ** 31)
        c = (sx * s).astype(F)
        y = (acc.astype(F) * c).astype(F)
        y = (y + b).astype(F)
        x = swish_fast(y) if i + 1 < len(qlayers) else y
    return np.tanh(x[:act_dim]).astype(F)


# ---------------------------------------------------------------------
#  Offline quantization with error feedback (what the generator ships)
# ---------------------------------------------------------------------
# Round-to-nearest int8 (quantize_dense; the firmware's run-time
# policy::quantizeDense) costs the (256,256) walk14 student ~0.007 rms /
# 0.06 max in action on its own rollouts. GPTQ (Frantar et al. 2022) rounds
# each kernel's inputs in turn and folds the rounding error into the inputs
# not yet rounded, weighted by the layer's input covariance on real
# observations -- the same int8 format at run time, about half the error.
# Layers are done in order, each calibrated on the activations the already
# quantized layers before it produce.
#
# Calibration data: CPU-plant rollouts of the float net itself, fixed seeds
# and commands, so a regeneration is reproducible on one machine.
CAL_SEEDS = tuple(range(1000, 1012))
CAL_CMDS = ((0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            (0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            (0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0),
            (0.2, 0.0, -0.5, 1.0, 0.0, 0.0, 0.0))
CAL_STEPS = 400


def float_forward(mean, std, layers, obs, act_dim):
    """The float32 reference (gen_policy_weights.forward), batched."""
    x = ((np.asarray(obs, dtype=F) - mean) / std).astype(F)
    for i, (w, b) in enumerate(layers):
        x = (x @ w + b).astype(F)
        if i + 1 < len(layers):
            x = (x / (F(1.0) + np.exp(-x, dtype=F))).astype(F)
    return np.tanh(x[..., :act_dim]).astype(F)


def calibration_obs(root, run, mean, std, layers, act_dim,
                    seeds=CAL_SEEDS, cmds=CAL_CMDS, steps=CAL_STEPS):
    """Raw observations from closed-loop CPU-plant rollouts of the float
    net (sim/mjx/eval_precision.make_env, the referee's env)."""
    import json
    import os
    import sys
    for d in (os.path.join(root, "sim"), os.path.join(root, "sim", "mjx")):
        if d not in sys.path:
            sys.path.insert(0, d)
    import eval_precision as E
    cfg = json.load(open(os.path.join(root, "sim", "runs", run,
                                      "config.json")))
    env = E.make_env(cfg, 12.0, True, os.path.join(root, "sim",
                                                   cfg["xml_path"]))
    obs = []
    for k, seed in enumerate(seeds):
        o, _ = env.reset(seed=seed)
        env.set_command(*cmds[k % len(cmds)])
        for _ in range(steps):
            obs.append(np.asarray(o, dtype=F))
            a = float_forward(mean, std, layers, o, act_dim)
            o, _, term, _, _ = env.step(a)
            if term:
                break
    return np.array(obs, dtype=F)


def _gptq_kernel(w, x, damp=0.01):
    """flax kernel (in, out), calibration inputs x (n, in) -> int8
    (out, in), per-output scale (max-abs / 127, as quantize_dense)."""
    W0 = np.asarray(w, dtype=np.float64).T                  # (out, in)
    H = x.astype(np.float64).T @ x.astype(np.float64)
    # Inputs the calibration never moved (the frozen channels: normalised to
    # exactly 0 in every rollout) go FIRST: rounded from their own values,
    # never handed anybody's error. Error feedback into them would cost
    # nothing in distribution and everything if one ever moved.
    dead = np.diag(H) <= 0.0
    order = np.concatenate([np.flatnonzero(dead), np.flatnonzero(~dead)])
    W = W0[:, order].copy()
    H = H[np.ix_(order, order)]
    H += np.eye(H.shape[0]) * (damp * np.mean(np.diag(H)) + 1e-12)
    scale = np.max(np.abs(W0), axis=1).astype(F)
    scale = np.where(scale > F(0), (scale / F(QMAX_W)).astype(F),
                     F(1.0)).astype(F)
    s64 = scale.astype(np.float64)
    U = np.linalg.cholesky(np.linalg.inv(H)).T                # upper
    qo = np.zeros(W.shape, np.int64)
    for i in range(W.shape[1]):
        qi = np.clip(np.round(W[:, i] / s64), -QMAX_W, QMAX_W)
        qo[:, i] = qi
        err = (W[:, i] - qi * s64) / U[i, i]
        W[:, i + 1:] -= np.outer(err, U[i, i + 1:])
    q = np.zeros(W.shape, np.int64)
    q[:, order] = qo
    return q.astype(np.int8), scale


def gptq_quantize_net(mean, std, layers, act_dim, cal_obs):
    """[(w, b)] -> [(wq (k, pad4(in)) int8, scale, b, in)] like quantize_net,
    with GPTQ rounding calibrated on cal_obs (raw observations)."""
    x = ((np.asarray(cal_obs, dtype=F) - mean) / std).astype(F)
    out = []
    for i, (w, b) in enumerate(layers):
        last = i + 1 == len(layers)
        k = act_dim if last else w.shape[1]
        q, scale = _gptq_kernel(np.asarray(w, dtype=F)[:, :k], x)
        n_in = w.shape[0]
        qp = np.zeros((k, pad4(n_in)), np.int8)
        qp[:, :n_in] = q
        bb = np.asarray(b, dtype=F)[:k].copy()
        out.append((qp, scale, bb, n_in))
        deq = (q.astype(F) * scale[:, None]).T.astype(F)        # (in, k)
        x = (x @ deq + bb).astype(F)
        if not last:
            x = (x / (F(1.0) + np.exp(-x, dtype=F))).astype(F)
    return out


# ---------------------------------------------------------------------
#  Reading an int8 weights.h back (the SIL profile test's reference)
# ---------------------------------------------------------------------
def _c_array(txt, name, dtype):
    import re
    m = re.search(r"\b%s\[\d+\]\s*=\s*\{(.*?)\};" % re.escape(name), txt, re.S)
    if not m:
        raise ValueError("no array %s in the header" % name)
    body = m.group(1).replace("f", "") if dtype == F else m.group(1)
    return np.array([float(v) for v in body.replace("\n", " ").split(",")
                     if v.strip()], dtype=np.float64).astype(dtype)


def read_q8_header(path):
    """An int8 policy/weights.h -> (mean, std, qlayers, act_dim): exactly the
    compiled-in net, for forward_q8."""
    import re
    txt = open(path).read()
    if "#define POLICY_WEIGHTS_Q8" not in txt:
        raise ValueError("%s is not an int8 weights.h" % path)
    act_dim = int(re.search(r"kActDim = (\d+);", txt).group(1))
    mean = _c_array(txt, "kNormMean", F)
    std = _c_array(txt, "kNormStd", F)
    table = re.search(r"kLayersQ8\[kNumLayers\] = \{(.*?)\};", txt, re.S).group(1)
    qlayers = []
    for i, row in enumerate(re.findall(r"\{(\d+), (\d+), (\d+), kQW\d+", table)):
        n_in, in_pad, out = (int(v) for v in row)
        w = _c_array(txt, "kQW%d" % i, np.int8)[:out * in_pad]
        qlayers.append((w.reshape(out, in_pad), _c_array(txt, "kQS%d" % i, F),
                        _c_array(txt, "kQB%d" % i, F), n_in))
    return mean, std, qlayers, act_dim
