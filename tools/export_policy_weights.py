#!/usr/bin/env python3
"""Export a trained brax PPO policy to the SIL/firmware weight format.

    JAX_PLATFORMS=cpu .venv/bin/python tools/export_policy_weights.py \
        --run loco_v6creep

Writes (defaults, all under sim/sil/):

  weights/<run>.silw        little-endian f32 blob, self-describing header
  weights/<run>.silw.json   sidecar: layer_sizes, activation, dims, normalizer
  golden/policy_vectors.json  16 normalized-obs -> action pairs (brax, det.)
  golden/obs_vectors.json     8 walker_env states -> _obs() frame + Inputs

WHY BOTH A HEADER AND A SIDECAR (docs/sil-harness.md ambiguity, resolved):
the design says "little-endian f32 blob + JSON sidecar", but `sil_init()`
takes exactly ONE path, and a -fno-exceptions firmware host build has no JSON
parser.  So the .silw is SELF-DESCRIBING: a 64-byte header (magic, dims,
layer sizes, activation name) followed by the f32 payload.  The sidecar is
written too, byte-for-byte agreeing, and records `blob.header_bytes` so a
reader that only wants floats can seek past it.  `--raw` emits the headerless
variant if the C side really wants a bare float blob.

Semantics reproduced by the export (this is the whole point -- the bench found
swish-vs-tanh and the missing normalizer the hard way):

    x      = (obs - mean) / std                 running_statistics.normalize
    h      = swish(h @ W_i + b_i)               linen.swish for every hidden
    logits = h @ W_out + b_out                  linear, 2 * act_dim wide
    action = tanh(logits[:act_dim])             NormalTanhDistribution.mode()

Kernels are stored in flax's (in, out) row-major order, verbatim -- no reader
ever has to transpose.
"""
import argparse
import hashlib
import json
import os
import pickle
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))

RUNS = os.path.join(ROOT, "sim", "runs")
SIL = os.path.join(ROOT, "sim", "sil")
SPEC_H = os.path.join(ROOT, "firmware", "components", "obs", "include", "obs",
                      "obs_spec.h")

MAGIC = b"SILW"
VERSION = 1
ACT_NAME = b"swish"
ACT_FIELD = 16          # bytes reserved for the activation name


# ---------------------------------------------------------------------
#  obs_spec.h drift check (read-only; the header is generated elsewhere)
# ---------------------------------------------------------------------
def read_obs_spec():
    """Pull the int constants out of the generated firmware obs spec."""
    out = {}
    try:
        txt = open(SPEC_H).read()
    except OSError:
        return out
    for line in txt.splitlines():
        line = line.strip()
        if line.startswith("inline constexpr int k"):
            name = line.split()[3]
            try:
                out[name] = int(line.split("=")[1].strip().rstrip(";"))
            except ValueError:
                pass
    return out


# ---------------------------------------------------------------------
#  params.pkl -> layer list
# ---------------------------------------------------------------------
def policy_layers(policy_params):
    """[(W, b), ...] in application order, from a brax MLP param dict."""
    layers = policy_params["params"]
    keys = sorted((k for k in layers if k.startswith("hidden_")),
                  key=lambda k: int(k.split("_")[1]))
    out = []
    for k in keys:
        w = np.asarray(layers[k]["kernel"], dtype=np.float32)
        b = np.asarray(layers[k]["bias"], dtype=np.float32)
        if w.ndim != 2 or b.ndim != 1 or w.shape[1] != b.shape[0]:
            raise SystemExit(f"layer {k}: bad shapes {w.shape} {b.shape}")
        out.append((w, b))
    for i in range(len(out) - 1):
        if out[i][0].shape[1] != out[i + 1][0].shape[0]:
            raise SystemExit("layer shapes do not chain")
    return out


def normalizer(norm_params):
    mean = np.asarray(norm_params.mean, dtype=np.float32)
    std = np.asarray(norm_params.std, dtype=np.float32)
    if mean.shape != std.shape or mean.ndim != 1:
        raise SystemExit("normalizer mean/std shape mismatch")
    if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
        raise SystemExit("normalizer has non-finite entries")
    if np.any(std <= 0.0):
        raise SystemExit("normalizer std has non-positive entries")
    return mean, std


def swish(x):
    x = np.asarray(x, dtype=np.float32)
    return (x / (1.0 + np.exp(-x, dtype=np.float32))).astype(np.float32)


def numpy_forward(mean, std, layers, obs, act_dim):
    """The reference float32 forward pass; the firmware must match this."""
    x = ((np.asarray(obs, dtype=np.float32) - mean) / std).astype(np.float32)
    for i, (w, b) in enumerate(layers):
        x = (x @ w + b).astype(np.float32)
        if i + 1 < len(layers):
            x = swish(x)
    return np.tanh(x[:act_dim]).astype(np.float32)


# ---------------------------------------------------------------------
#  blob writer
# ---------------------------------------------------------------------
def build_blob(mean, std, layers, act_dim, raw=False):
    sizes = [layers[0][0].shape[0]] + [w.shape[1] for w, _ in layers]
    payload = [mean.astype("<f4"), std.astype("<f4")]
    order = ["norm_mean", "norm_std"]
    for i, (w, b) in enumerate(layers):
        payload.append(np.ascontiguousarray(w, dtype="<f4").ravel())
        payload.append(np.asarray(b, dtype="<f4"))
        order += [f"W{i}", f"b{i}"]
    floats = np.concatenate([p.ravel() for p in payload]).astype("<f4")

    if raw:
        return floats.tobytes(), 0, sizes, order, floats.size

    header_bytes = 28 + ACT_FIELD + 4 * len(sizes)
    hdr = b"".join([
        MAGIC,
        struct.pack("<I", VERSION),
        struct.pack("<I", header_bytes),
        struct.pack("<I", sizes[0]),
        struct.pack("<I", act_dim),
        struct.pack("<I", len(layers)),
        struct.pack("<I", len(sizes)),
        ACT_NAME.ljust(ACT_FIELD, b"\0"),
        b"".join(struct.pack("<I", s) for s in sizes),
    ])
    assert len(hdr) == header_bytes, (len(hdr), header_bytes)
    return hdr + floats.tobytes(), header_bytes, sizes, order, floats.size


# ---------------------------------------------------------------------
#  golden vectors
# ---------------------------------------------------------------------
def golden_policy_vectors(brax_act, mean, std, layers, act_dim, n=16,
                          seed=20260730):
    """n normalized-obs -> action pairs, computed by BRAX in deterministic
    mode.  The normalized vector z is sampled first and the raw obs is
    reconstructed as mean + std * z, so (z, action) is an exact pair with no
    round-trip error in the normalizer."""
    rng = np.random.default_rng(seed)
    obs_dim = mean.shape[0]
    cases = []
    for k in range(n):
        if k == 0:
            z = np.zeros(obs_dim, np.float32)
        elif k == 1:
            z = np.ones(obs_dim, np.float32)
        else:
            scale = 0.25 + 0.5 * (k % 6)
            z = (rng.normal(0.0, scale, obs_dim)).astype(np.float32)
        raw = (mean + std * z).astype(np.float32)
        a_brax = np.asarray(brax_act(raw), dtype=np.float32)
        a_np = numpy_forward(mean, std, layers, raw, act_dim)
        cases.append(dict(obs_norm=[float(v) for v in z],
                          obs=[float(v) for v in raw],
                          action=[float(v) for v in a_brax],
                          max_abs_err_vs_numpy=float(np.max(np.abs(a_brax
                                                                   - a_np)))))
    return cases


def golden_obs_vectors(env, spec, n=8, seed=7):
    """n walker_env states -> the python _obs() frame, with the firmware-side
    obs::Inputs fields recorded alongside.

    The Inputs are read back OUT of the produced frame (not out of the env)
    for up/gyro: walker_env._obs() pushes the IMU through a per-episode
    mounting rotation and additive noise, so the only self-consistent source
    for "what the firmware would have been handed" is the frame itself."""
    import mujoco

    off = spec["off"]
    nj, ncmd, fd = spec["nj"], spec["ncmd"], spec["frame"]
    rng = np.random.default_rng(seed)
    cases = []
    commands = [
        (0, 0, 0, 1, 0, 0, 0),
        (0.35, 0, 0, 1, 0, 0, 0),
        (0, 0, 0.6, 1, 0, 0, 0),
        (0.2, 0.15, -0.3, 1, 0, 0, 0),
        (0, 0, 0, 0.7, 0, 0, 0),
        (0.5, 0, 0, 1, 1, 0.04, 0.05),
        (-0.3, 0, 0, 1, 0, 0, 0),
        (0.1, -0.2, 0.2, 0.85, 0, 0, 0),
    ]
    for k in range(n):
        env.reset(seed=1000 + k)
        env.set_command(*commands[k % len(commands)])
        # vary the pose: a random draw inside the joint limits, plus a
        # non-trivial joint velocity and a non-zero previous action
        lo, hi = env._lo, env._hi
        q = lo + (hi - lo) * rng.uniform(0.15, 0.85, size=nj)
        if k == 0:
            q = env._default.copy()          # exact standing pose
        env.data.qpos[env._jqpos] = q
        env.data.qvel[env._jqvel] = rng.uniform(-2.0, 2.0, size=nj)
        # torso attitude: tilt so the up-vector is not trivially (0,0,1)
        if k > 1:
            ang = rng.uniform(-0.25, 0.25, 3)
            env.data.qpos[3:7] = np.array([
                np.cos(0.5 * np.linalg.norm(ang)),
                *(np.sin(0.5 * np.linalg.norm(ang))
                  * ang / max(np.linalg.norm(ang), 1e-9))])
            env.data.qvel[3:6] = rng.uniform(-1.5, 1.5, 3)
        mujoco.mj_forward(env.model, env.data)
        env._prev_action = (rng.uniform(-1, 1, nj).astype(np.float32)
                            if k else np.zeros(nj, np.float32))
        env._gait_phase = float(rng.uniform(-np.pi, np.pi)) if k else 0.0

        obs = np.asarray(env._obs(), dtype=np.float32)
        frame = obs[:fd]
        hist = [obs[(i + 1) * fd:(i + 2) * fd] for i in range(spec["hist"] - 1)]
        inputs = dict(
            q=[float(v) for v in frame[off["q"]:off["q"] + nj]],
            dq=[float(v) for v in frame[off["dq"]:off["dq"] + nj]],
            up=[float(v) for v in frame[off["up"]:off["up"] + 3]],
            gyro=[float(v) for v in frame[off["gyro"]:off["gyro"] + 3]],
            prev_action=[float(v) for v in
                         frame[off["prev"]:off["prev"] + nj]],
            phase=float(env._gait_phase),
            cmd=[float(v) for v in frame[off["cmd"]:off["cmd"] + ncmd]],
        )
        cases.append(dict(
            name=f"state_{k}",
            cmd_label=list(commands[k % len(commands)]),
            inputs=inputs,
            frame=[float(v) for v in frame],
            hist=[[float(v) for v in h] for h in hist],
            obs=[float(v) for v in obs],
        ))
    return cases


# ---------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", default="loco_v6creep")
    p.add_argument("--out", default=None,
                   help="output .silw path (default sim/sil/weights/<run>.silw)")
    p.add_argument("--raw", action="store_true",
                   help="write a headerless bare-f32 blob (escape hatch)")
    p.add_argument("--no-golden", action="store_true")
    p.add_argument("--golden-dir", default=os.path.join(SIL, "golden"))
    p.add_argument("--n-policy", type=int, default=16)
    p.add_argument("--n-obs", type=int, default=8)
    args = p.parse_args()

    run_dir = args.run if os.path.isdir(args.run) else os.path.join(RUNS,
                                                                    args.run)
    run_name = os.path.basename(os.path.normpath(run_dir))
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    with open(os.path.join(run_dir, "params.pkl"), "rb") as f:
        params = pickle.load(f)
    if len(params) < 2:
        raise SystemExit("params.pkl: expected (normalizer, policy, [value])")

    mean, std = normalizer(params[0])
    layers = policy_layers(params[1])
    obs_dim = int(mean.shape[0])
    out_dim = int(layers[-1][0].shape[1])
    if out_dim % 2 != 0:
        raise SystemExit(f"output width {out_dim} is not 2 * act_dim")
    act_dim = out_dim // 2
    if layers[0][0].shape[0] != obs_dim:
        raise SystemExit(f"first kernel takes {layers[0][0].shape[0]} inputs "
                         f"but the normalizer is {obs_dim}-wide")

    # -- assert dims against the run config + the plant ----------------
    from eval_precision import make_env, load_policy    # noqa: E402

    xml = cfg.get("xml_path") or "bimo_biped_v3yaw.xml"
    if not os.path.isabs(xml) and not os.path.exists(xml):
        xml = os.path.join(ROOT, "sim", os.path.basename(xml))
    env = make_env(cfg, 12.0, True, xml)
    env_obs = int(env.observation_space.shape[0])
    env_act = int(env.action_space.shape[0])
    if env_obs != obs_dim:
        raise SystemExit(f"config plant obs {env_obs} != checkpoint {obs_dim}")
    if env_act != act_dim:
        raise SystemExit(f"config plant act {env_act} != checkpoint {act_dim}")
    hist = int(cfg.get("obs_hist_len", 1))
    if env.obs_frame * hist != obs_dim:
        raise SystemExit(f"frame {env.obs_frame} * hist {hist} != {obs_dim}")
    if cfg.get("action_map") != "full":
        raise SystemExit("this exporter targets action_map='full' only "
                         f"(config says {cfg.get('action_map')!r})")
    if not cfg.get("imu_obs"):
        raise SystemExit("config imu_obs is false; the firmware zeroes "
                         "linvel+height and would diverge")
    spec_h = read_obs_spec()
    for key, val in (("kObsDim", obs_dim), ("kActDim", act_dim),
                     ("kFrameDim", env.obs_frame), ("kHistLen", hist),
                     ("kNumJoints", env_act)):
        if key in spec_h and spec_h[key] != val:
            raise SystemExit(f"obs_spec.h {key}={spec_h[key]} but this run "
                             f"needs {val} -- regenerate tools/gen_obs_spec.py")

    # -- blob + sidecar -------------------------------------------------
    out_path = args.out or os.path.join(SIL, "weights", f"{run_name}.silw")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    blob, header_bytes, sizes, order, nfloats = build_blob(
        mean, std, layers, act_dim, raw=args.raw)
    with open(out_path, "wb") as f:
        f.write(blob)
    sha = hashlib.sha256(blob).hexdigest()

    sidecar = dict(
        run=run_name,
        format="silw",
        version=VERSION,
        activation="swish",
        output_activation="tanh(logits[:act_dim])",
        obs_dim=obs_dim,
        act_dim=act_dim,
        layer_sizes=[int(s) for s in sizes],
        normalizer=dict(mean=[float(v) for v in mean],
                        std=[float(v) for v in std]),
        blob=dict(file=os.path.basename(out_path),
                  header_bytes=header_bytes,
                  magic="SILW" if not args.raw else None,
                  dtype="float32", byte_order="little",
                  kernel_layout="(in, out) row-major, flax order",
                  order=order, n_floats=int(nfloats),
                  bytes=len(blob), sha256=sha),
        source=dict(params="params.pkl", config="config.json",
                    xml=os.path.basename(xml),
                    obs_frame=int(env.obs_frame), hist_len=hist,
                    num_cmd=int(env._ncmd)),
    )
    with open(out_path + ".json", "w") as f:
        json.dump(sidecar, f, indent=1)

    # Channels the run never varied: running_statistics floors their std at
    # std_eps (1e-6), so ANY deviation from the recorded mean is amplified a
    # millionfold on the way into the net.  The firmware must hold these
    # constant or the policy sees a 1e5-magnitude input.
    frozen = np.where(std <= 1e-5)[0]
    if frozen.size:
        slots = sorted(set(int(i) % int(env.obs_frame) for i in frozen))
        print(f"  WARNING: {frozen.size} frozen obs channels (std == std_eps) "
              f"-> frame slots {slots}")
        print("           the firmware MUST feed these their training value "
              "exactly; a deviation of d becomes d/1e-6 after normalisation")

    n_params = sum(w.size + b.size for w, b in layers)
    print(f"{run_name}: {' -> '.join(str(s) for s in sizes)}  swish")
    print(f"  {n_params} weights + {2 * obs_dim} normalizer floats "
          f"= {nfloats} f32 ({len(blob) / 1024.0:.1f} KB)")
    print(f"  wrote {out_path}  (header {header_bytes} B, sha256 {sha[:16]})")
    print(f"  wrote {out_path}.json")

    if args.no_golden:
        return

    # -- golden vectors -------------------------------------------------
    os.makedirs(args.golden_dir, exist_ok=True)
    brax_act = load_policy(run_dir, obs_dim, act_dim)
    cases = golden_policy_vectors(brax_act, mean, std, layers, act_dim,
                                  n=args.n_policy)
    worst = max(c["max_abs_err_vs_numpy"] for c in cases)
    pv = os.path.join(args.golden_dir, "policy_vectors.json")
    with open(pv, "w") as f:
        json.dump(dict(run=run_name, obs_dim=obs_dim, act_dim=act_dim,
                       activation="swish", atol=1e-5,
                       note="obs_norm = (obs - mean) / std; action is brax "
                            "deterministic mode = tanh(logits[:act_dim])",
                       max_abs_err_vs_numpy=worst, cases=cases), f)
    print(f"  wrote {pv} ({len(cases)} cases, "
          f"brax-vs-numpy worst {worst:.2e})")

    spec = dict(nj=env_act, ncmd=int(env._ncmd), frame=int(env.obs_frame),
                hist=hist,
                off=dict(q=0, dq=env_act, up=2 * env_act,
                         linvel=2 * env_act + 3, gyro=2 * env_act + 6,
                         prev=2 * env_act + 9, height=3 * env_act + 9,
                         phase=3 * env_act + 10, cmd=3 * env_act + 12))
    ocases = golden_obs_vectors(env, spec, n=args.n_obs)
    ov = os.path.join(args.golden_dir, "obs_vectors.json")
    with open(ov, "w") as f:
        json.dump(dict(run=run_name, frame_dim=spec["frame"], obs_dim=obs_dim,
                       hist_len=hist, num_joints=spec["nj"],
                       num_cmd=spec["ncmd"], offsets=spec["off"],
                       note="inputs are the obs::Inputs fields the firmware "
                            "must assemble into `frame`; linvel and height "
                            "are zero (imu_obs=1)",
                       cases=ocases), f)
    print(f"  wrote {ov} ({len(ocases)} cases)")


if __name__ == "__main__":
    main()
