#!/usr/bin/env python3
"""Convert a LEGACY-action-map policy checkpoint to the FULL action map.

    CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu .venv/bin/python \
        tools/convert_action_map.py --src robot_legs_walk9 \
        --dst robot_legs_walk9_fullmap

Why: the exporter (tools/export_policy_weights.py) and the firmware's
actionToAngles only implement action_map="full", and every v6 walk so far
trained with "legacy". The two maps differ only where a joint's home pose is
off the middle of its policy range:

    legacy  target = clip(home + 0.5 * (hi - lo) * a, lo, hi)
    full    target = home + (hi - home) * a   (a >= 0)
                     home + (home - lo) * a   (a <  0)

so the full-map action that serves the same target is, per joint,

    a_full = g(a_legacy) = clip(r+ * a, -1, 1) (a >= 0),  clip(r- * a, -1, 1) (a < 0)
    r+ = 0.5 (hi - lo) / (hi - home),  r- = 0.5 (hi - lo) / (home - lo)

(the v6 knee: r- 0.52, r+ 13.5; hip roll 0.83 / 1.25; hip pitch 1.17 / 0.88;
yaw, ankle and ankle roll 1 / 1). The policy's action is tanh(loc), and g is
piecewise, so no rescaling of the last layer is exact. Instead:

 1. roll the legacy policy out (deterministic) in its own training env
    (MJX, CPU), recording the observations it sees;
 2. map every prev_action block of the observation through g (the full-map
    policy observes its own full-map action) and refit the normalizer's
    mean/std on those dims so the first layer sees what it saw before
    (least squares x ~ alpha * g(x) + beta per dim);
 3. refit the LAST layer's loc rows (ridge least squares on the last hidden
    activations, then Gauss-Newton on the served target in degrees), then a
    short supervised polish of the whole MLP (--bc-steps) on the same
    states, loss = served-target error. The value network is kept;
 4. verify closed loop: the converted policy in the full-map env against the
    legacy one in its own env, same seeds.

robot_legs_walk9 -> robot_legs_walk9_fullmap (2026-10-30, 128 envs x 300
ticks, 35.9k states): served-target error 0.4-0.9 deg rms (knee 1.6, p99
4.6); closed loop survive 0.820 vs legacy 0.836, reward/step 6.293 vs 6.298
(the legacy weights served through the full map unchanged: 0.789, 5.285).

The output run directory holds params.pkl and a config.json identical to the
source's except action_map="full" (and train.action_map), plus a
"converted_from" record. It is a warm-start source (--init-from), not a
result to deploy as is.
"""
import argparse
import inspect
import json
import os
import pickle
import sys
import time

os.environ.setdefault("JAX_PLATFORMS", "cpu")

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))

import jax  # noqa: E402
import jax.numpy as jp  # noqa: E402

from env_mjx import BimoMJXEnv  # noqa: E402

RUNS = os.path.join(ROOT, "sim", "runs")


def env_from_config(cfg, action_map, overrides=None):
    params = set(inspect.signature(BimoMJXEnv.__init__).parameters)
    kw = {k: (tuple(v) if isinstance(v, list) else v)
          for k, v in cfg.items() if k in params}
    kw["xml_path"] = os.path.join(ROOT, "sim", os.path.basename(cfg["xml_path"]))
    kw["mjx_impl"] = "jax"            # CPU: the Warp backend is CUDA
    kw.pop("warp_envs", None)
    kw["action_map"] = action_map
    for k in ("held_joints", "servo_kp_scale", "servo_types"):
        if isinstance(kw.get(k), dict):
            kw[k] = dict(kw[k])
    if overrides:
        kw.update(overrides)
    return BimoMJXEnv(**kw)


def make_forward(mean, std, layers, act_dim):
    def hidden(obs):
        x = (obs - mean) / std
        for w, b in layers[:-1]:
            x = jax.nn.swish(x @ w + b)
        return x

    def act(obs, w_out=None, b_out=None):
        h = hidden(obs)
        w, b = layers[-1] if w_out is None else (w_out, b_out)
        return jp.tanh((h @ w + b)[..., :act_dim])
    return hidden, act


def rollout(env, act_fn, n_envs, n_steps, seed):
    """Deterministic rollout of n_envs for n_steps. Returns obs (T, N, D),
    action (T, N, A), alive mask (T, N) (False from the first done on),
    and per-step forward velocity / commanded speed for a closed-loop check."""
    keys = jax.random.split(jax.random.PRNGKey(seed), n_envs)
    st = jax.jit(jax.vmap(env.reset))(keys)

    def one(carry, _):
        s, alive = carry
        a = act_fn(s.obs)
        s2 = jax.vmap(env.step)(s, a)
        rec = (s.obs, a, alive, s2.reward)
        alive = alive & (s2.done < 0.5)
        return (s2, alive), rec

    scan = jax.jit(lambda s: jax.lax.scan(
        one, (s, jp.ones((n_envs,), bool)), None, length=n_steps))
    (_, alive_end), (obs, act, alive, rew) = scan(st)
    return (np.asarray(obs), np.asarray(act), np.asarray(alive),
            np.asarray(rew), np.asarray(alive_end))


def bc_finetune(layers, mean, std, X, tgt, home, lo, hi, nj, steps, seed, t0,
                batch=2048, lr=3e-5):
    """Supervised polish of the whole policy MLP (loc rows only in the loss):
    minimise the served-target error (rad^2) of the full-map policy against
    the legacy targets on the recorded states. Starts from the last-layer
    refit, so it only has to remove what one linear layer cannot."""
    import optax
    params = [(jp.asarray(w, jp.float32), jp.asarray(b, jp.float32)) for w, b in layers]
    Xn = jp.asarray((X - mean) / std, jp.float32)
    T = jp.asarray(tgt, jp.float32)
    h0, l0, h1 = (jp.asarray(v, jp.float32) for v in (home, lo, hi))

    def served(p, xn):
        x = xn
        for w, b in p[:-1]:
            x = jax.nn.swish(x @ w + b)
        t = jp.tanh((x @ p[-1][0] + p[-1][1])[..., :nj])
        return h0 + jp.where(t >= 0, (h1 - h0) * t, (h0 - l0) * t)

    def loss(p, xn, tg):
        return jp.mean((served(p, xn) - tg) ** 2)

    opt = optax.adam(lr)
    st = opt.init(params)

    @jax.jit
    def step(p, s, k):
        idx = jax.random.randint(k, (batch,), 0, Xn.shape[0])
        lv, g = jax.value_and_grad(loss)(p, Xn[idx], T[idx])
        up, s = opt.update(g, s, p)
        return optax.apply_updates(p, up), s, lv

    key = jax.random.PRNGKey(seed + 1)
    full0 = float(loss(params, Xn, T))
    for i in range(steps):
        key, k = jax.random.split(key)
        params, st, lv = step(params, st, k)
    full1 = float(loss(params, Xn, T))
    print(f"[{time.time()-t0:5.0f}s] BC polish {steps} steps: served-target rms "
          f"{np.degrees(np.sqrt(full0)):.2f} -> {np.degrees(np.sqrt(full1)):.2f} deg (all joints)")
    return [(np.asarray(w, np.float64), np.asarray(b, np.float64)) for w, b in params]


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--src", required=True, help="legacy run under sim/runs")
    p.add_argument("--dst", required=True, help="new run name for the full-map checkpoint")
    p.add_argument("--runs", default=RUNS)
    p.add_argument("--also-runs", default=None,
                   help="a second runs dir to copy the result into (the night clone's)")
    p.add_argument("--envs", type=int, default=128)
    p.add_argument("--steps", type=int, default=300, help="control ticks per rollout (20 ms)")
    p.add_argument("--ridge", type=float, default=1e-3)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--within-stops", action="store_true",
                   help="the full-map env clips its policy range to the mechanical stops "
                        "(train_mjx --ctrl-within-stops); the converted policy serves the "
                        "legacy targets inside that range")
    p.add_argument("--bc-steps", type=int, default=3000,
                   help="supervised polish of the whole MLP after the last-layer refit (0 = off)")
    a = p.parse_args(argv)

    src = os.path.join(a.runs, a.src)
    cfg = json.load(open(os.path.join(src, "config.json")))
    if cfg.get("action_map", "legacy") != "legacy":
        raise SystemExit(f"{a.src} is already action_map={cfg.get('action_map')!r}")
    params = pickle.load(open(os.path.join(src, "params.pkl"), "rb"))
    norm, pol = params[0], params[1]
    mean = np.asarray(norm.mean, np.float64)
    std = np.asarray(norm.std, np.float64)
    keys = sorted(pol["params"], key=lambda k: int(k.split("_")[1]))
    layers = [(np.asarray(pol["params"][k]["kernel"], np.float64),
               np.asarray(pol["params"][k]["bias"], np.float64)) for k in keys]

    t0 = time.time()
    env_l = env_from_config(cfg, "legacy")
    over_f = {"ctrl_within_stops": True} if a.within_stops else None
    env_f = env_from_config(cfg, "full", over_f)
    nj = env_l.action_size
    lo_l, hi_l, home = (np.asarray(x, np.float64) for x in (env_l._lo, env_l._hi, env_l._default))
    lo, hi = (np.asarray(x, np.float64) for x in (env_f._lo, env_f._hi))   # the full map's range
    half = 0.5 * (hi_l - lo_l)
    rp, rm = half / (hi - home), half / (home - lo)

    def legacy_target(x):
        """the target a legacy action serves, clipped into the full map's range"""
        return np.clip(np.clip(home + half * x, lo_l, hi_l), lo, hi)

    def to_full(x):
        """legacy action -> the full-map action serving the same target"""
        t = legacy_target(x) - home
        return np.where(t >= 0, t / (hi - home), t / (home - lo))
    frame = env_l.obs_frame
    hist = env_l.obs_hist_len
    off_prev = 3 * 3 + 2 * nj         # q, dq, up, linvel, gyro, then prev_action
    prev_idx = np.concatenate([f * frame + off_prev + np.arange(nj) for f in range(hist)])
    assert env_f.obs_size == env_l.obs_size == mean.shape[0]
    names = env_l._act_names
    print(f"[{time.time()-t0:5.0f}s] envs built; obs {env_l.obs_size}, act {nj}")
    for n, x, y in zip(names, rp, rm):
        print(f"   {n:13s} r+ {x:6.2f}  r- {y:5.2f}")

    # 1) the legacy policy in its own env
    m32 = lambda x: jp.asarray(x, jp.float32)  # noqa: E731
    L32 = [(m32(w), m32(b)) for w, b in layers]
    _, act_l = make_forward(m32(mean), m32(std), L32, nj)
    obs, act, alive, rew_l, alive_l = rollout(env_l, act_l, a.envs, a.steps, a.seed)
    print(f"[{time.time()-t0:5.0f}s] legacy rollout: survive {alive_l.mean():.3f}, "
          f"reward/step {rew_l[alive].mean():.3f}")
    X = obs[alive].astype(np.float64)              # (S, D)
    A = act[alive].astype(np.float64)              # (S, A)

    # 2) observation: prev_action blocks through g, normalizer refit on those dims
    XF = X.copy()
    xp = X[:, prev_idx]
    gp = np.concatenate([to_full(xp[:, f * nj:(f + 1) * nj]) for f in range(hist)], 1)
    XF[:, prev_idx] = gp
    mean_f, std_f = mean.copy(), std.copy()
    for k, i in enumerate(prev_idx):
        G = np.stack([gp[:, k], np.ones(len(gp))], 1)
        (alpha, beta), *_ = np.linalg.lstsq(G, xp[:, k], rcond=None)
        alpha = max(alpha, 1e-3)
        mean_f[i] = (mean[i] - beta) / alpha
        std_f[i] = std[i] / alpha

    # 3) last-layer loc refit
    hid_f, _ = make_forward(mean_f, std_f, layers, nj)
    H = np.asarray(jax.jit(hid_f)(XF))
    # start: linear LS on atanh of the mapped action (clipped at 0.9: a legacy
    # policy pressing a joint into its stop -- the knee's 5 deg -- maps to
    # a_full = 1, loc = inf), then Gauss-Newton on the SERVED TARGET (deg),
    # the quantity that has to match
    def tgt_full(x):
        return home + np.where(x >= 0, (hi - home) * x, (home - lo) * x)
    tgt_l = legacy_target(A)
    target = np.clip(to_full(A), -0.9, 0.9)
    Y = np.arctanh(target)
    Hb = np.concatenate([H, np.ones((len(H), 1))], 1)
    reg = a.ridge * len(H) * np.eye(Hb.shape[1])
    reg[-1, -1] = 0.0
    sol = np.linalg.solve(Hb.T @ Hb + reg, Hb.T @ Y)
    for j in range(nj):
        w = sol[:, j]
        for _ in range(30):
            z = Hb @ w
            t = np.tanh(z)
            span = np.where(t >= 0, hi[j] - home[j], home[j] - lo[j])
            r = home[j] + span * t - tgt_l[:, j]
            Jz = (span * (1.0 - t * t))[:, None] * Hb
            step = np.linalg.solve(Jz.T @ Jz + reg * span.mean() ** 2, -(Jz.T @ r))
            w_try = w + step
            t2 = np.tanh(Hb @ w_try)
            r2 = home[j] + np.where(t2 >= 0, hi[j] - home[j], home[j] - lo[j]) * t2 - tgt_l[:, j]
            if (r2 ** 2).mean() < (r ** 2).mean():
                w = w_try
            else:
                break
        sol[:, j] = w
    w_new = layers[-1][0].copy()
    b_new = layers[-1][1].copy()
    w_new[:, :nj] = sol[:-1]
    b_new[:nj] = sol[-1]
    new_layers = layers[:-1] + [(w_new, b_new)]
    if a.bc_steps > 0:
        new_layers = bc_finetune(new_layers, mean_f, std_f, XF, tgt_l, home, lo, hi,
                                 nj, a.bc_steps, a.seed, t0)
    w_new, b_new = new_layers[-1]
    hid_f2, _ = make_forward(mean_f, std_f, new_layers, nj)
    H = np.asarray(jax.jit(hid_f2)(XF))
    Hb = np.concatenate([H, np.ones((len(H), 1))], 1)
    sol = np.concatenate([w_new[:, :nj], b_new[None, :nj]], 0)
    a_fit = np.tanh(Hb @ sol)
    err = np.degrees(tgt_full(a_fit) - tgt_l)
    rms = np.sqrt((err ** 2).mean(0))
    p99 = np.percentile(np.abs(err), 99, axis=0)
    print(f"[{time.time()-t0:5.0f}s] refit on {len(H)} samples: served-target error "
          f"rms / p99 (deg) per joint:")
    for n, r, q in zip(names, rms, p99):
        print(f"   {n:13s} {r:5.2f} / {q:5.2f}")

    # 4) closed loop: converted policy in the full-map env, same seeds
    F32 = [(m32(w), m32(b)) for w, b in new_layers]
    _, act_f = make_forward(m32(mean_f), m32(std_f), F32, nj)
    _, _, alive2, rew_f, alive_f = rollout(env_f, act_f, a.envs, a.steps, a.seed)
    print(f"[{time.time()-t0:5.0f}s] full-map rollout of the converted policy: survive "
          f"{alive_f.mean():.3f} (legacy {alive_l.mean():.3f}), reward/step "
          f"{rew_f[alive2].mean():.3f} (legacy {rew_l[alive].mean():.3f})")
    # naive baseline: the legacy weights served through the full map unchanged
    _, _, alive3, rew_n, alive_n = rollout(env_f, act_l, a.envs, a.steps, a.seed)
    print(f"   (unconverted legacy weights on the full map: survive {alive_n.mean():.3f}, "
          f"reward/step {rew_n[alive3].mean():.3f})")

    # write
    sv = np.asarray(norm.summed_variance, np.float64)
    sv_f = sv * (std_f / std) ** 2
    norm_f = norm.replace(mean=jp.asarray(mean_f, norm.mean.dtype),
                          std=jp.asarray(std_f, norm.std.dtype),
                          summed_variance=jp.asarray(sv_f, norm.summed_variance.dtype))
    pol_f = {"params": {k: {
        "kernel": jp.asarray(w, pol["params"][k]["kernel"].dtype),
        "bias": jp.asarray(b, pol["params"][k]["bias"].dtype)}
        for k, (w, b) in zip(keys, new_layers)}}
    out_params = (norm_f, pol_f) + tuple(params[2:])
    cfg_f = dict(cfg)
    cfg_f["action_map"] = "full"
    if a.within_stops:
        cfg_f["ctrl_within_stops"] = True
    if isinstance(cfg_f.get("train"), dict):
        cfg_f["train"] = dict(cfg_f["train"], action_map="full",
                              ctrl_within_stops=bool(a.within_stops))
    cfg_f["converted_from"] = dict(
        run=a.src, tool="tools/convert_action_map.py", samples=int(len(H)),
        envs=a.envs, steps=a.steps, seed=a.seed, ridge=a.ridge,
        within_stops=bool(a.within_stops), bc_steps=a.bc_steps,
        target_err_rms_deg={n: round(float(r), 3) for n, r in zip(names, rms)},
        survive_legacy=float(alive_l.mean()), survive_converted=float(alive_f.mean()),
        survive_unconverted=float(alive_n.mean()),
        reward_legacy=float(rew_l[alive].mean()), reward_converted=float(rew_f[alive2].mean()))
    for runs in [a.runs] + ([a.also_runs] if a.also_runs else []):
        dst = os.path.join(runs, a.dst)
        if os.path.exists(os.path.join(dst, "params.pkl")):
            raise SystemExit(f"{dst}/params.pkl exists; pick another --dst")
        os.makedirs(dst, exist_ok=True)
        with open(os.path.join(dst, "params.pkl"), "wb") as f:
            pickle.dump(out_params, f)
        with open(os.path.join(dst, "config.json"), "w") as f:
            json.dump(cfg_f, f, indent=2)
        print(f"wrote {dst}")


if __name__ == "__main__":
    main()
