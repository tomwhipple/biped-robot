#!/usr/bin/env python3
"""Distill a big brax teacher into the (128, 128) net the firmware can hold.

    .venv/bin/python sim/mjx/distill_student.py --teacher loco_v12knee_warm \
        --out loco_v12knee_warm_s128

WHY: firmware-design section 6. The precision runs train a (512, 256, 128)
MLP -- ~232k params, ~930 KB fp32 -- which does not fit the WROOM. The decided
path is "train big, distill small": a (128, 128) student is ~38k params,
~150 KB fp32, and that lives in flash DROM as a constexpr, not in SRAM.

The method is DAgger -- aggregated dataset, teacher-mix schedule -- built on
the training stack's own pieces:

  * rollouts in BimoMJXEnv on the GPU, from the teacher's OWN config.json, so
    the state distribution is the plant/DR/command curriculum it was trained
    on (never a hand-written ENV_KW that can drift from the run),
  * the teacher's frozen running_statistics normalizer is reused VERBATIM by
    the student.  The firmware net consumes NORMALIZED obs and ships the
    normalizer next to the weights (docs/sil-harness.md); refitting it here
    would mean re-exporting a second normalizer and re-deriving the frozen-
    channel analysis for no gain.  Frozen it stays.
  * the student is a real brax PPO network, so the saved params.pkl is
    (normalizer, policy, value) exactly like train_mjx.py writes.  Both
    referee columns -- eval_precision.py and eval_precision.py --sil via
    tools/export_policy_weights.py -- load it with no shim.

LABEL: the teacher's DETERMINISTIC action, NormalTanhDistribution.mode() =
tanh(mean).  That is the quantity the robot executes and the quantity the
firmware golden vectors compare, so the regression is done in tanh space
(post-squash) rather than on the raw logits: a saturated teacher logit of 4 or
40 is the same command, and only the former is learnable at this width.

TEACHER MIX: round 0 is pure behaviour cloning (teacher drives every env).
From round 1 a fraction beta of the env slots are teacher-driven and the rest
run the student's own actions for the whole chunk -- whole trajectories, not
per-step coin flips, so the data actually contains the student's compounding
mistakes, which is the entire point of DAgger over BC.
"""
from __future__ import annotations

import argparse
import functools
import json
import os
import pickle
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
RUNS = os.path.join(ROOT, "sim", "runs")
sys.path.insert(0, HERE)

# train_mjx installs the jax.device_put_replicated shim brax 0.14 still needs
# on jax 0.10 and defines the batched/auto-resetting env adapter.  Importing it
# (main() is __main__-guarded) keeps ONE definition of that adapter.
from train_mjx import BatchedEnv                      # noqa: E402

import jax                                            # noqa: E402
import jax.numpy as jp                                # noqa: E402
import optax                                          # noqa: E402
from brax.training.acme import running_statistics     # noqa: E402
from brax.training.agents.ppo import networks as ppo_networks   # noqa: E402

from env_mjx import BimoMJXEnv                        # noqa: E402

# config.json keys that are provenance, not env kwargs
_NOT_ENV = ("train", "git_sha", "git_dirty", "distill", "_run")


def hidden_sizes(net_params, fallback):
    """MLP hidden sizes from checkpoint shapes -- same derivation as
    eval_precision._hidden_sizes (brax names every layer hidden_i and the
    last one is the output layer)."""
    try:
        layers = net_params["params"]
        ks = sorted((k for k in layers if k.startswith("hidden_")),
                    key=lambda k: int(k.split("_")[1]))
        return tuple(int(layers[k]["kernel"].shape[1]) for k in ks[:-1])
    except Exception:
        return fallback


def env_kwargs(cfg):
    """Rebuild the teacher's BimoMJXEnv kwargs from its config.json."""
    kw = {k: v for k, v in cfg.items() if k not in _NOT_ENV}
    for k, v in list(kw.items()):
        if isinstance(v, list):
            kw[k] = tuple(v)
    xml = kw.get("xml_path") or "bimo_biped_v3yaw.xml"
    if not os.path.isabs(xml):
        xml = os.path.join(ROOT, "sim", os.path.basename(xml))
    kw["xml_path"] = xml
    return kw


def make_net(obs_size, act_size, hidden, value_hidden=(256, 256)):
    return ppo_networks.make_ppo_networks(
        observation_size=obs_size, action_size=act_size,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=tuple(hidden),
        value_hidden_layer_sizes=tuple(value_hidden))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--teacher", default="loco_v12knee_warm")
    p.add_argument("--out", default=None,
                   help="run dir name (default <teacher>_s128)")
    p.add_argument("--hidden", default="128,128",
                   help="student hidden widths")
    p.add_argument("--envs", type=int, default=512)
    p.add_argument("--rounds", type=int, default=12,
                   help="DAgger rounds (round 0 is pure BC)")
    p.add_argument("--chunk", type=int, default=64,
                   help="env steps per scanned rollout chunk")
    p.add_argument("--chunks-per-round", type=int, default=4)
    p.add_argument("--epochs", type=int, default=6,
                   help="passes over the aggregated buffer per round")
    p.add_argument("--batch", type=int, default=4096)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--lr-final", type=float, default=1e-4)
    p.add_argument("--beta0", type=float, default=1.0,
                   help="teacher-driven env fraction in round 1")
    p.add_argument("--beta-decay", type=float, default=0.5)
    p.add_argument("--buffer", type=int, default=2_000_000,
                   help="max aggregated samples (FIFO past this)")
    p.add_argument("--act-noise", type=float, default=0.0,
                   help="gaussian noise added to EXECUTED actions only")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--holdout", type=int, default=20000)
    args = p.parse_args()

    t_dir = os.path.join(RUNS, args.teacher)
    out_name = args.out or f"{args.teacher}_s128"
    out_dir = os.path.join(RUNS, out_name)
    hidden = tuple(int(x) for x in args.hidden.split(","))

    with open(os.path.join(t_dir, "config.json")) as f:
        cfg = json.load(f)
    with open(os.path.join(t_dir, "params.pkl"), "rb") as f:
        t_params = pickle.load(f)
    norm_params = t_params[0]
    t_hidden = hidden_sizes(t_params[1], (512, 256, 128))

    env = BimoMJXEnv(**env_kwargs(cfg))
    wrapped = BatchedEnv(env, env.max_steps, num_envs=args.envs)
    obs_size, act_size = env.obs_size, env.action_size
    print(f"teacher {args.teacher}: {t_hidden} hidden, obs {obs_size}, "
          f"act {act_size}, plant {os.path.basename(cfg['xml_path'])}")
    print(f"student {out_name}: {hidden} hidden")

    t_net = make_net(obs_size, act_size, t_hidden,
                     hidden_sizes(t_params[2], (256, 256))
                     if len(t_params) > 2 else (256, 256))
    s_net = make_net(obs_size, act_size, hidden, hidden)
    dist = s_net.parametric_action_distribution

    key = jax.random.PRNGKey(args.seed)
    key, k_pol, k_val = jax.random.split(key, 3)
    s_policy = s_net.policy_network.init(k_pol)
    s_value = s_net.value_network.init(k_val)

    n_params = sum(int(np.size(x))
                   for x in jax.tree_util.tree_leaves(s_policy))
    print(f"student policy params: {n_params} "
          f"({n_params * 4 / 1024.0:.1f} KB fp32) + "
          f"{2 * obs_size} normalizer floats")

    # -- the three jitted primitives ------------------------------------
    @jax.jit
    def teacher_act(obs):
        logits = t_net.policy_network.apply(norm_params, t_params[1], obs)
        return dist.mode(logits)

    @jax.jit
    def student_act(policy, obs):
        logits = s_net.policy_network.apply(norm_params, policy, obs)
        return dist.mode(logits)

    @functools.partial(jax.jit, static_argnums=(3,))
    def rollout(state, policy, rng, steps, use_teacher, noise):
        """Scan `steps` env steps; return (state, obs[T,N,O], act[T,N,A],
        n_falls).  `use_teacher` is a per-env {0,1} mask held for the whole
        chunk, so the student's slots produce whole self-driven trajectories."""
        def body(carry, _):
            st, rg = carry
            obs = st.obs
            a_t = teacher_act(obs)
            a_s = student_act(policy, obs)
            m = use_teacher.reshape(-1, 1)
            a = jp.where(m > 0, a_t, a_s)
            rg, kn = jax.random.split(rg)
            a = jp.where(
                noise > 0,
                jp.clip(a + noise * jax.random.normal(kn, a.shape), -1.0, 1.0),
                a)
            nxt = wrapped.step(st, a)
            return (nxt, rg), (obs, a_t, nxt.done)
        (st, rng), (obs, act, done) = jax.lax.scan(
            body, (state, rng), None, length=steps)
        return st, obs, act, jp.sum(done)

    def loss_fn(policy, obs, tgt):
        logits = s_net.policy_network.apply(norm_params, policy, obs)
        pred = dist.mode(logits)
        return jp.mean((pred - tgt) ** 2)

    # cosine LR over the TOTAL minibatch count -- the buffer grows every
    # round, so that count has to be projected up front rather than guessed
    per_round = args.envs * args.chunk * args.chunks_per_round
    total_updates = max(1, sum(
        args.epochs * (min(args.buffer, per_round * (r + 1)) // args.batch)
        for r in range(args.rounds)))
    sched = optax.cosine_decay_schedule(
        args.lr, total_updates, alpha=args.lr_final / args.lr)
    opt = optax.chain(optax.clip_by_global_norm(1.0), optax.adam(sched))
    opt_state = opt.init(s_policy)
    print(f"optimizer: {total_updates:,d} projected minibatch steps, "
          f"lr {args.lr:g} -> {args.lr_final:g} (cosine)")

    @jax.jit
    def epoch_step(policy, opt_state, obs, tgt):
        """One optimizer step on one minibatch."""
        loss, grad = jax.value_and_grad(loss_fn)(policy, obs, tgt)
        updates, opt_state = opt.update(grad, opt_state, policy)
        return optax.apply_updates(policy, updates), opt_state, loss

    # -- DAgger ----------------------------------------------------------
    key, k_reset = jax.random.split(key)
    state = wrapped.reset(jax.random.split(k_reset, args.envs))

    X = np.zeros((0, obs_size), np.float32)
    Y = np.zeros((0, act_size), np.float32)
    hx = hy = None
    curve = []
    t0 = time.time()
    for r in range(args.rounds):
        beta = 1.0 if r == 0 else max(0.0, args.beta0 * args.beta_decay ** r)
        falls = 0
        obs_c, act_c = [], []
        for c in range(args.chunks_per_round):
            key, k_mix, k_roll = jax.random.split(key, 3)
            mask = (jax.random.uniform(k_mix, (args.envs,)) < beta
                    ).astype(jp.float32)
            state, o, a, nf = rollout(state, s_policy, k_roll, args.chunk,
                                      mask, jp.float32(args.act_noise))
            falls += int(nf)
            obs_c.append(np.asarray(o, np.float32).reshape(-1, obs_size))
            act_c.append(np.asarray(a, np.float32).reshape(-1, act_size))
        Xn = np.concatenate(obs_c)
        Yn = np.concatenate(act_c)
        if hx is None:          # first round: carve a fixed holdout
            n = min(args.holdout, len(Xn) // 4)
            hx, hy = jp.asarray(Xn[:n]), jp.asarray(Yn[:n])
            Xn, Yn = Xn[n:], Yn[n:]
        X = np.concatenate([X, Xn])[-args.buffer:]
        Y = np.concatenate([Y, Yn])[-args.buffer:]

        rng_np = np.random.default_rng(args.seed * 1000 + r)
        for ep in range(args.epochs):
            perm = rng_np.permutation(len(X))
            tot, nb = 0.0, 0
            for i in range(0, len(X) - args.batch + 1, args.batch):
                idx = perm[i:i + args.batch]
                s_policy, opt_state, l = epoch_step(
                    s_policy, opt_state, jp.asarray(X[idx]),
                    jp.asarray(Y[idx]))
                tot += float(l)
                nb += 1
            train_loss = tot / max(nb, 1)
        val = float(loss_fn(s_policy, hx, hy))
        vmae = float(jp.mean(jp.abs(student_act(s_policy, hx) - hy)))
        steps = (r + 1) * args.chunks_per_round * args.chunk * args.envs
        rec = dict(round=r, beta=round(beta, 4), dataset=int(len(X)),
                   env_steps=int(steps), bc_loss=train_loss, val_mse=val,
                   val_mae=vmae, rollout_falls=falls,
                   t=round(time.time() - t0, 1))
        curve.append(rec)
        print(f"round {r:2d}  beta {beta:4.2f}  data {len(X):>9,d}  "
              f"BC mse {train_loss:.5f}  val mse {val:.5f}  "
              f"val mae {vmae:.4f}  falls {falls:>5d}  "
              f"[{time.time() - t0:6.0f}s]", flush=True)

    # -- save as a normal brax run dir ------------------------------------
    os.makedirs(out_dir, exist_ok=True)
    params = (norm_params, jax.device_get(s_policy), jax.device_get(s_value))
    with open(os.path.join(out_dir, "params.pkl"), "wb") as f:
        pickle.dump(params, f)

    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
                             capture_output=True, timeout=10).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT,
            text=True, capture_output=True, timeout=10).stdout.strip())
    except Exception:
        sha, dirty = None, None
    out_cfg = dict(cfg)
    out_cfg["distill"] = dict(
        teacher=args.teacher, teacher_hidden=list(t_hidden),
        student_hidden=list(hidden), student_params=int(n_params),
        normalizer="teacher, frozen", label="tanh(mean), deterministic mode",
        rounds=args.rounds, envs=args.envs, chunk=args.chunk,
        chunks_per_round=args.chunks_per_round, epochs=args.epochs,
        batch=args.batch, lr=args.lr, lr_final=args.lr_final,
        beta0=args.beta0, beta_decay=args.beta_decay, buffer=args.buffer,
        act_noise=args.act_noise, seed=args.seed,
        env_steps=int(args.rounds * args.chunks_per_round * args.chunk
                      * args.envs),
        curve=curve)
    out_cfg["git_sha"] = sha
    out_cfg["git_dirty"] = dirty
    with open(os.path.join(out_dir, "config.json"), "w") as f:
        json.dump(out_cfg, f, indent=2)
    with open(os.path.join(out_dir, "distill.jsonl"), "w") as f:
        for rec in curve:
            f.write(json.dumps(rec) + "\n")
    # the teacher's own progress log is useful provenance next to the student
    src = os.path.join(t_dir, "config.json")
    if os.path.exists(src):
        shutil.copyfile(src, os.path.join(out_dir, "teacher_config.json"))
    print(f"saved -> {out_dir}  ({time.time() - t0:.0f}s, "
          f"{curve[-1]['env_steps']:,d} env steps)")


if __name__ == "__main__":
    main()
