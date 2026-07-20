"""Brax-PPO training on the MJX env (stage 2b) -- runs on Mira's RTX 4070 Ti.

Wraps BimoMJXEnv for brax's PPO with a hand-rolled batch wrapper instead of
brax's stock wrappers, because stock AutoReset restores the cached first
state WITHOUT re-drawing our per-episode DR fields (latency, backlash, servo
gains, IMU error, command) -- reseed() keeps that diversity alive.

The trained policy is saved as params.pkl + config.json under
sim/runs/<out>/. Evaluate with sim/mjx/eval_ref.py, which runs the policy
in the CPU BimoWalkerEnv (the referee) -- never trust MJX numbers alone.

Run (on Mira):
  .venv/bin/python sim/mjx/train_mjx.py --out mjx_cmd_v1 --steps 150000000
"""
import argparse
import functools
import json
import os
import pickle
import time

import jax
import jax.numpy as jp

# brax 0.14 still calls jax.device_put_replicated, removed in jax 0.10.
# Old semantics: stack x once per device (leading axis = n_devices) and place
# one shard on each. Restore it (per the migrate_pmap drop-in) before brax
# imports resolve the attribute.
if not hasattr(jax, "device_put_replicated"):
    def _device_put_replicated(x, devices):
        from jax.sharding import Mesh, NamedSharding, PartitionSpec
        import numpy as _np
        mesh = Mesh(_np.array(devices), ("_dpr",))
        sharding = NamedSharding(mesh, PartitionSpec("_dpr"))
        return jax.tree_util.tree_map(
            lambda l: jax.device_put(
                jp.broadcast_to(l, (len(devices),) + jp.shape(l)), sharding),
            x)
    jax.device_put_replicated = _device_put_replicated

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")

from brax.envs import base as brax_base
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo

from env_mjx import BimoMJXEnv


class BatchedEnv(brax_base.Env):
    """Batched, episodic, auto-resetting adapter: BimoMJXEnv -> brax Env.
    Used with ppo.train(wrap_env=False), so this owns vmapping, truncation
    at episode_length, and auto-reset (cached first state + reseed)."""

    def __init__(self, env: BimoMJXEnv, episode_length: int):
        self._env = env
        self._len = episode_length

    @property
    def observation_size(self):
        return self._env.obs_size

    @property
    def action_size(self):
        return self._env.action_size

    @property
    def backend(self):
        return "mjx"

    def reset(self, rng: jax.Array) -> brax_base.State:
        st = jax.vmap(self._env.reset)(rng)
        n = rng.shape[0]
        zeros = jp.zeros(n)
        # info contract brax's PPO actor/evaluator expects (normally provided
        # by EpisodeWrapper): steps/truncation/episode_done/episode_metrics
        episode_metrics = {"sum_reward": zeros, "length": zeros}
        episode_metrics.update({k: zeros for k in st.metrics})
        info = {"st": st, "first_st": st, "steps": zeros,
                "truncation": zeros, "episode_done": zeros,
                "episode_metrics": episode_metrics}
        return brax_base.State(pipeline_state=st.data, obs=st.obs,
                               reward=st.reward, done=st.done,
                               metrics=dict(st.metrics), info=info)

    def step(self, state: brax_base.State, action: jax.Array) -> brax_base.State:
        st = jax.vmap(self._env.step)(state.info["st"], action)
        prev_done = state.info["episode_done"]
        # steps: zeroed at the START of the step AFTER done (EvalWrapper reads
        # the full count at the done step itself -- brax wrapper timing)
        steps = jp.where(prev_done > 0, jp.zeros_like(state.info["steps"]),
                         state.info["steps"]) + 1.0
        trunc = jp.where(steps >= self._len, 1.0 - st.done, 0.0)
        done = jp.maximum(st.done, (steps >= self._len).astype(jp.float32))

        # episode metric aggregation (EpisodeWrapper semantics)
        em = dict(state.info["episode_metrics"])
        em["sum_reward"] = em["sum_reward"] * (1 - prev_done) + st.reward
        em["length"] = em["length"] * (1 - prev_done) + 1.0
        for k in st.metrics:
            em[k] = em[k] * (1 - prev_done) + st.metrics[k]

        # auto-reset (cached first physics state, re-drawn per-episode DR)
        first = state.info["first_st"]
        reseeded = jax.vmap(self._env.reseed)(first, st.rng)

        def pick(a, b):
            d = done.reshape(done.shape + (1,) * (a.ndim - 1))
            return jp.where(d > 0, a, b)

        nxt = jax.tree_util.tree_map(pick, reseeded, st)
        info = {"st": nxt, "first_st": first, "steps": steps,
                "truncation": trunc, "episode_done": done,
                "episode_metrics": em}
        return brax_base.State(pipeline_state=nxt.data, obs=nxt.obs,
                               reward=st.reward, done=done,
                               metrics=dict(st.metrics), info=info)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="mjx_cmd_v1")
    p.add_argument("--steps", type=int, default=150_000_000)
    p.add_argument("--envs", type=int, default=2048)
    p.add_argument("--episode-seconds", type=float, default=10.0)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--entropy", type=float, default=1e-2)
    p.add_argument("--discounting", type=float, default=0.97)
    p.add_argument("--unroll", type=int, default=20)
    p.add_argument("--minibatches", type=int, default=32)
    p.add_argument("--updates", type=int, default=4)
    p.add_argument("--batch-size", type=int, default=1024)
    p.add_argument("--num-evals", type=int, default=20)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--cmd-dense", action="store_true")
    p.add_argument("--payload", type=float, default=0.154)   # GoPro on
    p.add_argument("--getup", action="store_true",
                   help="fall-recovery objective instead of command tracking")
    p.add_argument("--action-map", default="legacy", choices=["legacy", "full"])
    p.add_argument("--precision", action="store_true",
                   help="7-channel precision command curriculum (ext_cmd): "
                        "sidestep, backward, crouch, one-leg balance, air "
                        "circles; severe fall cost; as-built plant default")
    p.add_argument("--xml", default=None,
                   help="plant override (default: v2; precision default: "
                        "bimo_biped_v2_asbuilt.xml -- the 100 mm printed feet)")
    p.add_argument("--fall-cost", type=float, default=None)
    p.add_argument("--walk-submix", default=None,
                   help="backward,sidestep fractions of walk commands, "
                        "e.g. 0.25,0.30")
    p.add_argument("--recover-mix", type=float, default=None,
                   help="override the fraction of recovery-start episodes "
                        "(1.0 = a dedicated recovery expert)")
    p.add_argument("--recover-start-mix", default=None,
                   help="ragdoll,kneel,squat,sit fractions, e.g. 0.2,0.2,0.2,0.4")
    p.add_argument("--w-pitch-rate", type=float, default=None,
                   help="override torso roll/pitch-rate penalty (smoothness)")
    p.add_argument("--w-action-rate", type=float, default=None)
    p.add_argument("--w-power", type=float, default=None)
    p.add_argument("--init-from", default=None,
                   help="warm-start from sim/runs/<name>/params.pkl (same "
                        "objective only -- objective changes need from-"
                        "scratch, a lesson learned three times over)")
    p.add_argument("--hip-flex", type=float, default=None,
                   help="widen hip flexion to this many degrees (study: >=95)")
    p.add_argument("--getup-mix", default="1,0,0",
                   help="getup reset mix: ragdoll,kneel,squat fractions")
    args = p.parse_args()

    # env config: the exp_walk reward shape (the day-5 tuned set), full-length
    # episodes, hardened from step 0 (imu_obs + latency/backlash/push DR)
    env_kw = dict(
        episode_seconds=args.episode_seconds,
        w_upright=0.8, alive_bonus=0.3, w_height=0.3, w_energy=0.002,
        w_action_rate=0.15, fall_cost=6.0,
        w_feet_air=20.0, air_time_target=0.3, w_single_support=0.3,
        w_lateral=0.5, w_pitch_rate=0.25, w_power=0.008,
        supply_voltage=11.1,
        payload_mass=args.payload,
        domain_rand=True,
        latency_ms=0.0, latency_ms_max=8.0, latency_jitter_ms=1.0,
        backlash_deg=0.5, backlash_deg_max=1.0,
        cmd_v_range=(0.3, 1.0), cmd_w_range=1.0, cmd_stand_prob=0.3,
        cmd_resample_s=(2.5, 4.5), cmd_dense=args.cmd_dense,
        w_track_v=2.0, w_track_w=2.0,
        imu_obs=True, imu_noise=1.0,
        action_map=args.action_map, hip_flex_deg=args.hip_flex,
    )
    if args.precision:
        # precision round (2026-07-17): the 7 user skills as one command-
        # conditioned policy, trained on the AS-BUILT plant (100 mm printed
        # feet) with the true camera CG. Falls are severely penalized
        # (fall_cost 10 on top of episode termination).
        env_kw.update(
            ext_cmd=True,
            w_track_h=1.0, w_lift=1.0, w_track_foot=1.0,
            fall_cost=10.0,
            payload_cg_z=0.0945,
            xml_path=os.path.join(HERE, "..", "bimo_biped_v2_asbuilt.xml"),
            # dense directional progress: without it precision_v1 converged
            # to a 0-falls/0-motion standing optimum (kernels pay standers)
            cmd_dense=True,
            # user feedback 2026-07-18: lifted foot must clear 3 cm (env
            # default lift_clear=0.03 applies); 15% of env slots practice
            # recovery-from-fallen; single-leg crouch removed from the mix
            recover_mix=0.15,
            # user feedback 2026-07-19: gait-symmetry pressure + reverse
            # curriculum for the recovery slots (ragdoll/kneel/squat starts;
            # the referee still grades pure ragdoll)
            w_symmetry=1.0,
            # sit-first reverse curriculum (user 2026-07-19): the sit is
            # where fallen robots naturally end up AND the start of the
            # study's rise path -- make it the dominant training start
            recover_start_mix=(0.2, 0.2, 0.2, 0.4),
        )
    if args.xml:
        env_kw["xml_path"] = args.xml
    if args.fall_cost is not None:
        env_kw["fall_cost"] = args.fall_cost
    if args.walk_submix is not None:
        env_kw["walk_submix"] = tuple(
            float(x) for x in args.walk_submix.split(","))
    if args.recover_mix is not None:
        env_kw["recover_mix"] = args.recover_mix
    if args.recover_start_mix is not None:
        env_kw["recover_start_mix"] = tuple(
            float(x) for x in args.recover_start_mix.split(","))
    if args.w_pitch_rate is not None:
        env_kw["w_pitch_rate"] = args.w_pitch_rate
    if args.w_action_rate is not None:
        env_kw["w_action_rate"] = args.w_action_rate
    if args.w_power is not None:
        env_kw["w_power"] = args.w_power
    if args.getup:
        # recovery objective: gait shaping off (crawling/rolling is fine),
        # recovery terms carry the gradient; shorter episodes; same hardening
        env_kw.update(
            getup=True, episode_seconds=8.0,
            w_upright=0.0, alive_bonus=0.0, w_height=0.0,
            w_feet_air=0.0, w_single_support=0.0, w_lateral=0.0,
            w_pitch_rate=0.1, w_action_rate=0.15, w_power=0.008,
            w_recover_h=1.0, w_recover_up=0.8, stand_bonus=1.0,
            getup_start_mix=tuple(float(x) for x in args.getup_mix.split(",")),
        )
    env = BimoMJXEnv(**env_kw)
    episode_length = env.max_steps
    wrapped = BatchedEnv(env, episode_length)

    out = os.path.join(RUNS, args.out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "config.json"), "w") as f:
        cfg = {k: (list(v) if isinstance(v, tuple) else v)
               for k, v in env_kw.items()}
        if "xml_path" in cfg:
            # store the basename: the run may train on a remote host whose
            # absolute path means nothing locally -- eval harnesses resolve
            # the basename against the local sim/ directory
            cfg["xml_path"] = os.path.basename(cfg["xml_path"])
        cfg.update(train=vars(args))
        json.dump(cfg, f, indent=2)
    log_path = os.path.join(out, "progress.jsonl")
    t0 = time.time()

    def progress(num_steps, metrics):
        rec = {"t": round(time.time() - t0, 1), "steps": int(num_steps)}
        rec.update({k: float(v) for k, v in metrics.items()
                    if isinstance(v, (int, float)) or hasattr(v, "item")})
        with open(log_path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        ep = metrics.get("eval/avg_episode_length", float("nan"))
        rew = metrics.get("eval/episode_reward", float("nan"))
        print(f"[{rec['t']:8.1f}s] {num_steps:>12,d} steps  "
              f"ep_len {float(ep):6.1f}/{episode_length}  "
              f"reward {float(rew):9.1f}", flush=True)

    def save_ckpt(step, make_policy, params):
        with open(os.path.join(out, "params.pkl"), "wb") as f:
            pickle.dump(params, f)   # (normalizer, policy, value)

    restore = None
    if args.init_from:
        with open(os.path.join(RUNS, args.init_from, "params.pkl"), "rb") as f:
            restore = pickle.load(f)
        print(f"warm-starting from {args.init_from}/params.pkl")

    train_fn = functools.partial(
        ppo.train,
        restore_params=restore,
        num_timesteps=args.steps,
        num_evals=args.num_evals,
        episode_length=episode_length,
        num_envs=args.envs,
        batch_size=args.batch_size,
        unroll_length=args.unroll,
        num_minibatches=args.minibatches,
        num_updates_per_batch=args.updates,
        learning_rate=args.lr,
        entropy_cost=args.entropy,
        discounting=args.discounting,
        normalize_observations=True,
        reward_scaling=1.0,
        network_factory=functools.partial(
            ppo_networks.make_ppo_networks,
            policy_hidden_layer_sizes=(128, 128),
            value_hidden_layer_sizes=(256, 256)),
        seed=args.seed,
        wrap_env=False,
        policy_params_fn=save_ckpt,
    )
    make_inference_fn, params, metrics = train_fn(
        environment=wrapped, progress_fn=progress)

    with open(os.path.join(out, "params.pkl"), "wb") as f:
        pickle.dump(params, f)
    print(f"saved -> {out}  ({time.time() - t0:.0f}s total)")


if __name__ == "__main__":
    main()
