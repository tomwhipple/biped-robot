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
import subprocess
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

from env_mjx import BimoMJXEnv, domain_randomize


class BatchedEnv(brax_base.Env):
    """Batched, episodic, auto-resetting adapter: BimoMJXEnv -> brax Env.
    Used with ppo.train(wrap_env=False), so this owns vmapping, truncation
    at episode_length, and auto-reset (cached first state + reseed).

    Batch-level domain randomization (mass/inertia/friction/payload): when the
    wrapped env has domain_rand=True, the model is randomized PER ENV via
    env_mjx.domain_randomize() and the batched model is threaded through
    reset/step with jax.vmap -- the MJX equivalent of the CPU env's
    _randomize_dynamics() on every reset. This closes the sim-to-real gap
    where the flagship mjx_cmd_v1 policy was trained without ever seeing
    mass/friction variation (Percy finding GH#30)."""

    def __init__(self, env: BimoMJXEnv, episode_length: int, num_envs: int = 1):
        self._env = env
        self._len = episode_length
        # Batch-level DR: one randomized model per parallel env, drawn once
        # per batch size (the standard MJX pattern -- with thousands of envs
        # the per-gradient-batch diversity is far higher than the CPU env's
        # per-episode redraw). brax resets THIS SAME env with num_envs while
        # training and with num_eval_envs (default 128) in the evaluator, so
        # the batched model is built per batch size and cached -- one model
        # batch pinned to num_envs blows up the evaluator's vmap with
        # "inconsistent sizes for array axes to be mapped".
        self._dr_key = jax.random.PRNGKey(0)
        self._dr_cache = {}
        if env.domain_rand:
            self._batched_model(num_envs)      # warm the training batch

    def _batched_model(self, n: int):
        """(model, in_axes) for a batch of n envs. DR off -> (single model,
        None), i.e. vmap broadcasts the one model (unchanged path)."""
        if not self._env.domain_rand:
            return self._env.model, None
        if n not in self._dr_cache:
            e = self._env
            payload_bid = e._payload_bid
            payload_max = e.payload_max if payload_bid is not None else None
            # brax first asks for the eval batch size from inside its jitted
            # reset; the cache must hold CONCRETE arrays, not that trace's
            # tracers (they leak into later traces -- opt.gravity DR tripped
            # this). All inputs are concrete closures, so evaluate eagerly.
            with jax.ensure_compile_time_eval():
                self._dr_cache[n] = domain_randomize(
                    e.model, jax.random.split(self._dr_key, n),
                    mass_range=e.mass_range, friction_range=e.friction_range,
                    payload_max=payload_max, payload_bid=payload_bid,
                    floor_gid=e._floor_gid, nom_mass=e._nom_mass,
                    nom_inertia=e._nom_inertia, nom_friction=e._nom_friction,
                    tilt_max_deg=e.tilt_max_deg)
        return self._dr_cache[n]

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
        model, model_in_axes = self._batched_model(rng.shape[0])
        if model_in_axes is not None:
            st = jax.vmap(self._env.reset, in_axes=(0, model_in_axes))(
                rng, model)
        else:
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
        model, model_in_axes = self._batched_model(action.shape[0])
        if model_in_axes is not None:
            st = jax.vmap(self._env.step, in_axes=(0, 0, model_in_axes))(
                state.info["st"], action, model)
        else:
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
        if model_in_axes is not None:
            reseeded = jax.vmap(self._env.reseed,
                                in_axes=(0, 0, model_in_axes))(
                first, st.rng, model)
        else:
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
    p.add_argument("--terrain", action="store_true",
                   help="train on the tiled terrain mosaic (0-20 mm rough "
                        "ground, per-episode spawn = per-episode roughness)")
    p.add_argument("--mimic-knee-w", type=float, default=1.0,
                   help="knee weight in the gait-imitation kernel (the "
                        "lump-sum kernel let the knee stay jammed straight)")
    p.add_argument("--cmd-v-range", default="0.3,1.0",
                   help="forward-speed command range, m/s (loco_v6: "
                        "0.05,1.0 teaches the creep band the goal-homing "
                        "outer loop needs)")
    p.add_argument("--discovery", action="store_true",
                   help="stage-1 clean-physics discovery: no DR/payload/"
                        "latency/backlash/pushes (stage 2 re-hardens)")
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
    p.add_argument("--family", choices=["all", "loco", "skills", "getup"],
                   default="all",
                   help="specialist policy family (plan v2 / progress "
                        "review 2026-07-22): restricts the command mix so "
                        "each policy masters one skill family")
    p.add_argument("--w-mimic", type=float, default=None)
    p.add_argument("--kick-range", default=None,
                   help="velocity-kick magnitudes, e.g. 0.05,0.3 (m/s)")
    p.add_argument("--push-prob", type=float, default=None,
                   help="per-step perturbation probability")
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
    p.add_argument("--w-pitch-hinge", type=float, default=None,
                   help="quadratic penalty on |torso pitch| past the deadband "
                        "(the direct lever on the forward-lean 'controlled "
                        "fall'; 0 = off, the default)")
    p.add_argument("--pitch-deadband", type=float, default=None,
                   help="free torso-pitch band in DEGREES before "
                        "--w-pitch-hinge bites (default 5)")
    p.add_argument("--w-action-rate", type=float, default=None)
    p.add_argument("--w-symmetry", type=float, default=None,
                   help="gait-symmetry penalty: on touchdown, the swing-duration\nmismatch vs the other foot. Was pinned at 1.0 and untunable until 2026-08-06.")
    p.add_argument("--w-power", type=float, default=None)
    p.add_argument("--w-foot-under", type=float, default=None,
                   help="raised-foot-under-hip kernel (knee-flexion lifts)")
    p.add_argument("--w-up-vel", type=float, default=None,
                   help="momentum-friendly rise incentive while down")
    p.add_argument("--w-rise-ref", type=float, default=None,
                   help="override the staged-rise reference weight")
    p.add_argument("--turn-emph", action="store_true",
                   # NOTE the %%: argparse %-formats help strings, and python
                   # 3.14 VALIDATES them inside add_argument -- the bare "85%"
                   # made every train_mjx.py invocation die with "badly formed
                   # help string" on 3.14 (found while adding --quantize)
                   help="sustained-turn command emphasis (loco_v5t): 85%% of "
                        "forward walks turn, |wz| floored at 0.25 rad/s")
    p.add_argument("--w-heading", type=float, default=None,
                   help="integrated-heading kernel weight (turn-to-face "
                        "authority; loco family experiment 2026-07-23)")
    p.add_argument("--w-com-stance", type=float, default=None,
                   help="CoM-over-stance-foot kernel while lifted "
                        "(knee-flexion balance lifts)")
    p.add_argument("--march-mix", type=float, default=None,
                   help="fraction of ext_cmd episodes drawn as a KNEE-HIGH "
                        "march: marching in place with alternating "
                        "exaggerated knee lifts (swing sole raised to the "
                        "opposite knee's standing height, ~2x normal swing "
                        "clearance) at zero net translation. Expressed "
                        "through the existing 7 command channels; pair with "
                        "--w-knee-high to pay for the clearance")
    p.add_argument("--w-mirror-loss", type=float, default=None,
                   help="mirror-symmetry auxiliary loss weight: penalize "
                        "|policy(mirror(obs)) - mirror(policy(obs))| on the "
                        "pre-tanh action mean, using the physics-verified "
                        "signed permutations in mirror.py (tests/"
                        "test_mirror.py). The structural answer to the "
                        "20-35%% gait asymmetry that w-symmetry reward "
                        "pressure provably does not fix (2026-08-14)")
    p.add_argument("--mimic-crouch-gate-knee", action="store_true",
                   help="knee-only crouch gate: during cmd[3]<0.97 zero the "
                        "knee+ankle mimic components, keep hip swing/phase "
                        "paying (alternative to --mimic-crouch-gate)")
    p.add_argument("--mimic-crouch-gate", action="store_true",
                   help="mimic term off while a crouch is commanded "
                        "(cmd[3] < 0.97): the zero-velocity reference has "
                        "straight knees and fought every crouch (2026-09-08)")
    p.add_argument("--clock-freeze-stand", action="store_true",
                   help="hold the gait-clock phase at a plain stand: the "
                        "sin/cos obs stop oscillating, removing the rhythmic "
                        "drive the policy otherwise must ignore to stand "
                        "still (2026-08-12)")
    p.add_argument("--w-still", type=float, default=None,
                   help="stand-gated joint-velocity penalty (-w*sum(dq^2) "
                        "while commanded to plain-stand): pays for stillness "
                        "itself, not just small corrections (2026-08-12)")
    p.add_argument("--w-stand-zero", type=float, default=None,
                   help="stand-gated action-magnitude penalty: zero command "
                        "-> zero action -> home pose (Tom 2026-09-04)")
    p.add_argument("--w-stand-home", type=float, default=None,
                   help="stand-gated L1 pull of the served target onto home, "
                        "radians (rest = home; replaces --w-stand-zero)")
    p.add_argument("--crouch-pose-ref", action="store_true",
                   help="mimic reference = the bench level-foot squat under a "
                        "crouch command; height targets use the feasible depth")
    p.add_argument("--crouch-theta-max-deg", type=float, default=None)
    p.add_argument("--crouch-rsi-mix", type=float, default=None,
                   help="fraction of env slots starting IN the squat reference "
                        "(reference-state initialization; needs --crouch-pose-ref)")
    p.add_argument("--w-crouch-pull", type=float, default=None,
                   help="dense L1 pull onto the squat reference (stationary crouch only)")
    p.add_argument("--w-hip-yaw", type=float, default=None,
                   help="sum(hip_yaw^2) penalty, ungated (2026-09-11)")
    p.add_argument("--w-crouch-track", type=float, default=None,
                   help="tight crouch-depth kernel, active under a crouch command")
    p.add_argument("--crouch-track-sigma", type=float, default=None)
    p.add_argument("--crouch-release", action="store_true",
                   help="release w_still/w_stand_com gates while a crouch is commanded")
    p.add_argument("--w-stand-com", type=float, default=None,
                   help="stand-gated CoM-over-midfoot kernel: a torque-off "
                        "stand only survives if the CoM stays within ~16 mm "
                        "of the support center (backdrive friction 0.35 Nm); "
                        "the v21 line stands 28 mm aft and topples "
                        "(stand_off 0/8, 2026-08-19)")
    p.add_argument("--speed-clock", action="store_true", default=None,
                   help="scale the gait clock with commanded planar speed "
                        "(sqrt law, x1.0 at 0.35 m/s, clip [0.7, 1.7]): the "
                        "schedule-pinned cadence capped speed at ~0.37 m/s "
                        "and made the 1.0 Hz metronome unreachable "
                        "(2026-08-25)")
    p.add_argument("--speed-clock-hi", type=float, default=None,
                   help="upper clip of the speed-clock multiplier. 1.7 "
                        "(2.55 Hz) exceeds the measured STS3215 swing-speed "
                        "envelope and cost the top end (v24clockv); 1.25 "
                        "(~1.9 Hz) stays inside it (2026-08-26)")
    p.add_argument("--w-stand-knee", type=float, default=None,
                   help="stand-gated knee-angle kernel toward +0.10 rad: a "
                        "dead-straight knee sag-collapses under torque-off "
                        "(knee+ankle fold together); the physics sweep says "
                        "a slight bias holds at ~5 cm (2026-08-24)")
    p.add_argument("--w-contact-sched", type=float, default=None,
                   help="clock cadence enforcement: each foot earns +-w for "
                        "matching the schedule's stance/swing flag (left "
                        "swings on sin>0, duty band 0.4). w_feet_phase only "
                        "shapes swing height; this pays for the TIMING -- "
                        "the 2026-08-17 'limp' is surge-stall cadence, not "
                        "left/right bias")
    p.add_argument("--sched-duty", type=float, default=None,
                   help="stance window edge in sin units for "
                        "--w-contact-sched (default 0.4 ~= 63%% stance)")
    p.add_argument("--ext-mix", default=None,
                   help="override the ext_cmd command mix as 7 comma floats "
                        "(stand,crouch,balance,air-circle,pivot,march,sway; "
                        "remainder = walk). Applied AFTER the family preset, "
                        "so a loco run can buy back the drill commands the "
                        "preset zeroes (weight_shift/metronome scenarios, "
                        "2026-08-06). Obs contract untouched")
    p.add_argument("--march-hz", type=float, default=None,
                   help="commanded march cadence in full L/R cycles per "
                        "second (1.0 = 0.5 s per lift). Omitted/0 alternates "
                        "on the gait clock instead, inheriting its 1.25-1.75 "
                        "Hz draw -- only ~165 ms of upswing to reach knee "
                        "height. A pinned cadence decouples march swing "
                        "timing from the gait-clock obs channel by design")
    p.add_argument("--w-foot-cross", type=float, default=None,
                   help="feet-crossing penalty weight (preset 0.5)")
    p.add_argument("--foot-cross-sep", type=float, default=None,
                   help="lateral sole separation (m) below which the "
                        "feet-crossing penalty pays (default 0.051)")
    p.add_argument("--swing-height", type=float, default=None,
                   help="swing-foot peak height target for w_feet_phase (m; preset 0.06)")
    p.add_argument("--w-feet-phase", type=float, default=None,
                   help="swing-height tracking kernel weight (preset 1.0)")
    p.add_argument("--feet-phase-s2", type=float, default=None,
                   help="swing-height kernel denominator (m^2; default 0.004 = "
                        "sigma 6.3 cm -- a 1 cm shuffle still earns 0.5 of a "
                        "6 cm target; 0.002 halves that)")
    p.add_argument("--w-knee-high", type=float, default=None,
                   help="knee-high clearance kernel weight: fraction of the "
                        "commanded swing height (lift_height + c6) the swing "
                        "sole reaches, on the correct one-foot contact "
                        "pattern (skill kernel for --march-mix)")
    p.add_argument("--init-from", default=None,
                   help="warm-start from sim/runs/<name>/params.pkl (same "
                        "objective only -- objective changes need from-"
                        "scratch, a lesson learned three times over)")
    p.add_argument("--hip-flex", type=float, default=None,
                   help="widen hip flexion to this many degrees (study: >=95)")
    p.add_argument("--getup-mix", default="1,0,0",
                   help="getup reset mix: ragdoll,kneel,squat fractions")
    p.add_argument("--quantize", action="store_true",
                   help="STS3215 encoder realism: joint pos/vel observations "
                        "and commanded targets quantized to the servo tick "
                        "grid (4096/rev) and reg-58 integer steps/s -- the "
                        "SIL boundary, inside training")
    p.add_argument("--cmd-crouch-range", default="1,1",
                   help="per-episode crouch-command draw, lo,hi (SIL finding "
                        "#2: cmd[3] frozen at 1.0 collapsed its normalizer "
                        "std, while firmware battguard ramps it below 1.0 on "
                        "a sagging pack). 1,1 = the legacy frozen channel")
    p.add_argument("--tilt-max", type=float, default=None,
                   help="batch-level gravity-tilt DR in degrees: each env's "
                        "gravity is tilted by U(0,max) about a random "
                        "azimuth -- an un-level floor (the bench desk "
                        "measures ~3.5 deg; user asked +-3, 2026-09-01)")
    p.add_argument("--backlash-deg", default=None,
                   help="per-episode gear backlash draw 'lo,hi' deg, applied "
                        "AFTER the --precision block (which zeroes it)")
    p.add_argument("--zero-offset-deg", type=float, default=None,
                   help="per-episode per-joint zero-offset DR, +-deg (the "
                        "hardware zero is set by eye; moved 1-4.7 deg 2026-09-03)")
    p.add_argument("--gyro-gain-range", default=None,
                   help="per-episode gyro OBS gain DR 'lo,hi' (2026-09-03: the "
                        "robot stands with the gyro obs at 0.5x, falls at 1x)")
    p.add_argument("--gyro-delay-max", type=int, default=None,
                   help="per-episode gyro OBS delay DR, 0..N ticks (max 2)")
    p.add_argument("--act-delay-max", type=int, default=None,
                   help="per-episode servo dead time DR, 0..N control ticks "
                        "(20 ms each); 2026-09-05 bench measured ~85 ms")
    p.add_argument("--init-pose-deg", type=float, default=None,
                   help="episode start-pose jitter half-width per joint, deg "
                        "(unset = the historical 1.7 deg); served target starts "
                        "at the jittered pose when set")
    p.add_argument("--act-lag", default=None,
                   help="per-episode action-chain lag pole draw, lo,hz "
                        "(3 cascaded stages, the deployed C2 shaper's "
                        "structure). Stand-in for shaper + servo dynamic "
                        "response: the 2026-08-31 bench walk measured joint "
                        "motion at ~0.5x sim, reproduced only by ~2 Hz of "
                        "3-stage filtering. e.g. '2,12'")
    args = p.parse_args()
    if args.precision and args.entropy == 1e-2:
        args.entropy = 0.005          # Playground's biped setting (plan v2)

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
        cmd_v_range=tuple(float(x) for x in args.cmd_v_range.split(",")),
        cmd_w_range=1.0, cmd_stand_prob=0.3,
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
            # Camera CG above the TORSO CENTRE. 2026-08-06: pelvis v6 deleted
            # the tower, so the 0.1260 that put the GoPro on a tower top is
            # 67.9 mm too high -- it floats the 154 g camera (12 % of the
            # robot) well clear of the body, visibly so in the render.
            # v6 bolts gopro_base flat to the deck: CAD cam_z is 54.0 mm over
            # the deck (GP_BASE_T + GP_HOLE_H + 6 + CAM_BODY_z/2, parts.py),
            # and the deck is 4.11 mm over the torso centre.
            # z from check_assembly.camera_mock()'s CORRECTED body bottom
            # (hole_z + GP_PRONG_OD/2 + 0.5, fixed 2026-08-04), not parts.py's
            # cam_z, which still carries the tower-era hole_z + 6.0 and is
            # 2 mm low. CG = 56.0 over the deck, deck = 4.11 over torso centre.
            payload_cg_z=0.0601,   # was 0.1260 (tower top), 0.0945 before that
            # And the camera is NOT on the centreline: it bolts to gopro_base
            # at GP_MOUNT_X. x was hard-coded 0 in both envs until 2026-08-06,
            # which floated it 24 mm forward of its own mount.
            payload_cg_x=-0.0240,  # GP_MOUNT_X
            # DEFAULT PLANT = the 10-DOF hip-yaw robot (user 2026-07-24:
            # "assume the 10-dof with the new pelvis for all simulations
            # going forward" -- the A/B verdict made yaw the build target).
            # 8-DOF plants remain available via --xml for legacy referees.
            # 2026-08-05: v3yaw and v4rom BOTH load the retired tower.stl,
            # which pelvis v6 deleted, so neither opens any more. v5body is
            # the pelvis-v6 plant and the only 10-DOF model that loads.
            xml_path=os.path.join(HERE, "..", "bimo_biped_v5body.xml"),
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
            w_symmetry=1.0,   # overridable via --w-symmetry below
            # sit-first reverse curriculum (user 2026-07-19): the sit is
            # where fallen robots naturally end up AND the start of the
            # study's rise path -- make it the dominant training start
            recover_start_mix=(0.2, 0.2, 0.2, 0.4),
            # articulation exercises + feet-crossing guard (user 2026-07-20)
            ext_mix=(0.15, 0.08, 0.12, 0.10, 0.08, 0.10, 0.07),
            w_foot_cross=0.5,
            # plan-v2 Phase A (docs/training-plan-v2.md): Playground-recipe
            # terms + Open Duck BAM servo constants + obs history
            gait_clock=True, w_feet_phase=1.0, swing_height=0.06,
            w_feet_slip=0.25, w_orientation=1.0, w_ang_vel_xy=0.15,
            w_pose=0.3, w_dof_limits=1.0,
            push_kick=True, obs_hist_len=3,
            joint_frictionloss=0.05, joint_armature=0.028,
            w_feet_air=5.0, w_pitch_rate=0.1,
        )
    if args.xml:
        # accept a bare basename (resolved against sim/, where the plants
        # live) or an explicit path -- night-queue args use basenames
        env_kw["xml_path"] = (args.xml if os.path.exists(args.xml)
                              else os.path.join(HERE, "..", args.xml))
    if args.fall_cost is not None:
        env_kw["fall_cost"] = args.fall_cost
    if args.family == "loco":
        # locomotion specialist: walk/backward/sidestep/turn/pivot/stand,
        # with the Phase B procedural-gait imitation prior
        env_kw.update(ext_mix=(0.20, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0),
                      walk_submix=(0.25, 0.30), recover_mix=0.0,
                      w_mimic=1.5)
    elif args.family == "skills":
        # balance-family specialist: stand/crouch/one-leg/circles/march/sway
        # (mix sums to 1.0 -> walk commands never drawn)
        env_kw.update(ext_mix=(0.15, 0.15, 0.25, 0.20, 0.0, 0.15, 0.10),
                      recover_mix=0.0,
                      # knee-flexion balance lifts (user 2026-07-23): pay
                      # for CoM planted over the support foot while lifted
                      w_com_stance=0.75)
    elif args.family == "getup":
        # w_rise_ref (getup_v3): the ratchet fixed the economics but PPO
        # never FOUND the rise -- dense staged-reference guidance added
        env_kw.update(recover_mix=1.0, w_rise_dofvel=0.002, w_rise_ref=2.0)
    if args.w_mimic is not None:
        env_kw["w_mimic"] = args.w_mimic
    if args.kick_range is not None:
        env_kw["kick_range"] = tuple(
            float(x) for x in args.kick_range.split(","))
    if args.push_prob is not None:
        env_kw["push_prob"] = args.push_prob
    if args.terrain:
        env_kw["terrain"] = True
    if args.mimic_knee_w != 1.0:
        env_kw["mimic_knee_w"] = args.mimic_knee_w
    if args.discovery:
        # getup_v11: stage-1 DISCOVERY physics (unified-humanoid-getup
        # recipe): strip DR, payload, latency, backlash and pushes so the
        # skill can be FOUND at all; stage 2 warm-starts from the result
        # and layers the hardware-claim conditions back on. 11 rounds of
        # discovery-under-full-hardening produced parked local optima.
        env_kw.update(domain_rand=False, payload_mass=0.0,
                      latency_ms=0.0, latency_ms_max=None,
                      latency_jitter_ms=0.0,
                      backlash_deg=0.0, backlash_deg_max=None,
                      push_kick=False, push_prob=0.0)
    if args.tilt_max is not None:
        env_kw["tilt_max_deg"] = args.tilt_max
    if args.act_lag is not None:
        lo, hi = (float(x) for x in args.act_lag.split(","))
        env_kw.update(act_lag_hz=lo, act_lag_hz_max=hi)
    if args.backlash_deg is not None:
        lo, hi = (float(x) for x in args.backlash_deg.split(","))
        env_kw.update(backlash_deg=lo, backlash_deg_max=hi)
    if args.zero_offset_deg is not None:
        env_kw["zero_offset_deg"] = float(args.zero_offset_deg)
    if args.gyro_gain_range is not None:
        env_kw["gyro_gain_range"] = tuple(
            float(x) for x in args.gyro_gain_range.split(","))
    if args.gyro_delay_max is not None:
        env_kw["gyro_delay_max"] = int(args.gyro_delay_max)
    if args.act_delay_max is not None:
        env_kw["act_delay_max"] = int(args.act_delay_max)
    if args.init_pose_deg is not None:
        env_kw["init_pose_deg"] = float(args.init_pose_deg)
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
    if args.w_pitch_hinge is not None:
        env_kw["w_pitch_hinge"] = args.w_pitch_hinge
    if args.pitch_deadband is not None:
        env_kw["pitch_deadband_deg"] = args.pitch_deadband
    if args.w_action_rate is not None:
        env_kw["w_action_rate"] = args.w_action_rate
    if args.w_symmetry is not None:
        env_kw["w_symmetry"] = args.w_symmetry
    if args.w_power is not None:
        env_kw["w_power"] = args.w_power
    if args.w_heading is not None:
        env_kw["w_heading"] = args.w_heading
    if args.turn_emph:
        env_kw["turn_emph"] = True
    if args.w_foot_under is not None:
        env_kw["w_foot_under"] = args.w_foot_under
    if args.w_up_vel is not None:
        env_kw["w_up_vel"] = args.w_up_vel
    if args.w_rise_ref is not None:
        env_kw["w_rise_ref"] = args.w_rise_ref
    if args.w_com_stance is not None:
        env_kw["w_com_stance"] = args.w_com_stance
    # knee-high marching (2026-08-01). Applied AFTER the family blocks so a
    # skills/loco recipe can be given the march slice without editing the
    # family mix; both land in config.json, which the CPU referee rebuilds
    # its env from.
    if args.march_mix is not None:
        env_kw["march_mix"] = args.march_mix
    if args.w_still is not None:
        env_kw["w_still"] = args.w_still
    if args.w_stand_com is not None:
        env_kw["w_stand_com"] = args.w_stand_com
    if args.w_hip_yaw is not None:
        env_kw["w_hip_yaw"] = args.w_hip_yaw
    if args.w_crouch_pull is not None:
        env_kw["w_crouch_pull"] = args.w_crouch_pull
    if args.crouch_rsi_mix is not None:
        env_kw["crouch_rsi_mix"] = args.crouch_rsi_mix
    if args.crouch_pose_ref:
        env_kw["crouch_pose_ref"] = True
    if args.crouch_theta_max_deg is not None:
        env_kw["crouch_theta_max_deg"] = args.crouch_theta_max_deg
    if args.w_crouch_track is not None:
        env_kw["w_crouch_track"] = args.w_crouch_track
    if args.crouch_track_sigma is not None:
        env_kw["crouch_track_sigma"] = args.crouch_track_sigma
    if args.crouch_release:
        env_kw["crouch_release"] = True
    if args.w_stand_zero is not None:
        env_kw["w_stand_zero"] = args.w_stand_zero
    if args.w_stand_home is not None:
        env_kw["w_stand_home"] = float(args.w_stand_home)
    if args.w_stand_knee is not None:
        env_kw["w_stand_knee"] = args.w_stand_knee
    if args.speed_clock:
        env_kw["speed_clock"] = True
    if args.speed_clock_hi is not None:
        env_kw["speed_clock_hi"] = args.speed_clock_hi
    if args.w_contact_sched is not None:
        env_kw["w_contact_sched"] = args.w_contact_sched
    if args.sched_duty is not None:
        env_kw["sched_duty"] = args.sched_duty
    if args.clock_freeze_stand:
        env_kw["clock_stand_freeze"] = True
    if args.mimic_crouch_gate:
        env_kw["mimic_crouch_gate"] = True
    if args.mimic_crouch_gate_knee:
        env_kw["mimic_crouch_gate_knee"] = True
    if args.ext_mix is not None:
        mix = tuple(float(x) for x in args.ext_mix.split(","))
        if len(mix) != 7 or sum(mix) > 1.0 + 1e-9:
            raise SystemExit(f"--ext-mix needs 7 floats summing <= 1, got {mix}")
        env_kw["ext_mix"] = mix
    if args.march_hz is not None:
        env_kw["march_hz"] = args.march_hz
    if args.w_knee_high is not None:
        env_kw["w_knee_high"] = args.w_knee_high
    for _k in ("w_foot_cross", "foot_cross_sep", "swing_height", "w_feet_phase",
               "feet_phase_s2"):
        if getattr(args, _k) is not None:
            env_kw[_k] = float(getattr(args, _k))
    # SIL-boundary realism (2026-07-31). Recorded unconditionally so
    # config.json says which side of the quantizer a run trained on -- the
    # CPU referee (eval_precision) reconstructs its env from these keys.
    env_kw["quantize_ticks"] = bool(args.quantize)
    env_kw["cmd_crouch_range"] = tuple(
        float(x) for x in args.cmd_crouch_range.split(","))
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
    wrapped = BatchedEnv(env, episode_length, num_envs=args.envs)

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
        # provenance: which committed tree trained this run (the 07-31
        # stale-plant incident took an md5 bisect to answer exactly this)
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            sha = subprocess.run(
                ["git", "rev-parse", "HEAD"], cwd=here, text=True,
                capture_output=True, timeout=10).stdout.strip() or None
            dirty = bool(subprocess.run(
                ["git", "status", "--porcelain", "--untracked-files=no"],
                cwd=here, text=True, capture_output=True,
                timeout=10).stdout.strip())
        except Exception:
            sha, dirty = None, None
        cfg.update(git_sha=sha, git_dirty=dirty)
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

    if args.w_mirror_loss:
        # Symmetry regularization by monkeypatch: brax's train() resolves
        # ppo_losses.compute_ppo_loss at call time, so wrapping the module
        # attribute is a no-fork hook. The aux term pushes the POLICY
        # FUNCTION toward left/right equivariance -- unlike the per-
        # touchdown w_symmetry reward, it supplies gradient on every
        # minibatch sample whether or not a foot lands. Mirrored obs are
        # normalized with the unmirrored normalizer stats; those stats are
        # near-symmetric and converge symmetric as the policy does, so the
        # approximation self-corrects. loc is PRE-tanh: tanh is odd and
        # elementwise, so mirroring commutes with the squash.
        import mirror as mirror_mod
        from brax.training.agents.ppo import losses as ppo_losses
        operm, osign = mirror_mod.obs_perm_signs(
            env.mj_model, ncmd=(7 if env.ext_cmd else 2),
            hist_len=env.obs_hist_len)
        aperm, asign = mirror_mod.action_perm_signs(env.mj_model)
        operm_j = jp.asarray(operm); osign_j = jp.asarray(osign)
        aperm_j = jp.asarray(aperm); asign_j = jp.asarray(asign)
        w_mirror = float(args.w_mirror_loss)
        _orig_ppo_loss = ppo_losses.compute_ppo_loss

        def _mirror_ppo_loss(params, normalizer_params, data, rng,
                             ppo_network, **kw):
            loss, metrics = _orig_ppo_loss(params, normalizer_params, data,
                                           rng, ppo_network=ppo_network, **kw)
            obs = data.observation
            apply = ppo_network.policy_network.apply
            logits = apply(normalizer_params, params.policy, obs)
            logits_m = apply(normalizer_params, params.policy,
                             obs[..., operm_j] * osign_j)
            na = logits.shape[-1] // 2          # (loc, raw_scale)
            loc, loc_m = logits[..., :na], logits_m[..., :na]
            sym = jp.mean((loc_m - loc[..., aperm_j] * asign_j) ** 2)
            metrics = dict(metrics)
            metrics["sym_loss"] = sym
            return loss + w_mirror * sym, metrics

        ppo_losses.compute_ppo_loss = _mirror_ppo_loss
        print(f"mirror-symmetry loss armed (w={w_mirror})")

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
            # plan-v2: Playground's proven biped sizes for precision runs
            policy_hidden_layer_sizes=((512, 256, 128) if args.precision
                                       else (128, 128)),
            value_hidden_layer_sizes=((512, 256, 128) if args.precision
                                      else (256, 256))),
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
