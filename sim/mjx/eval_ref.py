"""CPU-referee evaluation of an MJX/brax-trained policy.

THE rule of stage 2b: MJX training numbers are never trusted directly --
every policy is re-evaluated here, in the CPU BimoWalkerEnv (the engine every
prior result was validated in), under eval conditions that MATCH the claim
(GoPro payload, latency/backlash DR on, realizable IMU observations).

Scenarios (command-conditioned, 10 s episodes, N seeds each):
  walk    : vx=0.6            -- survive 10 s, track the speed
  slow    : vx=0.35           -- ditto
  stand   : vx=0, wz=0        -- survive, planar speed ~0, low power
  pivot_l : wz=+0.5           -- survive, turn left
  pivot_r : wz=-0.5           -- survive, turn right
  turn    : vx=0.4, wz=+0.4   -- walk a left arc

The plant, the observation width and the network shape all come from the
run's OWN config.json and checkpoint -- refereeing a policy on a different
robot than it was trained on scores nothing (issue #50). What the referee
does NOT inherit is the conditions it grades: servo action lag and the shove
model are pinned here, so runs trained over different DR ranges stay
comparable (the same rule eval_precision.py states).

Run:  .venv/bin/python sim/mjx/eval_ref.py --run mjx_cmd_v1 [--video]
"""
import argparse
import json
import inspect
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
RUNS = os.path.join(HERE, "..", "runs")
# Fallback only. The plant a run is refereed on comes from the run's OWN
# config.json ("xml_path"), because refereeing a policy on a different robot
# than it was trained on scores nothing: bimo_biped_v2.xml is the 8-DOF
# day-5 plant, and every policy since the v3yaw hip-yaw redesign has a 10-DOF
# action vector that will not even fit its actuators (issue #50).
DEFAULT_XML = os.path.join(HERE, "..", "bimo_biped_v5body.xml")

import jax
from brax.training.acme import running_statistics
from brax.training.agents.ppo import networks as ppo_networks

from walker_env import BimoWalkerEnv

_ENV_PARAMS = set(inspect.signature(BimoWalkerEnv.__init__).parameters)

SCENARIOS = [
    ("walk",    0.6,  0.0),
    ("slow",    0.35, 0.0),
    ("stand",   0.0,  0.0),
    ("pivot_l", 0.0,  0.5),
    ("pivot_r", 0.0, -0.5),
    ("turn",    0.4,  0.4),
]


def plant_xml(cfg):
    """The MJCF this run was trained on, resolved locally.

    train_mjx.py stores the basename (the run may have trained on another
    host), so an unqualified name resolves against sim/.
    """
    xml = cfg.get("xml_path") or DEFAULT_XML
    if not os.path.isabs(xml) and not os.path.exists(xml):
        xml = os.path.join(HERE, "..", os.path.basename(xml))
    return xml


def _hidden_sizes(net_params, fallback):
    """MLP hidden sizes from the checkpoint's own param shapes (brax names
    every layer hidden_i; the last is the output layer). --precision runs are
    (512, 256, 128), not the (128, 128) this used to hard-code."""
    try:
        layers = net_params["params"]
        ks = sorted((k for k in layers if k.startswith("hidden_")),
                    key=lambda k: int(k.split("_")[1]))
        return tuple(int(layers[k]["kernel"].shape[1]) for k in ks[:-1])
    except Exception:
        return fallback


def load_policy(run_dir, obs_size=38, act_size=8):
    """Rebuild the run's inference net. obs/act sizes come from the env the
    caller built from this run's config -- the 38/8 defaults are the 8-DOF
    day-5 shape, kept only for callers still refereeing those runs."""
    with open(os.path.join(run_dir, "params.pkl"), "rb") as f:
        params = pickle.load(f)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    net = ppo_networks.make_ppo_networks(
        observation_size=obs_size, action_size=act_size,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=_hidden_sizes(params[1], (128, 128)),
        value_hidden_layer_sizes=(_hidden_sizes(params[2], (256, 256))
                                  if len(params) > 2 else (256, 256)))
    make_policy = ppo_networks.make_inference_fn(net)
    policy = make_policy((params[0], params[1]), deterministic=True)
    policy = jax.jit(policy)
    rng = jax.random.PRNGKey(0)

    def act(obs):
        nonlocal rng
        rng, sub = jax.random.split(rng)
        a, _ = policy(obs[None].astype(np.float32), sub)
        return np.asarray(a[0])
    return act, cfg


def make_env(cfg, seed_payload=True, getup_start_mix=None, act_lag_hz=0.0):
    """CPU env matching the training conditions (the claim we referee).

    The run's own config seeds the construction, filtered to the env's real
    signature. That is not a nicety: obs width is a function of ext_cmd,
    obs_hist_len, gait_clock and friends, so a referee that ignores them
    builds an observation the policy was never trained to read. The explicit
    block below then pins the conditions the referee GRADES (payload, latency,
    backlash, DR) over whatever the run trained with.
    """
    kw = {k: v for k, v in cfg.items() if k in _ENV_PARAMS}
    # Training-time reset machinery must not leak into the scored episodes.
    if "recover_mix" in _ENV_PARAMS:
        kw["recover_mix"] = 0.0
    kw.update(dict(
        # PINNED, never inherited (same rule as eval_precision.py): the
        # servo action-chain lag is a property of the plant the referee
        # models, not of the distribution a run happened to train over.
        # Inheriting loco_v27's act_lag_hz=2.0 scores every scenario at the
        # worst corner of its own DR and makes runs trained on different
        # ranges incomparable.
        act_lag_hz=act_lag_hz,
        # Also pinned, same rule and the same history as eval_precision.py:
        # the graded shove is the HISTORICAL gentle 5 N force at 1%, not
        # whatever the run trained with. loco_v27 trains with velocity kicks
        # (push_kick=True); inheriting them scored stand at 0/2 with the
        # policy falling at ~4 s, against 8/8 from the precision referee.
        push_kick=False, push_force=5.0, push_prob=0.01,
        xml_path=plant_xml(cfg), actuator_model="sts3215",
        supply_voltage=cfg.get("supply_voltage", 11.1),
        command_mode=True, imu_obs=True,
        domain_rand=True,
        payload_mass=cfg.get("payload_mass", 0.154),
        latency_ms=cfg.get("latency_ms", 0.0),
        latency_ms_max=cfg.get("latency_ms_max", 8.0),
        latency_jitter_ms=cfg.get("latency_jitter_ms", 1.0),
        backlash_deg=cfg.get("backlash_deg", 0.5),
        backlash_deg_max=cfg.get("backlash_deg_max", 1.0),
        w_track_v=cfg.get("w_track_v", 2.0),
        w_track_w=cfg.get("w_track_w", 2.0),
        getup=cfg.get("getup", False),
        # The referee grades the CLAIM -- recovery from a ragdoll FALL -- so it
        # deliberately does NOT inherit the training run's getup_start_mix
        # (a reverse curriculum whose kneel/squat seeds start near the goal and
        # would inflate the recovered rate, and would make runs trained on
        # different mixes incomparable). --start-mix overrides for diagnostics.
        getup_start_mix=tuple(getup_start_mix or (1.0, 0.0, 0.0)),
        episode_seconds=cfg.get("episode_seconds", 10.0),
        action_map=cfg.get("action_map", "legacy"),
        hip_flex_deg=cfg.get("hip_flex_deg"),
        render_mode="rgb_array",
    ))
    return BimoWalkerEnv(**kw)


def rollout_getup(env, act, seed, record=False):
    """Fall-recovery episode: recovered = standing (tall + upright) held for
    a continuous second at any point; also reports time to first such hold."""
    obs, _ = env.reset(seed=seed)
    frames = []
    hold, t_stand, pw = 0, None, []
    for t in range(env.max_steps):
        a = act(obs)
        obs, r, term, trunc, info = env.step(a)
        pw.append(info["power_w"])
        hold = hold + 1 if info["standing"] > 0.5 else 0
        if hold >= 50 and t_stand is None:
            t_stand = (t + 1) * env.control_dt - 1.0   # start of the hold
        if record and t % 2 == 0:
            frames.append(env.render())
    return dict(recovered=t_stand is not None,
                t_stand=float("nan") if t_stand is None else t_stand,
                power_w=float(np.mean(pw)), frames=frames)


def _command(env, v, w):
    """(forward, yaw-rate) into whichever command layout this run uses.

    set_command()'s channel 1 is yaw rate on a legacy 2-wide env but SIDEWAYS
    VELOCITY on a 7-wide ext_cmd env. Passing (v, w) positionally to an ext
    env therefore asked every pivot/turn scenario for a sidestep, and scored
    it against a yaw rate nobody had commanded (pivot_l 0/4 before this).
    """
    if getattr(env, "ext_cmd", False):
        env.set_command(v, 0.0, w)      # vx, vy, wz (crouch defaults to 1)
    else:
        env.set_command(v, w)


def rollout(env, act, v, w, seed, record=False):
    obs, _ = env.reset(seed=seed)
    _command(env, v, w)
    obs = env._obs()          # refresh command channels post-override
    frames = []
    vxs, wzs, gyr, pw, planar = [], [], [], [], []
    t_alive = 0.0
    for t in range(env.max_steps):
        a = act(obs)
        obs, r, term, trunc, info = env.step(a)
        t_alive = (t + 1) * env.control_dt
        vxs.append(info.get("vx_body", 0.0))
        wzs.append(info.get("wz", 0.0))
        gyr.append(np.linalg.norm(env.data.qvel[3:5]))
        pw.append(info["power_w"])
        planar.append(float(np.hypot(env.data.qvel[0], env.data.qvel[1])))
        if record and t % 2 == 0:
            frames.append(env.render())
        if term:
            break
    n = max(len(vxs), 1)
    return dict(
        alive_s=t_alive,
        survived=t_alive >= 0.99 * env.max_steps * env.control_dt,
        vx_err=float(np.mean(np.abs(np.asarray(vxs) - v))),
        wz_err=float(np.mean(np.abs(np.asarray(wzs) - w))),
        wobble=float(np.sqrt(np.mean(np.asarray(gyr) ** 2))),
        power_w=float(np.mean(pw)),
        planar=float(np.mean(planar[-50:])) if planar else 0.0,
        frames=frames,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--video", action="store_true")
    p.add_argument("--act-lag-hz", type=float, default=0.0,
                   help="servo action-chain lag pole applied to the plant "
                        "(Hz; 2.0 = the 2026-08-31 bench measurement). "
                        "PINNED rather than inherited from the run's DR.")
    p.add_argument("--start-mix", default=None,
                   help="getup start-pose mix 'rag,kneel,squat' for DIAGNOSTIC "
                        "runs (e.g. 0,1,0 to probe the kneel seed). Default = "
                        "1,0,0: the graded claim is recovery from a fall.")
    args = p.parse_args()
    run_dir = os.path.join(RUNS, args.run)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    start_mix = (tuple(float(x) for x in args.start_mix.split(","))
                 if args.start_mix else None)
    # Env FIRST: it is the thing that knows how wide this run's observation
    # and action vectors are, and the net has to be built to match them.
    env = make_env(cfg, getup_start_mix=start_mix, act_lag_hz=args.act_lag_hz)
    act, cfg = load_policy(run_dir, env.observation_space.shape[0],
                           env.action_space.shape[0])
    print(f"plant {os.path.basename(plant_xml(cfg))}  "
          f"obs {env.observation_space.shape[0]}  "
          f"act {env.action_space.shape[0]}")

    if cfg.get("getup"):
        print(f"CPU referee (GET-UP): {args.run}  "
              f"({args.episodes} ragdoll-fall starts, {env.max_steps * env.control_dt:.0f} s "
              f"to recover; recovered = stand held 1 s)")
        rs = [rollout_getup(env, act, seed=100 * i + 7)
              for i in range(args.episodes)]
        n_rec = sum(r["recovered"] for r in rs)
        ts = [r["t_stand"] for r in rs if r["recovered"]]
        print(f"recovered {n_rec}/{args.episodes}"
              + (f"  median time-to-stand {np.median(ts):.2f}s" if ts else "")
              + f"  median watts {np.median([r['power_w'] for r in rs]):.1f}")
        if args.video:
            order = sorted(range(len(rs)),
                           key=lambda i: (not rs[i]["recovered"],
                                          rs[i]["t_stand"]))
            r = rollout_getup(env, act, seed=100 * order[0] + 7, record=True)
            import imageio
            path = os.path.join(run_dir, "ref_getup.mp4")
            imageio.mimsave(path, r["frames"], fps=25)
            print(f"video -> {path}  (best of {args.episodes} takes)")
        with open(os.path.join(run_dir, "referee.json"), "w") as f:
            json.dump({"getup": {"recovered": f"{n_rec}/{args.episodes}",
                                 "t_stand_median": float(np.median(ts)) if ts else None,
                                 "watts": float(np.median([r["power_w"] for r in rs]))}},
                      f, indent=2)
        return

    print(f"CPU referee: {args.run}  (payload {cfg.get('payload_mass', 0.154)*1000:.0f} g, "
          f"latency 0-8 ms, backlash 0.5-1.0 deg, IMU-noise DR, "
          f"{args.episodes} eps/scenario)")
    header = f"{'scenario':8s} {'survive':>8s} {'alive_s':>8s} {'vx_err':>7s} {'wz_err':>7s} {'wobble':>7s} {'watts':>6s}"
    print(header)
    all_rows = {}
    for name, v, w in SCENARIOS:
        rs = [rollout(env, act, v, w, seed=100 * i + 7) for i in range(args.episodes)]
        surv = sum(r["survived"] for r in rs)
        row = dict(
            survive=f"{surv}/{args.episodes}",
            alive_s=float(np.median([r["alive_s"] for r in rs])),
            vx_err=float(np.median([r["vx_err"] for r in rs])),
            wz_err=float(np.median([r["wz_err"] for r in rs])),
            wobble=float(np.median([r["wobble"] for r in rs])),
            power_w=float(np.median([r["power_w"] for r in rs])),
        )
        all_rows[name] = row
        print(f"{name:8s} {row['survive']:>8s} {row['alive_s']:8.2f} "
              f"{row['vx_err']:7.3f} {row['wz_err']:7.3f} "
              f"{row['wobble']:7.2f} {row['power_w']:6.1f}")
        if args.video:
            best = max(range(len(rs)), key=lambda i: rs[i]["alive_s"])
            r = rollout(env, act, v, w, seed=100 * best + 7, record=True)
            if r["frames"]:
                import imageio
                path = os.path.join(run_dir, f"ref_{name}.mp4")
                imageio.mimsave(path, r["frames"], fps=25)
                print(f"         video -> {path}  (seed take {best + 1}/{args.episodes})")

    with open(os.path.join(run_dir, "referee.json"), "w") as f:
        json.dump(all_rows, f, indent=2)
    print(f"\nsaved -> {os.path.join(run_dir, 'referee.json')}")


if __name__ == "__main__":
    main()
