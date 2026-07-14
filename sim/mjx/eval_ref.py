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

Run:  .venv/bin/python sim/mjx/eval_ref.py --run mjx_cmd_v1 [--video]
"""
import argparse
import json
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
RUNS = os.path.join(HERE, "..", "runs")
XML = os.path.join(HERE, "..", "bimo_biped_v2.xml")

import jax
from brax.training.acme import running_statistics
from brax.training.agents.ppo import networks as ppo_networks

from walker_env import BimoWalkerEnv

SCENARIOS = [
    ("walk",    0.6,  0.0),
    ("slow",    0.35, 0.0),
    ("stand",   0.0,  0.0),
    ("pivot_l", 0.0,  0.5),
    ("pivot_r", 0.0, -0.5),
    ("turn",    0.4,  0.4),
]


def load_policy(run_dir):
    with open(os.path.join(run_dir, "params.pkl"), "rb") as f:
        params = pickle.load(f)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    net = ppo_networks.make_ppo_networks(
        observation_size=38, action_size=8,
        preprocess_observations_fn=running_statistics.normalize,
        policy_hidden_layer_sizes=(128, 128),
        value_hidden_layer_sizes=(256, 256))
    make_policy = ppo_networks.make_inference_fn(net)
    policy = make_policy((params[0], params[1]), deterministic=True)
    policy = jax.jit(policy)
    rng = jax.random.PRNGKey(0)

    def act(obs):
        a, _ = policy(obs[None].astype(np.float32), rng)
        return np.asarray(a[0])
    return act, cfg


def make_env(cfg, seed_payload=True):
    """CPU env matching the training conditions (the claim we referee)."""
    return BimoWalkerEnv(
        xml_path=XML, actuator_model="sts3215",
        supply_voltage=cfg.get("supply_voltage", 11.1),
        command_mode=True, imu_obs=True,
        domain_rand=True,
        payload_mass=cfg.get("payload_mass", 0.154),
        latency_ms=0.0, latency_ms_max=8.0, latency_jitter_ms=1.0,
        backlash_deg=0.5, backlash_deg_max=1.0,
        w_track_v=cfg.get("w_track_v", 2.0),
        w_track_w=cfg.get("w_track_w", 2.0),
        getup=cfg.get("getup", False),
        episode_seconds=cfg.get("episode_seconds", 10.0),
        action_map=cfg.get("action_map", "legacy"),
        hip_flex_deg=cfg.get("hip_flex_deg"),
        render_mode="rgb_array",
    )


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


def rollout(env, act, v, w, seed, record=False):
    obs, _ = env.reset(seed=seed)
    env.set_command(v, w)
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
    args = p.parse_args()
    run_dir = os.path.join(RUNS, args.run)
    act, cfg = load_policy(run_dir)
    env = make_env(cfg)

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

    print(f"CPU referee: {args.run}  (GoPro {cfg.get('payload_mass', 0.154)*1000:.0f} g, "
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
