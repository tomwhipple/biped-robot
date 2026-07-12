"""Evaluate every trained run under sim/runs/ and print a comparison table.

Handy for the reward-shaping loop: shows at a glance which config actually walks
(high survival + forward distance) vs. which lunges (short episodes, big dist) or
stands (full length, ~0 dist). Deterministic policy, no rendering.

Run: python compare_runs.py [--episodes 10]
"""
from __future__ import annotations
import argparse
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from walker_env import BimoWalkerEnv
from eval_policy import run_env_kwargs

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


def eval_run(run_dir, episodes, dr=False, terrain=0.0, terrain_smoothness=0.15,
             fixed_payload=None, fixed_latency_ms=None, **overrides):
    # plant config (xml/actuator/dash) follows the run's env_config.json --
    # old runs on the old xml + ideal actuators, new runs on what they trained on
    env_kwargs = run_env_kwargs(run_dir, **overrides)
    if fixed_payload is not None:     # fixed-payload eval: kill any random draw
        env_kwargs["payload_mass"] = fixed_payload
        env_kwargs["payload_max"] = None
    if fixed_latency_ms is not None:  # fixed-latency eval: kill any random draw
        env_kwargs["latency_ms"] = fixed_latency_ms
        env_kwargs["latency_ms_max"] = None
        env_kwargs["latency_jitter_ms"] = 0.0
    env = DummyVecEnv([lambda: BimoWalkerEnv(
        domain_rand=dr, action_latency=1 if dr else 0,
        terrain_amplitude=terrain, terrain_smoothness=terrain_smoothness,
        **env_kwargs)])
    stats = os.path.join(run_dir, "vecnormalize.pkl")
    if os.path.exists(stats):
        env = VecNormalize.load(stats, env)
        env.training = False
        env.norm_reward = False
    model = PPO.load(os.path.join(run_dir, "model"), device="cpu")
    base = env.venv.envs[0] if isinstance(env, VecNormalize) else env.envs[0]
    max_steps = base.max_steps

    rets, dists, lens, surv = [], [], [], 0
    dsup, airs = 0, []
    dash_wins, dash_times = 0, []
    for _ in range(episodes):
        obs = env.reset()
        done, ret, steps, last = False, 0.0, 0, {}
        trunc = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, dones, infos = env.step(action)
            ret += float(reward[0]); last = infos[0]; steps += 1
            dsup += int(infos[0].get("double_support", False))
            ta = infos[0].get("touchdown_air", 0.0)
            if ta > 0:
                airs.append(ta)
            trunc = bool(infos[0].get("TimeLimit.truncated", False))
            done = bool(dones[0])
        rets.append(ret); dists.append(last.get("x", 0.0) * 1000); lens.append(steps)
        win = bool(last.get("dash_success", False))
        dash_wins += int(win)
        if win:
            dash_times.append(last["time_to_2m"])
        surv += int(trunc or steps >= max_steps or win)
    env.close()
    lens = np.array(lens); dists = np.array(dists)
    speed = (dists / 1000.0 / (lens * base.control_dt)).mean()
    return dict(ret=np.mean(rets), dist=dists.mean(), length=lens.mean(),
                surv=surv, episodes=episodes, speed=speed, max_steps=max_steps,
                dsup=dsup / max(int(lens.sum()), 1),
                swing=float(np.mean(airs)) if airs else 0.0,
                dash_wins=dash_wins,
                dash_med=float(np.median(dash_times)) if dash_times else float("nan"),
                dash_best=float(min(dash_times)) if dash_times else float("nan"))


def verdict(r):
    """Classify a policy from survival, episode length, distance, and speed.

    A lunge and a walk-that-falls both fail to survive, but differ in shape: a
    lunge is a single short high-speed dive (few steps, big dist); a real gait
    lasts many steps at a controlled speed before tipping.
    """
    frac = r["length"] / r["max_steps"]        # fraction of the 10 s episode survived
    survives = r["surv"] >= 0.8 * r["episodes"]
    if survives and r["dist"] > 200:
        return "WALKS robustly (survives full episode + moves)"
    if survives:
        return "stands (survives, ~no forward)"
    if frac > 0.3 and r["dist"] > 500 and r["speed"] < 0.6:
        return "walks then falls (gait, needs robustness)"
    if frac < 0.25 and r["dist"] > 300:
        return "lunges (short dive, then falls)"
    return "unstable"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--dr", action="store_true",
                   help="evaluate under domain randomization + shoves (robustness)")
    p.add_argument("--terrain-amplitude", type=float, default=0.0,
                   help="evaluate on procedural terrain with this bump height (m)")
    p.add_argument("--terrain-smoothness", type=float, default=0.15)
    p.add_argument("--runs", nargs="*", default=None,
                   help="only evaluate these run names (default: all)")
    p.add_argument("--dash", action="store_true",
                   help="evaluate every run in dash mode (2 m + 1 s upright) and "
                        "add success-rate / median / best time-to-2m columns")
    p.add_argument("--xml", default=None, help="override MJCF for all runs")
    p.add_argument("--actuator-model", default=None, choices=["ideal", "sts3215"],
                   help="override actuator model for all runs")
    p.add_argument("--voltage", type=float, default=None,
                   help="override servo supply voltage for all runs")
    p.add_argument("--payload", type=float, default=None,
                   help="evaluate all runs with this FIXED payload mass (kg)")
    p.add_argument("--latency-ms", type=float, default=None,
                   help="evaluate all runs with this FIXED sub-step latency (ms)")
    args = p.parse_args()
    xml = args.xml
    if xml and not os.path.isabs(xml):
        xml = os.path.join(os.path.dirname(os.path.abspath(__file__)), xml)
    overrides = dict(xml_path=xml, actuator_model=args.actuator_model,
                     supply_voltage=args.voltage, dash=True if args.dash else None)

    runs = sorted(d for d in os.listdir(RUNS)
                  if os.path.exists(os.path.join(RUNS, d, "model.zip"))) \
        if os.path.isdir(RUNS) else []
    if args.runs:
        runs = [r for r in runs if r in set(args.runs)]
    if not runs:
        raise SystemExit(f"No trained runs under {RUNS}")

    desc = "DOMAIN-RANDOMIZED + shoves" if args.dr else "nominal"
    if args.terrain_amplitude > 0:
        desc += f" + terrain {args.terrain_amplitude*1000:.0f}mm"
    else:
        desc += " (flat ground)"
    if args.dash:
        desc += " + dash objective"
    print(f"eval env: {desc}")
    dash_hdr = f" {'2m-ok':>6s} {'med-t':>6s} {'best':>6s}" if args.dash else ""
    print(f"{'run':16s} {'len/max':>10s} {'surv':>6s} {'dist(mm)':>9s} "
          f"{'m/s':>6s} {'dsup%':>6s} {'swing':>6s} {'return':>8s}{dash_hdr}  verdict")
    print("-" * (92 + (21 if args.dash else 0)))
    for name in runs:
        r = eval_run(os.path.join(RUNS, name), args.episodes, dr=args.dr,
                     terrain=args.terrain_amplitude,
                     terrain_smoothness=args.terrain_smoothness,
                     fixed_payload=args.payload,
                     fixed_latency_ms=args.latency_ms, **overrides)
        dash_col = (f" {r['dash_wins']:2d}/{r['episodes']:<3d} "
                    f"{r['dash_med']:5.2f}s {r['dash_best']:5.2f}s") if args.dash else ""
        print(f"{name:16s} {r['length']:5.0f}/{r['max_steps']:<4d} "
              f"{r['surv']:2d}/{r['episodes']:<3d} {r['dist']:9.0f} "
              f"{r['speed']:6.2f} {r['dsup']:6.0%} {r['swing']:5.2f}s "
              f"{r['ret']:8.1f}{dash_col}  {verdict(r)}")


if __name__ == "__main__":
    main()
