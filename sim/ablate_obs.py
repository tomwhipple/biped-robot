"""Observation-ablation eval: which torso-state observations does a trained
policy actually need? Motivated by the sensing gap found 2026-07-11: the servo
encoders cover the 16 joint obs, an IMU covers up-vector + angular velocity,
but torso LINEAR velocity and terrain-relative HEIGHT have no direct sensor
on the real robot.

Run:  .venv/bin/python sim/ablate_obs.py --run-name dash_11v1_hardlat \
          --episodes 16 --payload 0.154 --latency-ms 4

Masking = zeroing the VecNormalize-normalized value, i.e. the policy sees that
observation frozen at its training-set mean -- the standard ablation. Obs
layout (walker_env._obs): [0:8] qpos, [8:16] qvel, [16:19] up-vector,
[19:22] linear vel, [22:25] angular vel, [25:33] prev action, [33] height,
[34:36] phase clock.
"""
import argparse
import numpy as np

from eval_policy import load, run_env_kwargs  # noqa: E402

MASKS = {
    "baseline (full state)":            [],
    "no linear velocity":               list(range(19, 22)),
    "no height":                        [33],
    "no linvel + no height (IMU-only)": list(range(19, 22)) + [33],
    "no IMU at all (also up+gyro)":     list(range(16, 25)) + [33],
}


def run(model, env, idx, episodes):
    wins, times = 0, []
    for _ in range(episodes):
        obs = env.reset()
        done = False
        last = {}
        while not done:
            if idx:
                obs[0, idx] = 0.0          # frozen at the training mean
            action, _ = model.predict(obs, deterministic=True)
            obs, _, dones, infos = env.step(action)
            last = infos[0]
            done = bool(dones[0])
        if last.get("dash_success", False):
            wins += 1
            times.append(last["time_to_2m"])
    med = float(np.median(times)) if times else float("nan")
    return wins, med


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--episodes", type=int, default=16)
    p.add_argument("--payload", type=float, default=None)
    p.add_argument("--latency-ms", type=float, default=None)
    args = p.parse_args()

    import os
    run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "runs", args.run_name)
    kw = run_env_kwargs(run_dir, payload_mass=args.payload)
    if args.latency_ms is not None:
        kw.update(latency_ms=args.latency_ms, latency_ms_max=None,
                  latency_jitter_ms=0.0)
    model, env = load(run_dir, **kw)
    print(f"{args.run_name}  ({args.episodes} eps each)")
    for name, idx in MASKS.items():
        wins, med = run(model, env, idx, args.episodes)
        print(f"  {name:36s} {wins:2d}/{args.episodes}  median t2m "
              f"{med:.2f}s" if wins else
              f"  {name:36s} {wins:2d}/{args.episodes}  --")
