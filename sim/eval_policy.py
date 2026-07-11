"""Evaluate a trained PPO policy on BimoWalkerEnv and (optionally) render a gif.

Loads the saved policy + VecNormalize stats, runs deterministic episodes, and
reports how far the robot walks vs. the standing baseline. Use --render to write
a gif of one episode (needs working GL: native on the Mac, MUJOCO_GL=egl/osmesa
on Linux).

Usage:
    python eval_policy.py                 # 10 episodes, numeric report
    python eval_policy.py --episodes 20
    python eval_policy.py --render        # also write runs/ppo_baseline/walk.gif
"""
from __future__ import annotations
import argparse
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from walker_env import BimoWalkerEnv

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


def load(run_dir, render_mode=None, terrain_amplitude=0.0, terrain_smoothness=0.15):
    env = DummyVecEnv([lambda: BimoWalkerEnv(
        render_mode=render_mode, terrain_amplitude=terrain_amplitude,
        terrain_smoothness=terrain_smoothness)])
    stats = os.path.join(run_dir, "vecnormalize.pkl")
    if os.path.exists(stats):
        env = VecNormalize.load(stats, env)
        env.training = False        # freeze running stats
        env.norm_reward = False     # report raw reward
    model = PPO.load(os.path.join(run_dir, "model"), device="cpu")
    return model, env


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--render", action="store_true", help="write a gif of episode 0")
    p.add_argument("--run-name", default="ppo_baseline", help="subdir under runs/")
    p.add_argument("--terrain-amplitude", type=float, default=0.0,
                   help="evaluate on procedural terrain with this bump height (m)")
    p.add_argument("--terrain-smoothness", type=float, default=0.15)
    p.add_argument("--gif-name", default="walk.gif",
                   help="output gif filename (under the run dir)")
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run_name)
    if not os.path.exists(os.path.join(run_dir, "model.zip")):
        raise SystemExit(f"No trained model at {run_dir}/model.zip -- run train_ppo.py first.")

    model, env = load(run_dir, render_mode="rgb_array" if args.render else None,
                      terrain_amplitude=args.terrain_amplitude,
                      terrain_smoothness=args.terrain_smoothness)
    base = env.venv.envs[0] if isinstance(env, VecNormalize) else env.envs[0]

    returns, distances, lengths, survived = [], [], [], []
    dsup_steps, total_steps, airs = 0, 0, []
    frames = []
    max_steps = base.max_steps
    for ep in range(args.episodes):
        obs = env.reset()
        done = False
        ep_ret, steps, last = 0.0, 0, {}
        trunc = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, dones, infos = env.step(action)
            ep_ret += float(reward[0])
            last = infos[0]
            steps += 1
            dsup_steps += int(infos[0].get("double_support", False))
            ta = infos[0].get("touchdown_air", 0.0)
            if ta > 0:
                airs.append(ta)
            # SB3 VecEnv auto-resets on done; the true terminal info is stashed.
            trunc = bool(infos[0].get("TimeLimit.truncated", False))
            done = bool(dones[0])
            if args.render and ep == 0:
                frames.append(base.render())
        returns.append(ep_ret)
        distances.append(last.get("x", 0.0) * 1000)   # mm
        lengths.append(steps)
        total_steps += steps
        survived.append(trunc or steps >= max_steps)

    returns, distances, lengths = np.array(returns), np.array(distances), np.array(lengths)
    speed = distances / 1000.0 / (lengths * base.control_dt)   # m/s per episode
    print(f"episodes      : {args.episodes}")
    print(f"return        : {returns.mean():8.2f} +/- {returns.std():.2f}")
    print(f"forward dist  : {distances.mean():8.0f} +/- {distances.std():.0f} mm")
    print(f"episode length: {lengths.mean():8.1f} / {max_steps} steps "
          f"({lengths.mean()*base.control_dt:.1f}s)")
    print(f"survived full : {sum(survived)}/{args.episodes}  (reached 10s truncation)")
    print(f"avg fwd speed : {speed.mean():8.2f} m/s")
    # gait quality: double-support fraction ~1.0 => shuffle (feet never lift);
    # a clean alternating stride is ~0.2-0.5 with swing times near 0.2-0.3 s.
    print(f"double-support: {dsup_steps / max(total_steps, 1):8.0%} of steps")
    print(f"swing time    : {np.mean(airs) if airs else 0.0:8.2f} s mean "
          f"({len(airs)} touchdowns)")
    print(f"(standing baseline: length {max_steps}, dist ~0mm. Short length + big "
          f"dist => lunge/fall, not a gait.)")

    if args.render and frames:
        import imageio.v2 as imageio
        out = os.path.join(run_dir, args.gif_name)
        imageio.mimsave(out, frames, fps=50)
        print(f"rendered {len(frames)} frames -> {out}")

    env.close()


if __name__ == "__main__":
    main()
