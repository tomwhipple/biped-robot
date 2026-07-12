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
import json
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from walker_env import BimoWalkerEnv

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")

# env_config.json keys that describe the *plant* (model + actuator + objective)
# rather than reward shaping -- these must carry over into the eval env so each
# run is evaluated on the physics it was trained with. Reward weights stay at
# env defaults, as before (returns across runs are then comparable).
_PLANT_KEYS = ("xml_path", "actuator_model", "supply_voltage",
               "servo_kp", "servo_kd", "payload_mass", "payload_max",
               "dash", "dash_distance", "dash_hold", "dash_stop",
               "stand_speed", "imu_obs", "imu_noise", "command_mode",
               "backlash_deg", "backlash_deg_max", "fall_height", "fall_up_z",
               "cmd_stand_prob", "w_track_v", "w_track_w")


def run_env_kwargs(run_dir, **overrides):
    """Eval-env kwargs for a run: its saved plant config (env_config.json,
    written by train_ppo) overlaid with any explicit CLI overrides. Old runs
    have no config file -> original defaults (old xml, ideal actuators)."""
    kw = {}
    cfg_path = os.path.join(run_dir, "env_config.json")
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            cfg = json.load(f)
        kw = {k: cfg[k] for k in _PLANT_KEYS if k in cfg}
    kw.update({k: v for k, v in overrides.items() if v is not None})
    return kw


def load(run_dir, render_mode=None, **env_kwargs):
    env = DummyVecEnv([lambda: BimoWalkerEnv(render_mode=render_mode, **env_kwargs)])
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
    p.add_argument("--xml", default=None,
                   help="override the MJCF file (default: the run's env_config)")
    p.add_argument("--actuator-model", default=None, choices=["ideal", "sts3215"],
                   help="override the actuator model (default: the run's env_config)")
    p.add_argument("--voltage", type=float, default=None,
                   help="override servo supply voltage (default: the run's env_config)")
    p.add_argument("--dash", action="store_true",
                   help="evaluate in dash mode (2 m + 1 s upright finish) and "
                        "report dash metrics, even for a non-dash-trained run")
    p.add_argument("--payload", type=float, default=None,
                   help="evaluate with this FIXED torso-top payload mass (kg), "
                        "overriding the run's payload config (e.g. 0 or 0.154)")
    p.add_argument("--latency-ms", type=float, default=None,
                   help="evaluate with this FIXED sub-step action latency (ms), "
                        "overriding any latency DR in the run's config")
    p.add_argument("--gif-name", default="walk.gif",
                   help="output gif filename (under the run dir)")
    args = p.parse_args()

    run_dir = os.path.join(RUNS, args.run_name)
    if not os.path.exists(os.path.join(run_dir, "model.zip")):
        raise SystemExit(f"No trained model at {run_dir}/model.zip -- run train_ppo.py first.")

    xml = args.xml
    if xml and not os.path.isabs(xml):
        xml = os.path.join(os.path.dirname(os.path.abspath(__file__)), xml)
    env_kwargs = run_env_kwargs(
        run_dir, xml_path=xml, actuator_model=args.actuator_model,
        supply_voltage=args.voltage, dash=True if args.dash else None)
    if args.payload is not None:      # fixed-payload eval: kill any random draw
        env_kwargs["payload_mass"] = args.payload
        env_kwargs["payload_max"] = None
    if args.latency_ms is not None:   # fixed-latency eval: kill any random draw
        env_kwargs["latency_ms"] = args.latency_ms
        env_kwargs["latency_ms_max"] = None
        env_kwargs["latency_jitter_ms"] = 0.0
    print("eval env:", env_kwargs or "(original defaults)")
    model, env = load(run_dir, render_mode="rgb_array" if args.render else None,
                      terrain_amplitude=args.terrain_amplitude,
                      terrain_smoothness=args.terrain_smoothness, **env_kwargs)
    base = env.venv.envs[0] if isinstance(env, VecNormalize) else env.envs[0]

    returns, distances, lengths, survived = [], [], [], []
    dsup_steps, total_steps, airs = 0, 0, []
    dash_wins, dash_times = 0, []
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
        if last.get("dash_success", False):
            dash_wins += 1
            dash_times.append(last["time_to_2m"])
        survived.append(trunc or steps >= max_steps
                        or last.get("dash_success", False))

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
    if getattr(base, "dash", False):
        med = np.median(dash_times) if dash_times else float("nan")
        best = min(dash_times) if dash_times else float("nan")
        print(f"dash (2 m + upright {base.dash_hold:.0f}s): "
              f"{dash_wins}/{args.episodes} confirmed finishes, "
              f"time-to-2m median {med:.2f}s  best {best:.2f}s")
    print(f"(standing baseline: length {max_steps}, dist ~0mm. Short length + big "
          f"dist => lunge/fall, not a gait.)")

    if args.render and frames:
        import imageio.v2 as imageio
        out = os.path.join(run_dir, args.gif_name)
        imageio.mimsave(out, frames, fps=50, loop=0)   # loop=0 -> loop forever
        print(f"rendered {len(frames)} frames -> {out}")
        # also emit a QuickTime-friendly .mov (macOS Preview doesn't animate gifs)
        import subprocess
        import sys
        gif2mov = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "tools", "gif2mov.py")
        try:
            subprocess.run([sys.executable, gif2mov, out], check=True)
        except (OSError, subprocess.CalledProcessError) as e:
            print(f"(gif2mov failed: {e} -- gif is still fine)")

    env.close()


if __name__ == "__main__":
    main()
