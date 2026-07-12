"""Scenario evals for command-conditioned policies (command_mode runs).

Run:  .venv/bin/python sim/eval_commands.py --run-name cmd_11v1 [--episodes 8]
          [--payload 0.154] [--latency-ms 4] [--render]

Scenarios (each an episode with scripted set_command sequences):
  dash+stop : cmd (1.0, 0) until x >= 2 m, then (0, 0). Success = within 2 s
              of the switch, planar speed stays < 0.15 m/s for 1 s, upright
              through the end. This is the objective the dash_stop reward
              never delivered -- here stopping is just another command.
  turn      : cmd (0.5, +0.8) for 3 s then (0.5, -0.8) for 3 s. Reports the
              achieved yaw sweep vs commanded (integral of wz_cmd), success =
              >= 60 % of commanded sweep both ways and no fall.
  stand     : cmd (0, 0) for the whole episode. Success = no fall and total
              drift < 0.3 m.
With --render, writes <scenario>.gif/.mov of episode 0 into the run dir.
"""
import argparse
import os
import subprocess

import numpy as np
import imageio.v2 as imageio

from eval_policy import load, run_env_kwargs

HERE = os.path.dirname(os.path.abspath(__file__))


def base_env(env):
    return env.venv.envs[0] if hasattr(env, "venv") else env.envs[0]


def norm_cmd(env, cmd):
    """Normalize raw command values the way VecNormalize would (the obs we
    patch is already normalized -- writing raw values made a 'stand' command
    read as 'walk at the training-mean speed')."""
    if hasattr(env, "obs_rms"):
        m = env.obs_rms.mean[-2:]
        s = np.sqrt(env.obs_rms.var[-2:] + 1e-8)
        return np.clip((np.asarray(cmd) - m) / s, -10, 10)
    return np.asarray(cmd)


def rollout(model, env, script, max_s=10.0, render=False):
    """script(t, base) -> (v, w) command for time t; returns trace + frames."""
    base = base_env(env)
    obs = env.reset()
    trace, frames = [], []
    for i in range(int(max_s / base.control_dt)):
        t = i * base.control_dt
        v, w = script(t, base)
        base.set_command(v, w)
        obs[0, -2:] = norm_cmd(env, base._cmd)
        action, _ = model.predict(obs, deterministic=True)
        obs, _, dones, infos = env.step(action)
        trace.append(infos[0])
        if render:
            frames.append(base.render())
        if dones[0] and not infos[0].get("TimeLimit.truncated", False):
            break
    return trace, frames


def planar(info):
    return abs(info.get("vx_body", 0.0))  # body-frame fwd speed proxy


def scen_dash_stop(t, base):
    return (1.0, 0.0) if float(base.data.qpos[0]) < 2.0 else (0.0, 0.0)


def scen_turn(t, base):
    return (0.5, 0.8) if t < 3.0 else (0.5, -0.8)


def scen_stand(t, base):
    return (0.0, 0.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", required=True)
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--payload", type=float, default=None)
    p.add_argument("--latency-ms", type=float, default=None)
    p.add_argument("--render", action="store_true")
    args = p.parse_args()

    run_dir = os.path.join(HERE, "runs", args.run_name)
    kw = run_env_kwargs(run_dir, payload_mass=args.payload)
    if args.latency_ms is not None:
        kw.update(latency_ms=args.latency_ms, latency_ms_max=None,
                  latency_jitter_ms=0.0)
    model, env = load(run_dir,
                      render_mode="rgb_array" if args.render else None, **kw)
    print("scenario eval:", args.run_name, "| payload:", kw.get("payload_mass"),
          "| latency:", kw.get("latency_ms"))

    # -- dash + stop --------------------------------------------------------
    wins, t2m, tstop = 0, [], []
    for ep in range(args.episodes):
        trace, frames = rollout(model, env, scen_dash_stop,
                                render=args.render and ep == 0)
        fell = trace[-1]["up_z"] < 0.4 or trace[-1]["height"] < 0.18
        cross_i = next((i for i, s in enumerate(trace) if s["x"] >= 2.0), None)
        ok = False
        if cross_i is not None and not fell:
            t2m.append(cross_i * 0.02)
            still = 0
            for i in range(cross_i, len(trace)):
                still = still + 1 if abs(trace[i]["vx_body"]) < 0.15 else 0
                if still >= 50 and (i - cross_i) * 0.02 <= 3.0 + 1.0:
                    ok, stop_t = True, (i - 50 - cross_i) * 0.02
                    tstop.append(stop_t)
                    break
        wins += ok
        if args.render and ep == 0 and frames:
            save(frames, run_dir, "dash_stop")
    print(f"  dash+stop : {wins}/{args.episodes} stood still past 2 m"
          + (f"  t2m {np.median(t2m):.2f}s  brake {np.median(tstop):.2f}s"
             if tstop else ""))

    # -- turn ---------------------------------------------------------------
    wins, sweeps = 0, []
    for ep in range(args.episodes):
        trace, frames = rollout(model, env, scen_turn, max_s=6.0,
                                render=args.render and ep == 0)
        fell = trace[-1]["up_z"] < 0.4 or trace[-1]["height"] < 0.18
        half = min(150, len(trace))
        got_l = sum(s["wz"] for s in trace[:half]) * 0.02
        got_r = sum(s["wz"] for s in trace[half:]) * 0.02
        want_l = 0.8 * min(3.0, len(trace) * 0.02)
        want_r = -0.8 * max(0.0, len(trace) * 0.02 - 3.0)
        ok = (not fell and len(trace) * 0.02 > 5.5
              and got_l >= 0.6 * want_l and got_r <= 0.6 * want_r)
        wins += ok
        sweeps.append((np.degrees(got_l), np.degrees(got_r)))
        if args.render and ep == 0 and frames:
            save(frames, run_dir, "turn")
    m = np.median(np.array(sweeps), axis=0)
    print(f"  turn      : {wins}/{args.episodes} tracked both turns "
          f"(median sweep {m[0]:+.0f} deg then {m[1]:+.0f} deg; "
          f"commanded +137/-137)")

    # -- stand --------------------------------------------------------------
    wins, drifts = 0, []
    for ep in range(args.episodes):
        trace, _ = rollout(model, env, scen_stand, max_s=8.0)
        fell = trace[-1]["up_z"] < 0.4 or trace[-1]["height"] < 0.18
        drift = float(np.hypot(trace[-1]["x"], trace[-1].get("y", 0.0)))
        ok = not fell and len(trace) * 0.02 > 7.5 and drift < 0.3
        wins += ok
        drifts.append(drift)
    print(f"  stand     : {wins}/{args.episodes} stood 8 s "
          f"(median drift {np.median(drifts)*100:.0f} cm)")


def save(frames, run_dir, name):
    gif = os.path.join(run_dir, f"{name}.gif")
    imageio.mimwrite(gif, [f for f in frames if f is not None], fps=50, loop=0)
    try:
        import imageio_ffmpeg
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-i", gif,
                        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                        "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart",
                        gif.replace(".gif", ".mov")],
                       check=True, capture_output=True)
    except Exception as e:  # gif still useful without the mov
        print("  (mov conversion failed:", e, ")")
    print(f"  wrote {gif} (+.mov)")


if __name__ == "__main__":
    main()
