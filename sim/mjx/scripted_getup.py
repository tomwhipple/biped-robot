"""Hand-authored open-loop get-up: feasibility proof + imitation reference.

Plays a keyframed joint-target schedule through the CPU referee env (real
servo model: PD gains, torque ceiling, backlash off, no DR) from the sit and
fallen-prone starts. If a schedule stands the robot up, two things follow:
 1. the plant CAN get up within STS3215 torque limits (feasibility), and
 2. the recorded rollout (qpos trajectory) is a dynamically-consistent
    imitation reference for getup_v5 (track-the-demo + RSI along it).

Run:  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/scripted_getup.py
      [--start sit|fallen] [--render out.mp4] [--save-ref ref.npz]

Keyframes are (time_s, {joint: target_rad}) with linear interpolation
between them; unlisted joints hold the standing default. Symmetric L/R.
Tuning log lives in docs/precision-progress.md (night 4 prep).
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import walker_env  # noqa: E402


# ---------------------------------------------------------------- schedules
# Sit start: legs straight out front, torso up. Strategy (mirrors what the
# staged reference intended, now with the real dynamics in the loop):
# tuck heels under, rock forward onto the feet, deep-squat, then extend
# with the torso leading slightly forward so the CoM stays over the feet.
# Quasi-static rises fail from the sit: folding the knees lifts the feet
# into the air (telemetry: foot_z rose to 0.20 while the butt carried the
# weight) -- there is no arm to bridge the weight transfer. Human solution
# without hands = MOMENTUM ROCK: roll back onto the shoulders with the feet
# tucked in the air, then snap the hips to swing the legs down-and-under so
# the body rotates forward over the landing feet, catch in a deep squat,
# rise with the torso folded.
SIT_KEYS = [
    (0.0, dict(hip_pitch=-1.57, knee=-0.09, ankle=0.00)),   # hold the sit
    (0.5, dict(hip_pitch=-0.60, knee=-1.20, ankle=0.50)),   # roll back, tuck
    (1.0, dict(hip_pitch=-0.45, knee=-1.66, ankle=0.70)),   # on the back, tight
    (1.3, dict(hip_pitch=-1.92, knee=-1.66, ankle=0.70)),   # SNAP hips: legs
    (1.9, dict(hip_pitch=-1.92, knee=-1.60, ankle=0.62)),   # down + body pitches
    (2.9, dict(hip_pitch=-1.70, knee=-1.05, ankle=0.40)),   # catch -> extend,
    (3.9, dict(hip_pitch=-1.00, knee=-0.50, ankle=0.20)),   # torso stays folded
    (4.7, dict(hip_pitch=-0.35, knee=-0.20, ankle=0.08)),   # erect torso
    (5.3, dict(hip_pitch=None, knee=None, ankle=None)),     # stand (defaults)
]

# Kneel rise (shins on the ground, feet plantarflexed under the butt):
# fold the torso far forward to move the CoM over the knee->toe support
# line, roll the contact from instep to sole by dorsiflexing, then rise
# with the torso folded. Candidate quasi-static path.
KNEEL_KEYS = [
    (0.0, dict(hip_pitch=-0.20, knee=-1.62, ankle=-0.60)),  # hold the kneel
    (1.2, dict(hip_pitch=-1.92, knee=-1.62, ankle=-0.55)),  # torso folds fwd
    # hip stays PINNED at max fold through the whole knee extension: any
    # early torso rise moves the CoM behind the knee pivot and the
    # extension lifts the feet instead of the hips (telemetry, try 1)
    (2.6, dict(hip_pitch=-1.92, knee=-1.35, ankle=0.10)),   # hips rise over
    (3.8, dict(hip_pitch=-1.92, knee=-1.05, ankle=0.45)),   # knees, soles down
    (4.8, dict(hip_pitch=-1.92, knee=-0.80, ankle=0.60)),   # squat, feet flat
    (5.8, dict(hip_pitch=-1.30, knee=-0.55, ankle=0.30)),   # butt up more
    (6.6, dict(hip_pitch=-0.60, knee=-0.25, ankle=0.10)),   # erect torso
    (7.2, dict(hip_pitch=None, knee=None, ankle=None)),
]

# Fallen-prone recovery reuses the same rise once the feet are planted; the
# initial phase folds the legs under from wherever the ragdoll settled.
FALLEN_KEYS = [
    (0.0, dict()),
    (1.0, dict(hip_pitch=-1.85, knee=-1.60, ankle=0.60)),
    (2.0, dict(hip_pitch=-1.60, knee=-1.60, ankle=0.35)),
    (3.0, dict(hip_pitch=-1.15, knee=-1.25, ankle=0.20)),
    (4.2, dict(hip_pitch=-0.55, knee=-0.60, ankle=0.08)),
    (5.0, dict(hip_pitch=None, knee=None, ankle=None)),
]


def target_at(env, keys, t):
    """Interpolated full joint-target vector at time t (None -> default)."""
    def frame_vec(frame):
        out = env._default.copy()
        for name, val in frame.items():
            if val is None:
                continue
            for side in (env._legL, env._legR):
                out[side[name]] = val
        return out

    times = [k[0] for k in keys]
    if t <= times[0]:
        return frame_vec(keys[0][1])
    if t >= times[-1]:
        return frame_vec(keys[-1][1])
    i = int(np.searchsorted(times, t) - 1)
    t0, t1 = times[i], times[i + 1]
    a, b = frame_vec(keys[i][1]), frame_vec(keys[i + 1][1])
    w = (t - t0) / (t1 - t0)
    return a * (1 - w) + b * w


def run(start="sit", seed=0, render=None, save_ref=None, hold_s=2.0):
    env = walker_env.BimoWalkerEnv(
        xml_path=os.path.join(HERE, "..", "bimo_biped_v3yaw.xml"),
        ext_cmd=True, command_mode=True, recover_mix=1.0,
        recover_start_mix={"sit": (0.0, 0.0, 0.0, 1.0),
                           "kneel": (0.0, 1.0, 0.0, 0.0),
                           "fallen": (1.0, 0.0, 0.0, 0.0)}[start],
        domain_rand=False, imu_obs=False, action_latency=0,
        backlash_deg=0.0, render_mode=("rgb_array" if render else None))
    env.reset(seed=seed)
    keys = {"sit": SIT_KEYS, "kneel": KNEEL_KEYS,
            "fallen": FALLEN_KEYS}[start]
    total_s = keys[-1][0] + hold_s
    n = int(total_s / env.control_dt)
    frames, qpos_log, qvel_log = [], [], []
    for i in range(n):
        t = i * env.control_dt
        tgt = target_at(env, keys, t)
        # drive the position actuators directly (open loop, real PD + torque
        # ceiling still apply through the actuator model)
        env.data.ctrl[:] = tgt
        import mujoco
        for _ in range(env.frame_skip if hasattr(env, "frame_skip") else
                       int(env.control_dt / env.model.opt.timestep)):
            mujoco.mj_step(env.model, env.data)
        env._step_i += 1
        qpos_log.append(env.data.qpos.copy())
        qvel_log.append(env.data.qvel.copy())
        if i % int(0.4 / env.control_dt) == 0:
            import mujoco as _mj
            com = env.data.subtree_com[1]      # whole-robot CoM (root body 1)
            fl = env.data.geom("L_sole").xpos
            fr = env.data.geom("R_sole").xpos
            fx = 0.5 * (fl[0] + fr[0])
            print(f"  t={t:4.1f}  com_x-foot_x {com[0]-fx:+.3f}  "
                  f"com_z {com[2]:.3f}  foot_z {0.5*(fl[2]+fr[2]):.3f}")
        if render and i % 2 == 0:
            frames.append(env.render())
    h = float(env.data.qpos[2])
    q = env.data.qpos[3:7]
    up_z = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    stood = h > 0.85 * env._nominal_h and up_z > 0.9
    print(f"start={start} seed={seed}: final height {h:.3f} "
          f"(nominal {env._nominal_h:.3f}), up_z {up_z:.2f} -> "
          f"{'STOOD UP' if stood else 'failed'}")
    if render and frames:
        import imageio
        imageio.mimsave(render, frames, fps=int(0.5 / env.control_dt))
        print("video ->", render)
    if save_ref and stood:
        np.savez(save_ref, qpos=np.array(qpos_log),
                 control_dt=env.control_dt, start=start)
        print("reference ->", save_ref)
    return stood, np.array(qpos_log), np.array(qvel_log)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="sit",
                    choices=("sit", "kneel", "fallen"))
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--render", default=None)
    ap.add_argument("--save-ref", default=None)
    args = ap.parse_args()
    wins = sum(run(args.start, seed=s,
                   render=(args.render if s == 0 else None),
                   save_ref=(args.save_ref if s == 0 else None))[0]
               for s in range(args.seeds))
    print(f"{wins}/{args.seeds} seeds stood up")
