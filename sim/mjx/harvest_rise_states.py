"""Build the getup_v7 RSI state bank: a ladder of SETTLED, statically
stable ground poses along the feasible rise corridor, each tagged with its
rise-reference phase t0.

Corridor (open-loop study 2026-07-26, docs/precision-progress.md):
  ball/child's pose (t0=0) -> high-kneel (t0=rise/3, h 0.23, stable)
  -> snap rows (t0=rise/3..rise) -> stand.
The deep-squat path tips backward (CoM 9 cm behind the feet at max fold)
and is banked nowhere. Rows are settled holding their OWN pose (qvel ~ 0);
a stability gate drops anything that drifts. Ladder poses are jittered so
the bank is a corridor, not a point set.

Snap rows (getup_v8): v7 climbed ball -> high-kneel but never crossed the
last 5 cm -- training `recovered` stayed ~0 for 110M steps, so the value
function had no evidence standing pays. These rows are sampled along the
stand-drive snap from the settled high-kneel (ctrl=default), which climbs
h 0.225 -> 0.27+ WITH upward momentum (torque is sufficient; only the
final balance catch fails open-loop). Kept strictly below the recovered
threshold: RSI from here is 0.2-0.5 s from the goal, close enough for
exploration to finish the job and light up the income signal.

Run from sim/mjx:
  ../../.venv/bin/python harvest_rise_states.py --out ../getup_catch_states.npz
"""
import argparse
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import walker_env  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--out", default=os.path.join(HERE, "..",
                                             "getup_catch_states.npz"))
p.add_argument("--jitters", type=int, default=24)
p.add_argument("--rise-secs", type=float, default=4.0)
args = p.parse_args()

env = walker_env.BimoWalkerEnv(
    xml_path=os.path.join(HERE, "..", "bimo_biped_v3yaw.xml"),
    ext_cmd=True, command_mode=True, recover_mix=1.0,
    domain_rand=False, imu_obs=False, action_latency=0, backlash_deg=0.0)
env.reset(seed=0)
rng = np.random.default_rng(0)


def pose_vec(hip, knee, ankle, yaw=0.0, roll=0.0, side_only=None):
    """Symmetric leg pose; side_only='L'/'R' restricts hip/knee/ankle to one
    leg (the other keeps the previous value in j -- caller composes)."""
    j = env._default.copy()
    for name, side in (("L", env._legL), ("R", env._legR)):
        if side_only is not None and name != side_only:
            continue
        j[side["hip_pitch"]] = hip
        j[side["knee"]] = knee
        j[side["ankle"]] = ankle
        j[side["hip_yaw"]] = yaw
        j[side["hip_roll"]] = roll if name == "L" else -roll
    return j


def settle_row(qpos_init, ctrl_pose, hold_s=1.5):
    """Settle holding ctrl_pose; return (qpos, qvel) if stable else None."""
    env.data.qpos[:] = qpos_init
    env.data.qpos[env._jqpos] = np.clip(ctrl_pose, env._lo, env._hi)
    env.data.qvel[:] = 0.0
    env.data.ctrl[:] = np.clip(ctrl_pose, env._lo, env._hi)
    mujoco.mj_forward(env.model, env.data)
    n = int(hold_s / env.model.opt.timestep)
    for _ in range(n):
        mujoco.mj_step(env.model, env.data)
    if float(np.abs(env.data.qvel).mean()) > 0.10:
        return None
    return env.data.qpos.copy(), env.data.qvel.copy()


def qpos_base(h, pitch_deg=0.0):
    q = env.model.qpos0.copy()
    q[2] = h
    a = np.deg2rad(pitch_deg) / 2
    q[3:7] = [np.cos(a), 0, np.sin(a), 0]
    return q


rows, t0s, tags = [], [], []


# a settled row must still BE its pose: minimum pelvis height per rung
# (collapsed-to-lying heaps settle stably too and must not enter the bank)
H_MIN = {"ball": 0.07, "kneel": 0.19, "lunge": 0.13}


def bank(tag, t0, qpos_init, pose):
    r = settle_row(qpos_init, pose)
    if r is None or r[0][2] < H_MIN[tag]:
        return 0
    qp, qv = r
    rows.append((qp, qv))
    t0s.append(t0)
    tags.append(tag)
    return 1


def snap_rows(qpos_init, qvel_init, kneel_pose, h_cap, ramp_s=1.0,
              h_lo=0.15, per_s=0.05):
    """Ramp servo targets kneel -> default over ramp_s; sample every 50 ms
    while strictly below the recovered threshold, rising or holding only.
    Called twice per kneel jitter: from the SETTLED kneel (slow-rise band,
    h 0.225-0.24, small vz) and from the raw drop (whose impact bounce is
    a ballistic flight through h 0.21 at vz ~1.5 -- the exact mid-flight
    of the v6 settle trajectory whose endpoint the v6 policy proved
    holdable)."""
    pose = np.clip(kneel_pose, env._lo, env._hi)
    out, per = [], max(1, int(per_s / env.model.opt.timestep))
    env.data.qpos[:] = qpos_init
    env.data.qvel[:] = qvel_init
    env.data.ctrl[:] = pose
    mujoco.mj_forward(env.model, env.data)
    for i in range(int((ramp_s + 0.4) / env.model.opt.timestep)):
        w = min(i * env.model.opt.timestep / ramp_s, 1.0)
        env.data.ctrl[:] = pose * (1 - w) + env._default * w
        mujoco.mj_step(env.model, env.data)
        h = float(env.data.qpos[2])
        if h >= h_cap:
            break
        if i % per == 0 and env.data.qvel[2] > -0.05 and h > h_lo:
            out.append((env.data.qpos.copy(), env.data.qvel.copy(), h))
    return out


R = args.rise_secs
kept = {"ball": 0, "kneel": 0, "lunge": 0, "snap": 0}
for i in range(args.jitters):
    dj = rng.uniform(-0.12, 0.12, size=3)
    # ball / child's pose: reached from a prone fold; settle the folded pose
    # face-down (pitch 90) resting on shins+knees+forehead region
    ball = pose_vec(-1.92 + abs(dj[0]), -1.62 + abs(dj[1]),
                    -0.55 + dj[2] * 0.5)
    kept["ball"] += bank("ball", 0.0, qpos_base(0.09, 75 + 10 * dj[0]),
                         ball)
    # high-kneel: torso vertical on the knees (stable at h ~ 0.23)
    kneel = pose_vec(-0.20 + dj[0], -1.62 + abs(dj[1]) * 0.3,
                     -0.60 + dj[2] * 0.3,
                     yaw=dj[0] * 0.8, roll=dj[1] * 0.4)
    got = bank("kneel", R / 3.0, qpos_base(0.13), kneel)
    kept["kneel"] += got
    if got and i % 2 == 0:
        # finish rung: states along commanded rises of THIS jittered kneel
        # pose, phase-mapped by height across the last stretch
        h_cap = 0.85 * env._nominal_h - 0.004
        kq, kv = rows[-1]
        drop = qpos_base(0.13)
        drop[env._jqpos] = np.clip(kneel, env._lo, env._hi)
        # settled slow-rise band sampled at 50 ms; ballistic drop flight
        # sampled EVERY step (the 0.24-0.27 band lasts < 50 ms)
        starts = [(kq, kv, 0.20, 0.05),
                  (drop, np.zeros_like(kv), 0.21, 0.005)]
        for q0, v0, h_lo, per_s in starts:
            for qp, qv, h in snap_rows(q0, v0, kneel, h_cap, h_lo=h_lo,
                                       per_s=per_s):
                frac = np.clip((h - 0.15) / (h_cap - 0.15), 0.0, 1.0)
                rows.append((qp, qv))
                t0s.append(R / 3.0 + frac * (R * 2.0 / 3.0))
                tags.append("snap")
                kept["snap"] += 1
    # half-lunge: one foot planted ahead, other knee down (human no-hands
    # getup transfer pose); alternate the lead leg
    lead = "L" if i % 2 == 0 else "R"
    lunge = pose_vec(-0.20 + dj[0], -1.62, -0.60)     # kneeling leg
    lunge_lead = pose_vec(-1.60 + dj[0] * 0.5, -0.50 + dj[1],
                          0.40 + dj[2] * 0.5, side_only=lead)
    for side in (env._legL if lead == "L" else env._legR,):
        for nm in ("hip_pitch", "knee", "ankle"):
            lunge[side[nm]] = lunge_lead[side[nm]]
    kept["lunge"] += bank("lunge", R / 2.0, qpos_base(0.17, 25), lunge)

if not rows:
    sys.exit("no stable rows -- bank not written")
np.savez(args.out,
         qpos=np.array([r[0] for r in rows], dtype=np.float64),
         qvel=np.array([r[1] for r in rows], dtype=np.float64),
         t0=np.array(t0s, dtype=np.float64))
hs = [r[0][2] for r in rows]
print(f"bank -> {args.out}  ({len(rows)} rows: {kept}, "
      f"h {min(hs):.3f}..{max(hs):.3f})")
