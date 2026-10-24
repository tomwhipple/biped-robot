"""Roll-compliance bench test in the CPU sim (hw 2026-10-24, legs-only body).

The bench: one foot clamped to the desk, the other lifted 30 mm straight up
with hip pitch / knee / ankle only (joint-space targets, no weight shift).
Measured: the torso tilts toward the lifted side by 5.2 deg (right foot
lifted) / 5.0 deg (left foot lifted), steady -- but the stance encoders move
only ~1 deg (ankle roll 12-17 ticks = 1.0-1.5 deg at 10-14 % load, hip roll
8-10 ticks). The rest is play downstream of the encoders.

Here: the stance foot is welded to the world (the clamp), the lifted foot's
body is solved by IK to rise 30 mm with its orientation and x/y unchanged,
the joint targets ramp over 1 s and hold for 3 s. Reported per lifted side:
the steady torso tilt toward the lifted side (true attitude) and the stance
roll encoders' error (target - shaft, i.e. what an encoder on the servo side
of the play would read). Run with several extra roll deadzone widths to
calibrate the play (`--roll-lash-deg`), e.g.

    .venv/bin/python sim/roll_compliance.py --roll-lash-deg 0,1,2,3,4,5

The servo set is the robot's (walker_env servo_kp_scale 'robot': STS3250 hip
rolls, ankle roll 2.8x, knee 2.8x), joint friction 0.35 N*m (the measured
level), no latency, no DR. docs/design-v6/roll-compliance-2026-10-24.md has
the numbers.
"""
import argparse
import os
import re
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "mjx"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

import robot_plant  # noqa: E402
import walker_env as W  # noqa: E402

HW = {"R": dict(tilt=5.2, ankle_enc=(1.0, 1.5)), "L": dict(tilt=5.0, ankle_enc=(1.0, 1.5))}


def clamped_xml(xml, stance):
    """The plant with the stance foot welded to the world (the desk clamp)."""
    src = open(robot_plant.robot_xml_path(xml)).read()
    src = re.sub(r'meshdir="([^"]+)"', lambda m: 'meshdir="' + os.path.normpath(os.path.join(
        os.path.dirname(robot_plant.robot_xml_path(xml)), m.group(1))) + '"', src, count=1)
    eq = f'  <equality>\n    <weld body1="{stance}_foot"/>\n  </equality>\n'
    src = src.replace("</mujoco>", eq + "</mujoco>")
    fd, path = tempfile.mkstemp(suffix=".xml")
    os.write(fd, src.encode())
    os.close(fd)
    return path


def make_env(xml, stance, roll_lash_deg=0.0, play_deg=0.0, friction=0.35, **over):
    kw = dict(supply_voltage=11.1, action_map="full", quantize_ticks=False, domain_rand=False,
              latency_ms=0.0, backlash_deg=0.0, push_prob=0.0, push_force=0.0,
              joint_frictionloss=friction, joint_armature=0.028, servo_kp_scale="robot",
              roll_lash_deg=roll_lash_deg, ext_cmd=True, imu_obs=False)
    kw.update(robot_plant.robot_env_kwargs(xml))
    kw["servo_kp_scale"] = "robot"
    kw["xml_path"] = clamped_xml(xml, stance)
    if play_deg:
        kw.update(play_deg=play_deg, play_joints=tuple(
            f"{s}_{j}" for s in "LR" for j in ("hip_roll", "ankle_roll")))
    kw.update(over)
    return W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, **kw)


def lift_targets(env, side, dz=0.030):
    """Joint targets (hip_pitch, knee, ankle) of `side` that raise its foot body dz
    with orientation and x/y unchanged (IK on the plant itself, torso frame)."""
    from scipy.optimize import least_squares
    m, d = env.model, mujoco.MjData(env.model)
    d.qpos[:] = m.qpos0
    d.qpos[:7] = [0, 0, 1, 1, 0, 0, 0]
    names = [f"{side}_hip_pitch", f"{side}_knee", f"{side}_ankle"]
    adr = [m.joint(n).qposadr[0] for n in names]
    bid = m.body(f"{side}_foot").id
    mujoco.mj_forward(m, d)
    p0, R0 = d.xpos[bid].copy(), d.xmat[bid].reshape(3, 3).copy()
    q0 = np.array([d.qpos[a] for a in adr])

    def res(q):
        for a, v in zip(adr, q):
            d.qpos[a] = v
        mujoco.mj_kinematics(m, d)
        dp = d.xpos[bid] - (p0 + [0, 0, dz])
        dR = d.xmat[bid].reshape(3, 3) @ R0.T
        return np.concatenate([dp * 1e3, [dR[0, 2], dR[1, 2], dR[0, 1]]])

    # straight legs are a singularity (dz ~ knee^2): start from a bent guess, either hip sign,
    # and keep the solution with the smallest joint excursion
    best = None
    for h in (-0.5, 0.5):
        sol = least_squares(res, q0 + np.array([h, -1.0 * np.sign(h) * abs(h) * 2, 0.0]))
        if np.abs(sol.fun).max() < 1e-3 and (best is None or np.abs(sol.x - q0).sum() < np.abs(best.x - q0).sum()):
            best = sol
    assert best is not None, "no IK solution"
    sol = best
    return dict(zip(names, sol.x)), dict(zip(names, q0))


def run(env, side, ramp=1.0, hold=3.0, settle=1.0):
    """One lift. Returns (tilt toward the lifted side, deg; stance encoder error dict, deg)."""
    stance = "R" if side == "L" else "L"
    env.reset(seed=0)
    qlift, q0 = lift_targets(env, side)
    names = env._act_names
    base = env._default.copy()
    tgt = base.copy()
    n_ctrl = int((settle + ramp + hold) / env.control_dt)
    tilts, enc, lifts = [], [], []
    bl, bs = env.model.body(f"{side}_foot").id, env.model.body(f"{stance}_foot").id
    sgn = 1.0 if side == "L" else -1.0
    for k in range(n_ctrl):
        t = k * env.control_dt
        s = np.clip((t - settle) / ramp, 0.0, 1.0)
        for n, v in qlift.items():
            tgt[names.index(n)] = base[names.index(n)] + s * (v - q0[n])
        env._action_to_ctrl = lambda a, tgt=tgt.copy(): tgt
        env.step(np.zeros(env.action_size if hasattr(env, "action_size") else 12, np.float32))
        if t > settle + ramp + hold - 1.0:
            up = env.data.sensordata[env._up_id:env._up_id + 3]
            tilts.append(np.degrees(np.arctan2(sgn * up[1], up[2])))
            lifts.append(1e3 * (env.data.xpos[bl][2] - env.data.xpos[bs][2]))
            # the encoder: servo-side of the play. target - link - (deadzone half-width at the edge)
            err = {}
            for j in ("hip_roll", "ankle_roll"):
                n = f"{stance}_{j}"
                i = env._sname2i[n]
                q = env.data.qpos[env._jq0 + i]
                tgt_i = env.data.ctrl[i]
                half = 0.5 * (env._lash_rad + env._roll_lash_rad) + (env._play_h[i] if env._play_rad else 0.0)
                e = abs(tgt_i - q) - half
                err[j] = np.degrees(max(e, 0.0))
            enc.append(err)
    return float(np.mean(tilts)), {j: float(np.mean([e[j] for e in enc])) for j in enc[0]}, float(np.mean(lifts))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default=robot_plant.ROBOT_LEGS_XML)
    ap.add_argument("--roll-lash-deg", default="0")
    ap.add_argument("--play-deg", default="0", help="the walker_env shaft/link play model instead (same widths)")
    ap.add_argument("--friction", type=float, default=0.35)
    ap.add_argument("--backlash-deg", type=float, default=0.0, help="global gear backlash on every joint (the referee pins 0.7)")
    a = ap.parse_args(argv)
    print(f"hw: tilt toward lifted side R-lifted {HW['R']['tilt']} / L-lifted {HW['L']['tilt']} deg; "
          f"stance ankle-roll encoder 1.0-1.5 deg, hip roll 0.7-0.9 deg")
    out = []
    for w in [float(x) for x in a.roll_lash_deg.split(",")]:
        for pw in [float(x) for x in a.play_deg.split(",")]:
            row = {}
            for side in ("L", "R"):
                env = make_env(a.xml, "R" if side == "L" else "L", roll_lash_deg=w, play_deg=pw,
                               friction=a.friction, backlash_deg=a.backlash_deg)
                row[side] = run(env, side)
                os.unlink(env.xml_path) if hasattr(env, "xml_path") and os.path.exists(env.xml_path) else None
            line = (f"roll_lash {w:4.1f}  play {pw:4.1f} | L-lifted tilt {row['L'][0]:5.2f} "
                    f"(enc ankle {row['L'][1]['ankle_roll']:4.2f} hip {row['L'][1]['hip_roll']:4.2f}) | "
                    f"R-lifted tilt {row['R'][0]:5.2f} "
                    f"(enc ankle {row['R'][1]['ankle_roll']:4.2f} hip {row['R'][1]['hip_roll']:4.2f}) | lift {row['L'][2]:.1f}/{row['R'][2]:.1f} mm")
            print(line, flush=True)
            out.append(line)
    return out


if __name__ == "__main__":
    main()
