"""Final no-appendage rise probe: from the skid-seated tall-squat the CoM is
+33..+52 mm OVER the feet (hip -90/-100 verified in getup_v6_seated_feas.py),
but extending the hips from a foot anchor tips the torso back (the wall). The
remaining lever is ROLLING the pelvis off the skid FORWARD onto the feet: rock
the weight from the skid to the toes (ankle dorsiflex + slight hip extension),
which lifts the pelvis free of the skid onto the feet, then stand. This is a
rock-forward off a raised seat -- the classic chair rise -- using the passive
skid as the chair and the deep-ankle ROM (+-45) to transfer weight onto the toes
first (beyond-human ankle reach).

Phases (skid body, no tail/arms):
  lie -> sit up -> fold to hip -95 / knee -130 (CoM over feet, pelvis on skid)
  -> rock forward: ankle to toes + hip to -70 (weight slides off skid onto toes)
  -> lift pelvis off skid (hip -50, knee -50) as the toes push
  -> stand.

Run: .venv/bin/python sim/getup_v6_rockrise.py [--render out.mp4] [--skid-h ..]
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import dataclasses as dc
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from static_gait import make_env, _write_video
import mujoco

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def build_seq(hold=-95, ktuck=-130, rock_h=-70, rock_a=-40, lift_h=-50, lift_k=-50,
              t1=1.4, t2=2.0, t3=2.0, t4=2.2):
    # FOLD FIRST, then TUCK (getup_study.py pitfall): with legs STRAIGHT as the
    # counterweight, fold the torso to hip -95 (CoM comes forward over the feet;
    # weight stays on the skid seat); THEN bend the knees to bring the feet
    # under the hips; THEN rock forward onto the toes and rise.
    return [
        ("lie", {}, 0.4, 0.5),
        ("sit up (upright)", dict(hip_pitch=-90, knee=10, ankle=0), t1, 0.5),
        ("fold torso to hip -95 (legs straight)", dict(hip_pitch=hold, knee=10, ankle=0), t2, 1.0),
        ("tuck feet under (knee -130)", dict(hip_pitch=hold, knee=ktuck, ankle=-20), t2, 1.0),
        ("rock fwd onto toes", dict(hip_pitch=rock_h, knee=ktuck, ankle=rock_a), t3, 1.2),
        ("lift pelvis off skid", dict(hip_pitch=lift_h, knee=lift_k, ankle=-30), t3, 1.2),
        ("stand tall", dict(hip_pitch=-25, knee=-5, ankle=-15), t4, 1.0),
        ("straight", {}, 1.0, 0.5),
    ]

def run_one(p, env, seq, name, render=None):
    env.fall_up_z = -2; env.fall_height = -1
    m, d = env.model, env.data
    d0, hi, lo = env._default, env._hi, env._lo
    na = env._nq_act
    def inv(q):
        q = np.concatenate([q, np.zeros(na - len(q))]) if len(q) < na else q[:na]
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    qprev = qq(seq[0][1]); G.settle_fallen(env, p, "supine", qprev)
    dt = env.control_dt; frames = []; t = 0.0; k = 0
    if render:
        try:
            rnd = mujoco.Renderer(m, 540, 720); cam = mujoco.MjvCamera()
            cam.distance, cam.elevation, cam.azimuth = (1.6, -14, 100)
        except Exception as e:
            print(f"[renderer unavailable]"); render = None
    from getup_v6 import contacts_summary
    peak = -1; max_tau = 0.0
    for label, off, move_s, hold_s in seq:
        qt = qq(off); nstep = int(move_s / dt)
        for i in range(nstep + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(nstep, 1)); s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = qprev + (qt - qprev) * s
            env.step(inv(q)); t += dt; k += 1
            up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]; peak = max(peak, up)
            max_tau = max(max_tau, float(np.abs(env._servo_tau).max()))
            if render and k % 2 == 0:
                cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.2]; rnd.update_scene(d, cam); frames.append(rnd.render().copy())
        qprev = qt
        print(f"  {label:30s} up={up:+.2f} pelZ={d.qpos[2]:.3f} | {contacts_summary(m, d)}")
    up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
    ok = up > 0.9 and d.qpos[2] > 0.8 * p.z_yaw_above_sole
    print(f"[{name}] peak_up {peak:+.2f} max_tau {max_tau:.2f} -> {'STANDING' if ok else 'no'} (up {up:+.2f} z {d.qpos[2]:.3f})")
    if render: _write_video(frames, render, fps=25)
    return ok

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", default=None); ap.add_argument("--skid-h", type=float, default=0.08)
    a = ap.parse_args()
    p = dc.replace(DesignParams(), skid=True, skid_h=a.skid_h, skid_x=-0.02, skid_len=0.08)
    xml = "/tmp/v6_rockrise.xml"; open(xml, "w").write(build_xml(p))
    env = make_env(p, xml); obs, _ = env.reset(seed=0)
    kp, kd, stall_s, w0_s = env._servo; na = env._nq_act
    def servo():
        stall = np.full(na, float(stall_s)); w0 = np.full(na, float(w0_s))
        kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
        for i, n in enumerate(G.JN):
            if n in G.PJ_DEFAULT:
                s = G.SERVOS["sts3250"]; stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
        return np.array([kpa, kda, stall, w0], dtype=object)
    for i, (rock_h, rock_a) in enumerate(((-70, -40), (-60, -45), (-80, -35), (-75, -40))):
        env._servo = servo()
        run_one(p, env, build_seq(rock_h=rock_h, rock_a=rock_a),
                f"rock{rock_h}/a{rock_a}", render=(a.render if i == 0 else None))

main()
