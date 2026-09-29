"""Dynamic no-appendage get-up with the passive pelvis skid: the kinematic
finding (getup_v6_seated_feas.py) is that tucking to full hip flexion -125
parks the CoM 17 mm BEHIND the heel (the heel-to-buttock pull drags the heel
forward under the pelvis), while holding hip at -100..-90 keeps the CoM +33..
+69 mm OVER the feet. The record's flex130 skid study always tucked to -125
and therefore always failed. This sequence holds hip ~ -95 during the weight
transfer and rises.

Chain (all on the skid body, no tail/arms):
  lie -> sit up (upright) -> fold to hip -95, knee -130 (CoM over feet) ->
  butt-up: knees extend, hip stays ~-95 (CoM crosses over the feet) ->
  unfold torso -> stand.

Run: .venv/bin/python sim/getup_v6_skid_rise.py [--render out.mp4] [--skid-h ..]
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import dataclasses as dc
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from skid_alias import replace_skid_h
from static_gait import make_env, _write_video
import mujoco

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def build_seq(hip=0, kneef=-95, hold_hip=-95, butt_k=-15, unfold_hip=-20,
              ankle0=-20, t1=1.4, t2=2.4, t3=2.4):
    # Reach an UPRIGHT tall-squat (hip -95, knee near straight) with CoM over
    # the feet (seated_feas: hip -90..-100 keeps CoM +33..+52 mm over feet),
    # HOLD it, then EXTEND hip+knee together to stand while the torso (already
    # upright) stays balanced. No separate "unfold" -- the body is already
    # upright; we just lift.
    return [
        ("lie", {}, 0.4, 0.5),
        ("sit up (upright)", dict(hip_pitch=-90, knee=10, ankle=0), t1, 0.5),
        ("fold to hip -95 (CoM over feet)", dict(hip_pitch=hold_hip, knee=kneef, ankle=ankle0), t2, 1.0),
        ("butt-up -> upright tall-squat", dict(hip_pitch=hold_hip, knee=butt_k, ankle=-25), t2, 1.2),
        ("rise (extend hip+knee, lift)", dict(hip_pitch=-55, knee=-20, ankle=-25), t3, 1.0),
        ("stand tall", dict(hip_pitch=-25, knee=-5, ankle=-15), t3, 1.0),
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
            cam.distance, cam.elevation, cam.azimuth = (1.7, -16, 100)
        except Exception as e:
            print(f"[renderer unavailable: {e}]"); render = None
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
    p = replace_skid_h(DesignParams(), skid_h=a.skid_h, skid_x=-0.02, skid_len=0.08)
    xml = "/tmp/v6_skid_rise.xml"; open(xml, "w").write(build_xml(p))
    env = make_env(p, xml); obs, _ = env.reset(seed=0)
    kp, kd, stall_s, w0_s = env._servo; na = env._nq_act
    def servo():
        stall = np.full(na, float(stall_s)); w0 = np.full(na, float(w0_s))
        kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
        for i, n in enumerate(G.JN):
            if n in G.PJ_DEFAULT:
                s = G.SERVOS["sts3250"]; stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
        return np.array([kpa, kda, stall, w0], dtype=object)
    for i, (hold, butt_k) in enumerate(((-95, -15), (-95, -10), (-90, -15), (-100, -15))):
        env._servo = servo()
        run_one(p, env, build_seq(hold_hip=hold, butt_k=butt_k),
                f"hold{hold} butt{butt_k}", render=(a.render if i == 0 else None))

main()
