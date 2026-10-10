"""Record the open-loop seat-push get-up (elbow arms; shoulder 60->0, elbow -60->0, ankle -25,
push 2.0 s) on the committed plant as a tracking reference: per control step the commanded
servo targets for all actuators (radians, actuator order), the full qpos/qvel, and time."""
import os, sys, numpy as np
ROOT = "/home/claw/code/robot"; sys.path.insert(0, os.path.join(ROOT, "sim")); os.chdir(os.path.join(ROOT, "sim"))
import getup_v6_shoulder as S, getup_v6 as G
OUT = sys.argv[1]   # e.g. sim/getup_ref_v6ar.npz (train_mjx --rise-ref getup_ref_v6ar.npz)
XML = os.path.join(ROOT, "sim", "bimo_biped_v6ar.xml")
P, _ = S.plant("r6_asdrawn_rom120")
seq = S.seat_push(False, 60, 0, 2.0, -25.0, elbow=(-60, 0))
env = G.make_env(P, XML, mu=0.7, play_deg=3.0)
env.reset(seed=0)
G.EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
kp, kd, stall, w0 = env._servo; na = env._nq_act
stall = np.full(na, stall); w0 = np.full(na, w0); kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
for i, n in enumerate(G.JN):
    s = G.SERVOS[G.PJ_DEFAULT[n]] if n in G.PJ_DEFAULT else None
    if s: stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
env._servo = np.array([kpa, kda, stall, w0], dtype=object)
env.fall_up_z = -2.0; env.fall_height = -1.0
m, d = env.model, env.data; d0, hi, lo = env._default, env._hi, env._lo
def inv(q):
    q = q[:na] if len(q) >= na else np.concatenate([q, np.zeros(na - len(q))])
    return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
q_prev = G.q_from_offsets(seq[0][1]); G.settle_fallen(env, P, "supine", q_prev)
dt = env.control_dt; T, Q, QP, QV, LAB = [], [], [], [], []
t = 0.0
for label, off, move_s, hold_s in seq:
    q_tgt = G.q_from_offsets(off); n = int(move_s / dt)
    for i in range(n + int(hold_s / dt)):
        s = min(1.0, (i + 1) / max(n, 1)); s = 10 * s**3 - 15 * s**4 + 6 * s**5
        q = q_prev + (q_tgt - q_prev) * s
        T.append(t); Q.append(q[:na].copy()); QP.append(d.qpos.copy()); QV.append(d.qvel.copy()); LAB.append(label)
        env.step(inv(q)); t += dt
    q_prev = q_tgt
up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
names = [env.model.actuator(i).name for i in range(na)]
np.savez(OUT, t=np.array(T), q=np.array(Q), qpos=np.array(QP), qvel=np.array(QV), label=np.array(LAB),
         joint_names=np.array(names), control_dt=dt, final_up=up, final_z=d.qpos[2],
         source="open-loop seat_push(60,0,2.0,-25,elbow=(-60,0)) on bimo_biped_v6ar.xml, play 3 deg, PJ_DEFAULT servos")
print(f"steps {len(T)} ({t:.1f} s), dt {dt}, na {na}, final up {up:+.2f} pelvis {d.qpos[2]:.3f}; joints {names}")
