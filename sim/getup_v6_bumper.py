"""Body-mod: rear pelvis bumper (skid extended rearward + slight lip) so the
unfold rise has a chair-back to lean against instead of tipping backward over
the heel. The skid in gen_plant_v6 is a floor seat; add a vertical rear wall so
the torso can push up against it and translate the CoM forward onto the feet.

Sweep skid_x (rearward placement) and skid_len at a fixed skid_h, and check
whether a butt-up + unfold sequence now holds the torso upright past the
heel->CoM corridor.
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import dataclasses as dc
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from skid_alias import replace_skid_h
from static_gait import make_env

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def run_rise(p, seq_kind="buttup", hold=-95, butt_k=-15):
    xml = "/tmp/v6_bumper.xml"; open(xml, "w").write(build_xml(p))
    env = make_env(p, xml); obs, _ = env.reset(seed=0)
    kp, kd, stall_s, w0_s = env._servo; na = env._nq_act
    stall = np.full(na, float(stall_s)); w0 = np.full(na, float(w0_s))
    kpa = np.full(na, float(kp)); kda = np.full(na, float(kd))
    for i, n in enumerate(G.JN):
        if n in G.PJ_DEFAULT:
            s = G.SERVOS["sts3250"]; stall[i] = s["stall"]; w0[i] = s["w0"]; kpa[i] *= s["kp_scale"]; kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall, w0], dtype=object)
    env.fall_up_z = -2; env.fall_height = -1
    m, d = env.model, env.data
    d0, hi, lo = env._default, env._hi, env._lo
    def inv(q):
        q = np.concatenate([q, np.zeros(na - len(q))]) if len(q) < na else q[:na]
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    # with a rear bumper the sit-up can pivot off the skid lip; try butt-up then unfold
    seq = [
        ("sit", dict(hip_pitch=-90, knee=10, ankle=0), 1.4, 0.5),
        ("fold", dict(hip_pitch=hold, knee=10, ankle=0), 2.0, 1.0),
        ("chair: lean fwd onto toes", dict(hip_pitch=-55, knee=-30, ankle=-35), 2.0, 1.0),
        ("push off seat + extend", dict(hip_pitch=-15, knee=-5, ankle=-20), 2.6, 2.0),
    ]
    qprev = qq({}); G.settle_fallen(env, p, "supine", qprev)
    dt = env.control_dt
    res = {}
    for label, off, ms, hs in seq:
        qt = qq(off); nstep = int(ms / dt)
        for i in range(nstep + int(hs / dt)):
            s = min(1.0, (i + 1) / max(nstep, 1)); s = 10*s**3 - 15*s**4 + 6*s**5
            env.step(inv(qprev + (qt - qprev) * s))
        qprev = qt
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        res[label] = (up, d.qpos[2])
    return res

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skid-h", type=float, default=0.03)
    a = ap.parse_args()
    print(f"### rear-bumper chair-sweep: skid_h={a.skid_h}, lean fwd then extend ###")
    print(f"{'skid_x':>7} {'skid_len':>8} | {'sit':>6} {'fold':>6} {'chair(u)':>8} {'extend(up/pelZ)':>16}")
    for skid_x, skid_len in ((-0.04, 0.10), (-0.05, 0.11), (-0.06, 0.12), (-0.07, 0.13), (-0.08, 0.14)):
        p = replace_skid_h(DesignParams(), skid_h=a.skid_h, skid_x=skid_x, skid_len=skid_len)
        r = run_rise(p)
        u, z = r["push off seat + extend"]
        mark = " <== STANDS" if u > 0.9 and z > 0.3 else ""
        print(f"{skid_x:>7.3f} {skid_len:>8.3f} | {r['sit'][0]:>6.2f} {r['fold'][0]:>6.2f} {r['chair: lean fwd onto toes'][0]:>8.2f} | {u:>7.2f} z={z:>5.3f}{mark}")

main()
