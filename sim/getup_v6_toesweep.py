"""Body-mod feasibility: does extending the toe (foot_len/foot_toe forward)
stop the unfold backward-tip on the skid body? Sweep foot_toe with the passive
skid and run the rise (butt-up -> unfold). The physics hypothesis: a longer
anterior foot grows the support polygon forward, letting the CoM stay over it
through the hip-extension rise instead of tipping back behind the heel.

Run: .venv/bin/python sim/getup_v6_toesweep.py [--skid-h ..]
"""
import sys, os, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import dataclasses as dc
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from static_gait import make_env
import mujoco

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def run_rise(p, hold=-95, butt_k=-15):
    """Run sit->fold->tuck->buttup->unfold on the given body; return peak up
    during the unfold (does the torso rise stay upright?) and final up_z."""
    xml = "/tmp/v6_toesweep.xml"; open(xml, "w").write(build_xml(p))
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
    seq = [
        ("sit", dict(hip_pitch=-90, knee=10, ankle=0), 1.4, 0.5),
        ("fold", dict(hip_pitch=hold, knee=10, ankle=0), 2.0, 1.0),
        ("buttup", dict(hip_pitch=hold, knee=butt_k, ankle=-15), 2.0, 1.2),
        ("unfold", dict(hip_pitch=-20, knee=-10, ankle=-10), 2.4, 1.6),
    ]
    qprev = qq({}); G.settle_fallen(env, p, "supine", qprev)
    dt = env.control_dt
    results = {}
    for label, off, ms, hs in seq:
        qt = qq(off); nstep = int(ms / dt)
        for i in range(nstep + int(hs / dt)):
            s = min(1.0, (i + 1) / max(nstep, 1)); s = 10*s**3 - 15*s**4 + 6*s**5
            env.step(inv(qprev + (qt - qprev) * s))
        qprev = qt
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        results[label] = (up, d.qpos[2])
    return results

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skid-h", type=float, default=0.06)
    a = ap.parse_args()
    base = dc.replace(DesignParams(), skid=True, skid_h=a.skid_h, skid_x=-0.02, skid_len=0.08)
    print(f"skid_h={a.skid_h} (sweeping HEEL: keep toe fixed, grow foot_len rearward)")
    print(f"{'heel':>7} {'foot_len':>9} {'foot_toe':>9} | {'sit':>7} {'fold':>7} {'buttup':>7} {'unfold(up/pelZ)':>16}")
    # heel grows: foot_len increases, foot_toe stays 0.075 (foot extends backward)
    for foot_len in (0.13, 0.145, 0.16, 0.175, 0.19):
        p = dc.replace(base, foot_toe=0.075, foot_len=foot_len)
        heel = foot_len - 0.075
        r = run_rise(p)
        unfold_up, unfold_z = r["unfold"]
        mark = " <== STAYS UP" if unfold_up > 0.9 else ""
        print(f"{heel:>7.3f} {foot_len:>9.3f} {0.075:>9.3f} | {r['sit'][0]:>7.2f} {r['fold'][0]:>7.2f} {r['buttup'][0]:>7.2f} | {unfold_up:>7.2f} z={unfold_z:>5.3f}{mark}")

main()
