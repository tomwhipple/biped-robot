"""IMU axis check from leg kinematics: with a foot flat on the floor the torso's body
rates are minus the sum of that leg's joint rates about each axis (small angles):
  wx = -(hip_roll' + ankle_roll'), wy = -(hip_pitch' - knee' + ankle'), wz = -hip_yaw'
(joint axes from sim/bimo_biped_v6legs.xml). Regress the recorded gyro on that."""
import os, sys, json
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec

P = {n: i for i, n in enumerate(rec.POL)}


def kin_rates(q, side):
    s = side + "_"
    dq = np.gradient(q, 0.02, axis=0)
    wx = -(dq[:, P[s + "hip_roll"]] + dq[:, P[s + "ankle_roll"]])
    wy = -(dq[:, P[s + "hip_pitch"]] - dq[:, P[s + "knee"]] + dq[:, P[s + "ankle"]])
    wz = -dq[:, P[s + "hip_yaw"]]
    return np.c_[wx, wy, wz]


def check(r, lo, hi, band=(0.5, 8.0)):
    b, a = butter(2, [band[0] / 25, band[1] / 25], "band")
    g = filtfilt(b, a, r["gyro"][:hi], axis=0)[lo:hi]
    out = {}
    for side in "LR":
        k = filtfilt(b, a, kin_rates(r["q"][:hi], side), axis=0)[lo:hi]
        for ax, nm in enumerate("xyz"):
            c = np.corrcoef(g[:, ax], k[:, ax])[0, 1]
            slope = np.dot(g[:, ax], k[:, ax]) / np.dot(k[:, ax], k[:, ax])
            out[f"{side}_{nm}"] = (float(c), float(slope))
    return out


if __name__ == "__main__":
    r = rec.load()
    for lo, hi in [(25, 160), (100, 160), (160, 260)]:
        res = check(r, lo, hi)
        print(f"policy stand {lo/50:.1f}-{hi/50:.1f} s:", {k: f"corr {c:+.2f} slope {s:+.2f}" for k, (c, s) in res.items()})
    rs = rec.load(rec.SWEEP, source="pose")
    import sweep_fit as SF
    _, t0 = SF.analyse(rs)
    k0 = np.searchsorted(rs["t"], t0); k1 = k0 + 1500
    res = check(rs, k0, k1)
    print("roll sweep:", {k: f"corr {c:+.2f} slope {s:+.2f}" for k, (c, s) in res.items()})
