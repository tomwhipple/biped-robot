"""Goal -> encoder transfer function per joint group, real (policy stand) vs the sim plant
fed the same goals (goal_replay.py), from cross-spectra (Welch, H1 = Sgq / Sgg)."""
import os, sys, json
import numpy as np
from scipy.signal import csd, welch
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec
GROUPS = {"hip_yaw": ["L_hip_yaw", "R_hip_yaw"], "hip_roll": ["L_hip_roll", "R_hip_roll"],
          "hip_pitch": ["L_hip_pitch", "R_hip_pitch"], "knee": ["L_knee", "R_knee"],
          "ankle": ["L_ankle", "R_ankle"], "ankle_roll": ["L_ankle_roll", "R_ankle_roll"]}
FQ = [1.0, 2.0, 3.0, 4.0, 6.0, 9.0]


def tf(goal, q, lo, hi, nper=64):
    out = {}
    for g, names in GROUPS.items():
        Sgq = 0; Sgg = 0
        for n in names:
            j = rec.POL.index(n)
            x = goal[lo:hi, j] - goal[lo:hi, j].mean(); y = q[lo:hi, j] - q[lo:hi, j].mean()
            f, s1 = csd(x, y, fs=50, nperseg=nper); _, s2 = welch(x, fs=50, nperseg=nper)
            Sgq = Sgq + s1; Sgg = Sgg + s2
        H = Sgq / Sgg
        out[g] = {fq: (float(np.abs(np.interp(fq, f, H.real) + 1j * np.interp(fq, f, H.imag))),
                       float(np.degrees(np.angle(np.interp(fq, f, H.real) + 1j * np.interp(fq, f, H.imag)))))
                  for fq in FQ}
    return out


def show(label, o):
    print(label)
    for g, v in o.items():
        print(f"  {g:10s} " + "  ".join(f"{fq:g}Hz {v[fq][0]:.2f}/{v[fq][1]:4.0f}" for fq in FQ))


if __name__ == "__main__":
    r = rec.load()
    show("REAL 0.5-3.3 s (gain/phase deg)", tf(r["goal"], r["q"], 25, 165))
    show("REAL 3.3-9.0 s", tf(r["goal"], r["q"], 165, 450))
    import goal_replay as GR, simh
    for plant, model in [("fit", None), ("train", None), ("fit", dict(roll_lash_deg=0.0, backlash_deg=0.0))]:
        sim, _ = GR.replay(plant, model, simh.Servo(lag_hz=12))
        show(f"SIM {plant} {model} fed the robot's goals, 0.5-3.3 s", tf(r["goal"][:len(sim["q"])], sim["q"], 25, min(165, len(sim["q"]))))
