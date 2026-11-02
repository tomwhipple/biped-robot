"""The 2026-11-01 double-support lateral-roll sweep (firmware `sweep 11`): per frequency,
present/goal amplitude ratio and phase for the four roll joints, and body roll / goal."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec

FREQS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0]
ROLLS = ["L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll"]
T_SWEEP = 1793568765.99259 - 0.03     # 'sweep' ack; t0 is the next housekeeping tick


def segments():
    out, ts = [], 0.0
    for f in FREQS:
        dur = max(3.0, 4.0 / f) + 0.8
        out.append((f, ts + 0.4, ts + dur - 0.4))
        ts += dur
    return out


def fit_sine(t, x, f):
    A = np.c_[np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t), np.ones_like(t), t]
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return complex(c[0], c[1])     # amplitude/phase phasor (sin reference)


def analyse(r=None):
    r = r or rec.load(rec.SWEEP, source="pose")
    t = r["t"]
    # sweep start: first tick where any roll goal departs from the held pose after T_SWEEP
    j = [rec.POL.index(n) for n in ROLLS]
    g = r["goal"][:, j]
    k0 = np.searchsorted(t, T_SWEEP)
    base = g[k0]
    moved = np.where(np.any(np.abs(g[k0:] - base) > 1e-6, axis=1))[0]
    t0 = t[k0 + moved[0]] - 0.02 if len(moved) else t[k0]
    res = []
    roll = np.radians(r["roll"]) if np.abs(r["roll"]).max() > 1.5 else r["roll"]
    for f, a, b in segments():
        m = (t >= t0 + a) & (t <= t0 + b)
        tt = t[m] - t0
        row = dict(f=f)
        for n, jj in zip(ROLLS, j):
            G = fit_sine(tt, r["goal"][m, jj], f)
            # present position is sampled AFTER the goal write: fb time offset folded into tt
            P = fit_sine(tt, r["q"][m, jj], f)
            H = P / G
            row[n] = dict(goal_deg=float(np.degrees(abs(G))), gain=float(abs(H)),
                          phase_deg=float(np.degrees(np.angle(H))))
        Gm = np.mean([fit_sine(tt, r["goal"][m, jj], f) for jj in j[:2]])
        R = fit_sine(tt, roll[m], f)
        row["body_roll"] = dict(deg=float(np.degrees(abs(R))), per_goal=float(abs(R / Gm)),
                                phase_deg=float(np.degrees(np.angle(R / Gm))))
        res.append(row)
    return res, t0


if __name__ == "__main__":
    res, t0 = analyse()
    print("sweep t0", t0)
    for row in res:
        s = f"{row['f']:3.1f} Hz "
        for n in ROLLS:
            v = row[n]; s += f"| {n} goal {v['goal_deg']:.2f} gain {v['gain']:.2f} ph {v['phase_deg']:6.0f} "
        v = row["body_roll"]; s += f"| body roll {v['deg']:.2f} deg ({v['per_goal']:.2f}x hip goal, ph {v['phase_deg']:.0f})"
        print(s)
    json.dump(res, open(os.path.join(HERE, "sweep_real.json"), "w"), indent=1)
