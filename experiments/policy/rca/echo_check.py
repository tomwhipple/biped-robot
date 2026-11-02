"""Does the joint-velocity observation echo the policy's own goals? Per joint: correlation
of the observed dq with the goal's rate one tick earlier (the firmware writes goal k, the
servo moves, tick k+1 reads it), real robot vs the training plant (link-side encoders)."""
import os, sys, json
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec


def echo(goal, dq, lo, hi, band=(1.0, 12.0)):
    b, a = butter(2, [band[0] / 25, band[1] / 25], "band")
    dg = np.gradient(goal, 0.02, axis=0)
    dg = filtfilt(b, a, dg, axis=0); dqf = filtfilt(b, a, dq, axis=0)
    out = []
    for j in range(goal.shape[1]):
        x = dg[lo - 1:hi - 1, j]; y = dqf[lo:hi, j]
        c = np.corrcoef(x, y)[0, 1]; g = np.dot(x, y) / max(np.dot(x, x), 1e-12)
        out.append((float(c), float(g)))
    return out


if __name__ == "__main__":
    r = rec.load()
    res = {"real": echo(r["goal"][:498], r["dq_reg"][:498], 50, 160)}
    import takeover as T, simh
    for lab, model in [("sim train (link encoders)", None),
                       ("sim fit + yaw play 2, encoder-side", dict(play={"hip_roll": 1.55, "ankle_roll": 0.96, "hip_yaw": 2.0}))]:
        m = T.episode("train" if "train" in lab else "fit", 0, simh.Servo(lag_hz=12), model, record=True)
        tr = m["trace"]
        # sim obs dq: rebuild from q obs trace by the same estimator the obs used
        dq = np.gradient(tr["q"], 0.02, axis=0)
        res[lab] = echo(tr["goal"], dq, 50, min(160, len(dq)))
    print("joint          " + " | ".join(f"{k[:34]:34s}" for k in res))
    for j, n in enumerate(rec.POL):
        print(f"{n:14s} " + " | ".join(f"corr {v[j][0]:+.2f} gain {v[j][1]:+.2f}{'':14s}" for v in res.values()))
    json.dump(res, open(os.path.join(HERE, "echo.json"), "w"), indent=1)
