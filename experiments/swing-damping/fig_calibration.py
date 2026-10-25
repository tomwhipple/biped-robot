"""Figure: hardware balance logs vs the calibrated sim (right stance), kd 0.03 / 0.05 / 0.07, and the swing-law effect."""
import os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import hwlog, swing_eval as E, swing_sim as S
hw = hwlog.runs()
fig, ax = plt.subplots(3, 2, figsize=(11, 7.5), sharex=True)
for i, (kd, k) in enumerate(((0.03, 0), (0.05, 2), (0.07, 3))):
    h = hw[k]
    ax[i, 0].plot(h[:, 0], h[:, 4], lw=0.9, color="k", label="hardware")
    r = S.run(E.cal(seed=0), dict(kd=kd), None); L = r["log"]
    ax[i, 0].plot(L[:, 0], np.degrees(L[:, 2]), lw=0.7, color="C0", label="sim (calibrated)")
    ax[i, 0].set_ylabel(f"kd {kd}\nroll rate, deg/s"); ax[i, 0].set_xlim(0, 12.2); ax[i, 0].grid(alpha=.3)
    if kd == 0.07: ax[i, 0].set_ylim(-150, 150)
    if i == 0: ax[i, 0].legend(loc="upper right", fontsize=8)
    for lab, sw, c in (("stance only", None, "C0"), ("preload 3 + hip rate 0.3", dict(sr=0.3, sh=1.0, sb=3.0, lim=8.0), "C3")):
        r = S.run(E.cal(seed=0), dict(kd=kd), sw); L = r["log"]
        ax[i, 1].plot(L[:, 0], np.degrees(L[:, 2]), lw=0.7, color=c, label=lab)
    ax[i, 1].set_xlim(0, 12.2); ax[i, 1].grid(alpha=.3)
    if kd == 0.07: ax[i, 1].set_ylim(-150, 150)
    if i == 0: ax[i, 1].legend(loc="upper right", fontsize=8)
ax[2, 0].set_xlabel("s from engage / lift start"); ax[2, 1].set_xlabel("s from lift start")
ax[0, 0].set_title("calibration: right stance, 1.3x shift, 30 mm in 4 s"); ax[0, 1].set_title("sim: swing-leg law on / off")
fig.tight_layout(); fig.savefig(os.path.join(HERE, "2026-10-25-calibration-and-swing-law.png"), dpi=110)
