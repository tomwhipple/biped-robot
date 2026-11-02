"""Parse the firmware `balance log` dumps in a serial log into runs of numpy arrays
(t s, roll deg, err deg, u deg, rate deg/s) and score them the way the sim is scored."""
from __future__ import annotations

import os
import re

import numpy as np

LOG = os.environ.get("GYROSIGN_LOG", "/home/claw/code/robot/hw_sessions/2026-10-25/gyrosign_serial.log")
NAMES = ["baseline kd 0.03", "ZV shaper", "kd 0.05", "kd 0.07 (unstable)"]


def runs(path=LOG):
    out, cur = [], None
    for ln in open(path, errors="replace"):
        if "balance log:" in ln:
            cur = []
            out.append(cur)
        elif cur is not None and (m := re.search(r"BAL (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)", ln)):
            ms, ro, er, u, rt = (int(x) for x in m.groups())
            cur.append((ms / 1e3, ro / 100, er / 100, u / 100, rt / 10))
        elif "balance log end" in ln:
            cur = None
    return [np.array(r) for r in out]


def score(t, rate, err, wins=((0, 4), (4, 8), (8, 12)), band=(1.2, 4.0)):
    """rate rms per window (deg/s), peak |err|, dominant frequency of the rate in `band`
    over the first 8 s (Hz), and the lightly-damped decay: rms ratio of windows 2/1, 3/2."""
    rms = [float(np.sqrt(np.mean(rate[(t >= a) & (t < b)] ** 2))) if np.any((t >= a) & (t < b)) else float("nan") for a, b in wins]
    sel = (t >= 1.0) & (t < 8.0)
    tt, rr = t[sel], rate[sel] - np.mean(rate[sel])
    tg = np.arange(tt[0], tt[-1], 0.02)
    rg = np.interp(tg, tt, rr) * np.hanning(len(tg))
    f = np.fft.rfftfreq(len(tg) * 8, 0.02)
    P = np.abs(np.fft.rfft(rg, len(tg) * 8)) ** 2
    m = (f >= band[0]) & (f <= band[1])
    fpk = float(f[m][np.argmax(P[m])])
    return dict(rms=rms, err_peak=float(np.max(np.abs(err))), f_hz=fpk)


if __name__ == "__main__":
    for n, r in zip(NAMES, runs()):
        t = r[:, 0]
        s = score(t, r[:, 4], r[:, 2])
        print(f"{n:22s} N {len(r)} T {t[-1]:.1f}s  rate rms {['%.2f' % x for x in s['rms']]}  err peak {s['err_peak']:.2f}  f {s['f_hz']:.2f} Hz  roll p-p {np.ptp(r[:,1]):.2f}")
