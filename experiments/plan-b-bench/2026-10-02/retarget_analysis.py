"""Accelerometer around a mid-move goal change (sweep_smooth.py R- / R+), 2026-10-02.
For each R row: the retarget's host time is sweep_host_t0 + t_retarget; the
accelerometer sample's host time is ~bno_host_t0 + t_us (the `bno rec` reply
lands a few ms after the firmware's t0). Reports y (rotation direction) RMS
above 5 Hz in [-0.3, +0.8] s around the change vs the rest of the moving part,
and the servo's reported speed just before / after.
  .venv/bin/python experiments/plan-b-bench/2026-10-02/retarget_analysis.py <sweep_smooth.json>
"""
import json, sys
import numpy as np


def hp(x, fs, fc=5.0):
    f = np.fft.rfftfreq(len(x), 1 / fs)
    return np.fft.irfft(np.where(f >= fc, np.fft.rfft(x - x.mean()), 0), len(x))


def main(path):
    r = json.load(open(path))
    print(f"{'P/D':7s} {'prof':4s} {'way':5s} {'t_ret':>6s} {'y win':>7s} {'y rest':>7s} {'ratio':>6s}  spd before -> after (steps/s)")
    for row in r["rows"]:
        if not row.get("bno") or len(row["bno"]) < 200:
            continue
        a = np.array(row["bno"], float)
        t = row["bno_host_t0"] + a[:, 0] * 1e-6
        fs = (len(a) - 1) / ((a[-1, 0] - a[0, 0]) * 1e-6)
        y = hp(a[:, 2], fs)
        tr = row["metrics"].get("t_retarget")
        tr_host = None if tr is None else row["sweep_host_t0"] + tr
        tt = [s["t"] for s in row["trace"]]
        moving = [row["sweep_host_t0"] + s["t"] for s in row["trace"] if s["spd"] != 0]
        if not moving:
            continue
        m0, m1 = min(moving), max(moving)
        mov = (t >= m0) & (t <= m1)
        if tr_host is None:
            print(f"{row['p']}/{row['d']:<4} {row['profile']:4s} {row['way']:5s} {'-':>6s} {'':>7s} {np.sqrt(np.mean(y[mov]**2)):7.0f}")
            continue
        win = (t >= tr_host - 0.3) & (t <= tr_host + 0.8)
        rest = mov & ~win
        yw, yr = np.sqrt(np.mean(y[win] ** 2)), np.sqrt(np.mean(y[rest] ** 2))
        before = [s["spd"] for s in row["trace"] if tr - 0.4 <= s["t"] < tr]
        after = [s["spd"] for s in row["trace"] if tr <= s["t"] < tr + 0.6]
        print(f"{row['p']}/{row['d']:<4} {row['profile']:4s} {row['way']:5s} {tr:6.2f} {yw:7.0f} {yr:7.0f} {yw / yr:6.2f}  {before} -> {after}")


if __name__ == "__main__":
    main(sys.argv[1])
