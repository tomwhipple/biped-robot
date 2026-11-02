"""The stance plant's frequency response, u -> torso roll, in the sim and from the robot's limit cycles.

The hard-floor oscillation is a loop instability: at the robot's limit cycle the roll PD
C(jw) = kp + kd*jw and the plant P(jw) = roll/u satisfy C*P = 1. The robot's P at the oscillation
frequency is read straight from its own u and roll (u is computed on board, so its spectrum is
exact): ~2.3 deg/deg at -46 deg at 1.8-1.9 Hz on all three unstable stands. The sim's P is measured
by adding a sinusoid to u in the 30 mm hold (Ctl.dither_*) and demodulating roll and u at that
frequency over whole periods.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_frf.py hw
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_frf.py sim --set k=v ... [--floor hard] [--side R]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402

R2D = 180 / math.pi
FREQS = (1.0, 1.4, 1.8, 2.2, 2.8, 3.5)


def frf_hw(sid, a, b):
    """(f, |P|, phase deg) at the dominant 1-3 Hz line of roll in [l30 + a, l30 + b]."""
    st = H.load_stand(sid)
    d = st.ticks
    t0 = st.stage("l30").t0
    ok = (d.bal_state == 1).values
    tg = np.arange(t0 + a, t0 + b, 0.01)
    u = np.interp(tg, d.t.values[ok], d.bal_u.values[ok])
    y = np.interp(tg, d.t.values[ok], d.roll.values[ok])
    u = u - np.polyval(np.polyfit(tg, u, 1), tg)
    y = y - np.polyval(np.polyfit(tg, y, 1), tg)
    w = np.hanning(len(tg))
    U, Y = np.fft.rfft(u * w), np.fft.rfft(y * w)
    f = np.fft.rfftfreq(len(tg), 0.01)
    k = np.argmax(np.abs(Y) * ((f > 1.0) & (f < 3.0)))
    P = Y[k] / U[k]
    return float(f[k]), float(abs(P)), float(np.degrees(np.angle(P)))


HW_CYCLES = [("203534R", 2.4, 5.6), ("203257R", 2.5, 7.4), ("203759R3", 2.3, 5.6)]


def _one(job):
    pl, floor, side, f, amp, ref = job
    own, moves = H.protocol_moves(side, ref)
    ctl = S.Ctl(i_max=math.radians(3.0), dither_deg=amp, dither_hz=f, dither_t0=6.0)
    r = S.Replay(own, pl.on(floor), ctl, moves=moves).run()
    l30 = next(m for m in moves if m.name == "l30")
    n_per = max(3, int(round(5.0 * f)))
    ta = l30.t0 + 6.0 + 1.0 / f                       # skip the first period
    tb = ta + n_per / f
    w = (r["t"] >= ta) & (r["t"] < tb) & (r["state"] == 1)
    if w.sum() < 20 or np.any(r["state"][(r["t"] >= l30.t0) & (r["t"] < tb)] == 2):
        return f, float("nan"), float("nan")
    t = r["t"][w]
    ph = 2 * math.pi * f * t
    def demod(x):
        x = x - x.mean()
        return 2 * np.mean(x * np.exp(-1j * ph))
    P = demod(r["roll"][w]) / demod(r["u"][w])
    return f, float(abs(P)), float(np.degrees(np.angle(P)))


def frf_sim(pl, floor="hard", side="R", freqs=FREQS, amp=1.0, ref="194709"):
    with ProcessPoolExecutor(max_workers=len(freqs)) as ex:
        return list(ex.map(_one, [(pl, floor, side, f, amp, ref) for f in freqs]))


def loop_margin(rows, kp=0.3, kd=0.03):
    """|C*P| and its phase per frequency (C = kp + kd*jw): instability where |CP| > 1 near phase 0."""
    out = []
    for f, m, ph in rows:
        C = kp + kd * 1j * 2 * math.pi * f
        L = abs(C) * m
        out.append((f, m, ph, L, ph + math.degrees(np.angle(C))))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("hw", "sim"))
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    ap.add_argument("--floor", default="hard")
    ap.add_argument("--side", default="R")
    ap.add_argument("--amp", type=float, default=1.0)
    a = ap.parse_args()
    if a.mode == "hw":
        for sid, t_a, t_b in HW_CYCLES:
            f, m, ph = frf_hw(sid, t_a, t_b)
            print(f"{sid}: {f:.2f} Hz  |P| {m:.2f}  phase {ph:+.0f} deg")
    else:
        d = json.load(open(a.fitted))
        pl = S.Plant(**d["plant"])
        pl = replace(pl, **{k: float(v) for k, v in (x.split("=") for x in a.set)})
        for f, m, ph, L, phL in loop_margin(frf_sim(pl, a.floor, a.side, amp=a.amp)):
            print(f"  {f:.1f} Hz  |P| {m:.2f}  phase {ph:+5.0f}   |CP| {L:.2f}  phase {phL:+5.0f}")
