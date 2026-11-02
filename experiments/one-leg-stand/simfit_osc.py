"""The roll oscillation in a B stage, measured the same way on the robot and in the sim.

The hard-floor stands (2026-11-01) go unstable in a ~1.6-2.1 Hz roll mode that the carpet damps.
A tick-by-tick rms of err is phase-sensitive and says little about an oscillation, so the floor fit
scores the mode itself:

    osc(t, roll, t0, t1) -> dict(
        f_hz     dominant frequency of the band-passed roll (0.8-4 Hz), from its zero crossings
        a_early  rms of the band-passed roll over [t0+4.5, t0+6.5] s (deg) -- after the lift, before
                 the hard-floor runs diverge
        a_late   rms over the last 2 s of the window (the 1.5 s before an abort, or the hold's end)
        growth   log(a_late / a_early) / dt (1/s): > 0 = growing
        p2p      max - min of the band-passed roll in the window (deg)
        env      (t, envelope) of the band-passed roll for plots
    )

Both sides are resampled to a uniform 100 Hz grid first (the robot's ticks jitter around 26 ms).
"""
from __future__ import annotations

import math

import numpy as np
from scipy.signal import butter, sosfiltfilt, hilbert

R2D = 180 / math.pi
FS = 100.0
BAND = (0.8, 4.0)


def bandpass(t, x, band=BAND):
    """uniform-grid (tg, band-passed x) of x(t)."""
    ok = np.isfinite(x)
    t, x = np.asarray(t)[ok], np.asarray(x)[ok]
    if len(t) < 10:
        return np.array([]), np.array([])
    tg = np.arange(t[0], t[-1], 1.0 / FS)
    xg = np.interp(tg, t, x)
    if len(tg) < 40:
        return tg, np.zeros_like(tg)
    sos = butter(2, band, btype="band", fs=FS, output="sos")
    return tg, sosfiltfilt(sos, xg - xg.mean())


def osc(t, roll, t0, t1, early=(4.5, 6.5)):
    """roll in rad; window [t0, t1] (t1 = the abort or the stage end)."""
    t = np.asarray(t, float)
    w = (t >= t0 - 0.5) & (t <= t1)
    tg, xb = bandpass(t[w], np.asarray(roll, float)[w] * R2D)
    out = dict(f_hz=float("nan"), a_early=float("nan"), a_late=float("nan"), growth=float("nan"), p2p=float("nan"),
               env=(tg, np.abs(hilbert(xb)) if len(xb) else xb))
    if len(tg) < 100:
        return out
    m = tg >= t0
    out["p2p"] = float(xb[m].max() - xb[m].min())
    late = (tg >= max(t0, t1 - 2.0)) & (tg <= t1 - 0.1)
    ea = (tg >= t0 + early[0]) & (tg <= min(t0 + early[1], t1 - 0.1))
    if late.sum() > 20:
        out["a_late"] = float(np.sqrt(np.mean(xb[late] ** 2)))
        # frequency from the zero crossings over the last 3 s
        z = (tg >= max(t0, t1 - 3.0)) & (tg <= t1 - 0.1)
        s = np.signbit(xb[z]).astype(int)
        zc = np.where(np.diff(s) != 0)[0]
        if len(zc) >= 3:
            tz = tg[z][zc]
            out["f_hz"] = float((len(zc) - 1) / (2.0 * (tz[-1] - tz[0])))
    if ea.sum() > 20:
        out["a_early"] = float(np.sqrt(np.mean(xb[ea] ** 2)))
    if late.sum() > 20 and ea.sum() > 20:
        dt = (t1 - 1.0) - (t0 + 0.5 * (early[0] + early[1]))
        if dt > 0.5:
            out["growth"] = float(math.log(max(out["a_late"], 1e-3) / max(out["a_early"], 1e-3)) / dt)
    return out


def hw_osc(st, stage="l30"):
    mv = st.stage(stage)
    t0, t1 = st.stage_window(stage)
    d = st.ticks
    return osc(d.t.values, d.roll.values, t0, t1)


def sim_osc(st, r, stage="l30"):
    mv = st.stage(stage)
    t0 = mv.t0
    t_next = next((m.t0 for m in st.moves if m.t0 > t0 + 1e-6), t0 + mv.dur + 12)
    ab = np.where((r["t"] >= t0) & (r["t"] < t_next) & (r["state"] == 2))[0]
    t1 = float(r["t"][ab[0]]) if len(ab) else t_next
    return osc(r["t"], r["roll"], t0, t1), (t1 - t0 if len(ab) else None)


if __name__ == "__main__":
    import simfit_hw as H
    for st in H.all_stands():
        for stg in ("l10", "l30"):
            o = hw_osc(st, stg)
            print(f"{st.id:10s} {st.floor:6s} {stg} f {o['f_hz']:.2f} Hz  early {o['a_early']:.2f}  late {o['a_late']:.2f}  "
                  f"growth {o['growth']:+.2f}/s  p2p {o['p2p']:.1f} deg")
