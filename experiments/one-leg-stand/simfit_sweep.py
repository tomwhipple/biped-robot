"""The double-support lateral-roll stepped sine (firmware `sweep`, PR #184), robot vs sim.

hw 2026-11-01 21:32:46 UTC, HARD floor, crouch pose, `sweep 11`: +-11 ticks on all four roll joints
(both hip rolls and both ankle rolls, the same tick sign = a lateral pelvis shift) at 0.5, 1, 1.5,
2, 2.5, 3, 4 Hz, each max(3 s, 4 cycles) + 0.4 s cosine tapers at both ends. Per frequency a
least-squares sinusoid gives, on the robot and in the sim:

    roll / cmd     body roll over the commanded offset (deg/deg; cmd = 11 ticks on L_hip_roll)
    <joint> act/goal   the servo's read-back over its goal (the shaft: encoder in hw, link + play in sim)

`sweep_cost`: sum over frequencies of |ln(|G_sim|/|G_hw|)|/0.3 + |phase error|/30 deg for roll/cmd,
plus half that for the hip-roll and ankle-roll act/goal (the stance-loaded tracking).

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_sweep.py [--set k=v ...]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402

TAG = "213236"
H.SESSIONS.setdefault(TAG, (os.path.join(H.HW_ROOT, "2026-11-01"), "flight_20261101T213236Z_ilim_stands", "hard", H._ILIM3))
AMP = 11
FREQS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0)
ROLLS = ("R_hip_roll", "L_hip_roll", "R_ankle_roll", "L_ankle_roll")
TICK_DEG = 360.0 / 4096


def segments():
    """[(f, t_start, t_end)] in sweep time (the steady part, tapers and the first cycle cut)."""
    out, ts = [], 0.0
    for f in FREQS:
        dur = max(3.0, 4.0 / f) + 0.8
        out.append((f, ts + 0.4 + 1.0 / f, ts + dur - 0.4))
        ts += dur
    return out, ts


def sweep_offset(u):
    """ticks added to the four roll joints at u s into the sweep (cli.cpp cmdSweep)."""
    ts = 0.0
    for f in FREQS:
        dur = max(3.0, 4.0 / f) + 0.8
        if u < ts + dur:
            v = u - ts
            env = min(1.0, min(v, dur - v) / 0.4)
            e = 0.5 - 0.5 * math.cos(math.pi * max(0.0, env))
            return AMP * e * math.sin(2 * math.pi * f * v)
        ts += dur
    return 0.0


def load():
    e = H._events(TAG)
    d = H._ticks(TAG)
    pose = e[(e.kind == "cmd") & e.text.astype(str).str.startswith("pose ")].iloc[0]
    sw = e[(e.kind == "out") & e.text.astype(str).str.startswith("sweep: +-")].iloc[0]
    t0 = float(pose.t_utc)
    tok = str(pose.text).split()
    ticks = [int(x) for x in tok[1:13]]
    t_sw = float(sw.t_utc) - t0 - 0.03             # sw.t0 is set just before the reply
    moves = [H.Move("crouch", 0.0, 3.0, ticks, None)]
    w = d[(d.t_utc >= t0 - 1.0) & (d.t_utc <= t0 + t_sw + 40)].copy()
    w["t"] = w.t_utc - t0
    st = H.Stand(TAG, "R", t0, moves, w.reset_index(drop=True), None, None, {}, H._cal_from(d),
                 rep=1, floor="hard", fw={})
    return st, t_sw


def sinfit(t, x, f):
    """complex amplitude of x at f (least squares with offset + linear drift)."""
    ok = np.isfinite(x)
    t, x = t[ok], x[ok]
    if len(t) < 10:
        return complex("nan")
    A = np.stack([np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t), np.ones_like(t), t - t.mean()], 1)
    c, *_ = np.linalg.lstsq(A, x, rcond=None)
    return complex(c[0], c[1])          # x ~ Re{(a - j b) ...}: a sin + b cos = |.| sin(wt + phi)


def ratio(num, den):
    """gain and phase (deg) of num over den, both as (sin, cos) coefficient pairs."""
    pn = math.atan2(num.imag, num.real)
    pd = math.atan2(den.imag, den.real)
    return abs(num) / max(abs(den), 1e-9), (math.degrees(pn - pd) + 180) % 360 - 180


def response(t, roll_deg, goal, act, t_sw):
    """per frequency: (f, roll/cmd gain, phase, {joint: (act/goal gain, phase)})."""
    segs, _ = segments()
    rows = []
    for f, a, b in segs:
        w = (t >= t_sw + a) & (t <= t_sw + b)
        tt = t[w] - t_sw
        cmd = sinfit(tt, np.array([sweep_offset(u) for u in tt]) * TICK_DEG, f)
        rr = sinfit(tt, roll_deg[w], f)
        g, ph = ratio(rr, cmd)
        js = {}
        for j in ROLLS:
            gj = sinfit(tt, goal[j][w], f)
            aj = sinfit(tt, act[j][w], f)
            js[j] = ratio(aj, gj)
        rows.append((f, g, ph, js))
    return rows


def hw_response():
    st, t_sw = load()
    d = st.ticks
    t = d.t.values
    goal = {j: d[f"{j}.goal"].values.astype(float) for j in ROLLS}
    act = {j: d[f"{j}.pos"].values.astype(float) for j in ROLLS}
    return response(t, d.roll.values * 180 / math.pi, goal, act, t_sw)


def sim_response(pl):
    st, t_sw = load()
    extra = np.zeros(12)
    idx = [H.BUS.index(j) for j in ROLLS]

    def fn(t):
        extra[:] = 0.0
        extra[idx] = sweep_offset(t - t_sw) if t >= t_sw else 0.0
        return extra

    _, tot = segments()
    r = S.Replay(st, pl.on("hard"), S.Ctl(), extra=fn, t_end=t_sw + tot + 0.5).run()
    t = r["t"]
    goal = {j: r["goal"][:, H.BUS.index(j)] for j in ROLLS}
    act = {j: r["enc"][:, H.BUS.index(j)] for j in ROLLS}
    return response(t, r["roll"] * 180 / math.pi, goal, act, t_sw)


def sweep_cost(sim, hw):
    c = 0.0
    for (f, g, ph, js), (_, gh, phh, jh) in zip(sim, hw):
        if not np.isfinite(g):
            c += 3.0
            continue
        c += abs(math.log(max(g, 1e-3) / gh)) / 0.3 + abs((ph - phh + 180) % 360 - 180) / 30.0
        for j in ("R_hip_roll", "R_ankle_roll"):
            a, p = js[j]
            ah, pp = jh[j]
            c += 0.5 * (abs(math.log(max(a, 1e-3) / max(ah, 1e-3))) / 0.3 + abs((p - pp + 180) % 360 - 180) / 30.0)
    return c / len(sim)


_HW = None


def cost_job(pl):
    global _HW
    if _HW is None:
        _HW = hw_response()
    s = sim_response(pl)
    return sweep_cost(s, _HW), s


def report(rows, lab):
    print(lab)
    for f, g, ph, js in rows:
        print(f"  {f:3.1f} Hz  roll/cmd {g:4.2f} {ph:+5.0f}   " +
              "  ".join(f"{j[:1]}{j[2:5]}{j[-4:]} {a:4.2f} {p:+5.0f}" for j, (a, p) in js.items()))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    a = ap.parse_args()
    d = json.load(open(a.fitted))
    pl = replace(S.Plant(**d["plant"]), **{k: float(v) for k, v in (x.split("=") for x in a.set)})
    hw = hw_response()
    report(hw, "robot")
    sim = sim_response(pl)
    report(sim, "sim")
    print(f"sweep cost {sweep_cost(sim, hw):.2f}")
