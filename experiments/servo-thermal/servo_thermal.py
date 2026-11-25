#!/usr/bin/env python3
"""Servo temperature from the flight records: what exists, a first-order thermal fit, and the margin to the
STS over-temperature cutoff (#6, thermal derating).

    .venv/bin/python experiments/supply-r/supply_r.py extract          # the shared cache (once; /tmp/s6r/cache)
    .venv/bin/python experiments/servo-thermal/servo_thermal.py bins    # 1 s bins -> /tmp/s6th/bins.npz
    .venv/bin/python experiments/servo-thermal/servo_thermal.py survey  # sessions, per-servo maxima, cold starts
    .venv/bin/python experiments/servo-thermal/servo_thermal.py fit [--boot 200] [--json fit.json]
    .venv/bin/python experiments/servo-thermal/servo_thermal.py predict [--json fit.json]

Data: every flight record in hw_sessions/ on mira, through supply_r.py's cache: in each 50 Hz tick the firmware
reads registers 56-70 of every fitted servo in one sync read, so each tick has, per leg servo, temperature
(reg 63, 1 degC), voltage (62, 0.1 V) and present current (69, 6.5 mA). Only ticks where all twelve leg servos
answered are cached (supply_r.extract_one).

Cleaning: reg 63 has bus-read spikes (single ticks up to 150 degC, 0.001-0.11 % of ticks per servo, the same
corruption at any load); a 51-tick (1 s) running median removes them.

The model, per servo (one thermal node: the temperature register's sensor and whatever it sits on):
    tau dT/dt = Tss + Rth P(t) - T,     P = V I            (W; reg 62 x reg 69, the servo's electrical input)
Tss is the servo's idle equilibrium (ambient + the electronics' own standby heating); it and the start
temperature T0 are free per chunk (a contiguous recording, cut at CHUNK_S), so ambient drift and the unknown
power-off history drop out; tau and Rth are common to all chunks. Output-error least squares at 1 s: for each
tau on a grid, (T0, Tss) per chunk and Rth are linear. The cold starts (power on after >= 6 h off, so T0 is the
room) give the standby rise dT0 = Tss - T0 directly.
P = V I counts the mechanical output as heat too (an upper bound on the heat; the robot's slow motions do
little work). The sim's heat is K_CU tau^2 (sts_servo_model), which is V I - max(tau w, 0) for its bridge model.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "experiments", "supply-r"))
sys.path.insert(0, os.path.join(ROOT, "sim"))

BINS = os.environ.get("SERVO_THERMAL_BINS", "/tmp/s6th/bins.npz")
LEGS = ("R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
        "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
        "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll")
STS3250 = ("R_hip_roll", "L_hip_roll")            # servo-map: IDs 51, 52
MEDIAN_TICKS = 51
SPIKE_C = 3.0
GAP_S = 5.0
CHUNK_S = 3600.0
COLD_GAP_S = 6 * 3600.0
T_CUTOFF = 70.0                                    # servo-map section 4: above 70 degC torque goes off (reg 13)
TAU_GRID = np.r_[np.arange(60, 600, 30), np.arange(600, 3601, 60)].astype(float)


def _utc(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%m-%dT%H:%M")


# ----------------------------------------------------------------------------- bins
def cmd_bins(a):
    import supply_r as S
    from scipy.ndimage import median_filter
    d = S.load_all(merge=True)["all"]
    assert tuple(S.LEGS) == LEGS
    t, T = d["t_utc"], d["temp"]
    med = median_filter(T, size=(MEDIAN_TICKS, 1), mode="nearest")
    spikes = (np.abs(T - med) > SPIKE_C).sum(0)
    P = d["V"] * np.abs(d["cur"]) * S.REG69_A
    Q = sim_heat(d)
    sec = np.floor(t).astype(np.int64)
    u, inv = np.unique(sec, return_inverse=True)
    cnt = np.bincount(inv)
    Tb = np.stack([np.bincount(inv, med[:, j]) / cnt for j in range(12)], 1)
    Pb = np.stack([np.bincount(inv, P[:, j]) / cnt for j in range(12)], 1)
    Qb = np.stack([np.bincount(inv, Q[:, j]) / cnt for j in range(12)], 1)
    bb = np.zeros(len(u), dtype=np.int64)
    bb[inv] = d["boot"]
    os.makedirs(os.path.dirname(BINS), exist_ok=True)
    np.savez_compressed(BINS, t=u.astype(float), T=Tb, P=Pb, Q=Qb, boot=bb, n=cnt, spikes=spikes, ticks=len(t),
                        raw_max=T.max(0), med_max=med.max(0))
    print(f"{len(t)} ticks -> {len(u)} s bins -> {BINS}")


def sim_heat(d):
    """Per tick and servo: the SIM's heat, K_CU tau^2 (W; sts_servo_model / supply_sag's bridge copper term,
    each servo by its type's K_CU), with tau from the logged PWM duty and speed through the sim's envelope
    exactly as supply_r.currents prices the bridge current (duty = -load, tau = stall(V) (d - w / w0(V)))."""
    import current_budget as CB
    import sts_servo_model as SM
    import supply_r as S
    import walker_env as W
    typ = [CB.servo_type(n) for n in LEGS]
    stall12 = np.array([SM.STALL_NM_12V * W.SERVO_TYPES[t][0] for t in typ])
    w012 = np.array([W._STS_NOLOAD_12V * W.SERVO_TYPES[t][1] for t in typ])
    kcu = np.array([CB.SERVO_ELEC[t][2] for t in typ])
    V = d["V"]
    duty = np.clip(-d["load"] / 1000.0, -1.0, 1.0)
    w = d["speed"] * S.STEP_RAD
    tau = stall12 * V / 12.0 * (duty - w / (w012 * V / 12.0))
    return kcu * tau ** 2


def load_bins():
    z = np.load(BINS)
    return {k: z[k] for k in z.files}


def segments(t, boot):
    """(start, end) index pairs of contiguous recording: a new segment at a > GAP_S gap or a boot change."""
    brk = np.r_[True, (np.diff(t) > GAP_S) | (np.diff(boot) != 0)]
    s = np.where(brk)[0]
    return list(zip(s, np.r_[s[1:], len(t)]))


def chunks(t, boot):
    out = []
    for s, e in segments(t, boot):
        k = s
        while k < e:
            j = k + np.searchsorted(t[k:e], t[k] + CHUNK_S)
            if j - k >= 120:                            # >= 2 min of data
                out.append((k, j))
            k = j
    return out


# ----------------------------------------------------------------------------- survey
def cmd_survey(a):
    B = load_bins()
    t, T, P = B["t"], B["T"], B["P"]
    print(f"{int(B['ticks'])} ticks, {len(t)} s ({len(t) / 3600:.1f} h of powered recording), "
          f"{_utc(t[0])} .. {_utc(t[-1])}Z\n")
    print("per servo: reg-63 spikes removed, max raw / max after the 1 s median, median, p99; mean / p99.9 / max "
          "heat P = V I (W)")
    for j, n in enumerate(LEGS):
        print(f"  {n:13s} {'3250' if n in STS3250 else '3215'} spikes {int(B['spikes'][j]):5d}  raw max "
              f"{B['raw_max'][j]:5.0f}  clean max {T[:, j].max():4.1f} med {np.median(T[:, j]):4.1f} "
              f"p99 {np.percentile(T[:, j], 99):4.1f}  P mean {P[:, j].mean():.3f} p99.9 "
              f"{np.percentile(P[:, j], 99.9):.2f} max {P[:, j].max():.2f}")
    print("\nper servo, the worst 10-minute mean heat inside one recording (W; V x reg 69 / the sim's K_CU tau^2), "
          "and when")
    Q = B["Q"]
    for j, n in enumerate(LEGS):
        best = (0.0, 0.0, None)
        for s, e in segments(t, B["boot"]):
            if e - s < 600:
                continue
            cp = np.r_[0.0, np.cumsum(P[s:e, j])]
            cq = np.r_[0.0, np.cumsum(Q[s:e, j])]
            mp = (cp[600:] - cp[:-600]) / 600.0
            k = int(np.argmax(mp))
            if mp[k] > best[0]:
                best = (float(mp[k]), float((cq[k + 600] - cq[k]) / 600.0), t[s + k])
        print(f"  {n:13s} {best[0]:.3f} / {best[1]:.3f} W  from {_utc(best[2]) if best[2] else '-'}Z")
    print("\nsegments (>= 3 min): start, duration, gap before, mean T at start / end, hottest servo, bus heat")
    prev = None
    for s, e in segments(t, B["boot"]):
        gap = (t[s] - prev) if prev is not None else float("inf")
        prev = t[e - 1]
        if t[e - 1] - t[s] < 180:
            continue
        j = int(np.argmax(T[s:e].max(0)))
        print(f"  {_utc(t[s])}Z {(t[e - 1] - t[s]) / 60:6.1f} min  gap {min(gap / 60, 99999):7.1f} min  "
              f"T {T[s:s + 10].mean():4.1f} -> {T[e - 10:e].mean():4.1f}  hot {LEGS[j]} {T[s:e, j].max():4.1f}  "
              f"P {P[s:e].sum(1).mean():.2f} W mean, {P[s:e].sum(1).max():.1f} max"
              f"{'  COLD START' if gap >= COLD_GAP_S else ''}")


# ----------------------------------------------------------------------------- fit
def _filt(P, a):
    """y[k] = a y[k-1] + (1 - a) P[k-1], y[0] = 0, per chunk (the load's response from rest)."""
    from scipy.signal import lfilter
    y = lfilter([0.0, 1.0 - a], [1.0, -a], P)
    return y


def _project(t_ch, a, T, y):
    """Remove each chunk's (a^k, 1 - a^k) span from T and y: returns the residualised pair (Frisch-Waugh)."""
    k = np.arange(len(T), dtype=float)
    X = np.column_stack([a ** k, 1.0 - a ** k])
    q, _ = np.linalg.qr(X)
    return T - q @ (q.T @ T), y - q @ (q.T @ y)


def fit_servo(T, P, ch, tau_grid=TAU_GRID, tau_load=None, idx=None):
    """Best (tau, Rth, rmse, sse) over the grid for one servo (T, P 1-D) or for several pooled with one common
    tau and Rth (T, P 2-D: a column per servo; each servo keeps its own T0, Tss per chunk). tau_load: a separate
    time constant for the load path (None: the same tau)."""
    idx = range(len(ch)) if idx is None else idx
    T2 = T[:, None] if T.ndim == 1 else T
    P2 = P[:, None] if P.ndim == 1 else P
    best = None
    for tau in tau_grid:
        a = math.exp(-1.0 / tau)
        al = a if tau_load is None else math.exp(-1.0 / tau_load)
        num = den = 0.0
        parts = []
        for i in idx:
            s, e = ch[i]
            for c in range(T2.shape[1]):
                Tr, yr = _project(None, a, T2[s:e, c], _filt(P2[s:e, c], al))
                parts.append((Tr, yr))
                num += yr @ Tr
                den += yr @ yr
        R = num / den if den > 0 else 0.0
        R = max(R, 0.0)
        sse = sum(float(((Tr - R * yr) ** 2).sum()) for Tr, yr in parts)
        n = sum(len(Tr) for Tr, _ in parts)
        if best is None or sse < best[3]:
            best = (tau, R, math.sqrt(sse / n), sse)
    return best


def cold_starts(B):
    t, T = B["t"], B["T"]
    out = []
    prev = None
    for s, e in segments(t, B["boot"]):
        gap = (t[s] - prev) if prev is not None else float("inf")
        prev = t[e - 1]
        if gap >= COLD_GAP_S and t[e - 1] - t[s] >= 25 * 60:
            out.append((s, e))
    return out


def cmd_fit(a):
    B = load_bins()
    t, T = B["t"], B["T"]
    P = B["Q"] if a.heat == "sim" else B["P"]
    ch = chunks(t, B["boot"])
    rng = np.random.default_rng(0)
    res = {"heat": a.heat, "chunks": len(ch), "seconds": int(sum(e - s for s, e in ch)), "servos": {}}
    print(f"{len(ch)} chunks, {res['seconds'] / 3600:.1f} h")
    # cold starts: the standby rise and the time constant from the power-on step alone (P ~ 0)
    cs = cold_starts(B)
    res["cold_starts"] = [_utc(t[s]) for s, _ in cs]
    print("cold starts:", ", ".join(res["cold_starts"]))
    for j, n in enumerate(LEGS):
        tau, R, rmse, _ = fit_servo(T[:, j], P[:, j], ch)
        r = dict(type="sts3250" if n in STS3250 else "sts3215", tau_s=tau, rth_k_per_w=R, rmse_c=rmse)
        # the power-on step: fit T0 + dT0 (1 - exp(-t / tau)) on each cold start's first 25 min .. whole segment
        steps = []
        for s, e in cs:
            tt = t[s:e] - t[s]
            y = T[s:e, j]
            bestc = None
            for tc in TAU_GRID:
                X = np.column_stack([np.ones_like(tt), 1.0 - np.exp(-tt / tc)])
                c, *_ = np.linalg.lstsq(X, y, rcond=None)
                sse = float(((X @ c - y) ** 2).sum())
                if bestc is None or sse < bestc[0]:
                    bestc = (sse, tc, c[0], c[1])
            steps.append(dict(start=_utc(t[s]), tau_s=bestc[1], t0=round(bestc[2], 2), rise=round(bestc[3], 2)))
        r["cold"] = steps
        r["cold_tau_median_s"] = float(np.median([x["tau_s"] for x in steps]))
        r["cold_rise_median_c"] = float(np.median([x["rise"] for x in steps]))
        if a.boot:
            bs = []
            for _ in range(a.boot):
                idx = rng.integers(0, len(ch), len(ch))
                bt, bR, _, _ = fit_servo(T[:, j], P[:, j], ch, tau_grid=TAU_GRID[::2], idx=idx)
                bs.append((bt, bR))
            bs = np.array(bs)
            r["tau_ci_s"] = [float(np.percentile(bs[:, 0], 2.5)), float(np.percentile(bs[:, 0], 97.5))]
            r["rth_ci"] = [float(np.percentile(bs[:, 1], 2.5)), float(np.percentile(bs[:, 1], 97.5))]
        res["servos"][n] = r
        print(f"{n:13s} {r['type']}  tau {tau / 60:5.1f} min  Rth {R:5.2f} K/W  rmse {rmse:.2f} C"
              + (f"  CI tau {r['tau_ci_s'][0] / 60:.1f}-{r['tau_ci_s'][1] / 60:.1f} min, Rth "
                 f"{r['rth_ci'][0]:.2f}-{r['rth_ci'][1]:.2f}" if a.boot else "")
              + f"   cold starts: tau {r['cold_tau_median_s'] / 60:.1f} min, standby rise "
                f"{r['cold_rise_median_c']:.1f} C")
    # one (tau, Rth) per servo type, pooled over its servos (what the sim's kernel takes)
    res["types"] = {}
    for typ in ("sts3215", "sts3250"):
        cols = [j for j, n in enumerate(LEGS) if (n in STS3250) == (typ == "sts3250")]
        tau, R, rmse, _ = fit_servo(T[:, cols], P[:, cols], ch)
        r = dict(servos=len(cols), tau_s=tau, rth_k_per_w=R, rmse_c=rmse)
        if a.boot:
            bs = []
            for _ in range(a.boot):
                idx = rng.integers(0, len(ch), len(ch))
                bt, bR, _, _ = fit_servo(T[:, cols], P[:, cols], ch, tau_grid=TAU_GRID[::2], idx=idx)
                bs.append((bt, bR))
            bs = np.array(bs)
            r["tau_ci_s"] = [float(np.percentile(bs[:, 0], 2.5)), float(np.percentile(bs[:, 0], 97.5))]
            r["rth_ci"] = [float(np.percentile(bs[:, 1], 2.5)), float(np.percentile(bs[:, 1], 97.5))]
        res["types"][typ] = r
        print(f"pooled {typ} ({len(cols)} servos): tau {tau / 60:.1f} min, Rth {R:.2f} K/W, rmse {rmse:.2f} C"
              + (f"  CI tau {r['tau_ci_s'][0] / 60:.1f}-{r['tau_ci_s'][1] / 60:.1f} min, Rth "
                 f"{r['rth_ci'][0]:.2f}-{r['rth_ci'][1]:.2f}" if a.boot else ""))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
        print("->", a.json)


# ----------------------------------------------------------------------------- predict
def minutes_to(t_start, t_ss, tau_s, limit=T_CUTOFF):
    """Minutes for a first-order node from t_start toward t_ss to reach `limit` (inf if t_ss <= limit)."""
    if t_ss <= limit:
        return math.inf
    return tau_s / 60.0 * math.log((t_ss - t_start) / (t_ss - limit))


def cmd_predict(a):
    import sts_servo_model as SM
    import walker_env as W
    with open(a.json) as f:
        fit = json.load(f)
    print(f"cutoff {T_CUTOFF} C; start = idle equilibrium {a.t_idle} C (ambient {a.ambient} C); heat = K_CU tau^2 "
          f"(the sim's), load as a fraction of the 12 V stall held continuously (duty-averaged)")
    for typ, stall in (("sts3215", SM.STALL_NM_12V), ("sts3250", SM.STALL_NM_12V * W.SERVO_TYPES["sts3250"][0])):
        import current_budget as CB
        kcu = CB.SERVO_ELEC[typ][2]
        rows = [v for v in fit["servos"].values() if v["type"] == typ]
        taus = [v["tau_s"] for v in rows]
        Rs = [v["rth_k_per_w"] for v in rows]
        Rhi = max(v.get("rth_ci", [0, v["rth_k_per_w"]])[1] for v in rows)
        print(f"\n{typ}: tau {min(taus) / 60:.1f}-{max(taus) / 60:.1f} min, Rth fitted {min(Rs):.2f}-{max(Rs):.2f} "
              f"K/W (CI top {Rhi:.2f})")
        for f in (0.1, 0.2, 0.3, 0.5, 0.65, 0.8, 1.0):
            Pw = kcu * (f * stall) ** 2
            for R in sorted({max(Rs), Rhi, a.rth_prior}):
                tss = a.t_idle + R * Pw
                m = minutes_to(a.t_idle, tss, max(taus))
                print(f"  {f:4.0%} of stall: {Pw:5.1f} W  Rth {R:5.2f}: steady {tss:6.1f} C  -> 70 C "
                      f"{'never' if math.isinf(m) else f'in {m:.1f} min'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("bins")
    sp.add_parser("survey")
    f = sp.add_parser("fit")
    f.add_argument("--boot", type=int, default=0)
    f.add_argument("--heat", choices=("sim", "reg69"), default="sim",
                   help="the heat regressor: sim = K_CU tau^2 from the logged duty and speed (the sim's own "
                        "heat, so the fitted Rth pairs with it); reg69 = V x register 69")
    f.add_argument("--json")
    p = sp.add_parser("predict")
    p.add_argument("--json", default=os.path.join(HERE, "fit.json"))
    p.add_argument("--t-idle", type=float, default=31.0)
    p.add_argument("--ambient", type=float, default=22.0)
    p.add_argument("--rth-prior", type=float, default=8.0)
    a = ap.parse_args()
    {"bins": cmd_bins, "survey": cmd_survey, "fit": cmd_fit, "predict": cmd_predict}[a.cmd](a)


if __name__ == "__main__":
    main()
