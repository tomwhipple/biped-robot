#!/usr/bin/env python3
"""Where the control-loop stall lands on the weight shift, per stance side, and the snap split by it (#259, #297).

    python stall_split.py robot CSV [CSV ...]                  # tip_causes.py table CSVs (robot rows)
    python stall_split.py sim   CSV [CSV ...] [--side R]       # snap_replay.py run CSVs (robot + plant rows)

Round 7 (docs/design-v6/2026-11-25-weight-shift-snap-sim.md) found a ~115 ms stall of the firmware's control loop
0.2-0.5 s after every touchdown (the host's per-step `imu` poll), and that a stall on the shift's fast part makes the
weight-shift snap. Its stall statistics came from throwaway scripts. This tool keeps them, per side:

- A stall is a gap of more than GAP_S between two ticks of the record (tip_causes.py's cache, build/tip-causes/).
- Per single support (a tip_causes row; the double support before it is td_prev = lift_utc - ds_s to lift_utc) the
  stall is the longest gap that ends inside that double support. `stall_rel` is its end minus the shift's midpoint
  (td_prev + shift_t50), s; `stall_len` its length.
- "On the fast part" is FAST_LO <= stall_rel <= FAST_HI: tip_causes.py's lag window (t50 - 0.35 .. t50 + 0.15 s).
- The snap is tip_causes.SNAP_DEG on ds_roll_peak.

`robot` prints, per side and timing, the snap with and without a stall on the fast part (Fisher p, Wilson 95 % CIs),
the snap by stall_rel bins, the stall's timing after the touchdown and against the shift, and the shift itself
(amplitude, t50, peak goal rate). `sim` joins snap_replay's plant rows to its robot rows by (stamp, n) and prints
the same split for each plant and variant, with the per-step correlation of ds_roll_peak against the robot's.
Nothing is written; the stall columns come from the caches.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tip_causes as TC  # noqa: E402

GAP_S = 0.08
FAST_LO, FAST_HI = -0.35, 0.15
BINS = ((-9.0, -0.80), (-0.80, -0.50), (-0.50, -0.35), (-0.35, -0.20), (-0.20, -0.05), (-0.05, 0.15), (0.15, 9.0))


# ------------------------------------------------------------------------------------------------
# pure parts (tests/test_stall_split.py)
# ------------------------------------------------------------------------------------------------
def tick_gaps(t, gap_s=GAP_S):
    """(start, end) arrays of the gaps longer than gap_s between consecutive ticks t (s, sorted)."""
    t = np.asarray(t, float)
    if len(t) < 2:
        return np.zeros(0), np.zeros(0)
    d = np.diff(t)
    i = np.where(d > gap_s)[0]
    return t[i], t[i + 1]


def stall_in_ds(starts, ends, td, tl):
    """The longest gap ending inside the double support (td, tl]: (start, end) or None."""
    m = (ends > td) & (ends <= tl)
    if not m.any():
        return None
    j = np.where(m)[0][np.argmax((ends - starts)[m])]
    return float(starts[j]), float(ends[j])


def on_fast_part(rel, lo=FAST_LO, hi=FAST_HI):
    return bool(np.isfinite(rel) and lo <= rel <= hi)


def wilson(k, n, z=1.96):
    """Wilson 95 % interval of k / n, (lo, hi); (nan, nan) for n == 0."""
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def fisher_p(a, b, c, d):
    """Two-sided Fisher exact p of [[a, b], [c, d]]."""
    from scipy import stats
    return float(stats.fisher_exact([[a, b], [c, d]])[1])


# ------------------------------------------------------------------------------------------------
# the record
# ------------------------------------------------------------------------------------------------
_GAPS = {}


def gaps_for(stamp):
    if stamp not in _GAPS:
        p = os.path.join(TC.CACHE, f"{stamp}.npz")
        _GAPS[stamp] = tick_gaps(np.load(p)["a"][:, 0]) if os.path.exists(p) else (np.zeros(0), np.zeros(0))
    return _GAPS[stamp]


def add_stall(r):
    """r (a tip_causes / snap_replay row, numbers parsed) with stall_rel, stall_len, stall_td (end after the
    touchdown), stall_fast (1 / 0) added; NaN / 0 without a double support or a stall in it."""
    r = dict(r)
    for k in ("stall_rel", "stall_len", "stall_td"):
        r[k] = float("nan")
    r["stall_fast"] = 0
    tl, ds, t50 = (r.get(k, float("nan")) for k in ("lift_utc", "ds_s", "shift_t50"))
    if not (np.isfinite(tl) and np.isfinite(ds)):
        return r
    td = tl - ds
    st = stall_in_ds(*gaps_for(r["stamp"]), td, tl)
    if st is None:
        return r
    r["stall_len"] = st[1] - st[0]
    r["stall_td"] = st[1] - td
    if np.isfinite(t50):
        r["stall_rel"] = st[1] - (td + t50)
        r["stall_fast"] = int(on_fast_part(r["stall_rel"]))
    return r


def snapped(r, key="ds_roll_peak"):
    return float(r.get(key, float("nan"))) >= TC.SNAP_DEG


def early(r):
    return float(r.get("shift_t10", float("nan"))) < TC.LAG_S


def pct(k, n):
    lo, hi = wilson(k, n)
    return f"{k:4d}/{n:<4d} {100 * k / max(n, 1):5.1f} % [{100 * lo:4.1f}-{100 * hi:4.1f}]"


def q(rows, k, ps=(10, 50, 90)):
    v = np.array([float(r.get(k, np.nan)) for r in rows], float)
    v = v[np.isfinite(v)]
    return "/".join(f"{x:+.2f}" for x in np.percentile(v, ps)) if len(v) else "n/a"


def split(rows, label, key="ds_roll_peak"):
    f = [r for r in rows if r["stall_fast"]]
    o = [r for r in rows if not r["stall_fast"]]
    a, c = sum(snapped(r, key) for r in f), sum(snapped(r, key) for r in o)
    p = fisher_p(a, len(f) - a, c, len(o) - c) if f and o else float("nan")
    tot = sum(snapped(r, key) for r in rows)
    print(f"  {label:34s} all {pct(tot, len(rows))} | stall on fast part {pct(a, len(f))} | "
          f"not {pct(c, len(o))} | Fisher p {p:.1e}")


def robot(paths):
    rows = []
    for p in paths:
        rows += TC.read_csv(p)
    rows = [add_stall(r) for r in rows if r.get("k", 0) >= 2 and np.isfinite(r.get("ds_roll_peak", np.nan))]
    print(f"{len(rows)} single supports (k >= 2); stall = longest tick gap > {GAP_S * 1000:.0f} ms ending in the double "
          f"support; fast part = t50 {FAST_LO:+.2f} .. {FAST_HI:+.2f} s; snap >= {TC.SNAP_DEG} deg")
    for timing in ("normal", "fast"):
        for start in ("early", "late"):
            print(f"\n{timing} timing, {start} shift start (t10 {'<' if start == 'early' else '>='} {TC.LAG_S} s)")
            for side in "LR":
                b = [r for r in rows if r["timing"] == timing and r["side"] == side and early(r) == (start == "early")]
                if not b:
                    continue
                split(b, f"{side} stances")
                ns = sum(np.isfinite(r["stall_rel"]) for r in b)
                print(f"      stall found in {ns}/{len(b)}; ends after td p10/50/90 {q(b, 'stall_td')} s; "
                      f"length {q(b, 'stall_len')} s; rel t50 {q(b, 'stall_rel')} s")
                print(f"      shift: t10 {q(b, 'shift_t10')}  t50 {q(b, 'shift_t50')}  t90 {q(b, 'shift_t90')} s; "
                      f"goal rate at the fast part {q(b, 'goal_rate_mid')} deg/s; lag stance hip "
                      f"{q(b, 'lag_mid_' + side + '_hip')} deg")
                cells = []
                for lo, hi in BINS:
                    c = [r for r in b if lo <= r["stall_rel"] < hi]
                    if c:
                        cells.append(f"[{lo:+.2f},{hi:+.2f}) {sum(map(snapped, c))}/{len(c)}")
                print("      snap by stall end - t50: " + "  ".join(cells))
    return rows


def sim(paths, side="R"):
    rows = []
    for p in paths:
        rows += TC.read_csv(p)
    rob = {(r["stamp"], int(r["n"])): add_stall(r) for r in rows if r["plant"] == "robot"}
    keys = sorted({(r["plant"], str(r["delay"])) for r in rows if r["plant"] != "robot"})
    for timing in ("normal", "fast"):
        for start in ("early", "late"):
            rr = [r for r in rob.values() if r["side"] == side and r["timing"] == timing and early(r) == (start == "early")]
            if not rr:
                continue
            print(f"\n{side} stances, {timing} timing, {start} shift start (stall columns from the robot's record)")
            split(rr, "robot")
            for lab, dl in keys:
                pairs = []
                for r in rows:
                    if r["plant"] != lab or str(r["delay"]) != dl or r["side"] != side:
                        continue
                    ro = rob.get((r["stamp"], int(r["n"])))
                    if ro is None or ro["timing"] != timing or early(ro) != (start == "early"):
                        continue
                    pairs.append((ro, dict(r, stall_fast=ro["stall_fast"], stall_rel=ro["stall_rel"])))
                if not pairs:
                    continue
                split([s for _, s in pairs], f"{lab} {dl}")
                for lab2, sel in (("all", pairs), ("fast", [p for p in pairs if p[0]["stall_fast"]]),
                                  ("not", [p for p in pairs if not p[0]["stall_fast"]])):
                    pr = np.array([float(a["ds_roll_peak"]) for a, _ in sel])
                    ps = np.array([float(b["ds_roll_peak"]) for _, b in sel])
                    ok = np.isfinite(pr) & np.isfinite(ps)
                    if ok.sum() < 5:
                        continue
                    rr_ = np.corrcoef(pr[ok], ps[ok])[0, 1]
                    sr, ss = pr[ok] >= TC.SNAP_DEG, ps[ok] >= TC.SNAP_DEG
                    print(f"      {lab2:4s} n {ok.sum():4d} r {rr_:+.2f}  peak rob/sim {np.median(pr[ok]):.2f}/"
                          f"{np.median(ps[ok]):.2f}  both {np.sum(sr & ss)} rob-only {np.sum(sr & ~ss)} "
                          f"sim-only {np.sum(~sr & ss)} neither {np.sum(~sr & ~ss)}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["robot", "sim"])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--side", default="R")
    a = ap.parse_args(argv)
    if a.what == "robot":
        robot(a.csv)
    else:
        sim(a.csv, a.side)


if __name__ == "__main__":
    main()
