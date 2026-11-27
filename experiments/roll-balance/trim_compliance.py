#!/usr/bin/env python3
"""The left hip's trim: the two signatures of the 2026-11-27 right-stance record, per plant and variant (#259).

    python trim_compliance.py CSV [CSV ...] [--pair 'grade:0|grade:c0.5@L_hip_roll|0.5' ...]
                         # snap_replay.py run CSVs (robot rows from a delay-0 run), --side LR (or L / R)

docs/design-v6/2026-11-27-right-stance-snaps.md found two things the plants miss, both pointing at how the left side
takes the large hip-roll trim. This tool scores a snap_replay.py CSV on both, and on the left-side result `grade`
already gets right, on the same steps as the robot (normal timing, early shift start, k >= 2; the stall columns from
the robot's record, stall_split.add_stall):

- Right stances: the snap share (all / stall on the fast part / not), the mean ds_roll_peak (stall / not /
  increment), how far the plant sits under the robot on the stall steps, r per step, and the slope of ds_roll_peak on
  the previous left stance's hike at its touchdown (prev_hike_td, the robot's; walk fixed effects) on the stall steps
  and on the others (robot: +0.48 / +0.61 deg per deg). The previous left stance's error at its touchdown (prev_e_td:
  the plant's own on a plant row) and its slope on that hike (on the robot the two are collinear); the stall steps'
  roll on the hike and that error together, and on the hike's two parts together: the learned T L (T_other) and the
  rest, hike - T L (the trim's answer to the lean at that moment).
- Both sides: the single support's lean toward the swing foot and the tips.
- Left stances: the stall increment (robot +1.97), the snap share (robot 53 %), r per step (grade 0.93), the plant's
  tips (lean_sim >= 7 deg or a fall) on the robot's tips and on clean steps (false), and the slope of the plant's lean
  (lean_sim) on the robot's T at the lift-off (T_lift; walk fixed effects). Robot (its own lean_in): +0.02.

Nothing is written. Variants are keyed as snap_replay.vkey writes them.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stall_split as SS  # noqa: E402
import tip_causes as TC  # noqa: E402


# ------------------------------------------------------------------------------------------------
# pure parts (tests/test_trim_compliance.py)
# ------------------------------------------------------------------------------------------------
def vkey(v):
    """snap_replay.vkey without importing the sim: '0', '0.3', 'c0.5@L_hip_roll' ..."""
    v = str(v)
    try:
        return "0" if float(v) == 0 else f"{float(v):g}"
    except ValueError:
        return v


def walk_slope(ys, xs, walks):
    """The walk-fixed-effects slope of ys on xs (TC.ols on one regressor): (slope, se, n)."""
    rows = [dict(stamp=w, y=y, x=x) for y, x, w in zip(ys, xs, walks)]
    b, se, _, n = TC.ols(rows, "y", ["x"])
    return float(b[0]), float(se[0]), n


def tip_counts(sim_tip, rob_tip):
    """(the plant's tips, those on the robot's tips, those on the robot's clean steps)."""
    s, r = np.asarray(sim_tip, bool), np.asarray(rob_tip, bool)
    return int(s.sum()), int((s & r).sum()), int((s & ~r).sum())


def mean_or_nan(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(v.mean()) if len(v) else float("nan")


# ------------------------------------------------------------------------------------------------
# the scoring
# ------------------------------------------------------------------------------------------------
def pairs_by_key(rows, side):
    """{(plant, variant): [(robot row, plant row)]} on the selected steps; the robot's own pairs under
    ('robot', '0')."""
    rob = {}
    for r in rows:
        if r["plant"] != "robot" or r["side"] != side or r["timing"] != "normal" or r.get("k", 0) < 2:
            continue
        if not (SS.early(r) and np.isfinite(r.get("ds_roll_peak", np.nan))):
            continue
        rob[(r["stamp"], int(r["n"]))] = SS.add_stall(r)
    out = {("robot", "0"): [(ro, ro) for ro in rob.values()]}
    for r in rows:
        if r["plant"] == "robot" or r["side"] != side:
            continue
        ro = rob.get((r["stamp"], int(r["n"])))
        if ro is not None:
            out.setdefault((r["plant"], vkey(r["delay"])), []).append((ro, r))
    return out


def score(pairs, side):
    """The metrics of one plant / variant on one side (module doc)."""
    ro = [a for a, _ in pairs]
    si = [b for _, b in pairs]
    fast = np.array([a["stall_fast"] for a in ro], bool)
    pr = np.array([a["ds_roll_peak"] for a in ro], float)
    ps = np.array([b["ds_roll_peak"] for b in si], float)
    walks = [a["stamp"] for a in ro]
    m = dict(n=len(pairs), n_fast=int(fast.sum()))
    snap = ps >= TC.SNAP_DEG
    m["snap"] = 100 * snap.mean()
    m["snap_fast"] = 100 * snap[fast].mean() if fast.any() else np.nan
    m["snap_not"] = 100 * snap[~fast].mean() if (~fast).any() else np.nan
    m["mean_fast"], m["mean_not"] = mean_or_nan(ps[fast]), mean_or_nan(ps[~fast])
    m["incr"] = m["mean_fast"] - m["mean_not"]
    m["short_fast"] = mean_or_nan(pr[fast] - ps[fast])
    ok = np.isfinite(pr) & np.isfinite(ps)
    m["r"] = float(np.corrcoef(pr[ok], ps[ok])[0, 1]) if ok.sum() > 4 else np.nan
    okf = ok & fast
    m["r_fast"] = float(np.corrcoef(pr[okf], ps[okf])[0, 1]) if okf.sum() > 4 else np.nan
    if side == "R":
        h = np.array([a.get("prev_hike_td", np.nan) for a in ro], float)
        for lab, sel in (("fast", fast), ("not", ~fast)):
            idx = np.where(sel)[0]
            b, se, n = walk_slope(ps[idx], h[idx], [walks[i] for i in idx])
            m[f"hike_slope_{lab}"], m[f"hike_se_{lab}"] = b, se
        # the previous left stance's state at its touchdown, the plant's own (its error toward the right foot):
        # on the robot it is collinear with the hike
        e = np.array([b_.get("prev_e_td", np.nan) for b_ in si], float)
        m["etd_mean"] = mean_or_nan(e)
        m["etd_slope_hike"], m["etd_se_hike"], _ = walk_slope(e, h, walks)
        # the right double support's roll on both together (stall steps): the hike's own share and the carry-over of
        # the left touchdown's state
        idx = np.where(fast)[0]
        rr = [dict(stamp=walks[i], y=ps[i], h=h[i], e=e[i]) for i in idx]
        b, se, _, _ = TC.ols(rr, "y", ["h", "e"]) if len(rr) > 5 else (np.full(2, np.nan), np.full(2, np.nan), 0, 0)
        m["part_h"], m["part_e"], m["part_h_se"], m["part_e_se"] = float(b[0]), float(b[1]), float(se[0]), float(se[1])
        # the hike split into the learned T L (T_other: the trim's slow part) and the rest u = hike - T L (the trim's
        # answer to the lean then, kp s e + ki): the stall steps' roll on both together
        rr = [dict(stamp=walks[i], y=ps[i], T=ro[i].get("T_other", np.nan), u=h[i] - ro[i].get("T_other", np.nan))
              for i in idx]
        b, se, _, _ = TC.ols(rr, "y", ["T", "u"]) if len(rr) > 5 else (np.full(2, np.nan), np.full(2, np.nan), 0, 0)
        m["on_T"], m["on_u"], m["on_T_se"], m["on_u_se"] = float(b[0]), float(b[1]), float(se[0]), float(se[1])
    # the single support's lean toward the swing foot (TS lean_in; the plant's lean_sim) and the tips, both sides
    if si and si[0]["plant"] == "robot":
        lean = np.array([a.get("lean_in", np.nan) for a in ro], float)
        tips = np.array([a["tip"] for a in ro], bool)
    else:
        lean = np.array([b.get("lean_sim", np.nan) for b in si], float)
        tips = np.array([b.get("tip_sim", 0) for b in si], float) > 0
    T = np.array([a.get("T_lift", np.nan) for a in ro], float)
    m["lean_slope_T"], m["lean_se_T"], _ = walk_slope(lean, T, walks)
    m["lean_mean"] = mean_or_nan(lean)
    m["tips"], m["tips_hit"], m["tips_false"] = tip_counts(tips, [a["tip"] for a in ro])
    return m


def report(paths):
    rows = []
    for p in paths:
        rows += TC.read_csv(p)
    sides = sorted({r["side"] for r in rows})
    for side in sides:
        pk = pairs_by_key(rows, side)
        keys = sorted(pk, key=lambda k: (k[0] != "robot", k[0], k[1]))
        print(f"\n{side} stances, normal timing, early shift start, k >= 2 (stall from the robot's record)")
        if side == "R":
            print(f"  {'plant':<10} {'variant':<18} {'n':>4} {'snap%':>6} {'fast%':>6} {'not%':>6} {'m_fast':>7} "
                  f"{'m_not':>6} {'incr':>6} {'short':>6} {'r':>6} {'r_fast':>6} {'hike/fast':>14} {'hike/not':>14}")
            print(f"  {'':<29} then: e_td (the left touchdown's error) mean, its slope on the hike; the stall steps'"
                  f" roll on hike + e_td together")
        else:
            print(f"  {'plant':<10} {'variant':<18} {'n':>4} {'snap%':>6} {'fast%':>6} {'not%':>6} {'m_fast':>7} "
                  f"{'m_not':>6} {'incr':>6} {'short':>6} {'r':>6} {'tips':>5} {'hit':>4} {'false':>5} "
                  f"{'lean':>6} {'lean/T':>14}")
        for k in keys:
            m = score(pk[k], side)
            head = (f"  {k[0]:<10} {k[1]:<18} {m['n']:>4} {m['snap']:>6.1f} {m['snap_fast']:>6.1f} "
                    f"{m['snap_not']:>6.1f} "
                    f"{m['mean_fast']:>7.2f} {m['mean_not']:>6.2f} {m['incr']:>+6.2f} {m['short_fast']:>+6.2f} "
                    f"{m['r']:>6.2f}")
            if side == "R":
                print(head + f" {m['r_fast']:>6.2f} {m['hike_slope_fast']:>+7.2f}±{m['hike_se_fast']:.2f} "
                      f"{m['hike_slope_not']:>+7.2f}±{m['hike_se_not']:.2f}")
                print(f"  {'':<29} e_td {m['etd_mean']:>+6.2f}, on hike {m['etd_slope_hike']:>+6.2f}"
                      f"±{m['etd_se_hike']:.2f}; roll = {m['part_h']:+.2f}±{m['part_h_se']:.2f} hike "
                      f"{m['part_e']:+.2f}±{m['part_e_se']:.2f} e_td;"
                      f" lean {m['lean_mean']:.2f}, tips {m['tips']} (false {m['tips_false']})")
                print(f"  {'':<29} roll = {m['on_T']:+.2f}±{m['on_T_se']:.2f} T L {m['on_u']:+.2f}±{m['on_u_se']:.2f}"
                      f" (hike - T L)")
            else:
                print(head + f" {m['tips']:>5d} {m['tips_hit']:>4d} {m['tips_false']:>5d} {m['lean_mean']:>6.2f} "
                      f"{m['lean_slope_T']:>+7.2f}±{m['lean_se_T']:.2f}")


def origin_slope(y, x):
    """The least-squares slope of y on x through the origin (NaNs dropped)."""
    y, x = np.asarray(y, float), np.asarray(x, float)
    ok = np.isfinite(y) & np.isfinite(x)
    return float(np.sum(x[ok] * y[ok]) / np.sum(x[ok] ** 2)) if ok.any() else float("nan")


def paired(rows, a, b, frac, min_hike=1.0):
    """The trim's authority on a plant, from two variants of the same steps (right stances): a = (plant, variant)
    with the left hip's correction in full, b = the same with it scaled down by `frac` (c<1-frac>@L_hip_roll). Per
    degree of the removed hike at the left touchdown (frac x prev_hike_td > min_hike): the change of the left
    touchdown's error (prev_e_td, the plant's) and of the right double support's roll (ds_roll_peak), all steps / stall
    steps / the others. -> dict."""
    pk = pairs_by_key(rows, "R")
    da = {(x["stamp"], int(x["n"])): (x, y) for x, y in pk.get(a, [])}
    db = {(x["stamp"], int(x["n"])): y for x, y in pk.get(b, [])}
    ks = [k for k in da if k in db]
    h = np.array([da[k][0].get("prev_hike_td", np.nan) for k in ks], float) * frac
    de = np.array([da[k][1].get("prev_e_td", np.nan) - db[k].get("prev_e_td", np.nan) for k in ks], float)
    dr = np.array([da[k][1]["ds_roll_peak"] - db[k]["ds_roll_peak"] for k in ks], float)
    fast = np.array([da[k][0]["stall_fast"] for k in ks], bool)
    ok = np.isfinite(h) & (h > min_hike)
    return dict(n=int(ok.sum()), e_td=origin_slope(de[ok], h[ok]), roll=origin_slope(dr[ok], h[ok]),
                roll_fast=origin_slope(dr[ok & fast], h[ok & fast]),
                roll_not=origin_slope(dr[ok & ~fast], h[ok & ~fast]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--pair", action="append", default=[],
                    help="'PLANT:VARIANT|PLANT:VARIANT2|FRAC' (paired(): e.g. 'grade:0|grade:c0.5@L_hip_roll|0.5')")
    a = ap.parse_args(argv)
    report(a.csv)
    if a.pair:
        rows = []
        for p in a.csv:
            rows += TC.read_csv(p)
        print("\nthe left hip's trim authority, paired steps (right stances): per deg of the removed hike at the left "
              "touchdown, the change of the left touchdown's error and of the right double support's roll")
        for spec in a.pair:
            x, y, f = spec.split("|")
            m = paired(rows, tuple(x.split(":", 1)), tuple(y.split(":", 1)), float(f))
            print(f"  {x} vs {y} (x {f}): n {m['n']}, e_td {m['e_td']:+.2f}, roll {m['roll']:+.2f} "
                  f"(stall steps {m['roll_fast']:+.2f}, others {m['roll_not']:+.2f})")


if __name__ == "__main__":
    main()
