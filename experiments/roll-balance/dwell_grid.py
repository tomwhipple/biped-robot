"""The dwell sway (dwell.py) through the firmware's track + trim in the SIL (sil_gait.run, model 'dwell'): which
step times hold with the robot's static shift requirement as a hard constraint, and with how much double
support.

Per (T, ds): falls, trim aborts, lifts, the TS lean, the swing sole's clearance, and where the sim's CoM is
when the foot comes up against where it sits mid single support (`arrive`, mm: + = still short of it at
lift-off, i.e. the shift has not arrived; - = past it). The sim is optimistic on right stance in absolute
terms (fw_replay.py), so feasibility is read from the RELATIVE numbers: the CoM has arrived before the lift
(|arrive| small), the lean stays small, nothing falls.

    python dwell_grid.py sweep      # T x ds, fit / hi / lo / hard, 6 directions, no extra delay
    python dwell_grid.py final --ds 4:0.4,3:0.4,2.5:0.45,2:0.5     # the chosen split, 0 / 60 ms
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sil_gait as G  # noqa: E402

SEED = (5.2, 3.4)


def one(a):
    T, ds, d, plant, delay = a
    try:
        r = G.run(T, d, plant, delay, 6, seed=SEED, model="dwell", gkw=(dict(ds=ds) if ds is not None else {}))
        ph = r["phases"]
        arr = [p["com_in_lo_mm"] - p["com_in_mid_mm"] for p in ph
               if p["com_in_lo_mm"] is not None and p["com_in_mid_mm"] is not None]
        return dict(T=T, ds=ds, dir=d, plant=plant, delay=delay, fell=r["fell"], fall_dir=r["fall_dir"],
                    lifts=r["lifts"], n=r["n_steps"], aborts=r["aborts"], peak_tilt=r["peak_tilt"],
                    lean_in=[x["lean_in"] for x in r["reports"]], lean_out=[x["lean_out"] for x in r["reports"]],
                    T_mid={s: [x["T_mid"] for x in r["reports"] if x["stance"] == s] for s in "LR"},
                    hike_mid={s: [x["hike_mid"] for x in r["reports"] if x["stance"] == s] for s in "LR"},
                    reverted=sum(x.get("reverted", False) for x in r["reports"]),
                    clr=[p["clr_max_mm"] for p in ph], arrive=arr,
                    com_lo={s: [p["com_in_lo_mm"] for p in ph if p["stance"] == s and p["com_in_lo_mm"] is not None]
                            for s in "LR"})
    except Exception as e:  # noqa: BLE001
        return dict(args=list(a), error=repr(e)[:300])


def table(rows, key=("T", "ds")):
    ok = [r for r in rows if "error" not in r]
    groups = sorted({tuple(r[k] for k in key) for r in ok}, key=lambda g: (-g[0], g[1] or 0) if len(g) > 1 else (-g[0],))
    lines = ["T s   ds    runs falls aborts  lifts      TS lean in med/max  out max  clearance med/p10 mm  "
             "CoM arrive at lift-off med/worst mm  T mid L/R  hike mid L/R  T put back"]
    for g in groups:
        rs = [r for r in ok if tuple(r[k] for k in key) == g]
        li = [x for r in rs for x in r["lean_in"]] or [0.0]
        lo = [x for r in rs for x in r["lean_out"]] or [0.0]
        cl = [x for r in rs for x in r["clr"]] or [0.0]
        ar = [x for r in rs for x in r["arrive"]] or [0.0]
        tm = {s: [x for r in rs for x in r["T_mid"][s]] or [float("nan")] for s in "LR"}
        hm = {s: [x for r in rs for x in r["hike_mid"][s]] or [float("nan")] for s in "LR"}
        n = sum(r["n"] for r in rs)
        lines.append(
            f"{g[0]:<5g} {str(g[1]) if len(g) > 1 and g[1] is not None else 'sched':<5} {len(rs):4d} {sum(r['fell'] for r in rs):5d} "
            f"{sum(r['aborts'] for r in rs):6d}  {sum(r['lifts'] for r in rs):3d}/{n:<3d} ({100 * sum(r['lifts'] for r in rs) / max(n, 1):3.0f}%)  "
            f"{np.median(li):4.1f} / {max(li):4.1f}       {max(lo):4.1f}    {np.median(cl):4.1f} / {np.percentile(cl, 10):4.1f}"
            f"             {np.median(ar):+5.1f} / {max(ar, key=abs):+5.1f}               "
            f"{np.median(tm['L']):4.1f}/{np.median(tm['R']):4.1f}  {np.median(hm['L']):4.1f}/{np.median(hm['R']):4.1f}"
            f"     {sum(r['reverted'] for r in rs)}")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["sweep", "final", "schedule"])
    ap.add_argument("--T", default="4,3,2.5,2,1.75")
    ap.add_argument("--dsl", default="0.3,0.4,0.5,0.6")
    ap.add_argument("--ds", default="4:0.4,3:0.4,2.5:0.45,2:0.5", help="final: T:ds pairs")
    ap.add_argument("--plants", default="fit,lo,hi,hard")
    ap.add_argument("--dirs", default=",".join(G.DIRS))
    ap.add_argument("--workers", type=int, default=16)
    a = ap.parse_args(argv)
    if a.mode == "sweep":
        jobs = [(float(T), float(ds), d, p, 0.0) for T in a.T.split(",") for ds in a.dsl.split(",")
                for d in a.dirs.split(",") for p in a.plants.split(",")]
        out = os.path.join(HERE, "results", "dwell_sweep.json")
    elif a.mode == "schedule":
        import dwell as DW
        jobs = [(T, None, d, p, dl) for T in sorted(DW.SCHEDULE, reverse=True) for d in a.dirs.split(",")
                for p in a.plants.split(",") for dl in (0.0, 60.0)]
        out = os.path.join(HERE, "results", "dwell_schedule.json")
    else:
        pairs = [tuple(float(x) for x in kv.split(":")) for kv in a.ds.split(",")]
        jobs = [(T, ds, d, p, dl) for T, ds in pairs for d in a.dirs.split(",") for p in a.plants.split(",")
                for dl in (0.0, 60.0)]
        out = os.path.join(HERE, "results", "dwell_final.json")
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(one, jobs))
    with open(out, "w") as f:
        json.dump(rows, f)
    errs = [r for r in rows if "error" in r]
    print(f"{len(rows) - len(errs)} runs, {len(errs)} errors", errs[:1])
    print(table(rows))
    falls = [r for r in rows if "error" not in r and r["fell"]]
    for r in falls[:12]:
        print("  fell:", r["T"], r["ds"], r["dir"], r["plant"], r["delay"], r["fall_dir"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
