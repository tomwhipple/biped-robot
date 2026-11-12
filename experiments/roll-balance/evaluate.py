"""Evaluate balance laws over runs x plants x delays x speeds (testbed.run), in parallel.

    python evaluate.py --laws NAME[,NAME] --runs A,B --plants P;Q --delays 0,30,60 --speeds 1,2 --out results/x
Laws are named in LAWS below. Writes <out>.csv (one row per case) and <out>.md (per law: lifts, falls IN/OUT,
peak lean, margin), and prints the summary.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import laws as Lw  # noqa: E402
import testbed as T  # noqa: E402

LAWS = {
    "off": lambda: None,
    "stepper": "logged",                                        # the stepper's own trims, as logged (no law)
    "oldPI-through": lambda: Lw.OldPI("through"),
    "oldPI-armed": lambda: Lw.OldPI("armed"),
    "hipP": lambda: Lw.HipLevel(kp=0.5),
    "trim": lambda: Lw.Trim(kp=0.5, ki=1.0),
    "trim-kd": lambda: Lw.Trim(kp=0.5, kd=0.03, ki=1.0),
    "trim-hi": lambda: Lw.Trim(kp=1.0, kd=0.05, ki=1.0),
    "trim-ank": lambda: Lw.Trim(kp=0.5, ki=1.0, ka=0.3),
    "trim-out": lambda: Lw.Trim(kp=0.5, ki=1.0, out_rate=20.0),
    "t35-7": lambda: Lw.Trim(kp=0.35, ki=0.7),
    "t35-7a": lambda: Lw.Trim(kp=0.35, ki=0.7, ka=0.2),
    "t25-5": lambda: Lw.Trim(kp=0.25, ki=0.5),
    "t25-5a": lambda: Lw.Trim(kp=0.25, ki=0.5, ka=0.2),
    "t25-10": lambda: Lw.Trim(kp=0.25, ki=1.0),
    "t50-5": lambda: Lw.Trim(kp=0.5, ki=0.5),
    "t30-10": lambda: Lw.Trim(kp=0.3, ki=1.0),
    "t30-10a": lambda: Lw.Trim(kp=0.3, ki=1.0, ka=0.2),
    "t25-10c1": lambda: Lw.Trim(kp=0.25, ki=1.0, kt=0.1),
    "t25-10c3": lambda: Lw.Trim(kp=0.25, ki=1.0, kt=0.3),
    "t25-10c6": lambda: Lw.Trim(kp=0.25, ki=1.0, kt=0.6),
    "t25-10cm1": lambda: Lw.Trim(kp=0.25, ki=1.0, kt=-0.1),
    "t25-10cm3": lambda: Lw.Trim(kp=0.25, ki=1.0, kt=-0.3),
}

PLANTS = {
    "fit": T.BOARD,                                                         # play L6 R8 La3 Ra0, grade floor
    "lo": "grade+play_L_hip=4,play_R_hip=6,play_L_ankle=2,play_R_ankle=0",
    "hi": "grade+play_L_hip=10,play_R_hip=10,play_L_ankle=3,play_R_ankle=2",
    "hard": T.BOARD + ",floor_tc=0.01",
}


def case(job):
    law_name, run, plant, delay, speed, gain = job
    f = LAWS[law_name]
    trims = "logged" if f == "logged" else "none"
    law = None if f == "logged" else f()
    if law is not None and gain != 1.0:
        for k in ("kp", "kd", "ki", "ka"):
            if hasattr(law, k):
                setattr(law, k, getattr(law, k) * gain)
    try:
        src = run
        if run.startswith("synth:"):        # synth:period=2/dx=0.02/dy=0/k_shift=1.0/h=0.025/n_steps=6
            import synth as S
            kw = {}
            for tok in run[6:].split("/"):
                k, _, v = tok.partition("=")
                kw[k] = float(v) if k not in ("first",) else v
            if "n_steps" in kw:
                kw["n_steps"] = int(kw["n_steps"])
            src = S.Synth(**kw)
            if f == "logged":
                return dict(law=law_name, run=run, plant=plant, delay=delay, speed=speed, gain=gain, error="no log")
        o = T.run(src, law, plant=PLANTS.get(plant, plant), trims=trims, speed=speed, extra_ms=delay)
    except Exception as e:      # a case that cannot run is a row, not a crash
        return dict(law=law_name, run=run, plant=plant, delay=delay, speed=speed, gain=gain, error=repr(e)[:200])
    return dict(law=law_name, run=run, plant=plant, delay=delay, speed=speed, gain=gain, fell=o["fell"],
                fall_dir=o["fall_dir"], lifts=o["lifts"], n=o["n_phases"], peak=round(o["peak_deg"], 2),
                pitch=round(o["pitch_deg"], 2),
                margin=round(o["margin_mm"], 1))


def summary(rows):
    out = ["| law | cases | lifts (of phases) | falls OUT / IN | peak roll lean, median / max deg | peak pitch, median / max | margin min mm |",
           "|---|---|---|---|---|---|---|"]
    for law in dict.fromkeys(r["law"] for r in rows):
        rs = [r for r in rows if r["law"] == law and "error" not in r]
        if not rs:
            continue
        lifts, n = sum(r["lifts"] for r in rs), sum(r["n"] for r in rs)
        fo = sum(r["fell"] and r["fall_dir"] == "OUT" for r in rs)
        fi = sum(r["fell"] and r["fall_dir"] != "OUT" for r in rs)
        pk = [r["peak"] for r in rs]
        mg = [r["margin"] for r in rs if r["margin"] == r["margin"]]
        out.append(f"| {law} | {len(rs)} | {lifts}/{n} ({100 * lifts / max(n, 1):.0f} %) | {fo} / {fi} | "
                   f"{np.median(pk):.1f} / {max(pk):.1f} | {np.median([r['pitch'] for r in rs]):.1f} / {max(r['pitch'] for r in rs):.1f} | "
                   f"{min(mg) if mg else float('nan'):.0f} |")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laws", default="off,stepper,oldPI-through,trim")
    ap.add_argument("--runs", default="2026-11-11/181923_lift,2026-11-11/182051_march,2026-11-11/182343_walk")
    ap.add_argument("--plants", default="fit")
    ap.add_argument("--delays", default="0")
    ap.add_argument("--speeds", default="1")
    ap.add_argument("--gains", default="1")
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "eval"))
    a = ap.parse_args(argv)
    jobs = [(l, r, p, float(d), float(s), float(g)) for l in a.laws.split(",") for r in a.runs.split(",")
            for p in a.plants.split(";") for d in a.delays.split(",") for s in a.speeds.split(",")
            for g in a.gains.split(",")]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        rows = list(ex.map(case, jobs, chunksize=1))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(a.out + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    md = summary(rows)
    open(a.out + ".md", "w").write(md + "\n")
    print(md)
    errs = [r for r in rows if "error" in r]
    if errs:
        print(f"{len(errs)} errors, e.g. {errs[0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
