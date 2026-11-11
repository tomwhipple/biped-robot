"""Fit the plant's roll chain to the 2026-11-11 stepper lifts (#288): per-side roll play, a lateral
torso CoM offset, ankle-roll friction -- each a separate hypothesis -- scored by stepper_replay.py's
open-loop replay of the robot's written goals.

    python fit_stepper.py grid  --name NAME --base SPEC --axes "k=v1,v2;k2=..." [--runs ...] [--workers N]
    python fit_stepper.py check --name NAME --plants "label=SPEC;..." [--runs ...]

`grid` runs every combination of the axes on top of the base plant spec (stepper_replay's syntax,
e.g. `grade` or `grade+torso_kg=0.4855,zoff_La=0`). An axis key is a gate_common term, or one of
  sL / sR   both rolls of that side's play x (hip 4, ankle 5 deg p-p): sL=1 -> play_L_hip=4,play_L_ankle=5
The cost of a plant (--cost): "lift" (default), the weighted RMS of the per-run, per-stance mean roll
errors at the lift increments and while the foot was up (stepper_replay's nool / air rows, crouch
referenced with --ref crouch); or "rms", the mean roll RMS while both are upright. Either plus FALL_W
deg x the share of runs whose fall / no-fall the sim gets wrong. Every (run, plant) row is written to
results/stepper_replay/fit_<name>.csv, the per-plant costs to fit_<name>_cost.csv.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stepper_replay as SR  # noqa: E402

FIT_RUNS = ["101255_lift", "101413_lift", "103709_lift", "104032_lift", "104147_lift", "111627_lift",
            "111741_lift", "111855_lift", "112013_lift", "112144_lift", "112303_lift", "112422_lift",
            "112541_lift", "112700_lift"]
HELD_OUT = ["104303_march", "104435_walk", "110045_walk", "110708_walk"]
FALL_W = 3.0
HIP, ANKLE = 4.0, 5.0      # the clamp fit's play, deg p-p (README Round 4)


def axis_terms(k, v):
    if k in ("sL", "sR"):
        s = k[1]
        return {f"play_{s}_hip": round(HIP * v, 3), f"play_{s}_ankle": round(ANKLE * v, 3)}
    return {k: v}


def spec_of(base, terms):
    if not terms:
        return base
    body = ",".join(f"{k}={v:g}" for k, v in terms.items())
    return base + ("," if "+" in base else "+") + body


def one(job):
    stamp, spec = job
    try:
        s = SR.simulate((stamp, spec))
        o, _ = SR.score(s)
        return o
    except Exception as e:        # a plant the env refuses is a row, not a crash
        return dict(run=stamp, plant=spec, error=repr(e))


def cost(rows, kind="rms"):
    """kind "rms": the mean upright roll RMS. kind "lift": the RMS of the per-run, per-stance mean
    roll errors at the lift increments and while up (nool / air rows), weighted by their row counts
    -- the part of each run where the roll chain carries one leg. Both + FALL_W per wrong fall."""
    ok = [r for r in rows if "error" not in r]
    if len(ok) < len(rows):
        return float("inf"), {}
    rms = float(np.mean([r["roll_rms_up"] if r["roll_rms_up"] is not None else 30.0 for r in ok]))
    bad = sum(r["robot_fell"] != r["sim_fell"] for r in ok)
    e, wts, side = [], [], {"L": [], "R": []}
    for r in ok:
        for st in "LR":
            for k, n in ((f"nool_droll_st{st}", r.get(f"nool_n_st{st}", 0)), (f"air_droll_st{st}", r.get("air_n", 0))):
                v = r.get(k)
                if v is not None and np.isfinite(v) and n:
                    e.append(v); wts.append(n); side[st].append((v, n))
    lift = float(np.sqrt(np.average(np.square(e), weights=wts))) if e else float("nan")
    mean_st = {st: (float(np.average([v for v, _ in side[st]], weights=[n for _, n in side[st]])) if side[st] else None)
               for st in "LR"}
    main = rms if kind == "rms" else lift
    return main + FALL_W * bad / len(ok), dict(rms=rms, lift=lift, err_stL=mean_st["L"], err_stR=mean_st["R"],
                                               fall_wrong=bad, n=len(ok))


def run_jobs(runs, specs, workers):
    for s in runs:
        if not os.path.exists(SR._cache_path(s)):
            SR.extract([s])
    jobs = [(r, p) for p in specs for r in runs]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(one, jobs, chunksize=1))


def write(name, rows, costs):
    os.makedirs(SR.OUT, exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(SR.OUT, f"fit_{name}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(SR.OUT, f"fit_{name}_cost.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["plant", "cost", "roll_rms_up_mean", "lift_rms", "err_stance_L", "err_stance_R", "fall_wrong", "n"])
        r4 = lambda v: None if v is None else round(v, 4)
        for p, (c, d) in sorted(costs.items(), key=lambda x: x[1][0]):
            w.writerow([p, round(c, 4), r4(d.get("rms")), r4(d.get("lift")), r4(d.get("err_stL")), r4(d.get("err_stR")),
                        d.get("fall_wrong"), d.get("n")])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("grid", "check"))
    ap.add_argument("--name", required=True)
    ap.add_argument("--base", default="grade")
    ap.add_argument("--axes", default="sL=0,0.5,1,1.5,2;sR=0,0.5,1,1.5,2")
    ap.add_argument("--plants", default="")
    ap.add_argument("--runs", default=",".join(FIT_RUNS))
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--cost", choices=("rms", "lift"), default="lift")
    ap.add_argument("--ref", choices=("stand", "crouch"), default="stand",
                    help="stepper_replay.score's level reference")
    a = ap.parse_args(argv)
    os.environ["SR_REF"] = a.ref          # the workers score with it
    runs = [x for x in a.runs.split(",") if x]
    if a.what == "grid":
        axes = []
        for tok in a.axes.split(";"):
            k, _, vs = tok.partition("=")
            axes.append((k.strip(), [float(v) for v in vs.split(",")]))
        specs = []
        for combo in itertools.product(*[vs for _, vs in axes]):
            t = {}
            for (k, _), v in zip(axes, combo):
                t.update(axis_terms(k, v))
            specs.append(spec_of(a.base, t))
    else:
        specs = [p for p in a.plants.split(";") if p]
    rows = run_jobs(runs, specs, a.workers)
    by = {}
    for r in rows:
        by.setdefault(r["plant"], []).append(r)
    costs = {p: cost(rs, a.cost) for p, rs in by.items()}
    write(a.name, rows, costs)
    print(f"{len(specs)} plants x {len(runs)} runs; best {a.top}:")
    for p, (c, d) in sorted(costs.items(), key=lambda x: x[1][0])[:a.top]:
        f = lambda v: "  -  " if v is None else f"{v:+5.1f}"
        print(f"  {c:6.2f}  rms {d.get('rms', float('nan')):5.2f}  lift {d.get('lift', float('nan')):5.2f}  "
              f"stance L {f(d.get('err_stL'))} R {f(d.get('err_stR'))}  fall wrong {d.get('fall_wrong')}/{d.get('n')}  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
