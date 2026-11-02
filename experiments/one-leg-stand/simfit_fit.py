"""Fit the plant to the recorded one-leg stands (simfit_compare cost, all stands jointly).

    sweep  KEY=v1,v2,...  [--base KEY=VAL ...]       one-at-a-time sensitivity around a base plant
    grid   KEY=v1,v2 KEY2=w1,w2 ... [--base ...]     full grid (every combination)
    fit    KEY=lo:hi ... [--base ...] [--train STANDS] [--iters N]   Nelder-Mead in the box, then
           a profile (cost vs each parameter around the optimum) for the uncertainty

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_fit.py sweep roll_lash=0,1,3,5
Every (plant, stand) replay is one process; 32 cores run ~5 plants at a time.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace, asdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_compare as C  # noqa: E402
import simfit_sim as S  # noqa: E402
import simfit_hw as H  # noqa: E402

POOL = None


def pool():
    global POOL
    if POOL is None:
        POOL = ProcessPoolExecutor(max_workers=int(os.environ.get("SIMFIT_WORKERS", os.cpu_count() or 8)))
    return POOL


# The robot's stance plant at its hard-floor limit cycle (simfit_frf.py hw): roll/u at 1.85 Hz.
FRF_HW = (1.85, 2.3, -46.0)
FRF_W = float(os.environ.get("SIMFIT_FRF_W", "0"))     # weight of the FRF term (0 = off)


def frf_cost(m, ph):
    """|ln(|P|/2.3)|/0.3 + |phase error|/30 deg; an abort under the 1 deg dither costs 1.5."""
    if not np.isfinite(m):
        return 1.5
    dph = (ph - FRF_HW[2] + 180.0) % 360.0 - 180.0
    return abs(math.log(max(m, 1e-3) / FRF_HW[1])) / 0.3 + abs(dph) / 30.0


def _frf_job(pl):
    import simfit_frf as F
    return F._one((pl, "hard", "R", FRF_HW[0], 1.0, "194709"))


TIP_W = float(os.environ.get("SIMFIT_TIP_W", "0"))     # weight of the inside-tip test (simfit_tip), 0 = off


SWEEP_W = float(os.environ.get("SIMFIT_SWEEP_W", "0"))  # weight of the double-support roll sweep (simfit_sweep)


def _sweep_job(pl):
    import simfit_sweep as W
    return W.cost_job(pl)[0]


def _tip_job(args):
    import simfit_tip as T
    o = T._job(args)
    return o


def eval_many(plants, stands=C.ALL, ctl=None, seeds=(0,), frf_w=None, tip_w=None):
    """[(plant, total cost, per-stand results)] for many plants at once. With an FRF weight, each
    plant also pays frf_w * frf_cost of its hard-floor R-stance roll/u at 1.85 Hz (a pseudo-stand
    "FRF" in the chunk)."""
    frf_w = FRF_W if frf_w is None else frf_w
    jobs = [(sid, pl, ctl, dict(seed=sd)) for pl in plants for sd in seeds for sid in stands]
    futs = [pool().submit(C.eval_stand, j) for j in jobs]
    ffut = [pool().submit(_frf_job, pl) for pl in plants] if frf_w > 0 else []
    tip_w = TIP_W if tip_w is None else tip_w
    tfut = [[pool().submit(_tip_job, (pl, sd)) for sd in "LR"] for pl in plants] if tip_w > 0 else []
    sfut = [pool().submit(_sweep_job, pl) for pl in plants] if SWEEP_W > 0 else []
    out = [f.result() for f in futs]
    for o in out:
        o.pop("r")
    res = []
    n = len(stands) * len(seeds)
    for i, pl in enumerate(plants):
        chunk = out[i * n:(i + 1) * n]
        cost = sum(o["cost"] for o in chunk) / len(seeds)
        if ffut:
            f, m, ph = ffut[i].result()
            fc = frf_w * frf_cost(m, ph)
            cost += fc
            chunk = chunk + [dict(id="FRF", side="R", floor="hard", cost=fc, stages={"frf": (m, ph)})]
        if sfut:
            sc = SWEEP_W * sfut[i].result()
            cost += sc
            chunk = chunk + [dict(id="SWEEP", side="-", floor="hard", cost=sc, stages={})]
        if tfut:
            for f in tfut[i]:
                o = f.result()
                tc = tip_w * o["cost"]
                cost += tc
                chunk = chunk + [dict(id="TIP" + o["side"], side=o["side"], floor="hard", cost=tc,
                                      stages={"tip": (o["sim_step"], o["hw_step"])})]
        res.append((pl, cost, chunk))
    return res


def line(pl, cost, chunk, keys):
    vals = " ".join(f"{k}={getattr(pl, k):g}" for k in keys)
    per = " ".join(f"{o['id'][3:]}:{o['cost']:.1f}" for o in chunk)
    l30 = " ".join(f"{o['side']}u{o['stages'].get('l30', {}).get('sim_u_end', float('nan')):+.1f}"
                   f"{'A' if o['stages'].get('l30', {}).get('sim_abort') else ''}" for o in chunk if o["id"] not in ("FRF", "TIPL", "TIPR", "SWEEP"))
    frf = [o["stages"]["frf"] for o in chunk if o["id"] == "FRF"]
    if frf:
        l30 += f" P {frf[0][0]:.2f}/{frf[0][1]:+.0f}"
    tips = [o["stages"]["tip"] for o in chunk if o["id"] in ("TIPL", "TIPR")]
    if tips:
        l30 += " tip " + "/".join(str(t[0]) for t in tips)
    fe = "".join("F" if o["stages"].get("fell_early") else "." for o in chunk if o["id"] not in ("FRF", "TIPL", "TIPR", "SWEEP"))
    return f"{vals:40s} cost {cost:6.2f}  [{per}]  {l30} {fe}"


def stands_of(a):
    if not a.train:
        return C.ALL
    return H.all_ids(a.train) if a.train in ("carpet", "hard") else a.train.split(",")


def parse_base(items):
    return C.parse_kv(items, S.Plant)


def cmd_sweep(a):
    base = parse_base(a.base)
    plants, keys = [base], []
    for spec in a.specs:
        k, vs = spec.split("=")
        keys.append(k)
        for v in vs.split(","):
            plants.append(replace(base, **{k: float(v)}))
    for pl, cost, chunk in eval_many(plants, stands_of(a)):
        print(line(pl, cost, chunk, keys), flush=True)


def cmd_grid(a):
    base = parse_base(a.base)
    keys, vals = [], []
    for spec in a.specs:
        k, vs = spec.split("=")
        keys.append(k)
        vals.append([float(v) for v in vs.split(",")])
    plants = [replace(base, **dict(zip(keys, combo))) for combo in itertools.product(*vals)]
    res = eval_many(plants, stands_of(a))
    rows = []
    for pl, cost, chunk in sorted(res, key=lambda x: x[1]):
        print(line(pl, cost, chunk, keys), flush=True)
        rows.append(dict(plant=asdict(pl), cost=cost, stands=[(o["id"], o["cost"], o["stages"]) for o in chunk]))
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1, default=float)


def cmd_fit(a):
    from scipy.optimize import minimize
    base = parse_base(a.base)
    keys, lo, hi, logs = [], [], [], []
    for spec in a.specs:
        k, r = spec.split("=")
        parts = r.split(":")
        l, h = float(parts[0]), float(parts[1])
        lg = len(parts) > 2 and parts[2] == "log"        # KEY=lo:hi:log searches log10(value)
        keys.append(k); lo.append(math.log10(l) if lg else l); hi.append(math.log10(h) if lg else h); logs.append(lg)
    lo, hi, logs = np.array(lo), np.array(hi), np.array(logs)
    stands = stands_of(a)
    hist = []

    def to_pl(z):
        x = lo + (hi - lo) * np.clip(z, 0, 1)
        x = np.where(logs, 10.0 ** x, x)
        return replace(base, **dict(zip(keys, x.tolist())))

    def f_batch(Z):
        res = eval_many([to_pl(z) for z in Z], stands)
        for (pl, cost, chunk), z in zip(res, Z):
            pen = 100.0 * float(np.sum(np.clip(-z, 0, None) + np.clip(z - 1, 0, None)))
            hist.append((cost + pen, pl, chunk))
        return [c + 100.0 * float(np.sum(np.clip(-z, 0, None) + np.clip(z - 1, 0, None))) for (pl, c, ch), z in zip(res, Z)]

    # coarse start: a small random design in the box (parallel), then Nelder-Mead from the best
    rng = np.random.default_rng(0)
    Z0 = rng.uniform(0, 1, size=(a.n0, len(keys)))
    if a.x0:
        x0 = np.array([float(v) for v in a.x0.split(",")])
        Z0[0] = (np.where(logs, np.log10(np.maximum(x0, 1e-12)), x0) - lo) / (hi - lo)
    c0 = f_batch(Z0)
    order = np.argsort(c0)
    print("start design best:", [line(hist[i][1], hist[i][0], hist[i][2], keys) for i in order[:3]], flush=True)
    zbest = Z0[order[0]]
    cache = {}

    def f(z):
        key = tuple(np.round(z, 4))
        if key not in cache:
            cache[key] = f_batch([z])[0]
            print(f"  nm {len(cache):3d}  {line(hist[-1][1], hist[-1][0], hist[-1][2], keys)}", flush=True)
        return cache[key]

    # Nelder-Mead evaluates one point at a time; the initial simplex is evaluated in parallel first
    n = len(keys)
    simplex = [zbest] + [zbest + np.eye(n)[i] * (0.15 if zbest[i] < 0.85 else -0.15) for i in range(n)]
    cs = f_batch(simplex)
    for z, cc in zip(simplex, cs):
        cache[tuple(np.round(z, 4))] = cc
    r = minimize(f, zbest, method="Nelder-Mead", options=dict(initial_simplex=np.array(simplex), maxfev=a.iters,
                                                              xatol=0.01, fatol=0.02))
    best = to_pl(r.x)
    print("BEST", best.label(), "cost", r.fun, flush=True)
    # profile: each parameter alone across its box, the others at the optimum (parallel)
    prof = {}
    plants = []
    for i, k in enumerate(keys):
        for v in np.linspace(lo[i], hi[i], a.nprof):
            plants.append(replace(best, **{k: float(10 ** v if logs[i] else v)}))
    res = eval_many(plants, stands)
    j = 0
    for i, k in enumerate(keys):
        prof[k] = []
        for v in np.linspace(lo[i], hi[i], a.nprof):
            prof[k].append((float(10 ** v if logs[i] else v), res[j][1]))
            j += 1
        print("profile", k, " ".join(f"{v:g}:{c:.2f}" for v, c in prof[k]), flush=True)
    if a.json:
        json.dump(dict(best=asdict(best), cost=float(r.fun), keys=keys, lo=lo.tolist(), hi=hi.tolist(), log=logs.tolist(),
                       profile=prof, stands=stands,
                       history=[(c, asdict(pl)) for c, pl, ch in hist]), open(a.json, "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("sweep", "grid", "fit"))
    ap.add_argument("specs", nargs="+")
    ap.add_argument("--base", nargs="*", default=[])
    ap.add_argument("--train", default=None)
    ap.add_argument("--iters", type=int, default=80)
    ap.add_argument("--n0", type=int, default=24)
    ap.add_argument("--x0", default=None)
    ap.add_argument("--nprof", type=int, default=7)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    t0 = time.time()
    {"sweep": cmd_sweep, "grid": cmd_grid, "fit": cmd_fit}[a.mode](a)
    print(f"({time.time() - t0:.0f} s)")
