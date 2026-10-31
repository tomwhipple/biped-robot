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

POOL = None


def pool():
    global POOL
    if POOL is None:
        POOL = ProcessPoolExecutor(max_workers=int(os.environ.get("SIMFIT_WORKERS", os.cpu_count() or 8)))
    return POOL


def eval_many(plants, stands=C.ALL, ctl=None, seeds=(0,)):
    """[(plant, total cost, per-stand results)] for many plants at once."""
    ctl = ctl or S.Ctl()
    jobs = [(t, s, pl, ctl, dict(seed=sd)) for pl in plants for sd in seeds for (t, s) in stands]
    out = list(pool().map(C.eval_stand, jobs))
    for o in out:
        o.pop("r")
    res = []
    n = len(stands) * len(seeds)
    for i, pl in enumerate(plants):
        chunk = out[i * n:(i + 1) * n]
        res.append((pl, sum(o["cost"] for o in chunk) / len(seeds), chunk))
    return res


def line(pl, cost, chunk, keys):
    vals = " ".join(f"{k}={getattr(pl, k):g}" for k in keys)
    per = " ".join(f"{o['tag'][-3:]}{o['side']}:{o['cost']:.1f}" for o in chunk)
    l30 = " ".join(f"{o['side']}u{o['stages'].get('l30', {}).get('sim_u_end', float('nan')):+.1f}"
                   f"{'A' if o['stages'].get('l30', {}).get('sim_abort') else ''}" for o in chunk)
    fe = "".join("F" if o["stages"].get("fell_early") else "." for o in chunk)
    return f"{vals:40s} cost {cost:6.2f}  [{per}]  {l30} {fe}"


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
    for pl, cost, chunk in eval_many(plants):
        print(line(pl, cost, chunk, keys), flush=True)


def cmd_grid(a):
    base = parse_base(a.base)
    keys, vals = [], []
    for spec in a.specs:
        k, vs = spec.split("=")
        keys.append(k)
        vals.append([float(v) for v in vs.split(",")])
    plants = [replace(base, **dict(zip(keys, combo))) for combo in itertools.product(*vals)]
    res = eval_many(plants)
    rows = []
    for pl, cost, chunk in sorted(res, key=lambda x: x[1]):
        print(line(pl, cost, chunk, keys), flush=True)
        rows.append(dict(plant=asdict(pl), cost=cost, stands=[(o["tag"], o["side"], o["cost"], o["stages"]) for o in chunk]))
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1, default=float)


def cmd_fit(a):
    from scipy.optimize import minimize
    base = parse_base(a.base)
    keys, lo, hi = [], [], []
    for spec in a.specs:
        k, r = spec.split("=")
        l, h = (float(x) for x in r.split(":"))
        keys.append(k); lo.append(l); hi.append(h)
    lo, hi = np.array(lo), np.array(hi)
    stands = C.ALL if not a.train else [(x[:-1], x[-1]) for x in a.train.split(",")]
    hist = []

    def to_pl(z):
        x = lo + (hi - lo) * np.clip(z, 0, 1)
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
        Z0[0] = (np.array([float(v) for v in a.x0.split(",")]) - lo) / (hi - lo)
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
            plants.append(replace(best, **{k: float(v)}))
    res = eval_many(plants, stands)
    j = 0
    for i, k in enumerate(keys):
        prof[k] = []
        for v in np.linspace(lo[i], hi[i], a.nprof):
            prof[k].append((float(v), res[j][1]))
            j += 1
        print("profile", k, " ".join(f"{v:g}:{c:.2f}" for v, c in prof[k]), flush=True)
    if a.json:
        json.dump(dict(best=asdict(best), cost=float(r.fun), keys=keys, lo=lo.tolist(), hi=hi.tolist(),
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
