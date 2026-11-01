"""Fit the actuator model and the structure/floor compliance to the double-support roll sweep
(simfit_sweep.py; hw 2026-11-01 21:32, hard floor) -- a random design in a box, then Nelder-Mead.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_sweepfit.py \
        friction=0.001:0.35:log kp_scale=0.3:1.5 ... --base k=v ... [--n0 400] [--iters 300] [--json out]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace, asdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_sim as S  # noqa: E402
import simfit_sweep as W  # noqa: E402


_HWSTEP = None


def _step_job(pl):
    """step cost of one plant, the joints run serially (this is already one pool worker)."""
    global _HWSTEP
    import simfit_step as T
    if _HWSTEP is None:
        _HWSTEP = T.hw_steps()
    sim = dict(T._sim_one((pl, j)) for j in T.STEP_JOINTS)
    return T.step_cost(sim, _HWSTEP)


def main():
    from scipy.optimize import minimize
    ap = argparse.ArgumentParser()
    ap.add_argument("specs", nargs="+")
    ap.add_argument("--base", nargs="*", default=[])
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    ap.add_argument("--n0", type=int, default=400)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    base = replace(S.Plant(**json.load(open(a.fitted))["plant"]),
                   **{k: float(v) for k, v in (x.split("=") for x in a.base)})
    keys, lo, hi, lg = [], [], [], []
    for sp in a.specs:
        k, r = sp.split("=")
        p = r.split(":")
        l, h, isl = float(p[0]), float(p[1]), len(p) > 2 and p[2] == "log"
        keys.append(k); lo.append(math.log10(l) if isl else l); hi.append(math.log10(h) if isl else h); lg.append(isl)
    lo, hi, lg = np.array(lo), np.array(hi), np.array(lg)

    def to_pl(z):
        x = lo + (hi - lo) * np.clip(z, 0, 1)
        x = np.where(lg, 10 ** x, x)
        return replace(base, **dict(zip(keys, x.tolist())))

    ex = ProcessPoolExecutor(int(os.environ.get("SIMFIT_WORKERS", "30")))

    step_w = float(os.environ.get("STEP_W", "0"))      # + step_w x simfit_step.step_cost (loaded step responses)

    def batch(Z):
        pls = [to_pl(z) for z in Z]
        out = list(ex.map(W.cost_job, pls))
        sc = list(ex.map(_step_job, pls)) if step_w > 0 else [0.0] * len(pls)
        return [o[0] + step_w * s_ + 100 * float(np.sum(np.clip(-z, 0, None) + np.clip(z - 1, 0, None)))
                for o, s_, z in zip(out, sc, Z)], out

    rng = np.random.default_rng(1)
    Z0 = rng.uniform(0, 1, (a.n0, len(keys)))
    c0, _ = batch(Z0)
    order = np.argsort(c0)
    for i in order[:3]:
        print("design", round(c0[i], 2), to_pl(Z0[i]).label(), flush=True)
    cache = {}

    def f(z):
        kk = tuple(np.round(z, 4))
        if kk not in cache:
            cache[kk] = batch([z])[0][0]
            if len(cache) % 20 == 0:
                print(f"  nm {len(cache)} {cache[kk]:.2f}", flush=True)
        return cache[kk]

    z = Z0[order[0]]
    n = len(keys)
    simplex = [z] + [z + np.eye(n)[i] * (0.15 if z[i] < 0.85 else -0.15) for i in range(n)]
    r = minimize(f, z, method="Nelder-Mead", options=dict(initial_simplex=np.array(simplex), maxfev=a.iters,
                                                          xatol=0.005, fatol=0.01))
    best = to_pl(r.x)
    c, rows = W.cost_job(best)
    sc = _step_job(best) if step_w > 0 else float("nan")
    print("BEST", round(c, 3), "step", round(sc, 3), best.label(), flush=True)
    W.report(rows, "sim (best)")
    if a.json:
        json.dump(dict(best=asdict(best), cost=c, step_cost=sc, keys=keys), open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
