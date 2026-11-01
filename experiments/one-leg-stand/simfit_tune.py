"""Retune the bench roll balance in the FITTED plant and a robustness band around it.

The scenario is the full 10-25 protocol (shift, 10 mm lift 6 s with B, lower, 30 mm lift 4 s
with B + 12 s hold, lower, crouch) for each stance, on the recorded timing and pose ticks
(simfit_hw.protocol_moves), with 0.05 deg IMU noise and 0.0075 rad/s gyro noise.

Per run:  ok   = no abort, no fall, the foot stays off for >= 90 % of the 30 mm hold
          marg = min over the 30 mm stage (lift + hold) of the CoM distance to the OUTER sole
                 edge (mm; the outward fall's margin), and to the inner edge
          err5 = rms err over the last 5 s of the hold, u_end, peak |err|
The plant is simfit_plant.json: the fitted plant plus a band (each fitted parameter at the ends
of its profile range, the symmetric plant, and three "hardware-like L" variants whose L stance
tips over at u 5-6.5 deg as the robot did); seeds 1.. add IMU / gyro noise.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_tune.py grid \
        kp=0.2,0.3 ki=0.5,1.0 kd=0.03 i_lim_deg=0,6 ...   [--plant fitted]
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace, asdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402

R2D = 180 / math.pi
# the fitted plant and its robustness band: simfit_plant.json (2026-10-30-sim-fit.md)


def band_plants(base, band):
    out = [("nominal", base)]
    for name, kw in band.items():
        out.append((name, replace(base, **kw)))
    return out


PROTO = os.environ.get("SIMFIT_PROTO", "194709")     # the session whose timing/poses the protocol replays (clr.sh)


def run_one(job):
    side, pname, pl, ctl, seed = job[:5]
    floor = job[5] if len(job) > 5 else "carpet"
    own, moves = H.protocol_moves(side, PROTO)
    plf = pl.on(floor)
    r = S.Replay(own, plf, ctl, seed=seed, imu_noise_deg=0.05 if seed else 0.0,
                 gyro_noise=0.0075 if seed else 0.0, moves=moves).run()
    pl = plf
    m = {mv.name: mv for mv in moves}
    t = r["t"]
    l10, l30, lo30 = m["l10"], m["l30"], m["lower30"]
    ab10 = bool(np.any((t >= l10.t0) & (t < m["lower10"].t0) & (r["state"] == 2)))
    ab30 = bool(np.any((t >= l30.t0) & (t < lo30.t0) & (r["state"] == 2)))
    base = r["roll"][(t > -0.6) & (t < 0)].mean()
    fell = bool(np.any(np.abs(r["roll"] - base) > math.radians(15.0)))
    hold = (t >= l30.t0 + l30.dur) & (t < lo30.t0)
    w30 = (t >= l30.t0) & (t < lo30.t0)
    W = pl.sole_w / 2
    W_out = W - (pl.sole_out_dL if side == "L" else pl.sole_out_dR)
    W_in = W - (pl.sole_in_dL if side == "L" else pl.sole_in_dR)
    comy = r["comy"]
    marg_out = 1e3 * float(np.min(W_out - comy[w30]))
    marg_in = 1e3 * float(np.min(W_in + comy[w30]))
    # the roll mode in the hold: band-passed (0.8-4 Hz) roll rms over the last 8 s of the hold (deg)
    import simfit_osc as O
    hh = (t >= lo30.t0 - 8.0) & (t < lo30.t0)
    tg, xb = O.bandpass(t[hh], r["roll"][hh] * R2D) if hh.sum() > 50 else (None, np.array([0.0]))
    osc = float(np.sqrt(np.mean(xb[50:-50] ** 2))) if len(xb) > 120 else float("nan")
    weight = np.median(r["fz_st"][hold] + r["fz_sw"][hold]) if hold.any() else 1.0
    off = float(np.mean(r["fz_sw"][hold] < 0.03 * weight)) if hold.any() else 0.0
    sg = 1.0 if side == "L" else -1.0
    last5 = hold & (t >= lo30.t0 - 5.0)
    err5 = R2D * float(np.sqrt(np.mean(r["err"][last5] ** 2))) if last5.any() else float("nan")
    errpk = R2D * float(np.max(np.abs(r["err"][w30]))) if w30.any() else float("nan")
    errout = R2D * float(np.max(-sg * r["err"][w30])) if w30.any() else float("nan")   # toward the stance foot
    u_end = R2D * sg * float(r["u"][hold][-1]) if hold.any() else float("nan")
    w10 = (t >= l10.t0) & (t < m["lower10"].t0)
    marg10 = 1e3 * float(np.min(W_out - comy[w10]))
    ok = (not ab10) and (not ab30) and (not fell) and off >= 0.9 and not (osc > 0.5)
    return dict(side=side, floor=floor, plant=pname, seed=seed, ok=ok, osc=osc, ab10=ab10, ab30=ab30, fell=fell, off=off,
                marg_out=marg_out, marg_in=marg_in, marg10=marg10, err5=err5, errpk=errpk, errout=errout, u_end=u_end)


POOL = None


def evaluate(ctls, plants, seeds=(0, 1, 2), sides=("R", "L"), floors=("carpet", "hard")):
    global POOL
    if POOL is None:
        POOL = ProcessPoolExecutor(max_workers=int(os.environ.get("SIMFIT_WORKERS", os.cpu_count())))
    jobs = [(sd, pn, pl, c, s, fl) for c in ctls for (pn, pl) in plants for fl in floors for sd in sides for s in seeds]
    out = list(POOL.map(run_one, jobs))
    n = len(plants) * len(sides) * len(seeds) * len(floors)
    return [(c, out[i * n:(i + 1) * n]) for i, c in enumerate(ctls)]


def summarise(c, rows):
    s = {}
    for key in sorted({(r["floor"], r["side"]) for r in rows}):
        rr = [r for r in rows if (r["floor"], r["side"]) == key]
        side = key[0][0] + key[1]          # "cR", "hL", ...
        s[side] = dict(ok=sum(r["ok"] for r in rr), n=len(rr), osc=max(r["osc"] for r in rr),
                       marg_out=min(r["marg_out"] for r in rr), marg_in=min(r["marg_in"] for r in rr),
                       marg_out_nom=next((r["marg_out"] for r in rr if r["plant"] == "nominal" and r["seed"] == 0), float("nan")),
                       err5=max(r["err5"] for r in rr), errpk=max(r["errpk"] for r in rr),
                       u_end=float(np.median([r["u_end"] for r in rr])))
    return s


def ctl_label(c):
    d, b = asdict(c), asdict(S.Ctl())
    return " ".join(f"{k}={v:g}" for k, v in d.items() if v != b[k]) or "bench default"


def fmt(c, s):
    return (f"{ctl_label(c):58s} " + "  ".join(
        f"{sd}: {v['ok']}/{v['n']} out>={v['marg_out']:4.1f} in>={v['marg_in']:4.1f} osc<={v['osc']:.2f} "
        f"pk<={v['errpk']:.1f} u {v['u_end']:+.1f}" for sd, v in s.items()))


def parse_grid(specs):
    keys, vals = [], []
    for spec in specs:
        k, vs = spec.split("=")
        keys.append(k)
        vals.append([float(v) for v in vs.split(",")])
    out = []
    for combo in itertools.product(*vals):
        kw = dict(zip(keys, combo))
        if "i_lim_deg" in kw:          # limit on the I contribution ki*I, in degrees of u
            lim = kw.pop("i_lim_deg")
            ki = kw.get("ki", S.Ctl().ki)
            kw["i_max"] = math.radians(lim) / ki if lim > 0 and ki > 0 else 0.0
        out.append(S.Ctl(**kw))
    return out


def load_fitted(path):
    d = json.load(open(path))
    return S.Plant(**d["plant"]), {k: v for k, v in d.get("band", {}).items()}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("grid",))
    ap.add_argument("specs", nargs="*")
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    ap.add_argument("--nominal-only", action="store_true")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--floors", default="carpet,hard")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    pl, band = load_fitted(a.fitted)
    plants = band_plants(pl, {} if a.nominal_only else band)
    ctls = parse_grid(a.specs) if a.specs else [S.Ctl()]
    res = evaluate(ctls, plants, seeds=tuple(int(x) for x in a.seeds.split(",")), floors=tuple(a.floors.split(",")))
    rows = []
    for c, rr in sorted(res, key=lambda x: -sum(r["ok"] for r in x[1])):
        s = summarise(c, rr)
        print(fmt(c, s), flush=True)
        rows.append(dict(ctl=asdict(c), summary=s, runs=rr))
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)
