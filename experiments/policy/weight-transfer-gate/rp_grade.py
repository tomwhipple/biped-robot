"""grade.py's `stride1 0.3 robot` and `march5 robot` probes with the fitted roll play beyond the
encoder (fit_rollplay.py: hip 4, ankle 5 deg peak to peak), through grade.py's GRADE_PLANT_TERMS.

    python rp_grade.py [seeds] [--runs a,b] [--md out.md] [--json out.json]

wt_grade.py's per-foot numbers (lift_cm, shift_when_up, falls / guard trips) plus the torso tilt
acos(up_z) at +0.3 / +0.5 / +0.6 s from the walk command (the 2026-11-10 robot on the same probe:
5.6 / 11.3 / 14.4 deg; 2026-11-10-robot/README.md). A run trained with a series spring has it
replaced by the play (gate_common's play terms clear the spring)."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402
import wt_grade as WT  # noqa: E402

RUNS = ["robot_legs_prec16_f075_js20_s128r24", "robot_legs_prec52_march_s128r24",
        "robot_legs_prec57_march_rs_s128r24"]
PROBES = ["stride1 0.3 robot", "march5 robot"]
PLAY = "play_hip=4,play_ankle=5"
VARIANTS = {"grade as is": {},
            "play 4/5": {"GRADE_PLANT_TERMS": PLAY},
            "MASS_SYM": {"GRADE_PLANT_TERMS": GC.MASS_SYM},
            "MASS_SYM + play 4/5": {"GRADE_PLANT_TERMS": GC.MASS_SYM + "," + PLAY}}
ROBOT_TILT = (5.6, 11.3, 14.4)
_TL = []


def _install():
    G = WT._install()
    import simh
    if getattr(simh, "_rp_wrapped", False):
        return G
    b0 = simh.build

    def build(*a, **k):
        env, act, zoff = b0(*a, **k)
        s0 = env.step

        def step(a_):
            out = s0(a_)
            _TL.append((math.degrees(math.acos(max(-1.0, min(1.0, float(out[4]["up_z"]))))),
                        float(env._cmd[0])))
            return out
        env.step = step
        return env, act, zoff
    simh.build = build
    loop0 = G._episode_loop

    def loop(*a, **k):
        _TL.clear()
        return loop0(*a, **k)
    G._episode_loop = loop
    simh._rp_wrapped = True
    return G


def tilt_after_cmd(tl, dt=0.02):
    """tilt at +0.3 / +0.5 / +0.6 s from the first tick with a nonzero vx command."""
    tl = np.asarray(tl, float)
    k = np.flatnonzero(tl[:, 1] != 0.0)
    if not len(k):
        return [None, None, None]
    k0 = int(k[0])
    out = []
    for s in (0.3, 0.5, 0.6):
        i = k0 + int(round(s / dt)) - 1
        out.append(float(tl[i, 0]) if i < len(tl) else None)
    return out


def job(args):
    run, vname, probe, seed = args
    for k in WT.ENV_KEYS:
        os.environ.pop(k, None)
    os.environ.update(VARIANTS[vname])
    G = _install()
    p = G.PROBES[probe]
    spec = dict(run=run, shaper_hz=GC.SHAPER_HZ, stand_phase=np.pi / 2, clock="coast")
    spec.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
    m = G.episode(spec, seed=seed, run_s=p["run_s"], hold_s=p.get("hold_s", 0.0))
    m = dict(fell=bool(m["fell"]), trip=m.get("trip"), trips_by=m.get("trips_by"),
             tilt_max=m.get("tilt_max"), peak_rate=m.get("peak_rate"),
             tilt_cmd=tilt_after_cmd(_TL), **WT.wt_metrics(WT._FT))
    return run, vname, probe, seed, m


def _m(xs, nd=1):
    xs = [x for x in xs if x is not None and x == x]
    return f"{np.mean(xs):.{nd}f}" if xs else "-"


def table(res, runs):
    L = ["| student | plant | probe | lift L / R cm, mean (min) | shift_when_up L-up / R-up | "
         "tilt +0.3 / +0.5 / +0.6 s, deg (robot 5.6 / 11.3 / 14.4, stride1) | tilt max | falls / trips [kinds] |",
         "|---|---|---|---|---|---|---|---|"]
    for run in runs:
        for vn in VARIANTS:
            for pr in PROBES:
                e = [m for (r, v, p, s), m in res.items() if r == run and v == vn and p == pr]
                if not e:
                    continue
                lf = np.array([m["lift_cm"] for m in e])
                sh = np.array([[np.nan if x is None else x for x in m["shift_when_up"]] for m in e], float)
                tc = [m["tilt_cmd"] for m in e]
                tcs = " / ".join(_m([t[j] for t in tc]) for j in range(3)) if pr.startswith("stride") else "-"
                L.append(f"| {run.replace('robot_legs_', '').replace('_s128r24', '')} | {vn} | {pr} | "
                         f"{lf[:, 0].mean():.2f} ({lf[:, 0].min():.2f}) / {lf[:, 1].mean():.2f} ({lf[:, 1].min():.2f}) | "
                         f"{_m(sh[:, 0], 2)} / {_m(sh[:, 1], 2)} | {tcs} | {_m([m['tilt_max'] for m in e])} | "
                         f"{sum(m['fell'] for m in e)}/{sum(m['trip'] is not None for m in e)} of {len(e)}"
                         f" [{WT._kinds(e)}] |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("seeds", nargs="?", type=int, default=8)
    ap.add_argument("--runs", default=",".join(RUNS))
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "16")))
    a = ap.parse_args(argv)
    runs = [r for r in a.runs.split(",") if os.path.exists(os.path.join(GC.run_dir(r), "params.pkl"))]
    jobs = [(r, v, p, s) for r in runs for v in VARIANTS for p in PROBES for s in range(a.seeds)]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        res = {(r, v, p, s): m for r, v, p, s, m in ex.map(job, jobs)}
    md = table(res, runs)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump([dict(run=r, variant=v, probe=p, seed=s, **m) for (r, v, p, s), m in res.items()],
                  open(a.json, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
