"""Round 2, item 4: grade.py's weight-transfer columns with and without the roll-chain opt-ins
(GRADE_ROLL_LASH / GRADE_ROLL_SERIES, grade.roll_plant_opts).

    python wt_grade.py [seeds] [--md out.md] [--json out.json]

Per student x plant variant x probe (`stride1 0.3 robot`, `steps5 0.3 robot`), the same per-foot
numbers the 11-10 planner ranked by (its job-tmp grade_recover.py, re-implemented here so the record
re-runs from the repo): lift_cm = the peak rise of each sole over the policy part (ticks 75..) above
the median of the first 50 ticks; shift_when_up = while a foot is > 1 cm up, the CoM's fraction
toward the other foot (-1 = over the R foot, +1 = over the L foot; L-up takes the min, R-up the max);
plus falls. grade.py's own episode runs unchanged; the soles and the CoM are read after every
policy tick through a wrapped env.step."""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

RUNS = ["robot_legs_prec16_f075_js20_s128r24", "robot_legs_prec37_f075_js20_sh35_s2t_s128r24"]
PROBES = ["stride1 0.3 robot", "steps5 0.3 robot"]
VARIANTS = {"grade as is": {},
            "roll lash 1.25 deg (train DR centre)": {"GRADE_ROLL_LASH": "1.25"},
            "series spring hip 14.2 N m/rad": {"GRADE_ROLL_SERIES": "hip=14.2"}}
_FT = []


def _install():
    import grade as G
    import simh
    if getattr(simh, "_wt_wrapped", False):
        return G
    b0 = simh.build

    def build(*a, **k):
        env, act, zoff = b0(*a, **k)
        s0 = env.step

        def step(a_):
            out = s0(a_)
            d, sg = env.data, env._sole_gids
            _FT.append((d.geom_xpos[sg[0]][2], d.geom_xpos[sg[1]][2], d.geom_xpos[sg[0]][0], d.geom_xpos[sg[0]][1],
                        d.geom_xpos[sg[1]][0], d.geom_xpos[sg[1]][1],
                        d.subtree_com[env._torso_bid][0], d.subtree_com[env._torso_bid][1]))
            return out
        env.step = step
        return env, act, zoff
    simh.build = build
    loop0 = G._episode_loop

    def loop(*a, **k):
        _FT.clear()              # the settle before the policy is not counted (grade_recover's FT)
        return loop0(*a, **k)
    G._episode_loop = loop
    simh._wt_wrapped = True
    return G


def wt_metrics(ft):
    """grade_recover.py's lift_cm / shift_when_up from the per-tick sole and CoM record."""
    ft = np.asarray(ft, float)
    z0 = np.median(ft[:50, 0:2], axis=0)
    lift = ft[75:, 0:2] - z0
    L, R, C = ft[75:, 2:4], ft[75:, 4:6], ft[75:, 6:8]
    mid, u = 0.5 * (L + R), L - R
    half = 0.5 * np.linalg.norm(u, axis=1)
    u = u / np.maximum(2 * half, 1e-6)[:, None]
    f = np.sum((C - mid) * u, axis=1) / np.maximum(half, 1e-6)
    upL, upR = lift[:, 0] > 0.01, lift[:, 1] > 0.01
    return dict(lift_cm=(lift.max(0) * 100).tolist(),
                shift_when_up=[float(f[upL].min()) if upL.any() else None, float(f[upR].max()) if upR.any() else None])


def job(args):
    run, vname, probe, seed = args
    for k in ("GRADE_ROLL_LASH", "GRADE_ROLL_SERIES"):
        os.environ.pop(k, None)
    os.environ.update(VARIANTS[vname])
    G = _install()
    p = G.PROBES[probe]
    spec = dict(run=run, shaper_hz=GC.SHAPER_HZ, stand_phase=np.pi / 2, clock="coast")
    spec.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
    m = G.episode(spec, seed=seed, run_s=p["run_s"], hold_s=p.get("hold_s", 0.0))
    m = dict(fell=bool(m["fell"]), trip=m.get("trip"), dx=m.get("dx"), **wt_metrics(_FT))
    return run, vname, probe, seed, m


def table(res, runs, seeds):
    L = ["| student | plant | probe | lift L / R cm, mean (min) | shift_when_up L-up / R-up | falls / trips |",
         "|---|---|---|---|---|---|"]
    for run in runs:
        for vn in VARIANTS:
            for pr in PROBES:
                e = [m for (r, v, p, s), m in res.items() if r == run and v == vn and p == pr]
                if not e:
                    continue
                lf = np.array([m["lift_cm"] for m in e])
                sh = np.array([[np.nan if x is None else x for x in m["shift_when_up"]] for m in e], float)
                L.append(f"| {run.replace('robot_legs_', '').replace('_s128r24', '')} | {vn} | {pr} | "
                         f"{lf[:, 0].mean():.2f} ({lf[:, 0].min():.2f}) / {lf[:, 1].mean():.2f} ({lf[:, 1].min():.2f}) | "
                         f"{np.nanmean(sh[:, 0]):+.2f} / {np.nanmean(sh[:, 1]):+.2f} | "
                         f"{sum(m['fell'] for m in e)}/{sum(m['trip'] is not None for m in e)} of {len(e)} |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("seeds", nargs="?", type=int, default=8)
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "24")))
    a = ap.parse_args(argv)
    runs = [r for r in RUNS if os.path.exists(os.path.join(GC.run_dir(r), "params.pkl"))]
    jobs = [(r, v, p, s) for r in runs for v in VARIANTS for p in PROBES for s in range(a.seeds)]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        res = {(r, v, p, s): m for r, v, p, s, m in ex.map(job, jobs)}
    md = table(res, runs, a.seeds)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump([dict(run=r, variant=v, probe=p, seed=s, **m) for (r, v, p, s), m in res.items()],
                  open(a.json, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
