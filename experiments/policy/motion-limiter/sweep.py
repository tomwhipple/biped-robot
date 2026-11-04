"""Motion-limiter sweep on the shaper-train grader (2026-11-03): one run through the deployed
chain (C2 shaper -> takeover ramp -> obs::MotionLimiter -> fast STS3215 -> the joint-fit plant),
the limiter at a few settings, on the probes where the robot's first walk step went wrong.

  CUDA_VISIBLE_DEVICES= python sweep.py [seeds] [out.json]
  RUN=<run> LIMITS='off;200,1000;200,2000' ONLY='stride1 0.6,walk 0.3 -> stop' python sweep.py 8

Per cell (mean over seeds, worst in brackets where it matters): falls / flail-guard trips,
the guard's EMA body rate, the raw peak body YAW rate (rad/s) and yaw over the window (deg),
the written goals' peak acceleration (deg/s^2 -- what the limiter bounds), the plant's peak
joint deceleration (10 ms window, physics substeps, deg/s^2) and peak joint speed, forward
travel. Window: the first walk command to 2.5 s after the last command change."""
from __future__ import annotations
import json, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "shaper-train"))
import grade as G  # noqa: E402

RUN = os.environ.get("RUN", "robot_legs_walk18_shp10sq_s128r24")
LIMITS = os.environ.get("LIMITS", "off;200,1000;200,2000;200,3000;200,4000;300,4000;"
                                  "200,2000,50000;200,2000,20000").split(";")
PROBES = [p for p in os.environ.get("ONLY", "stride1 0.6,walk 0.3 -> stop,stand").split(",") if p]
KEYS = ("peak_rate", "yaw_rate_peak", "yaw_deg_2s5", "goal_acc_peak_dps2",
        "joint_decel_peak_dps2", "joint_vel_peak_dps", "walk_dx_2s5", "goal_rms_mean",
        "squeeze_deg", "power_w")


def job(a):
    lim, pn, seed = a
    p = G.PROBES[pn]
    v = dict(run=RUN, shaper_hz=10.0, stand_phase=G.P2, clock="coast",
             limit=None if lim == "off" else lim)
    v.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
    try:
        m = G.episode(v, seed=seed, run_s=p["run_s"], hold_s=p.get("hold_s", 0.0))
    except Exception as e:  # noqa: BLE001
        m = dict(seed=seed, error=repr(e), fell=True, trip=("error", 0, 0))
    m.update(limit=lim, probe=pn)
    return m


def summarize(res):
    rows = []
    for lim in LIMITS:
        for pn in PROBES:
            e = [r for r in res if r["limit"] == lim and r["probe"] == pn]
            if not e:
                continue
            row = dict(limit=lim, probe=pn, n=len(e), falls=sum(bool(r["fell"]) for r in e),
                       trips=sum(1 for r in e if r.get("trip")),
                       trip_kinds=sorted({r["trip"][0] for r in e if r.get("trip")}))
            for k in KEYS:
                vals = np.array([r[k] for r in e if k in r and r[k] is not None], float)
                if vals.size:
                    row[k] = float(np.nanmean(vals))
                    row[k + "_worst"] = float(np.nanmax(np.abs(vals)))
            rows.append(row)
    return rows


def _f(r, k, nd=0, worst=False):
    if k not in r:
        return "—"
    s = f"{r[k]:.{nd}f}"
    return s + (f" ({r[k + '_worst']:.{nd}f})" if worst else "")


def table(rows):
    L = ["| limit (vmax, amax[, jmax]) | probe | falls / trips | guard rate | peak yaw rate, rad/s | "
         "yaw, deg | goal accel peak, deg/s² | joint decel peak, deg/s² | joint speed peak, deg/s | "
         "dx 2.5 s, m |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        tk = f" [{' '.join(r['trip_kinds'])}]" if r["trip_kinds"] else ""
        L.append(f"| {r['limit']} | {r['probe']} | {r['falls']}/{r['trips']}{tk} of {r['n']} | "
                 f"{_f(r, 'peak_rate', 2, True)} | {_f(r, 'yaw_rate_peak', 2, True)} | "
                 f"{_f(r, 'yaw_deg_2s5', 1)} | {_f(r, 'goal_acc_peak_dps2', 0, True)} | "
                 f"{_f(r, 'joint_decel_peak_dps2', 0, True)} | {_f(r, 'joint_vel_peak_dps', 0)} | "
                 f"{_f(r, 'walk_dx_2s5', 2)} |")
    L.append("")
    L.append("Mean over seeds, worst seed in brackets. `stand` rows: the window is the whole 20 s "
             "run; goal jitter / squeeze are in the json.")
    return "\n".join(L)


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "sweep.json")
    jobs = [(lim, pn, s) for lim in LIMITS for pn in PROBES for s in range(seeds)]
    with ProcessPoolExecutor(max_workers=int(os.environ.get("WORKERS", "24"))) as ex:
        res = list(ex.map(job, jobs, chunksize=1))
    rows = summarize(res)
    json.dump(dict(run=RUN, rows=rows, episodes=res), open(out, "w"), indent=1, default=str)
    md = table(rows)
    open(os.path.splitext(out)[0] + ".md", "w").write(f"run `{RUN}`, shape 10 Hz, fast servo\n\n" + md + "\n")
    print(md)


if __name__ == "__main__":
    main()
