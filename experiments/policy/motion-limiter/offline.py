"""How much of a policy's commanded motion each limiter setting cuts, open loop (2026-11-03):
the run's own shaped + ramped goals from an unlimited closed-loop episode (grade.episode, limit
off) replayed through obs::MotionLimiter's numpy copy. The deviation is what a fine-tune with
--fw-limit has to absorb (the closed-loop sweep, sweep.py, shows the untrained run cannot).

  CUDA_VISIBLE_DEVICES= python offline.py [seeds]      -> offline.md / offline.json"""
from __future__ import annotations
import json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "shaper-train"))
import grade as G  # noqa: E402
import simh  # noqa: E402
import fw_limit as FL  # noqa: E402

RUN = os.environ.get("RUN", "robot_legs_walk18_shp10sq_s128r24")
LIMITS = ["200,1000", "200,2000", "200,3000", "200,4000", "250,3000", "300,4000",
          "200,2000,50000", "200,2000,20000"]
PROBES = ["stride1 0.6", "walk 0.3 -> stop"]


def goals(pn, seed):
    rec = []
    call0 = simh.TypedServo.__call__

    def tap(self, g):
        rec.append(np.array(g, float))
        return call0(self, g)
    simh.TypedServo.__call__ = tap
    try:
        p = G.PROBES[pn]
        v = dict(run=RUN, shaper_hz=10.0, stand_phase=G.P2, clock="coast")
        v.update({k: x for k, x in p.items() if k not in ("run_s", "hold_s")})
        G.episode(v, seed=seed, run_s=p["run_s"])
    finally:
        simh.TypedServo.__call__ = call0
    return np.array(rec)


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    out = {}
    for pn in PROBES:
        GS = [goals(pn, s) for s in range(seeds)]
        g0 = GS[0]
        v = np.diff(g0, axis=0) / G.DT
        a = np.diff(g0, 2, axis=0) / G.DT ** 2
        out[pn] = dict(goal_speed_peak_dps=float(np.degrees(np.abs(v).max())),
                       goal_acc_peak_dps2=float(np.degrees(np.abs(a).max())),
                       goal_speed_p99_dps=float(np.degrees(np.percentile(np.abs(v), 99))),
                       goal_acc_p99_dps2=float(np.degrees(np.percentile(np.abs(a), 99))))
        for lim in LIMITS:
            dev, bind = [], []
            for g in GS:
                L = FL.MotionLimiter(FL.parse(lim)); L.reset(g[0])
                y = np.array([L(x)[0] for x in g])
                dev.append(np.degrees(np.abs(y - g)))
                bind.append(np.mean(np.abs(y - g) > math.radians(0.5)))
            D = np.concatenate(dev)
            out[pn][lim] = dict(dev_max_deg=float(D.max()), dev_rms_deg=float(np.sqrt((D ** 2).mean())),
                                dev_p95_deg=float(np.percentile(D, 95)),
                                bound_frac=float(np.mean(bind)))
    json.dump(dict(run=RUN, probes=out), open(os.path.join(HERE, "offline.json"), "w"), indent=1)
    L = [f"run `{RUN}`, its unlimited goals (shape 10 Hz, fast servo), replayed through the limiter\n"]
    for pn, o in out.items():
        L.append(f"**{pn}**: unlimited goals peak {o['goal_speed_peak_dps']:.0f} deg/s "
                 f"(p99 {o['goal_speed_p99_dps']:.0f}), {o['goal_acc_peak_dps2']:.0f} deg/s² "
                 f"(p99 {o['goal_acc_p99_dps2']:.0f})\n")
        L.append("| limit | deviation max, deg | RMS, deg | p95, deg | joint-ticks > 0.5 deg off |")
        L.append("|---|---|---|---|---|")
        for lim in LIMITS:
            r = o[lim]
            L.append(f"| {lim} | {r['dev_max_deg']:.1f} | {r['dev_rms_deg']:.2f} | "
                     f"{r['dev_p95_deg']:.2f} | {100 * r['bound_frac']:.0f} % |")
        L.append("")
    md = "\n".join(L)
    open(os.path.join(HERE, "offline.md"), "w").write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
