"""Does low foot-floor friction reproduce the robot's stride1 spin (2026-11-03)?

  python sweep.py [seeds] [out.json]     RUN=... (default walk18_shp10sq_s128r24)

stride1 0.6 through grade.py's firmware-path episode (fast servo, C2 shaper 10 Hz, standclock
coast pi/2, no obsfreeze) with the EFFECTIVE contact friction forced on the floor AND the sole
pads (grade.set_contact): mu x torsion grid. The robot: yaw -19 deg at +180 ms, |gyro z| peak
3.9 rad/s, flail guard rate trip at +350 ms (1.74 rad/s), fell."""
from __future__ import annotations
import json, os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "shaper-train"))
import grade as G  # noqa: E402

MUS = [float(x) for x in os.environ.get("MUS", "1.0,0.8,0.6,0.5,0.4,0.3").split(",")]
TORS = [float(x) for x in os.environ.get("TORS", "0.02,0.01,0.005").split(",")]
VX = float(os.environ.get("VX", "0.6"))


def job(a):
    run, mu, tor, seed = a
    v = dict(run=run, shaper_hz=float(os.environ.get("SHAPE_HZ", "10")), stand_phase=G.P2,
             clock="coast", cmd_schedule=G.stride(VX), contact=dict(mu=mu, torsion=tor),
             trace=seed == 0)
    if os.environ.get("TYPED"):     # servo override on the fast model, e.g. dead15=0,tau15=0
        v["typed"] = dict(G.FAST, **{k: float(x) for k, x in
                                     (kv.split("=") for kv in os.environ["TYPED"].split(","))})
    if os.environ.get("PLANT"):     # plant knobs on top of JF_PLANT (simh.apply_model), e.g. kp_scale=1.5
        v["plant"] = json.loads(os.environ["PLANT"]) if os.environ["PLANT"].startswith("{") else \
            {k: float(x) for k, x in (kv.split("=") for kv in os.environ["PLANT"].split(","))}
    try:
        m = G.episode(v, seed=seed, run_s=6.0)
    except Exception as e:  # noqa: BLE001
        m = dict(error=repr(e), fell=True)
    m.update(run=run, mu=mu, torsion=tor, seed=seed)
    tr = m.get("trace")
    if tr:   # the robot's R hip pitch / knee at +180 ms: -31 / -31 deg (11-03 stride1)
        q = np.array(tr["q"]); m["rhip_180"], m["rknee_180"] = float(q[9, 8]), float(q[9, 9])
    return m


def _hk(e):
    x = next((x for x in e if "rhip_180" in x), None)
    return "-" if x is None else f"{x['rhip_180']:+.0f} / {x['rknee_180']:+.0f}"


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "sweep.json")
    runs = os.environ.get("RUN", "robot_legs_walk18_shp10sq_s128r24").split(",")
    jobs = [(r, mu, t, s) for r in runs for mu in MUS for t in TORS for s in range(seeds)]
    with ProcessPoolExecutor(max_workers=int(os.environ.get("WORKERS", "24"))) as ex:
        res = list(ex.map(job, jobs, chunksize=1))
    json.dump(res, open(out, "w"), indent=1, default=str)
    L = ["| run | mu | torsion | falls | guard trips [rate/pose/tilt/chatter] | first trip (s after cmd) | "
         "peak rate EMA mean/max | peak abs gyro z mean/max | yaw @+180 ms mean [min..max] (deg) | max abs yaw 1 s (deg) | "
         "R hip / knee @+180 ms, seed 0 (robot -31 / -31) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in runs:
        for mu in MUS:
            for t in TORS:
                e = [x for x in res if x["run"] == r and x["mu"] == mu and x["torsion"] == t]
                if not e:
                    continue
                bk = {k: sum(1 for x in e if k in (x.get("trips_by") or {})) for k in ("rate", "pose", "tilt", "chatter")}
                ft = [x["trip"][1] - 2.0 for x in e if x.get("trip")]
                y = [x.get("walk_yaw_180ms", np.nan) for x in e]
                L.append(f"| {r} | {mu:g} | {t:g} | {sum(x['fell'] for x in e)}/{len(e)} | "
                         f"{sum(1 for x in e if x.get('trip'))} [{bk['rate']}/{bk['pose']}/{bk['tilt']}/{bk['chatter']}] | "
                         f"{(min(ft) if ft else float('nan')):.2f} | "
                         f"{np.nanmean([x.get('peak_rate', np.nan) for x in e]):.2f}/{np.nanmax([x.get('peak_rate', np.nan) for x in e]):.2f} | "
                         f"{np.nanmean([x.get('walk_gz_max', np.nan) for x in e]):.2f}/{np.nanmax([x.get('walk_gz_max', np.nan) for x in e]):.2f} | "
                         f"{np.nanmean(y):+.1f} [{np.nanmin(y):+.1f}..{np.nanmax(y):+.1f}] | "
                         f"{np.nanmean([x.get('walk_yaw_max', np.nan) for x in e]):.1f} | "
                         f"{_hk(e)} |")
    md = "\n".join(L)
    open(os.path.splitext(out)[0] + ".md", "w").write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
