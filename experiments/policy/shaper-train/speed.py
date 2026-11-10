"""Joint-speed table for the prec line (grade.py's episode with the speed stats added).

Per probe and student, over the walk window (first walk command .. the stand command, ticks at 50 Hz):
mean |dq| per knee / hip pitch (deg/s), p95 knee, peak, the share of walk time any knee or hip pitch
is above 180 deg/s, the knee range L/R (deg), dx, dyaw.
Usage (GRADE_XML / JAX_PLATFORMS=cpu as for grade.py):
  RUNS=a,b [CROUCH=0.9] SHAPE_HZ=8 python speed.py <seeds> <out.json> [probe,...]
CROUCH sets the crouch command channel (cmd[3]) wherever a probe sends the default 1.0.
Planner 2026-11-08 (the prec line: per-joint MEAN speed, not the peak, is the yardstick).
"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
out_arg = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else None
os.chdir(HERE)
import numpy as np
import grade as G
from concurrent.futures import ProcessPoolExecutor

CROUCH = float(os.environ.get("CROUCH", "1.0"))
if CROUCH != 1.0:
    import walker_env as WE
    _sc0 = WE.BimoWalkerEnv.set_command if hasattr(WE, "BimoWalkerEnv") else None

    def _sc(self, *ch):
        ch = list(ch)
        if len(ch) >= 4 and ch[3] == 1.0:
            ch[3] = CROUCH
        return _sc0(self, *ch)
    WE.BimoWalkerEnv.set_command = _sc

IDX = {n: i for i, n in enumerate(G.rec.POL)}
_m0 = G._episode_metrics


def _m(env, GO, GY, PW, TL, SQ, X, fell, fg, sched, n, nh, seed, QV, WZ, v=None, hz=None, **ext):
    m = _m0(env, GO, GY, PW, TL, SQ, X, fell, fg, sched, n, nh, seed, QV, WZ, v=v, hz=hz, **ext)
    walks = [ts for ts, c in sched if not G.S.stand_gate(c)]
    if not walks:
        return m
    i0 = int(round(walks[0] / G.DT))
    i1 = int(round(max(ts for ts, _ in sched) / G.DT))
    qd = np.degrees(np.abs(np.array(ext["QD"])))[i0:i1]
    qm = np.degrees(np.array(ext["QM"]))[i0:i1]
    if len(qd) < 5:
        return m
    s = {}
    for j in ("L_knee", "R_knee", "L_hip_pitch", "R_hip_pitch", "L_hip_roll", "R_hip_roll"):
        s[j + "_mean"] = float(qd[:, IDX[j]].mean())
    kn = qd[:, [IDX["L_knee"], IDX["R_knee"]]]
    hp = qd[:, [IDX["L_hip_pitch"], IDX["R_hip_pitch"]]]
    s["knee_p95"] = float(np.percentile(kn, 95))
    s["knee_peak"] = float(kn.max())
    s["hip_peak"] = float(hp.max())
    s["frac_above180"] = float((np.concatenate([kn, hp], 1) > 180).any(1).mean())
    s["L_knee_range"] = float(np.ptp(qm[:, IDX["L_knee"]]))
    s["R_knee_range"] = float(np.ptp(qm[:, IDX["R_knee"]]))
    m["spd"] = s
    return m


G._episode_metrics = _m


def main():
    seeds = int(sys.argv[1]); out = out_arg
    probes = sys.argv[3].split(",") if len(sys.argv) > 3 else ["steps5 0.3 robot", "dir fwd 0.3"]
    hz = float(os.environ.get("SHAPE_HZ", "8"))
    runs = [r for r in os.environ["RUNS"].split(",") if r]
    variants = {f"{r} @ shape {hz:g}": dict(run=r, shaper_hz=hz) for r in runs}
    jobs = [(vn, pn, s, v, G.PROBES[pn]) for vn, v in variants.items() for pn in probes for s in range(seeds)]
    with ProcessPoolExecutor(max_workers=int(os.environ.get("WORKERS", "12"))) as ex:
        res = list(ex.map(G.job, jobs, chunksize=1))
    json.dump(res, open(out, "w"), default=str)
    keys = ["L_knee_mean", "R_knee_mean", "L_hip_pitch_mean", "R_hip_pitch_mean", "knee_p95", "knee_peak",
            "hip_peak", "frac_above180", "L_knee_range", "R_knee_range"]
    print("| student | probe | falls | " + " | ".join(keys) + " | dx | dyaw |")
    print("|" + "---|" * (len(keys) + 5))
    for vn in variants:
        for pn in probes:
            e = [x for x in res if x["variant"] == vn and x["probe"] == pn]
            sp = [x["spd"] for x in e if "spd" in x]
            row = [f"{np.mean([s[k] for s in sp]):.2f}" if k == "frac_above180" else f"{np.mean([s[k] for s in sp]):.0f}"
                   for k in keys] if sp else ["-"] * len(keys)
            print(f"| {vn} | {pn} | {sum(bool(x['fell']) for x in e)}/{len(e)} | " + " | ".join(row)
                  + f" | {np.mean([x['dx'] for x in e]):+.2f} | {np.mean([x['dyaw'] for x in e]):+.0f} |")


if __name__ == "__main__":
    main()
