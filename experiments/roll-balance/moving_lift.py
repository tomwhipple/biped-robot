"""The moving lift (#297; README "The moving lift (2026-11-24)"): Variant M's timing against the same step with the
lift-off moved into the shift's min-jerk tail (dwell.Dwell moving_lift), through the firmware's track + trim + lift gate
(sil_gait.run) on the per-side play plant, the foot-off sag plant and the inset sole, march / fwd 20 / fwd 40 mm, both first feet,
and the pelvis amplitude k lowered (1.30 -> 1.20 -> 1.10) as the hold margin: the lowest k that still holds.

    python moving_lift.py                       # the grid, 8 processes -> results/moving_lift/grid.json + summary.md
    python moving_lift.py --cfg ml30 --dirs march --plants sag --amps 1.3 --first R   # one cell
    python moving_lift.py --ladder              # the first robot ladder's lines -> results/moving_lift/ladder.md
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("JAX_PLATFORMS", "cpu")

OUT = os.path.join(HERE, "results", "moving_lift")
# Variant M (DESIGN_moving_lift_gait.md 4.1; held on the robot 2026-11-16, ~450 steps at 3.47 s with shift 1.3)
VM = dict(T=4.0, td_pause=0.3, shift_s=1.2, settle=0.1, raise_s=0.77, move_s=0.45, lower_s=0.55)
CFGS = {
    "vm": dict(VM),
    "vm13": dict(VM, shift_s=1.3),                                            # the robot's held 3.47 s
    "ml20": {**{k: v for k, v in VM.items() if k != "settle"}, "moving_lift": 0.2},
    "ml30": {**{k: v for k, v in VM.items() if k != "settle"}, "moving_lift": 0.3},
    "ml40": {**{k: v for k, v in VM.items() if k != "settle"}, "moving_lift": 0.4},
    # the robot's held Variant M (shift 1.3) with its settle replaced by the moving lift: the one-change robot runs
    "ml20s13": {**{k: v for k, v in VM.items() if k != "settle"}, "shift_s": 1.3, "moving_lift": 0.2},
    "ml30s13": {**{k: v for k, v in VM.items() if k != "settle"}, "shift_s": 1.3, "moving_lift": 0.3},
    # the same with the move and the lowering cut back (the raise stays 0.77 s: the release, design note 3)
    "ml40f": {**{k: v for k, v in VM.items() if k != "settle"}, "moving_lift": 0.4, "move_s": 0.3, "lower_s": 0.45},
}
PLANTS = {"play": "grade+play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0",
          "sag": "grade+sag_L=6.4,sag_R=2.9",
          # the inset sole on the play plant: the static inner edge (README "The inset sole"); not with the sag
          "inset": "grade+play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0,sole_in_L=8,sole_in_R=-12"}
DIRS = {"march": dict(dx=0.0), "fwd20": dict(dx=0.02), "fwd40": dict(dx=0.04, swing_tilt=3.0)}
N_STEPS = 10


def one(job):
    cfg, dname, plant, amp, first, delay = job
    import dwell as DW
    import sil_gait as G
    G.PLANTS.update(PLANTS)
    c = dict(CFGS[cfg])
    T = c.pop("T")
    d = dict(DIRS[dname])
    tilt = d.pop("swing_tilt", 0.0)
    gkw = dict(pelvis_x="stance", ankle_ff=(2.0, 2.0), **c, **({"swing_tilt": tilt} if tilt else {}))
    try:
        r = G.run(T, "march", plant=plant, extra_ms=delay, n_steps=N_STEPS, first=first, amp=(amp, amp), seed=(5.2, 3.4),
                  model="dwell", gkw=gkw, step_kw=d, lift_gate=True)
    except Exception as e:  # noqa: BLE001
        return dict(cfg=cfg, dir=dname, plant=plant, amp=amp, first=first, delay=delay, error=repr(e)[:300])
    sw = DW.Dwell(T=T, n_steps=N_STEPS, kL=amp, kR=amp, first=first, dt=0.1, **d, **gkw)
    per = {}
    for s in "LR":
        rep = [x for x in r["reports"] if x["stance"] == s]
        ph = [p for p in r["phases"] if p["stance"] == s]
        lo = [x for x in sw.liftoff if x["stance"] == s]
        per[s] = dict(lean_in=round(max((x["lean_in"] for x in rep), default=float("nan")), 1),
                      n=len(rep), tips=sum(x["lean_in"] >= TIP_DEG for x in rep),
                      lean_out=round(max((x["lean_out"] for x in rep), default=float("nan")), 1),
                      com_in_lo_mm=round(max((p["com_in_lo_mm"] for p in ph if p["com_in_lo_mm"] is not None),
                                             default=float("nan")), 1),
                      ratio_lo=round(min((x["ratio"] for x in lo), default=float("nan")), 3),
                      dcm_lo=round(min((x.get("dcm", x["ratio"]) for x in lo), default=float("nan")), 3))
    g = r["gate"]
    return dict(cfg=cfg, dir=dname, plant=plant, amp=amp, first=first, delay=delay, step_s=round(sw.step_time, 2),
                fell=r["fell"], fall_dir=r["fall_dir"], t_fall=r["t_fall"], aborts=r["aborts"],
                lifts=f"{r['lifts']}/{len(r['phases'])}", peak_tilt=r["peak_tilt"],
                gate=dict(holds=g["holds"], held_ms=g["held_ms"], timeouts=g["timeouts"]), per=per)


def verdict(r):
    if "error" in r:
        return "error"
    if r["fell"]:
        return f"FELL {r['fall_dir']}"
    if r["gate"]["timeouts"]:
        return "gate timeout"
    if max(r["per"][s]["lean_in"] for s in "LR") >= TIP_DEG:
        return "tip"
    return "clean"


TIP_DEG = 7.0       # a single support whose lean toward the lifted foot (the firmware's TS line) reaches this is a tip


def summary(rows):
    out = ["| cfg | step s | plant | walk | k | runs clean | tips L / R (single supports) | lean in max L / R | "
           "lean out max L / R | CoM inboard at lift-off, max L / R (mm) | plan k at lift-off / DCM, min | gate holds (runs) |",
           "|" + "---|" * 12]
    groups = {}
    for r in rows:
        groups.setdefault((r["cfg"], r["plant"], r["dir"], r["amp"]), []).append(r)
    for key in sorted(groups, key=lambda k: (list(CFGS).index(k[0]), k[1], list(DIRS).index(k[2]), -k[3])):
        rs = [r for r in groups[key] if "error" not in r]
        if not rs:
            out.append(f"| {key[0]} | | {key[1]} | {key[2]} | {key[3]:g} | error |||||||")
            continue
        P = {s_: [r["per"][s_] for r in rs] for s_ in "LR"}
        out.append(f"| {key[0]} | {rs[0]['step_s']:.2f} | {key[1]} | {key[2]} | {key[3]:g} | "
                   f"{sum(verdict(r) == 'clean' for r in rs)}/{len(groups[key])} | "
                   f"{sum(x['tips'] for x in P['L'])}/{sum(x['n'] for x in P['L'])} / "
                   f"{sum(x['tips'] for x in P['R'])}/{sum(x['n'] for x in P['R'])} | "
                   f"{max(x['lean_in'] for x in P['L'])} / {max(x['lean_in'] for x in P['R'])} | "
                   f"{max(x['lean_out'] for x in P['L'])} / {max(x['lean_out'] for x in P['R'])} | "
                   f"{max(x['com_in_lo_mm'] for x in P['L'])} / {max(x['com_in_lo_mm'] for x in P['R'])} | "
                   f"{min(min(x['ratio_lo'] for x in P[s_]) for s_ in 'LR'):.2f} / "
                   f"{min(min(x['dcm_lo'] for x in P[s_]) for s_ in 'LR'):.2f} | "
                   f"{sum(r['gate']['holds'] for r in rs)} ({sum(r['gate']['holds'] > 0 for r in rs)}) |")
    # the hold margin: per cfg x plant x walk, the lowest k (from 1.3 down) with every run clean
    out += ["", "| cfg | plant | walk | clean at k (every run) | lowest clean k |", "|---|---|---|---|---|"]
    keys = sorted({(r["cfg"], r["plant"], r["dir"]) for r in rows}, key=lambda k: (list(CFGS).index(k[0]), k[1], list(DIRS).index(k[2])))
    for key in keys:
        rs = [r for r in rows if (r["cfg"], r["plant"], r["dir"]) == key]
        amps = sorted({r["amp"] for r in rs}, reverse=True)
        ok = [a for a in amps if all(verdict(r) == "clean" for r in rs if r["amp"] == a)]
        low = None
        for a in amps:               # contiguous from the top
            if a in ok:
                low = a
            else:
                break
        out.append(f"| {key[0]} | {key[1]} | {key[2]} | {', '.join(f'{a:g}' for a in ok) or 'none'} | "
                   f"{'none' if low is None else f'{low:g}'} |")
    return "\n".join(out)


# the first robot ladder (README "The moving lift"): track_host's own steps (the yaw trim included), first foot R, 6 / 8 steps
LADDER = [("VM march (baseline)", "vm13", "march", 6), ("ML 0.2 march", "ml20s13", "march", 6),
          ("ML 0.2 fwd 20", "ml20s13", "fwd20", 8), ("ML 0.3 march", "ml30s13", "march", 6),
          ("ML 0.3 fwd 20", "ml30s13", "fwd20", 8)]


def ladder_one(job):
    name, cfg, dname, n, plant, delay = job
    import sil_gait as G
    import track_host as H
    G.PLANTS.update(PLANTS)
    c = dict(CFGS[cfg])
    T = c.pop("T")
    dx = DIRS[dname]["dx"]
    kw, _ = H.sil_step_kw(cmd=H.step_cmd(1e3 * dx, 0.0, 0.0))
    r = G.run(T, "march", plant=plant, extra_ms=delay, n_steps=n, first="R", amp=(1.3, 1.3), seed=H.ROBOT_SEED,
              model="dwell", gkw=dict(pelvis_x=H.ROBOT_PELVIS_X, ankle_ff=H.ROBOT_ANKLE_FF, **c), step_kw=kw, lift_gate=True)
    li = {s: max((x["lean_in"] for x in r["reports"] if x["stance"] == s), default=float("nan")) for s in "LR"}
    lo = {s: max((x["lean_out"] for x in r["reports"] if x["stance"] == s), default=float("nan")) for s in "LR"}
    return (f"| {name} | {plant} | {delay:g} | {'FELL ' + r['fall_dir'] if r['fell'] else ('gate timeout' if r['gate']['timeouts'] else 'done')} "
            f"| {r['lifts']}/{len(r['phases'])} | {li['L']:.1f} / {li['R']:.1f} | {lo['L']:.1f} / {lo['R']:.1f} "
            f"| {r['gate']['holds']} ({r['gate']['held_ms']} ms) |")


def ladder(procs=8):
    jobs = [(nm, c, d, n, p, dl) for nm, c, d, n in LADDER for p in PLANTS for dl in (0.0, 60.0)]
    with ProcessPoolExecutor(min(procs, 8)) as ex:
        out = list(ex.map(ladder_one, jobs))
    return "\n".join(["| line | plant | delay ms | result | lifts | lean in max L / R | lean out max L / R | gate holds |",
                      "|---|---|---|---|---|---|---|---|"] + out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cfg", default=",".join(CFGS))
    ap.add_argument("--dirs", default=",".join(DIRS))
    ap.add_argument("--plants", default=",".join(PLANTS))
    ap.add_argument("--amps", default="1.3,1.2,1.1")
    ap.add_argument("--first", default="R,L")
    ap.add_argument("--delays", default="0,60", help="ms of extra roll latency (sil_gait's 0 / 60)")
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--out", default=os.path.join(OUT, "grid.json"))
    ap.add_argument("--ladder", action="store_true", help="the first robot ladder's lines only -> results/moving_lift/ladder.md")
    a = ap.parse_args(argv)
    if a.ladder:
        md = ladder(a.procs)
        os.makedirs(OUT, exist_ok=True)
        open(os.path.join(OUT, "ladder.md"), "w").write(md + "\n")
        print(md)
        return
    jobs = [(c, d, p, float(k), f, dl) for c in a.cfg.split(",") for d in a.dirs.split(",") for p in a.plants.split(",")
            for k in a.amps.split(",") for f in a.first.split(",") for dl in [float(x) for x in a.delays.split(",")]]
    with ProcessPoolExecutor(min(a.procs, 8)) as ex:
        rows = []
        for r in ex.map(one, jobs):
            rows.append(r)
            print(json.dumps({k: v for k, v in r.items() if k != "per"}), verdict(r), flush=True)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(rows, open(a.out, "w"), separators=(",", ":"))
    md = summary(rows)
    open(os.path.splitext(a.out)[0] + ".md", "w").write(md + "\n")
    print(md)


if __name__ == "__main__":
    main()
