"""The B1 firmware's first hardware runs (2026-11-12 afternoon, plain `pose ... S<L|R>[h]` over Wi-Fi)
replayed in the sim: what the sim predicts for exactly those moves, against what the robot did.

Per run (hw_sessions/2026-11-12/fwtrim_*.json gives the UTC window), from the flight record:
- the robot's NOMINAL targets each tick (the pose stream before the trim) drive the sim plant;
- the trim in the sim is the Python twin of the firmware (sim/stance_trim.py), driven by the robot's own
  trim state transitions (CLOSED / HOLD / FADE at the same ticks), with the robot's learned T at each
  chain's start and the chain's first move ramping it in (that move's min-jerk progress, as the firmware);
- the roll comes back one tick late.
Compared per chain: the peak lean toward the lifted foot over the CLOSED part, e / hike / T where the
hold begins, the end of the run, the swing sole's clearance over the hold (sim only) and falls.

    python fw_replay.py [--plants fit,lo,hi]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import testbed as TB  # noqa: E402

HW = "/home/claw/code/robot/hw_sessions"
FREC = os.path.join(HW, "2026-11-09", "flight_20261112T215156Z_prec16.frec")
RUNS = ["fwtrim_lift1", "fwtrim_lift2", "fwtrim_lift3", "fwtrim_lift4_L", "fwtrim_march1"]
PLANTS = {"fit": TB.BOARD, "lo": "grade+play_L_hip=4,play_R_hip=6,play_L_ankle=2,play_R_ankle=0",
          "hi": "grade+play_L_hip=10,play_R_hip=10,play_L_ankle=3,play_R_ankle=2"}
FW_START_S = 0.07
D = math.degrees


def robot_runs():
    import flight_reader as FR
    S = FR.load(FREC)
    ticks = FR.tick_rows(S)
    evs = FR.event_rows(S)
    out = {}
    for name in RUNS:
        w = json.load(open(os.path.join(HW, "2026-11-12", name + ".json")))
        u0, u1 = w["u0"], w["u1"]
        rows = [r for r in ticks if u0 - 0.5 <= r["t_utc"] <= u1 + 0.5]
        t = np.array([r["t_utc"] - u0 for r in rows])
        nom = np.array([[r.get(f"{n}.nom_rad", np.nan) for n in TB.BUS] for r in rows], float)
        pos = np.array([[r.get(f"{n}.pos_rad", np.nan) for n in TB.BUS] for r in rows], float)
        for j in range(nom.shape[1]):            # forward fill; before the first nominal, the encoder
            last = pos[0, j]
            for i in range(len(nom)):
                if np.isfinite(nom[i, j]):
                    last = nom[i, j]
                else:
                    nom[i, j] = last
        cmds = [(e["t_utc"] - u0, e["text"]) for e in evs if e["kind"] == "cmd" and u0 - 0.5 <= e["t_utc"] <= u1 + 0.5]
        st = [r["bal_state_name"] for r in rows]
        side = [chr(r["bal_side"]) if r["bal_side"] else "" for r in rows]
        out[name] = dict(t=t, nom=nom, pos0=pos[0], cmds=cmds, state=st, side=side,
                         roll=np.array([-D(math.asin(max(-1.0, min(1.0, r["up_y"])))) for r in rows]),
                         err=np.array([D(r["bal_err"]) for r in rows]), hike=np.array([D(r["bal_u"]) for r in rows]),
                         T=np.array([D(r["bal_integral"]) for r in rows]))
    return out


def chains(R):
    """[(i_begin, i_hold, i_fade, i_off, side, ramp(t) start / ms)] from the robot's state transitions."""
    out, cur = [], None
    for i, s in enumerate(R["state"]):
        if s == "TRIM_CLOSED" and (cur is None or cur.get("fade") is not None):
            if cur is not None:
                out.append(cur)
            # the chain's first move: the S pose sent just before
            c = [(tc, txt) for tc, txt in R["cmds"] if txt.startswith("pose ") and " S" in txt and tc <= R["t"][i] + 0.05]
            tc, txt = c[-1]
            ms = next(int(x[1:]) for x in txt.split() if x[0] == "s" and x[1:].isdigit())
            cur = dict(begin=i, side=R["side"][i], t_cmd=tc, ms=ms, hold=None, fade=None, off=None)
        elif s == "TRIM_HOLD" and cur is not None and cur["hold"] is None:
            cur["hold"] = i
        elif s == "TRIM_FADE" and cur is not None and cur["fade"] is None:
            cur["fade"] = i
        elif s == "OFF" and cur is not None and cur["fade"] is not None and cur["off"] is None:
            cur["off"] = i
    if cur is not None:
        out.append(cur)
    return out


def sim_run(args):
    name, plant, R = args
    import gate_common as GC
    import rec
    import simh
    from stance_trim import StanceTrim
    t, nom = R["t"], R["nom"]
    ch = chains(R)
    _, env = TB.SR.build(PLANTS[plant], float(t[-1] - t[0]) + 10)
    pol = list(rec.POL)
    ix = [TB.BUS.index(n) for n in pol]
    simh.place_and_hold(env, np.clip(nom[0][ix], env._lo, env._hi), 1.5)
    cur = {"t": nom[0][ix].copy()}
    env._action_to_ctrl = lambda a_: cur["t"]
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    d = env.data
    pads = GC.pad_gids(env)
    tw = StanceTrim()
    prev_roll = None
    res = dict(err=[], hike=[], T=[], roll=[], clr=[])
    fell = None
    by_begin = {c["begin"]: c for c in ch}
    by_hold = {c["hold"]: c for c in ch if c["hold"] is not None}
    by_fade = {c["fade"]: c for c in ch if c["fade"] is not None}
    active = None
    for i in range(len(t)):
        up = GC.up_of(d.qpos[3:7])
        roll = -math.asin(max(-1.0, min(1.0, up[1])))
        meas = prev_roll if prev_roll is not None else roll
        prev_roll = roll
        if i in by_begin:
            c = by_begin[i]
            if tw.state in (5, 6):
                tw.release()
            tw.T[c["side"]] = math.radians(R["T"][i])          # the robot's learned T entering this chain
            tw.begin(c["side"], meas)
            active = c
        if i in by_hold:
            tw.hold()
        if i in by_fade:
            tw.release()
        ramp = 1.0
        if active is not None and active["hold"] is None or (active is not None and i < (active["hold"] or 10 ** 9)):
            u = (t[i] - active["t_cmd"] - FW_START_S) / (active["ms"] / 1e3)
            ramp = TB._minjerk(u)
        tw.update(meas, 0.02, ramp)
        tgt = {n: nom[i][TB.BUS.index(n)] for n in pol}
        if tw.state != 0 and tw.side:
            tgt[f"{tw.side}_hip_roll"] += tw.hip_joint()
        cur["t"] = np.clip(np.array([tgt[n] for n in pol]), env._lo, env._hi)
        env.step(a0)
        res["err"].append(D(tw.err) if tw.state != 0 else np.nan)
        res["hike"].append(D(tw.hike))
        res["T"].append(D(tw.T[active["side"]]) if active else 0.0)
        res["roll"].append(D(roll))
        c = GC.clearance(env, pads)
        res["clr"].append((c["L"], c["R"]))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        if tilt > TB.FALL_DEG and fell is None:
            fell = float(t[i])
            break
    n = len(res["err"])
    rows = []
    for c in ch:
        b, h = c["begin"], c["hold"] if c["hold"] is not None else (c["fade"] or len(t) - 1)
        f = c["fade"] if c["fade"] is not None else len(t) - 1
        s = -1.0 if c["side"] == "R" else 1.0
        sw = "L" if c["side"] == "R" else "R"
        rob_in = max(0.0, float(np.nanmax(s * R["err"][b:h])))
        rob_out = max(0.0, float(np.nanmax(-s * R["err"][b:h])))
        row = dict(run=name, plant=plant, side=c["side"], t_begin=round(float(t[b]), 2),
                   robot=dict(lean_in=rob_in, lean_out=rob_out, e_hold=float(R["err"][h]), hike_hold=float(R["hike"][h]),
                              T_hold=float(R["T"][h]), e_end=float(R["err"][f - 1]), hike_end=float(R["hike"][f - 1]),
                              T_end=float(R["T"][f - 1]), T_begin=float(R["T"][b])))
        if h < n and b < n:
            se = np.array(res["err"])
            row["sim"] = dict(lean_in=max(0.0, float(np.nanmax(s * se[b:h]))), lean_out=max(0.0, float(np.nanmax(-s * se[b:h]))),
                              e_hold=float(se[h]), hike_hold=res["hike"][h], T_hold=res["T"][h],
                              e_end=float(se[min(f, n) - 1]), hike_end=res["hike"][min(f, n) - 1], T_end=res["T"][min(f, n) - 1],
                              clr_hold_mm=1e3 * float(np.max([x[0 if sw == "L" else 1] for x in res["clr"][h:min(f, n)]] or [np.nan])),
                              clr_raise_end_mm=1e3 * float(res["clr"][h][0 if sw == "L" else 1]))
        else:
            row["sim"] = None
        rows.append(row)
    return dict(run=name, plant=plant, fell=fell, chains=rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plants", default="fit,lo,hi")
    ap.add_argument("--out", default=os.path.join(HERE, "results", "fw_replay_1112.json"))
    a = ap.parse_args(argv)
    runs = robot_runs()
    jobs = [(n, p, runs[n]) for n in RUNS for p in a.plants.split(",")]
    with ProcessPoolExecutor(min(15, len(jobs))) as ex:
        out = list(ex.map(sim_run, jobs))
    with open(a.out, "w") as f:
        json.dump(out, f)
    f1 = lambda x: "  -  " if x is None else f"{x:5.1f}"  # noqa: E731
    print("lean = peak toward the lifted foot over the closed part (deg); e/hike/T at the hold; clr = sim swing sole peak over the hold")
    print(f"{'run':16s} {'plant':5s} side  {'lean rob/sim':>13s}  {'e@hold rob/sim':>15s}  {'hike@hold':>12s}  "
          f"{'T@hold':>12s}  {'T end':>12s}  clr mm  sim fell")
    for r in out:
        for c in r["chains"]:
            rb, sm = c["robot"], c["sim"] or {}
            print(f"{r['run']:16s} {r['plant']:5s} {c['side']:4s} {rb['lean_in']:5.1f}/{f1(sm.get('lean_in'))}  "
                  f"{rb['e_hold']:+6.1f}/{f1(sm.get('e_hold'))}  {rb['hike_hold']:5.1f}/{f1(sm.get('hike_hold'))}  "
                  f"{rb['T_hold']:5.1f}/{f1(sm.get('T_hold'))}  {rb['T_end']:5.1f}/{f1(sm.get('T_end'))}  "
                  f"{f1(sm.get('clr_hold_mm'))}  {r['fell']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
