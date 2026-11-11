"""Check 1 of the weight-transfer gate (#259): the 2026-10-24 / 10-25 one-leg bench protocol, run on
the WALKING plant, numbers next to the robot's.

    python check1_oneleg.py [--plants train,train:lash0.5,train:lash2.0,grade] [--terms SPEC]
                            [--json out.json] [--md out.md] [--only floorR2,clrR,...]

What runs: every `pose ... s<ms> [B<side>]` the robot was sent in these serial logs, replayed with
its own timing (the firmware's "streaming" stamp, its own duration), through
experiments/one-leg-stand/simfit_sim.Replay -- the firmware streamer (quintic min-jerk from the
read-back position) and the bench roll PI (balance/roll_pi, with the gains the log prints: kp 0.25,
ki 0.5, ankle +1 / hip -1, clamp 10, abort 12, no gyro term on 10-25) -- on the walking plant
(gate_common: the prec16 training chain at its DR centre, or grade.py's robot-probe plant):

  floorR2   10-25 18:29Z, R stance, the model's shift (62 mm): 10 mm lift / 6 s with BR, lower,
            30 mm / 4 s with BR + hold (hw_sessions/2026-10-24/stand/floorR2_serial.log)
  clrR/clrL 10-25 19:11Z / 19:17Z, the same at 1.3x the shift (80.6 mm), each stance
  latR/latL 10-25 18:55Z / 19:04Z: at 0.5..1.4x the shift, both feet down, lift the other foot
            5 mm over 1.5 s with NO feedback and read the roll change (the unloading-tilt curve)
  clamp     10-24: one foot welded to the world, the other lifted 30 mm by hip pitch/knee/ankle,
            no shift: the steady torso tilt toward the lifted side (sim/roll_compliance.py's
            protocol on the walking plant; robot 5.2 deg R lifted / 5.0 deg L lifted)

Robot numbers are read from the same logs: the `balance` polls (~1 Hz: err, u, roll now), so the
robot's "peak" is the largest POLLED value; the sim is reported at the robot's own poll times
("polled peak") and continuously. Clearance on the robot is a person's eyes / the EMEET frames
(NOTES.md); in the sim it is the lowest sole pad of the lifted foot above the floor.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import types
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

R2D = 180.0 / math.pi
LOGS = {
    "floorR2": os.path.join(GC.HW, "2026-10-24", "stand", "floorR2_serial.log"),
    "clrR": os.path.join(GC.HW, "2026-10-25", "clrR_serial.log"),
    "clrL": os.path.join(GC.HW, "2026-10-25", "clrL_serial.log"),
    "latR": os.path.join(GC.HW, "2026-10-25", "latR_serial.log"),
    "latL": os.path.join(GC.HW, "2026-10-25", "latL_serial.log"),
}
STANCE = {"floorR2": "R", "clrR": "R", "clrL": "L", "latR": "R", "latL": "L"}
SHIFT_MM = {"floorR2": 62.0, "clrR": 80.6, "clrL": 80.6}
LAT_X = (0.5, 0.75, 1.0, 1.1, 1.2, 1.3, 1.4)          # NOTES.md: the shift ladder, x the model's 62 mm
# the robot's numbers as NOTES.md states them (a person's eyes / EMEET for clearance)
HW_NOTES = {
    "floorR2": dict(l10_clear="no (Tom: did NOT clear at 10 mm)", l30_clear="yes (frame: lifted foot visibly higher)",
                    l10_peak=-2.7, l30_peak=-5.2),
    "clrR": dict(l10_clear="not stated (NOTES' YES is the 30 mm hold)", l30_clear="yes, ~1-2 cm (pixel est.)",
                 l10_peak=-1.25, l30_peak=-3.17),
    "clrL": dict(l10_clear="not stated (as R)", l30_clear="yes, clear gap", l10_peak=+1.27, l30_peak=+4.05),
    "latR": dict(curve=[-1.91, -1.85, -1.69, -1.67, -1.45, -1.30, -1.42]),
    "latL": dict(curve=[+1.90, +1.75, +1.62, +1.56, +1.53, +1.35, +1.34]),
    "clamp": dict(R=5.2, L=5.0),
}
# one_leg.CAL: the as-built calibration the 10-25 poses were written for (zero, dir)
CAL = {"R_hip_roll": (2082, -1), "R_hip_pitch": (1997, 1), "R_knee": (2125, -1), "R_ankle": (2045, 1),
       "L_hip_roll": (1991, -1), "L_hip_pitch": (2058, 1), "L_knee": (1977, -1), "L_ankle": (2091, 1),
       "R_hip_yaw": (2184, 1), "L_hip_yaw": (2153, 1), "R_ankle_roll": (2108, 1), "L_ankle_roll": (1997, 1)}

_TS = re.compile(r"^(\d\d):(\d\d):(\d\d\.\d+)Z\s+(.*)$")


def _sec(h, m, s):
    return int(h) * 3600 + int(m) * 60 + float(s)


def parse_log(path):
    """-> dict(moves=[(t0, dur, ticks12, B)], polls=[dict(t, roll, err, u, state)], gains=dict)
    Times are seconds of the UTC day. A move's t0 is the firmware's "streaming" line (the
    stream start), its duration the one the firmware echoes."""
    moves, polls, gains = [], [], {}
    pending = None
    cur = None
    for line in open(path, errors="replace"):
        mm = _TS.match(line.rstrip("\n"))
        if not mm:
            continue
        t = _sec(*mm.groups()[:3])
        txt = mm.group(4)
        if txt.startswith("> pose "):
            tok = txt.split()[2:]
            ticks = [int(x) for x in tok[:12]]
            B = next((x[1] for x in tok if re.fullmatch(r"B[LR]", x)), None)
            pending = dict(ticks=ticks, B=B, t_cmd=t)
            continue
        m2 = re.search(r"pose -> 12 joints, SMOOTH minimum-jerk over (\d+) ms.*streaming", txt)
        if m2 and pending is not None:
            moves.append((t, int(m2.group(1)) / 1e3, pending["ticks"], pending["B"]))
            pending = None
            continue
        g = re.search(r"gains\s+kp ([\d.]+)\s+ki ([\d.]+) /s\s+ankle x([+-][\d.]+)\s+hip x([+-][\d.]+)\s+clamp ([\d.]+) deg\s+abort ([\d.]+)", txt)
        if g:
            gains = dict(kp=float(g.group(1)), ki=float(g.group(2)), ka=float(g.group(3)), kh=float(g.group(4)),
                         clamp_deg=float(g.group(5)), abort_deg=float(g.group(6)))
            kd = re.search(r"kd ([\d.]+)", txt)
            gains["kd"] = float(kd.group(1)) if kd else 0.0
        r = re.search(r"roll now ([+-]?[\d.]+) deg", txt)
        if r:
            cur = dict(t=t, roll=float(r.group(1)), err=None, u=None)
            polls.append(cur)
        e = re.search(r"ref ([+-]?[\d.]+) deg\s+err ([+-]?[\d.]+) deg\s+I [+-]?[\d.]+\s+u ([+-]?[\d.]+) deg", txt)
        if e and cur is not None:
            cur["ref"], cur["err"], cur["u"] = float(e.group(1)), float(e.group(2)), float(e.group(3))
    return dict(moves=moves, polls=polls, gains=gains)


def stand_of(log, stance):
    """A simfit_sim.Replay-compatible stand: moves from the log's first (zero) pose, t from it."""
    import simfit_hw as H
    t0 = log["moves"][0][0]
    mv = []
    for k, (t, dur, ticks, B) in enumerate(log["moves"]):
        mv.append(H.Move(name=f"m{k}", t0=t - t0, dur=dur, ticks=list(ticks), B=B))
    st = types.SimpleNamespace(cal=dict(CAL), side=stance, moves=mv, t_utc0=t0)
    return st, t0


def b_stages(st):
    """the B moves (the lifts with balance), in order: [(index, move)]"""
    return [(i, m) for i, m in enumerate(st.moves) if m.B]


def run_replay(job):
    """One log on one plant -> sim trace (dict of arrays) + the stand."""
    name, plant, terms = job
    import simfit_sim as SS
    log = parse_log(LOGS[name])
    st, t0 = stand_of(log, STANCE[name])
    g = log["gains"]
    ctl = SS.Ctl(kp=g.get("kp", 0.25), ki=g.get("ki", 0.5), kd=g.get("kd", 0.0), ka=g.get("ka", 1.0),
                 kh=g.get("kh", -1.0), clamp_deg=g.get("clamp_deg", 10.0), abort_deg=g.get("abort_deg", 12.0))
    if plant.startswith("oneleg"):
        # reference rows, NOT the walking plant: the one-leg fit's own harness plant
        # (sim/bimo_biped_v6legs.xml, simfit_sim.Plant), "oneleg:fit" = simfit_plant.json's fit,
        # "oneleg:base" = simfit_sim's training-plant defaults -- a check of this harness
        pl = SS.Plant(**oneleg_fit()) if plant == "oneleg:fit" else SS.Plant()
        SS.CTRL_DT = 0.024
        env = SS.make_env(pl)
        GC.apply_terms(env, terms)
    else:
        pl = SS.Plant()
        env = GC.make_plant(plant, terms=terms)
        if plant == "grade":        # grade's fit plant carries the L ankle-roll zero error; Replay owns zero errors
            pl = SS.Plant(zoff_La=-2.12)
        SS.CTRL_DT = env.control_dt
    env._zero_off = None
    pads = GC.pad_gids(env)
    clr = []
    step0 = env.step

    def step(a):
        out = step0(a)
        c = GC.clearance(env, pads)
        clr.append((c["L"], c["R"]))
        return out
    env.step = step
    orig_make = SS.make_env
    SS.make_env = lambda pl_, seed=0: env
    try:
        rp = SS.Replay(st, pl, ctl)
        alpha = getattr(env, "_gate_alpha", None)
        if alpha is not None and np.any(alpha > 0):
            enc0 = rp.encoder_rad
            def enc():     # a series spring: the encoder reads the servo shaft
                q = env.data.qpos[env._sqpos]
                return enc0() + (alpha * (env.data.ctrl - q))[rp.jidx]
            rp.encoder_rad = enc
        r = rp.run()
    finally:
        SS.make_env = orig_make
    clr = np.array(clr)
    n_pre = len(clr) - len(r["t"])
    r["clrL"], r["clrR"] = clr[n_pre:, 0], clr[n_pre:, 1]
    return name, plant, {k: np.asarray(v).tolist() for k, v in r.items() if k in
                         ("t", "roll", "err", "u", "state", "clrL", "clrR", "comy", "fz_sw", "fz_st")}, log, t0


def summarize_lift(name, tr, log, t0):
    """per B stage (l10, l30): robot polled peak / settled, sim polled peak / continuous peak /
    settled, sim clearance (max over the stage and its hold, and the fraction of the hold > 5 mm)."""
    st, _ = stand_of(log, STANCE[name])
    t = np.array(tr["t"])
    err = np.array(tr["err"]) * R2D
    u = np.array(tr["u"]) * R2D
    stg = b_stages(st)
    swing = "L" if STANCE[name] == "R" else "R"
    clr = np.array(tr["clr" + swing]) * 1e3
    clr0 = float(np.median(clr[(t > 0.5) & (t < st.moves[1].t0)])) if st.moves[1].t0 > 1.0 else float(clr[:20].mean())
    out = {}
    for label, (i, mv) in zip(("l10", "l30"), stg):
        t_end = st.moves[i + 1].t0 if i + 1 < len(st.moves) else t[-1]
        w = (t >= mv.t0) & (t < t_end)
        sgn = -1.0 if STANCE[name] == "R" else 1.0          # toward the lifted foot: R stance -> negative err
        hw_p = [p for p in log["polls"] if p["err"] is not None and mv.t0 <= p["t"] - t0 < t_end]
        hw_err = [p["err"] for p in hw_p]
        sim_at = [float(np.interp(p["t"] - t0, t, err)) for p in hw_p]
        pick = lambda xs: (max(xs, key=lambda x: sgn * x) if xs else None)
        hold = w & (t >= mv.t0 + mv.dur)
        out[label] = dict(
            hw_polled_peak=pick(hw_err), hw_settled=hw_err[-1] if hw_err else None,
            hw_u_end=hw_p[-1]["u"] if hw_p else None,
            sim_polled_peak=pick(sim_at), sim_peak=float(pick(list(err[w]))) if w.any() else None,
            sim_settled=sim_at[-1] if sim_at else None, sim_u_end=float(u[w][-1]) if w.any() else None,
            sim_clear_max_mm=float(clr[w].max()), sim_clear_hold_med_mm=float(np.median(clr[hold])) if hold.any() else None,
            sim_clear_hold_frac5=float((clr[hold] > 5.0).mean()) if hold.any() else None,
            sim_clear_base_mm=clr0, sim_aborted=bool((np.array(tr["state"])[w] == 2).any()),
            sim_fell=bool((np.abs(np.array(tr["roll"])[w]) * R2D > 25.0).any()),
            hw_clear=HW_NOTES[name].get(label + "_clear"), hw_notes_peak=HW_NOTES[name].get(label + "_peak"))
    return out


def summarize_lat(name, tr, log, t0):
    """the unloading-tilt curve: per shift level, roll after the 5 mm lift minus roll before it,
    robot (its polls) and sim (at the same instants)."""
    t = np.array(tr["t"])
    roll = np.array(tr["roll"]) * R2D
    st, _ = stand_of(log, STANCE[name])
    polls = log["polls"]
    rows = []
    # the ladder: shift (s2000) -> lift (s1500) -> lower (s1500); the lifts are the s1500 moves
    # that follow an s2000 move
    mv = st.moves
    swing = "L" if STANCE[name] == "R" else "R"
    clr = np.array(tr["clr" + swing]) * 1e3
    lifts = [k for k in range(1, len(mv)) if abs(mv[k].dur - 1.5) < 1e-6 and abs(mv[k - 1].dur - 2.0) < 1e-6]
    for x, k in zip(LAT_X, lifts):
        tl = mv[k].t0
        before = [p for p in polls if p["t"] - t0 < tl and p["t"] - t0 > mv[k - 1].t0]
        after = [p for p in polls if tl < p["t"] - t0 < mv[k + 1].t0]
        if not before or not after:
            rows.append(dict(x=x, hw=None, sim=None))
            continue
        tb, ta = before[-1]["t"] - t0, after[0]["t"] - t0
        w = (t >= tl) & (t < mv[k + 1].t0)
        fsw, fst = np.array(tr["fz_sw"]), np.array(tr["fz_st"])
        b = (t >= tl - 0.3) & (t < tl)
        rows.append(dict(x=x, hw=after[0]["roll"] - before[-1]["roll"],
                         sim=float(np.interp(ta, t, roll) - np.interp(tb, t, roll)),
                         sim_clear_max_mm=float(clr[w].max()) if w.any() else None,
                         sim_swing_load=float(fsw[b].mean() / max(1e-9, (fsw[b] + fst[b]).mean())) if b.any() else None,
                         sim_fell=bool((np.abs(roll[w]) > 25.0).any())))
    return rows


def oneleg_fit():
    fj = json.load(open(os.path.join(GC.ROOT, "experiments", "one-leg-stand", "simfit_plant.json")))
    return dict(fj["plant"])


def clamp_test(plant, terms):
    """10-24 clamped-foot lift on the walking plant: roll_compliance's protocol and IK, the env
    built by gate_common on a copy of the plant XML with the stance foot welded."""
    import roll_compliance as RC
    out = {}
    for side in ("R", "L"):
        stance = "L" if side == "R" else "R"
        xml = RC.clamped_xml(GC.XML, stance)
        try:
            env = GC.make_plant(plant, terms=terms, xml=xml)
            env.domain_rand = False
            tilt, enc, lift = GC.clamp_run(env, side)
        finally:
            os.unlink(xml)
        out[side] = dict(tilt=tilt, enc_ankle=enc["ankle_roll"], enc_hip=enc["hip_roll"], lift_mm=lift)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--plants", default="train,train:lash0.5,train:lash2.0,grade")
    ap.add_argument("--terms", default="", help="probe terms on every plant (gate_common.parse_terms)")
    ap.add_argument("--only", default="", help="subset of floorR2,clrR,clrL,latR,latL,clamp")
    ap.add_argument("--json", default=None)
    ap.add_argument("--md", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "12")))
    a = ap.parse_args(argv)
    plants = [p for p in a.plants.split(",") if p]
    terms = GC.parse_terms(a.terms)
    names = [n for n in (a.only.split(",") if a.only else list(LOGS) + ["clamp"])]
    jobs = [(n, p, terms) for p in plants for n in names if n in LOGS]
    res = {p: {} for p in plants}
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        futs = [ex.submit(run_replay, j) for j in jobs]
        cl = ({p: ex.submit(clamp_test, p, terms) for p in plants if not p.startswith("oneleg")}
              if "clamp" in names else {})
        for f in futs:
            name, plant, tr, log, t0 = f.result()
            res[plant][name] = (summarize_lat if name.startswith("lat") else summarize_lift)(name, tr, log, t0)
        for p, f in cl.items():
            res[p]["clamp"] = f.result()
    md = table(res, plants, terms)
    print(md)
    if a.json:
        json.dump(dict(plants=plants, terms=terms, results=res, hw=HW_NOTES), open(a.json, "w"), indent=1)
    if a.md:
        open(a.md, "w").write(md + "\n")
    return res


def _f(x, nd=1):
    return GC.fmt(x, nd)


def table(res, plants, terms):
    L = [f"Check 1, one-leg protocol on the walking plant ({GC.XML_NAME})" + (f", terms {terms}" if terms else ""), ""]
    L.append("| test | stage | robot err peak (polled) / settled, deg | robot clearance | "
             + " | ".join(f"{p}: err peak polled / cont. / settled, deg; lifted-sole clearance max / hold median, mm" for p in plants) + " |")
    L.append("|---|---|---|---|" + "---|" * len(plants))
    for n in ("floorR2", "clrR", "clrL"):
        for stg in ("l10", "l30"):
            r0 = next((res[p][n][stg] for p in plants if n in res[p]), None)
            if r0 is None:
                continue
            cells = []
            for p in plants:
                r = res[p].get(n, {}).get(stg)
                if r is None:
                    cells.append("-")
                    continue
                cells.append(f"{_f(r['sim_polled_peak'], 2)} / {_f(r['sim_peak'], 2)} / {_f(r['sim_settled'], 2)}; "
                             f"{r['sim_clear_max_mm']:.1f} / {_f(r['sim_clear_hold_med_mm'])}"
                             + (" ABORT" if r["sim_aborted"] else "") + (" FELL" if r["sim_fell"] else ""))
            L.append(f"| {n} ({SHIFT_MM[n]:.1f} mm, {STANCE[n]} stance) | {stg} | {_f(r0['hw_polled_peak'], 2)} / "
                     f"{_f(r0['hw_settled'], 2)} | {r0['hw_clear']} | " + " | ".join(cells) + " |")
    L.append("")
    for n in ("latR", "latL"):
        if not any(n in res[p] for p in plants):
            continue
        L.append(f"Unloading tilt, {n} ({STANCE[n]} stance, 5 mm lift, no feedback): roll change, deg, by shift x 62 mm")
        L.append("")
        L.append("| shift x | robot | " + " | ".join(f"{p} (swing-foot share of the load before the lift)" for p in plants) + " |")
        L.append("|---|---|" + "---|" * len(plants))
        rows0 = next(res[p][n] for p in plants if n in res[p])
        for k, r0 in enumerate(rows0):
            cells = []
            for p in plants:
                r = res[p][n][k]
                cells.append(f"{_f(r['sim'], 2)} ({r['sim_swing_load']:.2f})" + (" FELL" if r.get("sim_fell") else ""))
            L.append(f"| {r0['x']:.2f} | {_f(r0['hw'], 2)} | " + " | ".join(cells) + " |")
        L.append("")
    if any("clamp" in res[p] for p in plants):
        L.append("Clamped-foot 30 mm lift, no shift (10-24): steady torso tilt toward the lifted side, deg "
                 "(stance ankle / hip roll encoder deg; foot rise mm)")
        L.append("")
        L.append("| lifted | robot | " + " | ".join(plants) + " |")
        L.append("|---|---|" + "---|" * len(plants))
        for side in ("R", "L"):
            cells = []
            for p in plants:
                c = res[p].get("clamp", {}).get(side)
                cells.append("-" if c is None else f"{c['tilt']:.2f} ({c['enc_ankle']:.2f} / {c['enc_hip']:.2f}; {c['lift_mm']:.1f})")
            L.append(f"| {side} | {HW_NOTES['clamp'][side]} (1.0-1.5 / 0.7-0.9) | " + " | ".join(cells) + " |")
    return "\n".join(L)


if __name__ == "__main__":
    main()
