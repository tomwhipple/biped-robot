"""The inside tipping-point test (hw 2026-11-01 20:53, hard floor; tip.sh), replayed in the sim.

Per side: the proven stand (shift, 10 mm lift with B, lower, 30 mm lift with B + ~4 s hold), then
FREEZE (the l30 pose with the live correction ticks baked in, balance off, s800), then lean toward
the raised foot: the stance hip roll and ankle roll both -/+6 ticks per step (L stance -, R stance +;
the PI's ankle +u / hip -u pelvis translation in the joint frame), s500, ~1.33 s per step, until the
raised foot lands (roll jumps). The robot:

    L stance: frozen +48/+48 ticks, roll +2.02 deg; tipped at step 17 (-102 ticks), roll 6.58 -> 10.28
    R stance: frozen -46/-46 ticks, roll -3.92 deg; tipped at step 12 (+72 ticks), roll -6.87 -> -14.77

The replay runs the recorded command sequence (targets, durations, the robot's start stamps) tick
for tick like simfit_sim, and reads the roll where tip.sh read it (~1.2 s after each step command).
`tip_cost` scores the tip step and the roll-per-step curve before it:

    |tip offset sim - hw| / 12 ticks  +  rms(roll_sim - roll_hw over the pre-tip steps, each relative
    to its frozen roll) / 0.5 deg

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_tip.py [--set k=v ...]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402

R2D = 180 / math.pi
TIP_TAG = "205338"
H.SESSIONS.setdefault(TIP_TAG, (os.path.join(H.HW_ROOT, "2026-11-01"), "flight_20261101T205338Z_ilim_stands", "hard", H._ILIM3))
STEP_TICKS = 6
# the robot's result (tip_run.txt + tip2_run.txt, 3 runs per side): the lean offset (ticks beyond the
# frozen correction) at which the raised foot landed. L: -102 / -102 / -102; R: +72 / +66 / +72.
HW_TIP = {"L": dict(step=17, offset=102.0, roll0=2.02, roll_pre=6.58, roll_post=10.28),
          "R": dict(step=12, offset=70.0, roll0=-3.92, roll_pre=-6.87, roll_post=-14.77)}


def load_tip(side):
    """Stand-like object for the tip sequence of `side`: moves shift, l10, lower10, l30, freeze,
    step1..N, guard, crouch (times from the shift's streaming stamp)."""
    e = H._events(TIP_TAG)
    d = H._ticks(TIP_TAG)
    cmds = e[e.kind == "cmd"].reset_index(drop=True)
    outs = e[e.kind == "out"]
    stream = outs[outs.text.astype(str).str.contains("SMOOTH minimum-jerk")].t_utc.values
    poses = []
    for _, r in cmds.iterrows():
        t = str(r.text)
        if not t.startswith("pose "):
            continue
        tok = t.split()
        ts = stream[stream >= r.t_utc]
        poses.append(dict(t0=float(ts[0]) if len(ts) and ts[0] - r.t_utc < 0.5 else float(r.t_utc),
                          dur=int(next(x for x in tok if x.startswith("s"))[1:]) / 1000.0,
                          ticks=[int(x) for x in tok[1:13]], B=next((x[1] for x in tok if x.startswith("B")), None)))
    il10 = next(i for i, p in enumerate(poses) if p["B"] == side and abs(p["dur"] - 6.0) < 1e-6)
    seq = poses[il10 - 1:]
    names = ["shift", "l10", "lower10", "l30", "freeze"]
    moves = []
    t_utc0 = seq[0]["t0"]
    k = 0
    for i, p in enumerate(seq):
        if i < 5:
            nm = names[i]
        elif abs(p["dur"] - 0.5) < 1e-6:
            k += 1
            nm = f"step{k}"
        elif abs(p["dur"] - 1.5) < 1e-6:
            nm = "guard"
        else:
            nm = "crouch"
        moves.append(H.Move(nm, p["t0"] - t_utc0, p["dur"], p["ticks"], p["B"]))
        if nm == "crouch":
            break
    w = d[(d.t_utc >= t_utc0 - 1.0) & (d.t_utc <= t_utc0 + moves[-1].t0 + 4.0)].copy()
    w["t"] = w.t_utc - t_utc0
    st = H.Stand(TIP_TAG, side, t_utc0, moves, w.reset_index(drop=True), None, None, {}, H._cal_from(d),
                 rep=1, floor="hard", fw=dict(H._ILIM3))
    return st


def _roll_at(t, roll, tq):
    return float(np.interp(tq, t, roll))


def step_rolls(moves, t, roll, sample=0.85):
    """roll (deg) at the freeze end and ~1.2 s after each step command."""
    fr = next(m for m in moves if m.name == "freeze")
    steps = [m for m in moves if m.name.startswith("step")]
    r0 = _roll_at(t, roll, steps[0].t0 - 0.05) * R2D if steps else float("nan")
    return r0, np.array([_roll_at(t, roll, m.t0 + sample) * R2D for m in steps])


def tip_step(r0, rolls, sign):
    """first step whose roll jumps > 2.5 deg from the previous, or drifts > 6 deg from the frozen
    roll (tip.sh's rule), toward the raised foot (sign +1 for L stance: roll +)."""
    prev = r0
    for k, r in enumerate(rolls):
        if abs(r - prev) > 2.5 or abs(r - r0) > 6.0:
            return k + 1
        prev = r
    return None


def run_tip(pl, side, ctl=None, seed=0):
    st = load_tip(side)
    ctl = ctl or S.Ctl(**st.fw)
    r = S.Replay(st, pl.on("hard"), ctl, seed=seed).run()
    sg = 1.0 if side == "L" else -1.0
    r0, rolls = step_rolls(st.moves, r["t"], r["roll"])
    k = tip_step(r0, rolls, sg)
    h0, hrolls = step_rolls(st.moves, st.ticks.t.values, st.ticks.roll.values)
    return dict(side=side, sim_r0=r0, sim_rolls=rolls, sim_step=k, hw_r0=h0, hw_rolls=hrolls,
                hw_step=HW_TIP[side]["step"], r=r, st=st)


def tip_cost(o):
    n_hw = o["hw_step"]
    n_sim = o["sim_step"] if o["sim_step"] is not None else len(o["sim_rolls"]) + 3
    c = abs(n_sim * STEP_TICKS - HW_TIP[o["side"]]["offset"]) / 12.0
    m = min(n_hw, n_sim) - 1
    if m > 0:
        ds = (o["sim_rolls"][:m] - o["sim_r0"]) - (o["hw_rolls"][:m] - o["hw_r0"])
        c += float(np.sqrt(np.mean(ds ** 2))) / 0.5
    return c


def _job(args):
    pl, side = args
    o = run_tip(pl, side)
    o.pop("r"); o.pop("st")
    o["cost"] = tip_cost(o)
    return o


def evaluate(pl, sides=("L", "R")):
    with ProcessPoolExecutor(max_workers=2) as ex:
        return list(ex.map(_job, [(pl, s) for s in sides]))


def report(res):
    for o in res:
        print(f"  {o['side']}: tip step sim {o['sim_step']} hw {o['hw_step']}  frozen roll sim {o['sim_r0']:+.2f} hw {o['hw_r0']:+.2f}"
              f"  cost {o['cost']:.2f}")
        print("     d roll sim " + " ".join(f"{x - o['sim_r0']:+.1f}" for x in o["sim_rolls"]))
        print("     d roll hw  " + " ".join(f"{x - o['hw_r0']:+.1f}" for x in o["hw_rolls"]))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    a = ap.parse_args()
    d = json.load(open(a.fitted))
    pl = S.Plant(**d["plant"])
    pl = replace(pl, **{k: float(v) for k, v in (x.split("=") for x in a.set)})
    report(evaluate(pl))
