"""Loaded servo step responses (firmware `stepresp`, 2026-11-01 22:16 UTC, double support at the
stand pose, +30 ticks and back per joint, ~3 ms single-servo reads), robot vs sim.

Per joint and direction: dead time (first 7 % of the step), t63, t90 and the final fraction.
The robot: STS3215 joints dead 66-87 ms, t63 117-159 ms; STS3250 hip rolls dead 12-18 ms, t63
27-39 ms (one stalled at 63 %). The sim replays the same steps on the crouch pose of the sweep
session (simfit_sweep.load) and reads the joint (link + roll play) at the 2 ms physics rate.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_step.py [--set k=v ...]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402
import simfit_sweep as W  # noqa: E402

RAW = os.environ.get("STEPRESP_RAW", "/home/claw/.claude/jobs/df084c64/tmp/step_loaded.txt")
IDS = {52: "R_hip_roll", 6: "R_hip_pitch", 7: "R_knee", 8: "R_ankle", 51: "L_hip_roll", 2: "L_hip_pitch",
       3: "L_knee", 4: "L_ankle", 10: "R_hip_yaw", 9: "L_hip_yaw", 11: "R_ankle_roll", 12: "L_ankle_roll"}
STEP, HOLD = 30, 0.3


def metrics(t, p, target):
    fr = p / target
    def first(th):
        k = np.where(fr >= th)[0]
        return 1e3 * t[k[0]] if len(k) else float("nan")
    return first(0.07), first(0.63), first(0.9), float(np.mean(fr[-5:]))


def hw_steps():
    """{joint: [(dead, t63, t90, final) out, (...) back]} in ms."""
    cur, data = None, {}
    for line in open(RAW):
        m = re.search(r"stepresp id (\d+): start (\d+), step ([+-]\d+) ticks", line)
        if m:
            cur = IDS[int(m.group(1))]
            data[cur] = []
            continue
        m = re.search(r"SR (\d) (\d+) (-?\d+) (-?\d+) (-?\d+)", line)
        if m and cur:
            data[cur].append([int(x) for x in m.groups()])
    out = {}
    for j, rows in data.items():
        a = np.array(rows)
        res = []
        for ph in (0, 1):
            r = a[a[:, 0] == ph]
            t = (r[:, 1] - r[0, 1]) / 1e6
            p = r[:, 2].astype(float) - r[0, 2]
            res.append(metrics(t, p, STEP if ph == 0 else -float(r[0, 2])))
        out[j] = res
    return out


def _sim_one(args):
    pl, j = args
    st, _ = W.load()
    k = H.BUS.index(j)
    t_on = 4.0
    ext = np.zeros(12)
    def fn(t):
        ext[:] = 0.0
        if t_on <= t < t_on + HOLD:
            ext[k] = STEP
        return ext
    rp = S.Replay(st, pl.on("hard"), S.Ctl(), extra=fn, t_end=t_on + 2 * HOLD + 0.2)
    r = rp.run()
    tt = r["t"]
    enc = r["enc"][:, k]
    goal = r["goal"][:, k]
    base = np.median(enc[(tt > t_on - 0.3) & (tt < t_on)])
    w1 = (tt >= t_on) & (tt < t_on + HOLD)
    w2 = (tt >= t_on + HOLD) & (tt < t_on + 2 * HOLD)
    d = st.cal[j][1]                      # +30 ticks is +30 on the bus; enc/goal are bus ticks
    m1 = metrics(tt[w1] - t_on, enc[w1] - base, STEP)
    end1 = enc[w1][-1] - base
    m2 = metrics(tt[w2] - t_on - HOLD, enc[w2] - base - end1, -end1 if end1 else -STEP)
    return j, [m1, m2]


STEP_JOINTS = ("R_hip_roll", "L_hip_roll", "R_ankle_roll", "L_ankle_roll", "R_hip_pitch", "L_hip_pitch",
               "R_knee", "L_knee", "R_ankle", "L_ankle")


def sim_steps(pl, joints=None):
    joints = joints or list(STEP_JOINTS)
    with ProcessPoolExecutor(min(12, len(joints))) as ex:
        return dict(ex.map(_sim_one, [(pl, j) for j in joints]))


def step_cost(sim, hw):
    """mean over joints/directions of |dead err|/20 ms + |t63 err|/30 ms + |final err|/0.15."""
    c, n = 0.0, 0
    for j, hv in hw.items():
        if j not in sim:
            continue
        for (d, t63, t90, fin), (sd, st63, st90, sfin) in zip(hv, sim[j]):
            if not np.isfinite(sd):
                sd = 300.0
            if not np.isfinite(st63):
                st63 = 300.0
            c += abs(sd - d) / 20.0 + (abs(st63 - t63) / 30.0 if np.isfinite(t63) else 0.0) + abs(sfin - fin) / 0.15
            n += 1
    return c / max(n, 1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--fitted", default=os.path.join(HERE, "simfit_plant.json"))
    a = ap.parse_args()
    d = json.load(open(a.fitted))
    pl = replace(S.Plant(**d["plant"]), **{k: float(v) for k, v in (x.split("=") for x in a.set)})
    hw = hw_steps()
    sim = sim_steps(pl)
    print(f"{'joint':13s}  robot out | back (dead t63 final)        sim out | back")
    for j in hw:
        f = lambda m: f"{m[0]:4.0f} {m[1]:4.0f} {m[3]:4.2f}"
        print(f"{j:13s}  {f(hw[j][0])} | {f(hw[j][1])}     {f(sim[j][0])} | {f(sim[j][1])}")
    print(f"step cost {step_cost(sim, hw):.2f}")
