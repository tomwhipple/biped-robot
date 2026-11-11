"""Round 3, item 2: the 2026-11-10 scale ladder (both feet down, foot load against lateral shift) on
candidate plants, against the robot's scale readings.

    python ladder.py [--plants grade,grade:<terms>,...] [--md out.md] [--json out.json]

The method is hw_sessions/2026-11-10/m271/part2.py's: the ladder's own pose commands
(shift_ladder.json, copied from the 10-25 lat logs), replayed through simfit_sim.Replay with no
balance loop: the stand, then crouch (its logged s3000) at 3.5 s, 3 s later the shift pose (its
logged s2000, straight from the crouch), 3 s settle. Load = the mean over the last 0.5 s of each
foot's summed normal contact force; the L share = L / (L + R). A plant is `name[:terms]` with name
grade | train and gate_common terms after the colon (`|` separates terms on the command line,
since terms use commas)."""
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

LADDER = os.path.join(GC.HW, "2026-11-10", "m271", "shift_ladder.json")
MULTS = (0.5, 0.75, 1.0, 1.1, 1.2, 1.3, 1.4)
# the robot's L-foot share (2026-11-10-robot/README.md section 2; scale g / 1621 g)
ROBOT = {"stand": 0.504, "crouch": 0.476,
         "L": [0.568, 0.632, 0.695, 0.714, 0.737, 0.771, 0.793],
         "R": [0.374, 0.326, 0.276, 0.257, 0.234, 0.213, 0.193]}


def parse_plant(spec):
    name, _, t = spec.partition(":")
    return name, GC.parse_terms(t.replace("|", ","))


def _ticks(c):
    return [int(x) for x in c.split()[:12]]


def _dur(c):
    return int(re.search(r"s(\d+)", c).group(1)) / 1e3


def one(job):
    """(plant spec, side, label, cmds) -> L share and the loads (N)."""
    spec, side, label, stand, crouch, pose = job
    import simfit_sim as SS
    import simfit_hw as H
    import check1_oneleg as C1
    name, terms = parse_plant(spec)
    env = GC.make_plant(name, terms=terms)
    zl = getattr(env, "_gate_zoff_La", -2.12 if name == "grade" else 0.0)
    pl = SS.Plant(zoff_La=zl)
    SS.CTRL_DT = env.control_dt
    env._zero_off = None
    mv = [H.Move("stand", 0.0, _dur(stand), _ticks(stand), None)]
    t = 3.5
    if label != "stand":
        mv.append(H.Move("crouchm", t, _dur(crouch), _ticks(crouch), None))
        t += _dur(crouch) + 3.0
    if label not in ("stand", "crouch"):
        mv.append(H.Move("shift", t, _dur(pose), _ticks(pose), None))
        t += _dur(pose)
    elif label == "crouch":
        t = 3.5 + _dur(crouch)
    st = types.SimpleNamespace(cal=dict(C1.CAL), side=side, moves=mv, t_utc0=0.0)
    orig = SS.make_env
    SS.make_env = lambda pl_, seed=0: env
    try:
        rp = SS.Replay(st, pl, SS.Ctl(), t_end=t + 3.0)
        alpha = getattr(env, "_gate_alpha", None)
        if alpha is not None and np.any(alpha > 0):
            enc0 = rp.encoder_rad

            def enc():
                q = env.data.qpos[env._sqpos]
                return enc0() + (alpha * (env.data.ctrl - q))[rp.jidx]
            rp.encoder_rad = enc
        r = rp.run()
    finally:
        SS.make_env = orig
    tt = np.array(r["t"])
    w = tt >= tt[-1] - 0.5
    sw = "L" if side == "R" else "R"
    F = {side: float(np.mean(np.array(r["fz_st"])[w])), sw: float(np.mean(np.array(r["fz_sw"])[w]))}
    return spec, side, label, dict(L=F["L"], R=F["R"], share=F["L"] / (F["L"] + F["R"]),
                                   mass=float(env.model.body_mass.sum()))


def run(plants, workers=24):
    J = json.load(open(LADDER))
    jobs = []
    for spec in plants:
        s = J["sides"]["L"]
        jobs.append((spec, "L", "stand", s["stand"]["cmd"], s["crouch"]["cmd"], None))
        jobs.append((spec, "L", "crouch", s["stand"]["cmd"], s["crouch"]["cmd"], None))
        for side in "LR":
            s = J["sides"][side]
            for g in s["rungs"]:
                jobs.append((spec, side, f"x{g['mult']}", s["stand"]["cmd"], s["crouch"]["cmd"], g["cmd"]))
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(one, jobs))
    out = {p: {} for p in plants}
    for spec, side, label, r in res:
        out[spec][label if label in ("stand", "crouch") else f"{side}{label}"] = r
    return out


def table(out, plants):
    L = ["L-foot share, both feet down (robot: 2026-11-10 kitchen scale under the L foot, 1621 g)", "",
         "| pose | robot | " + " | ".join(f"`{p}`" for p in plants) + " |", "|---|---|" + "---|" * len(plants),
         "| total mass, g | 1621 | " + " | ".join(f"{1e3 * out[p]['stand']['mass']:.0f}" for p in plants) + " |"]
    rows = [("stand", "stand", ROBOT["stand"]), ("crouch", "crouch", ROBOT["crouch"])]
    for side, tag in (("L", "toward L"), ("R", "toward R")):
        for k, x in enumerate(MULTS):
            rows.append((f"{tag} {62 * x:.1f} mm", f"{side}x{x}", ROBOT[side][k]))
    for lab, key, rob in rows:
        L.append(f"| {lab} | {rob:.3f} | " + " | ".join(f"{out[p][key]['share']:.3f}" for p in plants) + " |")
    err = {p: [out[p][key]["share"] - rob for _, key, rob in rows] for p in plants}
    L.append("| rms vs robot | | " + " | ".join(f"{math.sqrt(np.mean(np.square(err[p]))):.3f}" for p in plants) + " |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--plants", default="grade")
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "24")))
    a = ap.parse_args(argv)
    plants = [p for p in a.plants.split(",") if p]
    out = run(plants, a.workers)
    md = table(out, plants)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
