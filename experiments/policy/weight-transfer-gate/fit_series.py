"""Round 2, item 3: fit a spring in series with the roll servos (output side, beyond the encoders) to
the 2026-10-24 clamped-foot signature ONLY: torso tilt toward the lifted side 5.2 deg (R lifted) /
5.0 deg (L lifted) with 1.0-1.5 deg at the stance ankle-roll encoder and 0.7-0.9 deg at the stance
hip-roll encoder (docs/design-v6/roll-compliance-2026-10-24.md).

    python fit_series.py [--md out.md] [--json out.json]

The plant is `train` (gate_common) with the roll play set to ZERO, so the spring carries everything
beyond the encoders. Three placements: hip rolls only (ser_hip), ankle rolls only (ser_ankle), both
at the same stiffness. A stiffness grid, then linear interpolation in 1/k for the tilt 5.1 deg
(the bench's mean), then that k re-run and reported with its encoders."""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

TARGET_TILT = 5.1
GRID = (3.0, 5.0, 8.0, 12.0, 20.0, 40.0, 60.0)
PLACES = {"hip": ("ser_hip",), "ankle": ("ser_ankle",), "both": ("ser_hip", "ser_ankle")}


def clamp(job):
    place, k, base = job
    import roll_compliance as RC
    terms = dict(base)
    for key in PLACES[place]:
        terms[key] = k
    out = {}
    for side in ("R", "L"):
        stance = "L" if side == "R" else "R"
        xml = RC.clamped_xml(GC.XML, stance)
        try:
            env = GC.make_plant("train", terms=terms, xml=xml)
            env.domain_rand = False
            tilt, enc, lift = GC.clamp_run(env, side)
        finally:
            os.unlink(xml)
        out[side] = dict(tilt=tilt, enc_ankle=enc["ankle_roll"], enc_hip=enc["hip_roll"], lift_mm=lift)
    return place, k, out


def fit(workers=12, base=None):
    base = dict(lash=0.0) if base is None else base
    jobs = [(p, k, base) for p in PLACES for k in GRID]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        grid = list(ex.map(clamp, jobs))
    best = {}
    for p in PLACES:
        rows = sorted([(k, o) for pp, k, o in grid if pp == p])
        ks = np.array([k for k, _ in rows])
        tilt = np.array([0.5 * (o["R"]["tilt"] + o["L"]["tilt"]) for _, o in rows])
        # tilt falls as k rises; interpolate in 1/k
        order = np.argsort(1.0 / ks)
        inv = np.interp(TARGET_TILT, tilt[order], (1.0 / ks)[order])
        best[p] = float(1.0 / inv) if tilt.min() <= TARGET_TILT <= tilt.max() else None
    with ProcessPoolExecutor(max_workers=workers) as ex:
        fitted = list(ex.map(clamp, [(p, k, base) for p, k in best.items() if k]))
    return grid, best, fitted


def table(grid, best, fitted):
    L = ["Clamped-foot 30 mm lift on `train` with zero roll play and a series spring (deg; stance encoders "
         "ankle / hip deg). Robot: 5.2 (R lifted) / 5.0 (L lifted), encoders 1.0-1.5 / 0.7-0.9.", "",
         "| placement | k N m/rad | R lifted tilt (enc) | L lifted tilt (enc) |", "|---|---|---|---|"]
    for p, k, o in grid + [(p, k, dict(o, fitted=True)) for p, k, o in fitted]:
        tag = " **fit**" if o.get("fitted") else ""
        L.append(f"| {p}{tag} | {k:.1f} | {o['R']['tilt']:.2f} ({o['R']['enc_ankle']:.2f} / {o['R']['enc_hip']:.2f}) | "
                 f"{o['L']['tilt']:.2f} ({o['L']['enc_ankle']:.2f} / {o['L']['enc_hip']:.2f}) |")
    L.append("")
    L.append("Fitted (tilt 5.1 deg): " + ", ".join(f"{p} k = {k:.1f} N m/rad" if k else f"{p}: out of the grid"
                                                for p, k in best.items()))
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "12")))
    a = ap.parse_args(argv)
    grid, best, fitted = fit(a.workers)
    md = table(grid, best, fitted)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump(dict(best=best, grid=grid, fitted=fitted), open(a.json, "w"), indent=1)
    return best


if __name__ == "__main__":
    main()
