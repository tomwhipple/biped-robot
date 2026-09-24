"""v6 part set: export every printed part (STL + STEP), the bed check, the mass
rollup and the per-segment masses the sim plant needs.

    .venv/bin/python cad/v6/parts_v6.py            # everything
    .venv/bin/python cad/v6/parts_v6.py --only foot_L head

PARTS lists (name, builder, qty, print note). The unchanged hip parts come
from v5 (cad/parts.py); the new ones from their modules in this directory.
Every entry writes cad/v6/stl/<name>.stl and cad/v6/step/<name>.step from the
same solid in the same run (v5 rule: the STEPs went stale once).
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5  # noqa: E402
D = V.D


def _lazy(mod, fn, *a):
    def f():
        return getattr(__import__(mod), fn)(*a)
    f.__name__ = f"{mod}.{fn}"
    return f


PARTS = [
    # name, builder, qty, note
    ("pelvis_v7", _lazy("pelvis_v7", "pelvis_v7"), 1, "one print, deck-top-down"),
    ("head", _lazy("head", "head"), 1, "stereo-periscope head: shell + lid + camera sled, fused for the rollup (prints as 3: head.py)"),
    ("neck_collar", _lazy("neck_collar", "neck_collar"), 1, "collar round the neck servo, flange-down"),
    ("yaw_carrier_v6", _lazy("yaw_carrier_v6", "yaw_carrier_v6"), 2, "v5 carrier + hip-yaw bearing boss (study-yaw-bearing.md)"),
    ("yoke_roll", v5.yoke_roll, 2, "v5 part, unchanged (slicer supports)"),
    ("yoke_pitch_v6", _lazy("yoke_pitch_v6", "yoke_pitch_v6"), 2, "v5 clevis + flange chamfer for hip flexion 125 (slicer supports)"),
    ("leg_link_v6", _lazy("leg_link_v6", "leg_link_v6"), 4, "thigh + shin, 110 mm, box section"),
    ("ankle_link", _lazy("ankle_link", "ankle_link"), 2, "pitch grip + X fork onto the roll servo"),
    ("foot_L", _lazy("foot_v6", "foot", "L"), 1, "asymmetric sole, mirrored pair"),
    ("foot_R", _lazy("foot_v6", "foot", "R"), 1, "asymmetric sole, mirrored pair"),
    ("sole_tpu_L", _lazy("foot_v6", "sole_tpu", "L"), 1, "TPU 95A"),
    ("sole_tpu_R", _lazy("foot_v6", "sole_tpu", "R"), 1, "TPU 95A"),
]

# The hip yoke is ONE print, hip_yoke_v6 (default since 2026-09-24, the
# assembly_v6.hip_yoke_variant switch); HIP_YOKE_VARIANT=split rolls up the
# legacy bolted pair instead.
if os.environ.get("HIP_YOKE_VARIANT", "single") == "single":
    _i = next(i for i, r in enumerate(PARTS) if r[0] == "yoke_roll")
    PARTS = [r for r in PARTS if r[0] not in ("yoke_roll", "yoke_pitch_v6")]
    PARTS.insert(_i, ("hip_yoke_v6", _lazy("hip_yoke_v6", "hip_yoke_v6"), 2,
                      "ONE PRINT: roll + pitch clevis fused, no flange bolts (on edge like yoke_roll; supports + brim)"))

# ARMS=1 (the assembly_v6.arms_on switch): the two get-up arms. Round 5 (the
# shoulder girdle, 2026-09-19) made this 3 designs / 5 prints instead of 3/6:
# the two per-side shoulder cradles became ONE symmetric girdle that also
# absorbs the neck collar. The arm LINKS are still a MIRROR PAIR, so each is
# two part numbers. Default (armless) rollup unchanged.
if os.environ.get("ARMS", "0") not in ("", "0", "no", "false", "off"):
    PARTS = [r for r in PARTS if r[0] != "neck_collar"]   # absorbed by the girdle
    for _n, _b, _q, _note in [
            ("shoulder_girdle_v6", _lazy("shoulder_girdle_v6", "shoulder_girdle_v6"), 1,
             "ONE PRINT: both shoulder pods + the neck tube + the trapezius webs (base down)"),
            ("arm_upper_v6_L", _lazy("arm_v6", "arm_upper_v6", "L"), 1, "shoulder horn -> elbow fork, 160 mm (on its back)"),
            ("arm_upper_v6_R", _lazy("arm_v6", "arm_upper_v6", "R"), 1, "mirror of _L"),
            ("arm_fore_v6_L", _lazy("arm_v6", "arm_fore_v6", "L"), 1, "elbow grip -> hand knuckle, 160 mm (on its back)"),
            ("arm_fore_v6_R", _lazy("arm_v6", "arm_fore_v6", "R"), 1, "mirror of _L"),
    ]:
        PARTS.append((_n, _b, _q, _note))

SERVO_COUNT = {"STS3250": 6, "STS3215": 7}
if os.environ.get("ARMS", "0") not in ("", "0", "no", "false", "off"):
    SERVO_COUNT = dict(SERVO_COUNT, STS3215=SERVO_COUNT["STS3215"] + V.ARM_SERVO_COUNT)


def mass_g(solid, tpu=False):
    rho = 1.21e-3 if tpu else D.FILAMENT_RHO
    return solid.volume * rho * D.PRINT_MASS_FACTOR


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args(argv)
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    total_print = 0.0
    rows = []
    for name, fn, qty, note in PARTS:
        if a.only and name not in a.only:
            continue
        try:
            s = fn()
        except Exception as e:  # noqa: BLE001
            rows.append((name, qty, None, None, f"NOT BUILT: {type(e).__name__}: {str(e)[:50]}"))
            continue
        bb = s.bounding_box()
        dims = sorted([bb.size.X, bb.size.Y, bb.size.Z], reverse=True)
        bed_ok = dims[0] <= 250 and dims[1] <= D.BED and dims[2] <= D.BED
        m = mass_g(s, tpu=name.startswith("sole_tpu"))
        total_print += m * qty
        export_stl(s, os.path.join(HERE, "stl", f"{name}.stl"))
        export_step(s, os.path.join(HERE, "step", f"{name}.step"))
        rows.append((name, qty, m, dims, ("BED-OK" if bed_ok else "** TOO BIG **") + "  " + note))
    print(f"{'part':14s} {'qty':>3s} {'g each':>7s} {'bbox (mm)':>22s}  note")
    for name, qty, m, dims, note in rows:
        if m is None:
            print(f"{name:14s} {qty:3d} {'-':>7s} {'-':>22s}  {note}")
        else:
            print(f"{name:14s} {qty:3d} {m:7.1f} {dims[0]:6.1f}x{dims[1]:6.1f}x{dims[2]:6.1f}  {note}")
    servos = SERVO_COUNT["STS3250"] * V.SERVO_MASS_3250 + SERVO_COUNT["STS3215"] * V.SERVO_MASS_3215
    elec = V.BATT_MASS + V.PI4_MASS + V.GD_MASS + V.PWR_MASS + V.CAM3_MASS + V.WIRING_MASS
    print(f"\nprinted total {total_print:.0f} g; servos {servos:.0f} g ({SERVO_COUNT}); pack + boards + wiring {elec:.0f} g; "
          f"robot ~{total_print + servos + elec:.0f} g")


if __name__ == "__main__":
    main()
