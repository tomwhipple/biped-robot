"""v6 part set: export every printed part (STL + STEP), the bed check, the mass
rollup and the per-segment masses the sim plant needs.

    .venv/bin/python cad/v6/parts_v6.py              # the robot: every part + docs/design-v6/parts_v6_rollup.txt
    .venv/bin/python cad/v6/parts_v6.py --no-export  # the rollup only (builds, measures, writes nothing else)
    .venv/bin/python cad/v6/parts_v6.py --only foot_L head_shell

The defaults build THE ROBOT TO PRINT (issue #76):
    ARMS=1 (default) | 0          the shoulder girdle + both arms, and the pelvis
                                  with the girdle's deck pilots; ARMS=0 is the
                                  armless variant (neck_collar, no pilots), which
                                  is not a print target (cad/PRINT_LIST.md).
    HIP_YOKE_VARIANT=single|split the one-print hip yoke (default) or the legacy
                                  bolted yoke_roll + yoke_pitch_v6 pair.
    SERVO_PLAN=hips (default) | B | 3250
                                  hips: an STS3250 (same case, 74.5 g) at each
                                  hip roll, STS3215s everywhere else (DESIGN.md
                                  section 4); B: 17 x STS3215; 3250: STS3250s at
                                  the six roll + knee joints.

PARTS lists (name, builder, qty, print note). Every entry writes
cad/v6/stl/<name>.stl and cad/v6/step/<name>.step from the same solid in the
same run (v5 rule: the STEPs went stale once). REFERENCE solids (not printed,
used by the assembly, the checks and the sim plant) are exported the same way
but are not in the print total.
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

ROLLUP = os.path.join(HERE, "..", "..", "docs", "design-v6", "parts_v6_rollup.txt")


def _on(name, default):
    return os.environ.get(name, default) not in ("", "0", "no", "false", "off")


ARMS = _on("ARMS", "1")
HIP_YOKE = os.environ.get("HIP_YOKE_VARIANT", "single")
SERVO_PLAN = os.environ.get("SERVO_PLAN", "hips")
SERVO_PLANS = {"hips": V.SERVO_3250_JOINTS, "B": (), "3250": V.SERVO_3250_JOINTS_SIX}
if SERVO_PLAN not in SERVO_PLANS:
    raise SystemExit(f"SERVO_PLAN={SERVO_PLAN!r}: expected one of {', '.join(SERVO_PLANS)}")


def _lazy(mod, fn, *a, **k):
    def f():
        return getattr(__import__(mod), fn)(*a, **k)
    f.__name__ = f"{mod}.{fn}"
    return f


# the armless pelvis exports under its own name, so it never overwrites the
# robot's pelvis_v7.stl
PELVIS = "pelvis_v7" + ("" if ARMS else "_armless")

PARTS = [
    # name, builder, qty, note
    (PELVIS, _lazy("pelvis_v7", "pelvis_v7", arm_mounts=ARMS), 1,
     f"one print, deck-top-down; bearing-housing pilots{', girdle pilots' if ARMS else ''}"),
    ("yaw_bearing_housing", _lazy("yaw_bearing_housing", "yaw_bearing_housing"), 1,
     "one print, top face down; both 6810-2RS outer-race seats, screwed up under the pelvis"),
    ("head_shell", _lazy("head", "head_shell"), 1, "neck horn carrier + shell, base down"),
    ("head_face", _lazy("head", "head_face"), 1, "camera face plate, flat"),
    ("neck_floor", _lazy("neck_floor", "neck_floor"), 1, "the neck servo's seat, flat (#90)"),
    ("yaw_carrier_v6", _lazy("yaw_carrier_v6", "yaw_carrier_v6"), 2, "hip-yaw carrier, round hub for the 6810-2RS"),
]
if HIP_YOKE == "single":
    PARTS.append(("hip_yoke_v6", _lazy("hip_yoke_v6", "hip_yoke_v6"), 2,
                  "ONE PRINT: roll + pitch clevis fused, no flange bolts (on edge; supports + brim)"))
else:
    PARTS += [("yoke_roll", v5.yoke_roll, 2, "legacy split yoke (slicer supports)"),
              ("yoke_pitch_v6", _lazy("yoke_pitch_v6", "yoke_pitch_v6"), 2, "legacy split yoke (slicer supports)")]
PARTS += [
    ("leg_link_v6", _lazy("leg_link_v6", "leg_link_v6"), 4, "thigh + shin, 110 mm, open U, end walls"),
    ("ankle_link", _lazy("ankle_link", "ankle_link"), 2, "pitch grip + X fork onto the roll servo"),
    ("foot_L", _lazy("foot_v6", "foot", "L"), 1, "asymmetric sole, mirrored pair"),
    ("foot_R", _lazy("foot_v6", "foot", "R"), 1, "asymmetric sole, mirrored pair"),
    ("sole_tpu_L", _lazy("foot_v6", "sole_tpu", "L"), 1, "TPU 95A"),
    ("sole_tpu_R", _lazy("foot_v6", "sole_tpu", "R"), 1, "TPU 95A"),
]
if ARMS:
    PARTS += [
        ("shoulder_girdle_v6", _lazy("shoulder_girdle_v6", "shoulder_girdle_v6"), 1,
         "ONE PRINT: both shoulder pods + the neck tube + the trapezius webs (base down)"),
        ("arm_upper_v6_L", _lazy("arm_v6", "arm_upper_v6", "L"), 1, "shoulder horn -> elbow fork, 160 mm (on its back)"),
        ("arm_upper_v6_R", _lazy("arm_v6", "arm_upper_v6", "R"), 1, "mirror of _L"),
        ("arm_fore_v6_L", _lazy("arm_v6", "arm_fore_v6", "L"), 1, "elbow grip -> hand knuckle, 160 mm (on its back)"),
        ("arm_fore_v6_R", _lazy("arm_v6", "arm_fore_v6", "R"), 1, "mirror of _L"),
    ]
else:
    PARTS.append(("neck_collar", _lazy("neck_collar", "neck_collar"), 1,
                  "ARMS=0 only; NOT a print target (its flange pilots miss the deck)"))

# not printed: solids the assembly, the checks and the sim plant read
REFERENCE = [
    ("head", _lazy("head", "head"), "head_shell + head_face fused (mass, assembly, plant)"),
]

# servos: 12 legs + the neck (+ 4 in the arms); the STS3250s are leg joints, one per leg each
N_SERVOS = 12 + 1 + (V.ARM_SERVO_COUNT if ARMS else 0)
N_3250 = 2 * len(SERVO_PLANS[SERVO_PLAN])
SERVO_COUNT = {"STS3215": N_SERVOS - N_3250, "STS3250": N_3250}


def mass_g(solid, tpu=False):
    rho = 1.21e-3 if tpu else D.FILAMENT_RHO
    return solid.volume * rho * D.PRINT_MASS_FACTOR


BEARING_MASS_G = 2 * V.YAW_BRG_MASS_G   # two hip-yaw bearings


def servo_mass_g():
    return SERVO_COUNT["STS3215"] * V.SERVO_MASS_3215 + SERVO_COUNT["STS3250"] * V.SERVO_MASS_3250


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--no-export", action="store_true", help="build and measure; write the rollup only")
    ap.add_argument("--rollup", default=None,
                    help="where to write the rollup (default: docs/design-v6/parts_v6_rollup.txt, "
                         "written only for the default build)")
    a = ap.parse_args(argv)
    export = not a.no_export
    os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
    os.makedirs(os.path.join(HERE, "step"), exist_ok=True)
    total_print = 0.0
    rows = []
    for name, fn, qty, note in PARTS + [(n, f, 0, note) for n, f, note in REFERENCE]:
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
        if export:
            export_stl(s, os.path.join(HERE, "stl", f"{name}.stl"))
            export_step(s, os.path.join(HERE, "step", f"{name}.step"))
        tag = ("BED-OK" if bed_ok else "** TOO BIG **") if qty else "reference, not printed"
        rows.append((name, qty, m, dims, tag + "  " + note))
    out = []
    out.append(f"build: ARMS={int(ARMS)}  HIP_YOKE_VARIANT={HIP_YOKE}  SERVO_PLAN={SERVO_PLAN}")
    out.append(f"{'part':20s} {'qty':>3s} {'g each':>7s} {'bbox (mm)':>22s}  note")
    for name, qty, m, dims, note in rows:
        if m is None:
            out.append(f"{name:20s} {qty:3d} {'-':>7s} {'-':>22s}  {note}")
        else:
            out.append(f"{name:20s} {qty:3d} {m:7.1f} {dims[0]:6.1f}x{dims[1]:6.1f}x{dims[2]:6.1f}  {note}")
    elec = V.BATT_MASS + V.PI4_MASS + V.GD_MASS + V.PWR_MASS + V.CAM3_MASS + V.WIRING_MASS
    servos = servo_mass_g()
    counts = ", ".join(f"{n} x {k}" for k, n in SERVO_COUNT.items() if n)
    out.append("")
    out.append(f"printed total {total_print:.0f} g; servos {servos:.0f} g ({counts}); "
               f"hip-yaw bearings {BEARING_MASS_G:.0f} g (2 x 6810-2RS); "
               f"pack + boards + wiring {elec:.0f} g; robot ~{total_print + servos + BEARING_MASS_G + elec:.0f} g"
               + ("" if not a.only else "  (partial: --only)"))
    out.append("masses: build123d volume x PETG 1.27 g/cm3 x 0.90 print factor (TPU 1.21 g/cm3), "
               "before supports and brims; bought parts at catalogue mass")
    text = "\n".join(out)
    print(text)
    default_build = ARMS and HIP_YOKE == "single" and SERVO_PLAN == "hips"
    path = a.rollup or (ROLLUP if default_build else None)
    if path and not a.only:
        with open(path, "w") as f:
            f.write(text + "\n")
        print("wrote", os.path.relpath(path))


if __name__ == "__main__":
    main()
