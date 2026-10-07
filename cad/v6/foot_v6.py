"""foot_L / foot_R (mirrored pair) -- cad/v6/foot_v6.py

Sole plate + ankle-ROLL servo cradle. Local frame: the roll axis' vertical
projection at the origin, +X toe, z = 0 at the PLATE BOTTOM (the silicone sole
hangs TPU_SOLE_T below that). For foot_L, outboard is +Y; foot_R is built as
foot_L then mirrored about the X-Z plane (mirror(..., about=Plane.XZ)), which
also flips the roll servo's long (cable) side to outboard on the right.

The roll servo lies FLAT on the plate top, output axis along +X (horn
forward, toe side), case length along Y (from -SV_AXIS_FROM_OUT_END inboard
to +SV_AXIS_FROM_REAR outboard/cable end), case width (24.72) VERTICAL --
axis at V.ANKLE_ROLL_ABOVE_PLATE (16.36) above the plate top. The ankle
link's tines bolt onto the horn disc (x = SV_HORN_FACE, front) and the idler
disc (x = SV_IDLER_FACE, rear) from above; the foot only has to hold the case
down and located, not carry the joint load (that is the tine pads' job).

    .venv/bin/python cad/v6/foot_v6.py
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import *  # noqa: E402
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402
import check_assembly as CA  # noqa: E402

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")

# ---------------------------------------------------------------------------
# case footprint in the foot's own (pre-mirror, "L") frame
CASE_Y = (-D.SV_AXIS_FROM_OUT_END, D.SV_AXIS_FROM_REAR)     # -10.11..35.11
CASE_X = (D.SV_IDLER_CASE_FACE, D.SV_TOPFACE)               # -14.75..17.35 (real case material)
CASE_Z0 = V.FOOT_PLATE_T                                    # 4.0, sits on the plate top
CASE_Z1 = CASE_Z0 + D.SV_WID                                # 28.72, case top
AXIS_Z = V.ANKLE_ROLL_ABOVE_PLATE                           # 16.36


def mock_roll_servo(side):
    """The ankle-roll servo, seated in THIS foot's own (pre-mirror) frame:
    output axis +X, case length along Y with the cable end OUTBOARD. Built
    the same way assembly_v6.mock_roll_servo_in_foot does (rolled off
    check_assembly.servo_mock_x()), duplicated here rather than imported to
    avoid a circular import (assembly_v6 imports this module)."""
    sgn = 1 if side == "L" else -1
    m = CA.servo_mock_x()
    bb = m.bounding_box()
    long_down = abs(bb.min.Z) > abs(bb.max.Z)
    ang = 90 if (long_down == (sgn > 0)) else -90
    out = Pos(0, 0, AXIS_Z) * Rot(ang, 0, 0) * m
    bb2 = out.bounding_box()
    assert (bb2.max.Y if sgn > 0 else -bb2.min.Y) > 30.0, \
        "roll servo mock: long side not outboard"
    return out


def _extrude_outline(t, r=V.FOOT_CORNER_R):
    """Rounded-rectangle sole outline (centreline V.FOOT_Y_OFF outboard),
    extruded to thickness t."""
    x0, x1 = -V.FOOT_HEEL, V.FOOT_TOE
    y0, y1 = -V.FOOT_IN, V.FOOT_OUT          # FOOT_IN/OUT are measured from the roll axis
    p = parts.box(x0 + r, x1 - r, y0, y1, 0, t)
    p += parts.box(x1 - r, x1, y0 + r, y1 - r, 0, t)
    p += parts.box(x0, x0 + r, y0 + r, y1 - r, 0, t)
    for cx, cy in ((x1 - r, y0 + r), (x1 - r, y1 - r),
                  (x0 + r, y0 + r), (x0 + r, y1 - r)):
        p += parts.cyl_z(r, 0, t, cx, cy)
    return p


def sole_tpu(side):
    """The sole's outline: 2 mm silicone rubber sheet is cut to it (not
    printed). TPU_SOLE_T thick, hanging BELOW the plate bottom (z 0 down to
    -TPU_SOLE_T), 1 mm chamfer on the bottom (ground-facing) edge --
    best-effort; an OCC chamfer that fails just leaves that edge square
    rather than aborting the build."""
    t = V.TPU_SOLE_T
    solid = Pos(0, 0, -t) * _extrude_outline(t)
    try:
        bot_edges = [e for e in solid.edges()
                    if abs(e.vertices()[0].Z + t) < 1e-6
                    and abs(e.vertices()[1].Z + t) < 1e-6]
        solid = chamfer(bot_edges, 1.0)
    except Exception:  # noqa: BLE001
        print("  [sole_tpu] chamfer skipped (OCC could not fillet the full loop)")
    if side == "R":
        solid = mirror(solid, about=Plane.XZ)
    return solid


def _vertical_insertion_clearance(top=40.0, step=2.0):
    """The roll servo's insertion path -- straight down (-z) from `top` mm
    above its seated pose -- as a solid to subtract from the cradle. Built
    ALWAYS in the foot_L convention (side='L'); foot() mirrors the whole
    finished part for 'R', which correctly flips this cut along with
    everything else.

    Doing it this way (rather than hand-deriving where the horn rib / idler
    platform land in this rotated frame) is deliberate: those SAME features
    already forced a relief on leg_link's grip plates and v5's foot tabs, and
    re-deriving their footprint by hand in a THIRD axis convention is exactly
    how the a4ce69e-class bugs happened. The exact mock is authoritative; a
    swept subtraction from it can't miss a feature the hand math would."""
    mock = mock_roll_servo("L")
    cut = None
    dz = 0.0
    while dz <= top + 1e-6:
        m = Pos(0, 0, dz) * mock
        cut = m if cut is None else cut + m
        dz += step
    return cut


def foot(side):
    p = _extrude_outline(V.FOOT_PLATE_T)

    # --- roll-servo cradle: two low rails flanking the case (fore/aft, i.e.
    # X either side of the case), FIT clear, FOOT_CRADLE_H tall.
    rail_h = V.FOOT_CRADLE_H
    railY0, railY1 = CASE_Y[0] - D.FIT, CASE_Y[1] + 2.0   # run out to the open (cable) end
    # front (horn-side) rail
    p += parts.box(CASE_X[1] + D.FIT, CASE_X[1] + D.FIT + D.WALL,
                   railY0, railY1, V.FOOT_PLATE_T, V.FOOT_PLATE_T + rail_h)
    # rear (idler-side) rail, WITH a notch for the connector lead (SV_CONN_L,
    # on the idler-side face -- it exits sideways through this rail)
    rail_x0, rail_x1 = CASE_X[0] - D.FIT - D.WALL, CASE_X[0] - D.FIT
    p += parts.box(rail_x0, rail_x1, railY0, D.SV_CONN_L[0] - 1.0,
                   V.FOOT_PLATE_T, V.FOOT_PLATE_T + rail_h)
    p += parts.box(rail_x0, rail_x1, D.SV_CONN_L[1] + 1.0, railY1,
                   V.FOOT_PLATE_T, V.FOOT_PLATE_T + rail_h)
    # NOTCH both rails clear of the ankle-link tine pads' O20 boss (centred
    # on the roll axis, y=0, dipping to z=6.36 -- ANKLE_ROLL_ABOVE_PLATE -
    # PAD_D/2): the discs bolt straight to the case there and a rail run
    # through that band collides with the boss (2026-09-14 finding).
    pad_clear_hy = D.PAD_D / 2 + 1.0
    p -= parts.box(-100, 100, -pad_clear_hy, pad_clear_hy,
                   V.FOOT_PLATE_T, V.FOOT_PLATE_T + rail_h)

    # --- end stop at the inboard (output) end
    stop_y = CASE_Y[0] - D.FIT
    p += parts.box(rail_x0, CASE_X[1] + D.FIT + D.WALL, stop_y - D.WALL, stop_y,
                   V.FOOT_PLATE_T, V.FOOT_PLATE_T + rail_h)

    # --- retention tabs, gusseted to the plate. Grown 0.8 mm PAST
    # V.FOOT_TAB_FRONT_X/REAR_X on the OUTER face only (inner/seat face
    # unchanged) -- the horn rib / idler platform relief below
    # (_vertical_insertion_clearance) eats most of V.FOOT_TAB_T at the seat,
    # and the nominal 2.4 mm leaves < 0.85 mm of wall past it. Extra material
    # is outboard of the screw countersink, so it does not touch the fit.
    TAB_GROW = 0.8
    for x0, x1, row in ((V.FOOT_TAB_FRONT_X[0], V.FOOT_TAB_FRONT_X[1] + TAB_GROW, V.FOOT_TAB_ROW_HORN),
                        (V.FOOT_TAB_REAR_X[0] - TAB_GROW, V.FOOT_TAB_REAR_X[1], V.FOOT_TAB_ROW_IDLER)):
        y0, y1 = row - V.FOOT_TAB_HW, row + V.FOOT_TAB_HW
        p += parts.box(x0, x1, y0, y1, V.FOOT_PLATE_T, V.FOOT_PLATE_T + V.FOOT_TAB_H)
        # gusset (vertical face against the tab, sloped top -- support-free
        # printing sole-down)
        outer_x = x1 if x0 == V.FOOT_TAB_FRONT_X[0] else x0
        run = 8.0
        sgn = 1 if outer_x > 0 else -1
        p += parts.wedge_y([(outer_x, V.FOOT_PLATE_T + V.FOOT_TAB_H),
                            (outer_x - sgn * run, V.FOOT_PLATE_T),
                            (outer_x, V.FOOT_PLATE_T)], y0, y1)
        for zh in V.FOOT_SCREW_Z:
            sign = 1 if x0 == V.FOOT_TAB_FRONT_X[0] else -1
            face = x1 if sign > 0 else x0
            p -= parts.teardrop_x(D.CASE_SCREW_CLEAR / 2,
                                  x0 - 1 if sign > 0 else x0,
                                  x1 if sign > 0 else x1 + 1,
                                  row, zh, roll=0)
            p -= parts.csk_x(row, zh, face, sign)

    # --- relief so the ankle-link tine pads' O20 bosses (which dip a little
    # below the roll axis) never touch the plate top -- the design already
    # clears by 2.36 mm (dimensions_v6 note), so no cut is needed; kept as a
    # documented non-issue rather than a silent assumption.
    assert AXIS_Z - D.PAD_D / 2 > V.FOOT_PLATE_T + 1.0, \
        "tine pads would need a relief slot in the sole"

    # local reinforcement at the front tab's rib-relief boundary (2026-09-14,
    # verified by density probing at 26-48% -- genuinely thin, not a
    # ray-cast artifact): the horn-rib relief window in
    # _vertical_insertion_clearance leaves less than 0.85 mm of tab past it
    # in two spots. Added BEFORE that cut so anything still unsafe is
    # trimmed straight back off.
    fx = (V.FOOT_TAB_FRONT_X[0] + V.FOOT_TAB_FRONT_X[1] + TAB_GROW) / 2
    p += parts.cyl_z(2.0, V.FOOT_PLATE_T + 2.0, V.FOOT_PLATE_T + V.FOOT_TAB_H - 2.0,
                     fx, V.FOOT_TAB_ROW_HORN + V.FOOT_TAB_HW - 0.5)

    # --- clear the servo's real geometry (horn rib, idler platform, disc
    # boss -- all proud of the case's nominal faces) along its vertical
    # insertion path. Without this the tabs and rails, seated flush against
    # the nominal case faces, foul those protrusions (2026-09-14 finding).
    p -= _vertical_insertion_clearance()

    if side == "R":
        p = mirror(p, about=Plane.XZ)
    return p


PRINT_ORIENT = "IDENT"     # sole down, flat on the bed -- the standard v5
                            # foot orientation; every added face here (rails,
                            # tabs, gussets, end stop) is vertical or a
                            # support-free upward slope.

SERVO_INSERT = {"foot": ((0, 0, -1), 30.0)}   # servo drops in from above


def SCREWS():
    s = []
    for x0, x1, row, tag in ((V.FOOT_TAB_FRONT_X[0], V.FOOT_TAB_FRONT_X[1],
                             V.FOOT_TAB_ROW_HORN, "front"),
                            (V.FOOT_TAB_REAR_X[0], V.FOOT_TAB_REAR_X[1],
                             V.FOOT_TAB_ROW_IDLER, "rear")):
        for zh in V.FOOT_SCREW_Z:
            s.append({"name": f"tab_{tag}_{zh:.1f}", "kind": "M2.5x8 flat self-tap",
                      "pos": ((x0 + x1) / 2, row, zh), "axis": (1, 0, 0), "length": 8.0})
    return s


if __name__ == "__main__":
    import time
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    for side in ("L", "R"):
        t0 = time.time()
        p = foot(side)
        bb = p.bounding_box()
        mass = p.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        print(f"foot_{side}  bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm  "
              f"vol {p.volume/1000:.1f} cm3  mass {mass:.1f} g  ({time.time()-t0:.1f}s)")
        export_stl(p, os.path.join(OUT_STL, f"foot_{side}.stl"))
        export_step(p, os.path.join(OUT_STEP, f"foot_{side}.step"))
        tp = sole_tpu(side)
        tm = tp.volume * 1.21e-3
        print(f"sole_tpu_{side}  vol {tp.volume/1000:.1f} cm3  mass {tm:.1f} g")
        export_stl(tp, os.path.join(OUT_STL, f"sole_tpu_{side}.stl"))
        export_step(tp, os.path.join(OUT_STEP, f"sole_tpu_{side}.step"))
