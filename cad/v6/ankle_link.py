"""ankle_link (qty 2, same part both legs) -- grips the ankle-PITCH servo case
the way leg_link_v6 grips its servo (the servo hangs inside the link), then
forks DOWN onto the ankle-ROLL servo, which lies across the foot with its
output axis along +X.

Local frame: pitch axis == Y axis through the origin (leg_link's convention);
roll axis == X axis at z = -AL_DROP; +X forward.

The part is a box around the base of the pitch servo, open only at the top
(the servo goes in from above), with the two tines below it:

- BACK: leg_link_v6's web, with the servo lead's window.
- SIDES: leg_link_v6's two grip plates (horn +y, idler -y; rib and platform
  detents, 6x countersunk M2.5 case-grip screws), the idler plate square at
  V.LL_IDLER_PLATE_TOP. Both run on from the web to the front wall, between
  GRIP_BOT and that height.
- FRONT: the front tine, carried up across the full width to the same height.
- FLOOR (V.AL_FLOOR_Z) under the case, from the rear tine to the front wall.
- TINES: below the floor each narrows through a 45 deg shoulder
  (V.AL_SHOULDER_Y) to V.AL_TINE_W and ends in an O20 pad on the roll axis:
  O14 BCD (4x M3 clearance) and the centre relief. The rear tine sits behind
  the web (its inner face is on the idler disc) and hangs from the floor.

Why the sides join the fork through the front wall and not straight down:
the roll servo's case, swept through the ankle-roll ROM in both mounting
orientations (the foot is mirrored, this part is not), reaches z -37.0 under
the idler plate and -35.8 under the horn plate, and the pitch servo's case
bottom is at -35.11. Past the case's front end (x >= 18.5, in front of the
horn rib) the room is there. Two clearance cuts come last:
_roll_sweep_clearance_cut() trims the sides' and the floor's lower edges
beside the case; _tab_keepout() bevels the lower outer edges past the case's
ends, where the foot's retention tabs swing up beside them (to z -33.7 at
the horn plate's outer face at full roll).

Print: RY_XUP (model +X -> print +Z), leg_link's orientation: the rear tine
on the bed; sides and floor rise as walls; the grip screws' teardrop roofs
point +X; the pad bores are vertical. Supported: the web and the sides'
rear ends, 4.6 mm over the bed (from the plate); the front tine below the
floor, a ledge over the roll servo's space. The front wall bridges between
the sides above the floor (cad/v6/audit_ankle_foot.py, cad/PRINT_LIST.md).

    .venv/bin/python cad/v6/ankle_link.py
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import *  # noqa: E402
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402  (v5 helpers: box/cyl_*/teardrop_*/csk_*/wedge_*)
import check_assembly as CA  # noqa: E402  (real servo mocks)

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")

AL_DROP = V.AL_DROP

# teardrop_y's roll toward print-up: 90 = model +X (RY_XUP)
ROLL_UP = 90

# the grip channel (leg_link_v6's numbers)
WEB_X1 = -12.36 - D.WEB_GAP                          # web inner face (cable gap)
WEB_X0 = WEB_X1 - 2.4                                # web outer face, -15.16
GRIP_X1 = 13.2                                       # grip plates' front edge
HORN_Y = (D.SV_TOPFACE, D.SV_TOPFACE + D.GRIP_PLATE_T)               # 17.35..19.75
IDLER_SEAT = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR                  # -14.90
IDLER_Y = (IDLER_SEAT - D.GRIP_PLATE_T_IDLER, IDLER_SEAT)            # -17.90..-14.90

X_REAR = V.AL_TINE_REAR_X[0]                         # -19.80, rear tine outer face
X_FRONT = V.AL_TINE_FRONT_X[1]                       # 23.45, front tine outer face
END_TOP = V.LL_IDLER_PLATE_TOP                       # -25.05

# servo lead window (leg_link's, just above GRIP_BOT), through the web
CABLE_WINDOW_Y = (-4.5, 4.5)
CABLE_WINDOW_Z = (D.GRIP_BOT + 1.0, D.GRIP_BOT + 8.0)


def _frame():
    """The box (web, sides, front wall), the floor and the tines: the solid
    before any hole or clearance cut."""
    # horn grip plate, full height over the case
    p = parts.box(WEB_X0, GRIP_X1, *HORN_Y, D.GRIP_BOT, D.GRIP_TOP_HORN)
    # side walls: both plates, web to front wall, GRIP_BOT..END_TOP
    for y0, y1 in (HORN_Y, IDLER_Y):
        p += parts.box(WEB_X0, X_FRONT, y0, y1, D.GRIP_BOT, END_TOP)
    # back web
    p += parts.box(WEB_X0, WEB_X1, IDLER_Y[0], HORN_Y[1], D.GRIP_BOT, D.WEB_TOP)
    # front wall: the front tine carried up across the full width
    p += parts.box(*V.AL_TINE_FRONT_X, IDLER_Y[0], HORN_Y[1], D.GRIP_BOT, END_TOP)
    # floor, rear tine to front wall
    p += parts.box(X_REAR, X_FRONT, *V.AL_FLOOR_Y, *V.AL_FLOOR_Z)
    # tines: AL_TINE_W at the pad, 45 deg shoulders out to AL_SHOULDER_Y at
    # the floor's underside, up to GRIP_BOT
    hw, sy = V.AL_TINE_W / 2, V.AL_SHOULDER_Y
    z_fl = V.AL_FLOOR_Z[0]
    z_sh = z_fl - (sy - hw)
    tine = [(-hw, -AL_DROP), (hw, -AL_DROP), (hw, z_sh), (sy, z_fl), (sy, D.GRIP_BOT),
            (-sy, D.GRIP_BOT), (-sy, z_fl), (-hw, z_sh)]
    for x0, x1 in (V.AL_TINE_REAR_X, V.AL_TINE_FRONT_X):
        p += parts.wedge_x(tine, x0, x1)
        p += parts.cyl_x(D.PAD_D / 2, x0, x1, 0, -AL_DROP)
    return p


def _grip_features(p):
    """leg_link_v6's grip-channel cuts, unchanged: horn boss relief, rib and
    platform detents (with their print ramps), case-grip screw holes."""
    t = D.GRIP_PLATE_T
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    # DETENT for the servo's horn-side RIB
    p -= parts.box(-D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR, D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR,
                   D.SV_TOPFACE - 0.01, D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
                   -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR,
                   D.GRIP_TOP_HORN + 0.01)
    _rx = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    _ry0 = D.SV_TOPFACE - 0.01
    _ry1 = D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR
    _rr = 1.2 * (_ry1 - _ry0)
    p -= parts.wedge_z([(_rx, _ry0), (_rx, _ry1), (_rx + _rr, _ry0)],
                       -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR - 0.6,
                       -D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR + 0.6)
    # DETENT for the idler-side PLATFORM
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0 = IDLER_SEAT
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= parts.box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    _irr = 1.2 * (_iy0 - _iy1)
    p -= parts.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                       _iz0 - 0.6, _iz1 + 0.6)
    # case-grip screws (M2.5 flat, countersunk flush)
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                                  D.SV_TOPFACE + t + 1, lx, -zrow, roll=ROLL_UP)
            p -= parts.csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, IDLER_Y[0] - 1,
                              IDLER_SEAT + 1, lx, -D.CASE_HOLES_BOT[1], roll=ROLL_UP)
        p -= parts.csk_y(lx, -D.CASE_HOLES_BOT[1], IDLER_Y[0], -1)
    return p


def _roll_sweep_clearance_cut():
    """The ankle-roll servo's swept envelope about the roll axis, ANGLE-PADDED
    by 1 deg beyond V.ROM['ankle_roll'] (~0.5 mm of arc at the case corners)
    and unioned over BOTH mounting orientations (+90/-90 about X): this part
    is the same on both legs but the foot is mirrored, so the roll servo's
    long (cable) side is on +y for one leg and -y for the other. Only the
    sweep is padded, not the seated servo: the tine pads sit flush on the
    discs by design."""
    lo, hi = V.ROM["ankle_roll"]
    pad = 1.0
    cut = None
    for sgn in (1, -1):
        base = Pos(0, 0, -AL_DROP) * (Rot(sgn * 90, 0, 0) * CA.servo_mock_x())
        deg = lo - pad
        while deg <= hi + pad + 1e-6:
            m = Pos(0, 0, -AL_DROP) * Rot(deg, 0, 0) * Pos(0, 0, AL_DROP) * base
            cut = m if cut is None else cut + m
            deg += 2.0
    return cut


def _tab_keepout():
    """The foot's two retention tabs (outboard of the roll servo: +y on the
    left foot, -y on the mirrored right) at full ankle roll, D.SWEEP_BUFFER
    clear. Each tab rises from the plate to its inner-top edge; rolled by the
    ROM limit, that edge is the apex of a wedge bounded by the tab's top face
    (outward) and its inner face (downward) -- every smaller roll angle lies
    inside the same wedge. Offsetting both faces by the buffer moves the apex
    in and up. Each wedge runs from its tab out through this part's end face,
    so the cuts are flat bevels with nothing over them in the print."""
    th = math.radians(max(abs(a) for a in V.ROM["ankle_roll"]))
    c, s = math.cos(th), math.sin(th)
    zt = V.FOOT_PLATE_T + V.FOOT_TAB_H - V.ANKLE_ROLL_ABOVE_PLATE   # tab top over the roll axis
    cut = None
    b = D.SWEEP_BUFFER
    for row, x0, x1 in ((V.FOOT_TAB_ROW_HORN, V.FOOT_TAB_FRONT_X[0] - b, X_FRONT + 10),
                        (V.FOOT_TAB_ROW_IDLER, X_REAR - 10, V.FOOT_TAB_REAR_X[1] + b)):
        yi = row - V.FOOT_TAB_HW                                     # tab inner face
        ay = yi * c - zt * s - b * (s + c)
        az = yi * s + zt * c + b * (c - s) - AL_DROP
        L = 40.0
        wedge = [(ay, az), (ay + L * c, az + L * s), (ay + L * s, az - L * c)]
        for sgn in (1, -1):
            w = parts.wedge_x([(sgn * y, z) for y, z in wedge], x0, x1)
            cut = w if cut is None else cut + w
    return cut


def ankle_link():
    p = _grip_features(_frame())

    # pads: centre relief and the 4x M3 BCD (bores are vertical in the print)
    r = D.BCD / 2
    for (x0, x1), relief_d in ((V.AL_TINE_FRONT_X, D.HORN_CENTER_RELIEF_D),
                               (V.AL_TINE_REAR_X, D.IDLER_CENTER_RELIEF_D)):
        p -= parts.cyl_x(relief_d / 2, x0 - 1, x1 + 1, 0, -AL_DROP)
        for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
            p -= parts.cyl_x(D.PAD_HOLE / 2, x0 - 1, x1 + 1, dy, -AL_DROP + dz)

    # servo lead window through the web
    p -= parts.box(WEB_X0 - 1, WEB_X1 + 1, *CABLE_WINDOW_Y, *CABLE_WINDOW_Z)

    p -= _roll_sweep_clearance_cut()
    p -= _tab_keepout()

    # the sweep cut can leave boolean fuzz (<< 1 mm3) as loose solids; keep
    # the link, and say so if what is dropped is not tiny
    solids = sorted(p.solids(), key=lambda s: s.volume, reverse=True)
    if len(solids) > 1:
        junk = sum(s.volume for s in solids[1:])
        tag = "boolean fuzz" if junk < 1.0 else "** REAL DETACHED VOLUME **"
        print(f"  [ankle_link] {len(solids)-1} extra solid(s) after the sweep "
              f"cut, {junk:.3f} mm3 total ({tag})")
        p = solids[0]
    return p


# ---------------------------------------------------------------------------
PRINT_ORIENT = "RY_XUP"    # model +X -> print +Z, leg_link's orientation

SERVO_INSERT = {
    # the ankle-pitch servo enters the grip channel from the top, like
    # leg_link's -- +z
    "ankle_link_grip": ((0, 0, 1), 32.0),
    # the link drops onto the roll servo already seated in the foot cradle,
    # tine pads onto its discs -- -z
    "ankle_link_fork": ((0, 0, -1), 20.0),
}


def SCREWS():
    """Every fastener in this part's local frame: {name, kind, pos, axis,
    length}. 6x M2.5 flat-head self-tap grip the ankle-pitch servo case
    (leg_link's own pattern); 8x M3 button (4 horn + 4 idler) grip the
    ankle-roll servo's discs through the tine pads. `axis` points from the
    head toward the tip."""
    s = []
    seat = D.SV_TOPFACE + D.GRIP_PLATE_T
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append({"name": f"grip_horn_{lx:+.1f}_{zrow:.1f}", "kind": "M2.5x8 flat self-tap",
                      "pos": (lx, seat, -zrow), "axis": (0, 1, 0), "length": 8.0})
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append({"name": f"grip_idler_{lx:+.1f}", "kind": "M2.5x8 flat self-tap",
                  "pos": (lx, IDLER_Y[0], -D.CASE_HOLES_BOT[1]), "axis": (0, 1, 0), "length": 8.0})
    # disc screws: each tine is a 3.0 plate straight on its disc, so both
    # take the length V.disc_screw gives a 3.0 stack (M3x5), no washers.
    # The head bears on the tine's OUTER face.
    r = D.BCD / 2
    for tag, head_x, stack in (("horn", V.AL_TINE_FRONT_X[1], V.AL_TINE_FRONT_X[1] - D.SV_HORN_FACE),
                               ("idler", V.AL_TINE_REAR_X[0], D.SV_IDLER_FACE - V.AL_TINE_REAR_X[0])):
        length, eng, flange = V.disc_screw(stack, tag)
        for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
            s.append({"name": f"tine_{tag}_{dy:+.1f}_{dz:+.1f}",
                      "kind": f"M3x{length} button",
                      "pos": (head_x, dy, -AL_DROP + dz), "axis": (-1, 0, 0) if tag == "horn" else (1, 0, 0),
                      "length": float(length), "stack": stack, "engage": eng, "flange": flange})
    return s


if __name__ == "__main__":
    import time
    t0 = time.time()
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    p = ankle_link()
    bb = p.bounding_box()
    mass = p.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    print(f"ankle_link  bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm  "
          f"vol {p.volume/1000:.1f} cm3  mass {mass:.1f} g  ({time.time()-t0:.1f}s build)")
    export_stl(p, os.path.join(OUT_STL, "ankle_link.stl"))
    export_step(p, os.path.join(OUT_STEP, "ankle_link.step"))
    print("wrote stl/ankle_link.stl, step/ankle_link.step")
