"""shoulder_girdle_v6: the torso's shoulder yoke -- one print that carries both
shoulder servos and the neck, so the arms stop being bolted-on brackets.

WHY THIS PART EXISTS
--------------------
Tom, 2026-09-19, looking at the round-4 arms in FreeCAD:

    "we need to revisit the arm design... looks like the arms are just bolted
     on. Aside from being brittle, it looks ugly. We should rotate the servo
     bodies 90deg and incorporate them into the torso, then put the head above
     them, so they look like real shoulders."
    "... and put the shoulder joint in the same plane as the hips."

Round 4 (arm_v6.shoulder_mount_v6, now deleted) stood each shoulder servo ON
END on the deck lid in a little cradle held by four M2.5 self-taps, with the
cradle floor cantilevering 21.7 mm past the deck edge and a 320 mm arm hanging
off it. Both complaints were fair. This part answers all four asks at once:

  ROTATED   the case turns 90 deg about its OWN output axis, so the case
            WIDTH (24.72) stands up and the case LENGTH (45.22) lies fore-aft.
            The axis stays lateral and the horn stays outboard -- which is why
            the arm LINKS (arm_upper_v6 / arm_fore_v6) are untouched by all of
            this. See dimensions_v6's "the shoulder servo's pose" block.
  IN THE    the pods are not brackets sitting on the lid: they are the torso's
  TORSO     own upper corners. The pod's inboard wall stands GIRDLE_SKIN_AIR =
            1.24 mm off the housing skin and the rails bolt down along the
            deck's stiffest line (the edge beam over the housing side wall).
  HEAD      the neck tube is part of this same solid, rising between the two
  ABOVE     pods (tube top 33.10, pod tops 31.32) so the head's base plate
            clears the shoulder line and the head sits above the shoulders
            instead of beside them.
  HIP PLANE ARM_SHOULDER_X = 0: the shoulder axis is directly above the hip
            yaw axis. That undid round 4b's aft mount, so it was RE-MEASURED
            (docs/design-v6/shoulder-girdle.md section 2) rather than assumed.

WHY IT IS NOT BRITTLE ANY MORE
------------------------------
Round 4's load path was: arm moment -> 4x M2.5 self-tap in tension -> a 5 mm
deck plate, with the bracket hanging off the edge. Here it is: arm moment ->
4x M2.5 into the servo's HORN-side face -> a grip plate that is one wall of a
closed pod box (floor + two end walls + inboard wall) -> a 4 mm rail bolted to
the deck at four stations along its length -> and, across the robot, a forward
clavicle beam tying the two pods together. The couple is reacted over the
girdle's own 31 mm depth and the rail's 54 mm length instead of by screw
tension in a thin plate.

THE GRIP PLATE IS ON THE HORN SIDE, AND THAT IS AN ACCESS RESULT
----------------------------------------------------------------
Every other servo on this robot is gripped on its idler face. Not this one:
with the case lying against the torso there is 1.24 mm between the girdle's
inboard wall and the housing skin, and no driver reaches into a 1.24 mm slot.
The horn-side face is open air, so the four case screws are driven from
OUTBOARD, through a plate with D.GRIP_HORN_RELIEF around the O19.6 case boss
-- exactly leg_link's own horn-side grip plate. The inboard wall keeps its
job (locating the case, tying floor to top) but carries no screws.

PRINT: BASE DOWN, AND NOTHING HANGS BELOW z = 0
-----------------------------------------------
The part prints on its deck face (IDENT), walls rising -- the pelvis's own
orientation logic. That is the single constraint that fixes the shoulder
HEIGHT: the servo bay's floor cannot dip below the bed plane, so the axis sits
GIRDLE_FLOOR + SV_WID/2 = 16.36 above the deck top and no lower. A low-slung
pod straddling the deck plane was drawn first and thrown away for exactly this
reason (it bridges 125 mm of air between its two skirts).
Every bay is open UPWARD: all four pod walls are full height, which does not
block a servo dropped straight down, and a roof would (a flat 32 mm ceiling is
unprintable and a 45 deg vault over the same span is 32 mm tall). The servos
drop in, then their screws go in from outboard.

    .venv/bin/python cad/v6/shoulder_girdle_v6.py          # STL + STEP + checks + render
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

from build123d import Plane, Pos, Rot, export_step, export_stl, fillet, mirror  # noqa: E402
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402
import check_assembly as CA  # noqa: E402
import neck_floor as NF  # noqa: E402

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

IDENT = np.eye(3)
PRINT_ORIENT = {"shoulder_girdle_v6": (IDENT, "base down on the deck face, walls up")}
# OrcaSlicer supports (Tom, 2026-09-24: support work is the slicer's job
# wherever it can do it). What needs them, base down: the four trapezius-web
# window tops (34 mm spans), the servo bays' relief/pocket roofs and the neck
# tube's aft window top. All are open to the plate, so supports come out.
SUPPORT_NOTE = ("supports on (build plate only): trapezius-web window tops, the "
                "grip plates' rib-relief roofs, the bays' disc-relief tops")

# ---------------------------------------------------------------------------
# derived, local to this module (everything dimensional lives in dimensions_v6)
# ---------------------------------------------------------------------------
_AX = V.ARM_SHOULDER_X                    # 0.0  -- the hip plane
_AZ = V.ARM_AXIS_Z                        # 16.36 above the deck top
CASE_X0, CASE_X1 = V.ARM_CASE_X           # -35.11 .. +10.11 (cable end AFT)
BAY_X = (CASE_X0 - 0.5, CASE_X1 + 0.5)    # 0.5 mm of slip either end
END_T = D.WALL                            # 2.6 pod end walls
POD_X = (BAY_X[0] - END_T, BAY_X[1] + END_T)              # -38.21 .. +13.21
CASE_Z = (_AZ - D.SV_WID / 2, _AZ + D.SV_WID / 2)         # 4.00 .. 28.72
POD_TOP = CASE_Z[1] + D.WALL                              # 31.32
WY0, WY1 = V.GIRDLE_WALL_Y0, V.ARM_SEAT_Y                 # 62.50 .. 65.50 inboard wall
GRIP_Y0, GRIP_Y1 = V.ARM_CASE_Y1, V.GIRDLE_Y1             # 97.75 .. 100.15 grip plate
BASE_T = V.GIRDLE_FLOOR                                   # 4.0

RAIL_Y0 = 53.8          # rail inboard edge. Set by the deck-pilot countersink:
                        # its O5.4 mouth at y 57 reaches 54.3, and a rail edge
                        # at 54.8 (the first cut, which stopped at the battery
                        # aperture's 54.66) left a sub-mm sliver the audit
                        # flagged THIN. It now overhangs the aperture edge by
                        # 0.86 mm, which costs nothing: the pack goes in BEFORE
                        # the girdle (it cannot pass the neck tube either way).
assert RAIL_Y0 + 0.5 <= V.GIRDLE_RAIL_PILOT_Y - D.CASE_CS_D / 2, \
    "the rail pilots' countersinks would break out of the rail's inboard edge"
RAIL_X = (-42.0, 16.0)  # aft of the pod (extra bolting land) to the clavicle
CLAV_X = (POD_X[1] - END_T, 16.0)     # +10.61 .. +16.0 forward clavicle beam
CLAV_HY = 58.0                        # inside the deck's R3 top fillet (61.26 - 3)
CLAV_H = 12.0
TIE_X = (-36.0, -31.0)                # aft tie: a bridge, no bolts (see below)
TIE_H = 10.0
WEB_WIN_Y = (22.0, 56.0)              # trapezius-web window (|y| band)
WEB_WIN_Z = (7.0, 25.5)               # ...and its z band
# 45 deg underside chamfer on the pod's outboard edge (the shoulder flare).
# Its size is set by the LOWEST case screw, not by eye: the chamfer must stay
# clear of that screw's countersink MOUTH, whose bottom edge sits at
# axis - CASE_HOLE_LAT - CASE_CS_D/2 above the deck.
_CSK_BOT_Z = _AZ - D.CASE_HOLE_LAT - D.CASE_CS_D / 2      # 3.01
GIRDLE_FLARE = 3.0
assert GIRDLE_FLARE <= _CSK_BOT_Z, \
    "the pod flare would eat the lowest case screw's countersink"

GIRDLE_CORNER_R = 6.0                 # pod outer vertical corners (2x ARM_EDGE_R:
                                      # this is the robot's outline at the shoulder)

# neck tube -- the same footprint neck_collar.py builds, fused in here
NCX0, NCX1 = V.NECK_WELL_X                    # -35.41, +10.41
NC_HY = V.NECK_WELL_HW[0]                     # 12.66
NC_WALL = D.WALL
COLLAR_TOP = V.NECK_AXIS_Z + D.SV_TOPFACE + 1.0       # 33.10, 2.1 short of the horn disc
AFT_WIN = (12.0, 10.0)

# deck pilots -- EVERY one of these is verified against the real pelvis solid
# by check_deck_pilots() below, because the deck here is cut by the Pi slide
# slot (x -51.76..-37.76, |y| <= 45), the battery aperture (x -36.51..+10.41,
# |y| < 54.66), the General Driver lead slot and the R3 top fillet. The rail
# pilots sit on the one continuous strip of solid deck that survives all of
# that: y 55..59, the edge beam over the housing side wall.
PILOT_D = V.ARM_PILOT_D           # 2.05
PILOT_DEPTH = V.ARM_PILOT_DEPTH   # 4.5


def pilot_xy():
    """Every deck pilot the girdle needs, pelvis frame, both sides. Lives in
    dimensions_v6 so pelvis_v7 drills exactly what this part clears."""
    return list(V.GIRDLE_PILOT_XY)


# ===========================================================================
# the part
# ===========================================================================
def _pod(sgn):
    """One shoulder pod, on the +y side then mirrored. Open UPWARD (the servo
    drops in) and open OUTBOARD above the grip plate (the arm swings there)."""
    y = lambda v: sgn * v  # noqa: E731

    # --- floor under the case, from the rail out to the grip plate
    p = parts.box(POD_X[0], POD_X[1], y(RAIL_Y0), y(GRIP_Y1), 0, BASE_T)
    # --- inboard wall: locates the case, ties floor to top, carries no screws
    p += parts.box(POD_X[0], POD_X[1], y(WY0), y(WY1), 0, POD_TOP)
    # --- outboard grip plate: THE structural wall, 4x case screws from +y
    p += parts.box(POD_X[0], POD_X[1], y(GRIP_Y0), y(GRIP_Y1), 0, POD_TOP)
    # --- two end walls wrapping the case fore and aft
    p += parts.box(POD_X[0], BAY_X[0], y(WY0), y(GRIP_Y1), 0, POD_TOP)
    p += parts.box(BAY_X[1], POD_X[1], y(WY0), y(GRIP_Y1), 0, POD_TOP)

    # --- grip-plate relief: the O19.6 case boss stands proud of the horn face
    # and the horn disc TURNS inside it, so the plate is opened to
    # D.GRIP_HORN_RELIEF, leg_link's own number for this same joint.
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, y(GRIP_Y0 - 1), y(GRIP_Y1 + 1), _AX, _AZ)
    # --- the four case screws, driven from OUTBOARD into the horn-side face.
    # Rows run from the axis toward the CABLE end, which is -x here; the
    # lateral pair is across the case width, which is z here.
    for xrow in D.CASE_HOLES_TOP:
        for lz in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, y(GRIP_Y0 - 1), y(GRIP_Y1 + 1),
                                  _AX - xrow, _AZ + lz, roll=0)
            p -= parts.csk_y(_AX - xrow, _AZ + lz, y(GRIP_Y1), sgn)

    # --- relief for the moulded HORN-side rib. The case has a 1.13 mm rib
    # standing proud of its horn face along most of its length; a flat plate
    # lands on the rib instead of the face and the servo sits crooked. Pocket,
    # not window: the four screws all land outside the rib's band (rows at
    # z +-10.25 vs the rib's +-7.82), so they still bear on full plate.
    _rx = (_AX - D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR,
           _AX - D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR)
    _rz = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    p -= parts.box(_rx[0], _rx[1], y(GRIP_Y0 - 0.01),
                   y(GRIP_Y0 + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR),
                   _AZ - _rz, _AZ + _rz)

    # --- inboard wall openings. ONE keyhole, for the same reason round 4's
    # cradle used one: cut separately, the disc relief and the connector
    # window leave a sliver of wall between them that prints as a strand.
    #   * the idler DISC turns with the joint and stands 2.05 proud -> a hole
    #     on the axis, clipped at the floor so the wall/floor joint stays solid
    #   * the connector TRENCH is where the servo's own lead plugs in. Lying
    #     down, that trench faces INBOARD into a 1.24 mm slot, so the window is
    #     cut OPEN TO THE TOP of the wall: the lead leaves the plug and goes
    #     straight UP out of the notch, and never has to fit the slot.
    rel = D.SV_IDLER_MOAT_R - 0.4                                   # 10.6
    opening = parts.cyl_y(rel, y(WY0 - 1), y(WY1 + 1), _AX, _AZ)
    # the notch runs INTO the relief, to its centre line: stopping at the
    # trench's own edge left a 0.55 mm post between them, and stopping just
    # inside the circle left thin cusps where the circle curves away -- both
    # THIN audit findings. Ending on the axis, the notch's edge meets the
    # circle square at its top and bottom, so no cusp exists to be thin.
    opening += parts.box(_AX - D.SV_CONN_L[1] - 0.6, _AX,
                         y(WY0 - 1), y(WY1 + 1), _AZ - D.SV_CONN_HW, POD_TOP + 1)
    opening -= parts.box(POD_X[0] - 1, POD_X[1] + 1, y(WY0 - 2), y(WY1 + 2),
                         -1, BASE_T + 0.4)
    p -= opening
    # --- detent for the moulded back-cover platform (leg_link's, rotated: the
    # band runs along the case LENGTH, which is -x here, and its half width is
    # across the case, which is z here).
    # the band runs axis -> cable end, and the cable end is -x, so the band's
    # own (negative) numbers are already in this frame; GROW it by the
    # clearance at both ends, do not shrink it.
    _bx = (_AX + D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR,
           _AX + D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR)
    _bz = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _by = V.ARM_SERVO_MID_Y + D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    p -= parts.box(_bx[0], _bx[1], y(_by), y(WY1 + 0.01), _AZ - _bz, _AZ + _bz)
    return p


def shoulder_girdle_v6():
    """The whole girdle, in the PELVIS frame (deck top z = 0, robot forward
    +x, left +y) so it reads directly against pelvis_v7's own geometry.
    Qty 1 -- it is symmetric, so it is NOT a mirror pair.
    """
    p = _pod(+1) + _pod(-1)

    # --- the two rails: 4 mm plate on the deck's edge beam, carrying the
    # pod straight into the stiffest line the deck has.
    for sgn in (1, -1):
        p += parts.box(RAIL_X[0], RAIL_X[1], sgn * RAIL_Y0, sgn * WY1, 0, BASE_T)

    # --- forward clavicle beam: the transverse tie. It can only live here --
    # forward of x +10.5 is the one place the deck is solid all the way across
    # (everything aft of it is the battery aperture).
    # --- aft tie: a BRIDGE over the battery aperture, landing on both rails.
    # No pilots -- there is no deck under it. It is free stiffness: the pack
    # already cannot pass the neck tube, so the aperture is not an access path
    # once the girdle is on either way (see the assembly-order note below).
    #
    # Both bands also carry the TRAPEZIUS WEB: a full-height panel from the
    # neck tube out to each pod. Structurally it is the top chord -- it ties
    # the two pods to each other and to the neck at the height the arm's
    # moment actually acts, instead of only at the deck. Visually it is the
    # whole point: without it the girdle reads as three separate lumps (a box,
    # a tube, a box) rather than one shoulder line running out of the neck.
    # Fenestrated because a solid panel is 8 g of nothing, and because the
    # window repeats the rounded-rectangle opening used everywhere else.
    for x0, x1, h in ((CLAV_X[0], CLAV_X[1], CLAV_H), (TIE_X[0], TIE_X[1], TIE_H)):
        p += parts.box(x0, x1, -CLAV_HY, CLAV_HY, 0, h)
        for sgn in (1, -1):
            p += parts.box(x0, x1, sgn * (NC_HY + NC_WALL), sgn * WY1, 0, POD_TOP)
            p -= parts.box(x0 - 1, x1 + 1, sgn * WEB_WIN_Y[0], sgn * WEB_WIN_Y[1],
                           WEB_WIN_Z[0], WEB_WIN_Z[1])

    # --- the neck tube: the head's mount, rising BETWEEN the pods
    p += parts.box(NCX0 - NC_WALL, NCX1 + NC_WALL, -(NC_HY + NC_WALL),
                   NC_HY + NC_WALL, 0.0, COLLAR_TOP)
    # case cavity, open top (the mouth) and open bottom: the neck servo's seat
    # is neck_floor.py, a plate screwed up into four bosses on the tube's
    # side walls that drops into the deck's battery aperture at the old well
    # depth (issue #90). It is its own print because it hangs below this
    # part's print plane.
    p -= parts.box(NCX0, NCX1, -NC_HY, NC_HY, -1.0, COLLAR_TOP + 1.0)
    # aft window at the case's cable end
    p -= parts.box(NCX0 - NC_WALL - 1.0, NCX0 + 1.0, -AFT_WIN[0] / 2, AFT_WIN[0] / 2,
                   0.0, AFT_WIN[1])
    boss_add, boss_cut = NF.tube_bosses()
    p += boss_add
    p -= boss_cut

    # --- deck pilots: clearance + countersink through whatever sits over them
    for hx, hy in pilot_xy():
        p -= parts.cyl_z(D.CASE_SCREW_CLEAR / 2, -1, BASE_T + 1, hx, hy)
        p -= parts.csk_z(hx, hy, BASE_T, +1)

    # --- the flare. A 45 deg chamfer under each pod's OUTBOARD edge, so the
    # shoulder grows out of the torso instead of stepping off it: the torso
    # skin is 61.26 wide and the pod reaches 100.15, and without this the
    # front view is a hard T. 45 deg because the part prints base-down and an
    # underside shallower than that is an unsupported overhang. GIRDLE_FLARE
    # is capped by the LOWEST case screw (z 6.11): the chamfer must not eat
    # its countersink.
    for sgn in (1, -1):
        p -= parts.wedge_x([(sgn * GRIP_Y1, 0.0),
                            (sgn * GRIP_Y1, GIRDLE_FLARE),
                            (sgn * (GRIP_Y1 - GIRDLE_FLARE), 0.0)],
                           POD_X[0] - 1, POD_X[1] + 1)

    p = _style(p)
    return p


def _style(p):
    """The blends. Functionality first -- these are all cosmetic or
    stress-relief, so every one is attempted in its own try/except and the
    build never fails on a refused fillet (hip_yoke_v6's _blend_one rule).

    Design language, and where it comes from (docs/design-v6/shoulder-girdle.md
    section 5): ONE form repeated per joint type, soft curved surfaces with
    eased edges rather than arrises, and a silhouette that flares continuously
    from the torso out to the shoulder rather than stepping.
    """
    # 1. the outer vertical corners of each pod -- the corners a hand reaches
    # past, and the ones that read as the shoulder's outline from the front.
    try:
        edges = [e for e in p.edges()
                 if abs(e.tangent_at(0.5).Z) > 0.999
                 and abs(abs(e.center().Y) - (GRIP_Y0 + GRIP_Y1) / 2) < 25
                 and abs(e.center().Y) > WY1
                 and (abs(e.center().X - (POD_X[0] + 0.0)) < 3.0
                      or abs(e.center().X - POD_X[1]) < 3.0)]
        for r in (GIRDLE_CORNER_R, 4.5, 3.0, 2.0):
            try:
                p = fillet(edges, r)
                print(f"  [girdle] pod corners rounded R{r}")
                break
            except Exception:  # noqa: BLE001 -- OCC refuses radii it cannot fit
                continue
    except Exception as e:  # noqa: BLE001
        print(f"  [girdle] pod corner fillet skipped ({type(e).__name__}: {str(e)[:70]})")
    # 2. ease every top edge: this is the surface a finger runs along, and on
    # a base-down print a top edge is free (no overhang either way). Blended
    # ONE EDGE AT A TIME -- OCC refuses a whole group if any single member is
    # impossible, and hip_yoke_v6._blend_one exists for exactly this reason.
    tops = []
    for e in p.edges():
        z = e.center().Z
        if any(abs(z - t) < 1e-6 for t in (POD_TOP, CLAV_H, TIE_H, COLLAR_TOP)) and e.length > 4.0:
            tops.append(e)
    done = 0
    for e in tops:
        for r in (1.6, 0.8):
            try:
                p = fillet(p.edges().filter_by_position(
                    __import__("build123d").Axis.Z, e.center().Z - 1e-4, e.center().Z + 1e-4
                ).sort_by_distance(e.center())[0:1], r)
                done += 1
                break
            except Exception:  # noqa: BLE001
                continue
    print(f"  [girdle] top edges eased: {done}/{len(tops)}")
    return p


# ===========================================================================
# mocks
# ===========================================================================
def shoulder_servo_mock(side="L"):
    """The shoulder STS3215 in the PELVIS frame. The canonical mock has the
    axis +Y, horn +Y, case length along Z with the output end at +10.11; here
    it is turned +90 about Y, which maps the case's +Z (output end) onto +X: the
    case LENGTH lies fore-aft with the output end FORWARD and the cable end
    AFT, and the case WIDTH (24.72) stands up."""
    m = Pos(_AX, V.ARM_SERVO_MID_Y, _AZ) * Rot(0, 90, 0) * CA.servo_mock()
    return m if side == "L" else mirror(m, Plane.XZ)


def mock_neck_servo():
    return Pos(V.NECK_X, 0, V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()


# ===========================================================================
# fasteners + insertion paths (the contract leg_link_v6.SCREWS keeps)
# ===========================================================================
def SCREWS():
    """Every fastener the girdle adds, with the frame each is quoted in.
    `axis` points from the head toward the tip."""
    out = []
    for hx, hy in pilot_xy():
        top = BASE_T
        out.append(dict(name=f"girdle_deck_{hx:+.0f}_{hy:+.0f}", frame="pelvis",
                        kind="M2.5x8 self-tap, flat head (2.05 x 4.5 pilot in the deck)",
                        pos=(hx, hy, top), axis=(0, 0, -1), length=8.0))
    for side, sgn in (("L", 1), ("R", -1)):
        for xrow in D.CASE_HOLES_TOP:
            for lz in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
                out.append(dict(name=f"shoulder_case_{side}_{xrow:.0f}_{lz:+.0f}", frame="pelvis",
                                kind="M2.5x8 self-tap, flat head (into the servo case's HORN-side face,"
                                     " driven from OUTBOARD -- the inboard face is 1.24 mm off the skin)",
                                pos=(_AX - xrow, sgn * GRIP_Y1, _AZ + lz),
                                axis=(0, -sgn, 0), length=8.0))
    return out


# the order IS the bench order; animate_v6 flies the parts in along these.
# On the bench, before the girdle goes on the robot: the neck floor is screwed
# up into the tube's bosses, and the neck servo drops into the tube onto it
# (neck_floor.INSERT) and is screwed from below -- the plate sits over the
# pack, so none of that is reachable once the girdle is on the deck. Then the
# girdle is lowered onto the deck (the pack is already in) and screwed down
# through its two rails; the rail pilots are clear of the bays, so the
# shoulder servos can go in before or after.
INSERT = {
    "shoulder_girdle_v6": (0, 0, 1),   # lowered onto the deck, screwed down through the rails
    "servo_shoulder": (0, 0, 1),       # dropped into the open bay from above
}


# ===========================================================================
# checks
# ===========================================================================
def check_pilot_access(verbose=True):
    """Every deck screw has to be DRIVABLE: nothing above its seat but air,
    within a 7 mm driver's radius (the check the print list quotes). The
    first girdle failed this for six of its twelve pilots, which sat under
    the clavicle beam, the aft tie and the webs."""
    p = shoulder_girdle_v6()
    ok = True
    for hx, hy in pilot_xy():
        shaft = parts.cyl_z(3.5, BASE_T + 0.5, POD_TOP + 5, hx, hy)
        try:
            hit = (shaft & p).volume
        except Exception:  # noqa: BLE001
            hit = float("nan")
        good = hit < 0.5
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  driver access to pilot ({hx:+6.1f},{hy:+6.1f}): "
                  f"{hit:6.2f} mm3 in the way")
    return ok


def check_deck_pilots(verbose=True):
    """Every deck pilot must be fully BURIED in the pelvis deck: 4.5 mm into a
    5 mm plate leaves 0.5 mm of floor. Checked against the REAL pelvis solid,
    both sides -- not against a drawing of where the cuts are. This check is
    the reason the rail sits at y 57: it is the only continuous strip of deck
    that survives the Pi slot, the battery aperture and the GD lead slot.
    """
    import pelvis_v7 as PV
    solid = PV.pelvis_v7()
    ok = True
    for hx, hy in pilot_xy():
        pilot = parts.cyl_z(PILOT_D / 2, -PILOT_DEPTH, 0.0, hx, hy)
        try:
            stray = (pilot - solid).volume
        except Exception:  # noqa: BLE001
            stray = float("nan")
        good = stray < 0.5
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  deck pilot ({hx:+6.1f},{hy:+6.1f})  "
                  f"outside the deck: {stray:6.2f} mm3")
    return ok


def check_servo_fit(verbose=True):
    """The two shoulder servo mocks must sit INSIDE their bays without
    touching the girdle, and the girdle must not reach into the plane the
    rotating arm plate occupies."""
    p = shoulder_girdle_v6()
    ok = True
    for side in ("L", "R"):
        m = shoulder_servo_mock(side)
        try:
            overlap = (m & p).volume
        except Exception:  # noqa: BLE001
            overlap = float("nan")
        good = overlap < 1.0
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  shoulder servo {side} vs girdle: "
                  f"{overlap:7.2f} mm3 of overlap")
    bb = p.bounding_box()
    wide = bb.size.Y
    good = wide <= D.BED
    ok &= good
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  girdle width {wide:.2f} mm <= bed {D.BED:.0f}")
        print(f"  {'PASS' if bb.min.Z > -1e-6 else 'FAIL'}  nothing below the deck plane "
              f"(min z {bb.min.Z:+.3f})")
    ok &= bb.min.Z > -1e-6
    return ok


def check_arm_clearance(verbose=True):
    """The girdle's outer face must stay clear of the plane the arm's horn
    plate bolts to, or the arm grinds on the torso every time it swings."""
    gap = V.ARM_HORN_FACE_Y - V.GIRDLE_Y1
    good = gap >= 0.5
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  girdle outer face {V.GIRDLE_Y1:.2f} vs arm "
              f"horn face {V.ARM_HORN_FACE_Y:.2f}: {gap:.2f} mm of air")
    return good


def check_head_above_shoulders(verbose=True):
    """Tom's actual ask: the head has to sit ABOVE the shoulders, not between
    them. Measured as the head's base plate vs the pod tops."""
    head_base = V.NECK_HORN_FACE_Z          # 35.20 above the deck top
    good = head_base > POD_TOP
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  head base {head_base:.2f} vs pod top "
              f"{POD_TOP:.2f} above the deck: {head_base - POD_TOP:+.2f} mm")
        print(f"        neck tube top {COLLAR_TOP:.2f}, shoulder axis {_AZ:.2f}, "
              f"head top {V.TOP_Z - V.DECK_TOP_Z:.2f}")
    return good


def check_print(verbose=True):
    """check_printability's audit in the declared orientation (base down).
    Anything the audit calls an overhang that OrcaSlicer can support goes in
    SUPPORT_NOTE and is left to the slicer -- nothing is modelled for it."""
    import check_printability as CP
    rot, note = PRINT_ORIENT["shoulder_girdle_v6"]
    CP.STL = OUT_STL
    CP.ORIENT["shoulder_girdle_v6"] = (rot, note)
    CP.PRINT_STL["shoulder_girdle_v6"] = "shoulder_girdle_v6.stl"
    if SUPPORT_NOTE:
        CP.SUPPORTED["shoulder_girdle_v6"] = SUPPORT_NOTE
    return not CP.audit("shoulder_girdle_v6")


def run_checks():
    print("\n== printability (base down) ==")
    ok = check_print()
    print("\n== driver access to every deck pilot ==")
    ok &= check_pilot_access()
    print("\n== deck pilots (against the real pelvis solid) ==")
    ok &= check_deck_pilots()
    print("\n== servo fit / print envelope ==")
    ok &= check_servo_fit()
    print("\n== arm vs girdle clearance ==")
    ok &= check_arm_clearance()
    print("\n== head above the shoulders ==")
    ok &= check_head_above_shoulders()
    return ok


def mass_g(solid, density=1.27e-3):
    return solid.volume * density


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checks", action="store_true", help="run the checks too (slow: builds the pelvis)")
    a = ap.parse_args()
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    p = shoulder_girdle_v6()
    bb = p.bounding_box()
    print(f"shoulder_girdle_v6  {bb.size.X:6.2f} x {bb.size.Y:6.2f} x {bb.size.Z:6.2f} mm   "
          f"{mass_g(p):5.1f} g   {'BED-OK' if max(bb.size.X, bb.size.Y, bb.size.Z) <= D.BED else '** TOO BIG **'}")
    export_stl(p, os.path.join(OUT_STL, "shoulder_girdle_v6.stl"))
    export_step(p, os.path.join(OUT_STEP, "shoulder_girdle_v6.step"))
    print("wrote stl + step")
    if a.checks:
        ok = run_checks()
        print("\nALL CHECKS PASS" if ok else "\n** CHECKS FAILED **")


if __name__ == "__main__":
    main()
