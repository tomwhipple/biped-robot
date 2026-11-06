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
4x M2.5 into the servo's HORN-side face -> the CHEEK, a grip plate that closes
the pod box (floor + two end walls + inboard wall), nested in a rebate between
the end lugs' rims and screwed to them at its four corners -> a 4 mm rail
bolted to the deck at four stations along its length -> and, across the robot,
a forward clavicle beam tying the two pods together. The case itself is boxed
by the pod: its platform and free-hub post sit in the inboard wall, its boss
in the cheek, so the cheek's screws locate a plate, they do not carry the arm
alone. The couple is reacted over the girdle's own 31 mm depth and the rail's
54 mm length instead of by screw tension in a thin plate.

THE SERVO GOES IN ALONG ITS OWN AXIS, SO THE GRIP PLATE IS ITS OWN PRINT
------------------------------------------------------------------------
Tom, 2026-11-01: "It is currently impossible to insert a servo into their
slots due to the conformal features." The one-piece girdle had all four pod
walls full height and the servo dropping in from above, but every feature the
case has on its two big faces stands proud ALONG THE AXIS: the O19.6 output
boss (3.1 proud, through the grip plate) and the horn-side rib (1.13) outboard,
the back-cover platform (1.9) and the free-hub post (2.6) inboard. Each sat in
a closed pocket of a full-height wall, so a vertical drop met wall above the
axis on both sides (check_insertion measures it). Every one of those features
engages ALONG the axis, so that is the path: the servo is pushed in from
OUTBOARD onto the inboard wall, then the cheek (shoulder_cheek_v6, the old
grip plate) goes on along the same axis over the boss, is screwed to the end
lugs, and the four case screws go in through it. INSERT carries both paths.

THE GRIP PLATE IS ON THE HORN SIDE, AND THAT IS AN ACCESS RESULT
----------------------------------------------------------------
Every other servo on this robot is gripped on its idler face. Not this one:
with the case lying against the torso there is 1.24 mm between the girdle's
inboard wall and the housing skin, and no driver reaches into a 1.24 mm slot.
The horn-side face is open air, so the four case screws are driven from
OUTBOARD, through a plate with D.GRIP_HORN_RELIEF around the O19.6 case boss
-- exactly leg_link's own horn-side grip plate. The inboard wall carries no
screws, so it holds the case by fit alone: the shoulder servos go in WITHOUT
their idler disc (the hip-yaw rule), the wall bears on the bare case face
with a hole for the free-hub post, and four crush pads on its seat face are
squashed as the cheek's screws push the case home (dimensions_v6
GIRDLE_PAD_*), so the case is clamped between the wall and the cheek.

PRINT: BASE DOWN, AND NOTHING HANGS BELOW z = 0
-----------------------------------------------
The part prints on its deck face (IDENT), walls rising -- the pelvis's own
orientation logic. That is the single constraint that fixes the shoulder
HEIGHT: the servo bay's floor cannot dip below the bed plane, so the axis sits
GIRDLE_FLOOR + SV_WID/2 = 16.36 above the deck top and no lower. A low-slung
pod straddling the deck plane was drawn first and thrown away for exactly this
reason (it bridges 125 mm of air between its two skirts).
Every bay is open UPWARD (a flat 32 mm ceiling is unprintable and a 45 deg
vault over the same span is 32 mm tall) and, with its cheek off, OUTBOARD.
The cheeks print flat, outer face down (PRINT_ORIENT), so their eight bores
are vertical and need no teardrops.

    .venv/bin/python cad/v6/shoulder_girdle_v6.py          # STL + STEP (girdle + both cheeks)
    .venv/bin/python cad/v6/shoulder_girdle_v6.py --checks # + printability, fits, insertion paths
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

from build123d import Plane, Pos, Rot, export_step, export_stl, mirror  # noqa: E402
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402
import check_assembly as CA  # noqa: E402
import neck_floor as NF  # noqa: E402
import paths  # noqa: E402

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

IDENT = np.eye(3)
RY_YDOWN = np.array([(1, 0, 0), (0, 0, 1), (0, -1, 0)], float)       # model +Y -> print -Z
PRINT_ORIENT = {"shoulder_girdle_v6": (IDENT, "base down on the deck face, walls up"),
                "shoulder_cheek_v6": (RY_YDOWN, "flat, outer face down (the left pod's pose; x2)")}
# NO SUPPORTS (Tom, 2026-11-01: "let's get rid of the windows front and back.
# Instead let's just put a thin wall there so we don't have to worry about
# print supports"). The four trapezius-web windows were 34 mm flat-topped
# spans, supported inside enforcer boxes; they are thin walls now, the neck
# tube's aft window is gone, and every other roof on the part -- the web
# recesses behind those walls, the platform detents -- is a 50 deg slope.
SUPPORT_NOTE = None
ROOF_RISE = 1.2   # a self-supporting roof rises 1.2 mm per mm it overhangs: 50 deg, clear of
                  # the 45 deg line check_printability calls one way or the other on rounding

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
                        # 0.86 mm, which costs nothing: the pack swaps through
                        # the torso's side door (torso_v8), not the aperture.
assert RAIL_Y0 + 0.5 <= V.GIRDLE_RAIL_PILOT_Y - D.CASE_CS_D / 2, \
    "the rail pilots' countersinks would break out of the rail's inboard edge"
RAIL_X = (-42.0, 16.0)  # aft of the pod (extra bolting land) to the clavicle
CLAV_X = (POD_X[1] - END_T, 16.0)     # +10.61 .. +16.0 forward clavicle beam
CLAV_HY = 58.0                        # 3 mm inside the deck edge (61.26)
CLAV_H = 12.0
TIE_X = (-36.0, -31.0)                # aft tie: a bridge, no bolts (see below)
TIE_H = 10.0
WEB_T = 1.6                           # the trapezius web's thin wall, flush with the band's OUTER face
WEB_REC_Y = (22.0, 56.0)              # the lightening recess behind it (|y| band) ...
WEB_REC_TOP = 25.5                    # ... from the beam's top up to here at the open (inner) face
# the elbow lead's slot through the AFT tie's web (#198; Tom, 2026-11-04: "aft
# is correct"). The shoulder-to-elbow lead rises out of the inboard wall's
# notch (x -12..0), runs aft inside the girdle to the tie, out through this
# slot, round the pod's aft lug and down the back of the upper arm's web. It
# sits at the recess's outboard end, so its outboard face is the recess's own
# and the 6.5 mm of web between it and the pod's inboard wall stays whole;
# its floor is the beam's top, so the beam stays whole too. The lead's plug
# has to pass (whichever end is threaded through): D.WIRE_PLUG_W 9 wide, with
# the arm's own 11 mm cable-window size across the slot.
LEAD_SLOT_Y = (WEB_REC_Y[1] - 11.0, WEB_REC_Y[1])   # |y| 45 .. 56
LEAD_SLOT_EAVE = 17.0                 # straight sides from the beam's top (TIE_H) to here, then a
                                      # ROOF_RISE peak (+6.6, to 23.6): 7.7 mm of web left above it
# No flare under the pod's outboard edge any more (Tom, 2026-11-01): it was
# a 45 deg chamfer for the one-piece girdle, which printed that edge base-down;
# the cheek prints flat, and the edges stay square.

# the cheek and the lugs it screws to. The cheek's four screws sit on the case
# screws' own two rows, just outboard of each end of the bay; the cheek stops
# GIRDLE_CHEEK_EDGE past their countersinks, and the lugs carry a rim past
# that, so the cheek drops into a rebate and is boxed fore-aft by the pod.
# Its height is centred on the shoulder axis (top flush with the pod top),
# and every feature on it is symmetric about the axis, so the right pod's
# cheek is the left's turned over: ONE part, printed twice. The 1.4 mm under
# it is a ledge on the floor, which it sits on.
CSK_MOUTH_R = (D.CASE_SCREW_CLEAR / 2
               + (D.CASE_CS_D / 2 + 0.3 - D.CASE_SCREW_CLEAR / 2) * D.CASE_CS_DEPTH / (D.CASE_CS_DEPTH + 0.3))  # 2.70
CHEEK_SCREW_X = (BAY_X[0] - V.GIRDLE_CHEEK_SCREW_DX, BAY_X[1] + V.GIRDLE_CHEEK_SCREW_DX)   # -38.41, +13.41
CHEEK_SCREW_Z = (_AZ - D.CASE_HOLE_LAT, _AZ + D.CASE_HOLE_LAT)                          # 6.11, 26.61
CHEEK_X = (CHEEK_SCREW_X[0] - CSK_MOUTH_R - V.GIRDLE_CHEEK_EDGE,
           CHEEK_SCREW_X[1] + CSK_MOUTH_R + V.GIRDLE_CHEEK_EDGE)                         # -41.71, +16.71
REBATE_X = (CHEEK_X[0] - V.GIRDLE_CHEEK_CLR, CHEEK_X[1] + V.GIRDLE_CHEEK_CLR)
LUG_X = (REBATE_X[0] - V.GIRDLE_RIM_W, REBATE_X[1] + V.GIRDLE_RIM_W)                   # -43.51, +18.51
LUG_Y0 = GRIP_Y0 - V.GIRDLE_LUG_D                                                       # 90.25
CHEEK_Z = (2 * _AZ - POD_TOP, POD_TOP)                                                  # 1.40, 31.32
assert CHEEK_Z[0] + 1.5 <= _AZ - D.CASE_HOLE_LAT - CSK_MOUTH_R, \
    "the cheek's bottom edge would come within 1.5 mm of its lowest countersinks"
CHEEK_PILOT_DEPTH = V.GIRDLE_CHEEK_PILOT_DEPTH
assert CHEEK_SCREW_X[0] + V.ARM_PILOT_D / 2 + 1.5 <= BAY_X[0] and \
    CHEEK_SCREW_X[1] - V.ARM_PILOT_D / 2 - 1.5 >= BAY_X[1], \
    "a cheek pilot would leave less than 1.5 mm of lug between it and the bay"
assert CHEEK_PILOT_DEPTH < V.GIRDLE_LUG_D - 1.0, "the cheek pilots would break out of the lugs' inboard face"

# neck tube -- the same footprint neck_collar.py builds, fused in here
NCX0, NCX1 = V.NECK_WELL_X                    # -35.41, +10.41
NC_HY = V.NECK_WELL_HW[0]                     # 12.66
NC_WALL = D.WALL
COLLAR_TOP = V.NECK_AXIS_Z + D.SV_TOPFACE + 1.0       # 33.10, 2.1 short of the horn disc

# deck pilots -- EVERY one of these is verified against the real torso solid
# by check_deck_pilots() below. Under the girdle the deck is cut by the battery
# aperture (x -36.51..+10.50, |y| < 54.66), so the rail pilots sit on the strip
# between it and the deck edge (61.26), at y 57, where torso_v8 hangs a deck
# edge rail under the deck so the pilot does not end over the battery layer.
PILOT_D = V.ARM_PILOT_D           # 2.05
PILOT_DEPTH = V.ARM_PILOT_DEPTH   # 4.5


def pilot_xy():
    """Every deck pilot the girdle needs, pelvis frame, both sides. Lives in
    dimensions_v6 so torso_v8 drills exactly what this part clears."""
    return list(V.GIRDLE_PILOT_XY)


# ===========================================================================
# the part
# ===========================================================================
def _pod(sgn, pads=True):
    """One shoulder pod, on the +y side then mirrored. Open UPWARD and, until
    its cheek (shoulder_cheek_v6) is screwed on, OUTBOARD: the servo goes in
    along its own axis from outboard, the one direction every feature on its
    two faces can engage from."""
    y = lambda v: sgn * v  # noqa: E731

    # --- floor under the case, from the rail out to the cheek's seat plane
    p = parts.box(POD_X[0], POD_X[1], y(RAIL_Y0), y(GRIP_Y0), 0, BASE_T)
    # --- inboard wall: locates the case, ties floor to top, carries no screws
    p += parts.box(POD_X[0], POD_X[1], y(WY0), y(WY1), 0, POD_TOP)
    # --- two end walls wrapping the case fore and aft
    p += parts.box(POD_X[0], BAY_X[0], y(WY0), y(GRIP_Y0), 0, POD_TOP)
    p += parts.box(BAY_X[1], POD_X[1], y(WY0), y(GRIP_Y0), 0, POD_TOP)
    # --- the cheek's lugs: each end wall thickened outward over its last
    # GIRDLE_LUG_D, full height, and carried out to the girdle's outer face as
    # a rim, so the cheek nests in a rebate between the two rims.
    p += parts.box(LUG_X[0], BAY_X[0], y(LUG_Y0), y(GRIP_Y1), 0, POD_TOP)
    p += parts.box(BAY_X[1], LUG_X[1], y(LUG_Y0), y(GRIP_Y1), 0, POD_TOP)
    p -= parts.box(REBATE_X[0], REBATE_X[1], y(GRIP_Y0), y(GRIP_Y1 + 1), -1, POD_TOP + 1)
    # --- the ledge the cheek sits on: the floor run out to the outer face
    # under it, GIRDLE_CHEEK_CLR short of the cheek's bottom edge
    p += parts.box(REBATE_X[0], REBATE_X[1], y(GRIP_Y0 - 0.01), y(GRIP_Y1), 0, CHEEK_Z[0] - V.GIRDLE_CHEEK_CLR)
    # --- the cheek's four pilots, along the axis into the lugs' seat faces.
    # Horizontal in print (base down), so teardropped like every other bore.
    for x, z in cheek_screw_xz():
        p -= parts.teardrop_y(PILOT_D / 2, y(GRIP_Y0 - CHEEK_PILOT_DEPTH), y(GRIP_Y0 + 0.5), x, z, roll=0)

    # --- inboard wall openings. ONE keyhole, for the same reason round 4's
    # cradle used one: cut separately, the hub hole and the connector window
    # leave a sliver of wall between them that prints as a strand.
    #   * no idler disc on this servo, but its free-hub post (2.6 proud of the
    #     case face) still turns with the shaft -> a teardrop hole on the axis
    #   * the connector TRENCH is where the servo's own lead plugs in. Lying
    #     down, that trench faces INBOARD, so the window is cut OPEN TO THE TOP
    #     of the wall: the leads leave the plugs and go straight UP out of the
    #     notch, and as the servo is pushed in from outboard they slide into
    #     the notch through its open face.
    opening = parts.teardrop_y(D.SV_IDLER_HUB_HW + V.GIRDLE_HUB_CLR, y(WY0 - 1), y(WY1 + 1), _AX, _AZ)
    # the notch runs to the axis, so its edge meets the hub hole square at its
    # top and bottom and leaves no cusp; it also gives both bus leads room.
    opening += parts.box(_AX - D.SV_CONN_L[1] - 0.6, _AX,
                         y(WY0 - 1), y(WY1 + 1), _AZ - D.SV_CONN_HW, POD_TOP + 1)
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
    # its roof rises ROOF_RISE from the back of the detent to the seat face, so
    # the 2 mm it overhangs the bay prints unsupported
    _top, _dep = _AZ + _bz, WY1 + 0.01 - _by
    p -= parts.wedge_x([(y(_by), _top - 0.01), (y(WY1 + 0.01), _top - 0.01),
                        (y(WY1 + 0.01), _top + ROOF_RISE * _dep)], _bx[0], _bx[1])
    return p + crush_pads(sgn) if pads else p


def case_screw_xz():
    """The four case screws, (x, z): rows from the axis toward the CABLE end,
    which is -x here; the lateral pair across the case width, which is z."""
    return [(_AX - xrow, _AZ + lz) for xrow in D.CASE_HOLES_TOP
            for lz in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT)]


def cheek_screw_xz():
    """The cheek's four corner screws, (x, z), into the end lugs."""
    return [(x, z) for x in CHEEK_SCREW_X for z in CHEEK_SCREW_Z]


# crush pads on the inboard wall's seat face, two on each band of bare case
# face the wall can reach: the strip between the cable end and the platform
# detent, and the strip between the hub hole and the output end. They are
# squashed ALONG the axis -- by the cheek's screws pushing the case home -- so
# they are short pads, not the long ribs a case pushed down past them needs.
_PLAT_AFT_X = _AX + D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR               # -29.91
_HUB_R = D.SV_IDLER_HUB_HW + V.GIRDLE_HUB_CLR                             # 3.54
PAD_X = ((CASE_X0 + _PLAT_AFT_X) / 2, (_AX + _HUB_R + CASE_X1) / 2)       # -32.51, +6.83
PAD_Z = (_AZ - 8.0, _AZ + 8.0)                                            # 8.36, 24.36


def crush_pads(sgn):
    """The pod's four crush pads, +y side (sgn +1) or -y: a trapezoid
    GIRDLE_PAD_W at the wall and 0.4 at the crest, which stands GIRDLE_PAD_H
    off the seat face, GIRDLE_PAD_L tall."""
    w, h, crest = V.GIRDLE_PAD_W, V.GIRDLE_PAD_H, 0.4
    y0, y1 = sgn * (WY1 - 0.3), sgn * (WY1 + h)
    out = None
    for px in PAD_X:
        for pz in PAD_Z:
            r = parts.wedge_z([(px - w / 2, y0), (px + w / 2, y0),
                               (px + crest / 2, y1), (px - crest / 2, y1)],
                              pz - V.GIRDLE_PAD_L / 2, pz + V.GIRDLE_PAD_L / 2)
            out = r if out is None else out + r
    return out


def shoulder_cheek_v6(side="L"):
    """The pod's outboard wall as its own print: THE structural wall, the
    four case screws through it into the servo's horn-side face. It goes on
    along the shoulder axis once the servo is home, over the output boss,
    into the rebate between the end lugs' rims, and four corner screws hold
    it to the lugs, its bottom edge on the floor's ledge. Pelvis frame, like
    the girdle. ONE part for both pods: it is built for the left pod and
    turned over (180 deg about the shoulder axis) for the right, which is the
    left's mirror image because everything on it is symmetric about the axis
    (check_cheek_one_part)."""
    c = _cheek_left()
    return c if side == "L" else Pos(0, 0, _AZ) * Rot(180, 0, 0) * Pos(0, 0, -_AZ) * c


def _cheek_left():
    sgn = 1
    y = lambda v: sgn * v  # noqa: E731
    c = parts.box(CHEEK_X[0], CHEEK_X[1], y(GRIP_Y0), y(GRIP_Y1), *CHEEK_Z)
    # --- relief: the O19.6 case boss stands proud of the horn face and the
    # horn disc TURNS inside it, so the plate is opened to D.GRIP_HORN_RELIEF,
    # leg_link's own number for this same joint. It is also the hole the
    # cheek is threaded over the boss through.
    c -= parts.cyl_y(D.GRIP_HORN_RELIEF, y(GRIP_Y0 - 1), y(GRIP_Y1 + 1), _AX, _AZ)
    # --- the four case screws and the four corner screws, all driven from
    # OUTBOARD; flat heads, countersunk flush (the arm's horn plate turns
    # GIRDLE_ARM_CLR off this face). Vertical in print: plain bores.
    for x, z in case_screw_xz() + cheek_screw_xz():
        c -= parts.cyl_y(D.CASE_SCREW_CLEAR / 2, y(GRIP_Y0 - 1), y(GRIP_Y1 + 1), x, z)
        c -= parts.csk_y(x, z, y(GRIP_Y1), sgn)
    # --- relief for the moulded HORN-side rib. The case has a 1.13 mm rib
    # standing proud of its horn face along most of its length; a flat plate
    # lands on the rib instead of the face and the servo sits crooked. Pocket,
    # not window: the four case screws all land outside the rib's band (rows
    # at z +-10.25 vs the rib's +-7.82), so they still bear on full plate.
    _rx = (_AX - D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR,
           _AX - D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR)
    _rz = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    c -= parts.box(_rx[0], _rx[1], y(GRIP_Y0 - 0.01),
                   y(GRIP_Y0 + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR),
                   _AZ - _rz, _AZ + _rz)
    return c


def lead_slot(sgn):
    """The elbow lead's slot (#198) through the whole aft tie band, +y side
    (sgn +1) or -y: straight sides from the beam's top to LEAD_SLOT_EAVE, then
    a peaked top rising ROOF_RISE on both sides, so it prints unsupported.
    Behind the thin wall it opens into the web's lightening recess, and above
    the recess's sloped roof it runs through the band as a short tunnel."""
    y0, y1 = LEAD_SLOT_Y
    peak = LEAD_SLOT_EAVE + ROOF_RISE * (y1 - y0) / 2
    prof = [(sgn * y0, TIE_H), (sgn * y1, TIE_H), (sgn * y1, LEAD_SLOT_EAVE),
            (sgn * (y0 + y1) / 2, peak), (sgn * y0, LEAD_SLOT_EAVE)]
    return parts.wedge_x(prof, TIE_X[0] - 1.0, TIE_X[1] + 1.0)


def shoulder_girdle_v6(pads=True):
    """The whole girdle, in the PELVIS frame (deck top z = 0, robot forward
    +x, left +y) so it reads directly against torso_v8's own geometry.
    Qty 1 -- it is symmetric, so it is NOT a mirror pair. Its two outboard
    walls are separate prints (shoulder_cheek_v6). pads=False leaves the crush
    pads off, for the insertion check.
    """
    p = _pod(+1, pads) + _pod(-1, pads)

    # --- the two rails: 4 mm plate on the deck's edge beam, carrying the
    # pod straight into the stiffest line the deck has.
    for sgn in (1, -1):
        p += parts.box(RAIL_X[0], RAIL_X[1], sgn * RAIL_Y0, sgn * WY1, 0, BASE_T)

    # --- forward clavicle beam: the transverse tie. It can only live here --
    # forward of x +10.5 is the one place the deck is solid all the way across
    # (everything aft of it is the battery aperture).
    # --- aft tie: a BRIDGE over the battery aperture, landing on both rails.
    # No pilots -- there is no deck under it. It is free stiffness: the pack
    # swaps through the torso's side door, and the yaw stator screws under the
    # aperture are driven with the girdle off.
    #
    # Both bands also carry the TRAPEZIUS WEB: a full-height panel from the
    # neck tube out to each pod. Structurally it is the top chord -- it ties
    # the two pods to each other and to the neck at the height the arm's
    # moment actually acts, instead of only at the deck. Visually it is the
    # whole point: without it the girdle reads as three separate lumps (a box,
    # a tube, a box) rather than one shoulder line running out of the neck.
    # It was fenestrated, and the 34 mm flat window tops needed supports; now
    # (Tom, 2026-11-01) the window is a WEB_T wall flush with the band's OUTER
    # face (front for the clavicle, back for the tie), with a recess behind it
    # that keeps the weight down. The recess sits above the beam, so the beam
    # stays whole, and its roof is a ROOF_RISE slope rising from the thin wall
    # to the open inner face, so nothing on it needs support. The aft tie's
    # web keeps one opening, the elbow lead's slot (lead_slot(), #198), at the
    # recess's outboard end beside each pod; its top is peaked, not flat.
    for x0, x1, h, outer in ((CLAV_X[0], CLAV_X[1], CLAV_H, +1), (TIE_X[0], TIE_X[1], TIE_H, -1)):
        p += parts.box(x0, x1, -CLAV_HY, CLAV_HY, 0, h)
        x_mem = x1 - WEB_T if outer > 0 else x0 + WEB_T     # the thin wall's inner face
        x_open = x0 if outer > 0 else x1                    # the band's inner face, where the recess opens
        d = abs(x_mem - x_open)
        x_out = x_open - outer * 1.0
        recess = [(x_mem, h), (x_out, h), (x_out, WEB_REC_TOP), (x_open, WEB_REC_TOP),
                  (x_mem, WEB_REC_TOP - ROOF_RISE * d)]
        for sgn in (1, -1):
            p += parts.box(x0, x1, sgn * (NC_HY + NC_WALL), sgn * WY1, 0, POD_TOP)
            p -= parts.wedge_y(recess, sgn * WEB_REC_Y[0], sgn * WEB_REC_Y[1])
            if outer < 0:
                p -= lead_slot(sgn)

    # --- the neck tube: the head's mount, rising BETWEEN the pods
    p += parts.box(NCX0 - NC_WALL, NCX1 + NC_WALL, -(NC_HY + NC_WALL),
                   NC_HY + NC_WALL, 0.0, COLLAR_TOP)
    # case cavity, open top (the mouth) and open bottom: the neck servo's seat
    # is neck_floor.py, a plate screwed up into four bosses on the tube's
    # side walls that drops into the deck's battery aperture at the old well
    # depth (issue #90). It is its own print because it hangs below this
    # part's print plane.
    p -= parts.box(NCX0, NCX1, -NC_HY, NC_HY, -1.0, COLLAR_TOP + 1.0)
    # no aft window (Tom, 2026-11-01): the neck's leads leave through the neck
    # floor's window toward +y and rise beside the tube (docs/assembly.md 12),
    # so the tube's aft wall is closed.
    boss_add, boss_cut = NF.tube_bosses()
    p += boss_add
    p -= boss_cut

    # --- deck pilots: clearance + countersink through whatever sits over them
    for hx, hy in pilot_xy():
        p -= parts.cyl_z(D.CASE_SCREW_CLEAR / 2, -1, BASE_T + 1, hx, hy)
        p -= parts.csk_z(hx, hy, BASE_T, +1)

    # edges stay square: rounding them buys nothing (Tom, 2026-10-24), and the
    # 45 deg flare under each pod's outboard edge went with the one-piece
    # grip plate (Tom, 2026-11-01)
    return p


# ===========================================================================
# mocks
# ===========================================================================
def shoulder_servo_mock(side="L"):
    """The shoulder STS3215 in the PELVIS frame. The canonical mock has the
    axis +Y, horn +Y, case length along Z with the output end at +10.11; here
    it is turned +90 about Y, which maps the case's +Z (output end) onto +X: the
    case LENGTH lies fore-aft with the output end FORWARD and the cable end
    AFT, and the case WIDTH (24.72) stands up. No idler disc: the inboard
    wall seats on the bare case face."""
    m = Pos(_AX, V.ARM_SERVO_MID_Y, _AZ) * Rot(0, 90, 0) * CA.servo_mock(idler_disc=False)
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
        for x, z in case_screw_xz():
            out.append(dict(name=f"shoulder_case_{side}_{_AX - x:.0f}_{z - _AZ:+.0f}", frame="pelvis",
                            kind="M2.5x8 self-tap, flat head (through the cheek into the servo case's"
                                 " HORN-side face, driven from OUTBOARD -- the inboard face is 1.24 mm"
                                 " off the skin)",
                            pos=(x, sgn * GRIP_Y1, z), axis=(0, -sgn, 0), length=8.0))
        for x, z in cheek_screw_xz():
            out.append(dict(name=f"shoulder_cheek_{side}_{x:+.0f}_{z:.0f}", frame="pelvis",
                            kind=f"M2.5x8 self-tap, flat head (cheek corner, driven from OUTBOARD into "
                                 f"a {PILOT_D:.2f} x {CHEEK_PILOT_DEPTH:.1f} pilot in the end lug)",
                            pos=(x, sgn * GRIP_Y1, z), axis=(0, -sgn, 0), length=8.0))
    return out


# the order IS the bench order; animate_v6 flies the parts in along these.
# On the bench, before the girdle goes on the robot: the neck floor is screwed
# up into the tube's bosses, and the neck servo drops into the tube onto it
# (neck_floor.INSERT) and is screwed from below -- the plate sits over the
# pack, so none of that is reachable once the girdle is on the deck. Then the
# girdle is lowered onto the deck and screwed down
# through its two rails; the rail pilots are clear of the bays, so the
# shoulder servos can go in before or after. Each shoulder servo is pushed in
# along its own axis from OUTBOARD (lead plugged in first: it rises out of the
# inboard wall's notch) until its idler face lands on the crush pads, then its
# cheek goes on along the same axis over the boss, the four corner screws push
# the case home past the pads, and the four case screws go in through the
# cheek. Vectors are the LEFT side's (the right's are mirrored in y) and point
# where the part comes FROM, as animate_v6.INSERT's do.
INSERT = {
    "shoulder_girdle_v6": (0, 0, 1),   # lowered onto the deck, screwed down through the rails
    "servo_shoulder": (0, 1, 0),       # in along the shoulder axis from outboard, cheek off
    "shoulder_cheek": (0, 1, 0),       # then the cheek, on the same axis, over the output boss
}
INSERT_D = (0.0, 0.1, 0.25, 0.5, 1.0, 1.5, 2.0, 2.6, 3.0, 4.0, 6.0, 10.0, 20.0, 40.0)  # mm back along the path


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
    """Every deck pilot must be fully BURIED in the torso's deck: 4.5 mm into
    a 5 mm plate leaves 0.5 mm of floor (torso_v8's deck edge rails add more).
    Checked against the REAL torso solid, both sides -- not against a drawing
    of where the cuts are: the rail sits at y 57, between the battery aperture
    and the deck edge, and this is what proves the strip is there.
    """
    import torso_v8 as TV
    solid = TV.torso_v8()
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


def pad_crush_mm3(side="L", back=0.0):
    """The designed overlap of a shoulder servo with its pod's crush pads,
    `back` mm short of home along its path: the volume the pads give up as
    the cheek's screws push the case onto them."""
    sgn = 1 if side == "L" else -1
    return (Pos(0, sgn * back, 0) * shoulder_servo_mock(side) & crush_pads(sgn)).volume


def _pod_crop(solid, sgn):
    """`solid` cut down to one pod's neighbourhood, so the sweeps intersect a
    small solid instead of the whole girdle."""
    return solid & parts.box(LUG_X[0] - 5, LUG_X[1] + 5, sgn * 50.0, sgn * (GRIP_Y1 + 60), -5, POD_TOP + 60)


def check_servo_fit(verbose=True):
    """The two shoulder servo mocks must sit INSIDE their bays touching the
    girdle only through the inboard wall's crush pads, by their designed
    interference (and they must touch: a pad that misses the case leaves the
    idler side loose); the cheek must seat on the case's horn face and in its
    rebate without overlapping either; the girdle must not reach into the
    plane the rotating arm plate occupies."""
    p = shoulder_girdle_v6()
    ok = True
    for side in ("L", "R"):
        m = shoulder_servo_mock(side)
        c = shoulder_cheek_v6(side)
        try:
            overlap = (m & p).volume
            pads = pad_crush_mm3(side)
            cheek_servo = (c & m).volume
            cheek_girdle = (c & p).volume
        except Exception:  # noqa: BLE001
            overlap = pads = cheek_servo = cheek_girdle = float("nan")
        good = 0.0 < pads and overlap <= pads + 0.5
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  shoulder servo {side} vs girdle: "
                  f"{overlap:7.2f} mm3 of overlap, all of it the crush pads ({pads:.2f} mm3, "
                  f"{V.GIRDLE_PAD_CRUSH:.2f} mm deep)")
        for what, v in (("servo", cheek_servo), ("girdle", cheek_girdle)):
            good = v < 0.5
            ok &= good
            if verbose:
                print(f"  {'PASS' if good else 'FAIL'}  cheek {side} vs {what}: {v:7.2f} mm3 of overlap")
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


def _sweep(moving, static, vec):
    """[(d, overlap mm3)] for `moving` backed off d mm along unit `vec`, for
    every INSERT_D, against `static`."""
    out = []
    for d in INSERT_D:
        try:
            v = (Pos(*(np.asarray(vec, float) * d)) * moving & static).volume
        except Exception:  # noqa: BLE001
            v = float("nan")
        out.append((d, v))
    return out


def check_insertion(verbose=True):
    """Tom, 2026-11-01: the servo has to be able to get INTO its bay. Each
    piece is backed off along its INSERT path, every INSERT_D, and must touch
    nothing already there: the servo against the girdle (cheek off; the crush
    pads are allowed their designed bite in the last GIRDLE_PAD_CRUSH and
    nowhere else), then the cheek against the girdle and the seated servo.
    The old path -- the servo dropped in from above into the one-piece pod,
    i.e. girdle + cheek -- is measured too, and must be BLOCKED: that is the
    defect this layout fixes, and the check says so if it ever stops being
    true (a mock that lost its boss, say)."""
    ok = True
    g_bare, g_pads = shoulder_girdle_v6(pads=False), shoulder_girdle_v6()
    for side, sgn in (("L", 1), ("R", -1)):
        m, c = shoulder_servo_mock(side), shoulder_cheek_v6(side)
        out = (0, sgn, 0)
        bare = _pod_crop(g_bare, sgn)
        steps = [
            (f"servo {side} in from outboard, vs the girdle (no pads)", m, bare, 0.05),
            (f"cheek {side} on over the boss, vs the girdle", c, _pod_crop(g_pads, sgn), 0.05),
            (f"cheek {side} on over the boss, vs the seated servo", c, m, 0.05),
        ]
        for label, mov, st, tol in steps:
            sw = _sweep(mov, st, out)
            worst = max(sw, key=lambda t: t[1])
            good = worst[1] <= tol
            ok &= good
            if verbose:
                print(f"  {'PASS' if good else 'FAIL'}  {label:52s} worst {worst[1]:6.2f} mm3 "
                      f"(at {worst[0]:g} mm back)")
        # the pads: their bite only in the last GIRDLE_PAD_CRUSH, and none before
        bites = [(d, pad_crush_mm3(side, d)) for d in INSERT_D]
        early = [b for d, b in bites if d >= V.GIRDLE_PAD_CRUSH + 0.01]
        good = bites[0][1] > 0 and max(early) <= 0.05
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  servo {side} vs its crush pads: {bites[0][1]:.2f} mm3 "
                  f"home, {max(early):.2f} mm3 once {V.GIRDLE_PAD_CRUSH + 0.01:.2f} mm back")
        # approach clearance off the floor it slides on: end walls and inboard wall
        try:
            walls = bare - parts.box(POD_X[0] - 1, POD_X[1] + 1, sgn * RAIL_Y0, sgn * GRIP_Y1, -1, BASE_T)
            clr = (Pos(0, sgn * 1.0, 0) * m).distance_to(walls)
        except Exception:  # noqa: BLE001
            clr = float("nan")
        if verbose:
            print(f"        servo {side} 1 mm short of home, sliding on the floor: {clr:.2f} mm "
                  f"from the walls")
        # the old path, for the record
        one_piece = _pod_crop(g_pads, sgn) + c
        drop = dict(_sweep(m, one_piece, (0, 0, 1)))
        good = drop[3.0] > 1.0
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  the OLD path, servo {side} dropped from above into the "
                  f"one-piece pod: {drop[3.0]:.0f} mm3 in the way 3 mm up, {drop[10.0]:.0f} mm3 at 10 mm "
                  f"(blocked, as built)")
    return ok


def check_cheek_lugs(verbose=True):
    """Every cheek pilot must be BURIED in its lug: a 1.2 mm wall of girdle
    round its whole depth (the teardrop's 8% overshoot excluded), and every
    cheek screw's countersink inside the cheek's own face."""
    p = shoulder_girdle_v6()
    ok = True
    for side, sgn in (("L", 1), ("R", -1)):
        c = shoulder_cheek_v6(side)
        for x, z in cheek_screw_xz():
            ring = (parts.cyl_y(PILOT_D / 2 + 1.2, sgn * (GRIP_Y0 - CHEEK_PILOT_DEPTH), sgn * GRIP_Y0, x, z)
                    - parts.cyl_y(PILOT_D / 2 * 1.1, sgn * (GRIP_Y0 - CHEEK_PILOT_DEPTH - 1),
                                  sgn * (GRIP_Y0 + 1), x, z))
            mouth = parts.cyl_y(CSK_MOUTH_R + 0.5, sgn * GRIP_Y0, sgn * GRIP_Y1, x, z)
            try:
                stray = (ring - p).volume
                rim = (mouth - parts.box(CHEEK_X[0], CHEEK_X[1], sgn * GRIP_Y0, sgn * GRIP_Y1, 0, POD_TOP)).volume
            except Exception:  # noqa: BLE001
                stray = rim = float("nan")
            good = stray < 0.5 and rim < 0.05
            ok &= good
            if verbose:
                print(f"  {'PASS' if good else 'FAIL'}  cheek {side} screw ({x:+6.2f}, z {z:5.2f}): "
                      f"{stray:5.2f} mm3 of its 1.2 mm pilot wall outside the lug, countersink "
                      f"{'inside' if rim < 0.05 else 'OFF'} the cheek")
    return ok


def check_cheek_one_part(verbose=True):
    """The right pod is the left's mirror image, so its cheek must be the
    left cheek's mirror -- and the right cheek is BUILT as the left one turned
    over. They agree only if everything on the cheek is symmetric about the
    shoulder axis, which is what makes it one part printed twice."""
    left, right = shoulder_cheek_v6("L"), shoulder_cheek_v6("R")
    want = mirror(left, Plane.XZ)
    try:
        diff = (want - right).volume + (right - want).volume
    except Exception:  # noqa: BLE001
        diff = float("nan")
    good = diff < 0.05
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  left cheek mirrored vs left cheek turned over: "
              f"{diff:.3f} mm3 apart")
    return good


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
    CP.STL = OUT_STL
    fails = 0
    for name, (rot, note) in PRINT_ORIENT.items():
        CP.ORIENT[name] = (rot, note)
        CP.PRINT_STL[name] = f"{name}.stl"
        if SUPPORT_NOTE and name == "shoulder_girdle_v6":
            CP.SUPPORTED[name] = SUPPORT_NOTE
        fails += bool(CP.audit(name, verbose=verbose))
    return not fails


def run_checks():
    print("\n== printability (girdle base down, cheeks flat) ==")
    ok = check_print()
    print("\n== driver access to every deck pilot ==")
    ok &= check_pilot_access()
    print("\n== deck pilots (against the real torso solid) ==")
    ok &= check_deck_pilots()
    print("\n== servo fit / print envelope ==")
    ok &= check_servo_fit()
    print("\n== insertion paths: servo in along its axis, then the cheek ==")
    ok &= check_insertion()
    print("\n== cheek screws: pilots buried in the lugs ==")
    ok &= check_cheek_lugs()
    print("\n== one cheek part for both pods ==")
    ok &= check_cheek_one_part()
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
    c = shoulder_cheek_v6("L")      # the one cheek part, in the left pod's pose; x2
    bb = c.bounding_box()
    print(f"shoulder_cheek_v6    {bb.size.X:6.2f} x {bb.size.Y:6.2f} x {bb.size.Z:6.2f} mm    "
          f"{mass_g(c):5.1f} g   x2 (both pods)")
    export_stl(c, os.path.join(OUT_STL, "shoulder_cheek_v6.stl"))
    export_step(c, os.path.join(OUT_STEP, "shoulder_cheek_v6.step"))
    print("wrote stl + step")
    if a.checks:
        ok = run_checks()
        print("\nALL CHECKS PASS" if ok else "\n** CHECKS FAILED **")


if __name__ == "__main__":
    main()
