"""arm_v6: the two 2-DOF (shoulder pitch + elbow) arms the get-up study chose.

WHY THESE ARMS EXIST, and why every number is somebody else's
-------------------------------------------------------------
`docs/design-v6/getup-decision-2026-09-17.md` closed a three-day search: the
v7 body cannot stand up from supine with its legs alone (~150 hand-built
sequences + continuous searches, 0 stood), and of everything tried only a
pusher that can reach the floor BEHIND the hips works. The winner is
`top_elbow_16_16_aft`: two arms, shoulder pitch + elbow, 160 mm upper arm +
160 mm forearm, shoulder at the top of the torso 50 mm AFT of the torso
origin, hanging straight down at the sides when idle. 0.32 m end-to-end is a
measured THRESHOLD (0.30 never stands; 0.32 does), the aft mount is the
measured fix for the hanging arm walking into its own leg (7327 -> 31 contacts
over an 8-step walk), and 2 DOF is measured too (a third ab/adduction DOF was
0/468). So: this module DRAWS that design. It does not re-derive it. Where the
CAD had to deviate -- shoulder 8.9 mm higher, arm plane 7.7 mm further out --
dimensions_v6.py says so by name next to the constant, and the design note
(docs/design-v6/arms.md) lists both as sign-off items.

WHAT IS HERE (2 designs, 4 prints -- arms are mirror pairs, like the feet)
    (the shoulder MOUNT is not here any more. Round 5 -- Tom, 2026-09-19:
     "rotate the servo bodies 90deg and incorporate them into the torso" --
     replaced the per-side deck bracket with ONE part that carries both
     shoulder servos AND the neck: cad/v6/shoulder_girdle_v6.py. The arm
     LINKS below are unchanged by that: the shoulder servo was rotated about
     its OWN output axis, so the horn is in exactly the same place.)
    arm_upper_v6(side)       shoulder horn -> elbow. Single-sided horn plate at
                             the top (the hip YAW joint's proven grip: the
                             shoulder servo's idler face looks inboard over the
                             deck, so a clevis there is not drawable), a fork
                             at the bottom straddling the elbow servo's horn
                             AND idler discs.
    arm_fore_v6(side)        elbow -> hand. Grips the elbow servo's case in a
                             leg_link-style channel and runs 160 mm to a 12 mm
                             knuckle -- the plant's hand sphere, same radius.

PRINT ORIENTATION -- the one thing not copied from leg_link_v6
--------------------------------------------------------------
Both arm links print ON THEIR BACK (model +X up, check_printability's RY_XUP),
NOT standing on the fork end like leg_link_v6. This is deliberate and it is
the whole reason the sections below are open C's rather than closed boxes.

Read the block above `yoke_roll` in cad/parts.py: both hip clevises used to
print flange-down, arms rising as vertical columns, which lays the layer lines
ACROSS the arm -- and a cantilevered arm carries its bending load as tension
ALONG the arm, straight through the interlayer bond. They kept snapping
mid-arm. The control case cited there is v5's leg_link, which "prints web-down,
so the filament runs the length of the leg", far longer arm, thinner mid-span,
never broken. These arms are 160 mm cantilevers that the robot pushes its own
mass up on; they get the control case's orientation. Laid on their back, the
joint-to-joint axis (model Z) and the straddle direction (model Y) are BOTH
bed-plane axes, so the bending tension runs along the filament and -- a free
bonus -- any profile drawn in the (y,z) plane extrudes along X as plain
vertical walls. The only thing that is a real overhang in this orientation is
a bore along Y, and those are teardropped with roll = ROLL_UP (90, peak toward
model +x = print up), exactly as v5's leg_link does.

What it costs: an open C section (back web + two side rails, opening forward)
instead of leg_link_v6's closed box, because a front plate would be a flat
ceiling spanning tine to tine in this orientation -- the exact finding that
made leg_link_v6 stand up instead. The C is weaker in TORSION, not in the
bending that matters, and peak measured joint torque here is 1.59 N-m
(shoulder) / 1.18 (elbow) against the STS3215's 2.72 N-m simulated stall.

    .venv/bin/python cad/v6/arm_v6.py          # STL + STEP + audits + renders
    .venv/bin/python cad/v6/arm_v6.py --only arm_upper_v6_L
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")

from build123d import (Cone, Plane, Pos, Rot, export_step, export_stl,  # noqa: E402
                       fillet, mirror)
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402  (v5 helpers: box/cyl_*/teardrop_*/wedge_*/csk_*/bcd_*)
import check_assembly as CA  # noqa: E402  (the canonical servo mock)

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientations, in check_printability's convention (rows = print axes in
# model coords). RY_XUP == v5 leg_link's "on its back: web face on bed".
RY_XUP = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)
IDENT = np.eye(3)
PRINT_ORIENT = {
    "arm_upper_v6": (RY_XUP, "on its back: web face on bed, arm length in the bed plane"),
    "arm_fore_v6": (RY_XUP, "on its back: web face on bed, arm length in the bed plane"),
}
# Parts printed WITH OrcaSlicer supports (check_printability waives their
# overhang findings; cad/PRINT_LIST.md's "supports" column says the
# same). Support work is the slicer's job wherever it can do it -- only what a
# slicer cannot clean up (small horizontal bores: teardropped) is designed in.
SUPPORT_NOTE = {
    "arm_upper_v6": "supports on: the two elbow pads' undersides start 5 mm off the bed "
                    "(the fork prints on its back); support touching the build plate only",
}
# print-up for this orientation is model +X, so every horizontal bore's
# teardrop peak points that way (teardrop_*'s roll=90). Named, not a bare 90,
# for the same reason leg_link_v6 names its ROLL_UP=0.
ROLL_UP = 90

# derived, local to this module (everything dimensional lives in dimensions_v6)
_AX = V.ARM_SHOULDER_X                                   # 0.0 -- the hip plane
_AZ = V.ARM_AXIS_Z                                       # 16.36 above the deck top


def _mirrored(solid, side):
    """The right-hand part. The arms ARE a mirror pair: the shoulder horn has
    to face outboard on both sides, which is the one thing a translation (the
    trick the legs use -- "legs are translations, not mirrors") cannot do."""
    if side == "L":
        return solid
    return mirror(solid, Plane.XZ)


# ===========================================================================
# 2. arm_upper_v6 -- shoulder horn to elbow fork
# ===========================================================================
def arm_upper_v6(side="L"):
    """Upper arm. Local frame: the SHOULDER axis is the Y axis at the origin,
    local y = 0 is the arm plane (== the elbow servo's mid-plane, so the arm
    is straight, no jog between the two joints); +x robot forward, the arm
    hangs to -z, the elbow axis is at z = -V.ARM_UPPER. +y is OUTBOARD.
    Qty 2 (mirror pair). Print: on its back, RY_XUP (see the module docstring).

    Top end is SINGLE-SIDED on the shoulder horn -- the horn plate + 1.0
    seating boss + the 4x M3 disc bolt circle, exactly yoke_roll's horn arm.
    But it is no longer a bare plate: the arm's BOX SECTION runs all the way to
    the shoulder (inboard flange + aft web + outboard flange, opening forward),
    with one access bore through the outboard flange for the disc screws. As a
    flat 3 mm plate this head had Z = 41 mm3 about X and a 20 N knock at the
    hand put ~157 MPa in it; as the box it is 583 mm3 and ~11 MPa. There is no
    idler-side arm and there cannot be one: the shoulder servo's idler face
    looks INBOARD, straight over the deck, so a second tine would have to wrap
    round the case and sweep the deck and the head every time the arm folds up.
    The precedent for carrying a real load on one disc is the hip YAW joint,
    which hangs the entire leg (and half the robot) off one horn.

    Bottom end is a FORK straddling the elbow servo's horn AND idler discs,
    which is what keeps the forearm in the same plane as the upper arm; going
    single-sided at the elbow instead would push the forearm 15-18 mm INBOARD,
    straight at the thigh. The fork is the widest thing on the robot after the
    feet (y +23.45 local) and its inboard tine is the closest thing to the leg
    (y -20.40); check_assembly_v6's arm rows are where that gets measured, not
    argued.
    """
    wx0, wx1 = V.ARM_WEB_X                    # -15.16, -12.76
    fx = V.ARM_FRONT_X                        # 12.0
    sy0, sy1 = V.ARM_SHAFT_Y                  # -3.5, +10.5
    rt = V.ARM_RAIL_T                         # 2.8
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE           # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE     # -17.40, -20.40
    drop = -V.ARM_UPPER                       # -160, the elbow axis
    jz0, jz1 = V.ARM_JOG_Z                    # -108, -136
    boss_y0, boss_y1 = -D.PLATE / 2 - D.HORN_BOSS_H, -D.PLATE / 2  # -2.5, -1.5
    py1 = D.PLATE / 2                                              # +1.5
    head_top = 12.0                           # plate reaches above the axis to
                                              # cover the upper bolt-circle pair
    shaft_top = -24.0
    trans_z = -40.0                           # head plate -> full shaft section

    # --- head: a CLOSED box, not a plate. The arm's section runs all the way
    # to the shoulder (Tom, 2026-09-21) and is closed at the front (Tom,
    # 2026-09-24: "connecting the inner and outer face in front"): inboard
    # flange + aft web + outboard flange + front wall. Its only opening is the
    # screw-access bore on the side facing AWAY from the body.
    #   inboard flange == the horn plate: still D.PLATE thick, still bearing on
    #   the horn at boss_y1, so the four disc screws and their stack are
    #   untouched and ARM_Y is untouched. The head runs forward to
    #   ARM_HEAD_FRONT_X so the front wall clears the forward screw head.
    hfx = V.ARM_HEAD_FRONT_X                  # 13.4
    p = parts.box(wx0, hfx, boss_y1, py1, trans_z, head_top)
    p += parts.cyl_y(D.HORN_BOSS_D / 2, boss_y0, boss_y1, 0, 0)
    #   aft web and outboard flange, carried up from the shaft at full depth
    p += parts.box(wx0, wx1, sy0, sy1, trans_z, head_top)
    p += parts.box(wx0, hfx, sy1 - rt, sy1, trans_z, head_top)
    #   THE OPENING, away from the body: one bore through the outboard flange
    #   on the shoulder axis to drop the four M3s in. Teardropped (horizontal
    #   bore in this print); its roof is then cut off by the front wall below,
    #   which leaves a flat bridge ARM_HEAD_ACCESS_R is sized to keep < 8 mm.
    p -= parts.teardrop_y(V.ARM_HEAD_ACCESS_R, sy1 - rt - 1, sy1 + 1, 0, 0, roll=ROLL_UP)
    #   the FRONT WALL, added after the bore so the bore never cuts it: joins
    #   the inboard and outboard flanges and closes the box. In print it is a
    #   bridge between the two flanges, not a ledge.
    p += parts.box(V.ARM_HEAD_WALL_IN_X, hfx, boss_y1, sy1, trans_z, head_top)
    # knock the two top corners off (no sharp external corners; also keeps the
    # plate from reaching further round the servo than it needs to)
    # The two chamfers are NOT the same shape, and that is a print result. In
    # RY_XUP the bed is model -x, so the AFT corner's chamfer is a DOWN-facing
    # slope starting at the first layer: at the old 7.0 x 7.5 it was 47 deg and
    # the audit called it a LEDGE the moment the box gave it the outboard
    # flange to cut through as well. 8.0 x 6.0 is 37 deg -- self-supporting.
    # The forward corner faces UP and is free, so it keeps the tighter shape.
    for cx in (wx0, hfx):
        s = 1 if cx > 0 else -1
        dx, dz = (8.0, 6.0) if cx < 0 else (6.5, 7.0)
        p -= parts.wedge_y([(cx + s * 0.5, head_top + 0.5),
                            (cx + s * 0.5, head_top - dz),
                            (cx - s * dx, head_top + 0.5)],
                           boss_y1 - 1, sy1 + 1)

    # --- no transition any more. The old solid wedge existed to flare a 3 mm
    # plate out to the C section over 16 mm; with the box run to the shoulder
    # there is nothing to flare, and a hollow section of the same envelope is
    # both stiffer and lighter than the solid one it replaces.

    # --- shaft: open C, back web + two rails, opening forward. The rails taper
    # from ARM_FRONT_X at the shoulder (where the bending moment is the hand
    # force x 320 mm) to ARM_TIP_X at the elbow (x 160 mm) -- the section
    # follows the moment. Free in print: x is the print HEIGHT in RY_XUP, so a
    # shrinking x-extent is an up-facing slope, never an overhang.
    tip = V.ARM_TIP_X
    p += parts.box(wx0, wx1, sy0, sy1, jz0, trans_z)
    for a0, a1 in ((sy0, sy0 + rt), (sy1 - rt, sy1)):
        p += parts.wedge_y([(wx0, trans_z), (fx, trans_z), (tip, jz0), (wx0, jz0)], a0, a1)

    # --- jog: the C widens into the fork. Web full width, rails follow the
    # outer edges (slanted, so their perpendicular thickness is rt*cos(~30) =
    # 2.4 -- still three perimeters, and well clear of the THIN gate).
    p += parts.wedge_x([(sy0, jz0), (iy0, jz1), (hy1, jz1), (sy1, jz0)], wx0, wx1)
    p += parts.wedge_x([(sy0, jz0), (sy0 + rt, jz0), (iy0 + rt, jz1), (iy0, jz1)], wx0, fx)
    p += parts.wedge_x([(sy1, jz0), (sy1 - rt, jz0), (hy1 - rt, jz1), (hy1, jz1)], wx0, fx)
    # the jog's rails re-grow the shaft's tapered depth back to ARM_FRONT_X
    p -= parts.wedge_y([(tip + 0.01, jz0), (fx + 1, jz0), (fx + 1, jz0 - 8.0)], iy0 - 1, hy1 + 1)

    # --- fork: two tines onto the elbow servo's discs, joined by the web for
    # as long as the elbow servo's own SWEEP allows, then two independent tines
    # (exactly as the leg's fork is below LL_WEB_END).
    #
    # WEB_END IS NOT A STYLE CHOICE. The elbow servo's case turns WITH the
    # forearm about the elbow axis, and its nearest corner sits at
    # hypot(SV_WID/2, SV_AXIS_FROM_OUT_END) = 15.97 mm -- leg_link's "r 16
    # rule", the same number check_r16 guards there. The web spans the full
    # tine-to-tine width at x -15.16..-12.76, i.e. straight through that circle,
    # so it has to stop before it. First cut ended it at drop + 2 and
    # check_assembly_v6's `servo_elbow vs arm_upper` row caught it immediately:
    # 174 mm3 of overlap at elbow -27 deg. The web now clears r 16 by
    # V.ARM_R16_BUFFER at its innermost corner (x = ARM_WEB_X[1]).
    _r16 = math.hypot(D.SV_WID / 2, D.SV_AXIS_FROM_OUT_END) + V.ARM_R16_BUFFER
    web_end = drop + math.sqrt(_r16 ** 2 - wx1 ** 2)
    p += parts.box(wx0, wx1, iy0, hy1, web_end, jz1)
    # NO modelled pad support. Printed on its back, the bed is model -x and the
    # pad's lowest point sits 5.16 mm above it facing straight down -- that is
    # OrcaSlicer's job, not this part's (Tom, 2026-09-24: "leave the print
    # support work to OrcaSlicer as long as it is capable of handling it").
    # The rectangular slab that used to sit under each pad stuck out past it,
    # and was also the thing the forearm hit at elbow +20 deg; without it the
    # CAD elbow range opens to +60. SUPPORT_NOTE below registers the part as
    # printed with slicer supports, so the audit waives the pad undersides.
    for a0, a1 in ((iy0, iy1), (hy0, hy1)):
        p += parts.box(wx0, fx, a0, a1, drop, jz1)
        p += parts.cyl_y(D.PAD_D / 2, a0, a1, 0, drop)
    # idler boss, OD tapered 45 deg so its print-underside band never exceeds 45
    _ibh = abs(D.IDLER_BOSS_H)
    p += Pos(0, (D.SV_IDLER_FACE + iy1) / 2, drop) * Rot(90, 0, 0) * Cone(
        D.IDLER_BOSS_D / 2 - _ibh, D.IDLER_BOSS_D / 2, _ibh)

    # --- holes: the elbow disc bolt circle straight through both tines, and
    # the two centre reliefs (horn screw head / idler free-hub post)
    for h in parts.bcd_y(iy0 - 1, hy1 + 1, 0, drop, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, drop)
    p -= parts.cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
                     D.SV_IDLER_FACE + 0.7, 0, drop)
    # --- holes: the shoulder disc bolt circle through the horn plate
    # (only through the INBOARD flange: the access bore already opened the
    # outboard flange over the whole bolt circle)
    for h in parts.bcd_y(boss_y0 - 1, py1 + 1, 0, 0, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, boss_y0 - 1, py1 + 1, 0, 0)

    # --- cable window through the shaft web: the elbow servo's lead runs up
    # the inside of the C to the deck, and wants a hole to change sides at.
    p -= parts.box(wx0 - 1, wx1 + 1, -4.5, 4.5, -96.0, -78.0)

    # --- trim anything that ran below the elbow axis outside the pad radius
    _below = parts.box(-40, 40, -40, 40, drop - 60, drop)
    p -= _below - parts.cyl_y(D.PAD_D / 2, -40, 40, 0, drop)
    return _mirrored(p, side)


# ===========================================================================
# 3. arm_fore_v6 -- elbow grip to hand
# ===========================================================================
def arm_fore_v6(side="L"):
    """Forearm. Local frame: the ELBOW axis is the Y axis at the origin and is
    the gripped servo's own axis (its case hangs down the forearm, output end
    +10.11 above the axis, cable end -35.11); +x forward, +y is the horn side
    (== outboard). Hand centre at z = -V.ARM_FORE. Qty 2 (mirror pair).
    Print: on its back, RY_XUP.

    The grip channel is leg_link's, unchanged in y and z -- same servo, same
    two case faces, same 2.4/3.0 plates, same GRIP_HORN_RELIEF for the horn
    disc and the same GRIP_TOP_IDLER that keeps the idler plate clear of the
    idler disc. What is NOT copied is leg_link_v6's arched web and its front
    plate: printed on its back, the web's z extent lies in the BED PLANE, so a
    plain rectangular web has no overhang to arch away from.

    The hand is a plain PETG knuckle at V.ARM_HAND_R -- the same 12 mm radius
    the plant contacts the floor with. The plant gave it friction 1.0; bare
    PETG is nearer 0.3-0.4, which is inside the study's own robustness sweep
    (mu 0.3 / 0.7 / 1.0, 6/6 standing), so a rubber cap is an improvement, not
    a prerequisite. It is listed as an open item in the design note.
    """
    t = D.GRIP_PLATE_T                            # 2.4
    wx0, wx1 = V.ARM_WEB_X
    fx = V.ARM_FRONT_X
    rt = V.ARM_RAIL_T
    grip_x1 = 13.2                                # leg_link's grip plate front edge
    hy = V.ARM_FORE_SHAFT_HY                      # 8.0
    drop = -V.ARM_FORE                            # -160, the hand centre
    r = V.ARM_HAND_R                              # 12
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR      # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                 # -17.90
    grip_bot = D.GRIP_BOT                                    # -36.0
    tip = V.ARM_TIP_X
    taper_z = -52.0

    # --- grip channel on the elbow servo's case (leg_link's, y/z unchanged)
    p = parts.box(wx0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t,
                  grip_bot, D.GRIP_TOP_HORN)
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    p += parts.box(wx0, grip_x1, _igo, idler_seat, grip_bot, D.GRIP_TOP_IDLER)
    p += parts.box(wx0, wx1, _igo, D.SV_TOPFACE + t, grip_bot, D.WEB_TOP)

    # --- detents for the two moulded features on the case (leg_link's, byte
    # for byte: the horn-side rib and the idler-side back-cover platform)
    p -= parts.box(-D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR, D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR,
                   D.SV_TOPFACE - 0.01, D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
                   -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR, D.GRIP_TOP_HORN + 0.01)
    p -= parts.box(-D.SV_IDLER_BOSS_HW - D.RIB_RELIEF_CLR, D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR,
                   D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR, idler_seat + 0.01,
                   D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR, D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR)
    # ...and the 45 deg ramps off each detent's x side (leg_link's `_rr`/`_irr`
    # wedges). Printed on its back, a detent's far x wall is a flat DOWN-facing
    # strip 30 mm long -- a 2.4 mm-wide ceiling the audit reads as a BEAM. The
    # ramp turns it into a self-supporting slope. Without them this part fails
    # check_printability; they are not cosmetic.
    _rx = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    _ry0, _ry1 = D.SV_TOPFACE - 0.01, D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR
    _rr = 1.2 * (_ry1 - _ry0)
    # ONE side only, the +x one -- printed on its back that is the pocket's
    # DOWN-facing wall; the -x wall faces up and needs nothing. Ramping both
    # (tried) doubles the knife-edge where the ramp grazes the case-screw bore
    # at x 10.25, which is the 2.5 mm2 thin-note leg_link_v6 already carries.
    p -= parts.wedge_z([(_rx, _ry0), (_rx, _ry1), (_rx + _rr, _ry0)],
                       -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR - 0.6,
                       -D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR + 0.6)
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0, _iy1 = idler_seat, D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _irr = 1.2 * (_iy0 - _iy1)
    p -= parts.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                       D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR - 0.6,
                       D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR + 0.6)

    # --- taper from the grip channel down to the shaft, then the shaft itself
    p += parts.wedge_x([(_igo, grip_bot), (-hy, taper_z), (hy, taper_z),
                        (D.SV_TOPFACE + t, grip_bot)], wx0, wx1)
    p += parts.wedge_x([(_igo, grip_bot), (_igo + rt, grip_bot),
                        (-hy + rt, taper_z), (-hy, taper_z)], wx0, fx)
    p += parts.wedge_x([(D.SV_TOPFACE + t, grip_bot), (D.SV_TOPFACE + t - rt, grip_bot),
                        (hy - rt, taper_z), (hy, taper_z)], wx0, fx)
    p += parts.box(wx0, wx1, -hy, hy, drop, taper_z)
    for a0, a1 in ((-hy, -hy + rt), (hy - rt, hy)):
        p += parts.wedge_y([(wx0, taper_z), (fx, taper_z), (tip, drop), (wx0, drop)], a0, a1)

    # --- the hand: a knuckle at the plant's own contact radius (its hand is a
    # 12 mm sphere), PRISMATIC across y so the only down-facing geometry is in
    # the (x, z) profile, and with the back quarter of that circle replaced by
    # two 1.3:1 ramps down to the web plane. A plain sphere or a plain cylinder
    # leaves its back quadrant -- 17 x 16 mm of it -- hanging over the web with
    # nothing to print onto; the ramps land it on the bed instead. In the
    # SAGITTAL plane, the only plane this arm ever pushes in, the contact arc
    # is still exactly the plant's r 12 circle.
    q = r / math.sqrt(2.0)                    # the circle's +-45 deg back points
    p += parts.cyl_y(r, -hy, hy, 0, drop)
    _ramp = (-q - wx0) / 1.3                  # 1.3:1, comfortably inside 45 deg
    p += parts.wedge_y([(wx0, drop + q - _ramp), (-q, drop + q),
                        (-q, drop - q), (wx0, drop - q + _ramp)], -hy, hy)

    # --- holes: the case grip screws (horn-side rows both, idler-side lower
    # row only -- the upper idler row is under the back-cover platform)
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                                  D.SV_TOPFACE + t + 1, lx, -zrow, roll=ROLL_UP)
            p -= parts.csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, _igo - 1, idler_seat + 1,
                              lx, -D.CASE_HOLES_BOT[1], roll=ROLL_UP)
        p -= parts.csk_y(lx, -D.CASE_HOLES_BOT[1], _igo, -1)

    # --- cable window through the web (the elbow servo's lead leaves here)
    p -= parts.box(wx0 - 1, wx1 + 1, -4.5, 4.5, -72.0, -60.0)
    return _mirrored(p, side)


def elbow_servo_mock(side="L"):
    """The elbow STS3215 in the FOREARM's local frame: the canonical mock
    unrotated (axis +Y, horn +Y, case hanging to -z), exactly how leg_link
    grips the knee servo."""
    return _mirrored(CA.servo_mock(), side)


# ===========================================================================
# fasteners + insertion paths (the same contract leg_link_v6.SCREWS keeps)
# ===========================================================================
def SCREWS():
    """Every fastener the arm pair adds, per side, with the frame each one is
    quoted in. `axis` points from the head toward the tip."""
    s = []
    # the shoulder's own fasteners (deck pilots + the four case screws) moved
    # to shoulder_girdle_v6.SCREWS() with the mount itself.
    for i in range(4):
        ang = math.radians(90 * i)
        s.append(dict(name=f"shoulder_horn_{i}", frame="arm_upper",
                      kind="M3x6 machine screw into the horn disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.PLATE / 2, D.BCD / 2 * math.cos(ang)),
                      axis=(0, -1, 0), length=6.0))
        s.append(dict(name=f"elbow_horn_{i}", frame="arm_upper",
                      kind="M3x6 machine screw into the elbow horn disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.SV_HORN_FACE + D.PLATE,
                           -V.ARM_UPPER + D.BCD / 2 * math.cos(ang)),
                      axis=(0, -1, 0), length=6.0))
        s.append(dict(name=f"elbow_idler_{i}", frame="arm_upper",
                      kind="M3x10 machine screw + washer into the elbow idler disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.IDLER_ARM_INNER - D.PLATE,
                           -V.ARM_UPPER + D.BCD / 2 * math.cos(ang)),
                      axis=(0, 1, 0), length=10.0))
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append(dict(name=f"elbow_grip_horn_{zrow:.0f}_{lx:+.0f}", frame="arm_fore",
                          kind="M2.5x8 self-tap, flat head (into the servo case)",
                          pos=(lx, D.SV_TOPFACE + D.GRIP_PLATE_T, -zrow),
                          axis=(0, -1, 0), length=8.0))
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append(dict(name=f"elbow_grip_idler_{lx:+.0f}", frame="arm_fore",
                      kind="M2.5x8 self-tap, flat head (into the servo case)",
                      pos=(lx, D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER,
                           -D.CASE_HOLES_BOT[1]), axis=(0, 1, 0), length=8.0))
    return s


# where each piece is offered from, on the bench (fly-in / animate_v6)
INSERT = {
    "servo_shoulder": (0, 0, 1),     # dropped into the open cradle from above
    "arm_upper": (0, 1, 0),          # offered straight in onto the horn, outboard->in
    "servo_elbow": (1, 0, 0),        # slid into the forearm's grip channel from the front
    "arm_fore": (0, 0, -1),          # lifted up between the fork tines onto the discs
}


# ===========================================================================
# audits
# ===========================================================================
def check_elbow_rom(samples=41, verbose=True):
    """The elbow's real limit: sweep the forearm AND the elbow servo it carries
    against the upper arm's fork, and report the widest band that clears
    D.SWEEP_BUFFER. The plant assumed -150..150; the get-up only ever uses
    -90..0.

    The SERVO is in the sweep on purpose. Its case turns with the forearm and
    its nearest corner is 15.97 mm from the elbow axis, which is a bigger
    circle than anything on the forearm itself -- leaving it out is how the
    fork web got drawn straight through it (see arm_upper_v6's r16 block).

    The servo is scored on OVERLAP only, never on distance: its two discs are
    bolted flat to the fork tines, so their distance is 0 by design -- the same
    "designed to touch" split check_assembly_v6 makes with its TOUCHING set."""
    upper = arm_upper_v6("L")
    fore = arm_fore_v6("L")
    servo = elbow_servo_mock("L")
    lo, hi = -170.0, 60.0
    good = []
    for k in range(samples):
        ang = lo + (hi - lo) * k / (samples - 1)
        at = Pos(0, 0, -V.ARM_UPPER) * Rot(0, ang, 0)
        vol = 0.0
        for child in (at * fore, at * servo):
            try:
                vol += (upper & child).volume
            except Exception:  # noqa: BLE001
                pass
        try:
            dist = upper.distance_to(at * fore)
        except Exception:  # noqa: BLE001
            dist = 0.0
        if vol < 0.5 and dist >= D.SWEEP_BUFFER - 1e-6:
            good.append(ang)
    need = (-90.0, 0.0)
    if not good:
        print("  elbow ROM: NOTHING clears -- geometry is wrong")
        return (0.0, 0.0)
    band = (min(good), max(good))
    if verbose:
        print(f"  elbow ROM (CAD, {samples} samples over {lo:.0f}..{hi:.0f}): "
              f"{band[0]:+.0f}..{band[1]:+.0f} deg;  declared V.ARM_ROM "
              f"{V.ARM_ROM['elbow']};  get-up needs {need[0]:+.0f}..{need[1]:+.0f}  "
              f"{'OK' if band[0] <= need[0] and band[1] >= need[1] else '** SHORT **'}")
    return band


def mass_g(solid):
    return solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def run_audits(built, verbose=True):
    """printability (in each part's declared orientation) + the deck pilots +
    the elbow ROM + the mass table. Returns True if nothing failed."""
    sys.path.insert(0, os.path.join(HERE, ".."))
    import check_printability as CP  # noqa: E402 -- runtime override only

    ok = True
    print("\n== printability (cad/check_printability.py, declared orientations) ==")
    CP.STL = OUT_STL
    for name, path in built.items():
        base = name[:-2] if name.endswith(("_L", "_R")) else name
        if name.endswith("_R"):
            continue                      # mirror image: same audit by symmetry
        rot, note = PRINT_ORIENT[base]
        CP.ORIENT[name] = (rot, note)
        CP.PRINT_STL[name] = os.path.basename(path)
        if base in SUPPORT_NOTE:
            CP.SUPPORTED[name] = SUPPORT_NOTE[base]
        if CP.audit(name):
            ok = False

    # (deck pilots moved with the mount: shoulder_girdle_v6.check_deck_pilots)

    print("\n== elbow range of motion (CAD, forearm vs the upper arm's fork) ==")
    band = check_elbow_rom()
    if not (band[0] <= -90.0 and band[1] >= 0.0):
        ok = False

    print("\n== mass ==")
    per_arm = 0.0
    for name in ("arm_upper_v6_L", "arm_fore_v6_L"):
        s = BUILDERS[name]()
        m = mass_g(s)
        per_arm += m
        print(f"  {name:22s} {s.volume/1000:6.2f} cm3 -> {m:6.1f} g PETG")
    servos = V.ARM_SERVO_COUNT / 2 * V.SERVO_MASS_3215
    print(f"  {'printed, per arm':22s} {'':6s}    {per_arm:6.1f} g")
    print(f"  {'+ 2x STS3215':22s} {'':6s}    {servos:6.1f} g")
    print(f"  {'= per arm':22s} {'':6s}    {per_arm + servos:6.1f} g   "
          f"(pair {2 * (per_arm + servos):.0f} g)")
    print(f"\n{'PASS' if ok else 'FAIL'}: arm_v6 "
          f"{'clears every audit' if ok else 'has findings above'}")
    return ok


def render(stl_path, png_path, px=560):
    """Three views of one part, MuJoCo offscreen -- cad/render_part.py's recipe
    (MUJOCO_GL set before mujoco is imported; cgl on macOS, not egl)."""
    import mujoco
    import imageio.v2 as imageio

    xml = f"""
    <mujoco>
      <asset><mesh name="part" file="{stl_path}" scale="0.001 0.001 0.001"/></asset>
      <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/>
        <global offwidth="{px}" offheight="{px}"/></visual>
      <worldbody><geom type="mesh" mesh="part" rgba="0.45 0.62 0.90 1"/></worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    m = model.mesh(0)
    v = model.mesh_vert[m.vertadr[0]:m.vertadr[0] + m.vertnum[0]]
    radius = np.linalg.norm(v - v.mean(0), axis=1).max()
    centre = np.zeros(3)
    mujoco.mju_rotVecQuat(centre, v.mean(0), model.mesh_quat[0])
    centre += model.mesh_pos[0]
    ren = mujoco.Renderer(model, px, px)
    frames = []
    for az, el in ((135, -25), (90, 0), (215, -10)):
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = centre, 3.0 * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render().copy())
    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


BUILDERS = {
    "arm_upper_v6_L": lambda: arm_upper_v6("L"),
    "arm_upper_v6_R": lambda: arm_upper_v6("R"),
    "arm_fore_v6_L": lambda: arm_fore_v6("L"),
    "arm_fore_v6_R": lambda: arm_fore_v6("R"),
}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-audit", action="store_true")
    a = ap.parse_args(argv)
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    built = {}
    for name, fn in BUILDERS.items():
        if a.only and name not in a.only:
            continue
        s = fn()
        bb = s.bounding_box()
        stl = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, stl)
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        built[name] = stl
        print(f"{name:22s} bbox {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm   "
              f"{mass_g(s):5.1f} g   {'BED-OK' if max(bb.size.X, bb.size.Y, bb.size.Z) <= 250 else '** TOO BIG **'}")
        if not a.no_render and not name.endswith("_R"):
            print("  render ->", render(stl, os.path.join(OUT_REN, f"{name}.png")))
    ok = True if a.no_audit else run_audits(built)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
