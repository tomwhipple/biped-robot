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
                             deck, so a clevis there is not drawable), and at
                             the bottom leg_link's grip channel turned end for
                             end: the elbow servo's CASE rides the upper arm,
                             running up it from the elbow axis.
    arm_fore_v6(side)        elbow -> hand. A fork at the top straddling the
                             elbow servo's horn AND idler discs, then 160 mm to
                             a 12 mm knuckle -- the plant's hand sphere, same
                             radius.

WHY THE CASE IS IN THE UPPER ARM (2026-10-07)
---------------------------------------------
Nothing ever chose the forearm: it was the plant's round-2 placeholder and the
leg's pattern (the distal link grips), and in the leg that pattern is forced
by one part serving four joints. The upper arm grips no servo at its shoulder
end -- the shoulder servo lives in the girdle -- so the elbow servo can ride
it. Then the servo's lead crosses only the shoulder fold, not the elbow, and
55 g sits 25 mm nearer the shoulder (the arm's inertia about the shoulder
drops ~10 %). docs/design-v6/elbow-servo-upper-arm-2026-10-07.md has the
numbers.

PRINT ORIENTATION -- leg_link_v6's
----------------------------------
Both arm links print ON THEIR BACK (model +X up, check_printability's RY_XUP),
as leg_link_v6 does. This is deliberate and it is the whole reason the
sections below are open C's rather than closed boxes.

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

What it costs: an open C section (back web + two side rails, opening forward),
as in leg_link_v6, because a front plate would be a flat ceiling spanning tine
to tine in this orientation. The C is weaker in TORSION, not in the bending
that matters, and peak measured joint torque here is 1.59 N-m
(shoulder) / 1.18 (elbow) against the STS3215's 2.72 N-m simulated stall.
Against twist, the leg link's answer: an END WALL across the C at each end of
its open span (plates in the X-Y plane, web to front, side to side), which
print as walls rising off the bed. Upper arm: below the shoulder head's
access bore (V.ARM_HEAD_WALL_Z) and just past the cable window above the
elbow servo (V.ARM_LOW_WALL_Z). Forearm: where the fork tines end
(V.ARM_FORE_WALL_Z); the hand knuckle closes the other end.

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
    "arm_fore_v6": "supports on: the two elbow pads' undersides start 5 mm off the bed "
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
# 1. the elbow's two halves, drawn where their parent links already draw them
# ===========================================================================
def _mz(solid):
    """Mirror about the XY plane (z -> -z). The STS3215 is symmetric across its
    width (x), so a part drawn round a HANGING servo, mirrored, is that part
    round the same servo turned end for end about its own axis. That is how
    the elbow's grip channel (leg_link's, round a hanging case) and its fork
    (drawn reaching up from the axis) are carried onto the two arm links
    without being redrawn. Teardrops (+x) and countersinks (y) are unmoved."""
    return mirror(solid, Plane.XY)


def _elbow_grip():
    """The elbow servo's grip channel in the CANONICAL servo frame (axis +Y at
    the origin, horn +Y, case hanging to -z, web on -x). leg_link_v6's, y and z
    unchanged -- same two case faces, same 2.4/3.0 plates, same
    GRIP_HORN_RELIEF for the horn disc, the rib and platform detents with their
    print ramps, the countersunk case screws -- and, as on the leg link, the
    idler plate stops square at V.LL_IDLER_PLATE_TOP, 5 mm above its screws.
    arm_upper_v6 carries it end for end (_mz) at the elbow axis."""
    t = D.GRIP_PLATE_T                            # 2.4
    wx0, wx1 = V.ARM_WEB_X
    grip_x1 = 13.2                                # leg_link's grip plate front edge
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR      # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                 # -17.90
    grip_bot = D.GRIP_BOT                                    # -36.0

    p = parts.box(wx0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t,
                  grip_bot, D.GRIP_TOP_HORN)
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    p += parts.box(wx0, grip_x1, _igo, idler_seat, grip_bot, V.LL_IDLER_PLATE_TOP)
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
    return p


def _elbow_fork():
    """The elbow fork, drawn REACHING UP from the elbow axis at the origin (the
    frame the upper arm used to carry it in): two tines onto the servo's horn
    and idler discs, each ending in an O20 pad with the O14 bolt circle, joined
    by the web for as long as the servo allows. arm_fore_v6 carries it end for
    end (_mz), tines reaching down the forearm.

    Straddling BOTH discs is what keeps the forearm in the same plane as the
    upper arm; going single-sided at the elbow would push the forearm 15-18 mm
    INBOARD, straight at the thigh. The fork is the widest thing on the robot
    after the feet (y +23.45 local) and its inboard tine is the closest thing
    to the leg (y -20.40); check_assembly_v6's arm rows are where that gets
    measured, not argued.

    WEB_END IS NOT A STYLE CHOICE. The forearm turns about the elbow servo,
    whose output-end corners sit at hypot(SV_WID/2, SV_AXIS_FROM_OUT_END) =
    15.97 mm from the axis -- leg_link's "r 16 rule", the same number check_r16
    guards there. The web spans the full tine-to-tine width at x -15.16..-12.76,
    so it has to stop before that circle: it clears r 16 by V.ARM_R16_BUFFER at
    its innermost corner (x = ARM_WEB_X[1])."""
    wx0, wx1 = V.ARM_WEB_X
    fx = V.ARM_FRONT_X
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE           # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE     # -17.40, -20.40
    tine = -V.ARM_FORE_JOG_Z[0]                                    # 24: tine length
    _r16 = math.hypot(D.SV_WID / 2, D.SV_AXIS_FROM_OUT_END) + V.ARM_R16_BUFFER
    web_end = math.sqrt(_r16 ** 2 - wx1 ** 2)
    p = parts.box(wx0, wx1, iy0, hy1, web_end, tine)
    # NO modelled pad support. Printed on its back, the bed is model -x and the
    # pad's lowest point sits 5.16 mm above it facing straight down -- that is
    # OrcaSlicer's job, not this part's (SUPPORT_NOTE registers it).
    for a0, a1 in ((iy0, iy1), (hy0, hy1)):
        p += parts.box(wx0, fx, a0, a1, 0, tine)
        p += parts.cyl_y(D.PAD_D / 2, a0, a1, 0, 0)
    # idler boss, OD tapered 45 deg so its print-underside band never exceeds 45
    _ibh = abs(D.IDLER_BOSS_H)
    p += Pos(0, (D.SV_IDLER_FACE + iy1) / 2, 0) * Rot(90, 0, 0) * Cone(
        D.IDLER_BOSS_D / 2 - _ibh, D.IDLER_BOSS_D / 2, _ibh)
    # --- holes: the disc bolt circle straight through both tines, and the two
    # centre reliefs (horn screw head / idler free-hub post)
    for h in parts.bcd_y(iy0 - 1, hy1 + 1, 0, 0, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, 0)
    p -= parts.cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
                     D.SV_IDLER_FACE + 0.7, 0, 0)
    # --- trim anything past the axis outside the pad radius
    _past = parts.box(-40, 40, -40, 40, -60, 0)
    p -= _past - parts.cyl_y(D.PAD_D / 2, -40, 40, 0, 0)
    return p


# ===========================================================================
# 2. arm_upper_v6 -- shoulder horn to the elbow servo's grip channel
# ===========================================================================
def arm_upper_v6(side="L"):
    """Upper arm. Local frame: the SHOULDER axis is the Y axis at the origin,
    local y = 0 is the arm plane (== the elbow servo's mid-plane, so the arm
    is straight, no jog between the two joints); +x robot forward, the arm
    hangs to -z, the elbow axis is at z = -V.ARM_UPPER. +y is OUTBOARD.
    Qty 2 (mirror pair). Print: on its back, RY_XUP (see the module docstring).

    Top end is SINGLE-SIDED on the shoulder horn -- the horn plate + 1.0
    seating boss + the 4x M3 disc bolt circle, exactly yoke_roll's horn arm --
    with the arm's section run all the way up to it: inboard flange (the horn
    plate) + aft web + outboard flange, open at the front like the rest of the
    arm, and one access bore through the outboard flange for the disc screws.
    As a flat 3 mm plate this head had Z = 41 mm3 about X and a 20 N knock at
    the hand put ~157 MPa in it. There is no idler-side arm and there cannot be
    one: the shoulder servo's idler face looks INBOARD, straight over the deck,
    so a second tine would have to wrap round the case and sweep the deck and
    the head every time the arm folds up. The precedent for carrying a real
    load on one disc is the hip YAW joint, which hangs the entire leg (and half
    the robot) off one horn.

    Bottom end grips the ELBOW servo's case: _elbow_grip() turned end for end
    at the elbow axis, so the case runs up the arm (output end 10.11 below the
    axis, cable end 35.11 above) with its horn still outboard. The forearm's
    fork straddles the two discs.
    """
    wx0, wx1 = V.ARM_WEB_X                    # -15.16, -12.76
    fx = V.ARM_FRONT_X                        # 12.0
    sy0, sy1 = V.ARM_SHAFT_Y                  # -1.70, +10.5
    rt = V.ARM_RAIL_T                         # 2.6
    drop = -V.ARM_UPPER                       # -160, the elbow axis
    jz0, jz1 = V.ARM_JOG_Z                    # -108, -124
    gy0 = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER   # -17.90 grip, idler outer face
    gy1 = D.SV_TOPFACE + D.GRIP_PLATE_T                                   # +19.75 grip, horn outer face
    boss_y0, boss_y1 = -D.PLATE / 2 - D.HORN_BOSS_H, -D.PLATE / 2  # -2.5, -1.5
    py1 = D.PLATE / 2                                              # +1.5
    head_top = 12.0                           # plate reaches above the axis to
                                              # cover the upper bolt-circle pair
    trans_z = -40.0                           # head plate -> full shaft section

    # --- head: the shaft's section run to the shoulder (Tom, 2026-09-21), OPEN
    # at the front like the leg link and tied by an end wall (2026-10-07):
    #   inboard flange == the horn plate: still D.PLATE thick, still bearing on
    #   the horn at boss_y1, so the four disc screws and their stack are
    #   untouched and ARM_Y is untouched. The flanges run forward to
    #   ARM_HEAD_FRONT_X, which the outboard flange's ring round the bore needs.
    hfx = V.ARM_HEAD_FRONT_X                  # 13.4
    p = parts.box(wx0, hfx, boss_y1, py1, trans_z, head_top)
    p += parts.cyl_y(D.HORN_BOSS_D / 2, boss_y0, boss_y1, 0, 0)
    #   aft web and outboard flange, carried up from the shaft at full depth
    p += parts.box(wx0, wx1, sy0, sy1, trans_z, head_top)
    p += parts.box(wx0, hfx, sy1 - rt, sy1, trans_z, head_top)
    #   THE OPENING, away from the body: one bore through the outboard flange
    #   on the shoulder axis to drop the four M3s in. Teardropped (horizontal
    #   bore in this print); its peak is then filled back by the flange's own
    #   front strip, a flat roof ARM_HEAD_ACCESS_R is sized to keep < 8 mm.
    p -= parts.teardrop_y(V.ARM_HEAD_ACCESS_R, sy1 - rt - 1, sy1 + 1, 0, 0, roll=ROLL_UP)
    p += parts.box(V.ARM_HEAD_BORE_ROOF_X, hfx, sy1 - rt, sy1, trans_z, head_top)
    #   the END WALL: flange to flange, web to front, just below the bore --
    #   out of the screws' path, and where the horn's load enters the section.
    p += parts.box(wx0, hfx, boss_y1, sy1, *V.ARM_HEAD_WALL_Z)
    # knock the two top corners off (no sharp external corners; also keeps the
    # plate from reaching further round the servo than it needs to)
    # The two chamfers are NOT the same shape, and that is a print result. In
    # RY_XUP the bed is model -x, so the AFT corner's chamfer is a DOWN-facing
    # slope starting at the first layer: at 7.0 x 7.5 it was 47 deg and the
    # audit called it a LEDGE once the outboard flange gave it something to cut
    # through as well. 8.0 x 6.0 is 37 deg -- self-supporting. The forward
    # corner faces UP and is free, so it keeps the tighter shape.
    for cx in (wx0, hfx):
        s = 1 if cx > 0 else -1
        dx, dz = (8.0, 6.0) if cx < 0 else (6.5, 7.0)
        p -= parts.wedge_y([(cx + s * 0.5, head_top + 0.5),
                            (cx + s * 0.5, head_top - dz),
                            (cx - s * dx, head_top + 0.5)],
                           boss_y1 - 1, sy1 + 1)

    # --- shaft: open C, back web + two rails, opening forward. The rails taper
    # from ARM_FRONT_X at the shoulder (where the bending moment is the hand
    # force x 320 mm) to ARM_TIP_X at the elbow (x 160 mm) -- the section
    # follows the moment. Free in print: x is the print HEIGHT in RY_XUP, so a
    # shrinking x-extent is an up-facing slope, never an overhang.
    tip = V.ARM_TIP_X
    p += parts.box(wx0, wx1, sy0, sy1, jz0, trans_z)
    for a0, a1 in ((sy0, sy0 + rt), (sy1 - rt, sy1)):
        p += parts.wedge_y([(wx0, trans_z), (fx, trans_z), (tip, jz0), (wx0, jz0)], a0, a1)

    # --- jog: the C widens into the grip channel, whose plates it meets at
    # their cable-end edge (jz1). Web full width, rails follow the outer edges
    # (slanted, so their perpendicular thickness is ~2.3 -- still well clear of
    # the THIN gate).
    p += parts.wedge_x([(sy0, jz0), (gy0, jz1), (gy1, jz1), (sy1, jz0)], wx0, wx1)
    p += parts.wedge_x([(sy0, jz0), (sy0 + rt, jz0), (gy0 + rt, jz1), (gy0, jz1)], wx0, fx)
    p += parts.wedge_x([(sy1, jz0), (sy1 - rt, jz0), (gy1 - rt, jz1), (gy1, jz1)], wx0, fx)
    # the jog's rails re-grow the shaft's tapered depth back to ARM_FRONT_X
    p -= parts.wedge_y([(tip + 0.01, jz0), (fx + 1, jz0), (fx + 1, jz0 - 8.0)], gy0 - 1, gy1 + 1)
    # the bottom END WALL of the open span (leg link's top wall, end for end):
    # 2 mm past the cable window, so the elbow servo's lead leaves through the
    # web below it.
    p += parts.box(wx0, tip, sy0, sy1, *V.ARM_LOW_WALL_Z)

    # --- the grip channel on the elbow servo's case, end for end
    p += Pos(0, 0, drop) * _mz(_elbow_grip())

    # --- cable window through the jog's web, just past the case's cable end
    # (leg link's, end for end): the elbow servo's lead leaves to the back of
    # the arm here and runs up the web to the shoulder.
    p -= parts.box(wx0 - 1, wx1 + 1, *V.ARM_CABLE_WINDOW_Y, *V.ARM_CABLE_WINDOW_Z)

    # --- holes: the shoulder disc bolt circle through the horn plate
    # (only through the INBOARD flange: the access bore already opened the
    # outboard flange over the whole bolt circle)
    for h in parts.bcd_y(boss_y0 - 1, py1 + 1, 0, 0, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, boss_y0 - 1, py1 + 1, 0, 0)
    return _mirrored(p, side)


# ===========================================================================
# 3. arm_fore_v6 -- elbow fork to hand
# ===========================================================================
def arm_fore_v6(side="L"):
    """Forearm. Local frame: the ELBOW axis is the Y axis at the origin; +x
    forward, +y outboard (the servo's horn side); the forearm hangs to -z with
    the hand centre at z = -V.ARM_FORE. Qty 2 (mirror pair). Print: on its
    back, RY_XUP; the two pad ends take slicer supports (SUPPORT_NOTE).

    Top end is the elbow FORK, _elbow_fork() turned end for end: tines reaching
    down from the axis, an end wall where they stop (V.ARM_FORE_WALL_Z), then
    a jog narrowing to the shaft. Printed on its back like leg_link_v6, every
    one of those is a wall rising off the bed.

    The hand is a plain PETG knuckle at V.ARM_HAND_R -- the same 12 mm radius
    the plant contacts the floor with. The plant gave it friction 1.0; bare
    PETG is nearer 0.3-0.4, which is inside the study's own robustness sweep
    (mu 0.3 / 0.7 / 1.0, 6/6 standing), so a rubber cap is an improvement, not
    a prerequisite. It is listed as an open item in the design note.
    """
    wx0, wx1 = V.ARM_WEB_X
    fx = V.ARM_FRONT_X
    rt = V.ARM_RAIL_T
    hy = V.ARM_FORE_SHAFT_HY                      # 8.0
    drop = -V.ARM_FORE                            # -160, the hand centre
    r = V.ARM_HAND_R                              # 12
    tip = V.ARM_TIP_X
    fz0, fz1 = V.ARM_FORE_JOG_Z                   # -24, -52
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE           # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE     # -17.40, -20.40

    # --- the fork on the elbow servo's discs, end for end
    p = _mz(_elbow_fork())
    # --- the END WALL where the tines end: tine to tine, web to front
    p += parts.box(wx0, fx, iy1 - 0.01, hy0 + 0.01, *V.ARM_FORE_WALL_Z)

    # --- jog from the fork's width down to the shaft, then the shaft itself
    p += parts.wedge_x([(iy0, fz0), (-hy, fz1), (hy, fz1), (hy1, fz0)], wx0, wx1)
    p += parts.wedge_x([(iy0, fz0), (iy0 + rt, fz0), (-hy + rt, fz1), (-hy, fz1)], wx0, fx)
    p += parts.wedge_x([(hy1, fz0), (hy1 - rt, fz0), (hy - rt, fz1), (hy, fz1)], wx0, fx)
    p += parts.box(wx0, wx1, -hy, hy, drop, fz1)
    for a0, a1 in ((-hy, -hy + rt), (hy - rt, hy)):
        p += parts.wedge_y([(wx0, fz1), (fx, fz1), (tip, drop), (wx0, drop)], a0, a1)

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
    return _mirrored(p, side)


def elbow_servo_mock(side="L"):
    """The elbow STS3215 in the ELBOW frame -- origin on the elbow axis, the
    upper arm's orientation: the canonical mock turned end for end about its
    own axis, horn still +Y (outboard), case running UP the upper arm (output
    end 10.11 below the axis, cable end 35.11 above). It rides the upper arm."""
    return _mirrored(Rot(0, 180, 0) * CA.servo_mock(), side)


# ===========================================================================
# fasteners + insertion paths (the same contract leg_link_v6.SCREWS keeps)
# ===========================================================================
def SCREWS():
    """Every fastener the arm pair adds, per side, with the frame each one is
    quoted in. `axis` points from the head toward the tip."""
    s = []
    # the shoulder's own fasteners (deck pilots + the four case screws) moved
    # to shoulder_girdle_v6.SCREWS() with the mount itself.
    # disc screws, length by the stack under the head (V.disc_screw), no
    # washers. The shoulder horn plate sits on a 1.0 mm seating boss, so its
    # stack is 4.0 (plate + boss), not the bare 3.0 of the elbow's horn tine.
    sh_stack = D.PLATE + D.HORN_BOSS_H                                   # 4.0
    el_h_stack = D.PLATE                                                 # 3.0
    el_i_stack = D.SV_IDLER_FACE - (D.IDLER_ARM_INNER - D.PLATE)        # 3.6
    sh = V.disc_screw(sh_stack, "horn")
    eh = V.disc_screw(el_h_stack, "horn")
    ei = V.disc_screw(el_i_stack, "idler")
    for i in range(4):
        ang = math.radians(90 * i)
        s.append(dict(name=f"shoulder_horn_{i}", frame="arm_upper",
                      kind=f"M3x{sh[0]} button head into the shoulder horn disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.PLATE / 2, D.BCD / 2 * math.cos(ang)),
                      axis=(0, -1, 0), length=float(sh[0]), stack=sh_stack, engage=sh[1], flange=sh[2]))
        s.append(dict(name=f"elbow_horn_{i}", frame="arm_fore",
                      kind=f"M3x{eh[0]} button head into the elbow horn disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.SV_HORN_FACE + D.PLATE,
                           D.BCD / 2 * math.cos(ang)),
                      axis=(0, -1, 0), length=float(eh[0]), stack=el_h_stack, engage=eh[1], flange=eh[2]))
        s.append(dict(name=f"elbow_idler_{i}", frame="arm_fore",
                      kind=f"M3x{ei[0]} button head into the elbow idler disc",
                      pos=(D.BCD / 2 * math.sin(ang), D.IDLER_ARM_INNER - D.PLATE,
                           D.BCD / 2 * math.cos(ang)),
                      axis=(0, 1, 0), length=float(ei[0]), stack=el_i_stack, engage=ei[1], flange=ei[2]))
    # the grip screws, in the upper arm: the case runs UP from the elbow axis
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append(dict(name=f"elbow_grip_horn_{zrow:.0f}_{lx:+.0f}", frame="arm_upper",
                          kind="M2.5x8 self-tap, flat head (into the servo case)",
                          pos=(lx, D.SV_TOPFACE + D.GRIP_PLATE_T, -V.ARM_UPPER + zrow),
                          axis=(0, -1, 0), length=8.0))
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append(dict(name=f"elbow_grip_idler_{lx:+.0f}", frame="arm_upper",
                      kind="M2.5x8 self-tap, flat head (into the servo case)",
                      pos=(lx, D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER,
                           -V.ARM_UPPER + D.CASE_HOLES_BOT[1]), axis=(0, 1, 0), length=8.0))
    return s


# where each piece is offered from, on the bench (fly-in / animate_v6)
INSERT = {
    "servo_shoulder": (0, 0, 1),     # dropped into the open cradle from above
    "arm_upper": (0, 1, 0),          # offered straight in onto the horn, outboard->in
    "servo_elbow": (1, 0, 0),        # slid into the upper arm's grip channel from the front
    "arm_fore": (0, 0, -1),          # lifted up, its fork tines either side of the discs
}


# ===========================================================================
# audits
# ===========================================================================
def check_elbow_rom(samples=41, verbose=True):
    """The elbow's real limit: sweep the forearm against the upper arm AND the
    elbow servo it carries, and report the widest band that clears
    D.SWEEP_BUFFER. The plant assumed -150..150; the get-up only ever uses
    -90..0.

    The SERVO is in the sweep on purpose. The forearm turns about its case,
    whose output-end corners are 15.97 mm from the elbow axis -- inside the
    fork, which is how a fork web gets drawn straight through it (see
    _elbow_fork's r16 note).

    The servo is scored on OVERLAP only, never on distance: its two discs are
    bolted flat to the fork tines, so their distance is 0 by design -- the same
    "designed to touch" split check_assembly_v6 makes with its TOUCHING set."""
    el = Pos(0, 0, -V.ARM_UPPER)
    upper = arm_upper_v6("L")
    fore = arm_fore_v6("L")
    servo = el * elbow_servo_mock("L")
    lo, hi = -170.0, 60.0
    good = []
    for k in range(samples):
        ang = lo + (hi - lo) * k / (samples - 1)
        at = el * Rot(0, ang, 0)
        vol = 0.0
        for fixed in (upper, servo):
            try:
                vol += (fixed & (at * fore)).volume
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

    print("\n== elbow range of motion (CAD, forearm's fork vs the upper arm + elbow servo) ==")
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
