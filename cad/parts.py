"""Printable parts for the biped (build123d, algebra mode).

Run:  .venv/bin/python cad/parts.py        -> exports STLs to cad/stl/, prints
                                              per-part bbox / bed check / mass and
                                              the assembly mass rollup.

Part set (7 unique, 13 prints) -- pelvis v6, 2026-08-04:
  pelvis        x1  THE WHOLE TORSO: deck + yaw housing + battery bay + board
                    recess + gopro pad, in one deck-top-down print
  yaw_carrier   x2  bolts to the yaw horn, carries the hip-roll bay
  yoke_roll     x2  clevis on the hip-roll servo horn/idler, flange below
  yoke_pitch    x2  clevis on the thigh servo horn/idler, flange above
                    (bolts to yoke_roll flange, rotated 90 deg -> hip universal)
  leg_link      x4  thigh AND shin: grips a servo case, forks to the next servo
  foot          x2  sole + ankle servo pocket + rear retention walls
  gopro_base    x1  three-prong camera mount, bolted to the DECK (the roof)

v6 folds v5's battery_tray and board_frame into the pelvis -- "I'd hoped to make
this a single part". The tray's flat floor (the reason it could not print in the
pelvis) became two 47 deg seat chamfers with open air between them; the board
frame became a recess with two standoffs and two guide cheeks. Nine screws, four
heat-sets and two part numbers went away, and the printed torso lost 21 % of its
volume. v6 also corrects BOARD_GD_OUTLINE to the manufacturer's 65.01 x 56.01
(the wiki's 65 x 65 was wrong by 8.99 mm), which is what lets the board stand
upright with its ports edge 15 mm above the deck, and puts the GoPro back --
straight onto the deck, since the deck is now the roof.

LEGACY, still in this file but no longer built: tower(), gopro_base(),
imu_carrier(). The tower carried the board and the pack; pelvis v4 splits those
onto battery_guard and board_frame (see the v4 block in dimensions.py). The
camera/IMU head comes back as its own bolt-on; until it does, those three
functions and their BOARD_GD_*/BATT_SEAT_X/TOWER_* constants are kept intact so
the old geometry still reads back.

Conventions: every pitch joint has the servo HORN on +Y; the same part serves
left and right legs (legs are translations, not mirrors).
"""
import math
import os
from build123d import *
import dimensions as D

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
# STEP is exported alongside every STL, from the SAME solid in the same run.
# They used to be separate commands (cad/export_step.py) and the STEPs drifted:
# on 2026-07-27 every per-part STEP was three days stale, predating the
# imu_carrier redesign, the yaw-carrier clearance and that day's pad changes --
# so anyone opening one in FreeCAD was measuring superseded geometry. Coupling
# them here makes that impossible.
STEP_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl_x(r, x0, x1, y, z):
    """Cylinder along X from x0..x1 at (y, z)."""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def cyl_y(r, y0, y1, x, z):
    """Cylinder along Y from y0..y1 at (x, z)."""
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_z(r, z0, z1, x, y):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def teardrop_y(r, y0, y1, x, z, roll=0, full=False):
    """Self-supporting horizontal hole (axis along Y): a round bore plus a
    45 deg roof, so a horizontal hole prints without a bridged (sagging) top.
    Returns the SOLID void to subtract (bore + roof), not a hole in a part.
    `roll` spins the roof about the bore axis toward the part's PRINT-up
    direction: 0 = +z (parts printed model-up), 90 = +x (leg_link prints
    web-down, print-up = model +x), 180 = -z (parts modeled upside-down vs
    their print, e.g. yoke_pitch).
    Default is CAPPED: the roof is truncated at the round bore's own top
    (center + r), leaving a 2r*(sqrt(2)-1) ~ 0.83r flat mini-bridge. The void
    then never reaches past the round hole it replaces -- a full r*sqrt(2)
    peak pierced plate edges and left ~0.1 mm shells on the O19 idler bosses
    wherever holes were placed with round-hole margins. Pass full=True only
    where clearance above the hole is proven (e.g. gopro_base M5)."""
    L = abs(y1 - y0)
    bore = Rot(90, 0, 0) * Cylinder(r, L)
    # A square of side 2r rotated 45 deg about Y peaks at (0, r*sqrt(2)); clip
    # it to |dx| <= r and above the bore center so it only adds the top roof
    # (its 45 deg faces meet the circle exactly at the tangent points).
    diamond = Rot(0, 45, 0) * Box(2 * r, L, 2 * r)
    keep = box(-r, r, -L / 2, L / 2, 0, 2 * r if full else r)
    return Pos(x, (y0 + y1) / 2, z) * Rot(0, roll, 0) * (bore + (diamond & keep))


def teardrop_x(r, x0, x1, y, z, roll=0, full=False):
    """teardrop_y's sibling with the bore along X. roll about the bore axis:
    0 = peak +z, 180 = peak -z (pelvis prints deck-top-down)."""
    L = abs(x1 - x0)
    bore = Rot(0, 90, 0) * Cylinder(r, L)
    diamond = Rot(45, 0, 0) * Box(L, 2 * r, 2 * r)
    keep = box(-L / 2, L / 2, -r, r, 0, 2 * r if full else r)
    return Pos((x0 + x1) / 2, y, z) * Rot(roll, 0, 0) * (bore + (diamond & keep))


def wedge_y(pts_xz, y0, y1):
    """Triangular (or any polygon) prism: a profile of (x, z) points extruded
    thin along Y into the band [y0, y1]. Used for gussets and print chamfers.
    NOTE: don't assume which way extrude() runs from Plane.XZ -- measure and
    shift, so the prism lands exactly in [y0, y1] on any build123d version
    (the old +Pos(0, hi, 0) guess put the v2 foot gussets 2.4 mm outside
    their tab bands without anything catching it)."""
    lo, hi = (y0, y1) if y0 < y1 else (y1, y0)
    s = extrude(Plane.XZ * Polygon(*pts_xz, align=None), amount=(hi - lo))
    return Pos(0, lo - s.bounding_box().min.Y, 0) * s


def wedge_z(pts_xy, z0, z1):
    """Same idea as wedge_y, but the profile is (x, y) and the prism runs along
    Z. Used for print chamfers on pockets whose overhang is in the x-y plane
    (leg_link's rib and platform detents, which print with +x up)."""
    lo, hi = (z0, z1) if z0 < z1 else (z1, z0)
    s = extrude(Plane.XY * Polygon(*pts_xy, align=None), amount=(hi - lo))
    return Pos(0, 0, lo - s.bounding_box().min.Z) * s


def wedge_x(pts_yz, x0, x1):
    """wedge_y's third sibling: a (y, z) profile extruded thin along X into the
    band [x0, x1]. Needed wherever the chamfer/gusset slope lives in the Y-Z
    plane -- X-NORMAL plates (board_frame's bulkhead gable, the yaw box side
    lead-ins, the guard's foot gussets). Same bbox-measured placement as
    wedge_y: do not assume which way extrude() runs off Plane.YZ."""
    lo, hi = (x0, x1) if x0 < x1 else (x1, x0)
    s = extrude(Plane.YZ * Polygon(*pts_yz, align=None), amount=(hi - lo))
    return Pos(lo - s.bounding_box().min.X, 0, 0) * s


def csk_y(x, z, y_face, sign):
    """90-deg countersink void for an M2.5 FLAT-head self-tapper (CASE_CS_D
    mouth), mouth on the face at y_face opening toward sign*Y, apex meeting
    the CASE_SCREW_CLEAR bore at CASE_CS_DEPTH. 0.3 overshoot past the face.
    45-deg walls: prints self-supporting even as a horizontal bore."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(-90, 0, 0) if sign > 0 else Rot(90, 0, 0)
    mid = y_face + sign * (0.3 - h / 2)
    return Pos(x, mid, z) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def csk_x(y, z, x_face, sign):
    """csk_y's twin for bores along X (yaw_carrier bay walls)."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(0, 90, 0) if sign > 0 else Rot(0, -90, 0)
    mid = x_face + sign * (0.3 - h / 2)
    return Pos(mid, y, z) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def csk_z(x, y, z_face, sign):
    """csk_y's twin for bores along Z (pelvis deck stator screws)."""
    h = D.CASE_CS_DEPTH + 0.3
    rot = Rot(0, 0, 0) if sign > 0 else Rot(180, 0, 0)
    mid = z_face + sign * (0.3 - h / 2)
    return Pos(x, y, mid) * rot * Cone(D.CASE_SCREW_CLEAR / 2,
                                       D.CASE_CS_D / 2 + 0.3, h)


def bcd_y(y0, y1, x, z, roll=0):
    """4x M3 clearance holes (horn/idler bolt circle) along Y at pad (x, z).
    Teardropped (roll = print-up, see teardrop_y): these bores are horizontal
    in every part's print orientation, and a sagged bore top binds the bolt."""
    r = D.BCD / 2
    return [teardrop_y(D.PAD_HOLE / 2, y0, y1, x + dx, z + dz, roll)
            for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


def bcd_x(x0, x1, y, z, roll=0):
    r = D.BCD / 2
    return [teardrop_x(D.PAD_HOLE / 2, x0, x1, y + dy, z + dz, roll)
            for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r))]


# ---------------------------------------------------------------- yoke_roll
# MODELLED SUPPORT FINS DELETED 2026-07-30 (user: "none of the supports break
# away or are done very well ... it's actually pretty terrible"). Both yokes
# now print with SLICER supports -- see the yoke_roll docstring. What was
# measured wrong with the hand-modelled version before deleting it:
#
#   - The anchor tabs ran from `lo - 0.3`, i.e. 0.3 mm BELOW the bed plane, so
#     the TABS became the lowest geometry and the flange -- the actual bed
#     adhesion -- floated. First layer was 26 mm2 (roll) / 36 mm2 (pitch) of
#     disconnected 2x3 stamps instead of the flange's 194 / 175 mm2: 14 % and
#     21 % of what the part alone would give. The flared feet added FOR bed
#     area sat 0.30 mm up, touching nothing.
#   - Every foot splayed 2.00 mm PAST the part's own silhouette.
#   - The pad-rim fins were flat-topped blocks under a CYLINDRICAL pad: contact
#     on one tangent line, gap opening to 3.20 mm over 7 mm of run.
#   - Each tab fused 0.3 mm INTO the part across 2x3 mm -- a weld, not a
#     break-away contact.
#
# The root cause is worth keeping: those tabs existed to stop
# check_printability calling the plate a BEAM, and the full-width walls to stop
# it calling them an ISLAND. The geometry was shaped to satisfy the audit --
# which has no concept of "a support sits 0.35 mm under this" -- rather than
# the printer.


def yoke_roll():
    """Hip-roll clevis. Local frame: roll axis == X axis through origin.
    +X = robot forward = servo horn side. Flange faces down (mates yoke_pitch).
    Print: WALL -- on edge, model -Y on the bed, WITH SLICER SUPPORTS. Qty 2.

    FIXED 2026-07-30 (user: print them "as a wall for strength"). It used to
    print flange-down with the arms rising as vertical columns, so the layer
    lines lay ACROSS the arm -- and a cantilevered arm carries its bending load
    as tension ALONG the arm, straight through the interlayer bond. They kept
    snapping mid-arm. The control case is leg_link: same cantilever-with-a-pad
    load path, far longer arm, thinner mid-span, never broken -- because it
    prints web-down, so "the filament runs the length of the leg vs extending
    vertically from the print table".

    Standing it on edge puts the arm length in the bed plane, so that tension
    now runs along the filament. Thickening PLATE was NOT the alternative --
    it adds to the idler screw stack and undoes the engagement fix (see
    IDLER_ARM_INNER), and it would not have addressed the actual failure plane.

    SUPPORTS ARE THE SLICER'S JOB (2026-07-30). On edge, the arm plates and the
    pad rims start in mid-air, so this needs support -- but the modelled fins
    that used to live here were worse than none: they put their anchor tabs
    0.3 mm BELOW the bed and left the flange, the only real bed adhesion,
    floating on 14 % of its footprint. See the block above _fin_wall's grave.

    Turn supports on in the slicer instead: they follow the cylindrical pad
    undersides, keep a proper interface gap, and are built to peel. There is
    now ONE STL for this part -- no _print variant to keep in sync.
    """
    zf0 = -D.ROLL_AXIS_TO_FLANGE                    # flange top
    zf1 = zf0 - D.YOKE_FLANGE_T                     # flange bottom
    hx0, hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H, D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE
    ix1, ix0 = D.ROLL_ARM_INNER, D.ROLL_ARM_INNER - D.PLATE  # (wall outer -19.95, 1 gap)

    p = box(-23.95, 24.45, -D.YOKE_FLANGE_Y / 2, D.YOKE_FLANGE_Y / 2, zf1, zf0)
    # horn arm: plate + boss through nothing (horn sits outside the bay wall)
    p += box(hx0, hx1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, hx0, hx1, 0, 0)
    p += cyl_x(D.HORN_BOSS_D / 2, D.SV_HORN_FACE, hx0, 0, 0)          # boss 1.0
    # idler arm: plate + long boss reaching through the bay-wall slot
    p += box(ix0, ix1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, ix0, ix1, 0, 0)
    p += cyl_x(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, ix1, 0, 0)        # boss 4.15
    # SINK the idler pad 2026-07-30 (user: "the yoke roll pad is too thick for
    # our screws"). It was, by 1.55 mm of engagement -- see ROLL_* in
    # dimensions.py for why this joint cannot reach the pitch side's 3.60 and
    # takes an M3x8 into a sunk pad instead. Cut, not a thinner plate: the arm
    # keeps its full 3.0 everywhere it carries bending.
    p -= cyl_x(D.PAD_D / 2, ix0, ix0 + D.ROLL_IDLER_PAD_SINK, 0, 0)
    # holes: ONE bore per bolt-circle position, drilled from the idler-arm OUTER
    # face (ix0) clear through to past the horn plate (hx1). BUGFIX 2026-07-23:
    # this started at SV_IDLER_FACE-1 (the disc face), leaving the idler arm's
    # outer plate + long boss SOLID -- the 4x M3x10 idler-disc screws the BOM
    # buys ("through the long boss") were un-fittable. Now the idler bolt circle
    # is a true clearance through-hole. (Same fix in yoke_pitch and leg_link.)
    for h in bcd_x(ix0 - 1, hx1 + 1, 0, 0):
        p -= h
    p -= cyl_x(D.HORN_CENTER_RELIEF_D / 2, D.SV_HORN_FACE - 1, hx1 + 1, 0, 0)
    # relief deepened to a through-bore 2026-07-24: the servo's free-hub
    # post is O6.1 and reaches up to 1.37 past the disc face (vendor-STEP
    # worst case) -- the old 1 mm-deep relief could land the arm on the post
    p -= cyl_x(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6, D.SV_IDLER_FACE + 0.7, 0, 0)
    b = D.YOKE_BOLT_SQ / 2
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        p -= cyl_z(D.M3_CLEAR / 2, zf1 - 1, zf0 + 1, sx, sy)
    return p


# ---------------------------------------------------------------- yoke_pitch
def yoke_pitch():
    """Hip-pitch clevis on the thigh-servo horn/idler. Local frame: pitch axis
    == Y axis through origin; flange on top (heat-set inserts, mates yoke_roll).
    Print: WALL -- on its back, model -X on the bed, WITH SLICER SUPPORTS.
    Qty 2.

    FIXED 2026-07-30 alongside yoke_roll; see that docstring for why the old
    flange-down orientation was snapping arms. This was the worse of the two:
    PITCH_ARM_REACH 26 against yoke_roll's 16 is ~1.6x the root moment for the
    same load.

    It gets RY_XUP -- the exact transform leg_link prints in -- because it
    grips the same servo the same way (straddle along model Y, arm along model
    Z). If leg_link survives this load path in this orientation with a 97 mm
    arm, a 26 mm one should. Supports come from the SLICER, not from modelled
    fins -- see yoke_roll. This is the tippier of the two: 32 mm tall standing
    on the flange edge, 175 mm2, so give it a brim as well as supports.
    """
    zf1 = D.PITCH_ARM_REACH                          # flange bottom (arms side)
    zf0 = zf1 + D.YOKE_FLANGE_T                      # flange top (mating face)
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21

    p = box(-D.YOKE_FLANGE_X / 2, D.YOKE_FLANGE_X / 2, iy0, hy1, zf1, zf0)
    p += box(-12, 12, hy0, hy1, 0, zf1) + cyl_y(D.PAD_D / 2, hy0, hy1, 0, 0)
    # idler arm: NOT the horn arm's full-width plate. That plate capped hip
    # flexion at ~105 deg: the thigh leg_link's idler grip plate + web share
    # this arm's Y band (-20.35..-18 of -21..-18), and the grip plate's top
    # front corner (r=20.75 off the axis) sweeps into the plate front edge.
    # Get-up needs -110 (>=95 kinematic bound + margin, DESIGN 2026-07-14).
    # The thigh sweep only covers angles <=65 deg (front) and >=166 deg
    # (rear) at r>=16, so the arm is a hub disc r14 (2.0 under the r=16
    # swing floor) plus a riser plate through the top-rear dead sector:
    # front edge x=4 -> flexion clears to ~129 deg, rear edge x=-14 ->
    # extension clears to ~97 deg. The hub's front-upper quadrant is cut to
    # a 45 deg face off the riser edge: printed flange-down this is the
    # disc's print-underside, and the face keeps it support-free (bolt rims
    # stay >=1.3 mm past the cut).
    hub = cyl_y(14.0, iy0, iy1, 0, 0)
    hub -= wedge_y([(4, 10), (14, 0), (17, 0), (17, 15), (4, 15)],
                   iy0 - 1, iy1 + 1)
    p += hub + box(-14, 4, iy0, iy1, 9, zf1)
    p += cyl_y(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, iy1, 0, 0)   # boss 1.2
    # idler bolt circle drilled from the arm OUTER face (iy0) through to the horn
    # side -- BUGFIX 2026-07-23 (was SV_IDLER_FACE-1, leaving the outer plate
    # solid; see yoke_roll). Makes the 4x M3x8+washer idler screws fittable.
    for h in bcd_y(iy0 - 1, hy1 + 1, 0, 0, roll=180):
        p -= h
    p -= cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, 0)
    # through-bore relief for the O6.1 free-hub post (see yoke_roll note)
    p -= cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6, D.SV_IDLER_FACE + 0.7, 0, 0)
    b = D.YOKE_BOLT_SQ / 2
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        p -= cyl_z(D.HEATSET_D / 2, zf1 - 1, zf0 + 1, sx, sy)    # heat-set M3
    return p


# ---------------------------------------------------------------- leg_link
def leg_link(print_fins=False):
    """Thigh / shin link (same part, qty 4). Local frame: upper joint axis ==
    Y axis at origin (this is the gripped servo's horn axis); the servo hangs
    below (top end +10.11, bottom -35.11); +X = robot forward; the lower fork
    grips the NEXT servo's horn (+Y) / idler (-Y) at z = -90.
    Print: lying on the back web (web face on the bed), from
    leg_link_print.stl -- print_fins=True adds break-away support fins under
    the narrow fork slabs and pad rims, which otherwise float 4.7 mm above
    the bed with nothing beneath them (check_printability ISLAND/LEDGE; the
    fork CANNOT reach the bed there -- that volume is swept by the foot walls
    and the servo case top at joint extremes). Snap the fins out after
    printing; they are not part of the working geometry, so the plain STL
    (sim meshes, assembly checks, renders) never includes them.
    """
    t = D.GRIP_PLATE_T
    drop = -D.LINK_DROP
    web_x1 = -12.36 - D.WEB_GAP                     # web inner face (cable gap)
    web_x0 = web_x1 - 2.4                           # web outer face, -15.16
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21
    # --- grip channel on the servo case (plates reach the web outer face).
    # Front edge 13.2, not the case half-width 12.36: the +10.25 case screws
    # end 11.95 from center, and a 12.36 edge left a 0.41 mm web past the
    # hole (single filament -- flakes off) with the screw head overhanging.
    grip_x1 = 13.2
    p = box(web_x0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t, D.GRIP_BOT, D.GRIP_TOP_HORN)
    # relief around the O19.6 output boss / horn skirt (they spin vs this plate)
    p -= cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    # idler grip plate: its inner face follows the REAL idler-side case face
    # (SV_IDLER_CASE_FACE, -14.75), not the mirrored SV_BOTFACE (-17.35). There
    # is no case material at -17.35 -- the plate used to clamp a 2.60 mm air
    # gap, which is what "the idler side does not conform" meant. Seating here
    # puts the plate on the same face the two grip screws pull into.
    # Its OUTER face is GRIP_PLATE_T_IDLER off the seat (-17.90), NOT the old
    # run-out to iy0 (-21): that flush-out (print-review 2026-07-15, killing a
    # 0.65 step) plus the seat move made the wall 6.1 mm, and the grip screws
    # suddenly needed M2.5x10. Pulled back so M2.5x8 works universally (user,
    # 2026-07-30) -- head-to-case is now 3.15, an x8 bites ~4.9 mm of case,
    # same class as the horn side's 5.6. The step vs the fork plate is back,
    # but the jog block below already bridges it, exactly as on the horn side.
    # The web still runs out to iy0; only the grip plate pulled in.
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR          # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                     # -17.90 outer
    p += box(web_x0, grip_x1, _igo, idler_seat, D.GRIP_BOT, D.GRIP_TOP_IDLER)
    p += box(web_x0, web_x1, iy0, D.SV_TOPFACE + t, D.WEB_END, D.WEB_TOP)
    # CROSS BRACE between the fork tines (2026-07-29). The fork is a 38.45 mm
    # wide U -- the horn arm at y 20.45..23.45 and the idler arm at y -21..-18 --
    # and the only thing joining them is the 2.4 mm web along the BACK edge. So
    # the tines can splay and twist about that web, worst at the pads 90 mm down.
    # This is a plate in the X-Y plane spanning tine to tine, which closes the U
    # into a box. It sits BELOW the cable window (z -48..-37) so it cannot foul
    # the servo lead. It runs all the way back to the WEB (user, 2026-07-29):
    # the first cut started at FORK_NARROW_X "to stay out of the web's x-band",
    # which left its print-underside floating 2.3 mm over the web -- a 38 mm
    # tine-to-tine bridge (the check_printability 202 mm2 LEDGE at print z 4.7).
    # Rooted on the web it prints as a wall growing straight off the back plate,
    # nothing to bridge. The x-band it now fills (web..FORK_NARROW_X, z -54..-50)
    # is below the case bottom (-35.11) and off the raceway (outer web face), so
    # it blocks nothing -- but it does bury the z -52 zip-tie bores, which are
    # deleted below. Its z band is a constant: it is what gives if the next
    # servo's sweep about the lower axis needs the room -- check_assembly is
    # the arbiter.
    p += box(web_x0, 12, D.IDLER_ARM_INNER, D.SV_HORN_FACE,
             D.BRACE_Z[0], D.BRACE_Z[1])
    # --- fork arms down to the next servo (wide near the web, narrow below).
    # The wide horn-side section starts at the web/grip edge (19.75), not the
    # horn face (20.45): the 0.7 band is only a running clearance where the
    # NEXT link's grip plate sweeps it, and that sweep (r <= 39 about the
    # lower axis) never rises past z ~ -72 -- above that the 0.7 slot between
    # web edge and fork plate was dead air, so it is solid (same feedback).
    for a0, a1, wide0 in ((hy0, hy1, D.SV_TOPFACE + t), (iy0, iy1, iy0)):
        p += box(web_x0, 12, wide0, a1, D.FORK_WIDE_Z, -33)
        p += box(D.FORK_NARROW_X, 12, a0, a1, drop, D.FORK_WIDE_Z)
        p += cyl_y(D.PAD_D / 2, a0, a1, 0, drop)
    # --- permanent slab backing ("make it solid" -- print feedback
    # 2026-07-16). Foot-sweep mapping at ankle +-45 shows the swept volume
    # in the slab bands is ONLY: below z -93 (both sides, heel passing
    # under the axis) and -- idler side -- up to z -55.6 (wall corner at
    # -45 deg). Everything else fills to the web face for free: the horn
    # slab becomes a full-depth blade, the idler side gets a stub, and the
    # break-away fin problem shrinks to one short free-ended fin + two
    # finger-snap pad stubs (in leg_link_print.stl below).
    p += box(web_x0, D.FORK_NARROW_X, hy0, hy1, -92, D.FORK_WIDE_Z)
    # idler side: the corner lobe's union is exactly z -68.4..-55.6, so
    # solid fills BOTH sides of it (1.6 mm margins); the 16 mm stretch
    # left between the fills prints as a 3 mm-wide end-anchored ribbon
    # bridge -- no support at all (fin-under-slab was rejected 3x in
    # print review; 45-deg chamfers shrinking the span land inside the
    # swept lobe, verified).
    p += box(web_x0, D.FORK_NARROW_X, iy0, iy1, -54, D.FORK_WIDE_Z)
    p += box(web_x0, D.FORK_NARROW_X, iy0, iy1, -92, -70)
    # idler boss: OD tapered so its print-underside band never exceeds 45 deg --
    # it used to need a break-away fin wedged 0.1 mm from the arm plate
    # (unremovable, print feedback 2026-07-16). The OD is a loose locator;
    # concentricity comes from the screw pattern, so the taper costs nothing.
    # The run is tied to the boss HEIGHT (2026-07-30) rather than the old fixed
    # 1.5: when IDLER_BOSS_H shrank 1.2 -> 0.6 for the screw fit, a 1.5 run
    # would have laid the taper back to 22 deg off horizontal -- a worse
    # overhang than the ledge it was cut to avoid. Run == rise keeps it at 45.
    _ibh = abs(D.IDLER_BOSS_H)
    p += Pos(0, (D.SV_IDLER_FACE + iy1) / 2, drop) * Rot(90, 0, 0) * Cone(
        D.IDLER_BOSS_D / 2 - _ibh, D.IDLER_BOSS_D / 2, _ibh)
    # jog block joining horn grip plate (out at 19.75) to fork plate (21.45+)
    p += box(web_x0, 12, D.SV_TOPFACE, hy1, -36.5, -33)
    p += box(web_x0, 12, iy0, idler_seat, -36.5, -33)
    # DETENT for the servo's horn-side RIB (measured off cad/vendor/ST3215.step,
    # 2026-07-28). The rib stands 1.13 mm proud of the case top face right under
    # the horn grip plate, so without this pocket the plate lands on the rib
    # instead of the case and the link ROCKS -- confirmed on the bench.
    # Cut LAST: the jog block above re-fills the cable end of it (z -36.5..-33),
    # so cutting with the grip plate left a 14 mm3 sliver of the rib still
    # buried. The plate seats on the case either side of the pocket, and the
    # M2.5 grip screws at +/-10.25 clamp OUTSIDE the +/-7.42 rib band, so the
    # clamp load path is untouched.
    # ...and run its TOP END out through the channel mouth (2026-07-30). The
    # servo enters this C-section along +z ONLY -- web on -x, both grip plates
    # on +/-y -- so the rib has to travel the whole plate on its way down to
    # the pocket. Ending the pocket at the rib's own top left two slivers of
    # plate between the pocket end (z -8.13) and the horn relief circle (which
    # only reaches z -7.13 out at the rib's +/-7.57 x-band): 1.72 mm3 of
    # material the rib scraped past for its entire 26 mm of travel
    # (check_assembly's component insertion path, the servo-scale twin of the
    # a4ce69e screw find). Opening the pocket to GRIP_TOP_HORN costs only
    # those two lunes -- everything else up there is already inside the relief.
    p -= box(-D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR, D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR,
             D.SV_TOPFACE - 0.01, D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
             -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR,
             D.GRIP_TOP_HORN + 0.01)
    # ...and RAMP its print-top edge (2026-07-29, user: "the conformal edges
    # of the servo rib need supporting"). This part prints web-down, so
    # print-up is model +x, and the pocket's +x wall is where plate material
    # RESUMES over the void -- a 1.44 x 26.33 mm ledge bridging 26 mm on two end
    # anchors (check_printability BEAM, 38 mm2 at print z 23.0). Opening the
    # ledge into a ramp lets the slicer grow it -- but the ramp must be
    # ANCHORED on the pocket-floor wall (y past _ry1), the only material that
    # is solid below it all through the pocket band. So the void closes toward
    # the SEATING face as the print rises: hypotenuse from (_rx, _ry1) up to
    # (_rx + _rr, _ry0), print-down normal ~(-0.64, -0.77). FIXED 2026-07-29
    # (user flagged the 58 mm2 face in CAD): the first cut sloped the OPPOSITE
    # way, growing the front off the seat-face corner -- under which there is
    # only the pocket void and servo-side air, i.e. an unsupported knife-edge
    # ribbon bridging the full 27 mm. Cost of the fix: the seat land loses a
    # 1.7 mm chamfered strip along the pocket edge (x 7.8..9.6); the plate
    # still seats either side of it and the M2.5 grip screws at +/-10.25
    # clamp outboard of the ramp entirely.
    _rx = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    _ry0 = D.SV_TOPFACE - 0.01
    _ry1 = D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR
    _rr = 1.2 * (_ry1 - _ry0)          # ~40 deg from vertical: inside the
                                       # 45 deg rule, the pocket is a relief
    p -= wedge_z([(_rx, _ry0), (_rx, _ry1), (_rx + _rr, _ry0)],
                 -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR - 0.6,
                 -D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR + 0.6)
    # DETENT for the idler-side PLATFORM -- the mirror of the rib pocket above,
    # and the second half of seating this plate properly (2026-07-29). The
    # moulded back-cover platform stands 1.90 mm proud of the case face over
    # most of the plate's footprint, and it sits BETWEEN the plate and the face
    # the screws pull into, so it is relieved rather than seated on. What is
    # left bearing is a band across the cable end -- which carries both grip
    # screws at z -32.75 -- plus a land up each side outboard of the platform.
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR          # +/-11.10
    _iy0 = idler_seat                                    # -14.90, at the face
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR    # -16.95, pocket floor
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    # ...and RAMP its +x wall (2026-07-29). Printing web-down, model +x is up,
    # so the +x wall is a 28.5 mm2 DOWNWARD-facing face hanging over the
    # pocket -- the same defect the rib pocket had, just short enough
    # (13.9 mm) to stay under the BEAM span threshold and therefore never
    # reported. Same fix, same anchoring rule: the ramp grows off the
    # pocket-floor wall (y past _iy1, solid below through the whole pocket
    # band) and the void closes toward the seating face -- hypotenuse from
    # (_ix, _iy1) up to (_ix + _irr, _iy0), print-down normal ~(-0.64, +0.77).
    # FIXED 2026-07-29 (user flagged the 39.7 mm2 face): the first cut sloped
    # the opposite way, growing the front off the open seat-face corner with
    # nothing below it. The mirror cut at -x is GONE (user flagged its 3.20 mm
    # hypotenuse): the -x wall is the pocket FLOOR in print -- notching it
    # supported nothing and just gouged the floor.
    _irr = 1.2 * (_iy0 - _iy1)                           # ~40 deg, as the rib
    p -= wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                 _iz0 - 0.6, _iz1 + 0.6)
    # --- holes: case grip screws (M2.5 FLAT-head self-tap into the servo case
    # holes -- bench truth 2026-07-28, M3 is too wide); teardropped with the
    # peak +x (this part prints web-down, print-up = model +x). Every grip
    # hole is COUNTERSUNK so the head sits FLUSH: the yoke/fork arm of the
    # joint above sweeps just 0.70 mm off the horn-plate outer face, and a
    # proud pan head there rode the arm and skewed the link on the bench
    # (25.5 mm3 overlap from hip +/-60 deg on -- see check_assembly).
    for zrow in D.CASE_HOLES_TOP:                 # horn-side face rows
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                            D.SV_TOPFACE + t + 1, lx, -zrow, roll=90)
            p -= csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):   # idler face: row 32.75 only
        p -= teardrop_y(D.CASE_SCREW_CLEAR / 2, _igo - 1,
                        idler_seat + 1, lx, -D.CASE_HOLES_BOT[1], roll=90)
        p -= csk_y(lx, -D.CASE_HOLES_BOT[1], _igo, -1)
        # head/driver ACCESS counterbore (2026-07-30, user: "screw holes on
        # the highlighted surface are blocked"). The jog block and the fork
        # wide plate still run out to iy0 (-21), 3.1 mm proud of the head
        # seat now that the grip plate pulled back to _igo -- and their top
        # edges (z -33) cut across the row-32.75 head circle (csk mouth
        # bottoms at z -35.45). The lower half of each countersink was
        # buried, so the screw could neither be inserted nor driven square.
        # Only this row: the horn rows (8.30/29.00) stay >= 1.3 mm clear of
        # the -33 edge. Teardropped like the bores (peak +x = print-up).
        p -= teardrop_y(D.CASE_CS_D / 2 + 0.4, iy0 - 1, _igo + 0.05,
                        lx, -D.CASE_HOLES_BOT[1], roll=90)
    # --- holes: lower joint pads
    # idler bolt circle drilled from the fork OUTER face (iy0) through to the
    # horn side -- BUGFIX 2026-07-23 (was SV_IDLER_FACE-1, leaving the idler fork
    # plate solid so the 4x M3x8+washer idler-disc screws could not be fitted;
    # see yoke_roll). Already-printed links have a SOLID outer face (no pilot):
    # hand-drill on the O14 BCD marked off the idler boss centre, or from a
    # printed jig locating on the boss.
    for h in bcd_y(iy0 - 1, hy1 + 1, 0, drop, roll=90):
        p -= h
    p -= cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, drop)
    # through-bore relief for the O6.1 free-hub post (see yoke_roll note)
    p -= cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
               D.SV_IDLER_FACE + 0.7, 0, drop)
    # --- cable window through the web: the servo's rear ports sit INBOARD
    # of the web (case end, z ~ -36) while the raceway runs down the web's
    # OUTER face -- without an opening every joint-crossing cable pierced
    # the plastic (print-review feedback 2026-07-16; dress.py measured
    # 27-74 mm3 of cable/web intersection per segment). 9 x 11 passes a
    # 3-pin plug; both the OUT cable (down to the raceway) and the incoming
    # IN cable (up into the port) share it.
    p -= box(web_x0 - 1, web_x1 + 1, -4.5, 4.5, -48, -37)
    # NOTE (2026-07-23): an idler-side cable "notch" was briefly added here on a
    # bad assumption (a lead routed DOWN the idler exterior). It was removed --
    # the gripped servo's lead routes via this web window to the back raceway
    # (assembly.md), not the idler face, and a sweep analysis of the ANKLE
    # servo's heel connector through the full ankle ROM (0/+-40, toes-pointed =
    # +40) shows ~10 mm clearance to this fork, 0 mm3 interference even with an
    # oversized connector block. No notch needed; don't cut a hole in this
    # load-bearing fork plate speculatively. See check_assembly ankle-cable block.
    # --- zip-tie holes in the web (servo cable runs down the back); at +-9
    # so the window keeps a >=2 mm ligament to each hole. One pair only: it
    # straddles the window and captures the cables right at the exit. The
    # second pair at z -52 is GONE (user, 2026-07-29): the cross brace now
    # fills the web's inner side across z -54..-50, so a tie could no longer
    # loop through there -- the bores would just perforate the brace root.
    for ly in (9, -9):
        p -= cyl_x(2.25, web_x0 - 1, web_x1 + 1, ly, -40)
    # --- trim everything BELOW the lower joint axis back to the pad radius.
    # The slab filled to z -92 while the fork starts at the axis (z -90), so a
    # 2 mm lip hung below with four sharp vertical arrises at x -10.50 (y -21,
    # -18, +20.45, +23.45) sitting at r 10.69 -- outside the PAD_D/2 = 10.0 pad
    # they hang off. Material below the axis carries nothing (the leg runs
    # UPWARD from here) but it is the first thing to swing into the mating part,
    # so it costs joint travel for free. Cutting back to the pad circle removes
    # the corners entirely and leaves the sub-axis silhouette exactly the pad
    # (user, 2026-07-28). Must precede the print fins: those stubs live below
    # -92.8 outside the pad and are meant to survive this.
    _below = box(-40, 40, -40, 40, -200, drop)
    p -= _below - cyl_y(D.PAD_D / 2, -40, 40, 0, drop)
    # --- and chamfer the two corners BESIDE the axis, in the plane of the plate.
    # Trimming below the axis (above) left the profile stepping straight off the
    # pad: r 10 -> 12.0 at the front edge, r 10 -> 15.0 at the rear web face,
    # both within 15 deg of the axis. Those steps are the corners that swing into
    # the mating part, so each is blended from the pad tangent out to full width.
    # Kept strictly outside r 10, so the pad and its bolt circle are untouched.
    _pr = D.PAD_D / 2
    _yl, _yh = iy0 - 1, hy1 + 1                      # spans both fork plates
    p -= wedge_y([(_pr, drop), (12.0, drop),
                  (12.0, drop + D.LEG_CORNER_CUT_FRONT_H)], _yl, _yh)
    p -= wedge_y([(-_pr, drop), (web_x0, drop),
                  (web_x0, drop + D.LEG_CORNER_CUT_REAR_H)], _yl, _yh)
    if print_fins:
        # break-away print supports, final round (2026-07-16: fin-under-slab
        # was rejected three times -- every variant is trapped under the
        # slab it supports). There are NO fins anymore. With the permanent
        # backing above, the idler slab's only unsupported stretch
        # (z -84..-54) is anchored at BOTH ends -- solid stub one side, the
        # pad body the other -- and prints as a plain 3 mm-wide bridge, the
        # same class the slicer already rates trivial. The only break-away
        # pieces left are two 4 mm pad stubs under the pads' bottom
        # tangents (permanent material is foot-swept below -93): they stand
        # at the fork tips, in the open past the part's end, and snap out
        # with fingers.
        for yc, sgn in ((hy0 + D.PLATE / 2, 1), (iy0 + D.PLATE / 2, -1)):
            # Run the blank IN to the pad and let the pad-clearance cut below
            # define its top face, instead of stopping at FORK_NARROW_X - 0.35.
            # That x limit was sized for the sub-axis slab, and once that slab
            # was trimmed back to the pad circle (2026-07-28) these stubs stood
            # 1.21 mm off the part -- floating debris supporting nothing. They
            # were already loose at 0.80 mm before the trim; now they hug the
            # pad at the same 0.35 mm break-away gap the island posts use.
            ht = D.FIN_T / 2
            stub = box(web_x0, 0.0, yc - ht, yc + ht, -97.0, -92.8)
            stub -= cyl_y(D.PAD_D / 2 + 0.35, yc - 2, yc + 2, 0, drop)
            # flared foot stays full width: a 1.2 mm wall needs the bed area
            stub += box(web_x0, web_x0 + 1.5, min(yc - ht, sgn * (abs(yc) + 4.3)),
                        max(yc + ht, sgn * (abs(yc) + 4.3)), -97.0, -92.8)
            p += stub
        # island posts under the idler ribbon (print review 2026-07-16: the
        # bare 16 mm bridge was rejected): two 2.4 x 3 columns, 0.35 under
        # the slab, >=1 mm clear of the web (-58) and the solid fills (-70,
        # -54) at bed level -- separate first-layer islands like the pad
        # stubs, attached to nothing; the remaining bridge spans are
        # 2 / 2.5 / 5.5 mm.
        yc = iy0 + D.PLATE / 2
        for z0, z1 in ((-68.0, -65.0), (-62.5, -59.5)):
            p += box(web_x0, D.FORK_NARROW_X - 0.35,
                     yc - D.FIN_T / 2, yc + D.FIN_T / 2, z0, z1)
    return p


# ---------------------------------------------------------------- yaw_carrier
def yaw_carrier(print_fins=False):
    """Hip-yaw carrier (v3yaw, qty 2). Bolts to the yaw-servo HORN (below the
    deck) and carries the hip-roll bay that used to hang off the pelvis. Local
    frame: yaw axis == Z through origin, z=0 at the horn mounting face (top),
    +X = robot forward = roll-servo output side, +Z toward the servo. The roll
    bay hangs below (roll axis at CARRIER_ROLL_AXIS); the roll servo slides UP
    into it exactly as it did into the old pelvis bay (output end down, horn
    forward, 8x M2.5 through the walls) -- BAY_BORE / BAY_WALL_DROP / cheeks /
    U-slot are unchanged, just relocated. Print: like the old pelvis bay --
    horn-plate face on the bed, walls rise (RX180), U-slot prints upward-open.
    SLICE yaw_carrier_print.stl -- print_fins=True adds the break-away breakout
    that carries the connector window's ceiling bar (see below). The plain STL
    stays clean for sim meshes and assembly checks.
    """
    zc = D.CARRIER_ROLL_CEIL                         # -3.0 bay ceiling/plate bot
    za = D.CARRIER_ROLL_AXIS                         # -38.11 roll axis
    zw = zc - D.BAY_WALL_DROP                        # -44.0 wall bottoms
    cy0 = 12.36 + D.BAY_CHEEK_GAP                    # cheek inner face offset
    hw = cy0 + D.WALL                                # bay half width in Y, 15.26
    # horn mount plate (== bay ceiling): spans the bay footprint, z [zc, 0]
    p = box(-19.95, 19.95, -hw, hw, zc, 0)
    # roll bay walls (identical to the old pelvis bay, centered at y=0)
    p += box(D.SV_TOPFACE, 19.95, -hw, hw, zw, zc)          # front (horn +X)
    p += box(-19.95, -D.SV_TOPFACE, -hw, hw, zw, zc)        # rear (idler -X)
    for s in (1, -1):
        p += box(-D.SV_TOPFACE, D.SV_TOPFACE, s * cy0, s * (cy0 + D.WALL),
                 zw, zc)                                     # cheeks
    # IDLER-SIDE SEAT + PLATFORM DETENT (2026-08-01, user: "add the idler wheel
    # side detent to conform to that side of the servo"). The rear wall's inner
    # face above is SV_TOPFACE MIRRORED (-17.35), and THERE IS NO SERVO THERE:
    # the real idler-side case face is SV_IDLER_CASE_FACE (-14.75). Placing the
    # mock in this bay measures the first material inboard of that face as the
    # free hub (-17.35) and the idler disc (-16.80) -- both of which ROTATE --
    # then the moulded platform (-16.65), then the case at -14.75. So this wall
    # and its two M2.5 retention screws have been clamping 2.60 mm of air. Same
    # defect the leg_link grip plate (2026-07-29) and the foot tab (2026-07-30)
    # already had fixed; the carrier was the last part still on the phantom face.
    #
    # The seat grows INWARD ONLY. The wall's OUTER face stays at -19.95 because
    # ROLL_ARM_INNER is measured off it -- moving it would silently re-open the
    # hip-roll idler screw budget that was just settled. Screw engagement does
    # not change either way: the M2.5x8 sits flush in its countersink at -19.95
    # and bites the case 2.80 mm whether the 2.45 mm ahead of it is plastic or
    # air. What changes is that it now pulls the case against something.
    _seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR       # -14.90, at the face
    p += box(-D.SV_TOPFACE, _seat, -cy0, cy0, zw,
             za + D.SV_AXIS_FROM_REAR)                   # up to the cable end
    # DETENT for the moulded back-cover PLATFORM (SV_IDLER_BOSS_*: 1.90 proud of
    # the case face, y +/-10.70, z -23.86..-10.60 in this frame). It sits BETWEEN
    # the screws and the face they pull into, so it is RELIEVED, not seated on --
    # and the relief is a CHANNEL OPEN AT THE BOTTOM, not the leg_link's closed
    # pocket: this servo slides UP into the bay, so the platform has to travel
    # the full height of the seat to reach its place. A closed pocket would just
    # be a wall the servo cannot get past. Its side walls straddle the platform
    # with RIB_RELIEF_CLR a side, which is what makes it a DETENT rather than a
    # clearance hole -- it locates the servo across the bay. Printed
    # horn-plate-down (RX180) model +z is print-down, so the channel's closed
    # end is its print FLOOR and nothing bridges.
    _px = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR          # +/-11.10
    _pf = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR     # -16.95, channel floor
    p -= box(_pf, _seat + 0.01, -_px, _px, zw - 1,
             za - D.SV_IDLER_BOSS_Z[0] + D.RIB_RELIEF_CLR)   # -10.20, closed top
    # axis bore: downward-open U-slot in both walls (servo slides up)
    p -= cyl_x(D.BAY_BORE / 2, -21, 21, 0, za)
    p -= box(-21, 21, -D.BAY_BORE / 2, D.BAY_BORE / 2, zw - 1, za)
    # roll-servo retention screws (teardrop peak -z, printed ceiling-on-bed)
    # COUNTERSUNK for FLAT heads (2026-07-28): the build now uses M2.5 flat-head
    # self-tappers throughout, so these sit flush in the wall instead of standing
    # CASE_HEAD_H proud of it. 1.25 mm of the 2.6 mm wall, 1.35 mm left behind.
    for zrow in D.CASE_HOLES_TOP:                    # front wall (horn face)
        for s in (1, -1):
            p -= teardrop_x(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1, 21,
                            s * D.CASE_HOLE_LAT, za + zrow, roll=180)
            p -= csk_x(s * D.CASE_HOLE_LAT, za + zrow, 19.95, +1)
    for zrow in D.CASE_HOLES_BOT:                    # rear wall (idler face)
        for s in (1, -1):
            # ...through the idler SEAT as well as the wall (2026-08-01): the
            # bore used to stop at the phantom face and would now dead-end
            # 2.45 mm inside the seat pad added above.
            p -= teardrop_x(D.CASE_SCREW_CLEAR / 2, -21, _seat + 1,
                            s * D.CASE_HOLE_LAT, za + zrow, roll=180)
            p -= csk_x(s * D.CASE_HOLE_LAT, za + zrow, -19.95, -1)
    # yaw-horn bolts: 4x M3 into the horn disc (O14 circle). Bores are VERTICAL
    # in the print (part flipped, Z stays Z) -> plain holes. (The rear bolt was
    # briefly dropped for a ceiling cable channel that turned out to align with
    # nothing -- see the SV_CONN note -- so it is restored.)
    r = D.BCD / 2
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        p -= cyl_z(D.PAD_HOLE / 2, zc - 1, 1, dx, dz)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, zc - 1, 1, 0, 0)
    # roll-servo CONNECTOR window, over the MEASURED trench (SV_CONN: band
    # 11.75..16.35 above the roll axis, nearly full case width; sockets open
    # OUT of the idler face = rearward here). Plugs + leads pass through the
    # rear wall and route out the open bay rear. Bottom edge za+11.3 leaves a
    # 1.0 mm full-width bar to the bore crown (za+10.3) and 1.25 mm to the
    # za+8.30 retention screw bores; width +/-11.4 clears the sockets and
    # keeps 3.86 mm posts to the wall edges (+/-15.26).
    # (x1 runs past the idler seat, 2026-08-01 -- the window used to stop at the
    # phantom face, which would leave the seat pad standing in front of the
    # sockets the plugs come out of.)
    p -= box(-21, _seat + 1, -D.SV_CONN_HW - 0.5, D.SV_CONN_HW + 0.5,
             za + D.SV_CONN_L[0] - 0.45, za + D.SV_CONN_L[1] + 1.0)
    if print_fins:
        # break-away BREAKOUT under the connector window (2026-07-26). Printed
        # horn-plate-down, that window's print-CEILING is the 1.0 mm bar named
        # above: a 2.6 x 22.8 mm strip carrying nothing but itself, anchored
        # only at the +/-11.4..15.26 posts, with the U-slot void directly over
        # it so no wall backfills the span. 22.8 mm of bare 1 mm PETG bridge is
        # ~3x the 8 mm rule and the same class as the 16 mm leg_link ribbon the
        # 2026-07-16 print review rejected. (check_printability read the SHORT
        # side -- 2.6 mm, the wall thickness -- and passed it; that side is open
        # on BOTH faces, so nothing bridges across it. Fixed there too, see
        # _sides_anchored.) Three columns split it into four 4.95 mm bridges,
        # well inside the rule -- worth the extra piece on a bar this thin (1 mm
        # of bare perimeter with the U-slot void above it, so no infill and no
        # next layer to iron it flat). They stand on the window sill and fuse to
        # the bar through 1.0 mm necks, full wall depth so the ceiling face
        # parts cleanly; the 2.4 mm body is inset 0.2 mm from each wall face so
        # a blade gets behind it. NOTE the spacing is set by the NECK, not the
        # body -- the neck is what interrupts the bridge. Snip or twist them out
        # before the roll servo goes in; nothing seats on this bar, so the nubs
        # only need trimming flush enough to clear the plug bodies.
        wl = za + D.SV_CONN_L[0] - 0.45          # -26.81, window lower edge
        wu = za + D.SV_CONN_L[1] + 1.0           # -20.76, window upper edge
        hwin = D.SV_CONN_HW + 0.5                # 11.40, window half width
        body, neck, nh = 2.4, 1.0, 1.0           # body / neck width, neck rise
        n = 3                                    # columns
        span = (2 * hwin - n * neck) / (n + 1)   # 4.95 mm bridge left per gap
        for i in range(n):
            yc = -hwin + (i + 1) * span + (i + 0.5) * neck   # -5.95, 0, +5.95
            for w, inset, z0, z1 in (
                    (neck, 0.0, wl, wl + nh),           # neck onto the bar
                    (body, 0.2, wl + nh, wu - nh),      # body (blade relief)
                    (neck, 0.0, wu - nh, wu)):          # neck onto the sill
                p += box(-19.95 + inset, -D.SV_TOPFACE - inset,
                         yc - w / 2, yc + w / 2, z0, z1)
    # DETENT for the roll servo's horn-side RIB (same defect as leg_link and
    # foot, 2026-07-28). The bay's front wall face IS SV_TOPFACE, so the rib --
    # 1.13 mm proud over y +/-7.42, z CARRIER_ROLL_AXIS+8.73..+34.26 -- lands
    # flat on it and holds the servo off the wall by its full height (the whole
    # 415 mm3 was buried). Cut LAST so nothing unions over it. The 8x M2.5 case
    # screws run at y +/-10.25, outboard of the rib band, so their seats keep
    # full wall thickness.
    # Its LOWER end runs out into the U-slot mouth (2026-07-30): the servo
    # slides UP into this bay, so the rib travels the front wall from the wall
    # bottom to its detent. Ending the detent at the rib's own bottom edge left
    # two corner slivers between it (ra+8.58) and the bore crown -- at the
    # rib's +/-7.57 y-band the O20.6 bore only opens to ra+6.98 -- so the rib
    # scraped 1.72 mm3 of wall on the way in (check_assembly's component
    # insertion path; the leg_link had the identical defect at its own mouth).
    # Below the axis the U-slot has already taken everything out to +/-10.3, so
    # this costs the two slivers and nothing else, and the 8x M2.5 case screws
    # stay outboard at y +/-10.25 as before.
    ra = D.CARRIER_ROLL_AXIS
    p -= box(D.SV_TOPFACE - 0.01,
             D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
             -D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR,
             D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR,
             zw - 0.01,
             ra + D.SV_HORN_RIB_L[1] + D.RIB_RELIEF_CLR)
    return p


# ---------------------------------------------------------------- pelvis
def pelvis():
    """THE WHOLE TORSO, one print (v6, 2026-08-04). Local frame: deck top at
    z=0, robot forward = +X, legs at y = +/-HIP_SEP/2 = +/-33.

    v5 was three printed parts -- pelvis, battery_tray, board_frame -- and the
    user's read was "I'd hoped to make this a single part". It is one now. The
    tray became two seat chamfers; the board frame became a recess with two
    standoffs and a pair of card rails. Nine screws and two part numbers went
    with them. Only gopro_base still bolts on, and that one is deliberate: it is
    the camera's crash fuse.

    WHAT IS IN HERE, front to back:
      battery bay   underslung ahead of the housing, pack lying across y on two
                    SEAT CHAMFERS, lifted out vertically through a deck aperture
      yaw housing   one block, two servo cells + the centre wire channel between
                    two full-width 36 mm shear webs (unchanged from v5)
      board recess  aft of the rear web: two cheeks with card RAILS, two
                    standoff bosses, the board standing upright and transverse.
                    The +y cheek is the SERVICE side and is cut for it (window,
                    shell relief, ear notch -- BR_SVC_* and the block below)
      deck          the roof: gopro pad, wire window, aperture, belt slots

    THE PRINT RULE THAT SHAPES ALL OF IT. This prints DECK TOP FACE ON THE BED,
    so model -z is print-UP: material may hang below the deck freely, but
    NOTHING can be printed above it, and any cavity that closes underneath has
    to close on faces steeper than 45 deg or bridge less than 8 mm. That single
    rule is why the battery bay has no floor (two 47 deg chamfers catch the pack
    and the middle is open air), why the board's UPPER screw row has no boss
    (it lands at +11.5, above the deck -- the rails do that job instead), and
    why the belt notch is 18 mm and not 21 (the wall below it bridges that span;
    check_printability's BEAM_OK is 20).

    WHICH WAY ROUND THE BOARD GOES IS NOT OPTIONAL (2026-08-06). BUS EDGE UP,
    SERVICE EDGE TO +Y. Flip it end-for-end and the power switch and the USB
    port go to the BOTTOM of the recess, 35 mm down a 2.6 mm slot; flip it about
    the other axis and the service cuts are in the wrong cheek. The recess is
    symmetric enough to accept all four, so this is a build instruction, not a
    fit -- see BOARD_GD_SVC_EDGE.

    THE BOARD IS UPRIGHT, NOT RECLINED. The corrected 56.01 mm board
    (BOARD_GD_OUTLINE, from the manufacturer's drawing -- the wiki's 65 x 65 was
    wrong by 8.99) could be reclined ~47 deg to bring all four screw rows below
    the deck. It would cost 38 mm of aft overhang, putting the torso's tail at
    x ~ -78, i.e. 26 mm BEHIND THE HEEL, where a backward fall lands. Upright
    costs the two upper screws and nothing else: the tail stays at BR_AFT_X
    (-54.44), 2.4 mm behind the heel and 2.6 mm shorter than v5's separate
    frame, and the ports edge ends up 15 mm ABOVE the deck -- the best access
    the board has had in any revision, with the leads dropping straight onto it
    from the deck-top corridors.

    CLEARANCE IS Z-SEPARATION, as in v5. Every printed face here stays above
    TORSO_FLOOR_Z (-41.8), 1.0 mm over the yaw carrier's horn-plate top, and the
    carrier is the HIGHEST moving part of the leg -- so no pose of any joint can
    reach torso structure. The board's own bottom edge sits on BR_BOT_Z (-41.0)
    for the same reason.
    """
    zd = -D.DECK_T                                  # -5 deck bottom
    zb = D.YAW_BOX_BOT                              # -41 housing bottom
    cx0 = D.YAW_CASE_X_REAR - D.YAW_SEAT_GAP        # -35.41 cell rear inner
    cx1 = D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP       # +10.41 cell front inner
    cyw = D.YAW_BOX_HW_IN                           # 12.66 case half width + fit
    w = D.YAW_SEAT_WALL
    lead = D.YAW_BOX_LEADIN
    hw = D.HOUSING_HW                               # 48.26
    # ---- deck. Full width forward; aft of DECK_CUT_X only the two EARS
    # survive, because the board stands up through the deck plane inboard of
    # DECK_CUT_HY and its components with it.
    p = box(D.DECK_AFT_X, D.DECK_FWD_X, -D.DECK_L / 2, D.DECK_L / 2, zd, 0)
    p -= box(D.DECK_AFT_X - 1, D.DECK_CUT_X, -D.DECK_CUT_HY, D.DECK_CUT_HY,
             zd - 1, 1)
    # ---- the housing: ONE solid block, three cells cut out of it (v5, kept
    # verbatim). A solid minus its cells has continuous walls and solid corners
    # by construction; a union of wall rings can only butt-join at the corners,
    # and the corners are what carry the differential twist.
    p += box(D.YAW_BOX_X_REAR, D.YAW_BOX_X_FRONT, -hw, hw, zb, zd)
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        p -= box(cx0, cx1, by - cyw, by + cyw, zb - 1, zd)          # servo cell
        # mouth lead-in on all four inner faces: the servo is offered UP into a
        # 36 mm-deep slip fit that would otherwise catch on its own first layer.
        # Printed deck-top-down these chamfers face print-UP, so they are free.
        p -= wedge_y([(cx0, zb), (cx0, zb + lead), (cx0 - lead, zb)],
                     by - cyw - w - 1, by + cyw + w + 1)               # rear
        p -= wedge_y([(cx1, zb), (cx1, zb + lead), (cx1 + lead, zb)],
                     by - cyw - w - 1, by + cyw + w + 1)               # front
        for s in (1, -1):
            yi = by + s * cyw
            p -= wedge_x([(yi, zb), (yi, zb + lead), (yi + s * lead, zb)],
                         cx0 - w - 1, cx1 + w + 1)                     # cheeks
    # centre cell == the wire channel, closed fore and aft by the two shear webs
    p -= box(cx0, cx1, -D.HOUSING_CHAN_HW, D.HOUSING_CHAN_HW, zb - 1, zd)
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        # stator screws: 4x M2.5 flat self-tap DOWN through the deck into the
        # idler-side case rows (8.30 and 32.75 behind the axis). Vertical, and
        # the only retention the yaw servos have -- which is why every part of
        # this design keeps an ACCESS_D cylinder clear above each one.
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zd - 1, 1,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                p -= csk_z(-xrow, by + s * D.CASE_HOLE_LAT, 0.0, +1)
        # --- idler-face interface, verbatim from v3/v4/v5 (only the leg centre
        # has ever moved). This is the interface the assembled robot proves.
        # (1) disc + hub CLEARANCE POCKET: the idler disc (O19.2, +0.27 proud)
        # and its hub screws (+0.82) ROTATE with the output -- clamping them
        # against a flat deck binds the yaw joint. Pocket them 1.3 deep.
        p -= cyl_z(21.5 / 2, zd, zd + 1.3, 0, by)
        # (2) stator screw PADS: the screw bosses sit ~1.78 BELOW the slab plane
        # the deck touches, so bare screws would bow the case. O7 pads descend
        # SEAT_PAD_H (deliberate under-reach), then the clearance is re-drilled.
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p += cyl_z(3.5, zd - D.SEAT_PAD_H, zd,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zd - D.SEAT_PAD_H - 0.1, 1,
                           -xrow, by + s * D.CASE_HOLE_LAT)
        # (3) yaw CONNECTOR deck HOLE over the measured trench: the sockets open
        # UP out of the case face; plugs and leads pass through and route AFT
        # along the deck top. Nothing overhangs it -- the board is 25 mm further
        # aft and stands clear above -- so there is full plug room here.
        p -= box(-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 0.6,
                 by - 10.5, by + 10.5, zd - 1, 1)
        p -= box(-D.SV_CONN_L[0] + 0.5, -10.0, by - 4.5, by + 4.5, zd, zd + 1.3)
    # ---- battery bay: side and front walls hang off the deck; the AFT wall is
    # the housing's own front web. No floor: the pack lands on two SEAT
    # CHAMFERS and the middle of the bay's underside is open air.
    yi, yo = D.BT_WALL_Y_IN, D.BT_WALL_Y_OUT
    for s in (1, -1):
        p += box(D.BT_SEAT_X, D.BT_X1, s * yi, s * yo, D.BT_SEAT_Z, zd)
    # front wall runs down to the bay bottom, not just to the pack's seat: the
    # forward seat chamfer hangs off its inner FACE, and stopping the wall at
    # the seat plane left that chamfer touching it along one edge only -- a
    # detached solid (caught by a solids() count, which is why parts.py prints
    # one).
    p += box(D.BT_WALL_X1, D.BT_X1, -yo, yo, D.BT_BAY_BOT, zd)      # front wall
    # belt notch: the side wall is absent from the deck down to BT_BELT_NOTCH_Z
    # over BT_BELT_X, so the strap can pass from the deck slot, inboard under
    # the deck, and over the pack. The wall BELOW the notch bridges the notch's
    # length in the print, which is why that length is 18 and not 21.
    for s in (1, -1):
        p -= box(D.BT_BELT_X[0], D.BT_BELT_X[1], s * (yi - 1), s * (yo + 1),
                 D.BT_BELT_NOTCH_Z, zd + 1)
    # seat chamfers, on the bay's two X-NORMAL faces (the narrow direction --
    # 31.5 across, against 69 the other way, so the chamfer that catches the
    # pack is a tenth the depth). 47 deg, not 45: an exactly-45 face sits ON
    # check_printability's threshold. They run the pack's full length, so a soft
    # LiPo is carried along two lines 31 apart instead of on its end edges.
    _rise = D.BT_SEAT_CHAMFER * math.tan(math.radians(D.BT_SEAT_DEG))
    for x0, sgn in ((D.BT_SEAT_X, +1), (D.BT_WALL_X1, -1)):
        p += wedge_y([(x0, D.BT_SEAT_Z), (x0, D.BT_SEAT_Z - _rise),
                      (x0 + sgn * D.BT_SEAT_CHAMFER, D.BT_SEAT_Z - _rise)],
                     -yi, yi)
    # ---- board recess: two cheeks with card RAILS + two standoff bosses.
    for s in (1, -1):
        p += box(D.YAW_BOX_X_REAR, D.BR_AFT_X,
                 s * D.BR_CHEEK_Y_IN, s * D.BR_CHEEK_Y_OUT, D.BR_BOT_Z, zd)
        # cheek mouth lead-in: the board is dropped in from above between the
        # cheeks, so their top inner corners are funnelled. Printed deck-top-down
        # this face is print-UP, so it is free. (There is deliberately no edge
        # RAIL here -- see BR_CHEEK_LEADIN in dimensions.py for why the board's
        # own edge components rule one out and why two screws are enough.)
        p -= wedge_x([(s * D.BR_CHEEK_Y_IN, zd),
                      (s * D.BR_CHEEK_Y_IN, zd - D.BR_CHEEK_LEADIN),
                      (s * (D.BR_CHEEK_Y_IN + D.BR_CHEEK_LEADIN), zd)],
                     D.BR_AFT_X - 1, D.YAW_BOX_X_REAR + 1)
        # standoff boss off the rear web's AFT face, at the board's LOWER screw
        # row. M2.5 self-tap along +x from behind, head on the PCB's aft face in
        # open air. Teardrop roof: printed deck-top-down a horizontal cylinder's
        # model-top is its print-underside (v4's lesson, same fix).
        sy = s * D.BOARD_GD_SCREW_DY
        p += cyl_x(3.5, D.BR_PCB_X0, D.YAW_BOX_X_REAR, sy, D.BR_SCREW_Z)
        r, th = 3.5, math.radians(D.BF_BOSS_ROOF_DEG)
        roof = wedge_x([(sy - r / math.sin(th), D.BR_SCREW_Z),
                        (sy + r / math.sin(th), D.BR_SCREW_Z),
                        (sy, D.BR_SCREW_Z + r / math.cos(th))],
                       D.BR_PCB_X0, D.YAW_BOX_X_REAR)
        p += roof & box(D.BR_PCB_X0, D.YAW_BOX_X_REAR, sy - r, sy + r,
                        D.BR_SCREW_Z, D.BR_SCREW_Z + 2 * r)
        p -= cyl_x(D.M25_TAP / 2, D.BR_PCB_X0 - 1, D.YAW_BOX_X_REAR + 1,
                   sy, D.BR_SCREW_Z)
    # ---- SERVICE EDGE ACCESS (2026-08-06). The board's XH power inlet and its
    # two USB-C ports all mate ALONG one 56.01 edge, and in this mount that edge
    # is vertical at +y -- not the top edge the v6 recess was drawn for
    # (BOARD_GD_SVC_EDGE, corrected off the manufacturer's dimension drawing).
    # The +y cheek was therefore standing 0.41 mm INSIDE the USB-C shells, never
    # mind in front of the plug. Three cuts, +y side only; the -y cheek keeps
    # its full section and does the guiding.
    svc = D.BR_SVC_SIGN
    wx0, wx1 = D.BR_SVC_WIN_X
    wz0, wz1 = D.BR_SVC_WIN_Z
    yin, yout = svc * D.BR_CHEEK_Y_IN, svc * D.BR_CHEEK_Y_OUT
    # (1) the WINDOW the "USB" plug passes through, open up to the cheek's own
    # top edge so the inlet plug (which sits ON the deck plane) clears it too.
    p -= box(wx0, wx1, yin - svc * 0.5, yout + svc * 1.0, wz0, wz1)
    # (1a) ...and its ROOF. Printed deck-top-down, the window's LOWER edge is
    # what has to re-form in mid-air, and at 8.5 mm across that is past
    # BRIDGE_OK. So it closes to a point on BR_SVC_WIN_TAPER faces instead --
    # the same trick as every other chamfer in this part, and it lands in the
    # relief groove below, where there is nothing to protect.
    _h = (wx1 - wx0) / 2 * math.tan(math.radians(D.BR_SVC_WIN_TAPER))
    p -= wedge_y([(wx0, wz0), (wx1, wz0), ((wx0 + wx1) / 2, wz0 - _h)],
                 yin - svc * 0.5, yout + svc * 1.0)
    # (2) the RELIEF groove below the window: nothing plugs into the "LIDAR"
    # USB-C, but its shell is just as proud as the other one, so the cheek's
    # inner face is grooved rather than the board being asked to fit.
    p -= box(D.BR_SVC_RELIEF_X[0], D.BR_SVC_RELIEF_X[1],
             yin, yin + svc * D.BR_SVC_RELIEF,
             D.BR_SVC_RELIEF_Z[0] - 1, D.BR_SVC_RELIEF_Z[1])
    # (3) the EAR NOTCH: the inlet straddles z=0, so its plug dips ~3.6 mm into
    # the deck ear. Cut the ear over the same x band, out to BR_SVC_EAR_Y; the
    # pigtail then climbs over what is left of the ear on its way forward.
    p -= box(wx0, wx1, yin, svc * D.BR_SVC_EAR_Y, zd - 0.1, 1)
    # ---- deck top furniture -------------------------------------------------
    # battery aperture: the pack's lift-out path, envelope + 0.5 forward (its
    # aft face is the housing's front web, so it leaves through a slot its own
    # size). Anything else inside the bay -- the pack's own lead, off its +y end
    # -- comes straight up through here too.
    p -= box(D.BT_APER_X[0], D.BT_APER_X[1],
             -D.BT_APER_HY, D.BT_APER_HY, zd - 1, 1)
    for s in (1, -1):
        p -= box(D.BT_BELT_X[0], D.BT_BELT_X[1],
                 s * D.BT_BELT_SLOT_Y[0], s * D.BT_BELT_SLOT_Y[1], zd - 1, 1)
    # centre wire riser window into the channel below. Its FORWARD edge stops on
    # the channel's own front face: the front web's inner face is 0.59 mm short
    # of a nominal +11, and a +11 window bit a 0.6 x 24 ribbon out of the top of
    # that shear web (audit-caught in v5). Its AFT edge came forward to -8 in v6
    # to clear the gopro pad.
    p -= box(D.WIRE_WIN_X[0], cx1, -D.WIRE_WIN_HY, D.WIRE_WIN_HY, zd - 1, 1)
    # gopro_base pad: 4x M3 heat-sets in bosses hung UNDER the deck into the
    # wire channel. Under, not over: printed deck-top-down these rise with the
    # walls, where a boss on the deck TOP would hold the whole first layer in
    # the air (the 2026-07-15 audit). Thread = 5 deck + 4 boss = 9 > HEATSET_L.
    gx, gy = D.GP_SCREW_XY
    for sx in (D.GP_MOUNT_X + gx, D.GP_MOUNT_X - gx):
        for sy in (gy, -gy):
            p += cyl_z(D.GP_BOSS_D / 2, zd - D.GP_BOSS_H, zd, sx, sy)
            p -= cyl_z(D.HEATSET_D / 2, zd - D.GP_BOSS_H + 1, 0.01, sx, sy)
    return p


# ---------------------------------------------------------------- foot
def foot():
    """Sole plate + ankle-servo pocket. Local frame: ankle axis vertical
    projection at origin, +X = toe, z=0 at the sole bottom. Servo lies on its
    side (horn +Y), output end forward at +10.11, cable end at the heel.
    Retention: 4x M2.5 through the two rear tabs into the case holes + front
    end stop. Sole underside is FLAT (no bridge); glue a thin TPU/rubber pad on.
    v3 heel: the two retention tabs are tied into a heel BULKHEAD behind the
    servo (cable window on top), closing each free-standing blade into a
    channel section -- v2's lone blades snapped across layer lines under a
    lateral knock, which the aft gusset alone never addressed. Every added
    face is vertical, so nothing new bridges. Print: sole down. Qty 2.
    """
    x0, x1 = -D.FOOT_HEEL, D.FOOT_L - D.FOOT_HEEL   # -38 .. +58
    w = D.FOOT_W / 2
    # sole: FLAT underside (pad glued on), with BOTH ends ROUNDED in plan -- a
    # square corner is what catches on a door frame or a cable run, and it reads
    # blocky. Structure is unaffected: the aft buttress sits 10 mm forward of the
    # heel edge and the cable window stays inside the straight centre section.
    rt, rh = D.FOOT_TOE_R, D.FOOT_HEEL_R
    p = box(x0 + rh, x1 - rt, -w, w, 0, D.FOOT_T)
    p += box(x1 - rt, x1, -(w - rt), w - rt, 0, D.FOOT_T)
    p += box(x0, x0 + rh, -(w - rh), w - rh, 0, D.FOOT_T)
    for sy in (1, -1):
        p += cyl_z(rt, 0, D.FOOT_T, x1 - rt, sy * (w - rt))
        p += cyl_z(rh, 0, D.FOOT_T, x0 + rh, sy * (w - rh))
    # servo pocket (top)
    px0 = -D.SV_AXIS_FROM_REAR - D.FIT
    px1 = D.SV_AXIS_FROM_OUT_END + D.FIT
    py = D.SV_TOPFACE + D.FIT                        # 17.65
    p -= box(px0, px1, -py, py, D.FOOT_T - D.FOOT_POCKET_D, D.FOOT_T + 1)
    # relief slots under the shin-fork joint pads (O24 pads dip below the
    # ankle axis: 16.36 - 12 = 4.36 -> relieve to 3.5 for 0.8 clearance)
    for sy0, sy1 in ((17.4, 24.1), (-24.1, -17.4)):
        p -= box(-13, 13, sy0, sy1, 3.5, D.FOOT_T + 1)
    # rear retention tabs + heel bulkhead (U-channel) + front stop
    wx0, wx1 = D.FOOT_WALL_X
    bx0, bx1 = D.FOOT_BULK_X
    zp = D.FOOT_T - D.FOOT_POCKET_D                  # pocket floor, 4.0
    zr = D.FOOT_T                                    # gusset root = sole top, 6.0
    aL, aH = D.FOOT_WALL_GUSSET_AFT                  # aft buttress (heel side)
    # horn-side tab only here -- the idler tab is built at its SEAT face below
    # (2026-07-30), since the idler side of the case is not the mirror of this
    p += box(wx0, wx1, py, py + D.FOOT_WALL_T, zp, zp + D.FOOT_WALL_H)
    # bulkhead between the tab aft ends. Runs out to the tab OUTER faces, not
    # just the pocket width: stopping at +/-py made it die into the tabs' inner
    # faces, leaving each tab's outer 2.4 mm band tied only by the 2 mm aft
    # buttress wedge. Full width closes tabs + bulkhead into a continuous
    # channel section behind the servo (user, 2026-07-28). Still all vertical
    # faces, so nothing new bridges.
    p += box(bx0, bx1, -(py + D.FOOT_WALL_T), py + D.FOOT_WALL_T,
             zp, zp + D.FOOT_WALL_H)
    # one full-width aft buttress bracing tabs + bulkhead together: vertical
    # face against them, sloped face up (support-free), ends at the heel edge
    p += wedge_y([(wx0, zr), (wx0 - aL, zr), (wx0, zr + aH)],
                 -(py + D.FOOT_WALL_T), py + D.FOOT_WALL_T)
    # IDLER-SIDE tab, built at its SEAT face (2026-07-29, user: the same
    # servo-conformance treatment the leg_link grip plates got). The old tab
    # sat at -17.65 -- the MIRRORED pocket half-width -- but the idler side
    # of the case is not a mirror of the horn side: the real case face is
    # SV_IDLER_CASE_FACE (-14.75), so the two M2.5s at x -32.75 clamped
    # ~2.8 mm of air (vendor-solid placement check in CAD; the leg_link's
    # idler plate measured 2.60 mm of the same). The tab's inner face is the
    # leg_link's idler_seat (-14.90 = face - GRIP_SEAT_CLR), and it keeps its
    # ORIGINAL FOOT_WALL_T thickness measured off that seat (2026-07-30,
    # user): the first cut kept the old -20.05 outer face too, which made the
    # wall 5.15 mm and pushed the retention screws to M2.5x10. With the outer
    # face at -17.30, head-to-case is 2.55 and an M2.5x8 bites ~5.4 mm of
    # case -- 8 mm screws work universally. Pulling the wall in also gets it
    # clear of the shin fork's idler-plate sweep band (y -18..-21) entirely.
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR         # -14.90
    _ito = idler_seat - D.FOOT_WALL_T                           # -17.30 outer
    p += box(wx0, wx1, _ito, idler_seat, zp, zp + D.FOOT_WALL_H)
    # DETENT for the moulded back-cover PLATFORM (1.90 proud of the case
    # face, SV_IDLER_BOSS_*): its aft corner (foot x -29.51..-16.25,
    # z 5.66..27.06) overlaps the tab's forward end, and it sits BETWEEN the
    # screws and the seat, so it is relieved rather than seated on -- same
    # rule as the leg_link. Shaped as a WINDOW, open forward, out the top,
    # and THROUGH the wall, not the leg_link's closed pocket: the servo drops
    # in vertically, so the platform has to slide down past the tab on its
    # way in, and at 2.4 mm wall a closed relief would leave a 0.35 mm skin
    # -- so there is none. Open top + all-vertical faces still means nothing
    # overhangs printing sole-down. Bearing lands: the aft band
    # x -40..-29.91 (carrying both -32.75 screws) and the sliver below the
    # platform, z 4..5.26.
    _pz0 = D.ANKLE_AXIS_ABOVE_SOLE - D.SV_IDLER_BOSS_HW - D.RIB_RELIEF_CLR
    p -= box(D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR, wx1 + 0.1,
             _ito - 0.1, idler_seat + 0.01,
             _pz0, zp + D.FOOT_WALL_H + 0.1)
    # DETENT WINDOW for the horn-side RIB (bench truth 2026-07-30, user: "it
    # no longer fits in the foot"). The rib is 1.13 proud over x -34.26..-8.73
    # and the +Y tab face is only 0.30 off the case face, so ~0.8 mm of rib
    # lands on the tab. The old NOTE here trusted the placed vendor solid's
    # 0.2 mm "clearance" over the dims -- but that model's idler datums carry
    # a known ~0.2 error, and the bench has now voted with the dims. What
    # exposed it: seating the idler tab on the REAL case face (2026-07-29)
    # removed the 2.75 mm of -Y slack the old air-clamping tab left, which is
    # where the rib had been hiding. Same open-top WINDOW treatment as the
    # platform on the other tab: the servo drops in vertically, the rib sweeps
    # the whole tab band above its seat, so the relief must run out the top --
    # a closed pocket passes nothing. Floor measured off the WALL face (not
    # the case face): with the case leaned fully +Y against the wall the rib
    # still keeps its 0.3. Skin left outboard: 0.97 mm, the leg_link class.
    # The z 26.61 horn screw whose bearing land this consumes is deleted at
    # the retention rows below. All-vertical faces + open top: nothing new
    # bridges printing sole-down.
    _rz0 = D.ANKLE_AXIS_ABOVE_SOLE - D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR
    p -= box(-D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR, wx1 + 0.1,
             py - 0.01, py + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
             _rz0, zp + D.FOOT_WALL_H + 0.1)
    # NO cable window. It used to run out through the bulkhead and buttress on
    # the premise that the servo lead leaves the rear END face -- it does not.
    # SV_CONN (measured off cad/vendor/ST3215.step) puts the sockets on the
    # IDLER-SIDE face, x -11.75..-16.35, so the lead exits sideways a third of
    # the way along the foot and never comes near the heel. With nothing to pass
    # through, the slot was just a hole in the one structure meant to tie the two
    # retention tabs together, so the heel is now SOLID (user, 2026-07-28).
    # front end stop: +/-15 spans the whole 24.72 case but stops 3.0 short of
    # the fork idler plate's y=-18 plane -- full pocket width (+/-17.65) left
    # only 0.35 to the fork's front corner at ankle +40 (audit 2026-07-28)
    p += box(px1, px1 + D.WALL, -15.0, 15.0, zp, zp + 8)
    # FRONT retention bosses at the case's 8.30 hole row -- the row nearest the
    # output end, i.e. right beside the rotor. Without them the case is held only
    # at the heel tabs and levers against the sole; these pick up the LOW lateral
    # hole (z 6.11) so the tie-down is close to the sole where the flex is.
    # Deliberately SHORT (top 10.0 vs the heel tabs' 30.0): the shin fork sweeps
    # this region, and a full-height wall here would not survive ankle travel.
    # HORN SIDE ONLY. The idler side cannot take one: the shin fork's idler
    # plate occupies y -18..-21 through the whole ankle sweep, which is exactly
    # where a -Y boss would stand (47.6 mm3 at ankle -40, and fouling even at
    # neutral). On the horn side the fork clears from y 20.45, leaving 0.4 mm.
    # HORN SIDE ONLY -- see the note above: the idler side is measured, not
    # assumed. A mirrored boss clears the shin outright (0.00 mm3 at every ankle
    # angle once the rotor arc trims it), but the CLEARANCE is the problem: the
    # gap to the shin is set purely by FOOT_ROTOR_CLEAR_R, and reaching the
    # 0.40 mm this joint already runs at needs R ~10.9, which cuts 0.9 mm into
    # the countersink mouth (centre 13.19 from the axis, flat head radius 2.35)
    # and unseats the screw head. R 10.3 keeps the head but leaves only 0.11 mm
    # to the shin. Head seat or shin clearance -- not both, so it stays off.
    # BOTH SIDES. Note what the -Y boss shares space with: SV_CONN puts the
    # servo's sockets on the IDLER FACE, trench band x -11.75..-16.35 (measured
    # off cad/vendor/ST3215.step -- NOT the heel end face, whatever the old cable
    # comments said). The boss's rear corner reaches x -12.80, i.e. 1.05 mm into
    # that band, and no size escapes it: the countersink mouth must reach -11.30,
    # so clearing the trench needs HW < 3.45 while keeping a rim round the mouth
    # needs HW > 3.0. USER CALL 2026-07-28: keep the screw -- the lead is
    # flexible and will route around the boss. Plug the connector BEFORE the foot
    # goes on; there is no room to work it in afterwards.
    fb = -D.FOOT_FRONT_BOSS_X
    # Each side seats on ITS OWN case face -- the servo is not symmetric about
    # the output axis, and this boss used to be mirrored (+/-py) as if it were.
    # BENCH 2026-07-30 (user, on the placed vendor solid): "the highlighted screw
    # attachment point is not flush with the servo body". It was not: the +Y boss
    # sat 0.30 off the horn face (the FIT clearance, correct), but the -Y boss sat
    # at -17.65 against a case face at -14.75 -- 2.90 mm of air, so its screw
    # clamped nothing and the boss could not pull up against the case. Same defect
    # class as the leg_link idler grip plate (2.60 mm) and the yaw_carrier rear
    # wall. The rear idler tab above was re-seated on 2026-07-29; this one was
    # missed because it is built in a mirrored loop.
    _fb_faces = {1: (py, py + D.FOOT_WALL_T),          # horn: seat 17.65
                 -1: (idler_seat, _ito)}               # idler: seat -14.90
    for s in (1, -1):
        _fin, _fout = _fb_faces[s]
        p += box(fb - D.FOOT_FRONT_BOSS_HW, fb + D.FOOT_FRONT_BOSS_HW,
                 min(_fin, _fout), max(_fin, _fout),
                 D.FOOT_PAD_RELIEF_Z, D.FOOT_FRONT_BOSS_TOP)
    # ...and carve its inner corner back to follow the ROTOR. The horn disc
    # sweeps O19.2 about the ankle axis from y 18.35 out, so any boss material
    # out there has to stay off that circle -- the first cut of this boss buried
    # 13.7 mm3 in it. A matching arc is the detent; the screw at (x -8.30,
    # z 6.11) sits at r 13.2, well outside it.
    p -= cyl_y(D.FOOT_ROTOR_CLEAR_R,
               -(py + D.FOOT_WALL_T + 0.01), py + D.FOOT_WALL_T + 0.01,
               0, D.ANKLE_Z - D.TPU_PROUD)
    for s in (1, -1):
        _fin, _fout = _fb_faces[s]
        p -= teardrop_y(D.CASE_SCREW_CLEAR / 2,
                        min(_fin, _fout) - 1, max(_fin, _fout) + 1,
                        fb, zp + 2.11)
        p -= csk_y(fb, zp + 2.11, _fout, s)
    # retention screw holes: horn face row 29.0 (+Y), idler face row 32.75 (-Y);
    # M2.5 FLAT-head self-tap (bench truth 2026-07-28: the case holes take
    # M2.5, not M3); teardropped (horizontal bores printed sole-down, peak +z).
    # COUNTERSUNK flush: a proud pan head on the +Y tab was exactly tangent to
    # the shin fork blade at ankle -40 (audit 2026-07-28) -- same class as the
    # leg_link skew. The divots stay: the driver still needs them (LOW row).
    for zh in (2.11, 22.61):
        # idler row: countersunk in the relocated tab's outer face (-17.30),
        # bore straight through to the seat -- M2.5x8, like everywhere else
        p -= teardrop_y(D.CASE_SCREW_CLEAR / 2, _ito - 1,
                        idler_seat + 1, -32.75, zp + zh)
        p -= csk_y(-32.75, zp + zh, _ito, -1)
    # horn row: LOW screw ONLY (2026-07-30). The z 26.61 seat fell to the rib
    # window above -- 0.97 mm of skin behind a 1.25 mm countersink is no seat
    # -- and every alternative kept the rib out instead: seating ON the rib
    # clamps ~1.3 mm of air at the case face beside it (the defect class this
    # week has been about deleting), and the horn face has no other hole row
    # inside the tab's x-span. Retention is now 5 screws -- idler low+high
    # biting 5.4 mm on the real seat, this one, both front bosses -- plus the
    # front stop and the closed heel channel.
    p -= teardrop_y(D.CASE_SCREW_CLEAR / 2, py - 1, py + D.FOOT_WALL_T + 1,
                    -29.0, zp + 2.11)
    p -= csk_y(-29.0, zp + 2.11, py + D.FOOT_WALL_T, +1)
    # driver-access DIVOTS for the LOW retention row (z = zp+2.11 = 6.11, at the
    # sole top): the sole shelf outboard of the tabs blocks the head + Y-driver
    # (user report / probe 2026-07-23). Relieve the shelf TOP over each low screw
    # from the tab outer face out through the sole edge, down to FOOT_DIVOT_FLOOR
    # -- the pad still bonds to the full z=0 underside. The HIGH row is clear.
    # CO-AXIAL WITH THE SCREW (user 2026-07-28): bore the relief along the screw
    # axis, head radius + margin, from the tab outer face out through the sole
    # edge. The screw sits at z 6.11 and the sole top is 6.0, so the channel only
    # bites 3.19 mm into the shelf and leaves 2.81 mm under it. The FRONT boss
    # screw needs one too -- it is the same head at the same height, just further
    # forward. Only the LOW row needs relieving; the high row clears the sole.
    # THE DIVOT MUST START AT THE HEAD-SEAT FACE (2026-07-30). The idler tab's
    # outer face moved from -20.05 to _ito (-17.30) when the tab was rebuilt on
    # its real seat, and this loop still opened its channel at the OLD -20.05 --
    # leaving 2.75 mm of un-relieved sole shelf standing directly in front of
    # the -32.75 low countersink. Same class of miss as the buried grip
    # countersinks (a4ce69e), and now caught by check_assembly's screw
    # insertion-path sweep (33.3 mm3 before this line was fixed).
    # Each entry carries its OWN head-seat face, because they are not all at
    # +/-(py + FOOT_WALL_T): everything on the IDLER side seats at _ito (-17.30),
    # the rear tab since 2026-07-29 and the front boss since 2026-07-30.
    for xh, yface in ((-29.0, py + D.FOOT_WALL_T),
                      (-32.75, _ito),
                      (-D.FOOT_FRONT_BOSS_X, py + D.FOOT_WALL_T),
                      (-D.FOOT_FRONT_BOSS_X, _ito)):
        ysgn = 1 if yface > 0 else -1
        yedge = ysgn * (D.FOOT_W / 2 + 1)            # just past the sole edge
        p -= cyl_y(D.FOOT_DIVOT_R, min(yface, yedge), max(yface, yedge),
                   xh, zp + 2.11)
    # DETENT for the ankle servo's horn-side RIB (same defect as leg_link, found
    # 2026-07-28 once the mock carried the rib). The servo lies on its side here,
    # so Rot(0,90,0) maps the rib to x = -34.26..-8.73, y from SV_TOPFACE up
    # 1.13, z about the servo axis at ANKLE_Z - TPU_PROUD. It lands on the +Y
    # retention tab's inner face -- 101.7 mm3 of it -- and holds the case off.
    # The retention screws sit at z 6.11 and 26.61, clear of the 8.94..23.78 rib
    # band by 2.4 mm either side, so both screw seats keep full tab thickness.
    zc = D.ANKLE_Z - D.TPU_PROUD
    p -= box(-D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR,
             -D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR,
             D.SV_TOPFACE - 0.01,
             D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
             zc - D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR,
             zc + D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR)
    return p


# ---------------------------------------------------------------- tower
def tower():
    """Electronics tower. The GoPro base bolts on top (4x M3 into bosses under
    the plate); the driver board hangs INSIDE, face down on standoffs under the
    top plate (screwed M2.5 from below). The 3S battery swaps tool-free: it
    tilt-loads through a window in the -x wall onto the pelvis deck, seats
    against far-wall rail stubs (dash x-loads) between the feet-tab gussets
    (y-location), parked by two corner detents (x slide while the belt is
    off); a 20 mm hook-loop belt around the tower (guide ribs set its height)
    closes the window -- the belt, not the wall, is the tumble retention --
    and a ribbon under the pack is the pull-tab. Feet tabs screw down into
    the deck heat-sets (access holes in the top plate). Local frame: z=0 at
    deck top. Print: upside down (top plate on the bed), fully support-free:
    gussets/rail stubs/corner detents are true >=45 deg wedges and the window
    is open to the deck (a sill would be a 70 mm member printing in mid-air
    -- 2026-07-16 slice reviews). Only ceilings left: the 4 Ø6.6 feet-screw
    counterbores (standard short bridges).
    """
    hx = D.TOWER_W / 2                               # walls are X-normal, 21
    hy = D.TOWER_L / 2                               # spans Y like the deck, 48
    zt0, zt1 = D.TOWER_H - D.TOWER_TOP_T, D.TOWER_H
    p = box(-hx, hx, -hy, hy, zt0, zt1)
    for s in (1, -1):
        p += box(s * (hx - D.WALL), s * hx, -hy, hy, 0, zt0 + 1)
        # feet tabs (inward) + 45 deg gusset wedge to the wall (sized for the
        # 154 g camera cantilevered ~85 mm above: a 10 g side hit ~ 1.3 N*m
        # -> ~25 N per screw, well inside heat-set / tab capacity with gussets)
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            p += box(s * (hx - D.WALL - 8.5), s * hx, sy - 6, sy + 6, 0, 4)
            # gusset: a real 46 deg wedge, tab inner edge to the wall. Upside
            # down its underside is the hypotenuse (self-supporting); the old
            # stepped box left flat 6 and 2.5 mm ledges drooping over the
            # interior (first-print review 2026-07-16).
            p += wedge_y([(s * (hx - D.WALL - 8.5), 4.0),
                          (s * (hx - D.WALL), 4.0),
                          (s * (hx - D.WALL), 13.0)], sy - 6, sy + 6)
            p -= cyl_z(D.M3_CLEAR / 2, -1, 3.9, s * D.TOWER_FOOT_X, sy)
            # head + driver well through the wedge: screw head seats on the
            # tab itself (Ø6.6 clears an M3 button/socket head)
            p -= cyl_z(3.3, 3.9, 13.1, s * D.TOWER_FOOT_X, sy)
            p -= cyl_z(3.2, zt0 - 6, zt1 + 1, s * D.TOWER_FOOT_X, sy)  # driver access
    # battery window in the -x wall: OPEN to the deck, posts at the ends keep
    # the feet tabs. There is deliberately no sill: a full-width lip prints
    # as a 70 mm member 41 mm up in mid-air (the bridging/support saga of the
    # 2026-07-16 slice reviews), and retention was never its job -- the
    # hook-loop belt closes the window for tumbles and dash loads. What the
    # sill did contribute (parking the pack against -x slide while the belt
    # is off) is done by two 2.5 mm corner detents: 45 deg wedges growing off
    # the window posts, self-supporting in the inverted print, and the pack
    # tilts over them exactly as it did over the old sill.
    bw = D.BATT[0] / 2 + 1.0
    p -= box(-hx - 1, -hx + D.WALL + 1, -bw, bw, 0, D.BATT[2] + 2.5)
    for sgn in (1, -1):
        prof = [(sgn * bw, 0.0), (sgn * bw, 2.5), (sgn * (bw - 2.5), 0.0)]
        det = extrude(Plane.YZ * Polygon(*prof, align=None), amount=D.WALL)
        p += Pos(-hx - det.bounding_box().min.X, 0, 0) * det
    # The far-wall rail stubs are GONE (2026-07-28). They existed to seat the
    # pack's inner face against the +x wall, and the board partition below now
    # sits exactly on that plane -- BATT_SEAT_X + BATT[1] == BOARD_GD_PARTITION_X
    # by construction, so the pack seats on a full-length wall instead of two
    # 12 mm stubs. Asserted rather than commented, because if either constant
    # moves independently the pack quietly loses its seat.
    seat_in = D.BATT_SEAT_X + D.BATT[1]              # pack inner (+x) face
    assert abs(seat_in - D.BOARD_GD_PARTITION_X) < 1e-9, (
        f"pack inner face {seat_in} no longer matches the board partition "
        f"{D.BOARD_GD_PARTITION_X} -- one of BATT_SEAT_X / BATT[1] / "
        f"BOARD_GD_PARTITION_X moved without the others")
    # belt guide ribs: +x wall full-width, -x wall on the window posts. Each
    # rib carries a 45 deg chamfer wedge on its model-TOP face: upside down
    # that face is the rib's print-underside, and a square 1.5 mm ledge
    # droops (check_printability LEDGE) -- the wedge makes it self-supporting.
    for rz in (D.BATT[2] - 8, D.BATT[2] + 3):
        p += box(hx, hx + 1.5, -22, 22, rz, rz + 1.5)
        p += wedge_y([(hx, rz + 1.5), (hx + 1.5, rz + 1.5), (hx, rz + 3.0)],
                     -22, 22)
        for sy in (1, -1):
            p += box(-hx - 1.5, -hx, sy * (bw + 1), sy * (hy - 1), rz, rz + 1.5)
            p += wedge_y([(-hx, rz + 1.5), (-hx - 1.5, rz + 1.5),
                          (-hx, rz + 3.0)], sy * (bw + 1), sy * (hy - 1))
    # ---- driver board: UPRIGHT against the +x side, 2026-07-28 --------------
    # The General Driver board is 65 x 65 and will not lie down anywhere on this
    # torso (see BOARD_GD_* in dimensions.py), so it stands on edge. The
    # partition below does two jobs at once: it is the pack's +x seat (replacing
    # the rail stubs, whose space the board now occupies) and it is the board's
    # mounting face. It also ties the two long walls together, which the old
    # open bay never did -- welcome at 75 mm tall.
    px0 = D.BOARD_GD_PARTITION_X
    px1 = px0 + D.WALL
    bz0 = D.BOARD_GD_CZ - D.BOARD_GD_OUTLINE[1] / 2      # 2.5
    bz1 = D.BOARD_GD_CZ + D.BOARD_GD_OUTLINE[1] / 2      # 67.5
    # partition spans the board's height and a little either side; it stops
    # short of the top plate so the bay still vents and wires can pass over.
    p += box(px0, px1, -hy + D.WALL, hy - D.WALL, 0, bz1 + 2)
    # lighten it: the partition is a shear web, not a pressure vessel. Windows
    # sit between the four screw bosses and are inset from every edge.
    for wy in (-D.BOARD_GD_SCREW_DY / 2, D.BOARD_GD_SCREW_DY / 2):
        p -= box(px0 - 1, px1 + 1, wy - 9, wy + 9,
                 D.BOARD_GD_CZ - 12, D.BOARD_GD_CZ + 12)
    # four bosses off the partition's +x face -- M2.5 self-tap into the board's
    # O3 holes. Screws go in along +x, so these print as horizontal cylinders;
    # they are short (4.0) and land on a vertical face, which is fine inverted.
    for sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY):
        for sz in (D.BOARD_GD_CZ + D.BOARD_GD_SCREW_DZ,
                   D.BOARD_GD_CZ - D.BOARD_GD_SCREW_DZ):
            p += cyl_x(3.5, px1, px1 + D.BOARD_GD_STANDOFF, sy, sz)
            p -= cyl_x(D.M25_TAP / 2, px0 - 1,
                       px1 + D.BOARD_GD_STANDOFF + 1, sy, sz)
            # driver ACCESS through the +x wall (2026-07-30). These four screws
            # run along +x with their heads on the board's +x face (x 15.23),
            # and the +x wall stands at 18.40 -- a 3.17 mm gap for an 8 mm
            # screw plus a PH1 bit. The board simply could not be fastened; the
            # 2026-07-28 move from face-down-under-the-top-plate to upright
            # took the old runway away with it. Found by check_assembly's screw
            # insertion-path sweep (49 mm3 of wall per screw). Bore the wall
            # coaxially so the screw is offered in from OUTSIDE, exactly like
            # the feet bolts' O6.4 wells above; teardropped peak model -z
            # (printed top-plate-down, so -z is print-up) since these are
            # horizontal bores. Clear of the belt ribs (|y| <= 22) and the
            # feet-tab gussets (|y| >= 36) at y +/-29.
            p -= teardrop_x(D.M25_HEAD_D / 2 + 0.2, hx - D.WALL - 1, hx + 1,
                            sy, sz, roll=180)
    # GoPro base screw bosses (M3 self-tap from above, through-pilots)
    gx, gy = D.GP_SCREW_XY
    for sx in (gx, -gx):
        for sy in (gy, -gy):
            p += cyl_z(4.0, zt0 - 3, zt0, sx, sy)
            p -= cyl_z(D.M3_ST_PILOT / 2, zt0 - 4, zt1 + 1, sx, sy)
    # wire / vent holes (clear of the 30 x 24 GoPro base footprint)
    for sy in (20, -20):
        p -= cyl_z(5, zt0 - 1, zt1 + 1, 0, sy)
    return p


# ---------------------------------------------------------------- gopro_base
def gopro_base():
    """GoPro three-prong mount base for the camera's folding two-finger mount.
    Prongs stacked along Y => lens axis fore-aft (X). Dimensions follow the
    GoProScad standard (see dimensions.py). Bolts to the tower top with 4x M3
    self-tappers; the stock M5 thumbscrew clamps the camera. Local frame: z=0
    at the base bottom (tower top plane). Print: base down, prongs up (this is
    the standard, proven orientation for printed GoPro mounts; use PETG or
    100%-infill PLA). Separate part on purpose: it is the crash fuse.
    """
    hx, hy = D.GP_BASE_X / 2, D.GP_BASE_Y / 2
    zb = D.GP_BASE_T
    zh = zb + D.GP_HOLE_H                            # M5 hole center, 13.5
    p = box(-hx, hx, -hy, hy, 0, zb)
    pitch = D.GP_PRONG_T + D.GP_SLOT                 # 6.2
    for cy in (-pitch, 0.0, pitch):
        y0, y1 = cy - D.GP_PRONG_T / 2, cy + D.GP_PRONG_T / 2
        p += box(-D.GP_PRONG_OD / 2, D.GP_PRONG_OD / 2, y0, y1, zb, zh)
        p += cyl_y(D.GP_PRONG_OD / 2, y0, y1, 0, zh)
    if D.GP_HOLE_TEARDROP:
        # full peak: proven clearance (printed), keeps the part byte-stable
        p -= teardrop_y(D.GP_HOLE_D / 2, -hy - 1, hy + 1, 0, zh, full=True)
    else:
        p -= cyl_y(D.GP_HOLE_D / 2, -hy - 1, hy + 1, 0, zh)
    gx, gy = D.GP_SCREW_XY
    for sx in (gx, -gx):
        for sy in (gy, -gy):
            p -= cyl_z(D.M3_CLEAR / 2, -1, zb + 1, sx, sy)
    return p


# ---------------------------------------------------------------- imu_carrier
def imu_carrier():
    """GY-BNO08X carrier: a plate between the tower top and gopro_base, clamped
    by the SAME 4x M3x12, with a rear tongue carrying the IMU.

    Pocket LOCATES, screws CLAMP. The pocket is cut to the board's outline
    (15.5 x 25.4, the one dimension confirmed twice over) and fixes x, y and
    rotation; two M2.5 self-tappers then hold it down hard, which is what an
    IMU needs -- any shift corrupts the gravity vector the policy reads.

    The screw pattern was recovered from a photo by perspective-correcting it
    against the known outline (see dimensions.py), and carries ~0.25 mm of
    uncertainty. That is fine BECAUSE of the split above: the board's own holes
    are O3.15 on an M2.5, so 0.3 mm of radial slack absorbs the error. Locating
    on this pattern instead would not have been safe.

    The tongue is thicker than the pad (IMU_TONGUE_T vs IMU_CARRIER_T) so the
    pocket floor is still 3.65 mm -- enough to tap. It grows UPWARD so the part
    still prints flat on its underside, pocket up, support-free.

    A through-slot under the pad row runs the full length and out the rear: it
    clears the soldered pin tails on the board's underside (the header can face
    either way) and is the wire exit down the open rear of the tower.

    Local frame: z=0 on the tower top plane. ~6 g PETG.
    """
    hx, hy = D.GP_BASE_X / 2, D.GP_BASE_Y / 2        # gopro pad 15, 12
    T = D.IMU_CARRIER_T
    TT = D.IMU_TONGUE_T
    px, py, pt = D.IMU_PCB
    c = D.IMU_POCKET_CLEAR
    ox, oy = px / 2 + c, py / 2 + c                  # pocket half-extents
    wall = 1.6
    tx = ox + wall                                   # tongue half-width
    ty = D.IMU_CY - oy - wall                        # tongue rear edge

    p = box(-hx, hx, -hy, hy, 0, T)                  # pad under gopro_base
    p += box(-tx, tx, ty, -hy + 2, 0, T)             # tongue at pad thickness
    # Only the REAR of the tongue is raised to IMU_TONGUE_T. Raising all of it
    # drove 2.5 mm of material up into gopro_base, which sits on the pad from
    # y = -12 forward (caught by an explicit interference check, 95.5 mm3).
    p += box(-tx, tx, ty, -hy - 0.5, 0, TT)          # raised pocket section
    gx, gy = D.GP_SCREW_XY
    for sx in (gx, -gx):                             # shared M3x12 through-holes
        for sy in (gy, -gy):
            p -= cyl_z(D.M3_CLEAR / 2, -1, T + 1, sx, sy)

    floor = TT - D.IMU_POCKET_D
    p -= box(-ox, ox, D.IMU_CY - oy, D.IMU_CY + oy, floor, TT + 1)   # pocket

    # Pin-tail relief + wire exit: through the floor, and open at the rear so
    # nothing catches whichever way the header faces.
    p -= box(-ox, -ox + D.IMU_SOLDER_SLOT, ty - 1, D.IMU_CY + oy - 0.8, -1,
             floor + 0.1)

    # M2.5 self-tap pilots, through the floor so a slightly long screw can
    # protrude rather than bottom out and lift the board.
    for sy in (D.IMU_CY + D.IMU_SCREW_DY, D.IMU_CY - D.IMU_SCREW_DY):
        p -= cyl_z(D.M25_TAP / 2, -1, floor + 0.1, D.IMU_SCREW_X, sy)
    return p


# ---------------------------------------------------------------- build all
# v4 (2026-08-04): tower / gopro_base / imu_carrier dropped out of the build.
# They are still defined above (legacy), but nothing prints them -- the tower's
# two jobs moved to battery_guard and board_frame, and the camera/IMU head is
# waiting on its own bolt-on. Anything that still wants a tower STL has to say
# so explicitly rather than getting one by default.
PARTS = [
    # name, builder, qty, print orientation note
    ("pelvis", pelvis, 1,
     "upside down: deck top on bed, housing walls rise 36 mm"),
    ("yaw_carrier", yaw_carrier, 2, "horn-plate face on bed, bay walls rise "
     "(print yaw_carrier_print.stl: break-away breakout in the cable window)"),
    ("yoke_roll", yoke_roll, 2,
     "WALL: on edge, arms along the bed -- SLICER SUPPORTS ON"),
    ("yoke_pitch", yoke_pitch, 2,
     "WALL: on its back like leg_link -- SLICER SUPPORTS ON + brim"),
    ("leg_link", leg_link, 4, "on its back: web face on bed "
     "(print leg_link_print.stl: break-away fins under the fork slabs)"),
    ("foot", foot, 2, "sole down"),
    # v6: battery_tray and board_frame are GONE -- folded into pelvis(). The
    # only bolt-on left is gopro_base, and that one is deliberate (crash fuse).
    ("gopro_base", gopro_base, 1, "base down, prongs up (PETG or 100% infill)"),
]


# v4 torso/CG, measured off commit 0933847 with the identical method (see the
# CG block in main): the number v5 has to beat, kept here so the comparison
# survives the code that produced it.
V4_CG, V4_TORSO_CG = 192.7, 335.3
# ...and v5 (commit f46229b) by the same method, for the v6 comparison.
V5_CG, V5_TORSO_CG, V5_TORSO_M = 185.4, 320.1, 359.0


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(STEP_OUT, exist_ok=True)
    rows, print_mass = [], 0.0
    for name, fn, qty, orient in PARTS:
        part = fn()
        path = os.path.join(OUT, f"{name}.stl")
        export_stl(part, path)
        # exact BREP solid for FreeCAD/Onshape, same solid, same run
        export_step(part, os.path.join(STEP_OUT, f"{name}.step"))
        if name == "leg_link":       # print variant with break-away fins; the
            lp = leg_link(print_fins=True)           # plain STL stays clean for
            export_stl(lp, os.path.join(OUT, "leg_link_print.stl"))  # sim meshes
            export_step(lp, os.path.join(STEP_OUT, "leg_link_print.step"))
        if name == "yaw_carrier":    # ...ditto: the breakout that holds up the
            yp = yaw_carrier(print_fins=True)        # connector-window bar
            export_stl(yp, os.path.join(OUT, "yaw_carrier_print.stl"))
            export_step(yp, os.path.join(STEP_OUT, "yaw_carrier_print.step"))
        # yoke_roll / yoke_pitch used to export a _print variant with modelled
        # break-away fins. Deleted 2026-07-30 -- they print with SLICER
        # supports now, so the one STL above is what goes to the slicer.
        bb = part.bounding_box()
        dims = sorted((bb.size.X, bb.size.Y, bb.size.Z))
        fits = dims[0] <= 250 and dims[1] <= D.BED and dims[2] <= D.BED
        vol = part.volume / 1000.0                     # cm^3
        mass = part.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        print_mass += mass * qty
        rows.append((name, qty, bb.size, vol, mass, fits, orient))
        print(f"{name:13s} x{qty}  bbox {bb.size.X:6.1f} x {bb.size.Y:6.1f} x "
              f"{bb.size.Z:6.1f} mm  vol {vol:6.1f} cm3  ~{mass:5.1f} g  "
              f"{'BED-OK' if fits else '** TOO BIG **'}  [{orient}]")

    print(f"\nexported {len(PARTS)} parts + 2 print variants -> "
          f"{os.path.relpath(OUT)}/*.stl AND {os.path.relpath(STEP_OUT)}/*.step")

    # battery = worst case of the 3S 850 XT30 field the bay now fits (~80 g,
    # see dimensions.BATT); the flat Zeee is 74 g. Its pigtail
    # lives in the wiring/misc bucket, not here (issue #2).
    # sole pads: 2x 106 x 46 cut from 1/16" self-adhesive silicone sheet
    # (B0FJ8TBMQK); ~9.8 g each
    servos, batt, board, fasteners, tpu = (10 * D.SERVO_MASS, D.BATT_PACK_MASS,
                                           20.0, 55.0, 19.6)   # v3yaw: 10 servos
    total = print_mass + servos + batt + board + fasteners + tpu
    print(f"\nprinted plastic ~{print_mass:.0f} g   servos {servos:.0f} g (10)   "
          f"battery {batt:.0f} g   board {board:.0f} g   fasteners {fasteners:.0f} g"
          f"   TPU pads {tpu:.0f} g")
    print(f"TOTAL ROBOT ~{total:.0f} g   (+{D.CAM_MASS:.0f} g GoPro MAX = "
          f"{total + D.CAM_MASS:.0f} g -- v6 puts the camera back, on the deck)")
    # v6: nothing printed stands above the deck. The tallest thing on the robot
    # is the camera; the tallest STRUCTURE is the board's own top edge.
    top_v6 = D.DECK_BOT_Z_YAW + D.DECK_T + D.BR_TOP_Z
    print(f"\nheights: ankle {D.ANKLE_Z:.1f}  knee {D.KNEE_Z:.1f}  "
          f"hip-pitch {D.HIP_PITCH_Z:.1f}  hip-roll {D.HIP_ROLL_Z:.1f}  "
          f"hip-yaw {D.HIP_YAW_Z:.1f}  deck-bot(yaw) {D.DECK_BOT_Z_YAW:.1f}  "
          f"deck-top(yaw) {D.DECK_BOT_Z_YAW + D.DECK_T:.1f}  "
          f"board top edge {top_v6:.1f} mm  "
          f"(legacy tower stack: TOP_Z_YAW {D.TOP_Z_YAW:.1f})")

    # segment mass rollup for the sim update
    m = {n: r[4] for n, r in ((row[0], row) for row in rows)}
    # v6: the torso is ONE printed part again (plus gopro_base). battery_tray
    # and board_frame folded into pelvis(); the camera is BACK, on the deck.
    # Body bottom must match check_assembly.camera_mock(), which was corrected
    # on 2026-08-04: the prongs' round tops reach hole_z + GP_PRONG_OD/2, so
    # the tower-era "+ 6.0" used here swallowed 1.5 mm of every prong. This
    # line kept the stale form until 2026-08-06 and read 2 mm low.
    cam_z = (D.GP_BASE_T + D.GP_HOLE_H + D.GP_PRONG_OD / 2 + 0.5
             + D.CAM_BODY[2] / 2)                                  # 56.0 over deck
    seg = {
        "torso": m["pelvis"] + m["gopro_base"] + 2 * D.SERVO_MASS
                 + batt + board + 27,
        "yaw carrier": m["yaw_carrier"] + D.SERVO_MASS + 5,
        "hip(roll link)": m["yoke_roll"] + m["yoke_pitch"] + 5,
        "thigh": m["leg_link"] + D.SERVO_MASS + 4,
        "shin": m["leg_link"] + D.SERVO_MASS + 4,
        "foot": m["foot"] + D.SERVO_MASS + tpu / 2 + 4,
    }
    print("\nsegment masses for sim v6 (g):  [camera listed separately]")
    for k, v in seg.items():
        print(f"  {k:16s} {v:6.1f}  (x2 legs)" if k != "torso" else
              f"  {k:16s} {v:6.1f}")
    print(f"  + camera {D.CAM_MASS:.0f} g at {cam_z:.0f} mm above the deck")
    print(f"  yaw stack drop {D.YAW_STACK_DROP:.1f} mm  (study estimated ~28)")
    print(f"  stance HIP_SEP {D.HIP_SEP:.0f} mm, deck "
          f"{D.DECK_FWD_X - D.DECK_AFT_X:.0f} x {D.DECK_L:.0f} -- plant "
          f"changes: rebuild sim/build_v2_inertia.py and retrain")

    # ---- standing CG, from the REAL centroids of the exported solids (v5's
    # method, so the v4/v5/v6 numbers are comparable). Deck top is the datum;
    # leg segments keep their measured neutral-stance heights.
    dt = D.DECK_BOT_Z_YAW + D.DECK_T                 # deck top above ground
    torso = [
        (m["pelvis"], dt + pelvis().center().Z),
        (m["gopro_base"], dt + D.GP_BASE_T / 2 + 6),
        # the pack rests on the seat chamfers, so a REAL pack's centre follows
        # BT_PACK_BOT, not the envelope
        (batt, dt + D.BT_PACK_BOT + D.BATT_PACK[2] / 2),
        (board, dt + D.BR_CZ),                       # PCB centre
        (2 * D.SERVO_MASS, dt + (-D.DECK_T + D.YAW_CASE_BOT_Z) / 2),
        (27, dt - 2),                                # wiring / belt / misc
    ]
    legs = [
        (2 * (m["yaw_carrier"] + D.SERVO_MASS + 5), D.HIP_ROLL_Z + 20),
        (2 * (m["yoke_roll"] + m["yoke_pitch"] + 5), D.HIP_ROLL_Z - 25),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), D.HIP_PITCH_Z - 45),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), D.KNEE_Z - 45),
        (2 * (m["foot"] + D.SERVO_MASS + tpu / 2 + 4), 12),
    ]
    items = torso + legs
    mt = sum(w for w, _ in items)
    cg = sum(w * z for w, z in items) / mt
    tm = sum(w for w, _ in torso)
    tcg = sum(w * z for w, z in torso) / tm
    cg_cam = ((sum(w * z for w, z in items) + D.CAM_MASS * (dt + cam_z))
              / (mt + D.CAM_MASS))
    print(f"\nstanding CG ~{cg:.1f} mm   (torso {tm:.0f} g with its own CG at "
          f"{tcg:.1f} mm, i.e. {tcg - dt:+.1f} vs the deck top at {dt:.0f})")
    print(f"  v5 (commit f46229b), same method: CG {V5_CG:.1f} mm, torso "
          f"{V5_TORSO_M:.0f} g at {V5_TORSO_CG:.1f} ({V5_TORSO_CG - dt:+.1f} vs "
          f"deck);  v4 (0933847): CG {V4_CG:.1f} mm")
    print(f"  -> v6 moves the torso CG {tcg - V5_TORSO_CG:+.1f} mm and the whole "
          f"robot {cg - V5_CG:+.1f} mm vs v5, on {V5_TORSO_M - tm:.0f} g less torso")
    print(f"  WITH the {D.CAM_MASS:.0f} g camera back on the roof: CG "
          f"{cg_cam:.1f} mm ({cg_cam - cg:+.1f} mm) -- the camera is 12 % of the "
          f"robot at {cam_z:.0f} mm above the deck, and it is the single worst "
          f"item on the machine for balance")
    print(f"  above-deck structure: 0 mm printed (the board's own top edge "
          f"stands {D.BR_TOP_Z:.1f}) -- v5 26.6, v4 74.0")


def export_assemblies():
    """assembly.step + assembly_full.step, from the geometry just exported.

    Chained here (user, 2026-07-27) for the same reason STEP was: they are
    built from the same parts and go stale the moment someone changes a
    dimension and only runs parts.py. Imported lazily -- both modules import
    parts, so a top-level import would be circular.

    BLOCKED AS OF 2026-08-04 (pelvis v6): export_assembly.py,
    export_assembly_full.py, check_assembly.py and fasteners.py still place the
    v5 torso -- parts.battery_tray(), parts.board_frame(), D.BT_FLOOR_*/PAD_*/
    SCREW_* and the whole v5 D.BF_* set -- none of which exist any more. v6 was
    scoped to dimensions.py and parts.py, so those four are a follow-up edit.
    Rather than let this raise a bare AttributeError after a successful part
    export, say what is wrong and stop.
    """
    try:
        import export_assembly
        import export_assembly_full
    except (ImportError, AttributeError) as e:      # pragma: no cover
        _stale(e)
        return
    try:
        export_assembly.main()
        export_assembly_full.main()
    except AttributeError as e:
        _stale(e)


def _stale(e):
    import sys
    print("\n*** assemblies NOT refreshed -- the downstream scripts are still "
          "on the v5 torso ***")
    print(f"    {type(e).__name__}: {e}")
    print("    to update, in one change set: cad/export_assembly.py, "
          "cad/export_assembly_full.py,\n"
          "    cad/check_assembly.py, cad/fasteners.py -- battery_tray and "
          "board_frame are not parts\n"
          "    any more (they are pelvis features: BT_SEAT_* and BR_*), the "
          "board takes 2 M2.5 from\n"
          "    aft at (+/-29, BR_SCREW_Z), and gopro_base is back on 4 M3 at "
          "GP_MOUNT_X +/- GP_SCREW_XY.")
    print("    the STLs and per-part STEPs above ARE current.")
    sys.exit(1)


if __name__ == "__main__":
    import sys
    main()
    # ~55 s of the run is the two assemblies; --no-assembly skips them while
    # iterating on a single part. The default is to keep everything current.
    if "--no-assembly" in sys.argv:
        print("\n(--no-assembly: assembly.step / assembly_full.step NOT refreshed)")
    else:
        export_assemblies()
