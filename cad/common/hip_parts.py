"""The hip pieces the v6 hip is built on (build123d, algebra mode).

    yoke_roll()     the hip-roll clevis on the roll servo's horn and idler
    yoke_pitch()    the hip-pitch clevis on the thigh servo, flanged to yoke_roll
    yaw_carrier()   bolts to the hip-yaw servo's horn and carries the roll bay

cad/v6/hip_yoke_glued.py builds the glued hip yoke from the two clevises and
cad/v6/yaw_carrier_v6.py the bearing-hub carrier from yaw_carrier(); none of
the three is printed as it stands. Conventions: every pitch joint has the
servo HORN on +Y; the same part serves left and right legs.
"""
import math  # noqa: F401

from build123d import *  # noqa: F401,F403

from common import dimensions as D
from common.shapes import (bcd_x, bcd_y, box, csk_x, cyl_x, cyl_y, cyl_z,  # noqa: F401
                           teardrop_x, wedge_y)


# ---------------------------------------------------------------- yoke_roll
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
