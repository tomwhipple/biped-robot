"""Printable parts for the Bimo-like biped (build123d, algebra mode).

Run:  .venv/bin/python cad/parts.py        -> exports STLs to cad/stl/, prints
                                              per-part bbox / bed check / mass and
                                              the assembly mass rollup.

Part set (6 unique, 12 prints):
  pelvis      x1  deck + two hanging bays for the hip-roll servos
  yoke_roll   x2  clevis on the hip-roll servo horn/idler, flange below
  yoke_pitch  x2  clevis on the thigh servo horn/idler, flange above
                  (bolts to yoke_roll flange, rotated 90 deg -> hip universal)
  leg_link    x4  thigh AND shin: grips a servo case, forks to the next servo
  foot        x2  sole + ankle servo pocket + rear retention walls
  tower       x1  electronics: driver board on top, battery strapped inside

Conventions: every pitch joint has the servo HORN on +Y; the same part serves
left and right legs (legs are translations, not mirrors).
"""
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
def yoke_roll():
    """Hip-roll clevis. Local frame: roll axis == X axis through origin.
    +X = robot forward = servo horn side. Flange faces down (mates yoke_pitch).
    Print: flange face on the bed, arms up. Qty 2.
    """
    zf0 = -D.ROLL_AXIS_TO_FLANGE                    # flange top
    zf1 = zf0 - D.YOKE_FLANGE_T                     # flange bottom
    hx0, hx1 = D.SV_HORN_FACE + D.HORN_BOSS_H, D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE
    ix1, ix0 = -19.95 - 1.0, -19.95 - 1.0 - D.PLATE  # idler plate (wall outer -19.95, 1 gap)

    p = box(-23.95, 24.45, -D.YOKE_FLANGE_Y / 2, D.YOKE_FLANGE_Y / 2, zf1, zf0)
    # horn arm: plate + boss through nothing (horn sits outside the bay wall)
    p += box(hx0, hx1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, hx0, hx1, 0, 0)
    p += cyl_x(D.HORN_BOSS_D / 2, D.SV_HORN_FACE, hx0, 0, 0)          # boss 1.0
    # idler arm: plate + long boss reaching through the bay-wall slot
    p += box(ix0, ix1, -12, 12, zf0, 0) + cyl_x(D.PAD_D / 2, ix0, ix1, 0, 0)
    p += cyl_x(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, ix1, 0, 0)        # boss 4.15
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
    Print: flange face on the bed, arms up (i.e. modeled upside-down vs print).
    Qty 2.
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
            p -= teardrop_x(D.CASE_SCREW_CLEAR / 2, -21, -D.SV_TOPFACE + 1,
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
    p -= box(-21, -D.SV_TOPFACE + 1, -D.SV_CONN_HW - 0.5, D.SV_CONN_HW + 0.5,
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
    """Deck + two flat yaw-servo seats (v3yaw). Local frame: deck top at z=0,
    robot forward = +X, legs at y = +/-28. The yaw servos lie FLAT under the
    deck (length along X, vertical output axis, horn down, idler-side case face
    against the deck underside); a collar wraps the top of each case (keys its
    reaction torque, locates it) and 4x M3 pass DOWN through the deck into the
    idler-side case holes to fix the stator. The hanging roll bays are GONE --
    they moved onto `yaw_carrier`. Deck footprint (DECK_L x DECK_W), the tower
    heat-set pattern (TOWER_FOOT_X/Y) and the centre wire window are preserved;
    a local rear tab under each seat carries the -32.75 stator-screw row (the
    case cable end overhangs the deck rear edge). Print: upside down (deck top
    face on the bed), collars rise as walls -- like the old pelvis.
    """
    zd = -D.DECK_T                                  # -5 deck bottom
    zseat = zd - D.YAW_SEAT_DROP                    # -9 collar bottom
    p = box(-D.DECK_W / 2, D.DECK_W / 2, -D.DECK_L / 2, D.DECK_L / 2, zd, 0)
    cx0 = D.YAW_CASE_X_REAR - D.YAW_SEAT_GAP        # -35.41 collar rear inner
    cx1 = D.YAW_CASE_X_FRONT + D.YAW_SEAT_GAP       # +10.41 collar front inner
    cyw = 12.36 + D.YAW_SEAT_GAP                     # 12.66 case half width + fit
    w = D.YAW_SEAT_WALL
    for by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        # rear local deck tab: gives the -32.75 stator screws deck material to
        # thread into (the case cable end sits behind the deck rear edge -23)
        p += box(cx0 - w, -D.DECK_W / 2, by - cyw - w, by + cyw + w, zd, 0)
        # yaw-case collar: front / rear walls (X-normal) + side walls (Y-normal)
        p += box(cx0 - w, cx0, by - cyw - w, by + cyw + w, zseat, zd)   # rear
        p += box(cx1, cx1 + w, by - cyw - w, by + cyw + w, zseat, zd)   # front
        for s in (1, -1):
            # side walls run forward to YAW_FOOT_REACH so the +x tower feet land
            # on collar material (the -x feet already sit over the case span)
            p += box(cx0, D.YAW_FOOT_REACH, by + s * cyw, by + s * (cyw + w),
                     zseat, zd)
        # stator screws: 4x M2.5 pan self-tap DOWN through the deck (+ rear
        # tab) into the idler-side case face rows (8.30 and 32.75 behind the
        # axis). Vertical. COUNTERBORED from the deck top: the battery pack
        # sits flat on the deck and its footprint covers the -8.30 row heads
        # (audit 2026-07-28) -- sink all 8 sub-flush. Printed deck-top-down,
        # the O5.8 -> O2.9 step is a standard short counterbore bridge.
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zd - 1, 1,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                # COUNTERSINK, not the old O5.8 x 2.3 pan counterbore
                # (2026-07-28): flat heads throughout. Shallower too -- 1.25 mm
                # instead of 2.3 -- so the deck keeps 1.05 mm more material, and
                # the battery still lands on a flush top.
                p -= csk_z(-xrow, by + s * D.CASE_HOLE_LAT, 0.0, +1)
        # --- idler-face interface, from the measured SV_IDLER/SV_CONN truth ---
        # (1) disc + hub CLEARANCE POCKET: the idler disc (O19.2, +0.27 proud)
        # and its hub screws (+0.82) ROTATE with the output -- clamping them
        # against a flat deck binds the yaw joint. Pocket them 1.3 deep.
        p -= cyl_z(21.5 / 2, zd, zd + 1.3, 0, by)
        # (2) stator screw PADS: the 4 screw bosses sit ~1.78 BELOW the slab
        # plane the deck touches (vendor STEP), so bare screws would bow the
        # case. O7 pads descend SEAT_PAD_H=1.5 (deliberate under-reach) to
        # near-land on the bosses (rows -8.30 / -32.75, y +/-10.25), then the
        # screw clearance is re-drilled through them (the deck bores above
        # were cut before the pads existed).
        for xrow in D.YAW_CASE_HOLES_IDLER:
            for s in (1, -1):
                p += cyl_z(3.5, zd - D.SEAT_PAD_H, zd,
                           -xrow, by + s * D.CASE_HOLE_LAT)
                p -= cyl_z(D.CASE_SCREW_CLEAR / 2, zd - D.SEAT_PAD_H - 0.1, 1,
                           -xrow, by + s * D.CASE_HOLE_LAT)
        # (3) yaw CONNECTOR deck HOLE, over the measured trench (11.75..16.35
        # behind the axis): the two sockets open UP out of the face; plugs +
        # leads pass through the deck to the board. 0.6 mm margin lengthwise
        # (1.1 mm ligament to the -8.30 screw bores); width capped at +/-10.5
        # (not the full +/-10.9 trench) to keep 1.2 mm to the tower heat-set
        # pilots at (+/-14, +/-42) -- the sockets span well under +/-9.
        p -= box(-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 0.6,
                 by - 10.5, by + 10.5, zseat - 1, 1)
        # merge the hole into the pocket across the centre band -- the crescent
        # web between the circle edge and the hole edge is <0.85 mm for
        # |dy| < ~4 and would flag as unprintable
        p -= box(-D.SV_CONN_L[0] + 0.5, -10.0, by - 4.5, by + 4.5, zd, zd + 1.3)
    # tower mounting: heat-set pilots straight down through the deck into the
    # collar side-wall material below (feet land at |y|=42 over the side walls).
    # No raised bosses (they held the first layer off the bed -- audit
    # 2026-07-15). Thread depth = 5 mm deck + 4 mm collar wall = 9 mm > insert.
    for sx in (D.TOWER_FOOT_X, -D.TOWER_FOOT_X):
        for sy in (D.TOWER_FOOT_Y, -D.TOWER_FOOT_Y):
            p -= cyl_z(D.HEATSET_D / 2, -D.HEATSET_L, 0.01, sx, sy)
    # center lightening / wire riser window (leg + yaw cables rise to the tower)
    p -= box(-11, 11, -12, 12, zd - 1, 1)
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
    for s in (1, -1):
        p += box(fb - D.FOOT_FRONT_BOSS_HW, fb + D.FOOT_FRONT_BOSS_HW,
                 s * py, s * (py + D.FOOT_WALL_T),
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
        p -= teardrop_y(D.CASE_SCREW_CLEAR / 2,
                        s * (py - 1) if s > 0 else s * (py + D.FOOT_WALL_T + 1),
                        s * (py + D.FOOT_WALL_T + 1) if s > 0 else s * (py - 1),
                        fb, zp + 2.11)
        p -= csk_y(fb, zp + 2.11, s * (py + D.FOOT_WALL_T), s)
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
    # Each entry carries its OWN head-seat face, because they are no longer all
    # at +/-(py + FOOT_WALL_T): the rear idler tab seats at _ito (-17.30) while
    # the two front bosses still stand at +/-20.05.
    for xh, yface in ((-29.0, py + D.FOOT_WALL_T),
                      (-32.75, _ito),
                      (-D.FOOT_FRONT_BOSS_X, py + D.FOOT_WALL_T),
                      (-D.FOOT_FRONT_BOSS_X, -(py + D.FOOT_WALL_T))):
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
PARTS = [
    # name, builder, qty, print orientation note
    ("pelvis", pelvis, 1, "upside down: deck top on bed, collars rise"),
    ("yaw_carrier", yaw_carrier, 2, "horn-plate face on bed, bay walls rise "
     "(print yaw_carrier_print.stl: break-away breakout in the cable window)"),
    ("yoke_roll", yoke_roll, 2, "flange face on bed, arms up"),
    ("yoke_pitch", yoke_pitch, 2, "flange face on bed, arms up"),
    ("leg_link", leg_link, 4, "on its back: web face on bed "
     "(print leg_link_print.stl: break-away fins under the fork slabs)"),
    ("foot", foot, 2, "sole down"),
    ("tower", tower, 1, "upside down: top plate on bed (support-free: "
     "open window, no sill)"),
    ("gopro_base", gopro_base, 1, "base down, prongs up (PETG or 100% infill)"),
    ("imu_carrier", imu_carrier, 1, "flat on bed, pocket up (support-free)"),
]


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
        bb = part.bounding_box()
        dims = sorted((bb.size.X, bb.size.Y, bb.size.Z))
        fits = dims[0] <= 250 and dims[1] <= D.BED and dims[2] <= D.BED
        vol = part.volume / 1000.0                     # cm^3
        mass = part.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        print_mass += mass * qty
        rows.append((name, qty, bb.size, vol, mass, fits, orient))
        print(f"{name:11s} x{qty}  bbox {bb.size.X:6.1f} x {bb.size.Y:6.1f} x "
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
          f"{total + D.CAM_MASS:.0f} g)")
    print(f"\nheights: ankle {D.ANKLE_Z:.1f}  knee {D.KNEE_Z:.1f}  "
          f"hip-pitch {D.HIP_PITCH_Z:.1f}  hip-roll {D.HIP_ROLL_Z:.1f}  "
          f"hip-yaw {D.HIP_YAW_Z:.1f}  deck-bot(yaw) {D.DECK_BOT_Z_YAW:.1f}  "
          f"top(yaw) {D.TOP_Z_YAW:.1f} mm  "
          f"torso-center {D.TORSO_CENTER_Z:.1f}  top {D.TOP_Z:.1f} mm")

    # segment mass rollup for the sim update
    m = {n: r[4] for n, r in ((row[0], row) for row in rows)}
    # v3yaw: the 2 YAW servos sit in the pelvis (torso); each carrier + its ROLL
    # servo hang on the leg (a new hip-yaw segment above the roll link).
    seg = {
        "torso": m["pelvis"] + m["tower"] + m["gopro_base"]
                 + 2 * D.SERVO_MASS + batt + board + 27,
        "yaw carrier": m["yaw_carrier"] + D.SERVO_MASS + 5,
        "hip(roll link)": m["yoke_roll"] + m["yoke_pitch"] + 5,
        "thigh": m["leg_link"] + D.SERVO_MASS + 4,
        "shin": m["leg_link"] + D.SERVO_MASS + 4,
        "foot": m["foot"] + D.SERVO_MASS + tpu / 2 + 4,
    }
    print("\nsegment masses for sim v3yaw (g):  [camera +154 g on torso mounted]")
    for k, v in seg.items():
        print(f"  {k:16s} {v:6.1f}  (x2 legs)" if k != "torso" else
              f"  {k:16s} {v:6.1f}")
    print(f"  yaw stack drop {D.YAW_STACK_DROP:.1f} mm  (study estimated ~28)")

    # standing CG estimate (approximate segment CG heights, mm above ground)
    cam_z = D.TOP_Z + D.GP_BASE_T + D.GP_HOLE_H + 6 + D.CAM_BODY[2] / 2  # ~373
    items = [
        (m["pelvis"], 265), (m["tower"], 306), (m["gopro_base"], D.TOP_Z + 3),
        (batt, 297), (board, 313), (2 * D.SERVO_MASS, 262), (27, 290),
        (2 * (m["yoke_roll"] + m["yoke_pitch"] + 5), 225),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), 175),
        (2 * (m["leg_link"] + D.SERVO_MASS + 4), 85),
        (2 * (m["foot"] + D.SERVO_MASS + tpu / 2 + 4), 12),
    ]
    mt = sum(w for w, _ in items)
    cg = sum(w * z for w, z in items) / mt
    cg_cam = (sum(w * z for w, z in items) + D.CAM_MASS * cam_z) / (mt + D.CAM_MASS)
    print(f"\nstanding CG ~{cg:.0f} mm; with camera ~{cg_cam:.0f} mm "
          f"(+{cg_cam - cg:.0f} mm, camera CG ~{cam_z:.0f} mm)")


def export_assemblies():
    """assembly.step + assembly_full.step, from the geometry just exported.

    Chained here (user, 2026-07-27) for the same reason STEP was: they are
    built from the same parts and go stale the moment someone changes a
    dimension and only runs parts.py. Imported lazily -- both modules import
    parts, so a top-level import would be circular.
    """
    import export_assembly
    import export_assembly_full
    export_assembly.main()
    export_assembly_full.main()


if __name__ == "__main__":
    import sys
    main()
    # ~55 s of the run is the two assemblies; --no-assembly skips them while
    # iterating on a single part. The default is to keep everything current.
    if "--no-assembly" in sys.argv:
        print("\n(--no-assembly: assembly.step / assembly_full.step NOT refreshed)")
    else:
        export_assemblies()
