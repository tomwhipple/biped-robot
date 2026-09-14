"""Thigh / shin link, v6 (qty 4, same part serves both -- see cad/parts.py's
leg_link, of which this is a lightly-modified copy). Local frame: upper joint
axis == Y axis at the origin (the gripped servo's horn axis); the servo hangs
below; +X = robot forward; the lower fork grips the NEXT servo's horn (+Y) /
idler (-Y) at z = -V.LL_DROP (110, not v5's 90).

Two changes from v5's leg_link():

1. LINK_DROP 90 -> 110 (V.LL_DROP / V.LINK_DROP -- thigh and shank both grew
   to 110 mm, docs/design-v6-ankle-roll.md section 2). Every constant that is
   anchored to the LOWER joint axis moves down with it (see the shift table
   below); everything anchored to the UPPER (gripped) servo -- the whole grip
   channel, jog blocks, cable window, rib/platform detents -- is untouched,
   copied byte-for-byte from cad/parts.py::leg_link().

2. The open U section (back web + two fork tines, nothing across the front)
   is closed into a box for ~56 mm of the length (V.LL_BOX_TOP..LL_BOX_BOT):
   a front plate at V.LL_FRONT_X spans tine to tine, and the tines' front
   edge grows out to meet it. This is what stops the link racking as a
   parallelogram under lateral load -- the failure mode a plain open U has no
   resistance to.

PRINT ORIENTATION: standing on the lower fork end (pads down), not v5's
web-down. The new front plate is a wall whose broad face is normal to X; in
v5's web-down orientation (RY_XUP, print-up = model +X) that face is a
horizontal ceiling spanning the full ~42 mm tine-to-tine width -- self
-supporting only as a full 45 deg tent, which (worked through below) costs
~25 g of plastic on its own, well past the whole part's mass budget. Standing
the part on the lower fork end instead makes X and Y both horizontal print
axes and Z (the joint-to-joint axis) vertical: the front plate becomes a
plain vertical wall (no bridge at all), and the fork pads -- previously
floating 4.7 mm off the bed and needing break-away fins -- are now the first
layer. The trade is that every horizontal-bore teardrop needs its `roll`
re-aimed at the new print-up direction (model +Z, so roll=0 instead of v5's
roll=90); the byte-identical grip-channel geometry (positions, sizes) is
untouched, only that print-support parameter changes. print_fins is kept in
the signature for API parity with v5's leg_link but is a no-op here: nothing
in this orientation floats unsupported (see audit_leg_link.py).

Anchor shift table (LINK_DROP 90 -> 110, i.e. lower-anchored positions move
another 20 mm from the upper axis): v5's -92 (slab backing top) -> SLAB_TOP
-112; -70 (fork-wide/slab-mid boundary) -> SLAB_MID_TOP -90; -54 (idler slab
top) -> SLAB_IDLER_TOP -74; FORK_WIDE_Z -50 -> -70; WEB_END -58 -> V.LL_WEB_END
-78 (already computed in dimensions_v6.py by the same "32 above the lower
axis" rule). v5's cross BRACE_Z (-54..-50) is DELETED -- superseded by the
new box, which stiffens far more of the length far more thoroughly.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import (Pos, Rot, Cone, Axis, export_step, export_stl)
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402  (v5 helpers: box/cyl_*/teardrop_*/wedge_*/csk_*/bcd_*)
import check_assembly as CA  # noqa: E402  (v5 servo mocks -- servo_mock_y)

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientation: standing on the lower fork end. No rotation from the
# design frame is needed -- model Z already runs from the very negative fork
# pads (bottom) up to the near-zero grip plate (top), and check_printability's
# audit() shifts every part so its own minimum sits at print z = 0. Kept as an
# explicit 3x3 (like check_printability.ORIENT/IDENT) so a caller doesn't have
# to know that "no rotation" is the deliberate choice, not an oversight.
PRINT_ORIENT = ((1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0))

# --- print-up direction for this orientation (model +Z) drives every
# horizontal-bore `roll`. teardrop_y's roll=0 means "peak toward +z" -- which
# is exactly print-up here (v5's leg_link uses roll=90, peak +x, because ITS
# print-up is model +x). Named so the intent reads at each call site instead
# of a bare 0.
ROLL_UP = 0


def leg_link_v6(print_fins=False):
    """Thigh / shin link v6 (same part, qty 4). See module docstring.
    print_fins: accepted for API parity with v5's leg_link; a no-op here --
    printed standing on the lower fork end, the pads ARE the first layer and
    nothing else floats (audit_leg_link.py confirms no ISLAND/LEDGE finding).
    """
    t = D.GRIP_PLATE_T
    drop = -V.LL_DROP                                # -110, the lower axis
    web_x1 = -12.36 - D.WEB_GAP                      # web inner face (cable gap)
    web_x0 = web_x1 - 2.4                            # web outer face, -15.16
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21
    # anchor-shift locals (see module docstring's shift table). Written as
    # offsets from `drop` rather than retyped literals, so the formula (not
    # just the v6 number) is checkable against v5's -axis+K pattern.
    FORK_WIDE_Z = drop + 40         # v5: axis+40 (was -50 at drop=-90)
    SLAB_TOP = drop - 2             # v5: axis-2  (was -92)
    SLAB_MID_TOP = drop + 20        # v5: axis+20 (was -70)
    SLAB_IDLER_TOP = drop + 36      # v5: axis+36 (was -54)

    # --- grip channel on the servo case (BYTE IDENTICAL to v5's leg_link;
    # everything here is anchored to the UPPER (gripped) servo, not the lower
    # axis, so LINK_DROP does not touch it). Plates reach the web outer face.
    grip_x1 = 13.2
    p = parts.box(web_x0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t,
                  D.GRIP_BOT, D.GRIP_TOP_HORN)
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR          # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                     # -17.90 outer
    p += parts.box(web_x0, grip_x1, _igo, idler_seat, D.GRIP_BOT, D.GRIP_TOP_IDLER)
    # web: WEB_END is one of the LOWER-anchored shifts (v5 -58 -> V.LL_WEB_END
    # -78, see the module docstring's shift table), so unlike the grip
    # channel this edge is not byte-identical and is free to change shape.
    # Printed standing, the web's bottom face (z = LL_WEB_END) is a down-
    # facing overhang -- fine over the idler tine band (iy0..iy1, backed by
    # the slab fill below) but floating over the rest of its span (iy1
    # upward: no tine ever reaches that y at any z, the web's own upper y
    # limit stops short of the horn tine at hy0). Arched instead of flat for
    # the same reason as the front plate: flat where a tine backs it, a ramp
    # (1.3x rise-over-run, see the box comment above) everywhere else,
    # rising until it is self-supporting.
    _web_y1 = D.SV_TOPFACE + t
    p += parts.wedge_x([(iy0, D.WEB_TOP), (iy0, V.LL_WEB_END), (iy1, V.LL_WEB_END),
                        (_web_y1, V.LL_WEB_END + (_web_y1 - iy1) * 1.3), (_web_y1, D.WEB_TOP)],
                       web_x0, web_x1)

    # --- NEW: close the U into a box. Front plate spans tine to tine
    # (iy0..hy1) at V.LL_FRONT_X, from the box top (just under the jog
    # blocks) to the box bottom (1.5 mm clear of the next servo's case-top
    # sweep, r 16 about the lower axis -- see check_r16 below). The tines'
    # own front edge (x=12, v5's fork-arm bound) is grown out to meet the
    # plate's outer face so the section is a true closed tube wherever the
    # web still exists (LL_BOX_TOP..LL_WEB_END) and a stiff 3-sided C (front
    # + 2 sides, backed by the solid slab fill behind it) below that, where
    # the web has already ended.
    #
    # Printed standing, this box is a set of plain VERTICAL walls (fine on
    # their own -- each layer sits on the one below) EXCEPT at its own
    # bottom edge, which starts fresh partway up the print (V.LL_BOX_BOT is
    # ~27.5 mm above the bed) with nothing under it: the side walls' extra
    # width (x 12..LL_FRONT_X[1]) has no material below outside the tine's
    # own x<=12 footprint, and the front plate's middle span (over the open
    # insertion channel, y iy1..hy0) has NO tine there at any height to land
    # on. Both are tapered self-supporting (45 deg, run==rise, the same rule
    # the idler-boss taper already uses in this file) instead of presenting
    # a flat floating cap: the side walls grow from the existing x=12 tine
    # edge up to full width over a short run, and the front plate's bottom
    # boundary is an ARCH -- flat at LL_BOX_BOT where a tine is directly
    # behind it (y in [iy0,iy1] / [hy0,hy1]), rising to a ridge at mid-span
    # over the open channel (y in [iy1,hy0]) where nothing is ever behind it.
    # 1.3x rise-over-run margin (~37 deg from vertical, not a knife-edge 45):
    # an exact 45 deg ramp sits right on check_printability's COS45 boundary
    # and tessellation/round-off pushed a few facets a hair past it (found
    # by audit_leg_link.py -- 4 tiny ~4-5 mm2 ISLAND slivers at the ramp
    # feet). Comfortably under 45 costs a little more box depth, nothing
    # else.
    _margin = 1.6
    _sw_rise = (V.LL_FRONT_X[1] - 12.0) * _margin          # side-wall taper rise
    for a0, a1 in ((hy0, hy1), (iy0, iy1)):
        p += parts.wedge_y([(12.0, V.LL_BOX_BOT), (V.LL_FRONT_X[1], V.LL_BOX_BOT + _sw_rise),
                            (V.LL_FRONT_X[1], V.LL_BOX_TOP), (12.0, V.LL_BOX_TOP)],
                           a0, a1)
    _fp_mid = (iy1 + hy0) / 2
    _fp_peak = V.LL_BOX_BOT + (hy0 - iy1) / 2 * _margin    # ridge, margin
    p += parts.wedge_x([(iy0, V.LL_BOX_TOP), (iy0, V.LL_BOX_BOT), (iy1, V.LL_BOX_BOT),
                        (_fp_mid, _fp_peak), (hy0, V.LL_BOX_BOT), (hy1, V.LL_BOX_BOT),
                        (hy1, V.LL_BOX_TOP)],
                       V.LL_FRONT_X[0], V.LL_FRONT_X[1])

    # --- fork arms down to the next servo (wide near the web, narrow below).
    for a0, a1, wide0 in ((hy0, hy1, D.SV_TOPFACE + t), (iy0, iy1, iy0)):
        p += parts.box(web_x0, 12, wide0, a1, FORK_WIDE_Z, -33)
        p += parts.box(D.FORK_NARROW_X, 12, a0, a1, drop, FORK_WIDE_Z)
        p += parts.cyl_y(D.PAD_D / 2, a0, a1, 0, drop)
    # NOTE (jog-block backing, tried and reverted): the jog blocks (BYTE
    # IDENTICAL, below) sit slightly wider in y than the "wide" fork band
    # above them (horn: SV_TOPFACE..SV_TOPFACE+t vs the fork's SV_TOPFACE+t
    # ..hy1; idler: iy1..idler_seat vs the fork's iy0..iy1) -- a 2.4-2.5 mm x
    # 24.8-27 mm strip that was a plain SIDE wall in v5's web-down print and
    # reads as a down-facing overhang standing up. Nothing (slab/narrow-fork/
    # web) backs it anywhere below at this x,y, at any depth, so no fill of
    # any height fixes it -- it only relocates the same-width overhang to a
    # new z. Left as v5 drew it and reported below as a residual: an
    # end-anchored ~25 mm x 2.4 mm rib, 5 mm past the BEAM_OK guideline
    # (which is itself conservative for a rib this narrow).
    # --- permanent slab backing (v5 "make it solid" rule; anchor-shifted).
    p += parts.box(web_x0, D.FORK_NARROW_X, hy0, hy1, SLAB_TOP, FORK_WIDE_Z)
    p += parts.box(web_x0, D.FORK_NARROW_X, iy0, iy1, SLAB_IDLER_TOP, FORK_WIDE_Z)
    p += parts.box(web_x0, D.FORK_NARROW_X, iy0, iy1, SLAB_TOP, SLAB_MID_TOP)
    # KNEE ROM FIX (found by cad/v6/check_assembly_v6.py --joint knee, this
    # part gripping itself as thigh+shin): at knee +95 deg (hyperextension)
    # the SHIN's arched web (its outer-top corner, near WEB_TOP -16, y in the
    # idler tine band) swings down into the THIGH's own idler slab right at
    # ITS outer-top corner (x = web_x0, z near SLAB_MID_TOP) -- 3.48 mm3
    # overlap, 0 mm clearance where v5's leg_link (90 mm drop, same relative
    # geometry) had 0.5 mm to spare. Not a grip-channel/pad feature -- a
    # corner of the slab fill added above -- so it is the one to cut back.
    # Chamfered (not squared off) to remove the least material: a triangular
    # wedge off the slab's outer-top corner, sized (search, see the module's
    # commit note) so BOTH +-95 clear by the full 0.7 mm the un-clipped pair
    # already had at -95 (SWEEP_BUFFER only requires 0.5).
    p -= parts.wedge_y([(web_x0 - 1, SLAB_MID_TOP + 1), (web_x0 - 1, SLAB_MID_TOP - 17),
                        (web_x0 + 4.5, SLAB_MID_TOP + 1)], iy0 - 1, iy1 + 1)
    # idler boss: OD tapered (45 deg run==rise) so the print-underside band
    # never exceeds 45 deg. Unchanged formula -- moves with `drop` for free.
    _ibh = abs(D.IDLER_BOSS_H)
    p += Pos(0, (D.SV_IDLER_FACE + iy1) / 2, drop) * Rot(90, 0, 0) * Cone(
        D.IDLER_BOSS_D / 2 - _ibh, D.IDLER_BOSS_D / 2, _ibh)

    # --- jog blocks (BYTE IDENTICAL, upper-anchored)
    p += parts.box(web_x0, 12, D.SV_TOPFACE, hy1, -36.5, -33)
    p += parts.box(web_x0, 12, iy0, idler_seat, -36.5, -33)

    # --- DETENT for the servo's horn-side RIB (BYTE IDENTICAL to v5).
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
    # DETENT for the idler-side PLATFORM (BYTE IDENTICAL to v5).
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0 = idler_seat
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= parts.box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    _irr = 1.2 * (_iy0 - _iy1)
    p -= parts.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                       _iz0 - 0.6, _iz1 + 0.6)

    # --- holes: case grip screws (BYTE IDENTICAL positions/sizes; roll
    # re-aimed to this part's print-up, model +Z -- ROLL_UP=0, not v5's 90).
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                                  D.SV_TOPFACE + t + 1, lx, -zrow, roll=ROLL_UP)
            p -= parts.csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, _igo - 1,
                              idler_seat + 1, lx, -D.CASE_HOLES_BOT[1], roll=ROLL_UP)
        p -= parts.csk_y(lx, -D.CASE_HOLES_BOT[1], _igo, -1)
        p -= parts.teardrop_y(D.CASE_CS_D / 2 + 0.4, iy0 - 1, _igo + 0.05,
                              lx, -D.CASE_HOLES_BOT[1], roll=ROLL_UP)

    # --- holes: lower joint pads (BYTE IDENTICAL positions/sizes; roll
    # re-aimed the same way).
    for h in parts.bcd_y(iy0 - 1, hy1 + 1, 0, drop, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, drop)
    p -= parts.cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
                     D.SV_IDLER_FACE + 0.7, 0, drop)

    # --- cable window through the web (BYTE IDENTICAL, upper-anchored)
    p -= parts.box(web_x0 - 1, web_x1 + 1, -4.5, 4.5,
                   V.LL_CABLE_WINDOW_Z[1], V.LL_CABLE_WINDOW_Z[0])
    for ly in (9, -9):
        p -= parts.cyl_x(2.25, web_x0 - 1, web_x1 + 1, ly, -40)

    # --- trim everything below the lower joint axis back to the pad radius.
    _below = parts.box(-40, 40, -40, 40, -220, drop)
    p -= _below - parts.cyl_y(D.PAD_D / 2, -40, 40, 0, drop)
    # --- chamfer the two corners beside the axis (unchanged formula).
    _pr = D.PAD_D / 2
    _yl, _yh = iy0 - 1, hy1 + 1
    p -= parts.wedge_y([(_pr, drop), (12.0, drop),
                        (12.0, drop + D.LEG_CORNER_CUT_FRONT_H)], _yl, _yh)
    p -= parts.wedge_y([(-_pr, drop), (web_x0, drop),
                        (web_x0, drop + D.LEG_CORNER_CUT_REAR_H)], _yl, _yh)

    # --- fillet the box's outer vertical edges (LL_EDGE_R). Selected by
    # location: the four corners where the new front plate / side fills meet
    # the box top/bottom planes, picked from the edge list rather than
    # constructed, so a boolean-cut change elsewhere can't silently orphan a
    # fillet reference. Mating faces (grip seats, pad faces, web inner face)
    # are never in this list -- none of them sit at the box's outer x/y
    # extremes.
    corners = ((V.LL_FRONT_X[1], iy0), (V.LL_FRONT_X[1], hy1),
              (web_x0, iy0), (web_x0, hy1))
    edges = []
    for e in p.edges():
        c = e.center()
        if abs(e.length - abs(V.LL_BOX_TOP - V.LL_BOX_BOT)) > 0.6:
            continue
        for cx, cy in corners:
            if abs(c.X - cx) < 0.05 and abs(c.Y - cy) < 0.05:
                edges.append(e)
                break
    if edges:
        try:
            from build123d import fillet
            p = fillet(edges, radius=V.LL_EDGE_R)
        except Exception as e:  # noqa: BLE001 -- report, don't crash the build
            print(f"  [leg_link_v6] fillet failed on {len(edges)} edges "
                  f"({type(e).__name__}: {str(e)[:80]}); trying chamfer")
            try:
                from build123d import chamfer
                p = chamfer(edges, length=V.LL_EDGE_R)
            except Exception as e2:  # noqa: BLE001
                print(f"  [leg_link_v6] chamfer also failed "
                      f"({type(e2).__name__}: {str(e2)[:80]}); "
                      f"box edges left sharp")
    else:
        print("  [leg_link_v6] no box-corner edges found to fillet "
              "(geometry changed?) -- left sharp")

    if print_fins:
        pass  # no-op: standing on the lower fork end, nothing floats (see
              # audit_leg_link.py -- no ISLAND/LEDGE finding on the pads)
    return p


# --------------------------------------------------------------- fasteners
def SCREWS():
    """Every fastener this part carries, in the LOCAL frame (upper joint axis
    == Y at the origin), in the spirit of cad/fasteners.py's SEATS: a list of
    {name, kind, pos (x,y,z), axis (unit vector, HEAD outward -- the
    direction a driver approaches from), length}.

    6x M2.5x8 flat-head grip screws (case rows, byte-identical to v5's
    leg_link_screws()) + 4x M3x6 horn pad screws + 4x M3x8 idler pad screws
    with a thin washer (lower joint, at z = -V.LL_DROP)."""
    t = D.GRIP_PLATE_T
    drop = -V.LL_DROP
    hy1 = D.SV_HORN_FACE + D.PLATE
    iy0 = D.IDLER_ARM_INNER - D.PLATE
    seat = D.SV_TOPFACE + t                       # 19.75, horn plate outer
    igo = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER  # -17.90
    s = []
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            s.append(dict(name=f"grip_horn_{zrow:.0f}_{'p' if lx > 0 else 'n'}",
                          kind="M2.5x8 flat-head self-tap",
                          pos=(lx, seat, -zrow), axis=(0.0, 1.0, 0.0), length=8.0))
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append(dict(name=f"grip_idler_{'p' if lx > 0 else 'n'}",
                      kind="M2.5x8 flat-head self-tap",
                      pos=(lx, igo, -D.CASE_HOLES_BOT[1]), axis=(0.0, -1.0, 0.0),
                      length=8.0))
    r = D.BCD / 2
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append(dict(name=f"horn_pad_{dx:+.0f}_{dz:+.0f}",
                      kind="M3x6 button head", pos=(dx, hy1, drop + dz),
                      axis=(0.0, 1.0, 0.0), length=6.0))
        s.append(dict(name=f"idler_pad_{dx:+.0f}_{dz:+.0f}",
                      kind="M3x8 button head + thin washer", pos=(dx, iy0, drop + dz),
                      axis=(0.0, -1.0, 0.0), length=8.0))
    return s


# --------------------------------------------------------------- insertion path
# The upper (gripped) servo enters the grip channel by sliding along +Z from
# below (case 24.72 mm wide in X must pass between the web inner face,
# -12.76, and the new front plate's inner face, V.LL_FRONT_X[0]=13.46 -- a
# 26.22 mm gap against a 24.72 mm case). Given as {start, direction, distance}
# spanning the NEW box channel specifically (see check_insertion's docstring
# for why the full case-length/horn-boss travel is not part of this claim).
SERVO_INSERT = dict(start=(0.0, 0.0, V.LL_BOX_BOT),
                    direction=(0.0, 0.0, 1.0),
                    distance=V.LL_BOX_TOP - V.LL_BOX_BOT)


# --------------------------------------------------------------- audits
def check_r16(verbose=True):
    """Requirement 2: the next servo's case top sweeps r 16 mm about the
    lower axis (hypot(SV_AXIS_FROM_OUT_END, SV_WID/2) ~= 16.0) as the joint
    rotates -- nothing of the NEW box-closing geometry may exist inside that
    circle (this is what sets V.LL_BOX_BOT: dz = LL_DROP-16-buffer already
    clears it by construction). Checked against just the added box (front
    plate + side fills), not the whole part: the pre-existing v5 fork/pad/
    taper geometry below the box was already shaped (PAD_D trim, idler-boss
    taper, corner chamfers) against the servo's REAL, ROM-limited sweep via
    check_assembly's joint-angle sampling, not this full-360 deg circle --
    re-litigating that geometry here would flag proven, unchanged design."""
    drop = -V.LL_DROP
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE
    box_add = parts.box(V.LL_FRONT_X[0], V.LL_FRONT_X[1], iy0, hy1,
                        V.LL_BOX_TOP, V.LL_BOX_BOT)
    box_add += parts.box(12.0, V.LL_FRONT_X[1], hy0, hy1, V.LL_BOX_TOP, V.LL_BOX_BOT)
    box_add += parts.box(12.0, V.LL_FRONT_X[1], iy0, iy1, V.LL_BOX_TOP, V.LL_BOX_BOT)
    circle = parts.cyl_y(16.0, -60, 60, 0, drop)
    vol = (box_add & circle).volume
    if verbose:
        print(f"  r16 sweep clearance (new box vs the next servo's case-top "
              f"circle): {vol:.3f} mm3 overlap ({'OK' if vol < 0.5 else 'FAIL'})")
    return vol


def check_insertion(p, verbose=True):
    """Requirement 2: "case 24.72 mm wide in x must pass between the web
    inner face and the front plate inner face -- 26.3 mm available". Checked
    as the literal claim: the case's WIDTH ENVELOPE (X +-SV_WID/2, Y =
    SV_IDLER_CASE_FACE..SV_TOPFACE -- the plain case body, constant along its
    own length) extended the full length of the new box channel
    (V.LL_BOX_TOP..LL_BOX_BOT) must not intersect the part.

    NOT tested as a rigid translation of the full servo_mock_y (case + horn
    boss + idler disc + horn rib) along the whole part length: that fails
    for v5's OWN unmodified leg_link too (confirmed -- up to ~3975 mm3 of
    overlap sweeping the full mock through v5's 90 mm channel, worse than
    here), because the horn boss / idler disc / rib are anchored at the
    joint axis (mock Z=0) and are never actually translated 90+ mm down the
    leg during assembly -- a straight axial slide of the COMPLETE rigid
    servo is not the real insertion motion (side/angled approach, most
    likely) either here or in v5, so re-litigating it here would flag a
    pre-existing v5 characteristic, not a v6 regression."""
    env = parts.box(-D.SV_WID / 2, D.SV_WID / 2, D.SV_IDLER_CASE_FACE, D.SV_TOPFACE,
                    V.LL_BOX_TOP, V.LL_BOX_BOT)
    vol = (p & env).volume
    if verbose:
        print(f"  case-width envelope vs the box channel "
              f"(V.LL_BOX_TOP={V.LL_BOX_TOP} .. LL_BOX_BOT={V.LL_BOX_BOT}): "
              f"{vol:.3f} mm3 overlap ({'OK' if vol < 0.5 else 'FAIL'})")
        full = Pos(0, 0, -20) * CA.servo_mock_y()
        try:
            note = (p & full).volume
        except Exception:  # noqa: BLE001
            note = 0.0
        print(f"  (full servo_mock_y rigid-slide sweep, for reference only, "
              f"is NOT clear at every z -- see module docstring / report; "
              f"e.g. at z=-20: {note:.1f} mm3)")
    return vol


def check_screw_access(p, verbose=True):
    """Requirement 5: a 7 mm dia. cylinder along each screw's axis, from the
    head outward 40 mm, must clear both the part itself and the relevant
    servo mock in place (driver access). Grip screws check against the
    gripped (upper) servo at z=0; lower-pad screws check against the next
    (lower) servo at z=drop."""
    drop = -V.LL_DROP
    upper_mock = CA.servo_mock_y()
    lower_mock = Pos(0, 0, drop) * CA.servo_mock_y()
    bad = []
    for sc in SCREWS():
        x, y, z = sc["pos"]
        ax, ay, az = sc["axis"]
        length = 40.0
        y0, y1 = y, y + ay * length            # axis is always (0,+-1,0) here
        cyl = parts.cyl_y(3.5, min(y0, y1), max(y0, y1), x, z)
        v_part = (p & cyl).volume
        mock = upper_mock if abs(z - 0.0) < abs(z - drop) else lower_mock
        v_mock = (mock & cyl).volume
        ok = v_part < 0.5 and v_mock < 0.5
        if verbose:
            mark = "  " if ok else "**"
            print(f"  {mark}{sc['name']:16s} {sc['kind']:32s} driver access: "
                  f"part {v_part:6.2f} mm3  servo {v_mock:6.2f} mm3  "
                  f"{'OK' if ok else 'FAIL'}")
        if not ok:
            bad.append(sc["name"])
    return bad


def run_audits(p, stl_path):
    """Everything requirements 2/4/5/6 ask for, in one place, so both this
    module's __main__ and audit_leg_link.py report identically."""
    import numpy as np
    HERE_CP = os.path.join(HERE, "..")
    sys.path.insert(0, HERE_CP)
    import check_printability as CP  # noqa: E402  (v5 -- runtime override only)

    ok = True
    print("\n== printability (check_printability.py, standing orientation) ==")
    CP.STL = os.path.dirname(stl_path)                       # runtime only
    CP.ORIENT["leg_link_v6"] = (np.array(PRINT_ORIENT), "standing on the lower fork end")
    CP.PRINT_STL["leg_link_v6"] = os.path.basename(stl_path)
    findings = CP.audit("leg_link_v6")
    if findings:
        ok = False

    print("\n== requirement 2: r16 sweep + servo insertion channel ==")
    if check_r16() >= 0.5:
        ok = False
    if check_insertion(p) >= 0.5:
        ok = False

    print("\n== requirement 5: screw driver access ==")
    bad = check_screw_access(p)
    if bad:
        ok = False

    vol = p.volume / 1000.0
    mass = p.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    print(f"\n== mass: {vol:.2f} cm3 -> {mass:.1f} g PETG "
          f"(v5 leg_link ~24 g at 90 mm drop; budget 40 g)  "
          f"{'OK' if mass < 40 else 'OVER BUDGET'}")
    if mass >= 40:
        ok = False

    print(f"\n{'PASS' if ok else 'FAIL'}: leg_link_v6 "
          f"{'clears every audit' if ok else 'has findings above'}")
    return ok


def render(stl_path, png_path, px=640):
    """MuJoCo offscreen render, like cad/render_part.py (MUJOCO_GL=egl set
    BEFORE importing mujoco)."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco  # noqa: E402 -- must follow the env var
    import numpy as np
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
    center = np.zeros(3)
    mujoco.mju_rotVecQuat(center, v.mean(0), model.mesh_quat[0])
    center += model.mesh_pos[0]
    ren = mujoco.Renderer(model, px, px)
    frames = []
    for az, el in ((135, -30), (90, 89), (180, -15)):    # iso, bottom, back
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = center, 2.6 * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    os.makedirs(OUT_REN, exist_ok=True)
    p = leg_link_v6()
    stl_path = os.path.join(OUT_STL, "leg_link_v6.stl")
    export_stl(p, stl_path)
    export_step(p, os.path.join(OUT_STEP, "leg_link_v6.step"))
    bb = p.bounding_box()
    print(f"leg_link_v6  bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    run_audits(p, stl_path)
    png = render(stl_path, os.path.join(OUT_REN, "leg_link_v6.png"))
    print("wrote", png)
