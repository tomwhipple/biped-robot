"""Thigh / shin link, v6 (qty 4, same part serves both -- see cad/parts.py's
leg_link, of which this is a modified copy). Local frame: upper joint axis ==
Y axis at the origin (the gripped servo's horn axis); the servo hangs below;
+X = robot forward; the lower fork grips the NEXT servo's horn (+Y) / idler
(-Y) at z = -V.LL_DROP (110, not v5's 90).

The section is v5's open-front U: a back web and two side walls (the grip
plates up top, the fork tines below), nothing across the front. What v6
changes:

1. LINK_DROP 90 -> 110 (V.LL_DROP -- thigh and shank both grew to 110 mm,
   docs/design-v6/2026-09-13-design-record.md section 2). Every constant
   anchored to the LOWER joint axis moves down with it (shift table below);
   everything anchored to the UPPER (gripped) servo -- grip channel, jog
   blocks, cable window, rib/platform detents -- is v5's.

2. END WALLS. An open U twists about its web; two walls tie the side walls
   together, one at each end of the open span. Each is a plate in the X-Y
   plane, web to the tines' front edge, tine to tine:
   - the TOP wall (V.LL_TOP_WALL_Z) just below the cable window, where v5's
     single cross brace sat, so the servo lead still leaves through the web;
   - the BOTTOM wall (V.LL_BOT_WALL_Z) where the web ends and the fork
     begins. It cannot sit lower: at deep knee flexion the shin's web swings
     up inside the thigh's fork as far as ~33 mm above the knee axis, all the
     way across the channel.
   Rooted on the web, both print as walls rising off the bed.

3. Side walls without needless cuts. v5's 16 mm window in the idler side
   wall (a clearance for a v5 foot wall) is now LL_KNEE_POCKET: only where
   the shin's corner passes at deep knee flexion, with sloped ends so it
   prints with a 6.6 mm roof. v5's knee-hyperextension chamfer on that wall and
   its rear corner chamfer beside the pads are gone: nothing sweeps them. The
   relief cuts for deep knee flexion (LL_FLEX_CUT, LL_FOLD_CHAMFER) and hip
   flexion (LL_HIP_RELIEF_X0) are sized against this geometry.

4. The idler grip plate stops square 5 mm above its screws
   (V.LL_IDLER_PLATE_TOP).

PRINT ORIENTATION: on its back, web face on the bed (v5's, RY_XUP: print-up =
model +X). The web, both side walls and both end walls are walls rising off
the bed; nothing spans the open front. Horizontal bores are teardropped
toward +X (roll 90). The fork's round pad ends sit a few mm above the bed past
the slab: the slicer supports them (build plate only), nothing is modelled.

Anchor shift table (LINK_DROP 90 -> 110): v5's -92 (slab backing top) ->
SLAB_TOP -112; FORK_WIDE_Z -50 -> -70; WEB_END -58 -> V.LL_WEB_END -78.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import (Pos, Rot, Cone, export_step, export_stl)
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402  (v5 helpers: box/cyl_*/teardrop_*/wedge_*/csk_*/bcd_*)
import check_assembly as CA  # noqa: E402  (v5 servo mocks -- servo_mock_y)

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientation: on its back, web face down (check_printability.RY_XUP,
# model +X -> print +Z), kept here as the explicit 3x3.
PRINT_ORIENT = ((0.0, 0.0, -1.0),
                (0.0, 1.0, 0.0),
                (1.0, 0.0, 0.0))

# teardrop_y's roll toward print-up: 90 = model +X.
ROLL_UP = 90

# jog blocks (v5, upper-anchored): the grip plates step out to the fork tines here
JOG_Z = (-36.5, -33.0)


def _seg_dist(p, a, b):
    """Distance from point p to segment ab, all (x, z)."""
    (px, pz), (ax, az), (bx, bz) = p, a, b
    dx, dz = bx - ax, bz - az
    t = max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / (dx * dx + dz * dz)))
    return math.hypot(px - ax - t * dx, pz - az - t * dz)


def leg_link_v6(print_fins=False):
    """Thigh / shin link v6 (same part, qty 4). See module docstring.
    print_fins: accepted for API parity with v5's leg_link; a no-op -- the
    pad ends are left to the slicer's supports.
    """
    t = D.GRIP_PLATE_T
    drop = -V.LL_DROP                                # -110, the lower axis
    web_x1 = -12.36 - D.WEB_GAP                      # web inner face (cable gap)
    web_x0 = web_x1 - 2.4                            # web outer face, -15.16
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -17.4..-20.4
    FORK_WIDE_Z = drop + 40         # v5: axis+40 (was -50 at drop=-90)
    SLAB_TOP = drop - 2             # v5: axis-2  (was -92)

    # --- grip channel on the servo case (v5's leg_link but for the idler
    # plate's top, V.LL_IDLER_PLATE_TOP: square, 5 mm above its screws).
    # Plates reach the web outer face.
    grip_x1 = 13.2
    p = parts.box(web_x0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t,
                  D.GRIP_BOT, D.GRIP_TOP_HORN)
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR          # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                     # -17.90 outer
    p += parts.box(web_x0, grip_x1, _igo, idler_seat, D.GRIP_BOT, V.LL_IDLER_PLATE_TOP)

    # --- web, WEB_TOP down to LL_WEB_END, straight across. Its horn end runs
    # on to hy0 below FORK_WIDE_Z, closing the 0.7 slot to the horn slab.
    _web_y1 = D.SV_TOPFACE + t
    p += parts.wedge_x([(iy0, D.WEB_TOP), (iy0, V.LL_WEB_END), (hy0, V.LL_WEB_END),
                        (hy0, FORK_WIDE_Z), (_web_y1, FORK_WIDE_Z), (_web_y1, D.WEB_TOP)],
                       web_x0, web_x1)

    # --- fork arms down to the next servo (wide near the web, narrow below),
    # and the slab backing that makes each side wall solid to the web face.
    for a0, a1, wide0 in ((hy0, hy1, D.SV_TOPFACE + t), (iy0, iy1, iy0)):
        p += parts.box(web_x0, 12, wide0, a1, FORK_WIDE_Z, JOG_Z[1])
        p += parts.box(D.FORK_NARROW_X, 12, a0, a1, drop, FORK_WIDE_Z)
        p += parts.cyl_y(D.PAD_D / 2, a0, a1, 0, drop)
        p += parts.box(web_x0, D.FORK_NARROW_X, a0, a1, SLAB_TOP, FORK_WIDE_Z)

    # --- END WALLS (V.LL_TOP_WALL_Z, V.LL_BOT_WALL_Z): web outer face to the
    # tines' front edge, from the idler tine's inner face to the horn tine's
    # (hy0: below FORK_WIDE_Z the horn tine starts there, not at _web_y1).
    for z0, z1 in (V.LL_TOP_WALL_Z, V.LL_BOT_WALL_Z):
        p += parts.box(web_x0, 12, iy1 - 0.01, hy0 + 0.01, z0, z1)

    # DEEP-FLEXION RELIEF (V.LL_FLEX_CUT, see dimensions_v6): the polygon
    # (x, z) cut through the idler tine band only (y iy0..iy1, plus the 0.6
    # mm the thigh's idler-boss taper protrudes past the tine face). It is the
    # region the thigh's idler tine sweeps through on this corner up to 131
    # deg of knee flexion, plus 0.6 mm -- no more -- so the lower idler grip
    # screw's access bore keeps >= 1 mm of wall (asserted below).
    # NOTE on signs: assembly +knee = human flexion (shin swings BACK); the
    # sim's knee axis is -Y so that is sim -130.
    if V.LL_FLEX_CUT:
        p -= parts.wedge_y(list(V.LL_FLEX_CUT), iy0 - 1, iy1 + 0.6)
        _fc = list(V.LL_FLEX_CUT)
        _cs = (-D.CASE_HOLE_LAT, -D.CASE_HOLES_BOT[1])
        _cs_wall = min(_seg_dist(_cs, a, b) for a, b in zip(_fc, _fc[1:] + _fc[:1])) - (D.CASE_CS_D / 2 + 0.4)
        assert _cs_wall >= 1.0, f"LL_FLEX_CUT leaves {_cs_wall:.2f} mm to the idler grip screw's access bore"
    # KNEE POCKET (V.LL_KNEE_POCKET_Z): where the shin's corner passes the
    # idler slab at deep flexion. Full slab depth; its ends slope
    # LL_KNEE_POCKET_RAMP so the web-down print bridges only the roof.
    _kz0, _kz1 = drop + V.LL_KNEE_POCKET_Z[0], drop + V.LL_KNEE_POCKET_Z[1]
    _kh = D.FORK_NARROW_X - web_x0
    _kr = V.LL_KNEE_POCKET_RAMP
    p -= parts.wedge_y([(web_x0 - 1, _kz0 - 1 / _kr), (web_x0 - 1, _kz1 + 1 / _kr),
                        (D.FORK_NARROW_X, _kz1 - _kh / _kr), (D.FORK_NARROW_X, _kz0 + _kh / _kr)],
                       iy0 - 1, iy1)
    # FOLD CHAMFERS (V.LL_FOLD_CHAMFER): the web end's outer-bottom corner
    # across its whole width (the shin's rear face lands on it at 131 deg) and
    # the horn jog block's rear-top corner (applied after the block, below).
    _c = V.LL_FOLD_CHAMFER
    p -= parts.wedge_y([(web_x0 - 1, V.LL_WEB_END - 1), (web_x0 - 1, V.LL_WEB_END + _c),
                        (web_x0 + _c, V.LL_WEB_END - 1)], iy0 - 1, hy0 - 0.01)
    # idler boss: OD tapered (45 deg run==rise) so its print-underside band
    # never exceeds 45 deg.
    _ibh = abs(D.IDLER_BOSS_H)
    p += Pos(0, (D.SV_IDLER_FACE + iy1) / 2, drop) * Rot(90, 0, 0) * Cone(
        D.IDLER_BOSS_D / 2 - _ibh, D.IDLER_BOSS_D / 2, _ibh)

    # --- jog blocks (v5, upper-anchored)
    p += parts.box(web_x0, 12, D.SV_TOPFACE, hy1, *JOG_Z)
    p += parts.box(web_x0, 12, iy0, idler_seat, *JOG_Z)
    p -= parts.wedge_y([(web_x0 - 1, JOG_Z[1] + 1), (web_x0 - 1, JOG_Z[1] - _c),
                        (web_x0 + _c, JOG_Z[1] + 1)], _web_y1 + 0.05, hy1 + 1)

    # --- DETENT for the servo's horn-side RIB (v5), with its print ramp: the
    # pocket's +X wall is where plate material resumes over the void when the
    # part prints web-down, so it is ramped instead of bridged.
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
    # DETENT for the idler-side PLATFORM (v5), same print ramp.
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0 = idler_seat
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= parts.box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    _irr = 1.2 * (_iy0 - _iy1)
    p -= parts.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                       _iz0 - 0.6, _iz1 + 0.6)

    # --- holes: case grip screws (v5 positions/sizes, teardropped toward +X)
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

    # --- holes: lower joint pads (v5 positions/sizes)
    for h in parts.bcd_y(iy0 - 1, hy1 + 1, 0, drop, roll=ROLL_UP):
        p -= h
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, drop)
    p -= parts.cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
                     D.SV_IDLER_FACE + 0.7, 0, drop)

    # --- cable window through the web (v5, upper-anchored)
    p -= parts.box(web_x0 - 1, web_x1 + 1, -4.5, 4.5,
                   V.LL_CABLE_WINDOW_Z[1], V.LL_CABLE_WINDOW_Z[0])
    for ly in (9, -9):
        p -= parts.cyl_x(2.25, web_x0 - 1, web_x1 + 1, ly, -40)

    # --- trim everything below the lower joint axis back to the pad radius.
    _below = parts.box(-40, 40, -40, 40, -220, drop)
    p -= _below - parts.cyl_y(D.PAD_D / 2, -40, 40, 0, drop)
    # --- chamfer the front corner beside the axis (v5 formula). v5's matching
    # rear chamfer is gone: nothing sweeps that corner (knee 0.58 mm at 130,
    # ankle clear), and it held the fork end off the bed in the web-down print.
    _pr = D.PAD_D / 2
    p -= parts.wedge_y([(_pr, drop), (12.0, drop),
                        (12.0, drop + D.LEG_CORNER_CUT_FRONT_H)], iy0 - 1, hy1 + 1)

    # HIP-FLEXION RELIEF (V.LL_HIP_RELIEF_X0 / _UP): at deep hip flexion the
    # roll flange's front-bottom edge meets the grip plates' front strip just
    # above the jog blocks; everything forward of LL_HIP_RELIEF_X0 is trimmed
    # there. Harmless on the shin (same part).
    p -= parts.box(V.LL_HIP_RELIEF_X0, grip_x1 + 1.0, iy0 - 1.0, hy1 + 1.0,
                   JOG_Z[0] - 0.01, JOG_Z[0] + V.LL_HIP_RELIEF_UP)

    if print_fins:
        pass  # no-op: supports are the slicer's job (cad/PRINT_LIST.md)
    return p


# --------------------------------------------------------------- fasteners
def SCREWS():
    """Every fastener this part carries, in the LOCAL frame (upper joint axis
    == Y at the origin), in the spirit of cad/fasteners.py's SEATS: a list of
    {name, kind, pos (x,y,z), axis (unit vector, HEAD outward -- the
    direction a driver approaches from), length}.

    6x M2.5x8 flat-head grip screws (case rows, byte-identical to v5's
    leg_link_screws()) + 4 horn pad and 4 idler pad disc screws on the lower
    joint (z = -V.LL_DROP), their length chosen by the stack under the head
    (V.disc_screw: M3x5 on the 3.0 horn pad, M3x6 on the 3.6 idler pad, no
    washers). `stack` / `engage` / `flange` are what check_assembly_v6's
    disc-screw audit verifies."""
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
    h_stack, i_stack = hy1 - D.SV_HORN_FACE, D.SV_IDLER_FACE - iy0      # 3.0, 3.6
    hl, he, hf = V.disc_screw(h_stack, "horn")
    il, ie, iflg = V.disc_screw(i_stack, "idler")
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        s.append(dict(name=f"horn_pad_{dx:+.0f}_{dz:+.0f}",
                      kind=f"M3x{hl} button head", pos=(dx, hy1, drop + dz),
                      axis=(0.0, 1.0, 0.0), length=float(hl), stack=h_stack, engage=he, flange=hf))
        s.append(dict(name=f"idler_pad_{dx:+.0f}_{dz:+.0f}",
                      kind=f"M3x{il} button head", pos=(dx, iy0, drop + dz),
                      axis=(0.0, -1.0, 0.0), length=float(il), stack=i_stack, engage=ie, flange=iflg))
    return s


# --------------------------------------------------------------- audits
def check_r16(verbose=True):
    """Requirement 2: the next servo's case top sweeps r 16 mm about the
    lower axis (hypot(SV_AXIS_FROM_OUT_END, SV_WID/2) ~= 16.0) as the joint
    rotates -- the end walls may not reach inside that circle (plus
    SWEEP_BUFFER). The rest of the fork was shaped against the servo's real,
    ROM-limited sweep by check_assembly_v6."""
    drop = -V.LL_DROP
    hy0 = D.SV_HORN_FACE
    iy1 = D.IDLER_ARM_INNER
    walls = None
    for z0, z1 in (V.LL_TOP_WALL_Z, V.LL_BOT_WALL_Z):
        w = parts.box(-12.36 - D.WEB_GAP - 2.4, 12, iy1, hy0, z0, z1)
        walls = w if walls is None else walls + w
    circle = parts.cyl_y(16.0 + D.SWEEP_BUFFER, -60, 60, 0, drop)
    vol = (walls & circle).volume
    if verbose:
        print(f"  r16 sweep clearance (end walls vs the next servo's case-top "
              f"circle + {D.SWEEP_BUFFER} mm): {vol:.3f} mm3 overlap "
              f"({'OK' if vol < 0.5 else 'FAIL'})")
    return vol


def check_insertion(p, verbose=True):
    """Requirement 2: the gripped servo goes in from the TOP -- its horn boss
    drops into the horn plate's relief notch, which is open at the top edge;
    from below or the front the boss would meet the plate. So the plain case
    body (X +-SV_WID/2, Y = SV_IDLER_CASE_FACE..SV_TOPFACE) swept from above
    the part down to its seat (bottom at -35.11) must not intersect the part.
    The horn boss and rib travel down the relief notch and the rib detent,
    both open at the top."""
    env = parts.box(-D.SV_WID / 2, D.SV_WID / 2, D.SV_IDLER_CASE_FACE, D.SV_TOPFACE,
                    -D.SV_AXIS_FROM_REAR, 60.0)
    vol = (p & env).volume
    if verbose:
        print(f"  case body swept down from the top to its seat: "
              f"{vol:.3f} mm3 overlap ({'OK' if vol < 0.5 else 'FAIL'})")
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
    print("\n== printability (check_printability.py, on its back: web face on the bed) ==")
    CP.STL = os.path.dirname(stl_path)                       # runtime only
    CP.ORIENT["leg_link_v6"] = (np.array(PRINT_ORIENT), "on its back: web face on the bed")
    CP.PRINT_STL["leg_link_v6"] = os.path.basename(stl_path)
    findings = CP.audit("leg_link_v6")
    if findings:
        ok = False

    print("\n== requirement 2: r16 sweep + servo insertion ==")
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
