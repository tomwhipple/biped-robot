"""ankle_link (qty 2, same part both legs) -- grips the ankle-PITCH servo case
exactly the way leg_link grips its servo (the knee hangs inside the shin; the
ankle-pitch servo hangs inside this link the same way), then forks DOWN in the
X-Z plane onto the ankle-ROLL servo, which lies across the foot with its
output axis along +X.

Local frame: pitch axis == Y axis through the origin (identical convention to
leg_link); roll axis == X axis at z = -AL_DROP; +X forward.

Upper half (z from GRIP_TOP_HORN down to the top plate) IS leg_link's grip
channel, copied unchanged from cad/parts.leg_link: web on -x, both grip
plates with the rib/platform detents and ramps, 6x countersunk M2.5 case-grip
screws, horn/idler centre reliefs, jog blocks, cable window. Every number
comes from ../dimensions.py (D) -- nothing here re-derives a servo constant.

Lower half is new: a top plate under the grip plates (V.AL_TOP_PLATE_Z/_T)
closing the channel's floor and reaching out to both tines; an inboard web
(V.AL_WEB_Y, -y: the roll servo's long/cable side is OUTBOARD = +y so the web
lives on the short/output-end side, D.FIT clear of it) tying the tines
together fore-aft; and two tines in the X-Z plane at V.AL_TINE_FRONT_X (front,
bolts to the roll servo's HORN disc) and V.AL_TINE_REAR_X (rear, IDLER disc),
each ending in an O20 pad on the roll axis with the O14 BCD (4x M3 clearance,
teardropped print-up since these bores are VERTICAL in the chosen print
orientation -- see PRINT_ORIENT), a centre relief and a tapered pad boss.

Print: RY_XUP (model +X -> print +Z), the SAME transform leg_link uses --
required because the grip channel's case-screw bores are copied verbatim with
their teardrop roofs already tuned to that orientation (roll=90, i.e.
print-up = model +x). SLICER SUPPORTS REQUIRED (see cad/v6/audit_ankle_foot.py):
the top-plate/tine block below the grip channel has no support-free
orientation once the ankle-roll clearance cut is applied.

    .venv/bin/python cad/v6/ankle_link.py
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import *  # noqa: E402
import dimensions_v6 as V  # noqa: E402
D = V.D
import parts  # noqa: E402  (v5 helpers: box/cyl_*/teardrop_*/csk_*/bcd_*/wedge_*)
import check_assembly as CA  # noqa: E402  (real servo mocks)

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")

AL_DROP = V.AL_DROP

# ---------------------------------------------------------------------------
# top-plate / tine layout, derived from V (dimensions_v6.py) -- not restated
# there because these are THIS PART's footprint choices, not shared numbers.
TP_Z0 = V.AL_TOP_PLATE_Z                          # -36.6 top face
TP_Z1 = TP_Z0 - V.AL_TOP_PLATE_T                  # -39.2 bottom face
TP_X = (V.AL_TINE_REAR_X[0] - 1.0, V.AL_TINE_FRONT_X[1])   # -20.80..23.45

TINE_HALF_W = V.AL_TINE_W / 2                     # 12.0, centred on the roll axis
# The pad boss (O20, r=10) must fit inside the tine's half-width with a rim;
# 12.0 leaves 2.0 mm, the same class leg_link's own fork plates run.
#
# EVERYTHING AT OR BELOW GRIP_BOT (-36, where the grip channel's own plates
# stop) is kept to this NARROW width, not the grip channel's own +-21ish
# span: the ankle-roll servo's case swings its long (cable) side up into
# that wider band at the ROM extremes (2026-09-14 finding -- a full-width
# top plate at AL_TOP_PLATE_Z reads ~2000 mm3 of interference at +-25 deg).
# The grip channel's own back web stays FULL WIDTH but only down to
# GRIP_BOT, which the servo sweep never reaches (its swept top is ~2.5 mm
# below GRIP_BOT even on the SHORT (10.11) side; the long side needs the
# full narrow treatment below).
TP_Y = (-TINE_HALF_W, TINE_HALF_W)                # -12..12, the tines' own width


def _grip_channel():
    """leg_link's grip channel (v5, verbatim -- see cad/parts.leg_link), minus
    everything that belongs to leg_link's Y-oriented fork (cross brace, fork
    arms, permanent slab backing, lower-joint pad holes, sub-axis trim). The
    web is EXTENDED down to TP_Z1 so it meets the top plate directly (v5's
    web ran to WEB_END = 32 mm above the lower axis; here it runs all the way
    to the new top plate, which starts far closer)."""
    t = D.GRIP_PLATE_T
    web_x1 = -12.36 - D.WEB_GAP                     # web inner face (cable gap)
    web_x0 = web_x1 - 2.4                           # web outer face, -15.16
    hy0, hy1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE          # 20.45..23.45
    iy1, iy0 = D.IDLER_ARM_INNER, D.IDLER_ARM_INNER - D.PLATE    # -18..-21
    grip_x1 = 13.2
    p = parts.box(web_x0, grip_x1, D.SV_TOPFACE, D.SV_TOPFACE + t,
                  D.GRIP_BOT, D.GRIP_TOP_HORN)
    p -= parts.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    idler_seat = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR          # -14.90
    _igo = idler_seat - D.GRIP_PLATE_T_IDLER                     # -17.90 outer
    p += parts.box(web_x0, grip_x1, _igo, idler_seat, D.GRIP_BOT, D.GRIP_TOP_IDLER)
    # back web (v5's own z-range, GRIP_BOT..WEB_TOP -- NOT extended further
    # down: below GRIP_BOT this Y-width (-21..19.75) reaches into the ankle-
    # roll servo's swept envelope at the ROM extremes; see the narrow top
    # plate / tine web below, which is what continues the structure down).
    p += parts.box(web_x0, web_x1, iy0, D.SV_TOPFACE + t, D.GRIP_BOT, D.WEB_TOP)
    # leg_link's jog blocks (joining the horn/idler grip plates out to Y
    # hy1/iy0) are DROPPED here, not copied: their job in leg_link is to
    # reach a fork plate at that same wide Y that only exists in leg_link's
    # own Y-oriented fork. ankle_link's fork is the X-oriented tine pair at
    # Y +-12 (TINE_HALF_W), which the back web (just above) already reaches
    # -- a jog block to Y 23.45 here is unused material, and at AL_DROP=57
    # it is exactly what the foot's tab sweeps into at ankle-roll extremes
    # (2026-09-14 finding, 1.05 mm3 at +25 deg). Everything else in this
    # function is still leg_link's, unchanged.
    # DETENT for the servo's horn-side RIB (v5, verbatim)
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
    # DETENT for the idler-side PLATFORM (v5, verbatim)
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0 = idler_seat
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= parts.box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    _irr = 1.2 * (_iy0 - _iy1)
    p -= parts.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)],
                       _iz0 - 0.6, _iz1 + 0.6)
    # case-grip screw holes (M2.5 flat, countersunk flush -- v5, verbatim)
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1,
                                  D.SV_TOPFACE + t + 1, lx, -zrow, roll=90)
            p -= parts.csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        p -= parts.teardrop_y(D.CASE_SCREW_CLEAR / 2, _igo - 1,
                              idler_seat + 1, lx, -D.CASE_HOLES_BOT[1], roll=90)
        p -= parts.csk_y(lx, -D.CASE_HOLES_BOT[1], _igo, -1)
        p -= parts.teardrop_y(D.CASE_CS_D / 2 + 0.4, iy0 - 1, _igo + 0.05,
                              lx, -D.CASE_HOLES_BOT[1], roll=90)
    p -= parts.cyl_y(D.HORN_CENTER_RELIEF_D / 2, hy0 - 1, hy1 + 1, 0, 0)
    p -= parts.cyl_y(D.IDLER_CENTER_RELIEF_D / 2, D.SV_IDLER_FACE - 6,
                     D.SV_IDLER_FACE + 0.7, 0, 0)
    # cable pass-through: placed low in the back web, just above GRIP_BOT --
    # position is approximate (the case's real port band is close to the
    # idler face; verify against a real servo before trusting the exact z).
    p -= parts.box(web_x0 - 1, web_x1 + 1, -4.5, 4.5, D.GRIP_BOT + 1.0, D.GRIP_BOT + 8.0)
    return p


def _roll_sweep_clearance_cut():
    """The ankle-roll servo's swept envelope about the roll axis, ANGLE-PADDED
    by 1 deg beyond V.ROM['ankle_roll'] (>> D.SWEEP_BUFFER at this ~54 mm
    radius: 1 deg of arc is ~0.9 mm) and unioned over BOTH mounting
    orientations (+90/-90 about X) -- ankle_link is the SAME PART on both
    legs (no mirror), but the foot IS mirrored, so the roll servo's long
    (cable) side ends up on +y for one leg and -y for the other. This part
    has to clear it either way.

    2026-09-14 finding: at AL_DROP = 54 mm, the servo's swept top edge
    reaches ~19-23 mm above the roll axis depending on how far outboard the
    material extends in y (worse further out) -- HIGHER than the design
    doc's own single-corner estimate (18 mm at GRIP_BOT), because the
    corner that matters at wide y is the case's LONG (cable) end, not the
    short end the doc's formula used. leg_link's own back web (copied
    verbatim, full Y width down to GRIP_BOT) does not clear this on its
    own; this cut is what makes it clear, and it costs a sliver of the
    idler-side jog block / grip-plate corner nearest the case end -- see
    ankle_link()'s docstring note."""
    lo, hi = V.ROM["ankle_roll"]
    pad = 1.0
    # NOTE: dilating the WHOLE mock (offset()) before sweeping was tried and
    # reverted (2026-09-14) -- it also eats the disc faces the tine pads are
    # DESIGNED to sit flush against (zero clearance, bolted), leaving ~95% of
    # the O20 pad missing even at 0 deg. The guard has to apply to the sweep
    # only, not to the seated contact; see the local per-facet thickening in
    # ankle_link() instead.
    cut = None
    for sgn in (1, -1):
        base = Pos(0, 0, -AL_DROP) * (Rot(sgn * 90, 0, 0) * CA.servo_mock_x())
        deg = lo - pad
        while deg <= hi + pad + 1e-6:
            m = Pos(0, 0, -AL_DROP) * Rot(deg, 0, 0) * Pos(0, 0, AL_DROP) * base
            cut = m if cut is None else cut + m
            deg += 2.0
    return cut


def ankle_link():
    p = _grip_channel()

    # --- top plate: floor of the grip channel, reaching out to both tines.
    # Narrow (TP_Y, +-12) below GRIP_BOT -- see the note above. A thin filler
    # closes the 0.6 mm reveal AL_TOP_PLATE_Z leaves under GRIP_BOT so the
    # plate is solidly connected to the (full-width, higher-up) back web
    # rather than floating 0.6 mm under it.
    p += parts.box(TP_X[0], TP_X[1], TP_Y[0], TP_Y[1], TP_Z1, TP_Z0)
    p += parts.box(TP_X[0], TP_X[1], TP_Y[0], TP_Y[1], TP_Z0, D.GRIP_BOT)

    # --- two tines, X-Z blades ending in an O20 pad on the roll axis
    for x0, x1, relief_d in ((V.AL_TINE_FRONT_X[0], V.AL_TINE_FRONT_X[1],
                              D.HORN_CENTER_RELIEF_D),
                             (V.AL_TINE_REAR_X[0], V.AL_TINE_REAR_X[1],
                              D.IDLER_CENTER_RELIEF_D)):
        p += parts.box(x0, x1, -TINE_HALF_W, TINE_HALF_W, -AL_DROP, TP_Z1)
        p += parts.cyl_x(D.PAD_D / 2, x0, x1, 0, -AL_DROP)
        # taper the pad boss OD so its print-underside band stays <=45 deg
        # (leg_link's idler-boss trick, ported to the X axis)
        p += Pos((x0 + x1) / 2, 0, -AL_DROP) * Rot(0, 90, 0) * Cone(
            D.PAD_D / 2 - 1.5, D.PAD_D / 2, 1.5)
        p -= parts.cyl_x(relief_d / 2, x0 - 1, x1 + 1, 0, -AL_DROP)
        for h in parts.bcd_x(x0 - 1, x1 + 1, 0, -AL_DROP):
            p -= h

    # --- inboard web tying the tines together (V.AL_WEB_Y): fore-aft plate
    # filling the gap between the two tines' inner faces, from the top plate
    # down to just above the pads (clear of the O20 boss taper).
    web_z0 = TP_Z1
    web_z1 = -AL_DROP + D.PAD_D / 2 + 3.0
    p += parts.box(V.AL_TINE_REAR_X[1], V.AL_TINE_FRONT_X[0],
                   V.AL_WEB_Y[0], V.AL_WEB_Y[1], web_z1, web_z0)

    # NOTE: no separate gusset is needed to tie the top plate's rear reach
    # (TP_X[0] = -20.8, 5.6 mm past the grip channel's own back web at
    # -15.16) back to the grip channel -- the top plate itself already spans
    # the full TP_X range at TP_Y, and the filler box above closes the
    # 0.6 mm reveal up to the back web, so the two are one connected mass
    # with a flush base in the RY_XUP print orientation.

    # --- local reinforcement at the seven spots the roll-sweep cut leaves
    # < 0.85 mm thin (2026-09-14, found by check_printability + confirmed by
    # direct density probing at 10-47%, i.e. genuinely thin, not a ray-cast
    # artifact off the cut's curved boundary). A small sphere at each,
    # BEFORE the cut, merges into whichever wall is there and thickens it;
    # applying the cut AFTER these means any bit that would still violate
    # clearance is trimmed straight back off, so this can't reopen the
    # servo-sweep interference the cut exists to prevent -- it can only make
    # an already-clear wall thicker. Coordinates are this build's; re-audit
    # after any dimensions_v6.py change to AL_DROP/AL_TOP_PLATE_*/AL_TINE_*.
    # NOTE: a reinforcement at (0, 19.75, -21.6) was tried and dropped -- a
    # direct 1-D thickness probe there (scanning Y at fixed x, z) found the
    # horn plate already 100% solid across its full 2.4 mm nominal
    # thickness; the 95.5 mm2 finding check_printability reported at that
    # spot is a ray-cast false positive (the inward ray exits through a
    # distant near-parallel surface the sweep cut exposed elsewhere, not
    # through a genuinely thin local wall). Verified with the tool, not
    # assumed -- see the audit log.
    for x, y, z in ((-15.16, -17.64, -35.88), (-16.65, -12.14, -41.49),
                   (2.43, -19.76, -33.61), (-16.7, -13.01, -42.4),
                   (-14.75, -17.64, -35.27), (2.39, -18.73, -33.0),
                   (-2.79, -18.58, -33.0), (-14.75, 17.75, -35.21)):
        p += Pos(x, y, z) * Sphere(3.0)

    # --- clear the ankle-roll servo's swept envelope (both orientations,
    # ROM +buffer) -- see _roll_sweep_clearance_cut(). Applied last, before
    # rounding, so it also softens whatever it exposes.
    p -= _roll_sweep_clearance_cut()

    # the clearance cut can leave numerical-noise slivers (boolean fuzz,
    # typically << 1 mm3) disconnected from the main body; keep only the
    # part that actually IS the link, and flag anything that isn't tiny.
    solids = sorted(p.solids(), key=lambda s: s.volume, reverse=True)
    if len(solids) > 1:
        junk = sum(s.volume for s in solids[1:])
        tag = "boolean fuzz" if junk < 1.0 else "** REAL DETACHED VOLUME **"
        print(f"  [ankle_link] {len(solids)-1} extra solid(s) after the sweep "
              f"cut, {junk:.3f} mm3 total ({tag})")
        p = solids[0]

    # --- round the external edges (R4-5 asked for; a blanket fillet() over
    # every edge on a solid this feature-dense is not just fragile -- OCC
    # crashed the process outright on one attempt during development, not
    # merely raised. So this touches ONLY the four known vertical corner
    # edges of the top-plate/tine block (identified by exact XY location,
    # nowhere near a bore), one at a time, each in its own try/except so a
    # single bad edge can't cost the others.
    corners = [(TP_X[0], TP_Y[0]), (TP_X[0], TP_Y[1]),
              (TP_X[1], TP_Y[0]), (TP_X[1], TP_Y[1])]
    rounded = 0
    for cx, cy in corners:
        for e in p.edges():
            try:
                if (abs(e.vertices()[0].X - cx) < 0.01
                        and abs(e.vertices()[0].Y - cy) < 0.01
                        and abs(e.vertices()[1].X - cx) < 0.01
                        and abs(e.vertices()[1].Y - cy) < 0.01):
                    p = fillet([e], 3.0)
                    rounded += 1
                    break
            except Exception:  # noqa: BLE001
                continue
    if rounded:
        print(f"  [ankle_link] rounded {rounded}/4 outer top-plate corners")
    else:
        print("  [ankle_link] corner rounding skipped (no safe edge found)")
    return p


# ---------------------------------------------------------------------------
PRINT_ORIENT = "RY_XUP"    # model +X -> print +Z; identical to leg_link's,
                            # required so the copied grip-channel bores (their
                            # teardrop roofs already assume print-up = model
                            # +x) stay self-supporting.

SERVO_INSERT = {
    # the ankle-pitch servo enters the grip channel exactly like leg_link's
    # (top mouth at GRIP_TOP_HORN, slide down from the cable end) -- +z
    "ankle_link_grip": ((0, 0, 1), 32.0),
    # the roll servo's horn/idler discs are offered UP into the tine pads
    # from below (the tines fork DOWN onto the servo already seated in the
    # foot cradle) -- -z, i.e. the link drops onto the servo
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
    igo = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR - D.GRIP_PLATE_T_IDLER
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        s.append({"name": f"grip_idler_{lx:+.1f}", "kind": "M2.5x8 flat self-tap",
                  "pos": (lx, igo, -D.CASE_HOLES_BOT[1]), "axis": (0, 1, 0), "length": 8.0})
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
