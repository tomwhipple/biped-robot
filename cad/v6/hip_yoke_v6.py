"""Hip yoke, v6 ONE-PRINT variant (qty 2): the hip-roll clevis (v5
`yoke_roll`) and the hip-pitch clevis (`yoke_pitch_v6`) fused into a single
solid, replacing the two parts that were screwed flange to flange with 4x
M3x10 into 4x M3 heat-set inserts (cad/fasteners.py::flange_bolts).

Local frame == yoke_roll's: the ROLL axis is X through the origin, +X forward
(roll servo horn side), +Z up. The pitch clevis hangs D.ROLL_TO_PITCH below,
pitch axis along Y through (0, 0, -50) -- exactly where the assembly used to
place yoke_pitch with its own translation, so HIP_ROLL_Z, HIP_PITCH_Z and
every other kinematic constant are untouched, and both servo interfaces (roll
horn boss + sunk idler pad on X, pitch horn plate + idler hub on Y, all four
O14 bolt circles, centre reliefs) are the v5 parts' own geometry, not a copy.

WHAT CHANGED vs the two parts
  - the two 4 mm flanges (roll: z -20..-16, pitch: z -24..-20) are one 8 mm
    block; the mating face at z -20 is gone
  - the 4 bolt columns at (+-YOKE_BOLT_SQ/2)^2 -- O3.4 clearance through the
    roll flange, O4.1 heat-set pilots in the pitch flange -- are filled back
    to solid (one cylinder per column, trimmed by the flexion chamfer where
    the front pair grazes it, exactly as the heat-set bores did)
  - the hip-flexion chamfer (V.YOKE_FLEX_CHAMFER, docs/design-v6-ankle-roll.md
    section 11.3) is kept as-is: the thigh's grip plates still sweep the same
    flange corner at -125 deg, and this part's envelope is the union of the
    two it replaces, so the clearance cannot get worse. Re-verified by
    check_assembly_v6.py with HIP_YOKE_VARIANT=single.
  - nothing else. In particular the 8 mm block is NOT thinned: the flange
    thicknesses are what set PITCH_ARM_REACH (26 = 50 - 16 - 2*4), which is
    a kinematic-adjacent constant every hip check is written against.
  - SINCE 2026-09-19 the union is also STYLED -- blended at the four steps
    where the two flange footprints disagree, rounded everywhere that is not
    a seat or a bore, filleted at every arm root, tapered where the plates
    oversail their pads. See the STYLING block below the fused() that
    produces the raw union, and section 7 of the design note. The kinematics,
    the servo interfaces and the flexion chamfer are untouched by it; the one
    clearance that moved is recorded there, to the mm3.

PRINT ORIENTATION: WALL, on edge, model -Y on the bed (check_printability's
RY_ROLL_WALL -- yoke_roll's own orientation), WITH SLICER SUPPORTS + BRIM.

  Why this is the failure mode's orientation, not a compromise. yoke_roll's
  arms snapped when printed flange-down because the layers were stacked
  ALONG the arm (model Z), and a cantilevered arm carries bending as tension
  along its length -- straight across the interlayer bond. The fix was to
  put the arm length in the bed plane. In this frame BOTH clevises have
  their arm length along model Z (roll arms: flange z -16 up to the pads at
  z 0; pitch arms: flange z -24 down to the pads at z -50), so ANY
  orientation with model Z in the bed plane keeps every arm's bending
  tension along the filament. That leaves model +X up (yoke_pitch's RY_XUP)
  or model +Y up (yoke_roll's RY_ROLL_WALL); a 45 deg diagonal would let both
  arm sets lean at exactly the overhang limit but stands the part on one
  edge, so it is out.

  Either way ONE clevis prints as walls and the other as horizontal slabs,
  because the roll plates are normal to X and the pitch plates normal to Y.
  A slab arm is still sound against the recorded failure: its layers are
  planes containing the arm length, so bending tension (in-plane or
  out-of-plane) runs along filament; only a pure pull along the plate
  normal loads the interlayer bond, and that direction is the bolt preload
  (compression onto the disc, the servo body as the spacer). The choice
  between the two is therefore print quality, and +Y up wins on every count:

    - first layer: the pitch IDLER arm (r14 hub + riser) and the pitch
      flange's end land flat on the bed, 823 mm2 (23 % of the footprint)
      vs ~360 mm2 (11 %) for +X up, where only the sunk roll idler pad and
      the roll flange's end touch
    - height: 43.85 mm vs 48.4
    - the ROLL arms -- the ones that broke -- print in exactly their proven
      wall orientation, and the 26 mm pitch idler arm (1.6x the root moment
      of a roll arm) prints as a bed-flat slab, the best case an FDM part
      gets
    - the bolt-circle bores that are horizontal in the print (the roll
      pads', axis X) are the O3.4 ones with a 4 mm-class roof, a NOTE not a
      fail in check_printability; the pitch pads' bores are vertical

  Supports (slicer): the pitch HORN arm is a horizontal slab ~41 mm up over
  the pitch servo void (its underside, y 20.45, is the horn seating face --
  a support-interface finish on a face clamped to a metal disc by 4 screws;
  the +X-up alternative puts the same finish on the ROLL horn boss face
  instead, so this is a wash), the roll pads' lower rims and the roll arm
  plates start in mid-air as in the yoke_roll print, and the roll flange's
  -Y face sits 3.4 mm above the bed. Brim: 44 mm tall on a T of 4 mm walls.
  No modelled fins -- see the note above yoke_roll in cad/parts.py.

    .venv/bin/python cad/v6/hip_yoke_v6.py   # stl/step/renders + audit + mass
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos, chamfer, export_step, export_stl, fillet  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5  # noqa: E402
import yoke_pitch_v6  # noqa: E402
D = V.D

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# check_printability's RY_ROLL_WALL, spelled out (rows = print axes in model
# coords): model +Y -> print +Z, i.e. the part lies on its -Y face.
PRINT_ORIENT = ((1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0),
                (0.0, 1.0, 0.0))
SUPPORT_NOTE = ("supports on + brim: pitch horn arm slab over the servo void, "
                "roll pad rims / arm plates start in mid-air (as yoke_roll)")

# the 4 M3x10 + 4 heat-sets this part removes, per hip (fasteners.flange_bolts)
REMOVED_FASTENERS = {"M3x10 button head": 4, "M3 heat-set insert": 4}


def pitch_offset():
    """Location of the pitch yoke's frame in this part's (roll) frame."""
    return Pos(0, 0, -D.ROLL_TO_PITCH)


def fused():
    """The raw union: the two v5 clevises plus the filled bolt columns, with
    every face either part's own. This is the part as it shipped at afd4daf --
    kept as its own function because it is the reference the styling pass is
    measured against (interface volumes, mass, clearances)."""
    roll = v5.yoke_roll()
    pitch = pitch_offset() * yoke_pitch_v6.yoke_pitch_v6()
    p = roll + pitch
    # Fill the flange bolt columns. Both bores live only in the two flanges
    # (the columns at +-10 miss every arm), so one cylinder from the pitch
    # flange's bottom to the roll flange's top restores solid material; the
    # radius covers the larger (heat-set) bore with a hair of margin so no
    # coincident cylindrical face is left for the fuse to chew on.
    r_fill = max(D.M3_CLEAR, D.HEATSET_D) / 2 + 0.05
    b = D.YOKE_BOLT_SQ / 2
    fill = None
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        c = v5.cyl_z(r_fill, ZF_BOT, ZF_TOP, sx, sy)
        fill = c if fill is None else fill + c
    # the front pair's heat-set bores just grazed the flexion chamfer (bore
    # edge x 12.05 vs the chamfer's foot at x 12.0); trim the fill the same way
    fill -= pitch_offset() * yoke_pitch_v6.flex_chamfer_wedge()
    return p + fill


# ==========================================================================
# STYLING (2026-09-19)
# ==========================================================================
# "This 'merged' part just looks like the two parts are just slapped together
# ... let's make it more organic and streamline all of the sharp corners as
# much as possible."  It did, and for a reason: fused() is a pure boolean
# union, so wherever the two flange FOOTPRINTS disagree the solid keeps a bare
# step. They disagree everywhere, because the two clevises are rotated 90 deg
# to each other:
#
#     roll flange   x -23.95..24.45 (48.40)   y +-17.00      (34.00)
#     pitch flange  x +-16.00       (32.00)   y -20.40..23.45 (43.85)
#
# so the 8 mm block is a PLUS sign in plan: the roll flange overhangs the
# pitch flange by 8.45/7.95 mm fore/aft (a bare downward ledge at z -20) and
# the pitch flange overhangs the roll flange by 6.45/3.40 mm on +-y (a bare
# upward ledge at the same plane). That plane -- the old bolted joint -- was
# what read as "slapped together".
#
# The blend is a COVE at every one of those four reentrant lines (R_WAIST),
# which turns the step into a continuous curved transition from one footprint
# to the other, plus a rounded silhouette everywhere else. The radii are not
# free: this part lives inside two servo sweeps with almost nothing to spare,
# and everything a cove ADDS is material moving toward a neighbour.
#
#   * the roll servo's case corner sweeps 0.08 mm over the roll flange's top
#     face at +-55 deg of hip roll, right across x -17.35..20.45 (this is the
#     known 0.08 mm FAIL row). Nothing may be added above z -16 between the
#     roll arms: that is why the roll arms' INNER root fillets are
#     R_ROOT_SERVO (0.6) and not R_ROOT -- the servo is 1.0 mm away and
#     D.SWEEP_BUFFER is 0.5.
#   * the carrier's bay walls run 1.0 mm off each roll arm's inner face, the
#     same limit from the other side.
#   * the thigh sweeps the roll flange's front rim in deep flexion (it makes
#     contact at -118.40 deg, styled or not), so the FRONT cove is cut back
#     to the flexion chamfer's own plane -- see _flex_relief().
#
# Everything else here is subtractive (rounds and tapers), which can only
# move clearances the right way. Radii live in this module: no dimensions
# file is edited, and nothing outside hip_yoke_v6 changes by a byte.
#
# WHAT THIS COST, measured against fused() with the same mocks
# check_assembly_v6 uses (docs/design-v6/hip-yoke-single-print.md section 7.4):
# roll servo 0.076 mm and bay walls 1.000 mm at +-55 deg -- both unchanged;
# thigh first contact -118.40 deg and 0.700 mm at -117 -- both unchanged;
# -121 better by 1.40 mm3; -125 worse by 2.15 mm3 (+0.8 % on an already
# 271.70 mm3 interference, 7 deg past that hard contact), all of it the R2.5
# fillet at the roll HORN arm's root -- the one blend here that is structural
# rather than cosmetic. Called out, not quietly traded.

R_WAIST = 2.75         # the cove that blends the two flange footprints
R_CORNER = 4.5         # the block's six vertical silhouette corners
R_BROW = 1.0           # flange rims. Small on purpose: at 1.4 (the most a
                       # 4 mm flange can take and still keep D.WALL of flat)
                       # OCC dropped four of the block's rims outright, and
                       # at 1.0 it takes all of them -- measured, not guessed
R_PLATE = 2.0          # arm-plate outline corners
R_RIM = 1.2            # free disc / hub rims (never a fastener seat)
R_ROOT = 2.5           # arm roots with room around them
R_ROOT_SERVO = 0.6     # arm roots facing a servo case or a bay wall
SHOULDER = (1.7, 3.0)  # arm-plate shoulder taper (across, along the arm).
                       # 1.7 is the whole oversail: the plate is AHW (12) half
                       # wide over a r 10 pad, and the taper must stop 0.3
                       # outside the pad so no interface zone is touched.
# HIP_YOKE_TRACE=1 prints a flushed breadcrumb per edge -- the only way to
# name the blend in flight when OCC ends the process instead of raising.
TRACE = bool(os.environ.get("HIP_YOKE_TRACE"))

# the part's own landmarks, spelled out once (all from dimensions.py)
ZF_TOP = -D.ROLL_AXIS_TO_FLANGE                 # -16.00 roll flange top
ZF_MID = ZF_TOP - D.YOKE_FLANGE_T               # -20.00 the old mating plane
ZF_BOT = ZF_MID - D.YOKE_FLANGE_T               # -24.00 pitch flange bottom
RX1 = D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE  # +24.45 roll horn arm, outer
RX0 = D.ROLL_ARM_INNER - D.PLATE                # -23.95 roll idler arm, outer
RXI1 = D.SV_HORN_FACE + D.HORN_BOSS_H           # +21.45 roll horn arm, inner
RXI0 = D.ROLL_ARM_INNER                         # -20.95 roll idler arm, inner
RHY = D.YOKE_FLANGE_Y / 2                       # 17.00 roll flange half-depth
PX = D.YOKE_FLANGE_X / 2                        # 16.00 pitch flange half-width
PY1 = D.SV_HORN_FACE + D.PLATE                  # +23.45 pitch horn arm, outer
PY0 = D.IDLER_ARM_INNER - D.PLATE               # -20.40 pitch idler arm, outer
PYI1 = D.SV_HORN_FACE                           # +20.45 pitch horn arm, inner
PYI0 = D.IDLER_ARM_INNER                        # -17.40 pitch idler arm, inner
AHW = 12.0                                      # arm plate half-width (v5)
PZ = -D.ROLL_TO_PITCH                           # -50.00 pitch axis
ROLL_ARM_X = (RX1, RXI1, RX0, RXI0)             # the four roll plate faces
RISER_X = (-14.0, 4.0)                          # pitch idler riser, v5 literals
HUB_R = 14.0                                    # pitch idler hub, v5 literal


# ------------------------------------------------------------- edge picking
def _pts(e):
    """The edge's ends plus its midpoint -- a plane test that survives an
    earlier fillet trimming the edge, which a centre-only test does not."""
    c = e.center()
    return [(c.X, c.Y, c.Z)] + [(v.X, v.Y, v.Z) for v in e.vertices()]


def _on(e, **planes):
    """True when the whole edge lies on the given x=/y=/z= planes."""
    idx = {"x": 0, "y": 1, "z": 2}
    for ax, val in planes.items():
        i = idx[ax]
        if any(abs(p[i] - val) > 0.03 for p in _pts(e)):
            return False
    return True


def _dir(e, axis):
    """True when the edge is a straight run along x/y/z."""
    try:
        t = e.tangent_at(0.5)
    except Exception:  # noqa: BLE001
        return False
    if str(e.geom_type) != "GeomType.LINE":
        return False
    return abs(abs({"x": t.X, "y": t.Y, "z": t.Z}[axis]) - 1.0) < 1e-3


def _span(e, axis, lo, hi):
    i = {"x": 0, "y": 1, "z": 2}[axis]
    return all(lo - 0.03 <= p[i] <= hi + 0.03 for p in _pts(e))


# ------------------------------------------------------------- the groups
def _groups():
    """(name, radius, predicate) in APPLICATION order: structural roots first,
    then the blends, then the silhouette, then decoration -- so if OCC gives
    up late the part still has the fillets that matter. Each predicate is
    written against planes the part is DEFINED by, never against an edge
    index, so a boolean change upstream orphans nothing silently."""
    g = []

    # 1. ARM ROOTS. Every predicate pins BOTH faces' planes, never just one:
    #    a fillet leaves a tangent line one radius away on each face, and a
    #    one-plane test happily re-selects that line on the next pass (it is
    #    an edge, it is straight, it is on the plane) -- which cannot be
    #    filleted and would be reported as a phantom failure.
    #    Roll plate sides: x is clear of the roll servo (<= 20.45) and of the
    #    bay walls (<= 19.95) entirely, so these take the full R_ROOT.
    g.append(("roll arm root, plate sides", R_ROOT,
              lambda e: _dir(e, "x") and any(_on(e, y=sy * AHW, z=ZF_TOP) for sy in (1, -1))))
    #    Roll plate inner faces: the corner the roll servo's case sweeps 1.0 mm
    #    from at +-55 deg, so this one is R_ROOT_SERVO, not R_ROOT.
    g.append(("roll arm root, servo side", R_ROOT_SERVO,
              lambda e: _dir(e, "y") and any(_on(e, x=vx, z=ZF_TOP) for vx in (RXI1, RXI0))))
    g.append(("pitch horn arm root, plate sides", R_ROOT,
              lambda e: _dir(e, "y") and any(_on(e, x=sx * AHW) for sx in (1, -1))
              and _span(e, "y", PYI1 - 0.1, PY1 + 0.1)
              and _span(e, "z", ZF_BOT - 1.5, ZF_BOT + 0.03)))
    g.append(("pitch horn arm root, servo side", R_ROOT,
              lambda e: _dir(e, "x") and _on(e, y=PYI1, z=ZF_BOT)))
    g.append(("pitch riser root, plate sides", R_ROOT,
              lambda e: _dir(e, "y") and any(_on(e, x=vx, z=ZF_BOT) for vx in RISER_X)
              and _span(e, "y", PY0 - 0.1, PYI0 + 0.1)))
    g.append(("pitch riser root, servo side", R_ROOT,
              lambda e: _dir(e, "x") and _on(e, y=PYI0, z=ZF_BOT)))

    # 1b. the flexion chamfer's foot, FIRST of the convex rims: it is the one
    #    edge where a 45 deg face, a horizontal face and the pitch horn arm's
    #    clipped corner all meet, and rounding it after its neighbours had
    #    been rounded took the process down with SIGSEGV rather than an
    #    exception (2026-09-19). Rounded before them it is routine.
    g.append(("flexion chamfer foot", R_BROW,
              lambda e: _dir(e, "y") and _on(e, z=ZF_BOT, x=PX - V.YOKE_FLEX_CHAMFER)))

    # 2. THE WAIST. Four reentrant lines at the old mating plane: the roll
    #    flange's underside meeting the pitch flange's two side faces (x), and
    #    the roll flange's two side faces meeting the pitch flange's top (y).
    #    These four coves ARE the blend -- they are what turns the step into a
    #    continuous transition between the two footprints.
    g.append(("waist cove, fore/aft", R_WAIST,
              lambda e: _dir(e, "y") and any(_on(e, x=sx * PX, z=ZF_MID) for sx in (1, -1))
              and _span(e, "y", -RHY, RHY)))
    g.append(("waist cove, sides", R_WAIST,
              lambda e: _dir(e, "x") and any(_on(e, y=sy * RHY, z=ZF_MID) for sy in (1, -1))
              and _span(e, "x", -PX, PX)))

    # 3. the block's vertical silhouette corners, BEFORE the rims that run
    #    into them. Order was measured, not assumed: rims first leaves each
    #    4 mm corner trimmed to 1.2-2.6 mm between two blends and OCC then
    #    refuses it at every radius on the ladder (4 of the 6 corners were
    #    lost that way), while corners first costs only the corner ARCS,
    #    which group 5 picks up again. The pitch flange's FRONT corners are
    #    not here at all: the flexion chamfer already took them, and what is
    #    left of them is its two diagonal side edges.
    g.append(("block corners, roll flange", R_CORNER,
              lambda e: _dir(e, "z")
              and any(_on(e, x=vx, y=sy * RHY) for vx in (RX1, RX0) for sy in (1, -1))))
    g.append(("block corners, pitch flange", R_CORNER,
              lambda e: _dir(e, "z") and any(_on(e, x=-PX, y=vy) for vy in (PY1, PY0))))
    g.append(("flexion chamfer corners", R_PLATE,
              lambda e: str(e.geom_type) == "GeomType.LINE"
              and not (_dir(e, "x") or _dir(e, "y") or _dir(e, "z"))
              and (_on(e, y=PY1) or _on(e, y=PY0))
              and _span(e, "z", ZF_BOT - 1.5, ZF_MID + 0.03) and _span(e, "x", 10.0, PX + 0.03)))

    # 4. the block's brow: every convex horizontal rim of the two flanges.
    #    R_BROW leaves D.WALL of flat flange behind the round.
    g.append(("flange rim, roll top", R_BROW,
              lambda e: _dir(e, "x") and any(_on(e, y=sy * RHY, z=ZF_TOP) for sy in (1, -1))))
    g.append(("flange rim, roll top ends", R_BROW,
              lambda e: _dir(e, "y") and any(_on(e, x=vx, z=ZF_TOP) for vx in (RX1, RX0))))
    g.append(("flange rim, roll ledge", R_BROW,
              lambda e: _dir(e, "y") and any(_on(e, x=vx, z=ZF_MID) for vx in (RX1, RX0))))
    g.append(("flange rim, roll ledge sides", R_BROW,
              lambda e: _dir(e, "x") and any(_on(e, y=sy * RHY, z=ZF_MID) for sy in (1, -1))
              and not _span(e, "x", -PX, PX)))
    g.append(("flange rim, pitch top", R_BROW,
              lambda e: _dir(e, "x") and any(_on(e, y=vy, z=ZF_MID) for vy in (PY1, PY0))))
    g.append(("flange rim, pitch top ends", R_BROW,
              lambda e: _dir(e, "y") and any(_on(e, x=sx * PX, z=ZF_MID) for sx in (1, -1))
              and not _span(e, "y", -RHY, RHY)))
    g.append(("flange rim, pitch bottom", R_BROW,
              lambda e: (_dir(e, "x") or _dir(e, "y"))
              and (_on(e, x=-PX, z=ZF_BOT) or _on(e, y=PY1, z=ZF_BOT) or _on(e, y=PY0, z=ZF_BOT))))

    # 5. what group 3 left behind: the quarter-circle arcs where each rounded
    #    corner meets the block's top, waist and bottom planes. Rounding them
    #    is what makes the corner read as a solid blob rather than a rounded
    #    extrusion -- the last flat/round crease in the silhouette.
    g.append(("block corner arcs", R_BROW,
              lambda e: str(e.geom_type) == "GeomType.CIRCLE"
              and any(_on(e, z=zz) for zz in (ZF_TOP, ZF_MID, ZF_BOT))
              and abs(e.radius - R_CORNER) < R_CORNER * 0.9))

    # 6. arm-plate outline corners, and the pitch idler hub's free rim.
    #    NOT the pitch horn plate's inner (y = PYI1) corners -- that face is
    #    the horn DISC SEAT. NOT the roll pads' rims either: with the bolt
    #    circle at r 7 an M3 head's edge lands at r 10.0, the pad's own
    #    radius, so a round there would undercut a fastener seat. Both are
    #    listed in the styling section of the design note rather than left
    #    unexplained.
    g.append(("roll arm plate corners", R_PLATE,
              lambda e: _dir(e, "z")
              and any(_on(e, x=vx, y=sy * AHW) for vx in ROLL_ARM_X for sy in (1, -1))))
    g.append(("pitch horn plate corners", R_PLATE,
              lambda e: _dir(e, "z") and any(_on(e, x=sx * AHW, y=PY1) for sx in (1, -1))))
    g.append(("pitch riser corners", R_PLATE,
              lambda e: _dir(e, "z")
              and any(_on(e, x=vx, y=vy) for vx in RISER_X for vy in (PY0, PYI0))))
    g.append(("pitch idler hub rim", R_RIM,
              lambda e: str(e.geom_type) == "GeomType.CIRCLE" and _on(e, y=PY0)
              and all(abs(math.hypot(p[0], p[2] - PZ) - HUB_R) < 0.05 for p in _pts(e))))
    return g


LADDER = (1.0, 0.7, 0.5, 0.35, 0.2)      # radius fallbacks, as a fraction of r


def _blend_one(p, e, r):
    """Round ONE edge: target radius first, then down LADDER, then the same
    ladder as a CHAMFER (leg_link_v6's fallback -- OCC accepts a chamfer on
    plenty of edges it refuses to fillet, and a 45 deg break still reads as
    streamlined next to a raw arris). Returns (solid, radius, "fillet"/
    "chamfer") or (solid, None, None) if the edge survived everything.

    Edges lying in the BED PLANE (y = PY0 in PRINT_ORIENT) are chamfered
    FIRST, not as a fallback: a fillet there starts horizontal at the first
    layer -- an unsupported 90 deg overhang around the part's own bed contact
    -- where a chamfer is a printable 45."""
    ops = [(fillet, "fillet"), (chamfer, "chamfer")]
    if _on(e, y=PY0):
        ops.reverse()
    for op, kind in ops:
        for f in LADDER:
            try:
                return op([e], r * f), r * f, kind
            except Exception:  # noqa: BLE001 -- OCC refuses blends for a dozen
                continue      # reasons; the only useful answer is "try smaller"
    return p, None, None


def _round_edges(p, verbose=True):
    """Fillet each group ONE EDGE AT A TIME, reporting anything it could not
    round.

    One edge at a time, deliberately. Filleting a group in a single call
    blends the corners where its edges meet and is the prettier result -- but
    on THIS solid (two clevises fused, then 60-odd blends layered on top) a
    group call does not merely fail, it takes the process down: OCC
    segfaulted on the four pitch-flange bottom rims, exit 139, no STL, no
    traceback. A crash cannot be caught in process, so the group path is
    gone. Each edge gets its target radius, then LADDER fractions of it, and
    an edge that survives none of them is SKIPPED AND REPORTED with its
    location and length -- never dropped quietly.

    Everything is re-queried from the CURRENT solid at the start of each
    group, so a blend that moved an edge is seen, not stale."""
    skipped = []
    for name, r, test in _groups():
        try:
            edges = [e for e in p.edges() if test(e)]
        except Exception as ex:  # noqa: BLE001
            print(f"  [hip_yoke_v6] edge query failed for '{name}' ({type(ex).__name__}); skipped")
            continue
        if not edges:
            print(f"  [hip_yoke_v6] '{name}': no edges matched (geometry changed?) -- left sharp")
            continue
        n, got, chamfered = len(edges), [], 0
        if TRACE:
            print(f"  [hip_yoke_v6] -> {name} ({n} edge(s), R{r:.2f})", flush=True)
        for e in edges:
            c, L = e.center(), e.length
            if TRACE:
                # OCC can still take the process down; leave a flushed
                # breadcrumb naming the edge in flight:
                #   HIP_YOKE_TRACE=1 .venv/bin/python cad/v6/hip_yoke_v6.py
                print(f"  [hip_yoke_v6]    edge ({c.X:7.2f},{c.Y:7.2f},{c.Z:7.2f}) len {L:6.2f}", flush=True)
            p, rr, kind = _blend_one(p, e, r)
            if rr is None:
                skipped.append((name, (c.X, c.Y, c.Z), L))
            else:
                got.append(rr)
                chamfered += kind == "chamfer"
        if verbose:
            worst = f", smallest R{min(got):.2f}" if got and min(got) < r - 1e-9 else ""
            print(f"  [hip_yoke_v6] {name}: {len(got)}/{n} edge(s) at R{r:.2f}{worst}"
                  f"{f', {chamfered} chamfered' if chamfered else ''}"
                  f"{f', {n - len(got)} SKIPPED' if n - len(got) else ''}")
    return p, skipped


def _shoulder_reliefs():
    """leg_link_v6's corner-cut idiom, applied to the clevis plates: each arm
    is a rectangle AHW (12) half-wide carrying a O PAD_D (r 10) pad on its
    end, so the two corners where the rectangle oversails the pad carry no
    load at all -- the bolt circle is r 7 and the bending goes down the
    middle. Taper them back toward the pad's tangent (SHOULDER, 1.7 across x
    3.0 along). The cuts never reach inside r 10, so no pad, bolt circle,
    boss or relief is touched.

    NOT on the roll IDLER arm. That plate is the one with the sunk pad
    (ROLL_IDLER_PAD_SINK 1.35 of its 3.0), so behind the recess it is already
    a 1.65 mm floor; cutting its shoulder as well left a feather edge where
    the taper ran tangent to the sink, and check_printability picked it up as
    a 2.6 mm2 patch of wall under 0.85 mm -- a wall this part is not allowed
    to grow. Its shoulders stay square, and the arm reads a little different
    from its partner, which it already does: it is the sunk one."""
    across, along = SHOULDER
    cut = None

    def add(s):
        nonlocal cut
        cut = s if cut is None else cut + s

    # apex 0.3 outside the pad radius: _interface_zones() is drawn at
    # PAD_D/2 + 0.2, so the taper provably never reaches a servo interface
    pr = D.PAD_D / 2 + 0.3
    assert AHW - pr >= across - 1e-9, "shoulder taper wider than the pad oversail"
    for sy in (1, -1):
        prof = [(sy * pr, 0.0), (sy * AHW, 0.0), (sy * AHW, -along)]
        add(v5.wedge_x(prof, RXI1 - 0.01, RX1 + 0.01))     # roll horn arm
    for sx in (1, -1):
        prof = [(sx * pr, PZ), (sx * AHW, PZ), (sx * AHW, PZ + along)]
        add(v5.wedge_y(prof, PYI1 - 0.01, PY1 + 0.01))     # pitch horn arm
    return cut


# the servo interfaces, as volumes: the guard below asserts the styling pass
# leaves every one of them byte-identical to fused()'s. r 10.2 is the pad
# radius plus a hair -- the pads, both bolt circles (r 7 + O3.4), both bosses,
# the sunk idler pad and every centre relief live inside it.
def _interface_zones():
    r = D.PAD_D / 2 + 0.2
    z = v5.cyl_x(r, D.SV_HORN_FACE - 0.1, RX1 + 0.1, 0, 0)
    z += v5.cyl_x(r, RX0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    z += pitch_offset() * v5.cyl_y(r, D.SV_HORN_FACE - 0.1, PY1 + 0.1, 0, 0)
    z += pitch_offset() * v5.cyl_y(r, PY0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    return z


def _flex_relief():
    """The flexion chamfer's own plane, re-cut across the whole flange band
    AFTER the blends.

    The front waist cove is the one blend that would otherwise put material
    outboard of that plane: it fills the reentrant corner under the roll
    flange's front overhang, which is the corner the chamfer was cut to keep
    clear in the first place. Clipping it back to the chamfer's plane costs
    nothing (it is a cove, not structure), guarantees the front of the flange
    band is no fuller than fused() left it, and reads better anyway -- the
    cove dies into the chamfer instead of bulging past it.

    Strictly z ZF_BOT..ZF_MID, so the chamfer itself (which runs on down past
    the pitch flange to clip the horn arm's corner at z -25) is untouched,
    and the roll flange above ZF_MID keeps every millimetre it had."""
    c = V.YOKE_FLEX_CHAMFER
    return v5.wedge_y([(PX - c, ZF_BOT), (RX1 + 2, ZF_BOT),
                       (RX1 + 2, ZF_MID), (PX, ZF_MID)], PY0 - 1, PY1 + 1)


def hip_yoke_v6(style=True, verbose=True):
    """The one-print hip yoke, styled (see the STYLING block above). Pass
    style=False for the raw fused solid fused() -- the A/B reference."""
    p = fused()
    if not style:
        return p
    p -= _shoulder_reliefs()
    p, skipped = _round_edges(p, verbose=verbose)
    p -= _flex_relief()
    globals()["LAST_SKIPPED"] = skipped
    if skipped:
        print(f"  [hip_yoke_v6] {len(skipped)} edge(s) NOT rounded:")
        for name, c, L in skipped:
            print(f"      {name:34s} at ({c[0]:7.2f},{c[1]:7.2f},{c[2]:7.2f})  length {L:6.2f}")
    return p


LAST_SKIPPED = []


def check_interfaces(styled=None, raw=None, verbose=True):
    """HARD GATE: every servo interface must come through the styling pass
    untouched. Compares the volume of the part inside _interface_zones()
    before and after -- any fillet that had eaten into a pad, a boss, a bolt
    circle or a centre relief moves it."""
    raw = fused() if raw is None else raw
    styled = hip_yoke_v6(verbose=False) if styled is None else styled
    z = _interface_zones()
    a, b = (raw & z).volume, (styled & z).volume
    ok = abs(a - b) < 0.01     # 0.01 mm3 = boolean noise on a 3909 mm3
                               # intersection; a fillet that clipped even the
                               # rim of a O20 pad would move it by ~1 mm3
    if verbose:
        print(f"  [hip_yoke_v6] servo interfaces: {a:.4f} mm3 raw vs {b:.4f} mm3 styled -> "
              f"{'IDENTICAL' if ok else '** CHANGED **'}")
    return ok


def disc_seat_zones():
    """The volumes of this part that are DESIGNED to touch a neighbour, for
    check_assembly_v6's buffer rule (in this frame): the roll idler boss,
    which rides the yaw_carrier's bay bore at the 0.30 slip fit and seats on
    the idler disc, and the roll horn boss, which seats on the horn disc.
    Subtracting these from the posed part lets everything else -- arms,
    pads, both flanges, the whole pitch clevis -- be held to the full
    D.SWEEP_BUFFER against the carrier and the roll servo, which is
    STRICTER than the split-part check (yoke_roll vs carrier was
    volume-only there)."""
    idler = v5.cyl_x(D.IDLER_BOSS_D / 2 + 0.5, D.ROLL_ARM_INNER - 0.01,
                     D.SV_IDLER_FACE + 0.1, 0, 0)
    horn = v5.cyl_x(D.HORN_BOSS_D / 2 + 0.5, D.SV_HORN_FACE - 0.1,
                    D.SV_HORN_FACE + D.HORN_BOSS_H + 0.01, 0, 0)
    return idler + horn


def mass_g(solid):
    return solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def audit(stl_path):
    """check_printability.py on the exported STL, in PRINT_ORIENT, with the
    slicer-support waiver the yokes already have (runtime overrides only --
    v5's tables are not edited)."""
    import numpy as np
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT["hip_yoke_v6"] = (np.array(PRINT_ORIENT), "WALL: on edge like yoke_roll (-Y on the bed) + supports, brim")
    CP.PRINT_STL["hip_yoke_v6"] = os.path.basename(stl_path)
    CP.SUPPORTED["hip_yoke_v6"] = SUPPORT_NOTE
    return CP.audit("hip_yoke_v6")


def render(stl_path, png_path, px=640):
    """MuJoCo offscreen render (iso / bottom / back), like leg_link_v6.
    MUJOCO_GL must be set before mujoco is imported (cgl on a Mac)."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco  # noqa: E402
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
    for az, el in ((135, -30), (90, 89), (180, -15)):
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = center, 2.6 * radius
        cam.azimuth, cam.elevation = az, el
        ren.update_scene(data, cam)
        frames.append(ren.render())
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


if __name__ == "__main__":
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    p = hip_yoke_v6()
    stl_path = os.path.join(OUT_STL, "hip_yoke_v6.stl")
    export_stl(p, stl_path)
    export_step(p, os.path.join(OUT_STEP, "hip_yoke_v6.step"))
    bb = p.bounding_box()
    print(f"hip_yoke_v6  bbox x {bb.min.X:.2f}..{bb.max.X:.2f}  y {bb.min.Y:.2f}..{bb.max.Y:.2f}  "
          f"z {bb.min.Z:.2f}..{bb.max.Z:.2f} mm  (print height {bb.size.Y:.2f} on edge)")
    # mass: before (two parts) vs after (one), same density assumption
    r, q = v5.yoke_roll(), yoke_pitch_v6.yoke_pitch_v6()
    raw = fused()
    print(f"volume  yoke_roll {r.volume:8.1f} + yoke_pitch_v6 {q.volume:8.1f} = {r.volume + q.volume:8.1f} mm3"
          f"  ->  hip_yoke_v6 {p.volume:8.1f} mm3  (+{p.volume - r.volume - q.volume:.1f} = the 4 filled bolt columns)")
    print(f"styling fused {raw.volume:8.1f} mm3 {mass_g(raw):6.2f} g  ->  styled {p.volume:8.1f} mm3 "
          f"{mass_g(p):6.2f} g   ({p.volume - raw.volume:+.1f} mm3, {mass_g(p) - mass_g(raw):+.3f} g: "
          f"blends add, rounds and tapers take away)")
    print(f"mass    PETG {D.FILAMENT_RHO * 1e3:.2f} g/cm3 x {D.PRINT_MASS_FACTOR} print factor: "
          f"{mass_g(r) + mass_g(q):.2f} g -> {mass_g(p):.2f} g per hip, plus the removed hardware "
          f"{REMOVED_FASTENERS}")
    check_interfaces(p, raw)
    findings = audit(stl_path)
    print("printability:", "PASS" if not findings else f"{len(findings)} finding(s) above")
    print("wrote", render(stl_path, os.path.join(OUT_REN, "hip_yoke_v6.png")))
