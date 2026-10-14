"""Hip yoke, GLUED variant (qty 2 of each piece): the hip-roll and hip-pitch
clevises as two U-channels that print WITHOUT SUPPORTS, snap together and are
glued. The test part for building printed links from support-free U-channels
(Tom, 2026-10-13: the one-print parts need supports that are hard to remove;
"bridging to the baseplate is OK, but I'd like to avoid large internal
bridges. (holes are also OK)").

Local frame == hip_yoke_v6's (yoke_roll's): roll axis X through the origin,
+X forward, pitch axis along Y through (0, 0, -50). Both pieces are cut from
the v5 clevises (cad/parts.py yoke_roll, yoke_pitch_v6), so every servo
interface -- pads, bosses, bolt circles, centre reliefs, the sunk roll idler
pad, the hip-flexion chamfer -- is theirs to the byte. check_interfaces()
holds that.

WHY ONE PRINT NEEDS SUPPORTS AND TWO DO NOT
  hip_yoke_v6 fuses two clevises whose arm plates are normal to X (roll) and
  to Y (pitch). Every arm must print with its length in the bed plane (the
  roll arms snapped when their layers ran across them), so whichever way the
  one part lies, one clevis prints as walls and the other as slabs over a
  void. Split, each clevis lies on edge in its own orientation, every arm is
  a wall, and each U-channel is one profile extruded off one flat bed face.

  ROLL U (print RY_ROLL_WALL, model -Y on the bed, as yoke_roll): the bed face
  is y = -10, the pads' and the idler boss's own radius. On yoke_roll's -12
  face the O20 idler boss -- a journal turning 0.3 mm inside the carrier's
  bay bore -- started 2 mm up as a 4.15 mm cantilever, and the pad rims hung
  in the air; tangent to the bed they print like a cylinder lying on it. So
  the arms lose 2 mm on their -y side (-10..12) and the flange runs -10..17.
  Nothing is added: every clearance can only open.

  PITCH U (print RY_XUP, model -X on the bed, as yoke_pitch): the bed face is
  x = -14, the idler riser's rear edge, where the r14 idler hub is already
  tangent. The flange's rear (-16..-14) is trimmed to it; the horn arm's rear
  edge comes back from -12 to -14 and meets its pad through a 45 deg face
  tangent to the pad (PITCH_ARM_EXT) -- the only material added anywhere,
  and the CAD gate checks it.

THE JOINT
  The roll flange's underside sits on the pitch flange's top at the old
  bolted plane, z -20: a 27 x 30 mm glued lap. Two split-prong snap clips
  (SNAP_*) hang from the roll flange through two holes in the pitch flange
  and click under it; they locate the pieces in x and y and hold them while
  the glue cures. The prongs' fingers are cut free of the flange 2 mm up
  (relief slots) so a 0.3 mm barb bends them ~1.2 %, well inside PETG's
  yield. Both fingers flex along X, in the roll U's print plane: the prong
  is part of its profile. One prong rises off the roll U's bed; the other
  sticks out of the flange 21 mm up, so its underside slopes 45 deg back to
  the flange and its hole tapers to match. Glue: CA or epoxy on lightly
  sanded faces.

WHAT IT COSTS against hip_yoke_v6 (measured in __main__)
  - the band between the flanges is 4 + 4 mm glued, not 8 mm fused;
  - the roll flange in front of the pitch flange (x 16..21.45), which turns
    the roll torque back to the lap, is 27 mm wide where it was 34;
  - the roll arms are 22 mm wide where they were 24 (in-plane section
    modulus -16 %); the pitch horn arm is 26 where it was 24 (+17 %).

    .venv/bin/python cad/v6/hip_yoke_glued.py   # stl/step + audits + mass + render
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

from build123d import Pos, chamfer, export_step, export_stl, fillet  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as v5  # noqa: E402
import yoke_pitch_v6  # noqa: E402
import hip_yoke_v6 as HY  # noqa: E402
import paths  # noqa: E402
D = V.D

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

PIECES = ("hip_yoke_glued_roll", "hip_yoke_glued_pitch")
# rows = print axes in model coords (check_printability's convention)
PRINT_ORIENT = {
    "hip_yoke_glued_roll": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),   # RY_ROLL_WALL
    "hip_yoke_glued_pitch": ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),  # RY_XUP
}

ZF_TOP, ZF_MID, ZF_BOT = HY.ZF_TOP, HY.ZF_MID, HY.ZF_BOT      # -16 / -20 / -24
PZ = HY.PZ                                                     # -50 pitch axis

# roll U: the bed face, and the flange's +y edge (yoke_roll's own)
ROLL_BED_Y = -D.PAD_D / 2                                      # -10.00
ROLL_FLANGE_Y = (ROLL_BED_Y, D.YOKE_FLANGE_Y / 2)              # -10 .. 17
# pitch U: the bed face, at the idler riser's rear edge
PITCH_BED_X = HY.RISER_X[0]                                    # -14.00
assert abs(PITCH_BED_X + HY.HUB_R) < 1e-9, "the idler hub must be tangent to the bed"

# the snap clips: two prongs on the roll flange's underside at x = 0, one at
# each of its y edges. Finger 1.0 thick, slot 1.2, barb 0.45 proud of the
# shank against a hole 0.15 clear of it: 0.30 of catch per finger. In the
# roll U's print the bed-edge prong rises off the bed. The top-edge prong
# sticks out sideways from the flange 21 mm up, so its underside slopes
# 45 deg back to the flange (snap_y_min): 6 mm wide at the tip, 11 at the
# flange, and its hole in the pitch flange tapers to match.
SNAP_Y = ((ROLL_FLANGE_Y[0], ROLL_FLANGE_Y[0] + 6.0),          # -10 .. -4
          (ROLL_FLANGE_Y[1] - 6.0, ROLL_FLANGE_Y[1]))          #  11 .. 17 at the tip
SNAP_SHANK = 1.6          # finger outer face, |x|
SNAP_SLOT = 0.6           # finger inner face, |x|
SNAP_BARB = 0.45          # barb proud of the shank
SNAP_HOLE_CLR = 0.15      # hole clearance on the shank, each side (x)
SNAP_HOLE_Y_CLR = 0.20    # and along y
SNAP_ROOT = ZF_MID + 2.0  # fingers cut free of the flange up to here (-18)
SNAP_RELIEF = 0.8         # relief slot outboard of each finger
SNAP_CATCH = ZF_BOT - 0.1  # barb's catch face, 0.1 under the pitch flange
SNAP_TIP = SNAP_CATCH - 0.9

R_ROOT, R_ROOT_SERVO, R_EDGE = HY.R_ROOT, HY.R_ROOT_SERVO, 1.0

# A round end tangent to the bed rises off it as a near-horizontal overhang
# for its first millimetres. Each pad and the pitch idler hub gets a 45 deg
# FOOT on its bed side instead: the bed face runs on to where a 45 deg face
# tangent to the round meets it. Each foot is a triangle closed by a chord
# of its round, so it adds nothing inside the round (the bolt circles stay
# >= 0.9 mm clear of the chords) and reaches r 10.8 (pads) / 15.2 (hub, under
# the thigh's r 16 swing floor -- yoke_pitch's note).
_T = D.PAD_D / 2 / 2 ** 0.5                                    # 7.071
ROLL_PAD_FOOT = [(ROLL_BED_Y, 0.0), (ROLL_BED_Y, 2 * _T + ROLL_BED_Y), (-_T, _T)]   # (y, z)
_TH = HY.HUB_R / 2 ** 0.5                                      # 9.899
PITCH_HUB_FEET = [[(PITCH_BED_X, 0.0), (PITCH_BED_X, s * (2 * _TH + PITCH_BED_X)), (-_TH, s * _TH)]
                  for s in (1, -1)]                            # (x, z), pitch frame
# the roll idler pad is sunk 1.35 into its arm; on the bed face its rim comes
# within 0.2 mm of the edge for |z| < 4. Open the pocket through to the bed
# there rather than print a sub-perimeter lip.
SINK_LIP_Y = -(D.PAD_D / 2 - 0.85)                             # -9.15
SINK_LIP_Z = ((D.PAD_D / 2) ** 2 - SINK_LIP_Y ** 2) ** 0.5     # 4.04


def _fill_bolt_columns(r, z0, z1):
    """The v5 flange bolt columns, filled back to solid (they bolted the two
    clevises; the glue and the clips replace them)."""
    b = D.YOKE_BOLT_SQ / 2
    fill = None
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        c = v5.cyl_z(r, z0, z1, sx, sy)
        fill = c if fill is None else fill + c
    return fill


def snap_y_min(z):
    """The top-edge prong's underside at height z: SNAP_Y[1][0] at the tip,
    one more mm of width per mm up toward the flange (45 deg)."""
    return SNAP_Y[1][0] + SNAP_TIP - z


def _below_slope(x0, x1, z_hi, z_lo, clr=0.0):
    """The region under the top-edge prong's 45 deg underside, less `clr`,
    between heights z_lo and z_hi."""
    return v5.wedge_x([(-40, z_hi), (snap_y_min(z_hi) - clr, z_hi),
                       (snap_y_min(z_lo) - clr, z_lo), (-40, z_lo)], x0, x1)


def _prong(y0, y1, sloped=False):
    """One split-prong clip, in the roll frame. Profile in X-Z, extruded over
    [y0, y1]: two fingers from the flange down through the pitch flange, each
    with a barb whose flat catch faces up and whose lead-in ramp faces down.
    `sloped`: the top-edge prong, its underside cut to snap_y_min."""
    s, o, b = SNAP_SLOT, SNAP_SHANK, SNAP_BARB
    if sloped:
        y0 = snap_y_min(ZF_MID)
    p = None
    for sx in (1, -1):
        prof = [(sx * s, ZF_MID + 0.01), (sx * o, ZF_MID + 0.01), (sx * o, SNAP_CATCH),
                (sx * (o + b), SNAP_CATCH), (sx * o, SNAP_TIP), (sx * s, SNAP_TIP)]
        f = v5.wedge_y(prof, y0, y1)
        p = f if p is None else p + f
    if sloped:
        p -= _below_slope(-5, 5, ZF_MID + 1, SNAP_TIP - 1)
    return p


def _prong_reliefs(y0, y1, sloped=False):
    """Slots in the roll flange that free the fingers up to SNAP_ROOT: the
    split between them and one outboard of each. Each runs the prong's y
    band (for the sloped prong, as wide as its slope reaches at SNAP_ROOT),
    so in the print it rises off the bed or a floor and opens at the top."""
    s, o, w = SNAP_SLOT, SNAP_SHANK, SNAP_RELIEF
    if sloped:
        y0 = snap_y_min(SNAP_ROOT)
    cut = v5.box(-s, s, y0, y1, ZF_MID - 0.01, SNAP_ROOT)
    for sx in (1, -1):
        cut += v5.box(min(sx * o, sx * (o + w)), max(sx * o, sx * (o + w)), y0, y1,
                      ZF_MID - 0.01, SNAP_ROOT)
    return cut


def _prong_holes():
    """The two holes the prongs pass through, in the PITCH flange (roll
    frame), SNAP_HOLE_Y_CLR clear of each prong along y: the bed-edge one
    square, the top-edge one tapered to the prong's slope. Horizontal in the
    pitch U's print: bridged roofs, 6.4 mm and 7.4..11.4 mm."""
    h = SNAP_SHANK + SNAP_HOLE_CLR
    c = SNAP_HOLE_Y_CLR
    (a0, a1), (b0, b1) = SNAP_Y
    cut = v5.box(-h, h, a0 - c, a1 + c, ZF_BOT - 1, ZF_MID + 1)
    top = v5.box(-h, h, snap_y_min(ZF_MID + 1) - c, b1 + c, ZF_BOT - 1, ZF_MID + 1)
    top -= _below_slope(-h - 1, h + 1, ZF_MID + 2, ZF_BOT - 2, clr=c)
    return cut + top


def _blend(p, groups, tag):
    """hip_yoke_v6's one-edge-at-a-time blend (target radius, then smaller,
    then a chamfer), reporting anything it could not round."""
    skipped = []
    for name, r, test in groups:
        edges = [e for e in p.edges() if test(e)]
        if not edges:
            print(f"  [{tag}] '{name}': no edges matched -- left sharp")
            continue
        got = 0
        for e in edges:
            c = e.center()
            done = False
            for op in (fillet, chamfer):
                for f in HY.LADDER:
                    try:
                        p = op([e], r * f)
                        done = True
                        break
                    except Exception:  # noqa: BLE001
                        continue
                if done:
                    break
            got += done
            if not done:
                skipped.append((name, (c.X, c.Y, c.Z)))
        print(f"  [{tag}] {name}: {got}/{len(edges)} at R{r:.2f}")
    for name, c in skipped:
        print(f"  [{tag}] NOT rounded: {name} at ({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})")
    return p


def roll_u(style=True):
    """The roll clevis: yoke_roll with its bolt columns filled, its -y side
    trimmed to the bed face, the roll-corner relief and the +y shoulder
    taper of hip_yoke_v6, and the two snap prongs."""
    p = v5.yoke_roll() + _fill_bolt_columns(D.M3_CLEAR / 2 + 0.05, ZF_MID, ZF_TOP)
    p -= v5.box(-40, 40, -40, ROLL_BED_Y, -40, 30)
    # the idler arm's foot stops at its sunk pad's floor: over the 1.35 mm sink
    # the pad is a pocket, and a foot there would be a feather round its rim
    for x0, x1 in ((HY.RXI1, HY.RX1), (HY.RX0 + D.ROLL_IDLER_PAD_SINK, HY.RXI0)):
        p += v5.wedge_x(ROLL_PAD_FOOT, x0, x1)
    p -= v5.box(HY.RX0 - 1, HY.RX0 + D.ROLL_IDLER_PAD_SINK, ROLL_BED_Y - 1, SINK_LIP_Y,
                -SINK_LIP_Z, SINK_LIP_Z)
    p -= HY._roll_corner_relief()
    pr = D.PAD_D / 2 + 0.3
    across, along = HY.SHOULDER
    p -= v5.wedge_x([(pr, 0.0), (HY.AHW, 0.0), (HY.AHW, -along)], HY.RXI1 - 0.01, HY.RX1 + 0.01)
    if style:
        on, dr, span = HY._on, HY._dir, HY._span
        p = _blend(p, [
            ("arm root, servo side", R_ROOT_SERVO,
             lambda e: dr(e, "y") and any(on(e, x=vx, z=ZF_TOP) for vx in (HY.RXI1, HY.RXI0))),
            ("arm root, +y side", R_ROOT,
             lambda e: dr(e, "x") and on(e, y=HY.AHW, z=ZF_TOP)),
            ("flange ends, underside", R_EDGE,
             lambda e: dr(e, "y") and any(on(e, x=vx, z=ZF_MID) for vx in (HY.RX1, HY.RX0))),
            ("flange +y edge, ends", R_EDGE,
             lambda e: dr(e, "z") and any(on(e, x=vx, y=ROLL_FLANGE_Y[1]) for vx in (HY.RX1, HY.RX0))),
        ], "roll_u")
    for (y0, y1), sloped in zip(SNAP_Y, (False, True)):
        p -= _prong_reliefs(y0, y1, sloped)
        p += _prong(y0, y1, sloped)
    return p


# the pitch horn arm's rear edge brought back to the bed face, meeting the
# pad (r 10) through a 45 deg face tangent to it, closed by a chord of the pad
# as the feet are -- in the pitch clevis's own frame (pitch axis at the
# origin, flange above)
PITCH_ARM_EXT = [(PITCH_BED_X, D.PITCH_ARM_REACH), (PITCH_BED_X, -2 * _T - PITCH_BED_X),
                 (-_T, -_T), (-D.PAD_D / 2, 0.0), (-HY.AHW, 0.0), (-HY.AHW, D.PITCH_ARM_REACH)]


def pitch_u(style=True):
    """The pitch clevis: yoke_pitch_v6 with its heat-set columns filled, its
    flange's rear trimmed to the bed face, the horn arm's rear edge brought
    back to it, the +x shoulder taper of hip_yoke_v6, and the two holes for
    the snap prongs. Built in the pitch frame, returned in the roll frame."""
    zf1, zf0 = D.PITCH_ARM_REACH, D.PITCH_ARM_REACH + D.YOKE_FLANGE_T
    q = yoke_pitch_v6.yoke_pitch_v6() + _fill_bolt_columns(D.HEATSET_D / 2 + 0.05, zf1, zf0)
    q -= yoke_pitch_v6.flex_chamfer_wedge()      # the fill grazes it (hip_yoke_v6.fused)
    q -= v5.box(-40, PITCH_BED_X, -40, 40, zf1, zf0 + 1)
    q += v5.wedge_y(PITCH_ARM_EXT, D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE)
    for foot in PITCH_HUB_FEET:
        q += v5.wedge_y(foot, HY.PY0, HY.PYI0)
    pr = D.PAD_D / 2 + 0.3
    across, along = HY.SHOULDER
    q -= v5.wedge_y([(pr, 0.0), (HY.AHW, 0.0), (HY.AHW, along)], HY.PYI1 - 0.01, HY.PY1 + 0.01)
    p = HY.pitch_offset() * q
    if style:
        on, dr = HY._on, HY._dir
        p = _blend(p, [
            ("horn arm root, servo side", R_ROOT,
             lambda e: dr(e, "x") and on(e, y=HY.PYI1, z=ZF_BOT)),
            ("riser root, servo side", R_ROOT,
             lambda e: dr(e, "x") and on(e, y=HY.PYI0, z=ZF_BOT)),
            ("horn arm root, front", R_ROOT,
             lambda e: dr(e, "y") and on(e, x=HY.AHW, z=ZF_BOT)),
            ("riser root, front", R_ROOT,
             lambda e: dr(e, "y") and on(e, x=HY.RISER_X[1], z=ZF_BOT)),
            ("flange top, outer edges", R_EDGE,
             lambda e: dr(e, "x") and any(on(e, y=vy, z=ZF_MID) for vy in (HY.PY1, HY.PY0))),
        ], "pitch_u")
    return p - _prong_holes()


def hip_yoke_glued(style=True):
    """The two pieces as glued: the solid the assembly, the CAD gate and the
    plant use (cf. hip_yoke_v6.hip_yoke_v6)."""
    return roll_u(style) + pitch_u(style)


def build(style=True):
    return {"hip_yoke_glued_roll": roll_u(style), "hip_yoke_glued_pitch": pitch_u(style)}


def _pad_zones():
    """hip_yoke_v6's interface zones drawn just inside the pads (r 10, not
    10.2): the bed trim is tangent to the roll pads and the pitch horn arm's
    45 deg face is tangent to its pad, so both touch r 10 and nothing inside."""
    r = D.PAD_D / 2 - 0.01
    z = v5.cyl_x(r, D.SV_HORN_FACE - 0.1, HY.RX1 + 0.1, 0, 0)
    z += v5.cyl_x(r, HY.RX0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    z += HY.pitch_offset() * v5.cyl_y(r, D.SV_HORN_FACE - 0.1, HY.PY1 + 0.1, 0, 0)
    z += HY.pitch_offset() * v5.cyl_y(r, HY.PY0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    return z


def check_interfaces(glued, verbose=True):
    """HARD GATE: every servo interface -- the four pads with their bosses,
    bolt circles and centre reliefs -- must match the raw fused clevises."""
    z = _pad_zones()
    a = (HY.fused() & z).volume
    b = (glued & z).volume
    ok = abs(a - b) < 0.05
    if verbose:
        print(f"  [hip_yoke_glued] servo interfaces: {a:.3f} mm3 fused vs "
              f"{b:.3f} mm3 glued -> {'IDENTICAL' if ok else '** CHANGED **'}")
    return ok


def snap_strain():
    """Peak bending strain in a finger deflected by its catch: a cantilever
    from SNAP_ROOT to the catch face, 1.5 t d / L^2."""
    t = SNAP_SHANK - SNAP_SLOT
    d = SNAP_BARB - SNAP_HOLE_CLR
    L = SNAP_ROOT - SNAP_CATCH
    return 1.5 * t * d / L ** 2


# What the audit is told, and why, for each piece -- the only findings it may
# raise that are not failures. Anything else fails.
#  - CONTACT: both pieces stand on edge on 310-340 mm2 of first layer, under
#    the audit's 25 %-of-bbox floor, a tip-over risk for tall parts. These
#    are 27-30 mm tall on U-shaped footprints, and print WITHOUT a brim (Tom,
#    2026-10-14). If one lifts, add hip_yoke_v6's 5 mm outer brim in
#    cad/v6/print_settings.json.
#  - the roll idler boss: the O20 journal that turns 0.3 mm inside the yaw
#    carrier's bay bore lies tangent on the bed (ROLL_BED_Y), so its first
#    millimetre widens off a line of bed contact, like any cylinder printed on
#    its side. It cannot have a foot -- the bore is round all the way round --
#    and the audit, which looks for something UNDER each down-facing facet,
#    calls the two flanks of that line LEDGEs 0.4 mm up, 4.2 mm long (the
#    boss's length, D.SV_IDLER_FACE - D.ROLL_ARM_INNER).
BOSS_LEN = D.SV_IDLER_FACE - D.ROLL_ARM_INNER                 # 4.15


def _expected(name, kind, desc):
    import re
    if kind == "CONTACT":
        return "small first layer for an on-edge part, no brim"
    m = re.search(r"([\d.]+) x\s+([\d.]+) mm\s+at z\s+([\d.]+)", desc)
    if (name == "hip_yoke_glued_roll" and kind == "LEDGE" and m
            and abs(float(m.group(1)) - BOSS_LEN) < 0.1 and float(m.group(3)) < 1.0):
        return "idler boss journal, tangent on the bed"
    return None


def audit(name, stl_path):
    """check_printability in the piece's PRINT_ORIENT with NO support waiver:
    these pieces exist to print without supports. Returns (failures,
    accepted): `accepted` are the findings _expected() names, with why."""
    import numpy as np
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT[name] = (np.array(PRINT_ORIENT[name]), "glued hip yoke piece, on edge, no supports")
    CP.PRINT_STL[name] = os.path.basename(stl_path)
    CP.SUPPORTED.pop(name, None)
    fails, ok = [], []
    for kind, a, desc in CP.audit(name):
        why = _expected(name, kind, desc)
        (ok if why else fails).append((kind, a, desc, why))
    return fails, ok


def in_print_orient(solid, name):
    """The piece as it lies on the bed: PRINT_ORIENT applied, bed at z 0."""
    from build123d import Matrix
    r = PRINT_ORIENT[name]
    s = solid.transform_geometry(Matrix([[*r[0], 0], [*r[1], 0], [*r[2], 0], [0, 0, 0, 1]]))
    bb = s.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s


def render_shots(shots, png_path, px=560):
    """A strip of MuJoCo offscreen renders (hip_yoke_v6.render's setup), one
    per shot = (items, azimuth, elevation); items is a solid or a list of
    (solid, rgba) drawn together. Camera on the items' bounding box."""
    os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
    import tempfile
    import imageio.v2 as imageio
    import mujoco
    import numpy as np
    frames = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, (items, az, el) in enumerate(shots):
            if not isinstance(items, list):
                items = [(items, (0.45, 0.62, 0.90, 1))]
            assets, geoms, lo, hi = [], [], np.full(3, np.inf), np.full(3, -np.inf)
            for j, (s, rgba) in enumerate(items):
                stl = os.path.join(tmp, f"{i}_{j}.stl")
                export_stl(s, stl)
                assets.append(f'<mesh name="m{j}" file="{stl}" scale="0.001 0.001 0.001"/>')
                geoms.append(f'<geom type="mesh" mesh="m{j}" rgba="{" ".join(map(str, rgba))}"/>')
                bb = s.bounding_box()
                lo = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z])
                hi = np.maximum(hi, [bb.max.X, bb.max.Y, bb.max.Z])
            model = mujoco.MjModel.from_xml_string(f"""
            <mujoco>
              <asset>{''.join(assets)}</asset>
              <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7"/>
                <global offwidth="{px}" offheight="{px}"/></visual>
              <worldbody>{''.join(geoms)}</worldbody>
            </mujoco>""")
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            ren = mujoco.Renderer(model, px, px)
            cam = mujoco.MjvCamera()
            cam.lookat = (lo + hi) / 2 * 0.001
            cam.distance = 1.2 * np.linalg.norm(hi - lo) * 0.001
            cam.azimuth, cam.elevation = az, el
            ren.update_scene(data, cam)
            frames.append(ren.render())
    imageio.imwrite(png_path, np.concatenate(frames, axis=1))
    return png_path


def render(pieces, png_path, px=560):
    """One strip: each piece as it lies on the bed, the pair exploded along
    the snap's axis, and glued."""
    roll, pitch = pieces["hip_yoke_glued_roll"], pieces["hip_yoke_glued_pitch"]
    return render_shots([(in_print_orient(roll, "hip_yoke_glued_roll"), 135, -35),
                         (in_print_orient(pitch, "hip_yoke_glued_pitch"), 135, -35),
                         (Pos(0, 0, 14) * roll + pitch, 140, -20),
                         (roll + pitch, 140, -20)], png_path, px)


def SCREWS():
    """The disc screws are hip_yoke_v6's: the pads and their stacks are the
    same. The joint has no fasteners."""
    return HY.SCREWS()


disc_seat_zones = HY.disc_seat_zones


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    pieces = build()
    glued = pieces["hip_yoke_glued_roll"] + pieces["hip_yoke_glued_pitch"]
    one = HY.hip_yoke_v6(verbose=False)
    findings = {}
    for name, s in pieces.items():
        n = len(s.solids())
        bb = s.bounding_box()
        path = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, path)
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        print(f"{name:22s} {n} solid(s)  {HY.mass_g(s):5.2f} g  bbox "
              f"x {bb.min.X:.2f}..{bb.max.X:.2f} y {bb.min.Y:.2f}..{bb.max.Y:.2f} "
              f"z {bb.min.Z:.2f}..{bb.max.Z:.2f}")
        findings[name] = audit(name, path)
    print(f"glued pair {HY.mass_g(glued):.2f} g vs hip_yoke_v6 {HY.mass_g(one):.2f} g  "
          f"({time.time() - t0:.0f}s)")
    print(f"snap finger strain {snap_strain() * 100:.2f} %")
    check_interfaces(glued)
    for name, (fails, ok) in findings.items():
        notes = "; ".join(sorted({w for *_, w in ok}))
        print(f"printability {name}: "
              f"{'PASS, no supports' if not fails else f'** {len(fails)} FAILURE(S) **'}"
              f"{f' (accepted: {notes})' if notes else ''}")
        for kind, a, desc, _ in fails:
            print(f"    {kind:8s} {desc}")
    print("wrote", render(pieces, os.path.join(OUT_REN, "hip_yoke_glued.png")))
