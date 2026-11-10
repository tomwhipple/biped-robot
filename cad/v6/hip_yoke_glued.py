"""Hip yoke (qty 2 of each piece): the hip-roll and hip-pitch clevises as two
U-channels that print WITHOUT SUPPORTS, snap together and are glued (Tom,
2026-10-13: "bridging to the baseplate is OK, but I'd like to avoid large
internal bridges. (holes are also OK)").

Local frame (the joint frame): roll axis X through the origin, +X forward
(the roll servo's horn side), pitch axis along Y through (0, 0, -50). Both
pieces are cut from the base clevises (cad/common/hip_parts.py yoke_roll,
yoke_pitch), so
every servo interface -- pads, bosses, bolt circles, centre reliefs, the sunk
roll idler pad -- is theirs to the byte. check_interfaces() holds that.

WHY TWO PIECES
  The two clevises' arm plates are normal to X (roll) and to Y (pitch). Every
  arm must print with its length in the bed plane (the roll arms snapped when
  their layers ran across them), so one print would have one clevis as walls
  and the other as slabs over a void, on supports. Split, each clevis lies on
  edge in its own orientation, every arm is a wall, and each U-channel is one
  profile extruded off one flat bed face.

  ROLL U (print RY_ROLL_WALL, model -Y on the bed): the bed face is y = -10,
  the pads' and the idler boss's own radius, so the O20 idler boss (a journal
  turning 0.3 mm inside the carrier's bay bore) and the pad rims print like a
  cylinder lying on the bed.

  PITCH U (print RY_XUP, model -X on the bed): the bed face is x = -14, where
  the r14 idler hub is tangent. The horn arm's rear edge comes back to it and
  meets its pad through a 45 deg face tangent to the pad (PITCH_ARM_EXT).

THE JOINT
  The roll flange's underside sits on the pitch flange's top at z -20 (the
  glue plane, ZF_MID): a glued lap about 1060 mm2. Two split-prong snap clips
  (SNAP_*) hang from the roll flange through two holes in the pitch flange
  and click under it; they locate the pieces in x and y and hold them while
  the glue cures. The prongs' fingers are cut free of the flange 2 mm up
  (relief slots) so a 0.3 mm barb bends them ~1.2 %, well inside PETG's
  yield. Glue: CA or epoxy on lightly sanded faces.

THE LOAD PATHS (2026-11-02, docs/design-v6/2026-11-02-hip-yoke-roll-failure.md)
  The yoke carries the roll servo's torque (an STS3250, 4.9 N m stall) from
  the roll HORN arm to both pitch pads, the pitch servo's from the pitch horn
  arm to both roll pads, and the yaw servo's as a couple across both forks.
  An FDM part is weakest across its layers, and on these on-edge prints a
  moment carried by a thin plate puts its shear on the layer planes. The
  first glued yoke broke that way on the robot: the lap ended at x 16, the
  roll flange went on alone, 27 x 4 mm, to the horn arm at 21.45, and the horn
  arm came off with a strip of flange (the glue held). FEM at stall put
  121 MPa across the layers at the horn arm's root. Every fork was then
  checked in its own print frame under all three servo torques and two leg
  forces, and given the same treatment where it needed it:
    - the pitch flange runs on to LAP_X1 (21.6), under the horn arm's inner
      face, on a toe whose underside follows the thigh's and pitch servo's
      swept envelope at -120 deg of flexion with >= 1 mm to spare (TOE_*);
      the old 45 deg flexion chamfer was for a -125 deg ROM the joint never
      reached, and the toe replaces it;
    - the roll flange is as wide as the pitch flange (to PY1, 23.45), so the
      lap and the torque section are 33.45 mm wide where they were 27;
    - a FIN rises off the roll flange's +y edge from arm to arm, and a WEB
      carries each roll arm out to the same edge, its top tangent to the pad:
      the open 4 mm strip becomes a deep section, inside the carrier's and the
      roll servo's swept envelope at +-55 deg of roll with >= 1 mm to spare.
      The -y side has no such room (the carrier and the roll servo sweep it);
    - the pitch IDLER arm is a plate the full width of its r14 hub (x +-14),
      not v5's 18 mm riser off to the rear, which bent in its plane under the
      yaw servo's torque.
  Both pieces print at 100 % infill (cad/v6/print_settings.json): the
  fracture ran through the sparse core of a 3-wall, 35 % print.

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
from common import shapes  # noqa: E402
from common import hip_parts  # noqa: E402
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

# ------------------------------------------------------------------ landmarks
# all from dimensions.py: the v5 clevises' own faces
ZF_TOP = -D.ROLL_AXIS_TO_FLANGE                 # -16.00 roll flange top
ZF_MID = ZF_TOP - D.YOKE_FLANGE_T               # -20.00 the glue plane
ZF_BOT = ZF_MID - D.YOKE_FLANGE_T               # -24.00 pitch flange bottom
PZ = -D.ROLL_TO_PITCH                           # -50.00 pitch axis
RX1 = D.SV_HORN_FACE + D.HORN_BOSS_H + D.PLATE  # +24.45 roll horn arm, outer
RX0 = D.ROLL_ARM_INNER - D.PLATE                # -23.95 roll idler arm, outer
RXI1 = D.SV_HORN_FACE + D.HORN_BOSS_H           # +21.45 roll horn arm, inner
RXI0 = D.ROLL_ARM_INNER                         # -20.95 roll idler arm, inner
PX = D.YOKE_FLANGE_X / 2                        # 16.00 v5 pitch flange half-width
PY1 = D.SV_HORN_FACE + D.PLATE                  # +23.45 pitch horn arm, outer
PY0 = D.IDLER_ARM_INNER - D.PLATE               # -20.40 pitch idler arm, outer
PYI1 = D.SV_HORN_FACE                           # +20.45 pitch horn arm, inner
PYI0 = D.IDLER_ARM_INNER                        # -17.40 pitch idler arm, inner
AHW = 12.0                                      # arm plate half-width (v5)
RISER_X = (-14.0, 4.0)                          # v5's pitch idler riser (its rear edge is the bed face)
HUB_R = 14.0                                    # pitch idler hub, v5 literal

# roll U: the bed face, and the flange's +y edge (the pitch flange's, PY1, from
# the lap's rear forward; yoke_roll's own 17 on the rear overhang)
ROLL_BED_Y = -D.PAD_D / 2                                      # -10.00
ROLL_FLANGE_Y = (ROLL_BED_Y, PY1)                              # -10 .. 23.45
# pitch U: the bed face, at the idler riser's rear edge
PITCH_BED_X = RISER_X[0]                                       # -14.00
assert abs(PITCH_BED_X + HUB_R) < 1e-9, "the idler hub must be tangent to the bed"

# ------------------------------------------------- the roll torque path
# The pitch flange's TOE: from the flange bottom at TOE_X0 its underside rises
# at TOE_SLOPE to the front face at LAP_X1, so the lap reaches under the horn
# arm. Drawn >= 1.0 mm (vertical) above the thigh's and the pitch servo's
# swept envelope over the hip-pitch ROM (it peaks under the horn arm, z -21.8
# at x 24; z -23.25 at x 20). The CAD gate holds the 0.5 mm buffer.
LAP_X1 = 21.6
TOE_X0, TOE_SLOPE = 17.0, 0.583
# The FIN on the roll flange's +y edge: its underside leaves the flange top at
# FIN_Y0 and rises at FIN_SLOPE (dz/dy, 36 deg off the print's vertical) to
# FIN_TOP_Z, >= 1.2 mm (vertical) under the carrier's and the roll servo's
# sweep at +-55 deg of roll; it runs FIN_X, arm to arm.
FIN_Y0, FIN_SLOPE, FIN_TOP_Z = 8.0, 0.72, -8.0
FIN_X = (RXI0, RXI1)
FIN_Y_CAP = FIN_Y0 + (FIN_TOP_Z - ZF_TOP) / FIN_SLOPE          # 19.11
# the profile's corner there is bevelled (inside the profile, so it can only
# open a clearance): the fin's junction with the arms then rounds at a real radius
FIN_BEVEL = ((FIN_Y_CAP - 2.0, FIN_TOP_Z - 2.0 * FIN_SLOPE), (FIN_Y_CAP + 1.5, FIN_TOP_Z))
# each roll arm's +y WEB, from the arm's edge out to PY1: its top runs on the
# tangent from the pad's rim down to the fin's top at PY1, so the arm, the web
# and the fin are one outline with no notch (a web whose top met the arm's
# edge in a corner put a 59 MPa peak there at stall). It stays outside the
# pad's radius, so no servo interface is touched. Nothing sweeps outboard of
# an arm's inner face (the carrier's bay walls are inside them).
WEB_PAD_R = D.PAD_D / 2
# the webs end WEB_RISE above the fin's top at PY1, so the fin meets each arm's
# inner face under a face of real height (where they met at one point the
# junction could not be rounded)
WEB_RISE = 3.0
# the idler pad is sunk 1.35 into its arm, so there the web keeps a 1.0 mm rim
# round the pocket: its top is tangent to a circle 1.0 outside the pad, and
# the rim ends in a 1.0 mm step at the tangent point
WEB_IDLER_RIM = 1.0
# THE PITCH IDLER ARM. v5 made it a r14 hub on an 18 mm riser (x -14..4) to
# keep it out of the v5 thigh's idler grip plate; the v6 thigh grips inboard
# of the arm's y band and sweeps nothing there forward to x 15 (the swept
# map, 2026-11-02), and the off-centre riser bent in its plane under the yaw
# servo's torque, 32 MPa across its layers at stall. It is now a plate the
# hub's own width, x -14 (the bed face) to +14, centred on the pitch axis.
IDLER_X1 = HUB_R
R_IDLER_FRONT = 4.0       # its front root: under the yaw servo's torque the arm bends
                          # in its plane and this corner carries the peak

# the snap clips: two prongs on the roll flange's underside at x = 0, one at
# each of yoke_roll's flange edges (-10 and 17; at the new +y edge the prong
# would land on the pitch horn arm). Finger 1.0 thick, slot 1.2, barb 0.45
# proud of the shank against a hole 0.15 clear of it: 0.30 of catch per
# finger. In the roll U's print the bed-edge prong rises off the bed; the
# +y prong sticks out sideways from the flange 21 mm up, so its underside
# slopes 45 deg back to the flange (snap_y_min): 6 mm wide at the tip, 11 at
# the flange, and its hole in the pitch flange tapers to match.
SNAP_Y = ((ROLL_BED_Y, ROLL_BED_Y + 6.0),                      # -10 .. -4
          (D.YOKE_FLANGE_Y / 2 - 6.0, D.YOKE_FLANGE_Y / 2))    #  11 .. 17 at the tip
SNAP_SHANK = 1.6          # finger outer face, |x|
SNAP_SLOT = 0.6           # finger inner face, |x|
SNAP_BARB = 0.45          # barb proud of the shank
SNAP_HOLE_CLR = 0.15      # hole clearance on the shank, each side (x)
SNAP_HOLE_Y_CLR = 0.20    # and along y
SNAP_ROOT = ZF_MID + 2.0  # fingers cut free of the flange up to here (-18)
SNAP_RELIEF = 0.8         # relief slot outboard of each finger
SNAP_CATCH = ZF_BOT - 0.1  # barb's catch face, 0.1 under the pitch flange
SNAP_TIP = SNAP_CATCH - 0.9

R_ROOT = 2.5              # arm roots with room around them
R_ROOT_SERVO = 0.6        # the idler arm's root facing the carrier's bay wall
R_ROOT_HORN = 2.0         # the horn arm's servo-side root: the roll servo's case
                          # ends at x 18.48 and the carrier's bay wall stands
                          # above z -14.5, so this fillet (x >= 19.45, clear of the
                          # roll-corner relief's end at 18.98) sweeps clear
R_EDGE = 1.0
SHOULDER = (1.7, 3.0)     # the pitch horn arm's shoulder taper (across, along
                          # the arm), stopping 0.3 outside the r 10 pad
LADDER = (1.0, 0.7, 0.5, 0.35, 0.2)      # blend radius fallbacks, as a fraction of r

# The roll servo's output-end corner sits r 15.97 from the roll axis and at
# +-55 deg of roll points straight down at the flange top, 16.0 below the axis.
# A 0.5 mm relief exactly where it passes: across the case's own thickness
# (its idler boss to its horn rib, +-0.5) and, along y, where it comes within
# the buffer (|y| < 4; at y 6 the corner is already 1.2 above the flange). Its
# +y end ramps out at 45 deg, so the print has no 0.5 mm ledge there, and it
# stops short of the fin.
ROLL_CORNER_RELIEF = 0.5
ROLL_RELIEF_X = (D.SV_IDLER_BOSS_Y - 0.5,                          # -17.15
                 D.SV_TOPFACE + D.SV_HORN_RIB_H + 0.5)             # +18.98
ROLL_RELIEF_Y = (ROLL_BED_Y - 1.0, 6.0)
assert ROLL_RELIEF_X[0] > RXI0 + 2.0 and ROLL_RELIEF_X[1] < RXI1 - 2.0, \
    "the roll-corner relief must stay clear of the roll arm plates' roots"

# A round end tangent to the bed rises off it as a near-horizontal overhang
# for its first millimetres. Each pad and the pitch idler hub gets a 45 deg
# FOOT on its bed side instead: the bed face runs on to where a 45 deg face
# tangent to the round meets it. Each foot is a triangle closed by a chord
# of its round, so it adds nothing inside the round (the bolt circles stay
# >= 0.9 mm clear of the chords) and reaches r 10.8 (pads) / 15.2 (hub, under
# the thigh's r 16 swing floor -- yoke_pitch's note).
_T = D.PAD_D / 2 / 2 ** 0.5                                    # 7.071
ROLL_PAD_FOOT = [(ROLL_BED_Y, 0.0), (ROLL_BED_Y, 2 * _T + ROLL_BED_Y), (-_T, _T)]   # (y, z)
_TH = HUB_R / 2 ** 0.5                                         # 9.899
PITCH_HUB_FEET = [[(PITCH_BED_X, 0.0), (PITCH_BED_X, s * (2 * _TH + PITCH_BED_X)), (-_TH, s * _TH)]
                  for s in (1, -1)]                            # (x, z), pitch frame
# the roll idler pad is sunk 1.35 into its arm; on the bed face its rim comes
# within 0.2 mm of the edge for |z| < 4. Open the pocket through to the bed
# there rather than print a sub-perimeter lip.
SINK_LIP_Y = -(D.PAD_D / 2 - 0.85)                             # -9.15
SINK_LIP_Z = ((D.PAD_D / 2) ** 2 - SINK_LIP_Y ** 2) ** 0.5     # 4.04


def pitch_offset():
    """Location of the pitch clevis's frame in this part's (roll) frame."""
    return Pos(0, 0, PZ)


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


def _fill_bolt_columns(r, z0, z1):
    """The v5 flange bolt columns, filled back to solid (they bolted the two
    clevises; the glue and the clips replace them)."""
    b = D.YOKE_BOLT_SQ / 2
    fill = None
    for sx, sy in ((b, b), (b, -b), (-b, b), (-b, -b)):
        c = shapes.cyl_z(r, z0, z1, sx, sy)
        fill = c if fill is None else fill + c
    return fill


def snap_y_min(z):
    """The +y prong's underside at height z: SNAP_Y[1][0] at the tip, one
    more mm of width per mm up toward the flange (45 deg)."""
    return SNAP_Y[1][0] + SNAP_TIP - z


def _below_slope(x0, x1, z_hi, z_lo, clr=0.0):
    """The region under the +y prong's 45 deg underside, less `clr`, between
    heights z_lo and z_hi."""
    return shapes.wedge_x([(-40, z_hi), (snap_y_min(z_hi) - clr, z_hi),
                       (snap_y_min(z_lo) - clr, z_lo), (-40, z_lo)], x0, x1)


def _prong(y0, y1, sloped=False):
    """One split-prong clip, in the roll frame. Profile in X-Z, extruded over
    [y0, y1]: two fingers from the flange down through the pitch flange, each
    with a barb whose flat catch faces up and whose lead-in ramp faces down.
    `sloped`: the +y prong, its underside cut to snap_y_min."""
    s, o, b = SNAP_SLOT, SNAP_SHANK, SNAP_BARB
    if sloped:
        y0 = snap_y_min(ZF_MID)
    p = None
    for sx in (1, -1):
        prof = [(sx * s, ZF_MID + 0.01), (sx * o, ZF_MID + 0.01), (sx * o, SNAP_CATCH),
                (sx * (o + b), SNAP_CATCH), (sx * o, SNAP_TIP), (sx * s, SNAP_TIP)]
        f = shapes.wedge_y(prof, y0, y1)
        p = f if p is None else p + f
    if sloped:
        p -= _below_slope(-5, 5, ZF_MID + 1, SNAP_TIP - 1)
    return p


def _prong_reliefs(y0, y1, sloped=False):
    """Slots in the roll flange that free the fingers up to SNAP_ROOT: the
    split between them and one outboard of each. Each runs the prong's y
    band (for the sloped prong, as wide as its slope reaches at SNAP_ROOT,
    and on through the widened flange to its +y face), so in the print it
    rises off the bed or a floor and opens at the top."""
    s, o, w = SNAP_SLOT, SNAP_SHANK, SNAP_RELIEF
    if sloped:
        y0, y1 = snap_y_min(SNAP_ROOT), ROLL_FLANGE_Y[1] + 0.01
    cut = shapes.box(-s, s, y0, y1, ZF_MID - 0.01, SNAP_ROOT)
    for sx in (1, -1):
        cut += shapes.box(min(sx * o, sx * (o + w)), max(sx * o, sx * (o + w)), y0, y1,
                      ZF_MID - 0.01, SNAP_ROOT)
    return cut


def _prong_holes():
    """The two holes the prongs pass through, in the PITCH flange (roll
    frame), SNAP_HOLE_Y_CLR clear of each prong along y: the bed-edge one
    square, the +y one tapered to the prong's slope. Horizontal in the pitch
    U's print: bridged roofs, 6.4 mm and 7.4..11.4 mm."""
    h = SNAP_SHANK + SNAP_HOLE_CLR
    c = SNAP_HOLE_Y_CLR
    (a0, a1), (b0, b1) = SNAP_Y
    cut = shapes.box(-h, h, a0 - c, a1 + c, ZF_BOT - 1, ZF_MID + 1)
    top = shapes.box(-h, h, snap_y_min(ZF_MID + 1) - c, b1 + c, ZF_BOT - 1, ZF_MID + 1)
    top -= _below_slope(-h - 1, h + 1, ZF_MID + 2, ZF_BOT - 2, clr=c)
    return cut + top


def _roll_corner_relief():
    y0, y1, d = ROLL_RELIEF_Y[0], ROLL_RELIEF_Y[1], ROLL_CORNER_RELIEF
    return shapes.wedge_x([(y0, ZF_TOP - d), (y1, ZF_TOP - d), (y1 + d, ZF_TOP),
                       (y1 + d, ZF_TOP + 0.01), (y0, ZF_TOP + 0.01)], *ROLL_RELIEF_X)


def _blend(p, groups, tag):
    """One edge at a time (OCC can take the process down on a group call):
    target radius, then smaller, then a chamfer, reporting anything it could
    not round."""
    skipped = []
    for name, r, test, *chain in groups:
        edges = [e for e in p.edges() if test(e)]
        if not edges:
            print(f"  [{tag}] '{name}': no edges matched -- left sharp")
            continue
        if chain:
            # a chain of edges that meet at corners, rounded in one call
            for f in LADDER:
                try:
                    p = fillet(edges, r * f)
                    print(f"  [{tag}] {name}: chain of {len(edges)} at R{r * f:.2f}")
                    break
                except Exception:  # noqa: BLE001
                    continue
            else:
                skipped += [(name, (e.center().X, e.center().Y, e.center().Z)) for e in edges]
                print(f"  [{tag}] {name}: chain of {len(edges)} NOT rounded")
            continue
        got = 0
        for e in edges:
            c = e.center()
            done = False
            for op in (fillet, chamfer):
                for f in LADDER:
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


def roll_fin():
    """The +y fin (x FIN_X) and both arms' webs, each reaching 0.5 into the
    flange below so the union has no coplanar seam."""
    zb = ZF_TOP - 0.5
    fin = shapes.wedge_x([(FIN_Y0, zb), (FIN_Y0, ZF_TOP), *FIN_BEVEL,
                      (PY1, FIN_TOP_Z), (PY1, zb)], FIN_X[0] - 0.01, FIN_X[1] + 0.01)
    def prof(r):
        ty, tz = web_tangent(r)
        return [(0.0, zb), (PY1, zb), (PY1, FIN_TOP_Z + WEB_RISE), (ty, tz), (0.0, 0.0)]
    webs = (shapes.wedge_x(prof(WEB_PAD_R), RXI1, RX1)
            + shapes.wedge_x(prof(WEB_PAD_R + WEB_IDLER_RIM), RX0, RXI0))
    # inside the pads' own radius everything stays the pads'
    return fin + (webs - shapes.cyl_x(D.PAD_D / 2, RX0 - 1, RX1 + 1, 0, 0))


def web_tangent(r=WEB_PAD_R):
    """(y, z) where the web's top leaves the circle of radius r round the
    pad: the upper tangent point from the web's end, (PY1, FIN_TOP_Z + WEB_RISE)."""
    import math
    px, pz = PY1, FIN_TOP_Z + WEB_RISE
    d = math.hypot(px, pz)
    a = math.atan2(pz, px) + math.acos(r / d)
    return r * math.cos(a), r * math.sin(a)


def roll_u(style=True):
    """The roll clevis: yoke_roll with its bolt columns filled, its -y side
    trimmed to the bed face, its flange widened to PY1 over the lap, the fin
    and the horn arm's web, the roll-corner relief, and the two snap
    prongs."""
    p = hip_parts.yoke_roll() + _fill_bolt_columns(D.M3_CLEAR / 2 + 0.05, ZF_MID, ZF_TOP)
    p -= shapes.box(-40, 40, -40, ROLL_BED_Y, -40, 30)
    p += shapes.box(RX0, RX1, D.YOKE_FLANGE_Y / 2 - 0.5, PY1, ZF_MID, ZF_TOP)
    # the idler arm's foot stops at its sunk pad's floor: over the 1.35 mm sink
    # the pad is a pocket, and a foot there would be a feather round its rim
    for x0, x1 in ((RXI1, RX1), (RX0 + D.ROLL_IDLER_PAD_SINK, RXI0)):
        p += shapes.wedge_x(ROLL_PAD_FOOT, x0, x1)
    p -= shapes.box(RX0 - 1, RX0 + D.ROLL_IDLER_PAD_SINK, ROLL_BED_Y - 1, SINK_LIP_Y,
                -SINK_LIP_Z, SINK_LIP_Z)
    p += roll_fin()
    p -= _roll_corner_relief()
    if style:
        p = _blend(p, [
            ("horn arm root, servo side", R_ROOT_HORN,
             lambda e: _dir(e, "y") and _on(e, x=RXI1, z=ZF_TOP)),
            ("idler arm root, servo side", R_ROOT_SERVO,
             lambda e: _dir(e, "y") and _on(e, x=RXI0, z=ZF_TOP)),
            # the fin's profile where it meets each arm's inner face: its sloped
            # underside and its flat top, as one chain
            ("fin into the arms", R_ROOT_HORN,
             lambda e: any(_on(e, x=vx) for vx in (RXI1, RXI0)) and not _dir(e, "z")
             and ((not _dir(e, "y") and _span(e, "y", FIN_Y0, FIN_BEVEL[1][0]) and _span(e, "z", ZF_TOP, FIN_TOP_Z))
                  or (_dir(e, "y") and _on(e, z=FIN_TOP_Z) and _span(e, "y", FIN_BEVEL[1][0], PY1))), "chain"),
            ("flange ends, underside", R_EDGE,
             lambda e: _dir(e, "y") and any(_on(e, x=vx, z=ZF_MID) for vx in (RX1, RX0))),
            ("flange +y edge, ends", R_EDGE,
             lambda e: _dir(e, "z") and any(_on(e, x=vx, y=PY1) for vx in (RX0, RX1))),
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
                 (-_T, -_T), (-D.PAD_D / 2, 0.0), (-AHW, 0.0), (-AHW, D.PITCH_ARM_REACH)]


def pitch_toe():
    """The pitch flange carried forward from PX to LAP_X1 (pitch frame): its
    top is the glue plane, its underside rises from TOE_X0 at TOE_SLOPE."""
    zf1, zf0 = D.PITCH_ARM_REACH, D.PITCH_ARM_REACH + D.YOKE_FLANGE_T
    z_front = zf1 + TOE_SLOPE * (LAP_X1 - TOE_X0)
    return shapes.wedge_y([(PX - 1.0, zf1), (TOE_X0, zf1), (LAP_X1, z_front), (LAP_X1, zf0), (PX - 1.0, zf0)],
                      PY0, PY1)


def pitch_u(style=True):
    """The pitch clevis: yoke_pitch with its heat-set columns filled, its
    flange's rear trimmed to the bed face and its front carried on to the toe,
    the horn arm's rear edge brought back to the bed face, the horn arm's
    shoulder taper, and the two holes for the snap prongs. Built in the pitch
    frame, returned in the roll frame."""
    zf1, zf0 = D.PITCH_ARM_REACH, D.PITCH_ARM_REACH + D.YOKE_FLANGE_T
    q = hip_parts.yoke_pitch() + _fill_bolt_columns(D.HEATSET_D / 2 + 0.05, zf1, zf0) + pitch_toe()
    # the idler arm as a plate on its full hub, outside the pad's radius
    idler = shapes.box(PITCH_BED_X, IDLER_X1, PY0, PYI0, 0.0, zf1 + 0.5) + shapes.cyl_y(HUB_R, PY0, PYI0, 0, 0)
    q += idler - shapes.cyl_y(D.PAD_D / 2, PY0 - 1, PYI0 + 1, 0, 0)
    q -= shapes.box(-40, PITCH_BED_X, -40, 40, zf1, zf0 + 1)
    q += shapes.wedge_y(PITCH_ARM_EXT, D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE)
    for foot in PITCH_HUB_FEET:
        q += shapes.wedge_y(foot, PY0, PYI0)
    pr = D.PAD_D / 2 + 0.3
    across, along = SHOULDER
    q -= shapes.wedge_y([(pr, 0.0), (AHW, 0.0), (AHW, along)], PYI1 - 0.01, PY1 + 0.01)
    p = pitch_offset() * q
    if style:
        p = _blend(p, [
            ("horn arm root, servo side", R_ROOT,
             lambda e: _dir(e, "x") and _on(e, y=PYI1, z=ZF_BOT)),
            ("idler arm root, servo side", R_ROOT,
             lambda e: _dir(e, "x") and _on(e, y=PYI0, z=ZF_BOT)),
            ("horn arm root, front", R_ROOT,
             lambda e: _dir(e, "y") and _on(e, x=AHW, z=ZF_BOT)),
            ("idler arm root, front", R_IDLER_FRONT,
             lambda e: _dir(e, "y") and _on(e, x=IDLER_X1, z=ZF_BOT)),
            # not the +y one: the roll flange is flush with it there
            ("flange top, idler-side edge", R_EDGE,
             lambda e: _dir(e, "x") and _on(e, y=PY0, z=ZF_MID)),
        ], "pitch_u")
    return p - _prong_holes()


def hip_yoke_glued(style=True):
    """The two pieces as glued: the solid the assembly, the CAD gate and the
    plant use."""
    return roll_u(style) + pitch_u(style)


def build(style=True):
    return {"hip_yoke_glued_roll": roll_u(style), "hip_yoke_glued_pitch": pitch_u(style)}


def raw_clevises():
    """The v5 clevises as they were bolted: the reference the servo
    interfaces are held to."""
    return hip_parts.yoke_roll() + pitch_offset() * hip_parts.yoke_pitch()


def _pad_zones():
    """The four pads with their bosses, bolt circles and centre reliefs, drawn
    just inside the pads (r 10, not 10.2): the bed trim is tangent to the roll
    pads and the pitch horn arm's 45 deg face is tangent to its pad, so both
    touch r 10 and nothing inside."""
    r = D.PAD_D / 2 - 0.01
    z = shapes.cyl_x(r, D.SV_HORN_FACE - 0.1, RX1 + 0.1, 0, 0)
    z += shapes.cyl_x(r, RX0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    z += pitch_offset() * shapes.cyl_y(r, D.SV_HORN_FACE - 0.1, PY1 + 0.1, 0, 0)
    z += pitch_offset() * shapes.cyl_y(r, PY0 - 0.1, D.SV_IDLER_FACE + 0.1, 0, 0)
    return z


def check_interfaces(glued, verbose=True):
    """HARD GATE: every servo interface -- the four pads with their bosses,
    bolt circles and centre reliefs -- must match the raw v5 clevises."""
    z = _pad_zones()
    a = (raw_clevises() & z).volume
    b = (glued & z).volume
    ok = abs(a - b) < 0.05
    if verbose:
        print(f"  [hip_yoke_glued] servo interfaces: {a:.3f} mm3 v5 clevises vs "
              f"{b:.3f} mm3 glued -> {'IDENTICAL' if ok else '** CHANGED **'}")
    return ok


def disc_seat_zones():
    """The volumes of this part that are DESIGNED to touch a neighbour, for
    check_assembly_v6's buffer rule (in this frame): the roll idler boss,
    which rides the yaw_carrier's bay bore at the 0.30 slip fit and seats on
    the idler disc, and the roll horn boss, which seats on the horn disc.
    Subtracting these from the posed part lets everything else -- arms,
    pads, both flanges, the fin, the whole pitch clevis -- be held to the
    full D.SWEEP_BUFFER against the carrier and the roll servo."""
    idler = shapes.cyl_x(D.IDLER_BOSS_D / 2 + 0.5, D.ROLL_ARM_INNER - 0.01,
                     D.SV_IDLER_FACE + 0.1, 0, 0)
    horn = shapes.cyl_x(D.HORN_BOSS_D / 2 + 0.5, D.SV_HORN_FACE - 0.1,
                    D.SV_HORN_FACE + D.HORN_BOSS_H + 0.01, 0, 0)
    return idler + horn


def SCREWS():
    """The 16 disc screws the yoke carries (part-local frame), length by the
    stack under each head (V.disc_screw, no washers): the roll horn arm sits
    on a 1.0 mm seating boss (stack 4.0), the roll idler arm's pad is sunk
    D.ROLL_IDLER_PAD_SINK so its 5.8 stack takes an M3x8, and the pitch
    clevis is leg_link's horn pad / idler pad pair (3.0 / 3.6). `axis`
    points from the head toward the tip. The joint has no fasteners."""
    r = D.BCD / 2
    ring = ((r, 0), (-r, 0), (0, r), (0, -r))
    rows = (("roll_horn", RX1, RX1 - D.SV_HORN_FACE, "horn", "x", -1),
            ("roll_idler", RX0 + D.ROLL_IDLER_PAD_SINK, D.SV_IDLER_FACE - (RX0 + D.ROLL_IDLER_PAD_SINK), "idler", "x", +1),
            ("pitch_horn", PY1, PY1 - D.SV_HORN_FACE, "horn", "y", -1),
            ("pitch_idler", PY0, D.SV_IDLER_FACE - PY0, "idler", "y", +1))
    s = []
    for tag, face, stack, side, ax, sgn in rows:
        length, eng, flange = V.disc_screw(stack, side)
        for a, b in ring:
            if ax == "x":
                pos, axis = (face, a, b), (sgn, 0, 0)
            else:
                pos, axis = (a, face, PZ + b), (0, sgn, 0)
            s.append(dict(name=f"{tag}_{a:+.0f}_{b:+.0f}", kind=f"M3x{length} button head",
                          pos=pos, axis=axis, length=float(length), stack=stack, engage=eng, flange=flange))
    return s


def mass_g(solid):
    return solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def snap_strain():
    """Peak bending strain in a finger deflected by its catch: a cantilever
    from SNAP_ROOT to the catch face, 1.5 t d / L^2."""
    t = SNAP_SHANK - SNAP_SLOT
    d = SNAP_BARB - SNAP_HOLE_CLR
    L = SNAP_ROOT - SNAP_CATCH
    return 1.5 * t * d / L ** 2


# What the audit is told, and why, for each piece -- the only findings it may
# raise that are not failures. Anything else fails.
#  - CONTACT: both pieces stand on edge on a first layer under the audit's
#    25 %-of-bbox floor, a tip-over risk for tall parts. These are 33-36 mm
#    tall on U-shaped footprints, and print WITHOUT a brim (Tom, 2026-10-14).
#    If one lifts, add a 5 mm outer brim in cad/v6/print_settings.json.
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
    from common import check_printability as CP
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
    """A strip of MuJoCo offscreen renders, one per shot = (items, azimuth,
    elevation); items is a solid or a list of (solid, rgba) drawn together.
    Camera on the items' bounding box."""
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


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    pieces = build()
    glued = pieces["hip_yoke_glued_roll"] + pieces["hip_yoke_glued_pitch"]
    findings = {}
    for name, s in pieces.items():
        n = len(s.solids())
        bb = s.bounding_box()
        path = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, path)
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        print(f"{name:22s} {n} solid(s)  {mass_g(s):5.2f} g  bbox "
              f"x {bb.min.X:.2f}..{bb.max.X:.2f} y {bb.min.Y:.2f}..{bb.max.Y:.2f} "
              f"z {bb.min.Z:.2f}..{bb.max.Z:.2f}")
        findings[name] = audit(name, path)
    print(f"glued pair {mass_g(glued):.2f} g  ({time.time() - t0:.0f}s)")
    lap = (Pos(0, 0, -0.05) * pieces["hip_yoke_glued_roll"] & pieces["hip_yoke_glued_pitch"]).volume / 0.05
    print(f"glued lap {lap:.0f} mm2   snap finger strain {snap_strain() * 100:.2f} %")
    check_interfaces(glued)
    for name, (fails, ok) in findings.items():
        notes = "; ".join(sorted({w for *_, w in ok}))
        print(f"printability {name}: "
              f"{'PASS, no supports' if not fails else f'** {len(fails)} FAILURE(S) **'}"
              f"{f' (accepted: {notes})' if notes else ''}")
        for kind, a, desc, _ in fails:
            print(f"    {kind:8s} {desc}")
    print("wrote", render(pieces, os.path.join(OUT_REN, "hip_yoke_glued.png")))
