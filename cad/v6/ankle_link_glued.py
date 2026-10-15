"""Ankle link, GLUED variant (qty 2 of each piece): a conformal cradle round the
ankle-pitch servo's base and a fork onto the ankle-roll servo, each printed
WITHOUT SUPPORTS, snapped together and glued (ANKLE_LINK_VARIANT=glued). The
method is hip_yoke_glued's; the brief is Tom's, 2026-10-14: "apply the same
methodology to the ankle link... Be sure to make the servo base conformal to
the servo body."

Local frame == ankle_link's: pitch axis == Y through the origin, the pitch
servo hanging in the link (case x +-12.36, bottom z -35.11); roll axis == X
at z -AL_DROP; +X forward.

WHY ankle_link CANNOT SIMPLY BE SPLIT LIKE THE YOKE
  Its box round the servo's base was not conformal: the front wall WAS the
  front tine carried up, 8.1 mm in front of the case, because the tine has
  to sit on the roll servo's horn disc (x 20.45) and it carries the whole
  roll torque (the idler side spins free). A wall that hugs the case needs
  another way for that torque to reach the box.

  CRADLE (ankle_link_glued_cradle, print IDENT, bottom down): a conformal
  four-wall tube round the servo's base: ankle_link's web and grip plates,
  and a front wall 0.4 mm off the case front (the web's gap), all from one
  flat bottom, TUBE_BOT. No floor: under the plates the roll servo's case
  sweeps up to z -34.6 at full roll, so a floor cannot run under them, and a
  cup whose plates stop short of its floor floats them in any print. The
  sweep's edge, z ~ SWEEP_Z0 + SWEEP_SLOPE |y|, is what sets TUBE_BOT and the
  45 deg chamfers on the tube's +-y bottom edges (_tube_chamfers).

  FORK (ankle_link_glued_fork, print RY_ROLL_WALL, model -Y down): the floor
  under the case (0.39 mm), both tines, and a closed FRONT BOX: the front
  tine carried up to END_TOP, a top bar, and an inner wall glued face to face
  onto the cradle's front wall. The roll torque goes tine -> box -> that
  260 mm2 lap -> cradle. All of it is one X-Z profile on one flat face,
  y FORK_Y0, the tines' -y edges (45 deg FEET under the pads): the floor,
  the box and the rear block start on it, and only their +y sides reach out to the
  tines' shoulders (+14.5).

THE JOINT
  The cradle drops onto two seats on the fork's floor, under its web and its
  front wall, between the front box's inner wall and the REAR BLOCK (the rear
  tine carried up to the same height and filled to 0.15 mm behind the web).
  A short barb on the block clicks into a groove in the middle of the web's
  back: the web bows 0.35 mm to pass it (before the servo is fitted; after,
  it cannot, and the snap is locked). A tongue on the inner wall runs in a
  groove up the front wall and locates y. Glue the seats and both laps.

NO LEAD WINDOW. ankle_link's web has leg_link's window at the case bottom,
  from when the lead was taken to leave the case there. The STS3215's bus
  ports are in the trench on its idler face, SV_CONN_L 11.75..16.35 mm from
  the axis (cad/README.md, servo geometry): z -11.75..-16.35 here, 9 mm above
  the cradle's idler plate. The leads plug in above the cradle; the web and
  the rear block are solid (Tom, 2026-10-14: "why the openings on the -X side?").

    .venv/bin/python cad/v6/ankle_link_glued.py   # stl/step + audits + mass + render
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

from build123d import Pos, export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts  # noqa: E402
import ankle_link as AL  # noqa: E402
import paths  # noqa: E402
D = V.D

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

PIECES = ("ankle_link_glued_cradle", "ankle_link_glued_fork")
# rows = print axes in model coords (check_printability's convention)
PRINT_ORIENT = {
    "ankle_link_glued_cradle": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),   # IDENT
    "ankle_link_glued_fork": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),    # RY_ROLL_WALL
}

AL_DROP = V.AL_DROP
CASE_X = D.SV_WID / 2                                   # 12.36
CASE_BOT = -D.SV_AXIS_FROM_REAR                         # -35.11
WALL_T = AL.WEB_X1 - AL.WEB_X0                          # 2.4, the web's
FRONT_X = (CASE_X + D.WEB_GAP, CASE_X + D.WEB_GAP + WALL_T)   # 12.76 .. 15.16
END_TOP = AL.END_TOP                                    # -25.05

# The roll servo's case swept through the ankle-roll ROM (ankle_link's
# padded _roll_sweep_clearance_cut, both mounting orientations) reaches up
# to z ~ SWEEP_Z0 + SWEEP_SLOPE * |y| under this part: -37.2 at |y| 14.5,
# -35.8 at the horn plate's inner face, -34.6 at its outer face (measured;
# check_sweep() holds it).
SWEEP_Z0, SWEEP_SLOPE = -44.224, 0.4878
# The cradle's flat bottom, 0.11 mm above the case bottom. Its chamfer off
# the horn plate's bottom-outer edge starts where the sweep meets it,
# CHAMFER_Y, and that has to be outboard of the rib detent's pocket (y to
# 18.78): the pocket's lower end, fixed by the rib at z -34.66, comes within
# 0.45 mm of the sweep there, so the plate cannot close under it in a print
# off this bottom. The pocket runs out through the bottom instead (_tube_cuts;
# Tom, 2026-10-14: no open slice in that chamfer).
TUBE_BOT = -35.0
CHAMFER_Y = (TUBE_BOT - SWEEP_Z0) / SWEEP_SLOPE         # 18.91: the sweep meets TUBE_BOT here
RIB_POCKET_Y1 = D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR
assert CHAMFER_Y > RIB_POCKET_Y1, "the horn-side chamfer would cut the rib pocket's back wall"
# the floor stays under the case (0.39 mm); the cradle sits on two ribs on it
# under the web and the front wall
FLOOR_TOP = CASE_BOT - 0.39                             # -35.5

# the fork's bed face: tangent to the pads (r 10), as hip_yoke_glued's roll
# U. Not the shoulders' -14.5 nor the tines' -12: at full ankle roll the
# foot's near-row parts beside the idler disc swing in to y -12.1 just under
# the roll axis, where a pad foot from either face would reach (the gate's
# foot vs ankle_link row: 0.60 mm3 at -14.5). From -10 the feet keep 2 mm.
FORK_Y0 = -D.PAD_D / 2                                  # -10.0
FORK_Y1 = V.AL_SHOULDER_Y                               # 14.5
FLOOR_Z = (FLOOR_TOP - D.PLATE - 0.2, FLOOR_TOP)        # -38.7 .. -35.5
FLOOR_Y = (FORK_Y0, V.AL_FLOOR_Y[1])                    # -10.0 .. 14.0
FLOOR_X = (AL.X_REAR, AL.X_FRONT)                       # -19.8 .. 23.45

LAP_CLR = 0.15                                          # glue gap, both laps
INNER_T = 2.0
INNER_X = (FRONT_X[1] + LAP_CLR, FRONT_X[1] + LAP_CLR + INNER_T)   # 15.31 .. 17.31
TOPBAR_T = 2.0
TONGUE = (0.8, 1.0)                                     # proud of the inner wall, half-width
TONGUE_CLR = 0.15

# The rear block: the rear tine carried up to the front box's height and
# filled solid to LAP_CLR behind the web, so the fork stands up on both sides
# of the cradle and laps it front and back (Tom, 2026-10-14: the bare rear
# looked off; "make sure that part merges with the tab to its immediate +x").
REAR_X = (V.AL_TINE_REAR_X[0], AL.WEB_X0 - LAP_CLR)    # -19.8 .. -15.31
# The snap: a short barb on the block, centred on the web, into a groove in
# the web's back. The block is rigid, so the WEB gives: before the servo goes
# in it bows BARB_IN between the two grip plates as the barb passes (its
# bottom-outer chamfer leads it in); with the servo fitted, 0.4 mm behind it,
# it cannot bow that far again, so the snap is then locked.
GROOVE_D = 0.5                                          # into the web
BARB_IN = 0.35                                          # barb tip past the web's face
BARB_HW = 4.0                                           # barb half-width along y
CATCH_Z = -27.0                                         # groove's flat bottom; the barb's 0.1 above
LEAD = 0.7                                              # web's bottom-outer chamfer (snap lead-in), deep
# The tube's chamfers off its bed edge rise at 50 deg, not 45: a 45 deg face
# at the bed is exactly the audit's (and the slicer's) overhang limit.
CHAMFER_SLOPE = 1.2

_T = D.PAD_D / 2 / 2 ** 0.5                             # 7.071
# 45 deg foot under each pad's -y side: the bed face runs on to where a
# 45 deg face tangent to the pad meets it, and the triangle closes on a chord
# of the pad (the bolt circle stays 2.8 mm clear of it), (y, z)
PAD_FOOT = [(FORK_Y0, -AL_DROP), (FORK_Y0, -AL_DROP - (2 * _T + FORK_Y0)), (-_T, -AL_DROP - _T)]


# ------------------------------------------------------------------ cradle
def _tube():
    w0, w1 = AL.WEB_X0, AL.WEB_X1
    hy, iy = AL.HORN_Y, AL.IDLER_Y
    p = parts.box(w0, FRONT_X[1], *hy, TUBE_BOT, END_TOP)                    # horn plate
    p += parts.box(w0, AL.GRIP_X1, *hy, TUBE_BOT, D.GRIP_TOP_HORN)           #   and its upper part
    p += parts.box(w0, FRONT_X[1], *iy, TUBE_BOT, END_TOP)                   # idler plate
    p += parts.box(w0, w1, iy[0], hy[1], TUBE_BOT, D.WEB_TOP)                # web
    p += parts.box(*FRONT_X, iy[0], hy[1], TUBE_BOT, END_TOP)                # front wall
    return p


def _tube_chamfers():
    """Chamfers off the tube's two bottom outer edges along x, from where the
    roll servo's sweep meets TUBE_BOT (+-CHAMFER_Y) outward: the sweep rises
    at SWEEP_SLOPE, the chamfer at CHAMFER_SLOPE, so it clears it everywhere."""
    cut = None
    for s in (1, -1):
        prof = [(s * CHAMFER_Y, TUBE_BOT - 1), (s * 30, TUBE_BOT - 1),
                (s * 30, TUBE_BOT + (30 - CHAMFER_Y) * CHAMFER_SLOPE), (s * CHAMFER_Y, TUBE_BOT)]
        w = parts.wedge_x(prof, AL.WEB_X0 - 1, FRONT_X[1] + 1)
        cut = w if cut is None else cut + w
    return cut


def _web_cuts():
    """The snap groove across the web's back (flat bottom = the catch, 45 deg
    roof) and the web's bottom-outer lead-in chamfer."""
    w0 = AL.WEB_X0
    groove = parts.wedge_y([(w0 - 1, CATCH_Z), (w0 + GROOVE_D, CATCH_Z),
                            (w0 + GROOVE_D, CATCH_Z + GROOVE_D), (w0 - 1, CATCH_Z + 2 * GROOVE_D + 1)],
                           -BARB_HW - 0.5, BARB_HW + 0.5)
    k = CHAMFER_SLOPE
    lead = parts.wedge_y([(w0 - 1, TUBE_BOT - 1), (w0 + LEAD + 1 / k, TUBE_BOT - 1),
                          (w0 - 1, TUBE_BOT + (LEAD + 1) * k)],
                         AL.IDLER_Y[0] - 1, AL.HORN_Y[1] + 1)
    return groove + lead


def _rib_pocket_runout():
    """ankle_link's rib detent (the horn plate's inner-face pocket for the
    servo's horn-side rib), carried down out through the cradle's bottom."""
    hw = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    top = -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR + 0.01
    return parts.box(-hw, hw, D.SV_TOPFACE - 0.01, RIB_POCKET_Y1, TUBE_BOT - 1, top)


def _front_groove():
    """The tongue's groove up the front wall's outer face, open at both ends."""
    tx, th = TONGUE
    x0 = INNER_X[0] - tx - TONGUE_CLR
    return parts.box(x0, FRONT_X[1] + 1, -th - TONGUE_CLR, th + TONGUE_CLR, TUBE_BOT - 1, END_TOP + 1)


def cradle():
    p = AL._grip_features(_tube(), roll=0)               # print up = model +z
    p -= _tube_chamfers()
    p -= _web_cuts()
    p -= _front_groove()
    p -= _rib_pocket_runout()
    p -= AL._roll_sweep_clearance_cut()
    p -= AL._tab_keepout()
    return _one(p, "cradle")


# -------------------------------------------------------------------- fork
# The rear tine's pad foot stops short of the foot's rear-tab band (x from
# FOOT_TAB_REAR_X[0]), D.SWEEP_BUFFER clear: on the right leg, at -19..-25 deg
# of roll, that band passes 0.5 mm outside the bare pad rim, where the foot
# would be (gate, --side R: 0.12 mm3). The tine's inner 1.0 mm keeps the bare
# rim, a 1 mm ledge in the print.
REAR_FOOT_X1 = V.FOOT_TAB_REAR_X[0] - D.SWEEP_BUFFER     # -17.8


def _tine(x0, x1, top, foot_x1=None):
    hw, sy = V.AL_TINE_W / 2, V.AL_SHOULDER_Y
    z_fl = FLOOR_Z[0]
    z_sh = z_fl - (sy - hw)
    prof = [(FORK_Y0, -AL_DROP), (hw, -AL_DROP), (hw, z_sh), (sy, z_fl), (sy, top), (FORK_Y0, top)]
    t = parts.wedge_x(prof, x0, x1)
    t += parts.cyl_x(D.PAD_D / 2, x0, x1, 0, -AL_DROP)
    t += parts.wedge_x(PAD_FOOT, x0, min(x1, foot_x1) if foot_x1 is not None else x1)
    return t


def _rear_block():
    """The rear tine's fill up to the web (the tine itself is carried up by
    _tine) and the snap barb on its face: flat catch on top of the groove's
    bottom, 45 deg back to the face above."""
    x1 = REAR_X[1]
    b = parts.box(V.AL_TINE_REAR_X[1] - 0.01, x1, FORK_Y0, FORK_Y1, FLOOR_Z[1] - 0.01, END_TOP)
    tip = AL.WEB_X0 + BARB_IN
    zc = CATCH_Z + 0.1
    b += parts.wedge_y([(x1 - 0.01, zc), (tip, zc), (tip, zc + 0.3), (x1 - 0.01, zc + 0.3 + tip - x1 + 0.01)],
                       -BARB_HW, BARB_HW)
    return b


def fork():
    x_r, x_f = V.AL_TINE_REAR_X, V.AL_TINE_FRONT_X
    p = parts.box(*FLOOR_X, *FLOOR_Y, *FLOOR_Z)
    for x0, x1 in ((AL.WEB_X0, AL.WEB_X1), FRONT_X):                        # the cradle's seats
        p += parts.box(x0, x1, *FLOOR_Y, FLOOR_Z[1] - 0.01, TUBE_BOT)
    p += _tine(*x_r, END_TOP, foot_x1=REAR_FOOT_X1)                            # the rear block's back
    p += _tine(*x_f, END_TOP)                                                  # the front box's front
    p += parts.box(*INNER_X, FORK_Y0, FORK_Y1, FLOOR_Z[0], END_TOP)             # inner wall
    p += parts.box(INNER_X[1] - 0.01, x_f[0] + 0.01, FORK_Y0, FORK_Y1, END_TOP - TOPBAR_T, END_TOP)
    tx, th = TONGUE
    p += parts.box(INNER_X[0] - tx, INNER_X[0] + 0.01, -th, th, TUBE_BOT, END_TOP)
    p += _rear_block()
    # pads: centre relief and the 4x M3 BCD, as ankle_link's
    r = D.BCD / 2
    for (x0, x1), relief_d in ((x_f, D.HORN_CENTER_RELIEF_D), (x_r, D.IDLER_CENTER_RELIEF_D)):
        p -= parts.cyl_x(relief_d / 2, x0 - 1, x1 + 1, 0, -AL_DROP)
        for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
            p -= parts.cyl_x(D.PAD_HOLE / 2, x0 - 1, x1 + 1, dy, -AL_DROP + dz)
    p -= AL._roll_sweep_clearance_cut()
    p -= AL._tab_keepout()
    return _one(p, "fork")


def _one(p, tag):
    """Keep the piece; say so if a boolean left anything but fuzz."""
    solids = sorted(p.solids(), key=lambda s: s.volume, reverse=True)
    if len(solids) > 1:
        junk = sum(s.volume for s in solids[1:])
        print(f"  [ankle_link_glued] {tag}: {len(solids) - 1} extra solid(s), {junk:.3f} mm3 "
              f"({'boolean fuzz' if junk < 1.0 else '** REAL DETACHED VOLUME **'})")
        p = solids[0]
    return p


def build():
    return {"ankle_link_glued_cradle": cradle(), "ankle_link_glued_fork": fork()}


def ankle_link_glued():
    """The two pieces as glued: the solid the assembly and the CAD gate use."""
    b = build()
    return b["ankle_link_glued_cradle"] + b["ankle_link_glued_fork"]


# ------------------------------------------------------------------ checks
def check_sweep(verbose=True):
    """The tube's flat bottom and chamfers must clear the padded roll sweep on
    their own, before ankle_link's exact sweep cut: whatever that cut still
    removes from the chamfered tube is a face the print would not get."""
    raw = AL._grip_features(_tube(), roll=0) - _tube_chamfers()
    v = (raw & AL._roll_sweep_clearance_cut()).volume
    ok = v < 0.05
    if verbose:
        print(f"  [ankle_link_glued] tube vs the roll sweep before its cut: {v:.3f} mm3 "
              f"-> {'clear' if ok else '** the chamfers do not cover it **'}")
    return ok


def check_interfaces(glued, verbose=True):
    """The servo interfaces must be ankle_link's: the four grip-screw seats
    and the detents (the grip channel above TUBE_BOT) and both pads (r 10
    round the roll axis, tines' x). Compared as volumes inside those zones."""
    old = AL.ankle_link()
    r = D.PAD_D / 2 - 0.01
    zones = None
    for x0, x1 in (V.AL_TINE_FRONT_X, V.AL_TINE_REAR_X):
        c = parts.cyl_x(r, x0, x1, 0, -AL_DROP)
        zones = c if zones is None else zones + c
    a, b = (old & zones).volume, (glued & zones).volume
    # the two grip plates over the case's own width, above the new bottom and
    # its chamfers (which top out at z -33.1, under the lowest countersink rim,
    # -31.7): every case-grip screw seat and both detents
    x = FRONT_X[0] - 0.05
    z0 = TUBE_BOT + (AL.HORN_Y[1] - CHAMFER_Y) * CHAMFER_SLOPE + 0.1
    grip = parts.box(-x, x, *AL.HORN_Y, z0, D.GRIP_TOP_HORN)
    grip += parts.box(-x, x, *AL.IDLER_Y, z0, END_TOP)
    # less each screw bore's teardrop roof, which points up in each part's own
    # print (+x in ankle_link's, +z in the cradle's)
    h = D.CASE_SCREW_CLEAR / 2 + 0.1
    for zrow, (y0, y1) in [(z, AL.HORN_Y) for z in D.CASE_HOLES_TOP] + [(D.CASE_HOLES_BOT[1], AL.IDLER_Y)]:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            grip -= parts.box(lx - h, lx + h, y0 - 1, y1 + 1, -zrow - h, -zrow + h)
    g_old, g_new = (old & grip).volume, (glued & grip).volume
    ok = abs(a - b) < 0.05 and abs(g_old - g_new) < 0.05
    if verbose:
        print(f"  [ankle_link_glued] pads: {a:.3f} vs {b:.3f} mm3; grip plates: {g_old:.3f} vs "
              f"{g_new:.3f} mm3 -> {'IDENTICAL' if ok else '** CHANGED **'}")
    return ok


def check_snap_path(pieces=None, verbose=True):
    """The cradle lowered onto the fork along -z, 12 mm to seated: on the way
    the only overlap may be the barb riding over the web's back (BARB_IN deep,
    inside the barb's own box), and seated there must be none."""
    pieces = pieces or build()
    c, f = pieces["ankle_link_glued_cradle"], pieces["ankle_link_glued_fork"]
    barb = parts.box(AL.WEB_X0 - 0.01, AL.WEB_X0 + BARB_IN + 0.01, -BARB_HW - 0.01, BARB_HW + 0.01,
                     CATCH_Z, CATCH_Z + 1.0)
    worst, stray = 0.0, []
    for i in range(25):
        dz = 12.0 - 0.5 * i
        ov = Pos(0, 0, dz) * c & f
        v = ov.volume
        if v < 1e-3:
            continue
        outside = (ov - barb).volume
        worst = max(worst, v)
        if outside > 0.01 or dz == 0:
            stray.append((dz, v, outside))
    ok = not stray
    if verbose:
        print(f"  [ankle_link_glued] snap path: barb rides the web, worst {worst:.2f} mm3; "
              f"{'seats clean' if ok else f'** other overlaps: {stray} **'}")
    return ok


def snap_strain():
    """Peak strain in the web bowed BARB_IN at mid-span by the barb: a strip
    fixed at both grip plates, 12 d t / L^2."""
    span = AL.HORN_Y[0] - AL.IDLER_Y[1]                  # between the plates' inner faces
    return 12 * BARB_IN * (AL.WEB_X1 - AL.WEB_X0) / span ** 2


# Findings the audit may raise that are not failures (cf. hip_yoke_glued):
#  - CONTACT: the cradle stands on its walls' bottoms and the fork on edge,
#    both under the audit's 25 %-of-bbox floor; printed without a brim
#    (Tom, 2026-10-14, for the hip yoke). Add a 5 mm outer brim if one lifts.
def _expected(name, kind, desc):
    if kind == "CONTACT":
        return "small first layer, no brim"
    return None


def audit(name, stl_path):
    import numpy as np
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT[name] = (np.array(PRINT_ORIENT[name]), "glued ankle link piece, no supports")
    CP.PRINT_STL[name] = os.path.basename(stl_path)
    CP.SUPPORTED.pop(name, None)
    fails, ok = [], []
    for kind, a, desc in CP.audit(name):
        why = _expected(name, kind, desc)
        (ok if why else fails).append((kind, a, desc, why))
    return fails, ok


def mass_g(s):
    return s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


def in_print_orient(solid, name):
    """The piece as it lies on the bed: PRINT_ORIENT applied, bed at z 0."""
    from build123d import Matrix
    r = PRINT_ORIENT[name]
    s = solid.transform_geometry(Matrix([[*r[0], 0], [*r[1], 0], [*r[2], 0], [0, 0, 0, 1]]))
    bb = s.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s


ORANGE, GREY, SERVO = (0.95, 0.55, 0.20, 1), (0.72, 0.74, 0.78, 1), (0.22, 0.23, 0.27, 1)


def render(pieces, png_path, px=560):
    """One strip: each piece as it lies on the bed, the pair exploded along
    the snap's axis, and glued with the pitch servo seated in the cradle."""
    import check_assembly as CA
    import hip_yoke_glued as HG
    c, f = pieces["ankle_link_glued_cradle"], pieces["ankle_link_glued_fork"]
    return HG.render_shots([
        ([(in_print_orient(c, "ankle_link_glued_cradle"), ORANGE)], 135, -35),
        ([(in_print_orient(f, "ankle_link_glued_fork"), GREY)], 135, -35),
        ([(Pos(0, 0, 16) * c, ORANGE), (f, GREY)], 140, -15),
        ([(c, ORANGE), (f, GREY), (CA.servo_mock(), SERVO)], 140, -15),
    ], png_path, px)


def SCREWS():
    """ankle_link's: the grip screws and the tine pads are where they were."""
    return AL.SCREWS()


SERVO_INSERT = AL.SERVO_INSERT


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    pieces = build()
    glued = pieces["ankle_link_glued_cradle"] + pieces["ankle_link_glued_fork"]
    findings = {}
    for name, s in pieces.items():
        bb = s.bounding_box()
        path = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, path)
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        print(f"{name:24s} {len(s.solids())} solid(s)  {mass_g(s):5.2f} g  bbox "
              f"x {bb.min.X:.2f}..{bb.max.X:.2f} y {bb.min.Y:.2f}..{bb.max.Y:.2f} "
              f"z {bb.min.Z:.2f}..{bb.max.Z:.2f}")
        findings[name] = audit(name, path)
    print(f"glued pair {mass_g(glued):.2f} g vs ankle_link {mass_g(AL.ankle_link()):.2f} g  "
          f"({time.time() - t0:.0f}s)")
    print(f"snap: web bows {BARB_IN} mm, strain {snap_strain() * 100:.2f} %")
    check_sweep()
    check_interfaces(glued)
    check_snap_path(pieces)
    for name, (fails, ok) in findings.items():
        notes = "; ".join(sorted({w for *_, w in ok}))
        print(f"printability {name}: "
              f"{'PASS, no supports' if not fails else f'** {len(fails)} FAILURE(S) **'}"
              f"{f' (accepted: {notes})' if notes else ''}")
        for kind, a, desc, _ in fails:
            print(f"    {kind:8s} {desc}")
    print("wrote", render(pieces, os.path.join(OUT_REN, "ankle_link_glued.png")))
