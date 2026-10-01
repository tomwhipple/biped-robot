"""Plan B rig v3 (CONCEPT, issue #73): the servo held on BOTH case faces, and
the lever a FORK on the horn AND the idler disc -- the robot's own joint.

Why: the v2 bracket holds the servo by its idler face only, and the lever and
its load sit ~45 mm in front of that plate, so the plate twists about its own
long axis (Tom, 2026-10-01: "a fair amount of twisting on the vertical
plate"); and a rigid weight off the lever's plane rocks the lever. Here:

- the BRACKET is parts.leg_link's case grip, verbatim (plates on both case
  faces, the web under the cable-end half, 6 x M2.5x8 flat heads: 4 on the
  horn face at 8.30 / 29.00, 2 on the idler face at 32.75, the rib and
  platform detents), carried by two thick side walls and a base plate, and
  closed into a box by a strap behind the servo;
- the LEVER is a fork like leg_link's lower fork: an arm plate on the horn and
  one on the idler disc (4 x M3 each, the idler seating boss), joined by a
  floor along their length and a crossbar at the tip. The weights -- M10
  flange bolts with nuts and washers -- bolt through the floor at r 40, 70 and
  100 (ARM), halfway between the two output bearings, so the load sits on the
  servo's mid-plane and is rigid on the lever (nothing hangs, nothing swings).

The grip plates stop short of the output end exactly so the fork can swing
past them, as on the robot (the walls and base stop there too); the checks
sweep the fork from level to hanging.

Run: MUJOCO_GL=cgl .venv/bin/python experiments/plan-b-bench/plan_b_rig_v3.py
     (fit checks; STEP files for FreeCAD into $CLAUDE_JOB_DIR/tmp or ./_v3_step)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))

from build123d import Cylinder, Pos, Rot, export_step  # noqa: E402

import dimensions as D  # noqa: E402
import parts as PT  # noqa: E402
import plan_b_rig as R  # noqa: E402  (the world frame, servo_frame, box, vol)

# ------------------------------------------------- servo frame (leg_link's)
# x across the case, y the output axis (horn +y), z along the case (cable end -z)
T = D.GRIP_PLATE_T                                        # 2.4 horn grip plate
WEB_X1 = -12.36 - D.WEB_GAP                               # web inner face
WEB_X0 = WEB_X1 - 2.4                                     # web outer face
GRIP_X1 = 13.2                                            # plates' open edge
IDLER_SEAT = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR       # -14.90
IDLER_OUT = IDLER_SEAT - D.GRIP_PLATE_T_IDLER             # -17.90
GRIP_IY0 = D.IDLER_ARM_INNER - D.PLATE                    # leg_link's web idler end
# the fork: leg_link's lower-fork arm planes, on THIS servo
HY0, HY1 = D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE       # 20.45 .. 23.45
IY1 = D.IDLER_ARM_INNER                                   # -17.40
IY0 = IY1 - D.PLATE                                       # -20.40
MID_Y = (D.SV_HORN_FACE + D.SV_IDLER_FACE) / 2            # 1.825: between the bearings
ARM = R.ARM                                               # 100 mm to the load
FORK_W = D.PAD_D                                          # 20: arm width == the pad's diameter,
                                                          # edges tangent -- it prints flat
FLOOR_T = 3.0                                             # the web joining the arms across x
FLOOR_R0 = 20.0                                           # it starts here: the case corner is at
                                                          # r 15.97, the grip plate's at r 15.45
# the weights are M10 flange bolts with flange nuts and fender washers (Tom,
# 2026-10-01; the photo against the mat's grid: thread ~10, flange ~21, washer
# ~25 x 11.5), bolted through the floor on the servo's mid-plane
WEIGHT_BOLT_D = 10.5                                      # M10 clearance, close fit
WEIGHT_R = (ARM - 60, ARM - 30, ARM)                      # 40 / 70 / 100: 0.4 / 0.7 / 1.0 of the
                                                          # load point's torque per weight
WASHER_OD = 25.4                                          # 3/8" fender washer: the stack's widest
assert min(b - a for a, b in zip(WEIGHT_R, WEIGHT_R[1:])) > WASHER_OD   # stacks side by side
BAR_R0 = ARM + WASHER_OD / 2 + 1.0                        # crossbar just past the r 100 stack
BAR_T = 10.0                                              # crossbar depth along the arm

# ------------------------------------------------- walls and base (world)
WALL_T = 6.0
IDLER_WALL = (IDLER_OUT + R.MID_X - WALL_T, IDLER_OUT + R.MID_X)              # -9.15 .. -3.15
HORN_WALL = (D.SV_TOPFACE + T + R.MID_X, D.SV_TOPFACE + T + R.MID_X + WALL_T)  # 34.50 .. 40.50
FRONT_Y = R.AXIS_Y + D.WEB_TOP        # 34: walls and base stop where the grip's web does
WALL_TOP = R.AXIS_Z + GRIP_X1         # 33.2: the grip plates' open edge
BASE_Z1 = R.AXIS_Z + WEB_X1           # 7.24: the base's top IS the web's top
BACK_Y = -80.0                        # over the table, for the clamp
STRAP_Y = (-20.0, 10.0)               # closes the U into a box behind the servo
STRAP_T = 6.0
BACK_T = 6.0                          # the wall behind the servo (Tom, 2026-10-01): its
BACK_WALL_Y = (R.AXIS_Y + D.GRIP_BOT - BACK_T, R.AXIS_Y + D.GRIP_BOT)   # face where the grip
                                      # stops, 0.89 short of the case's cable end (14.89)


def servo_grip():
    """parts.leg_link's case grip, verbatim, servo frame (as in the v2 note:
    only the fork arms, brace, jog blocks and the web below GRIP_BOT -- the
    link's own structure -- are left out)."""
    t, idler_seat, _igo = T, IDLER_SEAT, IDLER_OUT
    p = PT.box(WEB_X0, GRIP_X1, D.SV_TOPFACE, D.SV_TOPFACE + t, D.GRIP_BOT, D.GRIP_TOP_HORN)
    p -= PT.cyl_y(D.GRIP_HORN_RELIEF, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1, 0, 0)
    p += PT.box(WEB_X0, GRIP_X1, _igo, idler_seat, D.GRIP_BOT, D.GRIP_TOP_IDLER)
    p += PT.box(WEB_X0, WEB_X1, GRIP_IY0, D.SV_TOPFACE + t, D.GRIP_BOT, D.WEB_TOP)
    p -= PT.box(-D.SV_HORN_RIB_HW - D.RIB_RELIEF_CLR, D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR,
                D.SV_TOPFACE - 0.01, D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR,
                -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR, D.GRIP_TOP_HORN + 0.01)
    _rx = D.SV_HORN_RIB_HW + D.RIB_RELIEF_CLR
    _ry0 = D.SV_TOPFACE - 0.01
    _ry1 = D.SV_TOPFACE + D.SV_HORN_RIB_H + D.RIB_RELIEF_DEPTH_CLR
    p -= PT.wedge_z([(_rx, _ry0), (_rx, _ry1), (_rx + 1.2 * (_ry1 - _ry0), _ry0)],
                    -D.SV_HORN_RIB_L[1] - D.RIB_RELIEF_CLR - 0.6,
                    -D.SV_HORN_RIB_L[0] + D.RIB_RELIEF_CLR + 0.6)
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= PT.box(-_ix, _ix, _iy1, idler_seat + 0.01, _iz0, _iz1)
    p -= PT.wedge_z([(_ix, idler_seat), (_ix, _iy1), (_ix + 1.2 * (idler_seat - _iy1), idler_seat)],
                    _iz0 - 0.6, _iz1 + 0.6)
    for zrow in D.CASE_HOLES_TOP:
        for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
            p -= PT.teardrop_y(D.CASE_SCREW_CLEAR / 2, D.SV_TOPFACE - 1, D.SV_TOPFACE + t + 1,
                               lx, -zrow, roll=90)
            p -= PT.csk_y(lx, -zrow, D.SV_TOPFACE + t, +1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        p -= PT.teardrop_y(D.CASE_SCREW_CLEAR / 2, _igo - 1, idler_seat + 1, lx,
                           -D.CASE_HOLES_BOT[1], roll=90)
        p -= PT.csk_y(lx, -D.CASE_HOLES_BOT[1], _igo, -1)
    return p


def idler_access(back=GRIP_IY0 - 1):
    """leg_link's idler access bores (0.05 into the plate), servo frame; the
    bracket carries them on through its wall (back=...)."""
    a = None
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        t = PT.teardrop_y(D.CASE_CS_D / 2 + 0.4, back, IDLER_OUT + 0.05, lx,
                          -D.CASE_HOLES_BOT[1], roll=90)
        a = t if a is None else a + t
    return a


def bracket():
    """The grip on a boxed frame: walls behind both grip plates, the base under
    them (its top is the web's), a wall across the channel behind the servo's
    cable end, and a strap across the top behind that."""
    xa, xb = IDLER_WALL[0], HORN_WALL[1]
    b = R.box(xa, xb, BACK_Y, FRONT_Y, 0.0, BASE_Z1)                       # base
    b += R.box(*IDLER_WALL, BACK_Y, FRONT_Y, 0.0, WALL_TOP)                # idler wall
    b += R.box(*HORN_WALL, BACK_Y, FRONT_Y, 0.0, WALL_TOP)                 # horn wall
    b += R.box(xa, xb, *STRAP_Y, WALL_TOP, WALL_TOP + STRAP_T)             # strap
    b += R.box(xa, xb, *BACK_WALL_Y, 0.0, WALL_TOP + STRAP_T)              # wall behind the servo
    b += R.servo_frame() * servo_grip()
    # head/driver access behind the countersinks the walls cover: the idler
    # row 32.75 and the horn row 29.00 (the horn 8.30 row is past FRONT_Y)
    b -= R.servo_frame() * idler_access(IDLER_OUT - WALL_T - 1)
    for lx in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
        b -= R.servo_frame() * PT.teardrop_y(D.CASE_CS_D / 2 + 0.4, D.SV_TOPFACE + T - 0.05,
                                             D.SV_TOPFACE + T + WALL_T + 1, lx,
                                             -D.CASE_HOLES_TOP[1], roll=90)
    # the connector trench's window through the idler wall and grip plate
    b -= R.servo_frame() * PT.box(-R.WIN_HW, R.WIN_HW, IDLER_OUT - WALL_T - 1, IDLER_SEAT + 1,
                                  *R.WIN_Z)
    return b


def fork():
    """Servo frame, pointing along +z (world: straight out, away from the table)."""
    length = BAR_R0 + BAR_T
    f = None
    for y0, y1 in ((HY0, HY1), (IY0, IY1)):
        arm = PT.box(-FORK_W / 2, FORK_W / 2, y0, y1, 0.0, length)
        arm += PT.cyl_y(D.PAD_D / 2, y0, y1, 0, 0)
        f = arm if f is None else f + arm
    f += PT.box(-FORK_W / 2, FORK_W / 2, IY0, HY1, BAR_R0, length)
    # the floor: joins the arms across x along their length (a channel, not
    # two cantilevers), on the -x side -- the bed when printed, the underside
    # when level -- clear of the case and the grip at every angle level..hanging
    f += PT.box(-FORK_W / 2, -FORK_W / 2 + FLOOR_T, IY0, HY1, FLOOR_R0, BAR_R0)
    f += PT.cyl_y(D.IDLER_BOSS_D / 2, IY1, D.SV_IDLER_FACE, 0, 0)       # seats on the idler disc
    # bores along y are horizontal in print: teardropped, peak +x (print-up)
    for h in PT.bcd_y(IY0 - 1, HY1 + 1, 0, 0, roll=90):
        f -= h
    f -= PT.teardrop_y(D.HORN_CENTER_RELIEF_D / 2, HY0 - 1, HY1 + 1, 0, 0, roll=90)
    f -= PT.teardrop_y(D.IDLER_CENTER_RELIEF_D / 2, IY0 - 1, D.SV_IDLER_FACE + 0.01, 0, 0,
                       roll=90)
    # the weight bolts, between the bearings: vertical through the floor (and
    # on the bed when printed -- round, no teardrop)
    for r in WEIGHT_R:
        f -= Pos(0, MID_Y, r) * Rot(0, 90, 0) * Cylinder(WEIGHT_BOLT_D / 2, FORK_W + 2)
    return f


def weight_stack(r):
    """Fork frame: the envelope of a bolted weight at radius r, HEAD UNDER the
    floor (14 mm, washer-wide) and the shank up into the channel (to 45 mm),
    its washers and nut stacked on the floor (30 mm). Not the other way up: at
    r 40, hanging, a shank out the bottom points at the table's edge."""
    x0, x1 = -FORK_W / 2, -FORK_W / 2 + FLOOR_T     # the floor's faces
    s = Pos(0, MID_Y, r) * Rot(0, 90, 0)
    return (s * Pos(0, 0, x0 - 7) * Cylinder(WASHER_OD / 2, 14)
            + s * Pos(0, 0, x1 + 15) * Cylinder(WASHER_OD / 2, 30)
            + s * Pos(0, 0, x0 + 22.5) * Cylinder(5.0, 45))


def fork_in_world(down_deg=0.0):
    """down_deg 0 = straight out (level), 90 = hanging straight down."""
    return R.servo_frame() * (Rot(0, -down_deg, 0) * fork())


def main():
    br, sv, tb = bracket(), R.servo_in_world(), R.table()
    ok = True

    def check(name, v, want_zero=True):
        nonlocal ok
        good = (v < 1e-3) if want_zero else (v > 1e-3)
        ok &= good
        print(f"  {'ok ' if good else 'BAD'} {name}: {v:.3f} mm3")

    print("fit checks (mm3 of overlap; the servo is the repo's conservative mock):")
    region = PT.box(WEB_X0, GRIP_X1, IDLER_OUT, D.SV_TOPFACE + T, D.GRIP_BOT, D.GRIP_TOP_HORN)
    ref, ours = PT.leg_link() & region, (servo_grip() - idler_access()) & region
    check("grip vs parts.leg_link over the grip region (symmetric difference)",
          ref.volume + ours.volume - 2 * (ref & ours).volume)
    check("bracket vs servo", R.vol(br, sv))
    check("bracket vs table", R.vol(br, tb))
    check("grip BEARS on the idler case face (servo 0.2 toward it)",
          R.vol(br, Pos(-0.2, 0, 0) * sv), want_zero=False)
    for d in (1.0, 5.0, 20.0, 40.0, 60.0):   # its way in: along the case, toward the output end
        check(f"servo {d:g} mm out along the case (slides in like a leg link)",
              R.vol(br, Pos(0, d, 0) * sv))
    fk = fork()
    stacks = None
    for r in WEIGHT_R:
        stacks = weight_stack(r) if stacks is None else stacks + weight_stack(r)
    check(f"weights at r {', '.join(f'{r:g}' for r in WEIGHT_R)} vs the fork itself",
          R.vol(stacks, fk))
    for a in (0, 15, 30, 45, 60, 75, 90):
        fw = fork_in_world(a)
        ww = R.servo_frame() * (Rot(0, -a, 0) * stacks)
        check(f"fork {a:2d} deg down vs servo", R.vol(fw, sv))
        check(f"fork {a:2d} deg down vs bracket", R.vol(fw, br))
        check(f"fork {a:2d} deg down vs table", R.vol(fw, tb))
        check(f"weights {a:2d} deg down vs servo + bracket + table",
              R.vol(ww, sv) + R.vol(ww, br) + R.vol(ww, tb))
    print(f"bracket {br.volume / 1000:.1f} cm3, fork {fk.volume / 1000:.1f} cm3 (solid); "
          f"load point {MID_Y:+.2f} from the servo's mid-plane datum, "
          f"bearing span {D.SV_HORN_FACE - D.SV_IDLER_FACE:.2f}")
    out = os.path.join(os.environ.get("CLAUDE_JOB_DIR", HERE), "tmp", "plan_b_rig")
    if "CLAUDE_JOB_DIR" not in os.environ:
        out = os.path.join(HERE, "_v3_step")
    os.makedirs(out, exist_ok=True)
    for name, solid in (("Bracket_v3", br), ("Fork_level", fork_in_world(0)),
                        ("Fork_hanging", fork_in_world(90)),
                        ("Weights_envelope_level", R.servo_frame() * stacks)):
        export_step(solid, os.path.join(out, name + ".step"))
    print(f"STEP: {out}")
    # print orientation: the bracket on its base, the fork on its floor (the
    # -x face of its frame down; its arms and pads rise 20 mm, bores teardropped up)
    from build123d import export_stl
    for name, part in (("bracket", R.to_bed(br, Rot(0, 0, 0))),
                       ("fork", R.to_bed(fork(), Rot(0, -90, 0)))):
        path = os.path.join(HERE, "stl", f"plan_b_v3_{name}.stl")
        export_stl(part, path)
        bb = part.bounding_box()
        print(f"  wrote {os.path.relpath(path, ROOT)} ({bb.size.X:.0f} x {bb.size.Y:.0f} x "
              f"{bb.size.Z:.0f} mm on the bed)")
    print("ALL CLEAR" if ok else "FIT CHECK FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
