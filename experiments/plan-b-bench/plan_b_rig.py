"""Plan B bench rig (issue #73): the printed parts that put a lever and weights
on ONE loose STS3215 for tools/gain_bench.py `stiffness` and `hold`.

Design note, diagram and procedure: docs/design-v6/2026-09-30-plan-b-rig.md.

    bracket     clamps to a table edge; holds the servo by its idler face,
                output axis horizontal and PARALLEL to the edge, 50 mm out
                past the edge and 20 mm above the tabletop
    lever_stiff 130 mm lever, V-notches on both edges at 50 / 75 / 100 mm:
                a string loop in the 100 mm notch carries the bottle bag
    lever_hold  the same root, with a cup at 100 mm: ~0.25 kg of packed
                table salt (or sand, or steel hardware) is the leg-like
                inertia. Rigid -- a hung weight would decouple and hide a
                limit cycle.
    lid         press-fit lid for the cup (tape it)
    stop_pin    removable catch 15 deg below horizontal: if torque drops with
                1.5 kg hung, the lever lands on it instead of swinging

World frame (the rig on the table): x = the servo's output axis (horn +x),
y = away from the table (edge at y = 0, table at y < 0), z = up (tabletop
at z = 0). The servo's idler case face is the plane x = 0.

Seating: the servo is screwed to ONE face of the bracket -- its idler face --
through all four of that face's case holes (rows 8.30 and 32.75 behind the
axis, +/-10.25 across; ST3215 outline drawing), on a CONFORMAL seat built from
the leg link's idler grip plate (parts.leg_link): the plate bears on the real
case face at GRIP_SEAT_CLR, with the back-cover platform detent, M2.5x8 flat
heads countersunk flush in teardrop bores, and head/driver access through the
plate behind. Past the leg link's footprint the seat also has to clear the
idler disc and hub (they ROTATE: a pocket) and the connector trench (the
sockets open out of this face: a window the plugs and leads pass through). A
cradle hugs the case's two long sides so the lever torque is also taken in
bearing.

Run:  .venv/bin/python experiments/plan-b-bench/plan_b_rig.py   (writes stl/plan_b_*.stl
      beside it,
      print orientation, and prints the fit checks)
"""
import math
import os
import re
import sys

from build123d import (Box, Cone, Cylinder, Location, Plane, Pos, Rot,
                       export_stl, extrude, Polygon)

HERE = os.path.dirname(os.path.abspath(__file__))          # experiments/plan-b-bench/
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))                 # dimensions, parts, check_assembly
import dimensions as D  # noqa: E402
import parts as PT  # noqa: E402  (the leg link's own helpers, for idler_seat)

# ------------------------------------------------------------------ the rig
AXIS_Y = 50.0          # output axis past the table edge
AXIS_Z = 20.0          # output axis above the tabletop
MID_X = -D.SV_IDLER_CASE_FACE           # servo mid-plane: idler case face at x 0
HORN_X = MID_X + D.SV_HORN_FACE         # 35.20, lever seats here

# ------------------------------------------------------------------ bracket
# The idler seat, in the servo frame (x across the case, y the output axis
# with the horn +y, z along the case, cable end -z): leg_link's idler grip
# plate -- the same expressions -- run the full case length.
SEAT_IN = D.SV_IDLER_CASE_FACE - D.GRIP_SEAT_CLR             # -14.90, bears on the case
SEAT_OUT = SEAT_IN - D.GRIP_PLATE_T_IDLER                    # -17.90, heads flush here
SEAT_X = (-12.36 - D.WEB_GAP - 2.4, 13.2)                    # leg_link's plate width
SEAT_Z = (D.GRIP_BOT, D.SV_AXIS_FROM_OUT_END + 0.5)          # -36 .. past the output end
DISC_R = D.GRIP_HORN_RELIEF                                  # 10.30 round the O19.2 disc
DISC_FLOOR = D.SV_BOTFACE - 0.3                              # -17.65: 0.3 past the hub
# the bracket plate (world): its front face is the seat's back face
PLATE_X1 = SEAT_OUT - D.SV_IDLER_CASE_FACE                   # -3.15
SLAB_T = 6.0
SLAB_X0 = PLATE_X1 - SLAB_T                                  # -9.15, back face (on the bed)
# connector WINDOW over the measured trench, through seat and plate (the roll
# bay's margins, parts.py: 0.45 axis side, 1.0 cable-end side, +/-0.5 across)
WIN_Z = (-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 0.45)      # servo frame
WIN_HW = D.SV_CONN_HW + 0.5
CRADLE_CLR = 0.25      # per side, across the case width
CRADLE_T = 4.0
CRADLE_X1 = 10.0       # cradle walls reach 10 mm up the case side
CRADLE_Y = (18.0, 56.0)
PLATE_Y = (-60.0, 100.0)
PLATE_Z1 = 50.0
TAB_Z0 = -16.0         # below the tabletop, only past the edge
TAB_Y0 = 2.0
BASE_Y0 = -80.0
BASE_X1 = 60.0
BASE_T = 6.0
GUSSET_T = 5.0

# ------------------------------------------------------------------ levers
LEVER_T = 8.0          # M3x10 button head -> 2.0 into the horn's 2.5 thread
LEVER_H = 18.0
LEVER_L = 130.0        # axis to tip
PAD_R = 12.0
ARM = 100.0            # the test lever arm
NOTCHES = (50.0, 75.0, 100.0)
NOTCH_DEPTH = 3.0      # 90 deg V
CUP_ID = 54.0
CUP_WALL = 2.5
CUP_FLOOR = 3.0
CUP_IN_H = 70.0        # 160 cm3: ~190 g of table salt, ~0.25 kg with cup + lid
LID_T = 2.0
LID_SPIGOT_H = 5.0
LID_SPIGOT_CLR = 0.2

# ------------------------------------------------------------------ stop pin
PIN_D = 12.0
PIN_HOLE_D = 12.4
PIN_R = 40.0           # along the lever from the axis
PIN_DROP_DEG = 15.0
PIN_HEAD_D = 18.0
PIN_HEAD_T = 4.0
PIN_REACH_X = HORN_X + LEVER_T + 4.0    # past the lever's outer face


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_x(r, x0, x1, y, z):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, x1 - x0)


def cone_x(r0, r1, x0, x1, y, z):
    """r0 at x0, r1 at x1."""
    c = Cone(r0, r1, abs(x1 - x0))
    c = Rot(0, 90, 0) * c if x1 > x0 else Rot(0, -90, 0) * c
    return Pos((x0 + x1) / 2, y, z) * c


def pin_centre():
    """(y, z) of the pin: its surface touches the lever's lower edge once the
    lever has dropped PIN_DROP_DEG about the axis."""
    a = math.radians(PIN_DROP_DEG)
    n = LEVER_H / 2 + PIN_D / 2
    dy = PIN_R * math.cos(a) - n * math.sin(a)
    dz = -(PIN_R * math.sin(a) + n * math.cos(a))
    return AXIS_Y + dy, AXIS_Z + dz


def case_holes():
    """The idler face's four case holes, servo frame (x, z): rows 8.30 and
    32.75 behind the axis, +/-10.25 across (the outline drawing's idler view)."""
    return [(s * D.CASE_HOLE_LAT, -row) for row in D.CASE_HOLES_BOT for s in (1, -1)]


def idler_seat():
    """The conformal seat on the idler face, servo frame. leg_link's idler
    grip plate (seat, platform detent + its ramp, teardrop bores, flush
    countersinks), run the whole case length so it takes BOTH hole rows,
    plus the two features that length meets: the rotating disc and hub (a
    pocket) and the connector trench (a window, cut in bracket())."""
    p = PT.box(*SEAT_X, SEAT_OUT, SEAT_IN, *SEAT_Z)
    # platform DETENT and its ramp (leg_link, verbatim)
    _ix = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    _iy0 = SEAT_IN
    _iy1 = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    _iz0 = D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR
    _iz1 = D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR
    p -= PT.box(-_ix, _ix, _iy1, _iy0 + 0.01, _iz0, _iz1)
    _irr = 1.2 * (_iy0 - _iy1)
    p -= PT.wedge_z([(_ix, _iy0), (_ix, _iy1), (_ix + _irr, _iy0)], _iz0 - 0.6, _iz1 + 0.6)
    # the idler disc and hub turn with the joint: never touched
    p -= PT.cyl_y(DISC_R, DISC_FLOOR, SEAT_IN + 1, 0, 0)
    # case screws, M2.5x8 flat head countersunk flush (leg_link's idler row,
    # on both rows)
    for x, z in case_holes():
        p -= PT.teardrop_y(D.CASE_SCREW_CLEAR / 2, SEAT_OUT - 1, SEAT_IN + 1, x, z, roll=90)
        p -= PT.csk_y(x, z, SEAT_OUT, -1)
    return p


def access_bores():
    """Head/driver access behind each seat countersink, servo frame: leg_link's
    idler access bore (0.05 into the seat), carried through the plate."""
    a = None
    for x, z in case_holes():
        t = PT.teardrop_y(D.CASE_CS_D / 2 + 0.4, SEAT_OUT - SLAB_T - 1, SEAT_OUT + 0.05,
                          x, z, roll=90)
        a = t if a is None else a + t
    return a


def bracket():
    b = box(SLAB_X0, PLATE_X1, *PLATE_Y, 0.0, PLATE_Z1)
    b += box(SLAB_X0, PLATE_X1, TAB_Y0, PLATE_Y[1], TAB_Z0, 0.0)
    # base on the tabletop, on the servo side, over the table only
    b += box(SLAB_X0, BASE_X1, BASE_Y0, 0.0, 0.0, BASE_T)
    for y0 in (-GUSSET_T - 1.0, PLATE_Y[0]):
        b += _gusset(y0)
    # the conformal seat on the plate's front face, and the cradle
    b += servo_frame() * idler_seat()
    seat_x = -D.GRIP_SEAT_CLR                        # the seat face, world
    half = D.SV_WID / 2 + CRADLE_CLR
    for z0, z1 in ((AXIS_Z + half, AXIS_Z + half + CRADLE_T),
                   (AXIS_Z - half - CRADLE_T, AXIS_Z - half)):
        b += box(seat_x, CRADLE_X1, *CRADLE_Y, z0, z1)
    # through seat + plate: the connector window; through the plate: each
    # screw's head/driver access (leg_link's access bore, carried to the back
    # face); and the stop-pin hole
    b -= servo_frame() * PT.box(-WIN_HW, WIN_HW, SEAT_OUT - SLAB_T - 1, SEAT_IN + 1, *WIN_Z)
    b -= servo_frame() * access_bores()
    py, pz = pin_centre()
    b -= cyl_x(PIN_HOLE_D / 2, SLAB_X0 - 1, 1.0, py, pz)
    return b


def _gusset(y0):
    """Triangular rib in the plane y = const, slab face to base top."""
    x0, x1, z0, z1 = PLATE_X1, BASE_X1 - 5.0, BASE_T, PLATE_Z1 - 2.0
    tri = Polygon((x0, z0), (x1, z0), (x0, z1), align=None)
    # Polygon lives in XY; stand it up into XZ and give it thickness in +y
    return Pos(0, y0, 0) * Rot(90, 0, 0) * extrude(tri, amount=-GUSSET_T)


def lever_root():
    """Pad + 4x M3 on the horn's O14 circle + O6.2 centre relief, lever frame:
    axis at the origin, lever along +x, horn face at z = 0, outer face +z."""
    r = Pos(0, 0, LEVER_T / 2) * Cylinder(PAD_R, LEVER_T)
    return r


def root_holes(part):
    for k in range(4):
        a = math.radians(45 + 90 * k)
        part -= Pos(D.BCD / 2 * math.cos(a), D.BCD / 2 * math.sin(a), LEVER_T / 2) * \
            Cylinder(D.PAD_HOLE / 2, LEVER_T + 2)
    part -= Pos(0, 0, LEVER_T / 2) * Cylinder(D.HORN_CENTER_RELIEF_D / 2, LEVER_T + 2)
    return part


def bar(length):
    end = length - LEVER_H / 2
    b = Pos(end / 2, 0, LEVER_T / 2) * Box(end, LEVER_H, LEVER_T)
    b += Pos(end, 0, LEVER_T / 2) * Cylinder(LEVER_H / 2, LEVER_T)
    return b


def lever_stiff():
    p = lever_root() + bar(LEVER_L)
    for x in NOTCHES:
        for s in (1, -1):
            e = s * LEVER_H / 2
            pts = [(x - NOTCH_DEPTH, e + s * 0.01), (x + NOTCH_DEPTH, e + s * 0.01),
                   (x, e - s * NOTCH_DEPTH)]
            if s > 0:
                # counter-clockwise on BOTH edges: extrude() follows the face
                # normal, and a clockwise triangle extrudes -z, away from the
                # lever -- which silently left the +y edge un-notched
                pts.reverse()
            tri = Polygon(*pts, align=None)
            p -= Pos(0, 0, -1) * extrude(tri, amount=LEVER_T + 2)
    return root_holes(p)


def lever_hold():
    ro = CUP_ID / 2 + CUP_WALL
    p = lever_root() + bar(ARM)
    p += Pos(ARM, 0, (CUP_FLOOR + CUP_IN_H) / 2) * Cylinder(ro, CUP_FLOOR + CUP_IN_H)
    p -= Pos(ARM, 0, CUP_FLOOR + CUP_IN_H / 2 + 0.5) * Cylinder(CUP_ID / 2, CUP_IN_H + 1)
    return root_holes(p)


def lid():
    ro = CUP_ID / 2 + CUP_WALL
    p = Pos(0, 0, LID_T / 2) * Cylinder(ro, LID_T)
    p += Pos(0, 0, LID_T + LID_SPIGOT_H / 2) * Cylinder(CUP_ID / 2 - LID_SPIGOT_CLR, LID_SPIGOT_H)
    return p


def stop_pin():
    """Print lying down (layers along the pin: strong in bending)."""
    shaft = PIN_REACH_X - SLAB_X0
    p = Pos(0, 0, PIN_HEAD_T / 2) * Cylinder(PIN_HEAD_D / 2, PIN_HEAD_T)
    p += Pos(0, 0, PIN_HEAD_T + shaft / 2) * Cylinder(PIN_D / 2, shaft)
    # a 1 mm flat along head AND shaft so it lies on the bed (-y goes down)
    p -= Pos(0, -PIN_D / 2 + 1.0 - 5, 50) * Box(40, 10, 200)
    return p


# ----------------------------------------------------------- placement (world)
def lever_in_world(part, angle_deg=0.0):
    """Lever frame -> world: lever +x along world +y (away from the table),
    rotated angle_deg about the output axis (+ = tip DOWN), horn face at HORN_X."""
    pl = Plane(origin=(HORN_X, AXIS_Y, AXIS_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return pl.location * (Rot(0, 0, -angle_deg) * part)


def stop_pin_in_world(part):
    py, pz = pin_centre()
    pl = Plane(origin=(SLAB_X0 - PIN_HEAD_T, py, pz), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return pl.location * part


def servo_frame():
    """Servo frame -> world, shared by the servo mock and its seat."""
    return Plane(origin=(MID_X, AXIS_Y, AXIS_Z), x_dir=(0, 0, 1), z_dir=(0, 1, 0)).location


def servo_in_world():
    import check_assembly as CA
    return servo_frame() * CA.servo_mock()


def table():
    return box(-200, 200, -300, 0, -30, 0)


def to_bed(part, rot):
    p = rot * part
    bb = p.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * p


def vol(a, b):
    try:
        return (a & b).volume
    except Exception:
        return 0.0


def main():
    br, ls, lh, li, pin = bracket(), lever_stiff(), lever_hold(), lid(), stop_pin()
    sv, tb = servo_in_world(), table()
    pin_w = stop_pin_in_world(pin)
    ok = True

    def check(name, v, want_zero=True):
        nonlocal ok
        good = (v < 1e-3) if want_zero else (v > 1e-3)
        ok &= good
        print(f"  {'ok ' if good else 'BAD'} {name}: {v:.3f} mm3")

    print("fit checks (mm3 of overlap; the servo is the repo's conservative mock):")
    check("bracket vs servo", vol(br, sv))
    check("bracket BEARS on the idler case face (servo 0.2 toward it)",
          vol(br, Pos(-0.2, 0, 0) * sv), want_zero=False)
    for d in (0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0):   # its way on: straight onto the face
        check(f"servo {d:g} mm off the face along the axis (seats straight on)",
              vol(br, Pos(d, 0, 0) * sv))
    # the seat IS leg_link's idler grip plate over that plate's footprint
    # below the connector window (servo frame)
    reg = PT.box(*SEAT_X, SEAT_OUT, SEAT_IN, D.GRIP_BOT, WIN_Z[0] - 0.01)
    ref, ours = PT.leg_link() & reg, (idler_seat() - access_bores()) & reg
    check("seat vs parts.leg_link's idler grip plate (symmetric difference)",
          ref.volume + ours.volume - 2 * (ref & ours).volume)
    check("bracket vs table", vol(br, tb))
    check("servo vs table", vol(sv, tb))
    for name, lev in (("stiff lever", ls), ("hold lever", lh)):
        for ang, what in ((0, "horizontal"), (90, "hanging down")):
            w = lever_in_world(lev, ang)
            check(f"{name} {what} vs servo", vol(w, sv))
            check(f"{name} {what} vs bracket", vol(w, br))
            check(f"{name} {what} vs table", vol(w, tb))
        check(f"{name} horizontal vs stop pin", vol(lever_in_world(lev, 0), pin_w))
        check(f"{name} 5 deg down vs stop pin", vol(lever_in_world(lev, 5), pin_w))
        check(f"{name} 16 deg down HITS the stop pin",
              vol(lever_in_world(lev, PIN_DROP_DEG + 1), pin_w), want_zero=False)
    check("stop pin vs servo", vol(pin_w, sv))
    for x in NOTCHES:                       # a notch on BOTH edges at every station
        for sg, edge in ((1, "top"), (-1, "bottom")):
            probe = Pos(x, sg * (LEVER_H / 2 - 1.0), LEVER_T / 2) * Box(1.0, 1.0, LEVER_T - 1.0)
            check(f"stiff lever {x:g} mm notch, {edge} edge: cut", vol(ls, probe))

    py, pz = pin_centre()
    print(f"axis at y {AXIS_Y}, z {AXIS_Z}; horn face x {HORN_X:.2f}; "
          f"stop pin at y {py:.2f}, z {pz:.2f}")
    cup_cm3 = math.pi * (CUP_ID / 2) ** 2 * CUP_IN_H / 1000
    print(f"cup volume {cup_cm3:.0f} cm3: salt ~{cup_cm3 * 1.2:.0f} g, "
          f"sand ~{cup_cm3 * 1.5:.0f} g, rice ~{cup_cm3 * 0.8:.0f} g")
    for name, p in (("bracket", br), ("lever_stiff", ls), ("lever_hold", lh),
                    ("lid", li), ("stop_pin", pin)):
        print(f"  {name}: {p.volume / 1000:.1f} cm3 solid (~{p.volume / 1000 * 1.24:.0f} g PLA at 100%)")

    out = os.path.join(HERE, "stl")
    beds = {
        # back face down: countersinks open at the bed, everything else rises
        "bracket": to_bed(br, Rot(0, -90, 0)),
        "lever_stiff": to_bed(ls, Rot(0, 0, 0)),
        "lever_hold": to_bed(lh, Rot(0, 0, 0)),
        "lid": to_bed(li, Rot(0, 0, 0)),
        "stop_pin": to_bed(pin, Rot(90, 0, 0)),
    }
    for name, p in beds.items():
        bb = p.bounding_box()
        foot = (p & box(-500, 500, -500, 500, -1, 0.3)).volume / 0.3
        print(f"  {name}: {foot:.0f} mm2 on the bed")
        path = os.path.join(out, f"plan_b_{name}.stl")
        export_stl(p, path)
        print(f"  wrote {os.path.relpath(path, ROOT)}  "
              f"({bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm on the bed)")
    figs = os.path.join(HERE, "figs")
    render(os.path.join(figs, "plan_b_rig_render.png"))
    diagram_svg(os.path.join(figs, "plan_b_rig_side.svg"))
    print("ALL CLEAR" if ok else "FIT CHECK FAILED")
    return 0 if ok else 1



# ------------------------------------------------------------------ figure
def render(path, px=900):
    """Two shaded views of the assembled rig from the real solids (MuJoCo
    offscreen, like render_part.py): the stiffness setup with the bottle bag,
    and the hold setup hanging down. Bottles, bag and string are primitives."""
    import tempfile
    import imageio.v2 as imageio
    import mujoco
    import numpy as np
    os.environ.setdefault("MUJOCO_GL", "egl")

    tmp = tempfile.mkdtemp(prefix="plan_b_render_")
    solids = {
        "table": (box(-120, 160, -160, 0, -25, 0), "0.72 0.58 0.42 1"),
        "bracket": (bracket(), "0.95 0.62 0.20 1"),
        "servo": (servo_in_world(), "0.16 0.17 0.19 1"),
        "pin": (stop_pin_in_world(stop_pin()), "0.80 0.18 0.15 1"),
        "lever_s": (lever_in_world(lever_stiff(), 0), "0.95 0.62 0.20 1"),
        "lever_h": (lever_in_world(lever_hold(), 90), "0.95 0.62 0.20 1"),
    }
    for k, (solid, _) in solids.items():
        export_stl(solid, os.path.join(tmp, k + ".stl"))

    lx = (HORN_X + LEVER_T / 2) / 1000        # lever mid-plane, m
    ny = (AXIS_Y + ARM) / 1000
    top = (AXIS_Z - LEVER_H / 2) / 1000
    bag_top, bottle_h, bottle_r = top - 0.055, 0.20, 0.032

    def scene(which):
        geoms = []
        for k, (_, rgba) in solids.items():
            if (k == "lever_s" and which != "stiff" or k == "lever_h" and which != "hold"
                    or k == "pin" and which == "hold"):      # pulled for the down hold
                continue
            geoms.append(f'<geom type="mesh" mesh="{k}" rgba="{rgba}"/>')
        if which == "stiff":
            geoms.append(f'<geom type="capsule" fromto="{lx} {ny} {top + 0.012} {lx} {ny} {bag_top}" '
                         f'size="0.0012" rgba="0.9 0.9 0.85 1"/>')
            for i, dx in enumerate((-0.068, 0.0, 0.068)):
                zc = bag_top - 0.03 - bottle_h / 2
                geoms.append(f'<geom type="cylinder" pos="{lx + dx} {ny} {zc}" '
                             f'size="{bottle_r} {bottle_h / 2}" rgba="0.45 0.70 0.95 0.85"/>')
                geoms.append(f'<geom type="cylinder" pos="{lx + dx} {ny} {zc + bottle_h / 2 + 0.012}" '
                             f'size="0.013 0.012" rgba="0.45 0.70 0.95 0.85"/>')
            geoms.append(f'<geom type="box" pos="{lx} {ny} {bag_top - 0.03 - bottle_h / 2 + 0.01}" '
                         f'size="0.106 0.036 {bottle_h / 2 + 0.03}" rgba="0.9 0.9 0.9 0.25"/>')
        meshes = "".join(f'<mesh name="{k}" file="{os.path.join(tmp, k)}.stl" '
                         f'scale="0.001 0.001 0.001"/>' for k in solids)
        return f"""<mujoco><asset>{meshes}</asset>
          <asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="16" height="16"/></asset>
          <visual><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
            <global offwidth="{px}" offheight="{px}"/><quality shadowsize="4096"/></visual>
          <worldbody><light pos="0.3 0.4 0.6" dir="-0.4 -0.5 -1"/>{''.join(geoms)}</worldbody>
        </mujoco>"""

    frames = []
    for which, lookat, dist in (("stiff", (0.03, 0.08, -0.10), 0.78),
                                ("hold", (0.03, 0.04, -0.03), 0.50)):
        model = mujoco.MjModel.from_xml_string(scene(which))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ren = mujoco.Renderer(model, px, px)
        cam = mujoco.MjvCamera()
        cam.lookat, cam.distance = np.array(lookat), dist
        cam.azimuth, cam.elevation = -148.0, -22.0
        ren.update_scene(data, cam)
        frames.append(ren.render())
    imageio.imwrite(path, np.concatenate(frames, axis=1))
    print(f"  rendered {os.path.relpath(path, ROOT)}")


def diagram_svg(path):
    """The annotated side view for the doc, drawn to scale (1.5 px/mm) from
    THIS file's constants, looking along the output axis from the lever side.
    The bag hangs below at true size; bottles line up along the axis, so one
    silhouette stands for 1-3."""
    S, X0, Y0 = 1.5, 60.0, 200.0

    def P(y, z):
        return X0 + S * (y + 90.0), Y0 - S * z

    def poly(pts, cls):
        return '<polygon class="%s" points="%s"/>' % (
            cls, " ".join("%.1f,%.1f" % P(*q) for q in pts))

    def rect(y0, y1, z0, z1, cls):
        return poly([(y0, z0), (y1, z0), (y1, z1), (y0, z1)], cls)

    def circ(y, z, r, cls):
        x, yy = P(y, z)
        return '<circle class="%s" cx="%.1f" cy="%.1f" r="%.1f"/>' % (cls, x, yy, S * r)

    def line(a, b, cls):
        (x1, y1), (x2, y2) = P(*a), P(*b)
        return '<line class="%s" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>' % (cls, x1, y1, x2, y2)

    def text(x, y, lines, cls="t"):
        """lines: strings; **word** marks bold. Line starts carry x/dy, the rest
        are sibling tspans that flow inline (no nesting: some renderers stack
        nested tspans)."""
        out = '<text class="%s" x="%.1f" y="%.1f">' % (cls, x, y)
        for i, ln in enumerate(lines):
            parts = re.split(r"(\*\*[^*]+\*\*)", ln)
            first = True
            for seg in parts:
                if not seg:
                    continue
                bold = seg.startswith("**")
                seg = seg.strip("*") if bold else seg
                attrs = ' class="b"' if bold else ""
                if first:
                    attrs += ' x="%.1f" dy="%s"' % (x, "0" if i == 0 else "15")
                    first = False
                out += "<tspan%s>%s</tspan>" % (attrs, seg)
        return out + "</text>"

    def leader(a_px, b_px):
        return '<path class="lead" d="M%.1f,%.1f L%.1f,%.1f"/><circle class="dot" cx="%.1f" cy="%.1f" r="2.2"/>' % (
            a_px[0], a_px[1], b_px[0], b_px[1], b_px[0], b_px[1])

    e = []
    # table (section) and C-clamp
    e.append(rect(-90, 0, -25, 0, "table"))
    c = -88.0                                  # 45 deg hatch: y - z = c, clipped
    while c < 25.0:
        zlo, zhi = max(-25.0, -90.0 - c), min(0.0, -c)
        if zlo < zhi:
            e.append(line((c + zlo, zlo), (c + zhi, zhi), "hat"))
        c += 7.0
    cx = -62.0
    e.append(poly([(cx - 9, BASE_T + 10), (cx + 9, BASE_T + 10), (cx + 9, BASE_T), (cx - 9, BASE_T)], "clamp"))
    e.append(poly([(cx - 9, -25), (cx + 9, -25), (cx + 9, -31), (cx - 9, -31)], "clamp"))
    e.append(poly([(cx - 30, BASE_T + 16), (cx + 9, BASE_T + 16), (cx + 9, BASE_T + 10), (cx - 24, BASE_T + 10),
                   (cx - 24, -37), (cx + 9, -37), (cx + 9, -43), (cx - 30, -43)], "clampf"))
    e.append(line((cx, -31), (cx, -43), "screw"))
    # bracket: plate (behind), base + gussets + cradle (in front of it)
    e.append(poly([(PLATE_Y[0], 0), (PLATE_Y[1], 0), (PLATE_Y[1], PLATE_Z1), (PLATE_Y[0], PLATE_Z1)], "print"))
    e.append(rect(TAB_Y0, PLATE_Y[1], TAB_Z0, 0, "print"))
    e.append(rect(BASE_Y0, 0, 0, BASE_T, "print2"))
    for y0 in (-GUSSET_T - 1.0, PLATE_Y[0]):
        e.append(rect(y0, y0 + GUSSET_T, BASE_T, PLATE_Z1 - 2, "print2"))
    half = D.SV_WID / 2 + CRADLE_CLR
    for z0, z1 in ((AXIS_Z + half, AXIS_Z + half + CRADLE_T), (AXIS_Z - half - CRADLE_T, AXIS_Z - half)):
        e.append(rect(*CRADLE_Y, z0, z1, "print2"))
    # servo, horn
    e.append(rect(AXIS_Y - D.SV_AXIS_FROM_REAR, AXIS_Y + D.SV_AXIS_FROM_OUT_END,
                  AXIS_Z - D.SV_WID / 2, AXIS_Z + D.SV_WID / 2, "servo"))
    sx, sy = P(AXIS_Y - D.SV_AXIS_FROM_REAR + 2, AXIS_Z - 1.5)
    e.append('<text class="tw" x="%.1f" y="%.1f">STS3215</text>' % (sx, sy))
    # ghost: the hold lever hanging down with its cup (end-on)
    hw = LEVER_H / 2
    e.append(rect(AXIS_Y - hw, AXIS_Y + hw, AXIS_Z - ARM, AXIS_Z, "ghost"))
    e.append(circ(AXIS_Y, AXIS_Z - ARM, CUP_ID / 2 + CUP_WALL, "ghost"))
    e.append(circ(AXIS_Y, AXIS_Z - ARM, CUP_ID / 2, "ghostin"))
    # stiffness lever, horizontal
    e.append(circ(AXIS_Y, AXIS_Z, PAD_R, "lever"))
    end = AXIS_Y + LEVER_L - hw
    e.append(rect(AXIS_Y, end, AXIS_Z - hw, AXIS_Z + hw, "lever"))
    e.append(circ(end, AXIS_Z, hw, "lever"))
    e.append(rect(AXIS_Y, end, AXIS_Z - hw + 0.4, AXIS_Z + hw - 0.4, "leverfill"))
    for x in NOTCHES:
        for sgn in (1, -1):
            ez = AXIS_Z + sgn * hw
            e.append(poly([(AXIS_Y + x - NOTCH_DEPTH, ez + sgn * 0.3), (AXIS_Y + x + NOTCH_DEPTH, ez + sgn * 0.3),
                           (AXIS_Y + x, ez - sgn * NOTCH_DEPTH)], "notch"))
    e.append(circ(AXIS_Y, AXIS_Z, D.SV_HORN_D / 2, "horn"))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        e.append(circ(AXIS_Y + D.BCD / 2 * math.cos(a), AXIS_Z + D.BCD / 2 * math.sin(a), 2.6, "screwhead"))
    e.append(circ(AXIS_Y, AXIS_Z, 1.2, "axis"))
    # stop pin + its 15 deg line
    py, pz = pin_centre()
    a = math.radians(-PIN_DROP_DEG)
    e.append(line((AXIS_Y, AXIS_Z), (AXIS_Y + 62 * math.cos(a), AXIS_Z + 62 * math.sin(a)), "thin"))
    ax_, ay_ = P(AXIS_Y, AXIS_Z)
    r = 56 * S
    e.append('<path class="thin" d="M%.1f,%.1f A%.1f,%.1f 0 0 1 %.1f,%.1f"/>' % (
        ax_ + r, ay_, r, r, ax_ + r * math.cos(a), ay_ - r * math.sin(a)))
    e.append('<text class="ts" x="%.1f" y="%.1f">15°</text>' % (ax_ + r + 6, ay_ + 22))
    e.append(circ(py, pz, PIN_D / 2, "pin"))
    # string + bag + bottle
    ny = AXIS_Y + ARM
    bag_top = AXIS_Z - hw - 55
    bx, by0 = P(ny, AXIS_Z + hw - NOTCH_DEPTH)
    e.append('<ellipse class="string" cx="%.1f" cy="%.1f" rx="4" ry="%.1f"/>' % (bx, P(ny, AXIS_Z)[1] + 1, S * hw + 1))
    e.append(line((ny, AXIS_Z - hw - 1), (ny, bag_top), "string"))
    bw, bh = 34.0, 200.0
    e.append(poly([(ny - 12, bag_top), (ny + 12, bag_top), (ny + bw + 6, bag_top - 22),
                   (ny + bw + 8, bag_top - 32 - bh), (ny - bw - 8, bag_top - 32 - bh), (ny - bw - 6, bag_top - 22)], "bag"))
    bt = bag_top - 30
    e.append(poly([(ny - 13, bt), (ny + 13, bt), (ny + 13, bt - 8), (ny + 30, bt - 25), (ny + 32, bt - bh),
                   (ny - 32, bt - bh), (ny - 30, bt - 25), (ny - 13, bt - 8)], "bottle"))
    # theta at the tip
    tx, ty = P(AXIS_Y + LEVER_L + 6, AXIS_Z)
    e.append('<path class="arrow" d="M%.1f,%.1f q9,14 0,28" marker-end="url(#ah)"/>' % (tx, ty - 12))
    # g
    gx, gy = P(-80, -120)
    e.append('<path class="arrow" d="M%.1f,%.1f v40" marker-end="url(#ah)"/><text class="t" x="%.1f" y="%.1f">g</text>' % (
        gx, gy, gx + 8, gy + 25))
    # dimensions
    dz = AXIS_Z + 35
    e.append(line((AXIS_Y, AXIS_Z + 13), (AXIS_Y, dz + 3), "ext"))
    e.append(line((ny, AXIS_Z + hw + 2), (ny, dz + 3), "ext"))
    x1, y1 = P(AXIS_Y, dz)
    x2, _ = P(ny, dz)
    e.append('<path class="dim" d="M%.1f,%.1f H%.1f" marker-start="url(#ahs)" marker-end="url(#ah)"/>' % (x1, y1, x2))
    e.append('<text class="td" x="%.1f" y="%.1f" text-anchor="middle">100.0 mm (3.94 in): the lever arm</text>' % ((x1 + x2) / 2, y1 - 5))
    dz2 = -34
    e.append(line((0, -25), (0, dz2 - 3), "ext"))
    e.append(line((AXIS_Y, AXIS_Z - hw - 2), (AXIS_Y, dz2 - 3), "extg"))
    x1, y1 = P(0, dz2)
    x2, _ = P(AXIS_Y, dz2)
    e.append('<path class="dim" d="M%.1f,%.1f H%.1f" marker-start="url(#ahs)" marker-end="url(#ah)"/>' % (x1, y1, x2))
    e.append('<text class="td" x="%.1f" y="%.1f" text-anchor="middle">50 mm</text>' % ((x1 + x2) / 2, y1 + 14))

    # labels: a bold title line, then plain lines (no inline mixing: not every
    # SVG renderer flows sibling tspans)
    R = 520
    L = []

    def label(x, y, title, lines, to):
        L.append(text(x, y, ["**%s**" % title] + lines))
        L.append(leader((x - 4, y - 4), to))

    label(R, 118, "Stiffness lever (printed)",
          ["130 mm long, 18 × 8 mm; V-notches on both", "edges at 50 / 75 / 100 mm"],
          P(AXIS_Y + 115, AXIS_Z + hw))
    label(R, 176, "θ = deflection under load",
          ["read by the servo's own encoder", "(lever bending does not count)"], (tx + 10, ty + 4))
    label(R, 234, "Stop pin (printed, Ø12)",
          ["15° below horizontal: catches the lever if", "torque drops with weights on.",
           "Pull it for the hanging-down hold."], P(py + 4, pz - 3))
    label(R, 308, "String loop", ["seated in the 100 mm notch"], P(ny + 1, bag_top + 12))
    label(R, 380, "Bag + 1 / 2 / 3 full 500 mL bottles",
          ["(16.9 fl oz), side by side along the axis:",
           "≈ 0.51 / 1.02 / 1.53 kg (1.12 / 2.25 / 3.37 lb)",
           "→ 0.5 / 1.0 / 1.5 N·m at 100 mm.",
           "Weigh bag + bottles; type what the scale says."], P(ny + 30, bag_top - 90))
    label(22, 22, "Bracket (printed)",
          ["base on the tabletop under a C-clamp; the plate holds the servo's",
           "idler face on a conformal seat, 4 × M2.5×8 flat-head; cradle walls",
           "hug the case and take the torque"], P(-30, PLATE_Z1 - 4))
    L[-1] = leader((150, 70), P(-30, PLATE_Z1 - 4))
    label(452, 60, "Horn", ["4 × M3×10 button head, Ø14 circle"], P(AXIS_Y + 5, AXIS_Z + 6))
    label(22, 470, "Hold test, hanging down (dashed)",
          ["the hold lever, its cup packed full of table salt:",
           "≈ 0.25 kg (8.8 oz) rigid at 100 mm; stop pin out"], P(AXIS_Y - 14, AXIS_Z - ARM - 22))
    L[-1] = leader((160, 456), P(AXIS_Y - 14, AXIS_Z - ARM - 22))
    L.append(text(22, 290, ["C-clamp"]))
    L.append(leader((70, 286), P(cx - 27, -20)))
    L.append(text(22, 322, ["table edge", "(section)"]))
    L.append(leader((80, 318), P(-40, -12)))

    svg = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 830 650" width="830" height="650" font-family="Helvetica, Arial, sans-serif">
<defs>
 <marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#333"/></marker>
 <marker id="ahs" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="#333"/></marker>
</defs>
<style>
 .table{fill:#d9bf97;stroke:#6b5234;stroke-width:1.2}.hat{stroke:#8a6a44;stroke-width:1}
 .clamp{fill:#8d949c;stroke:#333}.clampf{fill:#5f666e;stroke:#333}.screw{stroke:#333;stroke-width:3}
 .print{fill:#fbd49b;stroke:#b86e12;stroke-width:1.3}.print2{fill:#f6b25a;stroke:#b86e12;stroke-width:1.3}
 .servo{fill:#2b2f36;stroke:#111}.horn{fill:#a9b1b9;stroke:#555}.screwhead{fill:#555}.axis{fill:#fff}
 .lever{fill:#f39a1e;stroke:#9a5b00;stroke-width:1.3}.leverfill{fill:#f39a1e;stroke:none}.notch{fill:#fff;stroke:#9a5b00;stroke-width:1}
 .ghost{fill:none;stroke:#b86e12;stroke-width:1.4;stroke-dasharray:5 4}.ghostin{fill:none;stroke:#b86e12;stroke-width:1;stroke-dasharray:2 4}
 .pin{fill:#d0392b;stroke:#7a1d14;stroke-width:1.2}
 .string{fill:none;stroke:#444;stroke-width:1.4}
 .bag{fill:#eef2f5;fill-opacity:.7;stroke:#8795a1;stroke-width:1.2}.bottle{fill:#8cc4f2;stroke:#3f7fb5;stroke-width:1.2}
 .thin{fill:none;stroke:#666;stroke-width:1;stroke-dasharray:3 3}
 .dim{stroke:#333;stroke-width:1;fill:none}.ext{stroke:#666;stroke-width:.8}.extg{stroke:#666;stroke-width:.8;stroke-dasharray:2 3}
 .arrow{fill:none;stroke:#333;stroke-width:1.4}
 .lead{fill:none;stroke:#888;stroke-width:.9}.dot{fill:#888}
 .t{font-size:12.5px;fill:#222}.b{font-weight:bold}.ts{font-size:11px;fill:#444}.td{font-size:12px;fill:#222}.tw{font-size:8px;fill:#fff}
</style>
<rect width="830" height="650" fill="#fff"/>
""" + "\n".join(e) + "\n" + "\n".join(L) + """
<text class="ts" x="18" y="636">Side view along the servo's output axis, from the lever side, to scale (1.5 px/mm). Generated by cad/plan_b_rig.py.</text>
</svg>
"""
    with open(path, "w") as f:
        f.write(svg)
    print(f"  drew {os.path.relpath(path, ROOT)}")


if __name__ == "__main__":
    sys.exit(main())
