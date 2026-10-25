"""gd_plate: the General Driver alone in the torso's front bay, on a plate
glued to the front inner wall (Tom, 2026-10-24).

WHY IT EXISTS
-------------
The Pi + General Driver HAT stack (torso_v8's mock_stack) does not fit the
bay: the cables do not fit and the General Driver does not sit on the Pi the
way the stack assumed. Waveshare's own STEP of the board
(docs/datasheets/general-driver/General_Driver_for_Robots_STEP.zip, 2023-01-13)
says why:

  - The board is 65.0 x 65.01 overall, not 65.01 x 56.01: an 8.99 x 52.51 tab
    carries both 2 x 20 headers past the 56.01 edge (the dimension drawing
    dimensions it too). Its 2 x 20 sockets hang 9.0 below the PCB.
  - Seen from its parts side with the bus edge (switch SW1, servo ports H5 and
    H6) up, its service edge (XH inlet H1, both USB-C) is on the LEFT. Parts
    forward and bus edge up therefore puts the service edge at -y, not the +y
    of torso_v8's mock, whose board is a mirror image.
  - H5 and H6 are top-entry: the servo leads plug in square to the board, so
    the room in front of the parts face is what the cables need.

This plate carries the General Driver only; where the Pi goes is the next
step (it keeps its place in the plant and the torso's mocks until then).

THE MOUNT
---------
The Pi's four bosses on the front inner wall are left alone: as printed they
are deformed, so nothing here touches them or trusts their shape. Each is
kept out as its nominal teardrop grown BOSS_SLOP sideways and BOSS_FACE_SLOP
at its face (boss_keepouts()).

    board    parts face forward (+x), bus edge DOWN, tab UP, service edge at
             +y. The bay is 66.5 tall and the board 65, so its height is fixed:
             the bus edge on the torso's bottom plane, the tab 1.5 under the
             deck. It sits SHIFT_Y toward +y off the bosses' grid, so its four
             holes (49 x 58) clear the bosses' keep-outs; the USB-C plugs reach
             through the +y skin's window. Its sockets pass in front of the +y
             upper boss, SOCKET_AIR clear of its keep-out.
    plate    PLATE_T thick, flat on the wall's front face between the bosses
             (a cut-out round each +y boss; the -y pair are off its edge), and
             GLUED there. Two lightening windows; the wall's top windows (lead
             passes) stay open above it.
    keys     two, through square holes in the plate into the wall's two lower
             windows (y -8..8 and 20..32, z -52..-38): they locate the plate
             while the glue cures and are glued with it. The window's lower
             edge printed over support (the torso prints deck down), so the
             keys keep KEY_CLR_BOT off it and KEY_CLR off its sides and top;
             in the 16 mm window a key locates in z only. Separate prints: a
             tab on the plate's glue face would hold the whole plate 2 mm off
             the bed.
    spacers  four columns from the wall face to the board's underside, at the
             board's holes: O6.2 at the upper pair (beside the sockets' ends),
             O7 at the lower. Each takes an M3 self-tapper in an M3_TAP pilot
             PILOT_D deep.
    screws   4 x M3 x 8 pan-head self-tappers through the board into the
             spacers (6.4 of thread). The board's holes are O3.0 in the STEP,
             so an M3 passes them snugly.
    print    the plate back (glue) face down, the spacers rising; the keys on
             their back ends. No supports.

Room left: 18.8 mm in front of the parts face to the front cover, for a servo
plug SERVO_PLUG_H tall and BEND_ROOM for its lead's bend.

NOT MEASURED (each sets a clearance below; measure before printing):
    GD_PCB_T            the board's thickness (the STEP's PCB is 0.41)
    SOCKET              the sockets' depth under the board (the STEP's pins
                        reach 9.0; a stock 2 x 20 socket body is 8.5)
    SERVO_PLUG_H        a mated servo lead in H5 (header 5.9 from the STEP +
                        the 5264 housing's whole 8.0 depth: an upper bound)
    H1_BX / H1_BY       the XH inlet, which the STEP leaves out: photo-scaled
                        (dimensions BOARD_GD_U_INLET, +-1.5)
    BOSS_SLOP           how far the printed bosses stray from their CAD

THE IMU: the board turns 180 deg against the prototype's mount, so set
`imu mount 0.5 -0.5 -0.5 0.5` (sensor -> body: chip +X to +y, +Y down, +Z
aft, derived from the prototype's measured mount in docs/firmware-design.md),
check it with the tilt test there, then `imu bias` in the standing pose.

    .venv/bin/python cad/v6/gd_plate.py     # STLs (cad/v6/stl), STEPs + renders (build/v6), checks
"""
from __future__ import annotations

import functools
import os
import sys
import tempfile
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))

os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import (Compound, GeomType, Location, Plane, Pos, export_step,  # noqa: E402
                       export_stl, extrude, import_step)
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import paths  # noqa: E402
import torso_v8 as T  # noqa: E402

box, cyl_x = P.box, P.cyl_x

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(paths.BUILD, "v6", "step")
OUT_REN = os.path.join(paths.BUILD, "v6", "renders")

PRINT_ORIENT = {"gd_plate": T.RY_XUP,         # back (glue) face on the bed, the spacers rise
                "gd_key": T.RY_XUP}           # its back end on the bed

# ----------------------------------------------------------------------------
# the board, from Waveshare's STEP
# ----------------------------------------------------------------------------
VENDOR_ZIP = os.path.join(paths.ROOT, "docs", "datasheets", "general-driver",
                          "General_Driver_for_Robots_STEP.zip")
VENDOR_MEMBER = "General Driver for Robots STEP.step"
# STEP frame: bx from the bus edge (0) to the tab's edge (65.0), by from the
# service edge (0) to the motor-header edge (65.01), bz off the parts face.
STEP_PCB_T = 0.41                              # the model's PCB, z -0.41..0
STEP_HOLES = ((3.5, 3.5), (52.5, 3.5), (3.5, 61.5), (52.5, 61.5))   # (bx, by), O3.0
GD_PCB_T = D.BR_PCB_T                          # 1.63, NOT measured on this board
SOCKETS = ("P2", "P4")                         # the STEP draws their pins only
SOCKET = (50.8, 5.08, 9.0)                     # 2 x 20 at 2.54: length, width, depth under the PCB
H1_BX = D.BOARD_GD_U_INLET                     # (9.6, 19.6) off the bus edge, photo-scaled
H1_BY = (0.0, 7.0)                             # in from the service edge (photo)
H1_H = 6.1                                     # JST XH side entry, mated (datasheet p.2)
SERVO_PORTS = ("H5", "H6")                     # "D V G", top entry
SERVO_PLUG_H = 5.9 + 8.0                       # header (STEP) + the 5264 housing's depth
BEND_ROOM = 4.0                                # a 3-wire lead's 90 deg bend, plug top .. cover

# ----------------------------------------------------------------------------
# the torso's bosses (deformed as printed: keep-outs, never fits) and windows
# ----------------------------------------------------------------------------
WALL_FACE_X = V.FWD_WALL_X[1]                  # +15.81 the front inner wall's front face
BOSS_FACE_X = V.PI_PCB_X0                      # +19.81 the bosses' faces (CAD)
BOSS_R = 3.5                                   # torso_v8._board_bosses
BOSS_YZ = [(sy, sz) for sy in V.PI_SCREW_Y for sz in V.PI_SCREW_Z]
BOSS_ROOF = BOSS_R / np.cos(np.radians(D.BF_BOSS_ROOF_DEG))   # 4.95: the 45 deg roof's apex above the axis
BOSS_SLOP = 1.5                                # keep-out past the CAD teardrop, sideways, up and down
BOSS_FACE_SLOP = 1.0                           # ... and past the CAD face
BOSS_CUT_AIR = 0.5                             # the plate's cut-outs past the keep-out
LOWER_WINDOWS = ((-8.0, 8.0), (20.0, 32.0))    # torso_v8._windows: the front wall, below the battery floor
LOWER_WIN_Z = (-52.0, V.CELL_TOP_Z - 2.0)      # -52 .. -38; the -52 edge printed over support

# ----------------------------------------------------------------------------
# the mount
# ----------------------------------------------------------------------------
PLATE_T = 2.5
PLATE_X1 = WALL_FACE_X + PLATE_T               # +18.31 its front face
SPACER_R = {"upper": 3.1, "lower": 3.5}        # the upper pair beside P2's ends (3.43 off the holes' centres)
SHIFT_Y = BOSS_R + BOSS_SLOP + BOSS_CUT_AIR + SPACER_R["lower"]   # 9.0: the board's holes off the bosses' grid
SOCKET_AIR = 0.5
GD_X0 = BOSS_FACE_X + BOSS_FACE_SLOP + SOCKET_AIR + 9.0   # +30.31 the board's underside (9.0: SOCKET's depth)
GD_X1 = GD_X0 + GD_PCB_T                       # +31.94 its parts face
GD_Z0 = V.PI_SCREW_Z[0] - STEP_HOLES[0][0]     # -71.495 the bus edge (bx = 0)
GD_Y0 = V.PI_SCREW_Y[1] + STEP_HOLES[0][1] + SHIFT_Y   # +41.5 the service edge (by = 0)
# torso (x, y, z) = R . (bx, by, bz) + t: bx up, by toward -y, bz forward
R_BOARD = ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
assert abs(np.linalg.det(np.array(R_BOARD)) - 1.0) < 1e-12, "the board's placement must not mirror it"
HOLES = [(GD_Y0 - by, GD_Z0 + bx, "upper" if bx > 30 else "lower") for bx, by in STEP_HOLES]
GD_TOP_Z = GD_Z0 + 65.004                      # -6.49 the tab's edge
assert GD_TOP_Z <= -D.DECK_T - 1.0, "the board's tab must stay under the deck"
assert GD_Z0 >= V.TORSO_BOT_Z - 0.01, "the board must not hang below the torso"
assert GD_X0 - SOCKET[2] >= BOSS_FACE_X + BOSS_FACE_SLOP + 0.5, "the sockets must stay off the bosses' keep-outs"

PLATE_Y = (min(h[0] for h in HOLES) - SPACER_R["lower"] - 0.1,
           max(h[0] for h in HOLES) + SPACER_R["lower"] + 0.1)   # -23.6 .. +41.6
PLATE_Z = (V.TORSO_BOT_Z + 0.3, -16.5)         # the wall's top windows (z -16 .. -6) stay open
LIGHTEN = ((-12.0, 19.0, -32.0, -22.0), (-12.0, 19.0, -66.0, -57.0))   # y0, y1, z0, z1
M3_TAP = round(3.0 * D.M25_TAP / 2.5, 1)       # 2.6: the M2.5 pilots' 0.88 x d, for an M3 self-tapper
PILOT_D = 10.0
SCREW_L = 8.0                                  # M3 x 8 pan head: 6.37 into the spacer
SCREW_HEAD = (5.6, 2.4)                        # ISO 7045 M3 pan head: dk, k

KEY_CLR = 0.3                                  # key vs the window's sides and top
KEY_CLR_BOT = 0.8                              # ... and its supported lower edge
KEY_IN = 2.0                                   # into the 2.6 wall
KEY_W = min(b - a for a, b in LOWER_WINDOWS) - 2 * KEY_CLR   # 11.4: one key fits both windows
KEY_Z = (LOWER_WIN_Z[0] + KEY_CLR_BOT, LOWER_WIN_Z[1] - KEY_CLR)
KEY_X = (WALL_FACE_X - KEY_IN, PLATE_X1)       # flush with the plate's front face
KEY_HOLE_CLR = 0.15                            # the plate's square holes round the keys (glued)
MIN_AIR = 0.3                                  # plate vs anything on the board's underside


def _M(x_at_bz0):
    """STEP frame -> torso, the STEP's z = 0 plane at x = x_at_bz0."""
    col = lambda j: tuple(R_BOARD[i][j] for i in range(3))
    return Location(Plane(origin=(x_at_bz0, GD_Y0, GD_Z0), x_dir=col(0), z_dir=col(2)))


@functools.lru_cache(maxsize=1)
def vendor_model():
    with zipfile.ZipFile(VENDOR_ZIP) as z, tempfile.TemporaryDirectory() as tmp:
        return import_step(z.extract(VENDOR_MEMBER, tmp))


@functools.lru_cache(maxsize=1)
def vendor_parts():
    """{ref: (bx0, bx1, by0, by1, bz0, bz1)}, each part's box in the STEP, the PCB left out."""
    out = {}
    for k in vendor_model().children:
        if str(k.label) != "Board":
            b = k.bounding_box(optimal=False)
            out[str(k.label)] = (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)
    return out


def _yz(bx0, bx1, by0, by1):
    return GD_Y0 - by1, GD_Y0 - by0, GD_Z0 + bx0, GD_Z0 + bx1


def gd_envelopes(plugs=True):
    """[(name, (x0, x1, y0, y1, z0, z1), side)]: every part's box, top-side
    parts off the parts face and under-side parts off the underside (the PCB
    taken at GD_PCB_T, not the STEP's 0.41); the sockets' bodies; the XH inlet
    the STEP leaves out; and with plugs, the mated servo, USB-C and XH plugs.
    side: "top", "under" or "plug"."""
    vp = vendor_parts()
    env = []
    for ref, (bx0, bx1, by0, by1, bz0, bz1) in vp.items():
        yz = _yz(bx0, bx1, by0, by1)
        if bz1 > 0.05:
            env.append((ref, (GD_X1, GD_X1 + bz1, *yz), "top"))
        if bz0 < -STEP_PCB_T - 0.05 and ref not in SOCKETS:
            env.append((f"{ref} (under)", (GD_X0 - (-STEP_PCB_T - bz0), GD_X0, *yz), "under"))
    for ref in SOCKETS:
        bx0, bx1, by0, by1, *_ = vp[ref]
        cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
        ln, wd, dp = SOCKET
        env.append((f"{ref} socket", (GD_X0 - dp, GD_X0, *_yz(cx - wd / 2, cx + wd / 2, cy - ln / 2, cy + ln / 2)), "under"))
    env.append(("H1 XH inlet", (GD_X1, GD_X1 + H1_H, *_yz(*H1_BX, *H1_BY)), "top"))
    if plugs:
        for ref in SERVO_PORTS:
            bx0, bx1, by0, by1, *_ = vp[ref]
            env.append((f"{ref} servo plug", (GD_X1, GD_X1 + SERVO_PLUG_H, *_yz(bx0, bx1, by0, by1)), "plug"))
        along, thick, out = D.USBC_PLUG
        for ref in ("Type_C1", "Type_C2"):
            bx0, bx1, by0, by1, bz0, bz1 = vp[ref]
            bc, xc = (bx0 + bx1) / 2, GD_X1 + (bz0 + bz1) / 2
            env.append((f"{ref} USB-C plug", (xc - thick / 2, xc + thick / 2, GD_Y0 - by0, GD_Y0 - by0 + out,
                                              GD_Z0 + bc - along / 2, GD_Z0 + bc + along / 2), "plug"))
        along, thick, out = D.XH2_PLUG
        bc = sum(H1_BX) / 2
        env.append(("H1 XH plug", (GD_X1, GD_X1 + thick, GD_Y0 - H1_BY[0], GD_Y0 - H1_BY[0] + out,
                                   GD_Z0 + bc - along / 2, GD_Z0 + bc + along / 2), "plug"))
    return env


def _box(e, grow=0.0):
    x0, x1, y0, y1, z0, z1 = e
    return box(x0 - grow, x1 + grow, y0 - grow, y1 + grow, z0 - grow, z1 + grow)


def _solids(env, sides):
    return Compound([_box(e) for _, e, s in env if s in sides])


@functools.lru_cache(maxsize=1)
def gd_pcb():
    """The board's PCB: the STEP's outline (tab and holes), GD_PCB_T thick."""
    board = next(k for k in vendor_model().children if str(k.label) == "Board")
    top = max((f for f in board.faces() if f.geom_type == GeomType.PLANE and abs(f.normal_at().Z) > 0.99),
              key=lambda f: f.center().Z)
    return extrude(top.moved(_M(GD_X0)), amount=GD_PCB_T, dir=(1, 0, 0))


def gd_render_solids(chunk=25):
    """The vendor model's parts placed for pictures (top-side parts on the
    parts face, under-side parts on the underside), plus the sockets' bodies
    the STEP leaves out, in compounds small enough for one mesh each."""
    tops, unders = [], []
    for k in vendor_model().children:
        if str(k.label) == "Board":
            continue
        for s in k.solids():
            (unders if s.bounding_box(optimal=False).center().Z < -STEP_PCB_T / 2 else tops).append(s)
    mt, mu = _M(GD_X1), _M(GD_X0 + STEP_PCB_T)
    placed = [s.moved(mt) for s in tops] + [s.moved(mu) for s in unders]
    placed += [_box(e) for n, e, _ in gd_envelopes(plugs=False) if n.endswith("socket") or n.startswith("H1")]
    return [Compound(placed[i:i + chunk]) for i in range(0, len(placed), chunk)]


# ============================================================================
# THE PLATE AND ITS KEYS
# ============================================================================
def boss_keepouts():
    """Each boss as it may have printed: the CAD teardrop's box grown
    BOSS_SLOP in y and z and BOSS_FACE_SLOP at the face."""
    out = None
    for sy, sz in BOSS_YZ:
        k = box(WALL_FACE_X - 0.5, BOSS_FACE_X + BOSS_FACE_SLOP, sy - BOSS_R - BOSS_SLOP, sy + BOSS_R + BOSS_SLOP,
                sz - BOSS_R - BOSS_SLOP, sz + BOSS_ROOF + BOSS_SLOP)
        out = k if out is None else out + k
    return out


def _key_box(yc, grow=0.0):
    return box(KEY_X[0] - grow, KEY_X[1] + grow, yc - KEY_W / 2 - grow, yc + KEY_W / 2 + grow,
               KEY_Z[0] - grow, KEY_Z[1] + grow)


KEY_YC = tuple((a + b) / 2 for a, b in LOWER_WINDOWS)


def gd_keys():
    """Both keys in place."""
    return _key_box(KEY_YC[0]) + _key_box(KEY_YC[1])


def gd_key():
    """One key (the print part), in the 20..32 window."""
    return _key_box(KEY_YC[1])


def gd_plate():
    """The plate on the wall: cut-outs round the bosses it spans, the keys'
    square holes, two lightening windows; the four spacers rising from the
    wall face, each with its blind M3 pilot."""
    x0, x1 = WALL_FACE_X, PLATE_X1
    p = box(x0, x1, *PLATE_Y, *PLATE_Z)
    for sy, sz in BOSS_YZ:
        g = BOSS_SLOP + BOSS_CUT_AIR
        p -= box(x0 - 1, x1 + 1, sy - BOSS_R - g, sy + BOSS_R + g, sz - BOSS_R - g, sz + BOSS_ROOF + g)
    for yc in KEY_YC:
        p -= _key_box(yc, KEY_HOLE_CLR) + box(x0 - 1, x1 + 1, yc - KEY_W / 2 - KEY_HOLE_CLR,
                                              yc + KEY_W / 2 + KEY_HOLE_CLR, KEY_Z[0] - KEY_HOLE_CLR,
                                              KEY_Z[1] + KEY_HOLE_CLR)
    for ya, yb, za, zb in LIGHTEN:
        p -= box(x0 - 1, x1 + 1, ya, yb, za, zb)
    for y, z, row in HOLES:
        p += cyl_x(SPACER_R[row], x0, GD_X0, y, z)
    for y, z, _ in HOLES:
        p -= cyl_x(M3_TAP / 2, GD_X0 - PILOT_D, GD_X0 + 1, y, z)
    return p


def SCREWS():
    """T.SCREWS()'s convention: head at the parts face, axis head -> tip."""
    return [dict(name=f"gd_{y:+.0f}_{z:+.0f}", part="gd_plate",
                 kind=f"M3x{SCREW_L:.0f} self-tap, pan head: the board into the spacer's {M3_TAP} pilot",
                 axis=(-1, 0, 0), pos=(GD_X1, y, z), length=SCREW_L,
                 clamp=GD_PCB_T, pilot=PILOT_D) for y, z, _ in HOLES]


def screw_mocks():
    """The four screws as cylinders (shank + pan head), for pictures and the head check."""
    out = None
    for sc in SCREWS():
        _, y, z = sc["pos"]
        s = cyl_x(1.5, GD_X1 - sc["length"], GD_X1, y, z) + cyl_x(SCREW_HEAD[0] / 2, GD_X1, GD_X1 + SCREW_HEAD[1], y, z)
        out = s if out is None else out + s
    return out


# ============================================================================
# checks
# ============================================================================
def _vol(a, b):
    try:
        return (a & b).volume
    except Exception:  # noqa: BLE001
        return 0.0


def _line(ok, what, detail):
    print(f"  {'PASS' if ok else 'FAIL'}  {what}: {detail}")
    return ok


def glue_area(plate, torso):
    """mm2 of the plate's back face on solid wall."""
    t = 0.1
    a = plate & box(WALL_FACE_X, WALL_FACE_X + t, -100, 100, -100, 10)
    b = Pos(t, 0, 0) * (torso & box(WALL_FACE_X - t, WALL_FACE_X, -100, 100, -100, 10))
    return _vol(a, b) / t


def check_fits(plate, keys, torso, cover, pelvis):
    """The wall: the windows the keys go into are open in the torso; plate and
    keys clear of torso, pelvis and the bosses' keep-outs; the glue area. The
    board, seated with the plugs in: the plate MIN_AIR clear of every part
    under it; the board, its parts and the mated plugs clear of the torso, the
    front cover, the bearing housing, the bosses' keep-outs and the plate.
    Installed: plate, keys and board in from the front, cover off. The USB-C
    plugs leave straight out through the +y skin and the XH plug's wires
    through its window. The servo plugs leave BEND_ROOM under the cover."""
    import yaw_bearing_housing as YBH
    env = gd_envelopes(plugs=True)
    pcb = gd_pcb()
    keep = boss_keepouts()
    ok = True
    for (a, b) in LOWER_WINDOWS:
        probe = box(V.FWD_WALL_X[0] + 0.1, WALL_FACE_X - 0.1, a + 0.1, b - 0.1, LOWER_WIN_Z[0] + 0.1, LOWER_WIN_Z[1] - 0.1)
        v = _vol(probe, torso)
        ok &= _line(v < 0.01, f"the wall's window y {a:+.0f}..{b:+.0f} is open in the torso", f"{v:.2f} mm3 solid")
    v = _vol(plate, torso)
    ok &= _line(v < 0.01, "plate vs torso", f"{v:.2f} mm3 (it lies on the wall's front face)")
    v = _vol(keys, torso + pelvis)
    ok &= _line(v < 0.01, "keys vs torso and pelvis", f"{v:.2f} mm3 ({KEY_CLR} off the windows' sides and tops, "
                f"{KEY_CLR_BOT} off their supported lower edge, {KEY_X[0] - (V.FWD_WALL_X[0] - V.TORSO_FIT):.1f} short of the pelvis)")
    v = _vol(plate + keys, keep)
    ok &= _line(v < 0.01, f"plate + keys vs the bosses' keep-outs (CAD + {BOSS_SLOP} / face + {BOSS_FACE_SLOP})", f"{v:.2f} mm3")
    g = glue_area(plate, torso)
    ok &= _line(g >= 800.0, "glue area: the plate's back face on solid wall", f"{g:.0f} mm2")
    unders = [(n, e) for n, e, s in env if s == "under"]
    hits = [(n, _vol(plate, _box(e, MIN_AIR))) for n, e in unders]
    hits = [(n, h) for n, h in hits if h > 0.0]
    ok &= _line(not hits, f"plate vs the board's under-side parts (+{MIN_AIR} air)",
                "clear" if not hits else ", ".join(f"{n} {h:.2f} mm3" for n, h in hits))
    sock = [e for n, e, _ in env if n == "P2 socket"][0]
    up = [(y, z) for y, z, row in HOLES if row == "upper"]
    gap_y = min(min(abs(y - sock[2]), abs(y - sock[3])) for y, _ in up) - SPACER_R["upper"]
    ok &= _line(gap_y >= MIN_AIR, "upper spacers vs the P2 socket's ends", f"{gap_y:.2f} mm")
    walls = {row: r - M3_TAP / 2 for row, r in SPACER_R.items()}
    ok &= _line(min(walls.values()) >= 1.5, f"spacer walls round the O{M3_TAP} M3 pilot",
                ", ".join(f"{row}: {w:.2f} mm" for row, w in walls.items()))
    fixed = torso + cover + YBH.yaw_bearing_housing()
    board = Compound([pcb] + [_box(e) for _, e, _ in env])
    v = _vol(board, fixed)
    ok &= _line(v < 0.01, "board + parts + mated plugs vs torso, front cover, bearing housing", f"{v:.2f} mm3")
    v = _vol(board, keep)
    ok &= _line(v < 0.01, "board + parts vs the bosses' keep-outs", f"{v:.2f} mm3 (the sockets pass the +y upper one "
                f"{sock[0] - BOSS_FACE_X - BOSS_FACE_SLOP:.2f} in front)")
    v = _vol(board, plate)
    ok &= _line(v < 0.01, "board + parts vs plate", f"{v:.2f} mm3 (the spacers bear on the underside)")
    v = _vol(screw_mocks(), _solids(gd_envelopes(plugs=False), ("top",)))
    ok &= _line(v < 0.01, f"M3 pan heads (O{SCREW_HEAD[0]}) vs the board's parts", f"{v:.2f} mm3")
    mover = Compound([plate, keys, pcb] + [_box(e) for _, e, s in env if s != "plug"])
    ok &= T._sweep(mover, torso, ((70.0, 0, 0), (0, 0, 0)), "plate + keys + board in from the front (cover off)", n=24)
    y_out = V.HOUSING_HW + 5.0
    for n, e, s in env:
        if s == "plug" and "USB-C" in n:
            x0, x1, y0, y1, z0, z1 = e
            v = _vol(box(x0, x1, y0, y_out, z0, z1), torso)
            ok &= _line(v < 0.01, f"{n}: straight out through the +y skin", f"{v:.2f} mm3")
        if s == "plug" and "XH" in n:
            x0, x1, y0, y1, z0, z1 = e
            xc, zc_ = (x0 + x1) / 2, (z0 + z1) / 2
            v = _vol(box(xc - 1.5, xc + 1.5, y1, y_out, zc_ - 1.5, zc_ + 1.5), torso)
            ok &= _line(v < 0.01, f"{n}: its wires (O3) straight out through the +y skin", f"{v:.2f} mm3")
    room = V.GD_FRONT_X - (GD_X1 + SERVO_PLUG_H)
    ok &= _line(room >= BEND_ROOM, "servo plugs' tops to the front cover",
                f"{room:.2f} mm (>= {BEND_ROOM}); parts face {GD_X1:.2f}, cover {V.GD_FRONT_X:.2f}")
    deck = -D.DECK_T - GD_TOP_Z
    ok &= _line(deck >= 1.0, "board's tab under the deck", f"{deck:.2f} mm")
    return ok


def check_screws(torso, verbose=True):
    """Bite in the spacer's pilot (T.MIN_BITE .. the pilot), and a straight
    driver from each head out along +x, the cover off, missing the torso and
    the board's top-side parts (D.ACCESS_D, else T.SLIM_DRIVER_D)."""
    ok = True
    tops = _solids(gd_envelopes(plugs=False), ("top",))
    solid = torso + tops
    for sc in SCREWS():
        bite = sc["length"] - sc["clamp"]
        good = T.MIN_BITE <= bite <= sc["pilot"] - 0.5
        hit = T._driver_hit(sc, solid, D.ACCESS_D)
        slim = T._driver_hit(sc, solid, T.SLIM_DRIVER_D)
        reach = hit < 5.0 or slim < 5.0
        ok &= good and reach
        if verbose:
            print(f"  {'PASS' if good and reach else 'FAIL'}  {sc['name']:12s} {sc['kind']}: bite {bite:.2f} of "
                  f"{sc['pilot']:.2f} (>= {T.MIN_BITE}); driver {D.ACCESS_D:.0f} mm {hit:.1f} mm3, "
                  f"{T.SLIM_DRIVER_D:.0f} mm {slim:.1f} mm3")
    return ok


def audit(name, stl_path):
    import check_printability as CP
    CP.STL = os.path.dirname(stl_path)
    CP.ORIENT[name] = (np.array(PRINT_ORIENT[name]), f"{name}, no supports")
    CP.PRINT_STL[name] = os.path.basename(stl_path)
    CP.SUPPORTED.pop(name, None)
    return list(CP.audit(name))


# ============================================================================
# pictures
# ============================================================================
C_PLATE, C_SCREW, C_KEY = (0.90, 0.55, 0.20), (0.55, 0.55, 0.60), (0.85, 0.25, 0.25)


def in_print_orient(solid, name):
    saved = dict(T.PRINT_ORIENT)
    T.PRINT_ORIENT.update(PRINT_ORIENT)
    try:
        return T.in_print_orient(solid, name)
    finally:
        T.PRINT_ORIENT.clear()
        T.PRINT_ORIENT.update(saved)


def render(plate, keys, torso, cover, out_dir):
    """gd_plate.png: the plate and a key as they print; plate and keys on the
    wall (cover off; the bosses show through the cut-outs, red: the keys in
    the wall's windows); the board on it with the plugs mated (light grey);
    the bay cut at y = +26 through the side key, seen from +y, cover on.
    gd_plate_exploded.png: torso, keys, plate, board and cover apart along x."""
    board = [(s, T.C_BOARD) for s in gd_render_solids()] + [(gd_pcb(), (0.12, 0.30, 0.18))]
    plugs = [(_box(e), (0.85, 0.85, 0.90)) for _, e, s in gd_envelopes() if s == "plug"]
    keep = box(8.0, 60.0, -200, KEY_YC[1], -80.0, 3.0)
    def cut(parts):
        out = []
        for s, c in parts:
            try:
                sol = (s & keep).solids()
            except AssertionError:          # nothing of it inside the cut
                continue
            if sol:
                out.append((Compound(sol), c))
        return out
    seated = [(torso, T.C_TORSO), (plate, C_PLATE), (keys, C_KEY), (screw_mocks(), C_SCREW)] + board
    shots = [([(in_print_orient(plate, "gd_plate"), C_PLATE), (Pos(45, 0, 0) * in_print_orient(gd_key(), "gd_key"), C_KEY)],
              135, -50, 2.0),
             ([(torso, T.C_TORSO), (plate, C_PLATE), (keys, C_KEY)], 160, -25, 1.5),
             (seated + plugs, 215, -25, 1.5),
             (cut(seated + plugs + [(cover, T.C_COVER)]), -90, 0, 2.1)]
    a = T._render_strip(shots, os.path.join(out_dir, "gd_plate.png"))
    dx = 40.0
    ex = ([(torso, T.C_TORSO), (Pos(dx, 0, 0) * keys, C_KEY), (Pos(2 * dx, 0, 0) * plate, C_PLATE),
           (Pos(4 * dx, 0, 0) * cover, T.C_COVER)] + [(Pos(3 * dx, 0, 0) * s, c) for s, c in board])
    b = T._render_strip([(ex, 150, -20, 1.5)], os.path.join(out_dir, "gd_plate_exploded.png"), px=900)
    return a, b


def mass_g(s):
    return s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    plate, keys = gd_plate(), gd_keys()
    torso, cover, pelvis = T.torso_v8(arm_mounts=True), T.front_cover_v8(), T.pelvis_v8()
    paths_ = {}
    for name, s in (("gd_plate", plate), ("gd_key", gd_key())):
        bb = s.bounding_box()
        paths_[name] = os.path.join(OUT_STL, f"{name}.stl")
        export_stl(s, paths_[name])
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))
        print(f"{name:9s} {len(s.solids())} solid(s)  {mass_g(s):.1f} g  "
              f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")
    print(f"built in {time.time() - t0:.0f}s; board: underside x {GD_X0:.2f}, parts face {GD_X1:.2f}, "
          f"z {GD_Z0:.2f}..{GD_TOP_Z:.2f}, y {GD_Y0 - 65.01:+.2f}..{GD_Y0:+.2f} (service edge), "
          f"holes {', '.join(f'({y:+.1f}, {z:+.1f})' for y, z, _ in HOLES)}")
    print("\n-- fits --")
    ok = check_fits(plate, keys, torso, cover, pelvis)
    print("\n-- screws --")
    ok &= check_screws(torso)
    print("\n-- printability --")
    for name, path in paths_.items():
        findings = audit(name, path)
        print(f"  {name}: {'PASS, no supports' if not findings else f'** {len(findings)} FINDING(S) **'}")
        for kind, a, desc in findings:
            print(f"      {kind:8s} {desc}")
        ok &= not findings
    print("\nwrote", *render(plate, keys, torso, cover, OUT_REN))
    print("ALL CHECKS PASS" if ok else "** SOME CHECKS FAIL **")
    sys.exit(0 if ok else 1)
