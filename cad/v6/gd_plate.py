"""gd_plate: the General Driver alone in the torso's front bay, on a plate on
the Pi's four bosses (Tom, 2026-10-24).

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

This first plate carries the General Driver only; where the Pi goes is the
next step (it keeps its place in the plant and the torso's mocks until then).

THE MOUNT
---------
    board    parts face forward (+x), bus edge DOWN, tab UP, service edge at
             +y, facing the +y skin's windows. The bay is 66.5 tall and the
             board 65, so the height is fixed: the bus edge is on the torso's
             bottom plane, the tab's top 1.5 under the deck. There the
             board's four holes (49 x 58) land on the Pi's four bosses
             (dimensions_v6 PI_SCREW_Y / PI_SCREW_Z) exactly.
    spacers  four tubes, SPACER_L long, from the bosses' faces to the board's
             underside. The upper sockets' ends come within 0.1 of the bosses
             sideways, so the sockets must stay in front of the bosses' faces:
             SPACER_L leaves them 1.0 clear.
    plate    PLATE_T thick on the bosses' faces: a rail through each pair of
             spacers and two rungs, open at the top behind the sockets. It
             holds the spacers in register while the screws go in.
    screws   4 x M2.5 x 16 pan-head self-tappers, from the parts face through
             the board and the spacer (clearance bore) into the boss's pilot
             (2.2 x 5.0); an x 18 bottoms out. Heads <= O6.9: a USB-C shell
             is 3.5 from a hole's centre.
             The screws hold everything: board, plate, torso. Glue (epoxy or
             CA on the four spacer feet) holds the plate on the bosses while
             the board is off, if wanted; the screws go in the same either way.
    print    flat, back face down, the spacers rising; no supports.

Room left: 19.3 mm in front of the parts face to the front cover, for a
servo plug SERVO_PLUG_H tall and BEND_ROOM for its lead's bend.

NOT MEASURED (each sets a clearance below; measure before printing):
    GD_PCB_T            the board's thickness (the STEP's PCB is 0.41)
    SOCKET              the sockets' depth under the board (the STEP's pins
                        reach 9.0; a stock 2 x 20 socket body is 8.5)
    SERVO_PLUG_H        a mated servo lead in H5 (header 5.9 from the STEP +
                        the 5264 housing's whole 8.0 depth: an upper bound)
    H1_BX / H1_BY       the XH inlet, which the STEP leaves out: photo-scaled
                        (dimensions BOARD_GD_U_INLET, +-1.5)

THE IMU: the board turns 180 deg against the prototype's mount, so set
`imu mount 0.5 -0.5 -0.5 0.5` (sensor -> body: chip +X to +y, +Y down, +Z
aft, derived from the prototype's measured mount in docs/firmware-design.md),
check it with the tilt test there, then `imu bias` in the standing pose.

    .venv/bin/python cad/v6/gd_plate.py     # STL (cad/v6/stl), STEP + renders (build/v6), checks
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

PRINT_ORIENT = {"gd_plate": T.RY_XUP}          # back face (-x) on the bed, the spacers rise

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
# the mount
# ----------------------------------------------------------------------------
BOSS_FACE_X = V.PI_PCB_X0                      # +19.81 the bosses' faces
BOSS_YZ = [(sy, sz) for sy in V.PI_SCREW_Y for sz in V.PI_SCREW_Z]
SPACER_L = 10.0
GD_X0 = BOSS_FACE_X + SPACER_L                 # +29.81 the board's underside
GD_X1 = GD_X0 + GD_PCB_T                       # +31.44 its parts face
GD_Z0 = V.PI_SCREW_Z[0] - STEP_HOLES[0][0]     # -71.495 the bus edge (bx = 0)
GD_Y0 = V.PI_SCREW_Y[1] + STEP_HOLES[0][1]     # +32.5 the service edge (by = 0)
# torso (x, y, z) = R . (bx, by, bz) + t: bx up, by toward -y, bz forward
R_BOARD = ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
assert abs(np.linalg.det(np.array(R_BOARD)) - 1.0) < 1e-12, "the board's placement must not mirror it"
for _bx, _by in STEP_HOLES:
    assert any(abs(GD_Y0 - _by - sy) < 0.01 and abs(GD_Z0 + _bx - sz) < 0.01 for sy, sz in BOSS_YZ), \
        "every board hole must land on a boss"
GD_TOP_Z = GD_Z0 + 65.004                      # -6.49 the tab's edge
assert GD_TOP_Z <= -D.DECK_T - 1.0, "the board's tab must stay under the deck"
assert GD_Z0 >= V.TORSO_BOT_Z - 0.01, "the board must not hang below the torso"
assert GD_X0 - SOCKET[2] >= BOSS_FACE_X + 0.5, "the sockets must stay in front of the bosses"

PLATE_T = 2.5
TUBE_R = 2.9                                   # P2's ends are 3.54 off the upper holes' centres
TUBE_BORE = D.CASE_SCREW_CLEAR                 # 2.9: the screw threads only into the boss
RUNG_Z = (V.PI_SCREW_Z[0], (V.PI_SCREW_Z[0] + V.PI_SCREW_Z[1]) / 2)
SCREW_L = 16.0
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
# THE PLATE
# ============================================================================
def gd_plate():
    """Rails through each pair of spacers, two rungs, four spacers; the bores
    pass the screws."""
    x0, x1 = BOSS_FACE_X, BOSS_FACE_X + PLATE_T
    (ylo, yhi), (za, zb) = V.PI_SCREW_Y, V.PI_SCREW_Z
    p = None
    for sy in (ylo, yhi):
        r = box(x0, x1, sy - TUBE_R, sy + TUBE_R, za, zb)
        p = r if p is None else p + r
    for zr in RUNG_Z:
        p += box(x0, x1, ylo, yhi, zr - TUBE_R, zr + TUBE_R)
    for sy, sz in BOSS_YZ:
        p += cyl_x(TUBE_R, x0, GD_X0, sy, sz)
    for sy, sz in BOSS_YZ:
        p -= cyl_x(TUBE_BORE / 2, x0 - 1, GD_X0 + 1, sy, sz)
    return p


def SCREWS():
    """T.SCREWS()'s convention: head at the parts face, axis head -> tip."""
    return [dict(name=f"gd_{sy:+.0f}_{sz:+.0f}", part="gd_plate",
                 kind=f"M2.5x{SCREW_L:.0f} self-tap, pan head: board + spacer into the boss's 2.2 pilot",
                 axis=(-1, 0, 0), pos=(GD_X1, sy, sz), length=SCREW_L,
                 clamp=GD_PCB_T + SPACER_L, pilot=T.PI_PILOT) for sy, sz in BOSS_YZ]


def screw_mocks():
    """The four screws as cylinders (shank + a 4.5 x 1.8 pan head), for pictures."""
    out = None
    for sc in SCREWS():
        _, y, z = sc["pos"]
        s = cyl_x(1.25, GD_X1 - sc["length"], GD_X1, y, z) + cyl_x(2.25, GD_X1, GD_X1 + 1.8, y, z)
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


def check_fits(plate, torso, cover):
    """Seated, with the plugs in: the plate against the torso; the plate
    MIN_AIR clear of every part under the board; the board, its parts and the
    mated plugs clear of the torso, the front cover and the bearing housing.
    Installed: plate + board in from the front, cover off. The service plugs'
    cables leave straight out through the +y skin. The servo plugs leave
    BEND_ROOM under the front cover."""
    import yaw_bearing_housing as YBH
    env = gd_envelopes(plugs=True)
    pcb = gd_pcb()
    ok = True
    v = _vol(plate, torso)
    ok &= _line(v < 0.01, "plate vs torso", f"{v:.2f} mm3 (it bears on the four boss faces)")
    unders = [(n, e) for n, e, s in env if s == "under"]
    hits = [(n, _vol(plate, _box(e, MIN_AIR))) for n, e in unders]
    hits = [(n, h) for n, h in hits if h > 0.0]
    ok &= _line(not hits, f"plate vs the board's under-side parts (+{MIN_AIR} air)",
                "clear" if not hits else ", ".join(f"{n} {h:.2f} mm3" for n, h in hits))
    sock = [e for n, e, _ in env if n == "P2 socket"][0]
    gap_y = min(abs(sy) for sy in V.PI_SCREW_Y) - TUBE_R - max(abs(sock[2]), abs(sock[3]))
    ok &= _line(gap_y >= MIN_AIR, "upper spacers vs the P2 socket's ends", f"{gap_y:.2f} mm")
    gap_x = sock[0] - BOSS_FACE_X
    ok &= _line(gap_x >= 0.5, "sockets in front of the bosses' faces", f"{gap_x:.2f} mm")
    fixed = torso + cover + YBH.yaw_bearing_housing()
    board = Compound([pcb] + [_box(e) for _, e, _ in env])
    v = _vol(board, fixed)
    ok &= _line(v < 0.01, "board + parts + mated plugs vs torso, front cover, bearing housing", f"{v:.2f} mm3")
    v = _vol(board, plate)
    ok &= _line(v < 0.01, "board + parts vs plate", f"{v:.2f} mm3 (the spacers bear on the underside)")
    mover = Compound([plate, pcb] + [_box(e) for _, e, s in env if s != "plug"])
    ok &= T._sweep(mover, torso, ((70.0, 0, 0), (0, 0, 0)), "plate + board in from the front (cover off)", n=24)
    y_out = V.HOUSING_HW + 5.0
    for n, e, s in env:
        if s == "plug" and ("USB-C" in n or "XH" in n):
            x0, x1, y0, y1, z0, z1 = e
            v = _vol(box(x0, x1, y0, y_out, z0, z1), torso)
            ok &= _line(v < 0.01, f"{n}: straight out through the +y skin", f"{v:.2f} mm3")
    room = V.GD_FRONT_X - (GD_X1 + SERVO_PLUG_H)
    ok &= _line(room >= BEND_ROOM, "servo plugs' tops to the front cover",
                f"{room:.2f} mm (>= {BEND_ROOM}); parts face {GD_X1:.2f}, cover {V.GD_FRONT_X:.2f}")
    deck = -D.DECK_T - GD_TOP_Z
    ok &= _line(deck >= 1.0, "board's tab under the deck", f"{deck:.2f} mm")
    return ok


def check_screws(torso, verbose=True):
    """Bite in the boss's pilot (T.MIN_BITE .. the pilot), and a straight
    driver from each head out along +x, the cover off, missing the torso and
    the board's top-side parts (D.ACCESS_D, else T.SLIM_DRIVER_D)."""
    ok = True
    tops = _solids(gd_envelopes(plugs=False), ("top",))
    solid = torso + tops
    for sc in SCREWS():
        bite = sc["length"] - sc["clamp"]
        good = T.MIN_BITE <= bite <= sc["pilot"] + 1e-9
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
    CP.ORIENT[name] = (np.array(PRINT_ORIENT[name]), "gd_plate, no supports")
    CP.PRINT_STL[name] = os.path.basename(stl_path)
    CP.SUPPORTED.pop(name, None)
    return list(CP.audit(name))


# ============================================================================
# pictures
# ============================================================================
C_PLATE, C_SCREW = (0.90, 0.55, 0.20), (0.55, 0.55, 0.60)


def in_print_orient(solid, name):
    saved = dict(T.PRINT_ORIENT)
    T.PRINT_ORIENT.update(PRINT_ORIENT)
    try:
        return T.in_print_orient(solid, name)
    finally:
        T.PRINT_ORIENT.clear()
        T.PRINT_ORIENT.update(saved)


def render(plate, torso, cover, out_dir):
    """gd_plate.png: the plate as it prints; seated on the bosses (cover
    off); the board on it with the plugs mated (light grey: the servo plugs'
    envelopes square to the board, the USB-C and XH plugs out of the service
    edge toward the +y skin's window), from the front and +y; the bay cut at
    y = +29 through the +y screws, seen from +y, cover on.
    gd_plate_exploded.png: torso, plate, board and cover apart along x."""
    board = [(s, T.C_BOARD) for s in gd_render_solids()] + [(gd_pcb(), (0.12, 0.30, 0.18))]
    plugs = [(_box(e), (0.85, 0.85, 0.90)) for _, e, s in gd_envelopes() if s == "plug"]
    keep = box(8.0, 60.0, -200, -V.PI_SCREW_Y[0] + 0.01, -80.0, 3.0)
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
    seated = [(torso, T.C_TORSO), (plate, C_PLATE), (screw_mocks(), C_SCREW)] + board
    shots = [([(in_print_orient(plate, "gd_plate"), C_PLATE)], 135, -50, 2.2),
             ([(torso, T.C_TORSO), (plate, C_PLATE)], 160, -25, 1.5),
             (seated + plugs, 215, -25, 1.5),
             (cut(seated + plugs + [(cover, T.C_COVER)]), -90, 0, 2.1)]
    a = T._render_strip(shots, os.path.join(out_dir, "gd_plate.png"))
    dx = 45.0
    ex = ([(torso, T.C_TORSO), (Pos(dx, 0, 0) * plate, C_PLATE), (Pos(3 * dx, 0, 0) * cover, T.C_COVER)]
          + [(Pos(2 * dx, 0, 0) * s, c) for s, c in board])
    b = T._render_strip([(ex, 150, -20, 1.5)], os.path.join(out_dir, "gd_plate_exploded.png"), px=900)
    return a, b


def mass_g(s):
    return s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


if __name__ == "__main__":
    import time
    for d in (OUT_STL, OUT_STEP, OUT_REN):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    plate = gd_plate()
    torso, cover = T.torso_v8(arm_mounts=True), T.front_cover_v8()
    bb = plate.bounding_box()
    path = os.path.join(OUT_STL, "gd_plate.stl")
    export_stl(plate, path)
    export_step(plate, os.path.join(OUT_STEP, "gd_plate.step"))
    print(f"gd_plate  {len(plate.solids())} solid(s)  {mass_g(plate):.1f} g  "
          f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm; built in {time.time() - t0:.0f}s")
    print(f"board: underside x {GD_X0:.2f}, parts face {GD_X1:.2f}, z {GD_Z0:.2f}..{GD_TOP_Z:.2f}, "
          f"service edge y {GD_Y0:+.2f}")
    print("\n-- fits --")
    ok = check_fits(plate, torso, cover)
    print("\n-- screws --")
    ok &= check_screws(torso)
    print("\n-- printability --")
    findings = audit("gd_plate", path)
    print(f"  gd_plate: {'PASS, no supports' if not findings else f'** {len(findings)} FINDING(S) **'}")
    for kind, a, desc in findings:
        print(f"      {kind:8s} {desc}")
    ok &= not findings
    print("\nwrote", *render(plate, torso, cover, OUT_REN))
    print("ALL CHECKS PASS" if ok else "** SOME CHECKS FAIL **")
    sys.exit(0 if ok else 1)
