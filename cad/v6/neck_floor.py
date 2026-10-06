"""neck_floor: the neck servo's seat -- a floor plate screwed into the neck tube
(issue #90).

WHY IT EXISTS
-------------
The neck STS3215 stands on its idler face, horn up, and is held only by the
four stator screws in that face; it carries the head and camera through +-90
deg of yaw. The deck cannot seat it: the deck's battery aperture
(dimensions_v6.DECK_APER_X / _HY) runs under the servo's whole footprint, and
the pack has to lift out through that aperture. So the seat is this plate: it
hangs from the neck tube (the girdle's, or neck_collar's in the armless build)
and drops into the aperture at the old well depth, so the neck height is
unchanged.

    floor     2 mm, top at -NECK_WELL_D (-3.0), bottom at the deck-bottom
              plane (-5.0): 4.5 mm over the pack (dimensions_v6 asserts >= 4).
    pads      four, up to the case's idler face (z 0) at the stator rows
              (8.30 / 32.75 behind the axis, +-10.25). They are trimmed clear
              of everything on that face that stands proud of it: the rotating
              idler disc (O19.2, 2.05 proud -> NECK_DISC_RELIEF_R), its hub
              (2.6 proud -> a pocket in the floor) and the moulded back-cover
              platform (1.9 proud).
    stators   4 x M2.5 x 8 flat-head, driven UP through the floor and the pads
              into the idler-face rows. Heads seat in a 1 mm counterbore, so
              each screw bites 4.0 mm into the case (the yaw cells' is 3.9).
    lugs      four, rising to the tube's print plane (z 0), each screwed UP
              with an M2.5 x 8 flat-head into a boss on the tube's side wall
              (tube_bosses(), 6 mm of bite). The load path is case -> stator
              screws -> pads -> plate -> lugs -> tube walls -> girdle.
    lead      a window through the floor under the connector trench, open to
              the robot's left (+y) past the tube wall, so the two bus leads
              leave the plugs sideways at floor level (or straight down) and
              rise through the aperture beside the tube.

WHY A SEPARATE PRINT
--------------------
The girdle prints base-down on its deck face, and nothing of it may hang below
that plane (shoulder_girdle_v6's print block). A floor 5 mm under it would make
this small plate the girdle's only bed contact and leave the whole 200 mm base
in mid-air. As its own part the plate prints flat, bottom down, with no
supports: the counterbores and countersinks open onto the bed, the pads and
lugs rise from it.

BENCH ORDER
-----------
The pack goes into the pelvis before the girdle, and the plate sits over the
pack, so the neck is built into the girdle ON THE BENCH: plate screwed to the
tube bosses, servo (leads plugged first) dropped into the tube from above onto
the pads, four stator screws up from below; then the girdle goes onto the deck.

Pelvis frame throughout (deck top z = 0, +x forward, +y left), the frame the
girdle and neck_collar are drawn in.

    .venv/bin/python cad/v6/neck_floor.py          # STL + STEP + checks
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos, Rot, export_step, export_stl  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402

D = V.D
box, cyl_z, wedge_y = P.box, P.cyl_z, P.wedge_y

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")

IDENT = np.eye(3)
PRINT_ORIENT = {"neck_floor": (IDENT, "flat, bottom face down, pads and lugs up")}

Z0, Z1 = V.NECK_FLOOR_Z                      # -5.0, -3.0
X0, X1 = V.NECK_FLOOR_X                      # -36.21, +10.10
HY = V.NECK_FLOOR_HY                         # 15.26
CASE_Z = 0.0                                 # the case's idler face (stator rows) in the pelvis frame
LX0, LX1 = V.NECK_LEAD_WIN                   # -17.35, -10.75


def lug_xy():
    return [(x, s * V.NECK_LUG_Y) for x in V.NECK_LUG_X for s in (1, -1)]


# ===========================================================================
# the part
# ===========================================================================
def neck_floor():
    """The plate, pelvis frame. Qty 1 per robot (both builds)."""
    p = box(X0, X1, -HY, HY, Z0, Z1)
    # --- lugs: a round ear out to each boss, full height to the tube's
    # print plane, webbed back into the plate
    for lx, ly in lug_xy():
        p += cyl_z(V.NECK_LUG_R, Z0, CASE_Z, lx, ly)
        p += box(lx - V.NECK_LUG_R, lx + V.NECK_LUG_R, np.sign(ly) * (HY - 1.0), ly, Z0, CASE_Z)
    # --- stator pads, up to the case face
    for px, py in V.NECK_PAD_XY:
        p += cyl_z(V.NECK_PAD_R, Z1 - 0.01, CASE_Z, px, py)
    # the pads and lugs stop at the plate's own outline in x (aperture edges)
    p &= box(X0, X1, -HY - 10, HY + 10, Z0 - 1, CASE_Z + 1)
    # --- reliefs for what stands proud of the idler face
    #   the idler disc turns with the joint: nothing inside its circle above the floor
    p -= cyl_z(V.NECK_DISC_RELIEF_R, Z1, CASE_Z + 1, 0, 0)
    #   its hub stands 2.6 proud (0.4 over the floor top): a pocket under it
    r_hub, d_hub = V.NECK_HUB_RELIEF
    p -= cyl_z(r_hub, Z1 - d_hub, Z1 + 0.01, 0, 0)
    #   the moulded back-cover platform (1.9 proud, x -29.51..-16.25, +-10.7)
    p -= box(D.SV_IDLER_BOSS_Z[0] - D.RIB_RELIEF_CLR, D.SV_IDLER_BOSS_Z[1] + D.RIB_RELIEF_CLR,
             -(D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR), D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR,
             Z1 + 0.01, CASE_Z + 1)
    # --- the lead window under the connector trench, through the plate and
    # out past the tube wall on +y, so the leads can leave sideways
    p -= box(LX0, LX1, -(D.SV_CONN_HW + 0.5), HY + 10, Z0 - 1, CASE_Z + 1)
    # --- the aft end sits over the pelvis's rear web (top at z -5, front edge
    # x -35.41): a chamfer on the plate's aft-bottom edge keeps the plate
    # 0.5 mm off it. 55 deg, not 45: steeper than the printability rule's
    # overhang limit, so it is a wall in print, not a ledge.
    p -= wedge_y([(X0 - 0.01, Z0 - 0.01), (X0 - 0.01, Z1), (X0 + 1.4, Z0 - 0.01)], -HY - 10, HY + 10)
    # --- stator screws: clearance up through plate + pad, countersunk from
    # below at the top of a 1 mm counterbore
    for px, py in V.NECK_PAD_XY:
        seat = Z0 + V.NECK_STATOR_CB
        p -= cyl_z(D.CASE_SCREW_CLEAR / 2, Z0 - 1, CASE_Z + 1, px, py)
        p -= cyl_z(D.CASE_CS_D / 2, Z0 - 1, seat, px, py)   # the cone's own mouth: no ledge at the seat
        p -= P.csk_z(px, py, seat, -1)
    # --- lug screws: counterbored so the head seats at z -2.0
    for lx, ly in lug_xy():
        seat = Z0 + V.NECK_LUG_CB
        p -= cyl_z(D.CASE_SCREW_CLEAR / 2, Z0 - 1, CASE_Z + 1, lx, ly)
        p -= cyl_z(D.CASE_CS_D / 2, Z0 - 1, seat, lx, ly)
        p -= P.csk_z(lx, ly, seat, -1)
    return p


def tube_bosses():
    """(solid to ADD, solid to CUT) for the neck tube that carries the floor:
    four bosses on the tube's side walls, z 0 .. NECK_BOSS_H, each with a
    blind 2.05 pilot from the tube's bottom face. Used by shoulder_girdle_v6
    and neck_collar, which share the tube footprint."""
    add = cut = None
    for lx, ly in lug_xy():
        b = cyl_z(V.NECK_BOSS_R, 0.0, V.NECK_BOSS_H, lx, ly)
        h = cyl_z(D.CASE_SCREW_PILOT / 2, -1.0, V.NECK_BOSS_PILOT, lx, ly)
        add = b if add is None else add + b
        cut = h if cut is None else cut + h
    return add, cut


# ===========================================================================
# mocks, fasteners, insertion
# ===========================================================================
def mock_neck_servo():
    """The neck STS3215 as assembly_v6 places it (pelvis frame)."""
    return Pos(V.NECK_X, 0, V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()


def SCREWS():
    """Every fastener the floor adds, pelvis frame. `axis` points from the head
    toward the tip; `bite` is the thread length inside the part it holds."""
    out = []
    for px, py in V.NECK_PAD_XY:
        seat = Z0 + V.NECK_STATOR_CB
        out.append(dict(name=f"neck_stator_{-px:.0f}_{py:+.0f}", frame="pelvis",
                        kind="M2.5x8 self-tap, flat head (UP through the floor into the neck servo's idler-face rows)",
                        pos=(px, py, seat), axis=(0, 0, 1), length=8.0, bite=seat + 8.0 - CASE_Z))
    for lx, ly in lug_xy():
        seat = Z0 + V.NECK_LUG_CB
        out.append(dict(name=f"neck_floor_lug_{lx:+.0f}_{ly:+.0f}", frame="pelvis",
                        kind="M2.5x8 self-tap, flat head (UP through the floor lug into the neck tube boss)",
                        pos=(lx, ly, seat), axis=(0, 0, 1), length=8.0, bite=seat + 8.0))
    return out


# the order IS the bench order (girdle upside down on the bench)
INSERT = {
    "neck_floor": (0, 0, -1),    # offered UP into the tube from below, screwed to the bosses
    "servo_neck": (0, 0, 1),     # then the servo drops into the tube from above onto the pads
}


# ===========================================================================
# checks
# ===========================================================================
def check_seat(verbose=True):
    """The servo must sit ON the pads (touching, no overlap) and nothing else
    of the plate may come near what turns on the idler face."""
    f = neck_floor()
    m = mock_neck_servo()
    ok = True
    vol = (f & m).volume
    good = vol < 0.05
    ok &= good
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  neck servo vs floor overlap {vol:.3f} mm3 (seated on the pads)")
    # the pads must actually meet the case face: top of each pad at z 0
    for px, py in V.NECK_PAD_XY:
        ring = cyl_z(V.NECK_PAD_R - 0.6, CASE_Z - 0.2, CASE_Z, px, py) \
            - cyl_z(D.CASE_SCREW_CLEAR / 2 + 0.2, CASE_Z - 1, CASE_Z + 1, px, py)
        top = f & ring
        good = top.volume > 1.0
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  pad under the stator row ({px:+.2f}, {py:+.2f}) reaches the case face")
    # the rotating parts: distance from the plate to the disc + hub
    disc = m & box(-12, 12, -12, 12, -5, -0.01)
    d = f.distance_to(disc)
    good = d >= 0.3
    ok &= good
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  idler disc + hub to the floor: {d:.2f} mm (>= 0.3; they turn)")
    return ok


def check_pack_clearance(verbose=True):
    """Floor vs the pack mock and the rest of what is under the aperture."""
    import pelvis_v7 as PV
    f = neck_floor()
    ok = True
    for name, solid, need in (("pack", PV.mock_pack(), 4.0), ("buck", PV.mock_buck(), 4.0),
                              ("Pi 4B", PV.mock_pi4(), 1.0), ("General Driver", PV.mock_gd(), 1.0)):
        d = f.distance_to(solid)
        good = d >= need
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  floor vs {name:15s} {d:6.2f} mm (need >= {need})")
    return ok


def check_pelvis_clearance(verbose=True):
    """The plate drops into the aperture: it must not touch the pelvis."""
    import pelvis_v7 as PV
    f = neck_floor()
    pel = PV.pelvis_v7(arm_mounts=True)
    vol = (f & pel).volume
    d = f.distance_to(pel)
    good = vol < 0.01 and d >= 0.25
    if verbose:
        print(f"  {'PASS' if good else 'FAIL'}  floor vs pelvis: overlap {vol:.3f} mm3, gap {d:.2f} mm (need >= 0.25)")
    return good


def check_screw_bite(verbose=True):
    """Every screw must bite >= 3.5 mm and stay inside its pilot."""
    ok = True
    for s in SCREWS():
        lim = V.NECK_BOSS_PILOT if "lug" in s["name"] else 6.0   # case holes take >= 5.45 (the grip screws)
        good = 3.5 <= s["bite"] <= lim
        ok &= good
        if verbose:
            print(f"  {'PASS' if good else 'FAIL'}  {s['name']:26s} bites {s['bite']:.1f} mm (3.5 .. {lim:.1f})")
    return ok


def check_print(verbose=True):
    import check_printability as CP
    rot, note = PRINT_ORIENT["neck_floor"]
    CP.STL = OUT_STL
    CP.ORIENT["neck_floor"] = (rot, note)
    CP.PRINT_STL["neck_floor"] = "neck_floor.stl"
    return not CP.audit("neck_floor", verbose=verbose)


def run_checks():
    print("\n== seat ==")
    ok = check_seat()
    print("\n== clearance under the aperture ==")
    ok &= check_pack_clearance()
    ok &= check_pelvis_clearance()
    print("\n== screw bite ==")
    ok &= check_screw_bite()
    print("\n== printability (flat, bottom down) ==")
    ok &= check_print()
    return ok


def mass_g(solid):
    return solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR


if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    s = neck_floor()
    bb = s.bounding_box()
    print(f"neck_floor  {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm  {mass_g(s):.1f} g  "
          f"solids {len(s.solids())}")
    export_stl(s, os.path.join(OUT_STL, "neck_floor.stl"))
    export_step(s, os.path.join(OUT_STEP, "neck_floor.step"))
    ok = run_checks()
    print("\nALL CHECKS PASS" if ok else "\n** CHECKS FAILED **")
    sys.exit(0 if ok else 1)
