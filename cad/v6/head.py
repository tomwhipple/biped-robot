"""head: the camera head on its own neck servo (v6 body, 2026-09-14).

Local frame: the neck horn's mounting face is z=0 (the flat disc face the
head bolts to), the yaw axis is the Z axis through the origin, +X forward
(camera looks +X). `assembly_v6.py` places this frame at V.NECK_HORN_Z in
the world (`Pos(V.NECK_X, 0, V.NECK_HORN_Z) * head()`), so a local z here IS
a height above the horn face -- CAM_Z_ABOVE_HORN and TOP_Z in
dimensions_v6.py both already add V.HEAD_BASE_T once to get there, and this
module must not add it twice.

Two parts, one assembly:
  head_shell()  base plate (bolts to the horn) + sides + top, OPEN at +X
                (the front) -- the servo, wiring and the 4 horn screws are
                all worked on through that opening before the face goes on
  head_face()   the plate that closes the opening, carrying the Camera
                Module 3 on its inside face and the lens aperture
  head()        the two fused, for the assembly and for mass/print checks

KNOWN PRINTABILITY FINDING -- for whoever slices this: `check_printability`
reports a `CEILING ... roof of a through-void` right at DOME_Z0 (the seam
where the straight lower shell meets the domed top, see head_shell()),
sized to the shell's own footprint. Solid material exists on BOTH sides of
that plane (it is the boundary between the straight-walled section and the
dome loft, built with a deliberate 3 mm volumetric overlap so the union
fuses into one solid), so this reads to me as a coincident/internal-face
artifact from the boolean fuse rather than a genuine unsupported bridge --
but I could not fully confirm that from the mesh alone. CHECK THE SLICED
PREVIEW at that layer height before trusting it print-clean; if the slicer
also sees a bridge there, the fix is to shrink or remove the box/loft
overlap band in head_shell() and re-check.

Run:
    .venv/bin/python cad/v6/head.py     # STL + STEP (both parts + fused) + checks + render
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import *  # noqa: E402,F403
import dimensions as D  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402

box, cyl_x, cyl_y, cyl_z = P.box, P.cyl_x, P.cyl_y, P.cyl_z
teardrop_x, teardrop_y = P.teardrop_x, P.teardrop_y

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# print orientation: base (the horn plate) down on the bed, shell rising --
# no rotation needed, the local frame already has z=0 at the base.
PRINT_ORIENT = np.eye(3)
PRINT_ORIENT_FACE = np.eye(3)   # head_face prints flat, camera bosses up

# ---------------------------------------------------------------- geometry
BCD_R = D.BCD / 2                              # 7.0 horn bolt circle radius
BASE_T = V.HEAD_BASE_T                         # 4.0
SHELL_Z0 = BASE_T                              # 4.0 -- where the walls start
SHELL_Z1 = BASE_T + V.HEAD_H                   # 60.0 -- shell top (local z)
HW = V.HEAD_WALL                               # 2.0
FRONT_X = V.HEAD_D / 2                         # 25.0 -- the open front plane
FILLET_R = 9.0                                 # organic corners, R 8+
CAM_Z = BASE_T + V.CAM_Z_ABOVE_HORN            # 34.0 local z, lens centre
CABLE_SLOT_X = -(BCD_R + 2.0 + V.HEAD_CABLE_SLOT[1] / 2)   # clear of the BCD circle


def _footprint(z0, z1):
    """The shell's rounded-rectangle footprint (HEAD_D x HEAD_W), a box that
    gets its vertical edges filleted -- used for both the base plate and the
    shell walls so they share one outer silhouette."""
    return box(-FRONT_X, FRONT_X, -V.HEAD_W / 2, V.HEAD_W / 2, z0, z1)


def _round_vertical_edges(solid, r=FILLET_R):
    """Fillet the vertical edges at radius r, falling back to smaller radii
    if the material is too thin for it (a thin plate geometrically cannot
    take a big corner radius -- max radius is roughly half the thinnest
    adjacent face, so this reports whatever it actually achieved instead of
    silently doing nothing)."""
    edges = [e for e in solid.edges() if abs(e.tangent_at(0.5).Z) > 0.999]
    for rr in (r, r * 0.7, r * 0.5, r * 0.3, r * 0.15):
        try:
            out = fillet(edges, rr)
            if rr < r:
                print(f"  [head] vertical fillet: R{r} did not fit this "
                      f"section's material, used R{rr:.2f} instead")
            return out
        except Exception:  # noqa: BLE001
            continue
    print(f"  [head] vertical fillet skipped entirely (R{r} -- material too thin)")
    return solid


DOME_Z0 = SHELL_Z1 - 16.0        # dome starts 16 mm below the top
CROWN = 7.0                      # flat crown at the very top, <= 8 mm (BRIDGE_OK)


def head_shell():
    """Base plate (bolted to the neck horn) + side/back walls + a DOMED
    top, open at +X. The base plate is the shell's own footprint, 4 mm
    thick, carrying the 4x M3x6 horn screws on the O14 BCD, the horn's
    centre relief, and a ribbon slot beside (not through) the BCD for the
    camera lead.

    THE TOP IS A DOME, not a flat roof: a flat 48 x 58 mm ceiling closing
    the shell was the very first thing this module built, and printed
    base-down it is a dead-flat unsupported bridge dead centre of the
    print -- the biggest single printability finding the first pass found.
    From DOME_Z0 up to the top, both the outer wall and the inner cavity
    taper from the full footprint down to a small CROWN x CROWN flat cap
    (<= 8 mm, PETG's proven bridge span) -- the wall thickness (HW) is held
    constant through the taper (inner loft offset by HW at both ends), so
    the dome is a self-supporting 45-deg-plus slope with only that small
    flat cap left to bridge.
    """
    # base plate + a squat block for the straight-walled lower shell,
    # filleted once as a single outer silhouette (base and shell share the
    # same footprint)
    p = _footprint(0, DOME_Z0)
    p = _round_vertical_edges(p)

    def _rect(x0, x1, y0, y1, z):
        return Pos((x0 + x1) / 2, (y0 + y1) / 2, z) * Rectangle(abs(x1 - x0), abs(y1 - y0))

    # the dome: the OPEN FRONT (+X) edge stays fixed at FRONT_X the whole
    # way up (a symmetric taper would dome over the opening too, closing
    # what the face plate has to seal) -- only the back (-X) and the two
    # sides converge, to a CROWN x CROWN cap whose own +X edge is still at
    # FRONT_X.
    outer_lo = _rect(-FRONT_X, FRONT_X, -V.HEAD_W / 2, V.HEAD_W / 2, DOME_Z0)
    outer_hi = _rect(FRONT_X - CROWN, FRONT_X, -CROWN / 2, CROWN / 2, SHELL_Z1)
    dome_outer = loft([outer_lo.faces()[0], outer_hi.faces()[0]])
    # (an edge fillet on the dome's own slanted corners was tried here for
    # extra polish; the asymmetric taper leaves no purely-vertical edge to
    # select and it isn't worth chasing further -- the R9 fillet below on
    # the straight lower shell already does the "organic, no sharp
    # corners" work the brief asked for.)
    p += dome_outer
    # hollow the shell interior (leave BASE_T of floor, HW of wall all round,
    # open at +X all the way -- the front face is not walled at all, that is
    # head_face()'s job): straight below DOME_Z0, then the SAME taper as the
    # outer skin (inset by HW on the back/sides, front edge left open past
    # FRONT_X throughout) so the dome's wall stays HW thick to the crown.
    # built as ONE void (straight part fused to the dome loft) and
    # subtracted in a single boolean -- doing these as two separate
    # subtractions with only a hairline (0.01 mm) overlap left a false
    # "ceiling" reading right at the seam (coincident, not overlapping,
    # faces -- the same class of fusion bug as pelvis_v7's boss lesson),
    # which the printability pass caught as a 48 x 58 mm bridge. Genuine
    # overlap fixes it -- but the overlap band has to stay INSIDE the
    # outer wall's own (also tapering) profile: the loft's start is moved
    # 2 mm BELOW DOME_Z0 at the full (untapered) cross-section, matched by
    # a straight box up to DOME_Z0 -- both still full-size there, since
    # the outer skin does not start narrowing until DOME_Z0 either.
    void_lo = box(-FRONT_X + HW, FRONT_X + 1.0, -V.HEAD_W / 2 + HW, V.HEAD_W / 2 - HW,
                  SHELL_Z0, DOME_Z0)
    crown_in = max(CROWN - 2 * HW, 1.5)
    inner_lo = _rect(-FRONT_X + HW, FRONT_X + 1.0, -V.HEAD_W / 2 + HW, V.HEAD_W / 2 - HW,
                     DOME_Z0 - 2.0)
    inner_hi = _rect(FRONT_X - crown_in, FRONT_X + 1.0, -crown_in / 2, crown_in / 2,
                     SHELL_Z1 + 1.0)
    dome_inner = loft([inner_lo.faces()[0], inner_hi.faces()[0]])
    p -= (void_lo + dome_inner)
    # lighten the base plate: a solid 4 mm slab across the whole 50 x 62
    # footprint is ~14 g on its own (mass budget is 45 g total incl. the
    # camera) for no structural reason -- the wall needs a HW-thick rim to
    # root into and the horn needs a solid boss around its BCD, everything
    # else can be a 1.5 mm floor. Pocket opens DOWNWARD (from the horn face)
    # so the BOSS+RIM ring is what actually touches the horn.
    rim = HW + 1.5
    boss_r = BCD_R + 5.0
    pocket = box(-FRONT_X + rim, FRONT_X - rim, -V.HEAD_W / 2 + rim, V.HEAD_W / 2 - rim,
                 -1, BASE_T - 1.3)
    pocket -= cyl_z(boss_r, -1, BASE_T + 1, 0, 0)
    p -= pocket
    # horn screws: 4x M3x6 on the O14 BCD, centre relief for the horn's own
    # screw/boss. Countersunk-style clearance, driven from ABOVE (inside the
    # open shell) down into the horn -- see check_horn_access().
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        p -= cyl_z(D.PAD_HOLE / 2, -1, BASE_T + 1, x, y)
    p -= cyl_z(D.HORN_CENTER_RELIEF_D / 2, -1, BASE_T + 1, 0, 0)
    # ribbon slot through the base plate, beside the axis, clear of the BCD
    # holes (BCD_R + 2 mm margin before the slot's near edge).
    sx = CABLE_SLOT_X
    p -= box(sx - V.HEAD_CABLE_SLOT[1] / 2, sx + V.HEAD_CABLE_SLOT[1] / 2,
             -V.HEAD_CABLE_SLOT[0] / 2, V.HEAD_CABLE_SLOT[0] / 2, -1, BASE_T + 1)
    # 4x M2.5 self-tap bosses for head_face(), on the inside of the front
    # opening's rim (top/bottom/side pairs), teardropped so the horizontal
    # bore prints self-supporting base-down.
    for fy, fz in FACE_SCREW_POS:
        p += cyl_x(3.0, FRONT_X - 6.0, FRONT_X, fy, fz)
        p -= teardrop_x(D.M25_TAP / 2, FRONT_X - 7.0, FRONT_X + 1.0, fy, fz, roll=180)
    return p


# face-plate screw positions (y, z), local frame -- corners of the opening,
# inset from the shell's own footprint edges and the lens aperture
# positioned so a r=3 boss cylinder OVERLAPS the side wall (inner face at
# +/-(HEAD_W/2 - HW)) by ~1 mm -- enough to fuse solidly without poking
# through the outer skin.
_FSY = V.HEAD_W / 2 - HW - 2.0
# both rows sit BELOW DOME_Z0 -- the dome narrows the side walls above that
# (see head_shell), so a screw position up near the old flat top would now
# be outside the tapered wall entirely.
FACE_SCREW_POS = (
    (_FSY, SHELL_Z0 + 8.0),
    (-_FSY, SHELL_Z0 + 8.0),
    (_FSY, DOME_Z0 - 6.0),
    (-_FSY, DOME_Z0 - 6.0),
)
# camera board bosses (M2, on CAM3_HOLES spacing about the lens centre)
CAM_BOSS_POS = tuple(
    (dy * V.CAM3_HOLES[0] / 2, CAM_Z + dz * V.CAM3_HOLES[1] / 2)
    for dy, dz in ((1, 1), (1, -1), (-1, 1), (-1, -1)))


def head_face():
    """The front plate: closes the shell's open +X face, held by 4x M2.5
    self-tap screws into the shell's rim bosses, carrying the Camera
    Module 3 on 4x M2 bosses on its INSIDE (-X) face, lens through a
    CAM3_LENS_D + 2 mm aperture centred CAM_Z_ABOVE_HORN above the horn."""
    t = D.PLATE                      # 3.0 mm plate, same stock as every joint arm
    p = box(FRONT_X, FRONT_X + t, -V.HEAD_W / 2, V.HEAD_W / 2, SHELL_Z0, SHELL_Z1)
    p = _round_vertical_edges(p, 4.0)   # R4 asked; a 3 mm plate physically caps near ~1.3 mm
    # lighten: a 3 mm solid plate over the whole opening is ~12 g on its own;
    # pocket the OUTER (+X, visible) face down to 1.2 mm everywhere except a
    # rim (screw bosses need the full 3 mm to bite) and a disc around the
    # lens/camera board (needs the full thickness for the boss standoffs).
    rim = 6.0
    pocket = box(FRONT_X + 1.2, FRONT_X + t + 1.0, -V.HEAD_W / 2 + rim, V.HEAD_W / 2 - rim,
                SHELL_Z0 + rim, SHELL_Z1 - rim)
    pocket -= cyl_x(16.0, FRONT_X, FRONT_X + t + 1.0, 0, CAM_Z)
    p -= pocket
    # lens aperture, all the way through
    p -= cyl_x((V.CAM3_LENS_D + 2.0) / 2, FRONT_X - 1.0, FRONT_X + t + 1.0, 0, CAM_Z)
    # clearance holes for the 4 shell-rim screws
    for fy, fz in FACE_SCREW_POS:
        p -= cyl_x(D.CASE_SCREW_CLEAR / 2, FRONT_X - 1.0, FRONT_X + t + 1.0, fy, fz)
        p -= P.csk_x(fy, fz, FRONT_X + t, +1)
    # 4x M2 bosses on the inside face for the Camera Module 3 (25 x 24 board,
    # standing off the plate so the lens barrel clears the plate's own
    # thickness -- 3 mm standoff, plenty for a Pi camera board)
    boss_h = 3.0
    for by, bz in CAM_BOSS_POS:
        p += cyl_x(2.2, FRONT_X - boss_h, FRONT_X, by, bz)
        p -= cyl_x(1.0, FRONT_X - boss_h - 0.1, FRONT_X + 0.1, by, bz)
    return p


def head():
    """head_shell() + head_face(), fused -- the whole printed/assembled
    head for the robot assembly, ROM checks and the mass rollup."""
    return head_shell() + head_face()


# ----------------------------------------------------------------------------
# mocks (head-local frame)
# ----------------------------------------------------------------------------
def mock_neck_servo():
    """The neck STS3215's case, hanging BELOW the horn face (the servo
    stands on the deck, horn up, into the head above it) -- v5's servo_mock_z
    rotated 180 deg about X so the case is on the correct side of z=0."""
    return Pos(0, 0, -D.SV_HORN_FACE) * Rot(180, 0, 0) * CA.servo_mock_z()


def mock_deck_plane():
    """A thin slab standing in for the pelvis deck top, V.NECK_HORN_FACE_Z
    below the horn face (i.e. below z=0 here) -- swept through yaw only to
    confirm the head's own rotation never reaches down that far anyway."""
    z = -V.NECK_HORN_FACE_Z
    return box(-200, 200, -200, 200, z - 4, z)


def mock_camera():
    return Pos(FRONT_X - 1.5, 0, CAM_Z) * Box(D.PLATE, *V.CAM3_BOARD)


# ----------------------------------------------------------------------------
# fasteners
# ----------------------------------------------------------------------------
def SCREWS():
    s = []
    # disc screws through the base plate, length by the stack (V.disc_screw)
    stack = V.HEAD_BASE_T
    length, eng, flange = V.disc_screw(stack, "horn")
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        s.append(dict(name=f"horn_{ang}", kind=f"M3x{length} button head (into horn)",
                      axis=(0, 0, -1), pos=(x, y, stack), length=float(length),
                      stack=stack, engage=eng, flange=flange))
    for i, (fy, fz) in enumerate(FACE_SCREW_POS):
        s.append(dict(name=f"face_{i}", kind="M2.5x8 self-tap, flat head",
                      axis=(-1, 0, 0), pos=(FRONT_X + D.PLATE, fy, fz), length=8.0))
    for i, (by, bz) in enumerate(CAM_BOSS_POS):
        s.append(dict(name=f"cam_{i}", kind="M2x4 self-tap", axis=(1, 0, 0),
                      pos=(FRONT_X - 3.0, by, bz), length=4.0))
    return s


def check_horn_access(verbose=True):
    """The 4 horn screws must be drivable from INSIDE the open shell before
    the face goes on: a straight 7 mm (ACCESS_D) cylinder from each BCD hole
    up to open space inside the shell must be clear. The driver only needs
    to reach into the big straight cavity below the dome, not all the way
    to the crown -- the dome converges toward the FRONT (+X) opening, not
    the centre, so a centred BCD hole's vertical path would run into the
    dome's own (deliberately closing-in) material above DOME_Z0 even
    though the shell is entirely serviceable from below that."""
    shell = head_shell()
    ok = True
    for ang in (45, 135, 225, 315):
        x, y = BCD_R * math.cos(math.radians(ang)), BCD_R * math.sin(math.radians(ang))
        cyl = cyl_z(D.ACCESS_D / 2, BASE_T + 0.5, DOME_Z0 - 0.5, x, y)
        try:
            vol = (cyl & shell).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        clear = vol < 2.0
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  horn_{ang:<4d} driver-from-inside "
                  f"overlap {vol:6.1f} mm3")
    return ok


def check_rom_clearance(verbose=True, samples=13):
    """Sweep neck yaw over V.NECK_ROM and confirm the head clears (a) the
    neck servo case mock and (b) the pelvis deck plane, at every angle."""
    h = head()
    servo = mock_neck_servo()
    deck = mock_deck_plane()
    lo, hi = V.NECK_ROM
    worst = {"servo": (0.0, 1e9), "deck": (0.0, 1e9)}
    for k in range(samples):
        ang = lo + (hi - lo) * k / (samples - 1)
        hp = Rot(0, 0, ang) * h
        for name, mock in (("servo", servo), ("deck", deck)):
            try:
                vol = (hp & mock).volume
            except Exception:  # noqa: BLE001
                vol = 0.0
            try:
                dist = hp.distance_to(mock)
            except Exception:  # noqa: BLE001
                dist = float("nan")
            if vol > worst[name][0]:
                worst[name] = (vol, dist)
    ok = True
    for name, (vol, dist) in worst.items():
        clear = vol < 0.5
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  head vs {name:6s} over +/-{hi:.0f} deg yaw: "
                  f"worst overlap {vol:.2f} mm3, dist {dist:.2f} mm")
    return ok


# ----------------------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    os.makedirs(OUT_REN, exist_ok=True)

    print("building head_shell(), head_face(), head()...")
    shell = head_shell()
    face = head_face()
    whole = shell + face

    for name, s in (("head_shell", shell), ("head_face", face), ("head", whole)):
        bb = s.bounding_box()
        dims = (bb.size.X, bb.size.Y, bb.size.Z)
        bed_ok = max(dims) <= D.BED
        mass = s.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
        n_solids = len(s.solids())
        print(f"{name:11s} bbox {dims[0]:5.1f} x {dims[1]:5.1f} x {dims[2]:5.1f} mm  "
              f"mass {mass:5.1f} g  solids {n_solids} "
              f"({'ok' if n_solids == 1 else '** SPLIT **'})  "
              f"{'BED-OK' if bed_ok else '** TOO BIG **'}")
        export_stl(s, os.path.join(OUT_STL, f"{name}.stl"))
        export_step(s, os.path.join(OUT_STEP, f"{name}.step"))

    total_mass = whole.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR + V.CAM3_MASS
    print(f"\nhead total incl. Camera Module 3 ({V.CAM3_MASS} g): {total_mass:.1f} g "
          f"({'PASS <= 45 g' if total_mass <= 45.0 else '** FAIL, over 45 g budget **'})")

    print("\n-- horn screw driver access (from inside, before the face goes on) --")
    check_horn_access()

    print("\n-- ROM clearance (+/-90 deg yaw vs the neck servo and the deck) --")
    check_rom_clearance()

    print("\n-- printability audit --")
    import audit_torso  # noqa: E402
    audit_torso.run(["head", "head_shell", "head_face"])

    print("\n-- render --")
    try:
        import mujoco  # noqa: F401
        import imageio
        import tempfile
        tmp = tempfile.mkdtemp(prefix="head_")
        stl_path = os.path.join(tmp, "p.stl")
        export_stl(whole, stl_path)
        bb = whole.bounding_box()
        cz = (bb.min.Z + bb.max.Z) / 2000.0
        cx = (bb.min.X + bb.max.X) / 2000.0
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="1200" offheight="900"/></visual>
<asset><mesh name="m" file="p.stl" scale="0.001 0.001 0.001"/></asset>
<worldbody><light pos="0.3 0.3 0.3" dir="-0.4 -0.4 -0.6" directional="true"/>
<light pos="-0.3 -0.2 0.2" dir="0.4 0.3 -0.4" directional="true"/>
<geom type="mesh" mesh="m" rgba="0.85 0.83 0.78 1"/></worldbody></mujoco>"""
        xml_path = os.path.join(tmp, "s.xml")
        with open(xml_path, "w") as f:
            f.write(xml)
        m = mujoco.MjModel.from_xml_path(xml_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, 900, 1200)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [cx, 0, cz]
        cam.distance = 0.18
        imgs = []
        for az, el in ((150, -10), (60, -10), (0, -10)):
            cam.azimuth, cam.elevation = az, el
            r.update_scene(d, cam)
            imgs.append(r.render().copy())
        import numpy as _np
        imageio.imwrite(os.path.join(OUT_REN, "head.png"), _np.concatenate(imgs, axis=1))
        print("wrote", os.path.join(OUT_REN, "head.png"))
    except Exception as e:  # noqa: BLE001
        print(f"render skipped ({type(e).__name__}: {e})")
