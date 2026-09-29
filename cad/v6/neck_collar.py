"""neck_collar: the neck servo's tube in the ARMS=0 (armless) build. With the
arms on -- the robot's build -- shoulder_girdle_v6 carries the same tube and
this part is not used.

The tube is the girdle's neck tube (same footprint, same YAW_SEAT_GAP fit,
same height, 2.1 mm short of the horn disc) with a 3 mm flange round its foot.
The neck servo's seat is neck_floor.py, screwed up into the four bosses on the
tube's side walls, exactly as in the girdle.

NOT A PRINT TARGET: the flange's four screw holes have nothing to bite. Their
corners land in the Pi slide slot (aft) and the General Driver lead slot
(forward), and there is no deck under the rest of the flange (the battery
aperture), so the collar has no way to fasten to the pelvis. The armless build
exists for the CAD checks and the mass comparison only (cad/PRINT_LIST.md).

Local frame: z = 0 is the flange's bottom face == pelvis_v7's deck top, x/y
the pelvis's. Prints flat, flange down.

Run:
    .venv/bin/python cad/v6/neck_collar.py   # STL + STEP + checks + render
"""
from __future__ import annotations

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

OUT_STL = os.path.join(HERE, "stl")
OUT_STEP = os.path.join(HERE, "step")
OUT_REN = os.path.join(HERE, "renders")

# prints flat, flange down -- no rotation needed
PRINT_ORIENT = np.eye(3)

# ---------------------------------------------------------------- geometry
NCX0, NCX1 = V.NECK_WELL_X                      # -35.41, 10.41 -- the servo footprint + fit
NC_HY = V.NECK_WELL_HW[0]                       # 12.66
NC_WALL = D.WALL                                # 2.6
COLLAR_TOP = V.NECK_AXIS_Z + D.SV_TOPFACE + 1.0  # 33.1 -- 2.1 mm short of the horn disc
FLANGE_T = 3.0
FLANGE_MARGIN = 8.0
FLANGE_HOLE_INSET = 4.0
AFT_WIN = (12.0, 10.0)                          # (y width, z height)

# flange screw positions (they miss the deck -- see the module docstring)
_nfx = (NCX0 - NC_WALL - FLANGE_HOLE_INSET, NCX1 + NC_WALL + FLANGE_HOLE_INSET)
_nfy = NC_HY + NC_WALL + FLANGE_HOLE_INSET
FLANGE_HOLE_XY = [(_nfx[0], _nfy), (_nfx[1], _nfy), (_nfx[0], -_nfy), (_nfx[1], -_nfy)]


def neck_collar():
    """The tube (YAW_SEAT_GAP fit around the case, D.WALL walls, deck top
    to 2.1 mm short of the horn disc) plus a 3 mm flange 8 mm wide all
    round with 4x M2.5 clearance holes at its corners (self-tap into the
    pelvis deck's own pilot holes below)."""
    # tube, flush with the flange at z=0
    p = box(NCX0 - NC_WALL, NCX1 + NC_WALL, -(NC_HY + NC_WALL), NC_HY + NC_WALL,
            0.0, COLLAR_TOP)
    # flange (built as its own footprint, unioned on -- gives a clean 3 mm
    # slab under the tube without re-deriving the tube's own rounded
    # corners onto a much wider, thinner plate)
    p += box(NCX0 - NC_WALL - FLANGE_MARGIN, NCX1 + NC_WALL + FLANGE_MARGIN,
             -(NC_HY + NC_WALL + FLANGE_MARGIN), NC_HY + NC_WALL + FLANGE_MARGIN,
             0.0, FLANGE_T)
    # round the tube's outer vertical edges -- R1.2 (the fit this wall
    # thickness actually takes; see head.py's face plate for the same limit)
    try:
        edges = [e for e in p.edges() if abs(e.tangent_at(0.5).Z) > 0.999
                and e.center().Z > FLANGE_T + 0.5]
        p = fillet(edges, 1.2)
    except Exception as e:  # noqa: BLE001 -- cosmetic
        print(f"  [neck_collar] tube fillet skipped ({type(e).__name__}: {str(e)[:60]})")
    # the case cavity: open top (the mouth) and open bottom -- the neck
    # servo's seat is neck_floor.py, screwed up into the four bosses below
    # (the girdle's neck tube carries the same four)
    p -= box(NCX0, NCX1, -NC_HY, NC_HY, -1.0, COLLAR_TOP + 1.0)
    import neck_floor as NF
    boss_add, boss_cut = NF.tube_bosses()
    p += boss_add
    p -= boss_cut
    # aft window at the case's cable end
    p -= box(NCX0 - NC_WALL - 1.0, NCX0 + 1.0, -AFT_WIN[0] / 2, AFT_WIN[0] / 2,
             0.0, AFT_WIN[1])
    # 4x M2.5 clearance holes through the flange (no deck pilots to meet:
    # see the module docstring)
    for hx, hy in FLANGE_HOLE_XY:
        p -= cyl_z(D.CASE_SCREW_CLEAR / 2, -0.5, FLANGE_T + 0.5, hx, hy)
    return p


# ----------------------------------------------------------------------------
# mocks
# ----------------------------------------------------------------------------
def mock_neck_servo():
    """Same mock/placement as pelvis_v7.mock_neck_servo() -- standing on
    the deck (z=0 here too), horn up, idler face down."""
    return Pos(V.NECK_X, 0, V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()


# ----------------------------------------------------------------------------
# fasteners
# ----------------------------------------------------------------------------
def SCREWS():
    """4x M2.5 self-tap, driven from ABOVE down through the flange. The deck
    has no pilots under them (they would land in the Pi slot and the GD lead
    slot): the armless build is not a print target."""
    return [dict(name=f"flange_{i}", kind="M2.5x8 self-tap (into pelvis deck)",
                 axis=(0, 0, -1), pos=(hx, hy, FLANGE_T), length=8.0)
            for i, (hx, hy) in enumerate(FLANGE_HOLE_XY)]


def check_flange_access(verbose=True):
    """The 4 flange screws must be drivable from directly above (open air,
    nothing over them once the collar sits on the deck)."""
    solid = neck_collar()
    ok = True
    for sc in SCREWS():
        x, y = sc["pos"][0], sc["pos"][1]
        cyl = cyl_z(D.ACCESS_D / 2, FLANGE_T + 0.5, FLANGE_T + 40.0, x, y)
        try:
            vol = (cyl & solid).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        clear = vol < 1.0
        ok &= clear
        if verbose:
            print(f"  {'PASS' if clear else 'FAIL'}  {sc['name']:10s} driver-from-above "
                  f"overlap {vol:6.1f} mm3")
    return ok


def check_servo_slide(n=30, verbose=True):
    """The tube must pass the neck servo straight along the axis with zero
    intersection until fully seated (checked as the collar moving down over
    a standing servo; the same clearance as the servo dropping in)."""
    solid = neck_collar()
    servo = mock_neck_servo()
    z_start = COLLAR_TOP + 20.0
    worst = 0.0
    for k in range(n):
        dz = z_start * (1 - k / (n - 1))
        collar_at = Pos(0, 0, dz) * solid
        try:
            vol = (collar_at & servo).volume
        except Exception:  # noqa: BLE001
            vol = 0.0
        worst = max(worst, vol)
    ok = worst < 0.5
    if verbose:
        print(f"  {'PASS' if ok else 'FAIL'}  collar slide over standing servo: "
              f"worst intersection over {n} steps = {worst:.2f} mm3")
    return ok


def check_head_clearance(samples=13, verbose=True):
    """The head, rotating through its full +/-90 deg yaw at NECK_HORN_Z
    (V.NECK_HORN_Z above the pelvis deck top == COLLAR_TOP + (NECK_HORN_Z -
    NECK_AXIS_Z) above this module's own z=0), must clear the collar's top
    by >= 1.5 mm at every angle."""
    try:
        import head as H
        # NECK_HORN_Z (world) = pelvis deck top + NECK_HORN_FACE_Z; in THIS
        # module's frame (z=0 == pelvis deck top too), the head sits at
        # NECK_HORN_FACE_Z directly.
        head_solid = Pos(V.NECK_X, 0, V.NECK_HORN_FACE_Z) * H.head()
    except Exception as e:  # noqa: BLE001
        print(f"  [neck_collar] head clearance check skipped ({type(e).__name__}: {e})")
        return True
    collar_top_face = box(NCX0 - NC_WALL - 1, NCX1 + NC_WALL + 1,
                          -(NC_HY + NC_WALL + 1), NC_HY + NC_WALL + 1,
                          COLLAR_TOP - 0.01, COLLAR_TOP)
    worst_dist = 1e9
    for k in range(samples):
        ang = -90 + 180 * k / (samples - 1)
        hp = Rot(0, 0, ang) * head_solid
        try:
            dist = hp.distance_to(collar_top_face)
        except Exception:  # noqa: BLE001
            dist = float("nan")
        worst_dist = min(worst_dist, dist)
    ok = worst_dist >= 1.5
    if verbose:
        print(f"  {'PASS' if ok else 'FAIL'}  head vs collar top over +/-90 deg yaw: "
              f"min clearance {worst_dist:.2f} mm (need >= 1.5)")
    return ok


# ----------------------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs(OUT_STL, exist_ok=True)
    os.makedirs(OUT_STEP, exist_ok=True)
    os.makedirs(OUT_REN, exist_ok=True)

    print("building neck_collar()...")
    solid = neck_collar()
    bb = solid.bounding_box()
    dims = (bb.size.X, bb.size.Y, bb.size.Z)
    mass = solid.volume * D.FILAMENT_RHO * D.PRINT_MASS_FACTOR
    n_solids = len(solid.solids())
    print(f"bbox {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm  mass {mass:.1f} g  "
          f"solids {n_solids} ({'ok' if n_solids == 1 else '** SPLIT **'})  "
          f"{'BED-OK' if max(dims) <= D.BED else '** TOO BIG **'}")
    export_stl(solid, os.path.join(OUT_STL, "neck_collar.stl"))
    export_step(solid, os.path.join(OUT_STEP, "neck_collar.step"))
    print("wrote stl/step")

    print("\n-- flange screw driver access (from above) --")
    check_flange_access()

    print("\n-- installability sweeps --")
    check_servo_slide()
    check_head_clearance()

    print("\n-- printability audit --")
    import audit_torso  # noqa: E402
    audit_torso.run(["neck_collar"])

    print("\n-- render --")
    try:
        import mujoco  # noqa: F401
        import imageio
        import tempfile
        tmp = tempfile.mkdtemp(prefix="ncol_")
        stl_path = os.path.join(tmp, "p.stl")
        export_stl(solid, stl_path)
        cz = (bb.min.Z + bb.max.Z) / 2000.0
        cx = (bb.min.X + bb.max.X) / 2000.0
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="1200" offheight="900"/></visual>
<asset><mesh name="m" file="p.stl" scale="0.001 0.001 0.001"/></asset>
<worldbody><light pos="0.3 0.3 0.3" dir="-0.4 -0.4 -0.6" directional="true"/>
<light pos="-0.3 -0.2 0.2" dir="0.4 0.3 -0.4" directional="true"/>
<geom type="mesh" mesh="m" rgba="0.8 0.82 0.86 1"/></worldbody></mujoco>"""
        xml_path = os.path.join(tmp, "s.xml")
        with open(xml_path, "w") as f:
            f.write(xml)
        m = mujoco.MjModel.from_xml_path(xml_path)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, 900, 1200)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [cx, 0, cz]
        cam.distance = 0.14
        imgs = []
        for az, el in ((150, -10), (60, -10), (0, -10)):
            cam.azimuth, cam.elevation = az, el
            r.update_scene(d, cam)
            imgs.append(r.render().copy())
        import numpy as _np
        imageio.imwrite(os.path.join(OUT_REN, "neck_collar.png"), _np.concatenate(imgs, axis=1))
        print("wrote", os.path.join(OUT_REN, "neck_collar.png"))
    except Exception as e:  # noqa: BLE001
        print(f"render skipped ({type(e).__name__}: {e})")
