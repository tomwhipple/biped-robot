"""v6 assembly: the articulated robot as a kinematic chain of build123d solids.

One table (CHAIN) places every printed part, servo mock and electronics mock
in the STANDING pose from cad/v6/dimensions_v6.py, and `robot(pose)` poses it
through the joint angles (product of rotations about each joint's standing-
pose axis, root first). Everything downstream reads this table: the STEP
export, the pose renders, the ROM interference check (check_assembly_v6.py)
and the fly-in animation (animate_v6.py) -- so a figure cannot show a
placement the checker never tried.

Parts come from the v6 part modules when they exist (leg_link_v6, ankle_link,
foot_v6, pelvis_v7, head) and from v5 for the unchanged hip stack
(yaw_carrier, yoke_roll, yoke_pitch); a missing v6 module falls back to a
labelled envelope box so the chain, the export and the checks run while the
parts are still being drawn.

    .venv/bin/python cad/v6/assembly_v6.py                 # step/assembly_v6.step + renders
    .venv/bin/python cad/v6/assembly_v6.py --pose knee=-60,hip_pitch=-30
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

os.environ.setdefault("MUJOCO_GL", "egl")   # before anything imports mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import (Box, Color, Compound, Cylinder, Location, Pos, Rot,  # noqa: E402
                       export_step, export_stl)
import dimensions_v6 as V  # noqa: E402
D = V.D
import check_assembly as CA  # noqa: E402  (v5 servo mocks)
import parts as v5parts  # noqa: E402

OUT_STEP = os.path.join(HERE, "step")
OUT_STL = os.path.join(HERE, "stl")
OUT_REN = os.path.join(HERE, "renders")

COL_PRINT = Color(0.82, 0.84, 0.87)
COL_SERVO = Color(0.22, 0.23, 0.27)
COL_FOOT = Color(0.30, 0.31, 0.34)
COL_MOCK = Color(0.15, 0.45, 0.75)
COL_HEAD = Color(0.90, 0.88, 0.80)

JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")


# --------------------------------------------------------------------------- part sources
def _try(modname, fn, *a, **k):
    try:
        mod = __import__(modname)
        return getattr(mod, fn)(*a, **k), True
    except Exception as e:  # noqa: BLE001 -- placeholder while the part is being built
        print(f"  [assembly_v6] {modname}.{fn} unavailable ({type(e).__name__}: {str(e)[:60]}); envelope box used")
        return None, False


def part_leg_link():
    s, ok = _try("leg_link_v6", "leg_link_v6")
    if ok:
        return s
    # envelope: web/plates box from the grip to the pads
    return Pos(0, 1.0, -V.LL_DROP / 2) * Box(30, 44, V.LL_DROP + 20)


def part_ankle_link():
    s, ok = _try("ankle_link", "ankle_link")
    if ok:
        return s
    return Pos(0, 1.0, -V.AL_DROP / 2) * Box(44, 44, V.AL_DROP + 20)


def part_foot(side):
    s, ok = _try("foot_v6", "foot", side)
    if ok:
        return s
    sgn = 1 if side == "L" else -1
    return Pos(V.FOOT_TOE - V.FOOT_L_LEN / 2, sgn * V.FOOT_Y_OFF, V.FOOT_PLATE_T / 2) * Box(V.FOOT_L_LEN, V.FOOT_W, V.FOOT_PLATE_T)


def part_pelvis():
    s, ok = _try("pelvis_v7", "pelvis_v7")
    if ok:
        return s
    x0, x1 = V.HOUSING_X
    return Pos((x0 + x1) / 2, 0, V.YAW_BOX_BOT_Z / 2) * Box(x1 - x0, 2 * V.HOUSING_HW, -V.YAW_BOX_BOT_Z)


def part_collar():
    s, ok = _try("neck_collar", "neck_collar")
    if ok:
        return s
    return Pos(-12.5, 0, 16) * Box(50, 30, 32)


def part_head():
    s, ok = _try("head", "head")
    if ok:
        return s
    return Pos(0, 0, V.HEAD_H / 2 + V.HEAD_BASE_T) * Box(V.HEAD_D, V.HEAD_W, V.HEAD_H)


def mock_pack():
    return Pos((V.BATT_X[0] + V.BATT_X[1]) / 2, 0, (V.BATT_Z[0] + V.BATT_Z[1]) / 2) * Box(V.BATT[1], V.BATT[0], V.BATT[2])


def mock_pi():
    return Pos((V.PI_PCB_X0 + V.PI_PCB_X1) / 2, 0, V.PI_CZ) * Box(V.PI_PCB_T, V.PI_OUTLINE[0], V.PI_OUTLINE[1])


def mock_gd():
    return Pos((V.GD_PCB_X0 + V.GD_PCB_X1) / 2, 0, (V.GD_BOT_Z + V.GD_TOP_Z) / 2) * Box(D.BR_PCB_T, D.BOARD_GD_OUTLINE[0], D.BOARD_GD_OUTLINE[1])


def mock_roll_servo_in_foot(side):
    """roll servo lying across the foot: output axis +X (horn forward), case
    length along +y OUTBOARD (cable end), width vertical. Built from v5's
    axis-X mock (case length along Z in the carrier) rolled about X."""
    sgn = 1 if side == "L" else -1
    m = CA.servo_mock_x()
    bb = m.bounding_box()
    # v5 mock_x: case length runs along -z? measure and roll so the LONG side goes to sgn*+y
    long_down = abs(bb.min.Z) > abs(bb.max.Z)
    # Rot(+90 about X) maps -z -> +y; the long side must end up on sgn*+y
    ang = 90 if (long_down == (sgn > 0)) else -90
    out = Rot(ang, 0, 0) * m
    bb2 = out.bounding_box()
    assert (bb2.max.Y if sgn > 0 else -bb2.min.Y) > 30.0, "roll servo mock: long side not outboard"
    return out


# --------------------------------------------------------------------------- chain
_CACHE = {}


def cached(key, fn):
    """standing-pose solids are built once per process; posing is a cheap
    Location product on top (the ROM checker poses hundreds of times)."""
    if key not in _CACHE:
        _CACHE[key] = fn()
    return _CACHE[key]


def leg_chain(side):
    return cached(("leg", side), lambda: _leg_chain(side))


def _leg_chain(side):
    """[(link_index, label, colour, world_solid_in_standing_pose)] for one leg,
    plus the joint list [(name, axis_unit, point)] in the standing pose."""
    y = V.HIP_SEP / 2 * (1 if side == "L" else -1)
    at = lambda z: Pos(0, y, z)
    joints = [("hip_yaw", (0, 0, 1), (0, y, V.HIP_YAW_Z)),
              ("hip_roll", (1, 0, 0), (0, y, V.HIP_ROLL_Z)),
              ("hip_pitch", (0, 1, 0), (0, y, V.HIP_PITCH_Z)),
              ("knee", (0, 1, 0), (0, y, V.KNEE_Z)),
              ("ankle_pitch", (0, 1, 0), (0, y, V.ANKLE_PITCH_Z)),
              ("ankle_roll", (1, 0, 0), (0, y, V.ANKLE_ROLL_Z))]
    pieces = [
        (0, f"servo_hip_yaw_{side}", COL_SERVO, Pos(0, y, V.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()),
        (1, f"yaw_carrier_{side}", COL_PRINT, Pos(0, y, V.HIP_YAW_Z) * v5parts.yaw_carrier()),
        (1, f"servo_hip_roll_{side}", COL_SERVO, at(V.HIP_ROLL_Z) * CA.servo_mock_x()),
        (2, f"yoke_roll_{side}", COL_PRINT, at(V.HIP_ROLL_Z) * v5parts.yoke_roll()),
        (2, f"yoke_pitch_{side}", COL_PRINT, at(V.HIP_PITCH_Z) * v5parts.yoke_pitch()),
        (3, f"servo_hip_pitch_{side}", COL_SERVO, at(V.HIP_PITCH_Z) * CA.servo_mock_y()),
        (3, f"thigh_{side}", COL_PRINT, at(V.HIP_PITCH_Z) * part_leg_link()),
        (4, f"servo_knee_{side}", COL_SERVO, at(V.KNEE_Z) * CA.servo_mock_y()),
        (4, f"shin_{side}", COL_PRINT, at(V.KNEE_Z) * part_leg_link()),
        (5, f"servo_ankle_pitch_{side}", COL_SERVO, at(V.ANKLE_PITCH_Z) * CA.servo_mock_y()),
        (5, f"ankle_link_{side}", COL_PRINT, at(V.ANKLE_PITCH_Z) * part_ankle_link()),
        (6, f"servo_ankle_roll_{side}", COL_SERVO, at(V.ANKLE_ROLL_Z) * mock_roll_servo_in_foot(side)),
        (6, f"foot_{side}", COL_FOOT, at(V.TPU_SOLE_T) * part_foot(side)),
    ]
    return pieces, joints


def torso_pieces():
    return cached("torso", _torso_pieces)


def _torso_pieces():
    z = V.DECK_TOP_Z
    return [
        (0, "pelvis_v7", COL_PRINT, Pos(0, 0, z) * part_pelvis()),
        (0, "pack_mock", COL_MOCK, Pos(0, 0, z) * mock_pack()),
        (0, "pi4_mock", COL_MOCK, Pos(0, 0, z) * mock_pi()),
        (0, "gd_mock", COL_MOCK, Pos(0, 0, z) * mock_gd()),
        (0, "servo_neck", COL_SERVO, Pos(V.NECK_X, 0, z + V.NECK_AXIS_Z) * Rot(180, 0, 0) * CA.servo_mock_z()),
        (0, "neck_collar", COL_PRINT, Pos(V.NECK_X, 0, z) * part_collar()),
    ]


def head_pieces():
    return cached("head", lambda: [(1, "head", COL_HEAD, Pos(V.NECK_X, 0, V.NECK_HORN_Z) * part_head())])


def rot_about(axis, point, deg):
    """Location: rotation of `deg` about the unit axis through `point` (world)."""
    ax = tuple(axis)
    rot = Rot(ax[0] * deg, ax[1] * deg, ax[2] * deg)   # axis-aligned axes only
    return Pos(*point) * rot * Pos(-point[0], -point[1], -point[2])


def pose_transforms(joints, angles):
    """cumulative world Location per link index (0 = root, k = after joint k)."""
    T = [Location()]
    for (name, axis, point), a in zip(joints, angles):
        T.append(T[-1] * rot_about(axis, point, a))
    return T


def robot(pose=None):
    """Compound of every piece at the given pose. pose: dict of joint name ->
    deg, keys like 'L_knee', 'knee' (both legs), 'neck'."""
    pose = pose or {}
    children = []
    for idx, label, col, solid in torso_pieces():
        s = solid
        s.label, s.color = label, col
        children.append(s)
    neck = pose.get("neck", 0.0)
    Tn = rot_about((0, 0, 1), (V.NECK_X, 0, V.NECK_HORN_Z), neck)
    for idx, label, col, solid in head_pieces():
        s = Tn * solid
        s.label, s.color = label, col
        children.append(s)
    for side in ("L", "R"):
        pieces, joints = leg_chain(side)
        angles = [pose.get(f"{side}_{j}", pose.get(j, 0.0)) for j in JOINTS]
        T = pose_transforms(joints, angles)
        for idx, label, col, solid in pieces:
            s = T[idx] * solid
            s.label, s.color = label, col
            children.append(s)
    return Compound(label="bimo_v6", children=children)


def parse_pose(txt):
    pose = {}
    for kv in (txt or "").split(","):
        if kv.strip():
            k, v = kv.split("=")
            pose[k.strip()] = float(v)
    return pose


# --------------------------------------------------------------------------- render (MuJoCo, from STLs)
def render(comp: Compound, png_path, views=((150, -12), (90, -8), (0, -12)), size=(640, 800)):
    import mujoco
    import numpy as np
    import imageio
    os.environ.setdefault("MUJOCO_GL", "egl")
    tmp = tempfile.mkdtemp(prefix="v6asm_")
    geoms = []
    for i, ch in enumerate(comp.children):
        path = os.path.join(tmp, f"p{i}.stl")
        export_stl(ch, path)
        c = ch.color
        try:
            rgba = f"{c.red:.3f} {c.green:.3f} {c.blue:.3f} 1" if c else "0.8 0.8 0.8 1"
        except AttributeError:
            try:
                t = tuple(c)
                rgba = f"{t[0]:.3f} {t[1]:.3f} {t[2]:.3f} 1"
            except Exception:  # noqa: BLE001
                rgba = "0.8 0.8 0.8 1"
        geoms.append((f"p{i}", path, rgba))
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1600" offheight="1600"/><headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4"/></visual>',
           '<asset><texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.92" rgb2="0.8 0.8 0.84" width="512" height="512"/>',
           '<material name="floor" texture="grid" texrepeat="10 10" texuniform="true"/>']
    xml += [f'<mesh name="{n}" file="{os.path.basename(p)}" scale="0.001 0.001 0.001"/>' for n, p, _ in geoms]
    xml += ['</asset><worldbody><light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>',
            '<geom type="plane" size="2 2 0.1" material="floor"/>']
    xml += [f'<geom type="mesh" mesh="{n}" rgba="{rgba}"/>' for n, _, rgba in geoms]
    xml += ['</worldbody></mujoco>']
    with open(os.path.join(tmp, "asm.xml"), "w") as f:
        f.write("\n".join(xml))
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "asm.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, V.TOP_Z / 2000.0]
    cam.distance = 1.6
    imgs = []
    for az, el in views:
        cam.azimuth, cam.elevation = az, el
        r.update_scene(d, cam)
        imgs.append(r.render().copy())
    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    imageio.imwrite(png_path, np.concatenate(imgs, axis=1))
    return png_path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="")
    ap.add_argument("--no-step", action="store_true")
    ap.add_argument("--png", default=os.path.join(OUT_REN, "assembly_v6.png"))
    a = ap.parse_args(argv)
    comp = robot(parse_pose(a.pose))
    bb = comp.bounding_box()
    print(f"assembly bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}  z {bb.min.Z:.1f}..{bb.max.Z:.1f} mm, {len(comp.children)} pieces")
    if not a.no_step:
        os.makedirs(OUT_STEP, exist_ok=True)
        export_step(comp, os.path.join(OUT_STEP, "assembly_v6.step"))
        print("wrote", os.path.join(OUT_STEP, "assembly_v6.step"))
    print("wrote", render(comp, a.png))


if __name__ == "__main__":
    main()
