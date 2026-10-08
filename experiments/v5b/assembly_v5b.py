"""v5b assembly: the printed v5 torso and legs with the v6 ankle under each shin.

Per leg, the v5 foot comes off and the v6 ankle stack goes on: `ankle_link`
(grips the ankle-pitch servo, which moves out of the v5 foot), an ankle-roll
STS3215, `foot_L`/`foot_R` and the silicone sole. The v5 shin's lower fork and
`leg_link_v6`'s are built from the same pad constants (cad/dimensions.py), so
the stack bolts on unchanged. Everything above the ankle-pitch axis is v5,
from `v5_cad` (pinned to the commit the parts were printed from).

The ankle-pitch servo's case turns 90 deg relative to the shin: the v5 foot
held it lying back along -X, `ankle_link` holds it hanging along -Z. Its
encoder zero moves with it -- see README.md, calibration.

World frame: floor at z = 0 (sole bottom), +X forward, legs at
y = +-HIP_SEP/2 (v5: 66 mm). Same chain conventions as cad/v6/assembly_v6.py
(link index per piece, `robot(pose)` with pose keys like 'L_knee' or 'knee').

    .venv/bin/python experiments/v5b/assembly_v5b.py               # step + render
    .venv/bin/python experiments/v5b/assembly_v5b.py --pose L_hip_roll=10,L_ankle_roll=-10
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

if sys.platform == "darwin":
    os.environ.setdefault("MUJOCO_GL", "cgl")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path[:0] = [HERE, os.path.join(REPO, "cad", "v6"), os.path.join(REPO, "cad")]

from build123d import Color, Compound, Pos, export_step, export_stl  # noqa: E402
import v5_cad  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import assembly_v6 as A6  # noqa: E402
import check_assembly as CA  # noqa: E402  (current servo mocks: the servo truth)

v5 = v5_cad.load()
D5 = v5.D

OUT_STEP = os.path.join(HERE, "step")
OUT_FIG = os.path.join(HERE, "figs")

COL_PRINT, COL_SERVO, COL_FOOT, COL_SOLE = A6.COL_PRINT, A6.COL_SERVO, A6.COL_FOOT, A6.COL_SOLE
COL_V6 = Color(0.95, 0.55, 0.15)          # the new prints, orange in the figures
COL_BATT = Color(0.16, 0.30, 0.55)
COL_PCB = Color(0.05, 0.32, 0.18)

JOINTS = A6.JOINTS                        # hip_yaw .. ankle_roll

# v5 sits higher by the difference between the two ankle-pitch axis heights
LIFT = V.ANKLE_PITCH_Z - D5.ANKLE_Z       # 76.36 - 17.96 = 58.40
HIP_SEP = D5.HIP_SEP
HIP_YAW_Z = D5.HIP_YAW_Z + LIFT
HIP_ROLL_Z = D5.HIP_ROLL_Z + LIFT
HIP_PITCH_Z = D5.HIP_PITCH_Z + LIFT
KNEE_Z = D5.KNEE_Z + LIFT
ANKLE_PITCH_Z = V.ANKLE_PITCH_Z
ANKLE_ROLL_Z = V.ANKLE_ROLL_Z
DECK_TOP_Z = D5.HIP_ROLL_Z + D5.ROLL_BELOW_DECK_YAW + LIFT
assert abs(D5.KNEE_Z - D5.LINK_DROP - D5.ANKLE_Z) < 1e-6, "v5 shin length"

# hips and knee: the v5 robot's; ankle: the v6 stack's
ROM = {**{k: D5.ROM[k] for k in ("hip_yaw", "hip_roll", "hip_pitch", "knee")},
       "ankle_pitch": V.ROM["ankle_pitch"], "ankle_roll": V.ROM["ankle_roll"]}

_CACHE = {}


def cached(key, fn):
    if key not in _CACHE:
        _CACHE[key] = fn()
    return _CACHE[key]


def v5_part(name):
    return cached(("v5", name), getattr(v5.parts, name))


def battery_mock():
    """v5's 3S 850 pack in its bay (cad/export_assembly.py's battery_mock),
    pelvis frame: it lies in the V of the two seat chamfers."""
    ly, wx, hz = D5.BATT_PACK
    xc = (D5.BT_SEAT_X + D5.BT_WALL_X1) / 2
    z0 = D5.BT_SEAT_Z - (D5.BT_SEAT_BAY_W - wx) / 2 * math.tan(math.radians(D5.BT_SEAT_DEG))
    return v5.parts.box(xc - wx / 2, xc + wx / 2, -ly / 2, ly / 2, z0, z0 + hz)


def leg_chain(side):
    return cached(("leg", side), lambda: _leg_chain(side))


def _leg_chain(side):
    """[(link_index, label, colour, world solid in the standing pose)] and the
    joints [(name, axis, point)] for one leg."""
    y = HIP_SEP / 2 * (1 if side == "L" else -1)
    at = lambda z: Pos(0, y, z)
    joints = [("hip_yaw", (0, 0, 1), (0, y, HIP_YAW_Z)),
              ("hip_roll", (1, 0, 0), (0, y, HIP_ROLL_Z)),
              ("hip_pitch", (0, 1, 0), (0, y, HIP_PITCH_Z)),
              ("knee", (0, 1, 0), (0, y, KNEE_Z)),
              ("ankle_pitch", (0, 1, 0), (0, y, ANKLE_PITCH_Z)),
              ("ankle_roll", (1, 0, 0), (0, y, ANKLE_ROLL_Z))]
    pieces = [
        (0, f"servo_hip_yaw_{side}", COL_SERVO, Pos(0, y, HIP_YAW_Z + D5.SV_HORN_FACE) * CA.servo_mock_z()),
        (1, f"yaw_carrier_{side}", COL_PRINT, Pos(0, y, HIP_YAW_Z) * v5_part("yaw_carrier")),
        (1, f"servo_hip_roll_{side}", COL_SERVO, at(HIP_ROLL_Z) * CA.servo_mock_x()),
        (2, f"yoke_roll_{side}", COL_PRINT, at(HIP_ROLL_Z) * v5_part("yoke_roll")),
        (2, f"yoke_pitch_{side}", COL_PRINT, at(HIP_PITCH_Z) * v5_part("yoke_pitch")),
        (3, f"servo_hip_pitch_{side}", COL_SERVO, at(HIP_PITCH_Z) * CA.servo_mock_y()),
        (3, f"thigh_{side}", COL_PRINT, at(HIP_PITCH_Z) * v5_part("leg_link")),
        (4, f"servo_knee_{side}", COL_SERVO, at(KNEE_Z) * CA.servo_mock_y()),
        (4, f"shin_{side}", COL_PRINT, at(KNEE_Z) * v5_part("leg_link")),
        (5, f"servo_ankle_pitch_{side}", COL_SERVO, at(ANKLE_PITCH_Z) * CA.servo_mock_y()),
        (5, f"ankle_link_{side}", COL_V6, at(ANKLE_PITCH_Z) * cached("ankle_link", A6.part_ankle_link)),
        (6, f"servo_ankle_roll_{side}", COL_SERVO, at(ANKLE_ROLL_Z) * A6.mock_roll_servo_in_foot(side)),
        (6, f"foot_{side}", COL_V6, at(V.TPU_SOLE_T) * cached(("foot", side), lambda: A6.part_foot(side))),
        (6, f"sole_{side}", COL_SOLE, at(V.TPU_SOLE_T) * cached(("sole", side), lambda: A6.part_sole(side))),
    ]
    return pieces, joints


def torso_pieces():
    return cached("torso", lambda: [
        (0, "pelvis", COL_PRINT, Pos(0, 0, DECK_TOP_Z) * v5_part("pelvis")),
        (0, "battery_3s_850_mock", COL_BATT, Pos(0, 0, DECK_TOP_Z) * battery_mock()),
        (0, "board_pcb_mock", COL_PCB, Pos(0, 0, DECK_TOP_Z) * v5.check_assembly.board_pcb_mock()),
    ])


def angles_for(side, pose):
    return [pose.get(f"{side}_{j}", pose.get(j, 0.0)) for j in JOINTS]


def leg_pieces(side, pose, labels=None):
    """{label: posed solid} for one leg, optionally only `labels`."""
    pieces, joints = leg_chain(side)
    T = A6.pose_transforms(joints, angles_for(side, pose))
    return {lab: T[i] * s for i, lab, _, s in pieces if labels is None or lab in labels}


def robot(pose=None):
    pose = pose or {}
    children = []
    for _, label, col, solid in torso_pieces():
        s = Pos(0, 0, 0) * solid
        s.label, s.color = label, col
        children.append(s)
    for side in ("L", "R"):
        pieces, joints = leg_chain(side)
        T = A6.pose_transforms(joints, angles_for(side, pose))
        for i, label, col, solid in pieces:
            s = T[i] * solid
            s.label, s.color = label, col
            children.append(s)
    return Compound(label="v5b", children=children)


def render(comp, png_path, views=((150, -12), (90, -6), (0, -10)), size=(560, 720),
           lookat=(0, 0, 0.2), distance=0.8):
    """One PNG strip of `comp` from `views` (azimuth, elevation), MuJoCo from STLs."""
    import imageio
    import mujoco
    import numpy as np
    tmp = tempfile.mkdtemp(prefix="v5b_")
    xml = ['<mujoco><compiler meshdir="."/><visual><global offwidth="1600" offheight="1600"/>'
           '<headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4"/></visual><asset>'
           '<texture name="grid" type="2d" builtin="checker" rgb1="0.9 0.9 0.92" rgb2="0.8 0.8 0.84" width="512" height="512"/>'
           '<material name="floor" texture="grid" texrepeat="10 10" texuniform="true"/>']
    geoms = []
    for i, ch in enumerate(comp.children):
        export_stl(ch, os.path.join(tmp, f"p{i}.stl"))
        rgba = " ".join(f"{t:.3f}" for t in tuple(ch.color)[:3])
        geoms.append(f'<geom type="mesh" mesh="p{i}" rgba="{rgba} 1"/>')
        xml.append(f'<mesh name="p{i}" file="p{i}.stl" scale="0.001 0.001 0.001"/>')
    xml += ['</asset><worldbody><light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>',
            '<geom type="plane" size="2 2 0.1" material="floor"/>', *geoms, '</worldbody></mujoco>']
    with open(os.path.join(tmp, "asm.xml"), "w") as f:
        f.write("\n".join(xml))
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "asm.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = list(lookat)
    cam.distance = distance
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
    ap.add_argument("--png", default=os.path.join(OUT_FIG, "assembly_v5b.png"))
    ap.add_argument("--closeup", default=os.path.join(OUT_FIG, "assembly_v5b_ankle.png"),
                    help="the left leg below the knee, at the same pose")
    ap.add_argument("--step", default=os.path.join(OUT_STEP, "assembly_v5b.step"))
    a = ap.parse_args(argv)
    comp = robot(A6.parse_pose(a.pose))
    bb = comp.bounding_box()
    print(f"v5b: lift {LIFT:.2f} mm; hip yaw {HIP_YAW_Z:.2f}, roll {HIP_ROLL_Z:.2f}, pitch {HIP_PITCH_Z:.2f}, "
          f"knee {KNEE_Z:.2f}, ankle pitch {ANKLE_PITCH_Z:.2f}, ankle roll {ANKLE_ROLL_Z:.2f} mm")
    print(f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}  "
          f"z {bb.min.Z:.1f}..{bb.max.Z:.1f} mm, {len(comp.children)} pieces")
    if not a.no_step:
        os.makedirs(os.path.dirname(a.step), exist_ok=True)
        export_step(comp, a.step)
        print("wrote", a.step)
    if os.environ.get("MUJOCO_GL") in ("cgl", "egl", "osmesa", "glfw"):
        print("wrote", render(comp, a.png))
        y = HIP_SEP / 2e3
        lower = Compound(children=[c for c in comp.children
                                   if c.label.endswith("_L") and c.label.split("_")[0] in
                                   ("shin", "servo", "ankle", "foot", "sole")
                                   and not c.label.startswith(("servo_hip", "servo_knee"))])
        print("wrote", render(lower, a.closeup, views=((150, -15), (90, -5), (35, -20)),
                              lookat=(0.005, y, 0.07), distance=0.36))


if __name__ == "__main__":
    main()
