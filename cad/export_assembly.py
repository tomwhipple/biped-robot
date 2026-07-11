"""Export the full robot as one assembled STEP (neutral standing pose).

Run:  .venv/bin/python cad/export_assembly.py   -> cad/step/assembly.step

Printed parts + mock STS3215 servo bodies (from check_assembly) placed with the
same transforms the interference checks use, in world frame: ground z=0,
+X forward, legs at y = +/-HIP_SEP/2. Open in FreeCAD/Onshape -- parts arrive
as a labeled tree (pelvis, tower, leg_L/*, leg_R/*).
"""
import os
from build123d import Pos, Rot, Compound, Color, export_step
import dimensions as D
import parts
import check_assembly as CA

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")

DECK_TOP_Z = D.DECK_BOT_Z + D.DECK_T          # pelvis/tower local z=0 lives here

COL_PRINT = Color(0.82, 0.84, 0.87)
COL_SERVO = Color(0.25, 0.26, 0.30)
COL_FOOT = Color(0.70, 0.72, 0.78)
COL_CAM = Color(0.10, 0.10, 0.12)

TOWER_TOP_Z = DECK_TOP_Z + D.TOWER_H


def camera_mock():
    """GoPro MAX 360 stand-in: body box + two folded mount fingers reaching
    into the gopro_base slots (bottom of body ~6 above the M5 hole center)."""
    hole_z = D.GP_BASE_T + D.GP_HOLE_H               # above the tower top
    bot = hole_z + 6.0
    dx, dy, dz = D.CAM_BODY
    body = parts.box(-dx / 2, dx / 2, -dy / 2, dy / 2, bot, bot + dz)
    fingers = None
    for cy in (-(D.GP_PRONG_T + D.GP_SLOT) / 2, (D.GP_PRONG_T + D.GP_SLOT) / 2):
        f = parts.box(-6, 6, cy - 1.45, cy + 1.45, hole_z - 6, bot + 1)
        fingers = f if fingers is None else fingers + f
    return body + fingers


def piece(label, color, solid):
    solid.label = label
    solid.color = color
    return solid


def leg(y, tag):
    """One leg at lateral offset y. Same parts both sides (translations, not
    mirrors; every pitch horn faces +Y)."""
    at = lambda z: Pos(0, y, z)
    return Compound(label=f"leg_{tag}", children=[
        piece("servo_hip_roll", COL_SERVO, at(D.HIP_ROLL_Z) * CA.servo_mock_x()),
        piece("yoke_roll", COL_PRINT, at(D.HIP_ROLL_Z) * parts.yoke_roll()),
        piece("yoke_pitch", COL_PRINT, at(D.HIP_PITCH_Z) * parts.yoke_pitch()),
        piece("servo_hip_pitch", COL_SERVO, at(D.HIP_PITCH_Z) * CA.servo_mock_y()),
        piece("link_thigh", COL_PRINT, at(D.HIP_PITCH_Z) * parts.leg_link()),
        piece("servo_knee", COL_SERVO, at(D.KNEE_Z) * CA.servo_mock_y()),
        piece("link_shin", COL_PRINT, at(D.KNEE_Z) * parts.leg_link()),
        piece("servo_ankle", COL_SERVO,
              at(D.ANKLE_Z) * Rot(0, 90, 0) * CA.servo_mock_y()),
        piece("foot", COL_FOOT, at(D.TPU_PROUD) * parts.foot()),
    ])


robot = Compound(label="bimo_biped", children=[
    piece("pelvis", COL_PRINT, Pos(0, 0, DECK_TOP_Z) * parts.pelvis()),
    piece("tower", COL_PRINT, Pos(0, 0, DECK_TOP_Z) * parts.tower()),
    piece("gopro_base", COL_PRINT, Pos(0, 0, TOWER_TOP_Z) * parts.gopro_base()),
    piece("camera_gopro_max_mock", COL_CAM, Pos(0, 0, TOWER_TOP_Z) * camera_mock()),
    leg(D.HIP_SEP / 2, "L"),
    leg(-D.HIP_SEP / 2, "R"),
])

os.makedirs(OUT, exist_ok=True)
path = os.path.join(OUT, "assembly.step")
export_step(robot, path)
bb = robot.bounding_box()
print(f"assembly -> {path}")
cam_top = D.TOP_Z + D.GP_BASE_T + D.GP_HOLE_H + 6 + D.CAM_BODY[2]
print(f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}"
      f"  z {bb.min.Z:.1f}..{bb.max.Z:.1f}  "
      f"(expect ~0..{D.TOP_Z:.0f} structure, ~{cam_top:.0f} incl. camera)")
