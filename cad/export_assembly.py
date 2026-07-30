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
import fasteners as F
import check_assembly as CA

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")

# v3yaw stack: the yaw servo + carrier are inserted BETWEEN the hip-roll axis
# and the deck, so everything from the deck up rises by YAW_STACK_DROP. The deck
# top (pelvis/tower local z=0) sits ROLL_BELOW_DECK_YAW above the roll axis.
DECK_TOP_Z = D.HIP_ROLL_Z + D.ROLL_BELOW_DECK_YAW   # 327.77 world = deck top

COL_PRINT = Color(0.82, 0.84, 0.87)
COL_SERVO = Color(0.25, 0.26, 0.30)
COL_FOOT = Color(0.70, 0.72, 0.78)
COL_CAM = Color(0.10, 0.10, 0.12)
COL_BATT = Color(0.16, 0.30, 0.55)
COL_STEEL = Color(0.55, 0.57, 0.62)     # fasteners (fasteners.py solids)

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


def battery_mock():
    """The modeled 3S 850 pack (D.BATT_PACK) seated on the deck: outer face at
    BATT_SEAT_X, inner face toward the far-wall rails. Follows BATT_PACK rather
    than the BATT envelope, so the fly-in proves the real pack inserts."""
    ly, wx, hz = D.BATT_PACK
    x0 = D.BATT_SEAT_X + (D.BATT[1] - wx)
    return parts.box(x0, x0 + wx, -ly / 2, ly / 2, 0, hz)


def piece(label, color, solid):
    solid.label = label
    solid.color = color
    return solid


COL_PATH = Color(0.93, 0.46, 0.10)      # insertion sweeps (docs figures only)


def fastener_frames(y):
    """(name, placement, fastener groups) for one leg -- ONE table feeding BOTH
    the seated screw solids and their INSERTION SWEEPS (fasteners.paths). The
    assembly figures draw the sweeps from this and check_assembly checks the
    same groups, so a doc figure cannot show an approach the checker never
    tried. Groups are fasteners.SEATS keys."""
    at = lambda z: Pos(0, y, z)
    return [("yaw_stack", Pos(0, y, D.HIP_YAW_Z),
             ("screws_yaw_horn", "screws_yaw_wall")),
            ("hip_roll", at(D.HIP_ROLL_Z), ("screws_disc_x",)),
            ("flange", at(D.HIP_ROLL_Z), ("screws_flange",)),
            ("hip_pitch", at(D.HIP_PITCH_Z), ("screws_disc_y",)),
            ("thigh_grip", at(D.HIP_PITCH_Z), ("screws_grip",)),
            ("knee", at(D.KNEE_Z), ("screws_disc_y",)),
            ("shin_grip", at(D.KNEE_Z), ("screws_grip",)),
            ("ankle", at(D.ANKLE_Z), ("screws_disc_y",)),
            ("foot", at(D.TPU_PROUD), ("screws_foot",))]


TORSO_FRAMES = [("deck", Pos(0, 0, DECK_TOP_Z), ("screws_deck",)),
                ("tower", Pos(0, 0, DECK_TOP_Z), ("screws_tower",)),
                ("head_stack", Pos(0, 0, TOWER_TOP_Z), ("screws_head_stack",))]


def _fuse(solids):
    out = None
    for s in solids:
        out = s if out is None else out + s
    return out


def fasteners(frames, paths=False):
    """The screw pieces (or, with paths=True, their insertion sweeps) for a
    frame table."""
    return [piece(("paths_" if paths else "screws_") + name,
                  COL_PATH if paths else COL_STEEL,
                  frame * _fuse([F.paths(g) if paths else F.SEATS[g][1]()
                                 for g in groups]))
            for name, frame, groups in frames]


def insertion_paths():
    """Every screw's approach volume, placed in world exactly where its screw
    is. NOT part of `robot` -- it is drawing, not hardware; docs figures
    (render_assembly_steps) ask for it explicitly."""
    return Compound(label="insertion_paths", children=(
        fasteners(TORSO_FRAMES, paths=True)
        + [Compound(label=f"leg_{tag}",
                    children=fasteners(fastener_frames(sy), paths=True))
           for sy, tag in ((D.HIP_SEP / 2, "L"), (-D.HIP_SEP / 2, "R"))]))


def leg(y, tag):
    """One leg at lateral offset y. Same parts both sides (translations, not
    mirrors; every pitch horn faces +Y)."""
    at = lambda z: Pos(0, y, z)
    return Compound(label=f"leg_{tag}", children=[
        # v3yaw hip-yaw stack: the yaw servo lies flat under the deck (horn DOWN,
        # vertical output axis) and the carrier bolts to its horn and holds the
        # roll bay. Yaw horn face == carrier local z=0 at HIP_YAW_Z; the roll
        # servo (below) rides in the carrier bay at HIP_ROLL_Z, unchanged.
        piece("servo_hip_yaw", COL_SERVO,
              Pos(0, y, D.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()),
        piece("yaw_carrier", COL_PRINT, Pos(0, y, D.HIP_YAW_Z) * parts.yaw_carrier()),
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
        # fasteners (fasteners.py; same frames as the parts they ride with) --
        # from fastener_frames(), which also drives the insertion sweeps
    ] + fasteners(fastener_frames(y)))


robot = Compound(label="bimo_biped", children=[
    piece("pelvis", COL_PRINT, Pos(0, 0, DECK_TOP_Z) * parts.pelvis()),
    piece("tower", COL_PRINT, Pos(0, 0, DECK_TOP_Z) * parts.tower()),
    piece("battery_3s_mock", COL_BATT, Pos(0, 0, DECK_TOP_Z) * battery_mock()),
    piece("imu_carrier", COL_PRINT, Pos(0, 0, TOWER_TOP_Z) * parts.imu_carrier()),
    piece("imu_bno055_mock", COL_CAM,
          Pos(0, D.IMU_CY, TOWER_TOP_Z + D.IMU_PCB_Z)
          * parts.box(-D.IMU_PCB[0] / 2, D.IMU_PCB[0] / 2, -D.IMU_PCB[1] / 2,
                      D.IMU_PCB[1] / 2, 0, D.IMU_PCB[2])),
    # gopro stack rides IMU_CARRIER_T higher: the carrier is under the base
    piece("gopro_base", COL_PRINT,
          Pos(0, 0, TOWER_TOP_Z + D.IMU_CARRIER_T) * parts.gopro_base()),
    piece("camera_gopro_max_mock", COL_CAM,
          Pos(0, 0, TOWER_TOP_Z + D.IMU_CARRIER_T) * camera_mock()),
] + fasteners(TORSO_FRAMES) + [
    leg(D.HIP_SEP / 2, "L"),
    leg(-D.HIP_SEP / 2, "R"),
])

def main():
    """Write cad/step/assembly.step.

    Behind a main() since 2026-07-27: this used to run at IMPORT time, so every
    render script that only wanted `piece` (render_electronics_steps,
    render_exploded_drawing) silently paid a ~17 s assembly export just to get
    at a helper. `robot` stays module-level -- those scripts do read it.
    """
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "assembly.step")
    export_step(robot, path)
    bb = robot.bounding_box()
    print(f"assembly -> {path}")
    cam_top = (D.TOP_Z_YAW + D.IMU_CARRIER_T + D.GP_BASE_T + D.GP_HOLE_H + 6
               + D.CAM_BODY[2])
    print(f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}"
          f"  z {bb.min.Z:.1f}..{bb.max.Z:.1f}  "
          f"(expect ~0..{D.TOP_Z_YAW:.0f} structure, ~{cam_top:.0f} incl. camera)")


if __name__ == "__main__":
    main()
