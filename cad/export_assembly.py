"""Export the full robot as one assembled STEP (neutral standing pose).

Run:  .venv/bin/python cad/export_assembly.py   -> cad/step/assembly.step

Printed parts + mock STS3215 servo bodies (from check_assembly) placed with the
same transforms the interference checks use, in world frame: ground z=0,
+X forward, legs at y = +/-HIP_SEP/2. Open in FreeCAD/Onshape -- parts arrive
as a labeled tree (pelvis, gopro_base, leg_L/*, leg_R/*).

TORSO IS v6 as of 2026-08-04 (see the v6 blocks in dimensions.py): the WHOLE
torso is one print. battery_tray and board_frame are gone -- the pack now sits
on two seat chamfers in a bay under the deck, and the board stands upright and
transverse in a recess aft of the housing, held by two M2.5 into standoffs off
the rear web. The only bolt-on left is gopro_base, which is back on the roof
(the deck IS the roof) and is deliberately separate: it is the camera's crash
fuse. The board's ports edge stands 15 mm above the deck; nothing else printed
does.
"""
import math
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
COL_PCB = Color(0.05, 0.32, 0.18)
COL_CHIP = Color(0.15, 0.15, 0.17)

TOWER_TOP_Z = DECK_TOP_Z + D.TOWER_H    # LEGACY (v3 tower); nothing in the v4
                                        # assembly places anything against it


def camera_mock():
    """Delegates to check_assembly, which owns the bought-part mocks (it may not
    import this module, so they cannot live here). Kept as a name because
    render_electronics_steps and the assembly figures call it."""
    return CA.camera_mock()


def battery_mock():
    """The modeled 3S 850 pack (D.BATT_PACK) seated in the v6 bay, pelvis frame.

    There is no tray floor any more: the pack lies across Y and rests in the V
    formed by the bay's two 47 deg SEAT CHAMFERS, which centre it between the
    bay walls. So its x is set by the V's centreline, not by either wall, and
    its bottom is BT_SEAT_Z minus the drop its own width buys it -- 1.287 mm for
    a 30 mm pack against 0.751 for the 31 mm envelope (see BT_SEAT_DROP, whose
    derivation this pass corrected). Follows BATT_PACK rather than the envelope,
    so the fly-in proves the REAL pack drops through the deck aperture."""
    ly, wx, hz = D.BATT_PACK
    xc = (D.BT_SEAT_X + D.BT_WALL_X1) / 2                    # the V's centre
    z0 = D.BT_SEAT_Z - ((D.BT_SEAT_BAY_W - wx) / 2
                        * math.tan(math.radians(D.BT_SEAT_DEG)))
    return parts.box(xc - wx / 2, xc + wx / 2, -ly / 2, ly / 2, z0, z0 + hz)


def board_pcb_mock():
    return CA.board_pcb_mock()


def board_comp_mock():
    return CA.board_comp_mock()


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


# v4 torso: everything up here is bolted to the deck top, so all three groups
# share the pelvis frame (the tower/head_stack frames went with the tower).
# v6 leaves three torso groups: the stator screws, the board's two M2.5 from
# aft, and the gopro base's four M3 from above. Each is its own piece because
# each is its own assembly step on its own axis, and animate_assembly flies one
# axis per piece. All three live in the pelvis frame (the gopro screws included
# -- their seats are given at GP_BASE_T above the deck, in pelvis coordinates).
TORSO_FRAMES = [("deck", Pos(0, 0, DECK_TOP_Z), ("screws_deck",)),
                ("driver_board", Pos(0, 0, DECK_TOP_Z), ("screws_board",)),
                ("gopro", Pos(0, 0, DECK_TOP_Z), ("screws_gopro",))]


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
    # v6 torso: the tray and the frame are IN the pelvis print above. What is
    # left to place is the one bolt-on (gopro_base, on the deck at GP_MOUNT_X),
    # the camera it carries, and the two mocks the pelvis holds.
    piece("gopro_base", COL_PRINT,
          Pos(D.GP_MOUNT_X, 0, DECK_TOP_Z) * parts.gopro_base()),
    piece("camera_gopro_max_mock", COL_CAM,
          Pos(D.GP_MOUNT_X, 0, DECK_TOP_Z) * camera_mock()),
    piece("battery_3s_mock", COL_BATT, Pos(0, 0, DECK_TOP_Z) * battery_mock()),
    piece("board_pcb_mock", COL_PCB, Pos(0, 0, DECK_TOP_Z) * board_pcb_mock()),
    piece("board_parts_mock", COL_CHIP,
          Pos(0, 0, DECK_TOP_Z) * board_comp_mock()),
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
    # v6: the tallest PRINTED thing is the board's own top edge at BR_TOP_Z
    # (15 mm over the deck) -- but the camera is back, and it is what sets the
    # bbox now: base + legs + the M5 clamp + the body.
    struct = DECK_TOP_Z + D.BR_TOP_Z
    cam = (DECK_TOP_Z + D.GP_BASE_T + D.GP_HOLE_H + 6.0 + D.CAM_BODY[2])
    print(f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f}  y {bb.min.Y:.1f}..{bb.max.Y:.1f}"
          f"  z {bb.min.Z:.1f}..{bb.max.Z:.1f}  "
          f"(expect ~0..{cam:.0f} incl. camera; printed structure tops out at "
          f"{struct:.0f}, the board's ports edge)")


if __name__ == "__main__":
    main()
