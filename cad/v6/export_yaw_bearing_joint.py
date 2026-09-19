"""STEP export for the hip-yaw bearing study (docs/design-v6/
study-yaw-bearing.md): the changed parts (carrier, pelvis) PLUS a small joint
sub-assembly (pelvis cell region, cropped, + carrier + both bearing rings +
the yaw AND roll servo mocks) so Tom can open the actual seat in FreeCAD
without loading the whole 33-piece robot.

    .venv/bin/python cad/v6/export_yaw_bearing_joint.py A   # -> step/yaw_bearing_recommended/
    .venv/bin/python cad/v6/export_yaw_bearing_joint.py C   # -> step/yaw_bearing_runner_up/
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
os.environ.setdefault("MUJOCO_GL", "egl")

from build123d import Pos, Compound, Box, export_step  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402
import pelvis_v7  # noqa: E402

D = V.D
cyl_z = P.cyl_z

FOLDER = {"A": "yaw_bearing_recommended", "C": "yaw_bearing_runner_up"}
LABEL = {"A": "Option A: round hub, 6810-2RS (RECOMMENDED)",
         "C": "Option C: 6811 wrap, the 2026-09-17 baseline (runner-up)"}


def ring_body(r0, r1, z0, z1):
    return cyl_z(r1, z0, z1, 0, 0) - cyl_z(r0, z0 - 1, z1 + 1, 0, 0)


def build(variant):
    if variant == "A":
        import yaw_carrier_v6_optA
        carrier = yaw_carrier_v6_optA.yaw_carrier_v6_optA()
        brg_id, brg_od, brg_w = V.YAWA_BRG_ID, V.YAWA_BRG_OD, V.YAWA_BRG_W
        boss_r = V.YAWA_BRG_BOSS_R
        inner_od, outer_id = V.YAWA_EST_INNER_RING_OD, V.YAWA_EST_OUTER_RING_ID
        recess_r = V.YAWA_BRG_RECESS_R
    else:
        import yaw_carrier_v6
        carrier = yaw_carrier_v6.yaw_carrier_v6()
        brg_id, brg_od, brg_w = V.YAW_BRG_ID, V.YAW_BRG_OD, V.YAW_BRG_W
        boss_r = V.YAW_BRG_BOSS_R
        inner_od, outer_id = V.YAW_BRG_EST_INNER_RING_OD, V.YAW_BRG_EST_OUTER_RING_ID
        recess_r = V.YAW_BRG_RECESS_R
    pelvis = pelvis_v7.pelvis_v7(bearing_variant=variant)

    y = V.HIP_SEP / 2
    # ---- joint sub-assembly, WORLD frame (same placements as assembly_v6._leg_chain)
    ring_z = (V.HIP_YAW_Z - brg_w, V.HIP_YAW_Z)
    inner_ring = Pos(0, y, 0) * ring_body(boss_r, inner_od, *ring_z)
    outer_ring = Pos(0, y, 0) * ring_body(outer_id, recess_r, *ring_z)
    carrier_w = Pos(0, y, V.HIP_YAW_Z) * carrier
    yaw_servo_w = Pos(0, y, V.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()
    roll_servo_w = Pos(0, y, V.HIP_ROLL_Z) * CA.servo_mock_x()
    pelvis_w = Pos(0, 0, V.DECK_TOP_Z) * pelvis

    # crop the pelvis to just the cell region around this one hip (task asks
    # for "the pelvis cell region", not the whole torso print)
    crop = Pos(-15, y, V.YAW_BOX_BOT_Z / 2 + V.DECK_TOP_Z) * Box(90, 90, -V.YAW_BOX_BOT_Z + 20)
    pelvis_cell = pelvis_w & crop

    named = [(pelvis_cell, "pelvis_v7_cell"), (carrier_w, f"yaw_carrier_{variant}"),
             (inner_ring, "bearing_inner_race"), (outer_ring, "bearing_outer_race"),
             (yaw_servo_w, "servo_hip_yaw_mock"), (roll_servo_w, "servo_hip_roll_mock")]
    for solid, label in named:
        solid.label = label
    joint = Compound(label=f"yaw_joint_{variant}", children=[solid for solid, _ in named])

    return carrier, pelvis, joint


def main(variant):
    folder = os.path.join(HERE, "step", FOLDER[variant])
    os.makedirs(folder, exist_ok=True)
    carrier, pelvis, joint = build(variant)
    export_step(carrier, os.path.join(folder, f"yaw_carrier_v6_{variant}.step"))
    export_step(pelvis, os.path.join(folder, f"pelvis_v7_{variant}.step"))
    export_step(joint, os.path.join(folder, f"joint_assembly_{variant}.step"))
    print(f"{LABEL[variant]}")
    print(f"wrote {folder}/yaw_carrier_v6_{variant}.step, pelvis_v7_{variant}.step, joint_assembly_{variant}.step")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "A")
