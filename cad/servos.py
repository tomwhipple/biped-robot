"""Servo mock STEP exports for the poseable FreeCAD assembly.

`check_assembly.py` already defines the three servo mocks the interference
checks trust (case + horn + idler recess, measured from the real SO-ARM/STS
case).  FreeCAD can't run build123d, so -- exactly like `fasteners.py` -- this
writes one STEP per servo orientation in the SAME joint-local frame the dressed
model places it in, and `freecad_articulate.py` reads them back.

Frames match `dress.py:dressed_leg` one-for-one, so a servo lands in the FCStd
where the dressed renders put it:

    servo_yaw     torso   @ HIP_YAW_Z    horn DOWN, case under the deck
    servo_roll    yaw     @ HIP_ROLL_Z   output +X, case rides the carrier bay
    servo_pitch   thigh   @ HIP_PITCH_Z  output +Y  (same STEP reused for...)
    servo_pitch   shin    @ KNEE_Z       ...the knee -- identical orientation
    servo_ankle   foot    @ ANKLE_Z      output +Y, rolled 90 deg into the heel

These are MOCKS for visual/fit context, not collision evidence: the disc faces
touch the servo cases by design, so `freecad_pose.py` skips them when it
intersects bodies (see SERVO_PREFIX there).

Run:  .venv/bin/python cad/servos.py     (-> cad/step/servo_*.step)
"""
import os

from build123d import Pos, Rot, export_step

import check_assembly as CA
import dimensions as D

# ------------------------------------------------------------- STEP exports
# Each lambda returns the mock in its JOINT-LOCAL frame: origin on the joint
# axis, in the leg plane (x=0, y=0).  freecad_articulate.py adds the lateral
# offset and joint height with a plain placement, the same way it does for the
# printed parts and the screw groups.
GROUPS = {
    # yaw servo hangs below the deck: its horn face sits at the carrier's top
    "servo_yaw":   lambda: Pos(0, 0, D.SV_HORN_FACE) * CA.servo_mock_z(),
    "servo_roll":  CA.servo_mock_x,
    "servo_pitch": CA.servo_mock_y,                       # hip pitch AND knee
    "servo_ankle": lambda: Rot(0, 90, 0) * CA.servo_mock_y(),
}


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step")
    os.makedirs(out, exist_ok=True)
    for name, fn in GROUPS.items():
        solid = fn()
        export_step(solid, os.path.join(out, f"{name}.step"))
        print(f"{name:14s} {solid.volume / 1000.0:6.2f} cm3 "
              f"-> step/{name}.step")


if __name__ == "__main__":
    main()
