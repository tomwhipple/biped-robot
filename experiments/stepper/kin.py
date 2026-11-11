"""Poses for the stepper: the one-leg experiment's IK (sim/v6_kin through design_gates.q_of) on the
as-built legs, as {joint: rad}. A pose is a crouch depth, a pelvis offset over the feet, and an
offset for each foot from where it stands."""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for d in ("sim", os.path.join("experiments", "one-leg-stand")):
    sys.path.insert(0, os.path.join(ROOT, d))
os.environ.setdefault("MUJOCO_GL", "egl")
import one_leg as OL  # noqa: E402
from design_gates import JN, Key, q_of, standing_key  # noqa: E402
from gen_plant_v6 import params_from_args  # noqa: E402


class Kin:
    def __init__(self, drop=0.02):
        self.p, _ = params_from_args(["-o", "/dev/null", "--set", *OL.AS_BUILT])
        self.drop = drop
        self.k0 = standing_key(self.p, 0.0)
        self.k1 = standing_key(self.p, drop)

    def model_offset(self, stance, lift=0.03, bias_y=-0.005):
        """The model's pelvis offset (dx, dy) in m from the crouched stand that puts its CoM over the
        `stance` sole with the other foot `lift` up (one_leg.keyframes' solve). +y = the robot's left."""
        swing = "R" if stance == "L" else "L"
        sgn = 1.0 if stance == "L" else -1.0
        m, d = OL.K.load(self.p)
        feet = {"L": self.k1.footL.copy(), "R": self.k1.footR.copy()}
        mid = feet[swing] + np.array([0.0, 0.0, lift])
        pel = OL.SG.solve_pelvis(m, d, self.p, self.k1.pelvis, feet, stance, mid, bias=(0.0, sgn * bias_y))
        off = pel - self.k1.pelvis
        return float(off[0]), float(off[1])

    def pose(self, shift=0.0, fwd=0.0, footL=(0.0, 0.0, 0.0), footR=(0.0, 0.0, 0.0), drop=None, straight=False,
             hike=None, ankle=None, ankle_p=None, ref=0.0, heading=0.0, yawL=0.0, yawR=0.0):
        """shift, fwd: pelvis offset (m) from the crouched stand, +y left / +x forward.
        footL/footR: (dx, dy, dz) of that sole from where it stands.
        hike: {"L": rad, "R": rad}, a trim on that leg's hip roll alone that lifts the OTHER hip when
        the leg is the one standing (the pelvis rolls toward it over a flat foot). Returns {joint: rad}.
        (Key.roll is not this: it rolls the soles with the pelvis, which on a flat floor stands the
        legs back upright -- the 2026-11-11 10:28 fall.)
        ankle: {"L": rad, "R": rad}, the same trim on that leg's ankle roll (same sign: it moves the body roll
        the same way as the hip's). ankle_p: {"L": rad, "R": rad}, a trim added to that leg's ankle PITCH joint.
        Sole levelling (FK-checked in dry_run.py, MuJoCo model): a body rolled by r about its forward axis (IMU
        roll +, right side down) tips the sole's normal by r about x: ankle=-r on the left leg, +r on the right
        cancels it; a body pitched by p (IMU pitch +, forward) needs ankle_p = -p on both; ankle=+b (both) puts
        the OUTER edge of either sole up by b.
        Turning (2026-11-11 check: Key.heading / yawL / yawR are WORLD yaws about +Z, + = counter-clockwise from above =
        the robot's left; a foot yaw of +a with the pelvis square gives hip_yaw +a, a pelvis heading of +a with the
        feet fixed gives hip_yaw -a and the legs re-aimed; rotating everything rigidly changes no joint):
        ref: the angle (rad) of the frame the offsets above are given in (body-aligned: x forward, y left of the
        heading it was last re-based to); pelvis and feet are rotated by it about the origin;
        heading: the pelvis yaw now; yawL / yawR: the soles' world yaws. All 0.0: exactly the pose without them."""
        k = (self.k0 if straight else (self.k1 if drop is None else standing_key(self.p, drop))).copy()
        k.pelvis = k.pelvis + np.array([fwd, shift, 0.0])
        k.footL = k.footL + np.asarray(footL, float)
        k.footR = k.footR + np.asarray(footR, float)
        if ref:
            c, s = math.cos(ref), math.sin(ref)
            rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            k.pelvis, k.footL, k.footR = rot @ k.pelvis, rot @ k.footL, rot @ k.footR
        k.heading, k.yawL, k.yawR = heading, yawL, yawR
        q = q_of(self.p, k)
        out = {n: float(q[JN.index(n)]) for n in JN}
        # body roll (IMU sign, + = right side down) = -(ankle_roll + hip_roll) of the standing leg
        for side, sgn in (("L", +1.0), ("R", -1.0)):
            out[f"{side}_hip_roll"] += sgn * float((hike or {}).get(side, 0.0))
            out[f"{side}_ankle_roll"] += sgn * float((ankle or {}).get(side, 0.0))
            out[f"{side}_ankle"] += float((ankle_p or {}).get(side, 0.0))
        return out
