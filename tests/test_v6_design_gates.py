"""The v6 (ankle-roll) design record must keep passing its own gates.

docs/design-v6-ankle-roll.md states the numbers these tests pin: the plant
loads with 12 actuators, the analytic leg IK round-trips through MuJoCo's
forward kinematics, Gate A finds a static single-foot stance with >= 25 mm of
CoM margin inside every joint limit, and Gate B finds no STS3215 joint short
of the torque margin at the design cadence (the knee's speed margin is the
known exception, covered by the STS3250 assignment).
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

from gen_plant_v6 import DesignParams  # noqa: E402
import v6_kin as K  # noqa: E402
import design_gates as G  # noqa: E402


@pytest.fixture(scope="module")
def p():
    return DesignParams()


def test_plant_loads_12_dof(p):
    m, d = K.load(p)
    assert m.nu == 12 and m.nq == 19
    assert 1.1 < m.body_subtreemass[0] < 1.4


@pytest.mark.parametrize("knee", ["fwd", "bwd"])
def test_ik_roundtrip(knee):
    import dataclasses
    p = dataclasses.replace(DesignParams(), knee=knee)
    m, d = K.load(p)
    pel = np.array([0.0, 0.0, 0.0])
    zf = -(p.d_roll_pitch + p.thigh + p.shank + p.d_ankle) - p.d_yaw_roll
    for dp, dl, dr in (([0, 0, -0.03], [0, 0, 0], [0, 0, 0]),
                       ([0, 0.04, -0.01], [0, 0, 0], [0, 0, 0]),
                       ([0.02, -0.03, -0.02], [0.04, 0, 0.03], [-0.03, 0, 0])):
        fL = np.array([0, p.hip_sep / 2, zf]) + dl
        fR = np.array([0, -p.hip_sep / 2, zf]) + dr
        q = K.pose_from_feet(p, pel + dp, fL, fR)
        K.set_pose(m, d, q, torso_pos=pel + dp + [0, 0, 1.0])
        for s, f in (("L", fL), ("R", fR)):
            pos, R = K.foot_frame(m, d, s)
            assert np.linalg.norm(pos - (f + [0, 0, 1.0])) < 1e-4
            assert abs(R[2, 2] - 1.0) < 1e-6


def test_gate_a_passes(p):
    A = G.gate_a(p, verbose=False)
    assert A["A_pass"]
    assert A["A2"]["poses"]["mid_swing"]["margin"] >= 0.025
    assert A["A2"]["poses"]["mid_swing"]["jok"]
    assert A["A2"]["poses"]["mid_swing"]["ncon"] == 0
    assert A["A5"]["crouch_depth"] >= 0.05


def test_gate_b_torque_margins(p):
    A = G.gate_a(p, verbose=False)
    B = G.gate_b(p, A["A2"]["y_lift"], verbose=False, servos=("sts3215",))
    r = B["sts3215"]
    assert not r["fell"]
    assert min(r["torque_margin"]) >= 1.5
    assert max(r["sat"]) == 0.0
    # only the swing knee is short of the 2x speed margin on the STS3215
    assert set(r["short"]) <= {"R_knee", "L_knee"}
