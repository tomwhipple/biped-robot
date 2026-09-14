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


def test_plant_loads(p):
    """12 leg actuators + the v7 neck; mass in the modelled band."""
    m, d = K.load(p)
    assert m.nu == 13 and m.nq == 20
    assert m.actuator(12).name == "neck_yaw"
    assert 1.4 < m.body_subtreemass[0] < 1.7


def test_cad_and_sim_agree(p):
    """cad/v6/dimensions_v6.py and the sim plant share every kinematic number."""
    sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))
    import dimensions_v6 as V
    for k, v in V.SIM_EXPECT.items():
        assert abs(getattr(p, k) - v) < 1e-6, (k, getattr(p, k), v)


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
    assert A["A2"]["poses"]["mid_swing"]["margin"] >= 0.0249   # = the inboard sole half-width
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


def test_ik_yawed_feet_roundtrip(p):
    """pose_world: pelvis heading and per-foot yaws land the soles flat, at the
    requested world position and yaw, with the hip yaw carrying the difference."""
    import math
    m, d = K.load(p)
    pel = np.array([0.3, 0.1, p.z_yaw_above_sole - 0.02])
    hd = math.radians(20)
    fL = pel + K._rz(hd) @ np.array([0.02, 0.042, 0.0]); fL[2] = p.roll_h
    fR = pel + K._rz(hd) @ np.array([-0.03, -0.042, 0.0]); fR[2] = p.roll_h
    yL, yR = math.radians(10), math.radians(30)
    q = K.pose_world(p, pel, hd, fL, fR, yL, yR)
    K.set_pose(m, d, q, torso_pos=pel, torso_quat=K.heading_quat(hd))
    for s, f, yw, qi in (("L", fL, yL, 0), ("R", fR, yR, 6)):
        pos, R = K.foot_frame(m, d, s)
        assert np.linalg.norm(pos - f) < 1e-4
        assert abs(R[2, 2] - 1.0) < 1e-6
        assert abs(math.degrees(math.atan2(R[1, 0], R[0, 0])) - math.degrees(yw)) < 1e-3
        assert abs(q[qi] - (yw - hd)) < 1e-6


def test_arc_walk_turns_under_deploy_model(p):
    """4 steps at 15 deg/step under the deploy servo model (STS3250 rolls +
    knees) must complete and change the heading by at least 30 deg."""
    import static_gait as S
    from gen_plant_v6 import build_xml
    import tempfile, os
    xml = os.path.join(tempfile.gettempdir(), "v6_test_arc.xml")
    open(xml, "w").write(build_xml(p))
    pj = {j: "sts3250" for j in S.ROLLS + ("L_knee", "R_knee")}
    tl, win = S.walk_timeline(p, n_steps=4, lift_h=0.04, turn_deg=15.0)
    r = S.run_walk(p, xml, tl, win, play_deg=3.0, per_joint=pj)
    assert r["ok"], r
    assert r["heading_deg"] > 30.0
