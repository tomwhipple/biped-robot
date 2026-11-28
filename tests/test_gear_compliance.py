"""experiments/gear-compliance (#6, 2026-11-28): the flight-record tool behind the per-side roll
series spring (docs/design-v6/2026-11-28-gear-compliance.md). No recordings needed: the servo law the
holds showed, the tilt and gravity conventions on the plant, and the spring fit's recovery of known k.
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "gear-compliance"))
os.environ.setdefault("MUJOCO_GL", "disabled")

import gear_compliance as G  # noqa: E402


def test_servo_law():
    e = np.arange(-5, 6)
    np.testing.assert_allclose(G.servo_law(e, 1) * 100,            # an STS3215 (R hip pitch)
                               [-4.8, -4.0, -3.2, -2.4, 0, 0, 0, 2.4, 3.2, 4.0, 4.8])
    np.testing.assert_allclose(G.servo_law(e, 4) * 100, 0.8 * e)  # the STS3250 (L hip roll)
    assert G.LEGS[0] == "R_hip_roll" and G.LEGS[4] == "L_hip_roll"


def test_tilt_and_up_body():
    # the firmware's roll / pitch (ZYX) back through up_body and tilt
    r, p = np.array([-0.18, 0.05]), np.array([0.07, -0.02])
    rr, pp = G.tilt(G.up_body(r, p))
    np.testing.assert_allclose(rr, r, atol=1e-12)
    np.testing.assert_allclose(pp, p, atol=1e-12)


def test_kinematics_and_statics_on_the_plant():
    if not os.path.exists(G.plant_path()):
        pytest.skip("the plant is not built (make plant)")
    S = G.Statics()
    up, pf, tg = S.run(np.zeros((1, 12)), np.array([[0.0, 0.0, 1.0]]))
    for s in "LR":
        np.testing.assert_allclose(up[s][0], [0, 0, 1], atol=1e-9)        # straight legs, level
    assert abs(pf["L"][0, 2] - pf["R"][0, 2]) < 1e-9
    assert pf["L"][0, 1] > 0 > pf["R"][0, 1]                               # +y = the robot's left
    # standing on one foot, the stance hip roll carries the torso and the other leg toward the
    # lifted side: + for the left stance, - for the right, the same size (the plant is symmetric)
    assert tg["L"][0, 0] > 0.5 and tg["R"][0, 0] < -0.5
    assert tg["L"][0, 0] == pytest.approx(-tg["R"][0, 0], rel=0.02)
    # a left-side-down roll (firmware roll < 0) lowers the left foot
    g = G.up_body(np.array([math.radians(-5.0)]), np.array([0.0]))
    _, pf, _ = S.run(np.zeros((1, 12)), g)
    hL, hR = -(pf["L"] @ g[0]), -(pf["R"] @ g[0])
    assert hL[0] > hR[0] + 0.005


def test_spring_fit_recovers_known_k():
    rng = np.random.default_rng(1)
    kh, ka = 10.0, 15.0
    rows = []
    for i in range(14):
        th = 0.84 + 0.01 * rng.standard_normal()
        ta = rng.uniform(-0.45, 0.5)
        sag = math.degrees(th / kh + ta / ka) + 0.05 * rng.standard_normal()
        rows.append(dict(session=f"s{i}", n=1000, sag_deg=sag, tau_hip=th, tau_ankle=ta))
    rows.append(dict(session="odd", n=1000, sag_deg=20.0, tau_hip=0.5, tau_ankle=1.2))  # out: tau_hip < H_MIN
    f = G.spring_fit(rows, nboot=200)
    assert f["sessions"] == 14
    assert f["k_hip"] == pytest.approx(kh, rel=0.05) and f["k_ankle"] == pytest.approx(ka, rel=0.1)
    assert f["k_hip_ci"][0] < kh < f["k_hip_ci"][1]
    assert f["rms_spring_deg"] < f["rms_constant_deg"]
