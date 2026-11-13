"""--sole-in-mm (2026-11-12, #297): the sole's inner support edge moved toward the ankle-roll axis.

The dwell-gait runs on the robot put its inward single-support edge at a shift of ~1.2-1.3 x 62 mm on both
feet; the plant's sole (pads reaching ~22 mm inboard of the ankle-roll axis) holds down to ~0.5 x. Plant FK on
the flight record puts the robot's load-bearing inner limit near the axis (the play inside the roll servos lets
the stance leg pivot there). The term moves every sole pad inboard of a line onto it, per foot, in the
MjModel before the MJX upload (walker_env.apply_sole_inset, shared by both envs), so training, the referee,
grade.py (gate_common `sole_in*` terms) and the SIL see the same sole. SIL calibration: L 8, R -12 mm.
"""
import inspect
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
GATE = os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--quantize"]


def _pads(model):
    import mujoco
    out = {"L": [], "R": []}
    for g, y in W.sole_pad_nominal(model).items():
        out[(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "?")[0]].append(y)
    return {k: sorted(v) for k, v in out.items()}


def test_parser():
    P = W.parse_sole_in
    assert P(None) is None and P("") is None
    assert P("5") == {"L": 5.0, "R": 5.0}
    assert P("8,-12") == {"L": 8.0, "R": -12.0}
    assert P("L=8,R=-12") == {"L": 8.0, "R": -12.0}
    assert P("R=-12") == {"L": None, "R": -12.0}
    assert P({"L": 8, "R": -12}) == {"L": 8.0, "R": -12.0}
    for bad in ("1,2,3", "X=3", "40"):
        with pytest.raises(ValueError):
            P(bad)


def test_defaults_and_cli():
    from env_mjx import BimoMJXEnv
    for cls in (BimoMJXEnv, W.BimoWalkerEnv):
        assert inspect.signature(cls.__init__).parameters["sole_in_mm"].default is None
    import train_mjx
    kw = train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + ["--sole-in-mm", "L=8,R=-12"]))
    assert kw["sole_in_mm"] == {"L": 8.0, "R": -12.0}
    kw0 = train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS))
    assert "sole_in_mm" not in kw0


def _cpu_env(**over):
    import parity_test as P
    return W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False, **P.rob_kw(**over))


def test_inset_moves_only_inner_pads_and_off_is_exact():
    e0 = _cpu_env()
    e1 = _cpu_env(sole_in_mm="L=8,R=-12")
    p0, p1 = _pads(e0.model), _pads(e1.model)
    assert min(p0["L"]) < -0.015 and max(p0["R"]) > 0.015          # the plant's pads reach ~22 mm inboard
    assert min(p1["L"]) == pytest.approx(-0.008) and max(p1["R"]) == pytest.approx(-0.012)
    # pads outboard of the lines keep their place
    assert [y for y in p1["L"] if y > -0.008] == [y for y in p0["L"] if y > -0.008]
    assert [y for y in p1["R"] if y < -0.012] == [y for y in p0["R"] if y < -0.012]
    # off: bit-exact
    assert np.array_equal(e0.model.geom_pos, _cpu_env(sole_in_mm=None).model.geom_pos)


def test_mjx_model_carries_the_same_sole():
    import parity_test as P
    c, g, _ = P.envs_for(P.rob_kw(sole_in_mm="L=8,R=-12"))
    assert np.array_equal(np.asarray(g.mj_model.geom_pos), np.asarray(c.model.geom_pos))
    assert max(_pads(g.mj_model)["R"]) == pytest.approx(-0.012)


def test_gate_term_idempotent():
    sys.path.insert(0, GATE)
    import gate_common as GC
    e = _cpu_env()
    p0 = _pads(e.model)
    GC.apply_terms(e, GC.parse_terms("sole_in_L=8,sole_in_R=-12"))
    GC.apply_terms(e, GC.parse_terms("sole_in_L=8,sole_in_R=-12"))
    p1 = _pads(e.model)
    assert min(p1["L"]) == pytest.approx(-0.008) and max(p1["R"]) == pytest.approx(-0.012)
    GC.apply_terms(e, GC.parse_terms("sole_in=30"))                  # wider than the plant: nothing moves
    assert _pads(e.model) == p0
