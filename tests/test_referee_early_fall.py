"""A seed that falls early must be graded as a failure, not crash the referee
and lose the whole scorecard: every scenario's evaluate() has to cope with no
rows, and with rows that all end before the scenario's settle time (2026-11-06:
metronome's `max()` of nothing took down robot_legs_omni5_dqlag_s1)."""
import math
import os
import sys
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

SHARED = dict(mean_watts=30.0, wobble_rms=2.0)


def _rows(t_end):
    """the rows Driver.step() appends for a robot that falls at t_end"""
    n = int(round(t_end / 0.02))
    return [dict(t=(k + 1) * 0.02, x=0.0, y=0.0, yaw=0.0, planar=0.0, wob2=0.0, watts=30.0,
                 height=0.1, up_z=0.2, con_l=True, con_r=True, foot_err=0.0, foot_clear=0.0,
                 foot_dx=0.0, foot_dz=0.0, cmd_lift=0.0, com_stance=0.0, recovered=0.0)
            for k in range(n)]


def _scenarios():
    import eval_precision as E
    return sorted(E._registry().items())


@pytest.mark.parametrize("t_end", [0.0, 0.02, 0.4])
def test_every_scenario_grades_an_early_fall(t_end):
    crashed = {}
    for name, (_, (build, evaluate), _) in _scenarios():
        _, ev = build()
        ev["_env"] = SimpleNamespace(march_clear=0.1)  # run_one always sets it
        try:
            res = evaluate(_rows(t_end), ev, True, 0.2, SHARED)
        except Exception as e:  # collect them all: one report, not one per run
            crashed[name] = repr(e)
            continue
        assert not res["success"], name
    assert not crashed, crashed


def test_metronome_fall_before_settle_has_no_drift():
    import eval_precision as E
    build, evaluate = E.scen_metronome()
    _, ev = build()
    res = evaluate(_rows(0.4), ev, True, 0.2, SHARED)
    assert res["success"] is False
    assert math.isnan(res["metrics"]["drift"])
    assert res["headline"] == "no-lock"
