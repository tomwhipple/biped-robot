"""experiments/roll-balance/tip_causes.py: the pure parts (joint frame, left/right modes, the AUC and its permutation p,
the walk-fixed-effects regression, the logistic fit). The flight-record side runs on real data only."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
import tip_causes as TC  # noqa: E402


def test_joint_deg_matches_bus_steps_to_angle():
    assert TC.joint_deg(2048 + 4096 / 360.0 * 10, 2048, 1) == pytest.approx(10.0)
    assert TC.joint_deg(2048 + 4096 / 360.0 * 10, 2048, -1) == pytest.approx(-10.0)
    np.testing.assert_allclose(TC.joint_deg([2048, 3072], 2048, 1), [0.0, 90.0])


def test_modes_split_differential_and_common():
    sq, cm = TC.modes(40.0, -40.0)
    assert (sq, cm) == (40.0, 0.0)
    sq, cm = TC.modes(30.0, 10.0)
    assert (sq, cm) == (10.0, 20.0)


def test_auc_ties_and_extremes():
    assert TC.auc([3, 4], [1, 2]) == 1.0
    assert TC.auc([1, 2], [3, 4]) == 0.0
    assert TC.auc([1, 2], [1, 2]) == 0.5
    assert np.isnan(TC.auc([], [1.0]))
    assert TC.auc([2, float("nan")], [1]) == 1.0          # NaNs dropped


def test_perm_p_auc_separates_and_does_not():
    rng = np.random.default_rng(1)
    a, b = list(rng.normal(2.0, 1.0, 12)), list(rng.normal(0.0, 1.0, 150))
    assert TC.perm_p_auc(a, b, n=2000) < 0.01
    same = list(rng.normal(0.0, 1.0, 150))
    assert TC.perm_p_auc(same[:12], same[12:], n=2000) > 0.05


def test_ols_walk_fixed_effects_removes_walk_offsets():
    rows = []
    for w, off in (("a", 0.0), ("b", 10.0), ("c", -5.0)):
        for k in range(1, 9):
            x = k * 0.5
            rows.append(dict(stamp=w, x=x, y=off + 2.0 * x))   # a walk-level offset, slope 2 within walks
    b, se, r2, n = TC.ols(rows, "y", ["x"])
    assert n == 24 and b[0] == pytest.approx(2.0) and r2 == pytest.approx(1.0)
    b, _, _, _ = TC.ols(rows, "y", ["x"], by_walk=False)  # with a constant, no fixed effects: the offsets leak into it
    assert b[1] == pytest.approx(2.0)


def test_logit_nll_finds_the_predictor():
    rng = np.random.default_rng(2)
    rows = []
    for _ in range(300):
        x, z = rng.normal(), rng.normal()
        rows.append(dict(x=x, z=z, tip=int(rng.random() < 1.0 / (1.0 + np.exp(-(-2.0 + 2.0 * x))))))
    nll_x, zs = TC.logit_nll(rows, ["x"])
    nll_z, _ = TC.logit_nll(rows, ["z"])
    assert zs[1] > 4.0 and nll_x < nll_z
