"""experiments/roll-balance/stall_split.py: the pure parts (tick gaps, the stall in a double support, the fast-part
window, Wilson intervals). The split itself runs on real data only."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
import stall_split as S  # noqa: E402


def test_tick_gaps_finds_only_long_gaps():
    t = np.r_[np.arange(0, 1.0, 0.02), 1.12 + np.arange(0, 0.5, 0.02), 1.75]
    s, e = S.tick_gaps(t)
    np.testing.assert_allclose(s, [0.98, 1.60])
    np.testing.assert_allclose(e, [1.12, 1.75])
    assert S.tick_gaps(np.arange(0, 1, 0.02))[0].size == 0
    assert S.tick_gaps([0.0])[0].size == 0


def test_stall_in_ds_takes_the_longest_gap_ending_inside():
    starts = np.array([0.90, 1.20, 1.50, 2.40])
    ends = np.array([1.05, 1.30, 1.63, 2.60])
    # td 1.0 .. tl 2.0: the gaps ending at 1.05 (150 ms), 1.30 (100 ms), 1.63 (130 ms); 2.60 ends after the lift-off
    assert S.stall_in_ds(starts, ends, 1.0, 2.0) == (pytest.approx(0.90), pytest.approx(1.05))
    assert S.stall_in_ds(starts, ends, 1.1, 2.0) == (pytest.approx(1.50), pytest.approx(1.63))
    assert S.stall_in_ds(starts, ends, 2.7, 3.0) is None


def test_on_fast_part_window_is_inclusive_and_nan_safe():
    assert S.on_fast_part(-0.35) and S.on_fast_part(0.15) and S.on_fast_part(-0.2)
    assert not S.on_fast_part(-0.36) and not S.on_fast_part(0.16)
    assert not S.on_fast_part(float("nan"))


def test_wilson_interval():
    lo, hi = S.wilson(92, 165)
    assert lo == pytest.approx(0.481, abs=2e-3) and hi == pytest.approx(0.631, abs=2e-3)
    lo, hi = S.wilson(0, 20)
    assert lo == 0.0 and hi == pytest.approx(0.161, abs=2e-3)
    assert all(np.isnan(S.wilson(0, 0)))


def test_add_stall_without_a_cache_or_double_support(monkeypatch):
    monkeypatch.setattr(S, "gaps_for", lambda st: (np.array([10.30]), np.array([10.43])))
    r = S.add_stall(dict(stamp="x", lift_utc=11.5, ds_s=1.8, shift_t50=0.71))   # td 9.7, t50 at 10.41
    assert r["stall_len"] == pytest.approx(0.13) and r["stall_td"] == pytest.approx(0.73)
    assert r["stall_rel"] == pytest.approx(0.02) and r["stall_fast"] == 1
    r = S.add_stall(dict(stamp="x", lift_utc=11.5, ds_s=float("nan"), shift_t50=0.71))
    assert np.isnan(r["stall_rel"]) and r["stall_fast"] == 0
