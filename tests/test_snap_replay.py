"""experiments/roll-balance/snap_replay.py: the pure parts (forward fill, the goal columns, the touchdown-pause
counterfactual, the replay window, the lean, the chain study terms). The replay itself runs on real data only."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
import snap_replay as S  # noqa: E402
import tip_causes as TC  # noqa: E402


def test_ffill_fills_down_and_leading():
    x = np.array([[np.nan, 1.0], [2.0, np.nan], [np.nan, np.nan], [5.0, 3.0]])
    np.testing.assert_array_equal(S.ffill(x), [[2.0, 1.0], [2.0, 1.0], [2.0, 1.0], [5.0, 3.0]])


def _arr(n, dt=0.02):
    a = np.zeros((n, 10 + len(TC.LEG) * len(TC.JF)))
    a[:, 0] = np.arange(n) * dt
    return a


def test_goal_cols_treats_raw_zero_as_missing():
    a = _arr(4)
    c = TC.col("L_hip_roll", "goal")
    a[:, c] = [2000, 0, 2010, 0]
    g = S.goal_cols(a)
    np.testing.assert_array_equal(g[:, TC.LEG.index("L_hip_roll")], [2000, 2000, 2010, 2010])


def test_delay_shift_holds_the_shift_and_keeps_the_fade_in_real_time():
    a = _arr(101)                                   # 0 .. 2 s at 50 Hz; touchdown row 10 (0.2 s), lift row 80 (1.6 s)
    cg, cn = TC.col("L_hip_roll", "goal"), TC.col("L_hip_roll", "nominal")
    t = a[:, 0]
    a[:, cn] = 2000 + 100 * np.clip((t - 0.2) / 1.0, 0, 1)      # the shift: a ramp from the touchdown
    fade = 30 * np.clip(1 - (t - 0.2) / 0.6, 0, 1) * (t >= 0.2)   # the last correction, fading over 0.6 s
    a[:, cg] = a[:, cn] + fade
    a[:, 1] = np.where(t < 0.2, 6, np.where(t < 0.8, 7, 0))       # HOLD, FADE, OFF
    a[80:, 1] = 5                                                 # CLOSED at the lift-off
    b, i_lift = S.delay_shift(a, 10, 80, 0.3)
    tb = b[:, 0]
    assert len(b) == len(a) + 15
    assert i_lift == 95 and tb[i_lift] == pytest.approx(1.9)
    assert b[i_lift, 1] == 5 and b[i_lift - 1, 1] == 0
    # the nominal shift moved 0.3 s later; its value at the touchdown held through the pause
    np.testing.assert_allclose(np.interp([0.3, 0.5, 0.8, 1.2], tb, b[:, cn]), [2000, 2000, 2030, 2070], atol=1e-6)
    # the fade (goal - nominal) and the trim state keep the real time
    res = b[:, cg] - b[:, cn]
    np.testing.assert_allclose(np.interp([0.3, 0.5, 0.79], tb, res), [25, 15, 0.5], atol=0.6)
    assert b[np.searchsorted(tb, 0.5), 1] == 7 and b[np.searchsorted(tb, 0.85), 1] == 0
    # zero delay is the identity
    b0, i0 = S.delay_shift(a, 10, 80, 0.0)
    np.testing.assert_array_equal(b0, a)
    assert i0 == 80


def test_window_starts_before_the_previous_single_support():
    t = np.arange(0, 20, 0.02)
    ix = lambda s: int(round(s / 0.02))  # noqa: E731
    ch = [(ix(2.0), None, ix(4.0)), (ix(6.0), None, ix(8.0)), (ix(10.0), None, ix(12.0))]
    lo, hi = S.window(t, ch, 3)
    assert lo == pytest.approx(max(6.0 - S.PLACE_S, 4.0 + S.PLACE_TD_S))
    assert hi == pytest.approx(12.0 + S.END_S)
    lo2, _ = S.window(t, ch, 2)
    assert lo2 == pytest.approx(0.5)                     # no touchdown before it: the record's start + 0.5 s


def test_lean_in_toward_the_swing_foot():
    t = np.linspace(0, 2, 101)
    err = np.sin(np.pi * t) * 5
    assert S.lean_in(t, err, 0.0, 2.0, 1.0) == pytest.approx(5.0, abs=0.01)
    assert S.lean_in(t, -err, 0.0, 2.0, -1.0) == pytest.approx(5.0, abs=0.01)
    assert np.isnan(S.lean_in(t, err, 3.0, 4.0, 1.0))


def test_split_chain_keeps_the_plant_terms():
    spec, st = S.split_chain("grade+sag_L=6.4,x_dead50=40,sag_R=2.9,x_db15=0.5")
    assert spec == "grade+sag_L=6.4,sag_R=2.9"
    assert st == {"x_dead50": 40.0, "x_db15": 0.5}
    assert S.split_chain("grade") == ("grade", {})


def test_delay_shift_rt_keeps_the_record_ticks_before_the_lift_off():
    a = _arr(101)
    a[30:, 0] += 0.12                                  # a 140 ms stall in the record at 0.58 s (0.38 s after the touchdown)
    cg, cn = TC.col("L_hip_roll", "goal"), TC.col("L_hip_roll", "nominal")
    t = a[:, 0]
    a[:, cn] = 2000 + 100 * np.clip((t - 0.2) / 1.0, 0, 1)
    a[:, cg] = a[:, cn]
    a[:, 1] = np.where(t < 0.2, 6, 0)
    a[80:, 1] = 5
    b, i_lift = S.delay_shift_rt(a, 10, 80, 0.3)
    tb = b[:, 0]
    np.testing.assert_array_equal(tb[:80], t[:80])     # the stall stays where it was after the touchdown
    assert tb[i_lift] == pytest.approx(t[80] + 0.3) and b[i_lift, 1] == 5
    # the nominal is the record's 0.3 s earlier (held at the touchdown's before that)
    np.testing.assert_allclose(b[:80, cn][t[:80] > 0.2], np.interp(np.maximum(t[:80][t[:80] > 0.2] - 0.3, 0.2), t, a[:, cn]))
    assert np.all(np.diff(tb) > 0)


def test_fill_gaps_moves_the_goals_on_through_a_stall():
    a = _arr(10, dt=0.04)
    a[5:, 0] += 0.12                                   # one 160 ms gap
    cg = TC.col("L_hip_roll", "goal")
    a[:, cg] = 1000 + 100 * a[:, 0]                    # a goal moving with time
    b = S.fill_gaps(a)
    assert len(b) == len(a) + 3
    assert np.all(np.diff(b[:, 0]) <= 0.05 + 1e-9)
    np.testing.assert_allclose(b[:, cg], 1000 + 100 * b[:, 0])
    np.testing.assert_array_equal(S.fill_gaps(_arr(10)), _arr(10))     # no gap: unchanged


def test_variant_names():
    a = _arr(101)
    a[:, 1] = np.where(a[:, 0] < 0.2, 6, 0)
    a[80:, 1] = 5
    assert S.variant(a, 10, 80, "0") is a
    assert len(S.variant(a, 10, 80, "0.3")) == len(a) + 15
    assert len(S.variant(a, 10, 80, "0.3rt")) == len(a) + 15
    assert S.vkey("0.0") == "0" and S.vkey("0.30") == "0.3" and S.vkey("fill") == "fill"


def test_scale_correction_scales_goal_minus_nominal_only():
    a = _arr(5)
    cg, cn = TC.col("L_hip_roll", "goal"), TC.col("L_hip_roll", "nominal")
    a[:, cn] = 2000
    a[:, cg] = 2000 + np.array([0, 10, 20, 30, 40])
    b = S.scale_correction(a, 0.5)
    np.testing.assert_array_equal(b[:, cg], 2000 + np.array([0, 5, 10, 15, 20]))
    np.testing.assert_array_equal(b[:, cn], a[:, cn])
    assert a[4, cg] == 2040                                   # the input is not modified


def test_stall_in_ss_moves_the_stall_into_the_single_support():
    a = _arr(151)                                             # 0 .. 3 s at 50 Hz
    t = a[:, 0]
    a[:, 1] = np.where((t >= 1.0) & (t < 2.0), 5, 0)          # one single support: CLOSED from 1.0 s
    a[t >= 2.0, 1] = 7                                        # touchdown (FADE) at 2.0 s
    a = np.delete(a, np.where((t > 0.3) & (t < 0.42))[0], 0)  # a 140 ms stall in the double support before it
    b = S.stall_in_ss(a)
    tb = b[:, 0]
    gaps = np.diff(tb)
    assert gaps[tb[:-1] < 1.0].max() <= 0.05                  # the double support's stall is filled
    big = np.where(gaps > 0.08)[0]
    assert len(big) == 1 and tb[big[0]] == pytest.approx(1.06) and gaps[big[0]] == pytest.approx(0.12)   # 0.115 s at 50 Hz ticks
    assert np.all(np.diff(tb) > 0)


def test_variant_names_c_and_ss():
    a = _arr(101)
    a[:, 1] = np.where(a[:, 0] < 0.2, 6, 0)
    a[80:, 1] = 5
    cg, cn = TC.col("R_hip_roll", "goal"), TC.col("R_hip_roll", "nominal")
    a[:, cg] = a[:, cn] + 8
    np.testing.assert_array_equal(S.variant(a, 10, 80, "c0")[:, cg], a[:, cn])
    np.testing.assert_array_equal(S.variant(a, 10, 80, "c0.5fill")[:, cg], a[:, cn] + 4)
    assert len(S.variant(a, 10, 80, "ss")) < len(a)
    assert S.vkey("c0.5") == "c0.5" and S.vkey("ss") == "ss"
