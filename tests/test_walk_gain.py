"""experiments/policy/x15-margin/walk_gain.py: the pure parts (the probe's high-pass, the lagged gain fit, the pooled
zero-lag H1, the single / double support windows). The flight-record and replay sides run on real data only."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "x15-margin"))
import walk_gain as W  # noqa: E402


def test_ema_hp_is_the_out_gain_law():
    # grade.py: target = ema + G (a - ema), ema += (1 - exp(-DT / 0.5)) (a - ema): ema_hp is the (a - ema) part
    a = np.sin(np.arange(500) * W.DT * 2 * math.pi * 1.5)[:, None]
    k = 1 - math.exp(-W.DT / 0.5)
    e, ref = a[0].copy(), []
    for x in a:
        e = e + k * (x - e)
        ref.append(x - e)
    assert np.allclose(W.ema_hp(a), np.array(ref))
    assert np.allclose(W.ema_hp(np.ones((50, 2))), 0.0)       # a constant is all mean


def test_ratio_recovers_gain_and_lag():
    rng = np.random.default_rng(0)
    ys = W.band(rng.normal(size=(3000, 1)), (0.3, 3.0))[:, 0]
    lag = 2                                                  # 40 ms: the robot behind the sim
    yr = 1.7 * np.roll(ys, lag) + 0.05 * rng.normal(size=ys.size)
    m = np.zeros(ys.size, bool)
    m[100:2900] = True
    g, r, tau, _, _ = W.ratio(yr, ys, m)
    assert abs(g - 1.7) < 0.05 and r > 0.95 and tau == 40.0


def test_pooled_ignores_uncorrelated_robot_motion_and_finds_the_lag():
    rng = np.random.default_rng(1)
    rows = []
    for _ in range(20):
        ys = W.band(rng.normal(size=(600, 1)), (1.0, 3.0))[:, 0]
        yr = 0.8 * np.roll(ys, 4) + 2.0 * np.std(ys) * rng.normal(size=600)   # 0.8 x the sim, 80 ms late, + own
        m = np.zeros(600, bool)
        m[50:550] = True
        rows.append(dict(n=int(m.sum()), lc=W.lagcov(yr, ys, m), rms_s=float(np.std(ys[m])),
                         rms_r=float(np.std(yr[m]))))
    g, coh, lag, g0, rr = W.pooled(rows)
    assert abs(g - 0.8) < 0.08 and lag == 80.0               # H1 is unbiased by the robot's own motion
    assert g0 < g                                            # the lag costs the zero-lag fit
    assert 0.0 < coh < 0.3 and rr > 2.0                      # the rms ratio is not


def test_pooled_coherence_is_bounded_when_lags_reach_other_phases():
    # a quiet window next to a loud one: a long lag pulls the loud samples in; the per-lag variance keeps coh <= 1
    rng = np.random.default_rng(2)
    ys = np.r_[0.01 * rng.normal(size=300), rng.normal(size=300)]
    yr = 1.0 * ys
    m = np.zeros(600, bool)
    m[200:300] = True
    x = dict(n=100, lc=W.lagcov(yr, ys, m), rms_s=float(np.std(ys[m])), rms_r=float(np.std(yr[m])))
    g, coh, lag, g0, rr = W.pooled([x] * 5)
    assert coh <= 1.0 + 1e-9 and abs(g - 1.0) < 1e-6 and lag == 0.0


def test_windows_split_single_and_double_support():
    t = np.arange(0, 12, W.DT)
    st = np.zeros(t.size, int)
    sd = np.zeros(t.size, int)
    st[(t >= 2) & (t < 4)] = W.CLOSED
    st[(t >= 4) & (t < 5)] = W.HOLD
    sd[(t >= 2) & (t < 5)] = ord("L")
    st[(t >= 5) & (t < 5.3)] = W.FADE                        # short fade: part of the double support
    st[(t >= 7) & (t < 9)] = W.CLOSED
    sd[(t >= 7) & (t < 9)] = ord("R")
    r = dict(kind="walk", t=t, bal_state=st, bal_side=sd)
    s = dict(t=t)
    w = W.windows(r, s)
    kinds = [(k, side) for k, side, _, _ in w]
    assert kinds == [("ds", ""), ("ss", "L"), ("ds", ""), ("ss", "R"), ("ds", "")]
    ss_l = w[1]
    assert abs(t[ss_l[2]] - 2.0) < 1e-9 and abs(t[ss_l[3] - 1] - (5.0 - W.DT)) < 1e-6
    assert W.leg_role("L_knee", "ss", "L") == "stance" and W.leg_role("R_knee", "ss", "L") == "swing"
    assert W.leg_role("R_knee", "ds", "") == "ds"


def test_stand_windows_are_fixed_length():
    t = np.arange(0, 20, W.DT)
    w = W.windows(dict(kind="stand"), dict(t=t))
    assert all(k == "stand" and i1 - i0 == int(W.STAND_WIN_S / W.DT) for k, _, i0, i1 in w)
    assert len(w) == 4
