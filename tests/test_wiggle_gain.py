"""experiments/policy/x15-margin/wiggle_gain.py: the pure parts (the out_gain law's gain at a frequency, the firmware's
Catmull-Rom, the instrumental-variable gain against the known excitation, the alignment of the logged excitation to the
written goals, the template from a log). The replays and the flight records run on real data only (`simcheck`)."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "x15-margin"))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

W = pytest.importorskip("wiggle_gain")
TH = pytest.importorskip("track_host")


def test_law_gain_is_the_out_gain_law_on_a_sine():
    f, G = 2.0, 1.25
    t = np.arange(0, 20, W.DT)
    x = np.sin(2 * math.pi * f * t)[:, None]
    y = x + (G - 1) * W.WG.ema_hp(x)
    m = t > 5
    amp = np.sqrt(2) * np.std(y[m, 0]) / (np.sqrt(2) * np.std(x[m, 0]))
    assert abs(amp - W.law_gain(G, f)) < 0.005
    assert np.allclose(W.law_gain(1.0, [0.5, 3.0]), 1.0)
    assert 1.2 < W.law_gain(1.25, 1.07) < W.law_gain(1.25, 2.87) < 1.25


def test_catmull_at_is_the_track_players():
    t = np.arange(0, 2001, 100.0)
    v = np.random.default_rng(0).normal(size=len(t))
    tt, play = TH.catmull_play(t, v)
    assert np.allclose(W.catmull_at(t, v, tt), play)
    assert np.allclose(W.catmull_at(t, v, t), v)                 # through the frames


def _excitation(n_win=40, seed=0):
    """A synthetic walk: one excitation window per 4 s step, on the 50 Hz grid."""
    spec = TH.wiggle_spec(1.0, "roll", seed=seed)
    win = [(4000.0 * k + 200.0, 4000.0 * k + 1500.0) for k in range(n_win)]
    t_kf = np.arange(0.0, 4000.0 * n_win, 100.0)
    n, _ = TH.wiggle_ticks(t_kf, win, spec)
    v = n / TH.TICKS_PER_DEG
    T = np.arange(0.0, 4.0 * n_win, W.DT)
    return T, t_kf, v, win, W.catmull_at(t_kf, v, 1e3 * T)


def test_iv_gain_ignores_drift_and_the_robots_own_motion():
    rng = np.random.default_rng(1)
    T, _, _, win, u = _excitation()
    h = np.r_[0.0, 0.2, 0.5, 0.3, -0.1] * 0.6                       # the plant's response to u: a short FIR
    zt = np.convolve(u, h)[:len(u)]
    wins = [(T >= a / 1e3) & (T <= b / 1e3 + 0.3) for a, b in win]
    sz = np.std(zt[np.any(wins, 0)])
    drift = W.bp(rng.normal(size=len(T)), (0.5, 4.0))               # the replays drifting apart, not driven by u
    drift *= sz / np.std(drift)
    z = zt + drift
    noise = W.bp(rng.normal(size=len(T)), (0.5, 4.0))
    own = noise / np.std(noise) * 1.5 * sz + 0.8 * drift           # louder than zt, and partly the drift
    G = 1.3
    e = G * zt + own
    S, _ = W.read_channel([(u, e, z, wins)])
    r = W.pool(S)
    assert abs(r["g0"] - G) < 0.08 and 0.05 < r["coh0"] < 0.6
    lo, hi = W.boot(S)
    assert lo < r["g0"] < hi
    # the plain regression on z would have read the drift as gain
    m = np.any(wins, 0)
    naive = float(W.bp(e)[m] @ W.bp(z)[m]) / float(W.bp(z)[m] @ W.bp(z)[m])
    assert abs(naive - G) > 0.15 > 3 * abs(r["g0"] - G)


def test_align_finds_each_window_through_a_hold():
    T, t_kf, v, win, _ = _excitation(n_win=8, seed=2)
    rng = np.random.default_rng(3)
    true = [0.4] * 3 + [0.4 + 1.37] * 5                             # a lift gate held the clock 1.37 s before window 4
    u = np.zeros(len(T))
    for (a, b), d in zip(win, true):
        tq = 1e3 * (T - d)
        m = (tq >= a - 150) & (tq <= b + 150)
        u[m] += W.catmull_at(t_kf, v, tq[m])
    gait = 3.0 * np.sin(2 * math.pi * 0.25 * T) + 0.02 * rng.normal(size=len(T))   # slow gait moves + a little noise
    out = W.align(gait + u, T, t_kf, v, win, delta0=0.0)
    for (d, r, amp), d0 in zip(out, true):
        assert abs(d - d0) <= 0.003 and r > 0.9 and abs(amp - 1.0) < 0.1
    back = W.u_on_grid(T, t_kf, v, win, out)
    assert np.max(np.abs(back - u)) < 0.15 * np.max(np.abs(u))


def test_template_from_the_log():
    frames = [[300, 5], [400, -11], [900, 6]]
    t, v = W.template(frames, 1000.0)
    assert t[0] == 0 and np.all(np.diff(t) == 100.0)
    assert v[3] == 5 / TH.TICKS_PER_DEG and v[4] == -11 / TH.TICKS_PER_DEG and v[9] == 6 / TH.TICKS_PER_DEG
    assert np.count_nonzero(v) == 3


def test_excite_windows_follow_track_host():
    w = W.excite_windows([(0.0, 1.8), (4.0, 5.8), (8.0, 8.9)])
    assert w[0] == pytest.approx((0.1, 1.4)) and w[1] == pytest.approx((4.1, 5.4)) and len(w) == 2   # too short: none
