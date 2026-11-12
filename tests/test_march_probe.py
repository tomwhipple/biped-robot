"""grade.py's march probe (experiments/policy/shaper-train): the schedule is the robot's march5
script at the policy's clock, the metrics read per-step lifts and the weight transfer correctly,
and the channels a run never saw move are pinned as the firmware pins them."""
import math
import os
import pickle
import sys
import types

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
for p in ("experiments/policy/shaper-train", "experiments/policy/stand-still", "experiments/policy/rca",
          "link", "sim"):
    sys.path.insert(0, os.path.join(ROOT, p))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

import grade as G  # noqa: E402
import sources  # noqa: E402

DT = G.DT


def test_probe_schedule_is_the_robots_script_at_the_clock():
    sched = G.at_clock(G.PROBES["march5 robot"], 0.75)["cmd_schedule"]
    assert isinstance(sched, G.March) and sched[0] == (0.0, G.STAND_CMD) and sched[-1][1] == G.STAND_CMD
    src = sources.ScriptSource("march5")
    for ts, c in sched[1:-1]:
        ext = src.poll(ts)[3]
        assert (c[4], c[6]) == (ext["lift"], pytest.approx(ext["foot_dz"], abs=1e-9))
        assert c[:3] == (0.0, 0.0, 0.0) and c[3] == 1.0
    # five full swings and the quarter-cycle lead-in at 0.75 Hz: 3.67 s of march from t = 2 s
    assert sched[-1][0] == pytest.approx(2.0 + 2.75 / 0.75, abs=DT)
    assert sched[1][0] == 2.0


@pytest.mark.parametrize("hz", [0.6, 0.75, 1.0, 1.5])
def test_schedule_retimes_to_the_clock(hz):
    s = G.at_clock(G.PROBES["march5 robot"], hz)["cmd_schedule"]
    assert s[-1][0] == pytest.approx(2.0 + 2.75 / hz, abs=DT)
    lifts = [c[4] for _, c in s[1:-1]]
    flips = [k for k in range(1, len(lifts)) if lifts[k] != lifts[k - 1]]
    assert len(flips) == 5                       # L -> R -> L -> R -> L -> R: five changes, six runs
    assert (flips[1] - flips[0]) * DT == pytest.approx(0.5 / hz, abs=DT)
    assert pickle.loads(pickle.dumps(s)) == s


def test_march_probes_leave_the_other_probes_alone():
    assert G.PROBES["stride1 0.3 robot"]["cmd_schedule"][-1][0] == pytest.approx(2.0 + 1 / 1.5)
    assert "march" not in G.PROBES["steps5 0.3 robot"]


def _rows(lift_L, lift_R, com_f, n=400):
    """FT rows (tick x 8) for a standing robot: soles at z0 + lift, feet 6 cm either side, the CoM
    at fraction com_f of the way from the midpoint to the L foot (+1 over L)."""
    t = np.arange(n)
    z0 = 0.01
    ft = np.zeros((n, 8))
    ft[:, 0] = z0 + lift_L(t)
    ft[:, 1] = z0 + lift_R(t)
    ft[:, 3], ft[:, 5] = 0.06, -0.06
    ft[:, 7] = 0.06 * com_f(t)
    return ft


def _tick_bump(t, a, b, cm):
    return np.where((t >= a) & (t < b), cm / 100.0 * np.sin(math.pi * (t - a) / (b - a)), 0.0)


def test_march_metrics_reads_per_step_lift_and_shift():
    n = 400
    c4 = np.zeros(n)
    sched = [(0.0, G.STAND_CMD)]
    # a march of 1 lead-in + 4 full swings from tick 100 (entry 101 acts from tick 100), 33 ticks each
    edges = [100, 117, 150, 183, 216, 249]
    sides = [-1.0, 1.0, -1.0, 1.0, -1.0]
    for a, b, sd in zip(edges[:-1], edges[1:], sides):
        c4[a:b] = sd
        for k in range(a, b):
            sched.append(((k + 1) * DT, (0, 0, 0, 1.0, sd, 0, 0.0)))
    sched.append(((edges[-1] + 1) * DT, G.STAND_CMD))
    # L bumps 4.0 / 3.5 cm on its two full swings, R 2.5 / 3.2 cm: R's first full step is short
    L = lambda t: _tick_bump(t, 150, 183, 3.5) + _tick_bump(t, 216, 249, 4.0) + _tick_bump(t, 100, 117, 5.0)
    R = lambda t: _tick_bump(t, 117, 150, 2.5) + _tick_bump(t, 183, 216, 3.2)
    # CoM toward the stance foot while a foot is up: L up -> toward R (f -0.7), R up -> toward L (+0.5)
    f = lambda t: np.where((t >= 150) & (t < 183), -0.7, np.where((t >= 117) & (t < 150), 0.5, 0.0)) \
        + np.where((t >= 216) & (t < 249), -0.7, 0.0) + np.where((t >= 183) & (t < 216), 0.5, 0.0)
    ft = _rows(L, R, f, n)
    X = np.linspace(0, 0.03, n)
    Y = np.linspace(0, -0.02, n)
    YW = np.linspace(0, math.radians(4.0), n)
    m = G.march_metrics(ft, c4, X, Y, YW, sched, 0.75, fell=False)
    assert m["march_lead_cm"] == pytest.approx(5.0, abs=0.2)
    assert [s for s, _ in m["march_steps"]] == ["R", "L", "R", "L"] and m["march_n"] == 4
    peaks = [p for _, p in m["march_steps"]]
    assert peaks == pytest.approx([2.5, 3.5, 3.2, 4.0], abs=0.15)
    assert m["march_min_cm"] == pytest.approx(2.5, abs=0.15)
    assert m["march_min_l_cm"] == pytest.approx(3.5, abs=0.15) and m["march_min_r_cm"] == pytest.approx(2.5, abs=0.15)
    assert m["march_steps_ok"] == 3                 # the 2.5 cm step misses the 3 cm bar
    assert m["march_shift_l"] == pytest.approx(-0.7, abs=0.02) and m["march_shift_r"] == pytest.approx(0.5, abs=0.02)
    assert m["march_shift_ok"] is False             # |0.5| < 0.6
    assert m["march_drift_m"] == pytest.approx(math.hypot(0.03 * 300 / 399, 0.02 * 300 / 399), abs=2e-3)
    assert m["march_yaw_deg"] == pytest.approx(4.0 * 300 / 399, abs=0.1)
    assert m["march_pass"] is False and m["march_lifts_pass"] is False
    # raise the short step and the shift: the same march passes; a fall fails it
    R2 = lambda t: _tick_bump(t, 117, 150, 3.4) + _tick_bump(t, 183, 216, 3.2)
    f2 = lambda t: f(t) * np.where(f(t) > 0, 1.4, 1.0)
    m2 = G.march_metrics(_rows(L, R2, f2, n), c4, X, Y, YW, sched, 0.75, fell=False)
    assert m2["march_pass"] is True and m2["march_shift_r"] == pytest.approx(0.7, abs=0.02)
    assert G.march_metrics(_rows(L, R2, f2, n), c4, X, Y, YW, sched, 0.75, fell=True)["march_pass"] is False
    # torso roll (+ = left side down): sags 3 deg toward L while L is up, leans 2 deg left (away) while R is up
    t = np.arange(n)
    up_l = L(t) > G.UP_CM
    up_r = R(t) > G.UP_CM
    RL = np.where(up_l & ~up_r, 3.0, 0.0) + np.where(up_r & ~up_l, 2.0, 0.0)
    m3 = G.march_metrics(ft, c4, X, Y, YW, sched, 0.75, fell=False, RL=RL)
    assert m3["march_roll_l_deg"] == pytest.approx(3.0) and m3["march_roll_l_max"] == pytest.approx(3.0)
    assert m3["march_roll_r_deg"] == pytest.approx(-2.0) and m3["march_roll_r_max"] == pytest.approx(2.0)
    assert "march_roll_l_deg" not in m                # no roll trace, no roll keys


def test_feet_metrics_is_the_planners_shift_when_up():
    n = 300
    L = lambda t: _tick_bump(t, 100, 150, 4.0)
    R = lambda t: _tick_bump(t, 180, 230, 2.0)
    f = lambda t: np.where((t >= 100) & (t < 150), -0.4, 0.0) + np.where((t >= 180) & (t < 230), 0.3, 0.0)
    m = G.feet_metrics(_rows(L, R, f, n))
    assert m["lift_cm"] == pytest.approx([4.0, 2.0], abs=0.15)
    assert m["lift_steps"] == [1, 1]
    assert m["shift_when_up"] == pytest.approx([-0.4, 0.3], abs=0.02)
    assert G.feet_metrics(np.zeros((50, 8))) == {}


def test_frozen_channels_are_pinned_to_their_trained_value(tmp_path, monkeypatch):
    """The std-0 slots of the run's normaliser (the channels a run never saw move) are replaced by
    their mean, as policy::pinFrozen does; a run that moved c4 keeps it."""
    frame = 55
    for run, std_cmd in (("never", [0.2, 0.06, 0.46, 0.0, 0.0, 0.0, 0.0]),
                         ("marched", [0.2, 0.06, 0.46, 0.0, 0.5, 0.0, 0.01])):
        std = np.ones(3 * frame)
        mean = np.zeros(3 * frame)
        std[frame - 7:frame] = std_cmd
        mean[frame - 7:frame] = [0, 0, 0, 1.0, 0, 0, 0]
        (tmp_path / run).mkdir()
        with open(tmp_path / run / "params.pkl", "wb") as fh:
            pickle.dump((types.SimpleNamespace(std=std, mean=mean), None), fh)
    monkeypatch.setattr(G.simh, "run_root", lambda run: str(tmp_path))
    env = types.SimpleNamespace(obs_frame=frame)
    assert G.frozen_cmd("never", env) == {3: 1.0, 4: 0.0, 5: 0.0, 6: 0.0}
    assert G.frozen_cmd("marched", env) == {3: 1.0, 5: 0.0}
