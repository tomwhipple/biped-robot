"""--yaw-kick-range (2026-11-06): body yaw rate kicks while a move is commanded, for the robot's step
yaw the sim under-reads (experiments/policy/yaw-gap/README.md). Off by default, bit-exact; the kick
changes the body's yaw rate only under a moving command. Also grade.py's stop-timing and robot-speed
probes, and the coast clock's `coast_back` hold."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "stand-still"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco"]


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def test_flags_reach_env_kw():
    assert "yaw_kick_range" not in _kw() and "yaw_kick_prob" not in _kw()
    kw = _kw("--yaw-kick-range", "0.5,2.5", "--yaw-kick-prob", "0.1")
    assert kw["yaw_kick_range"] == (0.5, 2.5) and kw["yaw_kick_prob"] == 0.1
    assert _kw("--yaw-kick-range", "1,2")["yaw_kick_prob"] == 0.03


def _roll(env, cmd=None, steps=4, key=3):
    import jax
    st = jax.jit(env.reset)(jax.random.PRNGKey(key))
    if cmd is not None:
        st = st._replace(cmd=st.cmd.at[:3].set(jax.numpy.asarray(cmd)))
    step = jax.jit(env.step)
    a = jax.numpy.zeros(env.action_size)
    for _ in range(steps):
        st = step(st, a)
    return st


@pytest.fixture(scope="module")
def envs():
    from env_mjx import BimoMJXEnv
    base = dict(_kw(), push_prob=0.0)
    return (BimoMJXEnv(**base),
            BimoMJXEnv(**dict(base, yaw_kick_range=(3.0, 3.0), yaw_kick_prob=1.0)))


def test_off_is_bit_exact(envs):
    from env_mjx import BimoMJXEnv
    off, _ = envs
    base = dict(_kw(), push_prob=0.0)
    np.testing.assert_array_equal(np.asarray(_roll(off).data.qvel),
                                  np.asarray(_roll(BimoMJXEnv(**dict(base, yaw_kick_range=(3.0, 3.0), yaw_kick_prob=0.0))).data.qvel))


def test_kicks_only_under_a_moving_command(envs):
    off, on = envs
    still = (0.0, 0.0, 0.0)
    np.testing.assert_array_equal(np.asarray(_roll(off, still).data.qvel), np.asarray(_roll(on, still).data.qvel))
    walk = (0.4, 0.0, 0.0)
    a, b = _roll(off, walk, steps=1), _roll(on, walk, steps=1)
    assert abs(float(b.data.qvel[5]) - float(a.data.qvel[5])) > 0.3      # the feet's grip eats part of 3 rad/s


def test_grade_probes_and_coast_hold():
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "shaper-train"))
    import grade as G
    import stand_sim as S
    assert G.PROBES["steps5 0.3 robot"]["typed"]["dead15"] == 60.0
    assert G.PROBES["steps5 0.3 robot dq5"]["dq_obs"]["window"] == 5
    assert G.PROBES["stride1 0.3 robot"]["plant"]["w0_scale"] == dict(hip_pitch=0.87, knee=1.0)
    t_on = G.PROBES["stride1 0.3 refit dqlag"]["cmd_schedule"][-1][0]
    assert G.PROBES["stride1 0.3 refit dqlag late40"]["cmd_schedule"][-1][0] == pytest.approx(t_on + 0.04)
    assert G.PROBES["stride1 0.3 refit dqlag early40"]["cmd_schedule"][-1][0] == pytest.approx(t_on - 0.04)
    p2 = math.pi / 2
    # 0.3 rad past the stand phase under the stand command: coast runs on, coast_back holds
    clk = dict(ph=p2 + 0.3, held=False, f=S.CLOCK_HZ)
    assert S.advance_clock(clk, "coast", dict(stand_phase=p2), G.STAND_CMD, 0.02) != pytest.approx(p2)
    clk = dict(ph=p2 + 0.3, held=False, f=S.CLOCK_HZ)
    assert S.advance_clock(clk, "coast", dict(stand_phase=p2, coast_back=1.0), G.STAND_CMD, 0.02) == pytest.approx(p2)
    assert clk["held"]
