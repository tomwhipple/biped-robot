"""Per-step clearance penalty (w_low_swing, 2026-10-25).

Tom 2026-10-24: every foot lifts >= 3 cm on every step (the referee's
walk_steps_L/R). The night-4 walkers' steady steps clear 5-8 cm, and every
failing step is the first one after the walk command or the stop step: low
shuffles under commands the gait shaping (gated on a moving command) never
grades. The penalty charges each completed swing of >= 3 ticks its shortfall
below 3 cm, under any command. One jax-free kernel in walker_env is shared by
both envs (env_mjx imports it).
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

XML = os.path.join(SIM, "bimo_biped_v6legs.xml")


def test_constants_match_the_referee():
    import eval_precision as ep
    assert W.LOW_SWING_CLEAR == ep.SWING_CLEAR
    assert W.LOW_SWING_MIN_TICKS == ep.SWING_MIN_TICKS


def test_kernel_values():
    k = lambda p: float(W.low_swing_shortfall(p, np))
    assert k(0.0) == 1.0
    assert k(-0.002) == 1.0
    assert np.isclose(k(0.01), 2.0 / 3.0)
    assert k(0.03) == 0.0 and k(0.08) == 0.0


def test_kernel_numpy_matches_jax():
    import jax.numpy as jp
    p = np.array([-0.01, 0.0, 0.007, 0.0299, 0.03, 0.2])
    np.testing.assert_allclose(W.low_swing_shortfall(p, np),
                               np.asarray(W.low_swing_shortfall(jp.asarray(p, jp.float32), jp)),
                               atol=1e-6)


def _cpu_reward(w, air_ticks, peak):
    """Step reward of a standing CPU env whose left foot is touching down after
    a swing of air_ticks ticks that peaked at `peak` m."""
    env = W.BimoWalkerEnv(xml_path=XML, command_mode=True, ext_cmd=True,
                          actuator_model="sts3215", domain_rand=False,
                          w_low_swing=w)
    env.reset(seed=5)
    env._air_time[:] = [air_ticks * env.control_dt, 0.0]
    env._swing_peak[:] = [peak, 0.0]
    _, r, *_ = env.step(np.zeros(env.action_space.shape[0], np.float32))
    assert env._foot_contacts()[0]             # the left foot did land
    assert env._swing_peak[0] == 0.0           # and its peak was reset
    return r


@pytest.mark.parametrize("ticks, peak, pays", [
    (10, 0.01, 2.0 / 3.0),     # a 1 cm shuffle step
    (10, 0.0, 1.0),            # a slide that never rose
    (10, 0.05, 0.0),           # a real step
    (2, 0.0, 0.0),             # contact chatter, not a step
])
def test_cpu_env_pays_the_shortfall(ticks, peak, pays):
    d = _cpu_reward(2.0, ticks, peak) - _cpu_reward(0.0, ticks, peak)
    assert np.isclose(d, -2.0 * pays, atol=1e-6)


def test_mjx_env_pays_the_shortfall():
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    rewards = {}
    for w in (0.0, 2.0):
        e = BimoMJXEnv(xml_path=XML, ext_cmd=True, domain_rand=False, w_low_swing=w)
        st = jax.jit(e.reset)(jax.random.PRNGKey(5))
        st = st._replace(air_time=jp.asarray([10 * e.control_dt, 0.0], jp.float32),
                         swing_peak=jp.asarray([0.01, 0.0], jp.float32),
                         recovered=jp.ones(()))
        nxt = jax.jit(e.step)(st, jp.zeros(e.action_size))
        assert float(nxt.air_time[0]) == 0.0      # the left foot did land
        assert float(nxt.swing_peak[0]) == 0.0
        rewards[w] = float(nxt.reward)
    assert np.isclose(rewards[2.0] - rewards[0.0], -2.0 * 2.0 / 3.0, atol=1e-4)
