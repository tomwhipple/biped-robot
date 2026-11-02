"""The firmware command shaper, goal speed, stand-clock coast and stand-squeeze penalty in
the TRAINING env (env_mjx, 2026-11-02 shaper-train).

  * fw_shaper_hz must be obs::CommandShaper exactly: 3 cascaded first-order lags,
    k = 1 - exp(-2 pi f dt), seeded at the start pose -- the chain the robot runs;
  * the per-episode pole draw stays in its range; 0 Hz is a pass-through;
  * clock_stand_coast is obs::GaitClock::coastTo: at a plain stand the clock keeps its
    rate until the stand phase, then holds there; a walk command runs it; walker_env
    (the referee) follows the same rule;
  * w_squeeze reports a finite, non-negative stand squeeze; goal speed leaves the
    step finite and only ever slows the served target.
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import jax  # noqa: E402
import jax.numpy as jp  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")
STAND = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
WALK = (0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


def _mjx(**kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, obs_hist_len=3, domain_rand=False,
                imu_noise=0.0, quantize_ticks=False, latency_ms=0.0,
                backlash_deg=0.0, cmd_fixed=STAND)
    base.update(kw)
    return BimoMJXEnv(**base)


def _shaper_law(targets, y0, hz, dt):
    """obs::CommandShaper::update (firmware/components/obs/actuation.cpp)."""
    k = 1.0 - math.exp(-2.0 * math.pi * hz * dt)
    y = np.stack([np.asarray(y0, np.float64)] * 3)
    for t in targets:
        y[0] += k * (t - y[0])
        y[1] += k * (y[0] - y[1])
        y[2] += k * (y[1] - y[2])
    return y


def _targets(env, a):
    a = np.asarray(a, np.float64)
    d, hi, lo = (np.asarray(x, np.float64) for x in (env._default, env._hi, env._lo))
    if env.action_map == "full":
        return d + np.where(a >= 0.0, hi - d, d - lo) * np.clip(a, -1, 1)
    return np.clip(d + np.asarray(env._scale, np.float64) * np.clip(a, -1, 1), lo, hi)


def test_shaper_matches_firmware_law():
    env = _mjx(fw_shaper_hz=10.0)
    s = env.reset(jax.random.PRNGKey(1))
    y0 = np.asarray(s.shp_y[0], np.float64)
    acts = [0.3, -0.2, 0.5, 0.5, 0.1]
    step = jax.jit(env.step)
    for a in acts:
        s = step(s, a * jp.ones(env.action_size))
    want = _shaper_law([_targets(env, a * np.ones(env.action_size)) for a in acts],
                       y0, 10.0, env.control_dt)
    np.testing.assert_allclose(np.asarray(s.shp_y), want, rtol=0, atol=2e-5)
    # nothing downstream (no dead time, no act-lag): the PD is served the shaped goal
    np.testing.assert_allclose(np.asarray(s.last_target), want[2], rtol=0, atol=2e-5)


def test_shaper_off_is_bit_identical():
    a = 0.4 * jp.ones(_mjx().action_size)
    s0 = _mjx().step(_mjx().reset(jax.random.PRNGKey(2)), a)
    e1 = _mjx(fw_shaper_hz=0.0, fw_shaper_hz_max=0.0)   # drawn 0 Hz = pass-through
    s1 = e1.step(e1.reset(jax.random.PRNGKey(2)), a)
    np.testing.assert_array_equal(np.asarray(s0.last_target), np.asarray(s1.last_target))


def test_pole_draw_in_range():
    env = _mjx(fw_shaper_hz=6.0, fw_shaper_hz_max=15.0)
    hz = np.array([float(env.reset(jax.random.PRNGKey(i)).shp_hz) for i in range(12)])
    assert np.all((hz >= 6.0) & (hz <= 15.0)) and hz.std() > 0.5


def test_act_lag_order_1_is_first_order():
    env = _mjx(act_lag_hz=5.0, act_lag_order=1)
    s = env.reset(jax.random.PRNGKey(3))
    y0 = np.asarray(s.lag_y[0], np.float64)
    a = 0.5 * jp.ones(env.action_size)
    s = env.step(s, a)
    k = 1.0 - math.exp(-2.0 * math.pi * 5.0 * env.control_dt)
    want = y0 + k * (_targets(env, np.asarray(a)) - y0)
    np.testing.assert_allclose(np.asarray(s.last_target), want, rtol=0, atol=2e-5)


def _phases(env, s, n, cmd=None):
    out = []
    step = jax.jit(env.step)
    for _ in range(n):
        if cmd is not None:
            s = s._replace(cmd=jp.asarray(cmd, jp.float32))
        s = step(s, jp.zeros(env.action_size))
        out.append(float(s.gait_phase))
    return np.array(out), s


def test_clock_coast_holds_at_stand_phase_and_runs_under_walk():
    env = _mjx(gait_clock=True, clock_stand_coast=math.pi / 2)
    s = env.reset(jax.random.PRNGKey(4))
    ph, s = _phases(env, s, 60)              # <= one cycle at >= 1.25 Hz is 40 ticks
    assert abs(ph[-1] - math.pi / 2) < 1e-6 and np.all(np.abs(ph[-10:] - ph[-1]) < 1e-6)
    i = int(np.argmax(np.abs(ph - math.pi / 2) < 1e-6))
    d = np.diff(np.unwrap(ph[:i + 1]))
    # coasting: every step before the hold is the trained rate (never faster); the last
    # step lands exactly on the target (at most one step)
    rate = 2 * math.pi * env.control_dt * float(s.gait_freq)
    assert np.all(d[:-1] > 0.99 * rate) and np.all(d <= rate * 1.0001)
    ph2, _ = _phases(env, s, 5, cmd=WALK)
    assert np.all(np.abs(np.diff(np.unwrap(ph2))) > 0.5 * rate)


def test_walker_env_coast_mirror():
    from walker_env import BimoWalkerEnv
    env = BimoWalkerEnv(xml_path=XML, command_mode=True, ext_cmd=True,
                        actuator_model="sts3215", imu_obs=True, domain_rand=False,
                        latency_ms=0.0, backlash_deg=0.0, quantize_ticks=False,
                        gait_clock=True, clock_stand_coast=math.pi / 2)
    env.reset(seed=0)
    env.set_command(*STAND)
    for _ in range(60):
        env.step(np.zeros(env.action_space.shape[0], np.float32))
    assert abs(env._gait_phase - math.pi / 2) < 1e-6


@pytest.mark.parametrize("gs", [False, True])
def test_squeeze_metric_and_goal_speed(gs):
    env = _mjx(fw_shaper_hz=10.0, fw_goal_speed=gs, w_squeeze=1.0)
    s = env.reset(jax.random.PRNGKey(5))
    step = jax.jit(env.step)
    for a in (0.6, -0.6, 0.6):
        s = step(s, a * jp.ones(env.action_size))
        sq = float(s.metrics["squeeze"])
        assert np.isfinite(sq) and sq >= 0.0 and np.isfinite(float(s.reward))
    if gs:
        # the slewed set point never passes the shaped goal
        assert np.all(np.abs(np.asarray(s.last_target - s.shp_y[2])) < 0.2)


def test_loop_gain_dr_amplifies_the_deviation_only():
    """loop_gain_range: served target = mean + g x (action - mean), the probe's law
    (stand-still stand_sim out_gain); prev_action stays the raw action."""
    env = _mjx(loop_gain_range=(1.5, 1.5))
    s = env.reset(jax.random.PRNGKey(6))
    assert float(s.lg_gain) == 1.5
    step = jax.jit(env.step)
    k = 1.0 - math.exp(-env.control_dt / 0.5)
    ema = None
    for a in (0.1, 0.3, -0.2):
        av = a * np.ones(env.action_size)
        s = step(s, jp.asarray(av, jp.float32))
        ema = av.copy() if ema is None else ema + k * (av - ema)
        want = _targets(env, np.clip(ema + 1.5 * (av - ema), -1, 1))
        np.testing.assert_allclose(np.asarray(s.last_target), want, rtol=0, atol=2e-5)
        np.testing.assert_allclose(np.asarray(s.prev_action), av, rtol=0, atol=1e-6)
