"""Per-servo actuator response (2026-11-01 loaded step responses: the STS3215 joints sit 66-87 ms
dead behind a goal write, the STS3250 hip rolls 12-18 ms; experiments/one-leg-stand/
2026-11-01-floors-fit.md). act_delay_extra adds per-actuator dead time (control ticks) on top of
the common act_delay; act_lag_scale multiplies the act-lag pole per actuator. Both default off =
the old path bit for bit; on, walker_env and env_mjx serve the same per-servo targets."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import jax  # noqa: E402
import jax.numpy as jp  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v6legs.xml")


def _walker(**kw):
    from walker_env import BimoWalkerEnv
    base = dict(xml_path=XML, command_mode=True, ext_cmd=True, actuator_model="sts3215", imu_obs=True,
                domain_rand=False, latency_ms=0.0, backlash_deg=0.0, quantize_ticks=False)
    base.update(kw)
    return BimoWalkerEnv(**base)


def test_preset_resolves_per_role():
    env = _walker(act_delay_extra="sts2026", act_lag_scale="sts2026")
    for n, d, s in zip(env._servo_names, env._act_delay_vec, env._act_lag_scale):
        if n.endswith("hip_roll"):
            assert d == 0 and s == 4.0, n
        elif n[2:] in ("hip_yaw", "hip_pitch", "knee", "ankle", "ankle_roll"):
            assert d == 3 and s == 1.0, n


def test_off_is_the_old_path():
    env = _walker()
    assert not env._act_delay_vec.any() and np.all(env._act_lag_scale == 1.0)


def test_walker_per_servo_dead_time():
    """the hip rolls (extra 0) see a new target after act_delay_ticks; the others 3 ticks later."""
    env = _walker(act_delay_ticks=1, act_delay_extra="sts2026")
    env.reset(seed=0)
    hip = np.array([n.endswith("hip_roll") for n in env._servo_names])
    other = ~hip
    a0 = np.zeros(env.action_space.shape[0], np.float32)
    a1 = 0.5 * np.ones_like(a0)
    t1 = env._to_servo(env._action_to_ctrl(a1))
    env.step(a1)                                       # tick 0: everything still on the default
    np.testing.assert_allclose(env.data.ctrl, env._sdefault, atol=1e-6)
    env.step(a1)                                       # tick 1: the hip rolls move, the rest wait
    np.testing.assert_allclose(env.data.ctrl[hip], t1[hip], atol=1e-6)
    np.testing.assert_allclose(env.data.ctrl[other], env._sdefault[other], atol=1e-6)
    for _ in range(2):                                 # ticks 2, 3: the rest still wait (extra 3)
        env.step(a1)
        np.testing.assert_allclose(env.data.ctrl[other], env._sdefault[other], atol=1e-6)
    env.step(a1)
    np.testing.assert_allclose(env.data.ctrl, t1, atol=1e-6)   # tick 4 = 1 + 3: everyone


def test_mjx_matches_walker_dead_time_and_lag():
    from env_mjx import BimoMJXEnv
    kw = dict(act_delay_extra="sts2026", act_lag_scale="sts2026")
    env = BimoMJXEnv(xml_path=XML, ext_cmd=True, obs_hist_len=3, domain_rand=False, imu_noise=0.0,
                     quantize_ticks=False, latency_ms=0.0, backlash_deg=0.0, act_lag_hz=4.0,
                     cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02), **kw)
    assert env._act_hist_len == 4
    s = env.reset(jax.random.PRNGKey(3))
    a = 0.5 * jp.ones(env.action_size)
    lag = []
    for _ in range(5):
        s = env.step(s, a)
        lag.append(np.asarray(s.lag_y[2], np.float64))
    w = _walker(act_lag_hz=4.0, **kw)
    w.reset(seed=0)
    w._sdefault = np.asarray(env._sdefault, np.float64)     # same seed pose for the comparison
    w._lag_y = None
    w._act_hist = None
    aw = 0.5 * np.ones(w.action_space.shape[0], np.float32)
    for i in range(5):
        w.step(aw)
        np.testing.assert_allclose(w._lag_y[2], lag[i], atol=2e-4, err_msg=f"tick {i}")


def test_mjx_types_and_extra_compose():
    """ONE dead-time mechanism (#189 + #192): with --act-delay-types AND --act-delay-extra together,
    each servo serves the target from clip(its per-episode type draw + its fixed extra) ticks ago,
    and the ring is sized max draw + max extra + 1."""
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(xml_path=XML, ext_cmd=True, obs_hist_len=3, domain_rand=False, imu_noise=0.0,
                     quantize_ticks=False, latency_ms=0.0, backlash_deg=0.0,
                     cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02),
                     servo_types={"hip_roll": "sts3250"},
                     act_delay_types={"sts3215": (1, 2), "sts3250": (0, 1)}, act_delay_extra="sts2026")
    n = len(env._servo_names)
    assert env.act_delay_max == 2 and env._act_hist_len == 2 + 3 + 1
    extra = np.asarray(env._act_delay_extra)
    hip = np.array([s.endswith("hip_roll") for s in env._servo_names])
    assert np.all(extra[hip] == 0) and np.all(extra[~hip] == 3)
    step = jax.jit(env.step)
    for seed in (3, 11):
        s = env.reset(jax.random.PRNGKey(seed))
        draw = np.asarray(s.act_delay)
        assert draw.shape == (n,) and s.act_hist.shape == (env._act_hist_len, n)
        assert np.all((draw[hip] >= 0) & (draw[hip] <= 1)) and np.all((draw[~hip] >= 1) & (draw[~hip] <= 2))
        want = np.clip(draw + extra, 0, env._act_hist_len - 1)
        checked = 0
        for k in range(env._act_hist_len + 1):                # fill the ring with distinct targets
            s = step(s, 0.03 * (k + 1) * jp.ones(env.action_size))
            hist, served = np.asarray(s.act_hist), np.asarray(s.last_target)
        for j in range(n):
            col = hist[:, j]
            if len(np.unique(np.round(col, 7))) < len(col):    # held joint: rows not distinguishable
                continue
            got = np.flatnonzero(np.isclose(col, served[j], atol=1e-6))
            assert got.tolist() == [want[j]], (env._servo_names[j], got, want[j])
            checked += 1
        assert checked >= 8, checked
