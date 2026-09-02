"""Referee-side act-lag: walker_env can now apply the MEASURED servo
action-chain lag (3 cascaded first-order stages, the same law env_mjx trains
--act-lag DR against). The lag-less columns must stay bit-identical, the
cascade must follow the analytic law, and the walker/MJX laws must match --
otherwise the referee's "measured servo" is a different servo than the one
the policy trained for."""
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

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def _walker(**kw):
    from walker_env import BimoWalkerEnv
    base = dict(xml_path=XML, command_mode=True, ext_cmd=True,
                actuator_model="sts3215", imu_obs=True,
                domain_rand=False, latency_ms=0.0, backlash_deg=0.0,
                quantize_ticks=False)
    base.update(kw)
    return BimoWalkerEnv(**base)


def _cascade(target_seq, y0, f_hz, dt):
    """Analytic 3-stage first-order cascade, the shared law."""
    k = 1.0 - np.exp(-2.0 * np.pi * f_hz * dt)
    y = np.stack([np.asarray(y0, dtype=np.float64)] * 3)
    out = []
    for t in target_seq:
        y[0] += k * (t - y[0])
        y[1] += k * (y[0] - y[1])
        y[2] += k * (y[1] - y[2])
        out.append(y[2].copy())
    return out


def test_off_by_default_passthrough():
    """act_lag_hz defaults 0: ctrl is exactly the mapped action target and
    the cascade state is never even allocated -- legacy scorecards are
    untouched."""
    env = _walker()
    assert env.act_lag_hz == 0.0
    env.reset(seed=0)
    a = 0.4 * np.ones(env.action_space.shape[0], dtype=np.float32)
    env.step(a)
    assert env._lag_y is None
    np.testing.assert_allclose(env.data.ctrl, env._action_to_ctrl(a),
                               rtol=0, atol=1e-6)


def test_cascade_matches_analytic_law():
    """With a 2 Hz pole the ctrl the plant sees must be the analytic
    3-stage response, seeded at the default pose like env_mjx."""
    env = _walker(act_lag_hz=2.0)
    env.reset(seed=0)
    a = 0.6 * np.ones(env.action_space.shape[0], dtype=np.float32)
    target = env._action_to_ctrl(a).astype(np.float64)
    want = _cascade([target] * 8, env._default, 2.0, env.control_dt)
    for i in range(8):
        env.step(a)
        np.testing.assert_allclose(env.data.ctrl, want[i], rtol=0, atol=1e-5,
                                   err_msg=f"step {i}")


def test_reset_reseeds_cascade():
    """A new episode must not inherit the previous episode's filter state."""
    env = _walker(act_lag_hz=2.0)
    env.reset(seed=0)
    a = np.ones(env.action_space.shape[0], dtype=np.float32)
    for _ in range(20):
        env.step(a)
    env.reset(seed=1)
    assert env._lag_y is None
    b = -0.5 * np.ones(env.action_space.shape[0], dtype=np.float32)
    env.step(b)
    target = env._action_to_ctrl(b).astype(np.float64)
    want = _cascade([target], env._default, 2.0, env.control_dt)[0]
    np.testing.assert_allclose(env.data.ctrl, want, rtol=0, atol=1e-5)


def test_law_matches_mjx():
    """env_mjx with a FIXED act_lag pole must hold the same lag_y state the
    walker law predicts after one step -- same k, same seeding, same order."""
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(xml_path=XML, ext_cmd=True, obs_hist_len=3,
                     domain_rand=False, imu_noise=0.0, quantize_ticks=False,
                     latency_ms=0.0, backlash_deg=0.0,
                     act_lag_hz=2.0,
                     cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02))
    s = env.reset(jax.random.PRNGKey(3))
    a = 0.5 * jp.ones(env.action_size)
    s2 = env.step(s, a)
    # reproduce the mapped target (action_map default) then the cascade
    a_np = np.asarray(a)
    default = np.asarray(env._default, dtype=np.float64)
    if env.action_map == "full":
        hi = np.asarray(env._hi)
        lo = np.asarray(env._lo)
        span = np.where(a_np >= 0.0, hi - default, default - lo)
        target = default + span * a_np
    else:
        target = np.clip(default + np.asarray(env._scale) * a_np,
                         np.asarray(env._lo), np.asarray(env._hi))
    want = _cascade([target], default, 2.0, env.control_dt)[0]
    got = np.asarray(s2.lag_y[2], dtype=np.float64)
    np.testing.assert_allclose(got, want, rtol=0, atol=1e-4)
