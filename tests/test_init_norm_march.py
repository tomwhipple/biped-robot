"""--init-norm-march: every slow-gait student so far drew no march, so its observation normalizer
holds std 1e-6 on the lift (c4) and foot-dz (c6) command slots (the channels never moved; measured
on robot_legs_prec16_f075_js20: std 1e-6 on c3..c6, count 3.1e9). A warm start with --march-mix
must re-seat them or a march reads as ~1e6 sigma. Pinned like the crouch re-seat: the slots are where
env_mjx writes c4 / c6 in every stacked frame, and only they change."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--action-map", "full",
        "--march-mix", "0.15", "--init-norm-march"]


def _env(*extra):
    import train_mjx
    from env_mjx import BimoMJXEnv
    a = train_mjx.make_parser().parse_args(ARGS + list(extra))
    assert a.init_norm_march
    return train_mjx, BimoMJXEnv(**train_mjx.build_env_kw(a))


def _restore(env, idx_frozen):
    import jax.numpy as jp
    from brax.training.acme import running_statistics, specs
    st = running_statistics.init_state(specs.Array((env.obs_size,), jp.float32))
    obs = np.zeros((4, env.obs_size), np.float32)
    obs[:, 0] = np.arange(4)                                # one ordinary channel
    st = running_statistics.update(st, jp.asarray(obs))     # every other channel frozen at 0
    return (st, "policy", "value")


def test_flag_off_by_default():
    import train_mjx
    assert not train_mjx.make_parser().parse_args([]).init_norm_march


def test_slots_hold_the_march_command():
    import jax
    import jax.numpy as jp
    train_mjx, env = _env("--march-lift", "0.08", "--ext-mix", "0,0,0,0,0,0,0", "--march-mix", "1.0")
    s4, s6 = train_mjx.march_obs_slots(env, 4), train_mjx.march_obs_slots(env, 6)
    assert len(s4) == len(s6) == env.obs_hist_len
    step = jax.jit(env.step)
    st = env.reset(jax.random.PRNGKey(1))
    for _ in range(14):
        st = step(st, jp.zeros(env.action_size))
    c, o = np.asarray(st.cmd), np.asarray(st.obs)
    assert float(st.traj_on) == 4.0 and c[4] != 0.0 and c[6] > 0.0
    # the newest frame carries this tick's command at the slots
    assert any(abs(float(o[i]) - c[4]) < 1e-5 for i in s4)
    assert any(abs(float(o[i]) - c[6]) < 1e-5 for i in s6)


def test_reseat_only_touches_the_march_slots():
    train_mjx, env = _env("--march-lift", "0.08")
    st, pol, val = _restore(env, None)
    new, idx = train_mjx.reseat_march_norm((st, pol, val), env)
    assert sorted(idx) == sorted(train_mjx.march_obs_slots(env, 4) + train_mjx.march_obs_slots(env, 6))
    assert new[1:] == (pol, val)
    m, s = np.asarray(new[0].mean), np.asarray(new[0].std)
    p, dz = 0.15, env._march_dz
    assert dz == pytest.approx(0.02)
    s4 = train_mjx.march_obs_slots(env, 4)
    s6 = train_mjx.march_obs_slots(env, 6)
    assert np.allclose(m[s4], 0.0) and np.allclose(s[s4], np.sqrt(p), rtol=1e-5)
    e1, e2 = p * dz * 2 / np.pi, p * dz * dz / 2
    assert np.allclose(m[s6], e1, rtol=1e-4) and np.allclose(s[s6], np.sqrt(e2 - e1 * e1), rtol=1e-4)
    keep = [i for i in range(env.obs_size) if i not in idx]
    assert np.allclose(m[keep], np.asarray(st.mean)[keep]) and np.allclose(s[keep], np.asarray(st.std)[keep])
    # a march's c4 = 1 reads as ~2.6 sigma, not ~1e6 (the std the frozen slot held)
    assert 2.0 < (1.0 - m[s4[0]]) / s[s4[0]] < 3.2


def test_reseat_leaves_c6_frozen_when_the_march_asks_none():
    """--march-lift 0.06: the swing target is lift_height, c6 stays 0 and stays pinned on the robot."""
    train_mjx, env = _env("--march-lift", "0.06")
    assert env._march_dz == pytest.approx(0.0, abs=1e-12)
    st = _restore(env, None)
    new, idx = train_mjx.reseat_march_norm(st, env)
    assert sorted(idx) == sorted(train_mjx.march_obs_slots(env, 4))
    s6 = train_mjx.march_obs_slots(env, 6)
    assert np.allclose(np.asarray(new[0].std)[s6], np.asarray(st[0].std)[s6])


def test_reseat_needs_a_march():
    import train_mjx
    from env_mjx import BimoMJXEnv
    a = train_mjx.make_parser().parse_args(ARGS[:-3] + ["--init-norm-march"])
    env = BimoMJXEnv(**train_mjx.build_env_kw(a))
    with pytest.raises(ValueError):
        train_mjx.reseat_march_norm(_restore(env, None), env)
