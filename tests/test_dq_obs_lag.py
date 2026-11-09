"""--dq-obs-window / --dq-obs-quantum (2026-11-05): the robot's joint-velocity observation.

On the robot the policy's dq is the STS3215 speed register 58. Fit on 1308 moving samples it
is the MEAN joint velocity over the last ~40 ms (~20 ms late), in 50 steps/s (4.4 deg/s)
steps (hw_sessions/2026-11-05/kick_analysis.md section 3). The sim fed the instantaneous
qvel. With dq_obs_window = N the dq obs is (q_t - q_{t-N}) / (N control_dt) from a joint
history carried in the state (env_mjx State.dq_qhist; walker_env._dq_qhist), seeded at rest
at the start pose in every reset path; dq_obs_quantum = Q rounds it to Q steps/s. Off by
default, bit-exact.
"""
import inspect
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

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco"]
TICK_RAD = 2 * np.pi / 4096


def _kw(*extra):
    import train_mjx
    return train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS + list(extra)))


def test_cli_flags_parse_into_env_kw():
    kw = _kw()
    assert "dq_obs_window" not in kw and "dq_obs_quantum" not in kw      # off by default
    assert "dq_obs_window" not in _kw("--dq-obs-window", "0")
    assert _kw("--dq-obs-window", "2")["dq_obs_window"] == (2, 2)
    kw = _kw("--dq-obs-window", "1,3", "--dq-obs-quantum", "50")
    assert kw["dq_obs_window"] == (1, 3) and kw["dq_obs_quantum"] == 50.0


def test_defaults_off_and_parser():
    from env_mjx import BimoMJXEnv
    from walker_env import BimoWalkerEnv, parse_dq_obs_window
    for cls in (BimoMJXEnv, BimoWalkerEnv):
        sig = inspect.signature(cls.__init__)
        assert sig.parameters["dq_obs_window"].default is None
        assert sig.parameters["dq_obs_quantum"].default == 0.0
    assert parse_dq_obs_window(None) is None and parse_dq_obs_window(0) is None
    assert parse_dq_obs_window("0") is None and parse_dq_obs_window([0, 0]) is None
    assert parse_dq_obs_window(2) == (2, 2) and parse_dq_obs_window("1,3") == (1, 3)
    assert parse_dq_obs_window([2, 3]) == (2, 3)          # config.json stores a list
    for bad in ("3,1", -1, (0, 2), 99):
        with pytest.raises(ValueError):
            parse_dq_obs_window(bad)


# -- env_mjx -------------------------------------------------------------------
def _act(n):
    return 0.4 * np.sin(np.arange(n, dtype=np.float32) + 0.3)


def _mjx_roll(env, steps=6, key=3):
    import jax
    st = jax.jit(env.reset)(jax.random.PRNGKey(key))
    step = jax.jit(env.step)
    a = jax.numpy.asarray(_act(env.action_size))
    obs, q = [np.asarray(st.obs)], [np.asarray(env._p(st.data.qpos[env._jq0:env._jq1]))]
    for _ in range(steps):
        st = step(st, a)
        obs.append(np.asarray(st.obs))
        q.append(np.asarray(env._p(st.data.qpos[env._jq0:env._jq1])))
    return st, np.array(obs), np.array(q, dtype=np.float64)


@pytest.fixture(scope="module")
def mjx_off():
    from env_mjx import BimoMJXEnv
    return _mjx_roll(BimoMJXEnv(**_kw()))


def test_mjx_off_is_bit_exact(mjx_off):
    from env_mjx import BimoMJXEnv
    st0, obs0, _ = mjx_off
    st1, obs1, _ = _mjx_roll(BimoMJXEnv(**dict(_kw(), dq_obs_window=0, dq_obs_quantum=0.0)))
    np.testing.assert_array_equal(obs0, obs1)
    assert st0.dq_qhist.shape == (0, st0.prev_action.shape[0])
    assert int(st0.dq_win) == 0


@pytest.mark.parametrize("n", [2])
def test_mjx_dq_is_the_window_mean(mjx_off, n):
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**dict(_kw(), dq_obs_window=n))
    st, obs, q = _mjx_roll(env)
    nj = env._nq_act
    dt = env.control_dt
    dq = obs[:, nj:2 * nj]
    # the history starts at rest at the start pose: tick 0 reads exactly 0
    assert np.all(np.isfinite(dq)) and np.all(dq[0] == 0.0)
    for t in range(len(q)):
        ref = (q[t] - q[max(0, t - n)]) / (n * dt)
        np.testing.assert_allclose(dq[t], ref, atol=2e-4, rtol=1e-4)
    # the joints really move, and the mean lags the instantaneous qvel the default feeds
    _, obs_off, q_off = mjx_off
    np.testing.assert_allclose(q, q_off, atol=1e-6)           # the obs does not change physics
    assert np.abs(dq[-1]).max() > 0.2
    assert np.abs(dq[2:] - obs_off[2:, nj:2 * nj]).max() > 0.05
    # every other obs channel is untouched (the frame's own channels; the history frames
    # carry the old dq)
    frame = env.obs_frame
    np.testing.assert_array_equal(obs[:, :nj], obs_off[:, :nj])
    np.testing.assert_array_equal(obs[:, 2 * nj:frame], obs_off[:, 2 * nj:frame])


def test_mjx_quantum_rounds_the_window_mean():
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**dict(_kw(), dq_obs_window=2, dq_obs_quantum=50.0))
    _, obs, q = _mjx_roll(env)
    nj, dt, qq = env._nq_act, env.control_dt, 50 * TICK_RAD
    dq = obs[:, nj:2 * nj].astype(np.float64)
    steps = dq / qq
    np.testing.assert_allclose(steps, np.rint(steps), atol=1e-3)   # on the 50 steps/s grid
    ref = np.array([(q[t] - q[max(0, t - 2)]) / (2 * dt) for t in range(len(q))])
    assert np.abs(dq - ref).max() <= 0.5 * qq + 1e-3
    assert np.abs(dq).max() >= qq                                   # not all rounded to 0


def test_mjx_window_draw_and_reseed():
    import jax
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**dict(_kw(), dq_obs_window=(1, 3)))
    sts = jax.jit(jax.vmap(env.reset))(jax.random.split(jax.random.PRNGKey(0), 32))
    w = np.asarray(sts.dq_win)
    assert set(w.tolist()) == {1, 2, 3}
    assert sts.dq_qhist.shape == (32, 4, env._nq_act)
    st = jax.tree_util.tree_map(lambda x: x[0], sts)
    st = jax.jit(env.step)(st, jax.numpy.asarray(_act(env.action_size)))
    rs = jax.jit(env.reseed)(st, jax.random.PRNGKey(9))
    q0 = np.asarray(env._p(rs.data.qpos[env._jq0:env._jq1]))
    np.testing.assert_allclose(np.asarray(rs.dq_qhist), np.tile(q0, (4, 1)), atol=1e-6)
    nj = env._nq_act
    assert np.all(np.asarray(rs.obs)[nj:2 * nj] == 0.0)
    assert 1 <= int(rs.dq_win) <= 3


# -- walker_env (the CPU referee / grade.py twin) ------------------------------
def _walker(**over):
    import eval_precision as ep
    cfg = dict(_kw(), **over)
    env = ep.make_env(cfg, 4.0, True, cfg["xml_path"])
    return env


def _walker_roll(env, steps=6, seed=3):
    obs0, _ = env.reset(seed=seed)
    a = _act(env.action_space.shape[0])
    obs, q = [obs0], [np.array(env.data.qpos[env._jqpos])]
    for _ in range(steps):
        o, *_ = env.step(a)
        obs.append(o)
        q.append(np.array(env.data.qpos[env._jqpos]))
    return np.array(obs), np.array(q)


def test_walker_off_is_bit_exact_and_window_mean():
    e0 = _walker()
    assert e0.dq_obs_window is None and e0.dq_obs_quantum == 0.0
    obs0, q0 = _walker_roll(e0)
    obs0b, _ = _walker_roll(_walker(dq_obs_window=0, dq_obs_quantum=0.0))
    np.testing.assert_array_equal(obs0, obs0b)

    e2 = _walker(dq_obs_window=2)
    obs2, q2 = _walker_roll(e2)
    nj, dt = e2._nq_act, e2.control_dt
    np.testing.assert_allclose(q2, q0, atol=1e-12)            # physics untouched
    dq = obs2[:, nj:2 * nj]
    assert np.all(dq[0] == 0.0)
    for t in range(len(q2)):
        ref = (q2[t] - q2[max(0, t - 2)]) / (2 * dt)
        np.testing.assert_allclose(dq[t], ref, atol=1e-4, rtol=1e-5)
    assert np.abs(dq[-1]).max() > 0.2

    # grade.py flips it on a cached env per probe: set_dq_obs + reset
    e2.set_dq_obs(2, 50.0)
    obs5, q5 = _walker_roll(e2)
    st = obs5[:, nj:2 * nj].astype(np.float64) / (50 * TICK_RAD)
    np.testing.assert_allclose(st, np.rint(st), atol=1e-3)
    e2.set_dq_obs(None, 0.0)
    obs_back, _ = _walker_roll(e2)
    np.testing.assert_array_equal(obs_back, obs0)


def test_grade_dqlag_probes():
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "shaper-train"))
    import grade
    for p in ("stride1 0.3 refit dqlag", "steps5 0.3 refit dqlag"):
        assert grade.PROBES[p]["dq_obs"] == dict(window=2, quantum=50.0)
    base = grade.PROBES["steps5 0.3 refit"]["cmd_schedule"]
    assert base[1][0] == 2.0 and base[2][0] == pytest.approx(2.0 + 2.5 / grade.S.CLOCK_HZ)
