"""Reference-state initialization for the squat (2026-09-12): env slots that
start episodes IN the squat reference with a stationary crouch command. Off
by default = identical resets; on, the initial pose is the reference, the
torso is lowered by the feasible drop, the command is pinned and re-pinned
across the trainer's cached auto-reset."""
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
import pytest  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def _env(**kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, gait_clock=True, obs_hist_len=3,
                domain_rand=False, imu_noise=0.0, quantize_ticks=False,
                latency_ms=0.0, backlash_deg=0.0, crouch_pose_ref=True,
                cmd_crouch_range=(0.7, 1.0))
    base.update(kw)
    return BimoMJXEnv(**base)


def test_off_by_default_identical_reset():
    a = _env().reset(jax.random.PRNGKey(4)); b = _env(crouch_rsi_mix=0.0).reset(jax.random.PRNGKey(4))
    np.testing.assert_allclose(np.asarray(a.data.qpos), np.asarray(b.data.qpos), atol=0)
    np.testing.assert_allclose(np.asarray(a.cmd), np.asarray(b.cmd), atol=0)
    assert float(a.crouch_slot) == 0.0


def test_requires_pose_ref():
    with pytest.raises(ValueError):
        _env(crouch_pose_ref=False, crouch_rsi_mix=0.5)


def test_slot_starts_in_the_squat_and_repins():
    e = _env(crouch_rsi_mix=1.0)
    s = e.reset(jax.random.PRNGKey(0))
    assert float(s.crouch_slot) == 1.0
    c = float(s.crouch_c); assert 0.7 <= c < 0.95
    cmd = np.asarray(s.cmd); assert cmd[3] == pytest.approx(c) and abs(cmd[:3]).max() == 0.0
    q_ref = np.asarray(e._mimic_ref(jp.array([0, 0, 0, c, 0, 0, 0.0]), 0.0, 1.5))
    np.testing.assert_allclose(np.asarray(s.data.qpos[e._jq0:e._jq1]), q_ref, atol=1e-5)
    tall = _env().reset(jax.random.PRNGKey(0))
    drop = float(tall.data.qpos[2] - s.data.qpos[2])
    want = e._nominal_h - float(e._crouch_h_target(c))
    assert abs(drop - want) < 2e-3, (drop, want)
    # auto-reset keeps the slot and re-pins the stationary crouch command
    s2 = e.reseed(s, jax.random.PRNGKey(99))
    assert float(s2.crouch_slot) == 1.0 and np.asarray(s2.cmd)[3] == pytest.approx(c)
    # and it steps
    s3 = jax.jit(e.step)(s2, jp.zeros(e.action_size))
    assert np.isfinite(float(s3.reward)) and float(s3.done) == 0.0
