"""Knee-only crouch gate for the mimic term (2026-09-10). The whole-term
gate (mimic_crouch_gate) let the knees answer a crouch but cost cadence,
because the mimic reference carries leg-swing timing. The knee-only gate
must (a) zero exactly the knee + ankle components while crouching, (b) keep
every component when standing tall, and (c) be off by default."""
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


def _env(crouch, **kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, gait_clock=True, obs_hist_len=3,
                domain_rand=False,
                imu_noise=0.0, quantize_ticks=False, latency_ms=0.0,
                backlash_deg=0.0, w_mimic=1.5, mimic_knee_w=4.0,
                cmd_fixed=(0.0, 0.0, 0.0, crouch, 0.0, 0.0, 0.02))
    base.update(kw)
    return BimoMJXEnv(**base)


def test_off_by_default_and_exclusive():
    env = _env(1.0)
    assert env.mimic_crouch_gate_knee is False
    import pytest
    with pytest.raises(ValueError):
        _env(1.0, mimic_crouch_gate=True, mimic_crouch_gate_knee=True)


def test_keep_mask_zeroes_knee_and_ankle_only():
    env = _env(1.0, mimic_crouch_gate_knee=True)
    keep = np.asarray(env._mimic_keep)
    zero = set(np.where(keep == 0.0)[0].tolist())
    want = set(np.asarray(env._i_knee).tolist()) | set(
        np.asarray(env._i_ankle).tolist())
    assert zero == want, (zero, want)
    assert (keep[list(set(range(len(keep))) - zero)] == 1.0).all()


def _mimic_income(env, q_offsets, crouch):
    """Reproduce the mimic kernel the reward block computes for a joint
    deviation vector, under the env's gating rule."""
    w = np.asarray(env._mimic_w)
    if env.mimic_crouch_gate_knee and crouch < 0.97:
        w = w * np.asarray(env._mimic_keep)
    return float(env.w_mimic * np.exp(-np.sum(w * q_offsets ** 2)
                                      / env.mimic_s2))


def test_knee_error_is_free_only_while_crouching():
    """A pure knee deviation costs mimic income when standing tall and
    nothing while crouching; a hip deviation costs in both cases."""
    env = _env(0.8, mimic_crouch_gate_knee=True)
    n = env._nq_act
    knee_dev = np.zeros(n)
    knee_dev[np.asarray(env._i_knee)] = 0.4
    hip_dev = np.zeros(n)
    hip_dev[np.asarray(env._i_pitch)] = 0.4
    full = _mimic_income(env, np.zeros(n), 0.8)
    assert abs(_mimic_income(env, knee_dev, 0.8) - full) < 1e-9
    assert _mimic_income(env, hip_dev, 0.8) < full - 1e-6
    assert _mimic_income(env, knee_dev, 1.0) < full - 1e-6


def test_env_steps_with_gate():
    """The gated env runs end-to-end under jit with a crouch command."""
    env = _env(0.8, mimic_crouch_gate_knee=True)
    s = env.reset(jax.random.PRNGKey(0))
    step = jax.jit(env.step)
    for _ in range(3):
        s = step(s, jp.zeros(env.action_size))
    assert np.isfinite(float(s.reward))
