"""Reward terms for a real crouch (2026-09-11): w_hip_yaw (ungated yaw^2
penalty), w_crouch_track (tight depth kernel, only under a crouch command),
crouch_release (w_still / w_stand_com gates open while crouching). All off
by default so every existing recipe is bit-identical."""
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


def _env(crouch=1.0, **kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, gait_clock=True, obs_hist_len=3,
                domain_rand=False, imu_noise=0.0, quantize_ticks=False,
                latency_ms=0.0, backlash_deg=0.0,
                cmd_fixed=(0.0, 0.0, 0.0, crouch, 0.0, 0.0, 0.0))
    base.update(kw)
    return BimoMJXEnv(**base)


def test_defaults_off():
    env = _env()
    assert env.w_hip_yaw == 0.0 and env.w_crouch_track == 0.0
    assert env.crouch_release is False
    assert env._i_yaw is not None  # v5body has hip yaw


def _reward_after_step(env, qpos_edit=None):
    s = env.reset(jax.random.PRNGKey(0))
    if qpos_edit is not None:
        q = s.data.qpos
        for i, v in qpos_edit:
            q = q.at[env._jq0 + int(i)].set(v)
        s = s._replace(data=s.data.replace(qpos=q))
    s2 = jax.jit(env.step)(s, jp.zeros(env.action_size))
    return float(s2.reward)


def test_hip_yaw_penalty_charges_yaw():
    env0 = _env(); env1 = _env(w_hip_yaw=3.0)
    yaw = [(int(i), 0.15) for i in np.asarray(env1._i_yaw)]
    # with no yaw offset the two envs agree; with 0.15 rad on both yaws
    # the penalised env is lower by ~3 * 2 * 0.15^2 (before dynamics moves it)
    r0 = _reward_after_step(env0, yaw); r1 = _reward_after_step(env1, yaw)
    assert r1 < r0
    # the reward reads qpos AFTER the step's dynamics pulled the yaws back
    # toward zero, so the charge is below the injected 3*2*0.15^2 = 0.135
    # but well above zero
    assert 0.04 < (r0 - r1) <= 0.135 + 1e-3, (r0, r1)


def test_crouch_track_only_under_crouch_command():
    # standing tall (crouch 1.0): the kernel is gated off, reward unchanged
    assert abs(_reward_after_step(_env(1.0, w_crouch_track=2.0))
               - _reward_after_step(_env(1.0))) < 1e-5
    # crouch 0.8 commanded while standing at nominal height: err ~ 5.7 cm
    # >> sigma 1.5 cm, so the kernel pays ~0 -- no free income for ignoring
    r_off = _reward_after_step(_env(0.8)); r_on = _reward_after_step(_env(0.8, w_crouch_track=2.0))
    assert abs(r_on - r_off) < 0.02, (r_on, r_off)
    # sigma widened to cover the error: the kernel pays close to its weight
    r_wide = _reward_after_step(_env(0.8, w_crouch_track=2.0, crouch_track_sigma=0.5))
    assert r_wide - r_off > 1.5, (r_wide, r_off)


def test_crouch_release_opens_still_gate():
    """With a crouch commanded, w_still charges joint velocity unless
    crouch_release is set. Inject joint velocity and compare."""
    def run(release):
        env = _env(0.8, w_still=5.0, crouch_release=release)
        s = env.reset(jax.random.PRNGKey(0))
        qv = s.data.qvel.at[env._jv0:env._jv1].set(2.0)
        s = s._replace(data=s.data.replace(qvel=qv))
        return float(jax.jit(env.step)(s, jp.zeros(env.action_size)).reward)
    r_gated, r_released = run(False), run(True)
    assert r_released > r_gated + 1.0, (r_gated, r_released)
