"""--w-gyro-hinge / --gyro-hinge (2026-11-05): price the flail guard's own quantity.

The firmware flail guard trips on EMA(|gyro|, 0.2 s) > 1.5 rad/s. On the measured-sole
plant with the refit actuator, every stride1 of the deploy students tripped it at
1.6 - 1.7 rad/s (yaw-led) without a fall, while --w-pitch-rate prices roll and pitch only.
The hinge is -w * max(|qvel[3:6]| - h, 0)^2; off by default.
"""
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
        "--precision", "--mjx-impl", "jax", "--family", "loco"]


def test_cli_flags_parse_into_env_kw():
    import train_mjx
    p = train_mjx.make_parser()
    kw = train_mjx.build_env_kw(p.parse_args(ARGS))
    assert "w_gyro_hinge" not in kw and "gyro_hinge" not in kw      # off by default
    kw = train_mjx.build_env_kw(p.parse_args(ARGS + ["--w-gyro-hinge", "2.5", "--gyro-hinge", "1.1"]))
    assert kw["w_gyro_hinge"] == 2.5 and kw["gyro_hinge"] == 1.1


def test_default_env_has_hinge_off():
    from env_mjx import BimoMJXEnv
    import inspect
    sig = inspect.signature(BimoMJXEnv.__init__)
    assert sig.parameters["w_gyro_hinge"].default == 0.0
    assert sig.parameters["gyro_hinge"].default == 1.2


def _rollout(env, steps=4):
    import jax
    import jax.numpy as jp
    st = env.reset(jax.random.PRNGKey(3))
    act = 0.4 * jp.sin(jp.arange(env.action_size, dtype=jp.float32))
    for _ in range(steps):
        st = env.step(st, act)
    return st


def test_hinge_prices_exactly_the_excess():
    import train_mjx
    from env_mjx import BimoMJXEnv
    kw = train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS))
    s_off = _rollout(BimoMJXEnv(**kw))
    w = 3.0
    s_on = _rollout(BimoMJXEnv(**dict(kw, w_gyro_hinge=w, gyro_hinge=0.0)))
    om = float(np.linalg.norm(np.asarray(s_off.pipeline_state.qvel[3:6])
                              if hasattr(s_off, "pipeline_state") else np.asarray(s_off.data.qvel[3:6])))
    assert om > 0.0                                                  # the torso does rotate
    # the penalty does not change the dynamics, so the twins see the same omega
    assert float(s_off.reward) - float(s_on.reward) == pytest.approx(w * om * om, rel=1e-4, abs=1e-6)
    # a band above the rate costs nothing
    s_free = _rollout(BimoMJXEnv(**dict(kw, w_gyro_hinge=w, gyro_hinge=om + 1.0)))
    assert float(s_free.reward) == pytest.approx(float(s_off.reward), abs=1e-6)
