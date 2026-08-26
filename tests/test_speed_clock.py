"""The speed-coupled gait clock is a timing contract shared by env_mjx (JAX),
walker_env (the referee), and eventually the firmware clock. A mismatch means
the policy is judged against a clock it never trained on — so pin the law and
prove both envs advance phase identically for the same command."""
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

from env_mjx import (speed_clock_scale, SPEED_CLOCK_REF,  # noqa: E402
                     SPEED_CLOCK_LO, SPEED_CLOCK_HI)

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def test_law_shape():
    assert abs(float(speed_clock_scale(SPEED_CLOCK_REF)) - 1.0) < 1e-6
    assert abs(float(speed_clock_scale(0.0)) - SPEED_CLOCK_LO) < 1e-6
    assert abs(float(speed_clock_scale(5.0)) - SPEED_CLOCK_HI) < 1e-6
    grid = np.linspace(0.0, 1.5, 301)
    vals = np.array([float(speed_clock_scale(v)) for v in grid])
    assert (np.diff(vals) >= -1e-6).all()  # monotone up to float32 jitter
    assert (vals >= SPEED_CLOCK_LO - 1e-6).all()
    assert (vals <= SPEED_CLOCK_HI + 1e-6).all()


def test_mjx_phase_advances_with_command():
    """Same reset key (same freq draw): a fast command must advance the
    clock faster than a slow one, by exactly the law's ratio."""
    from env_mjx import BimoMJXEnv

    def phase_rate(vx):
        env = BimoMJXEnv(xml_path=XML, ext_cmd=True, gait_clock=True,
                         speed_clock=True, obs_hist_len=3,
                         domain_rand=False, imu_noise=0.0,
                         quantize_ticks=False, latency_ms=0.0,
                         backlash_deg=0.0,
                         cmd_fixed=(vx, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02))
        s = env.reset(jax.random.PRNGKey(7))
        f0, p0 = float(s.gait_freq), float(s.gait_phase)
        s2 = env.step(s, jp.zeros(env.action_size))
        dp = (float(s2.gait_phase) - p0 + np.pi) % (2 * np.pi) - np.pi
        return dp, f0, env

    dp_slow, f_slow, env = phase_rate(0.1)
    dp_fast, f_fast, _ = phase_rate(0.9)
    assert abs(f_slow - f_fast) < 1e-6      # same key -> same base freq draw
    want = (float(speed_clock_scale(0.9)) / float(speed_clock_scale(0.1)))
    assert abs(dp_fast / dp_slow - want) < 1e-3, (dp_slow, dp_fast, want)


def test_referee_matches_mjx_law():
    """walker_env's inline mirror must advance phase by the same scaled rate
    for the same command and base frequency."""
    from walker_env import BimoWalkerEnv
    env = BimoWalkerEnv(xml_path=XML, command_mode=True, ext_cmd=True,
                        gait_clock=True, speed_clock=True,
                        actuator_model="sts3215", imu_obs=True,
                        domain_rand=False, latency_ms=0.0, backlash_deg=0.0)
    for vx in (0.1, 0.35, 0.9):
        env.reset(seed=0)
        env._gait_freq = 1.5
        env.set_command(vx, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
        p0 = env._gait_phase
        env.step(np.zeros(env.action_space.shape[0]))
        dp = (env._gait_phase - p0 + np.pi) % (2 * np.pi) - np.pi
        want = (2 * np.pi * env.control_dt * 1.5
                * float(speed_clock_scale(vx)))
        assert abs(dp - want) < 1e-6, (vx, dp, want)


def test_off_by_default():
    """speed_clock defaults False: the old fixed-rate advance is untouched
    (old configs referee identically)."""
    from walker_env import BimoWalkerEnv
    env = BimoWalkerEnv(xml_path=XML, command_mode=True, ext_cmd=True,
                        gait_clock=True,
                        actuator_model="sts3215", imu_obs=True,
                        domain_rand=False, latency_ms=0.0, backlash_deg=0.0)
    assert env.speed_clock is False
    env.reset(seed=0)
    env._gait_freq = 1.5
    env.set_command(0.9, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    p0 = env._gait_phase
    env.step(np.zeros(env.action_space.shape[0]))
    dp = (env._gait_phase - p0 + np.pi) % (2 * np.pi) - np.pi
    assert abs(dp - 2 * np.pi * env.control_dt * 1.5) < 1e-6


def test_hi_knob_threads_both_envs():
    """speed_clock_hi must reach both the law and the referee mirror."""
    assert abs(float(speed_clock_scale(5.0, hi=1.25)) - 1.25) < 1e-6
    from walker_env import BimoWalkerEnv
    env = BimoWalkerEnv(xml_path=XML, command_mode=True, ext_cmd=True,
                        gait_clock=True, speed_clock=True,
                        speed_clock_hi=1.25,
                        actuator_model="sts3215", imu_obs=True,
                        domain_rand=False, latency_ms=0.0, backlash_deg=0.0)
    env.reset(seed=0)
    env._gait_freq = 1.5
    env.set_command(0.9, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    p0 = env._gait_phase
    env.step(np.zeros(env.action_space.shape[0]))
    dp = (env._gait_phase - p0 + np.pi) % (2 * np.pi) - np.pi
    want = (2 * np.pi * env.control_dt * 1.5
            * float(speed_clock_scale(0.9, hi=1.25)))
    assert abs(dp - want) < 1e-6, (dp, want)
