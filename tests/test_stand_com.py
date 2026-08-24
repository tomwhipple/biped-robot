"""w_stand_com pays for parking the standing CoM over the midfoot point.
The stand_off scenario is passive physics: with servo torque released, the
stand survives only while the ankle gravity moment stays under backdrive
friction, i.e. CoM within ~16 mm of the support center. The kernel must
(a) thread through the env, (b) pay MORE the closer the CoM is to the
midfoot, (c) pay nothing while a locomotion command is active."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

import jax  # noqa: E402
import jax.numpy as jp  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def _env(**kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, gait_clock=True,
                obs_hist_len=3, domain_rand=False, imu_noise=0.0,
                quantize_ticks=False, latency_ms=0.0, backlash_deg=0.0)
    base.update(kw)
    return BimoMJXEnv(**base)


def test_kernel_threads_and_reward_finite():
    env = _env(w_stand_com=1.0,
               cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02))
    assert env.w_stand_com == 1.0 and env.stand_com_sigma == 0.02
    s = env.reset(jax.random.PRNGKey(0))
    s = env.step(s, jp.zeros(env.action_size))
    assert np.isfinite(float(s.reward))


def test_pays_more_when_centered():
    """Reward delta between w=0 and w=5 from the same standing state must be
    positive (the reset pose is near-centered, kernel > 0) and bounded by w
    (kernel <= 1)."""
    cmd = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    e0 = _env(w_stand_com=0.0, cmd_fixed=cmd)
    e1 = _env(w_stand_com=5.0, cmd_fixed=cmd)
    a = jp.zeros(e0.action_size)
    r0 = float(e0.step(e0.reset(jax.random.PRNGKey(1)), a).reward)
    r1 = float(e1.step(e1.reset(jax.random.PRNGKey(1)), a).reward)
    delta = r1 - r0
    assert 0.0 < delta <= 5.0 + 1e-5, delta


def test_gated_off_while_moving():
    """A forward command must see zero contribution from the kernel."""
    cmd = (0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    e0 = _env(w_stand_com=0.0, cmd_fixed=cmd)
    e1 = _env(w_stand_com=5.0, cmd_fixed=cmd)
    a = jp.zeros(e0.action_size)
    r0 = float(e0.step(e0.reset(jax.random.PRNGKey(2)), a).reward)
    r1 = float(e1.step(e1.reset(jax.random.PRNGKey(2)), a).reward)
    assert abs(r1 - r0) < 1e-5


def test_stand_knee_threads_and_gates():
    """w_stand_knee: threads, pays while standing, silent while moving."""
    stand = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    move = (0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.02)
    e0 = _env(w_stand_knee=0.0, cmd_fixed=stand)
    e1 = _env(w_stand_knee=5.0, cmd_fixed=stand)
    assert e1.w_stand_knee == 5.0 and e1.stand_knee_target == 0.10
    a = jp.zeros(e0.action_size)
    r0 = float(e0.step(e0.reset(jax.random.PRNGKey(3)), a).reward)
    r1 = float(e1.step(e1.reset(jax.random.PRNGKey(3)), a).reward)
    assert 0.0 < (r1 - r0) <= 5.0 + 1e-5, r1 - r0
    m0 = _env(w_stand_knee=0.0, cmd_fixed=move)
    m1 = _env(w_stand_knee=5.0, cmd_fixed=move)
    rm0 = float(m0.step(m0.reset(jax.random.PRNGKey(4)), a).reward)
    rm1 = float(m1.step(m1.reset(jax.random.PRNGKey(4)), a).reward)
    assert abs(rm1 - rm0) < 1e-5
