"""The mirror map is only useful if it is EXACTLY right -- one wrong sign
and symmetry regularization silently trains the policy toward a garbage
correspondence. These tests pin it against the physics, not against the
author's reasoning:

  1. build-time consistency (ranges/ctrlranges) is exercised implicitly by
     constructing the maps at all;
  2. the dynamics test rolls the full contact model forward under a random
     ctrl sequence from a randomized state, and again from the mirrored
     state under the mirrored ctrls -- the trajectories must be mirror
     images to float tolerance. Any wrong permutation entry or sign in the
     JOINT map fails this within a step or two;
  3. the obs test runs the actual training env (nominal, DR off) on
     mirrored command scripts and asserts the assembled obs vectors map to
     each other through obs_perm_signs -- this pins the FRAME layout
     (up/linvel/gyro/phase/cmd blocks), which test 2 cannot see.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

import mujoco  # noqa: E402

import mirror  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(XML)


def test_joint_map_builds_and_is_involutive(model):
    perm, sign = mirror.joint_perm_signs(model)
    # applying the mirror twice is the identity
    v = np.random.default_rng(0).normal(size=len(perm))
    assert np.allclose(mirror.mirror(mirror.mirror(v, perm, sign), perm, sign), v)
    # L and R blocks actually swap
    assert not np.array_equal(perm, np.arange(len(perm)))


def test_action_map_matches_ctrlranges(model):
    # raises inside if the plant's ctrlranges break the signed-permutation
    # property; also involutive
    perm, sign = mirror.action_perm_signs(model)
    v = np.random.default_rng(1).normal(size=len(perm))
    assert np.allclose(mirror.mirror(mirror.mirror(v, perm, sign), perm, sign), v)


def test_mirrored_dynamics(model):
    """Rollout(mirror(state), mirror(ctrls)) == mirror(Rollout(state, ctrls)).

    Uses raw mujoco with the position actuators, through ground contact and
    the inter-leg/foot contact pairs. The plant is sagittally symmetric up
    to build tolerances (the torso inertial carries ~1e-4 asymmetries), so
    the comparison uses a tolerance and a short horizon.
    """
    perm, sign = mirror.joint_perm_signs(model)
    d1 = mujoco.MjData(model)
    d2 = mujoco.MjData(model)
    rng = np.random.default_rng(7)

    mujoco.mj_resetData(model, d1)
    # a mildly randomized but stable-ish start: nominal pose + joint noise
    d1.qpos[7:] += rng.uniform(-0.15, 0.15, size=model.nq - 7)
    d1.qvel[:] = 0.0
    # 25 ctrls x 4 steps: long enough that any wrong permutation entry or
    # sign explodes (they diverge by ~radians within a ctrl or two), short
    # enough that the plant's real ~1e-4 build asymmetries (torso inertial)
    # stay under tolerance. Measured: correct map drifts to 6.9e-3 rad by
    # ctrl 31; a flipped hip-roll sign is at >0.5 rad by ctrl 2.
    ctrls = []
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    for _ in range(25):
        ctrls.append(rng.uniform(lo, hi))

    mujoco.mj_resetData(model, d2)
    d2.qpos[:7] = mirror.base_qpos_mirror(d1.qpos[:7])
    d2.qpos[7:] = mirror.mirror(np.array(d1.qpos[7:]), perm, sign)
    d2.qvel[:6] = mirror.base_qvel_mirror(d1.qvel[:6])
    d2.qvel[6:] = mirror.mirror(np.array(d1.qvel[6:]), perm, sign)

    # ctrl mirror in ANGLE space: the ctrlrange consistency asserted by
    # action_perm_signs makes this the same signed permutation
    for k, c in enumerate(ctrls):
        d1.ctrl[:] = c
        d2.ctrl[:] = mirror.mirror(c, perm, sign)
        for _ in range(4):
            mujoco.mj_step(model, d1)
            mujoco.mj_step(model, d2)
        got = np.array(d2.qpos[7:])
        want = mirror.mirror(np.array(d1.qpos[7:]), perm, sign)
        err = np.max(np.abs(got - want))
        assert err < 5e-3, f"joint mirror diverged at ctrl {k}: {err:.4f} rad"
        base_err = abs(d2.qpos[1] + d1.qpos[1])
        assert base_err < 5e-3, f"base-y mirror diverged: {base_err:.4f} m"


def test_obs_frame_mirror():
    """The assembled obs of mirrored episodes map through obs_perm_signs.

    Nominal env (DR, noise, quantization off), fixed mirrored commands,
    zero actions -- every obs source (posture, gyro, clock, cmd) is then
    deterministic and sagittally mirrored, so the vectors must correspond.
    """
    from env_mjx import BimoMJXEnv
    import jax
    import jax.numpy as jp

    kw = dict(xml_path=XML, ext_cmd=True, gait_clock=True, obs_hist_len=3,
              domain_rand=False, imu_noise=0.0, quantize_ticks=False,
              latency_ms=0.0, backlash_deg=0.0)
    cmd_a = (0.3, 0.10, 0.4, 1.0, -1.0, 0.0, 0.02)
    cmd_b = (0.3, -0.10, -0.4, 1.0, 1.0, 0.0, 0.02)
    env_a = BimoMJXEnv(cmd_fixed=cmd_a, **kw)
    env_b = BimoMJXEnv(cmd_fixed=cmd_b, **kw)
    perm, sign = mirror.obs_perm_signs(env_a.mj_model, ncmd=7,
                                       hist_len=env_a.obs_hist_len)
    perm_j, sign_j = mirror.joint_perm_signs(env_a.mj_model)

    sa = env_a.reset(jax.random.PRNGKey(3))
    sb = env_b.reset(jax.random.PRNGKey(3))
    # force b's physics state to be a's mirror (reset randomizes pose)
    qpos = np.array(sa.data.qpos)
    qvel = np.array(sa.data.qvel)
    mq = np.concatenate([mirror.base_qpos_mirror(qpos[:7]),
                         mirror.mirror(qpos[7:], perm_j, sign_j)])
    mv = np.concatenate([mirror.base_qvel_mirror(qvel[:6]),
                         mirror.mirror(qvel[6:], perm_j, sign_j)])
    sb = sb._replace(data=sb.data.replace(qpos=jp.asarray(mq),
                                         qvel=jp.asarray(mv)),
                    # half-cycle shift wrapped to [-pi, pi):
                    # wrap(x) = mod(x + pi, 2pi) - pi with x = phase + pi
                    gait_phase=jp.mod(sa.gait_phase + 2 * jp.pi,
                                      2 * jp.pi) - jp.pi,
                    gait_freq=sa.gait_freq)

    # zero normalized action is its own mirror: it maps to each side's
    # ctrlrange CENTER, and centers mirror (asserted by action_perm_signs).
    a_act = jp.zeros(env_a.action_size)
    hist = env_a.obs_hist_len
    for it in range(hist + 2):
        sa = env_a.step(sa, a_act)
        sb = env_b.step(sb, a_act)
        if it < hist - 1:
            continue    # b's history ring still holds pre-mirror reset frames
        got = np.array(sb.obs)
        want = mirror.mirror(np.array(sa.obs), perm, sign)
        err = np.max(np.abs(got - want))
        # tolerance: the two episodes run real physics from mirrored states,
        # and the plant's ~1e-4 inertial asymmetries make the trajectories
        # (hence dq/gyro obs) drift ~1e-2 apart within a few steps -- same
        # scale test_mirrored_dynamics measures. A LAYOUT error (wrong slot
        # or sign) mismatches by O(1), far above this gate.
        assert err < 2e-2, f"obs mirror mismatch {err:.5f} at step {it}"
