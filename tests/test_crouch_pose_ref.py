"""The explicit squat (2026-09-11, Tom: "an explicit example rather than
something that's RL trained"): under a crouch command the mimic reference is
the bench's level-foot squat family (hip -theta, knee -2 theta, ankle -theta)
and every height target uses the feasible depth. Off by default = bit-exact
old reference; MJX and the walker referee must agree; the pose must really
be a level-foot squat in the physics."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import jax.numpy as jp  # noqa: E402
import mujoco  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def _mjx(**kw):
    from env_mjx import BimoMJXEnv
    base = dict(xml_path=XML, ext_cmd=True, gait_clock=True, obs_hist_len=3,
                domain_rand=False, imu_noise=0.0, quantize_ticks=False,
                latency_ms=0.0, backlash_deg=0.0,
                cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
    base.update(kw)
    return BimoMJXEnv(**base)


def _walker(**kw):
    from walker_env import BimoWalkerEnv
    base = dict(xml_path=XML, command_mode=True, ext_cmd=True, gait_clock=True,
                actuator_model="sts3215", imu_obs=True, domain_rand=False,
                latency_ms=0.0, backlash_deg=0.0, quantize_ticks=False)
    base.update(kw)
    return BimoWalkerEnv(**base)


def test_off_by_default_reference_unchanged():
    e0, e1 = _mjx(), _mjx(crouch_pose_ref=False)
    for c in (1.0, 0.8, 0.7):
        cmd = jp.array([0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0])
        a = np.asarray(e0._mimic_ref(cmd, 0.3, 1.5)); b = np.asarray(e1._mimic_ref(cmd, 0.3, 1.5))
        np.testing.assert_allclose(a, b, atol=0)
    assert e0.crouch_pose_ref is False
    assert abs(float(e0._crouch_h_target(0.8)) - 0.8 * e0._nominal_h) < 1e-6


def test_theta_mapping_and_feasible_depth():
    e = _mjx(crouch_pose_ref=True)
    assert abs(float(e._crouch_theta(1.0))) < 1e-9
    assert abs(float(e._crouch_theta(0.7)) - np.radians(39.0)) < 1e-6
    assert abs(float(e._crouch_theta(0.85)) - np.radians(19.5)) < 1e-6
    assert float(e._crouch_theta(0.5)) <= np.radians(39.0) + 1e-6   # clipped (float32)
    drop = e._nominal_h - float(e._crouch_h_target(0.7))
    assert 0.03 < drop < 0.06, drop           # ~4.0 cm for 18 cm of leg at 39 deg
    assert abs(drop - e._leg_len * (1 - np.cos(np.radians(39)))) < 1e-6
    # the knee at theta_max stays inside its limit (-95 deg on v5body)
    cmd = jp.array([0.0, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0])
    q = np.asarray(e._mimic_ref(cmd, 0.0, 1.5))
    lo = np.asarray(e._lo)
    assert (q[np.asarray(e._i_knee)] >= lo[np.asarray(e._i_knee)] - 1e-6).all()
    # and the ankle is NOT clipped (a clipped ankle tilts the sole -- 45 deg did)
    assert (q[np.asarray(e._i_ankle)] > lo[np.asarray(e._i_ankle)] + 1e-4).all()


def test_squat_is_stationary_only():
    """A walking command with a stance draw gets the plain gait reference;
    the squat offsets appear only when nothing is commanded."""
    e = _mjx(crouch_pose_ref=True)
    still = np.asarray(e._mimic_ref(jp.array([0, 0, 0, 0.7, 0, 0, 0.0]), 0.0, 1.5))
    tall = np.asarray(e._mimic_ref(jp.array([0, 0, 0, 1.0, 0, 0, 0.0]), 0.0, 1.5))
    assert np.abs(still - tall)[np.asarray(e._i_knee)].max() > 1.0     # squat present
    walk_c = np.asarray(e._mimic_ref(jp.array([0.3, 0, 0, 0.7, 0, 0, 0.0]), 0.7, 1.5))
    walk_t = np.asarray(e._mimic_ref(jp.array([0.3, 0, 0, 1.0, 0, 0, 0.0]), 0.7, 1.5))
    np.testing.assert_allclose(walk_c, walk_t, atol=1e-6)               # no squat-walk
    assert abs(float(e._crouch_h_target(0.7, True)) - 0.7 * e._nominal_h) < 1e-6
    assert float(e._crouch_h_target(0.7, False)) > 0.7 * e._nominal_h + 0.03


def test_walker_mirrors_mjx():
    em, ew = _mjx(crouch_pose_ref=True), _walker(crouch_pose_ref=True)
    for c in (1.0, 0.85, 0.7):
        for ph, vx in ((0.0, 0.0), (1.1, 0.3)):
            cmd = np.array([vx, 0.0, 0.0, c, 0.0, 0.0, 0.0])
            a = np.asarray(em._mimic_ref(jp.asarray(cmd), ph, 1.5))
            b = np.asarray(ew._mimic_ref(cmd, ph, 1.5))
            np.testing.assert_allclose(a, b, atol=1e-5, err_msg=f"c={c} ph={ph}")
        for mv in (False, True):
            assert abs(float(em._crouch_h_target(c, mv)) - ew._crouch_h_target(c, mv)) < 1e-5


def test_reference_is_a_level_foot_squat_in_the_physics():
    """Apply the crouch-0.7 reference pose to the model: soles and torso stay
    level, and the hip drops by the feasible depth the height target uses."""
    e = _mjx(crouch_pose_ref=True)
    m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    hp0 = d.xanchor[m.joint("L_hip_pitch").id][2] - d.xanchor[m.joint("L_ankle").id][2]
    q_ref = np.asarray(e._mimic_ref(jp.array([0, 0, 0, 0.7, 0, 0, 0.0]), 0.0, 1.5))
    d.qpos[e._jq0:e._jq1] = q_ref
    mujoco.mj_forward(m, d)
    for g in ("L_sole", "R_sole"):
        R = d.geom_xmat[m.geom(g).id].reshape(3, 3)
        assert abs(R[2, 2] - 1.0) < 1e-3, (g, R[2, 2])
    Rt = d.xmat[m.body("torso").id].reshape(3, 3)
    assert abs(Rt[2, 2] - 1.0) < 1e-3
    hp1 = d.xanchor[m.joint("L_hip_pitch").id][2] - d.xanchor[m.joint("L_ankle").id][2]
    want = e._nominal_h - float(e._crouch_h_target(0.7))
    assert abs((hp0 - hp1) - want) < 2e-3, (hp0 - hp1, want)


def test_crouch_pull_has_slope_and_is_gated():
    """Standing tall under a stationary crouch 0.7 is charged by the pull
    (unlike the flat mimic kernel); the same pose under a walking command
    with the same stance draw is not."""
    import jax
    def r(vx, w):
        e = _mjx(crouch_pose_ref=True, w_crouch_pull=w,
                 cmd_fixed=(vx, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0))
        s0 = e.reset(jax.random.PRNGKey(0))
        return float(jax.jit(e.step)(s0, jp.zeros(e.action_size)).reward)
    assert r(0.0, 0.0) - r(0.0, 0.5) > 1.0        # ~5 rad of L1 distance x 0.5
    assert abs(r(0.3, 0.0) - r(0.3, 0.5)) < 1e-4  # walking: term off


DEEP = (-65.0, -95.0, -40.0)     # Tom 2026-09-13, held open-loop on the robot


def test_deep_ref_pose_and_depth():
    """With crouch_deep_ref the 0.7 reference IS the target triple, scaled
    linearly by depth, and the height target is the FK drop of that pose."""
    e = _mjx(crouch_pose_ref=True, crouch_deep_ref=DEEP)
    for c, dep in ((1.0, 0.0), (0.85, 0.5), (0.7, 1.0), (0.5, 1.0)):
        h, k, a = (float(x) for x in e._crouch_pose(c))
        np.testing.assert_allclose([h, k, a], np.radians(DEEP) * dep, atol=1e-5)
    q = np.asarray(e._mimic_ref(jp.array([0, 0, 0, 0.7, 0, 0, 0.0]), 0.0, 1.5))
    d = np.asarray(e._default)
    np.testing.assert_allclose(np.degrees(q[np.asarray(e._i_pitch)] - d[np.asarray(e._i_pitch)]), [-65, -65], atol=1e-3)
    np.testing.assert_allclose(np.degrees(q[np.asarray(e._i_knee)] - d[np.asarray(e._i_knee)]), [-95, -95], atol=1e-3)
    np.testing.assert_allclose(np.degrees(q[np.asarray(e._i_ankle)] - d[np.asarray(e._i_ankle)]), [-40, -40], atol=1e-3)
    drop = e._nominal_h - float(e._crouch_h_target(0.7))
    want = e._shank * (1 - np.cos(np.radians(40))) + e._thigh * (1 - np.cos(np.radians(55)))
    assert abs(drop - want) < 1e-6 and 0.05 < drop < 0.07, (drop, want)   # ~5.9 cm
    # walking keeps the plain reference and the old height law
    walk_c = np.asarray(e._mimic_ref(jp.array([0.3, 0, 0, 0.7, 0, 0, 0.0]), 0.7, 1.5))
    walk_t = np.asarray(e._mimic_ref(jp.array([0.3, 0, 0, 1.0, 0, 0, 0.0]), 0.7, 1.5))
    np.testing.assert_allclose(walk_c, walk_t, atol=1e-6)
    assert abs(float(e._crouch_h_target(0.7, True)) - 0.7 * e._nominal_h) < 1e-6


def test_deep_ref_off_is_the_level_foot_family():
    """crouch_deep_ref=None reproduces the level-foot reference and depth
    exactly (the refactor to _crouch_pose/_crouch_drop changed nothing)."""
    e = _mjx(crouch_pose_ref=True)
    th = np.radians(39.0)
    np.testing.assert_allclose([float(x) for x in e._crouch_pose(0.7)], [-th, -2 * th, -th], atol=1e-6)
    drop = e._nominal_h - float(e._crouch_h_target(0.7))
    assert abs(drop - e._leg_len * (1 - np.cos(th))) < 1e-6


def test_deep_ref_walker_parity():
    em, ew = _mjx(crouch_pose_ref=True, crouch_deep_ref=DEEP), _walker(crouch_pose_ref=True, crouch_deep_ref=DEEP)
    for c in (1.0, 0.85, 0.7):
        for ph, vx in ((0.0, 0.0), (1.1, 0.3)):
            cmd = np.array([vx, 0.0, 0.0, c, 0.0, 0.0, 0.0])
            a = np.asarray(em._mimic_ref(jp.asarray(cmd), ph, 1.5))
            b = np.asarray(ew._mimic_ref(cmd, ph, 1.5))
            np.testing.assert_allclose(a, b, atol=1e-5, err_msg=f"c={c} ph={ph}")
        for mv in (False, True):
            assert abs(float(em._crouch_h_target(c, mv)) - ew._crouch_h_target(c, mv)) < 1e-5


def test_deep_ref_in_the_physics():
    """The deep 0.7 pose on the model: soles flat, torso pitched ~10 deg
    forward (the lean that keeps the hips over the feet), hip drop = the FK
    number the height target uses, and every joint inside its range."""
    e = _mjx(crouch_pose_ref=True, crouch_deep_ref=DEEP)
    m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    hp0 = d.xanchor[m.joint("L_hip_pitch").id][2] - d.xanchor[m.joint("L_ankle").id][2]
    q_ref = np.asarray(e._mimic_ref(jp.array([0, 0, 0, 0.7, 0, 0, 0.0]), 0.0, 1.5))
    lo, hi = np.asarray(e._lo), np.asarray(e._hi)
    assert (q_ref >= lo - 1e-6).all() and (q_ref <= hi + 1e-6).all(), np.degrees(q_ref)
    d.qpos[e._jq0:e._jq1] = q_ref
    # the root is the torso: pitch it until the soles are flat (on the robot
    # the foot is on the table and the TORSO leans) and read the lean off
    best = None
    for phi in np.radians(np.arange(-20.0, 20.01, 0.25)):
        d.qpos[3:7] = [np.cos(phi / 2), 0.0, np.sin(phi / 2), 0.0]
        mujoco.mj_forward(m, d)
        flat = min(d.geom_xmat[m.geom(g).id].reshape(3, 3)[2, 2] for g in ("L_sole", "R_sole"))
        if best is None or flat > best[0]:
            best = (flat, phi)
    flat, phi = best
    assert flat > 0.9999, flat
    assert 8.0 < abs(np.degrees(phi)) < 12.0, np.degrees(phi)     # ~10 deg lean, as measured on the robot
    d.qpos[3:7] = [np.cos(phi / 2), 0.0, np.sin(phi / 2), 0.0]
    mujoco.mj_forward(m, d)
    hp1 = d.xanchor[m.joint("L_hip_pitch").id][2] - d.xanchor[m.joint("L_ankle").id][2]
    want = e._nominal_h - float(e._crouch_h_target(0.7))
    assert abs((hp0 - hp1) - want) < 2e-3, (hp0 - hp1, want)


def test_deep_ref_with_rsi_resets_and_steps():
    """Reference-state init draws the deep pose: reset + one step run."""
    import jax
    e = _mjx(crouch_pose_ref=True, crouch_deep_ref=DEEP, crouch_rsi_mix=1.0, cmd_fixed=None, cmd_crouch_range=(0.7, 1.0))
    st = e.reset(jax.random.PRNGKey(3))
    q = np.asarray(st.pipeline_state.qpos[e._jq0:e._jq1]) if hasattr(st, "pipeline_state") else np.asarray(st.data.qpos[e._jq0:e._jq1])
    d = np.asarray(e._default)
    assert np.degrees(q[np.asarray(e._i_knee)] - d[np.asarray(e._i_knee)]).max() < -60.0
    st2 = e.step(st, jp.zeros(e.action_size))
    assert np.isfinite(float(st2.reward))
