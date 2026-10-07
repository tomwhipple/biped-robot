"""The training stack on the ROBOT's plant (issue #86 groundwork).

Builds the CPU referee env (walker_env) and the MJX training env (env_mjx)
on sim/bimo_biped_v6ar.xml under the robot preset (sim/mjx/robot_plant.py):
the 12 leg joints as the policy, the neck -- and the arms, on a plant that
has them -- held at the walking pose, Plan B per-servo stiffness, hip
flexion -120 deg. Nothing is trained; these pin that the stack RUNS on the
robot and configures it the way DESIGN.md and docs/training.md say:

  * smoke: both envs, zero actions for 300 control steps (6 s): finite,
    no explosion, and the home pose is a stand under the servo model;
  * the same on a generated 17-servo plant (arms, generator inertials); the
    committed plant carries the arms too, with CAD inertials;
  * per-servo stiffness: the Plan B profile lands on exactly the six roll
    and knee servos, and the PD law applies it (x4 torque for the same
    error, kp AND kd);
  * joint roles: the per-leg tables are the same in both envs, an
    unknown policy joint is refused, the imitation reference keeps the sole
    level (pitch and roll) on the robot;
  * the mirror map over the policy joints (neck held) matches the physics;
  * the referee builds its world on the robot plant: no payload, the
    get-up scenarios n/a, a scenario runs end to end.

The MJX-vs-CPU arithmetic on the robot plant is gated by
sim/mjx/parity_test.py (blocks R1-R3; `--robot-only` runs just those).
"""
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

import mujoco  # noqa: E402

import robot_plant  # noqa: E402
import walker_env as W  # noqa: E402

ROBOT_XML = os.path.join(SIM, "bimo_biped_v6ar.xml")
PROTO_XML = os.path.join(SIM, "bimo_biped_v5body.xml")
ROLLS_KNEES = {f"{s}_{r}" for s in "LR"
               for r in ("hip_roll", "ankle_roll", "knee")}
# the committed plant is the robot as drawn: 17 servos, the neck and both
# arms held at their folded rest pose (dimensions_v6.ARM_REST)
HELD17 = {"neck_yaw": 0.0, "L_shoulder": -15.0, "L_elbow": -95.0,
          "R_shoulder": -15.0, "R_elbow": -95.0}

# a precision-style policy env (the --precision preset's observation and
# reward structure), on top of the robot preset
PREC = dict(supply_voltage=11.1, ext_cmd=True, gait_clock=True,
            obs_hist_len=3, imu_obs=True, action_map="full",
            quantize_ticks=True, w_mimic=1.5, w_pose=0.3, w_dof_limits=1.0,
            w_feet_phase=1.0, w_track_h=1.0, w_lift=1.0, w_track_foot=1.0,
            cmd_dense=True, joint_frictionloss=0.05, joint_armature=0.028,
            latency_ms=0.0, latency_ms_max=8.0, latency_jitter_ms=1.0,
            backlash_deg=0.5, backlash_deg_max=1.0, zero_offset_deg=2.0)


def _robot_kw(xml=None, **over):
    kw = dict(PREC)
    kw.update(robot_plant.robot_env_kwargs(xml))
    kw.update(over)
    return kw


def _cpu(**kw):
    return W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, **kw)


@pytest.fixture(scope="module")
def arms_xml(tmp_path_factory):
    return robot_plant.write_arms_test_plant(
        str(tmp_path_factory.mktemp("plant") / "arms17.xml"))


# --------------------------------------------------------------------- preset
def test_robot_preset_resolves_against_the_plant(arms_xml):
    kw = robot_plant.robot_env_kwargs()
    assert kw["held_joints"] == HELD17
    assert set(kw["servo_kp_scale"]) == ROLLS_KNEES
    # the robot's servo set: STS3250 hip rolls (4x), the ankle rolls and
    # knees at the bench's P 96 (2.8x)
    assert {k: v for k, v in kw["servo_kp_scale"].items()} == {
        f"{s}_{r}": f for s in "LR" for r, f in (("hip_roll", 4.0), ("ankle_roll", 2.8), ("knee", 2.8))}
    assert kw["hip_flex_deg"] == 120.0 and kw["payload_mass"] == 0.0
    kw17 = robot_plant.robot_env_kwargs(arms_xml)
    assert kw17["held_joints"] == HELD17
    m = mujoco.MjModel.from_xml_path
    assert robot_plant.is_robot_model(m(ROBOT_XML))
    assert robot_plant.is_robot_model(m(arms_xml))
    assert not robot_plant.is_robot_model(m(PROTO_XML))


# ---------------------------------------------------------------------- smoke
def test_smoke_cpu_robot_plant():
    env = _cpu(**_robot_kw(domain_rand=True))
    assert env._n_servo == 17 and env._nq_act == 12
    assert env.action_space.shape == (12,)
    assert env.observation_space.shape == (3 * (3 * 12 + 19),)   # 165
    assert "neck_yaw" not in env._act_names
    assert np.degrees(env._lo[env._jname2i["L_hip_pitch"]]) == \
        pytest.approx(-120.0)
    obs, _ = env.reset(seed=0)
    nominal = env._nominal_h
    for t in range(300):
        obs, r, term, trunc, info = env.step(np.zeros(12, np.float32))
        assert np.all(np.isfinite(obs)) and np.isfinite(r), t
        assert np.all(np.abs(env.data.qvel) < 50.0), t
        assert not term, f"zero action fell at step {t}"
    assert info["height"] > 0.85 * nominal          # the home pose stands
    neck = env.data.qpos[env._jq0 + env._sname2i["neck_yaw"]]
    assert abs(neck) < np.radians(3.0)              # held at 0


def test_smoke_mjx_robot_plant():
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    env = BimoMJXEnv(**_robot_kw(domain_rand=True))
    assert env.action_size == 12 and env.obs_size == 165
    st = jax.jit(env.reset)(jax.random.PRNGKey(0))

    def roll(s, _):
        s = env.step(s, jp.zeros(env.action_size))
        return s, (s.data.qpos[2], s.done, s.reward,
                   jp.max(jp.abs(s.data.qvel)), jp.all(jp.isfinite(s.obs)))
    st, (h, done, rew, vmax, fin) = jax.jit(
        lambda s: jax.lax.scan(roll, s, None, length=300))(st)
    assert bool(fin.all()) and bool(jp.isfinite(rew).all())
    assert float(vmax.max()) < 50.0
    assert float(done.max()) == 0.0
    assert float(h[-1]) > 0.85 * env._nominal_h


def test_smoke_17_servo_plant(arms_xml):
    """The committed plant gains the arms: the same stack, 17 servos."""
    import jax
    import jax.numpy as jp
    from env_mjx import BimoMJXEnv
    kw = _robot_kw(arms_xml, domain_rand=False, zero_offset_deg=0.0)
    cpu = _cpu(**kw)
    assert cpu._n_servo == 17 and cpu._nq_act == 12
    assert cpu.observation_space.shape == (165,)
    cpu.reset(seed=1)
    for _ in range(200):
        obs, r, term, _, info = cpu.step(np.zeros(12, np.float32))
        assert np.all(np.isfinite(obs)) and not term
    for side in "LR":
        sh = cpu.data.qpos[cpu._jq0 + cpu._sname2i[f"{side}_shoulder"]]
        el = cpu.data.qpos[cpu._jq0 + cpu._sname2i[f"{side}_elbow"]]
        assert np.degrees(sh) == pytest.approx(HELD17["L_shoulder"], abs=2.0)
        assert np.degrees(el) == pytest.approx(HELD17["L_elbow"], abs=2.0)
    g = BimoMJXEnv(**kw)
    assert g.action_size == 12 and g.obs_size == 165

    def roll(s, _):
        s = g.step(s, jp.zeros(12))
        return s, s.data.qpos[2]
    st = jax.jit(g.reset)(jax.random.PRNGKey(1))
    st, h = jax.jit(lambda s: jax.lax.scan(roll, s, None, length=100))(st)
    assert bool(jp.isfinite(h).all()) and float(h[-1]) > 0.85 * g._nominal_h
    sh = np.degrees(np.asarray(st.data.qpos)[g._jq0 + g._sname2i["L_shoulder"]])
    assert sh == pytest.approx(HELD17["L_shoulder"], abs=2.0)


# ----------------------------------------------------------- servo stiffness
def test_kp_scale_resolution():
    names = [mujoco.MjModel.from_xml_path(ROBOT_XML).joint(int(j)).name
             for j in mujoco.MjModel.from_xml_path(
                 ROBOT_XML).actuator_trnid[:, 0]]
    v = W.servo_kp_scale_vector(names, "planb")
    assert {n for n, f in zip(names, v) if f == 4.0} == ROLLS_KNEES
    assert np.count_nonzero(v != 1.0) == 6
    # an actuator name beats its role; unknown keys and wrong lengths raise
    v2 = W.servo_kp_scale_vector(names, {"knee": 3.0, "L_knee": 2.0})
    assert v2[names.index("L_knee")] == 2.0 and v2[names.index("R_knee")] == 3.0
    with pytest.raises(ValueError):
        W.servo_kp_scale_vector(names, {"hip_rol": 4.0})
    with pytest.raises(ValueError):
        W.servo_kp_scale_vector(names, [1.0] * 12)
    assert np.array_equal(W.servo_kp_scale_vector(names, None), np.ones(17))


def _one_substep_tau(kp_scale, joint, err=0.02):
    """Hoisted robot at rest, every servo on target but `joint` off by
    `err`: one physics substep of the PD law, returns the applied torques."""
    env = _cpu(xml_path=ROBOT_XML, supply_voltage=11.1, ext_cmd=True,
               servo_kp_scale=kp_scale, held_joints=HELD17,
               domain_rand=False, latency_ms=0.0, backlash_deg=0.0,
               quantize_ticks=False, push_prob=0.0)
    env.reset(seed=0)
    env.n_substeps = 1
    d = env.data
    d.qpos[:] = env.model.qpos0
    d.qpos[2] = 1.5
    d.qpos[env._jq0 + env._sname2i[joint]] -= err
    d.qvel[:] = 0.0
    mujoco.mj_forward(env.model, d)
    env.step(np.zeros(12, np.float32))
    return env._servo_tau.copy(), env._sname2i[joint]


@pytest.mark.parametrize("joint,factor", [("L_knee", 4.0), ("R_ankle_roll", 4.0),
                                          ("L_hip_roll", 4.0),
                                          ("L_hip_pitch", 1.0)])
def test_plan_b_scales_the_pd_torque(joint, factor):
    t0, i = _one_substep_tau(None, joint)
    t4, _ = _one_substep_tau("planb", joint)
    assert t0[i] == pytest.approx(12.0 * 0.02)      # kp 12 N*m/rad, stock
    assert t4[i] == pytest.approx(factor * t0[i])
    others = np.delete(np.arange(len(t0)), i)
    assert np.allclose(t4[others], 0.0) and np.allclose(t0[others], 0.0)


def test_per_actuator_gain_sequence_matches_scale():
    """servo_kp as a per-actuator sequence == the scalar with a scale."""
    names = W.BimoWalkerEnv(xml_path=ROBOT_XML)._servo_names
    scale = W.servo_kp_scale_vector(names, "planb")
    a = _cpu(xml_path=ROBOT_XML, servo_kp=list(12.0 * scale),
             servo_kd=list(0.25 * scale))
    b = _cpu(xml_path=ROBOT_XML, servo_kp_scale="planb")
    assert np.allclose(a._servo[0] * a._kp_prof, b._servo[0] * b._kp_prof)
    assert np.allclose(a._servo[1] * a._kd_prof, b._servo[1] * b._kd_prof)


# ------------------------------------------------------------------ roles
def test_role_tables_are_one_table():
    import env_mjx as M
    assert W.LEG_ROLES == M.LEG_ROLES and W.ARM_ROLES == M.ARM_ROLES
    assert W.BODY_JOINTS == M.BODY_JOINTS
    assert W.SERVO_KP_PRESETS == M.SERVO_KP_PRESETS
    names = W.BimoWalkerEnv(xml_path=ROBOT_XML)._servo_names
    for spec in (None, 2.0, "planb", {"knee": 3.0, "R_ankle_roll": 5.0}):
        assert np.array_equal(W.servo_kp_scale_vector(names, spec),
                              M.servo_kp_scale_vector(names, spec))
    for n in names:
        assert W.joint_role(n) == M.joint_role(n) is not None


def test_ankle_roll_has_its_role_and_prototypes_are_untouched():
    env = _cpu(**_robot_kw())
    assert env._i_aroll is not None
    assert [env._act_names[i] for i in env._i_aroll] == ["L_ankle_roll",
                                                         "R_ankle_roll"]
    proto = _cpu(xml_path=PROTO_XML)
    assert proto._i_aroll is None and proto._all_pol
    assert isinstance(proto._jqpos, slice)
    assert np.array_equal(proto._kp_prof, np.ones(10))


def test_unknown_policy_joint_is_refused(tmp_path):
    """A joint no reward term knows must be held (or given a role), never
    trained silently unregularized."""
    from gen_plant_v6 import DesignParams, build_xml
    xml = str(tmp_path / "tail.xml")
    with open(xml, "w") as f:
        f.write(build_xml(DesignParams(tail=True)))
    with pytest.raises(ValueError, match="tail_pitch"):
        _cpu(xml_path=xml, ext_cmd=True)
    env = _cpu(xml_path=xml, ext_cmd=True,
               held_joints={"tail_pitch": 0.0, "neck_yaw": 0.0})
    assert env._nq_act == 12
    with pytest.raises(ValueError):
        _cpu(xml_path=ROBOT_XML, held_joints={"neck": 0.0})


def _sole_normal(env, q_pol, side):
    d = mujoco.MjData(env.model)
    d.qpos[:] = env.model.qpos0
    d.qpos[env._jqpos] = q_pol
    mujoco.mj_forward(env.model, d)
    return d.xmat[env.model.body(f"{side}_foot").id].reshape(3, 3)[:, 2]


def test_imitation_reference_keeps_the_sole_level():
    """On the robot preset the reference sole stays level through the swing
    knee bend (pitch) and the sidestep hip-roll oscillation (roll)."""
    env = _cpu(**_robot_kw())
    cmd = np.array([0.4, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0])
    worst = 0.0
    for phase in np.linspace(-np.pi, np.pi, 13):
        q = env._mimic_ref(cmd, phase, 1.5)
        for side in "LR":
            n = _sole_normal(env, q, side)
            worst = max(worst, np.degrees(np.arccos(np.clip(n[2], -1, 1))))
    assert worst < 0.5, f"reference sole tilts {worst:.1f} deg"
    # the prototype formula (mimic_sole_level off) does tilt it in swing
    legacy = _cpu(**_robot_kw(mimic_sole_level=False))
    q = legacy._mimic_ref(cmd, np.pi / 2, 1.5)
    n = _sole_normal(legacy, q, "L")
    assert np.degrees(np.arccos(n[2])) > 10.0


# ------------------------------------------------------------------ mirror
def test_mirror_over_policy_joints_matches_physics():
    """Mirrored episodes on the robot plant (neck held) produce mirrored
    observations under obs_perm_signs(joints=policy joints)."""
    import mirror
    kw = _robot_kw(domain_rand=False, imu_obs=False, quantize_ticks=False,
                   latency_ms=0.0, latency_ms_max=None, latency_jitter_ms=0.0,
                   backlash_deg=0.0, backlash_deg_max=None,
                   zero_offset_deg=0.0, push_prob=0.0)
    a = _cpu(cmd_fixed=(0.3, 0.10, 0.4, 1.0, -1.0, 0.0, 0.02), **kw)
    b = _cpu(cmd_fixed=(0.3, -0.10, -0.4, 1.0, 1.0, 0.0, 0.02), **kw)
    perm, sign = mirror.obs_perm_signs(a.model, ncmd=7, hist_len=3,
                                       joints=a._act_names)
    jperm, jsign = mirror.joint_perm_signs(a.model)   # all hinges, neck too
    a.reset(seed=3)
    b.reset(seed=3)
    q, v = np.array(a.data.qpos), np.array(a.data.qvel)
    b.data.qpos[:] = np.concatenate([mirror.base_qpos_mirror(q[:7]),
                                     mirror.mirror(q[7:], jperm, jsign)])
    b.data.qvel[:] = np.concatenate([mirror.base_qvel_mirror(v[:6]),
                                     mirror.mirror(v[6:], jperm, jsign)])
    mujoco.mj_forward(b.model, b.data)
    b._gait_phase = (a._gait_phase + 2 * np.pi) % (2 * np.pi) - np.pi
    b._gait_freq = a._gait_freq
    zero = np.zeros(12, np.float32)
    for it in range(5):
        oa, *_ = a.step(zero)
        ob, *_ = b.step(zero)
        if it < 2:
            continue      # b's history ring still holds pre-mirror frames
        err = np.max(np.abs(ob - mirror.mirror(oa, perm, sign)))
        assert err < 2e-2, f"obs mirror mismatch {err:.4f} at step {it}"


# ----------------------------------------------------------------- referee
def test_referee_runs_on_the_robot_plant():
    import eval_precision as E
    import eval_ref as R
    cfg = _robot_kw()
    cfg["xml_path"] = os.path.basename(cfg["xml_path"])
    assert E.is_robot_plant(ROBOT_XML) and not E.is_robot_plant(PROTO_XML)
    assert E.referee_payload(ROBOT_XML) == 0.0
    assert E.referee_payload(PROTO_XML) == E.PROTOTYPE_PAYLOAD_KG
    assert {"recover_sit", "recover_fallen"} <= set(E.ROBOT_NOT_APPLICABLE)
    env = E.make_env(cfg, 11.0, False, ROBOT_XML)
    assert env._payload_bid is None and env.action_space.shape == (12,)
    assert env.held_joints == HELD17
    reg = E._registry()
    zero = lambda obs: np.zeros(12, np.float32)                 # noqa: E731
    mass = float(env.model.body_mass.sum())
    res = E.run_one(env, zero, reg["stand_10s"][1], seed=7, record=False,
                    N=env._nominal_h, mass=mass)
    assert {"success", "metrics", "fell", "shared"} <= set(res)
    assert np.isfinite(res["shared"]["mean_watts"])
    # the quick referee builds the same world from the same config
    renv = R.make_env(cfg)
    assert renv.action_space.shape == (12,) and renv.held_joints == HELD17
