"""Parity test: MJX env vs CPU BimoWalkerEnv.

Findings this test encodes (2026-07-13 investigation):
  * Physics arithmetic is EXACT between engines: synced single steps match to
    ~1e-16 in float64, including floor contacts, the STS3215 torque envelope,
    latency, and backlash.
  * The ONE semantic difference is the contact-manifold generator: MJX's
    plane_convex/box colliders emit only points within a 1 mm skin of the
    deepest penetration (mjx/_src/collision_convex.py), where CPU MuJoCo
    emits every penetrating corner. Manifolds therefore differ only in
    tilted-deep-penetration transients (impact instants) and foot-on-foot
    clipping. In settled stance (~0.2 mm penetration) manifolds are
    identical. Consequence: individual impact events resolve slightly
    differently -> trajectories diverge chaotically at impacts, exactly as
    they would under any 1e-7 perturbation. Mitigation: DR + every trained
    policy is refereed by the CPU harness (eval_policy/eval_commands)
    before being believed.

Gates (run in float64 so precision is not the limiter):
  1. AIRBORNE synced-step parity, latency 6 ms + backlash 0.5 deg active:
     validates the whole actuator/reward/obs port with no contacts involved.
  2. GROUNDED synced-step parity with gentle actions around stance
     (matched-manifold regime): validates contact physics where the
     manifolds agree.
  3. float32 free-rollout drift: informational only (chaos, not error).

  R. the ROBOT's plant (blocks R1-R3, see robot_blocks below): the same
     airborne/stance gates with 12 policy joints, the neck and arms held,
     per-servo (Plan B) stiffness and the sole-level imitation reference.

Run:  .venv/bin/python sim/mjx/parity_test.py               # both plants
      .venv/bin/python sim/mjx/parity_test.py --robot-only  # R1-R3 only
"""
import os
import sys

import jax
jax.config.update("jax_enable_x64", True)

import jax.numpy as jp
import numpy as np
from mujoco import mjx

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from walker_env import BimoWalkerEnv
from env_mjx import BimoMJXEnv

XML = os.path.join(HERE, "..", "bimo_biped_v2.xml")

SHARED = dict(
    supply_voltage=11.1,
    w_energy=0.002, w_action_rate=0.05, w_power=0.02,
    w_feet_air=0.1, w_single_support=0.05, w_lateral=0.5, w_pitch_rate=0.05,
    w_track_v=2.0, w_track_w=1.0,
    latency_ms=6.0, backlash_deg=0.5,
    cmd_fixed=(0.6, 0.0), imu_obs=False,
    action_map="full", hip_flex_deg=110.0,   # the get-up study additions
)

cpu = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                    command_mode=True, domain_rand=False, **SHARED)
gpu = BimoMJXEnv(xml_path=XML, domain_rand=False, **SHARED)
step_mjx = jax.jit(gpu.step)


def sync(state, cpu, gpu_env=None):
    # np.array copies everywhere: jp.asarray can alias numpy buffers
    # zero-copy on the CPU backend, and the CPU env mutates _prev_action
    # and _air_time IN PLACE during step -- an aliased array silently reads
    # post-step values inside the later jit call (cost: hours of debugging).
    gpu_env = gpu if gpu_env is None else gpu_env
    d = state.data.replace(qpos=jp.asarray(np.array(cpu.data.qpos)),
                           qvel=jp.asarray(np.array(cpu.data.qvel)))
    d = mjx.forward(gpu_env.model, d)
    return state._replace(
        data=d,
        prev_action=jp.asarray(np.array(cpu._prev_action)),
        last_target=jp.asarray(np.array(cpu._last_target)),
        step_i=jp.asarray(cpu._step_i, dtype=jp.int32),
        air_time=jp.asarray(np.array(cpu._air_time)),
        last_air=jp.asarray(np.array(cpu._last_air)),
        cmd=jp.asarray(np.array(cpu._cmd)),
        traj=jp.asarray(np.array(cpu._traj)),
        traj_on=jp.asarray(float(cpu._traj_on)),
        recover_slot=jp.asarray(1.0 if cpu._recover_ep else 0.0),
        recovered=jp.asarray(1.0 if cpu._recovered else 0.0),
        stand_streak=jp.asarray(float(cpu._stand_streak)),
        best_h=jp.asarray(float(cpu._best_h)),
        head_ref=jp.asarray(float(cpu._head_ref)),
        rise_t0=jp.asarray(float(cpu._rise_t0)),
        gait_freq=jp.asarray(float(cpu._gait_freq)),
        gait_phase=jp.asarray(float(cpu._gait_phase)),
        obs_hist=(jp.asarray(np.array(cpu._obs_hist))
                  if cpu.obs_hist_len > 1 else state.obs_hist),
    )


def _mjx_ncon(state):
    return _mjx_ncon_of(state)


def _mjx_ncon_of(state):
    cc = (state.data._impl.contact if hasattr(state.data, "_impl")
          else state.data.contact)
    return int(np.sum(np.asarray(cc.dist) < 0))


def run_block(name, n_steps, act_fn, prep_cpu, gates, matched_only=False,
              min_frac=0.8, envs=None):
    cpu_e, gpu_e, step_e = envs if envs is not None else (cpu, gpu, step_mjx)
    # actuated qpos/qvel slices derived from the env (8-DOF -> [7:15]/[6:14];
    # 10-DOF hip-yaw -> [7:17]/[6:16]; every SERVO, held ones included, on
    # the robot's plant) so the gate works on any plant
    jq, jv = cpu_e._sqpos, cpu_e._sqvel
    obs_c, _ = cpu_e.reset(seed=0)
    prep_cpu(cpu_e)
    state = gpu_e.reset(jax.random.PRNGKey(0))
    worst = dict(qpos=0.0, qvel=0.0, reward=0.0, obs=0.0)
    used = skipped = 0
    for t in range(n_steps):
        a = act_fn(t)
        state = sync(state, cpu_e, gpu_e)
        # matched_only screens on the SYNCED (pre-step) manifold: that is the
        # manifold the step's contact forces are computed from. Screening on
        # post-step ncon (the original code) misses steps where the engines
        # disagree pre-step but re-converge post-step -- found 2026-07-17 when
        # the silicone-pad sole drop (b4f22bc) deepened the settling transient
        # enough that MJX's 1 mm-skin collider kept a 0.05 mm grazing corner
        # CPU's box-plane collider omits, at exactly one force-carrying step.
        pre_cpu_ncon = cpu_e.data.ncon
        pre_mjx_ncon = _mjx_ncon(state)
        obs_c, r_c, term_c, trunc_c, _ = cpu_e.step(a)
        state = step_e(state, jp.asarray(a))
        if matched_only and pre_cpu_ncon != pre_mjx_ncon:
            skipped += 1     # manifold mismatch: regime this block can't gate
            continue
        used += 1
        worst["qpos"] = max(worst["qpos"], float(np.max(np.abs(
            np.asarray(state.data.qpos)[jq] - cpu_e.data.qpos[jq]))))
        worst["qvel"] = max(worst["qvel"], float(np.max(np.abs(
            np.asarray(state.data.qvel)[jv] - cpu_e.data.qvel[jv]))))
        worst["reward"] = max(worst["reward"], abs(float(state.reward) - r_c))
        worst["obs"] = max(worst["obs"], float(np.max(np.abs(
            np.asarray(state.obs) - obs_c))))
        if trunc_c:
            obs_c, _ = cpu_e.reset(seed=1000 + t)
            prep_cpu(cpu_e)
    enough = used >= min_frac * n_steps
    ok = enough and all(worst[k] < gates[k] for k in gates)
    print(f"== {name}: {'PASS' if ok else 'FAIL'} "
          f"({used} steps compared, {skipped} skipped) ==")
    for k in ("qpos", "qvel", "reward", "obs"):
        gate = f"  (gate {gates[k]:.0e})" if k in gates else ""
        print(f"   worst |d{k}| = {worst[k]:.2e}{gate}")
    return ok


# -- 1. airborne: pure actuator/reward/obs arithmetic, no contacts ------------
def hoist(env):
    """Torso 1.5 m up, zero velocity; re-hoisted every 20 steps so the robot
    never reaches the floor. Legs are splayed by the action offset below so
    the feet never touch each other (sole-sole box contact is exactly the
    manifold-divergent case this block must exclude)."""
    env.data.qpos[2] = 1.5
    env.data.qvel[:] = 0.0
    import mujoco
    mujoco.mj_forward(env.model, env.data)

# hip-roll offsets splay the legs; modest amplitude keeps feet apart
_SPLAY = np.array([0.5, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0])


# -- R. the ROBOT's plant ------------------------------------------------------
# sim/bimo_biped_v6ar.xml under the robot preset (robot_plant.py): the 12 leg
# joints as the policy, the neck (and arms) HELD at fixed targets, Plan B
# per-servo stiffness (kp AND kd x4 on hip roll, ankle roll, knee), hip
# flexion -120, the sole-level imitation reference with the ankle roll in it,
# tick quantization. R1 airborne arithmetic under a moving command (the mimic
# reference's hip-roll oscillation drives the ankle-roll term), R2 grounded
# stance on the 16 sole pads, R3 the same airborne gate on a generated
# 17-servo plant with both arms held at the walking pose. `--robot-only` runs
# just these (tests/test_robot_plant.py does, in a subprocess).
def robot_blocks():
    import tempfile
    import robot_plant
    from mujoco import mj_forward

    def rob_kw(xml=None, **over):
        kw = dict(
            supply_voltage=11.1, w_energy=0.002, w_action_rate=0.05,
            w_power=0.02, w_feet_air=0.1, w_single_support=0.05,
            w_lateral=0.5, w_pitch_rate=0.05, w_track_v=2.0, w_track_w=2.0,
            w_track_h=1.0, w_lift=1.0, w_track_foot=1.0, w_symmetry=0.5,
            gait_clock=True, w_feet_phase=1.0, w_feet_slip=0.25,
            w_orientation=1.0, w_ang_vel_xy=0.15, w_pose=0.5,
            w_dof_limits=1.0, obs_hist_len=3, joint_frictionloss=0.05,
            joint_armature=0.028, w_mimic=1.0, w_com_stance=0.75,
            w_heading=1.0, w_foot_under=0.75, latency_ms=6.0,
            backlash_deg=0.5, ext_cmd=True, fall_cost=10.0, cmd_dense=True,
            cmd_fixed=(0.3, -0.1, 0.2, 0.85, 0.0, 0.0, 0.0), imu_obs=False,
            action_map="full", quantize_ticks=True)
        kw.update(robot_plant.robot_env_kwargs(xml))
        kw.update(over)
        return kw

    def envs_for(kw):
        c = BimoWalkerEnv(actuator_model="sts3215", command_mode=True,
                          domain_rand=False, **kw)
        g = BimoMJXEnv(domain_rand=False, **kw)
        return c, g, jax.jit(g.step)

    def air_acts(env):
        n = env._nq_act
        splay = np.zeros(n)
        splay[env._legL["hip_roll"]] = 0.5
        splay[env._legR["hip_roll"]] = -0.5

        def act(t):
            if t % 20 == 0:
                hoist(env)
            return (splay + 0.3 * np.sin(0.353 * t + np.arange(n) * 0.7)
                    ).astype(np.float32)
        return act

    def layout_ok(c, g, n_servo, n_held):
        """The preset is really on, identically in both engines: policy vs
        servo counts, the stiffness profile the robot preset resolves to
        (robot_plant.KP_SCALE), the held joints at their hold in qpos0."""
        from walker_env import servo_kp_scale_vector
        prof = np.asarray(c._kp_prof)
        want = servo_kp_scale_vector(list(c._servo_names), robot_plant.KP_SCALE)
        held_q = [float(c.model.qpos0[c._jq0 + c._sname2i[k]])
                  for k in c.held_joints]
        want_q = [np.deg2rad(v) for v in c.held_joints.values()]
        return (c._n_servo == n_servo and c._nq_act == 12
                and g.action_size == 12 and len(c.held_joints) == n_held
                and np.allclose(prof, want)
                and np.array_equal(prof, np.asarray(g._kp_prof))
                and np.array_equal(np.asarray(c._kd_prof),
                                   np.asarray(g._kd_prof))
                and np.allclose(held_q, want_q)
                and c.observation_space.shape[0] == g.obs_size == 3 * 55)

    c1, g1, s1 = envs_for(rob_kw())
    lay1 = layout_ok(c1, g1, 17, 5)
    okr1 = run_block(
        "R1. ROBOT plant airborne arithmetic (12 policy + neck held, Plan B "
        "kp/kd, sole-level mimic, quantized)", 100, air_acts(c1), hoist,
        dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
        envs=(c1, g1, s1)) and lay1
    print(f"   layout: {c1._n_servo} servos, {c1._nq_act} policy joints, held "
          f"{c1.held_joints}, x4 on "
          f"{[n for n, p in zip(c1._servo_names, c1._kp_prof) if p != 1]}"
          f" -> {'OK' if lay1 else 'WRONG'}")

    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0)))
    n2 = c2._nq_act
    okr2 = run_block(
        "R2. ROBOT plant grounded stance (16 sole pads, matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))
                        ).astype(np.float32),
        lambda env: None, dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3),
        matched_only=True, min_frac=0.5, envs=(c2, g2, s2))

    with tempfile.TemporaryDirectory() as td:
        xml17 = robot_plant.write_arms_test_plant(
            os.path.join(td, "arms17.xml"))
        c3, g3, s3 = envs_for(rob_kw(xml17))
        lay3 = layout_ok(c3, g3, 17, 5)
        okr3 = run_block(
            "R3. 17-servo plant (arms held at the folded rest pose) "
            "airborne arithmetic", 100, air_acts(c3), hoist,
            dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
            envs=(c3, g3, s3)) and lay3
        # the held arms actually HOLD: after the block the shoulders and
        # elbows sit at their rest targets (within the backlash band + sag)
        mj_forward(c3.model, c3.data)
        hold = robot_plant.HOLD_DEG
        sh = [float(np.degrees(c3.data.qpos[c3._jq0 + c3._sname2i[k]]))
              for k in ("L_shoulder", "R_shoulder")]
        el = [float(np.degrees(c3.data.qpos[c3._jq0 + c3._sname2i[k]]))
              for k in ("L_elbow", "R_elbow")]
        held_live = (all(abs(v - hold["shoulder"]) < 3.0 for v in sh)
                     and all(abs(v - hold["elbow"]) < 3.0 for v in el))
        okr3 = okr3 and held_live
        print(f"   layout {'OK' if lay3 else 'WRONG'}; shoulders at "
              f"{sh[0]:.1f} / {sh[1]:.1f} deg, elbows at {el[0]:.1f} / {el[1]:.1f} deg "
              f"(rest {hold['shoulder']:g} / {hold['elbow']:g})")
    return okr1 and okr2 and okr3


if "--robot-only" in sys.argv:
    _ok = robot_blocks()
    print("\nPARITY (robot plant):", "PASS" if _ok else "FAIL")
    sys.exit(0 if _ok else 1)


class AirActs:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu)   # both get re-synced right after, so this is shared
        return (_SPLAY + 0.3 * np.sin(0.35 * t + np.arange(8) * 0.7)
                ).astype(np.float32)

ok1 = run_block(
    "1. airborne arithmetic (latency 6 ms, backlash 0.5 deg)", 100, AirActs(),
    hoist, dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5))

# -- 2. grounded quasi-static stance: flat feet, matched 4-corner manifolds ---
ok2 = run_block(
    "2. grounded stance physics (matched manifolds)", 100,
    lambda t: (0.05 * np.sin(0.25 * t + np.arange(8))).astype(np.float32),
    lambda env: None, dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3),
    matched_only=True, min_frac=0.5)

# -- 2b. get-up mode: recovery-reward arithmetic from fallen states -----------
cpu_g = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, getup=True,
                      supply_voltage=11.1, w_energy=0.002, w_action_rate=0.15,
                      w_power=0.008, w_pitch_rate=0.1, w_upright=0.0,
                      alive_bonus=0.0, imu_obs=False)
gpu_g = BimoMJXEnv(xml_path=XML, domain_rand=False, getup=True,
                   supply_voltage=11.1, w_energy=0.002, w_action_rate=0.15,
                   w_power=0.008, w_pitch_rate=0.1, w_upright=0.0,
                   alive_bonus=0.0, imu_obs=False)
step_g = jax.jit(gpu_g.step)
obs_c, _ = cpu_g.reset(seed=3)
sg = gpu_g.reset(jax.random.PRNGKey(3))
worst_r = worst_o = 0.0
used_g = 0
import mujoco as _mj
for t in range(80):
    if t % 20 == 0:      # hoist: recovery reward has no contact terms, so
        cpu_g.data.qpos[2] = 1.5          # gate its arithmetic contact-free
        cpu_g.data.qvel[:] = 0.0          # (heap dynamics divergence is the
        _mj.mj_forward(cpu_g.model, cpu_g.data)   # documented manifold caveat)
    a = (_SPLAY + 0.3 * np.sin(0.3 * t + np.arange(8))).astype(np.float32)
    d = sg.data.replace(qpos=jp.asarray(np.array(cpu_g.data.qpos)),
                        qvel=jp.asarray(np.array(cpu_g.data.qvel)))
    d = mjx.forward(gpu_g.model, d)
    sg = sg._replace(data=d, prev_action=jp.asarray(np.array(cpu_g._prev_action)),
                     last_target=jp.asarray(np.array(cpu_g._last_target)),
                     step_i=jp.asarray(cpu_g._step_i, dtype=jp.int32),
                     air_time=jp.asarray(np.array(cpu_g._air_time)),
                     cmd=jp.asarray(np.array(cpu_g._cmd)))
    obs_c, r_c, term_c, trunc_c, info_c = cpu_g.step(a)
    sg = step_g(sg, jp.asarray(a))
    if term_c:
        raise SystemExit("getup CPU env terminated -- must never happen")
    if float(sg.done) != 0.0:
        raise SystemExit("getup MJX env set done -- must never happen")
    if cpu_g.data.ncon != 0:
        continue                          # airborne-only gate
    used_g += 1
    worst_r = max(worst_r, abs(float(sg.reward) - r_c))
    worst_o = max(worst_o, float(np.max(np.abs(np.asarray(sg.obs) - obs_c))))
ok3 = used_g >= 40 and worst_r < 1e-5 and worst_o < 1e-5
print(f"== 2b. get-up recovery-reward arithmetic (contact-free): "
      f"{'PASS' if ok3 else 'FAIL'} ({used_g}/80 airborne steps) ==")
print(f"   worst |dreward| = {worst_r:.2e}  worst |dobs| = {worst_o:.2e}")

# -- 2c/2d. ext_cmd precision mode (2026-07-17) --------------------------------
# 2c airborne: gates the new kernel arithmetic (2D velocity, height-vs-crouch,
# swing-foot target) with a fixed one-leg command carrying a foot offset.
# 2d grounded stance: gates the lift contact-pattern term (lift_ok) and the
# crouch-scaled fall floor in the matched-manifold regime.
EXT = dict(
    supply_voltage=11.1, w_energy=0.002, w_action_rate=0.05, w_power=0.02,
    w_feet_air=0.1, w_single_support=0.05, w_lateral=0.5, w_pitch_rate=0.05,
    w_track_v=2.0, w_track_w=2.0, w_track_h=1.0, w_lift=1.0, w_track_foot=1.0,
    w_symmetry=0.5,
    # plan-v2 Phase A terms under gate (2026-07-20)
    gait_clock=True, w_feet_phase=1.0, w_feet_slip=0.25, w_orientation=1.0,
    w_ang_vel_xy=0.15, w_pose=0.5, w_dof_limits=1.0, obs_hist_len=3,
    joint_frictionloss=0.05, joint_armature=0.028,
    w_mimic=1.0,   # Phase B imitation arithmetic under gate (2026-07-22)
    w_com_stance=0.75, w_heading=1.0,   # knee-lift CoM + heading integrator
    w_foot_under=0.75,                  # raised-foot-under-hip kernel (skills_v4)
    # arithmetic under gate (2026-07-23; cmd_fixed lifts, so the CoM kernel
    # is live; heading kernel live in blocks 2d/2e via nonzero wz commands)
    latency_ms=6.0, backlash_deg=0.5, ext_cmd=True, fall_cost=10.0,
    cmd_fixed=(0.0, 0.0, 0.0, 0.9, -1.0, 0.01, 0.02), imu_obs=False,
    action_map="full", hip_flex_deg=110.0,
)
cpu_e = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT)
gpu_e = BimoMJXEnv(xml_path=XML, domain_rand=False, **EXT)
step_e = jax.jit(gpu_e.step)


class AirActsExt:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_e)
        return (_SPLAY + 0.3 * np.sin(0.35 * t + np.arange(8) * 0.7)
                ).astype(np.float32)

ok_e1 = run_block(
    "2c. ext_cmd airborne arithmetic (lift -1, foot target, crouch 0.9)",
    100, AirActsExt(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_e, gpu_e, step_e))

ok_e2 = run_block(
    "2d. ext_cmd grounded stance (lift_ok pattern, matched manifolds)", 100,
    lambda t: (0.05 * np.sin(0.25 * t + np.arange(8))).astype(np.float32),
    lambda env: None, dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3),
    matched_only=True, min_frac=0.5, envs=(cpu_e, gpu_e, step_e))

# -- 2e. ext_cmd dense-progress arithmetic (moving command, airborne) ---------
EXT_D = dict(EXT)
EXT_D.update(cmd_dense=True,
             cmd_fixed=(0.3, -0.1, 0.2, 0.85, 0.0, 0.0, 0.0))
cpu_d = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_D)
gpu_d = BimoMJXEnv(xml_path=XML, domain_rand=False, **EXT_D)
step_d = jax.jit(gpu_d.step)


class AirActsExtD:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_d)
        return (_SPLAY + 0.3 * np.sin(0.35 * t + np.arange(8) * 0.7)
                ).astype(np.float32)

ok_e3 = run_block(
    "2e. ext_cmd dense progress (cmd 0.3,-0.1,0.2, crouch 0.85, airborne)",
    100, AirActsExtD(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_d, gpu_d, step_d))

# -- 2f. ext_cmd recovery-from-fallen arithmetic (contact-free) ----------------
# Gates the recovery-primary reward + recovered-flag/termination gating that
# recover_mix adds to ext mode (2026-07-18). Same airborne-only screening as
# block 2b: fallen/heap contact manifolds are the documented divergence.
EXT_R = dict(EXT)
EXT_R.pop("cmd_fixed")
EXT_R.update(recover_mix=1.0, cmd_dense=True, w_rise_dofvel=0.002,
             w_rise_ref=2.0,   # staged-rise reference arithmetic under gate
             w_up_vel=2.0)     # getup_v5 momentum incentive under gate
cpu_r = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_R)
gpu_r = BimoMJXEnv(xml_path=XML, domain_rand=False, **EXT_R)
step_r = jax.jit(gpu_r.step)
obs_c, _ = cpu_r.reset(seed=5)
sr = gpu_r.reset(jax.random.PRNGKey(5))
worst_rr = worst_ro = 0.0
used_r = 0
saw_down = saw_up = False
for t in range(80):
    if t % 20 == 0:
        # mid-range joints + tilted torso at height: exercises the
        # height/up_gate/standing branches of the recovery reward WITHOUT
        # riding joint limits (the ragdoll clip parks joints exactly on
        # their limit rows -- limit constraints are solver territory, the
        # same divergence class as contacts, so this block avoids them)
        ang = 0.3 + 0.5 * (t / 80.0) * np.pi
        cpu_r.data.qpos[:] = cpu_r.model.qpos0
        cpu_r.data.qpos[2] = 1.5
        cpu_r.data.qpos[3:7] = [np.cos(ang / 2), 0.0, np.sin(ang / 2), 0.0]
        cpu_r.data.qvel[:] = 0.0
        _mj.mj_forward(cpu_r.model, cpu_r.data)
    # alternate the gated phase: first half graded while "down" (recovery
    # primary; the flag is re-forced each step because a hoisted torso
    # trivially satisfies the stand test), second half with the flag set
    cpu_r._recovered = (t >= 40)   # force the phase under test each step
    pre_rec = cpu_r._recovered
    a = (_SPLAY + 0.3 * np.sin(0.3 * t + np.arange(8))).astype(np.float32)
    sr = sync(sr, cpu_r, gpu_r)
    obs_c, r_c, term_c, trunc_c, info_c = cpu_r.step(a)
    sr = step_r(sr, jp.asarray(a))
    saw_down |= not pre_rec
    saw_up |= pre_rec
    n_lim = int(np.sum(np.asarray(cpu_r.data.efc_type[:cpu_r.data.nefc])
                       == int(_mj.mjtConstraint.mjCNSTR_LIMIT_JOINT)))
    if cpu_r.data.ncon != 0 or _mjx_ncon(sr) != 0 or n_lim != 0:
        # contact-free AND limit-free gate: active joint-LIMIT rows are
        # solver territory -- the same engine-divergence class as contact
        # manifolds (an early ragdoll variant of this block rode its limits
        # and diverged ~1.6% in gyro). Friction-loss rows are fine (always
        # active, arithmetically identical -- block 1 passes over them).
        continue
    used_r += 1
    worst_rr = max(worst_rr, abs(float(sr.reward) - r_c))
    worst_ro = max(worst_ro, float(np.max(np.abs(np.asarray(sr.obs) - obs_c))))
ok_e4 = used_r >= 40 and saw_down and saw_up and worst_rr < 1e-5 \
    and worst_ro < 1e-5
print(f"== 2f. ext_cmd recovery arithmetic (contact-free): "
      f"{'PASS' if ok_e4 else 'FAIL'} ({used_r}/80 airborne, "
      f"down+up phases={saw_down and saw_up}) ==")
print(f"   worst |dreward| = {worst_rr:.2e}  worst |dobs| = {worst_ro:.2e}")

# -- 2g. hip-yaw (10-DOF) plant: contact-free ext_cmd arithmetic --------------
# Morphology-A/B plant bimo_biped_v3yaw.xml adds a hip-yaw joint per side (10
# actuated joints, taller torso). Same airborne dense-progress pattern as 2e
# -- a fixed nonzero command INCLUDING wz -- proving the joint-count/layout
# parameterization is arithmetically identical between the CPU referee and MJX
# on the 10-DOF plant (the 8-DOF gates above prove the 8-DOF path stays
# bit-exact). Airborne so the manifold caveat never applies; kept fast.
XML_YAW = os.path.join(HERE, "..", "bimo_biped_v3yaw.xml")
EXT_Y = dict(EXT)
EXT_Y.update(cmd_dense=True,
             cmd_fixed=(0.3, -0.1, 0.2, 0.85, 0.0, 0.0, 0.0))
cpu_y = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_Y)
gpu_y = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, **EXT_Y)
step_y = jax.jit(gpu_y.step)
_n_act_y = cpu_y._nq_act
_splay_y = np.zeros(_n_act_y)                 # hip-roll splay keeps feet apart
_splay_y[cpu_y._legL["hip_roll"]] = 0.5
_splay_y[cpu_y._legR["hip_roll"]] = -0.5


class AirActsYaw:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_y)
        # 0.353 (was 0.35): with the 2026-07-28 inertias the old pattern's
        # t=88 landed several joint torques within machine epsilon of the
        # sts3215 envelope/backlash branch, and XLA-vs-numpy op ordering
        # flipped it (one step at dqvel 5e-5; the other 99 at 1e-12 --
        # verified by per-step probe). Boundary luck, not divergence; the
        # phase nudge steps off the knife edge at full gate strictness.
        return (_splay_y + 0.3 * np.sin(0.353 * t + np.arange(_n_act_y) * 0.7)
                ).astype(np.float32)

ok_e5 = run_block(
    "2g. hip-yaw 10-DOF ext_cmd airborne arithmetic (cmd 0.3,-0.1,0.2, wz)",
    100, AirActsYaw(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_y, gpu_y, step_y))

# -- 3. gait-amplitude manifold statistics (informational, no gate) -----------
print("== 3. gait-amplitude contact-manifold statistics (informational) ==")
obs_c, _ = cpu.reset(seed=7)
state = gpu.reset(jax.random.PRNGKey(7))
matched = total_con = 0
for t in range(150):
    a = (0.4 * np.sin(0.35 * t + np.arange(8) * 0.7)).astype(np.float32)
    state = sync(state, cpu)
    obs_c, r_c, term_c, trunc_c, _ = cpu.step(a)
    state = step_mjx(state, jp.asarray(a))
    if cpu.data.ncon:
        total_con += 1
        cc = (state.data._impl.contact if hasattr(state.data, "_impl")
              else state.data.contact)
        mjx_n = int(np.sum(np.asarray(cc.dist) < 0))
        matched += int(mjx_n == cpu.data.ncon)
    if term_c or trunc_c:
        obs_c, _ = cpu.reset(seed=2000 + t)
print(f"   contact steps: {total_con}, same contact-point count: {matched} "
      f"({100.0 * matched / max(total_con, 1):.0f}%)")
print("   (mismatches = tilted/deep penetration where MJX's 1 mm-skin")
print("    manifold pruning drops corners CPU keeps -- impact transients.")
print("    Settled stance manifolds are identical. Referee: CPU evals.)")

# -- 2h. terrain-mosaic + knee-weighted mimic arithmetic (contact-free) -------
# loco_v7knee (2026-07-29): the mosaic ground-height lookup (bilinear on the
# shared terrain_mosaic.npz) enters height/foot_clear/obs, and the mimic
# kernel gains a per-joint knee weight. Airborne over a ROUGH tile so the
# _gz/_ground_z arithmetic is live in every step's height channel while the
# manifold caveat never applies. Both envs load the same npz by construction.
EXT_T = dict(EXT)
EXT_T.update(cmd_dense=True, mimic_knee_w=4.0,
             cmd_fixed=(0.3, -0.1, 0.2, 0.85, 0.0, 0.0, 0.0))
cpu_t = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False,
                      terrain_mosaic=True, **EXT_T)
gpu_t = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, terrain=True,
                   **EXT_T)
step_t = jax.jit(gpu_t.step)
_splay_t = np.zeros(cpu_t._nq_act)
_splay_t[cpu_t._legL["hip_roll"]] = 0.5
_splay_t[cpu_t._legR["hip_roll"]] = -0.5


def hoist_rough(env):
    """Hoist over a rough mosaic tile (x=2.3, y=0.7 has nonzero height) so
    the ground lookup returns a varying, nonzero field under the flight."""
    env.data.qpos[0] = 2.3
    env.data.qpos[1] = 0.7
    env.data.qpos[2] = 1.5
    env.data.qvel[:] = 0.0
    import mujoco as _mj
    _mj.mj_forward(env.model, env.data)


class AirActsTerrain:
    def __call__(self, t):
        if t % 20 == 0:
            hoist_rough(cpu_t)
        return (_splay_t + 0.3 * np.sin(0.31 * t
                                        + np.arange(cpu_t._nq_act) * 0.7)
                ).astype(np.float32)

ok_e6 = run_block(
    "2h. terrain-mosaic ground-lookup + knee-weighted mimic (airborne)",
    100, AirActsTerrain(), hoist_rough,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_t, gpu_t, step_t))

# -- 2i. servo tick quantization + per-episode crouch draw (contact-free) -----
# task #16 (2026-07-31): quantize_ticks puts the STS3215 encoder/bus quantizer
# inside the env -- joint pos/vel OBSERVATIONS through angle->tick(round)->angle
# (4096/rev, zero 2048, dir +1 per sim/sil/cal/cal_nominal.json) and reg-58
# integer steps/s, plus the commanded target angles onto the same tick grid
# before the PD/backlash/latency path. cmd_crouch_range varies the otherwise
# FROZEN cmd[3] (SIL finding #2) per episode.
#
# Rounding must be BIT-identical, not close: one flipped tick is a 1.5e-3 rad
# obs step, 150x the gate. np.rint and jp.round are both round-half-to-even,
# and the inputs are bit-identical after sync, so the quantizers agree exactly
# -- this block is what proves it (and would catch a knife-edge, which the 2g
# recipe fixes by nudging the probe phase, never by loosening the gate).
#
# cmd_fixed pins channel 3 at exactly 1.0, which is precisely the value the
# per-episode draw substitutes -- so the CPU env's draw (0.7..0.95) is live in
# every step, and sync() carries it to MJX like every other per-episode draw
# (servo/latency/backlash: the RNG streams differ by construction, the
# arithmetic downstream of the draw is what this gates -- here the crouch-gated
# g_skill, the height kernel and the crouch-scaled fall floor).
EXT_Q = dict(EXT)
EXT_Q.update(cmd_dense=True,
             cmd_fixed=(0.3, -0.1, 0.2, 1.0, 0.0, 0.0, 0.0))
_QK = dict(quantize_ticks=True, cmd_crouch_range=(0.7, 0.95))
cpu_q = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_Q, **_QK)
gpu_q = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, **EXT_Q, **_QK)
step_q = jax.jit(gpu_q.step)
_splay_q = np.zeros(cpu_q._nq_act)
_splay_q[cpu_q._legL["hip_roll"]] = 0.5
_splay_q[cpu_q._legR["hip_roll"]] = -0.5


class AirActsQuant:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_q)
        return (_splay_q + 0.3 * np.sin(0.347 * t
                                        + np.arange(cpu_q._nq_act) * 0.7)
                ).astype(np.float32)

ok_e7 = run_block(
    "2i. tick quantization (4096/rev obs+targets) + crouch draw (airborne)",
    100, AirActsQuant(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_q, gpu_q, step_q))
# the block above only gates AGREEMENT; assert the quantizer is actually
# ENGAGED (a no-op quantizer would pass every gate trivially)
_q_obs = cpu_q._obs()[:cpu_q._nq_act]
_q_res = float(np.max(np.abs(_q_obs / (2 * np.pi / 4096)
                             - np.rint(_q_obs / (2 * np.pi / 4096)))))
_q_crouch = float(cpu_q._cmd[3])
# residual is float32-cast noise only (~3e-5 ticks); an UNQUANTIZED obs sits
# anywhere in +/-0.5 ticks, so 1e-3 separates "on the grid" from "not"
ok_e7 = ok_e7 and _q_res < 1e-3 and 0.7 <= _q_crouch <= 0.95
print(f"   quantizer live: obs off-grid residual {_q_res:.1e} ticks, "
      f"crouch cmd {_q_crouch:.3f} (drawn from 0.70..0.95)")

# -- 2j. knee-high march: command path + clearance kernel --------------------
# The march skill (2026-08-01) adds NO observation channel: it is a per-step
# COMMAND PATTERN on the existing 7 channels (vx=vy=wz=0, the lift channel c4
# alternating on the gait clock, c6 raising the swing-foot target from
# lift_height to the plant's knee height) plus one reward kernel (w_knee_high:
# the fraction of the commanded swing height the swing sole reaches, on the
# correct one-foot contact pattern). Both halves have to be bit-identical
# between engines or the CPU referee grades a different skill than MJX trained.
#
# ext_mix is zeroed and march_mix set to 1.0, so EVERY draw is a knee-high
# march and traj_on == 4 for the whole block; cmd_fixed is popped (pinned
# commands keep mode 0 and would skip the evolution entirely, block 2f's
# lesson). The gait clock is on, so the alternation runs off gait_phase --
# which sync() carries to MJX like every other per-episode draw.
#
# 2j (airborne, full strictness) gates the command evolution itself and every
# kernel it feeds through the ELEVATED c6: the swing-foot target, foot_under,
# the obs command channels. 2j-b poses a real one-leg stance so knee_frac is
# non-zero and its arithmetic is gated too (airborne, con_stance is false and
# the kernel is trivially 0 on both sides -- that would gate nothing).
EXT_M = dict(EXT)
EXT_M.pop("cmd_fixed")
EXT_M.update(cmd_dense=True, w_knee_high=1.0,
             ext_mix=(0.0,) * 7, march_mix=1.0)
cpu_m = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_M)
gpu_m = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, **EXT_M)
step_m = jax.jit(gpu_m.step)
_splay_m = np.zeros(cpu_m._nq_act)
_splay_m[cpu_m._legL["hip_roll"]] = 0.5
_splay_m[cpu_m._legR["hip_roll"]] = -0.5
_m_seen = {"lift": set(), "c6": 0.0}


class AirActsMarch:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_m)
        _m_seen["lift"].add(float(np.sign(cpu_m._cmd[4])))
        _m_seen["c6"] = max(_m_seen["c6"], float(cpu_m._cmd[6]))
        return (_splay_m + 0.3 * np.sin(0.359 * t
                                        + np.arange(cpu_m._nq_act) * 0.7)
                ).astype(np.float32)


ok_e8 = run_block(
    "2j. knee-high march command path (c4 alternation, c6 knee target, "
    "airborne)", 120, AirActsMarch(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_m, gpu_m, step_m))
# liveness: the block above only gates AGREEMENT. A march that never
# alternated, or a c6 stuck at 0, would pass it trivially -- assert the
# pattern the skill is made of actually ran, and that the knee-high target
# came off the PLANT (knee joint anchor 0.107 m, sole center 0.003 m).
_m_alt = {-1.0, 1.0}.issubset(_m_seen["lift"])
_m_dz_ok = abs(_m_seen["c6"] - cpu_m._march_dz) < 1e-3 and cpu_m._march_dz > 0
_m_same = abs(cpu_m._march_dz - float(gpu_m._march_dz)) < 1e-12
ok_e8 = ok_e8 and float(cpu_m._traj_on) == 4.0 and _m_alt and _m_dz_ok \
    and _m_same
print(f"   march live: traj_on {float(cpu_m._traj_on):.0f}, lifts "
      f"{sorted(_m_seen['lift'])}, peak c6 {_m_seen['c6']:.4f} m, "
      f"knee-high target {cpu_m.march_clear*100:.1f} cm "
      f"(= {cpu_m.march_clear / cpu_m.lift_height:.2f}x lift_height)")


def pose_march(env, lift):
    """Pose a real one-leg march stance: the COMMANDED swing leg's knee and
    hip flexed (foot well clear of the floor), the other leg planted with its
    corner pads ~0.3 mm into the floor. Sphere-pad contacts are the analytic
    single-point kind both engines agree on, so this stays in the
    matched-manifold regime while making knee_frac non-zero."""
    leg = env._legR if lift > 0 else env._legL
    q = env.model.qpos0.copy()
    # deliberately a PARTIAL lift: the clearance must land strictly between 0
    # and the commanded knee-high target, or the kernel's clip saturates and
    # the division under test is masked (see the interior-value assertion).
    #
    # Re-posed 2026-08-02 with the corrected knee sign (plant axis "0 -1 0",
    # negative = human flexion). The old (-0.62, -0.31) swung the shank
    # FORWARD on the old mirrored plant; flexing it now dips the swing foot
    # 10.5 mm THROUGH the floor, so con_swing was true and knee_frac collapsed
    # to a trivial 0 -- the kernel arithmetic this block exists to gate stopped
    # being exercised. (-0.90, -0.90) is a real high-knee stance: swing pads
    # 34 mm clear, ncon a flat 8 (stance foot only, matched manifold),
    # knee_frac ~0.44.
    q[env._jq0 + leg["knee"]] = -0.90
    q[env._jq0 + leg["hip_pitch"]] = -0.90
    env.data.qpos[:] = q
    env.data.qvel[:] = 0.0
    _mj.mj_forward(env.model, env.data)
    stance = env._pad_gids[0 if lift > 0 else 1]
    bottom = min(float(env.data.geom_xpos[g][2])
                 - float(env.model.geom_size[g][0]) for g in stance)
    env.data.qpos[2] -= bottom + 3e-4          # 0.3 mm penetration
    _mj.mj_forward(env.model, env.data)


obs_c, _ = cpu_m.reset(seed=11)
sm = gpu_m.reset(jax.random.PRNGKey(11))
worst_mr = worst_mo = 0.0
used_m = skip_m = 0
best_kf = 0.0
interior_kf = None            # a knee_frac strictly inside (0, 1): unclipped
for t in range(80):
    pose_march(cpu_m, float(cpu_m._cmd[4]))
    a = (0.02 * np.sin(0.3 * t + np.arange(cpu_m._nq_act))).astype(np.float32)
    sm = sync(sm, cpu_m, gpu_m)
    pre_c, pre_g = cpu_m.data.ncon, _mjx_ncon(sm)
    obs_c, r_c, term_c, trunc_c, info_c = cpu_m.step(a)
    sm = step_m(sm, jp.asarray(a))
    _kf = float(info_c["knee_frac"])
    best_kf = max(best_kf, _kf)
    if 0.05 < _kf < 0.95:
        interior_kf = _kf
    if pre_c != pre_g:
        skip_m += 1
        continue
    used_m += 1
    worst_mr = max(worst_mr, abs(float(sm.reward) - r_c))
    worst_mo = max(worst_mo, float(np.max(np.abs(np.asarray(sm.obs) - obs_c))))
    if trunc_c or term_c:
        obs_c, _ = cpu_m.reset(seed=5000 + t)
ok_e9 = (used_m >= 40 and interior_kf is not None
         and worst_mr < 1e-3 and worst_mo < 1e-3)
print(f"== 2j-b. knee-high clearance kernel on a posed one-leg march stance: "
      f"{'PASS' if ok_e9 else 'FAIL'} ({used_m} compared, "
      f"{skip_m} skipped) ==")
print(f"   worst |dreward| = {worst_mr:.2e}  worst |dobs| = {worst_mo:.2e}  "
      f"(kernel live: peak knee_frac {best_kf:.2f}, unclipped sample "
      f"{interior_kf if interior_kf is None else round(interior_kf, 3)})")
ok_e8 = ok_e8 and ok_e9

# -- 2j-c. PINNED march cadence (march_hz) ------------------------------------
# march_hz > 0 replaces the gait clock as the march's timebase: a deliberate
# 0.5 s/lift instead of the gait clock's 0.29-0.40 s (integrator call, the
# trainability risk). The flip is an exact integer compare on the control-step
# count precisely because the naive sin(2*pi*f*t) >= 0 test puts the flip on a
# zero crossing, where float32 (MJX) and float64 (CPU) land on opposite sides
# -- that would swap the COMMANDED LEG between engines at every flip, the
# largest possible parity failure, and it would be invisible in any block that
# never crosses one. This block runs 120 steps = 4.8 flips at 1 Hz.
EXT_H = dict(EXT_M)
EXT_H.update(march_hz=1.0)
cpu_h = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_H)
gpu_h = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, **EXT_H)
step_h = jax.jit(gpu_h.step)
_splay_h = np.zeros(cpu_h._nq_act)
_splay_h[cpu_h._legL["hip_roll"]] = 0.5
_splay_h[cpu_h._legR["hip_roll"]] = -0.5
_h_trace = []                      # (step_i, commanded lift) after each step


class AirActsMarchHz:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_h)
        _h_trace.append((int(cpu_h._step_i), float(cpu_h._cmd[4])))
        return (_splay_h + 0.3 * np.sin(0.359 * t
                                        + np.arange(cpu_h._nq_act) * 0.7)
                ).astype(np.float32)


ok_e10 = run_block(
    "2j-c. pinned march cadence march_hz=1.0 (0.5 s/lift, airborne)",
    120, AirActsMarchHz(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_h, gpu_h, step_h))
# liveness: the c4 flip period must be EXACTLY the commanded half cycle --
# 0.5 s at 1 Hz = 25 control steps at the 50 Hz control rate. A cadence that
# silently stayed on the gait clock (1.25-1.75 Hz) would show ~14-20.
_h_flips = [s for (s, v), (_, pv) in zip(_h_trace[1:], _h_trace[:-1])
            if v != pv and pv != 0.0]
_h_gaps = sorted({b - a for a, b in zip(_h_flips[:-1], _h_flips[1:])})
_h_steps = int(round(0.5 / cpu_h.control_dt))
ok_e10 = (ok_e10 and cpu_h._march_half == _h_steps
          and gpu_h._march_half == _h_steps
          and len(_h_flips) >= 4 and _h_gaps == [_h_steps])
print(f"   cadence live: half cycle {cpu_h._march_half} control steps "
      f"(= {cpu_h._march_half * cpu_h.control_dt:.2f} s at "
      f"march_hz {cpu_h.march_hz:.1f}), {len(_h_flips)} c4 flips, "
      f"observed flip gaps {_h_gaps} steps")
ok_e8 = ok_e8 and ok_e10

# -- 2k. torso-pitch hinge penalty (|pitch| past a deadband) ------------------
# day 13: the forward walk is a controlled fall -- torso pitched +12-13.5 deg
# into travel, CoM ~3 cm ahead of the feet (backward walking is upright at
# +1-3 deg, so the plant CAN walk upright). The upright term cannot buy it
# back: it is cos-flat (cos(13 deg) = 0.974 -> 0.013/step at w_upright 0.5)
# and the CoM-over-stance kernel is clearance-gated and never binds during
# the walk cycle (refuted, day 12). w_pitch_hinge is the direct lever:
#     w * max(0, |pitch| - deadband)^2,   pitch = asin(2(qw*qy - qz*qx))
# PITCH ONLY -- roll is untouched (sidestep gaits legitimately roll), and the
# term is off while DOWN in a recovery episode (there the face-plant IS the
# task). Both engines must read the SAME euler convention off the SAME
# quaternion or the CPU referee grades a different lean than MJX trained;
# that is what this block gates, at full airborne strictness.
EXT_P = dict(EXT)
EXT_P.update(cmd_dense=True, w_pitch_hinge=1.0, pitch_deadband_deg=5.0,
             cmd_fixed=(0.35, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
EXT_P0 = dict(EXT_P)
EXT_P0.update(w_pitch_hinge=0.0)          # reference twin: hinge OFF
cpu_p = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT_P)
gpu_p = BimoMJXEnv(xml_path=XML_YAW, domain_rand=False, **EXT_P)
step_p = jax.jit(gpu_p.step)
cpu_p0 = BimoWalkerEnv(xml_path=XML_YAW, actuator_model="sts3215",
                       command_mode=True, domain_rand=False, **EXT_P0)
_splay_p = np.zeros(cpu_p._nq_act)
_splay_p[cpu_p._legL["hip_roll"]] = 0.5
_splay_p[cpu_p._legR["hip_roll"]] = -0.5
_PDB = float(np.radians(5.0))
_P_AMP = float(np.radians(30.0))          # sweep +/-30 deg: in AND out of band


def _tilt(env, ang, axis="pitch"):
    """Torso 1.5 m up, zero velocity, root quaternion set to a PURE rotation
    of `ang` rad about the body x (roll) or y (pitch) axis. Re-posed EVERY
    step: run_block calls act_fn BEFORE sync(), so MJX is handed the identical
    quaternion and the block gates the kernel, not the free-fall drift."""
    env.data.qpos[2] = 1.5
    env.data.qvel[:] = 0.0
    q = np.array([np.cos(0.5 * ang), 0.0, 0.0, 0.0])
    q[1 if axis == "roll" else 2] = np.sin(0.5 * ang)
    env.data.qpos[3:7] = q
    _mj.mj_forward(env.model, env.data)


def _tilt_act(t):
    return (_splay_p + 0.3 * np.sin(0.311 * t
                                    + np.arange(cpu_p._nq_act) * 0.7)
            ).astype(np.float32)


def _pitch_of(env):
    """The probe's convention verbatim (scratchpad/gait_probe.py)."""
    qw, qx, qy, qz = env.data.qpos[3:7]
    return float(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)))


class TiltActs:
    def __init__(self, env, axis):
        self.env, self.axis = env, axis

    def __call__(self, t):
        _tilt(self.env, _P_AMP * np.sin(0.23 * t), self.axis)
        return _tilt_act(t)


ok_e11 = run_block(
    "2k. torso-pitch hinge, +/-30 deg pitch sweep across the 5 deg deadband "
    "(airborne)", 140, TiltActs(cpu_p, "pitch"),
    lambda env: _tilt(env, 0.0, "pitch"),
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_p, gpu_p, step_p))
# 2k-b reports the SAME worst |dqpos|/|dqvel| as 2k, and that is physics, not
# a copy-paste bug: a free-floating multibody system's JOINT dynamics are
# invariant to uniform gravity, and the probe zeroes the base twist every
# step -- so the joint trajectory is identical whichever way the torso is
# tilted (verified: max |q_pitch - q_roll| = 3.5e-17). What differs, and what
# this block gates, is the REWARD path off the root quaternion; the roll
# sweep's "must not fire" claim itself is carried by 2k-c below.
ok_e12 = run_block(
    "2k-b. torso-pitch hinge under a +/-30 deg ROLL sweep (must not fire)",
    140, TiltActs(cpu_p, "roll"), lambda env: _tilt(env, 0.0, "roll"),
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_p, gpu_p, step_p))


def _sweep(env, axis, n=140):
    """Replay the sweep on one CPU env, returning per-step (reward, |pitch|,
    |roll-axis angle|). Deterministic: cmd_fixed pins the command, the pose is
    forced every step, and the action depends only on t -- so the hinge-ON and
    hinge-OFF envs walk the SAME trajectory and their reward difference is
    exactly the hinge term."""
    env.reset(seed=7)
    _tilt(env, 0.0, axis)
    rs, ps = [], []
    for t in range(n):
        _tilt(env, _P_AMP * np.sin(0.23 * t), axis)
        _, r, _, _, _ = env.step(_tilt_act(t))
        rs.append(float(r))
        ps.append(abs(_pitch_of(env)))
    return np.array(rs), np.array(ps)


r_on, p_on = _sweep(cpu_p, "pitch")
r_off, p_off = _sweep(cpu_p0, "pitch")
hinge_obs = r_off - r_on                       # what the reward actually paid
hinge_pred = np.maximum(p_on - _PDB, 0.0) ** 2  # w_pitch_hinge = 1.0
_h_res = float(np.max(np.abs(hinge_obs - hinge_pred)))
_h_peak = float(np.max(hinge_obs))
# INSIDE the deadband the term must be EXACTLY zero -- an always-on |pitch|^2
# penalty (no hinge) would pay here and is rejected by this assertion
_in_band = p_on < _PDB
_h_band = float(np.max(np.abs(hinge_obs[_in_band]))) if _in_band.any() else 1.0
# ROLL must not fire at all: same +/-30 deg sweep about x, zero hinge income
rr_on, _ = _sweep(cpu_p, "roll")
rr_off, _ = _sweep(cpu_p0, "roll")
_h_roll = float(np.max(np.abs(rr_off - rr_on)))
ok_e13 = (_h_res < 1e-12 and _h_peak > 0.05 and _in_band.sum() >= 5
          and _h_band == 0.0 and _h_roll == 0.0
          and float(np.max(np.abs(p_on - p_off))) < 1e-12)
print(f"== 2k-c. pitch-hinge liveness (hinge ON vs OFF twin): "
      f"{'PASS' if ok_e13 else 'FAIL'} ==")
print(f"   term = reward(off) - reward(on): peak {_h_peak:.4f} at "
      f"|pitch| {np.degrees(p_on.max()):.1f} deg, residual vs "
      f"max(0,|pitch|-5deg)^2 = {_h_res:.1e}")
print(f"   deadband honoured: {int(_in_band.sum())} steps inside 5 deg pay "
      f"{_h_band:.1e}; +/-30 deg ROLL sweep pays {_h_roll:.1e}")
ok_e11 = ok_e11 and ok_e12 and ok_e13

ok_robot = robot_blocks()
ok_all = (ok1 and ok2 and ok3 and ok_e1 and ok_e2 and ok_e3 and ok_e4
          and ok_e5 and ok_e6 and ok_e7 and ok_e8 and ok_e11 and ok_robot)
print("\nPARITY:", "PASS" if ok_all else "FAIL")
sys.exit(0 if ok_all else 1)
