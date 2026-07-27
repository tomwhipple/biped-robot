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

Run:  .venv/bin/python sim/mjx/parity_test.py
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
    # 10-DOF hip-yaw -> [7:17]/[6:16]) so the gate works on either plant
    jq, jv = cpu_e._jqpos, cpu_e._jqvel
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
        return (_splay_y + 0.3 * np.sin(0.35 * t + np.arange(_n_act_y) * 0.7)
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

ok_all = (ok1 and ok2 and ok3 and ok_e1 and ok_e2 and ok_e3 and ok_e4
          and ok_e5)
print("\nPARITY:", "PASS" if ok_all else "FAIL")
sys.exit(0 if ok_all else 1)
