"""Parity test: MJX env vs CPU BimoWalkerEnv, on the robot's plant.

Every block runs on sim/bimo_biped_v6ar.xml under the robot preset
(robot_plant.robot_env_kwargs): the 12 leg joints as the policy, the neck and
arms HELD, Plan B per-servo stiffness, hip flexion -120, the sole-level
imitation reference, tick quantization.

What the test encodes:
  * Physics arithmetic is exact between engines: synced single steps match to
    ~1e-11 in float64, including floor contacts, the STS3215/STS3250 torque
    envelopes, latency and backlash.
  * The one semantic difference is the contact-manifold generator: MJX's
    plane_convex/box colliders emit only points within a 1 mm skin of the
    deepest penetration (mjx/_src/collision_convex.py), where CPU MuJoCo
    emits every penetrating corner. Manifolds therefore differ only in
    tilted-deep-penetration transients (impacts) and foot-on-foot clipping;
    in settled stance they are identical. Impacts resolve slightly
    differently and trajectories diverge chaotically there, as under any
    1e-7 perturbation. Mitigation: DR, and every trained policy is refereed
    by the CPU harness (eval_policy/eval_commands) before being believed.
  * Airborne blocks run both solvers to convergence (ARITH_TOL): at the
    plant's 1e-8 the C and MJX Newton solvers stop at different iterates on
    the held joints' frictionloss rows. Training and the referee keep the
    plant's tolerance.

Gates (float64, so precision is not the limiter):
  R1  airborne arithmetic, moving dense-progress command, layout check
      (17 servos, 12 policy, 5 held, Plan B profile, held qpos0)
  R2  grounded stance on the 16 sole pads (matched manifolds)
  R3  airborne arithmetic on a generated 17-servo plant, arms held at rest
  R4  R1 with the IK swing-lift reference (mimic_lift)
  R5  ext_cmd one-leg command (lift -1, foot target, crouch 0.9), airborne
  R6  the same command in grounded stance (lift_ok pattern)
  R7  get-up objective: recovery-reward arithmetic, contact-free
  R8  ext_cmd recovery (recover_mix, staged-rise reference, up-velocity),
      contact- and limit-free, both recovered phases
  R9  terrain-mosaic ground lookup + knee-weighted mimic, airborne
  R10 tick quantization liveness + per-episode crouch draw, airborne
  R11 knee-high march command path (c4 alternation, c6 knee target), airborne
  R12 knee-high clearance kernel on a posed one-leg march stance
  R13 pinned march cadence (march_hz)
  R14 torso-pitch hinge: pitch sweep, roll sweep, hinge ON/OFF liveness
  S   gait-amplitude contact-manifold statistics (informational, no gate)

Run (from sim/mjx, JAX_PLATFORMS=cpu):
      python parity_test.py               # every block
      python parity_test.py --robot-only  # R1-R4 only
"""
import os
import sys
import tempfile

import jax
jax.config.update("jax_enable_x64", True)

import jax.numpy as jp
import mujoco as _mj
import numpy as np
from mujoco import mjx

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import robot_plant  # noqa: E402
from walker_env import BimoWalkerEnv  # noqa: E402
from env_mjx import BimoMJXEnv  # noqa: E402

AIR_GATES = dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5)
STANCE_GATES = dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3)
ARITH_TOL = 1e-10     # solver tolerance for the airborne (arithmetic) blocks


def sync(state, cpu, gpu_env):
    # np.array copies everywhere: jp.asarray can alias numpy buffers
    # zero-copy on the CPU backend, and the CPU env mutates _prev_action
    # and _air_time IN PLACE during step -- an aliased array silently reads
    # post-step values inside the later jit call.
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
        swing_peak=jp.asarray(np.array(cpu._swing_peak)),
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
    cc = (state.data._impl.contact if hasattr(state.data, "_impl")
          else state.data.contact)
    return int(np.sum(np.asarray(cc.dist) < 0))


def run_block(name, n_steps, act_fn, prep_cpu, gates, envs,
              matched_only=False, min_frac=0.8):
    cpu_e, gpu_e, step_e = envs
    # every SERVO's qpos/qvel, held ones included
    jq, jv = cpu_e._sqpos, cpu_e._sqvel
    obs_c, _ = cpu_e.reset(seed=0)
    prep_cpu(cpu_e)
    state = gpu_e.reset(jax.random.PRNGKey(0))
    worst = dict(qpos=0.0, qvel=0.0, reward=0.0, obs=0.0)
    used = skipped = 0
    for t in range(n_steps):
        a = act_fn(t)
        state = sync(state, cpu_e, gpu_e)
        # matched_only screens on the SYNCED (pre-step) manifold: the one the
        # step's contact forces are computed from. A post-step screen misses
        # steps where the engines disagree pre-step and re-converge post-step.
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


def hoist(env):
    """Torso 1.5 m up, zero velocity; re-hoisted every 20 steps so the robot
    never reaches the floor."""
    env.data.qpos[2] = 1.5
    env.data.qvel[:] = 0.0
    _mj.mj_forward(env.model, env.data)


def rob_kw(xml=None, **over):
    """The ext_cmd policy env on the robot preset; `over` replaces keys."""
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


def envs_for(kw, solver_tol=None, cpu_kw=None, gpu_kw=None):
    """(CPU env, MJX env, jitted MJX step) on the same kwargs."""
    c = BimoWalkerEnv(actuator_model="sts3215", command_mode=True,
                      domain_rand=False, **kw, **(cpu_kw or {}))
    g = BimoMJXEnv(domain_rand=False, **kw, **(gpu_kw or {}))
    if solver_tol is not None:
        c.model.opt.tolerance = solver_tol
        g.mj_model.opt.tolerance = solver_tol
        g.model = g._put_model(g.mj_model)
    return c, g, jax.jit(g.step)


def splay(env):
    """Hip-roll offsets that keep the feet apart in flight (sole-sole contact
    is the manifold-divergent case the airborne blocks exclude)."""
    s = np.zeros(env._nq_act)
    s[env._legL["hip_roll"]] = 0.5
    s[env._legR["hip_roll"]] = -0.5
    return s


def air_acts(env, rate=0.353, prep=hoist, hook=None):
    """Splayed sinusoids, the CPU env re-posed by `prep` every 20 steps (both
    engines are re-synced right after). The rate only steers the probe off
    servo-envelope knife edges, where XLA-vs-numpy op order can flip a
    branch at machine epsilon."""
    n = env._nq_act
    s = splay(env)

    def act(t):
        if t % 20 == 0:
            prep(env)
        if hook is not None:
            hook(env)
        return (s + 0.3 * np.sin(rate * t + np.arange(n) * 0.7)
                ).astype(np.float32)
    return act


# -- R1-R4: the robot preset itself ---------------------------------------------
# R1 airborne arithmetic under a moving command (the mimic reference's hip-roll
# oscillation drives the ankle-roll term), R2 grounded stance on the 16 sole
# pads, R3 R1's gate on a generated 17-servo plant with both arms held at the
# rest pose, R4 R1's gate with the IK swing-lift reference (mimic_lift).
def robot_blocks():
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

    c1, g1, s1 = envs_for(rob_kw(), solver_tol=ARITH_TOL)
    lay1 = layout_ok(c1, g1, 17, 5)
    okr1 = run_block(
        "R1. airborne arithmetic (12 policy + 5 held, Plan B kp/kd, "
        "sole-level mimic, quantized, dense cmd 0.3,-0.1,0.2 crouch 0.85)",
        100, air_acts(c1), hoist, AIR_GATES, (c1, g1, s1)) and lay1
    print(f"   layout: {c1._n_servo} servos, {c1._nq_act} policy joints, held "
          f"{c1.held_joints}, x4 on "
          f"{[n for n, p in zip(c1._servo_names, c1._kp_prof) if p != 1]}"
          f" -> {'OK' if lay1 else 'WRONG'}")

    c4, g4, s4 = envs_for(rob_kw(mimic_lift=0.05), solver_tol=ARITH_TOL)
    okr4 = run_block(
        "R4. airborne arithmetic, IK swing-lift reference (mimic_lift 5 cm)",
        100, air_acts(c4), hoist, AIR_GATES, (c4, g4, s4))

    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0)))
    n2 = c2._nq_act
    okr2 = run_block(
        "R2. grounded stance (16 sole pads, matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))
                        ).astype(np.float32),
        lambda env: None, STANCE_GATES, (c2, g2, s2),
        matched_only=True, min_frac=0.5)

    with tempfile.TemporaryDirectory() as td:
        xml17 = robot_plant.write_arms_test_plant(
            os.path.join(td, "arms17.xml"))
        c3, g3, s3 = envs_for(rob_kw(xml17), solver_tol=ARITH_TOL)
        lay3 = layout_ok(c3, g3, 17, 5)
        okr3 = run_block(
            "R3. 17-servo plant (arms held at the folded rest pose) "
            "airborne arithmetic", 100, air_acts(c3), hoist, AIR_GATES,
            (c3, g3, s3)) and lay3
        # the held arms actually HOLD: after the block the shoulders and
        # elbows sit at their rest targets (within the backlash band + sag)
        _mj.mj_forward(c3.model, c3.data)
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
    return okr1 and okr2 and okr3 and okr4


# -- R5/R6: ext_cmd one-leg command -----------------------------------------------
# R5 airborne gates the kernel arithmetic (2D velocity, height-vs-crouch,
# swing-foot target) under a fixed one-leg command carrying a foot offset; R6
# gates the lift contact-pattern term (lift_ok) and the crouch-scaled fall
# floor in stance.
def ext_cmd_blocks():
    kw = rob_kw(cmd_dense=False,
                cmd_fixed=(0.0, 0.0, 0.0, 0.9, -1.0, 0.01, 0.02))
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    ok5 = run_block(
        "R5. ext_cmd airborne arithmetic (lift -1, foot target, crouch 0.9)",
        100, air_acts(c), hoist, AIR_GATES, (c, g, s))
    c, g, s = envs_for(kw)
    n = c._nq_act
    ok6 = run_block(
        "R6. ext_cmd grounded stance (lift_ok pattern, matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n))
                        ).astype(np.float32),
        lambda env: None, STANCE_GATES, (c, g, s),
        matched_only=True, min_frac=0.5)
    return ok5 and ok6


# -- R7: the get-up objective ---------------------------------------------------
# Recovery reward has no contact terms, so its arithmetic is gated
# contact-free (heap dynamics divergence is the manifold caveat).
def getup_block():
    kw = dict(robot_plant.robot_env_kwargs())
    kw.update(getup=True, supply_voltage=11.1, w_energy=0.002,
              w_action_rate=0.15, w_power=0.008, w_pitch_rate=0.1,
              w_upright=0.0, alive_bonus=0.0, imu_obs=False)
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    sp, n = splay(c), c._nq_act
    c.reset(seed=3)
    st = g.reset(jax.random.PRNGKey(3))
    worst_r = worst_o = 0.0
    used = 0
    for t in range(80):
        if t % 20 == 0:
            hoist(c)
        a = (sp + 0.3 * np.sin(0.3 * t + np.arange(n))).astype(np.float32)
        st = sync(st, c, g)
        obs_c, r_c, term_c, _, _ = c.step(a)
        st = s(st, jp.asarray(a))
        if term_c or float(st.done) != 0.0:
            print("== R7. get-up: an env terminated -- must never happen ==")
            return False
        if c.data.ncon != 0:
            continue
        used += 1
        worst_r = max(worst_r, abs(float(st.reward) - r_c))
        worst_o = max(worst_o, float(np.max(np.abs(np.asarray(st.obs) - obs_c))))
    ok = used >= 40 and worst_r < 1e-5 and worst_o < 1e-5
    print(f"== R7. get-up recovery-reward arithmetic (contact-free): "
          f"{'PASS' if ok else 'FAIL'} ({used}/80 airborne steps) ==")
    print(f"   worst |dreward| = {worst_r:.2e}  worst |dobs| = {worst_o:.2e}")
    return ok


# -- R8: ext_cmd recovery from fallen -------------------------------------------
# The recovery-primary reward and the recovered-flag/termination gating that
# recover_mix adds to ext mode, with the staged-rise reference and the
# up-velocity incentive. Contact-free AND limit-free: active joint-limit rows
# are solver territory, the same divergence class as contact manifolds.
def recovery_block():
    kw = rob_kw(recover_mix=1.0, w_rise_dofvel=0.002, w_rise_ref=2.0,
                w_up_vel=2.0)
    kw.pop("cmd_fixed")
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    sp, n = splay(c), c._nq_act
    c.reset(seed=5)
    st = g.reset(jax.random.PRNGKey(5))
    worst_r = worst_o = 0.0
    used = 0
    saw_down = saw_up = False
    for t in range(80):
        if t % 20 == 0:
            # mid-range joints, tilted torso at height: exercises the
            # height/up_gate/standing branches without riding joint limits
            ang = 0.3 + 0.5 * (t / 80.0) * np.pi
            c.data.qpos[:] = c.model.qpos0
            c.data.qpos[2] = 1.5
            c.data.qpos[3:7] = [np.cos(ang / 2), 0.0, np.sin(ang / 2), 0.0]
            c.data.qvel[:] = 0.0
            _mj.mj_forward(c.model, c.data)
        # first half graded "down" (recovery primary), second half with the
        # flag set; re-forced each step because a hoisted torso trivially
        # passes the stand test
        c._recovered = (t >= 40)
        pre_rec = c._recovered
        a = (sp + 0.3 * np.sin(0.3 * t + np.arange(n))).astype(np.float32)
        st = sync(st, c, g)
        obs_c, r_c, _, _, _ = c.step(a)
        st = s(st, jp.asarray(a))
        saw_down |= not pre_rec
        saw_up |= pre_rec
        n_lim = int(np.sum(np.asarray(c.data.efc_type[:c.data.nefc])
                           == int(_mj.mjtConstraint.mjCNSTR_LIMIT_JOINT)))
        if c.data.ncon != 0 or _mjx_ncon(st) != 0 or n_lim != 0:
            continue
        used += 1
        worst_r = max(worst_r, abs(float(st.reward) - r_c))
        worst_o = max(worst_o, float(np.max(np.abs(np.asarray(st.obs) - obs_c))))
    ok = (used >= 40 and saw_down and saw_up and worst_r < 1e-5
          and worst_o < 1e-5)
    print(f"== R8. ext_cmd recovery arithmetic (contact- and limit-free): "
          f"{'PASS' if ok else 'FAIL'} ({used}/80 compared, "
          f"down+up phases={saw_down and saw_up}) ==")
    print(f"   worst |dreward| = {worst_r:.2e}  worst |dobs| = {worst_o:.2e}")
    return ok


# -- R9: terrain mosaic + knee-weighted mimic ------------------------------------
# The mosaic ground-height lookup (bilinear on sim/terrain_mosaic.npz) enters
# height/foot_clear/obs, and the mimic kernel carries a per-joint knee weight.
# Airborne over a rough tile so the lookup is live in every step.
def terrain_block():
    def hoist_rough(env):
        """Hoist over a rough tile (x=2.3, y=0.7: nonzero height)."""
        env.data.qpos[0] = 2.3
        env.data.qpos[1] = 0.7
        hoist(env)

    c, g, s = envs_for(rob_kw(mimic_knee_w=4.0), solver_tol=ARITH_TOL,
                       cpu_kw=dict(terrain_mosaic=True),
                       gpu_kw=dict(terrain=True))
    return run_block(
        "R9. terrain-mosaic ground lookup + knee-weighted mimic (airborne)",
        100, air_acts(c, 0.31, hoist_rough), hoist_rough, AIR_GATES,
        (c, g, s))


# -- R10: tick quantization liveness + per-episode crouch draw -------------------
# R1 gates quantizer AGREEMENT (np.rint and jp.round are both round-half-to-
# even on bit-identical inputs; one flipped tick is a 1.5e-3 rad obs step,
# 150x the gate). This block also asserts the quantizer is ENGAGED and gates
# cmd_crouch_range: cmd_fixed pins channel 3 at exactly 1.0, the value the
# per-episode draw substitutes, so the draw is live and sync() carries it.
def quant_block():
    c, g, s = envs_for(rob_kw(cmd_fixed=(0.3, -0.1, 0.2, 1.0, 0.0, 0.0, 0.0),
                              cmd_crouch_range=(0.7, 0.95)),
                       solver_tol=ARITH_TOL)
    ok = run_block(
        "R10. tick quantization (4096/rev obs+targets) + crouch draw "
        "(airborne)", 100, air_acts(c, 0.347), hoist, AIR_GATES, (c, g, s))
    q = c._obs()[:c._nq_act]
    tick = 2 * np.pi / 4096
    res = float(np.max(np.abs(q / tick - np.rint(q / tick))))
    crouch = float(c._cmd[3])
    # residual is float32-cast noise only; an unquantized obs sits anywhere
    # in +/-0.5 ticks
    ok = ok and res < 1e-3 and 0.7 <= crouch <= 0.95
    print(f"   quantizer live: obs off-grid residual {res:.1e} ticks, "
          f"crouch cmd {crouch:.3f} (drawn from 0.70..0.95)")
    return ok


# -- R11-R13: knee-high march ----------------------------------------------------
# The march is a per-step COMMAND PATTERN on the existing 7 channels (vx=vy=
# wz=0, lift channel c4 alternating on the gait clock, c6 raising the swing-
# foot target from lift_height to the plant's knee height) plus one reward
# kernel (w_knee_high: the fraction of the commanded swing height the swing
# sole reaches, on the correct one-foot contact pattern). ext_mix zeroed and
# march_mix 1.0 make every draw a march (traj_on 4); cmd_fixed is popped
# because pinned commands keep mode 0.
def march_blocks():
    kw = rob_kw(w_knee_high=1.0, ext_mix=(0.0,) * 7, march_mix=1.0)
    kw.pop("cmd_fixed")

    # R11: the command evolution and every kernel it feeds through c6
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    seen = {"lift": set(), "c6": 0.0}

    def watch(env):
        seen["lift"].add(float(np.sign(env._cmd[4])))
        seen["c6"] = max(seen["c6"], float(env._cmd[6]))

    ok11 = run_block(
        "R11. knee-high march command path (c4 alternation, c6 knee target, "
        "airborne)", 120, air_acts(c, 0.359, hook=watch), hoist, AIR_GATES,
        (c, g, s))
    # liveness: the march alternated, and c6 reached the knee-high target
    # the plant gives (knee anchor height above the sole, less lift_height)
    alt = {-1.0, 1.0}.issubset(seen["lift"])
    dz_ok = abs(seen["c6"] - c._march_dz) < 1e-3 and c._march_dz > 0
    same = abs(c._march_dz - float(g._march_dz)) < 1e-12
    ok11 = ok11 and float(c._traj_on) == 4.0 and alt and dz_ok and same
    print(f"   march live: traj_on {float(c._traj_on):.0f}, lifts "
          f"{sorted(seen['lift'])}, peak c6 {seen['c6']:.4f} m, "
          f"knee-high target {c.march_clear*100:.1f} cm "
          f"(= {c.march_clear / c.lift_height:.2f}x lift_height)")

    # R12: a real one-leg stance so knee_frac is non-zero (airborne it is
    # trivially 0 on both sides)
    c, g, s = envs_for(kw)

    def pose_march(env, lift):
        """The commanded swing leg's knee and hip flexed, a PARTIAL lift so
        the clearance lands strictly inside (0, knee-high target) and the
        kernel's clip does not mask the division; the stance pads ~0.3 mm
        into the floor. Sphere pads give the analytic single-point contacts
        both engines agree on (matched manifold)."""
        leg = env._legR if lift > 0 else env._legL
        q = env.model.qpos0.copy()
        q[env._jq0 + leg["knee"]] = -0.90
        q[env._jq0 + leg["hip_pitch"]] = -0.90
        env.data.qpos[:] = q
        env.data.qvel[:] = 0.0
        _mj.mj_forward(env.model, env.data)
        stance = env._pad_gids[0 if lift > 0 else 1]
        bottom = min(float(env.data.geom_xpos[k][2])
                     - float(env.model.geom_size[k][0]) for k in stance)
        env.data.qpos[2] -= bottom + 3e-4
        _mj.mj_forward(env.model, env.data)

    n = c._nq_act
    c.reset(seed=11)
    st = g.reset(jax.random.PRNGKey(11))
    worst_r = worst_o = 0.0
    used = skip = 0
    best_kf = 0.0
    interior = None
    for t in range(80):
        pose_march(c, float(c._cmd[4]))
        a = (0.02 * np.sin(0.3 * t + np.arange(n))).astype(np.float32)
        st = sync(st, c, g)
        pre_c, pre_g = c.data.ncon, _mjx_ncon(st)
        obs_c, r_c, term_c, trunc_c, info_c = c.step(a)
        st = s(st, jp.asarray(a))
        kf = float(info_c["knee_frac"])
        best_kf = max(best_kf, kf)
        if 0.05 < kf < 0.95:
            interior = kf
        if pre_c != pre_g:
            skip += 1
            continue
        used += 1
        worst_r = max(worst_r, abs(float(st.reward) - r_c))
        worst_o = max(worst_o, float(np.max(np.abs(np.asarray(st.obs) - obs_c))))
        if trunc_c or term_c:
            c.reset(seed=5000 + t)
    ok12 = (used >= 40 and interior is not None
            and worst_r < 1e-3 and worst_o < 1e-3)
    print(f"== R12. knee-high clearance kernel on a posed one-leg march "
          f"stance: {'PASS' if ok12 else 'FAIL'} ({used} compared, "
          f"{skip} skipped) ==")
    print(f"   worst |dreward| = {worst_r:.2e}  worst |dobs| = {worst_o:.2e}  "
          f"(kernel live: peak knee_frac {best_kf:.2f}, unclipped sample "
          f"{interior if interior is None else round(interior, 3)})")

    # R13: march_hz > 0 replaces the gait clock as the march timebase. The
    # flip is an exact integer compare on the control-step count: a
    # sin(2*pi*f*t) >= 0 test puts the flip on a zero crossing, where float32
    # and float64 land on opposite sides and swap the commanded leg. 120
    # steps = 4.8 flips at 1 Hz.
    c, g, s = envs_for(dict(kw, march_hz=1.0), solver_tol=ARITH_TOL)
    trace = []                      # (step_i, commanded lift) per step

    def log(env):
        trace.append((int(env._step_i), float(env._cmd[4])))

    ok13 = run_block(
        "R13. pinned march cadence march_hz=1.0 (0.5 s/lift, airborne)",
        120, air_acts(c, 0.359, hook=log), hoist, AIR_GATES, (c, g, s))
    # liveness: the c4 flip period is exactly the commanded half cycle
    flips = [k for (k, v), (_, pv) in zip(trace[1:], trace[:-1])
             if v != pv and pv != 0.0]
    gaps = sorted({b - a for a, b in zip(flips[:-1], flips[1:])})
    half = int(round(0.5 / c.control_dt))
    ok13 = (ok13 and c._march_half == half and g._march_half == half
            and len(flips) >= 4 and gaps == [half])
    print(f"   cadence live: half cycle {c._march_half} control steps "
          f"(= {c._march_half * c.control_dt:.2f} s at march_hz "
          f"{c.march_hz:.1f}), {len(flips)} c4 flips, observed flip gaps "
          f"{gaps} steps")
    return ok11 and ok12 and ok13


# -- R14: torso-pitch hinge -----------------------------------------------------
#     w * max(0, |pitch| - deadband)^2,   pitch = asin(2(qw*qy - qz*qx))
# Pitch only (sidestep gaits legitimately roll), off while DOWN in a recovery
# episode. Both engines must read the same euler convention off the same
# quaternion; the pitch and roll sweeps gate that, the ON/OFF twin gates that
# the term is the hinge (zero inside the band, zero under roll).
def pitch_hinge_block():
    deadband = float(np.radians(5.0))
    amp = float(np.radians(30.0))          # +/-30 deg: in AND out of band
    kw = rob_kw(w_pitch_hinge=1.0, pitch_deadband_deg=5.0,
                cmd_fixed=(0.35, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    c_off = BimoWalkerEnv(actuator_model="sts3215", command_mode=True,
                          domain_rand=False, **dict(kw, w_pitch_hinge=0.0))
    c_off.model.opt.tolerance = ARITH_TOL
    sp, n = splay(c), c._nq_act

    def tilt(env, ang, axis):
        """Hoisted, zero velocity, root quaternion a pure rotation of `ang`
        about body x (roll) or y (pitch). Re-posed every step, before sync,
        so the block gates the kernel, not the free-fall drift."""
        env.data.qpos[2] = 1.5
        env.data.qvel[:] = 0.0
        q = np.array([np.cos(0.5 * ang), 0.0, 0.0, 0.0])
        q[1 if axis == "roll" else 2] = np.sin(0.5 * ang)
        env.data.qpos[3:7] = q
        _mj.mj_forward(env.model, env.data)

    def act(t):
        return (sp + 0.3 * np.sin(0.311 * t + np.arange(n) * 0.7)
                ).astype(np.float32)

    def tilt_acts(axis):
        def f(t):
            tilt(c, amp * np.sin(0.23 * t), axis)
            return act(t)
        return f

    def pitch_of(env):
        qw, qx, qy, qz = env.data.qpos[3:7]
        return float(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)))

    ok_p = run_block(
        "R14. torso-pitch hinge, +/-30 deg pitch sweep across the 5 deg "
        "deadband (airborne)", 140, tilt_acts("pitch"),
        lambda env: tilt(env, 0.0, "pitch"), AIR_GATES, (c, g, s))
    # the roll sweep's joint error equals the pitch sweep's: joint dynamics
    # of a free-floating body are invariant to uniform gravity and the base
    # twist is zeroed every step. What differs is the reward path off the
    # root quaternion.
    ok_r = run_block(
        "R14-b. torso-pitch hinge under a +/-30 deg ROLL sweep",
        140, tilt_acts("roll"), lambda env: tilt(env, 0.0, "roll"),
        AIR_GATES, (c, g, s))

    def sweep(env, axis, n_steps=140):
        """Per-step (reward, |pitch|) on one CPU env. Deterministic (pinned
        command, forced pose, action a function of t), so the ON and OFF
        envs walk the same trajectory and differ by exactly the hinge."""
        env.reset(seed=7)
        tilt(env, 0.0, axis)
        rs, ps = [], []
        for t in range(n_steps):
            tilt(env, amp * np.sin(0.23 * t), axis)
            _, r, _, _, _ = env.step(act(t))
            rs.append(float(r))
            ps.append(abs(pitch_of(env)))
        return np.array(rs), np.array(ps)

    r_on, p_on = sweep(c, "pitch")
    r_off, p_off = sweep(c_off, "pitch")
    paid = r_off - r_on
    res = float(np.max(np.abs(paid - np.maximum(p_on - deadband, 0.0) ** 2)))
    peak = float(np.max(paid))
    in_band = p_on < deadband
    band = float(np.max(np.abs(paid[in_band]))) if in_band.any() else 1.0
    rr_on, _ = sweep(c, "roll")
    rr_off, _ = sweep(c_off, "roll")
    roll = float(np.max(np.abs(rr_off - rr_on)))
    ok_c = (res < 1e-12 and peak > 0.05 and in_band.sum() >= 5
            and band == 0.0 and roll == 0.0
            and float(np.max(np.abs(p_on - p_off))) < 1e-12)
    print(f"== R14-c. pitch-hinge liveness (hinge ON vs OFF twin): "
          f"{'PASS' if ok_c else 'FAIL'} ==")
    print(f"   term = reward(off) - reward(on): peak {peak:.4f} at |pitch| "
          f"{np.degrees(p_on.max()):.1f} deg, residual vs "
          f"max(0,|pitch|-5deg)^2 = {res:.1e}")
    print(f"   deadband honoured: {int(in_band.sum())} steps inside 5 deg pay "
          f"{band:.1e}; +/-30 deg ROLL sweep pays {roll:.1e}")
    return ok_p and ok_r and ok_c


# -- S: gait-amplitude manifold statistics (informational, no gate) -------------
def manifold_stats():
    print("== S. gait-amplitude contact-manifold statistics (informational) ==")
    c, g, s = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)))
    n = c._nq_act
    c.reset(seed=7)
    st = g.reset(jax.random.PRNGKey(7))
    matched = total = 0
    for t in range(150):
        a = (0.4 * np.sin(0.35 * t + np.arange(n) * 0.7)).astype(np.float32)
        st = sync(st, c, g)
        _, _, term_c, trunc_c, _ = c.step(a)
        st = s(st, jp.asarray(a))
        if c.data.ncon:
            total += 1
            matched += int(_mjx_ncon(st) == c.data.ncon)
        if term_c or trunc_c:
            c.reset(seed=2000 + t)
    print(f"   contact steps: {total}, same contact-point count: {matched} "
          f"({100.0 * matched / max(total, 1):.0f}%)")
    print("   (mismatches = tilted/deep penetration where MJX's 1 mm-skin")
    print("    manifold pruning drops corners CPU keeps -- impact transients.")
    print("    Settled stance manifolds are identical. Referee: CPU evals.)")


def main():
    if "--robot-only" in sys.argv:
        ok = robot_blocks()
        print("\nPARITY (R1-R4):", "PASS" if ok else "FAIL")
        return ok
    results = [robot_blocks(), ext_cmd_blocks(), getup_block(),
               recovery_block(), terrain_block(), quant_block(),
               march_blocks(), pitch_hinge_block()]
    manifold_stats()
    ok = all(results)
    print("\nPARITY:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
