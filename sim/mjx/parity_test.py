"""Parity test: MJX env vs CPU BimoWalkerEnv, on the robot's plant.

Every block runs on build/v6/sim/bimo_biped_v6ar.xml under the robot preset
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
  R15 roll series spring (roll_series_k): airborne arithmetic + grounded
      stance, the encoder obs reading the servo shaft (link + tau/k)
  R16 roll play beyond the encoder (roll_play_deg): airborne arithmetic +
      grounded stance, the encoder obs reading the servo shaft (link + offset)
  R17 the ungated march lift term (w_march_unload): airborne arithmetic, the term live,
      and paid on a grounded march stand (not gated on the foot being up)
  R20 the two-foot (AND) march term (w_march_both): airborne arithmetic with the
      per-foot latch state carried, the term live once both feet's swings latched
  R21 the foot-off sag (footoff_sag): grounded stance with the per-side preloads and the
      load shares live, the encoder obs reading the servo shaft (link + deflection)
  R23 the operator command's slew and volume (cmd_slew, cmd_volume; sim/cmd_kernel.py): the
      kernels numpy vs jax and the firmware's golden ramp, airborne arithmetic through a
      set_command step, the CPU and MJX volume samplers' distributions
  R24 the supply sag under load (supply_sag_ohm; sim/supply_sag.py): the kernel numpy vs
      jax, airborne arithmetic and grounded stance at the bench's 0.44 ohm, liveness
  R25 the stand band (cmd_stand_band; sim/cmd_kernel.py): airborne arithmetic under slow
      commands inside the legacy band with the volume's band on (walks, not stands: gait
      shaping, reference, coasting clock), through set_command steps down to the zero command
  R26 the volume imitation reference (mimic_volume_ref, #263; walker_env.mimic_lead_out): the
      calibration equal in both envs, the reference CPU vs MJX over the command volume (eight
      headings x speeds, with and without a turn, back + turn), airborne reward parity under a
      diagonal command, liveness
  R27 the turn imitation reference (mimic_turn_ref on mimic_volume_ref, #263;
      walker_env.mimic_turn_out): the calibration equal in both envs, the reference CPU vs MJX
      over the command volume with turns (pivots, walk / back / side + turn, the saturated
      corners), airborne reward parity under a forward + turn command, liveness
  R28 the travel-calibrated volume reference (mimic_volume_cal on mimic_volume_ref, #263;
      walker_env.mimic_travel_cal): the FK travel tables equal in both envs, the reference CPU
      vs MJX over the command volume (with and without mimic_turn_ref; past the tables' ends,
      the saturated corners), airborne reward parity under a diagonal + turn, liveness
  S   gait-amplitude contact-manifold statistics (informational, no gate)

Run (from sim/mjx, JAX_PLATFORMS=cpu):
      python parity_test.py               # every block
      python parity_test.py --robot-only  # R1-R4 only
      python parity_test.py --roll-series # R15 only
      python parity_test.py --roll-play   # R16 only
      python parity_test.py --supply-sag  # R24 only
      python parity_test.py --mimic-volume-ref  # R26 only
      python parity_test.py --mimic-turn-ref    # R27 only
      python parity_test.py --mimic-volume-cal  # R28 only
      python parity_test.py --sole-in     # R19 only
      python parity_test.py --march-unload   # R17 only
      python parity_test.py --march-both     # R20 only
      python parity_test.py --footoff-sag    # R21 only
      python parity_test.py --cmd-slew       # R23 only
      python parity_test.py --stand-band     # R25 only
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
import trim_kernel as _tk  # noqa: E402
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
        march_cur=jp.asarray(np.array(cpu._march_cur)),
        march_last=jp.asarray(np.array(cpu._march_last)),
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
        ser_defl=(jp.asarray(np.array(cpu._ser_defl))
                  if getattr(cpu, "_ser_c", None) is not None else state.ser_defl),
        rp_h=(jp.asarray(np.array(cpu._rp_h))
              if getattr(cpu, "_rp_h", None) is not None else state.rp_h),
        rp_off=(jp.asarray(np.array(cpu._rp_off))
                if getattr(cpu, "_rp_h", None) is not None else state.rp_off),
        fs_amp=(jp.asarray(cpu._fs_part * cpu._fs_sign * cpu._fs_theta[cpu._fs_side])
                if getattr(cpu, "_fs_theta", None) is not None else state.fs_amp),
        fs_defl=(jp.asarray(np.array(cpu._fs_defl))
                 if getattr(cpu, "_fs_theta", None) is not None else state.fs_defl),
        dq_qhist=(jp.asarray(np.array(cpu._dq_qhist))
                  if getattr(cpu, "_dq_qhist", None) is not None else state.dq_qhist),
        dq_win=(jp.asarray(cpu._dq_win, dtype=jp.int32)
                if getattr(cpu, "_dq_qhist", None) is not None else state.dq_win),
        trim=(jp.asarray(np.array([cpu._trim[k] for k in _tk.KEYS], dtype=np.float32))
              if getattr(cpu, "_trim", None) is not None else state.trim),
        cmd_tgt=(jp.asarray(np.array(cpu._cmd_tgt, dtype=np.float32))
                 if getattr(cpu, "_cmd_tgt", None) is not None else state.cmd_tgt),
        sag_r=(jp.asarray(float(cpu._sag_r))
               if getattr(cpu, "_sag_r", None) is not None else state.sag_r),
        sag_s=(jp.asarray(float(cpu._sag_s))
               if getattr(cpu, "_sag_r", None) is not None else state.sag_s),
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
        ncon_before = c.data.ncon
        obs_c, r_c, term_c, _, _ = c.step(a)
        st = s(st, jp.asarray(a))
        if term_c or float(st.done) != 0.0:
            print("== R7. get-up: an env terminated -- must never happen ==")
            return False
        # a step that starts in contact is not contact-free even if it ends clear
        if ncon_before != 0 or c.data.ncon != 0:
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
# The mosaic ground-height lookup (bilinear on build/v6/sim/terrain_mosaic.npz) enters
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
    # R13b: the march reference (mimic_march): w_mimic pays during the march
    # (a lifted command otherwise gates it off) against the lift-and-sway
    # reference, and march_lift sets the c6 peak. Airborne, every reward term
    # incl. the mimic one is compared CPU vs MJX; the twin without mimic_march
    # must differ (the term is live), and c6 must peak at march_lift - lift_height.
    kwm = dict(kw, mimic_lift=0.02, mimic_sway=0.035, mimic_march=0.035,
               march_lift=0.08)
    c, g, s = envs_for(kwm, solver_tol=ARITH_TOL)
    seen = {"c6": 0.0}

    def watch2(env):
        seen["c6"] = max(seen["c6"], float(env._cmd[6]))

    ok13b = run_block(
        "R13b. march reference (mimic_march, march_lift, airborne)", 120,
        air_acts(c, 0.359, hook=watch2), hoist, AIR_GATES, (c, g, s))
    c0, _, _ = envs_for(dict(kwm, mimic_march=0.0), solver_tol=ARITH_TOL)
    c0.reset(seed=3)
    c.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = 0.0
    for _ in range(30):
        r_on += c.step(z)[1]
        r_off += c0.step(z)[1]
    dz = c._march_dz
    ok13b = (ok13b and abs(dz - 0.02) < 1e-9 and abs(float(g._march_dz) - dz) < 1e-12
             and abs(seen["c6"] - dz) < 1e-3 and abs(r_on - r_off) > 1e-3)
    print(f"   march reference live: peak c6 {seen['c6']:.4f} m (= march_lift 0.08 - "
          f"lift_height {c.lift_height:.2f}), 30-step reward with / without "
          f"mimic_march {r_on:.3f} / {r_off:.3f}")
    return ok11 and ok12 and ok13 and ok13b


# -- R17: the ungated march lift term (w_march_unload) -----------------------------
#     r = w * on * 0.5 * (u + h),  u = clip(((c - p_sw) . e) / |e|^2, 0, 1), e = p_st - p_sw,
#     h = clip(foot_clear / (lift_height + c6), 0, 1),  on = march (traj mode 4) and c4 set
# (walker_env.march_unload_term, shared by both engines). Airborne: every reward term
# incl. this one compared CPU vs MJX at arithmetic tolerance; the twin with the weight 0
# must differ (the term is live). Grounded: the swing foot posed ON the floor (contact,
# zero clearance) must still be paid u > 0 -- the term is not gated on the foot being up.
def march_unload_block():
    kw = rob_kw(w_knee_high=1.0, w_march_unload=1.0, ext_mix=(0.0,) * 7, march_mix=1.0)
    kw.pop("cmd_fixed")
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    ok = run_block("R17. ungated march lift term (w_march_unload, airborne)", 120,
                   air_acts(c, 0.359), hoist, AIR_GATES, (c, g, s))
    c0, _, _ = envs_for(dict(kw, w_march_unload=0.0), solver_tol=ARITH_TOL)
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = mu_sum = 0.0
    for _ in range(30):
        _, r1, _, _, info = c.step(z)
        r_on += r1
        r_off += c0.step(z)[1]
        mu_sum += float(info.get("march_unload", 0.0))
    live = abs((r_on - r_off) - mu_sum) < 1e-6 and mu_sum > 1e-3
    # grounded: both feet on the floor in the stand, a march commanded: u > 0, h ~ 0
    cg = BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False, **kw)
    cg.reset(seed=7)
    mus = []
    for _ in range(40):
        lift_cmd = abs(float(cg._cmd[4])) > 0.5     # the command the step's reward sees
        _, _, term, trunc, info = cg.step(z)
        if lift_cmd:
            mus.append(float(info["march_unload"]))
        if term or trunc:
            break
    grounded = len(mus) > 5 and min(mus) > 0.0
    ok = ok and live and grounded
    print(f"   term live: 30-step reward with / without {r_on:.3f} / {r_off:.3f}, "
          f"difference {r_on - r_off:.4f} = summed term {mu_sum:.4f}; grounded march stand: "
          f"{len(mus)} steps, term {min(mus) if mus else float('nan'):.3f}..{max(mus) if mus else float('nan'):.3f}")
    return ok


# -- R20: the two-foot (AND) march term (w_march_both) ----------------------------
#     per foot: cur = running max of 0.5 (u + h) over its commanded swing, latched into
#     last when the command moves to the other foot; r = w * on * min(last_L, last_R)
# (walker_env.march_both_update, shared by both engines). Airborne: every reward term
# compared CPU vs MJX at arithmetic tolerance over 120 steps (> 3 march half cycles, so both
# latches fire); the twin with the weight 0 must differ (the term is live).
def march_both_block():
    kw = rob_kw(w_knee_high=1.0, w_march_both=1.0, ext_mix=(0.0,) * 7, march_mix=1.0)
    kw.pop("cmd_fixed")
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    ok = run_block("R20. two-foot march term (w_march_both, airborne)", 120,
                   air_acts(c, 0.359), hoist, AIR_GATES, (c, g, s))
    c0, _, _ = envs_for(dict(kw, w_march_both=0.0), solver_tol=ARITH_TOL)
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = mb_sum = 0.0
    lasts = []
    for _ in range(120):
        _, r1, t1, tr1, info = c.step(z)
        _, r0, t0, tr0, _ = c0.step(z)
        if t1 or tr1 or t0 or tr0:
            break
        r_on += r1
        r_off += r0
        mb_sum += float(info.get("march_both", 0.0))
        lasts.append(np.array(c._march_last))
    live = abs((r_on - r_off) - mb_sum) < 1e-6 and mb_sum > 1e-3
    both = bool(lasts) and float(np.max(np.min(np.array(lasts), axis=1))) > 0.0
    ok = ok and live and both
    print(f"   term live: {len(lasts)}-step reward with / without {r_on:.3f} / {r_off:.3f}, "
          f"difference {r_on - r_off:.4f} = summed term {mb_sum:.4f}; both feet latched: {both}")
    return ok


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


# -- R15: roll series spring -----------------------------------------------------
# roll_series_k: the hip / ankle roll PD in series with a spring (quasi-static,
# gains x 1/(1 + kp/k)), and the encoders (q obs, quantized; the dq window
# history) reading the shaft = link + tau/k. Fixed k (no draw), so both engines
# carry the same compliance. Airborne arithmetic with the reg-58 dq window on,
# then grounded stance, where the stance rolls carry load; liveness: the obs
# differs from the link by the deflection, and the deflection is non-trivial.
ROLL_SERIES_PARITY = "hip=11.0,ankle=17.75"


def roll_series_block():
    over = dict(roll_series_k=ROLL_SERIES_PARITY, dq_obs_window=2)
    c, g, s = envs_for(rob_kw(**over), solver_tol=ARITH_TOL)
    ok_a = run_block(
        "R15. roll series spring (hip 11, ankle 17.75 N m/rad), airborne "
        "arithmetic, dq window 2", 100, air_acts(c), hoist, AIR_GATES, (c, g, s))
    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), **over))
    n2 = c2._nq_act
    ok_b = run_block(
        "R15-b. roll series spring, grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c2, g2, s2), matched_only=True, min_frac=0.5)
    # liveness on the stance env: the encoder obs is the link + deflection
    roll = [i for i, nm in enumerate(c2._servo_names) if nm.endswith("roll")]
    defl = np.degrees(np.abs(c2._ser_defl[roll]))
    pol = np.asarray(c2._p(np.arange(c2._n_servo)))
    q_obs = c2._obs()[:n2]
    zo = c2._p(c2._zero_off) if c2._zero_off is not None else 0.0
    q_link = c2.data.qpos[c2._jqpos] - zo
    gap = float(np.max(np.abs(q_obs - q_link - c2._p(c2._ser_defl))))
    other = [j for j, i in enumerate(pol) if i not in roll]
    ok_c = (float(defl.max()) > 0.05 and gap < 1.6e-3      # one tick of quantization
            and float(np.max(np.abs(c2._ser_defl[[i for i in range(c2._n_servo)
                                                  if i not in roll]]))) == 0.0
            and len(other) == n2 - 4)
    print(f"== R15-c. series spring liveness: {'PASS' if ok_c else 'FAIL'} ==")
    print(f"   stance roll deflections {np.round(defl, 3).tolist()} deg; |obs q - "
          f"(link + defl)| max {gap:.1e} rad (quantized)")
    return ok_a and ok_b and ok_c


# -- R16: roll play beyond the encoder -------------------------------------------
# roll_play_deg: the hip / ankle roll links float inside +/-play/2 of the servo
# shaft (driven only at the band edges, a contact that pushes only), and the
# encoders (q obs, quantized; the dq window history) read the shaft. Fixed
# widths (no draw), so both engines carry the same play. Airborne arithmetic
# with the reg-58 dq window on, then grounded stance, where the stance rolls
# carry load; liveness: the obs differs from the link by the shaft offset, the
# offset is non-trivial and inside the band, and off the rolls it is zero.
ROLL_PLAY_PARITY = "hip=4,ankle=5"


def roll_play_block():
    over = dict(roll_play_deg=ROLL_PLAY_PARITY, dq_obs_window=2)
    c, g, s = envs_for(rob_kw(**over), solver_tol=ARITH_TOL)
    ok_a = run_block(
        "R16. roll play (hip 4, ankle 5 deg), airborne arithmetic, dq window 2",
        100, air_acts(c), hoist, AIR_GATES, (c, g, s))
    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), **over))
    n2 = c2._nq_act
    ok_b = run_block(
        "R16-b. roll play, grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c2, g2, s2), matched_only=True, min_frac=0.5)
    roll = [i for i, nm in enumerate(c2._servo_names) if nm.endswith("roll")]
    off = np.degrees(np.abs(c2._rp_off[roll]))
    q_obs = c2._obs()[:n2]
    zo = c2._p(c2._zero_off) if c2._zero_off is not None else 0.0
    q_link = c2.data.qpos[c2._jqpos] - zo
    gap = float(np.max(np.abs(q_obs - q_link - c2._p(c2._rp_off))))
    other = [i for i in range(c2._n_servo) if i not in roll]
    ok_c = (float(off.max()) > 0.05 and gap < 1.6e-3      # one tick of quantization
            and np.all(np.abs(c2._rp_off) <= c2._rp_h + 1e-12)
            and float(np.max(np.abs(c2._rp_off[other]))) == 0.0)
    print(f"== R16-c. roll play liveness: {'PASS' if ok_c else 'FAIL'} ==")
    print(f"   stance roll shaft offsets {np.round(off, 3).tolist()} deg (half-widths "
          f"{np.round(np.degrees(c2._rp_h[roll]), 2).tolist()}); |obs q - (link + offset)| "
          f"max {gap:.1e} rad (quantized)")
    return ok_a and ok_b and ok_c


# -- R19: the sole's inner support edge moved toward the ankle-roll axis ---------
# sole_in_mm: every sole pad inboard of the line is moved onto it, in the MjModel,
# before the MJX upload (walker_env.apply_sole_inset, shared), so both engines
# carry the same sole. Grounded stance with the asymmetric calibration (L 8 mm
# inboard, R 12 mm OUTBOARD of the axis); liveness: the inner pads sit on the
# lines in both models, the outer pads where the plant put them.
SOLE_IN_PARITY = "L=8,R=-12"


def sole_inset_block():
    over = dict(sole_in_mm=SOLE_IN_PARITY)
    c, g, s = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), **over))
    n = c._nq_act
    ok_b = run_block(
        "R19. sole inset (L 8, R -12 mm), grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c, g, s), matched_only=True, min_frac=0.5)
    import mujoco
    from walker_env import sole_pad_nominal
    ys = {}
    for g_, y in sole_pad_nominal(c.model).items():
        nm = mujoco.mj_id2name(c.model, mujoco.mjtObj.mjOBJ_GEOM, g_)
        ys.setdefault(nm[0], []).append(y)
    same = bool(np.array_equal(np.asarray(c.model.geom_pos), np.asarray(g.mj_model.geom_pos)))
    ok_c = (same and abs(min(ys["L"]) + 0.008) < 1e-9 and abs(max(ys["R"]) + 0.012) < 1e-9
            and max(ys["L"]) > 0.04 and min(ys["R"]) < -0.05)
    print(f"== R19-c. sole inset liveness: {'PASS' if ok_c else 'FAIL'} ==")
    print(f"   L pads y {np.round(1e3 * np.array(sorted(ys['L'])), 1).tolist()} mm, R "
          f"{np.round(1e3 * np.array(sorted(ys['R'])), 1).tolist()} mm; CPU == MJX model pads: {same}")
    return ok_b and ok_c


# -- R18: the firmware stance trim on the gait clock (sim/trim_kernel.py) ---------
def stance_trim_block():
    over = dict(stance_trim=True, stance_trim_T="5.2,3.4")
    c, g, s = envs_for(rob_kw(**over), solver_tol=ARITH_TOL)
    seen = {"modes": set(), "hike": 0.0, "dT": 0.0}

    def watch(env):
        if env._trim is not None:
            seen["modes"].add(int(env._trim["mode"]))
            seen["hike"] = max(seen["hike"], abs(float(env._trim["hike"])))
            seen["dT"] = max(seen["dT"], abs(float(env._trim["T_L"]) - np.radians(5.2)),
                             abs(float(env._trim["T_R"]) - np.radians(3.4)))
    ok_a = run_block(
        "R18. stance trim on the gait clock (walking command), airborne arithmetic",
        150, air_acts(c, 0.347, hook=watch), hoist, AIR_GATES, (c, g, s))
    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), **over))
    n2 = c2._nq_act
    ok_b = run_block(
        "R18-b. stance trim, grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c2, g2, s2), matched_only=True, min_frac=0.5)
    ok_c = {5, 6, 7} <= seen["modes"] and seen["hike"] > np.radians(0.5) and seen["dT"] > 1e-4
    print(f"== R18-c. stance trim liveness: {'PASS' if ok_c else 'FAIL'} ==")
    print(f"   trim states seen {sorted(seen['modes'])}, peak |hike| {np.degrees(seen['hike']):.2f} deg, "
          f"learned T moved {np.degrees(seen['dT']):.2f} deg")
    return ok_a and ok_b and ok_c


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


# -- R21: the foot-off sag (footoff_sag) ------------------------------------------
# preloaded springs in series with the four roll servos beyond the encoder; each joint's
# rest angle is shifted by its side's preload x that foot's share of the floor load (the
# pads' penetration, walker_env.footoff_load_share / footoff_sag_shift, shared). Grounded
# stance (both feet loaded, the shares and the shifts live) with the 2026-11-16 fit;
# liveness: the roll joints' shaft - link deflection is non-zero, the encoder obs reads
# link + deflection, and no other joint carries one.
FOOTOFF_SAG_PARITY = "L=6.4,R=2.9"


def footoff_sag_block():
    over = dict(footoff_sag=FOOTOFF_SAG_PARITY)
    c, g, s = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), **over))
    n = c._nq_act
    ok_b = run_block(
        "R21. foot-off sag (L 6.4, R 2.9 deg), grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c, g, s), matched_only=True, min_frac=0.5)
    roll = [i for i, nm in enumerate(c._servo_names) if nm.endswith("roll")]
    other = [i for i in range(c._n_servo) if i not in roll]
    defl = np.degrees(np.abs(c._fs_defl[roll]))
    q_obs = c._obs()[:n]
    zo = c._p(c._zero_off) if c._zero_off is not None else 0.0
    q_link = c.data.qpos[c._jqpos] - zo
    gap = float(np.max(np.abs(q_obs - q_link - c._p(c._fs_defl))))
    share = c._footoff_share()
    ok_c = (float(defl.max()) > 0.05 and gap < 1.6e-3      # one tick of quantization
            and float(np.max(np.abs(c._fs_defl[other]))) == 0.0
            and abs(float(share.sum()) - 1.0) < 1e-9)
    print(f"== R21-c. foot-off sag liveness: {'PASS' if ok_c else 'FAIL'} ==")
    print(f"   roll shaft - link {np.round(defl, 3).tolist()} deg; load shares L/R "
          f"{np.round(share, 3).tolist()}; |obs q - (link + deflection)| max {gap:.1e} rad (quantized)")
    return ok_b and ok_c


# -- R22: the double-support dwell (mimic_dwell) ------------------------------------
#     walker_env.dwell_swing (numpy) and BimoMJXEnv._dwell_swing (JAX) on a phase grid, and the
#     full imitation reference built on them (walking and march), compared point by point; the
#     dwell's shape (feet down in the band, full shift held through the swing); then airborne
#     reward parity for the walk and the march with the dwell on, and liveness (the twin without
#     the dwell must pay differently).
def dwell_block():
    from walker_env import dwell_swing
    D = 0.3
    kw = rob_kw(mimic_lift=0.02, mimic_sway=0.075, mimic_dwell=D)
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    a = 0.5 * np.pi * D
    grid = np.linspace(-np.pi, np.pi, 1441)[:-1] + 1e-4
    worst_sw = worst_q = worst_qm = 0.0
    cmd = np.array([0.15, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    for ph in grid:
        l, r, y = dwell_swing(ph, D)
        swj, yj = g._dwell_swing(jp.float32(ph))
        worst_sw = max(worst_sw, abs(l - float(swj[0])), abs(r - float(swj[1])), abs(y - float(yj)))
        qc = c._mimic_ref(cmd, ph, 0.5)
        qg = np.asarray(g._mimic_ref(jp.asarray(cmd, jp.float32), jp.float32(ph), jp.float32(0.5)))
        worst_q = max(worst_q, float(np.max(np.abs(qc - qg))))
    shape = []
    for ph, want in ((0.5 * a, (0.0, 0.0)), (-0.5 * a, (0.0, 0.0)), (np.pi / 2, (1.0, 0.0)),
                     (-np.pi / 2, (0.0, 1.0)), (np.pi - 0.5 * a, (0.0, 0.0))):
        l, r, _ = dwell_swing(ph, D)
        shape.append(abs(l - want[0]) < 1e-9 and abs(r - want[1]) < 1e-9)
    # full shift held from the middle of the band through the swing, crossing at the zero
    shape.append(abs(dwell_swing(0.75 * a, D)[2] - 1.0) < 1e-9)
    shape.append(abs(dwell_swing(-0.75 * a, D)[2] + 1.0) < 1e-9)
    shape.append(abs(dwell_swing(np.pi / 2, D)[2] - 1.0) < 1e-9)
    shape.append(abs(dwell_swing(1e-6, D)[2]) < 1e-3)
    # the reference: in the band no lift (hip pitch / knee at the stride-only values)
    c0, _, _ = envs_for(dict(kw, mimic_dwell=0.0), solver_tol=ARITH_TOL)
    q_band = c._mimic_ref(cmd, 0.5 * a, 0.5)
    q_mid = c._mimic_ref(cmd, np.pi / 2, 0.5)
    q_mid0 = c0._mimic_ref(cmd, np.pi / 2, 0.5)
    kL = c._i_knee[0] if hasattr(c._i_knee, "__len__") else c._i_knee
    shape.append(abs(q_band[kL] - c._default[kL]) < 1e-9)
    shape.append(abs(q_mid[kL] - q_mid0[kL]) < 1e-9)          # mid-swing lift unchanged
    ok_a = worst_sw < 1e-5 and worst_q < 1e-5 and all(shape)
    print(f"== R22-a. dwell profile + reference, CPU vs MJX on {len(grid)} phases: "
          f"{'PASS' if ok_a else 'FAIL'} ==")
    print(f"   worst |dswing,y| {worst_sw:.1e}  worst |dq_ref| {worst_q:.1e} rad; shape checks "
          f"{sum(shape)}/{len(shape)}")
    ok_b = run_block("R22-b. walking with the dwell reference (mimic_dwell 0.3, sway 0.075, "
                     "feet-phase targets, airborne)", 120, air_acts(c, 0.359), hoist,
                     AIR_GATES, (c, g, s))
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = 0.0
    for _ in range(40):
        r_on += c.step(z)[1]
        r_off += c0.step(z)[1]
    live = abs(r_on - r_off) > 1e-3
    print(f"   dwell live: 40-step reward with / without {r_on:.3f} / {r_off:.3f}")
    kwm = rob_kw(w_knee_high=1.0, ext_mix=(0.0,) * 7, march_mix=1.0, mimic_lift=0.02,
                 mimic_sway=0.075, mimic_march=0.035, march_lift=0.08, mimic_dwell=D)
    kwm.pop("cmd_fixed")
    cm, gm, sm = envs_for(kwm, solver_tol=ARITH_TOL)
    for ph in grid[::12]:
        qc = cm._mimic_ref(np.zeros(7), ph, 0.5, march=True)
        qg = np.asarray(gm._mimic_ref(jp.zeros(7, jp.float32), jp.float32(ph), jp.float32(0.5),
                                      march=jp.bool_(True)))
        worst_qm = max(worst_qm, float(np.max(np.abs(qc - qg))))
    ok_c = run_block("R22-c. march with the dwell reference (airborne)", 120,
                     air_acts(cm, 0.359), hoist, AIR_GATES, (cm, gm, sm)) and worst_qm < 1e-5
    print(f"   march reference worst |dq_ref| {worst_qm:.1e} rad")
    return ok_a and ok_b and live and ok_c


# -- R23: the operator command's slew and volume (sim/cmd_kernel.py; #266, #263) ---------
#     a: the kernels themselves, numpy vs jax.numpy on the same inputs (slew_step on a grid of
#        current / target / step, volume_cmd on the same uniforms), and the GOLDEN trajectory the
#        firmware's host test pins, replayed through jax;
#     b: airborne reward / obs parity with cmd_slew on: the episode starts from the stand and ramps
#        to the pinned command, then set_command() steps the target mid-block (a stick step) and
#        both engines ramp it; liveness: the CPU's c0..c2 follow cmd_kernel exactly;
#     c: the samplers (cmd_volume 1.0, every draw a walk): the CPU env's and the MJX env's draws
#        share one law (cmd_kernel.volume_cmd) on different RNG streams, so their distributions
#        agree: per-channel quantiles, the octants (every diagonal, back + turn, side + turn), all
#        inside the envelope, speeds down to zero.
SLEW_PARITY = (0.5, 0.4, 1.5)


def cmd_slew_block():
    import cmd_kernel as ck
    rng = np.random.default_rng(23)
    cur = rng.uniform(-1, 1, (4000, 3)).astype(np.float32)
    tgt = np.where(rng.uniform(size=(4000, 3)) < 0.2, cur,
                   rng.uniform(-1, 1, (4000, 3))).astype(np.float32)
    dl = ck.slew_delta(SLEW_PARITY, 0.02)
    a_np = ck.slew_step(cur, tgt, dl)
    a_jp = np.asarray(jax.jit(lambda c, t: ck.slew_step(c, t, jp.asarray(dl), jp))(
        jp.asarray(cur), jp.asarray(tgt)))
    u = rng.uniform(size=(4, 4000)).astype(np.float32)
    axes = (0.24, 0.4, 0.25, 1.0)
    v_np = np.stack(ck.volume_cmd(*u, axes))
    v_jp = np.asarray(jp.stack(ck.volume_cmd(*(jp.asarray(x) for x in u), axes, jp)))
    gold = ck.golden()
    st_c = jp.zeros(3, jp.float32)
    st_t = jp.zeros(3, jp.float32)
    steps = dict(ck.GOLDEN_STEPS)
    worst_g = 0.0
    for k in range(gold.shape[0]):
        if k in steps:
            st_t = jp.asarray(steps[k], jp.float32)
        st_c = ck.slew_step(st_c, st_t, jp.asarray(ck.slew_delta(ck.GOLDEN_RATES, ck.GOLDEN_DT)), jp)
        worst_g = max(worst_g, float(np.max(np.abs(np.asarray(st_c) - gold[k]))))
    worst_s = float(np.max(np.abs(a_np - a_jp)))
    worst_v = float(np.max(np.abs(v_np - v_jp)))
    # the law: never more than one step per tick, lands exactly on the target
    step_ok = (np.all(np.abs(a_np - cur) <= dl + 1e-6)
               and np.all((np.abs(tgt - cur) > dl) | (a_np == tgt)))
    ok_a = worst_s < 1e-6 and worst_v < 1e-5 and worst_g == 0.0 and bool(step_ok)
    print(f"== R23-a. command kernels, numpy vs jax: {'PASS' if ok_a else 'FAIL'} ==")
    print(f"   slew worst {worst_s:.1e}, volume worst {worst_v:.1e}, golden replay worst "
          f"{worst_g:.1e} (bit-exact wanted), law checks {'OK' if step_ok else 'WRONG'}")

    kw = rob_kw(cmd_slew=SLEW_PARITY)
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    trace = []
    base = air_acts(c, 0.351)

    def act(t):
        if t == 60:
            c.set_command(-0.2, 0.15, -0.6, 0.85)     # a stick step mid-walk
        a = base(t)
        trace.append((t, np.array(c._cmd[:3]), np.array(c._cmd_tgt)))
        return a
    ok_b = run_block("R23-b. command slew 0.5,0.4,1.5 (start from the stand, a set_command "
                     "step at t=60), airborne arithmetic", 120, act, hoist, AIR_GATES, (c, g, s))
    # the CPU's c0..c2 against cmd_kernel replayed on the same targets
    ref = np.zeros(3)
    dl2 = ck.slew_delta(SLEW_PARITY, c.control_dt).astype(float)
    worst_l, ramped = 0.0, 0
    for (t, c3, t3), nxt in zip(trace[:-1], trace[1:]):
        ref = ck.slew_step(c3, t3, dl2)
        worst_l = max(worst_l, float(np.max(np.abs(ref - nxt[1]))))
        ramped += int(np.any((nxt[1] != nxt[2])))
    first = trace[0][1]
    ok_live = (worst_l < 1e-12 and ramped >= 20 and np.allclose(trace[-1][1], [-0.2, 0.15, -0.6])
               and np.max(np.abs(first)) <= np.max(dl2) + 1e-12)
    print(f"== R23-b2. slew liveness: {'PASS' if ok_live else 'FAIL'} ==")
    print(f"   start {np.round(first, 3).tolist()}, {ramped} ramping ticks, CPU vs kernel worst "
          f"{worst_l:.1e}, end {np.round(trace[-1][1], 3).tolist()}")

    kwv = rob_kw(ext_mix=(0.0,) * 7, cmd_volume=1.0)
    kwv.pop("cmd_fixed")
    cv = BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False, **kwv)
    gv = BimoMJXEnv(domain_rand=False, **kwv)
    cv.reset(seed=5)
    n = 3000
    cpu_d = []
    for _ in range(n):
        cv._sample_command_ext()
        cpu_d.append(np.array(cv._cmd[:3]))
    cpu_d = np.asarray(cpu_d)
    keys = jax.random.split(jax.random.PRNGKey(5), n)
    mjx_d = np.asarray(jax.jit(jax.vmap(
        lambda k: gv._sample_cmd_ext(k, jp.zeros((), jp.int32))[0][:3]))(keys))
    fwd, back, lat, w = ck.volume_axes(cv.cmd_v_range, cv.cmd_back_range, cv.cmd_vy_range,
                                       cv.cmd_w_range)

    def stats(d):
        ax = np.where(d[:, 0] >= 0, fwd, back)
        rho = np.hypot(d[:, 0] / ax, d[:, 1] / lat)
        octs = {(int(np.sign(x)), int(np.sign(y)), int(np.sign(z)))
                for x, y, z in d if abs(x) > 0.02 and abs(y) > 0.02}
        return dict(q=np.quantile(d, [0.1, 0.5, 0.9], axis=0), rho_max=float(rho.max()),
                    slow=float(np.mean(rho < 0.1)), turn=float(np.mean(d[:, 2] != 0.0)),
                    octs=octs, wmax=float(np.abs(d[:, 2]).max()))
    sc, sg = stats(cpu_d), stats(mjx_d)
    dq = float(np.max(np.abs(sc["q"] - sg["q"]) / np.array([fwd, lat, w])))
    ok_c = (dq < 0.08 and sc["rho_max"] <= 1 + 1e-6 and sg["rho_max"] <= 1 + 1e-6
            and sc["wmax"] <= w + 1e-6 and sg["wmax"] <= w + 1e-6
            and min(sc["slow"], sg["slow"]) > 0.05
            and abs(sc["turn"] - sg["turn"]) < 0.05 and abs(sc["turn"] - ck.VOLUME_P_TURN) < 0.05
            and len(sc["octs"]) == len(sg["octs"]) == 12)
    print(f"== R23-c. volume sampler, CPU vs MJX distributions ({n} draws each): "
          f"{'PASS' if ok_c else 'FAIL'} ==")
    print(f"   worst quantile gap {dq:.3f} of the axis; reach CPU {sc['rho_max']:.3f} / MJX "
          f"{sg['rho_max']:.3f}; under 10% of reach {sc['slow']:.3f} / {sg['slow']:.3f}; turning "
          f"{sc['turn']:.3f} / {sg['turn']:.3f}; sign octants (vx, vy, wz incl. 0) "
          f"{len(sc['octs'])} / {len(sg['octs'])}")
    return ok_a and ok_b and ok_live and ok_c


# -- R24: the supply sag under load (sim/supply_sag.py; #6) ----------------------------------
#     a: the kernel, numpy vs jax.numpy on the same inputs (bus current, the relaxed scale);
#     b: airborne arithmetic with supply_sag_ohm 0.44 (the 2026-11-03 bench lead resistance): the
#        scale is carried across ticks (sync copies the CPU's), both engines sag the envelope by it;
#     c: grounded stance (matched manifolds), where the stance servos carry the body;
#     d: liveness: the CPU's supply sags below 1 under the airborne flail and in stance, and with
#        the option off nothing is drawn or carried (empty state fields).
SAG_PARITY = 0.44


def supply_sag_block():
    import supply_sag as sg
    rng = np.random.default_rng(24)
    tau = rng.uniform(-3, 3, (2000, 17))
    qd = rng.uniform(-8, 8, (2000, 17))
    sc = rng.uniform(0.5, 1.0, 2000)
    el = sg.servo_elec(list(sg.CB.NAME_TO_ID))      # the bus map's 17: two STS3250s, 15 STS3215s
    el_j = tuple(jp.asarray(x) for x in el)
    s_np, i_np = (np.array(x) for x in zip(*(sg.sag_step(t, q, s_, 0.44, 11.1, el)
                                             for t, q, s_ in zip(tau, qd, sc))))
    s_jp, i_jp = jax.jit(jax.vmap(lambda t, q, s_: sg.sag_step(t, q, s_, 0.44, 11.1, el_j, jp)))(
        jp.asarray(tau), jp.asarray(qd), jp.asarray(sc))
    worst_k = max(float(np.max(np.abs(s_np - np.asarray(s_jp)))),
                  float(np.max(np.abs(i_np - np.asarray(i_jp)))))
    ok_a = worst_k < 1e-12 and bool(np.all((s_np >= sg.SAG_FLOOR) & (s_np <= 1.0)))
    print(f"== R24-a. supply sag kernel, numpy vs jax: {'PASS' if ok_a else 'FAIL'} ==")
    print(f"   worst {worst_k:.1e}; bus current {i_np.min():.2f}..{i_np.max():.2f} A")

    c, g, s = envs_for(rob_kw(supply_sag_ohm=SAG_PARITY), solver_tol=ARITH_TOL)
    scales = []
    ok_b = run_block("R24-b. supply sag 0.44 ohm, airborne arithmetic", 100,
                     air_acts(c, hook=lambda env: scales.append(env._sag_s)), hoist, AIR_GATES,
                     (c, g, s))
    i_k = float(c._sag_i)
    c2, g2, s2 = envs_for(rob_kw(cmd_fixed=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
                                 supply_sag_ohm=SAG_PARITY))
    n2 = c2._nq_act
    ok_c = run_block(
        "R24-c. supply sag 0.44 ohm, grounded stance (matched manifolds)",
        100, lambda t: (0.05 * np.sin(0.25 * t + np.arange(n2))).astype(np.float32),
        lambda env: None, STANCE_GATES, (c2, g2, s2), matched_only=True, min_frac=0.5)
    off_c, off_g, _ = envs_for(rob_kw())
    st = off_g.reset(jax.random.PRNGKey(0))
    off_c.reset(seed=0)
    ok_d = (min(scales) < 0.995 and c._sag_r == SAG_PARITY and 0.0 < i_k
            and off_c._sag_r is None and st.sag_r.shape == (0,) and st.sag_s.shape == (0,)
            and c2._sag_s < 1.0)
    print(f"== R24-d. supply sag liveness: {'PASS' if ok_d else 'FAIL'} ==")
    print(f"   airborne supply scale min {min(scales):.4f} (V {11.1 * min(scales):.2f}); last bus "
          f"current {i_k:.2f} A; stance supply {11.1 * c2._sag_s:.3f} V at {c2._sag_i:.2f} A; off: "
          f"no draw, empty state")
    return ok_a and ok_b and ok_c and ok_d


# -- R25: the stand band (sim/cmd_kernel.py "The STAND BAND"; #263) ---------------------------------
#     a: airborne reward / obs parity with the volume's band (0.0005) and the night recipe's coasting
#        clock: slow commands inside the legacy 0.05 band -- a slow diagonal, a slow pivot -- are walks
#        (gait shaping, the stepping reference, the clock runs), then the zero command is the stand;
#     b: liveness: the same block under the legacy band grades those commands as stands (the CPU's
#        moving gate differs, and so does its reward).
def stand_band_block():
    import cmd_kernel as ck
    sched = {40: (0.0, 0.0, 0.004, 1.0), 80: (0.0, 0.0, 0.0, 1.0)}
    slow = (0.03, 0.02, 0.0, 1.0, 0.0, 0.0, 0.0)

    def driver(c):
        base = air_acts(c, 0.352)
        trace = []

        def act(t):
            if t in sched:
                c.set_command(*sched[t])
            trace.append(c._cmd_is_moving(c._cmd))
            return base(t)
        return act, trace

    def kw(band):
        return rob_kw(cmd_stand_band=band, cmd_fixed=slow, clock_stand_coast=1.5708)

    envs = envs_for(kw(ck.STAND_BAND_VOLUME), solver_tol=ARITH_TOL)
    act, moving = driver(envs[0])
    ok_a = run_block("R25-a. stand band 0.0005 (slow diagonal, slow pivot, then the stand; coasting "
                     "clock), airborne arithmetic", 120, act, hoist, AIR_GATES, envs)
    ok_live_a = all(moving[1:80]) and not any(moving[81:])
    # the legacy band on the same block (CPU): every one of those commands is a stand
    rew = {}
    for band in (ck.STAND_BAND, ck.STAND_BAND_VOLUME):
        cb = BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=False, **kw(band))
        actb, trb = driver(cb)
        cb.reset(seed=0)
        hoist(cb)
        rew[band] = [cb.step(actb(t))[1] for t in range(60)]
        if band == ck.STAND_BAND:
            ok_live_b = not any(trb)
    ok_live = ok_live_a and ok_live_b and rew[ck.STAND_BAND] != rew[ck.STAND_BAND_VOLUME]
    print(f"== R25-b. stand band liveness: {'PASS' if ok_live else 'FAIL'} ==")
    print(f"   band 0.0005: walk ticks {sum(moving[1:80])}/79 before the stand, "
          f"{sum(moving[81:])} after; band 0.05: walk ticks 0 expected; rewards differ "
          f"{rew[ck.STAND_BAND] != rew[ck.STAND_BAND_VOLUME]}")
    return ok_a and ok_live
# -- R26: the volume imitation reference (mimic_volume_ref) ----------------------------
#     the lead-out signs and the lateral cap computed by both envs on their own model; the full
#     reference CPU vs MJX on a phase grid for commands across the volume (incl. the saturated
#     corners, back + turn, side + turn, slow); airborne reward parity under a diagonal
#     command with a turn; liveness (the twin without the option must pay differently).
def volume_ref_block():
    kw = rob_kw(mimic_lift=0.02, mimic_volume_ref=True,
                cmd_fixed=(0.12, 0.15, 0.6, 1.0, 0.0, 0.0, 0.0))
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    cal = (np.max(np.abs(c._lead_out - np.asarray(g._lead_out))) == 0.0
           and c._vref_bmax == g._vref_bmax)
    grid = np.linspace(-np.pi, np.pi, 181)[:-1] + 1e-4
    cmds = [(0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-0.4, 0.0, 1.0), (0.0, 0.25, -1.0),
            (0.03, -0.02, 0.0), (0.24, 0.25, 0.8)]
    for hd in range(0, 360, 45):
        th = np.radians(hd)
        for v in (0.1, 0.3):
            cmds.append((v * np.cos(th), v * np.sin(th), 0.0))
            cmds.append((v * np.cos(th), v * np.sin(th), -0.7))
    worst = 0.0
    for cm in cmds:
        cmd = np.array([cm[0], cm[1], cm[2], 1.0, 0.0, 0.0, 0.0])
        cj = jp.asarray(cmd, jp.float32)
        for f in (0.7, 1.5):
            for ph in grid:
                qc = c._mimic_ref(cmd, ph, f)
                qg = np.asarray(g._mimic_ref(cj, jp.float32(ph), jp.float32(f)))
                worst = max(worst, float(np.max(np.abs(qc - qg))))
    ok_a = cal and worst < 1e-5
    print(f"== R26-a. volume reference, CPU vs MJX on {len(cmds)} commands x 2 clocks x "
          f"{len(grid)} phases: {'PASS' if ok_a else 'FAIL'} ==")
    print(f"   lead-out signs {c._lead_out.tolist()} cap {c._vref_bmax:.4f} rad (equal: {cal}); "
          f"worst |dq_ref| {worst:.1e} rad")
    ok_b = run_block("R26-b. diagonal + turn with the volume reference (cmd 0.12,0.15,0.6, "
                     "airborne)", 120, air_acts(c, 0.359), hoist, AIR_GATES, (c, g, s))
    c0, _, _ = envs_for(dict(kw, mimic_volume_ref=False), solver_tol=ARITH_TOL)
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = 0.0
    for _ in range(40):
        r_on += c.step(z)[1]
        r_off += c0.step(z)[1]
    live = abs(r_on - r_off) > 1e-3
    print(f"   volume reference live: 40-step reward with / without {r_on:.3f} / {r_off:.3f}")
    return ok_a and ok_b and live


# -- R27: the turn imitation reference (mimic_turn_ref) -------------------------------
#     the hip-yaw signs and the yaw cap computed by both envs on their own model; the full
#     reference CPU vs MJX on a phase grid for turning commands across the volume (pivots both
#     ways, walk / back / side / diagonal + turn, the yaw cap and the lateral budget binding,
#     slow); airborne reward parity under a forward + turn command; liveness (the volume
#     reference alone must pay differently under the same turn).
def turn_ref_block():
    kw = rob_kw(mimic_lift=0.02, mimic_volume_ref=True, mimic_turn_ref=True,
                cmd_fixed=(0.12, 0.05, 0.7, 1.0, 0.0, 0.0, 0.0))
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    cal = (np.max(np.abs(c._yaw_sign - np.asarray(g._yaw_sign))) == 0.0
           and c._vref_ymax == g._vref_ymax and c._vref_bmax == g._vref_bmax)
    grid = np.linspace(-np.pi, np.pi, 181)[:-1] + 1e-4
    cmds = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.3), (0.0, 0.0, -1.0), (0.0, 0.0, 1.0),
            (-0.4, 0.0, 1.0), (0.24, 0.0, -1.0), (0.0, 0.25, -1.0), (0.0, -0.25, 1.0),
            (0.03, -0.02, 0.05), (0.24, 0.25, 0.8), (0.0, 0.0, 0.004)]
    for hd in range(0, 360, 45):
        th = np.radians(hd)
        for v in (0.1, 0.3):
            cmds.append((v * np.cos(th), v * np.sin(th), 0.6))
            cmds.append((v * np.cos(th), v * np.sin(th), -0.9))
    gref = jax.jit(g._mimic_ref)
    worst = 0.0
    for cm in cmds:
        cmd = np.array([cm[0], cm[1], cm[2], 1.0, 0.0, 0.0, 0.0])
        cj = jp.asarray(cmd, jp.float32)
        for f in (0.7, 1.5):
            for ph in grid:
                qc = c._mimic_ref(cmd, ph, f)
                qg = np.asarray(gref(cj, jp.float32(ph), jp.float32(f)))
                worst = max(worst, float(np.max(np.abs(qc - qg))))
    ok_a = cal and worst < 1e-5
    print(f"== R27-a. turn reference, CPU vs MJX on {len(cmds)} commands x 2 clocks x "
          f"{len(grid)} phases: {'PASS' if ok_a else 'FAIL'} ==")
    print(f"   hip-yaw signs {c._yaw_sign.tolist()} cap {c._vref_ymax:.4f} rad (equal: {cal}); "
          f"worst |dq_ref| {worst:.1e} rad")
    ok_b = run_block("R27-b. forward + turn with the turn reference (cmd 0.12,0.05,0.7, "
                     "airborne)", 120, air_acts(c, 0.361), hoist, AIR_GATES, (c, g, s))
    c0, _, _ = envs_for(dict(kw, mimic_turn_ref=False), solver_tol=ARITH_TOL)
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = 0.0
    for _ in range(40):
        r_on += c.step(z)[1]
        r_off += c0.step(z)[1]
    live = abs(r_on - r_off) > 1e-3
    print(f"   turn reference live: 40-step reward with / without {r_on:.3f} / {r_off:.3f}")
    return ok_a and ok_b and live


# -- R28: the travel-calibrated volume reference (mimic_volume_cal) -----------------------
#     the FK travel tables computed by both envs on their own model; the full reference CPU vs
#     MJX on a phase grid for commands across the volume (inside the tables, past their ends,
#     saturated corners, turns), alone and with mimic_turn_ref; airborne reward parity under a
#     diagonal + turn; liveness (the uncalibrated twin must pay differently).
def volume_cal_block():
    kw = rob_kw(mimic_lift=0.02, mimic_volume_ref=True, mimic_volume_cal=True,
                cmd_fixed=(0.12, 0.08, 0.5, 1.0, 0.0, 0.0, 0.0))
    c, g, s = envs_for(kw, solver_tol=ARITH_TOL)
    cal = (np.array_equal(c._cal_ta, np.asarray(g._cal_ta))
           and np.array_equal(c._cal_tb, np.asarray(g._cal_tb))
           and np.array_equal(c._cal_a, np.asarray(g._cal_a))
           and np.array_equal(c._cal_b, np.asarray(g._cal_b))
           and tuple(c._cal_ka) == tuple(g._cal_ka) and c._cal_kb == g._cal_kb)
    ct, gt, _ = envs_for(dict(kw, mimic_turn_ref=True), solver_tol=ARITH_TOL)
    grid = np.linspace(-np.pi, np.pi, 181)[:-1] + 1e-4
    cmds = [(0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (-0.4, 0.0, 1.0), (0.0, 0.25, -1.0),
            (0.03, -0.02, 0.0), (0.24, 0.25, 0.8), (0.29, 0.0, 0.0), (0.0, -0.21, 0.0),
            (0.002, 0.001, 0.0)]
    for hd in range(0, 360, 45):
        th = np.radians(hd)
        for v in (0.1, 0.3):
            cmds.append((v * np.cos(th), v * np.sin(th), 0.0))
            cmds.append((v * np.cos(th), v * np.sin(th), -0.7))
    worst = 0.0
    for ce, ge in ((c, g), (ct, gt)):
        gref = jax.jit(ge._mimic_ref)
        for cm in cmds:
            cmd = np.array([cm[0], cm[1], cm[2], 1.0, 0.0, 0.0, 0.0])
            cj = jp.asarray(cmd, jp.float32)
            for f in (0.7, 1.5):
                for ph in grid:
                    qc = ce._mimic_ref(cmd, ph, f)
                    qg = np.asarray(gref(cj, jp.float32(ph), jp.float32(f)))
                    worst = max(worst, float(np.max(np.abs(qc - qg))))
    ok_a = cal and worst < 1e-5
    print(f"== R28-a. calibrated volume reference, CPU vs MJX on {len(cmds)} commands x 2 "
          f"clocks x {len(grid)} phases, without and with the turn reference: "
          f"{'PASS' if ok_a else 'FAIL'} ==")
    print(f"   travel tables equal: {cal} (end travel per cycle L/R "
          f"{c._cal_ta[0, -1]:.4f}/{c._cal_ta[1, -1]:.4f} m at a = {c._cal_a[-1]:.3f}, lateral "
          f"{c._cal_tb[-1]:.4f} m at b = {c._cal_b[-1]:.4f}); worst |dq_ref| {worst:.1e} rad")
    ok_b = run_block("R28-b. diagonal + turn with the calibrated volume reference (cmd "
                     "0.12,0.08,0.5, airborne)", 120, air_acts(c, 0.357), hoist, AIR_GATES,
                     (c, g, s))
    c0, _, _ = envs_for(dict(kw, mimic_volume_cal=False), solver_tol=ARITH_TOL)
    c.reset(seed=3)
    c0.reset(seed=3)
    z = np.zeros(c._nq_act, np.float32)
    r_on = r_off = 0.0
    for _ in range(40):
        r_on += c.step(z)[1]
        r_off += c0.step(z)[1]
    live = abs(r_on - r_off) > 1e-3
    print(f"   calibration live: 40-step reward with / without {r_on:.3f} / {r_off:.3f}")
    return ok_a and ok_b and live


def main():
    if "--mimic-volume-cal" in sys.argv:
        ok = volume_cal_block()
        print("\nPARITY (R28):", "PASS" if ok else "FAIL")
        return ok
    if "--mimic-turn-ref" in sys.argv:
        ok = turn_ref_block()
        print("\nPARITY (R27):", "PASS" if ok else "FAIL")
        return ok
    if "--mimic-volume-ref" in sys.argv:
        ok = volume_ref_block()
        print("\nPARITY (R26):", "PASS" if ok else "FAIL")
        return ok
    if "--supply-sag" in sys.argv:
        ok = supply_sag_block()
        print("\nPARITY (R24):", "PASS" if ok else "FAIL")
        return ok
    if "--stand-band" in sys.argv:
        ok = stand_band_block()
        print("\nPARITY (R25):", "PASS" if ok else "FAIL")
        return ok
    if "--cmd-slew" in sys.argv:
        ok = cmd_slew_block()
        print("\nPARITY (R23):", "PASS" if ok else "FAIL")
        return ok
    if "--mimic-dwell" in sys.argv:
        ok = dwell_block()
        print("\nPARITY (R22):", "PASS" if ok else "FAIL")
        return ok
    if "--footoff-sag" in sys.argv:
        ok = footoff_sag_block()
        print("\nPARITY (R21):", "PASS" if ok else "FAIL")
        return ok
    if "--roll-series" in sys.argv:
        ok = roll_series_block()
        print("\nPARITY (R15):", "PASS" if ok else "FAIL")
        return ok
    if "--roll-play" in sys.argv:
        ok = roll_play_block()
        print("\nPARITY (R16):", "PASS" if ok else "FAIL")
        return ok
    if "--march-unload" in sys.argv:
        ok = march_unload_block()
        print("\nPARITY (R17):", "PASS" if ok else "FAIL")
        return ok
    if "--march-both" in sys.argv:
        ok = march_both_block()
        print("\nPARITY (R20):", "PASS" if ok else "FAIL")
        return ok
    if "--stance-trim" in sys.argv:
        ok = stance_trim_block()
        print("\nPARITY (R18):", "PASS" if ok else "FAIL")
        return ok
    if "--sole-in" in sys.argv:
        ok = sole_inset_block()
        print("\nPARITY (R19):", "PASS" if ok else "FAIL")
        return ok
    if "--robot-only" in sys.argv:
        ok = robot_blocks()
        print("\nPARITY (R1-R4):", "PASS" if ok else "FAIL")
        return ok
    results = [robot_blocks(), ext_cmd_blocks(), getup_block(),
               recovery_block(), terrain_block(), quant_block(),
               march_blocks(), pitch_hinge_block(), roll_series_block(),
               roll_play_block(), march_unload_block(), stance_trim_block(), sole_inset_block(),
               march_both_block(), footoff_sag_block(), dwell_block(), cmd_slew_block(),
               supply_sag_block(), stand_band_block(), volume_ref_block(), turn_ref_block(),
               volume_cal_block()]
    manifold_stats()
    ok = all(results)
    print("\nPARITY:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
