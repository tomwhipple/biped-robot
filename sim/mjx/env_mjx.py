"""Stage-2b: MJX (JAX) port of BimoWalkerEnv for GPU-scale training.

Ports the COMMAND-CONDITIONED configuration of sim/walker_env.py -- the one
the day-5 verdict says needs 100M+ steps -- as a pure-JAX functional env:

  * STS3215 actuator model: per-substep PD clamped to the DC-motor
    torque-speed envelope, gear-backlash deadzone, sub-step action latency.
  * imu_obs realizable observations (linvel/height zeroed, IMU misalignment
    + gyro bias + per-step noise under DR).
  * command tracking rewards (exp kernels sigma=0.5, optional cmd_dense),
    energy / action-rate / electrical-power penalties, gait shaping
    (feet air-time, single/double support, lateral, pitch-rate).

Deliberate divergences from the CPU env (documented, not accidental):
  * Flat ground only for now (no heightfield terrain port).
  * Dash objective not ported (superseded by command mode).
  * Mass/inertia/friction/payload DR is BATCH-level (per parallel env via a
    vmapped model, brax randomization_fn style) instead of per-episode --
    the standard MJX pattern; with thousands of envs the diversity per
    gradient batch is far higher than the CPU setup ever had. Servo gains,
    latency, backlash, IMU error, and pushes remain per-episode draws.
  * CAD mesh geoms carry no contacts (contype 0). This matches the physics
    the CPU env actually exercises: the pad-true sole boxes are the contact
    geometry; meshes only ever collided with the floor as a safety net.

Random streams do NOT match the CPU env (different RNG); physics and reward
arithmetic DO -- verified by sim/mjx/parity_test.py before any training.
"""
from __future__ import annotations

import os
import re
from typing import Any, NamedTuple

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from mujoco import mjx

_HERE = os.path.dirname(os.path.abspath(__file__))
_XML = os.path.join(_HERE, "..", "bimo_biped_v2.xml")


def _actuated_slices(m):
    """(qpos_lo, qpos_hi, qvel_lo, qvel_hi) spanning the actuated hinge joints,
    derived from the model's actuator transmission targets (free joint
    excluded). The joints are a contiguous chain after the freejoint, so
    min..max+1 covers exactly them -- works for the 8-DOF plant AND the
    10-DOF hip-yaw plant with no literals."""
    jnt = m.actuator_trnid[:, 0]
    qadr = m.jnt_qposadr[jnt]
    vadr = m.jnt_dofadr[jnt]
    return (int(qadr.min()), int(qadr.max()) + 1,
            int(vadr.min()), int(vadr.max()) + 1)

_STS_STALL_12V = 2.94                        # N*m  (30 kg*cm @12V)
_STS_NOLOAD_12V = float(np.deg2rad(60.0) / 0.222)   # 4.712 rad/s @12V
_PAYLOAD_REF = 0.154                         # kg, GoPro MAX incl. battery
_K_CU = 3.75                                 # W/(N*m)^2, ST3215 stall calib.


class State(NamedTuple):
    """Env state as a pytree. All leaves are jnp arrays (batchable)."""
    data: Any                 # mjx.Data
    obs: jax.Array
    reward: jax.Array
    done: jax.Array
    rng: jax.Array
    prev_action: jax.Array    # (n_act,)
    last_target: jax.Array    # (n_act,) target in force before this control step
    step_i: jax.Array         # ()
    air_time: jax.Array       # (2,) per-foot swing clocks
    cmd: jax.Array            # (2,) commanded (vx m/s, yaw rate rad/s), or (7,)
                              # in ext_cmd mode (vx, vy, wz, crouch, lift, fx, fz)
    cmd_next: jax.Array       # ()  step index of the next command resample
    traj: jax.Array           # (3,) swing-foot circle draw: radius, omega, phase0
    traj_on: jax.Array        # ()  1.0 while the current command evolves c5/c6
    last_air: jax.Array       # (2,) last completed swing duration per foot
                              # (gait-symmetry penalty compares L vs R)
    gait_freq: jax.Array      # ()  per-episode gait-clock frequency (Hz)
    gait_phase: jax.Array     # ()  running clock phase in [-pi, pi)
    obs_hist: jax.Array       # (hist-1, frame) previous obs frames (newest
                              # first); zeros-shaped (0, frame) when hist=1
    recover_slot: jax.Array   # ()  1.0 = this env slot starts episodes fallen
    recovered: jax.Array      # ()  0.0 while down (no fall termination); flips
                              # to 1.0 at the first achieved stand
    best_h: jax.Array         # ()  episode-best torso height: the recovery
                              # ratchet pays only for NEW height above this
                              # (static height income was a do-nothing optimum)
    head_ref: jax.Array       # ()  integrated commanded heading (rad): under-
                              # turning accumulates facing error the policy
                              # must pay back (rate kernels forgave it)
    servo: jax.Array          # (4,) kp, kd, stall, no-load speed (per-episode)
    lat_ms: jax.Array         # ()  per-episode sub-step latency
    lash: jax.Array           # ()  per-episode backlash (rad)
    imu_R: jax.Array          # (3,3) mounting misalignment
    imu_bias: jax.Array       # (3,) gyro bias
    metrics: dict[str, jax.Array]


def _quat_yaw(q):
    """Torso yaw from a wxyz quaternion (identical arithmetic to the CPU
    referee's -- the heading integrator depends on it matching)."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return jp.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _prep_model(xml_path: str, payload: bool, servo_joint_damping: float,
                mesh_floor: bool = False,
                payload_cg_z: float = 0.08) -> mujoco.MjModel:
    """Load the v2 MJCF patched for MJX: absolute meshdir, optional welded
    payload body, MJCF position actuators silenced (torque enters via
    qfrc_applied, exactly like the CPU sts3215 path).

    mesh_floor=False strips the cad-mesh floor contacts (sole boxes are the
    true contact geometry when upright -- cheaper). Get-up mode REQUIRES
    mesh_floor=True: a fallen robot rests on its torso/leg hulls, and both
    engines collide meshes via their convex hulls, so keeping the pairs makes
    the engines physically equivalent on the ground too."""
    with open(xml_path) as f:
        xml = f.read()
    xml = re.sub(
        r'meshdir="([^"]+)"',
        lambda m: 'meshdir="' + os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(xml_path)), m.group(1))) + '"',
        xml)
    if not mesh_floor:
        xml = re.sub(
            r'(<default class="cad">\s*<geom[^>]*?)contype="\d+" conaffinity="\d+"',
            r'\1contype="0" conaffinity="0"', xml)
    if payload:
        body = (f'<body name="payload" pos="0 0 {payload_cg_z}">'
                f'<geom name="payload" type="box" size="0.0125 0.032 0.0345" '
                f'mass="{_PAYLOAD_REF}" contype="0" conaffinity="0" '
                f'rgba="0.12 0.12 0.14 1"/></body>')
        xml, n = re.subn(r'(<site name="imu"[^>]*/>)',
                         lambda m: m.group(1) + body, xml, count=1)
        if n != 1:
            raise ValueError("expected exactly one imu site for the payload")
    m = mujoco.MjModel.from_xml_string(xml)
    m.actuator_gainprm[:] = 0.0
    m.actuator_biasprm[:] = 0.0
    _, _, jv0, jv1 = _actuated_slices(m)
    m.dof_damping[jv0:jv1] = servo_joint_damping
    return m


class BimoMJXEnv:
    """Functional command-mode env. reset(rng) -> State, step(State, a) -> State.
    Instances hold only static config + the mjx model, so methods close over
    them and jit/vmap cleanly. Episode truncation and auto-reset are the
    trainer's job (brax wrappers); `done` here means FELL."""

    def __init__(
        self,
        xml_path: str = _XML,
        control_hz: float = 50.0,
        episode_seconds: float = 10.0,
        # -- reward shaping (defaults mirror walker_env.py) ------------------
        w_upright: float = 0.5,
        alive_bonus: float = 0.1,
        w_height: float = 0.0,
        w_energy: float = 0.002,
        w_action_rate: float = 0.05,
        fall_cost: float = 1.0,
        w_feet_air: float = 0.0,
        air_time_target: float = 0.3,
        w_single_support: float = 0.0,
        w_lateral: float = 0.0,
        w_pitch_rate: float = 0.0,
        w_power: float = 0.0,
        # -- actuator ---------------------------------------------------------
        supply_voltage: float = 11.1,
        servo_kp: float = 12.0,
        servo_kd: float = 0.25,
        servo_range: float = 0.15,
        servo_joint_damping: float = 0.1,
        # -- payload (present iff payload_mass > 0 or payload_dr) -------------
        payload_mass: float = 0.0,
        payload_dr: bool = False,   # batch-level mass draw handled in
        # domain_randomize(); this flag only ensures the body exists
        # -- DR (per-episode, state-level) -------------------------------------
        domain_rand: bool = False,
        gain_range: float = 0.2,
        latency_ms: float = 0.0,
        latency_ms_max: float | None = None,
        latency_jitter_ms: float = 0.0,
        backlash_deg: float = 0.0,
        backlash_deg_max: float | None = None,
        push_prob: float | None = None,
        push_force: float | None = None,
        # -- termination --------------------------------------------------------
        fall_height: float = 0.18,
        fall_up_z: float = 0.4,
        # -- commands ------------------------------------------------------------
        cmd_v_range: tuple = (0.3, 1.0),
        cmd_w_range: float = 1.0,
        cmd_stand_prob: float = 0.3,
        cmd_resample_s: tuple = (2.5, 4.5),
        cmd_fixed: tuple | None = None,
        cmd_dense: bool = False,
        w_track_v: float = 2.0,
        w_track_w: float = 1.0,
        # -- precision command mode (2026-07-17) ---------------------------------
        # 7-channel commands: (vx, vy, wz, crouch height frac, foot-lift,
        # swing-foot dx, swing-foot dz). Adds the precision skills: sidestep
        # (vy), backward walk (negative vx), crouch (height tracking), one-leg
        # balance (lift = -1 left / +1 right), and swing-foot trajectory
        # tracking (c5/c6 driven along a per-episode circle while lifted --
        # "make circles in the air"). Off by default: obs stays (38,), old
        # runs bit-exact.
        ext_cmd: bool = False,
        cmd_vy_range: float = 0.25,           # |vy| sidestep command bound (m/s)
        cmd_back_range: tuple = (-0.4, -0.15),  # backward-walk vx draw (m/s)
        crouch_range: tuple = (0.6, 0.9),     # crouch-command height fractions
        lift_height: float = 0.06,            # nominal lifted-foot rise (m)
        w_track_h: float = 0.0,               # height-tracking kernel weight
        w_lift: float = 0.0,                  # correct one-foot contact pattern
        w_track_foot: float = 0.0,            # swing-foot target kernel weight
        foot_sigma: float = 0.06,             # foot kernel width (m). Was
        # 0.03: at precision_v4's ~15 cm foot error a 3 cm kernel pays ~0
        # gradient -- the 2026-07-13 kernel-width lesson, third occurrence.
        # Old runs' config.json recorded their trained value.
        traj_radius: tuple = (0.02, 0.05),    # air-circle radius draw (m)
        traj_period: tuple = (1.5, 3.5),      # air-circle period draw (s)
        ext_mix: tuple = (0.20, 0.08, 0.15, 0.12, 0.10),  # command mix: stand,
        # crouch, balance, air-circle, pivot[, march, sway]; remainder =
        # walk (fwd/back/side). 5-tuples mean no march/sway (legacy).
        # march = alternating leg lifts with a foot-height bob (knee
        # articulation); sway = standing lateral weight-shift tracking an
        # oscillating vy (hip-roll articulation). User request 2026-07-20:
        # "not a lot of motion in the knees or sideways in the hips."
        w_foot_cross: float = 0.0,    # penalty when the soles' torso-frame
        # lateral separation closes below sole width + 5 mm -- the leg
        # meshes don't self-collide, so only the reward keeps the feet from
        # visually occupying the same space (video review 2026-07-20)
        sway_vy: float = 0.12,        # sway-command vy amplitude (m/s)
        walk_submix: tuple = (0.15, 0.15),    # of walk commands: (backward,
        # sidestep) fractions; remainder walks forward with the turning draw
        # -- plan-v2 Phase A terms (2026-07-20, MuJoCo Playground recipes) ------
        # Numbers from the Playground biped envs (T1 / Berkeley Humanoid),
        # adapted to our 0.34 m scale. All default OFF -> old runs bit-exact.
        gait_clock: bool = False,   # per-episode gait clock (freq U(1.25,
        # 1.75) Hz) replaces the episode-fraction phase in the obs; feet are
        # half a cycle apart (left = phase, right = phase + pi)
        w_feet_phase: float = 0.0,  # exp(-sum((foot_z - rz(phase))^2)/s2):
        # periodic swing-height targets -- Playground's strongest gait term
        swing_height: float = 0.06, # rz peak (m; Playground 0.10-0.12 on
        # 0.5-0.8 m robots, scaled to our 0.34 m)
        feet_phase_s2: float = 0.004,   # kernel denominator (their 0.01
        # at swing 0.1 -> 0.004 at swing 0.06)
        w_feet_slip: float = 0.0,   # -w * sum(|foot vel xy| * contact):
        # anti-skating (their -0.25)
        w_orientation: float = 0.0,  # -w * (up_x^2 + up_y^2): quadratic
        # tilt cost (their -1.0)
        w_ang_vel_xy: float = 0.0,  # -w * sum(w_xy^2) (their -0.15)
        w_pose: float = 0.0,        # -w * sum((q - q_default)^2), gated to
        # plain locomotion/stand commands (fights skills otherwise)
        w_dof_limits: float = 0.0,  # -w * soft-limit violation (90% range)
        push_kick: bool = False,    # velocity-kick pushes (qvel += U(kick)
        # in a random direction) instead of force pushes -- Playground style
        kick_range: tuple = (0.1, 1.0),   # m/s
        obs_hist_len: int = 1,      # stacked obs frames (3 = Playground/
        # Open-Duck-style short history; changes obs size!)
        joint_frictionloss: float = 0.0,   # Coulomb friction per leg DOF
        # (N*m). Open Duck's BAM fit for the STS3215: friction_base ~0.05.
        # Genuinely missing from our model (the torque-speed envelope covers
        # viscous/back-EMF losses, NOT stiction). 0 = legacy exact.
        joint_armature: float = 0.0,       # reflected rotor inertia per leg
        # DOF (kg*m^2). BAM fit ~0.028 -- significant against our light legs.
        # -- plan-v2 Phase B: procedural gait imitation (Open Duck pattern) ----
        w_mimic: float = 0.0,       # exp(-sum((q - q_ref)^2)/mimic_s2) toward
        # a joint-space reference gait generated from (vx, vy, wz, gait
        # phase): sinusoidal stride/roll amplitudes scaled by command/freq,
        # knee bend during swing, level ankle. At zero command the reference
        # IS the standing pose, so the term is continuous across commands.
        # Backward/turning references exist BY CONSTRUCTION -- the two
        # locomotion skills reward shaping never cracked. Requires
        # gait_clock. (Proven recipe: Open Duck Mini, same servos.)
        mimic_s2: float = 0.72,     # kernel denominator (~(0.3 rad)^2 * 8)
        w_rise_dofvel: float = 0.0,  # joint-velocity penalty while DOWN in a
        # recovery episode (HumanUP/utra: jerk control during the rise is
        # what makes get-up hardware-deployable)
        w_rise_ref: float = 0.0,    # staged-rise reference kernel while DOWN
        # (getup_v2 lesson: the ratchet fixed the economics -- sitting still
        # now nets -0.7/step -- but PPO exploration NEVER FINDS the rise
        # from a settled start; training converged to the exact sit-still
        # floor. Dense guidance through a scripted joint-space rise is the
        # field-standard fix: tuck heels -> plant feet under pelvis -> deep
        # squat -> stand, piecewise-linear over rise_secs. Time-based phase,
        # like the gait clock.)
        rise_secs: float = 4.0,     # scripted rise duration
        rise_ref_s2: float = 2.0,   # exp kernel denominator (rad^2 summed)
        w_symmetry: float = 0.0,    # gait-symmetry penalty: on touchdown,
        # |this swing duration - the OTHER foot's last swing| (user
        # 2026-07-19: "work on the symmetry of motion in walking gaits";
        # mechanical root cause is the identical-parts yaw bias, this is
        # the software counterweight)
        lift_clear: float = 0.03,   # lifted-foot min clearance (m) above its
        # standing sole height for the lift reward to pay (user 2026-07-18:
        # "at least 3 cm off the ground" -- a 2 mm hover no longer counts)
        w_com_stance: float = 0.0,  # while lifted: CoM-over-stance-foot
        # kernel (user 2026-07-23: raise the foot by BENDING THE KNEE,
        # keeping the CG static -- not by sticking the leg out, which
        # shifts the CG forward). Pays for planted balance; the knee-lift
        # is the cheapest compliant posture.
        com_sigma: float = 0.04,    # CoM-offset kernel width (m)
        w_heading: float = 0.0,     # integrated-heading kernel: cmd yaw rate
        # integrates into a target FACING; under-turning accumulates error
        # (rate kernels forgave chronic half-authority turns -- the
        # square/circle return failure, user 2026-07-23: "work on turning
        # to face a different direction"). Resets to current yaw on
        # command resample; inside the g_skill-gated primary.
        recover_start_mix: tuple = (1.0, 0.0, 0.0),   # recovery-slot start
        # states: (ragdoll, upright kneel, feet-loaded squat) -- the getup
        # reverse curriculum, applied to ext recovery slots (2026-07-19).
        # The CPU referee always grades ragdoll starts (the CLAIM).
        recover_mix: float = 0.0,   # fraction of env slots whose episodes
        # START from a settled ragdoll fall (recovery-from-fallen skill,
        # user 2026-07-18). While down: getup-style recovery reward, command
        # pinned to stand, NO fall termination; after the first achieved
        # stand the episode continues under normal rules. Slots are fixed
        # across auto-resets (brax cached-first-state pattern, like batch DR).
        # -- payload CG above torso center (m). 0.08 = the 2026-07-11 spec
        # (tower top plate then at 319 mm); the current stack (battery-bay
        # tower + imu_carrier) puts the camera CG at 0.0945. Default keeps
        # old runs' plants byte-identical.
        payload_cg_z: float = 0.08,
        # -- observations ---------------------------------------------------------
        imu_obs: bool = True,
        imu_noise: float = 1.0,
        # -- get-up mode (fall recovery) -------------------------------------------
        # Episodes START from a settled ragdoll fall (random orientation +
        # joints, dropped and settled for getup_settle_s); there is NO fall
        # termination -- being down is the task, not the failure. The primary
        # reward swaps to recovery progress: height toward nominal + torso
        # uprightness, plus a standing bonus (upright AND tall AND still)
        # that makes a held stand the absorbing goal. Command channels are
        # pinned to (0,0), so the obs layout stays identical to command mode
        # (a future unified policy can read get-up as "stand, but fallen").
        getup: bool = False,
        getup_settle_s: float = 0.4,   # ragdoll settle time at reset
        w_recover_h: float = 1.0,      # height progress term
        w_recover_up: float = 0.8,     # uprightness term. GATED on height
        # (v3): paying for a vertical torso at pelvis height ~0.1 made the
        # upright KNEEL an absorbing local optimum -- the pike transfer to
        # standing requires the torso to fold DOWN first (reward valley).
        # The term now scales in over pelvis height 0.12->0.20 m.
        stand_bonus: float = 1.0,      # per-step, when upright+tall+still
        getup_start_mix: tuple = (1.0, 0.0, 0.0),   # reset-state mix:
        # (ragdoll fall, upright kneel, feet-loaded squat). Reverse
        # curriculum for v3: seeding episodes near the goal teaches the
        # rise backward from standing; the fall->kneel part is already
        # learned. Referee evals keep (1,0,0) -- the CLAIM is fall recovery.
        # -- see walker_env.py for both (kept arithmetically identical) ---------
        action_map: str = "legacy",    # "full" reaches asymmetric limits
        hip_flex_deg: float | None = None,   # widen hip FLEXION (deg)
    ):
        if getup and ext_cmd:
            raise ValueError("getup and ext_cmd are separate objectives")
        # recovery episodes rest the fallen robot on its CAD hulls -- both
        # engines must collide meshes with the floor (same rule as getup)
        self.mj_model = _prep_model(
            xml_path, payload_mass > 0 or payload_dr, servo_joint_damping,
            mesh_floor=getup or (ext_cmd and recover_mix > 0),
            payload_cg_z=payload_cg_z)
        # actuated-joint layout, derived from the model (NOT literals) so the
        # 8-DOF and 10-DOF hip-yaw plants both work
        (self._jq0, self._jq1, self._jv0,
         self._jv1) = _actuated_slices(self.mj_model)
        self._nq_act = self._jq1 - self._jq0
        if joint_frictionloss > 0:
            self.mj_model.dof_frictionloss[self._jv0:self._jv1] = \
                joint_frictionloss
        if joint_armature > 0:
            self.mj_model.dof_armature[self._jv0:self._jv1] = joint_armature
        if payload_mass > 0:
            self.mj_model.body_mass[self.mj_model.body("payload").id] = payload_mass
        self.model = mjx.put_model(self.mj_model)
        m = self.mj_model

        self.sim_dt = float(m.opt.timestep)
        self.n_substeps = max(1, round((1.0 / control_hz) / self.sim_dt))
        self.control_dt = self.n_substeps * self.sim_dt
        self.max_steps = int(episode_seconds / self.control_dt)

        jnt = m.actuator_trnid[:, 0]
        # name-based index maps (action/slice order == actuator order ==
        # qpos-address order by XML construction). Per-leg role -> action
        # index; hip_yaw is None on the 8-DOF plant, present on v3yaw.
        self._act_names = [m.joint(int(j)).name for j in jnt]
        self._jname2i = {n: i for i, n in enumerate(self._act_names)}
        _roles = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
        self._legL = {r: self._jname2i.get(f"L_{r}") for r in _roles}
        self._legR = {r: self._jname2i.get(f"R_{r}") for r in _roles}
        self._i_roll = jp.array([self._legL["hip_roll"], self._legR["hip_roll"]])
        self._i_pitch = jp.array([self._legL["hip_pitch"],
                                  self._legR["hip_pitch"]])
        self._i_knee = jp.array([self._legL["knee"], self._legR["knee"]])
        self._i_ankle = jp.array([self._legL["ankle"], self._legR["ankle"]])
        if hip_flex_deg is not None:
            for i in (self._jname2i["L_hip_pitch"], self._jname2i["R_hip_pitch"]):
                m.jnt_range[jnt[i], 0] = -np.deg2rad(hip_flex_deg)
            self.model = mjx.put_model(m)      # re-upload patched ranges
        if action_map not in ("legacy", "full"):
            raise ValueError(f"unknown action_map {action_map!r}")
        self.action_map = action_map
        self.hip_flex_deg = hip_flex_deg
        self._lo = jp.asarray(m.jnt_range[jnt, 0])
        self._hi = jp.asarray(m.jnt_range[jnt, 1])
        self._default = jp.asarray(m.qpos0[self._jq0:self._jq1])
        self._scale = 0.5 * (self._hi - self._lo)
        self._qpos0 = jp.asarray(m.qpos0)

        self._torso_bid = m.body("torso").id
        self._sole_gids = (m.geom("L_sole").id, m.geom("R_sole").id)
        self._up_adr = m.sensor("torso_up").adr[0]
        self._nominal_h = float(m.body("torso").pos[2])

        v = supply_voltage / 12.0
        self._nom_servo = jp.array([servo_kp, servo_kd,
                                    _STS_STALL_12V * v, _STS_NOLOAD_12V * v])

        self.w_upright = w_upright
        self.alive_bonus = alive_bonus
        self.w_height = w_height
        self.w_energy = w_energy
        self.w_action_rate = w_action_rate
        self.fall_cost = fall_cost
        self.w_feet_air = w_feet_air
        self.air_time_target = air_time_target
        self.w_single_support = w_single_support
        self.w_lateral = w_lateral
        self.w_pitch_rate = w_pitch_rate
        self.w_power = w_power
        self.domain_rand = domain_rand
        self.gain_range = gain_range
        self.servo_range = servo_range
        self.latency_ms = latency_ms
        self.latency_ms_max = latency_ms_max
        self.latency_jitter_ms = latency_jitter_ms
        self.backlash_rad = float(np.deg2rad(backlash_deg))
        self.backlash_rad_max = (None if backlash_deg_max is None
                                 else float(np.deg2rad(backlash_deg_max)))
        self.push_prob = (0.01 if domain_rand else 0.0) if push_prob is None else push_prob
        self.push_force = (5.0 if domain_rand else 0.0) if push_force is None else push_force
        self.fall_height = fall_height
        self.fall_up_z = fall_up_z
        self.cmd_v_range = cmd_v_range
        self.cmd_w_range = cmd_w_range
        self.cmd_stand_prob = cmd_stand_prob
        self.cmd_resample_s = cmd_resample_s
        self.cmd_fixed = (0.0, 0.0) if getup else cmd_fixed
        self.cmd_dense = cmd_dense
        self.w_track_v = w_track_v
        self.w_track_w = w_track_w
        self.ext_cmd = ext_cmd
        self.cmd_vy_range = cmd_vy_range
        self.cmd_back_range = cmd_back_range
        self.crouch_range = crouch_range
        self.lift_height = lift_height
        self.w_track_h = w_track_h
        self.w_lift = w_lift
        self.w_track_foot = w_track_foot
        self.foot_sigma = foot_sigma
        self.traj_radius = traj_radius
        self.traj_period = traj_period
        self.ext_mix = tuple(ext_mix) + (0.0,) * (7 - len(ext_mix))
        self.walk_submix = walk_submix
        self.w_foot_cross = w_foot_cross
        self.sway_vy = sway_vy
        self.gait_clock = gait_clock
        self.w_feet_phase = w_feet_phase
        self.swing_height = swing_height
        self.feet_phase_s2 = feet_phase_s2
        self.w_feet_slip = w_feet_slip
        self.w_orientation = w_orientation
        self.w_ang_vel_xy = w_ang_vel_xy
        self.w_pose = w_pose
        self.w_dof_limits = w_dof_limits
        self.push_kick = push_kick
        self.kick_range = kick_range
        self.obs_hist_len = max(1, int(obs_hist_len))
        self.w_mimic = w_mimic
        self.mimic_s2 = mimic_s2
        self.w_rise_dofvel = w_rise_dofvel
        self.w_rise_ref = w_rise_ref
        self.rise_secs = rise_secs
        self.rise_ref_s2 = rise_ref_s2
        if w_mimic > 0 and not gait_clock:
            raise ValueError("w_mimic requires gait_clock")
        self.w_symmetry = w_symmetry
        self.lift_clear = lift_clear
        self.w_com_stance = w_com_stance
        self.com_sigma = com_sigma
        self.w_heading = w_heading
        self.recover_mix = recover_mix
        if ext_cmd:
            # ext recovery slots reuse _fallen_data's start-state machinery
            self.getup_start_mix = recover_start_mix
        self.imu_obs = imu_obs
        self.imu_noise = imu_noise
        self.getup = getup
        self.getup_settle = max(1, round(getup_settle_s / self.sim_dt))
        self.w_recover_h = w_recover_h
        self.w_recover_up = w_recover_up
        self.stand_bonus = stand_bonus
        self.getup_start_mix = getup_start_mix

        self.action_size = self._nq_act
        # per-joint blocks (qpos, qvel, prev_action) scale with the joint
        # count; the rest is fixed. 8-DOF: 38 legacy / 43 ext; 10-DOF hip-yaw:
        # 44 legacy / 49 ext. obs history stacks obs_frame x hist (newest first)
        self.obs_frame = (3 * self._nq_act + 3 + 3 + 3 + 1 + 2
                          + (7 if ext_cmd else 2))
        self.obs_size = self.obs_frame * self.obs_hist_len
        # foot BODY ids for slip velocities (cvel linear part)
        self._foot_bids = (m.body("L_foot").id, m.body("R_foot").id)
        # soft joint limits (90% of range) for the dof-limit penalty
        _mid = 0.5 * (np.asarray(m.jnt_range[jnt, 0])
                      + np.asarray(m.jnt_range[jnt, 1]))
        _half = 0.5 * (np.asarray(m.jnt_range[jnt, 1])
                       - np.asarray(m.jnt_range[jnt, 0]))
        self._soft_lo = jp.asarray(_mid - 0.9 * _half)
        self._soft_hi = jp.asarray(_mid + 0.9 * _half)
        # swing-foot reference: standing sole centers relative to the torso
        # (computed once on the CPU model at qpos0 -- yaw 0, so this is already
        # the body frame). The lifted-foot target = this + (dx, 0, lift+dz).
        d0 = mujoco.MjData(m)
        d0.qpos[:] = m.qpos0
        mujoco.mj_forward(m, d0)
        self._foot_rel0 = jp.asarray(np.stack([
            d0.geom_xpos[self._sole_gids[0]] - d0.xpos[self._torso_bid],
            d0.geom_xpos[self._sole_gids[1]] - d0.xpos[self._torso_bid]]),
            dtype=jp.float32)
        # standing sole-center world height: lift clearance is measured
        # against this (flat ground -- terrain is not in the ext curriculum)
        self._sole_z0 = float(d0.geom_xpos[self._sole_gids[0]][2])
        self._metric_keys = self._METRIC_KEYS + (
            ("height_err", "foot_err", "foot_clear", "foot_sep", "lift_ok",
             "vy_body", "recovered", "com_stance", "head_err")
            if ext_cmd else ())

    # -- pieces ---------------------------------------------------------------
    def _sample_cmd(self, rng: jax.Array, step_i: jax.Array):
        """(cmd, cmd_next, traj_on): stand / pivot-in-place / walk mix, exactly
        the CPU distribution (draws are made unconditionally -- fixed RNG
        shape). traj_on flags an ext_cmd air-circle command (c5/c6 evolve)."""
        if self.cmd_fixed is not None:
            cmd = jp.asarray(self.cmd_fixed, dtype=jp.float32)
            want = 7 if self.ext_cmd else 2
            if cmd.shape[0] != want:
                raise ValueError(f"cmd_fixed needs {want} channels")
            return cmd, jp.asarray(10 ** 9, dtype=jp.int32), jp.zeros(())
        if self.ext_cmd:
            return self._sample_cmd_ext(rng, step_i)
        ru, rw, rs, rv, rp, rww, rh = jax.random.split(rng, 7)
        u = jax.random.uniform(ru)
        w_piv = jax.random.uniform(rw, minval=0.3, maxval=self.cmd_w_range)
        w_piv = jp.where(jax.random.uniform(rs) < 0.5, w_piv, -w_piv)
        v = jax.random.uniform(rv, minval=self.cmd_v_range[0],
                               maxval=self.cmd_v_range[1])
        w_walk = jp.where(
            jax.random.uniform(rp) < 0.6,
            jax.random.uniform(rww, minval=-self.cmd_w_range,
                               maxval=self.cmd_w_range), 0.0)
        cmd = jp.where(
            u < self.cmd_stand_prob, jp.zeros(2),
            jp.where(u < self.cmd_stand_prob + 0.15,
                     jp.array([0.0, 1.0]) * w_piv + jp.zeros(2),
                     jp.stack([v, w_walk])))
        hold = jax.random.uniform(rh, minval=self.cmd_resample_s[0],
                                  maxval=self.cmd_resample_s[1])
        return (cmd.astype(jp.float32),
                step_i + (hold / self.control_dt).astype(jp.int32),
                jp.zeros(()))

    def _sample_cmd_ext(self, rng: jax.Array, step_i: jax.Array):
        """7-channel precision command draw. Mix (ext_mix): stand / crouch /
        one-leg balance / air-circle / pivot; remainder walks (15% backward,
        15% pure sidestep, 70% forward with the legacy 60%-turning draw).
        (Single-leg crouch dropped from the mix 2026-07-18, user call.)
        All draws unconditional -- fixed RNG shape for jit."""
        rs = jax.random.split(rng, 13)
        u = jax.random.uniform(rs[0])
        p_stand, p_crouch, p_bal, p_traj, p_piv, p_march, p_sway = self.ext_mix
        t1 = p_stand
        t2 = t1 + p_crouch
        t3 = t2 + p_bal
        t4 = t3 + p_traj
        t5 = t4 + p_piv
        t6 = t5 + p_march
        t7 = t6 + p_sway
        m_crouch = (u >= t1) & (u < t2)
        m_bal = (u >= t2) & (u < t3)
        m_traj = (u >= t3) & (u < t4)
        m_piv = (u >= t4) & (u < t5)
        m_march = (u >= t5) & (u < t6)
        m_sway = (u >= t6) & (u < t7)
        m_walk = u >= t7
        crouch = jax.random.uniform(rs[1], minval=self.crouch_range[0],
                                    maxval=self.crouch_range[1])
        side = jp.where(jax.random.uniform(rs[2]) < 0.5, 1.0, -1.0)
        w_piv = side * jax.random.uniform(rs[3], minval=0.3,
                                          maxval=self.cmd_w_range)
        uw = jax.random.uniform(rs[4])
        vx_f = jax.random.uniform(rs[5], minval=self.cmd_v_range[0],
                                  maxval=self.cmd_v_range[1])
        vx_b = jax.random.uniform(rs[6], minval=self.cmd_back_range[0],
                                  maxval=self.cmd_back_range[1])
        vy_s = side * jax.random.uniform(rs[7], minval=0.1,
                                         maxval=self.cmd_vy_range)
        wz_f = jp.where(
            jax.random.uniform(rs[8]) < 0.6,
            jax.random.uniform(rs[9], minval=-self.cmd_w_range,
                               maxval=self.cmd_w_range), 0.0)
        p_back, p_side = self.walk_submix
        wk_back = uw < p_back
        wk_side = (uw >= p_back) & (uw < p_back + p_side)
        vx = jp.where(m_walk,
                      jp.where(wk_back, vx_b, jp.where(wk_side, 0.0, vx_f)),
                      0.0)
        vy = jp.where(m_walk & wk_side, vy_s, 0.0)
        wz = (jp.where(m_walk & ~(wk_back | wk_side), wz_f, 0.0)
              + jp.where(m_piv, w_piv, 0.0))
        lifted = m_bal | m_traj
        ch = jp.where(m_crouch, crouch, 1.0)
        lift = jp.where(lifted, side, 0.0)
        cmd = jp.stack([vx, vy, wz, ch, lift,
                        jp.zeros(()), jp.zeros(())]).astype(jp.float32)
        hold = jax.random.uniform(rs[12], minval=self.cmd_resample_s[0],
                                  maxval=self.cmd_resample_s[1])
        # trajectory mode code: 0 off, 1 foot-circle, 2 march, 3 sway
        mode = (m_traj.astype(jp.float32) * 1.0
                + m_march.astype(jp.float32) * 2.0
                + m_sway.astype(jp.float32) * 3.0)
        return (cmd, step_i + (hold / self.control_dt).astype(jp.int32),
                mode)

    def _rise_ref(self, t_s):
        """Scripted joint-space rise for recovery episodes (getup_v3): piecewise
        -linear through tuck -> plant -> squat -> stand over rise_secs, then
        holds the standing pose. Time-based phase from episode start (recovery
        episodes begin DOWN at t=0 with the command pinned to stand). Symmetric
        L/R, roll/yaw at default. Mirrored EXACTLY in walker_env.py."""
        d = self._default
        # stage targets (hip_pitch, knee, ankle) inside the joint limits
        # (-1.92..1.05 / -1.66..0.087 / +-0.70)
        stages = jp.array([
            [-1.85, -1.60, 0.60],   # tuck: heels to butt, toes down
            [-1.55, -1.55, 0.30],   # plant: weight rocks onto the feet
            [-1.00, -1.10, 0.15],   # deep squat, torso coming up
        ])
        stand = jp.stack([d[self._i_pitch][0], d[self._i_knee][0],
                          d[self._i_ankle][0]])
        keys = jp.concatenate([stages, stand[None]], axis=0)      # (4,3)
        u = jp.clip(t_s / self.rise_secs, 0.0, 1.0) * 3.0         # 3 segments
        i0 = jp.clip(jp.floor(u).astype(jp.int32), 0, 2)
        w = u - i0.astype(jp.float32)
        tgt = keys[i0] * (1.0 - w) + keys[i0 + 1] * w             # (3,)
        q = (d.at[self._i_pitch].set(tgt[0]).at[self._i_knee].set(tgt[1])
             .at[self._i_ankle].set(tgt[2]))
        return jp.clip(q, self._lo, self._hi)

    def _mimic_ref(self, cmd, phase, freq):
        """Joint-space procedural gait reference (plan-v2 Phase B).
        Stride amplitude ~ KX*v_leg/f (foot excursion ~0.18 m leg * A matches
        v/(4f) per half-cycle at KX=1.4); turning = differential stride via
        v_leg = vx -/+ hip_half_sep*wz; sidestep = lateral roll oscillation;
        knee bends 0.55 rad during its swing half; ankle keeps the sole
        level. At zero command the reference equals the standing pose.
        Mirrored EXACTLY in walker_env.py (parity-gated)."""
        f = jp.maximum(freq, 0.5)
        d = self._default
        s, c = jp.sin(phase), jp.cos(phase)
        sw = jp.stack([jp.clip(s, 0.0, None), jp.clip(-s, 0.0, None)])
        xn = jp.stack([-c, c])                 # foot fore-aft normal (L, R)
        A = jp.clip(1.4 * (cmd[0] + jp.array([-1.0, 1.0]) * 0.028 * cmd[2])
                    / f, -0.45, 0.45)
        B = jp.clip(1.1 * cmd[1] / f, -0.3, 0.3)
        hp0 = d[self._i_pitch]                  # hip-pitch defaults (L, R)
        kn0 = d[self._i_knee]
        hipP = hp0 - A * xn
        # knee swing-bend scales with commanded activity: zero command ->
        # the reference IS the standing pose (no knee pumping)
        mag = jp.minimum(1.0, (jp.max(jp.abs(A)) + jp.abs(B)) / 0.35)
        knee = kn0 - 0.55 * mag * sw
        roll = d[self._i_roll] + B * xn
        ank = d[self._i_ankle] - (hipP - hp0) - (knee - kn0)
        # scatter the per-leg references onto the standing pose; any hip_yaw
        # joints stay at their default (0.0) -> neutral regularization
        q = (d.at[self._i_roll].set(roll).at[self._i_pitch].set(hipP)
             .at[self._i_knee].set(knee).at[self._i_ankle].set(ank))
        return jp.clip(q, self._lo, self._hi)

    def _draw_traj(self, rng: jax.Array) -> jax.Array:
        """Per-episode air-circle parameters (radius, signed omega, phase)."""
        r1, r2, r3, r4 = jax.random.split(rng, 4)
        rad = jax.random.uniform(r1, minval=self.traj_radius[0],
                                 maxval=self.traj_radius[1])
        per = jax.random.uniform(r2, minval=self.traj_period[0],
                                 maxval=self.traj_period[1])
        omega = (2 * jp.pi / per) * jp.where(
            jax.random.uniform(r3) < 0.5, 1.0, -1.0)
        phase0 = jax.random.uniform(r4, maxval=2 * jp.pi)
        return jp.stack([rad, omega, phase0])

    def _foot_contacts(self, data) -> jax.Array:
        """(2,) bool: sole-in-contact flags from the MJX contact set."""
        c = data.contact
        hit = c.dist < 0.0
        out = []
        for gid in self._sole_gids:
            mine = (c.geom[:, 0] == gid) | (c.geom[:, 1] == gid)
            out.append(jp.any(hit & mine))
        return jp.stack(out)

    def _obs(self, data, prev_action, cmd, step_i, imu_R, imu_bias,
             rng: jax.Array, gait_phase=None) -> jax.Array:
        up = data.sensordata[self._up_adr:self._up_adr + 3]
        gyro = data.qvel[3:6]
        linvel = data.qvel[0:3]
        height = data.qpos[2]
        if self.imu_obs:
            noisy = self.domain_rand and self.imu_noise > 0.0
            up = imu_R @ up
            gyro = gyro + imu_bias
            if noisy:
                r1, r2 = jax.random.split(rng)
                up = up + 0.01 * self.imu_noise * jax.random.normal(r1, (3,))
                gyro = gyro + 0.03 * self.imu_noise * jax.random.normal(r2, (3,))
            linvel = jp.zeros(3)
            height = jp.zeros(())
        if self.gait_clock:
            phase = gait_phase        # per-episode gait clock (plan v2)
        else:
            phase = 2 * jp.pi * (step_i.astype(jp.float32) / self.max_steps)
        return jp.concatenate([
            data.qpos[self._jq0:self._jq1],
            data.qvel[self._jv0:self._jv1],
            up,
            linvel,
            gyro,
            prev_action,
            jp.atleast_1d(height),
            jp.stack([jp.sin(phase), jp.cos(phase)]),
            cmd,
        ]).astype(jp.float32)

    # -- per-episode draws (shared by reset and reseed) -------------------------
    def _draw_episode(self, rng: jax.Array):
        r_lat, r_lash, r_servo, r_ang, r_ax, r_bias = jax.random.split(rng, 6)
        if self.latency_ms_max is not None:
            lat_ms = jax.random.uniform(r_lat, minval=self.latency_ms,
                                        maxval=self.latency_ms_max)
        else:
            lat_ms = jp.asarray(self.latency_ms, dtype=jp.float32)
        if self.backlash_rad_max is not None:
            lash = jax.random.uniform(r_lash, minval=self.backlash_rad,
                                      maxval=self.backlash_rad_max)
        else:
            lash = jp.asarray(self.backlash_rad, dtype=jp.float32)
        if self.domain_rand:
            sf = jp.concatenate([
                jax.random.uniform(r_servo, (2,), minval=1 - self.gain_range,
                                   maxval=1 + self.gain_range),
                jax.random.uniform(jax.random.fold_in(r_servo, 1), (2,),
                                   minval=1 - self.servo_range,
                                   maxval=1 + self.servo_range)])
            servo = self._nom_servo * sf
        else:
            servo = self._nom_servo
        if self.imu_obs and self.domain_rand and self.imu_noise > 0.0:
            ang = jax.random.normal(r_ang) * np.deg2rad(2.0) * self.imu_noise
            ax = jax.random.normal(r_ax, (3,))
            ax = ax / (jp.linalg.norm(ax) + 1e-9)
            k = jp.array([[0.0, -ax[2], ax[1]],
                          [ax[2], 0.0, -ax[0]],
                          [-ax[1], ax[0], 0.0]])
            imu_R = jp.eye(3) + jp.sin(ang) * k + (1 - jp.cos(ang)) * (k @ k)
            imu_bias = jax.random.uniform(r_bias, (3,), minval=-0.03,
                                          maxval=0.03) * self.imu_noise
        else:
            imu_R = jp.eye(3)
            imu_bias = jp.zeros(3)
        return servo, lat_ms, lash, imu_R, imu_bias

    _METRIC_KEYS = ("power_w", "vx_body", "wz", "height", "up_z", "fell",
                    "track_v_err", "track_w_err", "standing")

    def _settle(self, qpos, n):
        data = mjx.make_data(self.model)
        data = data.replace(qpos=qpos,
                            ctrl=jp.zeros(self.mj_model.nu) + self._default)

        def fall(d, _):
            return mjx.step(self.model, d), None

        data, _ = jax.lax.scan(fall, data, None, length=n)
        return mjx.forward(self.model, data)

    def _fallen_data(self, r_q):
        """Settled start for get-up mode. Mix (getup_start_mix): ragdoll fall
        (random orientation + joints, dropped from 0.35 m) / upright kneel /
        feet-loaded deep squat -- the latter two are the reverse-curriculum
        seeds (learn the rise backward from near the goal)."""
        r_m, r_o, r_j = jax.random.split(r_q, 3)
        u1, u2, u3 = jax.random.uniform(r_o, (3,))
        quat = jp.array([jp.sqrt(u1) * jp.cos(2 * jp.pi * u3),
                         jp.sqrt(1 - u1) * jp.sin(2 * jp.pi * u2),
                         jp.sqrt(1 - u1) * jp.cos(2 * jp.pi * u2),
                         jp.sqrt(u1) * jp.sin(2 * jp.pi * u3)])
        joints = jp.clip(
            self._default + jax.random.uniform(r_j, (self._nq_act,),
                                               minval=-0.6, maxval=0.6)
            * self._scale, self._lo, self._hi)
        q_rag = (self._qpos0.at[2].set(0.35).at[3:7].set(quat)
                 .at[self._jq0:self._jq1].set(joints))
        mix = tuple(self.getup_start_mix)
        mix = mix + (0.0,) * (4 - len(mix))   # (ragdoll, kneel, squat, sit)
        p_rag, p_kneel, p_squat, p_sit = mix
        if p_rag >= 1.0:
            return self._settle(q_rag, self.getup_settle)
        # kneel: torso vertical, knees folded, shins on the ground
        j_kneel = jp.zeros(self._nq_act).at[self._i_pitch].set(-0.2) \
                             .at[self._i_knee].set(-1.62) \
                             .at[self._i_ankle].set(-0.6)
        j_kneel = jp.clip(j_kneel, self._lo, self._hi)
        q_kneel = self._qpos0.at[2].set(0.13).at[self._jq0:self._jq1].set(
            j_kneel)
        # deep squat, feet flat, torso folded (the study's rise-path start)
        j_squat = jp.zeros(self._nq_act).at[self._i_pitch].set(
                                 self._lo[self._legL["hip_pitch"]] + 0.05) \
                             .at[self._i_knee].set(-1.62) \
                             .at[self._i_ankle].set(0.65)
        j_squat = jp.clip(j_squat, self._lo, self._hi)
        ang = 0.55                     # pitch forward so the soles sit flat
        q_squat = (self._qpos0.at[2].set(0.10)
                   .at[3:7].set(jp.array([jp.cos(ang / 2), 0.0,
                                          jp.sin(ang / 2), 0.0]))
                   .at[self._jq0:self._jq1].set(j_squat))
        # sit (user 2026-07-19/20 spec): TORSO POINTING UP, waist bent 90
        # deg, feet & legs straight out front. Hip roll splays the feet a
        # few cm apart so the (non-self-colliding) leg meshes can never
        # render overlapped -- video review caught them merging. Verified
        # by sim/mjx/check_sit_pose.py (upright, stable, feet separated).
        j_sit = jp.zeros(self._nq_act).at[self._i_pitch].set(-1.57) \
                           .at[self._i_knee].set(-0.09) \
                           .at[jp.array([self._legL["hip_roll"]])].set(0.10) \
                           .at[jp.array([self._legR["hip_roll"]])].set(-0.10)
        j_sit = jp.clip(j_sit, self._lo, self._hi)
        q_sit = (self._qpos0.at[2].set(0.08)
                 .at[self._jq0:self._jq1].set(j_sit))
        d_rag = self._settle(q_rag, self.getup_settle)
        d_kneel = self._settle(q_kneel, 50)
        d_squat = self._settle(q_squat, 50)
        d_sit = self._settle(q_sit, 50)
        u = jax.random.uniform(r_m)
        pick_kneel = (u >= p_rag) & (u < p_rag + p_kneel)
        pick_squat = (u >= p_rag + p_kneel) & (u < p_rag + p_kneel + p_squat)
        pick_sit = u >= p_rag + p_kneel + p_squat

        def sel(a, b, c, s):
            out = jp.where(pick_kneel, b, a)
            out = jp.where(pick_squat, c, out)
            return jp.where(pick_sit, s, out)

        return jax.tree_util.tree_map(sel, d_rag, d_kneel, d_squat, d_sit)

    # -- api -------------------------------------------------------------------
    def reset(self, rng: jax.Array) -> State:
        rng, r_q, r_v, r_ep, r_cmd, r_obs = jax.random.split(rng, 6)
        recover_slot = jp.zeros(())
        if self.getup:
            data = self._fallen_data(r_q)
        elif self.ext_cmd and self.recover_mix > 0:
            # recovery slot draw: this env's episodes start from a settled
            # ragdoll fall (slot membership is FIXED across the trainer's
            # cached-first-state auto-resets -- the batch carries the mix)
            rng, r_mix = jax.random.split(rng)
            recover_slot = (jax.random.uniform(r_mix)
                            < self.recover_mix).astype(jp.float32)
            qpos = self._qpos0.at[self._jq0:self._jq1].add(
                jax.random.uniform(r_q, (self._nq_act,), minval=-0.03,
                                   maxval=0.03))
            qvel = jax.random.uniform(r_v, (self.mj_model.nv,),
                                      minval=-0.02, maxval=0.02)
            d_up = mjx.make_data(self.model)
            d_up = d_up.replace(qpos=qpos, qvel=qvel,
                                ctrl=jp.zeros(self.mj_model.nu) + self._default)
            d_up = mjx.forward(self.model, d_up)
            d_dn = self._fallen_data(jax.random.fold_in(r_q, 1))

            def pick(a, b):
                return jp.where(recover_slot > 0, a, b)

            data = jax.tree_util.tree_map(pick, d_dn, d_up)
        else:
            qpos = self._qpos0.at[self._jq0:self._jq1].add(
                jax.random.uniform(r_q, (self._nq_act,), minval=-0.03,
                                   maxval=0.03))
            qvel = jax.random.uniform(r_v, (self.mj_model.nv,),
                                      minval=-0.02, maxval=0.02)
            data = mjx.make_data(self.model)
            data = data.replace(qpos=qpos, qvel=qvel,
                                ctrl=jp.zeros(self.mj_model.nu) + self._default)
            data = mjx.forward(self.model, data)
        servo, lat_ms, lash, imu_R, imu_bias = self._draw_episode(r_ep)
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next, traj_on = self._sample_cmd(r_cmd, step_i)
        if self.ext_cmd:
            # recovery episodes: command pinned to STAND for the whole
            # episode (get up, then hold; locomotion-after-recovery is a
            # later curriculum stage)
            stand = jp.zeros(7).at[3].set(1.0).astype(jp.float32)
            cmd = jp.where(recover_slot > 0, stand, cmd)
            cmd_next = jp.where(recover_slot > 0,
                                jp.asarray(10 ** 9, dtype=jp.int32), cmd_next)
            traj_on = jp.where(recover_slot > 0, 0.0, traj_on)
        if self.ext_cmd:
            rng, r_traj = jax.random.split(rng)
            traj = self._draw_traj(r_traj)
        else:
            traj = jp.zeros(3)
        if self.gait_clock:
            rng, r_gf, r_gp = jax.random.split(rng, 3)
            gait_freq = jax.random.uniform(r_gf, minval=1.25, maxval=1.75)
            gait_phase = jax.random.uniform(r_gp, minval=-jp.pi, maxval=jp.pi)
        else:
            gait_freq = jp.zeros(())
            gait_phase = jp.zeros(())
        prev_action = jp.zeros(self._nq_act, dtype=jp.float32)
        frame = self._obs(data, prev_action, cmd, step_i, imu_R, imu_bias,
                          r_obs, gait_phase)
        obs_hist = jp.tile(frame, (self.obs_hist_len - 1, 1)) \
            if self.obs_hist_len > 1 else jp.zeros((0, self.obs_frame))
        obs = (jp.concatenate([frame, obs_hist.reshape(-1)])
               if self.obs_hist_len > 1 else frame)
        metrics = {k: jp.zeros(()) for k in self._metric_keys}
        return State(data=data, obs=obs, reward=jp.zeros(()),
                     done=jp.zeros(()), rng=rng, prev_action=prev_action,
                     last_target=self._default.astype(jp.float32),
                     step_i=step_i, air_time=jp.zeros(2),
                     last_air=jp.zeros(2),
                     gait_freq=gait_freq, gait_phase=gait_phase,
                     obs_hist=obs_hist, cmd=cmd,
                     cmd_next=cmd_next, traj=traj, traj_on=traj_on,
                     recover_slot=recover_slot,
                     recovered=1.0 - recover_slot,
                     best_h=data.qpos[2],
                     head_ref=_quat_yaw(data.qpos[3:7]),
                     servo=servo, lat_ms=lat_ms, lash=lash,
                     imu_R=imu_R, imu_bias=imu_bias, metrics=metrics)

    def reseed(self, first: State, rng: jax.Array) -> State:
        """Fresh episode that REUSES the cached first physics state (brax-style
        cheap auto-reset: no make_data/forward) but re-draws every per-episode
        quantity -- command, servo gains, latency, backlash, IMU error -- so
        episode-level DR diversity survives auto-resetting."""
        rng, r_ep, r_cmd, r_obs = jax.random.split(rng, 4)
        servo, lat_ms, lash, imu_R, imu_bias = self._draw_episode(r_ep)
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next, traj_on = self._sample_cmd(r_cmd, step_i)
        if self.ext_cmd:
            rng, r_traj = jax.random.split(rng)
            traj = self._draw_traj(r_traj)
            # recovery slots keep their fallen cached-first-state: re-pin the
            # stand command and reset the recovered flag each episode
            stand = jp.zeros(7).at[3].set(1.0).astype(jp.float32)
            cmd = jp.where(first.recover_slot > 0, stand, cmd)
            cmd_next = jp.where(first.recover_slot > 0,
                                jp.asarray(10 ** 9, dtype=jp.int32), cmd_next)
            traj_on = jp.where(first.recover_slot > 0, 0.0, traj_on)
        else:
            traj = jp.zeros(3)
        if self.gait_clock:
            rng, r_gf, r_gp = jax.random.split(rng, 3)
            gait_freq = jax.random.uniform(r_gf, minval=1.25, maxval=1.75)
            gait_phase = jax.random.uniform(r_gp, minval=-jp.pi, maxval=jp.pi)
        else:
            gait_freq = jp.zeros(())
            gait_phase = jp.zeros(())
        prev_action = jp.zeros(self._nq_act, dtype=jp.float32)
        frame = self._obs(first.data, prev_action, cmd, step_i, imu_R,
                          imu_bias, r_obs, gait_phase)
        obs_hist = jp.tile(frame, (self.obs_hist_len - 1, 1)) \
            if self.obs_hist_len > 1 else jp.zeros((0, self.obs_frame))
        obs = (jp.concatenate([frame, obs_hist.reshape(-1)])
               if self.obs_hist_len > 1 else frame)
        metrics = {k: jp.zeros(()) for k in self._metric_keys}
        return first._replace(
            obs=obs, reward=jp.zeros(()), done=jp.zeros(()), rng=rng,
            prev_action=prev_action,
            last_target=self._default.astype(jp.float32), step_i=step_i,
            air_time=jp.zeros(2), last_air=jp.zeros(2),
            gait_freq=gait_freq, gait_phase=gait_phase, obs_hist=obs_hist,
            cmd=cmd, cmd_next=cmd_next, traj=traj,
            traj_on=traj_on, recover_slot=first.recover_slot,
            recovered=1.0 - first.recover_slot,
            best_h=first.data.qpos[2],
            head_ref=_quat_yaw(first.data.qpos[3:7]), servo=servo,
            lat_ms=lat_ms, lash=lash, imu_R=imu_R, imu_bias=imu_bias,
            metrics=metrics)

    def step(self, state: State, action: jax.Array) -> State:
        m = self.model
        rng, r_push_u, r_push_a, r_jit, r_obs, r_cmd = jax.random.split(
            state.rng, 6)
        action = jp.clip(action, -1.0, 1.0)
        if self.action_map == "full":
            span = jp.where(action >= 0.0, self._hi - self._default,
                            self._default - self._lo)
            target = self._default + span * action
        else:
            target = jp.clip(self._default + self._scale * action,
                             self._lo, self._hi)

        ang = jax.random.uniform(r_push_a, maxval=2 * jp.pi)
        if self.push_kick:
            # velocity-kick perturbation (Playground style): an instantaneous
            # horizontal delta-v instead of a held force
            push_on = jax.random.uniform(r_push_u) < self.push_prob
            mag = jax.random.uniform(jax.random.fold_in(r_push_a, 1),
                                     minval=self.kick_range[0],
                                     maxval=self.kick_range[1])
            kick = jp.where(push_on, mag, 0.0)
            data = state.data.replace(
                qvel=state.data.qvel.at[0].add(kick * jp.cos(ang))
                                    .at[1].add(kick * jp.sin(ang)))
        else:
            # random shove, held for the whole control step (matches CPU env)
            push_on = (jax.random.uniform(r_push_u) < self.push_prob) & (
                self.push_force > 0.0)
            xfrc = jp.zeros((self.mj_model.nbody, 6))
            xfrc = xfrc.at[self._torso_bid, 0].set(
                jp.where(push_on, self.push_force * jp.cos(ang), 0.0))
            xfrc = xfrc.at[self._torso_bid, 1].set(
                jp.where(push_on, self.push_force * jp.sin(ang), 0.0))
            data = state.data.replace(xfrc_applied=xfrc)

        # sub-step latency: first lat_k substeps run on the previous target
        ms = state.lat_ms
        if self.latency_jitter_ms > 0.0:
            ms = ms + jax.random.uniform(r_jit, minval=-self.latency_jitter_ms,
                                         maxval=self.latency_jitter_ms)
        ms = jp.clip(ms, 0.0, self.control_dt * 1000.0)
        lat_k = jp.minimum(jp.round(ms / (self.sim_dt * 1000.0)).astype(jp.int32),
                           self.n_substeps)

        kp, kd, stall, w0 = (state.servo[0], state.servo[1],
                             state.servo[2], state.servo[3])
        lash = state.lash
        last_target = state.last_target

        def substep(carry, i):
            d, _ = carry
            cur = jp.where(i < lat_k, last_target, target)
            q = d.qpos[self._jq0:self._jq1]
            qd = d.qvel[self._jv0:self._jv1]
            cap = stall * jp.clip(1.0 - jp.abs(qd) / w0, 0.0, 1.0)
            err = cur - q
            err = jp.sign(err) * jp.maximum(jp.abs(err) - 0.5 * lash, 0.0)
            tau = jp.clip(kp * err - kd * qd, -cap, cap)
            d = d.replace(qfrc_applied=jp.zeros(self.mj_model.nv
                                                ).at[self._jv0:self._jv1].set(tau))
            d = mjx.step(m, d)
            return (d, tau), None

        (data, tau), _ = jax.lax.scan(
            substep, (data, jp.zeros(self._nq_act)),
            jp.arange(self.n_substeps))

        up_z = data.sensordata[self._up_adr + 2]
        height = data.qpos[2]                     # flat ground: ground_z == 0
        qd_j = data.qvel[self._jv0:self._jv1]
        energy = jp.sum(jp.abs(tau) * jp.abs(qd_j))
        action_rate = jp.sum((action - state.prev_action) ** 2)
        power_w = jp.sum(jp.maximum(tau * qd_j, 0.0) + _K_CU * tau ** 2)

        # command tracking (exp kernels sigma=0.5; cmd_dense option)
        yaw = _quat_yaw(data.qpos[3:7])
        cth, sth = jp.cos(yaw), jp.sin(yaw)
        vx_body = cth * data.qvel[0] + sth * data.qvel[1]
        vy_body = -sth * data.qvel[0] + cth * data.qvel[1]
        wz = data.qvel[5]
        cmd_v, cmd_w = state.cmd[0], state.cmd[1]
        if self.ext_cmd:
            # 7-channel layout: yaw rate lives in c2 (c1 is vy). Without this
            # the yaw kernel silently tracked the SIDESTEP channel -- caught
            # by parity block 2e's non-zero fixed command (2026-07-17).
            cmd_w = state.cmd[2]
        planar = jp.sqrt(data.qvel[0] ** 2 + data.qvel[1] ** 2)
        standing = ((height > 0.85 * self._nominal_h)
                    & (up_z > 0.9)).astype(jp.float32)
        # foot contacts + one-leg geometry BEFORE the reward: the round-4
        # skill-compliance gate prices the tracking kernels by lift_ok
        con = self._foot_contacts(data)           # (2,) bool
        lifted = jp.zeros((), dtype=bool)
        foot_err = jp.zeros(())
        foot_clear = jp.zeros(())
        lift_ok = jp.zeros(())
        foot_kernel = jp.zeros(())
        com_off = jp.zeros(())
        com_kernel = jp.zeros(())
        head_err = jp.zeros(())
        if self.ext_cmd:
            # one-leg modes: lift = -1 (left) / +1 (right). Correct contact
            # pattern (stance down, swing up, >= lift_clear clearance) and
            # the swing-foot target kernel in the torso-yaw frame.
            cmd_lift = state.cmd[4]
            lifted = jp.abs(cmd_lift) > 0.5
            is_r = cmd_lift > 0.0
            con_swing = jp.where(is_r, con[1], con[0])
            con_stance = jp.where(is_r, con[0], con[1])
            sole_w = jp.where(is_r, data.geom_xpos[self._sole_gids[1]],
                              data.geom_xpos[self._sole_gids[0]])
            foot_clear = sole_w[2] - self._sole_z0
            lift_ok = (lifted & ~con_swing & con_stance
                       & (foot_clear >= self.lift_clear)).astype(jp.float32)
            rel_w = sole_w - data.xpos[self._torso_bid]
            rel = jp.stack([cth * rel_w[0] + sth * rel_w[1],
                            -sth * rel_w[0] + cth * rel_w[1],
                            rel_w[2]])
            base = jp.where(is_r, self._foot_rel0[1], self._foot_rel0[0])
            tgt = base + jp.stack([state.cmd[5], jp.zeros(()),
                                   self.lift_height + state.cmd[6]])
            foot_err = jp.linalg.norm(rel - tgt)
            foot_kernel = (lifted.astype(jp.float32)
                           * jp.exp(-((foot_err / self.foot_sigma) ** 2)))
            # feet-crossing guard: torso-frame lateral separation of the two
            # sole centers must stay >= sole width + 5 mm
            pL = data.geom_xpos[self._sole_gids[0]] - data.xpos[self._torso_bid]
            pR = data.geom_xpos[self._sole_gids[1]] - data.xpos[self._torso_bid]
            y_sep = jp.abs((-sth * pL[0] + cth * pL[1])
                           - (-sth * pR[0] + cth * pR[1]))
            cross_frac = jp.clip((0.051 - y_sep) / 0.051, 0.0, 1.0)
            # CoM-over-stance-foot kernel (user 2026-07-23): pay for keeping
            # the whole-robot CoM planted over the support sole while a lift
            # is commanded -- the knee-flexion lift (thigh vertical, shank
            # folds back) is then the cheapest compliant posture, and the
            # stick-the-leg-out lift (CG lurches forward) is expensive.
            com = data.subtree_com[self._torso_bid]
            stance_w = jp.where(is_r, data.geom_xpos[self._sole_gids[0]],
                                data.geom_xpos[self._sole_gids[1]])
            com_off = jp.sqrt((com[0] - stance_w[0]) ** 2
                              + (com[1] - stance_w[1]) ** 2 + 1e-12)
            com_kernel = jp.exp(-((com_off / self.com_sigma) ** 2))

        if self.getup:
            # recovery: dense progress toward tall + upright, then a standing
            # bonus (half for being up, half for being STILL up there) that
            # makes a held stand the absorbing goal instead of thrashing.
            # The uprightness term is GATED on pelvis height (v3): a vertical
            # torso at kneel height must NOT pay, or the kneel becomes the
            # absorbing state (v2's ceiling) -- the pike to standing requires
            # folding the torso down through up_z ~ 0 first.
            still = jp.exp(-((planar / 0.2) ** 2))
            up_gate = jp.clip((height - 0.12) / 0.08, 0.0, 1.0)
            primary = (self.w_recover_h * jp.clip(height / self._nominal_h,
                                                  0.0, 1.0)
                       + self.w_recover_up * 0.5 * (up_z + 1.0) * up_gate
                       + self.stand_bonus * standing * (0.5 + 0.5 * still))
        elif self.ext_cmd:
            # precision tracking: joint 2D velocity kernel (vx AND vy -- a
            # sidestep command is a first-class target, not side-slip), yaw
            # kernel, and a height kernel against the commanded crouch.
            # cmd_dense (round-2 fix): under a moving velocity command the
            # kernel pays 1.05/step for STANDING (sigma 0.5 at cmd 0.4) --
            # precision_v1 farmed that plus the full yaw+height kernels into
            # a 5.4/step do-nothing optimum (0 falls, 0 motion). The dense
            # term is the dash-lineage gradient: velocity PROJECTED on the
            # commanded direction, capped at the command, zero for standing,
            # negative for moving the wrong way.
            cmd_vy, cmd_h = state.cmd[1], state.cmd[3]
            sp_cmd = jp.sqrt(cmd_v ** 2 + cmd_vy ** 2)
            v_kernel = self.w_track_v * jp.exp(
                -((vx_body - cmd_v) ** 2 + (vy_body - cmd_vy) ** 2) / 0.25)
            if self.cmd_dense:
                v_par = (vx_body * cmd_v + vy_body * cmd_vy) / jp.maximum(
                    sp_cmd, 1e-9)
                v_dense = self.w_track_v * jp.clip(
                    v_par / jp.maximum(sp_cmd, 1e-9), -1.0, 1.0)
                v_term = jp.where(sp_cmd > 0.05, v_dense, v_kernel)
            else:
                v_term = v_kernel
            w_term = self.w_track_w * jp.exp(-((wz - cmd_w) / 0.5) ** 2)
            # integrated-heading kernel: the rate kernel forgives chronic
            # under-turning (each step's shortfall is graded fresh); the
            # integrator makes facing error accumulate until paid back.
            # Sits inside the g_skill-gated primary like the rate kernel.
            ref_now = state.head_ref + cmd_w * self.control_dt
            head_err = jp.arctan2(jp.sin(yaw - ref_now),
                                  jp.cos(yaw - ref_now))
            w_term = w_term + self.w_heading * jp.exp(
                -((head_err / 0.25) ** 2))
            # height kernel at full weight only when no motion is commanded
            # (its full 1.0/step was part of the do-nothing income)
            h_gate = jp.where((sp_cmd > 0.05) | (jp.abs(cmd_w) > 0.05),
                              0.3, 1.0)
            h_norm = jp.exp(
                -(((height - cmd_h * self._nominal_h) / 0.04) ** 2))
            h_term = self.w_track_h * h_gate * h_norm
            # skill-compliance gate (round 4): precision_v3 answered lift/
            # crouch commands with calm standing -- the velocity/yaw kernels
            # paid ~4/step for vels=0 regardless. Under a skill command they
            # now pay in proportion to the skill being DONE (floor 0.2 keeps
            # a gradient toward stillness while learning the skill).
            crouching = (~lifted) & (cmd_h < 0.97)
            g_skill = jp.where(lifted, 0.2 + 0.8 * lift_ok, 1.0)
            g_skill = jp.where(crouching, 0.2 + 0.8 * h_norm, g_skill)
            primary = g_skill * (v_term + w_term) + h_term
            if self.recover_mix > 0:
                # while DOWN in a recovery episode: height RATCHET is the
                # primary -- only NEW height above the episode best pays
                # (getup_v1 lesson: absolute height paid ~0.33/step for
                # sitting motionless, a do-nothing optimum worth ~165/ep;
                # the ratchet pays a bounded one-time sum for the rise and
                # zero for holding any pose), plus the standing bonus and
                # time pressure sized to cancel the upright+alive income a
                # settled non-riser collects (0.5*up_z + 0.1 <= 0.6)
                still = jp.exp(-((planar / 0.2) ** 2))
                h_gain = jp.maximum(height - state.best_h, 0.0)
                rec_primary = (200.0 * h_gain
                               + 1.0 * standing * (0.5 + 0.5 * still)
                               - 0.7)
                primary = jp.where(state.recovered < 0.5, rec_primary,
                                   primary)
        else:
            v_kernel = self.w_track_v * jp.exp(-((vx_body - cmd_v) / 0.5) ** 2)
            if self.cmd_dense:
                v_dense = self.w_track_v * jp.minimum(
                    vx_body, cmd_v) / jp.maximum(cmd_v, 1e-9)
                v_term = jp.where(cmd_v > 0.05, v_dense, v_kernel)
            else:
                v_term = v_kernel
            primary = v_term + self.w_track_w * jp.exp(
                -((wz - cmd_w) / 0.5) ** 2)

        # posture reference: the commanded crouch height in ext mode
        h_ref = (state.cmd[3] * self._nominal_h if self.ext_cmd
                 else self._nominal_h)
        reward = (primary
                  + self.w_upright * up_z
                  + self.alive_bonus
                  - self.w_height * jp.abs(height - h_ref)
                  - self.w_energy * energy
                  - self.w_action_rate * action_rate
                  - self.w_power * power_w)

        # gait shaping: on for moving commands, double-support bonus for stand
        cmd_moving = (jp.abs(cmd_v) > 0.05) | (jp.abs(cmd_w) > 0.05)
        if self.ext_cmd:
            # lift + swing-foot rewards (geometry computed pre-reward above)
            reward += self.w_lift * lift_ok
            reward += self.w_track_foot * foot_kernel
            reward -= self.w_foot_cross * cross_frac
            # clearance-gated (skills_v2 lesson): ungated, the CoM kernel
            # paid 0.75/step for leaning onto the stance foot with BOTH
            # feet planted -- a partial do-nothing income that beat ever
            # lifting (balance clearance 0%). Now it scales with the swing
            # foot actually coming up, so the CoM shaping only pays as the
            # lift it is meant to shape happens.
            reward += (self.w_com_stance * lifted.astype(jp.float32)
                       * com_kernel
                       * jp.clip(foot_clear / self.lift_clear, 0.0, 1.0))
            # a lifted command is a balance task, not locomotion: gait shaping
            # off; a sidestep command (vy) is locomotion like any other.
            # (cmd_moving already covers c0 and c2 via cmd_v/cmd_w.)
            cmd_moving = (cmd_moving | (jp.abs(state.cmd[1]) > 0.05)) & ~lifted
        landed = con & (state.air_time > 0.0)
        if self.w_feet_air:
            reward += jp.where(cmd_moving, self.w_feet_air * jp.sum(
                jp.where(landed,
                         jp.minimum(state.air_time, self.air_time_target)
                         - 0.5 * self.air_time_target, 0.0)), 0.0)
        # plan-v2 Phase A terms (all default-off) -----------------------------
        gait_phase = state.gait_phase
        if self.gait_clock:
            gait_phase = jp.mod(state.gait_phase + 2 * jp.pi * self.control_dt
                                * state.gait_freq + jp.pi,
                                2 * jp.pi) - jp.pi
        if self.w_feet_phase:
            # periodic swing-height targets: feet half a cycle apart
            # (left swings on sin(phase) > 0, right on the opposite half)
            rz_l = self.swing_height * jp.clip(jp.sin(gait_phase), 0.0, None)
            rz_r = self.swing_height * jp.clip(-jp.sin(gait_phase), 0.0, None)
            z_l = data.geom_xpos[self._sole_gids[0]][2] - self._sole_z0
            z_r = data.geom_xpos[self._sole_gids[1]][2] - self._sole_z0
            r_phase = jp.exp(-((z_l - rz_l) ** 2 + (z_r - rz_r) ** 2)
                             / self.feet_phase_s2)
            reward += jp.where(cmd_moving, self.w_feet_phase * r_phase, 0.0)
        if self.w_feet_slip:
            v_l = data.cvel[self._foot_bids[0], 3:5]
            v_r = data.cvel[self._foot_bids[1], 3:5]
            slip = (con[0] * jp.sqrt(jp.sum(v_l ** 2) + 1e-12)
                    + con[1] * jp.sqrt(jp.sum(v_r ** 2) + 1e-12))
            reward -= self.w_feet_slip * slip
        if self.w_orientation:
            upv = data.sensordata[self._up_adr:self._up_adr + 3]
            reward -= self.w_orientation * (upv[0] ** 2 + upv[1] ** 2)
        if self.w_ang_vel_xy:
            reward -= self.w_ang_vel_xy * (data.qvel[3] ** 2
                                           + data.qvel[4] ** 2)
        if self.w_pose:
            # posture regularization only under plain locomotion/stand
            # commands with recovery complete (fights skills otherwise)
            if self.ext_cmd:
                pose_gate = ((~lifted) & (state.cmd[3] >= 0.97)
                             & (state.recovered > 0.5)).astype(jp.float32)
            else:
                pose_gate = 1.0
            reward -= (self.w_pose * pose_gate
                       * jp.sum((data.qpos[self._jq0:self._jq1]
                                 - self._default) ** 2))
        if self.w_dof_limits:
            q = data.qpos[self._jq0:self._jq1]
            out = (jp.maximum(self._soft_lo - q, 0.0)
                   + jp.maximum(q - self._soft_hi, 0.0))
            reward -= self.w_dof_limits * jp.sum(out)
        if self.w_mimic:
            q_ref = self._mimic_ref(state.cmd, gait_phase, state.gait_freq)
            dq = data.qpos[self._jq0:self._jq1] - q_ref
            mim_gate = 1.0
            if self.ext_cmd:
                mim_gate = ((~lifted) & (state.recovered > 0.5)
                            ).astype(jp.float32)
            reward += (self.w_mimic * mim_gate
                       * jp.exp(-jp.sum(dq ** 2) / self.mimic_s2))
        if self.w_rise_dofvel and self.ext_cmd and self.recover_mix > 0:
            # jerk control during the rise (HumanUP/utra lesson)
            reward -= (self.w_rise_dofvel
                       * jp.where(state.recovered < 0.5, 1.0, 0.0)
                       * jp.sum(qd_j ** 2))
        if self.w_rise_ref and self.ext_cmd and self.recover_mix > 0:
            # staged-rise reference while DOWN (getup_v3): dense guidance
            # toward the scripted tuck->plant->squat->stand trajectory
            q_rr = self._rise_ref(state.step_i.astype(jp.float32)
                                  * self.control_dt)
            dq_rr = data.qpos[self._jq0:self._jq1] - q_rr
            reward += (self.w_rise_ref
                       * jp.where(state.recovered < 0.5, 1.0, 0.0)
                       * jp.exp(-jp.sum(dq_rr ** 2) / self.rise_ref_s2))

        # gait symmetry: on touchdown, penalize the swing-duration mismatch
        # vs the OTHER foot's last completed swing (locomotion commands only)
        last_air = state.last_air
        if self.w_symmetry:
            other_last = jp.stack([state.last_air[1], state.last_air[0]])
            sym_valid = landed & (other_last > 0.0)
            reward -= jp.where(cmd_moving, self.w_symmetry * jp.sum(
                jp.where(sym_valid,
                         jp.abs(state.air_time - other_last), 0.0)), 0.0)
        last_air = jp.where(landed, state.air_time, state.last_air)
        air_time = jp.where(con, 0.0, state.air_time + self.control_dt)
        single = con[0] != con[1]
        double = con[0] & con[1]
        if self.w_single_support:
            # while lifted, single support IS the commanded state (the generic
            # stand-command double-support bonus would fight the lift reward)
            stand_ss = jp.where(lifted, single.astype(jp.float32),
                                double.astype(jp.float32))
            reward += jp.where(
                cmd_moving, self.w_single_support * single.astype(jp.float32),
                self.w_single_support * stand_ss)
        if self.w_lateral:
            # ext mode: vy is a tracked command channel, penalize the ERROR
            vy_ref = state.cmd[1] if self.ext_cmd else 0.0
            reward -= self.w_lateral * jp.abs(vy_body - vy_ref)
        if self.w_pitch_rate:
            reward -= self.w_pitch_rate * (jp.abs(data.qvel[3])
                                           + jp.abs(data.qvel[4]))

        step_i = state.step_i + 1
        # ext mode: the fall floor tracks the commanded crouch (a commanded
        # 0.6-height crouch is 0.17 m -- below the legacy 0.18 fall line)
        fall_h = (self.fall_height * state.cmd[3] if self.ext_cmd
                  else self.fall_height)
        fell = (height < fall_h) | (up_z < self.fall_up_z)
        recovered = state.recovered
        if self.ext_cmd and self.recover_mix > 0:
            # while down, being fallen is the TASK, not the failure; the
            # first achieved stand arms normal fall rules for the rest of
            # the episode (graded on the pre-step flag, updated after)
            fell = fell & (state.recovered > 0.5)
            recovered = jp.maximum(state.recovered, standing)
        if self.getup:
            done_flag = jp.zeros(())          # being down IS the task
        else:
            reward = reward - self.fall_cost * fell.astype(jp.float32)
            done_flag = fell.astype(jp.float32)

        # resample AFTER the reward graded the command the policy saw
        new_cmd, new_next, new_traj_on = self._sample_cmd(r_cmd, step_i)
        resample = step_i >= state.cmd_next
        # heading reference: advance by the GRADED command's yaw rate
        # (wrapped); a fresh command restarts the integrator at the current
        # facing so old debt isn't carried across command boundaries
        head_ref = state.head_ref + cmd_w * self.control_dt
        head_ref = jp.arctan2(jp.sin(head_ref), jp.cos(head_ref))
        head_ref = jp.where(resample, yaw, head_ref)
        cmd = jp.where(resample, new_cmd, state.cmd)
        cmd_next = jp.where(resample, new_next, state.cmd_next)
        traj_on = jp.where(resample, new_traj_on, state.traj_on)
        if self.ext_cmd:
            # trajectory-mode command evolution (traj_on carries the mode:
            # 0 off, 1 foot-circle, 2 march, 3 sway); scripted evals that
            # pin commands via cmd_fixed keep mode 0 and are untouched.
            th = (state.traj[2] + state.traj[1]
                  * step_i.astype(jp.float32) * self.control_dt)
            circle = (cmd.at[5].set(state.traj[0] * jp.cos(th))
                      .at[6].set(state.traj[0] * jp.sin(th)))
            # march: alternate the lifted leg each half period; the foot
            # target bobs 0..radius above the nominal lift height
            march = (cmd.at[4].set(jp.where(jp.sin(th) >= 0, 1.0, -1.0))
                     .at[5].set(0.0)
                     .at[6].set(state.traj[0] * jp.abs(jp.sin(th))))
            # sway: standing lateral weight-shift via an oscillating vy
            sway = cmd.at[1].set(self.sway_vy * jp.sin(th))
            cmd = jp.where(traj_on == 1.0, circle, cmd)
            cmd = jp.where(traj_on == 2.0, march, cmd)
            cmd = jp.where(traj_on == 3.0, sway, cmd)

        frame = self._obs(data, action, cmd, step_i, state.imu_R,
                          state.imu_bias, r_obs, gait_phase)
        if self.obs_hist_len > 1:
            obs = jp.concatenate([frame, state.obs_hist.reshape(-1)])
            obs_hist = jp.concatenate(
                [frame[None], state.obs_hist[:-1]], axis=0)
        else:
            obs = frame
            obs_hist = state.obs_hist
        metrics = {
            "power_w": power_w, "vx_body": vx_body, "wz": wz,
            "height": height, "up_z": up_z, "fell": fell.astype(jp.float32),
            "track_v_err": jp.abs(vx_body - cmd_v),
            "track_w_err": jp.abs(wz - cmd_w),
            "standing": standing,
        }
        if self.ext_cmd:
            metrics.update(
                height_err=jp.abs(height - h_ref),
                foot_err=lifted.astype(jp.float32) * foot_err,
                foot_clear=lifted.astype(jp.float32) * foot_clear,
                foot_sep=y_sep,
                lift_ok=lift_ok,
                vy_body=vy_body,
                recovered=recovered,
                com_stance=lifted.astype(jp.float32) * com_off,
                head_err=jp.abs(head_err))
        return State(data=data, obs=obs, reward=reward,
                     done=done_flag, rng=rng,
                     prev_action=action.astype(jp.float32),
                     last_target=target, step_i=step_i, air_time=air_time,
                     last_air=last_air,
                     gait_freq=state.gait_freq, gait_phase=gait_phase,
                     obs_hist=obs_hist,
                     cmd=cmd, cmd_next=cmd_next, traj=state.traj,
                     traj_on=traj_on, recover_slot=state.recover_slot,
                     recovered=recovered,
                     best_h=jp.maximum(state.best_h, height),
                     head_ref=head_ref,
                     servo=state.servo,
                     lat_ms=state.lat_ms, lash=state.lash, imu_R=state.imu_R,
                     imu_bias=state.imu_bias, metrics=metrics)


def domain_randomize(model, rng: jax.Array, mass_range: float = 0.15,
                     friction_range: float = 0.4,
                     payload_max: float | None = None,
                     payload_bid: int | None = None,
                     floor_gid: int = 0, nom_mass=None, nom_inertia=None,
                     nom_friction=None):
    """Batch-level model DR (brax randomization_fn style): per-env body
    mass/inertia scale, floor friction scale, and payload mass draw. Returns
    (batched_model, in_axes) for vmapped training."""
    nbody = model.body_mass.shape[0]

    @jax.vmap
    def rand(rng):
        r_m, r_f, r_p = jax.random.split(rng, 3)
        mf = jax.random.uniform(r_m, (nbody,), minval=1 - mass_range,
                                maxval=1 + mass_range)
        mass = nom_mass * mf
        inertia = nom_inertia * mf[:, None]
        ff = jax.random.uniform(r_f, minval=1 - friction_range,
                                maxval=1 + friction_range)
        friction = nom_friction.at[floor_gid, 0].set(
            nom_friction[floor_gid, 0] * ff)
        if payload_max is not None and payload_bid is not None:
            mp = jax.random.uniform(r_p, maxval=payload_max)
            mass = mass.at[payload_bid].set(mp)
            inertia = inertia.at[payload_bid].set(
                nom_inertia[payload_bid] * (mp / _PAYLOAD_REF))
        return mass, inertia, friction

    mass, inertia, friction = rand(rng)
    in_axes = jax.tree_util.tree_map(lambda x: None, model)
    in_axes = in_axes.tree_replace({
        "body_mass": 0, "body_inertia": 0, "geom_friction": 0})
    model = model.tree_replace({
        "body_mass": mass, "body_inertia": inertia,
        "geom_friction": friction})
    return model, in_axes
