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


def policy_range(m):
    """Per-actuator (lo, hi) the POLICY trains in, radians. EXACT MIRROR of
    sim/walker_env.py's policy_range() -- read the docstring there for why the
    policy range and the joint's mechanical stop are separate numbers, and why
    a plant that declares no ctrlrange (v2, v3yaw, every legacy referee) is
    bit-identical to what it always was. Duplicated rather than imported so
    the MJX env keeps its standalone import graph, like the _TICK_* block."""
    jnt = m.actuator_trnid[:, 0]
    lo = np.array(m.jnt_range[jnt, 0], dtype=float)
    hi = np.array(m.jnt_range[jnt, 1], dtype=float)
    lim = np.asarray(m.actuator_ctrllimited).astype(bool)
    lo[lim] = np.asarray(m.actuator_ctrlrange)[lim, 0]
    hi[lim] = np.asarray(m.actuator_ctrlrange)[lim, 1]
    return lo, hi

_STS_STALL_12V = 2.94                        # N*m  (30 kg*cm @12V)
_STS_NOLOAD_12V = 4.04   # rad/s @12V, MEASURED 2026-08-03 (86% of the
                         # datasheet 4.712) -- see walker_env.py's comment
                         # and tools/measure_servo_speed.py; keep in step
_PAYLOAD_REF = 0.154                         # kg, GoPro MAX incl. battery
_K_CU = 3.75                                 # W/(N*m)^2, ST3215 stall calib.

# -- STS3215 encoder/bus quantization (quantize_ticks) ------------------------
# Exact mirror of sim/walker_env.py's block (same names, same values), which in
# turn mirrors sim/sil/harness.py + docs/sil-harness.md: position is a uint16
# tick word (4096 ticks/rev, full scale 0..4095, zero 2048), velocity an
# integer reg-58 steps/s word (sign-magnitude, 15-bit magnitude), goal
# positions integer ticks. Parity-gated (block 2i).
_TICK_STEPS_PER_REV = 4096
_TICK_RAD = 2.0 * np.pi / _TICK_STEPS_PER_REV   # one quantum, rad
_TICK_ZERO = 2048                                # zero_steps, nominal cal
_TICK_MAX = 4095                                 # encoder full scale
_TICK_VEL_MAX = 32767                            # reg-58 sign-magnitude |v|
# Per-joint direction signs -- sim/sil/cal/cal_nominal.json "dir" is +1 on
# every joint (obs::Calibration's C++ default; a freshly "Set Middle
# Position"-ed, correctly oriented robot). See walker_env._TICK_DIR.
_TICK_DIR = 1.0


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
                              # to 1.0 at the first HELD stand (getup_v9:
                              # stand_streak >= stand_hold_n, not an instant
                              # height crossing -- ballistic bank starts were
                              # farming the flip mid-flight)
    stand_streak: jax.Array   # ()  consecutive standing steps (recovery
                              # episodes; resets to 0 on any non-standing step)
    best_h: jax.Array         # ()  episode-best torso height: the recovery
                              # ratchet pays only for NEW height above this
                              # (static height income was a do-nothing optimum)
    head_ref: jax.Array       # ()  integrated commanded heading (rad): under-
                              # turning accumulates facing error the policy
                              # must pay back (rate kernels forgave it)
    rise_t0: jax.Array        # ()  rise-reference phase offset (s) set by the
                              # start pose (getup_v4): a kneel start begins
                              # 1/3 into the schedule instead of being pulled
                              # BACK to the tuck the t=0 reference demands
    cmd_crouch: jax.Array     # ()  per-episode crouch-command draw: replaces
                              # the otherwise-FROZEN cmd[3]=1.0 in sampled
                              # commands (SIL finding #2); 1.0 when the
                              # feature is off
    servo: jax.Array          # (4,) kp, kd, stall, no-load speed (per-episode)
    lat_ms: jax.Array         # ()  per-episode sub-step latency
    lash: jax.Array           # ()  per-episode backlash (rad)
    zero_off: jax.Array       # (n_act,) per-episode joint ZERO OFFSET (rad):
                              # the servo's tick zero vs the policy's frame.
                              # The hardware zero is set by eye and moved by
                              # 1-4.7 deg on 2026-09-03 when loose horns were
                              # tightened; the policy must not depend on it.
                              # Applied as: physical target = target + off,
                              # observed q = physical q - off.
    act_lag: jax.Array        # ()  per-episode action-chain lag pole, Hz
                              # (0 = pass-through). Stand-in for the deployed
                              # C2 shaper PLUS the servo's dynamic response:
                              # 2026-08-31 bench walk measured joint motion at
                              # ~0.5x sim dq -- reproduced only by ~2 Hz of
                              # 3-stage filtering, 5x the 10 Hz shaper alone.
    lag_y: jax.Array          # (3, n_act) cascaded lag filter state
    act_hist: jax.Array       # (act_delay_max+1, n_act) recent joint TARGETS,
                              # row 0 = this tick; the actuator serves row
                              # act_delay. Servo DEAD TIME DR: 2026-09-05 bench
                              # (one foot clamped, 16 traces, 3 loads, 3-10 deg)
                              # measured ~85 ms pure delay + 30 ms lag before
                              # the STS3215 follows a target, amplitude- and
                              # load-independent -- phase without attenuation,
                              # which the lag cascade above cannot produce.
    act_delay: jax.Array      # ()  int32 per-episode target delay, 0..act_delay_max
    imu_R: jax.Array          # (3,3) mounting misalignment
    imu_bias: jax.Array       # (3,) gyro bias
    gyro_hist: jax.Array      # (3, 3) raw torso gyro at the last 3 obs ticks,
                              # row 0 = now; the obs reads row gyro_delay
    gyro_delay: jax.Array     # ()  int32 per-episode obs delay, 0..gyro_delay_max
    gyro_gain: jax.Array      # ()  per-episode gyro obs gain (1 = exact).
                              # 2026-09-03: on the robot the v27tilt_b student
                              # oscillates and falls with the gyro as measured
                              # (sign/scale/frame all verified against this
                              # sim) and STANDS with the gyro obs at 0.5x --
                              # the loop through the rate term is gain-limited
                              # on the real plant. Train so the policy cannot
                              # depend on a crisp, exact rate.
    metrics: dict[str, jax.Array]


def _quat_yaw(q):
    """Torso yaw from a wxyz quaternion (identical arithmetic to the CPU
    referee's -- the heading integrator depends on it matching)."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return jp.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _quat_pitch(q):
    """Torso ZYX-euler pitch from a wxyz quaternion (identical arithmetic to
    the CPU env's inline copy in walker_env.step, and to the gait probe:
    + = nose down toward +x). Used only by the w_pitch_hinge penalty."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return jp.arcsin(jp.clip(2 * (qw * qy - qz * qx), -1.0, 1.0))


def _grav_up(model):
    """Unit UP vector (-g/|g|) in world coords. (0,0,1) at nominal gravity."""
    g = model.opt.gravity
    return -g / jp.sqrt(jp.dot(g, g))


def _grav_rot(model):
    """3x3 rotation taking world vectors into the GRAVITY-ALIGNED frame
    (z axis = -g/|g|). Identity at nominal gravity.

    Gravity-tilt DR (tilt_max_deg) only rotates opt.gravity; the torso_up
    sensor, the upright/fall up_z, and the CoM-over-foot projections all
    measure against WORLD z. Without this frame the policy trains against
    a constant, UNOBSERVABLE lateral pull while still being paid to stay
    world-vertical -- v27tilt learned to brace against it (115 W standing,
    54/144). A real IMU reads gravity: on a tilted desk the robot plumb to
    gravity reads 'upright' and the floor is what slopes. Rodrigues
    rotation of a=-g/|g| onto z; the s2->0 branch is the exact limit."""
    a = _grav_up(model)
    z = jp.array([0.0, 0.0, 1.0])
    v = jp.cross(a, z)
    c = jp.dot(a, z)
    s2 = jp.dot(v, v)
    K = jp.array([[0.0, -v[2], v[1]],
                  [v[2], 0.0, -v[0]],
                  [-v[1], v[0], 0.0]])
    f = jp.where(s2 > 1e-12, (1.0 - c) / jp.maximum(s2, 1e-12), 0.5)
    return jp.eye(3) + K + f * (K @ K)


def _grav_proj_xy(model, p, z_ref):
    """xy of point p carried ALONG GRAVITY down to height z_ref (where the
    CoM actually loads the foot on a tilted support). Plain p[:2] at
    nominal gravity."""
    g = model.opt.gravity
    ghat = g / jp.sqrt(jp.dot(g, g))
    t = (z_ref - p[2]) / ghat[2]
    return p[:2] + t * ghat[:2]



# Speed-coupled gait clock (2026-08-25). The contact-schedule reward pinned
# cadence to the clock -- which capped speed at ~0.37 m/s (1.5 Hz x ~25 cm
# strides; speed_ladder top_speed, both 08-24 arms) and made the 1.0 Hz slow
# metronome unreachable (cadence_err_slow 0.30). Natural walkers change speed
# with cadence AND stride; this scales the clock with the commanded planar
# speed. One deterministic law shared by env_mjx, walker_env, and (pre-deploy)
# gen_obs_spec.py -> firmware; the sqrt shape follows cadence ~ sqrt(speed).
# Multiplier is 1.0 at SPEED_CLOCK_REF (the 0.35 m/s bread-and-butter speed),
# so existing gaits are untouched there.
SPEED_CLOCK_REF = 0.35
SPEED_CLOCK_LO = 0.7
SPEED_CLOCK_HI = 1.7


def speed_clock_scale(v_planar, lo=SPEED_CLOCK_LO, hi=SPEED_CLOCK_HI):
    """Clock-frequency multiplier for a commanded planar speed (m/s).
    hi is bounded by hardware: at 1.5 Hz base, x1.7 = 2.55 Hz stepping
    needs ~4.4 rad/s hip swings, over the measured 4.04 rad/s STS3215
    ceiling -- v24clockv (x1.7) lost the top end (top_speed 0.25).
    x1.25 (~1.9 Hz, ~2.8 rad/s swings) stays inside the envelope."""
    return jp.clip(jp.sqrt(jp.abs(v_planar) / SPEED_CLOCK_REF), lo, hi)


def contact_schedule(gait_phase, duty):
    """(2,) bool: scheduled STANCE flags (left, right) for a clock phase.

    Same convention as the w_feet_phase swing targets and the firmware
    clock: left swings on sin(phase) > 0, right on the opposite half.
    Stance is scheduled while the foot's swing drive sin is below `duty`
    (left: sin < duty; right: -sin < duty), so duty > 0 leaves a
    double-support band around each crossing instead of demanding an
    instantaneous flight-to-flight handoff. Module-level (not a method) so
    tests can pin the convention -- including that a half-cycle phase
    shift swaps the two feet, which the mirror map's phase entry relies
    on."""
    s = jp.sin(gait_phase)
    return jp.stack([s < duty, -s < duty])


def _terrain_patch(xml: str, spec: dict) -> str:
    """Replace the flat floor plane with the mosaic heightfield (same geom
    name 'floor' so friction handling is untouched). Mirrors the CPU env's
    _terrain_xml; the DATA both envs load is the same terrain_mosaic.npz,
    so the ground is bit-identical by construction."""
    asset = (f'<asset><hfield name="terrain" nrow="{int(spec["nrow"])}" '
             f'ncol="{int(spec["ncol"])}" '
             f'size="{spec["rx"]} {spec["ry"]} {spec["max_amp"]} 0.1"/>'
             f'</asset>\n  ')
    orig = re.search(r'<geom name="floor"[^>]*?/>', xml, flags=re.S)
    look = re.search(r'(material="[^"]+"|rgba="[^"]+")',
                     orig.group(0)) if orig else None
    geom = (f'<geom name="floor" type="hfield" hfield="terrain" '
            f'pos="{spec["cx"]} 0 0" contype="1" conaffinity="3" '
            f'{look.group(1) if look else ""} '
            # carpet friction -- mirrors sim/walker_env._terrain_xml EXACTLY
            f'friction="1.3 0.05 0.001" condim="4"/>')
    patched, n = re.subn(r'<geom name="floor"[^>]*?/>', geom, xml, flags=re.S)
    if n != 1:
        raise ValueError(f"expected exactly one floor geom, found {n}")
    return patched.replace("<worldbody>", asset + "<worldbody>", 1)


def _prep_model(xml_path: str, payload: bool, servo_joint_damping: float,
                mesh_floor: bool = False,
                payload_cg_z: float = 0.08,
                payload_cg_x: float = 0.0,
                terrain_spec: dict | None = None) -> mujoco.MjModel:
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
    if terrain_spec is not None:
        xml = _terrain_patch(xml, terrain_spec)
    if payload:
        # x matters: the camera bolts to gopro_base at GP_MOUNT_X, not on the
        # torso centreline. Hard-coding x=0 (before 2026-08-06) put 154 g --
        # 12 % of the robot -- 24 mm FORWARD of its mount, visibly floating off
        # the pad in every render and biasing the CoM the same way.
        body = (f'<body name="payload" pos="{payload_cg_x} 0 {payload_cg_z}">'
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
        w_still: float = 0.0,       # stand-gated joint-velocity penalty
        # (-w * sum(dq^2) while commanded to plain-stand, upright, no lift):
        # pays for the ABSENCE of motion, not just small corrections
        # (user 2026-08-12, stand shaking)
        w_stand_home: float = 0.0,  # stand-gated L1 pull of the SERVED joint
                                    # target onto the home pose, in RADIANS:
                                    # -w * sum|target - default|. 2026-09-05:
                                    # w_stand_zero (mean of squared ACTIONS)
                                    # is toothless in 'full' action units -- a
                                    # 10 deg knee split is a 0.11 action, so
                                    # w=10 charged 0.03/step and v30home
                                    # rested with knees +6/-10 deg. L1 makes
                                    # exactly-home a sharp optimum.
        w_stand_zero: float = 0.0,  # stand-gated ACTION-magnitude penalty: at a
                                    # plain stand the policy must output ~0, i.e.
                                    # hold the default (home) pose. 2026-09-04,
                                    # Tom: "train the policy that the zero
                                    # position is the stop/rest position" -- the
                                    # deployed student's stand was a learned
                                    # posture (7 deg hip adduction, 3 deg lean,
                                    # splayed yaws) that only holds on a sim
                                    # floor; on the robot it walks the legs into
                                    # each other. mean(action^2), same gate.
        w_hip_yaw: float = 0.0,     # 2026-09-11: sum(hip_yaw^2) penalty, ungated.
        # The line's "crouch" on hardware was a symmetric hip-yaw pinch
        # (L -7 / R +8 deg, twin + robot) with no descent; nothing charged
        # yaw (the mimic ref keeps yaw at 0 but its weight is tiny next to
        # the knee's). Turning is differential stride, not yaw, so 0 is right.
        w_crouch_track: float = 0.0,   # 2026-09-11: TIGHT crouch-depth kernel
        crouch_track_sigma: float = 0.015,  # (m). The 4 cm height kernel pays
        # 13% for standing tall at crouch 0.8 -- cheap to ignore. This one
        # pays only within ~1.5 cm of the commanded height and only while a
        # crouch is commanded (cmd[3] < 0.97).
        crouch_release: bool = False,  # 2026-09-11: while a crouch is commanded,
        # release the w_still and w_stand_com gates (both charged the descent:
        # joint velocity and CoM shift are the crouch).
        w_stand_com: float = 0.0,   # stand-gated CoM-over-midfoot kernel:
        # a passive (torque-off) stand only holds if the gravity moment at
        # the ankles stays under the servos' backdrive friction, i.e. the
        # CoM stays within ~16 mm of the support center
        stand_com_sigma: float = 0.02,
        speed_clock: bool = False,  # scale the gait clock with commanded
        # planar speed (speed_clock_scale) -- cadence follows the command
        # instead of pinning speed to 1.5 Hz x stride
        speed_clock_lo: float = SPEED_CLOCK_LO,
        speed_clock_hi: float = SPEED_CLOCK_HI,
        w_stand_knee: float = 0.0,  # stand-gated knee-angle kernel: a dead-
        # straight knee sag-collapses when torque is released (knee+ankle
        # fold together, ~28 cm drift); a +0.10 rad bias from the same pose
        # holds at ~5 cm (knee_lock_probe sweep, 2026-08-24)
        stand_knee_target: float = 0.10,
        stand_knee_sigma: float = 0.06,
        clock_stand_freeze: bool = False,   # hold the gait-clock phase at a
        # plain stand so the obs stop oscillating (see step(); 2026-08-12)
        w_pitch_hinge: float = 0.0,
        pitch_deadband_deg: float = 5.0,
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
        payload_max: float | None = None,  # if set, per-env payload mass drawn
        # uniform(0, payload_max) -- one policy handles camera-on AND -off
        # -- DR (per-episode, state-level) -------------------------------------
        domain_rand: bool = False,
        gain_range: float = 0.2,
        mass_range: float = 0.15,   # batch-level body mass/inertia scale +/- this
        friction_range: float = 0.4,  # batch-level floor friction scale +/- this
        latency_ms: float = 0.0,
        latency_ms_max: float | None = None,
        latency_jitter_ms: float = 0.0,
        backlash_deg: float = 0.0,
        backlash_deg_max: float | None = None,
        zero_offset_deg: float = 0.0,   # per-episode per-joint zero offset
        # drawn uniform(-z, +z) deg (calibration-error DR, 2026-09-03)
        act_lag_hz: float = 0.0,
        act_lag_hz_max: float | None = None,
        tilt_max_deg: float = 0.0,   # batch-level gravity-tilt DR (un-level floor)
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
        foot_cross_sep: float = 0.051,  # torso-frame lateral sole separation
        # below which w_foot_cross starts paying (m). 0.051 = sole width +
        # 5 mm (the visual-overlap guard); the hardware's feet BIND when they
        # meet mid-step (Tom, 2026-09-07/08), so a wider margin keeps a
        # swing foot clear of the stance foot earlier.
        w_foot_cross: float = 0.0,    # penalty when the soles' torso-frame
        # lateral separation closes below sole width + 5 mm -- the leg
        # meshes don't self-collide, so only the reward keeps the feet from
        # visually occupying the same space (video review 2026-07-20)
        sway_vy: float = 0.12,        # sway-command vy amplitude (m/s)
        # -- knee-high marching (2026-08-01; mirrored in walker_env.py) -------
        march_mix: float = 0.0,       # fraction of ext_cmd command draws that
        # are a KNEE-HIGH march: marching in place with alternating
        # exaggerated knee lifts, the swing sole raised to the height of the
        # opposite knee (~2x the normal swing clearance) at zero net
        # translation. Encoded entirely through the EXISTING 7 command
        # channels (vx=vy=wz=0, the lift channel c4 alternating on the gait
        # clock, c6 raising the swing-foot target to knee height) -- the obs
        # contract is untouched. Carved out as an extra slice after the
        # ext_mix slices, so 0.0 leaves the draw bit-identical.
        w_knee_high: float = 0.0,     # knee-high clearance kernel weight: the
        # FRACTION of the commanded swing height (lift_height + c6) the swing
        # sole reaches, on the correct one-foot contact pattern. Linear, not
        # Gaussian: a 10 cm target must still pay gradient from a 2 cm lift.
        march_hz: float = 0.0,        # COMMANDED march cadence, full L/R
        # cycles per second. 0.0 (default) = alternate on the gait clock and
        # inherit its 1.25-1.75 Hz draw (0.29-0.40 s per lift -- a sprinter's
        # march; 165 ms of upswing to knee height is the trainability risk).
        # A positive march_hz pins a deliberate cadence (1.0 -> 0.5 s per
        # lift) and DECOUPLES the march from the gait clock: the gait-clock
        # obs channel keeps running at its own rate, so the policy must learn
        # that during a march the swing timing is commanded, not clocked.
        walk_submix: tuple = (0.15, 0.15),    # of walk commands: (backward,
        # sidestep) fractions; remainder walks forward with the turning draw
        turn_emph: bool = False,    # sustained-turn emphasis (loco_v5t): 85%
                                    # of forward walks turn, |wz| floored 0.25
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
        w_contact_sched: float = 0.0,   # +-w per foot for matching the
        # clock's contact SCHEDULE (stance when sin is below sched_duty for
        # the left, above -sched_duty for the right). w_feet_phase only
        # shapes swing HEIGHT -- a policy can score on it while stepping
        # off-rhythm (measured 2026-08-17: step length 12+-6 cm, touchdown
        # phase sd 0.27 of the cycle, the "limp"). This term pays for the
        # timing itself.
        sched_duty: float = 0.4,    # stance window edge in sin units:
        # stance scheduled while sin(phase) < duty -> ~63% stance / foot,
        # leaving a double-support band at each crossing
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
        mimic_crouch_gate_knee: bool = False,  # 2026-09-10: KNEE-ONLY crouch
        # gate. mimic_crouch_gate switches the whole mimic term off while
        # cmd[3] < 0.97 -- and the 09-08 rhythm audit showed the term carries
        # leg-swing TIMING (v35mimic: 99%/CV 0.08 -> 93%/0.15). This variant
        # keeps the hip pitch/roll (swing/phase) components paying during a
        # crouch and zeroes only the knee + ankle components (the straight-
        # knee charge, and the ankle ref that is derived from that knee).
        mimic_crouch_gate: bool = False,  # 2026-09-08: the mimic reference at
        # zero velocity IS the standing pose (straight knees) whatever cmd[3]
        # says, so with mimic_knee_w 4 the term charged for the knee bend a
        # crouch needs -- squat_reps 0/8 in every student of the line, crouch
        # share 0.06 -> 0.20 (v34crouch) changed nothing. When set, the mimic
        # term is off while a crouch is commanded (cmd[3] < 0.97).
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
        w_foot_under: float = 0.0,  # while lifted: keep the RAISED foot
                                    # horizontally under its commanded spot
                                    # (tight sigma) -- clearance must come
                                    # from KNEE flexion, not a leg swung out
                                    # (user knee-articulation feedback)
        w_up_vel: float = 0.0,      # while down: pay positive root vertical
                                    # velocity (momentum-friendly getup_v5 --
                                    # the quasi-static reference is unreachable
                                    # from sit, see scripted_getup.py)
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
        # Camera is bolted at GP_MOUNT_X, off the centreline. Default 0.0 keeps
        # pre-2026-08-06 runs' plants byte-identical.
        payload_cg_x: float = 0.0,
        # -- observations ---------------------------------------------------------
        imu_obs: bool = True,
        imu_noise: float = 1.0,
        gyro_gain_range: tuple | None = None,   # per-episode gyro obs gain DR
        gyro_delay_max: int = 0,                # per-episode gyro obs delay, ticks
        act_delay_max: int = 0,                 # per-episode servo dead time, ticks
        init_pose_deg: float | None = None,     # start-pose jitter half-width per
                                                # joint (deg). None keeps the
                                                # historical +-0.03 rad (1.7 deg).
                                                # Tom 2026-09-06: "randomize the
                                                # starting foot position within
                                                # natural play limits" -- GUI arms
                                                # start from wherever the servos
                                                # were left. When set, the served
                                                # target and lag state also start
                                                # AT the jittered pose (firmware
                                                # reseeds the shaper from measured q).
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
        # -- terrain mosaic (loco_v7knee, 2026-07-29) ----------------------------
        terrain: bool = False,         # replace the plane with the shared
        # terrain_mosaic.npz heightfield (0-20 mm tiled roughness); every
        # episode spawns at a random spot, so per-episode roughness varies
        # without model batching. Height/clearance measured vs LOCAL ground.
        mimic_knee_w: float = 1.0,     # per-joint mimic weighting: knee error
        # scaled by this in the imitation kernel (the lump-sum kernel let the
        # hips+ankles satisfy it while the knee stayed jammed at +1 deg)
        # -- servo tick quantization (SIL boundary realism, 2026-07-31) --------
        # Puts the STS3215 encoder/bus quantizer inside training: joint
        # position/velocity OBSERVATIONS and the COMMANDED target angles are
        # rounded to the servo tick grid (see the _TICK_* constants). IMU
        # channels and the physics state are untouched. Arithmetically
        # identical to walker_env's (parity gate 2i); default False = old runs
        # bit-exact.
        quantize_ticks: bool = False,
        # -- crouch-command variation (SIL finding #2, docs/sil-harness.md) ----
        # cmd[3] was frozen at exactly 1.0 through every run, collapsing its
        # obs-normalizer std to ~1e-6 -- while the firmware feeds that channel
        # battguard's crouch(), which ramps below 1.0 on a sagging pack.
        # Per-episode uniform draw, substituted wherever a SAMPLED command
        # would have used the frozen 1.0 (an ext_mix crouch command keeps its
        # own crouch_range draw). (1.0, 1.0) = legacy, consumes no RNG.
        cmd_crouch_range: tuple = (1.0, 1.0),
    ):
        if getup and ext_cmd:
            raise ValueError("getup and ext_cmd are separate objectives")
        # recovery episodes rest the fallen robot on its CAD hulls -- both
        # engines must collide meshes with the floor (same rule as getup)
        self._terrain_spec = None
        self._hf_field = None
        if terrain:
            _tm = np.load(os.path.join(os.path.dirname(xml_path),
                                       "terrain_mosaic.npz"))
            self._terrain_spec = {k: float(_tm[k]) for k in
                                  ("nrow", "ncol", "rx", "ry", "cx",
                                   "max_amp")}
            self._hf_field = jp.asarray(_tm["field"], dtype=jp.float64)
        self.mj_model = _prep_model(
            xml_path, payload_mass > 0 or payload_dr or payload_max is not None,
            servo_joint_damping,
            mesh_floor=getup or (ext_cmd and recover_mix > 0),
            payload_cg_z=payload_cg_z, payload_cg_x=payload_cg_x,
            terrain_spec=self._terrain_spec)
        if terrain:
            self.mj_model.hfield_data[:] = (
                np.asarray(_tm["field"]).ravel() / self._terrain_spec["max_amp"])
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
        # hip yaw: present on v3yaw+ plants, None on the 8-DOF ones
        _ly, _ry = self._legL.get("hip_yaw"), self._legR.get("hip_yaw")
        self._i_yaw = (None if _ly is None or _ry is None
                       else jp.array([_ly, _ry]))
        self.terrain = terrain
        self.quantize_ticks = bool(quantize_ticks)
        self.cmd_crouch_range = tuple(cmd_crouch_range)
        self._crouch_dr = (self.cmd_crouch_range != (1.0, 1.0))
        if self._crouch_dr and not ext_cmd:
            raise ValueError("cmd_crouch_range requires ext_cmd "
                             "(cmd[3] only exists in the 7-channel layout)")
        if float(march_mix) > 0.0 and not ext_cmd:
            raise ValueError("march_mix requires ext_cmd (the knee-high "
                             "march is expressed through c4/c6)")
        # per-joint encoder direction (nominal calibration -- see _TICK_DIR)
        self._tick_dir = jp.full((self._nq_act,), _TICK_DIR)
        self.mimic_knee_w = mimic_knee_w
        _mw = np.ones(self._nq_act)
        _mw[np.asarray(self._i_knee)] = mimic_knee_w
        self._mimic_w = jp.asarray(_mw)
        # knee-only crouch gate: per-joint keep-mask with knee + ankle zeroed
        _keep = np.ones(self._nq_act)
        _keep[np.asarray(self._i_knee)] = 0.0
        _keep[np.asarray(self._i_ankle)] = 0.0
        self._mimic_keep = jp.asarray(_keep)
        if hip_flex_deg is not None:
            # training-range knob: moves the ctrlrange on a split plant, the
            # joint limit on the legacy ones (mirror of walker_env.py)
            for i in (self._jname2i["L_hip_pitch"], self._jname2i["R_hip_pitch"]):
                if m.actuator_ctrllimited[i]:
                    m.actuator_ctrlrange[i, 0] = -np.deg2rad(hip_flex_deg)
                else:
                    m.jnt_range[jnt[i], 0] = -np.deg2rad(hip_flex_deg)
            self.model = mjx.put_model(m)      # re-upload patched ranges
        if action_map not in ("legacy", "full"):
            raise ValueError(f"unknown action_map {action_map!r}")
        self.action_map = action_map
        self.hip_flex_deg = hip_flex_deg
        _plo, _phi = policy_range(m)
        self._lo = jp.asarray(_plo)
        self._hi = jp.asarray(_phi)
        self._default = jp.asarray(m.qpos0[self._jq0:self._jq1])
        self._scale = 0.5 * (self._hi - self._lo)
        self._qpos0 = jp.asarray(m.qpos0)

        self._torso_bid = m.body("torso").id
        self._sole_gids = (m.geom("L_sole").id, m.geom("R_sole").id)
        # Ground-contact geoms per foot -- mirror of BimoWalkerEnv._pad_gids.
        # v3yaw (2026-07-30): the sole box is a non-colliding reference geom and
        # the floor contact lives on four "<side>_pad_*" corner spheres (single
        # analytic contact points, which both engines agree on). Legacy plants
        # have no pad geoms, so the sole box itself is the contact geom.
        self._pad_gids = tuple(
            tuple(g for g in range(m.ngeom)
                  if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
                      or "").startswith(f"{side}_pad")) or (sole,)
            for side, sole in (("L", self._sole_gids[0]),
                               ("R", self._sole_gids[1])))
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
        self.w_still = w_still
        self.w_stand_com = w_stand_com
        self.w_hip_yaw = float(w_hip_yaw)
        self.w_crouch_track = float(w_crouch_track)
        self.crouch_track_sigma = float(crouch_track_sigma)
        self.crouch_release = bool(crouch_release)
        self.w_stand_zero = w_stand_zero
        self.w_stand_home = w_stand_home
        self.stand_com_sigma = stand_com_sigma
        self.speed_clock = speed_clock
        self.speed_clock_lo = speed_clock_lo
        self.speed_clock_hi = speed_clock_hi
        self.w_stand_knee = w_stand_knee
        self.stand_knee_target = stand_knee_target
        self.stand_knee_sigma = stand_knee_sigma
        self.clock_stand_freeze = clock_stand_freeze
        # torso-pitch MAGNITUDE hinge (mirrors walker_env): quadratic penalty
        # on |pitch| past a free deadband, pitch only (roll is untouched --
        # sidestep gaits legitimately roll). Default 0 -> the term is not
        # even traced.
        self.w_pitch_hinge = w_pitch_hinge
        self.pitch_deadband_deg = pitch_deadband_deg
        self._pitch_db = float(np.radians(pitch_deadband_deg))
        self.w_power = w_power
        self.domain_rand = domain_rand
        self.gain_range = gain_range
        self.servo_range = servo_range
        self.mass_range = mass_range
        self.friction_range = friction_range
        self.payload_max = payload_max
        # batch-level DR nominals (mirror walker_env._nom_*): the reference
        # mass/inertia/friction the per-env draws are scaled from. Captured
        # AFTER the payload body is welded and its mass set, so the payload
        # draw scales from the real camera's nominal inertia.
        self._nom_mass = jp.asarray(np.array(self.mj_model.body_mass))
        self._nom_inertia = jp.asarray(np.array(self.mj_model.body_inertia))
        self._nom_friction = jp.asarray(np.array(self.mj_model.geom_friction))
        self._floor_gid = self.mj_model.geom("floor").id
        self._payload_bid = (self.mj_model.body("payload").id
                             if (payload_mass > 0 or payload_dr
                                 or payload_max is not None) else None)
        self.latency_ms = latency_ms
        self.latency_ms_max = latency_ms_max
        self.latency_jitter_ms = latency_jitter_ms
        self.backlash_rad = float(np.deg2rad(backlash_deg))
        self.backlash_rad_max = (None if backlash_deg_max is None
                                 else float(np.deg2rad(backlash_deg_max)))
        self.tilt_max_deg = float(tilt_max_deg)
        self.act_lag_hz = float(act_lag_hz)
        self.act_lag_hz_max = (None if act_lag_hz_max is None
                               else float(act_lag_hz_max))
        self.zero_offset_deg = float(zero_offset_deg)
        self.zero_offset_rad = float(np.deg2rad(zero_offset_deg))
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
        self.turn_emph = turn_emph
        self.w_foot_cross = w_foot_cross
        self.foot_cross_sep = float(foot_cross_sep)
        self.sway_vy = sway_vy
        self.march_mix = float(march_mix)
        self.w_knee_high = w_knee_high
        self.march_hz = float(march_hz)
        self.gait_clock = gait_clock
        self.w_feet_phase = w_feet_phase
        self.swing_height = swing_height
        self.feet_phase_s2 = feet_phase_s2
        self.w_contact_sched = w_contact_sched
        self.sched_duty = sched_duty
        self.w_feet_slip = w_feet_slip
        self.w_orientation = w_orientation
        self.w_ang_vel_xy = w_ang_vel_xy
        self.w_pose = w_pose
        self.w_dof_limits = w_dof_limits
        self.push_kick = push_kick
        self.kick_range = kick_range
        self.obs_hist_len = max(1, int(obs_hist_len))
        self.w_mimic = w_mimic
        self.mimic_crouch_gate = bool(mimic_crouch_gate)
        self.mimic_crouch_gate_knee = bool(mimic_crouch_gate_knee)
        if self.mimic_crouch_gate and self.mimic_crouch_gate_knee:
            raise ValueError("mimic_crouch_gate and mimic_crouch_gate_knee "
                             "are alternatives; pick one")
        self.mimic_s2 = mimic_s2
        self.w_rise_dofvel = w_rise_dofvel
        self.w_rise_ref = w_rise_ref
        self.rise_secs = rise_secs
        # getup_v9: a stand only counts as "recovered" after this many
        # CONSECUTIVE standing steps (0.5 s) -- an instantaneous crossing
        # was farmable by ballistic bank starts
        self.stand_hold_n = max(1, round(0.5 / self.control_dt))
        self.rise_ref_s2 = rise_ref_s2
        if w_mimic > 0 and not gait_clock:
            raise ValueError("w_mimic requires gait_clock")
        self.w_symmetry = w_symmetry
        self.lift_clear = lift_clear
        self.w_com_stance = w_com_stance
        self.w_foot_under = w_foot_under
        self.w_up_vel = w_up_vel
        self.com_sigma = com_sigma
        self.w_heading = w_heading
        self.recover_mix = recover_mix
        if ext_cmd:
            # ext recovery slots reuse _fallen_data's start-state machinery
            self.getup_start_mix = recover_start_mix
        self.imu_obs = imu_obs
        self.imu_noise = imu_noise
        self.gyro_gain_range = (tuple(float(x) for x in gyro_gain_range)
                                if gyro_gain_range is not None else None)
        self.gyro_delay_max = int(gyro_delay_max)
        assert 0 <= self.gyro_delay_max <= 2, "gyro_hist holds 3 ticks"
        self.act_delay_max = int(act_delay_max)
        assert 0 <= self.act_delay_max <= 10, "act_delay_max in ticks (20 ms each)"
        self.init_pose_deg = (None if init_pose_deg is None
                              else float(init_pose_deg))
        self._init_q_rad = (0.03 if self.init_pose_deg is None
                            else float(np.radians(self.init_pose_deg)))
        self.getup = getup
        self.getup_settle = max(1, round(getup_settle_s / self.sim_dt))
        self.w_recover_h = w_recover_h
        self.w_recover_up = w_recover_up
        self.stand_bonus = stand_bonus
        if not ext_cmd:
            # BUG FOUND 2026-07-25: this assignment used to run UNCONDITIONALLY
            # and clobbered the ext-recovery recover_start_mix passthrough set
            # above -- getup_v3/v4/v5 all trained from PURE RAGDOLL starts, the
            # reverse-curriculum mix silently ignored (the CLI arg landed in
            # config.json but never in the env)
            self.getup_start_mix = getup_start_mix
        # getup_v6/v7: RSI state bank (qpos+qvel WITH momentum, no settle).
        # v6 held near-catch rock states; v7 holds states along the winnable
        # kneel-rise corridor (harvest_rise_states.py). Optional per-row "t0"
        # aligns each state with its rise-reference phase.
        _cs = os.path.join(os.path.dirname(xml_path), "getup_catch_states.npz")
        if os.path.exists(_cs):
            _z = np.load(_cs)
            self._catch_qpos = jp.asarray(_z["qpos"], dtype=jp.float32)
            self._catch_qvel = jp.asarray(_z["qvel"], dtype=jp.float32)
            self._catch_t0 = (jp.asarray(_z["t0"], dtype=jp.float32)
                              if "t0" in _z.files else None)
        else:
            self._catch_qpos = None
            self._catch_t0 = None

        self.action_size = self._nq_act
        # per-joint blocks (qpos, qvel, prev_action) scale with the joint
        # count; the rest is fixed. 8-DOF: 38 legacy / 43 ext; 10-DOF hip-yaw:
        # 44 legacy / 49 ext. obs history stacks obs_frame x hist (newest first)
        self.obs_frame = (3 * self._nq_act + 3 + 3 + 3 + 1 + 2
                          + (7 if ext_cmd else 2))
        self.obs_size = self.obs_frame * self.obs_hist_len
        # foot BODY ids for slip velocities (cvel linear part)
        self._foot_bids = (m.body("L_foot").id, m.body("R_foot").id)
        # soft joint limits (90% of range) for the dof-limit penalty. POLICY
        # range, not the mechanical stop -- see walker_env.py's mirror.
        _mid = 0.5 * (_plo + _phi)
        _half = 0.5 * (_phi - _plo)
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
        # KNEE-HIGH reference, MEASURED on the plant: world z of the knee
        # joint anchor in the standing pose (0.107 m on bimo_biped_v3yaw.xml
        # against a 0.003 m sole center -> 0.104 m of knee-high clearance,
        # 1.7x the 0.06 m lift_height). Mirrored in sim/walker_env.py.
        _knee_jid = m.joint(self._act_names[self._legL["knee"]]).id
        self._knee_z0 = float(d0.xanchor[_knee_jid][2])
        self.march_clear = max(self._knee_z0 - self._sole_z0, 0.0)
        # commanded c6 that puts the swing-foot TARGET at knee height
        self._march_dz = max(self.march_clear - self.lift_height, 0.0)
        # Fixed-cadence march clock (march_hz > 0) in WHOLE control steps.
        # th_m = 2*pi*march_hz*step_i*control_dt is the definition, but
        # evaluating the c4 sign flip as sin(th_m) >= 0 puts the flip exactly
        # on a zero crossing, where float32 (here, training) and float64 (the
        # CPU referee) land on OPPOSITE sides -- a whole-leg command
        # disagreement at every flip. The half cycle is therefore rounded to
        # an integer step count once, and the flip is an exact integer
        # compare in both engines; the |sin| envelope is evaluated on the
        # WRAPPED step count so its float argument stays inside [0, 2*pi).
        # Mirrored in sim/walker_env.py.
        self._march_half = (max(1, int(round(0.5 / (self.march_hz
                                                    * self.control_dt))))
                            if self.march_hz > 0.0 else 0)
        self._march_period = 2 * self._march_half
        self._metric_keys = self._METRIC_KEYS + (
            ("height_err", "foot_err", "foot_clear", "foot_sep", "lift_ok",
             "vy_body", "recovered", "com_stance", "head_err")
            if ext_cmd else ())

    # -- pieces ---------------------------------------------------------------
    def _sample_cmd(self, rng: jax.Array, step_i: jax.Array,
                    cmd_crouch=1.0):
        """(cmd, cmd_next, traj_on): stand / pivot-in-place / walk mix, exactly
        the CPU distribution (draws are made unconditionally -- fixed RNG
        shape). traj_on flags an ext_cmd air-circle command (c5/c6 evolve).
        cmd_crouch: this episode's crouch-channel draw (see _apply_crouch)."""
        if self.cmd_fixed is not None:
            cmd = jp.asarray(self.cmd_fixed, dtype=jp.float32)
            want = 7 if self.ext_cmd else 2
            if cmd.shape[0] != want:
                raise ValueError(f"cmd_fixed needs {want} channels")
            if self.ext_cmd:
                cmd = self._apply_crouch(cmd, cmd_crouch)
            return cmd, jp.asarray(10 ** 9, dtype=jp.int32), jp.zeros(())
        if self.ext_cmd:
            return self._sample_cmd_ext(rng, step_i, cmd_crouch)
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

    def _sample_cmd_ext(self, rng: jax.Array, step_i: jax.Array,
                        cmd_crouch=1.0):
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
        t8 = t7 + self.march_mix          # knee-high march slice
        m_crouch = (u >= t1) & (u < t2)
        m_bal = (u >= t2) & (u < t3)
        m_traj = (u >= t3) & (u < t4)
        m_piv = (u >= t4) & (u < t5)
        m_march = (u >= t5) & (u < t6)
        m_sway = (u >= t6) & (u < t7)
        m_kmarch = (u >= t7) & (u < t8)
        m_walk = u >= t8
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
        wz_raw = jax.random.uniform(rs[9], minval=-self.cmd_w_range,
                                    maxval=self.cmd_w_range)
        if self.turn_emph:
            # loco_v5t: sustained-turn emphasis -- 85% of forward walks turn,
            # magnitude floored at 0.25 rad/s (uniform-over-range made hard
            # turn-while-walk rare; circle_return was the weak scenario)
            wz_raw = jp.sign(wz_raw) * (0.25 + jp.abs(wz_raw)
                                        * (self.cmd_w_range - 0.25)
                                        / self.cmd_w_range)
        wz_f = jp.where(
            jax.random.uniform(rs[8]) < (0.85 if self.turn_emph else 0.6),
            wz_raw, 0.0)
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
        cmd = self._apply_crouch(cmd, cmd_crouch)
        hold = jax.random.uniform(rs[12], minval=self.cmd_resample_s[0],
                                  maxval=self.cmd_resample_s[1])
        # trajectory mode code: 0 off, 1 foot-circle, 2 march, 3 sway,
        # 4 knee-high march
        mode = (m_traj.astype(jp.float32) * 1.0
                + m_march.astype(jp.float32) * 2.0
                + m_sway.astype(jp.float32) * 3.0
                + m_kmarch.astype(jp.float32) * 4.0)
        return (cmd, step_i + (hold / self.control_dt).astype(jp.int32),
                mode)

    def _gz(self, x, y):
        """Local ground height under world (x, y): bilinear on the mosaic,
        0.0 on the flat plane or off the field. Mirrors walker_env._ground_z
        exactly (parity-gated); the grid is float64 meters from the npz."""
        if self._hf_field is None:
            return jp.zeros(())
        sp = self._terrain_spec
        nrow, ncol = int(sp["nrow"]), int(sp["ncol"])
        c = (x - (sp["cx"] - sp["rx"])) / (2 * sp["rx"]) * (ncol - 1)
        r = (y + sp["ry"]) / (2 * sp["ry"]) * (nrow - 1)
        inb = ((c >= 0.0) & (c <= ncol - 1) & (r >= 0.0) & (r <= nrow - 1))
        c = jp.clip(c, 0.0, ncol - 1)
        r = jp.clip(r, 0.0, nrow - 1)
        c0 = jp.clip(jp.floor(c).astype(jp.int32), 0, ncol - 1)
        r0 = jp.clip(jp.floor(r).astype(jp.int32), 0, nrow - 1)
        c1 = jp.minimum(c0 + 1, ncol - 1)
        r1 = jp.minimum(r0 + 1, nrow - 1)
        fc, fr = c - c0, r - r0
        g = self._hf_field
        h = ((1 - fr) * ((1 - fc) * g[r0, c0] + fc * g[r0, c1])
             + fr * ((1 - fc) * g[r1, c0] + fc * g[r1, c1]))
        return jp.where(inb, h, 0.0)

    def _rise_ref(self, t_s):
        """Scripted joint-space rise for recovery episodes: piecewise-linear
        through kneel -> hips-over-knees -> jackknife -> stand over rise_secs,
        then holds the standing pose. getup_v7: the old tuck->plant->deep-squat
        path is INFEASIBLE -- at max hip fold + knee flexion the CoM sits 9 cm
        behind the foot centers, so the deep squat tips backward under any
        extension (open-loop study 2026-07-26) and the reference was pulling
        the policy into that trap (v6 froze just short of the tip-over). The
        kneel corridor keeps weight on the shins+insteps support polygon --
        the one ground->stand path the v6 policy already climbs 3/3.
        Time-based phase from episode start (recovery episodes begin DOWN at
        t=0 with the command pinned to stand). Symmetric L/R, roll/yaw at
        default. Mirrored EXACTLY in walker_env.py."""
        d = self._default
        # stage targets (hip_pitch, knee, ankle) inside the joint limits
        # (-1.92..1.05 / -1.66..0.087 / +-0.70)
        stages = jp.array([
            [-1.92, -1.62, -0.55],  # ball/child's pose: shins+insteps down
            [-0.20, -1.62, -0.60],  # high-kneel: torso vertical on the knees
                                    # (statically stable at h 0.23 -- 71% up)
            [-0.80, -0.80, 0.10],   # knees extend, hips flex to keep the CoM
                                    # forward while the soles plant
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

    def _phase_cmd(self, cmd, recover_slot, recovered, rise_t0, step_i):
        """getup_v4: expose the rise-schedule phase to the policy through the
        foot_dx command channel c5 (zero on recovery episodes otherwise) --
        v3 asked the policy to track a time-indexed reference with no clock
        anywhere in the obs. Ramps 0..1 over the scripted rise, holds 1.0
        once recovered. Mirrored EXACTLY in walker_env.py."""
        phase = jp.clip((rise_t0 + step_i.astype(jp.float32) * self.control_dt)
                        / self.rise_secs, 0.0, 1.0)
        phase = jp.maximum(phase, recovered)
        return jp.where(recover_slot > 0, cmd.at[5].set(phase), cmd)

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
        for gids in self._pad_gids:
            mine = jp.zeros_like(hit)
            for gid in gids:
                mine |= (c.geom[:, 0] == gid) | (c.geom[:, 1] == gid)
            out.append(jp.any(hit & mine))
        return jp.stack(out)

    # -- servo tick quantization (mirror of walker_env's, parity-gated) --------
    def _quant_angle(self, rad):
        """rad -> encoder tick (round-half-to-even, clipped 0..4095) -> rad.
        jp.round is round-half-to-even, the same rule as np.rint (and
        lrintf()'s default mode) -- that is what makes gate 2i bit-identical
        rather than merely close."""
        ticks = jp.clip(_TICK_ZERO + jp.round(rad / _TICK_RAD * self._tick_dir),
                        0.0, float(_TICK_MAX))
        return (ticks - _TICK_ZERO) * _TICK_RAD * self._tick_dir

    def _quant_vel(self, rad_s):
        """rad/s -> integer reg-58 steps/s -> rad/s (sign-magnitude wire word,
        15-bit magnitude -> +/-32767 steps/s)."""
        steps = jp.clip(jp.round(rad_s / _TICK_RAD * self._tick_dir),
                        -float(_TICK_VEL_MAX), float(_TICK_VEL_MAX))
        return steps * _TICK_RAD * self._tick_dir

    def _apply_crouch(self, cmd, cmd_crouch):
        """Replace a FROZEN full-height crouch channel with this episode's
        draw (SIL finding #2). Only the exactly-1.0 case is replaced, so an
        ext_mix crouch command keeps its own crouch_range draw. Mirror of
        walker_env._apply_crouch_draw."""
        if not self._crouch_dr:
            return cmd
        # explicit cast: cmd is float32 while the draw is the jnp default
        # (float64 under the parity suite's x64), and an implicit downcast
        # inside .at[].set is a deprecated-and-soon-an-error scatter
        return cmd.at[3].set(jp.where(cmd[3] == 1.0, cmd_crouch,
                                      cmd[3]).astype(cmd.dtype))

    def _obs(self, data, prev_action, cmd, step_i, imu_R, imu_bias,
             rng: jax.Array, gait_phase=None, model=None,
             zero_off=None, gyro_hist=None, gyro_delay=None,
             gyro_gain=None) -> jax.Array:
        up = data.sensordata[self._up_adr:self._up_adr + 3]
        if self.tilt_max_deg > 0.0:
            # what an accelerometer reads: torso up against GRAVITY, not
            # world z (see _grav_rot)
            up = _grav_rot(self.model if model is None else model) @ up
        gyro = data.qvel[3:6]
        if gyro_hist is not None:
            # gyro obs DR: read the rate gyro_delay ticks back, scaled
            gyro = jp.take(gyro_hist, gyro_delay, axis=0)
        if gyro_gain is not None:
            gyro = gyro * gyro_gain
        linvel = data.qvel[0:3]
        height = data.qpos[2] - self._gz(data.qpos[0], data.qpos[1])
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
        q_j = data.qpos[self._jq0:self._jq1]
        if zero_off is not None:
            # calibration-error DR: the encoder's zero sits zero_off away from
            # the policy's frame, so the reported angle is physical - offset
            q_j = q_j - zero_off
        dq_j = data.qvel[self._jv0:self._jv1]
        if self.quantize_ticks:
            # servo-side view: what a SYNC READ can actually report (encoder
            # ticks + reg-58 steps/s). The physics state is untouched.
            q_j = self._quant_angle(q_j)
            dq_j = self._quant_vel(dq_j)
        return jp.concatenate([
            q_j,
            dq_j,
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
        if self.zero_offset_rad > 0.0:
            zero_off = jax.random.uniform(jax.random.fold_in(r_lash, 7),
                                          (self._nq_act,),
                                          minval=-self.zero_offset_rad,
                                          maxval=self.zero_offset_rad)
        else:
            zero_off = jp.zeros((self._nq_act,), dtype=jp.float32)
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
        if self._crouch_dr:
            # per-episode crouch-command draw (SIL finding #2). fold_in rather
            # than widening the split above, so every existing per-episode
            # draw keeps its exact stream when the feature is off (the CPU
            # mirror likewise consumes no RNG at the default).
            cmd_crouch = jax.random.uniform(
                jax.random.fold_in(rng, 3215),
                minval=self.cmd_crouch_range[0],
                maxval=self.cmd_crouch_range[1])
        else:
            cmd_crouch = jp.ones(())
        if self.act_lag_hz_max is not None:
            # fold_in like cmd_crouch: existing draws keep their exact
            # streams when the feature is off
            act_lag = jax.random.uniform(
                jax.random.fold_in(rng, 831),
                minval=self.act_lag_hz, maxval=self.act_lag_hz_max)
        else:
            act_lag = jp.asarray(self.act_lag_hz, dtype=jp.float32)
        return servo, lat_ms, lash, imu_R, imu_bias, cmd_crouch, act_lag, zero_off

    def _draw_gyro(self, rng: jax.Array):
        """Per-episode gyro obs gain + delay. fold_in like cmd_crouch so every
        existing draw keeps its exact stream when the feature is off."""
        if self.gyro_gain_range is not None:
            gyro_gain = jax.random.uniform(
                jax.random.fold_in(rng, 4471),
                minval=self.gyro_gain_range[0], maxval=self.gyro_gain_range[1])
        else:
            gyro_gain = jp.ones(())
        if self.gyro_delay_max > 0:
            gyro_delay = jax.random.randint(
                jax.random.fold_in(rng, 4472), (), 0, self.gyro_delay_max + 1)
        else:
            gyro_delay = jp.zeros((), dtype=jp.int32)
        return gyro_gain, gyro_delay

    @staticmethod
    def _gyro_hist0(data):
        return jp.tile(data.qvel[3:6][None, :], (3, 1))

    def _draw_act_delay(self, rng: jax.Array):
        """Per-episode servo dead time in control ticks (fold_in 4473 so the
        other draws keep their streams). 0 when the feature is off."""
        if self.act_delay_max > 0:
            return jax.random.randint(
                jax.random.fold_in(rng, 4473), (), 0, self.act_delay_max + 1)
        return jp.zeros((), dtype=jp.int32)

    def _act_hist0(self):
        # before the policy has acted the served target is the home pose
        return jp.tile(self._default.astype(jp.float32)[None, :],
                       (self.act_delay_max + 1, 1))

    _METRIC_KEYS = ("power_w", "vx_body", "wz", "height", "up_z", "fell",
                    "track_v_err", "track_w_err", "standing")

    def _settle(self, qpos, n, ctrl=None, model=None):
        # ctrl: servo targets HELD during the settle. Default = stand pose --
        # correct for ragdoll (limp-ish drop) but WRONG for staged poses:
        # getup_v7 found the "kneel" start settling under stand-drive to
        # h 0.28 > the 0.275 recovered threshold, so every kneel episode
        # started recovered=1 and trained nothing. Staged starts must settle
        # holding their OWN pose.
        model = self.model if model is None else model
        data = mjx.make_data(model)
        data = data.replace(qpos=qpos,
                            ctrl=jp.zeros(self.mj_model.nu)
                            + (self._default if ctrl is None else ctrl))

        def fall(d, _):
            return mjx.step(model, d), None

        data, _ = jax.lax.scan(fall, data, None, length=n)
        return mjx.forward(model, data)

    def _fallen_data(self, r_q, model=None):
        """Settled start for get-up mode. Mix (getup_start_mix): ragdoll fall
        (random orientation + joints, dropped from 0.35 m) / upright kneel /
        feet-loaded deep squat -- the latter two are the reverse-curriculum
        seeds (learn the rise backward from near the goal)."""
        model = self.model if model is None else model
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
        mix = mix + (0.0,) * (5 - len(mix))   # (rag, kneel, squat, sit, catch)
        p_rag, p_kneel, p_squat, p_sit, p_catch = mix
        if p_rag >= 1.0:
            return self._settle(q_rag, self.getup_settle, model=model), jp.zeros(())
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
        # staged poses settle 1.0 s: the kneel bounces off the shins to
        # h 0.277 (2 mm ABOVE the recovered threshold -> instant-recovered
        # episode) at 0.25 s before drooping to its true 0.23 static height
        d_rag = self._settle(q_rag, self.getup_settle, model=model)
        d_kneel = self._settle(q_kneel, 200, ctrl=j_kneel, model=model)
        d_squat = self._settle(q_squat, 200, ctrl=j_squat, model=model)
        d_sit = self._settle(q_sit, 200, ctrl=j_sit, model=model)
        u = jax.random.uniform(r_m)
        t_k = p_rag + p_kneel
        t_sq = t_k + p_squat
        t_st = t_sq + p_sit
        pick_kneel = (u >= p_rag) & (u < t_k)
        pick_squat = (u >= t_k) & (u < t_sq)
        pick_sit = (u >= t_sq) & (u < t_st)
        pick_catch = u >= t_st
        # bank start (v6 "catch"): a harvested state with its VELOCITY
        # (momentum preserved -- no settle). Row drawn per episode.
        t0_bank = jp.asarray(self.rise_secs / 3.0)
        if p_catch > 0 and self._catch_qpos is not None:
            r_c = jax.random.fold_in(r_m, 7)
            row = jax.random.randint(r_c, (), 0, self._catch_qpos.shape[0])
            if self._catch_t0 is not None:
                t0_bank = self._catch_t0[row]
            d_cat = mjx.make_data(model)
            d_cat = d_cat.replace(qpos=self._catch_qpos[row].astype(jp.float64)
                                  if d_cat.qpos.dtype == jp.float64
                                  else self._catch_qpos[row],
                                  qvel=self._catch_qvel[row].astype(
                                      d_cat.qvel.dtype),
                                  ctrl=jp.zeros(self.mj_model.nu)
                                  + self._default)
            d_cat = mjx.forward(model, d_cat)
        else:
            d_cat = None

        def sel(a, b, c, s, t):
            out = jp.where(pick_kneel, b, a)
            out = jp.where(pick_squat, c, out)
            out = jp.where(pick_sit, s, out)
            return out if t is None else jp.where(pick_catch, t, out)

        def sel4(a, b, c, s):
            return sel(a, b, c, s, None)

        # rise-phase offsets (getup_v7): kneel start == reference stage 1
        # (high-kneel) -> rise_secs/3; rag/sit enter at the ball (0); bank
        # rows carry their own harvested phase (fallback: stage 1)
        t0 = jp.where(pick_catch, t0_bank,
                      jp.where(pick_kneel, self.rise_secs / 3.0, 0.0))
        if d_cat is not None:
            data = jax.tree_util.tree_map(
                lambda a, b, c, s, t: sel(a, b, c, s, t),
                d_rag, d_kneel, d_squat, d_sit, d_cat)
        else:
            data = jax.tree_util.tree_map(sel4, d_rag, d_kneel, d_squat, d_sit)
        return data, t0

    def _terrain_spawn(self, qpos, rng):
        """Random spawn on the mosaic (per-episode roughness = per-episode
        location). 1 m margins keep episodes on the field; z rides the local
        ground. No-op on the flat plane. The trainer's cached-first-state
        auto-reset keeps one spawn per env slot -- 1024 slots is the
        diversity population, matching the batch-level DR convention."""
        if not self.terrain:
            return qpos
        r_sx, r_sy = jax.random.split(jax.random.fold_in(rng, 917))
        sx = jax.random.uniform(r_sx, minval=-0.5, maxval=4.0)
        sy = jax.random.uniform(r_sy, minval=-2.0, maxval=2.0)
        return (qpos.at[0].add(sx).at[1].add(sy)
                .at[2].add(self._gz(sx, sy)))

    # -- api -------------------------------------------------------------------
    def reset(self, rng: jax.Array, model=None) -> State:
        model = self.model if model is None else model
        rng, r_q, r_v, r_ep, r_cmd, r_obs = jax.random.split(rng, 6)
        recover_slot = jp.zeros(())
        rise_t0 = jp.zeros(())
        if self.getup:
            data, rise_t0 = self._fallen_data(r_q, model=model)
        elif self.ext_cmd and self.recover_mix > 0:
            # recovery slot draw: this env's episodes start from a settled
            # ragdoll fall (slot membership is FIXED across the trainer's
            # cached-first-state auto-resets -- the batch carries the mix)
            rng, r_mix = jax.random.split(rng)
            recover_slot = (jax.random.uniform(r_mix)
                            < self.recover_mix).astype(jp.float32)
            qpos = self._qpos0.at[self._jq0:self._jq1].add(
                jax.random.uniform(r_q, (self._nq_act,),
                                   minval=-self._init_q_rad,
                                   maxval=self._init_q_rad))
            qpos = self._terrain_spawn(qpos, rng)
            qvel = jax.random.uniform(r_v, (self.mj_model.nv,),
                                      minval=-0.02, maxval=0.02)
            d_up = mjx.make_data(model)
            d_up = d_up.replace(qpos=qpos, qvel=qvel,
                                ctrl=jp.zeros(self.mj_model.nu) + self._default)
            d_up = mjx.forward(model, d_up)
            d_dn, t0_dn = self._fallen_data(jax.random.fold_in(r_q, 1),
                                            model=model)

            def pick(a, b):
                return jp.where(recover_slot > 0, a, b)

            data = jax.tree_util.tree_map(pick, d_dn, d_up)
            rise_t0 = jp.where(recover_slot > 0, t0_dn, 0.0)
        else:
            qpos = self._qpos0.at[self._jq0:self._jq1].add(
                jax.random.uniform(r_q, (self._nq_act,),
                                   minval=-self._init_q_rad,
                                   maxval=self._init_q_rad))
            qpos = self._terrain_spawn(qpos, rng)
            qvel = jax.random.uniform(r_v, (self.mj_model.nv,),
                                      minval=-0.02, maxval=0.02)
            data = mjx.make_data(model)
            data = data.replace(qpos=qpos, qvel=qvel,
                                ctrl=jp.zeros(self.mj_model.nu) + self._default)
            data = mjx.forward(model, data)
        (servo, lat_ms, lash, imu_R, imu_bias,
         cmd_crouch, act_lag, zero_off) = self._draw_episode(r_ep)
        gyro_gain, gyro_delay = self._draw_gyro(r_ep)
        gyro_hist = self._gyro_hist0(data)
        act_delay = self._draw_act_delay(r_ep)
        if self.init_pose_deg is None:
            act_hist = self._act_hist0()
            lag_y0 = jp.tile(self._default.astype(jp.float32), (3, 1))
        else:
            q_init = data.qpos[self._jq0:self._jq1].astype(jp.float32)
            act_hist = jp.tile(q_init[None, :], (self.act_delay_max + 1, 1))
            lag_y0 = jp.tile(q_init, (3, 1))
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next, traj_on = self._sample_cmd(r_cmd, step_i, cmd_crouch)
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
        if self.ext_cmd and self.w_rise_ref:
            cmd = self._phase_cmd(cmd, recover_slot, 1.0 - recover_slot,
                                  rise_t0, step_i)
        frame = self._obs(data, prev_action, cmd, step_i, imu_R, imu_bias,
                          r_obs, gait_phase, model, zero_off=zero_off,
                          gyro_hist=gyro_hist, gyro_delay=gyro_delay,
                          gyro_gain=gyro_gain)
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
                     stand_streak=jp.zeros(()),
                     best_h=data.qpos[2] - self._gz(data.qpos[0],
                                                    data.qpos[1]),
                     head_ref=_quat_yaw(data.qpos[3:7]), rise_t0=rise_t0,
                     cmd_crouch=cmd_crouch,
                     servo=servo, lat_ms=lat_ms, lash=lash, zero_off=zero_off,
                     act_lag=act_lag,
                     lag_y=lag_y0,
                     act_hist=act_hist, act_delay=act_delay,
                     imu_R=imu_R, imu_bias=imu_bias,
                     gyro_hist=gyro_hist, gyro_delay=gyro_delay,
                     gyro_gain=gyro_gain, metrics=metrics)

    def reseed(self, first: State, rng: jax.Array, model=None) -> State:
        """Fresh episode that REUSES the cached first physics state (brax-style
        cheap auto-reset: no make_data/forward) but re-draws every per-episode
        quantity -- command, servo gains, latency, backlash, IMU error -- so
        episode-level DR diversity survives auto-resetting."""
        rng, r_ep, r_cmd, r_obs = jax.random.split(rng, 4)
        (servo, lat_ms, lash, imu_R, imu_bias,
         cmd_crouch, act_lag, zero_off) = self._draw_episode(r_ep)
        gyro_gain, gyro_delay = self._draw_gyro(r_ep)
        gyro_hist = self._gyro_hist0(first.data)
        act_delay = self._draw_act_delay(r_ep)
        act_hist = self._act_hist0()
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next, traj_on = self._sample_cmd(r_cmd, step_i, cmd_crouch)
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
        if self.ext_cmd and self.w_rise_ref:
            cmd = self._phase_cmd(cmd, first.recover_slot,
                                  1.0 - first.recover_slot, first.rise_t0,
                                  step_i)
        frame = self._obs(first.data, prev_action, cmd, step_i, imu_R,
                          imu_bias, r_obs, gait_phase, model, zero_off=zero_off,
                          gyro_hist=gyro_hist, gyro_delay=gyro_delay,
                          gyro_gain=gyro_gain)
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
            stand_streak=jp.zeros(()),
            best_h=first.data.qpos[2] - self._gz(first.data.qpos[0],
                                                 first.data.qpos[1]),
            head_ref=_quat_yaw(first.data.qpos[3:7]), cmd_crouch=cmd_crouch,
            servo=servo, lat_ms=lat_ms, lash=lash, zero_off=zero_off,
            act_lag=act_lag,
            lag_y=jp.tile(self._default.astype(jp.float32), (3, 1)),
            act_hist=act_hist, act_delay=act_delay,
            imu_R=imu_R, imu_bias=imu_bias,
            gyro_hist=gyro_hist, gyro_delay=gyro_delay, gyro_gain=gyro_gain,
            metrics=metrics)

    def step(self, state: State, action: jax.Array, model=None) -> State:
        m = self.model if model is None else model
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
        # servo dead time DR (see State.act_hist): the actuator serves the
        # target from act_delay ticks ago; row 0 is this tick's. Sits BEFORE
        # the lag cascade, as the real bus write precedes the servo's response.
        act_hist = jp.concatenate([target[None, :], state.act_hist[:-1]], axis=0)
        target = jp.take(act_hist, state.act_delay, axis=0)
        # action-chain lag (3 cascaded first-order stages, the firmware C2
        # shaper's structure) BEFORE quantization, matching the deploy path's
        # shaper -> angleToSteps order. Per-episode pole from _draw_episode;
        # a 0 Hz draw is a pass-through.
        if self.act_lag_hz > 0.0 or self.act_lag_hz_max is not None:
            k = 1.0 - jp.exp(-2.0 * jp.pi * state.act_lag * self.control_dt)
            k = jp.where(state.act_lag > 0.0, k, 1.0)
            y1 = state.lag_y[0] + k * (target - state.lag_y[0])
            y2 = state.lag_y[1] + k * (y1 - state.lag_y[1])
            y3 = state.lag_y[2] + k * (y2 - state.lag_y[2])
            lag_y = jp.stack([y1, y2, y3])
            target = y3
        else:
            lag_y = state.lag_y
        if self.quantize_ticks:
            # the firmware writes INTEGER goal ticks (SYNC WRITE): quantize
            # before the actuator path (latency, PD, backlash) sees the target
            target = self._quant_angle(target)
        # calibration-error DR (zero_offset_deg): the servo's zero is off from
        # the policy's frame, so the PHYSICAL target it serves is target + off
        # (the obs subtracts the same offset). Zeros when the feature is off.
        target = target + state.zero_off

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
        if self.tilt_max_deg > 0.0:
            # uprightness (reward + fall) against gravity, like the IMU
            up_z = jp.dot(_grav_up(m),
                          data.sensordata[self._up_adr:self._up_adr + 3])
        height = data.qpos[2] - self._gz(data.qpos[0], data.qpos[1])
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
        knee_frac = jp.zeros(())
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
            foot_clear = (sole_w[2] - self._gz(sole_w[0], sole_w[1])
                          - self._sole_z0)
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
            if self.w_foot_under:
                # tight horizontal kernel on the raised foot (sigma 0.03 vs
                # the loose foot_sigma): clearance gained by swinging the leg
                # out earns ~nothing; only a knee-flexion lift with the foot
                # under its commanded spot pays. Clearance-gated like the
                # CoM kernel so planted feet cannot farm it.
                d_xy2 = (rel[0] - tgt[0]) ** 2 + (rel[1] - tgt[1]) ** 2
                foot_under = (lifted.astype(jp.float32)
                              * jp.clip(foot_clear / self.lift_clear, 0.0, 1.0)
                              * jp.exp(-d_xy2 / (0.03 ** 2)))
            # knee-high clearance kernel (mirror of walker_env): fraction of
            # the COMMANDED swing height (lift_height + c6) the swing sole
            # actually reaches, on the correct one-foot contact pattern.
            # Linear rather than Gaussian so a knee-high (0.10 m) target
            # still pays gradient from a 2 cm lift -- the kernel-width
            # lesson, again. c6 = 0 under a plain balance lift, so this
            # degrades to "reach lift_height"; the knee-high march drives
            # c6 up to march_dz.
            tgt_clear = self.lift_height + state.cmd[6]
            knee_frac = (lifted.astype(jp.float32)
                         * (~con_swing).astype(jp.float32)
                         * con_stance.astype(jp.float32)
                         * jp.clip(foot_clear / jp.maximum(tgt_clear, 1e-6),
                                   0.0, 1.0))
            # feet-crossing guard: torso-frame lateral separation of the two
            # sole centers must stay >= sole width + 5 mm
            pL = data.geom_xpos[self._sole_gids[0]] - data.xpos[self._torso_bid]
            pR = data.geom_xpos[self._sole_gids[1]] - data.xpos[self._torso_bid]
            y_sep = jp.abs((-sth * pL[0] + cth * pL[1])
                           - (-sth * pR[0] + cth * pR[1]))
            cross_frac = jp.clip((self.foot_cross_sep - y_sep)
                                 / self.foot_cross_sep, 0.0, 1.0)
            # CoM-over-stance-foot kernel (user 2026-07-23): pay for keeping
            # the whole-robot CoM planted over the support sole while a lift
            # is commanded -- the knee-flexion lift (thigh vertical, shank
            # folds back) is then the cheapest compliant posture, and the
            # stick-the-leg-out lift (CG lurches forward) is expensive.
            com = data.subtree_com[self._torso_bid]
            stance_w = jp.where(is_r, data.geom_xpos[self._sole_gids[0]],
                                data.geom_xpos[self._sole_gids[1]])
            if self.tilt_max_deg > 0.0:
                com = _grav_proj_xy(m, com, stance_w[2])
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
                # getup_v8: relative height income while down (mirrors
                # walker_env; capped below the bleed) -- the one-way ratchet
                # alone made every parking height pay alike and v7 descended
                # from the high-kneel to park at h 0.14
                rec_primary = (200.0 * h_gain
                               + 0.6 * jp.minimum(height / self._nominal_h,
                                                  1.0)
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
        clock_freq = state.gait_freq
        if self.gait_clock:
            if self.speed_clock:
                v_planar = (jp.sqrt(cmd_v ** 2 + state.cmd[1] ** 2 + 1e-12)
                            if self.ext_cmd else jp.abs(cmd_v))
                clock_freq = clock_freq * speed_clock_scale(
                    v_planar, self.speed_clock_lo, self.speed_clock_hi)
            gait_phase = jp.mod(state.gait_phase + 2 * jp.pi * self.control_dt
                                * clock_freq + jp.pi,
                                2 * jp.pi) - jp.pi
            if self.clock_stand_freeze:
                # at a plain stand the clock HOLDS (user 2026-08-12, stand
                # shaking): sin/cos phase is a 1.25-1.75 Hz oscillator in
                # the obs that the policy otherwise has to actively ignore
                # to stand still -- w_still fights the symptom, this removes
                # the drive. Freeze is gated exactly like w_still: no
                # locomotion command, no lift (marches/balances keep their
                # clock). Mirrored in walker_env so referee obs match.
                plain_stand = ((~cmd_moving)
                               & (~lifted if self.ext_cmd else True))
                gait_phase = jp.where(plain_stand, state.gait_phase,
                                      gait_phase)
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
        if self.w_contact_sched:
            # cadence enforcement: each foot earns +w on-schedule / -w
            # off-schedule (sum shifted by -1 so all-matched pays +1w and
            # all-wrong -1w per step in w units). Gated like the other
            # gait shaping: moving commands only, so standing double
            # support is never punished.
            sched = contact_schedule(gait_phase, self.sched_duty)
            match = jp.sum((sched == con).astype(jp.float32))
            reward += jp.where(cmd_moving,
                               self.w_contact_sched * (match - 1.0), 0.0)
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
            q_ref = self._mimic_ref(state.cmd, gait_phase, clock_freq)
            dq = data.qpos[self._jq0:self._jq1] - q_ref
            mim_gate = 1.0
            mim_w = self._mimic_w
            if self.ext_cmd:
                mg = (~lifted) & (state.recovered > 0.5)
                if self.mimic_crouch_gate:
                    mg = mg & (state.cmd[3] >= 0.97)
                if self.mimic_crouch_gate_knee:
                    # crouching: knee + ankle components drop out, the hip
                    # swing/phase components keep paying (timing survives)
                    mim_w = jp.where(state.cmd[3] < 0.97,
                                     self._mimic_w * self._mimic_keep,
                                     self._mimic_w)
                mim_gate = mg.astype(jp.float32)
            reward += (self.w_mimic * mim_gate
                       * jp.exp(-jp.sum(mim_w * dq ** 2)
                                / self.mimic_s2))
        if self.w_rise_dofvel and self.ext_cmd and self.recover_mix > 0:
            # jerk control during the rise (HumanUP/utra lesson)
            reward -= (self.w_rise_dofvel
                       * jp.where(state.recovered < 0.5, 1.0, 0.0)
                       * jp.sum(qd_j ** 2))
        if self.w_rise_ref and self.ext_cmd and self.recover_mix > 0:
            # staged-rise reference while DOWN (getup_v3): dense guidance
            # toward the scripted tuck->plant->squat->stand trajectory
            q_rr = self._rise_ref(state.rise_t0
                                  + state.step_i.astype(jp.float32)
                                  * self.control_dt)
            dq_rr = data.qpos[self._jq0:self._jq1] - q_rr
            reward += (self.w_rise_ref
                       * jp.where(state.recovered < 0.5, 1.0, 0.0)
                       * jp.exp(-jp.sum(dq_rr ** 2) / self.rise_ref_s2))
        if self.w_up_vel and self.ext_cmd and self.recover_mix > 0:
            # getup_v5: momentum-friendly rise incentive -- positive root
            # vertical velocity pays while down (capped; the height ratchet
            # already prevents static-height farming)
            reward += (self.w_up_vel
                       * jp.where(state.recovered < 0.5, 1.0, 0.0)
                       * jp.clip(data.qvel[2], 0.0, 0.5))
        if self.w_foot_under and self.ext_cmd:
            reward += self.w_foot_under * foot_under
        if self.w_knee_high and self.ext_cmd:
            reward += self.w_knee_high * knee_frac

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
        if self.w_hip_yaw and self._i_yaw is not None:
            q_yaw = data.qpos[self._jq0:self._jq1][self._i_yaw]
            reward -= self.w_hip_yaw * jp.sum(q_yaw ** 2)
        if self.w_crouch_track and self.ext_cmd:
            # pays only while a crouch is commanded and only near the depth
            ct_gate = ((state.cmd[3] < 0.97) & (state.recovered > 0.5)
                       & (~lifted)).astype(jp.float32)
            ct_err = height - state.cmd[3] * self._nominal_h
            reward += (self.w_crouch_track * ct_gate
                       * jp.exp(-(ct_err / self.crouch_track_sigma) ** 2))
        if self.w_still:
            # stand stillness (user 2026-08-12: "focus on standing still --
            # way too much shaking"). Direct joint-velocity penalty, gated
            # to a plain stand: no locomotion command, no lift, upright.
            # action_rate/ang_vel_xy bound the SIZE of corrections; nothing
            # before this paid for their ABSENCE -- the policy idled in a
            # micro-stepping limit cycle (38.7 W standing vs 6.6 W released,
            # loco_v18b_mix referee). Quadratic, so disturbance recovery
            # (large dq for a moment) costs little vs perpetual dither.
            stand_gate = ((~cmd_moving)
                          & (~lifted if self.ext_cmd else True)
                          & (state.recovered > 0.5))
            if self.crouch_release and self.ext_cmd:
                stand_gate = stand_gate & (state.cmd[3] >= 0.97)
            reward -= (self.w_still
                       * jp.where(stand_gate, 1.0, 0.0)
                       * jp.sum(data.qvel[self._jv0:self._jv1] ** 2))
        if self.w_stand_zero:
            # zero command -> zero action -> home pose (see ctor note)
            # crouch is a COMMAND, not a stand: cmd[3] < 0.97 releases the pull
            # (2026-09-06: v30/v31home could not crouch -- this gate held them
            # at home while the slider asked for 0.87; same test as w_pose)
            sz_gate = ((~cmd_moving)
                       & ((~lifted) & (state.cmd[3] >= 0.97) if self.ext_cmd else True)
                       & (state.recovered > 0.5))
            reward -= (self.w_stand_zero
                       * jp.where(sz_gate, 1.0, 0.0)
                       * jp.mean(action ** 2))
        if self.w_stand_home:
            # rest = home, in joint units (see ctor note). Uses the target the
            # actuator serves this tick (post dead-time/lag), so the pull is
            # on the pose the servos are actually asked to hold.
            sh_gate = ((~cmd_moving)
                       & ((~lifted) & (state.cmd[3] >= 0.97) if self.ext_cmd else True)
                       & (state.recovered > 0.5))
            reward -= (self.w_stand_home
                       * jp.where(sh_gate, 1.0, 0.0)
                       * jp.sum(jp.abs(target - self._default)))
        if self.w_stand_com:
            # stand_off diagnosis 2026-08-19: the v21 line parks its standing
            # CoM 26-28 mm AFT of the midfoot point; with torque released,
            # ankle gravity moment (m*g*offset ~ 0.60 Nm at 28 mm) beats the
            # STS3215 backdrive friction estimate (0.35 Nm ~ 16 mm) and the
            # robot slowly topples. Policies at <= ~18 mm survive. Nothing
            # shaped the double-support stand CoM (w_com_stance is
            # single-support only) -- this kernel does, gated like w_still.
            sc_gate = ((~cmd_moving)
                       & (~lifted if self.ext_cmd else True)
                       & (state.recovered > 0.5))
            if self.crouch_release and self.ext_cmd:
                sc_gate = sc_gate & (state.cmd[3] >= 0.97)
            sc_mid = 0.5 * (data.geom_xpos[self._sole_gids[0]]
                            + data.geom_xpos[self._sole_gids[1]])
            sc_com = data.subtree_com[self._torso_bid]
            if self.tilt_max_deg > 0.0:
                sc_com = _grav_proj_xy(m, sc_com, sc_mid[2])
            sc_off2 = ((sc_com[0] - sc_mid[0]) ** 2
                       + (sc_com[1] - sc_mid[1]) ** 2)
            reward += (self.w_stand_com
                       * jp.where(sc_gate, 1.0, 0.0)
                       * jp.exp(-sc_off2 / self.stand_com_sigma ** 2))
        if self.w_stand_knee:
            # centering the CoM (w_stand_com) fixed the rigid backward
            # topple but exposed a second failure: from a dead-straight
            # knee the torque-off stand SAG-collapses (knees and ankles
            # fold together ~1.3 deg/s; v22fix stand_off 3/8). The pure-
            # physics sweep from v22fix's own cut pose says a +0.10 rad
            # knee bias flips it to a ~5 cm hold; the opposite direction
            # falls backward. Same gate as w_stand_com.
            sk_gate = ((~cmd_moving)
                       & (~lifted if self.ext_cmd else True)
                       & (state.recovered > 0.5))
            sk_q = data.qpos[self._jq0:self._jq1][self._i_knee]
            sk_kern = jp.mean(jp.exp(
                -((sk_q - self.stand_knee_target)
                  / self.stand_knee_sigma) ** 2))
            reward += (self.w_stand_knee
                       * jp.where(sk_gate, 1.0, 0.0) * sk_kern)
        if self.w_pitch_hinge:  # upright torso: |pitch| past a free deadband
            pitch = _quat_pitch(data.qpos[3:7])
            excess = jp.maximum(jp.abs(pitch) - self._pitch_db, 0.0)
            # locomotion only: while DOWN in a recovery episode a face-plant
            # pitch is the task, not a fault (mirrors walker_env's
            # `self._recovered and not self.getup` gate)
            hinge_gate = (0.0 if self.getup
                          else jp.where(state.recovered > 0.5, 1.0, 0.0))
            reward -= self.w_pitch_hinge * hinge_gate * excess * excess

        step_i = state.step_i + 1
        # ext mode: the fall floor tracks the commanded crouch (a commanded
        # 0.6-height crouch is 0.17 m -- below the legacy 0.18 fall line)
        fall_h = (self.fall_height * state.cmd[3] if self.ext_cmd
                  else self.fall_height)
        fell = (height < fall_h) | (up_z < self.fall_up_z)
        recovered = state.recovered
        stand_streak = state.stand_streak
        if self.ext_cmd and self.recover_mix > 0:
            # getup_v9: recovery episodes NEVER terminate on falls -- while
            # down the fall is the task, and post-recover a fall just loses
            # the standing income and the rise is re-practiced in the same
            # episode. (The old post-recover termination let ballistic bank
            # starts farm ratchet income and exit in 0.3 s.) recovered now
            # requires a HELD stand -- stand_hold_n consecutive standing
            # steps -- not an instantaneous height crossing mid-flight.
            fell = fell & (state.recover_slot < 0.5)
            stand_streak = jp.where(state.recover_slot > 0.5,
                                    jp.where(standing > 0.5,
                                             state.stand_streak + 1.0,
                                             jp.zeros(())),
                                    jp.zeros(()))
            recovered = jp.maximum(
                state.recovered,
                (stand_streak >= self.stand_hold_n).astype(jp.float32))
        if self.getup:
            done_flag = jp.zeros(())          # being down IS the task
        else:
            reward = reward - self.fall_cost * fell.astype(jp.float32)
            done_flag = fell.astype(jp.float32)

        # resample AFTER the reward graded the command the policy saw
        new_cmd, new_next, new_traj_on = self._sample_cmd(r_cmd, step_i,
                                                          state.cmd_crouch)
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
            # 0 off, 1 foot-circle, 2 march, 3 sway, 4 knee-high march);
            # scripted evals that pin commands via cmd_fixed keep mode 0 and
            # are untouched.
            th = (state.traj[2] + state.traj[1]
                  * step_i.astype(jp.float32) * self.control_dt)
            # knee-high march clock (mirror of walker_env). march_hz > 0 pins
            # a COMMANDED cadence, deliberately decoupled from the gait clock
            # (which keeps running at its own rate in the obs). march_hz == 0
            # falls back to the gait clock -- already advanced this step
            # above, and visible to the policy as sin/cos(phase) -- so the
            # alternation stays anticipatable and firmware can regenerate it
            # from its own gait clock; with neither, the per-episode traj
            # clock.
            if self.march_hz > 0.0:
                n_m = jp.mod(step_i, self._march_period)
                m_left = n_m < self._march_half
                th_m = (2 * jp.pi * n_m.astype(jp.float32)
                        / self._march_period)
            else:
                th_m = gait_phase if self.gait_clock else th
                m_left = jp.sin(th_m) >= 0
            circle = (cmd.at[5].set(state.traj[0] * jp.cos(th))
                      .at[6].set(state.traj[0] * jp.sin(th)))
            # march: alternate the lifted leg each half period; the foot
            # target bobs 0..radius above the nominal lift height
            march = (cmd.at[4].set(jp.where(jp.sin(th) >= 0, 1.0, -1.0))
                     .at[5].set(0.0)
                     .at[6].set(state.traj[0] * jp.abs(jp.sin(th))))
            # sway: standing lateral weight-shift via an oscillating vy
            sway = cmd.at[1].set(self.sway_vy * jp.sin(th))
            # knee-high march: vx/vy/wz are 0 by construction of the draw;
            # the LIFT channel alternates on the march clock with the same
            # left/right convention as w_feet_phase (left swings on the first
            # half cycle -> lift = -1), and c6 raises the swing-foot TARGET
            # from lift_height up to knee height over the swing.
            kmarch = (cmd.at[4].set(jp.where(m_left, -1.0, 1.0))
                      .at[5].set(0.0)
                      .at[6].set((self._march_dz * jp.abs(jp.sin(th_m))
                                  ).astype(cmd.dtype)))
            cmd = jp.where(traj_on == 1.0, circle, cmd)
            cmd = jp.where(traj_on == 2.0, march, cmd)
            cmd = jp.where(traj_on == 3.0, sway, cmd)
            cmd = jp.where(traj_on == 4.0, kmarch, cmd)

        if self.ext_cmd and self.w_rise_ref and self.recover_mix > 0:
            cmd = self._phase_cmd(cmd, state.recover_slot, recovered,
                                  state.rise_t0, step_i)
        gyro_hist = jp.concatenate([data.qvel[3:6][None, :],
                                    state.gyro_hist[:-1]], axis=0)
        frame = self._obs(data, action, cmd, step_i, state.imu_R,
                          state.imu_bias, r_obs, gait_phase, m, zero_off=state.zero_off,
                          gyro_hist=gyro_hist, gyro_delay=state.gyro_delay,
                          gyro_gain=state.gyro_gain)
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
                     recovered=recovered, stand_streak=stand_streak,
                     best_h=jp.maximum(state.best_h, height),
                     head_ref=head_ref, rise_t0=state.rise_t0,
                     cmd_crouch=state.cmd_crouch, servo=state.servo,
                     lat_ms=state.lat_ms, lash=state.lash, zero_off=state.zero_off,
                     act_lag=state.act_lag, lag_y=lag_y,
                     act_hist=act_hist, act_delay=state.act_delay,
                     imu_R=state.imu_R,
                     imu_bias=state.imu_bias,
                     gyro_hist=gyro_hist, gyro_delay=state.gyro_delay,
                     gyro_gain=state.gyro_gain, metrics=metrics)


def domain_randomize(model, rng: jax.Array, mass_range: float = 0.15,
                     friction_range: float = 0.4,
                     payload_max: float | None = None,
                     payload_bid: int | None = None,
                     floor_gid: int = 0, nom_mass=None, nom_inertia=None,
                     nom_friction=None, tilt_max_deg: float = 0.0):
    """Batch-level model DR (brax randomization_fn style): per-env body
    mass/inertia scale, floor friction scale, payload mass draw, and --
    when tilt_max_deg > 0 -- a per-env GRAVITY TILT. Returns
    (batched_model, in_axes) for vmapped training.

    The tilt is the un-level real world (user 2026-09-01: "the floor isn't
    even perfectly level" -- the bench desk measures ~3.5 deg off): rotating
    gravity by theta ~ U(0, tilt_max) about a random horizontal azimuth is,
    for a flat floor, exactly a floor inclined by theta. The IMU sees what
    the real one sees: a robot standing plumb to the SUPPORT reads a tilted
    up-vector against true gravity."""
    nbody = model.body_mass.shape[0]
    g0 = model.opt.gravity[2]           # -9.81 nominal; stays a jax scalar
    # (float() here breaks under jit: domain_randomize is traced inside
    # BatchedEnv.reset, where model leaves are abstract tracers.)

    @jax.vmap
    def rand(rng):
        r_m, r_f, r_p, r_t, r_a = jax.random.split(rng, 5)
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
        th = jax.random.uniform(r_t, maxval=np.deg2rad(tilt_max_deg))
        az = jax.random.uniform(r_a, maxval=2 * np.pi)
        gravity = jp.array([-g0 * jp.sin(th) * jp.cos(az),
                            -g0 * jp.sin(th) * jp.sin(az),
                            g0 * jp.cos(th)])
        return mass, inertia, friction, gravity

    mass, inertia, friction, gravity = rand(rng)
    in_axes = jax.tree_util.tree_map(lambda x: None, model)
    fields = {"body_mass": 0, "body_inertia": 0, "geom_friction": 0}
    values = {"body_mass": mass, "body_inertia": inertia,
              "geom_friction": friction}
    if tilt_max_deg > 0.0:
        fields["opt.gravity"] = 0
        values["opt.gravity"] = gravity
    in_axes = in_axes.tree_replace(fields)
    model = model.tree_replace(values)
    return model, in_axes
