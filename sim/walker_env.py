"""Stage-2a: Gymnasium environment for the Bimo-like biped (CPU MuJoCo).

Wraps sim/bimo_biped.xml as a standard RL env so a policy can learn to walk
forward. Kept deliberately framework-light (plain MuJoCo, no MJX) so it runs on
the Mac for fast iteration; the reward/obs design carries over to the MJX port
(Stage 2b) unchanged.

Env contract (Gymnasium >=1.0):
    obs, info              = env.reset(seed=...)
    obs, rew, term, trunc, info = env.step(action)

Action  : 8 target joint angles, normalized to [-1, 1] (mapped to each joint's
          physical range). Written to the position actuators.
Obs (36): joint qpos (8), joint qvel (8), torso up-vector (3), torso linear
          vel (3), torso angular vel (3), previous action (8), plus torso
          height (1) and a phase clock sin/cos (2)  -> stable, Markov-ish state.
Reward  : forward velocity + upright + alive - energy - action-rate,
          plus optional gait-quality terms (feet air-time, single-support,
          lateral/yaw drift, torso rate) that default to 0 so old runs reproduce.
Terminate: torso falls (height above local ground < 0.18 m) or tips over
          (up-vector z < 0.4). On flat ground local ground z == 0, so this is
          identical to the original world-z check.
Terrain : optional procedural heightfield (terrain_amplitude > 0). The flat
          plane in bimo_biped.xml is swapped for an hfield geom at model-load
          time; height data is re-randomized every reset (smoothed uniform
          noise with a flattened spawn pad). Default (0.0) keeps the original
          flat-plane model byte-for-byte.
Actuator: actuator_model="sts3215" replaces the ideal MJCF position servos
          with a PD torque controller clamped to the STS3215 DC-motor
          torque-speed envelope, scaled to supply_voltage (2S=7.4 V default).
          Default "ideal" reproduces every old run bit-exactly.
Dash    : dash=True adds the 2 m dash objective -- the episode succeeds
          (terminates with finish_bonus) only when torso x >= dash_distance
          AND the robot is still upright dash_hold seconds later; time-to-2m
          is reported in info. Off by default.
"""
from __future__ import annotations
import os
import re
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bimo_biped.xml")

# qpos/qvel layout: freejoint (7 pos+quat / 6 vel) then the hinge joints. The
# actuated-joint slices/index maps are derived per-instance from the loaded
# model (self._jqpos / self._jqvel / self._legL / self._legR), so both the
# 8-DOF plant and the 10-DOF hip-yaw plant work with no literals here.

# -- STS3215 servo datasheet numbers (Waveshare ST3215 wiki, checked 2026-07-11:
# https://www.waveshare.com/wiki/ST3215_Servo). "High torque, up to 30kg.cm@12V"
# and "No load Speed: 0.222sec/60deg (45RPM) @12V"; input voltage 6-12.6 V, so a
# 2S (7.4 V) or 3S (11.1 V) LiPo is in-spec. Stall torque and no-load speed are
# scaled linearly with supply voltage (standard DC-motor approximation).
_STS_STALL_12V = 2.94                       # N*m  (30 kg*cm)
# Datasheet says 0.222 s/60deg -> 4.712 rad/s at 12 V. MEASURED 2026-08-03
# on the assembled robot (tools/measure_servo_speed.py, R_knee free sweeps,
# 2140 ticks travel, both directions): ~2700 steps/s at 12.3 V = 4.14 rad/s,
# i.e. 86% of the datasheet line. Commanded goal speeds 500..2000 tracked
# within 0.5%, so this is the servo's ceiling, not a control artifact.
# 4.14 * (12.0/12.3) = 4.04 rad/s per 12 V.
_STS_NOLOAD_12V = 4.04                      # rad/s, measured (was 4.712)

# -- torso-top payload: GoPro MAX 360 camera, 154 g incl. battery, ~64 wide x
# 69 tall x 25 deep (mm), CG ~45 mm above the tower top plate with the folding
# finger mount => ~+0.08 m above the torso center. Box inertia via the compiler.
_PAYLOAD_REF = 0.154   # kg -- reference (real camera) mass for inertia scaling

# -- heightfield terrain constants -------------------------------------------
# Grid spans x in [cx-rx, cx+rx], y in [-ry, ry]; ~2 cm cells (feet are 8.4 cm
# long, so bumps are resolved well below foot scale). Long axis points +x so a
# full 10 s episode at >1 m/s stays on the field.
_HF_NROW, _HF_NCOL = 100, 600            # y rows, x cols
_HF_RX, _HF_RY, _HF_CX = 6.0, 1.0, 4.5   # meters: field spans x -1.5..10.5

# -- STS3215 encoder/bus quantization (quantize_ticks) ------------------------
# The deployment boundary, defined by sim/sil/harness.py + docs/sil-harness.md:
# the servo reports POSITION as a uint16 tick word (4096 ticks/rev, full scale
# 0..4095, "middle" 2048 == scsbus::kStepsPerRev / obs::Calibration) and
# VELOCITY as an integer reg-58 word in steps/s (sign-magnitude, 15-bit
# magnitude), and goal positions are written back as integer ticks. Mirrors
# harness.STEPS_PER_REV / RAD_PER_STEP / CENTER_STEPS / MAX_STEPS exactly, and
# is mirrored again in sim/mjx/env_mjx.py (parity gate 2i).
_TICK_STEPS_PER_REV = 4096
_TICK_RAD = 2.0 * np.pi / _TICK_STEPS_PER_REV   # one quantum, rad
_TICK_ZERO = 2048                                # zero_steps, nominal cal
_TICK_MAX = 4095                                 # encoder full scale
_TICK_VEL_MAX = 32767                            # reg-58 sign-magnitude |v|
# Per-joint direction signs. Cross-ref sim/sil/cal/cal_nominal.json ("dir":
# [1]*10 for the v3yaw joint order L_hip_yaw..R_ankle) == obs::Calibration's
# C++ default == harness.Calibration.nominal(): a correctly oriented, freshly
# "Set Middle Position"-ed robot has dir +1 on every joint. Kept as an explicit
# per-joint vector so a measured (dir=-1) calibration is a one-line change.
_TICK_DIR = 1.0


def policy_range(m):
    """Per-actuator (lo, hi) the POLICY trains in, radians. MIRRORED in
    sim/mjx/env_mjx.py -- keep the two identical.

    The plant's JOINT limit is the mechanical stop: how far the joint can be
    driven before it hits metal. The POLICY range is a separate, narrower
    thing: action +-1 is defined as its edge, so moving it silently rescales
    every action a trained policy emits, and every scorecard with it. The two
    were the same number until bimo_biped_v4rom.xml (2026-08-03), where the
    hip-roll and hip-pitch stops widened to the measured/CAD-verified truth
    while the training range deliberately stayed at v3yaw's.

    A plant declares the split by putting a ctrlrange on its <position>
    actuators; that is the policy range, and the joint limit is left as the
    mechanical stop. Plants that declare none -- bimo_biped_v2*.xml,
    bimo_biped_v3yaw.xml, every legacy referee -- fall back to the joint
    limit, which is what they have always used, so they load bit-identically
    and their runs keep meaning what they meant.

    This is the same split firmware/main/mech_envelope.h draws between
    obs_spec.h's kJointLo/kJointHi (policy, action scaling) and the bench
    clamp. tools/gen_obs_spec.py emits kJointLo/kJointHi from env._lo/_hi, so
    a run trained on a split plant still ships the POLICY range to the robot
    and the envelope's containment static_assert still holds.

    NOTE ctrlrange is in the actuator's own units and the compiler does NOT
    apply angle="degree" to it (unlike jnt_range) -- so a split plant spells
    its ctrlrange in radians. tests/test_rom_contacts.py pins those numbers.
    """
    jnt = m.actuator_trnid[:, 0]
    lo = np.array(m.jnt_range[jnt, 0], dtype=float)
    hi = np.array(m.jnt_range[jnt, 1], dtype=float)
    lim = np.asarray(m.actuator_ctrllimited).astype(bool)
    lo[lim] = np.asarray(m.actuator_ctrlrange)[lim, 0]
    hi[lim] = np.asarray(m.actuator_ctrlrange)[lim, 1]
    return lo, hi


def _gauss_smooth(field: np.ndarray, sigma: float) -> np.ndarray:
    """Separable Gaussian blur (reflect-padded), sigma in cells. Pure numpy so
    we don't add a scipy dependency for one filter."""
    if sigma < 0.5:
        return field
    r = max(1, int(3 * sigma))
    x = np.arange(-r, r + 1)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    k /= k.sum()
    conv = lambda v: np.convolve(np.pad(v, r, mode="reflect"), k, "valid")
    return np.apply_along_axis(conv, 0, np.apply_along_axis(conv, 1, field))


class BimoWalkerEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(
        self,
        xml_path: str = _XML,
        control_hz: float = 50.0,
        episode_seconds: float = 10.0,
        render_mode: str | None = None,
        # -- reward shaping (override to iterate without editing step()) --------
        w_forward: float = 1.5,        # weight on forward velocity
        target_speed: float | None = None,  # cap fwd-vel reward here (m/s); None = uncapped
        w_upright: float = 0.5,        # weight on torso up-vector z
        alive_bonus: float = 0.1,      # per-step reward for not terminating
        w_height: float = 0.0,         # penalty on |height - nominal| (posture)
        w_energy: float = 0.002,       # penalty on actuator work
        w_action_rate: float = 0.05,   # penalty on action jitter
        fall_cost: float = 1.0,        # one-off penalty on termination
        # -- gait-quality shaping (all default 0 -> old reward reproduces) ------
        w_feet_air: float = 0.0,       # reward per-foot swing time on touchdown
        air_time_target: float = 0.3,  # cap the rewarded swing duration (s)
        w_single_support: float = 0.0, # per-step reward when exactly 1 foot down
        w_lateral: float = 0.0,        # penalty on lateral vel + y drift
        w_yaw: float = 0.0,            # penalty on heading error + yaw rate
        w_pitch_rate: float = 0.0,     # penalty on torso roll/pitch rates
        w_pitch_hinge: float = 0.0,    # penalty on |torso pitch| past a deadband
        pitch_deadband_deg: float = 5.0,  # free pitch band (deg) before it bites
        # -- actuator model ("ideal" -> original MJCF position servos) ----------
        actuator_model: str = "ideal", # "ideal" | "sts3215" (torque-speed limited)
        supply_voltage: float = 7.4,   # V; stall torque + no-load speed scale ~V/12
        servo_kp: float = 12.0,        # sts3215 PD gain, N*m/rad (see fit note)
        servo_kd: float = 0.25,        # sts3215 PD damping, N*m*s/rad
        servo_range: float = 0.15,     # DR: stall/no-load speed scale +/- this
        servo_joint_damping: float = 0.1,  # sts3215: joint damping override --
        # the MJCF's 0.6 N*m*s/rad was a stability proxy for the ideal servo and
        # double-counts motor losses the datasheet no-load speed already includes
        # (it alone would eat the whole stall torque at ~3 rad/s). 0.1 leaves a
        # small bearing/gear-mesh loss. Fit check: 60 deg step @7.4 V reaches 90%
        # in 0.394 s (envelope-limited minimum 0.360 s), ~1.4% overshoot.
        # -- torso-top payload (GoPro MAX 360: 154 g, ~64x69x25 mm, CG ~45 mm
        # above the tower top plate => ~+0.08 m above the torso center) ---------
        payload_mass: float = 0.0,     # fixed payload mass (kg); 0 = none
        payload_max: float | None = None,  # if set, per-episode mass drawn
        # uniform(0, payload_max) -- one policy handles camera-on AND camera-off
        # -- 2 m dash objective (off by default) --------------------------------
        dash: bool = False,            # success = cross dash_distance AND stay
        dash_distance: float = 2.0,    # upright for dash_hold s after crossing
        dash_hold: float = 1.0,        # (kills the dive-across-the-line exploit)
        dash_stop: bool = False,       # stricter finish: after crossing, come to
        # a STANDSTILL -- planar speed < stand_speed continuously for dash_hold s
        # while upright (a jog-through no longer finishes). Post-cross, the
        # forward reward flips to a brake-and-stand term and the gait-shaping
        # bonuses (feet-air, single-support) switch off.
        stand_speed: float = 0.15,     # m/s planar speed that counts as standing
        w_time_stop: float = 1.5,      # EXTRA per-step time penalty once the
        # line is crossed (dash_stop only). Sized to beat the ~1.1/step that
        # alive+upright farm: without it, loitering past the line to
        # truncation out-earns the finish bonus (observed: dash_11v1_real
        # crossed then walked circles for 7 s, 0/16). With it, every
        # non-finishing post-cross step is net-negative and confirming the
        # stand ASAP is the only profitable move.
        w_time: float = 0.0,           # per-step time penalty (dash urgency)
        finish_bonus: float = 0.0,     # one-off reward on a confirmed finish
        # -- procedural terrain (0.0 -> original flat plane) --------------------
        terrain_amplitude: float = 0.0,   # max bump height (m); 0.01 is serious
        terrain_smoothness: float = 0.15, # bump feature size (m)
        terrain_amplitude_min: float | None = None,  # if set, per-episode bump
        # height is drawn uniform(min, amplitude) -- mixes easy/hard episodes
        # (a built-in curriculum) instead of every episode at max roughness
        terrain_mosaic: bool = False,  # loco_v7knee (2026-07-29): load the
        # SHARED static tiled mosaic (sim/terrain_mosaic.npz, 0-20 mm tiles)
        # instead of regenerating noise -- per-episode roughness variation
        # comes from a random spawn location. Mirrors sim/mjx exactly (the
        # npz is the single source of ground truth).
        mimic_knee_w: float = 1.0,     # per-joint mimic weighting on the knee
        # (mirrors sim/mjx; the lump-sum kernel let hips+ankles satisfy the
        # gait reference while the knee stayed jammed at its extension stop)
        # -- domain randomization (sim-to-real hardening; off by default) -------
        domain_rand: bool = False,     # master switch for mass/friction/gain DR
        mass_range: float = 0.15,      # per-body mass+inertia scale +/- this
        friction_range: float = 0.4,   # floor sliding friction scale +/- this
        gain_range: float = 0.2,       # actuator kp scale +/- this
        action_latency: int = 0,       # control-step delay on applied action
        # -- sub-step action latency: real bus/control latency is a few ms, far
        # finer than the 20 ms control step (the coarse action_latency above).
        # The first round(latency_ms / sim_dt) physics substeps of each control
        # step run with the PREVIOUS target still applied, then the new target
        # takes over. Composes with action_latency for delays > one step.
        latency_ms: float = 0.0,       # fixed sub-step latency, 0..20 ms
        latency_ms_max: float | None = None,  # if set, per-episode latency is
        # drawn uniform(latency_ms, latency_ms_max) -- built-in latency DR
        latency_jitter_ms: float = 0.0,  # per-control-step +/- jitter around
        # the episode latency (real bus timing is not constant)
        backlash_deg: float = 0.0,     # gear backlash as a +/-b/2 deadzone on
        # the PD position error (issues #6/#13: STS3215 geartrains measure
        # ~0.5-1.0 deg of lash). 0 = legacy behavior, bit-exact.
        backlash_deg_max: float | None = None,  # per-episode draw
        # uniform(backlash_deg, max) -- backlash DR
        fall_height: float = 0.18,     # terminate below this torso height (m)
        fall_up_z: float = 0.4,        # terminate below this up-vector z
        # (issue #10: 0.4 = 66 deg lean is generous; kept as the default for
        # legacy-run reproducibility -- tighten per-run for future training)
        push_prob: float | None = None,  # per-control-step shove chance (None=auto w/ DR)
        push_force: float | None = None, # shove magnitude (N); 0 disables, None=auto w/ DR
        # -- command-conditioned locomotion (2026-07-12) -------------------------
        # The successor to the dash objective: the policy observes a commanded
        # body-frame forward velocity + yaw rate (resampled mid-episode,
        # including zero = stand still) and is rewarded for TRACKING it.
        # Stopping and turning become continuously-practiced behaviors instead
        # of a terminal event -- which is what made the dash_stop objective
        # exploitable three different ways (see DESIGN.md). Obs grows by 2
        # (the command), so command policies don't warm-start from dash ones.
        command_mode: bool = False,
        cmd_v_range: tuple = (0.3, 1.0),   # forward-speed command draw (m/s)
        cmd_w_range: float = 1.0,          # |yaw-rate| command bound (rad/s)
        cmd_stand_prob: float = 0.3,       # chance a command is "stand still"
        cmd_resample_s: tuple = (2.5, 4.5),  # seconds between command changes
        cmd_fixed: tuple | None = None,  # pin the command to (v, w) for the
        # whole run: single-behavior EXPERT training for distillation (the
        # command obs channels stay present but constant, so experts share
        # the student's observation space)
        cmd_dense: bool = False,       # dense-progress velocity reward for
        # moving commands: w_track_v * min(vx, cmd_v)/cmd_v -- the dash-style
        # any-progress-pays gradient that demonstrably teaches walking on
        # this plant, instead of the exp kernel (which pays ~nothing until
        # you're already near the commanded speed). Stand commands and the
        # yaw term keep the kernel.
        w_track_v: float = 2.0,        # velocity-tracking reward (exp kernel)
        w_track_w: float = 1.0,        # yaw-rate-tracking reward (exp kernel)
        # -- precision command mode (2026-07-17; arithmetic mirrors sim/mjx) ----
        # 7-channel commands (vx, vy, wz, crouch height frac, foot-lift,
        # swing-foot dx, dz): sidestep, backward walk, crouch, one-leg
        # balance, air circles. Requires command_mode. obs grows to 43.
        ext_cmd: bool = False,
        cmd_vy_range: float = 0.25,
        cmd_back_range: tuple = (-0.4, -0.15),
        crouch_range: tuple = (0.6, 0.9),
        lift_height: float = 0.06,
        w_track_h: float = 0.0,        # height-tracking kernel weight
        w_lift: float = 0.0,           # correct one-foot contact pattern
        w_track_foot: float = 0.0,     # swing-foot target kernel weight
        foot_sigma: float = 0.06,      # widened 0.03 -> 0.06 with sim/mjx
        # (a 3 cm kernel pays ~0 gradient at v4's ~15 cm foot error)
        traj_radius: tuple = (0.02, 0.05),
        traj_period: tuple = (1.5, 3.5),
        ext_mix: tuple = (0.20, 0.08, 0.15, 0.12, 0.10),  # stand, crouch,
        # balance, circle, pivot[, march, sway]; 5-tuple = no march/sway
        walk_submix: tuple = (0.15, 0.15),  # of walks: (backward, sidestep)
        turn_emph: bool = False,       # mirrors sim/mjx turn_emph (loco_v5t)
        w_foot_cross: float = 0.0,     # feet-crossing guard (mirrors sim/mjx)
        sway_vy: float = 0.12,         # sway-command vy amplitude (m/s)
        # -- knee-high marching (2026-08-01; mirrors sim/mjx) -----------------
        march_mix: float = 0.0,        # fraction of ext_cmd command draws that
        # are a KNEE-HIGH march: an extra slice carved out after the ext_mix
        # slices (walk takes the remainder). 0.0 -> the slice is empty and the
        # draw is bit-identical to before the feature existed.
        w_knee_high: float = 0.0,      # knee-high clearance kernel weight
        # (mirrors sim/mjx): pays the FRACTION of the commanded swing height
        # (lift_height + c6) the swing sole actually reaches, gated on the
        # correct one-foot contact pattern. Ordinary lifts command c6 = 0, so
        # the kernel degrades to "reach lift_height"; the march commands
        # c6 = march_dz, i.e. a target at the opposite knee's standing height.
        march_hz: float = 0.0,         # COMMANDED march cadence in full L/R
        # cycles per second. 0.0 (default) = alternate on the gait clock,
        # which inherits its 1.25-1.75 Hz draw -> 0.29-0.40 s per lift; that
        # is a sprinter's march, and 165 ms of upswing to knee height is the
        # trainability risk. A positive march_hz pins a deliberate cadence
        # instead (1.0 -> 0.5 s per lift) and DECOUPLES the march from the
        # gait clock: the gait-clock obs channel keeps running at its own
        # rate, so the policy has to learn that during a march the swing
        # timing comes from the commanded pattern, not from the clock.
        # -- plan-v2 Phase A terms (2026-07-20; mirror sim/mjx, default off) ---
        gait_clock: bool = False,
        w_feet_phase: float = 0.0,
        swing_height: float = 0.06,
        feet_phase_s2: float = 0.004,
        w_feet_slip: float = 0.0,
        w_orientation: float = 0.0,
        w_ang_vel_xy: float = 0.0,
        w_pose: float = 0.0,
        w_dof_limits: float = 0.0,
        push_kick: bool = False,
        kick_range: tuple = (0.1, 1.0),
        obs_hist_len: int = 1,
        joint_frictionloss: float = 0.0,
        joint_armature: float = 0.0,
        # -- torque-off standing (eval-only power-saving idle) ------------------
        # When servo torque is released (set_torque_enabled(False)) the joints'
        # static friction is raised to off_frictionloss for the duration.
        # Rationale: joint_frictionloss (0.05 N*m) was BAM-identified for the
        # POWERED STS3215 and reflects gear-mesh + bearing drag while the motor
        # is driving. A POWERED-OFF STS3215 is much harder to backdrive: the
        # ~1:345 metal gear train reflects the rotor's detent/cogging torque and
        # coulomb friction to the output, so the static breakaway is a sizeable
        # fraction of rated output (30 kg*cm = 2.94 N*m @12V). 0.35 N*m ~= 12% of
        # stall is a plausible unpowered breakaway, but it is an ESTIMATE from
        # the gear-train class, not a measurement -- must be measured on the real
        # servos when they arrive (see the sensitivity sweep in eval_precision).
        off_frictionloss: float = 0.35,
        w_mimic: float = 0.0,          # procedural-gait imitation (Phase B;
        # mirrors sim/mjx _mimic_ref exactly -- requires gait_clock)
        mimic_s2: float = 0.72,
        w_rise_dofvel: float = 0.0,    # rise jerk control (recovery only)
        w_rise_ref: float = 0.0,       # staged-rise reference kernel while
                                       # down (mirrors sim/mjx _rise_ref)
        rise_secs: float = 4.0,
        rise_ref_s2: float = 2.0,
        w_symmetry: float = 0.0,       # gait-symmetry penalty on touchdown:
        # |this swing duration - other foot's last swing| (mirrors sim/mjx)
        w_foot_under: float = 0.0,     # mirrors sim/mjx w_foot_under
        w_up_vel: float = 0.0,         # mirrors sim/mjx w_up_vel (getup_v5)
        w_com_stance: float = 0.0,     # while lifted: CoM-over-stance-foot
                                       # kernel (mirrors sim/mjx -- knee-
                                       # flexion lifts keep the CG planted)
        com_sigma: float = 0.04,       # CoM-offset kernel width (m)
        w_heading: float = 0.0,        # integrated-heading kernel weight
                                       # (mirrors sim/mjx heading integrator)
        lift_clear: float = 0.03,      # lifted-foot min clearance (m); the
        # lift reward pays only >= this above the standing sole height
        recover_mix: float = 0.0,      # fraction of episodes starting from a
        # settled ragdoll fall: recovery reward while down, command pinned
        # to stand, no fall termination until the first achieved stand
        # (mirrors sim/mjx; single-leg-crouch mix removed same date)
        recover_start_mix: tuple = (1.0, 0.0, 0.0, 0.0),  # recovery-episode
        # start states (ragdoll, kneel, squat, sit) -- reverse curriculum;
        # poses mirror sim/mjx/_fallen_data. Referee scenarios pin this.
        # payload CG height above torso center (m); 0.08 = 2026-07-11 spec,
        # 0.0945 = current stack (battery-bay tower + imu_carrier + gopro_base)
        payload_cg_z: float = 0.08,
        payload_cg_x: float = 0.0,
        w_power: float = 0.0,          # electrical-power penalty (W). Unlike
        # w_energy (mechanical |tau*w|), this prices what drains the battery:
        # P = sum(max(tau*w, 0) + K_CU * tau^2) -- the tau^2 copper loss means
        # HOLDING torque costs watts at zero motion (a stiff, trembling stand
        # is expensive; a relaxed one is cheap). K_CU from ST3215 stall:
        # ~32 W electrical at 2.94 N*m stall, zero mechanical -> 3.75 W/(N*m)^2.
        # -- get-up mode (fall recovery, 2026-07-14; matches sim/mjx) -----------
        # Episodes start from a settled ragdoll fall; NO fall termination
        # (being down is the task). Primary reward (requires command_mode,
        # command pinned to (0,0) so the obs layout matches the command
        # family): height progress + uprightness + a standing bonus.
        getup: bool = False,
        getup_settle_s: float = 0.4,
        w_recover_h: float = 1.0,
        w_recover_up: float = 0.8,
        stand_bonus: float = 1.0,
        # -- action mapping (2026-07-14) -----------------------------------------
        # "legacy": target = default + 0.5*(hi-lo)*a -- a symmetric band that
        # CANNOT reach the far side of asymmetric ranges (the knee, -95..+5,
        # was capped at -50 deg for every policy ever trained -- found by the
        # get-up study's dangle test). "full": piecewise-linear residual that
        # reaches both true limits; action 0 is still the standing pose.
        action_map: str = "legacy",
        # -- hip-pitch flexion override (get-up study, 2026-07-14) ---------------
        # The study's static path analysis: a CoM-over-soles sit->stand needs
        # >= 95 deg of hip FLEXION (flexion is negative on this model); the
        # CAD yoke currently stops at 60. Set e.g. 110.0 to train/eval with
        # the widened range before the yoke is redesigned. None = XML truth.
        hip_flex_deg: float | None = None,
        # -- hardware-realizable observations (sensing audit 2026-07-11) --------
        imu_obs: bool = False,         # torso linear velocity + height are ZEROED
        # (the real robot has no sensor for them; obs stays 36-wide so warm
        # starts still work), and up-vector/gyro become IMU-like: per-episode
        # mounting misalignment + gyro bias, per-step noise (BNO085-class
        # magnitudes) -- applied only when domain_rand is on.
        imu_noise: float = 1.0,        # scale on IMU misalignment/bias/noise DR
        # -- servo tick quantization (SIL boundary realism, 2026-07-31) --------
        # The policy trains on float joint states, but the real robot reads
        # STS3215 ENCODERS: position as a uint16 tick word (4096/rev, zero
        # 2048) and velocity as an integer reg-58 steps/s word, and it writes
        # goal positions back as integer ticks. quantize_ticks puts that
        # quantizer inside training: joint-position/velocity OBSERVATIONS and
        # the COMMANDED target angles are rounded to the servo grid. The IMU
        # channels and the physics state itself are untouched -- only what the
        # policy sees and what the bus can express. Mirrored exactly in
        # sim/mjx/env_mjx.py (parity gate 2i). Default False = bit-exact
        # legacy behavior.
        quantize_ticks: bool = False,
        # -- crouch-command variation (SIL finding #2, docs/sil-harness.md) ----
        # cmd[3] (crouch height fraction) was frozen at exactly 1.0 through
        # every training run, so its obs-normalizer std collapsed to ~1e-6 --
        # but the firmware feeds that channel battguard's crouch(), which ramps
        # BELOW 1.0 on a sagging pack, normalizing to ~-1.5e5 (five orders out
        # of distribution, exactly when the battery is dying). A per-episode
        # uniform draw over this range replaces the frozen 1.0 wherever a
        # SAMPLED command would have used it; an explicit crouch-skill draw
        # (ext_mix crouch commands) and set_command() overrides are left alone.
        # (1.0, 1.0) = the legacy frozen channel, no RNG consumed.
        cmd_crouch_range: tuple = (1.0, 1.0),
    ):
        super().__init__()
        self.w_forward = w_forward
        self.target_speed = target_speed
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
        self.w_yaw = w_yaw
        self.w_pitch_rate = w_pitch_rate
        # torso-pitch MAGNITUDE hinge (day 13): the direct lever on the
        # "controlled fall" lean. The upright term is cos-flat (cos(13.5 deg)
        # = 0.972, so the whole lean costs 0.022/step at loco_v11gait's
        # w_upright 0.8 against a 2.0/step velocity income), and
        # the CoM-over-stance kernel is clearance-gated and never binds
        # during the walk cycle (refuted, docs/precision-progress.md day 12).
        # This penalizes |pitch| itself, quadratically past a free deadband.
        # PITCH ONLY -- roll is untouched, sidestep gaits legitimately roll.
        self.w_pitch_hinge = w_pitch_hinge
        self.pitch_deadband_deg = pitch_deadband_deg
        self._pitch_db = float(np.radians(pitch_deadband_deg))
        self.terrain_amplitude = terrain_amplitude
        self.terrain_smoothness = terrain_smoothness
        self.terrain_amplitude_min = terrain_amplitude_min
        self.domain_rand = domain_rand
        self.mass_range = mass_range
        self.friction_range = friction_range
        self.gain_range = gain_range
        # Shoves: auto-enable a *gentle* default with DR unless set explicitly.
        # None => auto; an explicit 0 disables (distinct from "unset").
        self.action_latency = action_latency
        self.latency_ms = latency_ms
        self.latency_ms_max = latency_ms_max
        self.latency_jitter_ms = latency_jitter_ms
        self.backlash_deg = backlash_deg
        self.backlash_deg_max = backlash_deg_max
        self._lash_rad = np.deg2rad(backlash_deg)
        self.fall_height = fall_height
        self.fall_up_z = fall_up_z
        self.push_prob = (0.01 if domain_rand else 0.0) if push_prob is None else push_prob
        self.push_force = (5.0 if domain_rand else 0.0) if push_force is None else push_force
        self.payload_mass = payload_mass
        self.payload_max = payload_max
        self.getup = getup
        self._getup_settle = max(1, round(getup_settle_s / 0.002))
        self.w_recover_h = w_recover_h
        self.w_recover_up = w_recover_up
        self.stand_bonus = stand_bonus
        if getup:
            if not command_mode:
                raise ValueError("getup mode requires command_mode "
                                 "(obs layout compatibility)")
            if ext_cmd:
                raise ValueError("getup and ext_cmd are separate objectives")
            cmd_fixed = (0.0, 0.0)
        if ext_cmd and not command_mode:
            raise ValueError("ext_cmd requires command_mode")
        self.command_mode = command_mode
        self.cmd_v_range = cmd_v_range
        self.cmd_w_range = cmd_w_range
        self.cmd_stand_prob = cmd_stand_prob
        self.cmd_resample_s = cmd_resample_s
        self.cmd_fixed = cmd_fixed
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
        _cs = os.path.join(os.path.dirname(xml_path), "getup_catch_states.npz")
        self._catch_states = np.load(_cs) if os.path.exists(_cs) else None
        self.w_foot_cross = w_foot_cross
        self.sway_vy = sway_vy
        self.march_mix = float(march_mix)
        self.w_knee_high = w_knee_high
        self.march_hz = float(march_hz)
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
        self._gait_freq = 0.0
        self._gait_phase = 0.0
        self._obs_hist = []
        self.w_symmetry = w_symmetry
        self.w_com_stance = w_com_stance
        self.w_foot_under = w_foot_under
        self.w_up_vel = w_up_vel
        self.com_sigma = com_sigma
        self.w_heading = w_heading
        self._last_air = np.zeros(2)
        self.lift_clear = lift_clear
        self.recover_mix = recover_mix
        self.recover_start_mix = recover_start_mix
        self._recover_ep = False       # this episode started fallen
        self._recovered = True         # first stand achieved (or normal ep)
        self._stand_streak = 0.0       # consecutive standing steps (getup_v9)
        self.payload_cg_z = payload_cg_z
        self.payload_cg_x = payload_cg_x
        self.w_power = w_power
        self._K_CU = 3.75              # W/(N*m)^2, ST3215 stall calibration
        self._ncmd = 7 if ext_cmd else 2
        self._cmd = np.zeros(self._ncmd)  # (vx, wz) or 7-channel ext
        if ext_cmd:
            self._cmd[3] = 1.0            # crouch channel: 1 = full height
        self.quantize_ticks = bool(quantize_ticks)
        self.cmd_crouch_range = tuple(cmd_crouch_range)
        # feature flag: (1,1) draws no RNG at all, so runs with the default
        # consume the identical random stream as before
        self._crouch_dr = (self.cmd_crouch_range != (1.0, 1.0))
        if self._crouch_dr and not ext_cmd:
            raise ValueError("cmd_crouch_range requires ext_cmd "
                             "(cmd[3] only exists in the 7-channel layout)")
        if self.march_mix > 0.0 and not ext_cmd:
            raise ValueError("march_mix requires ext_cmd (the knee-high "
                             "march is expressed through c4/c6)")
        self._cmd_crouch = 1.0            # per-episode crouch-command draw
        self._cmd_next = 0
        self._traj = np.zeros(3)       # air-circle: radius, omega, phase0
        self._traj_on = 0.0            # trajectory mode (0/1/2/3)
        self.imu_obs = imu_obs
        self.imu_noise = imu_noise
        self._imu_R = np.eye(3)          # per-episode mounting misalignment
        self._imu_gyro_bias = np.zeros(3)
        xml_src = None
        self.terrain_mosaic = terrain_mosaic
        self._mosaic = None
        if terrain_mosaic:
            _tm = np.load(os.path.join(os.path.dirname(os.path.abspath(
                xml_path)), "terrain_mosaic.npz"))
            self._mosaic = _tm
            xml_src = self._terrain_xml(
                xml_path, float(_tm["max_amp"]),
                nrow=int(_tm["nrow"]), ncol=int(_tm["ncol"]),
                rx=float(_tm["rx"]), ry=float(_tm["ry"]),
                cx=float(_tm["cx"]))
        elif terrain_amplitude > 0:
            # Patch the flat-plane MJCF into a heightfield variant at load time
            # (single source of truth: no duplicated terrain XML to drift).
            xml_src = self._terrain_xml(xml_path, terrain_amplitude)
        if payload_mass > 0 or payload_max:
            # Torso-top camera payload, also MJCF-patched (default: no payload,
            # model byte-for-byte identical). Compile with the reference camera
            # mass when randomizing so the nominal inertia is the real camera's.
            if xml_src is None:
                with open(xml_path) as f:
                    xml_src = f.read()
            xml_src = self._payload_xml(
                xml_src, payload_mass if payload_mass > 0 else _PAYLOAD_REF,
                payload_cg_z, payload_cg_x)
        if xml_src is not None:
            # from_xml_string resolves a relative meshdir against the CWD, not
            # the source file -- absolutize it against the XML's own directory
            xml_src = re.sub(
                r'meshdir="([^"]+)"',
                lambda m: 'meshdir="' + os.path.normpath(os.path.join(
                    os.path.dirname(os.path.abspath(xml_path)), m.group(1))) + '"',
                xml_src)
            self.model = mujoco.MjModel.from_xml_string(xml_src)
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self._payload_bid = (self.model.body("payload").id
                             if (payload_mass > 0 or payload_max) else None)

        # Actuated-joint layout, derived from the model (NOT literals) so the
        # 8-DOF and 10-DOF hip-yaw plants both work. Action/slice order ==
        # actuator order == qpos-address order by XML construction.
        _jnt = self.model.actuator_trnid[:, 0]
        _qadr = self.model.jnt_qposadr[_jnt]
        _vadr = self.model.jnt_dofadr[_jnt]
        self._jq0, self._jq1 = int(_qadr.min()), int(_qadr.max()) + 1
        self._jv0, self._jv1 = int(_vadr.min()), int(_vadr.max()) + 1
        self._nq_act = self._jq1 - self._jq0
        self._jqpos = slice(self._jq0, self._jq1)
        self._jqvel = slice(self._jv0, self._jv1)
        self._act_names = [self.model.joint(int(j)).name for j in _jnt]
        self._jname2i = {n: i for i, n in enumerate(self._act_names)}
        _roles = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
        self._legL = {r: self._jname2i.get(f"L_{r}") for r in _roles}
        self._legR = {r: self._jname2i.get(f"R_{r}") for r in _roles}
        # per-role (L, R) action-index pairs (hip_yaw absent on the 8-DOF plant)
        self._i_roll = np.array([self._legL["hip_roll"], self._legR["hip_roll"]])
        self._i_pitch = np.array([self._legL["hip_pitch"],
                                  self._legR["hip_pitch"]])
        self._i_knee = np.array([self._legL["knee"], self._legR["knee"]])
        self._i_ankle = np.array([self._legL["ankle"], self._legR["ankle"]])
        self.mimic_knee_w = mimic_knee_w
        self._mimic_w = np.ones(self._nq_act)
        self._mimic_w[self._i_knee] = mimic_knee_w
        # per-joint encoder direction (nominal calibration -- see _TICK_DIR)
        self._tick_dir = np.full(self._nq_act, _TICK_DIR, dtype=np.float64)

        # Realistic actuator model: torque is computed here (PD on the commanded
        # angle) and clamped to the DC-motor torque-speed envelope -- available
        # torque falls linearly from stall at zero speed to zero at no-load
        # speed (magnitude clamped symmetrically). The MJCF position actuators
        # are silenced (gain/bias zeroed) and torque enters via qfrc_applied.
        # This MUST happen before the nominal-parameter snapshot below, so DR
        # re-derives from the silenced actuators, not the ideal ones.
        self.actuator_model = actuator_model
        self.supply_voltage = supply_voltage
        if actuator_model == "sts3215":
            v = supply_voltage / 12.0
            # [kp, kd, stall torque, no-load speed] -- DR rescales from nominal
            self._nom_servo = np.array([servo_kp, servo_kd,
                                        _STS_STALL_12V * v, _STS_NOLOAD_12V * v])
            self._servo = self._nom_servo.copy()
            self.model.actuator_gainprm[:] = 0.0
            self.model.actuator_biasprm[:] = 0.0
            self.model.dof_damping[self._jqvel] = servo_joint_damping
        elif actuator_model == "ideal":
            self._nom_servo = None
            self._servo = None
        else:
            raise ValueError(f"unknown actuator_model {actuator_model!r}")
        self.servo_range = servo_range
        self._servo_tau = np.zeros(self._nq_act)
        self._torque_on = True         # see set_torque_enabled()
        self.off_frictionloss = off_frictionloss
        self._held_frictionloss = None  # saved joint frictionloss while released

        self.dash = dash
        self.dash_distance = dash_distance
        self.dash_hold = dash_hold
        self.dash_stop = dash_stop
        self.stand_speed = stand_speed
        self.w_time_stop = w_time_stop
        self.w_time = w_time
        self.finish_bonus = finish_bonus

        # Nominal dynamics parameters, snapshotted so each reset re-randomizes
        # from the originals rather than compounding noise across episodes.
        self._nom_mass = self.model.body_mass.copy()
        self._nom_inertia = self.model.body_inertia.copy()
        self._nom_friction = self.model.geom_friction.copy()
        self._nom_gain = self.model.actuator_gainprm.copy()
        self._nom_bias = self.model.actuator_biasprm.copy()
        self._floor_gid = self.model.geom("floor").id
        self._torso_bid = self.model.body("torso").id
        self._sole_gids = (self.model.geom("L_sole").id, self.model.geom("R_sole").id)
        # Ground-contact geoms per foot. On the v3yaw plant (2026-07-30) the
        # sole box became a non-colliding REFERENCE geom and the floor contact
        # moved to four "<side>_pad_*" corner spheres -- one analytic contact
        # point each, which MJX and CPU agree on, unlike a box manifold. Legacy
        # plants have no pad geoms, so the sole box itself is the contact geom.
        self._pad_gids = tuple(
            tuple(g for g in range(self.model.ngeom)
                  if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g)
                      or "").startswith(f"{side}_pad")) or (sole,)
            for side, sole in (("L", self._sole_gids[0]),
                               ("R", self._sole_gids[1])))

        # Terrain bookkeeping: cached (nrow, ncol) height grid in [0,1] for the
        # ground-height lookup, and a dirty flag so render() re-uploads the
        # hfield to the GPU after each re-randomization.
        self._hf_grid = None
        self._terrain_dirty = False
        # grid dims used by _ground_z (legacy defaults; mosaic overrides)
        self._hf_dims = (_HF_NROW, _HF_NCOL, _HF_RX, _HF_RY, _HF_CX)
        if self._mosaic is not None:
            _tm = self._mosaic
            self._hf_dims = (int(_tm["nrow"]), int(_tm["ncol"]),
                             float(_tm["rx"]), float(_tm["ry"]),
                             float(_tm["cx"]))
            # grid stored in METERS (amplitude baked in); _ground_z scales by
            # terrain_amplitude, so pin it to 1.0 in mosaic mode
            self._hf_grid = np.asarray(_tm["field"], dtype=np.float64)
            self.terrain_amplitude = 1.0
            self.model.hfield_data[:] = (self._hf_grid.ravel()
                                         / float(_tm["max_amp"]))
            self._terrain_dirty = True

        self.sim_dt = self.model.opt.timestep                 # 0.002 s
        self.n_substeps = max(1, round((1.0 / control_hz) / self.sim_dt))
        self.control_dt = self.n_substeps * self.sim_dt        # ~0.02 s
        # getup_v9: held-stand requirement for the recovered flip (mirrors
        # sim/mjx stand_hold_n)
        self._stand_hold_n = max(1, round(0.5 / self.control_dt))
        self.max_steps = int(episode_seconds / self.control_dt)

        # Per-joint actuator ranges (radians). Actions are a residual around the
        # nominal standing pose (qpos0, all-zeros here) so that action 0 == stand
        # -- the standard locomotion-RL convention. Without this, action 0 maps to
        # each joint's range midpoint (a deep knee crouch) and the robot topples.
        jnt = self.model.actuator_trnid[:, 0]
        # hip_flex_deg re-sets the hip-pitch FLEXION limit (get-up study). It
        # is a training-range knob, so on a plant that separates the two
        # (policy_range()) it moves the ctrlrange and leaves the mechanical
        # stop alone; on the legacy plants ctrlrange does not exist and it
        # moves jnt_range exactly as it always did.
        if hip_flex_deg is not None:
            for i in (self._jname2i["L_hip_pitch"], self._jname2i["R_hip_pitch"]):
                if self.model.actuator_ctrllimited[i]:
                    self.model.actuator_ctrlrange[i, 0] = -np.deg2rad(hip_flex_deg)
                else:
                    self.model.jnt_range[jnt[i], 0] = -np.deg2rad(hip_flex_deg)
        self.hip_flex_deg = hip_flex_deg
        if action_map not in ("legacy", "full"):
            raise ValueError(f"unknown action_map {action_map!r}")
        self.action_map = action_map
        self._lo, self._hi = policy_range(self.model)
        self._default = self.model.qpos0[self._jqpos].copy()        # standing pose
        self._scale = 0.5 * (self._hi - self._lo)              # per-joint residual

        self._up_id = self.model.sensor("torso_up").adr[0]
        self._nominal_h = float(self.model.body("torso").pos[2])  # 0.28 m
        if joint_frictionloss > 0:
            self.model.dof_frictionloss[self._jqvel] = joint_frictionloss
        if joint_armature > 0:
            self.model.dof_armature[self._jqvel] = joint_armature
        self._foot_bids = (self.model.body("L_foot").id,
                           self.model.body("R_foot").id)
        # soft joint limits (90% of range) for the dof-limit penalty. Measured
        # against the POLICY range, not the mechanical stop: the penalty exists
        # to keep a policy off the edge of what it may command, and reading it
        # off a widened stop would quietly relax the shaping on a split plant.
        _mid = 0.5 * (self._lo + self._hi)
        _half = 0.5 * (self._hi - self._lo)
        self._soft_lo = _mid - 0.9 * _half
        self._soft_hi = _mid + 0.9 * _half
        if True:   # sole/foot reference constants (used by ext skills AND
            # the plan-v2 feet-phase reward)
            d0 = mujoco.MjData(self.model)
            d0.qpos[:] = self.model.qpos0
            mujoco.mj_forward(self.model, d0)
            self._foot_rel0 = np.stack([
                d0.geom_xpos[self._sole_gids[0]] - d0.xpos[self._torso_bid],
                d0.geom_xpos[self._sole_gids[1]] - d0.xpos[self._torso_bid]])
            # standing sole-center height: lift clearance reference
            self._sole_z0 = float(d0.geom_xpos[self._sole_gids[0]][2])
            # KNEE-HIGH reference, MEASURED on the plant (not guessed):
            # world z of the knee joint's anchor in the standing pose. On
            # bimo_biped_v3yaw.xml that is 0.107 m, with the sole center at
            # 0.003 m -> a knee-high swing sole must clear 0.104 m, i.e.
            # 1.7x the 0.06 m nominal lift_height / 3.5x the 0.03 m
            # lift_clear floor. Mirrored in sim/mjx/env_mjx.py.
            _knee_jid = self.model.joint(
                self._act_names[self._legL["knee"]]).id
            self._knee_z0 = float(d0.xanchor[_knee_jid][2])
            self.march_clear = max(self._knee_z0 - self._sole_z0, 0.0)
            # commanded c6 offset that puts the swing-foot TARGET (which is
            # lift_height + c6 above the standing sole) at knee height
            self._march_dz = max(self.march_clear - self.lift_height, 0.0)
        # Fixed-cadence march clock (march_hz > 0), expressed in WHOLE control
        # steps. th_m = 2*pi*march_hz*step_i*control_dt is the definition, but
        # evaluating it that way puts the c4 sign flip exactly on sin(th_m)=0,
        # where float32 (MJX, training) and float64 (CPU referee) land on
        # OPPOSITE sides -- a whole-leg command disagreement at every flip.
        # So the half-cycle is rounded to an integer number of control steps
        # once, here, and the flip is an exact integer compare in both
        # engines; the |sin| envelope is then evaluated on the wrapped step
        # count, which keeps the float argument inside [0, 2*pi) and the two
        # engines within ~1e-7 of each other. Mirrored in sim/mjx/env_mjx.py.
        self._march_half = (max(1, int(round(0.5 / (self.march_hz
                                                    * self.control_dt))))
                            if self.march_hz > 0.0 else 0)
        self._march_period = 2 * self._march_half

        self.action_space = spaces.Box(-1.0, 1.0, shape=(self._nq_act,),
                                       dtype=np.float32)
        # per-joint blocks (qpos, qvel, prev_action) scale with the joint
        # count; the rest is fixed. 8-DOF: 36 + ncmd; 10-DOF hip-yaw: 42 + ncmd
        self.obs_frame = (3 * self._nq_act + 3 + 3 + 3 + 1 + 2
                          + (self._ncmd if command_mode else 0))
        obs_dim = self.obs_frame * self.obs_hist_len
        self.observation_space = spaces.Box(
            -np.inf, np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.render_mode = render_mode
        self._renderer = None
        # Optional decorative floor marker (green square) at a stored (x, y);
        # None = off. Drawn as a non-colliding mjvGeom in render() so it needs
        # no XML/model change and cannot touch physics. Set e.g. to the
        # episode's start position so a "return to start" is visually gradeable.
        self.mark_xy = None
        self._prev_action = np.zeros(self._nq_act, dtype=np.float32)
        self._air_time = np.zeros(2)   # per-foot time since last ground contact
        self._step_i = 0

    # -- helpers -----------------------------------------------------------
    def _action_to_ctrl(self, action: np.ndarray) -> np.ndarray:
        action = np.clip(action, -1.0, 1.0)
        if self.action_map == "full":
            # piecewise-linear: full range on both sides, action 0 = stand
            span = np.where(action >= 0.0, self._hi - self._default,
                            self._default - self._lo)
            return self._default + span * action
        return np.clip(self._default + self._scale * action, self._lo, self._hi)

    # -- servo tick quantization (mirrored in sim/mjx/env_mjx.py) -----------
    def _quant_angle(self, rad):
        """rad -> encoder tick (round-half-to-even, clipped to 0..4095) -> rad.

        obs::angleToSteps followed by obs::stepsToAngle under the nominal
        calibration (zero_steps 2048, dir +1) -- see harness.angle_to_steps /
        steps_to_angle. np.rint is round-half-to-even, matching lrintf()'s
        default mode (and jnp.round on the MJX side, which is what makes the
        parity gate bit-identical). No joint-range clamp here: the harness
        clamps because the firmware does, but every angle this env feeds
        through is already inside [lo, hi] (the physics enforce it for
        observations, _action_to_ctrl for targets)."""
        ticks = np.clip(_TICK_ZERO + np.rint(np.asarray(rad, dtype=np.float64)
                                             / _TICK_RAD * self._tick_dir),
                        0.0, float(_TICK_MAX))
        return (ticks - _TICK_ZERO) * _TICK_RAD * self._tick_dir

    def _quant_vel(self, rad_s):
        """rad/s -> integer reg-58 steps/s -> rad/s (harness.rad_s_to_steps_s /
        steps_s_to_rad_s). The wire word is sign-magnitude with a 15-bit
        magnitude, so the reachable range is +/-32767 steps/s (~+/-50 rad/s --
        never binding on this plant, but it is the boundary's real limit)."""
        steps = np.clip(np.rint(np.asarray(rad_s, dtype=np.float64)
                                / _TICK_RAD * self._tick_dir),
                        -float(_TICK_VEL_MAX), float(_TICK_VEL_MAX))
        return steps * _TICK_RAD * self._tick_dir

    # -- terrain -------------------------------------------------------------
    @staticmethod
    def _terrain_xml(xml_path: str, amplitude: float,
                     nrow: int = _HF_NROW, ncol: int = _HF_NCOL,
                     rx: float = _HF_RX, ry: float = _HF_RY,
                     cx: float = _HF_CX) -> str:
        """Return the MJCF with the flat floor plane replaced by an hfield geom
        (same name 'floor' so friction DR keeps working) plus its asset.
        Grid defaults are the legacy Stage-2b field; the mosaic passes its
        own dimensions from terrain_mosaic.npz."""
        with open(xml_path) as f:
            xml = f.read()
        asset = (f'<asset><hfield name="terrain" nrow="{nrow}" '
                 f'ncol="{ncol}" size="{rx} {ry} {amplitude} 0.1"/>'
                 f'</asset>\n  ')
        # keep the original floor's appearance (v2 has a checker material,
        # v1 a plain rgba) so terrain runs give the same motion cues
        orig = re.search(r'<geom name="floor"[^>]*?/>', xml, flags=re.S)
        look = re.search(r'(material="[^"]+"|rgba="[^"]+")', orig.group(0)) if orig else None
        geom = (f'<geom name="floor" type="hfield" hfield="terrain" '
                f'pos="{cx} 0 0" contype="1" conaffinity="3" '
                # carpet grips (user 2026-08-01): sliding 1.3 vs the smooth
                # plane's 1.0, torsional 0.05 (pile engagement resists yaw
                # pivots) -- pair friction = elementwise max vs the pads'
                # 1/0.02/0.001. Bench-measure both when the robot walks on
                # the real carpet; mirrors sim/mjx _terrain_patch EXACTLY.
                f'{look.group(1) if look else ""} friction="1.3 0.05 0.001" '
                f'condim="4"/>')
        patched, n = re.subn(r'<geom name="floor"[^>]*?/>', geom, xml, flags=re.S)
        if n != 1:
            raise ValueError(f"expected exactly one floor geom in {xml_path}, found {n}")
        return patched.replace("<worldbody>", asset + "<worldbody>", 1)

    @staticmethod
    def _payload_xml(xml: str, mass: float, cg_z: float = 0.08,
                     cg_x: float = 0.0) -> str:
        """Insert the camera payload as its own (jointless, i.e. welded) child
        body of the torso, so its mass/inertia stay separately addressable at
        runtime for per-episode payload randomization."""
        # cg_x: the camera bolts to gopro_base at GP_MOUNT_X, not on the
        # centreline (fixed 2026-08-06; x was hard-coded to 0).
        body = (f'<body name="payload" pos="{cg_x} 0 {cg_z}">'
                f'<geom name="payload" type="box" size="0.0125 0.032 0.0345" '
                f'mass="{mass}" contype="2" conaffinity="0" '
                f'rgba="0.12 0.12 0.14 1"/></body>')
        patched, n = re.subn(r'(<site name="imu"[^>]*/>)', lambda m: m.group(1) + body,
                             xml, count=1)
        if n != 1:
            raise ValueError("expected exactly one imu site to anchor the payload")
        return patched

    def _generate_terrain(self):
        """Re-randomize the heightfield: smoothed uniform noise, normalized to
        [0,1] (world height = data * amplitude), with the spawn area flattened
        to 0 and a smoothstep ramp so the robot starts on level ground.
        On CPU MuJoCo, collisions read model.hfield_data directly each step, so
        rewriting it before mj_resetData is sufficient (verified by test)."""
        dx = 2 * _HF_RX / (_HF_NCOL - 1)
        field = self.np_random.uniform(0.0, 1.0, (_HF_NROW, _HF_NCOL))
        field = _gauss_smooth(field, self.terrain_smoothness / dx)
        field -= field.min()
        field /= max(float(np.ptp(field)), 1e-9)
        # flat pad within |x| < 0.3 m of spawn, full roughness beyond 0.9 m
        x_world = (_HF_CX - _HF_RX) + np.arange(_HF_NCOL) * dx
        t = np.clip((np.abs(x_world) - 0.3) / 0.6, 0.0, 1.0)
        field *= (t * t * (3.0 - 2.0 * t))[None, :]
        if self.terrain_amplitude_min is not None:
            # per-episode difficulty draw: scale bump height down from the max
            # (hfield size[2] stays terrain_amplitude; data is in [0, scale])
            ep_amp = self.np_random.uniform(self.terrain_amplitude_min,
                                            self.terrain_amplitude)
            field *= ep_amp / self.terrain_amplitude
        self.model.hfield_data[:] = field.ravel()
        self._hf_grid = field
        self._terrain_dirty = True

    def _ground_z(self, x: float, y: float) -> float:
        """Terrain height (m) under world point (x, y); 0 on the flat plane or
        off the field. Bilinear interpolation of the cached grid."""
        if self._hf_grid is None:
            return 0.0
        nrow, ncol, rx, ry, cx = self._hf_dims
        c = (x - (cx - rx)) / (2 * rx) * (ncol - 1)
        r = (y + ry) / (2 * ry) * (nrow - 1)
        if not (0.0 <= c <= ncol - 1 and 0.0 <= r <= nrow - 1):
            return 0.0
        c0, r0 = int(c), int(r)
        c1, r1 = min(c0 + 1, ncol - 1), min(r0 + 1, nrow - 1)
        fc, fr = c - c0, r - r0
        g = self._hf_grid
        h = ((1 - fr) * ((1 - fc) * g[r0, c0] + fc * g[r0, c1])
             + fr * ((1 - fc) * g[r1, c0] + fc * g[r1, c1]))
        return float(h) * self.terrain_amplitude

    def _foot_contacts(self) -> tuple[bool, bool]:
        """(left, right) sole-in-contact flags. The pad geoms are the only ones
        with floor collisions enabled on the feet, so any contact involving one
        of them is ground contact."""
        n = self.data.ncon
        if n == 0:
            return False, False
        g1 = self.data.contact.geom1[:n]
        g2 = self.data.contact.geom2[:n]
        return tuple(bool(np.any(np.isin(g1, gids) | np.isin(g2, gids)))
                     for gids in self._pad_gids)

    def _randomize_dynamics(self):
        """Re-sample mass/inertia, floor friction, and actuator gain from nominal.
        Called on reset so the policy must be robust to model error -- the reason
        DR is required before sim-to-real. All draws are re-derived from the
        stored nominals, never compounded."""
        rng, m = self.np_random, self.model
        mf = rng.uniform(1 - self.mass_range, 1 + self.mass_range, size=m.nbody)
        m.body_mass[:] = self._nom_mass * mf
        m.body_inertia[:] = self._nom_inertia * mf[:, None]
        ff = rng.uniform(1 - self.friction_range, 1 + self.friction_range)
        m.geom_friction[self._floor_gid, 0] = self._nom_friction[self._floor_gid, 0] * ff
        gf = rng.uniform(1 - self.gain_range, 1 + self.gain_range)
        # Position actuator: gainprm[0]=kp, biasprm[1]=-kp -- scale both together.
        # (Under actuator_model="sts3215" the MJCF actuators are zeroed, so this
        # is a no-op there; the servo parameters are randomized instead.)
        m.actuator_gainprm[:] = self._nom_gain * gf
        m.actuator_biasprm[:] = self._nom_bias * gf
        if self._servo is not None:
            # kp/kd follow the actuator-gain range (+/-20% default); the motor
            # envelope (stall torque, no-load speed) gets its own +/-15% range.
            sf = np.concatenate([
                rng.uniform(1 - self.gain_range, 1 + self.gain_range, size=2),
                rng.uniform(1 - self.servo_range, 1 + self.servo_range, size=2)])
            self._servo = self._nom_servo * sf

    def _obs(self) -> np.ndarray:
        d = self.data
        up = d.sensordata[self._up_id : self._up_id + 3]
        gyro = d.qvel[3:6]
        linvel = d.qvel[0:3]
        height = d.qpos[2] - self._ground_z(d.qpos[0], d.qpos[1])
        if self.imu_obs:
            # what the real robot can sense: attitude + rates via the IMU
            # (misaligned, biased, noisy under DR); linear velocity and height
            # have no sensor -> zeroed (obs stays 36-wide for warm starts)
            noisy = self.domain_rand and self.imu_noise > 0.0
            up = self._imu_R @ up
            gyro = gyro + self._imu_gyro_bias
            if noisy:
                up = up + self.np_random.normal(0.0, 0.01 * self.imu_noise, 3)
                gyro = gyro + self.np_random.normal(0.0, 0.03 * self.imu_noise, 3)
            linvel = np.zeros(3)
            height = 0.0
        if self.gait_clock:
            phase = self._gait_phase
        else:
            phase = 2 * np.pi * (self._step_i / self.max_steps)
        q_j = d.qpos[self._jqpos]
        dq_j = d.qvel[self._jqvel]
        if self.quantize_ticks:
            # servo-side view: what a SYNC READ can actually report (encoder
            # ticks + reg-58 steps/s). The physics state is untouched.
            q_j = self._quant_angle(q_j)
            dq_j = self._quant_vel(dq_j)
        parts = [
            q_j,                                    # 8 joint angles
            dq_j,                                   # 8 joint velocities
            up,                                # 3 torso up-vector
            linvel,                            # 3 torso linear velocity
            gyro,                              # 3 torso angular velocity
            self._prev_action,                 # 8 last action
            # torso height above *local ground* (== world z on flat ground, so
            # flat-ground policies see the exact same observation as before)
            [height],
            [np.sin(phase), np.cos(phase)],    # 2 phase clock
        ]
        if self.command_mode:
            parts.append(self._cmd)            # 2 commanded (vx, yaw rate)
        frame = np.concatenate(parts).astype(np.float32)
        if self.obs_hist_len > 1:
            # pure read: history is updated explicitly in reset()/step()
            return np.concatenate([frame] + list(self._obs_hist))
        return frame

    def _phase_cmd(self):
        """Numpy mirror of sim/mjx _phase_cmd (getup_v4, parity-gated): on
        recovery episodes expose the rise-schedule phase through command
        channel c5 (0..1 over the scripted rise, held at 1 once recovered)."""
        if not (self.ext_cmd and self.w_rise_ref and self._recover_ep):
            return
        phase = float(np.clip(
            (self._rise_t0 + self._step_i * self.control_dt)
            / self.rise_secs, 0.0, 1.0))
        self._cmd[5] = max(phase, 1.0 if self._recovered else 0.0)

    def _rise_ref(self, t_s):
        """Numpy mirror of sim/mjx BimoMJXEnv._rise_ref (parity-gated)."""
        d = self._default
        stages = np.array([
            [-1.92, -1.62, -0.55],
            [-0.20, -1.62, -0.60],
            [-0.80, -0.80, 0.10],
        ])
        stand = np.array([d[self._i_pitch][0], d[self._i_knee][0],
                          d[self._i_ankle][0]])
        keys = np.concatenate([stages, stand[None]], axis=0)
        u = float(np.clip(t_s / self.rise_secs, 0.0, 1.0)) * 3.0
        i0 = int(np.clip(np.floor(u), 0, 2))
        w = u - i0
        tgt = keys[i0] * (1.0 - w) + keys[i0 + 1] * w
        q = d.copy()
        q[self._i_pitch] = tgt[0]
        q[self._i_knee] = tgt[1]
        q[self._i_ankle] = tgt[2]
        return np.clip(q, self._lo, self._hi)

    def _mimic_ref(self, cmd, phase, freq):
        """Numpy mirror of sim/mjx BimoMJXEnv._mimic_ref (parity-gated)."""
        f = max(float(freq), 0.5)
        d = self._default
        s, c = np.sin(phase), np.cos(phase)
        sw = np.array([max(0.0, s), max(0.0, -s)])
        xn = np.array([-c, c])
        A = np.clip(1.4 * (cmd[0] + np.array([-1.0, 1.0]) * 0.028 * cmd[2])
                    / f, -0.45, 0.45)
        B = float(np.clip(1.1 * cmd[1] / f, -0.3, 0.3))
        hp0 = d[self._i_pitch]                  # hip-pitch defaults (L, R)
        kn0 = d[self._i_knee]
        hipP = hp0 - A * xn
        # knee swing-bend scales with commanded activity: zero command ->
        # the reference IS the standing pose (no knee pumping)
        mag = min(1.0, (float(np.max(np.abs(A))) + abs(B)) / 0.35)
        knee = kn0 - 0.55 * mag * sw
        roll = d[self._i_roll] + B * xn
        ank = d[self._i_ankle] - (hipP - hp0) - (knee - kn0)
        # scatter the per-leg references onto the standing pose; any hip_yaw
        # joints stay at their default (0.0) -> neutral regularization
        q = d.copy()
        q[self._i_roll] = roll
        q[self._i_pitch] = hipP
        q[self._i_knee] = knee
        q[self._i_ankle] = ank
        return np.clip(q, self._lo, self._hi)

    def _sample_command(self):
        """New (vx, yaw-rate) command. Mix: stand / pivot-in-place / walk
        (straight or turning). Pivot commands matter: with no hip-yaw joint,
        pivoting is this morphology's easiest turn -- leaving it out of the
        training distribution left cmd_11v1/b unable to track yaw at all."""
        if self.cmd_fixed is not None:
            self._cmd = np.asarray(self.cmd_fixed, dtype=float)
            if len(self._cmd) != self._ncmd:
                raise ValueError(f"cmd_fixed needs {self._ncmd} channels")
            self._cmd_next = 10 ** 9
            self._traj_on = 0.0
            self._apply_crouch_draw()
            return
        if self.ext_cmd:
            self._sample_command_ext()
            return
        u = float(self.np_random.uniform())
        if u < self.cmd_stand_prob:
            self._cmd = np.zeros(2)
        elif u < self.cmd_stand_prob + 0.15:
            w = float(self.np_random.uniform(0.3, self.cmd_w_range))
            self._cmd = np.array([0.0, w if self.np_random.uniform() < 0.5
                                  else -w])
        else:
            v = float(self.np_random.uniform(*self.cmd_v_range))
            w = (float(self.np_random.uniform(-self.cmd_w_range,
                                              self.cmd_w_range))
                 if self.np_random.uniform() < 0.6 else 0.0)
            self._cmd = np.array([v, w])
        lo, hi = self.cmd_resample_s
        self._cmd_next = self._step_i + int(
            self.np_random.uniform(lo, hi) / self.control_dt)

    def _sample_command_ext(self):
        """7-channel precision command draw; mirrors sim/mjx _sample_cmd_ext
        (same mix and ranges; RNG streams differ by construction)."""
        rng = self.np_random
        u = float(rng.uniform())
        t = np.cumsum(self.ext_mix)    # stand, crouch, bal, circle, pivot,
        cmd = np.zeros(7)              # march, sway, knee-high march;
        cmd[3] = 1.0                   # remainder = walk
        t7 = float(t[6]) + self.march_mix     # knee-high march slice end
        self._traj_on = 0.0            # mode: 0 off, 1 circle, 2 march,
                                       # 3 sway, 4 knee-high march
        if u < t[0]:
            pass                                       # stand
        elif u < t[1]:                                 # crouch hold
            cmd[3] = float(rng.uniform(*self.crouch_range))
        elif u < t[3]:                                 # one-leg balance/circle
            cmd[4] = 1.0 if rng.uniform() < 0.5 else -1.0
            if u >= t[2]:
                self._traj_on = 1.0                    # air circle (c5/c6 ride)
        elif u < t[4]:                                 # pivot in place
            w = float(rng.uniform(0.3, self.cmd_w_range))
            cmd[2] = w if rng.uniform() < 0.5 else -w
        elif u < t[5]:                                 # march in place
            self._traj_on = 2.0
        elif u < t[6]:                                 # lateral sway
            self._traj_on = 3.0
        elif u < t7:                                   # knee-high march
            self._traj_on = 4.0
        else:                                          # walk fwd / back / side
            uw = float(rng.uniform())
            p_back, p_side = self.walk_submix
            if uw < p_back:
                cmd[0] = float(rng.uniform(*self.cmd_back_range))
            elif uw < p_back + p_side:
                vy = float(rng.uniform(0.1, self.cmd_vy_range))
                cmd[1] = vy if rng.uniform() < 0.5 else -vy
            else:
                cmd[0] = float(rng.uniform(*self.cmd_v_range))
                if rng.uniform() < (0.85 if self.turn_emph else 0.6):
                    wz = float(rng.uniform(-self.cmd_w_range,
                                           self.cmd_w_range))
                    if self.turn_emph:   # mirrors sim/mjx: |wz| floored 0.25
                        wz = float(np.sign(wz)) * (
                            0.25 + abs(wz) * (self.cmd_w_range - 0.25)
                            / self.cmd_w_range)
                    cmd[2] = wz
        self._cmd = cmd
        lo, hi = self.cmd_resample_s
        self._cmd_next = self._step_i + int(
            self.np_random.uniform(lo, hi) / self.control_dt)
        self._apply_crouch_draw()

    def _apply_crouch_draw(self):
        """Replace a FROZEN full-height crouch channel with this episode's
        draw (SIL finding #2). Only the exactly-1.0 case is replaced, so an
        ext_mix crouch command keeps its own crouch_range draw; set_command()
        never routes through here, so scenario evals are untouched. Mirrored
        in sim/mjx/env_mjx.py (_apply_crouch)."""
        if not self._crouch_dr:
            return
        if float(self._cmd[3]) == 1.0:
            self._cmd[3] = self._cmd_crouch

    def set_command(self, *chans: float):
        """External command override (scenario evals / teleop or goal-seeking
        layers -- see link/). Disables auto-resampling until the next reset.
        Legacy envs take (v, w); ext_cmd envs take up to 7 channels (vx, vy,
        wz, crouch, lift, foot_dx, foot_dz) -- omitted trailing channels
        default to (0, 0, 0, 1, 0, 0, 0). NOTE the channel meaning shift:
        channel 1 is yaw rate on legacy envs but vy on ext envs."""
        base = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])[:self._ncmd]
        base[:len(chans)] = [float(c) for c in chans]
        self._cmd = base
        self._cmd_next = 10 ** 9
        self._traj_on = 0.0

    def set_torque_enabled(self, on: bool):
        """Release / re-engage servo torque, both actuator models.

        The hardware analogue is the STS3215's torque-enable register (the
        "Release" step in docs/wiring.md's bring-up checklist), and the
        wireless link's watchdog needs it: a link dead for seconds must not
        leave eight servos cooking at stall to hold a pose nobody is asking
        for. Limp is the safe state off-link -- the robot is 28 cm tall and
        falls better than it overheats.

        Note the ideal-actuator path edits model gains, which a reset under
        domain_rand re-randomizes from nominal (i.e. re-enables). The link
        agent re-asserts this every tick, so it does not drift.
        """
        on = bool(on)
        if on == self._torque_on:
            return
        self._torque_on = on
        # Passive static friction bump while released (both actuator models):
        # a powered-off STS3215 resists backdriving far more than the powered
        # BAM-fit frictionloss. dof_frictionloss is untouched by DR, so the
        # saved value restores cleanly on re-engage.
        if on:
            if self._held_frictionloss is not None:
                self.model.dof_frictionloss[self._jqvel] = self._held_frictionloss
                self._held_frictionloss = None
        else:
            self._held_frictionloss = \
                self.model.dof_frictionloss[self._jqvel].copy()
            self.model.dof_frictionloss[self._jqvel] = self.off_frictionloss
        if self.actuator_model == "ideal":
            if on:
                # Restore what was there, NOT _nom_gain: under DR this episode
                # is running scaled gains, and re-engaging must not silently
                # hand the policy a different plant than it fell asleep on.
                self.model.actuator_gainprm[:] = self._held_gain
                self.model.actuator_biasprm[:] = self._held_bias
            else:
                # Silence the MJCF position actuators the same way the sts3215
                # model silences them permanently at construction.
                self._held_gain = self.model.actuator_gainprm.copy()
                self._held_bias = self.model.actuator_biasprm.copy()
                self.model.actuator_gainprm[:] = 0.0
                self.model.actuator_biasprm[:] = 0.0
        # sts3215 torque is recomputed per substep in step(), gated on the flag.

    # -- gym API -----------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        # Every episode starts powered: the torque-off idle is cut mid-episode
        # by set_torque_enabled(False), and a cached/reused env must not inherit
        # a released state (and its raised frictionloss) from a prior episode.
        if not self._torque_on:
            self.set_torque_enabled(True)
        if self.domain_rand:
            self._randomize_dynamics()
        if self._payload_bid is not None and self.payload_max:
            # per-episode payload draw (after DR, which rewrites all body
            # masses from nominal): uniform 0..payload_max, inertia scaled
            # with mass (fixed geometry). 0 kg = camera off; welded body, so
            # zero mass is numerically safe.
            mp = self.np_random.uniform(0.0, self.payload_max)
            self.model.body_mass[self._payload_bid] = mp
            self.model.body_inertia[self._payload_bid] = \
                self._nom_inertia[self._payload_bid] * (mp / _PAYLOAD_REF)
        if self.terrain_amplitude > 0 and self._mosaic is None:
            self._generate_terrain()   # before mj_resetData; CPU collisions read it live
        mujoco.mj_resetData(self.model, self.data)
        self._recover_ep = False
        self._recovered = True
        self._stand_streak = 0.0       # mirrors State.stand_streak
        self._rise_t0 = 0.0            # rise-phase offset (mirrors State.rise_t0)
        if self.ext_cmd and self.recover_mix > 0:
            self._recover_ep = bool(self.np_random.uniform()
                                    < self.recover_mix)
            self._recovered = not self._recover_ep
        if self.getup or self._recover_ep:
            # start-state draw for recovery episodes (getup mode stays pure
            # ragdoll -- that is the graded claim). Poses mirror
            # sim/mjx/_fallen_data exactly.
            kind = "ragdoll"
            if self._recover_ep:
                mix = tuple(self.recover_start_mix) + (0.0,) * 5
                u = float(self.np_random.uniform())
                if u < mix[0]:
                    kind = "ragdoll"
                elif u < mix[0] + mix[1]:
                    kind = "kneel"
                elif u < mix[0] + mix[1] + mix[2]:
                    kind = "squat"
                elif u < mix[0] + mix[1] + mix[2] + mix[3]:
                    kind = "sit"
                else:
                    kind = "catch"
            self.data.qpos[:] = self.model.qpos0
            self.data.qvel[:] = 0.0
            # staged poses settle 1.0 s holding their own pose (getup_v7):
            # at 0.25 s the kneel is mid-bounce at h 0.277, 2 mm above the
            # recovered threshold -- the episode began instantly recovered
            settle_n = 200
            # named-joint pose builders (any hip_yaw joints stay at 0)
            j = np.zeros(self._nq_act)
            if kind == "ragdoll":
                u1, u2, u3 = self.np_random.uniform(size=3)
                tp = 2 * np.pi
                quat = np.array([np.sqrt(u1) * np.cos(tp * u3),
                                 np.sqrt(1 - u1) * np.sin(tp * u2),
                                 np.sqrt(1 - u1) * np.cos(tp * u2),
                                 np.sqrt(u1) * np.sin(tp * u3)])
                joints = np.clip(self._default + self.np_random.uniform(
                    -0.6, 0.6, size=self._nq_act) * self._scale,
                    self._lo, self._hi)
                self.data.qpos[2] = 0.35
                self.data.qpos[3:7] = quat
                self.data.qpos[self._jqpos] = joints
                settle_n = self._getup_settle
            elif kind == "kneel":
                j[self._i_pitch] = -0.2
                j[self._i_knee] = -1.62
                j[self._i_ankle] = -0.6
                self.data.qpos[2] = 0.13
                self.data.qpos[self._jqpos] = np.clip(j, self._lo, self._hi)
                # getup_v7: kneel start == reference stage 1 (high-kneel)
                self._rise_t0 = self.rise_secs / 3.0
            elif kind == "squat":
                j[self._i_pitch] = self._lo[self._legL["hip_pitch"]] + 0.05
                j[self._i_knee] = -1.62
                j[self._i_ankle] = 0.65
                ang = 0.55
                self.data.qpos[2] = 0.10
                self.data.qpos[3:7] = [np.cos(ang / 2), 0, np.sin(ang / 2), 0]
                self.data.qpos[self._jqpos] = np.clip(j, self._lo, self._hi)
            elif kind == "catch":      # v6/v7: harvested RSI bank state
                z = self._catch_states
                row = int(self.np_random.integers(z["qpos"].shape[0]))
                self.data.qpos[:] = z["qpos"][row]
                self.data.qvel[:] = z["qvel"][row]
                settle_n = 0           # momentum IS the start state
                self._rise_t0 = (float(z["t0"][row]) if "t0" in z.files
                                 else self.rise_secs / 3.0)
            else:                      # sit: torso UP, waist 90 deg, legs
                j[self._i_pitch] = -1.57   # out front, feet splayed apart
                j[self._i_knee] = -0.09    # (mirrors sim/mjx; verified by
                j[self._legL["hip_roll"]] = 0.10    # check_sit_pose.py)
                j[self._legR["hip_roll"]] = -0.10
                self.data.qpos[2] = 0.08
                self.data.qpos[self._jqpos] = np.clip(j, self._lo, self._hi)
            # getup_v7: staged poses settle holding their OWN pose (mirrors
            # sim/mjx _settle ctrl arg) -- under stand-drive the kneel start
            # settled to h 0.28 > the recovered threshold, so every kneel
            # episode began recovered=1 and trained nothing
            self.data.ctrl[:] = (np.clip(j, self._lo, self._hi)
                                 if kind in ("kneel", "squat", "sit")
                                 else self._default)
            for _ in range(settle_n):
                mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
        else:
            # small noise so the policy can't memorize one trajectory
            self.data.qpos[:] = self.model.qpos0
            self.data.qpos[self._jqpos] += self.np_random.uniform(
                -0.03, 0.03, size=self._nq_act)
            if self._mosaic is not None:
                # mosaic spawn: per-episode roughness = per-episode location
                # (mirrors sim/mjx _terrain_spawn; margins keep the episode
                # on the field, z rides the local ground)
                sx = self.np_random.uniform(-0.5, 4.0)
                sy = self.np_random.uniform(-2.0, 2.0)
                self.data.qpos[0] += sx
                self.data.qpos[1] += sy
                self.data.qpos[2] += self._ground_z(sx, sy)
            self.data.qvel[:] = self.np_random.uniform(-0.02, 0.02, size=self.model.nv)
            self.data.ctrl[:] = self._default
            mujoco.mj_forward(self.model, self.data)
        self._prev_action[:] = 0.0
        # recovery height ratchet: episode-best torso height (mirrors sim/mjx
        # State.best_h; only NEW height above this pays while down)
        self._best_h = float(self.data.qpos[2])
        # heading integrator reference (mirrors sim/mjx State.head_ref)
        q = self.data.qpos[3:7]
        self._head_ref = float(np.arctan2(
            2 * (q[0] * q[3] + q[1] * q[2]),
            1 - 2 * (q[2] * q[2] + q[3] * q[3])))
        # action-latency buffer: past ctrl targets, applied `action_latency` steps late
        self._ctrl_buf = [self._default.copy() for _ in range(self.action_latency + 1)]
        # sub-step latency: per-episode draw (only draws RNG when enabled, so
        # disabled runs consume the identical random stream as before) and the
        # target that was in force before this control step (= stand at reset)
        if self.latency_ms_max is not None:
            self._lat_ms_ep = float(self.np_random.uniform(self.latency_ms,
                                                           self.latency_ms_max))
        else:
            self._lat_ms_ep = self.latency_ms
        if self.backlash_deg_max is not None:
            self._lash_rad = np.deg2rad(float(self.np_random.uniform(
                self.backlash_deg, self.backlash_deg_max)))
        if self._crouch_dr:
            # per-episode crouch-command draw (SIL finding #2). Drawn HERE,
            # with the other per-episode DR draws and before _sample_command()
            # consumes it; only draws RNG when the feature is on, so default
            # runs keep their exact random streams.
            self._cmd_crouch = float(self.np_random.uniform(
                *self.cmd_crouch_range))
        self._last_target = self._default.copy()
        self._air_time[:] = 0.0
        self._last_air[:] = 0.0
        self._servo_tau[:] = 0.0
        self._cross_t = None    # dash: time the torso first crossed dash_distance
        self._stand_t = None    # dash_stop: start of the current standstill
        if self.command_mode:
            self._sample_command()
        if self._recover_ep:
            # recovery episode: stand command pinned for the whole episode
            self._cmd = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            self._cmd_next = 10 ** 9
            self._traj_on = 0.0
        if self.ext_cmd:
            # per-episode air-circle parameters (mirrors sim/mjx _draw_traj)
            rad = float(self.np_random.uniform(*self.traj_radius))
            per = float(self.np_random.uniform(*self.traj_period))
            omega = (2 * np.pi / per) * (1.0 if self.np_random.uniform() < 0.5
                                         else -1.0)
            self._traj = np.array([rad, omega,
                                   float(self.np_random.uniform(0, 2 * np.pi))])
        # IMU DR: per-episode mounting misalignment (small random rotation) and
        # gyro bias, BNO085-class magnitudes scaled by imu_noise. Only draws
        # RNG when enabled, so other runs keep their exact random streams.
        if self.imu_obs and self.domain_rand and self.imu_noise > 0.0:
            ang = self.np_random.normal(0.0, np.deg2rad(2.0)) * self.imu_noise
            ax = self.np_random.normal(size=3)
            ax /= np.linalg.norm(ax) + 1e-9
            k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]],
                          [-ax[1], ax[0], 0]])
            self._imu_R = np.eye(3) + np.sin(ang) * k + (1 - np.cos(ang)) * k @ k
            self._imu_gyro_bias = (self.np_random.uniform(-0.03, 0.03, 3)
                                   * self.imu_noise)
        self._step_i = 0
        self._phase_cmd()
        if self.gait_clock:
            self._gait_freq = float(self.np_random.uniform(1.25, 1.75))
            self._gait_phase = float(self.np_random.uniform(-np.pi, np.pi))
        if self.obs_hist_len > 1:
            self._obs_hist = []
            first = self._obs()                # frame only (hist empty)
            self._obs_hist = [first.copy()
                              for _ in range(self.obs_hist_len - 1)]
        return self._obs(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        target = self._action_to_ctrl(action)
        if self.quantize_ticks:
            # the firmware writes INTEGER goal ticks (SYNC WRITE), so the
            # servo never sees the float target -- quantize before the
            # actuator path (latency buffer, PD, backlash) touches it
            target = self._quant_angle(target)
        if self.action_latency:                       # apply a delayed target
            self._ctrl_buf.append(target)
            target = self._ctrl_buf.pop(0)
        self.data.ctrl[:] = target

        # Random shove: apply a horizontal force to the torso for this control
        # step (models an unexpected push -- key for a robust real-world gait).
        self.data.xfrc_applied[self._torso_bid, :] = 0.0
        if self.push_kick:
            # velocity-kick perturbation (plan v2, Playground style)
            if self.np_random.uniform() < self.push_prob:
                ang = self.np_random.uniform(0, 2 * np.pi)
                mag = float(self.np_random.uniform(*self.kick_range))
                self.data.qvel[0] += mag * np.cos(ang)
                self.data.qvel[1] += mag * np.sin(ang)
        elif self.push_force and self.np_random.uniform() < self.push_prob:
            ang = self.np_random.uniform(0, 2 * np.pi)
            self.data.xfrc_applied[self._torso_bid, 0] = self.push_force * np.cos(ang)
            self.data.xfrc_applied[self._torso_bid, 1] = self.push_force * np.sin(ang)

        # Sub-step latency: the first lat_k substeps still run on the previous
        # control target (lat_k = 0 when the feature is off -> path identical).
        lat_k = 0
        if self._lat_ms_ep > 0.0 or self.latency_jitter_ms > 0.0:
            ms = self._lat_ms_ep
            if self.latency_jitter_ms > 0.0:
                ms += self.np_random.uniform(-self.latency_jitter_ms,
                                             self.latency_jitter_ms)
            ms = float(np.clip(ms, 0.0, self.control_dt * 1000.0))
            lat_k = min(int(round(ms / (self.sim_dt * 1000.0))), self.n_substeps)

        x_before = float(self.data.qpos[0])
        for i in range(self.n_substeps):
            cur = self._last_target if i < lat_k else target
            if self._servo is not None:
                # PD on the commanded angle, clamped each substep to the
                # DC-motor torque-speed envelope (linear stall -> no-load).
                kp, kd, stall, w0 = self._servo
                q = self.data.qpos[self._jqpos]
                qd = self.data.qvel[self._jqvel]
                cap = stall * np.clip(1.0 - np.abs(qd) / w0, 0.0, 1.0)
                err = cur - q
                if self._lash_rad > 0.0:
                    # gear backlash: +/-lash/2 deadzone on the position error
                    # (inside the lash the output gear floats -- no P torque)
                    err = np.sign(err) * np.maximum(
                        np.abs(err) - 0.5 * self._lash_rad, 0.0)
                self._servo_tau = np.clip(kp * err - kd * qd, -cap, cap)
                if not self._torque_on:      # released servos: limp, no hold
                    self._servo_tau = np.zeros(self._nq_act)
                self.data.qfrc_applied[self._jqvel] = self._servo_tau
            elif lat_k:
                # ideal position actuators read data.ctrl -- hold the previous
                # target for the first lat_k substeps, then switch
                self.data.ctrl[:] = cur
            mujoco.mj_step(self.model, self.data)
        self._last_target = target
        x_after = float(self.data.qpos[0])

        d = self.data
        up_z = float(d.sensordata[self._up_id + 2])
        # height above *local ground* -- on terrain, world z rises/falls with the
        # surface, so the fall check and posture term must be terrain-relative
        # (otherwise walking uphill/downhill falsely terminates). On the flat
        # plane _ground_z() is 0 and this is exactly the original world-z height.
        height = float(d.qpos[2]) - self._ground_z(float(d.qpos[0]), float(d.qpos[1]))

        fwd_vel = (x_after - x_before) / self.control_dt
        # Cap the forward term at target_speed so a lunge/faceplant (a big one-step
        # positive velocity) can't out-earn a steady gait -- the classic exploit.
        cap_speed = self.target_speed
        if (self.dash and self.dash_stop and self._cross_t is None
                and cap_speed is not None):
            # approach taper: the speed cap ramps down over the last 0.7 m
            # before the line, so the policy ARRIVES slow enough that braking
            # to a standstill is a small, discoverable step (a full-sprint
            # arrival makes every brake attempt a fall)
            dist_left = self.dash_distance - float(d.qpos[0])
            cap_speed = cap_speed * float(np.clip(dist_left / 0.7, 0.3, 1.0))
        fwd_term = fwd_vel if cap_speed is None else min(fwd_vel, cap_speed)
        # energy ~ actuator force * joint velocity; action-rate penalizes jitter
        # (under the sts3215 model the MJCF actuators are silent, so use the
        # torque we actually applied)
        force = self._servo_tau if self._servo is not None else d.actuator_force
        energy = float(np.sum(np.abs(force) * np.abs(d.qvel[self._jqvel])))
        action_rate = float(np.sum((action - self._prev_action) ** 2))
        # electrical power draw (W): driven mechanical work + copper losses.
        # Computed always (cheap, reported in info); penalized only if w_power.
        qd_j = np.asarray(d.qvel[self._jqvel])
        power_w = float(np.sum(np.maximum(np.asarray(force) * qd_j, 0.0)
                               + self._K_CU * np.asarray(force) ** 2))

        # dash_stop: once the line is crossed the mission changes -- the primary
        # term flips from "go forward" to "brake to a standstill" (same scale:
        # standing earns what running at the cap earned, so crossing is never
        # disincentivized) and the gait-shaping bonuses below switch off.
        braking = self.dash and self.dash_stop and self._cross_t is not None
        vx_body = wz_rate = 0.0
        if self.command_mode:
            # track the commanded body-frame velocity + yaw rate (exp kernels,
            # legged_gym-style). A zero command makes standing the primary
            # reward -- stopping is practiced continuously, not a terminal
            # event, which is what made the dash_stop objective exploitable.
            qw, qx, qy, qz = d.qpos[3:7]
            yaw = np.arctan2(2 * (qw * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))
            cth, sth = np.cos(yaw), np.sin(yaw)
            vx_body = cth * float(d.qvel[0]) + sth * float(d.qvel[1])
            vy_body = -sth * float(d.qvel[0]) + cth * float(d.qvel[1])
            wz_rate = float(d.qvel[5])
            # kernel widths: sigma 0.5 matches the legged_gym reference
            # exp(-err^2/0.25). The first five command runs used sigma 0.25 --
            # HALF the reference width -- which pays ~nothing (0.003) for
            # attempting to walk from standstill vs 0.24 at reference width.
            # Plausibly the root cause of "stands great, won't walk"
            # (found 2026-07-13 when even a pinned-command walk expert froze).
            if self.getup:
                # recovery primary (matches sim/mjx): height + HEIGHT-GATED
                # uprightness progress, standing bonus (half up, half still).
                # The gate keeps the upright kneel from being an absorbing
                # local optimum (see env_mjx.py / DESIGN.md get-up v3).
                planar_g = float(np.hypot(d.qvel[0], d.qvel[1]))
                standing = float((height > 0.85 * self._nominal_h)
                                 and (up_z > 0.9))
                still = float(np.exp(-((planar_g / 0.2) ** 2)))
                up_gate = float(np.clip((height - 0.12) / 0.08, 0.0, 1.0))
                primary = (self.w_recover_h
                           * float(np.clip(height / self._nominal_h, 0.0, 1.0))
                           + self.w_recover_up * 0.5 * (up_z + 1.0) * up_gate
                           + self.stand_bonus * standing * (0.5 + 0.5 * still))
            elif self.ext_cmd:
                # precision tracking (mirrors sim/mjx): joint 2D velocity
                # kernel, yaw kernel, height kernel vs the commanded crouch.
                # cmd_dense = round-2 fix for the precision_v1 do-nothing
                # optimum: dense directional progress under moving commands
                # (see env_mjx.py for the economics).
                sp_cmd = float(np.hypot(self._cmd[0], self._cmd[1]))
                v_kernel = self.w_track_v * float(np.exp(
                    -((vx_body - self._cmd[0]) ** 2
                      + (vy_body - self._cmd[1]) ** 2) / 0.25))
                if self.cmd_dense and sp_cmd > 0.05:
                    v_par = (vx_body * self._cmd[0]
                             + vy_body * self._cmd[1]) / max(sp_cmd, 1e-9)
                    v_term = self.w_track_v * float(np.clip(
                        v_par / max(sp_cmd, 1e-9), -1.0, 1.0))
                else:
                    v_term = v_kernel
                w_term = self.w_track_w * float(np.exp(
                    -((wz_rate - self._cmd[2]) / 0.5) ** 2))
                # integrated-heading kernel (mirrors sim/mjx): facing error
                # vs the integrated commanded heading, inside the gated
                # primary like the rate kernel
                ref_now = self._head_ref + float(self._cmd[2]) * self.control_dt
                head_err = float(np.arctan2(np.sin(yaw - ref_now),
                                            np.cos(yaw - ref_now)))
                w_term = w_term + self.w_heading * float(np.exp(
                    -((head_err / 0.25) ** 2)))
                h_gate = (0.3 if (sp_cmd > 0.05
                                  or abs(self._cmd[2]) > 0.05) else 1.0)
                h_norm = float(np.exp(
                    -(((height - self._cmd[3] * self._nominal_h) / 0.04) ** 2)))
                h_term = self.w_track_h * h_gate * h_norm
                # one-leg geometry pre-reward (mirrors sim/mjx): contact
                # pattern + clearance + swing-foot target kernel
                con_l, con_r = self._foot_contacts()
                lifted = abs(float(self._cmd[4])) > 0.5
                is_r = float(self._cmd[4]) > 0.0
                con_swing = con_r if is_r else con_l
                con_stance = con_l if is_r else con_r
                gid = self._sole_gids[1 if is_r else 0]
                foot_clear = float(d.geom_xpos[gid][2]) - self._sole_z0
                lift_ok = float(lifted and (not con_swing) and con_stance
                                and foot_clear >= self.lift_clear)
                rel_w = d.geom_xpos[gid] - d.xpos[self._torso_bid]
                rel = np.array([cth * rel_w[0] + sth * rel_w[1],
                                -sth * rel_w[0] + cth * rel_w[1], rel_w[2]])
                base = self._foot_rel0[1 if is_r else 0]
                tgt = base + np.array([self._cmd[5], 0.0,
                                       self.lift_height + self._cmd[6]])
                foot_err = float(np.linalg.norm(rel - tgt))
                foot_kernel = float(lifted) * float(np.exp(
                    -((foot_err / self.foot_sigma) ** 2)))
                # tight horizontal foot-under kernel (mirrors sim/mjx)
                d_xy2 = (rel[0] - tgt[0]) ** 2 + (rel[1] - tgt[1]) ** 2
                foot_under = (float(lifted)
                              * float(np.clip(foot_clear / self.lift_clear,
                                              0.0, 1.0))
                              * float(np.exp(-d_xy2 / (0.03 ** 2))))
                # knee-high clearance kernel (mirrors sim/mjx): the fraction
                # of the COMMANDED swing height (lift_height + c6) the swing
                # sole actually reaches, on the correct one-foot contact
                # pattern. Linear (not Gaussian) so a 10 cm target still
                # pays gradient from a 2 cm lift -- the kernel-width lesson.
                # c6 = 0 under a plain balance lift -> "reach lift_height";
                # the knee-high march drives c6 to march_dz.
                tgt_clear = self.lift_height + float(self._cmd[6])
                knee_frac = (float(lifted) * float(not con_swing)
                             * float(con_stance)
                             * float(np.clip(foot_clear
                                             / max(tgt_clear, 1e-6),
                                             0.0, 1.0)))
                # CoM-over-stance-foot kernel (mirrors sim/mjx): pay for
                # keeping the whole-robot CoM planted over the support sole
                # while a lift is commanded (knee-flexion lifts)
                com = d.subtree_com[self._torso_bid]
                stance_gid = self._sole_gids[0 if is_r else 1]
                stance_w = d.geom_xpos[stance_gid]
                com_off = float(np.sqrt((com[0] - stance_w[0]) ** 2
                                        + (com[1] - stance_w[1]) ** 2
                                        + 1e-12))
                com_kernel = float(np.exp(-((com_off / self.com_sigma) ** 2)))
                # skill-compliance gate (round 4, mirrors sim/mjx): under a
                # skill command the velocity/yaw kernels pay in proportion
                # to the skill being DONE (precision_v3 stood through them)
                crouching = (not lifted) and float(self._cmd[3]) < 0.97
                g_skill = (0.2 + 0.8 * lift_ok) if lifted else 1.0
                if crouching:
                    g_skill = 0.2 + 0.8 * h_norm
                primary = g_skill * (v_term + w_term) + h_term
                if self.recover_mix > 0 and not self._recovered:
                    # down in a recovery episode: height RATCHET is the
                    # primary until the first stand (mirrors sim/mjx --
                    # only NEW height above the episode best pays; static
                    # height income was a do-nothing optimum, getup_v1)
                    planar_g = float(np.hypot(d.qvel[0], d.qvel[1]))
                    standing_r = float((height > 0.85 * self._nominal_h)
                                       and (up_z > 0.9))
                    still = float(np.exp(-((planar_g / 0.2) ** 2)))
                    h_gain = max(height - self._best_h, 0.0)
                    # getup_v8: RELATIVE height income while down (0.6 * h/N,
                    # max 0.5/step < the 0.7 bleed, so do-nothing stays
                    # net-negative) -- v7 descended from the high-kneel to
                    # park at h 0.14 because the one-way ratchet made all
                    # parking spots pay alike; holding HIGH must beat low
                    primary = (200.0 * h_gain
                               + 0.6 * min(height / self._nominal_h, 1.0)
                               + 1.0 * standing_r * (0.5 + 0.5 * still)
                               - 0.7)
            elif self.cmd_dense and self._cmd[0] > 0.05:
                # any forward progress pays immediately, capped at the command
                v_term = self.w_track_v * min(vx_body, self._cmd[0]) / self._cmd[0]
                primary = v_term + self.w_track_w * float(
                    np.exp(-((wz_rate - self._cmd[1]) / 0.5) ** 2))
            else:
                v_term = self.w_track_v * float(np.exp(
                    -((vx_body - self._cmd[0]) / 0.5) ** 2))
                primary = v_term + self.w_track_w * float(
                    np.exp(-((wz_rate - self._cmd[1]) / 0.5) ** 2))
            reward_time_stop = 0.0
        elif braking:
            planar = float(np.hypot(d.qvel[0], d.qvel[1]))
            cap = 1.0 if self.target_speed is None else self.target_speed
            # sharp ramp: full credit only near standstill, zero above
            # 3x stand_speed -- "slow walking" earns (almost) nothing
            primary = self.w_forward * cap * float(
                np.clip(1.0 - planar / (3.0 * self.stand_speed), 0.0, 1.0))
            reward_time_stop = self.w_time_stop
        else:
            primary = self.w_forward * fwd_term
            reward_time_stop = 0.0
        # gait shaping is for locomotion: off while braking, and off under a
        # stand-still command (double support is rewarded instead, below)
        lifted = self.ext_cmd and abs(float(self._cmd[4])) > 0.5
        cmd_moving = (not self.command_mode) or abs(self._cmd[0]) > 0.05 \
            or abs(self._cmd[1]) > 0.05
        if self.ext_cmd:
            # a lifted command is a balance task, not locomotion
            cmd_moving = (abs(self._cmd[0]) > 0.05 or abs(self._cmd[1]) > 0.05
                          or abs(self._cmd[2]) > 0.05) and not lifted
        shaping_on = cmd_moving and not braking
        # posture reference: the commanded crouch height in ext mode
        h_ref = (self._cmd[3] * self._nominal_h if self.ext_cmd
                 else self._nominal_h)

        reward = (
            primary                                       # go forward / brake
            + self.w_upright * up_z                       # stay upright
            + self.alive_bonus                            # alive bonus
            - self.w_height * abs(height - h_ref)         # hold posture
            - self.w_energy * energy                      # be efficient
            - self.w_action_rate * action_rate            # be smooth
            - self.w_power * power_w                      # battery-life cost
            - reward_time_stop                            # post-cross urgency
        )

        # -- gait-quality shaping (all weights default 0 -> no-op) -----------
        con_l, con_r = self._foot_contacts()
        touchdown_air = 0.0   # summed swing durations of feet that just landed
        for f, con in enumerate((con_l, con_r)):
            if con:
                if self._air_time[f] > 0.0:
                    touchdown_air += self._air_time[f]
                    # legged_gym-style: on touchdown, reward the swing duration,
                    # capped at air_time_target (no hopping incentive), minus a
                    # baseline of half the target so quick shuffle taps score
                    # *negative* -- the gradient pushes toward deliberate swings.
                    if shaping_on:
                        reward += self.w_feet_air * (
                            min(self._air_time[f], self.air_time_target)
                            - 0.5 * self.air_time_target)
                        # gait symmetry: mismatch vs the OTHER foot's last
                        # completed swing (mirrors sim/mjx)
                        if self.w_symmetry and self._last_air[1 - f] > 0.0:
                            reward -= self.w_symmetry * abs(
                                self._air_time[f] - self._last_air[1 - f])
                    self._last_air[f] = self._air_time[f]
                self._air_time[f] = 0.0
            else:
                self._air_time[f] += self.control_dt
        single_support = con_l != con_r
        double_support = con_l and con_r
        if self.ext_cmd:
            # lift + swing-foot rewards (geometry computed pre-reward in the
            # command block, mirrors sim/mjx)
            reward += self.w_lift * lift_ok
            reward += self.w_track_foot * foot_kernel
            # clearance-gated (mirrors sim/mjx -- skills_v2 leaning loophole)
            reward += (self.w_com_stance * float(lifted) * com_kernel
                       * float(np.clip(foot_clear / self.lift_clear,
                                       0.0, 1.0)))
            if self.w_foot_under:
                reward += self.w_foot_under * foot_under
            if self.w_knee_high:
                reward += self.w_knee_high * knee_frac
        if self.w_single_support:
            if shaping_on:
                reward += self.w_single_support * float(single_support)
            elif self.command_mode and not cmd_moving:
                # under a stand command, plant both feet -- unless a foot is
                # commanded lifted, in which case single support IS the task
                reward += self.w_single_support * float(
                    single_support if lifted else double_support)
        if self.w_lateral:
            if self.ext_cmd:
                # vy is a tracked command channel: penalize the ERROR
                reward -= self.w_lateral * abs(vy_body - float(self._cmd[1]))
            elif self.command_mode:
                # body-frame side-slip (world drift is meaningless when the
                # command says turn); vy_body computed in the tracking block
                reward -= self.w_lateral * abs(vy_body)
            else:   # walk straight: lateral speed + sideways drift
                reward -= self.w_lateral * (abs(float(d.qvel[1]))
                                            + 0.5 * abs(float(d.qpos[1])))
        if self.w_yaw and not self.command_mode:  # face +x (yaw-rate tracking
            qw, qx, qy, qz = d.qpos[3:7]          # replaces this in cmd mode)
            yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            reward -= self.w_yaw * (abs(float(yaw)) + 0.1 * abs(float(d.qvel[5])))
        if self.w_pitch_rate:   # calm torso: roll + pitch angular rates
            reward -= self.w_pitch_rate * (abs(float(d.qvel[3]))
                                           + abs(float(d.qvel[4])))
        if self.w_pitch_hinge:  # upright torso: |pitch| past a free deadband
            # ZYX-euler pitch off the root quaternion -- the SAME arithmetic
            # as sim/mjx/env_mjx._quat_pitch and as the gait probe (+ = nose
            # down toward +x). Roll is deliberately NOT penalized.
            qw, qx, qy, qz = d.qpos[3:7]
            pitch = float(np.arcsin(np.clip(2 * (qw * qy - qz * qx),
                                            -1.0, 1.0)))
            excess = max(0.0, abs(pitch) - self._pitch_db)
            # locomotion only: while DOWN in a recovery episode a face-plant
            # pitch is the task, not a fault (mirrors w_up_vel/w_rise_* which
            # are gated the other way on the same flag)
            if self._recovered and not self.getup:
                reward -= self.w_pitch_hinge * excess * excess

        # -- plan-v2 Phase A terms (mirror sim/mjx; defaults off) --------------
        if self.gait_clock:
            self._gait_phase = float(
                (self._gait_phase + 2 * np.pi * self.control_dt
                 * self._gait_freq + np.pi) % (2 * np.pi) - np.pi)
        if self.w_feet_phase:
            rz_l = self.swing_height * max(0.0, float(np.sin(self._gait_phase)))
            rz_r = self.swing_height * max(0.0, float(-np.sin(self._gait_phase)))
            z_l = float(d.geom_xpos[self._sole_gids[0]][2]) - self._sole_z0
            z_r = float(d.geom_xpos[self._sole_gids[1]][2]) - self._sole_z0
            if cmd_moving:
                reward += self.w_feet_phase * float(np.exp(
                    -((z_l - rz_l) ** 2 + (z_r - rz_r) ** 2)
                    / self.feet_phase_s2))
        if self.w_feet_slip:
            v_l = d.cvel[self._foot_bids[0], 3:5]
            v_r = d.cvel[self._foot_bids[1], 3:5]
            reward -= self.w_feet_slip * (
                float(con_l) * float(np.sqrt(np.sum(v_l ** 2) + 1e-12))
                + float(con_r) * float(np.sqrt(np.sum(v_r ** 2) + 1e-12)))
        if self.w_orientation:
            upv = d.sensordata[self._up_id:self._up_id + 3]
            reward -= self.w_orientation * float(upv[0] ** 2 + upv[1] ** 2)
        if self.w_ang_vel_xy:
            reward -= self.w_ang_vel_xy * float(d.qvel[3] ** 2
                                                + d.qvel[4] ** 2)
        if self.w_pose:
            if self.ext_cmd:
                pose_gate = float((not lifted)
                                  and float(self._cmd[3]) >= 0.97
                                  and self._recovered)
            else:
                pose_gate = 1.0
            reward -= (self.w_pose * pose_gate
                       * float(np.sum((d.qpos[self._jqpos] - self._default) ** 2)))
        if self.w_dof_limits:
            q = d.qpos[self._jqpos]
            out = (np.maximum(self._soft_lo - q, 0.0)
                   + np.maximum(q - self._soft_hi, 0.0))
            reward -= self.w_dof_limits * float(np.sum(out))
        if self.w_mimic:
            q_ref = self._mimic_ref(self._cmd, self._gait_phase,
                                    self._gait_freq)
            dq = np.asarray(d.qpos[self._jqpos]) - q_ref
            mim_gate = 1.0
            if self.ext_cmd:
                mim_gate = float((not lifted) and self._recovered)
            reward += (self.w_mimic * mim_gate
                       * float(np.exp(-np.sum(self._mimic_w * dq ** 2)
                                      / self.mimic_s2)))
        if self.w_rise_dofvel and self.ext_cmd and self.recover_mix > 0:
            if not self._recovered:
                reward -= self.w_rise_dofvel * float(np.sum(
                    np.asarray(d.qvel[self._jqvel]) ** 2))
        if self.w_rise_ref and self.ext_cmd and self.recover_mix > 0:
            if not self._recovered:
                # staged-rise reference (mirrors sim/mjx _rise_ref exactly)
                q_rr = self._rise_ref(self._rise_t0
                                      + self._step_i * self.control_dt)
                dq_rr = np.asarray(d.qpos[self._jqpos]) - q_rr
                reward += self.w_rise_ref * float(np.exp(
                    -np.sum(dq_rr ** 2) / self.rise_ref_s2))
        if self.w_up_vel and self.ext_cmd and self.recover_mix > 0:
            # getup_v5 momentum incentive (mirrors sim/mjx)
            if not self._recovered:
                reward += self.w_up_vel * float(np.clip(d.qvel[2], 0.0, 0.5))

        self._prev_action[:] = action
        self._step_i += 1
        # ratchet update AFTER grading (mirrors sim/mjx: reward uses the
        # pre-step best, the returned state carries the new best)
        self._best_h = max(self._best_h, height)

        # ext mode: the fall floor tracks the commanded crouch (a commanded
        # 0.6-height crouch is 0.17 m -- below the legacy 0.18 fall line)
        fall_h = (self.fall_height * float(self._cmd[3]) if self.ext_cmd
                  else self.fall_height)
        fell = (height < fall_h) or (up_z < self.fall_up_z)
        if self.ext_cmd and self.recover_mix > 0 and self._recover_ep:
            # getup_v9 (mirrors sim/mjx): recovery episodes NEVER terminate
            # on falls -- pre-recover the fall is the task, post-recover a
            # fall just loses the standing income and the rise is
            # re-practiced in the same episode. recovered requires a HELD
            # stand (stand_hold_n consecutive standing steps), not an
            # instantaneous crossing (ballistic bank starts farmed the flip
            # mid-flight). Pre-step flag gates the reward; update after.
            fell = False
            if (height > 0.85 * self._nominal_h) and (up_z > 0.9):
                self._stand_streak += 1.0
            else:
                self._stand_streak = 0.0
            if self._stand_streak >= self._stand_hold_n:
                self._recovered = True
        # -- 2 m dash: success only if, after crossing the line, the robot is
        # still upright dash_hold seconds later. A dive that crosses the line
        # and faceplants therefore never finishes -- it just falls.
        dash_success = False
        if self.dash:
            reward -= self.w_time
            t_now = self._step_i * self.control_dt
            if self._cross_t is None and x_after >= self.dash_distance:
                self._cross_t = t_now              # clock stops at the crossing
            if self.dash_stop:
                # confirmed finish = a sustained STANDSTILL past the line:
                # planar speed under stand_speed for dash_hold s, upright the
                # whole time (a jog-through or a stumbling stop doesn't count)
                standing = (not fell and self._cross_t is not None
                            and float(np.hypot(d.qvel[0], d.qvel[1]))
                            < self.stand_speed)
                if standing:
                    if self._stand_t is None:
                        self._stand_t = t_now      # standstill clock starts
                    if t_now - self._stand_t >= self.dash_hold:
                        dash_success = True        # confirmed standing finish
                        reward += self.finish_bonus
                else:
                    self._stand_t = None           # moved/fell -> restart clock
            elif (not fell and self._cross_t is not None
                    and t_now - self._cross_t >= self.dash_hold):
                dash_success = True                # confirmed upright finish
                reward += self.finish_bonus

        terminated = (fell or dash_success) and not self.getup
        truncated = self._step_i >= self.max_steps
        if fell and not self.getup:
            # dash_stop: falling PAST the line costs 3x -- otherwise "sprint
            # and dive at the line" is the least-bad explored policy (braking
            # attempts that fall cost the same as diving, and the standing
            # payoff is never sampled; observed in dash_11v1_real2, 0/16 all
            # by post-cross falls)
            reward -= self.fall_cost * (
                3.0 if (self.dash_stop and self._cross_t is not None) else 1.0)

        info = {"fwd_vel": fwd_vel, "height": height, "up_z": up_z, "x": x_after,
                "double_support": double_support, "single_support": single_support,
                "touchdown_air": touchdown_air, "dash_success": dash_success,
                "power_w": power_w,
                "standing": float((height > 0.85 * self._nominal_h)
                                  and (up_z > 0.9)),
                "time_to_2m": float("nan") if self._cross_t is None else self._cross_t}
        if self.command_mode:
            info.update(cmd_v=float(self._cmd[0]), cmd_w=float(self._cmd[1]),
                        vx_body=vx_body, wz=wz_rate,
                        y=float(d.qpos[1]))
            if self.ext_cmd:
                info.update(cmd_w=float(self._cmd[2]),
                            cmd_vy=float(self._cmd[1]),
                            cmd_h=float(self._cmd[3]),
                            cmd_lift=float(self._cmd[4]),
                            vy_body=vy_body,
                            height_err=abs(height - h_ref),
                            foot_err=float(lifted) * foot_err,
                            foot_clear=float(lifted) * foot_clear,
                            # achieved swing-foot offset from the hang pose
                            # (comparable to commanded c5/c6; referee traces
                            # the actual circle from these)
                            foot_dx=float(rel[0] - base[0]),
                            foot_dz=float(rel[2] - base[2]
                                          - self.lift_height),
                            lift_ok=lift_ok,
                            # knee-high march telemetry (CPU referee only):
                            # the commanded swing-height target and the
                            # fraction of it actually reached
                            cmd_dz=float(self._cmd[6]),
                            knee_tgt=float(self.lift_height
                                           + self._cmd[6]),
                            knee_frac=knee_frac,
                            recovered=float(self._recovered),
                            com_stance=float(lifted) * com_off,
                            head_err=abs(head_err))
            # heading reference: advance by the GRADED command's yaw rate,
            # wrap, restart at the current facing on resample (mirrors
            # sim/mjx: yaw rate lives in c2 ext / c1 legacy)
            self._head_ref += (float(self._cmd[2]) if self.ext_cmd
                               else float(self._cmd[1])) * self.control_dt
            self._head_ref = float(np.arctan2(np.sin(self._head_ref),
                                              np.cos(self._head_ref)))
            # resample AFTER the reward (which graded the command the policy
            # saw); the returned obs carries the new command
            if self._step_i >= self._cmd_next:
                self._head_ref = yaw
                self._sample_command()
            if self.ext_cmd and self._traj_on:
                # trajectory-mode evolution (mirrors sim/mjx): 1 = foot
                # circle, 2 = march (alternating lifts + height bob),
                # 3 = lateral sway, 4 = KNEE-HIGH march. Pinned commands
                # keep mode 0.
                th = (self._traj[2]
                      + self._traj[1] * self._step_i * self.control_dt)
                # knee-high march clock (mirrors sim/mjx). march_hz > 0 pins
                # a COMMANDED cadence: the swing timing is then the command's
                # own, deliberately decoupled from the gait clock (which
                # keeps running at its own rate in the obs -- the policy has
                # to learn the difference). march_hz == 0 falls back to the
                # gait clock, so the alternation stays anticipatable from the
                # sin/cos(phase) obs channels and firmware can regenerate it
                # from its own gait clock; with neither, the per-episode
                # trajectory clock.
                if self.march_hz > 0.0:
                    n_m = self._step_i % self._march_period
                    m_left = n_m < self._march_half
                    th_m = 2 * np.pi * n_m / self._march_period
                else:
                    th_m = self._gait_phase if self.gait_clock else th
                    m_left = np.sin(th_m) >= 0
                if self._traj_on == 1.0:
                    self._cmd[5] = self._traj[0] * np.cos(th)
                    self._cmd[6] = self._traj[0] * np.sin(th)
                elif self._traj_on == 2.0:
                    self._cmd[4] = 1.0 if np.sin(th) >= 0 else -1.0
                    self._cmd[5] = 0.0
                    self._cmd[6] = self._traj[0] * abs(np.sin(th))
                elif self._traj_on == 3.0:
                    self._cmd[1] = self.sway_vy * np.sin(th)
                elif self._traj_on == 4.0:
                    # knee-high march: vx/vy/wz stay 0 (drawn that way), the
                    # LIFT channel alternates on the march clock using the
                    # same left/right convention as w_feet_phase (left
                    # swings on the first half cycle -> lift = -1), and c6
                    # raises the swing-foot TARGET from lift_height up to
                    # knee height over the swing.
                    self._cmd[4] = -1.0 if m_left else 1.0
                    self._cmd[5] = 0.0
                    self._cmd[6] = self._march_dz * abs(np.sin(th_m))
        self._phase_cmd()
        obs_out = self._obs()
        if self.obs_hist_len > 1:
            self._obs_hist = ([obs_out[:self.obs_frame].copy()]
                              + self._obs_hist[:-1])
        return obs_out, float(reward), terminated, truncated, info

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is not None and self._terrain_dirty:
            # hfield data changed since the renderer uploaded it to the GPU --
            # recreate the renderer so the drawn terrain matches the physics.
            self._renderer.close()
            self._renderer = None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            self._terrain_dirty = False
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        # track the torso in BOTH planar axes -- sidestep/turn maneuvers walk
        # out of a forward-only frame (user feedback 2026-07-18)
        x, y = float(self.data.qpos[0]), float(self.data.qpos[1])
        ground = self._ground_z(x, y)
        cam.lookat[:] = [x, y, 0.14 + ground]
        cam.distance, cam.azimuth, cam.elevation = 0.9, 135, -12
        self._renderer.update_scene(self.data, cam)
        # Decorative floor marker: a flat, semi-transparent green square at the
        # stored (x, y), drawn just above the floor. Appended AFTER scene update
        # as a decor geom -> purely visual, never enters physics.
        if self.mark_xy is not None:
            scn = self._renderer.scene
            if scn.ngeom < scn.maxgeom:
                mx, my = float(self.mark_xy[0]), float(self.mark_xy[1])
                mz = self._ground_z(mx, my) + 0.002
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(
                    g, mujoco.mjtGeom.mjGEOM_BOX,
                    np.array([0.06, 0.06, 0.002]),      # 12x12 cm, 4 mm thick
                    np.array([mx, my, mz]),
                    np.eye(3).ravel(),
                    np.array([0.2, 0.8, 0.2, 0.5], dtype=np.float32))
                g.category = int(mujoco.mjtCatBit.mjCAT_DECOR)
                scn.ngeom += 1
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
