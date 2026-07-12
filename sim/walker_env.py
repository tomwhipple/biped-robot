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

# qpos/qvel layout: freejoint (7 pos+quat / 6 vel) then the 8 hinge joints.
_JQPOS = slice(7, 15)   # 8 actuated-joint positions
_JQVEL = slice(6, 14)   # 8 actuated-joint velocities

# -- STS3215 servo datasheet numbers (Waveshare ST3215 wiki, checked 2026-07-11:
# https://www.waveshare.com/wiki/ST3215_Servo). "High torque, up to 30kg.cm@12V"
# and "No load Speed: 0.222sec/60deg (45RPM) @12V"; input voltage 6-12.6 V, so a
# 2S (7.4 V) or 3S (11.1 V) LiPo is in-spec. Stall torque and no-load speed are
# scaled linearly with supply voltage (standard DC-motor approximation).
_STS_STALL_12V = 2.94                       # N*m  (30 kg*cm)
_STS_NOLOAD_12V = np.deg2rad(60.0) / 0.222  # 4.712 rad/s

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
        w_track_v: float = 2.0,        # velocity-tracking reward (exp kernel)
        w_track_w: float = 1.0,        # yaw-rate-tracking reward (exp kernel)
        # -- hardware-realizable observations (sensing audit 2026-07-11) --------
        imu_obs: bool = False,         # torso linear velocity + height are ZEROED
        # (the real robot has no sensor for them; obs stays 36-wide so warm
        # starts still work), and up-vector/gyro become IMU-like: per-episode
        # mounting misalignment + gyro bias, per-step noise (BNO085-class
        # magnitudes) -- applied only when domain_rand is on.
        imu_noise: float = 1.0,        # scale on IMU misalignment/bias/noise DR
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
        self.push_prob = (0.01 if domain_rand else 0.0) if push_prob is None else push_prob
        self.push_force = (5.0 if domain_rand else 0.0) if push_force is None else push_force
        self.payload_mass = payload_mass
        self.payload_max = payload_max
        self.command_mode = command_mode
        self.cmd_v_range = cmd_v_range
        self.cmd_w_range = cmd_w_range
        self.cmd_stand_prob = cmd_stand_prob
        self.cmd_resample_s = cmd_resample_s
        self.w_track_v = w_track_v
        self.w_track_w = w_track_w
        self._cmd = np.zeros(2)        # (vx_body m/s, yaw rate rad/s)
        self._cmd_next = 0
        self.imu_obs = imu_obs
        self.imu_noise = imu_noise
        self._imu_R = np.eye(3)          # per-episode mounting misalignment
        self._imu_gyro_bias = np.zeros(3)
        xml_src = None
        if terrain_amplitude > 0:
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
                xml_src, payload_mass if payload_mass > 0 else _PAYLOAD_REF)
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
            self.model.dof_damping[_JQVEL] = servo_joint_damping
        elif actuator_model == "ideal":
            self._nom_servo = None
            self._servo = None
        else:
            raise ValueError(f"unknown actuator_model {actuator_model!r}")
        self.servo_range = servo_range
        self._servo_tau = np.zeros(8)

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

        # Terrain bookkeeping: cached (nrow, ncol) height grid in [0,1] for the
        # ground-height lookup, and a dirty flag so render() re-uploads the
        # hfield to the GPU after each re-randomization.
        self._hf_grid = None
        self._terrain_dirty = False

        self.sim_dt = self.model.opt.timestep                 # 0.002 s
        self.n_substeps = max(1, round((1.0 / control_hz) / self.sim_dt))
        self.control_dt = self.n_substeps * self.sim_dt        # ~0.02 s
        self.max_steps = int(episode_seconds / self.control_dt)

        # Per-joint actuator ranges (radians). Actions are a residual around the
        # nominal standing pose (qpos0, all-zeros here) so that action 0 == stand
        # -- the standard locomotion-RL convention. Without this, action 0 maps to
        # each joint's range midpoint (a deep knee crouch) and the robot topples.
        jnt = self.model.actuator_trnid[:, 0]
        self._lo = self.model.jnt_range[jnt, 0].copy()
        self._hi = self.model.jnt_range[jnt, 1].copy()
        self._default = self.model.qpos0[_JQPOS].copy()        # standing pose
        self._scale = 0.5 * (self._hi - self._lo)              # per-joint residual

        self._up_id = self.model.sensor("torso_up").adr[0]
        self._nominal_h = float(self.model.body("torso").pos[2])  # 0.28 m

        self.action_space = spaces.Box(-1.0, 1.0, shape=(8,), dtype=np.float32)
        obs_dim = 8 + 8 + 3 + 3 + 3 + 8 + 1 + 2 + (2 if command_mode else 0)
        self.observation_space = spaces.Box(
            -np.inf, np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.render_mode = render_mode
        self._renderer = None
        self._prev_action = np.zeros(8, dtype=np.float32)
        self._air_time = np.zeros(2)   # per-foot time since last ground contact
        self._step_i = 0

    # -- helpers -----------------------------------------------------------
    def _action_to_ctrl(self, action: np.ndarray) -> np.ndarray:
        action = np.clip(action, -1.0, 1.0)
        return np.clip(self._default + self._scale * action, self._lo, self._hi)

    # -- terrain -------------------------------------------------------------
    @staticmethod
    def _terrain_xml(xml_path: str, amplitude: float) -> str:
        """Return the MJCF with the flat floor plane replaced by an hfield geom
        (same name 'floor' so friction DR keeps working) plus its asset."""
        with open(xml_path) as f:
            xml = f.read()
        asset = (f'<asset><hfield name="terrain" nrow="{_HF_NROW}" '
                 f'ncol="{_HF_NCOL}" size="{_HF_RX} {_HF_RY} {amplitude} 0.1"/>'
                 f'</asset>\n  ')
        # keep the original floor's appearance (v2 has a checker material,
        # v1 a plain rgba) so terrain runs give the same motion cues
        orig = re.search(r'<geom name="floor"[^>]*?/>', xml, flags=re.S)
        look = re.search(r'(material="[^"]+"|rgba="[^"]+")', orig.group(0)) if orig else None
        geom = (f'<geom name="floor" type="hfield" hfield="terrain" '
                f'pos="{_HF_CX} 0 0" contype="1" conaffinity="3" '
                f'{look.group(1) if look else ""} friction="1 0.02 0.001"/>')
        patched, n = re.subn(r'<geom name="floor"[^>]*?/>', geom, xml, flags=re.S)
        if n != 1:
            raise ValueError(f"expected exactly one floor geom in {xml_path}, found {n}")
        return patched.replace("<worldbody>", asset + "<worldbody>", 1)

    @staticmethod
    def _payload_xml(xml: str, mass: float) -> str:
        """Insert the camera payload as its own (jointless, i.e. welded) child
        body of the torso, so its mass/inertia stay separately addressable at
        runtime for per-episode payload randomization."""
        body = ('<body name="payload" pos="0 0 0.08">'
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
        c = (x - (_HF_CX - _HF_RX)) / (2 * _HF_RX) * (_HF_NCOL - 1)
        r = (y + _HF_RY) / (2 * _HF_RY) * (_HF_NROW - 1)
        if not (0.0 <= c <= _HF_NCOL - 1 and 0.0 <= r <= _HF_NROW - 1):
            return 0.0
        c0, r0 = int(c), int(r)
        c1, r1 = min(c0 + 1, _HF_NCOL - 1), min(r0 + 1, _HF_NROW - 1)
        fc, fr = c - c0, r - r0
        g = self._hf_grid
        h = ((1 - fr) * ((1 - fc) * g[r0, c0] + fc * g[r0, c1])
             + fr * ((1 - fc) * g[r1, c0] + fc * g[r1, c1]))
        return float(h) * self.terrain_amplitude

    def _foot_contacts(self) -> tuple[bool, bool]:
        """(left, right) sole-in-contact flags. The soles are the only geoms
        with collisions enabled besides the floor, so any contact involving a
        sole geom is ground contact."""
        n = self.data.ncon
        if n == 0:
            return False, False
        g1 = self.data.contact.geom1[:n]
        g2 = self.data.contact.geom2[:n]
        lid, rid = self._sole_gids
        return (bool(np.any((g1 == lid) | (g2 == lid))),
                bool(np.any((g1 == rid) | (g2 == rid))))

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
        phase = 2 * np.pi * (self._step_i / self.max_steps)
        parts = [
            d.qpos[_JQPOS],                    # 8 joint angles
            d.qvel[_JQVEL],                    # 8 joint velocities
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
        return np.concatenate(parts).astype(np.float32)

    def _sample_command(self):
        """New (vx, yaw-rate) command. Mix: stand / pivot-in-place / walk
        (straight or turning). Pivot commands matter: with no hip-yaw joint,
        pivoting is this morphology's easiest turn -- leaving it out of the
        training distribution left cmd_11v1/b unable to track yaw at all."""
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

    def set_command(self, v: float, w: float):
        """External command override (scenario evals / future teleop or
        goal-seeking layers). Disables auto-resampling until the next reset."""
        self._cmd = np.array([float(v), float(w)])
        self._cmd_next = 10 ** 9

    # -- gym API -----------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
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
        if self.terrain_amplitude > 0:
            self._generate_terrain()   # before mj_resetData; CPU collisions read it live
        mujoco.mj_resetData(self.model, self.data)
        # small noise so the policy can't memorize one trajectory
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[_JQPOS] += self.np_random.uniform(-0.03, 0.03, size=8)
        self.data.qvel[:] = self.np_random.uniform(-0.02, 0.02, size=self.model.nv)
        self.data.ctrl[:] = self._default
        mujoco.mj_forward(self.model, self.data)
        self._prev_action[:] = 0.0
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
        self._last_target = self._default.copy()
        self._air_time[:] = 0.0
        self._servo_tau[:] = 0.0
        self._cross_t = None    # dash: time the torso first crossed dash_distance
        self._stand_t = None    # dash_stop: start of the current standstill
        if self.command_mode:
            self._sample_command()
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
        return self._obs(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        target = self._action_to_ctrl(action)
        if self.action_latency:                       # apply a delayed target
            self._ctrl_buf.append(target)
            target = self._ctrl_buf.pop(0)
        self.data.ctrl[:] = target

        # Random shove: apply a horizontal force to the torso for this control
        # step (models an unexpected push -- key for a robust real-world gait).
        self.data.xfrc_applied[self._torso_bid, :] = 0.0
        if self.push_force and self.np_random.uniform() < self.push_prob:
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
                q = self.data.qpos[_JQPOS]
                qd = self.data.qvel[_JQVEL]
                cap = stall * np.clip(1.0 - np.abs(qd) / w0, 0.0, 1.0)
                self._servo_tau = np.clip(kp * (cur - q) - kd * qd, -cap, cap)
                self.data.qfrc_applied[_JQVEL] = self._servo_tau
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
        energy = float(np.sum(np.abs(force) * np.abs(d.qvel[_JQVEL])))
        action_rate = float(np.sum((action - self._prev_action) ** 2))

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
            primary = (
                self.w_track_v * float(np.exp(-((vx_body - self._cmd[0])
                                                / 0.25) ** 2))
                # sigma 0.4: at 0.5 the left-yaw gait bias of the identical-
                # parts legs was cheaper to keep than to track out (cmd_11v1)
                + self.w_track_w * float(np.exp(-((wz_rate - self._cmd[1])
                                                  / 0.4) ** 2)))
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
        cmd_moving = (not self.command_mode) or abs(self._cmd[0]) > 0.05 \
            or abs(self._cmd[1]) > 0.05
        shaping_on = cmd_moving and not braking

        reward = (
            primary                                       # go forward / brake
            + self.w_upright * up_z                       # stay upright
            + self.alive_bonus                            # alive bonus
            - self.w_height * abs(height - self._nominal_h)  # hold posture
            - self.w_energy * energy                      # be efficient
            - self.w_action_rate * action_rate            # be smooth
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
                self._air_time[f] = 0.0
            else:
                self._air_time[f] += self.control_dt
        single_support = con_l != con_r
        double_support = con_l and con_r
        if self.w_single_support:
            if shaping_on:
                reward += self.w_single_support * float(single_support)
            elif self.command_mode and not cmd_moving:
                # under a stand command, plant both feet
                reward += self.w_single_support * float(double_support)
        if self.w_lateral:
            if self.command_mode:
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

        self._prev_action[:] = action
        self._step_i += 1

        fell = (height < 0.18) or (up_z < 0.4)
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

        terminated = fell or dash_success
        truncated = self._step_i >= self.max_steps
        if fell:
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
                "time_to_2m": float("nan") if self._cross_t is None else self._cross_t}
        if self.command_mode:
            info.update(cmd_v=float(self._cmd[0]), cmd_w=float(self._cmd[1]),
                        vx_body=vx_body, wz=wz_rate,
                        y=float(d.qpos[1]))
            # resample AFTER the reward (which graded the command the policy
            # saw); the returned obs carries the new command
            if self._step_i >= self._cmd_next:
                self._sample_command()
        return self._obs(), float(reward), terminated, truncated, info

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
        ground = self._ground_z(float(self.data.qpos[0]), 0.0)
        cam.lookat[:] = [float(self.data.qpos[0]), 0.0, 0.14 + ground]
        cam.distance, cam.azimuth, cam.elevation = 0.9, 135, -12
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
