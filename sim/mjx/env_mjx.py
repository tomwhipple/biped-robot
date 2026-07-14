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

# qpos/qvel layout: freejoint (7 pos+quat / 6 vel) then the 8 hinge joints.
_JQ0, _JQ1 = 7, 15
_JV0, _JV1 = 6, 14

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
    prev_action: jax.Array    # (8,)
    last_target: jax.Array    # (8,) target in force before this control step
    step_i: jax.Array         # ()
    air_time: jax.Array       # (2,) per-foot swing clocks
    cmd: jax.Array            # (2,) commanded (vx m/s, yaw rate rad/s)
    cmd_next: jax.Array       # ()  step index of the next command resample
    servo: jax.Array          # (4,) kp, kd, stall, no-load speed (per-episode)
    lat_ms: jax.Array         # ()  per-episode sub-step latency
    lash: jax.Array           # ()  per-episode backlash (rad)
    imu_R: jax.Array          # (3,3) mounting misalignment
    imu_bias: jax.Array       # (3,) gyro bias
    metrics: dict[str, jax.Array]


def _prep_model(xml_path: str, payload: bool, servo_joint_damping: float,
                mesh_floor: bool = False) -> mujoco.MjModel:
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
        body = ('<body name="payload" pos="0 0 0.08">'
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
    m.dof_damping[_JV0:_JV1] = servo_joint_damping
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
        w_recover_up: float = 0.8,     # uprightness term
        stand_bonus: float = 1.0,      # per-step, when upright+tall+still
    ):
        self.mj_model = _prep_model(
            xml_path, payload_mass > 0 or payload_dr, servo_joint_damping,
            mesh_floor=getup)
        if payload_mass > 0:
            self.mj_model.body_mass[self.mj_model.body("payload").id] = payload_mass
        self.model = mjx.put_model(self.mj_model)
        m = self.mj_model

        self.sim_dt = float(m.opt.timestep)
        self.n_substeps = max(1, round((1.0 / control_hz) / self.sim_dt))
        self.control_dt = self.n_substeps * self.sim_dt
        self.max_steps = int(episode_seconds / self.control_dt)

        jnt = m.actuator_trnid[:, 0]
        self._lo = jp.asarray(m.jnt_range[jnt, 0])
        self._hi = jp.asarray(m.jnt_range[jnt, 1])
        self._default = jp.asarray(m.qpos0[_JQ0:_JQ1])
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
        self.imu_obs = imu_obs
        self.imu_noise = imu_noise
        self.getup = getup
        self.getup_settle = max(1, round(getup_settle_s / self.sim_dt))
        self.w_recover_h = w_recover_h
        self.w_recover_up = w_recover_up
        self.stand_bonus = stand_bonus

        self.action_size = 8
        self.obs_size = 8 + 8 + 3 + 3 + 3 + 8 + 1 + 2 + 2   # 38, command mode

    # -- pieces ---------------------------------------------------------------
    def _sample_cmd(self, rng: jax.Array, step_i: jax.Array):
        """(cmd, cmd_next): stand / pivot-in-place / walk mix, exactly the CPU
        distribution (draws are made unconditionally -- fixed RNG shape)."""
        if self.cmd_fixed is not None:
            return (jp.asarray(self.cmd_fixed, dtype=jp.float32),
                    jp.asarray(10 ** 9, dtype=jp.int32))
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
                step_i + (hold / self.control_dt).astype(jp.int32))

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
             rng: jax.Array) -> jax.Array:
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
        phase = 2 * jp.pi * (step_i.astype(jp.float32) / self.max_steps)
        return jp.concatenate([
            data.qpos[_JQ0:_JQ1],
            data.qvel[_JV0:_JV1],
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

    def _fallen_data(self, r_q):
        """Settled ragdoll fall: random orientation + joints, dropped from
        0.35 m, stepped torque-free for getup_settle -- a natural heap."""
        r_o, r_j = jax.random.split(r_q)
        u1, u2, u3 = jax.random.uniform(r_o, (3,))
        quat = jp.array([jp.sqrt(u1) * jp.cos(2 * jp.pi * u3),
                         jp.sqrt(1 - u1) * jp.sin(2 * jp.pi * u2),
                         jp.sqrt(1 - u1) * jp.cos(2 * jp.pi * u2),
                         jp.sqrt(u1) * jp.sin(2 * jp.pi * u3)])
        joints = jp.clip(
            self._default + jax.random.uniform(r_j, (8,), minval=-0.6,
                                               maxval=0.6)
            * self._scale, self._lo, self._hi)
        qpos = (self._qpos0.at[2].set(0.35).at[3:7].set(quat)
                .at[_JQ0:_JQ1].set(joints))
        data = mjx.make_data(self.model)
        data = data.replace(qpos=qpos,
                            ctrl=jp.zeros(self.mj_model.nu) + self._default)

        def fall(d, _):
            return mjx.step(self.model, d), None

        data, _ = jax.lax.scan(fall, data, None, length=self.getup_settle)
        return mjx.forward(self.model, data)

    # -- api -------------------------------------------------------------------
    def reset(self, rng: jax.Array) -> State:
        rng, r_q, r_v, r_ep, r_cmd, r_obs = jax.random.split(rng, 6)
        if self.getup:
            data = self._fallen_data(r_q)
        else:
            qpos = self._qpos0.at[_JQ0:_JQ1].add(
                jax.random.uniform(r_q, (8,), minval=-0.03, maxval=0.03))
            qvel = jax.random.uniform(r_v, (self.mj_model.nv,),
                                      minval=-0.02, maxval=0.02)
            data = mjx.make_data(self.model)
            data = data.replace(qpos=qpos, qvel=qvel,
                                ctrl=jp.zeros(self.mj_model.nu) + self._default)
            data = mjx.forward(self.model, data)
        servo, lat_ms, lash, imu_R, imu_bias = self._draw_episode(r_ep)
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next = self._sample_cmd(r_cmd, step_i)
        prev_action = jp.zeros(8, dtype=jp.float32)
        obs = self._obs(data, prev_action, cmd, step_i, imu_R, imu_bias, r_obs)
        metrics = {k: jp.zeros(()) for k in self._METRIC_KEYS}
        return State(data=data, obs=obs, reward=jp.zeros(()),
                     done=jp.zeros(()), rng=rng, prev_action=prev_action,
                     last_target=self._default.astype(jp.float32),
                     step_i=step_i, air_time=jp.zeros(2), cmd=cmd,
                     cmd_next=cmd_next, servo=servo, lat_ms=lat_ms, lash=lash,
                     imu_R=imu_R, imu_bias=imu_bias, metrics=metrics)

    def reseed(self, first: State, rng: jax.Array) -> State:
        """Fresh episode that REUSES the cached first physics state (brax-style
        cheap auto-reset: no make_data/forward) but re-draws every per-episode
        quantity -- command, servo gains, latency, backlash, IMU error -- so
        episode-level DR diversity survives auto-resetting."""
        rng, r_ep, r_cmd, r_obs = jax.random.split(rng, 4)
        servo, lat_ms, lash, imu_R, imu_bias = self._draw_episode(r_ep)
        step_i = jp.zeros((), dtype=jp.int32)
        cmd, cmd_next = self._sample_cmd(r_cmd, step_i)
        prev_action = jp.zeros(8, dtype=jp.float32)
        obs = self._obs(first.data, prev_action, cmd, step_i, imu_R, imu_bias,
                        r_obs)
        metrics = {k: jp.zeros(()) for k in self._METRIC_KEYS}
        return first._replace(
            obs=obs, reward=jp.zeros(()), done=jp.zeros(()), rng=rng,
            prev_action=prev_action,
            last_target=self._default.astype(jp.float32), step_i=step_i,
            air_time=jp.zeros(2), cmd=cmd, cmd_next=cmd_next, servo=servo,
            lat_ms=lat_ms, lash=lash, imu_R=imu_R, imu_bias=imu_bias,
            metrics=metrics)

    def step(self, state: State, action: jax.Array) -> State:
        m = self.model
        rng, r_push_u, r_push_a, r_jit, r_obs, r_cmd = jax.random.split(
            state.rng, 6)
        action = jp.clip(action, -1.0, 1.0)
        target = jp.clip(self._default + self._scale * action,
                         self._lo, self._hi)

        # random shove, held for the whole control step (matches CPU env)
        push_on = (jax.random.uniform(r_push_u) < self.push_prob) & (
            self.push_force > 0.0)
        ang = jax.random.uniform(r_push_a, maxval=2 * jp.pi)
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
            q = d.qpos[_JQ0:_JQ1]
            qd = d.qvel[_JV0:_JV1]
            cap = stall * jp.clip(1.0 - jp.abs(qd) / w0, 0.0, 1.0)
            err = cur - q
            err = jp.sign(err) * jp.maximum(jp.abs(err) - 0.5 * lash, 0.0)
            tau = jp.clip(kp * err - kd * qd, -cap, cap)
            d = d.replace(qfrc_applied=jp.zeros(self.mj_model.nv
                                                ).at[_JV0:_JV1].set(tau))
            d = mjx.step(m, d)
            return (d, tau), None

        (data, tau), _ = jax.lax.scan(
            substep, (data, jp.zeros(8)), jp.arange(self.n_substeps))

        up_z = data.sensordata[self._up_adr + 2]
        height = data.qpos[2]                     # flat ground: ground_z == 0
        qd_j = data.qvel[_JV0:_JV1]
        energy = jp.sum(jp.abs(tau) * jp.abs(qd_j))
        action_rate = jp.sum((action - state.prev_action) ** 2)
        power_w = jp.sum(jp.maximum(tau * qd_j, 0.0) + _K_CU * tau ** 2)

        # command tracking (exp kernels sigma=0.5; cmd_dense option)
        qw, qx, qy, qz = (data.qpos[3], data.qpos[4], data.qpos[5],
                          data.qpos[6])
        yaw = jp.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        cth, sth = jp.cos(yaw), jp.sin(yaw)
        vx_body = cth * data.qvel[0] + sth * data.qvel[1]
        vy_body = -sth * data.qvel[0] + cth * data.qvel[1]
        wz = data.qvel[5]
        cmd_v, cmd_w = state.cmd[0], state.cmd[1]
        planar = jp.sqrt(data.qvel[0] ** 2 + data.qvel[1] ** 2)
        standing = ((height > 0.85 * self._nominal_h)
                    & (up_z > 0.9)).astype(jp.float32)
        if self.getup:
            # recovery: dense progress toward tall + upright, then a standing
            # bonus (half for being up, half for being STILL up there) that
            # makes a held stand the absorbing goal instead of thrashing
            still = jp.exp(-((planar / 0.2) ** 2))
            primary = (self.w_recover_h * jp.clip(height / self._nominal_h,
                                                  0.0, 1.0)
                       + self.w_recover_up * 0.5 * (up_z + 1.0)
                       + self.stand_bonus * standing * (0.5 + 0.5 * still))
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

        reward = (primary
                  + self.w_upright * up_z
                  + self.alive_bonus
                  - self.w_height * jp.abs(height - self._nominal_h)
                  - self.w_energy * energy
                  - self.w_action_rate * action_rate
                  - self.w_power * power_w)

        # gait shaping: on for moving commands, double-support bonus for stand
        cmd_moving = (jp.abs(cmd_v) > 0.05) | (jp.abs(cmd_w) > 0.05)
        con = self._foot_contacts(data)           # (2,) bool
        landed = con & (state.air_time > 0.0)
        if self.w_feet_air:
            reward += jp.where(cmd_moving, self.w_feet_air * jp.sum(
                jp.where(landed,
                         jp.minimum(state.air_time, self.air_time_target)
                         - 0.5 * self.air_time_target, 0.0)), 0.0)
        air_time = jp.where(con, 0.0, state.air_time + self.control_dt)
        single = con[0] != con[1]
        double = con[0] & con[1]
        if self.w_single_support:
            reward += jp.where(
                cmd_moving, self.w_single_support * single.astype(jp.float32),
                self.w_single_support * double.astype(jp.float32))
        if self.w_lateral:
            reward -= self.w_lateral * jp.abs(vy_body)
        if self.w_pitch_rate:
            reward -= self.w_pitch_rate * (jp.abs(data.qvel[3])
                                           + jp.abs(data.qvel[4]))

        step_i = state.step_i + 1
        fell = (height < self.fall_height) | (up_z < self.fall_up_z)
        if self.getup:
            done_flag = jp.zeros(())          # being down IS the task
        else:
            reward = reward - self.fall_cost * fell.astype(jp.float32)
            done_flag = fell.astype(jp.float32)

        # resample AFTER the reward graded the command the policy saw
        new_cmd, new_next = self._sample_cmd(r_cmd, step_i)
        resample = step_i >= state.cmd_next
        cmd = jp.where(resample, new_cmd, state.cmd)
        cmd_next = jp.where(resample, new_next, state.cmd_next)

        obs = self._obs(data, action, cmd, step_i, state.imu_R,
                        state.imu_bias, r_obs)
        metrics = {
            "power_w": power_w, "vx_body": vx_body, "wz": wz,
            "height": height, "up_z": up_z, "fell": fell.astype(jp.float32),
            "track_v_err": jp.abs(vx_body - cmd_v),
            "track_w_err": jp.abs(wz - cmd_w),
            "standing": standing,
        }
        return State(data=data, obs=obs, reward=reward,
                     done=done_flag, rng=rng,
                     prev_action=action.astype(jp.float32),
                     last_target=target, step_i=step_i, air_time=air_time,
                     cmd=cmd, cmd_next=cmd_next, servo=state.servo,
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
