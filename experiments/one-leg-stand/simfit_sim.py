"""Replay a recorded one-leg stand on the TRAINING plant, tick for tick, with the firmware's
streamer and roll PI in the loop, and log what the sim predicts in the recording's own terms.

Plant: sim/bimo_biped_v6legs.xml in walker_env.BimoWalkerEnv (the CPU twin of env_mjx), set up
as train_mjx --robot --xml bimo_biped_v6legs.xml --servo-kp-scale asbuilt --quantize
--joint-frictionloss 0.35 sets it up, minus the domain randomisation (every value fixed, below).
The fitted parameters (simfit_fit.py) change it through `Plant`:

    kp_scale      x the asbuilt servo stiffness map (2.8x STS3215, 4x STS3250), all servos
    roll_lash     extra deadzone on the four roll joints (walker_env roll_lash_deg), deg p-p;
                  roll_lash_hip sets the two hip rolls apart (>= 0; walker_env has one value),
                  roll_lash_dL adds to the LEFT leg's two rolls only (asymmetry, off by default)
    sole_w        effective sole width: the 8 pads per foot scaled in y about the sole centre (m)
    floor_tc      floor/pad contact solref time constant (s): carpet compliance; 0.02 = MuJoCo default
    floor_dmin    solimp dmin of the same contact (0.9 = default)
    mu            sliding friction, floor and pads
    torso_scale   torso mass/inertia x
    torso_com_y   torso CoM lateral offset (m, + = the robot's left)
    lag_hz        3-stage target lag pole (walker_env act_lag_hz), fit to the servos' ~100 ms
                  goal->pos delay in the recording
    friction      joint Coulomb friction N*m (dof_frictionloss)

The firmware side (cli.cpp poseTick / balTick, balance/roll_pi): every `pose ... s<ms>` starts a
quintic min-jerk from the servos' READ-BACK positions (the sim encoder) to the target ticks; a
`B<side>` move engages the PI with ref = the roll at that instant; each tick the stance ankle
roll gets + ka*u and the hip roll + kh*u in integer ticks on top of the nominal; |err| past the
abort limit fades u over 0.3 s and lands the pose on the move's start over 1.0 s; the next
non-B pose drops the correction (it starts from read-back). Ticks are 24 ms (the bench's are
~25 ms). The move list and times come from the recording (simfit_hw).

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_sim.py 222031 L
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field, asdict, replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
sys.path.insert(0, HERE)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

import robot_plant  # noqa: E402
import walker_env as W  # noqa: E402
import simfit_hw as H  # noqa: E402

TICK = 2 * math.pi / 4096
CTRL_DT = 0.024                      # 12 physics steps of 2 ms
SETTLE_S = 2.0                       # crouch settle before the stand's shift move


@dataclass
class Plant:
    kp_scale: float = 1.0
    roll_lash: float = 3.0
    roll_lash_dL: float = 0.0
    roll_lash_dLhip: float = 0.0     # extra play on the LEFT hip roll only (hw 2026-09-02: L hip roll ~3 deg)
    roll_lash_hip: float = -1.0      # the two HIP rolls' play when >= 0 (else roll_lash, as walker_env)
    backlash: float = 0.75
    sole_w: float = 0.084
    sole_out_dL: float = 0.0         # the LEFT foot's outer edge pulled in by this much (m): an L-only soft outer edge
    floor_tc: float = 0.02
    floor_dr: float = 1.0
    floor_dmin: float = 0.9
    mu: float = 1.0
    torso_scale: float = 1.0
    torso_com_y: float = 0.0
    lag_hz: float = 5.0
    latency_ms: float = 4.0
    friction: float = 0.35
    imu_mount_deg: float = 0.0       # roll offset of the IMU (only absolute roll sees it)
    zoff_La: float = 0.0             # calibration zero error, deg: the L ankle roll / hip roll sit this far
    zoff_Lh: float = 0.0             # from where the robot believes (sim joint frame, + like +u on L)

    def label(self):
        d = asdict(self)
        base = asdict(Plant())
        return " ".join(f"{k}={v:g}" for k, v in d.items() if v != base[k]) or "training plant"


@dataclass
class Ctl:
    """The bench balance (balance/roll_pi.h Config) plus the candidate extensions:
    i_max     |I| limit (rad*s), 0 = none
    cond_int  conditional integration: freeze I while |err| > cond_band... (0 = off)
    kaw       back-calculation gain (1/s) on (u_sat - u_unsat), 0 = off
    ff_deg    feedforward added to u as the B move progresses (min-jerk of the move), deg outward
    i_leak    integrator leak (1/s), 0 = none
    """
    kp: float = 0.3
    ki: float = 1.0
    kd: float = 0.03
    ka: float = 1.0
    kh: float = -1.0
    clamp_deg: float = 10.0
    abort_deg: float = 12.0
    i_max: float = 0.0
    kaw: float = 0.0
    cond_band_deg: float = 0.0
    ff_deg: float = 0.0
    i_leak: float = 0.0
    i_rate_gate: float = 0.0          # freeze I while err*rate < 0 and |rate| > this (rad/s): err already shrinking


def minjerk(x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


class RollPIfw:
    """balance/roll_pi.cpp RollBalance::update, plus the Ctl extensions (all off = the firmware)."""

    def __init__(self, c: Ctl):
        self.c = c
        self.clamp = math.radians(c.clamp_deg)
        self.abort_rad = math.radians(c.abort_deg)
        self.state = "off"
        self.ref = self.err = self.i = self.u = 0.0
        self.since = 0.0
        self.u_abort = 0.0

    def engage(self, ref):
        self.ref, self.err, self.i, self.u = ref, 0.0, 0.0, 0.0
        self.state = "active"
        self.since = 0.0

    def disengage(self):
        self.state = "off"
        self.err = self.i = self.u = 0.0

    def land_frac(self):
        return minjerk((self.since - 0.3) / 1.0) if self.state == "abort" else 0.0

    def update(self, roll, dt, rate, ff=0.0):
        c = self.c
        if self.state == "abort":
            self.since += dt
            self.u = self.u_abort * (1.0 - minjerk(self.since / 0.3))
            return
        if self.state != "active":
            return
        err = roll - self.ref
        if abs(err) > self.abort_rad:
            self.err = err
            self.state = "abort"
            self.u_abort = self.u
            self.since = 0.0
            return
        self.err = err
        unsat = c.kp * err + c.ki * self.i + c.kd * rate + ff
        u = max(-self.clamp, min(self.clamp, unsat))
        pushing = (u != unsat) and ((err > 0) == (u > 0))
        integrate = not pushing
        if c.cond_band_deg > 0 and abs(err) > math.radians(c.cond_band_deg):
            integrate = False
        if c.i_rate_gate > 0 and err * rate < 0 and abs(rate) > c.i_rate_gate:
            integrate = False
        if integrate:
            self.i += err * dt
        if c.kaw > 0 and u != unsat and c.ki > 0:
            self.i += c.kaw * (u - unsat) / c.ki * dt
        if c.i_leak > 0:
            self.i -= c.i_leak * self.i * dt
        if c.i_max > 0:
            self.i = max(-c.i_max, min(c.i_max, self.i))
        self.u = u


def make_env(pl: Plant, seed=0):
    kw = robot_plant.robot_env_kwargs("bimo_biped_v6legs.xml")
    kp = dict(W.SERVO_KP_PRESETS["asbuilt"])
    kw["servo_kp_scale"] = {k: v * pl.kp_scale for k, v in kp.items()}
    kw.update(actuator_model="sts3215", supply_voltage=11.1, control_hz=1.0 / CTRL_DT, action_map="full",
              domain_rand=False, quantize_ticks=True, latency_ms=pl.latency_ms, backlash_deg=pl.backlash,
              roll_lash_deg=pl.roll_lash, joint_frictionloss=pl.friction, joint_armature=0.028,
              act_lag_hz=pl.lag_hz, push_prob=0.0, push_force=0.0, fall_up_z=-2.0, fall_height=-1.0,
              episode_seconds=1e4, imu_obs=False)
    env = W.BimoWalkerEnv(**kw)
    m = env.model
    # per-joint roll play: walker_env applies roll_lash x _roll_mask; the mask carries the
    # hip/ankle split and the left/right difference as ratios to a reference width
    lash = {}
    for n in env._servo_names:
        if n[2:] in ("hip_roll", "ankle_roll"):
            w = pl.roll_lash_hip if (n[2:] == "hip_roll" and pl.roll_lash_hip >= 0) else pl.roll_lash
            dl = pl.roll_lash_dL if n.startswith("L_") else 0.0
            if n == "L_hip_roll":
                dl += pl.roll_lash_dLhip
            lash[n] = max(0.0, w + dl)
    ref = max(lash.values()) if lash and max(lash.values()) > 0 else 0.0
    env._roll_lash_rad = math.radians(ref)
    env._roll_mask = np.array([(lash.get(n, 0.0) / ref) if ref > 0 else 0.0 for n in env._servo_names])
    pads = [g for g in range(m.ngeom) if "_pad_" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
    floor = m.geom("floor").id
    for side in "LR":
        ps = [g for g in pads if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g).startswith(side)]
        ys = np.array([m.geom_pos[g][1] for g in ps])
        yc = 0.5 * (ys.max() + ys.min())
        half = 0.5 * (ys.max() - ys.min())
        outer = 1.0 if side == "L" else -1.0           # +y is outboard for the left foot
        for g in ps:
            m.geom_pos[g][1] = yc + (m.geom_pos[g][1] - yc) * (0.5 * pl.sole_w) / half
            if side == "L" and pl.sole_out_dL and (m.geom_pos[g][1] - yc) * outer > 0:
                m.geom_pos[g][1] -= outer * pl.sole_out_dL * (m.geom_pos[g][1] - yc) * outer / (0.5 * pl.sole_w)
    for g in pads + [floor]:
        m.geom_solref[g] = [pl.floor_tc, pl.floor_dr]
        m.geom_solimp[g][0] = pl.floor_dmin
        m.geom_friction[g][0] = pl.mu
    b = env._torso_bid
    m.body_mass[b] *= pl.torso_scale
    m.body_inertia[b] *= pl.torso_scale
    m.body_ipos[b][1] += pl.torso_com_y
    env.reset(seed=seed)
    return env


class Replay:
    def __init__(self, stand: H.Stand, pl: Plant, ctl: Ctl, seed=0, imu_noise_deg=0.0, gyro_noise=0.0,
                 t_end=None, moves=None):
        self.st, self.pl, self.ctl = stand, pl, ctl
        self.moves = moves if moves is not None else stand.moves
        self.rng = np.random.default_rng(seed)
        self.imu_noise = math.radians(imu_noise_deg)
        self.gyro_noise = gyro_noise
        self.env = make_env(pl, seed)
        e = self.env
        self.names = e._servo_names
        self.jidx = np.array([self.names.index(j) for j in H.BUS])         # bus k -> servo index
        self.cal = np.array([stand.cal[j] for j in H.BUS], dtype=float)    # (zero, dir)
        self.imu = e.model.site("imu").id
        self.hroll = 0.5 * e._roll_lash_rad * e._roll_mask                 # downstream play half-widths
        self.zoff = np.zeros(len(H.BUS))                                   # bus order, rad
        self.zoff[H.BUS.index("L_ankle_roll")] = math.radians(pl.zoff_La)
        self.zoff[H.BUS.index("L_hip_roll")] = math.radians(pl.zoff_Lh)
        self.stall = None
        last = self.moves[-1]
        self.t_end = t_end if t_end is not None else last.t0 + last.dur + 1.0

    # ---- unit conversions (bus order) ----
    def ticks_to_rad(self, ticks):
        z, d = self.cal[:, 0], self.cal[:, 1]
        return (np.asarray(ticks, float) - z) * d * TICK

    def rad_to_ticks(self, rad):
        z, d = self.cal[:, 0], self.cal[:, 1]
        return np.rint(z + d * np.asarray(rad) / TICK)

    def encoder_rad(self):
        """what the servos read back (bus order): the shaft, i.e. the link plus the downstream roll play."""
        e = self.env
        q = e.data.qpos[e._sqpos].copy()
        tgt = e.data.ctrl.copy()
        h = self.hroll
        enc = q + np.clip(tgt - q, -h, h)
        return enc[self.jidx] - self.zoff

    def roll_rate(self):
        e = self.env
        R = e.data.site_xmat[self.imu].reshape(3, 3)
        roll = math.atan2(-R[1, 2], R[2, 2]) + math.radians(self.pl.imu_mount_deg)
        rate = float(e.data.qvel[3])
        return roll, rate

    def _place(self, q_bus):
        """start standing on the floor in the pose q_bus (bus-order rad), level, at rest."""
        e = self.env
        m, d = e.model, e.data
        q = e._sdefault.copy() if hasattr(e, "_sdefault") else d.qpos[e._sqpos].copy()
        q[self.jidx] = q_bus + self.zoff
        d.qpos[:] = m.qpos0
        d.qpos[e._sqpos] = q
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qvel[:] = 0
        mujoco.mj_kinematics(m, d)
        pads = [g for g in range(m.ngeom) if "_pad_" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
        zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in pads)
        d.qpos[2] -= zmin - 0.0002
        mujoco.mj_forward(m, d)
        e._lag_y = np.stack([q.astype(np.float64)] * 3)
        e._act_hist = None
        e._shaft = None
        e._last_target = q.copy()
        return q

    def run(self):
        e, c = self.env, self.ctl
        st = self.st
        side = None
        pi = RollPIfw(c)
        # the stand starts at the crouch (the shift's start); the crouch ticks are the last move's
        q0_ticks = np.array(self.moves[-1].ticks if self.moves[-1].name == "crouch" else self.moves[0].ticks, float)
        qs = self._place(self.ticks_to_rad(q0_ticks))
        tgt_servo = qs.copy()
        e._action_to_ctrl = lambda a: tgt_servo
        e._to_servo = lambda x: x
        traj = dict(start=q0_ticks.copy(), tgt=q0_ticks.copy(), t0=-SETTLE_S, dur=1.0)
        home = q0_ticks.copy()
        abort_from = None
        mi = 0
        log = {k: [] for k in ("t", "roll", "rate", "state", "ref", "err", "I", "u", "ff", "goal", "enc", "load",
                               "comy", "solec", "fz_st", "fz_sw")}
        t = -SETTLE_S
        nT = int(round((self.t_end + SETTLE_S) / CTRL_DT))
        ba = bh = None
        dirs = self.cal[:, 1]
        pads = {s: [g for g in range(e.model.ngeom) if (mujoco.mj_id2name(e.model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{s}_pad_")]
                for s in "LR"}
        f6 = np.zeros(6)
        stall = None
        for k in range(nT):
            roll, rate = self.roll_rate()
            if self.imu_noise:
                roll += float(self.rng.normal(0, self.imu_noise))
            if self.gyro_noise:
                rate += float(self.rng.normal(0, self.gyro_noise))
            # a new pose command?
            while mi < len(self.moves) and t >= self.moves[mi].t0 - 1e-9:
                mv = self.moves[mi]
                mi += 1
                if pi.state == "abort" and pi.land_frac() < 1.0:
                    continue                             # refused while landing (cli.cpp balLanding)
                enc_ticks = self.rad_to_ticks(self.encoder_rad())
                pi.disengage()
                side = None
                traj = dict(start=enc_ticks, tgt=np.array(mv.ticks, float), t0=mv.t0, dur=mv.dur)
                if mv.B:
                    side = mv.B
                    ba, bh = H.BUS.index(f"{side}_ankle_roll"), H.BUS.index(f"{side}_hip_roll")
                    pi.engage(roll)
                    home = enc_ticks.copy()
                    abort_from = None
                    self.ff_move = (mv.t0, mv.dur)
            # poseTick: nominal
            el = t - traj["t0"]
            sc = minjerk(el / traj["dur"])
            nom = traj["tgt"] if el >= traj["dur"] else traj["start"] + np.rint(sc * (traj["tgt"] - traj["start"]))
            want = nom.copy()
            ff = 0.0
            if side is not None:
                if c.ff_deg:
                    t0f, df = self.ff_move
                    ff = math.radians(c.ff_deg) * minjerk((t - t0f) / df) * (1.0 if side == "L" else -1.0)
                pi.update(roll, CTRL_DT, rate, ff)
                if pi.state == "abort":
                    if abort_from is None:
                        abort_from = nom.copy()
                    lf = pi.land_frac()
                    want = abort_from + np.rint(lf * (home - abort_from)) if lf > 0 else abort_from.copy()
                want[ba] += np.rint(dirs[ba] * c.ka * pi.u / TICK)
                want[bh] += np.rint(dirs[bh] * c.kh * pi.u / TICK)
            q_bus = self.ticks_to_rad(want)
            tgt_servo[self.jidx] = q_bus + self.zoff
            e.step(np.zeros(e._nq_act, np.float32))
            t += CTRL_DT
            if t < -0.05:
                continue
            if stall is None:
                kp_, kd_, s_, w0_ = e._servo
                stall = (np.asarray(s_, float) * e._stall_prof)[self.jidx]
            fz = {}
            for s in "LR":
                fz[s] = 0.0
            for i in range(e.data.ncon):
                cc = e.data.contact[i]
                for s in "LR":
                    if cc.geom1 in pads[s] or cc.geom2 in pads[s]:
                        mujoco.mj_contactForce(e.model, e.data, i, f6)
                        fz[s] += abs(f6[0])
            stn = side or ("L" if st.side == "L" else "R")
            sw = "R" if stn == "L" else "L"
            pc = np.mean([e.data.geom_xpos[g] for g in pads[stn]], axis=0)
            log["t"].append(t); log["roll"].append(roll); log["rate"].append(rate)
            log["state"].append({"off": 0, "active": 1, "abort": 2}[pi.state])
            log["ref"].append(pi.ref); log["err"].append(pi.err if pi.state != "off" else 0.0)
            log["I"].append(pi.i); log["u"].append(pi.u if pi.state != "off" else 0.0); log["ff"].append(ff)
            log["goal"].append(want.copy()); log["enc"].append(self.rad_to_ticks(self.encoder_rad()))
            log["load"].append(1000.0 * np.asarray(e._servo_tau, float)[self.jidx] / stall)
            log["comy"].append(float(e.data.subtree_com[0][1] - pc[1]) * (1.0 if stn == "L" else -1.0))
            log["solec"].append(float(pc[1]))
            log["fz_st"].append(fz[stn]); log["fz_sw"].append(fz[sw])
        out = {k: np.asarray(v) for k, v in log.items()}
        return out


def run_stand(tag, side, pl=None, ctl=None, **kw):
    st = H.load_stand(tag, side)
    return st, Replay(st, pl or Plant(), ctl or Ctl(), **kw).run()


if __name__ == "__main__":
    import time
    tag, side = (sys.argv[1], sys.argv[2]) if len(sys.argv) > 2 else ("222031", "L")
    t0 = time.time()
    st, r = run_stand(tag, side)
    print(f"{tag} {side}: {len(r['t'])} ticks in {time.time() - t0:.1f} s")
    R = 180 / math.pi
    sg = 1 if side == "L" else -1
    for name in ("l10", "l30"):
        mv = st.stage(name)
        w = (r["t"] >= mv.t0) & (r["t"] < mv.t0 + mv.dur + 8)
        tt = r["t"][w] - mv.t0
        for tq in (1, 2, 2.4, 3, 4, 5, 6, 8, 10):
            if tq < tt[-1]:
                k = np.searchsorted(tt, tq)
                print(f"  {name} t {tq:4.1f}  err {sg * r['err'][w][k] * R:+5.2f}  u {sg * r['u'][w][k] * R:+5.2f}  "
                      f"state {r['state'][w][k]}  comy {1e3 * r['comy'][w][k]:+5.1f} mm  Fsw {r['fz_sw'][w][k]:.1f}")
