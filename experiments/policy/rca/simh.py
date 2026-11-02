"""Sim harness for the first-stand RCA: the CPU plant (walker_env via eval_stand_arm's
plant builders) with the firmware's action chain in front of it, and pluggable servo /
floor / sensor models for the 'which ingredient flails' sweep."""
from __future__ import annotations
import json, math, os, sys
import numpy as np
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec
SIM = os.path.join(rec.ROOT, "sim"); sys.path.insert(0, os.path.join(SIM, "mjx")); sys.path.insert(0, SIM)
sys.path.insert(0, os.path.join(SIM, "sil"))
import eval_stand_arm as ESA   # noqa: E402
import eval_precision as _EP   # noqa: E402
_load_policy = _EP.load_policy
_POL = {}


def _cached_load_policy(run_dir, obs_size, act_size=8):
    k = (os.path.realpath(run_dir), obs_size, act_size)
    if k not in _POL:
        _POL[k] = _load_policy(run_dir, obs_size, act_size)
    return _POL[k]


_EP.load_policy = _cached_load_policy
ESA.RUNS = os.path.join(rec.MAIN, "sim", "runs")
RUN_NAME = "robot_legs_walk14_s1_s128r24"
STAND = ESA.STAND
NJ = 12


_ACT = {}


def build(plant, run=RUN_NAME, ep_s=16.0, model=None):
    """plant: train | fit | fit:<band>. A FRESH env every call (apply_model / set_play /
    set_series_encoder edit the env in place, so a cached one would leak one variant's
    plant into the next episode of the same worker); the jitted policy is reused."""
    for k in [k for k in ESA._CACHE if isinstance(k, tuple)]:
        _ACT.setdefault((k[0],), ESA._CACHE[k][1])
        del ESA._CACHE[k]
    ESA.RUNS = run_root(run)
    env, act, zoff = ESA._setup(run, plant, 0.0, ep_s)
    return env, act, zoff


def run_root(run):
    """This checkout's sim/runs (a worktree's own runs), else the main checkout's, else the
    night clone's (tonight's fresh runs)."""
    for root in (os.path.join(rec.ROOT, "sim", "runs"), os.path.join(rec.MAIN, "sim", "runs"),
                 "/home/claw/code/robot-mjx/sim/runs"):
        if os.path.exists(os.path.join(root, run, "params.pkl")):
            return root
    return os.path.join(rec.MAIN, "sim", "runs")


def apply_model(env, m):
    """Plant knobs on an already-built env (CPU MuJoCo model edits; reversible by rebuild).
    floor_tc: contact solref time constant for the floor + pads (hard floor ~0.005-0.01).
    floor_dr: contact damping ratio. latency_ms: sub-step servo latency."""
    import mujoco
    mm = env.model
    if m.get("floor_tc") is not None:
        name = lambda g: mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        pads = [g for g in range(mm.ngeom) if "_pad_" in name(g)] + list(env._sole_gids)
        for g in pads + [mm.geom("floor").id]:
            mm.geom_solref[g] = [m["floor_tc"], m.get("floor_dr", 1.0)]
    if m.get("friction") is not None:
        for g in range(mm.ngeom):
            mm.geom_friction[g][0] = m["friction"]
    if m.get("roll_lash_deg") is not None:      # total width at the hip/ankle rolls (hip = ref)
        env.roll_lash_deg_max = None
        env._roll_lash_rad = math.radians(m["roll_lash_deg"])
        env._roll_mask = np.array([1.0 if "roll" in n else 0.0 for n in env._servo_names])
    if m.get("backlash_deg") is not None:
        env.backlash_deg_max = None
        env._lash_rad = math.radians(m["backlash_deg"])
    if m.get("kd_scale") is not None or m.get("kp_scale") is not None:
        # per joint role or a float: {'roll': x} scales hip_roll + ankle_roll only
        def vec(v):
            if isinstance(v, dict):
                return np.array([v.get(n[2:], v.get("roll" if "roll" in n else "other", 1.0)) for n in env._servo_names])
            return np.full(len(env._servo_names), float(v))
        if not hasattr(env, "_kd_prof0"):
            env._kd_prof0 = np.array(env._kd_prof, float).copy(); env._kp_prof0 = np.array(env._kp_prof, float).copy()
        if m.get("kd_scale") is not None:
            env._kd_prof = env._kd_prof0 * vec(m["kd_scale"])
        if m.get("kp_scale") is not None:
            env._kp_prof = env._kp_prof0 * vec(m["kp_scale"])
    if m.get("frictionloss") is not None:
        mm.dof_frictionloss[env._sqvel] = float(m["frictionloss"])
    if m.get("armature") is not None:
        mm.dof_armature[env._sqvel] = float(m["armature"])
    if m.get("latency_ms") is not None:
        env._lat_ms_ep = float(m["latency_ms"])
        env.latency_ms = float(m["latency_ms"])
        env.latency_ms_max = None


class Servo:
    """What the servo does with the goal it is written, at the 50 Hz tick, before the
    plant's PD (which already models stiffness, damping, torque-speed cap, lash):
      delay   whole ticks of dead time
      lag_hz  first-order lag pole (0 = none)
      db_deg  hysteresis (play operator) on the servo's internal set point, total width
              (a goal change smaller than db/2 after a reversal is ignored) -- per joint
              group: {'roll': x, 'other': y} or a float
    """
    def __init__(self, delay=0, lag_hz=0.0, db_deg=0.0, roll_only_db=False, lag3_hz=0.0):
        self.delay, self.lag_hz, self.lag3_hz = int(delay), float(lag_hz), float(lag3_hz)
        db = np.full(NJ, math.radians(db_deg))
        if roll_only_db:
            db[[i for i, n in enumerate(rec.POL) if "roll" not in n]] = 0.0
        self.db = db

    def reset(self, q):
        self.buf = [np.array(q, float)] * self.delay
        self.y = np.array(q, float)
        self.y3 = np.stack([np.array(q, float)] * 3)
        self.p = np.array(q, float)

    def __call__(self, g):
        g = np.asarray(g, float)
        if self.delay:
            self.buf.append(g.copy()); g = self.buf.pop(0)
        if self.lag3_hz > 0:
            k = 1 - math.exp(-2 * math.pi * self.lag3_hz * 0.02)
            self.y3[0] += k * (g - self.y3[0]); self.y3[1] += k * (self.y3[0] - self.y3[1])
            self.y3[2] += k * (self.y3[1] - self.y3[2]); g = self.y3[2].copy()
        if self.lag_hz > 0:
            k = 1 - math.exp(-2 * math.pi * self.lag_hz * 0.02)
            self.y += k * (g - self.y); g = self.y.copy()
        if np.any(self.db > 0):
            self.p = np.clip(self.p, g - 0.5 * self.db, g + 0.5 * self.db); g = self.p.copy()
        return g


class Shaper:
    """obs::CommandShaper: 3 cascaded first-order lags at pole_hz, seeded at measured q."""
    def __init__(self, hz=10.0):
        self.k = 1 - math.exp(-2 * math.pi * hz * 0.02) if hz > 0 else 1.0

    def reset(self, q):
        self.y = np.stack([np.array(q, float)] * 3)

    def __call__(self, x):
        y = self.y; k = self.k
        y[0] += k * (x - y[0]); y[1] += k * (y[0] - y[1]); y[2] += k * (y[1] - y[2])
        return y[2].copy()


def place_and_hold(env, q_pol, settle_s=1.5):
    """eval_stand_arm._place_and_hold with the env's cascade off (lag 0)."""
    ESA._place_and_hold(env, np.asarray(q_pol, float), settle_s)


def sim_roll_pitch(d):
    w, x, y, z = d.qpos[3:7]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def set_play(env, lash_deg, encoder_side=True, dq_mode="fd"):
    """Per-joint free play (total width, deg; dict name->deg or role->deg) as the plant's
    deadzone on the servo error, REPLACING the plant's roll lash. encoder_side: the policy's
    q/dq read the servo shaft (upstream of the play) as on the robot -- the shaft sits at the
    target while the link floats inside the band, and at link -/+ half-play when it pushes."""
    names = env._servo_names
    deg = np.zeros(len(names))
    for i, n in enumerate(names):
        for k, v in lash_deg.items():
            if n == k or n[2:] == k:
                deg[i] = v
    env.roll_lash_deg_max = None
    env._roll_lash_rad = math.radians(1.0)
    env._roll_mask = deg.copy()
    env._lash_rad = 0.0
    env.backlash_deg_max = None
    if not encoder_side:
        return
    h = 0.5 * np.radians(deg) * 0 + 0.5 * np.radians(deg)
    orig = env._obs
    state = {"prev": None}

    def obs():
        o = orig()
        d = env.data
        ql = np.asarray(d.qpos[env._sqpos], float)
        tgt = env._last_target if env._last_target is not None else ql
        enc = ql - np.clip(ql - tgt, -h, h)
        zo = env._zero_off if env._zero_off is not None else 0.0
        qb = enc - zo
        prev = state["prev"] if state["prev"] is not None else qb
        o = o.copy()
        o[0:NJ] = qb
        if dq_mode == "fd":
            o[NJ:2 * NJ] = (qb - prev) / env.control_dt
        state["prev"] = qb
        return o
    env._obs = obs


FITTED_ACTUATOR = dict(kp_scale=0.47, kd_scale=0.03, frictionloss=0.09, roll_lash_deg=0.04,
                       backlash_deg=0.0, latency_ms=3.0, alpha=0.4)   # actuator_fit.py, 2026-11-01


def set_series_encoder(env, alpha):
    """Series compliance downstream of the encoder: the policy's q reads the servo shaft,
    q_enc = q_link + alpha * (target - q_link); dq likewise blended with the target rate."""
    if not alpha:
        return
    orig = env._obs
    st = {"pt": None}

    def obs():
        o = orig().copy()
        d = env.data
        ql = np.asarray(d.qpos[env._sqpos], float); dql = np.asarray(d.qvel[env._sqvel], float)
        tgt = np.asarray(env._last_target if env._last_target is not None else ql, float)
        pt = st["pt"] if st["pt"] is not None else tgt
        zo = env._zero_off if env._zero_off is not None else 0.0
        o[0:NJ] = ql + alpha * (tgt - ql) - zo
        o[NJ:2 * NJ] = (1 - alpha) * dql + alpha * (tgt - pt) / env.control_dt
        st["pt"] = tgt.copy()
        return o
    env._obs = obs


IS3250 = np.array(["hip_roll" in n for n in rec.POL])


class TypedServo:
    """Per-servo-type actuator response from the robot's `stepresp` (2026-11-01, loaded, double
    support): pure dead time then a first-order rise, applied to the written goal at the tick
    rate (dead time by linear interpolation of the goal history, so fractional ticks count).
      STS3215 (11 of 12 joints): dead ~70 ms, tau ~57 ms from motion start
      STS3250 (hip rolls):       dead ~15 ms, tau ~19 ms
    The plant's PD then adds its own (fast) mechanical response; tau_* here are the set-point
    lags left after it (calibrated with sim_stepresp.py). db_deg: set-point hysteresis."""
    def __init__(self, dead15=70.0, tau15=45.0, dead50=14.0, tau50=12.0, db15=0.0, db50=0.0, scale=1.0,
                 zeta50=None, fn50=9.0, dither_deg=0.0, dither_hz=4.6, seed=0):
        # zeta50: STS3250 set-point as an underdamped 2nd-order (natural freq fn50, damping
        # zeta50) instead of the first-order tau50 (unloaded stepresp: 10-22 % overshoot).
        # dither_deg: the STS3250 limit cycle (legs hanging: ~4.6 Hz, ~0.5 deg p2p, L/R in
        # antiphase in the joint frame) as a sinusoid on the hip-roll set points, amplitude deg.
        self.zeta50, self.w50 = zeta50, 2 * np.pi * fn50
        self.dither = math.radians(dither_deg); self.dither_w = 2 * np.pi * dither_hz
        self.ph = np.random.default_rng(seed).uniform(0, 2 * np.pi)
        self.sign = np.array([1.0 if n.startswith("L_") else -1.0 for n in rec.POL])
        self.dead = np.where(IS3250, dead50, dead15) * scale / 1000.0
        self.tau = np.where(IS3250, tau50, tau15) * scale / 1000.0
        self.db = np.radians(np.where(IS3250, db50, db15))
        self.n = int(np.ceil(self.dead.max() / 0.02)) + 2

    def reset(self, q):
        self.buf = [np.array(q, float)] * self.n
        self.y = np.array(q, float); self.p = np.array(q, float)
        self.z = np.array(q, float); self.zd = np.zeros(NJ); self.k = 0

    def __call__(self, g):
        self.buf.append(np.asarray(g, float).copy()); self.buf = self.buf[-self.n:]
        H = np.array(self.buf)                         # oldest .. newest (newest = now)
        k = self.dead / 0.02                           # ticks back, fractional
        i0 = np.floor(k).astype(int); fr = k - i0
        idx = len(H) - 1 - i0
        a = H[np.clip(idx, 0, len(H) - 1), np.arange(NJ)]
        b = H[np.clip(idx - 1, 0, len(H) - 1), np.arange(NJ)]
        x = (1 - fr) * a + fr * b
        kk = np.where(self.tau > 0, 1 - np.exp(-0.02 / np.maximum(self.tau, 1e-6)), 1.0)
        self.y += kk * (x - self.y)
        if self.zeta50 is not None:
            # 2nd-order set point for the STS3250s, integrated at 1 ms inside the tick
            w, zt = self.w50, self.zeta50
            for _ in range(20):
                acc = w * w * (x - self.z) - 2 * zt * w * self.zd
                self.zd += acc * 0.001; self.z += self.zd * 0.001
            self.y = np.where(IS3250, self.z, self.y)
        self.k += 1
        if self.dither:
            self.y = self.y + np.where(IS3250, self.sign * self.dither *
                                       math.sin(self.dither_w * self.k * 0.02 + self.ph), 0.0)
        if np.any(self.db > 0):
            self.p = np.clip(self.p, self.y - 0.5 * self.db, self.y + 0.5 * self.db)
            return self.p.copy()
        return self.y.copy()


# the robot's measured servo (2026-11-01 stepresp, unloaded = intrinsic): STS3215 dead ~68 ms then
# first-order ~55 ms; STS3250 dead ~12 ms, underdamped (10-22 % overshoot), 4.6 Hz limit cycle
MEAS_SERVO = dict(dead15=68.0, tau15=55.0, dead50=12.0, tau50=0.0, zeta50=0.55, fn50=9.0)
# joint_fit.py best (robot stepresp loaded + roll sweep, 2026-11-01)
JF_SERVO = dict(dead15=28.4, tau15=57.4, dead50=6.5, tau50=2.4)
JF_PLANT = dict(kp_scale=0.92, kd_scale=0.167, frictionloss=0.169, roll_lash_deg=0.0, backlash_deg=0.0)
