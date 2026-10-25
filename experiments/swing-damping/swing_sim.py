"""Swing-leg reaction damping study: the one-leg lift-off sim, with extra physical knobs for
the calibration to the 2026-10-25 hardware log, and a swing-leg reaction law next to the
stance-leg PID of experiments/one-leg-stand/balance.py.

Everything here reuses the one-leg-stand plant, keyframes and servo model; nothing in
experiments/one-leg-stand is modified.  A case is a dict of plant knobs (CASE_DEFAULT), a
controller dict (ctl: stance PID gains, swing-law gains) and returns time series + metrics.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "experiments", "one-leg-stand"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")

import one_leg as OL  # noqa: E402
import static_gait as SG  # noqa: E402
from balance import Balance  # noqa: E402
from design_gates import JN, minjerk  # noqa: E402
from gen_plant_v6 import build_xml, params_from_args  # noqa: E402

_cache = {}
PJ = {"L_hip_roll": "sts3250", "R_hip_roll": "sts3250"}

# plant / actuation knobs.  shift 1.3 = the as-built shift the hardware needed (80.6 mm vs the model's 62).
CASE_DEFAULT = dict(
    stance="R", shift=1.3, t_lift=4.0, hold=8.5, torso=1.0, off=0.0, mu=0.7,
    play=0.0, lash=0.7, roll_lash=2.6,            # shaft/link play, global backlash, extra roll deadzone (all peak-to-peak, deg)
    lag=0.0, delay=1,                              # shaper pole (0 = bench streamer: none), dead time in 20 ms ticks
    kp_scale=1.0, servo_scale=1.0,                 # servo stiffness scale (all joints); torque-limit scale
    kp_hip=1.0, kp_ankle=1.0, kp_knee=1.0, kd_mult=1.0,   # extra kp(+kd) multipliers per joint family (hip rolls already 4x via PJ); servo D multiplier
    kick=0.0, kick_ms=40.0, kick_dir=(0.0, 0.0, 1.0),   # lift-off disturbance: impulse (N*s) on the swing foot when it leaves the carpet
    jfric=0.0, arm=0.0, jdamp=None,                            # joint frictionloss (N*m) and armature (kg*m^2)
    pad_solref=None, floor_solref=None,            # contact softness (timeconst, damping ratio)
    bias=-0.005, kf="asbuilt", imu_delay=1, imu_noise=0.1, gnoise=0.0075, gbias=0.0, seed=0)

CTL_STANCE = dict(kp=0.3, ki=1.0, kd=0.03, ka=1.0, kh=-1.0, clamp=10.0)
SWING0 = dict(sr=0.0, se=0.0, si=0.0, tau_w=1.0, sh=1.0, sa=0.0, lim=8.0, sb=0.0, sba=0.0, sp=0.0)   # swing law: see SwingHook


def _plant(sets=tuple(OL.AS_BUILT)):
    key = ("plant",) + tuple(sets)
    if key not in _cache:
        p, _ = params_from_args(["-o", "/dev/null", "--set", *sets])
        xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"swingdamp_{os.getpid()}_{len(_cache)}.xml")
        open(xml, "w").write(build_xml(p))
        _cache[key] = (p, xml)
    return _cache[key]


def _timeline(c):
    key = ("tl", c["stance"], c["shift"], c["t_lift"], c["hold"], c["bias"], c["kf"])
    if key not in _cache:
        p, _ = _plant(OL.AS_BUILT if c["kf"] == "asbuilt" else ())
        q, info = OL.keyframes(p, 0.02, 0.03, c["bias"], c["shift"], c["stance"])
        sw = "R" if c["stance"] == "L" else "L"
        tl = (OL.JointTimeline(q["straight"]).hold(0.5, "settle0").to(q["crouch"], 2.0, "crouch").hold(0.5, "h1")
              .to(q["shift"], 3.0, "shift").hold(1.0, "h2").to(q["lift"], c["t_lift"], "lift").hold(c["hold"], "hold"))
        win = [(tl.t_of("lift"), tl.T, sw)]
        _cache[key] = (tl, win, info)
    return _cache[key]


class SwingHook(Balance):
    """Stance PID (Balance) + swing-leg reaction law + per-tick logging.

    swing command  x = sr*rate + se*err + si*W,   W' = err - W/tau_w   (leaky integral of err)
    swing hip roll   += sh * clip(x)      swing ankle roll += sa * clip(x)       (rad; clip +-lim deg)
    `rate`, `err` are the same delayed/noisy gyro and IMU-roll signals the stance PID sees.  Signs follow the
    stance convention: + err (leaning to +roll) -> +x; the sign of sh/sa decides which way the leg moves
    and is part of the sweep.  The law is symmetric: it names the swing leg by the stance, hip/ankle roll
    joints mirror in the plant, so one gain set serves both sides.
    """

    def __init__(self, *a, sw=None, kick=0.0, kick_ms=40.0, kick_dir=(0, 0, 1.0), **kw):
        super().__init__(*a, **kw)
        self.kick, self.kick_ms = kick, kick_ms
        self.kick_dir = np.array(kick_dir, float) * np.array([1.0, 1.0 if self.s == 'R' else -1.0, 1.0])   # defined for right stance, mirrored
        self.t_kick = None
        self.sw = {**SWING0, **(sw or {})}
        self.W = 0.0
        self.log = []
        self.swing = "R" if self.s == "L" else "L"
        self.x_last = 0.0
        self.minh = None
        self.fg = None

    def __call__(self, t, env, qc):
        import mujoco
        m, d = env.model, env.data
        roll_true = self.roll_of(env)
        rate_true = self.rate_of(env)
        qc0 = qc
        qc = super().__call__(t, env, qc)
        sw = self.sw
        u_sw = 0.0
        if (sw["sb"] or sw["sba"]) and t >= self.t_on:
            # preload: take up the roll play of the swing leg by holding it against its outboard stop.
            # sb/sba in deg, + = outboard (away from the stance foot): mirrored by stance, one code path.
            ramp = min(1.0, (t - self.t_on) / 0.5)
            side = 1.0 if self.s == "L" else -1.0
            qc = qc.copy()
            qc[JN.index(f"{self.swing}_hip_roll")] += side * math.radians(sw["sb"]) * ramp
            qc[JN.index(f"{self.swing}_ankle_roll")] += side * math.radians(sw["sba"]) * ramp
        if (sw["sr"] or sw["se"] or sw["si"]) and self.t_on <= t < self.t_off + 1e9:
            err = (self.hist[max(0, len(self.hist) - 1 - self.delay)]) - self.ref
            rate = self.ghist[max(0, len(self.ghist) - 1 - self.gyro_delay)] + self.gyro_bias
            self.W += (err - self.W / sw["tau_w"]) * env.control_dt
            x = sw["sr"] * rate + sw["se"] * err + sw["si"] * self.W
            lim = math.radians(sw["lim"])
            x = max(-lim, min(lim, x))
            if t >= self.t_off:
                x *= 1.0 - minjerk(min(1.0, (t - self.t_off) / self.t_fade))
            u_sw = x
            qc = qc.copy()
            qc[JN.index(f"{self.swing}_hip_roll")] += sw["sh"] * x
            qc[JN.index(f"{self.swing}_ankle_roll")] += sw["sa"] * x
            if sw.get("sp"):
                qc[JN.index(f"{self.swing}_hip_pitch")] += sw["sp"] * x
        # logging: true signals + geometry
        if self.minh is None:
            self.sp = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{self.swing}_pad_")]
            self.tp = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{self.s}_pad_")]
            self.minh = 1
        clr = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.sp)
        # gap between the two foot collision boxes
        if self.fg is None:
            self.fg = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_col_foot") for s in (self.swing, self.s)]
        gap = float(mujoco.mj_geomDistance(m, d, self.fg[0], self.fg[1], 0.2, None))   # foot box to foot box, < 0 = interpenetrating
        selfc = 0
        fl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        for i in range(d.ncon):
            g1, g2 = d.contact[i].geom1, d.contact[i].geom2
            if g1 != fl and g2 != fl:
                selfc += 1
        if self.kick:
            fb = m.body(f"{self.swing}_foot").id
            d.xfrc_applied[fb, :3] = 0.0
            if self.t_kick is None and clr > 0.0005 and t > self.t_on:
                self.t_kick = t
            if self.t_kick is not None and t - self.t_kick < self.kick_ms * 1e-3 - 1e-9:
                d.xfrc_applied[fb, :3] = self.kick / (self.kick_ms * 1e-3) * self.kick_dir
        ref = self.ref if self.ref is not None else roll_true
        self.log.append((t, roll_true, rate_true, roll_true - ref, self.pi.u if t < self.t_off else 0.0, u_sw, clr, gap, selfc))
        return qc


def run(cfg=None, ctl=None, sw=None, keep_log=True):
    c = {**CASE_DEFAULT, **(cfg or {})}
    ctl = {**CTL_STANCE, **(ctl or {})}
    tl, win, info = _timeline(c)
    p, xml = _plant()
    seg = lambda n: tl.durs[tl.names.index(n) - 1]
    t_lift1 = tl.t_of("lift"); t_lift0 = t_lift1 - seg("lift"); t_hold1 = tl.T

    def setup(env):
        import mujoco
        b = env._torso_bid
        env.model.body_mass[b] *= c["torso"]; env.model.body_inertia[b] *= c["torso"]
        env.model.body_ipos[b][1] += c["off"]
        env._roll_lash_rad = math.radians(c["roll_lash"])
        names = list(env._servo_names)
        fam = np.array([c["kp_hip"] if n.endswith("hip_roll") else c["kp_ankle"] if n.endswith("ankle_roll")
                        else c["kp_knee"] if n.endswith("knee") else 1.0 for n in names]) * c["kp_scale"]
        env._kp_prof = env._kp_prof * fam
        env._kd_prof = env._kd_prof * fam * c["kd_mult"]
        if c["jfric"]:
            env.model.dof_frictionloss[env._sqvel] = c["jfric"]
        if c["jdamp"] is not None:
            env.model.dof_damping[env._sqvel] = c["jdamp"]
        if c["arm"]:
            env.model.dof_armature[env._sqvel] = c["arm"]
        m = env.model
        for g in range(m.ngeom):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if n.endswith("_pad_") or "_pad_" in n:
                if c["pad_solref"]:
                    m.geom_solref[g] = c["pad_solref"]
            if n == "floor" and c["floor_solref"]:
                m.geom_solref[g] = c["floor_solref"]

    hook = SwingHook(c["stance"], ctl["kp"], ctl["ki"], ctl["ka"], ctl["kh"], math.radians(ctl["clamp"]), kd=ctl["kd"],
                     t_on=t_lift0, t_off=t_hold1 + 1.0, t_fade=1.0, t_hold0=t_lift1, t_hold1=t_hold1,
                     delay_ticks=c["imu_delay"], noise=math.radians(c["imu_noise"]), seed=c["seed"],
                     gyro_noise=ctl.get("gnoise", c["gnoise"]), gyro_bias=c["gbias"], sw=sw,
                     kick=c["kick"], kick_ms=c["kick_ms"], kick_dir=c["kick_dir"])
    orig = SG.q_of
    SG.q_of = lambda p_, key: key if isinstance(key, np.ndarray) else orig(p_, key)
    try:
        r = SG.run_walk(p, xml, tl, win, seed=c["seed"], mu=c["mu"], play_deg=c["play"], backlash_deg=c["lash"],
                        lag_hz=c["lag"], delay_ticks=c["delay"], per_joint=PJ, servo_scale=c["servo_scale"],
                        setup=setup, hook=hook)
    finally:
        SG.q_of = orig
    L = np.array(hook.log)
    L[:, 0] -= t_lift0
    out = dict(cfg=c, ctl=ctl, sw=sw, fell=bool(r["fell"]), tripped=bool(hook.tripped), info=info)
    out.update(metrics(L, fell=out["fell"]))
    if keep_log:
        out["log"] = L
    return out


def metrics(L, fell=False, wins=((0, 4), (4, 8), (8, 12))):
    t, roll, rate, err, u, usw, clr, gap, selfc = L.T
    rdeg = np.degrees(rate)
    rms = []
    for a, b in wins:
        s = (t >= a) & (t < b)
        rms.append(float(np.sqrt(np.mean(rdeg[s] ** 2))) if s.sum() > 5 else float("nan"))
    sel = (t >= 1.0) & (t < 8.0) & (t < t[-1])
    fpk = float("nan")
    if sel.sum() > 50:
        x = rdeg[sel] - rdeg[sel].mean()
        x = x * np.hanning(len(x))
        f = np.fft.rfftfreq(len(x) * 8, 0.02)
        P = np.abs(np.fft.rfft(x, len(x) * 8)) ** 2
        k = (f >= 1.0) & (f <= 4.0)
        fpk = float(f[k][np.argmax(P[k])])
    s_hold = t >= 4.0
    return dict(rms=rms, err_peak=float(np.degrees(np.abs(err).max())), roll_pp=float(np.degrees(np.ptp(roll[t >= 1.0]))),
                f_hz=fpk, clr_min=float(1e3 * clr[(t >= 5.0)].min()) if s_hold.any() else float("nan"),
                clr_med=float(1e3 * np.median(clr[s_hold])) if s_hold.any() else float("nan"),
                gap_min=float(1e3 * gap.min()), contact_frac=float(np.mean(gap[t >= 2.0] < 0.0)), selfc=int(selfc.max()), usw_max=float(np.degrees(np.abs(usw).max())),
                u_max=float(np.degrees(np.abs(u).max())), t_end=float(t[-1]))


if __name__ == "__main__":
    import time
    t0 = time.time()
    r = run(dict(stance="R"))
    print({k: v for k, v in r.items() if k not in ("log", "cfg", "ctl", "info")}, time.time() - t0)
