"""One-foot stance study for v5b (v5 body + v6 ankle roll), and what the ankle
roll buys over the pitch-only v5 ankle.

Two questions, both under a stated plant (sim numbers are gates, not
predictions of hardware):

  A  Kinematic (design_gates Gate A style). With the pelvis 20 mm below the
     straight-leg height and the swing foot flat 30 mm off the floor, the best
     CoM margin over the stance sole: the signed distance from the CoM's ground
     projection to the edge of the hull of the stance foot's real contact pads.
     Searched over pelvis x/y, pelvis roll (v5b) and the swing foot's x/y
     offset by random sampling + Nelder-Mead; each pose comes from a numerical
     leg IK on the MuJoCo model (both soles flat, hip yaw free to keep the feet
     straight). The same search on the unmodified v5body shows what the ankle
     roll buys: there the stance foot can only be flat if the whole torso rolls
     over it, so the pelvis roll is solved by the IK instead of searched.
  B  Dynamic, quasi-static (design_gates Gate B style, same servo envelope).
     stand -> crouch 20 mm -> shift onto the left foot -> lift the right foot
     20 mm straight up (the bench lift: 30 mm needs the swing ankle past the
     firmware's -40 deg) -> hold 5 s, min-jerk in task space, joint targets from
     the IK at 50 Hz, torque = PD (walker_env's sts3215: 12 N*m/rad, 0.25
     N*m*s/rad) clamped to the STS3215 torque-speed line at 11.1 V. No actuation
     lag, dead time, backlash or joint play: this is the ideal-servo gate.
     Swept over stiffness (stock, measured stock, Plan B x2.8 / x3.5 on the
     hip rolls, knees and ankle rolls) and over the shift duration.

    .venv/bin/python experiments/v5b/stance_v5b.py

  D  Ankle-roll physics: the bench pose is a PURE lateral shift (pelvis x, z and
     level roll held from the crouch). From it the CoM's ground projection is
     stepped across the stance sole (d = offset from the ankle-roll axis, +
     outboard) with the pelvis level and the right foot 20 mm up; each d is
     run alone and as one continuous sequence, with the static ankle-roll
     moment next to the simulated servo torque.

Regenerates experiments/v5b/stance_v5b.txt.
"""
from __future__ import annotations

import math
import multiprocessing as mp
import os
import sys
import time

import mujoco
import numpy as np
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
import plant_v5b as P  # noqa: E402
from design_gates import SERVOS, V, hull, hull_margin, minjerk, pad_xy  # noqa: E402

OUT = os.path.join(HERE, "stance_v5b.txt")
Z0 = 1.0                      # kinematics are done 1 m in the air: no floor contact
DROP, LIFT = 0.020, 0.030     # the A study's crouch and lift
BENCH_DROP, BENCH_LIFT = 0.020, 0.020   # the bench sequence's: a 30 mm lift at a 20 mm crouch needs the swing ankle past -40 deg
                                        # (firmware limit), and 25 mm leaves the d sweep infeasible with 2 deg spare; 20 mm fits every row
SPARE = 2.0                   # deg every bench target keeps from the firmware envelope
# firmware/main/mech_envelope.h, deg (sign convention of the plant); yaw stays 0
ENVELOPE = {"hip_yaw": (-45, 45), "hip_pitch": (-110, 90), "knee": (-95, 95), "ankle": (-40, 40),
            "ankle_roll": (-25, 25), "L_hip_roll": (-25, 55), "R_hip_roll": (-55, 25)}


def env_limits(name):
    j = name[2:]
    return ENVELOPE[name if j == "hip_roll" else j]


def env_ok(name, deg):
    lo, hi = env_limits(name)
    if name.endswith("hip_yaw") and abs(deg) > 1.0:
        return False
    return lo + SPARE <= deg <= hi - SPARE


def env_ok_all(qvec_deg):
    return all(env_ok(n, qvec_deg[i]) for i, n in enumerate(P.JOINTS))
KP0, KD0 = 12.0, 0.25         # walker_env sts3215 PD, N*m/rad, N*m*s/rad
KP_MEASURED = 17.0            # docs/servo-map.md s5: stance hip roll at P=32
JOINT_DAMPING = 0.1           # walker_env servo_joint_damping
STALL = SERVOS["sts3215"]["stall"]
W0 = SERVOS["sts3215"]["w0"]
NPROC = 8
ROLLS_KNEE = ("hip_roll", "knee", "ankle_roll")


def load_v5body(wide_ankle=False):
    """the unmodified prototype; wide_ankle widens the ankle-pitch range from
    its +-40 to the v6 ankle's +-45 deg, to separate range from roll"""
    m = mujoco.MjModel.from_xml_path(P.V5)
    if wide_ankle:
        for n in ("L_ankle", "R_ankle"):
            m.jnt_range[m.joint(n).id] = np.radians([-45, 45])
    return m


# --------------------------------------------------------------------------- kinematics
class Kin:
    """numerical leg IK + pose evaluation on one plant (feet on z = Z0)"""

    def __init__(self, m, stance="L"):
        self.st, self.sw = stance, "R" if stance == "L" else "L"
        self.sgn = 1.0 if stance == "L" else -1.0   # + y toward the stance side
        self.m = m
        self.d = mujoco.MjData(m)
        self.roll = "L_ankle_roll" in [m.joint(i).name for i in range(m.njnt)]
        P.set_stand(m, self.d)
        self.zst = float(self.d.qpos[2])
        self.foot0 = {s: self.d.xpos[m.body(f"{s}_foot").id].copy() for s in "LR"}
        self.legj = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"] + (["ankle_roll"] if self.roll else [])
        self.adr = {s: [m.jnt_qposadr[m.joint(f"{s}_{j}").id] for j in self.legj] for s in "LR"}
        self.lim = {s: m.jnt_range[[m.joint(f"{s}_{j}").id for j in self.legj]] for s in "LR"}
        self.pads = {s: [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{s}_pad_")]
                     for s in "LR"}
        t = 0.4
        self.seed = np.array([0, 0, -t, -2 * t, -t] + ([0.0] if self.roll else []))

    def _torso(self, xyz, theta):
        q = self.d.qpos
        q[0:3] = xyz
        q[3:7] = [math.cos(theta / 2), math.sin(theta / 2), 0, 0]

    def _res(self, s, target, comps):
        b = self.m.body(f"{s}_foot").id
        R = self.d.xmat[b].reshape(3, 3)
        o = {"pitch": R[2, 0], "roll": R[2, 1], "yaw": R[1, 0]}
        return np.r_[self.d.xpos[b] - target, [o[c] for c in comps]]

    def solve_leg(self, s, xyz, theta, target, comps, q0, free_theta=False):
        n = len(self.adr[s])

        def f(u):
            self.d.qpos[self.adr[s]] = u[:n]
            self._torso(xyz, u[n] if free_theta else theta)
            mujoco.mj_kinematics(self.m, self.d)
            return self._res(s, target, comps)
        u = np.r_[q0, theta] if free_theta else np.array(q0, float)
        for _ in range(60):
            r = f(u)
            if np.linalg.norm(r) < 1e-9:
                break
            J = np.empty((len(r), len(u)))
            for k in range(len(u)):
                du = np.zeros(len(u)); du[k] = 1e-6
                J[:, k] = (f(u + du) - r) / 1e-6
            step = -np.linalg.solve(J.T @ J + 1e-9 * np.eye(len(u)), J.T @ r)
            u = u + np.clip(step, -0.15, 0.15)
        r = f(u)
        return u, bool(np.linalg.norm(r) < 1e-6)

    def pose(self, tx, ty, tz, theta, h, sdx=0.0, sdy=0.0, q0=None):
        """both soles flat, left on the floor at its standing spot, right `h`
        above it and offset (sdx, sdy). Returns (q dict {s: angles}, theta, ok)."""
        q0 = q0 or {"L": self.seed, "R": self.seed}
        xyz = np.array([tx, ty, tz + Z0])
        st, sw = self.st, self.sw
        tL = self.foot0[st] + [0, 0, Z0]
        tR = self.foot0[sw] + [sdx, sdy, Z0 + h]
        allc = ["pitch", "roll", "yaw"]
        if self.roll:
            uL, okL = self.solve_leg(st, xyz, theta, tL, allc, q0[st])
            th, uL = theta, uL
            uR, okR = self.solve_leg(sw, xyz, th, tR, allc, q0[sw])
        else:
            uL, okL = self.solve_leg(st, xyz, theta, tL, allc, q0[st], free_theta=True)
            th, uL = uL[-1], uL[:-1]
            uR, okR = self.solve_leg(sw, xyz, th, tR, ["pitch", "yaw"], q0[sw])
        return {st: uL, sw: uR}, th, okL and okR

    def set_state(self, q, th, tx, ty, tz):
        for s in "LR":
            self.d.qpos[self.adr[s]] = q[s]
        self._torso([tx, ty, tz + Z0], th)
        mujoco.mj_forward(self.m, self.d)

    def evaluate(self, tx, ty, theta, sdx, sdy, drop=DROP, h=LIFT, spare=0.0):
        """None if infeasible, else dict(margin, q, theta, ...)"""
        tz = self.zst - drop
        q, th, ok = self.pose(tx, ty, tz, theta, h, sdx, sdy)
        if not ok or abs(th) > math.radians(30):
            return None
        for s in "LR":
            sp = math.radians(spare)
            if np.any(q[s] < self.lim[s][:, 0] + sp - 1e-6) or np.any(q[s] > self.lim[s][:, 1] - sp + 1e-6):
                return None
        self.set_state(q, th, tx, ty, tz)
        if self.d.ncon:
            return None
        return dict(st=self.st, margin=self.margin(), q=q, theta=th, com=self.d.subtree_com[0].copy(),
                    xyz=(tx, ty, tz), swing=(sdx, sdy))

    def margin(self, d=None):
        d = d or self.d
        hl = hull([p for p in (d.geom_xpos[g][:2] for g in self.pads[self.st])])
        return hull_margin(hl, d.subtree_com[0][:2])


# --------------------------------------------------------------------------- search (multiprocess)
_K = {}


def _init():
    _K["v5b"] = Kin(P.build())
    _K["v5body"] = Kin(load_v5body())


def _x_to_args(x, roll, free_swing):
    tx, ty = x[0], x[1]
    i = 2
    th = 0.0
    if roll:
        th = math.radians(x[i]); i += 1
    sdx, sdy = (x[i], x[i + 1]) if free_swing else (0.0, 0.0)
    return tx, ty, th, sdx, sdy


BENCH_MARGIN = 0.025       # bench pose: margin capped here, then least roll-joint angle
ROLL_W = 0.01              # m of margin traded per rad of stance hip/ankle roll
TILT_W = 0.5               # ... and per rad of pelvis roll: the bench pose keeps the torso level


def _score(r, mode):
    if r is None:
        return -1.0
    if mode == "margin":
        return r["margin"]
    qL = r["q"][r["st"]]
    return min(r["margin"], BENCH_MARGIN) - ROLL_W * (abs(qL[1]) + abs(qL[5])) - TILT_W * abs(r["theta"])


def _bounds(roll, free_swing):
    """search box: pelvis x, y [m], pelvis roll [deg] (v5b), swing foot x, y offsets [m]"""
    lo = [-0.06, 0.0] + ([-12.0] if roll else []) + ([-0.04, -0.03] if free_swing else [])
    hi = [0.06, 0.09] + ([12.0] if roll else []) + ([0.06, 0.03] if free_swing else [])
    return np.array(lo), np.array(hi)


def _eval_x(args):
    x, free_swing, mode, which = args
    k = _K[which]
    lo, hi = _bounds(k.roll, free_swing)
    if np.any(np.asarray(x) < lo) or np.any(np.asarray(x) > hi):
        return -1.0
    return _score(k.evaluate(*_x_to_args(x, k.roll, free_swing)), mode)


def _refine(args):
    x0, free_swing, mode, which = args
    res = minimize(lambda x: -_eval_x((x, free_swing, mode, which)), x0, method="Nelder-Mead",
                   options=dict(xatol=2e-5, fatol=1e-7, maxiter=500))
    return res.x, -res.fun


def search(pool, which, free_swing, n=6000, seed=0, mode="margin"):
    rng = np.random.default_rng(seed)
    lo, hi = _bounds(which == "v5b", free_swing)
    xs = rng.uniform(lo, hi, size=(n, len(lo)))
    ms = np.array(pool.map(_eval_x, [(x, free_swing, mode, which) for x in xs], chunksize=50))
    top = np.argsort(-ms)[:NPROC]
    ref = pool.map(_refine, [(xs[i], free_swing, mode, which) for i in top])
    x, sc = max(ref, key=lambda r: r[1])
    return x, sc, int((ms > -1).sum()), n


def describe_pose(k, args):
    return k.evaluate(*args)


# --------------------------------------------------------------------------- lateral (bench) poses
def lateral_pose(k, ty):
    """pelvis level, x = 0, crouch z, right foot straight up: the bench family"""
    return k.evaluate(0.0, ty, 0.0, 0.0, 0.0, BENCH_DROP, BENCH_LIFT, SPARE)


def bench_pose(k):
    best = None
    for ty in k.sgn * np.arange(0.02, 0.11, 0.0005):
        r = lateral_pose(k, ty)
        sc = _score(r, "bench")
        if r is not None and (best is None or sc > best[0]):
            best = (sc, ty, r)
    return best[1], best[2]


def solve_d(k, d):
    """pelvis y whose whole-robot CoM sits d (+ outboard) from the stance ankle-roll axis"""
    target = k.foot0[k.st][1] + k.sgn * d
    pts = []
    for ty in k.sgn * np.arange(0.0, 0.14, 0.001):
        r = lateral_pose(k, ty)
        if r is not None:
            pts.append((ty, r["com"][1] - target))
    for (a, fa), (b, fb) in zip(pts, pts[1:]):
        if fa == 0 or fa * fb < 0:
            y0, f0, y1, f1 = a, fa, b, fb
            for _ in range(8):
                y2 = y1 - f1 * (y1 - y0) / (f1 - f0)
                r = lateral_pose(k, y2)
                if r is None:
                    break
                y0, f0, y1, f1 = y1, f1, y2, r["com"][1] - target
                if abs(f1) < 1e-6:
                    break
            r = lateral_pose(k, y1)
            if r is not None:
                return y1, r
    return None, None


def fore_aft(k):
    """CoM x relative to the stance ankle-pitch axis, and its distance to the nearer pad-hull end"""
    d = k.d
    px = [d.geom_xpos[g][0] for g in k.pads[k.st]]
    cx = d.subtree_com[0][0]
    return cx - d.xpos[k.m.body(f"{k.st}_foot").id][0], min(cx - min(px), max(px) - cx)


def static_ankle_roll(k):
    """gravity moment about the stance ankle-roll axis of everything above it
    (m_above * g * y offset of its CoM from the axis); the servo holds this"""
    m, d = k.m, k.d
    fb = m.body(f"{k.st}_foot").id
    mt, mf = m.body_subtreemass[0], m.body_mass[fb]
    cy = (mt * d.subtree_com[0][1] - mf * d.xipos[fb][1]) / (mt - mf)
    return (mt - mf) * 9.81 * k.sgn * (cy - d.xpos[fb][1])


# --------------------------------------------------------------------------- dynamics
GAINS = {
    "stock (12 N*m/rad, deploy fit)": dict(base=1.0, raised=1.0),
    "stock, measured (17 N*m/rad)": dict(base=KP_MEASURED / KP0, raised=KP_MEASURED / KP0),
    "Plan B x2.8 (hip roll, knee, ankle roll)": dict(base=1.0, raised=2.8),
    "Plan B x3.5 (hip roll, knee, ankle roll)": dict(base=1.0, raised=3.5),
    "all joints x3.5": dict(base=3.5, raised=3.5),
}
SWEEP_GAINS = {k: GAINS[k] for k in list(GAINS)[:2]}
D_LIST = (-0.010, -0.005, 0.0, 0.005, 0.010, 0.015, 0.0185, 0.025)


def dyn_model(which="v5b"):
    m = P.build() if which == "v5b" else load_v5body()
    m.actuator_gainprm[:] = 0.0
    m.actuator_biasprm[:] = 0.0
    m.dof_damping[6:6 + m.nu] = JOINT_DAMPING
    return m


def prelude(ty, t_shift=2.0):
    """stand 0.5 s, crouch 1 s, lateral shift, lift 1 s, settle 1 s. Poses are
    (x, y, drop below straight, pelvis roll, right-foot lift)."""
    return [(0.5, (0, 0, 0, 0, 0)), (1.0, (0, 0, BENCH_DROP, 0, 0)), (t_shift, (0, ty, BENCH_DROP, 0, 0)),
            (1.0, (0, ty, BENCH_DROP, 0, BENCH_LIFT)), (1.0, (0, ty, BENCH_DROP, 0, BENCH_LIFT))]


def pose_at(segs, t):
    acc = 0.0
    for dur, a, b in segs:
        if t <= acc + dur + 1e-12:
            s = minjerk((t - acc) / dur)
            return tuple(np.array(a) + (np.array(b) - np.array(a)) * s)
        acc += dur
    return segs[-1][2]


def cmd_vector(k, q):
    out = np.zeros(12)
    for i, n in enumerate(P.JOINTS):
        s, j = n[0], n[2:]
        out[i] = q[s][k.legj.index(j)]
    return out


def simulate(which, gain_cfg, wps, windows, stance="L", kp0=KP0, kd0=KD0):
    """wps: [(duration, pose)], the first is an initial hold, each next one a
    min-jerk move from the previous pose. windows: {name: (t_move0, t_hold0,
    t_end, t_mean0)} in sim time. Returns the whole-run verdict and, per
    window, min margin over the hold, stance hip/ankle-roll torque and sag
    (mean over [t_mean0, t_end], peak over the move and hold) and the peak
    torque of every joint over the move and hold."""
    m = dyn_model(which)
    d = mujoco.MjData(m)
    k = Kin(m, stance)
    st = stance
    segs, prev = [], None
    for dur, p in wps:
        p = (p[0], p[1], k.zst - p[2], p[3], p[4])
        segs.append((dur, prev if prev is not None else p, p))
        prev = p
    T = sum(s[0] for s in segs)
    jid = [m.joint(n).id for n in P.JOINTS]
    qadr, dadr = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
    scale = np.array([gain_cfg["raised"] if n[2:] in ROLLS_KNEE else gain_cfg["base"] for n in P.JOINTS])
    kp, kd = kp0 * scale, kd0 * scale
    p0 = pose_at(segs, 0.0)
    q, th, ok = k.pose(*p0)
    cmd = cmd_vector(k, q)
    d.qpos[:] = 0
    d.qpos[qadr] = cmd
    d.qpos[0:3] = [p0[0], p0[1], p0[2] + 0.001]
    d.qpos[3:7] = [math.cos(th / 2), math.sin(th / 2), 0, 0]
    mujoco.mj_forward(m, d)
    n_sub = int(round(1 / 50 / m.opt.timestep))
    rec = dict(t=[], margin=[], tau=[], taupk=[], err=[], up=[], stat=[], off=[])
    t, qprev, fell = 0.0, q, False
    cmin, cmax = np.degrees(cmd), np.degrees(cmd)
    while t < T - 1e-9:
        t += 0.02
        p = pose_at(segs, t)
        qn, th, ok = k.pose(*p, q0=qprev)
        if ok:
            qprev, cmd = qn, cmd_vector(k, qn)
            cmin, cmax = np.minimum(cmin, np.degrees(cmd)), np.maximum(cmax, np.degrees(cmd))
        tpk = np.zeros(12)
        for _ in range(n_sub):
            qq, qd = d.qpos[qadr], d.qvel[dadr]
            cap = STALL * np.clip(1 - np.abs(qd) / W0, 0, 1)
            tau = np.clip(kp * (cmd - qq) - kd * qd, -cap, cap)
            d.qfrc_applied[dadr] = tau
            mujoco.mj_step(m, d)
            tpk = np.maximum(tpk, np.abs(tau))
        up = float(d.xmat[m.body("torso").id].reshape(3, 3)[2, 2])
        hl = hull([d.geom_xpos[g][:2] for g in k.pads[st]])
        rec["t"].append(t); rec["margin"].append(hull_margin(hl, d.subtree_com[0][:2]))
        rec["tau"].append(tau.copy()); rec["taupk"].append(tpk); rec["err"].append(np.degrees(cmd - d.qpos[qadr]))
        rec["up"].append(up)
        fb = m.body(f"{st}_foot").id
        mt, mf = m.body_subtreemass[0], m.body_mass[fb]
        c = (mt * d.subtree_com[0] - mf * d.xipos[fb]) / (mt - mf)
        mom = np.cross(c - d.xpos[fb], [0, 0, -(mt - mf) * 9.81])
        sm = -k.sgn * float(mom @ d.xmat[fb].reshape(3, 3)[:, 0])   # + = mass outboard of the axis
        rec["stat"].append(sm); rec["off"].append(sm / ((mt - mf) * 9.81))
        if up < 0.8:
            fell = True
            break
    r = {key: np.array(v) for key, v in rec.items()}
    ids = {n: i for i, n in enumerate(P.JOINTS)}
    out = dict(fell=fell, t_end=t, windows={}, cmd_min=cmin, cmd_max=cmax)
    for name, (tm, th0, te, tmean) in windows.items():
        if fell and r["t"][-1] < te - 1e-6:
            out["windows"][name] = dict(fell=True)
            continue
        mv = (r["t"] > tm + 1e-9) & (r["t"] <= te + 1e-9)
        hd = (r["t"] > th0 + 1e-9) & (r["t"] <= te + 1e-9)
        mn = (r["t"] > tmean + 1e-9) & (r["t"] <= te + 1e-9)
        w = dict(fell=False, min_margin=float(r["margin"][hd].min()), tau_pk=r["taupk"][mv].max(0),
                 tilt=math.degrees(math.acos(max(-1, min(1, r["up"][mv].min())))))
        w["stat_sim"] = float(r["stat"][mn].mean())
        w["off_sim"] = float(r["off"][mn].mean())
        for nm in ("hip_roll", "ankle_roll"):
            i = ids[f"{st}_{nm}"]
            w[nm] = dict(tau_mean=float(r["tau"][mn, i].mean()), tau_pk=float(r["taupk"][mv, i].max()),
                         sag_mean=float(r["err"][mn, i].mean()), sag_pk=float(np.abs(r["err"][mv, i]).max()))
        out["windows"][name] = w
    return out


def _sim_job(a):
    return simulate(*a)


# --------------------------------------------------------------------------- report
def pose_lines(label, k, r, tag):
    com = r["com"]
    fa, fam = fore_aft(k)
    out = [f"\n{label:20s} {tag}: margin {r['margin']*1e3:6.1f} mm",
           f"   pelvis (x, y) = ({r['xyz'][0]*1e3:.1f}, {r['xyz'][1]*1e3:.1f}) mm, pelvis roll {math.degrees(r['theta']):+.1f} deg, "
           f"swing foot offset ({r['swing'][0]*1e3:+.1f}, {r['swing'][1]*1e3:+.1f}) mm, CoM ({com[0]*1e3:.1f}, {com[1]*1e3:.1f}) mm "
           f"(stance foot origin y = {k.foot0[k.st][1]*1e3:+.0f} mm; CoM x rel. stance ankle-pitch axis {fa*1e3:+.1f} mm, "
           f"fore-aft margin to nearer pad end {fam*1e3:.1f} mm)"]
    cols = [f"{n}={math.degrees(r['q'][n[0]][k.legj.index(n[2:])]):+.1f}" for n in
            [f"{s}_{j}" for s in "LR" for j in k.legj]]
    out.append("   joints (deg): " + " ".join(cols))
    return out


def fmt_win(w, stat=None):
    if w["fell"]:
        return ["FELL"]
    a, b = w["hip_roll"], w["ankle_roll"]
    L = [f"stays up, tilt max {w['tilt']:.1f} deg; min CoM margin {w['min_margin']*1e3:5.1f} mm",
         f"      stance ankle roll tau {b['tau_mean']:+.3f} N*m (peak {b['tau_pk']:.2f})"
         + (f" vs kinematic static {abs(stat):.3f}, sim-state static {abs(w['stat_sim']):.3f}" if stat is not None else "")
         + f", sag {b['sag_mean']:+.2f} deg (peak {b['sag_pk']:.2f})",
         f"      stance hip roll   tau {a['tau_mean']:+.3f} N*m (peak {a['tau_pk']:.2f}), sag {a['sag_mean']:+.2f} deg (peak {a['sag_pk']:.2f})",
         f"      worst joint peak {w['tau_pk'].max():.2f} N*m = {w['tau_pk'].max()/STALL*100:.0f} % of stall ({P.JOINTS[int(w['tau_pk'].argmax())]})"]
    return L


def main():
    t0 = time.time()
    lines = []

    def out(s=""):
        print(s, flush=True)
        lines.append(s)

    out("# GENERATED by experiments/v5b/stance_v5b.py -- do not edit; rerun the script.")
    out("# Sim numbers are gates under the stated plant (ideal servo: no lag, dead time, backlash or joint play), not hardware predictions.")
    m5b = P.build()
    k5b = Kin(m5b)
    m5 = load_v5body()
    k5 = Kin(m5)
    d = mujoco.MjData(m5b)
    out(f"\nplants: v5b = v5body + v6 ankle link/roll servo/foot ({m5b.body_subtreemass[0]:.4f} kg), "
        f"v5body = unmodified ({m5.body_subtreemass[0]:.4f} kg)")
    P.set_stand(m5b, d)
    out(f"v5b standing CoM height {d.subtree_com[0][2]*1e3:.1f} mm (torso z {d.qpos[2]*1e3:.1f} mm)")
    d5 = mujoco.MjData(m5)
    P.set_stand(m5, d5)
    out(f"v5body standing CoM height {d5.subtree_com[0][2]*1e3:.1f} mm (torso z {d5.qpos[2]*1e3:.1f} mm)")
    out("sign conventions (plant): hip_roll +X (positive = foot toward robot left), hip_pitch +Y (positive = foot backward),")
    out("  knee -Y (negative = flexion), ankle +Y (positive = toe down), ankle_roll +X, hip_yaw +Z. Left = +Y.")

    # ---------------- A
    out(f"\n== A  kinematic: pelvis {DROP*1e3:.0f} mm below straight, swing sole flat {LIFT*1e3:.0f} mm up, stance = LEFT")
    pool = mp.Pool(NPROC, initializer=_init)
    out("margin = CoM ground projection to the edge of the hull of the stance pad centres (+ inside); pads are 3 mm spheres on the sole outline")
    kins = {"v5b": k5b, "v5body": k5}
    for which, free in (("v5b", True), ("v5b", False), ("v5body", True), ("v5body", False)):
        x, sc, nval, n = search(pool, which, free, seed=1)
        k = kins[which]
        r = k.evaluate(*_x_to_args(x, k.roll, free))
        label = which
        kind = "swing foot free (x,y)" if free else "swing foot straight up"
        for ln in pose_lines(label, k, r, f"{kind}, best margin ({nval}/{n} random poses feasible)"):
            out(ln)
    out("\n   (the v5b margin saturates near the sole half-width: the CoM is then over the sole centreline)")

    ty_b, rb = bench_pose(k5b)
    lateral_pose(k5b, ty_b)
    for ln in pose_lines("v5b BENCH POSE", k5b, rb, f"pure lateral shift (pelvis x, z and level roll held from the crouch), swing foot straight up; "
                         f"margin capped at {BENCH_MARGIN*1e3:.0f} mm, least roll angles"):
        out(ln)
    out(f"   CoM y offset from the stance ankle-roll axis d = {(rb['com'][1]-k5b.foot0['L'][1])*1e3:+.1f} mm (+ = outboard)")
    hold_pose = (0.0, ty_b, 0.0)

    # ---------------- D kinematics (d sweep)
    out(f"\n== D  one-foot CoM sweep: bench crouch {BENCH_DROP*1e3:.0f} mm, pelvis level, x held, right foot {BENCH_LIFT*1e3:.0f} mm up; d = CoM ground projection offset from the stance")
    out("   ankle-roll axis (+ = outboard, toward the stance foot's outer edge; the sole centreline is d = +18.5 mm)")
    sweep = []
    for dd in D_LIST:
        ty, r = solve_d(k5b, dd)
        if r is None:
            out(f"   d = {dd*1e3:+5.1f} mm: no feasible pose (IK/joint limits/self-contact)")
            continue
        lateral_pose(k5b, ty)
        fa, fam = fore_aft(k5b)
        sweep.append(dict(d=dd, ty=ty, r=r, stat=static_ankle_roll(k5b), fa=fa, fam=fam))
    sweep_b = dict(d=rb["com"][1] - k5b.foot0["L"][1], ty=ty_b, r=rb)
    lateral_pose(k5b, ty_b)
    sweep_b["stat"] = static_ankle_roll(k5b)

    # ---------------- C keyframes
    segs_k = prelude(ty_b)
    kf = {}
    qprev = None
    for name, p in (("stand", (0, 0, 0, 0, 0)), ("crouch", (0, 0, BENCH_DROP, 0, 0)), ("shift", (0, ty_b, BENCH_DROP, 0, 0)),
                    ("one-foot", (0, ty_b, BENCH_DROP, 0, BENCH_LIFT))):
        q, th, ok = k5b.pose(p[0], p[1], k5b.zst - p[2], p[3], p[4], q0=qprev)
        qprev = q
        kf[name] = (cmd_vector(k5b, q), p, ok)
    out("\n== C  keyframes for the bench (joint targets, deg, plant sign conventions above; one-foot = straight lift of the right foot)")
    out("  (servo zero/direction calibration still applies: these are plant joint angles, not raw servo ticks)")
    out(f"{'ID':>3s} {'joint':13s} {'stand':>8s} {'crouch':>8s} {'shift':>8s} {'one-foot':>8s}")
    for n in sorted(P.JOINTS, key=lambda n: P.SERVO_ID[n]):
        i = P.JOINTS.index(n)
        out(f"{P.SERVO_ID[n]:3d} {n:13s} " + " ".join(f"{math.degrees(kf[c][0][i]):8.1f}" for c in ("stand", "crouch", "shift", "one-foot")))
    out(f"{'':3s} {'envelope':13s} " + " ".join(f"{('ok' if env_ok_all(np.degrees(kf[c][0])) else 'FAIL'):>8s}" for c in ("stand", "crouch", "shift", "one-foot")))
    out(f"  envelope = firmware/main/mech_envelope.h with >= {SPARE:.0f} deg spare (ankle +-40, roll +-25, hip roll L -25..55 / R -55..25, pitch -110..90, knee +-95, yaw 0)")
    out(f"  pelvis: x 0 throughout; crouch z {kf['crouch'][1][2]*1e3:.0f} mm below straight; shift/one-foot y = {ty_b*1e3:.1f} mm "
        f"(d = {sweep_b['d']*1e3:+.1f} mm); right foot +{BENCH_LIFT*1e3:.0f} mm at one-foot")
    out(f"  IK ok at all keyframes: {all(kf[c][2] for c in kf)}")

    out(f"\n== D keyframes: one-foot joint targets (deg) at each CoM offset d (right foot {BENCH_LIFT*1e3:.0f} mm up, pelvis level, x/z held)")
    cols = [("bench", sweep_b)] + [(f"d={s['d']*1e3:+.1f}", s) for s in sweep]
    out(f"{'ID':>3s} {'joint':13s} " + " ".join(f"{c:>9s}" for c, _ in cols))
    for n in sorted(P.JOINTS, key=lambda n: P.SERVO_ID[n]):
        out(f"{P.SERVO_ID[n]:3d} {n:13s} " + " ".join(f"{math.degrees(s['r']['q'][n[0]][k5b.legj.index(n[2:])]):9.1f}" for _, s in cols))
    out(f"{'':3s} {'envelope':13s} " + " ".join(f"{('ok' if env_ok_all(np.array([math.degrees(s['r']['q'][n[0]][k5b.legj.index(n[2:])]) for n in P.JOINTS])) else 'FAIL'):>9s}" for _, s in cols))
    out(f"{'':3s} {'pelvis y mm':13s} " + " ".join(f"{s['ty']*1e3:9.1f}" for _, s in cols))
    out(f"{'':3s} {'margin mm':13s} " + " ".join(f"{s['r']['margin']*1e3:9.1f}" for _, s in cols))
    out(f"{'':3s} {'CoM x mm*':13s} " + " ".join(f"{(s['r']['com'][0]-0.0)*1e3:9.1f}" for _, s in cols))
    out("   * CoM x relative to the stance ankle-pitch axis (x = 0)")
    out(f"{'':3s} {'static tau Nm':13s} " + " ".join(f"{abs(s['stat']):9.3f}" for _, s in cols))

    # ---------------- B
    out(f"\n== B  dynamic (bench lift {BENCH_LIFT*1e3:.0f} mm): stand 0.5 s, crouch 1 s, shift T, lift 1 s, hold 5 s; servo = PD clamped to STS3215 envelope "
        f"(stall {STALL:.2f} N*m, no-load {W0:.2f} rad/s at 11.1 V)")
    out(f"   pelvis target for the hold: x 0 mm, y {ty_b*1e3:.1f} mm, level (bench pose from A)")
    jobs, meta = [], []
    for name, cfg in GAINS.items():
        for ts in (1.0, 2.0, 4.0):
            pre = prelude(ty_b, ts)[:4] + [(5.0, (0, ty_b, BENCH_DROP, 0, BENCH_LIFT))]
            t_end = sum(w[0] for w in pre)
            tl = t_end - 5.0
            jobs.append(("v5b", cfg, pre, {"hold": (tl - 1.0, tl, t_end, tl)}))
            meta.append((name, ts))
    # ---------------- D dynamic: per-d and continuous
    pre = prelude(ty_b)
    t_pre = sum(w[0] for w in pre)
    for gname, cfg in SWEEP_GAINS.items():
        for s in sweep:
            wps = pre + [(2.0, (0, s["ty"], BENCH_DROP, 0, BENCH_LIFT)), (3.0, (0, s["ty"], BENCH_DROP, 0, BENCH_LIFT))]
            jobs.append(("v5b", cfg, wps, {"d": (t_pre, t_pre + 2.0, t_pre + 5.0, t_pre + 3.0)}))
            meta.append(("D", gname, s["d"]))
        wps, win, t = list(pre), {}, t_pre
        for s in sweep + [sweep_b]:
            wps += [(2.0, (0, s["ty"], BENCH_DROP, 0, BENCH_LIFT)), (3.0, (0, s["ty"], BENCH_DROP, 0, BENCH_LIFT))]
            win[s["d"]] = (t, t + 2.0, t + 5.0, t + 3.0)
            t += 5.0
        wps += [(1.5, (0, ty_b, BENCH_DROP, 0, 0)), (1.0, (0, ty_b, BENCH_DROP, 0, 0))]
        win["down"] = (t, t + 1.5, t + 2.5, t + 1.5)
        jobs.append(("v5b", cfg, wps, win))
        meta.append(("CONT", gname, None))
    rs = pool.map(_sim_job, jobs, chunksize=1)

    cur = None
    for (key, r) in zip(meta, rs):
        if key[0] in ("D", "CONT"):
            continue
        name, ts = key
        if name != cur:
            out(f"\n-- gain: {name}")
            cur = name
        w = r["windows"]["hold"]
        if w["fell"]:
            out(f"   shift {ts:.0f} s: FELL")
            continue
        out(f"   shift {ts:.0f} s: " + fmt_win(w)[0])
        for ln in fmt_win(w)[1:]:
            out("   " + ln)
        pk = ", ".join(f"{n}={w['tau_pk'][i]:.2f}" for i, n in enumerate(P.JOINTS))
        out(f"      peak |tau| N*m (stall {STALL:.2f}): {pk}")

    out("\n== D dynamic: per-d (prelude to the bench pose, 2 s min-jerk move to d, 3 s hold; torque/sag = mean over the last 2 s of the hold,")
    out("   peaks over move+hold; kinematic static = m_above_axis * g * lateral offset of the above-axis CoM from the axis, at the commanded pose;")
    out("   sim-state static = the same moment from the simulated state during the hold: gravity moment of everything above the ankle-roll axis about")
    out("   that axis (it includes the sag of the hip and ankle roll, so it shows how much the compliance adds))")
    stat_of = {s["d"]: s["stat"] for s in sweep}
    for gname in SWEEP_GAINS:
        out(f"\n-- gain: {gname}")
        for (key, r) in zip(meta, rs):
            if key[0] == "D" and key[1] == gname:
                w = r["windows"]["d"]
                body = fmt_win(w, stat_of[key[2]])
                out(f"   d = {key[2]*1e3:+5.1f} mm: " + body[0])
                for ln in body[1:]:
                    out("   " + ln)
        for (key, r) in zip(meta, rs):
            if key[0] == "CONT" and key[1] == gname:
                out(f"\n   CONTINUOUS sequence (prelude, then d = {', '.join(f'{s['d']*1e3:+.1f}' for s in sweep)}, back to the bench pose, foot down): "
                    f"{'FELL at t=%.1f s' % r['t_end'] if r['fell'] else 'stays up throughout'}")
                bad = [f"{n} [{r['cmd_min'][i]:.1f}, {r['cmd_max'][i]:.1f}]" for i, n in enumerate(P.JOINTS)
                       if not (env_limits(n)[0] + SPARE <= r['cmd_min'][i] and r['cmd_max'][i] <= env_limits(n)[1] - SPARE)
                       and not (n.endswith('hip_yaw') and max(abs(r['cmd_min'][i]), abs(r['cmd_max'][i])) <= 1.0)]
                out(f"   commanded targets over the whole run vs envelope (>= {SPARE:.0f} deg spare): " + ("all inside" if not bad else "OUTSIDE: " + "; ".join(bad)))
                for dk, w in r["windows"].items():
                    lab = "foot down" if dk == "down" else f"d = {dk*1e3:+5.1f} mm"
                    st = stat_of.get(dk, sweep_b["stat"] if dk == sweep_b["d"] else None)
                    body = fmt_win(w, st)
                    out(f"     {lab}: " + body[0])
                    for ln in body[1:]:
                        out("     " + ln)
    out("\n== D static check: lateral offset (mm) of the mass above the stance ankle-roll axis, kinematic vs simulated hold state, and the torque each implies (N*m)")
    m_above = m5b.body_subtreemass[0] - m5b.body_mass[m5b.body("L_foot").id]
    out(f"   m_above = {m_above:.4f} kg; amplification = sim-state offset - kinematic offset; sim tau = servo torque in the sim")
    for gname in SWEEP_GAINS:
        out(f"\n-- gain: {gname}")
        out(f"   {'d mm':>6s} {'kin off':>8s} {'sim off':>8s} {'ampl mm':>8s} {'kin tau':>8s} {'sim-state tau':>13s} {'sim tau':>8s} {'sim/sim-state':>13s}")
        for (key, r) in zip(meta, rs):
            if key[0] == "D" and key[1] == gname:
                w = r["windows"]["d"]
                if w["fell"]:
                    continue
                ks = stat_of[key[2]]
                out(f"   {key[2]*1e3:+6.1f} {ks/(m_above*9.81)*1e3:+8.1f} {w['off_sim']*1e3:+8.1f} {(w['off_sim']-ks/(m_above*9.81))*1e3:+8.1f} "
                    f"{abs(ks):8.3f} {abs(w['stat_sim']):13.3f} {abs(w['ankle_roll']['tau_mean']):8.3f} "
                    f"{abs(w['ankle_roll']['tau_mean'])/max(abs(w['stat_sim']),1e-6):13.2f}")
    # ---------------- right-foot stance (left foot lifted), solved on the plant
    out("\n\n==================== RIGHT-FOOT STANCE (left foot lifted), solved on the plant, not mirrored ====================")
    out("   same bench family: pelvis x 0, level, crouch %.0f mm, left foot straight up %.0f mm; d = CoM offset from the stance (right) ankle-roll axis, + outboard (toward -y)"
        % (BENCH_DROP * 1e3, BENCH_LIFT * 1e3))
    kR = Kin(m5b, "R")
    tyR, rbR = bench_pose(kR)
    lateral_pose(kR, tyR)
    fa, fam = fore_aft(kR)
    out(f"   bench pose: pelvis y {tyR*1e3:+.1f} mm, margin {rbR['margin']*1e3:.1f} mm, d = {(rbR['com'][1]-kR.foot0['R'][1])*-1e3:+.1f} mm, "
        f"CoM x rel. stance ankle axis {fa*1e3:+.1f} mm, fore-aft margin {fam*1e3:.1f} mm")
    sweepR = []
    for dd in D_LIST:
        ty, r = solve_d(kR, dd)
        if r is None:
            out(f"   d = {dd*1e3:+5.1f} mm: no feasible pose")
            continue
        lateral_pose(kR, ty)
        sweepR.append(dict(d=dd, ty=ty, r=r, stat=static_ankle_roll(kR)))
    sweepR_b = dict(d=-(rbR["com"][1] - kR.foot0["R"][1]), ty=tyR, r=rbR)
    lateral_pose(kR, tyR)
    sweepR_b["stat"] = static_ankle_roll(kR)

    def vec(k, q):
        return np.degrees(cmd_vector(k, q))

    def mirror_of(v):
        out_v = np.zeros(12)
        for i, n in enumerate(P.JOINTS):
            o = P.JOINTS.index(("L" if n[0] == "R" else "R") + n[1:])
            out_v[i] = (-1 if n[2:] in ("hip_roll", "ankle_roll", "hip_yaw") else 1) * v[o]
        return out_v

    def diffrows(colsR, colsL):
        mx, who = [], []
        for vr, vl in zip(colsR, colsL):
            dlt = np.abs(vr - mirror_of(vl))
            mx.append(dlt.max()); who.append(P.JOINTS[int(dlt.argmax())])
        return mx, who

    qprev, kfR = None, {}
    for name, p in (("stand", (0, 0, 0, 0, 0)), ("crouch", (0, 0, BENCH_DROP, 0, 0)), ("shift", (0, tyR, BENCH_DROP, 0, 0)),
                    ("one-foot", (0, tyR, BENCH_DROP, 0, BENCH_LIFT))):
        q, th, ok = kR.pose(p[0], p[1], kR.zst - p[2], p[3], p[4], q0=qprev)
        qprev = q
        kfR[name] = vec(kR, q)
    names = ("stand", "crouch", "shift", "one-foot")
    mx, who = diffrows([kfR[c] for c in names], [np.degrees(kf[c][0]) for c in names])
    out("\n== C (right stance) keyframes: joint targets, deg, plant sign conventions")
    out(f"{'ID':>3s} {'joint':13s} {'stand':>8s} {'crouch':>8s} {'shift':>8s} {'one-foot':>8s}")
    for n in sorted(P.JOINTS, key=lambda n: P.SERVO_ID[n]):
        i = P.JOINTS.index(n)
        out(f"{P.SERVO_ID[n]:3d} {n:13s} " + " ".join(f"{kfR[c][i]:8.1f}" for c in names))
    out(f"{'':3s} {'envelope':13s} " + " ".join(f"{('ok' if env_ok_all(kfR[c]) else 'FAIL'):>8s}" for c in names))
    out(f"{'':3s} {'max|diff| mirror':13s} " + " ".join(f"{x:8.2f}" for x in mx) + "   (deg, vs the mirrored left-stance value: roll/yaw signs flipped, sides swapped)")
    out(f"{'':3s} {'at joint':13s} " + " ".join(f"{w[2:] if False else w:>8s}" for w in who))

    colsR = [("bench", sweepR_b)] + [(f"d={s['d']*1e3:+.1f}", s) for s in sweepR]
    vR = [vec(kR, c["r"]["q"]) for _, c in colsR]
    leftcols = [("bench", sweep_b)] + [(f"d={s['d']*1e3:+.1f}", s) for s in sweep]
    vL_by = {c: np.array([math.degrees(s["r"]["q"][n[0]][k5b.legj.index(n[2:])]) for n in P.JOINTS]) for c, s in leftcols}
    mxd, whod = diffrows(vR, [vL_by[c] for c, _ in colsR if c in vL_by])
    out("\n== D keyframes (right stance): one-foot joint targets (deg), left foot lifted")
    out(f"{'ID':>3s} {'joint':13s} " + " ".join(f"{c:>9s}" for c, _ in colsR))
    for n in sorted(P.JOINTS, key=lambda n: P.SERVO_ID[n]):
        i = P.JOINTS.index(n)
        out(f"{P.SERVO_ID[n]:3d} {n:13s} " + " ".join(f"{v[i]:9.1f}" for v in vR))
    out(f"{'':3s} {'envelope':13s} " + " ".join(f"{('ok' if env_ok_all(v) else 'FAIL'):>9s}" for v in vR))
    out(f"{'':3s} {'pelvis y mm':13s} " + " ".join(f"{c['ty']*1e3:9.1f}" for _, c in colsR))
    out(f"{'':3s} {'margin mm':13s} " + " ".join(f"{c['r']['margin']*1e3:9.1f}" for _, c in colsR))
    out(f"{'':3s} {'max|diff| mirror':13s} " + " ".join(f"{x:9.2f}" for x in mxd) + "   (deg vs mirrored left-stance column)")
    out(f"{'':3s} {'at joint':13s} " + " ".join(f"{w:>9s}" for w in whod))
    out(f"   overall max |diff| vs mirror: keyframes {max(mx):.2f} deg, d sweep {max(mxd):.2f} deg")

    pre = prelude(tyR)
    t_pre = sum(w[0] for w in pre)
    cfg = SWEEP_GAINS["stock (12 N*m/rad, deploy fit)"]
    jobsR = [("v5b", cfg, pre + [(2.0, (0, c["ty"], BENCH_DROP, 0, BENCH_LIFT)), (3.0, (0, c["ty"], BENCH_DROP, 0, BENCH_LIFT))],
              {"d": (t_pre, t_pre + 2.0, t_pre + 5.0, t_pre + 3.0)}, "R") for _, c in colsR]
    rsR = pool.map(_sim_job, jobsR, chunksize=1)
    out("\n== D dynamic (right stance), stock 12 N*m/rad: prelude to the right bench pose, 2 s move to d, 3 s hold (same metrics as the left sweep)")
    for (cname, c), r in zip(colsR, rsR):
        w = r["windows"]["d"]
        body = fmt_win(w, c["stat"])
        out(f"   {cname}: " + ("FELL" if w["fell"] else body[0]))
        if not w["fell"]:
            for ln in body[1:]:
                out("   " + ln)
    pool.close()

    out(f"\n(run time {time.time()-t0:.0f} s)")
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
