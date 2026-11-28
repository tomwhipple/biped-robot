#!/usr/bin/env python3
"""Gear compliance and dead band of the leg servos, from the recorded flight data (#6).

    .venv/bin/python experiments/gear-compliance/gear_compliance.py extract [--workers 6]      # cache, once
    .venv/bin/python experiments/gear-compliance/gear_compliance.py servo  [--json servo.json]
    .venv/bin/python experiments/gear-compliance/gear_compliance.py bodyextract [--workers 6]  # kinematics
    .venv/bin/python experiments/gear-compliance/gear_compliance.py body   [--json body.json]

The record: docs/design-v6/2026-11-28-gear-compliance.md.

Data: hw_sessions/**/*.frec under rec.MAIN (gitignored, read only). Every 50 Hz flight tick carries,
per fitted servo, the goal written, the present position (the output-shaft encoder), speed, load
(reg 60, PWM duty in 0.1 %) and current, plus the IMU's fused roll and pitch. `extract` caches the
leg servos' ticks into CACHE (one .npz per session); the other commands read the cache.

Two questions, because the encoder sits on the output shaft:
  servo  what the encoder can see: the servo's static law between its position error (goal - pos)
         and its duty, per servo, in quasi-static holds (goal and position both still). A gear train
         that flexes BEFORE the encoder is inside the position loop and is invisible here; a pure
         P loop with a dead band explains all of goal - pos.
  body   what the encoder cannot see: flex AFTER the encoder (horn spline, horn and printed
         bracket, sole). In single support with the stance foot flat, the torso's roll and pitch
         from the IMU are compared with the ones the stance leg's encoders give through the plant's
         kinematics; the difference is regressed on the stance leg's joint torques (from duty).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "experiments", "policy", "rca"))

CACHE = os.environ.get("GEAR_COMPLIANCE_CACHE", "/tmp/gc/cache")
NLEG = 12
STEP_RAD = 2.0 * math.pi / 4096.0
STEP_DEG = 360.0 / 4096.0

JOINT_DT = np.dtype([("bus", "u1"), ("flags", "u1"), ("status", "u1"), ("temp", "u1"),
                     ("vdv", "u1"), ("moving", "u1"), ("nominal", "<i2"), ("goal", "<i2"),
                     ("goal_speed", "<u2"), ("pos", "<i2"), ("speed", "<i2"), ("load", "<i2"),
                     ("current", "<i2")])
assert JOINT_DT.itemsize == 20


def hw_dir():
    import rec
    return os.path.join(rec.MAIN, "hw_sessions")


# ----------------------------------------------------------------------------- extract
def extract_one(path):
    import flight_reader as R
    import flightrec as F
    out = os.path.join(CACHE, os.path.basename(path).replace(".frec", ".npz"))
    if os.path.exists(out):
        return out
    H, TF = F.HEADER, F.TICK_FIXED
    hdr_len, fix_len = F.HEADER_LEN, F.TICK_FIXED_LEN
    need = hdr_len + fix_len + NLEG * F.JOINT_LEN
    tick_rows, joint_rows, cal = [], [], {}
    with open(path, "rb") as f:                      # streamed: the biggest session is 5 GB
        if f.read(len(R.FILE_MAGIC)) != R.FILE_MAGIC:
            raise ValueError(path)
        f.readline()
        while True:
            rh = f.read(R.REC.size)
            if len(rh) < R.REC.size:
                break
            n, rx_us = R.REC.unpack(rh)
            data = f.read(n)
            if len(data) < n:
                break
            if n < hdr_len or data[:2] != F.MAGIC:
                continue
            ptype = data[3]
            if ptype == F.T_HELLO:
                try:
                    h = F.decode(data)
                except F.DecodeError:
                    continue
                cal[h["boot_id"]] = (list(h["zero"][:NLEG]), list(h["dir"][:NLEG]))
                continue
            if ptype != F.T_TICK or n < need:
                continue
            _, _, _, boot, _, t_utc, t_boot, _, _ = H.unpack_from(data, 0)
            v = TF.unpack_from(data, hdr_len)
            nj = v[3]
            if n != hdr_len + fix_len + nj * F.JOINT_LEN:
                continue
            js = np.frombuffer(data, JOINT_DT, count=nj, offset=hdr_len + fix_len)
            idx = js["bus"]
            if nj < NLEG or not np.array_equal(idx[:NLEG], np.arange(NLEG)):
                sel = np.full(NLEG, -1)
                for k, b in enumerate(idx):
                    if b < NLEG:
                        sel[b] = k
                if (sel < 0).any():
                    continue
                js = js[sel]
            else:
                js = js[:NLEG]
            if not (js["flags"] & F.JF_FB_OK).all():
                continue
            # source, flags, boot, t_utc, t_boot, rx, roll, pitch, gyro xyz, up xyz, bal state, bal side
            tick_rows.append((v[0], v[1], boot, t_utc, t_boot, rx_us, v[22], v[23], v[12], v[13],
                              v[14], v[19], v[20], v[21], v[24], v[25]))
            joint_rows.append(js.copy())
    os.makedirs(CACHE, exist_ok=True)
    if tick_rows:
        T = np.array(tick_rows, dtype=np.float64)
        J = np.stack(joint_rows)
    else:
        T = np.zeros((0, 16))
        J = np.zeros((0, NLEG), JOINT_DT)
    cb = np.array(sorted(cal), dtype=np.int64)
    np.savez_compressed(out, T=T, flags=J["flags"], goal=J["goal"], nominal=J["nominal"],
                        pos=J["pos"], speed=J["speed"], load=J["load"], current=J["current"],
                        vdv=J["vdv"], moving=J["moving"], cal_boot=cb,
                        cal_zero=np.array([cal[b][0] for b in cb], dtype=np.int64).reshape(-1, NLEG),
                        cal_dir=np.array([cal[b][1] for b in cb], dtype=np.int64).reshape(-1, NLEG),
                        src=path)
    return out


def cmd_extract(a):
    from multiprocessing import Pool
    fs = sorted(glob.glob(os.path.join(hw_dir(), "**", "*.frec"), recursive=True))
    fs.sort(key=os.path.getsize, reverse=True)        # the 5 GB session first
    with Pool(a.workers) as p:
        for o in p.imap_unordered(extract_one, fs):
            print(o, flush=True)




# ----------------------------------------------------------------------------- load
LEGS = ("R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
        "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
        "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll")      # bus order 0-11
JF_GOAL_VALID = 4
JF_DRIVEN = 16
# The tick's torque bit (TF_TORQUE) is not set on the bench's pose stream (the 11-13..11-16 track
# sessions never set it), so "driven" is per joint: a goal written by the stream or the policy.


def load_session(f):
    """One cached session -> dict of per-tick arrays; angles in the sim's joint frame (rad), ticks
    with an unknown calibration dropped. Times sorted; duplicates (same boot and boot clock) dropped."""
    z = np.load(f, allow_pickle=True)
    T = z["T"]
    if len(T) == 0:
        return None
    boot = T[:, 2].astype(np.int64)
    cb = z["cal_boot"]
    if len(cb) == 0:
        return None
    k = np.searchsorted(cb, boot)
    k = np.clip(k, 0, len(cb) - 1)
    ok = cb[k] == boot
    if not ok.any():
        return None
    zero, dr = z["cal_zero"][k], z["cal_dir"][k]
    t_utc = np.where(T[:, 3] > 0, T[:, 3], T[:, 5]) / 1e6
    d = dict(src=str(z["src"]), source=T[:, 0].astype(int), tflags=T[:, 1].astype(int), boot=boot,
             t=t_utc, t_boot=T[:, 4] / 1e6, roll=T[:, 6], pitch=T[:, 7], gyro=T[:, 8:11],
             up=T[:, 11:14], bal_state=T[:, 14].astype(int), bal_side=T[:, 15].astype(int),
             jflags=z["flags"], dir=dr, goal_raw=z["goal"].astype(np.int64),
             pos_raw=z["pos"].astype(np.int64), speed_raw=z["speed"].astype(np.int64),
             load_raw=z["load"].astype(np.int64), vdv=z["vdv"].astype(float))
    d["pos"] = (d["pos_raw"] - zero) * STEP_RAD * dr
    d["goal"] = (d["goal_raw"] - zero) * STEP_RAD * dr
    # the servo's PWM duty in the joint frame. Reg 60's sign is opposite to the speed's (supply_r),
    # and the speed register is in the raw step frame, so the duty pushing toward +joint is
    # -load * dir / 1000
    d["duty"] = -d["load_raw"] * dr / 1000.0
    _, first = np.unique(np.column_stack([boot, T[:, 4]]), axis=0, return_index=True)
    keep = np.zeros(len(T), bool)
    keep[first] = True
    keep &= ok
    o = np.argsort(t_utc[keep], kind="stable")
    for kk, v in list(d.items()):
        if isinstance(v, np.ndarray) and len(v) == len(T):
            d[kk] = v[keep][o]
    return d


def sessions():
    for f in sorted(glob.glob(os.path.join(CACHE, "*.npz"))):
        d = load_session(f)
        if d is not None and len(d["t"]) > 0:
            yield os.path.basename(f)[:-4], d


# ----------------------------------------------------------------------------- servo: the static law
HOLD_W = 25        # ticks (0.5 s at 50 Hz): goal fixed and position still over this window


def _win_range(x, w):
    from numpy.lib.stride_tricks import sliding_window_view as sw
    n = len(x)
    out = np.full(x.shape, np.iinfo(np.int64).max, dtype=np.int64)
    if n < w:
        return out
    v = sw(x, w, axis=0)
    r = v.max(-1) - v.min(-1)
    h = w // 2
    out[h:h + len(r)] = r
    return out


def holds(d, w=HOLD_W):
    """Per tick and servo: True where the servo holds still under a fixed goal (the servo's static
    state): torque on, goal valid, the goal unchanged and the position within 1 step over a centred
    window of w ticks with no time gap, and the speed register 0."""
    n = len(d["t"])
    gap = np.r_[0, np.diff(d["t"])] > 0.06
    seg = np.cumsum(gap | (np.r_[0, np.diff(d["boot"])] != 0))
    segr = _win_range(seg, w) == 0
    gv = (d["jflags"] & (JF_GOAL_VALID | JF_DRIVEN)) == (JF_GOAL_VALID | JF_DRIVEN)
    out = np.zeros((n, NLEG), bool)
    for j in range(NLEG):
        out[:, j] = (gv[:, j] & segr & (_win_range(d["goal_raw"][:, j], w) == 0)
                     & (_win_range(d["pos_raw"][:, j], w) <= 1) & (d["speed_raw"][:, j] == 0))
    return out


def collect_holds():
    E, D, V, J, S = [], [], [], [], []
    for stem, d in sessions():
        h = holds(d)
        for j in range(NLEG):
            k = h[:, j]
            if not k.any():
                continue
            E.append(np.rint((d["goal"][k, j] - d["pos"][k, j]) / STEP_RAD).astype(int))
            D.append(d["duty"][k, j])
            V.append(d["vdv"][k, j] / 10.0)
            J.append(np.full(k.sum(), j))
            S.append(np.full(k.sum(), stem))
    return np.concatenate(E), np.concatenate(D), np.concatenate(V), np.concatenate(J), np.concatenate(S)


# ----------------------------------------------------------------------------- body: beyond the encoders
def plant_path():
    """The legs-only plant as built: this checkout's build/, else the main checkout's (read only)."""
    import paths
    p = paths.v6_plant("bimo_biped_v6legs.xml")
    if os.path.exists(p):
        return p
    import rec
    return os.path.join(rec.MAIN, "build", "v6", "sim", "bimo_biped_v6legs.xml")


class Kin:
    """The plant's kinematics with the encoders' joint angles: per tick, the world's up in the IMU
    frame as either foot standing flat would put it, and the feet's heights below the IMU along the
    IMU's own up."""

    def __init__(self):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(plant_path())
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in LEGS])
        self.imu = self.m.site("imu").id
        self.foot = {s: self.m.body(f"{s}_foot").id for s in "LR"}
        self.d.qpos[:] = self.m.qpos0
        self.d.qpos[3:7] = (1, 0, 0, 0)
        mujoco.mj_kinematics(self.m, self.d)
        self.f0 = {s: self.d.xmat[self.foot[s]].reshape(3, 3).T @ np.array([0.0, 0.0, 1.0]) for s in "LR"}

    def run(self, q):
        """q (n, 12) joint angles in bus order -> up_fk {L, R} (n, 3) in the IMU frame, and the feet's
        positions (n, 3) in the IMU frame."""
        n = len(q)
        up = {s: np.zeros((n, 3)) for s in "LR"}
        pf = {s: np.zeros((n, 3)) for s in "LR"}
        m, d = self.m, self.d
        for i in range(n):
            d.qpos[self.qadr] = q[i]
            self.mj.mj_kinematics(m, d)
            Ri = d.site_xmat[self.imu].reshape(3, 3)
            xi = d.site_xpos[self.imu]
            for s in "LR":
                Rf = d.xmat[self.foot[s]].reshape(3, 3)
                up[s][i] = Ri.T @ (Rf @ self.f0[s])
                pf[s][i] = Ri.T @ (d.xpos[self.foot[s]] - xi)
        return up, pf


def tilt(u):
    """roll, pitch (rad) of a gravity-up vector in a body frame: the body's tilt as the IMU's
    up vector reads it (roll about x, pitch about y)."""
    return np.arctan2(u[:, 1], u[:, 2]), np.arctan2(-u[:, 0], np.hypot(u[:, 1], u[:, 2]))


def up_body(roll, pitch):
    """The world's up in the IMU frame from the tick's roll and pitch (ZYX from the fused quaternion,
    heading-free). Not the tick's `up`: that is the obs vector, the body's z axis in the (heading-
    drifting) world frame."""
    return np.column_stack([-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)])


def stall_nm(j, V):
    import current_budget as CB
    import sts_servo_model as SM
    import walker_env as W
    return SM.STALL_NM_12V * W.SERVO_TYPES[CB.servo_type(LEGS[j])][0] * V / 12.0


GRAV_JOINTS = ("hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")


class Statics(Kin):
    """Kin plus the gravity torque each joint of a STANCE leg carries in single support, quasi-static:
    the moment about the joint's axis of the weight of everything on the torso's side of it (the
    torso, the other leg, the stance leg above the joint), with gravity along the IMU's measured up.
    In the plant's joint sign: the torque the servo must apply to hold the body."""

    def __init__(self):
        super().__init__()
        m = self.m
        self.par = m.body_parentid.copy()
        self.mass = m.body_mass.copy()
        self.jid = {s: [m.joint(f"{s}_{n}").id for n in GRAV_JOINTS] for s in "LR"}
        self.upper = {}
        for s in "LR":
            for ji in self.jid[s]:
                b = m.jnt_bodyid[ji]
                sub = {b}
                for i in range(1, m.nbody):              # bodies are in tree order
                    if self.par[i] in sub:
                        sub.add(i)
                self.upper[ji] = np.array([i for i in range(1, m.nbody) if i not in sub])

    def run(self, q, up_imu):
        n = len(q)
        up = {s: np.zeros((n, 3)) for s in "LR"}
        pf = {s: np.zeros((n, 3)) for s in "LR"}
        tg = {s: np.zeros((n, len(GRAV_JOINTS))) for s in "LR"}
        m, d = self.m, self.d
        for i in range(n):
            d.qpos[self.qadr] = q[i]
            self.mj.mj_kinematics(m, d)
            self.mj.mj_comPos(m, d)
            Ri = d.site_xmat[self.imu].reshape(3, 3)
            xi = d.site_xpos[self.imu]
            g = -9.81 * (Ri @ up_imu[i])                 # gravity in the torso-fixed frame
            for s in "LR":
                Rf = d.xmat[self.foot[s]].reshape(3, 3)
                up[s][i] = Ri.T @ (Rf @ self.f0[s])
                pf[s][i] = Ri.T @ (d.xpos[self.foot[s]] - xi)
                for k, ji in enumerate(self.jid[s]):
                    u = self.upper[ji]
                    r = d.xipos[u] - d.xanchor[ji]
                    M = np.cross(r, self.mass[u, None] * g).sum(0)
                    tg[s][i, k] = M @ d.xaxis[ji]
        return up, pf, tg


QS_GYRO = 0.15      # rad/s: quasi-static body
QS_W = 15           # ticks: the gyro and the stance joints still over this window


def body_one(f):
    d = load_session(f)
    if d is None or len(d["t"]) < 100:
        return None
    from numpy.lib.stride_tricks import sliding_window_view as sw
    gn = np.linalg.norm(d["gyro"], axis=1)
    n = len(gn)
    gmax = np.full(n, np.inf)
    if n >= QS_W:
        gmax[QS_W // 2: QS_W // 2 + n - QS_W + 1] = sw(gn, QS_W).max(-1)
    on = ((d["jflags"] & (JF_GOAL_VALID | JF_DRIVEN)) == (JF_GOAL_VALID | JF_DRIVEN)).all(1)
    g = up_body(d["roll"], d["pitch"])
    k = np.where(on & (gmax < QS_GYRO) & (g[:, 2] > 0.8))[0]
    if len(k) == 0:
        return None
    S = Statics()
    up_imu = g[k]
    up, pf, tg = S.run(d["pos"][k], up_imu)
    stall = np.stack([stall_nm(j, d["vdv"][k, j] / 10.0) for j in range(NLEG)], 1)
    spmax = np.zeros((len(k), NLEG), np.int64)
    a = np.abs(d["speed_raw"])
    for j in range(NLEG):
        r = np.zeros(n, np.int64)
        if n >= QS_W:
            r[QS_W // 2: QS_W // 2 + n - QS_W + 1] = sw(a[:, j], QS_W).max(-1)
        spmax[:, j] = r[k]
    return dict(stem=os.path.basename(f)[:-4], t=d["t"][k], boot=d["boot"][k], source=d["source"][k],
                up_imu=up_imu, upL=up["L"], upR=up["R"], pfL=pf["L"], pfR=pf["R"], tgL=tg["L"],
                tgR=tg["R"], duty=d["duty"][k], tau=d["duty"][k] * stall, pos=d["pos"][k],
                goal=d["goal"][k], spmax=spmax, bal_state=d["bal_state"][k], bal_side=d["bal_side"][k])


def cmd_bodyextract(a):
    from multiprocessing import Pool
    fs = sorted(glob.glob(os.path.join(CACHE, "*.npz")))
    fs.sort(key=os.path.getsize, reverse=True)
    out = []
    with Pool(a.workers) as p:
        for r in p.imap_unordered(body_one, fs):
            if r is not None:
                out.append(r)
                print(r["stem"], len(r["t"]), flush=True)
    keys = [k for k in out[0] if k != "stem"]
    cat = {k: np.concatenate([r[k] for r in out]) for k in keys}
    cat["stem"] = np.concatenate([np.full(len(r["t"]), r["stem"]) for r in out])
    np.savez(a.out, **cat)



# ----------------------------------------------------------------------------- servo command
STS3250_BUS = (0, 4)        # the hip rolls


def servo_law(e, j):
    """The P law the holds show (duty, joint frame) for a position error of e steps: STS3215
    0.8 % x (|e| + 1) past a +-1-step dead band, STS3250 0.8 % x e with none (P 32 everywhere)."""
    e = np.asarray(e)
    if j in STS3250_BUS:
        return 0.008 * e
    return np.where(np.abs(e) >= 2, 0.008 * (np.abs(e) + 1) * np.sign(e), 0.0)


def cmd_servo(a):
    E, D, V, J, S = collect_holds()
    res = {}
    for j in range(NLEG):
        k = J == j
        r = D[k] - servo_law(E[k], j)
        tab = {}
        for e in range(-15, 16):
            m = k & (E == e)
            if m.sum() >= 20:
                tab[e] = dict(n=int(m.sum()), duty_median=float(np.median(D[m])),
                              duty_p5=float(np.percentile(D[m], 5)), duty_p95=float(np.percentile(D[m], 95)))
        res[LEGS[j]] = dict(n=int(k.sum()), sessions=int(len(np.unique(S[k]))),
                            exact=float(np.mean(np.abs(r) < 1e-9)), table=tab)
        print(f"{LEGS[j]:13s} holds {k.sum():7d} in {len(np.unique(S[k])):2d} sessions; duty = the P law "
              f"exactly in {np.mean(np.abs(r) < 1e-9):.2%}; " +
              " ".join(f"{e:+d}:{v['duty_median'] * 100:.1f}" for e, v in tab.items() if abs(e) <= 4))
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


# ----------------------------------------------------------------------------- body command
SINGLE_MM = 12.0     # one foot this much lower than the other (along the IMU's up): single support
DOUBLE_MM = 3.0      # both feet within this: double support
MIN_SINGLE, MIN_DOUBLE = 80, 200
H_MIN = 0.7          # a session's stance hip-roll gravity torque below this (N m): an odd posture, out


def body_sessions(z):
    """Per stance side: one row per session with both single and double support, the single-support
    roll beyond the encoders (IMU minus the stance leg's foot-flat kinematics, less the session's
    double-support median; + = toward the lifted foot), the pitch the same way, and the stance hip
    roll's and ankle roll's gravity torques (+ = toward the lifted foot), medians."""
    u = z["up_imu"]
    h = lambda p: -(p * u).sum(1)
    dh = (h(z["pfL"]) - h(z["pfR"])) * 1000.0
    ri, pi = tilt(u)
    st = z["stem"]
    bad = np.array(["hang" in s or "oneleg" in s for s in st])     # feet not on a floor
    dbl = (np.abs(dh) < DOUBLE_MM) & ~bad
    out = {}
    for s, sel in (("L", dh > SINGLE_MM), ("R", dh < -SINGLE_MM)):
        sel = sel & ~bad
        rf, pf = tilt(z["up" + s])
        sg = 1.0 if s == "L" else -1.0          # roll + = right side down; the lifted foot's side
        rows = []
        for s0 in np.unique(st[sel]):
            m, mb = sel & (st == s0), dbl & (st == s0)
            if m.sum() < MIN_SINGLE or mb.sum() < MIN_DOUBLE:
                continue
            r = np.rad2deg(ri - rf)
            p = np.rad2deg(pi - pf)
            y = sg * (r[m] - np.median(r[mb]))
            rows.append(dict(session=str(s0), n=int(m.sum()), n_double=int(mb.sum()),
                             sag_deg=float(np.median(y)), sag_iqr=[float(x) for x in np.percentile(y, [25, 75])],
                             pitch_deg=float(np.median(p[m] - np.median(p[mb]))),
                             tau_hip=float(np.median(sg * z["tg" + s][m, 0])),
                             tau_ankle=float(np.median(sg * z["tg" + s][m, 4])),
                             tau_knee=float(np.median(z["tg" + s][m, 2]))))
        out[s] = rows
    return out


def spring_fit(rows, nboot=1000, seed=0):
    """sag = tau_hip / k_hip + tau_ankle / k_ankle over sessions (no intercept: a spring from the
    double-support baseline), weighted by min(n, 2000); a session bootstrap for the CIs. And the
    constant-sag (play) model's rms, for comparison."""
    rows = [r for r in rows if r["tau_hip"] > H_MIN]
    A = np.array([[r["tau_hip"], r["tau_ankle"]] for r in rows])
    y = np.deg2rad([r["sag_deg"] for r in rows])
    w = np.minimum([r["n"] for r in rows], 2000).astype(float)

    def fit(ii):
        return np.linalg.lstsq(A[ii] * np.sqrt(w[ii, None]), y[ii] * np.sqrt(w[ii]), rcond=None)[0]
    ii = np.arange(len(rows))
    b = fit(ii)
    rng = np.random.default_rng(seed)
    bs = np.array([fit(rng.choice(ii, len(ii))) for _ in range(nboot)])
    lo, hi = np.percentile(bs, [2.5, 97.5], axis=0)
    rms = lambda r: float(np.rad2deg(np.sqrt(np.average(r ** 2, weights=w))))
    return dict(sessions=len(rows), k_hip=float(1 / b[0]), k_hip_ci=[float(1 / hi[0]), float(1 / lo[0])],
                k_ankle=float(1 / b[1]), k_ankle_ci=[float(1 / hi[1]), float(1 / lo[1])],
                rms_spring_deg=rms(y - A @ b), rms_constant_deg=rms(y - np.average(y, weights=w)),
                sag_median_deg=float(np.median(np.rad2deg(y))),
                tau_hip_median=float(np.median(A[:, 0])))


def cmd_body(a):
    z = dict(np.load(a.src))
    S = body_sessions(z)
    res = {}
    for s in "LR":
        print(f"{s} stance (the {'right' if s == 'L' else 'left'} foot lifted): per session")
        for r in S[s]:
            print(f"  {r['session'][7:45]:38s} n {r['n']:5d}  sag {r['sag_deg']:+5.2f} deg "
                  f"[{r['sag_iqr'][0]:+5.2f}, {r['sag_iqr'][1]:+5.2f}]  pitch {r['pitch_deg']:+5.2f}  "
                  f"tau hip {r['tau_hip']:.2f} ankle {r['tau_ankle']:+.2f} knee {r['tau_knee']:+.2f} N m")
        f = spring_fit(S[s])
        print(f"  spring fit over {f['sessions']} sessions: k_hip {f['k_hip']:.1f} [{f['k_hip_ci'][0]:.1f}, "
              f"{f['k_hip_ci'][1]:.1f}], k_ankle {f['k_ankle']:.1f} [{f['k_ankle_ci'][0]:.1f}, "
              f"{f['k_ankle_ci'][1]:.1f}] N m/rad; rms {f['rms_spring_deg']:.2f} deg "
              f"(a constant sag: {f['rms_constant_deg']:.2f})")
        res[s] = dict(sessions=S[s], spring_fit=f)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    e = sp.add_parser("extract")
    e.add_argument("--workers", type=int, default=6)
    b = sp.add_parser("bodyextract")
    b.add_argument("--workers", type=int, default=6)
    b.add_argument("--out", default=os.path.join(os.path.dirname(CACHE), "body.npz"))
    v = sp.add_parser("servo")
    v.add_argument("--json", default=None)
    o = sp.add_parser("body")
    o.add_argument("--src", default=os.path.join(os.path.dirname(CACHE), "body.npz"))
    o.add_argument("--json", default=None)
    a = ap.parse_args()
    {"extract": cmd_extract, "bodyextract": cmd_bodyextract, "servo": cmd_servo, "body": cmd_body}[a.cmd](a)


if __name__ == "__main__":
    main()
