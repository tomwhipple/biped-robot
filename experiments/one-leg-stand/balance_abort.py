"""Abort policy for the stance-leg roll balance, chosen in sim.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_abort.py table [--json out.json]
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_abort.py report --json out.json
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_abort.py golden   # rows pinned by firmware/host/test_roll_pi.cpp
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_abort.py scan     # kd x lag x gyro delay: where does the loop go unstable?

The controller (balance.py) engages at the start of the lift and, once |roll err| passes
the abort limit (12 deg) or the IMU fails, stops regulating. What the robot does THEN is the
policy compared here:

    HOLD          today: the correction freezes, the lifted foot stays up
    DECAY(T)      u -> 0 along a min-jerk over T; the nominal pose stays at the lift pose
    LAND(T)       the nominal streams back to the pose the B move started from (the shifted,
                  double-support pose) over T, min-jerk; u held at its frozen value
    LAND_DECAY(T) the same landing with u decaying to 0 over the same T
    LAND_FAST(T)  the landing over T with u decaying over 0.3 s
    SEQ(T)        u decays over 0.3 s first, then the landing over T

Triggers (stance leg L or R, 30 mm lift, hold 6 s, as-built plant, calibrated roll play):
  outward  a roll-rate impulse toward the STANCE side (the side nothing catches) hard enough to
           pass the 12 deg abort, or a slow CoM drift
  inward   the same toward the LIFTED foot
  gain     gains that destabilise the loop (kd 0.09+)
  fault    the IMU stops answering mid-hold (non-finite rate / stale attitude), with a small
           disturbance 0.5 s later -- no attitude, no way to regulate it
Torque is never released in any policy.
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import balance_sweep as BS  # noqa: E402
import one_leg as OL  # noqa: E402
import static_gait as SG  # noqa: E402
from balance import Balance  # noqa: E402
from design_gates import JN, minjerk  # noqa: E402

def abort_profile(since, decay=0.3, land=1.0):
    """The chosen policy (SEQ): (u_scale, land_frac) at `since` seconds after the abort. The C++ twin is
    balance::abortProfile (firmware/components/balance); `balance_abort.py golden` prints its test rows."""
    return 1.0 - minjerk(min(1.0, max(0.0, since / decay))), minjerk(min(1.0, max(0.0, (since - decay) / land)))


class AbortStudy(Balance):
    """Balance with an abort policy, a push and an IMU fault injected."""

    T_FAST = 0.3        # land_fast / seq: u decays over this, whatever T is

    def __init__(self, *a, policy="hold", T=0.6, q_start=None, push=None, fault_t=None, **kw):
        super().__init__(*a, **kw)
        self.policy, self.T, self.q_start = policy, T, q_start
        self.push = push                     # (t0, dur, force_N, +1/-1 world-y sign)
        self.fault_t = fault_t
        self.t_abort = None
        self.q_nom_abort = None
        self.u_abort = 0.0
        self.cause = None
        self.err_abort = 0.0
        self.post = []                       # (t, roll_err, swing pad height, tilt, stance slip)
        self.stance_xy0 = None
        self.pushed = False
        self.ipos0 = None

    def __call__(self, t, env, qc):
        import mujoco
        dt = env.control_dt
        m, d = env.model, env.data
        if self.pads is None:
            self.pads = [g for g in range(m.ngeom)
                         if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
                             ("R_pad_" if self.s == "L" else "L_pad_"))]
            self.stance_body = m.body(f"{self.s}_foot").id
        if self.push and self.push[1] == "m" and t >= self.push[0]:
            # a slow load: the torso CoM drifts laterally by push[2] metres over 2 s (a payload
            # leaning on it); sign: world y = -roll sign
            if self.ipos0 is None:
                self.ipos0 = float(m.body_ipos[env._torso_bid][1])
            m.body_ipos[env._torso_bid][1] = self.ipos0 - self.push[3] * self.push[2] * min(1.0, (t - self.push[0]) / 2.0)
        elif self.push and not self.pushed and t >= self.push[0]:
            # a shove: an impulsive roll rate (rad/s about the torso x axis, + = toward the
            # robot's right) -- env.step clears xfrc_applied, a velocity kick survives it
            if self.push[1] == "w":
                d.qvel[3] += self.push[3] * self.push[2]
            else:                                   # lateral velocity of the pelvis, m/s (world y = -roll sign)
                d.qvel[1] -= self.push[3] * self.push[2]
            self.pushed = True
        roll = self.roll_of(env)
        self.hist.append(roll)
        meas = self.hist[max(0, len(self.hist) - 1 - self.delay)]
        if self.noise:
            meas += float(self.rng.normal(0.0, self.noise))
        self.ghist.append(self.rate_of(env))
        rate = self.ghist[max(0, len(self.ghist) - 1 - self.gyro_delay)] + self.gyro_bias
        if self.gyro_noise:
            rate += float(self.rng.normal(0.0, self.gyro_noise))
        h_sw = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.pads)
        if self.h0 <= t < self.h1 and h_sw > 0.005:
            self.air += dt
        # the nominal is the B stream: it ends at the lift pose and stays there
        qn = qc
        if t < self.t_on:
            self.rec.append((t, roll, 0.0))
            return qc
        if self.ref is None:
            self.ref = meas
            self.stance_xy0 = d.xpos[self.stance_body][:2].copy()
        err = meas - self.ref
        if self.t_abort is None:
            faulted = self.fault_t is not None and t >= self.fault_t
            if faulted:
                self.cause = "imu"
            elif abs(err) > self.abort:
                self.cause = "tip"
            if self.cause:
                self.t_abort, self.q_nom_abort = t, qn.copy()
                self.err_abort = err
                self.u_abort = self.pi.u
        if self.t_abort is None:
            u = self.pi.step(err, dt, rate)
            q = qn
        else:
            tau = t - self.t_abort
            fade = 1.0 - minjerk(min(1.0, tau / self.T))
            if self.policy == "hold":
                u, q = self.u_abort, qn
            elif self.policy == "decay":
                u, q = self.u_abort * fade, qn
            else:
                t_land = self.T_FAST if self.policy == "seq" else 0.0       # seq: land after u is gone
                s = minjerk(min(1.0, max(0.0, tau - t_land) / self.T))
                q = self.q_nom_abort + (self.q_start - self.q_nom_abort) * s
                fast = self.u_abort * (1.0 - minjerk(min(1.0, tau / self.T_FAST)))
                u = {"land": self.u_abort, "land_decay": self.u_abort * fade}.get(self.policy, fast)
            slip = float(np.linalg.norm(d.xpos[self.stance_body][:2] - self.stance_xy0))
            up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
            self.post.append((t, err, h_sw, math.degrees(math.acos(max(-1.0, min(1.0, up)))), slip))
        self.rec.append((t, roll, u))
        q = q.copy()
        i_a, i_h = JN.index(f"{self.s}_ankle_roll"), JN.index(f"{self.s}_hip_roll")
        q[i_a] += self.ka * u
        q[i_h] += self.kh * u
        return q


def run_case(args):
    cfg, ctl, pol, T, trig = args
    c = {**BS.BASE, **cfg, "hold": 6.0}
    stance = c["stance"]
    tl, win, _ = BS._keys(OL.AS_BUILT, c["hold"], c["bias"], stance, c["t_lift"])
    p, xml = BS._plant(OL.AS_BUILT)
    seg = lambda n: tl.durs[tl.names.index(n) - 1]
    t_l1 = tl.t_of("lift")
    t_l0 = t_l1 - seg("lift")
    t_h1 = tl.t_of("hold")
    q_shift = tl.q[tl.names.index("lift") - 1]           # keyframe the lift move starts from
    out_sign = -1.0 if stance == "L" else 1.0            # roll + = toward the right = OUTWARD for R stance
    push = None
    fault_t = None
    if trig[0] == "outward":
        push = (t_l1 + trig[1], trig[3], trig[2], out_sign)
    elif trig[0] == "inward":
        push = (t_l1 + trig[1], trig[3], trig[2], -out_sign)
    elif trig[0] in ("fault", "faulto"):
        fault_t = t_l1 + trig[1]
        # a small bump 0.5 s after the IMU dies: toward the lifted foot ("fault") or the stance side ("faulto")
        push = (fault_t + 0.5, trig[3], trig[2], -out_sign if trig[0] == "fault" else out_sign)
    # trig[0] == "gain": no push, the gains destabilise the loop (ctl carries them)
    hook = AbortStudy(stance, ctl["kp"], ctl["ki"], ctl["ka"], ctl["kh"], math.radians(ctl.get("clamp", 10.0)),
                      t_on=t_l0, t_off=t_h1 + 99.0, t_fade=1.0, t_hold0=t_l1, t_hold1=t_h1,
                      kd=ctl.get("kd", 0.0), gyro_delay=ctl.get("gdelay", 1), delay_ticks=c["imu_delay"], noise=math.radians(0.1), seed=c["seed"],
                      policy=pol, T=T, q_start=q_shift, push=push, fault_t=fault_t)
    # the nominal after the B stream ends is the lift pose, not the timeline's "lower"
    class Nom:
        T = tl.T
        def at(self, t):
            return tl.at(min(t, t_l1))
    hold_end = t_h1
    orig = SG.q_of
    SG.q_of = lambda p_, key: key if isinstance(key, np.ndarray) else orig(p_, key)
    try:
        # run only until a few seconds after the hold would end
        r = SG.run_walk(p, xml, _Trunc(Nom(), hold_end + 1.0), win, seed=c["seed"], mu=c["mu"], play_deg=c["play"],
                        backlash_deg=c["lash"], lag_hz=c["lag"], delay_ticks=c["delay"], per_joint=BS.PJ,
                        hook=hook)
    finally:
        SG.q_of = orig
    res = dict(stance=stance, trig=list(trig), policy=pol, T=T, fell=bool(r["fell"]), t_fell=r["t_fell"],
               aborted=hook.t_abort is not None, cause=hook.cause, plant=cfg,
               outward=bool(hook.err_abort * out_sign > 0), err_abort_deg=math.degrees(hook.err_abort),
               t_abort=None if hook.t_abort is None else round(hook.t_abort - t_l1, 2))
    if hook.post:
        a = np.array(hook.post)
        res.update(err_peak_deg=math.degrees(float(np.abs(a[:, 1]).max())), tilt_peak_deg=float(a[:, 3].max()),
                   slip_mm=1e3 * float(a[:, 4].max()))
        # foot down = swing pad within 5 mm of the floor, and it stays so to the end
        up_ = np.where(a[:, 2] > 0.005)[0]
        res["down_s"] = float(a[min(len(a) - 1, up_[-1] + 1), 0] - hook.t_abort) if len(up_) and up_[-1] + 1 < len(a) \
            else (float("inf") if len(up_) else 0.0)
        last = a[a[:, 0] >= a[-1, 0] - 1.0]
        res["end_tilt_deg"] = float(last[:, 3].mean())
        res["end_foot_mm"] = 1e3 * float(last[:, 2].max())
    else:
        res.update(err_peak_deg=float("nan"), tilt_peak_deg=float("nan"), slip_mm=float("nan"),
                   down_s=float("nan"), end_tilt_deg=float("nan"), end_foot_mm=float("nan"))
    return res


class _Trunc:
    """the timeline, shortened: run_walk loops over tl.T"""
    def __init__(self, tl, T):
        self._tl, self.T = tl, T
        self.final_heading = 0.0

    def at(self, t):
        return self._tl.at(t)


PLANTS = {"nominal": {}, "play5": dict(play=5.0, lash=2.0), "light": dict(torso=0.6), "heavy_lag": dict(torso=1.4, lag=2.0, delay=4)}
GAINS = dict(kp=0.3, ki=1.0, ka=1.0, kh=-1.0, clamp=10.0)
VARIANTS = [("pi_off", None), ("hold", None)] + [(pol, T) for pol in ("decay", "land", "land_decay", "land_fast", "seq") for T in (0.4, 0.7, 1.0)]


def triggers():
    """(class, label, ctl-kd, trig tuple). push mag: w in rad/s of roll rate, v in m/s of pelvis velocity"""
    out = []
    for w in (5.5, 6.0):
        out.append(("outward", f"push w{w:g}", 0.03, ("outward", 1.0, w, "w")))
    for mm in (30, 45, 60):
        out.append(("outward", f"CoM drift {mm}mm", 0.03, ("outward", 1.0, mm / 1000.0, "m")))
    for w in (6.0, 6.5):
        out.append(("inward", f"push w{w:g}", 0.03, ("inward", 1.0, w, "w")))
    for mm in (30, 45, 60):
        out.append(("inward", f"CoM drift {mm}mm", 0.03, ("inward", 1.0, mm / 1000.0, "m")))
    for kd in (0.09, 0.12):
        out.append(("gain", f"kd {kd:g}", kd, ("gain", 1.0, 0.0, "w")))
    for kind in ("inward", "outward"):
        for w in (4.0, 5.0, 6.0):
            out.append(("imu", f"fault+{kind} bump w{w:g}", 0.03, ("fault", 1.0, w, "w") if kind == "inward" else ("faulto", 1.0, w, "w")))
    return out


def jobs_all():
    jobs = []
    for pn, pc in PLANTS.items():
        for st in ("L", "R"):
            for cls, lab, kd, trig in triggers():
                for pol, T in VARIANTS:
                    if pol == "pi_off":
                        continue
                    jobs.append(({**pc, "stance": st}, {**GAINS, "kd": kd}, pol, T or 0.6, trig, pn, cls, lab))
    return jobs


def _run(j):
    r = run_case(j[:5])
    r.update(plant_name=j[5], cls=j[6], label=j[7], T=j[3] if j[2] != "hold" else None)
    return r


def table(path):
    jobs = jobs_all()
    with mp.Pool(min(32, len(jobs))) as pool:
        rs = pool.map(_run, jobs, chunksize=1)
    if path:
        json.dump(rs, open(path, "w"), indent=1, default=lambda x: None if x != x else x)
    report(rs)


def report(rs):
    import collections
    # the abort time does not depend on the policy: take the runs whose abort fired
    g = collections.defaultdict(list)
    for r in rs:
        if r["aborted"]:
            g[(r["cls"], r["policy"], r["T"])].append(r)
    classes = ("outward", "inward", "gain", "imu")
    print(f"{'policy':<12}{'T':>5} | " + " | ".join(f"{c:^34}" for c in classes) + " | all falls")
    print(f"{'':<17} | " + " | ".join(f"{'falls  err_pk  tilt_pk  down_s  slip':^34}" for _ in classes) + " |")
    for pol, T in [v for v in VARIANTS if v[0] != "pi_off"]:
        cells = []
        tot = n = 0
        for c in classes:
            L = g.get((c, pol, T), [])
            if not L:
                cells.append(f"{'-':^34}")
                continue
            f = sum(r["fell"] for r in L)
            ok = [r for r in L if not r["fell"]]
            tot += f; n += len(L)
            ep = max((r["err_peak_deg"] for r in L), default=float("nan"))
            tp = max((r["tilt_peak_deg"] for r in ok), default=float("nan"))
            dn = max((r["down_s"] for r in ok), default=float("nan"))
            sl = max((r["slip_mm"] for r in ok), default=float("nan"))
            cells.append(f"{f:>2}/{len(L):<3} {ep:6.1f} {tp:7.1f} {dn:6.2f} {sl:6.1f}".center(34))
        print(f"{pol:<12}{(T or 0):>5} | " + " | ".join(cells) + f" | {tot}/{n}")


def scan():
    jobs = []
    for st in ("L", "R"):
      for lag in (2.0, 3.0, 4.0):
        for gd in (1, 2, 3):
            for kd in (0.05, 0.07, 0.09):
                jobs.append(({"stance": st, "lag": lag}, {**GAINS, "kd": kd, "gdelay": gd}, "hold", 0.6, ("gain", 1.0, 0.0, "w"), "x", "gain", "x"))
    with mp.Pool(32) as pool:
        for j, r in zip(jobs, pool.map(_run, jobs)):
            print(j[0], j[1]["kd"], j[1]["gdelay"], "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k in ("fell", "t_abort", "err_peak_deg", "down_s", "outward")})


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("table", "report", "scan", "golden"))
    ap.add_argument("--json")
    a = ap.parse_args()
    if a.cmd == "table":
        table(a.json)
    elif a.cmd == "golden":
        for t in (0.0, 0.1, 0.3, 0.5, 1.0, 1.3, 2.0):
            us, lf = abort_profile(t)
            print(f"{t:.2f} -> u_scale {us:.6f}  land {lf:.6f}")
    elif a.cmd == "scan":
        scan()
    else:
        report(json.load(open(a.json)))

