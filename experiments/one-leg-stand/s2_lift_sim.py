"""Two-step walk (walk.py keyframes, 4 cm, lateral shift x1.3) on the as-built plant with the
stance-leg roll PI as the firmware runs it (PR #165 continuation), to find out why the
SECOND step (L stance, rear R foot comes up alongside) tips outward on the robot.
2026-10-30-s2-lift-sim.md has the results.

Sequence (the bench runner `runwalk2.sh`, timings from walk2_serial.log):
    crouch | s1_shift 3 s + 0.6 | s1_lift 4 s BR + 3.6 | s1_swing 3 s BR | s1_touch 3 s BR | s1_centre 3 s (fade) + 2.4
           | s2_shift 3 s + 0.6 | s2_lift 4 s BL + 3.6 | s2_swing 3 s BL | s2_touch 3 s BL | s2_centre 3 s (fade) + 2
Balance: engages at the start of each lift with ref = roll then, continues (I, u, ref kept) through
swing and touch, fades along the centre move.

Fix modes (--fix, one code path for both stances, no per-side constants):
    base        as on the robot (PI engaged at the lift start, ref = roll there)
    nobal       open loop (the controller computes err but outputs nothing)
    freeze      (a) I integrates only while the commanded pose is not moving (holds)
    footoff     (b) no output until --footoff-frac of the lift move (1.0 = when the lift move ends),
                then engage with ref = roll there; without --footoff-frac, when the swing knee's
                servo torque falls below half its lift-start value (not usable: see the report)
    upfirst     (c) the swing foot goes straight up first (as high as the joint limits allow, 3 s),
                then to the usual lift pose (2 s)
    centredref  (d) ref = the roll of the centred double-support pose just before the shift
    pdlift      (d) P+D only during the lift move on the pre-lift ref; at the end of the move
                re-reference ref = roll and carry u over through I (bumpless), then the PI

Model-error / plant options: --play --lash --mu --plant-set foot_w=0.060 (a narrower effective sole,
the carpet proxy that reproduces the bench's s2 trace) --torso-scale --torso-off --lat --first L
(mirrored walk) --ctl-steps s2 (s1 open loop, the clean start for s2) --log run.json (per-tick log).

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/s2_lift_sim.py --fix base --plant-set foot_w=0.060 --ctl-steps s2
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

import one_leg as O  # noqa: E402
import static_gait as SG  # noqa: E402
import v6_kin as K  # noqa: E402
from balance import RollPI  # noqa: E402
from design_gates import JN, Key, minjerk, q_of, standing_key  # noqa: E402
from gen_plant_v6 import build_xml, params_from_args  # noqa: E402

DROP, LIFT = 0.02, 0.03


def walk_keys(p, step=0.04, lat=1.3, nstep=2, upfirst=False, first="R"):
    """walk.py, verbatim logic. upfirst: the lift pose keeps the swing foot's x (straight up)."""
    m, d = K.load(p)
    k1 = standing_key(p, DROP)
    z = k1.pelvis[2]
    feet = {"L": k1.footL.copy(), "R": k1.footR.copy()}
    keys, names = [], []

    def add(n, pel, f):
        keys.append(Key(np.array([pel[0], pel[1], z]), f["L"].copy(), f["R"].copy())); names.append(n)
    add("crouch", k1.pelvis, feet)
    for i in range(nstep):
        stance, swing = (first, "L" if first == "R" else "R") if i % 2 == 0 else ("L" if first == "R" else "R", first)
        sgn = 1.0 if stance == "L" else -1.0
        centre = 0.5 * (feet["L"] + feet["R"]); centre[2] = z
        mid = feet[swing] + np.array([0, 0, LIFT])
        pel = SG.solve_pelvis(m, d, p, centre.copy(), feet, stance, mid, bias=(0.0, sgn * -0.005))
        pel[1] = centre[1] + (pel[1] - centre[1]) * lat
        add(f"s{i+1}_shift", pel, feet)
        target_x = feet[stance][0] + (step if i < nstep - 1 else 0.0)
        f = {k: v.copy() for k, v in feet.items()}; f[swing][2] += LIFT
        lo, hi = sorted((feet[swing][0], target_x))
        if upfirst:              # straight up as far as the joint limits allow, then the usual lift pose
            fu = {k: v.copy() for k, v in feet.items()}
            for h in np.arange(LIFT, 0.0, -0.001):
                fu[swing][2] = feet[swing][2] + h
                qq = q_of(p, Key(np.array([pel[0], pel[1], z]), fu["L"], fu["R"]))
                if all(m.jnt_range[m.joint(j).id][0] <= qq[k] <= m.jnt_range[m.joint(j).id][1] for k, j in enumerate(JN)):
                    break
            add(f"s{i+1}_up", pel, fu)
        f[swing][0] = min(max(pel[0] - 0.010, lo), hi)
        add(f"s{i+1}_lift", pel, f)
        f[swing][0] = target_x; add(f"s{i+1}_swing", pel, f)
        f[swing][2] -= LIFT; add(f"s{i+1}_touch", pel, f)
        feet = f
        c = 0.5 * (feet["L"] + feet["R"]); add(f"s{i+1}_centre", c, feet)
    qs = [q_of(p, k) for k in keys]
    return dict(zip(names, qs)), dict(zip(names, keys))


class Ctl:
    """The stance-leg roll PI with continuation, plus the fix modes; logs every tick."""

    def __init__(self, p, segs, fix="base", kp=0.3, ki=1.0, kd=0.03, clamp=10.0, abort=12.0,
                 footoff_frac=None, noise=0.0017, seed=0, ctl_steps=("s1", "s2")):
        self.p, self.segs, self.fix = p, segs, fix
        self.kp, self.ki, self.kd, self.clamp = kp, ki, kd, math.radians(clamp)
        self.abort = math.radians(abort)
        self.footoff_frac = footoff_frac
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.pi = None
        self.ref = None
        self.side = None
        self.fade = None
        self.hist = []
        self.ghist = []
        self.log = []
        self.tau0 = None
        self.engaged_at = None
        self.init = False
        self.ctl_steps = ctl_steps
        self.centred_roll = None

    def lift_frac(self, t):
        """commanded lift progress 0..1 over all the 'lifting' segments of the current step"""
        ls = [s for s in self.segs if s.get("lifting") and s.get("B") == self.side and s["t0"] <= t + 1e-9]
        ls = [s for s in ls if s["step"] == self.seg_at(t)["step"]]
        if not ls:
            return 0.0
        T = sum(s["t1"] - s["t0"] for s in self.segs if s.get("lifting") and s["step"] == ls[0]["step"])
        done = sum(min(t, s["t1"]) - s["t0"] for s in ls)
        return done / T

    def seg_at(self, t):
        for s in self.segs:
            if s["t0"] <= t < s["t1"]:
                return s
        return self.segs[-1]

    def _init(self, env):
        m = env.model
        self.pad = {s: set(g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(f"{s}_pad_"))
                    for s in "LR"}
        self.fb = {s: m.body(f"{s}_foot").id for s in "LR"}
        self.init = True

    def fn(self, env):
        m, d = env.model, env.data
        out = {"L": 0.0, "R": 0.0}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for s in "LR":
                if c.geom1 in self.pad[s] or c.geom2 in self.pad[s]:
                    mujoco.mj_contactForce(m, d, i, f6)
                    out[s] += abs(f6[0])
        return out

    def __call__(self, t, env, qc):
        if not self.init:
            self._init(env)
        dt = env.control_dt
        d = env.data
        roll = math.atan2(d.xmat[env._torso_bid].reshape(3, 3)[2, 1], d.xmat[env._torso_bid].reshape(3, 3)[2, 2])
        self.hist.append(roll)
        meas = self.hist[max(0, len(self.hist) - 2)] + float(self.rng.normal(0, self.noise))
        self.ghist.append(float(d.qvel[3]))
        rate = self.ghist[max(0, len(self.ghist) - 2)] + float(self.rng.normal(0, 0.0075))
        seg = self.seg_at(t)
        B = seg.get("B")
        moving = seg["moving"]
        frac = (t - seg["t0"]) / max(seg["t1"] - seg["t0"], 1e-9)
        tau = env._servo_tau.astype(np.float64)
        u = 0.0
        err = float("nan")
        if seg["name"].endswith("_shift") and t - seg["t0"] < 0.5 * dt:
            self.centred_roll = meas            # the centred double-support roll, just before the shift
        if B and seg["step"] not in self.ctl_steps:
            B = None
        if B and seg.get("engage") and self.side != B:
            # start of a B chain: engage (base / freeze) or arm (footoff)
            self.side = B
            self.pi = RollPI(self.kp, self.ki, self.clamp, self.kd)
            self.ref = None
            self.fade = None
            self.engaged_at = None
            self.rebased = False
            sw = "R" if B == "L" else "L"
            self.tau0 = abs(tau[JN.index(f"{sw}_knee")])
        if B and self.side == B:
            if self.ref is None:
                go = True
                if self.fix == "footoff":
                    sw = "R" if B == "L" else "L"
                    if self.footoff_frac is not None:
                        go = (not seg.get("lifting")) or self.lift_frac(t) >= self.footoff_frac
                    else:
                        go = (not seg.get("lifting")) or abs(tau[JN.index(f"{sw}_knee")]) < 0.5 * self.tau0
                if go:
                    self.ref = self.centred_roll if self.fix == "centredref" else meas
                    self.engaged_at = t
            if self.fix == "pdlift" and self.ref is not None and not seg.get("lifting") and not getattr(self, "rebased", False):
                # the lift move is done: re-reference here, carry u over through I (bumpless)
                self.ref = meas
                self.pi.i = self.u_last / self.ki if self.ki else 0.0
                self.rebased = True
            if self.ref is not None:
                err = meas - self.ref
                if (self.fix == "freeze" and moving) or (self.fix == "pdlift" and seg.get("lifting")):
                    ki_save = self.pi.ki
                    unsat = self.pi.kp * err + self.pi.ki * self.pi.i + self.pi.kd * rate
                    u = max(-self.clamp, min(self.clamp, unsat))
                    self.pi.u = u
                else:
                    u = self.pi.step(err, dt, rate)
                self.u_last = u
        elif self.side is not None and not B:
            # the next pose without B: the correction fades along that move
            if self.fade is None:
                self.fade = (t, seg["t1"] - seg["t0"], getattr(self, "u_last", 0.0), self.side)
            t0, dur, u0, sd = self.fade
            u = u0 * (1.0 - minjerk(min(1.0, (t - t0) / dur)))
            if t - t0 >= dur:
                self.side = None
                self.pi = None
        sd = self.side or (self.fade[3] if self.fade else None)
        if sd and u:
            qc = qc.copy()
            qc[JN.index(f"{sd}_ankle_roll")] += u
            qc[JN.index(f"{sd}_hip_roll")] -= u
        # log
        F = self.fn(env)
        st = seg.get("stance")
        rec = dict(t=t, seg=seg["name"], roll=math.degrees(roll), err=math.degrees(err) if err == err else None,
                   u=math.degrees(u), I=(self.pi.i if self.pi else 0.0), rate=math.degrees(rate),
                   FL=F["L"], FR=F["R"], footL=d.xpos[self.fb["L"]][:2].tolist(), footR=d.xpos[self.fb["R"]][:2].tolist(),
                   yawL=float(math.atan2(d.xmat[self.fb["L"]].reshape(3, 3)[1, 0], d.xmat[self.fb["L"]].reshape(3, 3)[0, 0])),
                   yawR=float(math.atan2(d.xmat[self.fb["R"]].reshape(3, 3)[1, 0], d.xmat[self.fb["R"]].reshape(3, 3)[0, 0])),
                   tauKL=float(tau[JN.index("L_knee")]), tauKR=float(tau[JN.index("R_knee")]))
        com = d.subtree_com[0]
        rec["com_w"] = [float(com[0]), float(com[1])]
        for sd_ in "LR":
            pc = np.mean([d.geom_xpos[g] for g in self.pad[sd_]], axis=0)
            rec[f"sole{sd_}"] = [float(pc[0]), float(pc[1])]
        if st:
            pc = np.mean([d.geom_xpos[g] for g in self.pad[st]], axis=0)
            dv = K.in_foot(env.model, d, st, com) - K.in_foot(env.model, d, st, pc)
            rec["com_sole_out_mm"] = 1e3 * dv[1] * (1.0 if st == "L" else -1.0)   # stance foot frame: + = outboard of the pad centre
            _, c = K.com_margin(env.model, d, self.p, st)
            cy = self.p.foot_y_off if st == "L" else -self.p.foot_y_off
            rec["com_out_mm"] = 1e3 * (c[1] - cy) * (1.0 if st == "L" else -1.0)   # + = outboard of the stance sole centre
            rec["com_x_mm"] = 1e3 * (c[0] - (self.p.foot_toe - self.p.foot_len / 2))
            rec["margin_mm"] = 1e3 * K.sole_margin(self.p, c[:2], st)
        self.log.append(rec)
        return qc


def build(p, q, t_hold_lift=3.6, s2_only_hold=None, steps=("s1", "s2"), first="R"):
    """timeline + segment table (name, t0, t1, moving, B, stance, engage, lifting, step)."""
    q0 = q["crouch"] if steps[0] == "s1" else q["s1_centre"]
    tl = O.JointTimeline(q0)
    segs = []

    def seg(name, qq, dur, step, B=None, stance=None, moving=True, **kw):
        t0 = tl.T
        tl.to(qq, dur, name)
        segs.append(dict(name=name, t0=t0, t1=tl.T, moving=moving, B=B, stance=stance, step=step, **kw))

    seg("settle", q0, 1.5, "s0", moving=False)
    for s, st in (("s1", first), ("s2", "L" if first == "R" else "R")):
        if s not in steps:
            continue
        seg(f"{s}_shift", q[f"{s}_shift"], 3.0, s)
        seg(f"{s}_shift_h", q[f"{s}_shift"], 0.6, s, moving=False)
        if f"{s}_up" in q:      # (c): straight up first, then forward
            seg(f"{s}_up", q[f"{s}_up"], 3.0, s, B=st, stance=st, engage=True, lifting=True)
            seg(f"{s}_lift", q[f"{s}_lift"], 2.0, s, B=st, stance=st, lifting=True,
                moving=not np.allclose(q[f"{s}_up"], q[f"{s}_lift"]))
        else:
            seg(f"{s}_lift", q[f"{s}_lift"], 4.0, s, B=st, stance=st, engage=True, lifting=True)
        seg(f"{s}_lift_h", q[f"{s}_lift"], t_hold_lift, s, B=st, stance=st, moving=False)
        if s2_only_hold is not None:
            seg(f"{s}_hold", q[f"{s}_lift"], s2_only_hold, s, B=st, stance=st, moving=False)
            break
        same = np.allclose(q[f"{s}_swing"], q[f"{s}_lift"])
        seg(f"{s}_swing", q[f"{s}_swing"], 3.0, s, B=st, stance=st, moving=not same)
        seg(f"{s}_swing_h", q[f"{s}_swing"], 0.4, s, B=st, stance=st, moving=False)
        seg(f"{s}_touch", q[f"{s}_touch"], 3.0, s, B=st, stance=None)
        seg(f"{s}_touch_h", q[f"{s}_touch"], 0.4, s, B=st, stance=None, moving=False)
        seg(f"{s}_centre", q[f"{s}_centre"], 3.0, s)
        seg(f"{s}_centre_h", q[f"{s}_centre"], 2.4, s, moving=False)
    return tl, segs


def summarise(log, segs, first="R"):
    """per step: foot-off time (rel. lift start), err/u/I at foot-off, peak +/- err, CoM, slips."""
    out = {}
    for s, st in (("s1", first), ("s2", "L" if first == "R" else "R")):
        sw = "R" if st == "L" else "L"
        lift = next((g for g in segs if g["step"] == s and g.get("engage")), None)
        if lift is None:
            continue
        t0 = lift["t0"]
        endn = f"{s}_swing" if any(g["name"] == f"{s}_swing" for g in segs) else f"{s}_hold"
        tend = next(g for g in segs if g["name"] in (f"{s}_touch", f"{s}_hold"))["t0"] if True else None
        rows = [r for r in log if t0 - 0.02 <= r["t"] < tend + (100 if endn.endswith("hold") else 0)]
        rows = [r for r in rows if r["t"] < tend] if not endn.endswith("hold") else rows
        if not rows:
            continue
        F0 = rows[0][f"F{sw}"]
        Ftot = rows[0]["FL"] + rows[0]["FR"]
        off = next((r for r in rows if r[f"F{sw}"] < 0.02 * Ftot), None)
        t_off = off["t"] - t0 if off else None
        # swing-foot slide while loaded: xy displacement from lift start to foot-off, and its x part
        p0 = np.array(rows[0][f"foot{sw}"])
        pend = np.array(off[f"foot{sw}"]) if off else np.array(rows[-1][f"foot{sw}"])
        ps0 = np.array(rows[0][f"foot{st}"])
        stance_slip = max(np.linalg.norm(np.array(r[f"foot{st}"]) - ps0) for r in rows)
        stance_yaw = max(abs(r[f"yaw{st}"] - rows[0][f"yaw{st}"]) for r in rows)
        errs = [r["err"] for r in rows if r["err"] is not None]
        sgn = 1.0 if st == "L" else -1.0           # + = toward the lifting foot, either stance
        e_in = [sgn * e for e in errs]
        pre = [sgn * r["err"] for r in rows if r["err"] is not None and (off is None or r["t"] <= off["t"])]
        o = dict(t_off=t_off, F0=F0, Ftot=Ftot,
                 err_at_off=(sgn * off["err"] if off and off["err"] is not None else None),
                 u_at_off=(sgn * off["u"] if off else None),
                 com_out_at_off=(off.get("com_sole_out_mm") if off else None),
                 com_out_start=rows[0].get("com_sole_out_mm"),
                 com_out_end=rows[-1].get("com_sole_out_mm"),
                 pre_off_peak_in=max(pre) if pre else None,
                 peak_in=max(e_in) if e_in else None, peak_out=-min(e_in) if e_in else None,
                 u_end=sgn * rows[-1]["u"], err_end=(sgn * rows[-1]["err"] if rows[-1]["err"] is not None else None),
                 swing_slide_mm=1e3 * float(np.linalg.norm(pend - p0)), swing_slide_x_mm=1e3 * float(pend[0] - p0[0]),
                 stance_slip_mm=1e3 * stance_slip, stance_yaw_deg=math.degrees(stance_yaw),
                 margin_min=min((r.get("margin_mm", 1e9) for r in rows if off and r["t"] >= off["t"]), default=None))
        o["engaged_s"] = next((r["t"] - t0 for r in rows if r["err"] is not None), None)
        out[s] = o
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fix", default="base", choices=("base", "freeze", "footoff", "upfirst", "nobal", "centredref", "pdlift"))
    ap.add_argument("--first", default="R", choices=("R", "L"), help="stance of the first step (R = the bench walk)")
    ap.add_argument("--ctl-steps", default="s1,s2", help="steps with the balance on (others open loop)")
    ap.add_argument("--torso-scale", type=float, default=1.0, help="model error: torso mass x")
    ap.add_argument("--torso-off", type=float, default=0.0, help="model error: torso CoM lateral offset (m, + = robot's left)")
    ap.add_argument("--soft", type=float, default=0.0, help="carpet: contact solref time constant (s) for the foot pads, 0 = default")
    ap.add_argument("--soft-dmin", type=float, default=None, help="carpet: solimp dmin for the pads")
    ap.add_argument("--footoff-frac", type=float, default=None)
    ap.add_argument("--play", type=float, default=3.0)
    ap.add_argument("--lash", type=float, default=1.0)
    ap.add_argument("--mu", type=float, default=0.7)
    ap.add_argument("--lat", type=float, default=1.3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hold", type=float, default=None, help="stop after the lift with this extra hold (s)")
    ap.add_argument("--steps", default="s1,s2", help="s1,s2 (whole walk) / s1 / s2 (from the staggered s1_centre pose)")
    ap.add_argument("--plant-set", nargs="*", default=[])
    ap.add_argument("--log", default=None, help="write the per-tick log (json)")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    p, _ = params_from_args(["-o", "/dev/null", "--set", *O.AS_BUILT])
    p_sim = p if not a.plant_set else params_from_args(["-o", "/dev/null", "--set", *O.AS_BUILT, *a.plant_set])[0]
    q, keys = walk_keys(p, lat=a.lat, upfirst=(a.fix == "upfirst"), first=a.first)
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"s2lift_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p_sim))
    tl, segs = build(p, q, s2_only_hold=a.hold, steps=tuple(a.steps.split(",")), first=a.first)
    fix = "base" if a.fix == "upfirst" else a.fix
    ctl = Ctl(p_sim, segs, fix=fix, footoff_frac=a.footoff_frac, seed=a.seed, ctl_steps=tuple(a.ctl_steps.split(",")))
    if a.fix == "nobal":
        ctl.kp = ctl.ki = ctl.kd = 0.0
    pj = {"L_hip_roll": "sts3250", "R_hip_roll": "sts3250"}
    def setup(env):
        m = env.model
        b = env._torso_bid
        m.body_mass[b] *= a.torso_scale
        m.body_inertia[b] *= a.torso_scale
        m.body_ipos[b][1] += a.torso_off
        for g in range(m.ngeom):
            if "_pad_" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""):
                if a.soft:
                    m.geom_solref[g] = [a.soft, 1.0]
                if a.soft_dmin is not None:
                    m.geom_solimp[g][0] = a.soft_dmin
    r = O.run(p_sim, xml, tl, [], per_joint=pj, hook=ctl, mu=a.mu, play_deg=a.play, backlash_deg=a.lash, seed=a.seed,
              setup=setup)
    os.unlink(xml)
    summ = summarise(ctl.log, segs, a.first)
    # abort: first |err| > 12 deg
    ab = next((x for x in ctl.log if x["err"] is not None and abs(x["err"]) > 12.0), None)
    res = dict(fix=a.fix, fell=r["fell"], t_fell=r["t_fell"], abort_t=ab["t"] if ab else None, abort_seg=ab["seg"] if ab else None,
               steps=summ, T=tl.T, lift_t0={s["step"]: s["t0"] for s in segs if s.get("engage")})
    if not a.quiet:
        print(json.dumps(res, indent=1, default=lambda o: round(o, 3) if isinstance(o, float) else str(o)))
    if a.log:
        json.dump(dict(res=res, log=ctl.log, segs=segs), open(a.log, "w"))
    return res


if __name__ == "__main__":
    main()
