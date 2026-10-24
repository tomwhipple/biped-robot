"""One-leg stand for the v6 body: the Gate A path held still -- crouch, shift the
pelvis until the whole-robot CoM sits (5 mm inboard of) the LEFT sole centre,
lift the RIGHT foot, hold, and undo it all. No policy, no feedback.

What the robot runs is the firmware's `pose ... s<ms>` streamer: a minimum-jerk
move in JOINT space between keyframes. So this simulates exactly that -- IK
keyframes (sim/v6_kin.leg_ik via design_gates.q_of), joint-space min-jerk
between them -- on the dynamic plant under the deploy servo model of
sim/static_gait.run_walk (2 Hz shaper + 80 ms dead time, integer ticks, 1 deg
backlash, 3 deg play on the four rolls), and prints the same verdict row.

It also writes the keyframes as bus ticks for the as-built calibration
(ticks = zero + dir * rad * 4096 / 2pi, docs/servo-map.md section 3), in `pose`
bus order, so the bench session sends the poses this file simulated.

    .venv/bin/python experiments/one-leg-stand/one_leg.py              # nominal + sweep
    .venv/bin/python experiments/one-leg-stand/one_leg.py --lift 0.015 --json build/one_leg.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("MUJOCO_GL", "egl")

import static_gait as SG  # noqa: E402
from design_gates import JN, Key, minjerk, q_of, standing_key  # noqa: E402
from gen_plant_v6 import build_xml, params_from_args  # noqa: E402
import v6_kin as K  # noqa: E402

# `pose` bus order (firmware obs/bus_map.h kBusJointNames, as `cal show` lists it)
BUS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
       "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
N_BUS = 17                                     # 5 more (neck, arms): not fitted, ignored by `pose`
# as-built calibration, `cal show` after the 2026-10-24 re-zero (16:33Z) and roll-zero fix (18:14Z) (NVS): joint -> (zero ticks, dir).
# The 2026-10-24 stand rehearsal ran on the previous zeros (all 2047-2048).
CAL = {"R_hip_roll": (2082, -1), "R_hip_pitch": (1997, 1), "R_knee": (2125, -1), "R_ankle": (2045, 1),
       "L_hip_roll": (1991, -1), "L_hip_pitch": (2058, 1), "L_knee": (1977, -1), "L_ankle": (2091, 1),
       "R_hip_yaw": (2184, 1), "L_hip_yaw": (2153, 1), "R_ankle_roll": (2108, 1), "L_ankle_roll": (1997, 1)}
ROLLS = ("L_hip_roll", "R_hip_roll", "L_ankle_roll", "R_ankle_roll")
# The robot as it is on the bench (2026-10-24): v6 legs, tethered power, the torso is the print (m_pelvis)
# plus the General Driver board (m_board) and its leads. No pack, Pi, head, neck servo, power board, arms.
# Geometry untouched (torso_v7 stays on: the deck height and board position are the printed torso's).
# m_wiring 0.010 is a guess (board leads + servo bus cable); the true torso mass is unmeasured.
AS_BUILT = ["m_battery=0", "m_pi4=0", "m_power=0", "m_neck_servo=0", "m_head=0", "m_wiring=0.01"]


class JointTimeline:
    """joint-space min-jerk between keyframes -- what `pose s<ms>` streams."""

    def __init__(self, q0):
        self.q = [np.asarray(q0, float)]
        self.durs = []
        self.names = ["start"]

    def to(self, q, dur, name=""):
        self.q.append(np.asarray(q, float)); self.durs.append(dur); self.names.append(name)
        return self

    def hold(self, dur, name="hold"):
        return self.to(self.q[-1], dur, name)

    @property
    def T(self):
        return float(sum(self.durs))

    def t_of(self, name):
        return float(sum(self.durs[:self.names.index(name)]))

    def at(self, t):
        acc = 0.0
        for k, dur in enumerate(self.durs):
            if t <= acc + dur:
                s = minjerk((t - acc) / dur) if dur > 0 else 1.0
                return self.q[k] + (self.q[k + 1] - self.q[k]) * s
            acc += dur
        return self.q[-1]


def keyframes(p, drop=0.02, lift=0.03, bias_y=-0.005, frac=1.0, stance="L"):
    """IK keyframes (12 joint angles, JN order, rad). frac scales the lateral
    shift (0.5 = the half-amplitude bench stage). stance: the foot that stays
    down ("L"/"R"); the other lifts. The shift is solved from the model's CoM for
    whichever stance, and bias_y is inboard (toward the lifted foot) for both:
    one code path, no per-side constants."""
    swing = "R" if stance == "L" else "L"
    sgn = 1.0 if stance == "L" else -1.0
    m, d = K.load(p)
    k0 = standing_key(p, 0.0)
    k1 = standing_key(p, drop)
    feet = {"L": k1.footL.copy(), "R": k1.footR.copy()}
    mid = feet[swing] + np.array([0.0, 0.0, lift])
    pel = SG.solve_pelvis(m, d, p, k1.pelvis, feet, stance, mid, bias=(0.0, sgn * bias_y))
    pel[2] = k1.pelvis[2]
    pel = k1.pelvis + (pel - k1.pelvis) * frac
    k2 = Key(pel, k1.footL, k1.footR)
    k3 = k2.copy()
    if swing == "R":
        k3.footR = k3.footR + np.array([0.0, 0.0, lift])
    else:
        k3.footL = k3.footL + np.array([0.0, 0.0, lift])
    q = {n: q_of(p, k) for n, k in (("straight", k0), ("crouch", k1), ("shift", k2), ("lift", k3))}
    # static CoM margin on the left sole at the lifted pose, from the kinematics
    K.set_pose(m, d, q["lift"], torso_pos=k3.pelvis + np.array([0, 0, 1.0]))
    marg, _ = K.com_margin(m, d, p, stance)
    contacts = {}
    for n, qq in q.items():                    # self-contact at each keyframe (shift is a parallelogram,
        K.set_pose(m, d, qq)                   # so the gap between the legs is constant between them)
        contacts[n] = K.self_collides(m, d)
    return q, dict(pelvis_shift_mm=round(1e3 * (pel[1] - k1.pelvis[1]), 1), static_margin_mm=round(1e3 * marg, 1),
                   self_contacts=contacts)


def lift_only(p, side="R", heights=(0.015, 0.03), drop=0.0):
    """Keyframes for the clamped-foot bench: from standing `drop` below straight,
    raise one foot straight up by each height with the sole parallel to the
    desk -- hip pitch, knee and ankle only; no roll, no pelvis shift. The other
    foot (clamped) and the pelvis stay where they are."""
    k = standing_key(p, drop)
    q = {"stand": q_of(p, k)}
    for h in heights:
        kk = k.copy()
        if side == "R":
            kk.footR = kk.footR + np.array([0.0, 0.0, h])
        else:
            kk.footL = kk.footL + np.array([0.0, 0.0, h])
        q[f"lift{int(round(1e3 * h))}"] = q_of(p, kk)
    return q


def timeline(q, t_crouch=2.0, t_shift=3.0, t_lift=2.0, t_hold=10.0, swing="R"):
    tl = (JointTimeline(q["straight"]).hold(0.5, "settle0").to(q["crouch"], t_crouch, "crouch")
          .hold(0.5, "h1").to(q["shift"], t_shift, "shift").hold(1.0, "h2")
          .to(q["lift"], t_lift, "lift").hold(t_hold, "hold").to(q["shift"], t_lift, "lower")
          .hold(0.5, "h3").to(q["crouch"], t_shift, "centre").hold(0.5, "h4")
          .to(q["straight"], t_crouch, "straighten").hold(1.0, "end"))
    win = [(tl.t_of("lift"), tl.t_of("lower") + t_lift, swing)]
    return tl, win


def ticks(qjn):
    """JN-order radians -> 17 `pose` targets (bus order; unfitted slots 2048)."""
    out = []
    for n in BUS:
        z, s = CAL[n]
        out.append(int(round(z + s * qjn[JN.index(n)] * 4096 / (2 * math.pi))))
    return out + [2048] * (N_BUS - len(BUS))


def run(p, xml, tl, win, **kw):
    orig = SG.q_of
    SG.q_of = lambda p_, key: key if isinstance(key, np.ndarray) else orig(p_, key)
    try:
        return SG.run_walk(p, xml, tl, win, **kw)
    finally:
        SG.q_of = orig


def bal_hook(tl, stance, kp, ki, ka, kh, clamp_deg, **kw):
    """The stance-leg roll PI (balance.py) engaged at the start of the lift --
    reference captured at the shifted pose -- through the hold, faded out over the lowering."""
    from balance import Balance
    seg = lambda n: tl.durs[tl.names.index(n) - 1]      # t_of(name) is the END of that segment
    t_lift0, t_lift1 = tl.t_of("lift") - seg("lift"), tl.t_of("lift")
    t_hold1 = tl.t_of("hold")
    return Balance(stance, kp, ki, ka, kh, math.radians(clamp_deg), t_on=t_lift0, t_off=t_hold1,
                   t_fade=seg("lower"), t_hold0=t_lift1, t_hold1=t_hold1, **kw)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--stance", choices=("L", "R"), default="L", help="the foot that stays down; the other lifts")
    ap.add_argument("--lift", type=float, default=0.03)
    ap.add_argument("--drop", type=float, default=0.02)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--bias-y", type=float, default=-0.005)
    ap.add_argument("--rolls3250", action="store_true", help="STS3250 at the four rolls (design); "
                    "default = as built: STS3250 at the two HIP rolls only")
    ap.add_argument("--all3215", action="store_true", help="STS3215 everywhere (worst case)")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--json", default=None)
    ap.add_argument("--plant-set", nargs="*", default=[], metavar="KEY=VAL",
                    help="simulate on a plant changed from the one the keyframes are solved on "
                         "(e.g. m_battery=0 m_pi4=0 torso_v7=0: the tethered robot without pack, Pi, head)")
    ap.add_argument("--solve-set", nargs="*", default=[], metavar="KEY=VAL",
                    help="solve the keyframes on a plant changed from the design (default: the design plant)")
    ap.add_argument("--as-built", action="store_true",
                    help="solve the keyframes AND simulate on the as-built plant (AS_BUILT; --plant-set/--solve-set add to it)")
    ap.add_argument("--bal-kp", type=float, default=0.0, help="stance-leg roll PI: P gain (rad/rad)")
    ap.add_argument("--bal-ki", type=float, default=0.0, help="stance-leg roll PI: I gain (1/s)")
    ap.add_argument("--bal-ka", type=float, default=1.0, help="PI output -> stance ankle roll (signed mix)")
    ap.add_argument("--bal-kh", type=float, default=0.0, help="PI output -> stance hip roll (signed mix)")
    ap.add_argument("--bal-clamp", type=float, default=6.0, help="PI output clamp, deg")
    ap.add_argument("--fb-ankle", type=float, default=0.0, help="IMU torso roll -> ankle roll P gain (rad/rad)")
    ap.add_argument("--fb-hip", type=float, default=0.0, help="IMU torso roll -> hip roll P gain (rad/rad)")
    ap.add_argument("--fb-i", type=float, default=0.0, help="integral gain on the same roll error (1/s), ankle rolls")
    ap.add_argument("--lift-only", choices=("L", "R"), default=None,
                    help="print clamped-foot lift poses for this foot (no shift) and exit")
    a = ap.parse_args(argv)
    if a.as_built:
        a.solve_set = AS_BUILT + a.solve_set
        a.plant_set = AS_BUILT + a.plant_set
    p, _ = params_from_args(["-o", "/dev/null", "--set", *a.solve_set])
    if a.lift_only:
        q = lift_only(p, a.lift_only, heights=(0.015, 0.03), drop=a.drop if a.drop != 0.02 else 0.0)
        for n, v in q.items():
            print(f"{n:7s}", "  ".join(f"{j.replace('_hip', '').replace('ankle_roll', 'aroll')} "
                                       f"{math.degrees(float(v[JN.index(j)])):+5.1f}" for j in JN if j[0] == a.lift_only))
            print(f"P_{n}=\"pose {' '.join(map(str, ticks(v)))}\"")
        return
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"one_leg_{os.getpid()}.xml")
    p_sim = p
    if a.plant_set:
        p_sim, _ = params_from_args(["-o", "/dev/null", "--set", *a.plant_set])
    with open(xml, "w") as f:
        f.write(build_xml(p_sim))
    fb = dict(fb_ankle=a.fb_ankle, fb_hip=a.fb_hip, fb_i=a.fb_i)
    if a.all3215:
        pj = None
    elif a.rolls3250:
        pj = {j: "sts3250" for j in ROLLS}
    else:
        pj = {"L_hip_roll": "sts3250", "R_hip_roll": "sts3250"}
    out = dict(design=p.summary(), servos=pj or "sts3215 everywhere", lift=a.lift, drop=a.drop, stages={})
    for frac in (0.5, 1.0):
        q, info = keyframes(p, a.drop, a.lift, a.bias_y, frac, a.stance)
        out["stages"][f"shift{int(frac * 100)}"] = dict(
            info=info, rad={n: [round(float(x), 4) for x in v] for n, v in q.items()},
            deg={n: {j: round(math.degrees(float(v[JN.index(j)])), 1) for j in JN} for n, v in q.items()},
            ticks={n: ticks(v) for n, v in q.items()})
    q, info = keyframes(p, a.drop, a.lift, a.bias_y, 1.0, a.stance)
    tl, win = timeline(q, t_hold=a.hold, swing="R" if a.stance == "L" else "L")
    print(f"design {p.summary()}\nservos {out['servos']}  lift {1e3*a.lift:.0f} mm  drop {1e3*a.drop:.0f} mm  "
          f"hold {a.hold:.0f} s  total {tl.T:.1f} s\nkinematics: pelvis shift {info['pelvis_shift_mm']} mm, "
          f"static CoM margin on the {a.stance} sole {info['static_margin_mm']} mm, "
          f"self-contacts per keyframe {info['self_contacts']}")
    for n in ("crouch", "shift", "lift"):
        print(f"  {n:8s}", "  ".join(f"{j.replace('_hip', '').replace('ankle_roll', 'aroll')} "
                                     f"{math.degrees(float(q[n][JN.index(j)])):+5.1f}" for j in JN))
    print(f"bench sequence ({a.stance} stance; CAL ticks as of its comment -- refresh from `cal show` first):")
    for n, ms in (("crouch", 2000), ("shift", 3000), ("lift", 2000)):
        print(f"  pose {' '.join(map(str, ticks(q[n])))} s{ms}" + (f" B{a.stance}" if n == "lift" and True else ""))
    if a.plant_set or any(fb.values()):
        print(f"simulated plant: {p_sim.summary()}  set {a.plant_set}  feedback {fb}")
    bal = None
    if a.bal_kp or a.bal_ki:
        bal = bal_hook(tl, a.stance, a.bal_kp, a.bal_ki, a.bal_ka, a.bal_kh, a.bal_clamp)
        print(f"stance-leg roll PI on L: kp {a.bal_kp} ki {a.bal_ki} ankle x{a.bal_ka} hip x{a.bal_kh} clamp {a.bal_clamp} deg")
    r = run(p_sim, xml, tl, win, per_joint=pj, track_torque=True, hook=bal, **fb)
    if bal is not None:
        out["balance"] = bal.metrics()
        print("  balance:", {k: round(float(v), 2) for k, v in out["balance"].items()})
    print(SG.fmt_row("nominal (mu 0.7, play 3, lash 1, lag 2 Hz + 80 ms)", r))
    tau = {j: (round(r["peak_tau"][i], 2), round(r["stall"][i], 2)) for i, j in enumerate(JN)}
    print("  peak |tau| / stall N*m:", "  ".join(f"{j} {t}/{s}" for j, (t, s) in tau.items() if t > 0.3 * s))
    out["nominal"] = {k: v for k, v in r.items() if k not in ("peak_tau", "stall")}
    out["nominal"]["peak_tau_over_stall"] = {j: round(t / s, 2) for j, (t, s) in tau.items()}
    if not a.no_sweep:
        cases = [("friction mu 0.4", dict(mu=0.4)), ("friction mu 1.0", dict(mu=1.0)),
                 ("play 5 deg on rolls", dict(play_deg=5.0)), ("backlash 2 deg", dict(backlash_deg=2.0)),
                 ("mass x0.85", dict(mass_scale=0.85)), ("mass x1.15", dict(mass_scale=1.15)),
                 ("payload 120 g deck front", dict(payload=0.12)), ("servos -15 %", dict(servo_scale=0.85)),
                 ("floor tilt +2 deg", dict(floor_tilt_deg=2.0)), ("floor tilt -2 deg", dict(floor_tilt_deg=-2.0)),
                 ("mu 0.4 + play 5 + servos -15 %", dict(mu=0.4, play_deg=5.0, servo_scale=0.85))]
        out["sweep"] = {}
        bal = None if bal is None else True
        for label, kw in cases:
            rs = [run(p_sim, xml, tl, win, seed=s, per_joint=pj, **fb, **kw,
                      hook=(bal_hook(tl, a.stance, a.bal_kp, a.bal_ki, a.bal_ka, a.bal_kh, a.bal_clamp) if bal else None))
                  for s in range(3)]
            worst = min(rs, key=lambda x: (x["ok"], x["min_margin"] if x["min_margin"] == x["min_margin"] else -1))
            print(SG.fmt_row(label, worst) + f"   ({sum(x['ok'] for x in rs)}/3 seeds ok)")
            out["sweep"][label] = worst
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        json.dump(out, open(a.json, "w"), indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
        print("wrote", a.json)


if __name__ == "__main__":
    main()
