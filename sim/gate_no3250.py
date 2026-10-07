"""No-STS3250 study (2026-09-26): can the v6 body walk and get up with
STS3215s at the hip rolls, ankle rolls and knees -- the six joints the design
gave to the STS3250 (cad/v6/dimensions_v6.SERVO_3250_JOINTS_SIX)?

Tom asked for a plan under the assumption that no genuine STS3250 can be
bought. Every gate here is the design's own gate, re-run on the CURRENT plant
under the SAME deploy servo model the design doc states (kp 12 N*m/rad,
torque-speed clamp at 11.1 V, 2 Hz shaper + 80 ms dead time, integer ticks,
5 ms bus latency, 1 deg backlash, 3 deg free play on the four roll joints),
with only the servo assignment -- and the servo MASS -- changed:

  plants (CAD inertials, sim/build_v6_inertia.plant_xml, written to TMPDIR;
  the committed sim/bimo_biped_v6ar.xml is never touched):
    cad3250   74.5 g servos at the six places (the design)
    cad3215   55.0 g there (-117 g, 1.67 kg)
  and, for `arms` / `getup`, the as-drawn robot with the elbowed arms
  (getup_v6_shoulder round 6, r6_asdrawn[_rom120]: lumped, 2.10 kg, 55 g
  servos everywhere -- the all-STS3215 mass; its STS3250 rows are 117 g light)
  servo sets (design_gates.SERVOS; the kp_scale entries registered below):
    3250      STS3250 at rolls + knees (the design)
    3215      STS3215 everywhere
    3215_p2   STS3215 everywhere, the four ROLL servos with the position-loop
              P coefficient doubled (Feetech memory table V3.7, register 21,
              default 32 -> 64): modelled as kp_scale 2 on the 3215 envelope --
              the same way the model credits the STS3250 (kp_scale 4). The
              REAL stiffness gain of that register is unmeasured.
    3215_p3   the same at kp_scale 3 (P coefficient ~96); _p4 at 4 (~128,
              the stiffness the model credits the STS3250 with)
    *_rk      the same stiffening on the knees too

Modes (logs in docs/design-v6/no3250_*.txt):

    .venv/bin/python sim/gate_no3250.py walk      > docs/design-v6/no3250_walk.txt
    .venv/bin/python sim/gate_no3250.py sweep     > docs/design-v6/no3250_sweep.txt
    .venv/bin/python sim/gate_no3250.py envelope  > docs/design-v6/no3250_envelope.txt
    .venv/bin/python sim/gate_no3250.py arms      > docs/design-v6/no3250_arms_walk.txt
    .venv/bin/python sim/gate_no3250.py getup     > docs/design-v6/no3250_getup.txt

On the Mac: MUJOCO_GL=cgl in the environment (the sim modules setdefault egl).
Design record: docs/design-v6/2026-09-13-design-record.md section 14.
"""
from __future__ import annotations

import dataclasses
import itertools
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

import design_gates as DG  # noqa: E402
import static_gait as SG  # noqa: E402
from gen_plant_v6 import DesignParams, build_xml  # noqa: E402

TMP = os.environ.get("TMPDIR", "/tmp")
ROLLS = SG.ROLLS
KNEES = ("L_knee", "R_knee")

# study-only servo entries: the STS3215 envelope with a stiffer position loop
_s = DG.SERVOS["sts3215"]
DG.SERVOS["sts3215_p2"] = dict(stall=_s["stall"], w0=_s["w0"], kp_scale=2.0)
DG.SERVOS["sts3215_p3"] = dict(stall=_s["stall"], w0=_s["w0"], kp_scale=3.0)
DG.SERVOS["sts3215_p4"] = dict(stall=_s["stall"], w0=_s["w0"], kp_scale=4.0)
# bench, 2026-10-01/02 (experiments/plan-b-bench): P 128 / D 32 is the highest
# STABLE setting under a leg-like load and measures 3.5 +- 0.4x the P 32
# stiffness (friction-cancelled); P 160 (4.8x) limit-cycles
DG.SERVOS["sts3215_p35"] = dict(stall=_s["stall"], w0=_s["w0"], kp_scale=3.5)
# bench 2026-10-02: P 96 / D 0 -- quiet at hold, near-stock shaking at walking
# speeds -- measures ~2.8x (clamp probes); P 128 is rougher with a leg-like load
DG.SERVOS["sts3215_p28"] = dict(stall=_s["stall"], w0=_s["w0"], kp_scale=2.8)
# ...and the STS3250 if it is less stiff than the third-party 4x (design doc 8.1)
_b = DG.SERVOS["sts3250"]
DG.SERVOS["sts3250_k3"] = dict(stall=_b["stall"], w0=_b["w0"], kp_scale=3.0)
DG.SERVOS["sts3250_k2"] = dict(stall=_b["stall"], w0=_b["w0"], kp_scale=2.0)

SETS = {
    "3250": {j: "sts3250" for j in ROLLS + KNEES},
    "3250_k3": {j: "sts3250_k3" for j in ROLLS + KNEES},
    "3250_k2": {j: "sts3250_k2" for j in ROLLS + KNEES},
    "3215": {},
    "3215_p2": {j: "sts3215_p2" for j in ROLLS},
    "3215_p3": {j: "sts3215_p3" for j in ROLLS},
    "3215_p4": {j: "sts3215_p4" for j in ROLLS},
    "3215_p2rk": {j: "sts3215_p2" for j in ROLLS + KNEES},
    "3215_p3rk": {j: "sts3215_p3" for j in ROLLS + KNEES},
    "3215_p4rk": {j: "sts3215_p4" for j in ROLLS + KNEES},
}

# the four cases the CAD-inertial walk and the as-drawn arms walk were judged on
# (gateD_cad_inertials.txt, getup_search_r5_asdrawn_walk.txt)
CASES4 = (("turn  +0 mu 0.7 play 3", {}, {}),
          ("turn +15 mu 0.7 play 3", {}, dict(turn_deg=15.0)),
          ("turn  +0 mu 0.3 play 5", dict(mu=0.3, play_deg=5.0), {}),
          ("turn -15 mu 0.9 play 3", dict(mu=0.9), dict(turn_deg=-15.0)))

GAIT = dict(n_steps=8, step=0.06, lift_h=0.04)       # the CAD-inertial walk


# ------------------------------------------------------------------ plants
def cad_plant(servo_g: float, torso_minus_g: float = 0.0, p: DesignParams | None = None,
              arms: bool = True) -> str:
    """CAD-inertial plant with `servo_g` servos at the six STS3250 places,
    optionally with `torso_minus_g` taken off the torso (mass and inertia
    scaled together, CoM unchanged -- a lighter-torso lever estimate)."""
    import build_v6_inertia as BI
    p = p or DesignParams()
    src, _ = BI.plant_xml(p, servo_g, arms=arms)
    if torso_minus_g:
        m = re.search(r'(<body name="torso"[^>]*>\n\s*<freejoint[^>]*/>\n\s*<inertial [^>]*mass=")([0-9.]+)(" diaginertia=")([^"]+)(")', src)
        M = float(m.group(2))
        f = (M - torso_minus_g * 1e-3) / M
        I = " ".join(f"{float(x) * f:.4e}" for x in m.group(4).split())
        src = src[:m.start()] + m.group(1) + f"{M * f:.4f}" + m.group(3) + I + m.group(5) + src[m.end():]
    tag = (f"v6_{servo_g:g}g_t{torso_minus_g:g}_{p.thigh * 1e3:.0f}_fy{p.foot_y_off * 1e4:.0f}"
           f"_{'a' if arms else 'na'}_{os.getpid()}.xml")
    path = os.path.join(TMP, tag)
    with open(path, "w") as fh:
        fh.write(src)
    return path


def plant_mass(xml: str) -> float:
    import mujoco
    return float(sum(mujoco.MjModel.from_xml_path(xml).body_mass))


# ------------------------------------------------------------------ walk
def walk_one(p, xml, set_name, gait=None, run=None, tl_kw=None, seed=0, track=False):
    g = dict(GAIT, **(gait or {}), **(tl_kw or {}))
    tl, w = SG.walk_timeline(p, **g)
    return SG.run_walk(p, xml, tl, w, seed=seed, per_joint=SETS[set_name] or None,
                       track_torque=track, track_speed=track, **(run or {}))


def _walk_job(args):
    label, p, xml, set_name, gait, run, tl_kw = args
    r = walk_one(p, xml, set_name, gait, run, tl_kw)
    return label, r


def pmap(fn, jobs, n=None):
    from multiprocessing import Pool
    if not jobs:
        return []
    with Pool(n or min(len(jobs), max(1, (os.cpu_count() or 4) - 2))) as pool:
        return pool.map(fn, jobs)


def mode_walk():
    """the four gate cases for the design, STS3215 everywhere, and every
    mitigation tried on the STS3215 body -- all of them printed, pass or fail.
    Row verdicts are static_gait's: OK = 8/8 steps, swing clearance >= 15 mm
    and >= 0.3 s airborne; a row can stay up and still FAIL on clearance."""
    p = DesignParams()
    x50, x15 = cad_plant(74.5), cad_plant(55.0)
    xl60, xl120 = cad_plant(55.0, 60.0), cad_plant(55.0, 120.0)
    p100 = dataclasses.replace(p, thigh=0.100, shank=0.100)
    x15_100 = cad_plant(55.0, p=p100)
    print(f"== Gate D, 4 cases (8 steps, step 6 cm, lift 4 cm, shift 1.6 s/60 mm, swing 1.6 s, land 0.5 s), CAD-inertial plant, deploy servo model")
    print(f"   plant masses: cad3250 {plant_mass(x50):.3f} kg, cad3215 {plant_mass(x15):.3f} kg, "
          f"cad3215 torso -60 g {plant_mass(xl60):.3f}, -120 g {plant_mass(xl120):.3f}, legs 100/100 {plant_mass(x15_100):.3f}")
    V = [  # (variant label, params, xml, servo set, gait kw, run kw)
        ("A  design: STS3250 rolls+knees", p, x50, "3250", {}, {}),
        ("A2 STS3250 only 3x the 3215's stiffness", p, x50, "3250_k3", {}, {}),
        ("A3 STS3250 only 2x the 3215's stiffness", p, x50, "3250_k2", {}, {}),
        ("A5 STS3250 at 3x + sag comp k 1", p, x50, "3250_k3", {}, dict(fb_sag=1.0)),
        ("A6 STS3250 at 2x + sag comp k 1", p, x50, "3250_k2", {}, dict(fb_sag=1.0)),
        ("A4 design, faster: shift 1.2 swing 1.2 land 0.4", p, x50, "3250", dict(t_shift=1.2, t_swing=1.2, t_land=0.4), {}),
        ("B  STS3215 everywhere", p, x15, "3215", {}, {}),
        ("B1 3215 + sag comp k 0.5", p, x15, "3215", {}, dict(fb_sag=0.5)),
        ("B2 3215 + sag comp k 1.0", p, x15, "3215", {}, dict(fb_sag=1.0)),
        ("B3 3215 slower: shift 2.4 swing 2.0 land 0.8", p, x15, "3215", dict(t_shift=2.4, t_swing=2.0, t_land=0.8), {}),
        ("B4 3215 slower: shift 3.2 swing 2.4 land 1.0", p, x15, "3215", dict(t_shift=3.2, t_swing=2.4, t_land=1.0), {}),
        ("B5 3215 lift 3 cm", p, x15, "3215", dict(lift_h=0.03), {}),
        ("B6 3215 CoM aim 10 mm inboard", p, x15, "3215", dict(bias_y=-0.010), {}),
        ("B7 3215 land_out 20 mm, swing_out 15 mm", p, x15, "3215", dict(land_out=0.020, swing_out=0.015), {}),
        ("B8 3215 torso -60 g", p, xl60, "3215", {}, {}),
        ("B9 3215 torso -120 g", p, xl120, "3215", {}, {}),
        ("B10 3215 legs 100/100 mm", p100, x15_100, "3215", {}, {}),
        ("B11 3215 torso -120 g + sag comp k 1 + slower", p, xl120, "3215", dict(t_shift=2.4, t_swing=2.0, t_land=0.8), dict(fb_sag=1.0)),
        ("C1 3215, roll P coef x2 (kp_scale 2)", p, x15, "3215_p2", {}, {}),
        ("C2 3215, roll P coef x2 + sag comp k 1", p, x15, "3215_p2", {}, dict(fb_sag=1.0)),
        ("C3 3215, roll P coef x3 (kp_scale 3)", p, x15, "3215_p3", {}, {}),
        ("C4 3215, roll P coef x3 + sag comp k 1", p, x15, "3215_p3", {}, dict(fb_sag=1.0)),
        ("C5 3215, roll P coef x4 (= the 3250's modelled kp)", p, x15, "3215_p4", {}, {}),
        ("C6 3215, P coef x2 rolls + knees", p, x15, "3215_p2rk", {}, {}),
        ("C7 3215, P coef x3 rolls + knees", p, x15, "3215_p3rk", {}, {}),
        ("C8 3215, P coef x4 rolls + knees", p, x15, "3215_p4rk", {}, {}),
        ("C9 3215, P coef x3 rolls + knees, lift 5 cm", p, x15, "3215_p3rk", dict(lift_h=0.05), {}),
        ("C10 3215, P coef x3 rolls, lift 5 cm", p, x15, "3215_p3", dict(lift_h=0.05), {}),
        ("C11 3215, P coef x2 rolls + knees, lift 5 cm", p, x15, "3215_p2rk", dict(lift_h=0.05), {}),
        ("C12 3215, P x4 r+k, faster: shift 1.2 swing 1.2 land 0.4", p, x15, "3215_p4rk", dict(t_shift=1.2, t_swing=1.2, t_land=0.4), {}),
        ("C13 3215, P x4 r+k on the 74.5 g plant (mass control)", p, x50, "3215_p4rk", {}, {}),
        ("D1 3215, roll play 1 deg (every case)", p, x15, "3215", {}, dict(play_deg=1.0)),
        ("D2 3215 + sag comp k 1, roll play 1 deg", p, x15, "3215", {}, dict(play_deg=1.0, fb_sag=1.0)),
        ("D3 3215, P x3 rolls + knees, roll play 1 deg", p, x15, "3215_p3rk", {}, dict(play_deg=1.0)),
    ]
    jobs = []
    for vl, pp, xml, sn, gait, run in V:
        for cl, ckw, tkw in CASES4:
            # the variant's own run settings win over the case's (D*: play 1 everywhere)
            jobs.append((f"{vl:46s} | {cl}", pp, xml, sn, gait, dict(ckw, **run), tkw))
    res = pmap(_walk_job, jobs)
    last = None
    n_ok = n_up = 0
    for label, r in res:
        v = label.split("|")[0]
        if last is not None and v != last:
            print(f"   -> {n_up}/4 stayed up 8/8, {n_ok}/4 OK (clearance rule too)\n", flush=True)
            n_ok = n_up = 0
        last = v
        n_ok += bool(r["ok"])
        n_up += (not r["fell"]) and r["steps_completed"] == r["n_steps"]
        print(SG.fmt_row(label, r), flush=True)
    print(f"   -> {n_up}/4 stayed up 8/8, {n_ok}/4 OK (clearance rule too)")


def mode_sweep(variants_fn=None, arms=True):
    """static_gait's full Gate C/D adversity matrix, 8 steps, worst of 3
    seeds, on the CAD-inertial plants: the design and the best STS3215 fallbacks."""
    p = DesignParams()
    x50, x15 = cad_plant(74.5, arms=arms), cad_plant(55.0, arms=arms)
    cases = [("nominal (mu 0.7, play 3, lash 1, lag 2 Hz + 80 ms)", {})]
    for mu in (0.3, 0.5, 1.0):
        cases.append((f"friction mu {mu}", dict(mu=mu)))
    cases += [("no play, no backlash (best case chains)", dict(play_deg=0.0, backlash_deg=0.0)),
              ("play 1 deg on rolls", dict(play_deg=1.0)),
              ("play 5 deg on rolls", dict(play_deg=5.0)),
              ("backlash 2 deg", dict(backlash_deg=2.0)),
              ("mass x0.85", dict(mass_scale=0.85)), ("mass x1.15", dict(mass_scale=1.15)),
              ("payload 60 g on the deck front", dict(payload=0.06)),
              ("payload 120 g on the deck front", dict(payload=0.12)),
              ("servos -15 % (stall and speed)", dict(servo_scale=0.85)),
              ("servos -30 %", dict(servo_scale=0.70)),
              ("floor tilt +2 deg (fore-aft)", dict(floor_tilt_deg=2.0)),
              ("floor tilt -2 deg (fore-aft)", dict(floor_tilt_deg=-2.0)),
              ("no lag / no dead time (ideal chain)", dict(lag_hz=0.0, delay_ticks=0)),
              ("mu 0.3 + play 5 + servos -15 %", dict(mu=0.3, play_deg=5.0, servo_scale=0.85))]
    variants = [("A  design: STS3250 rolls+knees", x50, "3250", {}),
                ("B  STS3215 everywhere", x15, "3215", {}),
                ("C8 3215, P coef x4 rolls + knees", x15, "3215_p4rk", {}),
                ("C7 3215, P coef x3 rolls + knees", x15, "3215_p3rk", {})]
    if variants_fn:
        variants = variants_fn(x50, x15)
    for vl, xml, sn, run in variants:
        print(f"== {vl}  (CAD-inertial plant {plant_mass(xml):.3f} kg, 8 steps, lift 4 cm, worst of 3 seeds)", flush=True)
        jobs = [(f"{cl}#{s}", p, xml, sn, {}, dict(run, seed=s, **ckw), {}) for cl, ckw in cases for s in range(3)]
        res = pmap(_walk_job_seeded, jobs)
        by = {}
        for label, r in res:
            by.setdefault(label.split("#")[0], []).append(r)
        n = 0
        for cl, _ in cases:
            rs = by[cl]
            worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
            n += all(r["ok"] for r in rs)
            print(SG.fmt_row(cl, worst) + f"   ({sum(r['ok'] for r in rs)}/3 seeds ok)", flush=True)
        print(f"   -> {n}/{len(cases)} cases OK on every seed\n", flush=True)


def _walk_job_seeded(args):
    label, p, xml, set_name, gait, run, tl_kw = args
    run = dict(run)
    seed = run.pop("seed", 0)
    return label, walk_one(p, xml, set_name, gait, run, tl_kw, seed=seed)


# ------------------------------------------------------------------ envelope
def mode_envelope():
    """per-joint peak |torque| and |speed| against each servo's stall and
    no-load speed at 11.1 V: (1) in the deploy-model walk itself (the four
    gate cases, the max over them), (2) Gate B's one-step test (no shaper, the
    design_gates.run_servo PD) on the CAD plant, at the design cadence and
    with the swing slowed. Rule (design-stage gates): speed >= 2x, torque >= 1.5x."""
    import mujoco
    p = DesignParams()
    x50, x15 = cad_plant(74.5), cad_plant(55.0)
    JN = DG.JN
    print("== (1) peak demand in the deploy-model walk, max over the 4 gate cases, CAD-inertial plant")
    for vl, xml, sn, gait, run in (("A  design: STS3250 rolls+knees", x50, "3250", {}, {}),
                                   ("C8 3215, P coef x4 rolls + knees", x15, "3215_p4rk", {}, {}),
                                   ("C7 3215, P coef x3 rolls + knees", x15, "3215_p3rk", {}, {}),
                                   ("B  STS3215 everywhere (falls: peaks up to the fall)", x15, "3215", {}, {})):
        tq = np.zeros(12); qd = np.zeros(12); stall = w0 = None; oks = []
        for cl, ckw, tkw in CASES4:
            r = walk_one(p, xml, sn, gait, dict(run, **ckw), tkw, track=True)
            tq = np.maximum(tq, np.array(r["peak_tau"])[:12]); qd = np.maximum(qd, np.array(r["peak_qd"])[:12])
            stall = np.array(r["stall"])[:12]; w0 = np.array(r["w0"])[:12]; oks.append(r["ok"])
        print(f"-- {vl}: {sum(oks)}/4 cases OK")
        print(f"     {'joint':14s} {'servo':>10s} {'tau_pk':>7s} {'stall':>6s} {'torque x':>8s} {'qd_pk':>6s} {'w0':>5s} {'speed x':>8s}")
        for i, n in enumerate(JN):
            if n.startswith("R_"):
                continue      # L and R are mirror images over a turn set; print the worse of the two
            j = JN.index("R_" + n[2:])
            tqi, qdi = max(tq[i], tq[j]), max(qd[i], qd[j])
            sv = (SETS[sn].get(n) or "sts3215")
            flag = "  <--" if (stall[i] / tqi < 1.5 or w0[i] / qdi < 2.0) else ""
            print(f"     {n[2:]:14s} {sv:>10s} {tqi:7.2f} {stall[i]:6.2f} {stall[i] / tqi:8.1f} {qdi:6.2f} {w0[i]:5.2f} {w0[i] / qdi:8.1f}{flag}")
    print("\n== (2) Gate B one-step test (design_gates.step_timeline / run_servo: PD kp 25, no shaper) on the CAD-inertial plant")
    y_lift = DG.gate_a(p, verbose=False)["A2"]["y_lift"]
    for vl, xml, pj, scale in (("A  design (STS3250 rolls+knees), cadence x1", x50, SETS["3250"], 1.0),
                               ("B  STS3215 everywhere, cadence x1", x15, None, 1.0),
                               ("B  STS3215 everywhere, segments x1.33 (swing 1.6 s)", x15, None, 4 / 3),
                               ("B  STS3215 everywhere, segments x1.5 (swing 1.8 s)", x15, None, 1.5),
                               ("B  STS3215 everywhere, segments x1.67 (swing 2.0 s)", x15, None, 5 / 3)):
        m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)
        tl = DG.step_timeline(p, y_lift, scale=scale)
        r = DG.run_servo(p, tl, servo="sts3215", per_joint=pj, model_data=(m, d))
        short = [JN[i] for i in range(12) if r["speed_margin"][i] < 2.0 or r["torque_margin"][i] < 1.5 or r["sat"][i] > 0 or r["err_pk_deg"][i] > 5.0]
        ok = (not r["fell"]) and not short and r["clear_max"] >= 0.015 and r["t_air"] >= 0.3
        print(f"-- {vl}: step {tl.T:.1f} s, {'FELL' if r['fell'] else 'up'}, swing peak {1e3 * r['clear_max']:.0f} mm for {r['t_air']:.2f} s  "
              f"=> {'PASS' if ok else 'FAIL'} (short: {short})")
        for i in (1, 3, 5, 7, 9, 11):
            print(f"     {JN[i]:14s} w_pk {r['qd_pk'][i]:5.2f}  speed x {r['speed_margin'][i]:5.1f}   tau_pk {r['tau_pk'][i]:5.2f}  torque x {r['torque_margin'][i]:5.1f}")


# ------------------------------------------------------------------ arms (as drawn)
def mode_arms():
    """the as-drawn robot with the elbowed arms held at 15 deg (lumped plant,
    getup_v6_shoulder r6_asdrawn: 2.10 kg with 55 g servos everywhere -- i.e.
    the all-STS3215 mass; the STS3250 rows run on the same masses, which
    flatters them by 117 g, as round 5b did)."""
    import getup_v6_shoulder as S
    base = dataclasses.replace(S.BASE, **S.CONFIGS["r6_asdrawn"])
    xml = os.path.join(TMP, f"no3250_arms_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(base))
    print(f"== Gate D, 4 cases x arm contacts, as-drawn robot with arms (r6_asdrawn, lumped, {plant_mass(xml):.3f} kg), arms held shoulder +15 / elbow 0")
    jobs = []
    # a fifth case: the -15 turn at the nominal mu 0.7, to tell the sticky-foot
    # knife edge at mu 0.9 (design doc 11.3) from a turning failure
    cases = CASES4 + (("turn -15 mu 0.7 play 3", {}, dict(turn_deg=-15.0)),)
    for sn, run in (("3250", {}), ("3215", {}), ("3215_p3rk", {}), ("3215_p4rk", {})):
        for cl, ckw, tkw in cases:
            jobs.append((sn, cl, xml, ckw, tkw))
    for (sn, cl), r in zip([(j[0], j[1]) for j in jobs], pmap(_arms_job, jobs)):
        print(f"{sn:8s} {cl}  {'FELL' if r['fell'] else 'up  '} tilt_max {r['tilt_max']:5.1f} deg  arm-vs-leg contacts {r['arm_leg_contacts']:6d}", flush=True)


def _arms_job(args):
    import getup_v6_shoulder as S
    sn, cl, xml, ckw, tkw = args
    base = dataclasses.replace(S.BASE, **S.CONFIGS["r6_asdrawn"])
    return S.walk_with_arm_contacts(base, xml, per_joint=SETS[sn] or None, arm_pose=(15.0, 0.0), **ckw, **tkw)


# ------------------------------------------------------------------ get-up (as drawn)
def mode_getup():
    """the as-drawn get-up (getup_v6_shoulder round 6, r6_asdrawn_rom120,
    tuck hip -120 knee -130): the same 12-variant seat-push grid and the six
    robustness conditions, per servo set, with the peak |torque| of EVERY
    joint tracked over every control step; then the prone roll (legs only,
    design doc section 12.1) on the same plant."""
    import getup_v6_shoulder as S
    S.H, S.K = -120.0, -130.0
    name = "r6_asdrawn_rom120"
    conds = [dict(play_deg=3.0, mu=0.7, servo_scale=1.0), dict(play_deg=5.0, mu=0.7, servo_scale=1.0),
             dict(play_deg=3.0, mu=0.3, servo_scale=1.0), dict(play_deg=3.0, mu=1.0, servo_scale=1.0),
             dict(play_deg=3.0, mu=0.7, servo_scale=0.8), dict(play_deg=3.0, mu=0.7, servo_scale=0.65)]
    grid = list(itertools.product(((60, -60), (90, -90)), ((20, -20), (0, 0), (30, 0)), (-40, -25)))
    print(f"== GET-UP, as drawn ({name}: arms, girdle, CAD link masses, hip ROM -120; lumped plant, 55 g servos everywhere), "
          f"tuck hip {S.H:+.0f} knee {S.K:+.0f}; stall at 11.1 V: STS3215 {DG.SERVOS['sts3215']['stall']:.2f} N*m, "
          f"STS3250 {DG.SERVOS['sts3250']['stall']:.2f} N*m")
    for sn in ("3250", "3215", "3215_p3rk", "3215_p4rk"):
        jobs = [(sn, name, g) for g in grid]
        res = pmap(_getup_grid_job, jobs)
        winners = [g for g, (ok, _) in zip(grid, res) if ok]
        print(f"-- servo set {sn}: {len(winners)}/{len(grid)} seat-push variants stand: {winners}", flush=True)
        if not winners:      # where it stops: the per-keyframe trail of every variant (up-z / pelvis z / contacts)
            for g, (_, trail) in zip(grid, res):
                print(f"   {g}: {trail}", flush=True)
        rj = pmap(_getup_robust_job, [(sn, name, g, c) for g in winners for c in conds])
        k = 0
        for g in winners:
            peaks = {}; n_ok = 0
            for c in conds:
                ok, pk = rj[k]; k += 1
                n_ok += ok
                for jn, v in pk.items():
                    peaks[jn] = max(peaks.get(jn, 0.0), v)
            print(f"   robust {g}: {n_ok}/6", flush=True)
            _print_peaks(sn, peaks)
    print("\n== PRONE -> SUPINE leg roll (design doc 12.1: R leg yawed 45 out, rolled, flexed; push + L swing; "
          "L presses / R up and back), arms folded up (shoulder 180), same plant; success = on the back (front_z > 0.7)")
    for sn in ("3250", "3215", "3215_p4rk"):
        for c in conds:
            ok, front, peaks = _prone_roll(sn, name, c)
            worst = max(peaks.items(), key=lambda kv: kv[1])
            print(f"   {sn:9s} play {c['play_deg']:.0f} mu {c['mu']:.1f} servo {c['servo_scale']:.2f}  "
                  f"{'ON BACK' if ok else 'no     '} front {front:+.2f}  peak |tau| {worst[1]:.2f} N*m ({worst[0]})  "
                  f"hip_roll {max(peaks['L_hip_roll'], peaks['R_hip_roll']):.2f}  knee {max(peaks['L_knee'], peaks['R_knee']):.2f}", flush=True)


def _print_peaks(sn, peaks):
    parts = []
    for role in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll", "shoulder", "elbow"):
        v = max(peaks.get(f"L_{role}", 0.0), peaks.get(f"R_{role}", 0.0))
        sv = SETS[sn].get(f"L_{role}", "sts3215")
        stall = DG.SERVOS[sv]["stall"]
        parts.append(f"{role} {v:.2f} ({100 * v / stall:.0f} % of {stall:.2f})")
    print("      peak |tau| over the 6 conditions: " + ", ".join(parts), flush=True)


def _getup_grid_job(args):
    import getup_v6_shoulder as S
    import getup_v6 as G
    sn, name, ((s0, e0), (s1, e1), a_push) = args
    S.H, S.K = -120.0, -130.0
    G.PJ_DEFAULT = SETS[sn]
    p, xml = S.plant(name)
    r = G.run_sequence(p, xml, S.seat_push(False, s0, s1, 2.0, a_push, elbow=(e0, e1)), start="supine",
                       play_deg=3.0, per_joint=SETS[sn] or None, verbose=False)
    trail = " | ".join(f"{L[0][:11]}:{L[1]:+.2f}/{L[2]:.2f}" for L in r["log"][4:])
    return bool(r["ok"]), trail


def _getup_robust_job(args):
    import getup_v6_shoulder as S
    import getup_v6 as G
    sn, name, ((s0, e0), (s1, e1), a_push), c = args
    S.H, S.K = -120.0, -130.0
    G.PJ_DEFAULT = SETS[sn]          # run_traced reads the servo set from here
    p, xml = S.plant(name)
    track = tuple(DG.JN) + ("L_shoulder", "R_shoulder", "L_elbow", "R_elbow")
    rt = S.run_traced(p, xml, S.seat_push(False, s0, s1, 2.0, a_push, elbow=(e0, e1)), start="supine", track=track, **c)
    return bool(rt["ok"]), rt["peak"]


def _prone_roll(sn, name, c):
    """design doc 12.1's roll, arms folded; peak |torque| tracked on every leg joint."""
    import getup_v6_shoulder as S
    import getup_v6 as G
    G.PJ_DEFAULT = SETS[sn]
    p, xml = S.plant(name)
    fold = dict(shoulder=180, elbow=0)
    abd = 45.0
    seq = [("lie prone", dict(**fold), 0.5, 0.8),
           ("R leg out", dict(R_hip_yaw=45, R_hip_roll=-abd, R_hip_pitch=-100, R_knee=-130, R_ankle=0, **fold), 1.5, 0.8),
           ("R push + L swing -> side", dict(R_hip_yaw=45, R_hip_roll=-abd, R_hip_pitch=-20, R_knee=-20, L_hip_roll=-45, L_hip_pitch=-60, L_knee=-90, **fold), 1.5, 1.2),
           ("legs neutral", dict(**fold), 1.5, 1.0),
           ("L presses, R up+back -> back", dict(R_hip_pitch=60, R_hip_roll=-30, L_hip_roll=30, **fold), 1.0, 2.0),
           ("legs neutral (supine)", dict(**fold), 1.5, 1.5)]
    rt = S.run_traced(p, xml, seq, start="prone", track=tuple(DG.JN), **c)
    # run_traced returns ok for STANDING; the roll's success is "on the back": re-read front_z
    return _front_ok(p, xml, seq, c), _front_ok.front, rt["peak"]


# ------------------------------------------------------------------ mixed
HIP_ROLLS = ("L_hip_roll", "R_hip_roll")
ANKLE_ROLLS = ("L_ankle_roll", "R_ankle_roll")
MIX_PLACEMENTS = [  # (label, joints given an STS3250)
    ("0 STS3250 (all six tuned 3215)", ()),
    ("2 STS3250: knees", KNEES),
    ("2 STS3250: hip rolls", HIP_ROLLS),
    ("2 STS3250: ankle rolls", ANKLE_ROLLS),
    ("4 STS3250: all four rolls", ROLLS),
    ("4 STS3250: hip rolls + knees", HIP_ROLLS + KNEES),
]
MIX_TUNED = (("sts3215_p35", "3.5x"), ("sts3215_p3", "3.0x"), ("sts3215_p28", "2.8x"))


def _mix_name(kp, j50):
    return f"mix_{kp}_{len(j50)}_{'-'.join(j50) or 'none'}"


# registered at import: pool workers re-import this module and need the sets
for _tuned, _kp in MIX_TUNED:
    for _lab, _j50 in MIX_PLACEMENTS:
        SETS[_mix_name(_kp, _j50)] = {j: ("sts3250" if j in _j50 else _tuned) for j in ROLLS + KNEES}
SETS["6"] = SETS["3250"]


def mode_mixed():
    """Tom 2026-10-02: given the bench (a tuned STS3215 is stable to ~3.5x,
    not 4x), does the STS3250 still matter, and do all six joints need it or
    would 2 or 4 do? Every joint not given an STS3250 is an STS3215 tuned to
    the bench's stable stiffness (kp_scale 3.5, and 3.0 for margin); the
    STS3250 keeps the model's third-party 4x. Gate D's four cases, at the
    cases' own roll play (3 deg; 5 in one case) and at 1 deg everywhere.
    The plant builder takes one servo mass for all six places, so a MIXED set
    runs on both the 55 g and the 74.5 g plant (it brackets the true mass)."""
    p = DesignParams()
    arms = os.environ.get("MIX_ARMS", "1") != "0"     # 0: section 14's no-arms CAD plant
    x50, x15 = cad_plant(74.5, arms=arms), cad_plant(55.0, arms=arms)
    print(f"   plant: CAD-inertial, {'WITH the as-drawn arms' if arms else 'no arms (as in section 14)'}")
    variants = [("6 STS3250 (the design)", "6", x50, "74.5 g")]
    only = os.environ.get("MIX_KP")                      # e.g. "2.8x": just that level
    for tuned, kp in MIX_TUNED:
        if only and kp not in only.split(","):
            continue
        for lab, j50 in MIX_PLACEMENTS:
            masses = [(x15, "55 g")] if not j50 else [(x15, "55 g"), (x50, "74.5 g")]
            for xml, mtag in masses:
                variants.append((f"{lab}, others {kp}", _mix_name(kp, j50), xml, mtag))
    plays = (("roll play per case (3 / 5 deg)", {}), ("roll play 1 deg", dict(play_deg=1.0)))
    print("== Gate D, 4 cases (8 steps, step 6 cm, lift 4 cm, design cadence), CAD-inertial plant, deploy servo model")
    print(f"   plants: 55 g servos {plant_mass(x15):.3f} kg, 74.5 g {plant_mass(x50):.3f} kg; "
          f"tuned STS3215 = kp_scale 3.5 (bench P 128 / D 32) or 3.0; STS3250 = kp_scale 4 (model)\n")
    jobs = []
    for vl, sn, xml, mtag in variants:
        for pl, pkw in plays:
            for cl, ckw, tkw in CASES4:
                jobs.append((f"{vl} [{mtag}] | {pl} | {cl}", p, xml, sn, {}, dict(ckw, **pkw), tkw))
    res = pmap(_walk_job, jobs)
    summary = {}
    for label, r in res:
        v, pl, cl = (t.strip() for t in label.split("|"))
        e = summary.setdefault((v, pl), [0, 0, []])
        e[0] += (not r["fell"]) and r["steps_completed"] == r["n_steps"]
        e[1] += bool(r["ok"])
        e[2].append(r)
        print(SG.fmt_row(f"{v[:52]:52s} | {pl[:14]:14s} | {cl}", r), flush=True)
    print("\n== summary: stayed up 8/8 / OK (clearance >= 15 mm, >= 0.3 s airborne), of 4 cases")
    for (v, pl), (up, ok, rs) in summary.items():
        print(f"   {v:62s} | {pl:31s} | up {up}/4  OK {ok}/4")



def mode_mixsweep():
    """the full adversity matrix for the sets that decide Tom's 2026-10-02
    question: the design, all six tuned 3215 at the bench's 3.5x and 3.0x,
    and STS3250s on the knees only. MIX_ARMS=0 for section 14's no-arms plant."""
    arms = os.environ.get("MIX_ARMS", "1") != "0"
    print(f"   plant: CAD-inertial, {'WITH the as-drawn arms' if arms else 'no arms (as in section 14)'}")
    if os.environ.get("MIX_KP") == "2.8x-rolls":     # which rolls carry the gain
        mode_sweep(lambda x50, x15: [
            ("M7 2 STS3250 at the hip rolls, others 2.8x (55 g plant)", x15, _mix_name("2.8x", HIP_ROLLS), {}),
            ("M7b the same on the 74.5 g plant", x50, _mix_name("2.8x", HIP_ROLLS), {}),
            ("M8 2 STS3250 at the ankle rolls, others 2.8x (55 g plant)", x15, _mix_name("2.8x", ANKLE_ROLLS), {}),
        ], arms=arms)
        return
    if os.environ.get("MIX_KP") == "2.8x":
        mode_sweep(lambda x50, x15: [
            ("A  design: 6 STS3250", x50, "3250", {}),
            ("M4 0 STS3250, all six tuned 3215 at 2.8x (bench P 96 / D 0)", x15, _mix_name("2.8x", ()), {}),
            ("M5 2 STS3250 at the knees, others 2.8x (55 g plant)", x15, _mix_name("2.8x", KNEES), {}),
            ("M6 4 STS3250 at the rolls, knees 2.8x (55 g plant)", x15, _mix_name("2.8x", ROLLS), {}),
        ], arms=arms)
        return
    mode_sweep(lambda x50, x15: [
        ("A  design: 6 STS3250", x50, "3250", {}),
        ("M1 0 STS3250, all six tuned 3215 at 3.5x", x15, _mix_name("3.5x", ()), {}),
        ("M2 0 STS3250, all six tuned 3215 at 3.0x", x15, _mix_name("3.0x", ()), {}),
        ("M3 2 STS3250 at the knees, others 3.5x (55 g plant)", x15, _mix_name("3.5x", KNEES), {}),
    ], arms=arms)


def _front_ok(p, xml, seq, c):
    import getup_v6 as G
    r = G.run_sequence(p, xml, seq, start="prone", per_joint=G.PJ_DEFAULT or None, verbose=False, **c)
    _front_ok.front = r["front"]
    return r["front"] > 0.7


if __name__ == "__main__":
    {"walk": mode_walk, "sweep": mode_sweep, "envelope": mode_envelope, "arms": mode_arms,
     "getup": mode_getup, "mixed": mode_mixed, "mixsweep": mode_mixsweep}[sys.argv[1]]()
