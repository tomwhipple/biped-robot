"""Failure-mode search for the open-loop (kinematic) walk on the robot's plant.

Randomly samples the conditions the robot could meet -- floor friction and
tilt, roll-joint play and gear backlash, servo strength and stiffness, the
actuation lag and dead time, mass and payload errors -- and the gait itself
(step length, lift height, swing / shift timing, turn per step), walks
static_gait's kinematic gait on the committed plant (sim/bimo_biped_v6ar.xml:
the robot as drawn, its servo set, the arms held at their rest pose), and
classifies every failure. The point is to find HOW the walk fails, not to
grade it: a gate that passes says nothing about the cases nobody listed.

    .venv/bin/python sim/failure_sweep.py --n 2000 --out sim/runs/failure_sweep/<tag>
    .venv/bin/python sim/failure_sweep.py --until 07:00 --out ...     # keep sampling until then
    .venv/bin/python sim/failure_sweep.py --report <out>              # summary.md from results.csv

Writes <out>/results.csv (one row per walk, appended as results arrive, so a
run that is stopped keeps everything it did) and <out>/summary.md (failure
rates by category, and how each sampled parameter moves the failure rate).

Failure categories (a walk can carry several):
    fell            torso tilt past the env's fall test before the last step
    no_lift         a completed step whose swing sole peaked under 15 mm
    short_air       a completed step airborne for under 0.3 s
    slip            the stance foot slid more than 10 mm during a swing
    low_margin      the CoM margin over the stance sole went under 5 mm
    torque_sat      a joint's peak torque reached 95 % of its stall
    speed_sat       a joint's peak speed reached 95 % of its no-load speed
    heading         a straight walk drifted more than 10 deg in heading
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import sys
import time

import numpy as np

# before anything imports mujoco (static_gait sets egl, which only Linux has)
os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))

PLANT = os.path.join(HERE, "bimo_biped_v6ar.xml")

# sampled ranges: (low, high) uniform, or a tuple of choices
SPACE = {
    "mu": (0.3, 1.2),
    "play_deg": (0.0, 6.0),
    "backlash_deg": (0.0, 3.0),
    "servo_scale": (0.65, 1.0),
    "k_tuned": (2.0, 3.6),            # tuned STS3215 stiffness (bench: P 96 ~ 2.8x, P 128 ~ 3.5x)
    "k_3250": (3.0, 4.0),             # STS3250 stiffness credit (model: 4x)
    "mass_scale": (0.9, 1.15),
    "payload": (0.0, 0.12),
    "tilt_fore_deg": (-3.0, 3.0),
    "lag_hz": (0.0, 1.0, 2.0, 3.0),
    "delay_ticks": (0, 2, 4, 6, 8),
    "step": (0.03, 0.10),
    "lift_h": (0.02, 0.06),
    "t_swing": (0.8, 2.0),
    "t_shift": (0.8, 2.0),
    "turn_deg": (-20.0, 20.0),
    "straight": (True, False),        # half the walks go straight (turn 0)
}
COLUMNS = (["idx", "seed"] + list(SPACE) +
           ["fell", "t_fell", "steps_completed", "n_steps", "min_clear_mm", "min_t_air", "min_margin_mm",
            "slip_mm", "tilt_max", "heading_deg", "x_final", "y_final", "tau_frac", "qd_frac", "worst_tau_joint",
            "failures", "secs"])


def sample(rng):
    s = {}
    for k, v in SPACE.items():
        if len(v) == 2 and all(isinstance(x, float) for x in v):
            s[k] = float(rng.uniform(*v))
        else:
            s[k] = v[rng.integers(len(v))]
    if s["straight"]:
        s["turn_deg"] = 0.0
    return s


# ---------------------------------------------------------------- worker side
_W = {}


def _init_worker():
    """Import the sim stack once per process and hold the arms (and neck) at
    the plant's rest pose: static_gait pads every non-leg actuator with 0 rad
    (hanging), so the leg targets are extended with the rest pose instead."""
    import static_gait as SG
    import design_gates as DG
    import mujoco
    from gen_plant_v6 import DesignParams
    m = mujoco.MjModel.from_xml_path(PLANT)
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    rest = []
    for n in names[12:]:
        jid = m.joint(n).id
        rest.append(float(m.qpos0[m.jnt_qposadr[jid]]))     # the plant's rest pose (qpos0)
    rest = np.array(rest)
    orig = SG.q_of

    def q_with_rest(p, key):
        q = orig(p, key)
        return np.concatenate([q, rest]) if len(q) == 12 else q
    SG.q_of = q_with_rest
    _W.update(SG=SG, DG=DG, p=DesignParams(), names=names, rest=rest)


def _servo_set(s):
    DG = _W["DG"]
    b = DG.SERVOS["sts3215"]
    kt = f"sts3215_k{s['k_tuned']:.3f}"
    DG.SERVOS[kt] = dict(stall=b["stall"], w0=b["w0"], kp_scale=s["k_tuned"])
    c = DG.SERVOS["sts3250"]
    k5 = f"sts3250_k{s['k_3250']:.3f}"
    DG.SERVOS[k5] = dict(stall=c["stall"], w0=c["w0"], kp_scale=s["k_3250"])
    per = {j: kt for j in ("L_ankle_roll", "R_ankle_roll", "L_knee", "R_knee")}
    per.update({j: k5 for j in ("L_hip_roll", "R_hip_roll")})
    return per


def classify(r, s):
    f = []
    if r["fell"]:
        f.append("fell")
    if r["steps_completed"] and r["min_clear_peak"] < 0.015:
        f.append("no_lift")
    if r["steps_completed"] and r["min_t_air"] < 0.3:
        f.append("short_air")
    if r["slip_max"] > 10.0:
        f.append("slip")
    if r["steps_completed"] and math.isfinite(r["min_margin"]) and r["min_margin"] < 0.005:
        f.append("low_margin")
    if r.get("tau_frac", 0.0) >= 0.95:
        f.append("torque_sat")
    if r.get("qd_frac", 0.0) >= 0.95:
        f.append("speed_sat")
    if s["turn_deg"] == 0.0 and abs(r["heading_deg"]) > 10.0:
        f.append("heading")
    return f


def run_one(job):
    idx, seed, s = job
    SG, p = _W["SG"], _W["p"]
    t0 = time.time()
    try:
        tl, win = SG.walk_timeline(p, n_steps=8, step=s["step"], lift_h=s["lift_h"], t_swing=s["t_swing"],
                                   t_shift=s["t_shift"], turn_deg=s["turn_deg"])
        r = SG.run_walk(p, PLANT, tl, win, seed=seed, mu=s["mu"], play_deg=s["play_deg"],
                        backlash_deg=s["backlash_deg"], mass_scale=s["mass_scale"], servo_scale=s["servo_scale"],
                        payload=s["payload"], floor_tilt_deg=s["tilt_fore_deg"], per_joint=_servo_set(s),
                        lag_hz=s["lag_hz"], delay_ticks=int(s["delay_ticks"]), track_torque=True, track_speed=True)
        tau = np.array(r["peak_tau"][:12]) / np.maximum(np.array(r["stall"][:12], float), 1e-9)
        qd = np.array(r["peak_qd"][:12]) / np.maximum(np.array(r["w0"][:12], float), 1e-9)
        r["tau_frac"], r["qd_frac"] = float(tau.max()), float(qd.max())
        r["worst_tau_joint"] = _W["names"][int(tau.argmax())]
        fails = classify(r, s)
    except Exception as e:  # noqa: BLE001  -- an unsolvable gait is a result too
        r = dict(fell=None, t_fell=None, steps_completed=0, n_steps=8, min_clear_peak=0.0, min_t_air=0.0,
                 min_margin=float("nan"), slip_max=0.0, tilt_max=0.0, heading_deg=0.0, x_final=0.0, y_final=0.0,
                 tau_frac=0.0, qd_frac=0.0, worst_tau_joint="")
        fails = [f"error:{type(e).__name__}:{str(e)[:60]}"]
    row = dict(idx=idx, seed=seed, **s, fell=r["fell"], t_fell=r["t_fell"], steps_completed=r["steps_completed"],
               n_steps=r["n_steps"], min_clear_mm=round(1e3 * r["min_clear_peak"], 1),
               min_t_air=round(r["min_t_air"], 2),
               min_margin_mm=round(1e3 * r["min_margin"], 1) if math.isfinite(r["min_margin"]) else "",
               slip_mm=r["slip_max"], tilt_max=round(r["tilt_max"], 1), heading_deg=round(r["heading_deg"], 1),
               x_final=round(r["x_final"], 3), y_final=round(r["y_final"], 3),
               tau_frac=round(r["tau_frac"], 3), qd_frac=round(r["qd_frac"], 3),
               worst_tau_joint=r["worst_tau_joint"], failures=";".join(fails), secs=round(time.time() - t0, 1))
    return row


# ---------------------------------------------------------------- driver side
def run(out, n=None, until=None, procs=None, seed0=0):
    from multiprocessing import Pool
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "results.csv")
    start = 0
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    if not new:
        with open(path) as f:
            start = max(0, sum(1 for _ in f) - 1)
    rng = np.random.default_rng(seed0 + start)
    deadline = None
    if until:
        hh, mm = (int(x) for x in until.split(":"))
        now = dt.datetime.now()
        deadline = now.replace(hour=hh, minute=mm, second=0, microsecond=0)
        if deadline <= now:
            deadline += dt.timedelta(days=1)
    procs = procs or max(1, (os.cpu_count() or 4) - 2)
    with open(path, "a", newline="") as fh, Pool(procs, initializer=_init_worker) as pool:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        idx = start
        batch = procs * 4
        while True:
            if n is not None and idx >= start + n:
                break
            if deadline and dt.datetime.now() >= deadline:
                break
            k = batch if n is None else min(batch, start + n - idx)
            jobs = [(idx + i, int(rng.integers(1 << 30)), sample(rng)) for i in range(k)]
            for row in pool.imap_unordered(run_one, jobs):
                w.writerow(row)
            fh.flush()
            idx += k
            print(f"{dt.datetime.now():%H:%M} {idx - start} walks", flush=True)
    report(out)


def report(out):
    with open(os.path.join(out, "results.csv")) as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    if not n:
        return
    fl = [r["failures"] or "" for r in rows]
    cats = ["fell", "no_lift", "short_air", "slip", "low_margin", "torque_sat", "speed_sat", "heading"]
    has = {c: np.array([c in f.split(";") for f in fl]) for c in cats}
    pct = lambda m: f"{100.0 * np.mean(m):.1f} %" if len(m) else "-"
    lines = [f"# Failure sweep: {os.path.basename(os.path.normpath(out))}", "",
             f"{n} walks of the kinematic gait (8 steps) on `sim/bimo_biped_v6ar.xml`, conditions sampled "
             f"uniformly from `sim/failure_sweep.py` SPACE. Written {dt.datetime.now():%Y-%m-%d %H:%M}.", "",
             f"- **clean** (no failure category): {pct(np.array([f == '' for f in fl]))}",
             f"- **errors** (gait not solvable): {pct(np.array([f.startswith('error') for f in fl]))}", "",
             "| category | rate |", "|---|---|"]
    lines += [f"| {c} | {pct(has[c])} |" for c in cats]
    lines += ["", "## How each parameter moves the fall and no-lift rates",
              "Rate in the lowest / highest third of each sampled range.", "",
              "| parameter | fell, low third | fell, high third | no_lift, low third | no_lift, high third |",
              "|---|---|---|---|---|"]
    for k in SPACE:
        if k == "straight":
            continue
        x = np.array([float(r[k]) for r in rows])
        lo, hi = np.quantile(x, 1 / 3), np.quantile(x, 2 / 3)
        a_, b_ = x <= lo, x >= hi
        lines.append(f"| {k} | {pct(has['fell'][a_])} | {pct(has['fell'][b_])} | "
                     f"{pct(has['no_lift'][a_])} | {pct(has['no_lift'][b_])} |")
    bad = sorted((r for r in rows if r["failures"]), key=lambda r: int(r["steps_completed"]))[:15]
    cols = ["idx", "seed", "failures", "steps_completed", "t_fell", "mu", "play_deg", "servo_scale", "k_tuned",
            "step", "lift_h", "t_swing", "turn_deg", "tilt_fore_deg"]
    fmt = lambda v: f"{float(v):.3g}" if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v
    lines += ["", "## The 15 earliest failures", "", "| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(fmt(r[c]) for c in cols) + " |" for r in bad]
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines[:20]))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "runs", "failure_sweep", f"{dt.date.today():%Y%m%d}"))
    ap.add_argument("--n", type=int, default=None, help="walks to run (default: until --until, else 200)")
    ap.add_argument("--until", default=None, help="HH:MM local time to stop sampling")
    ap.add_argument("--procs", type=int, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--report", action="store_true", help="only rewrite summary.md from results.csv")
    a = ap.parse_args(argv)
    if a.report:
        report(a.out)
        return
    n = a.n if (a.n is not None or a.until) else 200
    run(a.out, n=n, until=a.until, procs=a.procs, seed0=a.seed)


if __name__ == "__main__":
    main()
