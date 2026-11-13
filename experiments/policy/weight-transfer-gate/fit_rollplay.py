"""Fit the roll play beyond the encoder (walker_env / env_mjx roll_play_deg) to the 2026-11-11 clamp
session (issue #277; hw_sessions/2026-11-11/m277/NOTES.md).

    python fit_rollplay.py [--base grade:<terms>] [--grid] [--check play_hip=..|play_ankle=..]
                           [--md out.md] [--json out.json]

The model: the free play is inside the roll servos, between the position sensor and the output
horn, so the encoder (the shaft) does not see it; the link floats inside +/-play/2 of the shaft
and is driven only at the band edges (gate_common's `play_hip` / `play_ankle` terms, walker_env
roll_play_deg).

Every test is the robot's own pose sequence (the NOTES' `pose` ticks, one_leg.CAL zeros), with the
R foot welded to the desk (roll_compliance.clamped_xml), driven open loop with the firmware's
minimum-jerk streamer over each pose's own duration:

  lift30   B_straight -> B_lift30 (s2000), 3 s hold. Robot: torso roll -0.36 -> -8.81 (8.45 deg;
           a repeat from Z_straight 8.70), stance encoders ankle roll +17 / +13 ticks (1.49 /
           1.14 deg), hip roll -5 / -4 ticks (0.44 / 0.35 deg)
  lift15   the same to the A_lift15 hip pitch / knee / ankle (s3000). Robot (A series): 7.0 deg,
           the lifted foot barely touching the desk, holding no weight
  shift    G: crouch (s3000) -> shift 1.3 x 62 mm over the clamped foot (s3000) -> L lift 30 mm
           (s4000) -> back to the shift pose (s2000). Robot: -3.30 -> -5.98 (2.68 deg), back +0.92
           (the same pose 4.22 deg apart by direction)
  rock     lift30, then a lateral push on the torso toward the stance side, ramped at 0.5 N/s
           (a free-joint force at the torso origin), until the torso has swung 9.4 deg back or the
           push reaches 12 N. Robot (H): a fingertip swung the torso 9.4 deg (-9.74 .. -0.35)
           while the stance encoders moved 2.7 (ankle) / 0.6 (hip) deg. The force is not
           measured on the robot

Reported per test: the torso roll change (gate_common.roll_of, + = toward the robot's right; tilt
toward the lifted L side is reported positive), the stance encoder changes (the shaft: link + the
play offset), the lifted foot's lowest pad (mm) and its normal load (N).
Cost (fit): squares of (tilt30 - 8.55), (ankle encoder30 - 1.3), (hip encoder30 - 0.45),
(tilt15 - 7.0), (shift tilt - 2.7), (rock: the swing beyond the encoders - 6.1), deg."""
from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gate_common as GC  # noqa: E402

POSES = {   # bus order (simfit_hw.BUS), 2026-11-11 NOTES
    "straight": "2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997",
    "lift30": "2082 1997 2125 2045 1991 1714 2666 1747 2184 2153 2108 1997",
    "lift15": "2082 1997 2125 2045 1991 1816 2461 1849 2184 2153 2108 1997",
    "crouch": "2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997",
    "shift13": "1915 1820 2511 1836 1824 1881 2363 1882 2184 2153 1941 1830",
    "shift_lift30": "1915 1820 2511 1836 1807 1685 2760 1681 2184 2153 1941 1813",
}
ROBOT = dict(tilt30=8.45, tilt30_b=8.70, enc_ankle30=1.49, enc_ankle30_b=1.14, enc_hip30=0.44,
             enc_hip30_b=0.35, tilt15=7.0, shift_tilt=2.68, shift_apart=4.22, rock_swing=9.4,
             rock_enc_ankle=2.7, rock_enc_hip=0.6)
TARGET = dict(tilt30=8.55, enc_ankle30=1.3, enc_hip30=0.45, tilt15=7.0, shift_tilt=2.7, rock_beyond=6.1)
STANCE, SWING = "R", "L"


def _ticks(name):
    return np.array([int(x) for x in POSES[name].split()], float)


def build(base, terms):
    import roll_compliance as RC
    name, _, bt = base.partition(":")
    t = dict(GC.parse_terms(bt.replace("|", ",")), **(terms or {}))
    xml = RC.clamped_xml(GC.XML, STANCE)
    try:
        env = GC.make_plant(name, terms=t, xml=xml)
    finally:
        os.unlink(xml)
    env.domain_rand = False
    env._zero_off = None
    return env


class Driver:
    """The firmware's smooth `pose` (minimum jerk from the last target) on the clamped plant,
    one control tick at a time, with the measurements of the module doc."""

    def __init__(self, env):
        import check1_oneleg as C1
        import simfit_hw as H
        import mujoco
        self.mj = mujoco
        self.env = e = env
        self.jidx = np.array([e._servo_names.index(j) for j in H.BUS])
        self.cal = np.array([C1.CAL[j] for j in H.BUS], float)
        self.tgt = e._sdefault.copy()
        self.tgt[self.jidx] = self.rad(_ticks("straight"))
        e._action_to_ctrl = lambda a: self.tgt
        e._to_servo = lambda x: x
        self.pads = GC.pad_gids(e)
        self.i_a = e._sname2i[f"{STANCE}_ankle_roll"]
        self.i_h = e._sname2i[f"{STANCE}_hip_roll"]
        self.t = 0.0
        self.log = []
        self.push = 0.0

    def rad(self, ticks):
        return (ticks - self.cal[:, 0]) * self.cal[:, 1] * (2 * math.pi / 4096)

    def enc(self, i):
        e = self.env
        q = float(e.data.qpos[e._sqpos][i])
        if getattr(e, "_rp_h", None) is not None:
            q += float(e._rp_off[i])
        return q

    def swing_load(self):
        e = self.env
        f6 = np.zeros(6)
        fz = 0.0
        for k in range(e.data.ncon):
            c = e.data.contact[k]
            if c.geom1 in self.pads[SWING] or c.geom2 in self.pads[SWING]:
                self.mj.mj_contactForce(e.model, e.data, k, f6)
                fz += abs(f6[0])
        return fz

    def tick(self):
        e = self.env
        e.data.qfrc_applied[1] = -self.push         # toward the stance (R, -y) side
        e.step(np.zeros(e._nq_act, np.float32))
        self.t += e.control_dt
        # tilt toward the lifted L side (+y lean): -roll_of
        self.log.append((self.t, -math.degrees(GC.roll_of(e.data.qpos[3:7])),
                         math.degrees(self.enc(self.i_a)), math.degrees(self.enc(self.i_h)),
                         1e3 * GC.clearance(e, self.pads)[SWING], self.swing_load(), self.push))

    def move(self, pose, dur, hold):
        import simfit_sim as SS
        a = self.tgt[self.jidx].copy()
        b = self.rad(_ticks(pose))
        n = int(round(dur / self.env.control_dt))
        for k in range(n):
            self.tgt[self.jidx] = a + SS.minjerk((k + 1) / n) * (b - a)
            self.tick()
        for _ in range(int(round(hold / self.env.control_dt))):
            self.tick()
        return self.mean(0.5)

    def mean(self, last):
        L = np.array(self.log)
        w = L[:, 0] > L[-1, 0] - last
        return dict(tilt=float(L[w, 1].mean()), enc_a=float(L[w, 2].mean()), enc_h=float(L[w, 3].mean()),
                    clear_mm=float(L[w, 4].mean()), load_n=float(L[w, 5].mean()))


def run_test(job):
    base, terms, test = job
    env = build(base, terms)
    env.reset(seed=0)
    env._zero_off = None
    d = Driver(env)
    s0 = d.move("straight", 0.5, 1.5)
    out = dict(test=test)
    if test in ("lift30", "lift15"):
        s1 = d.move(test, 2.0 if test == "lift30" else 3.0, 3.0)
        out.update(tilt=s1["tilt"] - s0["tilt"], enc_ankle=abs(s1["enc_a"] - s0["enc_a"]),
                   enc_hip=abs(s1["enc_h"] - s0["enc_h"]), clear_mm=s1["clear_mm"], load_n=s1["load_n"],
                   tilt_abs=s1["tilt"])
    elif test == "shift":
        d.move("crouch", 3.0, 2.0)
        s1 = d.move("shift13", 3.0, 2.0)
        s2 = d.move("shift_lift30", 4.0, 3.0)
        s3 = d.move("shift13", 2.0, 2.0)
        out.update(tilt=s2["tilt"] - s1["tilt"], apart=s1["tilt"] - s3["tilt"],
                   enc_ankle=abs(s2["enc_a"] - s1["enc_a"]), enc_hip=abs(s2["enc_h"] - s1["enc_h"]),
                   clear_mm=s2["clear_mm"], load_n=s2["load_n"])
    elif test == "rock":
        s1 = d.move("lift30", 2.0, 2.0)
        n0 = len(d.log)
        swing = 0.0
        while d.push < 12.0 and swing < ROBOT["rock_swing"]:
            d.push += 0.5 * env.control_dt
            d.tick()
            swing = s1["tilt"] - d.log[-1][1]
        L = np.array(d.log[n0:])
        tilt = L[:, 1]
        k_go = int(np.argmax(s1["tilt"] - tilt > 0.5))       # first 0.5 deg of swing
        out.update(swing=float(s1["tilt"] - tilt.min()), enc_ankle=float(np.ptp(L[:, 2])),
                   enc_hip=float(np.ptp(L[:, 3])), f_start=float(L[k_go, 6]), f_end=float(L[-1, 6]),
                   tilt0=s1["tilt"])
        out["beyond"] = out["swing"] - out["enc_ankle"] - out["enc_hip"]
        # the push needed per degree of swing over its middle half (N / deg): ~0 for free play
        mid = (s1["tilt"] - tilt > 0.25 * out["swing"]) & (s1["tilt"] - tilt < 0.75 * out["swing"])
        if mid.sum() > 3:
            out["n_per_deg"] = float(np.polyfit(s1["tilt"] - tilt[mid], L[mid, 6], 1)[0])
    return out


TESTS = ("lift30", "lift15", "shift", "rock")


def evaluate(base, terms, workers=4, ex=None):
    jobs = [(base, terms, t) for t in TESTS]
    if ex is None:
        with ProcessPoolExecutor(max_workers=workers) as ex2:
            res = list(ex2.map(run_test, jobs))
    else:
        res = list(ex.map(run_test, jobs))
    return {r["test"]: r for r in res}


def summary(r):
    return dict(tilt30=r["lift30"]["tilt"], enc_ankle30=r["lift30"]["enc_ankle"], enc_hip30=r["lift30"]["enc_hip"],
                tilt15=r["lift15"]["tilt"], shift_tilt=r["shift"]["tilt"], rock_beyond=r["rock"]["beyond"])


def cost(r):
    s = summary(r)
    return float(sum((s[k] - v) ** 2 for k, v in TARGET.items()))


def fmt_row(label, r):
    a, b, c, d = r["lift30"], r["lift15"], r["shift"], r["rock"]
    return (f"| {label} | {a['tilt']:.2f} ({a['enc_ankle']:.2f} / {a['enc_hip']:.2f}) | "
            f"{b['tilt']:.2f} ({b['clear_mm']:.1f} mm, {b['load_n']:.1f} N) | {c['tilt']:.2f} (apart {c['apart']:.2f}) | "
            f"{d['swing']:.2f} ({d['enc_ankle']:.2f} / {d['enc_hip']:.2f}; beyond {d['beyond']:.2f}; "
            f"push {d['f_start']:.2f} -> {d['f_end']:.2f} N, {d.get('n_per_deg', float('nan')):.3f} N/deg) | {cost(r):.2f} |")


HEAD = ("| plant | 30 mm: tilt (encoders ankle / hip), deg | 15 mm: tilt (L foot lowest pad, load) | "
        "shifted 81 mm lift: tilt (shift pose before vs after) | rock: swing (encoder ranges; beyond; push) | cost |\n"
        "|---|---|---|---|---|---|\n"
        f"| robot | {ROBOT['tilt30']}-{ROBOT['tilt30_b']} ({ROBOT['enc_ankle30_b']}-{ROBOT['enc_ankle30']} / "
        f"{ROBOT['enc_hip30_b']}-{ROBOT['enc_hip30']}) | {ROBOT['tilt15']} (barely touching, no weight) | "
        f"{ROBOT['shift_tilt']} (apart {ROBOT['shift_apart']}) | {ROBOT['rock_swing']} ({ROBOT['rock_enc_ankle']} / "
        f"{ROBOT['rock_enc_hip']}; beyond ~6.1; fingertip, not measured) | |")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="grade:" + GC.MASS_SYM.replace(",", "|"))
    ap.add_argument("--grid", default="", help="'ankle=a,b,c;hip=d,e,f' play grid (deg peak to peak)")
    ap.add_argument("--check", action="append", default=[], help="terms 'play_hip=..|play_ankle=..|lash=..'")
    ap.add_argument("--md", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("WORKERS", "16")))
    a = ap.parse_args(argv)
    out = {"base": a.base, "rows": []}
    L = [f"Roll play fit on `{a.base}`, R foot clamped, L lifted (2026-11-11 NOTES poses)", "", HEAD]
    combos = [GC.parse_terms(c.replace("|", ",")) for c in a.check]
    if a.grid:
        g = dict(tok.split("=") for tok in a.grid.split(";"))
        av = [float(x) for x in g["ankle"].split(",")]
        hv = [float(x) for x in g["hip"].split(",")]
        combos += [dict(play_ankle=x, play_hip=y) for x, y in itertools.product(av, hv)]
    jobs = [(a.base, t, test) for t in combos for test in TESTS]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        res = list(ex.map(run_test, jobs))
    for k, t in enumerate(combos):
        r = {x["test"]: x for x in res[4 * k:4 * k + 4]}
        out["rows"].append(dict(terms=t, res=r, cost=cost(r), summary=summary(r)))
    rows = sorted(out["rows"], key=lambda x: x["cost"]) if a.grid else out["rows"]
    for row in rows:
        L.append(fmt_row(", ".join(f"{k}={v:g}" if isinstance(v, float) else k for k, v in row["terms"].items())
                         or "(no term)", row["res"]))
    md = "\n".join(L)
    print(md)
    if a.md:
        open(a.md, "w").write(md + "\n")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
