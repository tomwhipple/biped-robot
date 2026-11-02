"""The 2026-10-30 one-leg-stand flight-recorder sessions, cut into stands and stages.

A STAND is one side of the 10-25 protocol as `regress.sh` ran it:

    shift (s3000) -> l10 (s6000 B<side>) -> lower (s2000) -> l30 (s4000 B<side>) + hold -> lower (s2500)
    [-> crouch (s3000)]

Everything is read from the session's own CSVs (tools/flight_reader.py csv): the moves (target
ticks, duration, the robot's own "streaming" stamp as the move start), the per-tick IMU roll /
gyro, the balance block (ref, err, I, u) and per joint goal / pos / load. Times are seconds from
the stand's shift command start (the "streaming" line of the shift).

    hw = load_stand("222031", "L")          # session tag, side
    hw.moves     -> [Move(name, t0, dur, ticks[12], B), ...]
    hw.ticks     -> DataFrame: t, roll, gyro_x, bal_* , <joint>.goal/.pos/.load/.goal_rad/.pos_rad
    hw.abort_t   -> t of `balance ABORT`, or None
"""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

HW_ROOT = os.environ.get("ONELEG_HW_ROOT", "/home/claw/code/robot/hw_sessions")
HW_DIR = os.environ.get("ONELEG_HW_DIR", os.path.join(HW_ROOT, "2026-10-30"))     # the 10-30 sessions
# tag -> (directory, file stem, floor, firmware balance as run: the sim Ctl fields that differ
# from the bench default).  The 10-30 firmware had no integrator cap; 11-01 ran the 3 deg cap.
_ILIM3 = dict(i_max=math.radians(3.0))
SESSIONS = {"214513": (HW_DIR, "flight_20261030T214513Z_oneleg_rec", "carpet", {}),   # R and L held; feedback reads mostly missing
            "222031": (HW_DIR, "flight_20261030T222031Z_oneleg_rec", "carpet", {}),   # R held, L aborted outward
            "222727": (HW_DIR, "flight_20261030T222727Z_oneleg_rec", "carpet", {}),   # R held, L aborted outward
            # 2026-11-01, fw ccbacba-era (3 deg I cap), clr.sh poses
            "194709": (os.path.join(HW_ROOT, "2026-11-01"), "flight_20261101T194709Z_ilim_stands", "carpet", _ILIM3),  # 3x R + 3x L, all held
            "203257": (os.path.join(HW_ROOT, "2026-11-01"), "flight_20261101T203257Z_ilim_stands", "hard", _ILIM3),    # R: 1.6 Hz oscillation, abort 7.7 s
            "203534": (os.path.join(HW_ROOT, "2026-11-01"), "flight_20261101T203534Z_ilim_stands", "hard", _ILIM3),   # R: 1.9 Hz oscillation, abort 5.7 s
            "203759": (os.path.join(HW_ROOT, "2026-11-01"), "flight_20261101T203759Z_ilim_stands", "hard", _ILIM3)}   # R, L, R, L held; R#3 oscillated, abort
# the stands each session holds: (side, rep) -- rep counts the side's 10 mm lifts in the session
STANDS = {"214513": [("R", 1), ("L", 1)], "222031": [("R", 1), ("L", 1)], "222727": [("R", 1), ("L", 1)],
          "194709": [("R", 1), ("L", 1), ("R", 2), ("L", 2), ("R", 3), ("L", 3)],
          "203257": [("R", 1)], "203534": [("R", 1)],
          "203759": [("R", 1), ("L", 1), ("R", 2), ("L", 2), ("R", 3)]}


def stand_id(tag, side, rep=1):
    return f"{tag}{side}" + (f"{rep}" if rep > 1 else "")


def parse_id(sid):
    """"222031L" -> ("222031", "L", 1); "194709R3" -> ("194709", "R", 3)"""
    m = re.fullmatch(r"(\d{6})([LR])(\d?)", sid)
    if not m:
        raise ValueError(sid)
    return m.group(1), m.group(2), int(m.group(3) or 1)


def floor_of(tag):
    return SESSIONS[tag][2]


def all_ids(floor=None):
    return [stand_id(t, s, r) for t in SESSIONS for (s, r) in STANDS[t] if floor in (None, floor_of(t))]
# `pose` bus order (one_leg.BUS)
BUS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch", "L_knee",
       "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
TICK = 2 * math.pi / 4096


@dataclass
class Move:
    name: str
    t0: float          # s from the stand start: the robot's "streaming" stamp
    dur: float         # s
    ticks: list        # 12 bus-order targets
    B: str | None      # stance side when the move carries B<side>


@dataclass
class Stand:
    session: str
    side: str
    t_utc0: float
    moves: list
    ticks: pd.DataFrame
    abort_t: float | None
    abort_err_deg: float | None = None
    refs_deg: dict = field(default_factory=dict)     # stage -> the robot's printed ref roll
    cal: dict = field(default_factory=dict)          # joint -> (zero, dir) from the session's rad columns
    rep: int = 1
    floor: str = "carpet"
    fw: dict = field(default_factory=dict)           # the balance as the firmware ran it (sim Ctl overrides)

    @property
    def id(self):
        return stand_id(self.session, self.side, self.rep)

    def stage(self, name):
        return next(m for m in self.moves if m.name == name)

    def stage_window(self, name):
        """(t0, t1) of a B stage: its move start to the next move start (or the abort)."""
        i = [m.name for m in self.moves].index(name)
        t1 = self.moves[i + 1].t0 if i + 1 < len(self.moves) else self.ticks.t.iloc[-1]
        if self.abort_t is not None and self.moves[i].t0 < self.abort_t < t1:
            t1 = self.abort_t
        return self.moves[i].t0, t1


def _path(tag, what):
    d, stem = SESSIONS[tag][:2]
    return os.path.join(d, f"{stem}_{what}.csv")


_CACHE = {}


def _events(tag):
    k = (tag, "events")
    if k not in _CACHE:
        _CACHE[k] = pd.read_csv(_path(tag, "events"), low_memory=False)
    return _CACHE[k]


def _ticks(tag):
    k = (tag, "ticks")
    if k not in _CACHE:
        _CACHE[k] = pd.read_csv(_path(tag, "ticks"), low_memory=False)
    return _CACHE[k]


def _cal_from(d):
    """(zero, dir) per joint, recovered from the session's own ticks->rad columns
    (flight_reader converts with the robot's HELLO calibration)."""
    cal = {}
    for j in BUS:
        g, r = d[f"{j}.goal"].values.astype(float), d[f"{j}.goal_rad"].values.astype(float)
        ok = np.isfinite(r) & (g != 0)
        g, r = g[ok], r[ok]
        if len(np.unique(g)) < 2:
            continue
        a, b = np.polyfit(g, r, 1)                 # r = (g - zero) * dir * TICK
        sgn = 1 if a > 0 else -1
        cal[j] = (int(round(-b / a)), sgn)
    return cal


def load_stand(tag, side=None, rep=1):
    if side is None:
        tag, side, rep = parse_id(tag)
    e = _events(tag)
    d = _ticks(tag)
    cmds = e[e.kind == "cmd"].reset_index(drop=True)
    outs = e[e.kind == "out"].reset_index(drop=True)
    stream = outs[outs.text.astype(str).str.contains("SMOOTH minimum-jerk")].t_utc.values
    poses = []
    for _, r in cmds.iterrows():
        t = str(r.text)
        if not t.startswith("pose "):
            continue
        tok = t.split()
        ticks = [int(x) for x in tok[1:13]]
        dur = int(next(x for x in tok if x.startswith("s"))[1:]) / 1000.0
        B = next((x[1] for x in tok if x.startswith("B")), None)
        # the robot's own start stamp: the first "streaming" line after the command
        ts = stream[stream >= r.t_utc]
        poses.append(dict(t_cmd=r.t_utc, t0=float(ts[0]) if len(ts) else r.t_utc, dur=dur, ticks=ticks, B=B))
    # the stand: the 10 mm B<side> lift, the shift just before it, and what follows up to the
    # crouch (the next pose whose L and R hip rolls are both back at their zeros-ish)
    il10 = [i for i, p in enumerate(poses) if p["B"] == side and abs(p["dur"] - 6.0) < 1e-6][rep - 1]
    seq = poses[il10 - 1: il10 + 5]
    names = ["shift", "l10", "lower10", "l30", "lower30", "crouch"]
    t_utc0 = seq[0]["t0"]
    moves = [Move(n, p["t0"] - t_utc0, p["dur"], p["ticks"], p["B"]) for n, p in zip(names, seq)]
    if len(seq) < 5 or not (seq[3]["B"] == side and abs(seq[3]["dur"] - 4.0) < 1e-6):
        raise ValueError(f"{tag} {side}: unexpected pose sequence")
    if abs(seq[4]["dur"] - 2.5) > 1e-6:                 # aborted: the runner went straight to the crouch
        moves = moves[:4]
        k = 4
        if abs(seq[4]["dur"] - 1.5) < 1e-6 and len(seq) > 5:   # regress.sh GUARD (|err| > 9): the shift in 1.5 s first
            moves.append(Move("guard", seq[4]["t0"] - t_utc0, seq[4]["dur"], seq[4]["ticks"], None))
            k = 5
        moves.append(Move("crouch", seq[k]["t0"] - t_utc0, seq[k]["dur"], seq[k]["ticks"], None))
    t_end = t_utc0 + moves[-1].t0 + moves[-1].dur + 1.0
    # an abort cuts the protocol short: the runner then sends the crouch
    ab = outs[outs.text.astype(str).str.contains("balance ABORT") & (outs.t_utc > t_utc0) & (outs.t_utc < t_end)]
    abort_t = float(ab.t_utc.iloc[0]) - t_utc0 if len(ab) else None
    abort_err = None
    if len(ab):
        m = re.search(r"roll err ([+-][0-9.]+) deg", ab.text.iloc[0])
        abort_err = float(m.group(1)) if m else None
    refs = {}
    for _, r in outs[outs.text.astype(str).str.contains("balance ON") & (outs.t_utc > t_utc0) & (outs.t_utc < t_end)].iterrows():
        m = re.search(r"ref roll ([+-][0-9.]+) deg", r.text)
        st = next((mv.name for mv in moves if mv.B and abs(mv.t0 - (r.t_utc - t_utc0)) < 0.2), None)
        if m and st:
            refs[st] = float(m.group(1))
    w = d[(d.t_utc >= t_utc0 - 1.0) & (d.t_utc <= t_end)].copy()
    w["t"] = w.t_utc - t_utc0
    for j in BUS:
        bad = w[f"{j}.pos"].values == 0                 # 0 = no feedback that tick
        w.loc[bad, f"{j}.pos"] = np.nan
        w.loc[bad, f"{j}.pos_rad"] = np.nan
        w.loc[bad, f"{j}.load"] = np.nan
    return Stand(tag, side, t_utc0, moves, w.reset_index(drop=True), abort_t, abort_err, refs, _cal_from(d),
                 rep=rep, floor=floor_of(tag), fw=dict(SESSIONS[tag][3]))


def all_stands(floor=None):
    return [load_stand(i) for i in all_ids(floor)]


def summary(st: Stand):
    """The per-stage numbers the fit compares (degrees, seconds from the stage start)."""
    R = 180 / math.pi
    out = {}
    for name in ("l10", "l30"):
        if name not in [m.name for m in st.moves]:
            continue
        t0, t1 = st.stage_window(name)
        w = st.ticks[(st.ticks.t >= t0) & (st.ticks.t <= t1) & (st.ticks.bal_state != 0)]
        if not len(w):
            continue
        tt = w.t.values - t0
        err = w.bal_err.values * R
        u = w.bal_u.values * R
        dur = st.stage(name).dur
        k_end = min(np.searchsorted(tt, dur), len(tt) - 1)
        sgn = 1.0 if st.side == "L" else -1.0              # + = toward the lifted foot / outward push
        o = dict(err_end_lift=sgn * err[k_end], u_end_lift=sgn * u[k_end],
                 err_peak_in=float(np.max(sgn * err)), t_peak_in=float(tt[np.argmax(sgn * err)]),
                 u_peak=float(np.max(sgn * u)), u_end=sgn * u[-1], err_end=sgn * err[-1],
                 aborted=bool(st.abort_t is not None and t0 < st.abort_t <= t1 + 1e-6),
                 t_abort=(st.abort_t - t0) if st.abort_t is not None and t0 < st.abort_t <= t1 + 1e-6 else None,
                 ref=st.refs_deg.get(name))
        out[name] = o
    return out


if __name__ == "__main__":
    for st in all_stands():
        print(st.id, st.floor, "abort", st.abort_t, st.abort_err_deg, "refs", st.refs_deg)
        print("   moves", [(m.name, round(m.t0, 2), m.dur, m.B) for m in st.moves])
        for k, v in summary(st).items():
            print("  ", k, {a: (round(b, 2) if isinstance(b, float) else b) for a, b in v.items()})


def protocol_moves(side, tag="222031"):
    """The full 10-25 protocol for `side` (shift, l10, lower10, l30 + 12 s hold, lower30, crouch) on
    the recorded timing of a stand that ran to the end (the same session's R stand), with the
    side's own pose ticks. For controller studies: an aborted recording lacks the tail."""
    ref = load_stand(tag, "R")
    own = load_stand(tag, side)
    shift_ticks = own.stage("shift").ticks
    moves = []
    for m in ref.moves:
        if m.name in ("shift", "l10", "lower10", "l30"):
            src = own.stage(m.name)
            moves.append(Move(m.name, m.t0, m.dur, src.ticks, side if m.B else None))
        elif m.name == "lower30":
            moves.append(Move("lower30", m.t0, m.dur, shift_ticks, None))
        else:
            moves.append(Move("crouch", m.t0, m.dur, ref.stage("crouch").ticks, None))
    return own, moves
