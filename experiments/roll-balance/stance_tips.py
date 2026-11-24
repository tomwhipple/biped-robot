#!/usr/bin/env python3
"""Inward stance tips by stance index and timing, from the track logs and the flight record (#297).

    python stance_tips.py STAMP [STAMP ...]            # the runs track_<STAMP>.jsonl (local time, as track_host names them)
    python stance_tips.py --day 20261116 --from 1800 --to 2010

One row per single support (results/stance_tips/<first>_<last>.csv) and a summary by timing (step period from the
track's TS lines: under 3.7 s = "fast", else "normal") x stance side x stance index (the k-th single support on that
side in the walk; "early" = k <= 4, "late" = k >= 5). A tip is the track's own rule: lean toward the swing foot >= 7 deg.

What the 2026-11-24 record (docs/design-v6/2026-11-24-left-stance-tips.md) found and this re-measures:
  e_05     the roll toward the swing foot 0.5 s after lift-off (the foot-off sag rising; tips sag a little LESS here)
  gyro_06  the roll rate at 0.6 s, deg/s, + toward the swing foot (clean steps are already swinging back, about -16)
  e_09     the roll at 0.9 s (the rebound: clean about -1.5 to -3, a tip has not come back, about +4)
  beyond_06 / beyond_08  the roll beyond the stance hip + ankle roll encoders (the play / sole edge), deg
  T_lift   the firmware's learned trim T of the stance side at lift-off (flight record), deg
  ld_st_hip, ld_st_ank   the stance hip roll / ankle roll servo load, mean over 0.3-0.5 s after lift-off (the raise,
           before any tip diverges; STS3215 load counts, +-1000 = full torque)
  ld_sw_hip  the swing hip roll load, mean over the 0.25 s before lift-off (the push between the feet in double support)
On 2026-11-16 the three loads built up over each normal-timing walk (stance k <= 4 -> k >= 5) and were largest on the
tips; see the record.
The robot side is read straight from the newest flight record created before each run (a window per run, nothing
cached); memory stays small (one run's ticks at a time).
"""
import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
import statistics
import struct
import sys
from zoneinfo import ZoneInfo

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(HERE, "results", "stance_tips")
LOCAL = ZoneInfo("America/Denver")
FREC_MAGIC = b"FRECSESS1\n"
REC = struct.Struct("<HQ")
TIP_DEG = 7.0
FAST_PERIOD_S = 3.7
LATE_K = 5
TS_RE = re.compile(r"TS (\d+) ([LR]) in ([\d.]+) out ([\d.]+) e ([+-][\d.]+) T ([+-][\d.]+) hike ([+-][\d.]+) "
                   r"(ok|ABORT)( Tback)?(?: G (\d+))?")
# balance::TrimState codes in the record's balance block
OFF, CLOSED, HOLD, FADE = 0, 5, 6, 7


# ------------------------------------------------------------------------------------------------
# pure parts (tested in tests/test_stance_tips.py)
# ------------------------------------------------------------------------------------------------
def parse_ts(rows):
    """The TS lines of a track log: {n: dict(t=first host time seen, side, lean_in, lean_out, e, T, hike, ok, tback,
    gate)} and the median spacing of their first appearance (the step period, s; None under 3 lines)."""
    first = {}
    for r in rows:
        for m in TS_RE.finditer(r.get("text") or ""):
            n = int(m.group(1))
            if n in first:
                continue
            first[n] = dict(t=r.get("t", 0.0), side=m.group(2), lean_in=float(m.group(3)), lean_out=float(m.group(4)),
                            e=float(m.group(5)), T=float(m.group(6)), hike=float(m.group(7)), ok=m.group(8) == "ok",
                            tback=bool(m.group(9)), gate=int(m.group(10)) if m.group(10) else None)
    ns = sorted(first)
    ts = [first[n]["t"] for n in ns]
    period = statistics.median(b - a for a, b in zip(ts, ts[1:])) if len(ts) > 2 else None
    return first, period


def chains(state):
    """Single supports from the trim's state per tick: [(i_lift, i_hold or None, i_touchdown or None)].
    Lift-off = the tick CLOSED begins (from OFF or FADE); touchdown = the first FADE after it."""
    out, cur = [], None
    for i in range(1, len(state)):
        a, b = int(state[i - 1]), int(state[i])
        if b == CLOSED and a in (OFF, FADE):
            cur = [i, None, None]
            out.append(cur)
        elif cur is None:
            continue
        elif a == CLOSED and b == HOLD:
            cur[1] = i
        elif a in (CLOSED, HOLD) and b == FADE:
            cur[2] = i
            cur = None
    return [tuple(c) for c in out]


def step_metrics(t, e_deg, gyro_dps, enc_deg, t_lift, sign):
    """The per-step numbers on one single support. t: s; e_deg: the trim's error (roll - its reference, deg, the
    record's sign); gyro_dps: body roll rate (deg/s); enc_deg: stance hip roll + ankle roll encoders (deg, joint
    frame); sign: +1 left stance, -1 right. Returned values are toward the swing foot (+)."""
    def at(y, dt_):
        return float(np.interp(t_lift + dt_, t, y))
    e = lambda d: sign * at(e_deg, d)  # noqa: E731
    enc0 = at(enc_deg, 0.0)
    beyond = lambda d: e(d) + sign * (at(enc_deg, d) - enc0)  # noqa: E731  imu minus kin (footoff_sag.step_traces)
    return dict(e_05=e(0.5), gyro_06=sign * at(gyro_dps, 0.6), e_09=e(0.9), beyond_06=beyond(0.6),
                beyond_08=beyond(0.8))


def window_mean(t, y, t_lift, a, b):
    """Mean of y over t_lift + [a, b] (NaN when no finite sample)."""
    m = (t >= t_lift + a) & (t <= t_lift + b) & np.isfinite(y)
    return float(np.mean(y[m])) if m.any() else float("nan")


def summarize(steps):
    """{(timing, side, 'early'|'late'): dict(n, tips, e_09 median of the clean ones)}"""
    out = {}
    for s in steps:
        if s.get("period") is None:
            continue
        key = ("fast" if s["period"] < FAST_PERIOD_S else "normal", s["side"], "late" if s["k"] >= LATE_K else "early")
        d = out.setdefault(key, dict(n=0, tips=0, e09=[], ank=[], sw=[]))
        d["n"] += 1
        d["tips"] += bool(s["tip"])
        if not s["tip"]:
            for k, name in (("e09", "e_09"), ("ank", "ld_st_ank"), ("sw", "ld_sw_hip")):
                v = s.get(name)
                if v is not None and np.isfinite(v):
                    d[k].append(v)
    for d in out.values():
        for k, name in (("e09", "e_09"), ("ank", "ld_st_ank"), ("sw", "ld_sw_hip")):
            vals = d.pop(k)
            d[name] = float(np.median(vals)) if vals else float("nan")
    return out


# ------------------------------------------------------------------------------------------------
# the record and the logs
# ------------------------------------------------------------------------------------------------
def track_path(stamp):
    dirs = [os.path.join(HERE, "results")] + sorted(glob.glob(os.path.expanduser("~/code/robot*/experiments/roll-balance/results")))
    best = None
    for d in dirs:
        p = os.path.join(d, f"track_{stamp}.jsonl")
        if os.path.exists(p) and os.path.getsize(p) > 0 and (best is None or os.path.getsize(p) > os.path.getsize(best)):
            best = p
    if best is None:
        raise FileNotFoundError(f"track_{stamp}.jsonl")
    return best


def stamps_for(day, t_from, t_to):
    found = set()
    for d in [os.path.join(HERE, "results")] + glob.glob(os.path.expanduser("~/code/robot*/experiments/roll-balance/results")):
        for p in glob.glob(os.path.join(d, f"track_{day}T*.jsonl")):
            st = os.path.basename(p)[6:21]
            if t_from <= st[9:13] <= t_to and os.path.getsize(p) > 0:
                found.add(st)
    return sorted(found)


def stamp_utc(stamp):
    return dt.datetime.strptime(stamp, "%Y%m%dT%H%M%S").replace(tzinfo=LOCAL).timestamp()


def frec_for(t_utc):
    best = None
    for p in glob.glob(os.path.join(ROOT, "hw_sessions", "*", "flight_*.frec")) + \
            glob.glob(os.path.expanduser("~/code/robot/hw_sessions/*/flight_*.frec")):
        with open(p, "rb") as f:
            if f.read(len(FREC_MAGIC)) != FREC_MAGIC:
                continue
            m = json.loads(f.readline())
        c = dt.datetime.fromisoformat(m["created_utc"]).timestamp() if "created_utc" in m else None
        if c is not None and c <= t_utc and (best is None or c > best[0]):
            best = (c, os.path.realpath(p))
    return best[1] if best else None


def robot_side(path, t0, t1):
    """Per tick rows over [t0, t1] (UTC s): t, state, e (deg), integral (deg), gyro_x (deg/s), enc L, enc R (hip + ankle
    roll, deg), loads of L hip roll, L ankle roll, R hip roll, R ankle roll."""
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import flightrec as F
    names = list(F.BUS_JOINT_NAMES)
    want = {k: names.index(k) for k in ("L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll")}
    rows, cal = [], {}
    with open(path, "rb") as f:
        f.read(len(FREC_MAGIC))
        f.readline()
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                break
            n, rx = REC.unpack(h)
            t = rx / 1e6
            if t < t0 - 2.0:
                f.seek(n, 1)
                continue
            if t > t1 + 2.0:
                break
            try:
                p = F.decode(f.read(n))
            except F.DecodeError:
                continue
            if p["type"] == F.T_TICK:
                js = {j["bus"]: j for j in p["joints"]}
                z, d_ = cal.get(p["boot_id"], ([2048] * F.MAX_JOINTS, [1] * F.MAX_JOINTS))
                enc, load = [], []
                for k in ("L_hip_roll", "L_ankle_roll", "R_hip_roll", "R_ankle_roll"):
                    j, b = js.get(want[k]), want[k]
                    ok = j is not None and j["flags"] & F.JF_FB_OK
                    enc.append(np.degrees(F.steps_to_rad(j["pos"], z[b], d_[b])) if ok else np.nan)
                    load.append(float(j["load"]) if ok else np.nan)
                b_ = p["bal"]
                rows.append([p["t_utc_us"] / 1e6 if p["t_utc_us"] else t, b_["state"], np.degrees(b_["err"]),
                             np.degrees(b_["integral"]), np.degrees(p["gyro"][0]), enc[0] + enc[1], enc[2] + enc[3]]
                            + load)
            elif p["type"] != F.T_EVENT:
                cal[p["boot_id"]] = (p["zero"], p["dir"])
    a = np.array(rows, float)
    return a[np.argsort(a[:, 0])] if len(a) else a


def run_steps(stamp):
    rows = [json.loads(line) for line in open(track_path(stamp)) if line.strip()]
    summ = rows[-1] if rows and rows[-1].get("summary") else {}
    ts, period = parse_ts(rows)
    t0 = stamp_utc(stamp)
    t1 = t0 + max(r.get("t", 0.0) for r in rows) + 6.0
    path = frec_for(t0)
    a = robot_side(path, t0 - 3.0, t1) if path else np.zeros((0, 11))
    cmd = {s["step"]: s for s in (summ.get("cmd_walk") or {}).get("steps", [])}
    out, count = [], {"L": 0, "R": 0}
    ch = chains(a[:, 1]) if len(a) else []
    for n in sorted(ts):
        x = ts[n]
        count[x["side"]] += 1
        s = dict(stamp=stamp, note=summ.get("note", ""), period=period, n=n, side=x["side"], k=count[x["side"]],
                 lean_in=x["lean_in"], lean_out=x["lean_out"], T_mid=x["T"], gate_ms=x["gate"],
                 tip=x["lean_in"] >= TIP_DEG, step=json.dumps((cmd.get(n) or {}).get("step_mm_deg")))
        if n - 1 < len(ch):
            i0 = ch[n - 1][0]
            sign = 1.0 if x["side"] == "L" else -1.0
            enc = a[:, 5] if x["side"] == "L" else a[:, 6]
            ok = np.isfinite(enc)
            s.update(step_metrics(a[ok, 0], a[ok, 2], a[ok, 4], enc[ok], a[i0, 0], sign))
            s["T_lift"] = float(a[min(i0 + 2, len(a) - 1), 3])
            s["lift_utc"] = float(a[i0, 0])
            st_hip, st_ank, sw_hip = (7, 8, 9) if x["side"] == "L" else (9, 10, 7)
            s["ld_st_hip"] = window_mean(a[:, 0], a[:, st_hip], a[i0, 0], 0.3, 0.5)
            s["ld_st_ank"] = window_mean(a[:, 0], a[:, st_ank], a[i0, 0], 0.3, 0.5)
            s["ld_sw_hip"] = window_mean(a[:, 0], a[:, sw_hip], a[i0, 0], -0.25, 0.0)
        out.append(s)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamps", nargs="*")
    ap.add_argument("--day", help="YYYYMMDD: every track log of that (local) day ...")
    ap.add_argument("--from", dest="t_from", default="0000", help="HHMM")
    ap.add_argument("--to", dest="t_to", default="2359", help="HHMM")
    a = ap.parse_args(argv)
    stamps = list(a.stamps) + (stamps_for(a.day, a.t_from, a.t_to) if a.day else [])
    if not stamps:
        ap.error("no runs")
    steps = []
    for st in stamps:
        try:
            steps += run_steps(st)
        except (FileNotFoundError, ValueError, KeyError) as ex:
            print(f"{st}: skipped ({ex})", file=sys.stderr)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{stamps[0]}_{stamps[-1]}.csv")
    keys = sorted({k for s in steps for k in s}, key=lambda k: (k not in ("stamp", "n", "side", "k", "tip"), k))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows([{k: round(v, 3) if isinstance(v, float) else v for k, v in s.items()} for s in steps])
    print(f"{len(stamps)} runs, {len(steps)} single supports -> {os.path.relpath(path, ROOT)}")
    print("timing  side  stance k   steps  tips   medians of the clean ones: e@0.9   stance ankle roll load   swing hip roll load")
    for key, d in sorted(summarize(steps).items()):
        print(f"{key[0]:7s} {key[1]:5s} {('<= 4' if key[2] == 'early' else '>= 5'):9s} {d['n']:6d} {d['tips']:5d}   "
              f"{d['e_09']:+30.2f} {d['ld_st_ank']:+24.0f} {d['ld_sw_hip']:+21.0f}")
    for s in steps:
        if s["tip"]:
            print(f"  tip {s['stamp']} {s['note']} step {s['n']} {s['side']} k {s['k']} in {s['lean_in']:.1f} "
                  f"T_lift {s.get('T_lift', float('nan')):.1f} e@0.5 {s.get('e_05', float('nan')):+.1f} "
                  f"gyro@0.6 {s.get('gyro_06', float('nan')):+.0f} e@0.9 {s.get('e_09', float('nan')):+.1f} "
                  f"loads st hip {s.get('ld_st_hip', float('nan')):.0f} st ank {s.get('ld_st_ank', float('nan')):.0f} "
                  f"sw hip {s.get('ld_sw_hip', float('nan')):.0f} {s['step']}")


if __name__ == "__main__":
    main()
