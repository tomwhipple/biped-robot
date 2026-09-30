#!/usr/bin/env python3
"""Plan B bench (issue #73): does a raised position-loop P make an STS3215
stiffer in proportion, and does it still hold quietly?

DESIGN.md section 13 steps 0 and 1, driven through the robot firmware's bench
CLI over the USB tether. The firmware owns every bus write: the gain write is
its guarded `gains <id> <P> <D>` (refused unless that servo's torque is off;
EEPROM unlock, write, lock, read back), and every motion is a plain `torque`,
`move` or `release`. This script sends nothing a human at the CLI could not.

    # step 0 -- one spare STS3215 alone on the bus, on a lever. `stiffness` and
    # `hold` refuse an ID on the robot's bus map (1-17): the firmware clamps a
    # `move` to that joint's calibrated envelope, AFTER sending it. Re-ID the
    # spare at the CLI first (`id 1 30`, one servo on the bus).
    tools/gain_bench.py scan                                # which ID it answers to
    tools/gain_bench.py read      30
    tools/gain_bench.py stiffness 30 --lever-m 0.10
    tools/gain_bench.py hold      30
    # step 1 -- the prototype's stance hip roll, robot supported between runs
    tools/gain_bench.py stance    --roll L --ladder 32,128 -- --no-cam
    # the tables for the record (docs/design-v6/<date>-plan-b-bench.md)
    tools/gain_bench.py report

    # every prompt of a subcommand against a simulated servo, no hardware
    tools/gain_bench.py --rehearse stiffness 30

The session directory (default hw_sessions/<today>/gain_bench, gitignored)
collects read.json, stiffness.json, hold.json, stance.json and session.log;
`report` renders whatever is there.

STIFFNESS. At each P the servo holds a horizontal lever; weights are hung at
--lever-m for the --loads torques (0.5 / 1.0 / 1.5 N*m), then taken off. The
stiffness is the least-squares slope of deflection on torque over the loaded
points, with an intercept, so a constant friction share drops out. What passes
is the RATIO to P = 32 in the same rig (>= 4x; >= 3x conditional on <= 1 deg
of roll-chain play). Each prompt takes the mass actually hung.

The default ladder is the issue's 32 / 64 / 96 / 128 plus 160: if stiffness
is exactly proportional to P, P = 128 lands ON the 4x line and a tick of
quantization decides, so the rung above shows whether the trend carries on.

HOLD. The leg-like inertia (~0.25 kg at 0.1 m) held in two orientations:
hanging DOWN (no static torque, so the gear play floats -- where a limit cycle
lives) and HORIZONTAL (preloaded). Per P: a quiet hold, then a commanded step
out and back, one go per motion. Quiet means a peak-to-peak range of <= 2
counts (+-1) in the hold and in the last second after each step, and no
protection bit. If a raised P buzzes with D raised alongside it, suspect D
first (it differentiates a quantized encoder): re-run that P as `128:32`.

STANCE. The 2026-09-13 test (8 deg of stance hip roll, foot planted; stock
stalled 1.2 deg short at load 120) is `tools/squat_bench.py --balance
<side>:8:0 --balance-mode stance`. This wraps it: per ladder entry, release
every servo (robot supported), write the gains on the stance hip roll, run
squat_bench, read the roll's shortfall from its CSV. At the end the roll gets
back the gains it started with. Pass: <= 0.3 deg short (<= 0.4 on the 3x
route). squat_bench opens /dev/ttyUSB0 only.

Bench rules (AGENTS.md): opening the port reboots the board -- whose boot
releases every servo (firmware/main/app_main.cpp) -- and eats the first
command, so the first command after connecting is a read that must parse
before anything writes;
every bus write is a motion command, so a motion waits for a typed `go`;
hands off the horn. Any abort releases torque on the servo under test.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import glob
import json
import math
import os
import re
import subprocess
import sys
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bus_cal import BUS  # noqa: E402

TICKS_PER_RAD = 4096 / (2 * math.pi)
TICKS_PER_DEG = 4096 / 360.0
G = 9.80665
STALL_NM = 2.94                 # docs/servo-map.md section 5
MODEL_KP = 12.0                 # the deploy model's fitted STS3215 stiffness, N*m/rad

# The pass rules: issue #73, DESIGN.md section 4.
PASS_RATIO = 4.0
COND_RATIO = 3.0
QUIET_RANGE = 2                 # "<= +-1 count at hold", peak to peak
STANCE_PASS_DEG = 0.3
STANCE_COND_DEG = 0.4
# The issue's ladder plus one rung past 4x: if stiffness is exactly
# proportional to P, P = 128 sits on the 4x line and noise decides (verdict()).
DEFAULT_LADDER = "32,64,96,128,160"

# How far a torque-on may move the joint before the script calls it a jump,
# and how far the hold goal may sit from where the servo is (ticks).
JUMP_TICKS = 20
PIN_TICKS = 60
PIN_SPEED = 100                 # steps/s for the move that pins the hold goal

STATUS_BITS = ((0x01, "voltage"), (0x02, "sensor"), (0x04, "temperature"),
               (0x08, "current"), (0x10, "angle"), (0x20, "overload"))
# The bits that fail a P outright. Voltage is the bench supply, not the gain.
FATAL_BITS = 0x02 | 0x04 | 0x08 | 0x20

# Register reads for the record (addr, bytes, name). docs/servo-map.md section 4.
READ_REGS = (
    (3, 2, "model_number"), (0, 1, "fw_major"), (1, 1, "fw_minor"),
    (16, 2, "max_torque"), (24, 2, "min_start_force"),
    (26, 1, "cw_dead_zone"), (27, 1, "ccw_dead_zone"),
    (28, 2, "protect_current"), (33, 1, "mode"),
    (34, 1, "protect_torque"), (35, 1, "protect_time"),
    (36, 1, "overload_torque"), (37, 1, "speed_p"),
    (38, 1, "overcurrent_time"), (39, 1, "speed_i"),
    (40, 1, "torque_enable"), (48, 2, "torque_limit"), (65, 1, "status"),
)

# The firmware's reply formats (firmware/main/cli.cpp). tests/test_gain_bench.py
# pins each against the printf format it parses, so a wording change in the
# firmware fails a test instead of silently parsing nothing.
POS_RE = re.compile(r"id (\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)\s+"
                    r"(-?[\d.]+) V\s+(\d+) C\s+err 0x([0-9A-Fa-f]{2})")
GAINS_READ_RE = re.compile(r"id (\d+)\s+P (\d+)\s+D (\d+)\s+I (\d+)")
GAINS_OK_RE = re.compile(r"id (\d+): P (\d+) D (\d+), verified after commit")
REG_RE = re.compile(r"id (\d+) reg (\d+) = (\d+)")
SCAN_RE = re.compile(r"id\s+(\d+)\s+pos\s+(-?\d+)\s+(-?[\d.]+) V\s+err 0x([0-9A-Fa-f]{2})\s+(\S+)")
UNKNOWN_RE = re.compile(r"\? \(try")
BUSY_RE = re.compile(r"busy: ")


class Abort(RuntimeError):
    """The session stops here; torque on the servo under test is released."""


def status_names(err: int) -> str:
    return ",".join(n for b, n in STATUS_BITS if err & b) or "-"


def parse_ladder(text: str, d_ratio: float) -> List[Tuple[int, int]]:
    """'32,64,96,128' or '128:32': P, or P:D. A bare P takes D = P x d_ratio
    (the factory pair is 32/32, so 1.0 raises D with P as the issue says)."""
    out = []
    for tok in text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if ":" in tok:
            p, d = (int(x) for x in tok.split(":"))
        else:
            p = int(tok)
            d = int(round(p * d_ratio))
        if not (1 <= p <= 254 and 0 <= d <= 254):
            raise ValueError(f"ladder entry {tok!r}: P must be 1-254 and D 0-254")
        out.append((p, d))
    if not out:
        raise ValueError("empty ladder")
    return out


# =====================================================================
#  parsing the CLI
# =====================================================================
def parse_pos(text: str, sid: int) -> Optional[dict]:
    for m in POS_RE.finditer(text):
        if int(m.group(1)) == sid:
            return {"pos": int(m.group(2)), "spd": int(m.group(3)),
                    "load": int(m.group(4)), "volt": float(m.group(5)),
                    "temp": int(m.group(6)), "err": int(m.group(7), 16)}
    return None


def parse_gains_read(text: str, sid: int) -> Optional[Tuple[int, int, int]]:
    for m in GAINS_READ_RE.finditer(text):
        if int(m.group(1)) == sid:
            return int(m.group(2)), int(m.group(3)), int(m.group(4))
    return None


def parse_gains_write(text: str, sid: int) -> Optional[Tuple[int, int]]:
    for m in GAINS_OK_RE.finditer(text):
        if int(m.group(1)) == sid:
            return int(m.group(2)), int(m.group(3))
    return None


def parse_scan(text: str) -> List[dict]:
    return [{"id": int(m.group(1)), "pos": int(m.group(2)), "volt": float(m.group(3)),
             "err": int(m.group(4), 16), "joint": m.group(5)} for m in SCAN_RE.finditer(text)]


def parse_reg(text: str, sid: int, addr: int) -> Optional[int]:
    for m in REG_RE.finditer(text):
        if int(m.group(1)) == sid and int(m.group(2)) == addr:
            return int(m.group(3))
    return None


# =====================================================================
#  analysis (pure; tests/test_gain_bench.py)
# =====================================================================
def _mean(xs: Sequence[float]) -> float:
    return sum(xs) / len(xs) if xs else float("nan")


def window_stats(samples: Sequence[dict]) -> dict:
    """One sampled window: where the servo sat, how much it moved, what it pushed."""
    if not samples:
        return {"n": 0}
    pos = [s["pos"] for s in samples]
    load = [s["load"] for s in samples]
    err = 0
    for s in samples:
        err |= s["err"]
    dur = samples[-1]["t"] - samples[0]["t"] if len(samples) > 1 else 0.0
    return {
        "n": len(samples),
        "hz": (len(samples) - 1) / dur if dur > 0 else float("nan"),
        "pos_mean": _mean(pos), "pos_min": min(pos), "pos_max": max(pos),
        "range": max(pos) - min(pos),
        "moving": sum(1 for s in samples if s["spd"] != 0),
        "load_mean": _mean(load), "load_min": min(load), "load_max": max(load),
        "err": err, "temp_max": max(s["temp"] for s in samples),
        "volt_min": min(s["volt"] for s in samples),
    }


def fit_stiffness(points: Sequence[Tuple[float, float]]) -> dict:
    """k from (torque N*m, deflection ticks) at the LOADED points.

    Least squares of deflection on torque WITH an intercept: the servo's
    powered friction (~0.24 N*m, servo-map.md section 5) holds part of the
    first load without any position error, which shifts every point by about
    the same amount and leaves the slope alone. The standard error carries a
    quantization floor (a uniform +-0.5 tick, sigma^2 = 1/12), because a
    static servo reads the same tick every sample and a perfect-looking fit
    of three integer means is not a precise one.
    """
    n = len(points)
    if n < 2:
        return {"k": float("nan"), "k_rel_se": float("nan"),
                "intercept_ticks": float("nan")}
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    mx, my = _mean(xs), _mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return {"k": float("nan"), "k_rel_se": float("nan"),
                "intercept_ticks": float("nan")}
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx   # ticks / N*m
    icpt = my - slope * mx
    s2 = 1.0 / 12.0
    if n > 2:
        s2 = max(s2, sum((y - icpt - slope * x) ** 2 for x, y in zip(xs, ys)) / (n - 2))
    if slope <= 0:
        return {"k": float("inf"), "k_rel_se": float("nan"), "intercept_ticks": icpt}
    return {"k": TICKS_PER_RAD / slope,
            "k_rel_se": math.sqrt(s2 / sxx) / slope,
            "intercept_ticks": icpt}


def stiffness_row(row: dict) -> dict:
    """Summarise one P of a stiffness block: deflection per load, the fit."""
    steps = row["steps"]
    base = window_stats(steps[0]["samples"])
    ret = window_stats(row["return"]["samples"]) if row.get("return") else {"n": 0}
    pts, per = [], []
    err = base.get("err", 0) | ret.get("err", 0)
    for st in steps[1:]:
        ws = window_stats(st["samples"])
        if not ws["n"] or not base["n"]:
            continue
        defl = abs(ws["pos_mean"] - base["pos_mean"])
        pts.append((st["tau"], defl))
        err |= ws["err"]
        per.append({"tau": st["tau"], "defl_ticks": defl,
                    "secant_k": (st["tau"] / (defl / TICKS_PER_RAD)) if defl > 0 else float("inf"),
                    "load_mean": ws["load_mean"], "range": ws["range"]})
    fit = fit_stiffness(pts)
    return {"p": row["p"], "d": row["d"], "k": fit["k"], "k_rel_se": fit["k_rel_se"],
            "intercept_ticks": fit["intercept_ticks"], "loads": per,
            "return_offset": (ret["pos_mean"] - base["pos_mean"]) if ret["n"] and base["n"] else float("nan"),
            "err": err}


def step_stats(samples: Sequence[dict], start: int, target: int) -> dict:
    """A commanded step: overshoot past where it came to rest, when it came to
    rest (every later sample within +-1 of the final mean), and how still that
    rest is over the last second."""
    if not samples:
        return {"n": 0}
    t_end = samples[-1]["t"]
    tail = [s for s in samples if s["t"] >= t_end - min(1.0, t_end / 2)]
    final = _mean([s["pos"] for s in tail])
    dirn = 1 if target >= start else -1
    over = max(0.0, max((s["pos"] - final) * dirn for s in samples))
    settle = None
    for k in range(len(samples)):
        if all(abs(s["pos"] - final) <= 1.0 for s in samples[k:]):
            settle = samples[k]["t"]
            break
    rng = max(s["pos"] for s in tail) - min(s["pos"] for s in tail)
    err = 0
    for s in samples:
        err |= s["err"]
    return {"n": len(samples), "overshoot": over, "settle_s": settle,
            "residual_range": rng, "offset": final - target, "err": err}


def hold_row(row: dict) -> dict:
    q = window_stats(row["quiet"]["samples"])
    out = step_stats(row["out"]["samples"], row["goal"], row["out"]["target"]) if row.get("out") else {"n": 0}
    back = step_stats(row["back"]["samples"], row["out"]["target"], row["goal"]) if row.get("back") else {"n": 0}
    ranges = [q.get("range", 99)] + [s["residual_range"] for s in (out, back) if s["n"]]
    err = q.get("err", 0) | out.get("err", 0) | back.get("err", 0)
    return {"p": row["p"], "d": row["d"], "orient": row["orient"], "quiet": q,
            "out": out, "back": back,
            "is_quiet": bool(q.get("n")) and max(ranges) <= QUIET_RANGE and not (err & FATAL_BITS),
            "err": err}


def verdict(ratio: float, quiet: Optional[bool], err: int, ratio_se: float = 0.0) -> str:
    """The issue's rule for one P. MARGINAL flags a ratio within one standard
    error under a threshold: a servo exactly proportional to P lands ON the 4x
    line at P = 128, and there a tick of quantization decides -- the next rung
    up says which side it is on."""
    if err & FATAL_BITS:
        return f"FAIL (protection: {status_names(err & FATAL_BITS)})"
    if quiet is False:
        return "FAIL (not quiet)"
    if not (ratio == ratio):                      # nan: no P = 32 row to compare with
        return "no P = 32 reference"
    se = ratio_se if ratio_se == ratio_se else 0.0
    if ratio >= PASS_RATIO:
        return "PASS" if quiet else "stiff enough; hold not run"
    if ratio + se >= PASS_RATIO:
        return f"MARGINAL (within error of {PASS_RATIO:g}x)"
    if ratio >= COND_RATIO:
        return ("CONDITIONAL (roll chains <= 1 deg play)" if quiet
                else "3x-stiff; hold not run")
    if ratio + se >= COND_RATIO:
        return f"MARGINAL (within error of {COND_RATIO:g}x)"
    return "not stiff enough"


def summarise(stiff: Optional[dict], hold: Optional[dict]) -> dict:
    """Join the stiffness and hold blocks per (P, D) and give each a verdict."""
    srows = [stiffness_row(r) for r in (stiff or {}).get("rows", [])]
    hrows = [hold_row(r) for r in (hold or {}).get("rows", [])]
    ref = next((r for r in srows if r["p"] == 32), None)
    keys = []
    for r in srows + hrows:
        if (r["p"], r["d"]) not in keys:
            keys.append((r["p"], r["d"]))
    joined = []
    for p, d in keys:
        s = next((r for r in srows if (r["p"], r["d"]) == (p, d)), None)
        hs = [r for r in hrows if (r["p"], r["d"]) == (p, d)]
        ratio = rse = float("nan")
        if s and ref and ref["k"] == ref["k"] and s["k"] == s["k"]:
            ratio = s["k"] / ref["k"]
            rse = math.hypot(s["k_rel_se"], ref["k_rel_se"]) * ratio
        quiet = all(h["is_quiet"] for h in hs) if hs else None
        err = (s["err"] if s else 0)
        for h in hs:
            err |= h["err"]
        joined.append({"p": p, "d": d, "k": s["k"] if s else float("nan"),
                       "ratio": ratio, "ratio_se": rse, "quiet": quiet, "err": err,
                       "verdict": verdict(ratio, quiet, err, rse)})
    passing = [j for j in joined if j["verdict"] == "PASS"]
    cond = [j for j in joined if j["verdict"].startswith("CONDITIONAL")]
    if passing:
        best = min(passing, key=lambda j: j["p"])
        overall = f"PASS at P {best['p']} D {best['d']}: {best['ratio']:.2f}x, quiet"
    elif cond:
        best = min(cond, key=lambda j: j["p"])
        overall = (f"CONDITIONAL at P {best['p']} D {best['d']}: {best['ratio']:.2f}x, quiet -- "
                   f"holds only if the printed roll chains measure <= 1 deg of play")
    elif not srows:
        overall = "stiffness not measured yet"
    elif any(j["verdict"].startswith("MARGINAL") for j in joined):
        overall = "no P clears the line outright; a MARGINAL rung wants the next P up"
    else:
        overall = "no P passes: DESIGN.md section 4 fallbacks, in order"
    return {"stiffness": srows, "hold": hrows, "joined": joined, "overall": overall}


def stance_from_csv(path: str, side: str, phi: float) -> dict:
    """The stance hip roll's shortfall from a squat_bench --balance CSV.

    squat_bench rolls the stance hip to -phi (L) or +phi (R) in the sim frame
    and writes every joint's read-back angle and load per phase; short is how
    far the roll stopped before its command, positive = short."""
    target = -phi if side == "L" else +phi
    sign = 1.0 if target >= 0 else -1.0
    col, lcol = f"{side}_hip_roll_deg", f"{side}_hip_roll_load"
    phases = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if not r["phase"].startswith("lean"):
                continue
            meas = float(r[col])
            phases.append({"phase": r["phase"], "measured": meas,
                           "short": (target - meas) * sign, "load": int(r[lcol]),
                           "tilt": float(r["tilt_deg"])})
    worst = max(phases, key=lambda x: x["short"]) if phases else None
    return {"target": target, "phases": phases,
            "short": worst["short"] if worst else float("nan"),
            "load": worst["load"] if worst else None}


def stance_verdict(short: float) -> str:
    if not (short == short):
        return "no reading"
    if short <= STANCE_PASS_DEG:
        return "PASS"
    if short <= STANCE_COND_DEG:
        return "PASS on the 3x route only"
    return "FAIL"


# =====================================================================
#  the report
# =====================================================================
def _f(x: float, fmt: str = ".1f") -> str:
    return "—" if x is None or x != x else format(x, fmt)


def render_report(session: str) -> str:
    def load(name):
        p = os.path.join(session, name)
        return json.load(open(p)) if os.path.exists(p) else None
    rd, st, hd, sn = (load(n) for n in ("read.json", "stiffness.json", "hold.json", "stance.json"))
    where = os.path.abspath(session)
    if where.startswith(ROOT + os.sep):
        where = os.path.relpath(where, ROOT)
    L = [f"# Plan B bench (issue #73): `{where}`", ""]
    if rd:
        r = rd["regs"]
        L += [f"**Servo** id {rd['id']}: model {r.get('model_number')}, "
              f"P/D/I {rd['gains']}, dead zone {r.get('cw_dead_zone')}/{r.get('ccw_dead_zone')}, "
              f"overload {r.get('overload_torque')} % for {r.get('protect_time')} x 10 ms -> "
              f"{r.get('protect_torque')} %, {rd['pos']['volt'] if rd.get('pos') else '?'} V.", ""]
    summ = summarise(st, hd)
    if st:
        L += [f"## Stiffness vs P (lever {st['lever_m'] * 1000:.0f} mm, hold goal {st['goal']})", "",
              "| P | D | deflection at " + " / ".join(f"{t:g}" for t in st["loads_nm"]) +
              " N·m (ticks) | k fit (N·m/rad) | × P 32 | load reg at max | return (ticks) | flags |",
              "|---|---|---|---|---|---|---|---|"]
        by_pd = {(j["p"], j["d"]): j for j in summ["joined"]}
        for s in summ["stiffness"]:
            j = by_pd[(s["p"], s["d"])]
            defl = " / ".join(_f(x["defl_ticks"]) for x in s["loads"])
            lmax = _f(s["loads"][-1]["load_mean"], ".0f") if s["loads"] else "—"
            L.append(f"| {s['p']} | {s['d']} | {defl} | {_f(s['k'])} ± {_f(s['k'] * s['k_rel_se'])} | "
                     f"{_f(j['ratio'], '.2f')} ± {_f(j['ratio_se'], '.2f')} | {lmax} | "
                     f"{_f(s['return_offset'])} | {status_names(s['err'])} |")
        ref = next((s for s in summ["stiffness"] if s["p"] == 32), None)
        if ref and ref["k"] == ref["k"]:
            L += ["", f"P 32 measures {ref['k']:.1f} N·m/rad in this rig; the deploy model "
                      f"fits {MODEL_KP:g} and credits P x N as N x {MODEL_KP:g}."]
        L.append("")
    if hd:
        L += [f"## Hold ({hd.get('inertia', '')}; step {hd['step_ticks']} ticks)", "",
              "| P | D | orientation | hold range (ticks) | moving samples | step out: overshoot / settle s / residual | back | flags | quiet |",
              "|---|---|---|---|---|---|---|---|---|"]
        for h in summ["hold"]:
            def st_(s):
                return (f"{_f(s['overshoot'], '.0f')} / {_f(s['settle_s'], '.2f')} / "
                        f"{s['residual_range']}") if s.get("n") else "—"
            L.append(f"| {h['p']} | {h['d']} | {h['orient']} | {h['quiet'].get('range', '—')} | "
                     f"{h['quiet'].get('moving', '—')}/{h['quiet'].get('n', 0)} "
                     f"({_f(h['quiet'].get('hz'), '.0f')} Hz) | {st_(h['out'])} | {st_(h['back'])} | "
                     f"{status_names(h['err'])} | {'yes' if h['is_quiet'] else '**no**'} |")
        L.append("")
    if st or hd:
        L += ["## Verdict per P", "", "| P | D | × P 32 | quiet | verdict |", "|---|---|---|---|---|"]
        for j in summ["joined"]:
            q = "—" if j["quiet"] is None else ("yes" if j["quiet"] else "no")
            L.append(f"| {j['p']} | {j['d']} | {_f(j['ratio'], '.2f')} | {q} | {j['verdict']} |")
        L += ["", f"**Step 0: {summ['overall']}.**", ""]
    if sn:
        L += [f"## Stance (step 1): {sn['side']}_hip_roll id {sn['roll_id']}, "
              f"{sn['phi']:g}° commanded, foot planted", "",
              "| P | D | phase | measured (°) | short (°) | load | tilt (°) | verdict |",
              "|---|---|---|---|---|---|---|---|"]
        for r in sn["rows"]:
            res = r.get("result") or {"phases": []}
            for ph in res["phases"]:
                L.append(f"| {r['p']} | {r['d']} | {ph['phase']} | {ph['measured']:+.2f} | "
                         f"{ph['short']:.2f} | {ph['load']} | {ph['tilt']:.1f} | "
                         f"{stance_verdict(ph['short'])} |")
            if not res["phases"]:
                L.append(f"| {r['p']} | {r['d']} | — | — | — | — | — | {r.get('note', 'no reading')} |")
        L += ["", f"Pass ≤ {STANCE_PASS_DEG}° short (≤ {STANCE_COND_DEG}° on the 3× route); "
                  "stock stalled 1.2° short at load 120 on 2026-09-13.", ""]
    return "\n".join(L)


# =====================================================================
#  the tether, and a servo to rehearse against
# =====================================================================
def find_port(explicit: Optional[str]) -> str:
    if explicit:
        return explicit
    cands = (["/dev/ttyUSB0"] if os.path.exists("/dev/ttyUSB0") else []) + sorted(
        glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB_USBtoUART*")
        + glob.glob("/dev/cu.wchusbserial*"))
    if not cands:
        raise Abort("no tether port (/dev/ttyUSB0, /dev/cu.usbserial*); pass --port")
    return cands[0]


class Board:
    """The tether: one CLI line out, the reply text back."""

    def __init__(self, port: str, log: Callable[[str], None]):
        import serial                               # only the live path needs pyserial
        self.log = log
        self.s = serial.Serial(port, 115200, timeout=0.05)
        # Opening the port reboots the board and the boot eats the first
        # command (AGENTS.md). Wait it out and drain; the caller's first
        # command is a read that has to parse before anything writes.
        time.sleep(2.5)
        self.s.read(65536)
        self.s.write(b"\r\n")
        time.sleep(0.4)
        self.s.read(65536)

    def cmd(self, line: str, until: str, timeout: float = 3.0) -> str:
        self.s.read(self.s.in_waiting or 0)
        self.s.write(line.encode() + b"\r\n")
        pat = re.compile(until)
        t0, out = time.monotonic(), ""
        while time.monotonic() - t0 < timeout:
            out += self.s.read(max(1, self.s.in_waiting)).decode(errors="replace")
            if pat.search(out):
                break
        self.log(f">>> {line}\n{out.rstrip()}")
        return out

    def rig(self, **_event) -> None:
        """A human did it; there is nothing to simulate."""

    def close(self) -> None:
        self.s.close()


class FakeBoard:
    """A servo on a lever speaking the firmware CLI's reply formats: for
    --rehearse and the tests. Static deflection = (torque - friction) /
    (k32 x P / 32), in whole ticks; from `buzz_p` up it limit-cycles +-3
    ticks. A walk-through of every prompt and parse, not a servo model."""

    REGS = {3: 777, 0: 3, 1: 10, 16: 1000, 24: 0, 26: 1, 27: 1, 28: 500, 33: 0,
            34: 20, 35: 200, 36: 80, 37: 10, 38: 200, 39: 10, 48: 1000, 65: 0}

    def __init__(self, sid: int = 30, k32: float = 17.0, buzz_p: int = 255,
                 friction_nm: float = 0.05, pos: int = 2048, latency_s: float = 0.0,
                 log=lambda s: None):
        self.sid, self.k32, self.buzz_p, self.fric = sid, k32, buzz_p, friction_nm
        self.latency_s = latency_s
        self.p, self.d, self.i = 32, 32, 0
        self.torque, self.goal, self.rest, self.tau, self.n = False, pos, pos, 0.0, 0
        self.log = log

    def rig(self, tau: Optional[float] = None, **_event) -> None:
        if tau is not None:
            self.tau = tau

    def _pos(self) -> Tuple[int, int]:
        self.n += 1
        if not self.torque:
            return self.rest, 0
        t = max(0.0, abs(self.tau) - self.fric)
        defl = int(round(t / (self.k32 * self.p / 32.0) * TICKS_PER_RAD))
        buzz = (3 if self.n % 2 else -3) if self.p >= self.buzz_p else 0
        self.rest = self.goal - defl + buzz
        return self.rest, (40 if buzz else 0)

    def cmd(self, line: str, until: str = "", timeout: float = 0.0) -> str:
        time.sleep(self.latency_s)
        a = line.split()
        c, sid = a[0], (int(a[1]) if len(a) > 1 and a[1].isdigit() else None)
        if sid is not None and sid != self.sid and c in ("pos", "reg", "gains", "move"):
            out = f"id {sid}: timeout\r\n"
        elif c == "pos":
            p, spd = self._pos()
            load = int(round(1000 * self.tau / STALL_NM)) if self.torque else 0
            out = (f"id {sid}  pos {p:5d}  spd {spd:5d}  load {load:4d}  12.1 V  31 C  "
                   f"err 0x00\r\n")
        elif c == "scan":
            out = (f"scanning IDs 0-253 ...\r\n  id {self.sid:3d}  pos {self.rest:5d}  12.1 V  "
                   f"err 0x00  -\r\n1 servo(s)\r\n")
        elif c == "reg":
            addr = int(a[2])
            v = int(self.torque) if addr == 40 else {21: self.p, 22: self.d, 23: self.i}.get(
                addr, self.REGS.get(addr, 0))
            out = f"id {sid} reg {addr} = {v} (0x{v:02X})\r\n"
        elif c == "gains" and len(a) == 2:
            out = (f"id {sid}  P {self.p}  D {self.d}  I {self.i}   expected "
                   f"(no row in servo_gains.h)\r\n")
        elif c == "gains" and len(a) == 4:
            if self.torque:
                out = (f"REFUSED: id {sid} has torque ON. `release {sid}` first -- gains "
                       f"are written only to a servo that is not holding.\r\n")
            else:
                self.p, self.d = int(a[2]), int(a[3])
                out = (f"id {sid}: writing P {self.p} D {self.d} to EEPROM (torque must be "
                       f"OFF; no goal is written) ...\r\nid {sid}: P {self.p} D {self.d}, "
                       f"verified after commit -- safe to power down\r\n")
        elif c in ("torque", "release"):
            on = c == "torque"
            if on and not self.torque:
                self.goal = self.rest                 # holds where it is
            self.torque = on
            out = f"{c} {a[1] if len(a) > 1 else 'all'}: ok\r\n"
        elif c == "move":
            if not self.torque:
                out = (f"id {sid}: torque is OFF, and a goal write would auto-enable it "
                       f"and MOVE the joint. `torque {sid}` first if you mean it.\r\n")
            else:
                self.goal = int(a[2])
                out = f"move id {sid} -> {a[2]} ticks, 0 ms, spd 0, acc 0: ok\r\n"
        else:
            out = "? (try `help`)\r\n"
        self.log(f">>> {line}\n{out.rstrip()}")
        return out

    def close(self) -> None:
        pass


class Operator:
    """The human at the bench. One motion, one go (AGENTS.md)."""

    def __init__(self, ask: Optional[Callable[[str], str]] = None,
                 say: Callable[[str], None] = print):
        self.ask = ask or self._stdin
        self.say = say

    @staticmethod
    def _stdin(prompt: str) -> str:
        # No terminal behind the prompt (a pipe that ran dry, an agent's shell)
        # is a stop, not a traceback: the session aborts through the same path
        # as a refused go, which releases and restores the gains. An agent
        # drives this from a tmux pane with the human's go typed into it.
        try:
            return input(prompt)
        except EOFError:
            raise Abort("no operator on stdin (EOF) -- run it in a terminal") from None

    def go(self, what: str) -> None:
        a = self.ask(f"\n  NEXT MOTION: {what}\n  hands clear of the horn; type go: ")
        if a.strip().lower() != "go":
            raise Abort(f"no go for: {what}")

    def wait(self, what: str) -> str:
        return self.ask(f"\n  {what}\n  enter when done: ")


# =====================================================================
#  bench primitives
# =====================================================================
POS_UNTIL = r"(err 0x[0-9A-Fa-f]{2}|id \d+: [a-z-]+\r?\n|busy: |\? \(try)"
MOVE_UNTIL = r"(move id .*: \S+\r?\n|torque is OFF|\? \(try|busy: )"


def read_pos(board, sid: int, tries: int = 1) -> dict:
    out = ""
    for _ in range(tries):
        out = board.cmd(f"pos {sid}", until=POS_UNTIL, timeout=1.5)
        if BUSY_RE.search(out):
            raise Abort("the control loop owns the bus -- `bench` it by hand first")
        r = parse_pos(out, sid)
        if r:
            return r
    raise Abort(f"id {sid}: no position reply ({out.strip()[-80:]!r}) -- pack on? right ID?")


def read_reg(board, sid: int, addr: int, width: int = 1) -> Optional[int]:
    out = board.cmd(f"reg {sid} {addr} {width}",
                    until=r"(reg \d+ = \d+ \(0x|reg \d+: [a-z-]+\r?\n|bad id|\? \(try)",
                    timeout=1.5)
    return parse_reg(out, sid, addr)


def read_gains(board, sid: int) -> Tuple[int, int, int]:
    out = board.cmd(f"gains {sid}", until=r"(expected.*\n|id \d+: [a-z-]+\r?\n|\? \(try|busy: )",
                    timeout=2.0)
    if UNKNOWN_RE.search(out):
        raise Abort("this firmware has no `gains` command: flash main (PR #94) first")
    g = parse_gains_read(out, sid)
    if not g:
        raise Abort(f"id {sid}: gains unreadable ({out.strip()[-80:]!r})")
    return g


def connect(board, sid: int) -> None:
    """Prove the link with reads before anything writes: the boot ate one
    command already, and a wrong calibration was once saved through that."""
    read_pos(board, sid, tries=4)
    read_gains(board, sid)


def sample(board, sid: int, n: int = 0, seconds: float = 0.0) -> List[dict]:
    """`pos` polled back to back: n samples, or for `seconds`."""
    out, t0 = [], time.monotonic()
    while (n and len(out) < n) or (seconds and time.monotonic() - t0 < seconds):
        r = read_pos(board, sid)
        r["t"] = round(time.monotonic() - t0, 4)
        out.append(r)
        if not n and not seconds:
            break
    return out


def release(board, sid: Optional[int] = None) -> None:
    board.cmd("release" + (f" {sid}" if sid is not None else ""), until=r"release \S+: \S+\r?\n")


def write_gains(board, sid: int, p: int, d: int) -> None:
    """The firmware's guarded write, after the release that makes it legal."""
    release(board, sid)
    out = board.cmd(f"gains {sid} {p} {d}",
                    until=r"(verified after commit|REFUSED|WROTE BUT|FAILED|usage|must be|\? \(try)",
                    timeout=8.0)
    got = parse_gains_write(out, sid)
    if got != (p, d):
        raise Abort(f"id {sid}: gains {p}/{d} not verified: {out.strip()[-120:]!r}")
    if read_gains(board, sid)[:2] != (p, d):
        raise Abort(f"id {sid}: gains read back differently after the write")


def capture_goal(board, op: Operator, sid: int, how: str) -> int:
    if read_reg(board, sid, 40) != 0:
        op.say("  torque is ON -- releasing it (the load may drop; hands clear)")
        release(board, sid)
    op.wait(f"Torque is OFF. {how}. Let go of it")
    return int(round(_mean([s["pos"] for s in sample(board, sid, n=3)])))


def hold_at(board, op: Operator, sid: int, goal: int, what: str, settle_s: float) -> None:
    """Torque on, then pin the goal to the rig's hold position -- one motion."""
    before = read_pos(board, sid)
    if abs(before["pos"] - goal) > PIN_TICKS:
        raise Abort(f"id {sid} sits at {before['pos']}, {abs(before['pos'] - goal)} ticks from "
                    f"the hold goal {goal}: re-seat the rig and start again")
    op.go(what)
    board.cmd(f"torque {sid}", until=r"torque \S+: \S+\r?\n")
    after = read_pos(board, sid)
    if abs(after["pos"] - before["pos"]) > JUMP_TICKS:
        release(board, sid)
        raise Abort(f"id {sid} jumped {after['pos'] - before['pos']} ticks at torque-on "
                    f"(a stale goal register?) -- released")
    out = board.cmd(f"move {sid} {goal} 0 {PIN_SPEED}", until=MOVE_UNTIL)
    if "clamped" in out or not re.search(r"move id .*: ok", out):
        release(board, sid)
        raise Abort(f"id {sid}: the pin move failed: {out.strip()[-80:]!r}")
    time.sleep(settle_s)


# =====================================================================
#  the subcommands
# =====================================================================
def run_read(board, op: Operator, sid: int) -> dict:
    p, d, i = read_gains(board, sid)
    regs = {name: read_reg(board, sid, addr, width) for addr, width, name in READ_REGS}
    pos = read_pos(board, sid)
    op.say(f"id {sid}: P {p} D {d} I {i}; model {regs['model_number']} "
           f"(STS3215 = 777); torque {regs['torque_enable']}; {pos['volt']} V; "
           f"status {status_names(regs['status'] or 0)}")
    if regs["model_number"] not in (None, 777):
        op.say(f"  !! model {regs['model_number']} is not an STS3215 (777)")
    return {"id": sid, "gains": [p, d, i], "regs": regs, "pos": pos}


def ask_mass(op: Operator, what: str, default: float) -> float:
    """Enter = as asked; a number = what was actually hung. A typo asks again
    rather than ending a session half-way up the ladder."""
    while True:
        ans = op.wait(what).strip()
        if not ans:
            return default
        try:
            m = float(ans)
        except ValueError:
            m = -1.0
        if 0 < m < 5:
            return m
        op.say(f"  {ans!r} is not a mass in kg (0-5)")


def run_stiffness(board, op: Operator, sid: int, ladder, lever_m: float,
                  loads_nm: Sequence[float], n: int, settle_s: float,
                  on_row: Callable[[dict], None] = lambda res: None) -> dict:
    goal = capture_goal(board, op, sid, "Set the lever HORIZONTAL, hook on the loading "
                                        "side, nothing hung on it")
    res = {"id": sid, "lever_m": lever_m, "goal": goal, "loads_nm": list(loads_nm),
           "ladder": ladder, "rows": []}
    for p, d in ladder:
        op.say(f"\n== P {p} D {d}")
        write_gains(board, sid, p, d)
        hold_at(board, op, sid, goal, f"P {p} D {d}: torque on, hold the bare lever at "
                                      f"{goal}", settle_s)
        row = {"p": p, "d": d, "steps": [{"tau": 0.0, "samples": sample(board, sid, n=n)}]}
        for tau in loads_nm:
            m = tau / (G * lever_m)
            m_act = ask_mass(op, f"P {p}: hang {m:.3f} kg in total at {lever_m * 1000:.0f} mm "
                                 f"({tau:g} N·m). Type the mass actually hung (kg) if it differs",
                             m)
            tau_act = m_act * G * lever_m
            board.rig(tau=tau_act)
            time.sleep(settle_s)
            ss = sample(board, sid, n=n)
            row["steps"].append({"tau": tau_act, "mass_kg": m_act, "samples": ss})
            w = window_stats(ss)
            op.say(f"  {tau_act:.3f} N·m: pos {w['pos_mean']:.1f} (range {w['range']}), "
                   f"load {w['load_mean']:.0f}, status {status_names(w['err'])}")
        op.wait(f"P {p}: take every weight off (bare lever)")
        board.rig(tau=0.0)
        time.sleep(settle_s)
        row["return"] = {"samples": sample(board, sid, n=n)}
        release(board, sid)
        s = stiffness_row(row)
        op.say(f"  P {p} D {d}: k {s['k']:.1f} N·m/rad (±{s['k'] * s['k_rel_se']:.1f}), "
               f"return {s['return_offset']:+.1f} ticks -- released")
        res["rows"].append(row)
        on_row(res)
    return res


ORIENT_TEXT = {
    "down": "Let the inertia HANG STRAIGHT DOWN (no static torque: the gear play floats)",
    "horizontal": "Set the inertia's lever HORIZONTAL (preloaded)",
}


def run_hold(board, op: Operator, sid: int, ladder, orients: Sequence[str], quiet_s: float,
             step_ticks: int, step_s: float, settle_s: float, inertia: str,
             on_row: Callable[[dict], None] = lambda res: None) -> dict:
    res = {"id": sid, "ladder": ladder, "inertia": inertia, "step_ticks": step_ticks, "rows": []}
    for orient in orients:
        if orient not in ORIENT_TEXT:
            raise Abort(f"orientation {orient!r}: one of {', '.join(ORIENT_TEXT)}")
        goal = capture_goal(board, op, sid, f"Mount the inertia ({inertia}). {ORIENT_TEXT[orient]}")
        if not 0 <= goal + step_ticks <= 4095:
            raise Abort(f"hold goal {goal} + step {step_ticks} leaves 0..4095")
        for p, d in ladder:
            op.say(f"\n== {orient}: P {p} D {d}")
            write_gains(board, sid, p, d)
            hold_at(board, op, sid, goal, f"P {p} D {d}: torque on, hold the inertia {orient}",
                    settle_s)
            row = {"p": p, "d": d, "orient": orient, "goal": goal,
                   "quiet": {"samples": sample(board, sid, seconds=quiet_s)}}
            q = window_stats(row["quiet"]["samples"])
            op.say(f"  hold: range {q['range']} ticks, moving {q['moving']}/{q['n']} "
                   f"({q['hz']:.0f} Hz), load {q['load_min']}..{q['load_max']}, "
                   f"status {status_names(q['err'])}")
            for leg, frm, to in (("out", goal, goal + step_ticks), ("back", goal + step_ticks, goal)):
                op.go(f"P {p}: step {to - frm:+d} ticks ({(to - frm) / TICKS_PER_DEG:+.1f}°) at full speed")
                out = board.cmd(f"move {sid} {to} 0 0", until=MOVE_UNTIL)
                if "clamped" in out or not re.search(r"move id .*: ok", out):
                    release(board, sid)
                    raise Abort(f"id {sid}: step move failed: {out.strip()[-80:]!r}")
                row[leg] = {"target": to, "samples": sample(board, sid, seconds=step_s)}
                s = step_stats(row[leg]["samples"], frm, to)
                op.say(f"  step {leg}: overshoot {s['overshoot']:.0f}, settle "
                       f"{_f(s['settle_s'], '.2f')} s, residual range {s['residual_range']}, "
                       f"offset {s['offset']:+.1f}")
            release(board, sid)
            h = hold_row(row)
            op.say(f"  {'quiet' if h['is_quiet'] else 'NOT QUIET'} -- released")
            res["rows"].append(row)
            on_row(res)
    return res


def roll_id(side: str) -> int:
    return next(i for name, i, _ in BUS if name == f"{side}_hip_roll")


# =====================================================================
#  main
# =====================================================================
def _saver(session: str, name: str, **extra) -> Callable[[dict], str]:
    """Write `name` after every completed row, so an abort keeps what was
    measured. A file from an earlier run is set aside once, not overwritten."""
    path = os.path.join(session, name)
    if os.path.exists(path):
        stamp = _dt.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%H%M%S")
        os.replace(path, path.replace(".json", f".{stamp}.json"))

    def save(obj: dict) -> str:
        with open(path, "w") as f:
            json.dump(dict(obj, **extra, saved=_dt.datetime.now().isoformat(timespec="seconds")),
                      f, indent=1)
        return path
    return save


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="the tether (default /dev/ttyUSB0, else cu.usbserial*)")
    ap.add_argument("--session", help="session directory (default hw_sessions/<today>/gain_bench)")
    ap.add_argument("--rehearse", action="store_true",
                    help="run the prompts against a simulated servo -- no hardware")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", help="every ID on the bus (the firmware's `scan`), read-only")

    s = sub.add_parser("read", help="step 0a: registers, read-only")
    s.add_argument("id", type=int)

    ladder_help = "P or P:D, comma-separated; a bare P takes D = P x --d-ratio"
    s = sub.add_parser("stiffness", help="step 0b: static stiffness vs P on a lever")
    s.add_argument("id", type=int)
    s.add_argument("--ladder", default=DEFAULT_LADDER, help=ladder_help)
    s.add_argument("--d-ratio", type=float, default=1.0)
    s.add_argument("--lever-m", type=float, default=0.10, help="hook distance from the axis, m")
    s.add_argument("--loads", default="0.5,1.0,1.5", help="torques, N*m")
    s.add_argument("--samples", type=int, default=20)
    s.add_argument("--settle", type=float, default=3.0, help="s after each load change")
    s.add_argument("--keep", action="store_true", help="leave the last P in the servo")

    s = sub.add_parser("hold", help="step 0c: hold and step response with a leg-like inertia")
    s.add_argument("id", type=int)
    s.add_argument("--ladder", default=DEFAULT_LADDER, help=ladder_help)
    s.add_argument("--d-ratio", type=float, default=1.0)
    s.add_argument("--orient", default="down,horizontal")
    s.add_argument("--quiet-s", type=float, default=5.0)
    s.add_argument("--step", type=int, default=23, help="step size, ticks (23 = 2 deg)")
    s.add_argument("--step-s", type=float, default=3.0)
    s.add_argument("--settle", type=float, default=2.0)
    s.add_argument("--inertia", default="0.25 kg at 0.10 m")
    s.add_argument("--keep", action="store_true")

    s = sub.add_parser("stance", help="step 1: stance hip roll shortfall per P (squat_bench)")
    s.add_argument("--roll", choices=("L", "R"), default="L", help="the stance side")
    s.add_argument("--phi", type=float, default=8.0)
    s.add_argument("--lift", type=float, default=0.0, help="swing-leg lift theta (0 = planted)")
    s.add_argument("--ladder", default="32,128", help=ladder_help)
    s.add_argument("--d-ratio", type=float, default=1.0)
    s.add_argument("squat_args", nargs="*", help="after --: passed to squat_bench.py")

    s = sub.add_parser("report", help="render the session's tables")
    s.add_argument("--md", help="also write the markdown here")

    A = ap.parse_args(argv)
    session = A.session or os.path.join(
        ROOT, "hw_sessions", _dt.date.today().isoformat(),
        "gain_bench_rehearsal" if A.rehearse else "gain_bench")
    os.makedirs(session, exist_ok=True)
    logf = open(os.path.join(session, "session.log"), "a")

    def log(text: str) -> None:
        logf.write(text + "\n")
        logf.flush()

    if A.cmd == "report":
        md = render_report(session)
        print(md)
        if A.md:
            with open(A.md, "w") as f:
                f.write(md + "\n")
        return 0

    op = Operator()
    log(f"# {_dt.datetime.now().isoformat(timespec='seconds')} {' '.join(sys.argv)}")
    try:
        if A.cmd == "stance":
            if A.rehearse:
                raise SystemExit("stance drives the whole robot through squat_bench: no rehearsal")
            return run_stance(A, session, op, log)
        if A.cmd == "scan":
            return run_scan(A, session, op, log)
        return run_servo(A, session, op, log)
    except (Abort, ValueError) as e:
        op.say(f"\n!! ABORT: {e}")
        log(f"!! ABORT: {e}")
        return 2


def run_scan(A, session: str, op: Operator, log) -> int:
    if not A.rehearse:
        op.say("opening the tether: the board reboots, and its boot releases every servo")
    board = FakeBoard(log=log) if A.rehearse else Board(find_port(A.port), log)
    try:
        found, out = [], ""
        for _ in range(2):                  # the boot may have eaten the first one
            out = board.cmd("scan", until=r"\d+ servo\(s\)|busy: |\? \(try", timeout=30.0)
            if BUSY_RE.search(out):
                raise Abort("the control loop owns the bus -- `bench` it by hand first")
            if re.search(r"\d+ servo\(s\)", out):
                found = parse_scan(out)
                break
        else:
            raise Abort(f"no scan reply: {out.strip()[-80:]!r}")
    finally:
        board.close()
    for f in found:
        op.say(f"  id {f['id']:3d}  pos {f['pos']:5d}  {f['volt']:.1f} V  "
               f"status {status_names(f['err'])}  {f['joint']}")
    op.say(f"{len(found)} servo(s)")
    on_map = [f["id"] for f in found if 1 <= f["id"] <= len(BUS)]
    if len(found) == 1 and on_map:
        op.say(f"  id {on_map[0]} is on the robot's bus map: re-ID it at the CLI before "
               f"stiffness/hold (`id {on_map[0]} 30`)")
    print("saved", _saver(session, "scan.json")({"servos": found}))
    return 0


def run_servo(A, session: str, op: Operator, log) -> int:
    """read / stiffness / hold: one servo on the bench."""
    sid = A.id
    if 1 <= sid <= len(BUS):
        if A.cmd != "read":
            raise Abort(f"id {sid} is the robot's {BUS[sid - 1][0]}: the firmware clamps a "
                        f"`move` to that joint's calibrated envelope after sending it, so "
                        f"the lever could be driven somewhere else. Re-ID the spare first "
                        f"(`id {sid} 30` at the CLI, one servo on the bus).")
        op.say(f"note: id {sid} is the robot's {BUS[sid - 1][0]}; re-ID it (`id {sid} 30`) "
               f"before stiffness/hold.")
    ladder = parse_ladder(A.ladder, A.d_ratio) if A.cmd != "read" else []
    if not A.rehearse:
        op.say("opening the tether: the board reboots, and its boot releases every servo")
    board = (FakeBoard(sid=sid, latency_s=0.02, log=log) if A.rehearse   # ~ the tether's poll rate
             else Board(find_port(A.port), log))
    settle = 0.2 if A.rehearse else getattr(A, "settle", 0.0)
    start = None
    try:
        connect(board, sid)
        start = read_gains(board, sid)
        if A.cmd == "read":                       # reads only, start to finish
            print("saved", _saver(session, "read.json")(run_read(board, op, sid)))
            return 0
        if A.cmd == "stiffness":
            save = _saver(session, "stiffness.json", start_gains=list(start))
            save(run_stiffness(board, op, sid, ladder, A.lever_m,
                               [float(x) for x in A.loads.split(",")], A.samples, settle,
                               on_row=save))
        else:
            save = _saver(session, "hold.json", start_gains=list(start))
            save(run_hold(board, op, sid, ladder, A.orient.split(","), A.quiet_s, A.step,
                          A.step_s, settle, A.inertia, on_row=save))
        print(render_report(session))
        return 0
    finally:
        if A.cmd != "read":
            try:
                release(board, sid)
                if start and not A.keep:
                    write_gains(board, sid, start[0], start[1])
                    op.say(f"id {sid}: released; gains back to P {start[0]} D {start[1]}")
            except Abort as e:
                op.say(f"!! could not restore the gains: {e} -- `gains {sid}` by hand")
        board.close()


def run_stance(A, session: str, op: Operator, log) -> int:
    side, rid = A.roll, roll_id(A.roll)
    ladder = parse_ladder(A.ladder, A.d_ratio)
    squat = os.path.join(ROOT, "tools", "squat_bench.py")
    res = {"side": side, "roll_id": rid, "phi": A.phi, "lift": A.lift, "ladder": ladder, "rows": []}

    def set_gains(p: int, d: int) -> None:
        board = Board(find_port(A.port), log)
        try:
            connect(board, rid)
            release(board)                          # every servo: the robot is supported
            write_gains(board, rid, p, d)
        finally:
            board.close()

    op.go("robot SUPPORTED (stand or spotter): opening the tether reboots the board, and "
          "its boot releases EVERY servo")
    board = Board(find_port(A.port), log)
    try:
        connect(board, rid)
        start = read_gains(board, rid)
    finally:
        board.close()
    op.say(f"{side}_hip_roll id {rid} starts at P {start[0]} D {start[1]}")
    save = _saver(session, "stance.json", start_gains=list(start))
    rc = 0
    try:
        for p, d in ladder:
            op.go(f"robot SUPPORTED: torque off on EVERY servo, then P {p} D {d} on "
                  f"{side}_hip_roll (id {rid})")
            set_gains(p, d)
            out = os.path.join(session, f"stance_P{p}_D{d}")
            op.go(f"squat_bench stands the robot and rolls the {side} hip {A.phi:g}° with "
                  f"the foot planted (P {p} D {d})")
            cmd = [sys.executable, squat, out, "--balance", f"{side}:{A.phi:g}:{A.lift:g}",
                   "--balance-mode", "stance"] + list(A.squat_args)
            log("$ " + " ".join(cmd))
            src = subprocess.call(cmd)
            row = {"p": p, "d": d, "squat_rc": src, "csv": os.path.relpath(out + ".csv", ROOT)}
            if os.path.exists(out + ".csv"):
                row["result"] = stance_from_csv(out + ".csv", side, A.phi)
                r = row["result"]
                op.say(f"  P {p} D {d}: {side}_hip_roll short {r['short']:.2f}° at load "
                       f"{r['load']} -- {stance_verdict(r['short'])}")
            else:
                row["note"] = f"squat_bench rc {src}, no CSV"
            res["rows"].append(row)
            save(res)
            if src:
                rc = 2
                op.say(f"!! squat_bench exited {src}: stopping the ladder")
                break
    except Abort as e:
        op.say(f"\n!! ABORT: {e}")
        rc = 2
    finally:
        print("saved", save(res))
        try:
            op.go(f"robot SUPPORTED: torque off on every servo to put {side}_hip_roll back "
                  f"to P {start[0]} D {start[1]}")
            set_gains(start[0], start[1])
            op.say(f"{side}_hip_roll back to P {start[0]} D {start[1]}")
        except Abort as e:
            op.say(f"!! {side}_hip_roll NOT restored ({e}): the arm gate will refuse until "
                   f"`gains {rid} {start[0]} {start[1]}`")
            rc = 2
    print(render_report(session))
    return rc


if __name__ == "__main__":
    sys.exit(main())
