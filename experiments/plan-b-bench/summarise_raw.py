"""Per-run summaries for the bench runs whose scripts did not write one.

Reads the raw session files under <date>/raw/ and writes, beside them:

  2026-09-30/bare_ladder_summary.json   session 1: registers as found + the unloaded P ladder
  2026-10-01/one_bottle_summary.json    session 2, run A: one bottle, two directions
  2026-10-01/two_bottle_summary.json    session 2, run B: two bottles at P 32 / 128 / 160
  2026-10-02/calibration_summary.json   session 3: the holding-load level search (153 ticks off)
  2026-10-02/bno_check_summary.json     session 3: the accelerometer at rest, before the sweeps

The statistics are gain_bench's own (window_stats, hold_row), so these match
the tables in the dated records. Run from the repo root:

    .venv/bin/python experiments/plan-b-bench/summarise_raw.py
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gain_bench import hold_row, window_stats  # noqa: E402


def load(rel):
    with open(os.path.join(HERE, rel)) as f:
        return json.load(f)


def save(rel, obj):
    path = os.path.join(HERE, rel)
    with open(path, "w") as f:
        json.dump(obj, f, indent=1)
        f.write("\n")
    print("wrote", os.path.relpath(path))


def r1(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(x, 2)


def step_brief(s):
    if not s.get("n"):
        return None
    return {"overshoot": r1(s["overshoot"]), "settle_s": r1(s["settle_s"]),
            "residual_range": s["residual_range"], "offset": r1(s["offset"]), "err": s["err"]}


def bare_ladder():
    raw = "2026-09-30/raw/"
    read = load(raw + "gain_bench/read.json")
    hold = load(raw + "gain_bench_bare/hold.json")
    first = load(raw + "gain_bench_bare/hold.112838.json")
    rows = []
    for row in hold["rows"]:
        h = hold_row(row)
        q = h["quiet"]
        rows.append({"p": h["p"], "d": h["d"], "gain_write": row.get("gain_write"),
                     "goal": row["goal"],
                     "hold": {"range": q["range"], "moving": q["moving"], "n": q["n"],
                              "load_min": q["load_min"], "load_max": q["load_max"],
                              "temp_max": q["temp_max"], "err": q["err"]},
                     "step_out": step_brief(h["out"]), "step_back": step_brief(h["back"]),
                     "quiet": h["is_quiet"]})
    return {
        "session": "2026-09-30, session 1: bare horn, no lever, no masses",
        "id": hold["id"], "inertia": hold["inertia"], "step_ticks": hold["step_ticks"],
        "registers_as_found": {"gains_pdi": read["gains"], **read["regs"]},
        "position_as_found": read["pos"],
        "rows": rows,
        "gain_write_acks": [r["gain_write"] for r in rows],
        "first_attempt": {"file": "raw/gain_bench_bare/hold.112838.json",
                          "rows": len(first.get("rows", [])),
                          "stopped": first.get("stopped")},
        "source": [raw + "gain_bench/read.json", raw + "gain_bench_bare/hold.json"],
    }


def one_bottle():
    raw = "2026-10-01/raw/gain_bench_onebottle/one_bottle.json"
    d = load(raw)
    g = d["goal"]
    rows = []
    for row in d["rows"]:
        rows.append({
            "p": row["p"], "d": row["d"], "gain_write": row.get("gain_write"),
            "from_above": [{"ticks": r1(w["pos_mean"] - g), "load": r1(w["load_mean"]),
                            "range": w["range"], "err": w["err"]} for w in row["plus"]],
            "from_below": [{"ticks": r1(w["pos_mean"] - g), "load": r1(w["load_mean"]),
                            "range": w["range"], "err": w["err"]} for w in row["minus"]],
        })
    return {
        "session": "2026-10-01, session 2, run A (12:19-12:24)",
        "id": d["id"], "goal": g, "approach_ticks": d["approach"], "speed": d["spd"],
        "settle_s": d["settle_s"], "load": d["load"],
        "note": "ticks are position minus goal; + is lower (the load side). The script's "
                "printed k assumed the full bottle torque with no friction and is not used.",
        "rows": rows, "source": [raw],
    }


def two_bottle():
    raw = "2026-10-01/raw/gain_bench_2bottle/"
    out = {"session": "2026-10-01, session 2, run B (12:39-12:53)", "rows": [], "runs": []}
    for name in ("two_bottle.json", "two_bottle_level.json"):
        d = load(raw + name)
        g = d["goal"]
        out["id"] = d["id"]
        out["goal"] = g
        out["bottle_kg_nominal"] = d["bottle_kg_nominal"]
        out["lever_m"] = d["lever_m"]
        out["runs"].append({"file": "raw/gain_bench_2bottle/" + name,
                            "stopped": d.get("stopped")})
        for row in d["rows"]:
            states = []
            for st in row["steps"]:
                s = st.get("stats") or window_stats(st.get("samples", []))
                states.append({"bottles": st["bottles"], "tau_nominal": r1(st["tau_nominal"]),
                               "ticks": r1(s["pos_mean"] - g), "load": r1(s["load_mean"]),
                               "range": s["range"], "moving": s["moving"], "n": s["n"],
                               "err": s["err"]})
            out["rows"].append({"p": row["p"], "d": row["d"], "gain_write": row.get("gain_write"),
                                "script": name, "states": states})
    out["note"] = ("states run bare, 1 bottle, 2 bottles, bottles off; ticks are position minus "
                   "goal, + is lower. P 160 was written with the bare arm level "
                   "(two_bottle_level.py), on Tom's 'no need to lower. just write'.")
    out["source"] = [raw + "two_bottle.json", raw + "two_bottle_level.json"]
    return out


def calibration():
    raw = "2026-10-02/raw/gain_bench_hold/console.txt"
    text = open(os.path.join(HERE, raw)).read()
    block = text.split("# 2026-10-02T10:02:27")[0]
    probe = re.search(r"gravity probe at (\d+): from \+40 err/load \(([-\d.]+), ([-\d.]+)\), "
                      r"from -40 err/load \(([-\d.]+), ([-\d.]+)\)", block)
    steps = [{"target": int(a), "pos": float(b), "abs_load": int(c)}
             for a, b, c in re.findall(r"step to (\d+): pos ([\d.]+), \|load\| (\d+)", block)]
    lvl = re.search(r"LEVEL = (\d+) \(peak \|load\| (\d+)\); DOWN = (\d+)", block)
    travel = re.search(r"travel 2497->3684: (.*)", block)
    rel = re.search(r"released hanging: pos (\d+)", block)
    return {
        "session": "2026-10-02, session 3, calibration (09:47-09:53)",
        "method": "probe +-40 ticks at the rest position to find gravity's side, then step 64 "
                  "ticks at a time against gravity reading the holding load; level = vertex of "
                  "a parabola through the top three loads",
        "rest": int(probe.group(1)),
        "probe_from_plus40": {"err": float(probe.group(2)), "load": float(probe.group(3))},
        "probe_from_minus40": {"err": float(probe.group(4)), "load": float(probe.group(5))},
        "steps": steps,
        "level_found": int(lvl.group(1)), "peak_abs_load": int(lvl.group(2)),
        "plumb_found": int(lvl.group(3)),
        "lowering_to_plumb_found": travel.group(1).strip(),
        "released_at": int(rel.group(1)),
        "plumb_by_eye": 3531, "level_by_eye": 2507,
        "error_ticks": int(lvl.group(3)) - 3531,
        "note": "the lowering toward the found plumb pushed the fork into the stand's C-clamp "
                "(load -388); the per-step JSON of this attempt was overwritten by the 10:02 "
                "run, so the console is the only record",
        "source": [raw],
    }


def bno_check():
    raw = "2026-10-02/raw/bno_setup/check.log"
    text = open(os.path.join(HERE, raw)).read()
    dump = text.split(">>> bno dump")[1].split("bno dump end")[0]
    xs, ys, zs = [], [], []
    for line in dump.splitlines():
        parts = line.split()
        if len(parts) == 4 and all(p.lstrip("-").isdigit() for p in parts):
            xs.append(int(parts[1]))
            ys.append(int(parts[2]))
            zs.append(int(parts[3]))
    n = len(xs)
    mag = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(xs, ys, zs)]

    def ms(v):
        m = sum(v) / len(v)
        return {"mean": round(m, 1), "sd": round(math.sqrt(sum((a - m) ** 2 for a in v) / len(v)), 2)}

    return {
        "session": "2026-10-02, session 3, accelerometer setup (14:28-14:40)",
        "sequence": [
            "14:28 old firmware on the new controller: no `bno` command; `imu scan` finds 0x4B",
            "14:31 flashed main + bench `bno` (1d86445, BNO055 driver): `bno: no reply at 0x28`",
            "14:39 flashed 2e9516b (`bno` reads a BNO08x over SHTP): BNO08x at 0x4B, READY",
        ],
        "i2c_devices": ["0x0C AK09918C", "0x42 INA219", "0x4B BNO08x (the bench breakout)",
                        "0x6B QMI8658C"],
        "at_rest_mg": {"n": n, "x": ms(xs), "y": ms(ys), "z": ms(zs), "abs_a": ms(mag)},
        "firmware_summary_line": re.search(r"SUMMARY .*", text).group(0),
        "note": "the firmware's 'sd x 4.5' is the x axis only; |a| varies less",
        "source": [raw],
    }


if __name__ == "__main__":
    save("2026-09-30/bare_ladder_summary.json", bare_ladder())
    save("2026-10-01/one_bottle_summary.json", one_bottle())
    save("2026-10-01/two_bottle_summary.json", two_bottle())
    save("2026-10-02/calibration_summary.json", calibration())
    save("2026-10-02/bno_check_summary.json", bno_check())
