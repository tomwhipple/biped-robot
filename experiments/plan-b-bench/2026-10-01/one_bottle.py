"""One-bottle stiffness ratio (issue #73), 2026-10-01. A reduced protocol,
NOT gain_bench.py's `stiffness`: Tom has one ~12 fl oz bottle hung at the
100 mm notch and no scale, so there is no bare-lever start and no load
ladder. Per P: hold the goal G with the bottle on, approach G from +A and
from -A (twice each), and read the steady position. The mean offset from G
is the load sag (friction cancels between the two approaches); half the
difference is the friction band. Stiffness RATIOS vs P 32 need no mass.
Reuses gain_bench's guarded gains write and torque-on (hold_at).

Run from the repo root:
  .venv/bin/python hw_sessions/2026-10-01/gain_bench_onebottle/one_bottle.py
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # experiments/plan-b-bench: gain_bench
# raw output goes where the other bench scripts put it: hw_sessions/<date>/gain_bench_onebottle/ under the cwd
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "gain_bench_onebottle")
os.makedirs(OUT, exist_ok=True)
import gain_bench as GB

SID, A = int(os.environ.get("SID", 11)), 40
G = int(os.environ.get("LEVEL_T", 2057))   # this rig's level
LADDER = [(32, 32), (64, 64), (96, 96), (128, 128), (160, 160), (32, 32)]
SPD, SETTLE_S, SAMPLE_S, REPS = 100, 6.0, 3.0, 2
log_f = open(os.path.join(OUT, "session.log"), "a")


def log(s):
    log_f.write(s + "\n"); log_f.flush()


def move(board, to):
    out = board.cmd(f"move {SID} {to} 0 {SPD}", until=GB.MOVE_UNTIL)
    if "clamped" in out or "outside" in out or "move id" not in out or ": ok" not in out:
        raise GB.Abort(f"move {to}: {out.strip()[-100:]!r}")


def steady(board):
    time.sleep(SETTLE_S)
    ss = GB.sample(board, SID, seconds=SAMPLE_S)
    w = GB.window_stats(ss)
    return {"pos_mean": w["pos_mean"], "range": w["range"], "load_mean": w["load_mean"],
            "err": w["err"], "n": len(ss)}


def main():
    log(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} one_bottle.py G {G} A {A} ladder {LADDER}")
    op = GB.Operator(ask=lambda prompt: (log(prompt.strip()), "go")[1],
                     say=lambda s: (print(s, flush=True), log(s)))
    board = GB.Board(GB.find_port(None), log)
    res = {"id": SID, "goal": G, "approach": A, "spd": SPD, "settle_s": SETTLE_S,
           "load": "one ~12 fl oz Gatorade bottle (unweighed, ~0.39-0.40 kg est.) at 100 mm",
           "rows": []}
    try:
        GB.read_pos(board, SID, tries=4)
        start = GB.read_gains(board, SID)
        for p, d in LADDER:
            wrote = GB.write_gains(board, SID, p, d)
            GB.hold_at(board, op, SID, G, f"P {p} D {d}: torque on at {G}", 3.0)
            row = {"p": p, "d": d, "gain_write": wrote, "plus": [], "minus": []}
            for _ in range(REPS):
                for side, off in (("plus", +A), ("minus", -A)):
                    move(board, G + off); time.sleep(2.0)
                    move(board, G)
                    row[side].append(steady(board))
            GB.release(board, SID)
            mp = sum(r["pos_mean"] for r in row["plus"]) / REPS
            mm = sum(r["pos_mean"] for r in row["minus"]) / REPS
            row["sag_ticks"] = (mp + mm) / 2 - G
            row["band_ticks"] = (mp - mm) / 2
            row["load_mean"] = sum(r["load_mean"] for r in row["plus"] + row["minus"]) / (2 * REPS)
            row["max_range"] = max(r["range"] for r in row["plus"] + row["minus"])
            row["err"] = max(r["err"] for r in row["plus"] + row["minus"])
            res["rows"].append(row)
            op.say(f"P {p:3d} D {d:3d}: sag {row['sag_ticks']:+6.2f} ticks, friction band "
                   f"+-{abs(row['band_ticks']):.2f}, from+ {mp:.2f} from- {mm:.2f}, load "
                   f"{row['load_mean']:.0f}, max range {row['max_range']}, err {row['err']} "
                   f"({wrote}) -- released")
    except GB.Abort as e:
        op.say(f"!! ABORT: {e}")
        try:
            GB.release(board, SID)
        except Exception:
            pass
        res["abort"] = str(e)
    finally:
        try:
            GB.release(board, SID)
            if GB.read_gains(board, SID)[:2] != (32, 32):
                GB.write_gains(board, SID, 32, 32)
            op.say(f"id {SID}: released; gains {GB.read_gains(board, SID)[:2]}")
        finally:
            board.close()
            with open(os.path.join(OUT, "one_bottle.json"), "w") as f:
                json.dump(res, f, indent=1)
    if res["rows"]:
        ref = res["rows"][0]["sag_ticks"]
        tau = 0.395 * GB.G * 0.100
        for r in res["rows"]:
            k = tau / (abs(r["sag_ticks"]) / GB.TICKS_PER_RAD) if r["sag_ticks"] else float("inf")
            op.say(f"  P {r['p']:3d}: x P32 {ref / r['sag_ticks'] if r['sag_ticks'] else float('inf'):5.2f}"
                   f"   k ~{k:5.1f} N*m/rad (tau ~{tau:.2f} N*m, unweighed)")


if __name__ == "__main__":
    main()
