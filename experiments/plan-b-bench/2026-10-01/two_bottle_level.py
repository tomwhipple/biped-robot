"""LEVEL variant (Tom 12:5x: "no need to lower. just write. go"): gains are
written with the BARE arm level at GOAL (released there; Tom: the bare level
arm only falls with a bottle attached), no lowering. Otherwise as two_bottle.py.

Two-bottle stiffness, servo-positioned (issue #73), 2026-10-01, on Tom's
"go. now" before gain_bench's --goal/--rest mode landed. Same spec as sent to
the Plan B session: REST = lever hanging straight down (the ONLY place it is
released and gains are written), GOAL = straight out. Per rung: gains at REST
-> go -> torque on (jump check) -> REST -> GOAL at 100 steps/s -> bare sample
-> hang 1 -> sample -> hang 2 -> sample -> bag off -> sample -> go -> REST at
100 steps/s -> release. Any abort off REST HOLDS torque. id 30, so no
envelope clamp. Reuses gain_bench's write_gains/read_pos/sample/window_stats.
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # experiments/plan-b-bench: gain_bench
# raw output goes where the other bench scripts put it: hw_sessions/<date>/gain_bench_2bottle/ under the cwd
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "gain_bench_2bottle")
os.makedirs(OUT, exist_ok=True)
import gain_bench as GB

SID, TOL, SPD = 30, 150, 100
GOAL = int(os.environ.get("LEVEL_T", 2057)); REST = int(os.environ.get("DOWN_T", 3081))   # this rig's level / plumb
LADDER = [(160, 160)]
BOTTLE_KG, LEVER_M, N, SETTLE = 0.39, 0.100, 20, 5.0
logf = open(os.path.join(OUT, "session.log"), "a")


def log(s):
    logf.write(s + "\n"); logf.flush()


def say(s):
    print(s, flush=True); log(s)


def ask(prompt):
    say(prompt)
    a = input("> ").strip()
    log(f"< {a!r}")
    return a


def go(what):
    if ask(f"\nNEXT MOTION: {what}\n  type go:").lower() != "go":
        raise GB.Abort(f"no go for: {what}")


def move(b, to, wait_s):
    out = b.cmd(f"move {SID} {to} 0 {SPD}", until=GB.MOVE_UNTIL)
    if ": ok" not in out or "clamp" in out:
        raise GB.Abort(f"move {to}: {out.strip()[-90:]!r}")
    time.sleep(wait_s)
    return GB.read_pos(b, SID)["pos"]


def at_rest(b):
    return abs(GB.read_pos(b, SID)["pos"] - REST) <= TOL


def window(b):
    ss = GB.sample(b, SID, n=N)
    return GB.window_stats(ss), ss


def main():
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} two_bottle_level.py id {SID} GOAL {GOAL} REST {REST} ladder {LADDER}")
    b = GB.Board(GB.find_port(None), log)
    res = {"id": SID, "goal": GOAL, "rest": REST, "bottle_kg_nominal": BOTTLE_KG,
           "lever_m": LEVER_M, "rows": []}
    torque_on = False
    try:
        p0 = GB.read_pos(b, SID, tries=4)["pos"]
        if abs(p0 - GOAL) > 20:
            raise GB.Abort(f"start refused: pos {p0}, need the bare arm within 20 of GOAL {GOAL}")
        for p, d in LADDER:
            say(f"\n== P {p} D {d}")
            wrote = GB.write_gains(b, SID, p, d)          # at REST, released
            say(f"  gains {p}/{d}: {wrote}")
            go(f"P {p}: torque on at level, then hold straight out ({GOAL}) at {SPD} steps/s")
            before = GB.read_pos(b, SID)["pos"]
            b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); torque_on = True
            after = GB.read_pos(b, SID)["pos"]
            if abs(after - before) > GB.JUMP_TICKS:
                raise GB.Abort(f"jumped {after - before} at torque-on")
            pos = move(b, GOAL, SETTLE)
            say(f"  at goal: pos {pos}")
            row = {"p": p, "d": d, "gain_write": wrote, "steps": []}
            for n_b, prompt in ((0, None), (1, "HANG 1 bottle at the 100 mm notch"),
                                (2, "HANG the 2nd bottle"), (0, "LIFT the bag/bottles OFF")):
                if prompt:
                    ask(f"\n{prompt}; press enter when done")
                    time.sleep(SETTLE)
                w, ss = window(b)
                tau = n_b * BOTTLE_KG * GB.G * LEVER_M
                row["steps"].append({"bottles": n_b, "tau_nominal": tau, "stats": w, "samples": ss})
                say(f"  {n_b} bottle(s): pos {w['pos_mean']:.2f} (dev {w['pos_mean'] - GOAL:+.2f}), "
                    f"range {w['range']}, load {w['load_mean']:.0f}, err {w['err']}")
            res["rows"].append(row)
            GB.release(b, SID); torque_on = False       # bare, level (Tom)
            time.sleep(2.0)
            say(f"  released level, bare: pos {GB.read_pos(b, SID)['pos']}")
        if abs(GB.read_pos(b, SID)["pos"] - GOAL) <= 20:
            GB.write_gains(b, SID, 32, 32)
            say(f"  gains back to {GB.read_gains(b, SID)[:2]}")
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        pos = GB.read_pos(b, SID)["pos"]
        if torque_on and abs(pos - REST) > TOL:
            say(f"!! HOLDING TORQUE at pos {pos} (off rest). Lift any load off, then "
                f"lower with: move {SID} {REST} 0 {SPD} -- do NOT release here.")
            input("press enter only when the arm is safe to leave HELD; the port stays open > ")
        elif torque_on:
            GB.release(b, SID)
            say(f"  released at rest: pos {pos}")
        res["stopped"] = repr(e)
    finally:
        with open(os.path.join(OUT, "two_bottle_level.json"), "w") as f:
            json.dump(res, f, indent=1)
        for r in res["rows"]:
            dev = [s["stats"]["pos_mean"] - GOAL for s in r["steps"]]
            say(f"P {r['p']:3d}: dev bare {dev[0]:+.2f}, 1b {dev[1]:+.2f}, 2b {dev[2]:+.2f}, "
                f"return {dev[3]:+.2f}  (1b-bare {dev[1]-dev[0]:+.2f}, 2b-1b {dev[2]-dev[1]:+.2f})"
                if len(dev) == 4 else f"P {r['p']}: partial {dev}")
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
