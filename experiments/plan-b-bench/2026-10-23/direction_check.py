"""Which way is "out" on the STS3250 (issue #73), 2026-10-23, v3 rig, washer
weights at r 70 and 100.

The STS3250 rests plumb at a new tick count (1254; the STS3215 was 3531), so
level is plumb -1024 or +1024 and only one of those swings the fork away from
the desk -- the other drives it into the table frame a few cm behind it. On
the STS3215 v3 rig out was the minus side; this checks it on camera before
any 90-degree raise: torque on at plumb, -A, +A, back to plumb, release there.
At plumb gravity holds no torque, so the +-A moves load nothing; the load at
each side says which way gravity pulls, the video which way is out.

Run from the repo root, with tools/sony_record.sh filming:
  SID=31 DOWN_T=1254 .venv/bin/python experiments/plan-b-bench/2026-10-23/direction_check.py
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gain_bench as GB

SID = int(os.environ.get("SID", 31))
DOWN = int(os.environ.get("DOWN_T", 1254))
A, SLOW, ACC, DOWN_TOL, HOLD_S = 45, 100, 2, 60, 3.0
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "direction_check")
os.makedirs(OUT, exist_ok=True)
logf = open(os.path.join(OUT, "session.log"), "a")
state = {"torque": False}


def log(s):
    logf.write(s + "\n"); logf.flush()


def say(s):
    print(s, flush=True); log(s)


def go(what):
    say(f"\nNEXT MOTION: {what}\n  type go:")
    a = input("> ").strip(); log(f"< {a!r}")
    if a.lower() != "go":
        raise GB.Abort(f"no go for: {what}")


def move(b, to, spd=SLOW, acc=ACC):
    out = b.cmd(f"move {SID} {int(to)} 0 {int(spd)} {int(acc)}", until=GB.MOVE_UNTIL)
    if ": ok" not in out or "clamp" in out:
        raise GB.Abort(f"move {to}: {out.strip()[-90:]!r}")


def dwell(b, label):
    """Hold HOLD_S, then a 10-sample window; wall-clock stamped for the video."""
    time.sleep(HOLD_S)
    w = GB.window_stats(GB.sample(b, SID, n=10))
    utc = time.strftime("%H:%M:%S", time.gmtime())
    say(f"  {utc}Z {label}: pos {w['pos_mean']:.1f}, load {w['load_mean']:+.0f} "
        f"({w['load_min']}..{w['load_max']}), err {w['err']}")
    return w


def main():
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} direction_check.py id {SID} DOWN {DOWN} A {A}")
    b = GB.Board(GB.find_port(None), log)
    res = {"id": SID, "down": DOWN, "A": A, "windows": {}}
    try:
        p0 = GB.read_pos(b, SID, tries=4)["pos"]
        if GB.read_reg(b, SID, 40) != 0:
            raise GB.Abort(f"start refused: torque is on at {p0}")
        if abs(p0 - DOWN) > DOWN_TOL:
            raise GB.Abort(f"start refused: pos {p0} is not within {DOWN_TOL} of plumb {DOWN}")
        res["gains"] = GB.read_gains(b, SID)
        go(f"torque on at {p0} (plumb), {-A:+d} ticks, {A:+d} ticks, back to plumb at "
           f"{SLOW} steps/s, release there")
        before = GB.read_pos(b, SID)["pos"]
        b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); state["torque"] = True
        if abs(GB.read_pos(b, SID)["pos"] - before) > GB.JUMP_TICKS:
            raise GB.Abort("jump at torque-on")
        move(b, p0); res["windows"]["plumb"] = dwell(b, f"plumb {p0}")
        move(b, p0 - A); res["windows"]["minus"] = dwell(b, f"minus {p0 - A}")
        move(b, p0 + A); res["windows"]["plus"] = dwell(b, f"plus {p0 + A}")
        move(b, p0); res["windows"]["back"] = dwell(b, f"back {p0}")
        GB.release(b, SID); state["torque"] = False
        time.sleep(2.0)
        res["released_at"] = GB.read_pos(b, SID)["pos"]
        say(f"  released at plumb: pos {res['released_at']}")
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        res["stopped"] = repr(e)
        try:
            if state["torque"]:
                pos = GB.read_pos(b, SID)["pos"]
                if abs(pos - DOWN) <= DOWN_TOL:      # +-A stays inside this: always safe
                    GB.release(b, SID); state["torque"] = False
                    say(f"  released at {pos}")
                else:
                    say(f"!! HOLDING TORQUE at {pos}, off plumb with the weights on")
                    input("press enter to leave it HELD and close the port > ")
        except Exception as e2:
            say(f"!! could not check/release: {e2!r} -- torque state unknown")
    finally:
        with open(os.path.join(OUT, "direction_check.json"), "w") as f:
            json.dump(res, f, indent=1)
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
