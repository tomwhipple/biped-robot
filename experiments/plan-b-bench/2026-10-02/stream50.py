"""The control loop's 50 Hz streaming, on the bench (issue #73), 2026-10-02.

Firmware `stream` (cli.cpp) sweeps one servo along a minimum-jerk curve from
the BOARD at 50 Hz -- the rate the walking firmware streams at:
  mode 0  the control loop's rule: target on the curve now, goal speed
          obs::goalSpeedSteps(target, present) (gap / 20 ms x 1.25), accel 0
  mode 1  matched: target `lead` ms ahead, goal speed = the curve's own speed,
          acceleration limit `acc`
The BNO08x on the fork records throughout (`bno rec`); the board logs target,
position, speed and load every tick (`stream dump`).

Per gain: written at plumb with the arm released, torque on, hold plumb; then
for each (mode, duration): raise plumb -> level, lower level -> plumb. Release at
plumb. Weights stay on; nothing is released off plumb.
  LEVEL_T=2507 DOWN_T=3531 .venv/bin/python experiments/plan-b-bench/2026-10-02/stream50.py
"""
import json, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import gain_bench as GB
import sweep_smooth as SS

SID = 30
LEVEL = int(os.environ.get("LEVEL_T", 2507))
DOWN = int(os.environ.get("DOWN_T", 3531))
GAINS = [tuple(int(v) for v in x.split(":")) for x in
         os.environ.get("GAINS", "32:32,96:0,128:0").split(",")]
# (label, mode, duration ms, lead ms, acc register): T 4 s peaks ~480 steps/s
# (the rolls' walk speeds), T 2 s ~960 steps/s (toward the knee's)
CASES = [("robot rule, 4 s", 0, 4000, 0, 0), ("matched, 4 s", 1, 4000, 40, 6),
         ("robot rule, 2 s", 0, 2000, 0, 0), ("matched, 2 s", 1, 2000, 40, 23)]
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "stream50")
os.makedirs(OUT, exist_ok=True)
SS.OUT = OUT
logf = open(os.path.join(OUT, "session.log"), "a")
SS.logf = logf


def log(s):
    logf.write(s + "\n"); logf.flush()


def say(s):
    print(s, flush=True); log(s)


SS.say = say


def stream(b, frm, to, case):
    _, mode, ms, lead, acc = case
    out = b.cmd(f"stream {SID} {frm} {to} {ms} {mode} {lead} {acc}",
                until=r"stream: .*\n|busy: .*\n|\? \(try", timeout=3)
    if "started" not in out:
        raise GB.Abort(f"stream failed: {out.strip()[-100:]!r}")
    t_end = time.monotonic() + ms / 1000 + 4
    while time.monotonic() < t_end:
        time.sleep(0.3)
        if "idle" in b.cmd("stream status", until=r"stream: .*\n", timeout=2):
            break
    dump = b.cmd("stream dump", until=r"stream dump end\r?\n|stream: .*\n", timeout=20)
    rows = [list(map(int, l.split())) for l in dump.splitlines()
            if re.fullmatch(r"\d+ -?\d+ -?\d+ -?\d+ -?\d+", l.strip())]
    return rows


def stream_metrics(rows, ms):
    mov = [r for r in rows if r[0] <= ms]
    if not mov:
        return {}
    err = [r[2] - r[1] for r in mov if r[2] >= 0]
    dirn = 1 if rows[-1][1] > rows[0][1] else -1
    back = sum(1 for r in mov if r[3] * dirn < 0)
    tail = [r[2] for r in rows if r[0] > ms + 200 and r[2] >= 0]
    return {"ticks": len(rows), "track_rms": round((sum(e * e for e in err) / len(err)) ** 0.5, 2),
            "track_max": max(abs(e) for e in err), "backward": back,
            "end_range": (max(tail) - min(tail)) if tail else None,
            "load_min": min(r[4] for r in mov), "load_max": max(r[4] for r in mov)}


def main():
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} stream50.py id {SID} LEVEL {LEVEL} DOWN {DOWN} "
        f"gains {GAINS} cases {[c[0] for c in CASES]}")
    b = GB.Board(GB.find_port(None), log)
    res = {"level": LEVEL, "down": DOWN, "cases": CASES, "rows": []}
    torque = False
    try:
        p0 = GB.read_pos(b, SID, tries=4)["pos"]
        if GB.read_reg(b, SID, 40) != 0 or abs(p0 - DOWN) > 60:
            raise GB.Abort(f"start refused: pos {p0}; need torque off, hanging within 60 of {DOWN}")
        out = b.cmd("bno", until=r"bno: .*\n|\? \(try", timeout=5)
        if "READY" not in out:
            # a board reset mid-read can leave the BNO08x holding SDA (seen
            # 2026-10-02 16:32 after a flash): clock the bus free and retry
            say(f"  {out.strip()} -- trying `imu reinit`")
            b.cmd("imu reinit", until=r"imu reinit: .*\n|\? \(try", timeout=5)
            out = b.cmd("bno", until=r"bno: .*\n|\? \(try", timeout=5)
        if "READY" not in out:
            raise GB.Abort(f"accelerometer not ready: {out.strip()[-100:]!r}")
        say(f"  {out.strip()}")
        for p, d in GAINS:
            say(f"\n== P {p} D {d}")
            if GB.read_gains(b, SID)[:2] != (p, d):
                say(f"  gains {p}/{d}: {GB.write_gains(b, SID, p, d)}")
            before = GB.read_pos(b, SID)["pos"]
            b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); torque = True
            if abs(GB.read_pos(b, SID)["pos"] - before) > GB.JUMP_TICKS:
                raise GB.Abort("jump at torque-on")
            b.cmd(f"move {SID} {DOWN} 0 100 2", until=GB.MOVE_UNTIL); time.sleep(1.5)
            for case in CASES:
                for frm, to, way in ((DOWN, LEVEL, "raise"), (LEVEL, DOWN, "lower")):
                    bno_t = SS.bno_start(b, case[2] + 2000)
                    t_s = time.monotonic()
                    rows = stream(b, frm, to, case)
                    acc = SS.bno_collect(b)
                    m = stream_metrics(rows, case[2])
                    am = SS.bno_metrics(acc)
                    res["rows"].append({"p": p, "d": d, "case": case[0], "way": way, "stream": rows,
                                        "metrics": m, "bno": acc, "bno_metrics": am,
                                        "bno_host_t0": bno_t, "stream_host_t0": t_s})
                    say(f"  {case[0]:16s} {way:5s}: accel y/x {am.get('rms_y_mg')}/{am.get('rms_x_mg')} mg "
                        f"(peak {am.get('peak_y_hz')}/{am.get('peak_x_hz')} Hz) | track rms {m.get('track_rms')} "
                        f"max {m.get('track_max')} | backward {m.get('backward')}/{m.get('ticks')} | "
                        f"end range {m.get('end_range')} | load {m.get('load_min')}..{m.get('load_max')}")
                    time.sleep(1.0)
            pos = GB.read_pos(b, SID)["pos"]
            if abs(pos - DOWN) > 60:
                raise GB.Abort(f"not at plumb before release: {pos}")
            GB.release(b, SID); torque = False
            time.sleep(1.5)
            say(f"  released at plumb: pos {GB.read_pos(b, SID)['pos']}")
        if GB.read_gains(b, SID)[:2] != (32, 32):
            GB.write_gains(b, SID, 32, 32)
        say(f"  gains back to {GB.read_gains(b, SID)[:2]}")
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        res["stopped"] = repr(e)
        try:
            if torque:
                pos = GB.read_pos(b, SID)["pos"]
                if abs(pos - DOWN) <= 60:
                    GB.release(b, SID); say(f"  released at plumb: {pos}")
                else:
                    say(f"!! HOLDING TORQUE at {pos} (weights on, off plumb): "
                        f"`move {SID} {DOWN} 0 100 2`, then `release {SID}`")
        except Exception as e2:
            say(f"!! could not check/release: {e2!r}")
    finally:
        with open(os.path.join(OUT, "stream50.json"), "w") as f:
            json.dump(res, f)
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
