"""Hold test with a rigid inertia (issue #73), 2026-10-01: C-clamps clamped to
the stiffness lever near the 100 mm notch (mass unknown, no scale).

Question: at P 128 / 160 does the STS3215 limit-cycle or buzz with a rigid,
leg-like inertia on it? Two orientations:
  DOWN  lever hanging straight down (no gravity preload: the gear play floats)
  LEVEL lever straight out (preloaded by the clamp's weight)
In each: a quiet hold, then four smooth (ACC-limited) +-A moves (out and back, both
sides), each traced at the bench's full sample rate. At LEVEL also two slow
probes (approach from above / below at 100 steps/s): their midpoint against
session 2's stiffness gives the clamp's torque.

Tom's rules: he never holds the lever; the clamp goes on with the arm held
straight out; nothing is released while the clamp is on unless the lever
hangs within DOWN_TOL of straight down (no gravity torque there); gains are
written with torque off, so only at DOWN; any abort off DOWN with the clamp
on HOLDS torque. Run from the repo root, in a terminal (tmux):
  .venv/bin/python experiments/plan-b-bench/2026-10-01/hold_clamp.py
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gain_bench as GB

SID, LEVEL, DOWN = 30, 2057, 3081          # DOWN = LEVEL + 1024 (90 deg, load pushes +)
# v3 rig (2026-10-02): plumb read off the servo with the fork aligned to a hanging
# string by Tom = 3531, so LEVEL 2507; pass them in rather than editing defaults
LEVEL = int(os.environ.get("LEVEL_T", LEVEL)); DOWN = int(os.environ.get("DOWN_T", DOWN))
LADDER = [tuple(int(v) for v in x.split(":")) for x in os.environ.get(
    "LADDER", "32:32,128:128,160:160").split(",")]
A, SLOW, START_TOL, DOWN_TOL = 23, 100, 60, 60
# Tom 15:41: "the jerkiness of the motion is a real problem" -- every move is
# acceleration-limited (ACC register, 100 steps/s^2 per LSB): ramps, not steps.
ACC, STEP_SPD = 2, 200
QUIET_S, TRACE_S, SETTLE = 5.0, 2.0, 3.0
OUT = os.path.join(os.getcwd(), "hw_sessions", time.strftime("%Y-%m-%d"), "gain_bench_hold")
os.makedirs(OUT, exist_ok=True)
logf = open(os.path.join(OUT, "session.log"), "a")
state = {"torque": False, "clamp": True}       # loaded until the end prompt says off


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


def move(b, to, spd, acc=ACC):
    out = b.cmd(f"move {SID} {to} 0 {spd} {acc}", until=GB.MOVE_UNTIL)
    if ": ok" not in out or "clamp" in out:
        raise GB.Abort(f"move {to}: {out.strip()[-90:]!r}")


TRAVELS = []


def travel(b, to):
    """Slow move between orientations, TRACED (Tom saw shaking on the move):
    pos/spd/load sampled through the travel and the settling after it."""
    at = GB.read_pos(b, SID)["pos"]
    move(b, to, SLOW)
    tr = GB.sample(b, SID, seconds=abs(to - at) / SLOW + SLOW / (ACC * 100.0) + SETTLE)
    dirn = 1 if to > at else -1
    mid = [s for s in tr if s["spd"] != 0]
    sp = [s["spd"] * dirn for s in mid]
    st = {"from": at, "to": to, "n": len(tr),
          "spd_mean": sum(sp) / len(sp) if sp else 0.0,
          "spd_min": min(sp) if sp else 0, "spd_max": max(sp) if sp else 0,
          "spd_sd": (sum((v - sum(sp) / len(sp)) ** 2 for v in sp) / len(sp)) ** 0.5 if sp else 0.0,
          "backward": sum(1 for v in sp if v < 0),
          "load_min": min(s["load"] for s in tr), "load_max": max(s["load"] for s in tr),
          "end": GB.step_stats(tr, at, to)}
    TRAVELS.append({"stats": st, "trace": tr})
    say(f"  travel {at}->{to}: spd {st['spd_mean']:.0f} (sd {st['spd_sd']:.0f}, {st['spd_min']}..{st['spd_max']}), "
        f"backward samples {st['backward']}/{len(mid)}, load {st['load_min']}..{st['load_max']}, "
        f"end overshoot {st['end']['overshoot']:.0f}, residual range {st['end']['residual_range']}, err {st['end']['err']}")
    return GB.read_pos(b, SID)["pos"]


def torque_on(b):
    before = GB.read_pos(b, SID)["pos"]
    b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); state["torque"] = True
    if abs(GB.read_pos(b, SID)["pos"] - before) > GB.JUMP_TICKS:
        raise GB.Abort("jump at torque-on")


def release_ok(b):
    pos = GB.read_pos(b, SID)["pos"]
    return (not state["clamp"]) or abs(pos - DOWN) <= DOWN_TOL, pos


def release(b):
    ok, pos = release_ok(b)
    if not ok:
        raise GB.Abort(f"refusing to release with the clamp on at {pos} (not within {DOWN_TOL} of {DOWN})")
    GB.release(b, SID); state["torque"] = False
    time.sleep(2.0)
    return GB.read_pos(b, SID)["pos"]


def block(b, where, goal, probes):
    """Quiet hold + four smooth 2-deg moves (+ slow probes at LEVEL)."""
    out = {"where": where, "goal": goal}
    q = GB.sample(b, SID, seconds=QUIET_S)
    out["quiet"] = GB.window_stats(q)
    w = out["quiet"]
    say(f"  {where} quiet: pos {w['pos_mean'] - goal:+.2f}, range {w['range']}, moving "
        f"{w['moving']}/{w['n']}, load {w['load_min']}..{w['load_max']}, err {w['err']}")
    if probes:
        for name, side in (("above", +1), ("below", -1)):
            move(b, goal + side * A, SLOW); time.sleep(1.5)
            move(b, goal, SLOW)
            tr = GB.sample(b, SID, seconds=3.0)
            tail = [s for s in tr if s["t"] >= tr[-1]["t"] - 1.0]
            out[f"probe_{name}"] = {"pos": sum(s["pos"] for s in tail) / len(tail),
                                    "load": sum(s["load"] for s in tail) / len(tail),
                                    "step": GB.step_stats(tr, goal + side * A, goal)}
        pa, pb = out["probe_above"], out["probe_below"]
        say(f"  {where} probes: above {pa['pos'] - goal:+.1f} (load {pa['load']:.0f}), below "
            f"{pb['pos'] - goal:+.1f} (load {pb['load']:.0f}), midpoint "
            f"{(pa['pos'] + pb['pos']) / 2 - goal:+.1f}")
    steps = []
    for side in (+1, -1):
        for frm, to in ((goal, goal + side * A), (goal + side * A, goal)):
            move(b, to, STEP_SPD)                            # smooth: ACC-limited ramp
            tr = GB.sample(b, SID, seconds=TRACE_S)
            st = GB.step_stats(tr, frm, to)
            st["moving_tail"] = sum(1 for s in tr if s["t"] >= tr[-1]["t"] - 1.0 and s["spd"] != 0)
            steps.append({"from": frm, "to": to, "stats": st, "trace": tr})
            say(f"  {where} step {frm}->{to}: overshoot {st['overshoot']:.0f}, settle {st['settle_s']}, "
                f"residual range {st['residual_range']}, offset {st['offset']:+.1f}, "
                f"moving in last 1 s {st['moving_tail']}, err {st['err']}")
    out["steps"] = steps
    return out


def find_level(b):
    """v3 rig (2026-10-02): the fork's angle on the horn is new, so LEVEL is
    found, not assumed. Torque on where the arm rests; probe +-40 to learn
    which way gravity pulls (that side stops short with more load); then step
    AGAINST gravity 64 ticks at a time, holding, and read the load: the
    holding torque peaks at level. Stop two steps past the peak (<= ~11 deg
    above level). A parabola through the top three gives LEVEL."""
    p0 = GB.read_pos(b, SID)["pos"]
    torque_on(b)
    move(b, p0, SLOW); time.sleep(1.5)
    side = {}
    for s_ in (+1, -1):
        move(b, p0 + 40 * s_, SLOW); time.sleep(1.5)
        move(b, p0, SLOW); time.sleep(2.0)
        w = GB.window_stats(GB.sample(b, SID, n=10))
        side[s_] = (abs(w["pos_mean"] - p0), abs(w["load_mean"]))
    say(f"  gravity probe at {p0}: from +40 err/load {side[+1]}, from -40 err/load {side[-1]}")
    if side[+1] == side[-1]:
        raise GB.Abort("gravity direction not resolved by the probe (both sides equal)")
    g = +1 if side[+1] > side[-1] else -1      # approach from the gravity side stops short
    say(f"  gravity pulls toward {'+' if g > 0 else '-'} ticks")
    pts, pos = [], p0
    for k in range(16):
        tgt = pos - g * 64                       # one step against gravity
        move(b, tgt, SLOW); time.sleep(2.5)
        w = GB.window_stats(GB.sample(b, SID, n=10))
        pts.append((tgt, abs(w["load_mean"]), w["pos_mean"]))
        say(f"    step to {tgt}: pos {w['pos_mean']:.1f}, |load| {abs(w['load_mean']):.0f}")
        pos = tgt
        best = max(range(len(pts)), key=lambda i: pts[i][1])
        if len(pts) - 1 - best >= 2 and pts[-1][1] < 0.9 * pts[best][1]:
            break
    else:
        raise GB.Abort("no load peak within 16 steps (90 deg) -- level not found")
    best = max(range(len(pts)), key=lambda i: pts[i][1])
    if 0 < best < len(pts) - 1:
        (x0, y0, _), (x1, y1, _), (x2, y2, _) = pts[best - 1], pts[best], pts[best + 1]
        den = (y0 - 2 * y1 + y2)
        lvl = x1 + (0.5 * (y0 - y2) / den) * (x2 - x1) if den else x1
    else:
        lvl = pts[best][0]
    level = int(round(lvl))
    say(f"  LEVEL = {level} (peak |load| {pts[best][1]:.0f}); DOWN = {level + g * 1024}")
    return level, level + g * 1024, pts, g


def main():
    global LEVEL, DOWN
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} hold_clamp.py id {SID} LEVEL {LEVEL} DOWN {DOWN} "
        f"A {A} ladder {LADDER}")
    b = GB.Board(GB.find_port(None), log)
    res = {"id": SID, "level": LEVEL, "down": DOWN, "step_ticks": A, "rows": []}
    try:
        p0 = GB.read_pos(b, SID, tries=4)["pos"]
        if os.environ.get("CAL"):
            if GB.read_reg(b, SID, 40) != 0 or GB.read_gains(b, SID)[:2] != (32, 32):
                raise GB.Abort("calibration needs torque off and gains 32/32 (no release possible)")
            go(f"CALIBRATE: torque on at {p0} (weights on), probe +-40, step against gravity "
               f"64 ticks at a time until the holding load peaks (<= ~11 deg past level), "
               f"then lower to hanging and release there")
            LEVEL, DOWN, pts, g = find_level(b)
            res.update({"level": LEVEL, "down": DOWN, "cal": {"points": pts, "gravity": g}})
            for x in (LEVEL, DOWN):
                if not 200 <= x <= 3895:                 # +-A and travel overshoot stay off the wrap
                    raise GB.Abort(f"{x} is too close to the 0/4095 wrap -- move the fork one "
                                   f"horn-hole position; torque stays ON (lower it by hand cmd)")
            travel(b, DOWN)
            say(f"  released hanging: pos {release(b)}")
            p0 = GB.read_pos(b, SID)["pos"]
        start_down = abs(p0 - DOWN) <= DOWN_TOL
        if GB.read_reg(b, SID, 40) != 0:
            raise GB.Abort(f"start refused: torque is on at {p0}")
        if not (start_down or abs(p0 - LEVEL) <= START_TOL):
            if not LEVEL - START_TOL <= p0 <= DOWN + DOWN_TOL:
                raise GB.Abort(f"start refused: pos {p0} is outside level..hanging")
            go(f"torque on at {p0} and lower the arm to hanging ({DOWN}) at {SLOW} steps/s, "
               f"then release it there")
            torque_on(b)
            travel(b, DOWN)
            say(f"  released hanging: pos {release(b)}")
            start_down = True
        say(f"  start: pos {p0} ({'hanging' if start_down else 'level'})")
        if not start_down and GB.read_gains(b, SID)[:2] != (32, 32):   # a write needs a release
            raise GB.Abort("start refused: gains are not 32/32 (writing them would release)")
        if not start_down:
            go("torque on at P 32 and hold the arm straight out (clamp on or going on)")
            torque_on(b)
            travel(b, LEVEL)
            ask("\nCLAMP(S) ON near the 100 mm notch (rigid, nothing dangling); press enter when done")
            time.sleep(SETTLE)
        if start_down and not os.environ.get("CAL"):         # started hanging, maybe bare
            ask("\nArm hangs released. Put the CLAMP ON near the 100 mm notch (rigid, as before); "
                "press enter when it is on")
        for i, (p, d) in enumerate(LADDER):
            i = i if not start_down else max(i, 1)       # from hanging: every P starts hanging
            say(f"\n== P {p} D {d}")
            row = {"p": p, "d": d}
            if i > 0:                                        # at DOWN, released, clamp on
                row["gain_write"] = GB.write_gains(b, SID, p, d) if GB.read_gains(b, SID)[:2] != (p, d) else "ok (already)"
                say(f"  gains {p}/{d}: {row['gain_write']}")
                go(f"P {p}: torque on hanging down; quiet hold + 4 smooth 2-deg moves; raise to "
                   f"straight out; quiet hold + 2 slow probes + 4 smooth 2-deg moves; lower and release")
                torque_on(b)
                travel(b, DOWN)
                row["down"] = block(b, "DOWN", DOWN, probes=False)
                travel(b, LEVEL)
                row["level"] = block(b, "LEVEL", LEVEL, probes=True)
            else:                                            # already held at LEVEL, P 32
                go("P 32: quiet hold + 2 slow probes + 4 smooth 2-deg moves straight out; "
                   "lower to hanging; quiet hold + 4 smooth 2-deg moves; release there")
                row["level"] = block(b, "LEVEL", LEVEL, probes=True)
                travel(b, DOWN)
                row["down"] = block(b, "DOWN", DOWN, probes=False)
            if i > 0:
                travel(b, DOWN)
            res["rows"].append(row)
            say(f"  released hanging: pos {release(b)}")
        GB.write_gains(b, SID, 32, 32)
        say(f"  gains back to {GB.read_gains(b, SID)[:2]}")
        ask("\nThe arm hangs released. Take the CLAMP OFF; press enter when done")
        state["clamp"] = False
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        res["stopped"] = repr(e)
        try:
            if state["torque"]:
                ok, pos = release_ok(b)
                if ok:
                    say(f"  released at {release(b)}")
                else:
                    say(f"!! HOLDING TORQUE at {pos} with the clamp on. To finish: "
                        f"`move {SID} {DOWN} 0 {SLOW}`, wait, then `release {SID}`.")
                    input("press enter to leave it HELD and close the port > ")
        except Exception as e2:
            say(f"!! could not check/release: {e2!r} -- torque state unknown")
    finally:
        with open(os.path.join(OUT, "hold_clamp.json"), "w") as f:
            res["travels"] = TRAVELS
            json.dump(res, f, indent=1)
        for r in res["rows"]:
            for where in ("down", "level"):
                blk = r.get(where)
                if not blk:
                    continue
                q = blk["quiet"]
                worst = max((s["stats"]["residual_range"] for s in blk["steps"]), default=0)
                over = max((s["stats"]["overshoot"] for s in blk["steps"]), default=0)
                say(f"P {r['p']:3d} {where:5s}: quiet range {q['range']}, moving {q['moving']}/{q['n']}; "
                    f"steps max overshoot {over:.0f}, max residual range {worst}")
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
