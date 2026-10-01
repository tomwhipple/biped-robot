"""Four-bottle stiffness + probes (issue #73), 2026-10-01, on Tom's "go"
before gain_bench grew the same mode. Servo-positioned, Tom's rules: gains
are written with the BARE arm released LEVEL (friction holds it); nothing is
released while a bottle may be hung; an abort while loaded HOLDS torque.
Per P (32/128/160): gains -> go -> torque on (jump check) -> GOAL ->
for bare, 1..4 bottles, bag off: static sample, then a probe from above
(GOAL+A -> GOAL) and from below (GOAL-A -> GOAL), each return traced.
Reuses gain_bench's write_gains/read_pos/sample/window_stats/step_stats.
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "experiments", "plan-b-bench"))
import gain_bench as GB

SID, GOAL, SPD, A = 30, 2057, 100, 23
LADDER = [(32, 32), (128, 128), (160, 160)]
BOTTLE_KG, LEVER_M, N, SETTLE = 0.39, 0.100, 20, 5.0
logf = open(os.path.join(HERE, "session.log"), "a")


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


def move(b, to):
    out = b.cmd(f"move {SID} {to} 0 {SPD}", until=GB.MOVE_UNTIL)
    if ": ok" not in out or "clamp" in out:
        raise GB.Abort(f"move {to}: {out.strip()[-90:]!r}")


def probe(b, side):
    move(b, GOAL + side * A)
    time.sleep(1.5)
    move(b, GOAL)
    tr = GB.sample(b, SID, seconds=3.0)
    st = GB.step_stats(tr, GOAL + side * A, GOAL)
    tail = [s for s in tr if s["t"] >= tr[-1]["t"] - 1.0]
    return {"trace": tr, "step": st,
            "pos": sum(s["pos"] for s in tail) / len(tail),
            "load": sum(s["load"] for s in tail) / len(tail)}


def main():
    say(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')} four_bottle.py id {SID} GOAL {GOAL} A {A} ladder {LADDER}")
    b = GB.Board(GB.find_port(None), log)
    res = {"id": SID, "goal": GOAL, "probe_ticks": A, "spd": SPD, "bottle_kg_nominal": BOTTLE_KG,
           "lever_m": LEVER_M, "rows": []}
    torque_on = loaded = False
    try:
        p0 = GB.read_pos(b, SID, tries=4)["pos"]
        if GB.read_reg(b, SID, 40) != 0 or abs(p0 - GOAL) > 20:
            raise GB.Abort(f"start refused: pos {p0}, need the bare arm released within 20 of {GOAL}")
        for p, d in LADDER:
            say(f"\n== P {p} D {d}")
            say(f"  gains {p}/{d}: {GB.write_gains(b, SID, p, d)}")   # bare, level, released
            go(f"P {p}: torque on, hold straight out ({GOAL}), and run the +-2 deg probes "
               f"(4 short moves) at EVERY load state of this P")
            before = GB.read_pos(b, SID)["pos"]
            b.cmd(f"torque {SID}", until=r"torque \S+: \S+\r?\n"); torque_on = True
            if abs(GB.read_pos(b, SID)["pos"] - before) > GB.JUMP_TICKS:
                raise GB.Abort("jump at torque-on")
            move(b, GOAL); time.sleep(SETTLE)
            row = {"p": p, "d": d, "states": []}
            plan = [(0, None)] + [(k, f"HANG bottle #{k} (total {k})") for k in (1, 2, 3, 4)] \
                + [(0, "LIFT ALL bottles OFF")]
            for nb, prompt in plan:
                if prompt:
                    if nb > 0:
                        loaded = True
                    ask(f"\n{prompt}; press enter when done")
                    if nb == 0:
                        loaded = False
                    time.sleep(SETTLE)
                w = GB.window_stats(GB.sample(b, SID, n=N))
                up, dn = probe(b, +1), probe(b, -1)
                st = {"bottles": nb, "tau_nominal": nb * BOTTLE_KG * GB.G * LEVER_M, "static": w,
                      "above": up, "below": dn}
                row["states"].append(st)
                say(f"  {nb} bottle(s): static {w['pos_mean'] - GOAL:+.2f} (load {w['load_mean']:.0f}, "
                    f"range {w['range']}) | from above {up['pos'] - GOAL:+.2f} (load {up['load']:.0f}, "
                    f"over {up['step']['overshoot']:.0f}, settle {up['step']['settle_s']}) | from below "
                    f"{dn['pos'] - GOAL:+.2f} (load {dn['load']:.0f}, over {dn['step']['overshoot']:.0f}, "
                    f"settle {dn['step']['settle_s']}) | err {w['err'] | up['step']['err'] | dn['step']['err']}")
            res["rows"].append(row)
            GB.release(b, SID); torque_on = False          # bare (bag off confirmed), level
            time.sleep(2.0)
            say(f"  released level, bare: pos {GB.read_pos(b, SID)['pos']}")
        GB.write_gains(b, SID, 32, 32)
        say(f"  gains back to {GB.read_gains(b, SID)[:2]}")
    except (GB.Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        if torque_on and loaded:
            say("!! HOLDING TORQUE with bottles possibly hung. Lift them off first.")
            input("press enter once ALL bottles are off (the arm stays held) > ")
            loaded = False
        if torque_on and not loaded:
            GB.release(b, SID)
            say(f"  released bare at pos {GB.read_pos(b, SID)['pos']}")
        res["stopped"] = repr(e)
    finally:
        with open(os.path.join(HERE, "four_bottle.json"), "w") as f:
            json.dump(res, f, indent=1)
        ref = None
        for r in res["rows"]:
            pts = []
            for s in r["states"]:
                pts += [(s["static"]["pos_mean"], s["static"]["load_mean"]),
                        (s["above"]["pos"], s["above"]["load"]), (s["below"]["pos"], s["below"]["load"])]
            n = len(pts); mx = sum(x for x, _ in pts) / n; my = sum(y for _, y in pts) / n
            sxx = sum((x - mx) ** 2 for x, _ in pts)
            slope = sum((x - mx) * (y - my) for x, y in pts) / sxx if sxx else float("nan")
            ref = ref or slope
            say(f"P {r['p']:3d}: load/tick {slope:6.2f} (x P32 {slope / ref:4.2f}) over {n} points")
        say(f"# end {time.strftime('%Y-%m-%dT%H:%M:%S')}")
        b.close()


if __name__ == "__main__":
    main()
