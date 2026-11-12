"""Host side of the on-board gait track (#297 B2): the sway gait as `track add` lines, uploaded ROLLING
while the robot plays them, with the per-step telemetry (the firmware's TS lines) and a stop rule.

DRY RUN BY DEFAULT: prints the session (the CLI lines, in order), the upload sizing and the SIL prediction
for this gait (results/sil_gait.json). `--live` sends it to the robot over the Wi-Fi CLI
(experiments/stepper/robot_io.py) -- only on the Stage C plan, a person at the robot.

The session:
    trim T L <seed>, trim T R <seed>     the learned trims (default: the robot's own, L 5.2, R 3.4 deg)
    track clear
    track add ... (the first `--ahead` steps)
    track play <lead_ms>                  min-jerk to the first frame (the crouch), then play
    ... track add as the play head moves (the host stays `--ahead` steps in front; `track` reads the head)
    the timeline ends at the last step end: the queue runs dry there, the firmware rests (= track stop)
Stop rule: a TS line with ABORT, or a lean toward the swing foot past --stop-lean deg -> `track stop`
(finish the step, rest pose); a refused line or a lost link -> `track halt` (freeze, the trim fades).

    python track_host.py --T 4 --dir march --steps 6              # dry run: what would be sent
    python track_host.py --T 4 --dir march --steps 6 --live       # on the robot (Stage C only)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import sil_gait as G  # noqa: E402

# The robot's own working values on the hard panels, 2026-11-12 (B1 runs fwtrim_*): static shift 1.3x over
# the left foot, 1.25x over the right (1.15x tipped in at foot-off), learned T L 5.2, R 3.4 deg. The SIL holds
# with them at T = 4 and 2 s in every direction (results/sil_gait_robotamp.json).
ROBOT_AMP = (1.3, 1.25)
ROBOT_SEED = (5.2, 3.4)
PER_LINE = 4                  # frames per `track add` (42 hex + space each; the CLI line is <= 192 chars)
TS_RE = re.compile(r"^TS (\d+) ([LR]) in ([-\d.]+) out ([-\d.]+) e ([-+\d.]+) T ([-+\d.]+) hike ([-+\d.]+) (ok|ABORT)",
                   re.M)


def lines_for(frames):
    """[(t_ms of the line's last frame, 'track add ...')], 4 frames a line, in order."""
    out = []
    for i in range(0, len(frames), PER_LINE):
        chunk = frames[i:i + PER_LINE]
        out.append((chunk[-1][0], "track add " + " ".join(h for _, h, _ in chunk)))
    return out


def plan(T_step, n_steps, direction, seeds=ROBOT_SEED, lead_ms=2500, cal=None, first="L", amp=ROBOT_AMP):
    sw, fr = G.gait(T_step, n_steps, first=first, cal=cal, kL=amp[0], kR=amp[1], **G.DIRS[direction])
    pre = [f"trim T L {seeds[0]:g}", f"trim T R {seeds[1]:g}", "track clear"]
    return dict(sway=sw, frames=fr, pre=pre, adds=lines_for(fr), play=f"track play {int(lead_ms)}",
                duration_s=fr[-1][0] / 1e3)


def predicted(T_step, direction):
    p = os.path.join(HERE, "results", "sil_gait.json")
    if not os.path.exists(p):
        return None
    rows = [r for r in json.load(open(p)) if "error" not in r and r["T"] == T_step and r["dir"] == direction]
    if not rows:
        return None
    reps = [x for r in rows for x in r["reports"]]
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")  # noqa: E731
    return dict(runs=len(rows), falls=sum(r["fell"] for r in rows), aborts=sum(r["aborts"] for r in rows),
                lifts=f"{sum(r['lifts'] for r in rows)}/{sum(r['n_steps'] for r in rows)}",
                lean_in_med=med([x["lean_in"] for x in reps]), lean_in_max=max(x["lean_in"] for x in reps),
                lean_out_max=max(x["lean_out"] for x in reps),
                T_mid={s: med([x["T_mid"] for x in reps if x["stance"] == s]) for s in "LR"},
                hike_mid={s: med([x["hike_mid"] for x in reps if x["stance"] == s]) for s in "LR"},
                clearance_mm=med([p["clr_max_mm"] for r in rows for p in r["phases"]]))


def dry(pl, a):
    adds = pl["adds"]
    nb = sum(len(l) + 1 for _, l in adds)
    ahead_ms = a.ahead * a.T * 1e3
    first = [l for t, l in adds if t <= ahead_ms]
    print(f"gait: sway T {a.T:g} s/step, {a.steps} steps, {a.dir}; {sum(1 for f in pl['frames'] if f[2] == 'kf')} "
          f"keyframes at {G.KF_S * 1e3:.0f} ms + {sum(1 for f in pl['frames'] if f[2] == 'rest')} rest frames; "
          f"track {pl['duration_s']:.1f} s after a {a.lead} ms lead-in")
    print(f"upload: {len(adds)} `track add` lines ({PER_LINE} frames each), {nb} bytes; longest line "
          f"{max(len(l) for _, l in adds)} chars (CLI limit 192)")
    print(f"before play: {len(first)} lines (the first {a.ahead:g} steps); at {a.ms_per_line:.0f} ms a line "
          f"(assumed CLI round trip, to be measured) that is {len(first) * a.ms_per_line / 1e3:.1f} s; while "
          f"playing the host needs {1e3 / (G.KF_S * 1e3) / PER_LINE:.1f} lines/s to keep pace, "
          f"{(1e3 / (G.KF_S * 1e3) / PER_LINE) * a.ms_per_line / 10:.0f} % of the link at that rate")
    print(f"queue: at most ~{int(ahead_ms / (G.KF_S * 1e3)) + 2 * PER_LINE} of 256 frames held "
          f"({a.ahead:g} steps ahead)")
    pr = predicted(a.T, a.dir)
    if pr:
        print(f"SIL prediction (sil_gait.py, {pr['runs']} runs over plants x delay): falls {pr['falls']}, aborts "
              f"{pr['aborts']}, lifts {pr['lifts']}; TS lean toward swing median {pr['lean_in_med']:.1f} max "
              f"{pr['lean_in_max']:.1f} deg, outward max {pr['lean_out_max']:.1f}; T at mid support L "
              f"{pr['T_mid']['L']:.1f} R {pr['T_mid']['R']:.1f}, hike L {pr['hike_mid']['L']:.1f} R "
              f"{pr['hike_mid']['R']:.1f} deg; swing sole clearance median {pr['clearance_mm']:.0f} mm")
    print("\nsession (dry run -- nothing sent):")
    for l in pl["pre"]:
        print("  " + l)
    for _, l in adds[:len(first)][:3]:
        print("  " + l)
    if len(first) > 3:
        print(f"  ... {len(first) - 3} more `track add` lines")
    print("  " + pl["play"])
    print(f"  ... {len(adds) - len(first)} `track add` lines while it plays, then the firmware rests by itself")


def live(pl, a):
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    log_path = a.log or os.path.join(HERE, "results", time.strftime("track_%Y%m%dT%H%M%S.jsonl"))
    logf = open(log_path, "a")

    def log(d):
        logf.write(json.dumps(d) + "\n")
        logf.flush()

    r = robot_io.Robot(host=a.host, log=log)
    steps, sent, t_line = [], 0, []
    adds = pl["adds"]
    ahead_ms = a.ahead * a.T * 1e3

    def scan(txt):
        for m in TS_RE.finditer(txt):
            s = dict(n=int(m.group(1)), stance=m.group(2), lean_in=float(m.group(3)), lean_out=float(m.group(4)),
                     e=float(m.group(5)), T=float(m.group(6)), hike=float(m.group(7)), ok=m.group(8) == "ok")
            if all(x["n"] != s["n"] for x in steps):
                steps.append(s)
                print(f"  step {s['n']} {s['stance']}: in {s['lean_in']:.1f} out {s['lean_out']:.1f} e {s['e']:+.1f} "
                      f"T {s['T']:+.1f} hike {s['hike']:+.1f} {'ok' if s['ok'] else 'ABORT'}")
                if not s["ok"] or s["lean_in"] > a.stop_lean:
                    return True
        return False

    def add(line):
        t = time.monotonic()
        txt = r.cmd(line)
        t_line.append(time.monotonic() - t)
        if not txt.lstrip().startswith("track +"):
            raise RuntimeError(f"refused: {txt.strip()[-160:]}")
        return txt

    try:
        for l in pl["pre"]:
            r.cmd(l)
        while sent < len(adds) and adds[sent][0] <= ahead_ms:
            add(adds[sent][1])
            sent += 1
        r.cmd(pl["play"])
        stop = False
        while True:
            txt = r.cmd("track")
            m = re.search(r"play t (\d+) ms", txt)
            head = int(m.group(1)) if m else 0
            stop = scan(txt) or stop
            if stop:
                r.cmd("track stop")
                print("  stop rule hit -> track stop")
                break
            while sent < len(adds) and adds[sent][0] <= head + ahead_ms:
                stop = scan(add(adds[sent][1])) or stop
                sent += 1
            if re.search(r"^track (done|idle)", txt, re.M) or (sent >= len(adds) and "resting" in txt):
                break
            time.sleep(a.poll)
    except Exception as e:  # noqa: BLE001 -- any failure: freeze, the trim fades
        print(f"  {e!r} -> track halt")
        try:
            r.cmd("track halt")
        except Exception:  # noqa: BLE001
            pass
    finally:
        try:
            r.c.idle(3.0)
            scan(r.cmd("track steps"))
            print(r.cmd("track").strip())
            print(r.cmd("trim").strip())
        except Exception:  # noqa: BLE001
            pass
        if t_line:
            ts = sorted(t_line)
            print(f"upload: {len(t_line)} lines, round trip median {1e3 * ts[len(ts) // 2]:.0f} ms, max "
                  f"{1e3 * ts[-1]:.0f} ms -> {PER_LINE / ts[len(ts) // 2]:.0f} frames/s")
        log(dict(summary=True, steps=steps, line_s=t_line))
        r.close()
        print(f"log: {log_path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--T", type=float, default=4.0, help="s per step (Stage C starts at 4 and steps down)")
    ap.add_argument("--dir", default="march", choices=list(G.DIRS))
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--first", default="L", choices=["L", "R"], help="the first swing foot")
    ap.add_argument("--seed", default=f"{ROBOT_SEED[0]:g},{ROBOT_SEED[1]:g}", help="trim T L,R (hike deg)")
    ap.add_argument("--amp", default=f"{ROBOT_AMP[0]:g},{ROBOT_AMP[1]:g}", help="sway amplitude over L,R (x 62 mm)")
    ap.add_argument("--lead", type=int, default=2500, help="ms from where it stands to the first frame")
    ap.add_argument("--ahead", type=float, default=2.0, help="steps the host stays ahead of the play head")
    ap.add_argument("--stop-lean", type=float, default=10.0, help="deg toward the swing foot -> track stop")
    ap.add_argument("--ms-per-line", type=float, default=100.0, help="dry run: assumed CLI round trip")
    ap.add_argument("--poll", type=float, default=0.25)
    ap.add_argument("--live", action="store_true", help="SEND to the robot (Stage C, a person at it)")
    ap.add_argument("--host", default="192.168.2.90")
    ap.add_argument("--log")
    a = ap.parse_args(argv)
    seeds = tuple(float(x) for x in a.seed.split(","))
    amp = tuple(float(x) for x in a.amp.split(","))
    if not a.live:
        dry(plan(a.T, a.steps, a.dir, seeds, a.lead, first=a.first, amp=amp), a)
        return 0
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    probe = robot_io.Robot(host=a.host)
    cal = {n: (z, d) for n, (_, z, d) in probe.cal.items()}
    probe.close()
    live(plan(a.T, a.steps, a.dir, seeds, a.lead, cal=cal, first=a.first, amp=amp), a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
