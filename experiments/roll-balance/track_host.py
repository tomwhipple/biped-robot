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
ROBOT_AMP = (1.3, 1.30)          # k_static L, R (dwell.K_STATIC; R 1.30 since the 16:55 runs)
ROBOT_SEED = (5.2, 3.4)
# the dwell gait's timing per step time is dwell.SCHEDULE (from the SIL: dwell_grid.py); T below dwell.T_FLOOR is
# refused unless --below-floor
MAX_AHEAD_S = 8.0     # the robot's queue holds 128 frames (12.8 s at 10 Hz): stay <= 8 s ahead
PER_LINE = 4                  # frames per `track add` (42 hex + space each; the CLI line is <= 192 chars)
TS_RE = re.compile(r"^TS (\d+) ([LR]) in ([-\d.]+) out ([-\d.]+) e ([-+\d.]+) T ([-+\d.]+) hike ([-+\d.]+) (ok|ABORT)"
                   r"( Tback)?", re.M)


def lines_for(frames):
    """[(t_ms of the line's last frame, 'track add ...')], 4 frames a line, in order."""
    out = []
    for i in range(0, len(frames), PER_LINE):
        chunk = frames[i:i + PER_LINE]
        out.append((chunk[-1][0], "track add " + " ".join(h for _, h, _ in chunk)))
    return out


def plan(T_step, n_steps, direction, seeds=ROBOT_SEED, lead_ms=2500, cal=None, first="L", amp=ROBOT_AMP,
         model="dwell", ds=None, h=None, yaw_trim=0.0):
    gkw = {}
    if model == "dwell":
        gkw = dict(**({"ds": ds} if ds is not None else {}), **({"h": h} if h is not None else {}),
                   **({"yaw_trim": yaw_trim} if yaw_trim else {}))
    sw, fr = G.gait(T_step, n_steps, first=first, cal=cal, kL=amp[0], kR=amp[1], model=model, **G.DIRS[direction],
                    **gkw)
    pre = [f"trim T L {seeds[0]:g}", f"trim T R {seeds[1]:g}", "track clear"]
    return dict(sway=sw, frames=fr, pre=pre, adds=lines_for(fr), play=f"track play {int(lead_ms)}",
                duration_s=fr[-1][0] / 1e3, cal=cal, amp=amp, model=model)


def precheck(pl, tol=0.01):
    """The robot's static shift requirement as a hard constraint (hw 2026-11-12: it fell when the foot came up
    at 1.0-1.1 x): from the FRAMES that would be sent, the pelvis must be at or beyond k_static[stance] x 62 mm
    over the stance foot at every lift-off and at every touchdown. The pelvis shift is recovered from the
    frames' stance hip roll goal (ticks -> rad through the calibration in use, interpolated at that instant)
    by inverting the IK with the plan's feet. -> (ok, rows)."""
    sw, fr, cal = pl["sway"], pl["frames"], pl["cal"]
    K = G.W.S.kin(0.02)
    oy = abs(K.model_offset("L")[1])
    need = {"L": pl["amp"][0], "R": pl["amp"][1]}
    kf = []                      # (track ms, decoded {joint: rad})
    for tm, hx, kind in fr:
        if kind != "kf":
            continue
        q = {}
        for j, n in enumerate(G.LEG):
            q[n] = G.from_ticks(n, int(hx[6 + 3 * j: 9 + 3 * j], 16), cal)
        kf.append((tm, q))

    def q_at(t_s):
        tm = 1e3 * (t_s - sw.t_first)
        for (ta, qa), (tb, qb) in zip(kf, kf[1:]):
            if ta <= tm <= tb:
                u = (tm - ta) / max(tb - ta, 1e-9)
                return {n: qa[n] + u * (qb[n] - qa[n]) for n in qa}
        return None

    def seg_at(t_s):
        return min(sw.segs[1:], key=lambda sg: abs(sg["t"] - t_s))

    def shift_of(q, pa, stance):
        """the absolute pelvis shift (m) whose IK gives this stance hip roll, with the plan's feet."""
        jn = f"{stance}_hip_roll"
        feet = pa["feet"]

        def f(sh):
            return K.pose(shift=sh, fwd=pa["fwd"], footL=tuple(feet["L"]), footR=tuple(feet["R"]))[jn] - q[jn]
        # the bracket: the pelvis's reach either way (the IK raises past it)
        mid0 = 0.5 * (feet["L"][1] + feet["R"][1])
        lo, hi = mid0 - 0.12, mid0 + 0.12
        while True:
            try:
                flo = f(lo)
                f(hi)
                break
            except ValueError:
                lo, hi = mid0 + 0.9 * (lo - mid0), mid0 + 0.9 * (hi - mid0)
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            fm = f(mid)
            if (fm > 0) == (flo > 0):
                lo, flo = mid, fm
            else:
                hi = mid
        return 0.5 * (lo + hi)

    rows, ok = [], True
    for k, p in enumerate(sw.phases):
        st = p["stance"]
        sg = 1.0 if st == "L" else -1.0
        for what, t in (("lift-off", p["t0"]), ("touchdown", p["t1"])):
            q = q_at(t)
            if q is None:
                continue
            pa = seg_at(t)["pa"]
            sh = shift_of(q, pa, st)
            ratio = sg * (sh - pa["feet"][st][1]) / oy
            good = ratio >= need[st] - tol
            ok = ok and good
            rows.append(dict(step=k + 1, stance=st, at=what, t=round(t - sw.t_first, 2), ratio=round(ratio, 3),
                             need=need[st], ok=good))
    return ok, rows


def predicted_dwell(T_step, direction):
    p = os.path.join(HERE, "results", "dwell_schedule.json")
    if not os.path.exists(p):
        return None
    rows = [r for r in json.load(open(p)) if "error" not in r and r["T"] == T_step and r["dir"] == direction]
    if not rows:
        return None
    med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")  # noqa: E731
    li = [x for r in rows for x in r["lean_in"]]
    return dict(runs=len(rows), falls=sum(r["fell"] for r in rows), aborts=sum(r["aborts"] for r in rows),
                lifts=f"{sum(r['lifts'] for r in rows)}/{sum(r['n'] for r in rows)}", lean_in_med=med(li),
                lean_in_max=max(li), lean_out_max=max(x for r in rows for x in r["lean_out"]),
                T_mid={s: med([x for r in rows for x in r["T_mid"][s]]) for s in "LR"},
                hike_mid={s: med([x for r in rows for x in r["hike_mid"][s]]) for s in "LR"},
                clearance_mm=med([x for r in rows for x in r["clr"]]))


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


def report_precheck(pl):
    ok, rows = precheck(pl)
    bad = [r for r in rows if not r["ok"]]
    worst = min(rows, key=lambda r: r["ratio"] - r["need"]) if rows else None
    print(f"static-shift pre-check ({pl['model']}): {'PASS' if ok else 'REFUSED'} -- pelvis over the stance foot at "
          f"every lift-off and touchdown, from the frames, needs L >= {pl['amp'][0]:g} x, R >= {pl['amp'][1]:g} x 62 mm"
          + (f"; tightest: step {worst['step']} {worst['stance']} {worst['at']} {worst['ratio']:.3f} x" if worst else ""))
    for r in bad[:8]:
        print(f"   step {r['step']} {r['stance']} stance at {r['at']} (t {r['t']} s): {r['ratio']:.3f} x < {r['need']:g} x")
    return ok


def dry(pl, a):
    adds = pl["adds"]
    nb = sum(len(l) + 1 for _, l in adds)
    ahead_ms = min(a.ahead * a.T, MAX_AHEAD_S) * 1e3
    first = [l for t, l in adds if t <= ahead_ms]
    print(f"gait: {pl['model']} T {a.T:g} s/step, {a.steps} steps, {a.dir}; {sum(1 for f in pl['frames'] if f[2] == 'kf')} "
          f"keyframes at {G.KF_S * 1e3:.0f} ms + {sum(1 for f in pl['frames'] if f[2] == 'rest')} rest frames; "
          f"track {pl['duration_s']:.1f} s after a {a.lead} ms lead-in")
    print(f"upload: {len(adds)} `track add` lines ({PER_LINE} frames each), {nb} bytes; longest line "
          f"{max(len(l) for _, l in adds)} chars (CLI limit 192)")
    print(f"before play: {len(first)} lines (the first {ahead_ms / 1e3:.1f} s); at {a.ms_per_line:.0f} ms a line "
          f"(assumed CLI round trip, to be measured) that is {len(first) * a.ms_per_line / 1e3:.1f} s; while "
          f"playing the host needs {1e3 / (G.KF_S * 1e3) / PER_LINE:.1f} lines/s to keep pace, "
          f"{(1e3 / (G.KF_S * 1e3) / PER_LINE) * a.ms_per_line / 10:.0f} % of the link at that rate")
    print(f"queue: at most ~{int(ahead_ms / (G.KF_S * 1e3)) + 2 * PER_LINE} of 128 frames held "
          f"({ahead_ms / 1e3:.1f} s ahead)")
    pr = predicted_dwell(a.T, a.dir) if pl["model"] == "dwell" else predicted(a.T, a.dir)
    if pr:
        print(f"SIL prediction ({pl['model']}, {pr['runs']} runs over plants x delay): falls {pr['falls']}, aborts "
              f"{pr['aborts']}, lifts {pr['lifts']}; TS lean toward swing median {pr['lean_in_med']:.1f} max "
              f"{pr['lean_in_max']:.1f} deg, outward max {pr['lean_out_max']:.1f}; T at mid support L "
              f"{pr['T_mid']['L']:.1f} R {pr['T_mid']['R']:.1f}, hike L {pr['hike_mid']['L']:.1f} R "
              f"{pr['hike_mid']['R']:.1f} deg; swing sole clearance median {pr['clearance_mm']:.0f} mm")
    report_precheck(pl)
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

    if not report_precheck(pl):
        print("REFUSED: the plan puts the pelvis under the static requirement -- nothing sent")
        return
    r = robot_io.Robot(host=a.host, log=log)
    steps, sent, t_line = [], 0, []
    adds = pl["adds"]
    ahead_ms = min(a.ahead * a.T, MAX_AHEAD_S) * 1e3

    def scan(txt):
        """Every new TS line in txt: record, print, and say whether one of them trips the stop rule. (2026-11-12:
        it returned at the first tripping line, so the lines after it were never printed.)"""
        trip = False
        for m in TS_RE.finditer(txt):
            s = dict(n=int(m.group(1)), stance=m.group(2), lean_in=float(m.group(3)), lean_out=float(m.group(4)),
                     e=float(m.group(5)), T=float(m.group(6)), hike=float(m.group(7)), ok=m.group(8) == "ok",
                     tback="Tback" in m.group(0))
            if all(x["n"] != s["n"] for x in steps):
                steps.append(s)
                print(f"  step {s['n']} {s['stance']}: in {s['lean_in']:.1f} out {s['lean_out']:.1f} e {s['e']:+.1f} "
                      f"T {s['T']:+.1f} hike {s['hike']:+.1f} {'ok' if s['ok'] else 'ABORT'}"
                      f"{' Tback' if s['tback'] else ''}", flush=True)
                if not s["ok"] or s["lean_in"] > a.stop_lean:
                    trip = True
        return trip

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
            # the firmware's TS lines are async output: over Wi-Fi they never reach a command's reply (2026-11-12:
            # 0 of 34 seen live, so the stop rule never ran). `track steps` repeats the last 16: poll it.
            m2 = re.search(r"(\d+) single supports reported", txt)
            if m2 and int(m2.group(1)) > len(steps):
                stop = scan(r.cmd("track steps")) or stop
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
    ap.add_argument("--T", type=float, default=6.0,
                    help="s per step: the dwell schedule has 6, 5, 4, 3.4, 3.2 (the SIL floor), 3; start at 6")
    ap.add_argument("--below-floor", action="store_true", help="allow T under dwell.T_FLOOR (the SIL says no)")
    ap.add_argument("--dir", default="march", choices=list(G.DIRS))
    ap.add_argument("--steps", type=int, default=6)
    ap.add_argument("--first", default="L", choices=["L", "R"], help="the first swing foot")
    ap.add_argument("--seed", default=f"{ROBOT_SEED[0]:g},{ROBOT_SEED[1]:g}", help="trim T L,R (hike deg)")
    ap.add_argument("--amp", default=f"{ROBOT_AMP[0]:g},{ROBOT_AMP[1]:g}",
                    help="k_static L,R: the pelvis over the stance foot from before lift-off to touchdown (x 62 mm)")
    ap.add_argument("--model", default="dwell", choices=["dwell", "sway"],
                    help="dwell (default): the static requirement as a hard constraint; sway: A2's sine (refused "
                         "by the pre-check)")
    ap.add_argument("--ds", type=float, default=None, help="dwell: double-support fraction of T (default per T)")
    ap.add_argument("--h", type=float, default=None, help="dwell: swing sole height, m (default 0.022)")
    ap.add_argument("--yaw-trim", type=float, default=0.0,
                    help="dwell: heading change per step, deg, + = to the robot's left (the stance hip turns the body "
                         "about the planted foot, the swing foot lands square). In the sim 1 deg asked turns ~1.4-1.5 "
                         "deg; calibrate on the robot. To cancel a leftward drift of d deg/step use -d")
    ap.add_argument("--lead", type=int, default=2500, help="ms from where it stands to the first frame")
    ap.add_argument("--ahead", type=float, default=2.0, help="steps the host stays ahead of the play head")
    ap.add_argument("--stop-lean", type=float, default=10.0, help="deg toward the swing foot -> track stop")
    ap.add_argument("--ms-per-line", type=float, default=29.0, help="dry run: CLI round trip (measured 29 ms median, 2026-11-12)")
    ap.add_argument("--poll", type=float, default=0.25)
    ap.add_argument("--live", action="store_true", help="SEND to the robot (Stage C, a person at it)")
    ap.add_argument("--host", default="192.168.2.90")
    ap.add_argument("--log")
    a = ap.parse_args(argv)
    seeds = tuple(float(x) for x in a.seed.split(","))
    import dwell as DW
    if a.model == "dwell" and a.T < DW.T_FLOOR and not a.below_floor:
        print(f"REFUSED: T {a.T:g} s is under the dwell gait's SIL floor ({DW.T_FLOOR:g} s): it falls or leans 11-14 deg "
              f"there (experiments/roll-balance/README.md); --below-floor to plan it anyway")
        return 2
    amp = tuple(float(x) for x in a.amp.split(","))
    if not a.live:
        dry(plan(a.T, a.steps, a.dir, seeds, a.lead, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h, yaw_trim=a.yaw_trim), a)
        return 0
    sys.path.insert(0, os.path.join(ROOT, "experiments", "stepper"))
    import robot_io
    probe = robot_io.Robot(host=a.host)
    cal = {n: (z, d) for n, (_, z, d) in probe.cal.items()}
    probe.close()
    live(plan(a.T, a.steps, a.dir, seeds, a.lead, cal=cal, first=a.first, amp=amp, model=a.model, ds=a.ds, h=a.h, yaw_trim=a.yaw_trim), a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
