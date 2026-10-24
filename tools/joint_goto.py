#!/usr/bin/env python3
"""Move ONE joint slowly to a target tick on a body whose calibration is not
trusted, then release it -- tools/rom_sweep.py's safeguards, for a chosen
target instead of a sweep (e.g. putting the legs back straight).

  .venv/bin/python tools/joint_goto.py --id 8 --to 2650 \
      --check-ids 2,3,4,6,7,8,9,10,11,12,51,52 --log hw_sessions/<date>/legs/goto.log

Same rules as rom_sweep: every --check-ids servo torque-off at the start; a
RAM-only `cal set` (never saved) so the firmware's clamp window holds both the
start and the target; torque, check it held, then steps of --step-deg at
--speed; stop on not-following, load or a status bit; release at the end (or
keep torque with --hold) and wait until the joint is still; restore the cal
row. Prints where the joint came to rest after the release.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rom_sweep import ENC_HI, ENC_LO, K, Abort, Port, board_joint, settle, utc  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--to", type=int, required=True)
    ap.add_argument("--check-ids", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--step-deg", type=float, default=2.0)
    ap.add_argument("--speed", type=int, default=60)
    ap.add_argument("--load-stop", type=int, default=450)
    ap.add_argument("--hold", action="store_true", help="leave torque on at the target")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()
    sid, tgt = a.id, a.to
    if not ENC_LO <= tgt <= ENC_HI:
        sys.exit(f"target {tgt} is outside {ENC_LO}..{ENC_HI} (encoder wrap)")
    log = open(a.log, "a")
    say = lambda s: (print(s, flush=True), log.write(s + "\n"), log.flush())
    p = Port(a.port, log)
    orig, start = None, None
    try:
        for i in [int(x) for x in a.check_ids.split(",")]:
            if p.torque_reg(i) != 0:
                raise Abort(f"id {i}: torque is ON -- refusing to start")
        start = p.pos(sid)["pos"]
        row = board_joint(p, sid)               # by the flashed firmware's map
        if row:
            orig = row[1:4]
            lo, hi = row[4]
            mid = (start + tgt) // 2                   # centre the window on the move
            zero = max(0, min(4095, mid - round((lo + hi) / 2 * K)))
            w = (zero + round(lo * K), zero + round(hi * K))
            if not (w[0] + 10 <= min(start, tgt) and max(start, tgt) <= w[1] - 10):
                raise Abort(f"move {start} -> {tgt} is wider than id {sid}'s range window {w}")
            if "zero <-" not in p.cmd(f"cal set {sid} {zero} 1", r"zero <- .*\n|usage"):
                raise Abort(f"cal set {sid} failed")
        say(f"# {utc()} joint_goto id {sid}: {start} -> {tgt} ({(tgt - start) / K:+.1f} deg)")
        p.cmd(f"torque {sid}", r"torque \S+: \S+\r?\n")
        if abs(p.pos(sid)["pos"] - start) > 20:
            p.release(sid)
            raise Abort(f"id {sid} moved at torque-on (released)")
        p.move(sid, start, a.speed)
        prev, step = start, round(a.step_deg * K)
        sgn = 1 if tgt > start else -1
        n = 0
        while True:
            n += 1
            nxt = start + sgn * step * n
            if (nxt - tgt) * sgn > 0:
                nxt = tgt
            p.move(sid, nxt, a.speed)
            time.sleep(step / a.speed)
            r = settle(p, sid)
            moved = (r["pos"] - prev) * sgn
            if r["err"] or abs(r["load"]) > a.load_stop or (
                    moved < 0.4 * min(step, abs(nxt - prev) or step) and abs(r["pos"] - nxt) > 0.6 * step):
                raise Abort(f"stopped at {r['pos']} (target {nxt}): load {r['load']}, err {r['err']}, moved {moved}")
            prev = r["pos"]
            if nxt == tgt:
                break
        r = settle(p, sid, 3.0)
        say(f"  {utc()} id {sid} at {r['pos']} (target {tgt}), load {r['load']}")
        if not a.hold:
            p.release(sid)
            rest, t_end = p.pos(sid)["pos"], time.monotonic() + 8.0
            while time.monotonic() < t_end:
                time.sleep(0.5)
                now = p.pos(sid)["pos"]
                if abs(now - rest) <= 2:
                    break
                rest = now
            say(f"  {utc()} id {sid} released, rests at {rest}"
                + (f"  !! moved {rest - r['pos']:+d} after release" if abs(rest - r["pos"]) > 20 else ""))
        else:
            say(f"  {utc()} id {sid} HELD at {r['pos']} (torque on)")
    except (Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        try:
            if p.torque_reg(sid):
                if a.hold:
                    say(f"  holding torque at {p.pos(sid)['pos']} (--hold)")
                else:
                    p.release(sid)
                    say(f"  released at {p.pos(sid)['pos']}")
        except Exception as e2:
            say(f"!! recovery failed ({e2!r}): broadcast release")
            p.cmd("release", r"release \S+: \S+\r?\n")
    finally:
        if orig:
            p.cmd(f"cal set {sid} {orig[0]} {orig[1]}", r"zero <- .*\n|usage")
            if not orig[2]:
                p.cmd(f"cal fit {sid} 0", r"NOT FITTED|usage")
            say(f"  cal row restored in RAM: zero {orig[0]} dir {orig[1]:+d} (nothing saved)")
        p.s.close()


if __name__ == "__main__":
    main()
