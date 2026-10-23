#!/usr/bin/env python3
"""Watch servo positions while someone moves the joints BY HAND (torque off).

Read-only: sends only `pos <id>` (and `reg <id> 40` once, to refuse if any
servo holds torque). Use it to check which servo sits in which joint, and to
read each joint's hand-measured range of motion, on a body whose calibration
is not trusted yet -- no goal write, so the firmware's calibrated-range clamp
(cli.cpp clampToJointRange) can never turn a check into a jump.

  .venv/bin/python tools/joint_monitor.py --ids 2,3,4,6,7,8,9,10,11,12,51,52 \
      --out hw_sessions/<date>/joint_monitor.json [--seconds 600]

Prints a line whenever a servo moves more than --thresh ticks from where it
was last reported (with the UTC time, for the videos), and on exit (Ctrl-C,
SIGTERM or --seconds) the per-ID start / min / max / span in ticks and degrees.
"""
import argparse
import json
import re
import signal
import sys
import time

import serial

POS_RE = re.compile(r"id\s+(\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)")
DEG = 360.0 / 4096


def utc():
    t = time.time()
    return time.strftime("%H:%M:%S", time.gmtime(t)) + f".{int(t * 1000) % 1000:03d}Z"


class Port:
    def __init__(self, dev):
        self.s = serial.Serial(dev, 115200, timeout=0.02)
        time.sleep(0.3)
        self.s.read(65536)

    def cmd(self, line, until, timeout=0.5):
        self.s.write((line + "\n").encode())
        buf, t0 = "", time.monotonic()
        while time.monotonic() - t0 < timeout:
            buf += self.s.read(4096).decode(errors="replace")
            if re.search(until, buf):
                break
        return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--thresh", type=int, default=30, help="ticks (30 = 2.6 deg)")
    ap.add_argument("--seconds", type=float, default=0.0)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]
    p = Port(a.port)
    for i in ids:
        r = p.cmd(f"reg {i} 40 1", r"reg 40 = \d+|timeout|\? \(try")
        m = re.search(r"reg 40 = (\d+)", r)
        if not m:
            sys.exit(f"id {i}: no reply to reg 40 ({r.strip()[-60:]!r})")
        if m.group(1) != "0":
            sys.exit(f"id {i}: torque is ON -- refusing (this tool is for hand moves, torque off)")
    stats, last, log = {}, {}, []
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    t_end = time.monotonic() + a.seconds if a.seconds else float("inf")
    print(f"# {utc()} watching {ids}, torque off on all; move one joint at a time", flush=True)
    while not stop["now"] and time.monotonic() < t_end:
        for i in ids:
            m = POS_RE.search(p.cmd(f"pos {i}", r"id\s+\d+\s+pos.*\n|timeout"))
            if not m:
                continue
            pos = int(m.group(2))
            log.append([round(time.time(), 3), i, pos, int(m.group(4))])
            st = stats.setdefault(i, {"start": pos, "min": pos, "max": pos})
            st["min"], st["max"] = min(st["min"], pos), max(st["max"], pos)
            if i not in last:
                last[i] = pos
            elif abs(pos - last[i]) >= a.thresh:
                print(f"{utc()} id {i:2d} moved {last[i]} -> {pos} ({(pos - last[i]) * DEG:+.1f} deg); "
                      f"seen {st['min']}..{st['max']} ({(st['max'] - st['min']) * DEG:.0f} deg)", flush=True)
                last[i] = pos
    p.s.close()
    print(f"# {utc()} summary (ticks; span in deg):")
    for i in ids:
        st = stats.get(i)
        if st:
            print(f"  id {i:2d}: start {st['start']:4d}  min {st['min']:4d}  max {st['max']:4d}  "
                  f"span {(st['max'] - st['min']) * DEG:5.1f}")
    with open(a.out, "w") as f:
        json.dump({"ids": ids, "stats": stats, "samples": log}, f)
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
