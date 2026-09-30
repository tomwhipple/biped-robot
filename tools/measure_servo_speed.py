#!/usr/bin/env python3
"""Measure real STS3215 speed vs the sim's torque-speed envelope.

Sweeps a joint across a long travel at a ladder of commanded goal speeds
(reg 46; 0 = unlimited) while polling `pos` as fast as the CLI allows, and
fits the steady-state slope over the middle of the travel. Run with the
robot on the stand.

  sg dialout -c '.venv/bin/python tools/measure_servo_speed.py'

Context (2026-08-03): walker_env's sts3215 model assumes no-load
4.712 rad/s at 12 V (datasheet 0.222 s/60deg), linearly voltage-scaled;
training runs use supply_voltage=11.1. This measures where the real
servos sit at the actual pack voltage.
"""
import json
import os
import re
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bus_cal import BUS_NAMES, BUS_IDS

PORT = "/dev/ttyUSB0"
TICKS_PER_RAD = 4096.0 / (2.0 * 3.141592653589793)
POS_RE = re.compile(r"id\s+(\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)"
                    r"\s+([\d.]+)\s*V")


def _id_of(name: str) -> int:
    """Servo ID for the joint name on the 17-servo bus map (bus_map.h)."""
    return BUS_IDS[BUS_NAMES.index(name)]


# (label, servo id, sweep lo ticks, sweep hi ticks)  -- inside the envelope.
# IDs are looked up from the bus map so a rename/renumber lands here too.
KNEE = ("R_knee", _id_of("R_knee"), 1000, 3100)          # ~184 deg of free travel
HIP = ("L_hip_pitch", _id_of("L_hip_pitch"), 2044, 2900)  # +75 deg backward, lifts the leg
SPEEDS = [500, 1000, 2000, 3400, 0]       # reg-46 goal speeds; 0 = unlimited


class Cli:
    def __init__(self):
        self.s = serial.Serial(PORT, 115200, timeout=0.05)
        time.sleep(0.3)
        t0 = time.time()                   # port open reset the board; drain
        while time.time() - t0 < 4.0:
            self.s.read(256)
        self.cmd("")

    def cmd(self, line, wait=0.4):
        self.s.reset_input_buffer()
        self.s.write((line + "\r\n").encode())
        time.sleep(wait)
        return self.s.read(4096).decode(errors="replace")

    def pos_stream(self, sid, seconds):
        """Poll pos as fast as possible; return [(t, ticks, load, volt)]."""
        rows = []
        end = time.monotonic() + seconds
        buf = ""
        self.s.reset_input_buffer()
        while time.monotonic() < end:
            self.s.write(f"pos {sid}\r\n".encode())
            t = time.monotonic()
            buf += self.s.read(512).decode(errors="replace")
            m = None
            for m in POS_RE.finditer(buf):
                pass                        # keep the LAST complete reply
            if m and int(m.group(1)) == sid:
                rows.append((t, int(m.group(2)), int(m.group(4)),
                             float(m.group(5))))
                buf = buf[m.end():]
        return rows


def slope(rows, lo, hi):
    """steps/s over the middle 60% of the travel, |lo..hi|."""
    a = lo + 0.2 * (hi - lo)
    b = lo + 0.8 * (hi - lo)
    xs = [(t, p) for (t, p, _l, _v) in rows if min(a, b) <= p <= max(a, b)]
    if len(xs) < 4:
        return None, len(xs)
    t0, p0 = xs[0]
    t1, p1 = xs[-1]
    if t1 - t0 <= 0:
        return None, len(xs)
    return (p1 - p0) / (t1 - t0), len(xs)


def sweep(cli, label, sid, lo, hi, speed):
    out = []
    for src, dst in ((lo, hi), (hi, lo)):
        cli.cmd(f"move {sid} {src} 0 3400", wait=1.8)   # park at start, fast
        cli.cmd(f"move {sid} {dst} 0 {speed}", wait=0.0)
        travel_s = abs(dst - src) / max(speed if speed else 3400, 200) + 0.8
        rows = cli.pos_stream(sid, min(travel_s, 6.0))
        v, n = slope(rows, src, dst)
        volt = rows[-1][3] if rows else None
        out.append(dict(direction=f"{src}->{dst}", cmd_speed=speed,
                        measured_steps_s=v, samples=n, volt=volt))
        d = f"{v:8.0f}" if v else "     n/a"
        print(f"  {label} spd {speed:4d} {src}->{dst}: {d} steps/s "
              f"({n} samples, {volt} V)")
    return out


def main():
    cli = Cli()
    print(cli.cmd("volt", wait=0.8).strip())
    results = {}
    for (label, sid, lo, hi), speeds in ((KNEE, SPEEDS), (HIP, [3400, 0])):
        cli.cmd(f"torque {sid}")
        results[label] = []
        for sp in speeds:
            results[label] += sweep(cli, label, sid, lo, hi, sp)
        zero = 2050 if sid == 3 else 2044
        cli.cmd(f"move {sid} {zero} 0 600", wait=2.0)
        cli.cmd(f"release {sid}")
    Path("sweeps").mkdir(exist_ok=True)
    out = Path("sweeps/servo_speed.json")
    out.write_text(json.dumps(results, indent=1))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
