#!/usr/bin/env python3
"""Powered range-of-motion sweep of ONE joint on a robot hanging free on the
stand, from wherever it rests, on a body whose calibration is not trusted.

Why not tools/bench_rom_sweep.py: that one sweeps around the board's stored
calibration. On a newly assembled body that calibration belongs to another
robot, and the firmware clamps every `move` on a bus-map id (1-17) into the
joint's range under it and SENDS the clamped value (cli.cpp
clampToJointRange) -- a 2-degree step becomes a jump to the range's edge.
2026-10-23, the v6 legs: 6 of 10 joints sat outside their stale range.

So, per direction:
  1. RAM-only `cal set <id> <zero> 1` (never `cal save`; a reboot restores
     NVS) placing the firmware's clamp window from the start position out in
     the sweep direction, as wide as the joint's mech_envelope.h range.
     Off-map ids (e.g. the 51/52 hip rolls) have no clamp and skip this.
  2. Torque on IN PLACE: `move <id> <present>` (a bare `torque` would drive
     to whatever stale goal the register holds).
  3. Steps of --step-deg at --speed steps/s. Stop on: the joint no longer
     following (moved < 40 % of a step while > 60 % short of it: a stop or a
     contact), |load| > --load-stop, a status bit, the window / range cap,
     or 30 ticks from the encoder's 0/4095 wrap.
  4. Back to the start, release.
Then the other direction, then the joint's original `cal` row is restored in
RAM (`cal set` + `cal fit 0` where it was not fitted). Every other servo in
--check-ids must be torque-off, or it refuses to start. Any error: back to
the start if it can, then release that servo.

  .venv/bin/python tools/rom_sweep.py --id 9 --check-ids 2,3,4,6,7,8,9,10,11,12,51,52 \
      --out hw_sessions/<date>/legs/rom_9.json
"""
import argparse
import json
import re
import sys
import time

import serial

K = 4096 / 360.0
# firmware/main/mech_envelope.h kMechLo / kMechHi, degrees, by JOINT NAME. The
# servo ID of each joint comes from the live board's `cal show` (board_joint),
# so the tool follows whichever bus map the flashed firmware carries (the v6
# legs' IDs changed on 2026-10-23: keying this by ID once went stale).
MECH_BY_NAME = {
    "R_hip_roll": (-55, 25), "R_hip_pitch": (-110, 90), "R_knee": (-95, 95),
    "R_ankle": (-40, 40), "L_hip_roll": (-25, 55), "L_hip_pitch": (-110, 90),
    "L_knee": (-95, 95), "L_ankle": (-40, 40), "R_hip_yaw": (-45, 45),
    "L_hip_yaw": (-45, 45), "R_ankle_roll": (-25, 25), "L_ankle_roll": (-25, 25)}
POS_RE = re.compile(r"id\s+(\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)"
                    r"\s+([\d.]+)\s*V\s+(\d+)\s*C\s+err\s+0x([0-9a-fA-F]+)")
CAL_RE = re.compile(r"(\w+)\s+id\s+(\d+)\s+zero\s+(\d+)\s+dir\s+([+-]1)\s+(fitted|NOT FITTED)")
ENC_LO, ENC_HI, EDGE = 30, 4065, 30
ACTIVE = {"start": None}       # the running direction's start, for recovery


class Abort(RuntimeError):
    pass


def utc():
    t = time.time()
    return time.strftime("%H:%M:%S", time.gmtime(t)) + f".{int(t * 1000) % 1000:03d}Z"


class Port:
    def __init__(self, dev, log):
        self.s = serial.Serial(dev, 115200, timeout=0.02)
        self.log = log
        time.sleep(0.3)
        self.s.read(65536)

    def cmd(self, line, until, timeout=1.0):
        self.s.write((line + "\n").encode())
        buf, t0 = "", time.monotonic()
        while time.monotonic() - t0 < timeout:
            buf += self.s.read(4096).decode(errors="replace")
            if re.search(until, buf):
                break
        self.log.write(f">>> {line}\n{buf}")
        self.log.flush()
        return buf

    def pos(self, sid):
        for _ in range(3):
            m = POS_RE.search(self.cmd(f"pos {sid}", r"id\s+\d+\s+pos.*\n|timeout"))
            if m:
                return {"pos": int(m.group(2)), "spd": int(m.group(3)), "load": int(m.group(4)),
                        "temp": int(m.group(6)), "err": int(m.group(7), 16)}
        raise Abort(f"id {sid}: no position reply")

    def torque_reg(self, sid):
        m = re.search(r"reg 40 = (\d+)", self.cmd(f"reg {sid} 40 1", r"reg 40 = \d+|timeout"))
        if not m:
            raise Abort(f"id {sid}: no reply to reg 40")
        return int(m.group(1))

    def move(self, sid, tgt, spd):
        out = self.cmd(f"move {sid} {int(tgt)} 0 {int(spd)} 2", r"move id .*: \S+\r?\n|clamped|\? \(try|busy")
        if "clamped" in out or ": ok" not in out:
            raise Abort(f"move {sid} {tgt}: {out.strip()[-120:]!r}")

    def release(self, sid):
        self.cmd(f"release {sid}", r"release \S+: \S+\r?\n")


def board_joint(p, sid):
    """The board's own `cal show` row for servo `sid`: (name, zero, dir,
    fitted, (lo, hi) deg), or None when the flashed firmware's bus map does not
    have that ID (then `move` is not clamped). A mapped joint without a range
    here is refused: its clamp could not be re-centred."""
    for m in CAL_RE.finditer(p.cmd("cal show", r"loaded from|defaults|\? \(try", 2.0)):
        if int(m.group(2)) == sid:
            name = m.group(1)
            if name not in MECH_BY_NAME:
                raise Abort(f"id {sid} is the board's {name}: no range for it in MECH_BY_NAME")
            return (name, int(m.group(3)), int(m.group(4)), m.group(5) == "fitted",
                    MECH_BY_NAME[name])
    return None


def settle(p, sid, tmax=2.0):
    t0, still, r = time.monotonic(), 0, None
    while time.monotonic() - t0 < tmax:
        r = p.pos(sid)
        still = still + 1 if r["spd"] == 0 else 0
        if still >= 2:
            break
    return r


def sweep(p, sid, sgn, a, say, budget=None, mech=None):
    """One direction from the resting position; returns the record. budget:
    degrees left of the whole sweep's travel (both directions together)."""
    p0 = p.pos(sid)["pos"]
    step = round(a.step_deg * K)
    window = (ENC_LO, ENC_HI)
    if mech:
        lo, hi = mech
        width = round((hi - lo) * K)
        # dir +1: window = [zero + lo*K, zero + hi*K]; start EDGE inside one end
        zero = p0 - EDGE - round(lo * K) if sgn > 0 else p0 + EDGE - round(hi * K)
        zero = max(0, min(4095, zero))
        w = (max(0, zero + round(lo * K)), min(4095, zero + round(hi * K)))
        if not w[0] + 10 <= p0 <= w[1] - 10:
            raise Abort(f"id {sid}: no cal window can hold the start {p0} (window {w})")
        out = p.cmd(f"cal set {sid} {zero} 1", r"zero <- .*\n|usage|\? \(try")
        if "zero <-" not in out:
            raise Abort(f"cal set {sid}: {out.strip()[-90:]!r}")
        window = (max(ENC_LO, w[0] + 5), min(ENC_HI, w[1] - 5))
        cap_deg = hi - lo
    else:
        cap_deg = a.max_deg
    cap = p0 + sgn * round(min(cap_deg, a.max_deg, budget if budget is not None else 1e9) * K)
    rec = {"dir": sgn, "start": p0, "window": window, "steps": [], "stop": None}
    ACTIVE["start"] = p0
    # `move` refuses with torque off (cli.cpp torqueGate), so: torque, check it
    # held where it was, then a goal at the present position
    p.cmd(f"torque {sid}", r"torque \S+: \S+\r?\n")
    r = p.pos(sid)
    if abs(r["pos"] - p0) > 20:
        p.release(sid)
        raise Abort(f"id {sid}: moved {r['pos'] - p0} ticks at torque-on (released)")
    p.move(sid, p0, a.speed)
    r = settle(p, sid)
    rec["torque"] = True
    base = r["load"]
    prev, n = r["pos"], 0
    say(f"  {utc()} id {sid} dir {sgn:+d}: start {p0}, window {window}, load {base}")
    while True:
        n += 1
        tgt = p0 + sgn * step * n
        lim = min(window[1], cap) if sgn > 0 else max(window[0], cap)
        if (tgt - lim) * sgn > 0:
            rec["stop"] = "range cap" if (cap - lim) * sgn <= 0 else (
                "encoder wrap" if lim in (ENC_LO, ENC_HI) else "cal window")
            break
        p.move(sid, tgt, a.speed)
        time.sleep(step / a.speed)
        r = settle(p, sid)
        r["tgt"] = tgt
        rec["steps"].append(r)
        moved = (r["pos"] - prev) * sgn
        if r["err"]:
            rec["stop"] = f"status 0x{r['err']:02x}"
            break
        if abs(r["load"]) > a.load_stop:
            rec["stop"] = f"load {r['load']}"
            break
        if moved < 0.4 * step and abs(r["pos"] - tgt) > 0.6 * step:
            rec["stop"] = f"stopped following (moved {moved}, {abs(r['pos'] - tgt)} short)"
            break
        prev = r["pos"]
    far = rec["steps"][-1]["pos"] if rec["steps"] else p0
    rec["reach"] = far
    rec["reach_deg"] = round((far - p0) / K, 1)
    rec["max_load"] = max((abs(s["load"]) for s in rec["steps"]), default=abs(base))
    say(f"  {utc()} id {sid} dir {sgn:+d}: reached {far} ({rec['reach_deg']:+.1f} deg), "
        f"stop: {rec['stop']}, max |load| {rec['max_load']}")
    p.move(sid, p0, a.speed)
    time.sleep(abs(far - p0) / a.speed + 0.5)
    r = settle(p, sid, 3.0)
    p.release(sid)
    rec["torque"] = False
    # a released joint can fall under gravity (2026-10-23: a hip pitch dropped
    # 45 deg once its leg's released knee had flopped); wait until it is still
    rest, t_end = p.pos(sid)["pos"], time.monotonic() + 8.0
    while time.monotonic() < t_end:
        time.sleep(0.5)
        now = p.pos(sid)["pos"]
        if abs(now - rest) <= 2:
            break
        rest = now
    rec["back"], rec["after_release"] = r["pos"], rest
    if abs(rest - r["pos"]) > 20:
        say(f"  !! id {sid} fell {rest - r['pos']:+d} ticks after release")
    say(f"  {utc()} id {sid}: back at {r['pos']}, released, rests at {rec['after_release']}")
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--check-ids", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dirs", default="+1,-1")
    ap.add_argument("--step-deg", type=float, default=2.0)
    ap.add_argument("--speed", type=int, default=60, help="steps/s")
    ap.add_argument("--max-deg", type=float, default=120.0)
    ap.add_argument("--load-stop", type=int, default=450, help="/1000 of the servo's max")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    a = ap.parse_args()
    sid = a.id
    log = open(a.out.rsplit(".", 1)[0] + ".log", "a")
    say = lambda s: (print(s, flush=True), log.write(s + "\n"), log.flush())
    p = Port(a.port, log)
    res = {"id": sid, "args": vars(a), "dirs": []}
    orig = None
    try:
        for i in [int(x) for x in a.check_ids.split(",")]:
            if p.torque_reg(i) != 0:
                raise Abort(f"id {i}: torque is ON -- refusing to start")
        row = board_joint(p, sid)
        mech = row[4] if row else None
        if row:
            orig = row[1:4]
            res["cal_orig"], res["joint"] = orig, row[0]
        say(f"# {utc()} rom_sweep id {sid} dirs {a.dirs} step {a.step_deg} deg @ {a.speed} steps/s, "
            f"load stop {a.load_stop}")
        # total travel over both directions <= the joint's design width + 20 deg
        # (2026-10-23: each direction alone allowed the full width, and the
        # stop-less hip yaw ran 86 + 85 deg, twice its +-45 design range)
        budget = (mech[1] - mech[0] + 20) if mech else 2 * a.max_deg
        for sgn in [int(x) for x in a.dirs.split(",")]:
            if budget <= a.step_deg:
                say(f"  id {sid} dir {sgn:+d}: skipped, travel budget used")
                continue
            rec = sweep(p, sid, sgn, a, say, budget, mech)
            res["dirs"].append(rec)
            budget -= abs(rec["reach_deg"])
    except (Abort, KeyboardInterrupt, Exception) as e:
        say(f"!! STOPPED: {e!r}")
        res["stopped"] = repr(e)
        try:
            start = p.pos(sid)
            # back to where this direction started, if it was torqued
            if p.torque_reg(sid):
                home = ACTIVE["start"] if ACTIVE["start"] is not None else start["pos"]
                say(f"  returning {start['pos']} -> {home}, then release")
                p.move(sid, home, a.speed)
                time.sleep(abs(start["pos"] - home) / a.speed + 0.5)
            p.release(sid)
        except Exception as e2:
            say(f"!! recovery failed ({e2!r}): broadcast release")
            p.cmd("release", r"release \S+: \S+\r?\n")
    finally:
        if orig:
            try:
                p.cmd(f"cal set {sid} {orig[0]} {orig[1]}", r"zero <- .*\n|usage")
                if not orig[2]:
                    p.cmd(f"cal fit {sid} 0", r"NOT FITTED|usage")
                say(f"  cal row restored in RAM: zero {orig[0]} dir {orig[1]:+d} "
                    f"{'fitted' if orig[2] else 'NOT FITTED'} (nothing saved)")
            except Exception as e3:
                say(f"!! cal restore failed: {e3!r} (a reboot restores NVS)")
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)
        p.s.close()
        say(f"# {utc()} end")


if __name__ == "__main__":
    main()
