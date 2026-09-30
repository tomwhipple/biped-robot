#!/usr/bin/env python3
"""Visual ROM confirmation sweep — one joint at a time, on the test stand.

Drives each joint alone through a fraction of its MODEL range in small
waypoints, capturing a webcam frame + pos/load reading at every waypoint,
then returns it to zero and releases torque before the next joint.

Safety posture (user 2026-08-02: model-range sweeps only, no end-stop
probing; release-all + alert on anything suspicious):
  - single-servo torque, all others released the whole time
  - waypoints stepped at gentle speed; per-waypoint following-error and
    load checks; ANY anomaly -> broadcast release, exit 2
  - per-joint direction caps for moves with collision potential
    (inward hip roll can cross the legs; see table)
  - the firmware's calibrated range clamp is the backstop, not the plan:
    targets here stay inside SWEEP_FRAC of the model range

Run:  .venv/bin/python tools/bench_rom_sweep.py [--joints R_knee,L_knee]
      [--frac 0.9] [--outdir sweeps/<ts>] [--port /dev/ttyUSB0]
Needs dialout (wrap with `sg dialout -c ...` in pre-group-refresh shells).
Frames + a per-waypoint log land in --outdir; review them, then update
docs/servo-map.md travel notes.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bus_cal import (
    BusCal, BusCalError, TICKS_PER_DEG,
    asbuilt_prototype_cal, fetch_cal,
)

# (joint, servo id, zero ticks, dir, model range deg (lo, hi), sweep cap deg)
# zero/dir = as-built calibration (docs/servo-map.md). Range = the plant
# model's joint range IN THE SIM SIGN CONVENTION current at flash time --
# JOINTS are name + per-servo sweep bounds. Zero/direction come from the
# board's `cal show` (cal blob v3, 17-servo bus map); the asbuilt table is
# the fallback when `cal show` cannot be parsed and the robot is the
# prototype. The bounds below are the bench clamp *for this script*
# (docs/servo-map.md §3.3): they agree with firmware/main/mech_envelope.h
# on the leg rows.
JOINTS = [
    # name          lo      hi      cap_lo  cap_hi
    ("L_hip_yaw",   -45.0,  45.0,  -20.0,   None),  # - = L toe-in
    ("L_hip_roll",  -25.0,  25.0,   -6.0,   None),  # - = L inward;
    # measured 2026-08-02: leg-on-leg contact begins ~-9 deg (load onset),
    # sim can't see it (no inter-leg collision geoms) -- keep 6 deg margin
    ("L_hip_pitch", -110.0, 60.0,   None,   30.0),  # + = backward
    ("L_knee",      -95.0,   5.0,   None,   None),  # full flex proven
    ("L_ankle",     -40.0,  40.0,   None,   None),
    ("R_hip_yaw",   -45.0,  45.0,   None,   20.0),  # + = R toe-in
    ("R_hip_roll",  -25.0,  25.0,   None,    6.0),  # + = R inward
    ("R_hip_pitch", -110.0, 60.0,   None,   30.0),  # + = backward
    ("R_knee",      -95.0,   5.0,   None,   None),  # full flex proven
    ("R_ankle",     -40.0,  40.0,   None,   None),
]

POS_RE = re.compile(
    r"id\s+(\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)"
    r"\s+([\d.]+)\s*V\s+(\d+)\s*C\s+err\s+0x([0-9a-fA-F]+)")

LOAD_ABORT = 300      # /1000 of stall; sweep is quasi-static, expect <60
FERR_ABORT = 40       # ticks of following error after settle
SETTLE_S = 1.2


class Bench:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.3)
        time.sleep(0.3)
        self.drain(4.0)            # opening the port reset the board
        self.cmd("")               # first command after reset can be eaten

    def drain(self, secs):
        end = time.time() + secs
        buf = b""
        while time.time() < end:
            buf += self.ser.read(256)
        return buf.decode(errors="replace")

    def cmd(self, line, wait=0.6):
        self.ser.reset_input_buffer()
        self.ser.write((line + "\r\n").encode())
        time.sleep(wait)
        return self.drain(0.4)

    def pos(self, sid):
        out = self.cmd(f"pos {sid}", wait=0.5)
        m = POS_RE.search(out)
        if not m or int(m.group(1)) != sid:
            raise RuntimeError(f"unparseable pos reply for id {sid}: {out!r}")
        g = m.groups()
        return dict(pos=int(g[1]), spd=int(g[2]), load=int(g[3]),
                    volt=float(g[4]), temp=int(g[5]), err=int(g[6], 16))

    def release_all(self):
        self.cmd("release")


def snap(outdir, tag):
    f = outdir / f"{tag}.jpg"
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "v4l2", "-i",
         "/dev/video0", "-frames:v", "1", "-update", "1", str(f)],
        check=True, timeout=15)
    return f.name


def sweep_joint(b, outdir, log, joint, lo, hi, cap_lo, cap_hi,
                frac, n_steps):
    """`joint` is a bus_cal.BusJoint (the live calibration row). The other
    arguments are the script-local envelope bounds from the JOINTS table."""
    name, sid, zero, dirn = joint.name, joint.servo_id, joint.zero_steps, joint.direction
    lo, hi = lo * frac, hi * frac
    if cap_lo is not None: lo = max(lo, cap_lo)
    if cap_hi is not None: hi = min(hi, cap_hi)
    # waypoints: zero -> hi in steps, back through zero -> lo, back to zero
    ups = [hi * (i + 1) / n_steps for i in range(n_steps)]
    downs = [lo * (i + 1) / n_steps for i in range(n_steps)]
    plan = ups + ups[-2::-1] + [0.0] + downs + downs[-2::-1] + [0.0]

    print(f"== {name} (id {sid}): sweep {lo:+.1f}..{hi:+.1f} deg")
    st = b.pos(sid)
    log.append(dict(joint=name, phase="pre", **st))
    # torque-off joints rest wherever gravity leaves them (ankles droop,
    # a released hip roll leans on the other leg) -- pull gently to zero
    # first; only a wildly-off start is an abort
    if abs(st["pos"] - zero) > 300:
        raise RuntimeError(f"{name} too far from zero before sweep: {st}")
    b.cmd(f"torque {sid}")
    b.cmd(f"move {sid} {zero}", wait=0.2)
    time.sleep(SETTLE_S)
    try:
        for k, ang in enumerate(plan):
            tgt = int(round(zero + dirn * ang * TICKS_PER_DEG))
            b.cmd(f"move {sid} {tgt}", wait=0.2)
            time.sleep(SETTLE_S)
            st = b.pos(sid)
            frame = snap(outdir, f"{name}_{k:02d}_{ang:+05.1f}deg")
            rec = dict(joint=name, phase="sweep", step=k, angle_deg=ang,
                       target=tgt, frame=frame, **st)
            log.append(rec)
            ferr = abs(st["pos"] - tgt)
            print(f"   {ang:+6.1f} deg  tgt {tgt}  pos {st['pos']}"
                  f"  ferr {ferr}  load {st['load']}  {st['temp']}C"
                  f"  err 0x{st['err']:02x}")
            if st["err"] != 0:
                raise RuntimeError(f"{name} fault flag 0x{st['err']:02x}")
            if abs(st["load"]) > LOAD_ABORT:
                raise RuntimeError(f"{name} load {st['load']} > {LOAD_ABORT}")
            if ferr > FERR_ABORT:
                # clamped-by-firmware targets also show up here: surface it
                raise RuntimeError(
                    f"{name} following error {ferr} ticks at {ang:+.1f} deg "
                    f"(firmware clamp? obstruction?)")
        b.cmd(f"move {sid} {zero}", wait=0.2)
        time.sleep(SETTLE_S)
        st = b.pos(sid)
        log.append(dict(joint=name, phase="post", **st))
        if abs(st["pos"] - zero) > 10:
            raise RuntimeError(f"{name} did not return to zero: {st}")
    finally:
        b.cmd(f"release {sid}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--joints", default="",
                    help="comma-separated subset, default all rows in the JOINTS table")
    ap.add_argument("--frac", type=float, default=0.9)
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--outdir", default="")
    args = ap.parse_args()

    sel = [j.strip() for j in args.joints.split(",") if j.strip()]
    rows = [r for r in JOINTS if not sel or r[0] in sel]
    if sel and len(rows) != len(sel):
        sys.exit(f"unknown joint in --joints (have {[r[0] for r in JOINTS]})")

    outdir = Path(args.outdir or
                  f"sweeps/rom_{time.strftime('%Y%m%d_%H%M%S')}")
    outdir.mkdir(parents=True, exist_ok=True)
    log = []
    b = Bench(args.port)
    # Calibration comes from the board (`cal show`, cal blob v3, 17-servo bus
    # map). The asbuilt prototype table is the fallback. Joints the script
    # sweeps MUST be fitted on this robot, or refuse. `b.cmd` accepts no
    # `until` regex; we pass a longer wait so the 17-row reply has drained.
    cal, cal_source = fetch_cal(lambda c, _until: b.cmd(c, wait=1.6))
    print(f"cal source: {cal_source}")
    for r in rows:
        bj = cal.by_name(r[0])
        if not bj.fitted:
            sys.exit(f"!! {r[0]} (id {bj.servo_id}) is NOT FITTED on this robot")
    try:
        snap(outdir, "00_all_zero")
        for r in rows:
            joint = cal.by_name(r[0])
            sweep_joint(b, outdir, log, joint, r[1], r[2], r[3], r[4],
                        frac=args.frac, n_steps=args.steps)
    except Exception as e:
        print(f"!! ABORT: {e}\n!! releasing all servos", file=sys.stderr)
        try:
            b.release_all()
        finally:
            (outdir / "sweep_log.json").write_text(json.dumps(log, indent=1))
        sys.exit(2)
    b.release_all()
    (outdir / "sweep_log.json").write_text(json.dumps(log, indent=1))
    print(f"done: {len(log)} records, frames in {outdir}/")


if __name__ == "__main__":
    main()
