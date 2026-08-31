#!/usr/bin/env python3
"""Servo frequency response on the bench: the dynamic-tracking measurement.

    .venv/bin/python tools/servo_frf.py --port /dev/ttyUSB0

WHY: the 2026-08-31 walk capture measured real joint velocities at ~0.5x sim
for the same policy, and constant-speed bench sweeps matched commanded speed
exactly -- so the loss is DYNAMIC. This drives one joint with a small
sinusoidal goal (`move`, unlimited slew) at several frequencies in the gait
band and fits the measured amplitude and phase per frequency. The resulting
pole is the number env_mjx's --act-lag DR should straddle.

Commands stream at ~20 Hz; position readbacks (`pos`) interleave at ~10 Hz
and are timestamped at read time. Amplitude/phase come from a least-squares
sinusoid fit at the KNOWN drive frequency, so irregular sampling is fine.

Safety: torque is explicitly enabled (firmware refuses goal writes
otherwise), the joint moves +-0.15 rad about its calibrated zero at most,
and the script ends on the rest target with torque LEFT ON, then prints a
summary. Ctrl-C mid-run still sends the rest target.
"""
import argparse
import re
import time

import numpy as np
import serial

ZERO = 2044                 # L_hip_pitch calibrated zero (cal show)
SERVO_ID = 6
TICKS_PER_RAD = 4096 / (2 * np.pi)
POS_RE = re.compile(rb"id 6\s+pos\s+(\d+)")


def drain(ser, secs=0.3):
    end = time.monotonic() + secs
    buf = b""
    while time.monotonic() < end:
        buf += ser.read(4096)
    return buf


def run_freq(ser, f_hz, amp_rad, seconds, settle=1.0, acc=0):
    amp = amp_rad * TICKS_PER_RAD
    t0 = time.monotonic()
    samples = []          # (t, ticks)
    last_cmd = 0.0
    last_pos_req = 0.0
    pending = b""
    while True:
        t = time.monotonic() - t0
        if t >= seconds:
            break
        if t - last_cmd >= 0.05:
            tgt = int(round(ZERO + amp * np.sin(2 * np.pi * f_hz * t)))
            ser.write(b"move %d %d 0 0 %d\r\n" % (SERVO_ID, tgt, acc))
            last_cmd = t
        if t - last_pos_req >= 0.09:
            ser.write(b"pos %d\r\n" % SERVO_ID)
            last_pos_req = t
        pending += ser.read(4096)
        for m in POS_RE.finditer(pending):
            samples.append((time.monotonic() - t0, int(m.group(1))))
        if samples:
            pending = pending[pending.rfind(b"\n") + 1:]
        time.sleep(0.005)
    ts = np.array([s for s, _ in samples])
    ys = np.array([p for _, p in samples], dtype=float)
    keep = ts > settle
    ts, ys = ts[keep], ys[keep]
    if len(ts) < 10:
        return None
    # least squares: y = a sin(wt) + b cos(wt) + c against the DRIVE phase
    w = 2 * np.pi * f_hz
    A = np.column_stack([np.sin(w * ts), np.cos(w * ts), np.ones_like(ts)])
    coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
    a, b, c = coef
    meas_amp = float(np.hypot(a, b))
    phase = float(np.degrees(np.arctan2(-b, a)))   # lag vs commanded sine
    return dict(n=len(ts), amp_ratio=meas_amp / (amp_rad * TICKS_PER_RAD),
                phase_deg=phase, center=c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--amp", type=float, default=0.15, help="rad")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--freqs", default="0.5,1.0,1.5,2.0,3.0")
    ap.add_argument("--acc", type=int, default=0,
                    help="ACC register value per goal write (100 steps/s^2 "
                         "per LSB; 0 = what the control loop sends)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0)
    time.sleep(2.5)                      # DTR-reset boot
    drain(ser, 1.0)
    ser.write(b"torque\r\n")
    time.sleep(0.5)
    reply = drain(ser, 0.5)
    assert b"torque all: ok" in reply, reply[-200:]
    ser.write(b"move %d %d 0 300\r\n" % (SERVO_ID, ZERO))   # gentle to zero
    time.sleep(1.5)
    drain(ser)

    results = {}
    try:
        for f in (float(x) for x in args.freqs.split(",")):
            r = run_freq(ser, f, args.amp, args.seconds, acc=args.acc)
            results[f] = r
            print(f"f={f:4.2f} Hz  " + (
                f"n={r['n']:3d}  amp {r['amp_ratio']:.3f}  "
                f"phase {r['phase_deg']:+6.1f} deg  center {r['center']:.0f}"
                if r else "insufficient samples"), flush=True)
    finally:
        ser.write(b"move %d %d 0 300\r\n" % (SERVO_ID, ZERO))
        ser.flush()
        time.sleep(1.0)
        ser.close()
    print("\ndrive: +-%.3f rad on L_hip_pitch (id %d), unlimited slew"
          % (args.amp, SERVO_ID))


if __name__ == "__main__":
    main()
