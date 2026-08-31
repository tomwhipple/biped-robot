#!/usr/bin/env python3
"""Servo frequency response in SIM: the calibration loop for servo_accel_max.

    .venv/bin/python tools/sim_frf.py                 # full measured sweep
    .venv/bin/python tools/sim_frf.py --accel 8.5     # try a candidate cap
    .venv/bin/python tools/sim_frf.py --amp 0.15 --freqs 0.5,1.0,1.5,2.0,3.0

Runs tools/servo_frf.py's sweep protocol against a BimoWalkerEnv instance
(actuator_model="sts3215" + the servo_accel_max profile model) and prints the
same table, side by side with the 2026-08-31 bench measurement. This is the
closed loop: tune --accel until the sim column reproduces the measured one.

Protocol mirror of servo_frf.py: one joint (L_hip_pitch -- the bench joint,
servo id 6), sinusoidal goal streamed at the 50 Hz control rate, measured
joint angle fitted with least squares at the KNOWN drive frequency after a
1 s settle cut. The target is injected at the actuator level by inverting the
action map for that joint (all other joints hold the stand default).

Bench conditions vs sim analog: on the bench the robot is clamped to the test
stand and the leg swings free. Here gravity is zeroed and the torso floats
free -- the joint FRF is a RELATIVE angle (leg vs torso), the torso is ~10x
the leg's reflected inertia, and the profile tracker upstream of the PD is
kinematic, so the clamp/float difference is far inside the fit noise.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim.walker_env import BimoWalkerEnv  # noqa: E402

_XML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "sim", "bimo_biped_v5body.xml")

# tools/servo_frf.py bench tables, 2026-08-31 (L_hip_pitch, 11.1 V, acc=0
# goal writes, unlimited slew): amp -> {freq Hz: measured amplitude ratio}
MEASURED = {
    0.15: {0.5: 0.98, 1.0: 0.94, 1.5: 0.77, 2.0: 0.42, 3.0: 0.20},
    0.075: {1.5: 0.92, 2.0: 0.86, 3.0: 0.35},
    0.30: {1.5: 0.37, 2.0: 0.21},
}


def make_env(accel, track_hz=3.5):
    env = BimoWalkerEnv(
        xml_path=_XML,
        actuator_model="sts3215",
        supply_voltage=11.1,          # 3S pack, matches the bench
        quantize_ticks=True,          # firmware writes integer goal ticks
        servo_accel_max=accel,
        servo_track_hz=track_hz,
        episode_seconds=1000.0,       # no truncation mid-sweep
    )
    # bench analog: torso clamped -> zero gravity, free float (see docstring)
    env.model.opt.gravity[:] = 0.0
    return env


def run_freq(env, j, f_hz, amp_rad, seconds, settle=1.0):
    env.reset(seed=0)
    n = int(round(seconds / env.control_dt))
    ts, ys = [], []
    for k in range(n):
        t = k * env.control_dt
        act = np.zeros(env._nq_act)
        # invert the legacy action map for joint j: target = default + scale*a
        act[j] = amp_rad * np.sin(2 * np.pi * f_hz * t) / env._scale[j]
        env.step(act)
        ts.append(t + env.control_dt)
        ys.append(float(env.data.qpos[env._jq0 + j]))
    ts, ys = np.array(ts), np.array(ys)
    keep = ts > settle
    ts, ys = ts[keep], ys[keep]
    # least squares: y = a sin(wt) + b cos(wt) + c against the DRIVE phase
    # (identical fit to tools/servo_frf.py)
    w = 2 * np.pi * f_hz
    A = np.column_stack([np.sin(w * ts), np.cos(w * ts), np.ones_like(ts)])
    coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
    a, b, c = coef
    return dict(n=len(ts), amp_ratio=float(np.hypot(a, b)) / amp_rad,
                phase_deg=float(np.degrees(np.arctan2(-b, a))), center=c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--accel", type=float, default=7.5,
                    help="servo_accel_max candidate (rad/s^2; 7.5 is the "
                         "2026-08-31 calibrated fit)")
    ap.add_argument("--track-hz", type=float, default=3.5,
                    help="servo_track_hz candidate (Hz; <= 0 disables)")
    ap.add_argument("--amp", type=float, default=None,
                    help="single amplitude (rad); default = full measured sweep")
    ap.add_argument("--freqs", default=None,
                    help="comma list of Hz; default = the measured points")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    if args.amp is not None:
        freqs = ([float(x) for x in args.freqs.split(",")] if args.freqs
                 else sorted(MEASURED.get(args.amp, MEASURED[0.15])))
        sweep = {args.amp: freqs}
    else:
        sweep = {a: sorted(fs) for a, fs in MEASURED.items()}

    env = make_env(args.accel, args.track_hz)
    j = env._legL["hip_pitch"]          # bench joint (servo id 6)
    worst = 0.0
    for amp, freqs in sweep.items():
        print(f"amp +-{amp:.3f} rad  (servo_accel_max={args.accel:g}, "
              f"servo_track_hz={args.track_hz:g})")
        for f in freqs:
            r = run_freq(env, j, f, amp, args.seconds)
            meas = MEASURED.get(amp, {}).get(f)
            line = (f"f={f:4.2f} Hz  n={r['n']:3d}  amp {r['amp_ratio']:.3f}  "
                    f"phase {r['phase_deg']:+6.1f} deg  center {r['center']:.3f}")
            if meas is not None:
                d = r["amp_ratio"] - meas
                worst = max(worst, abs(d))
                line += f"   | meas {meas:.2f}  d {d:+.3f}"
            print(line, flush=True)
    print(f"\ndrive: L_hip_pitch (bench servo id 6), 50 Hz goal stream, "
          f"least-squares fit past 1 s settle; worst |sim-meas| {worst:.3f}")


if __name__ == "__main__":
    main()
