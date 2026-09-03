#!/usr/bin/env python3
"""Arm the robot into a zero-command stand, log the sway, bail out early.

Tom (2026-09-02): "when I arm the robot it begins a growing oscillation that
eventually results in it falling over."  This is the instrumented repeat of
that GUI arm: ARM|POSE frames so every beacon carries the ten measured joint
angles, a CSV of every beacon, and a guard that e-stops + disarms BEFORE the
fall -- on torso tilt, or on any joint's peak-to-peak swing over the last
second.  Built on arm_script.Driver (same latch preamble, same seq rule).

    .venv/bin/python link/osc_probe.py --host 192.168.2.90 \
        --duration 8 --tilt 0.90 --pp-rad 0.5 --csv hw_sessions/x.csv
"""
import argparse
import csv
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_script import Driver, arm_off  # noqa: E402
from protocol import (FLAG_ARM, FLAG_ATT, FLAG_HOME, FLAG_POSE, ArmResult,  # noqa: E402
                      LinkState, diag_arm_result, diag_reason)

JOINTS = ["L_yaw", "L_roll", "L_pitch", "L_knee", "L_ankle",
          "R_yaw", "R_roll", "R_pitch", "R_knee", "R_ankle"]


class OscDriver(Driver):
    def __init__(self, host, csv_path, tilt, pp_rad, window_s=1.0, **kw):
        super().__init__(host, **kw)
        self.f = open(csv_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "epoch", "state", "vbat", "up_z", "vx_est",
                         "wz_est", "err", "late", "len", "up_x", "up_y"] + JOINTS)
        self.tilt, self.pp_rad = tilt, pp_rad
        self.hist = deque()          # (t, joints) over the last window
        self.window = window_s
        self.tilt_streak = 0
        self.max_pp = 0.0
        self.min_upz = 1.0
        self.n_logged = 0
        self.last_report = -1e9

    def pump(self, vx, wz, flags):
        n_before = self.n_tlm
        verdict = super().pump(vx, wz, flags)
        if verdict or self.n_tlm == n_before or self.tlm is None:
            return verdict
        tl, t = self.tlm, self.now()
        j = list(tl.joints) if tl.joints else []
        self.w.writerow([f"{t:.3f}", f"{time.time():.3f}", tl.state.value,
                         f"{tl.vbat_v:.2f}", f"{tl.up_z:.3f}",
                         f"{tl.vx_est:.3f}", f"{tl.wz_est:.3f}",
                         f"0x{tl.servo_err:02x}", tl.loop_late_pct,
                         40 if j else 20,
                         f"{tl.up_xy[0]:.3f}" if tl.up_xy else "", f"{tl.up_xy[1]:.3f}" if tl.up_xy else ""] + [f"{x:.4f}" for x in j])
        self.f.flush()
        self.n_logged += 1
        live = tl.state in (LinkState.LIVE, LinkState.STAND)
        if not live:
            self.hist.clear()
            self.tilt_streak = 0
            return None
        self.min_upz = min(self.min_upz, tl.up_z)
        # torso tilt guard: two consecutive live beacons below threshold
        self.tilt_streak = self.tilt_streak + 1 if tl.up_z < self.tilt else 0
        if self.tilt_streak >= 2:
            return f"tilt: up_z={tl.up_z:+.2f} < {self.tilt} for 2 beacons"
        # joint swing guard: peak-to-peak over the trailing window
        if j:
            self.hist.append((t, j))
            while self.hist and t - self.hist[0][0] > self.window:
                self.hist.popleft()
            if len(self.hist) >= 3:
                pp = [max(h[1][i] for h in self.hist) -
                      min(h[1][i] for h in self.hist) for i in range(len(j))]
                worst = max(range(len(pp)), key=pp.__getitem__)
                self.max_pp = max(self.max_pp, pp[worst])
                if t - self.last_report >= 0.5:
                    self.last_report = t
                    print(f"        pp/{self.window:g}s: {JOINTS[worst]}="
                          f"{pp[worst]:.3f} rad  up_z_min={self.min_upz:+.2f}",
                          flush=True)
                if pp[worst] > self.pp_rad:
                    return (f"swing: {JOINTS[worst]} p-p {pp[worst]:.3f} rad "
                            f"> {self.pp_rad} over {self.window:g} s")
        return None

    def close(self):
        self.f.close()
        super().close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True)
    p.add_argument("--duration", type=float, default=8.0,
                   help="seconds of ARM stand before a clean disarm")
    p.add_argument("--tilt", type=float, default=0.90,
                   help="up_z below this (2 beacons) -> ESTOP+disarm")
    p.add_argument("--pp-rad", type=float, default=0.5,
                   help="any joint peak-to-peak over 1 s above this -> bail")
    p.add_argument("--csv", required=True)
    p.add_argument("--home", action="store_true",
                   help="RESET SERVOS (FLAG_HOME edge) first; arm only if the "
                        "robot reports DISARMED_HOME")
    p.add_argument("--home-settle", type=float, default=4.0)
    p.add_argument("--no-prearm", action="store_true",
                   help="skip the ARM=0 preamble (twin --boot-armed reference)")
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--tlm-port", type=int, default=None)
    p.add_argument("--watch", metavar="HOST[:PORT]",
                   help="forward every beacon to a `bimo_gui --readonly` watcher (HOST[:PORT]); the robot beacons only to whoever commanded it last, so a second console cannot listen in")
    a = p.parse_args()

    kw = {k: v for k, v in (("cmd_port", a.port), ("tlm_port", a.tlm_port),
                            ("watch", a.watch))
          if v is not None}
    d = OscDriver(a.host, a.csv, a.tilt, a.pp_rad, **kw)
    verdict = None
    try:
        verdict = None if a.no_prearm else \
            d.run_for(0.5, arm_off, "pre-arm: ARM=0 frames (latch level)")
        if not verdict and a.home:
            d.run_for(0.4, lambda t: (0.0, 0.0, FLAG_HOME), "RESET SERVOS: FLAG_HOME edge")
            # the robot says HOME_PENDING until it has read the joints back
            # after the slew (firmware 2026-09-03); wait that out, up to 10 s
            r = None
            for _ in range(10):
                d.run_for(1.0, arm_off, "home settle: ARM=0 frames")
                r = diag_arm_result(d.tlm.diag) if d.tlm is not None else None
                if r is not None and r is not ArmResult.HOME_PENDING:
                    break
            print(f"== home verdict: {r} -- {diag_reason(d.tlm.diag) if d.tlm else 'no beacon'}", flush=True)
            if r is not ArmResult.DISARMED_HOME:
                verdict = f"home not confirmed ({r}); not arming"
                d.run_for(0.5, arm_off, "abort: ARM=0 frames")
        if not verdict:
            verdict = d.run_for(a.duration,
                                lambda t: (0.0, 0.0, FLAG_ARM | FLAG_POSE | FLAG_ATT),
                                "ARM|POSE zero-command stand")
        if verdict and not verdict.startswith("home not confirmed"):
            d.emergency(verdict)
        elif not verdict:
            d.run_for(1.0, arm_off, "disarm: ARM=0 frames")
    except KeyboardInterrupt:
        d.emergency("operator ctrl-C")
        verdict = "interrupted"
    finally:
        print(f"== done: {d.seq - d.seq0} frames sent, {d.n_tlm} beacons "
              f"({d.n_logged} logged), max joint p-p {d.max_pp:.3f} rad, "
              f"min up_z {d.min_upz:+.2f}; states: {d.states_seen}", flush=True)
        if d.tlm is not None and d.tlm.state == LinkState.BENCH:
            print(f"== benched: {diag_reason(d.tlm.seq_echo)}", flush=True)
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
