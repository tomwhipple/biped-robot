#!/usr/bin/env python3
"""Measure joint PLAY on the real robot with torque OFF.

Zero-motion recipe (proven 2026-09-02 15:39): ARM|ESTOP|POSE frames at 100 Hz
arm the loop but land it in the ESTOP limp branch from the first accepted
frame, so torque never engages while the measured pose is still published in
every beacon. The operator then wiggles each joint by hand through its slop;
this prints the per-joint min/max/peak-to-peak (rad and deg) live and at the
end, and writes every beacon to CSV. ARM=0 at the end benches the robot.
Keep the mailbox full (100 Hz): a starved handover tick reports STAND.

    .venv/bin/python link/play_probe.py --host 192.168.2.90 --duration 60 \
        --csv hw_sessions/2026-09-02/play1.csv
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_script import Driver, arm_off  # noqa: E402
from protocol import (FLAG_ARM, FLAG_ESTOP, FLAG_HOME, FLAG_POSE,  # noqa: E402
                      ArmResult, LinkState, diag_arm_result, diag_reason)

JOINTS = ["L_yaw", "L_roll", "L_pitch", "L_knee", "L_ankle",
          "R_yaw", "R_roll", "R_pitch", "R_knee", "R_ankle"]


class PlayDriver(Driver):
    def __init__(self, host, csv_path, **kw):
        super().__init__(host, send_hz=100.0, **kw)
        self.f = open(csv_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "epoch", "state", "len"] + JOINTS)
        self.lo = [math.inf] * 10
        self.hi = [-math.inf] * 10
        self.last_tab = -1e9
        self.n_pose = 0
        self.states = set()

    def pump(self, vx, wz, flags):
        n_before = self.n_tlm
        while True:
            try:
                buf, _ = self.rx.recvfrom(64)
            except BlockingIOError:
                break
            from protocol import ProtocolError, decode_telemetry
            try:
                self.tlm = decode_telemetry(buf)
                self.n_tlm += 1
            except ProtocolError:
                pass
        if self.n_tlm == n_before or self.tlm is None:
            return None
        tl, t = self.tlm, self.now()
        self.states.add(tl.state.value)
        j = list(tl.joints) if tl.joints else []
        self.w.writerow([f"{t:.3f}", f"{time.time():.3f}", tl.state.value,
                         40 if j else 20] + [f"{x:.4f}" for x in j])
        if tl.state in (LinkState.LIVE, LinkState.STAND):
            # torque would be ON in these states -- the recipe failed; bail.
            return f"state {tl.state.value}: torque may be on, aborting"
        if j and tl.state == LinkState.ESTOP:
            self.n_pose += 1
            for i, x in enumerate(j):
                self.lo[i] = min(self.lo[i], x)
                self.hi[i] = max(self.hi[i], x)
        if t - self.last_tab >= 1.0:
            self.last_tab = t
            self.table(t, tl)
        return None

    def table(self, t, tl):
        print(f"t={t:5.1f} {tl.state.value:5s} vbat={tl.vbat_v:4.1f}V "
              f"pose beacons={self.n_pose}", flush=True)
        if self.n_pose:
            print("   " + " ".join(f"{n:>8s}" for n in JOINTS))
            print("pp " + " ".join(f"{(h - l):8.3f}" for l, h in zip(self.lo, self.hi))
                  + "  rad")
            print("pp " + " ".join(f"{math.degrees(h - l):8.1f}" for l, h in zip(self.lo, self.hi))
                  + "  deg", flush=True)

    def close(self):
        self.f.close()
        super().close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True)
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--csv", required=True)
    p.add_argument("--home", action="store_true",
                   help="RESET SERVOS first (FLAG_HOME edge), then go limp")
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--tlm-port", type=int, default=None)
    p.add_argument("--watch", metavar="HOST[:PORT]",
                   help="forward every beacon to a `bimo_gui --readonly` watcher (HOST[:PORT]); the robot beacons only to whoever commanded it last, so a second console cannot listen in")
    a = p.parse_args()
    kw = {k: v for k, v in (("cmd_port", a.port), ("tlm_port", a.tlm_port),
                            ("watch", a.watch))
          if v is not None}
    d = PlayDriver(a.host, a.csv, **kw)
    verdict = None
    try:
        d.run_for(0.5, arm_off, "pre-arm: ARM=0 frames (latch level)")
        if a.home:
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
                raise SystemExit(f"home not confirmed ({r}); not going limp")
        verdict = d.run_for(a.duration,
                            lambda t: (0.0, 0.0, FLAG_ARM | FLAG_ESTOP | FLAG_POSE),
                            "ARM|ESTOP|POSE limp -- wiggle each joint now")
    except KeyboardInterrupt:
        verdict = "interrupted"
    finally:
        d.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ESTOP), "disarm: ARM=0 (ESTOP kept)")
        print(f"== done: {d.n_tlm} beacons, {d.n_pose} limp pose beacons, "
              f"states {sorted(d.states)}, verdict {verdict}", flush=True)
        d.table(d.now(), d.tlm) if d.tlm else None
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
