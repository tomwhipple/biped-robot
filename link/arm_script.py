#!/usr/bin/env python3
"""Arm the robot over the radio and run one scripted motion, ARM held.

commander.py never sets FLAG_ARM, so against firmware >= 42080a3 it can only
drive a robot something ELSE has armed -- and its own frames make the 1->0
edge that benches the robot again (firmware ArmLatch is one global latch,
linkproto/watchdog.cpp).  This tool holds ARM in every frame and runs a
supervised sequence in phases:

  arm      0.5 s of ARM-off frames (so the edge exists even on a fresh
           latch), then ARM stand frames for --duration s.  Prints every
           telemetry state change.  Ends still armed; the watchdog then
           parks the robot (STAND -> RELAX, torque off) on its own.
  script   ARM stand frames for --settle s (re-engages torque from RELAX),
           the named link/sources.py script, --tail s of stand, then --
           unless --stay-armed -- 1 s of ARM-off frames = disarm (BENCH).
  disarm   1 s of ARM-off frames.

Guard: servo fault bits or up_z < 0.5 (held 0.3 s) -> ESTOP frames for 1 s,
then disarm, then exit 2.  The robot latches FALLEN itself; this is belt and
braces for the servo-fault case the firmware only reports.
"""
import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from protocol import (CMD_PORT, FLAG_ARM, FLAG_ENABLE, FLAG_ESTOP,  # noqa: E402
                      SEND_HZ, TLM_PORT, LinkState, ProtocolError,
                      decode_telemetry, diag_reason, encode_command)
from sources import SCRIPTS  # noqa: E402


class Driver:
    def __init__(self, host, cmd_port=CMD_PORT, tlm_port=TLM_PORT,
                 send_hz=SEND_HZ):
        self.host, self.cmd_port = host, cmd_port
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", tlm_port))
        self.rx.setblocking(False)
        self.period = 1.0 / send_hz
        self.seq = 0
        self.t0 = time.monotonic()
        self.tlm = None
        self.n_tlm = 0
        self.last_state = None
        self.last_print = -1e9
        self.down_since = None
        self.states_seen = []
        self.fault_streak = 0
        self._n_seen = 0

    def now(self):
        return time.monotonic() - self.t0

    def send(self, vx, wz, flags):
        self.tx.sendto(encode_command(self.seq, vx, wz, flags),
                       (self.host, self.cmd_port))
        self.seq += 1

    def pump(self, vx, wz, flags):
        """Drain telemetry, print changes, return a guard verdict or None."""
        while True:
            try:
                buf, _ = self.rx.recvfrom(64)
            except BlockingIOError:
                break
            try:
                self.tlm = decode_telemetry(buf)
                self.n_tlm += 1
            except ProtocolError:
                pass
        t = self.now()
        tl = self.tlm
        if tl is None:
            if t - self.last_print >= 1.0:
                print(f"t={t:6.2f} cmd vx={vx:+.2f} wz={wz:+.2f} "
                      f"flags=0x{flags:02x}  tlm: none yet", flush=True)
                self.last_print = t
            return None
        changed = tl.state != self.last_state
        if changed:
            self.last_state = tl.state
            self.states_seen.append((round(t, 2), tl.state.value))
        if changed or t - self.last_print >= 0.5:
            print(f"t={t:6.2f} cmd vx={vx:+.2f} wz={wz:+.2f} "
                  f"flags=0x{flags:02x} | {tl.state.value:5s} "
                  f"vbat={tl.vbat_v:4.1f}V up_z={tl.up_z:+.2f} "
                  f"vx_est={tl.vx_est:+.2f} wz_est={tl.wz_est:+.2f} "
                  f"err=0x{tl.servo_err:02x} late={tl.loop_late_pct}% "
                  f"echo={tl.seq_echo}" + ("  <-- state change" if changed else ""),
                  flush=True)
            self.last_print = t
        if changed and tl.state == LinkState.BENCH:
            # Benched, seq_echo is the diagnostic byte (protocol.py DIAG_*):
            # the robot's own answer to "why is nothing happening?".
            print(f"        robot says: {diag_reason(tl.seq_echo)}", flush=True)
        if self.n_tlm != self._n_seen:      # only judge FRESH beacons
            self._n_seen = self.n_tlm
            # Debounced: the fault mask is one 20 ms tick's snapshot, and a
            # single overload flicker or missed servo reply sets a bit for
            # one frame (seen twice on 2026-08-30, healthy servo both times).
            # Three consecutive beacons = 0.3 s, same rule as the tilt guard.
            self.fault_streak = self.fault_streak + 1 if tl.servo_err else 0
        if self.fault_streak >= 3:
            return f"servo fault bits 0x{tl.servo_err:02x} for 3+ beacons"
        live = tl.state in (LinkState.LIVE, LinkState.STAND)
        if live and tl.up_z < 0.5:
            self.down_since = self.down_since or t
            if t - self.down_since > 0.3:
                return f"up_z={tl.up_z:+.2f} for >0.3 s"
        else:
            self.down_since = None
        return None

    def run_for(self, seconds, fn, label):
        """fn(t_phase) -> (vx, wz, flags). Returns guard verdict or None."""
        print(f"== {label} ({seconds:g} s)", flush=True)
        start = self.now()
        next_send = start
        while True:
            t = self.now() - start
            if t >= seconds:
                return None
            vx, wz, flags = fn(t)
            self.send(vx, wz, flags)
            verdict = self.pump(vx, wz, flags)
            if verdict:
                return verdict
            next_send += self.period
            time.sleep(max(0.0, next_send - self.now()))

    def emergency(self, why):
        print(f"!! GUARD: {why} -- sending ESTOP then disarm", flush=True)
        self.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ARM | FLAG_ESTOP), "ESTOP")
        self.run_for(1.0, lambda t: (0.0, 0.0, 0), "disarm")

    def close(self):
        self.tx.close()
        self.rx.close()


def stand_armed(t):
    return 0.0, 0.0, FLAG_ARM


def arm_off(t):
    return 0.0, 0.0, 0


def phase_arm(d, a):
    v = d.run_for(0.5, arm_off, "pre-arm: ARM=0 frames (establish latch level)")
    if v:
        return v
    v = d.run_for(a.duration, stand_armed, "ARM=1 stand frames")
    # If the robot FELL during the settle, do not exit still armed: the
    # FALLEN latch clears after ~2 s upright with ENABLE off, and an armed
    # loop then re-engages torque -- on 2026-08-30 that happened in the
    # operator's hands while righting the robot. A fall ends the session.
    if v is None and any(s == "fall" for _, s in d.states_seen):
        d.run_for(1.0, arm_off, "fall seen during arm -- disarming on exit")
        return "fall during arm phase (disarmed before exit)"
    return v


def phase_script(d, a):
    fn = SCRIPTS[a.script]
    v = d.run_for(a.settle, stand_armed, "settle: ARM stand frames")
    if v:
        return v

    def script(t):
        vx, wz = fn(t)
        flags = FLAG_ARM | (FLAG_ENABLE if (vx or wz) else 0)
        return vx, wz, flags
    v = d.run_for(a.duration, script, f"script {a.script}")
    if v:
        return v
    v = d.run_for(a.tail, stand_armed, "tail: ARM stand frames")
    if v:
        return v
    if not a.stay_armed:
        return d.run_for(1.0, arm_off, "disarm: ARM=0 frames")
    return None


def phase_disarm(d, a):
    return d.run_for(1.0, arm_off, "disarm: ARM=0 frames")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True)
    p.add_argument("--phase", choices=("arm", "script", "disarm"), required=True)
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--script", default="line_1m", choices=sorted(SCRIPTS))
    p.add_argument("--duration", type=float, default=6.0,
                   help="arm: seconds of ARM stand; script: script length")
    p.add_argument("--settle", type=float, default=1.5)
    p.add_argument("--tail", type=float, default=2.0)
    p.add_argument("--stay-armed", action="store_true")
    a = p.parse_args()

    d = Driver(a.host, cmd_port=a.port, tlm_port=a.tlm_port)
    try:
        verdict = {"arm": phase_arm, "script": phase_script,
                   "disarm": phase_disarm}[a.phase](d, a)
        if verdict:
            d.emergency(verdict)
    except KeyboardInterrupt:
        d.emergency("operator ctrl-C")
        verdict = "interrupted"
    finally:
        print(f"== done: {d.seq} frames sent, {d.n_tlm} telemetry received; "
              f"states: {d.states_seen}", flush=True)
        if d.tlm is not None and d.tlm.state == LinkState.BENCH:
            print(f"== benched: {diag_reason(d.tlm.seq_echo)}", flush=True)
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
