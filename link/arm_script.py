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

Guard: a robot-latched FALLEN, servo fault bits, or up_z < 0.5 (held 0.3 s)
-> ESTOP frames for 1 s, then disarm, then exit 2.  The tilt guard only judges
LIVE/STAND beacons (a benched beacon repeats a stale snapshot), so FALLEN is
checked on its own: without it a fallen robot kept receiving the rest of the
script, which is what the 2026-08-31 ground attempts did (issue #29).
"""
import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from protocol import (ArmResult, diag_arm_result, CMD_PORT, FLAG_ARM, FLAG_ENABLE, FLAG_ESTOP, FLAG_HOME,  # noqa: E402
                      SEND_HZ, TLM_PORT, LinkState, ProtocolError,
                      decode_telemetry, diag_reason, encode_command)
from sources import EXT_KEYS, SCRIPTS  # noqa: E402


def _watch_addr(spec, default_port):
    """"HOST" or "HOST:PORT" -> (host, port).

    No port means the one this process is bound to, which is where a stock
    `bimo_gui --readonly` listens.
    """
    host, _, port = spec.rpartition(":")
    if not host:                     # no colon at all
        return spec, default_port
    return host, int(port)


class Driver:
    def __init__(self, host, cmd_port=CMD_PORT, tlm_port=TLM_PORT,
                 send_hz=SEND_HZ, watch=None):
        self.host, self.cmd_port = host, cmd_port
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("0.0.0.0", tlm_port))
        self.rx.setblocking(False)
        self.period = 1.0 / send_hz
        # Seq as if this sender had been running since machine boot at the
        # send rate: any LATER process starts above any earlier one's current
        # seq. The robot's watchdog keeps last_seq across senders (it resets
        # only on an arm edge) and rejects small backwards jumps as replays --
        # on 2026-08-30 a phase-per-process run had its entire walk window
        # silently rejected (seq restarted at 0, jump < kSeqResyncGap).
        self.seq = int(time.monotonic() * send_hz) & 0xFFFFFFFF
        self.seq0 = self.seq
        self.t0 = time.monotonic()
        self.tlm = None
        self.n_tlm = 0
        self.last_state = None
        self.last_print = -1e9
        self.down_since = None
        self.states_seen = []
        self.fault_streak = 0
        self._n_seen = 0
        # Telemetry fan-out to a `bimo_gui --readonly` watcher. The robot
        # beacons to WHOEVER COMMANDED LAST and to nobody else
        # (firmware/main/wifi_link.cpp), so a second console cannot listen in
        # -- and the only way for it to become the destination is to send a
        # command, which would steal the beacon from THIS process and drive
        # the robot mid-probe. So the driver forwards instead. Every one of
        # these probes runs unattended for tens of seconds on real hardware;
        # this is how somebody gets to watch one happen.
        self.watch = _watch_addr(watch, tlm_port) if watch else None
        self.n_watched = 0
        if self.watch is not None:
            print(f"forwarding telemetry to "
                  f"{self.watch[0]}:{self.watch[1]}", flush=True)

    def now(self):
        return time.monotonic() - self.t0

    @staticmethod
    def host_clock_line():
        """One line for the log: host UTC now, and whether it is NTP-synced.

        The kernel flag systemd's timedatectl reads; "unknown" where that
        cannot be asked. A `utc=` column from an unsynced host is a number,
        not a time, and the log should say so up front."""
        synced = "unknown"
        try:
            out = subprocess.run(
                ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                capture_output=True, text=True, timeout=2).stdout.strip()
            synced = {"yes": "synced", "no": "UNSYNCED"}.get(out, "unknown")
        except (OSError, subprocess.SubprocessError):
            pass
        return (f"== host clock utc={time.time():.3f} "
                f"({time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}) "
                f"ntp={synced}")

    def send(self, vx, wz, flags, ext=None):
        self.tx.sendto(encode_command(self.seq, vx, wz, flags, **(ext or {})),
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
                continue          # stray traffic: decoded here, never relayed
            if self.watch is not None:
                # Verbatim, so the watcher runs the same decode over the same
                # bytes the robot signed. Best-effort: a probe on real
                # hardware must never die because a watcher went away.
                try:
                    self.tx.sendto(buf, self.watch)
                    self.n_watched += 1
                except OSError:
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
            # Two wall clocks on every line (docs/control-channel.md "Time on
            # the wire"): the host's, which is the time base burned into the
            # webcam frames by tools/cam_record.sh, and the robot's own SNTP
            # stamp from the beacon. `robot=unsynced` means the robot has not
            # heard from NTP yet (or runs pre-stamp firmware) -- correlate on
            # `utc` alone in that case, and say so in the write-up.
            robot = ("unsynced" if tl.t_utc is None
                     else f"{tl.t_utc:.3f}")
            print(f"t={t:6.2f} utc={time.time():.3f} robot={robot} "
                  f"cmd vx={vx:+.2f} wz={wz:+.2f} "
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
        if tl.state == LinkState.FALLEN:
            # The robot latched a fall: torso down, torque off, and (firmware
            # ctrl_task) it has disarmed itself. Nothing after this point is
            # a supervised motion -- keeping the script running just streams
            # commands at a robot on the floor, and holding ARM into the next
            # phase re-engages torque under whoever is picking it up. The
            # 2026-08-31 ground attempts ran on past exactly this.
            return "robot latched FALLEN (torso down, torque off)"
        live = tl.state in (LinkState.LIVE, LinkState.STAND)
        if self.n_tlm != self._n_seen:      # only judge FRESH beacons
            self._n_seen = self.n_tlm
            if not live:
                # A benched beacon repeats the LAST published snapshot -- after
                # a dead-bus session it still says 0xff (and up_z whatever was
                # last true). Fault bits only mean something while the loop is
                # publishing, i.e. LIVE/STAND; judging them while benched
                # e-stopped a healthy arm on 2026-08-30.
                self.fault_streak = 0
            # Debounced: the fault mask is one 20 ms tick's snapshot, and a
            # single overload flicker or missed servo reply sets a bit for
            # one frame (seen twice on 2026-08-30, healthy servo both times).
            # Three consecutive beacons = 0.3 s, same rule as the tilt guard.
            else:
                self.fault_streak = self.fault_streak + 1 if tl.servo_err else 0
        if self.fault_streak >= 3:
            return f"servo fault bits 0x{tl.servo_err:02x} for 3+ beacons"
        if live and tl.up_z < 0.5:
            self.down_since = self.down_since or t
            if t - self.down_since > 0.3:
                return f"up_z={tl.up_z:+.2f} for >0.3 s"
        else:
            self.down_since = None
        return None

    def run_for(self, seconds, fn, label):
        """fn(t_phase) -> (vx, wz, flags[, ext]). Returns guard verdict or
        None. ext is an optional dict of extended command channels
        (protocol.encode_command kwargs)."""
        print(f"== {label} ({seconds:g} s)", flush=True)
        start = self.now()
        next_send = start
        while True:
            t = self.now() - start
            if t >= seconds:
                return None
            out = fn(t)
            vx, wz, flags = out[0], out[1], out[2]
            ext = out[3] if len(out) > 3 else None
            self.send(vx, wz, flags, ext)
            verdict = self.pump(vx, wz, flags)
            if verdict:
                return verdict
            next_send += self.period
            time.sleep(max(0.0, next_send - self.now()))

    def emergency(self, why):
        print(f"!! GUARD: {why} -- sending ESTOP then disarm", flush=True)
        self.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ARM | FLAG_ESTOP), "ESTOP")
        self.run_for(1.0, lambda t: (0.0, 0.0, 0), "disarm")
        # Tom, 2026-09-07: "always reset servos a few seconds after the fall
        # protection engages". The ESTOP leaves the servos holding the fallen
        # pose; after the spotter has righted the robot, a RESET SERVOS edge
        # puts it back at home so the next run starts clean.
        self.run_for(3.0, lambda t: (0.0, 0.0, 0), "hold (spotter rights the robot)")
        self.home_and_release(FLAG_HOME, "RESET SERVOS after guard")

    def home_and_release(self, edge_flags, label="RESET SERVOS"):
        """The zeroing: a FLAG_HOME edge, then wait for the firmware's HOME
        verdict (since 2026-09-13 the routine slews, rolls each hip out and
        back to take up play, reads back and RELEASES torque itself -- ~8 s),
        then silence. Only if the reset did not finish clean is the ESTOP
        latch sent as the fallback release (Tom: a zeroing is not done until
        torque is off). Returns the verdict."""
        self.run_for(0.4, lambda t: (0.0, 0.0, edge_flags), f"{label}: FLAG_HOME edge")
        r = None
        for _ in range(16):
            self.run_for(1.0, lambda t: (0.0, 0.0, 0), "home: slew + hip wiggle + readback")
            r = diag_arm_result(self.tlm.diag) if self.tlm is not None else None
            if r in (ArmResult.DISARMED_HOME, ArmResult.HOME_NOT_REACHED,
                     ArmResult.HOME_NO_CAL, ArmResult.HOME_LOW_BATT, ArmResult.HOME_BUS_FAILED):
                break
        print(f"== {label} verdict: {r}", flush=True)
        if r is not ArmResult.DISARMED_HOME:
            self.run_for(1.0, lambda t: (0.0, 0.0, FLAG_ARM | FLAG_ESTOP),
                         "reset not clean -> ESTOP = torque release, then silence")
        return r

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
    # Same preamble as phase_arm: a fresh ArmLatch has no level until it sees
    # a frame, so ARM=1 from frame one is a level, not an edge, and the robot
    # (correctly) never arms. Observed 2026-08-31 after a serial-open reboot.
    v = d.run_for(0.5, arm_off, "pre-arm: ARM=0 frames (establish latch level)")
    if v:
        return v
    v = d.run_for(a.settle, stand_armed, "settle: ARM stand frames")
    if v:
        return v

    def script(t):
        out = fn(t)
        if isinstance(out, dict):
            # dict scripts drive the extended channels (sources.EXT_KEYS)
            ext = {k: out[k] for k in EXT_KEYS if k in out}
            vx, wz = out.get("vx", 0.0), out.get("wz", 0.0)
            active = bool(vx or wz or any(ext.values()))
            flags = FLAG_ARM | (FLAG_ENABLE if active else 0)
            return vx, wz, flags, ext
        vx, wz = out
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
    p.add_argument("--watch", metavar="HOST[:PORT]",
                   help="forward every beacon to a `bimo_gui --readonly` watcher (HOST[:PORT]); the robot beacons only to whoever commanded it last, so a second console cannot listen in")
    a = p.parse_args()

    d = Driver(a.host, cmd_port=a.port, tlm_port=a.tlm_port, watch=a.watch)
    print(Driver.host_clock_line(), flush=True)
    try:
        verdict = {"arm": phase_arm, "script": phase_script,
                   "disarm": phase_disarm}[a.phase](d, a)
        if verdict:
            d.emergency(verdict)
    except KeyboardInterrupt:
        d.emergency("operator ctrl-C")
        verdict = "interrupted"
    finally:
        print(f"== done: {d.seq - d.seq0} frames sent, {d.n_tlm} telemetry received; "
              f"states: {d.states_seen}", flush=True)
        if d.tlm is not None and d.tlm.state == LinkState.BENCH:
            print(f"== benched: {diag_reason(d.tlm.seq_echo)}", flush=True)
        d.close()
    sys.exit(2 if verdict else 0)


if __name__ == "__main__":
    main()
