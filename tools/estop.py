#!/usr/bin/env python3
"""Operator E-stop for the robot over Wi-Fi: one command, no session needed.

    tools/estop.py                 HARD stop: torque off now, run ended (limp)
    tools/estop.py --soft          SOFT stop: policy stopped, joints HELD stiff,
                                   run ended; the robot homes once upright and still
    tools/estop.py --button        a terminal "big red button": any key = HARD,
                                   s = SOFT, q = quit (no start-up delay per press)

It needs nothing but python3 (stdlib) and this repo's link/protocol.py, so it
starts in well under 100 ms and runs with or without a Claude session, a
commander, or the tether. The stop is the out-of-band operator-stop frame
(linkproto::kFlagStop, docs/control-channel.md): the robot acts on it on receipt
in any mode, including a tethered `run` with no commander, and no other sender
can undo it -- it never reaches the ARM/home latches or the link watchdog.
A HARD frame also carries the classic ESTOP flag with ARM clear, so firmware
older than kFlagStop still latches its watchdog E-stop (torque off).

Where it sends: --host, else $BIMO_HOST, else the robot address the flight
collector last heard (hw_sessions/robot_addr in the main checkout), AND the
LAN broadcast (--no-broadcast to skip). Each frame is repeated at 100 Hz for
--seconds (default 1): the robot's command mailbox is latest-wins, so a lone
frame could be overwritten by a commander's in the same 20 ms tick.

Then it listens for the robot's beacon (it goes to the last sender) and says
what the robot reports. Exit 0 = the robot confirmed (BENCH with the STOP or
FLAIL reason, or ESTOP/FALLEN; for a --soft stop on a robot that was already
BENCH, BENCH with no servo reset in progress -- see confirmed()); 2 = frames
sent, no confirmation heard.
A --soft stop the robot does not confirm within --confirm-ms is ESCALATED to a
hard stop: older firmware reads a soft frame as a plain stand, and a stop that
might not have happened is not one.

The servo power switch is still the last word: keep a hand on it.
"""
from __future__ import annotations

import os
import socket
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "link"))

ADDR_FILE = "robot_addr"     # in <main checkout>/hw_sessions/, written by flight_collector.py
SEND_HZ = 100.0

# The first frame goes out BEFORE link/protocol.py (typing, enum: ~50 ms) is
# imported -- it is only needed to read the beacon. So the command frame is
# encoded here, byte for byte protocol.encode_command's classic 14 B frame;
# tests/test_estop.py asserts the two agree, and these constants against it.
CMD_PORT, TLM_PORT = 4210, 4211
FLAG_ESTOP, FLAG_STOP = 1 << 1, 1 << 6
_VERSION = 2


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_stop(seq: int, flags: int) -> bytes:
    body = struct.pack("<2sBBIhh", b"BM", _VERSION, flags & 0xFF, seq & 0xFFFFFFFF, 0, 0)
    return body + struct.pack("<H", _crc16(body))


P = None    # link/protocol.py, imported after the first frame


def _protocol():
    global P
    if P is None:
        import protocol  # noqa: E402  (stdlib only)
        P = protocol
    return P


def main_checkout() -> str:
    import subprocess
    root = os.path.dirname(HERE)
    try:
        common = subprocess.run(["git", "-C", root, "rev-parse", "--path-format=absolute",
                                 "--git-common-dir"], capture_output=True, text=True,
                                timeout=2, check=True).stdout.strip()
        return os.path.dirname(common)
    except (OSError, subprocess.SubprocessError):
        return root


def robot_host(arg: str | None) -> tuple[str | None, str]:
    """The unicast target and where it came from."""
    if arg:
        return arg, "--host"
    if os.environ.get("BIMO_HOST"):
        return os.environ["BIMO_HOST"], "$BIMO_HOST"
    path = os.path.join(main_checkout(), "hw_sessions", ADDR_FILE)
    try:
        with open(path) as f:
            ip = f.read().split()[0]
        return ip, f"flight collector ({path})"
    except (OSError, IndexError):
        return None, "unknown"


def frame_flags(hard: bool) -> int:
    # ARM clear and ENABLE clear on purpose: on old firmware, dropping ARM can
    # only ever bench (torque off), and ESTOP latches the watchdog.
    return FLAG_STOP | (FLAG_ESTOP if hard else 0)


class Stopper:
    def __init__(self, host: str | None, broadcast: str | None, port: int, tlm_port: int):
        self.targets = [(h, port) for h in (host, broadcast) if h]
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.rx = None
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            rx.bind(("0.0.0.0", tlm_port))
            rx.setblocking(False)
            self.rx = rx
        except OSError as e:
            print(f"  (cannot listen for the beacon on udp {tlm_port}: {e}; sending blind)",
                  flush=True)
        # A large, time-derived seq: on OLD firmware the watchdog drops a seq
        # within 1000 below the last one it applied (a running commander's).
        self.seq = int(time.time() * 1000) & 0x3FFFFFFF
        self.sent = 0
        self.last = None      # (the newest beacon, monotonic time heard)

    def send(self, hard: bool) -> None:
        wire = encode_stop(self.seq, frame_flags(hard))
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        for t in self.targets:
            try:
                self.tx.sendto(wire, t)
                self.sent += 1
            except OSError as e:
                print(f"  send to {t[0]} failed: {e}", flush=True)

    def beacons(self):
        """Every telemetry frame waiting on the socket (non-blocking). The
        newest is also kept, with its time, as `last`."""
        out = []
        P = _protocol()
        while self.rx is not None:
            try:
                data, src = self.rx.recvfrom(256)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                out.append((P.decode_telemetry(data), src[0]))
            except P.ProtocolError:
                pass
        if out:
            self.last = (out[-1][0], time.monotonic())
        return out


def confirmed(t, hard: bool, was_bench: bool = False) -> bool:
    """Whether beacon `t` confirms a stop of this kind.

    `was_bench`: the robot was already BENCH when the stop was pressed. A
    benched robot's soft stop (firmware cli.cpp flailTickImpl) stops any bench
    motion where it is -- pose stream, balance, home -- and changes neither
    the mode nor the reported reason, so DISARMED_STOP never comes. What the
    beacon CAN show is that it is still BENCH (no run to end) and not in a
    servo reset (HOME_PENDING, the one bench motion on the wire). A stop
    during a reset leaves HOME_PENDING standing on the robot (the stop clears
    the reset without a verdict), so that case is never confirmed soft and
    escalates: nothing on the wire proves the joints stopped."""
    P = _protocol()
    if t.state in (P.LinkState.ESTOP, P.LinkState.FALLEN):
        return hard          # a soft stop that went limp is not what was asked; still a stop
    if t.state is P.LinkState.BENCH:
        r = P.diag_arm_result(t.diag)
        if r in (P.ArmResult.DISARMED_STOP, P.ArmResult.DISARMED_FLAIL):
            return True
        if hard:
            return r is not None   # benched by anything is torque-off enough for HARD
        return was_bench and r is not P.ArmResult.HOME_PENDING
    return False


def stop(st: Stopper, hard: bool, seconds: float, confirm_ms: float, quiet=False) -> int:
    kind = "HARD" if hard else "SOFT"
    t0 = time.monotonic()
    period = 1.0 / SEND_HZ
    nxt = t0
    seen = None
    ok = False
    escalated = False
    end = t0 + seconds
    # Was the robot BENCH at the press? From a beacon heard in the last second
    # (--button keeps the socket drained between presses); otherwise from the
    # first one this burst draws -- the robot beacons to the last sender, so
    # that is the first we hear of it, and a robot the stop benched from a
    # run says DISARMED_STOP there, which confirms on its own.
    pre = st.last if st.last and t0 - st.last[1] < 1.0 else None
    was_bench = None if pre is None else pre[0].state is _protocol().LinkState.BENCH
    while True:
        now = time.monotonic()
        if now >= nxt:
            st.send(hard)
            nxt += period
        for t, src in st.beacons():
            seen = (t, src)
            if was_bench is None:
                was_bench = t.state is _protocol().LinkState.BENCH
            if confirmed(t, hard, was_bench):
                ok = True
        if not hard and not ok and (now - t0) * 1000.0 >= confirm_ms:
            hard = escalated = True
            kind = "HARD (escalated)"
            end = max(end, now + 1.0)
            if not quiet:
                print(f"  soft stop NOT confirmed in {confirm_ms:.0f} ms -- escalating to HARD",
                      flush=True)
        if now >= end and (ok or now >= end + 0.5):
            break      # after the burst, up to 0.5 s more to hear the verdict
        time.sleep(max(0.0, min(nxt, end) - time.monotonic()) if now < end else 0.02)
    dt = time.monotonic() - t0
    if seen is None:
        print(f"{kind} STOP sent: {st.sent} frames in {dt:.2f} s to "
              f"{', '.join(h for h, _ in st.targets)} -- NO beacon heard. Check the robot; "
              f"servo power switch if in doubt.", flush=True)
        return 2
    P = _protocol()
    t, src = seen
    why = P.diag_reason(t.diag) if t.state is P.LinkState.BENCH else ""
    verdict = "CONFIRMED" if ok else "NOT confirmed"
    print(f"{kind} STOP {verdict}: robot {src} reports {t.state.name}"
          f"{(' -- ' + why) if why else ''} (up_z {t.up_z:.2f}, {st.sent} frames, {dt:.2f} s)",
          flush=True)
    if escalated:
        print("  (escalated: older firmware, or the soft stop did not take)", flush=True)
    return 0 if ok else 2


def button(st: Stopper, seconds: float, confirm_ms: float) -> int:
    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        sys.exit("--button needs a terminal")
    old = termios.tcgetattr(fd)
    print("E-STOP BUTTON: any key = HARD stop (limp), s = SOFT stop (hold), q = quit", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([fd], [], [], 1.0)
            st.beacons()                      # keep the socket drained
            if not r:
                continue
            ch = os.read(fd, 1)
            if ch in (b"q", b"Q"):
                return 0
            stop(st, ch not in (b"s", b"S"), seconds, confirm_ms)
            print("ready (any key = HARD, s = SOFT, q = quit)", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None) -> int:
    import argparse
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("\n", 2)[2])
    p.add_argument("--host", help="robot IP (default: $BIMO_HOST, then the flight collector's)")
    p.add_argument("--soft", action="store_true", help="hold stiff instead of going limp")
    p.add_argument("--button", action="store_true", help="interactive: a key press stops")
    p.add_argument("--seconds", type=float, default=1.0, help="burst length (default 1 s)")
    p.add_argument("--confirm-ms", type=float, default=400.0,
                   help="a soft stop not confirmed by then becomes hard (default 400)")
    p.add_argument("--broadcast", default="255.255.255.255",
                   help="also send to this broadcast address (default 255.255.255.255)")
    p.add_argument("--no-broadcast", action="store_true")
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    a = p.parse_args(argv)
    host, src = robot_host(a.host)
    bcast = None if a.no_broadcast else a.broadcast
    if host is None and bcast is None:
        sys.exit("no robot address: --host, $BIMO_HOST, or a running flight collector")
    print(f"estop -> {host or '(no unicast address)'} [{src}]"
          f"{' + broadcast ' + bcast if bcast else ''}", flush=True)
    st = Stopper(host, bcast, a.port, a.tlm_port)
    if a.button:
        return button(st, a.seconds, a.confirm_ms)
    return stop(st, not a.soft, a.seconds, a.confirm_ms)


if __name__ == "__main__":
    sys.exit(main())
