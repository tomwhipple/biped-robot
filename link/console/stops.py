"""HOLD and E-STOP: the out-of-band operator stops (linkproto kFlagStop).

The frames and the confirmation rule are tools/estop.py's (encode_stop,
frame_flags, confirmed), imported rather than copied. What differs is only
the plumbing a window needs:

* The burst runs on its own thread with its own socket, so a stop goes out
  even if the window hangs -- the one place this console sends from anywhere
  but the GUI thread, because a stop is the one frame that must not depend
  on the GUI being alive.
* It never binds the beacon port. The console's link already has it, and a
  second socket bound there (estop.Stopper does, with SO_REUSEPORT) would
  take beacons away from the console. The console feeds the burst what the
  robot reports instead (StopBurst.feed).

Both work in every role, an observer's included: the robot never puts a stop
frame in its command mailbox, so no latch or watchdog sees it and no commander
can undo it (docs/control-channel.md).
"""
from __future__ import annotations

import functools
import ipaddress
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "link"))

import estop  # noqa: E402  tools/estop.py
import protocol  # noqa: E402  link/protocol.py

BROADCAST = "255.255.255.255"
_STATES = list(protocol.LinkState)     # wire order


class _Tlm:
    """The two fields estop.confirmed() reads, from the core's status."""
    __slots__ = ("state", "diag")

    def __init__(self, state: int, diag: int):
        self.state = _STATES[state] if 0 <= state < len(_STATES) else None
        self.diag = diag


@functools.lru_cache(maxsize=32)
def resolve(host: str) -> str:
    """`host` as a dotted quad (a name is looked up once and remembered: a
    stop press must not wait on DNS twice); unchanged if it does not resolve,
    so the send fails and says so."""
    try:
        return str(ipaddress.IPv4Address(host))
    except ValueError:
        pass
    try:
        return socket.gethostbyname(host)
    except OSError:
        return host


def is_loopback(host: str | None) -> bool:
    try:
        return bool(host) and ipaddress.ip_address(resolve(host)).is_loopback
    except ValueError:
        return False


def targets(host: str | None, port: int, broadcast: str | None = BROADCAST):
    """Where a stop goes: the robot, and the LAN broadcast unless the robot is
    this machine (a sim on loopback -- `localhost` included, by resolving the
    name first) or broadcasting is off."""
    out = []
    if host and host != "0.0.0.0":
        out.append((resolve(host), port))
    if broadcast and not is_loopback(host):
        out.append((broadcast, port))
    return out


def stop_host(host: str | None) -> tuple[str | None, str]:
    """estop.robot_host: --host, else $BIMO_HOST, else the address the flight
    collector last heard."""
    if host and host != "0.0.0.0":
        return host, "--host"
    return estop.robot_host(None)


class StopBurst(threading.Thread):
    """One press: frames at estop.SEND_HZ for `seconds`; a soft stop not
    confirmed within `confirm_ms` becomes hard (estop.stop's rule)."""

    def __init__(self, hard: bool, dests, seconds: float = 1.0,
                 confirm_ms: float = 400.0, send_hz: float = estop.SEND_HZ,
                 was_bench: bool = False):
        super().__init__(daemon=True, name="stop-burst")
        self.asked_hard = hard
        # The robot was BENCH (by a fresh beacon) at the press: a soft stop
        # is then confirmed by BENCH with no servo reset going on, because a
        # benched robot's soft stop changes no reported reason (estop.confirmed).
        self.was_bench = was_bench
        self._hard = hard
        self.dests = list(dests)
        self.seconds = seconds
        self.confirm_ms = confirm_ms
        self.period = 1.0 / send_hz
        self.sent = 0
        self.errors: list[str] = []
        self.escalated = False
        self.done = False
        self.t0 = time.monotonic()
        self.end = self.t0 + seconds  # when frames stop; escalation moves it
        self.dt = 0.0
        self.verdict = ""            # the robot's report, once heard
        self._lock = threading.Lock()
        self._ok = threading.Event()
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Time-derived, like estop.Stopper: old firmware drops a seq within
        # 1000 below the last one it applied.
        self.seq = int(time.time() * 1000) & 0x3FFFFFFF

    @property
    def hard(self) -> bool:
        with self._lock:
            return self._hard

    @property
    def confirmed(self) -> bool:
        return self._ok.is_set()

    def feed(self, state: int, diag: int) -> None:
        """A beacon the console decoded (any thread)."""
        hard = self.hard
        if estop.confirmed(_Tlm(state, diag), hard, self.was_bench):
            if not self._ok.is_set():
                with self._lock:
                    name = _STATES[state].name if 0 <= state < len(_STATES) else "?"
                    why = (protocol.diag_reason(diag)
                           if state == _STATES.index(protocol.LinkState.BENCH) else "")
                    self.verdict = name + (f" -- {why}" if why else "")
            self._ok.set()

    def _send(self) -> None:
        wire = estop.encode_stop(self.seq, estop.frame_flags(self.hard))
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        for d in self.dests:
            try:
                self.tx.sendto(wire, d)
                self.sent += 1
            except OSError as e:
                if len(self.errors) < 8:
                    self.errors.append(f"{d[0]}: {e}")

    def left(self) -> float:
        """Seconds until the last frame goes out (0 once it has)."""
        return max(0.0, self.end - time.monotonic())

    def run(self) -> None:
        t0 = self.t0
        nxt = t0
        end = self.end
        while True:
            now = time.monotonic()
            if now < end and now >= nxt:
                self._send()
                nxt += self.period
            if (not self.hard and not self._ok.is_set()
                    and (now - t0) * 1000.0 >= self.confirm_ms):
                with self._lock:
                    self._hard = True
                self.escalated = True
                end = max(end, now + 1.0)
                self.end = end
            if now >= end:
                # After the burst, up to 0.5 s more to hear the verdict.
                self._ok.wait(max(0.0, end + 0.5 - now))
                break
            time.sleep(max(0.0, min(nxt, end) - time.monotonic()))
        self.dt = time.monotonic() - t0
        self.tx.close()
        self.done = True

    def summary(self) -> str:
        kind = ("HARD (escalated)" if self.escalated
                else "HARD" if self.asked_hard else "SOFT")
        where = ", ".join(h for h, _ in self.dests) or "nowhere"
        if self.confirmed:
            return (f"{kind} STOP CONFIRMED: robot reports {self.verdict} "
                    f"({self.sent} frames, {self.dt:.2f} s)")
        return (f"{kind} STOP sent: {self.sent} frames in {self.dt:.2f} s to "
                f"{where} -- NOT confirmed. Check the robot; servo power "
                f"switch if in doubt.")
