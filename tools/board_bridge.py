#!/usr/bin/env python3
"""Hold the robot's serial port open for a whole session, log it, and publish
bench telemetry to consoles over UDP.

    tools/board_bridge.py <fifo> <log> [--telemetry-port 8650] [--poll]
    echo "pos 52" > <fifo>            # an operator command (any process)
    echo "__quit__" > <fifo>          # close the port and exit

WHY IT EXISTS. Opening the board's serial port resets the board, and boot
releases torque -- a robot held up by its servos drops. So the port is opened
exactly ONCE, here, for the whole bench session; everything else (the
operator's shell, scripts, the polling below) goes through this process.

WHAT IT DOES.
  * Lines written to <fifo> (or stdin, with --stdin) are sent to the board.
  * Every line in either direction goes in <log>, UTC-stamped: '>' = an
    operator command, '>~' = a command the poller sent, ' ' = a board line,
    '#' = the bridge's own notes. The format of hw_sessions/*/*serial.log.
  * Board lines are parsed (parse_line) into structured samples -- servo
    readings, balance state, IMU -- and sent as one JSON object per UDP
    datagram to every console that has subscribed. See docs/bench-console.md
    for the datagram format.
  * With --poll it ALSO asks for those readings itself, at a low fixed rate,
    behind the operator (Poller).

SAFETY. The telemetry side never writes to the port. The only thing this
process ever sends that an operator did not type is a Poller command, and the
Poller can only produce POLL_COMMANDS -- `pos <id>`, `balance`, `imu`,
`imu raw 1` -- all read-only queries (tests/test_board_bridge.py pins the set).
Polling is OFF unless --poll is given. Consoles cannot send anything to the
board: a datagram from them is read only as subscribe/unsubscribe.
"""
import argparse
import json
import os
import queue
import re
import select
import socket
import sys
import threading
import time
from datetime import datetime, timezone

# -- what a servo id is called ------------------------------------------------
# The 12 fitted servos, from `cal show` on the as-built robot (hw_sessions/
# 2026-10-24/stand/serial.log). `cal show` replies seen on the wire override
# this, so a re-mapped robot is not mislabelled for long.
JOINT_NAMES = {
    52: "R_hip_roll", 6: "R_hip_pitch", 7: "R_knee", 8: "R_ankle",
    51: "L_hip_roll", 2: "L_hip_pitch", 3: "L_knee", 4: "L_ankle",
    10: "R_hip_yaw", 9: "L_hip_yaw", 11: "R_ankle_roll", 12: "L_ankle_roll",
}
# The order the poller walks them in (and the console lists them).
JOINT_IDS = [52, 6, 7, 8, 11, 51, 2, 3, 4, 12, 10, 9]

PROTOCOL_VERSION = 1
MAX_DATAGRAM = 1200            # one UDP datagram, well under any tunnel MTU
SUB_TTL_S = 10.0               # a console must re-subscribe within this
MAX_SUBSCRIBERS = 4
HEARTBEAT_S = 1.0

# -- parsing ------------------------------------------------------------------
# Unanchored searches on purpose. The firmware prints from more than one task
# and the lines tear -- a real transcript has
#   "           before trusting B -- the mostate    ACTIVE, L stance, 2.5 s ..."
# where a `balance` reply's tail is overwritten by the periodic status, and
# "-- holds aftegains    kp 0.250 ...". Anchoring on column 0 would drop the
# very lines that matter most while the robot is balancing.
_N = r"([+-]?\d+(?:\.\d+)?)"
_RE_POS = re.compile(
    r"\bid\s+(\d+)\s+pos\s+(-?\d+)\s+spd\s+(-?\d+)\s+load\s+(-?\d+)\s+"
    r"(\d+(?:\.\d+)?)\s*V\s+(-?\d+)\s*C\s+err\s+0x([0-9A-Fa-f]+)")
_RE_GAINS = re.compile(
    r"gains\s+kp\s+(\d+(?:\.\d+)?)\s+ki\s+(\d+(?:\.\d+)?)\s*/s\s+"
    r"ankle\s+x" + _N + r"\s+hip\s+x" + _N +
    r"\s+clamp\s+(\d+(?:\.\d+)?)\s*deg\s+abort\s+(\d+(?:\.\d+)?)\s*deg")
_RE_ROLL = re.compile(r"\broll now\s+" + _N + r"\s*deg")
_RE_ROLL_NA = re.compile(r"\broll now unavailable")
_RE_STATE_ON = re.compile(
    r"state\s{2,}(.+?),\s*([LR])\s+stance,\s*(\d+(?:\.\d+)?)\s*s since engage")
_RE_STATE_OFF = re.compile(r"state\s{2,}off\b")
_RE_REF = re.compile(
    r"\bref\s+" + _N + r"\s*deg\s+err\s+" + _N + r"\s*deg\s+I\s+" + _N +
    r"\s+u\s+" + _N + r"\s*deg")
_RE_CORR = re.compile(
    r"correction\s+([LR])\s+ankle roll\s+" + _N + r"\s*deg\s*=\s*([+-]?\d+)"
    r"\s*ticks\s+([LR])\s+hip roll\s+" + _N + r"\s*deg\s*=\s*([+-]?\d+)\s*ticks"
    r"(?:\s+\(envelope clamped (\d+) writes\))?")
_RE_UP = re.compile(r"\bup\s+" + _N + r"\s+" + _N + r"\s+" + _N + r"\s+\(body z")
_RE_PROJ = re.compile(r"\bprojgrav\s+" + _N + r"\s+" + _N + r"\s+" + _N)
_RE_GYRO = re.compile(r"\bgyro\s+" + _N + r"\s+" + _N + r"\s+" + _N + r"\s+rad/s")
_RE_LEVEL = re.compile(r"\blevelled\s+(yes|no)(?:\s+accel-reject streak\s+(\d+))?")
_RE_RAW = re.compile(
    r"\ba\s+" + _N + r"\s+" + _N + r"\s+" + _N + r"\s+\|a\|\s+" + _N +
    r"\s+g\s+" + _N + r"\s+" + _N + r"\s+" + _N)
_RE_CAL = re.compile(
    r"\b(\w+)\s+id\s+(\d+)\s+zero\s+(\d+)\s+dir\s+([+-]1)\s+(fitted|NOT FITTED)")


def _f(s):
    return float(s)


def parse_line(line):
    """One board line -> a list of sample dicts (usually 0 or 1).

    Pure: no clock, no I/O. A sample is {"k": kind, ...fields}; the caller
    adds the version, the time and, for joints, the display name.
    """
    line = line.rstrip("\r\n")
    out = []
    m = _RE_POS.search(line)
    if m:
        out.append({"k": "joint", "id": int(m[1]), "pos": int(m[2]),
                    "spd": int(m[3]), "load": int(m[4]), "volt": _f(m[5]),
                    "temp": int(m[6]), "err": int(m[7], 16)})
    m = _RE_GAINS.search(line)
    if m:
        out.append({"k": "bal_gains", "kp": _f(m[1]), "ki": _f(m[2]),
                    "ankle": _f(m[3]), "hip": _f(m[4]), "clamp": _f(m[5]),
                    "abort": _f(m[6])})
    m = _RE_ROLL.search(line)
    if m:
        out.append({"k": "bal_roll", "roll": _f(m[1])})
    elif _RE_ROLL_NA.search(line):
        out.append({"k": "bal_roll", "roll": None})
    m = _RE_STATE_ON.search(line)
    if m:
        out.append({"k": "bal_state", "state": m[1].strip(), "stance": m[2],
                    "since": _f(m[3])})
    elif _RE_STATE_OFF.search(line):
        out.append({"k": "bal_state", "state": "off", "stance": None,
                    "since": None})
    m = _RE_REF.search(line)
    if m:
        out.append({"k": "bal_ref", "ref": _f(m[1]), "err": _f(m[2]),
                    "I": _f(m[3]), "u": _f(m[4])})
    m = _RE_CORR.search(line)
    if m:
        out.append({"k": "bal_corr", "side": m[1], "ankle_deg": _f(m[2]),
                    "ankle_ticks": int(m[3]), "hip_deg": _f(m[5]),
                    "hip_ticks": int(m[6]),
                    "clamped": int(m[7]) if m[7] else 0})
    m = _RE_UP.search(line)
    if m:
        out.append({"k": "imu_up", "x": _f(m[1]), "y": _f(m[2]), "z": _f(m[3])})
    m = _RE_PROJ.search(line)
    if m:
        out.append({"k": "imu_proj", "x": _f(m[1]), "y": _f(m[2]),
                    "z": _f(m[3])})
    m = _RE_GYRO.search(line)
    if m:
        out.append({"k": "imu_gyro", "x": _f(m[1]), "y": _f(m[2]),
                    "z": _f(m[3])})
    m = _RE_LEVEL.search(line)
    if m:
        out.append({"k": "imu_level", "levelled": m[1] == "yes",
                    "streak": int(m[2]) if m[2] else 0})
    m = _RE_RAW.search(line)
    if m:
        out.append({"k": "imu_raw", "ax": _f(m[1]), "ay": _f(m[2]),
                    "az": _f(m[3]), "amag": _f(m[4]), "gx": _f(m[5]),
                    "gy": _f(m[6]), "gz": _f(m[7])})
    m = _RE_CAL.search(line)
    if m:
        out.append({"k": "cal", "id": int(m[2]), "name": m[1],
                    "zero": int(m[3]), "dir": int(m[4]),
                    "fitted": m[5] == "fitted"})
    return out


def dumps(sample):
    """One datagram's payload. Compact, and refuses to exceed MAX_DATAGRAM."""
    b = json.dumps(sample, separators=(",", ":"),
                   allow_nan=False).encode()
    if len(b) > MAX_DATAGRAM:
        raise ValueError("sample too large for a datagram")
    return b


# -- the telemetry side (never touches the port) --------------------------------
class Publisher:
    """UDP fan-out to subscribed consoles.

    A console subscribes by sending {"sub":1} to the telemetry port and keeps
    that up (every ~2 s) -- the bridge never connects out, so a Mac behind a
    NAT, a sleeping laptop or a restarted console needs no configuration here,
    and a vanished console costs nothing after SUB_TTL_S. {"unsub":1} leaves.
    On subscribe the console is sent the STATE CACHE (newest sample of each
    kind) so a console started mid-session is not blank until the next poll.

    This class holds no reference to the serial port, by construction.
    """

    def __init__(self, bind, port, clock=time.time):
        self.clock = clock
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.port = self.sock.getsockname()[1]
        self.subs = {}                  # (host, port) -> expiry
        self.cache = {}                 # (kind, id) -> payload bytes
        self.lock = threading.Lock()
        self.sent = 0
        self._stop = False
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while not self._stop:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.2)
                if not r:
                    continue
                data, addr = self.sock.recvfrom(512)
            except (OSError, ValueError):
                return
            self.handle(data, addr)

    def handle(self, data, addr):
        try:
            msg = json.loads(data.decode("utf-8", "replace"))
        except ValueError:
            return
        if not isinstance(msg, dict):
            return
        with self.lock:
            if msg.get("unsub"):
                self.subs.pop(addr, None)
            elif msg.get("sub"):
                new = addr not in self.subs
                if new and len(self.subs) >= MAX_SUBSCRIBERS:
                    self._expire()
                    if len(self.subs) >= MAX_SUBSCRIBERS:
                        return
                self.subs[addr] = self.clock() + SUB_TTL_S
                if new:
                    for payload in list(self.cache.values()):
                        self._sendto(payload, addr)

    def _expire(self):
        now = self.clock()
        for a in [a for a, t in self.subs.items() if t < now]:
            del self.subs[a]

    def _sendto(self, payload, addr):
        try:
            self.sock.sendto(payload, addr)
            self.sent += 1
        except OSError:
            pass

    def publish(self, sample, cache=True):
        """Stamp and send one sample; remember it for late subscribers."""
        s = {"v": PROTOCOL_VERSION, "t": round(self.clock(), 3)}
        s.update(sample)
        payload = dumps(s)
        with self.lock:
            if cache:
                self.cache[(s["k"], s.get("id"))] = payload
            self._expire()
            for addr in list(self.subs):
                self._sendto(payload, addr)

    def n_subscribers(self):
        with self.lock:
            self._expire()
            return len(self.subs)

    def close(self):
        self._stop = True
        self.sock.close()


# -- the poller ------------------------------------------------------------------
# The ONLY commands the bridge ever composes itself. Read-only queries.
def poll_commands(joint_ids=JOINT_IDS):
    return ({"balance", "imu", "imu raw 1"} |
            {f"pos {i}" for i in joint_ids})


POLL_COMMANDS = poll_commands()


class Poller:
    """Decides which read-only query, if any, may go out NOW. Pure scheduling:
    it returns a string and never touches a port, so the interleave rule can be
    tested with a fake clock.

    THE INTERLEAVE. The board's CLI is one task reading one line at a time and
    printing the reply; a poll sent into the middle of an operator's command
    would interleave their output and delay their next line. So a poll is
    allowed only when ALL of these hold:
      1. no operator line is waiting (the FIFO is empty), and none was sent in
         the last `quiet_s` (default 1.0 s) -- the operator's own reply, and
         whatever they type next, always go first;
      2. the previous poll's reply is finished: the board has been silent for
         `settle_s` (0.12 s), or `reply_timeout_s` (0.6 s) passed -- so there
         is never more than ONE query outstanding;
      3. at least `gap_s` (default 0.25 s, i.e. a ceiling of 4 queries/s) since
         the previous poll.
    Among the queries that are due, the most overdue (by its own period) goes
    first, so under load every query slows down proportionally rather than a
    low-priority one starving: balance every `balance_s` (1.0 s), `imu` every
    `imu_s` (2.0 s), and each of the 12 servos every `joint_s` (6.0 s). That
    demands ~3.5 queries/s of the 4/s ceiling; ~1 kB/s of the 11.5 kB/s link.
    `imu raw 1` (one accel+gyro sample) is polled only with raw_s > 0.
    """

    def __init__(self, joint_ids=JOINT_IDS, balance_s=1.0, imu_s=2.0,
                 joint_s=6.0, raw_s=0.0, gap_s=0.25, quiet_s=1.0,
                 settle_s=0.12, reply_timeout_s=0.6):
        self.gap_s, self.quiet_s = gap_s, quiet_s
        self.settle_s, self.reply_timeout_s = settle_s, reply_timeout_s
        self.items = [("balance", balance_s), ("imu", imu_s)]
        if raw_s > 0:
            self.items.append(("imu raw 1", raw_s))
        self.items += [(f"pos {i}", joint_s) for i in joint_ids]
        self.last = {c: None for c, _ in self.items}
        self.last_poll = None
        self.in_flight_since = None

    def due(self, now):
        """The most overdue query (by its own period), if any is due."""
        best, best_over = None, 0.0
        for cmd, period in self.items:
            t = self.last[cmd]
            over = 1e9 if t is None else (now - t) / period
            if over >= 1.0 and over > best_over:
                best, best_over = cmd, over
        return best

    def next(self, now, last_operator_tx, last_rx, operator_waiting):
        """The query to send now, or None. `last_*` are clock readings (or
        None for never)."""
        if operator_waiting:
            return None
        if last_operator_tx is not None and now - last_operator_tx < self.quiet_s:
            return None
        if self.in_flight_since is not None:
            waited = now - self.in_flight_since
            silent = last_rx is None or now - last_rx >= self.settle_s
            if not ((silent and waited >= self.settle_s)
                    or waited >= self.reply_timeout_s):
                return None
            self.in_flight_since = None
        if self.last_poll is not None and now - self.last_poll < self.gap_s:
            return None
        cmd = self.due(now)
        if cmd is None:
            return None
        self.last[cmd] = now
        self.last_poll = now
        self.in_flight_since = now
        return cmd


# -- the beacon relay ----------------------------------------------------------
class BeaconRelay:
    """Optionally re-send the robot's Wi-Fi beacons, byte for byte.

    The robot beacons only to the console that commanded it last. If that
    console is on mira (link/commander.py, a bimo_gui), point ITS --watch at
    this relay's UDP port and every beacon is fanned out unchanged to the
    --watch addresses here -- e.g. the laptop's `bimo_gui --readonly` listening
    on 192.168.2.105:4211, which decodes them with the same code as the
    driver. Datagrams are forwarded as received (no parsing, no re-framing:
    the receiver's own decoder is the arbiter), and never touch the serial
    port. Off unless --beacon-listen is given.
    """

    def __init__(self, bind, port, targets):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.port = self.sock.getsockname()[1]
        self.targets = list(targets)
        self.forwarded = 0
        self.received = 0
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.2)
                if not r:
                    continue
                data, _ = self.sock.recvfrom(2048)
            except (OSError, ValueError):
                return
            self.received += 1
            for t in self.targets:
                try:
                    self.sock.sendto(data, t)
                    self.forwarded += 1
                except OSError:
                    pass

    def close(self):
        self._stop = True
        self.sock.close()


def parse_hostport(spec, default_port=4211):
    host, _, port = spec.partition(":")
    if not host:
        raise ValueError(f"bad address {spec!r}")
    return host, int(port) if port else default_port


# -- the bridge ------------------------------------------------------------------
def stamp(t=None):
    dt = datetime.fromtimestamp(time.time() if t is None else t, timezone.utc)
    return dt.strftime("%H:%M:%S.%f")[:-3] + "Z"


class Bridge:
    """The one owner of the port. `port` needs read(n) -> bytes (may time out
    and return b"") and write(bytes); pyserial's Serial and the pty-backed
    FdPort below both do."""

    def __init__(self, port, log, publisher=None, poller=None,
                 clock=time.time):
        self.port, self.log, self.pub, self.poller = port, log, publisher, poller
        self.clock = clock
        self.cmds = queue.Queue()        # operator lines, in order
        self.log_lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.last_operator_tx = None
        self.last_rx = None
        self.rx_lines = 0
        self.polls_sent = 0
        self.stop = threading.Event()
        self.cal = {}                    # id -> cal sample, from `cal show`
        self.note("port opened")

    # -- logging
    def _write_log(self, mark, text):
        with self.log_lock:
            self.log.write(f"{stamp(self.clock())} {mark} {text}\n")
            self.log.flush()

    def note(self, text):
        self._write_log("#", text)

    # -- rx
    def rx_loop(self):
        buf = b""
        while not self.stop.is_set():
            b = self.port.read(512)
            if not b:
                continue
            self.last_rx = self.clock()
            buf += b
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.on_line(line.decode(errors="replace").rstrip("\r"))

    def on_line(self, line):
        self.rx_lines += 1
        self._write_log(" ", line)
        if self.pub is None:
            return
        for s in parse_line(line):
            if s["k"] in ("joint", "cal"):
                if s["k"] == "joint":
                    s["name"] = JOINT_NAMES.get(s["id"], f"id{s['id']}")
            if s["k"] == "cal":
                self.cal[s["id"]] = s
            try:
                self.pub.publish(s)
            except ValueError:
                pass

    # -- tx
    def send_line(self, line, poll=False):
        with self.tx_lock:
            self._write_log(">~" if poll else ">", line)
            self.port.write((line + "\n").encode())
        if poll:
            self.polls_sent += 1
        else:
            self.last_operator_tx = self.clock()

    def tx_loop(self):
        """Operator lines first, always; a poll only when the Poller allows."""
        while not self.stop.is_set():
            try:
                line = self.cmds.get(timeout=0.05)
            except queue.Empty:
                line = None
            if line is not None:
                if line == "__quit__":
                    self.close()
                    return
                self.send_line(line)
                continue
            if self.poller is not None:
                cmd = self.poller.next(self.clock(), self.last_operator_tx,
                                       self.last_rx, not self.cmds.empty())
                if cmd is not None:
                    assert cmd in POLL_COMMANDS, cmd   # belt and braces
                    self.send_line(cmd, poll=True)

    # -- operator input
    def feed_fifo(self, path):
        if not os.path.exists(path):
            os.mkfifo(path)
        while not self.stop.is_set():
            with open(path) as f:       # blocks until a writer opens it
                for line in f:
                    self.cmds.put(line.rstrip("\n"))

    def feed_stream(self, f):
        for line in f:
            self.cmds.put(line.rstrip("\n"))

    def heartbeat_loop(self):
        while not self.stop.wait(HEARTBEAT_S):
            now = self.clock()
            self.pub.publish({
                "k": "hb", "polling": self.poller is not None,
                "rx_lines": self.rx_lines, "polls": self.polls_sent,
                "rx_age": None if self.last_rx is None
                else round(now - self.last_rx, 2),
                "subs": self.pub.n_subscribers()}, cache=False)

    def start(self):
        threading.Thread(target=self.rx_loop, daemon=True).start()
        threading.Thread(target=self.tx_loop, daemon=True).start()
        if self.pub is not None:
            threading.Thread(target=self.heartbeat_loop, daemon=True).start()

    def close(self):
        time.sleep(0.5)                 # let the board finish its reply
        self.stop.set()
        self.note("port closed")
        try:
            self.port.close()
        except Exception:
            pass


class FdPort:
    """A file descriptor as a port: how the tests stand in for the board (a
    pty slave). Never used on a real serial device."""

    def __init__(self, fd):
        self.fd = fd

    def read(self, n):
        r, _, _ = select.select([self.fd], [], [], 0.05)
        if not r:
            return b""
        try:
            return os.read(self.fd, n)
        except OSError:
            return b""

    def write(self, b):
        os.write(self.fd, b)

    def close(self):
        pass


def seed_cal_from_log(path):
    """`cal show` samples from an old serial log: the zero each joint was
    calibrated to, for consoles to compare positions against."""
    out = {}
    with open(path, errors="replace") as f:
        for line in f:
            for s in parse_line(line):
                if s["k"] == "cal":
                    out[s["id"]] = s
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("fifo", help="commands in; '-' for none (use --stdin)")
    ap.add_argument("log", help="UTC-stamped transcript, appended")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="serial device (opened ONCE; opening resets the board)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--stdin", action="store_true",
                    help="also read commands from stdin")
    ap.add_argument("--telemetry-port", type=int, default=8650,
                    help="UDP port consoles subscribe to; 0 = no telemetry")
    ap.add_argument("--bind", default="100.105.166.4",
                    help="address for telemetry (mira's Tailscale address; "
                         "never 0.0.0.0 -- the LAN is not the audience)")
    ap.add_argument("--cal-from", metavar="LOG",
                    help="seed per-joint zeros from a serial log's `cal show`")
    ap.add_argument("--beacon-listen", type=int, metavar="PORT",
                    help="UDP port to receive the robot's Wi-Fi beacons on "
                         "(the --watch target of the console that drives it)")
    ap.add_argument("--beacon-bind", default="0.0.0.0")
    ap.add_argument("--watch", action="append", default=[], metavar="HOST[:PORT]",
                    help="forward each beacon, byte for byte, here (repeatable; "
                         "default port 4211)")
    ap.add_argument("--poll", action="store_true",
                    help="ALSO query pos/balance/imu at a low rate (read-only; "
                         "see Poller for the interleave rule)")
    ap.add_argument("--poll-gap", type=float, default=0.25)
    ap.add_argument("--poll-balance", type=float, default=1.0)
    ap.add_argument("--poll-imu", type=float, default=2.0)
    ap.add_argument("--poll-joint", type=float, default=6.0)
    ap.add_argument("--poll-raw", type=float, default=0.0,
                    help="period of `imu raw 1`; 0 = never")
    a = ap.parse_args(argv)

    import serial                       # late: tests do not need pyserial
    port = serial.Serial(a.port, a.baud, timeout=0.05)
    log = open(a.log, "a", buffering=1)
    pub = Publisher(a.bind, a.telemetry_port) if a.telemetry_port else None
    poller = Poller(balance_s=a.poll_balance, imu_s=a.poll_imu,
                    joint_s=a.poll_joint, raw_s=a.poll_raw,
                    gap_s=a.poll_gap) if a.poll else None
    br = Bridge(port, log, pub, poller)
    if a.beacon_listen:
        relay = BeaconRelay(a.beacon_bind, a.beacon_listen,
                            [parse_hostport(w) for w in a.watch])
        br.note(f"beacon relay udp :{relay.port} -> {a.watch}")
    if a.cal_from:
        br.cal = seed_cal_from_log(a.cal_from)
        if pub:
            for s in br.cal.values():
                pub.publish(s)
    if pub:
        br.note(f"telemetry on udp {a.bind}:{pub.port}; polling "
                f"{'ON' if poller else 'off'}")
    br.start()
    if a.fifo != "-":
        threading.Thread(target=br.feed_fifo, args=(a.fifo,),
                         daemon=True).start()
    if a.stdin:
        threading.Thread(target=br.feed_stream, args=(sys.stdin,),
                         daemon=True).start()
    try:
        while not br.stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        br.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
