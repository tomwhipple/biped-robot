"""The authenticated Wi-Fi CLI: wire format and the client session.

docs/wifi-cli.md is the manual and the threat model. This module is the
REFERENCE for the wire format: firmware/components/wcli conforms to it byte
for byte, and tools/gen_wcli_vectors.py freezes this encoder's output into
firmware/host/vectors/wcli_vectors.h for the C++ host test.

Every datagram (little-endian):

    'B' 'W' ver type | sid u64 | seq u32 | aux u32 | flags u16 | len u16 |
    payload[len] | tag[16]

tag = HMAC-SHA256(direction key, everything before it)[:16]. The direction
keys are derived from the 32-byte shared key, which is provisioned into the
robot's NVS over the USB tether (`wcli key <hex>`) and never crosses the air.

The session (the robot side is firmware/components/wcli/wcli.cpp):

    client                         robot
    HELLO  sid=client nonce   ->
                              <-   CHALLENGE sid=S (fresh random), payload=nonce
    POLL   sid=S              ->   (first authentic use of S makes it THE
                              <-   STATUS                      session)
    CMD    sid=S seq=1 line   ->   accepted once; a retransmit of seq 1 is
                              <-   STATUS (queued)             answered, never
                              <-   OUT seq=1.. aux=1 ... DONE  run twice
    POLL   aux=next out seq   ->   ACK (+ flag RESEND: re-send from aux)

Pure Python, stdlib only (hmac, hashlib, socket): it runs on mira and in the
pytest gate with no extra packages.
"""
from __future__ import annotations

import hashlib
import hmac
import os
import select
import socket
import stat
import struct
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

# -- wire --------------------------------------------------------------------
MAGIC = b"BW"
VERSION = 1
PORT = 4213            # robot listens; it answers the request's source port
KEY_LEN = 32
TAG_LEN = 16
HEADER = struct.Struct("<2sBBQIIHH")
HEADER_LEN = HEADER.size           # 24
OVERHEAD = HEADER_LEN + TAG_LEN    # 40
MAX_LINE = 192                     # linkproto::kMaxCliLine
MAX_CHUNK = 400

T_HELLO, T_CMD, T_CANCEL, T_POLL = 0x01, 0x02, 0x03, 0x04
T_CHALLENGE, T_OUT, T_STATUS, T_STALE = 0x81, 0x82, 0x83, 0x84
CLIENT_TYPES = (T_HELLO, T_CMD, T_CANCEL, T_POLL)
ROBOT_TYPES = (T_CHALLENGE, T_OUT, T_STATUS, T_STALE)

OUT_DONE, OUT_ASYNC, OUT_REFUSED = 1, 2, 4
POLL_RESEND = 1

ST_NONE, ST_QUEUED, ST_RUNNING, ST_DONE, ST_CANCELLED = 0, 1, 2, 3, 4
ST_STATE_MASK = 0x00FF
ST_BUSY = 0x0100
ST_LAGGING = 0x0200

LABEL_C2R = b"bimo-wcli c2r v1"
LABEL_R2C = b"bimo-wcli r2c v1"

DEFAULT_HOST = os.environ.get("BIMO_HOST", "192.168.2.90")
DEFAULT_KEY_FILE = os.environ.get(
    "BIMO_WCLI_KEY", os.path.expanduser("~/.config/bimo/wcli.key"))


class WireError(ValueError):
    """A datagram that is malformed or fails its tag."""


@dataclass
class Msg:
    type: int
    sid: int = 0
    seq: int = 0
    aux: int = 0
    flags: int = 0
    payload: bytes = b""


@dataclass
class Keys:
    c2r: bytes
    r2c: bytes


def derive_keys(key: bytes) -> Keys:
    if len(key) != KEY_LEN:
        raise ValueError(f"key must be {KEY_LEN} bytes, got {len(key)}")
    return Keys(hmac.new(key, LABEL_C2R, hashlib.sha256).digest(),
                hmac.new(key, LABEL_R2C, hashlib.sha256).digest())


def encode(m: Msg, mac_key: bytes) -> bytes:
    if len(m.payload) > MAX_CHUNK:
        raise ValueError(f"payload {len(m.payload)} B > {MAX_CHUNK}")
    head = HEADER.pack(MAGIC, VERSION, m.type, m.sid & (2**64 - 1),
                       m.seq & 0xFFFFFFFF, m.aux & 0xFFFFFFFF,
                       m.flags & 0xFFFF, len(m.payload))
    body = head + m.payload
    return body + hmac.new(mac_key, body, hashlib.sha256).digest()[:TAG_LEN]


def decode(buf: bytes, mac_key: bytes) -> Msg:
    if len(buf) < OVERHEAD:
        raise WireError("short")
    magic, ver, typ, sid, seq, aux, flags, n = HEADER.unpack_from(buf)
    if magic != MAGIC:
        raise WireError("magic")
    if ver != VERSION:
        raise WireError(f"version {ver} (want {VERSION})")
    if n > MAX_CHUNK or len(buf) != OVERHEAD + n:
        raise WireError("length")
    want = hmac.new(mac_key, buf[:HEADER_LEN + n], hashlib.sha256).digest()
    if not hmac.compare_digest(want[:TAG_LEN], buf[HEADER_LEN + n:]):
        raise WireError("tag")
    return Msg(typ, sid, seq, aux, flags, bytes(buf[HEADER_LEN:HEADER_LEN + n]))


# -- the key file --------------------------------------------------------------
def parse_key_hex(text: str) -> bytes:
    h = "".join(text.split()).lower()
    if len(h) != 2 * KEY_LEN or any(c not in "0123456789abcdef" for c in h):
        raise ValueError(f"a Wi-Fi CLI key is {2 * KEY_LEN} hex digits")
    return bytes.fromhex(h)


def load_key(path: str = DEFAULT_KEY_FILE) -> bytes:
    """The shared key, from a file only its owner can read (ssh's rule)."""
    st = os.stat(path)
    if st.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise PermissionError(
            f"{path} is readable by others (mode {stat.S_IMODE(st.st_mode):o}):"
            f" chmod 600 {path}")
    with open(path) as f:
        return parse_key_hex(f.read())


def new_key_file(path: str = DEFAULT_KEY_FILE) -> bytes:
    """Write a fresh random key, mode 0600, refusing to overwrite one."""
    os.makedirs(os.path.dirname(path) or ".", mode=0o700, exist_ok=True)
    key = os.urandom(KEY_LEN)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key.hex() + "\n")
    return key


def key_fingerprint(key: bytes) -> str:
    """What the robot's `wcli` prints: SHA-256(key)[:4], hex. Not the key."""
    return hashlib.sha256(key).digest()[:4].hex()


# -- the client ----------------------------------------------------------------
class LinkDown(RuntimeError):
    """The robot did not answer (no session could be made / kept)."""


@dataclass
class Result:
    """How one command ended.

    status: "done"      executed (or refused -- see .refused); output complete
            "refused"   the robot refused it (text says why); never executed
            "cancelled" the robot confirmed it was NOT executed
            "busy"      the robot would not take it (one in flight / queue
                        full) and confirmed it was not executed
            "unknown"   the session was lost after sending: it may have run
            "timeout"   accepted, but no output/progress for idle_timeout
    """
    seq: int
    status: str
    text: str = ""
    refused: bool = False
    lost_chunks: int = 0


@dataclass
class _Chunk:
    seq: int
    cmd: int
    flags: int
    text: str


class Client:
    """One paired session with the robot's Wi-Fi CLI.

    Output text is delivered strictly in the robot's order through `on_text`
    (command and async alike -- the same stream the tether would show), as
    raw text with the CLI's CRLF line ends. Never resends a command into a
    new session: if the session is lost while a command is outstanding the
    outcome is reported "unknown" rather than risking a second execution.
    """

    RETRY_S = 0.25        # CMD / CANCEL retransmit period
    POLL_S = 0.1          # POLL (ACK + "where are you") while a command runs
    KEEPALIVE_S = 2.0     # POLL while idle (the robot keeps async output for 30 s)
    RESEND_AFTER_S = 0.15  # a known gap this old is re-requested

    def __init__(self, host: str, key: bytes, port: int = PORT,
                 on_text: Optional[Callable[[str], None]] = None,
                 on_note: Optional[Callable[[str], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.addr = (socket.gethostbyname(host), port)
        self.keys = derive_keys(key)
        self.on_text = on_text or (lambda s: None)
        self.on_note = on_note or (lambda s: None)
        self.clock = clock
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)
        self._reset_session()
        self.last_heard = 0.0
        self.lost_total = 0

    # -- plumbing ------------------------------------------------------------
    def _reset_session(self):
        self.sid = 0
        self.seq = 0
        self.out_next = 1
        self.buffer: dict[int, _Chunk] = {}
        self.robot_out_next = 1
        self.status: Optional[Msg] = None
        self.stale = False
        self.gap_since: Optional[float] = None
        self.last_resend = 0.0
        self.last_poll = 0.0
        self.delivered_since_ack = 0
        self.cur = 0                 # the command whose output we collect
        self.cur_text: list[str] = []
        self.cur_done: Optional[_Chunk] = None
        self.cur_progress = 0.0

    @property
    def connected(self) -> bool:
        return self.sid != 0 and not self.stale

    def close(self):
        self.sock.close()

    def _send(self, typ, seq=0, aux=0, flags=0, payload=b"", sid=None):
        m = Msg(typ, self.sid if sid is None else sid, seq, aux, flags, payload)
        try:
            self.sock.sendto(encode(m, self.keys.c2r), self.addr)
        except OSError as e:          # no route / net down: the retry loop copes
            self.on_note(f"send failed: {e}")

    def _poll(self, resend=False):
        self.last_poll = self.clock()
        self.delivered_since_ack = 0
        self._send(T_POLL, aux=self.out_next,
                   flags=POLL_RESEND if resend else 0)

    def _recv(self, timeout: float) -> Optional[Msg]:
        """One authenticated robot datagram for this session, or None."""
        end = self.clock() + max(0.0, timeout)
        while True:
            left = end - self.clock()
            r, _, _ = select.select([self.sock], [], [], max(0.0, left))
            if not r:
                return None
            try:
                buf, src = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError:
                # ICMP port unreachable surfaces here on Linux: robot not up.
                continue
            if src[0] != self.addr[0]:
                continue
            try:
                m = decode(buf, self.keys.r2c)
            except WireError:
                continue            # forged, corrupt, or not for us: drop
            if m.type not in ROBOT_TYPES:
                continue
            return m

    def _handle(self, m: Msg) -> None:
        """Session bookkeeping for one robot datagram."""
        if m.type == T_CHALLENGE or m.sid != self.sid or self.sid == 0:
            return
        self.last_heard = self.clock()
        if m.type == T_STALE:
            self.stale = True
            return
        if m.type == T_STATUS:
            self.status = m
            if m.flags & ST_BUSY and m.seq == 0 and m.aux == 0 and not m.payload:
                return              # "busy, not paired yet" -- connect() retries
            self.robot_out_next = max(self.robot_out_next, m.seq)
            oldest = struct.unpack("<I", m.payload[:4])[0] if len(m.payload) >= 4 else 1
            if oldest > self.out_next:
                lost = oldest - self.out_next
                self._lost(lost)
                self.out_next = oldest
                self._deliver()
            self._note_gap()
            return
        if m.type == T_OUT:
            if m.seq < self.out_next or m.seq in self.buffer:
                return              # a duplicate (our resend crossed it)
            self.buffer[m.seq] = _Chunk(m.seq, m.aux, m.flags,
                                        m.payload.decode("ascii", "replace"))
            self.robot_out_next = max(self.robot_out_next, m.seq + 1)
            self._deliver()
            self._note_gap()

    def _lost(self, n: int):
        self.lost_total += n
        txt = f"[wcli: {n} chunk(s) of robot output lost]\r\n"
        self.on_note(txt.strip())
        self.on_text(txt)

    def _deliver(self):
        while self.out_next in self.buffer:
            c = self.buffer.pop(self.out_next)
            self.out_next += 1
            self.delivered_since_ack += 1
            if c.text:
                self.on_text(c.text)
            if self.cur and c.cmd == self.cur and not (c.flags & OUT_ASYNC):
                self.cur_text.append(c.text)
                self.cur_progress = self.clock()
                if c.flags & OUT_DONE:
                    self.cur_done = c

    def _note_gap(self):
        if self.out_next < self.robot_out_next:
            if self.gap_since is None:
                self.gap_since = self.clock()
        else:
            self.gap_since = None

    def _service(self, timeout: float):
        """Receive for up to `timeout`, then ACK / re-request as needed."""
        m = self._recv(timeout)
        while m is not None:
            self._handle(m)
            m = self._recv(0.0)
        now = self.clock()
        if (self.gap_since is not None and now - self.gap_since > self.RESEND_AFTER_S
                and now - self.last_resend > self.RESEND_AFTER_S):
            self.last_resend = now
            self._poll(resend=True)
        elif self.delivered_since_ack >= 8 or (
                self.delivered_since_ack and now - self.last_poll > 0.05):
            self._poll()

    # -- public ---------------------------------------------------------------
    def connect(self, timeout: float = 3.0) -> None:
        """HELLO -> CHALLENGE -> POLL -> STATUS: a fresh session."""
        self._reset_session()
        end = self.clock() + timeout
        while self.clock() < end:
            nonce = int.from_bytes(os.urandom(8), "little") or 1
            self._send(T_HELLO, sid=nonce)
            t_hello = self.clock()
            sid = 0
            while self.clock() - t_hello < self.RETRY_S * 2 and self.clock() < end:
                m = self._recv(0.05)
                if (m is not None and m.type == T_CHALLENGE
                        and m.payload == struct.pack("<Q", nonce)):
                    sid = m.sid
                    break
            if not sid:
                continue
            self.sid = sid
            t_poll = 0.0
            while self.clock() < end:
                if self.clock() - t_poll > self.RETRY_S:
                    t_poll = self.clock()
                    self._poll()
                m = self._recv(0.05)
                if m is None:
                    continue
                self._handle(m)
                if self.stale:
                    break           # challenge expired / superseded: HELLO again
                if (m.type == T_STATUS and m.sid == sid
                        and not (m.flags & ST_BUSY and m.seq == 0)):
                    self.out_next = m.seq       # pre-session output is not ours
                    self.robot_out_next = m.seq
                    self.seq = m.aux
                    return
            self._reset_session()
        self._reset_session()
        raise LinkDown(f"no Wi-Fi CLI session with {self.addr[0]}:{self.addr[1]}"
                       " (robot off the network, no key provisioned, or a "
                       "different key)")

    def idle(self, timeout: float) -> None:
        """Process async output for `timeout` s, keeping the session alive."""
        end = self.clock() + timeout
        while True:
            now = self.clock()
            if self.connected and now - self.last_poll > self.KEEPALIVE_S:
                self._poll()
            left = end - now
            if left <= 0:
                return
            self._service(min(left, 0.1))

    def command(self, line: str, accept_timeout: float = 3.0,
                idle_timeout: float = 30.0) -> Result:
        """Run one CLI line on the robot, at most once. Connects if needed."""
        data = line.encode("ascii", "strict")
        if len(data) > MAX_LINE:
            raise ValueError(f"line longer than {MAX_LINE} bytes")
        if not self.connected:
            self.connect()
        self.seq += 1
        seq = self.seq
        self.cur, self.cur_text, self.cur_done = seq, [], None
        try:
            return self._run(seq, data, accept_timeout, idle_timeout)
        finally:
            self.cur = 0

    def _accepted(self, seq: int) -> Optional[str]:
        """'accepted' / 'cancelled' / 'busy' from the latest STATUS, or None."""
        st = self.status
        if st is None:
            return None
        state = st.flags & ST_STATE_MASK
        if st.aux == seq:
            if state == ST_CANCELLED:
                return "cancelled"
            if state in (ST_QUEUED, ST_RUNNING, ST_DONE):
                return "accepted"
        if st.aux < seq and st.flags & ST_BUSY:
            return "busy"
        return None

    def _run(self, seq, data, accept_timeout, idle_timeout) -> Result:
        lost0 = self.lost_total
        t0 = self.clock()
        self.status = None
        self._send(T_CMD, seq=seq, payload=data)
        last_tx = self.clock()
        busy_seen = False
        # 1. until the robot says it has it (or that it never will)
        while True:
            self._service(0.02)
            if self.stale:
                return Result(seq, "unknown", "".join(self.cur_text))
            verdict = self._accepted(seq)
            if verdict == "accepted" or self.cur_done is not None:
                break               # (its DONE can beat a lost STATUS here)
            if verdict == "busy":
                busy_seen = True
                self.status = None
            if self.clock() - t0 > accept_timeout:
                return self._give_up(seq, busy_seen)
            if self.clock() - last_tx > self.RETRY_S:
                last_tx = self.clock()
                self._send(T_CMD, seq=seq, payload=data)
        # 2. until its DONE chunk (and everything before it) is delivered
        self.cur_progress = self.clock()
        while self.cur_done is None:
            self._service(0.05)
            if self.cur_done is not None:
                break
            if self.stale:
                return Result(seq, "unknown", "".join(self.cur_text))
            now = self.clock()
            if now - self.last_poll > self.POLL_S:
                # Keepalive + "where are you": a DONE chunk lost on the tail
                # shows up as robot_out_next > out_next and is re-requested.
                self._poll(resend=self.gap_since is not None)
            # No output from this command for idle_timeout: give up waiting
            # (it may still be running -- it is never re-sent).
            if now - self.cur_progress > idle_timeout:
                return Result(seq, "timeout", "".join(self.cur_text))
        refused = bool(self.cur_done.flags & OUT_REFUSED)
        # Command output done; ACK it now so the robot's window is clear.
        self._poll()
        return Result(seq, "refused" if refused else "done",
                      "".join(self.cur_text), refused,
                      self.lost_total - lost0)

    def _give_up(self, seq, busy_seen) -> Result:
        """No acceptance in time: CANCEL burns the seq and tells us the truth."""
        t0 = self.clock()
        self.status = None
        last_tx = 0.0
        while self.clock() - t0 < 3.0:
            if self.clock() - last_tx > self.RETRY_S:
                last_tx = self.clock()
                self._send(T_CANCEL, seq=seq)
            self._service(0.02)
            if self.stale:
                return Result(seq, "unknown")
            st = self.status
            if st is not None and st.aux >= seq:
                state = st.flags & ST_STATE_MASK
                if st.aux > seq or state == ST_CANCELLED:
                    return Result(seq, "busy" if busy_seen else "cancelled")
                # It got in after all (RUNNING / DONE): collect its output.
                return self._collect_late(seq)
        raise LinkDown(f"robot not answering (command {seq} outcome unknown)")

    def _collect_late(self, seq) -> Result:
        self.cur_progress = self.clock()
        while self.cur_done is None and self.clock() - self.cur_progress < 30.0:
            self._service(0.05)
            if self.stale:
                return Result(seq, "unknown", "".join(self.cur_text))
            if self.clock() - self.last_poll > self.POLL_S:
                self._poll(resend=self.gap_since is not None)
        if self.cur_done is None:
            return Result(seq, "timeout", "".join(self.cur_text))
        refused = bool(self.cur_done.flags & OUT_REFUSED)
        return Result(seq, "refused" if refused else "done",
                      "".join(self.cur_text), refused)


@dataclass
class LineSplitter:
    """Raw CLI text in, complete lines out (CR stripped, like the tether log)."""
    buf: str = ""
    lines: list = field(default_factory=list)

    def feed(self, text: str) -> list:
        self.buf += text
        out = []
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            out.append(line.rstrip("\r").rstrip())
        return out
