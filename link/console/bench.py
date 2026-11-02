"""The bench: serial-bridge telemetry (tools/board_bridge.py, UDP 8650) and the
camera relay's panes (tools/cam_relay.py, HTTP 8647), docs/bench-console.md.

A port of link/bench.{h,cpp} with the same semantics, so the Qt console and
bimo_gui show the same thing from the same datagrams:

  * one FLAT JSON object per datagram (strings, numbers, true/false/null, no
    nesting); anything else is rejected, never guessed at;
  * every time is this machine's clock at ARRIVAL (the caller's now_ms, the
    core's monotonic clock), never the bridge's `t`: a Mac and mira disagree
    by seconds, and a 30 s chart on the wrong clock shows the wrong 30 s;
  * the Wi-Fi beacon is the primary source where it overlaps: a beacon servo
    reading fresher than 2 s outranks the bridge's `joint` sample, and the
    beacon's up vector feeds the IMU, without counting as hearing the bridge;
  * WATCH-ONLY: the client sends {"sub":1} every 2 s (the bridge's lease is
    10 s) and {"unsub":1} on close, and nothing else, ever.

Every datagram the model accepts is handed to a sink verbatim; the console's
sink is the session recorder (Core.rec_bench), as bimo_gui's is.

No Qt here: the headless console uses all of it. The camera panes import
mjpeg.py (QImage) only when one is opened.
"""
from __future__ import annotations

import math
import re
import socket
import struct
from collections import deque
from dataclasses import dataclass, field

from . import core
from .options import BENCH_PORT, CAM_NAMES, CAM_PORT

# JsonObj (bench.h): at most 24 keys, keys kept to 19 bytes, strings to 79.
MAX_KEYS = 24
KEY_CAP = 19
STR_CAP = 79
NUM_CAP = 39
TS_CAP = 1024
MAX_ID = 64                 # kBenchMaxId
WINDOW_MS = 30000.0         # kBenchWindowMs
STALE_MS = 3500.0           # kBenchStaleMs: heartbeats are 1 Hz
BEACON_PREFER_MS = 2000.0   # kBeaconPreferMs
SUB_EVERY_MS = 2000.0
RECV_MAX = 1500
CAM_PANES = 2
CAM_LABELS = {"emeet": "EMEET C960 (floor, feet)", "brio": "Brio 101",
              "sony": "Sony A7 III"}
# bimo_gui's servo table order: right leg, then left leg (by servo id).
SERVO_ORDER = (52, 6, 7, 8, 11, 51, 2, 3, 4, 12, 10, 9)
OLD_S = 15.0                # a reading older than two poll sweeps is history

_HEX = frozenset(b"0123456789abcdefABCDEF")
_NUMC = frozenset(b"-+.eE0123456789")
_WS = frozenset(b" \t\r\n")


def f32(x: float) -> float:
    """bench.cpp keeps its readings as float: the same rounding, so a
    status line prints the same digits bimo_gui does."""
    return struct.unpack("f", struct.pack("f", x))[0]


# -- the flat JSON reader (JsonObj) -----------------------------------------
def _ws(s: bytes, p: int) -> int:
    n = len(s)
    while p < n and s[p] in _WS:
        p += 1
    return p


def _string(s: bytes, p: int, cap: int):
    """A "..." string at p: (text, position after the quote), or None.
    Too long is truncated (harmless for names); a bad escape, a raw control
    character or a missing quote is a refusal."""
    n = len(s)
    if p >= n or s[p] != 0x22:
        return None
    p += 1
    out = bytearray()
    while p < n and s[p] != 0x22:
        c = s[p]
        p += 1
        if c == 0x5C:
            if p >= n:
                return None
            c = s[p]
            p += 1
            if c == 0x6E:
                c = 0x0A
            elif c == 0x74:
                c = 0x09
            elif c in (0x22, 0x5C, 0x2F):
                pass
            elif c == 0x75:                      # \uXXXX: keep ASCII, else '?'
                hx = s[p:p + 4]
                if len(hx) < 4 or not all(h in _HEX for h in hx):
                    return None
                v = int(hx, 16)
                c = v if 0 < v < 128 else 0x3F
                p += 4
            else:
                return None
        elif c < 0x20:
            return None
        if len(out) < cap:
            out.append(c)
    if p >= n:
        return None
    return out.decode("utf-8", "replace"), p + 1


def parse_flat(data: bytes) -> dict | None:
    """One flat JSON object -> {key: float | str | bool | None}, or None.

    The first of a repeated key wins (JsonObj::find). Unlike JsonObj, an
    empty object with trailing bytes is refused too: it changes nothing a
    model accepts, and it is one less way to be lenient."""
    s = bytes(data)
    n = len(s)
    p = _ws(s, 0)
    if p >= n or s[p] != 0x7B:
        return None
    p = _ws(s, p + 1)
    obj: dict = {}
    if p < n and s[p] == 0x7D:
        return obj if _ws(s, p + 1) == n else None
    count = 0
    while True:
        if count >= MAX_KEYS:
            return None
        r = _string(s, _ws(s, p), KEY_CAP)
        if r is None:
            return None
        key, p = r
        p = _ws(s, p)
        if p >= n or s[p] != 0x3A:
            return None
        p = _ws(s, p + 1)
        if p >= n:
            return None
        if s[p] == 0x22:
            r = _string(s, p, STR_CAP)
            if r is None:
                return None
            val, p = r
        elif s.startswith(b"true", p):
            val, p = True, p + 4
        elif s.startswith(b"false", p):
            val, p = False, p + 5
        elif s.startswith(b"null", p):
            val, p = None, p + 4
        else:
            q = p
            while q < n and q - p < NUM_CAP and s[q] in _NUMC:
                q += 1
            if q == p:                           # also '{' and '[': nesting
                return None
            try:
                val = float(s[p:q])
            except ValueError:
                return None
            if not math.isfinite(val):
                return None
            p = q
        obj.setdefault(key, val)
        count += 1
        p = _ws(s, p)
        if p >= n:
            return None
        if s[p] == 0x2C:
            p += 1
            continue
        if s[p] == 0x7D:
            return obj if _ws(s, p + 1) == n else None
        return None


def num(o: dict, key: str) -> float | None:
    """None if absent, null or not a number."""
    v = o.get(key)
    return v if isinstance(v, float) else None


def num_or(o: dict, key: str, dflt: float) -> float:
    v = num(o, key)
    return dflt if v is None else v


def text(o: dict, key: str) -> str | None:
    v = o.get(key)
    return v if isinstance(v, str) else None


def flag(o: dict, key: str, dflt: bool) -> bool:
    v = o.get(key)
    return v if isinstance(v, bool) else dflt


# -- TimeSeries ---------------------------------------------------------------
class TimeSeries:
    """Samples over time, for a fixed-width chart (the newest 1024)."""

    def __init__(self, cap: int = TS_CAP):
        self.t: deque = deque(maxlen=cap)
        self.v: deque = deque(maxlen=cap)

    def push(self, t_ms: float, v: float) -> None:
        self.t.append(t_ms)
        self.v.append(v)

    def count(self) -> int:
        return len(self.t)

    def last(self) -> float:
        return self.v[-1] if self.v else 0.0

    def resample(self, now_ms: float, window_ms: float, bins: int):
        """(first, out): out[b] is the value in force at the END of bin b of
        [now - window, now] (sample-and-hold); bins before `first` hold no
        data (first == bins when nothing falls inside or before the window)."""
        out = [0.0] * max(bins, 0)
        if bins <= 0:
            return 0, out
        t0 = now_ms - window_ms
        bw = window_ms / bins
        i, n = 0, len(self.t)
        have, cur, first = False, 0.0, bins
        for b in range(bins):
            end = t0 + bw * (b + 1)
            while i < n and self.t[i] <= end:
                cur = self.v[i]
                have = True
                i += 1
            if have:
                if first == bins:
                    first = b
                out[b] = cur
        return first, out


# -- the model ------------------------------------------------------------------
@dataclass
class Joint:
    have: bool = False
    id: int = 0
    name: str = ""
    pos: int = 0
    spd: int = 0
    load: int = 0
    temp: int = 0
    err: int = 0
    volt: float = 0.0
    t_ms: float = 0.0
    cur: int = 0                 # 6.5 mA/LSB; beacon only (serial `pos` has none)
    from_beacon: bool = False    # which source supplied this reading
    have_cal: bool = False
    zero: int = 0
    dir: int = 1
    fitted: bool = True


@dataclass
class BenchModel:
    joints: list = field(default_factory=lambda: [Joint() for _ in range(MAX_ID)])
    # balance
    have_roll: bool = False
    roll_na: bool = False
    roll: float = 0.0
    t_roll: float = 0.0
    state: str = ""
    stance: str = ""
    since: float = 0.0
    t_state: float = 0.0
    have_ref: bool = False
    ref: float = 0.0
    err: float = 0.0
    integ: float = 0.0
    u: float = 0.0
    t_ref: float = 0.0
    have_corr: bool = False
    corr_side: str = ""
    ankle_deg: float = 0.0
    hip_deg: float = 0.0
    ankle_ticks: int = 0
    hip_ticks: int = 0
    clamped: int = 0
    have_gains: bool = False
    kp: float = 0.0
    ki: float = 0.0
    clamp: float = 0.0
    abort_deg: float = 0.0
    # imu
    have_up: bool = False
    up: list = field(default_factory=lambda: [0.0, 0.0, 1.0])
    t_up: float = 0.0
    have_level: bool = False
    levelled: bool = False
    streak: int = 0
    have_proj: bool = False
    proj: list = field(default_factory=lambda: [0.0, 0.0, 1.0])
    have_gyro: bool = False
    gyro: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    have_raw: bool = False
    acc: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accmag: float = 0.0
    graw: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    t_raw: float = 0.0
    s_roll: TimeSeries = field(default_factory=TimeSeries)
    s_err: TimeSeries = field(default_factory=TimeSeries)
    s_u: TimeSeries = field(default_factory=TimeSeries)
    # the bridge
    datagrams: int = 0
    rejected: int = 0
    last_rx_ms: float = 0.0      # any datagram
    last_hb_ms: float = 0.0
    hb_polling: bool = False
    hb_rx_age: float = -1.0      # seconds the bridge has gone without a board byte
    hb_polls: int = 0

    def apply(self, data: bytes, now_ms: float) -> bool:
        """False for a datagram that is not a valid v1 sample."""
        j = parse_flat(data)
        k = text(j, "k") if j is not None else None
        if j is None or num_or(j, "v", 0.0) != 1.0 or k is None:
            self.rejected += 1
            return False
        if k in ("joint", "cal"):
            jid = int(num_or(j, "id", -1.0))
            if not 0 <= jid < MAX_ID:
                # Not a sample after all: it must not count as hearing the bridge.
                self.rejected += 1
                return False
        self.datagrams += 1
        self.last_rx_ms = now_ms
        x3 = lambda: [f32(num_or(j, a, 0.0)) for a in ("x", "y", "z")]  # noqa: E731
        if k == "hb":
            self.last_hb_ms = now_ms
            self.hb_polling = flag(j, "polling", False)
            self.hb_rx_age = num_or(j, "rx_age", -1.0)
            self.hb_polls = int(num_or(j, "polls", 0.0))
        elif k in ("joint", "cal"):
            b = self.joints[jid]
            b.id = jid
            if k == "joint" and b.from_beacon and now_ms - b.t_ms < BEACON_PREFER_MS:
                return True          # heard (counts for live()), but outranked
            nm = text(j, "name")
            if nm:
                b.name = nm[:23]
            if k == "joint":
                b.have = True
                b.from_beacon = False
                b.pos = int(num_or(j, "pos", 0.0))
                b.spd = int(num_or(j, "spd", 0.0))
                b.load = int(num_or(j, "load", 0.0))
                b.temp = int(num_or(j, "temp", 0.0))
                b.err = int(num_or(j, "err", 0.0))
                b.volt = f32(num_or(j, "volt", 0.0))
                b.t_ms = now_ms
            else:
                b.have_cal = True
                b.zero = int(num_or(j, "zero", 0.0))
                b.dir = int(num_or(j, "dir", 1.0))
                b.fitted = flag(j, "fitted", True)
        elif k == "bal_roll":
            v = num(j, "roll")
            self.roll_na = v is None
            if v is not None:
                self.have_roll = True
                self.roll = f32(v)
                self.t_roll = now_ms
                self.s_roll.push(now_ms, self.roll)
        elif k == "bal_state":
            self.state = (text(j, "state") or "")[:63]
            st = text(j, "stance")
            self.stance = st[0] if st else ""
            self.since = f32(num_or(j, "since", 0.0))
            self.t_state = now_ms
        elif k == "bal_ref":
            self.have_ref = True
            self.ref = f32(num_or(j, "ref", 0.0))
            self.err = f32(num_or(j, "err", 0.0))
            self.integ = f32(num_or(j, "I", 0.0))
            self.u = f32(num_or(j, "u", 0.0))
            self.t_ref = now_ms
            self.s_err.push(now_ms, self.err)
            self.s_u.push(now_ms, self.u)
        elif k == "bal_corr":
            self.have_corr = True
            sd = text(j, "side")
            self.corr_side = sd[0] if sd else ""
            self.ankle_deg = f32(num_or(j, "ankle_deg", 0.0))
            self.hip_deg = f32(num_or(j, "hip_deg", 0.0))
            self.ankle_ticks = int(num_or(j, "ankle_ticks", 0.0))
            self.hip_ticks = int(num_or(j, "hip_ticks", 0.0))
            self.clamped = int(num_or(j, "clamped", 0.0))
        elif k == "bal_gains":
            self.have_gains = True
            self.kp = f32(num_or(j, "kp", 0.0))
            self.ki = f32(num_or(j, "ki", 0.0))
            self.clamp = f32(num_or(j, "clamp", 0.0))
            self.abort_deg = f32(num_or(j, "abort", 0.0))
        elif k == "imu_up":
            self.have_up = True
            self.up = x3()
            self.t_up = now_ms
        elif k == "imu_proj":
            self.have_proj = True
            self.proj = x3()
        elif k == "imu_gyro":
            self.have_gyro = True
            self.gyro = x3()
        elif k == "imu_level":
            self.have_level = True
            self.levelled = flag(j, "levelled", False)
            self.streak = int(num_or(j, "streak", 0.0))
        elif k == "imu_raw":
            self.have_raw = True
            self.acc = [f32(num_or(j, a, 0.0)) for a in ("ax", "ay", "az")]
            self.accmag = f32(num_or(j, "amag", 0.0))
            self.graw = [f32(num_or(j, a, 0.0)) for a in ("gx", "gy", "gz")]
            self.t_raw = now_ms
        # Unknown kinds are counted as heard (the bridge is alive) and
        # ignored: a newer bridge may send more than this console knows.
        return True

    def apply_beacon_up(self, x: float, y: float, z: float, now_ms: float) -> None:
        """The beacon's attitude (body-frame up, the bridge's convention).
        Does NOT count as hearing the bridge (live())."""
        self.have_up = True
        self.up = [f32(x), f32(y), f32(z)]
        self.t_up = now_ms

    def apply_beacon_servo(self, sid: int, pos: int, spd: int, load: int, cur: int,
                           volt_dv: int, temp: int, status: int, age_ms: int,
                           now_ms: float) -> None:
        """One entry of the beacon's SERVO block (bench.h applyBeaconServo).
        While it is fresher than BEACON_PREFER_MS a bridge `joint` sample for
        the same servo is ignored; age_ms 255 = no reading / unfitted."""
        if not 0 <= sid < MAX_ID or age_ms >= 255:
            return
        b = self.joints[sid]
        b.id = sid
        b.have = True
        b.from_beacon = True
        b.pos, b.spd, b.load, b.cur = pos, spd, load, cur
        b.volt = f32(volt_dv / 10.0)
        b.temp = temp
        b.err = status
        b.t_ms = now_ms - age_ms

    def joints_seen(self) -> int:
        return sum(1 for b in self.joints if b.have)

    def live(self, now_ms: float) -> bool:
        """The bridge: heard from lately?"""
        return self.datagrams > 0 and now_ms - self.last_rx_ms < STALE_MS


# -- the socket -------------------------------------------------------------------
_PORT = re.compile(r"[0-9]+\Z")


class BenchLink:
    """The subscriber. open() resolves once; poll() re-subscribes every 2 s
    and drains whatever has arrived, without blocking."""

    SUB = b'{"sub":1}'
    UNSUB = b'{"unsub":1}'

    def __init__(self):
        self.model = BenchModel()
        self.sink = None              # fn(datagram bytes, now_ms), per ACCEPTED datagram
        self.err = ""
        self.target = ""
        self._sock = None
        self._addr = None
        self._next_sub = 0.0

    @property
    def active(self) -> bool:
        return self._sock is not None

    def open(self, spec: str, default_port: int = BENCH_PORT) -> bool:
        """"host[:port]" (host may be a name). False on failure, err says why."""
        self.close()
        host, port = spec, default_port
        if ":" in spec:
            host, _, ps = spec.rpartition(":")
            if not _PORT.match(ps) or not 1 <= int(ps) <= 65535:
                self.err = f"bad port in `{spec}`"
                return False
            port = int(ps)
        if not host:
            self.err = f"expected HOST[:PORT], got `{spec}`"
            return False
        try:
            info = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_DGRAM)
        except (socket.gaierror, UnicodeError) as e:
            self.err = f"cannot resolve `{host}`: {getattr(e, 'strerror', None) or e}"
            return False
        if not info:
            self.err = f"cannot resolve `{host}`"
            return False
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as e:
            self.err = f"socket: {e.strerror or e}"
            return False
        s.setblocking(False)
        self._sock, self._addr = s, info[0][4]
        self.target = f"{host}:{port}"
        self.err = ""
        self._next_sub = 0.0
        self.model = BenchModel()
        return True

    def close(self) -> None:
        if self._sock is not None:
            self._send(self.UNSUB)
            self._sock.close()
            self._sock = None

    def _send(self, msg: bytes) -> None:
        try:
            self._sock.sendto(msg, self._addr)
        except OSError:
            pass                      # a lost subscribe is retried in 2 s

    def poll(self, now_ms: float) -> None:
        if self._sock is None:
            return
        if now_ms >= self._next_sub:
            # The lease is 10 s; 2 s keeps it through four lost datagrams.
            self._send(self.SUB)
            self._next_sub = now_ms + SUB_EVERY_MS
        for _ in range(256):
            try:
                data = self._sock.recv(RECV_MAX)
            except OSError:           # BlockingIOError: drained
                break
            if not data:
                break
            if self.model.apply(data, now_ms) and self.sink is not None:
                self.sink(data, now_ms)

    def state_name(self, now_ms: float) -> str:
        """bimo_gui's benchStateName: off / waiting / live / STALE."""
        if not self.active:
            return "off"
        if self.model.datagrams == 0:
            return "waiting"
        return "live" if self.model.live(now_ms) else "STALE"


# -- the console's bench: the link, the panes, the recording, the log -------------
def cam_host(spec: str) -> str:
    """--cam-relay HOST or HOST:PORT -> always host:port, for the URLs."""
    return spec if ":" in spec else f"{spec}:{CAM_PORT}"


class CamPane:
    """One camera pane: which relay source it watches (None = off), and the
    stream. Selecting is what opens a camera on mira; off lets it go."""

    def __init__(self, index: int):
        self.index = index
        self.sel: str | None = None
        self.url = ""
        self.stream = None            # mjpeg.MjpegStream
        self.last_err = ""

    def state(self) -> str:
        """off / waiting / live / STALE (mjpeg.py: no frame for 2 s)."""
        if self.sel is None or self.stream is None:
            return "off"
        return self.stream.state()

    def frames(self) -> int:
        return self.stream.frames if self.stream is not None else 0

    def age_s(self) -> float | None:
        return self.stream.age_s() if self.stream is not None else None

    def error(self) -> str:
        return self.stream.error if self.stream is not None else ""


class Bench:
    """What the Bench tab and --headless share: the bridge client, the two
    camera panes, and their recording and event-log lines. It runs on the
    controller's tick (Controller.listeners), so the window and headless poll
    it identically."""

    def __init__(self, ctl, clock=None):
        self.ctl = ctl
        o = ctl.o
        self.clock = clock or core.now_ms
        self.link = BenchLink()
        self.link.sink = self._record
        self.panes = [CamPane(i) for i in range(CAM_PANES)]
        self.cam_host = cam_host(o.cam_relay) if getattr(o, "cam_relay", None) else ""
        self.on_view = None           # the window: fn("bench" | other)
        self._was_live = False
        self._seen = 0                # beacons already fed to the model
        self._rec_for = None          # the take whose cam rows are written
        self._closed = False
        now = self.clock()
        spec = getattr(o, "bench_telemetry", None)
        if spec:
            # Failure is logged, not fatal: a console that cannot reach mira
            # should still say so.
            if self.link.open(spec, BENCH_PORT):
                ctl.log("bench", f"subscribing to {self.link.target}")
            else:
                ctl.log("ERROR", f"--bench-telemetry {spec}: {self.link.err}")
        self._sync_rec(now)
        if self.cam_host:
            ctl.log("cam", f"camera relay http://{self.cam_host}/")
            for i, name in enumerate(getattr(o, "cam", None) or []):
                if name in CAM_NAMES:
                    self.select(i, name)
        ctl.listeners.append(self._tick)

    # -- the tick ------------------------------------------------------------
    def _tick(self, st) -> None:
        now = self.clock()
        # The Wi-Fi beacon feeds the same model where it overlaps the
        # bridge. The core decodes no SERVO block yet (link/client.h), so
        # only the up vector; apply_beacon_servo is ready for it.
        if st.tlm_count != self._seen:
            self._seen = st.tlm_count
            if self.link.active and st.have_att:
                self.link.model.apply_beacon_up(st.up_x, st.up_y, st.up_z, now)
        self.link.poll(now)
        self._sync_rec(now, st)
        if self.link.active:
            live = self.link.model.live(now)
            if live != self._was_live:
                self._was_live = live
                self.ctl.log("bench", f"telemetry bridge {'heard' if live else 'LOST'} "
                             f"({self.link.target})")
        for p in self.panes:
            e = p.error()
            if e != p.last_err:
                p.last_err = e
                if e:
                    self.ctl.log("cam", f"pane {p.index}: {e}")

    def _record(self, data: bytes, now: float) -> None:
        self.ctl.core.rec_bench(now, data)    # a no-op while not recording

    def _sync_rec(self, now: float, st=None) -> bool:
        """A take that began since the last look gets a cam row for every
        pane already pointed somewhere (bimo_gui's toggleRecord). True while
        recording."""
        st = st if st is not None else self.ctl.core.status(now)
        key = self.ctl.core.rec_path if st.rec_active else None
        if key != self._rec_for:
            self._rec_for = key
            if key:
                for p in self.panes:
                    if p.sel is not None:
                        self.ctl.core.rec_cam(now, p.index, p.sel, p.url)
        return key is not None

    # -- the panes --------------------------------------------------------------
    def select(self, pane: int, name: str | None) -> bool:
        """Point a pane at a relay source (None = off). Never shows the
        previous camera: the old stream is dropped first."""
        if not 0 <= pane < CAM_PANES:
            return False
        p = self.panes[pane]
        if p.stream is not None:
            p.stream.stop()
        p.stream, p.sel, p.url, p.last_err = None, None, "", ""
        if name not in CAM_NAMES or not self.cam_host:
            return False
        try:
            from .mjpeg import MjpegStream
        except ImportError as e:
            self.ctl.log("ERROR", f"cam: camera panes need PyQt6 ({e})")
            return False
        now = self.clock()
        recording = self._sync_rec(now)
        url = f"http://{self.cam_host}/{name}"
        s = MjpegStream(url, f"cam{pane} {name}")
        s.start()
        p.stream, p.sel, p.url = s, name, url
        if recording:
            self.ctl.core.rec_cam(now, pane, name, url)
        return True

    # -- the headless line and verbs ---------------------------------------------
    def status_text(self, now: float) -> str:
        """bimo_gui --headless's bench and camera parts of the status line."""
        out = ""
        if self.link.active:
            m = self.link.model
            out += f" | bench {self.link.state_name(now)} joints {m.joints_seen()}"
            if m.have_roll:
                out += f" roll {m.roll:+.2f}"
            if m.have_ref:
                out += f" err {m.err:+.2f} u {m.u:+.2f}"
            if m.state:
                out += f" bal {m.state}"
            if m.have_up:
                out += f" up_z {m.up[2]:.3f}"
            if m.hb_polling:
                out += " polling"
        for p in self.panes:
            if p.sel is not None:
                out += f" | cam{p.index} {p.sel} {p.state()} frames {p.frames()}"
        return out

    def close(self) -> None:
        """Release the cameras and leave the bridge ({"unsub":1})."""
        if self._closed:
            return
        self._closed = True
        for p in self.panes:
            if p.stream is not None:
                p.stream.stop()
        self.link.close()
        if self._tick in self.ctl.listeners:
            self.ctl.listeners.remove(self._tick)


def attach(ctl) -> Bench:
    """The controller's Bench, made once (the window's tab and headless both
    ask)."""
    b = getattr(ctl, "bench", None)
    if b is None:
        b = Bench(ctl)
        ctl.bench = b
    return b


def verb(b: Bench | None, word: str, args: list) -> str | None:
    """The `cam <pane> <name|off>` and `view <bench|drive>` line commands;
    returns a note to print, if any."""
    if word == "view":
        if b is not None and b.on_view is not None:
            b.on_view(args[0] if args else "")
        return None                   # headless has no views
    pane = int(args[0]) if args and args[0].isdigit() else -1
    name = args[1] if len(args) > 1 else None
    if not 0 <= pane < CAM_PANES or (name not in CAM_NAMES and name != "off"):
        return "cam <0|1> emeet|brio|sony|off"
    if b is None or not b.cam_host:
        return "no --cam-relay"
    b.select(pane, None if name == "off" else name)
    return None
