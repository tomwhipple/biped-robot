"""Flight records in the console (design §8): the robot's, forwarded by mira's
collector (tools/flight_collector.py --forward), and the sim twin's
(sim/sil_twin.py --flight), each on its own UDP port.

poll() drains both sockets on the link tick and decodes every datagram with
tools/flightrec.py, the one decoder. Per source ("robot", "sim") it keeps the
latest HELLO (firmware, NTP state, fitted joints, the calibration that turns
steps into radians), rings of every TICK (per bus joint: nominal, goal,
position, speed, load, current, voltage, temperature, status, flags; the IMU;
the 7-wide command; the tick's timing), the EVENT packets, and counters: rate,
sequence holes, robot-side drops, undecodable datagrams, boot-id changes.

Time: samples are spaced by the robot's own boot clock (one SYNC READ per
tick, so every joint shares the stamp) and anchored to this host's
time.monotonic() by the least-delayed packet of the last ~10-20 s. A robot
trace and a sim trace then share one axis, and a stream that stops slides off
the left of a plot instead of standing still as if it were live.

Raw steps are kept; radians come from the latest HELLO's zero and direction
(flightrec.steps_to_rad), so ticks that arrive before the first HELLO become
angles once it does, and an angle is None/NaN until then.

Nothing blocks: the sockets are non-blocking, and poll() returns when both are
drained. Stdlib + numpy; no Qt.
"""
from __future__ import annotations

import collections
import math
import os
import socket
import struct
import sys
import time
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "link"))

import flightrec as F  # noqa: E402  tools/flightrec.py
import protocol  # noqa: E402  link/protocol.py
from flight_collector import SeqTracker  # noqa: E402  the collector's own hole counter

SOURCES = ("robot", "sim")
NJ = F.MAX_JOINTS
BUS = F.BUS_JOINT_NAMES
BUS_INDEX = {n: i for i, n in enumerate(BUS)}
CAP = 8192                 # ticks kept per source: 2.7 min at 50 Hz (Joints' 2 min window)
STALE_S = 3.0              # HELLO is 1 Hz: three silent seconds is "no records"
CMD_STALE_S = 1.0          # policy_cmd() goes None after this
RATE_WINDOW_S = 2.0
OFFSET_WINDOW_S = 10.0
SKEW_WINDOW_S = 10.0
HOLE_BINS_KEEP_S = 300
MAX_PER_POLL = 4000
RCVBUF = 1 << 20
EVENTS_KEEP = 2000
RAD = 2.0 * math.pi / F.STEPS_PER_REV
CMD_NAMES = ("vx", "vy", "wz", "crouch", "lift", "foot_dx", "foot_dz")
LINK_STATES = [s.name for s in protocol.LinkState]   # wire order
# STS3215 status register bits (firmware/components/scsbus/registers.h ErrorBit)
STATUS_BITS = ((1, "voltage"), (2, "sensor"), (4, "temperature"), (8, "current"),
               (16, "angle"), (32, "overload"))
FLAIL_HOLDS = {1: "torque held", 0: "already limp", 2: "autohome started"}

TICK_COLS = ("source", "flags", "link_state", "tick", "work_us", "fb_off_us",
             "fb_read_us", "imu_age_us", "ax", "ay", "az", "gx", "gy", "gz",
             "upx", "upy", "upz", "roll", "pitch", "phase", "vbat_mv", "batt",
             "home_stage", "bal_state", *(f"cmd{i}" for i in range(F.NUM_CMD)))
TC = {n: i for i, n in enumerate(TICK_COLS)}
JOINT_COLS = ("nominal", "goal", "pos", "speed", "load", "current", "voltage_dv",
              "temp_c", "status", "flags", "moving", "goal_speed")
JC = {n: i for i, n in enumerate(JOINT_COLS)}
_FB_COLS = [JC[k] for k in ("pos", "speed", "load", "current", "voltage_dv", "temp_c",
                            "status", "moving")]
# What the sim twin's servos do not have (sim/sil_twin_wire.py joint_block):
# it sends 0 there, so its rows store NaN -- absent, not a reading of 0.
_NO_SIM_COLS = [JC[k] for k in ("load", "current", "voltage_dv", "temp_c", "status",
                                "moving")]
SIM_FW_PREFIX = "sil_twin"     # the twin's HELLO fw: "sil_twin <run>" (sim/sil_twin.py)


def status_text(status: int) -> str:
    """A servo status byte as words ("" = no error bit)."""
    return ", ".join(n for bit, n in STATUS_BITS if status & bit)


def describe_event(p: dict) -> tuple[str, str]:
    """An EVENT packet as (kind, message) for the Session log."""
    code, a, b, text = p["code"], p["a"], p["b"], p["text"]
    kind = F.EVENT_NAMES.get(code, f"ev{code}")
    if code in (F.EV_CMD, F.EV_OUT):
        chan = "wifi" if a == F.CHAN_WIFI else "tether"
        seq = f" #{b}" if code == F.EV_CMD and a == F.CHAN_WIFI else ""
        return kind, f"[{chan}{seq}] {text}"
    if code == F.EV_ARM_EDGE:
        msg = f"arm edge: level {a}"
    elif code == F.EV_HOME_EDGE:
        msg = "home edge"
    elif code == F.EV_MODE:
        try:
            res = protocol.ArmResult(b).name
        except ValueError:
            res = f"result {b} (newer than this console)"
        msg = (f"mode: {'run' if a & 1 else 'bench'}, ctrl "
               f"{'owns' if a & 2 else 'does not own'} the bus, {res}")
    elif code == F.EV_BAL_STATE:
        msg = (f"balance {F.BAL_STATES.get(a, a)} -> {F.BAL_STATES.get(b, b)}")
    elif code == F.EV_LINK_STATE:
        def ls(v):
            return LINK_STATES[v] if 0 <= v < len(LINK_STATES) else str(v)
        msg = f"link {ls(a)} -> {ls(b)}"
    elif code == F.EV_FB_BACKOFF:
        msg = f"feedback read took {a / 1000.0:.1f} ms: backing off"
    elif code == F.EV_FALL:
        msg = (f"fall guard: {F.FALL_EVENTS.get(b, b)} "
               f"({F.FALL_MOTIONS.get(a, a)})")
    elif code == F.EV_FLAIL:
        msg = f"flail guard: {F.FLAIL_TRIPS.get(a, a)}, {FLAIL_HOLDS.get(b, b)}"
    elif code == F.EV_STOP:
        msg = (f"operator stop: {'HARD' if a else 'soft'}, "
               f"{'armed' if b else 'benched'}")
    elif code == F.EV_WCLI:
        ip = ".".join(str((a >> s) & 0xFF) for s in (0, 8, 16, 24))
        msg = f"wifi cli {ip}:{b}"
    else:
        msg = kind
    return kind, f"{msg}: {text}" if text else msg


class _Ring:
    """Ticks oldest..newest. Every row is written twice (slot w and w + cap),
    so the newest n rows are always one contiguous slice: reads are views."""

    def __init__(self, cap: int):
        self.cap = cap
        self.n = 0
        self.w = cap - 1
        self.t = np.zeros(2 * cap)                       # host monotonic s
        self.tick = np.zeros((2 * cap, len(TICK_COLS)))
        self.j = np.full((2 * cap, NJ, len(JOINT_COLS)), np.nan, dtype=np.float32)

    def push(self, t: float, tick_row, jrow) -> None:
        w = (self.w + 1) % self.cap
        for k in (w, w + self.cap):
            self.t[k] = t
            self.tick[k] = tick_row
            self.j[k] = jrow
        self.w = w
        self.n = min(self.n + 1, self.cap)

    def between(self, t0: float, t1: float) -> slice:
        hi = self.w + self.cap + 1
        lo = hi - self.n
        if self.n:
            t = self.t[lo:hi]
            hi = lo + int(np.searchsorted(t, t1, side="right"))
            lo += int(np.searchsorted(t, t0))
        return slice(lo, max(lo, hi))


class _WindowedMin:
    """The minimum of a value over the last one-to-two windows: tracks a slowly
    drifting floor (clock offset, skew) without the all-time minimum's lag."""

    def __init__(self, window_s: float):
        self.window = window_s
        self.cur = self.prev = None
        self.t0 = 0.0

    def add(self, now: float, v: float) -> None:
        if self.cur is None or now - self.t0 > self.window:
            self.prev, self.cur, self.t0 = self.cur, v, now
        elif v < self.cur:
            self.cur = v

    @property
    def value(self):
        if self.cur is None:
            return None
        return self.cur if self.prev is None else min(self.cur, self.prev)


class _Source:
    def __init__(self, name: str, port: int, log):
        self.name = name
        self.port = port
        self.log = log
        self.sock = None
        self.closed = False
        self.error = ""
        self.ring = _Ring(CAP)
        self.events: collections.deque = collections.deque(maxlen=EVENTS_KEEP)
        self.packets = self.ticks = self.n_events = self.hellos = 0
        self.undecodable = 0
        self.last_error = ""
        self.boots = 0                   # boot-id changes seen
        self.boot_id = None
        self.seq = SeqTracker()
        self.holes_before = 0            # holes of earlier boots
        self.drops_before = 0
        self.drop_base = None            # the robot's drop counters at this boot's first packet
        self.drop_now = 0
        self.peer = None
        self.first_rx = self.last_rx = None
        self.last_tick = None            # the decoded newest TICK
        self.last_tick_rx = None
        self.hello = None
        self.hello_rx = None
        self.zero = self.dir = None      # calibration arrays (17,), from HELLO
        self.offset = _WindowedMin(OFFSET_WINDOW_S)
        self.skew = _WindowedMin(SKEW_WINDOW_S)
        self.rx_times: collections.deque = collections.deque()
        self.hole_bins: dict[int, int] = {}
        self.ever_ok = np.zeros(NJ, dtype=bool)
        self.fault_since: list = [None] * NJ   # host time a fitted servo stopped answering
        self.cmd = None
        self.cmd_rx = None
        self.stale_logged = False
        if port:
            self._bind()

    def _bind(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)
        except OSError:
            pass
        try:
            s.bind(("0.0.0.0", self.port))
        except OSError as e:
            s.close()
            self.error = f"cannot bind UDP :{self.port} ({e.strerror or e})"
            self.log("ERROR", f"flight records ({self.name}): {self.error}")
            return
        s.setblocking(False)
        self.sock = s

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None
            self.closed = True

    # -- counters ------------------------------------------------------------
    @property
    def holes(self) -> int:
        return self.holes_before + len(self.seq.missing)

    @property
    def robot_drops(self) -> int:
        return self.drops_before + (self.drop_now - (self.drop_base or 0))

    def rate(self, now: float) -> float:
        while self.rx_times and now - self.rx_times[0] > RATE_WINDOW_S:
            self.rx_times.popleft()
        n = len(self.rx_times)
        if n < 2 or self.stale(now):
            return 0.0
        span = self.rx_times[-1] - self.rx_times[0]
        return (n - 1) / span if span > 0 else 0.0

    def stale(self, now: float) -> bool:
        return self.last_rx is None or now - self.last_rx > STALE_S

    def fitted(self) -> list[str]:
        if self.hello is not None:
            return [BUS[b] for b in range(NJ) if self.hello["fitted"][b]]
        if self.last_tick is not None:
            return [BUS[j["bus"]] for j in self.last_tick["joints"] if j["bus"] < NJ]
        return []

    @property
    def simulated(self) -> bool:
        """These records are the sim twin's: its own port, or a HELLO whose
        fw says sil_twin (a console driving the sim gets them on the robot's
        port, standing in for the robot's)."""
        if self.name == "sim":
            return True
        h = self.hello
        return h is not None and str(h.get("fw", "")).startswith(SIM_FW_PREFIX)

    def rad(self, b: int, steps):
        if self.zero is None:
            return None
        return F.steps_to_rad(steps, int(self.zero[b]), int(self.dir[b]))

    # -- one datagram ----------------------------------------------------------
    def ingest(self, data: bytes, addr, rx: float, wall: float) -> None:
        try:
            p = F.decode(data)
        except (F.DecodeError, struct.error, ValueError) as e:
            self.undecodable += 1
            self.last_error = str(e)
            return
        self.packets += 1
        if self.first_rx is None:
            self.first_rx = rx
            self.log("flight", f"{self.name} records arriving on :{self.port} "
                     f"from {addr[0]}:{addr[1]}")
        elif self.stale_logged:
            self.log("flight", f"{self.name} records resumed after "
                     f"{rx - self.last_rx:.1f} s")
        self.stale_logged = False
        self.last_rx = rx
        self.peer = addr
        bid = p["boot_id"]
        if bid != self.boot_id:
            if self.boot_id is not None:
                self.boots += 1
                self.holes_before = self.holes
                self.drops_before = self.robot_drops
                self.events.append((wall, "boot", f"{self.name} rebooted: boot id "
                                    f"{self.boot_id:08x} -> {bid:08x}"))
                self.log("flight", f"{self.name} rebooted: boot id {self.boot_id:08x} "
                         f"-> {bid:08x}")
            self.boot_id = bid
            self.seq = SeqTracker()
            self.drop_base = p["drop_ring"] + p["drop_net"]
            self.offset = _WindowedMin(OFFSET_WINDOW_S)
            self.ever_ok[:] = False
            self.fault_since = [None] * NJ
        before = len(self.seq.missing)
        self.seq.add(p["seq"])
        d = len(self.seq.missing) - before
        if d:
            k = int(rx)
            self.hole_bins[k] = self.hole_bins.get(k, 0) + d
            if len(self.hole_bins) > HOLE_BINS_KEEP_S:
                for old in [s for s in self.hole_bins if s < k - HOLE_BINS_KEEP_S]:
                    del self.hole_bins[old]
        self.drop_now = p["drop_ring"] + p["drop_net"]
        if p["t_utc_us"]:
            self.skew.add(rx, wall * 1e6 - p["t_utc_us"])
        t = p["type"]
        if t == F.T_TICK:
            self._tick(p, rx)
        elif t == F.T_EVENT:
            self.n_events += 1
            utc = p["t_utc_us"] / 1e6 if p["t_utc_us"] else wall
            self.events.append((utc, *describe_event(p)))
        elif t == F.T_HELLO:
            self.hellos += 1
            self.hello = p
            self.hello_rx = rx
            self.zero = np.array(p["zero"], dtype=float)
            self.dir = np.array(p["dir"], dtype=float)

    def _tick(self, p: dict, rx: float) -> None:
        self.ticks += 1
        tr = p["t_boot_us"] / 1e6
        self.offset.add(rx, rx - tr)
        t = tr + self.offset.value
        c = p["ctrl"]
        row = (p["source"], p["flags"], p["link_state"], p["tick"], p["work_us"],
               p["fb_off_us"], p["fb_read_us"], p["imu_age_us"], *p["accel"], *p["gyro"],
               *p["up"], p["roll"], p["pitch"], c["phase"], c["vbat_mv"], c["batt"],
               c["home_stage"], p["bal"]["state"], *c["cmd"])
        jrow = np.full((NJ, len(JOINT_COLS)), np.nan, dtype=np.float32)
        read_ran = bool(p["flags"] & F.TF_FB_READ)
        for j in p["joints"]:
            b = j["bus"]
            if b >= NJ:
                continue
            fl = j["flags"]
            ok = bool(fl & F.JF_FB_OK)
            if ok:
                self.ever_ok[b] = True
                self.fault_since[b] = None
            elif read_ran and self.fault_since[b] is None:
                self.fault_since[b] = t
            r = jrow[b]
            r[:] = [j[k] for k in JOINT_COLS]
            if not fl & F.JF_NOM_VALID:
                r[JC["nominal"]] = np.nan
            if not fl & F.JF_GOAL_VALID:
                r[JC["goal"]] = np.nan
            if not self.ever_ok[b]:
                r[_FB_COLS] = np.nan        # never answered: the block is not a reading
        if self.simulated:
            jrow[:, _NO_SIM_COLS] = np.nan  # the sim has no servo load/current/V/temp/status
        self.ring.push(t, row, jrow)
        self.last_tick = p
        self.last_tick_rx = rx
        self.rx_times.append(rx)
        if p["source"] == F.SRC_CTRL:
            self.cmd = tuple(c["cmd"])
            self.cmd_rx = rx


class FlightStore:
    """The console's flight-record receiver and store (see the module doc)."""

    def __init__(self, opts, log=None):
        self.log = log if log is not None else (lambda kind, msg: None)
        self.src = {"robot": _Source("robot", getattr(opts, "flight_port", 0) or 0, self.log),
                    "sim": _Source("sim", getattr(opts, "sim_flight_port", 0) or 0, self.log)}

    # -- the link tick ---------------------------------------------------------
    def poll(self, now_ms: float | None = None) -> int:
        """Drain both sockets. Returns the datagrams read."""
        n = 0
        for s in self.src.values():
            if s.sock is None:
                continue
            for _ in range(MAX_PER_POLL):
                try:
                    data, addr = s.sock.recvfrom(4096)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError as e:
                    s.error = str(e)
                    break
                n += 1
                s.ingest(data, addr, time.monotonic(), time.time())
            now = time.monotonic()
            if s.first_rx is not None and s.stale(now) and not s.stale_logged:
                s.stale_logged = True
                self.log("flight", f"{s.name} records stopped ({STALE_S:.0f} s silent)")
        return n

    def close(self) -> None:
        for s in self.src.values():
            s.close()

    # -- for the safety bar and the Session tab ------------------------------
    def line(self) -> str:
        """The safety bar's flight-recorder part: short, fixed width, HTML spans."""
        now = time.monotonic()
        dim = "<span style='color:#8a9099'>"
        r = self.src["robot"]
        if not r.port:
            out = f"{dim}flight&nbsp;rec&nbsp;off</span>"
        elif r.closed:
            out = f"{dim}flight&nbsp;rec&nbsp;:{r.port}&nbsp;closed</span>"
        elif r.sock is None:
            out = (f"<span style='color:#ff5a50'>flight&nbsp;rec&nbsp;:{r.port}&nbsp;"
                   f"BIND&nbsp;FAILED</span>")
        elif r.first_rx is None:
            out = f"{dim}flight&nbsp;rec&nbsp;:{r.port}&nbsp;waiting</span>"
        elif r.stale(now):
            out = (f"<span style='color:#c98a1b'>flight&nbsp;rec&nbsp;:{r.port}&nbsp;"
                   f"no&nbsp;records&nbsp;{_nb(f'{now - r.last_rx:5.0f}')}&nbsp;s</span>")
        else:
            out = (f"{dim}flight&nbsp;rec</span>&nbsp;:{r.port}&nbsp;&nbsp;"
                   f"{_nb(f'{r.rate(now):4.1f} Hz')}&nbsp;&nbsp;{_nb(f'holes {r.holes:3d}')}")
            sk = r.skew.value
            if sk is not None:
                out += (f"&nbsp;&nbsp;{dim}clock&nbsp;Δ</span>&nbsp;"
                        f"{_nb(f'{sk / 1000.0:+4.0f} ms')}")
        s = self.src["sim"]
        if s.first_rx is not None:
            if s.stale(now):
                out += "&nbsp;&nbsp;<span style='color:#8a9099'>sim&nbsp;none</span>"
            else:
                out += (f"&nbsp;&nbsp;<span style='color:#ffa41f'>sim</span>&nbsp;"
                        f"{_nb(f'{s.rate(now):4.1f} Hz')}")
        return out

    def detail(self) -> str:
        """Plain text for the Session tab."""
        now = time.monotonic()
        lines = []
        for s in self.src.values():
            head = f"{s.name:<5s}  "
            if not s.port:
                lines.append(head + "off (port 0)")
                continue
            if s.sock is None:
                lines.append(head + f":{s.port}  {s.error}")
                continue
            if s.first_rx is None:
                lines.append(head + f":{s.port}  waiting: nothing received yet")
                continue
            state = (f"no records for {now - s.last_rx:.0f} s" if s.stale(now)
                     else f"{s.rate(now):.1f} Hz, last {now - s.last_rx:.1f} s ago")
            lines.append(head + f":{s.port}  from {s.peer[0]}:{s.peer[1]}  {state}")
            h = s.hello
            if h is not None:
                fit = s.fitted()
                sk = s.skew.value
                clock = ("NTP synced" if h["utc_synced"] else "NTP not synced") + (
                    f", clock Δ {sk / 1000.0:+.0f} ms vs this host (incl. transit)"
                    if sk is not None else "")
                lines.append(f"       fw {h['fw'] or '?'} · calibration "
                             f"{'ok' if h['cal_ok'] else 'NOT LOADED'} · {len(fit)} of {NJ} "
                             f"fitted · {clock} · recorder {'on' if h['enabled'] else 'off'}, "
                             f"idle {h['idle_hz']} Hz · uptime {h['uptime_ms'] / 1000.0:.0f} s")
            else:
                lines.append("       no HELLO yet: no calibration, angles wait for it")
            lines.append(f"       packets {s.packets} (tick {s.ticks}, event {s.n_events}, "
                         f"hello {s.hellos}) · holes {s.holes} · robot drops {s.robot_drops} · "
                         f"undecodable {s.undecodable} · reboots {s.boots}"
                         + (f" · boot {s.boot_id:08x}" if s.boot_id is not None else ""))
            if s.undecodable and s.last_error:
                lines.append(f"       last undecodable: {s.last_error}")
        return "\n".join(lines)

    def events(self, source: str = "robot") -> list:
        """(utc seconds, kind, message) per EVENT packet, oldest first; kind is
        flightrec.EVENT_NAMES' (cmd, out, arm_edge, mode, link_state, fall,
        flail, stop, ...), plus "boot" when the boot id changes."""
        return list(self.src[source].events)

    def policy_cmd(self):
        """The 7-wide command the robot's policy saw on its newest ctrl tick
        (vx, vy, wz, crouch, lift, foot_dx, foot_dz), or None after 1 s."""
        r = self.src["robot"]
        if r.cmd is None or time.monotonic() - r.cmd_rx > CMD_STALE_S:
            return None
        return r.cmd

    # -- for the 3D view and the tabs ------------------------------------------
    def latest(self, source: str = "robot"):
        """The newest tick of a source, or None if it never sent one:
        {"t": host time.monotonic() of arrival, "age_s", "fitted": [bus names],
         "up": (x, y, z) or None, "joints": {bus_name: {"pos", "goal", "nominal"
         (rad or None), "flags", "fb_ok"}}}. A servo that missed this tick's
        read has fb_ok False and pos its held last good angle (None if it has
        not answered since this console started listening)."""
        s = self.src.get(source)
        if s is None or s.last_tick is None:
            return None
        p = s.last_tick
        joints = {}
        for j in p["joints"]:
            b = j["bus"]
            if b >= NJ:
                continue
            fl = j["flags"]
            joints[BUS[b]] = {
                "pos": s.rad(b, j["pos"]) if s.ever_ok[b] else None,
                "goal": s.rad(b, j["goal"]) if fl & F.JF_GOAL_VALID else None,
                "nominal": s.rad(b, j["nominal"]) if fl & F.JF_NOM_VALID else None,
                "flags": fl,
                "fb_ok": bool(fl & F.JF_FB_OK),
            }
        up = tuple(p["up"]) if p["flags"] & F.TF_IMU_VALID else None
        return {"t": s.last_tick_rx, "age_s": time.monotonic() - s.last_tick_rx,
                "fitted": s.fitted(), "up": up, "joints": joints}

    def feedback(self, source: str = "robot") -> dict:
        """Per joint in the newest tick, the servo's feedback block in units:
        {bus_name: {"temp_c", "load_pct", "current_ma", "voltage_v", "status",
        "fb_ok", "answered" (since listening), "fault" (missed a read that
        ran), "fault_since" (host monotonic s or None), "simulated"}}. Values
        are None until the servo has answered once. "simulated": the sim
        twin's records, which have no load, current, voltage, temperature or
        status -- those are None and status is None, never a reading."""
        s = self.src[source]
        p = s.last_tick
        if p is None:
            return {}
        read_ran = bool(p["flags"] & F.TF_FB_READ)
        sim = s.simulated
        out = {}
        for j in p["joints"]:
            b = j["bus"]
            if b >= NJ:
                continue
            ok = bool(j["flags"] & F.JF_FB_OK)
            have = bool(s.ever_ok[b])
            real = have and not sim
            out[BUS[b]] = {
                "temp_c": j["temp_c"] if real else None,
                "load_pct": j["load"] / 10.0 if real else None,
                "current_ma": j["current"] * 6.5 if real else None,
                "voltage_v": j["voltage_dv"] / 10.0 if real else None,
                "status": None if sim else (j["status"] if have else 0),
                "simulated": sim,
                "fb_ok": ok, "answered": have, "fault": read_ran and not ok,
                "fault_since": s.fault_since[b],
            }
        return out

    def hello(self, source: str = "robot"):
        """The newest HELLO exactly as flightrec.decode() returns it (fw,
        cal_ok, enabled, fitted, id, dir, zero, the counters...), or None if
        there is none or it is older than STALE_S (HELLO comes at 1 Hz)."""
        s = self.src[source]
        if s.hello is None or time.monotonic() - s.hello_rx > STALE_S:
            return None
        return s.hello

    def last_tick(self, source: str = "robot"):
        """The newest TICK exactly as flightrec.decode() returns it (its
        `joints` list carries bus, flags, temp_c, ...), or None if there is
        none or it is older than STALE_S. Raw steps: latest() has radians."""
        s = self.src[source]
        if s.last_tick is None or time.monotonic() - s.last_tick_rx > STALE_S:
            return None
        return s.last_tick

    def simulated(self, source: str = "robot") -> bool:
        """This source's records come from the sim twin (sim/sil_twin.py),
        whose servos have no load, current, voltage, temperature or status."""
        return self.src[source].simulated

    def bus_id(self, source: str, name: str):
        """The servo ID the robot's HELLO gives a bus joint, or None."""
        h = self.src[source].hello
        return h["id"][BUS_INDEX[name]] if h is not None else None

    def stats(self, source: str = "robot") -> dict:
        """Counters and state of one source, for tables."""
        s = self.src[source]
        now = time.monotonic()
        sk = s.skew.value
        return {"port": s.port, "bound": s.sock is not None, "error": s.error,
                "seen": s.first_rx is not None, "stale": s.stale(now),
                "age_s": None if s.last_rx is None else now - s.last_rx,
                "rate_hz": s.rate(now), "packets": s.packets, "ticks": s.ticks,
                "events": s.n_events, "hellos": s.hellos, "holes": s.holes,
                "robot_drops": s.robot_drops,
                "network_holes": max(0, s.holes - s.robot_drops),
                "undecodable": s.undecodable, "reboots": s.boots,
                "boot_id": s.boot_id, "peer": s.peer,
                "skew_ms": None if sk is None else sk / 1000.0,
                "calibrated": s.zero is not None, "simulated": s.simulated,
                "fw": s.hello["fw"] if s.hello is not None else None}

    def fresh(self, source: str) -> bool:
        """Ticks have arrived from this source within STALE_S."""
        s = self.src[source]
        return s.last_tick_rx is not None and time.monotonic() - s.last_tick_rx <= STALE_S

    def series(self, source: str, name: str, window_s: float, now: float | None = None):
        """(t, v) numpy arrays over the window_s seconds up to now (default:
        time.monotonic(); a paused view passes the instant it froze); t is
        seconds before now (<= 0, this host's monotonic clock, the robot's
        sample spacing).

        Joint fields, "<bus_name>.<field>": pos, goal, nominal (rad), err
        (pos - goal, rad, as tools/flight_reader.py), err_nominal (pos -
        nominal), speed (rad/s), load (%), current (mA), voltage (V), temp
        (°C), status, flags. NaN where a value is invalid (the sim twin's
        load, current, voltage, temp, status: it has none); angles are empty
        until the source's HELLO has given the calibration.
        Tick fields: work_ms, fb_read_ms, imu_age_ms, tilt_deg (from the up
        vector), gyro (|ω| rad/s), gyro_x/y/z, roll_deg, pitch_deg, vbat (V,
        ctrl ticks only), phase, link_state, source, flags, cmd.<vx|vy|wz|
        crouch|lift|foot_dx|foot_dz>.
        "holes": sequence holes per 1 s bin (host arrival time)."""
        s = self.src[source]
        now = time.monotonic() if now is None else now
        if name == "holes":
            bins = np.arange(int(now - window_s), int(now) + 1)
            return (bins + 0.5 - now).astype(float), np.array(
                [float(s.hole_bins.get(int(b), 0)) for b in bins])
        r = s.ring
        sl = r.between(now - window_s, now)
        t = r.t[sl] - now
        if "." in name and name.split(".", 1)[0] in BUS_INDEX:
            jn, field = name.split(".", 1)
            b = BUS_INDEX[jn]
            col = r.j[sl, b]
            return t, self._joint_field(s, b, col, field)
        tk = r.tick[sl]
        nan = np.nan
        with np.errstate(invalid="ignore"):
            if name == "work_ms":
                v = tk[:, TC["work_us"]] / 1000.0
            elif name == "fb_read_ms":
                v = tk[:, TC["fb_read_us"]] / 1000.0
            elif name == "imu_age_ms":
                v = tk[:, TC["imu_age_us"]] / 1000.0
            elif name == "tilt_deg":
                v = np.degrees(np.arccos(np.clip(tk[:, TC["upz"]], -1.0, 1.0)))
                v[(tk[:, TC["flags"]].astype(int) & F.TF_IMU_VALID) == 0] = nan
            elif name == "gyro":
                v = np.sqrt(tk[:, TC["gx"]] ** 2 + tk[:, TC["gy"]] ** 2 + tk[:, TC["gz"]] ** 2)
                v[(tk[:, TC["flags"]].astype(int) & F.TF_IMU_VALID) == 0] = nan
            elif name in ("gyro_x", "gyro_y", "gyro_z"):
                v = tk[:, TC["g" + name[-1]]].copy()
            elif name in ("roll_deg", "pitch_deg"):
                v = np.degrees(tk[:, TC[name[:-4]]])
            elif name == "vbat":
                v = tk[:, TC["vbat_mv"]] / 1000.0
                v[(tk[:, TC["source"]] != F.SRC_CTRL) | (v <= 0)] = nan
            elif name.startswith("cmd."):
                v = tk[:, TC[f"cmd{CMD_NAMES.index(name[4:])}"]].copy()
                v[tk[:, TC["source"]] != F.SRC_CTRL] = nan
            elif name in TC:
                v = tk[:, TC[name]].copy()
            else:
                raise KeyError(f"unknown series {name!r}")
        return t, v

    @staticmethod
    def _joint_field(s: _Source, b: int, col, field: str):
        if field in ("pos", "goal", "nominal", "err", "err_nominal", "speed") and s.zero is None:
            return np.full(len(col), np.nan)      # no HELLO yet: no calibration
        k = RAD * (s.dir[b] if s.dir is not None else 1.0)
        z = s.zero[b] if s.zero is not None else 0.0
        if field in ("pos", "goal", "nominal"):
            return (col[:, JC[field]].astype(float) - z) * k
        if field == "err":
            return (col[:, JC["pos"]].astype(float) - col[:, JC["goal"]]) * k
        if field == "err_nominal":
            return (col[:, JC["pos"]].astype(float) - col[:, JC["nominal"]]) * k
        if field == "speed":
            return col[:, JC["speed"]].astype(float) * k
        if field == "load":
            return col[:, JC["load"]].astype(float) / 10.0
        if field == "current":
            return col[:, JC["current"]].astype(float) * 6.5
        if field == "voltage":
            return col[:, JC["voltage_dv"]].astype(float) / 10.0
        if field == "temp":
            return col[:, JC["temp_c"]].astype(float)
        if field in JC:
            return col[:, JC[field]].astype(float)
        raise KeyError(f"unknown joint field {field!r}")


def _nb(s: str) -> str:
    return s.replace(" ", "&nbsp;")


def decimate(t, v, max_points: int):
    """Peak-preserving decimation for plotting: per block, its min and max, so
    a spike survives a 2 min window drawn into a few hundred pixels."""
    n = len(t)
    if n <= max_points or max_points < 4:
        return t, v
    k = int(math.ceil(n / (max_points // 2)))
    m = (n // k) * k
    head = n - m                       # keep the newest samples exact: trim the oldest
    tb = t[head:].reshape(-1, k)
    vb = v[head:].reshape(-1, k)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        lo = np.nanmin(vb, axis=1)
        hi = np.nanmax(vb, axis=1)
    tt = np.empty(2 * len(tb))
    vv = np.empty(2 * len(tb))
    tt[0::2] = tb[:, 0]
    tt[1::2] = tb[:, -1]
    vv[0::2] = lo
    vv[1::2] = hi
    return tt, vv
