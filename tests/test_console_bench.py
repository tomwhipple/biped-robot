"""The Qt console's bench (link/console/bench.py, tab_bench.py): the port of
link/bench.cpp, held to firmware/host/test_client.cpp's bench cases, then the
console end to end against fake servers on loopback -- a UDP bridge that
answers {"sub":1}, an MJPEG relay serving tiny JPEGs -- headless and in the
offscreen window. No serial port, no camera, no robot.

The model tests need nothing but python. The rest need PyQt6 (QImage makes
and decodes the JPEGs) and the core library (make -C firmware/host console);
the window test also needs pyqtgraph.

Run:  QT_QPA_PLATFORM=offscreen python -m pytest tests/test_console_bench.py -q
"""
import itertools
import json
import os
import pty
import re
import select
import socket
import subprocess
import sys
import threading
import time
import tty
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

from console import bench  # noqa: E402
from console.bench import (BEACON_PREFER_MS, STALE_MS, TS_CAP, BenchLink,  # noqa: E402
                           BenchModel, TimeSeries, parse_flat)

V2 = os.path.join(ROOT, "link", "bimo_console.py")
LIB = os.path.join(ROOT, "firmware", "host", "build", "libbimo_console."
                   + ("dylib" if sys.platform == "darwin" else "so"))
_ports = itertools.count(5212, 4)

try:
    from PyQt6 import QtCore, QtGui
    HAVE_QT = True
except ImportError:
    HAVE_QT = False
needs_qt = pytest.mark.skipif(not HAVE_QT, reason="needs PyQt6")
needs_lib = pytest.mark.skipif(not os.path.exists(LIB),
                               reason="build it: make -C firmware/host console")


# -- the flat JSON reader (test_json_flat_objects, test_json_rejects_garbage) ------
def test_json_flat_objects():
    j = parse_flat(b'{"v":1,"k":"joint","id":52,"volt":12.0,"x":-1.5e0,'
                   b'"ok":true,"no":false,"n":null}')
    assert j is not None
    assert bench.num(j, "id") == 52.0
    assert abs(bench.num_or(j, "x", 0) + 1.5) < 1e-9
    assert bench.text(j, "k") == "joint"
    assert bench.flag(j, "ok", False) and not bench.flag(j, "no", True)
    assert "n" in j and bench.num(j, "n") is None          # null is not a number
    assert bench.text(j, "id") is None                     # a number is not a string
    assert bench.num(j, "ok") is None                      # nor is a bool
    assert parse_flat(b" { } ") == {}
    j = parse_flat(b'{"s":"a\\"b\\\\c\\u0041"}')
    assert bench.text(j, "s") == 'a"b\\cA'
    assert bench.text(parse_flat(b'{"s":"\\u00e9\\n\\t\\/"}'), "s") == "?\n\t/"
    assert bench.num(parse_flat(b'{"a":1,"a":2}'), "a") == 1.0   # the first wins


def test_json_rejects_garbage():
    """The parser eats datagrams from a network. Hostile and broken ones must
    be refused, never half-read."""
    bad = [b"", b"{", b"}", b"[]", b'{"a":1', b'{"a":1,}', b'{"a"1}', b"{a:1}",
           b'{"a":{"b":1}}', b'{"a":[1]}', b'{"a":nan}', b'{"a":1e999}',
           b'{"a":"unterminated}', b'{"a":1} trailing', b'{"a":tru}',
           b'{"a":"\\q"}', b'{"a":"\\u12"}', b'{"a":--1}', b"\xff\xfe",
           b'{"a":1,"a":2,"b":', b'{"a":"\x01"}',
           b"{} trailing", b'{"a":"\\r"}', b'{"a":1_0}', b'{"a":1}}']
    for b in bad:
        assert parse_flat(b) is None, b
    big = b"{" + b",".join(b'"k%d":1' % i for i in range(30)) + b"}"
    assert parse_flat(big) is None                          # too many keys
    ok24 = b"{" + b",".join(b'"k%d":1' % i for i in range(24)) + b"}"
    assert parse_flat(ok24) is not None and parse_flat(ok24[:-1] + b',"k24":1}') is None
    # a very long string and key are truncated, not overflowed
    longs = b'{"%s":1,"s":"%s"}' % (b"7".zfill(300), b"7".zfill(400))
    j = parse_flat(longs)
    assert j is not None and len(bench.text(j, "s")) < 80
    assert all(len(k) < 20 for k in j)
    # the data is not NUL-terminated: the length is the datagram's
    exact = b'{"a":12}garbage'
    assert bench.num_or(parse_flat(exact[:8]), "a", 0) == 12.0
    assert parse_flat(exact[:7]) is None


# -- the model ----------------------------------------------------------------------
def A(m, s, t):
    return m.apply(s.encode() if isinstance(s, str) else s, t)


def test_bench_model_applies_samples():
    m = BenchModel()
    assert not m.live(0.0)
    assert A(m, '{"v":1,"k":"joint","id":52,"name":"R_hip_roll","pos":2082,'
                '"spd":0,"load":-16,"volt":12.0,"temp":24,"err":0}', 1000)
    assert A(m, '{"v":1,"k":"cal","id":52,"name":"R_hip_roll","zero":2048,'
                '"dir":-1,"fitted":true}', 1010)
    b = m.joints[52]
    assert b.have and b.have_cal and b.pos == 2082 and b.load == -16 and b.zero == 2048
    assert b.dir == -1 and b.name == "R_hip_roll"
    assert m.joints_seen() == 1
    assert A(m, '{"v":1,"k":"bal_roll","roll":0.37}', 1020)
    assert m.have_roll and not m.roll_na
    assert A(m, '{"v":1,"k":"bal_state","state":"ACTIVE","stance":"R","since":2.5}', 1030)
    assert m.state == "ACTIVE" and m.stance == "R"
    assert A(m, '{"v":1,"k":"bal_ref","ref":-0.48,"err":-0.11,"I":-0.0011,"u":-0.06}', 1040)
    assert abs(m.err + 0.11) < 1e-6
    assert m.s_err.count() == 1 and m.s_u.count() == 1 and m.s_roll.count() == 1
    # "roll now unavailable" must not leave the old roll looking current
    assert A(m, '{"v":1,"k":"bal_roll","roll":null}', 1050)
    assert m.roll_na and m.s_roll.count() == 1
    assert A(m, '{"v":1,"k":"bal_state","state":"off","stance":null,"since":null}', 1060)
    assert m.state == "off" and m.stance == ""
    assert A(m, '{"v":1,"k":"imu_up","x":-0.08,"y":0.02,"z":0.99}', 1070)
    assert A(m, '{"v":1,"k":"imu_raw","ax":0.7,"ay":9.3,"az":0.6,"amag":9.4,'
                '"gx":0,"gy":0.04,"gz":0}', 1080)
    assert m.have_up and m.have_raw and not m.have_gyro
    assert A(m, '{"v":1,"k":"hb","polling":true,"rx_age":0.4,"polls":7}', 1090)
    assert m.hb_polling and m.hb_polls == 7 and m.live(1100) and not m.live(1090 + STALE_MS + 1)
    # forward compatible: an unknown kind keeps the bridge "heard from"
    assert A(m, '{"v":1,"k":"future","q":1}', 2000)
    assert m.last_rx_ms == 2000
    # refused: wrong version, no kind, joint id out of range, not JSON
    before = m.datagrams
    assert not A(m, '{"v":2,"k":"hb"}', 3000)
    assert not A(m, '{"v":1}', 3000)
    assert not A(m, '{"v":1,"k":"joint","id":9999,"pos":1}', 3000)
    assert not A(m, '{"v":1,"k":"joint","id":-1,"pos":1}', 3000)
    assert not A(m, "not json", 3000)
    assert not A(m, '{"v":"1","k":"hb"}', 3000)            # a string is not version 1
    assert not A(m, '{"v":1,"k":7}', 3000)                  # nor a number a kind
    assert m.datagrams == before and m.last_rx_ms == 2000 and m.rejected >= 7


def test_beacon_servo_outranks_bridge_then_yields():
    """One sample model, two sources: the beacon outranks the bridge while
    fresh, the bridge fills in when it is not."""
    m = BenchModel()
    j = ('{"v":1,"k":"joint","id":52,"pos":2000,"spd":0,'
         '"load":1,"volt":12.0,"temp":24,"err":0}')
    A(m, j, 1000)
    assert m.joints[52].pos == 2000 and not m.joints[52].from_beacon
    m.apply_beacon_servo(52, 2082, 3, -16, 40, 119, 25, 0, 20, 2000)
    assert m.joints[52].pos == 2082 and m.joints[52].from_beacon and m.joints[52].cur == 40
    assert abs(m.joints[52].volt - 11.9) < 1e-4
    assert abs(m.joints[52].t_ms - 1980.0) < 1e-6          # the reading's own instant
    n = m.datagrams
    assert A(m, j, 2500)                                    # beacon fresh: outranked ...
    assert m.joints[52].pos == 2082
    assert m.datagrams == n + 1 and m.live(2500)            # ... but the bridge was heard
    A(m, j, 1980 + BEACON_PREFER_MS + 1)                    # beacon silent: the fallback
    assert m.joints[52].pos == 2000 and not m.joints[52].from_beacon
    m.apply_beacon_servo(7, 1, 1, 1, 1, 120, 20, 0, 255, 5000)    # unfitted
    m.apply_beacon_servo(99, 1, 1, 1, 1, 120, 20, 0, 0, 5000)     # out of range
    assert not m.joints[7].have
    # a cal sample is never outranked: the zero comes only from the bridge
    m.apply_beacon_servo(52, 2100, 0, 0, 0, 120, 25, 0, 0, 6000)
    assert A(m, '{"v":1,"k":"cal","id":52,"zero":2048,"dir":1,"fitted":true}', 6100)
    assert m.joints[52].have_cal and m.joints[52].zero == 2048 and m.joints[52].pos == 2100


def test_beacon_up_does_not_fake_a_bridge():
    m = BenchModel()
    m.apply_beacon_up(0.1, 0.0, 0.99, 20.0)
    assert m.have_up and m.datagrams == 0 and not m.live(21.0)


def test_timeseries_resample():
    s = TimeSeries()
    first, out = s.resample(60000, 30000, 30)
    assert first == 30                                     # empty: nothing valid
    for t in range(10, 41):                                # 1 Hz from 10 s to 40 s, value = s
        s.push(t * 1000.0, float(t))
    first, out = s.resample(40000, 30000, 30)              # window 10..40 s
    assert first == 0
    assert abs(out[29] - 40.0) < 1e-6
    assert abs(out[0] - 11.0) < 1e-6                       # bin 0 ends at 11 s
    assert all(out[i] >= out[i - 1] for i in range(1, 30))
    # later: the data has aged out of the right edge, held, not blanked
    first, out = s.resample(50000, 30000, 30)
    assert first == 0 and abs(out[29] - 40.0) < 1e-6
    # a window that starts before the first sample has leading invalid bins
    r = TimeSeries()
    r.push(25000, 5.0)
    first, out = r.resample(30000, 30000, 30)
    assert 20 < first < 30 and abs(out[29] - 5.0) < 1e-6
    # wrap-around: more samples than the ring holds stay in order
    w = TimeSeries()
    for i in range(TS_CAP * 2 + 7):
        w.push(i * 10.0, float(i))
    assert w.count() == TS_CAP
    now = (TS_CAP * 2 + 6) * 10.0
    first, out = w.resample(now, 3000, 30)
    assert first == 0 and abs(out[29] - (TS_CAP * 2 + 6)) < 1e-6 and out[0] < out[29]
    assert w.resample(now, 3000, 0)[0] == 0


# -- the socket ---------------------------------------------------------------------
def _udp():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    return s, f"127.0.0.1:{s.getsockname()[1]}"


def test_bench_link_subscribes_and_receives():
    """The real socket path against a UDP stand-in for the bridge: subscribes,
    is fed, and stops being live when the bridge goes quiet."""
    srv, spec = _udp()
    srv.settimeout(1.0)
    link = BenchLink()
    assert not link.open("", 1)
    assert not link.open("host:99999", 1) and "bad port" in link.err
    assert not link.open("host:abc", 1) and "bad port" in link.err
    assert not link.open(":8650", 1) and "expected HOST" in link.err
    assert not link.open("no.such.host.invalid", 8650) and "cannot resolve" in link.err
    assert not link.active
    assert link.open(spec, 1)
    seen = []
    link.sink = lambda d, t: seen.append((d, t))
    link.poll(0.0)
    d, frm = srv.recvfrom(64)
    assert d == b'{"sub":1}'
    # not again until the next lease refresh
    link.poll(500.0)
    srv.settimeout(0.1)
    with pytest.raises(socket.timeout):
        srv.recvfrom(64)
    link.poll(2100.0)
    assert srv.recvfrom(64)[0] == b'{"sub":1}'
    d1 = b'{"v":1,"k":"bal_roll","roll":-7.6}'
    srv.sendto(d1, frm)
    srv.sendto(b"garbage", frm)
    time.sleep(0.05)
    link.poll(2200.0)
    m = link.model
    assert m.have_roll and m.rejected == 1 and m.datagrams == 1
    assert seen == [(d1, 2200.0)]                           # the sink: accepted only, verbatim
    assert m.live(2300.0) and not m.live(2200.0 + STALE_MS + 10)
    assert link.state_name(2300.0) == "live"
    assert link.state_name(2200.0 + STALE_MS + 10) == "STALE"
    link.close()
    assert srv.recvfrom(64)[0] == b'{"unsub":1}'
    assert link.state_name(0) == "off"
    srv.close()


class StubCore:
    def __init__(self):
        self.rec_active, self.rec_path = False, ""
        self.bench_rows, self.cam_rows = [], []

    def status(self, now):
        return SimpleNamespace(rec_active=self.rec_active)

    def rec_bench(self, now, d):
        self.bench_rows.append((now, bytes(d)))

    def rec_cam(self, now, pane, src, url):
        self.cam_rows.append((pane, src, url))


class StubCtl:
    def __init__(self, **o):
        base = dict(bench_telemetry=None, cam_relay=None, cam=[])
        base.update(o)
        self.o = SimpleNamespace(**base)
        self.core = StubCore()
        self.listeners, self.logs = [], []

    def log(self, kind, msg):
        self.logs.append((kind, msg))

    def tick(self, **st):
        base = dict(tlm_count=0, have_att=0, up_x=0.0, up_y=0.0, up_z=1.0,
                    rec_active=self.core.rec_active)
        base.update(st)
        for fn in self.listeners:
            fn(SimpleNamespace(**base))


def test_the_console_feeds_beacon_and_bridge_and_records_what_it_shows():
    srv, spec = _udp()
    srv.settimeout(1.0)
    clock = [1000.0]
    ctl = StubCtl(bench_telemetry=spec)
    b = bench.Bench(ctl, clock=lambda: clock[0])
    assert ("bench", f"subscribing to {spec}") in ctl.logs
    ctl.tick()
    d, frm = srv.recvfrom(64)
    assert d == b'{"sub":1}'
    ok = b'{"v":1,"k":"joint","id":52,"pos":2000,"load":1,"volt":12.0,"temp":24,"err":0}'
    srv.sendto(ok, frm)
    srv.sendto(b'{"v":2,"k":"hb"}', frm)
    time.sleep(0.05)
    clock[0] = 1100.0
    ctl.tick(tlm_count=1, have_att=1, up_x=0.1, up_y=-0.2, up_z=0.97)
    m = b.link.model
    assert m.joints[52].pos == 2000 and m.datagrams == 1 and m.rejected == 1
    assert ctl.core.bench_rows == [(1100.0, ok)]            # accepted, verbatim, arrival clock
    assert m.have_up and abs(m.up[2] - 0.97) < 1e-6 and m.t_up == 1100.0
    assert ("bench", f"telemetry bridge heard ({spec})") in ctl.logs
    # the same beacon again is not news; a beacon without attitude is no up
    clock[0] = 1200.0
    ctl.tick(tlm_count=1, have_att=1, up_x=0.0, up_y=0.0, up_z=0.5)
    ctl.tick(tlm_count=2, have_att=0, up_z=0.5)
    assert abs(m.up[2] - 0.97) < 1e-6 and m.datagrams == 1
    assert "bench live joints 1" in b.status_text(1200.0)
    clock[0] = 1100.0 + STALE_MS + 1
    ctl.tick(tlm_count=2)
    assert ("bench", f"telemetry bridge LOST ({spec})") in ctl.logs
    assert "bench STALE joints 1 up_z 0.970" in b.status_text(clock[0])
    b.close()
    got = []
    srv.settimeout(0.3)
    try:
        while True:
            got.append(srv.recvfrom(64)[0])
    except socket.timeout:
        pass
    assert got == [b'{"sub":1}', b'{"unsub":1}']           # the lease renewed at 4.6 s, then bye
    assert ctl.listeners == []
    srv.close()


def test_an_unreachable_bridge_is_logged_not_fatal():
    ctl = StubCtl(bench_telemetry="no.such.host.invalid:8650")
    b = bench.Bench(ctl, clock=lambda: 0.0)
    assert any(k == "ERROR" and "cannot resolve" in msg for k, msg in ctl.logs)
    assert not b.link.active and b.status_text(0.0) == ""
    ctl.tick()
    b.close()


def test_cam_and_view_verbs():
    assert bench.verb(None, "cam", ["0", "emeet"]) == "no --cam-relay"
    assert bench.verb(None, "cam", ["2", "emeet"]) == "cam <0|1> emeet|brio|sony|off"
    assert bench.verb(None, "cam", ["0", "webcam9"]) == "cam <0|1> emeet|brio|sony|off"
    assert bench.verb(None, "cam", []) == "cam <0|1> emeet|brio|sony|off"
    assert bench.verb(None, "view", ["bench"]) is None      # headless has no views
    ctl = StubCtl(cam_relay="mira")
    b = bench.Bench(ctl, clock=lambda: 0.0)
    assert b.cam_host == "mira:8647"
    views = []
    b.on_view = views.append
    assert bench.verb(b, "view", ["bench"]) is None and views == ["bench"]
    assert bench.verb(b, "cam", ["1", "off"]) is None and b.panes[1].sel is None
    b.close()


# -- fake servers ---------------------------------------------------------------------
def tiny_jpeg(w=64, h=36, rgb=(40, 140, 220)):
    img = QtGui.QImage(w, h, QtGui.QImage.Format.Format_RGB32)
    img.fill(QtGui.QColor(*rgb))
    ba = QtCore.QByteArray()
    buf = QtCore.QBuffer(ba)
    buf.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
    assert img.save(buf, "JPEG")
    return bytes(ba)


class FakeRelay:
    """tools/cam_relay.py's wire format on loopback: multipart/x-mixed-replace,
    Content-Length per part, ~fps frames a second; `paused` stops the frames
    with the connection held open (a wedged camera). `jpeg` is one picture
    for every path, or {path: picture}."""

    def __init__(self, jpeg, fps=12.0, paused=False):
        self.jpeg = jpeg if isinstance(jpeg, dict) else {None: jpeg}
        self.period = 1.0 / fps
        self.paused = threading.Event()
        if paused:
            self.paused.set()
        self.clients = 0
        self.paths = []
        self.frames = 0
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self.sock], [], [], 0.1)
                if not r:
                    continue
                conn, _ = self.sock.accept()
            except (OSError, ValueError):
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        req = b""
        while b"\r\n\r\n" not in req:
            chunk = conn.recv(1024)
            if not chunk:
                conn.close()
                return
            req += chunk
        path = req.split(b" ")[1].decode()
        jpeg = self.jpeg.get(path) or next(iter(self.jpeg.values()))
        with self.lock:
            self.paths.append(path)
            self.clients += 1
        try:
            conn.sendall(b"HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace;"
                         b" boundary=frame\r\nCache-Control: no-cache\r\n\r\n")
            while not self._stop.is_set():
                if not self.paused.is_set():
                    conn.sendall(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d"
                                 b"\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
                    self.frames += 1
                    time.sleep(self.period)
                else:
                    # a held connection: notice a viewer that left
                    r, _, _ = select.select([conn], [], [], 0.1)
                    if r and not conn.recv(1):
                        break
        except OSError:
            pass
        finally:
            conn.close()
            with self.lock:
                self.clients -= 1

    def close(self):
        self._stop.set()
        self.sock.close()


class FakeBridge:
    """tools/board_bridge.py's subscription protocol on loopback: records every
    datagram a console sends, and sends `feed` datagrams to every subscriber."""

    def __init__(self):
        self.sock, self.spec = _udp()
        self.port = self.sock.getsockname()[1]
        self.got = []
        self.subs = set()
        self._stop = threading.Event()
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self.sock], [], [], 0.05)
                if not r:
                    continue
                d, addr = self.sock.recvfrom(2000)
            except (OSError, ValueError):
                return
            self.got.append(d)
            msg = json.loads(d)
            if msg.get("sub"):
                self.subs.add(addr)
            elif msg.get("unsub"):
                self.subs.discard(addr)

    def wait_sub(self, timeout=5.0):
        end = time.time() + timeout
        while not self.subs and time.time() < end:
            time.sleep(0.02)
        assert self.subs, "the console never subscribed"

    def send(self, *datagrams):
        for d in datagrams:
            d = d if isinstance(d, bytes) else json.dumps(d, separators=(",", ":")).encode()
            for a in list(self.subs):
                self.sock.sendto(d, a)

    def close(self):
        self._stop.set()
        self.sock.close()


NAMES = {52: "R_hip_roll", 6: "R_hip_pitch", 7: "R_knee", 8: "R_ankle_pitch",
         11: "R_ankle_roll", 51: "R_hip_yaw", 2: "L_hip_roll", 3: "L_hip_pitch",
         4: "L_knee", 12: "L_ankle_pitch", 10: "L_ankle_roll", 9: "L_hip_yaw"}


def bench_sample(i, joints=True):
    """One round of what a polling bridge publishes, i = 0, 1, 2, ..."""
    import math
    t = i * 0.1
    roll = 1.6 * math.sin(t * 1.3) + 0.3 * math.sin(t * 5.1)
    out = [{"v": 1, "t": time.time(), "k": "bal_roll", "roll": round(roll, 3)},
           {"v": 1, "t": time.time(), "k": "bal_ref", "ref": -0.48, "err": round(roll + 0.48, 3),
            "I": -0.0011, "u": round(-0.25 * (roll + 0.48), 3)},
           {"v": 1, "t": time.time(), "k": "bal_state", "state": "ACTIVE", "stance": "R",
            "since": round(2.5 + t, 1)}]
    if i % 10 == 0:
        out += [{"v": 1, "k": "hb", "polling": True, "rx_lines": 40 + i, "polls": 7 + i,
                 "rx_age": 0.1, "subs": 1},
                {"v": 1, "k": "bal_gains", "kp": 0.25, "ki": 0.5, "ankle": 1.0, "hip": -1.0,
                 "clamp": 10.0, "abort": 12.0},
                {"v": 1, "k": "bal_corr", "side": "R", "ankle_deg": 0.12, "ankle_ticks": 1,
                 "hip_deg": -0.12, "hip_ticks": -1, "clamped": 0},
                {"v": 1, "k": "imu_up", "x": -0.0844, "y": 0.0222, "z": 0.9962},
                {"v": 1, "k": "imu_level", "levelled": True, "streak": 0},
                {"v": 1, "k": "imu_raw", "ax": 0.82, "ay": -0.21, "az": 9.77, "amag": 9.81,
                 "gx": 0.002, "gy": -0.013, "gz": 0.001}]
        if joints:
            for k, (sid, name) in enumerate(NAMES.items()):
                out.append({"v": 1, "k": "cal", "id": sid, "name": name, "zero": 2048,
                            "dir": -1 if k % 2 else 1, "fitted": True})
                out.append({"v": 1, "k": "joint", "id": sid, "name": name,
                            "pos": 2048 + 30 * (k - 6), "spd": 0, "load": 16 - 3 * k,
                            "volt": 12.0, "temp": 24 + k % 3, "err": 0x20 if sid == 10 else 0})
    return out


@pytest.fixture
def relay():
    if not HAVE_QT:
        pytest.skip("needs PyQt6")
    r = FakeRelay(tiny_jpeg(), paused=True)
    yield r
    r.close()


@pytest.fixture
def bridge():
    b = FakeBridge()
    yield b
    b.close()


def _until(cond, timeout=6.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


@needs_qt
def test_a_pane_goes_waiting_live_stale_and_is_recorded(relay):
    ctl = StubCtl(cam_relay=f"127.0.0.1:{relay.port}", cam=["emeet"])
    b = bench.Bench(ctl, clock=lambda: 5.0)
    p = b.panes[0]
    try:
        assert p.sel == "emeet" and p.url == f"http://127.0.0.1:{relay.port}/emeet"
        assert ctl.core.cam_rows == []                     # not recording: no row
        assert _until(lambda: relay.clients == 1) and relay.paths == ["/emeet"]
        assert p.state() == "waiting" and p.frames() == 0
        assert f"cam0 emeet waiting frames 0" in b.status_text(0.0)
        relay.paused.clear()
        assert _until(lambda: p.state() == "live")
        img, seq = p.stream.latest()
        assert seq >= 1 and img.width() == 64
        relay.paused.set()                                 # the camera wedges
        assert _until(lambda: p.state() == "STALE", timeout=4.0)
        assert p.age_s() > 2.0 and "cam0 emeet STALE" in b.status_text(0.0)
        # a take begins: one cam row per pane already pointed somewhere
        ctl.core.rec_active, ctl.core.rec_path = True, "/tmp/a.jsonl"
        ctl.tick()
        ctl.tick()
        assert ctl.core.cam_rows == [(0, "emeet", p.url)]
        b.select(1, "brio")                                # while recording: its own row
        assert ctl.core.cam_rows[-1] == (1, "brio", f"http://127.0.0.1:{relay.port}/brio")
        ctl.core.rec_active = False
        ctl.tick()
        ctl.core.rec_active = True                         # a new take: both again
        ctl.tick()
        assert [r[:2] for r in ctl.core.cam_rows[-2:]] == [(0, "emeet"), (1, "brio")]
        # off lets the camera go; selecting a pane never shows the old stream
        b.select(0, None)
        assert p.state() == "off" and p.stream is None and p.frames() == 0
        assert _until(lambda: relay.clients == 1)          # only brio left
    finally:
        b.close()
    assert _until(lambda: relay.clients == 0)


# -- the headless console end to end ---------------------------------------------------
def headless(tmp_path, *args, record=False):
    return subprocess.Popen(
        [sys.executable, V2, "--headless", *([] if record else ["--no-record"]),
         "--sessions", str(tmp_path), "--keymap", str(tmp_path / "keys.conf"),
         "--tlm-port", str(next(_ports)), *args],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)


def wait_line(proc, pattern, timeout=15):
    """The first new stdout line matching pattern (every line is kept in
    proc.seen)."""
    rx = re.compile(pattern)
    if not hasattr(proc, "seen"):
        proc.seen = []
    end = time.time() + timeout
    start = len(proc.seen)
    while time.time() < end:
        r, _, _ = select.select([proc.stdout], [], [], 0.2)
        if not r:
            continue
        line = proc.stdout.readline()
        if not line:
            break
        proc.seen.append(line)
        if rx.search(line):
            return line
    raise AssertionError(f"no line matching {pattern!r}; saw:\n"
                         + "".join(proc.seen[start:][-8:]))


def stop(proc):
    """Close stdin (the console quits) and return ALL its output."""
    try:
        proc.stdin.close()
    except OSError:
        pass
    try:
        out = proc.communicate(timeout=8)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        out = proc.communicate()[0]
    return "".join(getattr(proc, "seen", [])) + out


@needs_qt
@needs_lib
def test_headless_shows_bench_and_cams_records_them_and_only_subscribes(tmp_path, bridge,
                                                                        relay):
    relay.paused.clear()
    p = headless(tmp_path, "--bench-telemetry", bridge.spec,
                 "--cam-relay", f"127.0.0.1:{relay.port}", "--cam", "emeet", record=True)
    try:
        wait_line(p, r"OBSERVER")
        wait_line(p, r"\| bench waiting joints 0 \| cam0 emeet (waiting|live) frames")
        bridge.wait_sub()
        sent = bench_sample(0)
        bridge.send(*sent, b"garbage", b'{"v":2,"k":"hb"}')
        line = wait_line(p, r"\| bench live joints 12 roll [+-]\d\.\d\d err [+-]\d\.\d\d "
                            r"u [+-]\d\.\d\d bal ACTIVE up_z 0\.996 polling "
                            r"\| cam0 emeet live frames [1-9]")
        assert "OBSERVER" in line and "STOP" not in line
        relay.paused.set()
        wait_line(p, r"cam0 emeet STALE", timeout=8)
        relay.paused.clear()
        wait_line(p, r"cam0 emeet live", timeout=8)
        p.stdin.write("cam 1 brio\nview drive\ncam 7 sony\n")
        p.stdin.flush()
        wait_line(p, r"note: cam <0\|1> emeet\|brio\|sony\|off")
        wait_line(p, r"cam0 emeet live frames \d+ \| cam1 brio (waiting|live)")
        p.stdin.write("cam 0 off\n")
        p.stdin.flush()
        wait_line(p, r"\| bench \S+ joints 12[^|]* \| cam1 brio live")   # cam0 is gone
        assert _until(lambda: relay.clients == 1)          # off released emeet
        # the bridge dies: the console must SAY so
        wait_line(p, r"bench STALE joints 12", timeout=8)
    finally:
        out = stop(p)
    assert "telemetry bridge heard" in out and "telemetry bridge LOST" in out
    assert _until(lambda: relay.clients == 0)
    # Watch-only: everything this console ever said to the bridge.
    msgs = [json.loads(d) for d in bridge.got]
    assert msgs[-1] == {"unsub": 1} and all(m == {"sub": 1} for m in msgs[:-1])
    # The session file: the accepted datagrams verbatim, the panes' sources.
    files = [f for f in os.listdir(tmp_path) if f.endswith(".jsonl")]
    assert len(files) == 1
    rows = [json.loads(ln) for ln in open(tmp_path / files[0]) if ln.strip()]
    rows_b = [r for r in rows if r.get("dir") == "bench"]
    assert len(rows_b) == len(sent)                        # garbage and v2 refused, not kept
    assert [r["d"] for r in rows_b] == json.loads(json.dumps(sent))
    assert all(r["utc"] > 1.7e9 and "t" in r for r in rows_b)
    cams = [(r["pane"], r["src"]) for r in rows if r.get("dir") == "cam"]
    assert cams == [(0, "emeet"), (1, "brio")]
    assert not any(r.get("dir") == "cmd" for r in rows)     # an observer commands nothing


@needs_lib
def test_headless_bench_only_console_says_only_subscribe(tmp_path, bridge):
    p = headless(tmp_path, "--bench-telemetry", bridge.spec)
    try:
        wait_line(p, r"OBSERVER")
        time.sleep(2.8)
        n = len(bridge.got)
        assert 1 <= n <= 3                                 # ~every 2 s, not a flood
        p.stdin.write("cam 0 emeet\n")
        p.stdin.flush()
        wait_line(p, r"note: no --cam-relay")
    finally:
        stop(p)
    msgs = [json.loads(d) for d in bridge.got]
    assert msgs[-1] == {"unsub": 1} and all(m == {"sub": 1} for m in msgs[:-1])


@needs_lib
def test_headless_against_the_real_bridge_prints_what_bimo_gui_prints(tmp_path):
    """tools/board_bridge.py over a pty standing in for the board: the line
    test_gui_bench_e2e.py asserts for bimo_gui, digit for digit."""
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import board_bridge as bb
    from test_board_bridge import FakeBoard
    master, slave = pty.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    board = FakeBoard(master)
    board.start()
    pub = bb.Publisher("127.0.0.1", 0)
    br = bb.Bridge(bb.FdPort(slave), open(tmp_path / "log", "a", buffering=1), pub,
                   bb.Poller(gap_s=0.05, quiet_s=0.2, balance_s=0.3, imu_s=0.5, joint_s=1.0))
    br.start()
    p = headless(tmp_path, "--bench-telemetry", f"127.0.0.1:{pub.port}")
    try:
        wait_line(p, r"bench live joints 12 roll \+0\.37 err -0\.11 u -0\.06 bal ACTIVE "
                     r"up_z 0\.996 polling")
        assert all(c in bb.POLL_COMMANDS for c in board.rx)
        br.stop.set()
        pub.close()
        wait_line(p, r"bench STALE", timeout=10)
    finally:
        stop(p)
        board.stop = True


# -- the window, offscreen ------------------------------------------------------------
@pytest.fixture(scope="module")
def qapp():
    pytest.importorskip("pyqtgraph")
    from PyQt6 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def pump(qapp, seconds, feed=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if feed is not None:
            feed()
        qapp.processEvents()
        time.sleep(0.01)


def bench_window(qapp, tmp_path, *extra):
    from console import options
    from console.app import Window
    from console.controller import Controller
    o = options.parse(["--no-record", "--sessions", str(tmp_path), "--keymap",
                       str(tmp_path / "keys.conf"), "--tlm-port", str(next(_ports)), *extra])
    ctl = Controller(o)
    assert ctl.start()
    w = Window(o, ctl)
    qapp.installEventFilter(w)
    w.show()
    return w


class Feeder:
    """Sends the bridge a round of samples every 0.1 s while pumped."""

    def __init__(self, bridge):
        self.bridge, self.i, self.next = bridge, 0, 0.0

    def __call__(self):
        if time.monotonic() >= self.next and self.bridge.subs:
            self.bridge.send(*bench_sample(self.i))
            self.i += 1
            self.next = time.monotonic() + 0.1


@needs_lib
def test_bench_tab_builds_shows_the_bench_and_lets_go_on_close(qapp, tmp_path, bridge, relay):
    from console import headless as hl
    relay.paused.clear()
    w = bench_window(qapp, tmp_path, "--bench-telemetry", bridge.spec,
                     "--cam-relay", f"127.0.0.1:{relay.port}", "--cam", "emeet")
    try:
        names = [w.tabs.tabText(i) for i in range(w.tabs.count())]
        assert "Bench" in names
        assert w.tabs.tabText(w.tabs.currentIndex()) == "Bench"   # bimo_gui's bench view
        tab = w.tab_widgets["Bench"]
        assert w.ctl.readonly                              # no --host: an observer
        pump(qapp, 1.6, Feeder(bridge))
        assert tab.servos.rowCount() == 12
        assert tab.servos.item(0, 0).text() == "52"        # bimo_gui's order: right leg first
        assert tab.servos.item(0, 1).text() == "R_hip_roll"
        r10 = next(r for r in range(12) if tab.servos.item(r, 0).text() == "10")
        assert tab.servos.item(r10, 8).text() == "0x20"    # a fault byte is shown, in red
        assert tab.bridge.text().startswith("live")
        assert tab.bal.text() == "balance ACTIVE"
        assert tab.up.text().startswith("up  -0.084 +0.022 +0.996")
        xs, ys = tab.charts[0][2].getData()
        assert xs is not None and len(xs) >= 5 and xs[-1] == pytest.approx(0.0)
        assert tab.panes[0].state.text().startswith("live")
        assert tab.panes[0].view.img is not None
        assert tab.panes[1].combo.currentText() == "(off)"
        shots = os.environ.get("BENCH_SHOTS")
        if shots:
            w.grab().save(os.path.join(shots, "bench_tab_live.png"))
        # the combo box is the pane's selector, and a scripted `cam` moves it
        tab.panes[1]._picked(2)
        assert w.ctl.bench.panes[1].sel == "brio"
        hl.command(w.ctl, "cam 1 off")
        pump(qapp, 0.2)
        assert tab.panes[1].combo.currentText() == "(off)"
        # view drive / view bench: the window's tabs
        hl.command(w.ctl, "view drive")
        assert w.tabs.tabText(w.tabs.currentIndex()) == "Drive"
        hl.command(w.ctl, "view bench")
        assert w.tabs.tabText(w.tabs.currentIndex()) == "Bench"
        # everything stops: the bridge goes STALE, the picture says STALE
        relay.paused.set()
        pump(qapp, 4.0)
        assert tab.bridge.text().startswith("STALE")
        assert tab.panes[0].state.text().startswith("STALE") and tab.panes[0].view.stale
        assert all(tab.servos.item(0, c).foreground().color().name() == "#8c8c8c"
                   for c in (0, 2))                        # greyed when the bridge is silent
        if shots:
            w.grab().save(os.path.join(shots, "bench_tab_stale.png"))
    finally:
        w.close()
    assert _until(lambda: relay.clients == 0)              # closing released the camera
    assert _until(lambda: json.loads(bridge.got[-1]) == {"unsub": 1}, timeout=2)


@needs_lib
def test_without_bench_flags_there_is_no_bench_tab(qapp, tmp_path):
    w = bench_window(qapp, tmp_path)
    try:
        assert "Bench" not in w.tab_widgets
        assert getattr(w.ctl, "bench", None) is None
        assert w.tabs.tabText(w.tabs.currentIndex()) == "Drive"
    finally:
        w.close()


@needs_lib
def test_drive_view_starts_on_drive(qapp, tmp_path):
    w = bench_window(qapp, tmp_path, "--cam-relay", "127.0.0.1:9", "--drive-view")
    try:
        assert "Bench" in w.tab_widgets
        assert w.tabs.tabText(w.tabs.currentIndex()) == "Drive"
        w.select_tab("Bench")
        pump(qapp, 0.3)
        tab = w.tab_widgets["Bench"]
        assert tab.bridge.text() == "no bridge"
        assert tab.panes[0].view.msg == "pick a camera above"
    finally:
        w.close()
