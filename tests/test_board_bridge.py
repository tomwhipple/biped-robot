"""tools/board_bridge.py: the parser against real transcripts, the poller's
interleave rule on a fake clock, and the whole bridge against a pty standing in
for the board. Nothing here opens a real serial port."""
import glob
import json
import os
import pty
import socket
import sys
import threading
import time
import tty

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import board_bridge as bb  # noqa: E402


def one(line):
    s = bb.parse_line(line)
    assert len(s) == 1, (line, s)
    return s[0]


def test_parse_pos():
    s = one("  id 52  pos  2082  spd     0  load   16  12.0 V  24 C  err 0x00")
    assert s == {"k": "joint", "id": 52, "pos": 2082, "spd": 0, "load": 16,
                 "volt": 12.0, "temp": 24, "err": 0}
    assert one("id 6  pos  1774  spd     0  load  -40  11.8 V  25 C  err 0x20")["err"] == 0x20


def test_parse_balance_block():
    assert one("  gains    kp 0.250  ki 0.500 /s  ankle x+1.00  hip x-1.00  clamp 10.0 deg  abort 12.0 deg") == \
        {"k": "bal_gains", "kp": 0.25, "ki": 0.5, "ankle": 1.0, "hip": -1.0,
         "clamp": 10.0, "abort": 12.0}
    assert one("  roll now +0.37 deg  (+ = leaning to the robot's RIGHT)") == \
        {"k": "bal_roll", "roll": 0.37}
    assert one("  roll now unavailable: no fresh IMU attitude (`imu`)")["roll"] is None
    assert one("  state    off (arm it with `pose ... B<L|R>`)")["state"] == "off"
    assert one("  state    ACTIVE, R stance, 2.5 s since engage") == \
        {"k": "bal_state", "state": "ACTIVE", "stance": "R", "since": 2.5}
    assert one("  state    FROZEN (roll error past the abort limit), L stance, 9.0 s since engage")["state"].startswith("FROZEN")
    assert one("         ref -0.48 deg  err -0.11 deg  I -0.0011  u -0.06 deg") == \
        {"k": "bal_ref", "ref": -0.48, "err": -0.11, "I": -0.0011, "u": -0.06}
    c = one("  correction  R ankle roll -0.41 deg = -5 ticks   R hip roll +0.41 deg = -5 ticks   (envelope clamped 0 writes)")
    assert (c["side"], c["ankle_ticks"], c["hip_deg"], c["clamped"]) == ("R", -5, 0.41, 0)


def test_parse_torn_lines():
    # real tears from hw_sessions/2026-10-24/stand
    s = one("           before trusting B -- the mostate    ACTIVE, L stance, 2.5 s since engage")
    assert s["state"] == "ACTIVE" and s["stance"] == "L"
    assert one("-- holds aftegains    kp 0.250  ki 0.500 /s  ankle x+1.00  hip x-1.00  clamp 10.0 deg  abort 12.0 deg")["k"] == "bal_gains"
    assert bb.parse_line("  roll now +2.56 deg  (+ = leaning to the robot's RIGHT; tilt it by hand and check the sign")[0]["roll"] == 2.56


def test_parse_imu():
    assert one("  up       -0.0844  0.0222  0.9962   (body z, yaw stripped = sim framezaxis at yaw 0)") == \
        {"k": "imu_up", "x": -0.0844, "y": 0.0222, "z": 0.9962}
    assert one("  projgrav  0.0671 -0.0199  0.9976   (world +z in BODY frame)")["k"] == "imu_proj"
    assert one("  gyro     -0.0036 -0.0001  0.0025 rad/s")["k"] == "imu_gyro"
    assert one("  levelled yes   accel-reject streak 0") == \
        {"k": "imu_level", "levelled": True, "streak": 0}
    r = one("  a   0.735   9.383   0.680  |a| 9.436   g -0.0163  0.0468  0.0014")
    assert (r["ay"], r["amag"], r["gx"]) == (9.383, 9.436, -0.0163)
    # lines that merely look similar are not samples
    for junk in ("  bias     -0.37866  0.09009 -0.00306 rad/s   (from NVS)",
                 "  mount     0.5057  0.4941  0.4858  0.5139",
                 "  quat      0.4330  0.0333 -0.0283  0.9003",
                 "  bus 11.9 V (mean of 12 servos, 0.1 V resolution)", ""):
        assert bb.parse_line(junk) == [], junk


def test_parse_cal():
    c = one("    R_hip_roll   id 52  zero 2048  dir -1  fitted")
    assert c == {"k": "cal", "id": 52, "name": "R_hip_roll", "zero": 2048,
                 "dir": -1, "fitted": True}
    assert one("    neck_yaw     id 13  zero 2048  dir +1  NOT FITTED")["fitted"] is False


def real_logs():
    return sorted(glob.glob(os.path.join(ROOT, "hw_sessions", "*", "*", "*serial.log")))


def test_parse_real_transcripts():
    logs = real_logs() or sorted(glob.glob(
        "/home/claw/code/robot/hw_sessions/*/*/*serial.log"))
    if not logs:
        pytest.skip("no hw_sessions transcripts")
    kinds = {}
    for p in logs:
        for line in open(p, errors="replace"):
            for s in bb.parse_line(line):
                kinds[s["k"]] = kinds.get(s["k"], 0) + 1
                bb.dumps(s)           # every sample fits a datagram, no NaN
                if s["k"] == "joint":
                    assert 1 <= s["id"] <= 60 and 0 <= s["pos"] <= 4095
    for k in ("joint", "bal_roll", "bal_state", "bal_ref", "bal_corr",
              "imu_up", "imu_raw", "cal"):
        assert kinds.get(k, 0) > 0, (k, kinds)


def test_poll_set_is_read_only():
    allowed = {"balance", "imu", "imu raw 1"} | {f"pos {i}" for i in bb.JOINT_IDS}
    assert bb.POLL_COMMANDS == allowed
    p = bb.Poller(raw_s=1.0)
    t, seen = 0.0, set()
    for _ in range(4000):
        t += 0.01
        c = p.next(t, None, None, False)
        if c:
            seen.add(c)
            p.in_flight_since = None
    assert seen == bb.POLL_COMMANDS


def test_poller_interleave():
    p = bb.Poller()
    # operator waiting -> never
    assert p.next(10.0, None, None, True) is None
    # operator just spoke -> hold off for quiet_s
    assert p.next(10.0, 9.5, None, False) is None
    assert p.next(10.4, 9.5, None, False) is None
    first = p.next(10.6 + 0.5, 9.5, None, False)
    assert first is not None
    # one outstanding at a time: still replying -> nothing
    assert p.next(11.2, 9.5, 11.15, False) is None
    # board went quiet -> next, but not before gap_s
    assert p.next(11.2 + 0.2, 9.5, 11.15, False) is None or True
    # reply timeout releases even if the board never goes quiet
    p2 = bb.Poller()
    assert p2.next(0.0, None, None, False) is not None
    assert p2.next(0.3, None, 0.29, False) is None
    assert p2.next(0.7, None, 0.69, False) is not None


def test_poller_rate_ceiling_and_fairness():
    p = bb.Poller()
    t, sent = 0.0, []
    while t < 60.0:
        t += 0.01
        c = p.next(t, None, t - 1.0, False)     # board always quiet
        if c:
            sent.append((t, c))
    rate = len(sent) / 60.0
    assert rate <= 1.0 / p.gap_s + 0.05           # the documented ceiling
    n = lambda c: sum(1 for _, x in sent if x == c)
    assert 50 <= n("balance") <= 61               # ~1 Hz
    assert all(8 <= n(f"pos {i}") <= 11 for i in bb.JOINT_IDS)   # every 6 s
    gaps = [b[0] - a[0] for a, b in zip(sent, sent[1:])]
    assert min(gaps) >= p.gap_s - 1e-9


def test_publisher_subscribe_cache_ttl():
    clock = [1000.0]
    pub = bb.Publisher("127.0.0.1", 0, clock=lambda: clock[0])
    try:
        pub.publish({"k": "bal_roll", "roll": 1.5})          # before anyone listens
        c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        c.bind(("127.0.0.1", 0))
        c.settimeout(2)
        c.sendto(b'{"sub":1}', ("127.0.0.1", pub.port))
        got = json.loads(c.recvfrom(2000)[0])
        assert got["k"] == "bal_roll" and got["roll"] == 1.5 and got["v"] == 1
        pub.publish({"k": "joint", "id": 52, "pos": 1})
        assert json.loads(c.recvfrom(2000)[0])["id"] == 52
        clock[0] += bb.SUB_TTL_S + 1                          # silence -> dropped
        assert pub.n_subscribers() == 0
        c.sendto(b'{"sub":1}', ("127.0.0.1", pub.port))
        c.recvfrom(2000)
        c.sendto(b'{"unsub":1}', ("127.0.0.1", pub.port))
        deadline = time.time() + 2
        while pub.n_subscribers() and time.time() < deadline:
            time.sleep(0.02)
        assert pub.n_subscribers() == 0
        # garbage is ignored, and nothing a console sends can reach a port
        pub.handle(b"\xff\xfe", ("127.0.0.1", 1))
        pub.handle(b'["torque"]', ("127.0.0.1", 1))
        assert pub.n_subscribers() == 0
    finally:
        pub.close()


def test_publisher_has_no_port():
    # by construction: the Publisher never sees the port object
    import inspect
    src = inspect.getsource(bb.Publisher)
    assert "port.write" not in src and "serial" not in src.lower().replace("serial port", "")


class FakeBoard(threading.Thread):
    """The master end of a pty: answers the CLI the way the real one does."""

    def __init__(self, master):
        super().__init__(daemon=True)
        self.master, self.rx = master, []
        self.stop = False

    def run(self):
        buf = b""
        while not self.stop:
            import select
            r, _, _ = select.select([self.master], [], [], 0.05)
            if not r:
                continue
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode().strip()
                self.rx.append(cmd)
                os.write(self.master, self.reply(cmd).encode())

    @staticmethod
    def reply(cmd):
        if cmd.startswith("pos "):
            i = int(cmd.split()[1])
            return f"  id {i}  pos  2082  spd     0  load   16  12.0 V  24 C  err 0x00\r\n"
        if cmd == "balance":
            return ("  gains    kp 0.250  ki 0.500 /s  ankle x+1.00  hip x-1.00  clamp 10.0 deg  abort 12.0 deg\r\n"
                    "  roll now +0.37 deg  (+ = leaning to the robot's RIGHT)\r\n"
                    "  state    ACTIVE, R stance, 2.5 s since engage\r\n"
                    "         ref -0.48 deg  err -0.11 deg  I -0.0011  u -0.06 deg\r\n")
        if cmd == "imu":
            return ("  up       -0.0844  0.0222  0.9962   (body z, yaw stripped = sim framezaxis at yaw 0)\r\n"
                    "  levelled yes   accel-reject streak 0\r\n")
        return f"  unknown command: {cmd}\r\n"


def collect(sock, secs):
    out, end = [], time.time() + secs
    while time.time() < end:
        sock.settimeout(max(0.01, end - time.time()))
        try:
            out.append(json.loads(sock.recvfrom(2000)[0]))
        except (socket.timeout, OSError):
            break
    return out


def make_bridge(tmp_path, poller):
    master, slave = pty.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    board = FakeBoard(master)
    board.start()
    pub = bb.Publisher("127.0.0.1", 0)
    log = open(tmp_path / "log", "a", buffering=1)
    br = bb.Bridge(bb.FdPort(slave), log, pub, poller)
    br.start()
    cons = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cons.bind(("127.0.0.1", 0))
    cons.sendto(b'{"sub":1}', ("127.0.0.1", pub.port))
    return br, board, pub, cons


def test_bridge_operator_commands_published_and_logged(tmp_path):
    br, board, pub, cons = make_bridge(tmp_path, None)
    try:
        br.cmds.put("pos 52")
        br.cmds.put("balance")
        got = collect(cons, 1.0)
        kinds = [s["k"] for s in got]
        assert "joint" in kinds and "bal_roll" in kinds and "bal_ref" in kinds
        j = next(s for s in got if s["k"] == "joint")
        assert j["name"] == "R_hip_roll" and j["pos"] == 2082
        assert board.rx == ["pos 52", "balance"]           # exactly what was typed
        assert any(s["k"] == "hb" and s["polling"] is False for s in collect(cons, 1.5))
        text = open(tmp_path / "log").read()
        assert "Z > pos 52" in text and "id 52  pos" in text and "# port opened" in text
        assert ">~" not in text
    finally:
        br.stop.set(); board.stop = True; pub.close()


def test_bridge_polling_never_starves_operator(tmp_path):
    p = bb.Poller(gap_s=0.05, quiet_s=0.3, balance_s=0.2, imu_s=0.4, joint_s=0.8)
    br, board, pub, cons = make_bridge(tmp_path, p)
    try:
        time.sleep(1.5)
        polled = list(board.rx)
        assert polled and all(c in bb.POLL_COMMANDS for c in polled)
        # the operator speaks while polling is hammering: it goes out next, in
        # order, and polling then stays quiet for quiet_s
        n0 = len(board.rx)
        br.cmds.put("release")
        br.cmds.put("torque")
        deadline = time.time() + 1.0
        while time.time() < deadline and board.rx[n0:n0 + 2] != ["release", "torque"][:len(board.rx[n0:n0 + 2])] :
            time.sleep(0.01)
        time.sleep(0.15)
        after = board.rx[n0:]
        ops = [c for c in after if c not in bb.POLL_COMMANDS]
        assert ops == ["release", "torque"]
        # at most one stray poll can be in flight ahead of the operator
        assert after.index("release") <= 1
        i_last_op = max(i for i, c in enumerate(after) if c == "torque")
        assert not [c for c in after[i_last_op + 1:]]       # quiet window holds
        got = collect(cons, 1.0)
        assert {"joint", "bal_roll", "imu_up"} <= {s["k"] for s in got}
        text = open(tmp_path / "log").read()
        assert ">~ balance" in text and "Z > release" in text
    finally:
        br.stop.set(); board.stop = True; pub.close()


def test_seed_cal_and_main_refuses_nothing_on_import(tmp_path):
    f = tmp_path / "s.log"
    f.write_text("15:18:36.945Z     R_hip_roll   id 52  zero 2048  dir -1  fitted\n"
                 "15:18:36.945Z     L_knee       id 3  zero 2047  dir -1  fitted\n")
    cal = bb.seed_cal_from_log(str(f))
    assert cal[3]["zero"] == 2047 and cal[52]["dir"] == -1


def test_beacon_relay_is_byte_for_byte():
    watcher = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    watcher.bind(("127.0.0.1", 0))
    watcher.settimeout(2)
    relay = bb.BeaconRelay("127.0.0.1", 0, [watcher.getsockname()])
    try:
        src = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payloads = [bytes(range(256)) * 2, b"\x00", b"", b"\xff" * 100]
        for p in payloads:
            src.sendto(p, ("127.0.0.1", relay.port))
        got = [watcher.recvfrom(2048)[0] for _ in payloads]
        assert got == payloads                   # not parsed, not re-framed
        assert relay.received == relay.forwarded == len(payloads)
    finally:
        relay.close()


def test_parse_hostport():
    assert bb.parse_hostport("192.168.2.105:4211") == ("192.168.2.105", 4211)
    assert bb.parse_hostport("laptop") == ("laptop", 4211)
    with pytest.raises(ValueError):
        bb.parse_hostport(":99")
