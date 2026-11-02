"""The Wi-Fi CLI client (link/wcliproto.py, tools/wcli.py, tools/wcli_bridge.py).

Run:  .venv/bin/python -m pytest tests/test_wcli.py -q

The wire format is pinned to the C++ by generated vectors (firmware/host/
test_wcli.cpp); here the Python client is driven END TO END against the
robot's real session logic: firmware/host/build/wcli_twin runs
components/wcli (auth, replay protection, at-most-once, the allowlist) on a
localhost UDP socket with a fake CLI, optionally dropping a fraction of the
datagrams in both directions. Those tests skip with a reason when the twin is
not built (`make -C firmware/host build/wcli_twin`, or `make host`).
"""
import os
import re
import socket
import stat
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "link"))

import wcliproto as W  # noqa: E402

TWIN = os.path.join(ROOT, "firmware", "host", "build", "wcli_twin")
WCLI = os.path.join(ROOT, "tools", "wcli.py")
BRIDGE = os.path.join(ROOT, "tools", "wcli_bridge.py")
STAMP = r"\d\d:\d\d:\d\d\.\d{3}Z"

needs_twin = pytest.mark.skipif(
    not os.access(TWIN, os.X_OK),
    reason="firmware/host/build/wcli_twin not built (make -C firmware/host "
           "build/wcli_twin)")


# -- wire --------------------------------------------------------------------------
KEY = bytes(range(32))


def test_roundtrip_and_direction_keys():
    k = W.derive_keys(KEY)
    m = W.Msg(W.T_CMD, sid=0x1122334455667788, seq=5, payload=b"ping")
    buf = W.encode(m, k.c2r)
    assert len(buf) == W.OVERHEAD + 4
    d = W.decode(buf, k.c2r)
    assert (d.type, d.sid, d.seq, d.payload) == (W.T_CMD, m.sid, 5, b"ping")
    with pytest.raises(W.WireError, match="tag"):
        W.decode(buf, k.r2c)          # a reflected datagram is not authentic
    for i in range(len(buf)):
        bad = bytearray(buf)
        bad[i] ^= 0x01
        with pytest.raises(W.WireError):
            W.decode(bytes(bad), k.c2r)
    with pytest.raises(W.WireError):
        W.decode(buf[:-1], k.c2r)
    with pytest.raises(W.WireError):
        W.decode(buf + b"\0", k.c2r)
    with pytest.raises(ValueError):
        W.encode(W.Msg(W.T_OUT, payload=b"x" * (W.MAX_CHUNK + 1)), k.r2c)


def test_key_file_rules(tmp_path):
    p = tmp_path / "bimo" / "wcli.key"
    key = W.new_key_file(str(p))
    assert stat.S_IMODE(os.stat(p).st_mode) == 0o600
    assert W.load_key(str(p)) == key
    with pytest.raises(FileExistsError):
        W.new_key_file(str(p))        # never overwrites a key
    os.chmod(p, 0o644)
    with pytest.raises(PermissionError):
        W.load_key(str(p))            # readable by others: refused
    with pytest.raises(ValueError):
        W.parse_key_hex("abcd")
    assert W.parse_key_hex(" " + KEY.hex().upper() + "\n") == KEY
    # The robot's `wcli` prints SHA-256(key)[:4]: 8 hex digits, not the key.
    fp = W.key_fingerprint(KEY)
    assert re.fullmatch(r"[0-9a-f]{8}", fp) and fp not in KEY.hex()


def test_line_splitter():
    s = W.LineSplitter()
    assert s.feed("\r\nbimo fir") == [""]
    assert s.feed("mware\r\n  a\r\nb") == ["bimo firmware", "  a"]
    assert s.feed("\r\n") == ["b"]


# -- end to end against the twin -----------------------------------------------------
class Twin:
    def __init__(self, key: bytes, *args, port: int = 0):
        self.key = key
        self.args = args
        self.proc = subprocess.Popen(
            [TWIN, "--key-hex", key.hex(), "--port", str(port), *args],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        line = self.proc.stdout.readline()
        assert line.startswith("listening "), line
        self.port = int(line.split()[1])

    def stop(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)

    def client(self, key=None, **kw):
        return W.Client("127.0.0.1", key or self.key, self.port, **kw)


@pytest.fixture
def twin():
    t = Twin(os.urandom(32))
    yield t
    t.stop()


@pytest.fixture
def lossy_twin():
    t = Twin(os.urandom(32), "--loss", "0.3", "--seed", "11")
    yield t
    t.stop()


@needs_twin
def test_command_reply_is_the_cli_text(twin):
    c = twin.client()
    c.connect()
    r = c.command("ping A")
    assert (r.status, r.text) == ("done", "pong A #1\r\n")
    r = c.command("help")
    assert r.status == "done"
    assert r.text.startswith("\r\nbimo firmware v1 -- bench mode")
    r = c.command("frobnicate")
    assert r.status == "refused" and "not on the Wi-Fi allowlist" in r.text


@needs_twin
@pytest.mark.parametrize("line, why", [
    ("id 1 2", "servo EEPROM"),
    ("middle 7", "servo EEPROM"),
    ("cal save", "calibration"),
    ("gains 7 40 32", "servo EEPROM"),
    ("imu bias", "NVS"),
    ("wifi net psk", "credentials"),
    ("wcli key " + "00" * 32, "key"),
    ("batt reset", "pack-guard latch"),
    ("run", "ARM edge"),
])
def test_persistent_writers_are_refused(twin, line, why):
    c = twin.client()
    r = c.command(line)
    assert r.status == "refused" and r.refused
    assert r.text.startswith("refused over Wi-Fi (tether only): ")
    assert why in r.text and r.text.endswith("\r\n")


@needs_twin
def test_armed_is_status_only():
    t = Twin(os.urandom(32), "--armed")
    try:
        c = t.client()
        r = c.command("ping A")
        assert r.status == "refused" and "armed" in r.text
        assert c.command("stat").status == "done"
    finally:
        t.stop()


@needs_twin
def test_wrong_key_gets_no_session(twin):
    c = twin.client(key=os.urandom(32))
    t0 = time.monotonic()
    with pytest.raises(W.LinkDown):
        c.connect(timeout=1.0)
    assert time.monotonic() - t0 < 2.0


@needs_twin
def test_replayed_command_is_not_executed_again(twin):
    c = twin.client()
    c.connect()
    assert c.command("ping R").text == "pong R #1\r\n"
    # Byte for byte the datagram the client sent (the encoding is
    # deterministic), as a sniffer on the LAN would have captured it.
    cmd = W.encode(W.Msg(W.T_CMD, c.sid, c.seq, payload=b"ping R"), c.keys.c2r)
    real = c.sock.sendto
    # The exact datagram again, from the same address (a replay with the
    # client's source) and from another socket (a replay from elsewhere).
    for _ in range(3):
        real(cmd, c.addr)
    other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    other.sendto(cmd, c.addr)
    other.close()
    time.sleep(0.2)
    assert c.command("ping R").text == "pong R #2\r\n"   # ran once each time


@needs_twin
def test_at_most_once_under_packet_loss(lossy_twin):
    c = lossy_twin.client()
    c.connect(timeout=10)
    for i in range(10):
        r = c.command(f"ping L{i}")
        assert (r.status, r.text) == ("done", f"pong L{i} #1\r\n"), i
    r = c.command("ping L0")
    assert r.text == "pong L0 #2\r\n"


@needs_twin
def test_long_reply_survives_packet_loss(lossy_twin):
    c = lossy_twin.client()
    c.connect(timeout=10)
    r = c.command("trace 600")
    assert r.status == "done" and r.lost_chunks == 0
    lines = r.text.splitlines()
    assert [int(x.split()[1]) for x in lines] == list(range(600))


@needs_twin
def test_async_output_after_the_reply(twin):
    got = []
    c = twin.client(on_text=got.append)
    r = c.command("pose 2048 s300")
    assert r.text == "pose: streaming\r\n"
    c.idle(1.0)
    assert "".join(got) == "pose: streaming\r\npose: done\r\n"


@needs_twin
def test_robot_reboot_mid_session_is_reported_not_resent(twin):
    c = twin.client()
    assert c.command("ping B").text == "pong B #1\r\n"
    port = twin.port
    twin.stop()
    restarted = Twin(twin.key, port=port)      # same address, fresh boot
    try:
        r = c.command("ping B")
        assert r.status == "unknown"            # old session: STALE, not run
        r = c.command("ping B")                 # reconnects
        assert (r.status, r.text) == ("done", "pong B #1\r\n")
    finally:
        restarted.stop()
        twin.proc = restarted.proc              # fixture teardown: already stopped


# -- the tools ------------------------------------------------------------------------------
def _key_file(tmp_path, key):
    p = tmp_path / "wcli.key"
    p.write_text(key.hex() + "\n")
    os.chmod(p, 0o600)
    return str(p)


@needs_twin
def test_wcli_tool_exit_codes(twin, tmp_path):
    kf = _key_file(tmp_path, twin.key)
    base = [sys.executable, WCLI, "--host", "127.0.0.1", "--port",
            str(twin.port), "--key-file", kf]
    r = subprocess.run(base + ["ping", "T"], capture_output=True, text=True,
                       timeout=30)
    assert (r.returncode, r.stdout) == (0, "pong T #1\n")
    r = subprocess.run(base + ["id 1 2"], capture_output=True, text=True,
                       timeout=30)
    assert r.returncode == 2 and r.stdout.startswith("refused over Wi-Fi")
    r = subprocess.run(base, input="ping P\nping P\n", capture_output=True,
                       text=True, timeout=30)
    assert (r.returncode, r.stdout) == (0, "pong P #1\npong P #2\n")
    os.chmod(kf, 0o640)
    r = subprocess.run(base + ["ping"], capture_output=True, text=True,
                       timeout=30)
    assert r.returncode == 1 and "chmod 600" in r.stderr


def _run_bridge(tmp_path, port, key, lines, extra=()):
    kf = _key_file(tmp_path, key)
    fifo, log = str(tmp_path / "board.fifo"), str(tmp_path / "board.log")
    p = subprocess.Popen([sys.executable, BRIDGE, fifo, log, "--host",
                          "127.0.0.1", "--port", str(port), "--key-file", kf,
                          *extra], stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True)
    t0 = time.monotonic()
    while not os.path.exists(fifo):
        assert time.monotonic() - t0 < 10
        time.sleep(0.05)
    for line in lines:
        with open(fifo, "w") as f:      # one writer per line, like `echo >`
            f.write(line + "\n")
        time.sleep(0.05)
    with open(fifo, "w") as f:
        f.write("__quit__\n")
    p.wait(timeout=30)
    return p.returncode, open(log).read().splitlines()


@needs_twin
def test_bridge_writes_the_tether_log_format(twin, tmp_path):
    rc, log = _run_bridge(tmp_path, twin.port, twin.key,
                          ["ping A", "id 1 2", "pose 1 s100"])
    assert rc == 0
    body = [re.sub(f"^{STAMP}", "T", ln) for ln in log]
    for ln in log:
        assert re.match(f"^{STAMP} ", ln), ln
    assert body[0].startswith("T # wcli bridge -> 127.0.0.1:")
    assert "T # session up" in body
    i = body.index("T > ping A")
    assert body[i + 1] == "T   pong A #1"
    j = body.index("T > id 1 2")
    assert body[j + 1].startswith("T   refused over Wi-Fi (tether only): ")
    k = body.index("T > pose 1 s100")
    assert body[k + 1] == "T   pose: streaming"
    assert "T   pose: done" in body[k + 2:]     # async, before the quit
    assert body[-1] == "T # bridge closed"


@needs_twin
def test_bridge_drops_rather_than_queues_when_the_link_is_down(tmp_path):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    dead_port = s.getsockname()[1]
    s.close()                           # nothing listens there now
    rc, log = _run_bridge(tmp_path, dead_port, os.urandom(32), ["release"])
    assert rc == 0
    body = [re.sub(f"^{STAMP} ", "", ln) for ln in log]
    assert any(b.startswith("# link down") for b in body)
    assert "# WRITE FAILED (link down): release" in body
    assert "> release" not in body
