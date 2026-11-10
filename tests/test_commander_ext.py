"""commander.py carries the extended command channels (vy, crouch, lift,
foot dx/dz) from any CommandSource into the wire frame.

Run:  .venv/bin/python -m pytest tests/test_commander_ext.py -q

No real socket is ever opened: socket.socket is replaced by a recorder, and
time by a fake clock that ends the stream loop after a set number of seconds.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

import commander                                                 # noqa: E402
import sources                                                   # noqa: E402
from protocol import (CMD_LEN, CMD_LEN_EXT, FLAG_ENABLE, VY_MAX,  # noqa: E402
                      W_MAX, Supervisor, decode_command, encode_command)

RATE = 20.0


class _Done(Exception):
    """Raised by the fake sleep to end commander.stream()."""


class FakeSock:
    sent = []

    def __init__(self, *a, **k):
        pass

    def setsockopt(self, *a):
        pass

    def bind(self, addr):
        assert addr[0] == "0.0.0.0"      # the telemetry listener, local only

    def setblocking(self, flag):
        pass

    def recvfrom(self, n):
        raise BlockingIOError

    def sendto(self, buf, addr):
        assert addr[0] == "127.0.0.1", f"non-loopback send to {addr}"
        FakeSock.sent.append(bytes(buf))

    def close(self):
        pass


def run_stream(monkeypatch, source, seconds):
    """Run commander.stream() for `seconds` of fake time; return the frames
    it sent (excluding any parting stand burst)."""
    FakeSock.sent = []
    now = [0.0]

    class FakeTime:
        @staticmethod
        def monotonic():
            return now[0]

        @staticmethod
        def sleep(s):
            now[0] += 1.0 / RATE
            if now[0] >= seconds:
                raise _Done

    monkeypatch.setattr(commander.socket, "socket", FakeSock)
    monkeypatch.setattr(commander, "time", FakeTime)
    with pytest.raises(_Done):
        commander.stream("127.0.0.1", source, send_hz=RATE, quiet=True)
    return list(FakeSock.sent)


def test_script_with_extras_reaches_the_wire(monkeypatch):
    # sidestep1 asks for vy +0.2 during 2.0 s .. 2.67 s
    src = sources.ScriptSource("sidestep1")
    frames = run_stream(monkeypatch, src, 4.0)
    pkts = [decode_command(f) for f in frames]
    assert any(p.vy == pytest.approx(0.2) for p in pkts), \
        "no frame carried vy: the lateral channel never left commander.py"
