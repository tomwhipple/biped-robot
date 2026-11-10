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
    tick = [0]    # integer ticks: t is exactly i / RATE, no float drift

    class FakeTime:
        @staticmethod
        def monotonic():
            return tick[0] / RATE

        @staticmethod
        def sleep(s):
            tick[0] += 1
            if tick[0] / RATE >= seconds:
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


def test_script_with_extras_sends_a_24_byte_frame_only_while_off_default(
        monkeypatch):
    frames = run_stream(monkeypatch, sources.ScriptSource("sidestep1"), 4.0)
    for f in frames:
        pkt = decode_command(f)
        assert len(f) == (CMD_LEN_EXT if pkt.vy else CMD_LEN)
    assert {len(f) for f in frames} == {CMD_LEN, CMD_LEN_EXT}


@pytest.mark.parametrize("name", sorted(sources.SCRIPTS))
def test_every_script_runs_through_the_commander_loop(monkeypatch, name):
    """Each script, for 20 s (past the end of every one of them), through
    stream(): no error, every frame decodes, and the wire carries exactly what
    the script asked for."""
    src = sources.ScriptSource(name)
    frames = run_stream(monkeypatch, src, 20.0)
    assert len(frames) == int(20.0 * RATE)
    ref = sources.ScriptSource(name)
    for i, f in enumerate(frames):
        vx, wz, flags, ext = ref.poll(i / RATE)
        want = encode_command(i, vx, wz, flags, **ext)
        assert f == want
        decode_command(f)
        # classic frame exactly when no extra is off its default
        assert (len(f) == CMD_LEN_EXT) == (len(want) == CMD_LEN_EXT)


def test_scripts_without_extras_are_the_classic_frame_on_the_wire(
        monkeypatch):
    # Byte-for-byte the pre-extras behaviour: a 14 B frame from
    # encode_command(seq, vx, wz, flags) and nothing else.
    frames = run_stream(monkeypatch, sources.ScriptSource("walk"), 2.0)
    assert frames[0] == encode_command(0, 0.6, 0.0, FLAG_ENABLE)
    assert all(len(f) == CMD_LEN for f in frames)


# -- gamepad ---------------------------------------------------------------
class FakeJoystick:
    def __init__(self, axes, buttons=()):
        self.axes, self.buttons = dict(axes), set(buttons)

    def get_axis(self, i):
        return self.axes.get(i, 0.0)

    def get_button(self, i):
        return int(i in self.buttons)


class FakePygame:
    class event:
        @staticmethod
        def pump():
            pass


def pad(axes, held=True):
    g = sources.GamepadSource.__new__(sources.GamepadSource)
    g._pygame = FakePygame
    g._js = FakeJoystick(axes, {sources.GamepadSource.BTN_DEADMAN}
                         if held else ())
    return g


G = sources.GamepadSource


def test_gamepad_three_axes_reach_the_wire_together():
    # left stick up-left, right stick left: forward, +vy (left), +wz (left)
    g = pad({G.AX_FWD: -1.0, G.AX_LAT: -1.0, G.AX_YAW: -1.0})
    vx, wz, flags, ext = g.poll(0.0)
    assert (vx, wz, flags, ext) == (pytest.approx(1.0), pytest.approx(W_MAX),
                                    FLAG_ENABLE,
                                    {"vy": pytest.approx(VY_MAX)})
    frame = encode_command(0, vx, wz, flags, **ext)
    assert len(frame) == CMD_LEN_EXT
    pkt = decode_command(frame)
    assert (pkt.vx, pkt.vy, pkt.wz) == (pytest.approx(1.0),
                                        pytest.approx(VY_MAX),
                                        pytest.approx(W_MAX))
    # the right and the opposite deflection flip every sign
    g = pad({G.AX_FWD: 1.0, G.AX_LAT: 1.0, G.AX_YAW: 1.0})
    vx, wz, flags, ext = g.poll(0.0)
    assert vx < 0 and wz < 0 and ext["vy"] == pytest.approx(-VY_MAX)


def test_gamepad_lateral_deadzone_and_clamp():
    assert pad({G.AX_LAT: 0.1}).poll(0.0)[3] == {}
    assert len(encode_command(0, *pad({G.AX_LAT: -0.1}).poll(0.0)[:3])) \
        == CMD_LEN
    for a in (-1.0, -0.5, 0.5, 1.0):
        ext = pad({G.AX_LAT: a}).poll(0.0)[3]
        assert abs(ext["vy"]) <= VY_MAX + 1e-9


def test_gamepad_deadman_and_estop_send_no_extras():
    assert pad({G.AX_LAT: -1.0}, held=False).poll(0.0) == (0.0, 0.0, 0, {})
    g = pad({G.AX_LAT: -1.0})
    g._js.buttons.add(G.BTN_ESTOP)
    assert g.poll(0.0)[3] == {}


def test_gamepad_through_commander_loop(monkeypatch):
    g = pad({G.AX_FWD: -1.0, G.AX_LAT: -1.0, G.AX_YAW: -1.0})
    g.close = lambda: None
    frames = run_stream(monkeypatch, g, 1.0)
    assert frames
    for f in frames:
        pkt = decode_command(f)
        assert len(f) == CMD_LEN_EXT
        assert (pkt.vx, pkt.vy, pkt.wz) == (pytest.approx(1.0),
                                            pytest.approx(VY_MAX),
                                            pytest.approx(W_MAX))


# -- the command vector the policy sees ------------------------------------
def test_vx_vy_wz_arrive_together_at_the_supervisor():
    """Reference Supervisor (the one link_twin.py and the firmware port run):
    a sidestep-with-turn frame ends up in command_ext() as
    (vx, vy, wz, crouch, lift, foot_dx, foot_dz)."""
    sup = Supervisor(armed=True)
    g = pad({G.AX_FWD: -1.0, G.AX_LAT: -1.0, G.AX_YAW: -1.0})
    vx, wz, flags, ext = g.poll(0.0)
    pkt = decode_command(encode_command(1, vx, wz, flags, **ext))
    assert sup.accept(pkt, 1000.0) is not None
    cmd = sup.command_ext(1010.0)
    assert cmd[:3] == (pytest.approx(1.0), pytest.approx(VY_MAX),
                       pytest.approx(W_MAX))
    assert cmd[3:] == (1.0, 0.0, 0.0, 0.0)
