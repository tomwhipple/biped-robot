"""Wire-protocol and watchdog tests.

Run:  .venv/bin/python -m pytest tests/ -q

The Watchdog takes its clock as an argument, so every link-loss case here is
exact and instant -- no sleeps, no flakes.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

from protocol import (CMD_LEN, FLAG_ENABLE, FLAG_ESTOP, RELAX_MS,  # noqa: E402
                      STALE_MS, TLM_LEN, V_MAX, W_MAX, Command, LinkState,
                      ProtocolError, Telemetry, Watchdog, clamp_to_envelope,
                      crc16_ccitt, decode_command, decode_telemetry,
                      encode_command, encode_telemetry)


# -- CRC -------------------------------------------------------------------
def test_crc16_ccitt_false_check_vector():
    # The standard CRC-16/CCITT-FALSE check value, so a C port can be diffed
    # against this exact number rather than against "whatever Python said".
    assert crc16_ccitt(b"123456789") == 0x29B1


# -- command frame ---------------------------------------------------------
def test_command_roundtrip():
    pkt = decode_command(encode_command(7, 0.85, -0.4, FLAG_ENABLE))
    assert pkt.seq == 7
    assert pkt.vx == pytest.approx(0.85, abs=1e-3)
    assert pkt.wz == pytest.approx(-0.4, abs=1e-3)
    assert pkt.enabled and not pkt.estop


def test_command_frame_is_fixed_size():
    assert len(encode_command(0, 0.0, 0.0, 0)) == CMD_LEN
    assert len(encode_command(2**32 - 1, 99.0, -99.0, 0xFF)) == CMD_LEN


def test_milli_quantization_is_within_a_millimetre():
    pkt = decode_command(encode_command(0, 0.6667, 0.3333, FLAG_ENABLE))
    assert pkt.vx == pytest.approx(0.6667, abs=1e-3)
    assert pkt.wz == pytest.approx(0.3333, abs=1e-3)


def test_seq_wraps_not_raises():
    assert decode_command(encode_command(2**32 + 5, 0, 0, 0)).seq == 5


@pytest.mark.parametrize("mutate,msg", [
    (lambda b: b[:-1], "frame"),                    # short
    (lambda b: b + b"\x00", "frame"),               # long
    (lambda b: b"XX" + b[2:], "magic"),             # foreign traffic
    (lambda b: b[:2] + b"\x09" + b[3:], "version"),  # future firmware
    (lambda b: b[:8] + b"\xff\xff" + b[10:], "CRC"),  # corrupted payload
])
def test_bad_frames_rejected(mutate, msg):
    good = encode_command(1, 0.5, 0.0, FLAG_ENABLE)
    with pytest.raises(ProtocolError, match=msg):
        decode_command(mutate(good))


def test_crc_catches_every_single_bit_flip():
    good = encode_command(3, 0.7, -0.2, FLAG_ENABLE)
    for byte in range(len(good) - 2):        # payload bytes only
        for bit in range(8):
            bad = bytearray(good)
            bad[byte] ^= 1 << bit
            with pytest.raises(ProtocolError):
                decode_command(bytes(bad))


# -- telemetry frame -------------------------------------------------------
def test_telemetry_roundtrip():
    t = Telemetry(seq_echo=99, state=LinkState.STAND, vbat_v=11.4, up_z=0.98,
                  vx_est=0.55, wz_est=-0.1, servo_err=0b0000_0100,
                  loop_late_pct=3)
    got = decode_telemetry(encode_telemetry(t))
    assert len(encode_telemetry(t)) == TLM_LEN
    assert got.seq_echo == 99 and got.state is LinkState.STAND
    assert got.vbat_v == pytest.approx(11.4, abs=1e-3)
    assert got.up_z == pytest.approx(0.98, abs=1e-3)
    assert got.servo_err == 0b0000_0100 and got.loop_late_pct == 3


def test_telemetry_rejects_command_frame():
    # Both sockets are on the same host; crossing them must not decode.
    with pytest.raises(ProtocolError):
        decode_telemetry(encode_command(1, 0, 0, 0) + b"\x00" * 6)


# -- training envelope -----------------------------------------------------
def test_clamp_snaps_the_untrained_speed_band_to_stand():
    # walker_env draws v from (0.3, 1.0) or a stand -- (0, 0.3) was never
    # trained, so it must not be passed through as a "slow walk".
    for v in (0.01, 0.1, 0.29):
        assert clamp_to_envelope(v, 0.0)[0] == 0.0


def test_clamp_preserves_trained_turn_in_place():
    # vx=0, wz!=0 IS trained (cmd_w_range), so the snap must keep the yaw.
    assert clamp_to_envelope(0.0, 0.7) == (0.0, 0.7)


def test_clamp_bounds_speed_and_yaw():
    assert clamp_to_envelope(9.0, 9.0) == (V_MAX, W_MAX)
    assert clamp_to_envelope(-9.0, -9.0) == (-V_MAX, -W_MAX)


# -- watchdog --------------------------------------------------------------
def live(seq=1, v=0.8, w=0.0, flags=FLAG_ENABLE):
    return Command(seq=seq, vx=v, wz=w, flags=flags)


def test_stands_before_it_has_ever_heard_anything():
    # Powered up but no commander yet: hold a stand, do not flop.
    dog = Watchdog()
    assert dog.state(0.0) is LinkState.STAND
    assert dog.command(0.0) == (0.0, 0.0)


def test_tracks_a_fresh_command():
    dog = Watchdog()
    assert dog.accept(live(), 0.0)
    assert dog.state(10.0) is LinkState.LIVE
    assert dog.command(10.0) == (0.8, 0.0)


def test_decays_to_stand_then_relax():
    dog = Watchdog()
    dog.accept(live(), 0.0)
    assert dog.state(STALE_MS - 1) is LinkState.LIVE
    assert dog.state(STALE_MS + 1) is LinkState.STAND
    assert dog.command(STALE_MS + 1) == (0.0, 0.0)   # the trained failsafe
    assert dog.state(RELAX_MS + 1) is LinkState.RELAX


def test_a_late_packet_revives_the_link():
    dog = Watchdog()
    dog.accept(live(seq=1), 0.0)
    assert dog.state(STALE_MS + 1) is LinkState.STAND
    dog.accept(live(seq=2), STALE_MS + 2)
    assert dog.state(STALE_MS + 3) is LinkState.LIVE


def test_disabled_command_stands_without_dropping_the_link():
    dog = Watchdog()
    dog.accept(live(flags=0), 0.0)
    assert dog.state(1.0) is LinkState.LIVE      # link is healthy...
    assert dog.command(1.0) == (0.0, 0.0)        # ...the operator said stop


def test_command_is_clamped_on_the_way_in():
    dog = Watchdog()
    dog.accept(live(v=9.0, w=9.0), 0.0)
    assert dog.command(1.0) == (V_MAX, W_MAX)


def test_reordered_packet_is_dropped():
    dog = Watchdog()
    dog.accept(live(seq=5, v=0.8), 0.0)
    assert not dog.accept(live(seq=4, v=0.3), 1.0)
    assert dog.command(1.0) == (0.8, 0.0)        # old command not applied
    assert dog.rejected == 1


def test_reordered_packet_does_not_pet_the_watchdog():
    # The bug this guards: if a stale frame refreshed _last_rx_ms, a burst of
    # reordered packets would hold the robot LIVE on a command from the past.
    dog = Watchdog()
    dog.accept(live(seq=5), 0.0)
    dog.accept(live(seq=4), STALE_MS + 10)
    assert dog.state(STALE_MS + 11) is LinkState.STAND


def test_duplicate_packet_is_dropped():
    dog = Watchdog()
    dog.accept(live(seq=5), 0.0)
    assert not dog.accept(live(seq=5), 1.0)


def test_sender_restart_resyncs():
    # Commander restarted at seq 0; without resync the robot would ignore it
    # forever and stand there looking broken.
    dog = Watchdog()
    dog.accept(live(seq=50_000), 0.0)
    assert dog.accept(live(seq=0, v=0.5), 10.0)
    assert dog.command(11.0) == (0.5, 0.0)


def test_estop_latches_and_needs_a_disabled_command_to_clear():
    dog = Watchdog()
    dog.accept(live(seq=1, flags=FLAG_ESTOP), 0.0)
    assert dog.state(1.0) is LinkState.ESTOP
    # Still stopped even when told to walk -- no instant re-arm.
    dog.accept(live(seq=2, flags=FLAG_ENABLE), 2.0)
    assert dog.state(3.0) is LinkState.ESTOP
    # Pass through neutral to clear.
    dog.accept(live(seq=3, flags=0), 4.0)
    assert dog.state(5.0) is LinkState.LIVE
    dog.accept(live(seq=4, flags=FLAG_ENABLE), 6.0)
    assert dog.command(7.0) == (0.8, 0.0)


def test_estop_outranks_a_dead_link():
    dog = Watchdog()
    dog.accept(live(flags=FLAG_ESTOP), 0.0)
    assert dog.state(RELAX_MS + 1) is LinkState.ESTOP
    assert dog.command(RELAX_MS + 1) == (0.0, 0.0)


def test_rejects_nonsense_timeouts():
    with pytest.raises(ValueError):
        Watchdog(stale_ms=500, relax_ms=100)


def test_end_to_end_encode_decode_accept():
    dog = Watchdog()
    wire = encode_command(1, 0.75, -0.5, FLAG_ENABLE)
    assert dog.accept(decode_command(wire), 0.0)
    v, w = dog.command(0.0)
    assert v == pytest.approx(0.75, abs=1e-3)
    assert w == pytest.approx(-0.5, abs=1e-3)
