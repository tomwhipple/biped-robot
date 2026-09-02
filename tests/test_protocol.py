"""Wire-protocol and watchdog tests.

Run:  .venv/bin/python -m pytest tests/ -q

The Watchdog takes its clock as an argument, so every link-loss case here is
exact and instant -- no sleeps, no flakes.
"""
import os
import sys

import pytest
import struct

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

from protocol import (CMD_LEN, DIAG_ARM_SHIFT, DIAG_CAL_OK,  # noqa: E402
                      DIAG_RUN, FLAG_ARM, FLAG_ENABLE, FLAG_ESTOP, FLAG_HOME,
                      FLAG_POSE,
                      NUM_JOINTS, RELAX_MS,
                      STALE_MS, TLM_LEN, TLM_LEN_EXT, TLM_LEN_EXT_V1,
                      TLM_LEN_V1, V_MAX, W_MAX,
                      ArmLatch, ArmResult, HomeLatch, is_home_result,
                      Command, LinkState, ProtocolError, Supervisor,
                      Telemetry, Watchdog, clamp_to_envelope, crc16_ccitt,
                      decode_command, decode_telemetry, diag_arm_result,
                      diag_reason, encode_command, encode_telemetry,
                      pack_diag)


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


def test_telemetry_every_state_roundtrips():
    # The wire encoding is LinkState declaration order; appending a state
    # (VLAND, VSAFE, FALLEN...) must round-trip and never renumber the rest.
    for i, st in enumerate(LinkState):
        t = Telemetry(seq_echo=i, state=st, vbat_v=11.1, up_z=0.2,
                      vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
        assert decode_telemetry(encode_telemetry(t)).state is st
    assert list(LinkState)[6] is LinkState.FALLEN   # frozen wire value


def test_telemetry_rejects_command_frame():
    # Both sockets are on the same host; crossing them must not decode.
    with pytest.raises(ProtocolError):
        decode_telemetry(encode_command(1, 0, 0, 0) + b"\x00" * 6)


# -- timestamps (2026-09-02, docs/control-channel.md "Time on the wire") ---
T_REAL = 1788390475605401       # 2026-09-02T22:27:55.605401Z, a real reading


def _legacy(t):
    """What a robot flashed before the stamp beacons: body [+ joints] + CRC.

    protocol.py cannot emit this any more, so the test packs it by hand from
    the stamped frame -- which is also the claim being made: the legacy
    frame IS the stamped frame with 8 bytes cut out of the middle."""
    wire = encode_telemetry(t)
    body = wire[:18] + wire[26:-2]
    return body + struct.pack("<H", crc16_ccitt(body))


def test_timestamp_round_trips_on_both_lengths():
    t = Telemetry(seq_echo=1, state=LinkState.LIVE, vbat_v=11.4, up_z=1.0,
                  vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0,
                  t_us=T_REAL)
    wire = encode_telemetry(t)
    assert len(wire) == TLM_LEN == 28
    assert struct.unpack("<Q", wire[18:26]) == (T_REAL,)     # LE, after body
    got = decode_telemetry(wire)
    assert got.t_us == T_REAL
    assert got.t_utc == pytest.approx(1788390475.605401, abs=1e-6)
    long = encode_telemetry(t._replace(joints=(0.1,) * NUM_JOINTS))
    assert len(long) == TLM_LEN_EXT == 48
    assert decode_telemetry(long).t_us == T_REAL
    assert decode_telemetry(long).joints == pytest.approx((0.1,) * NUM_JOINTS)


def test_zero_means_unsynced():
    t = Telemetry(seq_echo=1, state=LinkState.BENCH, vbat_v=11.4, up_z=0.0,
                  vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
    assert t.t_us == 0 and t.t_utc is None
    assert decode_telemetry(encode_telemetry(t)).t_utc is None


def test_u64_range_is_enforced():
    base = dict(seq_echo=1, state=LinkState.LIVE, vbat_v=11.4, up_z=1.0,
                vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
    top = Telemetry(t_us=2 ** 64 - 1, **base)
    assert decode_telemetry(encode_telemetry(top)).t_us == 2 ** 64 - 1
    for bad in (-1, 2 ** 64):
        with pytest.raises(ProtocolError):
            encode_telemetry(Telemetry(t_us=bad, **base))


def test_legacy_frames_still_decode_with_no_time():
    """A console from this tree against the robot as flashed on 2026-09-02.

    The pre-stamp lengths (20 B, 40 B) decode with t_us = 0 and everything
    else intact; they are the one kind of frame that says "unsynced" for a
    reason other than SNTP."""
    t = Telemetry(seq_echo=4242, state=LinkState.STAND, vbat_v=11.2,
                  up_z=0.97, vx_est=0.25, wz_est=-0.1, servo_err=0x03,
                  loop_late_pct=4, t_us=T_REAL)
    short = _legacy(t)
    assert len(short) == TLM_LEN_V1 == 20
    got = decode_telemetry(short)
    assert got.t_us == 0 and got.t_utc is None and got.joints == ()
    assert got.seq_echo == 4242 and got.state is LinkState.STAND
    assert got.vbat_v == pytest.approx(11.2, abs=1e-3)
    assert got.servo_err == 0x03 and got.loop_late_pct == 4

    q = tuple(0.1 * i - 0.5 for i in range(NUM_JOINTS))
    long = _legacy(t._replace(joints=q))
    assert len(long) == TLM_LEN_EXT_V1 == 40
    got = decode_telemetry(long)
    assert got.t_us == 0 and got.joints == pytest.approx(q, abs=1e-3)


def test_the_crc_covers_the_timestamp():
    t = Telemetry(seq_echo=1, state=LinkState.LIVE, vbat_v=11.4, up_z=1.0,
                  vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0,
                  t_us=T_REAL)
    wire = bytearray(encode_telemetry(t))
    wire[21] ^= 0x10                       # a bit inside t_us
    with pytest.raises(ProtocolError):
        decode_telemetry(bytes(wire))


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


# -- arm latch -------------------------------------------------------------
def test_arm_bit_rides_the_frame():
    pkt = decode_command(encode_command(1, 0.0, 0.0, FLAG_ARM | FLAG_ENABLE))
    assert pkt.arm and pkt.enabled and not pkt.estop
    assert not decode_command(encode_command(2, 0.0, 0.0, FLAG_ENABLE)).arm


def test_arm_is_an_edge_not_a_level():
    latch = ArmLatch()
    assert latch.level is None
    assert latch.update(0) is None               # first frame never counts
    assert latch.update(0) is None
    assert latch.update(FLAG_ARM) is True        # 0 -> 1: arm
    assert latch.update(FLAG_ARM | FLAG_ENABLE) is None   # held: nothing
    assert latch.update(FLAG_ENABLE) is False    # 1 -> 0: disarm
    assert latch.update(0) is None
    assert latch.level is False


def test_first_frame_never_arms():
    # A robot rebooting under a client that is still holding ARM=1 stays
    # benched until that client deliberately re-arms.
    latch = ArmLatch()
    assert latch.update(FLAG_ARM) is None
    assert latch.update(FLAG_ARM) is None
    assert latch.update(0) is False
    assert latch.update(FLAG_ARM) is True


def test_arm_ignorant_sender_never_benches_a_tethered_run():
    # The script/gamepad commanders send ARM=0 forever: no edge, no bench.
    latch = ArmLatch()
    for seq in range(50):
        assert latch.update(FLAG_ENABLE if seq % 2 else 0) is None


def test_arm_latch_is_independent_of_the_watchdog():
    # The watchdog drops a reordered frame; the latch still sees its level,
    # and a duplicate makes no edge either way.
    dog, latch = Watchdog(), ArmLatch()
    for seq, flags in [(1, 0), (2, FLAG_ARM), (2, FLAG_ARM), (1, FLAG_ARM)]:
        pkt = Command(seq=seq, vx=0.0, wz=0.0, flags=flags)
        dog.accept(pkt, float(seq))
        latch.update(pkt.flags)
    assert dog.rejected == 2
    assert latch.level is True


def test_home_bit_rides_the_frame_with_every_other_flag():
    # The states a servo reset is FOR are the ones the other bits describe,
    # so it has to coexist with all of them.
    pkt = decode_command(encode_command(1, 0.0, 0.0,
                                        FLAG_HOME | FLAG_ESTOP | FLAG_POSE))
    assert pkt.home and pkt.estop and pkt.pose
    assert not pkt.arm and not pkt.enabled
    assert not decode_command(encode_command(2, 0.0, 0.0, FLAG_ESTOP)).home


def test_home_is_a_rising_edge_only():
    latch = HomeLatch()
    assert latch.update(0) is False               # first frame: level seen
    assert latch.update(FLAG_HOME) is True        # 0 -> 1: the one event
    assert latch.update(FLAG_HOME) is False       # held: no re-home at 20 Hz
    assert latch.update(0) is False               # 1 -> 0 is not an "un-home"
    assert latch.update(FLAG_HOME) is True        # ... and the next press works


def test_first_frame_never_homes():
    # A client that reboots with the bit set must not re-pose ten joints on a
    # robot nobody is watching -- the same rule ArmLatch keeps, and it matters
    # more here, because this request MOVES the robot from a standstill.
    latch = HomeLatch()
    assert latch.update(FLAG_HOME) is False
    assert latch.update(FLAG_HOME) is False
    assert latch.update(0) is False
    assert latch.update(FLAG_HOME) is True


def test_home_works_from_the_states_that_refuse_to_move():
    # E-stopped and armed: the request ends the run, and torque comes back to
    # HOLD the stand. That is the whole point -- a latched E-stop is exactly
    # when an operator needs the robot back on its feet.
    sup = Supervisor()
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=2, flags=FLAG_ARM), 1.0)
    assert sup.state(1.0) is LinkState.LIVE
    sup.accept(live(seq=3, flags=FLAG_ARM | FLAG_ESTOP), 2.0)
    assert sup.state(2.0) is LinkState.ESTOP and not sup.torque_on(2.0)

    sup.accept(live(seq=4, flags=FLAG_ARM | FLAG_ESTOP | FLAG_HOME), 3.0)
    assert sup.homes == 1
    assert sup.state(3.0) is LinkState.BENCH        # the run is over ...
    assert sup.torque_on(3.0)                       # ... but it is HOLDING
    assert diag_arm_result(sup.diag()) is ArmResult.DISARMED_HOME
    assert "standing pose" in diag_reason(sup.diag())

    # It does not arm: a walk command after a home still moves nothing, and
    # re-arming stays a deliberate fresh edge.
    assert not sup.accept(live(seq=5, flags=FLAG_ENABLE), 4.0)
    assert sup.command(4.0) == (0.0, 0.0)


def test_every_home_verdict_reaches_the_operator():
    # A reset that quietly does nothing looks exactly like a dead button, so
    # each way it can fail has its own line -- and each is recognisable AS a
    # home verdict, which is how a console tells "answered" from "ignored by
    # a robot that predates the bit".
    for r in (ArmResult.DISARMED_HOME, ArmResult.HOME_NO_CAL,
              ArmResult.HOME_LOW_BATT, ArmResult.HOME_BUS_FAILED):
        assert is_home_result(r)
        why = diag_reason(pack_diag(False, True, r))
        assert "reset" in why and "unknown to this client" not in why
    for r in (ArmResult.ACCEPTED, ArmResult.REFUSED_NO_CAL,
              ArmResult.DISARMED_FALL, ArmResult.NONE):
        assert not is_home_result(r)


def test_home_needs_no_arm_and_never_arms():
    sup = Supervisor()
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=2, flags=FLAG_HOME), 1.0)
    assert sup.homes == 1 and not sup.armed
    assert sup.state(1.0) is LinkState.BENCH
    # Arming afterwards is an ordinary edge, and it hands the bus back to the
    # loop -- the "holding" state belongs to the bench half only.
    sup.accept(live(seq=3, flags=FLAG_ARM), 2.0)
    assert sup.armed and not sup.holding


def test_supervisor_boots_benched_and_arms_on_an_edge():
    sup = Supervisor()
    assert sup.state(0.0) is LinkState.BENCH
    assert not sup.torque_on(0.0)
    # A walk command from an unarmed client moves nothing.
    assert not sup.accept(live(seq=1, flags=FLAG_ENABLE), 0.0)
    assert sup.command(1.0) == (0.0, 0.0)
    sup.accept(live(seq=2, flags=FLAG_ARM), 2.0)               # 0 -> 1
    assert sup.state(3.0) is LinkState.LIVE
    assert sup.torque_on(3.0)
    sup.accept(live(seq=3, v=0.4, flags=FLAG_ARM | FLAG_ENABLE), 4.0)
    assert sup.command(5.0) == (0.4, 0.0)
    sup.accept(live(seq=4, flags=0), 6.0)                       # 1 -> 0
    assert sup.state(7.0) is LinkState.BENCH
    assert not sup.torque_on(7.0)
    assert sup.command(7.0) == (0.0, 0.0)


def test_supervisor_tethered_run_survives_an_arm_ignorant_client():
    sup = Supervisor(armed=True)
    for seq in range(1, 20):
        sup.accept(live(seq=seq, flags=FLAG_ENABLE), float(seq))
    assert sup.state(20.0) is LinkState.LIVE


def test_supervisor_refuses_to_arm_without_calibration():
    sup = Supervisor(arm_allowed=False)
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=2, flags=FLAG_ARM), 1.0)
    assert sup.state(2.0) is LinkState.BENCH


def test_supervisor_rearm_is_a_fresh_run():
    sup = Supervisor()
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=2, flags=FLAG_ARM | FLAG_ESTOP), 1.0)
    assert sup.state(2.0) is LinkState.ESTOP
    sup.accept(live(seq=3, flags=FLAG_ESTOP), 3.0)              # disarm
    sup.accept(live(seq=4, flags=FLAG_ARM), 4.0)                # re-arm
    assert sup.state(5.0) is LinkState.LIVE                     # not ESTOP


# -- bench diagnostics -----------------------------------------------------
def test_diag_values_are_append_only():
    # Same rule as LinkState: the wire meaning of an existing value must
    # never move, or an old client mis-reports a new robot.
    assert [(r.name, r.value) for r in ArmResult] == [
        ("NONE", 0), ("ACCEPTED", 1), ("REFUSED_NO_CAL", 2),
        ("DISARMED_FALL", 3), ("DISARMED_HOME", 4), ("HOME_NO_CAL", 5),
        ("HOME_LOW_BATT", 6), ("HOME_BUS_FAILED", 7)]
    assert DIAG_RUN == 1 and DIAG_CAL_OK == 2 and DIAG_ARM_SHIFT == 4


def test_diag_packs_the_three_facts_into_one_byte():
    d = pack_diag(False, False, ArmResult.REFUSED_NO_CAL)
    assert not d & DIAG_RUN and not d & DIAG_CAL_OK
    assert diag_arm_result(d) is ArmResult.REFUSED_NO_CAL
    d = pack_diag(True, True, ArmResult.ACCEPTED)
    assert d & DIAG_RUN and d & DIAG_CAL_OK
    assert diag_arm_result(d) is ArmResult.ACCEPTED
    assert pack_diag(False, False, ArmResult.NONE) == 0   # boot state


def test_diag_reason_names_the_gate_that_actually_refused():
    d = pack_diag(False, False, ArmResult.REFUSED_NO_CAL)
    assert "REFUSED" in diag_reason(d)
    assert "no as-built calibration in NVS" in diag_reason(d)
    # Uncalibrated but nobody has tried yet: warn BEFORE the operator waits.
    assert "REFUSED" in diag_reason(pack_diag(False, False, ArmResult.NONE))
    assert "ready to arm" in diag_reason(pack_diag(False, True,
                                                   ArmResult.NONE))


def test_diag_from_a_newer_firmware_is_not_mistaken_for_an_old_reason():
    # Reason 15 does not exist yet. A client that guessed would send someone
    # to re-run `cal` for a fault that has nothing to do with calibration.
    future = 0xF0 | DIAG_CAL_OK
    assert diag_arm_result(future) is None
    assert "unknown to this client" in diag_reason(future)


def test_diag_rides_seq_echo_only_while_benched():
    # The whole compatibility argument: seq_echo is "the seq the robot
    # APPLIED", and a benched loop applies nothing, so only there is the
    # field free. Everywhere else it must stay a sequence number.
    d = pack_diag(False, False, ArmResult.REFUSED_NO_CAL)
    t = Telemetry(seq_echo=d, state=LinkState.BENCH, vbat_v=11.4, up_z=0.0,
                  vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
    assert decode_telemetry(encode_telemetry(t)).diag == d
    live_t = t._replace(state=LinkState.LIVE, seq_echo=1234)
    assert decode_telemetry(encode_telemetry(live_t)).diag == 0
    assert decode_telemetry(encode_telemetry(live_t)).seq_echo == 1234


def test_diag_frame_still_decodes_on_a_client_that_knows_nothing_of_it():
    # An old commander sees an ordinary v1 frame with a small seq_echo --
    # indistinguishable from a fresh boot, and never a false servo fault.
    t = Telemetry(seq_echo=pack_diag(False, True, ArmResult.REFUSED_NO_CAL),
                  state=LinkState.BENCH, vbat_v=11.4, up_z=0.0, vx_est=0.0,
                  wz_est=0.0, servo_err=0, loop_late_pct=0)
    wire = encode_telemetry(t)
    assert len(wire) == TLM_LEN and wire[2] == 1        # unchanged version
    got = decode_telemetry(wire)
    assert got.servo_err == 0 and got.loop_late_pct == 0


def test_supervisor_publishes_the_refusal_it_just_made():
    # The incident this closes: ARM frames for seconds, robot stays BENCH,
    # and the only way to learn why was the tether.
    sup = Supervisor(arm_allowed=False)
    assert diag_arm_result(sup.diag()) is ArmResult.NONE
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=2, flags=FLAG_ARM), 1.0)
    assert sup.state(2.0) is LinkState.BENCH
    assert diag_arm_result(sup.diag()) is ArmResult.REFUSED_NO_CAL
    assert not sup.diag() & DIAG_CAL_OK
    assert sup.seq_echo(2.0) == sup.diag()             # it rides seq_echo
    assert "no as-built calibration in NVS" in diag_reason(sup.diag())


def test_supervisor_seq_echo_is_a_real_sequence_once_armed():
    sup = Supervisor()
    sup.accept(live(seq=1, flags=0), 0.0)
    sup.accept(live(seq=7, flags=FLAG_ARM), 1.0)
    assert sup.state(2.0) is LinkState.LIVE
    assert sup.seq_echo(2.0) == 7
    assert diag_arm_result(sup.diag()) is ArmResult.ACCEPTED
    assert sup.diag() & DIAG_RUN
    sup.accept(live(seq=8, flags=0), 3.0)              # disarm
    assert sup.state(4.0) is LinkState.BENCH
    assert not sup.diag() & DIAG_RUN
    assert "disarmed on request" in diag_reason(sup.seq_echo(4.0))


def test_bench_state_is_last_on_the_wire():
    # Append-only enum: BENCH must not have displaced anything.
    assert list(LinkState)[-1] is LinkState.BENCH
    assert list(LinkState).index(LinkState.FALLEN) == 6
    assert len(LinkState.BENCH.value) <= 5


# -- extended telemetry: mirror mode (docs/mirror-mode.md) -----------------
def _tlm(**kw):
    base = dict(seq_echo=1, state=LinkState.LIVE, vbat_v=12.0, up_z=1.0,
                vx_est=0.0, wz_est=0.0, servo_err=0, loop_late_pct=0)
    base.update(kw)
    return Telemetry(**base)


def test_joint_angles_round_trip():
    q = tuple(0.1 * i - 0.5 for i in range(NUM_JOINTS))
    wire = encode_telemetry(_tlm(joints=q))
    assert len(wire) == TLM_LEN_EXT
    back = decode_telemetry(wire)
    assert back.joints == pytest.approx(q, abs=1e-3)
    # ... and the classic body is untouched by the extra channels.
    assert back.state is LinkState.LIVE and back.vbat_v == pytest.approx(12.0)


def test_not_asking_changes_nothing():
    """The property every existing commander depends on.

    A robot that is not asked for joint angles must beacon the classic frame,
    byte for byte -- otherwise bimo_tui, commander.py and both twins go blind
    at once, which is why this frame is requested rather than volunteered.
    (Since 2026-09-02 "classic" is the 28 B stamped frame.)
    """
    t = _tlm()
    assert len(encode_telemetry(t)) == TLM_LEN
    assert decode_telemetry(encode_telemetry(t)).joints == ()


def test_a_length_between_the_four_is_refused():
    wire = encode_telemetry(_tlm(joints=(0.0,) * NUM_JOINTS))
    for bad in (wire[:TLM_LEN + 2], wire[:-1], wire + b"\x00",
                wire[:TLM_LEN_V1 + 1], wire[:TLM_LEN_EXT_V1 + 1],
                wire[:TLM_LEN_V1 - 1]):
        with pytest.raises(ProtocolError):
            decode_telemetry(bad)


def test_the_crc_covers_the_joints():
    q = [0.0] * NUM_JOINTS
    wire = bytearray(encode_telemetry(_tlm(joints=tuple(q))))
    wire[28] ^= 0x01                       # flip a bit inside a joint field
    with pytest.raises(ProtocolError):
        decode_telemetry(bytes(wire))


def test_wrong_joint_count_is_refused():
    with pytest.raises(ProtocolError):
        encode_telemetry(_tlm(joints=(0.0, 0.0)))


def test_pose_is_a_level_and_does_not_disturb_the_other_flags():
    c = decode_command(encode_command(1, 0.0, 0.0, FLAG_ARM | FLAG_POSE))
    assert c.pose and c.arm and not c.enabled and not c.estop
    assert not decode_command(encode_command(2, 0.0, 0.0, FLAG_ARM)).pose
    # Bit 3, so it cannot collide with the three that predate it.
    assert FLAG_POSE == 8
