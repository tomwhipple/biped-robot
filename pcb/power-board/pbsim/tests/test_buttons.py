"""Part-2.2: POWER (J10, momentary NO) + E-STOP (J11, latching, two NC
contacts) — press-and-hold wake, debounce coverage, E-stop mid-serve, and
fault injection on each NC contact.

What "green" looks like (design §7 + harness.md):

- press-and-hold POWER for ≥ 280 ms with the board off: the BQ76922
  wakes, its FETs close, PACK+ rises, and the rail does NOT close on its
  own (SERVO_EN low until firmware enables it).
- POWER pressed while on (the "orderly shutdown"): PWR_INT_N drops to
  LOW, the ESP32 stub sees it, drives KILL; within ~1 s of KILL held high
  the BQ76922 goes to SHUTDOWN and PACK+ falls to 0.
- E-stop asserted mid-serve (rail ON, walking load 3.05 A): SERVO+ goes
  to 0 within one tick (EN/UVLO below 0.3 V), ESP32 sees ESTOP_N high
  (contact B opened), pack+logic still up (the stop must NOT kill the
  logic).
- Contact A (the EN feed) welded: the ESP32 still sees the press through
  contact B and drops SERVO_EN, and INP opens the rail.
- Contact B (ESTOP_N) welded: the ESP32 is blind to the press, and contact
  A still opens the rail in hardware.
- Q3 welded: the press leaves the rail up; the firmware sees ESTOP_N high
  with RAIL_V up and raises KILL, and the BQ76922 opens PACK+ (§5.3).
- Both contacts welded: the press does nothing and nothing can see it —
  outside the one-failure claim, pinned as a limit.
- POWER jammed: the board re-wakes 280 ms after every shutdown; only
  unplugging the pack turns it off (design §7, step 6).

A note on debounce: design §7 and harness.md do not specify an RC on either
button beyond what the wake path already provides (R9 + D2 + TS2-input RC).
The POWER wake is *level*-sensitive on TS2 (not edge-sensitive) with a
280 ms turn-on delay — that IS the debounce. We assert that a button
glitch shorter than 280 ms does not wake the part.
"""
from __future__ import annotations

import pytest

from pbsim.scenarios import BootAndHoldOn as _FirmwareOn
from pbsim.scenarios import run as _run
from pbsim.sequencer_sim import (BoardState, Buttons, Esp32Stub, PowerBoard)


class _ShutdownOnPwrInt(Esp32Stub):
    """Firmware behaviour per design §7 POWER-pressed-while-on: see
    PWR_INT_N low, run the orderly shutdown, then drop KILL high."""
    def __init__(self):
        self.landed_at: float | None = None
        self.kill_at: float | None = None
        self._released_before_repress: float | None = None
        self._press_seen_low = False

    def tick(self, t_s, board: BoardState | None):
        if board is None or not hasattr(board, "pwr_int_n_v"):
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}
        if self.kill_at is not None:
            # KILL must be held high for ≥ 1.0 s to take effect
            # (datasheet §8.5). The firmware keeps driving KILL high; the
            # test asserts the BQ76922 goes to SHUTDOWN within that window.
            return {"servo_en_v": 0.0, "kill_v": 3.3, "i2c_start_v": 3.3}
        # PWR_INT_N reads low whenever the POWER button is pressed. Only
        # treat it as an orderly-shutdown request if the board has finished
        # its wake (rail has been ON at some point) AND the button has been
        # released at least once since. Without the release guard, the
        # 400 ms boot press at t<0.4 would be misread as a shutdown press
        # the moment the rail comes up.
        if board.pwr_int_n_v > 2.0:
            self._released_before_repress = t_s
        rail_was_on = board.rail_state == "ON"
        if board.pwr_int_n_v < 0.5 and rail_was_on \
                and self._released_before_repress is not None \
                and t_s - self._released_before_repress > 0.1:
            self.kill_at = t_s
            return {"servo_en_v": 0.0, "kill_v": 3.3, "i2c_start_v": 3.3}
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


# ----------------- POWER: wake and orderly shutdown ---------------------------


def test_power_button_held_less_than_280_ms_does_not_wake():
    """A glitch: the 280 ms internal wake latency is the POWER button's
    debounce. A button tap shorter than that must leave the board off."""
    board = PowerBoard()
    # buttons pressed for 200 ms only
    out = _run(board, lambda t: Buttons(power_pressed=(t < 0.200)),
               _FirmwareOn(), rail_load=0.0, n_ticks=500)
    assert all(s.bq_state == "SHUTDOWN" for s in out), (
        f"bq woke on a 200 ms tap; states: {[s.bq_state for s in out]}"
    )
    assert all(s.rail_state == "OFF" for s in out)
    assert all(s.packplus_v == 0.0 for s in out)


def test_power_button_held_280_ms_wakes():
    """Held past 280 ms → BQ76922 reaches NORMAL; rail stays OFF until
    firmware drives SERVO_EN high (here _FirmwareOn drives it high every
    tick so the STARTING→ON transition is exercised)."""
    board = PowerBoard()
    out = _run(board, lambda t: Buttons(power_pressed=(t < 0.500)),
               _FirmwareOn(), rail_load=0.5, n_ticks=900)
    states = [s.bq_state for s in out]
    assert "NORMAL" in states, f"bq never woke: {states[:50]}..."
    first_normal = next(i for i, s in enumerate(states) if s == "NORMAL")
    # wake should land between 280 and 350 ms
    assert 0.270 <= out[first_normal].t_s <= 0.40


def test_power_on_sequence_order():
    """§7 "Power on": button → BQ76922 wake (280 ms) → PACK+ alive → ESP32
    boots → SERVO_EN → rail soft-start → 40 ms → rail live."""
    board = PowerBoard()
    esp = _FirmwareOn()
    out = _run(board, lambda t: Buttons(power_pressed=(t < 0.350)),
               esp, rail_load=0.0, n_ticks=900)
    # PACK+ appears when bq first reports fets_closed
    pack_wakes_at = next(s.t_s for s in out if s.packplus_v > 5.0)
    # rail starts rising after the 40 ms soft-start once SERVO_EN goes high
    rail_on_at = next(s.t_s for s in out if s.rail_state == "ON")
    assert pack_wakes_at < rail_on_at
    # rail takes 30–80 ms to come up after EN+INP both high
    rail_starting_at = next(s.t_s for s in out if s.rail_state == "STARTING")
    tau_ms = (rail_on_at - rail_starting_at) * 1e3
    assert 30.0 < tau_ms < 80.0, (
        f"soft-start window out of spec: {tau_ms} ms (design ~40 ms)"
    )
    # rail_v at GPIO 34 through R38/R39
    rail_on = [s for s in out if s.rail_state == "ON"]
    assert rail_on[-1].rail_v_at_gpio34 == pytest.approx(
        rail_on[-1].rail_v * (10.0 / 57.0), rel=1e-3
    )


def test_power_pressed_while_on_runs_orderly_shutdown():
    """§7 "POWER pressed while on" — PWR_INT_N falls, firmware lands and
    raises KILL, BQ76922 shuts down within ~1 s of KILL held high, and
    the PACK+ rail falls to 0."""
    board = PowerBoard()
    esp = _ShutdownOnPwrInt()
    # press POWER for 400 ms to wake, release, then press again for the
    # orderly shutdown starting at t=1.5 s.
    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if 1.5 <= t < 2.0:
            return Buttons(power_pressed=True)
        return Buttons()
    out = _run(board, btns, esp, rail_load=0.5, n_ticks=4500)
    # board must come up first
    assert any(s.bq_state == "NORMAL" for s in out)
    assert any(s.rail_state == "ON" for s in out)
    # once KILL is asserted, the part must reach SHUTDOWN
    assert esp.kill_at is not None, "firmware never saw PWR_INT_N low"
    # KILL must hold ≥ 1.0 s (BQ76922 datasheet). Allow one tick of
    # hysteresis: filter from kill_at + 1.0 + one 1 ms tick + wake-up lag.
    shutdown_states = [s.bq_state for s in out
                       if s.t_s > esp.kill_at + 1.05]
    assert all(s == "SHUTDOWN" for s in shutdown_states), (
        f"after KILL ≥ 1 s, bq not in SHUTDOWN: first bad = "
        f"{next(s for s in shutdown_states if s != 'SHUTDOWN')}"
    )
    # rail must be down once KILL lands
    rail_up_after_kill = [s for s in out
                          if s.t_s >= esp.kill_at + 1.3 and s.rail_v > 0.5]
    assert not rail_up_after_kill


def test_kill_before_1s_does_not_shut_down():
    """The 1 s KILL holdoff is load-bearing (datasheet 8.5). Assert that a
    500 ms KILL pulse does NOT put the board down."""
    board = PowerBoard()

    class KillPulse(Esp32Stub):
        def tick(self, t, board):
            if 0.8 <= t < 1.3:
                return {"servo_en_v": 3.3, "kill_v": 3.3, "i2c_start_v": 3.3}
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}

    out = _run(board, lambda t: Buttons(power_pressed=t < 0.5),
               KillPulse(), rail_load=0.5, n_ticks=2000)
    after_pulse = [s for s in out if s.t_s > 1.4]
    assert all(s.bq_state == "NORMAL" for s in after_pulse), (
        "a 0.5 s KILL pulse shut the board down; the holdoff is wrong"
    )


# ----------------- E-STOP: hardware cut + firmware visibility -----------------

def test_estop_pressed_opens_servo_rail():
    """§7 "E-stop": latched contact A (J11 pins 1–2) opens the EN feed;
    SERVO+ falls regardless of what the ESP32 was commanding. The ESP32
    (firmware) sees contact B's ESTOP_N line go high."""
    board = PowerBoard()
    esp = _FirmwareOn()
    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            return Buttons(estop_latched=True)
        return Buttons()
    out = _run(board, btns, esp, rail_load=3.05, n_ticks=2500)
    rail_was_on = any(s.rail_state == "ON" for s in out
                      if 1.0 < s.t_s < 1.5)
    assert rail_was_on, "rail never came up before the E-stop"
    # after the press the rail must be down. The board ticks once at the
    # boundary with `estop_latched=True` and the state snapshot the test
    # sees is from BEFORE the latch — t=1.501 onward is what matters.
    after = [s for s in out if s.t_s > 1.51]
    assert all(s.rail_state == "OFF" for s in after)
    assert all(s.rail_v < 0.5 for s in after)
    # but PACK+ must still be live — the ESP32 and Pi stay up
    assert all(s.packplus_v > 9.0 for s in after)
    # and ESTOP_N must read high so firmware knows
    assert all(s.estop_n_v > 3.0 for s in after)


def test_estop_while_rail_starting_recovers_off():
    """The E-stop can arrive while the rail is still in its 40 ms STARTING
    window; the EN line is what decides, so the partial ramp must abort
    rather than reach ON.

    Timing measured on this suite at rail_load=0: BQ76922 reaches NORMAL
    at t=0.281 (wake latency), STARTING is entered that same tick, and the
    40 ms soft start lands ON at t=0.323. We press mid-ramp at t=0.295 —
    14 ms into the ~42 ms ramp — which exercises the property the previous
    version claimed but did not deliver (P1-1: a press at 0.42 s arrived
    ~97 ms AFTER ON, asserting nothing about the mid-ramp abort)."""
    board = PowerBoard()
    esp = _FirmwareOn()
    def btns(t):
        if t < 0.29:
            return Buttons(power_pressed=True)
        # press E-stop genuinely mid-STARTING — after wake (0.281) but
        # before the 40 ms soft start completes at 0.323.
        if 0.295 <= t < 0.9:
            return Buttons(estop_latched=True)
        return Buttons()
    out = _run(board, btns, esp, rail_load=0.0, n_ticks=1500)
    # The rail must never reach ON while the E-stop is pressed. Once the
    # button is released the rail may legally come up — that is asserted
    # separately by test_power_on_sequence_order and is not this test's
    # property.
    pressed = [s for s in out if 0.295 < s.t_s < 0.90]
    # first tick at 0.295 has the state *before* the latch landed; allow
    # one transitional tick.
    assert not any(s.rail_state == "ON" for s in pressed[1:]), (
        f"rail reached ON while E-stop was pressed: "
        f"{[s.rail_state for s in pressed]}"
    )
    # and the rail must actually back OUT of STARTING — a model that
    # ignored EN during STARTING would still reach ON behind the latch.
    assert any(s.rail_state == "STARTING" for s in out
               if 0.280 < s.t_s < 0.295), (
        "setup: rail must have entered STARTING before the press lands"
    )
    assert any(s.rail_state == "OFF" for s in pressed[5:]), (
        "rail never aborted STARTING back to OFF during the press"
    )


def test_estop_broken_wire_makes_rail_off():
    """harness.md: 'Pressing the button, or a broken wire, turns the rail
    off with no firmware involved.' Drive the J11 cable unplugged while
    the rail is ON."""
    board = PowerBoard()
    esp = _FirmwareOn()
    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            return Buttons(estop_broken_wire=True)
        return Buttons()
    out = _run(board, btns, esp, rail_load=3.05, n_ticks=2500)
    assert any(s.rail_state == "ON" for s in out if s.t_s < 1.4)
    after = [s for s in out if s.t_s > 1.51]
    assert all(s.rail_state == "OFF" for s in after)
    assert all(s.packplus_v > 9.0 for s in after)


def test_estop_stuck_a_contact_means_only_software_can_open_rail():
    """Fault injection: contact A (EN feed) WELDED SHUT. EN sees its feed
    regardless of E-stop. The rail must still open via INP (the ESP32's
    SERVO_EN). This is the second-of-two switches path the design claims:
    §5.3 'A rail stuck on is caught' — here it's contact A's job that
    fails, and INP must carry the day."""
    board = PowerBoard()
    class KillOnPress(Esp32Stub):
        def tick(self, t, board):
            if board is None:
                return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}
            # ESP32 sees ESTOP_N pinned LOW by the also-welded B — no, B
            # still works here (only A is welded). ESP32 sees ESTOP_N go
            # high on press and drops SERVO_EN in response. That's the F4
            # stuck-on-rail behaviour, exercised end-to-end.
            if board.estop_n_v > 2.0:
                return {"servo_en_v": 0.0, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}

    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            # E-stop pressed but contact A is stuck closed: EN still fed.
            return Buttons(estop_latched=True, estop_a_stuck_closed=True)
        return Buttons()

    out = _run(board, btns, KillOnPress(), rail_load=3.05, n_ticks=2500)
    before = [s for s in out if 1.0 < s.t_s < 1.5]
    assert any(s.rail_state == "ON" for s in before), "rail never came up"
    after = [s for s in out if s.t_s > 1.6]
    # ESP32 noticed via ESTOP_N (contact B still works) and dropped SERVO_EN.
    assert all(s.rail_state == "OFF" for s in after), (
        "with contact A welded, INP (SERVO_EN low) failed to open the rail"
    )


def test_estop_with_welded_q3_kill_opens_main_switch():
    """Design §5.3: 'A rail stuck on is caught. With the E-stop pressed and
    the rail still up, the firmware reports the fault and opens the main
    switch (the BQ76922's discharge FET). That is a second switch in
    series, so one failed FET does not defeat the stop.'

    Fault injection: Q3 (the rail FET) welded — the rail stays up whatever
    EN and INP say. Both E-stop contacts work. The press opens contact B,
    so the F4 stub sees ESTOP_N high; it drops SERVO_EN, sees RAIL_V still
    up 50 ms later, and raises KILL; the BQ76922 opens PACK+ 1 s after
    that. (Round 2's version dropped SERVO_EN at 0.8 s for no reason and
    issued KILL at 0.85 s, before its E-stop press at 1.5 s, so the press
    took no part — `test_stuck_on_rail_is_detectable_by_firmware` already
    covers a firmware-initiated stop.)"""
    board = PowerBoard()

    from pbsim.tps48111 import RailState
    def welded_q3_update(t_s, dt_s, en_pin_v, inp_pin_v, pack_v, rail_i_a):
        sw = board.rail_sw
        if pack_v > 6.0:
            sw.state = RailState.ON
            sw.rail_v = pack_v - rail_i_a * (2e-3 + 1.2e-3)
        else:
            sw.state = RailState.OFF
            sw.rail_v = 0.0
    board.rail_sw.update = welded_q3_update  # type: ignore[method-assign]

    class F4Logic(Esp32Stub):
        """F4 (design §9): on the E-stop, SERVO_EN low; if RAIL_V is still
        up 50 ms later, KILL (held, as the 1 s RST_SHUT window needs)."""
        def __init__(self):
            self.estop_seen_at: float | None = None
            self.kill_issued_at: float | None = None

        def tick(self, t_s, board):
            if board is None:
                return {"servo_en_v": 3.3, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            if self.kill_issued_at is not None:
                return {"servo_en_v": 0.0, "kill_v": 3.3,
                        "i2c_start_v": 3.3}
            if board.estop_n_v > 2.0:
                if self.estop_seen_at is None:
                    self.estop_seen_at = t_s
                if (board.rail_v_at_gpio34 > 0.5
                        and t_s - self.estop_seen_at >= 0.05):
                    self.kill_issued_at = t_s
                    return {"servo_en_v": 0.0, "kill_v": 3.3,
                            "i2c_start_v": 3.3}
                return {"servo_en_v": 0.0, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}

    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            return Buttons(estop_latched=True)
        return Buttons()
    esp = F4Logic()
    out = _run(board, btns, esp, rail_load=3.05, n_ticks=4000)
    assert esp.kill_issued_at is not None, (
        "F4 never saw the stuck rail and never issued KILL"
    )
    assert 1.5 < esp.kill_issued_at < 1.6, (
        f"KILL at {esp.kill_issued_at:.3f} s is not a response to the "
        f"press at 1.5 s"
    )
    # the weld is real: EN at 0 V and SERVO_EN low leave the rail up
    # until the main switch opens
    held = [s for s in out if 1.51 < s.t_s < esp.kill_issued_at + 0.9]
    assert all(s.rail_v > 9.0 and s.estop_n_v > 3.0 for s in held)
    after_kill = [s for s in out if s.t_s >= esp.kill_issued_at + 1.05]
    assert after_kill
    assert all(s.bq_state == "SHUTDOWN" for s in after_kill), (
        "KILL didn't put the BQ76922 into SHUTDOWN 1.05 s after F4 raised it"
    )
    assert all(s.packplus_v < 1.0 and s.rail_v < 1.0 for s in after_kill), (
        "PACK+ or the rail stayed live after KILL"
    )


def test_estop_both_contacts_welded_defeats_the_button():
    """Fault injection: BOTH contacts welded shut, Q3 healthy. Contact A
    keeps EN fed and contact B keeps ESTOP_N low, so the press reaches
    neither the TPS48111 nor the ESP32: the rail stays up and the board
    cannot tell the button was pressed. Two welded contacts are outside
    the design's one-failure claim (§5.3); this pins that limit, with a
    firmware that would drop SERVO_EN if it ever saw ESTOP_N."""
    board = PowerBoard()

    class ObeysEstop(Esp32Stub):
        def tick(self, t_s, board):
            if board is not None and board.estop_n_v > 2.0:
                return {"servo_en_v": 0.0, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}

    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            return Buttons(estop_latched=True,
                           estop_a_stuck_closed=True,
                           estop_b_stuck_closed=True)
        return Buttons()
    out = _run(board, btns, ObeysEstop(), rail_load=3.05, n_ticks=2500)
    after = [s for s in out if s.t_s > 1.51]
    assert all(s.rail_state == "ON" and s.rail_v > 9.0 for s in after)
    assert all(s.estop_n_v < 0.5 for s in after)


def test_estop_contact_b_welded_alone_rail_still_dies_via_contact_a():
    """Fault injection: contact B (ESTOP_N sense) WELDED SHUT — the ESP32
    never sees the press (ESTOP_N stays low even when the button is
    latched). This is the asymmetry design §5.3 leans on: with the firmware
    blind, the hardware stop still has to work through contact A.

    Asserts:
      * rail falls at the press (contact A still opens EN),
      * ESTOP_N reads LOW the whole time (ESP32 is blind),
      * PACK+ stays up (logic untouched)."""
    board = PowerBoard()
    esp = _FirmwareOn()   # ESP32 does NOT react to ESTOP_N — weld makes
                          # the firmware blind to the press.

    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            # E-stop pressed but contact B welded closed: ESTOP_N pinned
            # low even under press; contact A works normally.
            return Buttons(estop_latched=True, estop_b_stuck_closed=True)
        return Buttons()

    out = _run(board, btns, esp, rail_load=3.05, n_ticks=2500)
    before = [s for s in out if 1.0 < s.t_s < 1.5]
    assert any(s.rail_state == "ON" for s in before), "rail never came up"
    after = [s for s in out if s.t_s > 1.6]
    assert all(s.rail_state == "OFF" for s in after), (
        "with only contact B welded, contact A still must open the rail"
    )
    # The ESP32 never sees the press — ESTOP_N stays LOW throughout.
    assert all(s.estop_n_v < 0.5 for s in after), (
        "ESTOP_N read high with contact B welded closed — the weld model "
        "is wrong"
    )
    assert all(s.packplus_v > 9.0 for s in after)


def test_power_button_does_not_wake_when_kill_latched():
    """If the BQ76922 has already gone to SHUTDOWN via an SUCD or KILL,
    a fresh press of POWER for < 280 ms must do nothing; a *new* press-and-
    hold ≥ 280 ms is a legal re-wake. Both behaviours must be observable.
    """
    board = PowerBoard()
    esp = _ShutdownOnPwrInt()
    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if 1.5 <= t < 1.7:
            return Buttons(power_pressed=True)   # triggers shutdown
        if 3.6 <= t < 3.8:
            return Buttons(power_pressed=True)   # too short to re-wake
        if 5.0 <= t < 5.5:
            return Buttons(power_pressed=True)   # long enough to re-wake
        return Buttons()
    out = _run(board, btns, esp, rail_load=0.5, n_ticks=6000)
    # First up-and-down
    assert any(s.bq_state == "NORMAL" for s in out if s.t_s < 1.5)
    assert any(s.bq_state == "SHUTDOWN" for s in out if s.t_s > 3.0
               and s.t_s < 4.0)
    # 200 ms tap does NOT re-wake
    tap_window = [s for s in out if 4.0 < s.t_s < 4.9]
    assert all(s.bq_state == "SHUTDOWN" for s in tap_window)
    # 500 ms hold does re-wake
    hold_window = [s for s in out if s.t_s > 5.4]
    assert any(s.bq_state == "NORMAL" for s in hold_window), (
        "500 ms press failed to re-wake after KILL"
    )


def test_jammed_power_button_rewakes_after_every_shutdown():
    """POWER jammed down holds TS2 low, so 280 ms after each shutdown the
    BQ76922 wakes again: the board cannot stay off, and unplugging the
    pack is the only off (design §7, step 6: no hardware hold-off). The
    firmware here asks for shutdown 0.5 s after each boot (as an auto-off
    would) and is unpowered while PACK+ is down."""

    class KillAfterBoot(Esp32Stub):
        def __init__(self):
            self.booted_at: float | None = None

        def tick(self, t_s, board):
            if board is None or board.packplus_v < 5.0:
                self.booted_at = None
                return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 0.0}
            if self.booted_at is None:
                self.booted_at = t_s
            kill = 3.3 if t_s - self.booted_at >= 0.5 else 0.0
            return {"servo_en_v": 0.0, "kill_v": kill, "i2c_start_v": 3.3}

    out = _run(PowerBoard(), lambda t: Buttons(power_pressed=True),
               KillAfterBoot(), rail_load=0.0, n_ticks=8000)
    states = [s.bq_state for s in out]
    downs = [out[i].t_s for i in range(1, len(out))
             if states[i - 1] == "NORMAL" and states[i] == "SHUTDOWN"]
    ups = [out[i].t_s for i in range(1, len(out))
           if states[i - 1] == "SHUTDOWN" and states[i] == "NORMAL"]
    assert len(downs) >= 3, f"shutdowns: {downs}"
    for d in downs[:-1]:
        rewake = next(u for u in ups if u > d)
        assert 0.27 <= rewake - d <= 0.30, (
            f"shutdown at {d:.3f} s re-woke at {rewake:.3f} s"
        )

    # released, the same firmware's shutdown sticks
    out = _run(PowerBoard(), lambda t: Buttons(power_pressed=t < 0.4),
               KillAfterBoot(), rail_load=0.0, n_ticks=4000)
    shut = next(s.t_s for s in out if s.t_s > 0.5 and s.bq_state == "SHUTDOWN")
    assert all(s.bq_state == "SHUTDOWN" for s in out if s.t_s >= shut)
