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
- Stuck-closed contact A (the EN-feed contact) — the *welded* fault: with
  ESP32 commanding SERVO_EN off, the rail still must NOT close. That's
  INP doing its job.
- Stuck-closed contact B (ESTOP_N sense) — the *welded* fault: the ESP32
  cannot *sense* the E-stop, but the rail still opens in hardware. The
  firmware's job here is to *detect the disagreement* between commanded
  rail state and GPIO 34's RAIL_V reading; we assert that disagreement
  signal exists.

A note on debounce: design §7 and harness.md do not specify an RC on either
button beyond what the wake path already provides (R9 + D2 + TS2-input RC).
The POWER wake is *level*-sensitive on TS2 (not edge-sensitive) with a
280 ms turn-on delay — that IS the debounce. We assert that a button
glitch shorter than 280 ms does not wake the part.
"""
from __future__ import annotations

import pytest

from pbsim.pack import Pack3S
from pbsim.sequencer_sim import (BoardState, Buttons, Esp32Stub, PowerBoard)


class _FirmwareOn(Esp32Stub):
    """Steady-state firmware: SERVO_EN on, no KILL."""
    def tick(self, t_s, board):
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


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


def _run(board, buttons_at, esp, rail_load, n_ticks):
    out = []
    last = None
    for _ in range(n_ticks):
        b = buttons_at(board.t_s)
        st = board.tick(b, esp, rail_load_a=rail_load, board=last)
        out.append(st)
        last = st
    return out


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


def test_estop_both_contacts_stuck_means_kill_must_open_main_switch():
    """Design §5.3: 'A rail stuck on is caught. With the E-stop pressed and
    the rail still up, the firmware reports the fault and opens the main
    switch (the BQ76922's discharge FET). That is a second switch in
    series, so one failed FET does not defeat the stop.'

    Fault injection: *both* contacts welded, plus Q3 (the rail FET)
    welded — the *rail* stays up even with INP low; the F4 sequencer
    notices (RAIL_V high while SERVO_EN is commanded low), and raises
    KILL.

    This test asserts ONLY the chain the design claims: rail-up while
    commanded-off → firmware decides → KILL high → PACK+ drops. (The F4
    module proper is firmware-side and out of scope here.)"""
    board = PowerBoard()

    # Weld Q3: replace the rail switch with one that never opens on INP or
    # on EN. The previous version's docstring promised this monkeypatch but
    # never installed it (P2-1) — without a welded Q3 the rail falls via
    # SERVO_EN low alone and the test exercises nothing the simpler
    # `test_stuck_on_rail_is_detectable_by_firmware` does not already
    # cover.
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
        """F4 from design §9: sees rail-up while SERVO_EN is commanded
        low (the disagreement), raises KILL (with the 1 s RST_SHUT window
        the datasheet sets)."""
        def __init__(self):
            self.disagree_since: float | None = None
            self.kill_issued_at: float | None = None
            self.servo_en_v = 3.3

        def tick(self, t_s, board):
            if board is None:
                return {"servo_en_v": 3.3, "kill_v": 0.0,
                        "i2c_start_v": 3.3}
            if self.kill_issued_at is not None:
                return {"servo_en_v": 0.0, "kill_v": 3.3,
                        "i2c_start_v": 3.3}
            # command SERVO_EN off once the boot has had a chance to
            # settle — this is what makes "rail is up" a *disagreement*.
            if t_s > 0.8:
                self.servo_en_v = 0.0
            disagree = (board.rail_v_at_gpio34 > 0.05
                        and self.servo_en_v < 0.05)
            if disagree and self.disagree_since is None:
                self.disagree_since = t_s
            if disagree and t_s - self.disagree_since > 0.05:
                self.kill_issued_at = t_s
                return {"servo_en_v": 0.0, "kill_v": 3.3,
                        "i2c_start_v": 3.3}
            if not disagree:
                self.disagree_since = None
            return {"servo_en_v": self.servo_en_v, "kill_v": 0.0,
                    "i2c_start_v": 3.3}

    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            # both contacts welded: A stays feeding EN, B stays pulling
            # ESTOP_N low — firmware cannot see the press at all.
            return Buttons(estop_latched=True,
                           estop_a_stuck_closed=True,
                           estop_b_stuck_closed=True)
        return Buttons()
    esp = F4Logic()
    out = _run(board, btns, esp, rail_load=3.05, n_ticks=5000)
    # The fault: after t=0.8 the ESP32 has commanded SERVO_EN off, but the
    # welded Q3 keeps the rail live. F4 sees RAIL_V high while SERVO_EN is
    # low for ≥ 50 ms and raises KILL.
    assert esp.kill_issued_at is not None, (
        "F4 never saw the stuck rail and never issued KILL"
    )
    after_kill = [s for s in out if s.t_s >= esp.kill_issued_at + 1.2]
    assert all(s.bq_state == "SHUTDOWN" for s in after_kill), (
        "KILL didn't put BQ76922 into SHUTDOWN within 1.2 s of the F4 trigger"
    )
    assert all(s.packplus_v < 1.0 for s in after_kill), (
        "PACK+ stayed live after KILL"
    )


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
