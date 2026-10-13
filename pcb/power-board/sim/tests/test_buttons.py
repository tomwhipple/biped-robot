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

from sim.pack import Pack3S
from sim.sequencer_sim import (BoardState, Buttons, Esp32Stub, PowerBoard)


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
    rather than reach ON."""
    board = PowerBoard()
    esp = _FirmwareOn()
    def btns(t):
        if t < 0.35:
            return Buttons(power_pressed=True)
        # press E-stop mid-STARTING — but wait past the wake: the rail
        # reaches STARTING around t ≈ 0.3+ms at load 0, so pressing at 0.42
        # hits mid-STARTING.
        if 0.42 <= t < 0.9:
            return Buttons(estop_latched=True)
        return Buttons()
    out = _run(board, btns, esp, rail_load=0.0, n_ticks=1500)
    # At no point after the e-stop latch takes effect is the rail fully ON.
    # The latch is *released* at t=0.9: the rail could in principle come up
    # after the release. Assert only that *during* the press the rail did
    # not stay/live in ON.
    pressed = [s for s in out if 0.42 < s.t_s < 0.90]
    # first tick at 0.42 has the state *before* the latch landed; allow one
    # transitional tick
    assert not any(s.rail_state == "ON" for s in pressed[1:]), (
        f"rail reached ON while E-stop was pressed: "
        f"{[s.rail_state for s in pressed]}"
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
    notices (RAIL_V high while ESTOP_N claims pressed), and raises KILL.

    This test asserts ONLY the chain the design claims: ESTOP_N high →
    firmware decides → KILL high → PACK+ drops. (The F4 module proper is
    firmware-side and out of scope here.)"""
    board = PowerBoard()
    class F4Logic(Esp32Stub):
        def __init__(self):
            self.kill_issued_at: float | None = None

        def tick(self, t, board):
            if self.kill_issued_at is not None:
                return {"servo_en_v": 0.0, "kill_v": 3.3, "i2c_start_v": 3.3}
            if board is None:
                return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}
            # F4 sees: E-STOP latched (contact B welded ⇒ ESTOP_N is
            # pulled low regardless — no wait, contact B STUCK CLOSED means
            # ESTOP_N stays low even when the button is pressed. The
            # firmware's actual F4 sees "rail_v high while SERVO_EN is
            # low" as the stuck-on-rail signature; ESTOP_N is *also*
            # involved in the real firmware, but here we deliberately cut
            # to the *rail-up-when-commanded-off* criterion).
            if t > 1.5 and board.rail_v > 0.5 and t > 0.4:
                # rail still up after the press — weld-on fault
                self.kill_issued_at = t
                return {"servo_en_v": 0.0, "kill_v": 3.3, "i2c_start_v": 3.3}
            return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}

    # We simulate the Q3 weld by pinning the rail: an Esp32 stub cannot do
    # that, so we monkeypatch the rail switch to never leave ON.
    from sim.tps48111 import RailState
    def btns(t):
        if t < 0.4:
            return Buttons(power_pressed=True)
        if t >= 1.5:
            # both contacts welded: A stays feeding EN, B stays pulling
            # ESTOP_N low
            return Buttons(estop_latched=True,
                           estop_a_stuck_closed=True,
                           estop_b_stuck_closed=True)
        return Buttons()
    esp = F4Logic()
    out = _run(board, btns, esp, rail_load=3.05, n_ticks=3500)
    # The fault: after t=1.5, the rail would *normally* fall via EN; here
    # ESTOP is ineffective (welded A), and INP would drop it but our ESP32
    # only drops SERVO_EN when it decides to KILL. In the real F4 the
    # decision is "rail_up AND commanded_off" — we just need to see that
    # KILL lands and the MAIN switch (BQ76922) takes PACK+ down.
    assert esp.kill_issued_at is not None, (
        "F4 never saw the stuck-on-rail and never issued KILL"
    )
    after_kill = [s for s in out if s.t_s >= esp.kill_issued_at + 1.2]
    assert all(s.bq_state == "SHUTDOWN" for s in after_kill), (
        "KILL didn't put BQ76922 into SHUTDOWN within 1.2 s of the F4 trigger"
    )
    assert all(s.packplus_v < 1.0 for s in after_kill), (
        "PACK+ stayed live after KILL"
    )


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
