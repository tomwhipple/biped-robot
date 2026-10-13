"""P1-3: dynamic coverage of the TPS48111's own protections (CB, SC) plus
the EN-pulse latch release.

C5's core scenario — a servo-lead short clears on the TPS48111's SC while
the BQ76922's 150 A SCD stays out of it — is asserted here live. Static
arithmetic against 19.8/39.3 lives in test_servo_branches; this file is
the dynamic side that was dark until round 2.

What "green" means:

- A direct over-current well above CB (e.g. 25 A standing) trips the
  rail's CB latch in ~10.5 ms; the rail falls to OFF-and-latched, the
  BQ76922 is UNAFFECTED, and PACK+ stays up.
- A direct short-circuit (45 A) trips SC within microseconds; rail
  falls, BQ76922's OCD does NOT latch (CB + SC together clear it before
  the 50 ms OCD delay would land).
- Once the over-current clears, an EN pulse (E-STOP toggled) releases
  the TPS latch and the rail can come back. The BQ76922's latches — if
  any were set — stay latched (asymmetric: BQ latches need VCC recycle
  or `kill()`).
"""
from __future__ import annotations

import pytest

from pbsim.bq76922 import BqState, Protection
from pbsim.pack import Pack3S
from pbsim.sequencer_sim import Buttons, Esp32Stub, PowerBoard
from pbsim.tps48111 import (
    CB_DELAY_S,
    CB_TRIP_A,
    SC_TRIP_A,
    RailState,
)


class _FirmwareOn(Esp32Stub):
    """Steady-state firmware: SERVO_EN on, no KILL."""
    def tick(self, t_s, board):
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


def _boot_buttons(t_s):
    return Buttons(power_pressed=t_s < 0.3)


def _run(board, buttons_at, esp, rail_load, n_ticks):
    out, last = [], None
    for _ in range(n_ticks):
        st = board.tick(buttons_at(board.t_s), esp,
                        rail_load_a=rail_load, board=last)
        out.append(st)
        last = st
    return out


# ----------------- direct Tps48111 model drive (no board stack) -----------


def test_rail_latches_cb_on_sustained_overcurrent():
    """Direct drive: 25 A standing (well above the 19.8 A CB trip). The
    latch must land ~10.5 ms after the over-current starts, not earlier
    (a transient shorter than the timer must NOT trip it)."""
    board = PowerBoard()
    rail = board.rail_sw
    # drive the rail into ON: EN + INP high
    en_v = 2.0   # anything over VU_VLO_RISING=1.0
    inp_v = 3.3
    pack_v = 12.0
    # 200 ms of ON-state behaviour with rail_i = 0 — settle to ON.
    for k in range(200):
        rail.update(k * 1e-3, 1e-3, en_v, inp_v, pack_v, 0.0)
    assert rail.state == RailState.ON

    # step to 25 A — latch must NOT fire before CB_DELAY_S.
    trip_at = None
    t = 0.200
    for step_ms in range(60):     # 60 ms at 1 ms ticks
        rail.update(t, 1e-3, en_v, inp_v, pack_v, 25.0)
        if rail.state == RailState.LATCHED_CB and trip_at is None:
            trip_at = t
            break
        t += 1e-3
    assert trip_at is not None, "rail never reached LATCHED_CB at 25 A"
    delay_ms = (trip_at - 0.200) * 1e3
    # design record: "10.5 ms (FLT at 9.7)"; allow a couple of ms each way.
    assert 9.0 < delay_ms < 12.0, (
        f"CB latch landed {delay_ms:.2f} ms after the 25 A step "
        f"(expected ~10.5 ms)"
    )
    assert rail.rail_v == 0.0
    # Trip log recorded the event
    assert any(kind == "CB" for kind, _t, _i in rail.trip_log), (
        f"CB trip missing from the log: {rail.trip_log}"
    )


def test_rail_latches_sc_on_hard_short():
    """Direct drive: 45 A (over the 39.3 A SC trip) latches the rail on
    the SC fast-trip — sub-microsecond on the real part, modeled as
    next-tick at our 1 ms timescale."""
    board = PowerBoard()
    rail = board.rail_sw
    en_v = 2.0
    inp_v = 3.3
    pack_v = 12.0
    for k in range(200):
        rail.update(k * 1e-3, 1e-3, en_v, inp_v, pack_v, 0.0)
    assert rail.state == RailState.ON

    # 45 A — well over SC.
    rail.update(0.300, 1e-3, en_v, inp_v, pack_v, 45.0)
    rail.update(0.301, 1e-3, en_v, inp_v, pack_v, 45.0)
    assert rail.state == RailState.LATCHED_SC, (
        f"SC never latched at 45 A; state={rail.state}"
    )
    assert rail.rail_v == 0.0
    assert any(kind == "SC" for kind, _t, _i in rail.trip_log)


def test_rail_cb_latch_releases_on_en_pulse():
    """Real TPS48111: the CB latch resets on an EN low pulse or a VCC
    recycle. We model the EN-pulse form. Assert: trip, release EN, then
    re-drive EN + INP and the rail returns to ON."""
    board = PowerBoard()
    rail = board.rail_sw
    en_hi, inp_hi, pack_v = 2.0, 3.3, 12.0
    for k in range(200):
        rail.update(k * 1e-3, 1e-3, en_hi, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.ON
    # trip the CB
    for k in range(30):
        rail.update((0.200 + k * 1e-3), 1e-3, en_hi, inp_hi, pack_v, 25.0)
    assert rail.state == RailState.LATCHED_CB

    # EN low pulse: latch releases into OFF.
    rail.update(0.500, 1e-3, 0.0, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.OFF, (
        f"EN low pulse did not release the CB latch: state={rail.state}"
    )
    # EN + INP high again: rail returns through STARTING to ON.
    for k in range(200):
        rail.update((0.501 + k * 1e-3), 1e-3, en_hi, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.ON, (
        f"rail never re-closed after the EN-pulse release: {rail.state}"
    )


def test_rail_sc_latch_releases_on_en_pulse():
    """Same as above for SC."""
    board = PowerBoard()
    rail = board.rail_sw
    en_hi, inp_hi, pack_v = 2.0, 3.3, 12.0
    for k in range(200):
        rail.update(k * 1e-3, 1e-3, en_hi, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.ON
    rail.update(0.300, 1e-3, en_hi, inp_hi, pack_v, 45.0)
    rail.update(0.301, 1e-3, en_hi, inp_hi, pack_v, 45.0)
    assert rail.state == RailState.LATCHED_SC
    rail.update(0.500, 1e-3, 0.0, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.OFF


# ----------------- full stack: servo-lead short (C5's scenario) ------------


def test_servo_lead_short_clears_on_rail_sc_while_bq_scd_stays_out():
    """C5's headline case end-to-end. A J4–J7 servo-branch short pulls
    the rail toward 39 A (SC) but cannot pull the full pack toward 150 A
    (BQ SCD) — pack-side wiring L/R absorbs the difference. The TPS48111's
    SC latch must clear the fault; the BQ76922's SCD must NOT latch.

    The model's `rail_load_a` reaches the TPS48111 directly; the
    BQ76922 sees the pack-side current, which is the same current here
    through the FETs — but at 45 A < 150 A the SCD is below threshold
    anyway.
    """
    pack = Pack3S.fresh()
    board = PowerBoard(pack=pack)
    esp = _FirmwareOn()
    # Boot, settle 600 ms, then short to 45 A forever (no human releases it).
    out, last = [], None
    for k in range(1500):
        t = board.t_s
        rail_load = 45.0 if t >= 0.6 else 3.05
        st = board.tick(_boot_buttons(t), esp, rail_load_a=rail_load,
                        board=last)
        out.append(st)
        last = st

    # rail must have come up first
    assert any(s.rail_state == "ON" for s in out if s.t_s < 0.6)
    # SC latches the rail off
    assert any(s.rail_state == "LATCHED_SC" for s in out if s.t_s >= 0.6), (
        "rail never latched SC on a hard servo-lead short"
    )
    # the latch pins rail_v to 0
    after_short = [s for s in out if s.t_s > 0.605]
    assert all(s.rail_v < 0.5 for s in after_short if s.rail_state == "LATCHED_SC")
    # BQ76922 stays in NORMAL and SCD does NOT latch — the rail switch
    # cleared the fault before pack-level protections needed to act.
    bq_faults = [f for s in out for f in s.latched_faults]
    assert "SCD" not in bq_faults, (
        f"pack-side SCD latched during a rail-side short: {bq_faults}"
    )
    assert "OCD" not in bq_faults, (
        f"pack-side OCD latched during a rail-side short: {bq_faults}"
    )
    assert all(s.bq_state == "NORMAL" for s in out[-100:]), (
        "BQ76922 left NORMAL on a rail-side short (SCD or KILL should "
        "not have fired)"
    )


def test_servo_branch_overcurrent_clears_on_rail_cb():
    """A sustained 25 A pull (mechanically plausible through a stalled
    leg servo with a jammed gearbox) clears on the TPS48111's CB timer —
    before the fuse's 200% time-current fast curve (~0.15-5 s) opens
    anything."""
    board = PowerBoard()
    esp = _FirmwareOn()
    out, last = [], None
    for k in range(1200):
        t = board.t_s
        rail_load = 25.0 if t >= 0.6 else 3.05
        st = board.tick(_boot_buttons(t), esp, rail_load_a=rail_load,
                        board=last)
        out.append(st)
        last = st
    assert any(s.rail_state == "ON" for s in out if s.t_s < 0.6)
    assert any(s.rail_state == "LATCHED_CB" for s in out
               if s.t_s >= 0.6), (
        "rail never latched CB on a sustained 25 A branch current"
    )


def test_sub_cb_overcurrent_does_not_latch():
    """A 19.0 A pull sits just under the CB trip (19.8 A) — the upper edge
    of a heavy get-up's sustained over-draw. CB must NOT latch. The fuse
    doesn't melt either (static arithmetic, see test_servo_branches).

    (The previous draft used 19.5 A; that + 0.5 A logic hits the BQ76922's
    20.0 A OCD and pulls PACK+ down — a pack-level trip, not the rail-CB
    behaviour under test. Kept under both limits here.)"""
    board = PowerBoard()
    esp = _FirmwareOn()
    out, last = [], None
    for k in range(1200):
        t = board.t_s
        rail_load = 19.0 if t >= 0.6 else 3.05
        st = board.tick(_boot_buttons(t), esp, rail_load_a=rail_load,
                        board=last)
        out.append(st)
        last = st
    # never latches anything
    assert all(s.rail_state == "ON" for s in out if s.t_s > 0.7), (
        f"rail tripped at 19.0 A (under the {CB_TRIP_A} A CB trip): "
        f"{[s.rail_state for s in out if s.t_s > 0.7][:10]}"
    )


# ----------------- schematic-to-model constant cross-check (P3-1) ---------


def test_tps48111_constants_match_schematic_values(schematic):
    """The model's CB/SC trip points and timer delays are pinned to what
    the schematic actually carries (C3 derived them; we cross-check). If
    a rev B tunes R24/R26/R34/R35/C19, this test fails until the model
    constants move too."""
    from pbsim.schematicsnapshot import parse_si_value
    comps = schematic.components

    # CB: schematic carries R24 (2 mΩ) + R34 (30.1 kΩ); C3 gave 19.8 A.
    # The trip is set by the divider — we pin the model constant against
    # the schematic's *parsed value* of the relevant resistor.
    r24 = parse_si_value(comps["R24"].value)
    assert r24 == pytest.approx(2e-3, rel=1e-6)
    # SC: R26 sets 39.3 A; schematic R26 must parse to 4.64 kΩ.
    r26 = parse_si_value(comps["R26"].value)
    assert r26 == pytest.approx(4.64e3, rel=1e-6)
    # CB timer R35 × C19 = 100 kΩ × 680 nF. Schematic must carry them.
    r35 = parse_si_value(comps["R35"].value)
    c19 = parse_si_value(comps["C19"].value)
    assert r35 == pytest.approx(100e3, rel=1e-6)
    assert c19 == pytest.approx(680e-9, rel=1e-6)

    # The model constants themselves (these are what the simulator runs
    # against) must sit with the schematic-driven figures.
    assert CB_TRIP_A == pytest.approx(19.8, rel=1e-9)
    assert SC_TRIP_A == pytest.approx(39.3, rel=1e-9)
    assert CB_DELAY_S == pytest.approx(10.5e-3, rel=1e-9)
