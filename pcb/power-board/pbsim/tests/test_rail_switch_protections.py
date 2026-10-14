"""P1-3: dynamic coverage of the TPS48111's own protections (CB, SC, UVLO)
plus the latch releases, and the model constants derived from the parsed
schematic (P3-1).

C5's core scenario — a servo-lead short clears on the TPS48111's SC while
the BQ76922's 150 A SCD stays out of it — is asserted here at the
sequence level. Static arithmetic against 19.8/39.3 lives in
test_servo_branches; this file is the dynamic side.

What "green" means:

- A direct over-current well above CB (e.g. 25 A standing) trips the
  rail's CB latch in ~10.5 ms; the rail falls to OFF-and-latched, the
  BQ76922 is UNAFFECTED, and PACK+ stays up.
- A short at C5's peak current trips SC on the next tick; the rail
  latches, the BQ76922's OCD and SCD do NOT latch, and PACK+ stays up
  (C5: "a servo-lead short leaves the robot powered").
- Once the over-current clears, toggling INP (SERVO_EN) or EN/UVLO below
  ENF (E-stop) releases the TPS latch and the rail can come back
  (datasheet §8.3.3.2).
- EN/UVLO turns the rail off when PACK+ sags below 7.33 V and back on
  above 7.79 V (C3's divider).
"""
from __future__ import annotations

import math

import pytest

from pbsim.bq76922 import OTP_SCD_A, RSENSE_PACK_MOHM
from pbsim.scenarios import BootAndHoldOn, press_power_until
from pbsim.sequencer_sim import (R9_OHM, R11_OHM, R12_OHM, R30_OHM,
                                 R31_OHM, R32_OHM, R33_OHM, R38_OHM,
                                 R39_OHM, PowerBoard)
from pbsim.tps48111 import (CB_DELAY_S, CB_TRIP_A, SC_TRIP_A,
                            VU_VLO_FALLING, VU_VLO_RISING, RailState)

# C5 (verification record): servo-lead short of 10 mΩ + 100 nH with 2 mF
# on the rail. Peak Q3 current 58-69 A, pack current 56-66 A, Q3 off in
# 5.9-8.3 µs.
C5_PEAK_Q3_A = 69.0
C5_PEAK_PACK_A = 66.0
# BQ76922 V(SCD_ACC): ±35 % of the nominal threshold (the wider of its
# two bands), so the 150 A SCD can trip as low as 97.5 A.
BQ_SCD_ACC = 0.35


def _boot_buttons(t_s):
    return press_power_until(0.3)(t_s)


def _settle_on(rail, pack_v=12.0):
    for k in range(200):
        rail.update(k * 1e-3, 1e-3, 2.0, 3.3, pack_v, 0.0)
    assert rail.state == RailState.ON


# ----------------- direct Tps48111 model drive (no board stack) -----------


def test_rail_latches_cb_on_sustained_overcurrent():
    """Direct drive: 25 A standing (well above the 19.8 A CB trip). The
    latch must land ~10.5 ms after the over-current starts, not earlier
    (a transient shorter than the timer must NOT trip it)."""
    rail = PowerBoard().rail_sw
    en_v, inp_v, pack_v = 2.0, 3.3, 12.0
    _settle_on(rail, pack_v)

    trip_at = None
    t = 0.200
    for _ in range(60):     # 60 ms at 1 ms ticks
        rail.update(t, 1e-3, en_v, inp_v, pack_v, 25.0)
        if rail.state == RailState.LATCHED_CB:
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
    assert any(kind == "CB" for kind, _t, _i in rail.trip_log), (
        f"CB trip missing from the log: {rail.trip_log}"
    )


def test_rail_latches_sc_on_hard_short():
    """Direct drive: C5's 69 A peak (over the 39.3 A SC trip) latches the
    rail on the SC fast-trip — about a microsecond on the real part,
    modelled as next-tick at our 1 ms timescale."""
    rail = PowerBoard().rail_sw
    _settle_on(rail)
    rail.update(0.300, 1e-3, 2.0, 3.3, 12.0, C5_PEAK_Q3_A)
    rail.update(0.301, 1e-3, 2.0, 3.3, 12.0, C5_PEAK_Q3_A)
    assert rail.state == RailState.LATCHED_SC, (
        f"SC never latched at {C5_PEAK_Q3_A} A; state={rail.state}"
    )
    assert rail.rail_v == 0.0
    assert any(kind == "SC" for kind, _t, _i in rail.trip_log)


@pytest.mark.parametrize("latch, amps", [(RailState.LATCHED_CB, 25.0),
                                         (RailState.LATCHED_SC, 45.0)])
@pytest.mark.parametrize("release", ["en_below_enf", "inp_low"])
def test_rail_latch_releases_on_en_or_inp_toggle(latch, amps, release):
    """Datasheet §8.3.3.2: "Toggle INP or EN/UVLO (below ENF) or power
    cycle VS ... to reset the latch." A latch with both held high stays
    latched; dropping either releases it into OFF, and with both high
    again the rail returns through STARTING to ON."""
    rail = PowerBoard().rail_sw
    en_hi, inp_hi, pack_v = 2.0, 3.3, 12.0
    _settle_on(rail, pack_v)
    for k in range(30):
        rail.update(0.200 + k * 1e-3, 1e-3, en_hi, inp_hi, pack_v, amps)
    assert rail.state == latch
    for k in range(50):     # fault gone, EN and INP still high: latched
        rail.update(0.300 + k * 1e-3, 1e-3, en_hi, inp_hi, pack_v, 0.0)
    assert rail.state == latch

    en, inp = (0.0, inp_hi) if release == "en_below_enf" else (en_hi, 0.0)
    rail.update(0.500, 1e-3, en, inp, pack_v, 0.0)
    assert rail.state == RailState.OFF, (
        f"{release} did not release {latch}: state={rail.state}"
    )
    for k in range(200):
        rail.update(0.501 + k * 1e-3, 1e-3, en_hi, inp_hi, pack_v, 0.0)
    assert rail.state == RailState.ON, (
        f"rail never re-closed after the {release} release: {rail.state}"
    )


def test_rail_follows_uvlo_with_hysteresis():
    """EN/UVLO is PACK+ through R30/R31: the rail opens when PACK+ sags
    below V(UVLOF) × (R30+R31)/R31 = 7.33 V and closes again only above
    V(UVLOR) × 6.6 = 7.79 V (C3: 7.39 V falling, 7.79 V rising)."""
    rail = PowerBoard().rail_sw
    k_div = R31_OHM / (R30_OHM + R31_OHM)
    t = [0.0]

    def hold(pack_v, ms):
        for _ in range(ms):
            rail.update(t[0], 1e-3, pack_v * k_div, 3.3, pack_v, 0.0)
            t[0] += 1e-3

    hold(12.0, 200)
    assert rail.state == RailState.ON
    hold(7.45, 20)
    assert rail.state == RailState.ON, "rail opened above the falling threshold"
    hold(7.20, 2)
    assert rail.state == RailState.OFF, "rail stayed on below 7.33 V of PACK+"
    hold(7.60, 100)
    assert rail.state == RailState.OFF, "rail re-closed inside the hysteresis band"
    hold(8.00, 100)
    assert rail.state == RailState.ON, "rail did not re-close above 7.79 V"


# ----------------- full stack: servo-lead short (C5's scenario) ------------


def test_servo_lead_short_clears_on_rail_sc_while_bq_scd_stays_out():
    """C5's headline case at the sequence level. A J4–J7 servo-lead short
    draws C5's 69 A peak through Q3; the TPS48111's SC latches the rail;
    the BQ76922's OCD (20 A, 50 ms) and SCD (150 A) do not latch, and
    PACK+ stays up, so the ESP32 and the Pi ride through it.

    The microsecond race itself — Q3 off in 5.9–8.3 µs, before the pack
    current passes 66 A — is C5's ngspice result; this model's 1 ms tick
    cannot resolve it. What this asserts is the consequence, plus the
    margin C5 leans on: 66 A sits under the lowest SCD trip the BQ76922's
    ±35 % accuracy allows."""
    assert C5_PEAK_PACK_A < OTP_SCD_A * (1 - BQ_SCD_ACC)

    board = PowerBoard()
    out = []
    last = None
    for _ in range(1500):
        t = board.t_s
        rail_load = C5_PEAK_Q3_A if t >= 0.6 else 3.05
        last = board.tick(_boot_buttons(t), BootAndHoldOn(),
                          rail_load_a=rail_load, board=last)
        out.append(last)

    assert any(s.rail_state == "ON" for s in out if s.t_s < 0.6)
    latched = [s for s in out if s.t_s >= 0.6 and s.rail_state == "LATCHED_SC"]
    assert latched, "rail never latched SC on a hard servo-lead short"
    assert latched[0].t_s <= 0.603
    assert all(s.rail_v < 0.5 for s in latched)
    bq_faults = [f for s in out for f in s.latched_faults]
    assert "SCD" not in bq_faults, (
        f"pack-side SCD latched during a rail-side short: {bq_faults}"
    )
    assert "OCD" not in bq_faults, (
        f"pack-side OCD latched during a rail-side short: {bq_faults}"
    )
    after = [s for s in out if s.t_s >= 0.6]
    assert all(s.bq_state == "NORMAL" and s.packplus_v > 9.0 for s in after), (
        "a rail-side short took PACK+ down"
    )


def test_servo_branch_overcurrent_clears_on_rail_cb():
    """A sustained 25 A pull (mechanically plausible through a stalled
    leg servo with a jammed gearbox) clears on the TPS48111's CB timer —
    before the fuse's 200% time-current fast curve (~0.15-5 s) opens
    anything."""
    board = PowerBoard()
    out, last = [], None
    for _ in range(1200):
        t = board.t_s
        rail_load = 25.0 if t >= 0.6 else 3.05
        last = board.tick(_boot_buttons(t), BootAndHoldOn(),
                          rail_load_a=rail_load, board=last)
        out.append(last)
    assert any(s.rail_state == "ON" for s in out if s.t_s < 0.6)
    assert any(s.rail_state == "LATCHED_CB" for s in out
               if s.t_s >= 0.6), (
        "rail never latched CB on a sustained 25 A branch current"
    )


def test_sub_cb_overcurrent_does_not_latch():
    """A 19.0 A pull sits just under the CB trip (19.8 A) — the upper edge
    of a heavy get-up's sustained over-draw. CB must NOT latch. The fuse
    doesn't melt either (static arithmetic, see test_servo_branches).

    (19.5 A + 0.5 A logic would reach the BQ76922's 20.0 A OCD and pull
    PACK+ down — a pack-level trip, not the rail-CB behaviour under test.
    Kept under both limits here.)"""
    board = PowerBoard()
    out, last = [], None
    for _ in range(1200):
        t = board.t_s
        rail_load = 19.0 if t >= 0.6 else 3.05
        last = board.tick(_boot_buttons(t), BootAndHoldOn(),
                          rail_load_a=rail_load, board=last)
        out.append(last)
    assert all(s.rail_state == "ON" for s in out if s.t_s > 0.7), (
        f"rail tripped at 19.0 A (under the {CB_TRIP_A} A CB trip): "
        f"{[s.rail_state for s in out if s.t_s > 0.7][:10]}"
    )


# ----------------- model constants from the schematic (P3-1) ---------------


def _val(schematic, ref):
    from pbsim.schematicsnapshot import parse_si_value
    return parse_si_value(schematic.components[ref].value)


def test_tps48111_constants_follow_from_the_schematic(schematic):
    """Each TPS48111 model constant, recomputed from the parsed schematic
    with the datasheet's (SLUSEE5E) equations. TI's calculator, which C3
    and the model use, rounds its constants differently, so the trips
    agree within 2 % and the timer within 3 %. A rev-B change to any of
    these parts fails here until the model moves too."""
    r24, r25, r26 = (_val(schematic, r) for r in ("R24", "R25", "R26"))
    r34, r35, c19 = (_val(schematic, r) for r in ("R34", "R35", "C19"))
    r30, r31 = _val(schematic, "R30"), _val(schematic, "R31")

    i_oc = 11.9 * r25 / (r24 * r34)                         # Eq. 6
    i_sc = (r26 + 464.0) * 15.6e-6 / r24                    # Eq. 11
    t_oc = c19 * r35 * math.log(1 / (1 - 1.2 / (r35 * 82e-6)))  # Eq. 10
    assert CB_TRIP_A == pytest.approx(i_oc, rel=0.02)
    assert SC_TRIP_A == pytest.approx(i_sc, rel=0.02)
    assert CB_DELAY_S == pytest.approx(t_oc, rel=0.03)

    k = (r30 + r31) / r31
    assert VU_VLO_RISING * k == pytest.approx(7.79, rel=0.01)   # C3
    assert VU_VLO_FALLING * k == pytest.approx(7.39, rel=0.01)  # C3


def test_board_model_resistors_match_the_schematic(schematic):
    """The dividers sequencer_sim computes with, and the pack shunt the
    BQ76922's 150 A SCD is set against, are the schematic's parts."""
    for ref, model in (("R30", R30_OHM), ("R31", R31_OHM),
                       ("R32", R32_OHM), ("R33", R33_OHM),
                       ("R38", R38_OHM), ("R39", R39_OHM),
                       ("R11", R11_OHM), ("R12", R12_OHM),
                       ("R9", R9_OHM), ("R1", RSENSE_PACK_MOHM * 1e-3)):
        assert _val(schematic, ref) == pytest.approx(model, rel=1e-6), ref
