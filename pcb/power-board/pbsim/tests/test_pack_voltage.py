"""Part-2.4: main pack voltage — the BQ76922's protections on the 3S pack.

What must hold, per design §5.5 and the verification record (C3, C5, C13):

- OTP-set CUV threshold at 3.0 V/cell (below battguard's 3.3 V/cell
  landing, so the firmware lands the robot first; this IC is the
  firmware-independent backstop).
- OTP-set COV threshold at 4.2 V/cell (the LiPo chemistry limit).
- OCD at 20 A with a delay that clears the §2 get-up peaks (9.4–11.8 A).
- SCD at 150 A, fastest delay; this sits above the pack's
  lead-inductance-settled ~215 A onPACK+ hard-short current and below
  board-damaging transients (verification C5 explains why it MUST be at
  150 A: at lower thresholds the servo short trips it; at higher it
  stops protecting the pack).
- SUCD (the autonomous shutdown on stack voltage) at 9.0 V — again below
  battguard's VLAND, so the firmware gets the first word.
- Cell-imbalance excursion: one cell sagged to 2.8 V must open the
  discharge FET (CUV) *without* the ESP32 in the loop; the two healthy
  cells at 3.8 V must not clear the latch.

What we assert:
- every protection fires where the OTP intends;
- healthy pack stays silent at the thresholds' working band;
- the FETs' "on" state disappears when a protection latches;
- with a single-cell imbalance, the *stack* voltage stays in a healthy
  range (so a naive pack-level supervisor at 11.1 V would NOT trip) while
  the per-cell protection catches it. This is exactly the case the
  balance lead exists for.
"""
from __future__ import annotations

import numpy as np
import pytest

from pbsim.bq76922 import (BQ76922_I2C_ADDR, OTP_COV_V, OTP_CUV_V, OTP_OCD_A,
                          OTP_SCD_A, OTP_SUCD_STACK_V, Bq76922, BqState,
                          Protection)
from pbsim.pack import Pack3S, v_oc
from pbsim.sequencer_sim import BoardState, Buttons, Esp32Stub, PowerBoard


class _QuietEsp(Esp32Stub):
    def tick(self, t_s, board):
        return {"servo_en_v": 3.3, "kill_v": 0.0, "i2c_start_v": 3.3}


def _cold_buttons(t_s):
    return Buttons()


def _press_on_boot(t_s):
    return Buttons(power_pressed=t_s < 0.3)


# ------------- BQ76922 wakes on POWER and protections live in NORMAL ---------

def test_cuv_at_3_0v_per_cell():
    """Drive one cell to 3.0 V and confirm the part latches CUV. Driving
    it to 3.1 V must NOT latch."""
    bq = Bq76922()
    # boot it
    bq.state = BqState.NORMAL
    # Healthy taps (each cell ≈ 3.8 V): taps are CUMULATIVE at the J2 pins:
    # pin1=B-, pin2=cell1+, pin3=cell1++cell2+, pin4=+B (=pack).
    # Three cells at 3.8 V each → taps (0, 3.8, 7.6, 11.4).
    healthy = (0.0, 3.8, 7.6, 11.4)
    for k in range(200):
        bq.update(k*1e-3, 1e-3, healthy, pack_v=11.4, pack_i_a=0.5,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    assert not bq.any_latch(), "healthy pack tripped something"

    # Sag cell 1 to 3.1 V (still above the 3.0 V CUV): must stay clean.
    # taps: pin2 = 3.1; pin3 = 3.1+3.8 = 6.9; pin4 = 10.7
    v_low_but_ok = (0.0, 3.1, 6.9, 10.7)
    bq2 = Bq76922(); bq2.state = BqState.NORMAL
    for k in range(50):
        bq2.update(k*1e-3, 1e-3, v_low_but_ok, pack_v=10.7, pack_i_a=0.5,
                   ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    assert not bq2.any_latch(), "3.1 V cell tripped CUV (threshold too high)"

    # Sag cell 1 to exactly 3.0 V: CUV latches, FETs open.
    v_bad = (0.0, 3.0, 6.8, 10.6)   # cell 1 = 3.0 V, cells 2 & 3 = 3.8 V
    bq3 = Bq76922(); bq3.state = BqState.NORMAL
    for k in range(50):
        bq3.update(k*1e-3, 1e-3, v_bad, pack_v=10.6, pack_i_a=0.5,
                   ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq3.latched_faults]
    assert Protection.CUV in kinds, f"expected CUV, got {kinds}"
    assert Protection.COV not in kinds, "neighbour cells must not trip COV"
    assert bq3.state == BqState.NORMAL     # CUV latches, doesn't die
    assert not bq3.fets_closed


def test_cov_at_4_2v_per_cell():
    bq = Bq76922(); bq.state = BqState.NORMAL
    # cell 2 high
    v_hot = (0.0, 3.8, 7.6 + 4.25, 11.4 + 4.25 - 3.8)   # cell 2 at 4.25 V
    # recompute cleanly: cell i's V is taps[i+1] − taps[i]
    # want cell 2 at 4.25
    # taps: 0, 3.8 (cell1), 3.8+4.25=8.05 (cell1+cell2), 8.05+3.8=11.85
    v_hot = (0.0, 3.8, 8.05, 11.85)
    # COV has a programmable delay on the real part (modelled at 50 ms);
    # drive past it, not just to it.
    for k in range(200):
        bq.update(k*1e-3, 1e-3, v_hot, pack_v=11.85, pack_i_a=-1.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.COV in kinds, f"expected COV, got {kinds}"
    # COV opens the CHARGE FET; discharge continues so a full pack at
    # rest can still power the board (P2-2 / real BQ76922 semantics).
    assert not bq.charge_fet_closed
    assert bq.discharge_fet_closed, "COV must NOT block discharge"
    assert bq.fets_closed, "legacy fets_closed should equal discharge side"


def test_ocd_at_20a_with_delay():
    """Walking at 3.05 A RMS with 4.74 A transients must NOT trip OCD. A
    hard 25 A pull must trip it after the delay. The delay's purpose is
    to ignore gait transients (§2's 6.8 A ms-class)."""
    bq = Bq76922(); bq.state = BqState.NORMAL
    taps = (0.0, 3.8, 7.6, 11.4)
    t = 0.0
    # 4.74 A at walking: no trip
    for k in range(200):
        bq.update(t, 1e-3, taps, pack_v=11.4, pack_i_a=4.74,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0); t += 1e-3
    assert not bq.any_latch()
    # 11.8 A get-up for 2 s: must NOT trip OCD
    for k in range(2000):
        bq.update(t, 1e-3, taps, pack_v=11.4, pack_i_a=11.8,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0); t += 1e-3
    assert not bq.any_latch(), "get-up peak tripped OCD"
    # 25 A continuous: OCD fires
    for k in range(300):
        bq.update(t, 1e-3, taps, pack_v=11.4, pack_i_a=25.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0); t += 1e-3
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.OCD in kinds, f"expected OCD, got {kinds}"


def test_scd_at_150a_fastest():
    bq = Bq76922(); bq.state = BqState.NORMAL
    taps = (0.0, 3.8, 7.6, 11.4)
    # 100 A for 5 µs: BELOW SCD. The OCD delay (50 ms) keeps OCD out of it.
    for k in range(5):
        bq.update(k*1e-6, 1e-6, taps, pack_v=11.4, pack_i_a=100.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    assert not bq.any_latch(), f"100 A x 5 µs tripped something: {bq.latched_faults}"
    # 160 A for 50 µs: well above SCD
    for k in range(50):
        bq.update(k*1e-6, 1e-6, taps, pack_v=11.4, pack_i_a=160.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.SCD in kinds, f"expected SCD, got {kinds}"


def test_sucd_dies_below_the_stack_floor():
    """SUCD is the firmware-independent floor. Below 9.0 V stack the part
    must transition to SHUTDOWN (not just latch) — that is the auto-off
    the design's P4 requires as backstop."""
    bq = Bq76922(); bq.state = BqState.NORMAL
    # healthy above floor
    healthy = (0.0, 3.2, 6.4, 9.6)   # stack 9.6 V, all cells 3.2 V (above CUV)
    for k in range(50):
        bq.update(k*1e-3, 1e-3, healthy, pack_v=9.6, pack_i_a=0.5,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    assert not bq.any_latch(), \
        f"9.6 V (just above SUCD floor) tripped something: {bq.latched_faults}"
    # below floor
    low = (0.0, 2.95, 5.9, 8.85)   # stack 8.85, all cells 2.95
    for k in range(50):
        bq.update(k*1e-3, 1e-3, low, pack_v=8.85, pack_i_a=0.5,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.SUCD in kinds, f"expected SUCD, got {kinds}"
    assert bq.state == BqState.SHUTDOWN


def test_cell_imbalance_fires_cuv_while_stack_voltage_looks_fine():
    """The XHP-4 balance lead exists for this case: cell 2 sags to 2.8 V
    while cells 1 and 3 sit at 4.15 V (warm pack, hard use on one cell).
    The stack still reads 11.1 V — pack-level-only supervision (option C
    of the design) would NOT trip. The BQ76922's per-cell CUV must.
    """
    bq = Bq76922(); bq.state = BqState.NORMAL
    # taps: 0, 4.15, 4.15+2.8=6.95, 6.95+4.15=11.10
    taps = (0.0, 4.15, 6.95, 11.10)
    for k in range(50):
        bq.update(k*1e-3, 1e-3, taps, pack_v=11.10, pack_i_a=1.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.CUV in kinds
    # and the stack is 11.10 V: a pack-level guard sees nothing wrong.
    assert taps[3] == pytest.approx(11.10)


def test_otp_thresholds_match_schematic_intent(schematic):
    """Pin the OTP numbers against the *records* the schematic carries:
    the thresholds in the model are the ones §5.5 of the design states.

    This is not a check that the OTP program is correct — a real
    `bq76922.py` in firmware holds the burn table. It is a check that our
    *model* of the BQ76922 is configured the way the design says the part
    will be. If the design moves and our model doesn't (or vice versa),
    this fails first.
    """
    bq = Bq76922()
    assert bq.cov_v  == pytest.approx(4.20,  abs=1e-9)  # design: LiPo chemistry
    assert bq.cuv_v  == pytest.approx(3.00,  abs=1e-9)  # design §5.5, §10 V2
    assert bq.ocd_a  == pytest.approx(20.0,  abs=1e-9)
    assert bq.scd_a  == pytest.approx(150.0, abs=1e-9)
    assert bq.sucd_stack_v == pytest.approx(9.0, abs=1e-9)
    # The design's own statement that CUV is below battguard's 3.3 V/line:
    fw_land_v_per_cell = 3.3  # firmware/components/battguard/guard.h
    assert bq.cuv_v < fw_land_v_per_cell


def test_wake_consumption_is_microwatt_scale():
    """SHUTDOWN current is 1 µA typ (datasheet) — a 4200 mAh pack can
    sit with the board off for ~480 years before drain matters. We surface
    the design claim as an assertion so a regression of the model is loud.
    """
    bq = Bq76922()
    assert bq.state == BqState.SHUTDOWN
    assert bq.shutdown_current_ma == pytest.approx(0.001, rel=1e-9)
    pack = Pack3S.fresh()
    pack_i_before = 0.0 + 0.0  # nothing on
    # step 60 s with only the BQ76922 drawing
    pack.step(0.0 + bq.shutdown_current_ma * 1e-3, 60.0)
    # SOC drop after 60 s of 1 µA: 6e-3 mA·min / 60 = 1e-4 mAh min⁻¹
    # → essentially nothing. Assert numerically.
    assert pack.soc_min() > 0.9999


def test_fresh_full_pack_at_zero_load_powers_on_and_discharges_normally():
    """P2-2: a fresh pack at SOC=1.0 sits with v_oc = 4.20 V/cell — exactly
    at COV. With zero load and no charger, the real BQ76922 must NOT open
    the discharge FET; the bench test "plug it in, press POWER" must work.

    COV may latch (the cells genuinely are at the ceiling) but it must
    block the CHARGE side only; the discharge side stays live, and the
    hysteresis release clears the latch once the cells droop.
    """
    pack = Pack3S.fresh()
    assert pack.pack_v_open == pytest.approx(12.6, rel=1e-3)
    board = PowerBoard(pack=pack)
    esp = _QuietEsp()
    out, last = [], None
    for _ in range(800):
        st = board.tick(_press_on_boot(board.t_s), esp,
                        rail_load_a=0.0, board=last)
        out.append(st); last = st
    # board must wake
    assert any(s.bq_state == "NORMAL" for s in out), (
        "BQ76922 never woke on a fresh full pack"
    )
    # PACK+ must come up and STAY up: the discharge FET never opens even
    # though a full pack at rest rides exactly on COV.
    normal = [s for s in out if s.bq_state == "NORMAL"]
    assert all(s.bq_fets_closed for s in normal), (
        "discharge FET opened on a full pack at rest — COV must not "
        "block discharge (P2-2)"
    )
    assert all(s.packplus_v > 9.0 for s in normal)


def test_cov_hysteresis_releases_latch_once_cells_droop():
    """COV opens CHG only; a subsequent discharge sags cells under
    COV-hysteresis and the latch releases. A permanent COV would brick the
    board's charge path the moment the pack reaches 4.20 V/cell."""
    bq = Bq76922(); bq.state = BqState.NORMAL
    # over-voltage present long enough to trip the COV delay
    v_hot = (0.0, 3.8, 8.05, 11.85)   # cell 2 at 4.25 V
    for k in range(200):
        bq.update(k*1e-3, 1e-3, v_hot, pack_v=11.85, pack_i_a=-1.0,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    assert any(f.kind == Protection.COV for f in bq.latched_faults)
    assert not bq.charge_fet_closed
    # discharge sags cells back to 3.9 V each (above hysteresis-release
    # threshold of 4.20 − 0.05 = 4.15): latch clears.
    v_ok = (0.0, 3.9, 7.8, 11.7)
    for k in range(200, 300):
        bq.update(k*1e-3, 1e-3, v_ok, pack_v=11.7, pack_i_a=0.5,
                  ts2_pin_v=5.5, rst_shut_pin_v=0.0)
    kinds = [f.kind for f in bq.latched_faults]
    assert Protection.COV not in kinds, (
        f"COV latch did not release after cells drooped: {kinds}"
    )
    assert bq.charge_fet_closed
    assert bq.discharge_fet_closed


def test_esp32_is_never_required_for_cell_protection():
    """The whole point of the BQ76922: protections must work with the
    ESP32 never booted. Boot the board via POWER, drop one cell below
    3.0 V, and assert the protection latches with SERVO_EN held LOW the
    whole time (i.e. no firmware 'helping').
    """
    pack = Pack3S.fresh()
    pack.set_cell_soc(1, 0.01)   # cell 2 nearly empty: v_oc ≈ 2.85 V ⇒ CUV
    board = PowerBoard(pack=pack)
    class Dead(Esp32Stub):
        def tick(self, t, b):
            return {"servo_en_v": 0.0, "kill_v": 0.0, "i2c_start_v": 3.3}
    out, last = [], None
    for _ in range(800):
        st = board.tick(_press_on_boot(board.t_s), Dead(),
                        rail_load_a=2.0, board=last)
        out.append(st); last = st
    # The board woke on the button (hardware), the protection saw the low
    # cell, and the FETs opened. ESP32 never said anything (SERVO_EN low
    # throughout).
    assert any(s.bq_state == "NORMAL" for s in out)
    assert any("CUV" in f for s in out for f in s.latched_faults), (
        f"CUV never latched: faults={[(s.t_s, s.latched_faults) for s in out if s.latched_faults]}"
    )
    # and after the latch the FETs are open
    last_with_fault = [s for s in out if s.latched_faults][-1]
    assert not last_with_fault.bq_fets_closed
