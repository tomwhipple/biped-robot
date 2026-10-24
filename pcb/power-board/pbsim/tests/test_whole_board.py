"""Whole-board simulation (``pbsim/board``): the drawn netlist in ngspice with
pin models of U1 and U2, against the design's expected responses.

This is the gate's sample of the campaign. ``make pbfull`` runs all of it
(the 1152-case truth table, the sequences, the sweeps) and the record is
``docs/design-v6/2026-10-24-power-board-full-simulation.md``.

The runs go to a spawned process pool: ngspice's shared library is one
engine per process, and this process may already hold ``ngsim``'s.

Findings the record reports are pinned ``xfail(strict=True)``: the gate stays
green while the board has the defect, and a board change that fixes one
turns its test into a failure until the record and this file are updated.
"""
from __future__ import annotations

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor

import pytest

from pbsim import ngsim
from pbsim.board.netlist import find_kicad_cli

PASS = ["power_on_full_sequence", "power_tap_20ms", "kill_pulse_short",
        "estop_press_release_short", "rail_overcurrent_25A",
        "rail_cb_latch_reset_by_servo_en", "short_servo_lead_J4",
        "short_packplus_board", "power_while_kill_held_usb",
        "usb_only_pack_unplugged_outputs_driven", "full_pack_cov",
        "hotplug_xt60_after_balance"]
FINDINGS = {
    "shutdown_rail_dropped_first":
        "F-1: after KILL, SERVO+ (through Q3's body diode) and the General "
        "Driver's input capacitance hold LD up; the BQ76922 waits in soft "
        "shutdown and ignores POWER",
    "ocd_trip_then_power_press":
        "F-3: after an OCD cut POWER cannot switch the board back on",
    "rail_load_17A_ocd_low_corner":
        "F-4/F-5: OCD acts before the rail breaker; Q3's gate rides above "
        "U2's 16 V PU/PD−SRC rating as PACK+ collapses",
    "cuv_weak_cell_under_load":
        "F-6: the board switches itself back on after a CUV cut",
}
TRUTH = ["on-Ac-Bc-en_high-pwrR-usb0-gd_on-pack_both",
         "on-Ao-Bo-en_high-pwrR-usb0-gd_on-pack_both",
         "off-Ac-Bc-en_high-pwrH-usb1-gd_absent-pack_both",
         "on-Ac-Bc-en_hiz-pwrR-usb0-gd_on-pack_both",
         "on-Ac-Bc-en_high-pwrR-usb0-gd_on-pack_xt60_only",
         "on-Ac-Bc-en_high-pwrR-usb1-gd_sw1_off-pack_none"]

needs = pytest.mark.skipif(
    ngsim.find_lib() is None or find_kicad_cli() is None,
    reason="the whole-board simulation needs a libngspice and kicad-cli")


def _seq(name):
    from pbsim.board.sequences import run_named
    return run_named(name)


def _truth(name):
    from pbsim.board.truth import all_cases, run_case
    return run_case(next(c for c in all_cases() if c.name == name))


@pytest.fixture(scope="module")
def results():
    from pbsim.board.netlist import load
    load()
    ctx = mp.get_context("spawn")
    with ProcessPoolExecutor(max_workers=8, mp_context=ctx) as ex:
        seqs = {n: ex.submit(_seq, n) for n in PASS + list(FINDINGS)}
        truth = {n: ex.submit(_truth, n) for n in TRUTH}
        return ({n: f.result() for n, f in seqs.items()},
                {n: f.result() for n, f in truth.items()})


@needs
@pytest.mark.parametrize("name", TRUTH)
def test_truth_table_sample(results, name):
    r = results[1][name]
    assert not r["fail"], r["fail"]


@needs
@pytest.mark.parametrize("name", PASS)
def test_sequence_meets_design(results, name):
    r = results[0][name]
    assert not r["fail"], r["fail"]


@needs
@pytest.mark.parametrize("name", [
    pytest.param(n, marks=pytest.mark.xfail(strict=True, reason=why))
    for n, why in FINDINGS.items()])
def test_sequence_finding(results, name):
    r = results[0][name]
    assert not r["fail"], r["fail"]
