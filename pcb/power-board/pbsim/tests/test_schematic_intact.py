"""Schematic-integrity tests: prove the parsed schematic carries the parts
the design record says it carries, with the values C3 verified.

This is the cheapest defense against the classic failure mode of
*behavioral* simulation: the tests and the schematic drifting apart. If a
later rev changes R24, R30/R31, R34, R26, C16/C17, or C19 without updating
the design record's verification table, these fail first.
"""
from __future__ import annotations

import pytest

from pbsim.schematicsnapshot import load, parse_si_value


# C3's verified values, keyed by ref. Read directly from the verification
# record's C3 table (2026-10-07-power-board-verification.md §C3) — these are
# not re-derived here so a mismatch means either the schematic or the record
# changed (both are review material).
C3_VALUES = {
    "R24": 2e-3,           # servo-rail sense
    "R25": 100.0,          # CS+ filter
    "R26": 4.64e3,         # ISC programming (sets short-circuit threshold)
    "R30": 560e3,          # EN/UVLO divider top
    "R31": 100e3,          # EN/UVLO divider bottom
    "R34": 30.1e3,         # circuit-breaker threshold (with R25)
    "R35": 100e3,          # latch-off pull
    "R36": 9.1e3,          # IMON resistor
    "C16": 330e-9,         # gate-slew start (change 1)
    "C17": 4.7e-6,         # bootstrap
    "C19": 680e-9,         # fault timer
    "C21": 1e-3,           # rail bulk capacitor
    "C22": 10e-6,          # rail bulk capacitor (MLCC, secondary)
    "R38": 47e3,           # RAIL_V divider top (change 3)
    "R39": 10e3,           # RAIL_V divider bottom
    "R12": 1e3,            # KILL series
    "R11": 10e3,           # KILL pulldown
    "R32": 1e3,            # SERVO_EN series (drives INP)
    "R33": 100e3,          # SERVO_EN pulldown
    "R18": 10e6,           # Q1 G-S zener bias (BQ EVM topology)
    "R21": 10e6,           # Q2 G-S zener bias
    "R22": 10e6,           # gate bias continuation
    "R20": 4.7e3,          # DSG off-path resistor (change 2)
    "R23": 10e3,           # gate series on Q4 clamp collector
    "R17": 5.1e3,          # CHG gate R
    "D3": 16.0,            # V (zener, gate clamps)
    "D5": 16.0,
    "D6": 16.0,
}


def test_schematic_parses_to_expected_parts(schematic):
    comps = schematic.components
    # sanity: the part census the README + BOM give
    for ref in ["U1", "U2", "Q1", "Q2", "Q3", "Q4", "Q5",
                "F1", "F2",
                "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8",
                "J9", "J10", "J11", "J12",
                "R1", "R24"]:
        assert ref in comps, f"missing {ref}"

    # the ICs the whole board exists around
    assert comps["U1"].lib == "power-board:BQ76922"
    assert comps["U2"].lib == "power-board:TPS48111-Q1"

    # fuse assignments per harness.md
    assert parse_si_value(comps["F1"].value) == pytest.approx(15.0)
    assert parse_si_value(comps["F2"].value) == pytest.approx(3.0)

    # the four servo branch headers + the data input
    assert comps["J4"].value == "Servo R leg"
    assert comps["J5"].value == "Servo R upper"
    assert comps["J6"].value == "Servo L leg"
    assert comps["J7"].value == "Servo L upper"
    assert comps["J8"].value == "Data from H5"
    assert comps["J10"].value == "POWER button"
    assert comps["J11"].value == "E-STOP button"

    # control nets that have to exist as labels (Part-2.2/2.3 sanity)
    need_labels = {"BQ_CHG", "BQ_DSG", "BQ_PACK", "BQ_LD",
                   "SERVO_EN", "ESTOP_RET", "ESTOP_N", "KILL",
                   "PWR_INT_N", "RAIL_V", "SERVO_IMON", "SERVO_FLT_N",
                   "SDA", "SCL"}
    assert need_labels.issubset(schematic.local_labels)

    # power nets that have to exist (per the architecture figure)
    need_power = {"BAT+", "BAT-", "PACK+", "SERVO+", "GND", "+3V3_HOST"}
    assert need_power.issubset(schematic.power_names)


def test_component_values_match_c3(schematic):
    comps = schematic.components
    for ref, expected in C3_VALUES.items():
        assert ref in comps, f"{ref} missing"
        got = parse_si_value(comps[ref].value)
        assert got == pytest.approx(expected, rel=1e-6), (
            f"{ref}: schematic {comps[ref].value!r} parses to {got}, "
            f"verification record expects {expected}"
        )


def test_j11_is_a_two_contact_latching_button(schematic):
    """J11 E-STOP: latching button with TWO normally-closed contacts.

    Harness.md J11: contact A pins 1-2 (in series with EN feed), contact B
    pins 3-4 (the ESP32's ESTOP_N input). The schematic must use a 4-pin
    header, whose net names line up with that reading."""
    j11 = schematic.components["J11"]
    assert j11.lib == "Connector_Generic:Conn_01x04"
    assert sorted(j11.pins) == ["1", "2", "3", "4"]


def test_j10_is_a_two_pin_momentary_button(schematic):
    j10 = schematic.components["J10"]
    assert j10.lib == "Connector_Generic:Conn_01x02"
    assert sorted(j10.pins) == ["1", "2"]


def test_j12_pin_count_for_control(schematic):
    j12 = schematic.components["J12"]
    assert j12.lib == "Connector_Generic:Conn_01x08"
    assert sorted(j12.pins, key=int) == [str(i) for i in range(1, 9)]


def test_servo_branch_headers_carry_three_pins(schematic):
    for ref in ("J4", "J5", "J6", "J7", "J8"):
        comp = schematic.components[ref]
        assert sorted(comp.pins) == ["1", "2", "3"], ref
