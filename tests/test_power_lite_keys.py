"""Power lite's servo headers J4-J8 against the General Driver's H5 (pcb/power-lite/tools/netcheck.py --keys, #342).

A stock servo lead joins its two housings' bevelled corners, so J4-J8 must carry H5's nets in H5's order from
the bevelled corner: data there, ground at the far end. The board as first built had the bevel at GND.
"""
import sys
from pathlib import Path

import pytest

PL = Path(__file__).resolve().parents[1] / "pcb" / "power-lite"
sys.path.insert(0, str(PL / "tools"))
import netcheck as N  # noqa: E402
import parts as PD  # noqa: E402
import sexpr as S  # noqa: E402

BOARD = (PL / "power-lite.kicad_pcb").read_text()


def turned(board_text, refs):
    """The board with the named headers' silkscreen and fab drawings turned 180 degrees about pin 2 (pads,
    nets and copper as they are): the housings seated the other way round."""
    root = S.parse(board_text)[0]
    fps = N.footprints(root)
    for ref in refs:
        for g in S.find_all(fps[ref], "fp_line"):
            for key in ("start", "end"):
                pt = S.find(g, key)
                pt[1], pt[2] = 5.0 - float(pt[1]), -float(pt[2])
    return S.dump(root)


def test_the_board_is_keyed_to_h5():
    assert N.check_keys(BOARD) == []


def test_h5_puts_data_at_the_bevel():
    """The declared fact: H5's D (the bus) at its bevelled corner, G at the far end, V in the middle."""
    assert PD.H5_FROM_BEVEL == ("SERVO_DATA", "SERVOP", "GND")
    for ref in PD.SERVO_HEADERS:
        assert PD.parts[ref]["fp"] == PD.SPOX_FP
        assert PD.parts[ref]["nets"]["1"] == "SERVO_DATA" and PD.parts[ref]["nets"]["3"] == "GND"


@pytest.mark.parametrize("ref", PD.SERVO_HEADERS)
def test_a_header_seated_the_other_way_fails(ref):
    """The board as first built (2026-11-21): the bevel at pin 3, so H5's data wire lands on GND."""
    problems = N.check_keys(turned(BOARD, [ref]))
    assert any(p.startswith(f"{ref}: pad 3") and "SERVO_DATA" in p for p in problems), problems
    assert any(p.startswith(f"{ref}: pad 1") and "GND" in p for p in problems), problems
    assert not [p for p in problems if not p.startswith(ref)], problems


def test_swapped_nets_fail():
    """The other way to get it wrong: the outline right, data and ground swapped on the pads."""
    root = S.parse(BOARD)[0]
    for p in S.find_all(N.footprints(root)["J6"], "pad"):
        net = S.find(p, "net")
        if p[1] in ("1", "3"):
            net[-1] = S.Q("/GND" if p[1] == "1" else "/SERVO_DATA")
    problems = N.check_keys(S.dump(root))
    assert len(problems) == 2 and all(p.startswith("J6") for p in problems), problems


def test_j4_clears_h2s_screw_head():
    """The turned J4 sits 2.7 mm from H2's centre; the M2.5 button head (4.7 mm across) clears it by 0.35 mm.
    Moved 0.3 mm closer it must fail."""
    root = S.parse(BOARD)[0]
    fps = N.footprints(root)
    at = S.find(fps["H2"], "at")
    at[1] = float(at[1]) - 0.3
    problems = N.check_keys(S.dump(root))
    assert problems and all("H2's hardware" in p for p in problems), problems
