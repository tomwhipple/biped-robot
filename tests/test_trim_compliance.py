"""experiments/roll-balance/trim_compliance.py: the pure parts (the walk-fixed-effects slope, the tip counts) and
the scoring on a hand-made pair of rows."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
import trim_compliance as T  # noqa: E402


def test_walk_slope_removes_the_walk_means():
    x = np.array([0.0, 1.0, 2.0, 10.0, 11.0, 12.0])
    y = 0.5 * x + np.array([0, 0, 0, 100, 100, 100])       # a different level per walk, the same slope
    b, se, n = T.walk_slope(y, x, ["a", "a", "a", "b", "b", "b"])
    assert b == pytest.approx(0.5) and se == pytest.approx(0.0, abs=1e-9) and n == 6


def test_tip_counts_split_hits_and_false():
    assert T.tip_counts([1, 1, 0, 1], [1, 0, 1, 0]) == (3, 1, 2)


def test_vkey_matches_snap_replay():
    assert T.vkey(0.0) == "0" and T.vkey("c0.5@L_hip_roll") == "c0.5@L_hip_roll" and T.vkey("0.30") == "0.3"


def test_score_left_counts_false_tips_and_increment(monkeypatch):
    monkeypatch.setattr(T.SS, "add_stall", lambda r: dict(r))
    rob, sim = [], []
    for i in range(8):
        fast = i % 2
        ro = dict(stamp=f"w{i // 4}", n=i, side="L", timing="normal", k=3, shift_t10=0.2, plant="robot", delay=0,
                  ds_roll_peak=3.0 if fast else 1.0, stall_fast=fast, tip=int(i == 1), lean_in=2.0, T_lift=5.0 + i)
        rob.append(ro)
        sim.append(dict(ro, plant="g", ds_roll_peak=2.5 if fast else 1.0, tip_sim=int(i in (1, 2)),
                        lean_sim=0.5 * i))
    pk = T.pairs_by_key(rob + sim, "L")
    m = T.score(pk[("g", "0")], "L")
    assert m["n"] == 8 and m["incr"] == pytest.approx(1.5) and m["short_fast"] == pytest.approx(0.5)
    assert (m["tips"], m["tips_hit"], m["tips_false"]) == (2, 1, 1)
    assert m["lean_slope_T"] == pytest.approx(0.5)
    assert m["snap"] == pytest.approx(50.0)
    mr = T.score(pk[("robot", "0")], "L")
    assert mr["incr"] == pytest.approx(2.0) and mr["tips"] == 1 and mr["lean_slope_T"] == pytest.approx(0.0)


def test_paired_reads_the_authority_per_degree_of_removed_hike(monkeypatch):
    monkeypatch.setattr(T.SS, "add_stall", lambda r: dict(r))
    rows = []
    for i in range(6):
        hike = 3.0 + i
        ro = dict(stamp="w", n=i, side="R", timing="normal", k=3, shift_t10=0.2, plant="robot", delay=0,
                  ds_roll_peak=2.0, stall_fast=i % 2, prev_hike_td=hike)
        rows.append(ro)
        rows.append(dict(ro, plant="g", delay=0, ds_roll_peak=1.0, prev_e_td=-1.0 * hike))
        # half the correction removed: the error rises by 0.8 per deg of the removed half, the roll by 0.2
        rows.append(dict(ro, plant="g", delay="c0.5@L_hip_roll", ds_roll_peak=1.0 + 0.2 * 0.5 * hike,
                         prev_e_td=-1.0 * hike + 0.8 * 0.5 * hike))
    m = T.paired(rows, ("g", "0"), ("g", "c0.5@L_hip_roll"), 0.5)
    assert m["n"] == 6
    assert m["e_td"] == pytest.approx(-0.8) and m["roll"] == pytest.approx(-0.2)
    assert m["roll_fast"] == pytest.approx(-0.2) and m["roll_not"] == pytest.approx(-0.2)
