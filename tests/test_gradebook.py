"""Gradebook v1 tests: spec parse, normalization/weighting math, per-criterion
failure isolation (A4), A2 variant proof, A6 immutability, and the trace
schema round-trip. The parity gate (trace capture does not change referee
numbers) runs the referee on synthetic Drivers in tests/test_gradebook_parity.py.
"""
import json
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

from gradebook import card as card_mod                    # noqa: E402
from gradebook import grade as grade_mod                  # noqa: E402
from gradebook import spec as spec_mod                    # noqa: E402
from gradebook import trace as trace_mod                  # noqa: E402


# =================================================================
#  spec parse / validate
# =================================================================
def _base_spec():
    return {"version": 1, "suite": "test", "criteria": {
        "succ": {"type": "metric", "source": "scenario.successes / n",
                 "weight": 3.0, "score": {"direction": "higher_better",
                                          "normalize": [0, 1]}}}}


def test_spec_loads_shipped_v1(tmp_path):
    spec = spec_mod.load()
    assert spec["version"] == 1 and spec["suite"] == "referee_v2"
    assert set(spec["criteria"]) >= {"task_success", "fall_rate", "cot",
                                     "power", "stability_wobble", "foot_slip",
                                     "symmetry", "swing_clearance",
                                     "lift_quality"}
    # adjudicated: economy criteria carry completion conditioning
    assert spec["criteria"]["cot"]["condition"] == "completed"
    assert spec["criteria"]["power"]["condition"] == "completed"
    assert spec["criteria"]["symmetry"]["condition"] == "completed"
    # adjudicated: fall_rate is NOT loco-gated
    assert "applies_to" not in spec["criteria"]["fall_rate"]


def test_spec_rejects_missing_normalize(tmp_path):
    bad = _base_spec()
    del bad["criteria"]["succ"]["score"]["normalize"]
    p = tmp_path / "bad.yaml"
    p.write_text(__import__("yaml").safe_dump(bad))
    with pytest.raises(spec_mod.SpecError):
        spec_mod.load(str(p))


def test_spec_rejects_bad_direction(tmp_path):
    bad = _base_spec()
    bad["criteria"]["succ"]["score"]["direction"] = "sideways"
    p = tmp_path / "bad.yaml"
    p.write_text(__import__("yaml").safe_dump(bad))
    with pytest.raises(spec_mod.SpecError):
        spec_mod.load(str(p))


def test_spec_accepts_judged_types_but_grader_errors_them(tmp_path):
    """A4: rubric/check criteria are spec-legal but v1 can't grade them —
    the grader must surface a per-criterion error, never a fabricated norm."""
    spec_ = _base_spec()
    spec_["criteria"]["gait"] = {"type": "rubric", "backend": "claude",
                                 "levels": ["rigid", "ok", "smooth"],
                                 "weight": 1.0}
    p = tmp_path / "s.yaml"
    p.write_text(__import__("yaml").safe_dump(spec_))
    spec = spec_mod.load(str(p))
    card = {
        "line_1m": {"successes": 4, "fall_count": 0, "n": 4,
                    "locomotion": True, "episode_seconds": 14.0,
                    "metrics": {},
                    "shared": {"wobble_rms": {"mean": 1.0, "worst": 1.0}}},
        "summary": {"conditions": "x", "claim": "python", "stack": "python",
                    "plant": "x.xml", "seed_pass": "4/4",
                    "scenarios_all_pass": 1, "scenarios_run": 1},
    }
    grade, scens = grade_mod.grade_scorecard(card, spec, "r")
    cell = grade["scenarios"]["line_1m"]["criteria"]["gait"]
    assert "error" in cell and cell.get("norm") is None
    assert grade["complete"] is False


# =================================================================
#  normalization / weighting math
# =================================================================
def test_norm_windows():
    n = grade_mod.norm
    assert n(0.0, "higher_better", [0.0, 1.0]) == 0.0
    assert n(1.0, "higher_better", [0.0, 1.0]) == 1.0
    assert n(0.5, "higher_better", [0.0, 1.0]) == 0.5
    # lower better: window's low end (good) maps to 1, high end (bad) to 0
    assert n(2.0, "lower_better", [2.0, 45.0]) == 1.0
    assert n(45.0, "lower_better", [2.0, 45.0]) == 0.0
    assert n(23.5, "lower_better", [2.0, 45.0]) == 0.5
    # clamp
    assert n(-5.0, "higher_better", [0.0, 1.0]) == 0.0
    assert n(5.0, "higher_better", [0.0, 1.0]) == 1.0
    assert n(1.0, "lower_better", [2.0, 45.0]) == 1.0   # better than window
    assert n(99.0, "lower_better", [2.0, 45.0]) == 0.0
    # None-safe
    assert n(None, "higher_better", [0, 1]) is None


def _card(fall, cot, watts, sym):
    return {
        "line_1m": {"successes": 4 - fall, "fall_count": fall, "n": 4,
                    "locomotion": True, "episode_seconds": 14.0,
                    "metrics": {},
                    "shared": {"cot": {"mean": cot, "worst": cot},
                               "mean_watts": {"mean": watts, "worst": watts},
                               "symmetry": {"mean": sym, "worst": sym},
                               "foot_slip": {"mean": 0.04, "worst": 0.04},
                               "wobble_rms": {"mean": 1.0, "worst": 1.0}}},
        "summary": {"conditions": "x", "claim": "python", "stack": "python",
                    "plant": "x.xml", "seed_pass": f"{4-fall}/4",
                    "scenarios_all_pass": 0, "scenarios_run": 1},
    }


def test_completed_condition_zeroes_fell_episode_economy():
    """The CoT-on-falls pathology: a fell episode's 'better' CoT must not
    grade better. condition: completed forces norm=0.0 on all-fell scenarios."""
    spec = spec_mod.load()
    fell = _card(fall=4, cot=2.0, watts=10.0, sym=0.01)   # "great" numbers
    ok = _card(fall=0, cot=30.0, watts=120.0, sym=0.2)    # mediocre economy
    g_fell, _ = grade_mod.grade_scorecard(fell, spec, "fell")
    g_ok, _ = grade_mod.grade_scorecard(ok, spec, "ok")
    f = g_fell["scenarios"]["line_1m"]["criteria"]
    o = g_ok["scenarios"]["line_1m"]["criteria"]
    assert f["cot"]["norm"] == 0.0 and f["power"]["norm"] == 0.0 \
        and f["symmetry"]["norm"] == 0.0
    assert "conditioned" in f["cot"]["note"]
    assert o["cot"]["norm"] > 0.0 and o["power"]["norm"] < 1.0
    # weighted totals reflect it
    assert g_ok["scenarios"]["line_1m"]["total"] \
        > g_fell["scenarios"]["line_1m"]["total"]


def test_when_present_optional_column_skips_silently():
    """Cards from before the referee-clearance build lack swing columns —
    those criteria must be skipped, not errored (A4 is for real failures)."""
    spec = spec_mod.load()
    card = _card(fall=0, cot=30.0, watts=120.0, sym=0.2)
    grade, _ = grade_mod.grade_scorecard(card, spec, "r")
    cells = grade["scenarios"]["line_1m"]["criteria"]
    assert "swing_clearance" not in cells and "lift_quality" not in cells
    assert grade["complete"] is True    # absence is not a failure


def test_weighted_total_math():
    spec = spec_mod.load()
    # one criterion only, weight 2.5, norm 0.8 -> total must equal norm
    one = {"version": 2, "suite": "t", "criteria": {
        "x": {"type": "metric", "source": "shared.wobble_rms", "weight": 2.5,
              "score": {"direction": "lower_better", "normalize": [0.3, 2.3]}}}}
    card = _card(fall=0, cot=1.0, watts=1.0, sym=0.0)
    card["line_1m"]["shared"]["wobble_rms"]["mean"] = 1.3
    # monkey the spec dict through validate would reject path — use direct
    grade, _ = grade_mod.grade_scorecard(card, one, "r")
    cell = grade["scenarios"]["line_1m"]["criteria"]["x"]
    assert abs(cell["norm"] - 0.5) < 1e-3
    assert abs(cell["weighted"] - 1.25) < 1e-3
    assert abs(grade["scenarios"]["line_1m"]["total"] - 0.5) < 1e-3


# =================================================================
#  A2 config-as-data: two specs, two consistent cards, zero code edits
# =================================================================
def test_a2_variant_specs_on_same_card():
    card = _card(fall=1, cot=20.0, watts=80.0, sym=0.2)
    v1 = spec_mod.load()
    # variant: drop all economy criteria, re-weight task success
    v2 = {"version": 2, "suite": "t", "criteria": {
        "succ": {"type": "metric", "source": "scenario.successes / n",
                 "weight": 5.0, "score": {"direction": "higher_better",
                                          "normalize": [0, 1]}},
        "fr": {"type": "metric", "source": "scenario.fall_count / n",
               "weight": 1.0, "score": {"direction": "lower_better",
                                        "normalize": [0, 1]}}}}
    g1, _ = grade_mod.grade_scorecard(card, v1, "r")
    g2, _ = grade_mod.grade_scorecard(card, v2, "r")
    # v2 grades exactly its two criteria on identical data, no eval_precision
    # edits, and the totals differ from v1's (config drives the number)
    assert set(g2["scenarios"]["line_1m"]["criteria"]) == {"succ", "fr"}
    assert g1["spec_version"] == 1 and g2["spec_version"] == 2
    assert abs(g2["scenarios"]["line_1m"]["criteria"]["succ"]["norm"]
               - 0.75) < 1e-3
    assert g1["scenarios"]["line_1m"]["total"] \
        != g2["scenarios"]["line_1m"]["total"]


# =================================================================
#  A6 immutability: writing grade files leaves the scorecard untouched
# =================================================================
def test_a6_scorecard_byte_identical(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    card = _card(fall=1, cot=20.0, watts=80.0, sym=0.2)
    card_path = run / "scorecard.json"
    card_path.write_text(json.dumps(card, indent=2))
    before = card_path.read_bytes()
    import hashlib
    sha_before = hashlib.sha256(before).hexdigest()

    spec = spec_mod.load()
    grade, scens = grade_mod.grade_scorecard(card, spec, "run")
    card_mod.write_json(grade, str(run))
    card_mod.write_md(grade, str(run))
    card_mod.index_card(grade, str(run))

    assert card_path.read_bytes() == before
    assert hashlib.sha256(card_path.read_bytes()).hexdigest() == sha_before
    # grade artifacts exist alongside
    assert (run / "grade_v1.json").exists() and (run / "grade_v1.md").exists()
    assert (run / "grade_manifest.json").exists()
    # never writes scorecard.md either (there wasn't one; nothing created)
    assert not (run / "scorecard.md").exists()
    # manifest row is sane and re-indexing replaces, not duplicates
    card_mod.index_card(grade, str(run))
    man = json.load(open(run / "grade_manifest.json"))
    assert len(man["cards"]) == 1
    assert man["cards"][0]["spec_version"] == 1
    assert man["cards"][0]["card"] == "grade_v1.json"


# =================================================================
#  A5 card usability: md renders, FAIL visible, evidence resolves
# =================================================================
def test_a5_md_renders_with_fail_labels(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    card = _card(fall=4, cot=2.0, watts=10.0, sym=0.01)
    grade, scens = grade_mod.grade_scorecard(card, spec_mod.load(), "run")
    md_path = card_mod.write_md(grade, str(run))
    md = open(md_path).read()
    assert "# Grade v1" in md
    assert "line_1m" in md
    assert "| scenario | total |" in md
    assert "0.0 †" in md or "†" in md        # conditioned-fall cells labeled
    assert "A6" in md                        # additive artifact note


def test_incomplete_card_flagged_in_md(tmp_path):
    spec_ = _base_spec()
    spec_["criteria"]["gait"] = {"type": "rubric", "backend": "claude",
                                 "levels": ["a", "b"], "weight": 1.0}
    card = _card(fall=0, cot=30.0, watts=120.0, sym=0.2)
    grade, _ = grade_mod.grade_scorecard(card, spec_, "run")
    run = tmp_path / "run"
    run.mkdir()
    md_path = card_mod.write_md(grade, str(run))
    md = open(md_path).read()
    assert "INCOMPLETE CARD" in md
    assert "ERR" in md                       # the error cell, not a number


# =================================================================
#  trace schema round-trip
# =================================================================
def test_trace_save_load_round_trip(tmp_path):
    rows = [{"t": 0.04 * i, "x": 0.01 * i, "y": 0.0, "yaw": 0.1,
             "planar": 0.2, "wob2": 0.3, "watts": 10.0, "height": 0.31,
             "up_z": 0.99, "con_l": i % 2 == 0, "con_r": i % 2 == 1,
             "foot_err": 0.01, "foot_clear": 0.035, "foot_dx": 0.001,
             "foot_dz": -0.002, "cmd_lift": 0.0, "com_stance": 0.02,
             "recovered": 1.0}
            for i in range(50)]
    cmds = [(0.4, 0, 0, 1, 0)] * 50
    path = str(tmp_path / "traces" / "line_1m_s7.npz")
    trace_mod.save(path, scenario="line_1m", seed=7, dt=0.04,
                   episode_seconds=14.0, rows=rows, cmds=cmds,
                   fell=False, success=True,
                   events={"cross_t": 4.2, "cross_x": 1.01},
                   swing_summary={"swing_n": 5, "lift_frac": 0.31},
                   shared={"cot": 12.5}, headline="t=4.2s",
                   metrics={"time_to_1m": 3.2})
    t = trace_mod.load(path)
    assert t["up_z"].shape == (50,) and t["up_z"][0] == pytest.approx(0.99)
    assert t["con_l"].dtype == np.bool_
    assert t["cmd"].shape == (50, 7) and t["cmd"][0][0] == pytest.approx(0.4)
    assert t["meta"]["scenario"] == "line_1m" and t["meta"]["seed"] == 7
    assert t["meta"]["schema"] == trace_mod.SCHEMA_VERSION
    assert t["events"]["cross_t"] == 4.2
    assert t["meta"]["success"] is True and t["meta"]["fell"] is False


def test_trace_events_sanitize_numpy():
    import numpy as _np
    rows = [{"t": 0.04, "x": 0.0, "y": 0.0, "yaw": 0.0, "planar": 0.0,
             "wob2": 0.0, "watts": 0.0, "height": 0.3, "up_z": 1.0,
             "con_l": True, "con_r": True, "foot_err": 0.0,
             "foot_clear": 0.0, "foot_dx": 0.0, "foot_dz": 0.0,
             "cmd_lift": 0.0, "com_stance": 0.0, "recovered": 1.0}]
    ev = {"cross_t": _np.float64(4.2), "weird": object()}
    path = os.path.join(tempfile.mkdtemp(), "t.npz")
    trace_mod.save(path, scenario="x", seed=0, dt=0.04, episode_seconds=1.0,
                   rows=rows, cmds=[(0, 0, 0, 1, 0)], fell=False,
                   success=True, events=ev, swing_summary={}, shared={},
                   headline="", metrics={})
    t = trace_mod.load(path)
    assert t["events"]["cross_t"] == pytest.approx(4.2)
    assert isinstance(t["events"]["weird"], str)      # degraded, not lost
