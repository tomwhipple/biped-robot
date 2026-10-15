"""Grading engine (v1, metric-only): criteria YAML -> typed answers.

Reads a run's scorecard{,.json} (never writes it — A6), evaluates each
metric criterion from the aggregate columns, normalizes into [0, 1], weights
and aggregates, and returns the grade-card dict. The renderer (card.py)
turns the dict into grade_v{n}.json + grade_v{n}.md.

Honest failure (A4): a criterion that can't produce a number (column absent
and not skippable, judged type with no backend) carries an `error` field and
NO `norm`; the card's aggregate is computed over what remains and flagged
`complete: false`. Numbers are never fabricated.

Completion conditioning (design §6, CoT-on-falls pathology): a criterion
with `condition: completed` gets norm 0.0 (the window's worst end) on
scenarios where every episode fell, because a short fell episode's economy
misleads. A *partial* fall rate is real signal: the scorecard's across-seed
mean mixes both, so we grade it as-is.
"""
import json
import os

from . import spec as spec_mod


def norm(value, direction, window, lo=0.0, hi=1.0):
    """Normalize raw value into [lo, hi]; None-safe."""
    if value is None:
        return None
    w0, w1 = window
    span = (w1 - w0) or 1.0
    if direction == "higher_better":
        x = (value - w0) / span
    else:
        x = (w1 - value) / span
    return max(lo, min(hi, x))


def _round(v, nd=4):
    return round(v, nd) if v is not None else None


def _crit_answer(name, cs, block, n):
    """One criterion's typed answer for one scenario.

    Returns dict with keys among: value (raw), norm, weighted, note, error.
    A dict without 'norm' contributes nothing to aggregates (A4).
    """
    src = cs.get("source", "")
    ctype = cs.get("type")
    if ctype != "metric":
        return {"error": f"judge backend for {ctype!r} criterion not built "
                         f"(v1 is metric-only; design slice 6)"}

    completed = (n - block.get("fall_count", 0)) / n >= 1.0

    # ---- fetch the raw column value -------------------------------------
    if src == "scenario.successes / n":
        raw = block.get("successes", 0) / n
    elif src == "scenario.fall_count / n":
        raw = block.get("fall_count", 0) / n
    elif src.startswith("shared."):
        key = src.split(".", 1)[1]
        sh = block.get("shared", {}).get(key)
        if sh is None or sh.get("mean") is None:
            if cs.get("when_present", False):
                return {"note": "optional column absent in card", "skip": True}
            return {"error": f"column 'shared.{key}' absent in scorecard",
                    "value": None}
        raw = sh["mean"]
    elif src.startswith("trace."):
        return {"error": "trace-level criteria arrive with the trace "
                         "capture slice (design §9.1)"}
    else:                                            # pragma: no cover
        return {"error": f"unhandled source {src!r}"}  # validate() catches

    # ---- completion conditioning -----------------------------------------
    sc = cs.get("score", {})
    if cs.get("condition") == "completed" and not completed:
        return {"value": _round(raw), "norm": 0.0,
                "note": "all seeds fell: economy-on-a-fell-episode misleads "
                        "(design §6); conditioned to worst-case norm"}

    nv = norm(raw, sc.get("direction", "higher_better"),
              sc.get("normalize", [0, 1]))
    return {"value": _round(raw), "norm": _round(nv)}


def grade_scorecard(card, spec, run_name):
    """Compile spec -> questions -> answers over one scorecard dict."""
    crit_specs = spec["criteria"]
    default_w = (spec.get("defaults") or {}).get("weight", 1.0)

    scenarios = [k for k, v in card.items()
                 if isinstance(v, dict) and "successes" in v]
    summary = card.get("summary", {})
    not_applicable = {k: v["not_applicable"] for k, v in card.items()
                      if isinstance(v, dict) and "not_applicable" in v}

    rows = {}
    for scen in scenarios:
        block = card[scen]
        n = block.get("n", 1) or 1
        per = {}
        for name, cs in crit_specs.items():
            ans = _crit_answer(name, cs, block, n)
            if ans.pop("skip", False):
                continue
            per[name] = ans
        wsum, tot = 0.0, 0.0
        for name, cs in crit_specs.items():
            v = per.get(name)
            if not v or v.get("norm") is None:
                continue
            w = cs.get("weight", default_w)
            v["weighted"] = _round(w * v["norm"])
            wsum += w
            tot += w * v["norm"]
        rows[scen] = {"criteria": per,
                      "total": _round(tot / wsum) if wsum else None,
                      "registry": scen,
                      "locomotion": bool(block.get("locomotion"))}

    scen_scores = {s: r["total"] for s, r in rows.items()
                   if r["total"] is not None}
    run_total = (_round(sum(scen_scores.values()) / len(scen_scores))
                 if scen_scores else None)
    complete = not any(
        isinstance(v, dict) and "error" in v
        for r in rows.values() for v in r["criteria"].values())

    def _fam_total(prefix):
        vals = [r["total"] for r in rows.values()
                if r["locomotion"] == prefix and r["total"] is not None]
        return _round(sum(vals) / len(vals)) if vals else None

    grade = {
        "spec_version": spec["version"],
        "suite": spec["suite"],
        "spec_path": spec.get("_path"),
        "run": run_name,
        "graded_from": "scorecard",
        "graded_at_conditions": summary.get("conditions", ""),
        "claim": summary.get("claim"),
        "stack": summary.get("stack"),
        "plant": summary.get("plant"),
        "complete": complete,
        "scenarios": rows,
        "not_applicable": not_applicable,
        "run_aggregate": {
            "mean_scenario_total": run_total,
            "locomotion_total": _fam_total(True),
            "non_locomotion_total": _fam_total(False),
            "seed_pass": summary.get("seed_pass"),
            "scenarios_all_pass": summary.get("scenarios_all_pass"),
            "scenarios_run": summary.get("scenarios_run"),
            "fall_rate": {"value": summary.get("fall_rate"),
                          "norm": _round(norm(
                              summary.get("fall_rate"), "lower_better",
                              crit_specs.get("fall_rate", {})
                              .get("score", {}).get("normalize", [0, 1]))),
                          "ref_criterion": "fall_rate"},
            "cot_locomotion": {"value": summary.get("cot_locomotion"),
                               "ref_criterion": "cot"},
            "wobble_rms_mean": {"value": summary.get("wobble_rms_mean"),
                                "ref_criterion": "stability_wobble"},
            "foot_slip_mean": {"value": summary.get("foot_slip_mean"),
                               "ref_criterion": "foot_slip"},
            "symmetry_locomotion": {"value": summary.get("symmetry_locomotion"),
                                    "ref_criterion": "symmetry"},
        },
        "evidence": {
            "scorecard": "scorecard.json",
            "episode_seconds_per_scenario": {
                s: card[s].get("episode_seconds") for s in scenarios},
        },
        "note": "v1 grades aggregate columns from the existing scorecard; "
                "trace-level criteria arrive with trace capture (design "
                "§9.1). The scorecard file itself is never written (A6).",
    }
    return grade, scenarios


def grade_run(run_dir, spec):
    card_path = os.path.join(run_dir, "scorecard.json")
    with open(card_path) as f:
        card = json.load(f)
    run_name = os.path.basename(os.path.normpath(run_dir))
    return grade_scorecard(card, spec, run_name)
