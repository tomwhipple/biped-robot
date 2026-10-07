"""Criteria-spec loading and validation.

A spec is a versioned YAML artifact (docs/gradebook-design.md §4): the only
thing edited to change what's graded. Types: `metric` (deterministic code —
the only type v1 ships), `rubric` and `check` (judge criteria, spec-only
until slice 6 adds judge backends; accepted by this loader, rejected by the
grader with a per-criterion error — honest failure A4).
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SPEC = os.path.join(HERE, "criteria_v1.yaml")

METRIC_SOURCES = ("scenario.successes / n", "scenario.fall_count / n",
                  "shared.", "trace.")
CRITERION_TYPES = ("metric", "rubric", "check")
CONDITIONS = (None, "completed")


class SpecError(ValueError):
    """Malformed criteria spec — fails the whole load (nothing is graded
    from a spec we half-understood)."""


def load(path=None):
    path = path or DEFAULT_SPEC
    with open(path) as f:
        spec = yaml.safe_load(f)
    validate(spec, path)
    spec["_path"] = os.path.abspath(path)
    return spec


def validate(spec, path="<spec>"):
    if not isinstance(spec, dict):
        raise SpecError(f"{path}: top level must be a mapping")
    for key in ("version", "suite", "criteria"):
        if key not in spec:
            raise SpecError(f"{path}: missing required key '{key}'")
    if not isinstance(spec["version"], int) or spec["version"] < 1:
        raise SpecError(f"{path}: version must be a positive int")
    if not isinstance(spec["criteria"], dict) or not spec["criteria"]:
        raise SpecError(f"{path}: criteria must be a non-empty mapping")
    for name, cs in spec["criteria"].items():
        if not isinstance(cs, dict):
            raise SpecError(f"{path}.{name}: criterion must be a mapping")
        ctype = cs.get("type")
        if ctype not in CRITERION_TYPES:
            raise SpecError(f"{path}.{name}: unknown type {ctype!r} "
                            f"(want one of {CRITERION_TYPES})")
        cond = cs.get("condition")
        if cond not in CONDITIONS:
            raise SpecError(f"{path}.{name}: unknown condition {cond!r} "
                            f"(want one of {CONDITIONS})")
        w = cs.get("weight", (spec.get("defaults") or {}).get("weight", 1.0))
        if not isinstance(w, (int, float)) or w <= 0:
            raise SpecError(f"{path}.{name}: weight must be > 0 (got {w!r})")
        if ctype == "metric":
            src = cs.get("source", "")
            if not any(src == s or src.startswith(s) for s in METRIC_SOURCES):
                raise SpecError(f"{path}.{name}: unknown metric source "
                                f"{src!r} (want scenario.successes / n, "
                                f"scenario.fall_count / n, or a shared./trace. "
                                f"column)")
            score = cs.get("score", {})
            direction = score.get("direction", "higher_better")
            if direction not in ("higher_better", "lower_better"):
                raise SpecError(f"{path}.{name}: direction must be "
                                f"higher_better or lower_better")
            win = score.get("normalize")
            if (not isinstance(win, list) or len(win) != 2
                    or not all(isinstance(x, (int, float)) for x in win)
                    or win[0] == win[1]):
                raise SpecError(f"{path}.{name}: score.normalize must be "
                                f"[w0, w1] with w0 != w1 (got {win!r})")
        else:
            # rubric / check: judged criteria, no backend code in v1
            if not cs.get("backend"):
                raise SpecError(f"{path}.{name}: judged criteria must name "
                                f"a backend (design §4)")
            if ctype == "rubric" and not isinstance(cs.get("levels"), list):
                raise SpecError(f"{path}.{name}: rubric needs a levels list")
            if ctype == "check" and not cs.get("statement"):
                raise SpecError(f"{path}.{name}: check needs a statement")
    return spec
