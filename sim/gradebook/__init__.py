"""Gradebook — the configurable grader that replaces the LLM referee.

Two halves:

* **Capture** lives in the referee (`sim/mjx/eval_precision.py --trace`):
  every episode's `Driver.rows` (state series at control dt — up_z, height,
  planar, watts, contacts, foot metrics) plus the command schedule and the
  frozen verdict are written to `runs/<run>/traces/<scenario>_s{seed}.npz`.
  The Trace is the load-bearing interface: criteria can only grade what the
  Trace carries, so the schema is part of the design (see
  docs/gradebook-design.md §3).

* **Grading** lives here: a versioned criteria YAML compiles into typed
  questions per scenario, deterministic metric evaluators answer them from
  the scorecard columns (v1) or the trace (trace-level criteria, later
  slices), and the answers land in `grade_v{spec}.json` + `.md` beside the
  scorecard. Grading is ADDITIVE — the grader never writes a scorecard
  file (acceptance A6).

Usage:
  python -m gradebook <run_dir> [criteria.yaml] [... more run_dirs]
  python -m gradebook --manifest <run_dir> [run_dir ...]
"""
