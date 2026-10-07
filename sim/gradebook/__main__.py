"""CLI: python -m gradebook <run_dir> [criteria.yaml] [run_dir ...]

Grades each run's scorecard.json with the criteria spec (default: the
in-package criteria_v1.yaml), writes grade_v{n}.json + .md + updates
grade_manifest.json. Additive only — the scorecard files are read, never
written (A6).
"""
import sys

from . import card as card_mod
from . import grade as grade_mod
from . import spec as spec_mod


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        print((__doc__ or "python -m gradebook <run_dir> [criteria.yaml]").strip())
        return 2
    spec_path = spec_mod.DEFAULT_SPEC
    run_dirs = []
    for a in args:
        if a.endswith((".yaml", ".yml")):
            spec_path = a
        else:
            run_dirs.append(a)
    spec = spec_mod.load(spec_path)
    rc = 0
    for run_dir in run_dirs:
        try:
            grade, scenarios = grade_mod.grade_run(run_dir, spec)
        except FileNotFoundError:
            print(f"error: no scorecard.json under {run_dir}", file=sys.stderr)
            rc = 1
            continue
        jp = card_mod.write_json(grade, run_dir)
        mp = card_mod.write_md(grade, run_dir)
        man = card_mod.index_card(grade, run_dir)
        flag = "" if grade["complete"] else "  [INCOMPLETE — see card errors]"
        print(f"graded {grade['run']}  ->  {jp}\n"
              f"      {mp}\n"
              f"      {man}\n"
              f"  {len(scenarios)} scenarios · mean scenario total "
              f"{grade['run_aggregate']['mean_scenario_total']}{flag}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
