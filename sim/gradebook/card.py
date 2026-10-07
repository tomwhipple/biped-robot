"""Grade-card writers (design §5) + the manifest indexer.

Two documents per run, written beside the scorecard the grader READS
(never writes):
  grade_v{spec}.json  — machine: nightly selector, RL adapter, dashboards
  grade_v{spec}.md    — human referee read; FAIL criteria labeled FAIL

`grade_manifest.json` indexes one row per card per spec_version so old
cards stay retrievable across spec versions.
"""
import json
import os


def card_paths(run_dir, spec_version):
    base = os.path.join(run_dir, f"grade_v{spec_version}")
    return base + ".json", base + ".md"


def write_json(grade, run_dir):
    path, _ = card_paths(run_dir, grade["spec_version"])
    with open(path, "w") as f:
        json.dump(grade, f, indent=1)
    return path


def write_md(grade, run_dir):
    spec_version = grade["spec_version"]
    _, path = card_paths(run_dir, spec_version)

    agg = grade["run_aggregate"]
    scen = grade["scenarios"]
    crit_names = sorted({c for s in scen for c in scen[s]["criteria"]})
    weights = {}
    # surface weights from any criterion's first appearance
    for s in scen:
        for c, v in scen[s]["criteria"].items():
            weights.setdefault(c, v.get("weight"))

    lines = [f"# Grade v{spec_version} — {grade['run']}",
             "",
             f"plant `{grade['plant']}` · claim `{grade['claim']}` · "
             f"stack `{grade['stack']}`",
             f"_{grade['graded_at_conditions']}_",
             ""]
    if not grade["complete"]:
        lines.append("**INCOMPLETE CARD** — one or more criteria failed "
                     "honestly (see `error` cells; A4: no fabricated "
                     "numbers). Aggregate below is over the criteria that "
                     "answered.")
        lines.append("")
    lines.append(f"seed_pass {agg['seed_pass']} · "
                 f"scenarios all-pass {agg['scenarios_all_pass']}/"
                 f"{agg['scenarios_run']} · "
                 f"mean scenario total **{agg['mean_scenario_total']}**")
    if agg.get("locomotion_total") is not None:
        lines.append(f"locomotion total {agg['locomotion_total']} · "
                     f"non-locomotion total {agg['non_locomotion_total']}")
    fr = agg.get("fall_rate", {})
    if fr.get("value") is not None:
        lines.append(f"fall_rate {fr['value']} (norm {fr.get('norm')})")
    lines.append("")

    header = "| scenario | total | " + " | ".join(crit_names) + " |"
    lines.append(header)
    lines.append("|---|" + "---|" * (len(crit_names) + 1))
    for s in sorted(scen):
        row = scen[s]
        vals = []
        for c in crit_names:
            v = row["criteria"].get(c)
            if v is None:
                vals.append("-")                # when_present skip
            elif "error" in v:
                vals.append("ERR")
            elif v.get("norm") is None:
                vals.append("n/a")
            else:
                cell = f"{v['norm']}"
                if v.get("note", "").startswith("all seeds fell"):
                    cell += " †"
                vals.append(cell)
        total = row["total"] if row["total"] is not None else "n/a"
        lines.append(f"| {s} | {total} | " + " | ".join(vals) + " |")

    if grade.get("not_applicable"):
        lines += ["", "_not applicable on this plant:_ " +
                  "; ".join(f"`{k}` ({v})"
                            for k, v in sorted(grade["not_applicable"].items()))]

    lines += ["",
              "† conditioned to worst-case norm: every seed fell, so an "
              "economy number would mislead (design §6).",
              "",
              f"_norm ∈ [0,1]; higher is better within each cell after "
              f"direction-aware normalization. Spec: "
              f"`{grade.get('spec_path') or '<inline spec dict>'}` "
              f"(v{spec_version}). "
              f"total = Σ w·norm / Σ w._",
              "",
              "_additive artifact: the frozen `scorecard.{json,md}` beside "
              "this card is byte-identical after grading (acceptance A6)._"]
    md = "\n".join(lines) + "\n"
    with open(path, "w") as f:
        f.write(md)
    return path


# =====================================================================
#  manifest (design §5): one row per card, indexed by spec_version so
#  cards across spec versions stay retrievable.
# =====================================================================
MANIFEST = "grade_manifest.json"


def load_manifest(run_dir):
    path = os.path.join(run_dir, MANIFEST)
    if not os.path.exists(path):
        return {"cards": []}
    with open(path) as f:
        return json.load(f)


def index_card(grade, run_dir):
    """Add (or replace) this grade's row in run_dir/grade_manifest.json."""
    man = load_manifest(run_dir)
    row = {
        "spec_version": grade["spec_version"],
        "suite": grade["suite"],
        "run": grade["run"],
        "card": f"grade_v{grade['spec_version']}.json",
        "card_md": f"grade_v{grade['spec_version']}.md",
        "complete": grade["complete"],
        "scenario_count": len(grade["scenarios"]),
        "mean_scenario_total": grade["run_aggregate"]["mean_scenario_total"],
        "seed_pass": grade["run_aggregate"]["seed_pass"],
    }
    man["cards"] = [c for c in man["cards"]
                    if c.get("spec_version") != grade["spec_version"]]
    man["cards"].append(row)
    man["cards"].sort(key=lambda c: c.get("spec_version", 0))
    path = os.path.join(run_dir, MANIFEST)
    with open(path, "w") as f:
        json.dump(man, f, indent=1)
    return path
