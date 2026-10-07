"""Gradebook inspector — marimo notebook over the grade cards (design §7).

A READ-ONLY consumer of the gradebook's versioned artifacts: grade cards
(`grade_v{n}.json`), the manifest, and per-episode traces
(`traces/<scenario>_s<seed>.npz`, written by `eval_precision --trace`).
It runs no rollout, renders no video, drives no sim — it only reads what
the grader/referee already wrote.

Edit (authoring):
    sim/.venv/bin/python -m marimo edit sim/gradebook/inspect_gradebook.py --headless
Run (app mode; LAN-visible port, per the repo's ports convention):
    sim/.venv/bin/python -m marimo run sim/gradebook/inspect_gradebook.py \
        --host 0.0.0.0 --port 8085 --headless
"""
import marimo

__generated_with = "0.25.1"
app = marimo.App(width="full")


@app.cell
def _():
    import sys

    GB = "/home/claw/code/robot-gradebook/sim"
    RUNS = "/home/claw/code/robot/sim/runs"
    if GB not in sys.path:
        sys.path.insert(0, GB)
    return GB, RUNS


@app.cell
def _(RUNS):
    import json
    import os

    import altair as alt
    import marimo as mo
    import pandas as pd

    _rows = []
    for run in sorted(os.listdir(RUNS)):
        rdir = os.path.join(RUNS, run)
        if not os.path.isdir(rdir):
            continue
        for f in sorted(os.listdir(rdir)):
            if not (f.startswith("grade_v") and f.endswith(".json")):
                continue
            spec_v = int(f[len("grade_v"):-len(".json")])
            try:
                with open(os.path.join(rdir, f)) as _fh:
                    _g = json.load(_fh)
            except (OSError, json.JSONDecodeError):
                continue
            agg = _g.get("run_aggregate", {})

            def _val(a, key):
                v = a.get(key)
                return v.get("value") if isinstance(v, dict) else v

            _rows.append({
                "run": run, "spec_v": spec_v,
                "total": agg.get("mean_scenario_total"),
                "seed_pass": agg.get("seed_pass"),
                "fall_rate": _val(agg, "fall_rate"),
                "cot": _val(agg, "cot_locomotion"),
                "card": os.path.join(rdir, f),
            })
    df_cards = pd.DataFrame(_rows).sort_values(
        "total", ascending=False, na_position="last").reset_index(drop=True)
    runs_table = mo.ui.table(df_cards, selection="single",
                             label="graded runs (pick one — ranked by total)")
    _rank = (alt.Chart(df_cards.dropna(subset=["total"]))
             .mark_bar()
             .encode(
                 x=alt.X("total:Q", scale=alt.Scale(domain=[0, 1]),
                         title="mean scenario total"),
                 y=alt.Y("run:N", sort="-x"),
                 color=alt.Color("total:Q", scale=alt.Scale(
                     domain=[0, 1], scheme="redyellowgreen"), legend=None),
                 tooltip=["run", "spec_v", "seed_pass",
                          alt.Tooltip("total:Q", format=".3f")],
             )
             .properties(title="nightly ranking by normalized total",
                         height=220))
    _panel = mo.vstack([runs_table, _rank])
    _panel
    return alt, df_cards, json, mo, os, pd, runs_table


@app.cell
def _(json, mo, os, pd, runs_table):
    """Selected card → per-scenario criteria frames."""
    _sel = runs_table.value
    card = _sel.iloc[0]["card"] if _sel is not None and len(_sel) else None
    crit_wide = scen_df = None
    if card:
        with open(card) as _fh:
            _g = json.load(_fh)
        _rows = []
        for s, blk in _g.get("scenarios", {}).items():
            for c, v in blk.get("criteria", {}).items():
                if isinstance(v, dict):
                    _rows.append({"scenario": s, "criterion": c,
                                  "raw": v.get("value", v.get("raw")),
                                  "norm": v.get("norm", v.get("normalized")),
                                  "err": v.get("error")})
        scen_df = pd.DataFrame(_rows)
        crit_wide = (scen_df.pivot(index="scenario", columns="criterion",
                                   values="norm") if _rows else None)
    _head = mo.md(f"run card: `{card or '— none selected —'}`")
    _head
    return card, crit_wide, scen_df


@app.cell
def _(alt, crit_wide, mo, pd):
    """Scenario × criterion heatmap of normalized scores."""
    if crit_wide is None or crit_wide.empty:
        _heat = mo.md("*pick a run above*")
    else:
        _long = (crit_wide.reset_index()
                 .melt(id_vars="scenario", var_name="criterion",
                       value_name="norm")
                 .dropna(subset=["norm"]))
        _heat = (alt.Chart(_long).mark_rect().encode(
            x="criterion:N", y="scenario:N",
            color=alt.Color("norm:Q", scale=alt.Scale(
                domain=[0, 1], scheme="redyellowgreen")),
            tooltip=["scenario", "criterion",
                     alt.Tooltip("norm:Q", format=".3f")],
        ).properties(title="normalized criteria heatmap (0 bad — 1 good)",
                     height=480))
    _heat


@app.cell
def _(alt, crit_wide, mo, pd):
    """Per-scenario weighted totals — the selector's ranking view.
    (Weights mirror criteria_v1.yaml; single source of truth lives there.)"""
    if crit_wide is None or crit_wide.empty:
        _bar = mo.md("*pick a run above*")
    else:
        _wt = {"task_success": 3.0, "fall_rate": 2.0, "stability_wobble": 2.0,
               "cot": 1.5, "power": 0.5, "foot_slip": 1.0, "symmetry": 1.0,
               "swing_clearance": 1.0, "lift_quality": 1.0}
        _cols = [c for c in crit_wide.columns if c in _wt]
        if _cols:
            _tot = (crit_wide[_cols].fillna(0)
                    * pd.Series({c: _wt[c] for c in _cols})).sum(axis=1) \
                / sum(_wt[c] for c in _cols)
        else:
            _tot = crit_wide.mean(axis=1)
        _bar = (alt.Chart(_tot.rename("total")
                          .reset_index(names="scenario"))
                .mark_bar()
                .encode(
                    x=alt.X("total:Q", scale=alt.Scale(domain=[0, 1])),
                    y=alt.Y("scenario:N", sort="-x"),
                    color=alt.Color("total:Q", scale=alt.Scale(
                        domain=[0, 1], scheme="redyellowgreen"), legend=None),
                    tooltip=["scenario", alt.Tooltip("total:Q", format=".3f")],
                ).properties(title="per-scenario weighted totals (selector ranking)",
                             height=520))
    _bar


@app.cell
def _(card, mo, os):
    """Trace picker — for runs graded from `--trace` referee output."""
    _traces = []
    if card:
        _tdir = os.path.join(os.path.dirname(card), "traces")
        if os.path.isdir(_tdir):
            _traces = sorted(f for f in os.listdir(_tdir) if f.endswith(".npz"))
    if _traces:
        trace_pick = mo.ui.dropdown(_traces, value=_traces[0],
                                    label="trace file")
        _pick_panel = mo.vstack([mo.md("**traces** (written by `--trace`):"),
                                 trace_pick])
    else:
        trace_pick = None
        _pick_panel = mo.md("*no traces under this run — re-run the referee "
                            "with `--trace traces` to fill the trace views*")
    _pick_panel
    return (trace_pick,)


@app.cell
def _(card, mo, os, pd, trace_pick):
    """Trace loader via gradebook.trace.load(); `t` is already a row key."""
    trace_df = None
    trace_meta = None
    if trace_pick is not None and card:
        _path = os.path.join(os.path.dirname(card), "traces", trace_pick.value)
        from gradebook.trace import load as _tload
        _tr = _tload(_path)
        trace_meta = _tr["meta"]
        _keys = ("t", "up_z", "height", "planar", "watts", "foot_clear",
                 "recovered")
        _df = pd.DataFrame({k: _tr[k] for k in _keys if k in _tr})
        if len(_df):
            trace_df = _df
    _loaded = (mo.md("*no trace loaded*") if trace_df is None
               else mo.md(f"trace `{trace_pick.value}` — "
                          f"{trace_meta['scenario']} s{trace_meta['seed']}, "
                          f"{len(trace_df)} steps @ dt {trace_meta['dt']}"))
    _loaded
    return trace_df, trace_meta


@app.cell
def _(alt, mo, trace_df, trace_meta):
    """Trace series: up_z / height / planar / watts + recovered marker."""
    if trace_df is None:
        _series = mo.md("*—*")
    else:
        _base = alt.Chart(trace_df).encode(x=alt.X("t:Q", title="t (s)"))
        _charts = []
        for _col in ("up_z", "height", "planar", "watts"):
            if _col in trace_df.columns and trace_df[_col].notna().any():
                _charts.append(_base.mark_line().encode(
                    y=alt.Y(f"{_col}:Q", title=_col)))
        if "recovered" in trace_df.columns and trace_df["recovered"].gt(0).any():
            _rec = trace_df[trace_df["recovered"] > 0]
            _charts.append(alt.Chart(_rec).mark_rule(color="#c0392b")
                           .encode(x="t:Q"))
        _panel = alt.vconcat(*_charts) if _charts else None
        _title = (f"episode trace — {trace_meta['scenario']} "
                  f"s{trace_meta['seed']} · success={trace_meta['success']} · "
                  f"fell={trace_meta['fell']}")
        _series = (_panel.properties(title=_title) if _panel
                   else mo.md("*empty trace*"))
    _series


if __name__ == "__main__":
    app.run()