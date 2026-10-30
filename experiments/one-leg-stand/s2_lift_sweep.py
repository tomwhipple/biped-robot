"""The tables of 2026-10-30-s2-lift-sim.md: s2_lift_sim.py over fixes x plants x seeds (`fixes`), and
the model-error check of the candidates (`errors`). About 20 s each on 12 cores.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/s2_lift_sweep.py fixes [R|L] [--json out.json]
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/s2_lift_sweep.py errors
"""
import collections
import json
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s2_lift_sim as S  # noqa: E402

PLANTS = {"sole84": [], "sole60": ["--plant-set", "foot_w=0.060"],
          "sole60_mu04": ["--plant-set", "foot_w=0.060", "--mu", "0.4"]}
FIXES = {"open loop": ["--fix", "nobal"], "base": ["--fix", "base"],
         "(a) freeze I in moves": ["--fix", "freeze"],
         "(b) engage at 75% lift": ["--fix", "footoff", "--footoff-frac", "0.75"],
         "(b) engage at lift end": ["--fix", "footoff", "--footoff-frac", "1.0"],
         "(c) straight up first": ["--fix", "upfirst"],
         "(d) ref = centred roll": ["--fix", "centredref"],
         "(d) PD in lift, re-ref": ["--fix", "pdlift"]}
ERRORS = {"torso CoM 15 mm L": ["--torso-off", "0.015"], "torso CoM 15 mm R": ["--torso-off", "-0.015"],
          "torso x0.6": ["--torso-scale", "0.6"], "torso x1.4": ["--torso-scale", "1.4"],
          "shift x1.0": ["--lat", "1.0"], "shift x0.8": ["--lat", "0.8"], "shift x1.35": ["--lat", "1.35"],
          "play 5": ["--play", "5"]}


def _run(job):
    tag, argv = job
    return dict(tag=tag, res=S.main(argv + ["--quiet"]))


def _stat(rows, step):
    """worst over seeds: peak |err|, final u (+ = pushing outboard), CoM outboard at the end, min margin, falls"""
    pk, ue, co, mg, fl = [], [], [], [], 0
    for r in rows:
        res = r["res"]
        o = res["steps"].get(step)
        seg = res["abort_seg"] or ""
        if o is None or ((res["fell"] or res["abort_t"]) and seg.startswith(step)):
            fl += 1
        if o is None:
            continue
        pk.append(max(o["peak_in"] or 0, o["peak_out"] or 0))
        ue.append(o["u_end"])
        co.append(o["com_out_end"] if o["com_out_end"] is not None else float("nan"))
        if o["margin_min"] is not None:
            mg.append(o["margin_min"])
    f = lambda xs, fn: f"{fn(xs):5.1f}" if xs else "    -"
    return f"{f(pk, max)} {f(ue, lambda x: max(x, key=abs))} {f(co, max)} {f(mg, min)} {fl}/{len(rows)}"


def fixes(first="R", out=None):
    jobs = []
    for p, pa in PLANTS.items():
        for c, ca in FIXES.items():
            for mode in ("full", "s2only"):
                for seed in (0, 1, 2):
                    argv = pa + ca + ["--seed", str(seed), "--first", first] + (["--ctl-steps", "s2"] if mode == "s2only" else [])
                    jobs.append(((p, c, mode, seed), argv))
    with Pool(int(os.environ.get("NPROC", "12"))) as pool:
        res = pool.map(_run, jobs)
    if out:
        json.dump(res, open(out, "w"), default=float)
    g = collections.defaultdict(list)
    for r in res:
        p, c, mode, _ = r["tag"]
        g[(p, c, mode)].append(r)
    print(f"first stance {first}. Per step: peak|err| u_end comOut_end marginMin falls/seeds (deg, deg, mm, mm); "
          "s1 from the whole walk, s2 with s1 open loop; walk = the whole walk with the fix on both steps")
    for p in PLANTS:
        print(f"== {p}")
        for c in FIXES:
            full, s2o = g[(p, c, "full")], g[(p, c, "s2only")]
            walk = sum(1 for r in full if not r["res"]["fell"] and not r["res"]["abort_t"])
            print(f"  {c:24s} s1 {_stat(full, 's1')} | s2 {_stat(s2o, 's2')} | walk ok {walk}/{len(full)}")


def errors():
    cases = {k: FIXES[k] for k in ("open loop", "base", "(b) engage at lift end", "(d) PD in lift, re-ref")}
    jobs = [((p, e, c, f), PLANTS[p] + ERRORS[e] + cases[c] + ["--first", f])
            for p in ("sole84", "sole60") for e in ERRORS for c in cases for f in "RL"
            if not (e == "play 5" and p != "sole84")]
    with Pool(int(os.environ.get("NPROC", "12"))) as pool:
        res = pool.map(_run, jobs)
    for r in res:
        p, e, c, f = r["tag"]
        x = r["res"]
        ok = "ok" if not (x["fell"] or x["abort_t"]) else f"FAIL {x['abort_seg']}"
        st = "  ".join(f"{k} pk {max(v['peak_in'] or 0, v['peak_out'] or 0):4.1f} u {v['u_end']:+5.1f}" for k, v in x["steps"].items())
        print(f"{p:7s} {e:18s} {c:24s} {f} {ok:16s} {st}")


if __name__ == "__main__":
    a = sys.argv[1:]
    js = a[a.index("--json") + 1] if "--json" in a else None
    if a and a[0] == "errors":
        errors()
    else:
        fixes(a[1] if len(a) > 1 and a[1] in "RL" else "R", js)
