"""Final comparison: the robustness case set (both stances, 44 cases), the stance-gain margin, and the kd margin,
stance-only control vs stance + swing law.  Writes build/swing/final.json and prints markdown tables.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/swing-damping/final_eval.py
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import swing_eval as E, scan
from design_search import agg

LAWS = {
    "stance only (kd 0.03)":      (dict(kd=0.03), None),
    "stance only, kd 0.05":       (dict(kd=0.05), None),
    "preload 3 deg":              (dict(kd=0.03), dict(sr=1e-9, sb=3.0)),
    "preload 3 + hip rate 0.3":   (dict(kd=0.03), dict(sr=0.3, sh=1.0, sb=3.0, lim=8.0)),
    "preload 3 + hip rate 0.6":   (dict(kd=0.03), dict(sr=0.6, sh=1.0, sb=3.0, lim=8.0)),
    "preload 8/-8 + hip rate 0.3": (dict(kd=0.03), dict(sr=0.3, sh=1.0, sb=8.0, sba=-8.0, lim=8.0)),
}
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)), "build", "swing"); os.makedirs(OUT, exist_ok=True)
final = {}

print("## robustness (44 cases: 22 per stance)\n")
print("| law | pass | fell | tripped | rate rms med w1/w2/w3 | rate rms max w1/w2/w3 | err peak med / max | min clearance | feet touching (median fraction of hold) | max swing cmd |")
print("|---|---|---|---|---|---|---|---|---|---|")
cases = E.robust_cases()
for name, (ctl, sw) in LAWS.items():
    res = E.run_set(cases, ctl, sw)
    s = E.summ(res); final.setdefault("robust", {})[name] = s
    print(f"| {name} | {s['ok']}/{s['n']} | {s['fell']} | {s['trip']} | {'/'.join(map(str, s['rms_med']))} | {'/'.join(map(str, s['rms_max']))} | {s['err_med']} / {s['err_max']} deg | {s['clr_min']} mm | {s['contact_frac_med']} | {s['usw_max']} deg |", flush=True)

print("\n## nominal plant, 6 seeds x 2 stances: stance kd (swing law on/off)\n")
print("| stance kd | law | rate rms w1/w2/w3 (deg/s) | err peak max | unstable (err>6) | min clearance |")
print("|---|---|---|---|---|---|")
for kd in (0.03, 0.05, 0.07, 0.10, 0.15):
    for name in ("stance only (kd 0.03)", "preload 3 deg", "preload 3 + hip rate 0.3", "preload 3 + hip rate 0.6"):
        sw = LAWS[name][1]
        res = scan.pmap([(E.cal(seed=s, stance=st), dict(kd=kd), sw) for st in "RL" for s in range(6)])
        g, e, clr, gap, bad = agg(res); uns = sum(r["err_peak"] > 6 for r in res)
        final.setdefault("kd_margin", {})[f"{kd}|{name}"] = dict(rms=g.round(2).tolist(), err=e, unst=uns, clr=clr)
        print(f"| {kd} | {name.replace('stance only (kd 0.03)', 'none')} | {'/'.join(f'{x:.1f}' for x in g)} | {e:.1f} deg | {uns}/12 | {clr:.0f} mm |", flush=True)

print("\n## stance PID gains scaled together (kp 0.3, ki 1.0, kd 0.03 x scale), 6 seeds x 2 stances\n")
print("| scale | law | rate rms w1/w2/w3 | err peak max | unstable (err>6) |")
print("|---|---|---|---|---|")
for sc in (1.0, 1.5, 2.0, 2.5):
    for name in ("stance only (kd 0.03)", "preload 3 + hip rate 0.3"):
        sw = LAWS[name][1]
        res = scan.pmap([(E.cal(seed=s, stance=st), dict(kp=0.3 * sc, ki=1.0 * sc, kd=0.03 * sc), sw) for st in "RL" for s in range(6)])
        g, e, clr, gap, bad = agg(res); uns = sum(r["err_peak"] > 6 for r in res)
        final.setdefault("scale_margin", {})[f"{sc}|{name}"] = dict(rms=g.round(2).tolist(), err=e, unst=uns)
        print(f"| x{sc} | {name.replace('stance only (kd 0.03)', 'none')} | {'/'.join(f'{x:.1f}' for x in g)} | {e:.1f} deg | {uns}/12 |", flush=True)
json.dump(final, open(os.path.join(OUT, "final.json"), "w"), indent=1)
