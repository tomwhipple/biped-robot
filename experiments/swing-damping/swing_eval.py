"""Evaluate swing-leg laws: nominal gain sweeps and the robustness case set (both stances).

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/swing-damping/swing_eval.py grid  --json build/swing/grid.json
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/swing-damping/swing_eval.py robust --sw '{"sr":0.1,"sh":1.0}' 
"""
import argparse, itertools, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import scan
import swing_sim as S

CAL = json.load(open(os.path.join(HERE, "calibrated.json"))) if os.path.exists(os.path.join(HERE, "calibrated.json")) else {}


def cal(**kw):
    c = dict(CAL); c.update(kw)
    if isinstance(c.get("kick_dir"), list): c["kick_dir"] = tuple(c["kick_dir"])
    if isinstance(c.get("pad_solref"), list): c["pad_solref"] = tuple(c["pad_solref"])
    return c


def robust_cases():
    """the existing sweep's knobs (torso mass, deadzone/play, backlash, lag, dead time, friction, keyframe error,
    CoM offset) applied on top of the calibrated base, plus combined worst cases; both stances."""
    b = CAL
    d = lambda k, x: b.get(k, S.CASE_DEFAULT[k]) + x
    one = [{}, dict(torso=0.6), dict(torso=1.4), dict(roll_lash=d("roll_lash", 1.5)), dict(lash=d("lash", 1.0)),
           dict(lag=2.0), dict(delay=d("delay", 3)), dict(delay=max(1, d("delay", -1))), dict(mu=0.4), dict(mu=1.0), dict(kf="design"),
           dict(bias=0.003), dict(bias=-0.015), dict(off=0.012), dict(off=-0.012), dict(gbias=0.010), dict(gbias=-0.010)]
    combo = [dict(torso=0.6, kf="design"), dict(torso=0.6, roll_lash=d("roll_lash", 1.5), lash=d("lash", 1.0)),
             dict(torso=1.4, roll_lash=d("roll_lash", 1.5), lash=d("lash", 1.0), lag=2.0, delay=d("delay", 3)),
             dict(roll_lash=d("roll_lash", 1.5), lash=d("lash", 1.0), lag=2.0, delay=d("delay", 3), mu=0.4),
             dict(torso=0.6, off=0.015, lag=2.0, delay=d("delay", 3)),
             dict(torso=1.4, off=-0.015, roll_lash=d("roll_lash", 1.5), lag=2.0, delay=d("delay", 3), mu=0.4)]
    return [{**c, "stance": st} for st in ("R", "L") for c in one + combo]


def run_set(cases, ctl, sw, procs=12, seeds=(0,)):
    jobs = [(cal(**{**c, "seed": s}), ctl, sw) for c in cases for s in seeds]
    return scan.pmap(jobs, procs)


def ok(r, base=None):
    return bool((not r["fell"]) and not r["tripped"] and r["err_peak"] < 6.0 and r["clr_min"] >= 10.0)


def summ(res):
    rr = [r for r in res if "error" not in r]
    rms = np.array([r["rms"] for r in rr])
    return dict(n=len(res), err=len(res) - len(rr), fell=sum(r["fell"] for r in rr), trip=sum(r["tripped"] for r in rr),
                ok=sum(ok(r) for r in rr),
                rms_med=[round(float(x), 2) for x in np.nanmedian(rms, axis=0)], rms_max=[round(float(x), 2) for x in np.nanmax(rms, axis=0)],
                err_med=round(float(np.median([r["err_peak"] for r in rr])), 2), err_max=round(float(max(r["err_peak"] for r in rr)), 2),
                clr_min=round(min(r["clr_min"] for r in rr), 1), gap_min=round(min(r["gap_min"] for r in rr), 2),
                touch=sum(r["gap_min"] < 0 for r in rr), contact_frac_med=round(float(np.median([r["contact_frac"] for r in rr])), 2), usw_max=round(max(r["usw_max"] for r in rr), 1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("mode")
    ap.add_argument("--sw", default="{}"); ap.add_argument("--ctl", default="{}"); ap.add_argument("--json", default=None)
    a = ap.parse_args()
    if a.mode == "robust":
        res = run_set(robust_cases(), json.loads(a.ctl), json.loads(a.sw) or None)
        print(summ(res))
