"""Gain selection for the stance-leg roll PI (balance.py) by sweep.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_sweep.py grid  --json build/one_leg/bal_grid.json
    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/balance_sweep.py table --kp 0.5 --ki 0.3 --ka 1 --kh 0

A case is one simulated one-leg stand (keyframes -> JointTimeline -> run_walk
with the controller hook). Each case varies: torso mass (a scale on the
as-built torso body, 0.324 kg), roll play, gear backlash, actuation lag (the
bench streamer has no 2 Hz shaper), dead time in 20 ms ticks, sole friction,
and the open-loop shift error (keyframes solved on the DESIGN plant, run on
the as-built one). Success: never falls, foot off >= 90 % of the hold, CoM
margin >= 10 mm, controller never trips its 12 deg abort.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import one_leg as OL  # noqa: E402
import static_gait as SG  # noqa: E402
from design_gates import JN  # noqa: E402
from gen_plant_v6 import build_xml, params_from_args  # noqa: E402

_cache = {}


def _plant(sets):
    key = tuple(sets)
    if key not in _cache:
        p, _ = params_from_args(["-o", "/dev/null", "--set", *sets])
        xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"balsweep_{os.getpid()}_{len(_cache)}.xml")
        open(xml, "w").write(build_xml(p))
        _cache[key] = (p, xml)
    return _cache[key]


def _keys(solve_sets, hold, bias=-0.005, stance="L", t_lift=2.0):
    key = ("kf", tuple(solve_sets), hold, bias, stance, t_lift)
    if key not in _cache:
        p, _ = _plant(solve_sets)
        q, info = OL.keyframes(p, 0.02, 0.03, bias, 1.0, stance)
        tl, win = OL.timeline(q, t_lift=t_lift, t_hold=hold, swing="R" if stance == "L" else "L")
        _cache[key] = (tl, win, info)
    return _cache[key]


PJ = {"L_hip_roll": "sts3250", "R_hip_roll": "sts3250"}
BASE = dict(torso=1.0, play=3.0, lash=1.0, lag=0.0, delay=1, mu=0.7, kf="asbuilt", bias=-0.005, stance="L", off=0.0, imu_delay=1, seed=0, hold=10.0, t_lift=2.0, gbias=0.0)


def run_case(args):
    cfg, ctl = args
    c = {**BASE, **cfg}
    tl, win, _ = _keys(OL.AS_BUILT if c["kf"] == "asbuilt" else [], c["hold"], c["bias"], c["stance"], c["t_lift"])
    p, xml = _plant(OL.AS_BUILT)

    def setup(env):
        b = env._torso_bid
        env.model.body_mass[b] *= c["torso"]
        env.model.body_inertia[b] *= c["torso"]
        env.model.body_ipos[b][1] += c["off"]      # lateral CoM offset of the torso (board off-centre)
    hook = None
    if ctl:
        hook = OL.bal_hook(tl, c["stance"], ctl["kp"], ctl["ki"], ctl["ka"], ctl["kh"], ctl.get("clamp", 10.0), kd=ctl.get("kd", 0.0),
                           gyro_bias=c["gbias"], gyro_noise=ctl.get("gnoise", 0.0075),
                           delay_ticks=c["imu_delay"], noise=math.radians(ctl.get("noise", 0.1)), seed=c["seed"])
    orig = SG.q_of
    SG.q_of = lambda p_, key: key if isinstance(key, np.ndarray) else orig(p_, key)
    try:
        r = SG.run_walk(p, xml, tl, win, seed=c["seed"], mu=c["mu"], play_deg=c["play"], backlash_deg=c["lash"],
                        lag_hz=c["lag"], delay_ticks=c["delay"], per_joint=PJ, setup=setup, hook=hook)
    finally:
        SG.q_of = orig
    m = hook.metrics() if hook else {}
    hold_s = c["hold"]
    if not hook:   # open loop: foot-off from the run's own window figure
        m = dict(foot_off_s=r["min_t_air"], roll_rms_deg=float("nan"), tripped=False, u_max_deg=0.0, roll_err_max_deg=float("nan"))
    m["ok"] = bool((not r["fell"]) and r["min_margin"] >= 0.010 and m["foot_off_s"] >= 0.9 * hold_s and not m["tripped"])
    m.update(fell=bool(r["fell"]), margin_mm=1e3 * r["min_margin"], tilt_max=r["tilt_max"], cfg=c, ctl=ctl)
    return m


def cases(level="full"):
    return [{**c, "stance": st} for st in ("L", "R") for c in _cases(level)]


def _cases(level):
    out = [{}]
    for k, v in (("torso", 0.6), ("torso", 1.4), ("play", 5.0), ("lash", 2.0), ("lag", 2.0), ("delay", 0), ("delay", 4),
                 ("mu", 0.4), ("mu", 1.0), ("kf", "design"), ("bias", 0.010), ("bias", -0.020), ("off", 0.012), ("off", -0.012)):
        out.append({k: v})
    if level == "full":
        out += [dict(torso=0.6, kf="design"), dict(torso=1.4, kf="design"),
                dict(torso=0.6, play=5.0, lash=2.0), dict(torso=1.4, play=5.0, lash=2.0, lag=2.0, delay=4),
                dict(play=5.0, lash=2.0, lag=2.0, delay=4, mu=0.4), dict(play=5.0, lash=2.0, lag=2.0, delay=4, mu=1.0),
                dict(torso=0.6, play=5.0, lash=2.0, lag=2.0, delay=4, kf="design"),
                dict(torso=1.4, play=5.0, lash=2.0, lag=2.0, delay=4, kf="design", mu=0.4),
                dict(torso=1.0, play=5.0, lash=2.0, lag=0.0, delay=0, kf="design", imu_delay=0),
                dict(torso=0.6, bias=0.010), dict(torso=1.4, bias=-0.020), dict(torso=0.6, bias=0.010, play=5.0, lash=2.0, lag=2.0, delay=4),
                dict(torso=1.4, bias=-0.020, play=5.0, lash=2.0, lag=2.0, delay=4, mu=0.4),
                dict(off=0.015, torso=0.6), dict(off=-0.015, torso=1.4, play=5.0, lash=2.0, lag=2.0, delay=4),
                dict(off=0.015, kf="design"), dict(off=-0.015, kf="design", play=5.0, lash=2.0),
                dict(kf="design", lag=2.0, delay=4), dict(kf="design", delay=4, play=5.0, lash=2.0)]
    return out


def gyro_cases():
    """The transient spec's sweep: the quick robustness cases (torso 0.6-1.4x, play 3-5 deg, backlash
    1-2 deg, lag 0/2 Hz, dead time 0-4 ticks, mu 0.4-1.0, torso CoM +-15 mm) plus the gyro bias
    residual, each lifted straight to 30 mm over 2 s and over 4 s, both stances."""
    out = []
    for st in ("L", "R"):
        for tl in (2.0, 4.0):
            for c in [c for c in _cases("quick") if c.get("bias") != -0.020] + [dict(gbias=0.010), dict(gbias=-0.010),
                                         dict(off=0.015, torso=0.6, lag=2.0, delay=4),
                                         dict(off=-0.015, torso=0.6, lag=2.0, delay=4),
                                         dict(off=0.015, torso=1.4, play=5.0, lash=2.0, lag=2.0, delay=4, gbias=0.010),
                                         dict(off=-0.015, torso=0.6, play=5.0, lash=2.0, lag=2.0, delay=4, gbias=-0.010, mu=0.4)]:
                out.append({**c, "stance": st, "t_lift": tl})
    return out


def transient_ok(r):
    return bool((not r["fell"]) and not r["tripped"] and r["err_peak_deg"] < 6.0 and r["settle_s"] <= 3.0)


def evaluate(ctl, pool, level="full", seeds=(0,)):
    jobs = [({**c, "seed": s}, ctl) for c in cases(level) for s in seeds]
    return jobs, pool.map(run_case, jobs)


def summarize(res):
    return dict(n=len(res), ok=sum(r["ok"] for r in res), fell=sum(r["fell"] for r in res),
                min_margin=min(r["margin_mm"] for r in res), min_off=min(r["foot_off_s"] for r in res),
                max_rms=max(r["roll_rms_deg"] for r in res), max_rms5=max(r.get("roll_rms_last5_deg", float("nan")) for r in res), max_umax=max(r["u_max_deg"] for r in res))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("grid", "table", "open", "gyro"))
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--kds", type=float, nargs="*", default=[0.0, 0.05, 0.1, 0.2])
    ap.add_argument("--scales", type=float, nargs="*", default=[1.0])
    ap.add_argument("--gnoise", type=float, default=0.0075)
    ap.add_argument("--kp", type=float, default=0.5)
    ap.add_argument("--ki", type=float, default=0.3)
    ap.add_argument("--ka", type=float, default=1.0)
    ap.add_argument("--kh", type=float, default=0.0)
    ap.add_argument("--clamp", type=float, default=10.0)
    ap.add_argument("--scale", type=float, default=1.0, help="multiply kp and ki (stability margin: 2x)")
    ap.add_argument("--kps", type=float, nargs="*", default=[0.25, 0.5])
    ap.add_argument("--kis", type=float, nargs="*", default=[0.25, 0.5])
    ap.add_argument("--khs", type=float, nargs="*", default=[-1.0])
    ap.add_argument("--clamps", type=float, nargs="*", default=[10.0])
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--json", default=None)
    ap.add_argument("--procs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    a = ap.parse_args(argv)
    pool = mp.Pool(a.procs)
    out = {}
    if a.mode == "gyro":
        jobs0 = gyro_cases()
        for kp, ki, kd, cl in itertools.product(a.kps, a.kis, a.kds, a.clamps):
            for sc in a.scales:
                ctl = dict(kp=kp * sc, ki=ki * sc, kd=kd * sc, ka=a.ka, kh=-1.0 if a.kh == 0.0 else a.kh, clamp=cl, gnoise=a.gnoise)
                res = pool.map(run_case, [({**BASE, **c}, ctl) for c in jobs0])
                ok = [transient_ok(r) for r in res]
                pk = [r["err_peak_deg"] for r in res]
                st = [r["settle_s"] for r in res]
                rms = [r["roll_rms_last5_deg"] for r in res if not r["fell"]]
                worst = max(range(len(res)), key=lambda i: pk[i])
                print(f"kp {kp*sc:.3f} ki {ki*sc:.3f} kd {kd*sc:.3f} x{sc}: pass {sum(ok)}/{len(ok)}  fell {sum(r['fell'] for r in res)}  trip {sum(r['tripped'] for r in res)}  "
                      f"peak max {max(pk):.1f} med {np.median(pk):.1f}  settle max {max(st):.1f}  rms5 max {max(rms):.2f} med {np.median(rms):.2f}  worst {({k: v for k, v in jobs0[worst].items()})}", flush=True)
                out[json.dumps(ctl) + f"x{sc}"] = dict(pass_=int(sum(ok)), n=len(ok), peak=pk, settle=st, rms5=rms)
        if a.json:
            json.dump(out, open(a.json, "w"), indent=1, default=str)
        return
    if a.mode == "open":
        jobs, res = evaluate(None, pool, seeds=range(a.seeds))
    elif a.mode == "table":
        ctl = dict(kp=a.kp * a.scale, ki=a.ki * a.scale, kd=a.kd * a.scale, ka=a.ka, kh=a.kh, clamp=a.clamp)
        jobs, res = evaluate(ctl, pool, seeds=range(a.seeds))
    else:
        for kp, ki, ka, kh, cl in itertools.product(a.kps, a.kis, (1.0,), a.khs, a.clamps):
            row = {}
            for sc in (1.0, 2.0):          # the stability margin: the same loop at twice the gain
                ctl = dict(kp=kp * sc, ki=ki * sc, ka=ka, kh=kh, clamp=cl)
                jobs, res = evaluate(ctl, pool, level="quick")
                row[sc] = summarize(res)
            print(dict(kp=kp, ki=ki, ka=ka, kh=kh, clamp=cl),
                  "x1", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in row[1.0].items() if k != "n"},
                  "x2", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in row[2.0].items() if k in ("ok", "fell", "min_margin", "max_rms")}, flush=True)
            out[json.dumps(dict(kp=kp, ki=ki, ka=ka, kh=kh, clamp=cl))] = {"x1": row[1.0], "x2": row[2.0]}
        if a.json:
            json.dump(out, open(a.json, "w"), indent=1)
        return
    for (cfg, ctl), r in zip(jobs, res):
        print(("ok  " if r["ok"] else "FAIL"), {k: v for k, v in cfg.items() if BASE.get(k) != v}, f"margin {r['margin_mm']:.1f} mm  off {r['foot_off_s']:.2f} s  "
              f"rms {r['roll_rms_deg']:.2f}  errmax {r['roll_err_max_deg']:.2f}  umax {r['u_max_deg']:.1f}  tilt {r['tilt_max']:.1f}"
              f"{'  FELL' if r['fell'] else ''}{'  TRIP' if r['tripped'] else ''}")
    print(summarize(res))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        json.dump(dict(res=res, summary=summarize(res)), open(a.json, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
