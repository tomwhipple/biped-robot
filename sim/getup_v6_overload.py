"""Get-up margin to the STS overload cutoff (#83, 2026-09-29): a continuous
search over the as-drawn seat push for the sequence with the MOST margin to
the documented protection (sim/sts_servo_model.py: load > 80 % of the
servo's maximum held 2.0 s -> 20 % torque), with the same robustness as the
design's get-up -- all six conditions (play 3/5, mu 0.3/0.7/1.0, servos
100/80/65 %), self-collision on, Plan B (STS3215, P x4 on rolls + knees),
plant r5_asdrawn_rom120 (lumped, 2.10 kg, hip ROM -120).

The scored margin uses the STRICTEST reading the documents allow: "load" =
PWM duty (memory table reg 60), counted CUMULATIVELY (a timer that never
resets), with the cutoff enforced (latched) on the duty reading. The
torque reading (|tau| / stall: the datasheet's "stalled above 80 %") is
reported alongside.

Parameters (bounded, continuous; the seat push of getup_v6_shoulder.seat_push
with its timing and arm poses opened up; tuck hip -120 / knee -130 fixed by
the hip ROM):
  s0, e0     brace: shoulder / elbow as the hands go down behind the hips
  s1, e1     push end: shoulder / elbow
  a_push     ankle dorsiflexion during the push
  t_sit      sit-up move time; t_tuck tuck move time; t_push push move time;
  h_push     hold after the push; h_crouch crouch hold; t_rise each rise move;
  s_rest     shoulder after the push (the arm's "rest" through the rise)

    .venv/bin/python sim/getup_v6_overload.py search [restarts] [iters] > docs/design-v6/getup_overload_search.txt
    .venv/bin/python sim/getup_v6_overload.py verify '<json params>'
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")

import gate_no3250 as GN  # noqa: E402

H, K = -120.0, -130.0
SET = "3215_p4rk"
NAMES = ("s0", "e0", "s1", "e1", "a_push", "t_sit", "t_tuck", "t_push", "h_push", "h_crouch", "t_rise", "s_rest")
BOUNDS = dict(s0=(45, 120), e0=(-100, 0), s1=(-30, 45), e1=(-60, 10), a_push=(-45, -5),
              t_sit=(1.0, 3.0), t_tuck=(1.0, 4.0), t_push=(1.0, 5.0), h_push=(0.3, 2.0),
              h_crouch=(0.5, 3.0), t_rise=(1.0, 3.0), s_rest=(0, 90))
# the recommended as-drawn sequence (getup_v6_shoulder.seat_push, sh 90->0, el -90->0, ankle -25)
RECOMMENDED = dict(s0=90, e0=-90, s1=0, e1=0, a_push=-25, t_sit=1.5, t_tuck=1.5, t_push=2.0, h_push=1.0,
                   h_crouch=2.0, t_rise=1.5, s_rest=60)


def seat_push_p(x):
    """getup_v6_shoulder.seat_push with its times and arm poses as parameters;
    RECOMMENDED reproduces seat_push(False, 90, 0, 2.0, -25, elbow=(-90, 0))."""
    fold = dict(shoulder=180, elbow=0)
    behind = dict(shoulder=x["s0"], elbow=x["e0"])
    push = dict(shoulder=x["s1"], elbow=x["e1"])
    rest = dict(shoulder=x["s_rest"], elbow=0)
    a = x["a_push"]
    return [("lie", dict(**fold), 0.5, 1.5),
            ("sit up", dict(hip_pitch=-90, **fold), x["t_sit"], 0.8),
            ("fold", dict(hip_pitch=-110, **fold), 1.0, 0.6),
            ("brace", dict(hip_pitch=-110, **behind), 1.0, 0.8),
            ("tuck", dict(hip_pitch=H, knee=K, ankle=0, **behind), x["t_tuck"], 0.8),
            ("push", dict(hip_pitch=H, knee=K, ankle=a, **push), x["t_push"], x["h_push"]),
            ("crouch hold", dict(hip_pitch=H, knee=K, ankle=a, **rest), 1.0, x["h_crouch"]),
            ("rise 2", dict(hip_pitch=-80, knee=-70, ankle=-30, **rest), x["t_rise"], 1.0),
            ("rise 3", dict(hip_pitch=-45, knee=-50, ankle=-25, **rest), x["t_rise"], 1.0),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20, **rest), x["t_rise"], 1.0),
            ("straight", dict(**rest), 1.0, 0.8)]


def seq_duration(seq):
    return sum(m + h for _, _, m, h in seq)


def _job(args):
    x, cond, ts, load = args
    ok, up, z, rep = GN._overload_job((seat_push_p(x), ts, load, True, cond, SET, None))
    worst = lambda key: max(v[key] for v in rep.values())
    hot = lambda key: max(rep.items(), key=lambda kv: kv[1][key])[0]
    return dict(ok=ok, up=up, z=z, cum_duty=worst("cum_duty"), cont_duty=worst("cont_max_duty"),
                peak_torque=worst("peak_torque"), cum_torque=worst("cum_torque"),
                cont_torque=worst("cont_max_torque"), peak_duty=worst("peak_duty"),
                hot_duty=hot("cum_duty"), hot_torque=hot("peak_torque"),
                trips=sorted(j for j, v in rep.items() if v["trip_t"] is not None),
                sh_peak_torque=max(rep["L_shoulder"]["peak_torque"], rep["R_shoulder"]["peak_torque"]))


def evaluate_many(xs, ts=1.0, load="duty"):
    jobs = [(x, c, ts, load) for x in xs for c in GN.GETUP_CONDS]
    res = GN.pmap(_job, jobs)
    out = []
    for i, x in enumerate(xs):
        rs = res[6 * i: 6 * i + 6]
        n_ok = sum(r["ok"] for r in rs)
        agg = dict(n_ok=n_ok, cum_duty=max(r["cum_duty"] for r in rs), cont_duty=max(r["cont_duty"] for r in rs),
                   peak_torque=max(r["peak_torque"] for r in rs), cont_torque=max(r["cont_torque"] for r in rs),
                   sh_peak_nominal=rs[0]["sh_peak_torque"], T=seq_duration(seat_push_p(x)), per=rs)
        agg["score"] = (10.0 * n_ok - agg["cum_duty"] - 0.5 * agg["cont_duty"]
                        - 3.0 * max(0.0, agg["peak_torque"] - 0.6) - 0.02 * agg["T"])
        out.append(agg)
    return out


def _clip(x):
    return {k: float(np.clip(x[k], *BOUNDS[k])) for k in NAMES}


def _fmt(x):
    return " ".join(f"{k} {x[k]:.1f}" for k in NAMES)


def _summ(a):
    return (f"{a['n_ok']}/6  score {a['score']:+.2f}  duty>80%: total {a['cum_duty']:.2f} s, longest {a['cont_duty']:.2f} s  "
            f"peak torque load {100 * a['peak_torque']:.0f} % (>80 % longest {a['cont_torque']:.2f} s)  "
            f"shoulder at nominal {100 * a['sh_peak_nominal']:.0f} % of stall  T {a['T']:.1f} s")


def search(restarts=4, iters=25, pop=24, n_elite=6, seed0=0):
    """CEM per restart; restart 0 starts at the recommended sequence, the
    others at a uniform draw over the bounds."""
    lo = np.array([BOUNDS[k][0] for k in NAMES], float)
    hi = np.array([BOUNDS[k][1] for k in NAMES], float)
    base = evaluate_many([RECOMMENDED])[0]
    print(f"== baseline (recommended sh 90->0 / el -90->0): {_summ(base)}", flush=True)
    best_all = (base["score"], dict(RECOMMENDED), base)
    for r in range(restarts):
        rng = np.random.default_rng(seed0 + r)
        mu = np.array([RECOMMENDED[k] for k in NAMES], float) if r == 0 else rng.uniform(lo, hi)
        sig = 0.25 * (hi - lo)
        best = None
        for it in range(iters):
            xs = [_clip(dict(zip(NAMES, mu + sig * rng.standard_normal(len(NAMES))))) for _ in range(pop)]
            if best is not None:
                xs[0] = best[1]
            ev = evaluate_many(xs)
            order = np.argsort([-e["score"] for e in ev])
            if best is None or ev[order[0]]["score"] > best[0]:
                best = (ev[order[0]]["score"], xs[order[0]], ev[order[0]])
            el = np.array([[xs[i][k] for k in NAMES] for i in order[:n_elite]])
            mu = el.mean(0)
            sig = np.maximum(0.7 * sig + 0.3 * el.std(0), 0.02 * (hi - lo))
            print(f"   restart {r} iter {it:2d}: best {_summ(best[2])}", flush=True)
        print(f"-- restart {r} BEST: {_summ(best[2])}\n   params {json.dumps({k: round(v, 2) for k, v in best[1].items()})}",
              flush=True)
        if best[0] > best_all[0]:
            best_all = best
    print(f"\n== OVERALL BEST: {_summ(best_all[2])}\n   params {json.dumps({k: round(v, 2) for k, v in best_all[1].items()})}",
          flush=True)
    return best_all


def verify(x, label):
    """the method's verification: the six conditions at the scripted pace and
    3x slower, the cutoff enforced on each reading in turn."""
    print(f"== VERIFY {label}: {_fmt(x)}", flush=True)
    for ts in (1.0, 3.0):
        for load in ("duty", "torque"):
            a = evaluate_many([x], ts=ts, load=load)[0]
            print(f"-- pace x{ts:.0f}, cutoff enforced on the {load} reading: {_summ(a)}", flush=True)
            for c, r in zip(GN.GETUP_CONDS, a["per"]):
                print(f"   {GN._cond_label(c)}: {'STANDING' if r['ok'] else 'no      '} z {r['z']:.3f}  "
                      f"duty>80% total {r['cum_duty']:.2f} s ({r['hot_duty']}) longest {r['cont_duty']:.2f} s;  "
                      f"torque load peak {100 * r['peak_torque']:.0f} % ({r['hot_torque']}) >80% longest {r['cont_torque']:.2f} s;  "
                      f"trips {r['trips'] or 'none'}", flush=True)


def main():
    mode = sys.argv[1]
    if mode == "search":
        restarts = int(sys.argv[2]) if len(sys.argv) > 2 else 4
        iters = int(sys.argv[3]) if len(sys.argv) > 3 else 25
        print(f"== SEARCH for the get-up with the most margin to the STS overload cutoff; r5_asdrawn_rom120, Plan B "
              f"({SET}), self_collide on, six conditions per candidate; CEM, {restarts} restarts x {iters} iterations x 24; "
              f"score = 10 x (conditions standing) - (worst total time above 80 % duty) - 0.5 x (worst continuous) "
              f"- 3 x (worst peak torque load above 60 %) - 0.02 x (sequence seconds); workers {GN.n_workers()}", flush=True)
        best = search(restarts, iters)
        verify(RECOMMENDED, "the recommended sequence")
        verify(best[1], "the search's best")
    elif mode == "verify":
        verify(_clip(json.loads(sys.argv[2])), "given params")


if __name__ == "__main__":
    main()
