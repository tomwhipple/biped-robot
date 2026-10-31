"""Predicted vs actual for the recorded one-leg stands: run every stand through simfit_sim on a
plant, line the sim up with the recording tick by tick, and score / plot the discrepancy.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_compare.py \
        [--set roll_lash=4 sole_w=0.07 ...] [--ctl kp=0.3 ...] [--png out.png] [--stands 222031R,222031L]

The cost (`score`) is per B stage (l10, l30), over the stage window up to the recorded abort
(or the sim's own, whichever comes first):
    rms(err_sim - err_hw) / 1 deg  +  rms(u_sim - u_hw) / 2 deg
plus, when the sim and the robot disagree about an abort, |t_abort_sim - t_abort_hw| / 1 s with a
missing abort counted at the end of the window + 2 s, capped at 3 s, over 2 s. Stand-level: the sum over its two stages.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_hw as H  # noqa: E402
import simfit_sim as S  # noqa: E402

R2D = 180 / math.pi
ALL = [(tag, s) for tag in H.SESSIONS for s in ("R", "L")]
SIG_ERR, SIG_U, SIG_T = 1.0, 2.0, 2.0
PEN_CAP = 3.0                      # s: an abort the other side did not have costs at most 1.5


def parse_kv(items, cls):
    kw = {}
    for it in items or []:
        k, v = it.split("=")
        kw[k] = float(v)
    return cls(**kw)


def stage_cost(st, r, name):
    """(cost, details) for one B stage: hw vs sim on the hw tick times."""
    mv = st.stage(name)
    t0 = mv.t0
    t_next = next((m.t0 for m in st.moves if m.t0 > t0 + 1e-6), t0 + mv.dur + 10)
    hw_ab = st.abort_t if (st.abort_t is not None and t0 < st.abort_t < t_next) else None
    win_end = t_next - 0.05
    sw = (r["t"] >= t0) & (r["t"] < t_next) & (r["state"] > 0)
    s_ab_idx = np.where(sw & (r["state"] == 2))[0]
    sim_ab = float(r["t"][s_ab_idx[0]]) if len(s_ab_idx) else None
    t_cmp = min(x for x in (hw_ab, sim_ab, win_end) if x is not None)
    pen = 0.0
    if (hw_ab is None) != (sim_ab is None) or (hw_ab is not None and sim_ab is not None):
        a = hw_ab if hw_ab is not None else win_end + 2.0
        b = sim_ab if sim_ab is not None else win_end + 2.0
        pen = min(abs(a - b), PEN_CAP) / SIG_T
    if not sw.any():                       # the sim never ran this stage (still landing an earlier abort)
        pen = PEN_CAP / SIG_T
    d = st.ticks
    hw = d[(d.t >= t0 + 0.05) & (d.t <= t_cmp) & (d.bal_state == 1)]
    if len(hw) < 5:
        return pen + 3.0, dict(rms_err=float("nan"), rms_u=float("nan"), abort_pen=pen,
                               hw_abort=(hw_ab - t0) if hw_ab else None, sim_abort=(sim_ab - t0) if sim_ab else None)
    te = hw.t.values
    e_sim = np.interp(te, r["t"], r["err"]) * R2D
    u_sim = np.interp(te, r["t"], r["u"]) * R2D
    e_hw, u_hw = hw.bal_err.values * R2D, hw.bal_u.values * R2D
    rms_e = float(np.sqrt(np.mean((e_sim - e_hw) ** 2)))
    rms_u = float(np.sqrt(np.mean((u_sim - u_hw) ** 2)))
    cost = rms_e / SIG_ERR + rms_u / SIG_U + pen
    sg = 1.0 if st.side == "L" else -1.0
    k_end = np.searchsorted(r["t"], min(t0 + mv.dur, t_cmp))
    k_hold = np.searchsorted(r["t"], t_cmp) - 1
    return cost, dict(rms_err=rms_e, rms_u=rms_u, abort_pen=pen,
                      hw_abort=(hw_ab - t0) if hw_ab else None, sim_abort=(sim_ab - t0) if sim_ab else None,
                      sim_u_end=sg * r["u"][k_hold] * R2D, hw_u_end=sg * u_hw[-1],
                      sim_err_peak=float(np.max(sg * r["err"][sw])) * R2D if sw.any() else float("nan"),
                      hw_err_peak=float(np.max(sg * e_hw)))


def eval_stand(args):
    tag, side, pl, ctl, kw = args
    st = H.load_stand(tag, side)
    r = S.Replay(st, pl, ctl, **kw).run()
    res = {}
    tot = 0.0
    for name in ("l10", "l30"):
        c, det = stage_cost(st, r, name)
        res[name] = det
        tot += c
    # a sim fall where the robot did not fall (outside the B stages too, e.g. while lowering)
    t_ok = st.abort_t if st.abort_t is not None else r["t"][-1]
    base = r["roll"][(r["t"] > -0.6) & (r["t"] < 0.0)].mean()
    ab = np.where(r["state"] == 2)[0]
    if len(ab):                            # after the sim's own abort the fall is the abort's (already scored)
        t_ok = min(t_ok, float(r["t"][ab[0]]))
    pre = r["t"] < t_ok
    fell = bool(np.any(np.abs(r["roll"][pre] - base) > math.radians(15.0)))
    res["fell_early"] = fell
    if fell:
        tot += 5.0
    return dict(tag=tag, side=side, cost=tot, stages=res, r=r)


def evaluate(pl, ctl=None, stands=ALL, workers=None, keep=False, **kw):
    ctl = ctl or S.Ctl()
    jobs = [(t, s, pl, ctl, kw) for t, s in stands]
    if workers == 1:
        out = [eval_stand(j) for j in jobs]
    else:
        with ProcessPoolExecutor(max_workers=workers or len(jobs)) as ex:
            out = list(ex.map(eval_stand, jobs))
    if not keep:
        for o in out:
            o.pop("r")
    return out


def plot(results, path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(results)
    fig, axs = plt.subplots(n, 3, figsize=(17, 2.6 * n), squeeze=False)
    for i, o in enumerate(results):
        st = H.load_stand(o["tag"], o["side"])
        r = o["r"]
        d = st.ticks
        sg = 1.0 if st.side == "L" else -1.0
        base_hw = d.roll[(d.t > -0.6) & (d.t < 0.0)].mean()
        base_sim = r["roll"][(r["t"] > -0.6) & (r["t"] < 0.0)].mean()
        ax = axs[i]
        ax[0].plot(d.t, sg * (d.roll - base_hw) * R2D, "k", lw=1.2, label="robot")
        ax[0].plot(r["t"], sg * (r["roll"] - base_sim) * R2D, "C1", lw=1.2, label="sim")
        ax[0].set_ylim(-14, 8)
        ax[0].set_ylabel(f"{st.session} {st.side}\nroll vs crouch\n(+ toward lifted)", fontsize=8)
        hwb = d.bal_state != 0
        ax[1].plot(d.t[hwb], sg * d.bal_err[hwb] * R2D, "k.", ms=1.5)
        ax[1].plot(r["t"], np.where(r["state"] > 0, sg * r["err"] * R2D, np.nan), "C1", lw=1)
        ax[1].set_ylim(-6, 6)
        ax[1].set_ylabel("err (deg, + toward lifted)", fontsize=8)
        ax[2].plot(d.t, sg * d.bal_u * R2D, "k", lw=1.2)
        ax[2].plot(r["t"], sg * r["u"] * R2D, "C1", lw=1.2)
        ax[2].set_ylim(-3, 11)
        ax[2].set_ylabel("u (deg, + outward)", fontsize=8)
        for a in ax:
            for m in st.moves:
                a.axvline(m.t0, color="0.6", lw=0.6)
            if st.abort_t:
                a.axvline(st.abort_t, color="r", lw=0.8, ls="--")
            a.grid(alpha=0.3)
            a.set_xlim(-1, 32)
        s10, s30 = o["stages"].get("l10", {}), o["stages"].get("l30", {})
        ax[1].set_title(f"cost {o['cost']:.2f}  l30 rms err {s30.get('rms_err', 0):.2f} u {s30.get('rms_u', 0):.2f}  "
                        f"abort hw {s30.get('hw_abort')} sim {None if s30.get('sim_abort') is None else round(s30['sim_abort'], 2)}",
                        fontsize=8)
    axs[0][0].legend(fontsize=8)
    axs[-1][1].set_xlabel("s from the shift command")
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)


def report(results):
    tot = 0.0
    for o in results:
        tot += o["cost"]
        s = "  ".join(f"{k}: e {v.get('rms_err', 0):.2f} u {v.get('rms_u', 0):.2f} pk {v.get('sim_err_peak', 0):+.1f}/{v.get('hw_err_peak', 0):+.1f} "
                      f"uend {v.get('sim_u_end', 0):+.1f}/{v.get('hw_u_end', 0):+.1f} ab {v.get('sim_abort') and round(v['sim_abort'], 1)}/{v.get('hw_abort') and round(v['hw_abort'], 1)}"
                      for k, v in o["stages"].items() if isinstance(v, dict))
        print(f"  {o['tag']} {o['side']}  cost {o['cost']:5.2f}   {s}")
    print(f"  TOTAL {tot:.2f}")
    return tot


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--ctl", nargs="*", default=[])
    ap.add_argument("--png", default=None)
    ap.add_argument("--stands", default=None)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    pl = parse_kv(a.set, S.Plant)
    ctl = parse_kv(a.ctl, S.Ctl)
    stands = ALL if not a.stands else [(x[:-1], x[-1]) for x in a.stands.split(",")]
    res = evaluate(pl, ctl, stands, keep=bool(a.png), seed=a.seed)
    print(pl.label())
    report(res)
    if a.png:
        plot(res, a.png, pl.label())
        print("wrote", a.png)


def joint_table(st, r, stage="l30", t_from=6.0, t_len=6.0):
    """Hold medians, (pos - goal) ticks and |load| (per mille of stall), per joint: robot vs sim,
    over [t_from, t_from + t_len] s after the stage start (cut at an abort)."""
    mv = st.stage(stage)
    t0, t1 = mv.t0 + t_from, mv.t0 + t_from + t_len
    if st.abort_t is not None and st.abort_t > mv.t0:
        t1 = min(t1, st.abort_t - 0.3)
        t0 = min(t0, t1 - 1.0)
    d = st.ticks
    w = d[(d.t >= t0) & (d.t <= t1)]
    ws = (r["t"] >= t0) & (r["t"] <= t1)
    rows = {}
    for k, j in enumerate(H.BUS):
        p, g, ld = w[f"{j}.pos"].values, w[f"{j}.goal"].values.astype(float), w[f"{j}.load"].values
        ok = np.isfinite(p) & (g != 0)
        hw_e = float(np.median(p[ok] - g[ok])) if ok.sum() > 5 else float("nan")
        hw_l = float(np.median(np.abs(ld[ok]))) if ok.sum() > 5 else float("nan")
        se = float(np.median(r["enc"][ws, k] - r["goal"][ws, k])) if ws.any() else float("nan")
        sl = float(np.median(np.abs(r["load"][ws, k]))) if ws.any() else float("nan")
        rows[j] = (hw_e, se, hw_l, sl)
    return rows
