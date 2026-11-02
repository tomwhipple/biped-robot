"""Figures for 2026-11-01-floors-fit.md: predicted vs actual on both floors, the u -> roll frequency
response at the hard-floor limit cycle, and the inside-tip test.

    CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/one-leg-stand/simfit_floors_figs.py \
        --plant simfit_plant_floors.json --tag fitted [--ctl kp=0.3 ...]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simfit_compare as C  # noqa: E402
import simfit_frf as F  # noqa: E402
import simfit_sim as S  # noqa: E402
import simfit_tip as T  # noqa: E402

R2D = 180 / math.pi
FIG_STANDS = ["194709R", "194709L", "222031L", "203759R", "203759L", "203534R", "203257R", "203759R3"]


def load_plant(path):
    d = json.load(open(path))
    return S.Plant(**d["plant"])


def fig_tip(pl, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = T.evaluate(pl)
    fig, axs = plt.subplots(1, 2, figsize=(12, 3.6))
    for ax, o in zip(axs, res):
        sg = 1.0 if o["side"] == "L" else -1.0
        n = np.arange(1, len(o["hw_rolls"]) + 1) * T.STEP_TICKS
        ax.plot(n, sg * (o["hw_rolls"] - o["hw_r0"]), "ko-", ms=3, label="robot (run 1)")
        ax.plot(np.arange(1, len(o["sim_rolls"]) + 1) * T.STEP_TICKS, sg * (o["sim_rolls"] - o["sim_r0"]), "C1s-", ms=3,
                label="sim")
        ax.axvline(T.HW_TIP[o["side"]]["offset"], color="k", ls="--", lw=0.8)
        if o["sim_step"]:
            ax.axvline(o["sim_step"] * T.STEP_TICKS, color="C1", ls="--", lw=0.8)
        ax.set_title(f"{o['side']} stance: tip robot {T.HW_TIP[o['side']]['offset']:.0f} ticks, "
                     f"sim {o['sim_step'] * T.STEP_TICKS if o['sim_step'] else '-'}", fontsize=9)
        ax.set_xlabel("lean toward the raised foot (ticks on hip + ankle roll beyond the frozen correction)", fontsize=8)
        ax.set_ylabel("roll change toward the raised foot (deg)", fontsize=8)
        ax.set_ylim(-1, 12)
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    fig.suptitle(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)
    return res


def fig_frf(plants, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4))
    freqs = (1.0, 1.4, 1.85, 2.2, 2.8, 3.5)
    for (lab, pl), c in zip(plants, ("C0", "C1", "C2")):
        rows = F.frf_sim(pl, "hard", "R", freqs=freqs)
        axs[0].plot([r[0] for r in rows], [r[1] for r in rows], c + "o-", label=f"sim, {lab}")
        axs[1].plot([r[0] for r in rows], [r[2] for r in rows], c + "o-")
    for sid, a, b in F.HW_CYCLES:
        f, m, ph = F.frf_hw(sid, a, b)
        axs[0].plot(f, m, "k*", ms=10)
        axs[1].plot(f, ph, "k*", ms=10)
    ff = np.linspace(0.8, 3.6, 50)
    axs[0].plot(ff, 1 / np.abs(0.3 + 0.03j * 2 * np.pi * ff), "k:", lw=1, label="1/|C| (kp 0.3, kd 0.03)")
    axs[1].plot(ff, -np.degrees(np.angle(0.3 + 0.03j * 2 * np.pi * ff)), "k:", lw=1)
    axs[0].set_ylabel("|roll / u|"); axs[1].set_ylabel("phase (deg)")
    for a in axs:
        a.set_xlabel("Hz"); a.grid(alpha=0.3)
    axs[0].legend(fontsize=7)
    axs[0].set_title("hard floor, R stance, 1 deg dither on u in the 30 mm hold; * = robot limit cycles", fontsize=8)
    axs[1].set_title("oscillation where the sim curve meets the dotted line in both panels", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)


def fig_sweep(plants, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import simfit_sweep as W
    hw = W.hw_response()
    fig, axs = plt.subplots(1, 3, figsize=(15, 3.4))
    f = [r[0] for r in hw]
    axs[0].semilogy(f, [r[1] for r in hw], "k*-", ms=8, label="robot")
    axs[1].plot(f, [r[2] for r in hw], "k*-", ms=8)
    axs[2].plot(f, [r[3]["R_ankle_roll"][1] for r in hw], "k*-", ms=8, label="robot R ankle roll")
    axs[2].plot(f, [r[3]["R_hip_roll"][1] for r in hw], "k*--", ms=8, label="robot R hip roll")
    for (lab, pl), c in zip(plants, ("C0", "C1")):
        sm = W.sim_response(pl)
        axs[0].semilogy(f, [r[1] for r in sm], c + "o-", label=f"sim, {lab}")
        axs[1].plot(f, [r[2] for r in sm], c + "o-")
        axs[2].plot(f, [r[3]["R_ankle_roll"][1] for r in sm], c + "o-")
        axs[2].plot(f, [r[3]["R_hip_roll"][1] for r in sm], c + "o--")
    axs[0].set_ylabel("|body roll / commanded| (deg/deg)"); axs[1].set_ylabel("phase (deg)")
    axs[2].set_ylabel("servo read-back / goal phase (deg)")
    for a in axs:
        a.set_xlabel("Hz"); a.grid(alpha=0.3)
    axs[0].legend(fontsize=7); axs[2].legend(fontsize=7)
    fig.suptitle("double-support lateral-roll sine, +-11 ticks on all four roll joints, hard floor", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=70)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plant", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--ctl", nargs="*", default=[])
    ap.add_argument("--frf-vs", default=None, help="a second plant json for the FRF figure")
    a = ap.parse_args()
    pl = load_plant(a.plant if os.path.isabs(a.plant) else os.path.join(HERE, a.plant))
    ctl = C.parse_kv(a.ctl, S.Ctl) if a.ctl else None
    out = os.path.join(HERE, f"2026-11-01-floors-{a.tag}")
    res = C.evaluate(pl, ctl, FIG_STANDS, keep=True)
    C.report(res)
    C.plot(res, out + "-stands.png", f"{a.tag}: {pl.label()}"[:300])
    tip = fig_tip(pl, out + "-tip.png", a.tag)
    T.report(tip)
    if a.frf_vs:
        p2 = load_plant(a.frf_vs if os.path.isabs(a.frf_vs) else os.path.join(HERE, a.frf_vs))
        fig_frf([("10-30 plant", p2), (a.tag, pl)], out + "-frf.png")
        fig_sweep([("10-30 plant (real mass)", replace(p2, torso_scale=1.082)), (a.tag, pl)], out + "-sweep.png")
    print("wrote", out + "-*.png")
