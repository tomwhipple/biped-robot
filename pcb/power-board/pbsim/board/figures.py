"""Figures for the whole-board simulation record, drawn from the campaign's
saved traces and sweep results (run ``make pbfull`` first):

    python -m pbsim.board.figures        (from pcb/power-board)

writes ``docs/design-v6/figs/pbfull_*.png``. Static PNGs for a markdown
record: one quantity per axis, fixed series colours (the reference palette's
first slots, in order), a legend for every multi-series panel.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .campaign import OUT  # noqa: E402

FIGS = Path(__file__).resolve().parents[4] / "docs" / "design-v6" / "figs"
S = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]      # categorical slots 1–4
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e6e5e1", "#fcfcfb"
BAND = "#d9d8d3"

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
    "ytick.color": INK2, "text.color": INK, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10,
    "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False, "lines.linewidth": 1.4, "figure.dpi": 150})


def trace(name):
    return np.load(OUT / "traces" / f"{name}.npz")


def results(suite):
    return {r["name"]: r for r in json.loads((OUT / f"{suite}.json").read_text())}


def _save(fig, name):
    FIGS.mkdir(parents=True, exist_ok=True)
    p = FIGS / f"pbfull_{name}.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


def power_on():
    d = trace("power_on_full_sequence")
    t = d["t"] * 1e3
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 6.2), sharex=True)
    ax[0].plot(t, d["pack"], color=S[0], label="PACK+ / J3")
    ax[0].plot(t, d["servo"], color=S[1], label="SERVO+ (J4–J7)")
    ax[0].plot(t, d["v3v3"], color=S[2], label="+3V3_HOST (ESP32)")
    ax[0].set_ylabel("V")
    ax[0].set_title("Power-on: POWER tapped at 50 ms, SERVO_EN high once the ESP32 is up")
    ax[0].legend(loc="center right")
    ax[1].plot(t, d["q2_vgs"], color=S[0], label="Q2 (main switch) VGS")
    ax[1].plot(t, d["bst"], color=S[1], label="U2 BST − SRC")
    ax[1].plot(t, d["q3_vgs"], color=S[2], label="Q3 (servo switch) VGS")
    ax[1].axhline(7.6, color=INK2, lw=0.8, ls=":")
    ax[1].text(820, 6.4, "BST UVLO 7.6 V:\ngate driver enabled", color=INK2, fontsize=8)
    ax[1].set_ylabel("V")
    ax[1].legend(loc="upper left")
    ax[2].plot(t, d["i_q2"], color=S[0], label="Q2 current")
    ax[2].plot(t, d["i_q3"], color=S[1], label="Q3 current")
    ax[2].set_ylabel("A")
    ax[2].set_xlabel("time (ms)")
    ax[2].legend(loc="upper left")
    ax[2].annotate("45 A for ~10 µs: General Driver input\ncapacitance when Q2 closes",
                   xy=(329, 44), xytext=(420, 30), color=INK2, fontsize=8,
                   arrowprops=dict(arrowstyle="-", color=INK2, lw=0.6))
    _save(fig, "power_on")


def kill_hang():
    a = trace("shutdown_no_servos_bench")
    b = trace("shutdown_no_servos_fix_both")
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    for d, c, lab in ((a, S[0], "as drawn (R38/R39 47k/10k)"),
                      (b, S[1], "R38/R39 4.7k/1k + 4.7k PACK+ bleed")):
        ax[0].plot(d["t"], d["servo"], color=c, label=f"SERVO+, {lab}")
        ax[1].plot(d["t"], d["ld"], color=c, label=f"U1 LD, {lab}")
    ax[0].set_ylabel("V")
    ax[0].set_title("KILL at 1.5 s, FETs open at 2.5 s, no servos plugged in")
    ax[0].legend(loc="upper right")
    ax[1].axhspan(0.8, 2.25, color=BAND, alpha=0.6, lw=0)
    ax[1].text(9.9, 1.15, "wake-on-LD band 0.8–2.25 V:\nSHUTDOWN waits above it",
               color=INK2, fontsize=7.5)
    ax[1].set_ylabel("V")
    ax[1].set_xlabel("time (s)")
    ax[1].legend(loc="upper right")
    ax[1].axvline(8.0, color=INK2, lw=0.8, ls=":")
    ax[1].text(7.95, 6, "POWER pressed", color=INK2, fontsize=8, ha="right")
    _save(fig, "kill_hang")


def hot_wake():
    r = results("sweep")
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    for (fix, corner), c, lab in (((False, "typ"), S[0], "as drawn, typical leakage"),
                                  ((False, "max"), S[1], "as drawn, maximum leakage"),
                                  ((True, "typ"), S[2], "silicon pair + R10 100k, typical"),
                                  ((True, "max"), S[3], "silicon pair + R10 100k, maximum")):
        pts = sorted((x["metrics"]["temp_C"], x["metrics"]["ts2_V"]) for x in r.values()
                     if x["name"].startswith("hot_off_") and x["metrics"].get("fix") == fix
                     and x["metrics"].get("corner") == corner)
        if pts:
            ax.plot(*zip(*pts), color=c, marker="o", ms=4, label=lab)
    ax.axhspan(0.7, 1.1, color=BAND, alpha=0.7, lw=0)
    ax.text(50, 0.82, "wake threshold 0.7–1.1 V", color=INK2, fontsize=8)
    ax.set_xlabel("board temperature (°C), board off, General Driver unpowered")
    ax.set_ylabel("TS2 (V)")
    ax.set_title("TS2 with the board off: D2's leakage pulls it down when hot")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=2)
    _save(fig, "hot_wake")


def short_j4():
    d = trace("short_servo_lead_J4")
    e = trace("fix_packbulk_short_J4") if (OUT / "traces" / "fix_packbulk_short_J4.npz").exists() else None
    m = (d["t"] > 1.0 - 3e-6) & (d["t"] < 1.0 + 40e-6)
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    t = (d["t"][m] - 1.0) * 1e6
    ax[0].plot(t, d["i_q3"][m], color=S[0], label="Q3 current, as drawn")
    ax[1].plot(t, d["pack"][m], color=S[0], label="PACK+, as drawn")
    if e is not None:
        me = (e["t"] > 1.0 - 3e-6) & (e["t"] < 1.0 + 40e-6)
        te = (e["t"][me] - 1.0) * 1e6
        ax[0].plot(te, e["i_q3"][me], color=S[1], label="Q3 current, + 2 × 10 µF on PACK+")
        ax[1].plot(te, e["pack"][me], color=S[1], label="PACK+, + 2 × 10 µF")
    ax[0].axhline(39.8, color=INK2, lw=0.8, ls=":")
    ax[0].text(0.5, 41, "SC threshold 39.8 A", color=INK2, fontsize=8)
    ax[0].set_ylabel("A")
    ax[0].set_title("Short at the end of the right-leg lead (J4), rail on")
    ax[0].legend(loc="center right")
    ax[1].set_ylabel("V")
    ax[1].set_xlabel("time after the short (µs)")
    ax[1].legend(loc="upper right")
    _save(fig, "short_j4")


def q3_vgs():
    a = trace("rail_load_17A_ocd_low_corner")
    b = trace("fix_q3zener_ocd_cut_17A")
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    for d, c, lab in ((a, S[0], "as drawn"), (b, S[1], "+ 15 V zener gate–source on Q3")):
        m = (d["t"] > 1.045) & (d["t"] < 1.06)
        ax.plot((d["t"][m] - 1.0) * 1e3, d["q3_vgs"][m], color=c, label=lab)
    ax.axhline(16, color=INK2, lw=0.8, ls=":")
    ax.text(45.2, 16.3, "U2 PU/PD − SRC abs max 16 V", color=INK2, fontsize=8)
    ax.set_xlabel("time after a 17 A rail load starts (ms); U1 opens Q2 at 53 ms")
    ax.set_ylabel("Q3 VGS (V)")
    ax.set_title("Q3's gate rides up when PACK+ collapses with the rail on")
    ax.legend(loc="center left")
    _save(fig, "q3_vgs")


def main():
    for f in (power_on, kill_hang, hot_wake, short_j4, q3_vgs):
        try:
            f()
        except FileNotFoundError as e:
            print(f"skip {f.__name__}: {e}")


if __name__ == "__main__":
    main()
