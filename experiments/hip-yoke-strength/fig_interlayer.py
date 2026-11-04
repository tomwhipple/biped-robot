"""Worst interlayer stress per member, failed glued yoke vs the redesign, over
all five load cases (servo stall torques, 100 N up, 50 N forward). Interlayer
stress at a node = max(across-layer tension, interlayer shear) in the piece's
own print frame; bolt-hole walls excluded. Writes fig_interlayer.png + .csv to the output directory
(docs/design-v6/figs/hip_yoke_interlayer_stress.png is a copy).
    python fig_interlayer.py baseline v6"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
HERE = OUT_DIR
FEM = os.path.join(OUT_DIR, "fem")
CASES = ("roll", "pitch", "yaw", "fz", "fx")
CASE_NAME = {"roll": "roll torque", "pitch": "pitch torque", "yaw": "yaw torque",
             "fz": "100 N up", "fx": "50 N forward"}

# members: (label, piece, mask on P)
MEMBERS = [
    ("Roll horn arm + flange front", "roll", lambda P: P[:, 0] > 10),
    ("Roll flange over the lap", "roll", lambda P: (P[:, 0] >= -14) & (P[:, 0] <= 10)),
    ("Roll idler arm", "roll", lambda P: P[:, 0] < -14),
    ("Pitch horn arm", "pitch", lambda P: P[:, 1] > 17),
    ("Pitch idler arm", "pitch", lambda P: P[:, 1] < -15),
    ("Pitch flange + toe", "pitch", lambda P: (P[:, 1] >= -15) & (P[:, 1] <= 17)),
    ("Glue lap", "glue", lambda P: np.ones(len(P), bool)),
]


def interlayer(tag, case):
    a = np.load(os.path.join(FEM, tag if case == "roll" else f"{tag}_{case}", "stress.npy"))
    P, S, far = a[:, :3], a[:, 3:9], a[:, 9] > 0.5
    sxx, syy, szz, sxy, syz, szx = S.T
    roll = far & (P[:, 2] > -19.95)
    pitch = far & (P[:, 2] < -20.05)
    glue = far & (np.abs(P[:, 2] + 20) < 0.05)
    val = np.zeros(len(P))
    val[roll] = np.maximum(syy, np.hypot(sxy, syz))[roll]
    val[pitch] = np.maximum(sxx, np.hypot(sxy, szx))[pitch]
    val[glue] = np.maximum(szz, np.hypot(szx, syz))[glue]
    return P, val, {"roll": roll, "pitch": pitch, "glue": glue}


def worst(tag):
    out = {}
    for label, piece, m in MEMBERS:
        best = (0.0, None)
        for case in CASES:
            P, v, masks = interlayer(tag, case)
            sel = masks[piece] & m(P)
            if sel.any() and v[sel].max() > best[0]:
                best = (float(v[sel].max()), case)
        out[label] = best
    return out


if __name__ == "__main__":
    before, after = sys.argv[1], sys.argv[2]
    wb, wa = worst(before), worst(after)
    with open(os.path.join(HERE, "fig_interlayer.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["member", "failed MPa", "failed case", "redesign MPa", "redesign case"])
        for label, *_ in MEMBERS:
            w.writerow([label, f"{wb[label][0]:.1f}", wb[label][1], f"{wa[label][0]:.1f}", wa[label][1]])
    SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    C1, C2 = "#2a78d6", "#eb6834"
    labels = [m[0] for m in MEMBERS][::-1]
    y = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(9, 4.8), dpi=150)
    fig.patch.set_facecolor(SURF)
    ax.set_facecolor(SURF)
    h = 0.34
    for off, data, col, name in ((h / 2 + 0.02, wb, C1, "Failed glued yoke"),
                                 (-h / 2 - 0.02, wa, C2, "Redesign")):
        vals = [data[l][0] for l in labels]
        ax.barh(y + off, vals, height=h, color=col, label=name, zorder=2)
        for yi, (l, v) in zip(y + off, zip(labels, vals)):
            ax.text(v + 1.5, yi, f"{v:.0f}  ({CASE_NAME[data[l][1]]})", va="center", ha="left",
                    fontsize=8, color=INK2)
    ax.set_yticks(y, labels, fontsize=9, color=INK)
    ax.set_xlabel("Worst interlayer stress, MPa (each piece in its own print frame)", fontsize=9, color=INK2)
    ax.set_xlim(0, 150)
    ax.grid(axis="x", color=GRID, lw=0.8, zorder=0)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(axis="x", colors=INK2, labelsize=8)
    ax.tick_params(axis="y", length=0)
    ax.set_title("Hip yoke: worst stress across the print layers, at servo stall and 4x body weight",
                 fontsize=10, color=INK, loc="left")
    ax.legend(frameon=False, fontsize=8, loc="lower right", labelcolor=INK2)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_interlayer.png"), facecolor=SURF)
    for label, *_ in MEMBERS:
        print(f"{label:30s} {wb[label][0]:6.1f} ({wb[label][1]:5s}) -> {wa[label][0]:6.1f} ({wa[label][1]})")
