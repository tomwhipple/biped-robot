"""The harness figure for harness.md (J12): Waveshare's top photo of the General Driver, with the headers J12's
four wires plug into outlined and the four pins marked.
usage: uv run --with matplotlib --with pillow python tools/gd_fig.py   (from pcb/power-lite)"""
from pathlib import Path

import matplotlib
from PIL import Image

matplotlib.use("Agg")
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, ConnectionPatch, FancyBboxPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
PHOTO = ROOT / "docs" / "datasheets" / "general-driver" / "General_Driver_for_Robots-top.jpg"
OUT = ROOT / "docs" / "design-v6" / "figs" / "powerlite_j12_general_driver.png"
HI = "#ff2e93"
HALO = [pe.withStroke(linewidth=4, foreground="white")]

# the board in the 1600 x 1200 photo; 65.01 mm across is 807 px, 12.4 px/mm
BOARD = (368, 233, 1175, 1050)
# pixel positions below are in the cropped board
COL_X = 303  # the 1x10 2.54 mm pin header: P3 (RX ... IO5) above H2 (IO4, 5V, GND)
COL_Y = [230.5, 262, 294, 325.5, 357, 389, 420.5, 451.5, 484, 516]
H3_X = 97  # H3's contacts, 2.0 mm pitch, top to bottom MA2 GND AC2 AC1 3V3 MA1
H3_Y = [268.5, 294, 320, 343.5, 368.5, 394]
# (J12 pin, net, ESP32 pin, header and pin, (x, y), y of the gap between label rows the leader runs in,
# label y); the leaders run between the silkscreen labels, not over them
TARGETS = [
    ("J12.2", "SERVO_EN", "IO16", "P3 pin 4", (COL_X, COL_Y[3]), 310, 280),
    ("J12.3", "GND", "GND", "P3 pin 3", (COL_X, COL_Y[4]), 373, 385),
    ("J12.6", "ESTOP_N", "IO4", "H2 pin 1", (COL_X, COL_Y[7]), 468, 480),
    ("J12.5", "RAIL_V", "GPIO34", "H3 pin 4 (AC1)", (H3_X, H3_Y[3]), 356, 356),
]
ZOOM = (0, 195, 374, 550)  # x0, y0, x1, y1


def outline(ax, x0, y0, x1, y1, name, side, lw=2.5, fs=14):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=4",
                                fill=False, ec=HI, lw=lw, zorder=5))
    if side == "top":
        ax.text((x0 + x1) / 2, y0 - 4, name, ha="center", va="bottom", fontsize=fs, weight="bold",
                path_effects=HALO, zorder=6)
    else:
        ax.text(x1 + 4, (y0 + y1) / 2, name, ha="left", va="center", fontsize=fs, weight="bold",
                path_effects=HALO, zorder=6)


def tag(ax, x, y, text, ha):
    ax.text(x, y, text, ha=ha, va="center", fontsize=13, color="white", weight="bold", zorder=8,
            bbox=dict(boxstyle="round,pad=0.35", fc=HI, ec="white", lw=1.5))


def main():
    board = Image.open(PHOTO).convert("RGB").crop(BOARD)
    bw, bh = board.size
    fig = plt.figure(figsize=(16, 7.6), dpi=110)
    ov = fig.add_axes([0.005, 0.04, 0.34, 0.88])
    zm = fig.add_axes([0.35, 0.0, 0.645, 1.0])
    for ax in (ov, zm):
        ax.axis("off")

    ov.imshow(board)
    ov.set_xlim(0, bw)
    ov.set_ylim(bh, 0)
    ov.text(bw / 2, -18, "General Driver, top (Waveshare's photo)", ha="center", fontsize=14, weight="bold")
    outline(ov, COL_X - 14, COL_Y[0] - 16, COL_X + 14, COL_Y[6] + 16, "", "right", lw=2)
    outline(ov, COL_X - 14, COL_Y[7] - 15, COL_X + 14, COL_Y[9] + 16, "", "right", lw=2)
    outline(ov, 55, 228, 132, 432, "", "top", lw=2)
    ov.text(480, 30, "ESP32", ha="center", va="center", fontsize=12, weight="bold", color=HI, path_effects=HALO)
    x0, y0, x1, y1 = ZOOM
    ov.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="#222", lw=1.2, ls="--", zorder=4))

    zm.imshow(board.crop(ZOOM), extent=(x0, x1, y1, y0))
    zm.set_xlim(x0 - 185, x1 + 140)
    zm.set_ylim(y1, y0)
    for a, b in (((x1, y0), (x0, y0)), ((x1, y1), (x0, y1))):
        fig.add_artist(ConnectionPatch(a, b, "data", "data", axesA=ov, axesB=zm, color="#222", lw=1, ls="--"))
    zm.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="#222", lw=1.2, ls="--"))
    outline(zm, COL_X - 13, COL_Y[0] - 15, COL_X + 13, COL_Y[6] + 15, "", "right")
    outline(zm, COL_X - 13, COL_Y[7] - 14, COL_X + 13, COL_Y[9] + 15, "", "right")
    zm.text(COL_X - 17, COL_Y[0], "P3", ha="right", va="center", fontsize=15, weight="bold", path_effects=HALO)
    zm.text(COL_X - 17, COL_Y[8], "H2", ha="right", va="center", fontsize=15, weight="bold", path_effects=HALO)
    outline(zm, 55, 228, 132, 432, "H3", "top", fs=15)

    for j, net, gpio, hdr, (x, y), gy, ly in TARGETS:
        zm.add_patch(Circle((x, y), 9, fill=False, ec=HI, lw=3, zorder=7))
        d = -1 if x == H3_X else 1
        tx = x0 - 8 if d < 0 else x1 + 8
        zm.plot([x + d * 8, x + d * 14, x0 - 2 if d < 0 else x1 + 2, tx], [y + (gy - y) * 0.4, gy, gy, ly],
                color=HI, lw=2, zorder=6, solid_capstyle="round")
        tag(zm, tx, ly, f"{j} {net}\n{gpio} · {hdr}", "right" if d < 0 else "left")

    zm.text((x0 + x1) / 2 + 20, y1 + 14, "J12 pins 1, 4, 7, 8 unconnected", ha="center", va="top", fontsize=12,
            style="italic")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, facecolor="white")
    print(OUT)


if __name__ == "__main__":
    main()
