"""The assembly figures for assembly.md: a top render of the board after each step (3D models only for the parts
placed so far), with that step's parts outlined and their pin 1 or polarity marked.
usage: uv run --with matplotlib --with pillow python tools/asm_figs.py <work dir>   (from pcb/power-lite)"""
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
FIGS = PROJ.parent.parent / "docs" / "design-v6" / "figs"
KPY = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3"
KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
W, H = 97.0, 64.5
HI = "#ff2e93"
# where the alternation would collide; J4-J8, F1 and F2 carry their own silkscreen labels
SIDE = {"W2": "below", "W3": "right", "W4": "right", "W5": "right", "D2": "left", "F1": "none", "F2": "none",
        **{f"J{i}": "none" for i in range(4, 9)}}

R = [f"R{i}" for i in range(1, 9)]
# (file tag, parts, marks: (ref, pad, label))
STEPS = [
    ("01_resistors", R, []),
    ("02_d1", ["D1"], [("D1", "1", "band")]),
    ("03_c2", ["C2"], []),
    ("04_transistors", ["Q1", "Q2"], [("Q1", "1", "E"), ("Q2", "1", "E")]),
    ("05_leds", ["LED1", "LED2"], [("LED1", "1", "K"), ("LED2", "1", "K")]),
    ("06_tvs", ["D2", "D3"], [("D2", "1", "band"), ("D3", "1", "band")]),
    ("07_jst", ["J2", "J3", "J11", "J12"], [(j, "1", "1") for j in ("J2", "J3", "J11", "J12")]),
    ("08_servo_headers", ["J4", "J5", "J6", "J7", "J8"], [(j, "1", "1") for j in ("J4", "J5", "J6", "J7", "J8")]),
    ("09_fuse_holders", ["F1", "F2"], []),
    ("10_c1", ["C1"], [("C1", "1", "+")]),
    ("11_xt60", ["J1"], [("J1", "1", "−"), ("J1", "2", "+")]),
    ("12_relay", ["K1"], []),
    ("13_wires", ["W1", "W2", "W3", "W4", "W5", "W6", "W7"], []),
]


def render(i, work):
    tag, _, _ = STEPS[i]
    keep = [r for _, refs, _ in STEPS[:i + 1] for r in refs]
    brd, js, png = work / f"{tag}.kicad_pcb", work / f"{tag}.json", work / f"{tag}_raw.png"
    subprocess.run([KPY, HERE / "asm_strip.py", PROJ / "power-lite.kicad_pcb", brd, js, *keep],
                   check=True, capture_output=True)
    subprocess.run([KCLI, "pcb", "render", "--side", "top", "--width", "1600", "--height", "1100",
                    "--quality", "high", "-o", png, brd], check=True, capture_output=True)
    return json.loads(js.read_text()), png


def board_box(img):
    a = np.asarray(img.convert("RGB")).astype(int)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    ys, xs = np.nonzero((g > 50) & (g > r + 15) & (g > b + 5))
    return xs.min(), ys.min(), xs.max(), ys.max()


def annotate(i, geom, raw):
    tag, refs, marks = STEPS[i]
    img = Image.open(raw)
    x0, y0, x1, y1 = board_box(img)
    sx, sy = (x1 - x0) / W, (y1 - y0) / H

    def px(x, y):
        return x0 + x * sx, y0 + y * sy

    fig = plt.figure(figsize=(img.width / 200, img.height / 200), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.set_axis_off()
    halo = [pe.withStroke(linewidth=3, foreground="white")]
    last = {True: -1e9, False: -1e9}   # x of the last label above / below
    for ref in sorted(refs, key=lambda r: geom[r]["crtyd"][0]):
        g = geom[ref]
        (ax0, ay0), (ax1, ay1) = px(*g["crtyd"][:2]), px(*g["crtyd"][2:])
        ax.add_patch(FancyBboxPatch((ax0, ay0), ax1 - ax0, ay1 - ay0, boxstyle="round,pad=2,rounding_size=6",
                                    fill=False, ec=HI, lw=2.2))
        label = ref if ref.startswith(("W", "J", "K1", "F")) else f"{ref}\n{g['value']}"
        cx = (ax0 + ax1) / 2
        style = dict(fontsize=6.5, color="#1a1a1a", weight="bold", linespacing=1.0, path_effects=halo)
        if SIDE.get(ref) == "none":
            continue
        if SIDE.get(ref) in ("right", "left"):
            right = SIDE[ref] == "right"
            ax.text(ax1 + 6 if right else ax0 - 6, (ay0 + ay1) / 2, label, ha="left" if right else "right",
                    va="center", **style)
            continue
        top = ay0 > 80 and cx - last[True] > 9 * sx   # alternate sides when neighbours are close
        if not top and cx - last[False] < 9 * sx and ay0 > 80:
            top = True
        top = top and SIDE.get(ref) != "below"
        last[top] = cx
        ax.text(cx, ay0 - 6 if top else ay1 + 6, label, ha="center", va="bottom" if top else "top", **style)
    for ref, pad, lab in marks:
        x, y = px(*geom[ref]["pads"][pad])
        ax.plot([x], [y], "o", ms=11, mfc="none", mec=HI, mew=2.2)
        ax.text(x + 9, y - 9, lab, fontsize=6.5, color="white", weight="bold", ha="left", va="bottom",
                bbox=dict(boxstyle="round,pad=0.2", fc=HI, ec="none"))
    out = FIGS / f"powerlite_asm_{tag}.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


work = Path(sys.argv[1])
work.mkdir(parents=True, exist_ok=True)
shapes = work / "power-lite.3dshapes"   # the boards' ${KIPRJMOD} is <work dir>
if not shapes.exists():
    shapes.symlink_to(PROJ / "power-lite.3dshapes")
if "--annotate-only" in sys.argv:   # reuse the renders already in <work dir>
    results = [(json.loads((work / f"{t}.json").read_text()), work / f"{t}_raw.png") for t, _, _ in STEPS]
else:
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda i: render(i, work), range(len(STEPS))))
for i, (geom, raw) in enumerate(results):
    print("written", annotate(i, geom, raw))
