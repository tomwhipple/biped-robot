"""Draw the power architecture figure (docs/design-v6/2026-10-06-power-circuit.md §4).

Every box, panel and wire is placed by hand, so a layout change is a coordinate
change here. Boxes with a part number link to the maker's page in the SVG.

    python docs/design-v6/figs/power_architecture.py   # writes power_architecture.svg beside it
"""

from pathlib import Path
from xml.sax.saxutils import escape

W, H = 1080, 1050
FONT = "Helvetica, Arial, sans-serif"

RED, ORANGE, BLUE, INK, GREEN, DARKRED = "#c0392b", "#e67e22", "#2e86c1", "#212f3d", "#1e8449", "#922b21"

out = []


def attr(**kw):
    return " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items() if v is not None)


def text(x, y, s, size=11, weight="normal", anchor="middle", fill="#1c2833", halo=False, rotate=None):
    extra = ' stroke="white" stroke-width="3.5" paint-order="stroke" stroke-linejoin="round"' if halo else ""
    if rotate is not None:
        extra += f' transform="rotate({rotate} {x} {y})"'
    out.append(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" '
               f'text-anchor="{anchor}" fill="{fill}"{extra}>{escape(s)}</text>')


def panel(x0, y0, x1, y1, title, sub="", fill="#ffffff", stroke="#999999", sw=1.4, title_at="top", align="start"):
    out.append(f'<rect {attr(x=x0, y=y0, width=x1 - x0, height=y1 - y0, rx=12, fill=fill, stroke=stroke, stroke_width=sw)}/>')
    ty = y0 + 22 if title_at == "top" else y1 - 12
    tx = x0 + 14 if align == "start" else x1 - 14
    out.append(f'<text x="{tx}" y="{ty}" font-size="14" fill="#1c2833" text-anchor="{align}">'
               f'<tspan font-weight="bold">{escape(title)}</tspan>'
               + (f'<tspan dx="8" font-size="11" fill="#424949">{escape(sub)}</tspan>' if sub else "") + "</text>")


def box(cx, cy, w, h, title, sub=(), fill="#ffffff", stroke="#566573", sw=1.2, url=None, shape="rect"):
    x0, y0 = cx - w / 2, cy - h / 2
    if url:
        out.append(f'<a href="{escape(url)}" target="_blank">')
    if shape == "octagon":
        c = 12
        pts = [(x0 + c, y0), (x0 + w - c, y0), (x0 + w, y0 + c), (x0 + w, y0 + h - c),
               (x0 + w - c, y0 + h), (x0 + c, y0 + h), (x0, y0 + h - c), (x0, y0 + c)]
        out.append(f'<polygon points="{" ".join(f"{px},{py}" for px, py in pts)}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')
    else:
        out.append(f'<rect {attr(x=x0, y=y0, width=w, height=h, rx=9, fill=fill, stroke=stroke, stroke_width=sw)}/>')
    block = 16 + 13 * len(sub)
    top = cy - block / 2
    text(cx, top + 12, title, size=13, weight="bold")
    for i, s in enumerate(sub):
        text(cx, top + 16 + 13 * i + 10, s, size=10.5, fill="#34495e")
    if url:
        out.append("</a>")


def marker_id(color):
    return "arr" + color.lstrip("#")


def wire(pts, color, width=1.4, dash=None, start=False, end=True, label=None, lx=None, ly=None, anchor="middle",
         rotate=None):
    d = "M " + " L ".join(f"{x},{y}" for x, y in pts)
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"'
               + (f' stroke-dasharray="{dash}"' if dash else "")
               + (f' marker-end="url(#{marker_id(color)})"' if end else "")
               + (f' marker-start="url(#{marker_id(color)})"' if start else "") + "/>")
    if label:
        for i, line in enumerate(label.split("\n")):
            text(lx, ly + 12 * i, line, size=10, anchor=anchor, fill="#283747", halo=True, rotate=rotate)


# ---------------------------------------------------------------- panels (drawn first)
panel(40, 105, 620, 660, "Power board", "custom PCB, assembled by PCBWay or similar", fill="#fef5e7", stroke="#ca6f1e", sw=1.8,
      title_at="bottom", align="end")
panel(680, 330, 1050, 760, "Waveshare General Driver", fill="#ebf5fb", stroke="#2e86c1", sw=1.8, align="end")
panel(40, 700, 620, 808, "Servo branches", "IDs per servo-map.md §2.1", fill="#fef9e7", stroke="#b7950b", title_at="bottom")
panel(330, 872, 770, 1000, "Compute", fill="#e8f8f5", stroke="#17a589")

# ---------------------------------------------------------------- wires (under the boxes)
# main power
wire([(330, 79), (330, 152)], RED, 3.5, label="XT60", lx=340, ly=96, anchor="start")
wire([(420, 79), (420, 241)], RED, 1.3, dash="5,4", label="balance lead", lx=428, ly=165, anchor="start")
wire([(330, 198), (330, 241)], RED, 3.5)
wire([(250, 299), (250, 361)], RED, 3.5)
wire([(455, 278), (790, 367)], RED, 2.2, label="logic feed · XH 2-pin", lx=690, ly=316, anchor="start")
wire([(860, 413), (860, 457)], RED, 2.2)
wire([(860, 503), (860, 547)], RED, 2.2)

# switched servo rail
wire([(175, 419), (175, 467)], ORANGE, 3.5)
wire([(175, 513), (175, 571)], ORANGE, 3.5)
wire([(175, 629), (175, 680)], ORANGE, 3.5, end=False)
wire([(112, 680), (547, 680)], ORANGE, 3.5, end=False)
for bx in (112, 257, 402, 547):
    wire([(bx, 680), (bx, 717)], ORANGE, 3.5)

# 5 V
wire([(835, 593), (800, 661)], BLUE, 2.4)
wire([(895, 593), (950, 667)], BLUE, 2.4)
wire([(965, 713), (965, 850), (470, 850), (470, 925)], BLUE, 2.4, label="5 V ×2 · GND ×2", lx=880, ly=843)
wire([(540, 950), (580, 950)], BLUE, 1.4, label="CSI", lx=560, ly=942)

# servo bus data
wire([(690, 705), (645, 705), (645, 600), (285, 600)], INK, 1.5,
     label="H5 · UART1, 1 Mbaud\nDATA + GND (V+ unused)", lx=470, ly=578)

# I2C and control signals
wire([(430, 299), (430, 361)], GREEN, 1.4, dash="5,4", start=True, label="I²C · KILL", lx=438, ly=348, anchor="start")
wire([(295, 390), (395, 390)], GREEN, 1.4, dash="5,4", start=True, label="SERVO_EN · rail V", lx=345, ly=382)
wire([(595, 400), (660, 400), (660, 640), (760, 640), (760, 661)], GREEN, 2.2, dash="6,4", start=True,
     label="I²C (P1) + 5 signals", lx=656, ly=520, rotate=-90)
wire([(730, 150), (455, 255)], GREEN, 1.4, dash="5,4", label="TS2 wake (+ INT)", lx=560, ly=236)
wire([(715, 230), (650, 230), (650, 322), (280, 322), (280, 361)], DARKRED, 2.0, dash="6,4",
     label="NC contact → EN (+ state)", lx=545, ly=338)
wire([(430, 925), (430, 835), (760, 835), (760, 719)], GREEN, 1.4, dash="5,4", label="halted", lx=560, ly=829)

# ---------------------------------------------------------------- boxes
box(330, 50, 270, 58, "3S LiPo pack", ("11.1 V, not HV · XT60 + balance lead", "4000 mAh (Gens ace 30C class)"),
    fill="#fdebd0", stroke="#b9770e", url="https://gensace.de/products/gea403s30x6gt")
box(840, 150, 220, 46, "POWER button", ("momentary · on the torso shell",), fill="#d5f5e3", stroke=GREEN)
box(840, 230, 250, 46, "E-STOP button", ("latching, guarded · on the torso shell",), fill="#f5b7b1", stroke=DARKRED, shape="octagon")

box(330, 175, 160, 46, "Fuse", ("MINI blade, 15 A",))
box(330, 270, 250, 58, "TI BQ76922", ("cell protection · battery monitor", "main switch (high-side N-FETs)"),
    fill="#fadbd8", stroke=DARKRED, sw=1.8, url="https://www.ti.com/product/BQ76922")
box(175, 390, 240, 58, "TI TPS48111-Q1", ("servo-rail switch", "soft start · current limit · short trip"),
    fill="#fae5d3", stroke="#ba4a00", sw=1.8, url="https://www.ti.com/product/TPS4811-Q1")
box(495, 390, 200, 58, "Control connector", ("I²C · KILL · SERVO_EN", "INT · rail V · E-stop state"),
    fill="#e9f7ef", stroke=GREEN)
box(175, 490, 180, 46, "Bulk capacitor", ("≥ 1000 µF, ≥ 25 V",))
box(175, 600, 220, 58, "Servo headers ×4", ("V+ from the switched rail", "DATA + GND from H5"))

for bx, name, ids in ((112, "R leg", "9 · 1 · 2 · 3 · 4 · 11"), (257, "R upper", "13 (neck) · 14 · 15"),
                      (402, "L leg", "10 · 5 · 6 · 7 · 8 · 12"), (547, "L upper", "16 · 17")):
    box(bx, 745, 130, 46, name, (ids,))

box(860, 390, 200, 46, "H1 inlet", ("AO4407 · SW1 (left on)",))
box(860, 480, 220, 46, "INA219  0x42", ("pack V · logic + Pi current",), url="https://www.ti.com/product/INA219")
box(860, 570, 200, 46, "MP8759 5 V buck", ("8 A",), url="https://www.monolithicpower.com/en/mp8759.html")
box(785, 690, 190, 58, "ESP32 + IMU", ("50 Hz loop · battguard", "power sequencer"), fill="#d6eaf8", stroke="#1f618d", sw=1.8)
box(965, 690, 130, 46, "40-pin header", ("5 V",))

box(450, 950, 180, 50, "Raspberry Pi 4B", ("vision · navigation",), fill="#d1f2eb", stroke="#117a65",
    url="https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf")
box(665, 950, 170, 46, "Camera Module 3", (), url="https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf")

# ---------------------------------------------------------------- key, bottom left
panel(40, 860, 300, 1030, "Key", stroke="#aab7b8")
rows = ((RED, 3.5, None, "pack / main power"), (ORANGE, 3.5, None, "switched servo rail"), (BLUE, 2.4, None, "5 V"),
        (INK, 1.5, None, "servo bus data"), (GREEN, 1.4, "5,4", "I²C and control signals"))
for i, (c, sw, dash, label) in enumerate(rows):
    y = 900 + 22 * i
    wire([(58, y), (108, y)], c, sw, dash=dash, end=False)
    text(120, y + 4, label, size=11, anchor="start")
text(58, 1018, "boxes with a part number link to it", size=9.5, anchor="start", fill="#566573")

# ---------------------------------------------------------------- assemble
defs = "".join(
    f'<marker id="{marker_id(c)}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" '
    f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
    for c in (RED, ORANGE, BLUE, INK, GREEN, DARKRED))
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
       f'font-family="{FONT}">\n<defs>{defs}</defs>\n<rect width="{W}" height="{H}" fill="white"/>\n'
       + "\n".join(out) + "\n</svg>\n")
Path(__file__).with_name("power_architecture.svg").write_text(svg, encoding="utf-8")
