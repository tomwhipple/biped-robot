#!/usr/bin/env python3
"""Print-ready AprilTag 36h11 sheet (US Letter, print at 100 %) + placement guide.

    tools/vision/make_tags.py [-o OUTDIR]

Writes apriltags_letter.pdf (4 pages: board tags, board mid-edge + torso, torso top/feet/spares, placement guide)
and apriltags_letter_p<N>.png (300 dpi) into OUTDIR (default
hw_sessions/2026-11-16/vision). Needs only OpenCV + PIL.
"""
import argparse
import os

import cv2
from PIL import Image, ImageDraw, ImageFont
from PIL import JpegImagePlugin  # noqa: F401  (the PDF writer embeds JPEG)

from tagdefs import TAGS, tag_image

DPI = 300
W, H = int(8.5 * DPI), int(11 * DPI)
MM = DPI / 25.4
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

CORNER = {0: "far-left", 1: "far-right", 2: "near-left", 3: "near-right"}

GUIDE = """PLACEMENT GUIDE  (print at 100 % / "actual size"; check the 50 mm bars)

Orientation words are as seen from the Sony, which sits at the near end of the
board looking along it. "Up" on every tag = the tag's top edge (the label is
printed above the tag, the arrow points to its up).

BOARD TAGS (80 mm), lying flat on the board, quiet zone left white:
  ID 0 far-left, ID 1 far-right, ID 2 near-left, ID 3 near-right.
  Tag up-arrow points away from the Sony (toward the far end) on all of them.
  Put each tag with its black square's outer edge exactly 20 mm in from BOTH
  board edges at its corner (the tag centre is then 60 mm from each edge);
  record the real distances in the calibration file.
  Optional ID 4 / 5: at the middle of the left / right long edge, same 20 mm
  inset, arrow pointing away from the Sony.
  Flat means: tape the whole sheet down (no curl, no lifted corner), matte
  paper or matte tape over it (glare kills detection). Tags on two panels:
  do not straddle the seam.

ROBOT TAGS (60 mm on the torso; 40 mm spares of 10-12 are on page 3):
  ID 10 torso FRONT: on the front face of the torso box, arrow up.
  ID 11 torso BACK : on the back face, arrow up.
  ID 12 torso TOP  : on the top face, facing up; arrow pointing to the
                     robot's FRONT.
  If a 60 mm tag does not fit its face (torso is ~90 mm wide), use the 40 mm
  spare of the same ID and tell Claude which size is mounted: the size goes
  into the tracker's calibration.
  Optional 25 mm: ID 13 left foot top, ID 14 right foot top, arrow forward.
  Each tag flat to its face (stiff card, no wrinkles), upright edge parallel
  to the torso edge to within 1 degree; measure and record the offset from
  the torso centre. White quiet zone of at least one module (7.5 mm for
  60 mm tags, 5 mm for 40 mm, 3 mm for 25 mm) must stay free: do not mount over a seam or screw.

AFTER PLACING: tell Claude, who will take a calibration capture.
"""


def font(size, bold=False):
    return ImageFont.truetype(FONTB if bold else FONT, size)


def mm(v):
    return int(round(v * MM))


def scale_bar(d, x, y):
    """A 50 mm bar with ticks every 10 mm, starting at (x, y) px."""
    d.line([(x, y), (x + mm(50), y)], fill=0, width=4)
    for k in range(6):
        d.line([(x + mm(10 * k), y - 12), (x + mm(10 * k), y + 12)], fill=0, width=3)
    d.text((x + mm(50) + 15, y - 14), "50 mm - must measure 50 mm", fill=0, font=font(28))


def place(page, d, tag_id, cx, top, size=None):
    """Draw tag centred horizontally at cx; label above; returns y below the cell."""
    size0, role = TAGS[tag_id]
    size = size or size0
    px = mm(size)
    q = mm(size / 8)  # quiet zone: one module
    spare = " (SPARE)" if size != size0 else ""
    d.text((cx, top), f"ID {tag_id} - {size} mm{spare}", fill=0, font=font(32, True), anchor="mt")
    d.text((cx, top + 42), role, fill=0, font=font(24), anchor="mt")
    ty = top + 90
    cell = (cx - px // 2 - q, ty, cx + px // 2 + q, ty + px + 2 * q)
    d.rectangle(cell, outline=170, width=2)  # cut line, outside the quiet zone
    img = Image.fromarray(tag_image(tag_id, px))
    page.paste(img, (cx - px // 2, ty + q))
    # up arrow beside the tag
    ax = cell[2] + 14
    d.line([(ax, ty + q + px), (ax, ty + q)], fill=0, width=4)
    d.polygon([(ax - 10, ty + q + 20), (ax + 10, ty + q + 20), (ax, ty + q)], fill=0)
    return cell[3]


def sheet_page(title):
    page = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(page)
    d.text((mm(10), mm(8)), title, fill=0, font=font(40, True))
    return page, d


def build(outdir):
    pages = []
    # page 1: board tags 0-3
    page, d = sheet_page("Board tags (flat on board; arrow = away from the Sony)")
    cols = [W // 4 + 10, 3 * W // 4 - 10]
    y = mm(22)
    for row in ((0, 1), (2, 3)):
        bottom = y
        for cx, tid in zip(cols, row):
            bottom = place(page, d, tid, cx, y)
            d.text((cx, bottom + 8), f"corner: {CORNER[tid]} (seen from the Sony)", fill=0,
                   font=font(28), anchor="mt")
        y = bottom + mm(14)
    scale_bar(d, mm(20), H - mm(18))
    pages.append(page)
    # page 2: mid-edge board tags + torso front/back
    page, d = sheet_page("Mid-edge board tags + torso front / back")
    y = mm(22)
    bottom = y
    for cx, tid in zip(cols, (4, 5)):
        bottom = place(page, d, tid, cx, y)
    y = bottom + mm(14)
    for cx, tid in zip(cols, (10, 11)):
        bottom = place(page, d, tid, cx, y)
    scale_bar(d, mm(20), H - mm(18))
    pages.append(page)
    # page 3: torso top, feet, spare 40 mm torso tags
    page, d = sheet_page("Torso top, foot tags, spare 40 mm torso tags")
    y = mm(22)
    bottom = y
    for cx, tid in zip((W // 4, int(W * 0.58), int(W * 0.84)), (12, 13, 14)):
        bottom = max(bottom, place(page, d, tid, cx, y))
    y = bottom + mm(14)
    d.text((mm(12), y), "Spares, in case 60 mm does not fit the real torso (same IDs):",
           fill=0, font=font(30, True))
    y += 50
    for cx, tid in zip((W // 6, W // 2, 5 * W // 6), (10, 11, 12)):
        place(page, d, tid, cx, y, size=40)
    scale_bar(d, mm(20), H - mm(18))
    pages.append(page)
    # page 4: guide
    page, d = sheet_page("Placement guide")
    y = mm(22)
    for line in GUIDE.splitlines():
        d.text((mm(12), y), line, fill=0, font=ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 27))
        y += 40
    scale_bar(d, mm(20), H - mm(18))
    pages.append(page)

    os.makedirs(outdir, exist_ok=True)
    pdf = os.path.join(outdir, "apriltags_letter.pdf")
    pages[0].convert("RGB").save(pdf, save_all=True, resolution=DPI, quality=98,
                                 append_images=[p.convert("RGB") for p in pages[1:]])
    for i, p in enumerate(pages, 1):
        p.save(os.path.join(outdir, f"apriltags_letter_p{i}.png"), dpi=(DPI, DPI))
    with open(os.path.join(outdir, "placement_guide.md"), "w") as f:
        f.write("```\n" + GUIDE + "```\n")
    return pdf, pages


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    default = os.path.join("hw_sessions", "2026-11-16", "vision")
    ap.add_argument("-o", "--outdir", default=default)
    a = ap.parse_args()
    print(build(a.outdir)[0])


if __name__ == "__main__":
    main()
