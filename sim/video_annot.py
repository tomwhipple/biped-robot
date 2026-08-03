"""Shared video-annotation helpers for every renderer in sim/ and sim/mjx/.

Extracted from render_precision_reel.py (2026-08-03) so the caption style --
and, more importantly, its honesty rules -- are reused instead of reinvented
per script. The rules travel with the code (memory: video-quality-review):

  * every take carries its verdict: caption PASS or FAIL, never trim a FAIL
    into looking like a PASS;
  * best-of-N selection is stated, not silent (put "seed k/N" or similar in
    the headline);
  * motion quality is judged with ghost() / frame_strip(), not by squinting
    at a playing video -- wobble shows as blur in a ghost composite.

Font loading is cross-platform (macOS system fonts, Linux DejaVu -- the
training host renders too); the PIL default bitmap font is the last resort,
not the Linux default.
"""
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

BAR_H = 44          # caption bar height (px)

_FONT_CANDIDATES = (
    "/System/Library/Fonts/Helvetica.ttc",                    # macOS
    "/System/Library/Fonts/Supplemental/Arial.ttf",           # macOS
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",        # Debian/Ubuntu
    "/usr/share/fonts/dejavu/DejaVuSans.ttf",                 # Fedora
)

_font_cache = {}


def load_font(size):
    """Truetype font at `size`, first available platform candidate."""
    if size not in _font_cache:
        for cand in _FONT_CANDIDATES:
            if os.path.exists(cand):
                _font_cache[size] = ImageFont.truetype(cand, size)
                break
        else:
            _font_cache[size] = ImageFont.load_default()
    return _font_cache[size]


def caption(frame, title, verdict=None, headline="", t=None, total_t=None,
            bar_h=BAR_H):
    """Frame + top caption bar: title, PASS/FAIL verdict, headline, timecode.

    `frame` is an HxWx3 uint8 array; returns (H+bar_h)xWx3. `verdict` is
    "PASS", "FAIL" or None (no verdict chip); `t` seconds prints a timecode
    under the bar.
    """
    h, w = frame.shape[:2]
    img = Image.new("RGB", (w, h + bar_h), (16, 16, 20))
    img.paste(Image.fromarray(np.asarray(frame)), (0, bar_h))
    d = ImageDraw.Draw(img)
    # Right side builds right-to-left with measured widths, so verdict and
    # headline never collide with the title on narrow frames (the fixed
    # w-200 offsets of the original did, below ~500 px).
    x = w - 10
    if headline:
        d.text((x, 9), headline, font=load_font(15), fill=(180, 180, 180),
               anchor="ra")
        x -= d.textlength(headline, font=load_font(15)) + 12
    if verdict is not None:
        color = (90, 210, 120) if verdict == "PASS" else (235, 110, 90)
        d.text((x, 7), verdict, font=load_font(19), fill=color, anchor="ra")
        x -= d.textlength(verdict, font=load_font(19)) + 12
    # ... and the title ellipsizes into whatever space remains.
    avail = x - 10
    if d.textlength(title, font=load_font(19)) > avail:
        while title and d.textlength(title + "…",
                                     font=load_font(19)) > avail:
            title = title[:-1]
        title = title.rstrip() + "…"
    d.text((10, 7), title, font=load_font(19), fill=(235, 235, 235))
    if t is not None:
        d.text((10, bar_h + 6), f"{t:4.1f}s", font=load_font(14),
               fill=(200, 200, 200))
    return np.asarray(img)


def title_card(w, h, title, idx=None, n=None, bar_h=BAR_H):
    """Full-frame interstitial: centred title, optional take counter idx/n.
    Same height as caption() output so cards and takes concatenate."""
    img = Image.new("RGB", (w, h + bar_h), (16, 16, 20))
    d = ImageDraw.Draw(img)
    d.text((w // 2, (h + bar_h) // 2 - 20), title, font=load_font(30),
           fill=(235, 235, 235), anchor="mm")
    if idx is not None and n is not None:
        d.text((w // 2, (h + bar_h) // 2 + 22), f"{idx}/{n}",
               font=load_font(18), fill=(150, 150, 150), anchor="mm")
    return np.asarray(img)


def ghost(frames):
    """Mean of frames -- the wobble detector. Over a should-be-still phase
    (~25 frames), a calm robot ghosts sharp; wobble shows as blur/doubling.
    Look at this instead of claiming stillness from a playing video."""
    if not len(frames):
        raise ValueError("ghost() needs at least one frame")
    acc = np.zeros(np.asarray(frames[0]).shape, dtype=np.float64)
    for f in frames:
        acc += np.asarray(f, dtype=np.float64)
    return (acc / len(frames)).astype(np.uint8)


def frame_strip(frames, k=6, label_t=None):
    """K evenly-spaced frames side by side -- the consecutive-motion reader.
    `label_t` (seconds per source frame) stamps a timecode on each cell."""
    if not len(frames):
        raise ValueError("frame_strip() needs at least one frame")
    idx = np.linspace(0, len(frames) - 1, min(k, len(frames))).astype(int)
    cells = []
    for i in idx:
        cell = np.asarray(frames[i])
        if label_t is not None:
            cell = caption(cell, f"{i * label_t:5.2f}s", bar_h=28)
        cells.append(cell)
    return np.concatenate(cells, axis=1)


def trim_trailing_still(frames, fps, keep_s=2.0, thr=1.0):
    """Cut a trailing standstill down to `keep_s` seconds (user 2026-07-25):
    finds the last frame with visible motion (mean pixel delta > thr) and
    keeps keep_s beyond it. Trims boredom, never a moving frame."""
    if len(frames) <= 3:
        return frames
    deltas = [float(np.mean(np.abs(np.asarray(frames[j], dtype=np.int16)
                                   - np.asarray(frames[j - 1],
                                                dtype=np.int16))))
              for j in range(1, len(frames))]
    last_mv = max((j for j, dl in enumerate(deltas, 1) if dl > thr),
                  default=len(frames) - 1)
    return frames[:min(len(frames), last_mv + int(keep_s * fps))]
