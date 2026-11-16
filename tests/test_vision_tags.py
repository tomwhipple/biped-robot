"""tools/vision: tag sheet and the telemetry-only implausible-recovery check.
Synthetic images and arrays only; no camera, no flight record."""
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "vision"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import implausible as im  # noqa: E402
import make_tags  # noqa: E402
from tagdefs import TAGS, detect, tag_image  # noqa: E402


def _scene(tid, px):
    m = max(2, px // 8)
    img = np.full((px + 2 * m + 40, px + 2 * m + 40), 120, np.uint8)
    img[20:20 + px + 2 * m, 20:20 + px + 2 * m] = 255
    img[20 + m:20 + m + px, 20 + m:20 + m + px] = cv2.resize(
        tag_image(tid, 8 * 16), (px, px), interpolation=cv2.INTER_AREA)
    return img


def test_every_tag_decodes_at_24px():
    for tid in TAGS:
        assert tid in detect(_scene(tid, 24)), tid


def test_sheet_builds(tmp_path):
    pdf, pages = make_tags.build(str(tmp_path))
    assert os.path.getsize(pdf) > 10000 and len(pages) == 4
    assert pages[0].size == (2550, 3300)       # US Letter at 300 dpi


def _series(roll_deg_fn, fall_at=None, n=1000, dt=0.02):
    t = 1000.0 + np.arange(n) * dt
    roll = np.array([roll_deg_fn(x - 1000.0) for x in t])
    up = np.stack([np.zeros(n), np.zeros(n), np.cos(np.radians(roll))], 1)
    flags = np.zeros(n, int)
    if fall_at is not None:
        flags[t >= 1000.0 + fall_at] = 128
    return dict(t=t, roll_deg=roll, pitch_deg=np.zeros(n), flags=flags, up=up)


def test_quick_recovery_is_flagged():
    d = _series(lambda x: 30 if 5 < x < 6.5 else 0)
    ex = im.find_excursions(d, [])
    assert len(ex) == 1 and ex[0]["implausible"]


def test_fall_or_slow_or_small_not_flagged():
    d = _series(lambda x: 30 if 5 < x < 6.5 else 0, fall_at=6.0)
    assert not im.find_excursions(d, [])[0]["implausible"]
    d = _series(lambda x: 30 if 5 < x < 14 else 0)
    assert not im.find_excursions(d, [])[0]["implausible"]
    d = _series(lambda x: 12 if 5 < x < 6 else 0)
    assert im.find_excursions(d, []) == []
