"""tools/vision/tag_track.py on synthetic renders (no camera, no footage with tags exists yet).
Camera: f = 870 px, 1024x680, 475 mm high, 500 mm in front of the board, pitched 20 deg down --
the geometry fitted to the real Sony footage; the near corner tags are out of frame there, so
the board is calibrated from tags 4 and 5 (and the far tags when they decode)."""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "vision"))
import run_report  # noqa: E402,F401
import vsynth as synth  # noqa: E402
import tag_track as T  # noqa: E402
import vgeom as G  # noqa: E402
from tagdefs import detect  # noqa: E402

SIZE = (1024, 680)
F_TRUE = 870.0
K = G.make_K(F_TRUE, SIZE)
R_CB, T_CB = synth.look_at([450, -500, 475], [450, 400, 150])
CFG = G.load_config()


def render(cfg, hd, pt, rl, x, y, z=170.0, seed=0, robot=True, scale=1.0):
    ids = (10, 11, 12) if robot else ()
    tags = synth.scene_tags(cfg, G.board_R_from_euler(hd, pt, rl), [x, y, z], robot_ids=ids)
    if scale != 1.0:      # a board (print and layout) `scale` times the configured size
        tags = [(i, s * scale, R, t * scale) if i < 6 else (i, s, R, t) for i, s, R, t in tags]
    return synth.render(tags, K, R_CB, T_CB, SIZE, board_rect=(900, 1200), seed=seed, blur=1.0, noise=4)


@pytest.fixture(scope="module")
def tracker():
    tr = T.Tracker(CFG, SIZE)
    ds = [detect(render(CFG, 0, 0, 0, 450, 600, robot=False, seed=s), tr.det) for s in range(3)]
    cal = tr.calibrate(ds)
    assert cal is not None
    tr.cal = cal
    return tr


def test_focal_and_board_pose(tracker):
    cal = tracker.cal
    assert abs(cal["f_px"] - F_TRUE) / F_TRUE < 0.04          # observed: 856 vs 870 (1.6 %)
    assert cal["board_rms_px_median"] < 0.6
    cam = np.array(cal["camera_position_board_mm"])
    assert np.linalg.norm(cam - [450, -500, 475]) < 40


def test_walk_sequence_within_spec(tracker):
    tracker._last = None
    for k in range(0, 60, 3):
        y = 300 + k * 12
        x = 450 + 60 * np.sin(k / 9)
        hd, pt, rl = 15 * np.sin(k / 7), 6 * np.sin(k / 5), 8 * np.sin(k / 4 + 1)
        r = tracker.process(render(CFG, hd, pt, rl, x, y, seed=k), k * 0.04)
        assert r["stale"] is False
        assert abs(r["x_mm"] - x) < 10 and abs(r["y_mm"] - y) < 10, (k, r["x_mm"], r["y_mm"])
        assert abs(r["heading_deg"] - hd) < 3, k
        assert abs(r["pitch_deg"] - pt) < 2 and abs(r["roll_deg"] - rl) < 2, k


def test_facing_the_camera_uses_the_front_tag(tracker):
    tracker._last = None
    r = tracker.process(render(CFG, 180, 0, 0, 450, 500), 0.0)
    assert r["used_tags"] == [10]
    assert abs(abs(r["heading_deg"]) - 180) < 3 and abs(r["x_mm"] - 450) < 10 and abs(r["y_mm"] - 500) < 10


def test_edge_margins_and_nearest_edge(tracker):
    tracker._last = None
    r = tracker.process(render(CFG, 0, 0, 0, 150, 1000), 0.0)
    assert r["margin_min_edge"] == "left" and abs(r["margin_left_mm"] - 150) < 10
    assert abs(r["margin_far_mm"] - 200) < 10 and abs(r["margin_right_mm"] - 750) < 10


def test_hidden_robot_is_held_then_stale_then_lost(tracker):
    tracker._last = None
    good = tracker.process(render(CFG, 0, 0, 0, 450, 600), 10.0)
    assert good["stale"] is False
    empty = render(CFG, 0, 0, 0, 450, 600, robot=False)
    held = tracker.process(empty, 10.3)
    assert held["stale"] is True and abs(held["stale_s"] - 0.3) < 1e-6 and held["x_mm"] == good["x_mm"]
    lost = tracker.process(empty, 20.0)
    assert lost["stale"] is None and "x_mm" not in lost


def test_board_scale_is_not_observable_but_shows_in_the_camera_position():
    """A board 2 % smaller than configured gives the same image from a camera 2 % closer to it: the
    images cannot tell, so the report's camera position (compare with a tape measure) is the check."""
    cfg = G.load_config()
    tr = T.Tracker(cfg, SIZE)
    ds = [detect(render(cfg, 0, 0, 0, 450, 600, robot=False, seed=s, scale=0.98), tr.det) for s in range(2)]
    cam = np.array(tr.calibrate(ds)["camera_position_board_mm"])
    assert np.linalg.norm(cam - np.array([450, -500, 475]) / 0.98) < 45     # in configured mm, farther


def test_config_override_merges():
    cfg = G.load_config()
    assert cfg["board"]["tags"]["0"]["size"] == 80.0 and cfg["robot"]["tags"]["10"]["size"] == 60.0


def test_video_end_to_end_with_wall_clock_and_report_flags(tmp_path):
    import cv2
    path = str(tmp_path / "v.mp4")
    w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 25.0, SIZE)
    base = 1.8e9
    with open(str(tmp_path / "v.pts"), "w") as f:
        f.write("# timecode format v2\n")
        for k in range(30):
            y = 400 + 15 * k
            w.write(cv2.cvtColor(render(CFG, 5, 2, -3, 450, y, seed=k), cv2.COLOR_GRAY2BGR))
            f.write(f"{(base + k * 0.04) * 1000:.3f}\n")
    w.release()
    rows, cal = T.track_video(path, CFG, stride=3, calib_frames=6)
    assert cal is not None and abs(cal["f_px"] - F_TRUE) / F_TRUE < 0.05
    good = [r for r in rows if r.get("stale") is False]
    assert len(good) >= 8 and abs(good[0]["t"] - base) < 1e-3
    assert abs(good[-1]["y_mm"] - (400 + 15 * good[-1]["frame"])) < 40       # mp4v compression adds error
    import run_report as RR
    flags = RR.tag_flags(rows, None)
    assert flags["edge_margin_min_mm"] > 0 and flags["position_end"][1] > flags["position_start"][1]
