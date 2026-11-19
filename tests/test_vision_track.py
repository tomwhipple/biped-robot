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
import vsynth as synth  # noqa: E402
import tag_track as T  # noqa: E402
import vgeom as G  # noqa: E402
from tagdefs import detect  # noqa: E402

SIZE = (1024, 680)
F_TRUE = 870.0
K = G.make_K(F_TRUE, SIZE)
R_CB, T_CB = synth.look_at([450, -500, 475], [450, 400, 150])
# the Sony-geometry tests use the full measured-style layout and a front tag; the eMeet defaults
# (auto layout, tags 11/12) are tested below
CFG = G.load_config()
CFG["board"].update(layout="config", width_mm=900.0, length_mm=1200.0)   # the synthetic Sony scene
CFG["robot"]["tags"] = {
    "10": {"center": [30.0, 0.0, 0.0], "normal": [1, 0, 0], "up": [0, 0, 1], "size": 60.0},
    "11": {"center": [-30.0, 0.0, 0.0], "normal": [-1, 0, 0], "up": [0, 0, 1], "size": 60.0},
    "12": {"center": [0.0, 0.0, 45.0], "normal": [0, 0, 1], "up": [1, 0, 0], "size": 60.0},
}


def render(cfg, hd, pt, rl, x, y, z=170.0, seed=0, robot=True, scale=1.0):
    ids = None if robot else ()
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
    assert abs(r["margin_far_mm"] - 200) < 15 and abs(r["margin_right_mm"] - 750) < 15


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
    cfg = CFG
    tr = T.Tracker(cfg, SIZE)
    ds = [detect(render(cfg, 0, 0, 0, 450, 600, robot=False, seed=s, scale=0.98), tr.det) for s in range(2)]
    cam = np.array(tr.calibrate(ds)["camera_position_board_mm"])
    assert np.linalg.norm(cam - np.array([450, -500, 475]) / 0.98) < 45     # in configured mm, farther


def test_defaults_and_config_override(tmp_path):
    cfg = G.load_config()
    assert cfg["board"]["layout"] == "config" and cfg["board"]["measured"] is False
    assert cfg["board"]["width_mm"] == 812.8 and cfg["board"]["length_mm"] == 1219.2          # 32 x 48 in
    assert sorted(cfg["robot"]["tags"]) == ["11", "12"]
    assert cfg["robot"]["tags"]["11"]["size"] == 40.0 and cfg["robot"]["tags"]["12"]["size"] == 60.0
    p = tmp_path / "v.json"
    p.write_text('{"board": {"tags": {"2": {"center": [100, 50], "size": 79}}}}')
    m = G.load_config(str(p))
    assert m["board"]["layout"] == "config" and m["board"]["measured"] is True
    assert list(m["board"]["tags"]) == ["2"]                    # measured -> used as given


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


# ---- eMeet high mount: 1920x1080, ~f 1000, looking down at the tag end of the board ----------
ESIZE = (1920, 1080)
EF = 1000.0
EK = G.make_K(EF, ESIZE)
E_R, E_T = synth.look_at([400, 1100, 650], [400, 500, 0])
# board-end tags 2 and 3 as measured on the real frame: ~704 mm apart, symmetric about the board
ETAGS = {"2": {"center": [98.0, 0.0], "size": 80.0}, "3": {"center": [802.0, 0.0], "size": 80.0}}


def erender(cfg, hd, pt, rl, x, y, z=170.0, seed=0):
    cfg = {**cfg, "board": {**cfg["board"], "tags": ETAGS}}        # the scene has the measured layout
    tags = synth.scene_tags(cfg, G.board_R_from_euler(hd, pt, rl), [x, y, z], board_ids=[2, 3])
    return synth.render(tags, EK, E_R, E_T, ESIZE, blur=1.0, noise=3, seed=seed, floor=170)


def test_emeet_auto_layout_from_one_frame_and_measured_layout():
    cfg = G.load_config()                       # tags 11 (40 mm) and 12 (60 mm); no measured layout:
    cfg["board"]["layout"] = "auto_end"         # infer a frame from the two board-end tags
    img = erender(cfg, 160, 4, -3, 420, 600)
    tr = T.Tracker(cfg, ESIZE)
    cal = tr.calibrate([detect(img, tr.det)])
    assert cal["layout"].startswith("auto_end") and cal["tags_used"] == [2, 3]
    assert 600 < cal["f_px"] < 1400                                  # coplanarity is a shallow minimum
    assert abs(cal["tag_spacing_mm_measured"] - 704) < 60
    r = tr.process(img, 0.0, detect(img, tr.det))
    assert sorted(r["used_tags"]) == [11, 12] and abs(r["pitch_deg"] - 4) < 4 and abs(r["roll_deg"] + 3) < 4
    # measured layout + known f: position to a few mm, heading within 3 deg
    cfg2 = G.load_config()
    cfg2["board"]["layout"] = "config"
    cfg2["board"]["tags"] = ETAGS
    cfg2["camera"]["f_px"] = EF
    tr2 = T.Tracker(cfg2, ESIZE)
    d = detect(img, tr2.det)
    cal2 = tr2.calibrate([d])
    assert cal2["layout"].startswith("config") and max(v["rms_px"] for v in cal2["per_tag_rms_px"].values()) < 1.0
    r2 = tr2.process(img, 0.0, d)
    assert abs(r2["x_mm"] - 420) < 10 and abs(r2["y_mm"] - 600) < 10
    assert abs((r2["heading_deg"] - 160 + 180) % 360 - 180) < 3


def test_calibrate_frame_reports_per_tag_errors():
    import calibrate as CAL
    cfg = G.load_config()
    cfg["board"]["layout"] = "auto_end"
    out, _, _ = CAL.calibrate_frame(erender(cfg, 160, 0, 0, 420, 600), cfg)
    assert set(out["tag_sizes_px"]) >= {"2", "3", "11", "12"} and out["robot"]["used_tags"] == [11, 12]
    assert out["per_tag_pnp_rms_px"]["2"] < 1 and "camera_position_board_mm" in out
    blank, _, _ = CAL.calibrate_frame(erender(cfg, 160, 0, 0, 420, 600) * 0 + 150, cfg)
    assert "error" in blank


REAL = "/home/claw/code/robot/hw_sessions/2026-11-16/vision/calib_emeet_reaimed_1080p.jpg"


@pytest.mark.skipif(not os.path.exists(REAL), reason="real eMeet calibration frame not on this machine")
def test_real_emeet_frame_decodes_tags_and_calibrates():
    import calibrate as CAL
    import cv2
    out, _, _ = CAL.calibrate_frame(cv2.imread(REAL, cv2.IMREAD_GRAYSCALE))      # provisional layout, 812.8 x 1219.2
    assert set(out["tag_sizes_px"]) == {"2", "3", "11", "12"} and out["layout"].startswith("config (PROVISIONAL")
    assert 600 < out["f_px"] < 1500
    fit = out["layout_fit"]["2-3"]                           # observed 1.017: the provisional layout fits
    assert 0.9 < fit["ratio"] < 1.1 and max(v["rms_px"] for v in out["per_tag_rms_px"].values()) < 3
    assert out["robot"]["used_tags"] == [11, 12] and abs(abs(out["robot"]["heading_deg"]) - 180) < 10
