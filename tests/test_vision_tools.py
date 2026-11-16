"""camd / clip / sony_record refusal / where / run_report pieces. Synthetic sources only
(ffmpeg lavfi); the real Sony is never opened. Set VISION_CAMERA_TESTS=1 to also run the
(unwritten-by-default) hardware test."""
import os
import shutil
import subprocess
import sys
import time

import cv2
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIS = os.path.join(ROOT, "tools", "vision")
sys.path.insert(0, VIS)
sys.path.insert(0, os.path.join(ROOT, "tests"))
import clip  # noqa: E402
import camd  # noqa: E402
import run_report as RR  # noqa: E402
import where as W  # noqa: E402
import implausible as IM  # noqa: E402
import vgeom as G  # noqa: E402

needs_ffmpeg = pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
PY = sys.executable


@needs_ffmpeg
def test_camd_segments_lock_clip_and_latest(tmp_path):
    d = str(tmp_path)
    p = subprocess.Popen([PY, os.path.join(VIS, "camd.py"), "--source", "lavfi", "--dir", d,
                          "--segment-s", "2", "--duration", "6"])
    try:
        time.sleep(2.5)
        second = subprocess.run([PY, os.path.join(VIS, "camd.py"), "--source", "lavfi", "--dir", d, "--duration", "1"],
                                capture_output=True, text=True)
        assert second.returncode == 3 and "refusing" in second.stderr
        assert os.path.getsize(os.path.join(d, "lavfi_latest.jpg")) > 100
    finally:
        p.wait(timeout=30)
    pts = sorted(f for f in os.listdir(d) if f.endswith(".pts"))
    assert len(pts) >= 2 and all(os.path.getsize(os.path.join(d, f)) > 20 for f in pts)
    times = [t for f in pts for t in clip.read_pts(os.path.join(d, f))]
    out = os.path.join(d, "c.mp4")
    n = clip.make_clip(times[0] + 1.0, times[0] + 4.0, out, d, "lavfi")
    assert n > 10 and os.path.getsize(out) > 1000
    assert clip.make_clip(times[-1] + 100, times[-1] + 101, os.path.join(d, "none.mp4"), d, "lavfi") == 0


def test_prune_keeps_recent_and_drops_old(tmp_path):
    now = time.time()
    for off in (7200, 100):
        base = camd.seg_stamp(now - off)
        for ext in ("mp4", "pts"):
            (tmp_path / f"sony_{base}Z.{ext}").write_text("x")
    assert camd.prune(str(tmp_path), "sony", 3600, now) == 2
    assert len(list(tmp_path.iterdir())) == 2


@pytest.mark.skipif(shutil.which("flock") is None, reason="flock not installed")
def test_sony_record_refuses_when_camd_lock_is_held(tmp_path):
    lock = tmp_path / "sony.lock"
    lock.write_text("123")
    holder = subprocess.Popen(["flock", str(lock), "sleep", "5"])
    try:
        time.sleep(0.5)
        out = str(tmp_path / "x")
        r = subprocess.run([os.path.join(ROOT, "tools", "sony_record.sh"), out, "1", "t"],
                           env={**os.environ, "CAMD_DIR": str(tmp_path)}, capture_output=True, text=True)
        assert r.returncode == 3 and "camd owns the camera" in r.stderr
        assert not os.path.exists(out + ".mp4") and not os.path.exists(out + ".pts")
    finally:
        holder.kill()


@needs_ffmpeg
def test_sony_record_leaves_no_empty_file_when_the_camera_will_not_start(tmp_path):
    fake = tmp_path / "liveview.py"
    fake.write_text("import sys; sys.exit('camera_init failed: -53 busy')\n")
    out = str(tmp_path / "x")
    r = subprocess.run([os.path.join(ROOT, "tools", "sony_record.sh"), out, "1", "t"],
                       env={**os.environ, "LIVEVIEW": str(fake), "CAMD_DIR": str(tmp_path / "none")},
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 1 and "no file written" in r.stderr
    assert not os.path.exists(out + ".mp4") and not os.path.exists(out + ".pts")


def test_where_exit_codes_and_margins(tmp_path):
    import test_vision_track as t
    img = t.render(t.CFG, 10, 3, -4, 300, 900)
    p = str(tmp_path / "f.png")
    cv2.imwrite(p, img)

    import json
    conf = str(tmp_path / "v.json")
    with open(conf, "w") as f:                      # measured-style config of the Sony synthetic scene
        json.dump({"board": {"layout": "config"}, "robot": {"tags": t.CFG["robot"]["tags"]}}, f)

    def run(direction, margin):
        return subprocess.run([PY, os.path.join(VIS, "where.py"), "--image", p, "--direction", direction,
                               "--min-margin", str(margin), "--config", conf], capture_output=True, text=True)
    assert run("back", 400).returncode == 0
    r = run("forward", 400)
    assert r.returncode == 3 and "TOO CLOSE" in r.stdout
    blank = str(tmp_path / "b.png")
    cv2.imwrite(blank, img * 0 + 120)
    assert subprocess.run([PY, os.path.join(VIS, "where.py"), "--image", blank, "--config", conf],
                          capture_output=True).returncode == 2


def test_margin_along_diagonal():
    cfg = G.load_config()
    cfg["board"].update(width_mm=900.0, length_mm=1200.0)
    assert abs(W.margin_along(cfg, 450, 600, 0, 1) - 600) < 1e-9
    assert abs(W.margin_along(cfg, 100, 600, -1, 0) - 100) < 1e-9
    assert abs(W.margin_along(cfg, 450, 1100, 0.7071, 0.7071) - 100 / 0.7071) < 0.1


def test_log_anomalies_ts_lean_gate_hold_and_stop():
    rows = [
        dict(t=0.0, cmd="cal show", text="x"),
        dict(t=9.0, cmd="track steps", text="1.0 e +0.2 | TS 1 L in 2.5 out 2.5 ok G 0"),   # header cut off
        dict(t=15.0, cmd="track steps", text="TS 1 L in 2.5 out 2.5 | TS 2 R in 9.0 out 2.0 ok G 0"),
        dict(t=20.0, cmd="track", text="track gate: 7 gate frames, 1 holds, 0 ms held, 0 timeouts, HOLDING (clock parked)"),
        dict(t=21.0, cmd="track", text="track gate: 7 gate frames, 1 holds, 0 ms held, 0 timeouts, HOLDING (clock parked)"),
        dict(t=30.0, cmd="track stop", text="track: stopping at the next step end"),
        dict(t=31.0, cmd="imu", text="levelled yes   accel-reject streak 4"),
        dict(t=32.0, cmd="x", text="aborts   none since the last read"),
        dict(summary=True, steps=[dict(n=1, stance="L", lean_in=2.5, lean_out=2.5, gate_ms=0),
                                  dict(n=2, stance="R", lean_in=9.0, lean_out=2.0, gate_ms=0)], gate_timeout=None),
    ]
    a = RR.log_anomalies(rows, 1000.0)
    kinds = [(x["kind"], x["t_utc"]) for x in a]
    assert ("ts_lean", 1015.0) in kinds and ("gate_hold", 1020.0) in kinds and ("stop", 1030.0) in kinds
    assert len(a) == 3                      # one hold, no false abort from "accel-reject" / "aborts none"


@pytest.mark.skipif(not os.environ.get("VISION_CAMERA_TESTS"), reason="needs the real camera: set VISION_CAMERA_TESTS=1")
def test_real_camera_frame_via_where():
    r = subprocess.run([PY, os.path.join(VIS, "where.py"), "--json"], capture_output=True, text=True, timeout=60)
    assert r.returncode in (0, 2, 3)


def test_pick_videos_prefers_the_emeet_and_keeps_the_sony_as_context(tmp_path):
    for name in ("sony_a", "emeet_a", "other"):
        (tmp_path / f"{name}.mp4").write_text("x")
        (tmp_path / f"{name}.pts").write_text("# timecode format v2\n1000000\n1005000\n")
    prim, ctx = RR.pick_videos([str(tmp_path)], 1000.5, 1004.0)
    assert os.path.basename(prim) == "emeet_a.mp4" and os.path.basename(ctx) == "sony_a.mp4"
    assert RR.pick_videos([str(tmp_path)], 5000, 5001) == (None, None)


def test_find_frec_picks_the_open_session_even_before_the_collector_flushes(tmp_path):
    t0 = 1_794_846_934.0                       # 2026-11-16T16:35:34Z
    old = tmp_path / "flight_20261113T172054Z_prec16.frec"
    new = tmp_path / "flight_20261116T160000Z_prec16.frec"
    for f in (old, new):
        f.write_text("x")
    os.utime(old, (t0 - 90000, t0 - 90000))
    os.utime(new, (t0 + 5, t0 + 5))             # written after the run start, but BEFORE the run's end
    assert IM.find_frec(str(tmp_path), t0, t0 + 74) == str(new)
    assert IM.find_frec(str(tmp_path), t0 - 400000, t0 - 399000) is None


def test_check_run_waits_for_the_record_to_cover_the_end(tmp_path, monkeypatch):
    import numpy as np
    run = tmp_path / "track_20261116T093534.jsonl"
    run.write_text('{"t": 0.0, "cmd": "cal show"}\n{"t": 74.0, "cmd": "x"}\n')
    t0, t1 = IM.run_window(str(run))
    calls = []

    def fake_read(path, a, b):
        calls.append(1)
        end = t1 - 30 if len(calls) < 3 else t1 + 2        # the collector catches up on the 3rd look
        t = np.arange(t0 - 5, end, 0.02)
        return dict(t=t, roll_deg=np.zeros(len(t)), pitch_deg=np.zeros(len(t)), flags=np.zeros(len(t), int),
                    up=np.tile([0, 0, 1.0], (len(t), 1))), []
    monkeypatch.setattr(IM, "find_frec", lambda *a: "fake.frec")
    monkeypatch.setattr(IM, "read_window", fake_read)
    r = IM.check_run(str(run), wait_s=10, poll_s=0.01)
    assert len(calls) == 3 and not r.get("partial") and r["implausible_recovery"] == []
    calls.clear()
    r = IM.check_run(str(run), wait_s=0, poll_s=0.01)           # gives up at once: says partial, not "clean"
    assert r["partial"] and "not established" in r["note"]


# ---- run_report: endings, stationary positions, extent -------------------------------------------------------------
def _ts_rows(n_ts, t_end=70.0):
    rows = [dict(t=0.0, cmd="cal show", text="x")]
    for k in range(1, n_ts + 1):
        rows.append(dict(t=4.0 * k, cmd="track steps", text="".join(f"TS {j} L in 1.7 out 2.2 e +2.0 T +1.8 hike +1.4 ok G 0\n" for j in range(1, k + 1))))
    rows.append(dict(t=t_end, cmd="imu", text="levelled yes"))
    return rows


def _summary(n_steps, n_ts, **kw):
    return dict(summary=True, args=dict(steps=n_steps), gate_timeout=None,
                steps=[dict(n=j, stance="LR"[j % 2], lean_in=1.7, lean_out=2.2, gate_ms=0, ok=True) for j in range(1, n_ts + 1)], **kw)


def test_log_anomalies_a_fall_run_is_a_short_run_and_a_clean_run_is_not():
    # 2026-11-16 14:01: 17 steps decided, TS lines up to 16, no fall or abort text anywhere in the host log
    a = RR.log_anomalies(_ts_rows(16) + [_summary(17, 16)], 1000.0)
    assert [x["kind"] for x in a] == ["short_run"] and a[0]["t_utc"] == 1070.0 and "16 TS lines for 17" in a[0]["detail"]
    assert RR.log_anomalies(_ts_rows(17) + [_summary(17, 17)], 1000.0) == []


def test_log_anomalies_abort_stop_rule_and_gate_timeout_are_listed():
    rows = _ts_rows(5)
    s = _summary(8, 5, outcome=dict(abort=True, stop_rule=True, gate_timeout=False, fell=None))
    s["steps"][3]["ok"] = False
    k = [x["kind"] for x in RR.log_anomalies(rows + [s], 1000.0)]
    assert "firmware_abort" in k and "stop_rule" in k and "short_run" not in k      # the abort explains the short run
    s = _summary(8, 5, outcome=dict(abort=False, stop_rule=True, gate_timeout=True, fell=None))
    s["gate_timeout"] = dict(n=6, stance="R", held_ms=3000, dev=4.0, rate=0.2)
    a = RR.log_anomalies(rows + [s], 1000.0)
    gt = [x for x in a if x["kind"] == "gate_timeout"]
    assert gt and gt[0]["t_utc"] == 1070.0                                          # was dropped for want of a time
    assert any(x["kind"] == "stop_rule" for x in a)
    s = _summary(8, 5, outcome=dict(abort=False, stop_rule=False, gate_timeout=False, fell=True))
    assert "fall_latch" in [x["kind"] for x in RR.log_anomalies(rows + [s], 1000.0)]


def test_flight_record_fall_latch_times():
    import numpy as np
    t = np.arange(1000.0, 1010.0, 0.5)
    flags = np.zeros(len(t), int)
    flags[10:] = IM.F.TF_FALLEN                      # latched from t = 1005 on: one fall, not 10
    d = dict(t=t, flags=flags)
    assert IM.fall_times(d, [dict(kind="fall", t=1004.9), dict(kind="cmd", t=1001.0)]) == [1004.9, 1005.0]
    assert IM.fall_times(dict(t=t, flags=np.zeros(len(t), int)), []) == []


def _tag_rows(spec):
    """spec: [(t0, t1, fn(t) -> (x, y))] at 12 Hz -> tracker-like rows"""
    rows = []
    for a, b, fn in spec:
        for t in np.arange(a, b, 1 / 12):
            x, y = fn(t)
            rows.append(dict(t=1000.0 + float(t), stale=False, x_mm=x, y_mm=y, heading_deg=0.0, roll_deg=0.0, pitch_deg=0.0,
                             margin_min_mm=300.0, margin_min_edge="near"))
    return rows


import numpy as np  # noqa: E402


def test_position_start_end_are_taken_at_rest_not_mid_step():
    sway = lambda c: (lambda t: (c[0] + 80 * np.sin(2 * np.pi * t / 4.0), c[1] + 10 * np.sin(2 * np.pi * t / 8.0)))
    rows = _tag_rows([(0, 4, lambda t: (402.0 + 0.5 * np.sin(t), 518.0)),         # at rest before the first step
                      (4, 30, sway((450, 480))),                                     # stepping: +-80 mm sway
                      (30, 34, lambda t: (494.0, 443.0)),                           # rest at the end
                      ])
    # the end of the run is mid-sway: the last samples alone would be far off
    out = RR.tag_flags(rows, None)
    assert out["position_start"] == pytest.approx([402.0, 518.0], abs=1.0)
    assert out["position_end"] == pytest.approx([494.0, 443.0], abs=1.0)
    assert out["position_method"]["start"]["method"].startswith("median over a stationary window")
    assert out["position_method"]["end"]["window_s"] >= 1.0
    ext = out["position_track_extent"]
    assert ext["x_mm"][0] < 380 and ext["x_mm"][1] > 520 and ext["y_mm"][0] < 445 and ext["y_mm"][1] > 515


def test_position_end_falls_back_and_says_so_when_never_still():
    rows = _tag_rows([(0, 20, lambda t: (450 + 80 * np.sin(2 * np.pi * t / 4.0), 480.0))])
    out = RR.tag_flags(rows, None)
    assert "NO stationary window" in out["position_method"]["end"]["method"] and out["position_end"] is not None
    assert RR.tag_flags([], None)["position_method"] is None and RR.tag_flags([], None)["position_track_extent"] is None


# ---- where.py --plan-dx-mm / --plan-dy-mm ------------------------------------------------------------------------
def test_plan_end_rotates_the_robot_frame_by_the_heading():
    cfg = G.load_config()
    cfg["board"].update(width_mm=900.0, length_mm=1200.0)
    cfg["foot_margin_mm"] = 0.0
    ex, ey, m = W.plan_end(cfg, 450, 600, 0.0, 200, 0)               # heading 0 = +y: forward is +y, left is -x
    assert (round(ex), round(ey)) == (450, 800) and round(m["far"]) == 400 and round(m["near"]) == 800
    ex, ey, _ = W.plan_end(cfg, 450, 600, 0.0, 0, 100)
    assert (round(ex), round(ey)) == (350, 600)
    ex, ey, _ = W.plan_end(cfg, 450, 600, 90.0, 200, 0)              # turned left 90 deg: forward is -x
    assert (round(ex), round(ey)) == (250, 600)
    ex, ey, _ = W.plan_end(cfg, 450, 600, 90.0, 0, 100)              # and its left is -y
    assert (round(ex), round(ey)) == (450, 500)
    ex, ey, _ = W.plan_end(cfg, 450, 600, 180.0, -200, 50)           # facing -y: back is +y, left is +x
    assert (round(ex), round(ey)) == (500, 800)


def test_where_plan_refuses_an_end_point_near_the_edge(tmp_path):
    import json
    import test_vision_track as t
    img = t.render(t.CFG, 10, 3, -4, 300, 900)                         # robot near x 300, y 900 on the 900 x 1200 board
    p = str(tmp_path / "f.png")
    cv2.imwrite(p, img)
    conf = str(tmp_path / "v.json")
    with open(conf, "w") as f:
        json.dump({"board": {"layout": "config"}, "robot": {"tags": t.CFG["robot"]["tags"]}}, f)

    def run(*args):
        return subprocess.run([PY, os.path.join(VIS, "where.py"), "--image", p, "--min-margin", "250", "--config", conf, *args],
                              capture_output=True, text=True)
    ok = run("--plan-dx-mm", "-200")                                     # backing away from the far edge
    assert ok.returncode == 0 and "predicted end margins" in ok.stdout and "OK" in ok.stdout
    bad = run("--plan-dx-mm", "200")                                     # starts with ~300 mm to the far edge, ends within ~100
    assert bad.returncode == 3 and "END TOO CLOSE" in bad.stdout
    js = json.loads(run("--plan-dx-mm", "200", "--json").stdout)
    pl = js["plan"]
    assert pl["ok"] is False and pl["end_margin_min_edge"] == "far" and pl["end_margin_min_mm"] < 250
    assert pl["end_y_mm"] > js["y_mm"] + 150 and "position_uncertainty_mm" in js
    assert "plan" not in json.loads(run("--direction", "back", "--json").stdout)           # no plan flags: unchanged output
    left = run("--plan-dy-mm", "100")                                    # left = toward -x: x ~300 -> ~200 < 250 from the left edge
    assert left.returncode == 3 and "left edge" in left.stdout
    assert run("--plan-dy-mm", "-100").returncode == 0                   # right: x ~400
    assert "+-100 mm: f from" in bad.stdout                              # f from the coplanarity estimate: the uncertainty is printed


# ---- where.py: cached board pose when a board tag is hidden ---------------------------------------------------------
def _board_scene(hide=(), shift_cam=False, seed=0):
    import test_vision_track as t
    import vsynth as synth
    tags = synth.scene_tags(t.CFG, G.board_R_from_euler(10, 3, -4), [300, 900, 170.0])
    tags = [x for x in tags if x[0] not in hide]
    Rcb, Tcb = (synth.look_at([450, -500, 475], [450, 400, 150]) if not shift_cam else synth.look_at([470, -500, 475], [450, 400, 150]))
    return synth.render(tags, t.K, Rcb, Tcb, t.SIZE, board_rect=(900, 1200), seed=seed, blur=1.0, noise=4)


def _board_ids(img, cfg):
    import tagdefs
    return sorted(i for i in tagdefs.detect(img, tagdefs.make_detector()) if str(i) in cfg["board"]["tags"])


def test_where_uses_the_cached_board_pose_when_a_board_tag_is_hidden(tmp_path):
    import test_vision_track as t
    cache = str(tmp_path / "emeet_board_pose.json")
    full = _board_scene()
    ids = _board_ids(full, t.CFG)
    assert len(ids) >= 2
    live = W.where(full, t.CFG, pose_cache=cache)
    assert live["found"] and live["board_pose"] == "live" and os.path.exists(cache)
    # hide all but one board tag: no live calibration possible, the visible tag still sits where it was
    one = _board_scene(hide=tuple(i for i in range(6) if i != ids[0]))
    assert len(_board_ids(one, t.CFG)) == 1
    nocache = W.where(one, t.CFG)
    assert not nocache["found"] and "fewer than 2" in nocache["reason"]
    c = W.where(one, t.CFG, pose_cache=cache)
    assert c["found"] and c["board_pose"] == "cached" and c["board_pose_info"]["tags"] == [ids[0]]
    assert c["board_pose_info"]["max_shift_px"] < 3.0 and c["board_pose_info"]["age_s"] >= 0
    assert abs(c["x_mm"] - live["x_mm"]) < 15 and abs(c["y_mm"] - live["y_mm"]) < 15      # same pose, so the same position
    assert abs(c["heading_deg"] - live["heading_deg"]) < 3
    # a different image size cannot use it
    small = cv2.resize(one, (one.shape[1] // 2, one.shape[0] // 2))
    r = W.where(small, t.CFG, pose_cache=cache)
    assert not r["found"]


def test_where_refuses_the_cached_pose_when_the_camera_moved(tmp_path):
    import test_vision_track as t
    cache = str(tmp_path / "emeet_board_pose.json")
    full = _board_scene()
    ids = _board_ids(full, t.CFG)
    W.where(full, t.CFG, pose_cache=cache)
    keep = tuple(i for i in range(6) if i != ids[0])
    bumped = _board_scene(hide=keep, shift_cam=True)                      # camera moved 20 mm sideways: the tag shifts in the image
    assert _board_ids(bumped, t.CFG) == [ids[0]]
    r = W.where(bumped, t.CFG, pose_cache=cache)
    assert not r["found"] and r["board_pose"] == "refused" and "moved" in r["reason"]
    assert W.where(bumped, t.CFG, pose_cache=cache, pose_tol_px=400.0)["found"]          # the tolerance is the only gate
    none_visible = _board_scene(hide=tuple(range(6)))
    r = W.where(none_visible, t.CFG, pose_cache=cache)
    assert not r["found"] and r["board_pose"] == "refused" and "cannot be checked" in r["reason"]
    assert not W.where(none_visible, t.CFG, pose_cache=str(tmp_path / "missing.json"))["found"]
