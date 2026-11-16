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
