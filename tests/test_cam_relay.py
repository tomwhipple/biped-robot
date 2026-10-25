"""tools/cam_relay.py with synthetic (lavfi) sources only. The v4l2 and rtsp
command lines are checked as STRINGS and never executed: a test must not open
a camera."""
import json
import os
import re
import shutil
import socket
import sys
import threading
import time
import urllib.request

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import cam_relay as cr  # noqa: E402

needs_ffmpeg = pytest.mark.skipif(shutil.which("ffmpeg") is None,
                                  reason="ffmpeg not installed")


def test_v4l2_asks_for_mjpeg_and_never_raw():
    for p in cr.PRESETS:
        s = cr.Spec(p)
        cmd = " ".join(s.ffmpeg_cmd())
        if s.kind == "v4l2":
            assert "-input_format mjpeg" in cmd
            assert "-c:v copy" in cmd
            assert f"-framerate {s.fps}" in cmd
            assert not re.search(r"yuyv|rawvideo|pix_fmt", cmd, re.I)
    c = cr.Spec("emeet=v4l2:/dev/video0,size=1280x720,fps=10").ffmpeg_cmd()
    assert c[c.index("-i") + 1] == "/dev/video0"
    assert "1280x720" in c


def test_sony_is_rtsp_local_tcp():
    s = cr.Spec([p for p in cr.PRESETS if p.startswith("sony")][0])
    cmd = s.ffmpeg_cmd()
    assert s.kind == "rtsp" and "127.0.0.1:8554/camera" in cmd[cmd.index("-i") + 1]
    assert "tcp" in cmd and "mjpeg" in cmd


@pytest.mark.parametrize("bad", ["x", "a=", "a/b=lavfi:x", "a=usb:1",
                                 "a=lavfi:x,fps=0", "a=lavfi:x,size=ab",
                                 "a=lavfi:x,bogus=1"])
def test_bad_specs(bad):
    with pytest.raises(ValueError):
        cr.Spec(bad)


def test_split_jpegs():
    a, b = b"\xff\xd8AAA\xff\xd9", b"\xff\xd8BB\xff\xd9"
    assert cr.split_jpegs(a + b) == ([a, b], b"")
    assert cr.split_jpegs(a + b[:4]) == ([a], b[:4])
    assert cr.split_jpegs(b"junk" + a) == ([a], b"")


def read_parts(sock, n, timeout=8):
    """n Content-Length-framed parts off a raw HTTP response."""
    sock.settimeout(timeout)
    buf, parts = b"", []
    while len(parts) < n:
        d = sock.recv(65536)
        if not d:
            break
        buf += d
        while True:
            m = re.search(rb"Content-Length: (\d+)\r\n\r\n", buf)
            if not m or len(buf) < m.end() + int(m[1]):
                break
            parts.append(buf[m.end():m.end() + int(m[1])])
            buf = buf[m.end() + int(m[1]):]
    return parts


@pytest.fixture
def relay():
    srcs = [cr.Source(cr.Spec("t=lavfi:testsrc,size=160x120,fps=15"), 1.0),
            cr.Source(cr.Spec("u=lavfi:smptebars,size=160x120,fps=15"), 1.0)]
    srv = cr.make_server(srcs, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    stop = threading.Event()

    def reaper():
        while not stop.wait(0.2):
            for s in srcs:
                s.tick()
    threading.Thread(target=reaper, daemon=True).start()
    yield srv.server_address[1], srcs
    stop.set()
    for s in srcs:
        s.stop()
    srv.shutdown()


def get(port, path):
    s = socket.create_connection(("127.0.0.1", port))
    s.sendall(f"GET {path} HTTP/1.0\r\n\r\n".encode())
    return s


@needs_ffmpeg
def test_streams_jpegs_on_demand_and_idles_out(relay):
    port, (t, u) = relay
    assert not t.proc and not u.proc                     # nothing until watched
    s = get(port, "/t")
    parts = read_parts(s, 4)
    assert len(parts) >= 4
    assert all(p[:2] == b"\xff\xd8" and p[-2:] == b"\xff\xd9" for p in parts)
    assert t.proc is not None and u.proc is None         # only the watched one
    st = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/"))
    assert st["t"]["running"] and st["t"]["clients"] == 1 and not st["u"]["running"]
    s.close()
    deadline = time.time() + 8
    while t.proc is not None and time.time() < deadline:
        time.sleep(0.1)
    assert t.proc is None                                # released after --idle


@needs_ffmpeg
def test_single_frame_and_404(relay):
    port, _ = relay
    jpg = urllib.request.urlopen(f"http://127.0.0.1:{port}/u.jpg", timeout=10).read()
    assert jpg[:2] == b"\xff\xd8"
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(f"http://127.0.0.1:{port}/nope")
    assert e.value.code == 404


@needs_ffmpeg
def test_dead_source_goes_silent_not_frozen(relay):
    """A source that stops producing must send NOTHING -- the console's STALE
    depends on it -- and must be restarted while someone is watching."""
    port, (t, _) = relay
    s = get(port, "/t")
    assert len(read_parts(s, 3)) >= 3
    seq0 = t.seq
    t.proc.send_signal(19)                                # SIGSTOP: wedged, not dead
    time.sleep(0.6)
    seq1 = t.seq
    time.sleep(1.5)
    assert t.seq == seq1                                  # no frames invented
    t.proc.send_signal(18)                                # SIGCONT
    assert len(read_parts(s, 2)) >= 2
    # now kill it outright: the relay restarts it for the viewer still there
    starts = t.starts
    t.proc.kill()
    deadline = time.time() + 8
    while t.starts == starts and time.time() < deadline:
        time.sleep(0.1)
    assert t.starts == starts + 1
    assert len(read_parts(s, 2)) >= 2
    assert seq0 > 0
    s.close()


def test_missing_ffmpeg_is_an_error_not_a_crash():
    src = cr.Source(cr.Spec("t=lavfi:testsrc"), 1.0, ffmpeg="/nonexistent/ffmpeg")
    assert src.attach()
    assert "cannot start ffmpeg" in src.status()["error"]
    src.detach()


@needs_ffmpeg
def test_recording_writes_mp4_and_matching_pts(tmp_path):
    src = cr.Source(cr.Spec("t=lavfi:testsrc,size=160x120,fps=15"), 0.5,
                    record_dir=str(tmp_path))
    srv = cr.make_server([src], "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        s = get(srv.server_address[1], "/t")
        assert len(read_parts(s, 20)) >= 20
        s.close()
        deadline = time.time() + 10
        while src.proc is not None and time.time() < deadline:
            src.tick()
            time.sleep(0.2)
        assert src.proc is None
        time.sleep(1.0)                                  # encoder finishes
        files = sorted(os.listdir(tmp_path))
        assert len(files) == 2 and files[0].endswith("_t.mp4") and files[1].endswith("_t.pts")
        lines = open(tmp_path / files[1]).read().split("\n")
        assert lines[0] == "# timestamp format v2"
        ts = [float(x) for x in lines[1:] if x]
        assert len(ts) >= 20 and ts == sorted(ts)
        assert abs(ts[0] / 1000.0 - time.time()) < 60    # epoch milliseconds
        assert os.path.getsize(tmp_path / files[0]) > 0
        if shutil.which("ffprobe"):
            import subprocess
            n = subprocess.run(
                ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
                 "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0",
                 str(tmp_path / files[0])], capture_output=True, text=True).stdout.strip()
            assert abs(int(n) - len(ts)) <= 2            # one pts line per frame
    finally:
        src.stop()
        srv.shutdown()
