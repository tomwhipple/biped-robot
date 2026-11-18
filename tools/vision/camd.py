#!/usr/bin/env python3
"""Camera daemon: ONE process owns a camera and records rolling wall-clock segments.

    tools/vision/camd.py --source sony  [--dir DIR] [--segment-s 60] [--keep-hours 6]
    tools/vision/camd.py --source emeet [--size 1920x1080 --fps 15]    # /dev/video0, the measurement
                                                                       # camera; own lock, same dir is fine
    tools/vision/camd.py --source lavfi --dir /tmp/x         # synthetic, for tests

Writes DIR/<name>_<UTC start>.mp4 (H.264, "<name> <UTC>" burned in, as
tools/sony_record.sh) + DIR/<name>_<UTC start>.pts (mkvtimestamp_v2: epoch ms
per frame), a new pair every --segment-s seconds on wall-clock boundaries, and
prunes segments older than --keep-hours. DIR/<name>_latest.jpg is the newest
frame (5 per second, written whole and renamed) so a reader such as
tools/vision/where.py can look WITHOUT opening the camera, and
DIR/<name>.status.json is a heartbeat (frames, fps, last frame).

Single owner: an exclusive flock on DIR/<name>.lock held for the life of the
process; a second instance refuses (exit 3). tools/sony_record.sh checks the
same lock. (A flock cannot match its own command line the way pgrep -f can.)
The Sony serves ONE reader: while camd runs, nothing else may read it.
If the source delivers no frame for --stall-s seconds camd exits 4 (let the
supervisor restart it); SIGTERM/SIGINT stop it cleanly (Sony: SIGTERM only,
never kill -9, see tools/sony_record.sh).

Start (not started for you):
    setsid nohup tools/vision/camd.py --source sony >> ~/cam-segments/camd.log 2>&1 &
"""
import argparse
import datetime as dt
import fcntl
import glob
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

DEFAULT_DIR = os.path.expanduser(os.environ.get("CAMD_DIR", "~/cam-segments"))
LIVEVIEW = os.path.expanduser(os.environ.get("LIVEVIEW", "~/code/camera/liveview.py"))
SOI, EOI = b"\xff\xd8", b"\xff\xd9"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
STAMP_RE = re.compile(r"_(\d{8}T\d{6})Z\.(mp4|pts)$")


def source_cmd(name, size="1920x1080", fps=15):
    """argv of a process that writes concatenated JPEG frames on stdout."""
    if name == "sony":
        # the USB node is group plugdev, which long-lived shells may predate: sg
        return ["sg", "plugdev", "-c", f"exec /usr/bin/python3 -u '{LIVEVIEW}'"]
    if name == "emeet":
        # MJPEG passed through undecoded: never raw YUYV (see tools/cam_relay.py)
        return ["ffmpeg", "-loglevel", "error", "-f", "v4l2", "-input_format", "mjpeg",
                "-video_size", size, "-framerate", str(fps), "-i", "/dev/video0",
                "-c:v", "copy", "-f", "mjpeg", "-"]
    if name == "lavfi":
        return ["ffmpeg", "-loglevel", "error", "-re", "-f", "lavfi", "-i", "testsrc=size=320x240:rate=10",
                "-c:v", "mjpeg", "-f", "mjpeg", "-"]
    raise SystemExit(f"unknown source {name!r}")


def segment_cmd(base, label):
    """ffmpeg: JPEGs on stdin -> base.mp4 + base.pts (wall clock at read time)."""
    font = f"fontfile={FONT}" if os.path.exists(FONT) else "font=monospace"
    stamp = r"%{pts\:gmtime\:0\:%Y-%m-%dT%H\\\:%M\\\:%S}.%{eif\:mod(t*1000\,1000)\:d\:3}Z"
    dtxt = f"drawtext={font}:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6:x=8:y=8:text='{label} {stamp}'"
    return ["ffmpeg", "-loglevel", "error", "-y", "-use_wallclock_as_timestamps", "1",
            "-f", "image2pipe", "-c:v", "mjpeg", "-i", "-", "-copyts", "-fps_mode", "passthrough",
            "-filter_complex", f"[0:v]{dtxt},split[a][b];[a]setpts=PTS-STARTPTS[mp4]",
            "-map", "[mp4]", "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p",
            "-movflags", "+frag_keyframe+empty_moov", base + ".mp4",
            "-map", "[b]", "-c:v", "rawvideo", "-enc_time_base", "1/1000000",
            "-f", "mkvtimestamp_v2", base + ".pts"]


def take_lock(path):
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        holder = os.read(fd, 64).decode().strip() or "unknown pid"
        print(f"camd: another instance owns {path} (pid {holder}); refusing to start", file=sys.stderr)
        sys.exit(3)
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    return fd


def seg_stamp(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%Y%m%dT%H%M%S")


def parse_stamp(path):
    m = STAMP_RE.search(path)
    if not m:
        return None
    return dt.datetime.strptime(m.group(1), "%Y%m%dT%H%M%S").replace(tzinfo=dt.timezone.utc).timestamp()


def prune(directory, name, keep_s, now=None):
    now = now or time.time()
    n = 0
    for p in glob.glob(os.path.join(directory, f"{name}_*")):
        t = parse_stamp(p)
        if t is not None and now - t > keep_s:
            os.unlink(p)
            n += 1
    return n


class Writer:
    """The current segment's ffmpeg."""

    def __init__(self, directory, name, t):
        self.base = os.path.join(directory, f"{name}_{seg_stamp(t)}Z")
        self.p = subprocess.Popen(segment_cmd(self.base, name), stdin=subprocess.PIPE)

    def write(self, jpg):
        self.p.stdin.write(jpg)

    def close(self):
        try:
            self.p.stdin.close()
        except OSError:
            pass
        threading.Thread(target=self.p.wait, daemon=True).start()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="sony", choices=["sony", "emeet", "lavfi"])
    ap.add_argument("--name", help="file prefix (default: the source name)")
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--size", default="1920x1080", help="emeet capture size (MJPEG)")
    ap.add_argument("--fps", type=int, default=15, help="emeet frame rate")
    ap.add_argument("--segment-s", type=float, default=60.0)
    ap.add_argument("--keep-hours", type=float, default=6.0)
    ap.add_argument("--stall-s", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (tests)")
    a = ap.parse_args(argv)
    name = a.name or a.source
    os.makedirs(a.dir, exist_ok=True)
    lockfd = take_lock(os.path.join(a.dir, f"{name}.lock"))  # noqa: F841  (held until exit)

    src = subprocess.Popen(source_cmd(a.source, a.size, a.fps), stdout=subprocess.PIPE, start_new_session=True)
    stop = threading.Event()

    def on_signal(*_):
        stop.set()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    def kill_source():
        try:
            os.killpg(src.pid, signal.SIGTERM)   # sg forwards nothing: the whole group
        except ProcessLookupError:
            pass
        try:
            src.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("camd: source ignored SIGTERM for 10 s; leaving it (never kill -9 the Sony)", file=sys.stderr)

    last = {"t": time.time(), "n": 0}

    def watchdog():
        while not stop.wait(1.0):
            if time.time() - last["t"] > a.stall_s:
                print(f"camd: no frame for {a.stall_s:.0f} s (camera busy, unplugged or wedged); exiting",
                      file=sys.stderr)
                last["stalled"] = True
                stop.set()
                kill_source()
    threading.Thread(target=watchdog, daemon=True).start()

    start = time.time()
    buf, writer, seg_idx, n_total = b"", None, None, 0
    t_jpg = t_prune = t_status = 0.0
    keep_s = a.keep_hours * 3600
    rd = src.stdout
    try:
        while not stop.is_set():
            chunk = os.read(rd.fileno(), 1 << 16)
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(SOI)
                if s < 0:
                    buf = b""
                    break
                e = buf.find(EOI, s + 2)
                if e < 0:
                    buf = buf[s:]
                    break
                jpg, buf = buf[s:e + 2], buf[e + 2:]
                now = time.time()
                idx = int(now // a.segment_s)
                if idx != seg_idx:
                    if writer:
                        writer.close()
                    writer, seg_idx = Writer(a.dir, name, now), idx
                try:
                    writer.write(jpg)
                except BrokenPipeError:
                    print("camd: segment ffmpeg died", file=sys.stderr)
                    stop.set()
                    break
                last["t"], n_total = now, n_total + 1
                if now - t_jpg >= 0.2:
                    t_jpg = now
                    tmp = os.path.join(a.dir, f".{name}_latest.tmp")
                    with open(tmp, "wb") as f:
                        f.write(jpg)
                    os.replace(tmp, os.path.join(a.dir, f"{name}_latest.jpg"))
                if now - t_prune >= 60:
                    t_prune = now
                    prune(a.dir, name, keep_s, now)
                if now - t_status >= 2:
                    t_status = now
                    with open(os.path.join(a.dir, f"{name}.status.json"), "w") as f:
                        json.dump(dict(pid=os.getpid(), source=a.source, frames=n_total,
                                       fps=n_total / max(now - start, 1e-6), last_frame_utc=now,
                                       segment=writer.base), f)
                if a.duration and now - start >= a.duration:
                    stop.set()
                    break
    finally:
        stop.set()
        if writer:
            writer.close()
        kill_source()
        time.sleep(0.3)
    return 4 if last.get("stalled") else 0


if __name__ == "__main__":
    sys.exit(main())
