#!/usr/bin/env python3
"""MJPEG relay for the bench console's camera panels.

    tools/cam_relay.py                      # the mira presets, on Tailscale :8647
    tools/cam_relay.py --source cam=v4l2:/dev/video0,size=640x480,fps=10
    tools/cam_relay.py --source test=lavfi:testsrc  # a synthetic source

Serves each named source in the house MJPEG convention -- multipart/x-mixed-
replace, Content-Length on every part, newest frame wins -- which is what
link/mjpeg.h consumes (sim/sil_twin.py:8645 and tools/cam_live.py:8646 are the
other two servers). One process, one port, one URL per source:

    http://HOST:8647/<name>        the stream
    http://HOST:8647/<name>.jpg    the newest frame, once (a quick look)
    http://HOST:8647/              JSON: each source, running or not, why

ON DEMAND. A source's ffmpeg is started by its first viewer and stopped
`--idle` seconds (10) after its last one leaves. Nothing holds a camera nobody
is watching: a v4l2 device serves one reader, and the Sony's live view is held
by the mediamtx `camera` path only while somebody reads it (so while a viewer
is connected here, tools/sony_record.sh and ~/code/camera/liveview.py cannot
have the camera).

SOURCE KINDS
  v4l2:/dev/videoN   a webcam, asked for its own MJPEG (-input_format mjpeg)
                     and passed through UNDECODED (-c:v copy). Never raw YUYV:
                     two raw streams on the shared USB2 controller wedged the
                     hub. The camera is asked for `fps` (default 10) frames a
                     second at `size`, so the bus carries no more than that.
  rtsp://...         an H.264 stream (the mediamtx Sony path), decoded and
                     re-encoded to MJPEG at `fps` / `size`.
  lavfi:<graph>      an ffmpeg test pattern. For tests; opens no hardware.

RECORDING (--record-dir DIR). While a source runs, every frame it delivers is
also written to DIR/<UTC stamp>_<name>.mp4 with DIR/<UTC stamp>_<name>.pts (one
line per frame, milliseconds since the epoch on mira's NTP-disciplined clock:
mkvtimestamp_v2, as tools/sony_record.sh writes), so what a console displayed
can be replayed against the telemetry. The recording follows the viewers (it
exists only while someone watches), never blocks the stream, and a frame that
the encoder cannot keep up with is dropped from the file and counted. The
console's session JSONL (`utc` column, same clock family) and the `cam` rows in
it say which file belongs to which take.

A source that stops delivering sends NOTHING -- it never repeats its last frame
-- so the console's STALE indicator is true (tools/cam_live.py learned this the
hard way: a frozen frame that looks live).
"""
import argparse
import http.server
import json
import os
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

BOUNDARY = "camrelayframe"
SOI, EOI = b"\xff\xd8", b"\xff\xd9"
MAX_CLIENTS = 4
MAX_FRAME = 8 << 20

# The cameras on mira (see docs/bench-console.md).
PRESETS = [
    "emeet=v4l2:/dev/video0,size=1280x720,fps=10",
    "brio=v4l2:/dev/video2,size=960x720,fps=10",
    "sony=rtsp://127.0.0.1:8554/camera,size=960x640,fps=10",
]


class Spec:
    def __init__(self, text):
        name, _, rest = text.partition("=")
        if not name or not rest or "/" in name or "." in name:
            raise ValueError(f"bad --source {text!r}: want NAME=KIND:TARGET[,size=WxH][,fps=N]")
        parts = rest.split(",")
        self.name, self.target = name, parts[0]
        self.size, self.fps, self.q = None, 10, 5
        for opt in parts[1:]:
            k, _, v = opt.partition("=")
            if k == "size":
                w, _, h = v.partition("x")
                if not (w.isdigit() and h.isdigit()):
                    raise ValueError(f"bad size {v!r}")
                self.size = v
            elif k == "fps":
                self.fps = int(v)
            elif k == "q":
                self.q = int(v)
            else:
                raise ValueError(f"unknown option {k!r}")
        if not 1 <= self.fps <= 30:
            raise ValueError("fps must be 1..30")
        if self.target.startswith("v4l2:"):
            self.kind, self.arg = "v4l2", self.target[5:]
        elif self.target.startswith("rtsp://"):
            self.kind, self.arg = "rtsp", self.target
        elif self.target.startswith("lavfi:"):
            self.kind, self.arg = "lavfi", self.target[6:]
        else:
            raise ValueError(f"unknown source kind in {self.target!r}")

    def ffmpeg_cmd(self, ffmpeg="ffmpeg"):
        base = [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error"]
        out = ["-f", "image2pipe", "-"]
        if self.kind == "v4l2":
            cmd = base + ["-f", "v4l2", "-input_format", "mjpeg",
                          "-framerate", str(self.fps)]
            if self.size:
                cmd += ["-video_size", self.size]
            # mjpeg2jpeg: UVC frames often omit the Huffman tables; this adds
            # them so every part is a standalone JPEG.
            return cmd + ["-i", self.arg, "-c:v", "copy",
                          "-bsf:v", "mjpeg2jpeg"] + out
        vf = f"fps={self.fps}"
        if self.size:
            vf += "," + "scale=" + self.size.replace("x", ":")
        if self.kind == "rtsp":
            src = ["-rtsp_transport", "tcp", "-i", self.arg]
        else:
            g = self.arg
            if "rate=" not in g:
                g += (":" if "=" in g else "=") + f"rate={self.fps}"
            if self.size and "size=" not in g:
                g += f":size={self.size}"
            src = ["-re", "-f", "lavfi", "-i", g]
            vf = f"fps={self.fps}"
        return base + src + ["-an", "-vf", vf, "-c:v", "mjpeg",
                             "-q:v", str(self.q), "-bsf:v", "mjpeg2jpeg"] + out


def split_jpegs(buf):
    """(complete frames, remainder) from a concatenation of JPEGs."""
    frames = []
    while True:
        s = buf.find(SOI)
        if s < 0:
            return frames, b""
        e = buf.find(EOI, s + 2)
        if e < 0:
            return frames, buf[s:]
        frames.append(buf[s:e + 2])
        buf = buf[e + 2:]


class FrameRecorder:
    """Frames -> mp4 + .pts, off the streaming path (a bounded queue and a
    writer thread; a full queue drops the frame from the FILE only)."""

    def __init__(self, directory, name, ffmpeg="ffmpeg"):
        os.makedirs(directory, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.base = os.path.join(directory, f"{stamp}_{name}")
        self.pts = open(self.base + ".pts", "w", buffering=1)
        self.pts.write("# timestamp format v2\n")
        self.proc = subprocess.Popen(
            [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error", "-y",
             "-use_wallclock_as_timestamps", "1", "-f", "image2pipe",
             "-c:v", "mjpeg", "-i", "-", "-fps_mode", "passthrough", "-c:v", "libx264", "-preset",
             "veryfast", "-pix_fmt", "yuv420p", "-movflags",
             "+frag_keyframe+empty_moov", self.base + ".mp4"],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        self.q = queue.Queue(maxsize=64)
        self.frames = self.dropped = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def add(self, jpeg, t):
        try:
            self.q.put_nowait((jpeg, t))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            jpeg, t = item
            try:
                self.proc.stdin.write(jpeg)
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError):
                continue                      # encoder gone; keep draining
            self.pts.write(f"{t * 1000.0:.3f}\n")
            self.frames += 1

    def close(self):
        self.q.put(None)
        self.thread.join(10)
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(15)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
        self.pts.close()
        log(f"recorded {self.frames} frames ({self.dropped} dropped) -> {self.base}.mp4")


class Source:
    def __init__(self, spec, idle_s, ffmpeg="ffmpeg", record_dir=None):
        self.spec, self.idle_s, self.ffmpeg = spec, idle_s, ffmpeg
        self.record_dir = record_dir
        self.rec = None
        self.cv = threading.Condition()
        self.clients = 0
        self.proc = None
        self.jpeg, self.seq, self.t_frame = None, 0, 0.0
        self.error = ""
        self.starts = 0
        self.idle_since = None
        self.stopping = False

    # -- called by HTTP handlers
    def attach(self):
        with self.cv:
            if self.clients >= MAX_CLIENTS:
                return False
            self.clients += 1
            self.idle_since = None
            if self.proc is None:
                self._start()
            return True

    def detach(self):
        with self.cv:
            self.clients -= 1
            if self.clients == 0:
                self.idle_since = time.time()

    def _start(self):
        self.error = ""
        self.jpeg = None
        self.stopping = False
        try:
            self.proc = subprocess.Popen(
                self.spec.ffmpeg_cmd(self.ffmpeg), stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, stdin=subprocess.DEVNULL)
        except OSError as e:
            self.proc, self.error = None, f"cannot start ffmpeg: {e}"
            return
        self.starts += 1
        p = self.proc
        threading.Thread(target=self._pump, args=(p,), daemon=True).start()
        threading.Thread(target=self._errs, args=(p,), daemon=True).start()
        log(f"{self.spec.name}: started ffmpeg pid {p.pid}")
        if self.record_dir:
            try:
                self.rec = FrameRecorder(self.record_dir, self.spec.name,
                                         self.ffmpeg)
            except OSError as e:
                self.rec = None
                log(f"{self.spec.name}: cannot record: {e}")

    def _errs(self, p):
        last = b""
        for line in p.stderr:
            if line.strip():
                last = line.strip()
        if last and p is self.proc:
            self.error = last.decode(errors="replace")[-200:]

    def _pump(self, p):
        buf = b""
        while True:
            chunk = p.stdout.read1(65536)
            if not chunk:
                break
            buf += chunk
            frames, buf = split_jpegs(buf)
            if len(buf) > MAX_FRAME:
                buf = b""
            if frames:
                rec = self.rec
                if rec is not None:
                    now = time.time()
                    for f in frames:
                        rec.add(f, now)
                with self.cv:
                    self.jpeg = frames[-1]        # latest wins
                    self.seq += 1
                    self.t_frame = time.time()
                    self.cv.notify_all()
        p.wait()
        rec, self.rec = self.rec, None
        if rec is not None:
            rec.close()
        with self.cv:
            if p is self.proc:
                self.proc = None
                if not self.stopping:
                    self.error = self.error or f"ffmpeg exited ({p.returncode})"
                    log(f"{self.spec.name}: ffmpeg died: {self.error}")
            self.cv.notify_all()

    def stop(self):
        with self.cv:
            p, self.proc, self.stopping = self.proc, None, True
        if p is not None:
            p.terminate()                         # SIGTERM; never -9 first
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
            log(f"{self.spec.name}: stopped")

    def tick(self):
        """Idle shutdown, and the restart of a source that died while
        somebody is still watching."""
        with self.cv:
            idle = self.clients == 0 and self.idle_since is not None and \
                time.time() - self.idle_since >= self.idle_s and self.proc
            dead = self.clients > 0 and self.proc is None
        if idle:
            self.stop()
        elif dead:
            with self.cv:
                if self.proc is None and self.clients > 0:
                    self._start()

    def wait_frame(self, last_seq, timeout):
        with self.cv:
            self.cv.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.jpeg

    def status(self):
        with self.cv:
            return {"running": self.proc is not None, "clients": self.clients,
                    "frames": self.seq, "starts": self.starts,
                    "age_s": None if not self.t_frame
                    else round(time.time() - self.t_frame, 2),
                    "error": self.error, "target": self.spec.target,
                    "size": self.spec.size, "fps": self.spec.fps}


def log(msg):
    print(f"cam_relay: {msg}", flush=True)


def make_server(sources, bind, port):
    by_name = {s.spec.name: s for s in sources}

    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split("?")[0].strip("/")
            if path == "":
                body = json.dumps({n: s.status() for n, s in by_name.items()},
                                  indent=1).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            once = path.endswith(".jpg")
            src = by_name.get(path[:-4] if once else path)
            if src is None:
                self.send_error(404, "no such source")
                return
            if not src.attach():
                self.send_error(503, "too many viewers")
                return
            try:
                if once:
                    with src.cv:
                        src.cv.wait_for(lambda: src.jpeg is not None, 8.0)
                        jpeg = src.jpeg
                    if jpeg is None:
                        self.send_error(504, src.error or "no frame")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    f"multipart/x-mixed-replace; boundary={BOUNDARY}")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seq = 0
                while True:
                    # Wake to notice a vanished client; send ONLY new frames.
                    n, jpeg = src.wait_frame(seq, 1.0)
                    if n == seq or jpeg is None:
                        self.wfile.flush()
                        continue
                    seq = n
                    self.wfile.write(
                        f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            finally:
                src.detach()

    class Srv(http.server.ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    return Srv((bind, port), H)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bind", default="100.105.166.4",
                    help="address to serve on (mira's Tailscale address; the "
                         "LAN is not the audience)")
    ap.add_argument("--port", type=int, default=8647)
    ap.add_argument("--source", action="append", metavar="NAME=KIND:TARGET[,opts]",
                    help="repeatable; default: the mira presets "
                         + " ".join(p.split("=")[0] for p in PRESETS))
    ap.add_argument("--idle", type=float, default=10.0,
                    help="seconds after the last viewer before ffmpeg stops")
    ap.add_argument("--ffmpeg", default="ffmpeg")
    ap.add_argument("--record-dir", metavar="DIR",
                    help="also write each running source to DIR/<stamp>_<name>"
                         ".mp4 + .pts (see RECORDING)")
    a = ap.parse_args(argv)
    try:
        specs = [Spec(t) for t in (a.source or PRESETS)]
    except ValueError as e:
        ap.error(str(e))
    names = [s.name for s in specs]
    if len(set(names)) != len(names):
        ap.error("duplicate source name")
    sources = [Source(s, a.idle, a.ffmpeg, a.record_dir) for s in specs]
    srv = make_server(sources, a.bind, a.port)
    log(f"http://{a.bind}:{srv.server_address[1]}/ : "
        + ", ".join(f"{s.spec.name} ({s.spec.kind}, on demand)" for s in sources))

    def reaper():
        while True:
            time.sleep(0.5)
            for s in sources:
                s.tick()
    threading.Thread(target=reaper, daemon=True).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for s in sources:
            s.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
