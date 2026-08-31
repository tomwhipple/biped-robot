"""Live side-by-side MJPEG view of the bench webcams.

    .venv/bin/python tools/cam_live.py            # http://mira:8646/
    .venv/bin/python tools/cam_live.py --devices /dev/video0 /dev/video2

Same LAN-only MJPEG pattern as sim/sil_twin.py's live view. Left pane =
first device (side, video0), right pane = second (rear, video2).

NOTE a v4l2 device only serves one reader: stop this before recording an
attempt with ffmpeg (or the recording fails with 'Device busy').
"""
import argparse
import io
import threading
import time

import http.server
import imageio.v2 as imageio
import numpy as np

BOUNDARY = b"--camliveframe"


class Stream:
    def __init__(self, port):
        self.jpeg = None
        self.lock = threading.Lock()
        outer = self

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=camliveframe")
                self.end_headers()
                try:
                    while True:
                        with outer.lock:
                            buf = outer.jpeg
                        if buf:
                            self.wfile.write(BOUNDARY + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(buf)}\r\n\r\n".encode())
                            self.wfile.write(buf + b"\r\n")
                        time.sleep(0.15)
                except (BrokenPipeError, ConnectionResetError):
                    return

        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def push(self, rgb):
        buf = io.BytesIO()
        imageio.imwrite(buf, rgb, format="jpeg")
        with self.lock:
            self.jpeg = buf.getvalue()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--devices", nargs="+",
                   default=["/dev/video0", "/dev/video2"])
    p.add_argument("--port", type=int, default=8646)
    args = p.parse_args()

    # Pin the capture mode: v4l2 devices remember the last format another
    # tool set (a 1080p still capture left one camera wide, and the naive
    # height crop then showed only the top strip of that pane).
    readers = [imageio.get_reader(f"<video{d[-1]}>", size=(640, 480))
               for d in args.devices]
    stream = Stream(args.port)
    print(f"cam_live: {' + '.join(args.devices)} -> http://0.0.0.0:{args.port}/",
          flush=True)

    # One drain thread per camera, keeping only the newest frame: the compose
    # loop is slower than the 30 fps capture, and reading in-line lets a pipe
    # backlog grow until the "live" view runs minutes behind (bitten
    # 2026-08-31 -- judged a camera aim from stale frames).
    latest = [None] * len(readers)

    def drain(i, r):
        while True:
            latest[i] = np.asarray(r.get_next_data())

    for i, r in enumerate(readers):
        threading.Thread(target=drain, args=(i, r), daemon=True).start()
    try:
        while True:
            frames = [f for f in latest]
            if any(f is None for f in frames):
                time.sleep(0.1)
                continue
            h = min(f.shape[0] for f in frames)
            stream.push(np.hstack([f[:h] for f in frames]))
            time.sleep(0.12)
    finally:
        for r in readers:
            r.close()


if __name__ == "__main__":
    main()
