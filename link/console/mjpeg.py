"""MJPEG over HTTP (multipart/x-mixed-replace), read on a thread of its own.

The house servers -- sim/sil_twin.py (:8645), tools/cam_relay.py (:8647),
tools/cam_live.py (:8646) -- send Content-Length on every part; like
link/mjpeg.cpp, a header block without one (the HTTP response's own) is
skipped rather than guessed at. Only the newest frame is kept, decoded to a
QImage on the reader thread (QImage, unlike QPixmap, may be built off the
GUI thread). A stalled stream must look stalled: state() says STALE once no
frame has arrived for STALE_S.
"""
from __future__ import annotations

import socket
import threading
import time
import urllib.parse

from PyQt6 import QtGui

STALE_S = 2.0          # bimo_gui kStreamStaleMs
MAX_FRAME = 8 << 20
RETRY_S = 1.0


class Parser:
    """Incremental part splitter; feed bytes, take complete parts."""

    def __init__(self):
        self.buf = b""
        self.want = None
        self.parts: list[bytes] = []
        self.overflow = False

    def feed(self, data: bytes) -> None:
        self.buf += data
        while True:
            if self.want is None:
                end = self.buf.find(b"\r\n\r\n")
                if end < 0:
                    if len(self.buf) > 65536:
                        self.buf = b""
                        self.overflow = True
                    return
                head = self.buf[:end].decode("latin-1", "replace")
                self.buf = self.buf[end + 4:]
                n = None
                for line in head.split("\r\n"):
                    k, _, v = line.partition(":")
                    if k.strip().lower() == "content-length":
                        try:
                            n = int(v.strip())
                        except ValueError:
                            n = None
                if n is not None and 0 < n <= MAX_FRAME:
                    self.want = n
                elif n is not None:
                    self.overflow = True
            else:
                if len(self.buf) < self.want:
                    return
                self.parts.append(self.buf[: self.want])
                self.buf = self.buf[self.want:]
                self.want = None
                del self.parts[:-1]          # latest frame wins

    def take(self) -> bytes | None:
        return self.parts.pop() if self.parts else None


class MjpegStream:
    def __init__(self, url: str, name: str = ""):
        self.url = url
        self.name = name or url
        self.frames = 0
        self.last_t = 0.0                   # time.monotonic() of the last frame
        self.error = ""
        self._img = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sock = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"mjpeg {self.name}")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        s = self._sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def latest(self):
        """(QImage or None, frame sequence number)."""
        with self._lock:
            return self._img, self._seq

    def state(self, now: float | None = None) -> str:
        if self._thread is None:
            return "off"
        if self.frames == 0:
            return "waiting"
        now = time.monotonic() if now is None else now
        return "STALE" if now - self.last_t > STALE_S else "live"

    def age_s(self, now: float | None = None) -> float | None:
        if not self.frames:
            return None
        return (time.monotonic() if now is None else now) - self.last_t

    def _run(self) -> None:
        u = urllib.parse.urlsplit(self.url)
        host, port = u.hostname or "127.0.0.1", u.port or 80
        path = (u.path or "/") + (f"?{u.query}" if u.query else "")
        while not self._stop.is_set():
            try:
                s = socket.create_connection((host, port), timeout=3.0)
                self._sock = s
                s.sendall(f"GET {path} HTTP/1.0\r\nHost: {host}\r\n\r\n".encode())
                p = Parser()
                s.settimeout(STALE_S + 1.0)
                while not self._stop.is_set():
                    data = s.recv(65536)
                    if not data:
                        raise ConnectionError("server closed the stream")
                    p.feed(data)
                    jpeg = p.take()
                    if jpeg is None:
                        continue
                    img = QtGui.QImage.fromData(jpeg, "JPEG")
                    if img.isNull():
                        self.error = "a part was not a JPEG"
                        continue
                    with self._lock:
                        self._img = img
                        self._seq += 1
                    self.frames += 1
                    self.last_t = time.monotonic()
                    self.error = ""
            except (OSError, ConnectionError) as e:
                self.error = f"{self.url}: {e}"
            finally:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None
            self._stop.wait(RETRY_S)
