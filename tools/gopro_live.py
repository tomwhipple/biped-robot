"""Live preview off a GoPro MAX over its own WiFi AP (legacy gpControl API).

    .venv/bin/python tools/gopro_live.py --probe            # is the camera there?
    .venv/bin/python tools/gopro_live.py --snapshot /tmp/gopro.jpg
    .venv/bin/python tools/gopro_live.py --record 10 --out /tmp/gopro.mp4
    .venv/bin/python tools/gopro_live.py --serve 8647       # http://mira:8647/

The MAX (2019) predates Open GoPro and USB webcam mode -- see
docs/gopro-vision-input.md. The ONLY live path is the reverse-engineered
gpControl API on the camera's own access point:

  1. camera on, Preferences > Connections > Connect Device > GoPro App
     (this is what brings the AP up; there is no remote way to do it --
     the MAX has no BLE control API)
  2. this host joins that AP; the camera is always 10.5.5.9, we get 10.5.5.100
  3. GET /gp/gpControl/execute?p1=gpStream&c1=restart starts the preview
  4. MPEG-TS/H.264 arrives on UDP 8554 -- ~432x240, 1-4 s of latency
  5. the stream dies in ~10 s without a `_GPHD_` keep-alive every 2.5 s

Only one preview consumer at a time: close the phone app first.
"""
import argparse
import socket
import subprocess
import sys
import threading
import time
import urllib.request

CAM = "10.5.5.9"      # fixed by the camera's AP; --host overrides for testing
CTRL_PORT = 80
STREAM_PORT = 8554
# Keep-alive normally goes to the camera on the same port number we receive on
# -- no conflict, because the camera is a different host. Only a loopback stub
# needs this split (otherwise the keep-alive lands in our own ffmpeg socket).
KA_PORT = STREAM_PORT
KEEP_ALIVE = b"_GPHD_:0:0:2:0.000000\n"
KEEP_ALIVE_PERIOD = 2.5
# ffmpeg's own buffering is most of the latency. Measured against a synthetic
# 432x240 MPEG-TS feed: nobuffer + low_delay alone lock on cleanly when joining
# mid-stream. Adding -probesize 32768 -analyzeduration 0 on top does NOT --
# ffmpeg then misses the SPS/PPS and floods "non-existing PPS 0 referenced"
# until the next keyframe. Do not "optimise" those back in.
LOW_DELAY = ["-fflags", "nobuffer", "-flags", "low_delay"]
# We always join an already-running stream, so the first usable frame is the
# next IDR. Bound the wait rather than blocking forever on a dead camera.
WATCHDOG_S = 20.0
# Until that first IDR the decoder has no SPS/PPS and says so once per slice.
# It is noise, not a fault -- count it instead of spraying hundreds of lines.
LOCK_ON_NOISE = ("non-existing PPS", "decode_slice_header error", "no frame!",
                 "Last message repeated", "Packet corrupt",
                 "deprecated pixel format")
# The preview is nominally 30 fps; --fps overrides if a MAX measures otherwise.
PREVIEW_FPS = 30.0


def gp(path, timeout=5):
    """GET one gpControl URL. Returns the body, or raises."""
    url = f"http://{CAM}:{CTRL_PORT}{path}"
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def probe():
    """Report whether the camera is reachable and what it is."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    try:
        s.connect((CAM, CTRL_PORT))
    except OSError as e:
        print(f"NOT REACHABLE: {CAM}:{CTRL_PORT} -- {e}")
        print("  is this host joined to the camera's AP? check: ip -br -4 addr")
        return False
    finally:
        s.close()
    try:
        body = gp("/gp/gpControl").decode("utf-8", "replace")
    except Exception as e:
        print(f"{CAM}:{CTRL_PORT} open but /gp/gpControl failed: {e}")
        return False
    import json
    info = json.loads(body).get("info", {})
    for k in ("model_name", "firmware_version", "serial_number", "ap_ssid",
              "ap_mac"):
        if k in info:
            print(f"  {k:18s} {info[k]}")
    return True


def wake(mac):
    """Wake-on-LAN magic packet -- the only remote power-on the MAX has."""
    raw = bytes.fromhex(mac.replace(":", "").replace("-", ""))
    pkt = b"\xff" * 6 + raw * 16
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for dest in ((CAM, 9), ("255.255.255.255", 9)):
        s.sendto(pkt, dest)
    s.close()
    print(f"magic packet sent to {mac}; camera takes ~5 s to bring the AP up")


class KeepAlive(threading.Thread):
    """The stream stops in ~10 s without this. Ephemeral source port on
    purpose -- the camera replies to our IP on fixed port 8554, so ffmpeg
    keeps sole ownership of that bind."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                self.sock.sendto(KEEP_ALIVE, (CAM, KA_PORT))
            except OSError as e:
                print(f"keep-alive failed: {e}", file=sys.stderr)
            self.stop.wait(KEEP_ALIVE_PERIOD)


def start_stream():
    gp("/gp/gpControl/execute?p1=gpStream&c1=restart")
    ka = KeepAlive()
    ka.start()
    return ka


def run_ffmpeg(args, label, watchdog=None):
    """Run ffmpeg on the preview socket. watchdog=None means run until the
    user stops it (serve/play); a number bounds a one-shot capture."""
    url = f"udp://@0.0.0.0:{STREAM_PORT}?fifo_size=500000&overrun_nonfatal=1"
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning",
           *LOW_DELAY, "-i", url, *args]
    print(f"{label}: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True)
    noise = [0]

    def pump():
        for line in proc.stderr:
            if any(n in line for n in LOCK_ON_NOISE):
                noise[0] += 1
            else:
                sys.stderr.write(line)

    t = threading.Thread(target=pump, daemon=True)
    t.start()
    try:
        rc = proc.wait(timeout=watchdog)
        if noise[0]:
            print(f"{label}: {noise[0]} decoder lock-on lines suppressed "
                  f"(mid-GOP join, expected)", file=sys.stderr)
        return rc
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        print(f"{label}: no frame within {watchdog:.0f}s -- is the preview "
              f"running? (phone app holds it exclusively)", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        proc.terminate()
        proc.wait()
        return 0


def main():
    global CAM, CTRL_PORT, KA_PORT
    p = argparse.ArgumentParser()
    p.add_argument("--probe", action="store_true",
                   help="check reachability and print camera identity")
    p.add_argument("--wake", metavar="MAC",
                   help="send a wake-on-LAN packet to this camera MAC")
    p.add_argument("--snapshot", metavar="JPG",
                   help="grab one frame and exit -- the cheapest proof of life")
    p.add_argument("--record", type=float, metavar="SEC",
                   help="record SEC seconds to --out (.ts stream-copies, "
                        "any other extension re-encodes)")
    p.add_argument("--out", default="/tmp/gopro.mp4")
    p.add_argument("--serve", type=int, metavar="PORT",
                   help="re-serve as LAN MJPEG (same pattern as cam_live.py)")
    p.add_argument("--play", action="store_true", help="ffplay window")
    p.add_argument("--host", default=CAM,
                   help="camera address (default %(default)s)")
    p.add_argument("--port", type=int, default=CTRL_PORT,
                   help="gpControl HTTP port (default %(default)s)")
    p.add_argument("--ka-port", type=int, default=None, metavar="P",
                   help="loopback-testing hook: send keep-alive to port P")
    p.add_argument("--fps", type=float, default=PREVIEW_FPS, metavar="F",
                   help=f"preview frame rate, for --record (default {PREVIEW_FPS:.0f})")
    p.add_argument("--wait", type=float, default=WATCHDOG_S, metavar="SEC",
                   help=f"give up if no frame arrives (default {WATCHDOG_S:.0f}s)")
    a = p.parse_args()
    CAM, CTRL_PORT = a.host, a.port
    KA_PORT = a.ka_port if a.ka_port else STREAM_PORT

    if a.wake:
        wake(a.wake)
        time.sleep(5)
    if not probe():
        return 1
    if a.probe:
        return 0

    ka = start_stream()
    time.sleep(1.0)
    try:
        if a.snapshot:
            rc = run_ffmpeg(["-frames:v", "1", "-y", a.snapshot], "snapshot",
                            watchdog=a.wait)
        elif a.record:
            # Duration is counted in FRAMES, not with -t. We join an
            # already-running stream whose MPEG-TS timestamps start far from
            # zero, so `-t SEC` is satisfied immediately and writes an empty
            # file (measured: 262-byte mp4, 0.000000 duration). -frames:v is
            # timestamp-independent; setpts rebases so the duration reads true.
            n = max(1, int(round(a.record * a.fps)))
            if a.out.endswith(".ts"):
                # TS tolerates a mid-GOP start, so stream-copy is safe here.
                enc = ["-c", "copy"]
            else:
                # MP4 cannot: with no leading SPS/PPS the muxer builds no
                # extradata and the file is unplayable. Re-encoding is cheap
                # at 432x240 and always works.
                enc = ["-vf", "setpts=PTS-STARTPTS", "-c:v", "libx264",
                       "-preset", "veryfast", "-pix_fmt", "yuv420p", "-an"]
            rc = run_ffmpeg(["-frames:v", str(n), *enc, "-y", a.out],
                            "record", watchdog=a.record + a.wait)
        elif a.serve:
            rc = run_ffmpeg(
                ["-c:v", "mjpeg", "-q:v", "6", "-an", "-f", "mpjpeg",
                 f"http://0.0.0.0:{a.serve}?listen=1"], "serve")
        elif a.play:
            cmd = ["ffplay", "-hide_banner", *LOW_DELAY,
                   f"udp://@0.0.0.0:{STREAM_PORT}"]
            print(" ".join(cmd))
            rc = subprocess.call(cmd)
        else:
            rc = run_ffmpeg(["-frames:v", "1", "-y", "/tmp/gopro.jpg"],
                            "snapshot (default)", watchdog=a.wait)
    finally:
        ka.stop.set()
    return rc


if __name__ == "__main__":
    sys.exit(main())
