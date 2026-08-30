"""SIL twin: the REAL firmware control stack behind the real UDP link.

    .venv/bin/python sim/sil_twin.py --hang --stream-port 8645
    make -C firmware/host tui && firmware/host/build/bimo_tui --host mira

Where udp_agent.py runs the PYTHON policy stack behind the link, this runs
libctrl_sil -- the firmware's own obs assembler, gait clock, MLP and
actuation code, compiled for the host -- so what you drive from the console
is the same control law the robot executes, tick for tick. With --hang the
plant is welded to the world at stand height, feet free: the sim analogue of
the test stand, for eyeballing sim-vs-bench with the SAME console inputs
(2026-08-30: the bench gait resembled nothing; the question is whether the
sim twin, hanging, does the same thing for the same reasons).

Live view: --stream-port serves MJPEG on 0.0.0.0 (http://mira:<port>/) so
the rendered sim is watchable next to the physical robot. --record writes
the same frames to an mp4 on exit (hw_sessions/... is the natural home).
"""
import argparse
import io
import json
import os
import socket
import struct
import sys
import threading
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("JAX_PLATFORMS", "cpu")   # eval_precision imports jax

import numpy as np                                          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in ("mjx", "sil", ""):
    sys.path.insert(0, os.path.join(HERE, p))
sys.path.insert(0, os.path.join(ROOT, "link"))

from protocol import (CMD_PORT, TLM_PORT, LinkState, ProtocolError,  # noqa: E402
                      Supervisor, Telemetry, decode_command,
                      encode_telemetry)

DEFAULT_RUN = "loco_v22fix_e_s128"
HANG_XML = os.path.join(HERE, "bimo_biped_v5body_hang.xml")


# -- MJPEG live view --------------------------------------------------------
class Mjpeg:
    """Latest-frame MJPEG server: one JPEG shared across viewers, ~10 fps.
    LAN-only convenience (0.0.0.0, the mira:<port> convention)."""

    BOUNDARY = b"--siltwinframe"

    def __init__(self, port):
        import http.server
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
                    "multipart/x-mixed-replace; boundary=siltwinframe")
                self.end_headers()
                try:
                    while True:
                        with outer.lock:
                            buf = outer.jpeg
                        if buf:
                            self.wfile.write(outer.BOUNDARY + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(buf)}\r\n\r\n".encode())
                            self.wfile.write(buf + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return

        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        print(f"sil_twin: MJPEG live view on http://0.0.0.0:{port}/")

    def push(self, rgb):
        import imageio.v2 as imageio
        buf = io.BytesIO()
        imageio.imwrite(buf, rgb, format="jpeg")
        with self.lock:
            self.jpeg = buf.getvalue()


def run(args):
    from eval_precision import make_env, sil_setup          # noqa: E402

    run_dir = os.path.join(HERE, "runs", args.run_name)
    with open(os.path.join(run_dir, "config.json")) as f:
        cfg = json.load(f)
    xml = args.xml or (HANG_XML if args.hang else None) \
        or cfg.get("xml_path") or None
    H, lib, info = sil_setup(args.run_name, run_dir)
    env = make_env(cfg, episode_seconds=1e9, nominal=args.nominal,
                   xml=xml if xml else cfg.get("xml_path"))
    act = H.SilActAdapter(lib, env, log=False)
    obs, _ = env.reset(seed=args.seed)
    fw, nc = act.spec.frame_dim, act.spec.num_cmd

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", args.port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    dog = Supervisor(armed=args.boot_armed)
    stream = Mjpeg(args.stream_port) if args.stream_port else None
    frames = []
    peer = None
    was_armed = args.boot_armed
    last_state = None
    t0 = time.monotonic()
    late = 0
    dt = env.control_dt
    print(f"sil_twin: {args.run_name} on :{args.port} (tlm -> :{args.tlm_port})"
          f"  plant={os.path.basename(xml) if xml else 'run default'}"
          f"  spec={info['spec'][:60]}...")

    i = 0
    try:
        while args.duration is None or i * dt < args.duration:
            tick = time.monotonic()
            now_ms = (tick - t0) * 1000.0

            while True:                       # drain; last valid packet wins
                try:
                    buf, addr = rx.recvfrom(64)
                except BlockingIOError:
                    break
                peer = addr[0]
                try:
                    dog.accept(decode_command(buf), now_ms)
                except ProtocolError:
                    pass

            state = dog.state(now_ms)
            if state is not last_state:
                print(f"  [{now_ms / 1000:6.2f}s] link -> {state.value}",
                      flush=True)
                last_state = state

            # Firmware run-entry: fresh history, fresh clock (ctrl_task's
            # bench handover). Mirror it on the arm edge.
            if dog.armed and not was_armed:
                act.begin_episode()
            was_armed = dog.armed

            v, w = dog.command(now_ms)
            env.set_command(v, 0.0, w)
            env.set_torque_enabled(dog.torque_on(now_ms))

            # Driver.step's obs contract: current cmd patched into the frame
            # the firmware stack is fed.
            obs = np.asarray(obs, dtype=np.float64).copy()
            obs[fw - nc:fw] = env._cmd
            a = act(obs)
            obs, _, _, _, step_info = env.step(a)

            if (stream or args.record) and i % 3 == 0:      # ~16 fps
                rgb = env.render()
                if stream:
                    stream.push(rgb)
                if args.record:
                    frames.append(rgb)

            if peer and i % 5 == 0:                          # 10 Hz beacon
                tx.sendto(encode_telemetry(Telemetry(
                    seq_echo=dog.seq_echo(now_ms), state=state,
                    vbat_v=float(env.supply_voltage),
                    up_z=float(step_info.get("up_z", 0.0)),
                    vx_est=float(step_info.get("vx_body", 0.0)),
                    wz_est=float(step_info.get("wz", 0.0)),
                    servo_err=0,
                    loop_late_pct=int(100 * late / max(1, i)),
                )), (peer, args.tlm_port))

            i += 1
            slack = dt - (time.monotonic() - tick)
            if slack > 0:
                time.sleep(slack)
            else:
                late += 1
    except KeyboardInterrupt:
        print("\nsil_twin: stopped")
    finally:
        if args.record and frames:
            import imageio.v2 as imageio
            imageio.mimwrite(args.record, frames, fps=int(round(1 / dt / 3)))
            print(f"sil_twin: wrote {args.record} ({len(frames)} frames)")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-name", default=DEFAULT_RUN)
    p.add_argument("--hang", action="store_true",
                   help="welded-torso plant (test-stand analogue)")
    p.add_argument("--xml", default=None, help="explicit plant override")
    p.add_argument("--nominal", action="store_true", default=True,
                   help="clean plant (no DR/latency): bench-comparable")
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--stream-port", type=int, default=8645,
                   help="MJPEG live view port (0 disables)")
    p.add_argument("--record", default=None, help="write an mp4 on exit")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--boot-armed", action="store_true")
    run(p.parse_args())


if __name__ == "__main__":
    main()
