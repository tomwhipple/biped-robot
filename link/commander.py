"""Laptop-side sender: streams (vx, wz) intent to the robot over UDP.

Run:  .venv/bin/python link/commander.py --host 192.168.4.1 --source gamepad
      .venv/bin/python link/commander.py --host bimo.local --source script \
          --script square --duration 30

Point --host at the sim twin to drive MuJoCo with the identical bytes the
robot will get:
      .venv/bin/python sim/udp_agent.py --run-name cmd_11v1 --render &
      .venv/bin/python link/commander.py --host 127.0.0.1 --source gamepad

The robot is authoritative about safety (link/protocol.py Watchdog); this
process is just a mouth. It can die at any instant and the robot stands.
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sources                                                   # noqa: E402
from protocol import (CMD_PORT, SEND_HZ, TLM_PORT, ProtocolError,  # noqa: E402
                      decode_telemetry, encode_command)


def stream(host, source, send_hz=SEND_HZ, cmd_port=CMD_PORT,
           tlm_port=TLM_PORT, quiet=False):
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", tlm_port))
    rx.setblocking(False)

    period = 1.0 / send_hz
    seq, t0, tlm = 0, time.monotonic(), None
    print(f"commanding {host}:{cmd_port} from {source.name} "
          f"at {send_hz:g} Hz  (ctrl-C to stop)")
    try:
        while True:
            t = time.monotonic() - t0
            vx, wz, flags = source.poll(t)
            tx.sendto(encode_command(seq, vx, wz, flags), (host, cmd_port))
            seq += 1

            while True:  # drain to the freshest telemetry, never queue up
                try:
                    buf, _ = rx.recvfrom(64)
                except BlockingIOError:
                    break
                try:
                    tlm = decode_telemetry(buf)
                except ProtocolError:
                    pass
            if not quiet and seq % 5 == 0:
                _status(t, vx, wz, flags, tlm)

            time.sleep(max(0.0, period - ((time.monotonic() - t0) - t)))
    except KeyboardInterrupt:
        # Say "stand" on the way out instead of letting the watchdog time it
        # out 250 ms later. Repeated because any single UDP packet may vanish.
        for _ in range(10):
            tx.sendto(encode_command(seq, 0.0, 0.0, 0), (host, cmd_port))
            seq += 1
            time.sleep(0.01)
        print("\nstopped; sent stand command")
    finally:
        source.close()
        tx.close()
        rx.close()


def _status(t, vx, wz, flags, tlm):
    link = "--"
    if tlm is not None:
        link = (f"{tlm.state.value:5s} {tlm.vbat_v:4.1f}V "
                f"up{tlm.up_z:4.2f} vx{tlm.vx_est:+5.2f}")
        if tlm.servo_err:
            link += f" ERR{tlm.servo_err:08b}"
    sys.stdout.write(f"\r{t:6.1f}s  cmd v{vx:+5.2f} w{wz:+5.2f} "
                     f"f{flags:02b}  | robot {link}   ")
    sys.stdout.flush()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--host", required=True,
                   help="robot IP (192.168.4.1 in AP mode) or 127.0.0.1 "
                        "for the sim twin")
    p.add_argument("--source", default="script",
                   choices=["script", "gamepad", "goal"])
    p.add_argument("--script", default="stand",
                   choices=sorted(sources.SCRIPTS))
    p.add_argument("--duration", type=float, default=None,
                   help="stop commanding after N s (script source)")
    p.add_argument("--gamepad-index", type=int, default=0)
    p.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"))
    p.add_argument("--pose-port", type=int, default=4212,
                   help="sim-only ground-truth pose feed (goal source)")
    p.add_argument("--rate", type=float, default=SEND_HZ)
    p.add_argument("--cmd-port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()
    stream(args.host, sources.build(args), send_hz=args.rate,
           cmd_port=args.cmd_port, tlm_port=args.tlm_port, quiet=args.quiet)


if __name__ == "__main__":
    main()
