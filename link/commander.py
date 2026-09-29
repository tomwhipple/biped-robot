"""Laptop-side sender: streams (vx, wz) intent to the robot over UDP.

Run:  .venv/bin/python link/commander.py --host 192.168.4.1 --source gamepad
      .venv/bin/python link/commander.py --host bimo.local --source script \
          --script square --duration 30

Point --host at the sim twin to drive MuJoCo with the identical bytes the
robot will get (the real firmware code, compiled for the host):
      .venv/bin/python sim/sil_twin.py &
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
from protocol import (CMD_PORT, SEND_HZ, TLM_PORT, LinkState,  # noqa: E402
                      ProtocolError, decode_telemetry, encode_command)


def stream(host, source, send_hz=SEND_HZ, cmd_port=CMD_PORT,
           tlm_port=TLM_PORT, quiet=False, watch=None):
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", tlm_port))
    rx.setblocking(False)

    # The robot beacons to WHOEVER COMMANDED LAST and nobody else
    # (firmware/main/wifi_link.cpp). A second console therefore cannot simply
    # listen in -- the only way to become the destination is to send a
    # command, which both steals the beacon from this process and drives the
    # robot. So forward instead: every beacon this process accepts is re-sent
    # byte for byte to `bimo_gui --readonly`. Verbatim, so the watcher runs
    # the same decode over the same bytes the robot signed.
    watch_to = _watch_addr(watch, tlm_port) if watch else None
    if watch_to is not None:
        print(f"forwarding telemetry to {watch_to[0]}:{watch_to[1]}")

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
                    continue        # stray traffic: never forward it on
                if watch_to is not None:
                    tx.sendto(buf, watch_to)
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
        if tlm is not None and tlm.state is LinkState.BENCH:
            # Every frame this run was ignored. The robot knows why, and
            # this is the last chance to say it before the process exits.
            print(f"robot was BENCHED the whole time: {tlm.reason}")
    finally:
        source.close()
        tx.close()
        rx.close()


def _watch_addr(spec, default_port):
    """"HOST" or "HOST:PORT" -> (host, port).

    No port means the one this process is bound to, which is where a stock
    `bimo_gui --readonly` listens.
    """
    host, _, port = spec.rpartition(":")
    if not host:                     # no colon at all: rpartition puts it last
        return spec, default_port
    return host, int(port)


def _status(t, vx, wz, flags, tlm):
    link = "--"
    if tlm is not None:
        link = (f"{tlm.state.value:5s} {tlm.vbat_v:4.1f}V "
                f"up{tlm.up_z:4.2f} vx{tlm.vx_est:+5.2f}")
        if tlm.servo_err:
            link += f" ERR{tlm.servo_err:08b}"
        if tlm.state is LinkState.BENCH:
            # A benched robot ignores every command in this stream. Say why
            # here instead of leaving the operator to watch a status line
            # that never changes (docs/control-channel.md, 2026-08-30).
            link += f"  {tlm.reason}"
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
    p.add_argument("--watch", metavar="HOST[:PORT]",
                   help="forward every beacon to a `bimo_gui --readonly` "
                        "watcher; the robot only beacons to whoever commanded "
                        "it last, so a second console cannot listen in")
    args = p.parse_args()
    stream(args.host, sources.build(args), send_hz=args.rate,
           cmd_port=args.cmd_port, tlm_port=args.tlm_port, quiet=args.quiet,
           watch=args.watch)


if __name__ == "__main__":
    main()
