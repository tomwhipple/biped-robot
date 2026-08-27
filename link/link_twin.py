#!/usr/bin/env python3
"""Plant-free twin of the robot's link end: protocol, arm latch, watchdog.

    .venv/bin/python link/link_twin.py [--port 4210] [--tlm-port 4211]

No MuJoCo, no policy: this answers a client exactly as the firmware's link
does -- BENCH until an ARM edge, LIVE/STAND/RELAX/ESTOP from the reference
Watchdog, 10 Hz telemetry -- and prints every state change and command it
would hand the policy. It exists so a commander (link/tui.cpp above all) can
be exercised end to end on a laptop with nothing else running; the full
sim twin with a plant is sim/udp_agent.py, which shares the Supervisor.

Telemetry is synthetic: vx_est/wz_est echo the applied command, vbat is a
fixed 11.4 V, up_z is 1.0.
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from protocol import (CMD_PORT, TLM_PORT, LinkState, ProtocolError,  # noqa
                      Supervisor, Telemetry, decode_command, encode_telemetry)


def run(port=CMD_PORT, tlm_port=TLM_PORT, armed=False, arm_allowed=True,
        duration=None, quiet=False):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sup = Supervisor(armed=armed, arm_allowed=arm_allowed)
    peer, last_state, last_cmd = None, None, (0.0, 0.0)
    t0 = time.monotonic()
    tick = 0
    print(f"link_twin: listening on :{port}, telemetry -> :{tlm_port}, "
          f"boot {'armed' if armed else 'benched'}", flush=True)
    while duration is None or time.monotonic() - t0 < duration:
        now_ms = (time.monotonic() - t0) * 1000.0
        while True:
            try:
                buf, addr = rx.recvfrom(64)
            except BlockingIOError:
                break
            peer = addr[0]
            try:
                sup.accept(decode_command(buf), now_ms)
            except ProtocolError:
                pass
        state = sup.state(now_ms)
        cmd = sup.command(now_ms)
        if not quiet and (state is not last_state or cmd != last_cmd):
            print(f"  [{now_ms/1000:7.2f}s] {state.value:5s} "
                  f"vx {cmd[0]:+.2f} wz {cmd[1]:+.2f} "
                  f"torque {'on' if sup.torque_on(now_ms) else 'off'}",
                  flush=True)
            last_state, last_cmd = state, cmd
        if peer and tick % 5 == 0:
            tx.sendto(encode_telemetry(Telemetry(
                seq_echo=sup.last_seq, state=state, vbat_v=11.4, up_z=1.0,
                vx_est=cmd[0], wz_est=cmd[1], servo_err=0, loop_late_pct=0,
            )), (peer, tlm_port))
        tick += 1
        time.sleep(0.02)                          # the 50 Hz control tick


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--boot-armed", action="store_true",
                   help="start in run mode, as if `run` was typed on the "
                        "tether")
    p.add_argument("--no-cal", action="store_true",
                   help="refuse ARM, like a robot with no calibration in NVS")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--quiet", action="store_true")
    a = p.parse_args()
    run(a.port, a.tlm_port, armed=a.boot_armed, arm_allowed=not a.no_cal,
        duration=a.duration, quiet=a.quiet)


if __name__ == "__main__":
    main()
