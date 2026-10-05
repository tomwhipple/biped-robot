#!/usr/bin/env python3
"""Bench check of the robot's WiFi/UDP link -- no motion, ever.

Sends DISABLED stand frames (ENABLE off, vx=wz=0) at 20 Hz to the robot's
command port and listens for the 10 Hz telemetry beacon. A benched robot
ignores the commands (ctrl never drains the mailbox in bench mode) and
reports state BENCH; the point is to prove both wire directions and measure
loss, without arming anything.

    .venv/bin/python link/verify_udp.py --host <robot-ip> [--seconds 10]

Exit 0 iff at least one valid telemetry frame arrived and >50% of the
expected beacons were heard.
"""
import argparse
import socket
import time

import protocol


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="robot IP")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", protocol.TLM_PORT))
    rx.settimeout(0.05)

    sent = got = bad = 0
    states = {}
    last_tlm = None
    t_end = time.time() + args.seconds
    next_tx = time.time()
    while time.time() < t_end:
        now = time.time()
        if now >= next_tx:
            next_tx += 1.0 / protocol.SEND_HZ
            # flags=0: ENABLE off. The robot may never act on these frames.
            tx.sendto(protocol.encode_command(sent, 0.0, 0.0, 0),
                      (args.host, protocol.CMD_PORT))
            sent += 1
        try:
            buf, _ = rx.recvfrom(64)
        except socket.timeout:
            continue
        try:
            last_tlm = protocol.decode_telemetry(buf)
        except protocol.ProtocolError:
            bad += 1
            continue
        got += 1
        states[last_tlm.state.name] = states.get(last_tlm.state.name, 0) + 1

    expect = int(args.seconds * 10)  # 10 Hz beacon
    print(f"sent {sent} cmd frames, heard {got}/{expect} telemetry "
          f"({bad} undecodable)")
    if last_tlm is not None:
        print(f"states seen: {states}")
        print(f"last: state={last_tlm.state.name} vbat={last_tlm.vbat_v:.1f}V "
              f"up_z={last_tlm.up_z:.2f} seq_echo={last_tlm.seq_echo}")
    ok = got > 0 and got > expect // 2
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
