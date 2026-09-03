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

The per-tick line carries the EXTENDED channels too, but only when they are
off their trained defaults -- so the ordinary two-channel trace stays exactly
as it was, and a client that moves a slider (link/gui.cpp) is visibly
different from one that cannot (link/tui.cpp).
"""
import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from protocol import (CMD_PORT, FLAG_HOME, NUM_JOINTS, TLM_PORT,  # noqa
                      LinkState, ProtocolError, Supervisor, Telemetry,
                      decode_command, diag_reason, encode_telemetry)


def run(port=CMD_PORT, tlm_port=TLM_PORT, armed=False, arm_allowed=True,
        duration=None, quiet=False, deaf_home=False):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sup = Supervisor(armed=armed, arm_allowed=arm_allowed)
    peer, last_state, last_diag = None, None, None
    last_cmd = (0.0,) * 7
    # Mirror mode: joint angles ride the beacon only for a commander that asks
    # (FLAG_POSE). A level, not an edge -- stop asking and the very next
    # beacon is classic again. See docs/mirror-mode.md.
    want_pose, last_pose = False, None
    want_att, last_att = False, None
    homes_seen = sup.homes
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
                pkt = decode_command(buf)
            except ProtocolError:
                continue
            if deaf_home:
                # A robot that PREDATES the servo-reset bit: bit 4 means
                # nothing to it and the frame is otherwise ordinary, so the
                # console gets a perfectly reasonable-looking BENCH beacon
                # about something else entirely. That is what a console met on
                # 2026-09-02, and why it now says so out loud.
                pkt = pkt._replace(flags=pkt.flags & ~FLAG_HOME)
            sup.accept(pkt, now_ms)
            want_pose = pkt.pose
            want_att = pkt.att
        state = sup.state(now_ms)
        # (vx, vy, wz, crouch, lift, foot_dx, foot_dz) -- walker_env ext_cmd
        # order, which is NOT the wire order (vy sits between vx and wz).
        cmd = sup.command_ext(now_ms)
        diag = sup.diag()
        # Announced BEFORE the state line, because that is the order it
        # happens in: the request benches the loop and only then moves the
        # joints. There is no plant here, so "moved" is a claim about what
        # the firmware would do (cli.cpp cmdHome) -- the honest twin of a
        # bus transaction this file cannot perform.
        if sup.homes != homes_seen:
            homes_seen = sup.homes
            if not quiet:
                print(f"  [{now_ms/1000:7.2f}s] home: benched, all "
                      f"{NUM_JOINTS} joints -> calibrated zero (the stand), "
                      f"torque HOLDING", flush=True)
        if not quiet and (state is not last_state or cmd != last_cmd
                          or diag != last_diag):
            why = f"  -- {diag_reason(diag)}" if state is LinkState.BENCH \
                else ""
            # Only the channels that are saying something: an unchanged trace
            # for every commander that predates the extended frame.
            ext = ""
            if (cmd[1], cmd[3], cmd[4], cmd[5], cmd[6]) != (0.0, 1.0, 0.0,
                                                            0.0, 0.0):
                ext = (f" vy {cmd[1]:+.2f} crouch {cmd[3]:.2f} "
                       f"lift {cmd[4]:+.0f} fdx {cmd[5]:+.3f} "
                       f"fdz {cmd[6]:+.3f}")
            print(f"  [{now_ms/1000:7.2f}s] {state.value:5s} "
                  f"vx {cmd[0]:+.2f} wz {cmd[2]:+.2f}{ext} "
                  f"torque {'on' if sup.torque_on(now_ms) else 'off'}{why}",
                  flush=True)
            last_state, last_cmd, last_diag = state, cmd, diag
        if want_pose is not last_pose:
            print(f"  [{now_ms/1000:7.2f}s] pose "
                  f"{'requested -- beaconing joint angles' if want_pose else 'off -- classic beacon'}",
                  flush=True)
            last_pose = want_pose
        if want_att is not last_att:
            print(f"  [{now_ms/1000:7.2f}s] att "
                  f"{'requested -- beaconing the up vector' if want_att else 'off'}",
                  flush=True)
            last_att = want_att
        if peer and tick % 5 == 0:
            # seq_echo carries the bench diagnostic while BENCH, exactly as
            # firmware/main/wifi_link.cpp does it -- the twin has to be
            # faithful about that or the console is tested against a fiction.
            # A slow, deterministic wobble: no plant here, but a client in
            # mirror mode should see something MOVE, or it cannot tell a
            # working pose feed from ten zeros.
            joints = ()
            if want_pose:
                ph = now_ms / 1000.0
                joints = tuple(0.3 * math.sin(ph + 0.6 * i)
                               for i in range(NUM_JOINTS))
            # Attitude, on the same terms: only when asked (FLAG_ATT), and
            # visibly moving so a client can tell a working feed from a
            # hard-coded upright. A slow lean, well inside kFallUpZ.
            up_xy = ()
            if want_att:
                ph = now_ms / 1000.0
                up_xy = (0.25 * math.sin(0.5 * ph), 0.25 * math.cos(0.5 * ph))
            tx.sendto(encode_telemetry(Telemetry(
                seq_echo=sup.seq_echo(now_ms), state=state, vbat_v=11.4,
                up_z=(1.0 - (up_xy[0] ** 2 + up_xy[1] ** 2)) ** 0.5
                if up_xy else 1.0,
                vx_est=cmd[0], wz_est=cmd[2], servo_err=0,
                loop_late_pct=0, joints=joints, up_xy=up_xy,
                t_us=int(time.time() * 1e6),      # a twin is always synced
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
    p.add_argument("--deaf-home", action="store_true",
                   help="ignore FLAG_HOME, like firmware older than "
                        "2026-09-02")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--quiet", action="store_true")
    a = p.parse_args()
    run(a.port, a.tlm_port, armed=a.boot_armed, arm_allowed=not a.no_cal,
        duration=a.duration, quiet=a.quiet, deaf_home=a.deaf_home)


if __name__ == "__main__":
    main()
