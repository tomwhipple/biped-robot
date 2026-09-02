"""Sim twin of the on-robot command listener -- the ESP32 firmware's double.

Run:  .venv/bin/python sim/udp_agent.py --run-name cmd_11v1 --render
      .venv/bin/python link/commander.py --host 127.0.0.1 --source gamepad

This is the whole point of the link design: the robot-side half of the
protocol (decode_command + Watchdog + the torque-release policy) is ordinary
Python in link/protocol.py, so it can run against MuJoCo over a real UDP
socket. Every byte, every timeout, every stale-sequence rejection and every
link-loss decay is exercised here, at 1x real time, before a single part is
ordered. The firmware's job shrinks to a faithful C port of a module that has
already been debugged -- see docs/control-channel.md for the port notes.

What is NOT modeled: WiFi's actual loss and jitter (use --drop / --delay-ms
to inject some), and the 1 Mbaud servo bus below the policy, which
walker_env already covers via latency_ms / backlash_deg.
"""
import argparse
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time

import imageio.v2 as imageio
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "link"))

from eval_commands import base_env, norm_cmd            # noqa: E402
from eval_policy import load, run_env_kwargs            # noqa: E402
from protocol import (CMD_PORT, ProtocolError, TLM_PORT,  # noqa: E402
                      Supervisor, Telemetry, decode_command, encode_telemetry)

POSE_PORT = 4212     # sim-only ground-truth pose; see GoalSource in sources.py


def _yaw(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(run_dir, render=False, duration=None, port=CMD_PORT,
        tlm_port=TLM_PORT, pose_port=POSE_PORT, drop=0.0, delay_ms=0.0,
        realtime=True, out_dir=None, boot_armed=False, **env_kw):
    kw = run_env_kwargs(run_dir, **env_kw)
    model, env = load(run_dir, render_mode="rgb_array" if render else None,
                      **kw)
    base = base_env(env)
    if not base.command_mode:
        raise SystemExit(f"{run_dir} is not a command_mode run -- the link "
                         f"commands (vx, wz), which only those policies read")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Boots BENCH like the board: an ARM edge (link/tui.cpp) arms it.
    # --boot-armed is the twin of typing `run` on the tether, for the
    # ARM-ignorant script/gamepad commanders.
    dog = Supervisor(armed=boot_armed)
    homes_seen = dog.homes
    rng = np.random.default_rng(0)
    obs = env.reset()
    frames, peer, held, late = [], None, [], 0
    tick_states = []
    last_state = dog.state(0.0)
    t0 = time.monotonic()
    print(f"udp_agent: {os.path.basename(run_dir)} listening on :{port} "
          f"(telemetry -> :{tlm_port}, pose -> :{pose_port})")

    i = 0
    while duration is None or (i * base.control_dt) < duration:
        tick = time.monotonic()
        now_ms = (tick - t0) * 1000.0

        # -- drain the socket; last valid packet wins ----------------------
        while True:
            try:
                buf, addr = rx.recvfrom(64)
            except BlockingIOError:
                break
            peer = addr[0]
            if drop and rng.uniform() < drop:
                continue                       # injected WiFi loss
            held.append((now_ms + delay_ms, buf))
        for due, buf in [h for h in held if h[0] <= now_ms]:
            try:
                dog.accept(decode_command(buf), now_ms)
            except ProtocolError:
                pass                           # stray/corrupt: drop silently
        held = [h for h in held if h[0] > now_ms]

        # -- apply ---------------------------------------------------------
        # A "reset the servos" request (FLAG_HOME) benches the Supervisor and
        # leaves torque holding, exactly as on the robot. What it does NOT do
        # here is re-pose the plant: on the robot the move is a bench-mode
        # SERVO BUS transaction (cli.cpp cmdHome), and this twin has no bench
        # half -- the policy owns every actuator. Benched, the command decays
        # to the trained stand, which is the nearest true thing the plant can
        # do; the joint-accurate version of this belongs to the firmware.
        if dog.homes != homes_seen:
            homes_seen = dog.homes
            print(f"  [{now_ms/1000:6.2f}s] home: benched, torque holding a "
                  f"stand (no bus half in sim)", flush=True)
        state = dog.state(now_ms)
        if state is not last_state:
            print(f"  [{now_ms/1000:6.2f}s] link {last_state.value} -> "
                  f"{state.value}", flush=True)
            last_state = state
        v, w = dog.command(now_ms)
        base.set_command(v, w)
        # The twin never emits LinkState.VLAND / VSAFE: those come from the
        # firmware's battguard::Guard, and supply_voltage here is a fixed plant
        # parameter (it scales the torque-speed envelope), not a pack that
        # discharges. Nothing to guard in sim -- see docs/wiring.md.
        base.set_torque_enabled(dog.torque_on(now_ms))
        obs[0, -2:] = norm_cmd(env, base._cmd)
        action, _ = model.predict(obs, deterministic=True)
        obs, _, _, infos = env.step(action)
        if render:
            frames.append(base.render())
            tick_states.append(state.value)

        # -- telemetry + sim-only pose ------------------------------------
        if peer and i % 5 == 0:               # 10 Hz
            info = infos[0]
            tx.sendto(encode_telemetry(Telemetry(
                # seq_echo carries the bench diagnostic while BENCH,
                # exactly as the firmware's beacon does (protocol.py).
                seq_echo=dog.seq_echo(now_ms), state=state,
                vbat_v=float(base.supply_voltage), up_z=info.get("up_z", 0.0),
                vx_est=info.get("vx_body", 0.0), wz_est=info.get("wz", 0.0),
                servo_err=0, loop_late_pct=int(100 * late / max(1, i)),
                # A sim's "robot clock" is the host's: synced by definition.
                t_us=int(time.time() * 1e6),
            )), (peer, tlm_port))
            q = base.data.qpos
            tx.sendto(struct.pack("<3f", float(q[0]), float(q[1]),
                                  _yaw(q[3:7])), (peer, pose_port))

        i += 1
        if realtime:
            slack = base.control_dt - (time.monotonic() - tick)
            if slack > 0:
                time.sleep(slack)
            else:
                late += 1

    if frames:
        dest = out_dir or run_dir
        _save(frames, dest, "udp_agent")
        # Per-tick states, so a figure can label frames by what the watchdog
        # actually did rather than by guessing from wall-clock (rendering
        # does not keep up with real time, so the two drift apart).
        with open(os.path.join(dest, "link_states.json"), "w") as f:
            json.dump({"control_dt": base.control_dt, "states": tick_states},
                      f)
    return i


def _save(frames, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    gif = os.path.join(out_dir, f"{name}.gif")
    imageio.mimwrite(gif, [f for f in frames if f is not None], fps=50, loop=0)
    try:
        import imageio_ffmpeg
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-i", gif,
                        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                        "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart",
                        gif.replace(".gif", ".mov")],
                       check=True, capture_output=True, timeout=120)
    except Exception as e:
        print("  (mov conversion failed:", e, ")")
    print(f"  wrote {gif} (+.mov)")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--run-name", required=True)
    p.add_argument("--runs-dir", default=os.path.join(HERE, "runs"))
    p.add_argument("--out-dir", default=None, help="where renders land")
    p.add_argument("--render", action="store_true")
    p.add_argument("--duration", type=float, default=None, help="sim seconds")
    p.add_argument("--port", type=int, default=CMD_PORT)
    p.add_argument("--tlm-port", type=int, default=TLM_PORT)
    p.add_argument("--pose-port", type=int, default=POSE_PORT)
    p.add_argument("--drop", type=float, default=0.0,
                   help="fraction of command packets to discard (fake WiFi)")
    p.add_argument("--delay-ms", type=float, default=0.0,
                   help="extra link delay on every command packet")
    p.add_argument("--no-realtime", action="store_true",
                   help="run flat out; only useful with a scripted sender")
    p.add_argument("--payload", type=float, default=None)
    p.add_argument("--latency-ms", type=float, default=None)
    p.add_argument("--boot-armed", action="store_true",
                   help="start in run mode (the tethered `run`) instead of "
                        "BENCH; needed by commanders that never send ARM")
    args = p.parse_args()
    run(os.path.join(args.runs_dir, args.run_name), render=args.render,
        duration=args.duration, port=args.port, tlm_port=args.tlm_port,
        pose_port=args.pose_port, drop=args.drop, delay_ms=args.delay_ms,
        realtime=not args.no_realtime, out_dir=args.out_dir,
        boot_armed=args.boot_armed,
        payload_mass=args.payload, latency_ms=args.latency_ms)


if __name__ == "__main__":
    main()
