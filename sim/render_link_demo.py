"""Render the failsafe figure for docs/control-channel.md.

Run:  .venv/bin/python sim/render_link_demo.py --run-name cmd_11v1

Drives sim/udp_agent.py with a real commander over a real UDP socket, kills
the commander mid-stride, and lays out the frames as a filmstrip labelled by
the watchdog's own per-tick state log -- so the picture shows what the link
actually did, not what it was supposed to do.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import imageio.v2 as imageio
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PY = sys.executable

BANDS = {                      # state -> label stripe colour (RGB)
    "live": (46, 125, 50),
    "stand": (198, 124, 0),
    "relax": (176, 42, 42),
    "estop": (120, 20, 120),
}


def _band(state, w, h=14):
    return np.tile(np.array(BANDS[state], dtype=np.uint8), (h, w, 1))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-name", default="cmd_11v1")
    p.add_argument("--out", default=os.path.join(
        ROOT, "docs", "control-channel-failsafe.png"))
    p.add_argument("--walk-s", type=float, default=6.0)
    p.add_argument("--duration", type=float, default=24.0)
    p.add_argument("--tmp", default="/tmp")
    args = p.parse_args()

    agent = None
    cmd = None
    try:
        agent = subprocess.Popen(
            [PY, os.path.join(HERE, "udp_agent.py"), "--run-name", args.run_name,
             "--duration", str(args.duration), "--render", "--out-dir", args.tmp,
             "--port", "4810", "--tlm-port", "4811", "--pose-port", "4812",
             "--boot-armed"],                    # the script commander never ARMs
            stdout=subprocess.PIPE, text=True)
        time.sleep(6)                               # policy + model load
        cmd = subprocess.Popen(
            [PY, os.path.join(ROOT, "link", "commander.py"), "--host", "127.0.0.1",
             "--source", "script", "--script", "walk", "--quiet",
             "--cmd-port", "4810", "--tlm-port", "4811"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(args.walk_s)
        cmd.send_signal(signal.SIGKILL)          # the radio dies, mid-stride
        print("  commander killed; watching the watchdog")
        print(agent.communicate()[0])
    finally:
        # never orphan the UDP agent / commander: an early exit (exception,
        # KeyboardInterrupt, agent crash) must not leak processes holding
        # ports 4810-4812, which would block the next run.
        if cmd is not None and cmd.poll() is None:
            cmd.kill()
        if agent is not None and agent.poll() is None:
            agent.terminate()
            try:
                agent.wait(timeout=5)
            except subprocess.TimeoutExpired:
                agent.kill()

    frames = imageio.mimread(os.path.join(args.tmp, "udp_agent.gif"),
                             memtest=False)
    with open(os.path.join(args.tmp, "link_states.json")) as f:
        states = json.load(f)["states"]

    # One column per state transition plus samples inside each state, so the
    # strip reads as a story, not an even sample of a mostly-still robot.
    picks, seen = [], None
    for i, s in enumerate(states):
        if s != seen:
            picks.append(i + 20)                    # just after the change
            seen = s
    picks += [len(states) - 1]
    picks = sorted({min(p, len(frames) - 1) for p in picks if p < len(frames)})

    cols = []
    for i in picks:
        img = np.asarray(frames[i])[..., :3]
        cols.append(np.vstack([_band(states[i], img.shape[1]), img]))
    strip = np.hstack(cols)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    imageio.imwrite(args.out, strip)
    print(f"  wrote {args.out}  ({len(picks)} panels: "
          f"{[states[i] for i in picks]})")


if __name__ == "__main__":
    main()
