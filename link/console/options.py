"""The console's command line: bimo_gui's flags, plus what v2 adds."""
from __future__ import annotations

import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

CMD_PORT, TLM_PORT = 4210, 4211
STREAM_PORT = 8645          # sim/sil_twin.py's MJPEG
BENCH_PORT = 8650           # tools/board_bridge.py (UDP)
CAM_PORT = 8647             # tools/cam_relay.py (HTTP)
CAM_NAMES = ("emeet", "brio", "sony")
FLIGHT_PORT = 4221          # robot flight records, forwarded by mira's collector
SIM_FLIGHT_PORT = 4222      # the sim twin's flight records
SEND_HZ = 20.0
DEFAULT_VX, DEFAULT_VY, DEFAULT_WZ = 0.4, 0.2, 0.5   # client.h kDefault*
TAB_NAMES = ("checks", "drive", "joints", "telemetry", "sim", "session", "setup", "bench")

USAGE_NOTES = """\
Starts as an OBSERVER: it sends no command until TAKE CONTROL (the safety bar's
button, or a key bound to `control`), or --drive. HOLD [f] and E-STOP [e] are the out-of-band
operator stops (tools/estop.py's frames) and work in every role.

The robot beacons only to whoever commanded it last, so a watcher sees the
beacon only through a driver's --watch relay; the flight recorder (via mira's
collector, --forward to this machine's --flight-port) needs no driver at all.
"""


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="bimo_console", description="Operator console v2 (Qt).",
        epilog=USAGE_NOTES, formatter_class=argparse.RawDescriptionHelpFormatter)
    a = p.add_argument
    a("--host", help="robot (or sim) IP; where commands and stops go")
    a("--cmd-port", type=int, default=CMD_PORT,
      help=f"robot UDP port commands go to (default {CMD_PORT})")
    a("--tlm-port", type=int, default=TLM_PORT,
      help=f"local UDP port the beacon arrives on (default {TLM_PORT})")
    a("--drive", action="store_true",
      help="start DRIVING (needs --host) instead of observing")
    a("--readonly", action="store_true",
      help="start observing (the default; kept for bimo_gui's command lines)")
    a("--watch", metavar="HOST[:PORT]", help="relay each beacon to a watcher")
    a("--vx", type=float, default=DEFAULT_VX,
      help=f"forward / back speed a held key sends, m/s (Drive's vx max; default {DEFAULT_VX})")
    a("--vy", type=float, default=DEFAULT_VY,
      help=f"strafe speed a held key sends, m/s (Drive's vy max; default {DEFAULT_VY})")
    a("--wz", type=float, default=DEFAULT_WZ,
      help=f"turn rate a held key sends, rad/s (Drive's wz max; default {DEFAULT_WZ})")
    a("--rate", type=float, default=SEND_HZ,
      help=f"command send rate, Hz, in (0, 100]; under 5 is raised to 5, bimo_gui's "
           f"floor (default {SEND_HZ:g})")
    a("--keymap", help="keymap file (default ~/.bimo/keymap.conf)")
    a("--repo", default=ROOT,
      help="the robot repo: sim runs, the plant, <repo>/.venv for the Sim tab "
           "(default: this checkout)")
    a("--sessions", help="recordings and the log (default <repo>/hw_sessions)")
    a("--no-record", action="store_true",
      help="do not record the session's JSONL (recording is on by default)")
    a("--mirror", action="store_true",
      help="ask the robot for joint angles and attitude (the 3D view)")
    a("--flight-port", type=int, default=FLIGHT_PORT,
      help=f"UDP port for forwarded flight records (0 = off; default {FLIGHT_PORT})")
    a("--sim-flight-port", type=int, default=SIM_FLIGHT_PORT,
      help=f"UDP port for the sim twin's flight records (0 = off; default "
           f"{SIM_FLIGHT_PORT})")
    a("--plant", help="MuJoCo plant for the 3D view (default: the v6 plant)")
    a("--run-name", help="the policy run the Sim tab preselects (one exported to "
      "sim/sil/weights/<name>.silw.json)")
    a("--sim-cmd-port", type=int, default=0,
      help="the sim twin's command port (default 0: --cmd-port + 100)")
    a("--stream-port", type=int, default=STREAM_PORT,
      help=f"the sim's MJPEG render port on 127.0.0.1 (default {STREAM_PORT})")
    a("--sim-view", metavar="URL", help="an MJPEG sim stream to show")
    a("--bench-telemetry", metavar="HOST[:PORT]",
      help="tools/board_bridge.py's telemetry (watch-only)")
    a("--cam-relay", metavar="HOST[:PORT]",
      help=f"tools/cam_relay.py's bench cameras (default port {CAM_PORT})")
    a("--cam", action="append", default=[], choices=CAM_NAMES + ("off",),
      help="camera to open at start (twice for both panes)")
    a("--drive-view", action="store_true",
      help="with the bench flags: start on Drive, not Bench (bimo_gui's flag)")
    a("--stop-host", help="where HOLD / E-STOP go when there is no --host "
      "(default: $BIMO_HOST, then the flight collector's robot_addr)")
    a("--no-broadcast", action="store_true",
      help="send stops to the robot only, not also the LAN broadcast")
    a("--tab", type=str.lower, choices=TAB_NAMES,
      help="start on this tab (default drive; bench with the bench flags)")
    a("--headless", action="store_true",
      help="no window: line commands on stdin (tests/test_gui_e2e.py)")
    a("--screenshot", metavar="PNG", help="write a PNG of the window, then quit")
    a("--screenshot-after", type=float, default=4.0,
      help="seconds before --screenshot is taken (default 4)")
    a("--quit-after", type=float, default=0.0, help="quit after S seconds")
    return p


def parse(argv=None):
    o = parser().parse_args(argv)
    if len(o.cam) > 2:
        parser().error("--cam at most twice")
    if o.cam and not o.cam_relay:
        parser().error("--cam needs --cam-relay")
    if o.drive and o.readonly:
        parser().error("--drive and --readonly contradict")
    if o.drive and not o.host:
        parser().error("--drive needs --host: there is nowhere to command")
    if not (0.0 < o.rate <= 100.0):
        parser().error("--rate must be in (0, 100]")
    o.rate = max(o.rate, 5.0)            # the 5 Hz floor (bimo_gui's)
    o.observe = not o.drive
    o.sessions = o.sessions or os.path.join(o.repo, "hw_sessions")
    if not o.sim_cmd_port:
        o.sim_cmd_port = o.cmd_port + 100
    return o
