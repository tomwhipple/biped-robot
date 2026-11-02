#!/usr/bin/env python3
"""Drop-in replacement for the tether FIFO bridge, over the Wi-Fi CLI.

  wcli_bridge.py <fifo> <log> [--host H] [--key-file F] [--stale S]

Lines written to <fifo> are sent to the robot's Wi-Fi CLI (tools/wcli.py,
docs/wifi-cli.md); everything the robot prints is appended to <log> with a
UTC stamp per line -- the SAME format as the tether bridge, so session
scripts that write the FIFO and grep the log work unchanged:

  HH:MM:SS.mmmZ > pose 2048 ... s1500       a command, when it is sent
  HH:MM:SS.mmmZ   pose: streaming ...       a line the robot printed
  HH:MM:SS.mmmZ # ...                       the bridge's own notes

`__quit__` on the FIFO closes the bridge, as on the tether.

The tether bridge's rule holds: a command that cannot be delivered NOW is
DROPPED, never queued for later -- a stale motion command sent late is
worse than none. Each such line is logged as "# WRITE FAILED (...)" or
"# DROPPED (...)" so scripts can stop on it. Unlike the tether, delivery is
confirmed: every command is executed AT MOST ONCE (the robot keys it by
sequence number; retransmits are answered, never re-run), and the bridge
waits for each command's reply before sending the next, so lines run in
FIFO order. A line that waited longer than --stale seconds behind a slow
command is dropped rather than sent late.

What the log may say that the tether's never did:
  # NOT EXECUTED (...)      the robot confirmed the line did not run
  # OUTCOME UNKNOWN (...)   the session was lost after sending (robot
                            rebooted?) -- the line is NOT re-sent
  # TIMEOUT (...)           no output for --timeout s; it may still run
  # link lost / restored    no answer from the robot / session re-made
  [wcli: N chunk(s) of robot output lost]   (as a reply line) output the
                            robot could not re-send: a gap, not silence
"""
import argparse
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

import wcliproto as W  # noqa: E402
from wcli import NOTE, add_common_args  # noqa: E402

QUIT = "__quit__"
LINK_LOST_S = 6.0      # no answer to keepalives this long = link lost
RECONNECT_S = 2.0


def stamp() -> str:
    return datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


class Log:
    """The tether bridge's log format, line-buffered, thread-safe."""

    def __init__(self, path: str):
        self.f = open(path, "a", buffering=1)
        self.lock = threading.Lock()
        self.split = W.LineSplitter()

    def write(self, mark: str, text: str) -> None:
        with self.lock:
            self.f.write(f"{stamp()} {mark} {text}\n" if mark != " "
                         else f"{stamp()}   {text}\n")

    def note(self, text: str) -> None:
        self.write("#", text)

    def sent(self, line: str) -> None:
        self.write(">", line)

    def robot_text(self, text: str) -> None:
        for line in self.split.feed(text):
            self.write(" ", line)


def fifo_reader(fifo: str, q: "queue.Queue") -> None:
    while True:
        with open(fifo) as f:          # blocks until a writer opens it
            for line in f:
                line = line.rstrip("\n")
                q.put((time.monotonic(), line))
                if line == QUIT:
                    return


def run(a) -> int:
    if not os.path.exists(a.fifo):
        os.mkfifo(a.fifo)
    log = Log(a.log)
    try:
        key = W.load_key(a.key_file)
    except (OSError, ValueError) as e:
        log.note(f"wcli bridge NOT started: {e}")
        print(f"wcli_bridge: {e}", file=sys.stderr)
        return 1
    cli = W.Client(a.host, key, a.port, on_text=log.robot_text,
                   on_note=lambda s: log.note(f"wcli: {s}"))
    log.note(f"wcli bridge -> {cli.addr[0]}:{cli.addr[1]} "
             f"(key {W.key_fingerprint(key)})")
    up = False
    ever_up = False
    last_try = 0.0

    def try_connect(quiet_fail: bool) -> bool:
        nonlocal up, ever_up, last_try
        last_try = time.monotonic()
        try:
            cli.connect(timeout=1.5)
        except W.LinkDown as e:
            if not quiet_fail:
                log.note(f"link down: {e}")
            up = False
            return False
        log.note("link restored -- new session" if ever_up else "session up")
        up = ever_up = True
        return True

    try_connect(quiet_fail=False)
    q: "queue.Queue" = queue.Queue()
    threading.Thread(target=fifo_reader, args=(a.fifo, q), daemon=True).start()

    while True:
        try:
            t_in, line = q.get_nowait()
        except queue.Empty:
            # Idle: async output, keepalive, and noticing a dead link.
            if up:
                try:
                    cli.idle(0.1)
                except OSError:
                    pass
                if cli.stale:
                    log.note("session lost (robot rebooted, or another client "
                             "paired) -- reconnecting")
                    up = False
                elif cli.last_heard and time.monotonic() - cli.last_heard > LINK_LOST_S:
                    log.note(f"link lost (no answer for {LINK_LOST_S:.0f} s)")
                    up = False
            else:
                time.sleep(0.1)
            if not up and time.monotonic() - last_try > RECONNECT_S:
                try_connect(quiet_fail=True)
            continue

        if line == QUIT:
            cli.idle(0.5)
            log.note("bridge closed")
            cli.close()
            return 0
        waited = time.monotonic() - t_in
        if waited > a.stale:
            log.note(f"DROPPED (stale: waited {waited:.1f} s behind earlier "
                     f"commands): {line}")
            continue
        if not line.strip():
            continue
        if not up and not try_connect(quiet_fail=True):
            log.note(f"WRITE FAILED (link down): {line}")
            continue
        try:
            data_ok = len(line.encode("ascii")) <= W.MAX_LINE
        except UnicodeEncodeError:
            data_ok = False
        if not data_ok:
            log.note(f"WRITE FAILED (not ASCII, or longer than {W.MAX_LINE} "
                     f"bytes): {line}")
            continue
        log.sent(line)
        try:
            r = cli.command(line, idle_timeout=a.timeout)
        except W.LinkDown as e:
            log.note(f"WRITE FAILED ({e}): {line}")
            up = False
            continue
        if r.status in NOTE:
            log.note(f"{NOTE[r.status]}: {line}")
        if r.status == "unknown":
            up = False


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("\n\n", 1)[1])
    p.add_argument("fifo")
    p.add_argument("log")
    add_common_args(p)
    p.add_argument("--stale", type=float, default=10.0,
                   help="drop a line that waited longer than this behind "
                        "earlier commands (default 10 s)")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
