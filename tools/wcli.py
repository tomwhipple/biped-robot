#!/usr/bin/env python3
"""The robot's bench CLI over Wi-Fi -- authenticated, at-most-once.

The same command lines the USB tether takes (`ping`, `pose ...`, `balance`,
`home`, `release`, `sread`, `imu`, `gains`, ...), sent over UDP to the
firmware's Wi-Fi CLI and answered with the same text. docs/wifi-cli.md is the
manual: provisioning, what is refused over Wi-Fi, the threat model.

  wcli.py ping                       one command, print the reply, exit
  wcli.py 'pose ... s1500 BL' --linger 5     ... and keep printing async
                                     output (balance/pose verdicts) for 5 s
  wcli.py                            interactive: a line per command (stdin
  some_script | wcli.py              may be a pipe); async output as it comes
  wcli.py --keygen                   make ~/.config/bimo/wcli.key and print
                                     the tether line that provisions it
  wcli.py --fingerprint              the key's fingerprint (the robot's
                                     `wcli` prints the same 8 hex digits)

Options: --host (default $BIMO_HOST or 192.168.2.90), --port (4213),
--key-file (default $BIMO_WCLI_KEY or ~/.config/bimo/wcli.key).

Exit status (one-command mode): 0 executed, 2 refused by the robot, 3 NOT
executed (robot busy / cancelled before it ran), 4 outcome unknown or timed
out, 1 no session (robot unreachable, no key or a different key).

tools/wcli_bridge.py is the drop-in replacement for the tether FIFO bridge.
"""
import argparse
import os
import queue
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

import wcliproto as W  # noqa: E402

EXIT = {"done": 0, "refused": 2, "busy": 3, "cancelled": 3, "unknown": 4,
        "timeout": 4}

NOTE = {
    "busy": "NOT EXECUTED (robot busy: a command in flight or its queue full)",
    "cancelled": "NOT EXECUTED (no answer in time; the robot confirmed it "
                 "never ran)",
    "unknown": "OUTCOME UNKNOWN (session lost after sending -- robot rebooted "
               "or another client paired; NOT re-sent)",
    "timeout": "TIMEOUT (no output for the idle timeout; it may still be "
               "running -- NOT re-sent)",
}


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--host", default=W.DEFAULT_HOST,
                   help="robot address (default $BIMO_HOST or 192.168.2.90)")
    p.add_argument("--port", type=int, default=W.PORT)
    p.add_argument("--key-file", default=W.DEFAULT_KEY_FILE,
                   help="shared key, 64 hex digits, mode 0600 "
                        "(default $BIMO_WCLI_KEY or ~/.config/bimo/wcli.key)")
    p.add_argument("--timeout", type=float, default=30.0,
                   help="give up on a command after this many seconds with "
                        "no output from it (default 30)")


def keygen(path: str) -> int:
    try:
        key = W.new_key_file(path)
    except FileExistsError:
        print(f"{path} exists -- refusing to overwrite a key (delete it first "
              f"if you mean to rotate, then re-provision the robot)",
              file=sys.stderr)
        return 1
    print(f"wrote {path} (mode 600), fingerprint {W.key_fingerprint(key)}")
    print("Provision the robot over the USB TETHER (never over Wi-Fi), bench "
          "mode -- e.g. into the tether bridge's FIFO:")
    print(f'  echo "wcli key $(cat {path})" > <board.fifo>')
    print("then check the tether `wcli` line shows the same fingerprint. The "
          "key will appear in that tether log; it stays on mira.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split("\n\n", 1)[1])
    add_common_args(p)
    p.add_argument("--linger", type=float, default=0.0,
                   help="one-command mode: keep printing async output this "
                        "many seconds after the reply")
    p.add_argument("--keygen", action="store_true")
    p.add_argument("--fingerprint", action="store_true")
    p.add_argument("command", nargs="*", help="one CLI line (quote it, or "
                   "give the words); none = interactive")
    a = p.parse_args(argv)

    if a.keygen:
        return keygen(a.key_file)
    try:
        key = W.load_key(a.key_file)
    except (OSError, ValueError) as e:
        print(f"wcli: {e}", file=sys.stderr)
        return 1
    if a.fingerprint:
        print(W.key_fingerprint(key))
        return 0

    def out(text):
        sys.stdout.write(text.replace("\r\n", "\n").replace("\r", ""))
        sys.stdout.flush()

    def note(text):
        print(f"# {text}", file=sys.stderr)

    cli = W.Client(a.host, key, a.port, on_text=out, on_note=note)
    try:
        cli.connect()
    except W.LinkDown as e:
        print(f"wcli: {e}", file=sys.stderr)
        return 1

    if a.command:
        line = " ".join(a.command)
        try:
            r = cli.command(line, idle_timeout=a.timeout)
        except W.LinkDown as e:
            print(f"wcli: {e}", file=sys.stderr)
            return 4
        if r.status in NOTE:
            note(NOTE[r.status])
        if a.linger > 0:
            cli.idle(a.linger)
        return EXIT[r.status]

    # Interactive / pipe: stdin on a thread, the network on this one, so async
    # output (a balance abort, a pose finishing) prints while you type.
    lines: "queue.Queue[str | None]" = queue.Queue()

    def reader():
        tty = sys.stdin.isatty()
        while True:
            if tty:
                sys.stderr.write("wcli> ")
                sys.stderr.flush()
            s = sys.stdin.readline()
            if not s:
                lines.put(None)
                return
            lines.put(s.rstrip("\r\n"))

    threading.Thread(target=reader, daemon=True).start()
    worst = 0
    while True:
        try:
            line = lines.get_nowait()
        except queue.Empty:
            try:
                cli.idle(0.1)
            except W.LinkDown:
                pass
            if cli.stale:
                note("session lost (robot rebooted or another client paired)"
                     " -- reconnecting")
                try:
                    cli.connect()
                except W.LinkDown as e:
                    note(str(e))
            continue
        if line is None:
            cli.idle(0.5)        # let trailing async output arrive
            return worst
        if not line.strip():
            continue
        try:
            r = cli.command(line, idle_timeout=a.timeout)
        except (W.LinkDown, ValueError, UnicodeEncodeError) as e:
            note(f"{e}")
            worst = max(worst, 4)
            continue
        if r.status in NOTE:
            note(NOTE[r.status])
        worst = max(worst, EXIT[r.status])


if __name__ == "__main__":
    sys.exit(main())
