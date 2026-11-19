"""A terminal key controller for `track_host.py --cmd file:PATH`: it writes the command file, nothing else (no robot,
no network, stdlib only: termios / tty raw mode).

    python walk_keys.py /tmp/walk_cmd            # terminal 2; terminal 1: track_host.py --T 4 --cmd file:/tmp/walk_cmd --live

Keys (the command is `vx vy wz`: mm/s forward, mm/s left, deg/s left; one key press is one increment):
    W / S   forward / back faster      A / D   sideways left / right      Q / E   turn left / right
    X       zero the command (march in place)        SPACE or ESC   stop: writes `stop` and exits
    Ctrl-C  exits WITHOUT a stop (the host keeps the last command); press SPACE first
The file is written atomically (a temp file, then os.replace). At start it holds `0 0 0` (unless --keep). The line under
the keys shows the command and, from PATH.status (the host's one-line JSON), what the host applied after its
envelope / slew / pre-check, and its state.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import sys
import time


class Controller:
    """The key map and the command it builds: the testable part."""

    def __init__(self, T=4.0, step_mm=5.0, step_deg=1.0, limits=(50.0, 32.0, 15.0, 5.0)):
        """step_mm / step_deg: one key press changes the per-STEP displacement by this much (the speeds are that / T);
        limits: the host's default envelope (fwd, back, side, turn per step), the keys never go past it."""
        self.T, self.sx, self.sa = float(T), float(step_mm), float(step_deg)
        self.lim = limits
        self.v = [0.0, 0.0, 0.0]           # per-step displacement: mm, mm, deg
        self.stopped = False

    def _clamp(self):
        f, b, s, y = self.lim
        self.v[0] = max(-b, min(f, self.v[0]))
        self.v[1] = max(-s, min(s, self.v[1]))
        self.v[2] = max(-y, min(y, self.v[2]))
        self.v = [round(x, 3) for x in self.v]

    def key(self, k):
        """One key -> True if the command changed (or stop was asked)."""
        k = k.lower() if len(k) == 1 else k
        if k in (" ", "\x1b"):
            self.stopped = True
            return True
        d = {"w": (0, +self.sx), "s": (0, -self.sx), "a": (1, +self.sx), "d": (1, -self.sx),
             "q": (2, +self.sa), "e": (2, -self.sa)}.get(k)
        if k == "x":
            self.v = [0.0, 0.0, 0.0]
            return True
        if d is None:
            return False
        self.v[d[0]] += d[1]
        self._clamp()
        return True

    def line(self):
        if self.stopped:
            return "stop\n"
        return "{:g} {:g} {:g}\n".format(*[round(x / self.T, 4) for x in self.v])


def write_atomic(path, text):
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def read_status(path):
    try:
        with open(path + ".status") as f:
            return json.loads(f.read())
    except (OSError, ValueError):
        return None


def render(ctl, st):
    s = "asked {:+.2f} {:+.2f} {:+.2f}".format(*[x / ctl.T for x in ctl.v])
    if st is None:
        return s + "   host: no status yet"
    a = st.get("applied", [0, 0, 0])
    n = "; ".join(st.get("notes", []))[:60]
    return s + "   host: {} step {} applied {:+.2f} {:+.2f} {:+.2f}{}".format(
        st.get("state", "?"), st.get("step", "?"), a[0], a[1], a[2], f" ({n})" if n else "")


def _read_key(fd):
    ch = os.read(fd, 1).decode("latin1")
    if ch == "\x1b":                                     # a lone ESC is stop; an arrow key is ESC [ X: ignored
        if select.select([fd], [], [], 0.05)[0]:
            while select.select([fd], [], [], 0.01)[0]:
                os.read(fd, 1)
            return ""
    return ch


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="the command file track_host.py --cmd file:PATH reads")
    ap.add_argument("--T", type=float, default=4.0, help="the host's step time, s (a key press = --step-mm per step)")
    ap.add_argument("--step-mm", type=float, default=5.0, help="W/S/A/D: change of the per-step displacement, mm")
    ap.add_argument("--step-deg", type=float, default=1.0, help="Q/E: change of the per-step heading change, deg")
    ap.add_argument("--keep", action="store_true", help="do not reset the file to `0 0 0` at start")
    a = ap.parse_args(argv)
    import termios
    import tty
    ctl = Controller(a.T, a.step_mm, a.step_deg)
    if not a.keep:
        write_atomic(a.path, ctl.line())
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("W/S fwd/back  A/D left/right  Q/E turn  X zero  SPACE/ESC stop+exit  Ctrl-C exit (no stop)\r")
    try:
        tty.setcbreak(fd)
        while True:
            sys.stdout.write("\r\x1b[K" + render(ctl, read_status(a.path)))
            sys.stdout.flush()
            if select.select([fd], [], [], 0.2)[0]:
                if ctl.key(_read_key(fd)):
                    write_atomic(a.path, ctl.line())
                    if ctl.stopped:
                        sys.stdout.write("\r\x1b[K" + "stop written\r\n")
                        return 0
    except KeyboardInterrupt:
        sys.stdout.write("\r\nexited without a stop; the host keeps the last command\r\n")
        return 1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    sys.exit(main())
