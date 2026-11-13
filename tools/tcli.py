#!/usr/bin/env python3
"""Send CLI lines to the robot over the USB tether WITHOUT resetting the ESP32, print the replies, exit.

For the tether-only commands the Wi-Fi CLI refuses (`gains <id> <P> <D>`, `imu bias online save`, ...).
The port is opened with DTR and RTS held de-asserted from the start (set before open(), as idf.py monitor
does). ON THIS ROBOT THE ESP32 STILL RESETS ON OPEN (2026-11-12: boot log, trims zeroed): whatever lives in
RAM -- the learned trim T, a queued track, the online gyro bias -- is gone, and the robot is in its boot
state when the lines arrive. So by default the tool waits for the boot to finish (--boot-wait: the boot log
has gone quiet for 1 s, at most N s) before sending. EEPROM writes (`gains`) are unaffected by the reset.
Re-seed the trims (`trim T L/R`) afterwards; track_host.py does that at every run.

    python tools/tcli.py "gains 52"                       # read the R hip roll's P / D
    python tools/tcli.py "gains 52 64 32" "gains 52"      # several lines, one after the other
    python tools/tcli.py --port /dev/ttyUSB0 --timeout 5 "trim"

Each reply is read until the line has been quiet for --idle s (after something arrived) or --timeout s.
Exit 0 when every line got a reply, 1 otherwise. Without access to the port (not in `dialout` in this
session) it re-runs itself under `sg dialout`, as the flash scripts do.
"""
from __future__ import annotations

import argparse
import os
import shlex
import sys
import time


def open_port(port, baud):
    import serial
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.05
    # de-asserted BEFORE open: the ESP32's auto-reset sees no edge
    s.dtr = False
    s.rts = False
    s.open()
    return s


def drain(s, quiet=0.15, cap=2.0):
    """Read and drop what is pending (async output) until quiet for `quiet` s."""
    t0 = last = time.monotonic()
    while time.monotonic() - t0 < cap:
        if s.read(4096):
            last = time.monotonic()
        elif time.monotonic() - last >= quiet:
            return


def command(s, line, timeout, idle):
    s.write(line.encode() + b"\r\n")
    s.flush()
    buf, t0, last = b"", time.monotonic(), None
    while time.monotonic() - t0 < timeout:
        chunk = s.read(4096)
        if chunk:
            buf += chunk
            last = time.monotonic()
        elif last is not None and time.monotonic() - last >= idle:
            break
    return buf.decode(errors="replace")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lines", nargs="+", help="CLI lines, sent in order")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=4.0, help="s per line at most")
    ap.add_argument("--idle", type=float, default=0.4, help="s of quiet that ends a reply")
    ap.add_argument("--boot-wait", type=float, default=8.0,
                    help="after opening, wait up to this long for the (reset) boot log to go quiet for 1 s before "
                         "sending; 0 = do not wait (a board that does not reset)")
    ap.add_argument("--no-sg", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    try:
        s = open_port(a.port, a.baud)
    except PermissionError:
        if a.no_sg:
            raise
        cmd = shlex.join([sys.executable, os.path.abspath(__file__), *sys.argv[1:], "--no-sg"])
        os.execvp("sg", ["sg", "dialout", "-c", cmd])
    ok = True
    try:
        if a.boot_wait > 0:
            t0 = time.monotonic()
            got = 0
            last = t0
            while time.monotonic() - t0 < a.boot_wait:
                chunk = s.read(4096)
                if chunk:
                    got += len(chunk)
                    last = time.monotonic()
                elif got and time.monotonic() - last >= 1.0:
                    break
            print(f"(boot wait: {got} bytes in {time.monotonic() - t0:.1f} s)")
            s.write(b"\r\n")              # a throwaway line: the CLI's first read after boot
            time.sleep(0.2)
        drain(s)
        for line in a.lines:
            reply = command(s, line, a.timeout, a.idle)
            print(f">>> {line}")
            print(reply.rstrip() if reply.strip() else "(no reply)")
            ok = ok and bool(reply.strip())
    finally:
        s.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
