#!/usr/bin/env python3
"""Capture the robot's OBSERVATION VECTOR off the tether and summarise it.

WHY THIS EXISTS
---------------
The deployed policy walks in SIL (docs/sil-harness.md) and produces garbage
motion on hardware. SIL runs the firmware's own assembler, history ring,
quantiser and network against SIM-fed sensors, so everything between the
sensor and the servo is already covered. What is NOT covered is the one thing
SIL cannot supply: the numbers the REAL sensors put into `obs::Inputs`. A
degrees-for-radians scale error, a flipped sign, a swapped IMU axis or a
permuted joint index all look identical from outside -- the loop runs, the
servos move, the robot falls -- and none of them can be settled by reading
code. They have to be measured.

`obsdump` (firmware/main/obs_dump.h) makes the armed loop stream, at ~5 Hz,
one CSV line per tick holding BOTH halves of the observation: the raw
`obs::Inputs` the sensors produced, and the assembled 49-float frame handed to
the policy. This tool drives that, writes the records to a CSV, and prints
min/max/mean/RMS per named channel -- statistics that can be put beside the
same statistics from the sim.

USAGE
-----
    # 20 s capture from an ARMED robot (`run` first, robot supported!)
    .venv/bin/python tools/obs_capture.py --port /dev/ttyUSB0 --seconds 20

    # re-summarise a capture, including the assembled-frame columns
    .venv/bin/python tools/obs_capture.py --summarize captures/obs_x.csv --frame

    # parse a console log captured by some other means (no serial port)
    .venv/bin/python tools/obs_capture.py --replay console.log --out out.csv

    # per-channel RMS ratio, hardware vs sim
    .venv/bin/python tools/obs_capture.py --compare hw.csv sim.csv

NOTES
-----
* The robot must be ARMED (`run`). Benched, the control task returns before
  the servos are read, so there is no observation to dump and the firmware
  says so rather than streaming zeros.
* Records share the tether with the 10 Hz binary telemetry frames. Both are
  written by the same firmware task, so a record is never split by one -- but
  binary bytes DO land between records, which is why every line is located by
  its `OBS,` tag rather than by position.
* Column names come from the `OBSHDR` line the firmware emits when the dump is
  enabled, i.e. from the generated obs spec. This tool keeps no second copy of
  the joint order; a capture with no header is refused rather than guessed at.
* Nothing here writes to the servo bus. The only bytes sent are the three CLI
  lines `obsdump off` / `obsdump on` / `obsdump off`.
"""

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path

# Tag lines the firmware emits (obs_dump.cpp). Anchored on the tag, not on the
# start of a line: binary telemetry can precede a record on the same "line".
RE_HDR = re.compile(r"OBSHDR,([^\r\n]*)")
RE_REC = re.compile(r"OBS,([^\r\n]*)")

# How the summary table is grouped. Everything not matched falls into "other".
SECTIONS = (
    ("q", lambda n: n.startswith("q_")),
    ("dq", lambda n: n.startswith("dq_")),
    ("up", lambda n: n.startswith("up_")),
    ("gyro", lambda n: n.startswith("gyro_")),
    ("phase", lambda n: n == "phase"),
    ("cmd", lambda n: n.startswith("cmd_")),
)


def is_frame(name):
    return name.startswith("f_")


# -- capture ---------------------------------------------------------------


def parse_stream(blob):
    """Pull (columns, rows) out of a raw console byte string.

    `blob` is whatever came off the UART, binary telemetry frames and all.
    Records whose field count disagrees with the header are DROPPED and
    counted: a short line is a truncated one, and silently padding it would
    invent channel values.
    """
    text = blob.decode("utf-8", errors="replace")
    columns = None
    rows = []
    dropped = 0
    for line in text.replace("\r", "\n").split("\n"):
        m = RE_HDR.search(line)
        if m:
            cols = [c.strip() for c in m.group(1).split(",")]
            if columns is not None and cols != columns:
                raise SystemExit(
                    "obs_capture: the column names changed mid-capture -- the "
                    "board was reflashed or reset while streaming"
                )
            columns = cols
            continue
        m = RE_REC.search(line)
        if not m:
            continue
        if columns is None:
            dropped += 1        # a record from before our header; unnameable
            continue
        fields = m.group(1).split(",")
        if len(fields) != len(columns):
            dropped += 1
            continue
        try:
            rows.append([float(f) for f in fields])
        except ValueError:
            dropped += 1
    return columns, rows, dropped


def capture_serial(port, baud, seconds):
    """Enable the dump, read for `seconds`, disable it. Returns raw bytes."""
    try:
        import serial                                   # pyserial
    except ImportError:
        raise SystemExit("obs_capture: pyserial is missing (pip install pyserial)")

    blob = bytearray()
    with serial.Serial(port, baud, timeout=0.1) as ser:
        # Opening a CP2102 port RESETS the ESP32 and the board eats the first
        # command (MEMORY: esp32-serial-open-resets-board). Wait the boot out,
        # then drain the banner before saying anything that matters.
        time.sleep(1.5)
        ser.reset_input_buffer()

        # `off` first, unconditionally: enabling a dump that is ALREADY on
        # emits no fresh OBSHDR line, and this tool refuses to guess column
        # names. Off-then-on guarantees the header.
        ser.write(b"\r\nobsdump off\r\n")
        ser.flush()
        time.sleep(0.4)
        ser.reset_input_buffer()

        ser.write(b"obsdump on\r\n")
        ser.flush()
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                chunk = ser.read(4096)
                if chunk:
                    blob += chunk
        finally:
            # Always turn it back off, including on Ctrl-C: a board left
            # streaming fills the tether the next operator wants to type on.
            ser.write(b"\r\nobsdump off\r\n")
            ser.flush()
            time.sleep(0.3)
            blob += ser.read(4096)
    return bytes(blob)


def write_csv(path, columns, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(columns)
        w.writerows(rows)


def read_csv(path):
    with Path(path).open(newline="") as fh:
        r = csv.reader(fh)
        try:
            columns = next(r)
        except StopIteration:
            raise SystemExit(f"obs_capture: {path} is empty")
        rows = []
        for i, row in enumerate(r, start=2):
            if len(row) != len(columns):
                raise SystemExit(
                    f"obs_capture: {path}:{i} has {len(row)} fields, "
                    f"header has {len(columns)}"
                )
            rows.append([float(v) for v in row])
    return columns, rows


# -- statistics ------------------------------------------------------------


def channel_stats(columns, rows):
    """{name: (n, min, max, mean, rms)} for every column except `tick`."""
    stats = {}
    for j, name in enumerate(columns):
        if name == "tick":
            continue
        vals = [row[j] for row in rows]
        finite = [v for v in vals if math.isfinite(v)]
        if not finite:
            stats[name] = (len(vals), math.nan, math.nan, math.nan, math.nan)
            continue
        n = len(finite)
        mean = sum(finite) / n
        rms = math.sqrt(sum(v * v for v in finite) / n)
        stats[name] = (n, min(finite), max(finite), mean, rms)
    return stats


def print_table(title, names, stats):
    if not names:
        return
    width = max(max(len(n) for n in names), len("channel"))
    print(f"\n{title}")
    print(f"  {'channel':<{width}}  {'n':>5}  {'min':>11}  {'max':>11}  "
          f"{'mean':>11}  {'rms':>11}")
    print(f"  {'-' * width}  {'-' * 5}  {'-' * 11}  {'-' * 11}  {'-' * 11}  "
          f"{'-' * 11}")
    for name in names:
        n, lo, hi, mean, rms = stats[name]
        print(f"  {name:<{width}}  {n:>5}  {lo:>11.5g}  {hi:>11.5g}  "
              f"{mean:>11.5g}  {rms:>11.5g}")


def summarise(columns, rows, show_frame):
    if not rows:
        raise SystemExit("obs_capture: no records -- nothing to summarise")
    stats = channel_stats(columns, rows)
    ticks = [row[columns.index("tick")] for row in rows] if "tick" in columns else []
    print(f"{len(rows)} records, {len(columns) - 1} channels", end="")
    if ticks:
        span = ticks[-1] - ticks[0]
        # The loop ticks at 50 Hz (obs::kControlDt), so the tick span is the
        # wall time the capture covers regardless of the console's rate.
        print(f", ticks {int(ticks[0])}..{int(ticks[-1])} "
              f"({span * 0.02:.1f} s of loop time)", end="")
    print()

    named = [c for c in columns if c != "tick" and not is_frame(c)]
    claimed = set()
    for title, pred in SECTIONS:
        names = [c for c in named if pred(c)]
        claimed.update(names)
        print_table(f"{title}  (raw obs::Inputs)", names, stats)
    other = [c for c in named if c not in claimed]
    print_table("other  (raw obs::Inputs)", other, stats)

    if show_frame:
        print_table("assembled frame (kFrameDim wide, fed to the history ring)",
                    [c for c in columns if is_frame(c)], stats)
    else:
        nframe = sum(1 for c in columns if is_frame(c))
        if nframe:
            print(f"\n({nframe} assembled-frame columns hidden; --frame shows "
                  f"them)")


# -- compare ---------------------------------------------------------------


def compare(path_a, path_b, show_frame):
    """Per-channel RMS ratio between two captures.

    The intended pair is a hardware capture and a SIM capture of the same
    commanded behaviour: a channel whose ratio is ~57 is degrees where radians
    were expected, ~1 is agreement, and a channel that is ~0 on one side is a
    sensor that is not reaching the observation at all. Generating the sim
    side is a separate job; this is the comparison mechanics, and it works on
    any two CSVs this tool wrote.
    """
    cols_a, rows_a = read_csv(path_a)
    cols_b, rows_b = read_csv(path_b)
    stats_a = channel_stats(cols_a, rows_a)
    stats_b = channel_stats(cols_b, rows_b)

    print(f"A  {path_a}   {len(rows_a)} records")
    print(f"B  {path_b}   {len(rows_b)} records")

    names = [c for c in cols_a if c != "tick" and c in stats_b]
    if not show_frame:
        names = [c for c in names if not is_frame(c)]
    only_a = [c for c in cols_a if c != "tick" and c not in stats_b]
    only_b = [c for c in cols_b if c != "tick" and c not in stats_a]

    if not names:
        raise SystemExit("obs_capture: the two captures share no channels")

    width = max(max(len(n) for n in names), len("channel"))
    print(f"\n  {'channel':<{width}}  {'rms A':>11}  {'rms B':>11}  "
          f"{'A/B':>9}")
    print(f"  {'-' * width}  {'-' * 11}  {'-' * 11}  {'-' * 9}")
    for name in names:
        rms_a = stats_a[name][4]
        rms_b = stats_b[name][4]
        if not math.isfinite(rms_a) or not math.isfinite(rms_b):
            ratio = "nan"
        elif rms_b == 0.0:
            # Both flat at zero is agreement; only A moving is not a ratio.
            ratio = "1" if rms_a == 0.0 else "inf"
        else:
            ratio = f"{rms_a / rms_b:.4g}"
        print(f"  {name:<{width}}  {rms_a:>11.5g}  {rms_b:>11.5g}  "
              f"{ratio:>9}")

    for label, missing in (("A only", only_a), ("B only", only_b)):
        if missing:
            print(f"\n  {label}: {', '.join(missing)}")


# -- main ------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="capture and summarise the robot's observation vector")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial device")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="how long to stream (default 10)")
    ap.add_argument("--out", type=Path, default=None,
                    help="CSV to write (default captures/obs_<ts>.csv)")
    ap.add_argument("--frame", action="store_true",
                    help="also print the assembled-frame columns")
    ap.add_argument("--replay", type=Path, default=None,
                    help="parse a saved console log instead of opening the "
                         "serial port")
    ap.add_argument("--summarize", "--summarise", dest="summarize", type=Path,
                    default=None, help="re-print the table for an existing CSV")
    ap.add_argument("--compare", nargs=2, metavar=("A.csv", "B.csv"),
                    default=None,
                    help="per-channel RMS ratio of two captures (hardware vs "
                         "a sim capture)")
    args = ap.parse_args(argv)

    if args.compare:
        compare(args.compare[0], args.compare[1], args.frame)
        return 0

    if args.summarize:
        columns, rows = read_csv(args.summarize)
        summarise(columns, rows, args.frame)
        return 0

    if args.replay:
        blob = Path(args.replay).read_bytes()
    else:
        print(f"obs_capture: {args.port} @ {args.baud}, {args.seconds:g} s")
        print("  the loop must be ARMED (`run`) and the robot supported")
        blob = capture_serial(args.port, args.baud, args.seconds)

    columns, rows, dropped = parse_stream(blob)
    if columns is None:
        # Every refusal path names what was actually seen; guessing the column
        # order from obs_spec.h would defeat the purpose of the instrument.
        hint = ""
        if b"BENCHED" in blob or b"benched" in blob:
            hint = ("\n  the board answered that the loop is BENCHED -- `run` "
                    "first (robot supported)")
        raise SystemExit(
            f"obs_capture: no OBSHDR line in {len(blob)} bytes of console "
            f"output; the firmware emits one whenever a dump is enabled" + hint)
    if not rows:
        raise SystemExit(
            "obs_capture: header seen but no records -- the loop published "
            "nothing (is it armed, and is it past the fall latch?)")

    out = args.out or Path("captures") / f"obs_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    write_csv(out, columns, rows)
    print(f"wrote {out}  ({len(rows)} records"
          + (f", {dropped} malformed lines dropped)" if dropped else ")"))
    summarise(columns, rows, args.frame)
    return 0


if __name__ == "__main__":
    sys.exit(main())
