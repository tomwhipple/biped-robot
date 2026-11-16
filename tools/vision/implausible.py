#!/usr/bin/env python3
"""Telemetry-only check: did the body tip far over and come back WITHOUT a fall latch?

    tools/vision/implausible.py <track_*.jsonl> [--frec S.frec | --frec-dir DIR] [--json]

A ~35 cm biped that tips past ~20 deg does not recover on its own within a few
seconds; if the IMU says it did and the firmware never latched a fall, someone
(or something) outside the robot helped. This reads the IMU roll/pitch (radians
on the wire, tools/flightrec.py) from the flight record for the run's window and
flags such excursions. Needs no camera and no tags.

The flight record can be gigabytes: records are skipped by their host-receive
stamp (the 10-byte record header) and only the run's window is decoded.
"""
import argparse
import datetime as dt
import glob
import json
import math
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import flightrec as F  # noqa: E402

REC = struct.Struct("<HQ")
FILE_MAGIC = b"FRECSESS1\n"

EXCEED_DEG = 20.0     # body tip that counts as "over"
RETURN_DEG = 10.0     # back below this = "recovered"
MAX_RECOVERY_S = 6.0  # exceed -> recovered within this = implausible
FALL_PAD_S = 2.0      # a fall latch/event this close to the excursion explains it


def run_window(track_path):
    """(utc_start, utc_end) of a track_<YYYYmmddTHHMMSS>.jsonl run. The file name
    is host LOCAL time; the last row's `t` (s) is the run's length."""
    m = re.search(r"track_(\d{8}T\d{6})", os.path.basename(track_path))
    if not m:
        raise ValueError("track file name has no track_<local time> stamp")
    t0 = dt.datetime.strptime(m.group(1), "%Y%m%dT%H%M%S").astimezone().timestamp()
    dur = 0.0
    with open(track_path) as f:
        for line in f:
            try:
                dur = max(dur, float(json.loads(line).get("t", 0)))
            except (ValueError, TypeError):
                pass
    return t0, t0 + dur


def find_frec(directory, t0, t1):
    """The .frec whose [start from its name, mtime] covers [t0, t1]."""
    for p in sorted(glob.glob(os.path.join(directory, "**", "*.frec"), recursive=True)):
        m = re.search(r"flight_(\d{8}T\d{6})Z", p)
        if not m:
            continue
        s = dt.datetime.strptime(m.group(1), "%Y%m%dT%H%M%S").replace(tzinfo=dt.timezone.utc).timestamp()
        if s <= t0 and os.path.getmtime(p) >= t1:
            return p
    return None


def read_window(frec, t0, t1):
    """Ticks and events with host-rx time in [t0, t1] -> dict of arrays + event list."""
    rows, events = [], []
    with open(frec, "rb") as f:
        if f.read(len(FILE_MAGIC)) != FILE_MAGIC:
            raise ValueError(f"{frec}: not a flight session file")
        f.readline()
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                break
            n, rx = REC.unpack(h)
            t = rx / 1e6
            if t < t0 or t > t1:
                f.seek(n, 1)
                continue
            data = f.read(n)
            try:
                p = F.decode(data)
            except F.DecodeError:
                continue
            if p["type"] == F.T_TICK:
                rows.append((t, p["roll"], p["pitch"], p["flags"], *p["up"]))
            elif p["type"] == F.T_EVENT:
                events.append(dict(t=t, kind=F.EVENT_NAMES.get(p["code"], str(p["code"])),
                                   a=p["a"], b=p["b"], text=p["text"]))
    a = np.array(rows, dtype=float).reshape(-1, 7)
    order = np.argsort(a[:, 0], kind="stable")
    a = a[order]
    return dict(t=a[:, 0], roll_deg=np.degrees(a[:, 1]), pitch_deg=np.degrees(a[:, 2]),
                flags=a[:, 3].astype(int), up=a[:, 4:7]), events


def tilt_deg(d):
    """Total tilt from vertical, from the gravity-up vector (z of `up` = cos tilt)."""
    return np.degrees(np.arccos(np.clip(d["up"][:, 2], -1, 1)))


def find_excursions(d, events, exceed=EXCEED_DEG, back=RETURN_DEG, max_s=MAX_RECOVERY_S):
    """-> list of dicts for tip-overs that recovered within max_s with no fall.

    An excursion starts when max(|roll|,|pitch|) > exceed and ends when it is
    < back. Each is returned with `implausible` True when it ended within max_s
    and no fall latch (TF_FALLEN tick flag, or EV_FALL event) lies within
    FALL_PAD_S of it.
    """
    t = d["t"]
    big = np.maximum(np.abs(d["roll_deg"]), np.abs(d["pitch_deg"]))
    fall_t = [e["t"] for e in events if e["kind"] == "fall"]
    fall_t += list(t[(d["flags"] & F.TF_FALLEN) != 0])
    out, i, n = [], 0, len(t)
    while i < n:
        if big[i] <= exceed:
            i += 1
            continue
        s = i
        j = i
        while j < n and big[j] > back:     # still tipped (or on the way back)
            j += 1
        seg = slice(s, j)
        k = s + int(np.argmax(big[seg]))
        t_end = t[j] if j < n else None
        fell = any(t[s] - FALL_PAD_S <= ft <= (t_end or t[-1]) + FALL_PAD_S for ft in fall_t)
        dur = (t_end - t[s]) if t_end else None
        out.append(dict(t_start=float(t[s]), t_peak=float(t[k]), t_end=None if t_end is None else float(t_end),
                        peak_deg=float(big[k]), peak_roll_deg=float(d["roll_deg"][k]),
                        peak_pitch_deg=float(d["pitch_deg"][k]), peak_tilt_deg=float(tilt_deg(d)[k]),
                        over_s=dur, fall_latched=bool(fell),
                        implausible=bool(dur is not None and dur <= max_s and not fell)))
        i = j
    return out


def exact_start(events, t0):
    """The run's first command is `cal show`: its host-rx time in the flight record's events is
    the run's t=0 to ~10 ms (the file name only has the second, in local time)."""
    c = [e["t"] for e in events if e["kind"] == "cmd" and e["text"].startswith("cal show") and abs(e["t"] - t0) < 5]
    return min(c, key=lambda t: abs(t - t0)) if c else t0


def check_run(track_path, frec=None, frec_dir=None, pad=10.0, data=False):
    t0, t1 = run_window(track_path)
    frec = frec or find_frec(frec_dir or "hw_sessions", t0, t1)
    if frec is None:
        return dict(source=None, window_utc=[t0, t1], excursions=[], implausible_recovery=None,
                    note="no flight record covers this run")
    d, events = read_window(frec, t0 - pad, t1 + pad)
    t0x = exact_start(events, t0)
    t0, t1 = t0x, t1 + (t0x - t0)
    ex = find_excursions(d, events)
    r = dict(source=frec, window_utc=[t0, t1], n_ticks=int(len(d["t"])),
                max_tip_deg=float(np.max(np.maximum(np.abs(d["roll_deg"]), np.abs(d["pitch_deg"])))) if len(d["t"]) else None,
                excursions=ex, implausible_recovery=[e for e in ex if e["implausible"]])
    if data:
        r["_imu"], r["_events"] = d, events
    return r


def _iso(t):
    return None if t is None else dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track")
    ap.add_argument("--frec")
    ap.add_argument("--frec-dir", default="hw_sessions")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    r = check_run(a.track, a.frec, a.frec_dir)
    if a.json:
        print(json.dumps(r, indent=1))
        return 0
    print(f"{os.path.basename(a.track)}  frec={r['source'] and os.path.basename(r['source'])}  "
          f"max tip {r.get('max_tip_deg') and round(r['max_tip_deg'], 1)} deg")
    for e in r["excursions"]:
        print(f"  tip {e['peak_deg']:.0f} deg at {_iso(e['t_peak'])} ({_iso(e['t_start'])}..{_iso(e['t_end'])}) "
              f"fall={e['fall_latched']} -> {'IMPLAUSIBLE RECOVERY' if e['implausible'] else 'ok/explained'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
