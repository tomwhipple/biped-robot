#!/usr/bin/env python3
"""Cut a clip from camd segments (or any mp4+.pts recordings) by wall-clock time.

    tools/vision/clip.py --from 2026-11-16T16:36:10Z --to 2026-11-16T16:36:25Z -o out.mp4
        [--dir ~/cam-segments] [--source sony] [--margin 0]

Times are UTC (ISO 8601, trailing Z optional) or epoch seconds. Frames are
selected from each segment's .pts (epoch ms per frame), the pieces are
re-encoded (frame-accurate) and joined; out.pts gets the epoch ms of the frames
kept. Exit 1 when no recorded frame lies in the range.
"""
import argparse
import datetime as dt
import glob
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from camd import DEFAULT_DIR, parse_stamp  # noqa: E402


def parse_time(s):
    try:
        return float(s)
    except ValueError:
        s = s.strip().rstrip("Z")
        return dt.datetime.fromisoformat(s).replace(tzinfo=dt.timezone.utc).timestamp()


def read_pts(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                try:
                    out.append(float(line) / 1000.0)
                except ValueError:
                    pass
    return out


def find_segments(directory, name, t0, t1):
    """[(mp4, pts_times)] overlapping [t0, t1], sorted. Segment times come from the .pts.
    name=None: every <anything>.pts in the directory (hand-recorded sessions, no stamp in the name)."""
    segs = []
    for pts in sorted(glob.glob(os.path.join(directory, f"{name}_*.pts" if name else "*.pts"))):
        st = parse_stamp(pts)
        if name and (st is None or st > t1 or st < t0 - 3600):
            continue
        ts = read_pts(pts)
        mp4 = pts[:-4] + ".mp4"
        if ts and ts[-1] >= t0 and ts[0] <= t1 and os.path.exists(mp4):
            segs.append((mp4, ts))
    return segs


def make_clip(t0, t1, out, directory=DEFAULT_DIR, name="sony", ffmpeg="ffmpeg"):
    """-> number of frames written (0 = nothing in range)."""
    segs = find_segments(directory, name, t0, t1)
    pieces, kept = [], []
    with tempfile.TemporaryDirectory() as tmp:
        for i, (mp4, ts) in enumerate(segs):
            idx = [k for k, t in enumerate(ts) if t0 <= t <= t1]
            if not idx:
                continue
            a, b = idx[0], idx[-1]
            piece = os.path.join(tmp, f"p{i}.mp4")
            # frame-accurate: select by frame number, re-encode
            subprocess.run([ffmpeg, "-loglevel", "error", "-y", "-i", mp4, "-vf",
                            f"select=between(n\\,{a}\\,{b}),setpts=N/25/TB", "-an", "-c:v", "libx264",
                            "-preset", "veryfast", "-pix_fmt", "yuv420p", "-r", "25", piece], check=True)
            pieces.append(piece)
            kept += [ts[k] for k in idx]
        if not pieces:
            return 0
        lst = os.path.join(tmp, "list.txt")
        with open(lst, "w") as f:
            f.writelines(f"file '{p}'\n" for p in pieces)
        subprocess.run([ffmpeg, "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst,
                        "-c", "copy", out], check=True)
    with open(os.path.splitext(out)[0] + ".pts", "w") as f:
        f.write("# timecode format v2\n")
        f.writelines(f"{t * 1000:.3f}\n" for t in kept)
    return len(kept)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="t0", required=True)
    ap.add_argument("--to", dest="t1", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--source", default="sony")
    ap.add_argument("--margin", type=float, default=0.0, help="seconds added before and after")
    a = ap.parse_args(argv)
    n = make_clip(parse_time(a.t0) - a.margin, parse_time(a.t1) + a.margin, a.out, a.dir, a.source)
    if not n:
        print("clip: no recorded frames in that range", file=sys.stderr)
        return 1
    print(f"{a.out}: {n} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
