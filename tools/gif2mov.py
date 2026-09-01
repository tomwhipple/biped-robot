"""Convert animation GIFs to QuickTime-friendly .mov (H.264).

Why: macOS Preview/QuickLook doesn't animate GIFs (Preview shows frames as
pages), and some of our early GIFs lack the loop flag so they play once and
freeze. H.264 .mov plays natively everywhere on the Mac.

Run:  .venv/bin/python tools/gif2mov.py [paths-or-globs...]
      no args = convert every .gif under sim/runs/ and cad/renders/

Writes a .mov next to each .gif (skips ones already newer than the gif).
Frame timing is taken from the GIF itself; odd dimensions are rounded down
to even (H.264 requirement).
"""
import glob
import os
import subprocess
import sys

import imageio_ffmpeg

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()


def convert(gif):
    mov = os.path.splitext(gif)[0] + ".mov"
    if os.path.exists(mov) and os.path.getmtime(mov) >= os.path.getmtime(gif):
        print(f"up-to-date  {mov}")
        return
    cmd = [
        FFMPEG, "-y", "-i", gif,
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", mov,
    ]
    subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    print(f"wrote       {mov}  ({os.path.getsize(mov) // 1024} KB "
          f"from {os.path.getsize(gif) // 1024} KB gif)")


def main():
    if len(sys.argv) > 1:
        gifs = [p for a in sys.argv[1:] for p in glob.glob(a, recursive=True)]
    else:
        gifs = (glob.glob(os.path.join(ROOT, "sim", "runs", "**", "*.gif"),
                          recursive=True)
                + glob.glob(os.path.join(ROOT, "cad", "renders", "*.gif")))
    if not gifs:
        raise SystemExit("no gifs found")
    for g in sorted(gifs):
        try:
            convert(g)
        except subprocess.TimeoutExpired:
            print(f"timeout converting {g} -- skipping", file=sys.stderr)


if __name__ == "__main__":
    main()
