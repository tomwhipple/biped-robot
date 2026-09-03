#!/usr/bin/env bash
# Record one webcam with the wall-clock time on every frame.
#
#   tools/cam_record.sh <device> <out_base> <seconds> [label]
#   e.g. tools/cam_record.sh /dev/video0 hw_sessions/2026-09-02/step2_cam0 30 cam0
#
# Writes TWO files:
#   <out_base>.mp4   640x480 H.264, timestamps starting at 0 so every player
#                    works, with "<label> 2026-09-02T22:27:55.605Z" drawn in
#                    the top-left corner of EVERY frame -- the frame carries
#                    its own time wherever it ends up (a filmstrip, a Slack
#                    screenshot, a ghost composite).
#   <out_base>.pts   one line per frame, the frame's capture time in
#                    milliseconds since the Unix epoch (mkvtimestamp_v2:
#                    "# timecode format v2" then integers). Frame i of the
#                    mp4 (0-based) is line i+2: the header, then one line
#                    per frame. This is the file tooling joins against a
#                    telemetry log's `utc` column.
#
# WHERE THE TIME COMES FROM. The kernel stamps each V4L2 buffer when the DMA
# completes, on CLOCK_MONOTONIC; ffmpeg's `-ts mono2abs` converts that to
# CLOCK_REALTIME, which chrony disciplines to NTP on mira (measured 0.1 ms).
# So the burned-in text is the capture instant to about a millisecond, not
# the moment the frame was encoded or written. `-copyts` keeps those epoch
# timestamps through the filter graph (drawtext reads them as `pts`/`t`);
# setpts=PTS-STARTPTS then rebases the mp4 branch only. Verified 2026-09-02
# on /dev/video0 against `date`.
#
# The duration limit sits on the INPUT (`-t` before `-i`): as an output
# option it would apply to the mp4 alone and the .pts branch would record
# forever (bitten while testing this).
#
# SELF-TEST without a camera: a device of the form `lavfi:<epoch_seconds>`
# feeds a synthetic test pattern whose first frame is stamped at that epoch,
# so the burn-in and the sidecar can be checked by arithmetic:
#   tools/cam_record.sh lavfi:1788390475.605401 /tmp/synth 1 cam0
#
# NOTE a v4l2 device serves one reader: stop tools/cam_live.py first, or this
# fails with "Device or resource busy".
set -uo pipefail
DEV=${1:?usage: cam_record.sh <device> <out_base> <seconds> [label]}
OUT=${2:?out_base}
SECS=${3:?seconds}
LABEL=${4:-$(basename "$OUT")}
SIZE=${SIZE:-640x480}
FPS=${FPS:-30}

# Fontconfig can find a monospace face on its own, but naming the file makes
# the burn-in identical on every host that has the package.
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf
if [[ -f "$FONT" ]]; then FONTOPT="fontfile=$FONT"; else FONTOPT="font=monospace"; fi

# drawtext escaping, from the inside out: a ':' that separates a %{...}
# expansion's arguments is '\:', and a ':' that must SURVIVE inside the
# strftime format handed to gmtime is '\\\:' (ffmpeg 4.4 -- '\:' splits the
# format into extra arguments, '\\:' leaves a stray '%'; both tried, this is
# the one that renders 2026-09-02T23:07:55.605). The text sits in single
# quotes so the shell hands the backslashes over untouched. The milliseconds
# come from eif(mod(t*1000,1000)) because %{pts:gmtime} stops at whole
# seconds in this ffmpeg.
STAMP='%{pts\:gmtime\:0\:%Y-%m-%dT%H\\\:%M\\\:%S}.%{eif\:mod(t*1000\,1000)\:d\:3}Z'
DT="drawtext=$FONTOPT:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.6:x=6:y=6:text='$LABEL $STAMP'"

if [[ "$DEV" == lavfi:* ]]; then
  # settb first: testsrc's 1/30 s timebase would round the epoch offset to
  # the nearest frame period; a V4L2 device's timebase is already 1 us.
  INPUT=(-f lavfi -t "$SECS" -i "testsrc=size=$SIZE:rate=$FPS")
  PRE="settb=AVTB,setpts=PTS+${DEV#lavfi:}/TB,"
else
  INPUT=(-f v4l2 -ts mono2abs -framerate "$FPS" -video_size "$SIZE" -t "$SECS" -i "$DEV")
  PRE=""
fi

# -vsync passthrough: ffmpeg's default video sync renumbers frames at a
# constant rate, which would turn the sidecar into frame_index/30 dressed up
# as epoch time (and the encoder's default 1/30 s timebase would round the
# real capture instants to the nearest frame period). Passthrough keeps the
# kernel's instants; the 1 us timebase keeps their resolution. Both checked
# with the lavfi self-test: 605, 639, 672 ms for frames stamped 605.4,
# 638.7, 672.1.
exec ffmpeg -loglevel error -y "${INPUT[@]}" -copyts -vsync passthrough \
  -filter_complex "[0:v]${PRE}${DT},split[a][b];[a]setpts=PTS-STARTPTS[mp4]" \
  -map "[mp4]" -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT.mp4" \
  -map "[b]" -c:v rawvideo -enc_time_base 1/1000000 -f mkvtimestamp_v2 "$OUT.pts"
