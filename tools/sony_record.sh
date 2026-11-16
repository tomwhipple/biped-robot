#!/usr/bin/env bash
# Record the Sony A7 III's live view (USB, PC Remote mode) with the wall-clock
# time on every frame -- tools/cam_record.sh for the tethered camera.
#
#   tools/sony_record.sh <out_base> [seconds] [label]
#   e.g. tools/sony_record.sh hw_sessions/2026-10-23/hold/cam 0 sony
#
# seconds 0 or omitted = until stopped: `kill -TERM <pid>` (the script's pid)
# ends liveview.py cleanly, ffmpeg then sees EOF and finishes the mp4. Never
# kill -9 the liveview process: it leaves the A7 III wedged until power-cycled
# (~/code/camera/capture.py).
#
# Writes <out_base>.mp4 (1024x680 H.264, 15-29 fps, "<label> <UTC>" burned in)
# and <out_base>.pts (mkvtimestamp_v2, ms since epoch per frame), same layout
# as cam_record.sh.
#
# WHERE THE TIME COMES FROM: unlike a V4L2 buffer there is no capture stamp;
# -use_wallclock_as_timestamps (image2pipe; the raw mjpeg demuxer ignores it
# and invents 25 fps) stamps each JPEG when ffmpeg reads it from the
# pipe, i.e. capture + the camera's live-view latency + one USB transfer
# (tens of ms, not measured). Good for "which frame shows the move", not for
# millisecond alignment.
#
# The camera serves one reader: the mediamtx `camera` path
# (~/code/camera/camera-stream.sh) takes it whenever someone watches the
# stream, and then camera_init fails. This script therefore REFUSES (exit 3,
# nothing written) when the camera is held -- by tools/vision/camd.py (its
# flock), or by any other liveview.py -- and removes its files and exits 1 if
# no frame arrives within 15 s. The checks use a flock and an anchored pgrep
# pattern (^ + the liveview path); neither can match this script's own command line.
set -uo pipefail
OUT=${1:?usage: sony_record.sh <out_base> [seconds] [label]}
SECS=${2:-0}
LABEL=${3:-sony}
LIVEVIEW=${LIVEVIEW:-$HOME/code/camera/liveview.py}
[[ "$SECS" == 0 ]] && SECS=""

CAMD_LOCK=${CAMD_DIR:-$HOME/cam-segments}/sony.lock
if [[ -e "$CAMD_LOCK" ]] && ! flock -n "$CAMD_LOCK" true; then
  echo "sony_record: camd owns the camera (lock $CAMD_LOCK, pid $(cat "$CAMD_LOCK" 2>/dev/null)); cut a clip with tools/vision/clip.py instead. Not recording." >&2
  exit 3
fi
if HOLDERS=$(pgrep -af "^/usr/bin/python3 -u $LIVEVIEW"); then
  echo "sony_record: the camera is busy, another liveview reader is running:" >&2
  echo "$HOLDERS" >&2
  echo "sony_record: not recording (no file written)." >&2
  exit 3
fi

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf
if [[ -f "$FONT" ]]; then FONTOPT="fontfile=$FONT"; else FONTOPT="font=monospace"; fi
# escaping as in cam_record.sh
STAMP='%{pts\:gmtime\:0\:%Y-%m-%dT%H\\\:%M\\\:%S}.%{eif\:mod(t*1000\,1000)\:d\:3}Z'
DT="drawtext=$FONTOPT:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6:x=8:y=8:text='$LABEL $STAMP'"

# The USB node is group plugdev, which long-lived shells may predate: sg.
# setsid + trap so a TERM to this script reaches liveview.py (sg forwards none).
# LATEST=<file.jpg>: also keep that file as the newest frame (10 per second, written whole and renamed),
# for a program that wants to look while the recording runs -- the camera serves one reader.
SPLIT="split[a][b]"; LATEST_OUT=()
if [[ -n "${LATEST:-}" ]]; then
  SPLIT="split=3[a][b][c];[c]fps=10[jpg]"
  LATEST_OUT=(-map "[jpg]" -c:v mjpeg -q:v 3 -f image2 -update 1 -atomic_writing 1 "$LATEST")
fi
setsid sg plugdev -c "exec /usr/bin/python3 -u '$LIVEVIEW' $SECS" | \
  ffmpeg -loglevel error -y -use_wallclock_as_timestamps 1 -f image2pipe -c:v mjpeg -i - \
    -copyts -vsync passthrough \
    -filter_complex "[0:v]${DT},${SPLIT};[a]setpts=PTS-STARTPTS[mp4]" \
    -map "[mp4]" -c:v libx264 -preset veryfast -pix_fmt yuv420p -movflags +frag_keyframe+empty_moov "$OUT.mp4" \
    -map "[b]" -c:v rawvideo -enc_time_base 1/1000000 -f mkvtimestamp_v2 "$OUT.pts" \
    "${LATEST_OUT[@]}" &
FF=$!
trap 'pkill -TERM -f "python3 -u $LIVEVIEW" 2>/dev/null; wait $FF; exit 0' INT TERM HUP
# no frame within 15 s = the camera never started: leave no empty file behind
nframes() { local n; n=$(grep -vc '^#' "$OUT.pts" 2>/dev/null) || true; echo "${n:-0}"; }
for _ in $(seq 30); do
  [[ $(nframes) -gt 0 ]] && break
  kill -0 $FF 2>/dev/null || break
  sleep 0.5
done
if [[ $(nframes) -eq 0 ]]; then
  pkill -TERM -f "python3 -u $LIVEVIEW" 2>/dev/null
  wait $FF 2>/dev/null
  rm -f "$OUT.mp4" "$OUT.pts"
  echo "sony_record: no frame from the camera (busy, off or not in PC Remote mode); no file written." >&2
  exit 1
fi
wait $FF
