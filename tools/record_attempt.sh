#!/usr/bin/env bash
# Synchronized hardware-attempt capture (Tom, 2026-08-31: "video capture
# MUST be started just prior to the robot motion").
#
#   tools/record_attempt.sh <name> <arm_script args...>
#   e.g. tools/record_attempt.sh step2 --host <robot-ip> --phase script \
#          --script stride1 --settle 3 --duration 6 --tail 3
#
# Starts the webcam recorder(s) (and the Sony with SONY=1), WAITS until frames are actually flowing,
# then launches arm_script -- and writes <name>_timing.txt so composites are
# aligned by arithmetic, not by motion detection. Stops the live-view
# streamer for the duration (one reader per device) and restarts it after.
#
# TIME (2026-09-02, docs/control-channel.md "Time on the wire"). Every video
# frame now carries its own capture time: tools/cam_record.sh burns the UTC
# wall clock into the picture and writes <name>_cam<N>.pts, one epoch-ms per
# frame. arm_script's log carries `utc=` (host) and `robot=` (the robot's
# SNTP clock, from the beacon) on every line. All three are the same NTP
# time base, so "which frame was the robot at when the log says X" is a
# subtraction -- to a frame, where the old mtime-of-the-mp4 hint was good to
# a second. The timing file records whether the host clock was actually
# NTP-synced when this ran, because a `utc` from an unsynced host is a
# number, not a time.
set -uo pipefail
NAME=${1:?usage: record_attempt.sh <name> <arm_script args...>}; shift
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DIR=$ROOT/hw_sessions/$(date +%F)
DUR=${DUR:-30}
mkdir -p "$DIR"

pkill -f 'cam_live\.py' 2>/dev/null
sleep 1
# Cameras: CAM_DEVICES is a space-separated list (default: /dev/video0 only,
# the floor-level camera). Each file is <name>_cam<N>, N = the /dev/videoN
# number. SONY=1 adds a leg through tools/sony_record.sh (<name>_sony.*).
# SIZE / FPS / INPUT_FORMAT pass through to cam_record.sh (default 1280x720
# MJPEG at 30 fps; the C960 does SIZE=1920x1080).
CAM_DEVICES=${CAM_DEVICES:-/dev/video0}
LEGS=()   # leg names: cam0, cam2, sony
for dev in $CAM_DEVICES; do
  leg=cam${dev##*[!0-9]}
  [[ "$dev" == lavfi:* ]] && leg=cam${#LEGS[@]}   # self-test sources
  LEGS+=("$leg")
  "$ROOT/tools/cam_record.sh" "$dev" "$DIR/${NAME}_$leg" "$DUR" "$leg" &
done
if [[ "${SONY:-0}" == 1 ]]; then
  LEGS+=(sony)
  "$ROOT/tools/sony_record.sh" "$DIR/${NAME}_sony" "$DUR" sony &
fi
for leg in "${LEGS[@]}"; do
  f="$DIR/${NAME}_$leg.mp4"
  n=0
  until [[ -s "$f" ]] || (( n > 50 )); do sleep 0.2; n=$((n+1)); done
done
sleep 0.5
HOST_NTP=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)
T_ARM=$(date +%s.%N)
"$ROOT/.venv/bin/python" "$ROOT/link/arm_script.py" "$@" \
  2>&1 | tee "$DIR/${NAME}_arm.log"
ARM_EC=${PIPESTATUS[0]}
T_END=$(date +%s.%N)
{ echo "host_ntp_synced: $HOST_NTP"
  echo "arm_launch_epoch: $T_ARM"
  echo "arm_exit_epoch: $T_END"
  echo "arm_exit_code: $ARM_EC"
} > "$DIR/${NAME}_timing.txt"
wait   # let the recorders run out their -t
# Per-camera first/last frame instants from the sidecars (ms -> s), and the
# rec->arm offset from the real first-frame time rather than a file mtime.
FIRSTLEG=${LEGS[0]}
for leg in "${LEGS[@]}"; do
  P="$DIR/${NAME}_$leg.pts"
  if [[ -s "$P" ]]; then
    FIRST=$(sed -n 2p "$P"); LAST=$(tail -n 1 "$P"); N=$(( $(wc -l < "$P") - 1 ))
    { echo "${leg}_first_frame_epoch: $(echo "scale=3; $FIRST / 1000" | bc)"
      echo "${leg}_last_frame_epoch: $(echo "scale=3; $LAST / 1000" | bc)"
      echo "${leg}_frames: $N"
    } >> "$DIR/${NAME}_timing.txt"
    if [[ "$leg" == "$FIRSTLEG" ]]; then
      echo "offset_hint_s: $(echo "scale=3; $T_ARM - $FIRST / 1000" | bc)" \
        >> "$DIR/${NAME}_timing.txt"
    fi
  else
    echo "${leg}_first_frame_epoch: MISSING (no sidecar -- recorder failed?)" \
      >> "$DIR/${NAME}_timing.txt"
  fi
done
nohup "$ROOT/.venv/bin/python" "$ROOT/tools/cam_live.py" \
  > /tmp/cam_live.out 2>&1 &
echo "captured: $DIR/${NAME}_{${LEGS[*]// /,}}.{mp4,pts}  timing: ${NAME}_timing.txt"
exit "$ARM_EC"
