#!/usr/bin/env bash
# Synchronized hardware-attempt capture (Tom, 2026-08-31: "video capture
# MUST be started just prior to the robot motion").
#
#   tools/record_attempt.sh <name> <arm_script args...>
#   e.g. tools/record_attempt.sh step2 --host 192.168.2.90 --phase script \
#          --script stride1 --settle 3 --duration 6 --tail 3
#
# Starts both webcam recorders, WAITS until frames are actually flowing,
# then launches arm_script -- and writes <name>_timing.txt with the
# rec->arm offset so composites are aligned by arithmetic, not by motion
# detection. Stops the live-view streamer for the duration (one reader per
# device) and restarts it after.
set -uo pipefail
NAME=${1:?usage: record_attempt.sh <name> <arm_script args...>}; shift
ROOT=/home/claw/code/robot
DIR=$ROOT/hw_sessions/$(date +%F)
DUR=${DUR:-30}
mkdir -p "$DIR"

pkill -f 'cam_live\.py' 2>/dev/null
sleep 1
for d in 0 2; do
  ffmpeg -y -loglevel error -f v4l2 -video_size 640x480 -t "$DUR" \
    -i /dev/video$d "$DIR/${NAME}_cam$d.mp4" &
done
for f in "$DIR/${NAME}_cam0.mp4" "$DIR/${NAME}_cam2.mp4"; do
  n=0
  until [[ -s "$f" ]] || (( n > 50 )); do sleep 0.2; n=$((n+1)); done
done
sleep 0.5
T_REC=$(stat -c %Y "$DIR/${NAME}_cam0.mp4")
T_ARM=$(date +%s.%N)
"$ROOT/.venv/bin/python" "$ROOT/link/arm_script.py" "$@" \
  2>&1 | tee "$DIR/${NAME}_arm.log"
ARM_EC=${PIPESTATUS[0]}
T_END=$(date +%s.%N)
{ echo "rec_file_epoch: $T_REC"
  echo "arm_launch_epoch: $T_ARM"
  echo "arm_exit_epoch: $T_END"
  echo "arm_exit_code: $ARM_EC"
  echo "offset_hint_s: $(echo "$T_ARM - $T_REC" | bc)"
} > "$DIR/${NAME}_timing.txt"
wait   # let the recorders run out their -t
nohup "$ROOT/.venv/bin/python" "$ROOT/tools/cam_live.py" \
  > /tmp/cam_live.out 2>&1 &
echo "captured: $DIR/${NAME}_cam{0,2}.mp4  timing: ${NAME}_timing.txt"
exit "$ARM_EC"
