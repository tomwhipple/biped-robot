#!/usr/bin/env bash
# Synchronized hardware-attempt capture (Tom, 2026-08-31: "video capture
# MUST be started just prior to the robot motion").
#
#   tools/record_attempt.sh <name> <arm_script args...>
#   e.g. tools/record_attempt.sh step2 --host 192.168.2.90 --phase script \
#          --script stride1 --settle 3 --duration 6 --tail 3
#
# Starts both webcam recorders, WAITS until frames are actually flowing,
# then launches arm_script -- and writes <name>_timing.txt so composites are
# aligned by arithmetic, not by motion detection. Stops the live-view
# streamer for the duration (one reader per device) and restarts it after.
#
# TIME (2026-09-02, docs/control-channel.md "Time on the wire"). Every video
# frame now carries its own capture time: tools/cam_record.sh burns the UTC
# wall clock into the picture and writes <name>_cam<d>.pts, one epoch-ms per
# frame. arm_script's log carries `utc=` (host) and `robot=` (the robot's
# SNTP clock, from the beacon) on every line. All three are the same NTP
# time base, so "which frame was the robot at when the log says X" is a
# subtraction -- to a frame, where the old mtime-of-the-mp4 hint was good to
# a second. The timing file records whether the host clock was actually
# NTP-synced when this ran, because a `utc` from an unsynced host is a
# number, not a time.
set -uo pipefail
NAME=${1:?usage: record_attempt.sh <name> <arm_script args...>}; shift
ROOT=/home/claw/code/robot
DIR=$ROOT/hw_sessions/$(date +%F)
DUR=${DUR:-30}
mkdir -p "$DIR"

pkill -f 'cam_live\.py' 2>/dev/null
sleep 1
for d in 0 2; do
  "$ROOT/tools/cam_record.sh" /dev/video$d "$DIR/${NAME}_cam$d" "$DUR" "cam$d" &
done
for f in "$DIR/${NAME}_cam0.mp4" "$DIR/${NAME}_cam2.mp4"; do
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
for d in 0 2; do
  P="$DIR/${NAME}_cam$d.pts"
  if [[ -s "$P" ]]; then
    FIRST=$(sed -n 2p "$P"); LAST=$(tail -n 1 "$P"); N=$(( $(wc -l < "$P") - 1 ))
    { echo "cam${d}_first_frame_epoch: $(echo "scale=3; $FIRST / 1000" | bc)"
      echo "cam${d}_last_frame_epoch: $(echo "scale=3; $LAST / 1000" | bc)"
      echo "cam${d}_frames: $N"
    } >> "$DIR/${NAME}_timing.txt"
    if (( d == 0 )); then
      echo "offset_hint_s: $(echo "scale=3; $T_ARM - $FIRST / 1000" | bc)" \
        >> "$DIR/${NAME}_timing.txt"
    fi
  else
    echo "cam${d}_first_frame_epoch: MISSING (no sidecar -- recorder failed?)" \
      >> "$DIR/${NAME}_timing.txt"
  fi
done
nohup "$ROOT/.venv/bin/python" "$ROOT/tools/cam_live.py" \
  > /tmp/cam_live.out 2>&1 &
echo "captured: $DIR/${NAME}_cam{0,2}.{mp4,pts}  timing: ${NAME}_timing.txt"
exit "$ARM_EC"
