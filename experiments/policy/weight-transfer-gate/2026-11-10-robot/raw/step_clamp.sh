#!/usr/bin/env bash
# One pose of the clamp test: EMEET video, ONE pose, wait, settle, Sony full-res still, IMU + roll encoders.
# usage: step_clamp.sh <name> <move_ms> <settle_s> <pose args...>
set -u
cd /home/claw/code/robot
S=hw_sessions/2026-11-10/m271/clamp
NAME=$1; MS=$2; SETTLE=$3; shift 3
(INPUT_FORMAT=mjpeg SIZE=1920x1080 tools/cam_record.sh /dev/video0 $S/${NAME}_emeet $(( MS/1000 + SETTLE + 4 )) emeet > $S/${NAME}_emeet.out 2>&1 &)
sleep 2
T0=$(date -u +%H:%M:%S.%3NZ)
OUT=$(timeout 40 .venv/bin/python tools/wcli.py --timeout 20 --linger $(( MS/1000 + 1 )) pose "$@" 2>&1 | cut -c1-200)
sleep $SETTLE
IMU=$(timeout 15 .venv/bin/python tools/wcli.py imu 2>&1 | grep -E "^up" | awk '{print $2,$3,$4}')
ROLL=$(timeout 15 .venv/bin/python tools/wcli.py balance 2>&1 | grep "roll now" | awk '{print $3}')
ENC=""
for id in 52 11 51 12; do
  p=$(timeout 15 .venv/bin/python tools/wcli.py pos $id 2>&1 | awk '{for(i=1;i<=NF;i++) if($i=="pos") printf "%s/%s", $(i+1), $(i+5)}')
  ENC="$ENC id$id=$p"
done
timeout 40 python3 /home/claw/code/camera/capture.py $S/${NAME}_sony.jpg > $S/${NAME}_sony.out 2>&1
IMU2=$(timeout 15 .venv/bin/python tools/wcli.py balance 2>&1 | grep "roll now" | awk '{print $3}')
echo "$OUT" | tail -1
echo "RESULT $NAME sent $T0 | up $IMU | roll $ROLL deg (after the still: $IMU2) | roll enc pos/load:$ENC | still $(ls -la $S/${NAME}_sony.jpg 2>/dev/null | awk '{print $5}') B"
echo "- $T0 \`$NAME\`: pose $* -> $(echo "$OUT" | tail -1) | up $IMU | roll $ROLL (after still $IMU2) | enc pos/load$ENC" >> hw_sessions/2026-11-10/m271/NOTES.md
