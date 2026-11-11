#!/usr/bin/env bash
# One pose of the scale ladder: record both cameras, send ONE pose, wait for its done line, read IMU + roll encoders.
# usage: step.sh <name> <move_ms> <pose args...>
set -u
cd /home/claw/code/robot
S=hw_sessions/2026-11-10/m271
NAME=$1; MS=$2; shift 2
SECS=$(( MS/1000 + 7 ))
(INPUT_FORMAT=mjpeg tools/cam_record.sh /dev/video0 $S/${NAME}_emeet $SECS emeet > $S/${NAME}_emeet.out 2>&1 &)
(tools/sony_record.sh $S/${NAME}_sony $SECS sony > $S/${NAME}_sony.out 2>&1 &)
sleep 2.5
T0=$(date -u +%H:%M:%S.%3NZ)
OUT=$(timeout 40 .venv/bin/python tools/wcli.py --timeout 20 --linger $(( MS/1000 + 2 )) pose "$@" 2>&1 | cut -c1-200)
T1=$(date -u +%H:%M:%S.%3NZ)
IMU=$(timeout 15 .venv/bin/python tools/wcli.py imu 2>&1 | grep -E "^up" | awk '{print $2,$3,$4}')
ROLL=$(timeout 15 .venv/bin/python tools/wcli.py balance 2>&1 | grep "roll now" | awk '{print $3}')
ENC=""
for id in 52 11 51 12; do
  p=$(timeout 15 .venv/bin/python tools/wcli.py pos $id 2>&1 | awk '{for(i=1;i<=NF;i++) if($i=="pos") printf "%s/%s", $(i+1), $(i+5)}')
  ENC="$ENC id$id=$p"
done
echo "$OUT"
echo "RESULT $NAME sent $T0 done $T1 | up $IMU | roll $ROLL deg | roll enc pos/load:$ENC"
echo "- $T0 \`$NAME\`: pose $* -> $(echo "$OUT" | tail -1) | up $IMU | roll $ROLL deg | enc pos/load$ENC" >> $S/NOTES.md
