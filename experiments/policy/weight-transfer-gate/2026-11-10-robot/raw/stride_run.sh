#!/usr/bin/env bash
# Repeat of the 2026-11-09 prec16 run: arm_script --phase script --script stride1_02s --settle 3 --duration 5 --tail 0.5
# (phases read off hw_sessions/2026-11-09/prec16_run1_arm.log), from the deploy/prec16-f075 worktree, both cameras recording.
# usage: stride_run.sh <name>
set -u
NAME=${1:?name}
S=/home/claw/code/robot/hw_sessions/2026-11-10/m271
cd /home/claw/code/robot
(INPUT_FORMAT=mjpeg tools/cam_record.sh /dev/video0 $S/${NAME}_emeet 22 emeet > $S/${NAME}_emeet.out 2>&1 &)
(tools/sony_record.sh $S/${NAME}_sony 22 sony > $S/${NAME}_sony.out 2>&1 &)
n=0; until [[ -s $S/${NAME}_emeet.mp4 && -s $S/${NAME}_sony.mp4 ]] || (( n > 40 )); do sleep 0.2; n=$((n+1)); done
sleep 1
cd /home/claw/code/robot-deploy7
/home/claw/code/robot/.venv/bin/python link/arm_script.py --host 192.168.2.90 --phase script --script stride1_02s \
   --settle 3 --duration 5 --tail 0.5 2>&1 | tee $S/${NAME}_arm.log | grep -E "^==|state change|robot says" | cut -c1-200
echo "arm_script exit ${PIPESTATUS[0]}"
