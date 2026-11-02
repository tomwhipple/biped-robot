#!/usr/bin/env bash
# 2026-11-02 stand-still daytime GPU window: short fine-tunes of robot_legs_walk16_act (the
# actuator-DR legs teacher whose student passes x1.5 on the measured actuator) with the gait
# clock FROZEN at a plain stand (--clock-freeze-stand) and a stand-gated joint-velocity
# penalty (--w-still), each distilled to the stock (128,128) student. Holds the night-runner
# lock; vision paused, idle ollama models unloaded; nothing starts that cannot end by 21:45.
#   stand_gpu.sh "<out1>|<extra args1>" "<out2>|<extra args2>" ...
set -uo pipefail
BASE=$HOME/code/robot-mjx; N=$BASE/night; PY=$BASE/.venv/bin/python
LOG=${LOG:-$HOME/code/robot-stand-still/experiments/policy/stand-still/gpu/stand_gpu.log}; exec >> "$LOG" 2>&1
exec 9>"$N/.runner.lock"; flock -w 7200 9 || { echo "!!! $(date) runner lock still held after 2 h; not launching"; exit 1; }
BR=${BR:-main}
echo "=== $(date) stand_gpu start ==="
git -C "$BASE" fetch -q origin "$BR" && git -C "$BASE" reset -q --hard FETCH_HEAD || { echo "!!! fetch $BR failed"; exit 1; }
echo "repo @ $BR $(git -C "$BASE" rev-parse --short HEAD)"
"$N/vision_pause.sh" pause
for M in $(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}'); do ollama stop "$M" 2>/dev/null; echo "unloaded $M"; done
trap '"$N/vision_pause.sh" resume; echo "=== $(date) stand_gpu end (trap) ==="' EXIT
# robot_legs_walk16_act's recipe (night queue 1101214001), minus out/steps/init/seed and the
# dead-time draw: --act-delay-types goes in each job's extra args (the STS3215 dead time is a RAM
# register -- reg 85 = 50 factory ~70 ms, reg 85 = 0 ~20 ms, Tom 2026-11-02 -- so the draw must cover both)
COMMON="--robot --xml bimo_biped_v6legs.xml --precision --action-map full --ctrl-within-stops --servo-kp-scale asbuilt --quantize --mjx-impl warp --floor-contacts feet --family loco --walk-submix 0.2,0.2 --cmd-v-range 0.05,0.7 --turn-emph --mimic-lift 0.07 --swing-height 0.08 --w-mimic 1.5 --feet-phase-s2 0.002 --w-feet-phase 2.0 --roll-lash-deg 0.5,2.0 --backlash-deg 0,0.25 --zero-offset-deg 2 --torso-mass-range 0.4 --torso-com-y 0.015 --imu-mount-deg 3 --eval-scenarios walk_steps_L,walk_steps_R --stop-grace-s 0.4 --w-low-swing 10 --joint-frictionloss 0.35 --act-lag 2,8 --joint-frictionloss-range 0.1,0.4 --gyro-delay-max 1 --w-action-rate 0.3 --servo-kd-range 0.12,1.0 --servo-kp-range 0.6,1.2 --gyro-gain-range 0.7,1.6 --init-from robot_legs_walk16_act"
DEADLINE=$(date -d "today 21:30" +%s)
for spec in "$@"; do
  OUT=${spec%%|*}; EXTRA=${spec#*|}
  now=$(date +%s); left=$((DEADLINE - now))
  (( left > 3600 )) || { echo "$(date +%H:%M) skip $OUT: ${left}s left"; continue; }
  tt=$(( left - 2700 < 4200 ? left - 2700 : 4200 ))
  echo "$(date +%H:%M) teacher $OUT start (timeout ${tt}s): $EXTRA"
  cd "$BASE/sim/mjx"
  XLA_PYTHON_CLIENT_PREALLOCATE=false timeout "$tt" "$PY" train_mjx.py --out "$OUT" $COMMON $EXTRA > "$BASE/sim/runs/train_stand_$OUT.log" 2>&1
  echo "$(date +%H:%M) teacher $OUT exit $?"
  now=$(date +%s); left=$((DEADLINE - now)); db=$(( left - 300 )); (( db > 2400 )) && db=2400
  (( db > 600 )) || { echo "$(date +%H:%M) skip distill $OUT: ${left}s left"; continue; }
  cd "$BASE"
  echo "$(date +%H:%M) distill ${OUT}_s128r24 start (budget ${db}s)"
  DISTILL_BUDGET_S=$db DISTILL_NO_COLLECT=1 XLA_PYTHON_CLIENT_MEM_FRACTION=0.80 timeout $((db + 600)) \
      bash infra/night_distill.sh "$OUT" "${OUT}_s128r24" > "$BASE/sim/runs/cmd_stand_${OUT}_s128r24.log" 2>&1
  echo "$(date +%H:%M) distill ${OUT}_s128r24 exit $?"
done
echo "=== $(date) stand_gpu done ==="
