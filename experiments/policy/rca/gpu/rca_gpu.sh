#!/usr/bin/env bash
# 2026-11-01 first-stand RCA daytime GPU window: robustness teachers warm-started from
# robot_legs_walk14_s1 (the deployed student's teacher), each distilled to the stock (128,128)
# student. Holds the night-runner lock; vision paused, idle ollama models unloaded; hard stop
# well before 21:45 local. Usage: rca_gpu.sh "<out1>|<extra args1>" "<out2>|<extra args2>" ...
set -uo pipefail
BASE=$HOME/code/robot-mjx; N=$BASE/night; PY=$BASE/.venv/bin/python
LOG=$HOME/code/robot-rca/experiments/policy/rca/gpu/rca_gpu.log; exec >> "$LOG" 2>&1
exec 9>"$N/.runner.lock"; flock -n 9 || { echo "!!! $(date) night runner lock held; not launching"; exit 1; }
echo "=== $(date) rca_gpu start ==="
git -C "$BASE" fetch -q --filter=blob:none origin main && git -C "$BASE" reset -q --hard FETCH_HEAD
echo "repo @ main $(git -C "$BASE" rev-parse --short HEAD)"
"$N/vision_pause.sh" pause
for M in $(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}'); do ollama stop "$M" 2>/dev/null; echo "unloaded $M"; done
trap '"$N/vision_pause.sh" resume; echo "=== $(date) rca_gpu end (trap) ==="' EXIT
COMMON="--robot --xml bimo_biped_v6legs.xml --precision --action-map full --ctrl-within-stops --act-lag 2,12 --servo-kp-scale asbuilt --quantize --mjx-impl warp --floor-contacts feet --family loco --walk-submix 0.2,0.2 --cmd-v-range 0.05,0.7 --turn-emph --mimic-lift 0.07 --swing-height 0.08 --w-mimic 1.5 --feet-phase-s2 0.002 --w-feet-phase 2.0 --joint-frictionloss 0.35 --roll-lash-deg 0.5,2.0 --backlash-deg 0,0.25 --zero-offset-deg 2 --torso-mass-range 0.4 --torso-com-y 0.015 --imu-mount-deg 3 --eval-scenarios walk_steps_L,walk_steps_R --stop-grace-s 0.4 --w-low-swing 10 --init-from robot_legs_walk14_s1 --seed 1"
DEADLINE=$(date -d "today 21:20" +%s)   # local: nothing starts that cannot end before 21:45
for spec in "$@"; do
  OUT=${spec%%|*}; EXTRA=${spec#*|}
  now=$(date +%s); left=$((DEADLINE - now))
  (( left > 1800 )) || { echo "$(date +%H:%M) skip $OUT: ${left}s left"; continue; }
  tt=$(( left < 2400 ? left - 900 : 2400 ))
  echo "$(date +%H:%M) teacher $OUT start (timeout ${tt}s): $EXTRA"
  cd "$BASE/sim/mjx"
  XLA_PYTHON_CLIENT_PREALLOCATE=false timeout "$tt" "$PY" train_mjx.py --out "$OUT" $COMMON $EXTRA > "$BASE/sim/runs/train_rca_$OUT.log" 2>&1
  echo "$(date +%H:%M) teacher $OUT exit $?"
  now=$(date +%s); left=$((DEADLINE - now)); db=$(( left - 300 )); (( db > 2400 )) && db=2400
  (( db > 600 )) || { echo "$(date +%H:%M) skip distill $OUT: ${left}s left"; continue; }
  cd "$BASE"
  echo "$(date +%H:%M) distill ${OUT}_s128r24 start (budget ${db}s)"
  DISTILL_BUDGET_S=$db DISTILL_NO_COLLECT=1 XLA_PYTHON_CLIENT_MEM_FRACTION=0.80 timeout $((db + 600)) \
      bash infra/night_distill.sh "$OUT" "${OUT}_s128r24" > "$BASE/sim/runs/cmd_rca_${OUT}_s128r24.log" 2>&1
  echo "$(date +%H:%M) distill ${OUT}_s128r24 exit $?"
done
echo "=== $(date) rca_gpu done ==="
