#!/usr/bin/env bash
# 2026-11-02 shaper-train daytime GPU window: fine-tunes of robot_legs_walk16_act with the
# firmware's C2 shaper in the loop (--fw-shaper), the servo's own first-order rise as the
# act-lag, the fast-servo dead time, the stand clock coasting to pi/2 and the stand squeeze
# penalty, each distilled to the stock (128,128) student. Runs from THIS worktree (the night
# clone is not touched); holds the night-runner lock; vision paused, idle ollama models
# unloaded; nothing starts that cannot end by 21:45.
#   shaper_gpu.sh "<out1>|<extra args1>" "<out2>|<extra args2>" ...
#   "<out>|DISTILL_ONLY" distills an existing teacher (e.g. one cut short) without training
set -uo pipefail
WT=$(cd "$(dirname "$0")/../../../.." && pwd)
N=$HOME/code/robot-mjx/night; PY=$HOME/code/robot-mjx/.venv/bin/python
LOG=${LOG:-$WT/experiments/policy/shaper-train/gpu/shaper_gpu.log}; exec >> "$LOG" 2>&1
exec 9>"$N/.runner.lock"; flock -w 7200 9 || { echo "!!! $(date) runner lock still held after 2 h; not launching"; exit 1; }
echo "=== $(date) shaper_gpu start, $WT @ $(git -C "$WT" rev-parse --short HEAD) ==="
"$N/vision_pause.sh" pause
# the GPU must be free before Warp builds its graphs (11:40 the first try OOM'd on a vision
# model that was (re)loaded right after the pause): unload idle models until >= 10 GB free
free_gpu() {
  for i in $(seq 1 24); do
    for M in $(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}'); do ollama stop "$M" 2>/dev/null; echo "$(date +%H:%M:%S) unloaded $M"; done
    sleep 5
    f=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)
    (( f >= 10000 )) && { echo "$(date +%H:%M:%S) GPU free ${f} MiB"; return 0; }
  done
  echo "!!! $(date +%H:%M:%S) GPU still busy (${f} MiB free)"; return 1
}
trap '"$N/vision_pause.sh" resume; echo "=== $(date) shaper_gpu end (trap) ==="' EXIT
free_gpu || exit 1
# robot_legs_walk16_act's recipe (night queue 1101214001) minus out/steps/init/seed, the
# dead-time draw and the act-lag (both per job below)
COMMON="--robot --xml bimo_biped_v6legs.xml --precision --action-map full --ctrl-within-stops --servo-kp-scale asbuilt --quantize --mjx-impl warp --floor-contacts feet --family loco --walk-submix 0.2,0.2 --cmd-v-range 0.05,0.7 --turn-emph --mimic-lift 0.07 --swing-height 0.08 --w-mimic 1.5 --feet-phase-s2 0.002 --w-feet-phase 2.0 --roll-lash-deg 0.5,2.0 --backlash-deg 0,0.25 --zero-offset-deg 2 --torso-mass-range 0.4 --torso-com-y 0.015 --imu-mount-deg 3 --eval-scenarios walk_steps_L,walk_steps_R --stop-grace-s 0.4 --w-low-swing 10 --joint-frictionloss 0.35 --joint-frictionloss-range 0.1,0.4 --gyro-delay-max 1 --w-action-rate 0.3 --servo-kd-range 0.12,1.0 --servo-kp-range 0.6,1.2 --gyro-gain-range 0.7,1.6"
DEADLINE=$(date -d "today 21:40" +%s)
for spec in "$@"; do
  OUT=${spec%%|*}; EXTRA=${spec#*|}
  now=$(date +%s); left=$((DEADLINE - now))
  (( left > 3600 )) || { echo "$(date +%H:%M) skip $OUT: ${left}s left"; continue; }
  tt=$(( left - 1500 < 3300 ? left - 1500 : 3300 ))
  if [[ $EXTRA != DISTILL_ONLY ]]; then
    echo "$(date +%H:%M) teacher $OUT start (timeout ${tt}s): $EXTRA"
    free_gpu || { echo "skip $OUT: GPU busy"; continue; }
    cd "$WT/sim/mjx"
    XLA_PYTHON_CLIENT_PREALLOCATE=false timeout "$tt" "$PY" train_mjx.py --out "$OUT" $COMMON $EXTRA > "$WT/sim/runs/train_$OUT.log" 2>&1
    echo "$(date +%H:%M) teacher $OUT exit $?"
  fi
  [[ -s $WT/sim/runs/$OUT/params.pkl ]] || { echo "no params for $OUT"; continue; }
  now=$(date +%s); left=$((DEADLINE - now)); db=$(( left - 300 )); (( db > 1200 )) && db=1200
  (( db > 600 )) || { echo "$(date +%H:%M) skip distill $OUT: ${left}s left"; continue; }
  free_gpu || { echo "skip distill $OUT: GPU busy"; continue; }
  cd "$WT"
  echo "$(date +%H:%M) distill ${OUT}_s128r24 start (budget ${db}s)"
  PY=$PY DISTILL_BUDGET_S=$db DISTILL_NO_COLLECT=1 DISTILL_GATE= XLA_PYTHON_CLIENT_MEM_FRACTION=0.80 timeout $((db + 600)) \
      bash infra/night_distill.sh "$OUT" "${OUT}_s128r24" > "$WT/sim/runs/distill_${OUT}_s128r24.log" 2>&1
  echo "$(date +%H:%M) distill ${OUT}_s128r24 exit $?"
done
echo "=== $(date) shaper_gpu done ==="
