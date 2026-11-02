#!/usr/bin/env bash
# GPU distills for the int8 (256, 256) student of robot_legs_walk16_act, under the machine-wide
# GPU lock (night/.runner.lock), with the camera classifier paused and ollama models unloaded.
#   gpu_distill.sh <student> <hidden> [distill_student.py args...]
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
NIGHT=$HOME/code/robot-mjx/night
STUDENT=$1; HIDDEN=$2; shift 2
exec 9>"$NIGHT/.runner.lock"
echo "[$(date +%T)] waiting for the GPU lock"
flock 9
echo "[$(date +%T)] lock held"
"$NIGHT/vision_pause.sh" pause
for m in $(ollama ps 2>/dev/null | awk 'NR>1{print $1}'); do ollama stop "$m"; done
trap '"$NIGHT/vision_pause.sh" resume' EXIT
unset XLA_PYTHON_CLIENT_MEM_FRACTION
export XLA_PYTHON_CLIENT_PREALLOCATE=false
cd "$ROOT"
timeout 3000 .venv/bin/python sim/mjx/distill_student.py --teacher robot_legs_walk16_act \
  --out "$STUDENT" --hidden "$HIDDEN" --rounds 24 --mjx-impl warp --time-budget-s 2700 \
  --require-full-map "$@"
echo "[$(date +%T)] distill rc=$?"
