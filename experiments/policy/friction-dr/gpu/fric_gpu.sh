#!/usr/bin/env bash
# Daytime fine-tune + distill of robot_legs_walk20_fric (friction-dr, 2026-11-03), holding the
# night runner's lock (the runner exits if it finds it taken: release well before 22:00).
# Pauses the vision classifier, unloads idle ollama models, trains the night job's args line,
# distills (infra/night_distill.sh, no collector hand-off), resumes the classifier.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
JOB=$HERE/../night/1103fric01-robot_legs_walk20_fric
N=/home/claw/code/robot-mjx/night
W=/home/claw/code/robot-friction
PY=/home/claw/code/robot/.venv/bin/python
exec 9>"$N/.runner.lock"
flock -n 9 || { echo "lock busy"; exit 1; }
echo "$(date) lock taken"
trap '"$N/vision_pause.sh" resume; echo "$(date) resumed vision, releasing lock"' EXIT
"$N/vision_pause.sh" pause
while read -r M; do [[ -n $M ]] && { echo "unloading $M"; ollama stop "$M" || true; }; done < <(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}')
nvidia-smi --query-gpu=memory.used --format=csv
cd "$W/sim/mjx"
echo "$(date) train start @ $(git -C "$W" rev-parse --short HEAD)"
XLA_PYTHON_CLIENT_PREALLOCATE=false timeout 5400 "$PY" train_mjx.py $(grep -vE '^(BRANCH=|[[:space:]]*#|[[:space:]]*$)' "$JOB") > "$W/sim/runs/train_robot_legs_walk20_fric.log" 2>&1 < /dev/null
echo "$(date) train rc=$?"
cd "$W"
PY=$PY DISTILL_NO_COLLECT=1 DISTILL_BUDGET_S=1800 DISTILL_EVAL_RESERVE_S=900 \
  bash infra/night_distill.sh robot_legs_walk20_fric robot_legs_walk20_fric_s128r24 > "$W/sim/runs/cmd_robot_legs_walk20_fric_s128r24.log" 2>&1 < /dev/null
echo "$(date) distill rc=$?"
# a model ollama loaded onto the CPU while the GPU was held stays there: drop it (auto-reloads)
ollama ps 2>/dev/null | tail -n +2 | grep -i cpu | awk '{print $1}' | while read -r M; do ollama stop "$M" || true; done
