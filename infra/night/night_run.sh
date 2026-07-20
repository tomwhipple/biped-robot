#!/usr/bin/env bash
# Nightly training-window runner (cron 22:00, ON MIRA). Processes the job
# QUEUE at night/queue/* in sorted order -- multiple runs per night,
# sequentially. POLICY (user, 2026-07-20): training happens at NIGHT ONLY;
# days are reserved for the user's other work on this box (and we're
# waiting on printed parts anyway).
# Window: starts 22:00; no new job starts after 05:00; hard stop 07:00
# (night_stop.sh cron). Never evicts ACTIVE ollama inference; unloads IDLE
# resident models after ~10 min of 0% GPU (they auto-reload on demand).
# One-shot semantics: a job file moves to queue/done/ at LAUNCH, so an
# 07:00-interrupted job does not re-run the next night (its checkpoint is
# collectible either way).
set -uo pipefail
BASE="$HOME/code/robot-mjx"
N="$BASE/night"
exec >> "$N/night.log" 2>&1
echo "=== $(date) night_run ==="
PY="$BASE/.venv/bin/python"
[[ -x "$PY" ]] || { echo "!!! no venv python at $PY"; exit 1; }
mkdir -p "$N/queue" "$N/queue/done" "$BASE/sim/runs"
# legacy single-args file becomes the first queue item
[[ -f "$N/args" ]] && mv "$N/args" "$N/queue/00-legacy"

START_DEADLINE=$(date -d '05:00' +%s)
(( START_DEADLINE <= $(date +%s) )) && START_DEADLINE=$((START_DEADLINE + 86400))
END_HARD=$(date -d '07:00' +%s)
(( END_HARD <= $(date +%s) )) && END_HARD=$((END_HARD + 86400))

wait_gpu() {   # 0 = free; 1 = start deadline passed while waiting
  local IDLE_STREAK=0 LINE UTIL USED TOTAL FREE M
  while :; do
    LINE=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total \
           --format=csv,noheader,nounits | head -1 | tr -d ',')
    read -r UTIL USED TOTAL <<<"$LINE"
    FREE=$((TOTAL - USED))
    if (( UTIL < 20 && FREE >= 10000 )); then
      echo "$(date +%H:%M) gpu free (util=${UTIL}% free=${FREE}MiB)"
      return 0
    fi
    (( $(date +%s) > START_DEADLINE )) && return 1
    if (( UTIL < 5 )); then IDLE_STREAK=$((IDLE_STREAK + 1)); else IDLE_STREAK=0; fi
    if (( IDLE_STREAK >= 2 )); then
      while read -r M; do
        [[ -n "$M" ]] || continue
        echo "$(date +%H:%M) unloading idle ollama model $M (auto-reloads on demand)"
        ollama stop "$M" 2>/dev/null || true
      done < <(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}')
      IDLE_STREAK=0
      sleep 30
      continue
    fi
    echo "$(date +%H:%M) gpu busy (util=${UTIL}% free=${FREE}MiB); retry in 10 m"
    sleep 600
  done
}

for JOB in $(ls "$N/queue" 2>/dev/null | grep -v '^done$' | sort); do
  JF="$N/queue/$JOB"
  [[ -f "$JF" ]] || continue
  if (( $(date +%s) > START_DEADLINE )); then
    echo "no starts after 05:00; $JOB stays queued for tomorrow"
    break
  fi
  if ! wait_gpu; then
    echo "gpu never freed before 05:00; $JOB stays queued"
    break
  fi
  ARGS=$(cat "$JF")
  OUT=$(grep -oE '(--out)[= ][^ ]+' <<<"$ARGS" | awk -F'[= ]' '{print $2}')
  OUT=${OUT:-mjx_cmd_v1}
  REQ=$(grep -oE '(--steps)[= ][0-9]+' <<<"$ARGS" | grep -oE '[0-9]+$' || true)
  BUDGET=$(( (END_HARD - $(date +%s) - 900) * 17000 ))
  if (( BUDGET < 5000000 )); then
    echo "window too small for $JOB; stays queued"
    break
  fi
  if [[ -n "$REQ" ]] && (( REQ > BUDGET )); then
    ARGS=${ARGS/--steps $REQ/--steps $BUDGET}
    echo "capped --steps $REQ -> $BUDGET for $JOB"
  fi
  mv -f "$JF" "$N/queue/done/$JOB"      # one-shot: consumed at launch
  cd "$BASE/sim/mjx"
  XLA_PYTHON_CLIENT_MEM_FRACTION=0.90 nohup "$PY" train_mjx.py $ARGS \
    > "$BASE/sim/runs/train.log" 2>&1 < /dev/null &
  PID=$!
  { echo "PID=$PID"; echo "OUT=$OUT"; echo "STARTED=$(date +%s)"
    echo "ARGS=$ARGS"; } > "$N/state"
  echo "$(date +%H:%M) launched $JOB pid=$PID out=$OUT"
  # babysit this job until it finishes or the 06:55 guard (night_stop's
  # 07:00 SIGTERM handles the rest -- checkpoints save at every eval)
  while kill -0 "$PID" 2>/dev/null && (( $(date +%s) < END_HARD - 300 )); do
    sleep 120
  done
  if kill -0 "$PID" 2>/dev/null; then
    echo "$(date +%H:%M) $JOB still running at the 06:55 guard; night_stop takes it"
    break
  fi
  echo "$(date +%H:%M) $JOB finished"
  mv -f "$N/state" "$N/state.done.$OUT"
done
echo "$(date) night_run loop done"
