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
# single-instance guard: an early manual start (user 2026-07-23 "start the
# simulations now") must not collide with the 22:00 cron firing a second
# runner over the same queue
exec 9>"$N/.runner.lock"
if ! flock -n 9; then
  echo "=== $(date) night_run: another runner holds the lock; exiting ==="
  exit 0
fi
echo "=== $(date) night_run ==="
PY="$BASE/.venv/bin/python"
[[ -x "$PY" ]] || { echo "!!! no venv python at $PY"; exit 1; }

# $BASE is a git clone (converted 2026-07-31 after the stale-plant
# incident: rsync's hand-curated file list silently missed the v3yaw XML
# for two days). Every job trains on a freshly pulled committed tree; a
# queue file's optional BRANCH= line selects a branch for one-off
# experiments (default main). This runner copy lives OUTSIDE the tree
# (night/ is untracked) so a reset can never rewrite it mid-run.
sync_repo() {   # $1 = branch; leaves the tree at that branch's origin tip
  local BR=$1
  if git -C "$BASE" fetch -q --filter=blob:none origin "$BR"; then
    git -C "$BASE" reset -q --hard FETCH_HEAD
    echo "$(date +%H:%M) repo @ $BR $(git -C "$BASE" rev-parse --short HEAD)"
  else
    echo "$(date +%H:%M) !!! fetch $BR failed; training on existing tree" \
         "@ $(git -C "$BASE" rev-parse --short HEAD)"
  fi
}
sync_repo main
mkdir -p "$N/queue" "$N/queue/done" "$BASE/sim/runs"
# legacy single-args file becomes the first queue item
[[ -f "$N/args" ]] && mv "$N/args" "$N/queue/00-legacy"

START_DEADLINE=$(date -d '05:00' +%s)
(( START_DEADLINE <= $(date +%s) )) && START_DEADLINE=$((START_DEADLINE + 86400))
END_HARD=$(date -d '07:00' +%s)
(( END_HARD <= $(date +%s) )) && END_HARD=$((END_HARD + 86400))


restore_ollama() {
  # A model that ollama loads WHILE training holds the GPU falls back to
  # CPU inference and STAYS there after the GPU frees (ollama never
  # migrates a loaded model) -- the user's LLM then "takes forever" until
  # something forces a reload. Fix (user problem report 2026-07-21):
  # detect models not fully resident in VRAM, unload them, and re-warm --
  # the reload lands on the freed GPU. No service restart needed.
  local STRANDED M
  STRANDED=$(curl -s -m 10 localhost:11434/api/ps | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for m in d.get('models', []):
    if m.get('size_vram', 0) < m.get('size', 1):
        print(m['name'])
" 2>/dev/null)
  for M in $STRANDED; do
    echo "$(date +%H:%M) migrating CPU-stranded ollama model $M back to GPU"
    ollama stop "$M" 2>/dev/null || true
    curl -s -m 300 localhost:11434/api/generate -d "{\"model\":\"$M\"}" \
      >/dev/null 2>&1 || true
  done
}

wait_gpu() {   # 0 = free; 1 = start deadline passed while waiting
  local IDLE_STREAK=0 LINE UTIL USED TOTAL FREE M
  while :; do
    LINE=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total \
           --format=csv,noheader,nounits | head -1 | tr -d ',')
    read -r UTIL USED TOTAL <<<"$LINE"
    FREE=$((TOTAL - USED))
    # FREE gate 11500 not 10000: two v8 launches OOM'd (cuSolver) racing
    # ollama's migrate-back at ~10.5 GB free (2026-07-30)
    if (( UTIL < 20 && FREE >= 11500 )); then
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
      # recheck fast: a 30 s sleep here lost the race to a 3am vision
      # workload reloading llama3.2-vision onto the freed GPU (2026-07-30)
      sleep 5
      continue
    fi
    echo "$(date +%H:%M) gpu busy (util=${UTIL}% free=${FREE}MiB); retry in 10 m"
    sleep 600
  done
}

# re-scan the queue after every job (not a startup snapshot): jobs armed
# while an earlier run trains are picked up the same night (2026-07-31)
while JOB=$(ls "$N/queue" 2>/dev/null | grep -v '^done$' | sort | head -1); [[ -n "$JOB" ]]; do
  JF="$N/queue/$JOB"
  [[ -f "$JF" ]] || break   # vanished mid-scan; a continue would spin forever
  if (( $(date +%s) > START_DEADLINE )); then
    echo "no starts after 05:00; $JOB stays queued for tomorrow"
    break
  fi
  if ! wait_gpu; then
    echo "gpu never freed before 05:00; $JOB stays queued"
    break
  fi
  BR=$(sed -n 's/^BRANCH=//p' "$JF" | head -1)
  sync_repo "${BR:-main}"
  # CMD= jobs: run an arbitrary command from $BASE instead of train_mjx.py
  # (first need: nightly distill_student.py -- 2026-08-30, after two ad-hoc
  # launches lost GPU races to ollama's vision reload; ALL gpu work goes
  # through this loop's wait_gpu from now on). No --steps budget capping;
  # keep CMD jobs short or put them first in the queue.
  CMD=$(sed -n 's/^CMD=//p' "$JF" | head -1)
  if [[ -n "$CMD" ]]; then
    OUT=$(sed -n 's/^OUT=//p' "$JF" | head -1); OUT=${OUT:-$JOB}
    mv -f "$JF" "$N/queue/done/$JOB"
    cd "$BASE"
    XLA_PYTHON_CLIENT_MEM_FRACTION=0.80 nohup bash -c "$CMD" \
      > "$BASE/sim/runs/cmd_${OUT}.log" 2>&1 < /dev/null &
    PID=$!
    echo "$(date +%H:%M) launched CMD job $JOB pid=$PID"
    while kill -0 "$PID" 2>/dev/null && (( $(date +%s) < END_HARD - 300 )); do
      sleep 60
    done
    if kill -0 "$PID" 2>/dev/null; then
      echo "$(date +%H:%M) CMD job $JOB hit the 06:55 guard; killing"
      kill "$PID" 2>/dev/null
    fi
    continue
  fi
  ARGS=$(grep -v '^BRANCH=' "$JF")
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
    echo "SHA=$(git -C "$BASE" rev-parse HEAD)"
    echo "BRANCH=${BR:-main}"
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
  restore_ollama
done
# leave the tree on main so a branch job never strands the clone
[[ -n "${BR:-}" && "${BR:-main}" != main ]] && sync_repo main
echo "$(date) night_run loop done"
