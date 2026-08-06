#!/usr/bin/env bash
# CONTINUOUS training runner for a multi-day window.
#
# The nightly runner (night_run.sh) exists because of a standing policy:
# training happens 22:00-07:00 only, days belong to the user and the Hermes
# collective's LLM inference. This is the exception: user 2026-08-06 granted
# 17:30 Thu -> Mon 12:00 straight through. It is NOT a replacement for
# night_run.sh -- when the window closes, the nightly policy resumes.
#
# Differences from night_run.sh:
#   * runs until MARATHON_UNTIL instead of a 07:00 hard stop
#   * REFEREES AND RENDERS each job the moment it finishes, so scorecards and
#     movies accumulate through the window instead of waiting for a 07:15 cron
#   * caps VRAM so the collective's smaller ollama models still fit, and obeys
#     a PAUSE sentinel anyone can drop to get the GPU back between jobs
#   * takes the SAME flock as night_run.sh, so the 22:00 cron finds the lock
#     held and exits instead of launching a second trainer
set -uo pipefail
BASE="$HOME/code/robot-mjx"
REPO="$HOME/code/robot"
N="$BASE/night"
PY="$BASE/.venv/bin/python"
exec >> "$N/marathon.log" 2>&1

# VRAM cap, agreed with Aime 2026-08-06. 0.45 of 12282 MiB ~= 5.5 GB, which
# leaves ~6.7 GB -- enough for the collective's MEMORY STACK to stay resident:
# qwen2.5:7b-ctx16k (5.8 GB, honcho deriver+api) plus nomic-embed (0.6 GB).
# Verified empirically: 1024 envs trains fine at this cap, so the courtesy
# costs no env count. The 9.3 GB vision model (driveway classifier) cannot
# coexist with training and degrades to CPU for the window; Aime accepted
# that explicitly as the one casualty.
XLA_FRAC="${XLA_FRAC:-0.45}"
# Throughput estimate for step budgeting (measured 2333 steps/s at 1024 envs
# on the full card; assume the cap costs ~25%).
STEPS_PER_SEC=2000

[[ -f "$N/MARATHON_UNTIL" ]] || { echo "$(date) no MARATHON_UNTIL; refusing to run"; exit 1; }
DEADLINE=$(cat "$N/MARATHON_UNTIL")

exec 9>"$N/.runner.lock"
if ! flock -n 9; then
  echo "=== $(date) marathon: another runner holds the lock; exiting ==="
  exit 0
fi
echo "=== $(date) marathon_run START, until $(date -d @"$DEADLINE") ==="

sync_repo() {
  local BR=$1
  if git -C "$BASE" fetch -q --filter=blob:none origin "$BR"; then
    git -C "$BASE" reset -q --hard FETCH_HEAD
    echo "$(date +%H:%M) repo @ $BR $(git -C "$BASE" rev-parse --short HEAD)"
  else
    echo "$(date +%H:%M) !!! fetch $BR failed; training existing tree"
  fi
}

wait_gpu() {   # 0 = go, 1 = deadline passed
  local NEED=5900   # the cap (5.5 GB) plus a little headroom
  while :; do
    (( $(date +%s) > DEADLINE )) && return 1
    if [[ -f "$N/PAUSE" ]]; then
      echo "$(date +%H:%M) PAUSE sentinel present; holding (rm it to resume)"
      sleep 120; continue
    fi
    local LINE UTIL USED TOTAL FREE
    LINE=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total \
           --format=csv,noheader,nounits | head -1 | tr -d ',')
    read -r UTIL USED TOTAL <<<"$LINE"
    FREE=$((TOTAL - USED))
    if (( FREE >= NEED )); then
      echo "$(date +%H:%M) gpu ok (util=${UTIL}% free=${FREE}MiB, need ${NEED})"
      return 0
    fi
    # Only evict what is IDLE. A model actively serving the collective is left
    # alone -- this window is long enough that waiting a few minutes is cheap.
    if (( UTIL < 5 )); then
      while read -r M; do
        [[ -n "$M" ]] || continue
        echo "$(date +%H:%M) unloading idle ollama model $M"
        ollama stop "$M" 2>/dev/null || true
      done < <(ollama ps 2>/dev/null | tail -n +2 | awk '{print $1}')
      sleep 10
    else
      echo "$(date +%H:%M) gpu busy (util=${UTIL}% free=${FREE}MiB); retry 5m"
      sleep 300
    fi
  done
}

referee() {    # $1 = run name; scorecards + movie, right after the job
  local OUT=$1
  echo "$(date +%H:%M) refereeing $OUT"
  mkdir -p "$REPO/sim/runs/$OUT"
  rsync -aq "$BASE/sim/runs/$OUT/" "$REPO/sim/runs/$OUT/" || true
  ( cd "$REPO/sim/mjx" && MUJOCO_GL=egl JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" \
      eval_precision.py --run-name "$OUT" --render ) \
    && echo "$(date +%H:%M) SCORECARD sim/runs/$OUT/scorecard.md + movie" \
    || echo "$(date +%H:%M) !!! python referee failed for $OUT"
  ( cd "$REPO/sim/mjx" && MUJOCO_GL=egl JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" \
      eval_precision.py --run-name "$OUT" --sil ) \
    && echo "$(date +%H:%M) SCORECARD sim/runs/$OUT/scorecard_sil.md" \
    || echo "$(date +%H:%M) (no SIL number for $OUT)"
}

while :; do
  (( $(date +%s) > DEADLINE )) && { echo "$(date) window closed"; break; }
  JOB=$(ls "$N/queue" 2>/dev/null | grep -vE '^done$|^held$' | sort | head -1)
  [[ -n "$JOB" ]] || { echo "$(date +%H:%M) queue empty; idling 10m"; sleep 600; continue; }
  JF="$N/queue/$JOB"
  [[ -f "$JF" ]] || continue

  if ! wait_gpu; then echo "$(date) deadline reached while waiting"; break; fi

  BR=$(sed -n 's/^BRANCH=//p' "$JF" | head -1)
  sync_repo "${BR:-main}"
  ARGS=$(grep -vE '^BRANCH=' "$JF")
  OUT=$(grep -oE '(--out)[= ][^ ]+' <<<"$ARGS" | awk -F'[= ]' '{print $2}')
  OUT=${OUT:-marathon_job}

  # cap steps to what fits before the deadline (leave 25 min for the referee)
  REQ=$(grep -oE '(--steps)[= ][0-9]+' <<<"$ARGS" | grep -oE '[0-9]+$' || true)
  BUDGET=$(( (DEADLINE - $(date +%s) - 1500) * STEPS_PER_SEC ))
  if (( BUDGET < 3000000 )); then
    echo "$(date +%H:%M) only $BUDGET steps left in the window; stopping"
    break
  fi
  if [[ -n "$REQ" ]] && (( REQ > BUDGET )); then
    ARGS=${ARGS/--steps $REQ/--steps $BUDGET}
    echo "$(date +%H:%M) capped --steps $REQ -> $BUDGET for $JOB"
  fi

  mv -f "$JF" "$N/queue/done/$JOB"    # one-shot, consumed at launch
  cd "$BASE/sim/mjx" || exit 1
  XLA_PYTHON_CLIENT_MEM_FRACTION="$XLA_FRAC" nohup "$PY" train_mjx.py $ARGS \
    > "$BASE/sim/runs/train_$OUT.log" 2>&1 < /dev/null &
  PID=$!
  { echo "PID=$PID"; echo "OUT=$OUT"; echo "STARTED=$(date +%s)"
    echo "SHA=$(git -C "$BASE" rev-parse HEAD)"; echo "ARGS=$ARGS"; } > "$N/state"
  echo "$(date +%H:%M) launched $JOB pid=$PID out=$OUT"

  while kill -0 "$PID" 2>/dev/null; do
    if (( $(date +%s) > DEADLINE )); then
      echo "$(date +%H:%M) deadline: SIGTERM $PID (eval checkpoint stands)"
      kill "$PID" 2>/dev/null; sleep 30; kill -9 "$PID" 2>/dev/null || true
      break
    fi
    if [[ -f "$N/PAUSE" ]]; then
      echo "$(date +%H:%M) PAUSE raised mid-job; letting $OUT finish, then holding"
    fi
    sleep 120
  done
  echo "$(date +%H:%M) $JOB finished"
  mv -f "$N/state" "$N/state.done.$OUT" 2>/dev/null || true
  referee "$OUT"
done
echo "=== $(date) marathon_run DONE ==="
