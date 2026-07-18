#!/usr/bin/env bash
# Nightly training-window runner. Lives ON MIRA (~/code/robot-mjx/night/),
# fired by cron at 23:00. One-shot: runs only if infra/night_arm.sh left an
# armed args file. POLICY (user, 2026-07-17): train on Mira nightly instead
# of RunPod -- may START any time 23:00-05:00, HARD STOP 07:00. Never evict
# ollama: WAIT for the GPU to free up (ollama auto-unloads idle models
# after ~5 min); if it never frees by 05:00, skip the night rather than
# OOM at launch or kill someone's in-flight inference.
set -uo pipefail
BASE="$HOME/code/robot-mjx"
N="$BASE/night"
exec >> "$N/night.log" 2>&1
echo "=== $(date) night_run ==="
[[ -f "$N/args" ]] || { echo "not armed; exit"; exit 0; }
PY="$BASE/.venv/bin/python"
[[ -x "$PY" ]] || { echo "!!! no venv python at $PY"; exit 1; }

# no new starts after 05:00 (hard stop is 07:00)
WAIT_UNTIL=$(date -d '05:00' +%s)
(( WAIT_UNTIL <= $(date +%s) )) && WAIT_UNTIL=$((WAIT_UNTIL + 86400))
IDLE_STREAK=0
while :; do
  LINE=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total \
         --format=csv,noheader,nounits | head -1 | tr -d ',')
  read -r UTIL USED TOTAL <<<"$LINE"
  FREE=$((TOTAL - USED))
  if (( UTIL < 20 && FREE >= 10000 )); then
    echo "$(date +%H:%M) gpu free (util=${UTIL}% free=${FREE}MiB) -- go"
    break
  fi
  # idle-but-squatted: a resident ollama model with a long keep_alive can
  # hold VRAM ALL night (2026-07-17 skipped 23:00-05:00 exactly this way,
  # util ~0% throughout). After two consecutive idle samples (~10 min
  # apart), unload idle models -- `ollama stop` on an IDLE model is
  # harmless (it auto-reloads on the next request); active inference
  # (util >= 5%) resets the streak and is never touched.
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
  if (( $(date +%s) > WAIT_UNTIL )); then
    echo "SKIPPED tonight: gpu busy through the wait window" \
         "(util=${UTIL}% free=${FREE}MiB). Armed run kept for tomorrow."
    exit 0
  fi
  echo "$(date +%H:%M) gpu busy (util=${UTIL}% free=${FREE}MiB); retry in 10 m"
  sleep 600
done

ARGS=$(cat "$N/args")
OUT=$(grep -oE '(--out)[= ][^ ]+' <<<"$ARGS" | awk -F'[= ]' '{print $2}')
OUT=${OUT:-mjx_cmd_v1}
# cap --steps to what fits before the 07:00 hard stop (conservative 4070 Ti
# rate 17k steps/s, 15 min margin for the final eval/save)
REQ=$(grep -oE '(--steps)[= ][0-9]+' <<<"$ARGS" | grep -oE '[0-9]+$' || true)
END=$(date -d '07:00' +%s); NOW=$(date +%s)
(( END <= NOW )) && END=$((END + 86400))
BUDGET=$(( (END - NOW - 900) * 17000 ))
if [[ -n "$REQ" ]] && (( REQ > BUDGET )); then
  ARGS=${ARGS/--steps $REQ/--steps $BUDGET}
  echo "capped --steps $REQ -> $BUDGET to fit the window"
fi

mv "$N/args" "$N/args.running"
mkdir -p "$BASE/sim/runs"
cd "$BASE/sim/mjx"
XLA_PYTHON_CLIENT_MEM_FRACTION=0.85 nohup "$PY" train_mjx.py $ARGS \
  > "$BASE/sim/runs/train.log" 2>&1 < /dev/null &
PID=$!
{ echo "PID=$PID"; echo "OUT=$OUT"; echo "STARTED=$(date +%s)"
  echo "ARGS=$ARGS"; } > "$N/state"
echo "launched pid $PID out=$OUT"
