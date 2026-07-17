#!/usr/bin/env bash
# Morning collection for the nightly Mira training window: pull the run
# artifacts back into the local repo, run the CPU precision referee, and
# print where the scorecard landed. Reads OUT from Mira's night/state(.done).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

ST=$(ssh mira 'cat code/robot-mjx/night/state.done 2>/dev/null \
               || cat code/robot-mjx/night/state 2>/dev/null' || true)
[[ -n "$ST" ]] || { echo "no nightly run state on mira"; exit 1; }
OUT=$(sed -n 's/^OUT=//p' <<<"$ST")
PID=$(sed -n 's/^PID=//p' <<<"$ST")
echo ">>> nightly run: OUT=$OUT"
if ssh mira "kill -0 $PID 2>/dev/null"; then
  echo "!!! training still running on mira (pid $PID) -- collect later"
  ssh mira "tail -3 code/robot-mjx/sim/runs/train.log"
  exit 2
fi
echo "=== last training log lines ==="
ssh mira "tail -4 code/robot-mjx/sim/runs/train.log"
mkdir -p "$REPO/sim/runs/$OUT"
rsync -aq "mira:code/robot-mjx/sim/runs/$OUT/" "$REPO/sim/runs/$OUT/"
ssh mira "mv -f code/robot-mjx/night/state.done code/robot-mjx/night/state.collected 2>/dev/null || true"
echo "=== CPU precision referee ==="
( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" \
    eval_precision.py --run-name "$OUT" )
echo "ROUND COMPLETE: sim/runs/$OUT/scorecard.md"
