#!/usr/bin/env bash
# Morning collection for the nightly Mira window: pull EVERY completed run
# (night/state.done.*), referee each on the CPU, print scorecard paths.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

STS=$(ssh mira 'ls code/robot-mjx/night/state.done.* 2>/dev/null' || true)
if [[ -z "$STS" ]]; then
  echo "no completed nightly runs on mira; queue state:"
  ssh mira 'ls code/robot-mjx/night/queue 2>/dev/null | grep -v "^done$" || echo "(queue empty)"'
  exit 1
fi
for ST in $STS; do
  OUT=$(ssh mira "sed -n 's/^OUT=//p' '$ST'")
  [[ -n "$OUT" ]] || { echo "!!! $ST has no OUT; skipping"; continue; }
  echo "=== collecting $OUT ==="
  mkdir -p "$REPO/sim/runs/$OUT"
  rsync -aq "mira:code/robot-mjx/sim/runs/$OUT/" "$REPO/sim/runs/$OUT/"
  ssh mira "mv -f '$ST' 'code/robot-mjx/night/state.collected.$OUT'"
  ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" \
      eval_precision.py --run-name "$OUT" )
  echo "SCORECARD: sim/runs/$OUT/scorecard.md"
done
ssh mira 'pgrep -f "train_mjx[.]py" >/dev/null && echo "NOTE: a run is still ACTIVE on mira -- collect again later"' || true
