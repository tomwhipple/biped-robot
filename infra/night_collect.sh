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
  # SIL referee: the SAME run scored through the real firmware C++ control
  # stack (docs/sil-harness.md).  Standing column, but never fatal: a laptop
  # without a compiler, or a broken sil build, must not lose the night's
  # collection.  eval_precision --sil builds/export as needed and refuses to
  # half-run, so a non-zero exit here means "no SIL number", not "bad number".
  SIL_OK=0
  ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" \
      eval_precision.py --run-name "$OUT" --sil ) && SIL_OK=1 || true
  echo "SCORECARD: sim/runs/$OUT/scorecard.md   [python policy]"
  if [[ "$SIL_OK" == 1 ]]; then
    echo "SCORECARD: sim/runs/$OUT/scorecard_sil.md   [SIL (firmware stack)]"
  else
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "!!! SIL REFEREE SKIPPED for $OUT -- toolchain or build failure."
    echo "!!! Only the python-path scorecard exists; the firmware stack is"
    echo "!!! UNVERIFIED for this run.  Retry:"
    echo "!!!   make -C firmware/host sil"
    echo "!!!   cd sim/mjx && JAX_PLATFORMS=cpu ../../.venv/bin/python \\"
    echo "!!!       eval_precision.py --run-name $OUT --sil"
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  fi
done
ssh mira 'pgrep -f "train_mjx[.]py" >/dev/null && echo "NOTE: a run is still ACTIVE on mira -- collect again later"' || true
