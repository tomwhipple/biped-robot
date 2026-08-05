#!/usr/bin/env bash
# Morning collection, ON MIRA (cron 07:15, as claw).
#
# infra/night_collect.sh is the ORIGINAL, and it still assumes the old
# topology: laptop reaches mira over ssh + rsync. Since the project moved onto
# mira (2026-08-02) that script cannot run here -- there is no `ssh mira` from
# mira as claw. This is the same job done locally.
#
# Why it exists at all: night_run.sh only TRAINS. Nothing rendered the night's
# result automatically, which is how loco_v14turn_s128 became the deployed
# policy with no footage of it. This collects every finished run into the
# working checkout and referees it WITH --render, so the movie always exists.
#
# The training clone moved to claw on 2026-08-05 (user: "move anything that's
# still running in my workspace/user into yours"). Both the runner and this
# collector now run as claw out of ~/code/robot-mjx; tw's clone is left in
# place, untouched, as history.
set -uo pipefail

REPO=/home/claw/code/robot
NIGHT=/home/claw/code/robot-mjx/night
SRC=/home/claw/code/robot-mjx/sim/runs
PY="$REPO/.venv/bin/python"
LOG="$REPO/sim/runs/night_collect.log"

mkdir -p "$REPO/sim/runs"
exec >> "$LOG" 2>&1
echo "=== $(date) night_collect_local ==="

collect_one() {   # $1 = state file, $2 = OUT name
  local ST=$1 OUT=$2
  echo "--- collecting $OUT"
  mkdir -p "$REPO/sim/runs/$OUT"
  rsync -aq "$SRC/$OUT/" "$REPO/sim/runs/$OUT/" || {
      echo "!!! rsync failed for $OUT; leaving $ST in place"; return 1; }
  mv -f "$ST" "$NIGHT/state.collected.$OUT" 2>/dev/null \
      || echo "!!! could not rename $ST (ownership?); may re-collect tomorrow"

  # Python referee WITH the movie. --render is opt-in and forgetting it is the
  # exact failure this script exists to prevent, so it is not optional here.
  ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$PY" \
      eval_precision.py --run-name "$OUT" --render ) \
    && echo "SCORECARD: sim/runs/$OUT/scorecard.md   [python policy]" \
    || echo "!!! python referee FAILED for $OUT"

  # SIL referee: standing column, never fatal (a broken sil build must not lose
  # the night's collection).
  ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$PY" \
      eval_precision.py --run-name "$OUT" --sil ) \
    && echo "SCORECARD: sim/runs/$OUT/scorecard_sil.md   [SIL]" \
    || echo "(no SIL number for $OUT -- build firmware/host sil and rerun)"

  if [[ -f "$REPO/sim/runs/$OUT/$OUT.mov" ]]; then
    echo "MOVIE: sim/runs/$OUT/$OUT.mov ($(du -h "$REPO/sim/runs/$OUT/$OUT.mov" | cut -f1))"
  else
    echo "!!! NO MOVIE for $OUT -- eval_precision --render produced nothing"
  fi
}

found=0
for ST in "$NIGHT"/state.done.*; do
  [[ -e "$ST" ]] || continue
  OUT=$(sed -n 's/^OUT=//p' "$ST" | head -1)
  [[ -n "$OUT" ]] || { echo "!!! $ST has no OUT; skipping"; continue; }
  found=1
  collect_one "$ST" "$OUT"
done

# The 07:00 hard stop SIGTERMs a still-running job, which leaves a plain
# `state` behind rather than state.done.* -- loco_v11gait ended that way on
# 2026-08-02 and the original collector would skip it. Its last eval
# checkpoint is perfectly good, so collect it too once the PID is gone.
if [[ -f "$NIGHT/state" ]]; then
  PID=$(sed -n 's/^PID=//p' "$NIGHT/state" | head -1)
  OUT=$(sed -n 's/^OUT=//p' "$NIGHT/state" | head -1)
  if [[ -n "$OUT" ]] && { [[ -z "$PID" ]] || ! kill -0 "$PID" 2>/dev/null; }; then
    echo "--- $OUT was interrupted (pid $PID gone); collecting its last checkpoint"
    found=1
    collect_one "$NIGHT/state" "$OUT"
  fi
fi

(( found )) || echo "no completed runs to collect; queue: $(ls "$NIGHT/queue" 2>/dev/null | grep -v '^done$\|^held$' | tr '\n' ' ')"
echo "=== $(date) done ==="
