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

REPO=$HOME/code/robot
NIGHT=$HOME/code/robot-mjx/night
SRC=$HOME/code/robot-mjx/sim/runs
PY="$REPO/.venv/bin/python"
LOG="$REPO/sim/runs/night_collect.log"

# MuJoCo's renderer needs an OpenGL context. Under cron there is no DISPLAY,
# so the default GLFW/X11 backend dies with "an OpenGL platform library has
# not been loaded" -- and that kills the WHOLE python referee, not just the
# movie: the first --render frame throws and takes the scorecard with it
# (loco_v15body, 2026-08-06, produced a SIL card and nothing else).
# EGL renders offscreen against the NVIDIA driver with no display server.
# osmesa is NOT installed here, so it is not a fallback.
export MUJOCO_GL=egl

mkdir -p "$REPO/sim/runs"
exec >> "$LOG" 2>&1
echo "=== $(date) night_collect_local ==="
# the referee loads what the run trained against (a --rise-ref npz, a plant)
# from THIS checkout, which nothing else pulls: a reference added for last
# night's run would be missing here and the scorecard would fail
#
# 2026-11-05: the 11-05 collection refereed four runs on stale code because
# uncommitted generated STLs in $REPO blocked the fast-forward. Now: fetch; if
# the fast-forward is blocked, stash ONLY the dirty paths that upstream also
# changed (tagged, never re-applied -- `git stash list` shows them), retry, and
# leave every other local edit alone. Then VERIFY HEAD == origin/main.
# fix-and-continue (Tom 2026-11-05): a failure here starts a headless Claude
# session (infra/night/claude_fix.sh, installed in $NIGHT) and the step is
# retried once after it exits
fix() { "$NIGHT/claude_fix.sh" "$@"; }

sync_referee() {   # 0 = $REPO is at origin/main
  git -C "$REPO" fetch -q origin main
  if ! git -C "$REPO" merge -q --ff-only FETCH_HEAD 2>/dev/null; then
    local -a BLOCK
    mapfile -t BLOCK < <(comm -12 \
      <(git -C "$REPO" diff --name-only HEAD | sort) \
      <(git -C "$REPO" diff --name-only HEAD FETCH_HEAD | sort))
    if (( ${#BLOCK[@]} )); then
      local TAG="night_collect $(date +%F_%H%M) blocked ff"
      git -C "$REPO" stash push -q -m "$TAG" -- "${BLOCK[@]}" \
        && echo "stashed ${#BLOCK[@]} dirty path(s) that blocked the fast-forward as '$TAG': ${BLOCK[*]}"
    fi
    git -C "$REPO" merge -q --ff-only FETCH_HEAD
  fi
  if [[ $(git -C "$REPO" rev-parse HEAD) == $(git -C "$REPO" rev-parse FETCH_HEAD 2>/dev/null) ]]; then
    echo "repo @ origin/main $(git -C "$REPO" rev-parse --short HEAD) (verified)"
    return 0
  fi
  return 1
}
if ! sync_referee; then
  if fix collect-sync "$LOG" 30 <<EOF && sync_referee; then
night_collect_local.sh could not bring the referee checkout $REPO to origin/main
(fetch, then fast-forward, after stashing only the dirty paths that upstream also
changed). The referee must run on main's code. Fix it without discarding
uncommitted work; the collector retries once after you exit.
EOF
    echo "repo synced after a fixer"
  else
    echo "!!! STALE REFEREE: could not bring $REPO to origin/main; refereeing on" \
         "$(git -C "$REPO" rev-parse --short HEAD) -- re-referee every card from this morning on main"
  fi
fi

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
  referee() { ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$PY" \
      eval_precision.py --run-name "$OUT" --render ); }
  if referee; then
    echo "SCORECARD: sim/runs/$OUT/scorecard.md   [python policy]"
  elif fix "referee-$OUT" "$LOG" 45 <<EOF && referee; then
The morning referee failed for $OUT: in $REPO/sim/mjx,
\`JAX_PLATFORMS=cpu $PY eval_precision.py --run-name $OUT --render\` exited
non-zero (its output is at the end of $LOG). The run dir is
$REPO/sim/runs/$OUT. Fix the cause in the referee or the code (a bug: fix it
on main with tests; a run that is genuinely broken: say so). The collector
re-runs this exact command once after you exit.
EOF
    echo "SCORECARD: sim/runs/$OUT/scorecard.md   [python policy, after a fixer]"
  else
    echo "!!! python referee FAILED for $OUT"
  fi

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
