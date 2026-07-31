#!/usr/bin/env bash
# Arm tonight's 22:00-07:00 training window on Mira (POLICY 2026-07-17:
# nightly Mira window instead of RunPod). Since 2026-07-31 Mira's
# ~/code/robot-mjx is a git clone and night_run.sh pulls the committed
# tree before EVERY job -- this script no longer ships code. It verifies
# the branch is pushed, installs the runner scripts + cron (idempotent),
# and queues the train_mjx args.
#
# Usage: infra/night_arm.sh [--branch <name>] <train_mjx.py args>
#   --branch: train a one-off from a pushed branch instead of main.
# Collect results next morning with: infra/night_collect.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
[[ $# -gt 0 ]] || { echo "usage: night_arm.sh [--branch <name>] <train_mjx.py args>"; exit 1; }

BRANCH=main
ARGS=()
expect_branch=0
for a in "$@"; do
  if (( expect_branch )); then BRANCH="$a"; expect_branch=0; continue; fi
  case "$a" in
    --branch)   expect_branch=1 ;;
    --branch=*) BRANCH="${a#--branch=}" ;;
    *)          ARGS+=("$a") ;;
  esac
done
(( expect_branch )) && { echo "!!! --branch needs a value"; exit 1; }
(( ${#ARGS[@]} > 0 )) || { echo "!!! no train_mjx args left after --branch"; exit 1; }

echo "=== verifying $BRANCH is pushed (the run trains origin/$BRANCH) ==="
git -C "$REPO" fetch -q origin "$BRANCH" 2>/dev/null \
  || { echo "!!! branch '$BRANCH' not on origin -- push it first"; exit 1; }
LOCAL=$(git -C "$REPO" rev-parse "$BRANCH" 2>/dev/null || true)
REMOTE=$(git -C "$REPO" rev-parse "origin/$BRANCH")
if [[ -n "$LOCAL" && "$LOCAL" != "$REMOTE" ]]; then
  echo "!!! local $BRANCH ($LOCAL) != origin/$BRANCH ($REMOTE) -- push first"
  exit 1
fi
if [[ -n "$(git -C "$REPO" status --porcelain --untracked-files=no -- sim infra)" ]]; then
  echo ">>> WARNING: uncommitted sim/infra changes will NOT train tonight:"
  git -C "$REPO" status --short --untracked-files=no -- sim infra | sed 's/^/    /'
fi
echo "    origin/$BRANCH @ $(git -C "$REPO" rev-parse --short "origin/$BRANCH")"

echo "=== installing runner scripts + cron on mira (idempotent) ==="
# the runner lives OUTSIDE the clone (night/ is untracked) so a mid-run
# git reset can never rewrite the executing script; it self-updates here
rsync -aq "$HERE/night/night_run.sh" "$HERE/night/night_stop.sh" \
  mira:code/robot-mjx/night/
ssh mira 'chmod +x code/robot-mjx/night/night_*.sh
TAB=$(crontab -l 2>/dev/null | grep -v "robot-mjx/night/night_" || true)
printf "%s\n0 22 * * * %s\n0 7 * * * %s\n" "$TAB" \
  "$HOME/code/robot-mjx/night/night_run.sh" \
  "$HOME/code/robot-mjx/night/night_stop.sh" | crontab -
crontab -l | grep night_'

OUT="job"
prev=""
for a in "${ARGS[@]}"; do
  [[ "$prev" == "--out" ]] && OUT="$a"
  [[ "$a" == --out=* ]] && OUT="${a#--out=}"
  prev="$a"
done
QN="$(date +%m%d%H%M%S)-$OUT"
echo "=== queueing $QN [branch $BRANCH]: ${ARGS[*]} ==="
{ echo "BRANCH=$BRANCH"; printf '%s\n' "${ARGS[*]}"; } \
  | ssh mira "mkdir -p code/robot-mjx/night/queue && cat > code/robot-mjx/night/queue/$QN"
echo ">>> tonight's queue (runs in this order from 22:00; no starts after"
echo ">>> 05:00, hard stop 07:00; collect with infra/night_collect.sh):"
ssh mira 'ls code/robot-mjx/night/queue | grep -v "^done$"'
