#!/usr/bin/env bash
# Arm tonight's 22:00-07:00 training window on Mira (POLICY 2026-07-17:
# nightly Mira window instead of RunPod). Since 2026-07-31 Mira's
# ~/code/robot-mjx is a git clone and night_run.sh pulls the committed
# tree before EVERY job -- this script no longer ships code. It verifies
# the branch is pushed, installs the cron lines (idempotent), and queues the
# train_mjx args.
#
# The harness runs as claw (moved from tw 2026-08-05); `ssh mira` lands as
# tw, so every step on mira goes through `sudo -n -u claw`.
#
# Cron fires the scripts in night/ on mira (~claw/code/robot-mjx/night/), which
# the runner mirrors from the clone's infra/night/ (main) at every start; this
# script ships no scripts. A harness change is a merge to main.
#
# Usage: infra/night_arm.sh [--branch <name>] <train_mjx.py args>
#   --branch: train a one-off from a pushed branch instead of main.
# Results are refereed on mira at 07:15 (infra/night_collect_local.sh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
NIGHT=/home/claw/code/robot-mjx/night
BASE=/home/claw/code/robot-mjx
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

echo "=== installing the cron lines on mira (idempotent) ==="
# Cron fires the copies in night/ (the clone's untracked state directory), never
# the clone's own infra/night/: a BRANCH= job checks a feature branch out in the
# clone, and cron must not then run that branch's night_stop or plan_morning.
# night/'s scripts are mirrored from main's infra/night/ by the runner at its
# start (night_run.sh install_harness), so this script ships none.
ssh mira "sudo -n -u claw bash -s" <<EOF
for f in night_run night_stop plan_morning ensure_queue; do
  [[ -x $NIGHT/\$f.sh ]] || { echo "!!! $NIGHT/\$f.sh is missing: the harness is not deployed (docs/training.md)"; exit 1; }
done
TAB=\$(crontab -l 2>/dev/null | grep -v -e robot-mjx -e night_collect_local || true)
printf '%s\n%s\n%s\n%s\n%s\n%s\n%s\n' "\$TAB" \
  "0 22 * * * $NIGHT/night_run.sh" \
  "0 7 * * * $NIGHT/night_stop.sh" \
  "15 7 * * * /home/claw/code/robot/infra/night_collect_local.sh" \
  "30 7 * * * $NIGHT/plan_morning.sh" \
  "30 21 * * * $NIGHT/ensure_queue.sh" \
  "30 5 * * * $NIGHT/plan_morning.sh" | crontab -
crontab -l | grep -e robot-mjx -e night_collect_local
EOF

OUT="job"
prev=""
for a in "${ARGS[@]}"; do
  [[ "$prev" == "--out" ]] && OUT="$a"
  [[ "$a" == --out=* ]] && OUT="${a#--out=}"
  prev="$a"
done
QN="$(date +%m%d%H%M%S)-$OUT"
echo "=== queueing $QN [branch $BRANCH]: ${ARGS[*]} ==="
{ echo "BRANCH=$BRANCH"; printf '%s\n' "${ARGS[@]}"; } \
  | ssh mira "sudo -n -u claw tee $NIGHT/queue/$QN > /dev/null"
echo ">>> tonight's queue (runs in this order from 22:00; no starts after"
echo ">>> 06:45, hard stop 07:00; refereed on mira at 07:15):"
ssh mira "ls $NIGHT/queue | grep -v -x -e done -e held"
