#!/usr/bin/env bash
# Morning planner for the get-up campaign (cron 07:30, ON MIRA, as claw).
# Tom 2026-10-21: "In the morning, re-evaluate and plan the next run. There
# should be simulations running all night, every night, always pulling in the
# latest knowledge." The 07:15 collector referees the night's runs; this waits
# for it, then hands the evaluation and tonight's queue to a headless Claude
# Code session (planner_prompt.md), and DMs Tom the one-line outcome.
# ensure_queue.sh (21:30) is the safety net if this fails.
# Installed copy: ~/code/robot-mjx/night/ (outside the training tree, like
# night_run.sh); source of truth: infra/night/ in the repo.
set -uo pipefail
N=$HOME/code/robot-mjx/night
HERE=$(cd "$(dirname "$0")" && pwd)
COLLECT_LOG=$HOME/code/robot/sim/runs/night_collect.log
WT=$HOME/code/robot-planner
OUTDIR=$N/planner
TOM=mattermost:xhtnq11nhjn1jpytz1sj4zzimr     # Tom <-> aime DM (model-remediator's target)
mkdir -p "$OUTDIR"
exec >> "$N/planner.log" 2>&1
exec 8>"$N/.planner.lock"
flock -n 8 || { echo "=== $(date) plan_morning: another planner holds the lock; exiting"; exit 0; }
echo "=== $(date) plan_morning ==="

notify() {
  "$HOME/.local/bin/hermes" send -q -t "$TOM" "Get-up planner: $1" \
    || echo "!!! Mattermost notify failed; message was: $1"
}

# wait (up to 3 h) for today's collection to finish: it referees and renders
# every run, and the planner must read finished scorecards
TODAY=$(date '+%a %b %e')
for _ in $(seq 90); do
  grep -E '^=== .* done ===$' "$COLLECT_LOG" | tail -1 | grep -q "$TODAY" && break
  sleep 120
done
grep -E '^=== .* done ===$' "$COLLECT_LOG" | tail -1 | grep -q "$TODAY" \
  || echo "!!! collection not finished by $(date +%H:%M); planning on what is there"

# the planner's own worktree, reset to origin/main every morning
git -C "$HOME/code/robot" fetch -q origin
if [[ ! -d $WT ]]; then
  git -C "$HOME/code/robot" worktree add -q --detach "$WT" origin/main
  ln -sfn "$HOME/code/robot/.venv" "$WT/.venv"
fi
git -C "$WT" checkout -q --detach origin/main || echo "!!! could not reset $WT to origin/main"

STAMP=$(date +%Y%m%d)
{
  cat "$HERE/planner_prompt.md"
  echo; echo "## Context appended by plan_morning.sh ($(date))"; echo
  echo '### Last night: night.log (from the last night_run header)'; echo '```'
  awk '/^=== .* night_run ===$/ {buf=""} {buf = buf $0 "\n"} END {printf "%s", buf}' "$N/night.log" | tail -80
  echo '```'; echo; echo '### This morning: night_collect.log (from the last header)'; echo '```'
  awk '/^=== .* night_collect_local ===$/ {buf=""} {buf = buf $0 "\n"} END {printf "%s", buf}' "$COLLECT_LOG" | tail -80
  echo '```'; echo; echo "### Queue now"; echo '```'
  ls "$N/queue"; echo '```'
} > "$OUTDIR/$STAMP.prompt.md"

uid=$(id -u)
cd "$WT" || cd "$HOME/code/robot"
env HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    XDG_RUNTIME_DIR="/run/user/$uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus" \
  timeout 5h "$HOME/.local/bin/claude" -p --output-format json --model opus \
    --permission-mode bypassPermissions --add-dir "$HOME/code" \
    < "$OUTDIR/$STAMP.prompt.md" > "$OUTDIR/$STAMP.json" 2> "$OUTDIR/$STAMP.stderr"
RC=$?
LAST=$(python3 - "$OUTDIR/$STAMP.json" <<'PY'
import json, sys
try:
    r = json.load(open(sys.argv[1])).get("result") or ""
except Exception:
    r = ""
lines = [l for l in r.strip().splitlines() if l.startswith("PLAN_STATUS:")]
print(lines[-1] if lines else "")
PY
)
echo "$(date +%H:%M) claude exit=$RC: ${LAST:-<no PLAN_STATUS line>}"
QN=$(ls "$N/queue" | grep -cE '^[0-9]' || true)
if [[ -z "$LAST" ]]; then
  notify "the morning planning session failed (exit $RC, no status line; transcript $OUTDIR/$STAMP.json). $QN jobs queued; ensure_queue.sh tops the night up at 21:30."
else
  notify "${LAST#PLAN_STATUS: } ($QN jobs queued)"
fi
echo "=== $(date) plan_morning done ==="
