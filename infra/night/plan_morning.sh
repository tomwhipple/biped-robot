#!/usr/bin/env bash
# Morning planner for the get-up campaign (cron 05:30 and 07:30, ON MIRA, as
# claw; the collector also starts it when it finishes between 00:00 and 07:30).
# Tom 2026-10-21: "In the morning, re-evaluate and plan the next run. There
# should be simulations running all night, every night, always pulling in the
# latest knowledge." The collector referees the night's runs; this waits
# for the night's work to be done (runner finished, collector finished, nothing
# left to collect), then hands the evaluation and tonight's queue to a headless Claude
# Code session (planner_prompt.md), and DMs Tom the one-line outcome.
# ensure_queue.sh (21:30) is the safety net if this fails.
# Installed copy: ~/code/robot-mjx/night/ (outside the training tree, like
# night_run.sh); source of truth: infra/night/ in the repo.
set -uo pipefail
N=${NIGHT_DIR:-$HOME/code/robot-mjx/night}     # NIGHT_DIR etc.: for testing
HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/night_window.sh"          # night_window_open, night_continue_decision, lock_free
NIGHT_RUN=${NIGHT_RUN_CMD:-$N/night_run.sh}
COLLECT_LOG=${COLLECT_LOG:-$HOME/code/robot/sim/runs/night_collect.log}
WT=${PLAN_WT:-$HOME/code/robot-planner}
RB=${PLAN_REPO:-$HOME/code/robot}
CLAUDE_BIN=${CLAUDE_BIN:-$HOME/.local/bin/claude}
POLL_S=${PLAN_POLL_S:-120}
POLL_N=${PLAN_POLL_N:-90}                      # 90 x 120 s = the 3 h cap
OUTDIR=$N/planner
TOM=mattermost:xhtnq11nhjn1jpytz1sj4zzimr     # Tom <-> aime DM (model-remediator's target)
mkdir -p "$OUTDIR"
exec >> "$N/planner.log" 2>&1
exec 8>"$N/.planner.lock"
flock -n 8 || { echo "=== $(date) plan_morning: another planner holds the lock; exiting"; exit 0; }

# idempotent: a plan that completed today (a "plan_morning done" line after a
# "plan_morning" header, both dated today) is not redone. The planner is
# started by cron at 05:30 and 07:30 and by the collector.
# EXCEPTION, the ADDENDUM pass: when this script restarted the runner after
# planning (see the end), it left $N/.continued. The next trigger (the
# collector when that runner's queue empties, or the 07:30 cron) then runs once
# more to grade the runs that continuation produced; it is not a new plan.
TODAY=$(date '+%a %b %e')
MODE=plan
if awk -v t="=== $TODAY " '
     index($0, t) == 1 && $0 ~ / plan_morning ===$/ {h = 1}
     index($0, t) == 1 && $0 ~ / plan_morning done ===$/ && h {ok = 1}
     END {exit !ok}' "$N/planner.log"; then
  if [[ -f $N/.continued ]]; then
    MODE=addendum
  else
    echo "=== $(date) plan_morning: already planned today; exiting"
    exit 0
  fi
fi
if [[ $MODE == addendum ]]; then
  echo "=== $(date) plan_morning (addendum: the continued night's runs) ==="
else
  echo "=== $(date) plan_morning ==="
fi

notify() {
  # Tom 2026-11-02: status DMs OFF while Claude Code manages the sim
  # campaigns; re-enable with NIGHT_DM_NOTIFY=1
  if [[ ${NIGHT_DM_NOTIFY:-} == 1 ]]; then
    "$HOME/.local/bin/hermes" send -q -t "$TOM" "Get-up planner: $1" \
      || echo "!!! Mattermost notify failed; message was: $1"
  fi
}

# wait (up to 3 h) until the night's work is done: the runner has finished
# (its lock is free), the collector has finished (ditto), no state.done.* is
# left to collect, and no job is still training. The planner must read finished
# scorecards.
night_done() {
  lock_free "$N/.runner.lock" && lock_free "$N/.collect.lock" || return 1
  compgen -G "$N/state.done.*" > /dev/null && return 1
  # a job the 06:55 guard left training: night_stop turns it into state.done at 07:00
  if [[ -f $N/state ]]; then
    local PID; PID=$(sed -n 's/^PID=//p' "$N/state" | head -1)
    [[ -n $PID ]] && kill -0 "$PID" 2>/dev/null && return 1
  fi
  return 0
}
for _ in $(seq "$POLL_N"); do
  night_done && break
  sleep "$POLL_S"
done
night_done || echo "!!! night's work not finished by $(date +%H:%M); planning on what is there"
if [[ $MODE == addendum ]] && [[ -z $(find "$N/queue/done" -maxdepth 1 -type f -cnewer "$N/.continued" 2>/dev/null | head -1) ]]; then
  # the restarted runner launched nothing (budget too small, pre-flight, GPU
  # never free): nothing new to grade
  echo "=== $(date) plan_morning: the continued runner launched no job; nothing to add; exiting"
  rm -f "$N/.continued"
  exit 0
fi

# the planner's own worktree, reset to origin/main every morning
git -C "$RB" fetch -q origin
if [[ ! -d $WT ]]; then
  git -C "$RB" worktree add -q --detach "$WT" origin/main
  ln -sfn "$RB/.venv" "$WT/.venv"
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
  if [[ $MODE == addendum ]]; then
    echo; echo "### THIS IS AN ADDENDUM PASS"; echo
    echo "Today's plan is already done and posted. After it, night_run.sh was restarted"
    echo "(window still open) and ran these jobs (queue/done/, launched after the plan):"; echo '```'
    find "$N/queue/done" -maxdepth 1 -type f -cnewer "$N/.continued" -printf '%f\n' | sort
    echo '```'; echo "Follow the section 'Later pass the same morning' of the instructions above."
  fi
} > "$OUTDIR/$STAMP.prompt.md"

uid=$(id -u)
cd "$WT" || cd "$RB"
env HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    XDG_RUNTIME_DIR="/run/user/$uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus" \
  timeout 5h "$CLAUDE_BIN" -p --output-format json --model opus \
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
  # fix-and-continue (Tom 2026-11-05): another session finishes the job
  "$N/claude_fix.sh" morning-planner "$OUTDIR/$STAMP.stderr" 120 <<EOF \
    && LAST="PLAN_STATUS: planned by the fixer after the planner failed (exit $RC)"
The morning planning session (plan_morning.sh -> claude -p with
$OUTDIR/$STAMP.prompt.md) failed: exit $RC, no PLAN_STATUS line. Its transcript is
$OUTDIR/$STAMP.json (stderr in the log tail below). Do what it was asked to do:
read the prompt file and follow it -- evaluate last night, update the ledger, and
queue tonight -- then report FIX_STATUS.
EOF
  QN=$(ls "$N/queue" | grep -cE '^[0-9]' || true)
fi
if [[ -z "$LAST" ]]; then
  notify "the morning planning session failed (exit $RC, no status line; transcript $OUTDIR/$STAMP.json). $QN jobs queued; ensure_queue.sh tops the night up at 21:30."
else
  notify "${LAST#PLAN_STATUS: } ($QN jobs queued)"
fi
PLAN_OK=1
[[ -z "$LAST" ]] && PLAN_OK=0
[[ $LAST == "PLAN_STATUS: planned by the fixer"* ]] || (( RC == 0 )) || PLAN_OK=0
if [[ -z "$LAST" ]]; then
  echo "=== $(date) plan_morning failed ==="    # not "done": the next trigger retries
else
  echo "=== $(date) plan_morning done ==="
  rm -f "$N/.continued"    # an addendum is consumed once it is graded
fi

# Keep the night going (Tom 2026-11-13: "why didn't the planner pick up the next
# task at 03:49? that was plenty of time"): the queue is ready and the runner
# exited when the previous queue ran dry, so restart it if a job could still
# start before the 07:00 stop. Never by day (the decision checks the window),
# never on a failed/empty plan, and never over a live runner (lock). The runner
# takes its own flock, pre-flights, and consumes jobs one-shot as before; when
# its queue ends it starts the collector, which triggers the addendum pass.
if REASON=$(night_continue_decision "$N" "$PLAN_OK" "$LAST"); then
  echo "$(date +%H:%M) $REASON"
  echo "$(date) plan_morning: $REASON" >> "$N/night.log"
  date +%s > "$N/.continued"
  # no inherited lock fds: 8 = the planner lock, 9 = the runner's own
  setsid nohup "$NIGHT_RUN" 8>&- 9>&- > /dev/null 2>&1 < /dev/null &
else
  echo "$(date +%H:%M) $REASON"
fi
