#!/usr/bin/env bash
# Health check for the marathon window (cron every 20 min).
#
# User 2026-08-06: "check on the run periodically... we wouldn't want it to
# finish in error." Nobody is watching a 90-hour window in real time, so this
# is what watches it: it escalates to Slack when the run is dead, stalled,
# erroring, or idling, and stays quiet otherwise.
#
# Writes a one-screen status to $N/STATUS regardless, so the state is always
# readable without digging through logs.
set -uo pipefail
BASE="$HOME/code/robot-mjx"
REPO="$HOME/code/robot"
N="$BASE/night"
SLACK="slack:C0BD7Q39KG8"
STATUS="$N/STATUS"
ALERTED="$N/.watchdog_alerted"

now=$(date +%s)
[[ -f "$N/MARATHON_UNTIL" ]] || exit 0
until=$(cat "$N/MARATHON_UNTIL")
(( now > until )) && exit 0        # window closed; nothing to watch
# ...and stay quiet BEFORE it opens. MARATHON_UNTIL is written when the window
# is scheduled, which can be hours ahead of the start; without this the
# watchdog would page "runner dead" the moment it is armed.
if [[ -f "$N/MARATHON_FROM" ]] && (( now < $(cat "$N/MARATHON_FROM") )); then
  exit 0
fi

alert() {   # only once per distinct reason, so we do not spam
  local key=$1 msg=$2
  grep -qxF "$key" "$ALERTED" 2>/dev/null && return 0
  echo "$key" >> "$ALERTED"
  hermes send -t "$SLACK" "robot marathon: $msg" >/dev/null 2>&1 \
    || echo "(slack send failed) $msg"
}

runner_up=0
pgrep -u "$(id -u)" -f 'marathon_run.sh' >/dev/null && runner_up=1

train_pid=""; out=""
if [[ -f "$N/state" ]]; then
  train_pid=$(sed -n 's/^PID=//p' "$N/state" | head -1)
  out=$(sed -n 's/^OUT=//p' "$N/state" | head -1)
fi
train_up=0
[[ -n "$train_pid" ]] && kill -0 "$train_pid" 2>/dev/null && train_up=1

gpu=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits | head -1)
queued=$(ls "$N/queue" 2>/dev/null | grep -vcE '^done$|^held$' || echo 0)
donecnt=$(ls "$N/queue/done" 2>/dev/null | wc -l)

# most recent training log and how stale it is
log=$(ls -t "$BASE"/sim/runs/train_*.log 2>/dev/null | head -1)
stale="n/a"
if [[ -n "$log" ]]; then
  stale=$(( (now - $(stat -c %Y "$log")) / 60 ))
fi

{
  echo "marathon status @ $(date)"
  echo "  window ends   : $(date -d @"$until")  ($(( (until-now)/3600 ))h left)"
  echo "  runner alive  : $runner_up"
  echo "  training      : ${out:-<none>} pid=${train_pid:-none} alive=$train_up"
  echo "  log staleness : ${stale} min  (${log:-no log})"
  echo "  gpu util,MiB  : $gpu"
  echo "  queue         : $queued waiting, $donecnt consumed"
  [[ -f "$N/PAUSE" ]] && echo "  PAUSE         : SET (training held)"
} > "$STATUS"

# ---- escalation conditions ------------------------------------------------
if [[ -f "$N/PAUSE" ]]; then exit 0; fi     # deliberately held; not a fault

if (( runner_up == 0 )); then
  alert "runner-dead" "runner is NOT running but the window is open until $(date -d @"$until"). Nothing will train. $STATUS"
  exit 0
fi

if (( train_up == 0 && queued == 0 )); then
  alert "queue-empty" "no job training and the queue is EMPTY with $(( (until-now)/3600 ))h of GPU left -- the window is being wasted."
fi

if (( train_up == 1 )) && [[ "$stale" != "n/a" ]] && (( stale > 45 )); then
  alert "stalled-$out" "job $out is alive but its log has not moved in ${stale} min. Possible hang."
fi

if [[ -n "$log" ]] && grep -qE 'Traceback|RESOURCE_EXHAUSTED|out of memory|CUDA_ERROR' "$log"; then
  alert "error-$out" "job $out log contains an error signature. Check $log"
fi

# referee failures leave a marker in the marathon log
if [[ -f "$N/marathon.log" ]] && tail -200 "$N/marathon.log" | grep -q 'python referee failed'; then
  alert "referee-fail" "a referee/render failed during the window -- scorecards or movies may be missing."
fi

exit 0
