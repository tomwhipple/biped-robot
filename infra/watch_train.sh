#!/usr/bin/env bash
# watch_train.sh -- poll a training run started by train_launcher.sh (reads
# infra/.last_train, no args) until it finishes, then pull results back into
# the local repo and (for RunPod) tear down the pod.
#
# Completion = train.log contains "saved ->" (train_mjx.py's final print) OR
# the remote process is gone. The former is success; the latter without the
# "saved ->" line is a crash.
#
# Polls every 300s. Hard timeout: 8h without completion -> report and exit 2,
# leaving the run in place for inspection (a RunPod pod's own
# --terminate-after is the backstop there).
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
LAST_TRAIN="$HERE/.last_train"
LAST_POD="$HERE/runpod/.last_pod"
RUNPOD_SSHK="$HOME/.runpod/ssh/runpodctl-ssh-key"
RUNPOD_SSH_BASE=(-i "$RUNPOD_SSHK" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
                  -o BatchMode=yes -o ConnectTimeout=15)
MIRA_SSH_BASE=(-o ConnectTimeout=8 -o BatchMode=yes)
POLL_INTERVAL=300
HARD_TIMEOUT=$((8 * 3600))

# ssh_t <secs> <ssh args...> -- run ssh with a hard wall-clock timeout.
# (macOS has no timeout(1); ConnectTimeout alone doesn't bound a stuck auth.)
ssh_t() { local t="$1"; shift; ssh "$@" & local p=$!
  ( sleep "$t"; kill -9 "$p" 2>/dev/null ) & local w=$!
  wait "$p" 2>/dev/null; local rc=$?; kill "$w" 2>/dev/null; return "$rc"; }

[[ -f "$LAST_TRAIN" ]] || { echo "!!! $LAST_TRAIN not found -- nothing to watch (run train_launcher.sh first)"; exit 1; }
# shellcheck disable=SC1090
source "$LAST_TRAIN"
: "${HOST:?missing HOST in $LAST_TRAIN}" "${OUT:?missing OUT in $LAST_TRAIN}"
STARTED="${STARTED:-$(date +%s)}"

LOCAL_RUN_DIR="$REPO/sim/runs/$OUT"

case "$HOST" in
  mira)
    : "${PID:?missing PID in $LAST_TRAIN for HOST=mira}"
    REMOTE_RUN_DIR="~/code/robot-mjx/sim/runs/$OUT"
    REMOTE_LOG="~/code/robot-mjx/sim/runs/train.log"
    ;;
  runpod)
    [[ -f "$LAST_POD" ]] || { echo "!!! $LAST_POD not found -- can't reach the pod"; exit 1; }
    # shellcheck disable=SC1090
    source "$LAST_POD"
    : "${POD_ID:?}" "${IP:?}" "${PORT:?}"
    REMOTE_RUN_DIR="/root/robot/sim/runs/$OUT"
    REMOTE_LOG="/root/robot/sim/runs/train.log"
    ;;
  *)
    echo "!!! unknown HOST '$HOST' in $LAST_TRAIN"; exit 1 ;;
esac

echo ">>> watching HOST=$HOST OUT=$OUT (started $(date -r "$STARTED" 2>/dev/null || echo "epoch $STARTED"))"

# check_status -- one ssh call, prints DONE | RUNNING | CRASHED
check_status() {
  if [[ "$HOST" == mira ]]; then
    ssh_t 15 "${MIRA_SSH_BASE[@]}" mira "
      if grep -q 'saved ->' $REMOTE_LOG 2>/dev/null; then echo DONE
      elif kill -0 $PID 2>/dev/null; then echo RUNNING
      else echo CRASHED
      fi"
  else
    ssh_t 15 "${RUNPOD_SSH_BASE[@]}" -p "$PORT" "root@$IP" "
      if grep -q 'saved ->' $REMOTE_LOG 2>/dev/null; then echo DONE
      elif pgrep -f 'train_mjx.py' >/dev/null 2>&1; then echo RUNNING
      else echo CRASHED
      fi"
  fi
}

STATUS="RUNNING"
while true; do
  NOW=$(date +%s)
  ELAPSED=$((NOW - STARTED))
  if [[ $ELAPSED -ge $HARD_TIMEOUT ]]; then
    echo "!!! HARD TIMEOUT: $OUT on $HOST not complete after 8h (elapsed ${ELAPSED}s)."
    echo "!!! leaving the run in place for inspection (not tearing down)."
    exit 2
  fi
  STATUS="$(check_status)"
  STATUS="$(printf '%s' "$STATUS" | tail -1 | tr -d '[:space:]')"
  case "$STATUS" in
    DONE|CRASHED) break ;;
    RUNNING|*)
      echo ">>> [$(date '+%Y-%m-%d %H:%M:%S')] still running ($((ELAPSED / 60))m elapsed) -- sleeping ${POLL_INTERVAL}s"
      sleep "$POLL_INTERVAL"
      ;;
  esac
done

teardown_runpod() {
  echo ""
  echo "########################################################"
  echo "### TEARING DOWN RUNPOD POD $POD_ID (training finished)"
  echo "########################################################"
  runpodctl pod delete "$POD_ID"
}

pull_results() {
  mkdir -p "$LOCAL_RUN_DIR"
  echo "=== pulling results into $LOCAL_RUN_DIR ==="
  if [[ "$HOST" == mira ]]; then
    rsync -aq -e "ssh ${MIRA_SSH_BASE[*]}" \
      "mira:$REMOTE_RUN_DIR/params.pkl" \
      "mira:$REMOTE_RUN_DIR/config.json" \
      "mira:$REMOTE_RUN_DIR/progress.jsonl" \
      "$LOCAL_RUN_DIR/" 2>&1
  else
    rsync -aq -e "ssh ${RUNPOD_SSH_BASE[*]} -p $PORT" \
      "root@$IP:$REMOTE_RUN_DIR/params.pkl" \
      "root@$IP:$REMOTE_RUN_DIR/config.json" \
      "root@$IP:$REMOTE_RUN_DIR/progress.jsonl" \
      "$LOCAL_RUN_DIR/" 2>&1
  fi
}

pull_crash_log() {
  mkdir -p "$LOCAL_RUN_DIR"
  echo "=== pulling crash log tail into $LOCAL_RUN_DIR/crash.log ==="
  if [[ "$HOST" == mira ]]; then
    ssh_t 15 "${MIRA_SSH_BASE[@]}" mira "tail -50 $REMOTE_LOG" > "$LOCAL_RUN_DIR/crash.log" 2>&1
  else
    ssh_t 15 "${RUNPOD_SSH_BASE[@]}" -p "$PORT" "root@$IP" "tail -50 $REMOTE_LOG" > "$LOCAL_RUN_DIR/crash.log" 2>&1
  fi
}

if [[ "$STATUS" == "CRASHED" ]]; then
  echo "!!! training process on $HOST is gone WITHOUT a 'saved ->' line -- treating as a crash."
  pull_crash_log
  [[ "$HOST" == runpod ]] && teardown_runpod
  echo "!!! CRASH: see $LOCAL_RUN_DIR/crash.log"
  exit 1
fi

echo ">>> training completed (found 'saved ->' in train.log)"
pull_results
[[ "$HOST" == runpod ]] && teardown_runpod

EVAL_SCRIPT="$REPO/sim/mjx/eval_precision.py"
if [[ -f "$EVAL_SCRIPT" ]]; then
  echo "=== running local eval ==="
  ( cd "$REPO/sim/mjx" && JAX_PLATFORMS=cpu "$REPO/.venv/bin/python" eval_precision.py --run-name "$OUT" )
else
  echo ">>> eval_precision.py not found yet -- run this later:"
  echo "    cd $REPO/sim/mjx && JAX_PLATFORMS=cpu $REPO/.venv/bin/python eval_precision.py --run-name $OUT"
fi

echo "ROUND COMPLETE: sim/runs/$OUT/scorecard.md"
