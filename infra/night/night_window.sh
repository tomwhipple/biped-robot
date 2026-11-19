# Sourced by plan_morning.sh (and its tests): the night window and the
# "keep the night going after the plan" decision. Installed beside
# night_run.sh in ~/code/robot-mjx/night/; source of truth: infra/night/.
#
# THE WINDOW MUST MATCH night_run.sh: the hard stop is 07:00 (night_stop.sh)
# and no job starts after START_DEADLINE = hard stop - 900 s (06:45); the
# window opens at 22:00. tests/test_night_continue.py pins the 900 s against
# night_run.sh's own START_DEADLINE line so the two cannot drift apart.
#
# NIGHT_NOW (epoch seconds) overrides the clock: for tests only.
NIGHT_START_HHMM=${NIGHT_START_HHMM:-22:00}
NIGHT_END_HHMM=${NIGHT_END_HHMM:-07:00}
NIGHT_START_SLACK_S=900

night_now() { echo "${NIGHT_NOW:-$(date +%s)}"; }

# 0 = a job could still START now: after 22:00, or before the 06:45 deadline.
# Never true by day (07:00..22:00): the project rule is GPU at night only.
night_window_open() {
  local now day end start
  now=$(night_now)
  day=$(date -d "@$now" +%F)
  end=$(date -d "$day $NIGHT_END_HHMM" +%s)
  start=$(date -d "$day $NIGHT_START_HHMM" +%s)
  if (( now < end )); then
    (( now <= end - NIGHT_START_SLACK_S ))
  else
    (( now >= start ))
  fi
}

lock_free() { ( flock -n 3 ) 3> "$1"; }

# night_continue_decision NIGHTDIR PLAN_OK STATUS_LINE
#   PLAN_OK: 1 if the planner session succeeded (exit 0 with a PLAN_STATUS
#   line, or the fixer finished the job), else 0.
# Prints the reason; returns 0 when night_run.sh should be started again now.
night_continue_decision() {
  local nd=$1 ok=$2 status=${3:-} n
  n=$(ls "$nd/queue" 2>/dev/null | grep -cE '^[0-9]' || true)
  if [[ $ok != 1 ]]; then echo "not continuing: the planner did not finish cleanly"; return 1; fi
  if [[ $status == "PLAN_STATUS: FAILED"* ]]; then echo "not continuing: planner reported FAILED"; return 1; fi
  if (( n == 0 )); then echo "not continuing: no jobs queued"; return 1; fi
  if ! night_window_open; then
    echo "not continuing: outside the night window; $n job(s) wait for 22:00"; return 1
  fi
  if ! lock_free "$nd/.runner.lock"; then echo "not continuing: a runner already holds the lock"; return 1; fi
  echo "continuing the night: $n job(s) queued at $(date -d "@$(night_now)" +%H:%M), window open until $NIGHT_END_HHMM"
  return 0
}
