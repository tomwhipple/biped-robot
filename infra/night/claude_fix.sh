#!/usr/bin/env bash
# Fix-and-continue for the night harness (Tom 2026-11-05: "build it to notify
# you if there's a failure so that you can resolve and continue").
#
# A harness step that fails calls this helper. The helper starts a headless
# Claude Code session with the failure and its log, WAITS for the session to
# finish (bounded by a timeout), and exits 0 only when the session reports
# `FIX_STATUS: fixed`. The caller then retries its step once. If the retry also
# fails, the caller falls back to what it did before this helper existed.
#
#   claude_fix.sh STEP LOGFILE TIMEOUT_MIN <<'EOF'
#   what failed, what the caller retries after a fix, anything it knows
#   EOF
#
# Guards:
#   - one fixer at a time (flock);
#   - FIX_MAX_PER_DAY sessions a day (default 6), so a step that keeps failing
#     can't loop;
#   - CLAUDE_FIX=0 disables the helper (exit 1, the caller goes on as before).
#
# Every session leaves a transcript in night/fixer/. The morning planner reads
# fixer.log.
#
# Installed copy: ~/code/robot-mjx/night/ (outside the training tree).
# Source of truth: infra/night/ in the repo.
set -uo pipefail
STEP=${1:?step}; LOGF=${2:-/dev/null}; TMIN=${3:-30}
N=${NIGHT_DIR:-$HOME/code/robot-mjx/night}
OUT=$N/fixer
mkdir -p "$OUT"
FLOG=$N/fixer.log
DETAIL=$(cat 2>/dev/null || true)
log() { echo "$(date '+%F %H:%M') [$STEP] $*" | tee -a "$FLOG" >&2; }

[[ ${CLAUDE_FIX:-1} == 0 ]] && { log "CLAUDE_FIX=0: not calling a fixer"; exit 1; }
TODAY=$(date +%F)
USED=$(grep -c "^$TODAY .* fixer start" "$FLOG" 2>/dev/null || true)
if (( ${USED:-0} >= ${FIX_MAX_PER_DAY:-6} )); then
  log "daily fixer budget spent (${USED}); not calling a fixer"
  exit 1
fi
exec 7>"$N/.fixer.lock"
flock -w $((TMIN * 60)) 7 || { log "another fixer held the lock for ${TMIN} min; giving up"; exit 1; }

STAMP=$(date +%Y%m%d-%H%M%S)-${STEP//[^A-Za-z0-9_.-]/_}
P=$OUT/$STAMP.prompt.md
{
cat <<EOF
# Night harness fixer: \`$STEP\` failed

You were started headless by the biped-robot night-training harness on mira
(\`infra/night/claude_fix.sh\`) because a step failed. **Your job is to fix the
cause so that the step succeeds when the caller retries it right after you
exit.** Nobody is watching live. Work in \`~/code/robot\` (the main checkout),
\`~/code/robot-mjx\` (the training clone; \`night/\` there holds the installed
harness, queue, logs) or a worktree of your own. Start with \`AGENTS.md\` and the
Claude memory in \`~/.claude/projects/-home-claw-code-robot/memory/\`.

Time limit: ${TMIN} min, after which you are killed. Now: $(date).

## What failed
$DETAIL

## Log tail (\`$LOGF\`)
\`\`\`
$(tail -n 80 "$LOGF" 2>/dev/null)
\`\`\`

## Rules
- STRIDE FIRST (Tom 2026-11-05): no get-up work until the robot strides. Get-up
  jobs stay in \`queue/held/\`.
- Never touch the robot or the serial bus. Never kill or restart a running
  training process (\`night/state\` names it), and never start one yourself.
  The runner launches jobs.
- Never discard anyone's uncommitted work. Use a tagged \`git stash push -m\`
  of only the paths in the way, or a WIP branch, and say which.
- Code fixes: make them on a branch and run the relevant tests
  (\`JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES=\`; it is night, and the GPU
  belongs to training). Push with gh's credential helper, never a token in
  argv. Merge to main only if the change is safe and opt-in or a clear bug
  fix with green tests. Otherwise point the affected queue jobs at the branch
  with \`BRANCH=\`.
- Queue jobs: fix a broken job file in place, or move it to \`queue/held/\`
  with a \`# held by fixer: <reason>\` line. Never delete one.
- If a job crashed at launch and you fixed the cause, re-queue it: copy it
  from \`queue/done/\` back to \`queue/\` with its original name.
- The GitHub repo is tomwhipple/biped-robot. "Latest from GitHub" means
  origin/main plus any relevant unmerged branch.
- Leave a short note of what you did at the end of \`$FLOG\` (append only).

End your reply with exactly one line:
\`FIX_STATUS: fixed <one line>\` (the caller should retry) or
\`FIX_STATUS: not-fixed <one line: why, and what a human must do>\`.
EOF
} > "$P"

log "fixer start (timeout ${TMIN} min, prompt $P)"
uid=$(id -u)
cd "$HOME/code/robot" || cd "$HOME"
env HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    XDG_RUNTIME_DIR="/run/user/$uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus" \
    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= \
  timeout "${TMIN}m" "${CLAUDE_BIN:-$HOME/.local/bin/claude}" -p --output-format json --model opus \
    --permission-mode bypassPermissions --add-dir "$HOME/code" \
    < "$P" > "$OUT/$STAMP.json" 2> "$OUT/$STAMP.stderr"
RC=$?
LAST=$(python3 - "$OUT/$STAMP.json" <<'PY'
import json, sys
try:
    r = json.load(open(sys.argv[1])).get("result") or ""
except Exception:
    r = ""
l = [x for x in r.strip().splitlines() if x.startswith("FIX_STATUS:")]
print(l[-1] if l else "")
PY
)
log "fixer exit=$RC: ${LAST:-<no FIX_STATUS line>} (transcript $OUT/$STAMP.json)"
[[ $LAST == "FIX_STATUS: fixed"* ]]
