#!/usr/bin/env bash
# Distill a night teacher into the firmware-sized (128, 128) student, as a
# night-queue CMD job that runs right after its teacher (docs/training.md §9).
#
#   infra/night_distill.sh <teacher> [<student>] [distill_student.py args...]
#
# Queue file (sorted AFTER the teacher's job; night/preflight.sh.py checks
# that the teacher is either an existing run or an earlier job's --out):
#
#   BRANCH=main
#   # distill: robot_legs_walk11_full -> robot_legs_walk11_full_s128r24
#   CMD=infra/night_distill.sh robot_legs_walk11_full robot_legs_walk11_full_s128r24
#   OUT=robot_legs_walk11_full_s128r24
#
# What it does, in order:
#   1. refuses a teacher with no params.pkl/config.json (the teacher job
#      crashed or never started -- nothing to distill);
#   2. DAgger-distills it (sim/mjx/distill_student.py, 24 rounds = the
#      deployed recipe) on the teacher's own physics (Warp on the night
#      GPU; the jax impl when there is no GPU), inside a wall-clock budget
#      that ends DISTILL_END (06:45, before the runner's 06:55 guard) minus
#      the referee's reserve.  The student is saved after every round, so
#      even a kill leaves one;
#   3. referees the student on CPU on the walk gate -- walk_steps_L/R
#      (clearance: min peak swing height) + stand_10s -- into
#      sim/runs/<student>/scorecard_partial.md (the full family card is the
#      morning collector's job, step 4);
#   4. writes night/state.done.<student> so infra/night_collect_local.sh
#      (07:15) rsyncs the student into ~/code/robot and grades it exactly as
#      it grades a teacher: full family scorecard + eval_scenarios + reel.
#
# Env overrides: DISTILL_ALLOW_LEGACY=1 (a legacy-map teacher: referee-only
# student, not exportable), DISTILL_END=HH:MM, DISTILL_BUDGET_S=<s> (instead of the
# deadline, e.g. a daytime CPU run), DISTILL_EVAL_RESERVE_S, DISTILL_GATE
# (comma scenario list; "" skips step 3), DISTILL_NO_COLLECT=1 (skip step 4).
set -uo pipefail
[[ $# -ge 1 ]] || { sed -n '2,12p' "$0"; exit 2; }
TEACHER=$1; shift
STUDENT=${TEACHER}_s128r24
if [[ $# -ge 1 && $1 != --* ]]; then STUDENT=$1; shift; fi
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-$ROOT/.venv/bin/python}
RUNS=$ROOT/sim/runs
NIGHT=${NIGHT_DIR:-$HOME/code/robot-mjx/night}
T0=$(date +%s)
say() { echo "[night_distill $(date +%H:%M:%S)] $*"; }

TD=$RUNS/$TEACHER
for f in params.pkl config.json; do
  [[ -s $TD/$f ]] || { say "!!! teacher $TEACHER has no $f in $RUNS (did its job run?)"; exit 3; }
done
[[ -e $RUNS/$STUDENT ]] && { say "!!! $RUNS/$STUDENT already exists; refusing to overwrite"; exit 3; }
if [[ -s $TD/progress.jsonl ]]; then
  say "teacher $TEACHER last progress: $(tail -1 "$TD/progress.jsonl" | cut -c1-200)"
fi

# physics: the teacher's (Warp for the v6 robot runs) when a GPU is visible,
# the jax impl otherwise (Warp has no CPU path)
BACKEND=$(JAX_PLATFORMS= XLA_PYTHON_CLIENT_PREALLOCATE=false "$PY" -c 'import jax; print(jax.default_backend())' 2>/dev/null | tail -1)
IMPL=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1])).get("mjx_impl","jax"))' "$TD/config.json")
if [[ $BACKEND != gpu && $IMPL == warp ]]; then
  say "no GPU (jax backend: ${BACKEND:-?}); distilling the Warp teacher on the jax impl"
  IMPL=jax
fi
# Warp allocates outside XLA's pool: no preallocation for it (the runner
# exports MEM_FRACTION=0.80 for CMD jobs, which would starve Warp)
if [[ $IMPL == warp ]]; then
  unset XLA_PYTHON_CLIENT_MEM_FRACTION
  export XLA_PYTHON_CLIENT_PREALLOCATE=false
fi

RESERVE=${DISTILL_EVAL_RESERVE_S:-900}
if [[ -n ${DISTILL_BUDGET_S:-} ]]; then
  BUDGET=$DISTILL_BUDGET_S
else
  END=$(date -d "${DISTILL_END:-06:45}" +%s)
  (( END <= T0 )) && END=$((END + 86400))
  BUDGET=$((END - T0 - RESERVE))
fi
if (( BUDGET < 600 )); then
  say "!!! only ${BUDGET}s of budget left before ${DISTILL_END:-06:45}; not starting"
  exit 4
fi
say "distill $TEACHER -> $STUDENT  physics=$IMPL backend=${BACKEND:-?} budget=${BUDGET}s"

# the runner's 06:55 guard SIGTERMs this shell only; pass it on so the
# python child does not outlive the window holding the GPU
CHILD=
trap '[[ -n $CHILD ]] && kill -TERM $CHILD 2>/dev/null; say "SIGTERM: stopped (last finished round is saved)"; exit 143' TERM INT
# an exportable student needs a full-map teacher (export_policy_weights.py);
# DISTILL_ALLOW_LEGACY=1 distills a legacy-map one for the referee only
FULLMAP=--require-full-map
[[ -n ${DISTILL_ALLOW_LEGACY:-} ]] && FULLMAP=
distill() {   # $1 = physics impl, $2 = budget (s)
  "$PY" "$ROOT/sim/mjx/distill_student.py" --teacher "$TEACHER" --out "$STUDENT" \
    --rounds 24 --mjx-impl "$1" --time-budget-s "$2" $FULLMAP "${EXTRA[@]}" &
  CHILD=$!
  wait $CHILD
  RC=$?
  CHILD=
}
EXTRA=("$@")
distill "$IMPL" "$BUDGET"
if [[ ! -s $RUNS/$STUDENT/params.pkl && $IMPL == warp ]]; then
  # the Warp path is only ever exercised on the night GPU; if it dies before
  # its first round, the jax impl on the same GPU is what every v5 student
  # distilled on -- slower contacts, same plant
  BUDGET=$((BUDGET - $(date +%s) + T0))
  if (( BUDGET >= 600 )); then
    say "!!! Warp distill exited $RC before its first round; retrying on the jax impl (budget ${BUDGET}s)"
    rm -rf "${RUNS:?}/$STUDENT"
    IMPL=jax
    distill jax "$BUDGET"
  fi
fi
if [[ ! -s $RUNS/$STUDENT/params.pkl ]]; then
  say "!!! distill_student.py exited $RC with no student; nothing to referee"
  exit 5
fi
(( RC != 0 )) && say "!!! distill_student.py exited $RC; refereeing the last saved round"
say "distilled in $(( $(date +%s) - T0 ))s"

GATE=${DISTILL_GATE-walk_steps_L,walk_steps_R,stand_10s}
if [[ -n $GATE ]]; then
  say "referee (CPU): $GATE"
  ( cd "$ROOT/sim/mjx" && CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu timeout "$RESERVE" \
      "$PY" eval_precision.py --run-name "$STUDENT" --scenarios "$GATE" ) \
    || say "!!! gate referee failed (rc $?); the morning collector grades it anyway"
  [[ -s $RUNS/$STUDENT/scorecard_partial.md ]] && cat "$RUNS/$STUDENT/scorecard_partial.md"
fi

# hand the student to the 07:15 collector like any finished teacher (it
# collects from ~/code/robot-mjx/sim/runs, i.e. only a night-clone run)
if [[ -z ${DISTILL_NO_COLLECT:-} && -d $NIGHT && $RUNS -ef $HOME/code/robot-mjx/sim/runs ]]; then
  { echo "OUT=$STUDENT"; echo "STARTED=$T0"; echo "KIND=distill"
    echo "TEACHER=$TEACHER"
    echo "SHA=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null)"
    echo "ARGS=infra/night_distill.sh $TEACHER $STUDENT $*"; } > "$NIGHT/state.done.$STUDENT"
  say "queued $STUDENT for the morning collector ($NIGHT/state.done.$STUDENT)"
fi
say "done in $(( $(date +%s) - T0 ))s"
