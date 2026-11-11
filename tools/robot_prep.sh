#!/usr/bin/env bash
# Per-boot prep of the robot for a deployed policy: set AND VERIFY the two
# runtime settings the firmware does not persist and the policy was trained
# with (bench CLI over Wi-Fi, tools/wcli.py; docs/wifi-cli.md):
#
#   standclock  what the gait clock does at a plain stand (boot default: run)
#   shape <hz>  the firmware command shaper's pole (boot default: 10)
#
#   tools/robot_prep.sh --run robot_legs_prec16_f075_js20 [--host IP]
#   tools/robot_prep.sh --run <run> --check      read back and compare, set nothing
#   tools/robot_prep.sh --run <run> --dry-run    print what it would send
#
# The values come from the run's config.json (sim/runs/<run>/, or a directory
# given to --run), exactly as the referee reads them (walker_env):
#   shape      fw_shaper_hz (the middle of fw_shaper_hz..fw_shaper_hz_max when a
#              band was trained); absent = trained without the shaper = 0 = off
#   standclock clock_stand_coast set    -> `standclock coast <phase>`
#              clock_stand_freeze       -> nothing to set: the firmware's
#                                          obs::kClockStandFreeze always holds
#              neither                  -> `standclock run`
# Run it after every battery power-up (the settings do not survive a reboot),
# before arming. Wait for the robot over Wi-Fi first (up to 60 s).
#
# Exit: 0 set and verified (or --check: matches), 2 robot not reachable,
# 3 standclock does not read back / does not match, 4 shape does not read back
# / does not match, 5 the run's config.json is unusable.
# link/arm_script.py --check-prep <run> calls this with --check and refuses to
# arm on a non-zero exit.
#
# RUNS_DIR overrides sim/runs (trained runs are gitignored: a worktree has none).
# WCLI overrides the client command (tests use a stub); it gets --host first.
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RUN="" HOST="" CHECK=0 DRY=0
while (( $# )); do
  case $1 in
    --run) RUN=${2:?--run needs a run name or directory}; shift 2;;
    --host) HOST=${2:?--host needs an address}; shift 2;;
    --check) CHECK=1; shift;;
    --dry-run) DRY=1; shift;;
    -h|--help) sed -n '2,/^set -u/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "robot_prep: unknown argument $1" >&2; exit 5;;
  esac
done
[[ -n $RUN ]] || { echo "usage: robot_prep.sh --run <run> [--host IP] [--check|--dry-run]" >&2; exit 5; }
if [[ -d $RUN ]]; then CFG=$RUN/config.json; else CFG=${RUNS_DIR:-$ROOT/sim/runs}/$RUN/config.json; fi
PY=${PYTHON:-$ROOT/.venv/bin/python}

# MODE PHASE HZ, one line
read -r MODE PHASE HZ < <("$PY" - "$CFG" <<'PYEOF'
import json, sys
try:
    c = json.load(open(sys.argv[1]))
except (OSError, ValueError) as e:
    sys.exit(f"robot_prep: cannot read {sys.argv[1]}: {e}")
hz = float(c.get("fw_shaper_hz") or 0.0)
if c.get("fw_shaper_hz_max") is not None:
    hz = 0.5 * (hz + float(c["fw_shaper_hz_max"]))
coast = c.get("clock_stand_coast")
if coast is not None:
    print("coast", f"{float(coast):.4f}", f"{hz:g}")
elif c.get("clock_stand_freeze"):
    print("freeze", "-", f"{hz:g}")
else:
    print("run", "-", f"{hz:g}")
PYEOF
) || { echo "!!! cannot derive settings from $CFG" >&2; exit 5; }
[[ -n ${MODE:-} ]] || { echo "!!! cannot derive settings from $CFG" >&2; exit 5; }
[[ $PHASE == - ]] && PHASE=""
echo "run $(basename "$(dirname "$CFG")"): standclock $MODE${PHASE:+ $PHASE} , shape $HZ Hz  (from $CFG)"

W() {
  local -a c
  read -ra c <<< "${WCLI:-$PY $ROOT/tools/wcli.py}"
  timeout 10 "${c[@]}" ${HOST:+--host "$HOST"} "$@" 2>&1
}
if (( DRY )); then
  (( CHECK )) || { [[ $MODE != freeze ]] && echo "would send: standclock $MODE${PHASE:+ $PHASE}"
                   echo "would send: shape $HZ"; }
  echo "would read back: standclock, shape"
  exit 0
fi

for _ in $(seq 1 20); do W stat | grep -q policy && break; sleep 3; done
W stat | grep -E "policy|bus" || { echo "!!! robot not reachable over Wi-Fi" >&2; exit 2; }

if [[ $MODE == freeze ]]; then
  W standclock | grep -q "trained with the clock HELD" \
    || { echo "!!! standclock: expected the firmware to hold the clock at a stand (kClockStandFreeze); this build was generated for another run" >&2; exit 3; }
else
  (( CHECK )) || W standclock $MODE $PHASE >/dev/null
  GOT=$(W standclock)
  if [[ $MODE == coast ]]; then
    WANT="standclock: coast .* phase $(printf '%.3f' "$PHASE") rad"
  else
    WANT="standclock: $MODE "
  fi
  grep -qE "$WANT" <<< "$GOT" || { echo "!!! standclock $MODE ${PHASE:+ $PHASE} did not read back: $GOT" >&2; exit 3; }
fi

(( CHECK )) || W shape "$HZ" >/dev/null
GOT=$(W shape)
if [[ $HZ == 0 ]]; then WANT="shape: off"; else WANT="pole $(printf '%.2f' "$HZ") Hz"; fi
grep -qF -- "$WANT" <<< "$GOT" || { echo "!!! shape $HZ did not read back: $GOT" >&2; exit 4; }

echo "OK: standclock $MODE${PHASE:+ $PHASE}, shape $HZ Hz$( (( CHECK )) && echo ' (checked, nothing set)')"
W heading | head -1; W flail | sed -n 4p; W limit | head -1; W imu | grep -E "^up"; W volt | head -1
exit 0
