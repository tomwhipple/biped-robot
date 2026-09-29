#!/bin/sh
# Every automated CAD gate for the robot, in one command.
#
#   sh cad/run_checks.sh          # exit 0 = clear, 1 = something fails
#
# Two kinds of check:
#
#   check_assembly_v6.py    the articulated v6 assembly (cad/v6/assembly_v6.py)
#                           posed through every joint's ROM -- extremes plus
#                           interior samples -- for every pair of pieces that
#                           move relative to each other. Run twice: the default
#                           build, and ARMS=1 (shoulder girdle + the two arms).
#                           Results are what docs/design-v6/cad_rom_check.txt
#                           records.
#   audit_torso.py,         printability REPORTS (cad/check_printability.py's
#   audit_ankle_foot.py     engine: bridges, ceilings, islands, first-layer
#                           contact) on the exported STLs in cad/v6/stl. They
#                           do not fail the run: pelvis_v7 and ankle_link carry
#                           ceilings and islands by design, and supports are
#                           the slicer's job (cad/PRINT_LIST.md). Read the
#                           ** lines against the slice preview.
#
# The other parts audit themselves from their own module's __main__
# (leg_link_v6, hip_yoke_v6, shoulder_girdle_v6, arm_v6) -- those rewrite
# their STL as they go, so they are run when the part changes, not here.
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY="$ROOT/.venv/bin/python"

rc=0

run() {
    label=$1
    shift
    echo "=================== $label ==================="
    if "$@"; then
        echo "$label: PASS"
    else
        echo "$label: FAIL"
        rc=1
    fi
    echo
}

run "check_assembly_v6 (default build)" "$PY" "$ROOT/cad/v6/check_assembly_v6.py"
run "check_assembly_v6 (ARMS=1)" env ARMS=1 "$PY" "$ROOT/cad/v6/check_assembly_v6.py"

report() {
    label=$1
    shift
    echo "=================== $label (report) ==================="
    "$@" || echo "$label: findings above -- check them in the slice preview"
    echo
}

report "audit_torso" "$PY" "$ROOT/cad/v6/audit_torso.py"
report "audit_ankle_foot" "$PY" "$ROOT/cad/v6/audit_ankle_foot.py"

if [ "$rc" -eq 0 ]; then
    echo "ALL CHECKS PASS"
else
    echo "CHECKS FAILED -- do not print"
fi
exit "$rc"
