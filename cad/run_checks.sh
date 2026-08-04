#!/bin/sh
# Every automated interference check, in one command.
#
#   sh cad/run_checks.sh          # exit 0 = clear, 1 = something interferes
#   sh cad/run_checks.sh 25       # finer ROM sampling (slower)
#
# Two checks, because they cover different things and neither subsumes the other:
#
#   check_assembly.py       build123d solids at POSE EXTREMES plus a handful of
#                           combined poses, and the insertion-path scans. Catches
#                           anything the articulated FCStd does not model
#                           (torso: battery lift-out / board mount, servo
#                           mocks, screw reach).
#   freecad_rom_collide.py  the real articulated assembly, every joint SWEPT
#                           across its whole travel. Catches fouls that live in
#                           the MIDDLE of the range, which endpoint checks miss
#                           by construction.
#
# The two run under different interpreters -- check_assembly needs the venv's
# build123d, freecad_rom_collide needs FreeCAD's own Python -- which is why this
# is a shell script and not one more Python entry point.
set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
FREECADCMD=${FREECADCMD:-/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd}
STEPS=${1:-}

rc=0

echo "=================== check_assembly.py (pose extremes) ==================="
if "$ROOT/.venv/bin/python" "$ROOT/cad/check_assembly.py"; then
    echo "check_assembly: PASS"
else
    echo "check_assembly: FAIL"
    rc=1
fi

echo
echo "================= freecad_rom_collide.py (swept ROM) ===================="
if [ ! -x "$FREECADCMD" ]; then
    echo "freecad_rom_collide: SKIPPED -- no freecadcmd at $FREECADCMD"
    echo "  (set FREECADCMD=/path/to/freecadcmd to run it)"
    rc=1        # a skipped check is not a passed check
else
    # freecadcmd chatters on stderr (3Dconnexion) and spews carriage-returned
    # "(42 %)" progress bars on stdout, which bury the report. Strip those, but
    # take the exit status from freecadcmd itself, not from the filter.
    out=$(mktemp)
    "$FREECADCMD" "$ROOT/cad/freecad_rom_collide.py" $STEPS >"$out" 2>/dev/null
    sweep_rc=$?
    tr '\r' '\n' <"$out" | grep -vE '\([0-9]+ %\)|^\s*$|^(Importing|Postprocessing|Recompute|MbD:|Time =|FreeCAD 1|\(C\) 2|FreeCAD is free)'
    rm -f "$out"
    if [ "$sweep_rc" -eq 0 ]; then
        echo "freecad_rom_collide: PASS"
    else
        echo "freecad_rom_collide: FAIL"
        rc=1
    fi
fi

echo
if [ "$rc" -eq 0 ]; then
    echo "ALL CHECKS PASS"
else
    echo "CHECKS FAILED -- do not print"
fi
exit "$rc"
