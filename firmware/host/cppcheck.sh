#!/usr/bin/env bash
# cppcheck over the firmware + link sources, failing on ANY finding.
#
# cppcheck's --error-exitcode only fires on *error* severity, not on the
# warning/style/unusedFunction findings this gate is meant to catch, so a
# warning-only regression would otherwise pass silently. This wrapper runs the
# whole-program pass and fails if a single warning or error line is emitted.
#
# Whole-program view is what makes --enable=unusedFunction meaningful: every
# non-static function with no caller across the entire TU set is flagged, which
# is the cross-TU dead-code signal a per-file clang-tidy or -Wall cannot see.
set -euo pipefail

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HOST_DIR/../.." && pwd)"

# The firmware/console core is deliberately no-STL and raw-buffer; the
# suppressions file documents each id that is design-noise rather than a bug.
# --inline-suppr honours the // cppcheck-suppress comments in the sources.
LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT

cppcheck --inline-suppr \
    --enable=warning,style,unusedFunction --inconclusive \
    --suppress=missingIncludeSystem \
    --suppressions-list="$HOST_DIR/cppcheck-suppressions.txt" \
    --std=c++17 --language=c++ \
    -I"$ROOT/firmware/components/scsbus/include" \
    -I"$ROOT/firmware/components/linkproto/include" \
    -I"$ROOT/firmware/components/obs/include" \
    -I"$ROOT/firmware/components/policy/include" \
    -I"$ROOT/firmware/components/imu/include" \
    -I"$ROOT/firmware/components/battguard/include" \
    -I"$ROOT/link" \
    "$ROOT/firmware/components" "$ROOT/firmware/host" \
    "$ROOT/firmware/main" "$ROOT/link" \
    2>&1 | tee "$LOG"

# Fail on any finding line. cppcheck severities are error/warning/style/
# performance/portability; the "Checking ..." progress and the toomanyconfigs
# information note are not findings and must not fail the gate.
if grep -qE ": (error|warning|style|performance|portability):" "$LOG"; then
    echo "cppcheck: findings above" >&2
    exit 1
fi
echo "cppcheck: clean"
