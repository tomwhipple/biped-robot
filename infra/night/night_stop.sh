#!/usr/bin/env bash
# 07:00 hard stop for the nightly training window (cron, ON MIRA). SIGTERM
# is safe: train_mjx.py saves params.pkl at every eval callback, so at most
# the last few minutes of progress are lost. state -> state.done marks the
# run collectible by infra/night_collect.sh.
set -uo pipefail
N="$HOME/code/robot-mjx/night"
exec >> "$N/night.log" 2>&1
[[ -f "$N/state" ]] || exit 0
PID=$(sed -n 's/^PID=\([0-9]*\)$/\1/p' "$N/state")
OUT=$(sed -n 's/^OUT=//p' "$N/state")
if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
  echo "$(date) 07:00 hard stop: SIGTERM $PID (last eval checkpoint stands)"
  kill "$PID" 2>/dev/null
  sleep 20
  kill -9 "$PID" 2>/dev/null || true
else
  echo "$(date) 07:00 check: training already finished"
fi
mv -f "$N/state" "$N/state.done.${OUT:-unknown}"
# migrate any CPU-stranded ollama model back to the freed GPU (see
# night_run.sh restore_ollama for the why)
STRANDED=$(curl -s -m 10 localhost:11434/api/ps | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for m in d.get('models', []):
    if m.get('size_vram', 0) < m.get('size', 1):
        print(m['name'])
" 2>/dev/null)
for M in $STRANDED; do
  echo "$(date) migrating CPU-stranded ollama model $M back to GPU"
  ollama stop "$M" 2>/dev/null || true
  curl -s -m 300 localhost:11434/api/generate -d "{\"model\":\"$M\"}" \
    >/dev/null 2>&1 || true
done
