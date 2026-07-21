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
# user rule (2026-07-21): restart/re-warm ollama after training ends
if ! sudo -n systemctl restart ollama 2>/dev/null; then
  curl -s -m 180 localhost:11434/api/generate \
       -d '{"model":"llama3.2-vision:latest"}' >/dev/null 2>&1 || true
  curl -s -m 60 localhost:11434/api/embed \
       -d '{"model":"nomic-embed-text:latest","input":"warmup"}' \
       >/dev/null 2>&1 || true
  echo "$(date) ollama models re-warmed"
else
  echo "$(date) ollama service restarted"
fi
