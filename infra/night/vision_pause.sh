#!/usr/bin/env bash
# Pause / resume Tom's camera-watcher classifier (the daytime GPU owner: llama3.2-vision
# via ollama) around the training window. Tom 2026-10-07: "disable the vision queue while
# training runs". The backlog waits in redis and drains after resume. While paused, the
# sentinel tells ~/.hermes/scripts/camera_watchdog.py the lag is intentional.
#   vision_pause.sh pause | resume
set -uo pipefail
C=camera-watcher-classifier-1
S=$HOME/code/robot-mjx/night/CLASSIFIER_PAUSED
case "${1:-}" in
  pause)
    if [[ "$(docker inspect -f '{{.State.Status}}' $C 2>/dev/null)" == "running" ]]; then
      docker pause $C >/dev/null && echo "$(date +%H:%M) paused $C"
    fi
    date -Iseconds > "$S"
    # drop the resident vision model so the GPU is free at once (reloads on demand after resume)
    ollama stop llama3.2-vision:latest 2>/dev/null || true ;;
  resume)
    if [[ "$(docker inspect -f '{{.State.Status}}' $C 2>/dev/null)" == "paused" ]]; then
      docker unpause $C >/dev/null && echo "$(date +%H:%M) resumed $C"
    fi
    rm -f "$S" ;;
  *) echo "usage: $0 pause|resume"; exit 2 ;;
esac
