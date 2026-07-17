#!/usr/bin/env bash
# precision_round.sh -- one command, one completion message: launch a training
# round (mira-or-runpod, whichever is picked by train_launcher.sh) and then
# watch it through to completion (pulling results back / tearing down a pod).
#
# Meant to be backgrounded, e.g.:
#   nohup ./infra/precision_round.sh --out mjx_cmd_v2 --steps 150000000 \
#       > infra/precision_round.log 2>&1 &
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

"$HERE/train_launcher.sh" "$@" && exec "$HERE/watch_train.sh"
