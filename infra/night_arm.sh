#!/usr/bin/env bash
# Arm tonight's 23:00-05:00 training window on Mira (POLICY 2026-07-17:
# nightly Mira window instead of RunPod). Pushes current code + plant +
# night scripts, installs the two cron entries (idempotent), and leaves the
# train_mjx args armed. One-shot: night_run.sh consumes the args file.
#
# Usage: infra/night_arm.sh --precision --out precision_v3 --steps 300000000 ...
# Collect results next morning with: infra/night_collect.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
[[ $# -gt 0 ]] || { echo "usage: night_arm.sh <train_mjx.py args>"; exit 1; }

echo "=== syncing code + plant to mira ==="
rsync -aq "$REPO/sim/mjx" "$REPO/sim/walker_env.py" \
  "$REPO"/sim/bimo_biped_*.xml "$REPO/sim/terrain_mosaic.npz" \
  mira:code/robot-mjx/sim/
rsync -aq "$REPO/cad/stl" mira:code/robot-mjx/cad/
rsync -aq "$HERE/night/night_run.sh" "$HERE/night/night_stop.sh" \
  mira:code/robot-mjx/night/
ssh mira 'chmod +x code/robot-mjx/night/night_*.sh'

echo "=== installing cron entries (22:00 run / 07:00 stop) ==="
ssh mira 'TAB=$(crontab -l 2>/dev/null | grep -v "robot-mjx/night/night_" || true)
printf "%s\n0 22 * * * %s\n0 7 * * * %s\n" "$TAB" \
  "$HOME/code/robot-mjx/night/night_run.sh" \
  "$HOME/code/robot-mjx/night/night_stop.sh" | crontab -
crontab -l | grep night_'

OUT="job"
prev=""
for a in "$@"; do
  [[ "$prev" == "--out" ]] && OUT="$a"
  [[ "$a" == --out=* ]] && OUT="${a#--out=}"
  prev="$a"
done
QN="$(date +%m%d%H%M%S)-$OUT"
echo "=== queueing $QN: $* ==="
printf '%s\n' "$*" | ssh mira "mkdir -p code/robot-mjx/night/queue && cat > code/robot-mjx/night/queue/$QN"
echo ">>> tonight's queue (runs in this order from 22:00; no starts after"
echo ">>> 05:00, hard stop 07:00; collect with infra/night_collect.sh):"
ssh mira 'ls code/robot-mjx/night/queue | grep -v "^done$"'
