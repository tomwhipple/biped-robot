#!/usr/bin/env bash
# Safety net for "simulations running all night, every night" (Tom
# 2026-10-21; cron 21:30, ON MIRA, as claw). The morning planner
# (plan_morning.sh) queues the night; if it failed, or queued too little,
# this tops the queue up to MIN_JOBS from night/fallback.job -- the job the
# planner judged most worth repeating -- with fresh seeds and unique --out
# names, so the GPU is never idle for want of a plan. Never touches a queue
# that is already full. Installed copy: ~/code/robot-mjx/night/.
set -uo pipefail
N=${NIGHT_DIR:-$HOME/code/robot-mjx/night}   # NIGHT_DIR / NO_NOTIFY: for testing
Q=$N/queue
FB=$N/fallback.job
MIN_JOBS=5
TOM=mattermost:xhtnq11nhjn1jpytz1sj4zzimr
exec >> "$N/planner.log" 2>&1
have=$(ls "$Q" | grep -cE '^[0-9]' || true)
(( have >= MIN_JOBS )) && exit 0
echo "=== $(date) ensure_queue: $have job(s) queued, want $MIN_JOBS ==="
if [[ ! -s $FB ]]; then
  echo "!!! no $FB to top up from"
  [[ -n ${NO_NOTIFY:-} ]] || "$HOME/.local/bin/hermes" send -q -t "$TOM" \
    "Get-up loop: only $have job(s) queued for tonight and no fallback.job -- the GPU will idle after them." || true
  exit 1
fi
BR=$(sed -n 's/^BRANCH=//p' "$FB" | head -1)
ARGS=$(grep -vE '^(BRANCH=|[[:space:]]*#|[[:space:]]*$)' "$FB" | tr '\n' ' ')
OUT=$(grep -oE -- '--out[= ][^ ]+' <<<"$ARGS" | awk -F'[= ]' '{print $2}')
[[ -n $OUT ]] || { echo "!!! $FB has no --out"; exit 1; }
ARGS=$(sed -E 's/ --seed[= ][0-9]+//' <<<"$ARGS")
added=()
k=0
while (( have < MIN_JOBS )); do
  k=$((k + 1))
  NEW="${OUT}_fb$(date +%m%d)s$k"
  # unique: never reuse a name a run or a queued job already has
  if ls "$Q" "$Q/done" 2>/dev/null | grep -q -- "-$NEW\$" \
     || [[ -e $HOME/code/robot/sim/runs/$NEW || -e $HOME/code/robot-mjx/sim/runs/$NEW ]]; then
    continue
  fi
  JA=$(sed -E "s/--out[= ][^ ]+/--out $NEW/" <<<"$ARGS")
  printf 'BRANCH=%s\n# ensure_queue.sh %s: fallback.job (%s) with seed %d -- the planner queued only %d\n%s --seed %d\n' \
    "${BR:-main}" "$(date +%F)" "$OUT" "$((100 + k))" "$have" "$JA" "$((100 + k))" \
    > "$Q/$(date +%m%d)2359$(printf %02d "$k")-$NEW"
  added+=("$NEW")
  have=$((have + 1))
done
echo "queued ${added[*]}"
[[ -n ${NO_NOTIFY:-} ]] || "$HOME/.local/bin/hermes" send -q -t "$TOM" \
  "Get-up loop: the planner left the night short; ensure_queue.sh added ${#added[@]} fallback job(s): ${added[*]}" || true
