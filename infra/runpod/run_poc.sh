#!/usr/bin/env bash
# run_poc.sh -- provision a RunPod pod, run the PoC benchmark, tear it down.
#
# Usage:
#   ./run_poc.sh gpu        # cheapest available secure GPU, torch + MJX bench
#   ./run_poc.sh cpu        # a CPU-only pod, torch bench
#   ./run_poc.sh gpu --keep # leave the pod running (skip teardown)
#
# Requires: runpodctl (brew install runpod/runpodctl/runpodctl), an API key in
# ~/.runpod/config.toml or $RUNPOD_API_KEY, and an SSH key registered with
# `runpodctl ssh add-key` (already done for this account).
#
# Lessons baked in:
#  * The pod API's `runtime`/`uptimeSeconds` fields stay null/0 long after the
#    container is actually reachable -- do NOT gate on them. Poll SSH directly.
#  * secure-cloud GPU stock is thin; we try a preference list until one deploys.
#  * macOS has no `timeout(1)`; we wrap ssh with a background-kill watchdog.
#  * The runpod pytorch image is PEP-668 externally-managed; poc.py installs
#    MJX with --break-system-packages (fine on a throwaway pod).
set -uo pipefail

MODE="${1:-gpu}"
KEEP=0; [[ "${2:-}" == "--keep" ]] && KEEP=1

HERE="$(cd "$(dirname "$0")" && pwd)"
POC="$HERE/poc.py"
SSHK="$HOME/.runpod/ssh/runpodctl-ssh-key"
IMAGE="runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404"
# Preference order: cheap -> pricier. Skipped automatically when out of stock.
GPU_PREFS=("NVIDIA RTX 4000 Ada Generation" "NVIDIA GeForce RTX 4090" \
           "NVIDIA A40" "NVIDIA GeForce RTX 3090" "NVIDIA L40S")

SSH_BASE=(-i "$SSHK" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
          -o BatchMode=yes -o ConnectTimeout=15)

# ssh_t <secs> <ssh args...> -- run ssh with a hard wall-clock timeout.
ssh_t() { local t="$1"; shift; ssh "$@" & local p=$!
  ( sleep "$t"; kill -9 "$p" 2>/dev/null ) & local w=$!
  wait "$p" 2>/dev/null; local rc=$?; kill "$w" 2>/dev/null; return "$rc"; }

pod_field() { runpodctl pod get "$1" 2>/dev/null \
  | python3 -c "import sys,json;print(json.load(sys.stdin).get('$2',''))"; }

POD_ID=""
cleanup() {
  [[ -z "$POD_ID" ]] && return
  if [[ "$KEEP" == "1" ]]; then
    echo ">>> --keep set; pod $POD_ID left RUNNING. Delete with: runpodctl pod delete $POD_ID"
  else
    echo ">>> deleting pod $POD_ID"; runpodctl pod delete "$POD_ID" >/dev/null 2>&1 \
      && echo ">>> deleted" || echo "!!! delete FAILED -- check: runpodctl pod list"
  fi
}
trap cleanup EXIT INT TERM

# Auto-terminate deadline: now + 2h, computed portably (macOS `date -v` and
# GNU `date -d` disagree; python3 is already a dependency of this script).
KILL_AT="$(python3 -c 'from datetime import datetime,timedelta,timezone; print((datetime.now(timezone.utc)+timedelta(hours=2)).strftime("%Y-%m-%dT%H:%M:%SZ"))')"

echo "=== creating $MODE pod (auto-terminate $KILL_AT) ==="
if [[ "$MODE" == "cpu" ]]; then
  OUT=$(runpodctl pod create --name poc-cpu --compute-type cpu --image "$IMAGE" \
        --ports "22/tcp" --terminate-after "$KILL_AT" 2>&1)
  POD_ID=$(echo "$OUT" | python3 -c "import sys,json
try: print(json.load(sys.stdin).get('id',''))
except: print('')" 2>/dev/null)
else
  for gpu in "${GPU_PREFS[@]}"; do
    echo "--- trying GPU: $gpu"
    OUT=$(runpodctl pod create --name poc-gpu --template-id runpod-torch-v280 \
          --gpu-id "$gpu" --cloud-type SECURE --container-disk-in-gb 20 \
          --ports "22/tcp" --terminate-after "$KILL_AT" 2>&1)
    POD_ID=$(echo "$OUT" | python3 -c "import sys,json
try: print(json.load(sys.stdin).get('id',''))
except: print('')" 2>/dev/null)
    [[ -n "$POD_ID" ]] && { echo "    got: $gpu"; break; }
  done
fi
[[ -z "$POD_ID" ]] && { echo "!!! could not create pod:"; echo "$OUT" | head -3; exit 1; }
echo ">>> pod $POD_ID (\$$(pod_field "$POD_ID" costPerHr)/hr)"

echo "=== waiting for SSH (polling ssh info + SSH directly, not the API runtime field) ==="
# The ip/port are not assigned the instant the pod is created, and the pod
# API's runtime field lies about readiness. Poll ssh info for the endpoint,
# then poll the endpoint itself, in one loop.
IP=""; PORT=""; READY=0
for i in $(seq 1 40); do
  if [[ -z "$IP" || -z "$PORT" ]]; then
    eval "$(runpodctl ssh info "$POD_ID" 2>/dev/null | python3 -c "import sys,json
try:
    d=json.load(sys.stdin); print(f\"IP={d.get('ip','')}; PORT={d.get('port','')}\")
except: print('IP=; PORT=')" 2>/dev/null)"
  fi
  if [[ -n "$IP" && -n "$PORT" ]]; then
    if ssh_t 25 "${SSH_BASE[@]}" -p "$PORT" "root@$IP" "true" 2>/dev/null; then
      READY=1; echo ">>> SSH up at root@$IP:$PORT (~$((i*15))s)"; break; fi
  fi
  sleep 15
done
[[ "$READY" == "0" ]] && { echo "!!! SSH never came up (endpoint root@${IP:-?}:${PORT:-?})"; exit 1; }

echo "=== copying + running poc.py ==="
scp "${SSH_BASE[@]}" -P "$PORT" "$POC" "root@$IP:/root/poc.py" >/dev/null 2>&1 \
  || { echo "!!! scp failed"; exit 1; }
POC_ARGS=""; [[ "$MODE" == "gpu" ]] && POC_ARGS="--mjx"
ssh_t 600 "${SSH_BASE[@]}" -p "$PORT" "root@$IP" "cd /root && python3 poc.py $POC_ARGS"
echo "=== poc exit rc=$? ==="
