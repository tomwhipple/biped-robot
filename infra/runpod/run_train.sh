#!/usr/bin/env bash
# run_train.sh -- provision a RunPod GPU pod and launch an MJX training run.
#
# Usage:
#   ./run_train.sh --getup --out mjx_getup_v1 --steps 150000000 \
#                  --envs 2048 --batch-size 1024 --unroll 16 \
#                  --minibatches 32 --updates 4 --num-evals 30
#
# All args are passed through to sim/mjx/train_mjx.py. The pod is KEPT running
# (training is long); endpoint + pod id land in infra/runpod/.last_pod so a
# watcher can poll progress and tear down afterwards:
#   source infra/runpod/.last_pod
#   ssh -i ~/.runpod/ssh/runpodctl-ssh-key -p $PORT root@$IP 'tail /root/robot/sim/runs/*.log'
#   runpodctl pod delete $POD_ID
#
# Safety: --terminate-after now+12h means a forgotten pod self-destructs.
# Reuses the SSH/stock/readiness lessons from run_poc.sh (see README.md).
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SSHK="$HOME/.runpod/ssh/runpodctl-ssh-key"
GPU_PREFS=("NVIDIA GeForce RTX 4090" "NVIDIA RTX 4000 Ada Generation" \
           "NVIDIA A40" "NVIDIA L40S" "NVIDIA GeForce RTX 3090")
SSH_BASE=(-i "$SSHK" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
          -o BatchMode=yes -o ConnectTimeout=15)
KILL_AT="$(date -u -v+12H +%Y-%m-%dT%H:%M:%SZ)"

ssh_t() { local t="$1"; shift; ssh "$@" & local p=$!
  ( sleep "$t"; kill -9 "$p" 2>/dev/null ) & local w=$!
  wait "$p" 2>/dev/null; local rc=$?; kill "$w" 2>/dev/null; return "$rc"; }

echo "=== creating training pod (auto-terminate $KILL_AT) ==="
POD_ID=""
for gpu in "${GPU_PREFS[@]}"; do
  echo "--- trying GPU: $gpu"
  OUT=$(runpodctl pod create --name robot-train --template-id runpod-torch-v280 \
        --gpu-id "$gpu" --cloud-type SECURE --container-disk-in-gb 30 \
        --ports "22/tcp" --terminate-after "$KILL_AT" 2>&1)
  POD_ID=$(echo "$OUT" | python3 -c "import sys,json
try: print(json.load(sys.stdin).get('id',''))
except: print('')" 2>/dev/null)
  [[ -n "$POD_ID" ]] && { GPU_GOT="$gpu"; echo "    got: $gpu"; break; }
done
[[ -z "$POD_ID" ]] && { echo "!!! could not create pod:"; echo "$OUT" | head -3; exit 1; }

echo "=== waiting for SSH ==="
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
[[ "$READY" == "0" ]] && { echo "!!! SSH never came up; deleting pod"; \
  runpodctl pod delete "$POD_ID" >/dev/null 2>&1; exit 1; }

cat > "$HERE/.last_pod" <<EOF
POD_ID=$POD_ID
IP=$IP
PORT=$PORT
EOF
echo ">>> endpoint saved to infra/runpod/.last_pod"

echo "=== syncing code ==="
rsync -aq -e "ssh ${SSH_BASE[*]} -p $PORT" \
  "$REPO/sim/mjx" "$REPO/sim/walker_env.py" "$REPO/sim/bimo_biped_v2.xml" \
  "root@$IP:/root/robot/sim/" --rsync-path="mkdir -p /root/robot/sim && rsync"
rsync -aq -e "ssh ${SSH_BASE[*]} -p $PORT" \
  "$REPO/cad/stl" "root@$IP:/root/robot/cad/" \
  --rsync-path="mkdir -p /root/robot/cad && rsync"

echo "=== installing deps (jax[cuda12] + mjx + brax) ==="
ssh_t 600 "${SSH_BASE[@]}" -p "$PORT" "root@$IP" \
  "pip install -q --break-system-packages 'jax[cuda12]' mujoco mujoco-mjx brax 2>&1 | tail -2; \
   python3 -c 'import jax; print(\"jax\", jax.__version__, jax.devices())'"

echo "=== launching training: $* ==="
ssh_t 60 "${SSH_BASE[@]}" -p "$PORT" "root@$IP" \
  "mkdir -p /root/robot/sim/runs && cd /root/robot/sim/mjx && \
   XLA_PYTHON_CLIENT_MEM_FRACTION=0.85 nohup python3 train_mjx.py $* \
   > /root/robot/sim/runs/train.log 2>&1 & echo \"launched pid \$!\""
echo ">>> training started on $GPU_GOT ($POD_ID). Poll:"
echo "    source infra/runpod/.last_pod && ssh -i $SSHK -p \$PORT root@\$IP 'tail -3 /root/robot/sim/runs/train.log'"
