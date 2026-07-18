#!/usr/bin/env bash
# train_launcher.sh -- compute-aware dispatcher for MJX RL training.
#
# Usage:
#   ./train_launcher.sh [--force-runpod|--force-mira] [--dry-run] <train_mjx.py args...>
#
# Policy: train on the local GPU box "Mira" (RTX 4070 Ti) when it's idle;
# otherwise burst to RunPod. Mira is reachable as `ssh mira` (key auth already
# set up). Mira's ollama server often squats VRAM there -- if a model is
# loaded we fall through to RunPod rather than fight it for memory.
#
# Decision (unless overridden by --force-mira/--force-runpod): Mira is chosen
# iff ssh is reachable AND gpu util < 20% AND free VRAM >= 10000 MiB AND no
# ollama model is currently loaded.
#
# On the Mira path this rsyncs the sim code + assets to mira:~/code/robot-mjx
# (mirroring the repo's own sim/... , cad/stl/... layout) and launches
# train_mjx.py in the background there, recording the PID.
# On the RunPod path this just hands off to infra/runpod/run_train.sh, which
# provisions a pod, syncs code, and launches training (see its README for the
# hard-won SSH/readiness/teardown lessons this script reuses in spirit).
#
# Either way, infra/.last_train is written so infra/watch_train.sh can poll
# for completion and pull results back.
#
# --dry-run: does the real Mira probe over ssh (read-only: nvidia-smi +
# ollama ps) but launches nothing -- no rsync, no remote process, no pod.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
LAST_TRAIN="$HERE/.last_train"
MIRA_REMOTE="~/code/robot-mjx"

# ssh_t <secs> <ssh args...> -- run ssh with a hard wall-clock timeout.
# (macOS has no timeout(1); ConnectTimeout alone doesn't bound a stuck auth.)
ssh_t() { local t="$1"; shift; ssh "$@" & local p=$!
  ( sleep "$t"; kill -9 "$p" 2>/dev/null ) & local w=$!
  wait "$p" 2>/dev/null; local rc=$?; kill "$w" 2>/dev/null; return "$rc"; }

FORCE=""       # "" | mira | runpod
DRY_RUN=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --force-mira)   FORCE="mira" ;;
    --force-runpod) FORCE="runpod" ;;
    --dry-run)      DRY_RUN=1 ;;
    *)              ARGS+=("$a") ;;
  esac
done

# --- parse --out from the passthrough args (train_mjx.py defaults to
# mjx_cmd_v1 when --out is absent) ------------------------------------------
OUT="mjx_cmd_v1"
for ((i = 0; i < ${#ARGS[@]}; i++)); do
  a="${ARGS[$i]}"
  if [[ "$a" == "--out" && $((i + 1)) -lt ${#ARGS[@]} ]]; then
    OUT="${ARGS[$((i + 1))]}"
  elif [[ "$a" == --out=* ]]; then
    OUT="${a#--out=}"
  fi
done
echo ">>> --out resolved to: $OUT"

TRAIN_ARGS_STR="$(printf '%q ' "${ARGS[@]}")"

# =============================================================================
# Step 1: probe Mira (single ssh call: gpu stats + ollama ps)
# =============================================================================
DECISION=""
REASON=""

probe_mira() {
  local probe
  probe=$(ssh_t 8 -o ConnectTimeout=8 -o BatchMode=yes mira '
    nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits
    echo "---OLLAMA---"
    ollama ps 2>/dev/null | tail -n +2
  ' 2>/dev/null)
  local rc=$?
  printf '%s' "$probe"
  return $rc
}

if [[ "$FORCE" == "runpod" ]]; then
  DECISION="runpod"
  REASON="--force-runpod given"
elif [[ "$FORCE" == "mira" ]]; then
  DECISION="mira"
  REASON="--force-mira given"
else
  echo ">>> probing mira over ssh (nvidia-smi + ollama ps, ConnectTimeout 8)..."
  PROBE_OUT="$(probe_mira)"
  PROBE_RC=$?
  if [[ $PROBE_RC -ne 0 || -z "$PROBE_OUT" ]]; then
    DECISION="runpod"
    REASON="mira unreachable over ssh (rc=$PROBE_RC)"
  else
    GPU_LINE="$(printf '%s\n' "$PROBE_OUT" | sed -n '1p')"
    OLLAMA_LINES="$(printf '%s\n' "$PROBE_OUT" | awk '/---OLLAMA---/{f=1; next} f')"
    UTIL="$(printf '%s' "$GPU_LINE" | awk -F',' '{gsub(/ /,"",$1); print $1}')"
    MEM_USED="$(printf '%s' "$GPU_LINE" | awk -F',' '{gsub(/ /,"",$2); print $2}')"
    MEM_TOTAL="$(printf '%s' "$GPU_LINE" | awk -F',' '{gsub(/ /,"",$3); print $3}')"
    echo ">>> mira gpu: util=${UTIL}% used=${MEM_USED}MiB total=${MEM_TOTAL}MiB"
    if [[ -z "$UTIL" || -z "$MEM_USED" || -z "$MEM_TOTAL" ]]; then
      DECISION="runpod"
      REASON="could not parse nvidia-smi output from mira"
    else
      FREE=$((MEM_TOTAL - MEM_USED))
      # Resident ollama models are NOT a veto (user, 2026-07-17): nvidia-smi's
      # free number already accounts for them, so a small resident model (e.g.
      # nomic-embed, ~0.9 GB) coexists with training fine. A big one (llama3.2
      # -vision, ~9 GB) fails the free-VRAM gate below on its own. Only note
      # the consequence: while training holds ~10.4 GB, a LATER attempt to
      # load a big ollama model falls back to CPU inference until the run ends.
      OLLAMA_MODEL="$(printf '%s\n' "$OLLAMA_LINES" | awk 'NF{print $1; exit}')"
      if [[ "$UTIL" -ge 20 ]]; then
        DECISION="runpod"
        REASON="mira gpu util ${UTIL}% >= 20%"
      elif [[ "$FREE" -lt 10000 ]]; then
        DECISION="runpod"
        REASON="mira free vram ${FREE}MiB < 10000MiB"
        if [[ -n "$OLLAMA_MODEL" ]]; then
          REASON+=" (ollama '$OLLAMA_MODEL' resident; 'ssh mira \"ollama stop $OLLAMA_MODEL\"' frees it)"
        fi
      else
        DECISION="mira"
        REASON="mira idle: util=${UTIL}% free=${FREE}MiB"
        if [[ -n "$OLLAMA_MODEL" ]]; then
          echo ">>> note: ollama '$OLLAMA_MODEL' stays resident alongside training;"
          echo "    big-model loads during the run will fall back to CPU inference"
        fi
      fi
    fi
  fi
fi

echo "=== DECISION: $DECISION  ($REASON) ==="

# =============================================================================
# Step 2a: Mira path
# =============================================================================
run_mira() {
  echo "=== discovering training venv on mira ==="
  local venv_probe pybin
  venv_probe=$(ssh_t 15 -o ConnectTimeout=8 -o BatchMode=yes mira "
    if [[ -x $MIRA_REMOTE/.venv/bin/python ]]; then
      echo FOUND:$MIRA_REMOTE/.venv/bin/python
    elif [[ -x $MIRA_REMOTE/venv/bin/python ]]; then
      echo FOUND:$MIRA_REMOTE/venv/bin/python
    else
      echo NOTFOUND
      echo '--- contents of $MIRA_REMOTE:'
      ls -la $MIRA_REMOTE 2>&1
    fi
  " 2>/dev/null)
  if [[ "$venv_probe" != FOUND:* ]]; then
    echo "!!! no training venv found on mira (checked .venv/bin/python and venv/bin/python)"
    echo "$venv_probe"
    return 1
  fi
  pybin="${venv_probe#FOUND:}"
  echo ">>> using remote python: $pybin"

  if [[ $DRY_RUN -eq 1 ]]; then
    echo "--- [dry-run] would rsync to mira:~/code/robot-mjx/ (relative layout):"
    echo "    sim/mjx sim/walker_env.py sim/bimo_biped_v2.xml sim/bimo_biped_v2_asbuilt.xml cad/stl"
    echo "--- [dry-run] would launch on mira:"
    echo "    cd ~/code/robot-mjx/sim/mjx && XLA_PYTHON_CLIENT_MEM_FRACTION=0.90 nohup $pybin train_mjx.py $TRAIN_ARGS_STR > ~/code/robot-mjx/sim/runs/train.log 2>&1 < /dev/null &"
    echo "--- [dry-run] would write $LAST_TRAIN with HOST=mira, PID=<pid>, OUT=$OUT, PYBIN=$pybin"
    return 0
  fi

  echo "=== syncing code to mira:~/code/robot-mjx/ ==="
  ( cd "$REPO" && rsync -aq -R \
      -e "ssh -o ConnectTimeout=8 -o BatchMode=yes" \
      sim/mjx sim/walker_env.py sim/bimo_biped_v2.xml sim/bimo_biped_v2_asbuilt.xml cad/stl \
      "mira:$MIRA_REMOTE/" \
      --rsync-path="mkdir -p $MIRA_REMOTE && rsync" )

  echo "=== launching training on mira: $TRAIN_ARGS_STR ==="
  local launch_out pid
  launch_out=$(ssh_t 60 -o ConnectTimeout=8 -o BatchMode=yes mira "
    mkdir -p $MIRA_REMOTE/sim/runs && cd $MIRA_REMOTE/sim/mjx && \
    XLA_PYTHON_CLIENT_MEM_FRACTION=0.90 nohup $pybin train_mjx.py $TRAIN_ARGS_STR \
    > $MIRA_REMOTE/sim/runs/train.log 2>&1 < /dev/null & echo LAUNCHED_PID=\$!
  ")
  pid="$(printf '%s\n' "$launch_out" | sed -n 's/.*LAUNCHED_PID=//p' | tail -1)"
  if [[ -z "$pid" ]]; then
    echo "!!! failed to capture PID from mira launch:"
    echo "$launch_out"
    return 1
  fi
  echo ">>> training launched on mira, pid=$pid"

  cat > "$LAST_TRAIN" <<EOF
HOST=mira
PID=$pid
OUT=$OUT
PYBIN=$pybin
STARTED=$(date +%s)
EOF
  echo ">>> wrote $LAST_TRAIN"
}

# =============================================================================
# Step 2b: RunPod path
# =============================================================================
run_runpod() {
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "--- [dry-run] would run: $HERE/runpod/run_train.sh $TRAIN_ARGS_STR"
    echo "--- [dry-run] would write $LAST_TRAIN with HOST=runpod, OUT=$OUT"
    return 0
  fi
  "$HERE/runpod/run_train.sh" "${ARGS[@]}"
  local rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "!!! run_train.sh failed (rc=$rc)"
    return $rc
  fi
  cat > "$LAST_TRAIN" <<EOF
HOST=runpod
OUT=$OUT
STARTED=$(date +%s)
EOF
  echo ">>> wrote $LAST_TRAIN"
}

if [[ "$DECISION" == "mira" ]]; then
  run_mira
  rc=$?
else
  run_runpod
  rc=$?
fi

exit $rc
