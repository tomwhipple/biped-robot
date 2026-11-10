#!/bin/sh
# The weight-transfer gate (#259), every table in README.md, CPU only (~3 min on 32 cores).
#   sh experiments/policy/weight-transfer-gate/run_all.sh            # from the repo root
# Writes results/*.md (and check1.json / check2.json); hw_sessions/ and the prec16 run are read
# from the main checkout (GATE_MAIN, default /home/claw/code/robot).
set -e
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES=
cd "$(dirname "$0")"
PY=../../../.venv/bin/python
mkdir -p results
$PY check1_oneleg.py --plants oneleg:fit,train,train:lash0.5,train:lash2.0,grade --json results/check1.json --md results/check1.md
$PY check2_replay.py --schedules record,script,probe --seeds 8 --json results/check2.json --md results/check2.md
$PY check2_replay.py --fk-trace > results/check2_fk_trace.md
# attribution probes: one opt-in term set at a time (gate_common.apply_terms)
for T in "lash_fit" "torso=0.702,com_y=-0.0068" "lash=2.6" "roll_k=0.3" "dead=40" "torso=0.6,legs=1.2"; do
  $PY check1_oneleg.py --plants train,train:lash2.0,grade --terms "$T" --md "results/probe_check1_$T.md" > /dev/null
  $PY check2_replay.py --schedules record --seeds 4 --plants grade,train --terms "$T" --md "results/probe_check2_$T.md" > /dev/null
done
