#!/bin/sh
# The weight-transfer gate (#259), every table in README.md, CPU only (~5 min on 32 cores).
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
# round 2 (README "Round 2"): the sign check on 11-06, the 11-09 joints-replay perturbations, the
# series spring fitted to the 10-24 clamp and run through both checks, grade.py's roll opt-ins
for R in 1106t1 1106t2; do
  $PY check2_replay.py --record $R --schedules "" --seeds 0 --md results/signcheck_$R.md > /dev/null
done
$PY check2_replay.py --joint-variants --md results/r2_item2_joint_variants.md > /dev/null
$PY fit_series.py --md results/r2_item3_series_fit.md --json results/r2_item3_series_fit.json > /dev/null
for T in "lash=0,ser_hip=14.2" "lash=0,ser_ankle=27.8" "lash=0,ser_hip=40.1,ser_ankle=40.1"; do
  $PY check1_oneleg.py --plants train,grade --terms "$T" --md "results/r2_item3_check1_$T.md" > /dev/null
  $PY check2_replay.py --schedules record,probe --seeds 4 --terms "$T" --md "results/r2_item3_check2_$T.md" > /dev/null
done
$PY wt_grade.py 8 --md results/r2_item4_wt_grade.md --json results/r2_item4_wt_grade.json > /dev/null
