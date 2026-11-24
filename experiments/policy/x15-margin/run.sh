#!/bin/bash
# run.sh JOBS OUTDIR PLANT_TERMS
#   JOBS: lines "run gain seed [group|-] [gyro|-]" (group: trace_x15 XGROUP; gyro: GYRO_SCALE, e.g. z=0)
#   8 episodes in parallel, nice 15, CPU only; an existing trace is not re-run.
# e.g. a ladder on plant D:  for g in 1.0 1.4 1.5 1.6 1.7 1.75 2.0; do for s in 0 1 2 3 4 5 6 7; do
#        echo "robot_legs_prec97_walk_sagdr_vol_i86_vref_s128r24 $g $s"; done; done > jobs.txt
#      ./run.sh jobs.txt /tmp/x15/D "sag_L=6.4,sag_R=2.9" && python analyze.py /tmp/x15/D /tmp/x15/D.csv
HERE="$(cd "$(dirname "$0")" && pwd)"
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= SHAPE_HZ=${SHAPE_HZ:-8} \
       GRADE_XML=${GRADE_XML:-bimo_biped_v6legs_meas_sole.xml} GRADE_PLANT_TERMS="$3"
PY=${PY:-$(cd "$HERE/../../.." && pwd)/.venv/bin/python}
mkdir -p "$2"
export HERE PY OUT="$2"
xargs -P 8 -L 1 sh -c '
  grp=${3:--}; gy=${4:--}; tag=""
  [ "$grp" != "-" ] && tag="${tag}_$grp"; [ "$gy" != "-" ] && tag="${tag}_gy$gy"
  f="$OUT/$(echo $0 | sed s/robot_legs_//)${tag}_g$1_s$2.npz"
  [ -f "$f" ] && exit 0
  [ "$grp" != "-" ] && export XGROUP=$grp; [ "$gy" != "-" ] && export GYRO_SCALE=$gy
  nice -n 15 "$PY" "$HERE/trace_x15.py" "$f" "$0" "$1" "$2" 2>&1 | grep -v Warning | tail -1' < "$1"
