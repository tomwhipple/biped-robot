#!/usr/bin/env bash
# Twin crouch check: run the deployed-form student through the firmware stack
# (sil_twin, measured servo lag + dead time) with the crouch1 ramp, log every
# joint (link/crouch_probe.py, ARM|POSE beacons + guards) and print hip yaw,
# knees and the torso-height proxy at stand / hold / after.
#   sim/twin_crouch_check.sh <run_name> [out_dir]
# 2026-09-11: the v37knee_b student "crouched" on the robot by pinching its
# hip yaws (L -7 / R +8 deg) with no descent -- this makes that visible
# before a flash.
set -uo pipefail
# Root = the checkout this script lives in (the runner invokes the copy in
# ~/code/robot-mjx, where the student's run dir is); all paths absolute.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
RUN=${1:?run name}; OUT=${2:-$ROOT/sim/runs/$RUN}; mkdir -p "$OUT"
PY="$ROOT/.venv/bin/python"
PORT=${PORT:-4611}; TLM=${TLM:-4612}
MUJOCO_GL=egl JAX_PLATFORMS=cpu "$PY" sim/sil_twin.py --run-name "$RUN" --xml sim/bimo_biped_v5body.xml \
  --nominal --act-lag-hz 2.0 --act-delay-ticks 4 --stream-port 0 --port $PORT --tlm-port $TLM \
  --duration 60 --record "$OUT/twin_crouch1.mp4" > "$OUT/twin_crouch1_twin.log" 2>&1 &
TWIN=$!
for i in $(seq 1 60); do grep -q '"ready":true' "$OUT/twin_crouch1_twin.log" 2>/dev/null && break; sleep 1; done; sleep 2
"$PY" link/crouch_probe.py --host 127.0.0.1 --port $PORT --tlm-port $TLM \
  --csv "$OUT/twin_crouch1_beacons.csv" > "$OUT/twin_crouch1_probe.log" 2>&1
kill -INT $TWIN 2>/dev/null; wait $TWIN 2>/dev/null   # INT, not TERM: the twin flushes --record in its finally
"$PY" - "$OUT/twin_crouch1_beacons.csv" << 'PY'
import csv, math, sys
p=sys.argv[1]; rows=[r for r in csv.DictReader(open(p)) if r.get("L_yaw")]
if not rows: print("twin_crouch_check: no pose rows"); sys.exit(1)
t0=float(rows[0]["t"]); J=["L_yaw","R_yaw","L_knee","R_knee","L_ankle","R_ankle","L_pitch","R_pitch"]
scale=1.0 if max(abs(float(r[j])) for r in rows for j in J)>3.2 else 180/math.pi
def mean(a,b):
    s=[r for r in rows if a<=float(r["t"])-t0<b]; return {j: sum(float(r[j]) for r in s)/max(1,len(s))*scale for j in J}, len(s)
print("twin crouch1 joint means (deg)  " + "  ".join(f"{j:>7s}" for j in J))
for ph,(a,b) in {"settle stand":(2.5,4.5),"hold 0.7":(9.0,10.5),"tail":(13.0,14.5)}.items():
    m,n=mean(a,b); print(f"  {ph:13s} n={n:3d} " + "  ".join(f"{m[j]:+7.1f}" for j in J))
m0,_=mean(2.5,4.5); m1,_=mean(9.0,10.5)
print(f"TWIN_CROUCH: yaw pinch (R-L) {((m1['R_yaw']-m1['L_yaw'])-(m0['R_yaw']-m0['L_yaw'])):+.1f} deg, knee bend {0.5*((m1['L_knee']-m0['L_knee'])+(m1['R_knee']-m0['R_knee'])):+.1f} deg")
PY
