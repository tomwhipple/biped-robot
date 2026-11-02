#!/usr/bin/env bash
# Start / stop the flight-recorder collector around a hardware session.
#
#   tools/flight_session.sh start [label]   # background collector, port 4220
#   tools/flight_session.sh stop            # close the session, print its summary
#   tools/flight_session.sh rotate          # close this file, next packet opens another
#   tools/flight_session.sh status
#
# Sessions land in <main checkout>/hw_sessions/<local date>/flight_<UTC>[_label].frec
# with a .summary.json beside each (tools/flight_collector.py). The robot side
# is `rec <mira-ip>` once over the tether (saved in NVS; docs/flight-recorder.md).
# FLIGHT_PORT / FLIGHT_OUT override the port and the directory.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
MAIN=$(dirname "$(git -C "$ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || echo "$ROOT/.git")")
PY=${PY:-$MAIN/.venv/bin/python}
[[ -x $PY ]] || PY=python3
PORT=${FLIGHT_PORT:-4220}
OUT=${FLIGHT_OUT:-$MAIN/hw_sessions/$(date +%F)}
RUN=${XDG_RUNTIME_DIR:-/tmp}/flight_collector.$PORT
PIDF=$RUN.pid

running() { [[ -f $PIDF ]] && kill -0 "$(cat "$PIDF")" 2>/dev/null; }

case "${1:-status}" in
  start)
    if running; then echo "already running (pid $(cat "$PIDF")), log $(cat "$RUN.log" 2>/dev/null)"; exit 0; fi
    mkdir -p "$OUT"
    LOG=$OUT/flight_collector_$(date -u +%Y%m%dT%H%M%SZ).log
    LABEL=()
    [[ -n ${2:-} ]] && LABEL=(--label "$2")
    nohup "$PY" "$HERE/flight_collector.py" --port "$PORT" --out "$OUT" "${LABEL[@]}" \
      > "$LOG" 2>&1 &
    echo $! > "$PIDF"
    echo "$LOG" > "$RUN.log"
    sleep 0.5
    if running; then echo "collector up: pid $(cat "$PIDF"), udp :$PORT -> $OUT (log $LOG)"
    else echo "collector failed to start:"; cat "$LOG"; rm -f "$PIDF"; exit 1; fi ;;
  stop)
    if ! running; then echo "not running"; rm -f "$PIDF"; exit 0; fi
    PID=$(cat "$PIDF"); kill -TERM "$PID"
    for _ in $(seq 50); do kill -0 "$PID" 2>/dev/null || break; sleep 0.1; done
    rm -f "$PIDF"
    LOG=$(cat "$RUN.log" 2>/dev/null)
    [[ -n $LOG ]] && grep "session closed" "$LOG" | tail -n 5
    for f in $(grep -o "[^ ]*\.frec" "$LOG" 2>/dev/null | sort -u); do
      S=${f%.frec}.summary.json
      [[ -f $S ]] && "$PY" -c "import json,sys; s=json.load(open(sys.argv[1])); \
print(s['file'], s['duration_s'], 's', s['packets'], 'pkts', s['tick_rates'], \
{b: (v['missing'], 'missing;', v['robot_drop_ring'], 'ring,', v['robot_drop_net'], 'net') for b, v in s['boots'].items()})" "$S"
    done ;;
  rotate)
    running && kill -HUP "$(cat "$PIDF")" && echo "rotated" || echo "not running" ;;
  status)
    if running; then echo "running: pid $(cat "$PIDF")"; tail -n 2 "$(cat "$RUN.log")"; else echo "not running"; fi ;;
  *) echo "usage: $0 start [label] | stop | rotate | status"; exit 2 ;;
esac
