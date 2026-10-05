#!/usr/bin/env bash
# Reset the robot to the calibrated zero stand, gently (300 steps/s).
#   tools/reset_pose.sh
# Opens the tether directly (the CP2102 open reboots the board to bench --
# harmless here), enables torque, and poses all ten joints at their as-built
# zeros. Kills any console holder first (one reader per port).
set -uo pipefail
pkill -f 'serial_holder\.py' 2>/dev/null
sleep 0.5
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec sg dialout -c "$ROOT/.venv/bin/python - <<'EOF'
import time, serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
time.sleep(2.5)                       # DTR-reset boot
ser.read(65536)
ser.write(b'torque\r\n'); ser.flush(); time.sleep(0.8)
print(ser.read(4096).decode(errors='replace').strip()[-60:])
ser.write(b'pose 1693 2420 2044 1634 3516 1803 3533 2501 2050 3450 300\r\n')
ser.flush(); time.sleep(4.0)
print(ser.read(4096).decode(errors='replace').strip()[-60:])
ser.close()
print('reset to zero stand (torque ON, holding)')
EOF"
