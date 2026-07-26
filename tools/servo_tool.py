"""Bench tool for ST3215 bus servos, bypassing the vendor web UI.

WHY THIS EXISTS
---------------
The Waveshare board ships with demo firmware whose web UI cannot do the job:

1. **It is built for the wrong servo family.** `STSCTRL.h` in the vendor
   source has `SERVO_TYPE_SELECT = 2` (SC series, `ServoDigitalRange =
   1023`); the ST3215 is the ST series, range **4095**. So the UI's
   "Position+" commands ~1003 counts on a servo that has 4095 -- about a
   quarter of travel, which reads as "the buttons don't do much".
2. **The ID field only moves +/-1 per click** (`servotoSet += 1`, wrapping at
   250), so landing exactly on 9 or 10 means not overshooting once across
   nine clicks -- and an overshoot means clicking 240 more times to wrap.
3. **`setMode(id, 0)` in SC mode writes SC angle limits (20..1003) to
   registers 9/11** -- which the ST series uses for the *same* purpose. An
   ST3215 that has been through it is clamped to ~24 % of its travel until
   those registers are put back. `fixrange` below undoes exactly that.

Rather than fight it, this drives the servos with the real Feetech STS
protocol from the laptop. The vendor firmware has an undocumented-in-the-wiki
**`SERIAL_FORWARDING`** mode (`BOARD_DEV.h:157`) that bridges USB serial to
the 1 Mbaud servo bus byte-for-byte; `activeCtrl(14)` turns it on and
`activeCtrl(15)` turns it off, both reachable over HTTP. So:

    laptop --USB 115200--> ESP32 (dumb bridge) --1 Mbaud--> servo bus

Nothing is flashed. The vendor firmware stays exactly as it is.

Protocol facts, all verified against the vendor's own library rather than
assumed: ST series is **little-endian** (`SMS_STS.cpp`: `End = 0`; SC series
is `End = 1`), checksum is `~(id + len + inst + params) & 0xFF`, and "set
middle" is a write of **128** to the torque-enable register 40.

USAGE
-----
    # one-time per power cycle: put the board into bridge mode (needs WiFi)
    .venv/bin/python tools/servo_tool.py bridge on

    # then everything else runs over USB
    .venv/bin/python tools/servo_tool.py scan
    .venv/bin/python tools/servo_tool.py info 1
    .venv/bin/python tools/servo_tool.py setid 1 9
    .venv/bin/python tools/servo_tool.py move 9 2048
    .venv/bin/python tools/servo_tool.py fixrange 9
    .venv/bin/python tools/servo_tool.py bridge off

Join the board's WiFi (`ESP32_DEV` / `12345678`) for `bridge`; the USB cable
carries everything else. See docs/bringup-day1.md.
"""
from __future__ import annotations

import argparse
import glob
import sys
import time
import urllib.request

try:
    import serial
except ImportError:                                    # pragma: no cover
    sys.exit("pyserial missing:  .venv/bin/pip install pyserial")

BOARD_HTTP = "http://192.168.4.1"
USB_BAUD = 115200          # the CP2102 link; the ESP32 re-emits at 1 Mbaud

# --- ST3215 / SMS_STS register map (addresses shared with SCSCL where noted)
REG_ID = 5
REG_MIN_ANGLE = 9          # 2 B LE
REG_MAX_ANGLE = 11         # 2 B LE
REG_MODE = 33              # 1 B, 0=position 1=speed 2=pwm 3=step
REG_TORQUE_ENABLE = 40     # 1 B, 0=off 1=on 128="calibrate middle here"
REG_GOAL_ACC = 41          # 1 B
REG_GOAL_POSITION = 42     # 2 B LE
REG_GOAL_TIME = 44         # 2 B LE
REG_GOAL_SPEED = 46        # 2 B LE
REG_EEPROM_LOCK = 55       # 1 B, 0 = unlocked (writes persist), 1 = locked
REG_PRESENT_POSITION = 56  # 2 B LE
REG_PRESENT_VOLTAGE = 62   # 1 B, 0.1 V

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

POS_MAX = 4095             # ST3215 full scale. The SC series is 1023.
POS_MIDDLE = 2048


# =====================================================================
#  the vendor board's HTTP control surface (only used to flip the bridge)
# =====================================================================
def board_cmd(cmd_t: int, cmd_i: int, timeout: float = 4.0) -> None:
    """GET /cmd -- the handler reads four POSITIONAL args (CONNECT.h:189).

    cmd_t=0 -> activeID(cmd_i)   (relative step through the discovered list)
    cmd_t=1 -> activeCtrl(cmd_i) (the action table; 14/15 = bridge on/off)
    cmd_t=9 -> rescan the bus
    """
    url = f"{BOARD_HTTP}/cmd?a={cmd_t}&b={cmd_i}&c=0&d=0"
    urllib.request.urlopen(url, timeout=timeout).read()


# =====================================================================
#  Feetech STS protocol over the bridged USB port
# =====================================================================
class Bus:
    def __init__(self, port: str, timeout: float = 0.25):
        self.ser = serial.Serial(port, USB_BAUD, timeout=timeout)
        time.sleep(0.15)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    @staticmethod
    def _frame(sid: int, inst: int, params: bytes = b"") -> bytes:
        length = len(params) + 2
        body = bytes([sid, length, inst]) + params
        chk = (~sum(body)) & 0xFF
        return b"\xff\xff" + body + bytes([chk])

    def _txn(self, sid: int, inst: int, params: bytes = b"",
             expect: int = 0) -> bytes | None:
        """Send one packet and read the status reply. Returns the parameter
        bytes, or None on timeout/corruption (a missing servo is a timeout,
        not an error reply)."""
        self.ser.reset_input_buffer()
        self.ser.write(self._frame(sid, inst, params))
        self.ser.flush()

        # resync on the 0xFF 0xFF header rather than trusting alignment --
        # the bridge is a dumb byte pump and can hand us stale bytes
        deadline = time.time() + 0.35
        window = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            window = (window + b)[-2:]
            if window == b"\xff\xff":
                break
        else:
            return None

        head = self.ser.read(3)                       # id, len, err
        if len(head) < 3:
            return None
        rid, length, err = head[0], head[1], head[2]
        payload = self.ser.read(max(0, length - 2))
        chk = self.ser.read(1)
        if len(chk) != 1:
            return None
        calc = (~(rid + length + err + sum(payload))) & 0xFF
        if calc != chk[0] or rid != sid:
            return None
        if err:
            print(f"  ! servo {sid} error flags 0x{err:02x}", file=sys.stderr)
        if expect and len(payload) != expect:
            return None
        return payload

    # --- primitives -------------------------------------------------
    def ping(self, sid: int) -> bool:
        return self._txn(sid, INST_PING) is not None

    def read(self, sid: int, addr: int, n: int) -> bytes | None:
        return self._txn(sid, INST_READ, bytes([addr, n]), expect=n)

    def read_byte(self, sid: int, addr: int) -> int | None:
        r = self.read(sid, addr, 1)
        return None if r is None else r[0]

    def read_word(self, sid: int, addr: int) -> int | None:
        r = self.read(sid, addr, 2)
        return None if r is None else int.from_bytes(r, "little")

    def write(self, sid: int, addr: int, data: bytes) -> bool:
        return self._txn(sid, INST_WRITE, bytes([addr]) + data) is not None

    def write_byte(self, sid: int, addr: int, v: int) -> bool:
        return self.write(sid, addr, bytes([v & 0xFF]))

    def write_word(self, sid: int, addr: int, v: int) -> bool:
        return self.write(sid, addr, int(v).to_bytes(2, "little"))

    # --- EEPROM-guarded operations ----------------------------------
    def unlock(self, sid: int) -> bool:
        return self.write_byte(sid, REG_EEPROM_LOCK, 0)

    def lock(self, sid: int) -> bool:
        return self.write_byte(sid, REG_EEPROM_LOCK, 1)


# =====================================================================
#  commands
# =====================================================================
def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    cands = (sorted(glob.glob("/dev/cu.usbserial*"))
             + sorted(glob.glob("/dev/cu.SLAB_USBtoUART*"))
             + sorted(glob.glob("/dev/cu.wchusbserial*")))
    if not cands:
        sys.exit("no CP2102-style port found. Plug in USB, or pass --port. "
                 "macOS may need the CP210x driver (see docs/bringup-day1.md).")
    if len(cands) > 1:
        print(f"note: several ports, using {cands[0]} of {cands}")
    return cands[0]


def cmd_scan(bus: Bus, args) -> int:
    lo, hi = args.range
    print(f"scanning IDs {lo}-{hi} ...")
    found = []
    for sid in range(lo, hi + 1):
        if bus.ping(sid):
            pos = bus.read_word(sid, REG_PRESENT_POSITION)
            volt = bus.read_byte(sid, REG_PRESENT_VOLTAGE)
            mode = bus.read_byte(sid, REG_MODE)
            found.append(sid)
            print(f"  ID {sid:3d}   pos {pos if pos is not None else '?':>5}"
                  f"   mode {mode if mode is not None else '?'}"
                  f"   {volt/10 if volt is not None else '?'} V")
    print(f"{len(found)} servo(s): {found}" if found else "nothing responded.")
    return 0 if found else 1


def cmd_info(bus: Bus, args) -> int:
    sid = args.id
    if not bus.ping(sid):
        print(f"ID {sid} did not respond.")
        return 1
    lo = bus.read_word(sid, REG_MIN_ANGLE)
    hi = bus.read_word(sid, REG_MAX_ANGLE)
    print(f"ID {sid}")
    print(f"  mode            {bus.read_byte(sid, REG_MODE)}   (0=position)")
    print(f"  torque enable   {bus.read_byte(sid, REG_TORQUE_ENABLE)}")
    print(f"  angle limits    {lo} .. {hi}")
    print(f"  present pos     {bus.read_word(sid, REG_PRESENT_POSITION)}")
    v = bus.read_byte(sid, REG_PRESENT_VOLTAGE)
    print(f"  voltage         {v/10 if v is not None else '?'} V")
    if hi is not None and hi not in (0, POS_MAX):
        print(f"  ** travel is clamped to {lo}..{hi} of 0..{POS_MAX}. "
              f"Run:  fixrange {sid}")
    return 0


def cmd_setid(bus: Bus, args) -> int:
    old, new = args.old, args.new
    if not 0 <= new <= 253:
        sys.exit("new ID must be 0-253")
    if not bus.ping(old):
        print(f"ID {old} did not respond -- is exactly one servo connected?")
        return 1
    if old != new and bus.ping(new):
        print(f"refusing: ID {new} is already live on this bus.")
        return 1
    bus.unlock(old)
    ok = bus.write_byte(old, REG_ID, new)
    bus.lock(new)                     # the servo answers to `new` from here
    time.sleep(0.05)
    if ok and bus.ping(new):
        print(f"ID {old} -> {new}  (written to EEPROM, survives power-off)")
        return 0
    print(f"ID change failed; {old} still present: {bus.ping(old)}")
    return 1


def cmd_fixrange(bus: Bus, args) -> int:
    """Undo the vendor UI's SC-mode angle limits and restore ST full travel."""
    sid = args.id
    if not bus.ping(sid):
        print(f"ID {sid} did not respond.")
        return 1
    before = (bus.read_word(sid, REG_MIN_ANGLE), bus.read_word(sid, REG_MAX_ANGLE))
    bus.unlock(sid)
    bus.write_byte(sid, REG_MODE, 0)
    bus.write_word(sid, REG_MIN_ANGLE, 0)
    bus.write_word(sid, REG_MAX_ANGLE, POS_MAX)
    bus.lock(sid)
    after = (bus.read_word(sid, REG_MIN_ANGLE), bus.read_word(sid, REG_MAX_ANGLE))
    print(f"ID {sid}: angle limits {before[0]}..{before[1]} -> {after[0]}..{after[1]}, "
          f"mode 0 (position)")
    return 0 if after == (0, POS_MAX) else 1


def cmd_move(bus: Bus, args) -> int:
    sid, pos = args.id, args.position
    if not 0 <= pos <= POS_MAX:
        sys.exit(f"position must be 0-{POS_MAX} (ST3215 full scale)")
    if not bus.ping(sid):
        print(f"ID {sid} did not respond.")
        return 1
    bus.write_byte(sid, REG_TORQUE_ENABLE, 1)
    # acc(41), pos(42), time(44), speed(46) in one contiguous write
    payload = (bytes([args.acc])
               + int(pos).to_bytes(2, "little")
               + (0).to_bytes(2, "little")
               + int(args.speed).to_bytes(2, "little"))
    bus.write(sid, REG_GOAL_ACC, payload)
    time.sleep(args.settle)
    print(f"ID {sid} -> {pos}   (now at {bus.read_word(sid, REG_PRESENT_POSITION)})")
    return 0


def cmd_torque(bus: Bus, args) -> int:
    on = args.state == "on"
    ids = [args.id] if args.id is not None else list(range(0, 21))
    n = 0
    for sid in ids:
        if args.id is None and not bus.ping(sid):
            continue
        if bus.write_byte(sid, REG_TORQUE_ENABLE, 1 if on else 0):
            n += 1
    print(f"torque {'ON' if on else 'OFF'} for {n} servo(s)")
    return 0 if n else 1


def cmd_middle(bus: Bus, args) -> int:
    """Define the current physical position as 2048. Assembly-time only."""
    sid = args.id
    if not bus.ping(sid):
        print(f"ID {sid} did not respond.")
        return 1
    if not args.yes:
        print(f"This redefines ID {sid}'s CURRENT position as centre ({POS_MIDDLE}).\n"
              f"It belongs at the CAD-neutral pose during assembly, not on a "
              f"loose servo.\nRe-run with --yes if that is really where it is.")
        return 1
    bus.write_byte(sid, REG_TORQUE_ENABLE, 128)     # 128 = calibrate middle
    time.sleep(0.05)
    print(f"ID {sid} centre set; reads {bus.read_word(sid, REG_PRESENT_POSITION)}")
    return 0


def cmd_bridge(args) -> int:
    """Flip SERIAL_FORWARDING on the vendor firmware over HTTP."""
    action = 14 if args.state == "on" else 15
    try:
        board_cmd(1, action)
    except Exception as e:                             # noqa: BLE001
        sys.exit(f"could not reach {BOARD_HTTP}: {e}\n"
                 f"Join the board's WiFi (ESP32_DEV / 12345678) first.")
    if args.state == "on":
        print("bridge ON -- the OLED should read SERIAL_FORWARDING.\n"
              "USB serial is now wired straight to the 1 Mbaud servo bus.")
    else:
        print("bridge OFF -- the board is back to its normal web UI.")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--port", help="serial port (default: autodetect cu.usbserial*)")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("bridge", help="turn the board's USB<->bus bridge on/off (HTTP)")
    s.add_argument("state", choices=["on", "off"])

    s = sub.add_parser("scan", help="ping a range of IDs")
    s.add_argument("--range", type=int, nargs=2, default=(0, 20), metavar=("LO", "HI"))

    s = sub.add_parser("info", help="dump one servo's key registers")
    s.add_argument("id", type=int)

    s = sub.add_parser("setid", help="change a servo's ID (one servo on the bus)")
    s.add_argument("old", type=int)
    s.add_argument("new", type=int)

    s = sub.add_parser("fixrange", help="restore 0..4095 travel + position mode")
    s.add_argument("id", type=int)

    s = sub.add_parser("move", help="move to an absolute tick (0-4095)")
    s.add_argument("id", type=int)
    s.add_argument("position", type=int)
    s.add_argument("--speed", type=int, default=600)
    s.add_argument("--acc", type=int, default=50)
    s.add_argument("--settle", type=float, default=0.8)

    s = sub.add_parser("torque", help="enable/release torque")
    s.add_argument("state", choices=["on", "off"])
    s.add_argument("id", type=int, nargs="?", default=None)

    s = sub.add_parser("middle", help="define current position as centre (assembly)")
    s.add_argument("id", type=int)
    s.add_argument("--yes", action="store_true")

    args = p.parse_args()

    if args.cmd == "bridge":
        return cmd_bridge(args)

    bus = Bus(find_port(args.port))
    try:
        return {
            "scan": cmd_scan, "info": cmd_info, "setid": cmd_setid,
            "fixrange": cmd_fixrange, "move": cmd_move,
            "torque": cmd_torque, "middle": cmd_middle,
        }[args.cmd](bus, args)
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
