"""Bench tool for ST3215 bus servos, bypassing the vendor web UI.

WHY THIS EXISTS
---------------
Setting IDs through the vendor web UI by hand did not work: the "ID to Set"
field is an up/down counter with no set-to-value control, and in a browser it
was observed jumping ~10 per click, making exact targets like 9 unreachable.

Measured on the real board 2026-07-26, the firmware itself steps by **1**
(`ID to Set` went 30 -> 31 -> 30 over two `/cmd` calls), and the served page's
handler is byte-identical to the published source. So the jump is client-side
-- a click delivered repeatedly -- and the HTTP API is perfectly usable as
long as something issues an *exact* number of calls. That is what
`http-setid` does, and it needs no USB cable and no flashing.

CAUTION: **the firmware on the board is not the published source.** Anything
below cited from GitHub is a guide, not ground truth; prefer what the board
actually reports. (Known-good from live measurement: `/cmd` executes and then
closes the socket *without* an HTTP response; `/readID` serves the previous
scan result until an async rescan actually begins.)

The serial path exists for what HTTP cannot reach -- raw registers. The
vendor firmware has a **`SERIAL_FORWARDING`** mode (`BOARD_DEV.h:157`)
bridging USB serial to the 1 Mbaud servo bus byte-for-byte, toggled by
`activeCtrl(14/15)` over HTTP:

    laptop --USB 115200--> ESP32 (dumb bridge) --1 Mbaud--> servo bus

That gives arbitrary register reads/writes, which is the only way to *verify*
a servo's mode and angle limits rather than trust a button. It matters because
the UI's "Set Servo Mode" calls `setMode(id, 0)`, and in the published source
that writes SC-series angle limits (20..1003) into registers 9/11 -- which the
ST series uses for the same purpose, clamping an ST3215 to ~24 % of travel.
Whether this build does that is **unverified**; `info` will say, and
`fixrange` repairs it either way.

Nothing here flashes anything. The vendor firmware stays as it is.

Protocol facts, verified against the vendor's own library rather than assumed:
ST series is **little-endian** (`SMS_STS.cpp`: `End = 0`; SC is `End = 1`),
checksum is `~(id + len + inst + params) & 0xFF`, and "set middle" is a write
of **128** to torque-enable register 40. The packet encoder is checked against
the worked examples in Feetech's protocol manual.

USAGE
-----
    # WiFi only (ESP32_DEV / 12345678) -- no USB needed:
    .venv/bin/python tools/servo_tool.py state
    .venv/bin/python tools/servo_tool.py http-setid 9 --yes

    # with USB attached, for raw register access:
    .venv/bin/python tools/servo_tool.py bridge on
    .venv/bin/python tools/servo_tool.py scan
    .venv/bin/python tools/servo_tool.py info 9
    .venv/bin/python tools/servo_tool.py fixrange 9      # mode 0 + 0..4095
    .venv/bin/python tools/servo_tool.py move 9 2048
    .venv/bin/python tools/servo_tool.py bridge off

See docs/bringup-day1.md.
"""
from __future__ import annotations

import argparse
import glob
import sys
import time
import http.client
import urllib.error
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
    cmd_t=1 -> activeCtrl(cmd_i) (the action table below)
    cmd_t=9 -> rescan the bus

    activeCtrl actions used here: 9/10 = servotoSet +/-1, 16 = commit that
    value as the active servo's new ID, 14/15 = USB<->bus bridge on/off.

    NOTE: the handler executes the action and returns WITHOUT calling
    server.send(), so the ESP32 closes the socket with no HTTP response at
    all. curl shrugs; urllib raises RemoteDisconnected. The command has
    already run by then, so an empty reply is success, not failure.
    """
    url = f"{BOARD_HTTP}/cmd?inputT={cmd_t}&inputI={cmd_i}&inputA=0&inputB=0"
    try:
        urllib.request.urlopen(url, timeout=timeout).read()
    except http.client.RemoteDisconnected:
        pass
    except urllib.error.URLError as e:
        if not isinstance(e.reason, http.client.RemoteDisconnected):
            raise


def board_get(path: str, timeout: float = 6.0) -> str:
    return urllib.request.urlopen(f"{BOARD_HTTP}/{path}",
                                  timeout=timeout).read().decode("utf-8", "replace")


def board_status() -> dict:
    """Parse /readSTS into the handful of fields we actually act on."""
    raw = board_get("readSTS")
    out = {"raw": raw}
    for key, field in (("Active ID:", "active_id"), ("ID to Set:", "to_set"),
                       ("Position:", "position")):
        i = raw.find(key)
        if i >= 0:
            digits = ""
            for ch in raw[i + len(key):]:
                if ch.isdigit() or (ch == "-" and not digits):
                    digits += ch
                else:
                    break
            if digits:
                out[field] = int(digits)
    out["motor_mode"] = "Motor Mode" in raw
    out["torque_on"] = "Torque On" in raw
    return out


def board_ids() -> list[int]:
    """Discovered servo IDs. handleID prints them space-separated (CONNECT.h:125)."""
    raw = board_get("readID")
    if "Searching" in raw:
        return []
    return [int(t) for t in raw.replace("ID:", " ").split() if t.isdigit()]


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


def cmd_httpsetid(args) -> int:
    """Set a servo ID over WiFi only -- no USB, no bridge, no flashing.

    The web UI's ID field is an up/down counter (`servotoSet`, +/-1 per call,
    wrapping at 250) with no set-to-value command, so a browser that delivers
    a click more than once makes exact targets unreachable by hand. Issuing an
    exact number of calls does not have that problem.
    """
    target = args.target
    if not 0 <= target <= 253:
        sys.exit("ID must be 0-253")

    ids = board_ids()
    if not ids:
        print("no servos discovered -- press Start Searching, or check power.")
        return 1
    if len(ids) > 1:
        print(f"refusing: {len(ids)} servos on the bus {ids}. Connect exactly "
              f"one when changing IDs -- the board writes to whichever is "
              f"'active', and duplicates cannot be told apart afterwards.")
        return 1
    old = ids[0]
    if old == target:
        print(f"already ID {target}; nothing to do.")
        return 0

    st = board_status()
    cur = st.get("to_set")
    if cur is None:
        print(f"could not parse 'ID to Set' from /readSTS:\n  {st['raw']}")
        return 1
    print(f"one servo on the bus at ID {old}; staging value is {cur}, want {target}")

    # walk servotoSet to the target with an exact call count, verifying as we
    # go -- if the board's step size is ever not 1, this stops instead of
    # silently landing somewhere else
    action = 9 if target > cur else 10
    steps = abs(target - cur)
    for _ in range(steps):
        board_cmd(1, action)
    time.sleep(0.3)
    now = board_status().get("to_set")
    if now != target:
        print(f"staging value is {now}, expected {target} after {steps} calls "
              f"-- aborting before the write. (Step size is not 1?)")
        return 1
    print(f"staged: ID to Set = {now}")

    if not args.yes:
        print(f"\nWould write ID {old} -> {target} to EEPROM. Re-run with --yes.")
        return 0

    board_cmd(1, 16)                     # setID(active, servotoSet)
    time.sleep(0.4)
    board_cmd(9, 0)                      # rescan; listID is stale after a change

    # The rescan is ASYNCHRONOUS. Until it actually starts, handleID keeps
    # serving the PREVIOUS list -- so polling immediately reads the old ID and
    # reports a false failure. Wait for the "Searching..." phase to appear and
    # then clear before believing anything.
    saw_searching = False
    ids = []
    for _ in range(50):
        time.sleep(0.4)
        raw = board_get("readID")
        if "Searching" in raw:
            saw_searching = True
            continue
        if saw_searching:
            ids = board_ids()
            break
    else:
        time.sleep(2.0)                  # never caught the transition; settle
        ids = board_ids()
    if ids == [target]:
        print(f"ID {old} -> {target}  (EEPROM, survives power-off). Bus now {ids}.")
        return 0
    print(f"after rescan the bus reads {ids}, expected [{target}].")
    return 1


def cmd_state(args) -> int:
    st = board_status()
    print(f"bus              {board_ids()}")
    print(f"active servo     {st.get('active_id')}")
    print(f"position         {st.get('position')}")
    print(f"mode             {'MOTOR (step/continuous)' if st['motor_mode'] else 'servo (position)'}")
    print(f"torque           {'on' if st['torque_on'] else 'off'}")
    print(f"staged ID to Set {st.get('to_set')}")
    if st["motor_mode"]:
        print("\n  ** Motor Mode is why position commands look dead. It needs to be\n"
              "     servo/position mode before any DOA test means anything.")
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

    s = sub.add_parser("state", help="what the board reports right now (HTTP, no USB)")

    s = sub.add_parser("http-setid",
                       help="set the ID over WiFi only -- no USB, no bridge")
    s.add_argument("target", type=int)
    s.add_argument("--yes", action="store_true", help="actually write EEPROM")

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

    # HTTP-only commands: these need the board's WiFi, not the USB cable
    if args.cmd == "bridge":
        return cmd_bridge(args)
    if args.cmd == "state":
        return cmd_state(args)
    if args.cmd == "http-setid":
        return cmd_httpsetid(args)

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
