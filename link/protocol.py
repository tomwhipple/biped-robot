"""Wire protocol for the wireless command link (laptop -> robot).

The radio carries INTENT, not the control loop. The 50 Hz policy runs on the
ESP32 next to the 1 Mbaud servo bus (docs/wiring.md), and this link only
supplies the (vx, yaw-rate) command that walker_env.set_command() already
takes. That is what makes untethered operation cheap: a late or lost packet
costs staleness, never a bad joint angle, and the watchdog below decays a
dead link to the zero command -- which is a *trained* behavior (walker_env
draws a stand command 30 % of the time, cmd_stand_prob) rather than a
bolted-on emergency pose.

The frame is identical over UDP (wireless) and UART0 (tethered debug), so
one encoder feeds both paths. Little-endian throughout: the ESP32 and every
host we target are LE, so the firmware can memcpy these structs.

This module is pure -- no sockets, no clock. The robot-side half of it
(decode + Watchdog) is the reference for what the ESP32 firmware must do;
sim/udp_agent.py runs that exact code against MuJoCo so the protocol is
exercised before any hardware exists.
"""
import struct
from enum import Enum
from typing import NamedTuple

# -- framing ---------------------------------------------------------------
MAGIC_CMD = b"BM"          # laptop -> robot ("BiMo")
MAGIC_TLM = b"BT"          # robot -> laptop
VERSION = 1
CMD_LEN = 14
TLM_LEN = 20

CMD_PORT = 4210            # robot listens here
TLM_PORT = 4211            # laptop listens here

# -- command flags ---------------------------------------------------------
FLAG_ENABLE = 1 << 0       # 0 = stand still regardless of vx/wz
FLAG_ESTOP = 1 << 1        # latching torque release; see Watchdog.accept

# -- timing ----------------------------------------------------------------
# 20 Hz is ~2 orders of magnitude more command bandwidth than intent actually
# changes at, and it makes STALE_MS a 5-packet outage rather than a 1-packet
# hiccup -- WiFi routinely drops one.
SEND_HZ = 20.0
STALE_MS = 250.0           # no valid packet for this long -> decay to stand
RELAX_MS = 5000.0          # ... and for THIS long -> torque off entirely

# A restarted sender begins at seq 0, which would otherwise look like an
# ancient reordered packet forever. A backwards jump bigger than this is read
# as "sender restarted", not "stale duplicate".
SEQ_RESYNC_GAP = 1000

# -- trained command envelope ----------------------------------------------
# These MUST track walker_env's cmd_v_range / cmd_w_range. The policy has
# only ever seen a stand (0,0) or a walk in [V_MIN_WALK, V_MAX]: the band
# (0, V_MIN_WALK) is a hole in the training distribution, so a lightly-pushed
# stick asking for 0.15 m/s is an out-of-distribution query, not a slow walk.
# clamp_to_envelope() closes that hole by snapping; see also sources.py.
V_MIN_WALK = 0.3           # walker_env cmd_v_range[0]
V_MAX = 1.0                # walker_env cmd_v_range[1]
W_MAX = 1.0                # walker_env cmd_w_range


class ProtocolError(ValueError):
    """Malformed, foreign, or corrupt frame. Callers drop the packet."""


class LinkState(Enum):
    # The wire encoding is this declaration order (see _TLM_STATES), so new
    # members go on the END and existing ones never move. Values stay <= 5
    # characters: commander.py's status line formats them "{:5s}".
    LIVE = "live"          # fresh command, tracking it
    STAND = "stand"        # link stale -> zero command, policy holds a stand
    RELAX = "relax"        # link long dead -> torque off, robot settles
    ESTOP = "estop"        # operator-latched torque release
    # Pack under-voltage, robot-latched (firmware battguard::Guard). Nothing
    # the operator sends clears these -- only a pack swap and a reboot.
    VLAND = "vland"        # confirmed flat pack: crouching down under control
    VSAFE = "vsafe"        # crouch finished, torque off, and it stays off


class Command(NamedTuple):
    seq: int
    vx: float              # body-frame forward velocity, m/s
    wz: float              # yaw rate, rad/s
    flags: int

    @property
    def enabled(self) -> bool:
        return bool(self.flags & FLAG_ENABLE)

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)


class Telemetry(NamedTuple):
    seq_echo: int          # last command seq the robot applied
    state: LinkState
    vbat_v: float
    up_z: float            # torso up-vector z; 1.0 = perfectly upright
    vx_est: float          # m/s, body-frame forward
    wz_est: float          # rad/s
    servo_err: int         # bitmask, bit i = servo ID i+1 faulted
    loop_late_pct: int     # % of control ticks that overran 20 ms


# -- CRC -------------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).

    UDP already carries a checksum, so this is not about bit rot on the air.
    It earns its 2 bytes by (a) rejecting stray traffic from anything else
    that happens to hit our port -- in AP mode the robot's network is open by
    default -- and (b) letting the identical frame ride UART0 for tethered
    debug, where there is no transport checksum at all.
    """
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def _milli(x: float) -> int:
    """Scale to milli-units and saturate into int16.

    Fixed-point rather than float32 so the frame stays byte-identical across
    the Python sender and a C firmware that may lack an FPU-friendly layout.
    +-32.767 m/s and +-32.767 rad/s are far outside anything this plant does.
    """
    return max(-32768, min(32767, int(round(x * 1000.0))))


def clamp_to_envelope(vx: float, wz: float) -> tuple:
    """Snap a raw command into the region the policy was actually trained on.

    Enforced robot-side (not just at the source) because the robot is the one
    that eats a bad command, and we do not control every future sender.
    """
    wz = max(-W_MAX, min(W_MAX, wz))
    if abs(vx) < V_MIN_WALK:
        # The untrained band. Snap to a stand rather than interpolate into it;
        # a turn-in-place (vx=0, wz!=0) IS trained, so wz survives.
        return 0.0, wz
    return max(-V_MAX, min(V_MAX, vx)), wz


# -- command frame ---------------------------------------------------------
def encode_command(seq: int, vx: float, wz: float, flags: int) -> bytes:
    body = struct.pack(
        "<2sBBIhh", MAGIC_CMD, VERSION, flags & 0xFF, seq & 0xFFFFFFFF,
        _milli(vx), _milli(wz),
    )
    return body + struct.pack("<H", crc16_ccitt(body))


def decode_command(buf: bytes) -> Command:
    if len(buf) != CMD_LEN:
        raise ProtocolError(f"command frame is {len(buf)} B, want {CMD_LEN}")
    magic, ver, flags, seq, vx_mm, wz_mr = struct.unpack("<2sBBIhh", buf[:12])
    if magic != MAGIC_CMD:
        raise ProtocolError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise ProtocolError(f"protocol version {ver}, want {VERSION}")
    (crc,) = struct.unpack("<H", buf[12:])
    if crc != crc16_ccitt(buf[:12]):
        raise ProtocolError("CRC mismatch")
    return Command(seq=seq, vx=vx_mm / 1000.0, wz=wz_mr / 1000.0, flags=flags)


# -- telemetry frame -------------------------------------------------------
_TLM_STATES = list(LinkState)


def encode_telemetry(t: Telemetry) -> bytes:
    body = struct.pack(
        "<2sBBIHhhhBB", MAGIC_TLM, VERSION, _TLM_STATES.index(t.state),
        t.seq_echo & 0xFFFFFFFF, max(0, min(65535, int(t.vbat_v * 1000))),
        _milli(t.up_z), _milli(t.vx_est), _milli(t.wz_est),
        t.servo_err & 0xFF, max(0, min(255, int(t.loop_late_pct))),
    )
    return body + struct.pack("<H", crc16_ccitt(body))


def decode_telemetry(buf: bytes) -> Telemetry:
    if len(buf) != TLM_LEN:
        raise ProtocolError(f"telemetry frame is {len(buf)} B, want {TLM_LEN}")
    (magic, ver, state, seq, vbat_mv, up_z, vx, wz, err,
     late) = struct.unpack("<2sBBIHhhhBB", buf[:18])
    if magic != MAGIC_TLM:
        raise ProtocolError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise ProtocolError(f"protocol version {ver}, want {VERSION}")
    (crc,) = struct.unpack("<H", buf[18:])
    if crc != crc16_ccitt(buf[:18]):
        raise ProtocolError("CRC mismatch")
    if state >= len(_TLM_STATES):
        raise ProtocolError(f"unknown link state {state}")
    return Telemetry(seq_echo=seq, state=_TLM_STATES[state],
                     vbat_v=vbat_mv / 1000.0, up_z=up_z / 1000.0,
                     vx_est=vx / 1000.0, wz_est=wz / 1000.0,
                     servo_err=err, loop_late_pct=late)


# -- robot-side supervisor -------------------------------------------------
class Watchdog:
    """What the robot does with (and without) commands. Firmware reference.

    The clock is injected as now_ms on every call rather than read from
    time.monotonic(), so the sim twin and the unit tests can drive link loss
    deterministically instead of sleeping through it.
    """

    def __init__(self, stale_ms: float = STALE_MS,
                 relax_ms: float = RELAX_MS):
        if not 0 < stale_ms < relax_ms:
            raise ValueError("need 0 < stale_ms < relax_ms")
        self.stale_ms = stale_ms
        self.relax_ms = relax_ms
        self._last_rx_ms = None
        self._last_seq = None
        self._cmd = (0.0, 0.0)
        self._estop = False
        self.rejected = 0      # stale/duplicate frames, for telemetry

    def accept(self, pkt: Command, now_ms: float) -> bool:
        """Apply a decoded command. False = dropped as stale/reordered.

        UDP may reorder, so an older seq is not evidence of a live link and
        must not pet the watchdog -- otherwise a reordered burst could hold
        the robot LIVE on a command that is already history.
        """
        if self._last_seq is not None and pkt.seq <= self._last_seq \
                and self._last_seq - pkt.seq < SEQ_RESYNC_GAP:
            self.rejected += 1
            return False

        self._last_seq = pkt.seq
        self._last_rx_ms = now_ms

        if pkt.estop:
            self._estop = True
        elif self._estop and not pkt.enabled:
            # Leaving E-stop requires passing through a disabled command, so a
            # released dead-man cannot re-arm straight back into motion.
            self._estop = False

        self._cmd = clamp_to_envelope(pkt.vx, pkt.wz) if pkt.enabled \
            else (0.0, 0.0)
        return True

    def state(self, now_ms: float) -> LinkState:
        if self._estop:
            return LinkState.ESTOP
        if self._last_rx_ms is None:
            # Never heard from anyone: stand, do not relax into a heap. The
            # robot may be powered up and already on its feet.
            return LinkState.STAND
        age = now_ms - self._last_rx_ms
        if age >= self.relax_ms:
            return LinkState.RELAX
        if age >= self.stale_ms:
            return LinkState.STAND
        return LinkState.LIVE

    def command(self, now_ms: float) -> tuple:
        """The (vx, wz) to hand the policy right now."""
        return self._cmd if self.state(now_ms) is LinkState.LIVE else (0.0,
                                                                       0.0)

    @property
    def last_seq(self) -> int:
        return self._last_seq or 0
