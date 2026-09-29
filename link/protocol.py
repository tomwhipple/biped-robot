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
link/link_twin.py runs that exact code on a laptop, so the protocol is
exercised end to end before any hardware is involved.
"""
import struct
from enum import Enum
from typing import NamedTuple

# -- framing ---------------------------------------------------------------
MAGIC_CMD = b"BM"          # laptop -> robot ("BiMo")
MAGIC_TLM = b"BT"          # robot -> laptop
# Protocol version 2 (issue #81): the telemetry frame carries the robot's
# 17-servo bus -- servo_err widens from one byte to a u32 (bit b = servo ID
# b + 1) and the joint block is 17 angles in servo-ID order. The BASE block
# changed, which a length cannot say, so the version byte is 2 in BOTH
# directions: a version-1 console and a version-2 robot refuse each other's
# frames outright instead of one of them driving blind. Consoles and firmware
# from one tree always agree.
VERSION = 2
CMD_LEN = 14
# Extended command frame (2026-08-31): same 12-byte prefix, then FIVE more
# int16 milli-channels -- vy, crouch, lift, foot_dx, foot_dz -- completing
# walker_env.set_command()'s 7-wide ext_cmd vector over the wire. LENGTH
# selects the layout (no version bump): a 14 B frame is the classic pair
# with the extras at their trained defaults (0, 1.0, 0, 0, 0), so every
# pre-extension sender keeps working against new firmware. Senders emit the
# short frame whenever the extras ARE at defaults; old firmware drops long
# frames by length, which the arming handshake surfaces immediately.
CMD_LEN_EXT = 24
# Telemetry frame lengths. The 21-byte body (magic, version, state, seq_echo,
# vbat, up_z, vx_est, wz_est, the u32 servo_err, loop_late_pct), then a u64
# `t_us` -- microseconds since the Unix epoch, UTC, from the robot's
# SNTP-disciplined clock, 0 until the first sync -- then the optional blocks,
# then the CRC. The stamp is on EVERY beacon; see "Time on the wire" in
# docs/control-channel.md for why this one is volunteered rather than
# requested.
_TLM_BODY = "<2sBBIHhhhIB"
_TLM_TIME_OFF = struct.calcsize(_TLM_BODY)          # 21: the u64 t_us
TLM_LEN = _TLM_TIME_OFF + 8 + 2                     # 31
# The JOINT block (mirror mode): one int16 milli-radian per bus joint, in
# servo-ID order (JOINT_NAMES), for driving the real robot while a sim
# follows its observed pose. The robot is the SENDER here and every client is
# a decoder that rejects a length it does not know, so the long frame is
# REQUESTED (FLAG_POSE), never volunteered: a client that asks is by
# construction one that can read the answer.
#
# The joint set is the BUS joint set -- every servo the robot carries
# (firmware/components/obs/include/obs/bus_map.h, docs/servo-map.md section
# 2.1) -- not the policy's: the 10-joint prototype reports 0.0 for the seven
# servos it does not have. A consumer maps angles to its plant BY NAME.
JOINT_NAMES = (
    "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
    "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
    "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll",
    "neck_yaw", "R_shoulder", "R_elbow", "L_shoulder", "L_elbow",
)
NUM_JOINTS = len(JOINT_NAMES)                       # 17; servo ID = index + 1
TLM_LEN_EXT = TLM_LEN + 2 * NUM_JOINTS              # 65

# ATTITUDE: up_x and up_y appended after the joint block. up_z is in the base
# frame, but alone it is only the tilt MAGNITUDE -- how far from upright,
# never which way. Requested with FLAG_ATT, for the same reason FLAG_POSE
# exists. Four lengths, unambiguous because every block is fixed size:
# 31 base, 35 +att, 65 +joints, 69 +joints+att.
# Firmware reference: linkproto::kTlmLenAtt / kTlmLenExtAtt.
TLM_ATT_BYTES = 4
TLM_LEN_ATT = TLM_LEN + TLM_ATT_BYTES
TLM_LEN_EXT_ATT = TLM_LEN_EXT + TLM_ATT_BYTES
# Every length a decoder accepts.
TLM_LENS = (TLM_LEN, TLM_LEN_ATT, TLM_LEN_EXT, TLM_LEN_EXT_ATT)

CMD_PORT = 4210            # robot listens here
TLM_PORT = 4211            # laptop listens here

# -- command flags ---------------------------------------------------------
FLAG_ENABLE = 1 << 0       # 0 = stand still regardless of vx/wz
FLAG_ESTOP = 1 << 1        # latching torque release; see Watchdog.accept
FLAG_ARM = 1 << 2          # operator wants the control loop armed; ArmLatch
FLAG_POSE = 1 << 3         # "beacon joint angles too" -- see TLM_LEN_EXT
# "Put every joint back at its calibrated zero -- the standing pose -- now."
# (2026-09-02) A RECOVERY action rather than a command, and the difference is
# the whole point: it is honoured while the loop is BENCHED, while the FALL
# latch is tripped and while the E-stop is latched -- exactly the states in
# which every other channel on this wire correctly refuses to move anything,
# and exactly the states an operator is in when the robot is a heap on the
# floor and has to be stood back up before it can be armed again.
#
# It does not arm and cannot be used to walk: the robot BENCHES first (the
# CLI half of the firmware owns the bus for the move) and the joints slew at
# the bench's gentle speed. Acted on by its RISING EDGE (HomeLatch), like
# FLAG_ARM: a level would re-issue the move at 20 Hz, and a client that
# reboots with the bit set must not move a robot nobody is watching.
FLAG_HOME = 1 << 4
# "Beacon the torso up vector too" -- a LEVEL, like FLAG_POSE. Requests
# telemetry, commands nothing.
FLAG_ATT = 1 << 5

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
# extended-channel envelope, from the walker_env training draws:
VY_MAX = 0.3               # lateral sway amplitude the sway scenarios use
CROUCH_MIN = 0.6           # crouch_range[0] -- also the battguard ramp floor
FOOT_D_MAX = 0.05          # traj_radius hi: air-circle/march foot offsets, m
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
    # Torso down (up_z below the sim's own fall threshold, debounced),
    # robot-latched: torque off so a downed robot does not grind its servos
    # against the floor. A fall DISARMS the robot (firmware ctrl_task): this
    # state is reported for the fall tick, then the robot benches. Re-arming
    # is deliberate -- a fresh ARM edge or a tethered `run` -- never
    # automatic; the old righted-2s auto-clear is gone (2026-08-31).
    FALLEN = "fall"        # torso down, torque off; the run is over
    # Control loop benched: the CLI owns the servo bus, the link commands
    # nothing, torque is off. This is the boot state. Leaves it via an ARM
    # edge (ArmLatch) or the tethered `run`.
    BENCH = "bench"


# -- bench diagnostics ------------------------------------------------------
# Why the loop is NOT armed, carried to the operator over the radio.
#
# Until 2026-08-30 the refusal existed only as a line of UART text: a robot
# that declined a wireless ARM sat in BENCH saying nothing, and finding out
# why meant plugging the tether. This byte is that answer on the wire. It
# rides in `seq_echo` and ONLY while the state is BENCH -- seq_echo is "the
# last command seq the robot APPLIED", and a benched loop applies nothing, so
# in exactly that state the field carries no information to destroy. No
# version bump, no length change: every commander written before this still
# decodes every frame. See docs/control-channel.md for the alternatives that
# were rejected and why.
#
# Bit layout is APPEND-ONLY, like LinkState: reserved bits are sent zero and
# a reader ignores what it does not know.
DIAG_RUN = 1 << 0          # mode is run (0 = benched)
DIAG_CAL_OK = 1 << 1       # as-built calibration was loaded from NVS
# bits 2-3 reserved, sent zero
DIAG_ARM_SHIFT = 4         # bits 4-7: ArmResult
DIAG_ARM_MASK = 0xF0


class ArmResult(Enum):
    """What became of the last mode request (an ARM edge or a typed `run`).

    APPEND ONLY: a future refusal reason takes the next value. A client that
    meets a value it does not know must say so rather than report an older
    reason -- an unknown value means the ROBOT is newer than the client.
    """
    NONE = 0             # nothing has asked for a mode change yet
    ACCEPTED = 1         # the last request was honoured (arm or disarm)
    REFUSED_NO_CAL = 2   # arm refused: no as-built calibration in NVS
    DISARMED_FALL = 3    # the FALL latch tripped: run over, robot disarmed
    DISARMED_HOME = 4    # a FLAG_HOME request benched the loop and homed it
    # ... and the three ways that request can fail. A reset that silently
    # does nothing sends an operator looking for a broken button, a dead
    # radio or a seized servo, when the robot knew the answer all along.
    HOME_NO_CAL = 5      # reset REFUSED: no as-built calibration in NVS
    HOME_LOW_BATT = 6    # reset REFUSED: pack guard has torque latched off
    HOME_BUS_FAILED = 7  # reset FAILED: the servo bus did not accept it
    # (2026-09-03) the goal write is a broadcast with no reply, and one reset
    # that reported DISARMED_HOME moved nothing. The write now reports
    # HOME_PENDING; the robot reads every joint back after the slew and only
    # then says DISARMED_HOME -- or HOME_NOT_REACHED.
    HOME_PENDING = 8     # reset WRITTEN: joints slewing, readback not yet done
    HOME_NOT_REACHED = 9  # reset FAILED: readback found joints off their zeros
    # Plan B (issue #81): the position-loop gains live in each servo's EEPROM,
    # and a factory reset or a swapped spare silently comes back at P = 32.
    # The robot reads registers 21/22 back before every arm and refuses on
    # any difference from its compiled expected table.
    REFUSED_GAINS = 10   # arm refused: a servo's P/D (reg 21/22) is not the expected value


def pack_diag(run: bool, cal_ok: bool, result: ArmResult) -> int:
    d = (DIAG_RUN if run else 0) | (DIAG_CAL_OK if cal_ok else 0)
    return d | ((result.value << DIAG_ARM_SHIFT) & DIAG_ARM_MASK)


HOME_RESULTS = (ArmResult.DISARMED_HOME, ArmResult.HOME_NO_CAL,
                ArmResult.HOME_LOW_BATT, ArmResult.HOME_BUS_FAILED,
                ArmResult.HOME_PENDING, ArmResult.HOME_NOT_REACHED)


def is_home_result(r) -> bool:
    """True for every verdict a FLAG_HOME request can produce.

    A console that asked for a reset uses this to tell "the robot answered
    me" from "the robot is talking about something else" -- an old firmware
    that never heard of the request still reports ACCEPTED for the disarm
    that rode in with it.
    """
    return r in HOME_RESULTS


def diag_arm_result(diag: int):
    """The ArmResult in a diag byte, or None if this client is too old."""
    try:
        return ArmResult((diag & DIAG_ARM_MASK) >> DIAG_ARM_SHIFT)
    except ValueError:
        return None


def diag_reason(diag: int) -> str:
    """One line an operator can act on."""
    r = diag_arm_result(diag)
    if r is ArmResult.REFUSED_NO_CAL:
        return "arm REFUSED -- no as-built calibration in NVS (run `cal`)"
    if r is ArmResult.DISARMED_FALL:
        return ("disarmed -- FALL latch tripped; re-arm deliberately (ARM "
                "edge or run)")
    if r is ArmResult.DISARMED_HOME:
        return ("disarmed -- servos reset to the standing pose (hips rolled "
                "out/back for play) and torque RELEASED; arm to walk")
    if r is ArmResult.HOME_NO_CAL:
        return ("servo reset REFUSED -- no as-built calibration in NVS, so "
                "\"zero\" is not a stand (run `cal`)")
    if r is ArmResult.HOME_LOW_BATT:
        return ("servo reset REFUSED -- pack under-voltage latch; swap the "
                "pack, then `batt reset`")
    if r is ArmResult.HOME_BUS_FAILED:
        return ("servo reset FAILED -- the servo bus did not accept it "
                "(check pack, wiring, `scan`)")
    if r is ArmResult.HOME_PENDING:
        return ("servo reset written -- joints slewing to the stand, "
                "readback pending")
    if r is ArmResult.HOME_NOT_REACHED:
        return ("servo reset FAILED -- readback found joints OFF their zeros "
                "(see the tether for which; `scan`)")
    if r is ArmResult.REFUSED_GAINS:
        return ("arm REFUSED -- a servo's position-loop gains (reg 21/22) "
                "are not the expected values, or did not read back (run "
                "`gains`)")
    if r is ArmResult.ACCEPTED:
        return ("armed -- the control loop is running" if diag & DIAG_RUN
                else "disarmed on request -- press arm to run")
    if r is ArmResult.NONE:
        return ("no arm requested yet -- calibrated, ready to arm"
                if diag & DIAG_CAL_OK else
                "no arm requested yet -- and there is NO calibration in NVS, "
                "so arming will be REFUSED")
    return "arm REFUSED -- reason unknown to this client (newer firmware)"


class Command(NamedTuple):
    seq: int
    vx: float              # body-frame forward velocity, m/s
    wz: float              # yaw rate, rad/s
    flags: int
    # extended channels (walker_env ext_cmd layout beyond vx/wz); a classic
    # 14 B frame decodes to exactly these defaults
    vy: float = 0.0        # lateral velocity, m/s
    crouch: float = 1.0    # stance-height fraction command (cmd[3])
    lift: float = 0.0      # swing-foot selector: -1 left, +1 right, 0 none
    foot_dx: float = 0.0   # swing-foot target x offset, m
    foot_dz: float = 0.0   # swing-foot target extra height, m

    @property
    def enabled(self) -> bool:
        return bool(self.flags & FLAG_ENABLE)

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)

    @property
    def arm(self) -> bool:
        return bool(self.flags & FLAG_ARM)

    @property
    def pose(self) -> bool:
        """This sender wants joint angles in the beacon (mirror mode)."""
        return bool(self.flags & FLAG_POSE)

    @property
    def home(self) -> bool:
        """This sender is asking for the servos to be reset to zero."""
        return bool(self.flags & FLAG_HOME)

    @property
    def att(self) -> bool:
        """"Beacon the up vector too" -- a level, like `pose`."""
        return bool(self.flags & FLAG_ATT)


class Telemetry(NamedTuple):
    seq_echo: int          # last command seq the robot applied
    state: LinkState
    vbat_v: float
    up_z: float            # torso up-vector z; 1.0 = perfectly upright
    vx_est: float          # m/s, body-frame forward
    wz_est: float          # rad/s
    servo_err: int         # u32 bitmask, bit b = BUS joint b faulted, i.e.
                           # servo ID b + 1 (JOINT_NAMES order); only fitted
                           # servos ever set a bit
    loop_late_pct: int     # % of control ticks that overran 20 ms
    # Measured joint angles, radians, bus order (JOINT_NAMES: servo ID =
    # index + 1). Empty on a classic frame -- which is every frame, unless a
    # commander asked with FLAG_POSE.
    joints: tuple = ()
    # Microseconds since the Unix epoch (UTC) on the robot's clock, which is
    # SNTP-disciplined over the same WiFi link. 0 = the robot has not synced
    # yet (or the frame predates the field). When joints are present this is
    # the instant they were read off the bus; otherwise the instant the
    # beacon was assembled -- either way within one 20 ms tick of the values.
    t_us: int = 0

    @property
    def t_utc(self):
        """Robot time as float seconds since the epoch, or None if unsynced."""
        return None if self.t_us == 0 else self.t_us / 1e6
    # Torso up vector x and y (z is up_z above): the robot's own z axis in the
    # WORLD frame, obs_spec kOffUp / imu::Sample::up. None unless a commander
    # asked with FLAG_ATT. up_z is drift-immune; these two are yaw-dependent
    # and this robot's heading is dead-reckoned, so they give a lean direction
    # in a slowly rotating frame -- an attitude to draw, not a heading.
    up_xy: tuple = ()

    @property
    def diag(self) -> int:
        """The bench diagnostic byte, or 0 when this frame carries none.

        Reading seq_echo directly would be a bug waiting to happen: the byte
        is only meaningful while the state is BENCH (see pack_diag above).
        """
        return (self.seq_echo & 0xFF) if self.state is LinkState.BENCH else 0

    @property
    def reason(self) -> str:
        return diag_reason(self.diag)


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


def _milli_u16(x: float) -> int:
    """Scale to milli-units and saturate into uint16 (the vbat field).

    ROUND, not truncate.  The robot holds the pack voltage as integer mV and
    hands encodeTelemetry a float32 of it, and float32(11400/1000) is
    11.399999619 V -- truncating that costs a millivolt on the wire, and
    made the firmware and this reference disagree byte-for-byte on 11.4 V
    (issue #54).  Rounding makes mV -> V -> mV the identity for every value
    the field can hold, so both encoders land on the same integer.
    """
    return max(0, min(65535, int(round(x * 1000.0))))


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


def clamp_ext_to_envelope(pkt: "Command") -> tuple:
    """The extended channels, snapped into their trained draws.

    Returns (vy, crouch, lift, foot_dx, foot_dz)."""
    return (max(-VY_MAX, min(VY_MAX, pkt.vy)),
            max(CROUCH_MIN, min(1.0, pkt.crouch)),
            max(-1.0, min(1.0, pkt.lift)),
            max(-FOOT_D_MAX, min(FOOT_D_MAX, pkt.foot_dx)),
            max(-FOOT_D_MAX, min(FOOT_D_MAX, pkt.foot_dz)))


# -- command frame ---------------------------------------------------------
_EXT_DEFAULTS = (0.0, 1.0, 0.0, 0.0, 0.0)   # vy, crouch, lift, fdx, fdz


def encode_command(seq: int, vx: float, wz: float, flags: int,
                   vy: float = 0.0, crouch: float = 1.0, lift: float = 0.0,
                   foot_dx: float = 0.0, foot_dz: float = 0.0) -> bytes:
    body = struct.pack(
        "<2sBBIhh", MAGIC_CMD, VERSION, flags & 0xFF, seq & 0xFFFFFFFF,
        _milli(vx), _milli(wz),
    )
    ext = (vy, crouch, lift, foot_dx, foot_dz)
    if any(_milli(a) != _milli(b) for a, b in zip(ext, _EXT_DEFAULTS)):
        body += struct.pack("<5h", *(_milli(v) for v in ext))
    return body + struct.pack("<H", crc16_ccitt(body))


def decode_command(buf: bytes) -> Command:
    if len(buf) not in (CMD_LEN, CMD_LEN_EXT):
        raise ProtocolError(f"command frame is {len(buf)} B, want {CMD_LEN} "
                            f"or {CMD_LEN_EXT}")
    magic, ver, flags, seq, vx_mm, wz_mr = struct.unpack("<2sBBIhh", buf[:12])
    if magic != MAGIC_CMD:
        raise ProtocolError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise ProtocolError(f"protocol version {ver}, want {VERSION}")
    (crc,) = struct.unpack("<H", buf[-2:])
    if crc != crc16_ccitt(buf[:-2]):
        raise ProtocolError("CRC mismatch")
    ext = _EXT_DEFAULTS
    if len(buf) == CMD_LEN_EXT:
        ext = tuple(v / 1000.0 for v in struct.unpack("<5h", buf[12:22]))
    return Command(seq=seq, vx=vx_mm / 1000.0, wz=wz_mr / 1000.0, flags=flags,
                   vy=ext[0], crouch=ext[1], lift=ext[2],
                   foot_dx=ext[3], foot_dz=ext[4])


# -- telemetry frame -------------------------------------------------------
_TLM_STATES = list(LinkState)


def encode_telemetry(t: Telemetry) -> bytes:
    body = struct.pack(
        _TLM_BODY, MAGIC_TLM, VERSION, _TLM_STATES.index(t.state),
        t.seq_echo & 0xFFFFFFFF, _milli_u16(t.vbat_v),
        _milli(t.up_z), _milli(t.vx_est), _milli(t.wz_est),
        t.servo_err & 0xFFFFFFFF, max(0, min(255, int(t.loop_late_pct))),
    )
    if not 0 <= t.t_us < 2 ** 64:
        raise ProtocolError(f"t_us {t.t_us} does not fit a u64")
    body += struct.pack("<Q", t.t_us)
    if t.joints:
        if len(t.joints) != NUM_JOINTS:
            raise ProtocolError(f"{len(t.joints)} joints, want {NUM_JOINTS}")
        body += struct.pack(f"<{NUM_JOINTS}h", *(_milli(q) for q in t.joints))
    if t.up_xy:
        if len(t.up_xy) != 2:
            raise ProtocolError(f"up_xy is {len(t.up_xy)} long, want 2")
        body += struct.pack("<2h", *(_milli(v) for v in t.up_xy))
    return body + struct.pack("<H", crc16_ccitt(body))


def decode_telemetry(buf: bytes) -> Telemetry:
    if len(buf) not in TLM_LENS:
        raise ProtocolError(f"telemetry frame is {len(buf)} B, want one of "
                            f"{TLM_LENS}")
    (magic, ver, state, seq, vbat_mv, up_z, vx, wz, err,
     late) = struct.unpack(_TLM_BODY, buf[:_TLM_TIME_OFF])
    if magic != MAGIC_TLM:
        raise ProtocolError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise ProtocolError(f"protocol version {ver}, want {VERSION}")
    (crc,) = struct.unpack("<H", buf[-2:])
    if crc != crc16_ccitt(buf[:-2]):
        raise ProtocolError("CRC mismatch")
    if state >= len(_TLM_STATES):
        raise ProtocolError(f"unknown link state {state}")
    # Length selects the layout: joints on the two long lengths, the
    # attitude pair on the two +att ones.
    at = _TLM_TIME_OFF
    (t_us,) = struct.unpack("<Q", buf[at:at + 8])
    at += 8
    joints = ()
    if len(buf) in (TLM_LEN_EXT, TLM_LEN_EXT_ATT):
        joints = tuple(v / 1000.0 for v in
                       struct.unpack(f"<{NUM_JOINTS}h",
                                     buf[at:at + 2 * NUM_JOINTS]))
        at += 2 * NUM_JOINTS
    up_xy = ()
    if len(buf) in (TLM_LEN_ATT, TLM_LEN_EXT_ATT):
        up_xy = tuple(v / 1000.0 for v in struct.unpack("<2h", buf[at:at + 4]))
    return Telemetry(seq_echo=seq, state=_TLM_STATES[state],
                     vbat_v=vbat_mv / 1000.0, up_z=up_z / 1000.0,
                     vx_est=vx / 1000.0, wz_est=wz / 1000.0,
                     servo_err=err, loop_late_pct=late, joints=joints,
                     t_us=t_us, up_xy=up_xy)


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
        self._ext = _EXT_DEFAULTS
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
        # ENABLE gates the extended channels too: disabled = a plain stand
        # (extras at trained defaults), exactly like vx/wz.
        self._ext = clamp_ext_to_envelope(pkt) if pkt.enabled \
            else _EXT_DEFAULTS
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

    def command_ext(self, now_ms: float) -> tuple:
        """The full 7-wide ext_cmd vector (vx, vy, wz, crouch, lift,
        foot_dx, foot_dz). Stale links decay to the trained stand command,
        same as command()."""
        if self.state(now_ms) is LinkState.LIVE:
            (vx, wz), (vy, crouch, lift, fdx, fdz) = self._cmd, self._ext
        else:
            (vx, wz), (vy, crouch, lift, fdx, fdz) = (0.0, 0.0), _EXT_DEFAULTS
        return (vx, vy, wz, crouch, lift, fdx, fdz)

    @property
    def last_seq(self) -> int:
        return self._last_seq or 0


class ArmLatch:
    """Turns the ARM bit into bench/run mode requests. Firmware reference.

    ARM is a *level* on the wire ("the operator wants the loop armed"), like
    ENABLE, but the robot acts on its EDGES only:

      0 -> 1   arm: hand the servo bus to the control loop (the tethered `run`)
      1 -> 0   disarm: bench it -- torque off, bus back to the CLI

    Edges rather than the level, for two reasons. A sender that has never
    heard of the bit (the script/gamepad commanders, a pre-ARM firmware's
    twin) sends 0 forever and produces no edge, so it can drive a robot that
    was armed over the tether without benching it on its first frame. And the
    very first frame from a sender never counts: a robot that reboots under a
    client still holding ARM=1 stays benched until that client deliberately
    re-arms -- the client sees BENCH in telemetry and says so.

    Deliberately NOT inside Watchdog: the watchdog is re-created on every
    bench -> run handover (fresh link state for a fresh run), while this must
    outlive it to see the 1 -> 0 edge that ends the run. Sequence order is
    the watchdog's business; this sees every decoded frame in arrival order,
    duplicates included (a duplicate has the same level and makes no edge).
    """

    def __init__(self):
        self._level = None

    def update(self, flags: int):
        """Feed one decoded frame's flags. True = arm, False = disarm,
        None = no change requested."""
        level = bool(flags & FLAG_ARM)
        prev, self._level = self._level, level
        if prev is None or prev == level:
            return None
        return level

    @property
    def level(self):
        """The last ARM level seen, or None before the first frame."""
        return self._level


class HomeLatch:
    """The FLAG_HOME rising edge. Firmware reference (linkproto::HomeLatch).

    Same first-frame rule as ArmLatch -- a client that boots with the bit
    already set makes no edge and therefore moves nothing -- but ONE-SIDED:
    only 0 -> 1 is an event. Dropping the bit means "request over", not
    "un-home", so there is nothing to report on the falling edge.
    """

    def __init__(self):
        self._level = None

    def update(self, flags: int) -> bool:
        """Feed one decoded frame's flags. True exactly on the 0 -> 1 edge."""
        level = bool(flags & FLAG_HOME)
        prev, self._level = self._level, level
        return prev is False and level

    @property
    def level(self):
        return self._level


class Supervisor:
    """ArmLatch + Watchdog + the bench/run mode, as the firmware composes
    them (firmware/main/ctrl_task.cpp). Reference for the twins.

    Benched (the boot state) the link commands nothing, torque is off and
    telemetry says BENCH. An ARM edge arms: fresh Watchdog, fresh run. The
    1 -> 0 edge benches again. `arm_allowed` stands in for the firmware's
    refusal to run without an as-built calibration in NVS -- and, like the
    firmware (cli.cpp cmdMode), the refusal is RECORDED so the beacon can
    say why rather than leaving the operator with a silent BENCH.
    """

    def __init__(self, armed: bool = False, arm_allowed: bool = True):
        self.armed = armed
        self.arm_allowed = arm_allowed
        self.result = ArmResult.NONE
        self.latch = ArmLatch()
        self.home_latch = HomeLatch()
        self.dog = Watchdog()
        self.homes = 0        # completed servo resets, for the twins' log
        self.holding = False  # benched with torque HOLDING the homed pose

    def accept(self, pkt: Command, now_ms: float) -> bool:
        """Feed one decoded frame. Returns the watchdog's verdict (always
        False while benched: nothing was applied)."""
        want = self.latch.update(pkt.flags)
        if want is not None and want != self.armed:
            if want and not self.arm_allowed:
                self.result = ArmResult.REFUSED_NO_CAL   # refused; stays BENCH
            else:
                self.result = ArmResult.ACCEPTED
                self.armed = want
                if want:
                    self.dog = Watchdog()         # fresh run, fresh link state
                    self.holding = False          # the loop owns the bus now
        # A home request outranks everything: it is honoured benched, fallen
        # and E-stopped, and it ends any run first. The mode goes to bench
        # BEFORE the joints move, because on the robot the move is a CLI bus
        # transaction and the CLI only owns the bus while benched.
        if self.home_latch.update(pkt.flags):
            self.homes += 1
            self.armed = False
            self.holding = True
            self.result = ArmResult.DISARMED_HOME
            self.dog = Watchdog()
            return False
        if not self.armed:
            return False
        return self.dog.accept(pkt, now_ms)

    def diag(self) -> int:
        """The bench diagnostic byte the robot would beacon right now.

        `arm_allowed` IS the firmware's g_cal_from_nvs: same gate, same bit.
        """
        return pack_diag(self.armed, self.arm_allowed, self.result)

    def state(self, now_ms: float) -> LinkState:
        return self.dog.state(now_ms) if self.armed else LinkState.BENCH

    def command(self, now_ms: float) -> tuple:
        return self.dog.command(now_ms) if self.armed else (0.0, 0.0)

    def command_ext(self, now_ms: float) -> tuple:
        return (self.dog.command_ext(now_ms) if self.armed
                else (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))

    def torque_on(self, now_ms: float) -> bool:
        # A homed robot is benched with torque still ON, holding the standing
        # pose -- that is what makes it stand up rather than fold. Nothing
        # else about BENCH changes: the link commands nothing.
        if not self.armed:
            return self.holding
        return self.state(now_ms) not in (LinkState.RELAX, LinkState.ESTOP,
                                          LinkState.BENCH)

    def seq_echo(self, now_ms: float) -> int:
        """What belongs in a telemetry frame's seq_echo field right now.

        One place decides which field the diag byte rides in, so a twin
        cannot drift from the firmware (wifi_link.cpp does the same thing).
        """
        return (self.diag() if self.state(now_ms) is LinkState.BENCH
                else self.last_seq)

    @property
    def last_seq(self) -> int:
        return self.dog.last_seq
