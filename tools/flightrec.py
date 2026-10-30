"""Flight recorder wire format -- the Python mirror of
firmware/components/flightrec/include/flightrec/record.h.

One UDP datagram = one packet. Every packet starts with the 36-byte header;
the body is a TICK (one control tick), an EVENT (a CLI line, a printed reply,
an edge) or a HELLO (~1 Hz: firmware, calibration, counters). Little-endian,
float32. docs/flight-recorder.md describes every field.

The C++ encoder is checked byte-for-byte against vectors generated from THIS
module (tools/gen_flightrec_vectors.py -> firmware/host/vectors/
flightrec_vectors.h, firmware/host/test_flightrec.cpp), so a field moved on
one side fails a test on the other.

Stdlib only: the collector imports this on a box with no numpy.
"""
from __future__ import annotations

import math
import struct

MAGIC = b"FR"
VERSION = 1
DEFAULT_PORT = 4220

T_TICK, T_EVENT, T_HELLO = 1, 2, 3
SRC_CTRL, SRC_BENCH_POSE, SRC_BENCH_IDLE = 1, 2, 3
SOURCE_NAMES = {SRC_CTRL: "ctrl", SRC_BENCH_POSE: "pose", SRC_BENCH_IDLE: "idle"}

# TICK flags
TF_TORQUE, TF_TRAJ, TF_HOME, TF_BAL = 1, 2, 4, 8
TF_IMU_VALID, TF_FB_READ, TF_WROTE, TF_FALLEN = 16, 32, 64, 128
# joint flags
JF_FB_OK, JF_NOM_VALID, JF_GOAL_VALID, JF_WRITTEN, JF_DRIVEN = 1, 2, 4, 8, 16

EV_CMD, EV_OUT, EV_ARM_EDGE, EV_HOME_EDGE, EV_REC = 1, 2, 3, 4, 5
EV_MODE, EV_BAL_STATE, EV_LINK_STATE, EV_FB_BACKOFF = 6, 7, 8, 9
EVENT_NAMES = {EV_CMD: "cmd", EV_OUT: "out", EV_ARM_EDGE: "arm_edge",
               EV_HOME_EDGE: "home_edge", EV_REC: "rec", EV_MODE: "mode",
               EV_BAL_STATE: "bal_state", EV_LINK_STATE: "link_state",
               EV_FB_BACKOFF: "fb_backoff"}

# balance::State (firmware/components/balance/include/balance/roll_pi.h)
BAL_STATES = {0: "OFF", 1: "ACTIVE", 2: "ABORT_TIP", 3: "ABORT_IMU", 4: "ARMED"}

MAX_JOINTS = 17
MAX_TEXT = 180
NUM_CMD = 7
STEPS_PER_REV = 4096

# Bus order (obs/bus_map.h kBusJointNames == link/protocol.py JOINT_NAMES).
BUS_JOINT_NAMES = (
    "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
    "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
    "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll",
    "neck_yaw", "R_shoulder", "R_elbow", "L_shoulder", "L_elbow",
)

HEADER = struct.Struct("<2sBBIIQQII")
TICK_FIXED = struct.Struct("<BBBBIIiIi" "3f3f4f3f2f" "BBbb8fhhI" "7ffHBB")
JOINT = struct.Struct("<BBBBBBhhHhhhh")
EVENT_FIXED = struct.Struct("<BBHii")
HELLO_FIXED = struct.Struct("<32sBBBB19IBBBB")
HELLO_JOINT = struct.Struct("<BBbh")

HEADER_LEN = HEADER.size            # 36
TICK_FIXED_LEN = TICK_FIXED.size    # 164
JOINT_LEN = JOINT.size              # 20
assert (HEADER_LEN, TICK_FIXED_LEN, JOINT_LEN, EVENT_FIXED.size,
        HELLO_FIXED.size) == (36, 164, 20, 12, 116)

HEADER_FIELDS = ("type", "boot_id", "seq", "t_utc_us", "t_boot_us",
                 "drop_ring", "drop_net")
JOINT_FIELDS = ("bus", "flags", "status", "temp_c", "voltage_dv", "moving",
                "nominal", "goal", "goal_speed", "pos", "speed", "load",
                "current")
HELLO_COUNTERS = ("produced", "sent", "drop_ring", "drop_net",
                  "max_push_ctrl_us", "max_push_house_us", "bench_read_us",
                  "ctrl_ticks", "ctrl_late", "ctrl_worst_us",
                  "us_read", "us_imu", "us_obs", "us_net", "us_write",
                  "us_other", "uptime_ms", "ntp_syncs", "reserved")


class DecodeError(ValueError):
    pass


# -- encode (tests, vectors, synthetic sessions) ------------------------------
def _header(h: dict, ptype: int) -> bytes:
    return HEADER.pack(MAGIC, VERSION, ptype, h.get("boot_id", 0),
                       h.get("seq", 0), h.get("t_utc_us", 0),
                       h.get("t_boot_us", 0), h.get("drop_ring", 0),
                       h.get("drop_net", 0))


def encode_tick(h: dict, t: dict) -> bytes:
    bal = t.get("bal", {})
    ctl = t.get("ctrl", {})
    joints = t.get("joints", [])
    if len(joints) > MAX_JOINTS:
        raise ValueError("too many joints")
    cmd = list(ctl.get("cmd", [0.0] * NUM_CMD))
    fixed = TICK_FIXED.pack(
        t.get("source", SRC_CTRL), t.get("flags", 0), t.get("link_state", 0),
        len(joints), t.get("tick", 0), t.get("work_us", 0),
        t.get("fb_off_us", 0), t.get("fb_read_us", 0), t.get("imu_age_us", 0),
        *t.get("accel", (0.0, 0.0, 0.0)), *t.get("gyro", (0.0, 0.0, 0.0)),
        *t.get("quat", (1.0, 0.0, 0.0, 0.0)), *t.get("up", (0.0, 0.0, 1.0)),
        t.get("roll", 0.0), t.get("pitch", 0.0),
        bal.get("state", 0), bal.get("side", 0), bal.get("ankle_bus", -1),
        bal.get("hip_bus", -1), bal.get("ref", 0.0), bal.get("err", 0.0),
        bal.get("integral", 0.0), bal.get("u", 0.0), bal.get("rate", 0.0),
        bal.get("roll", 0.0), bal.get("ankle_rad", 0.0),
        bal.get("hip_rad", 0.0), bal.get("ankle_ticks", 0),
        bal.get("hip_ticks", 0), bal.get("clamps", 0),
        *cmd, ctl.get("phase", 0.0), ctl.get("vbat_mv", 0),
        ctl.get("batt", 0), ctl.get("home_stage", 0))
    body = b"".join(JOINT.pack(*(j.get(f, 0) for f in JOINT_FIELDS))
                    for j in joints)
    return _header(h, T_TICK) + fixed + body


def encode_event(h: dict, e: dict) -> bytes:
    text = e.get("text", "")
    raw = text.encode("utf-8", "replace") if isinstance(text, str) else text
    raw = raw.rstrip(b"\r\n")[:MAX_TEXT]
    return (_header(h, T_EVENT)
            + EVENT_FIXED.pack(e.get("code", EV_CMD), e.get("src", 0),
                               len(raw), e.get("a", 0), e.get("b", 0))
            + raw)


def encode_hello(h: dict, m: dict) -> bytes:
    fw = m.get("fw", "").encode()[:31]
    out = _header(h, T_HELLO) + HELLO_FIXED.pack(
        fw, m.get("utc_synced", 0), m.get("enabled", 0), m.get("idle_hz", 0),
        m.get("cal_ok", 0), *(m.get(k, 0) for k in HELLO_COUNTERS),
        MAX_JOINTS, 0, 0, 0)
    for b in range(MAX_JOINTS):
        out += HELLO_JOINT.pack(m.get("id", [0] * MAX_JOINTS)[b],
                                m.get("fitted", [0] * MAX_JOINTS)[b],
                                m.get("dir", [1] * MAX_JOINTS)[b],
                                m.get("zero", [2048] * MAX_JOINTS)[b])
    return out


# -- decode ------------------------------------------------------------------
def decode_header(buf: bytes) -> dict:
    if len(buf) < HEADER_LEN:
        raise DecodeError(f"short packet ({len(buf)} B)")
    magic, ver, ptype, *rest = HEADER.unpack_from(buf, 0)
    if magic != MAGIC:
        raise DecodeError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise DecodeError(f"version {ver}, this decoder speaks {VERSION}")
    return dict(zip(HEADER_FIELDS, (ptype, *rest)))


def decode(buf: bytes) -> dict:
    """One packet -> a dict: the header fields plus the body's."""
    h = decode_header(buf)
    off = HEADER_LEN
    if h["type"] == T_TICK:
        if len(buf) < off + TICK_FIXED_LEN:
            raise DecodeError("short tick")
        v = TICK_FIXED.unpack_from(buf, off)
        off += TICK_FIXED_LEN
        nj = v[3]
        if nj > MAX_JOINTS or len(buf) != off + nj * JOINT_LEN:
            raise DecodeError(f"tick length {len(buf)} does not fit {nj} joints")
        h.update(source=v[0], flags=v[1], link_state=v[2], njoint=nj,
                 tick=v[4], work_us=v[5], fb_off_us=v[6], fb_read_us=v[7],
                 imu_age_us=v[8], accel=v[9:12], gyro=v[12:15],
                 quat=v[15:19], up=v[19:22], roll=v[22], pitch=v[23])
        b = v[24:39]
        h["bal"] = dict(state=b[0], side=b[1], ankle_bus=b[2], hip_bus=b[3],
                        ref=b[4], err=b[5], integral=b[6], u=b[7], rate=b[8],
                        roll=b[9], ankle_rad=b[10], hip_rad=b[11],
                        ankle_ticks=b[12], hip_ticks=b[13], clamps=b[14])
        c = v[39:]
        h["ctrl"] = dict(cmd=c[0:7], phase=c[7], vbat_mv=c[8], batt=c[9],
                         home_stage=c[10])
        h["joints"] = [dict(zip(JOINT_FIELDS, JOINT.unpack_from(buf, off + k * JOINT_LEN)))
                       for k in range(nj)]
    elif h["type"] == T_EVENT:
        if len(buf) < off + EVENT_FIXED.size:
            raise DecodeError("short event")
        code, src, n, a, b = EVENT_FIXED.unpack_from(buf, off)
        off += EVENT_FIXED.size
        if n > MAX_TEXT or len(buf) != off + n:
            raise DecodeError("event text length mismatch")
        h.update(code=code, src=src, a=a, b=b,
                 text=buf[off:off + n].decode("utf-8", "replace"))
    elif h["type"] == T_HELLO:
        if len(buf) != off + HELLO_FIXED.size + MAX_JOINTS * HELLO_JOINT.size:
            raise DecodeError("hello length mismatch")
        v = HELLO_FIXED.unpack_from(buf, off)
        off += HELLO_FIXED.size
        h.update(fw=v[0].split(b"\0", 1)[0].decode("ascii", "replace"),
                 utc_synced=v[1], enabled=v[2], idle_hz=v[3], cal_ok=v[4])
        h.update(zip(HELLO_COUNTERS, v[5:24]))
        if v[24] != MAX_JOINTS:
            raise DecodeError("hello joint count")
        rows = [HELLO_JOINT.unpack_from(buf, off + k * HELLO_JOINT.size)
                for k in range(MAX_JOINTS)]
        h.update(id=[r[0] for r in rows], fitted=[r[1] for r in rows],
                 dir=[r[2] for r in rows], zero=[r[3] for r in rows])
    else:
        raise DecodeError(f"unknown packet type {h['type']}")
    return h


def steps_to_rad(steps: float, zero: int, direction: int) -> float:
    """obs::busStepsToAngle: (steps - zero) * 2 pi / 4096 * dir."""
    return (steps - zero) * (2.0 * math.pi / STEPS_PER_REV) * direction
