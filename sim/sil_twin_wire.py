"""What sim/sil_twin.py sends besides the policy's actions: its beacon and its
flight records, built by pure functions.

No sockets in the builders, no MuJoCo, no SIL library: sim state and SIL
targets in, datagram bytes out, so the tests drive them with synthetic state.
The bytes come from link/protocol.py (encode_telemetry) and tools/flightrec.py
(encode_tick / encode_event / encode_hello); the twin has no encoder of its
own.

Steps. Every step value in the twin's TICKs (nominal, goal, pos, speed) is
in ONE calibration per bus joint, and the twin's HELLO sends exactly that
calibration, so flight_reader's steps -> rad (zero and dir from the HELLO,
obs::busStepsToAngle) gives back the sim's angle. For the joints the policy
drives it is the SIL library's own calibration (harness.Calibration, the one
SilTargets.goal_ticks is written in); every other bus joint the plant has
gets the nominal one (2048, +1).

What the twin fills, and from where, is listed in docs/flight-recorder.md
("The SIL twin as a source").
"""
from __future__ import annotations

import math
import os
import random
import socket
import sys
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (os.path.join(ROOT, "tools"), os.path.join(ROOT, "link")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import flightrec as F                                        # noqa: E402
from protocol import LinkState, Telemetry, encode_telemetry  # noqa: E402

CENTER_STEPS = 2048
MAX_STEPS = 4095
RAD_PER_STEP = 2.0 * math.pi / F.STEPS_PER_REV
# LinkState's wire encoding is its declaration order (protocol._TLM_STATES,
# linkproto::LinkState) -- also the flight record's link_state byte.
LINK_STATES = list(LinkState)
BUS_INDEX = {n: b for b, n in enumerate(F.BUS_JOINT_NAMES)}
HELLO_PERIOD_US = 1_000_000


def link_state_code(state: LinkState) -> int:
    return LINK_STATES.index(state)


# -- units ------------------------------------------------------------------
def rad_to_steps(rad: float, zero: int = CENTER_STEPS, direction: int = 1) -> int:
    """Joint angle -> servo steps: the inverse of flightrec.steps_to_rad.

    zero + rint(rad / (2 pi / 4096) * dir), clipped to the encoder's 0..4095
    (round half to even, as lrintf and harness.angle_to_steps). No joint-range
    clamp: a measured angle is where the joint IS (harness.make_sensors).
    """
    return min(max(zero + round(rad / RAD_PER_STEP * direction), 0), MAX_STEPS)


def rad_s_to_steps_s(rad_s: float, direction: int = 1) -> int:
    """Joint rate -> the servo's signed steps/s (harness.rad_s_to_steps_s)."""
    return min(max(round(rad_s / RAD_PER_STEP * direction), -32768), 32767)


def roll_pitch_from_quat(q) -> tuple[float, float]:
    """flightrec::rollPitchFromQuat: ZYX roll and pitch (rad) from a
    body->world (w, x, y, z) quaternion."""
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(min(1.0, max(-1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


# -- calibration ------------------------------------------------------------
@dataclass(frozen=True)
class BusCal:
    """Per BUS joint (flightrec.BUS_JOINT_NAMES order): what the HELLO says."""
    id: tuple
    fitted: tuple
    zero: tuple
    dir: tuple


def bus_calibration(plant_joints, servo_ids, policy_cal) -> BusCal:
    """plant_joints: the joint names the plant has (its "fitted" servos);
    servo_ids: the servo ID per bus joint (bus_map.h kBusServoId);
    policy_cal: {name: (zero_steps, dir)} for the joints the SIL library
    drives, in its own calibration."""
    if len(servo_ids) != F.MAX_JOINTS:
        raise ValueError(f"{len(servo_ids)} servo ids, want {F.MAX_JOINTS}")
    unknown = sorted(set(policy_cal) - set(BUS_INDEX))
    if unknown:
        raise ValueError(f"policy joints with no bus slot: {unknown}")
    plant = set(plant_joints)
    zero, d = [CENTER_STEPS] * F.MAX_JOINTS, [1] * F.MAX_JOINTS
    for name, (z, s) in policy_cal.items():
        if s not in (1, -1):
            raise ValueError(f"{name}: dir {s}, want +1 or -1")
        zero[BUS_INDEX[name]], d[BUS_INDEX[name]] = int(z), int(s)
    return BusCal(id=tuple(int(i) for i in servo_ids),
                  fitted=tuple(int(n in plant) for n in F.BUS_JOINT_NAMES),
                  zero=tuple(zero), dir=tuple(d))


# -- the SIL controller's view of one tick -----------------------------------
def sil_view(H, lib, out) -> dict:
    """One sil_tick's output, in the twin's terms.

    H is sim/sil/harness, PASSED IN: this module never imports it, because the
    harness sizes its ABI from the policy profile and must be imported only
    after eval_precision.sil_profile_for has chosen one. lib needs .spec and
    .out_order (a harness.SilLib); out is a harness.SilTargets.

    targets: {joint name: (nominal_rad, goal_steps, goal_speed)} --
      nominal = the raw action through the action map (obs::actionToAngles,
      what the robot's recorder logs as nominal), goal = goal_ticks (the SYNC
      WRITE, shaped), goal_speed = its reg-46 speed (0 with shaping off).
    cmd, phase, up, gyro: from the frame the library assembled and fed its
    policy (SilTargets.obs, current frame first).
    """
    import numpy as np
    spec = lib.spec
    goal = H.from_bus(np.asarray(out.goal_ticks, dtype=np.int64), lib.out_order, spec)
    speed = H.from_bus(np.asarray(out.goal_speed, dtype=np.int64), lib.out_order, spec)
    nom = H.action_to_angles(np.asarray(out.action, dtype=np.float64))
    obs = np.asarray(out.obs, dtype=np.float64)
    f = H.split_obs(obs, spec)
    ph = spec.off["phase"]
    return dict(
        targets={n: (float(nom[i]), int(goal[i]), int(speed[i]))
                 for i, n in enumerate(spec.joint_names)},
        cmd=[float(v) for v in f["cmd"]],
        phase=float(math.atan2(obs[ph], obs[ph + 1])),
        up=[float(v) for v in f["up"]], gyro=[float(v) for v in f["gyro"]])


# -- the TICK ---------------------------------------------------------------
def joint_block(cal: BusCal, measured, targets=None, written=False) -> list:
    """Per fitted bus joint, ascending bus index (the robot's order).

    measured: {name: (pos_rad, vel_rad_s)} -- the sim's joint angle and rate.
    targets: {name: (nominal_rad | None, goal_steps | None, goal_speed)} for
      the joints the SIL controller drives, or None when it commands nothing.
    written: the plant tracked these goals this tick (torque on).
    Fields the sim does not have (load, current, voltage, temperature,
    status, moving) stay 0; the flags say which of the rest are valid.
    """
    rows = []
    for b, name in enumerate(F.BUS_JOINT_NAMES):
        if not cal.fitted[b]:
            continue
        j = dict(bus=b, flags=0)
        if name in measured:
            pos, vel = measured[name]
            j["pos"] = rad_to_steps(pos, cal.zero[b], cal.dir[b])
            j["speed"] = rad_s_to_steps_s(vel, cal.dir[b])
            j["flags"] |= F.JF_FB_OK
        if targets is not None and name in targets:
            nom, goal, gspd = targets[name]
            j["flags"] |= F.JF_DRIVEN
            if nom is not None:
                j["nominal"] = rad_to_steps(nom, cal.zero[b], cal.dir[b])
                j["flags"] |= F.JF_NOM_VALID
            if goal is not None and written:
                j["goal"] = int(goal)
                j["goal_speed"] = int(gspd)
                j["flags"] |= F.JF_GOAL_VALID | F.JF_WRITTEN
        rows.append(j)
    return rows


def tick_body(*, tick: int, armed: bool, torque: bool, link_state: int,
              work_us: int, joints: list, imu: dict | None = None,
              view: dict | None = None, vbat_mv: int = 0) -> dict:
    """The TICK body dict flightrec.encode_tick takes.

    armed -> source ctrl (the control loop owns the plant), else idle (the
    robot's benched source). imu: dict(accel, gyro, quat, up) or None.
    view: sil_view() when the SIL controller's output is reported -- its
    command (padded with zeros to 7, the firmware's rule) and gait phase.
    The sim reads its "servos" at the tick start, with no bus between, so
    fb_off_us / fb_read_us / imu_age_us are 0.
    """
    flags = 0
    if torque:
        flags |= F.TF_TORQUE
    if any(j["flags"] & F.JF_WRITTEN for j in joints):
        flags |= F.TF_WROTE
    if any(j["flags"] & F.JF_FB_OK for j in joints):
        flags |= F.TF_FB_READ
    t = dict(source=F.SRC_CTRL if armed else F.SRC_BENCH_IDLE,
             link_state=int(link_state), tick=int(tick) & 0xFFFFFFFF,
             work_us=max(0, int(work_us)), joints=joints)
    if imu is not None:
        flags |= F.TF_IMU_VALID
        t.update(accel=tuple(map(float, imu["accel"])),
                 gyro=tuple(map(float, imu["gyro"])),
                 quat=tuple(map(float, imu["quat"])),
                 up=tuple(map(float, imu["up"])))
        t["roll"], t["pitch"] = roll_pitch_from_quat(imu["quat"])
    ctrl = dict(vbat_mv=max(0, min(0xFFFF, int(vbat_mv))))
    if view is not None:
        cmd = list(view["cmd"])[:F.NUM_CMD]
        ctrl.update(cmd=cmd + [0.0] * (F.NUM_CMD - len(cmd)), phase=view["phase"])
    t["ctrl"] = ctrl
    t["flags"] = flags
    return t


# -- the stream ---------------------------------------------------------------
class TwinFlight:
    """The twin's flight-record stream: one boot id, one sequence for every
    packet type, the drop counters. Clocks are arguments, so the builders are
    deterministic; `post` hands a packet to `send` (a callable returning True
    on success) and counts the outcome the way the robot's sender does."""

    def __init__(self, fw: str, cal: BusCal, send=None, boot_id: int | None = None,
                 idle_hz: int = 50, cal_ok: int = 1):
        if boot_id is None:
            boot_id = random.SystemRandom().getrandbits(32) or 1
        self.boot_id = boot_id
        self.fw = fw
        self.cal = cal
        self.send = send
        self.idle_hz = idle_hz
        self.cal_ok = cal_ok
        self.seq = 0
        self.sent = 0
        self.drop_net = 0
        self._next_hello_us = None
        self._link = None               # last link_state recorded (robot: 0xFF)
        self._stop_us = None

    def header(self, t_boot_us: int, t_utc_us: int) -> dict:
        h = dict(boot_id=self.boot_id, seq=self.seq, t_utc_us=int(t_utc_us),
                 t_boot_us=int(t_boot_us), drop_ring=0, drop_net=self.drop_net)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return h

    def hello_due(self, t_boot_us: int) -> bool:
        if self._next_hello_us is None or t_boot_us >= self._next_hello_us:
            self._next_hello_us = t_boot_us + HELLO_PERIOD_US
            return True
        return False

    def hello(self, t_boot_us: int, t_utc_us: int, *, ticks: int = 0, late: int = 0,
              worst_us: int = 0) -> bytes:
        """The 1 Hz HELLO. Counters the twin has: produced, sent, drop_net,
        ctrl ticks, overruns, worst tick, uptime; the rest stay 0."""
        u32 = 0xFFFFFFFF
        m = dict(fw=self.fw, utc_synced=1, enabled=1, idle_hz=self.idle_hz,
                 cal_ok=self.cal_ok, produced=self.seq & u32, sent=self.sent & u32,
                 drop_net=self.drop_net & u32, ctrl_ticks=int(ticks) & u32,
                 ctrl_late=int(late) & u32, ctrl_worst_us=int(worst_us) & u32,
                 uptime_ms=(int(t_boot_us) // 1000) & u32,
                 id=list(self.cal.id), fitted=list(self.cal.fitted),
                 dir=list(self.cal.dir), zero=list(self.cal.zero))
        return F.encode_hello(self.header(t_boot_us, t_utc_us), m)

    def tick(self, t_boot_us: int, t_utc_us: int, body: dict) -> bytes:
        return F.encode_tick(self.header(t_boot_us, t_utc_us), body)

    def event(self, t_boot_us: int, t_utc_us: int, code: int, *, src: int = 0,
              a: int = 0, b: int = 0, text: str = "") -> bytes:
        return F.encode_event(self.header(t_boot_us, t_utc_us),
                              dict(code=code, src=src, a=int(a), b=int(b), text=text))

    def link_change(self, t_boot_us: int, t_utc_us: int, state_code: int):
        """EV_LINK_STATE (src ctrl, a = old or -1, b = new) when the armed
        loop's link state differs from the last one recorded, else None."""
        if state_code == self._link:
            return None
        old = -1 if self._link is None else self._link
        self._link = state_code
        return self.event(t_boot_us, t_utc_us, F.EV_LINK_STATE, src=1, a=old, b=state_code)

    def stop(self, t_boot_us: int, t_utc_us: int, hard: bool, armed: bool):
        """EV_STOP for an operator stop frame, once per 2 s burst (cli.cpp)."""
        if self._stop_us is not None and t_boot_us - self._stop_us <= 2_000_000:
            return None
        self._stop_us = t_boot_us
        what = (("operator HARD STOP (armed): torque OFF, run ended" if armed
                 else "operator HARD STOP (bench): motion stopped, torque OFF") if hard
                else ("operator soft STOP (armed): policy stopped, joints HELD, run ended"
                      if armed else "operator soft STOP (bench): motion stopped where it is"))
        return self.event(t_boot_us, t_utc_us, F.EV_STOP, a=int(hard), b=int(armed),
                          text=what)

    def post(self, pkt) -> bool:
        if pkt is None or self.send is None:
            return False
        ok = bool(self.send(pkt))
        if ok:
            self.sent += 1
        else:
            self.drop_net += 1
        return ok


class UdpSend:
    """`send` for TwinFlight: one non-blocking socket, the address resolved
    once; a failed send returns False (counted as drop_net), never waits."""

    def __init__(self, host: str, port: int):
        self.addr = F.resolve_ipv4(host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def __call__(self, pkt: bytes) -> bool:
        try:
            return self.sock.sendto(pkt, self.addr) == len(pkt)
        except OSError:
            return False

    def close(self):
        self.sock.close()


# -- the beacon ---------------------------------------------------------------
def beacon(*, seq_echo: int, state: LinkState, vbat_v: float, up, vx_est: float,
           wz_est: float, loop_late_pct: int, t_us: int, joints=(), att: bool = False) -> bytes:
    """The twin's 10 Hz telemetry frame, by firmware/main/wifi_link.cpp's rule.

    up: the torso up vector (x, y, z), world frame (MuJoCo framezaxis, the
    obs's `up`). up_z is always in the frame; up_x/up_y ride along exactly
    when the commander's latest frame asked (FLAG_ATT), with no freshness
    gate -- they are the same sample as up_z. `joints` (17, bus order) only
    for a commander that asked with FLAG_POSE; () otherwise.
    """
    return encode_telemetry(Telemetry(
        seq_echo=int(seq_echo), state=state, vbat_v=float(vbat_v),
        up_z=float(up[2]), vx_est=float(vx_est), wz_est=float(wz_est),
        servo_err=0, loop_late_pct=int(loop_late_pct), t_us=int(t_us),
        joints=tuple(joints),
        up_xy=(float(up[0]), float(up[1])) if att else ()))
