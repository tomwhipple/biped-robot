"""Software-in-the-loop harness: walker_env <-> libctrl_sil.

See docs/sil-harness.md.  One 20 ms tick:

    walker_env obs frame
      -> SERVO-SIDE VIEW (calibrated uint16 ticks, signed steps/s, IMU floats)
      -> struct SilSensors -> sil_tick() -- the REAL firmware code --
      -> struct SilTargets (goal ticks, bus order)
      -> stepsToAngle -> anglesToAction -> env.step(action)

Nothing here knows anything about the policy: the network, the obs assembler,
the gait clock, the history ring and the calibration maths all live inside the
shared library, which is the firmware's own source compiled for the host.

The module is import-safe without the library (Agent A builds it in parallel):
`find_lib()` returns None and every lib-dependent entry point raises
`LibMissing`, which the pytest suite turns into a skip.

ABI conventions that docs/sil-harness.md left ambiguous are AUTO-PROBED at
`SilLib.open()` rather than guessed -- see `SilLib.probe()`.  The probe uses
`SilTargets.obs`, which the ABI already exposes "for diagnostics/golden", so
it costs one throwaway tick and removes the whole class of "the two agents
disagreed about array order" failures.
"""
from __future__ import annotations

import ctypes
import json
import math
import os
import re
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
FIRMWARE = os.path.join(ROOT, "firmware")
SPEC_H = os.path.join(FIRMWARE, "components", "obs", "include", "obs",
                      "obs_spec.h")

WEIGHTS_DIR = os.path.join(HERE, "weights")
GOLDEN_DIR = os.path.join(HERE, "golden")
CAL_DIR = os.path.join(HERE, "cal")
# first policy trained on the corrected printed foot (f58a412); v6creep
# predates it and no longer stands on the current plant (2026-08-01)
DEFAULT_RUN = "loco_v8foot"

# Servo units.  4096 ticks / revolution (scsbus::kStepsPerRev), encoder full
# scale 0..4095, "middle" 2048.
STEPS_PER_REV = 4096
RAD_PER_STEP = 2.0 * math.pi / STEPS_PER_REV
MAX_STEPS = 4095
CENTER_STEPS = 2048

# The firmware's default gait-clock frequency (obs::GaitClock, ctrl_task.cpp).
DEFAULT_GAIT_HZ = 1.5


class LibMissing(RuntimeError):
    """libctrl_sil is not built yet."""


class ProbeFailed(RuntimeError):
    """The library is there but its ABI does not behave as documented."""


# =====================================================================
#  obs_spec.h  (single source of truth for widths / offsets / bus ids)
# =====================================================================
@dataclass
class ObsSpec:
    num_joints: int = 10
    act_dim: int = 10
    num_cmd: int = 7
    hist_len: int = 3
    frame_dim: int = 49
    obs_dim: int = 147
    control_dt: float = 0.02
    servo_id: tuple = (9, 1, 2, 3, 4, 10, 5, 6, 7, 8)
    joint_names: tuple = ()
    off: dict = field(default_factory=dict)

    def __post_init__(self):
        if not self.off:
            nj = self.num_joints
            self.off = dict(q=0, dq=nj, up=2 * nj, linvel=2 * nj + 3,
                            gyro=2 * nj + 6, prev_action=2 * nj + 9,
                            height=3 * nj + 9, phase=3 * nj + 10,
                            cmd=3 * nj + 12)

    def slice(self, name, width):
        o = self.off[name]
        return slice(o, o + width)


def _parse_obs_spec(path=SPEC_H) -> ObsSpec:
    try:
        txt = open(path).read()
    except OSError:
        return ObsSpec()
    ints, floats = {}, {}
    for line in txt.splitlines():
        m = re.match(r"inline constexpr int (k\w+)\s*=\s*(-?\d+);", line.strip())
        if m:
            ints[m.group(1)] = int(m.group(2))
        m = re.match(r"inline constexpr float (k\w+)\s*=\s*([-\d.eEf+]+);",
                     line.strip())
        if m:
            floats[m.group(1)] = float(m.group(2).rstrip("f"))
    ids = ()
    m = re.search(r"kServoId\[kNumJoints\]\s*=\s*\{([^}]*)\}", txt)
    if m:
        ids = tuple(int(x) for x in m.group(1).split(",") if x.strip())
    names = ()
    m = re.search(r"kJointNames\[kNumJoints\]\s*=\s*\{([^}]*)\}", txt)
    if m:
        names = tuple(x.strip().strip('"') for x in m.group(1).split(",")
                      if x.strip())
    spec = ObsSpec(
        num_joints=ints.get("kNumJoints", 10),
        act_dim=ints.get("kActDim", 10),
        num_cmd=ints.get("kNumCmd", 7),
        hist_len=ints.get("kHistLen", 3),
        frame_dim=ints.get("kFrameDim", 49),
        obs_dim=ints.get("kObsDim", 147),
        control_dt=floats.get("kControlDt", 0.02),
        servo_id=ids or (9, 1, 2, 3, 4, 10, 5, 6, 7, 8),
        joint_names=names,
        off=dict(q=ints["kOffQ"], dq=ints["kOffDq"], up=ints["kOffUp"],
                 linvel=ints["kOffLinVel"], gyro=ints["kOffGyro"],
                 prev_action=ints["kOffPrevAction"],
                 height=ints["kOffHeight"], phase=ints["kOffPhase"],
                 cmd=ints["kOffCmd"]) if "kOffQ" in ints else {},
    )
    return spec


SPEC = _parse_obs_spec()

# Joint limits, read from the same generated header, so the harness's inverse
# action map is the firmware's forward map run backwards -- not a second copy
# of walker_env's numbers.
def _parse_limits(path=SPEC_H):
    try:
        txt = open(path).read()
    except OSError:
        return None, None, None
    def arr(name):
        m = re.search(name + r"\[kNumJoints\]\s*=\s*\{([^}]*)\}", txt)
        if not m:
            return None
        return np.array([float(x.strip().rstrip("f"))
                         for x in m.group(1).split(",") if x.strip()])
    return arr("kJointLo"), arr("kJointHi"), arr("kJointDefault")


JOINT_LO, JOINT_HI, JOINT_DEFAULT = _parse_limits()


if JOINT_LO is None or JOINT_HI is None or JOINT_DEFAULT is None:
    raise ImportError(f"could not read joint limits from {SPEC_H}")


# =====================================================================
#  the exported weight blob (tools/export_policy_weights.py)
# =====================================================================
def read_silw(path=None, run=DEFAULT_RUN):
    """Parse a .silw blob back into {mean, std, layers, layer_sizes, ...}.

    Reference reader for the format the firmware must implement; also what
    the pytest suite uses to check the export against the brax goldens
    without needing jax."""
    import struct
    path = path or default_weights(run)
    with open(path, "rb") as f:
        blob = f.read()
    if blob[:4] != b"SILW":
        raise ValueError(f"{path}: not a SILW blob (use the sidecar's "
                         "layer_sizes for a --raw export)")
    (version, header_bytes, obs_dim, act_dim, n_layers,
     n_sizes) = struct.unpack_from("<6I", blob, 4)
    act = blob[28:28 + 16].split(b"\0")[0].decode()
    sizes = list(struct.unpack_from("<%dI" % n_sizes, blob, 28 + 16))
    if header_bytes != 28 + 16 + 4 * n_sizes:
        raise ValueError(f"{path}: header_bytes {header_bytes} inconsistent")
    f32 = np.frombuffer(blob, dtype="<f4", offset=header_bytes)
    at = 0

    def take(n):
        nonlocal at
        v = f32[at:at + n]
        at += n
        return np.asarray(v, dtype=np.float32)

    mean, std = take(obs_dim), take(obs_dim)
    layers = []
    for i in range(n_layers):
        w = take(sizes[i] * sizes[i + 1]).reshape(sizes[i], sizes[i + 1])
        layers.append((w, take(sizes[i + 1])))
    if at != f32.size:
        raise ValueError(f"{path}: {f32.size - at} trailing floats")
    return dict(version=version, activation=act, obs_dim=obs_dim,
                act_dim=act_dim, layer_sizes=sizes, mean=mean, std=std,
                layers=layers, header_bytes=header_bytes, path=path)


def silw_forward(w, obs):
    """float32 forward pass: normalize -> swish MLP -> tanh(logits[:act])."""
    x = ((np.asarray(obs, dtype=np.float32) - w["mean"])
         / w["std"]).astype(np.float32)
    n = len(w["layers"])
    for i, (kern, bias) in enumerate(w["layers"]):
        x = (x @ kern + bias).astype(np.float32)
        if i + 1 < n:
            x = (x / (1.0 + np.exp(-x, dtype=np.float32))).astype(np.float32)
    return np.tanh(x[:w["act_dim"]]).astype(np.float32)


def load_golden(name, golden_dir=GOLDEN_DIR):
    with open(os.path.join(golden_dir, name)) as f:
        return json.load(f)


# =====================================================================
#  calibration
# =====================================================================
@dataclass
class Calibration:
    """Per-joint {bus_id, zero_steps, dir}, indexed by JOINT index."""
    zero_steps: np.ndarray
    dir: np.ndarray
    bus_id: np.ndarray

    @staticmethod
    def nominal(spec: ObsSpec = SPEC) -> "Calibration":
        """What a freshly 'Set Middle Position'-ed, correctly oriented robot
        gives you -- identical to obs::Calibration's C++ default."""
        n = spec.num_joints
        return Calibration(zero_steps=np.full(n, CENTER_STEPS, dtype=np.int64),
                           dir=np.ones(n, dtype=np.int64),
                           bus_id=np.asarray(spec.servo_id, dtype=np.int64))

    @staticmethod
    def perturbed(seed=0, spec: ObsSpec = SPEC, flip_dirs=False
                  ) -> "Calibration":
        """Random zeros (docs/sil-harness.md), optionally flipped directions.
        Zeros stay far enough from the rails that the full joint range is
        still reachable inside 0..4095."""
        rng = np.random.default_rng(seed)
        n = spec.num_joints
        zero = rng.integers(CENTER_STEPS - 350, CENTER_STEPS + 350, size=n)
        d = np.ones(n, dtype=np.int64)
        if flip_dirs:
            d[rng.random(n) < 0.4] = -1
        return Calibration(zero_steps=zero.astype(np.int64), dir=d,
                           bus_id=np.asarray(spec.servo_id, dtype=np.int64))

    def to_dict(self, spec: ObsSpec = SPEC):
        joints = []
        for i in range(len(self.zero_steps)):
            joints.append(dict(
                index=i,
                name=(spec.joint_names[i] if i < len(spec.joint_names)
                      else f"joint_{i}"),
                bus_id=int(self.bus_id[i]),
                zero_steps=int(self.zero_steps[i]),
                dir=int(self.dir[i])))
        # flat mirrors so a reader without a full JSON object model still
        # gets the arrays in joint order
        return dict(num_joints=len(joints), joints=joints,
                    bus_id=[int(v) for v in self.bus_id],
                    zero_steps=[int(v) for v in self.zero_steps],
                    dir=[int(v) for v in self.dir])

    def write(self, path, spec: ObsSpec = SPEC):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(spec), f, indent=1)
        return path

    @staticmethod
    def read(path) -> "Calibration":
        with open(path) as f:
            d = json.load(f)
        return Calibration(zero_steps=np.asarray(d["zero_steps"], np.int64),
                           dir=np.asarray(d["dir"], np.int64),
                           bus_id=np.asarray(d["bus_id"], np.int64))


def write_cal_files(out_dir=CAL_DIR, spec: ObsSpec = SPEC):
    """Generate the nominal + perturbed calibration files sil_init() takes."""
    os.makedirs(out_dir, exist_ok=True)
    a = Calibration.nominal(spec).write(os.path.join(out_dir,
                                                     "cal_nominal.json"), spec)
    b = Calibration.perturbed(1, spec).write(
        os.path.join(out_dir, "cal_perturbed.json"), spec)
    return a, b


# =====================================================================
#  boundary conversion  (the sim-to-real translation layer under test)
# =====================================================================
def angle_to_steps(rad, cal: Calibration, clamp_joint_range=True):
    """obs::angleToSteps, vectorised.  Round-half-to-even matches lrintf()."""
    rad = np.asarray(rad, dtype=np.float64)
    if clamp_joint_range and JOINT_LO is not None:
        rad = np.clip(rad, JOINT_LO, JOINT_HI)
    ticks = np.rint(rad / RAD_PER_STEP * cal.dir).astype(np.int64)
    return np.clip(cal.zero_steps + ticks, 0, MAX_STEPS)


def steps_to_angle(steps, cal: Calibration):
    """obs::stepsToAngle, vectorised."""
    d = np.asarray(steps, dtype=np.int64) - cal.zero_steps
    return (d * RAD_PER_STEP * cal.dir).astype(np.float64)


def rad_s_to_steps_s(rad_s, cal: Calibration):
    """Inverse of obs::stepsPerSecToRadPerSec, clipped to the int16 field."""
    v = np.rint(np.asarray(rad_s, dtype=np.float64) / RAD_PER_STEP * cal.dir)
    return np.clip(v, -32768, 32767).astype(np.int64)


def steps_s_to_rad_s(steps_s, cal: Calibration):
    return (np.asarray(steps_s, dtype=np.float64) * RAD_PER_STEP
            * cal.dir).astype(np.float64)


def action_to_angles(action):
    """walker_env._action_to_ctrl for action_map='full' == obs::actionToAngles."""
    a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
    span = np.where(a >= 0.0, JOINT_HI - JOINT_DEFAULT,
                    JOINT_DEFAULT - JOINT_LO)
    return JOINT_DEFAULT + span * a


def angles_to_action(angle_rad):
    """obs::anglesToAction -- the inverse map the harness needs so that
    env.step(action) reproduces the target the firmware actually commanded."""
    d = np.asarray(angle_rad, dtype=np.float64) - JOINT_DEFAULT
    span = np.where(d >= 0.0, JOINT_HI - JOINT_DEFAULT,
                    JOINT_DEFAULT - JOINT_LO)
    with np.errstate(divide="ignore", invalid="ignore"):
        a = np.where(span > 1e-9, d / np.where(span > 1e-9, span, 1.0), 0.0)
    return np.clip(a, -1.0, 1.0)


# --- bus-id permutation ----------------------------------------------
def joint_to_slot_perm(spec: ObsSpec = SPEC):
    """P[i] = index of joint i inside an ASCENDING-BUS-ID array."""
    return np.asarray(spec.servo_id, dtype=np.int64) - 1


def to_bus(values, order, spec: ObsSpec = SPEC):
    """Joint-indexed -> the wire array the library expects."""
    v = np.asarray(values)
    if order == "joint":
        return v
    out = np.empty_like(v)
    out[joint_to_slot_perm(spec)] = v
    return out


def from_bus(values, order, spec: ObsSpec = SPEC):
    """The library's wire array -> joint-indexed."""
    v = np.asarray(values)
    if order == "joint":
        return v
    return v[joint_to_slot_perm(spec)]


# =====================================================================
#  the C ABI
# =====================================================================
NJ = SPEC.num_joints
NCMD = SPEC.num_cmd
NOBS = SPEC.obs_dim
NACT = SPEC.act_dim


class SilSensors(ctypes.Structure):
    """docs/sil-harness.md: what the bus + IMU actually deliver."""
    _fields_ = [
        ("pos_ticks", ctypes.c_uint16 * NJ),
        ("vel_ticks", ctypes.c_int16 * NJ),
        ("up", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("cmd", ctypes.c_float * NCMD),
    ]


class SilTargets(ctypes.Structure):
    _fields_ = [
        ("goal_ticks", ctypes.c_uint16 * NJ),
        ("action", ctypes.c_float * NACT),
        ("obs", ctypes.c_float * NOBS),
        # sil_abi 2: the per-servo goal speed (reg 46, steps/s) the SYNC WRITE
        # carries with the C2-shaped targets; 0 when shaping is off.
        ("goal_speed", ctypes.c_uint16 * NJ),
    ]


LIB_NAMES = ("libctrl_sil.dylib", "libctrl_sil.so", "ctrl_sil.dylib",
             "ctrl_sil.so")
LIB_DIRS = (os.path.join(FIRMWARE, "host", "build"),
            os.path.join(FIRMWARE, "host"))


def find_lib():
    """Path to libctrl_sil, or None.  $SIL_LIB overrides the search."""
    env = os.environ.get("SIL_LIB")
    if env:
        return env if os.path.exists(env) else None
    for d in LIB_DIRS:
        for n in LIB_NAMES:
            p = os.path.join(d, n)
            if os.path.exists(p):
                return p
    return None


def default_weights(run=DEFAULT_RUN):
    return os.path.join(WEIGHTS_DIR, f"{run}.silw")


class SilLib:
    """ctypes binding + the ABI probe."""

    def __init__(self, path, weights, cal_path, gait_hz=DEFAULT_GAIT_HZ,
                 cal: Calibration | None = None, spec: ObsSpec = SPEC,
                 probe=True):
        self.spec = spec
        self.path = path
        self.cal = cal if cal is not None else Calibration.read(cal_path)
        self.lib = ctypes.CDLL(path)
        self.lib.sil_init.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                      ctypes.c_float]
        self.lib.sil_init.restype = ctypes.c_int
        self.lib.sil_reset.argtypes = []
        self.lib.sil_reset.restype = None
        self.lib.sil_tick.argtypes = [ctypes.POINTER(SilSensors),
                                      ctypes.POINTER(SilTargets)]
        self.lib.sil_tick.restype = ctypes.c_int
        self.lib.sil_spec.argtypes = []
        self.lib.sil_spec.restype = ctypes.c_char_p
        # sil_abi 2 (optional on older libraries): the C2 command shaper.
        try:
            self.lib.sil_set_shaper.argtypes = [ctypes.c_float]
            self.lib.sil_set_shaper.restype = None
            self.has_shaper = True
        except AttributeError:
            self.has_shaper = False
        rc = self.lib.sil_init(weights.encode(), cal_path.encode(),
                               ctypes.c_float(gait_hz))
        if rc != 0:
            raise RuntimeError(f"sil_init({weights}, {cal_path}) -> {rc}")
        self.gait_hz = float(gait_hz)
        # ABI conventions, resolved by measurement (see probe()).
        self.in_order = "joint"
        self.out_order = "joint"
        self.vel_sign_magnitude = False
        if probe:
            self.probe()
        self.reset()

    # -- raw calls -----------------------------------------------------
    def spec_string(self):
        s = self.lib.sil_spec()
        return s.decode(errors="replace") if s else ""

    def set_shaper(self, pole_hz):
        """sil_abi 2: the C2 command shaper's pole in Hz; 0 = raw targets
        (the pre-shaper wire image). No-op on an abi-1 library."""
        if self.has_shaper:
            self.lib.sil_set_shaper(ctypes.c_float(pole_hz))

    def shaper_spec(self):
        """The library's CURRENT shaper config from sil_spec(), or None on an
        abi-1 library (which shipped raw targets unconditionally)."""
        try:
            return json.loads(self.spec_string()).get("shaper")
        except (ValueError, AttributeError):
            return None

    def reset(self):
        self.lib.sil_reset()

    def tick_raw(self, sensors: SilSensors) -> SilTargets:
        out = SilTargets()
        rc = self.lib.sil_tick(ctypes.byref(sensors), ctypes.byref(out))
        if rc != 0:
            raise RuntimeError(f"sil_tick -> {rc}")
        return out

    # -- the ABI probe -------------------------------------------------
    def probe(self):
        """Resolve the three conventions docs/sil-harness.md leaves open:

        1. are the sensor arrays indexed by JOINT (slot k carries bus id
           kServoId[k], which is how ctrl_task.cpp lays out g_fb) or sorted by
           ASCENDING BUS ID?
        2. same question for goal_ticks;
        3. is vel_ticks two's complement (the int16_t in the struct) or the
           servo's raw sign-magnitude-bit-15 encoding (the comment)?

        Answered by ticking once with a distinguishable pose and reading the
        assembled obs back out of SilTargets.obs.  One throwaway tick; the
        clock/history are cleaned up by the reset that follows.
        """
        n = self.spec.num_joints
        perm = joint_to_slot_perm(self.spec)
        forced = os.environ.get("SIL_ASSUME_ORDER")
        # Probe against RAW targets: this is about resolving array-order
        # conventions, and the raw action->angle->steps map is the invertible
        # reference. The shaper (sil_abi 2, on by default as deployed) is
        # restored afterwards; its own math is pinned by the C-side tests.
        shaper = self.shaper_spec() if self.has_shaper else None
        self.set_shaper(0.0)
        # A distinct tick per slot, well inside 0..4095 and distinguishable
        # under any calibration.  Everything below is compared in the TICK
        # domain so a non-uniform calibration cannot masquerade as a
        # permutation (and vice versa).
        ticks = np.array([1200 + 150 * i for i in range(n)], dtype=np.int64)
        vsteps = np.full(n, -137, dtype=np.int64)     # constant: order-blind

        s = SilSensors()
        for i in range(n):
            s.pos_ticks[i] = int(ticks[i])
            s.vel_ticks[i] = int(vsteps[i])
        s.up[2] = 1.0
        if self.spec.num_cmd > 3:
            s.cmd[3] = 1.0
        out = self.tick_raw(s)

        obs = np.asarray(out.obs, dtype=np.float64)
        q_lib = obs[self.spec.slice("q", n)]
        dq_lib = obs[self.spec.slice("dq", n)]
        # invert stepsToAngle to recover which SLOT each joint read
        seen = np.rint(q_lib / RAD_PER_STEP * self.cal.dir).astype(np.int64) \
            + self.cal.zero_steps
        e_joint = int(np.max(np.abs(seen - ticks)))
        e_bus = int(np.max(np.abs(seen - ticks[perm])))
        if forced in ("joint", "ascending_id"):
            self.in_order = self.out_order = forced
        elif e_joint == 0 and e_bus != 0:
            self.in_order = "joint"
        elif e_bus == 0 and e_joint != 0:
            self.in_order = "ascending_id"
        elif e_joint == 0 and e_bus == 0:
            self.in_order = "joint"          # permutation is unobservable
        else:
            raise ProbeFailed(
                "sil_tick did not echo the sensed positions through obs "
                f"(joint-order err {e_joint} ticks, bus-order err {e_bus} "
                "ticks); SilSensors.pos_ticks does not mean what this "
                "harness thinks.  Override with $SIL_ASSUME_ORDER.")

        # velocity encoding: constant fill, so array order cannot confuse it
        expect_dq = steps_s_to_rad_s(vsteps, self.cal)
        self.vel_sign_magnitude = bool(
            np.max(np.abs(dq_lib - expect_dq)) > 1e-3)

        # goal_ticks order: rebuild what the firmware must have written
        act = np.asarray(out.action, dtype=np.float64)
        want = angle_to_steps(action_to_angles(act), self.cal)
        got = np.asarray(out.goal_ticks, dtype=np.int64)
        d_joint = int(np.max(np.abs(got - want)))
        d_bus = int(np.max(np.abs(got - to_bus(want, "ascending_id",
                                               self.spec))))
        if forced not in ("joint", "ascending_id"):
            self.out_order = "joint" if d_joint <= d_bus else "ascending_id"
        if min(d_joint, d_bus) > 1:
            raise ProbeFailed(
                "goal_ticks do not match angleToSteps(actionToAngles(action)) "
                f"under either array order (joint {d_joint}, bus {d_bus} "
                "ticks); probe runs with sil_set_shaper(0), so shaping cannot "
                "explain this")
        if shaper and shaper.get("pole_hz"):
            self.set_shaper(float(shaper["pole_hz"]))   # back to as-deployed
        self.reset()
        return dict(in_order=self.in_order, out_order=self.out_order,
                    vel_sign_magnitude=self.vel_sign_magnitude,
                    probe_pos_err_ticks=min(e_joint, e_bus),
                    probe_goal_err_ticks=min(d_joint, d_bus))

    # -- the boundary --------------------------------------------------
    def make_sensors(self, q, dq, up, gyro, cmd) -> SilSensors:
        """Joint-space state -> the servo-side view the bus would deliver."""
        n = self.spec.num_joints
        ticks = to_bus(angle_to_steps(q, self.cal), self.in_order, self.spec)
        vel = to_bus(rad_s_to_steps_s(dq, self.cal), self.in_order, self.spec)
        s = SilSensors()
        for i in range(n):
            s.pos_ticks[i] = int(ticks[i])
            v = int(vel[i])
            if self.vel_sign_magnitude:
                mag = min(abs(v), 0x7FFF)
                raw = mag | (0x8000 if v < 0 else 0)
                v = raw - 0x10000 if raw >= 0x8000 else raw   # into int16
            s.vel_ticks[i] = int(v)
        for i in range(3):
            s.up[i] = float(up[i])
            s.gyro[i] = float(gyro[i])
        for i in range(min(self.spec.num_cmd, len(cmd))):
            s.cmd[i] = float(cmd[i])
        return s

    def targets_to_action(self, out: SilTargets):
        """goal_ticks (wire order) -> the env action that reproduces exactly
        the angle the firmware commanded."""
        got = from_bus(np.asarray(out.goal_ticks, dtype=np.int64),
                       self.out_order, self.spec)
        angles = steps_to_angle(got, self.cal)
        return angles_to_action(angles).astype(np.float32), angles


# =====================================================================
#  obs frame <-> obs::Inputs
# =====================================================================
def split_obs(obs, spec: ObsSpec = SPEC):
    """walker_env's obs frame -> the firmware's obs::Inputs fields.

    Taking the IMU channels from the OBS (rather than from env.data) is what
    puts the mounting error, the gyro bias and the sensor noise inside the SIL
    loop: the firmware sees exactly the vector the python policy would have
    seen, and the only remaining difference is the tick quantiser.
    """
    n = spec.num_joints
    o = np.asarray(obs, dtype=np.float64)
    return dict(q=o[spec.slice("q", n)], dq=o[spec.slice("dq", n)],
                up=o[spec.slice("up", 3)], gyro=o[spec.slice("gyro", 3)],
                prev_action=o[spec.slice("prev_action", n)],
                cmd=o[spec.slice("cmd", spec.num_cmd)])


def obs_block_errors(a, b, spec: ObsSpec = SPEC):
    """Per-block max|a - b| over one frame, keyed by obs_spec block name."""
    a = np.asarray(a, dtype=np.float64)[:spec.frame_dim]
    b = np.asarray(b, dtype=np.float64)[:spec.frame_dim]
    n = spec.num_joints
    widths = dict(q=n, dq=n, up=3, linvel=3, gyro=3, prev_action=n, height=1,
                  phase=2, cmd=spec.num_cmd)
    return {k: float(np.max(np.abs(a[spec.slice(k, w)] - b[spec.slice(k, w)])))
            for k, w in widths.items()}


# =====================================================================
#  gait-clock alignment
# =====================================================================
def gait_step(freq_hz, dt=None):
    return 2.0 * math.pi * (dt if dt is not None else SPEC.control_dt) * freq_hz


def pin_gait_clock(env, freq_hz=DEFAULT_GAIT_HZ):
    """Align walker_env's gait clock with the firmware's, AFTER env.reset().

    The two advance the clock at opposite ends of the tick: ctrl_task.cpp
    calls GaitClock::advance() BEFORE assembling the frame, walker_env.step()
    advances at the END of the step.  So the firmware's phase at tick k is
    (k+1)*w and walker_env's is p0 + k*w; setting p0 = w makes them equal.
    The history ring is refilled so tick 0 is not fed a stale phase.
    """
    w = gait_step(freq_hz, env.control_dt)
    env._gait_freq = float(freq_hz)
    env._gait_phase = float(w)
    if getattr(env, "obs_hist_len", 1) > 1:
        env._obs_hist = []
        first = env._obs()
        env._obs_hist = [first.copy() for _ in range(env.obs_hist_len - 1)]
    return w


# =====================================================================
#  the act-adapter (drop-in for eval_precision.Driver's `act`)
# =====================================================================
class SilActAdapter:
    """`act(obs) -> action`, but the action comes from the real firmware.

    The obs handed in is walker_env's own frame (Driver recomputes it every
    tick), so the IMU channels the firmware sees are EXACTLY the ones the
    python policy would have seen -- mounting error, bias and noise included.
    The joint channels go through the tick quantiser first, which is the point
    of the exercise.

    Divergence bookkeeping (docs/sil-harness.md pyramid item 3) is collected
    per tick in `self.rows`; pass `py_act` to also log the python policy's
    action on the same obs.
    """

    def __init__(self, lib: SilLib, env, py_act=None, skip_ticks=(),
                 spec: ObsSpec = SPEC, log=True):
        self.lib = lib
        self.env = env
        self.py_act = py_act
        self.skip_ticks = set(int(t) for t in skip_ticks)
        self.spec = spec
        self.log = log
        self.rows = []
        self.tick = 0
        self.episodes = 0
        self._last_action = np.zeros(spec.act_dim, dtype=np.float32)
        self._last_targets = None
        self._frames = []          # newest first, for the training-semantics
                                   # obs reconstruction (see below)

    # -- episode lifecycle --------------------------------------------
    def begin_episode(self):
        self.lib.reset()
        self.tick = 0
        self.episodes += 1
        self._last_action = np.zeros(self.spec.act_dim, dtype=np.float32)
        self._last_targets = None
        self._frames = []

    def training_obs(self, frame):
        """[f_t, f_{t-1}, ..., f_{t-H+1}] -- the stacking walker_env.step()
        and env_mjx both hand the policy during TRAINING, and the one
        obs::History implements.

        eval_precision.Driver re-reads env._obs() at the top of each tick
        (it has to: set_command must land in the current frame), and that
        read prepends a freshly recomputed current frame to a ring which
        walker_env.step() has ALREADY pushed the same frame into.  So the
        referee's live obs is [f_t, f_t, f_{t-1}] -- one frame of real
        history short.  The firmware is not wrong here; the referee's
        live-read path is, and this reconstruction is what the SIL stack
        must be compared against."""
        f = np.asarray(frame, dtype=np.float64)[:self.spec.frame_dim]
        prev = list(self._frames) or [f]
        while len(prev) < self.spec.hist_len - 1:
            prev.append(prev[-1])
        return np.concatenate([f] + prev[:self.spec.hist_len - 1])

    def split_obs(self, obs):
        return split_obs(obs, self.spec)

    # -- the callable --------------------------------------------------
    def __call__(self, obs):
        if getattr(self.env, "_step_i", None) == 0 and self.tick != 0:
            self.begin_episode()
        elif self.tick == 0 and self.episodes == 0:
            self.begin_episode()

        if self.tick in self.skip_ticks:
            # Simulated tick overrun: the firmware watchdog holds the last
            # commanded target (pyramid item 4).  No sil_tick, no state
            # advance -- exactly what a missed 20 ms slot looks like.
            if self.log:
                self.rows.append(dict(tick=self.tick, skipped=True,
                                      action=self._last_action.copy()))
            self.tick += 1
            return self._last_action.copy()

        fields = self.split_obs(obs)
        s = self.lib.make_sensors(fields["q"], fields["dq"], fields["up"],
                                  fields["gyro"], fields["cmd"])
        out = self.lib.tick_raw(s)
        action, angles = self.lib.targets_to_action(out)

        if self.log:
            lib_obs = np.asarray(out.obs, dtype=np.float64)
            fd, hl = self.spec.frame_dim, self.spec.hist_len
            train_obs = self.training_obs(obs)
            row = dict(tick=self.tick, skipped=False,
                       action=action.copy(),
                       raw_action=np.asarray(out.action, dtype=np.float64),
                       target_rad=angles,
                       obs_block_err=obs_block_errors(lib_obs, obs, self.spec))
            row["obs_frame_err"] = max(row["obs_block_err"].values())
            # the library's history ring vs the frames actually produced
            row["hist_err"] = float(np.max(np.abs(
                lib_obs[fd:hl * fd] - train_obs[fd:hl * fd]))) if hl > 1 else 0.0
            if self.py_act is not None:
                # (a) same-input parity: the firmware's net vs brax's on the
                #     library's OWN obs -- pure numerical divergence.
                a_lib = np.asarray(self.py_act(lib_obs), dtype=np.float64)
                row["net_err"] = float(np.max(np.abs(a_lib - row["raw_action"])))
                # (b) end-to-end against the TRAINING obs stacking
                a_tr = np.asarray(self.py_act(train_obs.astype(np.float32)),
                                  dtype=np.float64)
                row["train_action_err"] = float(
                    np.max(np.abs(a_tr - row["raw_action"])))
                # (c) end-to-end against the referee's live-read obs
                a_py = np.asarray(self.py_act(obs), dtype=np.float64)
                row["py_action"] = a_py
                row["action_err"] = float(np.max(np.abs(a_py - action)))
                row["raw_action_err"] = float(
                    np.max(np.abs(a_py - row["raw_action"])))
            self.rows.append(row)

        self._frames.insert(0, np.asarray(obs, dtype=np.float64)
                            [:self.spec.frame_dim].copy())
        del self._frames[max(self.spec.hist_len - 1, 1):]
        self._last_action = action
        self._last_targets = out
        self.tick += 1
        return action

    # -- summary -------------------------------------------------------
    def divergence(self):
        rows = [r for r in self.rows if not r["skipped"]]
        if not rows:
            return {}
        out = dict(ticks=len(rows),
                   obs_frame_err_max=max(r.get("obs_frame_err", 0.0)
                                         for r in rows))
        if "obs_block_err" in rows[0]:
            out["obs_block_err_max"] = {
                k: max(r["obs_block_err"][k] for r in rows)
                for k in rows[0]["obs_block_err"]}
        if "hist_err" in rows[0]:
            out["hist_err_max"] = max(r["hist_err"] for r in rows)
        if "action_err" in rows[0]:
            for key in ("action_err", "raw_action_err", "net_err",
                        "train_action_err"):
                v = np.array([r[key] for r in rows])
                out[key + "_max"] = float(v.max())
                out[key + "_mean"] = float(v.mean())
        return out


# =====================================================================
#  convenience: open everything
# =====================================================================
def open_lib(run=DEFAULT_RUN, cal="nominal", gait_hz=DEFAULT_GAIT_HZ,
             probe=True) -> SilLib:
    """Load libctrl_sil with this run's exported weights.  Raises LibMissing
    if the library (or the export) is not there yet."""
    path = find_lib()
    if path is None:
        raise LibMissing(
            "libctrl_sil not found -- build it with `make -C firmware/host "
            "sil` (searched %s, or set $SIL_LIB)"
            % ", ".join(LIB_DIRS))
    weights = default_weights(run)
    if not os.path.exists(weights):
        raise LibMissing(
            f"{weights} missing -- run tools/export_policy_weights.py "
            f"--run {run}")
    cal_path = cal if os.path.sep in str(cal) else os.path.join(
        CAL_DIR, f"cal_{cal}.json")
    if not os.path.exists(cal_path):
        write_cal_files()
    return SilLib(path, weights, cal_path, gait_hz=gait_hz, probe=probe)
