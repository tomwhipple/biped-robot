"""Command sources: the three things allowed to steer the robot.

All of them live laptop-side and emit the same (vx, wz, flags) triple, so the
robot only ever learns one wire format (link/protocol.py) no matter who is
driving. Adding a fourth commander is a class here, not a firmware change.

  script  -- time-scripted sequences, the on-hardware twin of
             sim/eval_commands.py, so a real run and a sim eval are the same
             shape of test.
  gamepad -- sticks, with a held dead-man. Hand-driving.
  goal    -- proportional goal-seeker; the seam the goal-conditioning layer
             plugs into. SIM-ONLY today (see GoalSource).
"""
import math

from protocol import (FLAG_ENABLE, FLAG_ESTOP, V_MAX, V_MIN_WALK, W_MAX)

# Sticks rest noisily around zero; below this the axis reads as "let go".
STICK_DEADZONE = 0.15


class CommandSource:
    """Emit (vx m/s, wz rad/s, flags) for wall-clock time t (seconds)."""

    name = "base"

    def poll(self, t: float) -> tuple:
        raise NotImplementedError

    def close(self):
        pass


def stick_to_speed(axis: float) -> float:
    """Map a [-1,1] stick axis onto the *trained* speed band.

    The naive `vx = axis * V_MAX` is wrong on this plant: walker_env draws
    commands from stand (0,0) or a walk in [0.3, 1.0], so (0, 0.3) is a hole
    in the training distribution. A stick at 20 % would ask for 0.2 m/s --
    a speed the policy has never once been rewarded for tracking, and the
    behavior there is undefined rather than merely slow.

    So: dead zone snaps to a stand, and everything above it is remapped onto
    [V_MIN_WALK, V_MAX] -- the stick's full travel buys real, trained speeds.
    """
    mag = abs(axis)
    if mag < STICK_DEADZONE:
        return 0.0
    frac = (mag - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)
    return math.copysign(V_MIN_WALK + frac * (V_MAX - V_MIN_WALK), axis)


# -- scripted --------------------------------------------------------------
def _stand(t):
    return 0.0, 0.0


def _walk(t):
    return 0.6, 0.0


def _dash_stop(t):
    # eval_commands.scen_dash_stop keyed on x >= 2 m; we have no odometry off
    # the robot, so the hardware twin keys on time instead.
    return (0.8, 0.0) if t < 4.0 else (0.0, 0.0)


def _line_1m(t):
    # eval_precision.scen_line_1m: 1 s settle, 0.4 m/s until x crosses 1 m.
    # Keyed on time like dash_stop; 2.73 s is loco_v14turn_s128's referee
    # mean time_to_1m (sim/runs/loco_v14turn_s128/scorecard.json), NOT a
    # hardware measurement -- tape the floor and correct it.
    if t < 1.0:
        return 0.0, 0.0
    return (0.4, 0.0) if t < 1.0 + 2.73 else (0.0, 0.0)


def _turn(t):
    return (0.4, 0.5) if t % 6.0 < 3.0 else (0.4, -0.5)


def _pivot(t):
    return (0.0, 0.5) if t % 6.0 < 3.0 else (0.0, -0.5)


def _square(t):
    # 3 s straight, then a ~90 deg pivot at 0.8 rad/s (1.96 s), repeat.
    leg = 3.0 + math.pi / 2 / 0.8
    return (0.5, 0.0) if t % leg < 3.0 else (0.0, 0.8)


def _rom_feet(t):
    """Policy-driven foot range-of-motion, through the TRAINED foot-target
    channels (2026-08-31): each foot in turn -- target forward, target back,
    down to rest. These are cm-scale offsets (traj_radius draws +-5 cm); a
    90-degree leg swing is bench `pose` territory, not the policy's.

    Dict scripts carry any subset of the 7 ext_cmd channels; omitted keys
    ride at their trained defaults. lift: -1 = left foot, +1 = right."""
    def leg(lift, tl):
        if tl < 3.0:
            return dict(lift=lift, foot_dx=+0.04)
        if tl < 6.0:
            return dict(lift=lift, foot_dx=-0.04)
        return {}
    if t < 2.0:
        return {}
    if t < 8.0:
        return leg(-1.0, t - 2.0)      # left foot fwd, back
    if t < 10.0:
        return {}
    if t < 16.0:
        return leg(+1.0, t - 10.0)     # right foot fwd, back
    return {}


def _march(t):
    """Knee-high march in place: the lift channel alternates on a 1.5 s
    cadence with a raised swing target, the trained march command shape
    (walker_env traj_on == 4)."""
    import math as _m
    if t < 2.0:
        return {}
    th = 2 * _m.pi * (t - 2.0) / 1.5
    left = _m.sin(th) > 0
    return dict(lift=-1.0 if left else 1.0,
                foot_dz=0.04 * abs(_m.sin(th)))


EXT_KEYS = ("vy", "crouch", "lift", "foot_dx", "foot_dz")

# Which referee scenario grades each script's skill. arm_script's hardware
# preflight looks up the DEPLOYED run's row before arming: on 2026-08-31 a
# crouch was commanded on the standing robot while the scorecard already
# said squat_reps 0/8 -- the robot ignored it and fell. The scorecard is
# the record of what the deployed policy can do; consult it by machine,
# not by memory. Unmapped scripts skip the check.
SCRIPT_SCENARIO = {
    "walk": "line_1m",
    "line_1m": "line_1m",
    "stride1": "line_1m",
    "dash_stop": "speed_ladder",
    "turn": "turn_180",
    "pivot": "turn_180",
    "square": "square_return",
    "crouch1": "squat_reps",
    "sidestep1": "sidestep_L",
    "march": "metronome",
    "stand": "stand_10s",
}

def _stride1(t):
    # ONE gait-clock cycle of walk (the deployed clock runs 1.5 Hz fixed;
    # clock_stand_freeze holds phase outside the window), stand either side.
    return (0.6, 0.0) if 2.0 <= t < 2.0 + 1.0 / 1.5 else (0.0, 0.0)


def _crouch1(t):
    # One crouch cycle through the trained crouch channel (cmd[3] drawn
    # 0.7-1.0 in training): ramp down, hold, ramp back up. Ramps rather
    # than steps -- training resamples the command, it never sees jumps
    # mid-episode... but a slow ramp is inside the distribution everywhere.
    if t < 2.0:
        return dict(crouch=1.0)
    if t < 4.0:
        return dict(crouch=1.0 - 0.3 * (t - 2.0) / 2.0)
    if t < 6.0:
        return dict(crouch=0.7)
    if t < 8.0:
        return dict(crouch=0.7 + 0.3 * (t - 6.0) / 2.0)
    return dict(crouch=1.0)


def _sidestep1(t):
    # ONE gait-clock cycle of leftward sidestep (vy +0.2 = scen_sidestep L,
    # the deployed line's strongest referee skill: 8/8).
    if 2.0 <= t < 2.0 + 1.0 / 1.5:
        return dict(vy=0.2)
    return {}


SCRIPTS = {
    "stand": _stand,
    "stride1": _stride1,
    "sidestep1": _sidestep1,
    "crouch1": _crouch1,
    "rom_feet": _rom_feet,
    "march": _march,
    "walk": _walk,
    "dash_stop": _dash_stop,
    "line_1m": _line_1m,
    "turn": _turn,
    "pivot": _pivot,
    "square": _square,
}


class ScriptSource(CommandSource):
    def __init__(self, script: str, duration: float | None = None):
        if script not in SCRIPTS:
            raise ValueError(f"unknown script {script!r}; "
                             f"have {sorted(SCRIPTS)}")
        self.name = f"script:{script}"
        self._fn = SCRIPTS[script]
        self._duration = duration

    def poll(self, t):
        if self._duration is not None and t >= self._duration:
            return 0.0, 0.0, 0        # disabled -> robot stands
        out = self._fn(t)
        if isinstance(out, dict):
            # dict scripts: any subset of the 7 ext_cmd channels
            ext = {k: out[k] for k in EXT_KEYS if k in out}
            return out.get("vx", 0.0), out.get("wz", 0.0), FLAG_ENABLE, ext
        v, w = out
        return v, w, FLAG_ENABLE


# -- gamepad ---------------------------------------------------------------
class GamepadSource(CommandSource):
    """Sticks via pygame, with a held dead-man.

    Dead-man is held-to-enable rather than toggled: letting go of the pad --
    or dropping it -- must stop the robot, and a toggle fails that test.
    """

    name = "gamepad"

    # Xbox/PS layout under SDL2.
    AX_FWD = 1          # left stick Y (up is negative)
    AX_YAW = 2          # right stick X
    BTN_DEADMAN = 5     # right shoulder
    BTN_ESTOP = 1       # B / circle

    def __init__(self, index: int = 0):
        try:
            import pygame
        except ImportError as e:
            raise SystemExit(
                "gamepad source needs pygame: pip install pygame") from e
        self._pygame = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= index:
            raise SystemExit(f"no gamepad at index {index} "
                             f"({pygame.joystick.get_count()} connected)")
        self._js = pygame.joystick.Joystick(index)
        self._js.init()
        self.name = f"gamepad:{self._js.get_name()}"

    def poll(self, t):
        self._pygame.event.pump()
        if self._js.get_button(self.BTN_ESTOP):
            return 0.0, 0.0, FLAG_ESTOP
        if not self._js.get_button(self.BTN_DEADMAN):
            return 0.0, 0.0, 0
        vx = stick_to_speed(-self._js.get_axis(self.AX_FWD))
        yaw = self._js.get_axis(self.AX_YAW)
        # Yaw has no training hole -- any |wz| <= 1.0 is in-distribution -- so
        # it gets a plain dead zone and a linear map, unlike stick_to_speed.
        wz = 0.0 if abs(yaw) < STICK_DEADZONE else -yaw * W_MAX
        return vx, wz, FLAG_ENABLE

    def close(self):
        self._pygame.quit()


# -- goal-seeking ----------------------------------------------------------
# Goal-seeker gains. These aim at the MIDDLE of the trained command
# distribution, not its edges, and that is the whole trick -- see the class
# docstring. Cruise is mid-band; yaw is capped below cmd_w_range's 1.0.
V_CRUISE = 0.6
W_GOAL_MAX = 0.8
FACE_FIRST_RAD = 1.2


class GoalSource(CommandSource):
    """Proportional goal-seeker: arc toward the goal, pivot only if lost.

    This is the seam the goal-conditioning work plugs into -- swap this
    controller for a learned policy and nothing else in the stack moves.

    **Aim at the middle of the trained distribution, not its edge.** The
    obvious controller (high yaw gain, turn-to-face, then sprint at V_MAX)
    falls over, and measurably so: it asks for pivots at wz=1.0, which is the
    exact upper bound of walker_env's cmd_w_range and a regime eval_commands
    only ever validates at +-0.5. Meanwhile walking-while-turning is the
    best-trained behavior there is -- 60 % of walker_env's moving commands
    pair a v with a w. So: arc toward the goal by default, cruise at a
    mid-band 0.6 m/s rather than the 1.0 m/s ceiling, cap yaw at 0.8, and
    pivot in place only when the goal is really behind you (> ~70 deg).
    Being in-distribution beats being fast.

    SIM-ONLY until the robot can estimate its own pose. pose_fn wants world
    (x, y, yaw); MuJoCo hands that over for free, but the real robot has no
    odometry -- the IMU gives attitude only, and torso linear velocity and
    height have no direct sensor at all. Closing that gap (leg odometry on
    the ESP32, or visual odometry on the Pi) is its own piece of work.
    """

    name = "goal"

    def __init__(self, goal: tuple, pose_fn, tol: float = 0.25,
                 k_yaw: float = 1.0, face_first_rad: float = FACE_FIRST_RAD,
                 cruise: float = V_CRUISE):
        self.goal = tuple(goal)
        self._pose = pose_fn
        self._tol = tol
        self._k_yaw = k_yaw
        self._face_first = face_first_rad
        self._cruise = cruise
        self.name = f"goal:({goal[0]:.1f},{goal[1]:.1f})"

    def poll(self, t):
        x, y, yaw = self._pose()
        dx, dy = self.goal[0] - x, self.goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < self._tol:
            return 0.0, 0.0, FLAG_ENABLE      # arrived: stand on the spot
        err = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                         math.cos(math.atan2(dy, dx) - yaw))
        wz = max(-W_GOAL_MAX, min(W_GOAL_MAX, self._k_yaw * err))
        if abs(err) > self._face_first:
            # Goal is behind us: pivot. Walking a wide arc backwards wastes
            # battery and floor, and this is the one case worth the risk.
            return 0.0, wz, FLAG_ENABLE
        # Ease off near the goal, but never into the untrained (0, 0.3) band --
        # clamp_to_envelope would snap that to a stand anyway: stop honestly.
        v = self._cruise if dist > 1.0 else max(V_MIN_WALK,
                                                self._cruise * dist)
        return v, wz, FLAG_ENABLE


class PoseFeed:
    """Ground-truth (x, y, yaw) from a simulator's debug pose socket.

    SIM ONLY, and deliberately not part of the robot protocol: it exists so
    the goal-seeking layer can be built and demonstrated against a real UDP
    link now, with the odometry gap represented by exactly one swappable
    object. When the robot can estimate its own pose, this class is what gets
    replaced -- nothing else in link/ knows the difference.
    """

    def __init__(self, port: int = 4212):
        import socket
        import struct
        self._struct, self._pose = struct, (0.0, 0.0, 0.0)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setblocking(False)

    def __call__(self):
        while True:                       # drain to the freshest pose
            try:
                buf = self._sock.recv(32)
            except BlockingIOError:
                return self._pose
            if len(buf) == 12:
                self._pose = self._struct.unpack("<3f", buf)

    def close(self):
        self._sock.close()


def build(args, pose_fn=None) -> CommandSource:
    """Construct the source an argparse namespace asks for."""
    if args.source == "script":
        return ScriptSource(args.script, duration=args.duration)
    if args.source == "gamepad":
        return GamepadSource(args.gamepad_index)
    if args.source == "goal":
        if not args.goal:
            raise SystemExit("goal source needs --goal X Y")
        return GoalSource(tuple(args.goal),
                          pose_fn or PoseFeed(getattr(args, "pose_port",
                                                      4212)))
    raise ValueError(f"unknown source {args.source!r}")
