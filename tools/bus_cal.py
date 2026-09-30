"""The 17-servo bus map + calibration blob v3, shared by every bench tool.

Single source of truth for "which servos are on the robot, under which IDs,
with what zero/direction." Before this module every tool hard-coded the
prototype's 10-servo `CAL = [(name, id, zero, dir), ...]` table; after the
17-servo bus landed (issue #81, PR #94) the live firmware's calibration is
the only safe source.

How a tool uses this:

    from bus_cal import BusCal, fetch_cal, BUS, POCKET
    s = serial.Serial(...)                     # NOT this module's problem
    cal = fetch_cal(send_cmd_fn)               # parse `cal show` from the board
    ticks = cal.pose_ticks({"L_knee": -30.0})  # 17 values, servo-ID order

`fetch_cal()` is the only function that needs a serial port, and it takes
the caller's `cmd(text) -> reply_text` callable so this module never imports
pyserial. The helper is therefore pure-Python and unit-testable offline.

The bus map (`BUS`, `POCKET`, `POLICY_ORDER`) is duplicated from
firmware/components/obs/include/obs/bus_map.h on purpose: the bench tools
cannot import C headers. If the firmware map changes, change this file and
docs/servo-map.md section 2.1 in the same commit (the SIL suite checks the
three agree).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

TICKS_PER_DEG: float = 4096.0 / 360.0  # STS3215: 12-bit encoder on the shaft

# The 17-servo bus layout. Docs/servo-map.md §2.1 is the proposal;
# firmware/components/obs/include/obs/bus_map.h is what the firmware compiles;
# this list is what the bench tools read. ALL THREE MUST AGREE (the SIL
# suite's test_bus_map_is_one_map_everywhere checks the firmware side).
# (name, id, port) — index b is servo ID b + 1 (bus_map.h's
# busIdsAreIndexPlusOne() static_assert).
BUS: Tuple[Tuple[str, int, str], ...] = (
    ("R_hip_roll",   1,  "A"),
    ("R_hip_pitch",  2,  "A"),
    ("R_knee",       3,  "A"),
    ("R_ankle",      4,  "A"),   # pitch
    ("L_hip_roll",   5,  "B"),
    ("L_hip_pitch",  6,  "B"),
    ("L_knee",       7,  "B"),
    ("L_ankle",      8,  "B"),   # pitch
    ("R_hip_yaw",    9,  "A"),
    ("L_hip_yaw",    10, "B"),
    ("R_ankle_roll", 11, "A"),
    ("L_ankle_roll", 12, "B"),
    ("neck_yaw",     13, "A"),
    ("R_shoulder",   14, "A"),
    ("R_elbow",      15, "A"),
    ("L_shoulder",   16, "B"),
    ("L_elbow",      17, "B"),
)

N_BUS: int = len(BUS)
assert N_BUS == 17, "the bus map changed without updating N_BUS"

BUS_NAMES: Tuple[str, ...] = tuple(b[0] for b in BUS)
BUS_IDS: Tuple[int, ...] = tuple(b[1] for b in BUS)
BUS_PORTS: Tuple[str, ...] = tuple(b[2] for b in BUS)

# Sanity-pin the layout invariant bus_map.h asserts in C++: index b is servo
# ID b+1. The Python tools read this list positionally to assemble 17-wide
# pose strings, so a permutation here would silently retarget commands.
for _b, _row in enumerate(BUS):
    assert _row[1] == _b + 1, f"BUS row {_b} has id {_row[1]}, expected {_b + 1}"

# The 10-servo prototype's policy order. This is what `pose`-with-10-targets
# means and what the legacy tools' CAL tables were indexed by. The bus-side
# pose of those tools is generated from this same mapping (and the helper's
# `BusCal.pose_ticks` returns the 17-wide bus form). Source: docs/servo-map.md
# §2.2 / firmware/components/obs/include/obs/obs_spec.h (legacy 10-joint runs);
# firmware/main/cal_store.h's kLegacyServoId is the same map and would catch a
# stale edit at boot.
POLICY_ORDER: Tuple[str, ...] = (
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle",
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle",
)

# The prototype's 2026-09-03 as-built calibration (firmware/main/asbuilt_cal.h,
# docs/servo-map.md §3.1). Fallback ONLY, used when the bus answers but
# `cal show` cannot be parsed; never trusted on its own. The values are
# already in bus-indexed form here (POCKET is keyed by name).
_ASBUILT_BY_NAME: Dict[str, Tuple[int, int, int]] = {
    # name: (id, zero, dir)
    "L_hip_yaw":   (10, 1692, +1),
    "L_hip_roll":  (5,  2418, -1),
    "L_hip_pitch": (6,  2001, +1),
    "L_knee":      (7,  1581, -1),
    "L_ankle":     (8,  3535, +1),
    "R_hip_yaw":   (9,  1806, +1),
    "R_hip_roll":  (1,  3532, -1),
    "R_hip_pitch": (2,  2479, +1),
    "R_knee":      (3,  2063, -1),
    "R_ankle":     (4,  3437, +1),
}


class BusCalError(RuntimeError):
    """The calibration source is unusable (parse failed, map mismatch)."""


@dataclass(frozen=True)
class BusJoint:
    """One row of the calibration: the static map entry plus the live blob."""
    name: str
    bus_index: int        # 0..16
    servo_id: int         # 1..17 (bus_index + 1)
    port: str             # 'A' or 'B'
    zero_steps: int       # encoder ticks at the joint-0 pose
    direction: int        # +1 or -1
    fitted: bool          # whether the robot carries this servo

    def deg_to_ticks(self, deg: float) -> int:
        """Joint-space offset (degrees, sim frame) to absolute encoder ticks."""
        return int(round(self.zero_steps + self.direction * deg * TICKS_PER_DEG))

    def ticks_to_deg(self, ticks: int) -> float:
        """Encoder ticks to joint-space angle in degrees, sim frame."""
        return (ticks - self.zero_steps) * self.direction / TICKS_PER_DEG


@dataclass
class BusCal:
    """The 17-servo calibration, indexed by bus joint.

    `joints[i]` is servo ID i+1. The two pose-assembly helpers produce the
    17-wide bus-order list the firmware's `pose` wants; per-joint targets
    start at the calibrated zero so a joint the caller does not name is
    held at its standing pose. The firmware skips fitted==False rows on
    the wire, so writing their zero is safe but a no-op.

    Source marker: 'nvs' when the firmware said "loaded from NVS at boot";
    'defaults' when it said DEFAULTS (a fresh, uncalibrated cal); 'asbuilt'
    when this BusCal came from the compiled-in prototype table.
    """
    joints: List[BusJoint]
    source: str = "nvs"

    def by_name(self, name: str) -> BusJoint:
        for j in self.joints:
            if j.name == name:
                return j
        raise BusCalError(f"no bus joint named {name!r}")

    def by_id(self, servo_id: int) -> BusJoint:
        for j in self.joints:
            if j.servo_id == servo_id:
                return j
        raise BusCalError(f"no bus joint for servo id {servo_id}")

    def fitted_joints(self) -> List[BusJoint]:
        return [j for j in self.joints if j.fitted]

    def pose_ticks(
        self,
        offsets_deg: Optional[Dict[str, float]] = None,
        only: Optional[List[str]] = None,
    ) -> List[int]:
        """Build the 17-wide servo-ID-order tick list for firmware pose.

        `offsets_deg` maps joint name -> sim-frame degree offset from zero.
        Names not in `offsets_deg` are held at their calibrated zero.

        `only` (optional) restricts the move to those joints; other FITTED
        joints also get their zero (same as today), but UNFITTED joints
        always do (the firmware skips them on the wire anyway).

        Always returns exactly 17 ints (matches kNumBusJoints).
        """
        offsets_deg = offsets_deg or {}
        out: List[int] = []
        for j in self.joints:
            tgt_deg = offsets_deg.get(j.name, 0.0)
            if only is not None and j.name not in only:
                tgt_deg = 0.0
            out.append(j.deg_to_ticks(tgt_deg))
        assert len(out) == N_BUS
        return out

    def policy_pose_ticks(
        self, offsets_deg: Optional[Dict[str, float]] = None
    ) -> List[int]:
        """Legacy 10-wide pose in policy order (what `pose` accepts when
        kNumJoints != kNumBusJoints; only valid while the deployed policy
        still drives the prototype's 10 joints).

        For new code, use pose_ticks() -- it sends the canonical 17-wide
        bus form, which remains correct under a robot policy too.
        """
        offsets_deg = offsets_deg or {}
        out = []
        for name in POLICY_ORDER:
            j = self.by_name(name)
            out.append(j.deg_to_ticks(offsets_deg.get(name, 0.0)))
        return out


# -----------------------------------------------------------------------------
# Parsing `cal show`
# -----------------------------------------------------------------------------
#
# The firmware's cmdCal prints (cli.cpp:1113):
#
#     R_hip_roll   id  1  zero 3532  dir -1  fitted
#     ... (16 more rows, one per bus joint, servo-ID order)
#     loaded from NVS at boot
#   or
#     DEFAULTS -- nothing stored (cal save)
#
# We accept either `fitted` or `NOT FITTED` as the final token; older blobs
# (v1/v2) always print `fitted` for their 10 rows and never print a row at
# all for unfitted joints. Blob v3 always emits 17 rows.
#
_CAL_ROW_RE = re.compile(
    r"^\s*(\w+)\s+id\s+(\d+)\s+zero\s+(-?\d+)\s+dir\s+([+-]?\d+)\s+"
    r"(fitted|NOT FITTED)\s*$",
    re.MULTILINE,
)


def parse_cal_show(text: str) -> BusCal:
    """Parse `cal show` output into a BusCal.

    Raises BusCalError on any of:
      * fewer/more than 17 parseable rows;
      * a row's name is not in the bus map;
      * a row's servo ID disagrees with the bus map (the proposal in
        docs/servo-map.md changed without re-flashing);
      * the direction is something other than +1/-1;
      * the terminator line ("loaded from NVS" / "DEFAULTS") is missing.

    Anything less than a clean parse is a refuse: the calibration is the
    only thing standing between a `pose` and a flipped leg, and a
    half-parsed table silently shifting IDs is exactly the 2026-08-02
    failure class.
    """
    rows: List[Tuple[str, int, int, int, bool]] = []
    for m in _CAL_ROW_RE.finditer(text):
        name = m.group(1)
        sid = int(m.group(2))
        zero = int(m.group(3))
        dirn = int(m.group(4))
        fitted = m.group(5) == "fitted"
        if dirn not in (+1, -1):
            raise BusCalError(f"cal show: {name} dir {dirn} (want +1/-1)")
        rows.append((name, sid, zero, dirn, fitted))

    if len(rows) != N_BUS:
        raise BusCalError(
            f"cal show: parsed {len(rows)} rows, want {N_BUS} (bus map v3)"
        )

    if "loaded from NVS" in text:
        source = "nvs"
    elif "DEFAULTS" in text:
        source = "defaults"
    else:
        raise BusCalError(
            "cal show: missing the NVS/DEFAULTS trailer line (got:"
            f" {text.strip()[-80:]!r})"
        )

    joints: List[BusJoint] = []
    seen_ids = set()
    for name, sid, zero, dirn, fitted in rows:
        if name not in BUS_NAMES:
            raise BusCalError(f"cal show: unknown joint {name!r}")
        if sid != BUS_IDS[BUS_NAMES.index(name)]:
            raise BusCalError(
                f"cal show: {name} is id {sid} on the board, "
                f"{BUS_IDS[BUS_NAMES.index(name)]} in the bus map"
            )
        if sid in seen_ids:
            raise BusCalError(f"cal show: servo id {sid} listed twice")
        seen_ids.add(sid)
        b = sid - 1
        joints.append(BusJoint(
            name=name,
            bus_index=b,
            servo_id=sid,
            port=BUS_PORTS[b],
            zero_steps=zero,
            direction=dirn,
            fitted=fitted,
        ))

    joints.sort(key=lambda j: j.bus_index)
    return BusCal(joints=joints, source=source)


def asbuilt_prototype_cal() -> BusCal:
    """The compiled-in 10-servo prototype calibration as a BusCal.

    Servos 11..17 are reported NOT FITTED at the v3 default (zero 2048,
    dir +1), matching what the firmware's v2->v3 migration produces for the
    prototype's blob (cal_store.cpp calMigrateV2). Use ONLY as a fallback
    when the live board is unreachable and the robot is known to be the
    10-servo prototype.
    """
    joints: List[BusJoint] = []
    for b, (name, sid, port) in enumerate(BUS):
        if name in _ASBUILT_BY_NAME:
            _id, zero, dirn = _ASBUILT_BY_NAME[name]
            assert _id == sid, (name, _id, sid)
            fitted = True
        else:
            zero, dirn, fitted = 2048, +1, False
        joints.append(BusJoint(
            name=name, bus_index=b, servo_id=sid, port=port,
            zero_steps=zero, direction=dirn, fitted=fitted,
        ))
    return BusCal(joints=joints, source="asbuilt")


def fetch_cal(
    cmd: Callable[[str, str], str],
    allow_asbuilt_fallback: bool = True,
) -> Tuple[BusCal, str]:
    """Send `cal show` through `cmd` and parse the reply.

    `cmd(line, until_regex) -> full_reply_text` is the caller's transport —
    the helper never opens a serial port itself.

    Returns (cal, source_description_for_log).

    Fallback chain:
      1. NVS-loaded v3 blob (clean parse, source='nvs').
      2. Firmware DEFAULTS header (clean parse, source='defaults') —
         the board booted uncalibrated; we surface that, callers decide.
      3. `allow_asbuilt_fallback=True` AND the parse failed: the asbuilt
         prototype table (10 fitted joints at the 2026-09-03 remeasure),
         source='asbuilt'. Callers MUST log the fallback and SHOULD refuse
         the run if the fitted set is not exactly the prototype's ten.

    Anything else raises BusCalError; the bench script aborts.
    """
    text = cmd("cal show", r"(loaded from NVS|DEFAULTS)")
    try:
        cal = parse_cal_show(text)
    except BusCalError as e:
        if not allow_asbuilt_fallback:
            raise
        return asbuilt_prototype_cal(), (
            f"asbuilt (fell back: {e})"
        )
    return cal, cal.source
