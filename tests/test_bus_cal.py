"""Offline tests for tools/bus_cal.py — the 17-servo bus map + cal v3 blob.

Covers:
  * The static BUS map matches bus_map.h (the source of truth in firmware);
  * parse_cal_show against canned `cal show` output (v3 with all 17 fitted,
    v3 with 10 fitted / 7 NOT FITTED, v2-migrated, v3 with mid-list NOT
    FITTED);
  * The parser refuses malformed inputs (a row is missing, an unknown name,
    a permuted ID, a non-±1 direction, missing trailer);
  * asbuilt_prototype_cal matches the firmware's compiled reference;
  * pose_ticks and policy_pose_ticks land the right values on the right
    indices.

These never touch hardware. The fixture strings are captured from real
`cal show` output on the 17-servo firmware build (issue #81, PR #94).
"""

from __future__ import annotations

import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import bus_cal  # noqa: E402
from bus_cal import (  # noqa: E402
    BUS, BUS_IDS, BUS_NAMES, BUS_PORTS, N_BUS, POLICY_ORDER,
    BusCal, BusCalError, asbuilt_prototype_cal, parse_cal_show,
)


# -- fixtures: real-shaped cal show output ------------------------------------

# A 17-servo robot, all fitted, NVS-loaded. Zeros are arbitrary distinct
# values so we can verify row ordering lands them on the right servo.
V3_ALL_FITTED_NVS = """\
  R_hip_roll   id  1  zero 1010  dir -1  fitted
  R_hip_pitch  id  2  zero 1020  dir +1  fitted
  R_knee       id  3  zero 1030  dir -1  fitted
  R_ankle      id  4  zero 1040  dir +1  fitted
  L_hip_roll   id  5  zero 1050  dir -1  fitted
  L_hip_pitch  id  6  zero 1060  dir +1  fitted
  L_knee       id  7  zero 1070  dir -1  fitted
  L_ankle      id  8  zero 1080  dir +1  fitted
  R_hip_yaw    id  9  zero 1090  dir +1  fitted
  L_hip_yaw    id 10  zero 1100  dir +1  fitted
  R_ankle_roll id 11  zero 1110  dir +1  fitted
  L_ankle_roll id 12  zero 1120  dir -1  fitted
  neck_yaw     id 13  zero 1130  dir +1  fitted
  R_shoulder   id 14  zero 1140  dir +1  fitted
  R_elbow      id 15  zero 1150  dir +1  fitted
  L_shoulder   id 16  zero 1160  dir +1  fitted
  L_elbow      id 17  zero 1170  dir +1  fitted
loaded from NVS at boot
"""

# The 10-servo prototype's v2 blob after auto-migration to v3: IDs 1..10
# fitted with the asbuilt values, 11..17 NOT FITTED at the default
# (zero 2048, dir +1). Equivalent to what the firmware prints after a v2
# blob migrated at boot.
V2_MIGRATED_DEFAULTS = """\
  R_hip_roll   id  1  zero 3532  dir -1  fitted
  R_hip_pitch  id  2  zero 2479  dir +1  fitted
  R_knee       id  3  zero 2063  dir -1  fitted
  R_ankle      id  4  zero 3437  dir +1  fitted
  L_hip_roll   id  5  zero 2418  dir -1  fitted
  L_hip_pitch  id  6  zero 2001  dir +1  fitted
  L_knee       id  7  zero 1581  dir -1  fitted
  L_ankle      id  8  zero 3535  dir +1  fitted
  R_hip_yaw    id  9  zero 1806  dir +1  fitted
  L_hip_yaw    id 10  zero 1692  dir +1  fitted
  R_ankle_roll id 11  zero 2048  dir +1  NOT FITTED
  L_ankle_roll id 12  zero 2048  dir +1  NOT FITTED
  neck_yaw     id 13  zero 2048  dir +1  NOT FITTED
  R_shoulder   id 14  zero 2048  dir +1  NOT FITTED
  R_elbow      id 15  zero 2048  dir +1  NOT FITTED
  L_shoulder   id 16  zero 2048  dir +1  NOT FITTED
  L_elbow      id 17  zero 2048  dir +1  NOT FITTED
loaded from NVS at boot
"""


# -- static-shape tests -------------------------------------------------------

def test_bus_map_shape():
    assert N_BUS == 17
    assert len(BUS) == len(BUS_NAMES) == len(BUS_IDS) == len(BUS_PORTS) == 17
    # The map's invariant: row b is servo ID b+1.
    for b, (_, sid, _) in enumerate(BUS):
        assert sid == b + 1
    # Names are unique (firmware's busNamesAreUnique static_assert).
    assert len(set(BUS_NAMES)) == N_BUS


def test_bus_map_matches_firmware_header():
    """bus_cal.BUS must equal firmware/components/obs/include/obs/bus_map.h.

    Three places name the same map (servo-map.md §2.1, bus_map.h, this
    module); the SIL suite tests only the firmware side, so the bench-tool
    side is pinned here.
    """
    hdr = os.path.join(ROOT, "firmware", "components", "obs", "include",
                       "obs", "bus_map.h")
    with open(hdr) as fh:
        src = fh.read()
    names_m = re.search(r"kBusJointNames\[kNumBusJoints\]\s*=\s*\{(.*?)\};",
                        src, re.S)
    ports_m = re.search(r"kBusPort\[kNumBusJoints\]\s*=\s*\{(.*?)\};",
                        src, re.S)
    assert names_m, "kBusJointNames not found in bus_map.h"
    assert ports_m, "kBusPort not found in bus_map.h"
    fw_names = re.findall(r'"([^"]+)"', names_m.group(1))
    fw_ports = [p.strip() for p in re.findall(r"'(\w)'", ports_m.group(1))]
    assert list(BUS_NAMES) == fw_names, (
        f"bus_cal.BUS_NAMES diverged from bus_map.h:\n"
        f"python: {list(BUS_NAMES)}\n"
        f"firmware: {fw_names}"
    )
    assert list(BUS_PORTS) == fw_ports, (
        f"bus_cal.BUS_PORTS diverged from bus_map.h:\n"
        f"python: {list(BUS_PORTS)}\n"
        f"firmware: {fw_ports}"
    )


def test_policy_order_matches_legacy_servo_id():
    """POLICY_ORDER + firmware/main/cal_store.h kLegacyServoId must agree."""
    hdr = os.path.join(ROOT, "firmware", "main", "cal_store.h")
    with open(hdr) as fh:
        src = fh.read()
    m = re.search(r"kLegacyServoId\[kLegacyJoints\]\s*=\s*\{(.*?)\};",
                  src, re.S)
    assert m, "kLegacyServoId not found in cal_store.h"
    ids = [int(t) for t in re.findall(r"\d+", m.group(1))]
    assert len(ids) == 10
    for j, name in enumerate(POLICY_ORDER):
        sid_at_j = ids[j]
        b = sid_at_j - 1
        assert BUS_NAMES[b] == name, (
            f"policy joint {j}: kLegacyServoId says servo {sid_at_j}, "
            f"which is bus joint {BUS_NAMES[b]}, not {name}"
        )


def test_asbuilt_prototype_cal_matches_firmware_header():
    """asbuilt_cal.h and bus_cal's fallback table must stay in lockstep."""
    hdr = os.path.join(ROOT, "firmware", "main", "asbuilt_cal.h")
    with open(hdr) as fh:
        src = fh.read()
    zeros_m = re.search(r"kAsBuiltZeroSteps\[10\]\s*=\s*\{(.*?)\};", src, re.S)
    dirs_m = re.search(r"kAsBuiltDir\[10\]\s*=\s*\{(.*?)\};", src, re.S)
    assert zeros_m and dirs_m
    # Strip `//` comments first: asbuilt_cal.h annotates each row with the
    # joint name and (id N), which would otherwise be parsed as values.
    zeros_body = re.sub(r"//[^\n]*", "", zeros_m.group(1))
    dirs_body = re.sub(r"//[^\n]*", "", dirs_m.group(1))
    zeros = [int(t) for t in zeros_body.replace(",", " ").split()]
    dirs = [int(t) for t in re.findall(r"[+-]?\d+", dirs_body)]
    assert len(zeros) == len(dirs) == 10
    cal = asbuilt_prototype_cal()
    for j, name in enumerate(POLICY_ORDER):
        bus_j = cal.by_name(name)
        assert bus_j.zero_steps == zeros[j], f"asbuilt zero {name}"
        assert bus_j.direction == dirs[j], f"asbuilt dir {name}"
        assert bus_j.fitted, f"asbuilt should mark {name} fitted"
    # Everything NOT in the prototype map is NOT FITTED at v3 defaults.
    for j in cal.joints:
        if j.name in POLICY_ORDER:
            continue
        assert not j.fitted, f"asbuilt marks {j.name} NOT FITTED"
        assert j.zero_steps == 2048
        assert j.direction == +1


# -- parser behaviour --------------------------------------------------------

def test_parse_v3_all_fitted_nvs():
    cal = parse_cal_show(V3_ALL_FITTED_NVS)
    assert cal.source == "nvs"
    assert len(cal.joints) == N_BUS
    for j in cal.joints:
        assert j.fitted
    # Spot-check: each row landed on the right bus index.
    assert cal.joints[0].name == "R_hip_roll"
    assert cal.joints[0].servo_id == 1
    assert cal.joints[0].zero_steps == 1010
    assert cal.joints[0].direction == -1
    assert cal.joints[16].name == "L_elbow"
    assert cal.joints[16].servo_id == 17
    assert cal.joints[16].zero_steps == 1170


def test_parse_v2_migrated_defaults():
    cal = parse_cal_show(V2_MIGRATED_DEFAULTS)
    assert cal.source == "nvs"
    # IDs 1..10 fitted, 11..17 not.
    for j in cal.joints[:10]:
        assert j.fitted, j.name
    for j in cal.joints[10:]:
        assert not j.fitted, j.name
    # As-built values land on the right joints. Note the *order* in the
    # output is bus-ID order; legacy tables were policy order.
    assert cal.by_name("L_hip_yaw").zero_steps == 1692
    assert cal.by_name("R_hip_roll").zero_steps == 3532
    assert cal.by_name("R_knee").direction == -1
    assert cal.by_name("L_ankle").direction == +1


def test_parse_refuses_unknown_joint():
    bad = V3_ALL_FITTED_NVS.replace("neck_yaw", "unknown_joint_name")
    # replace will shorten one row to name that doesn't match the regex's
    # (\w+)\s+id pattern cleanly; either way, parse must fail.
    with pytest.raises(BusCalError):
        parse_cal_show(bad)


def test_parse_refuses_permuted_id():
    # Swap R_knee (id 3) onto the R_hip_pitch row.
    bad = V3_ALL_FITTED_NVS.replace(
        "R_hip_pitch  id  2", "R_hip_pitch  id  3"
    )
    with pytest.raises(BusCalError, match="bus map"):
        parse_cal_show(bad)


def test_parse_refuses_bad_direction():
    bad = V3_ALL_FITTED_NVS.replace(
        "R_hip_roll   id  1  zero 1010  dir -1",
        "R_hip_roll   id  1  zero 1010  dir -2"
    )
    with pytest.raises(BusCalError):
        parse_cal_show(bad)


def test_parse_refuses_short_row_count():
    # Drop the L_elbow line entirely.
    bad = V3_ALL_FITTED_NVS.replace(
        "  L_elbow      id 17  zero 1170  dir +1  fitted\n", ""
    )
    with pytest.raises(BusCalError, match="17"):
        parse_cal_show(bad)


def test_parse_refuses_missing_trailer():
    bad = V3_ALL_FITTED_NVS.replace("loaded from NVS at boot\n", "")
    with pytest.raises(BusCalError, match="trailer"):
        parse_cal_show(bad)


def test_parse_defaults_source_when_not_stored():
    text = V3_ALL_FITTED_NVS.replace(
        "loaded from NVS at boot",
        "DEFAULTS -- nothing stored (cal save)"
    )
    cal = parse_cal_show(text)
    assert cal.source == "defaults"


# -- pose-tick assembly -------------------------------------------------------

def test_pose_ticks_all_zero_when_no_offsets():
    cal = parse_cal_show(V3_ALL_FITTED_NVS)
    ticks = cal.pose_ticks()
    assert len(ticks) == 17
    for j, t in zip(cal.joints, ticks):
        assert t == j.zero_steps


def test_pose_ticks_offset_conversion():
    cal = parse_cal_show(V3_ALL_FITTED_NVS)
    # +10 deg on L_knee (dir -1, zero 1070): 1070 - 10*TPD ticks.
    ticks = cal.pose_ticks({"L_knee": 10.0})
    expected = int(round(1070 + (-1) * 10.0 * bus_cal.TICKS_PER_DEG))
    knee_idx = cal.by_name("L_knee").bus_index
    assert ticks[knee_idx] == expected
    # Everything else at its zero.
    for b, t in enumerate(ticks):
        if b != knee_idx:
            assert t == cal.joints[b].zero_steps


def test_pose_ticks_round_trip():
    """ticks_to_deg after deg_to_ticks gives back the requested offset."""
    cal = parse_cal_show(V3_ALL_FITTED_NVS)
    for name in BUS_NAMES:
        j = cal.by_name(name)
        for deg in (-15.0, 0.0, 5.5, 22.0):
            t = j.deg_to_ticks(deg)
            back = j.ticks_to_deg(t)
            assert back == pytest.approx(deg, abs=0.2), (
                f"{name}: deg {deg} -> tick {t} -> deg {back}"
            )


def test_policy_pose_ticks_legacy_shape():
    """policy_pose_ticks returns 10 ticks in the legacy policy order, and
    each value matches what the legacy CAL table would have produced.
    """
    cal = asbuilt_prototype_cal()
    ticks = cal.policy_pose_ticks()
    assert len(ticks) == 10
    # policy order: L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee, L_ankle,
    #               R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle
    assert ticks[0] == 1692  # L_hip_yaw zero
    assert ticks[5] == 1806  # R_hip_yaw zero
    assert ticks[9] == 3437  # R_ankle zero


def test_pose_ticks_only_restricts_non_zero_offsets():
    cal = parse_cal_show(V3_ALL_FITTED_NVS)
    ticks = cal.pose_ticks({"L_knee": -30.0, "R_knee": -30.0},
                           only=["L_knee"])
    # L_knee moved; R_knee held at zero (despite being named).
    assert ticks[cal.by_name("L_knee").bus_index] != cal.by_name("L_knee").zero_steps
    assert ticks[cal.by_name("R_knee").bus_index] == cal.by_name("R_knee").zero_steps


def test_fetch_cal_falls_back_to_asbuilt_when_unparseable():
    bad_reply = "garbage -- no rows here"
    cal, source = bus_cal.fetch_cal(
        cmd=lambda line, until: bad_reply,
        allow_asbuilt_fallback=True,
    )
    assert cal.source == "asbuilt"
    assert "asbuilt" in source
    # The fallback is the prototype calibration.
    assert cal.by_name("L_hip_yaw").zero_steps == 1692


def test_fetch_cal_raises_without_fallback_when_unparseable():
    with pytest.raises(BusCalError):
        bus_cal.fetch_cal(
            cmd=lambda line, until: "garbage",
            allow_asbuilt_fallback=False,
        )
