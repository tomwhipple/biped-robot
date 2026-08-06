"""Every joint-limit table in the repo must agree with sim/joint_rom.py.

This is the test that would have caught the 2026-08-02 calibration only being
half-applied: the bench measurements reached mech_envelope.h and CAD, the
MJCF's ctrlrange never got them, and obs_spec.h -- generated from that
ctrlrange -- inherited the stale numbers. Nothing was red for four days.
"""
import os
import re
import sys

import mujoco
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "cad"))

import joint_rom  # noqa: E402

PLANT = os.path.join(ROOT, "sim", "bimo_biped_v5body.xml")
MECH_H = os.path.join(ROOT, "firmware", "main", "mech_envelope.h")
OBS_H = os.path.join(ROOT, "firmware", "components", "obs", "include",
                     "obs", "obs_spec.h")

TOL = 0.05   # degrees; the headers carry 9 significant figures of radians


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(PLANT)


def _c_float_array(path, name):
    """Pull `inline constexpr float NAME[...] = {...};` out of a header."""
    with open(path) as fh:
        src = fh.read()
    m = re.search(rf"{name}\[[^\]]*\]\s*=\s*\{{(.*?)\}};", src, re.S)
    assert m, f"{name} not found in {os.path.basename(path)}"
    # Strip // comments FIRST: mech_envelope.h's annotations contain commas
    # ("// L_knee flexion, measured"), so splitting on comma first tears them.
    body = re.sub(r"//[^\n]*", "", m.group(1))
    vals = []
    for tok in body.split(","):
        tok = tok.strip()
        if not tok:
            continue
        # mech_envelope.h writes "-95.0f * kDeg" (already degrees);
        # obs_spec.h writes bare radians like "-1.658062789f".
        mul = re.match(r"(-?[\d.]+)f?\s*\*\s*kDeg\s*$", tok)
        vals.append(float(mul.group(1)) if mul
                    else np.degrees(float(tok.rstrip("f"))))
    return vals


def test_mjcf_joint_stops_match_rom(model):
    """The MJCF's mechanical stops are the measured envelope."""
    for name, (lo, hi) in joint_rom.all_sim_ranges().items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert j >= 0, f"{name} missing from the plant"
        got = np.degrees(model.jnt_range[j])
        assert got[0] == pytest.approx(lo, abs=TOL), f"{name} lo"
        assert got[1] == pytest.approx(hi, abs=TOL), f"{name} hi"


def test_mjcf_policy_range_matches_rom(model):
    """ctrlrange -- what a policy may COMMAND -- is the measured envelope too.

    This is the assertion that was missing. Widening a stop while leaving the
    ctrlrange behind is invisible in every render and every scorecard.
    """
    for name, (lo, hi) in joint_rom.all_sim_ranges().items():
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert a >= 0, f"{name} has no actuator"
        got = np.degrees(model.actuator_ctrlrange[a])
        assert got[0] == pytest.approx(lo, abs=TOL), f"{name} ctrlrange lo"
        assert got[1] == pytest.approx(hi, abs=TOL), f"{name} ctrlrange hi"


def test_firmware_mech_envelope_matches_rom():
    lo = _c_float_array(MECH_H, "kMechLo")
    hi = _c_float_array(MECH_H, "kMechHi")
    want = joint_rom.all_sim_ranges()
    order = list(want)
    assert len(lo) == len(hi) == len(order)
    for i, name in enumerate(order):
        assert lo[i] == pytest.approx(want[name][0], abs=TOL), f"kMechLo {name}"
        assert hi[i] == pytest.approx(want[name][1], abs=TOL), f"kMechHi {name}"


def test_firmware_policy_range_matches_rom():
    """obs_spec.h is generated from the MJCF, so this pins the generator too."""
    lo = _c_float_array(OBS_H, "kJointLo")
    hi = _c_float_array(OBS_H, "kJointHi")
    want = joint_rom.all_sim_ranges()
    order = list(want)
    for i, name in enumerate(order):
        assert lo[i] == pytest.approx(want[name][0], abs=TOL), f"kJointLo {name}"
        assert hi[i] == pytest.approx(want[name][1], abs=TOL), f"kJointHi {name}"


def test_cad_rom_matches():
    """cad/dimensions.py drives the clearance sweeps; it must not lag either."""
    import dimensions as D
    for joint, (lo, hi) in joint_rom.ROM.items():
        cad_lo, cad_hi = D.ROM[joint]
        if joint in joint_rom.FLIPPED:
            # CAD states the PHYSICAL rotation, same as joint_rom.ROM
            pass
        assert cad_lo == pytest.approx(lo, abs=TOL), f"CAD ROM {joint} lo"
        assert cad_hi == pytest.approx(hi, abs=TOL), f"CAD ROM {joint} hi"
    assert D.ROM["hip_roll"] == pytest.approx(
        (-joint_rom.HIP_ROLL_ABDUCT, joint_rom.HIP_ROLL_ABDUCT), abs=TOL), \
        "CAD states hip_roll as the union of both legs' travel"
