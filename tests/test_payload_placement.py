"""The simulated camera must sit ON its mount, where the CAD assembly puts it.

cad/export_assembly.py places the camera at Pos(GP_MOUNT_X, 0, DECK_TOP_Z);
the sim built the payload body at a hard-coded x=0 until 2026-08-06, which
floated 154 g -- 12 % of the robot -- 24 mm forward of the gopro_base it is
bolted to. It was visible in every render for as long as it existed and
nothing failed, because no test related the two.
"""
import os
import sys

import mujoco
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "sim", "mjx"))
sys.path.insert(0, os.path.join(ROOT, "cad"))

import dimensions as D  # noqa: E402
import env_mjx  # noqa: E402

PLANT = os.path.join(ROOT, "sim", "bimo_biped_v5body.xml")

# what train_mjx.py pins for the v6 body
PAYLOAD_CG_X = -0.0240
PAYLOAD_CG_Z = 0.0601


@pytest.fixture(scope="module")
def model():
    return env_mjx._prep_model(
        PLANT, payload=True, servo_joint_damping=0.0,
        payload_cg_z=PAYLOAD_CG_Z, payload_cg_x=PAYLOAD_CG_X)


def _geom_world(model, name):
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert g >= 0, f"geom {name} missing"
    return d.geom_xpos[g].copy()


def _geom_in_torso(model, name):
    """geom_xpos is WORLD; every CAD constant here is torso-relative."""
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert g >= 0, f"geom {name} missing"
    t = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    return d.geom_xpos[g] - d.xpos[t]


def test_payload_is_centred_over_its_mount(model):
    """Camera x == gopro_base x. This is the assertion that was missing."""
    cam = _geom_world(model, "payload")
    mount = _geom_world(model, "gopro_base") if mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "gopro_base") >= 0 else None
    if mount is None:
        # the mesh geom is unnamed in the plant; fall back to the CAD constant
        assert cam[0] == pytest.approx(D.GP_MOUNT_X / 1000.0, abs=1e-3), \
            "camera is not at GP_MOUNT_X"
    else:
        assert cam[0] == pytest.approx(mount[0], abs=2e-3), \
            "camera is not centred over gopro_base"


def test_payload_x_matches_cad_mount():
    assert PAYLOAD_CG_X == pytest.approx(D.GP_MOUNT_X / 1000.0, abs=1e-4)


def test_payload_z_matches_cad_camera_mock():
    """z must follow check_assembly.camera_mock()'s CORRECTED body bottom.

    parts.py carried the tower-era `hole_z + 6.0` until 2026-08-06 and read
    2 mm low; pin the corrected derivation so the two cannot diverge again.
    """
    hole_z = D.GP_BASE_T + D.GP_HOLE_H
    bot = hole_z + D.GP_PRONG_OD / 2 + 0.5
    cam_cg_over_deck = bot + D.CAM_BODY[2] / 2
    deck_over_torso = (D.DECK_BOT_Z + D.DECK_T) - D.TORSO_CENTER_Z
    want = (deck_over_torso + cam_cg_over_deck) / 1000.0
    assert PAYLOAD_CG_Z == pytest.approx(want, abs=1e-4), \
        f"payload_cg_z should be {want:.4f} m"


def test_payload_does_not_float_above_the_mount(model):
    """Its underside should meet the mount's prongs, not hover over them."""
    cam_bot = _geom_in_torso(model, "payload")[2] - D.CAM_BODY[2] / 2000.0
    # gopro_base top = deck top + the base mesh's own height (21 mm)
    deck_over_torso = ((D.DECK_BOT_Z + D.DECK_T) - D.TORSO_CENTER_Z) / 1000.0
    base_top = deck_over_torso + 0.021
    gap = cam_bot - base_top
    assert -0.005 < gap < 0.010, (
        f"camera underside sits {gap*1000:.1f} mm from the mount top; "
        "it should rest on the prongs")
