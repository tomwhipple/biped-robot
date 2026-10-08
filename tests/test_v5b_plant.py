"""experiments/v5b/plant_v5b.py: the v5 body with the v6 ankle chain.

Run:  .venv/bin/python -m pytest tests/test_v5b_plant.py -q

Pins what v5b is: 12 actuated hinges (the v5 ten plus the two ankle rolls), the
roll joints under the ankle-pitch bodies, every actuator the same STS3215 model
as v5body's, a total mass in the band the plant prints, and a zero pose that
stands flat on all 16 sole pads.
"""
import math
import os
import sys

import mujoco
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "experiments", "v5b"))
import plant_v5b as P  # noqa: E402

# plant_v5b prints 1.2975 kg; the band allows a re-rolled inertia, not a
# missing or doubled ankle chain (+-0.2 kg)
MASS_KG = (1.20, 1.40)


@pytest.fixture(scope="module")
def plant():
    m = P.build()
    return m, mujoco.MjData(m)


def test_twelve_actuated_hinges(plant):
    m, _ = plant
    hinges = [m.joint(i).name for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    assert sorted(hinges) == sorted(P.JOINTS) and len(hinges) == 12
    assert m.nu == 12
    assert sorted(m.actuator(i).name for i in range(m.nu)) == sorted(P.JOINTS)
    for i in range(m.nu):
        assert m.actuator_trnid[i, 0] == m.joint(m.actuator(i).name).id


def test_ankle_roll_under_ankle_pitch(plant):
    m, _ = plant
    for s in "LR":
        roll = m.joint(f"{s}_ankle_roll")
        assert m.body(roll.bodyid[0]).name == f"{s}_foot"
        assert m.body_parentid[m.body(f"{s}_foot").id] == m.body(f"{s}_ankle_blk").id
        assert m.joint(f"{s}_ankle").bodyid[0] == m.body(f"{s}_ankle_blk").id
        assert m.body_parentid[m.body(f"{s}_ankle_blk").id] == m.body(f"{s}_shin").id
        assert np.allclose(m.body_pos[m.body(f"{s}_ankle_blk").id], [0, 0, P.ANKLE_Z])
        assert np.allclose(m.jnt_axis[roll.id], [1, 0, 0])
        assert np.allclose(np.degrees(m.jnt_range[roll.id]), [-25, 25])


def test_one_servo_model(plant):
    m, _ = plant
    ref = m.actuator("L_knee").id
    for i in range(m.nu):
        assert np.allclose(m.actuator_gainprm[i, :3], m.actuator_gainprm[ref, :3])
        assert np.allclose(m.actuator_biasprm[i, :3], m.actuator_biasprm[ref, :3])
        assert np.allclose(m.actuator_forcerange[i], [-3, 3])
    for n in P.JOINTS:
        j = m.joint(n).id
        assert m.dof_damping[m.jnt_dofadr[j]] == pytest.approx(0.6)
        assert m.dof_armature[m.jnt_dofadr[j]] == pytest.approx(0.008)


def test_total_mass(plant):
    m, _ = plant
    assert MASS_KG[0] < m.body_subtreemass[0] < MASS_KG[1]


def test_zero_pose_stands_on_soles(plant):
    m, d = plant
    P.set_stand(m, d)
    for _ in range(500):
        d.ctrl[:] = 0
        mujoco.mj_step(m, d)
    pads = [g for g in range(m.ngeom) if m.geom_contype[g] == 1 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
    assert len(pads) == 16
    touching = {g for c in d.contact[:d.ncon] for g in (c.geom1, c.geom2)} & set(pads)
    assert touching == set(pads)
    for s in "LR":
        assert d.xmat[m.body(f"{s}_foot").id].reshape(3, 3)[2, 2] > 0.999
    assert d.xmat[m.body("torso").id].reshape(3, 3)[2, 2] > 0.999
    assert 0.20 < d.subtree_com[0][2] < 0.22
