"""The hanging plant: sim/bimo_biped_v6legs_hang.xml, the test-stand analogue of sil_twin --hang.

Run:  .venv/bin/python -m pytest tests/test_plant_hang.py -q

Pins that the committed file is what the generator writes, that it is the legs-only plant and
nothing else (same joints, actuators, masses), and that the torso stays where it was welded with
the feet off the floor.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, SIM)
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

LEGS = os.path.join(SIM, "bimo_biped_v6legs.xml")
HANG = os.path.join(SIM, "bimo_biped_v6legs_hang.xml")


def test_hang_plant_is_generated():
    import build_v6_inertia as BI
    with open(HANG) as f:
        assert f.read() == BI.committed_hang_xml(), \
            "sim/bimo_biped_v6legs_hang.xml is stale: run make plant " \
            "(sim/build_v6_inertia.py --hang --asbuilt-legs --write)"


def test_hang_plant_is_the_legs_plant():
    a, b = mujoco.MjModel.from_xml_path(LEGS), mujoco.MjModel.from_xml_path(HANG)
    assert (a.nq, a.nv, a.nu, a.ngeom) == (b.nq, b.nv, b.nu, b.ngeom)
    np.testing.assert_array_equal(a.body_mass, b.body_mass)
    np.testing.assert_array_equal(a.jnt_range, b.jnt_range)
    assert [a.actuator(i).name for i in range(a.nu)] == [b.actuator(i).name for i in range(b.nu)]
    assert b.neq == 1 and a.neq == 0


def test_torso_hangs_with_the_feet_free():
    import build_v6_inertia as BI
    m = mujoco.MjModel.from_xml_path(HANG)
    d = mujoco.MjData(m)
    torso = m.body("torso").id
    z0 = float(m.body_pos[torso][2])
    for _ in range(1000):
        mujoco.mj_step(m, d)
    assert abs(float(d.xpos[torso][2]) - z0) < 2e-3
    floor = m.geom("floor").id
    for c in d.contact[:d.ncon]:
        assert floor not in (c.geom1, c.geom2), "a hanging foot touched the floor"
    legs = mujoco.MjModel.from_xml_path(LEGS)
    assert abs(z0 - float(legs.body_pos[legs.body("torso").id][2]) - BI.HANG_RAISE_M) < 1e-4
