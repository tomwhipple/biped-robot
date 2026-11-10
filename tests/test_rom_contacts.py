"""bimo_biped_v6legs.xml: the inter-leg contacts are the whole self-collision
story, and the training engine makes them too.

Run:  .venv/bin/python -m pytest tests/test_rom_contacts.py -q

The legs collide with each other only through declared <pair>s between the
fitted col geoms (thigh and shank capsules, ankle and foot boxes) plus the
sole-on-sole pair; every other body geom has conaffinity 0 and meets only
the floor. Where those contacts close against the CAD is pinned on the
measured plant (tests/test_plant_measured.py, section 4). Here: nothing else
can reach a leg, and MJX (training) agrees with the CPU engine (scoring) on
the contact -- a contact only counts if both engines make it.

Pure kinematics: set qpos, forward, measure. No dynamics, no policy.
"""
import math
import os
import sys

import numpy as np
import pytest

import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))

PLANT = os.path.join(ROOT, "sim", "bimo_biped_v6legs.xml")
COL = ("thigh", "shank", "ankle", "foot")


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(PLANT)


def test_the_legs_are_the_only_self_collision(model):
    """The declared pairs are every L col geom against every R col geom, plus
    the soles; the col geoms are contype 0 / conaffinity 0, so nothing else
    reaches them, and no other body geom accepts a contact (conaffinity 0:
    each meets the floor and nothing else)."""
    gid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
    want = {tuple(sorted((gid(f"L_col_{a}"), gid(f"R_col_{b}"))))
            for a in COL for b in COL}
    want.add(tuple(sorted((gid("L_sole"), gid("R_sole")))))
    declared = {tuple(sorted((int(model.pair_geom1[p]), int(model.pair_geom2[p]))))
                for p in range(model.npair)}
    assert model.npair == len(want) == 17
    assert declared == want
    for s in "LR":
        for k in COL:
            g = gid(f"{s}_col_{k}")
            assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0
    floor = gid("floor")
    others = [g for g in range(model.ngeom) if g != floor]
    assert not model.geom_conaffinity[others].any()


def test_mjx_agrees_with_the_cpu_referee_on_the_contact():
    """Training runs on MJX and scoring on the CPU engine. One hip rolled
    inward with the other leg at zero closes the feet (box-box) first; with
    the other leg pitched -70 it closes the thighs (capsule-capsule). Both
    engines must put the onset at the same angle and measure the same
    penetration."""
    jax = pytest.importorskip("jax")
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    jax.config.update("jax_enable_x64", True)
    from mujoco import mjx

    m = mujoco.MjModel.from_xml_path(PLANT)
    mx = mjx.put_model(m)
    qadr = {m.joint(j).name: m.jnt_qposadr[j] for j in range(m.njnt)}
    d = mujoco.MjData(m)

    def both(pair, pose):
        ga, gb = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in pair)
        q = np.array(m.qpos0)
        for k, v in pose.items():
            q[qadr[k]] = math.radians(v)
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        cpu = mujoco.mj_geomDistance(m, d, ga, gb, 0.5, None)
        dx = mjx.forward(mx, mjx.make_data(mx).replace(qpos=jax.numpy.asarray(q)))
        c = dx._impl.contact
        sel = [float(c.dist[i]) for i in range(c.dist.shape[0])
               if {int(c.geom1[i]), int(c.geom2[i])} == {ga, gb}]
        assert sel, f"MJX lost the {pair} pair at {pose}"
        return cpu, min(sel)

    # feet: apart at 4 and 6 deg, touching from ~6.3
    for roll in (4.0, 6.0, 6.5, 8.0, 10.0):
        cpu, gx = both(("L_col_foot", "R_col_foot"), {"R_hip_roll": roll})
        assert (cpu < 0) == (roll > 6.3), roll
        assert (gx < 0) == (cpu < 0), roll
        if cpu < 0:
            assert gx == pytest.approx(cpu, abs=1e-6), roll
    # thighs, the other leg forward: the capsule distance itself, either side of contact
    for roll in (20.0, 31.0, 35.0):
        cpu, gx = both(("L_col_thigh", "R_col_thigh"),
                       {"R_hip_roll": roll, "L_hip_pitch": -70})
        assert gx == pytest.approx(cpu, abs=1e-6), roll
        assert (cpu < 0) == (roll > 33.0), roll
