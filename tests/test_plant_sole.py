"""The measured-sole plant: sim/bimo_biped_v6legs_meas_sole.xml (2026-11-04).

Run:  .venv/bin/python -m pytest tests/test_plant_sole.py -q

On 2026-11-04 the sole friction was measured on the robot (one foot clamped, the other at
half weight, ~7.5 N): the free foot ROTATES under hip yaw with no measurable grip (<= ~0.1-0.15
N m, both feet) and slides at mu ~0.45-0.55. The edge-pad plants grip ~0.66 N m (Coulomb; ~0.34
N m effective in MuJoCo's soft contact at the bench's 10 deg/s). The _sole plant is
v6legs_meas with gen_plant_v6.measured_sole(): mu 0.5, torsional 0, pad priority 1, "flat" pads.
docs/design-v6/2026-11-04-sole-friction.md; sim/sole_friction.py reproduces the bench.

What this file pins, and why each one can drift silently:

  1. GENERATED: the committed file is what the generator writes, and the other plants did not
     move (the option is off by default).
  2. ONE VARIABLE: the _sole plant differs from v6legs_meas only in the pads (positions,
     friction, priority): same bodies, masses, joints, actuators, geom count.
  3. THE PADS: 8 per foot, same height and size, mirrored L/R, the 4 outer ones on the sole
     outline, mean distance from the sole centre 0.042 m.
  4. PRIORITY: the pad's friction IS the foot contact's, on CPU MuJoCo and on MJX, although the
     floor stays 1.0 (no MAX-combine); every other contact with the floor is unchanged.
  5. THE BENCH: the sim yaw test puts the _sole plant at the bench's yaw grip and v6legs_meas
     well above it.
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
for _p in (SIM, os.path.join(SIM, "mjx")):
    sys.path.insert(0, _p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

MEAS = os.path.join(SIM, "bimo_biped_v6legs_meas.xml")
SOLE = os.path.join(SIM, "bimo_biped_v6legs_meas_sole.xml")


def _m(path):
    return mujoco.MjModel.from_xml_path(path)


def _pads(m, side):
    return [g for g in range(m.ngeom) if (m.geom(g).name or "").startswith(f"{side}_pad_")]


def _stand(m):
    """settled on both feet, position actuators at home"""
    d = mujoco.MjData(m)
    for _ in range(500):
        mujoco.mj_step(m, d)
    return d


# -- 1. generated, and nothing else moved ---------------------------------------------
def test_sole_plant_is_generated():
    import build_v6_inertia as BI
    with open(SOLE) as f:
        assert f.read() == BI.committed_meas_sole_xml(), \
            "sim/bimo_biped_v6legs_meas_sole.xml is stale: run make plant " \
            "(sim/build_v6_inertia.py --measured-stops --measured-sole --asbuilt-legs --write)"


def test_option_off_by_default():
    from gen_plant_v6 import DesignParams, measured_sole
    p = DesignParams()
    assert (p.sole_pads, p.pad_friction, p.pad_priority) == ("edge", (1.0, 0.02, 0.001), 0)
    q = measured_sole(p)
    assert (q.sole_pads, q.pad_friction, q.pad_priority) == ("flat", (0.5, 0.0, 0.001), 1)
    assert q.sole_pad_r_mean == pytest.approx(0.042)
    m = _m(MEAS)
    for g in _pads(m, "L") + _pads(m, "R"):
        np.testing.assert_allclose(m.geom_friction[g], [1.0, 0.02, 0.001])
        assert m.geom_priority[g] == 0


# -- 2. one variable --------------------------------------------------------------------
def test_only_the_pads_differ():
    a, b = _m(MEAS), _m(SOLE)
    assert (a.nbody, a.njnt, a.nu, a.ngeom, a.nq) == (b.nbody, b.njnt, b.nu, b.ngeom, b.nq)
    np.testing.assert_array_equal(a.body_mass, b.body_mass)
    np.testing.assert_array_equal(a.body_inertia, b.body_inertia)
    np.testing.assert_array_equal(a.body_ipos, b.body_ipos)
    np.testing.assert_array_equal(a.jnt_range, b.jnt_range)
    np.testing.assert_array_equal(a.actuator_ctrlrange, b.actuator_ctrlrange)
    np.testing.assert_array_equal(a.dof_frictionloss, b.dof_frictionloss)
    pads = set(_pads(a, "L") + _pads(a, "R"))
    assert pads == set(_pads(b, "L") + _pads(b, "R"))
    rest = [g for g in range(a.ngeom) if g not in pads]
    for f in ("geom_pos", "geom_size", "geom_friction", "geom_priority", "geom_contype",
              "geom_conaffinity", "geom_condim", "geom_solref"):
        np.testing.assert_array_equal(getattr(a, f)[rest], getattr(b, f)[rest], err_msg=f)
    assert b.geom("floor").friction.tolist() == pytest.approx([1.0, 0.02, 0.001])


# -- 3. the pads --------------------------------------------------------------------------
def test_flat_pads():
    from gen_plant_v6 import DesignParams
    p = DesignParams()
    a, b = _m(MEAS), _m(SOLE)
    for side in "LR":
        pa, pb = _pads(a, side), _pads(b, side)
        assert len(pb) == 8
        np.testing.assert_array_equal(b.geom_size[pb], a.geom_size[pa])
        np.testing.assert_allclose(b.geom_pos[pb, 2], a.geom_pos[pa, 2])
        sole = b.geom(f"{side}_sole")
        c = sole.pos[:2]                                         # the sole centre, foot frame
        xy = b.geom_pos[pb, :2] - c
        r = np.hypot(xy[:, 0], xy[:, 1])
        assert r.mean() == pytest.approx(0.042, abs=1e-4)
        hx, hy = p.foot_len / 2, p.foot_w / 2
        assert np.all(np.abs(xy[:, 0]) <= hx + 1e-9) and np.all(np.abs(xy[:, 1]) <= hy + 1e-9)
        # the outer four sit on the rounded-rect outline: the 45 deg point of each corner arc
        out = np.argsort(r)[4:]
        cr = p.foot_r
        for x, y in xy[out]:
            cx, cy = math.copysign(hx - cr, x), math.copysign(hy - cr, y)
            assert math.hypot(x - cx, y - cy) == pytest.approx(cr, abs=1e-4)
        np.testing.assert_allclose(b.geom_friction[pb], np.tile([0.5, 0.0, 0.001], (8, 1)))
        assert np.all(b.geom_priority[pb] == 1)
    # mirrored: R's pads are L's with y negated (in each foot frame)
    L = sorted(map(tuple, np.round(b.geom_pos[_pads(b, "L")] * [1, -1, 1], 6)))
    R = sorted(map(tuple, np.round(b.geom_pos[_pads(b, "R")], 6)))
    assert L == R


# -- 4. priority: the pad's friction is the contact's -----------------------------------
def test_cpu_foot_contacts_use_the_pad_friction():
    m = _m(SOLE)
    d = _stand(m)
    floor = m.geom("floor").id
    pads = set(_pads(m, "L") + _pads(m, "R"))
    n = 0
    for i in range(d.ncon):
        c = d.contact[i]
        if floor in (c.geom1, c.geom2) and (c.geom1 in pads or c.geom2 in pads):
            np.testing.assert_allclose(c.friction, [0.5, 0.5, 0.0, 0.001, 0.001], atol=1e-5)
            n += 1
    assert n >= 12                                    # (a pad or two may sit just clear)
    # the old plant max-combines against the floor's 1.0 / 0.02
    m0 = _m(MEAS)
    d0 = _stand(m0)
    c = [d0.contact[i] for i in range(d0.ncon)][0]
    np.testing.assert_allclose(c.friction[:3], [1.0, 1.0, 0.02])


def test_other_floor_contacts_unchanged():
    """a fallcol geom (0.8) on the floor (1.0) still max-combines to 1.0 on the _sole plant"""
    m = _m(SOLE)
    d = mujoco.MjData(m)
    d.qpos[2] = 0.05                                  # torso near the floor: the body meets it
    d.qpos[3:7] = [math.sqrt(0.5), 0, math.sqrt(0.5), 0]   # lying on its back/front
    mujoco.mj_forward(m, d)
    floor = m.geom("floor").id
    pads = set(_pads(m, "L") + _pads(m, "R"))
    others = [d.contact[i] for i in range(d.ncon)
              if floor in (d.contact[i].geom1, d.contact[i].geom2)
              and not (d.contact[i].geom1 in pads or d.contact[i].geom2 in pads)]
    assert others, "no body-floor contact in the lying pose"
    for c in others:
        assert c.friction[0] == pytest.approx(1.0)


def test_mjx_foot_contacts_use_the_pad_friction():
    import jax
    from mujoco import mjx
    m = _m(SOLE)
    d = _stand(m)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.forward)(mx, mjx.put_data(m, d))
    con = dx._impl.contact if hasattr(dx, "_impl") and hasattr(dx._impl, "contact") else dx.contact
    g = np.asarray(con.geom)
    fr = np.asarray(con.friction)
    dist = np.asarray(con.dist)
    floor = m.geom("floor").id
    pads = set(_pads(m, "L") + _pads(m, "R"))
    rows = [i for i in range(len(g)) if floor in g[i] and (g[i][0] in pads or g[i][1] in pads)
            and dist[i] < 1e-3]
    assert len(rows) >= 8
    np.testing.assert_allclose(fr[rows][:, :3], np.tile([0.5, 0.5, 0.0], (len(rows), 1)), atol=1e-5)


# -- 5. the bench ----------------------------------------------------------------------
def test_bench_yaw_grip():
    """the bench (<= ~0.10-0.15 N m at ~7.5 N) against sim/sole_friction.py's yaw test, R clamped"""
    import sole_friction as SF
    new = SF.run(SOLE)
    old = SF.run(MEAS)
    assert 7.0 < new["n_clamped"] < 8.0                          # half the weight, like the bench
    assert 0.10 <= new["yaw_tau_7p5"] <= 0.18
    assert old["yaw_tau_7p5"] > 2.0 * 0.15                      # the edge pads: > 2x the bench
    assert new["yaw_track_deg"] < 1.0                            # the foot rotates with the joint


# -- the training and referee envs ----------------------------------------------------------
def test_mjx_dr_on_the_sole_plant():
    """--contact-mu draws the foot contact's mu on the _sole plant too (it sets floor + pads);
    without it the floor-only friction_range draw no longer reaches the foot contact at all
    (priority), so the pads stay at the measured 0.5 / 0."""
    import jax
    from env_mjx import BimoMJXEnv, domain_randomize

    def batched(**kw):
        e = BimoMJXEnv(xml_path=SOLE, ext_cmd=True, domain_rand=True, **kw)
        m, _ = domain_randomize(
            e.model, jax.random.split(jax.random.PRNGKey(0), 64),
            mass_range=e.mass_range, friction_range=e.friction_range,
            floor_gid=e._floor_gid, nom_mass=e._nom_mass,
            nom_inertia=e._nom_inertia, nom_friction=e._nom_friction,
            nom_frictionloss=e._nom_frictionloss, jv=(e._jv0, e._jv1),
            contact_gids=e._contact_gids, nom_solref=e._nom_solref,
            contact_mu_range=e.contact_mu_range, contact_torsion_range=e.contact_torsion_range)
        pads = [g for g in e._contact_gids if g != e._floor_gid]
        return e, np.asarray(m.geom_friction), pads

    e, fr, pads = batched()
    np.testing.assert_allclose(fr[:, pads, 0], 0.5)
    np.testing.assert_allclose(fr[:, pads, 1], 0.0)
    assert fr[:, e._floor_gid, 0].min() < 0.8                   # the floor draw still runs
    e, fr, pads = batched(contact_mu_range=(0.3, 0.7))
    mu = fr[:, pads, 0]
    assert np.allclose(mu, mu[:, :1]) and mu.min() >= 0.3 - 1e-6 and mu.max() <= 0.7 + 1e-6
    assert mu[:, 0].min() < 0.35 and mu[:, 0].max() > 0.65
    np.testing.assert_allclose(fr[:, pads, 1], 0.0)             # no --contact-torsion: none


def test_referee_env_stands_on_the_sole_plant():
    import robot_plant
    import walker_env as W
    kw = dict(robot_plant.robot_env_kwargs(SOLE))
    kw.update(xml_path=SOLE, action_map="full", domain_rand=False, push_prob=0.0, ext_cmd=True)
    env = W.BimoWalkerEnv(actuator_model="sts3215", command_mode=True, **kw)
    env.reset(seed=0)
    n = env.action_size if hasattr(env, "action_size") else env.model.nu
    for _ in range(100):                                         # 2 s at home
        out = env.step(np.zeros(n, np.float32))
        done = out[2] if len(out) > 2 else False
        assert not done
    torso = env.model.body("torso").id
    assert env.data.xpos[torso][2] > 0.3
