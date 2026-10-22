"""Joint-friction DR (get-up campaign, 2026-10-22).

robot_getup_t3k, trained at joint frictionloss 0.35, gets up from prone 8/8 at
0.35 but 6/8 at 0.05, and from supine 2/8 at 0.05: a policy trained at one
friction fails at another, and the true gearbox friction is not measured
(0.05 BAM, ~0.235 powered, 0.35 assumed backdrive). --joint-frictionloss-range
draws one value per env for every servo DOF. Off by default = the batched model
is unchanged; on, every env's servo DOFs share one draw inside the range and
no other DOF moves.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import jax  # noqa: E402

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v5body.xml")


def _batched(**kw):
    from env_mjx import BimoMJXEnv, domain_randomize
    e = BimoMJXEnv(xml_path=XML, ext_cmd=True, domain_rand=True,
                   joint_frictionloss=0.35, **kw)
    m, axes = domain_randomize(
        e.model, jax.random.split(jax.random.PRNGKey(0), 64),
        mass_range=e.mass_range, friction_range=e.friction_range,
        floor_gid=e._floor_gid, nom_mass=e._nom_mass,
        nom_inertia=e._nom_inertia, nom_friction=e._nom_friction,
        joint_frictionloss_range=e.joint_frictionloss_range,
        nom_frictionloss=e._nom_frictionloss, jv=(e._jv0, e._jv1))
    return e, m, axes


def test_off_by_default():
    e, m, axes = _batched()
    assert e.joint_frictionloss_range is None
    assert axes.dof_frictionloss is None
    np.testing.assert_array_equal(np.asarray(m.dof_frictionloss),
                                  np.asarray(e.model.dof_frictionloss))


def test_one_draw_per_env_on_the_servo_dofs():
    e, m, axes = _batched(joint_frictionloss_range=(0.05, 0.5))
    assert axes.dof_frictionloss == 0
    fl = np.asarray(m.dof_frictionloss)
    nom = np.asarray(e.model.dof_frictionloss)
    assert fl.shape == (64, nom.shape[0])
    servo = fl[:, e._jv0:e._jv1]
    assert np.all(servo >= 0.05) and np.all(servo <= 0.5)
    assert np.allclose(servo, servo[:, :1])            # common-mode: one value per env
    assert servo[:, 0].min() < 0.15 and servo[:, 0].max() > 0.4   # spans the range
    rest = np.ones(nom.shape[0], bool)
    rest[e._jv0:e._jv1] = False
    np.testing.assert_array_equal(fl[:, rest], np.broadcast_to(nom[rest], fl[:, rest].shape))
