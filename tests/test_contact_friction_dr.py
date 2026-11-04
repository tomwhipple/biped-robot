"""Foot/floor contact FRICTION DR (2026-11-03, experiments/policy/friction-dr).

MuJoCo combines a contact's friction as the elementwise MAX of its two geoms, so the floor-only
friction_range draw (0.6..1.4 x the floor's 1.0) never takes a foot contact below the pads' 1.0.
--contact-mu lo,hi / --contact-torsion lo,hi draw ONE effective sliding / torsional friction per
env and set it on the floor and every sole pad; nothing else moves. Off by default.
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

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v6legs_meas.xml")
MU = (0.35, 1.1)
TOR = (0.001, 0.02)


def _batched(**kw):
    from env_mjx import BimoMJXEnv, domain_randomize
    e = BimoMJXEnv(xml_path=XML, ext_cmd=True, domain_rand=True, **kw)
    m, axes = domain_randomize(
        e.model, jax.random.split(jax.random.PRNGKey(0), 256),
        mass_range=e.mass_range, friction_range=e.friction_range,
        floor_gid=e._floor_gid, nom_mass=e._nom_mass,
        nom_inertia=e._nom_inertia, nom_friction=e._nom_friction,
        nom_frictionloss=e._nom_frictionloss, jv=(e._jv0, e._jv1),
        contact_gids=e._contact_gids, nom_solref=e._nom_solref,
        contact_mu_range=e.contact_mu_range, contact_torsion_range=e.contact_torsion_range)
    return e, m, axes


def _effective(fr, e):
    """per env: the max-combined friction of each floor-pad pair, (n_env, n_pad, 3)"""
    pads = [g for g in e._contact_gids if g != e._floor_gid]
    return np.maximum(fr[:, pads, :], fr[:, e._floor_gid:e._floor_gid + 1, :])


def test_off_by_default_effective_mu_never_below_the_pads():
    e, m, _ = _batched()
    assert e.contact_mu_range is None and e.contact_torsion_range is None
    eff = _effective(np.asarray(m.geom_friction), e)
    # the gap this DR closes: the floor draw alone spans 0.6..1.4, the pair never < 1.0
    assert eff[..., 0].min() >= 1.0 - 1e-6
    assert np.allclose(eff[..., 1], 0.02)


def test_one_draw_per_env_reaches_low_friction():
    e, m, axes = _batched(contact_mu_range=MU, contact_torsion_range=TOR)
    assert axes.geom_friction == 0
    fr = np.asarray(m.geom_friction)
    g = e._contact_gids
    mu, tq = fr[:, g, 0], fr[:, g, 1]
    assert np.allclose(mu, mu[:, :1]) and np.allclose(tq, tq[:, :1])   # one value per env
    assert mu.min() >= MU[0] - 1e-6 and mu.max() <= MU[1] + 1e-6
    assert mu[:, 0].min() < 0.45 and mu[:, 0].max() > 1.0                # spans the range
    assert tq.min() >= TOR[0] - 1e-7 and tq.max() <= TOR[1] + 1e-7
    eff = _effective(fr, e)
    np.testing.assert_allclose(eff[..., 0], mu[:, 1:])                   # the pair sees the draw
    rest = np.ones(fr.shape[1], bool)
    rest[g] = False
    nom = np.asarray(e.model.geom_friction)
    np.testing.assert_array_equal(fr[:, rest], np.broadcast_to(nom[rest], fr[:, rest].shape))
    assert not np.allclose(mu[:, 0], tq[:, 0])                            # independent draws


def test_cli_flags_parse_into_env_kw():
    import train_mjx
    a = train_mjx.make_parser().parse_args(
        ["--robot", "--contact-mu", "0.35,1.1", "--contact-torsion", "0.001,0.02"])
    kw = train_mjx.build_env_kw(a)
    assert kw["contact_mu_range"] == (0.35, 1.1)
    assert kw["contact_torsion_range"] == (0.001, 0.02)
