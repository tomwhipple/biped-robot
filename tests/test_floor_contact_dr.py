"""Foot/floor contact DR (floors fit, 2026-11-01).

The bench roll balance holds on carpet and goes into a divergent ~1.9 Hz roll oscillation on a
hard floor (experiments/one-leg-stand/2026-11-01-floors-fit.md): the floor is part of the plant.
--floor-contact k_lo,k_hi,b_lo,b_hi draws one contact stiffness and damping per env (log-uniform)
and sets solref = (-k, -b) on the floor and every sole pad. Off by default = the batched model is
unchanged; on, only the floor and pad geoms move, each env's within the range and shared by them.
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

XML = os.path.join(HERE, "..", "sim", "bimo_biped_v6legs.xml")
RANGE = ((300.0, 3.0e5), (10.0, 1000.0))


def _batched(**kw):
    from env_mjx import BimoMJXEnv, domain_randomize
    e = BimoMJXEnv(xml_path=XML, ext_cmd=True, domain_rand=True, **kw)
    m, axes = domain_randomize(
        e.model, jax.random.split(jax.random.PRNGKey(0), 64),
        mass_range=e.mass_range, friction_range=e.friction_range,
        floor_gid=e._floor_gid, nom_mass=e._nom_mass,
        nom_inertia=e._nom_inertia, nom_friction=e._nom_friction,
        nom_frictionloss=e._nom_frictionloss, jv=(e._jv0, e._jv1),
        floor_contact_range=e.floor_contact_range, contact_gids=e._contact_gids,
        nom_solref=e._nom_solref)
    return e, m, axes


def test_off_by_default():
    e, m, axes = _batched()
    assert e.floor_contact_range is None
    assert axes.geom_solref is None
    np.testing.assert_array_equal(np.asarray(m.geom_solref), np.asarray(e.model.geom_solref))


def test_contact_geoms_are_the_floor_and_the_pads():
    e, _, _ = _batched()
    names = [e.mj_model.geom(int(g)).name for g in e._contact_gids]
    assert "floor" in names
    assert sum("_pad_" in n for n in names) == 16          # 8 per foot
    assert len(names) == 17


def test_one_draw_per_env_on_the_contact_geoms():
    e, m, axes = _batched(floor_contact_range=RANGE)
    assert axes.geom_solref == 0
    sr = np.asarray(m.geom_solref)
    nom = np.asarray(e.model.geom_solref)
    g = e._contact_gids
    k, b = -sr[:, g, 0], -sr[:, g, 1]
    assert np.allclose(k, k[:, :1]) and np.allclose(b, b[:, :1])     # one value per env, all contact geoms
    (k_lo, k_hi), (b_lo, b_hi) = RANGE
    assert np.all(k >= k_lo * 0.999) and np.all(k <= k_hi * 1.001)
    assert np.all(b >= b_lo * 0.999) and np.all(b <= b_hi * 1.001)
    lk = np.log10(k[:, 0])
    assert lk.min() < 3.5 and lk.max() > 4.5                          # log-uniform: spans the decades
    rest = np.ones(nom.shape[0], bool)
    rest[g] = False
    np.testing.assert_array_equal(sr[:, rest], np.broadcast_to(nom[rest], sr[:, rest].shape))


def test_cli_flag_parses_into_env_kw():
    import train_mjx
    a = train_mjx.make_parser().parse_args(["--robot", "--floor-contact", "300,3e5,10,1000"])
    kw = train_mjx.build_env_kw(a)
    assert kw["floor_contact_range"] == ((300.0, 3.0e5), (10.0, 1000.0))
