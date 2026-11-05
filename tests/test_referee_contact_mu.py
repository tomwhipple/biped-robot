"""The referee's --contact-mu / --contact-torsion override (eval_precision.set_contact_friction):
the floor, every sole pad and the soles carry the graded friction, and it survives the
per-reset domain randomisation (which re-draws the floor friction from its nominal)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))


def test_contact_mu_override_survives_dr_reset():
    import eval_precision as E
    from walker_env import BimoWalkerEnv
    xml = os.path.join(HERE, "..", "sim", "bimo_biped_v6legs_meas.xml")
    env = BimoWalkerEnv(xml_path=xml, domain_rand=True)
    E.set_contact_friction(env, 0.4, 0.003)
    pads = [g for side in env._pad_gids for g in side]
    assert len(pads) >= 2
    for _ in range(3):
        env.reset(seed=_)
        f = env.model.geom_friction
        for g in pads + list(env._sole_gids):
            assert abs(f[g, 0] - 0.4) < 1e-12 and abs(f[g, 1] - 0.003) < 1e-12
        # the floor is drawn around the new nominal: the pair (max) is >= 0.4, never the old 1.0
        lo, hi = 0.4 * (1 - env.friction_range), 0.4 * (1 + env.friction_range)
        assert lo - 1e-9 <= f[env._floor_gid, 0] <= hi + 1e-9
        assert max(f[env._floor_gid, 0], f[pads[0], 0]) < 1.0


def test_contact_mu_none_is_a_no_op():
    import eval_precision as E
    from walker_env import BimoWalkerEnv
    xml = os.path.join(HERE, "..", "sim", "bimo_biped_v6legs_meas.xml")
    env = BimoWalkerEnv(xml_path=xml)
    before = env.model.geom_friction.copy()
    E.set_contact_friction(env, None, None)
    assert (env.model.geom_friction == before).all()
