"""recover_supine_ph: the supine start at the policy's on-the-back phase
(get-up campaign, 2026-10-22).

A recorded get-up is indexed by time from the episode start, so a policy
trained on the prone reference replays the roll from a supine start. The
prone reference's roll ends on the back and the supine seat push follows, so
the supine start sits ~11.3 s into it; recover_supine_ph starts the policy
there. These pin the phase lookup and that the scenario's setup applies it.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")


def test_back_phase_of_each_reference():
    import eval_precision as ep
    assert ep.ref_back_phase("getup_ref_v6ar.npz") == 0.0
    assert ep.ref_back_phase("getup_ref_v6ar_ik.npz") == 0.0
    assert ep.ref_back_phase(None) == 0.0
    t = ep.ref_back_phase("getup_ref_v6ar_prone_ik.npz")
    assert 10.0 < t < 13.0


def test_prone_reference_at_the_phase_matches_the_supine_start():
    """The phase aligns the two seat pushes' "sit up" keyframes. There the
    prone reference is finishing the roll's straighten (every leg joint within
    15 deg of the supine start), and 0.75 s later it lies exactly as the supine
    start does (pelvis height within 1 cm, every joint within 5 deg)."""
    import eval_precision as ep
    sup = np.load(os.path.join(SIM, "getup_ref_v6ar.npz"))
    pro = np.load(os.path.join(SIM, "getup_ref_v6ar_prone_ik.npz"))
    names_s = [str(n) for n in sup["joint_names"]]
    names_p = [str(n) for n in pro["joint_names"]]
    t0 = ep.ref_back_phase("getup_ref_v6ar_prone_ik.npz")
    for dt, tol_deg, tol_h in ((0.0, 15.0, None), (0.75, 5.0, 0.01)):
        k = int(round((t0 + dt) / float(pro["control_dt"])))
        dq = [abs(pro["q"][k][names_p.index(n)] - sup["q"][0][names_s.index(n)])
              for n in names_s]
        assert max(dq) < np.deg2rad(tol_deg)
        if tol_h is not None:
            assert abs(pro["qpos"][k][2] - sup["qpos"][0][2]) < tol_h


def test_scenario_setup_sets_the_phase():
    import eval_precision as ep
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    held = {k: v for k, v in kw["held_joints"].items()
            if robot_plant._role(k) not in ("shoulder", "elbow")}
    secs, factory, _ = ep._registry()["recover_supine_ph"]
    for npz, want in (("getup_ref_v6ar.npz", 0.0),
                      ("getup_ref_v6ar_prone_ik.npz",
                       ep.ref_back_phase("getup_ref_v6ar_prone_ik.npz"))):
        cfg = dict(kw, held_joints=held, rise_ref_npz=npz, rise_ref_residual=0.3,
                   w_rise_ref=2.0, precision=True, action_map="full")
        env = ep.make_env(cfg, secs, True, kw["xml_path"],
                          extra=ep.ENV_EXTRA["recover_supine_ph"])
        env.reset(seed=3)
        drv = ep.Driver(env, None, 3, False)
        _, ev = factory[0]()
        ev["_setup"](env, drv)
        assert env._rise_t0 == want
        assert env.data.qpos[2] < 0.1          # lying, the supine start
