"""The centred crouch hold of the analytic get-up reference (get-up campaign,
2026-10-24).

The seat push lifted the hands off the floor at its crouch hold (push pose ->
REST), which moves the CoM to 48 mm behind the sole centre; replayed through
the referee, the reference and every hold-trained policy tipped back onto the
seat there. getup_ref_v6ar_prone_ikc.npz keeps the hands down and the ankle
at -35 deg through the hold. These pin that it changes nothing else: the same
keys and timing as prone_ik, so a policy's phase and the referee's starts
(recover_prone_fall, recover_supine_ph) are unchanged.
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

PATH = [(-100.0, -110.0, -30.0), (-50.0, -60.0, -20.0)]   # any waypoints: the rise is not under test


def test_centred_changes_only_the_crouch_hold():
    import getup_ref_analytic as A
    base, cen = A.seat_push_ik(PATH), A.seat_push_ik(PATH, centred=True)
    assert [k[0] for k in base] == [k[0] for k in cen]
    for kb, kc in zip(base, cen):
        assert kb[2:] == kc[2:]                       # move and hold times
        if kb[0] == "crouch hold":
            assert kc[1] == dict(kb[1], **A.CROUCH_C)
            assert kb[1]["shoulder"] == A.REST["shoulder"]
        else:
            assert kb[1] == kc[1]


def test_recorded_centred_reference_matches_prone_ik_outside_the_hold():
    import eval_precision as ep
    a = np.load(os.path.join(SIM, "getup_ref_v6ar_prone_ik.npz"))
    c = np.load(os.path.join(SIM, "getup_ref_v6ar_prone_ikc.npz"))
    assert a["q"].shape == c["q"].shape
    assert [str(x) for x in a["label"]] == [str(x) for x in c["label"]]
    np.testing.assert_array_equal(a["qpos"][0], c["qpos"][0])   # the recover_prone_fall start
    lab = [str(x) for x in c["label"]]
    k = lab.index("crouch hold")
    np.testing.assert_array_equal(a["q"][:k], c["q"][:k])
    assert ep.ref_back_phase("getup_ref_v6ar_prone_ikc.npz") == ep.ref_back_phase("getup_ref_v6ar_prone_ik.npz")
    assert float(c["final_z"]) > 0.38 and float(c["final_up"]) > 0.99
