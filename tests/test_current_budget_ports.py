"""The per-branch bus-current budget (sim/current_budget.py --trace refs | dwell; #77).

Power lite's four servo branches (J4 R leg, J5 R upper, J6 L leg, J7 L upper)
against the servo map, the per-branch summing, the per-servo-type current
models (STS3250 hip rolls) against sim/sts_servo_model.py, and one get-up
reference replay summed per branch.
"""
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import current_budget as CB  # noqa: E402
import sts_servo_model as SM  # noqa: E402

LEGS = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
        "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll"]
ALL17 = LEGS + ["neck_yaw", "L_shoulder", "L_elbow", "R_shoulder", "R_elbow"]


def test_branches_cover_the_map_once_and_match_the_ports():
    ids = [i for m in CB.BRANCHES.values() for i in m]
    assert sorted(ids) == sorted(CB.NAME_TO_ID.values())
    assert len(ids) == len(set(ids)) == 17
    b = CB.BRANCHES
    assert b["J4 R leg"] | b["J5 R upper"] == CB.PORT_A_IDS
    assert b["J6 L leg"] | b["J7 L upper"] == CB.PORT_B_IDS
    assert CB.STS3250_IDS == {CB.NAME_TO_ID["L_hip_roll"], CB.NAME_TO_ID["R_hip_roll"]}


def test_branches_match_servo_map_doc():
    """docs/servo-map.md 2.1's table: port A/B, leg/upper branch -> J4-J7"""
    text = open(os.path.join(ROOT, "docs", "servo-map.md")).read()
    rows = re.findall(r"^\| (\d+) \| `(\w+)`[^|]*\| ([AB]), (leg|upper) branch \|", text, re.M)
    assert len(rows) == 17
    jx = {("A", "leg"): "J4 R leg", ("A", "upper"): "J5 R upper",
          ("B", "leg"): "J6 L leg", ("B", "upper"): "J7 L upper"}
    for sid, name, port, branch in rows:
        assert CB.NAME_TO_ID[name] == int(sid)
        assert int(sid) in CB.BRANCHES[jx[(port, branch)]], (sid, name)


def test_branch_sums_follow_the_map():
    rng = np.random.default_rng(0)
    names = list(rng.permutation(ALL17))
    amps = rng.uniform(0.0, 2.0, size=(5, len(names)))
    out = CB.branch_amps(names, amps)
    for b, members in CB.BRANCHES.items():
        cols = [k for k, n in enumerate(names) if CB.NAME_TO_ID[n] in members]
        assert np.allclose(out[b], amps[:, cols].sum(axis=1))
    assert np.allclose(out["H5 (A)"], out["J4 R leg"] + out["J5 R upper"])
    assert np.allclose(out["H6 (B)"], out["J6 L leg"] + out["J7 L upper"])
    assert np.allclose(out["bus"], amps.sum(axis=1))


def test_one_servo_lands_on_its_branch():
    for name in ALL17:
        amps = np.array([[1.0 if n == name else 0.0 for n in ALL17]])
        out = CB.branch_amps(ALL17, amps)
        hit = [b for b in CB.BRANCHES if out[b][0] > 0.5]
        assert len(hit) == 1 and CB.NAME_TO_ID[name] in CB.BRANCHES[hit[0]]


def test_servos_the_plant_lacks_count_idle():
    out = CB.branch_amps(LEGS, np.zeros((3, 12)))
    assert np.allclose(out["J4 R leg"], 0.0) and np.allclose(out["J6 L leg"], 0.0)
    assert np.allclose(out["J5 R upper"], 3 * SM.I_IDLE)
    assert np.allclose(out["J7 L upper"], 2 * SM.I_IDLE)
    assert np.allclose(out["bus"], 5 * SM.I_IDLE)


def test_unknown_or_duplicate_servo_raises():
    with pytest.raises(KeyError):
        CB.branch_amps(["L_hip_yaw", "tail"], np.zeros((1, 2)))
    with pytest.raises(ValueError):
        CB.branch_amps(["L_knee", "L_knee"], np.zeros((1, 2)))


def test_sts3215_current_is_sts_servo_model_s():
    rng = np.random.default_rng(1)
    names = ["L_knee", "R_shoulder"]
    tau = rng.uniform(-3, 3, size=(50, 2))
    qd = rng.uniform(-4, 4, size=(50, 2))
    w0, v = np.array([3.7, 3.7]), 11.1
    assert np.allclose(CB.servo_amps(names, tau, qd, w0, v, "motor"),
                       SM.supply_current(tau, qd, 3.7, v, "motor"))
    assert np.allclose(CB.servo_amps(names, tau, qd, w0, v, "bridge"),
                       SM.supply_current(tau, qd, 3.7, v, "bridge"))


@pytest.mark.parametrize("volts", [11.1, 12.0])
def test_stall_current_per_servo_type(volts):
    """at its own stall torque (scaled to the bus, as the plant scales it) a
    servo draws its own stall current in both models; the STS3250 hip roll
    (4.2 A at 12 V) above the STS3215 (2.7 A)"""
    names = ["L_knee", "L_hip_roll"]
    k = volts / 12.0
    tau = np.array([[SM.STALL_NM_12V * k, 4.90 * k]])
    qd = np.zeros_like(tau)
    w0 = np.array([4.0, 4.0])
    motor = CB.servo_amps(names, tau, qd, w0, volts, "motor")[0]
    bridge = CB.servo_amps(names, tau, qd, w0, volts, "bridge")[0]
    assert motor == pytest.approx([2.7 * k, 4.2 * k])
    assert bridge == pytest.approx([SM.I_IDLE + 2.7 * k, SM.I_IDLE + 4.2 * k], rel=3e-3)


def test_worst_window():
    a = np.array([0, 0, 4, 4, 0, 1, 1, 1], float)
    assert CB.worst_window(a, 2) == 4.0
    assert CB.worst_window(a, 3) == pytest.approx(8 / 3)
    assert CB.worst_window(a, 100) == pytest.approx(a.mean())


def test_supine_reference_summed_per_branch():
    """getup_ref_overload.run(trace=True) on the supine reference: the traces
    reproduce that study's own per-servo peak current on every STS3215, and
    the per-branch stats are consistent (bus >= each lead, both models)"""
    import getup_ref_overload as O
    r = O.run("supine_ik", O.NOMINAL, "11.1V", trace=True)
    assert r["ok"]
    names = r["names"]
    assert r["tau"].shape == r["qd"].shape == (len(r["tau"]), 17)
    i_m = CB.servo_amps(names, r["tau"], r["qd"], r["w0"], r["volts"], "motor")
    for k, n in enumerate(names):
        if CB.servo_type(n) == "sts3215":
            assert i_m[:, k].max() == pytest.approx(r["prot"][n]["peak_i"], rel=1e-6), n
    st = CB.branch_stats(names, r["tau"], r["qd"], r["w0"], r["volts"], 0.02)
    for model in CB.CURRENT_MODELS:
        s = st[model]
        for b in CB.BRANCHES:
            assert s["bus"]["peak"] >= s[b]["peak"] > 0.0
            assert s[b]["peak"] >= s[b]["worst2s"] >= s[b]["mean"]
        assert s["bus"]["mean"] == pytest.approx(sum(s[b]["mean"] for b in CB.BRANCHES))
    assert st["motor"]["bus"]["mean"] > st["bridge"]["bus"]["mean"]
