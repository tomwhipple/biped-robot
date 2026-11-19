"""--pre-hike (dwell.hiked_pose, Dwell(pre_hike=)): a stance hip-roll hike in the plan, in over the settle, held through the
single support, out after the touchdown, the swing leg solved so its sole stays where the plan has it."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.join(HERE, "..", "experiments", "roll-balance")
sys.path.insert(0, RB)
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

np = pytest.importorskip("numpy")
H = pytest.importorskip("track_host")
D = pytest.importorskip("dwell")
S = pytest.importorskip("synth")
VK = pytest.importorskip("v6_kin")
from design_gates import JN  # noqa: E402

N = 5
KW = dict(T=4.0, n_steps=N, first="R", dx_steps=[0.02] * N, pelvis_x="stance", ankle_ff=(2.0, 2.0), dt=0.04)


def dw(**kw):
    return D.Dwell(**{**KW, **kw})


@pytest.mark.parametrize("stance", ["L", "R"])
@pytest.mark.parametrize("deg", [0.0, 5.2, 9.0])
@pytest.mark.parametrize("zsw", [0.0, 0.015])
def test_hiked_pose_keeps_both_soles_where_the_plan_puts_them_flat(stance, deg, zsw):
    """Forward kinematics in the MuJoCo plant with the pelvis rolled as the hike rolls it: both ankle-roll points on their
    targets, both soles level (the swing sole is on the floor / at its height, not lifted by the pelvis roll)."""
    K = S.kin(0.02)
    m, d = VK.load(K.p)
    sg = 1.0 if stance == "L" else -1.0
    fL = (0.01, 0.002, zsw if stance == "R" else 0.0)
    fR = (0.0, -0.003, zsw if stance == "L" else 0.0)
    q, al, pel = D.hiked_pose(K, math.radians(deg), stance, shift=sg * 0.04, fwd=0.005, footL=fL, footR=fR)
    VK.set_pose(m, d, np.array([q[n] for n in JN]), torso_pos=tuple(pel), torso_quat=VK.orient_quat(0.0, al))
    k = K.k1
    for side, tgt in (("L", k.footL + np.array(fL)), ("R", k.footR + np.array(fR))):
        pos, R = VK.foot_frame(m, d, side)
        assert np.linalg.norm(pos - tgt) < 1e-6, (side, pos - tgt)
        assert R[2, 2] > 1.0 - 1e-9
    # the pelvis rolled so that the SWING-side hip is the higher one
    assert (al < 0) == (stance == "L") or deg == 0.0


def test_without_the_swing_leg_solve_the_sole_would_lift():
    """Why the plan solves the swing leg: the firmware-style hike alone (the stance hip roll goal += theta) rolls the pelvis
    and the swing sole rises by the swing ankle's lateral distance from the stance hip x sin(theta) (12.5 mm at 5.2 deg) -- it would be unloaded in the settle."""
    K = S.kin(0.02)
    m, d = VK.load(K.p)
    th = math.radians(5.2)
    q = K.pose(shift=0.04, hike={"L": th})
    k = K.k1
    kk = k.copy()
    kk.pelvis = kk.pelvis + np.array([0.0, 0.04, 0.0])
    # the stance leg unchanged in the world, the pelvis rolled about its hip roll axis, the swing leg's joints as before
    p = K.p
    hr = kk.pelvis + np.array([0.0, p.hip_sep / 2, p.hip_z - p.d_yaw_roll])
    al = -th
    rx = np.array([[1, 0, 0], [0, math.cos(al), -math.sin(al)], [0, math.sin(al), math.cos(al)]])
    VK.set_pose(m, d, np.array([q[n] for n in JN]), torso_pos=tuple(hr + rx @ (kk.pelvis - hr)),
                torso_quat=VK.orient_quat(0.0, al))
    zL = VK.foot_frame(m, d, "L")[0][2]
    zR = VK.foot_frame(m, d, "R")[0][2]
    assert abs(zL - k.footL[2]) < 1e-6                                   # the stance foot is where it was
    assert zR - k.footR[2] > 0.006                                       # the swing sole is up
    assert 0.008 < zR - k.footR[2] < 0.016


def test_off_is_the_plan_as_it_was():
    a = dw()
    for kw in (dict(pre_hike=None), dict(pre_hike=(0.0, 0.0)), dict(pre_hike=(0.0, 0.0), pre_hike_frac=0.5)):
        b = dw(**kw)
        assert [s["q"] for s in a.segs] == [s["q"] for s in b.segs]
    base = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(20, 0, 0), lift_gate=True)
    same = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(20, 0, 0), lift_gate=True, pre_hike=None, pre_hike_frac=1.0)
    assert base["frames"] == same["frames"] and base["pre"] == same["pre"]


def test_the_hike_is_in_the_stance_hip_roll_only_over_the_settle_the_single_support_and_the_start_of_the_next_step():
    a, b = dw(), dw(pre_hike=(5.2, 3.4))
    sgn = {"L": 1.0, "R": -1.0}
    full = {"L": math.radians(5.2), "R": math.radians(3.4)}
    assert a.phases == b.phases
    for i, p in enumerate(b.phases):
        st = p["stance"]
        shift_end = p["t0"] - b.t_settle                     # the settle starts here
        t_out = 0.5 * b.t_shift
        for x, y in zip(a.segs[1:], b.segs[1:]):
            t = x["t"]
            dq = y["q"][f"{st}_hip_roll"] - x["q"][f"{st}_hip_roll"]
            if shift_end - 1e-9 <= t <= p["t1"] + 1e-9:
                m = D._mj((t - shift_end) / b.t_settle)
                assert dq == pytest.approx(sgn[st] * full[st] * m, abs=1e-12), (i, t)
            elif p["t1"] + 1e-9 < t <= p["t1"] + t_out:
                m = 1.0 - D._mj((t - p["t1"]) / t_out)
                assert dq == pytest.approx(sgn[st] * full[st] * m, abs=1e-12), (i, t)
            # the stance leg's other joints and the ankle roll: unchanged (the leg stays put in the world)
            if shift_end <= t <= p["t1"]:
                for n in ("hip_pitch", "knee", "ankle", "ankle_roll"):
                    assert y["q"][f"{st}_{n}"] == pytest.approx(x["q"][f"{st}_{n}"], abs=1e-12)
    # the feet's planned heights and places are the plan's
    for x, y in zip(a.segs[1:], b.segs[1:]):
        assert x["pa"]["feet"] == y["pa"]["feet"] and x["t"] == y["t"]
    # the swing leg moved to keep its sole: its joints differ, by a few degrees, in the settle and the single support
    p0 = b.phases[0]
    sw = p0["swing"]
    seg = lambda s_, t: min(s_.segs[1:], key=lambda g: abs(g["t"] - t))["q"]                    # noqa: E731
    t_mid = p0["t0"] + 0.5 * (p0["t1"] - p0["t0"])
    dsw = max(abs(seg(b, t_mid)[f"{sw}_{n}"] - seg(a, t_mid)[f"{sw}_{n}"]) for n in ("hip_roll", "hip_pitch", "knee", "ankle"))
    assert 0.5 < math.degrees(dsw) < 10.0


def test_pre_hike_frac_finishes_the_ramp_earlier_and_a_pause_takes_the_ramp_out():
    a = dw()
    b = dw(pre_hike=(5.0, 5.0), pre_hike_frac=0.5)
    p = b.phases[0]
    st = p["stance"]
    start = p["t0"] - b.t_settle
    seg = lambda s_, t: min(s_.segs[1:], key=lambda g: abs(g["t"] - t))["q"][f"{st}_hip_roll"]      # noqa: E731
    sg = 1.0 if st == "L" else -1.0
    assert seg(b, start + 0.5 * b.t_settle) - seg(a, start + 0.5 * b.t_settle) == pytest.approx(sg * math.radians(5.0), abs=1e-9)
    c = dw(pre_hike=(5.0, 5.0), td_pause=0.4)
    td = c.phases[0]["t1"]
    st = c.phases[0]["stance"]
    d_ = lambda t: min(c.segs[1:], key=lambda g: abs(g["t"] - t))["q"][f"{st}_hip_roll"]            # noqa: E731
    base = lambda t: min(dw(td_pause=0.4).segs[1:], key=lambda g: abs(g["t"] - t))["q"][f"{st}_hip_roll"]    # noqa: E731
    assert abs(d_(td) - base(td)) == pytest.approx(math.radians(5.0), abs=1e-9)
    assert abs(d_(td + 0.4) - base(td + 0.4)) == pytest.approx(0.0, abs=1e-9)            # out over the pause
    assert 0 < abs(d_(td + 0.2) - base(td + 0.2)) < math.radians(5.0)


def test_the_precheck_reads_the_lift_off_shift_through_the_hike_and_the_seeds_are_zeroed():
    base = dict(cmd=H.step_cmd(20, 0, 0), lift_gate=True)
    p0 = H.plan(4.0, 6, "fwd", **base)
    p1 = H.plan(4.0, 6, "fwd", **base, pre_hike=(5.2, 3.4))
    ok0, r0 = H.precheck(p0)
    ok1, r1 = H.precheck(p1)
    assert ok0 and ok1
    for x, y in zip([r for r in r0 if r["at"] in ("lift-off", "touchdown")], [r for r in r1 if r["at"] in ("lift-off", "touchdown")]):
        assert y["ratio"] == pytest.approx(x["ratio"], abs=0.004), (x, y)
    assert p0["pre"][:2] == ["trim T L 5.2", "trim T R 3.4"]
    assert p1["pre"][:2] == ["trim T L 0", "trim T R 0"]
    p2 = H.plan(4.0, 6, "fwd", **base, pre_hike=(5.2, 0.0))
    assert p2["pre"][:2] == ["trim T L 0", "trim T R 3.4"]
    assert H.plan(**p1["args"])["frames"] == p1["frames"]                               # a re-plan keeps it
    # a hike the legs cannot take is refused by the joint rule
    try:
        ok, rows = H.precheck(H.plan(4.0, 6, "fwd", **base, pre_hike=(12.0, 12.0), ankle_ff=(4.0, 4.0)))
    except ValueError:                                  # the IK cannot reach it: the plan is refused before the check
        ok = False
    assert not ok


def test_cli(capsys):
    assert H.main(["--T", "4", "--dir", "march", "--steps", "6", "--pre-hike", "5.2,3.4"]) == 0
    out = capsys.readouterr().out
    assert "pre-hike: L 5.2 / R 3.4 deg" in out and "seeded L 0, R 0 (was 5.2,3.4)" in out and "PASS" in out
    assert H.main(["--T", "4", "--dir", "march", "--steps", "6", "--pre-hike", "5.2,0", "--pre-hike-frac", "0.5",
                   "--raise-s", "0.42", "--move-s", "0.66", "--lower-s", "0.77"]) == 0
    out = capsys.readouterr().out
    assert "seeded L 0, R 3.4" in out and "raise 0.42 + move 0.66 + lower 0.77" in out
    for args in (["--pre-hike", "5"], ["--pre-hike", "13,2"], ["--pre-hike", "-1,2"], ["--pre-hike", "5,5", "--pre-hike-frac", "0"]):
        assert H.main(["--T", "4", "--dir", "march", "--steps", "4", *args]) == 2
        assert "REFUSED" in capsys.readouterr().out
