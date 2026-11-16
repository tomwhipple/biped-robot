"""track_host's by-step feed-forwards (2026-11-16, README "By step type"): the stance-ankle ff toward a sideways step's
own side (--ankle-ff-side), the first-stance-step boost (--first-ankle-boost), the heading drift by step type
(--heading-drift-side). Frame-level: the ankle-roll goals of the affected steps differ by the expected amount and
everything else is identical; with the options off the plans are the ones the tool made before."""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")
C = pytest.importorskip("cmd_walk")

BASE = dict(T_step=4.0, seeds=H.ROBOT_SEED, lead_ms=2500, cal=None, amp=H.ROBOT_AMP, model="dwell", ds=None, h=None,
            pelvis_x="stance", ankle_ff=(2.0, 2.0), k_ds=None, drop=None, lift_gate=True)


def _h(p):
    import hashlib
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]


def seg_diff(a, b):
    """{joint: max |difference| deg} over the keyframes of two plans, and which keyframes (by their step) differ."""
    sa, sb = a["sway"], b["sway"]
    assert len(sa.segs) == len(sb.segs)
    mx, where = {}, {}
    for k, (x, y) in enumerate(zip(sa.segs, sb.segs)):
        if x.get("q") is None:
            continue
        assert x["t"] == y["t"]
        for n in x["q"]:
            d = math.degrees(abs(y["q"][n] - x["q"][n]))
            if d > 1e-9:
                mx[n] = max(mx.get(n, 0.0), d)
                where.setdefault(n, set()).add(sum(1 for p in sa.phases if p["t0"] <= x["t"]) - 1)
    return mx, where


def seq_plan(dy_mm, n, **kw):
    return H.plan(4.0, n, "march", seq=[(H.step_cmd(0, dy_mm, 0), n)], lift_gate=True, **kw)


# ---- the ankle feed-forward

def test_ankle_extra_rule():
    # stepping right loads the RIGHT stance (steps 1, 3, 5 with the first swing foot R); linear in |dy| / 15 mm
    assert [H.ankle_extra(i, -0.015, "R", (2.0, 3.0)) for i in range(6)] == [0, 1, 0, 1, 0, 1]
    assert H.ankle_extra(1, -0.0075, "R", (2.0, 3.0)) == pytest.approx(0.5)
    assert H.ankle_extra(1, -0.030, "R", (2.0, 3.0)) == pytest.approx(1.0)          # capped at the flag's value
    assert H.ankle_extra(1, +0.015, "R", (2.0, 3.0)) == 0                            # left steps: no change at 2,3
    assert H.ankle_extra(0, +0.015, "R", (3.5, 3.0)) == pytest.approx(1.5)           # the mirrored parameter (left stance)
    assert H.ankle_extra(1, -0.015, "R", None) == 0 and H.ankle_extra(3, 0.0, "R", (2.0, 3.0)) == 0
    assert H.ankle_extra(0, 0.0, "R", None, 1.0) == 1.0 == H.ankle_extra(1, 0.0, "R", None, 1.0)
    assert H.ankle_extra(2, 0.0, "R", None, 1.0) == 0
    assert H.ankle_extra(0, -0.015, "L", (2.0, 3.0)) == 1.0                          # first swing foot L: step 0 is R stance


def test_side_ankle_ff_changes_only_the_right_stance_ankle_roll_of_the_right_steps():
    a, b = seq_plan(-15, 6), seq_plan(-15, 6, ankle_side=(2.0, 3.0))
    mx, where = seg_diff(a, b)
    assert set(mx) == {"R_ankle_roll"}
    assert mx["R_ankle_roll"] == pytest.approx(1.0, abs=1e-6)                       # 1 deg more at the held peak
    assert where["R_ankle_roll"] == {1, 3, 5}                                       # the R-stance steps only
    assert _h(a) != _h(b)
    # a left walk, and a half step (linear in |dy|): the left one is untouched at 2,3, the half step adds 0.5
    assert _h(seq_plan(15, 6)) == _h(seq_plan(15, 6, ankle_side=(2.0, 3.0)))
    mx, _ = seg_diff(seq_plan(-7.5, 6), seq_plan(-7.5, 6, ankle_side=(2.0, 3.0)))
    assert mx["R_ankle_roll"] == pytest.approx(0.5, abs=1e-6)
    # the mirrored parameter acts on the left stance of a left walk
    mx, where = seg_diff(seq_plan(15, 6), seq_plan(15, 6, ankle_side=(3.0, 2.0)))
    assert set(mx) == {"L_ankle_roll"} and where["L_ankle_roll"] == {0, 2, 4}


def test_the_learned_ankle_trim_is_added_to_not_replaced():
    st = [1.5, 2.5, 1.5, 2.5, 1.5, 2.5]
    a, b = seq_plan(-15, 6, ankle_steps=st), seq_plan(-15, 6, ankle_steps=st, ankle_side=(2.0, 3.0))
    mx, where = seg_diff(a, b)
    assert set(mx) == {"R_ankle_roll"} and mx["R_ankle_roll"] == pytest.approx(1.0, abs=1e-6)
    assert b["args"]["ankle_steps"] == st                       # the learner's own list is not touched (it learns from it)


def test_first_ankle_boost_is_the_first_stance_step_of_each_side_only():
    a, b = seq_plan(0, 6), seq_plan(0, 6, first_boost=1.0)
    mx, where = seg_diff(a, b)
    assert set(mx) == {"L_ankle_roll", "R_ankle_roll"}
    assert all(v == pytest.approx(1.0, abs=1e-6) for v in mx.values())
    assert where["L_ankle_roll"] == {0} and where["R_ankle_roll"] == {1}
    # it adds to the side ff of the same step: step 1 of a right walk gets 1 + 1
    c = seq_plan(-15, 6, ankle_side=(2.0, 3.0), first_boost=1.0)
    mx, where = seg_diff(seq_plan(-15, 6, ankle_side=(2.0, 3.0)), c)
    assert where["R_ankle_roll"] == {1} and where["L_ankle_roll"] == {0}


def test_options_off_leave_the_plans_as_they_were():
    for kw in (dict(), dict(ankle_side=None, first_boost=0.0, drift_side=None), dict(ankle_side=(2.0, 2.0)),
               dict(drift_side=(H.YAW_DRIFT, H.YAW_DRIFT))):
        for dy in (-15, 15, 0):
            assert _h(seq_plan(dy, 6, **kw)) == _h(seq_plan(dy, 6)), (kw, dy)
    # a --dir preset and a --step run, too
    assert _h(H.plan(4.0, 6, "sideR", ankle_side=(2.0, 2.0))) == _h(H.plan(4.0, 6, "sideR"))
    assert _h(H.plan(4.0, 6, "sideR", ankle_side=(2.0, 3.0))) != _h(H.plan(4.0, 6, "sideR"))
    mx, where = seg_diff(H.plan(4.0, 6, "sideR"), H.plan(4.0, 6, "sideR", ankle_side=(2.0, 3.0)))
    assert set(mx) == {"R_ankle_roll"} and mx["R_ankle_roll"] == pytest.approx(1.0, abs=1e-6)


def test_the_precheck_ankle_range_rule_covers_the_extra():
    ok, _ = H.precheck(seq_plan(-15, 6, ankle_side=(2.0, 3.0), first_boost=1.0))
    assert ok
    ok, rows = H.precheck(seq_plan(-15, 6, ankle_side=(2.0, 12.0)))
    bad = [r for r in rows if not r["ok"]]
    assert not ok and any(r["at"] == "joint limit" and r["stance"] == "R_ankle_roll" for r in bad)


def test_cli_values_are_checked(capsys):
    for flag, val in (("--ankle-ff-side", "2"), ("--ankle-ff-side", "2,9"), ("--first-ankle-boost", "5"),
                      ("--heading-drift-side", "x,1")):
        assert H.main(["--T", "4", "--seq", "sideR:4", flag, val]) == 2
        assert "REFUSED" in capsys.readouterr().out
    assert H.main(["--T", "4", "--seq", "sideR:4", "--ankle-ff-side", "2,3", "--first-ankle-boost", "1"]) == 0


# ---- the heading drift

def test_step_drift_blends_by_the_lateral_share():
    side = (0.94, 1.9)
    assert H.step_drift(H.step_cmd(40, 0, 0), side) == H.YAW_DRIFT
    assert H.step_drift(H.step_cmd(0, 0, 0), side) == H.YAW_DRIFT
    assert H.step_drift(H.step_cmd(0, 15, 0), side) == pytest.approx(0.94)
    assert H.step_drift(H.step_cmd(0, -15, 0), side) == pytest.approx(1.9)
    assert H.step_drift(H.step_cmd(15, 15, 0), side) == pytest.approx(0.5 * 1.9 + 0.5 * 0.94)
    assert H.step_drift(H.step_cmd(0, 15, 0), None) == H.YAW_DRIFT


def test_heading_hold_feed_forward_follows_the_step_type():
    cmds = [H.step_cmd(20, 0, 0), H.step_cmd(0, 15, 0), H.step_cmd(0, -15, 0)]
    hold = H.HeadingHold(3, [0.0] * 3, drift=[H.step_drift(c, (0.94, 1.9)) for c in cmds])
    assert hold.asked == pytest.approx([-1.9 / 1.34, -0.94 / 1.34, -1.9 / 1.34])
    assert H.HeadingHold(3, [0.0] * 3).asked == pytest.approx([-1.9 / 1.34] * 3)
    # the feedback acts on top of it, per step
    hold.update(1, -2.0, 1)
    assert hold.asked[1] == pytest.approx((0.5 * 2.0 - 0.94) / 1.34)


def test_constant_yaw_trim_is_scaled_by_the_drift_of_each_step():
    seq = [(H.step_cmd(20, 0, 0), 3), (H.step_cmd(0, 15, 0), 4)]
    a = H.plan(4.0, 7, "march", seq=seq, lift_gate=True)
    b = H.plan(4.0, 7, "march", seq=seq, lift_gate=True, drift_side=(0.94, 1.9))
    mx, where = seg_diff(a, b)
    assert set(mx) <= {"L_hip_yaw", "R_hip_yaw"} and mx
    assert min(min(w) for w in where.values()) >= 3                        # the three forward steps are untouched


# ---- --cmd: the walk decides each step's extras from its absolute index

def mk(script, **kw):
    return C.CmdWalk(H, C.ListSource(script), BASE, first="R", say=lambda *a, **k: None, **kw)


def test_cmd_walk_defaults_are_off_in_the_class_and_on_from_the_tool():
    cw = mk(["0 -3.75 0"] * 8)
    cw.rehearse(6)
    assert all(c["aff_x"] == 0 for c in cw.plan_kwargs()["step_cmds"])
    cw = mk(["0 -3.75 0"] * 8, ankle_side=(2.0, 3.0), first_boost=1.0, drift_side=(0.94, 1.9))
    cw.rehearse(6)
    assert any(c["aff_x"] for c in cw.plan_kwargs()["step_cmds"])


def test_cmd_walk_frames_differ_only_in_the_ankle_roll_goals_of_the_affected_steps():
    script = ["0 -3.75 0"] * 10
    plain, by = mk(script), mk(script, ankle_side=(2.0, 3.0), first_boost=1.0)
    plain.rehearse(6)
    by.rehearse(6)
    assert [c["dy"] for c in plain.steps] == [c["dy"] for c in by.steps]            # same decisions
    ex = [c["aff_x"] for c in by.plan_kwargs()["step_cmds"]]
    pa, pb = H.plan(**plain.plan_kwargs()), H.plan(**by.plan_kwargs())
    mx, where = seg_diff(pa, pb)
    assert set(mx) == {"L_ankle_roll", "R_ankle_roll"}
    # the walk's dy ramps 10 mm per step (slew): the extras are boost + (3 - 2) x min(1, |dy| / 15) on the R stance steps
    want = {"L": max(ex[i] for i in range(0, len(ex), 2)), "R": max(ex[i] for i in range(1, len(ex), 2))}
    assert mx["L_ankle_roll"] == pytest.approx(want["L"], abs=1e-6) and mx["R_ankle_roll"] == pytest.approx(want["R"], abs=1e-6)
    assert want["L"] == pytest.approx(1.0) and want["R"] == pytest.approx(2.0)      # the first R stance: boost 1 + 1
    assert set(where["R_ankle_roll"]) == {i for i in range(1, len(ex), 2) if ex[i]}


def test_cmd_walk_check_window_sees_the_extra_and_every_step_passes_the_whole_plan():
    for script in (["12.5 0 0"] * 4 + ["-10 0 0"] * 4, ["0 -3.75 0"] * 4 + ["0 3.75 0"] * 4):
        cw = mk(script + ["stop"], ankle_side=(2.0, 3.0), first_boost=1.0, drift_side=(0.94, 1.9),
                limits=dict(slew=(1e9, 1e9, 1e9)))
        cw.rehearse(8)
        ok, rows = H.precheck(H.plan(**cw.plan_kwargs()))
        assert ok, [r for r in rows if not r["ok"]]
        assert len(cw.steps) >= 8
    cw = mk(["12.5 0 0"] * 4 + ["-10 0 0"] * 4, limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.rehearse(8)
    assert max(c["dx"] for c in cw.steps) == pytest.approx(0.05) and min(c["dx"] for c in cw.steps) == pytest.approx(-0.04)
    assert [c["tilt"] for c in cw.steps[:4]] == [5.0] * 4


def test_cmd_walk_hold_asks_the_drift_of_the_step_type():
    hold = H.HeadingHold(1, [0.0])
    cw = mk(["0 3.75 0"] * 2 + ["10 0 0"] * 2, hold=hold, drift_side=(0.94, 1.9), limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.decide()
    cw.decide()
    cw.decide()
    assert hold.asked[0] == pytest.approx(-0.94 / 1.34) and hold.asked[1] == pytest.approx(-0.94 / 1.34)
    assert hold.asked[2] == pytest.approx(-1.9 / 1.34)
