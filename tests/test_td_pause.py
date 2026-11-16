"""dwell's step-timing and landing options (2026-11-17, README "The touchdown pause"): --td-pause, --settle,
--lower-frac / --lower-s, --land-early-mm. Frame level: what the option changes changes, everything else is identical,
and with the options off the frames are the ones the tool made before."""
import hashlib
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")
C = pytest.importorskip("cmd_walk")
D = pytest.importorskip("dwell")

N = 5
KW = dict(T=4.0, n_steps=N, first="R", dx_steps=[0.04] * N, pelvis_x="stance", ankle_ff=(2.0, 2.0), swing_tilt=3.0, dt=0.04)
P = 0.32                                    # eight 40 ms frames: a shifted plan lands on the same frame grid


def dw(**kw):
    return D.Dwell(**{**KW, **kw})


def _h(p):
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]


def maxdiff(a, b, joints=None, ia=None, ib=None):
    out = {}
    for x, y in zip(a, b):
        for n in x["q"]:
            d = abs(y["q"][n] - x["q"][n])
            if d > 1e-12:
                out[n] = max(out.get(n, 0.0), math.degrees(d))
    return out


def test_options_off_leave_the_plan_as_it_was():
    for kw in (dict(), dict(td_pause=0.0, settle=None, lower_frac=None, land_early=0.0)):
        a, b = dw(), dw(**kw)
        assert [s["q"] for s in a.segs] == [s["q"] for s in b.segs] and a.phases == b.phases
    base = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(40, 0, 0), swing_tilt=3.0, lift_gate=True)
    same = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(40, 0, 0), swing_tilt=3.0, lift_gate=True, td_pause=0.0, settle=None,
                  lower_frac=None, land_early=0.0)
    assert _h(base) == _h(same)
    assert dw().step_time == 4.0 and D.effective_step_s(4.0) == 4.0
    # the lowering share written out equals the default: the same frames
    assert [s["q"] for s in dw(lower_frac=0.35).segs] == [s["q"] for s in dw().segs]


def test_timeline_today_has_no_pause_the_next_shift_starts_at_the_touchdown():
    sw = dw()
    p0, p1 = sw.phases[0], sw.phases[1]
    assert p0["t1"] + sw.t_shift + sw.t_settle == pytest.approx(p1["t0"])          # the next shift starts AT the touchdown
    assert (p0["t1"] - p0["t0"], sw.t_shift, sw.t_settle) == pytest.approx((2.2, 1.4, 0.4))
    assert p0["lower_start"] - p0["t0"] == pytest.approx(0.65 * 2.2)


def test_td_pause_holds_everything_then_the_shift_is_the_old_one_shifted():
    a, b = dw(), dw(td_pause=P)
    assert b.step_time == pytest.approx(4.0 + P)
    k = round(P / 0.04)
    ta = {round(s["t"], 6): s for s in a.segs[1:]}
    for i, (pa_, pb_) in enumerate(zip(a.phases, b.phases)):
        assert pb_["t0"] - pa_["t0"] == pytest.approx(i * P)                       # every step after the first, P later
        assert (pb_["t1"] - pb_["t0"]) == pytest.approx(pa_["t1"] - pa_["t0"])
    # during each pause every joint stays where the touchdown frame left it ...
    for i in range(1, N):
        td = b.phases[i - 1]["t1"]
        seg_td = min(b.segs[1:], key=lambda s: abs(s["t"] - td))
        held = [s for s in b.segs[1:] if td - 1e-9 <= s["t"] <= td + P + 1e-9]
        assert len(held) == k + 1
        for s in held:
            assert s["q"] == pytest.approx(seg_td["q"], abs=1e-12)
        # ... and what follows is the old plan, P later
        ia = next(j for j, s in enumerate(a.segs) if abs(s["t"] - (td - (i - 1) * P)) < 1e-6)
        ib = next(j for j, s in enumerate(b.segs) if abs(s["t"] - (td + P)) < 1e-6)
        for j in range(60):
            assert b.segs[ib + j]["q"] == pytest.approx(a.segs[ia + j]["q"], abs=1e-12)
    # the ankle feed-forward and the swing tilt are out at the touchdown (the pause starts from the plain pose)
    seg = min(b.segs[1:], key=lambda s: abs(s["t"] - b.phases[0]["t1"]))
    assert seg["pa"]["feet"][b.phases[0]["swing"]][2] == pytest.approx(0.0, abs=1e-12)


def test_td_pause_moves_only_the_timing_of_the_closing_shift_too():
    a, b = dw(), dw(td_pause=P)
    assert b.t_end - a.t_end == pytest.approx(N * P + P)    # N steps after the first + the closing shift... (see below)


def test_settle_changes_only_the_settle_and_so_the_step_time():
    a, b = dw(), dw(settle=0.2)
    assert b.step_time == pytest.approx(3.8) and D.effective_step_s(4.0, 0.0, 0.2) == pytest.approx(3.8)
    assert (b.t_shift, b.t_settle, b.t_ss) == pytest.approx((1.4, 0.2, 2.2))
    for pa_, pb_ in zip(a.phases, b.phases):
        assert pb_["t1"] - pb_["t0"] == pytest.approx(2.2)
    assert b.phases[1]["t0"] - b.phases[0]["t0"] == pytest.approx(3.8)
    c = dw(td_pause=0.3, settle=0.2)
    assert c.step_time == pytest.approx(4.1) and D.effective_step_s(4.0, 0.3, 0.2) == pytest.approx(4.1)
    assert c.phases[1]["t0"] - c.phases[0]["t0"] == pytest.approx(4.1)
    assert c.phases[0]["t0"] == pytest.approx(a.phases[0]["t0"] - 0.2)              # the first step has no pause


def test_lower_frac_changes_the_descent_only():
    a, b = dw(), dw(lower_frac=0.45)
    for pa_, pb_ in zip(a.phases, b.phases):
        assert (pb_["t0"], pb_["t1"]) == pytest.approx((pa_["t0"], pa_["t1"]))
        assert pb_["lower_start"] - pb_["t0"] == pytest.approx(0.55 * 2.2)
    # identical up to the end of the raise (0.35), different after it
    p0 = a.phases[0]
    for x, y in zip(a.segs, b.segs):
        if x["t"] <= p0["raise_end"] + 1e-9:
            assert x["q"] == y["q"]
    z = lambda sw, t: min(sw.segs[1:], key=lambda s: abs(s["t"] - t))["pa"]["feet"][sw.phases[0]["swing"]][2]    # noqa: E731
    t_a, t_b = p0["t0"] + 0.75 * 2.2, p0["t0"] + 0.75 * 2.2
    assert z(b, t_b) < z(a, t_a)                                  # further down at the same instant: the descent starts earlier
    assert z(a, p0["t1"]) == pytest.approx(0.0, abs=1e-9) == pytest.approx(z(b, p0["t1"]), abs=1e-9)
    # the touchdown frame and everything after it
    for x, y in zip(a.segs, b.segs):
        if x["t"] >= p0["t1"] - 1e-9 and x["t"] <= a.phases[1]["t0"] - 1e-9:
            assert x["q"] == pytest.approx(y["q"], abs=1e-12)
    # the phase the firmware sees: raise end 0.35 and lowering start 0.65 land on this gait's
    import math as m
    ph = lambda t: b.phi(t) / m.pi - 1.0                            # noqa: E731  (first R: half cycle 0, offset pi)
    assert (ph(p0["t0"] + 0.35 * 2.2 + 1e-9) - 0.25) / 0.5 == pytest.approx(0.35, abs=1e-3)
    assert (ph(b.phases[0]["lower_start"] + 1e-9) - 0.25) / 0.5 == pytest.approx(0.65, abs=1e-3)
    with pytest.raises(ValueError):
        dw(lower_frac=0.6)


def test_land_early_presses_the_sole_below_the_floor_and_lets_up_over_the_next_shift():
    a, b = dw(), dw(land_early=2.0)
    p0, p1 = a.phases[0], a.phases[1]
    sw = p0["swing"]
    z = lambda s, t: min(s.segs[1:], key=lambda g: abs(g["t"] - t))["pa"]["feet"][sw][2]    # noqa: E731
    assert z(b, p0["t1"]) == pytest.approx(-2e-3, abs=1e-9) and z(a, p0["t1"]) == pytest.approx(0.0, abs=1e-12)
    # the foot reaches the floor BEFORE the touchdown time
    t_floor = min(g["t"] for g in b.segs[1:] if g["t"] > p0["lower_start"] and g["pa"]["feet"][sw][2] <= 0.0)
    assert t_floor < p0["t1"] - 0.05
    # what differs: that leg's joints, from the lowering on; the shift's release returns it to the plain pose by the end
    diff = maxdiff([g for g in a.segs[1:] if g["t"] <= p1["t0"]], [g for g in b.segs[1:] if g["t"] <= p1["t0"]])
    assert diff and all(n.startswith(sw + "_") or n.startswith(p1["swing"] + "_") or n.startswith(p0["stance"] + "_")
                        for n in diff)
    end_shift = p0["t1"] + b.t_shift
    for x, y in zip(a.segs, b.segs):
        if p0["t1"] + b.t_shift + 1e-9 <= x["t"] <= p1["t0"] - 1e-9 and x["pa"]["feet"][sw][2] == 0.0:
            assert x["pa"]["feet"][sw][2] == y["pa"]["feet"][sw][2] == 0.0
    assert end_shift < p1["t0"]
    # held through a pause
    c = dw(land_early=2.0, td_pause=P)
    td = c.phases[0]["t1"]
    held = [g for g in c.segs[1:] if td <= g["t"] <= td + P]
    assert all(g["pa"]["feet"][sw][2] == pytest.approx(-2e-3) for g in held)
    with pytest.raises(ValueError):
        dw(land_early=5.0)


def test_the_precheck_covers_the_options():
    base = dict(cmd=H.step_cmd(40, 0, 0), swing_tilt=3.0, lift_gate=True)
    for kw in (dict(td_pause=0.3), dict(td_pause=0.3, settle=0.2), dict(land_early=4.0), dict(lower_frac=0.45)):
        ok, rows = H.precheck(H.plan(4.0, 6, "fwd", **base, **kw))
        assert ok, (kw, [r for r in rows if not r["ok"]])


def test_seq_and_cmd_carry_the_options():
    seq = [(H.step_cmd(40, 0, 0), 3), (H.step_cmd(0, -15, 0), 3)]
    a = H.plan(4.0, 6, "march", seq=seq, lift_gate=True)
    b = H.plan(4.0, 6, "march", seq=seq, lift_gate=True, td_pause=0.3, settle=0.2)
    assert b["sway"].step_time == pytest.approx(4.1) and _h(a) != _h(b)
    assert b["args"]["td_pause"] == 0.3 and H.plan(**b["args"])["frames"] == b["frames"]       # a re-plan keeps them
    base = dict(T_step=4.0, seeds=H.ROBOT_SEED, lead_ms=2500, cal=None, amp=H.ROBOT_AMP, model="dwell", ds=None, h=None,
                pelvis_x="stance", ankle_ff=(2.0, 2.0), k_ds=None, drop=None, lift_gate=True, td_pause=0.3, settle=0.2)
    cw = C.CmdWalk(H, C.ListSource(["10 0 0"] * 6), base, first="R", say=lambda *a, **k: None, step_s=4.1,
                   limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.decide()
    assert cw.log[0]["step_mm_deg"][0] == pytest.approx(41.0) and cw.T == 4.1             # v x the effective step time
    cw.rehearse(5)
    pl = H.plan(**cw.plan_kwargs())
    assert pl["sway"].td_pause == 0.3 and H.precheck(pl)[0]


def test_cli(capsys):
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "6", "--stride-mm", "40", "--swing-tilt", "3",
                   "--td-pause", "0.3", "--settle", "0.2", "--lower-s", "1.0", "--land-early-mm", "2"]) == 0
    out = capsys.readouterr().out
    assert "timeline: pause 0.30 + shift 1.40 + settle 0.20" in out and "= 4.10 s per step" in out and "PASS" in out
    assert "lower 1.00" in out and "2 mm below the floor" in out and "one step per 4.1 s" in out
    for args in (["--td-pause", "3"], ["--land-early-mm", "5"], ["--lower-frac", "0.7"], ["--lower-s", "1", "--lower-frac", "0.3"],
                 ["--settle", "-1"]):
        assert H.main(["--T", "4", "--dir", "fwd", "--steps", "4", *args]) == 2
        assert "REFUSED" in capsys.readouterr().out
    f = os.path.join(os.path.dirname(__file__), "..", "README.md")
    assert H.main(["--T", "4", "--steps", "4"]) == 0 and "timeline: shift 1.40 + settle 0.40" in capsys.readouterr().out


# ---- placement: the options must not change WHERE a foot lands (sidesteps, turns, diagonals)

PLACE = {"sideL": dict(dy_steps=[0.015] * N), "sideR": dict(dy_steps=[-0.015] * N),
         "turnL": dict(yaw_trim=[5.0 / 1.34] * N), "turnR": dict(yaw_trim=[-5.0 / 1.34] * N),
         "diag": dict(dx_steps=[0.03] * N, dy_steps=[0.01] * N, yaw_trim=[2.0] * N)}
OPTS = {"pause": dict(td_pause=0.32), "pause+settle": dict(td_pause=0.3, settle=0.2),
        "lower": dict(lower_frac=0.45), "press": dict(land_early=3.0),
        "all": dict(td_pause=0.3, settle=0.2, lower_frac=0.4, land_early=2.0)}


def touchdowns(sw):
    """Per step: the swing foot's (x, y) relative to the stance foot and the two hip yaws, from the model's own
    keyframes at the touchdown frame (the foot z may differ: a press)."""
    out = []
    for p in sw.phases:
        g = min(sw.segs[1:], key=lambda s: abs(s["t"] - p["t1"]))
        f = g["pa"]["feet"]
        sx, sy = f[p["swing"]][0] - f[p["stance"]][0], f[p["swing"]][1] - f[p["stance"]][1]
        out.append((sx, sy, g["pa"]["yaw"]["L"], g["pa"]["yaw"]["R"]))
    return out


@pytest.mark.parametrize("opt", list(OPTS))
@pytest.mark.parametrize("kind", list(PLACE))
def test_touchdown_placement_is_the_same_with_and_without_the_options(kind, opt):
    kw = {**KW, **{"dx_steps": None}, **PLACE[kind]}
    a, b = D.Dwell(**kw), D.Dwell(**kw, **OPTS[opt])
    assert [p["swing"] for p in a.phases] == [p["swing"] for p in b.phases]
    for x, y in zip(touchdowns(a), touchdowns(b)):
        assert y == pytest.approx(x, abs=1e-12)
    # and the feet's goals at the end of every step (the stance foot of the next one)
    assert a._goals == b._goals


@pytest.mark.parametrize("kind", list(PLACE))
def test_the_pause_plan_is_the_plain_plan_with_holds_for_lateral_and_turning_steps(kind):
    """Every joint of every keyframe: the plain plan's value at time t is the pause plan's at t + i P (i = steps already
    done, the first step has no pause), and in each pause nothing moves from the touchdown frame."""
    kw = {**KW, **{"dx_steps": None}, **PLACE[kind]}
    a, b = D.Dwell(**kw), D.Dwell(**kw, td_pause=P)
    k = round(P / 0.04)
    ib = {round(s["t"], 6): s for s in b.segs[1:]}
    for sa in a.segs[1:]:
        i = sum(1 for p in a.phases if sa["t"] > p["t1"] + 1e-9)            # steps finished before this frame
        t_b = round(sa["t"] + i * P, 6)
        assert ib[t_b]["q"] == pytest.approx(sa["q"], abs=1e-12), (kind, sa["t"])
    for i in range(1, N):
        td = b.phases[i - 1]["t1"]
        ref = ib[round(td, 6)]
        for j in range(1, k + 1):
            assert ib[round(td + 0.04 * j, 6)]["q"] == pytest.approx(ref["q"], abs=1e-12)
