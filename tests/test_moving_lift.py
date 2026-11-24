"""The moving lift (#297, 2026-11-24; experiments/roll-balance/README "The moving lift"): dwell's moving_lift and
track_host's --moving-lift. Off, the frames are the ones the tool made before; on, the lift-off comes moving_lift s
before the shift ends, the shift's own min-jerk runs on into the raise, and the pre-check judges the pelvis's
divergent component at lift-off."""
import hashlib
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")
D = pytest.importorskip("dwell")

# the robot's held Variant M (2026-11-16, 3.47 s) without its settle, and with it
VM = dict(td_pause=0.3, shift_s=1.3, raise_s=0.77, move_s=0.45, lower_s=0.55)
KW = dict(T=4.0, n_steps=5, first="R", pelvis_x="stance", ankle_ff=(2.0, 2.0), dt=0.02, **VM)
VM_ARGS = ["--td-pause", "0.3", "--shift-s", "1.3", "--raise-s", "0.77", "--move-s", "0.45", "--lower-s", "0.55"]


def dw(**kw):
    return D.Dwell(**{**KW, **kw})


def _h(p):
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]


def _pel(sw, t):
    return min(sw.segs[1:], key=lambda s: abs(s["t"] - t))["pa"]["pel"]


def test_off_leaves_the_frames_as_they_were():
    a, b, c = dw(settle=0.1), dw(settle=0.1, moving_lift=None), dw(settle=0.1, moving_lift=0.0)
    assert [s["q"] for s in a.segs] == [s["q"] for s in b.segs] == [s["q"] for s in c.segs]
    assert a.phases == b.phases == c.phases and a.moving_lift == 0.0 and "dcm" not in a.liftoff[0]
    base = H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", 20, None), lift_gate=True, settle=0.1, **VM)
    for ml in (None, 0.0):
        assert _h(H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", 20, None), lift_gate=True, settle=0.1, moving_lift=ml,
                         **VM)) == _h(base)
    assert "moving_lift" not in base["args"]


@pytest.mark.parametrize("ml", [0.1, 0.2, 0.3])
def test_the_lift_off_comes_before_the_shift_ends_and_the_step_is_shorter(ml):
    vm, sw = dw(settle=0.1), dw(moving_lift=ml)
    assert sw.step_time == pytest.approx(vm.step_time - 0.1 - ml)
    assert sw.t_settle == pytest.approx(-ml)
    for p, ph in zip(sw._steps, sw.phases):
        assert ph["t0"] == pytest.approx(p["b"] - ml)            # lift-off = the shift's end less the moving lift
        assert ph["t1"] - ph["t0"] == pytest.approx(1.77)        # the single support is untouched
    assert D.effective_step_s(4.0, 0.3, moving_lift=ml, **{k: v for k, v in VM.items() if k != "td_pause"}) == \
        pytest.approx(sw.step_time)


def test_the_shift_runs_on_into_the_raise_unchanged():
    # the first step (no pause): the pelvis through the shift is the settle-0 plan's, lift-off or not
    ml = 0.3
    sw, s0 = dw(moving_lift=ml), dw(settle=0.0)
    p = sw._steps[0]
    for t in [p["s0"] + 0.02 * i for i in range(int(round(1.3 / 0.02)) + 1)]:
        assert _pel(sw, t) == pytest.approx(_pel(s0, t), abs=1e-12)
    # moving out (toward the stance foot) up to the shift's end, then held
    st = 1.0 if p["stance"] == "L" else -1.0
    ts = [p["lo"] + 0.02 * i for i in range(int(round((ml + 0.3) / 0.02)))]
    v = [st * _pel(sw, t) for t in ts]
    assert all(b_ >= a_ - 1e-12 for a_, b_ in zip(v, v[1:]))
    assert v[-1] == pytest.approx(st * _pel(sw, p["b"] + 0.1), abs=1e-12)


@pytest.mark.parametrize("ml", [0.2, 0.3])
def test_the_lift_off_pelvis_is_short_but_its_dcm_reaches_k(ml):
    sw = dw(moving_lift=ml, kL=1.3, kR=1.3)
    for lo in sw.liftoff[1:]:                     # the steps from over the other foot (the first comes from the centre)
        # the min-jerk tail: 1 - mj(u) of the 2 x 1.3 x 62 mm shift is still to go at u = 1 - ml / 1.3
        u = 1.0 - ml / 1.3
        assert lo["ratio"] == pytest.approx(1.3 - 2.6 * (1.0 - D._mj(u)), abs=0.02)
        rate = 2.6 * 30 * u * u * (1 - u) ** 2 / 1.3
        assert lo["dcm"] == pytest.approx(lo["ratio"] + rate / D.OMEGA, abs=0.03)
        assert lo["ratio"] < 1.3 - 0.05 and lo["dcm"] >= 1.3


def test_the_firmware_phase_lands_on_the_moving_lift_off():
    sw = dw(moving_lift=0.2, first="R")
    off = math.pi                                     # first = R: the half cycles are shifted by pi
    for k, ph in enumerate(sw.phases):
        assert sw.phi(ph["t0"]) == pytest.approx(math.pi * (k + 0.25) + off, abs=1e-9)
        assert sw.phi(ph["t1"]) == pytest.approx(math.pi * (k + 0.75) + off, abs=1e-9)


def test_dwell_guards():
    with pytest.raises(ValueError, match="replaces the settle"):
        dw(settle=0.1, moving_lift=0.2)
    with pytest.raises(ValueError, match="pre_hike"):
        dw(moving_lift=0.2, pre_hike=(5.2, 3.4))
    with pytest.raises(ValueError, match="moving_lift"):
        dw(moving_lift=0.35 * 1.3 + 0.01)


def test_precheck_judges_the_dcm_at_lift_off():
    for kw in (dict(direction="march"), dict(direction="fwd", cmd=H.preset_cmd("fwd", 20, None))):
        pl = H.plan(4.0, 6, lift_gate=True, moving_lift=0.2, **kw, **VM)
        ok, rows = H.precheck(pl)
        ml = [r for r in rows if r["at"] == "lift-off DCM"]
        assert ok and len(ml) == len(pl["sway"].phases)
        assert all(r["ratio"] >= r["need"] - 0.01 and r["pelvis"] < r["need"] for r in ml)
        assert not [r for r in rows if r["at"] == "lift-off"]
    # a lift-off so early that the DCM is short too (0.45 s of the 1.3 s shift: DCM ~1.17) is refused
    ok, rows = H.precheck(H.plan(4.0, 6, "march", lift_gate=True, moving_lift=0.45, **VM))
    assert not ok and any(r["at"] == "lift-off DCM" and not r["ok"] for r in rows)


def test_cli(capsys):
    assert H.main(["--T", "4", "--dir", "march", "--steps", "6", *VM_ARGS, "--moving-lift", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "timeline: pause 0.30 + shift 1.30 - moving lift 0.20 + raise 0.77 + move 0.45 + lower 0.55 = 3.17 s" in out
    assert "PASS" in out and "moving lift: at lift-off the pelvis is" in out
    for extra, why in ((["--moving-lift", "0.4"], "want 0.05 to 0.25"), (["--moving-lift", "0.2", "--settle", "0.1"], "drop --settle"),
                       (["--moving-lift", "0.2", "--pre-hike", "5.2,3.4"], "pre-hike"),
                       (["--moving-lift", "0.2", "--model", "sway"], "dwell")):
        assert H.main(["--T", "4", "--dir", "march", "--steps", "6", *VM_ARGS, *extra]) == 2
        assert why in capsys.readouterr().out
