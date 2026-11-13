"""track_host's per-step commands (--stride-mm / --side-mm / --step) and sequences (--seq): the defaults stay
byte-identical to main's plans, the pre-check refuses what the kinematics cannot do and reports the tightest joint
margin, a sequence is one continuous track, and the rolling upload / heading hold / ankle learner work across the
segment boundaries (a fake robot, as test_ankle_learner)."""
import hashlib
import json
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.join(HERE, "..", "experiments", "roll-balance")
sys.path.insert(0, RB)
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")

# sha256[:16] of the plan's frames (the hex lines joined), 6 steps, measured on main (7bacfa3) before this change
GOLD = {"fwd@4": "8b4f90c9f19cf760", "back@4": "6b4ad889423d0880", "sideL@4": "859b049981c0de76",
        "turnL@4": "6767356c52151eff", "sideR@4": "a8e27afd6ad68ee4", "march@4": "24f85dab56ac2317",
        "turnR@4": "d56baaa56fb52797", "fwd@5": "d354d9885792adde", "back@5": "10d1b69b360030ed",
        "sideL@5": "c37cae848e5808d6", "turnL@5": "1f932551aea2fd7e"}


def _h(p):
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]


@pytest.mark.parametrize("key", sorted(GOLD))
def test_default_plans_unchanged_vs_main(key):
    d, T = key.split("@")
    assert _h(H.plan(float(T), 6, d)) == GOLD[key]


def test_explicit_defaults_equal_the_presets():
    f = _h(H.plan(4.0, 6, "fwd"))
    assert _h(H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", stride_mm=20))) == f
    assert _h(H.plan(4.0, 6, "fwd", cmd=H.step_cmd(20, 0, 0))) == f
    assert _h(H.plan(4.0, 6, "back", cmd=H.step_cmd(-20, 0, 0))) == _h(H.plan(4.0, 6, "back"))
    assert _h(H.plan(4.0, 6, "sideL", cmd=H.preset_cmd("sideL", side_mm=15))) == _h(H.plan(4.0, 6, "sideL"))
    assert _h(H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", stride_mm=30))) != f


def test_commands():
    c = H.preset_cmd("back", stride_mm=30)
    assert c["dx"] == pytest.approx(-0.03) and c["key"] == "back"
    assert H.preset_cmd("sideR", side_mm=10)["dy"] == pytest.approx(-0.010)
    assert H.preset_cmd("fwd")["dx"] == 0.02 and H.preset_cmd("turnL")["dpsi"] == 5.0
    s = H.parse_step("20, 10, 3")
    assert (s["dx"], s["dy"], s["dpsi"]) == (0.02, 0.01, 3.0) and s["key"] == "step(20,10,3)"
    assert s["yt"] == pytest.approx(3 / H.YAW_GAIN)
    with pytest.raises(ValueError):
        H.parse_step("20,10")
    with pytest.raises(ValueError):
        H.preset_cmd("splay")


def test_precheck_refuses_long_strides_and_reports_the_margin():
    margins = []
    for mm in (20, 26, 30, 40):
        p = H.plan(4.0, 8, "fwd", cmd=H.preset_cmd("fwd", stride_mm=mm))
        ok, rows = H.precheck(p)
        tj = next(r for r in rows if r["at"] == "tightest joint")
        margins.append(tj["ratio"])
        assert ok == (mm <= 26), (mm, tj)
        if not ok:
            assert any(r["at"] == "joint limit" and r["stance"].endswith("_ankle") for r in rows)
    assert margins == sorted(margins, reverse=True) and margins[0] == pytest.approx(3.8, abs=0.1)
    # the default plan's margin (and pass) as before: the tightest joint is the stance-leg ankle pitch, 3.8 deg in
    ok, rows = H.precheck(H.plan(4.0, 8, "fwd"))
    assert ok


def test_pelvis_shift_speed_stays_inside_the_validated_timing():
    for T in (4.0, 5.0):
        for mm in (20, 40, 50):
            p = H.plan(T, 6, "fwd", cmd=H.preset_cmd("fwd", stride_mm=mm))
            assert H.pelvis_speed(p["sway"]) <= H.PELVIS_V_MAX
    # a 100 mm stride adds fore-aft travel to the same 1.4 s shift but the shift's lateral part dominates
    p = H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", stride_mm=20))
    assert 0.2 < H.pelvis_speed(p["sway"]) < H.PELVIS_V_MAX


def test_kinematically_impossible_step_is_refused_by_main(capsys):
    rc = H.main(["--T", "4", "--step", "150,0,0", "--steps", "4"])
    out = capsys.readouterr().out
    assert rc == 2 and "REFUSED" in out


def test_main_dry_runs(capsys):
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "8", "--stride-mm", "30"]) == 0
    out = capsys.readouterr().out
    assert "stride 30 mm" in out and "REFUSED" in out and "+7.5 mm/s forward" in out and "tightest joint margin" in out
    assert H.main(["--T", "5", "--step", "20,10,0", "--steps", "6"]) == 0
    out = capsys.readouterr().out
    assert "+4.0 mm/s forward, +2.0 mm/s sideways" in out and "PASS" in out
    assert H.main(["--T", "5", "--dir", "fwd", "--steps", "6", "--seq", "fwd:2", "--step", "1,1,1"]) == 2
    assert H.main(["--T", "5", "--seq", "fwd:4,bogus:3"]) == 2


def test_speeds():
    p = H.plan(5.0, 6, "sideL")
    (vx, vy, vw), _ = H.speeds(p)
    assert (vx, vy, vw) == pytest.approx((0.0, 3.0, 0.0))
    p = H.plan(4.0, 6, "turnL")
    assert H.speeds(p)[0][2] == pytest.approx(1.25)


# ---- sequences

def _seq(txt="fwd:4,turnL:3,fwd:4"):
    return H.parse_seq(txt)


def test_parse_seq():
    s = H.parse_seq("fwd:4,turnL:3,step(20,10,0):2")
    assert [(c["name"], n) for c, n in s] == [("fwd", 4), ("turnL", 3), ("step(20,10,0)", 2)]
    assert H.parse_seq("fwd:2", stride_mm=30)[0][0]["dx"] == pytest.approx(0.03)
    for bad in ("fwd", "fwd:0", "nope:2", "fwd:x"):
        with pytest.raises(ValueError):
            H.parse_seq(bad)


def test_no_closing_step_between_segments_unless_laterally_reversed():
    cm = H.expand_seq(_seq("fwd:4,turnL:3,fwd:4,sideR:4,back:4"))
    assert [c["name"] for c in cm].count("close") == 0 and len(cm) == 19
    # sideL x2 leaves the feet apart (the leading left foot out): sideR next is a reversal -> one explicit closing step
    cm = H.expand_seq(_seq("sideL:2,sideR:2"))
    assert [c["name"] for c in cm] == ["sideL", "sideL", "close", "sideR", "sideR"]
    assert cm[2]["dx"] == cm[2]["dy"] == 0.0 and cm[2]["key"] == "sideL"
    # sideL x3 ends feet together (R followed): no closing; sideL x2 then fwd: the fwd step takes the stagger up
    assert [c["name"] for c in H.expand_seq(_seq("sideL:3,sideR:2"))].count("close") == 0
    assert [c["name"] for c in H.expand_seq(_seq("sideL:2,fwd:2"))].count("close") == 0


def test_seq_plan_is_one_track_with_one_closing_step_at_the_end():
    p = H.plan(5.0, 0, "fwd", seq=_seq())
    sw = p["sway"]
    assert p["args"]["n_steps"] == 11
    closing = [i for i, st in enumerate(sw._steps) if st.get("closing")]
    assert closing in ([], [11])            # only dwell's final closing step (a turn leaves the hips yawed)
    assert [m["name"] for m in sw.step_meta][:5] == ["1:fwd"] * 4 + ["2:turnL"]
    assert p["dpsi"] == [0.0] * 4 + [5.0] * 3 + [0.0] * 4
    # a single-segment sequence is the plain plan
    assert _h(H.plan(4.0, 0, "fwd", seq=H.parse_seq("fwd:6"))) == _h(H.plan(4.0, 6, "fwd"))
    assert _h(H.plan(4.0, 0, "turnL", seq=H.parse_seq("turnL:6"))) == _h(H.plan(4.0, 6, "turnL"))


def test_seq_changes_the_frames_after_the_boundary_only():
    a = H.plan(4.0, 0, "fwd", seq=H.parse_seq("fwd:3,back:3"))
    b = H.plan(4.0, 0, "fwd", seq=H.parse_seq("fwd:3,fwd:3"))
    t_cut = 1e3 * (a["sway"].phases[3]["t0"] - a["sway"].t_first)      # the 4th step's lift-off
    early = lambda p: [x for tm, x, _ in p["frames"] if tm < t_cut - 1500]    # noqa: E731
    assert early(a) == early(b) and a["frames"] != b["frames"]


def test_seq_precheck_covers_every_segment():
    p = H.plan(5.0, 0, "fwd", seq=_seq("fwd:2,step(30,0,0):2"))
    ok, rows = H.precheck(p)
    assert not ok and any(r["at"] == "joint limit" for r in rows)
    ok, rows = H.precheck(H.plan(5.0, 0, "fwd", seq=_seq()))
    assert ok and sum(r["at"] == "lift-off" for r in rows) >= 11


def test_heading_hold_follows_the_active_segment():
    hh = H.HeadingHold(6, [0, 0, 5, 5, 0, 0])
    assert hh.ref(4) == 10 and hh.ref(0) == 0
    assert hh.asked[2] > hh.asked[0] and hh.asked[0] == pytest.approx((0 - H.YAW_DRIFT) / H.YAW_GAIN)
    a0 = list(hh.asked)
    assert not hh.update(2, 0.0, 2)             # 2 straight steps in, heading 0 = the reference: nothing to change
    assert hh.update(4, 0.0, 4)                 # past the turn without making it: the remaining steps ask for more
    assert hh.asked[:4] == a0[:4] and hh.asked[4] > a0[4]
    # scalar: as before (the same asked everywhere)
    h1 = H.HeadingHold(4, 5.0)
    assert h1.asked == [(5.0 - H.YAW_DRIFT) / H.YAW_GAIN] * 4 and h1.ref(3) == 15.0
    assert h1.update(1, 0.0, 1) and len(set(h1.asked[1:])) == 1


def test_ankle_bank_keys_and_steps():
    p = H.plan(5.0, 0, "fwd", seq=_seq("fwd:2,sideR:2"))
    bank = H.AnkleBank(["fwd", "sideR"], path="/nonexistent/state.json")
    bank.of("sideR").A["R"] = 3.5
    steps = H.ankle_steps_for(p["sway"], bank)
    meta = p["sway"].step_meta
    for v, m, ph in zip(steps, meta, p["sway"].phases):
        assert v == bank.of(m["key"]).A[ph["stance"]]
    assert any(v == 3.5 for v in steps) and bank.of("fwd").A["R"] == 2.0


class _Fake:
    """Plays 400 ms of the queue per `track` poll; the heading never moves (imu constant); every step clean."""
    sent, plans = [], []

    def __init__(self, host=None, log=None, **_):
        self.cal = {n: (i + 1, 2048, 1) for i, n in enumerate(H.G.LEG)}
        self.head = 0
        self.c = types.SimpleNamespace(idle=lambda *_a, **_k: None)

    def _n(self):
        sw = _Fake.plans[-1]["sway"]
        return sum(1 for p in sw.phases if 1e3 * (p["t1"] - sw.t_first) <= self.head)

    def cmd(self, line, idle=8.0):
        _Fake.sent.append(line)
        if line == "imu":
            return "up 0 0 1\r\nquat 1.0 0.0 0.0 0.0\r\n"
        if line.startswith("track add"):
            return "track +4"
        if line == "track":
            self.head += 400
            n = self._n()
            return f"track {'done' if n >= len(_Fake.plans[-1]['sway'].phases) else 'play'}: play t {self.head} ms\r\n" \
                   f"{n} single supports reported\r\n"
        if line == "track steps":
            return "\r\n".join(f"TS {i + 1} {p['stance']} in 4.0 out 1.0 e +0.0 T +3.0 hike +3.0 ok"
                               for i, p in enumerate(_Fake.plans[-1]["sway"].phases[:self._n()]))
        return "ok"

    def fallen(self):
        return False

    def close(self):
        pass


def test_live_seq_rolls_across_segments_with_hold_and_learner(tmp_path, monkeypatch, capsys):
    fake = types.ModuleType("robot_io")
    fake.Robot = _Fake
    monkeypatch.setitem(sys.modules, "robot_io", fake)
    monkeypatch.setattr(H, "ANKLE_STATE", str(tmp_path / "state.json"))
    real = H.plan

    def spy(*a, **k):
        p = real(*a, **k)
        _Fake.plans.append(p)
        return p
    monkeypatch.setattr(H, "plan", spy)
    _Fake.sent.clear()
    log = tmp_path / "run.jsonl"
    rc = H.main(["--T", "4", "--seq", "fwd:2,turnL:2,back:2", "--live", "--ankle-learn", "on", "--ahead", "1",
                 "--poll", "0", "--log", str(log), "--seed", "5.2,3.4", "--heading-hold", "on"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "[1:fwd]" in out and "[2:turnL]" in out and "[3:back]" in out, out
    summ = [json.loads(l) for l in open(log) if '"summary"' in l][-1]
    segs = [s["seg"] for s in summ["steps"]]
    assert segs[0] == "1:fwd" and "2:turnL" in segs and segs[-1] == "3:back"
    assert summ["args"]["dir"] == "seq fwd:2,turnL:2,back:2" and summ["args"]["steps"] == 6
    # re-planned across boundaries (plan spy: the initial plans + one per hold / learner change), all the same seq
    assert len(_Fake.plans) > 3 and all(p["args"]["seq"] is not None for p in _Fake.plans)
    assert summ["heading"] and summ["ankle_learn"]["keys"].keys() == {"fwd", "turnL", "back"}
    # every upload line well formed, and the final plan's whole timeline went out
    assert all(re.match(r"track add ", l) for l in _Fake.sent if l.startswith("track add"))
    state = json.load(open(tmp_path / "state.json"))
    assert set(state) == {"fwd", "turnL", "back"}


# ---- what bounds the stride: the swing ankle pitch; the crouch depth and the swing tilt move it

def _max_stride(**kw):
    best = None
    for mm in range(20, 70, 2):
        try:
            p = H.plan(4.0, 8, "fwd", cmd=H.preset_cmd("fwd", stride_mm=mm), **kw)
        except ValueError:
            break
        if not H.precheck(p)[0]:
            break
        best = mm
    return best


def test_defaults_with_drop_and_tilt_unset_are_the_old_plan():
    assert _h(H.plan(4.0, 6, "fwd", drop=None, swing_tilt=0.0)) == GOLD["fwd@4"]
    assert _h(H.plan(4.0, 6, "fwd", drop=0.02)) == GOLD["fwd@4"]


def test_tilt_and_crouch_raise_the_cap():
    base = _max_stride()
    assert 26 <= base <= 28
    assert _max_stride(swing_tilt=3.0) >= 42 > base
    assert 34 <= _max_stride(drop=0.015) <= 38
    with pytest.raises(ValueError):            # a 10 mm crouch cannot lift the foot 22 mm: IK out of reach
        H.plan(4.0, 6, "fwd", cmd=H.preset_cmd("fwd", stride_mm=20), drop=0.01)


def test_main_flags(capsys):
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "8", "--stride-mm", "36", "--drop-mm", "15"]) == 0
    out = capsys.readouterr().out
    assert "crouch 15 mm" in out and "PASS" in out
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "8", "--stride-mm", "40", "--swing-tilt", "3"]) == 0
    assert "swing tilt 3 deg" in capsys.readouterr().out
    assert H.main(["--T", "4", "--dir", "fwd", "--steps", "8", "--drop-mm", "10"]) == 2
