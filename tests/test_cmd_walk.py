"""track_host --cmd (experiments/roll-balance/cmd_walk.py): live velocity commands on the dwell gait. The command
grammar and sources, the envelope / slew / reversal rules, the incremental pre-check, the late decision of a step
(frames up to a step's lift-off do not depend on it), the stop, and the live loop against a fake robot: what it
uploads is exactly the frames of the final plan."""
import json
import os
import random
import signal
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.join(HERE, "..", "experiments", "roll-balance")
sys.path.insert(0, RB)
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")
C = pytest.importorskip("cmd_walk")

BASE = dict(T_step=4.0, seeds=H.ROBOT_SEED, lead_ms=2500, cal=None, amp=H.ROBOT_AMP, model="dwell", ds=None, h=None,
            pelvis_x="stance", ankle_ff=(2.0, 2.0), k_ds=None, drop=None, lift_gate=True)


def walk(script, **kw):
    cw = C.CmdWalk(H, C.ListSource(script), BASE, first="R", say=lambda *a, **k: None, **kw)
    return cw


def _h(p):
    import hashlib
    return hashlib.sha256("\n".join(x for _, x, _ in p["frames"]).encode()).hexdigest()[:16]


def dxs(cw):
    return [round(c["dx"] * 1e3, 2) for c in cw.steps]


# ---- the grammar and the sources

def test_parse_command():
    assert C.parse_command("10 0 -1.5") == ("vel", (10.0, 0.0, -1.5))
    assert C.parse_command("  10, 0, 0  # comment") == ("vel", (10.0, 0.0, 0.0))
    assert C.parse_command("STOP") == ("stop",) and C.parse_command("pause\n") == ("pause",)
    for bad in ("", "10 0", "10 0 0 0", "a b c", "nan 0 0", "inf 0 0", "# only a comment", "go"):
        assert C.parse_command(bad) is None


def test_file_source_keeps_the_last_on_a_bad_file(tmp_path):
    f = tmp_path / "cmd"
    src = C.FileSource(str(f))
    assert src.poll() is None and "No such file" in src.error
    f.write_text("5 0 0\n")
    assert src.poll() == ("vel", (5.0, 0.0, 0.0)) and src.error is None
    f.write_text("garbage\n")
    assert src.poll() is None and "unparseable" in src.error
    f.write_text("")
    assert src.poll() is None and src.error == "empty"


def test_stdin_source_is_non_blocking_and_sees_eof():
    r, w = os.pipe()
    src = C.StdinSource(r)
    assert src.poll() is None and not src.closed          # nothing yet: no blocking
    os.write(w, b"1 2 3\nbogus\n4 5")
    assert src.poll() == ("vel", (1.0, 2.0, 3.0)) and "unparseable" in src.error
    os.write(w, b" 6\nstop\n")
    assert src.poll() == ("stop",)                         # the newest line wins
    os.close(w)
    assert src.poll() is None and src.closed
    os.close(r)


# ---- the rules

def test_envelope_scales_all_three_uniformly_and_slew_limits_the_change():
    cw = walk(["20 10 2"] * 6, limits=dict(slew=(1e9, 1e9, 1e9), side=15.0))        # (the cap the table in the README had before 25) asks 80 mm, 80 mm (10 mm/s of body = 2 x 40 per pair), 8 deg
    cw.decide()
    s = cw.log[0]["step_mm_deg"]
    f = min(50 / 80, 15 / 80, 5 / 8)
    assert s == pytest.approx([80 * f, 80 * f, 8 * f], abs=0.02)         # the sideways cap binds: one factor, same direction
    assert any("envelope" in n for n in cw.log[0]["notes"])
    # backward is capped at 32 (steering mode), forward at 50
    cw = walk(["-20 0 0"] * 6, limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.decide()
    assert cw.log[0]["step_mm_deg"][0] == pytest.approx(-32.0)
    cw = walk(["-20 0 0"] * 6, limits=dict(slew=(1e9, 1e9, 1e9), back=40.0))      # 40 stays available by flag
    cw.decide()
    assert cw.log[0]["step_mm_deg"][0] == pytest.approx(-40.0)


def test_slew_walks_through_zero_from_full_forward_to_full_back():
    cw = walk(["10 0 0"] * 4 + ["-10 0 0"] * 5 + ["stop"])
    cw.rehearse(40)
    assert dxs(cw) == [20.0, 40.0, 40.0, 40.0, 20.0, 0.0, -20.0, -32.0, -32.0, 0.0]
    assert any("slew" in n for r in cw.log for n in r["notes"])
    # all three components move together (a diagonal keeps its direction while it is slewed)
    cw = walk(["10 1.875 0"] * 3)                                         # body vy 1.875 -> dy 15 per lead step
    cw.rehearse(3)
    assert [c["dy"] / c["dx"] for c in cw.steps[:3]] == pytest.approx([15 / 40] * 3, rel=2e-3)


def test_tilt_is_chosen_from_the_stride():
    assert [C.tilt_for(x) for x in (0, 20, 26, 27, 40, 43, 44)] == [0, 0, 0, 3, 3, 3, 5]
    cw = walk(["5 0 0"] * 3 + ["10 0 0"] * 3, limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.rehearse(6)
    assert [c["tilt"] for c in cw.steps[:6]] == [0, 0, 0, 3, 3, 3]
    # the stop step after a long stride keeps the tilt (its swing foot starts that far behind)
    assert cw.steps[-1]["tilt"] == 3


def test_stop_makes_a_closing_step_and_ends_the_plan_with_the_feet_together():
    cw = walk(["10 0 0"] * 3 + ["stop"])
    recs = []
    while not cw.finished:
        recs.append(cw.decide())
    assert len(cw.steps) == 4 and cw.steps[-1]["name"] == "stop" and recs[-1]["state"] == "stopping"
    pl = H.plan(**cw.plan_kwargs())
    sw = pl["sway"]
    assert not any(st.get("closing") for st in sw._steps)                # no further closing step: this one is it
    feet = sw.segs[-1]["pa"]["feet"]
    assert feet["L"][:2] == pytest.approx(feet["R"][:2]) and sw.segs[-1]["pa"]["yaw"] == {"L": 0.0, "R": 0.0}
    ok, rows = H.precheck(pl)
    assert ok, rows
    # a stop with nothing walked is a march step; further decisions are not asked for
    cw = walk(["stop"])
    r = cw.decide()
    assert cw.finished and len(cw.steps) == 1 and r["step_mm_deg"] == [0.0, 0.0, 0.0]


def test_stop_request_and_closed_source_end_the_walk_at_the_next_step():
    cw = walk(["10 0 0"] * 5)
    cw.decide()
    cw.request_stop("SIGTERM")
    r = cw.decide()
    assert cw.finished and "SIGTERM" in r["notes"][0] and len(cw.steps) == 2
    src = C.ListSource(["10 0 0"] * 5)
    cw = C.CmdWalk(H, src, BASE, first="R", say=lambda *a, **k: None)
    cw.decide()
    src.closed = True
    cw.decide()
    assert cw.finished and "closed" in cw.reason


def test_unparseable_command_keeps_the_last():
    cw = walk(["5 0 0", "garbage", "", "5 0 junk", "stop"])
    cw.rehearse(10)
    assert dxs(cw) == [20.0, 20.0, 20.0, 20.0, 0.0]


def test_max_steps_and_pause_bounds():
    cw = walk(["10 0 0"] * 50, limits=dict(slew=(1e9, 1e9, 1e9)), max_steps=5)
    cw.rehearse(40)
    assert len(cw.steps) == 6 and "max-steps" in cw.reason and cw.steps[-1]["name"] == "stop"
    cw = walk(["pause"] * 50, limits=dict(pause=3))
    cw.rehearse(40)
    assert len(cw.steps) == 4 and "in place" in cw.reason and dxs(cw) == [0.0] * 4
    # `0 0 0` is the same march; pause 0 = no limit
    cw = walk(["0 0 0"] * 50, limits=dict(pause=0))
    cw.rehearse(20)
    assert len(cw.steps) == 21 and "dry run" in cw.reason
    t = [0.0]
    cw = C.CmdWalk(H, C.ListSource(["10 0 0"] * 9), BASE, first="R", say=lambda *a, **k: None, max_time=10,
                   clock=lambda: t.__setitem__(0, t[0] + 4.0) or t[0])
    cw.rehearse(40)
    assert "max-time" in cw.reason and 3 <= len(cw.steps) <= 5


def test_a_sideways_reversal_with_the_feet_apart_inserts_one_closing_step_and_walking_on_does_not():
    # two left steps leave the left foot 15 mm out; a small right step would have the right foot travel LEFT (to 10 mm
    # beyond where it stands): the feet are brought together first, and the command waits one step
    cw = walk(["0 3.75 0"] * 2 + ["0 -0.625 0"] * 4, limits=dict(slew=(1e9, 1e9, 1e9), side=15.0))
    cw.rehearse(6)
    names = [c["name"] for c in cw.steps]
    assert names.count("close") == 1 and names[2] == "close" and cw.steps[3]["dy"] == pytest.approx(-0.005)
    # a full reversal (15 mm right after 15 mm left) is a null out-step, not a wrong-way one: no closing step
    cw = walk(["0 3.75 0"] * 2 + ["0 -3.75 0"] * 3, limits=dict(slew=(1e9, 1e9, 1e9), side=15.0))
    cw.rehearse(5)
    assert [c["name"] for c in cw.steps].count("close") == 0
    # sideways right all the way: R goes out, L follows alongside: never a closing step (the --seq boundary rule
    # `gap x dy < 0` would have fired after every odd step here)
    cw = walk(["0 -3.75 0"] * 7, limits=dict(slew=(1e9, 1e9, 1e9), side=15.0))
    cw.rehearse(7)
    assert [c["name"] for c in cw.steps].count("close") == 0
    assert all(c["dy"] == pytest.approx(-0.015) for c in cw.steps[:7])


def test_every_admitted_step_passes_the_whole_plans_precheck_and_so_does_the_stop():
    rnd = random.Random(7)
    for k in range(4):
        script = [f"{rnd.choice([-12, -8, -3, 0, 3, 6, 10, 14]):g} {rnd.choice([-5, -2, 0, 0, 2, 5]):g} "
                  f"{rnd.choice([-2, -1, 0, 0, 1, 2]):g}" for _ in range(14)]
        cw = walk(script)
        cw.rehearse(14)
        pl = H.plan(**cw.plan_kwargs())
        ok, rows = H.precheck(pl)
        assert ok, (script, [r for r in rows if not r["ok"]])
        assert not any("WARNING" in n for r in cw.log for n in r["notes"])


def test_the_pre_check_scales_a_step_the_envelope_allows_but_the_kinematics_do_not():
    # a 50 mm stride has no room in the swing ankle at tilt 3: with the envelope raised the pre-check cuts it back
    cw = walk(["20 0 0"] * 4, limits=dict(fwd=60.0, slew=(1e9, 1e9, 1e9)), fixed_tilt=3.0)
    cw.rehearse(4)
    top = max(dxs(cw))
    assert 40 <= top < 60
    assert any("pre-check" in n for r in cw.log for n in r["notes"])


def test_the_incremental_check_is_the_whole_plan_check_of_that_step():
    cw = walk(["10 0 0"] * 4)
    cw.rehearse(4)
    for mm in (30, 43, 46):
        c = cw._cmd(mm, 0.0, 0.0, 40.0)
        c["tilt"] = 3.0
        ok, _ = cw.check(c)
        pl = H.plan(**dict(cw.plan_kwargs(), step_cmds=list(cw.steps[:-1]) + [c]))
        ok_all, rows = H.precheck(pl, only_step=len(cw.steps) - 1)
        assert ok == ok_all


# ---- a step decided late: the frames before its lift-off do not depend on it

def test_frames_up_to_the_limit_do_not_depend_on_the_undecided_step():
    cw = walk(["5 0 0"] * 3)
    for _ in range(3):
        cw.decide()
    outs = []
    for last in (C.CmdWalk._cmd(cw, 20, 0, 0, 20), C.CmdWalk._cmd(cw, -32, 5, 3, 20), C.CmdWalk._cmd(cw, 0, -15, -5, 20)):
        cw2 = walk(["5 0 0"] * 3)
        for _ in range(3):
            cw2.decide()
        cw2.steps.append(last)                    # the next step, decided differently
        pl = H.plan(**dict(cw2.plan_kwargs(), step_cmds=list(cw2.steps), n_steps=len(cw2.steps)))
        cw2.on_plan(pl)
        outs.append((pl, None))
    cw.on_plan(H.plan(**cw.plan_kwargs()))
    lim = cw.limit_ms()
    pre = [[(t, x) for t, x, k in pl["frames"] if t <= lim] for pl, _ in outs]
    base = [(t, x) for t, x, k in H.plan(**cw.plan_kwargs())["frames"] if t <= lim]
    assert base and all(p == base for p in pre)
    # and the frame after it does
    after = [[(t, x) for t, x, k in pl["frames"] if lim < t <= lim + 600] for pl, _ in outs]
    assert after[0] != after[1] != after[2]


def test_constant_command_frames_equal_the_step_run():
    for mm, tilt, slew in ((20, 0.0, None), (40, 3.0, (40.0, 15.0, 5.0)), (-30, 3.0, (40.0, 15.0, 5.0))):
        cw = walk([f"{mm / 4:g} 0 0"] * 8, limits=dict(slew=slew) if slew else {})
        cw.rehearse(6)                              # 6 steps, then the stop step
        mine = H.plan(**cw.plan_kwargs())
        ref = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(mm, 0, 0), swing_tilt=tilt, lift_gate=True)
        assert _h(mine) == _h(ref), (mm, len(mine["frames"]), len(ref["frames"]))
        assert mine["frames"][-1] == ref["frames"][-1]


# ---- the live loop against a fake robot

class _Fake:
    """Plays 400 ms of the queue per `track` poll, as test_stride_seq's. `script`: [(head ms, text)]: the command file
    is rewritten when the head passes. Every `track add` frame is kept."""
    frames, plans, head_at_decision, script, cmdfile, cw = [], [], [], [], None, None

    def __init__(self, host=None, log=None, **_):
        self.cal = {n: (i + 1, 2048, 1) for i, n in enumerate(H.G.LEG)}
        self.head = 0
        self.c = types.SimpleNamespace(idle=lambda *_a, **_k: None)

    def _n(self):
        sw = _Fake.plans[-1]["sway"]
        return sum(1 for p in sw.phases if 1e3 * (p["t1"] - sw.t_first) <= self.head)

    def cmd(self, line, idle=8.0):
        if line == "imu":
            return "up 0 0 1\r\nquat 1.0 0.0 0.0 0.0\r\n"
        if line.startswith("track add"):
            _Fake.frames += line.split()[2:]
            return "track +4"
        if line == "track":
            self.head += 400
            while _Fake.script and _Fake.script[0][0] <= self.head:
                _Fake.cmdfile.write_text(_Fake.script.pop(0)[1] + "\n")
            n = self._n()
            done = n >= len(_Fake.plans[-1]["sway"].phases) and _Fake.cw.finished
            return f"track {'done' if done else 'play'}: play t {self.head} ms\r\n{n} single supports reported\r\n"
        if line == "track steps":
            return "\r\n".join(f"TS {i + 1} {p['stance']} in 4.0 out 1.0 e +0.0 T +3.0 hike +3.0 ok"
                               for i, p in enumerate(_Fake.plans[-1]["sway"].phases[:self._n()]))
        return "ok"

    def fallen(self):
        return False

    def close(self):
        pass


def _live(tmp_path, monkeypatch, capsys, script, first_cmd="0 0 0", args=(), hook=None):
    fake = types.ModuleType("robot_io")
    fake.Robot = _Fake
    monkeypatch.setitem(sys.modules, "robot_io", fake)
    monkeypatch.setattr(H, "ANKLE_STATE", str(tmp_path / "state.json"))
    real, real_walk = H.plan, C.CmdWalk
    cmdfile = tmp_path / "cmd"
    cmdfile.write_text(first_cmd + "\n")
    _Fake.frames, _Fake.plans, _Fake.script, _Fake.cmdfile = [], [], list(script), cmdfile

    def spy(*a, **k):
        p = real(*a, **k)
        if k.get("cal") is not None:                # the walk's own plans, not the incremental check's windows
            _Fake.plans.append(p)
        return p
    monkeypatch.setattr(H, "plan", spy)

    class Spy(real_walk):
        def __init__(s, *a, **k):
            super().__init__(*a, **k)
            _Fake.cw = s

        def decide(s):
            _Fake.head_at_decision.append(_Fake.cw_head())
            return super().decide()
    monkeypatch.setattr(C, "CmdWalk", Spy)
    _Fake.head_at_decision = []
    heads = {"h": 0}
    orig_cmd = _Fake.cmd

    def cmd(self, line, idle=8.0):
        r = orig_cmd(self, line, idle)
        heads["h"] = self.head
        return r
    monkeypatch.setattr(_Fake, "cmd", cmd)
    _Fake.cw_head = staticmethod(lambda: heads["h"])
    log = tmp_path / "run.jsonl"
    rc = H.main(["--T", "4", "--cmd", f"file:{cmdfile}", "--live", "--poll", "0", "--log", str(log), "--seed", "5.2,3.4",
                 *args])
    out = capsys.readouterr().out
    return rc, out, [json.loads(l) for l in open(log) if '"summary"' in l][-1], cmdfile


def test_live_walk_uploads_exactly_the_frames_of_the_final_plan_and_a_command_takes_effect_one_to_two_steps_later(
        tmp_path, monkeypatch, capsys):
    script = [(4000, "5 0 0"), (14000, "10 0 0"), (30000, "0 0 1.25"), (46000, "stop")]
    rc, out, summ, cmdfile = _live(tmp_path, monkeypatch, capsys, script, first_cmd="0 0 0", args=["--heading-hold", "off"])
    assert rc == 0, out
    cw = _Fake.cw
    assert cw.finished and cw.steps[-1]["name"] == "stop" and "stop command" in cw.reason
    final = H.plan(**cw.plan_kwargs())
    assert _Fake.frames == [h for _, h, _ in final["frames"]]            # nothing else was ever sent
    # command timeline: each decision read the file as it stood; a command written at head t is applied by the step
    # decided after t, i.e. at the latest one step later than the first decision that follows it
    when = [(h, r) for h, r in zip(_Fake.head_at_decision, cw.log)]
    assert when[0][1]["requested"] == [0.0, 0.0, 0.0]
    first_5 = next(r["step"] for h, r in when if r["requested"][0] == 5.0)
    first_10 = next(r["step"] for h, r in when if r["requested"][0] == 10.0)
    h5 = next(h for h, r in when if r["step"] == first_5)
    assert 4000 <= h5 <= 4000 + 4000 + 800                               # decided within a step of being written
    assert first_10 > first_5
    # the decision for step j is taken about one step before its lift-off: the head is at lift-off j-1 (+ a chunk, a poll)
    sw = final["sway"]
    for h, r in when[1:-1]:
        lo_prev = 1e3 * (sw.phases[r["step"] - 2]["t0"] - sw.t_first)
        assert lo_prev - 1e-6 <= h <= lo_prev + 1600, (r["step"], h, lo_prev)
    # what the summary says
    assert summ["cmd_walk"]["reason"] == "stop command" and summ["args"]["dir"].startswith("cmd file:")
    assert [s["state"] for s in summ["cmd_walk"]["steps"]][-1] == "stopping"
    # the status file the keys tool tails
    st = json.loads(open(str(cmdfile) + ".status").read())
    assert st["state"].startswith("ended") and "applied" in st and st["limits"]["fwd"] == 50.0
    assert "cmd step 1" in out


def test_live_walk_with_the_heading_hold_and_a_second_signal(tmp_path, monkeypatch, capsys):
    rc, out, summ, _ = _live(tmp_path, monkeypatch, capsys, [(8000, "0 0 1.25")], first_cmd="5 0 0",
                             args=["--heading-hold", "on", "--max-steps", "9"])
    assert rc == 0
    cw = _Fake.cw
    assert len(cw.steps) == 10 and "--max-steps 9" in cw.reason
    assert summ["heading"] and "heading hold" in out
    # the commanded heading rate: the hold's reference advances by the applied dyaw per step
    hold = [h for h in [cw.hold] if h][0]
    assert hold.ref(len(cw.steps)) == pytest.approx(sum(c["dpsi"] for c in cw.steps))
    assert hold.asked[-1] == 0.0                                          # the stop step asks no turn
    assert _Fake.frames == [h for _, h, _ in H.plan(**cw.plan_kwargs())["frames"]]


def test_sigint_stops_after_the_current_step(tmp_path, monkeypatch, capsys):
    sent = []
    orig = _Fake.cmd

    def cmd(self, line, idle=8.0):
        if line == "track" and self.head >= 9000 and not sent:
            sent.append(1)
            os.kill(os.getpid(), signal.SIGINT)
        return orig(self, line, idle)
    monkeypatch.setattr(_Fake, "cmd", cmd)
    rc, out, summ, _ = _live(tmp_path, monkeypatch, capsys, [], first_cmd="5 0 0", args=["--heading-hold", "off"])
    assert rc == 0 and "SIGINT" in out and _Fake.cw.finished and _Fake.cw.steps[-1]["name"] == "stop"
    assert "SIGINT" in _Fake.cw.reason and len(_Fake.cw.steps) < 8


# ---- main

def test_main_dry_run_and_refusals(tmp_path, capsys):
    f = tmp_path / "c"
    f.write_text("10 0 0\n")
    assert H.main(["--T", "4", "--cmd", f"file:{f}", "--dry-steps", "6"]) == 0
    out = capsys.readouterr().out
    assert "cmd walk:" in out and "dry run:" in out and "PASS" in out and "slew" in out
    assert "cmd step 6" in out and "stopping" in out
    f.write_text("stop\n")
    assert H.main(["--T", "4", "--cmd", f"file:{f}"]) == 0 and "nothing to do" in capsys.readouterr().out
    assert H.main(["--T", "4", "--cmd", f"file:{tmp_path}/nope"]) == 2 and "no readable command" in capsys.readouterr().out
    assert H.main(["--T", "4", "--cmd", "bogus"]) == 2
    f.write_text("0 0 0\n")
    assert H.main(["--T", "4", "--cmd", f"file:{f}", "--step", "20,0,0"]) == 2
    assert H.main(["--T", "4", "--cmd", f"file:{f}", "--ahead", "0.5"]) == 2


def test_default_plans_are_untouched_by_the_per_step_tilt():
    d = H.plan(4.0, 6, "fwd", swing_tilt=3.0)
    e = H.plan(4.0, 6, "fwd", cmd=H.step_cmd(20, 0, 0), swing_tilt=3.0)
    f = H.plan(4.0, 6, "fwd", step_cmds=[dict(H.step_cmd(20, 0, 0), seg=0, tilt=3.0)] * 6)
    assert _h(d) == _h(e) == _h(f)


# ---- the key controller (no tty needed)

def test_walk_keys_controller_and_atomic_file(tmp_path):
    import walk_keys as K
    ctl = K.Controller(T=4.0)
    for k in "wwA":
        assert ctl.key(k)
    assert ctl.v == [10.0, 5.0, 0.0] and C.parse_command(ctl.line()) == ("vel", (2.5, 1.25, 0.0))
    for k in "eeee":
        ctl.key(k)
    assert ctl.v[2] == -4.0
    for _ in range(20):
        ctl.key("w")
    assert ctl.v[0] == 50.0                            # never past the envelope
    for _ in range(20):
        ctl.key("s")
    assert ctl.v[0] == -32.0
    assert not ctl.key("z") and ctl.key("x") and ctl.line() == "0 0 0\n"
    assert ctl.key("\x1b") and ctl.line() == "stop\n"
    f = tmp_path / "cmd"
    K.write_atomic(str(f), "1 2 3\n")
    K.write_atomic(str(f), "stop\n")
    assert f.read_text() == "stop\n" and [p.name for p in tmp_path.iterdir()] == ["cmd"]
    assert K.read_status(str(f)) is None
    (tmp_path / "cmd.status").write_text(json.dumps(dict(state="walking", step=3, applied=[5.0, 0, 0], notes=["slew"])))
    assert "walking step 3 applied +5.00" in K.render(ctl, K.read_status(str(f)))


def test_live_walk_with_plain_side_timing_uploads_exactly_the_frames_of_the_final_plan(tmp_path, monkeypatch, capsys):
    """A sideways step's own pause / shift follow its displacement, so the first undecided step's frames are held back from
    the touchdown of the step before it (CmdWalk.limit_ms): nothing uploaded may differ from the final plan."""
    script = [(6000, "0 1.25 0"), (22000, "5 0 0"), (34000, "0 -1.25 0"), (46000, "stop")]
    rc, out, summ, cmdfile = _live(tmp_path, monkeypatch, capsys, script, first_cmd="0 0 0",
                                   args=["--heading-hold", "off", "--td-pause", "0.3", "--side-timing", "plain"])
    assert rc == 0, out
    cw = _Fake.cw
    assert cw.finished and cw.side_plain and any(c.get("lat") for c in cw.steps)
    final = H.plan(**cw.plan_kwargs())
    assert _Fake.frames == [h for _, h, _ in final["frames"]]
    assert "side timing plain" in out


def test_sideways_command_is_the_bodys_velocity_a_sidestep_pair_moves_it_by_the_lead_dy():
    """vy 1.25 mm/s at T 4 = 5 mm of body per step = 10 mm per pair = dy 10 on the lead step; the cap is per pair."""
    cw = walk(["0 1.25 0"] * 4, limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.rehearse(4)
    assert [round(c["dy"] * 1e3, 2) for c in cw.steps[:4]] == [10.0] * 4
    assert cw.log[0]["applied"][1] == pytest.approx(1.25)                # reported back as the body's velocity
    cw = walk(["0 5 0"] * 2, limits=dict(slew=(1e9, 1e9, 1e9)))
    cw.decide()
    assert cw.log[0]["step_mm_deg"][1] == pytest.approx(25.0)            # the default cap: 25 mm per pair = 3.1 mm/s at T 4 (robot: clean to 25)
    cw = walk(["0 5 0"] * 2, limits=dict(slew=(1e9, 1e9, 1e9), side=30.0))
    cw.decide()
    assert cw.log[0]["step_mm_deg"][1] == pytest.approx(30.0)
    # the feet's geometry: the first swing foot is R, so a right sidestep leads with it: the body moves dy / 2 in the lead
    # step; a left one does not (the foot lands alongside the stance foot)
    cw = walk(["0 0 0"])
    assert cw._body_dy(0, -10.0) == pytest.approx(-5.0) and cw._body_dy(0, 10.0) == pytest.approx(0.0)


def test_side_cap_flag_up_to_30_and_refused_beyond(capsys):
    assert H.main(["--cmd", "stdin", "--max-side-mm", "31"]) == 2
    assert "up to 30" in capsys.readouterr().out


def test_trim_tlim_sends_the_firmwares_own_gains_with_only_the_limit_changed():
    status = ("aborts   none since the last read\r\n"
              "gains    kp 0.250  ki 1.000 /s  lim 12.0 deg  tlim 14.0 deg  fade 0.60 s  abort 25.0 deg\r\n"
              "learned  T L +7.40 deg  T R +3.40 deg  (hike convention: + lifts the other hip)\r\n"
              "learning |e| <= 4.0 deg; past 8.0 deg, an abort or a clamped hike puts T back: last kept\r\n")
    assert H.trim_set_line(status, 6.9) == "trim set 0.250 1.000 12.0 6.9 0.60 25.0 4.0 8.0"
    assert H.trim_set_line("garbage", 6.9) is None
