#!/usr/bin/env python3
"""Dry run of stepper.py against FakeRobot: no robot, no camera, no waiting.

    JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/stepper/dry_run.py [-q]

Lifts, a march, a forward walk, sidesteps (both ways), the board steering/stop rules (with a stand-in for
the camera) and main()'s argument handling; plus board_view on the fixture frame in testdata/. Exits
non-zero on the first failure. experiments/ is not collected by `make pytest` (it runs tests/ and pbsim),
so this is a script; its test_* functions also run under `pytest experiments/stepper/dry_run.py`.
"""
from __future__ import annotations

import contextlib
import json
import math
import os
import random
import sys
import tempfile
import time
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stepper as S  # noqa: E402
from fake_robot import FakeRobot  # noqa: E402
from kin import Kin  # noqa: E402

QUIET = "-q" in sys.argv
_orig_go = S.Run.go


def _go(self, ms, tag):
    _orig_go(self, ms, tag)
    if hasattr(self.r, "tell"):
        self.r.tell(self.feet, self.hike, self.ankle, math.degrees(self.psi_p))    # the fake body follows the commanded feet, trims and heading


_load_state = S.load_state


def _safe_state():
    prm = _load_state()
    prm["notify_cmd"] = []                           # a dry run never calls the real notify script
    return prm


@contextlib.contextmanager
def env():
    """No sleeping, state and logs in a temp dir, the fake body told where the feet are."""
    random.seed(1)
    import fake_robot
    clock = fake_robot.CLOCK
    with tempfile.TemporaryDirectory() as d, mock.patch.object(time, "sleep", lambda s: clock.__setitem__(0, clock[0] + s)), \
            mock.patch.object(time, "monotonic", lambda: clock[0]), \
            mock.patch.object(S, "STATE", os.path.join(d, "state.json")), mock.patch.object(S.Run, "go", _go), mock.patch.object(S, "load_state", _safe_state):
        yield d


def make(d, name="dry"):
    prm = S.load_state()
    kin = Kin(prm["drop"])
    run = S.Run(name, FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
    run.stand_up()
    return run


def check_up(out, n):
    assert len(out) == n, out
    assert all(o[1] == "up" for o in out), out


def test_lifts_and_march():
    with env() as d:
        run = make(d)
        out = [(s,) + run.lift(s, hold=0.0) for s in "LRLR"]
        check_up(out, 4)
        run.finish(save=False)
    with env() as d:
        run = make(d)
        check_up(run.walk("R", 4, 0.0, 0.0, hold=0.0, march=True), 4)
        assert all(f[1] == 0.0 for f in run.feet.values())
        run.finish(save=False)


def test_forward_walk():
    with env() as d:
        run = make(d)
        out = run.walk("L", 6, 0.02, 0.0)
        check_up(out, 6)
        assert abs(run.feet["L"][0] - 0.10) < 1e-9 and abs(run.feet["R"][0] - 0.12) < 1e-9, run.feet   # dx, 2dx, 2dx / 2dx x3
        assert all(f[1] == 0.0 for f in run.feet.values()), run.feet
        run.finish(save=False)


def sidestep(dy, side, steps):
    with env() as d:
        run = make(d)
        seen = []
        lift0 = run.lift

        def lift(swing, **kw):
            r = lift0(swing, **kw)
            seen.append(run.feet["L"][1] - run.feet["R"][1])      # stance width, L minus R
            return r
        run.lift = lift
        out = run.walk(side, steps, 0.0, dy)
        check_up(out, steps)
        assert all(w >= -1e-9 for w in seen), seen                # the feet never cross: stance width stays >= 0
        yl, yr = run.feet["L"][1], run.feet["R"][1]
        run.finish(save=False)
        return yl, yr


def test_sidestep_left_and_right():
    yl, yr = sidestep(0.015, "L", 4)
    assert abs(yl - 0.03) < 1e-9 and abs(yr - 0.03) < 1e-9, (yl, yr)
    yl, yr = sidestep(0.015, "L", 3)                               # left foot out twice, right once: wider, not narrower
    assert abs(yl - 0.03) < 1e-9 and abs(yr - 0.015) < 1e-9, (yl, yr)
    yl, yr = sidestep(-0.015, "R", 4)
    assert abs(yl + 0.03) < 1e-9 and abs(yr + 0.03) < 1e-9, (yl, yr)


def board_stub(run, left0=100.0, right0=100.0, ahead=lambda run: 300, script=None, miss=None, sole0=(68.0, -69.0)):
    """A camera stand-in and a body that lands its feet like the robot, in board mm. `true` is where each sole
    really stands. The stance foot never moves. A foot that goes down lands relative to the TRUE position of
    the stance foot: true[f] = true[g] + (+-sep0) + 1e3 * (commanded y of f - commanded y of g) + miss[f], where
    sep0 is the separation the robot was put down with (the IK's zero offsets) and miss[f] (mm, + = left) is
    what that swing foot adds. The look reports the true positions on one fixed scale; left0 / right0 are the
    margins at the start."""
    miss = miss or {"L": 0.0, "R": 0.0}
    run.true = {"L": sole0[0], "R": sole0[1]}
    run.start = dict(run.true)
    run.sep0 = sole0[0] - sole0[1]
    run.landed = []
    lower0 = run.lower_foot

    def lower(swing, fast=False):
        r = lower0(swing, fast)
        g = "R" if swing == "L" else "L"
        sgn = 1.0 if swing == "L" else -1.0
        run.true[swing] = run.true[g] + sgn * run.sep0 + 1e3 * (run.fy(swing) - run.fy(g)) + miss[swing]
        run.landed.append((swing, run.true[swing]))
        return r
    run.lower_foot = lower
    calls = []

    def sole(f):
        return run.true[f]

    def look(tag):
        calls.append(tag)
        if script is not None:
            m = script(len(calls))
        else:
            t, s0 = run.true, run.start
            m = dict(left_mm=left0 - (t["L"] - s0["L"]), right_mm=right0 + (t["R"] - s0["R"]), ahead_px=ahead(run), mm_per_px=1.0,
                     sole_l_mm=t["L"], sole_r_mm=t["R"], sep_mm=t["L"] - t["R"], conf=0.9)
        run.log("board", tag=tag, seen=m is not None)
        return m
    run.look = look
    run.sole = sole
    return calls


def test_board_steers_toward_the_middle():
    with env() as d:
        run = make(d)
        run.p["board_centre"] = True
        board_stub(run, left0=130.0, right0=70.0)                  # 30 mm more room on the left: go left
        out = run.walk("L", 6, 0.0, 0.0, board=True)
        check_up(out, 6)
        e = 0.5 * ((130 - (run.true["L"] - 68.0)) - (70 + (run.true["R"] + 69.0)))
        assert abs(e) <= run.p["board_dead_mm"] + 1e-6, e          # it came back inside the dead band
        assert run.feet["L"][1] > 0.0 and run.feet["R"][1] > 0.0, run.feet
        assert run.stop_why is None
        run.finish(save=False)
    with env() as d:
        run = make(d)
        run.p["board_centre"] = True
        board_stub(run, left0=70.0, right0=130.0)                   # more room on the right: the line moves right
        out = run.walk("L", 6, 0.0, 0.0, board=True)
        check_up(out, 6)
        assert run.feet["L"][1] < 0.0 and run.feet["R"][1] < 0.0, run.feet
        run.finish(save=False)


def test_board_stops():
    for name, kw, steps in (("edge", dict(left0=20.0, right0=90.0), 0), ("ahead", dict(ahead=lambda r: 40), 0)):
        with env() as d:
            run = make(d)
            board_stub(run, **kw)
            out = run.walk("L", 4, 0.02, 0.0, board=True)
            assert out == [] and run.stop_why, (name, out)
            run.finish(save=False)
    with env() as d:                                                # runs out of board while walking
        run = make(d)
        board_stub(run, ahead=lambda r: int(250 - 1e3 * r.feet["L"][0] * 1.5))
        out = run.walk("L", 10, 0.02, 0.0, board=True)
        check_up(out, len(out))
        assert 0 < len(out) < 10 and "ahead" in run.stop_why, (out, run.stop_why)
        run.finish(save=False)
    with env() as d:                                                # one miss is tolerated, two stop
        run = make(d)
        ok = dict(left_mm=100.0, right_mm=100.0, ahead_px=300, mm_per_px=0.5, sep_mm=137.0, sole_l_mm=68.0, sole_r_mm=-69.0, conf=0.9)
        board_stub(run, script=lambda n: None if n in (2, 4, 5) else ok)
        out = run.walk("L", 8, 0.02, 0.0, board=True)
        check_up(out, 4)
        assert "not seen" in run.stop_why, run.stop_why
        run.finish(save=False)


def test_look_never_raises():
    with env() as d:
        run = make(d)
        run.latest = os.path.join(d, "nothing.jpg")
        t = [0.0]

        def mono():
            t[0] += 0.5
            return t[0]
        with mock.patch.object(S.time, "monotonic", mono):
            assert run.look("missing") is None
        with open(run.latest, "wb") as f:
            f.write(b"not a jpeg")
        with mock.patch.object(S.time, "monotonic", mono):
            assert run.look("garbage") is None
        run.finish(save=False)


def test_main_arguments():
    with env() as d:
        rc = S.main(["walk", "--dx", "0", "--dy", "0.015", "--steps", "4", "--hold", "0"],
                    make_robot=FakeRobot, log_dir=d)
        assert rc == 0
        logs = [f for f in os.listdir(d) if f.endswith(".jsonl")]
        assert len(logs) == 1, logs
        rows = [json.loads(line) for line in open(os.path.join(d, logs[0]))]
        end = rows[-1]
        assert end["ev"] == "end" and abs(end["feet"]["L"][1] - 0.03) < 1e-9 and abs(end["feet"]["R"][1] - 0.03) < 1e-9, end
        assert not any(r["ev"] == "error" for r in rows)
        try:
            S.main(["lift", "--board"], make_robot=FakeRobot, log_dir=d)
        except SystemExit as e:
            assert e.code == 2
        else:
            raise AssertionError("--board on lift should be refused")


def landing_errors(run):
    rows = [json.loads(line) for line in open(run.base + ".jsonl")]
    return [(r["side"], r["err"]) for r in rows if r["ev"] == "landing"]


MISS = {"L": -15.0, "R": -20.0}      # mm, + = left: both swing feet land to the robot's right (R wider, L narrower)


def targets(run):
    return {f: run.bt["centre"] + (run.bt["half"] if f == "L" else -run.bt["half"]) for f in "LR"}


def test_fake_is_honest_blind_drift():
    """The fake reproduces the robot: with no loop and no comp a march drifts right ~35 mm per pair."""
    with env() as d:
        run = make(d)
        board_stub(run, miss=MISS)
        out = run.walk("L", 6, 0.0, 0.0, hold=0.0, march=True)           # blind: no --board
        check_up(out, 6)
        shift = [run.true[f] - run.start[f] for f in "LR"]
        assert -115 < shift[1] < -95 and -95 < shift[0] < -75, shift     # about -35 mm per pair (the first L swing only -15)
        sep = run.true["L"] - run.true["R"]
        assert abs(sep - run.sep0) < 21.0                                # the stance only wobbles by the misses
        run.finish(save=False)


def run_march(steps, miss, comp=None, gain=0.5, walk=False, left0=300.0, right0=300.0, sole0=(68.0, -69.0), **over):
    with env() as d:
        prm = S.load_state()
        prm["lat_gain"] = gain
        prm.update(over)
        if comp:
            prm["comp"] = dict(comp)
        kin = Kin(prm["drop"])
        run = S.Run("m", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        board_stub(run, left0=left0, right0=right0, miss=miss, sole0=sole0)
        trace = []
        lower0 = run.lower_foot

        def lower(swing, fast=False):
            r = lower0(swing, fast)
            tg = targets(run) if run.bt else {"L": run.true["L"], "R": run.true["R"]}
            trace.append((swing, run.true["L"] - tg["L"], run.true["R"] - tg["R"], 0.5 * (run.true["L"] + run.true["R"]) - (run.bt["centre"] if run.bt else 0.0), run.true["L"] - run.true["R"]))
            return r
        run.lower_foot = lower
        out = run.walk("L", steps, 0.02 if walk else 0.0, 0.0, hold=0.0, board=True, march=not walk)
        check_up(out, steps)
        res = dict(trace=trace, comp=dict(run.p["comp"]), run=run, errs=landing_errors(run))
        run.finish(save=False)
        return res


def test_landing_loop_marches():
    r = run_march(12, MISS)
    tr = r["trace"]                                       # after each landing: (foot, L err, R err, centre err), mm
    for i, (f, el, er, c, _sep) in enumerate(tr):
        if i >= 6:                                        # by step 6 both soles are within 8 mm of their targets, and stay
            assert abs(el) < 8.0 and abs(er) < 8.0, (i, tr[i])
    assert abs(r["comp"]["R"] - 0.020) < 0.003 and abs(r["comp"]["L"] - 0.015) < 0.003, r["comp"]
    errs = [abs(e) for sd, e in r["errs"] if sd == "R"]
    assert errs[0] > 15 and errs[-1] < 5, errs                                # the right foot's landing error halves each time
    # starting from a saved comp: within 8 mm from the first pair
    r = run_march(6, MISS, comp={"L": 0.015, "R": 0.020})
    assert all(abs(t[1]) < 8.0 and abs(t[2]) < 8.0 for t in r["trace"]), r["trace"]
    # with the misses 1.5x (30 / 22.5 mm) it still converges, slower, without a growing oscillation. (min_sep_cmd
    # leaves 25 mm of narrowing for the right foot: this run lets it have 30. The IK's reach for the left foot ends
    # near +30 mm: doubled misses, 40 / 30 mm, do not fit the leg geometry at all.)
    r = run_march(16, {"L": -22.5, "R": -30.0}, min_sep_cmd=91.0)
    for f in "LR":
        e = [abs(x) for sd, x in r["errs"] if sd == f]
        assert e[-1] < 5.0 and max(e[1:]) <= e[0] + 1e-6, (f, e)
        assert all(b <= a + 2.0 for a, b in zip(e, e[1:])), (f, e)              # (nearly) monotone: nothing rings
    assert abs(r["trace"][-1][1]) < 8.0 and abs(r["trace"][-1][2]) < 8.0, r["trace"][-3:]
    assert abs(r["comp"]["R"] - 0.030) < 0.004 and abs(r["comp"]["L"] - 0.0225) < 0.004, r["comp"]
    # doubled (40 / 30 mm) with the default limits: the right foot's 40 mm is beyond min_sep_cmd: the soles are never
    # commanded closer than it, the comp stays inside its clamp, the run goes on (it drifts: the limit is in the way)
    r = run_march(8, {"L": -30.0, "R": -40.0})
    run = r["run"]
    assert run.true["L"] - run.true["R"] >= run.p["min_sep_cmd"] - 1e-6, run.true
    assert all(abs(r["comp"][f]) <= 0.05 for f in "LR"), r["comp"]


def test_landing_loop_walk_and_blind_comp():
    r = run_march(12, MISS, walk=True)
    assert all(abs(t[3]) < 12.0 for t in r["trace"][4:]), [round(t[3], 1) for t in r["trace"]]    # holds the line
    assert all(abs(t[1]) < 12.0 and abs(t[2]) < 12.0 for t in r["trace"][4:]), r["trace"]
    with env() as d:                                      # blind, from a saved comp: less than 8 mm per pair
        prm = S.load_state()
        prm["comp"] = {"L": 0.015, "R": 0.020}
        kin = Kin(prm["drop"])
        run = S.Run("blind", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        board_stub(run, miss=MISS)
        out = run.walk("L", 12, 0.02, 0.0)
        check_up(out, 12)
        assert all(abs(run.true[f] - run.start[f]) < 8.0 * 6 for f in "LR"), run.true
        assert all(abs(run.true[f] - run.start[f]) < 8.0 for f in "LR"), run.true      # (the saved comp is exact: no drift at all)
        run.finish(save=False)
    with env() as d:                                      # the saved comp is part of the state file
        prm = S.load_state()
        prm["comp"] = {"L": 0.011, "R": 0.012}
        kin = Kin(prm["drop"])
        run = S.Run("save", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        run.finish(save=True)
        assert json.load(open(S.STATE))["comp"] == {"L": 0.011, "R": 0.012}
        assert S.load_state()["comp"] == {"L": 0.011, "R": 0.012}


def test_wide_stance_and_reach_limit():
    # put down wide (sep0 = 155): the targets keep at most sep_max_mm (131); no Abort, walked back within 6 steps
    r = run_march(12, MISS, sole0=(77.5, -77.5))
    tr = r["trace"]
    assert abs(r["run"].bt["half"] - 65.5) < 1e-9, r["run"].bt
    assert all(abs(t[4] - 131.0) < 8.0 for t in tr[5:]), [round(t[4], 1) for t in tr]
    assert all(abs(t[3]) < 12.0 for t in tr[4:]), [round(t[3], 1) for t in tr]                 # the line is held
    assert all(abs(t[1]) < 8.0 and abs(t[2]) < 8.0 for t in tr[5:]), tr
    # a robot put down narrow is not widened past sep_min_mm / 2 either
    r = run_march(2, MISS, sole0=(50.0, -50.0))
    assert abs(r["run"].bt["half"] - 57.5) < 1e-9, r["run"].bt
    # a foot sent beyond the leg's reach is pulled back, and logged; no exception
    with env() as d:
        prm = S.load_state()
        prm["comp"] = {"L": 0.05, "R": 0.0}
        prm["lat_step_max"] = 0.05
        kin = Kin(prm["drop"])
        run = S.Run("reach", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        out = run.walk("L", 4, 0.0, 0.0, hold=0.0, march=True)             # blind: comp alone sends the left foot out
        check_up(out, 4)
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        lim = [r for r in rows if r["ev"] == "reach_limit"]
        assert lim and all(r["gave_up_mm"] > 0 for r in lim) and lim[0]["side"] == "L", lim
        assert abs(run.fy("L") - run.fy("R")) < 0.05                       # it was cut back
        assert run.reachable(run.pel[1], run.pel[0], run.cmd("L"), run.cmd("R"))
        run.finish(save=False)
    with env() as d:                                      # and with balance: a shift adjustment is clamped, never raised
        run = make(d)
        run.shift_over("R")
        real = run.reachable
        run.reachable = lambda shift, fwd, fl, fr: abs(shift - run.pel[1]) < 0.003 and real(shift, fwd, fl, fr)
        run.r.h = {"L": 0.02, "R": 0.0}
        run.feet["L"][2] = 0.02
        run.read = lambda tag="": dict(roll=run.ref0["roll"] - 2.0, pitch=run.ref0["pitch"], rate=0.0, gyro=[0, 0, 0], up=[0, 0, 1])
        run.tilt_s = 0.0
        before = run.pel[1]
        run.balance("L", 0.0)                             # (secs = 0: one pass, no adjustment needed after "up")
        assert abs(run.pel[1] - before) < 0.003
        run.finish(save=False)


def test_link_drop_retry_and_stand_down():
    import types
    import robot_io
    W = robot_io.W

    class Flaky:                                          # a wcliproto.Client whose link drops `drops` times
        def __init__(self, drops):
            self.drops, self.lines, self.connects = drops, [], 0

        def command(self, line, idle_timeout=0, **kw):
            self.lines.append(line)
            if line.startswith("pose") and self.drops > 0:
                self.drops -= 1
                raise W.LinkDown("robot not answering (command 257 outcome unknown)")
            return types.SimpleNamespace(status="done", text="ok\r\n", refused=False, seq=1)

        def connect(self, *a, **kw):
            self.connects += 1

        def close(self): pass
        def idle(self, t): pass

    def robot(drops, retries=3):
        r = robot_io.Robot.__new__(robot_io.Robot)
        r.c, r.link_retries, r.link_wait, r.log, r.t0 = Flaky(drops), retries, 2.0, None, 0.0
        r.events = []
        r.on_link = r.events.append
        return r
    with mock.patch.object(robot_io.time, "sleep", lambda s: None):
        r = robot(2)
        r.cmd("pose 1 2 3 s500")                              # two drops, then it gets through
        assert r.c.lines.count("pose 1 2 3 s500") == 3 and r.c.lines.count("imu") == 2, r.c.lines     # re-read the IMU, re-sent the pose
        assert [e["ev"] for e in r.events].count("down") == 2 and r.events[-1]["ev"] == "reconnected", r.events
        assert r.c.connects == 2
        r = robot(9)
        try:
            r.cmd("pose 1 2 3 s500")
        except W.LinkDown:
            assert r.c.lines.count("pose 1 2 3 s500") == 4                    # the first try and 3 retries
        else:
            raise AssertionError("a link that stays down must raise")
        r = robot(0)
        r.cmd("torque")
        assert r.c.lines == ["torque"]
    # the run: the stand-down gets a fresh connection; and says so plainly if there is none
    class DownRobot(FakeRobot):
        down = 0

        def pose(self, q, ms, wait=True, margin=0.0):
            if self.down > 0:
                self.down -= 1
                raise robot_io.LinkDown("robot not answering")
            return super().pose(q, ms, wait, margin)

        def fresh(self):
            self.down = 0
            self.fresh_calls = getattr(self, "fresh_calls", 0) + 1
    import io
    with env() as d:
        prm = S.load_state()
        kin = Kin(prm["drop"])
        rob = DownRobot(kin)
        run = S.Run("link", rob, kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        run.walk("L", 2, 0.02, 0.0)
        rob.down = 1                                       # the link drops for the stand-down pose
        run.finish(save=False)
        assert rob.fresh_calls == 1
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        assert any(r["ev"] == "link" and r.get("what") == "fresh_connection" for r in rows) and any(r["ev"] == "link" and r.get("what") == "stand_down_failed" for r in rows)
    with env() as d:
        prm = S.load_state()
        kin = Kin(prm["drop"])

        class Dead(DownRobot):
            def fresh(self):
                pass
        rob = Dead(kin)
        run = S.Run("link2", rob, kin, prm, quiet=True, log_dir=d)
        run.stand_up()
        rob.down = 99
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            run.finish(save=False)
        assert "torque is still ON" in buf.getvalue() and "LINK DOWN" in buf.getvalue(), buf.getvalue()
        assert any('"left_torqued"' in line for line in open(run.base + ".jsonl"))


def test_turning_kinematics():
    """The facts the turning rests on (checked against design_gates / v6_kin, 2026-11-11)."""
    import numpy as np
    import design_gates as G
    k = Kin(0.02)

    def q(**kw):
        key = k.k1.copy()
        for a, b in kw.items():
            setattr(key, a, b)
        v = G.q_of(k.p, key)
        return {n: float(v[G.JN.index(n)]) for n in G.JN}
    q0 = q()
    assert abs(q(yawL=0.1)["L_hip_yaw"] - q0["L_hip_yaw"] - 0.1) < 1e-9           # a sole yawed +a (CCW): hip_yaw +a
    assert abs(q(heading=0.2)["L_hip_yaw"] - q0["L_hip_yaw"] + 0.2) < 1e-9        # a pelvis turned +a, feet fixed: hip_yaw -a
    base = k.pose(shift=0.01, fwd=0.005, footL=(0.01, 0.002, 0.0), footR=(-0.01, 0.0, 0.0))
    for ref in (0.3, -0.5):                                                       # rotating everything rigidly: no joint moves
        turned = k.pose(shift=0.01, fwd=0.005, footL=(0.01, 0.002, 0.0), footR=(-0.01, 0.0, 0.0), ref=ref, heading=ref, yawL=ref, yawR=ref)
        assert max(abs(turned[n] - base[n]) for n in base) < 1e-9, ref
    plain = k.pose(shift=0.01)                                                    # with all of it 0 the pose is the old one
    assert k.pose(shift=0.01, ref=0.0, heading=0.0, yawL=0.0, yawR=0.0) == plain
    assert np.isfinite(list(k.pose(shift=0.02, ref=0.1, heading=0.15, yawL=0.2, yawR=0.2).values())).all()


def heading_fake(d, slip_per_landing=0.0):
    prm = S.load_state()
    kin = Kin(prm["drop"])
    rob = FakeRobot(kin)
    run = S.Run("turn", rob, kin, prm, quiet=QUIET, log_dir=d)
    run.stand_up()
    lower0 = run.lower_foot

    def lower(swing, fast=False):
        r = lower0(swing, fast)
        rob.slip += slip_per_landing                      # each landing yaws the body on its own
        return r
    run.lower_foot = lower
    return run, rob


def test_turn_and_hold_heading():
    with env() as d:                                      # 30 deg over 4 pairs, the fused yaw drifting at -1 deg/s
        run, rob = heading_fake(d)
        out = run.walk("L", 8, 0.0, 0.0, hold=0.0, march=True, dyaw=7.5)
        check_up(out, 8)
        run.centre()
        assert abs(rob.psi_deg + rob.slip - 30.0) < 3.0, (rob.psi_deg, rob.slip)
        assert abs(run.heading_now() - (rob.psi_deg + rob.slip)) < 1.0, (run.heading_now(), rob.psi_deg)
        assert max(abs(math.degrees(v)) for v in run.yawf.values()) <= 30.0 + 1e-6
        run.finish(save=False)
    with env() as d:                                      # a turn of 3 pairs to the right, walking
        run, rob = heading_fake(d)
        out = run.walk("L", 6, 0.02, 0.0, dyaw=-8.0)
        check_up(out, 6)
        run.centre()
        assert abs(rob.psi_deg + rob.slip + 24.0) < 3.0, rob.psi_deg
        run.finish(save=False)
    with env() as d:                                      # a straight walk, the body yawing 2 deg on its own every step
        run, rob = heading_fake(d, slip_per_landing=2.0)
        trace = []
        lift0 = run.lift

        def lift(swing, **kw):
            r = lift0(swing, **kw)
            trace.append(rob.psi_deg + rob.slip)
            return r
        run.lift = lift
        out = run.walk("L", 12, 0.02, 0.0)
        check_up(out, 12)
        assert all(abs(h) <= 5.0 for h in trace) and all(abs(h) <= 4.0 for h in trace[6:]), [round(h, 1) for h in trace]   # (24 without the loop)
        run.finish(save=False)
    with env() as d:                                      # the same with the loop off: the body walks away
        run, rob = heading_fake(d, slip_per_landing=2.0)
        run.p["yaw_gain"] = 0.0
        run.walk("L", 12, 0.02, 0.0)
        assert abs(rob.psi_deg + rob.slip) > 20.0
        run.finish(save=False)
    with env() as d:                                      # --board: a heading past yaw_board_max ends the run
        run, rob = heading_fake(d, slip_per_landing=2.0)
        run.p["yaw_gain"] = 0.0
        board_stub(run, left0=300.0, right0=300.0)
        out = run.walk("L", 20, 0.0, 0.0, hold=0.0, board=True, march=True)
        assert run.stop_why and "heading" in run.stop_why and 8 <= len(out) <= 14, (len(out), run.stop_why)
        run.finish(save=False)


def test_step_budget():
    """Where a step's time goes, on the fake's virtual clock (a round trip: IMU 94 ms, a pose 100 ms + its move +
    the margin). Round 8 cut it from ~14.9 s per step at pace 1.0 (41 IMU reads, 13.5 poses at 0.15 s margin, settle
    waits after every shift and landing)."""
    import fake_robot
    with env() as d:
        run = make(d)
        reads = [0]
        imu0 = run.r.imu

        def imu():
            reads[0] += 1
            return imu0()
        run.r.imu = imu
        t0, r0 = fake_robot.CLOCK[0], reads[0]
        out = run.walk("L", 4, 0.02, 0.0)
        check_up(out, 4)
        per = (fake_robot.CLOCK[0] - t0) / 4
        n = (reads[0] - r0) / 4
        print(f"   step budget: {per:.1f} s per step, {n:.0f} IMU reads per step")
        assert per < 14.0 and n < 30, (per, n)
        run.finish(save=False)


def test_turn_bookkeeping_invariants():
    """A 90 deg turn in 12 pairs. Re-basing the frame is a change of coordinates: the pose sent must not change by a hair
    (the round-8 code rotated the feet's OFFSETS, not their positions: its pose jumped by 0.0145 rad at each re-base,
    cumulative, and the bookkeeping walked off: the 261 mm 'shift' of the 161649 fall). In the stance foot's own frame
    the pelvis stays k_shift * oy from it and the swing foot where it was placed, at every heading."""
    with env() as d:
        run = make(d)
        jumps, rels, shifts = [], [], []

        def pose_now():
            return run.k.pose(shift=run.pel[1], fwd=run.pel[0], footL=run.cmd("L"), footR=run.cmd("R"), drop=run.p["drop"],
                              ref=run.psi_ref, heading=run.psi_p, yawL=run.yawf["L"], yawR=run.yawf["R"])
        rb0, so0 = run.rebase, run.shift_over

        def rebase(dd):
            q1 = pose_now()
            rb0(dd)
            q2 = pose_now()
            jumps.append(max(abs(q1[n] - q2[n]) for n in q1))
        run.rebase = rebase

        def shift_over(stance):
            r = so0(stance)
            sw = "R" if stance == "L" else "L"
            base = {"L": run.k.k1.footL[:2], "R": run.k.k1.footR[:2]}
            rels.append([base[sw][i] + run.feet[sw][i] - base[stance][i] - run.feet[stance][i] for i in (0, 1)])
            shifts.append((run.pel[1] - run.fy(stance), run.p["k_shift"][stance] * run.off[stance][1]))
            return r
        run.shift_over = shift_over
        out = run.walk("L", 24, 0.0, 0.0, hold=0.0, march=True, dyaw=7.5)
        check_up(out, 24)
        assert 80.0 < math.degrees(run.psi_ref) < 92.0, math.degrees(run.psi_ref)
        assert len(jumps) >= 20 and max(jumps) < 1e-9, max(jumps)                            # (0.0145 before)
        assert all(abs(a - b) < 1e-3 for a, b in shifts), shifts[:3]                        # the pelvis k * oy from the stance foot
        assert max(math.hypot(r[0], abs(r[1]) - 0.084) for r in rels) < 0.012, [round(1e3 * r[0], 1) for r in rels]   # the swing foot stays beside it
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        assert all(abs(r["shift"]) < 120 for r in rows if r["ev"] == "air"), [r["shift"] for r in rows if r["ev"] == "air"][:5]
        run.finish(save=False)


def test_sole_levelling_fk():
    """The swing sole's world normal is vertical (within 0.5 deg) after the trims, whatever roll / pitch the body
    hangs at: by forward kinematics on the MuJoCo model kin.py loads (signs derived there, not argued)."""
    import numpy as np
    import design_gates as G
    import v6_kin as K
    from kin import OL
    k = Kin(0.02)
    m, md = OL.K.load(k.p)

    def quat(ax, a):
        c, s = math.cos(a / 2), math.sin(a / 2)
        return np.array([c] + [s * x for x in ax])

    def qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])

    def normal(q, side, roll_deg, pitch_deg):
        tq = qmul(quat((1, 0, 0), math.radians(roll_deg)), quat((0, 1, 0), math.radians(pitch_deg)))   # IMU roll + = right down, pitch + = forward
        K.set_pose(m, md, np.array([q[n] for n in G.JN]), torso_pos=(0, 0, 1.0), torso_quat=tuple(tq))
        return K.foot_frame(m, md, side)[1][:, 2]
    with env() as d:
        run = make(d)
        run.ref0 = dict(roll=0.0, pitch=0.0)
        run.p["sole_bias"] = {"L": 0.0, "R": 0.0}
        for side in "LR":
            for roll, pitch in ((5.0, -3.0), (-6.0, 2.0), (4.0, 4.0), (0.0, 0.0)):
                run.sole_r = {"L": 0.0, "R": 0.0}
                run.sole_p = {"L": 0.0, "R": 0.0}
                for _ in range(6):                                            # sole_rate per update: converge
                    run.level_sole(side, dict(roll=roll, pitch=pitch))
                q = k.pose(ankle={f: math.radians(run.sole_r[f]) for f in "LR"}, ankle_p={f: math.radians(run.sole_p[f]) for f in "LR"})
                n = normal(q, side, roll, pitch)
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(n[2])))))
                assert tilt < 0.5, (side, roll, pitch, tilt, run.sole_r[side], run.sole_p[side])
                untrimmed = normal(k.pose(), side, roll, pitch)
                assert roll == pitch == 0.0 or math.degrees(math.acos(float(untrimmed[2]))) > 2.0     # it was tilted before
        # the bias: the OUTER edge of either sole up by 2 deg (n_y < 0 tips the left edge up, n_y > 0 the right)
        run.p["sole_bias"] = {"L": 2.0, "R": 2.0}
        for side, sgn in (("L", -1.0), ("R", 1.0)):
            run.sole_r = {"L": 0.0, "R": 0.0}
            run.sole_p = {"L": 0.0, "R": 0.0}
            for _ in range(3):
                run.level_sole(side, dict(roll=0.0, pitch=0.0))
            q = k.pose(ankle={f: math.radians(run.sole_r[f]) for f in "LR"}, ankle_p={f: math.radians(run.sole_p[f]) for f in "LR"})
            n = normal(q, side, 0.0, 0.0)
            assert abs(sgn * float(n[1]) - math.sin(math.radians(2.0))) < 0.002, (side, n)
        # clamped and rate-limited
        run.sole_r = {"L": 0.0, "R": 0.0}
        run.level_sole("L", dict(roll=-30.0, pitch=0.0))
        assert abs(run.sole_r["L"]) <= run.p["sole_rate"] + 1e-9
        for _ in range(10):
            run.level_sole("L", dict(roll=-30.0, pitch=40.0))
        assert abs(run.sole_r["L"]) <= run.p["sole_trim_max"] + 1e-9 and abs(run.sole_p["L"]) <= run.p["sole_trim_max"] + 1e-9
        run.finish(save=False)


def test_swing_room_and_soft_landing():
    with env() as d:
        run = make(d)
        how, h = run.lift("L", hold=0.0, dx=0.02)
        assert how == "up"
        run.lift("R", hold=0.0, dx=0.04)
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        poses = [r for r in rows if r["ev"] == "pose"]
        assert all("sole_r" in r and "sole_p" in r for r in poses)
        extra = [r for r in poses if r["tag"].startswith("extra L")]
        swing = [r for r in poses if r["tag"].startswith("swing L")]
        assert extra and swing
        assert abs(swing[0]["feet"]["L"][2] - extra[0]["feet"]["L"][2]) < 1e-12 and extra[0]["feet"]["L"][2] >= h + run.p["swing_extra"] - 1e-9, (extra[0]["feet"]["L"], h)
        low = [r for r in poses if r["tag"].startswith("lower L")]
        assert low[-1]["tag"] == "lower L 0" and low[-1]["ms"] == int(1e3 * run.p["t_land"]), low[-1]
        assert low[-2]["tag"] == "lower L 6", [r["tag"] for r in low]                       # the last 6 mm in one slow move
        assert max(abs(v) for r in poses for v in list(r["sole_r"].values()) + list(r["sole_p"].values())) <= run.p["sole_trim_max"] + 1e-9
        assert any(abs(r["sole_r"]["L"]) > 0.5 for r in poses if r["tag"].startswith(("raise L", "swing L")))      # it trims in the air
        # the trims go as the next shift loads the foot
        sh = [r for r in poses if r["tag"].startswith("shift L")]
        assert sh and all(abs(v) < 1e-9 for v in sh[-1]["sole_r"].values()), sh[-1]["sole_r"]
        run.finish(save=False)


def test_catch_reflex():
    """The robot starts to go over toward the lifted foot when it is lifted at too small a shift (the roll rate grows
    until the foot is back on the floor). The reflex must put the foot down flat, fast, before the fake "falls" (the foot
    not flat on the floor 0.6 s after the verdict); k rises; the next attempt succeeds."""
    import fake_robot
    clock = fake_robot.CLOCK
    with env() as d:
        run = make(d)
        st = dict(t0=None, caught=False, fell=None, verdict=None, touch=None)
        orig_read = run.read
        sw, stn = "L", "R"                                    # lift L over R

        def read(tag=""):
            s = dict(orig_read(tag))
            ratio = abs(run.pel[1] - run.fy(stn)) / abs(run.off[stn][1])
            up = run.feet[sw][2] > 0.003
            if up and ratio < 1.15 and not st["caught"]:
                if st["t0"] is None:
                    st["t0"] = clock[0]
                dt = clock[0] - st["t0"]
                tilt, rate = 0.5 * 12.0 * dt * dt, 12.0 * dt          # deg, deg/s toward the swing foot (L: roll -)
                s["roll"] -= tilt
                s["gyro"] = [-math.radians(rate), s["gyro"][1], s["gyro"][2]]
                s["rate"] = rate
                st["now_tilt"] = tilt
            elif not up:
                st["t0"] = None
            run.last_imu = s                                     # what the stepper last saw, tilt and all
            return s
        run.read = read
        raise0 = run.raise_foot

        def raise_foot(swing):
            r = raise0(swing)
            if r[0] in ("tipping_in", "fell_in", "tipping"):
                st["verdict"] = clock[0]
                st["tilt_at"] = st.get("now_tilt", 0.0)
            return r
        run.raise_foot = raise_foot
        go0 = run.go

        def go(ms, tag):
            go0(ms, tag)
            if tag.startswith("catch") and not tag.startswith("catch back") and st["touch"] is None:
                st["touch"] = (run.feet[sw][2], run.sole_r[sw], st["tilt_at"], clock[0] - st["verdict"])
                st["caught"] = True                                  # flat on the floor: it stops going over
            if st["verdict"] is not None and st["touch"] is None and clock[0] - st["verdict"] > 0.6 and run.feet[sw][2] > 0.003 and st["fell"] is None:
                st["fell"] = clock[0] - st["verdict"]
        run.go = go
        k0 = run.p["k_shift"][stn]
        how, h = run.lift(sw, hold=0.0)
        assert st["verdict"] is not None, "the fake never tipped"
        assert st["fell"] is None, st
        z, trim, tilt, dt = st["touch"]
        assert z == 0.0 and dt < 0.6, st
        assert abs(trim - tilt) < 2.0, (trim, tilt)                           # the left sole's trim is -(body roll), and the body rolled left by `tilt`
        assert run.p["k_shift"][stn] >= k0 + run.p["k_step"] - 1e-9 or how == "up"
        assert how == "up", how
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        lifts = [r for r in rows if r["ev"] == "lift"]
        assert lifts[0]["how"] in ("tipping_in", "fell_in") and lifts[-1]["how"] == "up", [r["how"] for r in lifts]
        cat = [r for r in rows if r["ev"] == "catch"]
        assert cat and cat[0]["swing"] == "L" and cat[0]["out_mm"] > 0, cat                 # a wider base: L goes further left
        catch_pose = [r for r in rows if r["ev"] == "pose" and r["tag"].startswith("catch L") and "back" not in r["tag"]]
        assert catch_pose and catch_pose[0]["ms"] == int(1e3 * run.p["t_catch"]) and catch_pose[0]["feet"]["L"][2] == 0.0
        run.finish(save=False)
    assert S.DEFAULTS["k_floor"] == 1.1 and S.DEFAULTS["tip_in_margin"] == 2.5 and S.DEFAULTS["tip_in_rate"] == 12.0
    with env() as d:                                      # the catch does not depend on --pace (t_catch is not scaled)
        prm = S.apply_pace(S.load_state(), 0.6)
        assert prm["t_catch"] == 0.2


def test_shift_tip_guard():
    """Past 60 mm of shift over the left foot the body leans TOWARD it (the foot is tipping over its outer edge):
    the guard backs the pelvis off, lowers k, and the lift goes on from there."""
    with env() as d:
        run = make(d)
        orig = run.read

        def read(tag=""):
            s = dict(orig(tag))
            y = 1e3 * run.pel[1]
            if y > 1:
                s["roll"] += 0.03 * y - (0.6 * (y - 60.0) if y > 60.0 else 0.0)
            return s
        run.read = read
        how, h = run.lift("R", hold=0.0)
        assert how == "up", how
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        tips = [r for r in rows if r["ev"] == "shift_tip"]
        assert tips and tips[0]["stance"] == "L", tips
        t = tips[0]
        assert t["toward"] > run.p["shift_out_lim"] and abs((t["shift_mm"] - t["back_to_mm"]) - run.p["shift_back_mm"]) < 1e-6, t
        assert abs(t["k"] - (t["k_old"] - run.p["k_step"])) < 1e-9, t
        assert not any(r["ev"] == "error" for r in rows)
        run.finish(save=False)


def test_k_cannot_ratchet():
    with env() as d:
        run = make(d)
        P = run.p
        P["k_shift"]["L"] = 1.0
        ks = [P["k_shift"]["L"]]
        for _ in range(20):                       # balance always had to push: and the body always wanted k above the ceiling
            ks.append(run.learn_k("L", 1.9, pushed=True))
        assert all(b - a <= 0.5 * P["k_step"] + 1e-9 for a, b in zip(ks, ks[1:])), ks      # at most half a step per lift
        assert max(ks) <= P["k_max"] + 1e-9 and ks[-1] == P["k_max"], ks
        # lift-off lower than the old value: it follows it down at once, not the push
        assert run.learn_k("L", 1.2, pushed=True) == 1.2
        # no push needed: decays 0.01 per lift to the floor (1.1) and stays
        seq = [run.learn_k("L", P["k_shift"]["L"], pushed=False) for _ in range(15)]     # (lift-off is where k put it)
        assert abs(seq[0] - 1.19) < 1e-9 and seq[-1] == P["k_floor"] and seq[-1] == seq[-2] == 1.1, seq
        assert min(seq) >= P["k_floor"]
        assert P["k_max"] == 1.4
        run.finish(save=False)
    with env() as d:                              # and through real lifts: the fake needs no push, k comes down toward the floor
        run = make(d)
        run.p["k_shift"]["R"] = 1.3
        run.balance = lambda swing, secs: "up"                  # a body that needs no push
        for _ in range(3):
            run.lift("L", hold=0.0)
        assert abs(run.p["k_shift"]["R"] - 1.27) < 1e-9, run.p["k_shift"]
        run.finish(save=False)


def test_landing_loop_limits():
    with env() as d:                                      # offsets that would cross the feet are held at min_sep_cmd, and capped
        prm = S.load_state()
        prm["comp"] = {"L": -0.05, "R": 0.05}
        kin = Kin(prm["drop"])
        run = S.Run("lim", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        board_stub(run)
        run.walk("L", 4, 0.0, 0.0, hold=0.0, board=True, march=True)
        assert run.true["L"] - run.true["R"] >= run.p["min_sep_cmd"] - 7.0, run.true     # the real soles keep apart
        run.finish(save=False)
    with env() as d:                                      # a sidestep command moves the line (half of dy per step) and the feet follow
        run = make(d)
        board_stub(run)
        run.walk("L", 6, 0.0, 0.015, hold=0.0, board=True, march=True)
        assert abs(run.bt["centre"] - (-0.5 + 6 * 7.5)) < 1e-6, run.bt
        assert abs(run.sole("L") - (68.0 + 5 * 7.5)) < 8.0 and abs(run.sole("R") - (-69.0 + 6 * 7.5 - 0.5 + 0.5)) < 8.0, run.true
        run.finish(save=False)
    with env() as d:                                      # a missed look: the same relative offset as last time, no learning
        run = make(d)
        board_stub(run, miss=MISS, script=lambda n: None if n == 3 else dict(
            left_mm=200.0, right_mm=200.0, ahead_px=300, mm_per_px=1.0, sole_l_mm=run.true["L"], sole_r_mm=run.true["R"],
            sep_mm=run.true["L"] - run.true["R"], conf=0.9))
        out = run.walk("L", 6, 0.0, 0.0, hold=0.0, board=True, march=True)
        check_up(out, 6)
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        assert sum(r["ev"] == "landing" for r in rows) >= 3
        run.finish(save=False)


# board_view on real frames (testdata/frames). Expected numbers are by eye from the pictures, with the
# board 450 mm wide (what the first look of a run would hand back: the 2026-11-11 nominal stance fixes it):
#   mm per px at a foot's row = 450 / (board's width in px there, read off the two edges), then
#   right_mm = picture-left sole's outer corner to the board's left edge, left_mm the other side, sep = centre to centre.
# name: (right_mm, left_mm, sep_mm) as (lo, hi) ranges, min ahead_px.
FRAMES = {
    "dim_start": ((100, 122), (113, 136), (116, 126), 150),                  # eye: 108, 123, 120; nominal stance
    "dim_shadow_right": ((100, 122), (113, 136), (116, 128), 150),           # same stance, hard shadow on the right
    "roomlight_wide_stance": ((35, 55), (148, 172), (148, 166), 110),        # eye: 42, 155, 155; the target lighting
    "walk110045_t20": ((100, 122), (85, 112), (140, 160), 150),              # eye: 110, 91, 154
    "walk110045_t34": ((90, 115), (92, 118), (140, 162), 140),               # eye: 102, 105, 148
    "walk110045_t47": ((90, 115), (125, 150), (116, 132), 120),              # eye: 103, 134, 124
    "walk110045_t61": ((55, 75), (125, 145), (150, 170), 120),               # eye: 61, 134, 156
    "walk110045_t74": ((55, 75), (140, 165), (132, 146), 110),               # eye: 68, 148, 140
    "walk110045_t88": ((30, 48), (142, 162), (158, 175), 100),               # eye: 35, 153, 165
    # the evening light (2026-11-11 23:50, daylight gone, room light on: the carpet warm yellow-beige, the wall under the
    # desk as saturated as the board): robot square and centred. By eye with the 450 mm board: right 105-115, left 100-110;
    # the plates are 115 px apart (99 px in dim_start, 122 mm) so about 140 mm: it stands wider than at the start
    "evening_square": ((100, 122), (95, 115), (130, 150), 150),
    # 00:07, 20 minutes later: the left plate (picture right) in the robot's own shadow; a pale carpet patch beside the board
    # (x 255-330, y 408-482) once passed for a plate. By eye on the 450 mm scale: right about 130, left about 85, sep about 137
    "evening_misdetect": ((115, 145), (70, 100), (125, 150), 150),
    # the board (it lies as in the others, the camera has not moved) with the robot's right foot half off its
    # left edge: the sole's outer edge is 40, 34, 84, 71 mm outside it by eye
    "walk104632_offedge_t30": ((-60, -25), (200, 235), (150, 190), 100),
    "walk104632_offedge_t45": ((-50, -18), (235, 280), (110, 140), 80),
    "walk104632_offedge_t58": ((-95, -55), (235, 280), (150, 180), 70),
    "walk104632_offedge_t61.5": ((-95, -50), (235, 280), (145, 175), 70),
}


def test_fall_notification_fires_once():
    import robot_io

    class Stub:                                                    # a robot that stays fallen for `until` s
        def __init__(self, until=100.0):
            self.until = until
            self.t = 0.0
            self.c = mock.Mock()
            self.polls = 0

        def fallen(self):
            self.polls += 1
            return self.t < self.until
    stub = Stub()
    calls = []

    def mono():
        return stub.t

    def sleep(s):
        stub.t += s
    with mock.patch.object(robot_io.time, "monotonic", mono), mock.patch.object(robot_io.time, "sleep", sleep):
        ok = robot_io.Robot.wait_recovered(stub, 1200.0, say=lambda *a: None, after=(45.0, lambda: calls.append(stub.t)))
    assert ok is True and len(calls) == 1 and 45.0 <= calls[0] < 47.0, (ok, calls)
    stub = Stub(1e9)
    with mock.patch.object(robot_io.time, "monotonic", mono), mock.patch.object(robot_io.time, "sleep", sleep):   # never picked up
        ok = robot_io.Robot.wait_recovered(stub, 300.0, say=lambda *a: None, after=(45.0, lambda: 1 / 0))
    assert ok is False                                             # a failing hook does not break the wait
    with env() as d:                                               # Run.fallen: the command is built and started once
        run = make(d)
        assert run.p["notify_cmd"] == []
        with mock.patch.object(S.subprocess, "Popen") as pop:
            run.t_fall = "12:34"
            run.notify()
            assert not pop.called                                  # disabled
            run.p["notify_cmd"] = ["/bin/true", "fell at {hhmm}"]
            run.notify()
            assert pop.call_count == 1 and pop.call_args[0][0] == ["/bin/true", "fell at 12:34"], pop.call_args
            pop.side_effect = FileNotFoundError("nope")
            run.notify()                                           # a missing script never raises
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        assert sum(r["ev"] == "notify" for r in rows) == 2, rows
        run.finish(save=False)
    assert S.DEFAULTS["notify_cmd"][0].endswith("ask-aime.sh") and S.DEFAULTS["notify_after_s"] == 45.0
    assert S.main(["lift", "--no-notify", "--set", "x=1"], make_robot=FakeRobot, log_dir=tempfile.mkdtemp()) in (0, 1)


def test_backward_and_shuttle():
    with env() as d:
        run = make(d)
        out = run.walk("L", 4, -0.02, 0.0)
        check_up(out, 4)
        assert abs(run.feet["L"][0] + 0.06) < 1e-9 and abs(run.feet["R"][0] + 0.08) < 1e-9, run.feet   # half, full, full, full strides
        run.finish(save=False)
    with env() as d:                                               # N forward, N back, twice: where it began
        run = make(d)
        out = run.shuttle("L", 4, 2, 0.02, 0.0)
        check_up(out, 16)
        assert all(abs(f[0]) < 1e-3 and abs(f[1]) < 1e-3 for f in run.feet.values()), run.feet    # (a heading correction may rotate it a hair)
        run.finish(save=False)
    with env() as d:                                               # a dx per cycle (a landing map)
        run = make(d)
        seen = []
        lift0 = run.lift
        run.lift = lambda swing, **kw: (seen.append(kw["dx"]), lift0(swing, **kw))[1]
        out = run.shuttle("R", 3, 3, 0.0, 0.0, dx_list=[0.02, 0.03])
        check_up(out, 18)
        assert abs(max(abs(x) for x in seen) - 0.06) < 1e-9 and abs(min(abs(x) for x in seen if x) - 0.02) < 1e-9, seen
        assert all(abs(f[0]) < 1e-3 for f in run.feet.values()), run.feet
        run.finish(save=False)
    with env() as d:                                               # the landing misses, learned away, over 2 cycles of 4
        run = make(d)
        board_stub(run, left0=300.0, right0=300.0, miss=MISS)
        out = run.shuttle("L", 4, 2, 0.02, 0.0, board=True)
        check_up(out, 16)
        assert all(abs(run.true[f] - run.start[f]) < 15.0 for f in "LR"), run.true
        assert all(abs(f[0]) < 1e-3 for f in run.feet.values()), run.feet
        run.finish(save=False)
    with env() as d:                                               # the board runs out ahead: turn round early, still home
        run = make(d)
        board_stub(run, ahead=lambda r: int(200 - 1500 * max(f[0] for f in r.feet.values())))
        out = run.shuttle("L", 6, 1, 0.03, 0.0, board=True)
        check_up(out, len(out))
        assert 0 < len(out) < 12 and run.stop_why is None, (len(out), run.stop_why)
        assert all(abs(f[0]) < 1e-3 for f in run.feet.values()), run.feet
        run.finish(save=False)
    with env() as d:                                               # an edge ends the run, forward or back
        run = make(d)
        board_stub(run, left0=20.0, right0=90.0)
        assert run.shuttle("L", 4, 1, 0.02, 0.0, board=True) == [] and run.stop_why
        run.finish(save=False)


def test_pace():
    with env() as d:
        prm = S.apply_pace(S.load_state(), 0.6)
        base = S.load_state()
        for k in S.TIMES:
            assert abs(prm[k] - 0.6 * base[k]) < 1e-12, k
        assert prm["t_crouch"] == base["t_crouch"] and prm["pace"] == 0.6
        rc = S.main(["walk", "--pace", "0.6", "--steps", "2", "--hold", "0"], make_robot=FakeRobot, log_dir=d)
        assert rc == 0
        rows = [json.loads(line) for line in open(os.path.join(d, [f for f in os.listdir(d) if f.endswith(".jsonl")][0]))]
        st = rows[0]
        assert st["ev"] == "start" and st["pace"] == 0.6 and abs(st["utc"] - time.time()) < 3600 and abs(st["prm"]["t_swing"] - 0.6) < 1e-9, st
        assert all("utc" in r for r in rows if r["ev"] == "lift")


def test_inward_tipping_raises_k():
    with env() as d:
        run = make(d)
        run.shift_over("R")
        ref = dict(run.ref0)
        # hanging 9 deg toward the lifted left foot: the weight is inboard of the stance foot
        run.read = lambda tag="": dict(roll=ref["roll"] - 9.0, pitch=ref["pitch"], rate=0.0, gyro=[0, 0, 0], up=[0, 0, 1])
        assert run.raise_foot("L")[0] == "tipping_in"
        run.read = lambda tag="": dict(roll=ref["roll"], pitch=ref["pitch"] + 9.0, rate=0.0, gyro=[0, 0, 0], up=[0, 0, 1])
        assert run.raise_foot("L")[0] == "tipping"                 # pitch alone: as before
        run.finish(save=False)
    with env() as d:
        run = make(d)
        k0 = run.p["k_shift"]["R"]
        raise0 = run.raise_foot
        calls = []

        def raise_foot(swing):
            calls.append(1)
            if len(calls) == 1:
                return "tipping_in", 0.01
            return raise0(swing)
        run.raise_foot = raise_foot
        how, h = run.lift("L", hold=0.0)
        assert how == "up"
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        lifts = [r for r in rows if r["ev"] == "lift"]
        assert lifts[0]["how"] == "tipping_in" and abs(lifts[1]["k"] - (k0 + run.p["k_step"])) < 1e-9, lifts
        run.finish(save=False)
    assert S.DEFAULTS["tilt_lim"] == 8.0


def test_servo_guard_stops_run():
    with env() as d:
        run = make(d)
        run.r.temp = 70
        out = run.walk("L", 4, 0.02, 0.0)
        assert out == [] and run.stop_why and "C" in run.stop_why, (out, run.stop_why)
        run.finish(save=False)
    with env() as d:
        run = make(d)
        run.p["guard_every"] = 2
        n = [0]
        pos0 = run.r.pos

        def pos(name):
            n[0] += 1
            run.r.volt = 11.8 if n[0] <= 16 else 9.9               # sags after the second set of readings
            return pos0(name)
        run.r.pos = pos
        out = run.walk("L", 8, 0.02, 0.0)
        assert len(out) == 4 and "V" in run.stop_why, (len(out), run.stop_why)
        rows = [json.loads(line) for line in open(run.base + ".jsonl")]
        assert sum(r["ev"] == "servos" for r in rows) == 3 and rows[-1]["ev"] in ("guard_stop", "servos"), rows[-3:]
        run.finish(save=False)


def test_playsweep_and_shiftsweep():
    with env() as d:
        rc = S.main(["playsweep", "--side", "L", "--hold", "0"], make_robot=FakeRobot, log_dir=d)
        assert rc == 0
        rows = [json.loads(line) for line in open(os.path.join(d, [f for f in os.listdir(d) if f.endswith('.jsonl')][0]))]
        sw = [r for r in rows if r["ev"] == "sweep"]
        joints = {r["joint"] for r in sw}
        assert joints == {"hip_roll", "ankle_roll"}, joints
        assert all(k in sw[0] for k in ("offset_deg", "roll", "pitch", "tilt", "rate")), sw[0]
        for j in joints:
            offs = [r["offset_deg"] for r in sw if r["joint"] == j]
            assert len(offs) >= 16 and max(offs) > 0 > min(offs), (j, len(offs))   # both ways round the found trim
        assert not any(r["ev"] == "error" for r in rows)
        assert rows[-1]["ev"] == "end"
    with env() as d:
        rc = S.main(["shiftsweep", "--set", "sweep_cycles=1"], make_robot=FakeRobot, log_dir=d)
        assert rc == 0
        rows = [json.loads(line) for line in open(os.path.join(d, [f for f in os.listdir(d) if f.endswith('.jsonl')][0]))]
        sw = [r for r in rows if r["ev"] == "sweep"]
        mm = [r["shift_mm"] for r in sw]
        assert max(mm) >= 61.9 and min(mm) <= -61.9 and abs(mm[-1]) < 1e-6 and all(k in sw[0] for k in ("roll", "pitch")), (max(mm), min(mm), mm[-1])
        assert sum(r["ev"] == "servos" for r in rows) >= len(sw) // 3 - 1
        assert not any(r["ev"] == "error" for r in rows)
    with env() as d:                                               # a lean past the limit: back to the centre, run ends clean
        prm = S.load_state()
        kin = Kin(prm["drop"])
        rob = FakeRobot(kin, noise=0.0)
        run = S.Run("lean", rob, kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        real = rob.imu
        rob.imu = lambda: dict(real(), roll=real()["roll"] + (9.0 if abs(run.pel[1]) > 0.03 else 0.0))
        assert run.shiftsweep() == "lean"
        assert abs(run.pel[1]) < 1e-6
        run.finish(save=False)


def test_board_view_frames():
    import cv2
    import board_view
    for name, (r, l, sep, ahead) in FRAMES.items():
        im = cv2.imread(os.path.join(HERE, "testdata", "frames", name + ".jpg"))
        assert im is not None, name
        m = board_view.margins(im, board_w_mm=450.0)
        assert m is not None, f"{name}: board_view could not read the frame"
        assert r[0] <= m["right_mm"] <= r[1], (name, "right", m["right_mm"], r)
        assert l[0] <= m["left_mm"] <= l[1], (name, "left", m["left_mm"], l)
        assert sep[0] <= m["sep_mm"] <= sep[1], (name, "sep", m["sep_mm"], sep)
        assert m["ahead_px"] > ahead, (name, m["ahead_px"])
        assert m["conf"] >= 0.5, (name, m["conf"])
        # where the soles stand across the board agrees with the margins: centre line to each sole
        assert abs((m["sole_l_mm"] - m["sole_r_mm"]) - m["sep_mm"]) < 1.0, name
        if name.startswith("walk104632"):
            assert m["right_mm"] < 25.0, name                          # the warning is there before the fall
    # no stored width: the soles' own scale (calibrated on the nominal stance) still reads the nominal stance
    m = board_view.margins(cv2.imread(os.path.join(HERE, "testdata", "frames", "dim_start.jpg")), None)
    assert m is not None and 116 <= m["sep_mm"] <= 126 and 400 <= m["board_w_mm"] <= 500, m   # (soles' own scale)
    for name in FRAMES:                                               # the default IS the fixed 450 mm scale
        im = cv2.imread(os.path.join(HERE, "testdata", "frames", name + ".jpg"))
        assert board_view.margins(im)["sep_mm"] == board_view.margins(im, 450.0)["sep_mm"]
    m = board_view.margins(cv2.imread(os.path.join(HERE, "testdata", "frames", "roomlight_wide_stance.jpg")), None)
    assert m is not None and 40 <= m["right_mm"] <= 55, m             # the room light, no width at all
    # one board edge not reliably seen: the other is inferred from the stored width, with less confidence
    im = cv2.imread(os.path.join(HERE, "testdata", "frames", "walk110045_t47.jpg"))
    full = board_view.margins(im, board_w_mm=450.0)
    assert full["inferred"] is None
    edges0 = board_view.edges
    for side, k in (("left", 0), ("right", 1)):
        with mock.patch.object(board_view, "edges", lambda b, y=None, k=k: tuple(None if i == k else e for i, e in enumerate(edges0(b, y)))):
            m = board_view.margins(im, board_w_mm=450.0, taper=full["taper"])
            assert m is not None and m["inferred"] == side and m["conf"] < 0.6 * full["conf"] + 1e-9, (side, m)
            assert abs(m["right_mm"] - full["right_mm"]) < 25 and abs(m["left_mm"] - full["left_mm"]) < 25, (side, m, full)
            m = board_view.margins(im, board_w_mm=450.0)                # no taper known: parallel, less trusted
            assert m is not None and m["conf"] < 0.4, m
            assert board_view.margins(im, None) is None                # no width to infer from: cannot tell
    assert board_view.margins(im * 0) is None                          # a black frame: cannot tell, says so


def test_track_tool():
    import io
    import cv2
    import board_view
    names = ("dim_start", "walk110045_t47", "walk110045_t88")
    ims = [cv2.imread(os.path.join(HERE, "testdata", "frames", n + ".jpg")) for n in names]
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "x_sony.avi")
        vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 5.0, (ims[0].shape[1], ims[0].shape[0]))
        if not vw.isOpened():
            print("   (no MJPG writer: track test skipped)")
            return
        for im in ims:
            vw.write(im)
        vw.release()
        with open(os.path.join(d, "x_sony.pts"), "w") as f:
            f.write("# timestamp format v2\n" + "\n".join(str(1.7e12 + 200 * i) for i in range(3)) + "\n")
        buf = io.StringIO()
        rows = board_view.track(path, every=1, board_w_mm=450.0, out=buf)
        lines = buf.getvalue().strip().splitlines()
        assert lines[0].split(",") == ["frame", "t_s", "utc", "sole_r_mm", "sole_l_mm", "right_mm", "left_mm", "sep_mm", "conf"], lines[0]
        assert len(rows) == 3 and len(lines) == 4
        good = 0
        for i, (r, im) in enumerate(zip(rows, ims)):
            m = board_view.margins(im, 450.0)
            if r[8] and float(r[8]) >= 0.5:                        # (video compression can cost a frame its confidence)
                good += 1
                assert abs(float(r[3]) - m["sole_r_mm"]) < 8 and abs(float(r[4]) - m["sole_l_mm"]) < 8, (r, m["sole_r_mm"])
            assert abs(float(r[2]) - (1.7e9 + 0.2 * i)) < 1e-3 and abs(float(r[1]) - 0.2 * i) < 1e-3, r
        assert good >= 2
        assert len(board_view.track(path, every=2, out=io.StringIO())) == 2


def test_board_view_cannot_take_carpet_for_a_sole():
    """A pale patch of carpet beside the board must never be a sole: it is not a wide low plate (square), it has no dark
    robot structure above it, it is not on the board. And with a real sole missing the answer is None, not a pair."""
    import cv2
    import board_view
    im = cv2.imread(os.path.join(HERE, "testdata", "frames", "evening_misdetect.jpg"))
    ref = board_view.margins(im)
    assert ref is not None and ref["conf"] > 0.8
    for box in ((255, 408, 330, 482), (230, 430, 330, 470), (240, 400, 340, 440)):        # square and flatter patches left of the board
        x0, y0, x1, y1 = box
        bad = im.copy()
        bad[y0:y1, x0:x1] = (190, 205, 215)                                                # pale warm grey
        m = board_view.margins(bad)
        assert m is not None and all(abs(m[k] - ref[k]) < 2.0 for k in ("right_mm", "left_mm", "sep_mm")), (box, m and (m["right_mm"], m["left_mm"], m["sep_mm"]))
    # a plate-shaped pale patch right beside the board, with a dark strap above it, is still not a sole of the pair under the robot
    bad = im.copy()
    bad[455:480, 230:320] = (190, 205, 215)
    bad[420:455, 230:320] = (60, 40, 30)
    m = board_view.margins(bad)
    assert m is not None and abs(m["sep_mm"] - ref["sep_mm"]) < 2.0, m and m["sep_mm"]
    # one real sole painted out: nothing to pair with but the patch: None
    bad = im.copy()
    bad[415:485, 405:505] = bad[560, 450]                                                  # the board's own colour
    bad[408:482, 255:330] = (190, 205, 215)
    assert board_view.margins(bad) is None
    # the stages: a pair found by a fallback carries less confidence
    m2 = board_view.margins(cv2.imread(os.path.join(HERE, "testdata", "frames", "evening_square.jpg")))
    assert m2 is not None and m2["conf"] < ref["conf"] - 0.1, (m2["conf"], ref["conf"])


def test_look_median_of_frames():
    import cv2
    ims = {n: cv2.imread(os.path.join(HERE, "testdata", "frames", n + ".jpg")) for n in ("dim_start", "dim_shadow_right")}
    blank = ims["dim_start"] * 0

    def run_with(seq):
        with env() as d:
            run = make(d)
            it = iter(seq)

            def nxt(after, tmax):
                try:
                    return next(it), after + 0.1
                except StopIteration:
                    return None
            run._next_frame = nxt
            m = run.look("t")
            files = sorted(f for f in os.listdir(d) if "_board_" in f)
            run.finish(save=False)
            return m, files, run
    m, files, run = run_with([ims["dim_start"], ims["dim_shadow_right"], ims["dim_start"]])
    assert m is not None, m
    assert 116 <= m["sep_mm"] <= 128 and run.bt is None            # the fixed scale; the look itself sets no target
    assert files == ["%s_board_01.jpg" % os.path.basename(run.base)], files
    m, _, _ = run_with([ims["dim_start"], blank, ims["dim_start"]])        # one frame lost: the other two do
    assert m is not None
    m, _, _ = run_with([blank, ims["dim_start"], blank])                   # only one good: not enough
    assert m is None
    m, _, _ = run_with([])                                                 # no frames at all
    assert m is None


def main():
    fails = 0
    for name, fn in sorted((k, v) for k, v in globals().items() if k.startswith("test_") and callable(v)):
        try:
            fn()
            print("ok  ", name, flush=True)
        except Exception as e:
            fails += 1
            import traceback
            traceback.print_exc()
            print("FAIL", name, repr(e), flush=True)
    print("FAILED" if fails else "all dry runs passed", f"({fails} failing)" if fails else "")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
