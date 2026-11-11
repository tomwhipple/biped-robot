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
        self.r.tell(self.feet, self.hike, self.ankle)    # the fake body follows the commanded feet and trims


_load_state = S.load_state


def _safe_state():
    prm = _load_state()
    prm["notify_cmd"] = []                           # a dry run never calls the real notify script
    return prm


@contextlib.contextmanager
def env():
    """No sleeping, state and logs in a temp dir, the fake body told where the feet are."""
    random.seed(1)
    with tempfile.TemporaryDirectory() as d, mock.patch.object(time, "sleep", lambda s: None), \
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


def board_stub(run, left0=130.0, right0=70.0, ahead=lambda run: 300, script=None, miss=None):
    """A camera stand-in and a body that lands its feet where it likes. `true` holds where each sole really
    stands (y, m; set when a foot goes down: the y it was sent to plus miss[side]); the margins and the
    separation come from it. left0/right0: the room left and right of the soles at y = 0."""
    miss = miss or {"L": 0.0, "R": 0.0}
    run.true = {"L": 0.0, "R": 0.0}
    run.landed = []
    lower0 = run.lower_foot

    def lower(swing, fast=False):
        r = lower0(swing, fast)
        run.true[swing] = run.fy(swing) + miss[swing]
        run.landed.append((swing, run.true[swing]))
        return r
    run.lower_foot = lower
    calls = []

    def look(tag):
        calls.append(tag)
        if script is not None:
            m = script(len(calls))
        else:
            t = run.true
            m = dict(left_mm=left0 - 1e3 * t["L"], right_mm=right0 + 1e3 * t["R"], ahead_px=ahead(run), mm_per_px=0.5,
                     sep_mm=121.0 + 1e3 * (t["L"] - t["R"]), conf=0.9)
        run.log("board", tag=tag, seen=m is not None)
        return m
    run.look = look
    return calls


def test_board_steers_toward_the_middle():
    with env() as d:
        run = make(d)
        board_stub(run)                                            # 30 mm more room on the left: go left
        out = run.walk("L", 6, 0.0, 0.0, board=True)
        check_up(out, 6)
        e = 0.5 * ((130 - 1e3 * run.true["L"]) - (70 + 1e3 * run.true["R"]))
        assert abs(e) <= run.p["board_dead_mm"] + 1e-6, e          # it came back inside the dead band
        assert run.feet["L"][1] > 0.0 and run.feet["R"][1] > 0.0, run.feet
        assert run.stop_why is None
        run.finish(save=False)
    with env() as d:
        run = make(d)
        board_stub(run, left0=70.0, right0=130.0)                   # more room on the right: right foot first
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
        ok = dict(left_mm=100.0, right_mm=100.0, ahead_px=300, mm_per_px=0.5, sep_mm=121.0, conf=0.9)
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


def test_landing_misses_converge():
    """The right foot lands 25 mm too wide, the left 8 mm: after a few steps lat_ff has taken the misses out,
    the stance is back at its nominal width, and the steering still brings the robot toward the middle."""
    miss = {"L": 0.008, "R": -0.025}                               # + = left: both outward
    with env() as d:
        run = make(d)
        board_stub(run, miss=miss)                                 # starts 30 mm right of the middle
        out = run.walk("L", 10, 0.0, 0.0, board=True)
        check_up(out, 10)
        ff = run.p["lat_ff"]
        for f in "LR":
            assert abs(miss[f] + ff[f]) < 0.006, (f, miss[f], ff[f])             # the landing error left
        assert abs(1e3 * (run.true["L"] - run.true["R"])) < 6.0, run.true       # stance width nominal +-6 mm
        e0, e1 = 30.0, 0.5 * ((130 - 1e3 * run.true["L"]) - (70 + 1e3 * run.true["R"]))
        assert abs(e1) < 20.0 and abs(e1) < e0, (e0, e1)
        assert any(abs(v) > 0 for v in ff.values())
        run.finish(save=True)
        saved = json.load(open(S.STATE))
        assert saved["lat_ff"] == ff, saved
    with env() as d:                                               # without --board nothing is measured or learned
        prm = S.load_state()
        prm["lat_ff"] = {"L": 0.01, "R": -0.01}
        kin = Kin(prm["drop"])
        run = S.Run("noboard", FakeRobot(kin), kin, prm, quiet=QUIET, log_dir=d)
        run.stand_up()
        run.look = lambda tag: (_ for _ in ()).throw(AssertionError("looked"))
        out = run.walk("L", 4, 0.0, 0.0)
        check_up(out, 4)
        assert prm["lat_ff"] == {"L": 0.01, "R": -0.01}
        assert abs(run.fy("L") - 0.01) < 1e-9 and abs(run.fy("R") + 0.01) < 1e-9, (run.fy("L"), run.fy("R"))   # applied
        assert run.feet["L"][1] == 0.0 and run.feet["R"][1] == 0.0                                             # belief unchanged
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
        assert all(abs(f[0]) < 1e-9 and abs(f[1]) < 1e-9 for f in run.feet.values()), run.feet
        xs = [round(f[0], 6) for f in run.feet.values()]
        assert xs == [0.0, 0.0], xs
        run.finish(save=False)
    with env() as d:                                               # a dx per cycle (a landing map)
        run = make(d)
        seen = []
        lift0 = run.lift
        run.lift = lambda swing, **kw: (seen.append(kw["dx"]), lift0(swing, **kw))[1]
        out = run.shuttle("R", 3, 3, 0.0, 0.0, dx_list=[0.02, 0.03])
        check_up(out, 18)
        assert abs(max(abs(x) for x in seen) - 0.06) < 1e-9 and abs(min(abs(x) for x in seen if x) - 0.02) < 1e-9, seen
        assert all(abs(f[0]) < 1e-9 for f in run.feet.values()), run.feet
        run.finish(save=False)
    with env() as d:                                               # the landing misses, learned away, over 2 cycles of 4
        run = make(d)
        board_stub(run, left0=100.0, right0=100.0, miss={"L": 0.008, "R": -0.025})
        out = run.shuttle("L", 4, 2, 0.02, 0.0, board=True)
        check_up(out, 16)
        assert all(abs(run.true[f]) < 0.015 for f in "LR"), run.true
        assert all(abs(f[0]) < 1e-9 for f in run.feet.values()), run.feet
        run.finish(save=False)
    with env() as d:                                               # the board runs out ahead: turn round early, still home
        run = make(d)
        board_stub(run, ahead=lambda r: int(200 - 1500 * max(f[0] for f in r.feet.values())))
        out = run.shuttle("L", 6, 1, 0.03, 0.0, board=True)
        check_up(out, len(out))
        assert 0 < len(out) < 12 and run.stop_why is None, (len(out), run.stop_why)
        assert all(abs(f[0]) < 1e-9 for f in run.feet.values()), run.feet
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
        rows = [json.loads(line) for line in open(os.path.join(d, os.listdir(d)[0]))]
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
        rows = [json.loads(line) for line in open(os.path.join(d, os.listdir(d)[0]))]
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
    m = board_view.margins(cv2.imread(os.path.join(HERE, "testdata", "frames", "dim_start.jpg")))
    assert m is not None and 116 <= m["sep_mm"] <= 126 and 400 <= m["board_w_mm"] <= 500, m
    m = board_view.margins(cv2.imread(os.path.join(HERE, "testdata", "frames", "roomlight_wide_stance.jpg")))
    assert m is not None and 40 <= m["right_mm"] <= 55, m             # the room light, no state at all
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
            assert board_view.margins(im) is None                      # no width to infer from: cannot tell
    assert board_view.margins(im * 0) is None                          # a black frame: cannot tell, says so


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
    assert m is not None and abs(m["sep_mm"] - 121.0) < 0.01, m           # first look: scaled to the nominal separation
    assert run.f_sep is not None and 0.8 < run.f_sep < 1.2 and run.bw() is not None
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
