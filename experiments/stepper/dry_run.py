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
        self.r.tell(self.feet, self.hike)            # the fake body follows the commanded feet


@contextlib.contextmanager
def env():
    """No sleeping, state and logs in a temp dir, the fake body told where the feet are."""
    random.seed(1)
    with tempfile.TemporaryDirectory() as d, mock.patch.object(time, "sleep", lambda s: None), \
            mock.patch.object(S, "STATE", os.path.join(d, "state.json")), mock.patch.object(S.Run, "go", _go):
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


def board_stub(run, left0=130.0, right0=70.0, ahead=lambda run: 300, script=None):
    """A camera stand-in: the room left and right of each sole from where the feet really are."""
    calls = []

    def look(tag):
        calls.append(tag)
        if script is not None:
            m = script(len(calls))
        else:
            m = dict(left_mm=left0 - 1e3 * run.feet["L"][1], right_mm=right0 + 1e3 * run.feet["R"][1],
                     ahead_px=ahead(run), mm_per_px=0.5, sep_mm=120.0)
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
        e = 0.5 * ((130 - 1e3 * run.feet["L"][1]) - (70 + 1e3 * run.feet["R"][1]))
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
        ok = dict(left_mm=100.0, right_mm=100.0, ahead_px=300, mm_per_px=0.5, sep_mm=120.0)
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


def test_board_view_fixture():
    import cv2
    import board_view
    im = cv2.imread(os.path.join(HERE, "testdata", "board_front.jpg"))
    assert im is not None
    m = board_view.margins(im)
    assert m is not None, "board_view could not read the fixture frame"
    assert 85 <= m["right_mm"] <= 110, m["right_mm"]
    assert 95 <= m["left_mm"] <= 120, m["left_mm"]
    assert m["ahead_px"] > 150, m["ahead_px"]


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
