"""track_host --polls-in-ss (#297, 2026-11-25): the per-step `track steps` / `imu` polls wait for a planned single
support. The old firmware runs every CLI command on the task that runs the track player, and its `imu` stalled that
loop ~115 ms right after each touchdown (docs/design-v6/2026-11-25-weight-shift-snap-sim.md). Off (the default) the
live loop's commands are exactly today's; on, no per-step poll goes out in a double support."""
import json
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "roll-balance"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

H = pytest.importorskip("track_host")


def _ss(sw):
    """[(lift-off ms, touchdown ms)] on the track clock."""
    return [(1e3 * (p["t0"] - sw.t_first), 1e3 * (p["t1"] - sw.t_first)) for p in sw.phases]


class _Robot:
    """Plays STEP_MS of the queue per `track` poll; records every command with the play head it went out at."""
    sw, step_ms, sent = None, 100, []

    def __init__(self, host=None, log=None, **_):
        self.cal = {n: (i + 1, 2048, 1) for i, n in enumerate(H.G.LEG)}
        self.head = 0
        self.c = types.SimpleNamespace(idle=lambda *_a, **_k: _Robot.sent.append(("<idle>", self.head)))

    def _n(self):
        return sum(1 for _, b in _ss(_Robot.sw) if b <= self.head)

    def cmd(self, line, idle=8.0):
        _Robot.sent.append((line, self.head))
        if line == "imu":
            return "up 0 0 1\r\nquat 1.0 0.0 0.0 0.0\r\n"
        if line.startswith("track add"):
            return "track +4"
        if line == "track":
            self.head += _Robot.step_ms
            n = self._n()
            return f"track {'done' if n >= len(_Robot.sw.phases) else 'play'}: play t {self.head} ms\r\n" \
                   f"{n} single supports reported\r\n"
        if line == "track steps":
            return "\r\n".join(f"TS {i + 1} {p['stance']} in 2.0 out 1.0 e +0.0 T +3.0 hike +3.0 ok"
                               for i, p in enumerate(_Robot.sw.phases[:self._n()]))
        return "ok"

    def fallen(self):
        return False

    def close(self):
        pass


def _run(tmp_path, monkeypatch, *flags):
    fake = types.ModuleType("robot_io")
    fake.Robot = _Robot
    monkeypatch.setitem(sys.modules, "robot_io", fake)
    monkeypatch.setattr(H, "ANKLE_STATE", str(tmp_path / "state.json"))
    real_plan = H.plan

    def spy(*a, **k):
        p = real_plan(*a, **k)
        _Robot.sw = p["sway"]
        return p
    monkeypatch.setattr(H, "plan", spy)
    _Robot.sent = []
    log = tmp_path / "run.jsonl"
    rc = H.main(["--T", "4", "--dir", "fwd", "--steps", "6", "--live", "--ahead", "1", "--poll", "0", "--log", str(log),
                 "--seed", "5.2,3.4", "--heading-hold", "on", *flags])
    assert rc == 0
    summ = [json.loads(l) for l in open(log) if '"summary"' in l][-1]
    play = next(i for i, (l, _) in enumerate(_Robot.sent) if l.startswith("track play"))
    end = next(i for i, (l, _) in enumerate(_Robot.sent) if l == "<idle>")     # live()'s wind-down starts here
    walking = _Robot.sent[play + 1:end]                      # the walk itself, not the before/after reads
    return summ, walking


def _in_ss(head, ss, before_ms=0.0):
    return any(a < head < b - before_ms for a, b in ss)


def test_default_off_polls_right_after_each_touchdown_as_today(tmp_path, monkeypatch):
    summ, walking = _run(tmp_path, monkeypatch)
    polls = [(l, h) for l, h in walking if l in ("track steps", "imu")]
    ss = _ss(_Robot.sw)
    assert len(summ["steps"]) == len(ss) and summ["polls_in_ss"]["on"] is False
    # today's behaviour: the poll follows the status reply that first counts the new single support, i.e. the
    # double support right after its touchdown -- the timing that stalls the old firmware's loop
    assert polls and all(not _in_ss(h, ss) for _, h in polls)
    assert sum(l == "imu" for l, _ in polls) == sum(l == "track steps" for l, _ in polls)


def test_on_every_per_step_poll_goes_out_inside_a_single_supports_window(tmp_path, monkeypatch):
    summ, walking = _run(tmp_path, monkeypatch, "--polls-in-ss")
    ss = _ss(_Robot.sw)
    polls = [(l, h) for l, h in walking if l in ("track steps", "imu")]
    assert polls
    for l, h in polls:
        assert any(a + H.POLL_SS_AFTER_MS <= h <= b - H.POLL_SS_BEFORE_MS for a, b in ss), (l, h)
    st = summ["polls_in_ss"]
    assert st["on"] and st["forced"] == 0 and st["deferred"] == st["polls"] > 0
    # the stop rule still sees every step, the heading hold still reads the heading after each
    assert len(summ["steps"]) == len(ss)
    assert sum(l == "imu" for l, _ in polls) >= len(ss) - 1


def test_window_is_the_planned_single_support_minus_its_guards():
    sw = types.SimpleNamespace(t_first=1.0, phases=[dict(t0=2.0, t1=3.0), dict(t0=4.0, t1=5.0)])
    open_ = lambda h, hold=False: H.poll_window_open(sw, h, hold, after_ms=60, before_ms=300)  # noqa: E731
    assert not open_(1000 + 30)                 # just lifted
    assert open_(1000 + 60) and open_(1700)    # inside
    assert not open_(1701)                      # too close to the touchdown for `track steps` + `imu`
    assert not open_(2500)                      # double support
    assert open_(3200) and not open_(3200, hold=True)   # a lift gate's hold has both feet down


def test_gate_defers_then_fires_and_forces_after_the_max_wait():
    sw = types.SimpleNamespace(t_first=0.0, phases=[dict(t0=1.0, t1=2.0)])
    t = {"now": 0.0}
    g = H.PollGate(True, clock=lambda: t["now"], max_wait_s=3.0)
    assert not g.ready(sw, 2100, False)         # double support: wait
    t["now"] = 0.5
    assert not g.ready(sw, 2600, False)
    t["now"] = 3.0
    assert g.ready(sw, 2900, False)             # 3 s without a window: forced
    assert g.stats["forced"] == 1 and g.stats["waited_s"] == [3.0]
    t["now"] = 4.0
    assert g.ready(sw, 1500, False)             # in a window: at once
    assert g.stats == dict(on=True, polls=2, deferred=1, forced=1, waited_s=[3.0, 0.0])
    off = H.PollGate(False)
    assert off.ready(sw, 2100, True) and off.stats["deferred"] == 0
