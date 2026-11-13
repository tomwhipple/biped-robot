"""The host-side learned stance-ankle trim (experiments/roll-balance/track_host.py, #297 B2): the step-rate rule,
keep / revert, the state file, and the live loop re-planning only the steps not uploaded yet (a fake robot)."""
import json
import os
import re
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.join(HERE, "..", "experiments", "roll-balance")
sys.path.insert(0, RB)

H = pytest.importorskip("track_host")


def test_rule_and_clamp(tmp_path):
    L = H.AnkleLearner("fwd", path=str(tmp_path / "s.json"))
    assert L.A == {"L": 2.0, "R": 2.0}
    assert L.update("R", 10.5, 0.1) == 3.0                    # tip: +1
    assert L.update("R", 2.0, 5.0) == 2.5                     # out 5: -0.5
    assert L.update("R", 2.0, 9.0) == pytest.approx(0.5)      # out 9 > 8: back to the keep (2.0: no clean step yet), -1.5
    assert L.update("L", 3.5, 1.0) == 2.5                     # in > 3, out < 2: +0.5
    assert L.update("L", 2.0, 3.0) == 2.5                     # hold
    for _ in range(5):
        L.update("L", 12.0, 0.0)
    assert L.A["L"] == H.ANKLE_MAX
    for _ in range(8):
        L.update("R", 1.0, 7.5)
    assert L.A["R"] == H.ANKLE_MIN


def test_keep_and_revert(tmp_path):
    L = H.AnkleLearner("back", path=str(tmp_path / "s.json"))
    L.update("R", 2.0, 3.0, used=2.0)          # clean: keep 2.0
    L.update("R", 7.5, 0.5, used=2.0)          # tip: +1 from 2.0
    L.update("R", 11.5, 0.0, used=3.0)         # another tip (past 8 inward: no revert): +1 again
    assert L.A["R"] == 4.0 and L.keep["R"] == 2.0
    L.update("R", 2.0, 3.0, used=4.0)          # clean at 4: keep 4
    L.update("R", 2.0, 3.0, used=4.0)
    L.A["R"] = 3.0
    L.update("R", 1.0, 8.5, used=3.0)          # past 8 outward: back to the keep (4.0), then -1.5
    assert L.A["R"] == pytest.approx(2.5) and L.log[-1]["reverted"]


def test_state_file(tmp_path):
    p = str(tmp_path / "s.json")
    L = H.AnkleLearner("sideR", path=p)
    assert L.A == {"L": 2.0, "R": 3.0}                        # the sideR seed
    L.update("R", 11.0, 0.0)
    L.save()
    assert json.load(open(p))["sideR"] == {"L": 2.0, "R": 4.0}
    assert H.AnkleLearner("sideR", path=p).A["R"] == 4.0
    assert H.AnkleLearner("fwd", path=p).A == {"L": 2.0, "R": 2.0}
    assert H.AnkleLearner("sideR", path=p, start=(2.0, 2.0)).A == {"L": 2.0, "R": 2.0}     # --ankle-seed


def test_replan_locks_uploaded_steps():
    p1 = H.plan(4.0, 6, "fwd", ankle_steps=[2.0] * 7)
    sw = p1["sway"]
    ff = 3
    steps2 = H.ankle_steps_for(sw, {"L": 2.0, "R": 3.5}, p1["args"]["ankle_steps"], ff)
    assert steps2[:ff] == [2.0] * ff
    assert all(v == (3.5 if p["stance"] == "R" else 2.0) for v, p in zip(steps2[ff:], sw.phases[ff:]))
    p2 = H.plan(**{**p1["args"], "ankle_steps": steps2})
    t_lock = 1e3 * (sw.phases[ff]["t0"] - sw.t_first)
    assert all(a == b for (ta, a), (_, b) in zip(p1["adds"], p2["adds"]) if ta <= t_lock)
    assert any(a != b for (_, a), (_, b) in zip(p1["adds"], p2["adds"]))


class _Reply:
    def __init__(self, text):
        self.text, self.status, self.refused = text, "done", False


class _FakeRobot:
    """Plays the queue 400 ms per `track` poll; R stances tip (in 10.5) until their A reaches 3, then run clean."""
    sent = []

    def __init__(self, host=None, log=None, **_):
        self.cal = {n: (i + 1, 2048, 1) for i, n in enumerate(H.G.LEG)}
        self.head, self.playing, self.adds = 0, False, []
        self.c = types.SimpleNamespace(idle=lambda *_a, **_k: None)

    def cmd(self, line, idle=8.0):
        _FakeRobot.sent.append(line)
        if line == "imu":
            return "up 0 0 1\r\nquat 1.0 0.0 0.0 0.0\r\n"
        if line.startswith("track add"):
            self.adds.append(line)
            return "track +4"
        if line.startswith("track play"):
            self.playing = True
            return "track play"
        if line == "track":
            self.head += 400
            n = sum(1 for p in self.sw.phases if 1e3 * (p["t1"] - self.sw.t_first) <= self.head)
            done = n >= len(self.sw.phases)
            return (f"track {'done' if done else 'play'}: play t {self.head} ms\r\n{n} single supports reported\r\n")
        if line == "track steps":
            n = sum(1 for p in self.sw.phases if 1e3 * (p["t1"] - self.sw.t_first) <= self.head)
            out = []
            for i, p in enumerate(self.sw.phases[:n]):
                a_used = self.ankle[i] if self.ankle else 0.0
                tip = p["stance"] == "R" and a_used < 3.0
                out.append(f"TS {i + 1} {p['stance']} in {10.5 if tip else 2.0:.1f} out {0.1 if tip else 3.0:.1f} "
                           f"e +0.0 T +3.0 hike +3.0 ok")
            return "\r\n".join(out)
        return "ok"

    def fallen(self):
        return False

    def close(self):
        pass


def test_live_loop_learns_and_saves(tmp_path, monkeypatch):
    fake = types.ModuleType("robot_io")
    fake.Robot = _FakeRobot
    monkeypatch.setitem(sys.modules, "robot_io", fake)
    monkeypatch.setattr(H, "ANKLE_STATE", str(tmp_path / "state.json"))
    real_plan = H.plan

    def plan_spy(*a, **k):
        p = real_plan(*a, **k)
        _FakeRobot.sw = p["sway"]
        _FakeRobot.ankle = p["args"].get("ankle_steps")
        return p
    monkeypatch.setattr(H, "plan", plan_spy)
    log = tmp_path / "run.jsonl"
    rc = H.main(["--T", "4", "--dir", "fwd", "--steps", "6", "--live", "--ankle-learn", "on", "--ahead", "1",
                 "--poll", "0", "--log", str(log), "--seed", "5.2,3.4", "--heading-hold", "off"])
    assert rc == 0
    summ = [json.loads(l) for l in open(log) if '"summary"' in l][-1]
    al = summ["ankle_learn"]
    assert al["start"]["R"] == 2.0 and al["saved"]
    R = [s for s in summ["steps"] if s["stance"] == "R"]
    assert R[0]["lean_in"] > 7 and R[0]["A_next"] == 3.0          # the first R stance tipped at 2 -> 3
    assert any(s["A_used"] == 3.0 and s["lean_in"] < 5 for s in R[1:])   # a later R stance ran (clean) at 3
    st = json.load(open(tmp_path / "state.json"))["fwd"]
    assert st["R"] == al["end"]["R"]
    # every reported step carries the A it ran with and the next one
    assert all("A_next" in s for s in summ["steps"])
    assert all(re.match(r"track add ", l) for l in _FakeRobot.sent if l.startswith("track add"))
