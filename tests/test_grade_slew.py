"""grade.py runs a slew-trained run's command through the firmware's slew (#266;
experiments/policy/shaper-train): every tick the probe's command goes through
cmd_kernel.CommandSlew at the run's recorded rates, from the stand at the arm, before the clock and
the frame (ctrl_task.cpp's order), and walker_env's own slew is off. A run without cmd_slew sees
each command as it is set (the PR's bit-identity proof covers grade.py's numbers on recorded runs)."""
import json
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
for p in ("experiments/policy/shaper-train", "experiments/policy/stand-still", "experiments/policy/rca",
          "link", "sim"):
    sys.path.insert(0, os.path.join(ROOT, p))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import grade as G  # noqa: E402
import cmd_kernel as ck  # noqa: E402

RATES = (0.5, 0.4, 1.5)
DEPLOYED = os.path.join(ROOT, "firmware", "deployed", "runs", "robot_legs_walk16_act_s128r24")


def test_fw_command_slews_the_planar_channels_only():
    s = ck.CommandSlew(RATES, G.DT)
    c = (0.24, -0.25, 0.8, 0.9, 1.0, 0.01, 0.02)
    out = [G.fw_command(s, c) for _ in range(40)]
    d = ck.slew_delta(RATES, G.DT)
    assert out[0][:3] == tuple(float(x) for x in d * np.sign(c[:3]))
    assert all(o[3:] == c[3:] for o in out)
    assert out[-1][:3] == tuple(float(np.float32(x)) for x in c[:3])     # lands exactly


@pytest.fixture
def slew_run(tmp_path, monkeypatch):
    """The deployed run (tracked inputs) with cmd_slew in its config.json, as grade finds it."""
    if not os.path.exists(os.path.join(DEPLOYED, "params.pkl")):
        pytest.skip("no deployed run inputs")
    name = "slew_probe_run"
    d = tmp_path / name
    d.mkdir()
    cfg = json.load(open(os.path.join(DEPLOYED, "config.json")))
    cfg["cmd_slew"] = list(RATES)
    (d / "config.json").write_text(json.dumps(cfg))
    os.symlink(os.path.join(DEPLOYED, "params.pkl"), d / "params.pkl")
    root0 = G.simh.run_root
    monkeypatch.setattr(G.simh, "run_root", lambda run: str(tmp_path) if run == name else root0(run))
    return name


def test_a_slew_run_is_graded_on_the_firmwares_ramp(slew_run, monkeypatch):
    """A stick step mid-probe reaches the frame as cmdslew's ramp, tick for tick, with the clock and
    the frame reading the same slewed command (ctrl_task's order); walker_env's own slew is off."""
    seen = {}
    build0 = G.simh.build

    def build(*a, **k):
        env, act, zoff = build0(*a, **k)
        seen["env"] = env
        step0 = env.step
        seen["cmd"] = []

        def step(a_):
            if seen.get("armed"):                            # the episode loop, not the settle
                seen["cmd"].append(np.array(env._cmd[:3]))  # what this tick's frame and clock read
            return step0(a_)
        env.step = step
        return env, act, zoff
    hold0 = G.simh.place_and_hold

    def place_and_hold(*a, **k):
        hold0(*a, **k)
        seen["armed"] = True
    monkeypatch.setattr(G.simh, "build", build)
    monkeypatch.setattr(G.simh, "place_and_hold", place_and_hold)
    walk = (0.2, 0.15, 0.6, 1.0, 0.0, 0.0, 0.0)
    sched = [(0.0, G.STAND_CMD), (0.5, walk), (1.5, G.STAND_CMD)]
    v = dict(run=slew_run, shaper_hz=12.0, stand_phase=G.P2, clock="coast", cmd_schedule=sched)
    G.episode(v, seed=0, run_s=2.5)
    env = seen["env"]
    assert env.cmd_slew is None                       # the env's own slew is off
    got = np.asarray(seen["cmd"])
    # the replay: the first armed tick before the loop, then one tick per step, the target switching
    # on the episode loop's grid
    s = ck.CommandSlew(RATES, env.control_dt)
    s.step(*G.STAND_CMD[:3])
    tgt, want = G.STAND_CMD, []
    for i in range(len(got)):
        for ts, c in sched:
            if int(round(ts / G.DT)) == i + 1:
                tgt = c
        want.append(s.step(*tgt[:3]))
    assert np.array_equal(got, np.asarray(want, np.float64))
    d = ck.slew_delta(RATES, env.control_dt)
    assert np.all(np.abs(np.diff(got, axis=0)) <= d + 1e-6)         # a ramp, never a step
    assert np.allclose(got.max(axis=0), walk[:3], atol=1e-6)          # it arrives (0.2 / 0.5 = 0.4 s)
    assert np.all(got[-5:] == 0.0)                                   # and ramps back to the stand
    assert math.isclose(env.cmd_stand_band, 0.05)                    # the run's band: none recorded
