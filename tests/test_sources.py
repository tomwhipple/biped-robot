"""Command-source tests.

Run:  .venv/bin/python -m pytest tests/test_sources.py -q

The theme: a source must only ever ask for things the policy was actually
trained to do. clamp_to_envelope() is the robot-side backstop, but a source
that constantly needs clamping is a source that is steering blind.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "link"))

import sources                                                   # noqa: E402
from protocol import (FLAG_ENABLE, V_MAX, V_MIN_WALK,           # noqa: E402
                      clamp_to_envelope)


# -- stick mapping ---------------------------------------------------------
def test_stick_never_emits_an_untrained_speed():
    for i in range(1001):
        v = sources.stick_to_speed(-1.0 + i * 0.002)
        assert v == 0.0 or V_MIN_WALK - 1e-9 <= abs(v) <= V_MAX + 1e-9


def test_stick_deadzone_and_full_travel():
    assert sources.stick_to_speed(0.05) == 0.0
    assert sources.stick_to_speed(1.0) == pytest.approx(V_MAX)
    assert sources.stick_to_speed(-1.0) == pytest.approx(-V_MAX)
    assert sources.stick_to_speed(0.16) == pytest.approx(V_MIN_WALK, abs=0.02)


# -- scripts ---------------------------------------------------------------
@pytest.mark.parametrize("name", sorted(sources.SCRIPTS))
def test_every_script_stays_in_the_trained_envelope(name):
    src = sources.ScriptSource(name)
    for i in range(400):                       # 20 s at 20 Hz
        out = src.poll(i * 0.05)
        v, w = out[0], out[1]
        assert clamp_to_envelope(v, w) == (v, w), \
            f"{name} at t={i*0.05}s emits {(v, w)}, which the robot clamps"
        if len(out) > 3:
            # dict scripts: the extended channels must survive the robot's
            # envelope clamp untouched too
            from protocol import Command, clamp_ext_to_envelope
            pkt = Command(seq=0, vx=v, wz=w, flags=out[2], **out[3])
            assert clamp_ext_to_envelope(pkt) == (
                pkt.vy, pkt.crouch, pkt.lift, pkt.foot_dx, pkt.foot_dz), \
                f"{name} at t={i*0.05}s emits ext {out[3]}, which the " \
                f"robot clamps"


def test_script_duration_disables_rather_than_stopping_the_stream():
    # Going quiet would make the robot wait out the 250 ms watchdog; sending
    # an explicit disabled command stops it now.
    src = sources.ScriptSource("walk", duration=5.0)
    assert src.poll(4.9)[2] & FLAG_ENABLE
    v, w, flags = src.poll(5.1)
    assert (v, w, flags) == (0.0, 0.0, 0)


def test_unknown_script_is_rejected_loudly():
    with pytest.raises(ValueError, match="unknown script"):
        sources.ScriptSource("moonwalk")


# -- goal seeker -----------------------------------------------------------
class FakePose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.pose = (x, y, yaw)

    def __call__(self):
        return self.pose


def test_goal_seeker_stays_inside_the_trained_envelope_everywhere():
    """The regression that actually bit: the first cut used k_yaw=1.5 and
    sprinted at V_MAX, so it commanded pivots at wz=1.0 -- the exact edge of
    cmd_w_range -- and the robot fell over repeatedly instead of arriving.
    """
    pose = FakePose()
    src = sources.GoalSource((2.0, 1.5), pose)
    for gx in (-3.0, -0.5, 0.0, 2.0, 5.0):
        for gy in (-3.0, 0.0, 1.5, 4.0):
            for yaw in [-math.pi + i * 0.2 for i in range(32)]:
                pose.pose = (0.0, 0.0, yaw)
                src.goal = (gx, gy)
                v, w, _ = src.poll(0.0)
                assert abs(w) <= sources.W_GOAL_MAX + 1e-9
                assert v == 0.0 or V_MIN_WALK - 1e-9 <= v <= sources.V_CRUISE
                assert clamp_to_envelope(v, w) == (v, w)


def test_goal_seeker_arcs_when_roughly_aimed():
    # Walking-while-turning is the best-trained regime; prefer it.
    src = sources.GoalSource((2.0, 1.0), FakePose(0.0, 0.0, 0.0))
    v, w, _ = src.poll(0.0)
    assert v > 0.0 and w > 0.0


def test_goal_seeker_pivots_only_when_the_goal_is_behind():
    src = sources.GoalSource((-2.0, 0.0), FakePose(0.0, 0.0, 0.0))
    v, w, _ = src.poll(0.0)
    assert v == 0.0 and abs(w) > 0.5


def test_goal_seeker_stands_on_arrival():
    src = sources.GoalSource((0.1, 0.0), FakePose(0.0, 0.0, 0.0))
    assert src.poll(0.0) == (0.0, 0.0, FLAG_ENABLE)


def test_goal_seeker_turns_the_short_way_around():
    # Robot at yaw -170 deg, goal bearing +170 deg. The short way is -20 deg
    # (out through -180 and round), NOT +340. Without the atan2(sin, cos)
    # wrap this spins the long way, and on a 28 cm biped that is a fall.
    src = sources.GoalSource((-2.0, 0.35),
                             FakePose(0.0, 0.0, math.radians(-170)))
    _, w, _ = src.poll(0.0)
    assert w == pytest.approx(math.radians(-20), abs=0.02)
