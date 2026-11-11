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
from protocol import (FLAG_ENABLE, V_BACK_MAX, V_BACK_MIN,       # noqa: E402
                      V_FWD_MAX, V_FWD_MIN, VY_MAX, VY_MIN, CROUCH_MIN,
                      clamp_to_envelope)


# -- stick mapping ---------------------------------------------------------
def test_stick_never_emits_an_untrained_speed():
    for i in range(1001):
        v = sources.stick_to_speed(-1.0 + i * 0.002)
        if v == 0.0:
            continue
        lo, hi = (V_FWD_MIN, V_FWD_MAX) if v > 0 else (V_BACK_MIN, V_BACK_MAX)
        assert lo - 1e-9 <= abs(v) <= hi + 1e-9
        # ... and the robot leaves it alone
        assert clamp_to_envelope(v, 0.0)[0] == pytest.approx(v)


def test_stick_deadzone_and_full_travel():
    assert sources.stick_to_speed(0.05) == 0.0
    assert sources.stick_to_speed(1.0) == pytest.approx(V_FWD_MAX)
    assert sources.stick_to_speed(0.16) == pytest.approx(V_FWD_MIN, abs=0.02)
    if V_BACK_MAX > 0:   # backward travel maps onto the backward band
        assert sources.stick_to_speed(-1.0) == pytest.approx(-V_BACK_MAX)
        assert sources.stick_to_speed(-0.16) == pytest.approx(-V_BACK_MIN,
                                                              abs=0.02)
    else:
        assert sources.stick_to_speed(-1.0) == 0.0


def test_lateral_stick_maps_onto_the_sidestep_band():
    assert sources.stick_to_lateral(0.05) == 0.0
    assert sources.stick_to_lateral(1.0) == pytest.approx(VY_MAX)
    assert sources.stick_to_lateral(-1.0) == pytest.approx(-VY_MAX)
    assert sources.stick_to_lateral(0.16) == pytest.approx(VY_MIN, abs=0.01)
    assert sources.stick_to_lateral(-0.16) == pytest.approx(-VY_MIN, abs=0.01)


def test_stick_maps_onto_a_given_band(monkeypatch):
    """The mapping follows the generated band, whatever it is: a different
    envelope (a prec-style 0.05-0.24 forward, -0.4..-0.15 backward) and a run
    that never trained backward walks."""
    for fwd, back in (((0.05, 0.24), (0.15, 0.4)), ((0.3, 1.0), (0.0, 0.0))):
        monkeypatch.setattr(sources, "V_FWD_MIN", fwd[0])
        monkeypatch.setattr(sources, "V_FWD_MAX", fwd[1])
        monkeypatch.setattr(sources, "V_BACK_MIN", back[0])
        monkeypatch.setattr(sources, "V_BACK_MAX", back[1])
        assert sources.stick_to_speed(1.0) == pytest.approx(fwd[1])
        assert sources.stick_to_speed(0.1) == 0.0
        assert sources.stick_to_speed(-1.0) == pytest.approx(-back[1])
        monkeypatch.setattr(sources, "VY_MIN", 0.1)
        monkeypatch.setattr(sources, "VY_MAX", 0.25)
        assert sources.stick_to_lateral(-1.0) == pytest.approx(-0.25)
        if back[1] > 0:
            assert sources.stick_to_speed(-0.16) == pytest.approx(-back[0],
                                                                  abs=0.02)


# -- scripts ---------------------------------------------------------------
@pytest.mark.parametrize("name", sorted(sources.SCRIPTS))
def test_every_script_stays_in_the_trained_envelope(name):
    if name == "crouch1" and CROUCH_MIN >= 1.0:
        pytest.skip("the deployed run was trained at crouch 1 only "
                    "(cmd_crouch_range 1,1): the robot clamps this script's "
                    "crouch to 1.0, so it is a no-op until a crouch-trained "
                    "run is deployed")
    src = sources.ScriptSource(name)
    for i in range(400):                       # 20 s at 20 Hz
        out = src.poll(i * 0.05)
        v, w = out[0], out[1]
        assert len(out) == 4 and isinstance(out[3], dict)
        assert clamp_to_envelope(v, w) == (v, w), \
            f"{name} at t={i*0.05}s emits {(v, w)}, which the robot clamps"
        if out[3]:
            # dict scripts: the extended channels must survive the robot's
            # envelope clamp untouched too
            from protocol import Command, clamp_ext_to_envelope
            pkt = Command(seq=0, vx=v, wz=w, flags=out[2], **out[3])
            assert clamp_ext_to_envelope(pkt) == (
                pkt.vy, pkt.crouch, pkt.lift, pkt.foot_dx, pkt.foot_dz), \
                f"{name} at t={i*0.05}s emits ext {out[3]}, which the " \
                f"robot clamps"


# The slow-gait scripts: the band of the 0.75 Hz policies (robot_legs_prec16_f075_js20's
# generated envelope: forward 0.05-0.24, backward 0.15-0.4), whatever run is deployed here.
SLOW_BAND = __import__("protocol").Envelope(0.05, 0.24, 0.15, 0.4, 0.1, 0.25, 1.0, 1.0)


@pytest.mark.parametrize("name,vx,cycles", [("stride1_02s", 0.2, 1.0),
                                            ("steps5_02s", 0.2, 2.5),
                                            ("back3_02s", -0.2, 1.5)])
def test_slow_gait_scripts_walk_whole_steps_on_the_075hz_clock(name, vx, cycles):
    """2 s of stand, then vx for `cycles` gait cycles at 0.75 Hz less 50 ms (ending before
    the cycle boundary, so the clock does not coast through an extra swing), then stand.
    Inside the 0.75 Hz policies' band, which the robot would not clamp."""
    src = sources.ScriptSource(name)
    end = 2.0 + cycles / 0.75 - 0.05
    assert sources.SCRIPT_SCENARIO[name] == "line_1m"
    for i in range(2000):                      # 20 s at 100 Hz
        t = i * 0.01
        v, w, flags, ext = src.poll(t)
        assert (w, flags, ext) == (0.0, FLAG_ENABLE, {})
        want = vx if 2.0 <= t < end else 0.0
        assert v == pytest.approx(want), f"{name} at t={t:.2f}s"
        assert clamp_to_envelope(v, w, SLOW_BAND) == (v, w)
    assert src.poll(end - 0.005)[0] == pytest.approx(vx)
    assert src.poll(end + 0.005)[0] == 0.0


def test_stride1_03e_ends_57_ms_before_its_1p5hz_cycle():
    src = sources.ScriptSource("stride1_03e")
    assert src.poll(1.99)[0] == 0.0
    assert src.poll(2.0)[0] == pytest.approx(0.3)
    assert src.poll(2.605)[0] == pytest.approx(0.3)
    assert src.poll(2.61)[0] == 0.0
    assert 2.0 + 1.0 / 1.5 - 2.61 == pytest.approx(0.0567, abs=1e-3)


def test_script_duration_disables_rather_than_stopping_the_stream():
    # Going quiet would make the robot wait out the 250 ms watchdog; sending
    # an explicit disabled command stops it now.
    src = sources.ScriptSource("walk", duration=5.0)
    assert src.poll(4.9)[2] & FLAG_ENABLE
    assert src.poll(5.1) == (0.0, 0.0, 0, {})


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
    sprinted at V_FWD_MAX, so it commanded pivots at wz=1.0 -- the exact edge of
    cmd_w_range -- and the robot fell over repeatedly instead of arriving.
    """
    pose = FakePose()
    src = sources.GoalSource((2.0, 1.5), pose)
    for gx in (-3.0, -0.5, 0.0, 2.0, 5.0):
        for gy in (-3.0, 0.0, 1.5, 4.0):
            for yaw in [-math.pi + i * 0.2 for i in range(32)]:
                pose.pose = (0.0, 0.0, yaw)
                src.goal = (gx, gy)
                v, w, _, _ = src.poll(0.0)
                assert abs(w) <= sources.W_GOAL_MAX + 1e-9
                assert v == 0.0 or V_FWD_MIN - 1e-9 <= v <= sources.V_CRUISE
                assert clamp_to_envelope(v, w) == (v, w)


def test_goal_seeker_arcs_when_roughly_aimed():
    # Walking-while-turning is the best-trained regime; prefer it.
    src = sources.GoalSource((2.0, 1.0), FakePose(0.0, 0.0, 0.0))
    v, w, _, _ = src.poll(0.0)
    assert v > 0.0 and w > 0.0


def test_goal_seeker_pivots_only_when_the_goal_is_behind():
    src = sources.GoalSource((-2.0, 0.0), FakePose(0.0, 0.0, 0.0))
    v, w, _, _ = src.poll(0.0)
    assert v == 0.0 and abs(w) > 0.5


def test_goal_seeker_stands_on_arrival():
    src = sources.GoalSource((0.1, 0.0), FakePose(0.0, 0.0, 0.0))
    assert src.poll(0.0) == (0.0, 0.0, FLAG_ENABLE, {})


def test_goal_seeker_turns_the_short_way_around():
    # Robot at yaw -170 deg, goal bearing +170 deg. The short way is -20 deg
    # (out through -180 and round), NOT +340. Without the atan2(sin, cos)
    # wrap this spins the long way, and on a 28 cm biped that is a fall.
    src = sources.GoalSource((-2.0, 0.35),
                             FakePose(0.0, 0.0, math.radians(-170)))
    _, w, _, _ = src.poll(0.0)
    assert w == pytest.approx(math.radians(-20), abs=0.02)


# -- march5: marching in place at the trained gait clock ------------------------------
def _march_run(name="march5", dt=0.01, t_end=12.0):
    src = sources.ScriptSource(name)
    rows = []
    for i in range(int(t_end / dt)):
        t = i * dt
        out = src.poll(t)
        ext = out[3] if len(out) > 3 else {}
        rows.append((t, out[0], out[1], ext.get("lift", 0.0), ext.get("foot_dz", 0.0)))
    return rows


def test_march5_is_five_full_swings_at_the_trained_cadence_then_the_stand():
    rows = _march_run()
    hz = sources.MARCH_HZ
    t0, T = sources.MARCH_T0, sources.march_swings_s(5)
    assert T == pytest.approx(2.75 / hz)                     # a quarter cycle + 5 half cycles
    lift = [(t, c4) for t, _, _, c4, _ in rows if c4 != 0.0]
    assert lift[0][0] == pytest.approx(t0, abs=0.011) and lift[-1][0] < t0 + T
    assert all(c4 == 0.0 and dz == 0.0 for t, _, _, c4, dz in rows if t < t0 or t >= t0 + T)
    # no velocity command anywhere: a march is in place
    assert all(v == 0.0 and w == 0.0 for _, v, w, _, _ in rows)
    # the swings: runs of one sign, left first (half-way through its swing), then R L R L R full ones
    runs = []
    for t, c4 in lift:
        if not runs or runs[-1][0] != c4:
            runs.append([c4, t, t])
        runs[-1][2] = t
    assert [r[0] for r in runs] == [-1.0, +1.0, -1.0, +1.0, -1.0, +1.0]
    lens = [r[2] - r[1] for r in runs]
    assert lens[0] == pytest.approx(0.25 / hz, abs=0.03)
    assert lens[1:-1] == pytest.approx([0.5 / hz] * 4, abs=0.03)   # each full swing: a half cycle
    assert lens[-1] == pytest.approx(0.5 / hz, abs=0.03)


def test_march5_foot_command_follows_the_swing_envelope():
    rows = _march_run()
    dz = [r[4] for r in rows if r[3] != 0.0]
    assert max(dz) == pytest.approx(sources.MARCH_DZ, abs=1e-4) and min(dz) >= 0.0
    # c6 returns to ~0 where the lifted leg changes (the swing's own envelope)
    prev = None
    for t, _, _, c4, d in rows:
        if prev is not None and c4 != 0.0 and c4 != prev and prev != 0.0:
            assert d < 0.15 * sources.MARCH_DZ
        prev = c4 if c4 != 0.0 else prev


def test_march5_command_is_the_trained_command_at_the_same_clock_phase():
    """The script's time maps onto the gait clock the way the env's does (the clock runs from the
    stand phase pi/2 the tick a lift is commanded); at every phase the env's own march command
    (env_mjx / walker_env traj_on == 4) equals the script's."""
    sys.path[:0] = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), p)
                    for p in ("sim", "sim/mjx")]
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    import numpy as np
    import robot_plant
    import walker_env as W
    kw = robot_plant.robot_env_kwargs("bimo_biped_v6legs_meas_sole.xml")
    kw.update(ext_cmd=True, gait_clock=True, march_mix=1.0, ext_mix=(0.0,) * 7, march_lift=0.08,
              gait_freq_range=(0.75, 0.75), command_mode=True, domain_rand=False)
    kw.pop("cmd_fixed", None)
    e = W.BimoWalkerEnv(**kw)
    e.reset(seed=4)
    z = np.zeros(e.action_space.shape[0], dtype=np.float32)
    checked = 0
    for _ in range(120):
        e.step(z)
        if e._traj_on != 4.0 or e._cmd[4] == 0.0:
            continue
        tm = ((float(e._gait_phase) - math.pi / 2) / (2 * math.pi * 0.75)) % (1 / 0.75)
        lift, dz = sources.march_cmd(tm, 0.75, e._march_dz)
        assert e._cmd[4] == lift and e._cmd[6] == pytest.approx(dz, abs=1e-6)
        checked += 1
    assert checked > 80 and e._march_dz == pytest.approx(sources.MARCH_DZ)
