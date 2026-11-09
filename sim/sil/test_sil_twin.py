"""sim/sil_twin.py's fall guard: the robot's armed-loop FALLEN behaviour.

    JAX_PLATFORMS=cpu .venv/bin/pytest sim/sil/test_sil_twin.py -q

The twin runs fallguard::Detector's fall half with Motion::kPolicy limits
(read from the firmware header) and, on a trip, does what ctrl_task.cpp
does: the run ends, torque goes off, the loop benches and the BENCH beacon
carries DISARMED_FALL. The guard and the link composition need nothing but
the repo; the plant test needs the deployed run's config.json and skips
without it.
"""
import json
import math
import os
import sys

import numpy as np
import pytest

os.environ.setdefault("JAX_PLATFORMS", "cpu")

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
for p in (HERE, SIM, os.path.join(SIM, "mjx")):
    if p not in sys.path:
        sys.path.insert(0, p)

import harness as H                                          # noqa: E402

# sil_twin picks a headless GL backend at import (MUJOCO_GL=egl off macOS).
# Nothing here renders; keep the rest of the suite's environment as it was.
_gl = os.environ.get("MUJOCO_GL")
import sil_twin                                              # noqa: E402
if _gl is None:
    os.environ.pop("MUJOCO_GL", None)
else:
    os.environ["MUJOCO_GL"] = _gl
import protocol as P                                         # noqa: E402  (sil_twin put link/ on the path)

DT_MS = 20                                  # the 50 Hz control tick
UPRIGHT, GYRO0 = (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)
DOWN = (0.0, math.sin(math.radians(60)), math.cos(math.radians(60)))  # 60 deg


def _up(tilt_deg):
    return (0.0, math.sin(math.radians(tilt_deg)), math.cos(math.radians(tilt_deg)))


def _feed(g, ups, t0=0):
    """Feed one sample per tick; the tick index each trip happened at."""
    return [k for k, up in enumerate(ups)
            if g.update(up, GYRO0, t0 + k * DT_MS)]


# =====================================================================
#  the limits are the firmware's
# =====================================================================
def test_limits_come_from_the_firmware_header():
    up_z, debounce = sil_twin.fallguard_limits()
    assert up_z == pytest.approx(math.cos(math.radians(45)), abs=1e-4)
    assert debounce == 200
    g = sil_twin.FallGuard()
    assert (g.fall_up_z, g.debounce_ms) == (up_z, debounce)


def test_limits_missing_header_is_loud(tmp_path):
    with pytest.raises(SystemExit):
        sil_twin.fallguard_limits(str(tmp_path / "nope.h"))


# =====================================================================
#  Detector semantics (firmware/host/test_fallguard.cpp, fall half)
# =====================================================================
def test_trips_after_the_debounce_not_before():
    g = sil_twin.FallGuard()
    # first below-line sample at tick 0, 200 ms later is tick 10
    assert _feed(g, [DOWN] * 30) == [10]
    assert g.fallen
    assert g.tilt_deg == pytest.approx(60.0, abs=0.01)


def test_45_deg_is_the_line():
    assert _feed(sil_twin.FallGuard(), [_up(44.0)] * 30) == []
    assert _feed(sil_twin.FallGuard(), [_up(46.0)] * 30) == [10]


def test_an_upright_sample_restarts_the_debounce():
    g = sil_twin.FallGuard()
    ups = [DOWN] * 10 + [UPRIGHT] + [DOWN] * 20     # 180 ms, then a footfall
    assert _feed(g, ups) == [11 + 10]


def test_an_invalid_sample_breaks_the_run_and_decides_nothing():
    g = sil_twin.FallGuard()
    nan = (float("nan"), 0.0, 0.0)
    assert _feed(g, [DOWN] * 10 + [nan] + [DOWN] * 20) == [21]
    g = sil_twin.FallGuard()
    assert not g.update(DOWN, (float("inf"), 0.0, 0.0), 0)
    assert not g.fallen


def test_latched_until_cleared():
    g = sil_twin.FallGuard()
    assert _feed(g, [DOWN] * 11) == [10]
    assert _feed(g, [DOWN] * 30, t0=1000) == []      # one trip per fall
    assert g.fallen
    g.clear()
    assert not g.fallen
    assert _feed(g, [DOWN] * 11, t0=5000) == [10]


# =====================================================================
#  the twin's composition: a fall ends the run like the robot's
# =====================================================================
def _frame(seq, flags, vx=0.4):
    return P.decode_command(P.encode_command(seq, vx, 0.0, flags))


def _armed_supervisor():
    sup = P.Supervisor()
    on = P.FLAG_ENABLE | P.FLAG_ARM
    sup.accept(_frame(1, P.FLAG_ENABLE), 0.0)          # level low ...
    sup.accept(_frame(2, on), 20.0)                    # ... rising edge arms
    assert sup.armed and sup.state(20.0) is P.LinkState.LIVE
    return sup, on


def test_a_fall_benches_with_disarmed_fall_and_torque_off():
    sup, on = _armed_supervisor()
    g = sil_twin.FallGuard()
    seq, tripped_at = 3, None
    for k in range(30):
        now = 40.0 + k * DT_MS
        sup.accept(_frame(seq, on), now)               # operator still driving
        seq += 1
        if g.tick(sup, DOWN, GYRO0, k * DT_MS):
            tripped_at = k
            break
        assert sup.state(now) is P.LinkState.LIVE and sup.torque_on(now)
    assert tripped_at == 10
    now = 40.0 + tripped_at * DT_MS
    assert not sup.armed
    assert sup.state(now) is P.LinkState.BENCH
    assert not sup.torque_on(now)
    # the beacon: BENCH, diag in seq_echo saying why -- what wifi_link sends
    tl = P.decode_telemetry(P.encode_telemetry(P.Telemetry(
        seq_echo=sup.seq_echo(now), state=sup.state(now), vbat_v=11.4,
        up_z=DOWN[2], vx_est=0.0, wz_est=0.0, servo_err=0,
        loop_late_pct=0)))
    assert tl.state is P.LinkState.BENCH
    assert P.diag_arm_result(tl.seq_echo) is P.ArmResult.DISARMED_FALL
    assert "FALL" in P.diag_reason(tl.seq_echo)


def test_after_a_fall_only_a_fresh_arm_edge_rearms():
    sup, on = _armed_supervisor()
    g = sil_twin.FallGuard()
    for k in range(11):
        g.tick(sup, DOWN, GYRO0, k * DT_MS)
    assert not sup.armed
    # the commander keeps holding ARM high: no edge, still benched, and a
    # benched loop checks nothing
    for k in range(20):
        sup.accept(_frame(100 + k, on), 1000.0 + k * 50)
        assert not g.tick(sup, DOWN, GYRO0, 1000 + k * DT_MS)
    assert not sup.armed
    assert P.diag_arm_result(sup.diag()) is P.ArmResult.DISARMED_FALL
    # a deliberate re-arm: 1 -> 0 -> 1; the twin clears the latch on the edge
    sup.accept(_frame(200, P.FLAG_ENABLE), 3000.0)
    sup.accept(_frame(201, on), 3020.0)
    assert sup.armed and sup.state(3020.0) is P.LinkState.LIVE
    g.clear()
    # still lying down: the fresh run ends the same way 200 ms later
    assert [k for k in range(15)
            if g.tick(sup, DOWN, GYRO0, 4000 + k * DT_MS)] == [10]
    assert not sup.armed


def test_fell_while_benched_changes_nothing():
    sup = P.Supervisor()
    sup.fell()
    assert sup.result is P.ArmResult.NONE and not sup.armed


# =====================================================================
#  on the plant: the attitude the twin feeds the guard
# =====================================================================
@pytest.fixture(scope="module")
def cfg():
    why = H.missing_run_artifacts(H.DEFAULT_RUN, ("config.json",))
    if why:
        pytest.skip(why)
    with open(os.path.join(H.run_dir(H.DEFAULT_RUN), "config.json")) as f:
        return json.load(f)


def test_a_toppled_plant_trips_the_guard_through_the_obs_frame(cfg):
    """The twin feeds the guard the obs frame's up and gyro (harness
    split_obs: what the firmware stack is fed). A torso laid over on the
    plant must read below the line there and trip exactly one debounce
    after the first below-line tick; the standing plant must not."""
    import mujoco
    from eval_precision import make_env
    env = make_env(cfg, 1e9, True, sil_twin._local_xml(cfg.get("xml_path")))
    if H.SPEC.frame_dim != env.observation_space.shape[0] // H.SPEC.hist_len:
        pytest.skip("the run's obs layout is not the compiled spec's")
    act0 = np.zeros(env.action_space.shape)

    obs, _ = env.reset(seed=0)
    g = sil_twin.FallGuard()
    for k in range(25):                          # standing, torque on
        f = H.split_obs(obs)
        assert not g.update(f["up"], f["gyro"], k * DT_MS)
        obs, *_ = env.step(act0)

    obs, _ = env.reset(seed=0)
    free = [j for j in range(env.model.njnt)
            if env.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    assert free, "the walking plant has no free joint"
    root = int(env.model.jnt_qposadr[free[0]])
    a = math.radians(80)                         # laid over on its side
    env.data.qpos[root + 3:root + 7] = (math.cos(a / 2), math.sin(a / 2), 0, 0)
    mujoco.mj_forward(env.model, env.data)
    env.set_torque_enabled(False)
    g = sil_twin.FallGuard()
    first_below = tripped = None
    for k in range(60):
        obs, *_ = env.step(act0)
        f = H.split_obs(obs)
        if first_below is None and f["up"][2] < g.fall_up_z:
            first_below = k
        if g.update(f["up"], f["gyro"], k * DT_MS):
            tripped = k
            break
    assert first_below is not None, "the toppled torso never read past 45 deg"
    assert tripped == first_below + g.debounce_ms // DT_MS
