"""--mimic-volume-ref (#263): the imitation reference's foot placement over the whole command
volume (--cmd-volume). Forward kinematics of the reference, torso fixed: per gait cycle the body
travels minus the stance soles' sweep (R stands on phase (0, pi), L on (pi, 2 pi)). The option
keeps the stance width at or above home (lead foot out, trail foot closes, as track_host's
sideways step) where the cardinal-band reference swung both feet -/+ about home and landed the
closing foot on or across the other; it keeps a diagonal's heading and a back + turn's stride
difference through saturation; and off a lateral or saturated command it IS the cardinal-band
reference. Record: docs/design-v6/2026-11-22-volume-reference-foot-placement.md."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

F = 0.75          # the night recipe's deploy clock (--gait-freq 0.7,0.8)
N = 181           # phases per cycle


@pytest.fixture(scope="module")
def envs():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    kw.update(mimic_lift=0.02)
    return BimoWalkerEnv(**kw), BimoWalkerEnv(**dict(kw, mimic_volume_ref=True))


def _soles(env, q):
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    for i, k in enumerate(env._pi):
        d.qpos[m.jnt_qposadr[m.joint(names[k]).id]] = q[i]
    mujoco.mj_kinematics(m, d)
    t = m.body("torso").id
    R = d.xmat[t].reshape(3, 3)
    return np.array([R.T @ (d.geom_xpos[g] - d.xpos[t]) for g in env._sole_gids])


def _cycle(env, vx, vy, wz, f=F):
    cmd = np.array([vx, vy, wz, 1.0, 0.0, 0.0, 0.0])
    ph = np.linspace(0.0, 2.0 * np.pi, N)
    P = np.array([_soles(env, env._mimic_ref(cmd, p, f)) for p in ph])
    h = (N - 1) // 2
    travel = -((P[h, 1, :2] - P[0, 1, :2]) + (P[-1, 0, :2] - P[h, 0, :2]))
    width = P[:, 0, 1] - P[:, 1, 1]
    return P, travel, width


def _volume_grid():
    """Eight headings x three radii of the default envelope (fwd 0.24, back 0.4, lat 0.25)."""
    out = []
    for hd in range(0, 360, 45):
        c, s = math.cos(math.radians(hd)), math.sin(math.radians(hd))
        for r in (0.15, 0.5, 1.0):
            k = r / math.hypot(c / (0.24 if c >= 0 else 0.4), s / 0.25)
            out.append((hd, k * c, k * s))
    return out


@pytest.mark.parametrize("hd,vx,vy", _volume_grid())
@pytest.mark.parametrize("wz", [0.0, 0.6])
def test_stance_never_narrows_and_heading_holds(envs, hd, vx, vy, wz):
    _, env = envs
    home = _soles(env, env._default)
    w0 = home[0, 1] - home[1, 1]
    _, travel, width = _cycle(env, vx, vy, wz)
    assert width.min() >= w0 - 1e-3                    # no inward step past home: no collision
    want = math.degrees(math.atan2(vy, vx))
    got = math.degrees(math.atan2(travel[1], travel[0]))
    assert abs((got - want + 180.0) % 360.0 - 180.0) < 5.0
    ratio = np.linalg.norm(travel) / (math.hypot(vx, vy) / F)
    assert 0.65 < ratio < 1.5                          # grade.py's compass band is 0.5-1.5x


@pytest.mark.parametrize("vy", [0.06, 0.15, -0.06, -0.15])
def test_lead_foot_out_trail_foot_closes(envs, vy):
    _, env = envs
    home = _soles(env, env._default)
    w0 = home[0, 1] - home[1, 1]
    P, travel, width = _cycle(env, 0.0, vy, 0.0)
    h = (N - 1) // 2
    w_ltd, w_rtd = width[h], width[0]                  # L lands at pi, R at 2 pi (= 0)
    lead, trail = (w_ltd, w_rtd) if vy > 0 else (w_rtd, w_ltd)
    assert trail == pytest.approx(w0, abs=1e-3)        # the trailing foot closes to home width
    assert lead - w0 == pytest.approx(abs(travel[1]), rel=0.02)   # the leading one steps out by a cycle's travel


def test_cardinal_reference_collides_where_the_volume_one_does_not(envs):
    # the finding behind the option: a 0.125 m/s sidestep brings the cardinal reference's soles
    # (84 mm wide, 121 mm apart at home) together
    base, vol = envs
    sole_w = 2.0 * float(base.model.geom_size[base._sole_gids[0]][1])
    assert _cycle(base, 0.0, 0.125, 0.0)[2].min() < sole_w
    assert _cycle(vol, 0.0, 0.125, 0.0)[2].min() > sole_w


def test_back_turn_keeps_the_stride_difference(envs):
    # (-0.4, wz 1) at 0.75 Hz: both legs past the 0.45 rad clip, so the cardinal reference drops
    # the turn; the volume one scales both strides together and keeps the differential (the
    # inside, L, leg of a left turn going back travels further back: v_L = vx - 0.028 wz)
    base, vol = envs
    h = (N - 1) // 2

    def per_leg(env):
        P, _, _ = _cycle(env, -0.4, 0.0, 1.0)
        return -(P[-1, 0, 0] - P[h, 0, 0]), -(P[h, 1, 0] - P[0, 1, 0])   # body travel on L, R stance
    tl0, tr0 = per_leg(base)
    tl1, tr1 = per_leg(vol)
    assert tl0 == pytest.approx(tr0, abs=1e-6)
    assert tl1 < tr1 - 0.01


@pytest.mark.parametrize("cmd", [(0.1, 0.0, 0.0), (-0.2, 0.0, 0.0), (0.15, 0.0, 0.5), (0.0, 0.0, 0.8),
                                 (0.0, 0.0, 0.0)])
def test_off_the_lateral_axis_and_unsaturated_it_is_the_cardinal_reference(envs, cmd):
    base, vol = envs
    c = np.array([*cmd, 1.0, 0.0, 0.0, 0.0])
    for ph in np.linspace(-np.pi, np.pi, 37):
        assert np.array_equal(base._mimic_ref(c, ph, 1.5), vol._mimic_ref(c, ph, 1.5))
