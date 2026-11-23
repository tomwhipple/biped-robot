"""--mimic-turn-ref (#263): the imitation reference turns with the hip yaws. Forward kinematics of
the reference, torso fixed: per gait cycle the body pose follows the stance soles (R stands on
phase (0, pi), L on (pi, 2 pi)) as SE(2) compositions, so a stance sole's yaw sweep relative to the
torso is the body's yaw. Without the option the hip yaws stay at home and the reference never yaws
the body; with it the body yaws wz / f per cycle (until the 0.35 rad sole-yaw cap or the shared
saturation scale binds), the turn's leading foot opens toe-out and the trailing one closes
parallel, the legs keep >= 10 mm apart and the reference stays inside the joint ranges. With no
turn command it IS the volume reference. Record: docs/design-v6/2026-11-23-volume-reference-turn.md.
CPU vs MJX: sim/mjx/parity_test.py R27."""
import math
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)

F = 0.75          # the night recipe's deploy clock (--gait-freq 0.7,0.8)
N = 121           # phases per cycle


@pytest.fixture(scope="module")
def envs():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    kw.update(mimic_lift=0.02, mimic_volume_ref=True)
    return BimoWalkerEnv(**kw), BimoWalkerEnv(**dict(kw, mimic_turn_ref=True))


def _put(env, q):
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    for i, k in enumerate(env._pi):
        d.qpos[m.jnt_qposadr[m.joint(names[k]).id]] = q[i]
    mujoco.mj_kinematics(m, d)


def _soles(env, q):
    """(L, R) sole (x, y, yaw) in the torso frame."""
    _put(env, q)
    m, d = env.model, env.data
    t = m.body("torso").id
    Rt = d.xmat[t].reshape(3, 3)
    out = []
    for g in env._sole_gids:
        p = Rt.T @ (d.geom_xpos[g] - d.xpos[t])
        R = Rt.T @ d.geom_xmat[g].reshape(3, 3)
        out.append((p[0], p[1], math.atan2(R[1, 0], R[0, 0])))
    return np.array(out)


def _se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def _cycle(env, vx, vy, wz, f=F, n=N):
    """Per-phase soles and the body's SE(2) displacement over one cycle (x, y, yaw)."""
    cmd = np.array([vx, vy, wz, 1.0, 0.0, 0.0, 0.0])
    ph = np.linspace(0.0, 2.0 * np.pi, n)
    P = np.array([_soles(env, env._mimic_ref(cmd, p, f)) for p in ph])
    h = (n - 1) // 2
    B = np.eye(3)
    for leg, a, b in ((1, 0, h), (0, h, n - 1)):
        B = B @ _se2(*P[a, leg]) @ np.linalg.inv(_se2(*P[b, leg]))
    return P, np.array([B[0, 2], B[1, 2], math.atan2(B[1, 0], B[0, 0])])


def _leg_geoms(env, side):
    m = env.model
    return [g for g in range(m.ngeom) if m.geom(g).name.startswith(side + "_")
            and ("col_" in m.geom(g).name or m.geom(g).name.endswith("_sole"))]


def _min_leg_distance(env, cmd, f=F, n=61):
    import mujoco
    pairs = [(a, b) for a in _leg_geoms(env, "L") for b in _leg_geoms(env, "R")]
    ft = np.zeros(6)
    best = 1e9
    for p in np.linspace(0.0, 2.0 * np.pi, n):
        _put(env, env._mimic_ref(cmd, p, f))
        for a, b in pairs:
            best = min(best, mujoco.mj_geomDistance(env.model, env.data, a, b, 0.1, ft))
    return best


TURNS = [(0.0, 0.0, 0.3), (0.0, 0.0, -0.6), (0.12, 0.0, 0.6), (0.2, 0.0, -0.8), (-0.2, 0.0, 0.6),
         (-0.15, 0.0, -0.9), (0.0, 0.08, 0.6), (0.0, -0.08, -0.6), (0.1, 0.1, -0.5),
         (-0.1, -0.08, 0.5)]


@pytest.mark.parametrize("vx,vy,wz", TURNS)
def test_body_yaw_per_cycle_is_the_command(envs, vx, vy, wz):
    vol, turn = envs
    _, d0 = _cycle(vol, vx, vy, wz)
    _, d1 = _cycle(turn, vx, vy, wz)
    assert abs(d0[2]) < 1e-6                       # the volume reference never yaws the body
    assert d1[2] == pytest.approx(wz / F, rel=0.02)


@pytest.mark.parametrize("vx,vy,wz", [(0.24, 0.0, 1.0), (-0.4, 0.0, 1.0), (0.0, 0.25, -1.0),
                                      (0.17, -0.17, 1.0), (0.0, 0.0, 1.0)])
def test_saturated_turns_still_turn_the_right_way(envs, vx, vy, wz):
    _, turn = envs
    _, d1 = _cycle(turn, vx, vy, wz, f=0.7)
    assert 0.4 < d1[2] / (wz / 0.7) <= 1.0 + 1e-6


@pytest.mark.parametrize("wz", [0.6, 1.0, -1.0])
def test_a_pivot_turns_in_place(envs, wz):
    _, turn = envs
    _, d1 = _cycle(turn, 0.0, 0.0, wz)
    assert math.hypot(d1[0], d1[1]) < 0.02         # m per cycle
    assert abs(d1[2]) > 0.75


@pytest.mark.parametrize("wz", [0.5, -0.5])
def test_lead_foot_opens_trail_foot_closes(envs, wz):
    _, turn = envs
    P, _ = _cycle(turn, 0.0, 0.0, wz)
    h = (N - 1) // 2
    y = abs(wz) / (4.0 * F)
    lead, trail = (0, 1) if wz > 0 else (1, 0)
    td_lead, td_trail = (h, 0) if wz > 0 else (0, h)   # L lands at pi, R at 2 pi (= 0)
    out = (1.0, -1.0)
    # the leading foot lands toe-out 2|Y|; the trailing one has turned toe-out 2|Y| under the body
    assert P[td_lead, lead, 2] * out[lead] == pytest.approx(2 * y, abs=0.01)
    assert P[td_lead, trail, 2] * out[trail] == pytest.approx(2 * y, abs=0.01)
    # the trailing foot lands parallel to the leading one, both straight ahead
    assert P[td_trail, lead, 2] == pytest.approx(0.0, abs=0.01)
    assert P[td_trail, trail, 2] == pytest.approx(0.0, abs=0.01)
    # never toe-in
    assert np.all(P[:, 0, 2] >= -1e-6) and np.all(P[:, 1, 2] <= 1e-6)


def _grid():
    out = []
    for hd in range(0, 360, 90):
        c, s = math.cos(math.radians(hd)), math.sin(math.radians(hd))
        for r in (0.5, 1.0):
            k = r / math.hypot(c / (0.24 if c >= 0 else 0.4), s / 0.25)
            for wz in (-1.0, 0.6):
                out.append((k * c, k * s, wz))
    return out + [(0.0, 0.0, 1.0), (0.0, 0.0, -0.6), (0.17, 0.17, -1.0)]


@pytest.mark.parametrize("vx,vy,wz", _grid())
def test_legs_apart_and_inside_the_joint_ranges(envs, vx, vy, wz):
    _, turn = envs
    cmd = np.array([vx, vy, wz, 1.0, 0.0, 0.0, 0.0])
    assert _min_leg_distance(turn, cmd, f=0.7) > 0.01            # m, no collision, no crossing
    lo, hi = turn._lo, turn._hi
    try:   # the reference before its final clip
        turn._lo, turn._hi = lo - 10.0, hi + 10.0
        q = np.array([turn._mimic_ref(cmd, p, f) for f in (0.7, 0.8)
                      for p in np.linspace(0.0, 2.0 * np.pi, 61)])
    finally:
        turn._lo, turn._hi = lo, hi
    assert np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)


@pytest.mark.parametrize("cmd", [(0.1, 0.0, 0.0), (-0.2, 0.15, 0.0), (0.0, 0.25, 0.0),
                                 (0.24, 0.25, 0.0), (0.0, 0.0, 0.0)])
def test_without_a_turn_it_is_the_volume_reference(envs, cmd):
    vol, turn = envs
    c = np.array([*cmd, 1.0, 0.0, 0.0, 0.0])
    for ph in np.linspace(-np.pi, np.pi, 37):
        assert np.array_equal(vol._mimic_ref(c, ph, 0.75), turn._mimic_ref(c, ph, 0.75))


def test_needs_the_volume_reference():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    with pytest.raises(ValueError, match="mimic_volume_ref"):
        BimoWalkerEnv(**dict(kw, mimic_lift=0.02, mimic_turn_ref=True))


def test_train_flag_reaches_the_env_kwargs():
    import train_mjx
    p = train_mjx.make_parser()
    kw = train_mjx.build_env_kw(p.parse_args(["--mimic-volume-ref", "--mimic-turn-ref"]))
    assert kw["mimic_turn_ref"] is True and kw["mimic_volume_ref"] is True
    assert "mimic_turn_ref" not in train_mjx.build_env_kw(p.parse_args(["--mimic-volume-ref"]))
    with pytest.raises(SystemExit):
        train_mjx.build_env_kw(p.parse_args(["--mimic-turn-ref"]))


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R27, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--mimic-turn-ref"],
                       cwd=os.path.join(SIM, "mjx"), env=env, capture_output=True, text=True,
                       timeout=1200)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
