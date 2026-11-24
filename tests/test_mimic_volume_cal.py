"""--mimic-volume-cal (#263): the volume imitation reference's travel calibrated to the command.
Forward kinematics of the reference, torso fixed: per gait cycle the body pose follows the stance
soles (R stands on phase (0, pi), L on (pi, 2 pi)) as SE(2) compositions. With the fixed stride
gains (hip pitch 1.4 v/f, hip roll 1.1 vy/f) the reference travels 1.2-1.44x the command below
saturation; with the option each stride amplitude is inverted from the plant's own FK travel
table (walker_env.mimic_travel_cal), so it travels 1.00x the command up to the unchanged
saturation scale. Past saturation it IS the uncalibrated volume reference (the same caps bind).
Record: docs/design-v6/2026-11-24-volume-reference-travel-calibration.md (its tables come from
`python tests/test_mimic_volume_cal.py`). CPU vs MJX: sim/mjx/parity_test.py R28."""
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


def _make(**extra):
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    kw.update(mimic_lift=0.02, mimic_volume_ref=True)
    kw.update(extra)
    return BimoWalkerEnv(**kw)


@pytest.fixture(scope="module")
def envs():
    """(volume ref, + cal, + turn, + turn + cal)."""
    return (_make(), _make(mimic_volume_cal=True), _make(mimic_turn_ref=True),
            _make(mimic_turn_ref=True, mimic_volume_cal=True))


def _soles(env, q):
    """(L, R) sole (x, y, yaw) in the torso frame."""
    import mujoco
    m, d = env.model, env.data
    d.qpos[:] = m.qpos0
    names = [m.joint(int(j)).name for j in m.actuator_trnid[:, 0]]
    for i, k in enumerate(env._pi):
        d.qpos[m.jnt_qposadr[m.joint(names[k]).id]] = q[i]
    mujoco.mj_kinematics(m, d)
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


def cycle(env, vx, vy, wz, f=F, n=N):
    """The body's SE(2) displacement over one gait cycle (x, y, yaw) and the stance width
    (L - R sole centres) per phase."""
    cmd = np.array([vx, vy, wz, 1.0, 0.0, 0.0, 0.0])
    ph = np.linspace(0.0, 2.0 * np.pi, n)
    P = np.array([_soles(env, env._mimic_ref(cmd, p, f)) for p in ph])
    h = (n - 1) // 2
    B = np.eye(3)
    for leg, a, b in ((1, 0, h), (0, h, n - 1)):
        B = B @ _se2(*P[a, leg]) @ np.linalg.inv(_se2(*P[b, leg]))
    return np.array([B[0, 2], B[1, 2], math.atan2(B[1, 0], B[0, 0])]), P[:, 0, 1] - P[:, 1, 1]


def speed_dir(disp, vx, vy, wz, f=F):
    """(speed x, direction error deg) of a cycle's chord against a constant-curvature path at
    the command: the chord of an arc turning th = wz / f is |v|/f * sin(th/2) / (th/2), headed
    at the command's heading + th/2."""
    th = wz / f
    k = 1.0 if abs(th) < 1e-9 else math.sin(th / 2.0) / (th / 2.0)
    want = math.hypot(vx, vy) / f * k
    x = math.hypot(disp[0], disp[1]) / want
    err = math.degrees(math.atan2(disp[1], disp[0]) - math.atan2(vy, vx)) - math.degrees(th / 2.0)
    return x, (err + 180.0) % 360.0 - 180.0


def saturated(env, vx, vy, wz, f=F):
    """True when the calibrated reference's saturation scale binds on this command."""
    from walker_env import MIMIC_TURN_ROOM, mimic_cal_stride
    tl = (vx + np.array([-1.0, 1.0]) * 0.028 * wz) / f
    A = [mimic_cal_stride(tl[i], env._cal_ta[i], env._cal_a, env._cal_ka[i]) for i in range(2)]
    B = mimic_cal_stride(vy / f, env._cal_tb, env._cal_b, env._cal_kb)
    from walker_env import MIMIC_A_MAX
    out = max(abs(a) for a in A) > MIMIC_A_MAX or abs(B) > env._vref_bmax
    if env.mimic_turn_ref:
        Y = wz / (4.0 * f)
        out = out or abs(Y) > env._vref_ymax or abs(B) + MIMIC_TURN_ROOM * abs(Y) > env._vref_bmax
    return out


def volume_grid():
    """Eight headings x three radii of the default envelope (fwd 0.24, back 0.4, lat 0.25)."""
    out = []
    for hd in range(0, 360, 45):
        c, s = math.cos(math.radians(hd)), math.sin(math.radians(hd))
        for r in (0.15, 0.5, 1.0):
            k = r / math.hypot(c / (0.24 if c >= 0 else 0.4), s / 0.25)
            out.append((hd, r, k * c, k * s))
    return out


@pytest.mark.parametrize("hd,r,vx,vy", volume_grid())
@pytest.mark.parametrize("wz", [0.0, 0.6])
def test_travel_matches_the_command_below_saturation(envs, hd, r, vx, vy, wz):
    vol, cal, _, _ = envs
    disp, width = cycle(cal, vx, vy, wz)
    if saturated(cal, vx, vy, wz):
        # past the cap: the same caps bind, later than the fixed gains' (which saturate the
        # stride at a lower command), so it travels at least as far and still not past 1.0x
        x0, _ = speed_dir(cycle(vol, vx, vy, wz)[0], vx, vy, 0.0)
        x1, _ = speed_dir(disp, vx, vy, 0.0)
        assert x0 - 0.01 < x1 < 1.02
        return
    # no turn reference: the body does not yaw, so the chord is the straight travel
    x, err = speed_dir(disp, vx, vy, 0.0)
    assert x == pytest.approx(1.0, abs=0.02)
    assert abs(err) < 1.0
    home = _soles(cal, cal._default)
    assert width.min() >= home[0, 1] - home[1, 1] - 1e-3   # lead-out kept: never inside home


@pytest.mark.parametrize("vx,vy,wz", [(0.12, 0.0, 0.4), (0.12, 0.0, 1.0), (-0.2, 0.0, 0.4),
                                      (-0.2, 0.0, 1.0), (0.0, 0.125, 0.4), (0.087, 0.087, 0.4),
                                      (0.087, 0.087, -1.0), (-0.106, -0.106, 1.0)])
def test_with_the_turn_reference(envs, vx, vy, wz):
    # the body still yaws wz / f per cycle (the turn's own scale, which the calibration leaves
    # alone, binds no sooner) and at a moderate turn the chord is ~1.0x a constant-curvature
    # path, where the uncalibrated one over-travelled 1.17-1.33x. At a full-rate turn the turn
    # reference's own geometry (the strides run along the toed-out soles) moves it to
    # 0.85-1.15x: inside the compass band, not a stride-gain error (the record)
    _, _, turn, both = envs
    d0, _ = cycle(turn, vx, vy, wz)
    d1, _ = cycle(both, vx, vy, wz)
    assert abs(d1[2]) >= abs(d0[2]) - 1e-9
    assert not saturated(both, vx, vy, wz)
    assert d1[2] == pytest.approx(wz / F, rel=0.01)
    x, err = speed_dir(d1, vx, vy, wz)
    if abs(wz) <= 0.4:
        assert x == pytest.approx(1.0, abs=0.05)
        assert abs(err) < 1.5
    else:
        assert 0.8 < x < 1.2
        assert abs(err) < 4.0
    assert x < speed_dir(d0, vx, vy, wz)[0]


@pytest.mark.parametrize("cmd", [(-0.4, 0.0, 0.0), (0.0, 0.25, 0.0),
                                 (0.4, 0.0, 0.0), (0.0, -0.3, 0.0)])
def test_single_axis_saturation_is_unchanged(envs, cmd):
    # a command past one axis's cap with nothing on the other: both references land on the
    # same capped stride (the 0.72x fast back stride is the 0.45 rad hip-pitch cap, not gain)
    vol, cal, _, _ = envs
    c = np.array([*cmd, 1.0, 0.0, 0.0, 0.0])
    for ph in np.linspace(-np.pi, np.pi, 37):
        assert np.allclose(vol._mimic_ref(c, ph, F), cal._mimic_ref(c, ph, F), atol=1e-12)


def test_tables(envs):
    from walker_env import MIMIC_A_MAX
    _, cal, _, _ = envs
    assert cal._cal_a[-1] == MIMIC_A_MAX and cal._cal_b[-1] == cal._vref_bmax
    for t in (*cal._cal_ta, cal._cal_tb):
        assert t[0] == 0.0 and np.all(np.diff(t) > 0.0)
    # the fixed gains' over-travel, from the tables' small-signal slopes: 1.4 rad per (m/cycle)
    # fore-aft, 1.1 lateral (the 1.23x / 1.44x of the foot-placement record)
    ga = 1.4 * (cal._cal_ta[:, 1] / cal._cal_a[1])
    gb = 1.1 * cal._cal_tb[1] / cal._cal_b[1]
    assert np.allclose(ga, 1.23, atol=0.01) and gb == pytest.approx(1.44, abs=0.01)


def test_zero_command_is_the_stand(envs):
    vol, cal, _, _ = envs
    c = np.zeros(7)
    c[3] = 1.0
    for ph in np.linspace(-np.pi, np.pi, 13):
        assert np.array_equal(vol._mimic_ref(c, ph, F), cal._mimic_ref(c, ph, F))


def test_needs_the_volume_reference():
    import robot_plant
    from walker_env import BimoWalkerEnv
    kw = robot_plant.robot_env_kwargs()
    with pytest.raises(ValueError, match="mimic_volume_ref"):
        BimoWalkerEnv(**dict(kw, mimic_lift=0.02, mimic_volume_cal=True))


def test_train_flag_reaches_the_env_kwargs():
    import train_mjx
    p = train_mjx.make_parser()
    kw = train_mjx.build_env_kw(p.parse_args(["--mimic-volume-ref", "--mimic-volume-cal"]))
    assert kw["mimic_volume_cal"] is True and kw["mimic_volume_ref"] is True
    assert "mimic_volume_cal" not in train_mjx.build_env_kw(p.parse_args(["--mimic-volume-ref"]))
    with pytest.raises(SystemExit):
        train_mjx.build_env_kw(p.parse_args(["--mimic-volume-cal"]))


def test_cpu_mjx_parity():
    """sim/mjx/parity_test.py R28, in a subprocess (parity_test turns jax_enable_x64 on)."""
    env = dict(os.environ, JAX_PLATFORMS="cpu", MUJOCO_GL="disabled")
    p = subprocess.run([sys.executable, "parity_test.py", "--mimic-volume-cal"],
                       cwd=os.path.join(SIM, "mjx"), env=env, capture_output=True, text=True,
                       timeout=1200)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]


def _tables():
    """The record's before / after tables (markdown on stdout)."""
    vol, cal, turn, both = (_make(), _make(mimic_volume_cal=True), _make(mimic_turn_ref=True),
                            _make(mimic_turn_ref=True, mimic_volume_cal=True))
    print(f"Clock {F} Hz. x = reference travel per cycle / commanded travel; dir err = travel "
          "heading - command heading; W min = minimum stance width (home "
          f"{1e3 * (_soles(cal, cal._default)[0, 1] - _soles(cal, cal._default)[1, 1]):.0f} mm); "
          "sat = the calibrated reference's saturation scale binds.\n")
    print("| heading | r | vx | vy | cmd travel mm | volume ref: x | dir err ° | W min | "
          "+ cal: x | dir err ° | W min | sat |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for hd, r, vx, vy in volume_grid():
        row = [f"{hd}", f"{r}", f"{vx:+.3f}", f"{vy:+.3f}",
               f"{1e3 * math.hypot(vx, vy) / F:.0f}"]
        for e in (vol, cal):
            disp, width = cycle(e, vx, vy, 0.0)
            x, err = speed_dir(disp, vx, vy, 0.0)
            row += [f"{x:.2f}", f"{err:+.1f}", f"{1e3 * width.min():.0f}"]
        row.append("yes" if saturated(cal, vx, vy, 0.0) else "")
        print("| " + " | ".join(row) + " |")
    print("\nTurning, with `--mimic-turn-ref` (yaw x = body yaw per cycle / (wz / f); speed x = "
          "chord / a constant-curvature path's chord; dir err against that path):\n")
    print("| vx | vy | wz | turn ref: yaw x | speed x | dir err ° | + cal: yaw x | speed x | "
          "dir err ° | sat |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for vx, vy, wz in [(0.12, 0.0, 0.4), (0.12, 0.0, 1.0), (0.24, 0.0, 1.0), (-0.2, 0.0, 0.4),
                       (-0.2, 0.0, 1.0), (-0.4, 0.0, 1.0), (0.0, 0.125, 0.4), (0.0, 0.125, -1.0),
                       (0.0, 0.25, 1.0), (0.087, 0.087, 0.4), (0.087, 0.087, -1.0),
                       (-0.106, -0.106, 1.0), (-0.212, -0.212, -1.0)]:
        row = [f"{vx:+.3f}", f"{vy:+.3f}", f"{wz:+.1f}"]
        for e in (turn, both):
            disp, _ = cycle(e, vx, vy, wz)
            x, err = speed_dir(disp, vx, vy, wz)
            row += [f"{disp[2] / (wz / F):.2f}", f"{x:.2f}", f"{err:+.1f}"]
        row.append("yes" if saturated(both, vx, vy, wz) else "")
        print("| " + " | ".join(row) + " |")
    print(f"\nTables: fore-aft travel per cycle at the 0.45 rad cap {cal._cal_ta[0, -1]:.4f} / "
          f"{cal._cal_ta[1, -1]:.4f} m (L / R), lateral at b_max {cal._cal_b[-1]:.4f} rad "
          f"{cal._cal_tb[-1]:.4f} m; small-signal 1.4 x slope "
          f"{1.4 * cal._cal_ta[0, 1] / cal._cal_a[1]:.3f}, 1.1 x lateral slope "
          f"{1.1 * cal._cal_tb[1] / cal._cal_b[1]:.3f}")


if __name__ == "__main__":
    _tables()
