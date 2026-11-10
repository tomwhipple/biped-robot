"""The weight-transfer gate's machinery (experiments/policy/weight-transfer-gate, issue #259): the
serial-log parser that rebuilds the 10-25 one-leg protocol, the training-chain stand-in, the probe
terms, and the shared metrics. The physics results are in the gate's README, not pinned here; the
two checks against the robot's own logs run only where hw_sessions/ is present."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "..", "experiments", "policy", "weight-transfer-gate")
sys.path.insert(0, GATE)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import gate_common as GC  # noqa: E402
import check1_oneleg as C1  # noqa: E402

LOG = """\
18:00:00.000Z > pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000
18:00:00.100Z   pose -> 12 joints, SMOOTH minimum-jerk over 3000 ms (speed x1.00, acc 0, step 1 ticks / 20 ms, ZV 0 ms): streaming
18:00:04.000Z > balance
18:00:04.050Z   gains    kp 0.250  ki 0.500 /s  ankle x+1.00  hip x-1.00  clamp 10.0 deg  abort 12.0 deg
18:00:04.050Z   roll now -0.50 deg  (+ = leaning to the robot's RIGHT)
18:00:04.050Z   state    off (arm it with `pose ... B<L|R>`)
18:00:05.000Z > pose 1915 1828 2510 1829 1819 1808 2525 1792 2184 2153 1941 1825 2048 2048 2048 2048 2048 s6000 BR
18:00:05.200Z   pose -> 12 joints, SMOOTH minimum-jerk over 6000 ms (speed x1.00, acc 0, step 1 ticks / 20 ms, ZV 0 ms): streaming
18:00:06.000Z > balance
18:00:06.050Z   roll now -2.00 deg  (+ = leaning to the robot's RIGHT)
18:00:06.050Z            ref -0.50 deg  err -1.50 deg  I -0.0100  u -0.70 deg
"""


def _log(tmp_path, text=LOG):
    p = tmp_path / "x_serial.log"
    p.write_text(text)
    return str(p)


def test_parse_log_moves_polls_gains(tmp_path):
    L = C1.parse_log(_log(tmp_path))
    assert len(L["moves"]) == 2
    t0, dur, ticks, B = L["moves"][0]
    assert dur == 3.0 and B is None and ticks[0] == 2082 and len(ticks) == 12
    # the move starts at the firmware's "streaming" stamp, not at the typed command
    assert L["moves"][1][0] - L["moves"][0][0] == pytest.approx(5.1)
    assert L["moves"][1][1] == 6.0 and L["moves"][1][3] == "R"
    assert L["gains"] == dict(kp=0.25, ki=0.5, ka=1.0, kh=-1.0, clamp_deg=10.0, abort_deg=12.0, kd=0.0)
    assert [p["roll"] for p in L["polls"]] == [-0.5, -2.0]
    assert L["polls"][0]["err"] is None and L["polls"][1]["err"] == -1.5 and L["polls"][1]["u"] == -0.7


def test_stand_of_is_replay_shaped(tmp_path):
    st, t0 = C1.stand_of(C1.parse_log(_log(tmp_path)), "R")
    assert st.side == "R" and st.moves[0].t0 == 0.0 and st.moves[1].B == "R"
    assert set(st.cal) == set(C1.CAL) and C1.b_stages(st) == [(1, st.moves[1])]


def test_train_chain_dead_time_and_lag():
    names = ["L_hip_roll", "L_knee"]
    ch = GC.TrainChain(names, 0.02, lag_hz=0.0, dead15_ticks=3.0, dead50_ticks=0.0)
    ch.reset([0.0, 0.0])
    out = [ch(np.array([1.0, 1.0])) for _ in range(5)]
    assert [o[0] for o in out] == [1.0] * 5                   # the hip roll (STS3250): no dead time
    assert [o[1] for o in out] == [0.0, 0.0, 0.0, 1.0, 1.0]  # the STS3215: three ticks
    half = GC.TrainChain(names, 0.02, lag_hz=0.0, dead15_ticks=0.5, dead50_ticks=0.5)
    half.reset([0.0, 0.0])
    assert half(np.array([1.0, 1.0]))[1] == pytest.approx(0.5)   # a fractional tick interpolates
    lag = GC.TrainChain(names, 0.02, lag_hz=5.0, lag_scale_hip_roll=4.0)
    lag.reset([0.0, 0.0])
    y = lag(np.array([1.0, 1.0]))
    assert y[1] == pytest.approx(1 - math.exp(-2 * math.pi * 5.0 * 0.02))
    assert y[0] == pytest.approx(1 - math.exp(-2 * math.pi * 20.0 * 0.02))   # x4 on the hip roll


def test_parse_terms_and_defaults():
    assert GC.parse_terms("") == {}
    assert GC.parse_terms("lash_fit, torso=0.7,com_y=-0.0068") == dict(lash_fit=True, torso=0.7, com_y=-0.0068)
    # the DR centres of the prec16 recipe (config.json ranges) the train plant is built at
    assert GC.TRAIN_MID["roll_lash"] == pytest.approx(1.25) and GC.TRAIN_MID["lag_hz"] == pytest.approx(5.325)


def test_roll_yaw_and_flail_rate():
    th = math.radians(10.0)
    qx = (math.cos(th / 2), math.sin(th / 2), 0.0, 0.0)            # +x (forward): the right side dips
    assert math.degrees(GC.roll_of(qx)) == pytest.approx(10.0)
    qz = (math.cos(th / 2), 0.0, 0.0, math.sin(th / 2))
    assert math.degrees(GC.yaw_of(qz)) == pytest.approx(10.0)
    r = GC.flail_rate(np.tile([0.0, 0.0, 2.0], (200, 1)))
    a = 1 - math.exp(-0.02 / 0.2)
    assert r[0] == pytest.approx(2.0 * a) and r[-1] == pytest.approx(2.0, abs=1e-3)


def test_check2_metrics_on_a_synthetic_spin():
    import check2_replay as C2
    t = np.arange(-1.0, 2.5, 0.02)
    wz = np.where((t >= 0.5) & (t < 1.0), -2.0, 0.0)             # -1 rad over 0.5 s
    yaw = np.cumsum(wz) * 0.02
    quat = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], 1)
    q = np.zeros((len(t), 12))
    q[:, 9] = np.where((t > 0.2) & (t < 0.8), -0.5, 0.0)          # R_knee
    gyro = np.stack([0 * t, 0 * t, wz], 1)
    m = C2.metrics(t, q, gyro, quat)
    assert m["yaw_end"] == pytest.approx(-math.degrees(1.0), abs=0.5)
    assert m["gz_peak"] == -2.0 and m["knee_R"] == pytest.approx(math.degrees(0.5))
    assert m["hip_roll_L"] == 0.0 and m["roll_L"] == pytest.approx(0.0, abs=1e-9)


def test_fk_soles_level_stand_is_symmetric():
    import check2_replay as C2
    z = C2.fk_soles(np.zeros((1, 12)), np.array([[1.0, 0.0, 0.0, 0.0]]))
    assert z[0, 0] == pytest.approx(z[0, 1], abs=1e-6)


@pytest.mark.skipif(not os.path.exists(C1.LOGS["latR"]), reason="hw_sessions/2026-10-25 not on this machine")
def test_robot_numbers_come_back_out_of_the_logs():
    """The 10-25 NOTES.md numbers, recovered from the raw serial logs by the gate's own parser
    (the sim side of the rows is a zero trace here)."""
    for name, curve in (("latR", C1.HW_NOTES["latR"]["curve"]), ("latL", C1.HW_NOTES["latL"]["curve"])):
        log = C1.parse_log(C1.LOGS[name])
        st, t0 = C1.stand_of(log, C1.STANCE[name])
        t = np.arange(0.0, st.moves[-1].t0 + 5.0, 0.02)
        z = np.zeros_like(t)
        tr = dict(t=t, roll=z, fz_sw=z + 1.0, fz_st=z + 1.0, clrL=z, clrR=z)
        rows = C1.summarize_lat(name, tr, log, t0)
        assert [round(r["hw"], 2) for r in rows] == curve
    log = C1.parse_log(C1.LOGS["clrR"])
    st, t0 = C1.stand_of(log, "R")
    t = np.arange(0.0, st.moves[-1].t0 + 5.0, 0.02)
    z = np.zeros_like(t)
    tr = dict(t=t, roll=z, err=z, u=z, state=z, clrL=z, clrR=z)
    out = C1.summarize_lift("clrR", tr, log, t0)
    assert out["l10"]["hw_polled_peak"] == -1.25 and out["l30"]["hw_polled_peak"] == -3.17


@pytest.mark.skipif(not os.path.exists(os.path.join(GC.run_dir(), "config.json")),
                    reason="the prec16 run (sim/runs, gitignored) is not on this machine")
def test_train_plant_and_terms():
    env = GC.train_env()
    assert env._gate_plant == "train:mid"
    assert math.degrees(env._roll_lash_rad) == pytest.approx(1.25)
    assert env.fw_shaper_hz == 0.0 and isinstance(env._gate_chain, GC.TrainChain)
    b = env._torso_bid
    m0 = float(env.model.body_mass[b])
    GC.apply_terms(env, dict(lash_fit=True, torso=0.5, roll_k=0.5))
    assert math.degrees(env._roll_lash_rad) == pytest.approx(1.552)
    mask = dict(zip(env._servo_names, env._roll_mask))
    assert mask["L_hip_roll"] == pytest.approx(1.0) and mask["L_ankle_roll"] == pytest.approx(0.963 / 1.552)
    assert mask["L_knee"] == 0.0
    assert env.model.body_mass[b] == pytest.approx(0.5 * m0)
    GC.apply_terms(env, dict(torso=1.0))                 # terms rescale from the nominal, never compound
    assert env.model.body_mass[b] == pytest.approx(m0)


def test_grade_roll_opt_ins_default_off(monkeypatch):
    """grade.py's roll-chain opt-ins (round 2): unset -> no change to the plant at all."""
    sys.path.insert(0, os.path.join(HERE, "..", "experiments", "policy", "shaper-train"))
    import grade
    monkeypatch.delenv("GRADE_ROLL_LASH", raising=False)
    monkeypatch.delenv("GRADE_ROLL_SERIES", raising=False)
    assert grade.roll_plant_opts() == {}
    monkeypatch.setenv("GRADE_ROLL_LASH", "1.25")
    monkeypatch.setenv("GRADE_ROLL_SERIES", "hip=14.2, ankle=27.8")
    assert grade.roll_plant_opts() == dict(roll_lash_deg=1.25, ser_hip=14.2, ser_ankle=27.8)


def test_series_joint_in_spec():
    import mujoco
    spec = mujoco.MjSpec.from_file(GC.XML)
    GC.add_series_joint(spec, "L_hip_roll", 14.2)
    m = spec.compile()
    j, j0 = m.joint("L_hip_roll_ser"), m.joint("L_hip_roll")
    assert j.bodyid[0] == j0.bodyid[0] and list(j.axis) == list(j0.axis)
    assert m.jnt_stiffness[j.id] == pytest.approx(14.2) and m.nq == mujoco.MjModel.from_xml_path(GC.XML).nq + 1


def test_wt_metrics_lift_and_shift():
    import wt_grade as WG
    n = 200
    ft = np.zeros((n, 8))
    ft[:, 3], ft[:, 5] = 0.06, -0.06                 # L sole at y +6 cm, R at -6 cm
    ft[100:120, 1] = 0.02                            # the R foot 2 cm up for 20 ticks ...
    ft[100:120, 7] = 0.03                            # ... with the CoM half way over the L foot
    m = WG.wt_metrics(ft)
    assert m["lift_cm"] == pytest.approx([0.0, 2.0])
    assert m["shift_when_up"][0] is None and m["shift_when_up"][1] == pytest.approx(0.5)


def test_records_and_joint_variants_are_named():
    import check2_replay as C2
    assert set(C2.RECORDS) == {"1109", "1106t1", "1106t2"} and C2.record_name() in C2.RECORDS
    assert C2.JOINT_VARIANTS["plant"] == {}
    assert C2.JOINT_VARIANTS["series spring, hip rolls 14.2 N m/rad"] == dict(ser_hip=14.2)


@pytest.mark.skipif(not os.path.exists(os.path.join(GC.run_dir(), "config.json")),
                    reason="the prec16 run (sim/runs, gitignored) is not on this machine")
def test_series_term_scales_the_roll_pd_and_hides_it_from_the_encoder():
    env = GC.train_env(terms=dict(lash=0.0, ser_hip=14.2))
    i = env._servo_names.index("L_hip_roll")
    k = env._sname2i["L_knee"]
    kp = float(env._servo[0]) * env._gate_nom["kp"][i]
    x = 14.2 / (kp + 14.2)
    assert env._kp_prof[i] == pytest.approx(env._gate_nom["kp"][i] * x)
    assert env._gate_alpha[i] == pytest.approx(1 - x) and env._gate_alpha[k] == 0.0
    assert env._kp_prof[k] == pytest.approx(env._gate_nom["kp"][k])
