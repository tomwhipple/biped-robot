"""sim/sil_twin.py's wire output: its flight records and its beacon.

Run:  .venv/bin/python -m pytest tests/test_sil_twin_wire.py -q

The builders in sim/sil_twin_wire.py are pure, so everything here is
synthetic state -- no MuJoCo run, no SIL library, no gitignored run
artifacts. Packets are decoded with tools/flightrec.decode and, end to end,
read back through tools/flight_reader with the calibration the twin's own
HELLO sent.
"""
import math
import os
import random
import subprocess
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("sim", "tools", "link"):
    sys.path.insert(0, os.path.join(ROOT, p))

import sil_twin_wire as W  # noqa: E402
import flightrec as F  # noqa: E402
import flight_collector as C  # noqa: E402
import flight_reader as R  # noqa: E402
import protocol as P  # noqa: E402

HALF_STEP = math.pi / F.STEPS_PER_REV + 1e-12
# v6 legs plant: 12 joints, all driven; the bus servo IDs of obs/bus_map.h
LEGS = ("L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_ankle", "L_ankle_roll",
        "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "R_ankle_roll")
BUS_IDS = (52, 6, 7, 8, 51, 2, 3, 4, 10, 9, 11, 12, 13, 14, 15, 16, 17)


def perturbed_cal(names, seed=3):
    rng = random.Random(seed)
    return {n: (2048 + rng.randint(-350, 350), rng.choice((1, -1))) for n in names}


def legs_state(seed=0):
    rng = random.Random(seed)
    measured = {n: (rng.uniform(-1.2, 1.2), rng.uniform(-6.0, 6.0)) for n in LEGS}
    targets = {n: (rng.uniform(-1.0, 1.0), rng.randint(1500, 2600), rng.randint(0, 900))
               for n in LEGS}
    return measured, targets


def view(cmd=(0.35, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0), phase=1.25):
    return dict(cmd=list(cmd), phase=phase, up=[0.0, 0.0, 1.0], gyro=[0.0, 0.0, 0.0])


IMU = dict(accel=(0.7, -0.02, 9.79), gyro=(0.01, -0.02, 0.03),
           quat=(math.cos(0.1), math.sin(0.1), 0.0, 0.0), up=(0.0, -0.199, 0.980))


# -- units ----------------------------------------------------------------------
def test_angle_round_trip_through_flightrec():
    rng = random.Random(1)
    for _ in range(2000):
        zero, d = rng.randint(1500, 2600), rng.choice((1, -1))
        rad = rng.uniform(-1.5, 1.5)
        steps = W.rad_to_steps(rad, zero, d)
        assert abs(F.steps_to_rad(steps, zero, d) - rad) <= HALF_STEP
    assert W.rad_to_steps(10.0) == 4095 and W.rad_to_steps(-10.0) == 0
    assert W.rad_to_steps(0.0, 1900, -1) == 1900
    one = 2 * math.pi / 4096
    assert W.rad_s_to_steps_s(100 * one) == 100 and W.rad_s_to_steps_s(100 * one, -1) == -100
    assert W.rad_s_to_steps_s(1e9) == 32767


def test_roll_pitch_matches_firmware_formula():
    q = (math.cos(0.2), math.sin(0.2), 0.0, 0.0)            # roll 0.4 rad about x
    assert W.roll_pitch_from_quat(q) == pytest.approx((0.4, 0.0), abs=1e-12)
    q = (math.cos(-0.15), 0.0, math.sin(-0.15), 0.0)        # pitch -0.3 rad about y
    assert W.roll_pitch_from_quat(q) == pytest.approx((0.0, -0.3), abs=1e-12)


# -- calibration ------------------------------------------------------------------
def test_bus_calibration_by_name():
    pol = perturbed_cal(LEGS)
    cal = W.bus_calibration(set(LEGS) | {"neck_yaw"}, BUS_IDS, pol)
    for b, name in enumerate(F.BUS_JOINT_NAMES):
        assert cal.id[b] == BUS_IDS[b]
        assert cal.fitted[b] == int(name in LEGS or name == "neck_yaw")
        if name in pol:
            assert (cal.zero[b], cal.dir[b]) == pol[name]
        else:
            assert (cal.zero[b], cal.dir[b]) == (2048, 1)
    with pytest.raises(ValueError):
        W.bus_calibration(LEGS, BUS_IDS, {"tail": (2048, 1)})
    with pytest.raises(ValueError):
        W.bus_calibration(LEGS, BUS_IDS, {"L_knee": (2048, 0)})


# -- TICK ---------------------------------------------------------------------------
def test_tick_joints_mapped_by_name_and_bus_index():
    pol = perturbed_cal(LEGS)
    cal = W.bus_calibration(set(LEGS) | {"neck_yaw"}, BUS_IDS, pol)
    measured, targets = legs_state()
    measured["neck_yaw"] = (0.3, -0.5)          # in the plant, not driven
    tf = W.TwinFlight("sil_twin test", cal, boot_id=0x1234ABCD)
    body = W.tick_body(tick=7, armed=True, torque=True, link_state=0, work_us=1800,
                       joints=W.joint_block(cal, measured, targets, written=True),
                       imu=IMU, view=view(), vbat_mv=11100)
    p = F.decode(tf.tick(5_000_000, 1_793_000_000_000_000, body))
    assert p["type"] == F.T_TICK and p["source"] == F.SRC_CTRL and p["tick"] == 7
    assert p["njoint"] == 13
    buses = [j["bus"] for j in p["joints"]]
    assert buses == sorted(W.BUS_INDEX[n] for n in list(LEGS) + ["neck_yaw"])
    full = F.JF_FB_OK | F.JF_NOM_VALID | F.JF_GOAL_VALID | F.JF_WRITTEN | F.JF_DRIVEN
    for j in p["joints"]:
        name = F.BUS_JOINT_NAMES[j["bus"]]
        zero, d = cal.zero[j["bus"]], cal.dir[j["bus"]]
        pos, vel = measured[name]
        assert abs(F.steps_to_rad(j["pos"], zero, d) - pos) <= HALF_STEP, name
        assert j["speed"] == W.rad_s_to_steps_s(vel, d)
        if name == "neck_yaw":
            assert j["flags"] == F.JF_FB_OK
            assert (zero, d) == (2048, 1) and j["nominal"] == j["goal"] == 0
            continue
        nom, goal, gspd = targets[name]
        assert j["flags"] == full, name
        assert abs(F.steps_to_rad(j["nominal"], zero, d) - nom) <= HALF_STEP
        assert (j["goal"], j["goal_speed"]) == (goal, gspd)
        # what the sim does not have stays zero
        assert (j["load"], j["current"], j["voltage_dv"], j["temp_c"], j["status"],
                j["moving"]) == (0, 0, 0, 0, 0, 0)
    assert p["flags"] == F.TF_TORQUE | F.TF_WROTE | F.TF_FB_READ | F.TF_IMU_VALID
    assert (p["fb_off_us"], p["fb_read_us"], p["imu_age_us"], p["work_us"]) == (0, 0, 0, 1800)
    assert p["ctrl"]["cmd"] == pytest.approx((0.35, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0))
    assert p["ctrl"]["phase"] == pytest.approx(1.25) and p["ctrl"]["vbat_mv"] == 11100
    assert p["bal"]["state"] == 0 and p["bal"]["ankle_bus"] == -1


def test_tick_flags_bench_and_torque_off():
    cal = W.bus_calibration(LEGS, BUS_IDS, perturbed_cal(LEGS))
    measured, targets = legs_state(1)
    # benched, torque off: feedback only, nothing driven, the robot's idle source
    p = F.decode(F.encode_tick({}, W.tick_body(
        tick=1, armed=False, torque=False, link_state=W.link_state_code(P.LinkState.BENCH),
        work_us=0, joints=W.joint_block(cal, measured, None), imu=None, vbat_mv=11100)))
    assert p["source"] == F.SRC_BENCH_IDLE and p["link_state"] == 7
    assert p["flags"] == F.TF_FB_READ                         # no torque, no IMU claim
    assert p["quat"] == (1.0, 0.0, 0.0, 0.0) and p["ctrl"]["cmd"] == (0.0,) * 7
    assert all(j["flags"] == F.JF_FB_OK for j in p["joints"])
    # armed but RELAX (torque off): the policy's target, no goal written
    p = F.decode(F.encode_tick({}, W.tick_body(
        tick=2, armed=True, torque=False, link_state=W.link_state_code(P.LinkState.RELAX),
        work_us=0, joints=W.joint_block(cal, measured, targets, written=False), imu=IMU,
        view=view())))
    assert p["source"] == F.SRC_CTRL and p["link_state"] == 2
    assert p["flags"] == F.TF_FB_READ | F.TF_IMU_VALID
    assert all(j["flags"] == F.JF_FB_OK | F.JF_NOM_VALID | F.JF_DRIVEN and j["goal"] == 0
               for j in p["joints"])


def test_imu_block_and_command_padding():
    p = F.decode(F.encode_tick({}, W.tick_body(
        tick=0, armed=True, torque=True, link_state=0, work_us=0, joints=[], imu=IMU,
        view=view(cmd=(0.4, -0.3)))))
    assert p["accel"] == pytest.approx(IMU["accel"]) and p["gyro"] == pytest.approx(IMU["gyro"])
    assert p["quat"] == pytest.approx(IMU["quat"]) and p["up"] == pytest.approx(IMU["up"])
    assert p["roll"] == pytest.approx(0.2, abs=1e-6) and p["pitch"] == pytest.approx(0.0, abs=1e-6)
    assert p["ctrl"]["cmd"] == pytest.approx((0.4, -0.3, 0, 0, 0, 0, 0))
    assert p["flags"] == F.TF_TORQUE | F.TF_IMU_VALID      # no joints: no read, no write


# -- HELLO, header, events ---------------------------------------------------------
def test_hello_identifies_the_sim_and_carries_its_calibration():
    pol = perturbed_cal(LEGS)
    cal = W.bus_calibration(LEGS, BUS_IDS, pol)
    tf = W.TwinFlight("sil_twin robot_legs_walk14_s256r24", cal, boot_id=7, idle_hz=50)
    tf.seq, tf.sent, tf.drop_net = 41, 39, 2
    h = F.decode(tf.hello(12_345_678, 1_793_000_000_000_000, ticks=600, late=3, worst_us=9100))
    assert h["fw"] == "sil_twin robot_legs_walk14_s256r24"[:31]    # the field's cap
    assert (h["utc_synced"], h["enabled"], h["idle_hz"], h["cal_ok"]) == (1, 1, 50, 1)
    assert (h["produced"], h["sent"], h["drop_net"], h["drop_ring"]) == (41, 39, 2, 0)
    assert (h["ctrl_ticks"], h["ctrl_late"], h["ctrl_worst_us"], h["uptime_ms"]) == \
        (600, 3, 9100, 12_345)
    assert (h["us_read"], h["max_push_ctrl_us"], h["ntp_syncs"]) == (0, 0, 0)
    assert h["id"] == list(BUS_IDS)
    assert h["fitted"] == [int(n in LEGS) for n in F.BUS_JOINT_NAMES]
    for b, n in enumerate(F.BUS_JOINT_NAMES):
        assert (h["zero"][b], h["dir"][b]) == pol.get(n, (2048, 1))
    assert h["seq"] == 41 and h["boot_id"] == 7


def test_header_seq_boot_and_drop_counting():
    cal = W.bus_calibration(LEGS, BUS_IDS, {})
    a, b = W.TwinFlight("x", cal), W.TwinFlight("x", cal)
    assert a.boot_id != 0 and a.boot_id != b.boot_id     # random per run
    outcomes = iter([True, False, True])
    tf = W.TwinFlight("x", cal, send=lambda pkt: next(outcomes))
    pkts = [tf.hello(0, 1), tf.tick(20_000, 2, W.tick_body(
        tick=0, armed=False, torque=False, link_state=7, work_us=0, joints=[])),
        tf.event(30_000, 3, F.EV_ARM_EDGE, a=1)]
    for pk in pkts:
        tf.post(pk)
    dec = [F.decode(pk) for pk in pkts]
    assert [d["seq"] for d in dec] == [0, 1, 2]           # one sequence for every type
    assert [d["t_boot_us"] for d in dec] == [0, 20_000, 30_000]
    assert (tf.sent, tf.drop_net) == (2, 1)
    assert F.decode(tf.event(40_000, 4, F.EV_HOME_EDGE))["drop_net"] == 1
    assert tf.post(None) is False and (tf.sent, tf.drop_net) == (2, 1)
    assert tf.hello_due(0) and not tf.hello_due(999_999) and tf.hello_due(1_000_000)


def test_events_link_state_arm_and_stop():
    tf = W.TwinFlight("x", W.bus_calibration(LEGS, BUS_IDS, {}), boot_id=1)
    e = F.decode(tf.link_change(0, 0, 0))
    assert (e["code"], e["src"], e["a"], e["b"]) == (F.EV_LINK_STATE, 1, -1, 0)
    assert tf.link_change(1, 1, 0) is None
    e = F.decode(tf.link_change(2, 2, 1))
    assert (e["a"], e["b"]) == (0, 1)
    e = F.decode(tf.event(3, 3, F.EV_ARM_EDGE, a=1))
    assert (e["code"], e["src"], e["a"], e["text"]) == (F.EV_ARM_EDGE, 0, 1, "")
    e = F.decode(tf.event(4, 4, F.EV_MODE, a=3, b=1, text="run"))
    assert (e["code"], e["a"], e["b"], e["text"]) == (F.EV_MODE, 3, 1, "run")
    e = F.decode(tf.stop(10_000_000, 0, hard=True, armed=True))
    assert (e["code"], e["a"], e["b"]) == (F.EV_STOP, 1, 1) and "HARD" in e["text"]
    assert tf.stop(11_000_000, 0, hard=True, armed=False) is None   # same burst
    assert F.decode(tf.stop(12_500_000, 0, hard=False, armed=False))["a"] == 0


def test_reader_gives_back_the_sim_angles(tmp_path):
    """Through the collector's file and flight_reader's tick_rows: steps go back
    to radians with the calibration the twin's own HELLO sent."""
    pol = perturbed_cal(LEGS, seed=9)
    cal = W.bus_calibration(LEGS, BUS_IDS, pol)
    tf = W.TwinFlight("sil_twin test", cal)
    col = C.Collector(str(tmp_path), log=lambda m: None)
    t_utc = 1_793_382_251_000_000
    col.handle(tf.hello(0, t_utc), t_utc + 500)
    states = []
    for k in range(20):
        measured, targets = legs_state(100 + k)
        states.append((measured, targets))
        body = W.tick_body(tick=k, armed=True, torque=True, link_state=0, work_us=900,
                           joints=W.joint_block(cal, measured, targets, written=True),
                           imu=IMU, view=view())
        col.handle(tf.tick(20_000 * (k + 1), t_utc + 20_000 * (k + 1), body),
                   t_utc + 20_000 * (k + 1) + 500)
    summ = col.rotate("end")
    s = R.load(os.path.join(str(tmp_path), summ["file"]))
    rows = R.tick_rows(s)
    assert len(rows) == 20 and summ["boots"][f"{tf.boot_id:08x}"]["missing"] == 0
    for r, (measured, targets) in zip(rows, states):
        for n in LEGS:
            assert abs(r[f"{n}.pos_rad"] - measured[n][0]) <= HALF_STEP, n
            assert abs(r[f"{n}.nom_rad"] - targets[n][0]) <= HALF_STEP, n
            b = W.BUS_INDEX[n]
            goal_rad = F.steps_to_rad(targets[n][1], cal.zero[b], cal.dir[b])
            assert r[f"{n}.goal_rad"] == pytest.approx(goal_rad)
            assert r[f"{n}.err_rad"] == pytest.approx(r[f"{n}.pos_rad"] - goal_rad)


# -- the SIL library's targets, with the real harness's maths (no library) ----------
def test_sil_view_maps_the_harness_arrays_by_name():
    sys.path.insert(0, os.path.join(ROOT, "sim", "sil"))
    H = pytest.importorskip("harness")
    spec = H.SPEC
    assert tuple(H.BUS_NAMES) == F.BUS_JOINT_NAMES == P.JOINT_NAMES
    # sil_twin's by-name bus index is the harness's by-servo-ID one
    assert [W.BUS_INDEX[n] for n in spec.joint_names] == list(H.joint_to_slot_perm(spec))
    rng = np.random.default_rng(5)
    n = spec.num_joints
    goal = rng.integers(1200, 2900, size=n)
    speed = rng.integers(0, 3000, size=n)
    action = rng.uniform(-1, 1, size=spec.act_dim)
    obs = np.zeros(spec.obs_dim)
    ph = spec.off["phase"]
    obs[ph], obs[ph + 1] = math.sin(-2.5), math.cos(-2.5)
    obs[spec.slice("cmd", spec.num_cmd)] = np.arange(spec.num_cmd) * 0.1
    obs[spec.slice("up", 3)] = (0.1, -0.2, 0.97)
    obs[spec.slice("gyro", 3)] = (0.5, 0.6, 0.7)
    cal = H.Calibration.perturbed(2, spec, flip_dirs=True)
    for order in ("ascending_id", "joint"):
        out = H.SilTargets()
        g = H.to_bus(goal, order, spec, fill=4000)
        sp = H.to_bus(speed, order, spec, fill=4000)
        for i in range(H.NB):
            out.goal_ticks[i], out.goal_speed[i] = int(g[i]), int(sp[i])
        for i in range(spec.act_dim):
            out.action[i] = float(action[i])
        for i in range(spec.obs_dim):
            out.obs[i] = float(obs[i])
        lib = type("Lib", (), dict(spec=spec, out_order=order, cal=cal))
        v = W.sil_view(H, lib, out)
        nom = H.action_to_angles(action)
        for i, name in enumerate(spec.joint_names):
            assert v["targets"][name] == (pytest.approx(nom[i]), int(goal[i]), int(speed[i]))
        assert v["phase"] == pytest.approx(-2.5)
        assert v["cmd"] == pytest.approx(list(np.arange(spec.num_cmd) * 0.1), abs=1e-6)
        assert v["up"] == pytest.approx([0.1, -0.2, 0.97]) and v["gyro"] == pytest.approx([0.5, 0.6, 0.7])
        # nominal steps in the HELLO's calibration == the harness's angleToSteps
        bcal = W.bus_calibration(spec.joint_names, H.BUS_IDS,
                                 {nm: (int(cal.zero_steps[i]), int(cal.dir[i]))
                                  for i, nm in enumerate(spec.joint_names)})
        rows = {F.BUS_JOINT_NAMES[j["bus"]]: j for j in W.joint_block(
            bcal, {}, v["targets"], written=True)}
        want = H.angle_to_steps(nom, cal)
        assert [rows[nm]["nominal"] for nm in spec.joint_names] == [int(x) for x in want]


# -- the beacon -------------------------------------------------------------------------
@pytest.mark.parametrize("att", [False, True])
@pytest.mark.parametrize("pose", [False, True])
def test_beacon_attitude_block_only_when_asked(att, pose):
    up = (0.123, -0.456, 0.880)
    joints = tuple(0.01 * k for k in range(P.NUM_JOINTS)) if pose else ()
    kw = dict(seq_echo=41, state=P.LinkState.LIVE, vbat_v=11.1, up=up, vx_est=0.2,
              wz_est=-0.1, loop_late_pct=3, t_us=1_793_000_000_123_456, joints=joints)
    frame = W.beacon(att=att, **kw)
    want = {(False, False): P.TLM_LEN, (True, False): P.TLM_LEN_ATT,
            (False, True): P.TLM_LEN_EXT, (True, True): P.TLM_LEN_EXT_ATT}[(att, pose)]
    assert len(frame) == want
    t = P.decode_telemetry(frame)
    assert t.up_z == pytest.approx(0.880)
    assert t.up_xy == (pytest.approx((0.123, -0.456)) if att else ())
    assert len(t.joints) == (P.NUM_JOINTS if pose else 0)
    # the protocol's own encoder, nothing else: same bytes as a direct encode
    assert frame == P.encode_telemetry(P.Telemetry(
        seq_echo=41, state=P.LinkState.LIVE, vbat_v=11.1, up_z=0.880, vx_est=0.2,
        wz_est=-0.1, servo_err=0, loop_late_pct=3, t_us=1_793_000_000_123_456,
        joints=joints, up_xy=(0.123, -0.456) if att else ()))


def test_sil_twin_rejects_a_bad_flight_destination():
    for args, msg in ((["--flight", "nohost"], "HOST:PORT"),
                      (["--flight", "127.0.0.1:4221", "--viewer"], "--viewer")):
        r = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "sil_twin.py"), *args],
                           capture_output=True, text=True, timeout=60)
        assert r.returncode == 2 and msg in r.stderr, r.stderr
