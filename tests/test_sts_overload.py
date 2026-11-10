"""The STS overload cutoff model (sim/sts_servo_model.py; #126, #83, #6).

The documented protection (memory table V3.7 regs 34-36: load > 80 % of the
servo's maximum held 2.0 s -> 20 % torque) on a stub env, the supply-current
model against the datasheet's stall point, the walker_env per-servo envelope
profile (the STS3250 hip rolls, the 2026-11-03 refit) in the load fraction,
and the 2026-11-10 record's claim on the current supine reference.
"""
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import sts_servo_model as SM  # noqa: E402

DT = 0.02
STALL, W0 = 2.72, 3.74


def stub(n=2, stall=STALL, w0=W0, stall_prof=None, w0_prof=None):
    env = SimpleNamespace(_nq_act=n, _servo=np.array([12.0, 0.25, stall, w0], dtype=object),
                          _servo_tau=np.zeros(n), _jqvel=np.arange(n),
                          data=SimpleNamespace(qvel=np.zeros(n), ctrl=np.zeros(n)))
    if stall_prof is not None:
        env._stall_prof = np.asarray(stall_prof, float)
        env._w0_prof = np.asarray(w0_prof if w0_prof is not None else np.ones(n), float)
    return env


def drive(env, prot, tau, seconds, qd=None, ctrl=None):
    for _ in range(int(round(seconds / DT))):
        env._servo_tau = np.asarray(tau, float)
        if qd is not None:
            env.data.qvel = np.asarray(qd, float)
        if ctrl is not None:
            env.data.ctrl = np.asarray(ctrl, float)
        prot.tick(env, DT)


def test_documented_constants():
    assert SM.OVERLOAD_FRAC == 0.80 and SM.PROTECT_TIME_S == 2.0 and SM.PROTECT_FRAC == 0.20
    assert SM.OVERCURRENT_A_DS == 2.0 and SM.OVERCURRENT_A_MT == pytest.approx(3.25)
    assert SM.STALL_NM_12V == pytest.approx(2.94, abs=0.005)


def test_trips_after_two_seconds_above_80_percent_and_drops_to_20():
    env = stub()
    prot = SM.STSProtection(env, ["a", "b"], load="torque")
    drive(env, prot, [0.9 * STALL, 0.7 * STALL], 1.9)
    assert prot.trip_t == [None, None]
    assert np.allclose(env._servo[2], STALL)
    drive(env, prot, [0.9 * STALL, 0.7 * STALL], 0.2)
    assert prot.trip_t[0] == pytest.approx(2.0, abs=DT) and prot.trip_t[1] is None
    assert env._servo[2][0] == pytest.approx(SM.PROTECT_FRAC * STALL)
    assert env._servo[2][1] == pytest.approx(STALL)
    rep = prot.report()
    assert rep["a"]["peak"] == pytest.approx(0.9) and rep["b"]["cont_max"] == 0.0


def test_measure_only_never_touches_the_plant():
    env = stub()
    prot = SM.STSProtection(env, ["a", "b"], enforce=False, load="torque")
    drive(env, prot, [STALL, STALL], 3.0)
    assert prot.trip_t[0] is not None             # it records the trip ...
    assert np.allclose(env._servo[2], STALL)      # ... but the servo keeps its stall


def test_continuous_timer_resets_cumulative_does_not():
    for timer, trips in (("continuous", False), ("cumulative", True)):
        env = stub(n=1)
        prot = SM.STSProtection(env, ["a"], load="torque", timer=timer)
        for _ in range(3):                         # 3 x 1.2 s above, separated by dips
            drive(env, prot, [0.9 * STALL], 1.2)
            drive(env, prot, [0.5 * STALL], 0.2)
        assert (prot.trip_t[0] is not None) is trips, timer
        assert prot.cum[0] == pytest.approx(3.6, abs=DT)


def test_duty_reading_counts_speed():
    """duty = tau / stall + w / w0: a joint at 50 % torque moving at 40 % of
    its no-load speed in the same direction reads 90 % duty, 50 % torque."""
    env = stub(n=1)
    prot = SM.STSProtection(env, ["a"], load="duty")
    drive(env, prot, [0.5 * STALL], 2.1, qd=[0.4 * W0])
    rep = prot.report()["a"]
    assert rep["peak_torque"] == pytest.approx(0.5) and rep["cont_max_torque"] == 0.0
    assert rep["peak_duty"] == pytest.approx(0.9)
    assert prot.trip_t[0] is not None


def test_goal_change_release():
    env = stub(n=1)
    prot = SM.STSProtection(env, ["a"], load="torque", release="goal_change")
    drive(env, prot, [STALL], 2.1, ctrl=[0.0])
    assert env._servo[2][0] == pytest.approx(SM.PROTECT_FRAC * STALL)
    drive(env, prot, [0.1 * STALL], DT, ctrl=[0.1])        # a new goal clears it
    assert env._servo[2][0] == pytest.approx(STALL) and not prot.tripped[0]


def test_envelope_profile_sets_the_load_and_the_trip():
    """walker_env scales env._servo's stall by _stall_prof (servo_types,
    --servo-envelope): the load is a fraction of that effective stall, and a
    trip writes 20 % of the BASE stall back (the profile still multiplies)."""
    env = stub(stall_prof=[1.0, 2.0], w0_prof=[1.0, 1.3])
    prot = SM.STSProtection(env, ["a", "b"], load="torque")
    assert np.allclose(prot.stall0, [STALL, 2 * STALL]) and np.allclose(prot.w0, [W0, 1.3 * W0])
    drive(env, prot, [0.9 * STALL, 0.9 * STALL], 2.1)
    assert prot.trip_t[0] is not None and prot.trip_t[1] is None   # b reads 45 %
    assert np.allclose(env._servo[2], [SM.PROTECT_FRAC * STALL, STALL])


def test_supply_current_at_the_datasheet_stall():
    i_m = SM.supply_current(SM.STALL_NM_12V, 0.0, 4.04, 12.0, "motor")
    assert i_m == pytest.approx(SM.I_STALL_12V)                    # clamped to the 2.7 A stall current
    i_b = SM.supply_current(SM.STALL_NM_12V, 0.0, 4.04, 12.0, "bridge")
    assert i_b == pytest.approx(SM.I_STALL_12V, abs=0.05)           # K_CU fitted to the same point
    assert SM.supply_current(0.0, 0.0, 4.04, 12.0, "motor") == pytest.approx(SM.I_IDLE)
    assert SM.supply_current(0.0, 4.04, 4.04, 12.0, "motor") == pytest.approx(SM.I_NOLOAD)


def test_walker_env_profile_is_read():
    """a real WalkerEnv with STS3250 hip rolls: the protection's per-joint
    maximum is the envelope the step clamps to"""
    from walker_env import BimoWalkerEnv, SERVO_TYPES
    env = BimoWalkerEnv(xml_path=os.path.join(SIM, "bimo_biped_v6ar_meas.xml"), actuator_model="sts3215",
                        supply_voltage=12.0, servo_types={"hip_roll": "sts3250"}, domain_rand=False)
    env.reset(seed=0)
    names = [env.model.actuator(i).name for i in range(env._nq_act)]
    prot = SM.STSProtection(env, names, enforce=False)
    for i, n in enumerate(names):
        want = 2.94 * (SERVO_TYPES["sts3250"][0] if n.endswith("hip_roll") else 1.0)
        assert prot.stall0[i] == pytest.approx(want, rel=1e-6), n


def test_current_supine_reference_does_not_trip_at_full_strength():
    """docs/design-v6/getup-overload-2026-11-10.md: the supine reference on
    the robot's plant stands with the cutoff enforced on either reading, and
    the shoulder never holds above 80 % of stall for the 2.0 s timer."""
    import getup_ref_overload as O
    for reading in ("torque", "duty"):
        r = O.run("supine_ik", O.NOMINAL, "11.1V", enforce=reading)
        assert r["ok"], reading
        assert all(v["trip_t"] is None for v in r["prot"].values()), reading
        for j in ("L_shoulder", "R_shoulder"):
            assert r["prot"][j]["cont_max_torque"] < SM.PROTECT_TIME_S
            assert r["prot"][j]["peak_torque"] < 0.9
