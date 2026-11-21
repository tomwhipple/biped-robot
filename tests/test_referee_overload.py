"""eval_precision.py --overload-column (#6): sts_servo_model.STSProtection MEASURES the STS3215's
default overload protection (> 80 % of stall for 2 s drops the servo to 20 %) on every servo in the
referee's get-up scenarios, without enforcing it. The meter must see every servo on the bus (the
held arms too), never write the env, and leave the rollout bit-identical. CPU only."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

ARGS = ["--robot", "--precision", "--mjx-impl", "jax", "--family", "loco", "--quantize"]


def _env():
    import eval_precision as ep
    import train_mjx
    cfg = train_mjx.build_env_kw(train_mjx.make_parser().parse_args(ARGS))
    return ep.make_env(cfg, 4.0, False, cfg["xml_path"])


def _roll(env, meter=None, steps=40):
    env.reset(seed=4)
    a = 0.9 * np.sin(np.arange(env.action_space.shape[0], dtype=np.float32) + 0.3)
    obs = []
    for t in range(steps):
        o, *_ = env.step(a if t % 10 < 5 else -a)
        obs.append(o)
        if meter is not None:
            meter[0].tick(meter[1], env.control_dt)
    return np.array(obs), np.array(env.data.qpos)


def test_meter_sees_every_servo_and_leaves_the_rollout_alone():
    import eval_precision as ep
    import sts_servo_model as SM
    e = _env()
    obs0, q0 = _roll(e)
    e.reset(seed=4)
    servo = e._servo
    meter = ep.overload_meter(e)
    assert e._servo is servo                                  # the env's own arrays untouched
    obs1, q1 = _roll(e, meter)
    np.testing.assert_array_equal(obs0, obs1)
    np.testing.assert_array_equal(q0, q1)
    rep = meter[0].report()
    assert list(rep) == e._servo_names and e._n_servo > e._nq_act   # held servos measured too
    peaks = np.array([r["peak_duty"] for r in rep.values()])
    assert peaks.max() > 0.3 and np.all(peaks >= 0.0)
    # its load is the servo's torque over that servo's own episode stall
    view = meter[1]
    stall = float(e._servo[2]) * e._stall_prof
    np.testing.assert_allclose(meter[0].stall0, stall)
    tau = np.asarray(e._servo_tau)
    assert np.all(meter[0].alt["torque"]["peak"] >= np.abs(tau) / stall - 1e-12)
    assert view._nq_act == e._n_servo and SM.OVERLOAD_FRAC == 0.80


def test_summary_aggregates_worst_over_seeds():
    import eval_precision as ep
    base = dict(peak=0.5, cont_max=0.0, cum=0.0, trip_t=None, peak_i=1.0, oc_cont_max=0.0,
                oc_trip_t=None, peak_torque=0.5, cont_max_torque=0.0, cum_torque=0.0,
                peak_duty=0.5, cont_max_duty=0.0, cum_duty=0.0)
    r1 = {"knee": dict(base, peak_duty=0.9, cont_max_duty=1.2, cum_duty=1.5)}
    r2 = {"knee": dict(base, peak_duty=1.02, cont_max_duty=2.1, cum_duty=2.6, trip_t=7.4)}
    s = ep.overload_summary([r1, r2])["knee"]
    assert s == dict(peak_pct=102.0, peak_torque_pct=50.0, over80_cont_s=2.1, over80_total_s=2.6,
                     would_trip="1/2", first_trip_s=7.4)


def test_meter_reads_duty_against_the_sagged_envelope():
    """With the supply sagged to scale s, a servo at its cap reads 100 % duty, not s."""
    import eval_precision as ep
    e = _env()
    e.reset(seed=4)
    e.supply_sag_ohm, e._sag_r, e._sag_s_run = (0.44, 0.44), 0.44, 0.8
    e._sag_s = 0.9                       # the relaxed next-substep scale: not what the meter reads
    meter, view = ep.overload_meter(e)
    stall = float(e._servo[2]) * e._stall_prof
    w0 = float(e._servo[3]) * e._w0_prof
    qd = np.zeros(e.model.nv)
    qd[e._sqvel] = 0.25 * w0                                   # every servo at a quarter speed
    e.data.qvel[:] = qd
    e._servo_tau = stall * (0.8 - 0.25)                        # at the sagged cap s stall (1 - w/(s w0))
    ep.overload_tick(meter, view, e.control_dt)
    np.testing.assert_allclose(meter.alt["duty"]["peak"], 1.0)
    assert meter.volts == pytest.approx(0.8 * float(e.supply_voltage))   # the sagged supply
