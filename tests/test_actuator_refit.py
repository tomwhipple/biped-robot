"""Actuator refit + smoothness penalties (2026-11-03, experiments/policy/actuator-refit).

servo_types takes an explicit (stall ratio, no-load ratio) pair next to the type names: the
measured hip pitch / knee envelope (--servo-envelope) has no catalogue name. A pair keeps the
STS3215's dead time under act_delay_types. --supply-voltage scales the envelope. --w-qacc and
--w-action-acc price the tick's joint acceleration and the action's second difference.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

NAMES = ["L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "R_hip_pitch", "R_knee"]


def test_envelope_pair_and_names():
    from walker_env import servo_envelope_vectors, SERVO_TYPES
    st, w0 = servo_envelope_vectors(NAMES, {"L_hip_roll": "sts3250", "hip_pitch": [2.0, 1.325],
                                            "knee": (2.0, 1.281)})
    np.testing.assert_allclose(st, [1, SERVO_TYPES["sts3250"][0], 2, 2, 2, 2])
    np.testing.assert_allclose(w0, [1, SERVO_TYPES["sts3250"][1], 1.325, 1.281, 1.325, 1.281])
    with pytest.raises(ValueError):
        servo_envelope_vectors(NAMES, {"knee": [2.0]})
    with pytest.raises(ValueError):
        servo_envelope_vectors(NAMES, {"knee": "sts9999"})


def test_cli_flags_parse_into_env_kw():
    import train_mjx
    a = train_mjx.make_parser().parse_args(
        ["--robot", "--xml", "bimo_biped_v6legs_meas.xml", "--servo-kp-scale", "asbuilt",
         "--servo-envelope", "hip_pitch=2:1.325,knee=2:1.281", "--supply-voltage", "12",
         "--w-qacc", "4e-5", "--w-action-acc", "1.0"])
    kw = train_mjx.build_env_kw(a)
    assert kw["servo_types"]["hip_pitch"] == [2.0, 1.325]
    assert kw["servo_types"]["knee"] == [2.0, 1.281]
    assert kw["servo_types"]["L_hip_roll"] == "sts3250"          # the plant's own types stay
    assert kw["supply_voltage"] == 12.0
    assert kw["w_qacc"] == 4e-5 and kw["w_action_acc"] == 1.0


def test_mjx_env_profiles_delay_and_penalties():
    import jax
    import jax.numpy as jp
    import train_mjx
    from env_mjx import BimoMJXEnv
    a = train_mjx.make_parser().parse_args(
        ["--robot", "--xml", "bimo_biped_v6legs_meas.xml", "--servo-kp-scale", "asbuilt",
         "--precision", "--mjx-impl", "jax", "--act-delay-types", "sts3215=0:2,sts3250=0:1",
         "--servo-envelope", "hip_pitch=2:1.325", "--supply-voltage", "12"])
    kw = train_mjx.build_env_kw(a)
    e0 = BimoMJXEnv(**kw)
    names = e0._servo_names
    i = names.index("R_hip_pitch")
    assert float(e0._stall_prof[i]) == pytest.approx(2.0) and float(e0._w0_prof[i]) == pytest.approx(1.325)
    assert int(e0._ad_hi[i]) == 2 and int(e0._ad_hi[names.index("R_hip_roll")]) == 1
    assert float(e0._nom_servo[3]) == pytest.approx(4.04)        # 12 V: the 12 V no-load
    e1 = BimoMJXEnv(**dict(kw, w_qacc=1e-3, w_action_acc=10.0))
    act = 0.3 * jp.ones(e0.action_size)
    r = []
    for e in (e0, e1):
        st = e.reset(jax.random.PRNGKey(1))
        for _ in range(3):
            st = e.step(st, act)
        r.append(float(st.reward))
        assert st.prev_action2.shape == (e.action_size,)
    assert r[1] < r[0]                                            # the penalties cost


def test_joint_speed_cost_is_off_by_default_and_charges_only_above_the_soft_speed():
    import jax
    import jax.numpy as jp
    import train_mjx
    from env_mjx import BimoMJXEnv
    args = ["--robot", "--xml", "bimo_biped_v6legs_meas.xml", "--servo-kp-scale", "asbuilt",
            "--precision", "--mjx-impl", "jax"]
    p = train_mjx.make_parser()
    kw = train_mjx.build_env_kw(p.parse_args(args))
    assert "w_joint_speed" not in kw
    k2 = train_mjx.build_env_kw(p.parse_args(args + ["--w-joint-speed", "0.05", "--joint-speed-soft", "90"]))
    assert k2["w_joint_speed"] == 0.05 and k2["joint_speed_soft_dps"] == 90.0
    act = 0.8 * jp.ones(BimoMJXEnv(**kw).action_size)

    def reward(**over):
        e = BimoMJXEnv(**dict(kw, **over))
        st = e.reset(jax.random.PRNGKey(1))
        for _ in range(5):
            st = e.step(st, act)
        return float(st.reward)

    r0 = reward()
    assert reward(w_joint_speed=0.0, joint_speed_soft_dps=1.0) == pytest.approx(r0)    # weight 0: no change
    assert reward(w_joint_speed=1.0, joint_speed_soft_dps=1e5) == pytest.approx(r0)    # nothing above the soft speed
    assert reward(w_joint_speed=1.0, joint_speed_soft_dps=1.0) < r0                    # moving above it costs
