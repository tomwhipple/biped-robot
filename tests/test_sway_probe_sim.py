"""experiments/sway-probe/sway_sim.py: the rehearsal's plant, playback and guard numbers (CPU, one short episode each)."""
import math
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "experiments", "sway-probe"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import sway_ref as SR  # noqa: E402
import sway_sim as S  # noqa: E402


@pytest.fixture(scope="module")
def setup():
    cfg = SR.load_config()
    ref = SR.SwayReference(cfg)
    rung = ref.rung(3, "sway", 0.039, cycles=2, ramp_s=1.0)
    return cfg, ref, rung


def test_guard_numbers_are_the_firmwares():
    fg = open(os.path.join(ROOT, "firmware/components/fallguard/include/fallguard/fallguard.h")).read()
    m = re.search(r"kBenchLimits = \{([\d.]+)f, (\d+)u\}", fg)
    assert math.degrees(math.acos(float(m.group(1)))) == pytest.approx(S.FALL_BENCH_DEG, abs=0.01) and int(m.group(2)) == 100
    fl = open(os.path.join(ROOT, "firmware/components/flailguard/include/flailguard/flailguard.h")).read()
    m = re.search(r"kLimits = \{(\d+)u, ([\d.]+)f, ([\d.]+)f, ([\d.]+)f, (\d+)u,", fl)
    assert float(m.group(2)) == S.FLAIL_RATE and float(m.group(3)) == S.FLAIL_TAU
    assert float(m.group(4)) == S.FLAIL_TILT_DEG and int(m.group(5)) == 200


def test_band_is_the_recipes_dr_ranges_and_deterministic(setup):
    cfg = setup[0]
    b = S.Band(cfg)
    assert b.r["mu"] == (0.3, 0.7) and b.r["roll_lash_deg"] == (0.5, 2.0) and b.r["lag_hz"] == (2.65, 8.0)
    a, c = b.lhs(8, 3), b.lhs(8, 3)
    assert a == c and len(a) == 8
    for p in a:
        assert 0.3 <= p.mu <= 0.7 and 0.6 <= p.torso_scale <= 1.4 and 2 <= p.delay_3215 <= 4 and 0 <= p.delay_3250 <= 1
    assert b.mid().delay_3215 == 3


def test_the_plant_edits_reach_the_env(setup):
    cfg = setup[0]
    p = S.Plant(mu=0.4, jfric=0.3, torso_scale=1.2, torso_com_y=0.01, lag_hz=4.0, delay_3215=2, delay_3250=1, roll_lash_deg=1.5)
    env = S.build_env(cfg, p, "prec")
    m = env.model
    assert m.geom_friction[env._floor_gid, 0] == 0.4 and np.all(m.dof_frictionloss[env._sqvel] == 0.3)
    assert m.body_mass[env._torso_bid] == pytest.approx(1.2 * env._nom_mass[env._torso_bid])
    assert m.body_ipos[env._torso_bid, 1] == pytest.approx(env._nom_ipos[env._torso_bid, 1] + 0.01)
    assert env._act_lag_ep == 4.0 and env.fw_shaper_hz == 8.0 and S.build_env(cfg, p, "pose").fw_shaper_hz == 0.0
    assert sorted(set(env._act_delay_vec)) == [1, 2]
    assert env._roll_lash_rad == pytest.approx(math.radians(1.5))


def test_action_inverse_round_trips_the_targets(setup):
    cfg, ref, rung = setup
    env = S.build_env(cfg, S.Plant())
    q = rung.q[60]
    assert np.allclose(env._action_to_ctrl(S.inv_action(env, q)), q, atol=1e-6)


def test_the_standing_robot_carries_its_weight_split_between_the_feet(setup):
    cfg, ref, rung = setup
    env = S.build_env(cfg, S.Plant())
    env.reset(seed=0)
    for _ in range(50):
        env.step(np.zeros(12))
    F, H = S.foot_state(env)
    assert sum(F) == pytest.approx(S.weight_n(env), rel=0.02)
    assert abs(F[0] - F[1]) < 0.3 * S.weight_n(env) and max(abs(h) for h in H) < 1e-3


def test_one_rung_runs_and_reports_per_side_numbers(setup):
    cfg, ref, rung = setup
    for seg, bal in ((0.0, "off"), (200.0, "off"), (200.0, "stance")):
        env = S.build_env(cfg, S.Plant(), "pose")
        tr = S.play(env, rung, seg, 0.0, bal)
        m = S.metrics(env, rung, tr)
        assert m["up"] and m["completed"] and m["tilt_peak_deg"] < 10
        assert set(m["sides"]) == {"L", "R"}
        for s in m["sides"].values():
            assert 0.0 <= s["F_min_frac"] <= 1.0 and s["clear_mm"] >= -1.0 and "com_margin_mm" in s
        assert m["margin_fall_deg"] == pytest.approx(S.FALL_BENCH_DEG - m["tilt_peak_deg"])
        assert (tr["u_peak_deg"] > 0) == (bal == "stance")


def test_chained_playback_starts_each_segment_from_the_readback(setup):
    cfg, ref, rung = setup
    env = S.build_env(cfg, S.Plant())
    env.reset(seed=0)
    st = S.Stream(env, rung, 200, 0, 0)
    g0 = st.goal(0)
    k = ref.names.index("L_hip_roll")
    start = st.q_start[k]                                          # the joints' present position (reset noise included)
    assert np.allclose(st.q_start, env._quant_angle(np.asarray(env.data.qpos[env._sqpos], dtype=float)))
    final = rung.q[10][k]                                          # the keyframe 200 ms (10 ticks) ahead
    assert g0[k] == pytest.approx(start + SR.minjerk(0.1) * (final - start))
    goals = [g0] + [st.goal(i) for i in range(1, 10)]
    assert goals[-1][k] == pytest.approx(final, abs=1e-12)         # the move lands on that keyframe
    assert all(np.sign(b - a) == np.sign(final - start) or abs(b - a) < 1e-12 for a, b in zip([g[k] for g in goals], [g[k] for g in goals][1:]))


def test_the_default_plant_is_the_old_one_and_refit_is_opt_in(setup):
    cfg = setup[0]
    b = S.Band(cfg)
    assert not b.refit and "ser_hip" not in b.r and b.r["torso_scale"] == (0.6, 1.4)
    env = S.build_env(cfg, S.Plant(), "pose")
    assert env.roll_series_k is None and env.model.body_mass[env._torso_bid] == pytest.approx(env._nom_mass[env._torso_bid])
    assert all(p.ser_hip == 0.0 and p.ser_ankle == 0.0 for p in b.lhs(4, 1))


def test_refit_band_draws_the_springs_and_fixes_the_mass(setup):
    cfg = setup[0]
    b = S.Band(cfg, refit=True)
    assert b.r["ser_hip"] == (10.0, 25.0) and b.r["ser_ankle"] == (16.0, 40.0) and b.r["torso_scale"] == (1.0, 1.0)
    old = S.Band(cfg).lhs(8, 3)
    new = b.lhs(8, 3)
    assert b.lhs(8, 3) == new
    for p in new:
        assert 10.0 <= p.ser_hip <= 25.0 and 16.0 <= p.ser_ankle <= 40.0 and p.torso_scale == 1.0
    assert len({p.ser_hip for p in new}) == 8 and len({p.ser_ankle for p in new}) == 8     # Latin hypercube: one draw per slice
    assert [p.mu for p in new] == [p.mu for p in old]                                      # the DR dimensions keep their draws
    assert b.mid().ser_hip == 17.5 and b.mid().ser_ankle == 28.0


def test_refit_env_carries_the_mass_and_the_springs(setup):
    cfg = setup[0]
    p = S.Plant(ser_hip=15.0, ser_ankle=30.0)
    env = S.build_env(cfg, p, "pose", refit=True)
    assert env.roll_series_k == {"hip": (15.0, 15.0), "ankle": (30.0, 30.0)}
    assert S.weight_n(env) / S.G == pytest.approx(1.621, abs=0.002)                        # the weighed robot, with the pack
    env.reset(seed=0)
    assert env._ser_c is not None and float(env._ser_c.max()) == pytest.approx(1 / 15.0)


def test_refit_rung_runs_and_the_meta_says_which_plant(setup):
    cfg, ref, rung = setup
    env = S.build_env(cfg, S.Band(cfg, True).mid(), "pose", refit=True)
    m = S.metrics(env, rung, S.play(env, rung, 200.0, 0.0, "off"))
    assert m["up"] and m["completed"] and set(m["sides"]) == {"L", "R"}
    res = S.rehearse(cfg, Ms=[0.03], n=1, chain="pose", seg_ms=200.0, refit=True)
    assert res["meta"]["plant"] == "refit" and tuple(res["meta"]["band"]["ser_hip"]) == (10.0, 25.0)
