"""The short-walk referee scenario (eval_precision.walk_steps_L/R): every step
of a few-step walk must lift ITS foot's sole >= 3 cm, per foot and per step
(Tom 2026-10-24), plus the registry wiring (extra scenarios, the run's
`eval_scenarios`) and balance's one-foot-time metrics."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")

DT = 0.02


def _samples(swings, t_end=7.0):
    """per-tick (t, con_l, con_r, z_l, z_r) from [(foot, t_on, ticks, peak)]; a swing is a triangle."""
    n = int(round(t_end / DT))
    con = {"L": [True] * n, "R": [True] * n}
    z = {"L": [0.0] * n, "R": [0.0] * n}
    for foot, t_on, ticks, peak in swings:
        k0 = int(round(t_on / DT))
        for j in range(ticks):
            con[foot][k0 + j] = False
            z[foot][k0 + j] = peak * (1 - abs(2 * (j + 0.5) / ticks - 1))
    return [(k * DT, con["L"][k], con["R"][k], z["L"][k], z["R"][k]) for k in range(n)]


def _rows(x_end=0.5, h=1.0, up=1.0, y=0.0):
    return [dict(t=0.0, x=0.0, y=0.0, height=h, up_z=up), dict(t=7.0, x=x_end, y=y, height=h, up_z=up)]


def _eval(swings, **rk):
    import eval_precision as E
    build, evaluate = E.scen_walk_steps("L")
    _, ev = build()
    ev["_h"] = _samples(swings)
    ev["x0"], ev["y0"] = 0.0, 0.0
    fell = rk.pop("fell", False)
    return evaluate(_rows(**rk), ev, fell, 1.0, {})


GOOD = [("L", 1.1, 14, 0.05), ("R", 1.5, 14, 0.05), ("L", 1.9, 14, 0.045), ("R", 2.3, 14, 0.05),
        ("L", 2.7, 14, 0.045)]


def test_swing_extraction_per_foot_and_window():
    import eval_precision as E
    sw = E.walk_steps_swings(_samples(GOOD + [("R", 5.0, 14, 0.05)]), 1.0, 3.2)
    assert [f for f, _, _ in sw] == ["L", "R", "L", "R", "L"]       # the t=5 swing is outside the window
    assert all(0.04 < p <= 0.05 for _, _, p in sw)
    # chatter shorter than SWING_MIN_TICKS is not a step
    assert E.walk_steps_swings(_samples([("L", 1.1, 2, 0.001)]), 1.0, 3.2) == []


def test_pass_when_every_step_clears_three_cm():
    r = _eval(GOOD)
    assert r["success"] and r["metrics"]["steps"] == 5 and r["metrics"]["steps_short"] == 0
    assert r["metrics"]["min_peak_clear"] >= 0.03 and r["metrics"]["lead_ok"] == 1.0


def test_one_low_step_fails_it_even_if_the_average_is_fine():
    low = list(GOOD)
    low[3] = ("R", 2.3, 14, 0.02)                      # a 2 cm shuffle among 5 cm steps
    r = _eval(low)
    assert not r["success"] and r["metrics"]["steps_short"] == 1
    assert abs(r["metrics"]["max_shortfall"] - 0.01) < 5e-3


def test_the_stopping_step_is_graded_too():
    stop = GOOD + [("R", 3.0, 8, 0.015)]               # begins 0.2 s after the stop command (2.8 s)
    assert not _eval(stop)["success"]


def test_one_foot_only_is_not_a_walk():
    hop = [("L", 1.1 + 0.4 * k, 14, 0.05) for k in range(5)]
    assert not _eval(hop)["success"]


def test_needs_progress_upright_and_no_fall():
    assert not _eval(GOOD, x_end=0.1)["success"]
    assert not _eval(GOOD, h=0.5)["success"]
    assert not _eval(GOOD, fell=True)["success"]
    assert not _eval(GOOD, y=0.4)["success"]


def test_wrong_lead_foot_is_reported_not_failed():
    r = _eval([("R", 1.1, 14, 0.05), ("L", 1.5, 14, 0.05), ("R", 1.9, 14, 0.05), ("L", 2.3, 14, 0.05)])
    assert r["success"] and r["metrics"]["lead_ok"] == 0.0


def test_registry_and_family_lists():
    import eval_precision as E
    reg = E._registry()
    assert "walk_steps_L" in reg and "walk_steps_R" in reg and reg["walk_steps_L"][2] is True
    for fam in E.FAMILY_SCENARIOS.values():
        assert not {"walk_steps_L", "walk_steps_R"} & set(fam)        # the comparable totals stay put
    assert {"walk_steps_L", "walk_steps_R"} <= set(E.EXTRA_SCENARIOS)
    assert E._headline("walk_steps_L", {"min_peak_clear": {"mean": 0.041}}) == "min peak 4.1cm"


def test_scenario_runs_on_the_legs_plant_with_a_zero_policy():
    """An end-to-end rollout on the as-built plant: the zero-action stand never walks, so
    it fails the scenario with zero steps -- and the rollout, the clock preset and the
    sole-height sampling run without error for both lead feet."""
    import eval_precision as E
    import robot_plant
    cfg = dict(robot_plant.robot_env_kwargs(robot_plant.ROBOT_LEGS_XML))
    cfg.update(action_map="full", gait_clock=True, ext_cmd=True, obs_hist_len=3, joint_frictionloss=0.35,
               servo_kp_scale="robot", roll_lash_deg=1.0, roll_lash_deg_max=2.6,
               torso_mass_range=0.4, torso_com_y=0.015, imu_mount_deg=3.0)
    xml = robot_plant.robot_xml_path(robot_plant.ROBOT_LEGS_XML)
    reg = E._registry()
    for name in ("walk_steps_L", "walk_steps_R"):
        secs, fac, _ = reg[name]
        env = E.make_env(cfg, secs, False, xml)
        n = env.action_space.shape[0]
        r = E.run_one(env, lambda o: np.zeros(n, np.float32), fac, seed=7, record=False,
                      N=float(env._nominal_h), mass=float(env.model.body_mass.sum()))
        assert not r["success"] and r["metrics"]["steps"] == 0 and not r["fell"]


def test_balance_reports_one_foot_time():
    import eval_precision as E
    build, evaluate = E.scen_balance("L")
    rows = []
    for k in range(600):
        t = (k + 1) * DT
        lifted = t >= 1.5
        rows.append(dict(t=t, x=0.0, y=0.0, con_l=not lifted, con_r=True, foot_clear=0.04 if lifted else 0.0,
                         wob2=0.0, watts=1.0, com_stance=0.01))
    r = evaluate(rows, {}, False, 1.0, {})
    m = r["metrics"]
    assert r["success"] and abs(m["one_foot_s"] - 9.5) < 0.1 and abs(m["one_foot_run_s"] - 9.5) < 0.1
