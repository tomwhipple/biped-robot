"""Foot outline study (2026-10-07): how far inboard can the sole reach?

The sole's inboard half sets two things that pull opposite ways:
  * the walk's CoM margin, which is inboard-limited (dimensions_v6 FOOT_*), and
  * the gap between the feet when the hip rolls adduct toward each other
    (check_assembly_v6's interleg row, 3 deg mutual adduction).

Modes (logs in docs/design-v6/foot_outline_*.txt):

    .venv/bin/python sim/foot_outline_study.py interleg   # foot-to-foot gap vs mutual adduction, the CAD feet
    .venv/bin/python sim/foot_outline_study.py gateD      # gate_no3250.mode_sweep's matrix at three sole placements

`interleg` builds the CAD assembly with the committed foot outline (cad/v6/foot_v6.py).
`gateD` runs the robot's servo set (STS3250 at the hip rolls, the other rolls and the
knees tuned STS3215 at 2.8x; gate_no3250 M7b, 74.5 g plant, arms) at foot_y_off 12 /
18.5 / 24 mm, keeping the 84 mm width. On the Mac: MUJOCO_GL=cgl.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "..", "cad", "v6"), os.path.join(HERE, "..", "cad")]

Y_OFFS = (0.012, 0.0185, 0.024)          # 30 / 23.5 / 18 mm inboard of the ankle roll axis
ANGLES = (0.0, 1.0, 1.5, 2.0, 2.5, 3.0)  # mutual hip-roll adduction, deg (ankles keep the soles flat)

CASES = [("nominal (mu 0.7, play 3, lash 1, lag 2 Hz + 80 ms)", {})]
for _mu in (0.3, 0.5, 1.0):
    CASES.append((f"friction mu {_mu}", dict(mu=_mu)))
CASES += [("no play, no backlash", dict(play_deg=0.0, backlash_deg=0.0)),
          ("play 1 deg on rolls", dict(play_deg=1.0)), ("play 5 deg on rolls", dict(play_deg=5.0)),
          ("backlash 2 deg", dict(backlash_deg=2.0)),
          ("mass x0.85", dict(mass_scale=0.85)), ("mass x1.15", dict(mass_scale=1.15)),
          ("payload 60 g", dict(payload=0.06)), ("payload 120 g", dict(payload=0.12)),
          ("servos -15 %", dict(servo_scale=0.85)), ("servos -30 %", dict(servo_scale=0.70)),
          ("floor tilt +2 deg", dict(floor_tilt_deg=2.0)), ("floor tilt -2 deg", dict(floor_tilt_deg=-2.0)),
          ("no lag / no dead time", dict(lag_hz=0.0, delay_ticks=0)),
          ("mu 0.3 + play 5 + servos -15 %", dict(mu=0.3, play_deg=5.0, servo_scale=0.85))]


def _interleg_one(th):
    import assembly_v6 as A
    import check_assembly_v6 as C
    pose = {"L_hip_roll": -th, "R_hip_roll": th, "L_ankle_roll": th, "R_ankle_roll": -th}
    P = C.pieces_by_label(A.robot(pose))
    try:
        vol = (P["foot_L"] & P["foot_R"]).volume
    except Exception:  # noqa: BLE001
        vol = 0.0
    return th, vol, P["foot_L"].distance_to(P["foot_R"])


def mode_interleg():
    from multiprocessing import Pool
    with Pool(len(ANGLES)) as pool:
        rows = pool.map(_interleg_one, ANGLES)
    print("== foot_L vs foot_R, CAD feet as committed, both hip rolls adducted by the angle, ankles flat")
    for th, vol, dist in rows:
        print(f"   mutual adduction {th:4.1f} deg: overlap {vol:8.1f} mm3, gap {dist:6.2f} mm")


def mode_gated():
    import gate_no3250 as G
    import static_gait as SG
    from gen_plant_v6 import DesignParams
    sn = G._mix_name("2.8x", G.HIP_ROLLS)
    jobs, plants = [], {}
    for yo in Y_OFFS:
        p = DesignParams(foot_y_off=yo)
        plants[yo] = G.cad_plant(74.5, p=p)
        jobs += [(f"{yo}|{cl}#{s}", p, plants[yo], sn, {}, dict(seed=s, **ckw), {})
                 for cl, ckw in CASES for s in range(3)]
    res = G.pmap(G._walk_job_seeded, jobs, max(1, min(12, (os.cpu_count() or 4) - 2)))
    by = {}
    for label, r in res:
        by.setdefault(label.split("#")[0], []).append(r)
    for yo in Y_OFFS:
        print(f"== sole {42 - yo * 1e3:.1f} mm inboard / {42 + yo * 1e3:.1f} outboard (foot_y_off {yo * 1e3:.1f} mm), "
              f"M7b, plant {G.plant_mass(plants[yo]):.3f} kg, 8 steps, lift 4 cm, worst of 3 seeds")
        n, margins = 0, []
        for cl, _ in CASES:
            rs = by[f"{yo}|{cl}"]
            worst = min(rs, key=lambda r: (r["ok"], r["steps_completed"], r["min_clear_peak"]))
            n += all(r["ok"] for r in rs)
            margins.append(min(r["min_margin"] for r in rs))
            print(SG.fmt_row(cl, worst) + f"   ({sum(r['ok'] for r in rs)}/3 seeds ok)", flush=True)
        margins.sort()
        print(f"   -> {n}/{len(CASES)} cases OK on every seed; CoM margin worst {margins[0] * 1e3:.1f} mm, "
              f"median {margins[len(margins) // 2] * 1e3:.1f} mm\n", flush=True)


if __name__ == "__main__":
    os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "interleg":
        os.environ.setdefault("ARMS", "0")
        mode_interleg()
    elif mode == "gateD":
        mode_gated()
    else:
        sys.exit(__doc__)
