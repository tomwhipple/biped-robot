"""Sole-friction bench test in the CPU sim (hw 2026-11-04, legs-only body).

The bench (hw_sessions/2026-11-04/NOTES.md, fric_clamp*.json): the robot stands in the stand
pose, one foot C-clamped to the board, the other free at about half weight (~7.5 N).

  * Yaw: the free leg's hip yaw steps to +-20 deg at ~10 deg/s. The foot ROTATES on the
    board, tracking within 0.4-0.7 deg, no stick-slip; the servo load (40-72) is what an
    unloaded joint moving reads. Yaw grip <= ~0.1-0.15 N m, both feet.
  * Slide: the free leg's hip roll steps outward. The foot slides with no stick-slip; the
    extra hip-roll load (~1 N m over a ~0.29 m lever, ~3.4 N against ~7.5 N) gives
    mu_slide ~0.45-0.55 (rough).

Here: the plant settles standing free on both feet (each carries half the weight), then the
clamp foot is welded to the world where it stands and the free leg runs the same moves on the
plant's own position actuators (kp 40, quasi-static; the drive does not enter the numbers):

  * yaw: hip yaw 0 -> +20 -> -20 -> 0 deg at 10 deg/s. Reported: the free foot's floor
    contacts' torque about the hip-yaw axis (sliding + torsional friction), median |tau|
    while the joint moves, raw and scaled to the bench's 7.5 N (Coulomb: tau ~ N).
  * slide: hip roll outward to 10 deg at 2 deg/s. Reported: the contacts' |F_tangential| /
    F_normal while the foot slides (the sim's true mu), and the bench's own estimate on the
    same run, the rise of the hip-roll torque over (hip-roll axis height x the settled
    normal load), to check the bench arithmetic.

Contact overrides (`mu`, `torsion`) set the floor and every pad together, as
experiments/policy/shaper-train/grade.py's set_contact and env_mjx's --contact-mu /
--contact-torsion do, so the pair sees exactly that value on any plant.

    .venv/bin/python sim/sole_friction.py            # the table in docs/design-v6/2026-11-04-sole-friction.md
    .venv/bin/python sim/sole_friction.py --json out.json
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "disabled")

import mujoco  # noqa: E402

# the bench, 2026-11-04 (hw_sessions/2026-11-04/NOTES.md)
HW = dict(load_n=7.5, yaw_nm_max=(0.10, 0.15), mu_slide=(0.45, 0.55))

MEAS = os.path.join(HERE, "bimo_biped_v6legs_meas.xml")
SOLE = os.path.join(HERE, "bimo_biped_v6legs_meas_sole.xml")
# (label, plant, mu, torsion): None = the plant as drawn
CASES = [
    ("v6legs_meas as drawn (mu 1, tors 0.02)", MEAS, None, None),
    ("v6legs_meas, DR low end (0.35 / 0.001)", MEAS, 0.35, 0.001),
    ("v6legs_meas, DR high end (1.1 / 0.02)", MEAS, 1.1, 0.02),
    ("v6legs_meas, mu 0.5, tors 0 (edge pads)", MEAS, 0.5, 0.0),
    ("v6legs_meas_sole as drawn (mu 0.5, tors 0)", SOLE, None, None),
    ("v6legs_meas_sole, --contact-mu low (0.3)", SOLE, 0.3, None),
    ("v6legs_meas_sole, --contact-mu high (0.7)", SOLE, 0.7, None),
]


def load(xml, mu=None, torsion=None, clamp="R"):
    """The plant with an (inactive) weld on the clamp foot, and the contact override."""
    spec = mujoco.MjSpec.from_file(xml)
    spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                      name1=f"{clamp}_foot", active=False)
    m = spec.compile()
    gids = [m.geom("floor").id] + [g for g in range(m.ngeom) if "_pad_" in (m.geom(g).name or "")]
    if mu is not None:
        m.geom_friction[gids, 0] = mu
    if torsion is not None:
        m.geom_friction[gids, 1] = torsion
    return m


def clamp_here(m, d, clamp):
    """Weld the clamp foot to the world at its current pose (eq_data: anchor, relpos, relquat)."""
    bid = m.body(f"{clamp}_foot").id
    # weld(body1, world): keeps the world frame's pose in body1's frame
    R1 = d.xmat[bid].reshape(3, 3)
    m.eq_data[0, 0:3] = 0.0
    m.eq_data[0, 3:6] = -R1.T @ d.xpos[bid]
    q = np.zeros(4)
    mujoco.mju_negQuat(q, d.xquat[bid])
    m.eq_data[0, 6:10] = q
    m.eq_data[0, 10] = 1.0
    d.eq_active[0] = 1


def foot_contacts(m, d, side):
    """`side`'s pads on the floor: (normal N, tangential force vec (world), the contact torque
    about the hip-yaw axis N m, and its Coulomb bound sum_i N_i (mu_i r_i + torsion_i) N m)."""
    floor = m.geom("floor").id
    pads = {g for g in range(m.ngeom) if (m.geom(g).name or "").startswith(f"{side}_pad_")}
    j = m.joint(f"{side}_hip_yaw").id
    a, u = d.xanchor[j], d.xaxis[j]
    fn, ft, tau, bound = 0.0, np.zeros(3), 0.0, 0.0
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        if not ((g1 == floor and g2 in pads) or (g2 == floor and g1 in pads)):
            continue
        mujoco.mj_contactForce(m, d, i, f6)
        fr = c.frame.reshape(3, 3)            # rows: normal (geom1 -> geom2), t1, t2
        sgn = 1.0 if g2 in pads else -1.0     # the force on the pad
        f = sgn * (f6[0] * fr[0] + f6[1] * fr[1] + f6[2] * fr[2])
        t = sgn * f6[3] * fr[0]               # torsional friction torque
        fn += f @ fr[0] * sgn
        ft += f - (f @ fr[0]) * fr[0]
        tau += u @ (np.cross(c.pos - a, f) + t)
        r = c.pos - a
        bound += f6[0] * (c.friction[0] * np.linalg.norm(r - (r @ u) * u) + c.friction[2])
    return fn, ft, tau, bound


def run(xml, mu=None, torsion=None, clamp="R", settle=1.5):
    free = "L" if clamp == "R" else "R"
    m = load(xml, mu, torsion, clamp)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    act = {m.actuator(i).name: i for i in range(m.nu)}
    jq = {n: m.jnt_qposadr[m.joint(n).id] for n in act}
    dt = m.opt.timestep
    for _ in range(int(settle / dt)):
        mujoco.mj_step(m, d)
    n0 = foot_contacts(m, d, free)[0]
    clamp_here(m, d, clamp)
    for _ in range(int(0.3 / dt)):
        mujoco.mj_step(m, d)
    n_clamped = foot_contacts(m, d, free)[0]

    # ---- yaw: 0 -> +20 -> -20 -> 0 deg at 10 deg/s
    yaw = f"{free}_hip_yaw"
    rate = np.radians(10.0)
    path = [0.0, np.radians(20), np.radians(-20), 0.0]
    taus, norms, track, bounds = [], [], [], []
    tgt = 0.0
    for a, b in zip(path[:-1], path[1:]):
        n = int(abs(b - a) / rate / dt)
        for k in range(n):
            tgt = a + (b - a) * (k + 1) / n
            d.ctrl[act[yaw]] = tgt
            mujoco.mj_step(m, d)
            if k > n // 10 and k < n - n // 10:       # moving, away from the turn-arounds
                fn, _, tau, bound = foot_contacts(m, d, free)
                taus.append(abs(tau))
                bounds.append(bound)
                norms.append(fn)
                track.append(abs(np.degrees(tgt - d.qpos[jq[yaw]])))
    for _ in range(int(0.5 / dt)):
        mujoco.mj_step(m, d)
    tau_med, n_med = float(np.median(taus)), float(np.median(norms))

    # ---- slide: hip roll outward to 10 deg at 2 deg/s (outward = the foot moves away from the body)
    roll = f"{free}_hip_roll"
    out_sign = 1.0 if free == "L" else -1.0           # +roll about +x swings the L leg to +y
    ja = m.joint(roll).id
    bfoot = m.body(f"{free}_foot").id
    y0 = d.xpos[bfoot][1]
    lever = d.xanchor[ja][2]                          # hip-roll axis above the floor
    q0r = d.qpos[jq[roll]]
    rate = np.radians(2.0)
    n = int(np.radians(10.0) / rate / dt)
    tau_roll, ratio, slid, fns = [], [], [], []
    for k in range(n):
        d.ctrl[act[roll]] = q0r + out_sign * np.radians(10.0) * (k + 1) / n
        mujoco.mj_step(m, d)
        fn, ft, _, _ = foot_contacts(m, d, free)
        tau_roll.append(out_sign * d.actuator_force[act[roll]])
        ratio.append(np.linalg.norm(ft) / max(fn, 1e-6))
        fns.append(fn)
        slid.append(out_sign * (d.xpos[bfoot][1] - y0))
    slid = np.array(slid)
    moving = slid > 0.002                              # the foot has slid 2 mm
    ft_fn = float(np.median(np.array(ratio)[moving])) if moving.any() else float("nan")
    d_tau = float(np.max(np.array(tau_roll)[moving]) - tau_roll[0]) if moving.any() else float("nan")
    mu_bench = d_tau / lever / n_clamped
    return dict(n_free=n0, n_clamped=n_clamped, yaw_tau=tau_med, yaw_n=n_med,
                yaw_tau_7p5=tau_med * HW["load_n"] / n_med, yaw_track_deg=float(np.median(track)),
                yaw_coulomb_7p5=float(np.median(bounds)) * HW["load_n"] / n_med,
                slide_ft_fn=ft_fn, mu_bench_method=mu_bench, slide_mm=float(1e3 * slid[-1]),
                roll_lever_m=float(lever), slide_n_peak=float(np.max(fns)))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--clamp", default="R", choices=("L", "R"), help="the clamped foot (bench: R, then L)")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    print(f"bench 2026-11-04: free foot ~{HW['load_n']} N; yaw grip <= {HW['yaw_nm_max'][0]}-"
          f"{HW['yaw_nm_max'][1]} N m (rotates, no stick-slip); mu_slide ~{HW['mu_slide'][0]}-{HW['mu_slide'][1]}")
    hdr = (f"{'case':46s} | {'N free':>6s} | {'yaw tau':>7s} {'@7.5N':>6s} {'x bench':>8s} {'Coulomb':>7s} | "
           f"{'Ft/Fn':>5s} {'N peak':>6s} {'mu, bench method':>16s}")
    print(hdr)
    print("-" * len(hdr))
    out = []
    for label, xml, mu, tor in CASES:
        r = run(xml, mu, tor, clamp=a.clamp)
        r.update(case=label, xml=os.path.basename(xml), mu=mu, torsion=tor)
        out.append(r)
        print(f"{label:46s} | {r['n_clamped']:6.2f} | {r['yaw_tau']:7.3f} {r['yaw_tau_7p5']:6.3f} "
              f"{r['yaw_tau_7p5'] / HW['yaw_nm_max'][1]:5.1f}-{r['yaw_tau_7p5'] / HW['yaw_nm_max'][0]:3.1f} "
              f"{r['yaw_coulomb_7p5']:7.3f} | {r['slide_ft_fn']:5.2f} {r['slide_n_peak']:6.1f} "
              f"{r['mu_bench_method']:16.2f}", flush=True)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
    return out


if __name__ == "__main__":
    main()
