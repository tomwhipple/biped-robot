"""The robot's body yaw at step onset, replayed: prescribe the MEASURED joint angles of the
2026-11-03 flight record (walk18_shp10sq, stride1 vx 0.6, every tick recorded) on the free-base
plant, with joint equality constraints instead of servos, and see what the plant's body does.
No policy and no actuator model are in the loop, so what differs from the robot is the plant's
body or contact (hw_sessions/2026-11-05/kick_analysis.md section 5, Q3).

    python replay_joints.py [variant ...]     # no args: every variant; prints yaw vs the robot

Robot at +180 ms: yaw -19.2 deg, peak |gz| 3.9 rad/s. Plant: bimo_biped_v6legs_meas_sole.xml.
"""
from __future__ import annotations
import math, os, sys
from concurrent.futures import ProcessPoolExecutor
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
XML = os.path.join(ROOT, "sim", "bimo_biped_v6legs_meas_sole.xml")
RECORDS = dict(w18=("w18_1103.npz", 240), o5=("o5_1106_t1.npz", 239))      # file, tick of the walk command
REC = os.path.join(HERE, RECORDS[os.environ.get("YAW_REC", "w18")][0])
REC_JOINTS = ["R_hip_roll", "R_hip_pitch", "R_knee", "R_ankle", "L_hip_roll", "L_hip_pitch",
              "L_knee", "L_ankle", "R_hip_yaw", "L_hip_yaw", "R_ankle_roll", "L_ankle_roll"]
IZZ_REF = 0.004      # N m s^2: rough body yaw inertia; the constraint's solref is per unit inertia
I0 = RECORDS[os.environ.get("YAW_REC", "w18")][1]
MARKS = tuple(int(x) for x in os.environ.get("YAW_MS", "60,120,180,240,300").split(","))


def load_record():
    d = np.load(REC)
    t = d["t"] - d["t"][I0]
    yaw = np.array([math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
                    for q in d["quat"]])
    yaw = np.degrees(np.unwrap(yaw))
    yaw -= yaw[I0 - 1]
    return t, d["pos"], yaw, d["gyro"]


def build(v):
    spec = mujoco.MjSpec.from_file(XML)
    for j in REC_JOINTS:
        e = spec.add_equality()
        e.type = mujoco.mjtEq.mjEQ_JOINT
        e.objtype = mujoco.mjtObj.mjOBJ_JOINT
        e.name1 = j
        e.name = "fix_" + j
        e.data[:5] = 0
        e.solref[:2] = [0.004, 1.0]
        e.solimp[:3] = [0.98, 0.999, 0.0005]
    m = spec.compile()
    for a in range(m.nu):
        m.actuator_gainprm[a, 0] = 0
        m.actuator_biasprm[a, :3] = 0
    m.dof_frictionloss[:] = 0
    m.dof_damping[6:] = 0
    return m


def apply_variant(m, v):
    bid = lambda n: m.body(n).id
    if "inertia" in v:      # scale the torso's inertia tensor
        m.body_inertia[bid("torso")] *= v["inertia"]
    if "leg_mass" in v:     # every leg body, mass and inertia
        for n in range(m.nbody):
            if n and m.body(n).name != "torso":
                m.body_mass[n] *= v["leg_mass"]
                m.body_inertia[n] *= v["leg_mass"]
    if "torso_mass" in v:
        m.body_mass[bid("torso")] *= v["torso_mass"]
        m.body_inertia[bid("torso")] *= v["torso_mass"]
    pads = [g for g in range(m.ngeom) if "_pad_" in (m.geom(g).name or "")]
    floor = m.geom("floor").id
    if "mu" in v:
        for g in pads + [floor]:
            m.geom_friction[g, 0] = v["mu"]
    if "torsion" in v:
        for g in pads + [floor]:
            m.geom_friction[g, 1] = v["torsion"]
    if "pad_r" in v:        # scale the pads toward the foot centre (yaw grip goes with r)
        c = {}
        for g in pads:
            c.setdefault(m.geom_bodyid[g], []).append(g)
        for gs in c.values():
            mean = m.geom_pos[gs].mean(0)
            m.geom_pos[gs] = mean + (m.geom_pos[gs] - mean) * [v["pad_r"], v["pad_r"], 1.0]
    if "yaw_k" in v:        # hip yaw servo + carrier as a soft torsional spring (N m/rad), damping ratio ~0.3
        for j in ("L_hip_yaw", "R_hip_yaw"):
            m.eq_solref[m.equality("fix_" + j).id] = [-v["yaw_k"] / IZZ_REF, -v.get("yaw_b", 0.05) / IZZ_REF]
    if v.get("no_gravity"):
        m.opt.gravity[:] = 0
    if v.get("no_contact"):
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT


def replay(name_v):
    name, v = name_v
    t, pos, yaw_r, gyro_r = load_record()
    m = build(v)
    apply_variant(m, v)
    d = mujoco.MjData(m)
    jq = [m.jnt_qposadr[m.joint(j).id] for j in REC_JOINTS]
    eqs = [m.equality("fix_" + j).id for j in REC_JOINTS]
    q0 = pos[I0 - 5]
    d.qpos[2] = 0.5

    def set_targets(q):
        for e, x in zip(eqs, q):
            m.eq_data[e, 0] = x

    set_targets(q0)
    d.qpos[jq] = q0
    if v.get("no_contact"):
        d.qpos[2] = 0.45
    else:
        # lower it onto the floor, then settle
        mujoco.mj_forward(m, d)
        z = min(d.geom_xpos[g, 2] - m.geom_size[g, 2] for g in range(m.ngeom)
                if "_pad_" in (m.geom(g).name or ""))
        d.qpos[2] -= z - 0.001
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    # replay from just before the command
    t0 = t[I0 - 5]
    t1 = t[-1]
    out_t, out_y, out_g, out_fz, y0 = [], [], [], [], None
    n = int((t1 - t0) / m.opt.timestep)
    foot = [m.body("L_foot").id, m.body("R_foot").id]
    yaw_of = lambda q: math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    fy0 = None
    for k in range(n):
        now = t0 + k * m.opt.timestep
        shift = v.get("shift_ms", 0) / 1e3      # the record's joints are older than its gyro by this
        tgt = [np.interp(now + shift, t, pos[:, i]) for i in range(12)]
        for i, j in enumerate(REC_JOINTS):      # hold the joints this variant does not replay
            if "only" in v and not any(x in j for x in v["only"]):
                tgt[i] = q0[i]
        set_targets(tgt)
        mujoco.mj_step(m, d)
        if k % 10 == 9:
            if y0 is None:
                y0 = yaw_of(d.qpos[3:7])
            out_t.append(now)
            out_y.append(math.degrees(yaw_of(d.qpos[3:7]) - y0))
            out_g.append(d.qvel[5])
            out_fz.append([yaw_of(d.xquat[f]) for f in foot])
    out_t, out_y, out_g = map(np.array, (out_t, out_y, out_g))
    out_f = np.degrees(np.unwrap(np.array(out_fz), axis=0))
    out_f -= out_f[0]
    sim_y = np.interp(t, out_t, out_y)
    win = (t >= 0.0) & (t <= MARKS[-1] / 1e3)
    rms = float(np.sqrt(np.mean((sim_y[win] - yaw_r[win]) ** 2)))
    at = lambda ms, arr=out_y: float(np.interp(ms / 1e3, out_t, arr))
    return dict(name=name, y={ms: at(ms) for ms in MARKS}, gz_peak=float(np.abs(out_g[out_t < MARKS[-1] / 1e3]).max()),
                rms=rms, footL180=at(180, out_f[:, 0]), footR180=at(180, out_f[:, 1]), z=float(d.qpos[2]))


VARIANTS = {
    "only R hip pitch + R knee": dict(only=("R_hip_pitch", "R_knee")),
    "only R leg sagittal": dict(only=("R_hip_pitch", "R_knee", "R_ankle")),
    "only R leg": dict(only=("R_",)),
    "only L leg": dict(only=("L_",)),
    "only hip yaws": dict(only=("hip_yaw",)),
    "only hip rolls + ankle rolls": dict(only=("hip_roll", "ankle_roll")),
    "no hip rolls/ankle rolls": dict(only=("hip_pitch", "knee", "_ankle", "hip_yaw")),
    "records 20 ms late": dict(shift_ms=20),
    "records 40 ms late": dict(shift_ms=40),
    "records 60 ms late": dict(shift_ms=60),
    "40 ms late + free body": dict(shift_ms=40, no_contact=True, no_gravity=True),
    "hip yaw soft k=1": dict(yaw_k=1.0),
    "hip yaw soft k=3": dict(yaw_k=3.0),
    "hip yaw soft k=10": dict(yaw_k=10.0),
    "plant (mu 0.5, torsion 0)": {},
    "no contact, no gravity (free body)": dict(no_contact=True, no_gravity=True),
    "mu 1.0": dict(mu=1.0),
    "mu 0.3": dict(mu=0.3),
    "mu 0.15": dict(mu=0.15),
    "mu 0.05": dict(mu=0.05),
    "pads r x0.5": dict(pad_r=0.5),
    "torso inertia x0.5": dict(inertia=0.5),
    "torso inertia x0.25": dict(inertia=0.25),
    "legs mass x1.5": dict(leg_mass=1.5),
    "legs mass x2": dict(leg_mass=2.0),
    "torso mass x0.7": dict(torso_mass=0.7),
    "free body: legs mass x2": dict(no_contact=True, no_gravity=True, leg_mass=2.0),
    "free body: torso inertia x0.25": dict(no_contact=True, no_gravity=True, inertia=0.25),
    "mu 0.3 + torso inertia x0.5": dict(mu=0.3, inertia=0.5),
    "mu 0.15 + legs mass x1.5": dict(mu=0.15, leg_mass=1.5),
}


def main():
    names = sys.argv[1:] or list(VARIANTS)
    t, pos, yaw_r, gyro_r = load_record()
    rob = {ms: float(np.interp(ms / 1e3, t, yaw_r)) for ms in MARKS}
    print(f"{'ROBOT':38s}" + " ".join(f"{rob[ms]:7.1f}" for ms in rob) + f"   |gz| {np.abs(gyro_r[I0:I0 + MARKS[-1] // 20, 2]).max():.1f}")
    print(f"{'yaw deg at +ms':38s}" + " ".join(f"{ms:7d}" for ms in rob) + "   |gz|peak  rms0-300  foot L/R yaw at +180")
    with ProcessPoolExecutor(max_workers=4) as ex:
        for r in ex.map(replay, [(n, VARIANTS[n]) for n in names]):
            print(f"{r['name']:38s}" + " ".join(f"{r['y'][ms]:7.1f}" for ms in rob)
                  + f"   {r['gz_peak']:6.1f}  {r['rms']:6.1f}   {r['footL180']:6.1f} {r['footR180']:6.1f}  z {r['z']:.3f}")


if __name__ == "__main__":
    main()
