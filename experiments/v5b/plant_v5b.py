"""The v5b plant: the v5 prototype with the v6 ankle link, ankle-roll servo and
foot under each shin.

v5b is the physical v5 body (torso and legs as-is, the v5body plant pinned by v5_cad at the commit the parts were printed from)
whose two feet are replaced by the v6 ankle chain (`sim/bimo_biped_v6ar.xml`:
`*_ankle_blk` carrying the ankle-pitch joint and the pitch servo case, then
`*_foot` on an ankle-roll joint, with the v6 sole pads), hung at the v5 shin's
ankle point (0, 0, -0.09). It exists to bench-test one-footed balance, i.e. the
physics of the ankle roll.

The model is built in memory with MjSpec: the v6 subtrees are attached with
their meshes, geoms (classes already resolved) and the two actuators per leg;
the v5 feet, their actuators and the foot contact pairs are removed. No XML is
committed because the two source plants use different mesh directories.

All 12 joints are one servo, the STS3215 of the v5 plant: the attached
actuators are reset to v5body's position-actuator model (kp 40, kv 0 via
biasprm, forcerange +-3 N*m) and its joint defaults (damping 0.6, armature
0.008, frictionloss 0.02), which the v6 plant shares. The ankle-pitch range is the
firmware's +-40 deg (firmware/main/mech_envelope.h, v5body's value), not the
v6 plant's +-45; the ankle-roll range is v6's +-25 deg, also the firmware's.

    .venv/bin/python experiments/v5b/plant_v5b.py
"""
from __future__ import annotations

import math
import os

import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v5_cad  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
V5 = v5_cad.v5body_xml()          # pinned: the printed v5 parts are frozen
V6 = os.path.join(ROOT, "sim", "bimo_biped_v6ar.xml")

ANKLE_Z = -0.09                   # shin frame -> ankle point, as the v5 foot
ANKLE_RANGE_DEG = 40.0            # firmware envelope (mech_envelope.h), v5body's range
PREFIX = {"L": "v6L_", "R": "v6R_"}

# sim actuator order is not bus order; servo IDs per docs/servo-map.md 2.1
SERVO_ID = {
    "R_hip_roll": 1, "R_hip_pitch": 2, "R_knee": 3, "R_ankle": 4,
    "L_hip_roll": 5, "L_hip_pitch": 6, "L_knee": 7, "L_ankle": 8,
    "R_hip_yaw": 9, "L_hip_yaw": 10, "R_ankle_roll": 11, "L_ankle_roll": 12,
}
JOINTS = [f"{s}_{j}" for s in "LR"
          for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "ankle_roll")]
CHANGED_BODIES = [f"{s}_{b}" for s in "LR" for b in ("ankle_blk", "foot")]


def _contact_pairs(spec):
    """v5body's foot pairs (sole-sole, sole-opposite shank, full-footprint
    box-box) plus the v6 ankle/foot boxes against the other leg's collision
    geoms. Pairs ignore contype/conaffinity, so the feet stay out of every
    automatic collision."""
    parts = ("col_thigh", "col_shank", "col_ankle", "col_foot")
    for a in parts:
        for b in parts:
            if "col_ankle" in (a, b) or "col_foot" in (a, b):
                spec.add_pair(geomname1=f"L_{a}", geomname2=f"R_{b}")
    spec.add_pair(geomname1="L_sole", geomname2="R_sole")
    spec.add_pair(geomname1="L_sole", geomname2="R_col_shank")
    spec.add_pair(geomname1="R_sole", geomname2="L_col_shank")


def build_spec() -> mujoco.MjSpec:
    s5 = mujoco.MjSpec.from_file(V5)
    ref = s5.actuator("L_knee")                    # v5body's STS3215 model
    gain, bias, frange = ref.gainprm.copy(), ref.biasprm.copy(), ref.forcerange.copy()

    for a in list(s5.actuators):
        if a.name.endswith("_ankle"):
            s5.delete(a)
    for p in list(s5.pairs):
        if any(k in p.geomname1 + p.geomname2 for k in ("sole", "col_foot")):
            s5.delete(p)
    for s in "LR":
        s5.delete(s5.body(f"{s}_foot"))
        s6 = mujoco.MjSpec.from_file(V6)           # fresh: assets cannot attach twice
        frame = s5.body(f"{s}_shin").add_frame()
        blk = frame.attach_body(s6.body(f"{s}_ankle_blk"), PREFIX[s], "")
        blk.pos = [0, 0, ANKLE_Z]                  # keeps its v6 offset otherwise

    for p in list(s5.pairs):                       # v6's own pairs came along
        if "v6" in p.geomname1 + p.geomname2:
            s5.delete(p)
    for kind in (s5.bodies, s5.joints, s5.geoms, s5.sites, s5.actuators):
        for e in kind:
            for pre in PREFIX.values():
                if e.name.startswith(pre):
                    e.name = e.name[len(pre):]
    for a in s5.actuators:
        for pre in PREFIX.values():
            if a.target.startswith(pre):
                a.target = a.target[len(pre):]
        if a.target.endswith("_ankle") or a.target.endswith("_ankle_roll"):
            a.gainprm, a.biasprm, a.forcerange = gain, bias, frange
    lim = math.radians(ANKLE_RANGE_DEG)
    for j in s5.joints:
        if j.name in ("L_ankle", "R_ankle"):
            j.range = [-ANKLE_RANGE_DEG, ANKLE_RANGE_DEG]
    for a in s5.actuators:
        if a.name in ("L_ankle", "R_ankle"):
            a.ctrlrange = [-lim, lim]
    _contact_pairs(s5)
    return s5


def stand_height(m: mujoco.MjModel, d: mujoco.MjData) -> float:
    """torso z that puts the lowest pad bottom on the floor at the zero pose"""
    d.qpos[:] = 0
    d.qpos[3] = 1
    d.qpos[2] = 1.0
    mujoco.mj_forward(m, d)
    low = min(d.geom_xpos[g][2] - m.geom_size[g][0]
              for g in range(m.ngeom) if m.geom_contype[g] == 1 and m.geom_type[g] == 2)
    return 1.0 - low


def build() -> mujoco.MjModel:
    return build_spec().compile()


def set_stand(m, d):
    d.qpos[:] = 0
    d.qvel[:] = 0
    d.qpos[3] = 1
    d.qpos[2] = stand_height(m, d)
    d.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)


def servo_joints(m):
    """the 12 actuated hinge joints, in JOINTS order"""
    return [m.joint(n).id for n in JOINTS]


def main():
    m = build()
    d = mujoco.MjData(m)
    set_stand(m, d)
    print(f"v5b plant: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.ngeom} geoms")
    print("\njoints (deg range, servo ID, actuator kp / forcerange):")
    for n in JOINTS:
        j = m.joint(n)
        a = m.actuator(n)
        lo, hi = np.degrees(m.jnt_range[j.id])
        print(f"  {n:13s} id {SERVO_ID[n]:2d}  [{lo:6.1f}, {hi:6.1f}]  axis {m.jnt_axis[j.id]}  "
              f"kp {m.actuator_gainprm[a.id][0]:g} kv {abs(m.actuator_biasprm[a.id][2]):g} "
              f"force +-{m.actuator_forcerange[a.id][1]:g}")
    print("\nactuators:", [m.actuator(i).name for i in range(m.nu)])
    print(f"\ntotal mass {m.body_subtreemass[0]:.4f} kg")
    print("changed bodies (mass g):",
          {n: round(1e3 * m.body_mass[m.body(n).id], 1) for n in CHANGED_BODIES})
    print("v5body feet were (g):", {"L_foot": 110.4, "R_foot": 110.4})
    com = d.subtree_com[0]
    print(f"\ntorso z at the zero pose {d.qpos[2]:.4f} m; CoM height {com[2]*1e3:.1f} mm, "
          f"CoM xy ({com[0]*1e3:.1f}, {com[1]*1e3:.1f}) mm")
    # settle on the soles: contacts of the pads only, feet flat
    d.qpos[2] += 0.0005
    mujoco.mj_forward(m, d)
    for _ in range(1000):
        d.ctrl[:] = 0
        mujoco.mj_step(m, d)
    pads = [g for g in range(m.ngeom) if m.geom_contype[g] == 1 and m.geom_type[g] == 2]
    touching = sorted({g for c in d.contact[:d.ncon] for g in (c.geom1, c.geom2) if g in pads})
    foot_up = [float(d.xmat[m.body(f"{s}_foot").id].reshape(3, 3)[2, 2]) for s in "LR"]
    print(f"after 2 s at zero command: torso z {d.qpos[2]*1e3:.1f} mm, tilt "
          f"{math.degrees(math.acos(d.xmat[m.body('torso').id].reshape(3,3)[2,2])):.2f} deg, "
          f"{len(touching)}/{len(pads)} sole pads in contact, foot-up z {foot_up}")


if __name__ == "__main__":
    main()
