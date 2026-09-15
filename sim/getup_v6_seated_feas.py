"""Seated-on-skid feasibility: when the robot rests hip-down on the passive
skid, are the hips high enough (skid_h) that a deep-flex crouch puts the CoM
over the feet? Place the torso at the skid-contact height (hips = skid height
above floor) in the fully-tucked pose and check CoM-x vs foot span. This is the
true entry to the get-up -- the record's flex130 says hips at 4cm are too low;
a skid raises them.
"""
import sys, os, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); os.environ.setdefault("MUJOCO_GL", "egl")
import dataclasses as dc
import numpy as np
from gen_plant_v6 import DesignParams, build_xml
import getup_v6 as G
from static_gait import make_env
import mujoco

def qq(off):
    q = np.zeros(12)
    for k, v in off.items():
        if k in G.J: q[G.J[k]] = math.radians(v)
        else:
            for s in "LR": q[G.J[f"{s}_{k}"]] = math.radians(v)
    return q

def foot_x(env):
    m, d = env.model, env.data
    xs = []
    for s in "LR":
        for g in range(m.ngeom):
            n = m.geom(g).name or ""
            if n.startswith(f"{s}_pad_"):
                xs.append(d.geom_xpos[g][0])
    return min(xs), max(xs)

# seated on the skid at hip height hip_z = skid_h, deep-flexed, feet tucked under
for skid_h in (0.04, 0.06, 0.08, 0.10, 0.12, 0.15):
    p = dc.replace(DesignParams(), skid=True, skid_h=skid_h, skid_x=-0.02, skid_len=0.08)
    xml = f"/tmp/v6_seat_{int(skid_h*100)}.xml"; open(xml, "w").write(build_xml(p))
    env = make_env(p, xml); obs, _ = env.reset(seed=0)
    m, d = env.model, env.data
    # deep-flex tuck: feet pulled under, torso folded to hip -125
    for hip_flex in (-125, -100, -90, -80):
        q = qq(dict(hip_pitch=hip_flex, knee=-130, ankle=-20))
        d.qpos[:] = 0
        d.qpos[7:19] = q
        d.qpos[2] = skid_h + 0.03   # torso origin just above the seat contact
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        com = d.subtree_com[0]
        heel, toe = foot_x(env)
        inside = heel <= com[0] <= toe
        print(f"  skid_h={skid_h:.2f} hip_flex={hip_flex:+.0f}  CoM={com[0]:+.3f} "
              f"heel={heel:+.3f} toe={toe:+.3f} -> {'OVER FEET' if inside else 'behind heel'} (margin {1000*(com[0]-heel):+.0f} mm)")
    print()
