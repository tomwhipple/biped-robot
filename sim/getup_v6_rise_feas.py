"""Static rise-path feasibility: sweep the butt-up -> stand keyframes (knees
extend while hips stay folded, torso unfolds last -- getup_study.py §95) and
check the whole-robot CoM stays over the foot support polygon at every step.
If YES, a no-appendage stand exists on the skid body (the passive pelvis skid
raises the seat; the deep ROM + butt-up carry the CoM out of the corridor).

Each candidate pose is placed by mj_forward with the freejoint height derived
from the feet resting on the floor, so the CoM measure is physically consistent
(no hand-set torso height). Uses the skid body only (the added lever).

Run: .venv/bin/python sim/getup_v6_rise_feas.py
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

def place_and_check(env, off, label):
    """Set joint angles, place the torso so the feet rest on the floor (scan the
    freejoint z until the lowest sole is just above the floor), then report the
    CoM-x vs the foot span."""
    m, d = env.model, env.data
    q = qq(off)
    d.qpos[:] = 0
    d.qpos[7:19] = q
    d.qvel[:] = 0
    # scan root height from 0 upward until the lowest sole clears the floor
    for zroot in np.linspace(0.0, 0.45, 90):
        d.qpos[2] = zroot
        mujoco.mj_forward(m, d)
        soles = [d.geom_xpos[g][2] for g in range(m.ngeom)
                 if (m.geom(g).name or "").find("_pad_") > 0]  # pad CENTRE z
        if min(soles) > 0.002:  # just above the floor (pad centre ~ r above bottom)
            break
    com = d.subtree_com[0]
    heel, toe = foot_x(env)
    inside = heel <= com[0] <= toe
    margin = 1000.0 * (com[0] - heel)
    print(f"  {label:34s} z_hip={d.qpos[2]:.3f} CoM={com[0]:+.3f} "
          f"heel={heel:+.3f} toe={toe:+.3f} -> "
          f"{'OVER FEET' if inside else 'BEHIND HEEL' } (margin {margin:+.0f} mm)")
    return inside

def main():
    # skid body: passive pelvis seat at a moderate height (belly flush + pad)
    p = dc.replace(DesignParams(), skid=True, skid_h=0.10, skid_x=-0.02, skid_len=0.08)
    xml = "/tmp/v6_rise_feas.xml"; open(xml, "w").write(build_xml(p))
    env = make_env(p, xml); obs, _ = env.reset(seed=0)

    # The deep crouch -> stand keyframes (butt-up: knees extend, hips stay
    # folded, then torso unfolds). (hip, knee, ankle) in sim degrees.
    poses = [
        ("deep crouch (loaded)", dict(hip_pitch=-125, knee=-130, ankle=-20)),
        ("buttup knee-100",      dict(hip_pitch=-125, knee=-100, ankle=-20)),
        ("buttup knee-70",       dict(hip_pitch=-125, knee=-70, ankle=-20)),
        ("rise hip-105/knee-60", dict(hip_pitch=-105, knee=-60, ankle=-22)),
        ("rise hip-85/knee-50",  dict(hip_pitch=-85, knee=-50, ankle=-22)),
        ("rise hip-65/knee-45",  dict(hip_pitch=-65, knee=-45, ankle=-20)),
        ("rise hip-45/knee-42",  dict(hip_pitch=-45, knee=-42, ankle=-20)),
        ("rise hip-25/knee-40",  dict(hip_pitch=-25, knee=-40, ankle=-20)),
        ("stand",                 dict(hip_pitch=-20, knee=-40, ankle=-20)),
    ]
    ok = sum(place_and_check(env, pf, lab) for lab, pf in poses)
    print(f"\n{ok}/{len(poses)} rise keyframes have CoM over the feet -> "
          f"{'a static no-appendage rise exists' if ok == len(poses) else 'not a full static path'}")

main()
