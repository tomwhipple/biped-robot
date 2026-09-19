"""Pelvis-skid get-up study (round 3b: Tom: "design and sim-verify the pelvis
skid concept"). The skid is a passive curved sole under/behind the pelvis
(gen_plant_v6.DesignParams.skid=True) that the seated body rests on instead
of the thigh tops. It has no servo, no firmware change -- a printed bumper.

The mechanism the bare body was missing: with the seat carried on the skid,
the torso stays pitched FORWARD over the thighs (hip at the -125 stop) as the
legs sweep under it, so the whole-robot CoM rides ahead of the pelvis near
the feet -- as measured, +3 mm fwd of the ankle at 18 deg of forward pitch,
vs -49/-80 mm sitting vertical. The sequence below is:

  lie supine -> sit up (legs straight) -> tuck feet (deep), pelvis lands on
  the skid -> walk the feet BACK under the braced torso (ankle + hip, the
  legs reposition while the skid carries the load) -> unfold the knees with
  the torso forward, rolling up over the skid's curved sole onto the feet ->
  crouch hold -> stand.

Run:
    MUJOCO_GL=disable .venv/bin/python sim/getup_v6_skid.py probe
    MUJOCO_GL=glfw    .venv/bin/python sim/getup_v6_skid.py run [mp4]
"""
from __future__ import annotations
import sys, os, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "disable")

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import mujoco  # noqa: E402
import v6_kin as VK  # noqa: E402
import getup_v6 as G  # noqa: E402

SP = os.environ.get("TMPDIR", "/tmp")
P = DesignParams(skid=True, skid_bot=-0.04, skid_len=0.11, skid_mass=0.030,
                 hip_roll_abd=55.0, hip_roll_add=45.0)
XML = None


def plant():
    global XML
    if XML is None:
        XML = os.path.join(SP, f"gu_skid_{os.getpid()}.xml")
        with open(XML, "w") as f:
            f.write(build_xml(P))
    return P, XML


K, H = -130.0, -125.0


def seq_skid():
    """(label, joint offsets, move_s, hold_s). The mechanism (measured in the
    probe): the skid carries the pelvis so the torso sits pitched ~40 deg
    forward over the thigh fronts; at knee ~-82, ankle ~+15 the feet plant
    flat at x~0 with the CoM 20-50 mm INSIDE the sole -- the only grounded
    stable tuck this body has. From there the knees open as an ankle-led roll
    over the toes, onto the feet, skid leaves the floor, rise, stand."""
    return [
        ("lie", dict(), 0.5, 1.0),
        ("sit up (legs straight)", dict(hip_pitch=-90), 1.5, 0.8),
        ("tuck onto the skid", dict(hip_pitch=H, knee=-90, ankle=15), 2.0, 1.2),
        ("feet under (shins fwd)", dict(hip_pitch=H, knee=-82, ankle=15), 1.5, 1.5),
        ("lean over the feet", dict(hip_pitch=H, knee=-82, ankle=10), 1.0, 1.5),
        # the rise: knees open, keeping the sole planted (the pelvis rolls fwd
        # on the skid, then the skid leaves). Each step keeps a ~flat sole.
        ("rise 1", dict(hip_pitch=-120, knee=-75, ankle=15), 1.5, 0.8),
        ("rise 2", dict(hip_pitch=-112, knee=-65, ankle=12), 1.5, 0.8),
        ("rise 3", dict(hip_pitch=-105, knee=-55, ankle=8), 1.5, 0.8),
        ("rise 4", dict(hip_pitch=-95, knee=-45, ankle=2), 1.5, 0.8),
        ("rise 5", dict(hip_pitch=-80, knee=-50, ankle=-15), 1.5, 0.8),
        ("crouch hold", dict(hip_pitch=-45, knee=-50, ankle=-25), 1.2, 2.0),
        ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.5, 0.8),
        ("straight", dict(), 1.5, 1.0),
    ]


def probe():
    """quasi-static: settle fallen, interpolate each keyframe under the plant's
    own position actuators, report pelvis / up / CoM / foot margins / contacts."""
    p, xml = plant()
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    d.qpos[0:3] = [0, 0, 0.12]
    d.qpos[3:7] = [math.cos(math.pi / 4), 0, -math.sin(math.pi / 4), 0]
    d.qvel[:] = 0
    for _ in range(int(1.2 / 0.002)):
        for i in range(m.nu):
            lo, hi = m.actuator_ctrlrange[i]
            d.ctrl[i] = np.clip(0.0, lo, hi)
        mujoco.mj_step(m, d)
    print(f"--- skid probe (settled z {d.qpos[2]:.3f}, contacts {G.contacts_summary(m, d)})")
    q_prev = d.qpos[7:7 + 12].copy()
    for label, off, mv, hd in seq_skid():
        q = G.q_from_offsets(off)[:12]
        tgt = np.concatenate([q, np.zeros(m.nu - 12)])
        for i in range(int(mv / 0.002) + int(hd / 0.002)):
            s = min(1.0, (i + 1) / max(int(mv / 0.002), 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            qt = np.concatenate([q_prev + (q - q_prev) * s, [0.0]])
            for a in range(m.nu):
                lo, hi = m.actuator_ctrlrange[a]
                v = qt[a] if a < len(qt) else 0.0
                d.ctrl[a] = np.clip(v, lo, hi)
            mujoco.mj_step(m, d)
        q_prev = d.qpos[7:7 + 12].copy()
        com = d.subtree_com[0].copy()
        up = d.xmat[m.body("torso").id].reshape(3, 3)[2, 2]
        marg = {}
        for side in "LR":
            c_foot = VK.in_foot(m, d, side, com)
            pos_f, _ = VK.foot_frame(m, d, side)
            grounded = (pos_f[2] - P.roll_h) < 0.04
            marg[side] = (VK.sole_margin(P, c_foot[:2], side) if grounded else None)
        ms = " ".join(f"{s}:{(f'{marg[s]*1e3:+.0f}' if marg[s] is not None else ' air')}" for s in "LR")
        print(f"{label:28s} z {d.qpos[2]:.3f} up {up:+.2f} CoM x {com[0]:+.3f}  ft[{ms}]  {G.contacts_summary(m, d)}")


def run(render=None):
    p, xml = plant()
    r = G.run_sequence(p, xml, seq_skid(), start="supine", play_deg=3.0,
                       per_joint=G.PJ_DEFAULT, verbose=True, render=render,
                       cam=(1.4, -14, 90))
    print("=> STANDING" if r["ok"] else "=> not standing",
          f"up {r['up']:+.2f} z {r['pelvis_z']:.3f}")
    return r


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if mode == "probe":
        probe()
    else:
        run(sys.argv[2] if len(sys.argv) > 2 else None)
