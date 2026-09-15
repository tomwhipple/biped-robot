"""Get-up study, round 3 (Tom: "without the tail or arms -- remember the
robot can rotate most of its joints further than humans"). Bare v7 body,
legs only.

NEGATIVE RESULT, with the geometry measured rather than assumed. The round-2
doc says the seat-push fails because "the CoM stays 5-8 cm behind the feet";
this round asked whether a more creative sequence (IK-placed floor poses,
dynamic kicks, extreme ranges) moves the feet or the CoM enough to close
that gap. It does not, and the blocker is not *where* the search looked but
a length mismatch that no sequence of leg moves can change:

  seated, feet flat on the floor reachable:      none (0 of the swept grid:
      at the settled seat height 0.104 with the hips at full -125 flexion,
      NO (knee, ankle) pair puts a sole flat on the floor; the foot bottom
      only reaches the pad plane at knee ~-73 with the sole 90-100 deg off
      the floor (vertical, on the toe face))
  supine bridge (hips +40..+90):                 the feet touch the floor
      flat in exactly ONE pose (hip +40, knee -120, ankle -45) at pelvis
      z 0.176, and the head is at 0.18 touching too -- the classic bridge
      that the run shows slides the whole body headward (pelvis x +0.017 ->
      +0.258) instead of pivoting over the feet: the head is the fulcrum
      and it skids. Every pelvis-height drop from there goes airborne and
      collapses back to the lie.
  side-lying fold ("fetal"):                     legs folded toward the
      chest park the feet 34 cm above the floor -- the fold brings the feet
      up across the body, not down under it. No pose with the knee down and
      a foot within reach exists.
  sit-back to kneel ("seiza"):                   from the child's pose the
      sit-back rolls on the head (head 2-8 N throughout) and lands back in
      the lie; the kick that would carry the pelvis over the heels also
      pitches the torso onto the head (the head is always the first contact
      forward of the hip line).
  knee-lever rise from the tucked pike:          the lever arc exists (the
      130 knee slides the torso-CoM forward as the knee opens) UNTIL
      knee ~-70, where the feet leave the floor (at knee -125..-73 the foot
      bottom is 10-23 cm below the settled pelvis line; the soles only meet
      the floor at knee >= -70, already pointing up). The pike pivot
      happens with the shanks still pointing at the sky -- no lever arm
      under the pivot, momentum or a wall only.

The root cause in one line: **leg 0.335 m vs torso 0.55 m.** From any
grounded pose the feet are farther from the pelvis than the head/upper-torso
CoM is, so every rotation about a leg contact lands the head, and every
rotation about the head lands on the face. The appendage worked in round 2
because it was a contact 20 cm from the pelvis on the CoM side -- a lever
arm this body does not have between its feet and its head.

    .venv/bin/python sim/getup_v6_legs.py probe <bridge|pike>
    .venv/bin/python sim/getup_v6_legs.py seiza    (renders the seiza attempt)
    .venv/bin/python sim/getup_v6_legs.py search x supine
"""
from __future__ import annotations
import sys, os, math, itertools
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from gen_plant_v6 import DesignParams, build_xml  # noqa: E402
import v6_kin as VK  # noqa: E402
import getup_v6 as G  # noqa: E402

SP = os.environ.get("TMPDIR", "/tmp")
K, H = -130.0, -125.0
P = DesignParams(hip_roll_abd=55.0, hip_roll_add=45.0)   # chain-study ranges
XML = None

r = math.radians


def plant():
    global XML
    if XML is None:
        XML = os.path.join(SP, f"gu_legs_{os.getpid()}.xml")
        with open(XML, "w") as f:
            f.write(build_xml(P))
    return P, XML


def probe(frames, start="supine", label=""):
    """interpolate each keyframe (symmetric leg joints) from the settled
    fallen state under the plant's own position actuators; report pelvis,
    up-vector, CoM, per-foot sole margin and contacts at each hold. This is
    the honest quasi-static read of a candidate path (no servo lag)."""
    p, xml = plant()
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    if start == "supine":
        d.qpos[0:3] = [0, 0, 0.12]
        d.qpos[3:7] = [math.cos(math.pi / 4), 0, -math.sin(math.pi / 4), 0]
    else:
        d.qpos[0:3] = [0, 0, 0.12]
        d.qpos[3:7] = [math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0]
    d.qvel[:] = 0
    for _ in range(int(1.2 / 0.002)):
        for i in range(13):
            lo, hi = m.actuator_ctrlrange[i]
            d.ctrl[i] = np.clip(0.0, lo, hi)
        mujoco.mj_step(m, d)
    print(f"--- probe {label}  (settled: z {d.qpos[2]:.3f}, contacts {G.contacts_summary(m, d)})")
    q_prev = d.qpos[7:7 + 12].copy()
    for f in frames:
        name, q = f[0], f[1]
        move_s = f[2] if len(f) > 2 else 1.5
        hold_s = f[3] if len(f) > 3 else 0.8
        for i in range(int(move_s / 0.002) + int(hold_s / 0.002)):
            s = min(1.0, (i + 1) / max(int(move_s / 0.002), 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            qt = np.concatenate([q_prev + (q - q_prev) * s, [0.0]])
            for a in range(13):
                lo, hi = m.actuator_ctrlrange[a]
                d.ctrl[a] = np.clip(qt[a], lo, hi)
            mujoco.mj_step(m, d)
        q_prev = d.qpos[7:7 + 12].copy()
        com = d.subtree_com[0].copy()
        contacts = G.contacts_summary(m, d)
        up = d.xmat[m.body("torso").id].reshape(3, 3)[2, 2]
        marg = {}
        for side in "LR":
            c_foot = VK.in_foot(m, d, side, com)
            pos_f, _ = VK.foot_frame(m, d, side)
            grounded = (pos_f[2] - P.roll_h) < 0.03
            marg[side] = (VK.sole_margin(P, c_foot[:2], side) if grounded else None)
        ms = " ".join(f"{s}:{(f'{marg[s]*1e3:+.0f}' if marg[s] is not None else ' air')}" for s in "LR")
        print(f"{name:30s} z {d.qpos[2]:.3f} up {up:+.2f}  CoM ({com[0]:+.3f},{com[1]:+.3f},{com[2]:.3f})  ft[{ms}]  {contacts}")


def q_sym(hp, k, a):
    out = np.zeros(12)
    for i in range(2):
        out[6 * i + 2], out[6 * i + 3], out[6 * i + 4] = r(hp), r(k), r(a)
    return out


def fam_bridge(hp_in=-60, k_in=-100, a_in=-20, hp_up=30, k_up=-95, a_up=-35,
               hp_rock=60, k_rock=-110, a_rock=-45, t_rock=1.5,
               rise=((40, -90, -45), (0, -60, -45), (-20, -40, -20), (0, 0, 0))):
    """supine bridge: feet pull in, hips extend (pelvis up on feet + head),
    rock forward over the feet, unfold. Renders to bridge_v1.mp4: the bridge
    lifts (pelvis 0.125) then the head-fulcrum skids and it collapses."""
    frames = [("feet in", q_sym(hp_in, k_in, a_in), 1.5, 0.8),
              ("bridge up", q_sym(hp_up, k_up, a_up), 1.5, 0.8),
              ("rock over feet", q_sym(hp_rock, k_rock, a_rock), t_rock, 1.0)]
    for j, (hp, k, a) in enumerate(rise):
        frames.append((f"rise {j+1}", q_sym(hp, k, a), 1.5, 0.8))
    return frames


def fam_pike():
    """sit-up route with a deep knee-lever rise: the 130 knee slides the
    torso-CoM forward as it opens -- until the feet go airborne at ~-70."""
    seq_ = [("sit up", q_sym(-90, 0, 0), 1.5, 0.8),
            ("tuck", q_sym(H, K, 10), 1.5, 0.8),
            ("ankle lever to pike", q_sym(H, K, -45), 2.0, 1.0)]
    for j, (hp, k, a) in enumerate([(-125, -115, -30), (-125, -95, -25), (-125, -70, -20),
                                    (-125, -45, -15), (-90, -35, -12), (-50, -20, -8),
                                    (-20, -12, -8), (0, 0, 0)]):
        seq_.append((f"knee-lever {j+1}", q_sym(hp, k, a), 1.2, 0.5))
    return seq_


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    fam = sys.argv[2] if len(sys.argv) > 2 else "bridge"
    if mode == "probe":
        probe(fam_bridge() if fam == "bridge" else fam_pike(), label=fam)
    elif mode == "seiza":
        kk, hh = K, H
        seq = [
            ("lie", dict(), 0.5, 1.0),
            ("child's pose", dict(hip_pitch=-80, knee=kk, ankle=40), 2.0, 1.0),
            ("rock heads-up", dict(hip_pitch=-60, knee=kk, ankle=45), 1.5, 0.8),
            ("kick + sit back", dict(hip_pitch=-10, knee=kk, ankle=40), 0.8, 0.5),
            ("kneel-sit", dict(hip_pitch=0, knee=kk, ankle=40), 1.5, 1.5),
            ("toe tuck", dict(hip_pitch=0, knee=kk, ankle=-45), 1.5, 1.0),
            ("kneel up", dict(hip_pitch=-70, knee=-125, ankle=-45), 2.0, 1.0),
            ("kneel up 2", dict(hip_pitch=-30, knee=-110, ankle=-45), 2.0, 1.0),
            ("half-kneel (L fwd)", dict(L_hip_pitch=-60, L_knee=-90, L_ankle=-20,
                                       R_hip_pitch=-20, R_knee=-125, R_ankle=-45), 1.5, 1.0),
            ("L foot to the fore-side", dict(L_hip_pitch=-90, L_knee=-110, L_ankle=-20,
                                             R_hip_pitch=-40, R_knee=-125, R_ankle=-45), 1.5, 1.0),
            ("both feet pike", dict(hip_pitch=H, knee=-120, ankle=-40), 1.5, 1.0),
            ("knee-lever 1", dict(hip_pitch=H, knee=-95, ankle=-30), 1.2, 0.5),
            ("knee-lever 2", dict(hip_pitch=H, knee=-70, ankle=-20), 1.2, 0.5),
            ("knee-lever 3", dict(hip_pitch=-100, knee=-45, ankle=-15), 1.2, 0.5),
            ("knee-lever 4", dict(hip_pitch=-60, knee=-25, ankle=-10), 1.2, 0.5),
            ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 1.5, 1.0),
            ("straight", dict(), 1.0, 0.8),
        ]
        p, xml = plant()
        r_ = G.run_sequence(p, xml, seq, start="supine", play_deg=3.0, per_joint=G.PJ_DEFAULT,
                            verbose=True, render="sim/renders/getup_legs/seiza.mp4", cam=(1.4, -14, 90))
        print("STANDING" if r_["ok"] else "not standing")
    elif mode == "search":
        # continuous-space keyframe search (symmetric hip/knee/ankle, 6 nodes,
        # hill-climb with restarts) under the deploy servo model. Best found
        # (supine): kneel-up dead-end, z 0.125 / up 1.0 (the search's +85 hip-
        # extension keyframes are it discovering the bridge -- then collapsing).
        import random
        random.seed(0); np.random.seed(0)

        def eval_path(V, start):
            seq = [(f"k{j}", {n: math.degrees(q_sym(*row)[i]) for i, n in enumerate(G.JN)}, 1.4, 0.6)
                   for j, row in enumerate(V)]
            p, xml = plant()
            res = G.run_sequence(p, xml, seq, start=start, play_deg=3.0, per_joint=G.PJ_DEFAULT, verbose=False)
            prog = sum(L[2] for L in res["log"]) / len(res["log"])
            return 3.0 * max(0, res["up"]) + 8.0 * res["pelvis_z"] + 0.5 * prog, res

        which = fam if fam in ("supine", "prone") else (sys.argv[3] if len(sys.argv) > 3 else "supine")
        NK, bounds = 6, [(-125, 90), (-130, 5), (-45, 45)]
        best = (-1e9, None, None)
        for restart in range(8):
            x = np.array([[random.uniform(*b) for b in bounds] for _ in range(NK)])
            s_best, step = -1e9, [30.0, 25.0, 12.0]
            for it in range(40):
                for _ in range(6):
                    y = x + np.array([[random.gauss(0, st) for st in step] for _ in range(NK)])
                    for j, b in enumerate(bounds):
                        y[:, j] = np.clip(y[:, j], *b)
                    s, res = eval_path(y, which)
                    if s > s_best:
                        s_best, x = s, y
                step = [st * 0.93 for st in step]
            s, res = eval_path(x, which)
            print(f"restart {restart}: score {s:.2f} up {res['up']:+.2f} z {res['pelvis_z']:.3f}  "
                  f"path {' '.join(f'({v[0]:+.0f},{v[1]:+.0f},{v[2]:+.0f})' for v in x)}", flush=True)
            if s > best[0]:
                best = (s, x.copy(), res)
        print(f"BEST score {best[0]:.2f} up {best[2]['up']:+.2f} z {best[2]['pelvis_z']:.3f}")


if __name__ == "__main__":
    main()
