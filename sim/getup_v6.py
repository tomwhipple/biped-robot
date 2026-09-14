"""Open-loop fall recovery on the v6 body: joint-space keyframe sequences run
under the deploy servo model from a settled fallen state (supine or prone),
with the whole body colliding with the floor (gen_plant_v6 fall_collision).

Design question, not controller question: which recovery paths does this
body (no arms, hip flexion -110, knee -95, ankle +-40, 75 mm toe, a head and
a boxy torso) admit quasi-statically, and what would it need. Every sequence
is a list of (label, joint offsets deg, move_s, hold_s); symmetric unless a
side is named. Success = standing with the torso up-vector z > 0.9 and the
pelvis at >= 80 % of the standing height at the end.

    .venv/bin/python sim/getup_v6.py --list
    .venv/bin/python sim/getup_v6.py --seq supine_situp --render sim/renders/v6_getup_supine.mp4
    .venv/bin/python sim/getup_v6.py --seq prone_roll --play 3
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from gen_plant_v6 import DesignParams, params_from_args, build_xml  # noqa: E402
from design_gates import SERVOS, JN  # noqa: E402
from static_gait import make_env, ROLLS, _write_video  # noqa: E402

J = {n: i for i, n in enumerate(JN)}
PJ_DEFAULT = {j: "sts3250" for j in ROLLS + ("L_knee", "R_knee")}


EXTRA = ["neck_yaw", "L_shoulder", "R_shoulder"]   # actuator order after the 12 leg joints;
# run_sequence overwrites this IN PLACE from the plant (neck, shoulders/elbows, tail)


def q_from_offsets(off: dict) -> np.ndarray:
    """offsets in DEG keyed by joint name, or by role for both legs
    ('hip_pitch' -> L and R; 'shoulder' -> both arms). Returns 12 + 3 values;
    run_sequence trims to the plant's actuator count."""
    q = np.zeros(12 + len(EXTRA))
    for k, v in off.items():
        if k in J:
            q[J[k]] = math.radians(v)
        elif k in EXTRA:
            q[12 + EXTRA.index(k)] = math.radians(v)
        elif k == "tail" and "tail_pitch" in EXTRA:
            q[12 + EXTRA.index("tail_pitch")] = math.radians(v)
        elif f"L_{k}" in EXTRA:                     # 'shoulder', 'elbow' -> both arms
            q[12 + EXTRA.index(f"L_{k}")] = q[12 + EXTRA.index(f"R_{k}")] = math.radians(v)
        else:
            for s in "LR":
                q[J[f"{s}_{k}"]] = math.radians(v)
    return q


# --------------------------------------------------------------------------- sequences
# (label, offsets deg, move_s, hold_s). Joint sign conventions (v6_kin):
#   hip_pitch -  = thigh forward (flexion), knee - = flexion, ankle + = toe down,
#   hip_roll + on L = abduction, ankle_roll: opposite sign of hip_roll keeps the sole level
SEQUENCES = {
    # supine (on the back): sit up with straight legs (legs are the counter-
    # weight), tuck the feet, fold the torso over the knees, rock onto the
    # feet, stand.
    "supine_situp": [
        ("lie", dict(), 0.5, 1.0),
        ("sit up (hip flex, legs straight)", dict(hip_pitch=-90), 1.5, 1.0),
        ("tuck feet (knees)", dict(hip_pitch=-110, knee=-95, ankle=40), 1.5, 1.0),
        ("fold torso over the knees", dict(hip_pitch=-110, knee=-95, ankle=40), 0.5, 0.5),
        ("rock onto the feet (ankle dorsiflex)", dict(hip_pitch=-110, knee=-95, ankle=-10), 1.5, 1.0),
        ("half rise", dict(hip_pitch=-70, knee=-80, ankle=-25), 1.5, 1.0),
        ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 2.0, 1.5),
        ("straight", dict(), 1.5, 1.0),
    ],
    # supine, tuck FIRST then sit up (feet hooked under the body's momentum)
    "supine_tuck_first": [
        ("lie", dict(), 0.5, 1.0),
        ("tuck", dict(hip_pitch=-110, knee=-95, ankle=-40), 1.5, 1.0),
        ("kick legs down + sit", dict(hip_pitch=-60, knee=-95, ankle=-40), 0.6, 0.5),
        ("fold forward", dict(hip_pitch=-110, knee=-95, ankle=-40), 1.0, 1.0),
        ("rise", dict(hip_pitch=-70, knee=-80, ankle=-25), 1.5, 1.0),
        ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 2.0, 1.5),
        ("straight", dict(), 1.5, 1.0),
    ],
    # prone (face down): roll over to supine with an asymmetric leg swing, then supine_situp
    "prone_roll": [
        ("lie prone", dict(), 0.5, 1.0),
        ("L leg swing across (roll)", dict(L_hip_roll=45, L_hip_pitch=-60, L_knee=-60, R_hip_roll=-20), 1.0, 1.0),
        ("legs back", dict(), 1.5, 1.0),
    ],
    # prone: tuck knees under (child's pose), then push the pelvis up and back
    "prone_kneel": [
        ("lie prone", dict(), 0.5, 1.0),
        ("tuck knees under", dict(hip_pitch=-110, knee=-95, ankle=40), 2.0, 1.0),
        ("push pelvis up (knees extend)", dict(hip_pitch=-110, knee=-40, ankle=-40), 2.0, 1.5),
        ("hinge up", dict(hip_pitch=-60, knee=-40, ankle=-25), 2.0, 1.5),
        ("stand", dict(hip_pitch=-20, knee=-40, ankle=-20), 2.0, 1.5),
        ("straight", dict(), 1.5, 1.0),
    ],
}


# --------------------------------------------------------------------------- run
def settle_fallen(env, p, how: str, q_hold: np.ndarray, seconds=1.5):
    """drop the robot lying on its back ('supine') or front ('prone') with
    the joints held at q_hold, let it settle."""
    d = env.data
    m = env.model
    mujoco.mj_resetData(m, d)
    if how == "supine":
        quat = [math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0]   # rotate -90 about y: +x (front) up
        z = 0.12
    elif how == "prone":
        quat = [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0]    # +x down
        z = 0.12
    else:
        raise ValueError(how)
    d.qpos[:] = 0
    d.qpos[0:3] = [0.0, 0.0, z]
    d.qpos[3:7] = quat
    d.qpos[7:7 + len(q_hold[:env._nq_act])] = q_hold[:env._nq_act]
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    d0, hi, lo = env._default, env._hi, env._lo
    inv = lambda q: np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    for _ in range(int(seconds / env.control_dt)):
        qq = q_hold[:env._nq_act] if len(q_hold) >= env._nq_act else np.concatenate([q_hold, np.zeros(env._nq_act - len(q_hold))])
        env.step(inv(qq))


def _caption(frame, text):
    """burn a one-line caption into the top-left of an RGB frame (PIL if present)."""
    try:
        from PIL import Image, ImageDraw
        im = Image.fromarray(frame)
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, im.width, 22], fill=(0, 0, 0))
        dr.text((6, 4), text, fill=(255, 255, 255))
        return np.asarray(im)
    except Exception:  # noqa: BLE001
        return frame


def contacts_summary(m, d):
    names = set()
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        for g in (g1, g2):
            if g == m.geom("floor").id:
                continue
            b = m.body(m.geom_bodyid[g]).name
            names.add(b)
    return sorted(names)


def run_sequence(p: DesignParams, xml_path, seq, start="supine", play_deg=3.0, per_joint=None, mu=0.7,
                 render=None, verbose=True, servo_scale=1.0, cam=(1.3, -15, 135), size=(480, 640), label_fn=None):
    """cam = (distance, elevation, azimuth): 135 = rear three-quarter, 90 = side on;
    label_fn(step_label, t) -> text burned into the frames (None = no caption)."""
    env = make_env(p, xml_path, mu=mu, play_deg=play_deg)
    obs, _ = env.reset(seed=0)
    EXTRA[:] = [env.model.actuator(i).name for i in range(12, env.model.nu)]
    kp, kd, stall, w0 = env._servo
    na = env._nq_act
    stall = np.full(na, stall) * servo_scale
    w0 = np.full(na, w0) * servo_scale
    kpa = np.full(na, float(kp))
    kda = np.full(na, float(kd))
    for i, n in enumerate(JN):
        if per_joint and n in per_joint:
            s = SERVOS[per_joint[n]]
            stall[i] = s["stall"] * servo_scale
            w0[i] = s["w0"] * servo_scale
            kpa[i] *= s["kp_scale"]
            kda[i] *= s["kp_scale"]
    env._servo = np.array([kpa, kda, stall, w0], dtype=object)
    env.fall_up_z = -2.0          # never terminate on attitude: we are on the floor on purpose
    env.fall_height = -1.0
    m, d = env.model, env.data
    d0, hi, lo = env._default, env._hi, env._lo

    def inv(q):
        q = q[:na] if len(q) >= na else np.concatenate([q, np.zeros(na - len(q))])
        return np.clip(np.where(q >= d0, (q - d0) / np.maximum(hi - d0, 1e-6), (q - d0) / np.maximum(d0 - lo, 1e-6)), -1, 1)
    q_prev = q_from_offsets(seq[0][1])
    settle_fallen(env, p, start, q_prev)
    dt = env.control_dt
    frames = []
    if render:
        rnd = mujoco.Renderer(m, size[0], size[1])
        _cam = cam
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = _cam
    z_stand = p.z_yaw_above_sole
    log = []
    t = 0.0
    k = 0
    for label, off, move_s, hold_s in seq:
        q_tgt = q_from_offsets(off)
        n = int(move_s / dt)
        for i in range(n + int(hold_s / dt)):
            s = min(1.0, (i + 1) / max(n, 1))
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = q_prev + (q_tgt - q_prev) * s
            env.step(inv(q))
            t += dt
            k += 1
            if render and k % 2 == 0:
                cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.2]
                rnd.update_scene(d, cam)
                fr = rnd.render().copy()
                if label_fn is not None:
                    fr = _caption(fr, label_fn(label, t))
                frames.append(fr)
        q_prev = q_tgt
        up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
        front = d.xmat[env._torso_bid].reshape(3, 3)[2, 0]     # world z of the torso +x: prone -1, supine +1
        pz = float(d.qpos[2])
        tau = env._servo_tau
        names = list(JN) + EXTRA
        log.append((label, up, pz, contacts_summary(m, d), float(np.abs(tau).max()), names[int(np.abs(tau).argmax())], front))
        if verbose:
            print(f"  {t:5.1f}s {label:38s} up_z {up:+.2f} front_z {front:+.2f}  pelvis z {pz:.3f}  |tau|max {log[-1][4]:.2f} ({log[-1][5]})  contacts {log[-1][3]}")
    up = d.xmat[env._torso_bid].reshape(3, 3)[2, 2]
    front = d.xmat[env._torso_bid].reshape(3, 3)[2, 0]
    ok = up > 0.9 and d.qpos[2] > 0.8 * z_stand
    if render:
        _write_video(frames, render, fps=25)
    return dict(ok=ok, up=up, front=float(front), pelvis_z=float(d.qpos[2]), log=log)


def main(argv=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--seq", default="supine_situp")
    ap.add_argument("--start", default=None, help="supine|prone (default from the sequence name)")
    ap.add_argument("--play", type=float, default=3.0)
    ap.add_argument("--mu", type=float, default=0.7)
    ap.add_argument("--render", default=None)
    ap.add_argument("--list", action="store_true")
    a, rest = ap.parse_known_args(argv)
    if a.list:
        for k, v in SEQUENCES.items():
            print(k, "->", [s[0] for s in v])
        return
    p, _ = params_from_args(rest + ["-o", "/dev/null"])
    xml = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"v6getup_{os.getpid()}.xml")
    open(xml, "w").write(build_xml(p))
    start = a.start or ("prone" if a.seq.startswith("prone") else "supine")
    print(f"== {a.seq} from {start}, play {a.play}, mu {a.mu}, STS3250 rolls+knees")
    r = run_sequence(p, xml, SEQUENCES[a.seq], start=start, play_deg=a.play, per_joint=PJ_DEFAULT, mu=a.mu, render=a.render)
    print(f"=> {'STANDING' if r['ok'] else 'not standing'}: up_z {r['up']:+.2f}, pelvis z {r['pelvis_z']:.3f} (standing {p.z_yaw_above_sole:.3f})")


if __name__ == "__main__":
    main()
