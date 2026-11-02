"""The robot's `stepresp` in sim: standing at the calibrated zero in double support, one joint
+30 ticks (2.64 deg) for 300 ms then back, the rest held; dead time / t10 / t90 / final."""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rec, simh

STEP = 30 * rec.RAD
STEP_JOINTS = ["R_hip_roll", "R_ankle_roll", "R_hip_pitch", "L_knee", "R_ankle"]
# the robot, 2026-11-01 loaded stepresp (mean of out/back, both sides): t10, t90 ms, final fraction
TARGET = {"R_hip_roll": (15, 58, 0.80), "R_ankle_roll": (77, 165, 0.80), "R_hip_pitch": (70, 157, 0.80),
          "L_knee": (77, 195, 0.85), "R_ankle": (82, 235, 0.95)}


def run(joint, servo_kw=None, model=None, plant="fit", trace=False):
    env, act, zoff = simh.build(plant, ep_s=6.0)
    env.domain_rand = False; env.push_prob = 0.0
    env.reset(seed=0)
    if plant.startswith("fit"):
        env._zero_off = zoff.copy()
    if model:
        simh.apply_model(env, model)
    q0 = np.zeros(12)
    simh.place_and_hold(env, q0, 1.5)
    sv = simh.TypedServo(**(servo_kw or {})); sv.reset(q0)
    j = rec.POL.index(joint)
    cur = {"g": q0}
    env._action_to_ctrl = lambda a: cur["g"]
    # substep-resolution record of the joint
    out = []
    for k in range(30):
        g = q0.copy()
        if k < 15:
            g[j] += STEP
        cur["g"] = sv(g)
        for s in range(env.n_substeps):
            pass
        env.step(np.zeros(12, np.float32))
        d = env.data
        out.append(float(d.qpos[env._jqpos][j] - (env._zero_off[j] if env._zero_off is not None else 0)))
    x = np.array(out) / STEP
    t = (np.arange(30) + 1) * 20.0
    def stats(xx, tt):
        fin = np.mean(xx[-3:])
        mv = np.where(xx >= 2 / 30)[0]
        f = lambda c: float(np.interp(c * fin, xx[:np.argmax(xx) + 1], tt[:np.argmax(xx) + 1])) if xx.max() > c * fin else float("nan")
        return dict(dead_ms=float(tt[mv[0]]) if len(mv) else float("nan"), t10_ms=f(0.1), t90_ms=f(0.9), final=float(fin))
    st = stats(x[:15], t[:15])
    if trace:
        st["t"] = t.tolist(); st["x"] = x.tolist()
    return st


if __name__ == "__main__":
    kw = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    model = json.loads(sys.argv[2]) if len(sys.argv) > 2 else None
    for jn in STEP_JOINTS:
        print(jn, run(jn, kw, model))
