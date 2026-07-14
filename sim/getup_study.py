"""Hip-pitch feasibility study: what flexion range admits a sit -> stand?

The mjx_getup_v1 experiment proved this plant reliably reaches a SIT from
any fall but cannot finish (hip pitch capped at 60 deg -- the torso cannot
fold over the knees to bring the CoM over the feet). Before changing CAD,
this script finds the MINIMUM hip-pitch flexion that makes the transfer
work, by executing a hand-authored quasi-static get-up motion at several
candidate limits (joint ranges patched at runtime; the honest STS3215
actuator model and the GoPro payload stay on).

Motion plan (symmetric, times scale with --slow):
  sit   : reclined torso, legs forward (the pose the RL policy parks in)
  tuck  : knees to full flexion, ankles dorsiflexed -- heels come under
  fold  : hips to max flexion -- torso folds over the knees, CoM forward
  plant : nose-down weight transfer onto the soles
  rise  : hips/knees/ankles extend together to stand

Run:  .venv/bin/python sim/getup_study.py [--hips 60,75,90,105,120,135]
Outputs per-H verdicts + filmstrips under sim/runs/getup_study/.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mujoco
from walker_env import BimoWalkerEnv

OUT = os.path.join(HERE, "runs", "getup_study")
XML = os.path.join(HERE, "bimo_biped_v2.xml")

# joint index layout per leg: [roll, hip_pitch, knee, ankle]
HIP_IDX = np.array([1, 5])       # of the 8 actuated joints
D2R = np.pi / 180.0


def make_env(hip_deg: float):
    env = BimoWalkerEnv(
        xml_path=XML, actuator_model="sts3215", supply_voltage=11.1,
        command_mode=True, getup=True, imu_obs=True,
        payload_mass=0.154, episode_seconds=14.0,
        render_mode="rgb_array")
    orig_scale = env._scale.copy()
    jnt = env.model.actuator_trnid[:, 0]
    for i in HIP_IDX:
        # widen FLEXION only (negative = flexion on this model; the sit
        # parks at hip -60, the flexion stop). Extension stays at +60.
        env.model.jnt_range[jnt[i]] = (-hip_deg * D2R, 60 * D2R)
    env._lo = env.model.jnt_range[jnt, 0].copy()
    env._hi = env.model.jnt_range[jnt, 1].copy()
    env._scale = 0.5 * (env._hi - env._lo)
    return env, orig_scale


_POLICY = None

def _policy():
    """The trained mjx_getup_v1 sit-up policy (reaches a stable sit from any
    fall). Loaded once; used to produce the REAL entry state for the study
    instead of a hand-built pose (a hand-built 'sit' collapses onto its back
    -- holding hip flexion with unanchored legs lifts the legs, not the
    torso; found the hard way)."""
    global _POLICY
    if _POLICY is None:
        sys.path.insert(0, os.path.join(HERE, "mjx"))
        from eval_ref import load_policy
        _POLICY = load_policy(os.path.join(HERE, "runs", "mjx_getup_v1"))[0]
    return _POLICY


def sit_start(env, seed=0, orig_scale=None):
    """Ragdoll fall (env reset) -> 3 s of the RL sit-up policy -> its stable
    sit. Policy actions are mapped with the ORIGINAL +/-60 deg hip scale (it
    was trained there); the widened range only benefits the scripted plan."""
    obs, _ = env.reset(seed=seed)
    act = _policy()
    scale_now = env._scale.copy()
    env._scale = orig_scale          # policy acts in its trained mapping
    for _ in range(int(3.0 / env.control_dt)):
        a = np.clip(act(obs), -1, 1).astype(np.float32)
        obs, *_ = env.step(a)        # plain step: prev_action stays honest
    env._scale = scale_now
    return obs


def step_targets(env, targets):
    """One control step driving the servos toward absolute joint targets.

    Bypasses the env's residual action mapping: that mapping is a +/-scale
    band around the STANDING pose, which on the asymmetric knee (-95..+5)
    can never command past -50 deg -- the deep tuck was unreachable to every
    policy trained so far (discovered by this study's dangle test). Here the
    PD targets are set to the absolute joint angles, clipped to the true
    joint range only."""
    tgt = np.clip(np.asarray(targets, dtype=float), env._lo, env._hi)
    if not hasattr(env, "_forced_target"):
        orig = env._action_to_ctrl
        env._forced_target = None
        def patched(action, _orig=orig):
            return (env._forced_target if env._forced_target is not None
                    else _orig(action))
        env._action_to_ctrl = patched
    env._forced_target = tgt
    out = env.step(np.zeros(8, dtype=np.float32))
    env._forced_target = None
    return out


def plan(H_deg: float, slow: float = 1.0, roll_l=0.0, roll_r=0.0):
    """(name, duration_s, targets) waypoints, parameterized by hip limit H.
    The sit's leg splay (roll_l/r) is held through the fold -- it's the
    support tripod -- and comes in only during the rise."""
    H = -H_deg * D2R              # NEGATIVE hip pitch = flexion (fold fwd)
    K = -95 * D2R                 # knee full flexion (the real limit --
    A = 40 * D2R                  # unreachable to the old action mapping)
    def leg(hip, knee, ankle, rf=1.0):
        return np.array([roll_l * rf, hip, knee, ankle,
                         roll_r * rf, hip, knee, ankle])
    Ht = max(H, -90 * D2R)        # tuck/squat hip: -90 is enough; deeper
    #                               overfolds into a forward nose-dive
    return [
        # FOLD FIRST: torso forward over the extended legs -- weight stays on
        # the butt while the CoM travels toward the feet. (Tuck-first swings
        # the legs up instead and topples the torso backward -- a V-sit.)
        ("fold",  2.0 * slow, leg(H, -15 * D2R, 0.0)),
        # drag the heels under the folded torso and plant the soles
        ("tuck",  2.0 * slow, leg(Ht, K, A)),
        # settle the heels-squat: CoM over the soles (observed: within
        # ~25 mm here for every hip range, even 60)
        ("rock",  1.5 * slow, leg(Ht, K, 0.25 * A)),
        # SQUAT-RISE, butt first: knees extend while hips STAY folded --
        # the torso pitches over the knees so the CoM tracks the soles.
        # (Extending hip+knee together tips it backward; the failure of
        # every earlier attempt.)
        ("buttup", 2.0 * slow, leg(Ht, -55 * D2R, 0.55 * A)),
        # unfold the torso last, ankles easing off
        ("unfold", 2.0 * slow, leg(0.4 * Ht, -28 * D2R, 0.25 * A, rf=0.5)),
        ("stand",  2.5 * slow, leg(0, 0, 0, rf=0.0)),
        ("hold",   2.0 * slow, leg(0, 0, 0, rf=0.0)),
    ]


def phase_debug(env, name):
    d, m = env.data, env.model
    com = d.subtree_com[1]
    sole = 0.5 * (d.geom_xpos[m.geom("L_sole").id]
                  + d.geom_xpos[m.geom("R_sole").id])
    con_l, con_r = env._foot_contacts()
    up_z = float(d.sensordata[env._up_id + 2])
    print(f"    [{name:6s}] com_x-sole_x={1000*(com[0]-sole[0]):+6.1f} mm  "
          f"h={d.qpos[2]:.3f}  up_z={up_z:+.2f}  soles={int(con_l)}{int(con_r)}  "
          f"pelvis_z={d.qpos[2]:.3f}")


def run_one(H_deg, seed=0, record=True, slow=1.0, debug=False):
    env, orig_scale = make_env(H_deg)
    obs = sit_start(env, seed, orig_scale)
    d = env.data
    up = float(d.sensordata[env._up_id + 2])
    if up < 0.5:
        return dict(recovered=None, stand_hold_s=0.0, best_h=0.0, frames=[],
                    skipped=True)   # policy parked on side/back, not a sit
    if debug:
        phase_debug(env, "sit0")
    frames, stand_hold, best_h = [], 0, 0.0
    t = 0
    for name, dur, tgt in plan(H_deg, slow, roll_l=d.qpos[7],
                               roll_r=d.qpos[11]):
        for _ in range(int(dur / env.control_dt)):
            obs, r, term, trunc, info = step_targets(env, tgt)
            best_h = max(best_h, info["height"])
            stand_hold = stand_hold + 1 if info["standing"] > 0.5 else 0
            if record and t % 6 == 0:
                frames.append(env.render())
            t += 1
        if debug:
            phase_debug(env, name)
    recovered = stand_hold >= int(1.0 / env.control_dt)
    return dict(recovered=recovered, stand_hold_s=stand_hold * env.control_dt,
                best_h=best_h, frames=frames)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hips", default="60,75,90,105,120,135")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--slow", type=float, default=1.0)
    p.add_argument("--debug", action="store_true")
    args = p.parse_args()
    os.makedirs(OUT, exist_ok=True)
    import imageio.v2 as imageio
    print(f"{'hip range':>10s} {'recovered':>10s} {'hold_s':>7s} {'best_h':>7s}")
    SIT_SEEDS = [4, 7, 207, 5, 6, 8, 9, 10, 11, 12]   # scanned: policy sits
    for H in [float(x) for x in args.hips.split(",")]:
        rs, tried = [], 0
        for s in SIT_SEEDS:
            if len(rs) >= args.seeds:
                break
            r = run_one(H, seed=s, record=(len(rs) == 0), slow=args.slow,
                        debug=(args.debug and len(rs) == 0))
            tried += 1
            if not r.get("skipped"):
                rs.append(r)
        ok = sum(r["recovered"] for r in rs)
        r0 = rs[0]
        print(f"{H:8.0f}deg {ok:>7d}/{args.seeds} "
              f"{max(r['stand_hold_s'] for r in rs):7.2f} "
              f"{max(r['best_h'] for r in rs):7.3f}")
        if r0["frames"]:
            idx = np.linspace(0, len(r0["frames"]) - 1, 10).astype(int)
            strip = np.concatenate([r0["frames"][i][..., :3] for i in idx], 1)
            imageio.imwrite(os.path.join(OUT, f"hip{int(H)}.png"),
                            strip[::2, ::2])
    print(f"strips -> {OUT}/hip<H>.png")


if __name__ == "__main__":
    main()
