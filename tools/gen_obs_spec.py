#!/usr/bin/env python3
"""Generate the firmware's obs spec header + golden observation vectors.

Run:  .venv/bin/python tools/gen_obs_spec.py --run loco_v5t

docs/firmware-design.md section 5 says the obs SPEC "is exported from the sim
as a generated header so it cannot drift by hand". This is that exporter. It
reads the deployed run's config.json and its MJCF, and writes:

  firmware/components/obs/include/obs/obs_spec.h   layout + joint table
  firmware/host/vectors/obs_vectors.h              real _obs() frames

The golden frames come from walker_env._obs() itself (domain randomisation
off, so the IMU misalignment is identity and the gyro bias zero), driven from
scripted qpos/qvel/quaternion/prev_action/phase/command values. If anybody
reorders the observation, the C assembler's host test goes red.

The action-index -> servo-ID map is NOT in the sim; it comes from
docs/wiring.md's bus layout and lives in ID_BY_ROLE below. Note that on the
10-DOF v3yaw plant it is a genuine permutation: the sim's action order is
proximal-to-distal INCLUDING hip yaw first (L_hip_yaw is action 0), while the
bus was numbered 1-8 for the two legs before hip yaw existed and the yaw
servos were appended as IDs 9/10. wiring.md's claim that the action vector
"maps to IDs 1-8 with no permutation table" was true for the 8-DOF plant only.
"""
import argparse
import inspect
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))

import mujoco                                          # noqa: E402
from walker_env import BimoWalkerEnv                   # noqa: E402

SPEC_OUT = os.path.join(ROOT, "firmware", "components", "obs", "include",
                        "obs", "obs_spec.h")
VEC_OUT = os.path.join(ROOT, "firmware", "host", "vectors", "obs_vectors.h")

# docs/wiring.md "Servo bus": port A is the left leg (IDs 1-4, proximal to
# distal), port B the right (5-8); the v3yaw hip-yaw servos were appended as
# 9 (left) and 10 (right).
#
# ASSEMBLY ERRATA 2026-08-02. Confirmed on the robot, not inferred:
#
#   * driving servo 9's hip yaw rotates the RIGHT leg (toe-in at +20 deg),
#   * hip-pitch servo 2 is on the RIGHT, hip-pitch servo 6 is on the LEFT,
#   * knee servo 3 is the RIGHT knee, knee servo 7 is the LEFT.
#
# So the two bus chains went onto the opposite legs from the plan: port A
# (9,1,2,3,4) is the RIGHT leg and port B (10,5,6,7,8) is the LEFT. The
# original "9/10 are swapped" report was the visible corner of this.
#
# The servos are not coming back out, so the map absorbs it. Note the yaw
# entries are the SAME as the first errata fix -- 9 is right, 10 is left --
# because a whole-chain swap and a yaw-only swap agree on the yaw servos.
ID_BY_ROLE = {
    "L_hip_roll": 5, "L_hip_pitch": 6, "L_knee": 7, "L_ankle": 8,
    "R_hip_roll": 1, "R_hip_pitch": 2, "R_knee": 3, "R_ankle": 4,
    "L_hip_yaw": 10, "R_hip_yaw": 9,
}


def fl(x):
    s = "%.9g" % float(x)
    if not any(c in s for c in ".eEn"):
        s += ".0"
    return s + "f"


def farr(v):
    return "{" + ", ".join(fl(x) for x in np.ravel(v)) + "}"


def build_env(run_dir):
    cfg = json.load(open(os.path.join(run_dir, "config.json")))
    params = set(inspect.signature(BimoWalkerEnv.__init__).parameters)
    kw = {k: v for k, v in cfg.items() if k in params}
    kw.update(
        xml_path=os.path.join(ROOT, "sim", cfg["xml_path"]),
        command_mode=True, imu_obs=True, actuator_model="sts3215",
        # DR off: we want the identity IMU mount and zero gyro bias, so the
        # frames are a pure function of the scripted inputs.
        domain_rand=False, latency_ms=0.0, latency_ms_max=None,
        render_mode=None,
    )
    return cfg, BimoWalkerEnv(**kw)


def sample_frames(env, n=8, seed=0):
    """Drive _obs() from scripted state and return (inputs, frame) pairs."""
    rng = np.random.default_rng(seed)
    env.reset(seed=seed)
    nq = env._nq_act
    out = []
    for k in range(n):
        d = env.data
        q = rng.uniform(env._lo, env._hi) if k else np.zeros(nq)
        dq = rng.uniform(-4.0, 4.0, nq) if k else np.zeros(nq)
        # A random unit quaternion gives a non-trivial up-vector; the first
        # sample stays upright so the "standing" case is pinned exactly.
        quat = np.array([1.0, 0.0, 0.0, 0.0]) if k == 0 else rng.normal(size=4)
        quat = quat / np.linalg.norm(quat)
        gyro = rng.uniform(-3.0, 3.0, 3) if k else np.zeros(3)
        prev = rng.uniform(-1.0, 1.0, nq) if k else np.zeros(nq)
        phase = float(rng.uniform(-np.pi, np.pi)) if k else 0.0
        cmd = rng.uniform(-1.0, 1.0, env._ncmd) if k else np.zeros(env._ncmd)

        d.qpos[env._jqpos] = q
        d.qvel[env._jqvel] = dq
        d.qpos[3:7] = quat
        d.qvel[3:6] = gyro
        d.qvel[0:3] = rng.uniform(-1.0, 1.0, 3)   # must be ZEROED in the obs
        d.qpos[2] = 0.31                          # ... and so must height
        mujoco.mj_forward(env.model, d)
        env._prev_action = prev.astype(np.float32)
        env._gait_phase = phase
        env._cmd = cmd
        # Read the up-vector the sensor produced: that is what the BNO085 has
        # to reproduce on hardware, so it is an INPUT to the C assembler.
        up = np.array(d.sensordata[env._up_id:env._up_id + 3])

        env._obs_hist = []          # frame-only read; history is tested in C
        frame = env._obs()
        assert frame.shape == (env.obs_frame,), frame.shape
        out.append(dict(q=q, dq=dq, up=up, gyro=gyro, prev=prev, phase=phase,
                        cmd=cmd, frame=frame))
    return out


def write_spec(cfg, env, path):
    names = env._act_names
    ids = [ID_BY_ROLE[n] for n in names]
    nq, ncmd, hist = env._nq_act, env._ncmd, env.obs_hist_len
    L = []
    w = L.append
    w("// GENERATED by tools/gen_obs_spec.py -- do not edit.")
    w("// Source: sim/runs/%s/config.json + sim/%s"
      % (os.path.basename(cfg["_run"]), cfg["xml_path"]))
    w("//")
    w("// The observation layout the deployed policy was trained against. Any")
    w("// retrain that changes joint count, command width or history depth")
    w("// regenerates this file; nothing downstream hardcodes 10 or 49.")
    w("#pragma once")
    w("#include <stdint.h>")
    w("")
    w("namespace obs {")
    w("")
    w("// -- deployed run ------------------------------------------------------")
    w('inline constexpr const char* kRunName = "%s";'
      % os.path.basename(cfg["_run"]))
    w('inline constexpr const char* kPlantXml = "%s";' % cfg["xml_path"])
    w('inline constexpr const char* kActionMap = "%s";' % env.action_map)
    w("inline constexpr bool kImuObs = %s;   // linvel + height are zeroed"
      % ("true" if env.imu_obs else "false"))
    w("inline constexpr bool kGaitClock = %s;"
      % ("true" if env.gait_clock else "false"))
    w("inline constexpr float kControlDt = %s;   // 50 Hz tick"
      % fl(env.control_dt))
    w("")
    w("// -- dimensions --------------------------------------------------------")
    w("inline constexpr int kNumJoints = %d;" % nq)
    w("inline constexpr int kActDim = %d;" % nq)
    w("inline constexpr int kNumCmd = %d;" % ncmd)
    w("inline constexpr int kHistLen = %d;" % hist)
    w("inline constexpr int kFrameDim = %d;" % env.obs_frame)
    w("inline constexpr int kObsDim = %d;" % (env.obs_frame * hist))
    w("")
    w("// -- frame layout (offsets into one kFrameDim-wide frame) --------------")
    off = 0
    for label, size in (("Q", nq), ("Dq", nq), ("Up", 3), ("LinVel", 3),
                        ("Gyro", 3), ("PrevAction", nq), ("Height", 1),
                        ("Phase", 2), ("Cmd", ncmd)):
        w("inline constexpr int kOff%s = %d;%s"
          % (label, off, "   // %d wide" % size))
        off += size
    assert off == env.obs_frame, (off, env.obs_frame)
    w("")
    w("// -- joints ------------------------------------------------------------")
    w("// Index == the policy's action index == the sim's actuator order")
    w("// (joint-tree / qpos-address order). kServoId maps it onto the bus IDs")
    w("// from docs/wiring.md -- a real permutation on the 10-DOF plant, since")
    w("// the yaw servos were appended as 9/10 after IDs 1-8 were assigned.")
    w("inline constexpr const char* kJointNames[kNumJoints] = {%s};"
      % ", ".join('"%s"' % n for n in names))
    w("inline constexpr uint8_t kServoId[kNumJoints] = {%s};"
      % ", ".join(str(i) for i in ids))
    w("inline constexpr float kJointLo[kNumJoints] = %s;" % farr(env._lo))
    w("inline constexpr float kJointHi[kNumJoints] = %s;" % farr(env._hi))
    w("inline constexpr float kJointDefault[kNumJoints] = %s;"
      % farr(env._default))
    w("")
    w("}  // namespace obs")
    w("")
    with open(path, "w") as f:
        f.write("\n".join(L))
    return ids


def write_vectors(env, samples, path):
    L = []
    w = L.append
    w("// GENERATED by tools/gen_obs_spec.py -- do not edit.")
    w("// Real walker_env._obs() frames (domain randomisation off).")
    w("#pragma once")
    w("#include <stddef.h>")
    w("")
    w("namespace obs_vectors {")
    w("")
    w("inline constexpr int kNumJoints = %d;" % env._nq_act)
    w("inline constexpr int kNumCmd = %d;" % env._ncmd)
    w("inline constexpr int kFrameDim = %d;" % env.obs_frame)
    w("")
    w("struct Case {")
    w("    float q[kNumJoints]; float dq[kNumJoints];")
    w("    float up[3]; float gyro[3]; float prev_action[kNumJoints];")
    w("    float phase; float cmd[kNumCmd];")
    w("    float frame[kFrameDim];")
    w("};")
    w("inline const Case kCases[] = {")
    for s in samples:
        w("    {%s, %s, %s, %s, %s, %s, %s,"
          % (farr(s["q"]), farr(s["dq"]), farr(s["up"]), farr(s["gyro"]),
             farr(s["prev"]), fl(s["phase"]), farr(s["cmd"])))
        w("     %s}," % farr(s["frame"]))
    w("};")
    w("inline constexpr size_t kNumCases = sizeof kCases / sizeof kCases[0];")
    w("")
    w("}  // namespace obs_vectors")
    w("")
    with open(path, "w") as f:
        f.write("\n".join(L))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="loco_v5t",
                    help="run under sim/runs/ whose config defines the spec")
    ap.add_argument("--samples", type=int, default=8)
    args = ap.parse_args()

    run_dir = os.path.join(ROOT, "sim", "runs", args.run)
    cfg, env = build_env(run_dir)
    cfg["_run"] = run_dir
    if not env.imu_obs:
        raise SystemExit("refusing to export: the deployed run must be "
                         "imu_obs=True (the robot has no linvel/height sensor)")
    samples = sample_frames(env, args.samples)
    os.makedirs(os.path.dirname(SPEC_OUT), exist_ok=True)
    os.makedirs(os.path.dirname(VEC_OUT), exist_ok=True)
    ids = write_spec(cfg, env, SPEC_OUT)
    write_vectors(env, samples, VEC_OUT)
    print("run       : %s (%s)" % (args.run, cfg["xml_path"]))
    print("obs       : %d joints, %d cmd, hist %d -> frame %d, obs %d"
          % (env._nq_act, env._ncmd, env.obs_hist_len, env.obs_frame,
             env.obs_frame * env.obs_hist_len))
    print("action->ID: %s"
          % ", ".join("%s=%d" % (n, i) for n, i in zip(env._act_names, ids)))
    print("wrote %s" % SPEC_OUT)
    print("wrote %s (%d cases)" % (VEC_OUT, len(samples)))


if __name__ == "__main__":
    main()
