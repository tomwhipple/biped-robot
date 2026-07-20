"""Gate for the sit start pose (video-review feedback 2026-07-20).

Verifies, on the CPU engine (the referee; sim/mjx uses the same constants):
  1. SPEC: torso pointing up (up_z), waist bent ~90 deg, legs out front
     (knees near straight), immediately after the settled reset.
  2. STABILITY: the pose holds through 2 s of SERVO-HELD sitting (PD
     targets = the settled sit joints, what a trivial policy would do) --
     a sit that topples onto its side/back fails. (Torque-free is the
     wrong bar: limp joints let the top-heavy torso fold backward, but a
     real robot sits with servos engaged.)
  3. NO FOOT OVERLAP: the foot meshes cannot occupy the same space --
     lateral gap between sole edges >= 5 mm throughout (the leg meshes
     don't self-collide, so only the pose keeps them apart).

Run:  JAX_PLATFORMS=cpu .venv/bin/python sim/mjx/check_sit_pose.py
Exits 0 on PASS, 1 on FAIL.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import mujoco
from walker_env import BimoWalkerEnv

SOLE_HALF_W = 0.023          # sole box half width (y)
MIN_GAP = 0.005              # required clearance between sole edges (m)


def main():
    env = BimoWalkerEnv(
        xml_path=os.path.join(HERE, "..", "bimo_biped_v2_asbuilt.xml"),
        command_mode=True, ext_cmd=True, actuator_model="sts3215",
        imu_obs=True, recover_mix=1.0, recover_start_mix=(0, 0, 0, 1),
        payload_mass=0.154, payload_cg_z=0.0945)
    ok = True
    for seed in range(4):
        env.reset(seed=seed)
        d = env.data

        def state(tag):
            nonlocal ok
            up_z = float(d.sensordata[env._up_id + 2])
            hip = np.rad2deg(float(d.qpos[7 + 1]))     # left hip pitch
            knee = np.rad2deg(float(d.qpos[7 + 2]))
            yl = float(d.geom_xpos[env._sole_gids[0]][1])
            yr = float(d.geom_xpos[env._sole_gids[1]][1])
            gap = abs(yl - yr) - 2 * SOLE_HALF_W
            checks = {
                "torso up (up_z >= 0.90)": up_z >= 0.90,
                "waist ~90 deg (hip in [-100,-75])": -100 <= hip <= -75,
                "legs out front (knee >= -25 deg)": knee >= -25,
                "foot gap >= 5 mm": gap >= MIN_GAP,
            }
            bad = [k for k, v in checks.items() if not v]
            print(f"  seed {seed} {tag}: up_z={up_z:.3f} hip={hip:.0f}deg "
                  f"knee={knee:.0f}deg foot_gap={gap*1000:.1f}mm "
                  f"{'OK' if not bad else 'FAIL: ' + '; '.join(bad)}")
            ok &= not bad

        state("post-reset ")
        # 2 s servo-held: PD toward the settled sit joints (legacy action
        # mapping: a = (target - default)/scale)
        hold = np.clip(np.asarray(d.qpos[7:15]), env._lo, env._hi)
        a = np.clip((hold - env._default) / env._scale, -1.0, 1.0)
        for _ in range(100):
            env.step(a.astype(np.float32))
        state("post-2s hold")
    print("SIT POSE:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
