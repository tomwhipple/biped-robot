"""Parity test: MJX env vs CPU BimoWalkerEnv.

Findings this test encodes (2026-07-13 investigation):
  * Physics arithmetic is EXACT between engines: synced single steps match to
    ~1e-16 in float64, including floor contacts, the STS3215 torque envelope,
    latency, and backlash.
  * The ONE semantic difference is the contact-manifold generator: MJX's
    plane_convex/box colliders emit only points within a 1 mm skin of the
    deepest penetration (mjx/_src/collision_convex.py), where CPU MuJoCo
    emits every penetrating corner. Manifolds therefore differ only in
    tilted-deep-penetration transients (impact instants) and foot-on-foot
    clipping. In settled stance (~0.2 mm penetration) manifolds are
    identical. Consequence: individual impact events resolve slightly
    differently -> trajectories diverge chaotically at impacts, exactly as
    they would under any 1e-7 perturbation. Mitigation: DR + every trained
    policy is refereed by the CPU harness (eval_policy/eval_commands)
    before being believed.

Gates (run in float64 so precision is not the limiter):
  1. AIRBORNE synced-step parity, latency 6 ms + backlash 0.5 deg active:
     validates the whole actuator/reward/obs port with no contacts involved.
  2. GROUNDED synced-step parity with gentle actions around stance
     (matched-manifold regime): validates contact physics where the
     manifolds agree.
  3. float32 free-rollout drift: informational only (chaos, not error).

Run:  .venv/bin/python sim/mjx/parity_test.py
"""
import os
import sys

import jax
jax.config.update("jax_enable_x64", True)

import jax.numpy as jp
import numpy as np
from mujoco import mjx

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from walker_env import BimoWalkerEnv
from env_mjx import BimoMJXEnv

XML = os.path.join(HERE, "..", "bimo_biped_v2.xml")

SHARED = dict(
    supply_voltage=11.1,
    w_energy=0.002, w_action_rate=0.05, w_power=0.02,
    w_feet_air=0.1, w_single_support=0.05, w_lateral=0.5, w_pitch_rate=0.05,
    w_track_v=2.0, w_track_w=1.0,
    latency_ms=6.0, backlash_deg=0.5,
    cmd_fixed=(0.6, 0.0), imu_obs=False,
    action_map="full", hip_flex_deg=110.0,   # the get-up study additions
)

cpu = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                    command_mode=True, domain_rand=False, **SHARED)
gpu = BimoMJXEnv(xml_path=XML, domain_rand=False, **SHARED)
step_mjx = jax.jit(gpu.step)


def sync(state, cpu, gpu_env=None):
    # np.array copies everywhere: jp.asarray can alias numpy buffers
    # zero-copy on the CPU backend, and the CPU env mutates _prev_action
    # and _air_time IN PLACE during step -- an aliased array silently reads
    # post-step values inside the later jit call (cost: hours of debugging).
    gpu_env = gpu if gpu_env is None else gpu_env
    d = state.data.replace(qpos=jp.asarray(np.array(cpu.data.qpos)),
                           qvel=jp.asarray(np.array(cpu.data.qvel)))
    d = mjx.forward(gpu_env.model, d)
    return state._replace(
        data=d,
        prev_action=jp.asarray(np.array(cpu._prev_action)),
        last_target=jp.asarray(np.array(cpu._last_target)),
        step_i=jp.asarray(cpu._step_i, dtype=jp.int32),
        air_time=jp.asarray(np.array(cpu._air_time)),
        cmd=jp.asarray(np.array(cpu._cmd)),
        traj=jp.asarray(np.array(cpu._traj)),
        traj_on=jp.asarray(1.0 if cpu._traj_on else 0.0),
    )


def _mjx_ncon(state):
    return _mjx_ncon_of(state)


def _mjx_ncon_of(state):
    cc = (state.data._impl.contact if hasattr(state.data, "_impl")
          else state.data.contact)
    return int(np.sum(np.asarray(cc.dist) < 0))


def run_block(name, n_steps, act_fn, prep_cpu, gates, matched_only=False,
              min_frac=0.8, envs=None):
    cpu_e, gpu_e, step_e = envs if envs is not None else (cpu, gpu, step_mjx)
    obs_c, _ = cpu_e.reset(seed=0)
    prep_cpu(cpu_e)
    state = gpu_e.reset(jax.random.PRNGKey(0))
    worst = dict(qpos=0.0, qvel=0.0, reward=0.0, obs=0.0)
    used = skipped = 0
    for t in range(n_steps):
        a = act_fn(t)
        state = sync(state, cpu_e, gpu_e)
        obs_c, r_c, term_c, trunc_c, _ = cpu_e.step(a)
        state = step_e(state, jp.asarray(a))
        if matched_only and cpu_e.data.ncon != _mjx_ncon(state):
            skipped += 1     # manifold mismatch: regime this block can't gate
            continue
        used += 1
        worst["qpos"] = max(worst["qpos"], float(np.max(np.abs(
            np.asarray(state.data.qpos[7:15]) - cpu_e.data.qpos[7:15]))))
        worst["qvel"] = max(worst["qvel"], float(np.max(np.abs(
            np.asarray(state.data.qvel[6:14]) - cpu_e.data.qvel[6:14]))))
        worst["reward"] = max(worst["reward"], abs(float(state.reward) - r_c))
        worst["obs"] = max(worst["obs"], float(np.max(np.abs(
            np.asarray(state.obs) - obs_c))))
        if trunc_c:
            obs_c, _ = cpu_e.reset(seed=1000 + t)
            prep_cpu(cpu_e)
    enough = used >= min_frac * n_steps
    ok = enough and all(worst[k] < gates[k] for k in gates)
    print(f"== {name}: {'PASS' if ok else 'FAIL'} "
          f"({used} steps compared, {skipped} skipped) ==")
    for k in ("qpos", "qvel", "reward", "obs"):
        gate = f"  (gate {gates[k]:.0e})" if k in gates else ""
        print(f"   worst |d{k}| = {worst[k]:.2e}{gate}")
    return ok


# -- 1. airborne: pure actuator/reward/obs arithmetic, no contacts ------------
def hoist(env):
    """Torso 1.5 m up, zero velocity; re-hoisted every 20 steps so the robot
    never reaches the floor. Legs are splayed by the action offset below so
    the feet never touch each other (sole-sole box contact is exactly the
    manifold-divergent case this block must exclude)."""
    env.data.qpos[2] = 1.5
    env.data.qvel[:] = 0.0
    import mujoco
    mujoco.mj_forward(env.model, env.data)

# hip-roll offsets splay the legs; modest amplitude keeps feet apart
_SPLAY = np.array([0.5, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0])

class AirActs:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu)   # both get re-synced right after, so this is shared
        return (_SPLAY + 0.3 * np.sin(0.35 * t + np.arange(8) * 0.7)
                ).astype(np.float32)

ok1 = run_block(
    "1. airborne arithmetic (latency 6 ms, backlash 0.5 deg)", 100, AirActs(),
    hoist, dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5))

# -- 2. grounded quasi-static stance: flat feet, matched 4-corner manifolds ---
ok2 = run_block(
    "2. grounded stance physics (matched manifolds)", 100,
    lambda t: (0.05 * np.sin(0.25 * t + np.arange(8))).astype(np.float32),
    lambda env: None, dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3),
    matched_only=True, min_frac=0.5)

# -- 2b. get-up mode: recovery-reward arithmetic from fallen states -----------
cpu_g = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, getup=True,
                      supply_voltage=11.1, w_energy=0.002, w_action_rate=0.15,
                      w_power=0.008, w_pitch_rate=0.1, w_upright=0.0,
                      alive_bonus=0.0, imu_obs=False)
gpu_g = BimoMJXEnv(xml_path=XML, domain_rand=False, getup=True,
                   supply_voltage=11.1, w_energy=0.002, w_action_rate=0.15,
                   w_power=0.008, w_pitch_rate=0.1, w_upright=0.0,
                   alive_bonus=0.0, imu_obs=False)
step_g = jax.jit(gpu_g.step)
obs_c, _ = cpu_g.reset(seed=3)
sg = gpu_g.reset(jax.random.PRNGKey(3))
worst_r = worst_o = 0.0
used_g = 0
import mujoco as _mj
for t in range(80):
    if t % 20 == 0:      # hoist: recovery reward has no contact terms, so
        cpu_g.data.qpos[2] = 1.5          # gate its arithmetic contact-free
        cpu_g.data.qvel[:] = 0.0          # (heap dynamics divergence is the
        _mj.mj_forward(cpu_g.model, cpu_g.data)   # documented manifold caveat)
    a = (_SPLAY + 0.3 * np.sin(0.3 * t + np.arange(8))).astype(np.float32)
    d = sg.data.replace(qpos=jp.asarray(np.array(cpu_g.data.qpos)),
                        qvel=jp.asarray(np.array(cpu_g.data.qvel)))
    d = mjx.forward(gpu_g.model, d)
    sg = sg._replace(data=d, prev_action=jp.asarray(np.array(cpu_g._prev_action)),
                     last_target=jp.asarray(np.array(cpu_g._last_target)),
                     step_i=jp.asarray(cpu_g._step_i, dtype=jp.int32),
                     air_time=jp.asarray(np.array(cpu_g._air_time)),
                     cmd=jp.asarray(np.array(cpu_g._cmd)))
    obs_c, r_c, term_c, trunc_c, info_c = cpu_g.step(a)
    sg = step_g(sg, jp.asarray(a))
    if term_c:
        raise SystemExit("getup CPU env terminated -- must never happen")
    if float(sg.done) != 0.0:
        raise SystemExit("getup MJX env set done -- must never happen")
    if cpu_g.data.ncon != 0:
        continue                          # airborne-only gate
    used_g += 1
    worst_r = max(worst_r, abs(float(sg.reward) - r_c))
    worst_o = max(worst_o, float(np.max(np.abs(np.asarray(sg.obs) - obs_c))))
ok3 = used_g >= 40 and worst_r < 1e-5 and worst_o < 1e-5
print(f"== 2b. get-up recovery-reward arithmetic (contact-free): "
      f"{'PASS' if ok3 else 'FAIL'} ({used_g}/80 airborne steps) ==")
print(f"   worst |dreward| = {worst_r:.2e}  worst |dobs| = {worst_o:.2e}")

# -- 2c/2d. ext_cmd precision mode (2026-07-17) --------------------------------
# 2c airborne: gates the new kernel arithmetic (2D velocity, height-vs-crouch,
# swing-foot target) with a fixed one-leg command carrying a foot offset.
# 2d grounded stance: gates the lift contact-pattern term (lift_ok) and the
# crouch-scaled fall floor in the matched-manifold regime.
EXT = dict(
    supply_voltage=11.1, w_energy=0.002, w_action_rate=0.05, w_power=0.02,
    w_feet_air=0.1, w_single_support=0.05, w_lateral=0.5, w_pitch_rate=0.05,
    w_track_v=2.0, w_track_w=2.0, w_track_h=1.0, w_lift=1.0, w_track_foot=1.0,
    latency_ms=6.0, backlash_deg=0.5, ext_cmd=True, fall_cost=10.0,
    cmd_fixed=(0.0, 0.0, 0.0, 0.9, -1.0, 0.01, 0.02), imu_obs=False,
    action_map="full", hip_flex_deg=110.0,
)
cpu_e = BimoWalkerEnv(xml_path=XML, actuator_model="sts3215",
                      command_mode=True, domain_rand=False, **EXT)
gpu_e = BimoMJXEnv(xml_path=XML, domain_rand=False, **EXT)
step_e = jax.jit(gpu_e.step)


class AirActsExt:
    def __call__(self, t):
        if t % 20 == 0:
            hoist(cpu_e)
        return (_SPLAY + 0.3 * np.sin(0.35 * t + np.arange(8) * 0.7)
                ).astype(np.float32)

ok_e1 = run_block(
    "2c. ext_cmd airborne arithmetic (lift -1, foot target, crouch 0.9)",
    100, AirActsExt(), hoist,
    dict(qpos=1e-8, qvel=1e-6, reward=1e-5, obs=1e-5),
    envs=(cpu_e, gpu_e, step_e))

ok_e2 = run_block(
    "2d. ext_cmd grounded stance (lift_ok pattern, matched manifolds)", 100,
    lambda t: (0.05 * np.sin(0.25 * t + np.arange(8))).astype(np.float32),
    lambda env: None, dict(qpos=1e-6, qvel=1e-4, reward=1e-3, obs=1e-3),
    matched_only=True, min_frac=0.5, envs=(cpu_e, gpu_e, step_e))

# -- 3. gait-amplitude manifold statistics (informational, no gate) -----------
print("== 3. gait-amplitude contact-manifold statistics (informational) ==")
obs_c, _ = cpu.reset(seed=7)
state = gpu.reset(jax.random.PRNGKey(7))
matched = total_con = 0
for t in range(150):
    a = (0.4 * np.sin(0.35 * t + np.arange(8) * 0.7)).astype(np.float32)
    state = sync(state, cpu)
    obs_c, r_c, term_c, trunc_c, _ = cpu.step(a)
    state = step_mjx(state, jp.asarray(a))
    if cpu.data.ncon:
        total_con += 1
        cc = (state.data._impl.contact if hasattr(state.data, "_impl")
              else state.data.contact)
        mjx_n = int(np.sum(np.asarray(cc.dist) < 0))
        matched += int(mjx_n == cpu.data.ncon)
    if term_c or trunc_c:
        obs_c, _ = cpu.reset(seed=2000 + t)
print(f"   contact steps: {total_con}, same contact-point count: {matched} "
      f"({100.0 * matched / max(total_con, 1):.0f}%)")
print("   (mismatches = tilted/deep penetration where MJX's 1 mm-skin")
print("    manifold pruning drops corners CPU keeps -- impact transients.")
print("    Settled stance manifolds are identical. Referee: CPU evals.)")

ok_all = ok1 and ok2 and ok3 and ok_e1 and ok_e2
print("\nPARITY:", "PASS" if ok_all else "FAIL")
sys.exit(0 if ok_all else 1)
