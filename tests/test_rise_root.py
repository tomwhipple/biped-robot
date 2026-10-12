"""Root tracking for the recorded get-up (w_rise_root, 2026-10-12).

Joint-only DeepMimic tracking let the face-down get-up policies park in a
hunched half-stand (pelvis 0.27 m, torso up-z ~0.6) that never counts as
standing. The root term pays for matching the reference's pelvis height and
torso up-z at the current phase. One jax-free kernel in walker_env is shared
by both envs (env_mjx imports it), so the formula cannot drift between them.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import walker_env as W  # noqa: E402

REF = os.path.join(SIM, "getup_ref_v6ar.npz")


def _quat_pitch(theta):
    """(w, x, y, z) for a rotation of theta about y: up-z = cos(theta)."""
    return np.array([np.cos(theta / 2), 0.0, np.sin(theta / 2), 0.0])


def test_reference_root_ends_upright():
    root = W.rise_root_ref(np.load(REF)["qpos"])
    assert root.shape[1] == 2
    assert root[-1, 1] > 0.98            # torso vertical at the end
    assert 0.37 < root[-1, 0] < 0.41     # pelvis at standing height
    assert root[0, 0] < 0.1              # starts lying


def test_kernel_values():
    ref = np.array([0.39, 1.0])
    assert np.isclose(W.rise_root_kernel(0.39, _quat_pitch(0.0), ref, np), 1.0)
    # the hunched half-stand the policies parked in: z 0.27, up-z 0.6
    hunched = W.rise_root_kernel(0.27, _quat_pitch(np.arccos(0.6)), ref, np)
    assert hunched < 0.05
    # up-z from the quaternion matches cos(pitch)
    for th in (0.3, 0.9, 1.4):
        k = W.rise_root_kernel(0.39, _quat_pitch(th), ref, np)
        assert np.isclose(k, np.exp(-((np.cos(th) - 1.0) / W.RISE_ROOT_SIG_UP) ** 2))


def test_kernel_numpy_matches_jax():
    import jax.numpy as jp
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        z, ref = rng.uniform(0.05, 0.4), np.array([rng.uniform(0.05, 0.4), rng.uniform(-1, 1)])
        a = W.rise_root_kernel(z, q, ref, np)
        b = float(W.rise_root_kernel(jp.float32(z), jp.asarray(q, jp.float32), jp.asarray(ref, jp.float32), jp))
        assert np.isclose(a, b, rtol=1e-4, atol=1e-6)


def test_cpu_env_pays_the_root_term():
    """At the reference's first frame, w_rise_root=5 adds 5 x the kernel to
    the step reward of an otherwise identical env."""
    import eval_precision as ep
    import robot_plant
    kw = robot_plant.robot_env_kwargs()
    held = {k: v for k, v in kw["held_joints"].items()
            if robot_plant._role(k) not in ("shoulder", "elbow")}
    secs, factory, _ = ep._registry()["recover_supine"]
    rewards = {}
    for w in (0.0, 5.0):
        cfg = dict(kw, held_joints=held, rise_ref_npz="getup_ref_v6ar.npz",
                   rise_ref_residual=0.3, w_rise_ref=2.0, w_rise_root=w,
                   precision=True, action_map="full")
        env = ep.make_env(cfg, secs, True, kw["xml_path"],
                          extra=ep.ENV_EXTRA["recover_supine"])
        env.reset(seed=3)
        drv = ep.Driver(env, None, 3, False)
        ep._ref_setup(env, drv, "getup_ref_v6ar.npz")
        _, r, *_ = env.step(np.zeros(env.action_space.shape[0], np.float32))
        k = W.rise_root_kernel(float(env.data.qpos[2]), np.asarray(env.data.qpos[3:7]),
                               env._rr["root"][min(env._step_i - 1, env._rr["T"] - 1)], np)
        rewards[w] = (r, k)
    (r0, _), (r5, k5) = rewards[0.0], rewards[5.0]
    assert k5 > 0.5                       # starts on the reference's root
    assert np.isclose(r5 - r0, 5.0 * k5, rtol=0.05, atol=0.05)
