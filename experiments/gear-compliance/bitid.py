#!/usr/bin/env python3
"""Bit-identity check for the per-side roll series spring (#6 gear compliance; the servo-thermal pattern).

Runs 60 domain-randomised steps of random actions in BOTH envs (MJX and CPU), on the robot preset of
sim/mjx/parity_test.py, the default plant and the plant with the SHARED roll series spring drawn per episode (the path that
existed before the per-side keys) -- from
the sim/ of the tree given, and saves every qpos, obs and reward. Run it on a `git archive` of main and on the
branch, then compare the two .npz files array for array:

    git archive origin/main sim paths.py | tar -x -C /tmp/base && ln -s $PWD/build /tmp/base/build
    JAX_PLATFORMS=cpu python experiments/gear-compliance/bitid.py /tmp/base /tmp/base.npz [f32]
    JAX_PLATFORMS=cpu python experiments/gear-compliance/bitid.py $PWD /tmp/branch.npz [f32]

f32: the MJX env in float32 (training); default float64 (parity_test turns x64 on).
"""
import os, sys
root = sys.argv[1]; out = sys.argv[2]
sys.path.insert(0, os.path.join(root, "sim", "mjx")); sys.path.insert(0, os.path.join(root, "sim"))
os.environ.setdefault("MUJOCO_GL", "disabled")
import numpy as np, jax, jax.numpy as jp
import parity_test as PT
if len(sys.argv) > 3 and sys.argv[3] == "f32":
    jax.config.update("jax_enable_x64", False)
res = {}
for tag, over in (("default", {}), ("series", dict(roll_series_k="hip=8:16,ankle=12:25"))):
    kw = PT.rob_kw(**over)
    from env_mjx import BimoMJXEnv
    from walker_env import BimoWalkerEnv
    g = BimoMJXEnv(domain_rand=True, **kw)
    step = jax.jit(g.step)
    st = g.reset(jax.random.PRNGKey(3))
    n = g.action_size if hasattr(g, "action_size") else st.prev_action.shape[0]
    rng = np.random.default_rng(0)
    q, o, r = [], [], []
    for t in range(60):
        a = jp.asarray(rng.uniform(-1, 1, n).astype(np.float32))
        st = step(st, a)
        q.append(np.asarray(st.data.qpos)); o.append(np.asarray(st.obs)); r.append(float(st.reward))
    res[f"{tag}_mjx_q"] = np.array(q); res[f"{tag}_mjx_o"] = np.array(o); res[f"{tag}_mjx_r"] = np.array(r)
    c = BimoWalkerEnv(actuator_model="sts3215", command_mode=True, domain_rand=True, **kw)
    c.reset(seed=5)
    q, o, r = [], [], []
    for t in range(60):
        a = rng.uniform(-1, 1, c.action_space.shape[0]).astype(np.float32)
        ob, rw, term, trunc, _ = c.step(a)
        q.append(np.array(c.data.qpos)); o.append(ob); r.append(rw)
        if term or trunc:
            c.reset(seed=6 + t)
    res[f"{tag}_cpu_q"] = np.array(q); res[f"{tag}_cpu_o"] = np.array(o); res[f"{tag}_cpu_r"] = np.array(r)
np.savez(out, **res)
print("saved", out, {k: v.shape for k, v in res.items()})
