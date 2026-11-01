"""The SIL stack on each POLICY PROFILE's plant (tools/policy_profile.py).

A profile is a second compiled-in policy for a plant the deployed one does not
drive -- v6legs: the robot's 12 leg joints, ankle rolls included. sim/sil/
harness.py binds to one profile per process ($BIMO_POLICY, read at import), so
each check here runs in a subprocess with the profile's environment.

Per profile, against the run its spec was generated from:
  * the library's ABI widths are the spec's, and the probe resolves the
    bus-index order (the v6 permutation: ankle rolls at bus slots 10/11);
  * closed loop on the CPU plant with shaping off: the firmware's MLP agrees
    with brax on the library's own obs to float precision, and end to end
    against the training obs stacking to the tick quantiser's size.

The run's artifacts are gitignored (sim/runs/, *.silw): absent, these SKIP
with what to regenerate -- for the v6legs test net,
`tools/make_test_policy_run.py --out v6legs_fullmap_testnet` then
`tools/export_policy_weights.py --run v6legs_fullmap_testnet --profile v6legs
--no-golden`.
"""
import json
import os
import re
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import policy_profile  # noqa: E402

PROFILES = policy_profile.list_profiles()

CHILD = r'''
import json, os, sys
import numpy as np
ROOT = sys.argv[1]; run = sys.argv[2]
sys.path[:0] = [os.path.join(ROOT, "sim"), os.path.join(ROOT, "sim", "mjx")]
import eval_precision as E
run_dir = os.path.join(ROOT, "sim", "runs", run)
cfg = json.load(open(os.path.join(run_dir, "config.json")))
xml = os.path.join(ROOT, "sim", cfg["xml_path"])
prof = E.sil_profile_for(xml)
H, lib, info = E.sil_setup(run, run_dir, verbose=False, profile=prof)
env = E.make_env(cfg, 12.0, True, xml)
E.sil_check_spec(H, env)
py = E.load_policy(run_dir, env.observation_space.shape[0],
                   env.action_space.shape[0])
lib.set_shaper(0.0)
ad = H.SilActAdapter(lib, env, py_act=py)
env.reset(seed=3)
env.set_command(0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
H.pin_gait_clock(env)
for _ in range(25):
    a = ad(env._obs())
    _, _, term, _, _ = env.step(a)
    if term:
        break
d = ad.divergence()
print("RESULT " + json.dumps(dict(
    profile=prof, spec=json.loads(lib.spec_string()), ticks=d["ticks"],
    in_order=lib.in_order, out_order=lib.out_order,
    net_err=d["net_err_max"], train_err=d["train_action_err_max"],
    q_err=d["obs_block_err_max"]["q"])))
'''


def _run_name(profile):
    txt = open(policy_profile.files(profile)["spec_h"]).read()
    return re.search(r'kRunName\s*=\s*"([^"]+)"', txt).group(1)


@pytest.mark.parametrize("profile", PROFILES)
def test_profile_closed_loop_parity(profile):
    run = _run_name(profile)
    run_dir = os.path.join(ROOT, "sim", "runs", run)
    gone = [f for f in ("config.json", "params.pkl")
            if not os.path.exists(os.path.join(run_dir, f))]
    if gone:
        pytest.skip(f"sim/runs/{run}/{{{','.join(gone)}}} absent "
                    "(gitignored); see this module's docstring")
    if not os.path.exists(os.path.join(HERE, "weights", run + ".silw")):
        pytest.skip(f"sim/sil/weights/{run}.silw absent (gitignored): "
                    f"tools/export_policy_weights.py --run {run} "
                    f"--profile {profile} --no-golden")
    env = dict(os.environ, BIMO_POLICY=profile, JAX_PLATFORMS="cpu")
    p = subprocess.run([sys.executable, "-c", CHILD, ROOT, run], env=env,
                       capture_output=True, text=True, timeout=600)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-4000:]
    line = [ln for ln in p.stdout.splitlines() if ln.startswith("RESULT ")]
    assert line, p.stdout[-2000:]
    r = json.loads(line[-1][len("RESULT "):])
    spec = r["spec"]
    assert r["profile"] == profile and spec["policy_profile"] == profile
    assert spec["obs_dim"] == spec["frame_dim"] * spec["hist_len"]
    assert spec["act_dim"] == spec["num_joints"]
    # the harness resolved the bus-index order, as on the deployed build
    assert r["in_order"] == r["out_order"] == "ascending_id"
    assert r["ticks"] >= 5
    # same obs -> same action: float32 MLP vs brax
    assert r["net_err"] < 1e-5, r
    # end to end vs the training obs: only the tick quantiser differs
    assert r["q_err"] < 2e-3, r
    assert r["train_err"] < 5e-2, r
