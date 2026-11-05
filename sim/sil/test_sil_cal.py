"""The committed SIL calibrations match the bus map compiled into the specs.

sim/sil/cal/**/cal_{nominal,perturbed}.json are what sim/sil_twin.py and
`eval_precision --sil` hand to sil_init(). The SIL suite never reads them (it
writes its own into a temp dir), so a bus-map change once left them listing
servo IDs the bus no longer has, and sil_init() refused them ("bus_id 5 is not
on the bus", #205). Each profile's committed files must equal what
harness.write_cal_files() generates from that profile's spec today; on a
mismatch, regenerate them:

    python -c "import sys; sys.path.insert(0, 'sim/sil'); \
import harness; harness.write_cal_files()"

(with BIMO_POLICY=<profile> for a policy profile's files).

harness.py binds one profile per process ($BIMO_POLICY, read at import), so
each profile is checked in a subprocess.
"""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import policy_profile  # noqa: E402

CHILD = r'''
import json, os, sys, tempfile
sys.path.insert(0, sys.argv[1])
import harness as H
out = {}
with tempfile.TemporaryDirectory() as tmp:
    for path in H.write_cal_files(tmp):
        name = os.path.basename(path)
        committed = os.path.join(H.CAL_DIR, name)
        out[name] = dict(fresh=json.load(open(path)),
                         committed=(json.load(open(committed))
                                    if os.path.exists(committed) else None),
                         path=os.path.relpath(committed, sys.argv[2]))
print(json.dumps(out))
'''


@pytest.mark.parametrize("profile",
                         [None] + policy_profile.list_profiles(),
                         ids=lambda p: p or "deployed")
def test_committed_cal_matches_bus_map(profile):
    env = dict(os.environ)
    env.pop("BIMO_POLICY", None)
    if profile:
        env["BIMO_POLICY"] = profile
    r = subprocess.run([sys.executable, "-c", CHILD, HERE, ROOT], env=env,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]
    files = json.loads(r.stdout.strip().splitlines()[-1])
    assert files
    for name, f in files.items():
        assert f["committed"] is not None, f"{f['path']} is missing"
        assert f["committed"]["bus_id"] == f["fresh"]["bus_id"], (
            f"{f['path']}: bus ids {f['committed']['bus_id']} are not the "
            f"spec's {f['fresh']['bus_id']} -- regenerate it "
            f"(harness.write_cal_files(), see this file's docstring)")
        assert f["committed"] == f["fresh"], (
            f"{f['path']} differs from harness.write_cal_files()'s output")
