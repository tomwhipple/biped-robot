"""The sway-probe sender's frames through the robot's REAL Wi-Fi CLI session logic: firmware/host/build/wcli_twin
(components/wcli: authentication, at-most-once, the allowlist) on a loopback UDP socket, driven by the sender's own
`Robot` client. Loopback only. Skips with a reason when the twin is not built (make -C firmware/host build/wcli_twin)."""
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "experiments", "sway-probe"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "link"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

import sway_probe as SP  # noqa: E402
import sway_ref as SR  # noqa: E402

TWIN = os.path.join(ROOT, "firmware", "host", "build", "wcli_twin")
pytestmark = pytest.mark.skipif(not os.access(TWIN, os.X_OK), reason="firmware/host/build/wcli_twin not built "
                                "(make -C firmware/host build/wcli_twin)")


def test_every_line_of_a_rung_is_accepted_by_the_cli_session(tmp_path):
    key = os.urandom(32)
    kf = tmp_path / "wcli.key"
    kf.write_text(key.hex() + "\n")
    os.chmod(kf, 0o600)
    proc = subprocess.Popen([TWIN, "--key-hex", key.hex(), "--port", "0"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        line = proc.stdout.readline()
        assert line.startswith("listening "), line
        port = int(line.split()[1])
        robot = SP.Robot("127.0.0.1", str(kf), port)
        ref = SR.SwayReference()
        rung = ref.rung(9, "lift", 0.039, cycles=1, ramp_s=0.5)
        cal = SP.illustrative_cal()
        lines = [SP.zero_line(cal)] + [s.line() for s in SP.plan(rung, cal, 200)] + \
                [s.line() for s in SP.plan(rung, cal, 200, balance="stance")[:3]]
        for ln in lines:
            r = robot.run(ln)
            assert r.startswith("pose: streaming"), (ln, r)         # the twin's fake CLI answers a pose line this way
    finally:
        proc.terminate()
        proc.wait(timeout=5)
