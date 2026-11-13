"""tools/robot_prep.sh: per-boot settings, taken from a run's config.json and
verified by reading them back -- against a stub Wi-Fi CLI client (WCLI), never
a robot.

Run:  .venv/bin/python -m pytest tests/test_robot_prep.py -q
"""
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREP = os.path.join(ROOT, "tools", "robot_prep.sh")
ARM = os.path.join(ROOT, "link", "arm_script.py")

# A robot's `shape` / `standclock` as firmware/main/cli.cpp words them, with
# state in a file. STUCK=shape makes `shape <hz>` a no-op (a set that did not
# take); BOOT_SHAPE/BOOT_CLOCK are the power-up state.
STUB = r'''#!{py}
import os, sys
a = [x for x in sys.argv[1:]]
if a[:1] == ["--host"]:
    a = a[2:]
st = os.environ["STUB_STATE"]
try:
    shape, clock, phase = open(st).read().split()
except OSError:
    shape, clock, phase = os.environ.get("BOOT_SHAPE", "10"), "run", "0"
c = a[0] if a else ""
if c == "stat":
    print("policy: ok\nbus: 17 servos")
elif c == "shape":
    if len(a) > 1 and os.environ.get("STUCK") != "shape":
        shape = a[1]
    h = float(shape)
    print("shape: off (raw targets, max-speed slew)" if h == 0 else
          "shape: C2 pole %.2f Hz (3 cascaded lags) + per-joint goal speed" % h)
elif c == "standclock":
    if len(a) > 1:
        clock = a[1]
        if len(a) > 2:
            phase = a[2]
    if os.environ.get("FREEZE_BUILD"):
        print("standclock: this policy was trained with the clock HELD at a stand -- it always holds (option ignored)")
    elif clock == "coast":
        print("standclock: coast -- at a plain stand the clock keeps 1.50 Hz until phase %.3f rad, then holds" % float(phase))
    else:
        print("standclock: %s -- at a plain stand ..." % clock)
open(st, "w").write("%s %s %s" % (shape, clock, phase))
'''


@pytest.fixture
def env(tmp_path):
    stub = tmp_path / "wcli_stub"
    stub.write_text(STUB.format(py=sys.executable))
    stub.chmod(0o755)
    # PYTHON: robot_prep.sh defaults to $ROOT/.venv/bin/python, which a fresh
    # worktree does not have; use the interpreter running the tests.
    e = dict(os.environ, WCLI=str(stub), STUB_STATE=str(tmp_path / "state"),
             PYTHON=sys.executable)
    e.pop("STUCK", None)
    return e


def run_cfg(tmp_path, **cfg):
    d = tmp_path / "run_x"
    d.mkdir(exist_ok=True)
    (d / "config.json").write_text(json.dumps(cfg))
    return str(d)


def prep(env, run, *args):
    return subprocess.run([PREP, "--run", run, *args], env=env,
                          capture_output=True, text=True, timeout=60)


def test_sets_and_verifies_coast_and_shape_from_the_config(env, tmp_path):
    run = run_cfg(tmp_path, fw_shaper_hz=8.0, clock_stand_coast=1.5708)
    r = prep(env, run)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "OK: standclock coast 1.5708, shape 8 Hz" in r.stdout
    assert open(env["STUB_STATE"]).read() == "8 coast 1.5708"


def test_shaper_band_uses_its_middle_and_no_shaper_means_off(env, tmp_path):
    r = prep(env, run_cfg(tmp_path, fw_shaper_hz=6.0, fw_shaper_hz_max=10.0))
    assert r.returncode == 0 and "shape 8 Hz" in r.stdout and "standclock run" in r.stdout
    r = prep(env, run_cfg(tmp_path))          # the deployed distill run has no key
    assert r.returncode == 0 and "shape 0 Hz" in r.stdout
    assert open(env["STUB_STATE"]).read().split()[0] == "0"


def test_a_shape_that_does_not_read_back_is_exit_4(env, tmp_path):
    env["STUCK"] = "shape"
    r = prep(env, run_cfg(tmp_path, fw_shaper_hz=8.0, clock_stand_coast=1.5708))
    assert r.returncode == 4 and "did not read back" in r.stderr


def test_check_sets_nothing_and_flags_a_mismatch(env, tmp_path):
    run = run_cfg(tmp_path, fw_shaper_hz=8.0, clock_stand_coast=1.5708)
    r = prep(env, run, "--check")             # boot state: shape 10, clock run
    assert r.returncode == 3 and "standclock" in r.stderr
    assert not os.path.exists(env["STUB_STATE"]) or \
        open(env["STUB_STATE"]).read() == "10 run 0"
    assert prep(env, run).returncode == 0
    r = prep(env, run, "--check")
    assert r.returncode == 0 and "checked, nothing set" in r.stdout
    env["BOOT_SHAPE"] = "10"
    open(env["STUB_STATE"], "w").write("10 coast 1.5708")
    assert prep(env, run, "--check").returncode == 4


def test_freeze_run_needs_nothing_set(env, tmp_path):
    run = run_cfg(tmp_path, fw_shaper_hz=10.0, clock_stand_freeze=True)
    env["FREEZE_BUILD"] = "1"
    assert prep(env, run).returncode == 0
    del env["FREEZE_BUILD"]
    assert prep(env, run, "--check").returncode == 3   # firmware for another run


def test_dry_run_sends_nothing_and_bad_config_is_exit_5(env, tmp_path):
    r = prep(env, run_cfg(tmp_path, fw_shaper_hz=8.0), "--dry-run")
    assert r.returncode == 0 and "would send: shape 8" in r.stdout
    assert not os.path.exists(env["STUB_STATE"])
    assert prep(env, str(tmp_path / "nope")).returncode == 5


def test_arm_script_refuses_to_arm_on_a_mismatch(env, tmp_path):
    run = run_cfg(tmp_path, fw_shaper_hz=8.0, clock_stand_coast=1.5708)
    base = [sys.executable, ARM, "--host", "127.0.0.1", "--port", "4491",
            "--tlm-port", "4492", "--phase", "arm", "--duration", "0.2"]
    r = subprocess.run(base + ["--check-prep", run], env=env,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 2 and "not arming" in r.stdout
    assert "== pre-arm" not in r.stdout       # no frame was ever sent
    assert prep(env, run).returncode == 0
    r = subprocess.run(base + ["--check-prep", run], env=env,
                       capture_output=True, text=True, timeout=60)
    assert "not arming" not in r.stdout and "== pre-arm" in r.stdout
