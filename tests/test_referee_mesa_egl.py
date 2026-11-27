"""eval_precision.prefer_mesa_egl: the CPU referee renders on Mesa EGL, off the
shared NVIDIA GPU (11-27: a referee died with SIGSEGV in libnvidia-eglcore while
ollama held 11.6 of 12.3 GB).

Unit cases cover when it does and does not set the vendor; the subprocess case
checks that a MuJoCo renderer opened after the switch really is not NVIDIA's
(skipped where EGL or the Mesa vendor file is missing).
"""
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))

import eval_precision as E                          # noqa: E402

V = "__EGL_VENDOR_LIBRARY_FILENAMES"


@pytest.fixture
def vendor(tmp_path):
    f = tmp_path / "50_mesa.json"
    f.write_text("{}")
    return str(f)


def test_sets_mesa_under_egl(vendor):
    env = {"MUJOCO_GL": "egl"}
    assert E.prefer_mesa_egl(env, vendor) is True
    assert env[V] == vendor


@pytest.mark.parametrize("env", [
    {},                                                  # glfw / unset: untouched
    {"MUJOCO_GL": "osmesa"},
    {"MUJOCO_GL": "egl", "BIMO_REFEREE_EGL": "nvidia"},   # opt-out
    {"MUJOCO_GL": "egl", V: "/x/10_nvidia.json"},         # caller already chose
])
def test_leaves_env_alone(env, vendor):
    before = dict(env)
    assert E.prefer_mesa_egl(env, vendor) is False
    assert env == before


def test_missing_vendor_file_is_a_noop(tmp_path):
    env = {"MUJOCO_GL": "egl"}
    assert E.prefer_mesa_egl(env, str(tmp_path / "nope.json")) is False
    assert V not in env


def test_import_does_not_touch_env():
    # only `python eval_precision.py` switches; importers (tests, tools) keep theirs
    code = ("import os, sys; sys.path.insert(0, %r); os.environ['MUJOCO_GL']='egl';"
            "os.environ.pop(%r, None); import eval_precision;"
            "print(os.environ.get(%r, 'unset'))"
            % (os.path.join(HERE, "..", "sim", "mjx"), V, V))
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         env={**os.environ, "JAX_PLATFORMS": "cpu",
                              "CUDA_VISIBLE_DEVICES": ""}, timeout=300)
    assert out.returncode == 0, out.stderr[-2000:]
    assert out.stdout.strip().splitlines()[-1] == "unset"


_CHILD = """
import os, sys, runpy
script = sys.argv[1]
sys.argv = [script, "--help"]
try:
    runpy.run_path(script, run_name="__main__")
except SystemExit:
    pass
import mujoco
m = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
r = mujoco.Renderer(m, 64, 64)
r.update_scene(mujoco.MjData(m))
r.render()
from OpenGL import GL
print("GL_RENDERER", GL.glGetString(GL.GL_RENDERER).decode())
print("VENDOR", os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES"))
"""


@pytest.mark.skipif(not os.path.exists(E.MESA_EGL_VENDOR), reason="no Mesa EGL vendor")
def test_main_path_renders_off_nvidia():
    # run eval_precision as __main__ (argparse --help exits before any rollout),
    # then open a renderer in the same process: libEGL was loaded by the script's
    # own imports, so this sees the vendor the referee's --render would get.
    script = os.path.abspath(os.path.join(HERE, "..", "sim", "mjx", "eval_precision.py"))
    env = {k: v for k, v in os.environ.items() if k not in ("BIMO_REFEREE_EGL", V)}
    env.update(MUJOCO_GL="egl", JAX_PLATFORMS="cpu", CUDA_VISIBLE_DEVICES="")
    out = subprocess.run([sys.executable, "-c", _CHILD, script], capture_output=True,
                         text=True, env=env, cwd=os.path.dirname(script), timeout=300)
    if out.returncode != 0 and ("EGL" in out.stderr or "OpenGL" in out.stderr):
        pytest.skip("no usable EGL here: " + out.stderr.strip().splitlines()[-1][:200])
    assert out.returncode == 0, out.stderr[-2000:]
    assert "VENDOR " + E.MESA_EGL_VENDOR in out.stdout, out.stdout[-1000:]
    line = [ln for ln in out.stdout.splitlines() if ln.startswith("GL_RENDERER")][-1]
    assert "NVIDIA" not in line, line
