"""pytest wiring for tests/ (the pre-push hook's link-tests gate).

JAX is pinned to the CPU before any test module imports it, as sim/sil's
conftest does for the SIL suite. The gates are CPU checks, and on the
training box a test that let JAX pick the GPU opened a CUDA context beside
the night campaign's run (AGENTS.md, "GPU BOUNDARY"). Subprocesses the tests
spawn inherit it. Most test modules set the same default themselves; this
covers the ones that do not. Set JAX_PLATFORMS explicitly to override.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))   # `import paths`
os.environ.setdefault("JAX_PLATFORMS", "cpu")
# MuJoCo's GL likewise: on Linux an EGL context lands on the NVIDIA card even
# with CUDA hidden (CUDA_VISIBLE_DEVICES does not reach EGL). A CPU-only run
# (CUDA_VISIBLE_DEVICES unset or empty, which the line below makes the
# default) gets no GL at all, and the 3D-view tests skip. Set MUJOCO_GL
# (osmesa, or egl with MUJOCO_EGL_DEVICE_ID on a software device) to render.
if not os.environ.get("CUDA_VISIBLE_DEVICES") and "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "disabled"
# torch too (test_mac_train): its trainer runs on the CPU, but torch.accelerator
# still opens a CUDA context, which OOMs whenever ollama holds the 12 GB card
# (2026-11-03: 11.6 GB, all three mac_train tests errored at setup). The gates
# are CPU checks; set CUDA_VISIBLE_DEVICES explicitly to override.
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")


import random
import socket
import time


def _port_slice(span=12000, slots=16):
    """The [lo, hi) probe range of this pytest-xdist worker.

    Each worker (gw0, gw1, ...) draws from its own slice of 20000-32000, so two
    workers cannot pick the same block between the probe and the bind; a plain
    run (no xdist) draws from the whole range. Workers past `slots` share
    slices, which is as likely to collide as the unsliced draw was.
    """
    w = os.environ.get("PYTEST_XDIST_WORKER", "")
    if not (w.startswith("gw") and w[2:].isdigit()):
        return 20000, 20000 + span
    width = span // slots
    lo = 20000 + (int(w[2:]) % slots) * width
    return lo, lo + width


def free_udp_ports(n=4):
    """The first of `n` consecutive UDP ports that are free right now.

    Tests that spawn a twin and a console need ports they can name up front.
    Fixed ones collide when two pytest runs overlap (the pre-push hook in two
    worktrees at once, #148), and the twins bind with SO_REUSEADDR so a
    collision splits the packet stream instead of failing. Under pytest-xdist
    each worker probes its own slice of the range (_port_slice). The probe
    range is below the kernel's ephemeral range, so a socket bound to port 0
    elsewhere cannot take one in between.
    """
    lo, hi = _port_slice()
    for _ in range(200):
        base = random.randrange(lo, hi - n)
        socks = []
        try:
            for k in range(n):
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                socks.append(s)
                s.bind(("127.0.0.1", base + k))
            return base
        except OSError:
            continue
        finally:
            for s in socks:
                s.close()
    raise RuntimeError("no free UDP port block")



# --- pytest-xdist: tests that must share one worker (make pytest: --dist loadgroup) ---
# Test modules whose tests cannot run on two workers at once, by module name.
# Everything else is isolated per test (tmp_path, ephemeral or per-worker
# ports); the audit is in the PR for #329. With one process (PYTEST_WORKERS=0)
# the groups change nothing.
XDIST_GROUPS = {
    # fixed UDP ports 4491/4492 (arm_script --port/--tlm-port)
    "test_robot_prep": "fixed-udp-4491",
    # infra/night shell scripts: fake night dirs, but the scripts' own flock /
    # detached-runner behaviour is timing-sensitive; one worker runs them in turn
    "test_night_continue": "infra-night",
}


import pytest


# tryfirst: xdist's own modifyitems hook reads the xdist_group markers to tag
# each nodeid "@<group>"; a marker added after it is silently ignored and the
# group's tests scatter over the workers.
@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    for item in items:
        group = XDIST_GROUPS.get(item.module.__name__.rsplit(".", 1)[-1])
        if group:
            item.add_marker(pytest.mark.xdist_group(group))
