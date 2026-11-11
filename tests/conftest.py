"""pytest wiring for tests/ (the pre-push hook's link-tests gate).

JAX is pinned to the CPU before any test module imports it, as sim/sil's
conftest does for the SIL suite. The gates are CPU checks, and on the
training box a test that let JAX pick the GPU opened a CUDA context beside
the night campaign's run (AGENTS.md, "GPU BOUNDARY"). Subprocesses the tests
spawn inherit it. Most test modules set the same default themselves; this
covers the ones that do not. Set JAX_PLATFORMS explicitly to override.
"""
import os

os.environ.setdefault("JAX_PLATFORMS", "cpu")
# torch too (test_mac_train): its trainer runs on the CPU, but torch.accelerator
# still opens a CUDA context, which OOMs whenever ollama holds the 12 GB card
# (2026-11-03: 11.6 GB, all three mac_train tests errored at setup). The gates
# are CPU checks; set CUDA_VISIBLE_DEVICES explicitly to override.
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")


import random
import socket
import time


def free_udp_ports(n=4):
    """The first of `n` consecutive UDP ports that are free right now.

    Tests that spawn a twin and a console need ports they can name up front.
    Fixed ones collide when two pytest runs overlap (the pre-push hook in two
    worktrees at once, #148), and the twins bind with SO_REUSEADDR so a
    collision splits the packet stream instead of failing. The probe range is
    below the kernel's ephemeral range, so a socket bound to port 0 elsewhere
    cannot take one in between.
    """
    for _ in range(200):
        base = random.randrange(20000, 32000 - n)
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

