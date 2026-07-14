#!/usr/bin/env python3
"""RunPod proof-of-concept benchmark.

Self-contained. Runs on either a CPU pod or a GPU pod with the RunPod
pytorch image (torch + CUDA pre-baked). Reports the device, runs a matmul
GFLOPS benchmark, and -- if MJX is importable -- a batched physics-step
benchmark that mirrors the real training workload (compare vs Mira's
~639k steps/s @ batch 4096 on the RTX 4070 Ti).

Usage:
    python3 poc.py            # torch matmul only (fast)
    python3 poc.py --mjx      # also install+run the MJX step benchmark
"""
import argparse
import platform
import subprocess
import sys
import time


def hr(title):
    print(f"\n{'=' * 8} {title} {'=' * 8}", flush=True)


def sysinfo():
    hr("SYSTEM")
    print(f"python   : {platform.python_version()}")
    print(f"platform : {platform.platform()}")
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.total,driver_version",
             "--format=csv,noheader"],
            capture_output=True, text=True, timeout=15)
        gpu = out.stdout.strip() or out.stderr.strip()
        print(f"nvidia-smi: {gpu or '(none)'}")
    except Exception as e:  # noqa: BLE001
        print(f"nvidia-smi: unavailable ({e})")


def torch_bench():
    hr("TORCH MATMUL")
    import torch
    print(f"torch    : {torch.__version__}")
    cuda = torch.cuda.is_available()
    print(f"cuda     : {cuda}")
    dev = "cuda" if cuda else "cpu"
    if cuda:
        print(f"device   : {torch.cuda.get_device_name(0)}")

    n = 8192 if cuda else 4096
    dtype = torch.float32
    a = torch.randn(n, n, device=dev, dtype=dtype)
    b = torch.randn(n, n, device=dev, dtype=dtype)

    # warmup
    for _ in range(3):
        c = a @ b
    if cuda:
        torch.cuda.synchronize()

    iters = 20
    t0 = time.perf_counter()
    for _ in range(iters):
        c = a @ b
    if cuda:
        torch.cuda.synchronize()
    dt = (time.perf_counter() - t0) / iters
    flops = 2 * n ** 3
    print(f"matmul   : {n}x{n} fp32, {dt * 1e3:.2f} ms/iter, "
          f"{flops / dt / 1e12:.2f} TFLOP/s")
    _ = float(c[0, 0])  # force realize
    return dev


def mjx_bench():
    hr("MJX STEP BENCHMARK")
    try:
        import jax
    except ImportError:
        print("installing jax[cuda] + mujoco-mjx ...", flush=True)
        subprocess.run([sys.executable, "-m", "pip", "install", "-q",
                        "--break-system-packages",
                        "mujoco-mjx", "jax[cuda12]"], check=True)
        import jax  # noqa: F811
    import jax.numpy as jp
    import mujoco
    from mujoco import mjx

    print(f"jax      : {jax.__version__}")
    print(f"backend  : {jax.default_backend()}")
    print(f"devices  : {jax.devices()}")

    # Minimal free-floating pendulum-ish model; enough to exercise the pipeline.
    xml = """
    <mujoco>
      <option timestep="0.004"/>
      <worldbody>
        <body pos="0 0 1">
          <joint type="free"/>
          <geom type="capsule" size="0.05 0.3" density="1000"/>
          <body pos="0 0 -0.4">
            <joint name="hinge" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.25" density="1000"/>
          </body>
        </body>
      </worldbody>
      <actuator><motor joint="hinge"/></actuator>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    mx = mjx.put_model(m)

    batch = 4096
    dx = jax.vmap(lambda _: mjx.make_data(mx))(jp.arange(batch))

    @jax.jit
    def step_n(dx, n):
        def body(i, dx):
            return jax.vmap(mjx.step, in_axes=(None, 0))(mx, dx)
        return jax.lax.fori_loop(0, n, body, dx)

    n_sub = 50
    dx = step_n(dx, n_sub)  # compile + warmup
    jax.block_until_ready(dx.qpos)

    iters = 5
    t0 = time.perf_counter()
    for _ in range(iters):
        dx = step_n(dx, n_sub)
    jax.block_until_ready(dx.qpos)
    dt = (time.perf_counter() - t0) / iters
    steps_per_s = batch * n_sub / dt
    print(f"batch    : {batch}, {n_sub} substeps/iter")
    print(f"throughput: {steps_per_s / 1e3:.0f}k physics-steps/s "
          f"(Mira ref: 639k)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjx", action="store_true",
                    help="also run the MJX batched-step benchmark")
    args = ap.parse_args()

    sysinfo()
    torch_bench()
    if args.mjx:
        try:
            mjx_bench()
        except Exception as e:  # noqa: BLE001
            print(f"MJX benchmark failed: {e}")
    hr("DONE")


if __name__ == "__main__":
    main()
