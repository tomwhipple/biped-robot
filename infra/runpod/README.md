# RunPod burst-compute workflow

Rent a RunPod GPU (or CPU) pod from the CLI, run a job over SSH, tear it down.
Burst capacity for MJX training when [Mira](../../DESIGN.md) (the RTX 4070 Ti
box) is busy or too small. **Verified end-to-end 2026-07-14** on both a CPU pod
and an A40 GPU pod.

## One-time setup (already done for this account)

```bash
brew install runpod/runpodctl/runpodctl        # -> runpodctl 2.7.0
runpodctl doctor                               # prompts for API key, saves to ~/.runpod/config.toml
runpodctl ssh add-key                          # registers ~/.runpod/ssh/runpodctl-ssh-key
```

Account funded (~$9.8 balance, $80/hr spend limit). Verify with `runpodctl user`.

## Run the PoC

```bash
export RUNPOD_API_KEY=$(python3 -c "import tomllib;print(tomllib.load(open('$HOME/.runpod/config.toml','rb'))['apikey'])")
./run_poc.sh gpu          # cheapest available secure GPU; torch matmul + MJX step bench
./run_poc.sh cpu          # CPU-only pod; torch matmul bench
./run_poc.sh gpu --keep   # leave the pod up for iterating (delete it yourself!)
```

`run_poc.sh` creates the pod, waits for SSH, copies `poc.py`, runs it, and
**always deletes the pod on exit** (trap). `poc.py` is self-contained: prints
device info, runs a matmul GFLOPS benchmark, and on `--mjx` installs
`mujoco-mjx` + `jax[cuda12]` and runs a batched physics-step benchmark.

## Verified results (2026-07-14)

| Pod            | $/hr  | torch fp32 matmul | MJX batched steps (toy 2-body model) |
|----------------|-------|-------------------|--------------------------------------|
| CPU (9 vCPU)   | 0.06  | 0.09 TFLOP/s      | n/a                                  |
| A40 (secure)   | 0.44  | 23.5 TFLOP/s      | ~15M steps/s @ batch 4096            |

Whole PoC (several pods, incl. the fumbles below) cost **~$0.12**. The MJX
number is a *toy* 2-body model, **not** comparable to Mira's 639k steps/s on the
real v2 robot — it only proves MJX-on-GPU runs on RunPod. Benchmark the real env
before trusting any GPU-choice math.

## Gotchas (learned the hard way — baked into the scripts)

1. **The pod API's `runtime`/`uptimeSeconds` fields lie.** They stay
   `null`/`0` for 10+ min after the container is actually SSH-reachable. Do
   **not** gate readiness on them — poll SSH (and `ssh info` for the endpoint)
   directly. This cost the most debugging time.
2. **`ssh info` ip/port aren't assigned instantly.** Poll for them in the same
   readiness loop (CPU pod took ~75s, GPU ~30s to become reachable).
3. **Use the direct-IP endpoint**, not the `ssh.runpod.io` proxy — the proxy
   rejected our RSA key (`Permission denied (publickey)`); direct IP:port works.
4. **macOS has no `timeout(1)`.** SSH auth can hang indefinitely (TCP connects,
   auth stalls); `ConnectTimeout` only covers the TCP connect. The scripts wrap
   ssh in a background-kill watchdog (`ssh_t`).
5. **Secure-cloud GPU stock is thin** ("Low" almost everywhere). `run_poc.sh`
   tries a preference list (4000 Ada → 4090 → A40 → 3090 → L40S) until one
   deploys. A5000 (the $0.27/hr one from the first funding test) was out of the
   catalog entirely this run.
6. **The runpod pytorch image is PEP-668 externally-managed.** pip needs
   `--break-system-packages` (fine on a throwaway pod) — baked into `poc.py`.
7. **Don't batch pod creates in a shell loop with a fragile success grep.** An
   early version created 3 pods at once because the break check missed the id in
   `head -3`. Always `runpodctl pod list` after creating, and the trap cleans up.

## Scaling to a real training run

- Use `--network-volume-id` (`runpodctl network-volume`) to persist checkpoints
  across pods instead of container disk.
- Bake deps into a custom image or a `runpodctl template` so you skip the
  ~2-min `pip install mujoco-mjx jax[cuda12]` each launch.
- `--terminate-after` / `--stop-after` are cheap insurance against a forgotten
  pod (the scripts set terminate-after as a backstop to the trap).
- For long runs prefer a bigger GPU from the pref list (L40S/4090/A100); the
  A40 here was just the cheapest available at test time.
