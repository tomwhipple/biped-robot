# RunPod burst compute

Rent a RunPod GPU (or CPU) pod from the CLI, run a job over SSH, tear it down.
This is burst capacity for MJX work when the nightly pipeline on mira is busy
or too small; mira is the default ([docs/training.md](../../docs/training.md) §7).

| file | what it is |
|---|---|
| `run_poc.sh` | the pod smoke test: create a pod, wait for SSH, copy and run `poc.py`, delete the pod on exit (trap) |
| `poc.py` | self-contained: device report, torch matmul benchmark, and with `--mjx` installs `mujoco-mjx` + `jax[cuda12]` and runs a batched physics-step benchmark |

## One-time setup

```bash
brew install runpod/runpodctl/runpodctl
runpodctl doctor            # prompts for the API key, saves ~/.runpod/config.toml
runpodctl ssh add-key       # registers ~/.runpod/ssh/runpodctl-ssh-key
runpodctl user              # check the account and spend limit
```

## Smoke-test a pod

```bash
export RUNPOD_API_KEY=$(python3 -c "import tomllib;print(tomllib.load(open('$HOME/.runpod/config.toml','rb'))['apikey'])")
./run_poc.sh gpu            # first secure GPU in the preference list that deploys
./run_poc.sh cpu            # CPU-only pod
./run_poc.sh gpu --keep     # leave the pod up to iterate on (delete it yourself)
```

The MJX number `poc.py` prints is for a toy two-body model: it proves MJX runs
on the pod, not how fast the robot's env trains. Benchmark the real env before
choosing a GPU.

## Training on a pod

Train from a **git clone of committed code**, exactly as the night pipeline
does: clone the repo on the pod, check out the pushed commit (or branch), and
run `sim/mjx/train_mjx.py` from it. Do not copy a hand-picked file list: it
silently misses files, a plant XML say, and the run trains a stale robot.
`train_mjx.py` stamps `git_sha` and `git_dirty` into the run's `config.json`,
which only means something when the tree is a clone.

- The pod needs `mujoco-mjx`, `jax[cuda12]` and `brax` on top of the image; a
  custom image or a `runpodctl template` saves the install on every launch.
- `--network-volume-id` (see `runpodctl network-volume`) keeps checkpoints
  across pods instead of on container disk.
- `--terminate-after` is the backstop against a forgotten pod; `run_poc.sh`
  sets it (2 h) in addition to its delete-on-exit trap. Set it on every pod.
- Bring `sim/runs/<run>/` back and referee it on the CPU like any other run
  ([docs/training.md](../../docs/training.md) §8).

## Gotchas (baked into the scripts)

1. **Gate readiness on SSH, not the pod API.** The `runtime`/`uptimeSeconds`
   fields stay `null`/`0` for minutes after the container is reachable. Poll
   `runpodctl ssh info` for the endpoint and then SSH itself, in one loop.
2. **The SSH endpoint is not assigned at create time.** Poll for the IP and
   port in the same readiness loop.
3. **Use the direct IP:port**, not the `ssh.runpod.io` proxy, which rejects
   the RSA key.
4. **macOS has no `timeout(1)`**, and SSH auth can hang after the TCP connect.
   The scripts wrap ssh in a background-kill watchdog (`ssh_t`).
5. **Secure-cloud GPU stock is thin.** `run_poc.sh` walks a preference list
   (RTX 4000 Ada → 4090 → A40 → 3090 → L40S) until one deploys.
6. **The RunPod pytorch image is PEP-668 externally managed**: `pip install`
   needs `--break-system-packages` (fine on a throwaway pod; `poc.py` does it).
7. **Run `runpodctl pod list` after every create.** A create loop with a
   fragile success check can start several pods at once; the list is the only
   truth about what is billing.
