# Get-up campaign: analytic references, then RL (from 2026-10-21)

> Dated campaign record and the memory of the nightly loop: the morning planner
> (`infra/night/plan_morning.sh`) reads this first and appends to it every
> morning. Status lives in issue #137. Simulation only.

Tom, 2026-10-21: *"Plan another run, focusing on getup. Put together some
analytic kinematic solutions to get started and then fine-tune/make robust
with RL. In the morning, re-evaluate and plan the next run. There should be
simulations running all night, every night, always pulling in the latest
knowledge."*

## Where it starts

- The method is the one [training.md](../training.md) §11 prescribes for an RL
  get-up: phase-indexed tracking (DeepMimic) of a recorded reference at
  dominant weight, reference-state initialisation (RSI) along it, residual
  actions around it (`--rise-ref`, `--w-rise-ref 8`, `--rise-ref-rsi 0.7`,
  `--rise-ref-residual 0.3`, `--rise-ref-et 0.1`). Reward shaping alone stalled
  at kneel → stand over 12 rounds; do not fund shaping-only runs.
- Best policies before the campaign, all at joint frictionloss 0.05:
  `robot_getup_r5k` supine 8/8 (up in 20 s); `robot_getup_p13k` / `p14k` prone
  7/8 and 8/8 (15/16 each on 16 seeds). No policy passes both: a reference is
  indexed by time from the episode start, so a prone policy replays the roll
  from supine and fails (0/8). At 0.35, `p13k` unretrained still does prone
  7/8, while the retrains `s1k` 3/8 and `s1kb` 1/8 got worse.
- **Why s1k/s1kb got worse (2026-10-21):** the prone reference is infeasible
  at 0.35. Replayed open loop on the CPU plant, the recorded prone get-up
  (`prone_roll()` + seat push) stands at joint frictionloss 0 and 0.05 and
  fails at 0.35: the roll never reaches the side. They trained toward an
  impossible target. The supine seat push stands open loop at 0, 0.05 and
  0.35, and over the whole robustness grid (below).
- 0.35 N·m is the sim's assumed unpowered backdrive friction, not a
  measurement (servo-map.md §5); measured powered friction is ~0.235. The
  references are therefore verified across 0.05 – 0.5, not at one value.

## The analytic references (`sim/getup_ref_analytic.py`)

Every reference is checked open loop over a robustness grid of 216 rollouts:
joint frictionloss 0.05/0.35/0.5 × sole μ 0.5/0.7/1.0 × play 1/4° × servo
stall 85/100 % × mass 100/110 % × start tilt −10/0/+10°. References record
at frictionloss 0.35, μ 0.7, play 3° (the `--joint-frictionloss 0.35` plant).

| reference | start | content | grid | stands at |
|---|---|---|---|---|
| `getup_ref_v6ar.npz` (existing) | supine, arms 180° | seat push, hand-tuned rise | 216/216 | ~23 s |
| `getup_ref_v6ar_ik.npz` | supine, arms 180° | seat push to the crouch hold, then the analytic rise | 216/216 | ~25 s |
| `getup_ref_v6ar_prone.npz` (existing) | prone, arms 180° | `prone_roll()` + seat push | fails at 0.35 | — |
| `getup_ref_v6ar_prone_ik.npz` | prone, arms idle (+15°) | searched roll + the analytic seat push | 216/216 | ~33 s |

**The analytic rise.** Grid every symmetric sagittal pose (hip pitch, knee,
ankle at 3°) with both soles levelled on the floor; keep the 2474 whose
whole-body CoM lies within 12 mm of the sole centre with every body clear of
the floor; band them by pelvis height between the seat push's crouch hold
(0.235 m, CoM 48 mm behind the sole centre) and standing (0.387 m); pick a
path through the bands by dynamic programming on the worst per-leg gravity
torque / stall plus joint travel. Result: eight waypoints, CoM within ±11 mm
of the sole centre all the way up, static torque ≤ 0.14 of stall (the
hand-tuned rise swings the CoM from −48 to +22 mm and back to −27 mm). The
static torque is low throughout, so the get-up's torque margin problem is
dynamic (the push), not the rise's statics. Pace: 40°/s tipped it over
backwards on reaching straight at frictionloss 0.05; 20°/s with 0.4 s holds
stands everywhere.

**The prone roll.** The 2026-09-29 search's recommended roll (branch
`sims-overload-prone-current`, #126; 12/12 on the 09-29 plant) scores 0 % on
the current plant. Re-searched with the same method — CEM over five free leg
keyframes plus a stowed arm pose, from prone with the arms at the walk's idle
pose (where a forward fall leaves them), scored on the worst of seven
conditions that include frictionloss 0.05 and 0.5 — seeded from that roll.
Four restarts × 25 iterations × 64 candidates (two seeded from the 09-29
roll, two from scratch): every restart found a roll scoring ≥ +1.98 of the
~2.0 maximum (on the back with both arms within 15° of 180° in all seven
conditions), so robust rolls are common on this plant once friction is in the
conditions. The winner stows both arms overhead (L 190°, R 188°) and lands on
the back at ~11 s; its parameters are `sim/getup_ref_analytic_roll.json`. With
the seat push and the analytic rise it stands 216/216.

The referee's `recover_prone` starts from the OLD prone reference (arms
folded at 180° before the fall, which a real fall never does). The new
reference starts arms-idle, so it gets its own scenario,
`recover_prone_fall` (the new reference's first frame, 45 s, stand within
41 s), in the get-up family. The `recover_prone` column stays for comparison. Unretrained,
`robot_getup_p13k` (frictionloss 0.05) passes `recover_prone_fall` 2/2 in a
2-seed smoke test, standing at the same 32.8 s as from `recover_prone`: its
own reference drives the arms to 180° in free air during the first second, so
the two starts converge before the roll.

## Campaign arc

1. **Specialists at 0.35 on feasible references** (supine, prone), with a
   control and seed replicates. *(night 1)*
2. **Harden**: the best specialist under the claim's DR, then pushes
   (`--push-prob`), then the residual scale; one at a time.
3. **One policy for both starts**: the env indexes one reference by time
   from the episode start; a unified get-up needs the reference chosen per
   start (supine / prone / side) and the phase made observable. Env work
   (MJX `env_mjx.py` and its CPU mirror `walker_env.py`).
4. **Hand over to the walker** (the reference ends arms forward: a 2 s ramp
   today), distil, SIL — the deployment path of training.md §9.

Throughout: when the referee shows a phase failing, fix the reference first
(`getup_ref_analytic.py verify` / `rise` / `search-roll`), then the RL.

## Open questions

- The recipe passes `--servo-3250 knee`, but the plant's `SERVO_PLAN=hips`
  also puts STS3250s at the hip rolls (getup_v6.PJ_DEFAULT). Is the MJX
  training plant's servo assignment the same as the referee's?
- 0.35 vs the measured 0.235: if the references stay robust across the range,
  is friction DR the better training condition than one value?

## Runs

| run | night | parent (init) | the one variable | hypothesis | supine | prone (old start) | prone fall | verdict |
|---|---|---|---|---|---|---|---|---|
| robot_getup_t3k | 10-21 | s1k recipe (p13k) | prone ref → `getup_ref_v6ar_prone_ik.npz` | prone recovers at 0.35 on a feasible reference | | | | |
| robot_getup_t2k | 10-21 | r5k recipe + 0.35 (r5k) | supine ref → `getup_ref_v6ar_ik.npz` | the centred rise tracks at least as well | | | | |
| robot_getup_t1k | 10-21 | r5k (r5k) | joint frictionloss 0.05 → 0.35 | control for t2k: r5k's supine survives retraining at 0.35 | | | | |
| robot_getup_t3k_s1 | 10-21 | t3k | seed 1 | replicate | | | | |
| robot_getup_t2k_s1 | 10-21 | t2k | seed 1 | replicate | | | | |

## Nights

*(the planner appends a "Night of YYYY-MM-DD" entry each morning: results,
what they mean, the decision, tonight's queue)*
