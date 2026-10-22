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
   control and seed replicates. *(night 1: done — `robot_getup_t3k`)*
2. **Harden**: joint friction first (the policies are friction-specific,
   night 1), then pushes (`--push-prob`), then the residual scale; one at a
   time. *(night 2: friction DR)*
3. **One policy for both starts**: largely solved without env work (night
   1). The prone reference's roll ends on the back and the supine seat push
   follows, so the prone policy gets up from supine when it starts at that
   phase (`recover_supine_ph`); the robot picks the phase from the IMU.
   Left: a side start, and the deployment choosing the phase.
4. **Hand over to the walker** (the reference ends arms forward: a 2 s ramp
   today), distil, SIL — the deployment path of training.md §9.

Throughout: when the referee shows a phase failing, fix the reference first
(`getup_ref_analytic.py verify` / `rise` / `search-roll`), then the RL.

## Open questions

- *Answered 2026-10-22:* the servo assignment is the same everywhere — every
  run's `config.json` has `servo_types` hip rolls + knees = STS3250 (the
  `--robot` preset adds the hip rolls to `--servo-3250 knee`), which the
  referee inherits and the references record with (PJ_DEFAULT).
- 0.35 vs the measured 0.235: *partly answered 2026-10-22* — a policy trained
  at 0.35 fails at 0.05 (t3k), so tonight trains over 0.05 – 0.5
  (`--joint-frictionloss-range`). The friction sweep of the referee
  (`eval_precision.py --joint-frictionloss 0.05` / `0.5`) is now part of
  every morning's evaluation.
- Why did the supine retrains (t1k, t2k) lose the hold after standing? They
  rise to 0.39 m upright and fall 1 – 6 s after the env hands control from
  reference + residual to the free policy; MJX's training curve shows no
  sign of it. Not the plant (r5k holds at 0.35), not the referee's friction
  (they fail at 0.05 too), not servo lag (0/8 with the 2 Hz lag). Open; the
  prone recipe (`--w-rise-root 4`, 45 s episodes) holds.

## Runs

| run | night | parent (init) | the one variable | hypothesis | supine | prone (old start) | prone fall | verdict |
|---|---|---|---|---|---|---|---|---|
Scores are CPU-referee passes of 8 seeds at the run's own joint frictionloss
(0.35 unless stated). "supine ph" is `recover_supine_ph` (from 10-22; for
night 1 measured by hand with the same phase, 11.26 s).

| run | night | parent (init) | the one variable | hypothesis | supine | supine ph | prone (old start) | prone fall | verdict |
|---|---|---|---|---|---|---|---|---|---|
| robot_getup_t3k | 10-21 | s1k recipe (p13k) | prone ref → `getup_ref_v6ar_prone_ik.npz` | prone recovers at 0.35 on a feasible reference | 0/8 | 8/8 | 8/8 (36.8 s) | 8/8 (36.8 s) | **held**; s1k was 3/8. At 0.05: prone fall 6/8, supine ph 2/8; at 0.5: 7/8, 8/8 |
| robot_getup_t2k | 10-21 | r5k recipe + 0.35 (r5k) | supine ref → `getup_ref_v6ar_ik.npz` | the centred rise tracks at least as well | 1/8 | = supine | 0/8 | 0/8 | rise **held** (8/8 reach 0.39 m upright at ~22 s), then 7/8 fall within 1 – 6 s of the handover (shared with t1k) |
| robot_getup_t1k | 10-21 | r5k (r5k) | joint frictionloss 0.05 → 0.35 | control for t2k: r5k's supine survives retraining at 0.35 | 0/8 | = supine | 0/8 | 0/8 | **failed**: 6/8 stand at ~22 s then fall; 2/8 stall at 0.28 m. r5k itself, unretrained, scores 8/8 at 0.35 |
| robot_getup_t3k_s1 | 10-21 | t3k | seed 1 | replicate | 0/8 | 8/8 | 6/8 (34.4 s) | 7/8 (34.4 s) | replicate holds (13/16 vs 16/16) |
| robot_getup_t2k_s1 | 10-21 | t2k | seed 1 | replicate | 1/8 | = supine | 0/8 | 0/8 | replicates t2k; **capped at 392M steps** by the 05:00 cutoff |
| robot_getup_u1k | 10-22 | u0k (t3k) | + joint friction DR 0.05 – 0.5 | t3k's 0.05 weakness lifts to ≥ 7/8 (prone fall, supine ph), 0.35 stays ≥ 7/8 | | | | | |
| robot_getup_u0k | 10-22 | t3k (t3k) | init p13k → t3k (a second 600M at 0.35) | control for u1k: 0.35 stays 8/8, 0.05 stays weak | | | | | |
| robot_getup_u2k | 10-22 | t3k (p13k) | + joint friction DR 0.05 – 0.5 | DR from the 0.05-trained p13k matches t3k at 0.35 and holds 0.05 | | | | | |
| robot_getup_u1k_s1 | 10-22 | u1k | seed 1 | replicate | | | | | |
| robot_getup_u0k_s1 | 10-22 | u0k | seed 1 | replicate (may be capped at ~390M) | | | | | |

## Nights

*(the planner appends a "Night of YYYY-MM-DD" entry each morning: results,
what they mean, the decision, tonight's queue)*

### Night of 2026-10-21 (evaluated 2026-10-22)

All five jobs trained (600M steps each; t2k_s1 capped at 392M by the 05:00
cutoff) and refereed on main `b06c7c7`; no plant, servo or env change
landed upstream since, so they compare with the earlier get-up runs.

- **Prone: the feasible reference fixed it.** t3k 8/8 on both prone starts
  at 0.35 (s1k, the same recipe on the infeasible reference: 3/8);
  t3k_s1 6/8 and 7/8. Time to stand ~35 – 37 s.
- **Supine: the retrains broke the hold, not the rise.** Traced on the CPU
  referee (`--trace`): t1k, t2k and t2k_s1 rise to standing height at ~22 s
  in almost every seed, then fall 1 – 6 s after `recovered` hands control
  from reference + residual to the free policy. Counterfactuals: r5k,
  unretrained, scores 8/8 at 0.35 — the plant at 0.35 is not the problem;
  t1k and t2k score 0/8 at 0.05 too — the policies broke, not the
  conditions; with the 2 Hz servo lag still 0/8. The analytic rise is
  tracked fine (t2k reaches 0.39 m upright). Cause of the lost hold: open.
- **One policy for both starts.** The prone reference from 11.26 s is the
  supine reference from 0 s (the roll ends lying on the back, then the same
  seat push and rise). Started supine at that phase, t3k and t3k_s1 both get
  up 8/8 (24 s) and hold. A reference indexed from the episode start made
  this look like a separate problem (`recover_supine` 0/8). The referee now
  has it as `recover_supine_ph`.
- **t3k is friction-specific.** Refereed at 0.05: prone fall 6/8, supine ph
  2/8 (it stalls between the crouch hold and the rise, at 0.2 – 0.27 m,
  tipping back); at 0.5: 7/8 and 8/8. The real gearbox friction is not
  measured (0.05 BAM, ~0.235 powered, 0.35 assumed), so a get-up that only
  works at 0.35 is not a result.

**Decision.** Drop the supine line: r5k (0.05-trained) stays the supine
specialist, and t3k covers supine through the phase. Arc step 1 is done;
step 3 is largely done without env work. Step 2 starts with friction:
`--joint-frictionloss-range` (per-env joint friction DR, training only) and
`eval_precision.py --joint-frictionloss` (the friction sweep), PR #139.

**Tonight (night 2)**, all on the t3k recipe (prone_ik reference, 0.35
nominal):

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_getup_u1k | u0k (t3k) | + friction DR 0.05 – 0.5 | lifts 0.05 to ≥ 7/8, keeps 0.35 ≥ 7/8 |
| 2 | robot_getup_u0k | t3k (t3k) | init p13k → t3k | control: more steps alone do not fix 0.05 |
| 3 | robot_getup_u2k | t3k (p13k) | + friction DR 0.05 – 0.5 | DR from p13k matches t3k at 0.35 and holds 0.05 |
| 4 | robot_getup_u1k_s1 | u1k | seed 1 | replicate |
| 5 | robot_getup_u0k_s1 | u0k | seed 1 | replicate (likely capped) |

**Tomorrow's evaluation** must referee each run at 0.05 and 0.5 as well as
its own 0.35: `cd ~/code/robot && .venv/bin/python sim/mjx/eval_precision.py
--run-name <run> --scenarios recover_supine_ph,recover_prone_fall
--joint-frictionloss 0.05` (and `0.5`) — CPU, ~10 min per run and value,
at most 12 workers in parallel. Baselines: t3k 0.05 → 2/8, 6/8; 0.5 → 8/8,
7/8 (supine ph, prone fall).
