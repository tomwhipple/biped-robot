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
| `getup_ref_v6ar_prone_ikc.npz` | prone, arms idle (+15°) | as prone_ik, centred crouch hold (below) | 216/216 | ~33 s |

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

**The centred crouch hold (2026-10-24).** The seat push lifts the hands off
the floor at its crouch hold (push pose → REST, shoulder 0 → 60, ankle −25):
in that crouch the REST arms put the CoM 48 mm behind the sole centre (sole
half-length 65 mm), the worst arm pose there. `prone_ikc` / `supine_ikc`
(`seat_push_ik(centred=True)`) keep the hands down and the ankle at −35
through the hold (CoM −12 mm) and move the arms to REST during the first rise
waypoint; same keys and timing, so the referee's starts and phases are
unchanged.

**Grading a reference with no RL.** `eval_precision.py --ref-replay <npz>`
replays a reference through the CPU referee (the run's plant and obs, every
action 0, hold on). The open-loop grid above has no per-body mass, gain or
latency DR; the referee does. Supine ph + prone fall at joint frictionloss
0.05 / 0.35 / 0.5: `prone_ik` 11/16, 14/16, 16/16 = **41/48** (its crouch tips
back under seeds 7/607/707); `prone_ikc` 11/16, 16/16, 16/16 = **43/48** (no
crouch failure left; the 0.05 misses stand and then fall backwards after
straightening, which needs feedback). A fine-tune has to beat its reference's
replay to have added anything.

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
   time. *(night 2: friction DR — confounded by the handover, see 10-23;
   night 3: `--rise-ref-hold`, with and without friction DR; night 4: the
   centred crouch hold — held, w0k 46/48; night 5: friction DR (dropped,
   third loss) and the residual scale — w2k 0.15 47/48; night 6: its seed
   spread and 0.075)*
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
- Why did the supine retrains (t1k, t2k) lose the hold after standing?
  *Answered 2026-10-23: the handover itself.* At `recovered` (0.5 s held
  stand) the env switched the action map from `ref + 0.3·a` to
  `default + span·a` and stopped paying the reference tracking (w 8 + 4 per
  step). The same output then means a different target, and the reward
  falls off a cliff at the latch, so longer training drifts toward falling
  after it or never latching (parking at ~0.29 m). MJX's curve does show it:
  `eval/episode_recovered` halves (t3k ~800 → u0k ~340) while
  `eval/episode_reward` stays flat at ~18.5k. Graded with the handover
  removed (`eval_precision.py --rise-ref-hold`, no retraining) u0k goes from
  0/16 to 16/16. `--rise-ref-hold` (PR #140) trains that way.

## Runs

Scores are CPU-referee passes of 8 seeds at the run's own joint frictionloss
(0.35 unless stated). "supine ph" is `recover_supine_ph` (from 10-22; for
night 1 measured by hand with the same phase, 11.26 s).

| run | night | parent (init) | the one variable | hypothesis | supine | supine ph | prone (old start) | prone fall | verdict |
|---|---|---|---|---|---|---|---|---|---|
| robot_getup_t3k | 10-21 | s1k recipe (p13k) | prone ref → `getup_ref_v6ar_prone_ik.npz` | prone recovers at 0.35 on a feasible reference | 0/8 | 8/8 | 8/8 (36.8 s) | 8/8 (36.8 s) | **held**; s1k was 3/8. At 0.05: prone fall 6/8, supine ph 2/8; at 0.5: 7/8, 8/8 |
| robot_getup_t2k | 10-21 | r5k recipe + 0.35 (r5k) | supine ref → `getup_ref_v6ar_ik.npz` | the centred rise tracks at least as well | 1/8 | = supine | 0/8 | 0/8 | rise **held** (8/8 reach 0.39 m upright at ~22 s), then 7/8 fall within 1 – 6 s of the handover (shared with t1k) |
| robot_getup_t1k | 10-21 | r5k (r5k) | joint frictionloss 0.05 → 0.35 | control for t2k: r5k's supine survives retraining at 0.35 | 0/8 | = supine | 0/8 | 0/8 | **failed**: 6/8 stand at ~22 s then fall; 2/8 stall at 0.28 m. r5k itself, unretrained, scores 8/8 at 0.35 |
| robot_getup_t3k_s1 | 10-21 | t3k | seed 1 | replicate | 0/8 | 8/8 | 6/8 (34.4 s) | 7/8 (34.4 s) | replicate holds (13/16 vs 16/16). At 0.05: prone fall 2/8, supine ph 0/8; at 0.5: 7/8, 8/8 |
| robot_getup_t2k_s1 | 10-21 | t2k | seed 1 | replicate | 1/8 | = supine | 0/8 | 0/8 | replicates t2k; **capped at 392M steps** by the 05:00 cutoff |
| robot_getup_u1k | 10-22 | u0k (t3k) | + joint friction DR 0.05 – 0.5 | t3k's 0.05 weakness lifts to ≥ 7/8 (prone fall, supine ph), 0.35 stays ≥ 7/8 | 0/8 | 0/8 | 0/8 | 0/8 | **failed** (stands, falls after the latch). 0.05: 0/8, 1/8; 0.5: 0/8, 0/8. *With hold* (sph, pf) 0.05: 3/8, 5/8; 0.35: 1/8, 4/8; 0.5: 2/8, 5/8 = 20/48 |
| robot_getup_u0k | 10-22 | t3k (t3k) | init p13k → t3k (a second 600M at 0.35) | control for u1k: 0.35 stays 8/8, 0.05 stays weak | 0/8 | 0/8 | 0/8 | 0/8 | **failed** as refereed (stands, falls after the latch; 0.05: 0/8, 0/8; 0.5: 2/8, 0/8). *With hold*: 0.05: 7/8, 8/8; 0.35: 8/8, 8/8; 0.5: 8/8, 7/8 = **46/48**, the most friction-robust get-up so far |
| robot_getup_u2k | 10-22 | t3k (p13k) | + joint friction DR 0.05 – 0.5 | DR from the 0.05-trained p13k matches t3k at 0.35 and holds 0.05 | 0/8 | 5/8 | 0/8 | 0/8 | **failed** from prone: parks at 0.29 m in every seed. Supine ph flat over friction: 0.05 5/8, 0.5 3/8; with hold 5/8 |
| robot_getup_u1k_s1 | 10-22 | u1k | seed 1 | replicate | 0/8 | 1/8 | 1/8 | 1/8 | as u1k. 0.05: 0/8, 0/8; 0.5: 0/8, 2/8. *With hold* 0.05: 5/8, 6/8; 0.35: 2/8, 7/8; 0.5: 4/8, 7/8 = 31/48 |
| robot_getup_u0k_s1 | 10-22 | u0k | seed 1 | replicate (may be capped at ~390M) | 0/8 | 0/8 | 0/8 | 0/8 | trained the full 600M (night started 20:24). Parks at 0.28 m from prone; 0/8 everywhere, with hold too |
| robot_getup_v0k | 10-23 | u0k (t3k) | + `--rise-ref-hold` | no drift: the second 600M keeps ≥ 7/8 on both starts and the latch is kept (MJX `episode_recovered` stays ~800) | 0/8 | 5/8 | 5/8 (34.6 s) | 5/8 (34.6 s) | **not held**: the latch is kept (MJX `episode_recovered` ~950 to the end) but 3/8 tip back onto the seat at the crouch hold. 0.05: 3/8, 0/8; 0.5: 5/8, 5/8 = 23/48 (u0k graded with hold: 46/48) |
| robot_getup_v1k | 10-23 | u1k (t3k) | + `--rise-ref-hold` | friction DR without the handover: ≥ 7/8 on both starts at 0.05, 0.35 and 0.5 | 0/8 | 7/8 | 4/8 (35.4 s) | 4/8 (35.4 s) | **not held**. 0.05: 1/8, 1/8; 0.5: 7/8, 5/8 = 25/48 (u1k with hold 20/48) |
| robot_getup_v0k_s1 | 10-23 | v0k | seed 1 | replicate (u0k_s1 parked; does hold training stop that?) | 0/8 | 6/8 | 5/8 (35.5 s) | 6/8 (35.5 s) | replicates v0k (u0k_s1 parked in every seed; this does not). 0.05: 1/8, 0/8; 0.5: 6/8, 6/8 = 25/48 |
| robot_getup_v3k | 10-23 | v0k (u0k) | init t3k → u0k | fine-tuning the 46/48 policy with hold keeps it (≥ 44/48 over the sweep) | 0/8 | 2/8 | 2/8 (34.5 s) | 5/8 (34.5 s) | **failed**: fine-tuning u0k with hold lost its robustness. 0.05: 0/8, 0/8; 0.5: 7/8, 5/8 = 19/48 |
| robot_getup_v1k_s1 | 10-23 | v1k | seed 1 | replicate (5th: likely starts tomorrow night) | 0/8 | 8/8 | 5/8 (35.6 s) | 5/8 (35.6 s) | capped at 442M steps. 0.05: 3/8, 3/8; 0.5: 7/8, 5/8 = 31/48, the best of the night |
| robot_getup_w0k | 10-24 | v0k (t3k) | ref prone_ik → prone_ikc | the crouch tip-back goes: ≥ 7/8 both starts at 0.35, ≥ 43/48 over the sweep (its replay) | 0/8 | 8/8 | 8/8 (35.4 s) | 8/8 (35.3 s) | **held**: no crouch failure left. 0.05: 7/8, 7/8; 0.5: 8/8, 8/8 = **46/48** (replay 43/48, v0k 23/48), the best get-up trained with hold |
| robot_getup_w0k_fb1024s1 | 10-24 | w0k | seed 101 (ensure_queue's fallback top-up) | replicate | 0/8 | 0/8 | 8/8 (35.6 s) | 6/8 (35.6 s) | replicate **does not hold** w0k: stands 0.40 m upright at ~23 s from supine ph, tips to 0.29 m / up-z 0.5 at 26 – 29 s in every seed (no fall counted). 0.05: 0/8, 0/8; 0.5: 0/8, 8/8 = 14/48 |
| robot_getup_w1k | 10-25 | v1k (t3k) | ref prone_ik → prone_ikc (= w0k + friction DR) | ≥ 44/48 and a narrower seed spread than w0k's (46 vs 14/48) | 0/8 | 6/8 | 3/8 (35.2 s) | 4/8 (35.2 s) | **failed**. 0.05: 1/8, 0/8; 0.5: 8/8, 5/8 = 24/48. Friction DR lost for the third time (u1k, v1k): dropped |
| robot_getup_w0k_s1 | 10-25 | w0k | seed 1 | replicate: which of 46/48 and 14/48 is typical (may be capped at ~520M) | 0/8 | 1/8 | 8/8 (34.9 s) | 8/8 (34.9 s) | replicate **does not hold** w0k (capped at 592M): stands at ~23 s from supine ph, then sits at 0.31 m (end height; w0k 0.39). 0.05: 0/8, 0/8; 0.5: 7/8, 8/8 = 24/48. Recipe over 3 seeds: 46, 14, 24 |
| robot_getup_w2k | 10-25 | w0k (t3k) | `--rise-ref-residual` 0.3 → 0.15 | less authority keeps the policy near its reference: ≥ 43/48, no stand-then-tip | 0/8 | 8/8 | 8/8 (35.4 s) | 8/8 (35.4 s) | **held**: 0.05: 8/8, 7/8; 0.5: 8/8, 8/8 = **47/48**, the best get-up so far; end height 0.39 m in every seed; one seed so far |
| robot_getup_w1k_s1 | — | w1k | seed 1 | replicate | | | | | held: w1k failed (24/48), not worth replicating |
| robot_getup_w2k_s1 | 10-26 | w2k | seed 1 | replicate: residual 0.15 narrows the seed spread (≥ 40/48, no stand-then-tip) | | | | | |
| robot_getup_w3k | 10-26 | w2k (t3k) | `--rise-ref-residual` 0.15 → 0.075 | brackets the authority down: ≥ 44/48 if nearness to the reference is the lever, else toward the replay's 43/48 | | | | | |
| robot_getup_w2k_s2 | 10-26 | w2k | seed 2 | replicate (may be capped ~590M) | | | | | |

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
  tipping back); at 0.5: 7/8 and 8/8. t3k_s1 more so: 2/8 and 0/8 at 0.05,
  7/8 and 8/8 at 0.5. The real gearbox friction is not
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
at most 12 workers in parallel. Baselines (supine ph, prone fall): t3k at
0.05 → 2/8, 6/8, at 0.5 → 8/8, 7/8; t3k_s1 at 0.05 → 0/8, 2/8, at 0.5 →
8/8, 7/8.

### Night of 2026-10-22 (evaluated 2026-10-23)

All five jobs trained the full 600M (the night started at 20:24 on Tom's
request, so u0k_s1 was not capped) and were refereed on main `02c2886` (#139:
no plant, servo or env change; the referee re-grades t3k 16/16 on supine ph +
prone fall today, so the comparison stands). The leftover fallback job
`t3k_fb1022s1` did not start (05:00 cutoff).

- **Every retrain collapsed, the control too.** u0k (t3k + a second 600M at
  the same 0.35) 0/8 on every start; u1k 0/8, u1k_s1 1/8, u0k_s1 0/8; u2k
  supine ph 5/8, prone 0/8. They get up (time to stand 23 – 39 s) and then
  fall, or park at ~0.29 m and never latch. The friction-DR hypothesis
  cannot be read from these scorecards: something shared broke them all.
- **Cause: the handover at `recovered`.** See Open questions. The env
  switches the action map (residual around the reference → full range
  around the default pose) and stops the tracking income exactly at the
  latch. The MJX eval shows the drift: `episode_recovered` falls from ~820
  to 340 – 470 over the 600M while `episode_reward` stays at ~18.5k.
  Counterfactual (CPU referee, the same policies, handover removed with
  `--rise-ref-hold`, sph / pf at 0.35): u0k 0/8, 0/8 → **8/8, 8/8**; u1k_s1
  1/8, 1/8 → 2/8, 7/8; u1k 0/8, 0/8 → 1/8, 4/8. t3k, which was trained
  with the handover and copes with it, gets slightly worse under hold (6/8,
  7/8 at 0.35; 0/8, 2/8 at 0.05), so hold is a training change, not a free
  referee win.
- **u0k under hold is friction-robust: 46/48** over 0.05 / 0.35 / 0.5 on
  both starts (0.05: 7/8, 8/8). t3k at 0.05 was 2/8, 6/8. More training at
  a single friction made the get-up itself robust; the handover hid it.
- **Friction DR: no measurable effect yet.** Graded with hold over the
  sweep: control pair u0k 46/48, u0k_s1 0/48; DR pair u1k 20/48, u1k_s1
  31/48. Seed variance swamps it. u2k (DR from p13k) is the flattest on the
  supine phase (5/8 at 0.05 and 0.35) but never stands from prone.

**Decision.** Fix the handover in training before anything else:
`--rise-ref-hold` (PR #140) keeps the residual reference map and the
tracking reward running after the held stand (the reference holds its
standing frame). The get-up policy holds the stand until a walker takes
over (arc step 4), which is how the deployment would run it anyway. Tonight
re-runs night 2's control and DR arms with hold (one variable each against
u0k and u1k), a seed replicate, and a fine-tune of u0k with hold.

**Tonight (night 3)**, all on the t3k recipe + `--rise-ref-hold`:

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_getup_v0k | u0k (t3k) | + hold | no drift: ≥ 7/8 both starts, `episode_recovered` stays ~800 |
| 2 | robot_getup_v1k | u1k (t3k) | + hold | DR without the handover: ≥ 7/8 both starts at 0.05/0.35/0.5 |
| 3 | robot_getup_v0k_s1 | v0k | seed 1 | replicate (u0k_s1 parked at 0.28 m) |
| 4 | robot_getup_v3k | v0k (u0k) | init t3k → u0k | fine-tuning the 46/48 policy with hold keeps ≥ 44/48 |
| 5 | robot_getup_v1k_s1 | v1k | seed 1 | replicate; at ~1 h 46 min per job it likely misses the 05:00 cutoff |

**Tomorrow's evaluation:** the referee picks hold up from each run's
config.json. Sweep every run at 0.05 and 0.5 as before; for comparison
with night 2 use the `scorecard_hold_partial` / `scorecard_fl*_hold_partial`
files in the u-runs (graded with `--rise-ref-hold`). Check the MJX
`eval/episode_recovered` curve: it should not fall.

### Night of 2026-10-23 (evaluated 2026-10-24)

All five jobs trained (v1k_s1 capped at 442M by the 05:00 cutoff) and were
refereed on main `f2da72e`. Upstream since the last plan: the as-built servo
IDs and calibration (#141, firmware/tools/docs and `current_budget.py`'s ID
map); no plant, servo-model or env change, so the runs compare with night 2.
The referee re-grades u0k with hold 16/16 at 0.35 today.

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5:

| run | 0.05 | 0.35 | 0.5 | sweep | parent (with hold) |
|---|---|---|---|---|---|
| v0k | 3/8 0/8 | 5/8 5/8 | 5/8 5/8 | 23/48 | u0k 46/48 |
| v1k | 1/8 1/8 | 7/8 4/8 | 7/8 5/8 | 25/48 | u1k 20/48 |
| v0k_s1 | 1/8 0/8 | 6/8 6/8 | 6/8 6/8 | 25/48 | — |
| v3k | 0/8 0/8 | 2/8 5/8 | 7/8 5/8 | 19/48 | u0k 46/48 |
| v1k_s1 (442M) | 3/8 3/8 | 8/8 5/8 | 7/8 5/8 | 31/48 | — |
| *prone_ik replayed, no RL* | 5/8 6/8 | 7/8 7/8 | 8/8 8/8 | **41/48** | |

- **Hold fixed the drift, not the score.** MJX `eval/episode_recovered`
  stays at ~950 – 1000 to the end in every run (u0k fell from ~820 to 340),
  and the referee counts no falls. But no run passes 7/8 on both
  starts: the hypotheses of v0k, v1k and v3k did not hold. Fine-tuning u0k
  with hold (v3k) lost its robustness (46/48 → 19/48). No hold run parks in
  every seed as u0k_s1 did, and the seed spread is narrower (19 – 31/48 vs
  0 – 46/48).
- **Where they fail: the crouch hold.** Traced (`--trace`, v0k at 0.35 and
  0.05, v1k_s1 at 0.05): every failing seed sits at the crouch hold (0.24 m,
  upright) and then, at 13 – 15 s into the seat push, drops to 0.20 m and
  tips back onto the seat, ending parked at 0.29 m tilted ~65°. The passing
  seeds go on to stand by the same timeline. At 0.05 a second mode appears:
  stand, then fall backwards after straightening.
- **The reference is the problem, and the RL made it worse.** The prone_ik
  reference replayed through the referee with zero action scores 41/48,
  better than every night-3 policy; its own failures are the same crouch
  tip-back (seeds 7/607/707) and, at 0.05, the fall after straightening. The
  crouch hold lifts the hands off the floor with the CoM 48 mm behind the
  sole centre (see "The centred crouch hold"). Keeping the hands down with
  the ankle at −35 (prone_ikc) removes every crouch failure of the replay
  (43/48). Ankle −35 alone, ankle −38, or ankle −35 with the arms overhead do
  not: seed 707 still tips back at 0.35 and stands 216/216 on the open-loop grid.
- **Friction DR**: with hold, the DR pair (25, 31/48) is slightly ahead of
  the control pair (23, 25/48), within seed variance.

**Decision.** Retrain on `getup_ref_v6ar_prone_ikc.npz` (arc step 2's
"fix the reference first"): v0k's and v1k's recipes with only the reference
changed, a seed replicate of each, and the residual scale (0.3 → 0.15) as
the next hardening variable, because every policy scored below its
reference's replay. From now on the morning evaluation also grades the
run's reference with `--ref-replay`: a policy must beat it.

**Tonight (night 4)**, all on the v0k recipe (t3k init, hold) with
`--rise-ref getup_ref_v6ar_prone_ikc.npz`:

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_getup_w0k | v0k (t3k) | ref → prone_ikc | crouch tip-back goes: ≥ 7/8 both starts at 0.35, ≥ 43/48 over the sweep |
| 2 | robot_getup_w1k | v1k (t3k) | ref → prone_ikc | as w0k; DR lifts 0.05 |
| 3 | robot_getup_w0k_s1 | w0k | seed 1 | replicate |
| 4 | robot_getup_w2k | w0k (t3k) | residual 0.3 → 0.15 | stays near its reference: ≥ 43/48 |
| 5 | robot_getup_w1k_s1 | w1k | seed 1 | replicate (capped at ~440M) |

**Night 4 re-ordered (2026-10-24, Tom): one-leg stands and short walks go
first.** Today's bench measured the body as built (the legs only, no pack, Pi,
head, neck or arms: [roll-compliance-2026-10-24.md](roll-compliance-2026-10-24.md)),
and Tom asked that tonight train single-leg stands on either foot and short
walks with a hard "every foot lifts ≥ 3 cm on every step" check on the
as-built plant. The queue for the night is therefore:

| order | run | what | steps |
|---|---|---|---|
| 1 | robot_legs_bal1 | `skills` family on `bimo_biped_v6legs.xml`: balance L/R 45 %, alternating-foot march 25 %, weight shift (sway) 15 % | 600M |
| 2 | robot_legs_walk1 | the c13 walking recipe on the as-built plant, init `robot_walk_c13`, graded by `walk_steps_L/R` | 400M |
| 3 | robot_legs_walk2 | walk1 with the swing-height targets raised (`--mimic-lift` 0.05 → 0.07, `--swing-height` 0.06 → 0.08) | 400M |
| 4 | robot_getup_w0k | **unchanged** (the get-up lead job of the table above; the plant and the DR of the first three jobs do not touch it) | 600M |

Moved to `night/queue/held/` with a `WHY-` note, **not deleted**, for the 07:30
planner to re-queue: w1k, w0k_s1, w2k, w1k_s1. The planner evaluates **both**
tracks tomorrow: the three new runs' scorecards (balance L/R, march, the
`walk_steps` rows with their per-step clearance) and w0k against the prone_ikc
replay as above; it re-queues the held get-up jobs behind the one-leg and
walk work it judges worth repeating, never over it (planner_prompt.md,
"Two tracks"). The new runs are on a different plant (1.475 kg, no arms): their
scorecards are not comparable with the ar-plant runs above, and the get-up
jobs still train on the ar plant, where get-up needs the arms.

**Tomorrow's evaluation:** the usual sweep (0.05 / 0.5 next to the 0.35
scorecard), plus `--trace` on any run under 7/8 to see whether the failures
moved off the crouch. Compare against the prone_ikc replay (43/48:
`sim/runs/diag_refreplay_prone_ik/scorecard_fl*_replay-getup_ref_v6ar_prone_ikc_partial.md`
in `~/code/robot`, local) and against the v-runs above.

### Night of 2026-10-24 (evaluated 2026-10-25)

All five jobs trained (three legs-track jobs, then w0k; `w0k_fb1024s1` is
`ensure_queue.sh`'s fallback top-up, the w0k recipe with seed 101) and were
refereed on main `8d7925b`. Upstream since the last plan: the legs-only plant
and its DR (#149, which the legs runs train on), the one-leg stand
experiment and firmware IMU roll balance (#151), the v6 roll zeros. The
get-up plant (`bimo_biped_v6ar.xml`), servo model and get-up env are unchanged,
so w0k compares with the v-runs.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5:

| run | 0.05 | 0.35 | 0.5 | sweep | compare |
|---|---|---|---|---|---|
| w0k | 7/8 7/8 | 8/8 8/8 | 8/8 8/8 | **46/48** | v0k 23/48; prone_ikc replayed 43/48 |
| w0k_fb1024s1 (seed 101) | 0/8 0/8 | 0/8 6/8 | 0/8 8/8 | 14/48 | — |

- **The centred crouch hold worked, and w0k beats its reference.** No
  crouch tip-back is left in w0k (v0k lost 6/16 there at 0.35), and 46/48 is
  above the zero-action replay of prone_ikc (43/48): the first policy trained
  with hold that adds something over its reference. It ties u0k-graded-with-hold
  (46/48), and unlike u0k it was trained that way.
- **The replicate stands, then tips.** Traced (`--trace`, supine ph at
  0.35, seeds 107/207/607): the same timeline as w0k up to standing (0.24 m
  crouch hold at 12 – 16 s, 0.40 m upright at ~23 – 26 s, `recovered` latched)
  and then, 1 – 3 s later, the pelvis drops to 0.29 m with up-z 0.5 and stays
  there to the end (the referee counts no fall in a recovery scenario). The
  supine-ph phase reaches the reference's arms-forward ending at ~26 – 28 s, which
  is where it goes. This is the failure the replay showed at 0.05 (falls after
  straightening), now seed-specific at every friction. Seed variance on this
  recipe is as large as before (46 vs 14/48).

**Decision.** Arc step 2 continues on prone_ikc. Friction DR (w1k) is the
next hardening variable; the residual scale (w2k) tests whether less authority
removes the stand-then-tip; a third seed (w0k_s1) says which of 46 and 14/48
is typical. All three are night 4's held jobs, re-queued unchanged behind the
legs track; w1k_s1 stays held. If w0k_s1 also tips after standing, the
reference's ending (the 2 s arms-forward ramp and the standing hold) is the
next thing to verify with `getup_ref_analytic.py verify` before more RL.

#### One-leg stands and short walks (as-built legs-only plant)

`bimo_biped_v6legs.xml` (1.50 kg, no arms): these scorecards are not
comparable with the ar-plant get-up runs, and not with the v5/v6 walkers.

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_bal1 | — (from scratch) | the plant + roll-play / torso / IMU DR | balance_L 7/8 (clear 100 %), **balance_R 0/8** (clear 41 %, longest one-foot run 0.9 s), march_in_place 5/8, march_10s 0/8 (peak 2.8 cm), hip_sway 6/8, stand_10s 7/8 | 25/96, falls 49 %, 159 W | hypothesis (balance L/R ≥ 90 % clear) **held one way round only** |
| robot_legs_walk1 | robot_walk_c13 (init) | the plant + DR, c13 recipe | line_1m 8/8, square_return 8/8, goal_home 7/8; walk_steps_L/R 0/8 (min peak 0.9 / 0.7 cm) | 74/160, falls 16 % | walks; fails the per-step rule |
| robot_legs_walk2 | walk1 recipe | swing targets `--mimic-lift` 0.05 → 0.07, `--swing-height` 0.06 → 0.08 | line_1m 5/8, square_return 7/8, goal_home 8/8; walk_steps_L/R 0/8 (min peak 0.8 / 0.9 cm); median swing peak 5.1 cm (walk1 3.9) | 68/160, falls 15 %, gait asym 30 % (walk1 11 %) | hypothesis (min clearance ≥ 3 cm) **not held** |

- **Balance is one-sided, and the asymmetry is the policy's.** bal1 holds
  the left foot up for 10 s (stands on the right) and cannot stand on the
  left. The plant is laterally symmetric (whole-body CoM y 0.25 mm at the
  home pose; the torso CoM DR is ±15 mm, symmetric), so this is a local
  optimum of the policy, not the body. bal1 also burns ~180 W standing (walk1
  ~70 W) with 0.7 rad/s of wobble, and its MJX eval reward flattened by a
  third of the 600M steps.
- **The walkers fail the per-step rule on two steps only.** Per-step sole
  peaks of the referee's `walk_steps` episodes (16 per run, logged by a
  wrapper around `walk_steps_swings`): every short step is either the
  **first** one after the walk command (0.4 – 2.5 cm, often a small tap of
  the wrong foot) or **the stop step** that begins within 0.4 s after the stop
  command (0.1 – 2 cm). Walk2's steady steps all peak 5 – 8.6 cm; walk1's
  3 – 5 cm with an occasional 2.2 – 2.5. Raising the swing targets lifted the
  steady steps and left the two transition steps where they were, because the
  gait shaping (`w_feet_phase`, `w_feet_air`, `w_mimic`) pays only under a
  moving command and the clock's swing target starts at zero.

**Decision.** Two new variables. `--w-low-swing` (PR #153, merged; both envs,
tests) charges every completed swing of ≥ 3 ticks its shortfall below 3 cm
under any command, which is the referee's rule as a training cost; tonight
brackets its weight (5, 15) on walk2's recipe. For balance, the mirror-symmetry
loss (`--w-mirror-loss 1.0`, the v22fix lineage's structural fix for
asymmetry) on bal1's recipe: the mirror maps build on the legs plant (12
policy joints, 165-d obs) and the assembled obs of mirrored episodes match to
0.002 (checked on CPU with the exact recipe).

#### Tonight (night 5)

Legs track first (Tom 2026-10-24), then get-up; ends ~05:50 (the last job may
be capped to ~520M by the runner's window budget).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_bal2 | bal1 (scratch) | + `--w-mirror-loss 1.0` | balance_R ≥ 6/8 with balance_L ≥ 6/8 |
| 2 | robot_legs_walk3 | walk2 (c13) | + `--w-low-swing 5` | walk_steps_L/R ≥ 6/8 each, line_1m / square_return / goal_home within 1/8 of walk2 |
| 3 | robot_legs_walk4 | walk2 (c13) | + `--w-low-swing 15` | as walk3; brackets the weight |
| 4 | robot_getup_w1k | v1k (t3k) | ref → prone_ikc (= w0k + friction DR) | ≥ 44/48, narrower seed spread |
| 5 | robot_getup_w2k | w0k (t3k) | residual 0.3 → 0.15 | ≥ 43/48, no stand-then-tip |
| 6 | robot_getup_w0k_s1 | w0k | seed 1 | replicate |

Preflight: PASS 6/6. Fallback: the w0k recipe.

**Tomorrow's evaluation:** get-up as before (sweep 0.05 / 0.5, `--trace` on
anything under 7/8; check whether failures are the stand-then-tip at the
reference's ending). Legs: bal2's balance_L/R against bal1; walk3/walk4's
`walk_steps_L/R` and their per-step peaks (the wrapper is
`walk_steps_swings`, printed per episode) against walk2, and whether the
penalty cost line_1m / square_return. Next legs candidates: bal1's recipe
initialised from a walker on the same plant (`--init-from robot_legs_walk1`,
for bal1's 49 % falls and 180 W stand), and the winner of walk3/walk4 with a
seed replicate.

### Night of 2026-10-25 (evaluated 2026-10-26)

All six jobs trained (three legs-track jobs, then w1k, w2k, w0k_s1; w0k_s1
capped at 592M by the runner) and were refereed on main `1db2851`. Upstream
since the last plan: firmware one-leg balance (gyro damping, abort policy,
#155/#158/#160), the arms print plate, the results page. No plant, servo or env
change for either track, so tonight's comparisons hold.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5 (CPU referee,
8 seeds each):

| run | 0.05 | 0.35 | 0.5 | sweep | compare |
|---|---|---|---|---|---|
| w2k (residual 0.15) | 8/8 7/8 | 8/8 8/8 | 8/8 8/8 | **47/48** | w0k 46/48; prone_ikc replayed 43/48 |
| w1k (w0k + friction DR) | 1/8 0/8 | 6/8 4/8 | 8/8 5/8 | 24/48 | w0k 46/48 |
| w0k_s1 (seed 1) | 0/8 0/8 | 1/8 8/8 | 7/8 8/8 | 24/48 | w0k 46/48, fb1024s1 14/48 |

- **Less authority won.** w2k is the best get-up so far and ends at 0.39 m
  in every supine-ph and prone-fall seed; even 0.05, where the reference
  replayed falls after straightening, is 15/16. One seed, so not yet a
  recipe result.
- **The w0k recipe's 46/48 was the lucky seed.** Three seeds: 46, 14, 24/48.
  w0k_s1 fails the same way as fb1024s1: it stands from supine ph at ~23 s,
  then sits at 0.31 m (mean end height; w0k and w2k 0.39). The reference
  itself stands 216/216 open loop and replays 43/48, so the tip after standing
  comes from the residual policy, not the reference's ending; I did not re-run
  `verify` (plant and npz unchanged since it passed).
- **Friction DR lost for the third time** (u1k 20/48 with hold, v1k 25/48,
  w1k 24/48, each below its no-DR sibling). The 0.05 end is reached better by
  staying near the reference (w2k) than by training across friction. The
  friction-DR line is dropped; w1k_s1 stays held.

**Decision.** Arc step 2 continues on w2k's recipe (prone_ikc, hold, residual
0.15). Two replicates (seeds 1 and 2) measure whether 0.15 narrows the seed
spread, the question the w0k recipe failed; w3k halves the residual again
(0.075) to bracket it. If the w2k replicates hold (≥ 40/48 each), the next
hardening variable is pushes (`--push-prob`), then arc step 4 (handover,
distil).

#### One-leg stands and short walks (as-built legs-only plant)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_bal2 | bal1 | + `--w-mirror-loss 1.0` | balance_L 5/8 (clear 74 %), **balance_R 0/8** (clear 35 %), march_in_place 7/8, hip_sway 8/8, weight_shift 3/8, stand_10s 8/8 | 31/96 (bal1 25), falls 24 % (49 %), 154 W | **not held** (balance_R ≥ 6/8): fewer falls, still one-sided |
| robot_legs_walk3 | walk2 | + `--w-low-swing 5` | line_1m 8/8, backward 2/8, square_return 8/8, goal_home 7/8; walk_steps_L/R 2/8, 0/8 (min peak 1.6 / 1.5 cm) | 77/160 (walk2 68), falls 18 %, gait asym 9 % (30 %) | **not held** (≥ 6/8 each), but the best walker on this plant |
| robot_legs_walk4 | walk2 | + `--w-low-swing 15` | line_1m 6/8, square_return 6/8, goal_home 5/8; walk_steps_L/R 0/8, 1/8 | 70/160, falls 20 % | **not held**; costs the walk rows |

- **The mirror loss was on and the asymmetry stayed.** bal2's
  `training/sym_loss` sits at 0.0045 – 0.0052 for the whole run (the policy's
  pre-tanh mean action is near-equivariant on the training distribution), yet
  balance_R is 0/8 and balance_L fell to 5/8. The plant is mirror-exact in
  what matters: joint ranges, axes, actuators and ctrlranges mirror, segment
  CoMs mirror within 2 – 3.6 mm laterally (tens of grams each), whole-body
  CoM y 0.25 mm. bal1 and bal2 both plateau within the first third of their
  steps, from scratch. Not yet explained; tonight tests the from-scratch start.
- **The penalty fixed the first step, not the stop step.** Per-step peaks of
  the referee's walk_steps episodes (16 per run, same seeds; `walk_steps_swings`
  wrapped to print each step):

  | run | first step: median, episodes < 3 cm | steady steps: min, median | stop step: median, episodes < 3 cm |
  |---|---|---|---|
  | walk2 | 1.6 cm, 14/16 | 1.0, 6.4 cm | 1.3 cm, 14/16 |
  | walk3 (w 5) | 2.6 cm, 9/16 | 3.2, 5.8 cm | 1.5 cm, 11/16 |
  | walk4 (w 15) | 3.0 cm, 8/16 | 2.9, 6.1 cm | 0.9 cm, 13/16 |

  The stop step (a closing shuffle that begins 0.06 – 0.36 s after the stop
  command) did not move between weight 5 and 15, so weight is not the lever.
  Either the transition is too rare in training (commands resample every
  2.5 – 4.5 s; a walk → stand switch is ~13 % of resamples) or the stand
  rewards pay more for closing the stance than the penalty costs. Tonight's
  walk5 tests the first.

**Decision.** walk3 is the walk line's parent. walk5 doubles the start/stop
transitions (`--cmd-resample-s 1.0,2.5`); walk3_s1 says whether walk3 vs walk4
is seed noise. bal3 starts bal2's recipe from walk3, a policy that already
alternates single support on both feet. If walk5's stop step stays low, the
next variable is the stand reward after a stop (or a referee question for Tom:
whether a closing step must clear 3 cm).

#### Tonight (night 6)

Legs track first (Tom 2026-10-24), then get-up; ends ~05:40 (the last job may
be capped near 590M).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_bal3 | bal2 (walk3) | + `--init-from robot_legs_walk3` | balance_R ≥ 6/8 with balance_L ≥ 6/8, falls ≤ 24 % |
| 2 | robot_legs_walk5 | walk3 (c13) | `--cmd-resample-s` 2.5,4.5 → 1.0,2.5 | stop step graded more often: walk_steps_L/R ≥ 4/8 each, walk rows within 1/8 of walk3 |
| 3 | robot_legs_walk3_s1 | walk3 | seed 1 | replicate |
| 4 | robot_getup_w2k_s1 | w2k | seed 1 | ≥ 40/48, no stand-then-tip |
| 5 | robot_getup_w3k | w2k (t3k) | residual 0.15 → 0.075 | ≥ 44/48 if nearness to the reference is the lever |
| 6 | robot_getup_w2k_s2 | w2k | seed 2 | replicate |

Preflight: PASS 6/6. Fallback: the w2k recipe.

**Tomorrow's evaluation:** get-up sweep at 0.05 / 0.5 for all three; the w2k
recipe's spread over 3 seeds decides whether pushes come next. Legs: bal3's
balance_L/R against bal2; walk5's per-step table (first / steady / stop)
against walk3, and walk3_s1 against walk3.
