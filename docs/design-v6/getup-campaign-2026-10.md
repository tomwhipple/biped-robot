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
   spread (47, 47, 43) and 0.075 (48/48); night 7: pushes, graded by the
   referee's push-stress card at 15 and 30 N — training shoves did not help,
   the failure is a sink after the stand; night 8: the early termination
   after the latch, `--rise-ref-et-hold` — held with shoves: 48/48 and
   15 N 14 and 16/16 on two seeds; night 9: training shove force 30 and
   22 N, for the 30 N grade — barely moved it (5, 6 and 3/16): closed on
   force, the 30 N misses are shoves off a time-indexed roll)*
   **On the as-built servos** (#147, from night 10): the w5k recipe on
   STS3215 knees, the robot's set. The analytic reference stands open loop
   on them (144/216; it fails only at friction 0.05, tipping back at the
   final straightening), unlike the hand-tuned rise #147's evidence was
   taken on. *(night 10: held, sweep 48/48 on three runs, 15 N 15 – 16/16;
   night 11: the robot's own gains, `--servo-kp-scale asbuilt`, then the
   deploy action contract `--quantize --ctrl-within-stops`)*
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
| robot_getup_w2k_s1 | 10-26 | w2k | seed 1 | replicate: residual 0.15 narrows the seed spread (≥ 40/48, no stand-then-tip) | 0/8 | 8/8 | 8/8 (35.5 s) | 8/8 (35.5 s) | **held**: 0.05: 7/8, 8/8; 0.5: 8/8, 8/8 = **47/48**. Push stress (0.35) 15 N @ 2 %: 8/16; 30 N: 2/16 |
| robot_getup_w3k | 10-26 | w2k (t3k) | `--rise-ref-residual` 0.15 → 0.075 | brackets the authority down: ≥ 44/48 if nearness to the reference is the lever, else toward the replay's 43/48 | 0/8 | 8/8 | 8/8 (35.4 s) | 8/8 (35.4 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**, the first clean sweep; one seed. Push stress 15 N: 8/16; 30 N: 1/16 |
| robot_getup_w2k_s2 | 10-26 | w2k | seed 2 | replicate (may be capped ~590M) | 0/8 | 8/8 | 7/8 (35.5 s) | 7/8 (35.6 s) | **held** (≥ 40): 0.05: 6/8, 7/8; 0.5: 8/8, 7/8 = 43/48; the misses stand, then sink (stand-then-tip). Recipe over 3 seeds: 47, 47, 43 |
| robot_getup_w4k | 10-27 | w2k (t3k) | training pushes: `--push-prob` 0 → 0.02 with `--push-force 15` (shoves, not kicks) | 15 N @ 2 % stress ≥ 15/16 (w2k 12/16), 30 N above 2/16, sweep ≥ 43/48 | 0/8 | 8/8 | 7/8 (35.2 s) | 8/8 (35.1 s) | **failed**: 15 N 7/16, 30 N 0/16; 0.05: 4/8, 6/8; 0.5: 7/8, 8/8 = 41/48. Under shoves 7/8 supine seeds stand, then sink to ~0.27 m |
| robot_getup_w3k_s1 | 10-27 | w3k | seed 1 | replicate: 0.075 is a recipe (≥ 44/48) | 0/8 | 8/8 | 8/8 (34.9 s) | 8/8 (34.9 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48** (w3k 48/48). 15 N 11/16, 30 N 1/16 |
| robot_getup_w4k_s1 | 10-27 | w4k | seed 1 | replicate (capped at 585M) | 0/8 | 8/8 | 8/8 (35.3 s) | 8/8 (35.3 s) | sweep 48/48 (0.05 and 0.5 8/8 each), 15 N 13/16, 30 N 2/16: within the no-push recipe's 15 N spread (w2k 12, w2k_s1 8) |
| robot_getup_w5k | 10-28 | w4k (t3k) | + `--rise-ref-et-hold` (#163: early termination after the latch) | the post-stand sink goes: 15 N ≥ 14/16, 30 N above 2/16, sweep ≥ 43/48 | 0/8 | 8/8 | 8/8 (35.5 s) | 8/8 (35.5 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**; 15 N **14/16** (w4k 7), 30 N 4/16 (w4k 0). One post-stand sink at 15 N (w4k 9) |
| robot_getup_w6k | 10-28 | w2k (t3k) | + `--rise-ref-et-hold` (no pushes) | control for w5k: sweep ≥ 47/48, 15 N above w2k's 12/16 | 0/8 | 8/8 | 8/8 (35.4 s) | 8/8 (35.4 s) | sweep **48/48** (w2k 47); 15 N 11/16, 30 N 1/16: **not held** on the push grade (4 post-stand sinks at 15 N). The termination alone does not harden; with shoves it does |
| robot_getup_w5k_s1 | 10-28 | w5k | seed 1 | replicate (may be capped ~590M) | 0/8 | 8/8 | 8/8 (35.9 s) | 8/8 (35.9 s) | trained the full 600M. **Held**: sweep **48/48**, 15 N **16/16** (no sink), 30 N 4/16. Recipe over 2 seeds: 48, 48; 15 N 14, 16 |
| robot_getup_w7k | 10-29 | w5k (t3k) | `--push-force` 15 → 30 | the 30 N misses (11/16 prone episodes never stand, plus sinks) are trainable: 30 N ≥ 8/16, 15 N ≥ 14/16, sweep ≥ 43/48 | 0/8 | 8/8 | 8/8 (34.5 s) | 8/8 (34.6 s) | **not held**: sweep 48/48, 15 N 15/16, 30 N **5/16** (w5k 4). 5 of 8 prone episodes at 30 N still never stand |
| robot_getup_w8k | 10-29 | w5k (t3k) | `--push-force` 15 → 22 | brackets w7k: 30 N above 4/16, 15 N ≥ 14/16, sweep ≥ 43/48 | 0/8 | 8/8 | 8/8 (34.8 s) | 8/8 (34.8 s) | **not held**: sweep 48/48, 15 N 16/16, 30 N 3/16 |
| robot_getup_w7k_s1 | 10-29 | w7k | seed 1 | replicate (may be capped) | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | capped at 592M. Sweep 48/48, 15 N 16/16, 30 N 6/16. Training force over 2 seeds: 30 N 5, 6 vs w5k's 4, 4 |
| robot_getup_w9k | 10-30 | w5k (t3k) | knee servos STS3250 → STS3215 (the as-built set; `--servo-3250 knee` dropped) | the get-up does not need STS3250 knees on the analytic reference (as-built open-loop verify 144/216): sweep ≥ 40/48, 15 N ≥ 12/16, graded on as-built servos | 0/8 | 8/8 | 8/8 (35.4 s) | 8/8 (35.4 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**; 15 N **16/16**, 30 N 3/16 (w5k 48, 14, 4). The get-up needs no STS3250 knees |
| robot_getup_w10k | 10-30 | w9k (w5k_s1) | init t3k → w5k_s1 | the best design-servo policy adapts to the softer knees at least as well as t3k: sweep ≥ w9k's, 15 N ≥ 12/16 | 0/8 | 8/8 | 8/8 (35.1 s) | 8/8 (35.2 s) | **held**, and no better than from t3k: sweep 48/48, 15 N 16/16, 30 N 2/16. The init does not matter |
| robot_getup_w9k_s1 | 10-30 | w9k | seed 1 | replicate | 0/8 | 8/8 | 8/8 (35.3 s) | 8/8 (35.3 s) | **replicates**: sweep 48/48, 15 N 15/16, 30 N 4/16 |
| robot_getup_w9k_s128r24 | 10-31 | w9k | (distil) the stock 24-round (128, 128) student | arc step 4: the residual get-up survives distillation, supine ph + prone fall ≥ 14/16 at 0.35 | | | | | |
| robot_getup_w11k | 10-31 | w9k (t3k) | `--servo-kp-scale robot` → `asbuilt` (factory P32: hip pitch, hip yaw, ankle 1× → 2.8×) | the get-up holds on the gains the robot runs: sweep ≥ 46/48, 15 N ≥ 14/16 | | | | | |
| robot_getup_w12k | 10-31 | w11k (t3k) | + `--quantize --ctrl-within-stops` (the rest of the legs deploy contract) | no loss vs w11k: the deployable get-up recipe | | | | | |
| robot_getup_w11k_s1 | 10-31 | w11k | seed 1 | replicate (may be capped) | | | | | |

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

### Night of 2026-10-26 (evaluated 2026-10-27)

All six jobs trained the full budget (three legs-track jobs, then w2k_s1, w3k,
w2k_s2; the night ended 05:34) and were refereed on main `1689596`. Upstream
since the last plan: only the ledger and the results page. No plant, servo or
env change on either track, so the comparisons hold. This morning's code
(#162 + `0246410`) adds the push-stress grade and `--push-force` and changes
nothing a past run trained or was graded with.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5 (CPU referee,
8 seeds each):

| run | 0.05 | 0.35 | 0.5 | sweep | compare |
|---|---|---|---|---|---|
| w2k_s1 (seed 1) | 7/8 8/8 | 8/8 8/8 | 8/8 8/8 | **47/48** | w2k 47/48 |
| w3k (residual 0.075) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | w2k 47/48; prone_ikc replayed 43/48 |
| w2k_s2 (seed 2) | 6/8 7/8 | 8/8 7/8 | 8/8 7/8 | 43/48 | w2k 47/48 |

- **The residual 0.15 recipe holds over three seeds: 47, 47 and 43/48**
  (the w0k recipe at 0.3: 46, 14, 24). Less authority is what narrowed the
  spread. w2k_s2's misses still stand at ~23 s and then sink (mean end height
  0.36 m at 0.05 with 6/8 passing, so the misses end near 0.28 m). That is the
  old stand-then-tip, now in 1 – 2 seeds out of 8 instead of most of them.
- **0.075 swept clean** (48/48, end height 0.39 m everywhere), one seed. Over
  0.3 → 0.15 → 0.075 the referee keeps improving as the policy's authority
  shrinks, while the zero-authority replay is 43/48, so the optimum sits
  somewhere between 0 and 0.15.
- **The referee's pushes are too gentle to separate these policies.** The
  hardware-claim card pins 5 N shoves held one control step at 1 % per tick,
  and every w-run passes it. I graded harder shoves (`eval_precision.py
  --push-force/--push-prob`, new this morning, writes `scorecard_push<N>x<p>`;
  supine ph + prone fall at 0.35):

  | policy | 15 N @ 2 % | 30 N @ 2 % |
  |---|---|---|
  | w2k (0.15) | 12/16 | 2/16 |
  | w2k_s1 (0.15) | 8/16 | 2/16 |
  | w3k (0.075) | 8/16 | 1/16 |

  No run has trained with pushes since the campaign began (`--push-prob 0`),
  and the `--precision` preset's pushes are velocity kicks, which ignore a
  force setting. `train_mjx.py --push-force` now trains with shoves of the
  kind the grade uses (it turns the kicks off).

**Decision.** Arc step 2 moves to pushes, as planned once the w2k replicates
held. w4k trains w2k's recipe with 15 N shoves at 2 % (graded at 15 and 30 N,
plus the friction sweep), with a seed replicate. The parent is w2k, not w3k:
w2k has three seeds behind it and the best 15 N grade, and less authority
leaves the policy less room to catch a shove. w3k_s1 decides whether 0.075 is
a recipe. If pushes cost the sweep, the next variable is the residual with
pushes on (0.15 vs 0.3).

#### One-leg stands and short walks (as-built legs-only plant)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_bal3 | bal2 | `--init-from robot_legs_walk3` | balance_L 2/8 (clear 86 %), **balance_R 2/8 (clear 93 %)**, march_in_place 8/8, weight_shift 6/8, hip_sway 2/8, stand_10s 8/8 | 28/96 (bal2 31), falls 31 % (24 %), 90 W (154 W) | **not held** (≥ 6/8 each), but **the first two-sided balance**: bal1/bal2 had balance_R 0/8 at clear 35 – 41 % |
| robot_legs_walk5 | walk3 | `--cmd-resample-s` 2.5,4.5 → 1.0,2.5 | line_1m 7/8, backward 4/8, square_return 8/8, goal_home 8/8; walk_steps_L/R 0/8, 0/8 (min peak 0.6 cm) | 76/160 (walk3 77) | **not held**: the stop step did not move and the first step got worse |
| robot_legs_walk3_s1 | walk3 | seed 1 | line_1m 7/8, backward 7/8, square_return 8/8, goal_home 8/8; walk_steps_L/R 0/8, 0/8 (min peak 1.1 / 0.4 cm) | 78/160 | walks as well as walk3; walk3's walk_steps_L 2/8 was seed luck |

- **bal3 balances on either foot.** On each side the lifted foot is up for
  8.5 – 8.8 s of the 10 s window, with drift ≤ 2 cm and no falls. It fails
  the pass rule on brief touchdowns (lifted-foot contact 3 – 7 % against a
  5 % limit, longest unbroken one-foot run 3.5 – 4.3 s on average) and on
  clearance (86 – 93 % against 90 %). It also stands at 64 W (bal2 188 W).
  Starting from a walker, not the mirror loss, is what removed the
  one-sidedness. Its MJX eval reward flattened over the last quarter
  (1494 → 1482).
- **The stop step is the same ~1 cm closing shuffle whatever we change.**
  Per-step peaks of the walk_steps episodes (16 per run, same seeds;
  `walk_steps_swings` wrapped to print each step):

  | run | first step: median, episodes < 3 cm | steady steps: min, median | stop step: median, episodes < 3 cm |
  |---|---|---|---|
  | walk3 | 2.6 cm, 9/16 | 3.2, 5.8 cm | 1.5 cm, 11/16 |
  | walk3_s1 (seed 1) | 2.7 cm, 10/16 | 0.5, 6.3 cm | 0.9 cm, 13/16 |
  | walk5 (2× transitions) | 1.8 cm, 13/16 | 1.4, 6.1 cm | 0.7 cm, 13/16 |

  So far: the low-swing weight (5, 15), the seed and the number of
  transitions have all left the stop step at 0.7 – 1.5 cm. The recipe has no
  term that charges moving at a plain stand (`w_still`, which the deployed
  loco lineage has used at 0.1 since v18d, is off in the c13/legs recipe),
  while `w_pose` 0.3 pulls toward the home pose, so closing the stance with
  a shuffle costs nothing.

**Decision.** walk6/walk7 add `--w-still` 0.1 / 0.3 to walk3 (a bracket, read
through the per-step table). bal3_s1 checks that the walker start, not the
seed, made balance two-sided. If bal3_s1 holds, the next balance variable
targets the touchdowns. Tom has not answered the open referee question,
whether the closing step after a stop must clear 3 cm. It goes to him again
today.

#### Tonight (night 7)

Legs track first (Tom 2026-10-24), then get-up; expected end ~05:40 (the last
job may be capped near 590M).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk6 | walk3 (c13) | + `--w-still 0.1` | stop-step episodes < 3 cm ≤ 6/16; walk_steps_L/R ≥ 3/8 each; walk rows within 1/8 of walk3 |
| 2 | robot_legs_walk7 | walk3 (c13) | + `--w-still 0.3` | as walk6; brackets the weight |
| 3 | robot_legs_bal3_s1 | bal3 (walk3) | seed 1 | both sides clear ≥ 80 %, neither 0/8 |
| 4 | robot_getup_w4k | w2k (t3k) | `--push-prob` 0 → 0.02, `--push-force 15` | 15 N grade ≥ 15/16 (w2k 12/16), 30 N above 2/16, sweep ≥ 43/48 |
| 5 | robot_getup_w3k_s1 | w3k | seed 1 | ≥ 44/48 |
| 6 | robot_getup_w4k_s1 | w4k | seed 1 | replicate |

Preflight: PASS 6/6 (w4k resolves to `push_kick` False, 15 N, 0.02). Fallback:
the w2k recipe (three seeds behind it).

**Tomorrow's evaluation:** get-up: the friction sweep, plus the push-stress
grade (`eval_precision.py --run-name <run> --scenarios
recover_supine_ph,recover_prone_fall --push-force 15 --push-prob 0.02`, and
30) for all three, compared with w2k (12/16, 2/16) and w3k (8/16, 1/16).
Legs: the per-step table for walk6/walk7 against walk3; bal3_s1's
balance_L/R against bal3.

### Night of 2026-10-27 (evaluated 2026-10-28)

All six jobs trained (three legs-track jobs, then w4k, w3k_s1, w4k_s1; the
runner capped w4k_s1 at 585M; the night ended 05:40) and were refereed on
main `e502e18`. Upstream since the last plan: only the ledger and the results
page. No plant, servo or env change on either track, so the comparisons
hold. This morning's #163 (`--rise-ref-et-hold`) is training-only and off by
default; it changes nothing a past run trained or was graded with.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w4k (15 N training shoves) | 4/8 6/8 | 8/8 8/8 | 7/8 8/8 | 41/48 | 7/16 | 0/16 | w2k 47/48, 12/16, 2/16 |
| w4k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 13/16 | 2/16 | w2k_s1 47/48, 8/16, 2/16 |
| w3k_s1 (residual 0.075, seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 11/16 | 1/16 | w3k 48/48, 8/16, 1/16 |

- **Training with 15 N shoves did not harden the get-up.** The 15 N grade
  is 7 and 13/16 with training pushes against 12 and 8/16 without (w2k,
  w2k_s1), and 30 N stays at 0 – 2/16. One of the two push seeds also lost
  the 0.05 end (41/48). Hypothesis not held.
- **0.075 is a recipe:** 48/48 on both seeds, the only recipe with two clean
  sweeps. Its push grade (8 and 11/16) is no better than 0.15's.
- **Where the shoves win: after the stand, not during the rise.** Per-seed
  push-grade episodes (the referee's evaluate wrapped to print stand time,
  end height and the first drop below 0.8 of standing height after the
  stand):

  | policy, 15 N | stood | misses that stood, then sank | misses that never stood |
  |---|---|---|---|
  | w2k (supine ph / prone fall) | 8/8, 7/8 | 1, 1 (end ~0.28 m, 3 s after the stand) | 0, 1 (flat, 0.05 m: shoved off the roll) |
  | w4k | 8/8, 8/8 | 7, 2 (end ~0.28 m, 3.5 – 7 s after) | 0, 0 |

  Every sunk seed stands, then settles 3 – 7 s later into a ~0.27 m crouch
  and stays there with no fall counted. That is the same stand-then-sink the
  unlucky seeds showed without pushes (w0k_s1, w0k_fb1024s1, w2k_s2), so the
  shoves only expose it. The env explains why it is a stable place to end:
  the DeepMimic early termination (`--rise-ref-et 0.1`) only fires before
  the stand latch, and under `--rise-ref-hold` the joint and root tracking
  keep paying after it. A crouch 12 cm below the reference's standing frame
  is never ended and keeps partial income, and training shoves (w4k) pushed
  the policy into it more often (7/8 supine seeds against w2k's 1/8).

**Decision.** #163 adds `--rise-ref-et-hold`: with hold on, the termination
also runs after the latch (vs the reference's standing frame, 0.1 m), so
sinking out of the stand ends the episode. w5k = w4k + the flag (pushes on),
w6k = w2k + the flag (no pushes, a control: does the termination alone fix
the seed misses and lift the push grade?), w5k_s1 a replicate. The parent
stays w2k/w4k (residual 0.15) rather than 0.075: the authority to catch a
shove is the point tonight. If the post-latch termination holds, the next
step is arc step 4 (handover to the walker): in deployment, the standing
balance under shoves is the walker's job.

#### One-leg stands and short walks (as-built legs-only plant)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_bal3_s1 | bal3 | seed 1 | **balance_L 7/8 (clear 98 %), balance_R 8/8 (clear 99 %)**, march_in_place 8/8, hip_sway 8/8, weight_shift 2/8, stand_10s 8/8 | 41/96 (bal3 28), falls 34 %, 82 W | **held**, and the first policy that passes one-leg balance on both feet (≥ 6/8 each). bal3 was the unlucky seed |
| robot_legs_walk6 | walk3 | + `--w-still 0.1` | line_1m 3/8, backward 8/8, square_return 7/8, goal_home 7/8; walk_steps_L/R 0/8, 0/8 (min peak 0.7 / 0.5 cm) | 77/160 (walk3 77), falls 19 % | **not held**; line_1m fell 8/8 → 3/8 |
| robot_legs_walk7 | walk3 | + `--w-still 0.3` | line_1m 6/8, backward 7/8, square_return 8/8, goal_home 7/8; walk_steps_L/R 1/8, 0/8 (min peak 1.3 / 0.7 cm) | 79/160, falls 15 %, gait asym 14 % | **not held** (stop step unmoved) |

Per-step peaks of the walk_steps episodes (16 per run, same seeds;
`walk_steps_swings` wrapped to print each step):

| run | first step: median, episodes < 3 cm | steady steps: min, median | stop step: median, episodes < 3 cm |
|---|---|---|---|
| walk3 | 2.6 cm, 9/16 | 3.2, 5.8 cm | 1.5 cm, 11/16 |
| walk6 (w_still 0.1) | 2.1 cm, 12/16 | 0.4, 6.1 cm | 0.7 cm, 14/16 |
| walk7 (w_still 0.3) | 2.3 cm, 12/16 | 2.1, 6.0 cm | 0.6 cm, 14/16 |

- **Charging motion at the stand did not stop the shuffle either.** Five
  reward-side variables (low-swing 5 and 15, 2× transitions, w_still 0.1 and
  0.3) and a seed have left the stop step at 0.6 – 1.5 cm. Reading the env:
  at a plain stand every gait-shaping term is off (feet phase, mimic, feet
  air, symmetry are gated on `cmd_moving`) and `w_single_support` pays double
  support every tick, while the gait clock keeps running (the legs recipe
  does not set `--clock-freeze-stand`). A closing shuffle starting 0.06 –
  0.36 s after the stop is when that clock swings the trailing foot.
- The first step got worse in both (12/16 < 3 cm vs 9/16).

**Decision.** Balance: bal3_s2 decides whether bal3_s1's two-sided pass is
the recipe (2 of 3 seeds ≥ 6/8 each) before the next balance variable. Walk:
walk8 = walk3 + `--clock-freeze-stand` (an existing flag, the deployed loco
lineage's 2026-08-12 stand fix, mirrored in the referee's env): if the
running clock drives the shuffle, holding it at a stand removes it. If walk8
also leaves the stop step, the next lever is env work, a short "finish the
step" window after a stop in which the gait terms stay on. Tom's referee
question (must the closing step after a stop clear 3 cm?) is still open
after three days. Even if it is waived, the first step still fails (walk3
9/16, walk6/7 12/16).

#### Tonight (night 8)

Legs track first (Tom 2026-10-24), then get-up; expected end ~06:30 (the
last job may be capped).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_bal3_s2 | bal3 (walk3) | seed 2 | balance_L/R ≥ 6/8 each |
| 2 | robot_legs_walk8 | walk3 (c13) | + `--clock-freeze-stand` | stop-step episodes < 3 cm ≤ 6/16; walk_steps_L/R ≥ 3/8 each; walk rows within 1/8 of walk3 |
| 3 | robot_getup_w5k | w4k (t3k) | + `--rise-ref-et-hold` | 15 N ≥ 14/16, 30 N above 2/16, sweep ≥ 43/48 |
| 4 | robot_getup_w6k | w2k (t3k) | + `--rise-ref-et-hold` | sweep ≥ 47/48; 15 N above 12/16 without training pushes |
| 5 | robot_getup_w5k_s1 | w5k | seed 1 | replicate |

Preflight: PASS 5/5 (preflight now maps `--rise-ref-et-hold` into the env
it constructs). Fallback: the w2k recipe.

**Tomorrow's evaluation:** get-up: the friction sweep and the push grade at
15 and 30 N for all three, plus the per-seed sink check (stood, then below
0.8 of standing height). Legs: bal3_s2's balance_L/R against bal3 and
bal3_s1; walk8's per-step table against walk3.

### Night of 2026-10-28 (evaluated 2026-10-29)

All five jobs trained the full budget (bal3_s2, walk8 at 400M, then w5k,
w6k, w5k_s1; the night ended 04:58) and were refereed on main `7382499`.
Upstream since the last plan: only the ledger and the results page. No plant,
servo or env change on either track, so the comparisons hold. This
morning's #164 (`--stop-grace-s`) is reward-only and off by default. It
changes nothing a past run trained or was graded with.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w5k (shoves + ET after the latch) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **14/16** | 4/16 | w4k 41/48, 7/16, 0/16 |
| w5k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 4/16 | w4k_s1 48/48, 13/16, 2/16 |
| w6k (ET after the latch, no shoves) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 11/16 | 1/16 | w2k 47/48, 12/16, 2/16 |

- **The post-latch termination closed the sink, but only with training
  shoves.** w5k's hypothesis held on both seeds: the sweep is clean, and
  the 15 N grade is 14 and 16/16, against 7 and 13 with shoves alone (w4k)
  and 12 and 8 with neither (w2k). Per-seed traces at 15 N (stood, then
  below 0.8 of standing height): w4k 9 sinks in 16, w5k 1, w5k_s1 0. The
  control w6k (termination, no shoves) also swept clean (48/48, w2k's
  seeds were 47, 47, 43), but its 15 N grade did not rise (11/16, 4 sinks).
  The termination alone does not teach the policy to hold a shoved stand.
  Shoves without the termination pushed it into the crouch (w4k). Together
  they hold. The miss left at 15 N is the same prone seed every time: shoved
  off the roll, never stands (w2k had it too).
- **30 N is where it fails now: on the prone roll, not after the stand.**
  Over w5k and w5k_s1 (32 episodes): 8 pass; 13 never stand (11 of them
  prone fall, shoved off the roll); 7 stand and then sink to 0.27 – 0.32 m;
  4 stand and then go down flat. Under 30 N the weak phase is the prone
  roll.
- The MJX curves show nothing new: `eval/episode_recovered` ~900 – 1000 at
  the end, as for w4k.

**Decision.** The w5k recipe is the get-up (two clean sweeps, the best push
grade so far) and the new fallback. Arc step 2 continues on the one gap the
grade shows: train at the force that breaks it. w7k = w5k with 30 N training
shoves, w8k = 22 N (a bracket: friction DR failed three times by asking too
much, so a half step guards against that), w7k_s1 a replicate. Arc step 4
(handover, distil, SIL) needs a body that exists: the get-up trains on the
arms body with STS3250 knees, and the as-built robot is the legs-only plant
with STS3215 knees (#147). A distil of w5k_s1 is ready to queue once Tom says
the arms body is still the target.

#### One-leg stands and short walks (as-built legs-only plant)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_bal3_s2 | bal3 | seed 2 | balance_L 6/8 (clear 95 %), **balance_R 0/8 (clear 24 %)**, march_in_place 8/8, hip_sway 2/8, weight_shift 0/8, stand_10s 8/8 | 24/96 (bal3 28, bal3_s1 41), falls 28 %, 90 W | **not held**: one-sided again |
| robot_legs_walk8 | walk3 | + `--clock-freeze-stand` | line_1m 7/8, backward 5/8, square_return 8/8, goal_home 8/8, sidestep 8/8 8/8; walk_steps_L/R 0/8, 0/8 (min peak 0.9 / 1.3 cm) | 81/160 (walk3 77), falls 16 % | **not held**: walk rows fine, stop step unmoved, first step worse |

Per-step peaks of the walk_steps episodes (16 per run, same seeds;
`walk_steps_swings` wrapped to print each step; walk3 re-run this morning,
identical to 10-26):

| run | first step: median, episodes < 3 cm | steady steps: min, median | stop step: median, episodes < 3 cm |
|---|---|---|---|
| walk3 | 2.6 cm, 9/16 | 3.2, 5.8 cm | 1.5 cm, 11/16 |
| walk8 (clock freeze) | 1.9 cm, 15/16 | 3.7, 5.9 cm | 0.9 cm, 12/16 |

- **The balance recipe is one seed in three.** Over bal3, bal3_s1 and
  bal3_s2 the balance rows are (2, 2), (7, 8), (6, 0). Across all five
  balance runs (bal1, bal2 and the three bal3 seeds), balance_R is the
  weak side in four. The walker start makes a two-sided policy possible
  but does not make it reliable. bal3_s1 remains the only policy that passes
  on both feet.
- **Holding the clock did not stop the shuffle either.** Seven variables now
  (low-swing 5 and 15, 2× transitions, w_still 0.1 and 0.3, clock freeze,
  seed) leave the stop step at 0.6 – 1.5 cm. The env explains why
  (`env_mjx.step`): `w_low_swing` charges a 1 cm shuffle once (2/3 × 5),
  but at a plain stand `w_single_support` pays double support every tick,
  feet-air and swing-height pay nothing, and `w_pose` pulls the feet home.
  A quick low shuffle is the cheapest way to close the stance. A 3 cm step
  needs more ticks in single support, and every one of them costs.

**Decision.** Walk: #164 adds `--stop-grace-s`. For that long after a
moving command, the reward gating treats the command as a walk (feet air,
swing height, cadence, symmetry, single support; the stand terms stay off).
It is reward-only: the obs, the clock freeze and the referee are unchanged.
walk9 = walk3 + 0.4 s (the referee's tail), walk9_s1 a replicate (walk
seeds vary: walk3 2/8 vs walk3_s1 0/8). The first step (a 2 cm tap within
50 ms of the walk command) is left for after the stop step. Balance: bal4
trains a second 600M from bal3_s1, the passing policy. Either the bal
recipe has a one-sided optimum that training finds from anywhere, or a
two-sided start keeps both sides. Tom's referee question (must the closing
step clear 3 cm?) is still open after four days.

#### Tonight (night 9)

Legs track first (Tom 2026-10-24), then get-up; expected end ~05:45 (the
last job may be capped).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk9 | walk3 (c13) | + `--stop-grace-s 0.4` | stop-step episodes < 3 cm ≤ 6/16 (walk3 11/16); walk rows within 1/8 of walk3 |
| 2 | robot_legs_walk9_s1 | walk9 | seed 1 | replicate |
| 3 | robot_legs_bal4 | bal3_s1 (bal3_s1) | init walk3 → bal3_s1 | both sides stay ≥ 7/8; falls (34 %) and weight_shift (2/8) improve |
| 4 | robot_getup_w7k | w5k (t3k) | `--push-force` 15 → 30 | 30 N ≥ 8/16, 15 N ≥ 14/16, sweep ≥ 43/48 |
| 5 | robot_getup_w8k | w5k (t3k) | `--push-force` 15 → 22 | 30 N above 4/16, 15 N ≥ 14/16, sweep ≥ 43/48 |
| 6 | robot_getup_w7k_s1 | w7k | seed 1 | replicate |

Preflight: PASS 6/6 (preflight now maps `--stop-grace-s` and
`--clock-freeze-stand` into the env it constructs; each job's arg line
also parses through `train_mjx.make_parser`). Fallback: the w5k recipe.

**Tomorrow's evaluation:** get-up: the friction sweep and the push grade at
15 and 30 N for all three, with the per-seed trace (never stood / sank / fell)
at 30 N. Legs: the per-step table for walk9/walk9_s1 against walk3; bal4's
balance_L/R against bal3_s1.

### Night of 2026-10-29 (evaluated 2026-10-30)

All six jobs trained (walk9, walk9_s1, bal4, then w7k, w8k, w7k_s1; the
runner capped w7k_s1 at 592M; the night ended 05:42) and were refereed on
main `85c6c84`. Upstream since the last plan: #165 (firmware: a same-side
`pose ... B` continues the running balance), the ledger and the results page.
No plant, servo or env change on either track, so the comparisons hold.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w7k (30 N training shoves) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 15/16 | 5/16 | w5k 48/48, 14/16, 4/16 |
| w7k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 16/16 | 6/16 | w5k_s1 48/48, 16/16, 4/16 |
| w8k (22 N) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 16/16 | 3/16 | w5k 48/48, 14/16, 4/16 |

Per-seed trace of the 30 N grade (the referee's evaluate wrapped to print
the stand time, the first drop below 0.8 of the stand height after it, and
the end height; 16 episodes per run):

| policy, 30 N | pass | never stood (of them prone) | stood, then down flat | stood, then sank to ~0.28 m |
|---|---|---|---|---|
| w5k + w5k_s1 (night 8) | 8/32 | 13 (11) | 4 | 7 |
| w7k + w7k_s1 | 11/32 | 11 (8) | 6 | 4 |
| w8k | 3/16 | 7 (5) | 3 | 3 |

- **Training at the grade's force barely moved the grade.** 30 N goes from
  4 and 4/16 to 5 and 6/16 (w7k, w7k_s1); 22 N gives 3/16. Hypothesis
  (≥ 8/16) not held on either. Nothing else moved: all three sweep 48/48,
  15 N is 15 – 16/16, and the MJX curves end lower than w5k's
  (`eval/episode_recovered` ~360 – 400 at 30 N, ~710 at 22 N) because
  training shoves end more episodes, not because the policies got worse.
- **The misses are structural.** Half the prone episodes at 30 N still never
  stand. They end flat at 0.05 m: shoved off the 11 s roll, while the
  time-indexed reference moves on to the seat push. There is no state to
  return to, and the 45 s episode has no time for a second attempt. A
  bigger shove in training cannot teach a retry. The real fix would be a
  restart of the reference from the IMU's lying orientation, which belongs
  to the deployment's phase choice (arc step 3). 30 N @ 2 % is a shove a
  second for 45 s, six times the hardware claim's force, so arc step 2
  stops here: **the w5k recipe stays the get-up** (w7k is equal within
  seeds and not worth its extra force).
- **#147: the analytic reference stands on the as-built knees.**
  `getup_ref_analytic.py verify --seq prone_ikc` with the as-built servo set
  (STS3250 at the hip rolls only, STS3215 knees; `G.PJ_DEFAULT` swapped,
  nothing else) stands **144/216** open loop: 72/72 at friction 0.35 and
  0.5, 0/72 at 0.05 (with the design's STS3250 knees it is 216/216). The
  0.05 trace runs the whole roll, the seat push and the rise to 0.39 m
  upright, then tips backwards during the final 1 s "straight" key. That is
  the same failure `rise_keys` documents for faster rises at 0.05, and it
  happens after the stand, where the ET-after-latch policies already hold
  where their replay does not. The rise's static load explains it: along the
  analytic rise the knee's gravity torque is 0.2 – 0.3 of the STS3215's
  stall (0.27 at the centred crouch hold), so the softer knee is not
  stall-bound. #147's evidence (STS3215 knees stall at a 0.27 m crouch;
  g4 0/8) was taken on the hand-tuned, torque-marginal rise of 10-11 – 10-13.
  The servo envelope is the only thing changed here, not the knee servo's
  mass in the plant.

**Decision.** Get-up tonight trains the w5k recipe on the as-built servos
(drop `--servo-3250 knee`; the referee inherits the run's servo set): w9k
from t3k, w10k from w5k_s1 (does the best design-servo policy adapt in one
budget?), w9k_s1 a replicate. If they hold, the robot's get-up needs no new
knee servos, and #147 can close the other way. Arc step 4 (distil, SIL)
then goes on the as-built body. Fallback stays the w5k recipe.

#### One-leg stands and short walks (as-built legs-only plant)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_walk9 | walk3 | + `--stop-grace-s 0.4` | line_1m 4/8, backward 6/8, square_return 8/8, goal_home 8/8, sidestep 8/8 8/8; walk_steps_L/R **3/8, 2/8** (min peak 2.5 / 1.8 cm) | 80/160 (walk3 77), falls 15 % | **held on the stop step** (4/16 low vs 11/16); walk rows not: line_1m 8/8 → 4/8 |
| robot_legs_walk9_s1 | walk9 | seed 1 | line_1m 1/8, backward 5/8, square_return 8/8, goal_home 7/8, sidestep 7/8 8/8; walk_steps_L/R 2/8, 3/8 (min peak 2.3 / 2.6 cm) | 71/160, falls 16 %, gait asym 17 % | stop step **replicates** (6/16 low); line_1m worse |
| robot_legs_bal4 | bal3_s1 | init walk3 → bal3_s1 | balance_L 7/8 (clear 98 %), **balance_R 1/8 (clear 56 %)**, march_in_place 8/8, hip_sway 6/8, weight_shift 4/8, stand_10s 8/8 | 34/96 (bal3_s1 41), falls 33 % | **not held**: training on from the two-sided pass turned it one-sided |

Per-step peaks of the walk_steps episodes (16 per run, same seeds;
`walk_steps_swings` wrapped to print each step; the tool reproduces the
ledger's walk3 row exactly):

| run | first step: median, episodes < 3 cm | steady steps: min, median | stop step: median, episodes < 3 cm |
|---|---|---|---|
| walk3 | 2.6 cm, 9/16 | 3.2, 5.8 cm | 1.5 cm, 11/16 |
| walk9 (stop grace 0.4 s) | 3.0 cm, 7/16 | 2.8, 6.3 cm | **4.6 cm, 4/16** |
| walk9_s1 | 2.4 cm, 10/16 | 3.8, 6.5 cm | **3.8 cm, 6/16** |

- **The stop window worked where seven reward variables had not.** The
  closing step clears 3 cm in 10 – 12 of 16 episodes on both seeds (walk3
  5/16), and walk_steps is 5/16 on both seeds, the best of the line. Tom's
  open question (must the closing step clear 3 cm?) is now less pressing:
  the policy can be trained to do it.
- **The first step is the gate now** (7 and 10/16 below 3 cm). A
  counterfactual on the CPU referee: the walk_steps scenario jumps the gait
  clock to the lead foot's swing at the walk command. Leaving the clock
  running instead moves walk9 only to 6/16 (median 3.8 cm) and walk3 to
  8/16. The jump is a small part of it, so a training-side phase reset is
  not worth an env change.
- **line_1m drifts sideways**: 4/8 and 1/8 (walk3 8/8, walk3_s1 7/8), no
  falls; the misses cross 1 m with y 15 – 41 cm (pass < 15 cm; walk3's mean
  7 cm). It shows on both seeds, so it is probably the window's, but two
  seeds are not enough to say so. walk9_s2 tonight.
- **Balance has a one-sided optimum.** A second 600M from bal3_s1, the only
  two-sided pass, lost balance_R (8/8 → 1/8) while weight_shift and
  hip_sway rose. Over six balance runs balance_R is the weak side in five.
  The asymmetry is not in the DR: the torso-CoM offset is ± in both envs
  (`env_mjx.domain_randomize`, `walker_env`), and so are the IMU mount and
  the roll lash. It needs a diagnosis by day, not another GPU night.
  bal3_s1 stays the balance policy for a floor test (Tom's call).

**Decision.** Walk: walk9 is the walk line's parent. walk9_s2 gives a third
seed for the stop step and the line_1m drift. walk10 = walk9 with
`--w-low-swing` 5 → 10, the one lever with a dose response on the first
step (w 0 / 5 / 15: 14, 9, 8 of 16 low). walk4's cost on the walk rows came
before the stop window existed. No balance job tonight.

#### Tonight (night 10)

Legs track first (Tom 2026-10-24), then get-up; expected end ~04:50.

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk9_s2 | walk9 (c13) | seed 2 | stop-step episodes < 3 cm ≤ 6/16; line_1m says whether the drift is the recipe's |
| 2 | robot_legs_walk10 | walk9 (c13) | `--w-low-swing` 5 → 10 | first-step episodes < 3 cm ≤ 4/16; walk_steps_L/R ≥ 4/8 each; walk rows within 1/8 of walk9 |
| 3 | robot_getup_w9k | w5k (t3k) | knee servos STS3250 → STS3215 (as built) | sweep ≥ 40/48, 15 N ≥ 12/16 on the as-built servos |
| 4 | robot_getup_w10k | w9k (w5k_s1) | init t3k → w5k_s1 | sweep ≥ w9k's, 15 N ≥ 12/16 |
| 5 | robot_getup_w9k_s1 | w9k | seed 1 | replicate |

Preflight: PASS 5/5. Fallback: the w5k recipe (unchanged).

**Tomorrow's evaluation:** get-up: the friction sweep and the 15 / 30 N
grades for w9k, w10k and w9k_s1 against w5k / w5k_s1. If the 0.05 end fails,
look at where (the final straightening, as in the open-loop replay?).
Legs: the per-step table for walk9_s2 and walk10 against walk9, and
line_1m's y at 1 m.

### Night of 2026-10-30 (evaluated 2026-10-31)

All seven jobs ran (walk13_fit, the walk12_full distil, walk13_fit_s1, the
walk13_fit distil, then w9k, w10k, w9k_s1; the night ended 04:54) and were
refereed on main `ca708b0`. Upstream since the last plan: on 10-30 Tom
handed the legs night to the Claude Code robot session, which replaced the
planned walk9_s2 / walk10 with the **deploy-first legs recipe** (#169, #172:
`--action-map full --ctrl-within-stops --servo-kp-scale asbuilt --quantize`,
the stock P32 / D32 the robot runs), trained walk11_full / walk12_full by
day, and fitted the legs plant to the 10-30 flight-recorder stands (#179:
roll play 0.5 – 2°, backlash 0 – 0.25°, zero offset ± 2°). **The legs
comparisons against walk3 / walk9 no longer hold**: the action map, the gains
and the training plant all changed. The get-up plant, servos and env did not
change, so the get-up comparisons hold. No new moving flight-recorder data
since the fit (the overnight 10-31 session is idle ticks only), so the fitted
flags stay.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each; every run on the
as-built servo set, STS3215 knees):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w9k (as-built knees, init t3k) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 3/16 | w5k 48/48, 14/16, 4/16 |
| w10k (init w5k_s1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 2/16 | w9k 48, 16, 3 |
| w9k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 15/16 | 4/16 | w5k_s1 48, 16, 4 |

- **The get-up does not need STS3250 knees.** All three runs on the
  as-built servos match the design-servo recipe on every grade. The
  open-loop reference failed at friction 0.05 (it tips back at the final
  straightening). The policies hold that stand on every seed, as the
  ET-after-latch policies did on the design servos. Times are unchanged
  (supine ph up in 22 – 24 s, prone 34.5 – 35.5 s). The MJX curves end
  where w5k's did (`eval/episode_recovered` 830 – 930, w5k 915). One caveat
  for #147: the plant changes only the knee servo's torque–speed envelope.
  The STS3215's lower mass is not modelled, and that would only help.
- **The warm start does not matter** (w10k from w5k_s1 = w9k from t3k).
- 30 N stays at 2 – 4/16, the structural miss of 10-29 (shoved off the
  time-indexed roll), as expected.

**Decision.** #147 can close the other way: no knee servo order for the
get-up. Arc step 2 on the as-built servos is done. What is left before a
get-up can deploy is the deploy contract the legs track adopted on 10-30. The
get-up still trains with `--servo-kp-scale robot`, which leaves the hip
pitch, hip yaw and ankle pitch at 1×, softer than the robot's real P32
(`asbuilt` = 2.8×). It also trains without the encoder quantizer and with
targets allowed past the stops. Tonight: w11k = w9k with `asbuilt` gains, and
w12k = w11k plus `--quantize --ctrl-within-stops`. The reference touches the
hip pitch −120°, knee −130° and ankle −40° stops, so the clip removes the
margin to push past them, which is the risk being tested. Also tonight, the
first distil of a get-up (w9k → (128, 128), arc step 4). The body question
from 10-28 is still Tom's: every get-up trains on the arms body, and the
as-built robot has no arms.

#### One-leg stands and short walks (as-built legs-only plant, deploy-first)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_walk13_fit | walk12_full (init) | the fitted plant's DR (play 0.5 – 2°, backlash 0 – 0.25°, zero offset ± 2°) | line_1m 4/8, backward 2/8, square_return 6/8, goal_home 7/8, sidestep 7/8 8/8, stand_10s 8/8, stand_off 6/8; walk_steps_L/R 3/8, 1/8 (min peak 2.0 / 1.7 cm) | 67/160, falls 22 %, 102 W | walk_steps ≈ walk12_full's partial (3/8, 2/8); no full card for walk12_full to compare |
| robot_legs_walk13_fit_s1 | walk13_fit | seed 1 | line_1m 4/8, backward 4/8, square_return 7/8, goal_home 7/8, sidestep 8/8 8/8, stand 8/8, stand_off 6/8; walk_steps_L/R **4/8**, 1/8 (min peak 3.0 / 1.5 cm) | **70/160**, falls 21 %, 97 W | the best full-map teacher |
| robot_legs_walk11_full_s128r24 | walk11_full | (distil) | walk_steps 1/8, 3/8; stand_10s 8/8, stand_off 2/8; square_return 6/8 | 56/160, falls 32 %, 138 W | student |
| robot_legs_walk12_full_s128r24 | walk12_full | (distil) | walk_steps 0/8, 1/8; stand_10s 7/8, stand_off 3/8 | 46/160, falls 36 %, 154 W | student (teacher partial: walk_steps 3/8, 2/8, stand 8/8) |
| robot_legs_walk13_fit_s128r24 | walk13_fit | (distil) | walk_steps 3/8, **0/8**; stand_10s 8/8, stand_off 5/8; square_return 2/8 (teacher 6/8) | 52/160 (teacher 67), falls 34 % (22 %), **171 W (102 W)** | student: loses 15 seed-passes and doubles the power |

- **Distillation is the legs track's gate now, not the teacher.** All three
  students lose 10 – 15 seed-passes to their teachers and draw 1.4 – 2× the
  power (stand_10s: 213 W vs the teacher's 80 W). A student that hunts
  around its target looks like that. The distil logs say why. The stock
  recipe (512 envs × 4 chunks × 64 steps × 24 rounds = 3.1M env steps,
  136 s on Warp) ends at val_mse **0.0071** with bc_loss 0.0068, which means
  the student underfits. It is not overfitting. The v5 students on the same
  recipe reached **0.0006** (loco_v41rsi_b_s128r24) and 0.0011
  (v22fix_e_s128), 6 – 12× tighter. In their own rollouts the legs students
  fall in ~300 of 512 slots per round, against the teacher's 61 in round 0.
  The teacher's actions are harder to fit (the stiffer asbuilt gains and the
  quantizer may both make the teacher's actions busier). Whether the limit is
  the SGD or the data is untested, so tonight tests both on the walk13_fit
  teacher against its stock student: `--epochs 24` (4× SGD) and
  `--envs 2048` (4× fresh data per round). Each is a few GPU minutes.
- walk_steps_R is the weak side on every legs run tonight (1/8, 1/8, 0/8;
  min peak 1.5 – 1.7 cm). That is the same side as balance_R's one-sided
  optimum, and the 10-30 fit found an L/R asymmetry (~2° roll zero, or a
  softer left outer edge, and torso CoM 6.8 mm right). The per-step table
  (first step vs stop step) was not run this morning: it is the first job
  for tomorrow's evaluation.

**Decision.** The legs teacher line continues from the best full-map
teacher. walk14 is a second 400M from walk13_fit_s1 (the first 400M on the
fitted plant started from a teacher that trained on the wrong play), and
walk14_s1 is a replicate. Both are distilled with the stock recipe. The
best existing teacher, walk13_fit_s1, gets its first student tonight. The
two distil variants above run on walk13_fit. Tomorrow, re-distil the best
teacher with whichever recipe wins.

#### Tonight (night 11)

Legs first (Tom 2026-10-24 / 10-30), then get-up. Expected: the legs block
~22:00 – 00:10 (two 400M teachers ~48 min each; five distils of 3 – 10 min),
then the get-up distil and three 600M get-up jobs (~1.7 h each), ending
~05:30.

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk13_fit_s1_s128r24 | walk13_fit_s1 | (stock distil) | a deployable student of the best teacher |
| 2 | robot_legs_walk13_fit_s128r24e24 | walk13_fit_s128r24 | `--epochs` 6 → 24 | underfit: val_mse well below 0.007, watts toward ~100 W, ≥ 60/160 |
| 3 | robot_legs_walk13_fit_s128r24n2048 | walk13_fit_s128r24 | `--envs` 512 → 2048 | data coverage: ≥ 60/160, falls toward the teacher's 22 % |
| 4 | robot_legs_walk14 | walk13_fit_s1 (walk13_fit_s1) | init walk12_full → walk13_fit_s1 | not converged on the fitted plant: ≥ 75/160, walk_steps_R ≥ 3/8 |
| 5 | robot_legs_walk14_s128r24 | walk14 | (stock distil) | |
| 6 | robot_legs_walk14_s1 | walk14 | seed 1 | replicate |
| 7 | robot_legs_walk14_s1_s128r24 | walk14_s1 | (stock distil) | |
| 8 | robot_getup_w9k_s128r24 | w9k | (stock distil, gate supine ph + prone fall) | the residual get-up survives distillation: ≥ 14/16 at 0.35 |
| 9 | robot_getup_w11k | w9k (t3k) | `--servo-kp-scale robot` → `asbuilt` | sweep ≥ 46/48, 15 N ≥ 14/16 on the robot's gains |
| 10 | robot_getup_w12k | w11k (t3k) | + `--quantize --ctrl-within-stops` | no loss vs w11k |
| 11 | robot_getup_w11k_s1 | w11k | seed 1 | replicate (may be capped) |

Preflight: PASS 11/11. Fallback: the w9k recipe (the as-built-servo
get-up; w5k recipe before).

**Tomorrow's evaluation:** legs: every student against its teacher (the
student is the headline) and the two distil variants against the stock
student (val_mse, watts, overall); walk14 / walk14_s1 against walk13_fit_s1;
the per-step table (first / steady / stop) for walk_steps_R. Get-up: the
sweep and the 15 / 30 N grades for w11k, w12k and w11k_s1 against w9k; the
w9k student's sweep against w9k.
