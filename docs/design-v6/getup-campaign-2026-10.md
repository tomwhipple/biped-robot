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
| `getup_ref_v6ar_prone_ikh.npz` | prone, arms idle (+15°) | as prone_ikc, arms **hanging** (shoulder 0) through the rise, rise re-planned (below) | 216/216 | ~33 s |
| `getup_ref_v6ar_prone_ikf.npz` | prone, arms idle (+15°) | as prone_ikc, arms **45° forward** through the rise, rise re-planned (below) | 216/216 | ~33 s |

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

**The arms through the rise (2026-10-24, Tom: "why does it always put its arms
straight back when coming out of the crouch? … moving the CG aft").** Every
reference so far holds REST (shoulder 60°, i.e. 60° back of hanging) from the
crouch on: a leftover of the seat push's hands-behind push, not a choice. The
static CoM (the reference's `_levelled_pose`, from the sole centre):

| pose | arms REST (60° back) | hanging (0°) | 45° forward |
|---|---|---|---|
| crouch, ankle −25 (prone_ik) | −40 mm | −29 mm | −15 mm |
| crouch, ankle −35 (prone_ikc's hold) | −14 mm | −6 mm | +8 mm |
| the REST rise's waypoints | −11 … +4 mm | +5 … +15 mm | +19 … +28 mm |

The REST rise is balanced *for* arms back (the legs lean forward to carry them).
`rise_path(arms=…)` now re-plans the rise for another arm pose; with the default
±12 mm band the torque-minimising DP parks the new rises at its back edge
(−2 … −12 mm), so the new ones use a band of −4 … +12 mm (`RISE_DX`) and come out
centred (−4 … +3 mm, as the REST rise). The crouch hold is prone_ikc's (hands
down, ankle −35); the arms go to the new pose on the first rise waypoint and
stay there standing. Referee replay, no RL (`--ref-replay`, the
`diag_refreplay_prone_ik` zero-residual run), supine ph / prone fall:

| reference | 0.05 | 0.35 | 0.5 | sweep | stands at 0.05 |
|---|---|---|---|---|---|
| prone_ikc (arms REST) | 5/8 6/8 | 8/8 8/8 | 8/8 8/8 | 43/48 | 22.6 / 34.0 s |
| **prone_ikh** (hanging) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 22.0 / 33.2 s |
| **prone_ikf** (45° forward) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 22.2 / 33.4 s |

The 0.05 "stand, then fall backwards after straightening" misses — the ones
the replay said need feedback, and w0k_fb1024s1's stand-then-tip at every
friction (night 4) — are gone from the replay with either arm pose; ikh uses
slightly less power (5.8 / 8.9 / 10.8 W vs 6.2 / 9.6 / 11.6). **Suggested
next:** w0k's recipe with `--rise-ref getup_ref_v6ar_prone_ikh.npz` (and a
seed replicate): its replay is already 48/48, so a policy can at best match it; the test
is whether the stand-then-tip seed variance (46 vs 14/48) goes.

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
   deploy action contract `--quantize --ctrl-within-stops` — held, w12k
   48/48, 15 N 14/16: the get-up trains on the full deploy contract)*
3. **One policy for both starts**: largely solved without env work (night
   1). The prone reference's roll ends on the back and the supine seat push
   follows, so the prone policy gets up from supine when it starts at that
   phase (`recover_supine_ph`); the robot picks the phase from the IMU.
   Left: a side start, and the deployment choosing the phase.
4. **Hand over to the walker** (the reference ends arms forward: a 2 s ramp
   today), distil, SIL — the deployment path of training.md §9. *(night 11:
   the first get-up student, w9k → (128, 128), loses 16/48 of the sweep and
   10/16 at 15 N; night 12: distil data, and training the (128, 128) policy
   directly with PPO, `--policy-hidden` — the w12k student sweeps 44/48 but
   keeps 4/16 at 15 N; the (128, 128) PPO policy sweeps 31 and 48/48 on two
   seeds, 15 N 7 and 8/16; night 13: distil noise / capacity, a (256, 256)
   PPO net, the small net's seed spread and residual 0.075 — **the DART
   student w12k_s128r24an3 (`--act-noise 0.3`) is the first (128, 128)
   get-up at the teacher's level: 48/48, 15 N 16/16**; PPO small nets stay
   at 7 – 9/16 whatever their size or residual; night 14: its replicates,
   and the get-up on the robot's fast actuator and firmware shaper — held:
   w17k 48/48, 15 N 16/16; w18k through the shaper 48/48, 16/16; the DART
   student replicates on a second teacher, 47/48, 16/16; night 15: the
   episode start seeded at the lying pose, `--seed-chain-at-q` — held, 48/48
   and 15 N 16/16 on two seeds, its DART student w19k_s128r24an3 48/48, 16/16,
   and all of them 16/16 on the measured sole friction μ 0.45 and 0.3;
   #214's smoothness terms bought nothing and were dropped; night 16: the
   firmware motion limiter, `--fw-limit` 200,3000 and 200,2000)*

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
| robot_getup_w9k_s128r24 | 10-31 | w9k | (distil) the stock 24-round (128, 128) student | arc step 4: the residual get-up survives distillation, supine ph + prone fall ≥ 14/16 at 0.35 | 0/8 | 8/8 | 6/8 (36.4 s) | 6/8 (36.5 s) | gate **held at the edge** (14/16), the rest **lost**: 0.05: 1/8, 7/8; 0.5: 8/8, 2/8 = 32/48 (teacher 48); 15 N 6/16 (16), 30 N 0/16. Sinks after standing at 0.05, never stands from prone at 0.5 and under shoves. bc 0.001 vs val 0.016: overfit |
| robot_getup_w11k | 10-31 | w9k (t3k) | `--servo-kp-scale robot` → `asbuilt` (factory P32: hip pitch, hip yaw, ankle 1× → 2.8×) | the get-up holds on the gains the robot runs: sweep ≥ 46/48, 15 N ≥ 14/16 | 0/8 | 7/8 | 8/8 (35.3 s) | 8/8 (35.2 s) | **held** at the edge: 0.05: 8/8, 8/8; 0.5: 7/8, 8/8 = 46/48; 15 N 14/16, 30 N 3/16 (w9k 48, 16, 3) |
| robot_getup_w12k | 10-31 | w11k (t3k) | + `--quantize --ctrl-within-stops` (the rest of the legs deploy contract) | no loss vs w11k: the deployable get-up recipe | 0/8 | 8/8 | 8/8 (34.3 s) | 8/8 (34.3 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**; 15 N 14/16, 30 N 5/16. The first get-up on the full deploy contract |
| robot_getup_w11k_s1 | 10-31 | w11k | seed 1 | replicate (may be capped) | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | trained the full 600M; **replicates**: 0.05: 8/8, 8/8; 0.5: 7/8, 8/8 = 47/48; 15 N 15/16, 30 N 5/16 |
| robot_getup_w12k_s128r24 | 11-01 | w12k | (distil) stock student of the deploy-contract teacher | the deployable teacher's baseline student (expect the w9k student's 32/48) | 0/8 | 8/8 | 8/8 (34.4 s) | 8/8 (34.4 s) | better than expected on friction, lost on shoves: 0.05: 4/8, 8/8; 0.5: 8/8, 8/8 = **44/48** (w9k student 32); 15 N **4/16** (teacher 14), 30 N 1/16. bc 0.0011, val 0.014: still overfit |
| robot_getup_w12k_s128r24n1024 | 11-01 | w12k_s128r24 | distil `--envs` 512 → 1024 | the get-up student overfits: more data, val_mse < 0.008, sweep ≥ 40/48 | 0/8 | 8/8 | 6/8 (34.1 s) | 6/8 (34.1 s) | **not held**: val 0.0185 (worse than 0.014); 0.05: 5/8, 3/8; 0.5: 8/8, 8/8 = 38/48 (stock 44); 15 N 8/16, 30 N 1/16. More data per round does not fix it |
| robot_getup_w13k | 11-01 | w12k (fresh) | policy net (512, 256, 128) → (128, 128) (`--policy-hidden`, #180; forces a fresh init) | the residual policy needs no big net: sweep ≥ 44/48, 15 N ≥ 12/16, deployable with no distil | 0/8 | 1/8 | 8/8 (33.0 s) | 8/8 (33.0 s) | **not held** on this seed: supine ph stands at 21.7 s, then sinks to 0.29 m. 0.05: 4/8, 8/8; 0.5: 2/8, 8/8 = 31/48; 15 N 7/16, 30 N 0/16. MJX `episode_recovered` plateaus by 160M like the big net's |
| robot_getup_w14k | 11-01 | w12k (fresh) | no `--init-from` (from scratch, (512, 256, 128)) | control for w13k: the reference carries a fresh policy, sweep ≥ 44/48 | 0/8 | 8/8 | 8/8 (33.0 s) | 8/8 (33.0 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**; 15 N 12/16, 30 N 4/16. The t3k init is not needed |
| robot_getup_w12k_s1 | 11-01 | w12k | seed 1 | replicate of the deploy-contract recipe | 0/8 | 8/8 | 8/8 (34.9 s) | 8/8 (34.9 s) | **replicates**: 48/48; 15 N **15/16**, 30 N 2/16 |
| robot_getup_w13k_s1 | 11-01 | w13k | seed 1 | replicate (likely capped) | 0/8 | 8/8 | 8/8 (33.0 s) | 8/8 (33.0 s) | capped at 532M. **48/48**; 15 N 8/16, 30 N 0/16. The (128, 128) recipe over 2 seeds: 31, 48; 15 N 7, 8 (big net 12 – 15) |
| robot_getup_w12k_s128r24an3 | 11-02 | w12k_s128r24 | distil `--act-noise` 0 → 0.3 (noise on the executed actions) | the student's buffer is too narrow for post-shove states: val < 0.01, 15 N ≥ 8/16, sweep ≥ 44/48 | 0/8 | 8/8 | 8/8 (33.7 s) | 8/8 (33.6 s) | **held**: 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = **48/48**; 15 N **16/16** (teacher 14, stock student 4), 30 N 5/16. The first (128, 128) get-up at the teacher's level. val 0.022 (bc 0.0072): its val is on the noisier buffer, so it does not compare with the stock student's 0.014 |
| robot_getup_w12k_s512r24 | 11-02 | w12k_s128r24 | distil `--hidden` → 512, 256, 128 (teacher size) | the distil loss is the DAgger recipe, not capacity: 15 N stays ≤ 8/16 | 0/8 | 8/8 | 8/8 (34.0 s) | 8/8 (34.0 s) | **not held**: capacity alone also closes the gap. 0.05: 8/8, 8/8; 0.5: 8/8, 8/8 = 48/48; 15 N **16/16**, 30 N 4/16; bc 0.00036, val 0.0077 (stock 0.0011, 0.014). Not deployable at this size |
| robot_getup_w15k | 11-02 | w13k (fresh) | `--policy-hidden` 128,128 → 256,256 | the small net's push loss is capacity: sweep 48/48, 15 N ≥ 12/16 | 0/8 | 8/8 | 8/8 (33.0 s) | 8/8 (33.0 s) | **not held**: 48/48 but 15 N 7/16, 30 N 1/16, the same as the (128, 128) nets. The PPO small-net push loss is not capacity |
| robot_getup_w13k_s2 | 11-02 | w13k | seed 2 | replicate: is 31 or 48/48 typical | 0/8 | 8/8 | 8/8 (33.7 s) | 8/8 (33.7 s) | 0.05: 6/8, 8/8; 0.5: 8/8, 8/8 = 46/48; 15 N 8/16, 30 N 2/16. The (128, 128) PPO recipe over 3 seeds: 31, 48, 46; 15 N 7, 8, 8 |
| robot_getup_w16k | 11-02 | w13k (fresh) | `--rise-ref-residual` 0.15 → 0.075 | less authority stops the small net's post-stand sink: sweep ≥ 46/48 (may be capped) | 0/8 | 8/8 | 8/8 (33.0 s) | 8/8 (33.0 s) | **held** on the sweep (48/48, no sink on this seed); 15 N 9/16, 30 N 1/16: shoves unchanged |
| robot_getup_w12k_s1_s128r24an3 | 11-03 | w12k_s128r24an3 | teacher w12k → w12k_s1 (distil `--act-noise 0.3`) | the noise recipe, not one lucky teacher: sweep ≥ 46/48, 15 N ≥ 14/16 | 0/8 | 8/8 | 8/8 (34.6 s) | 8/8 (34.6 s) | **held**: 0.05: 8/8, 8/8; 0.5: 7/8, 8/8 = 47/48; 15 N **16/16**, 30 N 5/16. The DART recipe over 2 teachers: 48, 47; 15 N 16, 16 |
| robot_getup_w12k_s128r24an6 | 11-03 | w12k_s128r24an3 | distil `--act-noise` 0.3 → 0.6 | brackets the noise: 30 N above 5/16, sweep ≥ 46/48 | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | **not held**: 47/48, 15 N 16/16, 30 N 6/16 (an3 5/16): within seed noise. More noise does not reach 30 N |
| robot_getup_w17k | 11-03 | w12k (t3k) | the actuator: 3-stage `--act-lag 2,12` → the measured fast servo (`--act-lag 2.65,8 --act-lag-order 1 --act-lag-scale sts2026 --act-delay-types sts3215=0:2,sts3250=0:1`) | the get-up is slow enough that the robot's actuator costs nothing: sweep ≥ 46/48, 15 N ≥ 12/16 | 0/8 | 8/8 | 8/8 (34.4 s) | 8/8 (34.4 s) | **held**: 48/48; 15 N **16/16** (w12k 14), 30 N 3/16 (5). The fast actuator costs nothing (measured-stops plant `v6ar_meas`) |
| robot_getup_w17k_s128r24an3 | 11-03 | w12k_s128r24an3 | teacher w12k → w17k | the deployable student carries over to the fast actuator: sweep ≥ 46/48, 15 N ≥ 12/16 | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | **held**: 0.5: 7/8, 8/8 = 47/48; 15 N **16/16**, 30 N 2/16 |
| robot_getup_w18k | 11-03 | w17k (t3k) | + `--fw-shaper 10 --fw-goal-speed` (the firmware chain the legs deploy through) | the get-up trains through the deploy shaper: sweep ≥ 46/48, 15 N ≥ 12/16 | 0/8 | 8/8 | 8/8 (33.7 s) | 8/8 (33.7 s) | **held**: **48/48**; 15 N **16/16**, 30 N 5/16. Graded through the shaper (`fw_shaper_hz` 10 in config.json → the referee's env). The get-up on the full deploy chain |
| robot_getup_w17k_s1 | 11-03 | w17k | seed 1 | replicate (may be capped) | 0/8 | 8/8 | 8/8 (33.3 s) | 8/8 (33.3 s) | trained the full 600M, but on the OLD plant `bimo_biped_v6ar.xml` (its job had no `--xml`): 48/48, 15 N 15/16, 30 N 3/16. Replicates w17k across the plant change |
| robot_getup_w18k_s128r24an3 | 11-04 | w17k_s128r24an3 | teacher w17k → w18k (+ the shaper; distil `--act-noise 0.3`) | the deployable student carries through the shaper: sweep ≥ 46/48, 15 N ≥ 14/16 | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | **held**: **48/48**; 15 N **16/16**, 30 N 5/16. Sole μ 0.45 and 0.3: 16/16 each |
| robot_getup_w19k | 11-04 | w18k (t3k) | + `--seed-chain-at-q` (the servo chain starts at the lying pose, as the firmware's shaper; main `6fac9db`) | the get-up does not lean on the start yank: sweep ≥ 46/48, 15 N ≥ 14/16 | 0/8 | 8/8 | 8/8 (34.4 s) | 8/8 (34.4 s) | **held**: **48/48**; 15 N **16/16**, 30 N 6/16 (w18k 48, 16, 5). `seed_chain_at_q` true in config.json. Sole μ 0.45 and 0.3: 16/16 each |
| robot_getup_w19k_s128r24an3 | 11-04 | w18k_s128r24an3 | teacher w18k → w19k | the deploy-faithful student: sweep ≥ 46/48, 15 N ≥ 14/16 | 0/8 | 8/8 | 8/8 (33.4 s) | 8/8 (33.4 s) | **held**: **48/48**; 15 N **16/16**, 30 N 3/16. Sole μ 0.45 and 0.3: 16/16 each. **The get-up candidate for the robot** |
| robot_getup_w19k_s1 | 11-04 | w19k | seed 1 | replicate | 0/8 | 8/8 | 8/8 (34.2 s) | 8/8 (34.2 s) | **replicates**: 48/48; 15 N 16/16, 30 N 4/16 |
| robot_getup_w20k | 11-04 | w19k (t3k) | + `--w-qacc 4e-5 --w-action-acc 1.0` (#214's smoothness terms, the legs' weights) | lower wobble at no cost: sweep ≥ 46/48, 15 N ≥ 14/16 (may be capped) | 0/8 | 8/8 | 8/8 (33.5 s) | 8/8 (33.5 s) | **not held**: trained the full 600M. 0.5: 7/8, 8/8 = 47/48; 15 N 15/16, 30 N 5/16. Wobble 0.37 / 0.41 rad/s (w19k 0.40 / 0.43, within card noise) for +13 – 19 % watts; PPO KL 0.22 (w19k 0.022). Dropped |
| robot_getup_w21k | 11-05 | w19k (t3k) | + `--fw-limit 200,3000` (the firmware motion limiter, #215; the legs line's setting) | the residual absorbs the 200 °/s cap (the reference's roll reaches 267 – 349 °/s on hip pitch, knee, shoulders): sweep ≥ 46/48, 15 N ≥ 14/16, stands within +2 s of w19k | | | | | |
| robot_getup_w21k_s128r24an3 | 11-05 | w19k_s128r24an3 | teacher w19k → w21k | the DART student carries through the limiter: sweep ≥ 46/48, 15 N ≥ 14/16 | | | | | |
| robot_getup_w22k | 11-05 | w21k (t3k) | `--fw-limit` 200,3000 → 200,2000 (the firmware's `limit on` default) | no loss vs w21k (the reference's peak acceleration is 1170 °/s²): sweep ≥ 46/48, 15 N ≥ 14/16 | | | | | |
| robot_getup_w21k_s1 | 11-05 | w21k | seed 1 | replicate | | | | | |
| robot_legs_omni7_rs | 11-06 | omni5_dqlag | the robot actuator (peak joint speed + 60 ms dead time) | `robot` steps5 rate and margin falls down, six directions kept | | | | | |
| robot_legs_omni8_rs_dq25 | 11-06 | omni7_rs | `--dq-obs-window 2,5` | delay margin: dq5, dq6 0 falls | | | | | |
| robot_legs_omni9_rs_crouch | 11-06 | omni7_rs | crouch channel | low rate, knee bend | | | | | |
| robot_legs_omni10_rs_yk | 11-06 | omni7_rs | yaw kicks | stride1 yk3 falls to 0 | | | | | |
| robot_legs_omni11_rs_lowswing | 11-06 | omni7_rs | lower swing | less rate and yaw, >= 3 cm kept | | | | | |
| robot_legs_omni12_rs_grace07 | 11-06 | omni7_rs | `--stop-grace-s 0.7` | stride1 late40 trips down | | | | | |
| robot_legs_omni13_rs_gh1h20 | 11-06 | omni7_rs | hinge 1 above 2.0 | less runaway rate, directions kept | | | | | |
| robot_legs_prec25_f075_js20_s2t | 11-08 | prec16_f075_js20 | `--feet-phase-s2` 0.002 → 0.0008 | median swing peak 1.2 → ≥ 1.8 cm, prec16's margin kept | | | | | **not held**: median swing peak 1.2 → 1.5 cm, lowest step 0.3 cm; x1.5 lost (8/8); fwd yaw −66 |
| robot_legs_prec26_f075_js20_sh35 | 11-08 | prec16_f075_js20 | `--swing-height` 0.025 → 0.035 (reference lift stays 2 cm) | clearance up without prec22's knee-speed cost | | | | | **clearance held, margin lost**: best clearance yet (median 1.6 cm, walk_steps 1.8/1.8, 12 % ≥ 3 cm), yaw +9/−8, knees 62/53; x1.5 6/6 |
| robot_legs_prec27_f075_js20_ls2 | 11-08 | prec16_f075_js20 | `--w-low-swing 2` | lowest step 0.3 → ≥ 1 cm, 0 falls kept | | | | | **not held**: lowest step 0.4/0.5 cm, ≥ 3 cm 1 %; keeps x1.5 0/0 but back falls 3/8 |
| robot_legs_prec28_f075_js20_crouch_mirror | 11-08 | prec24_f075_js20_crouch | `--w-mirror-loss 1.0` | symmetric knees on the knee-bend policy | | | | | **not held** (confounded: 229M + 70.6M resume after 4 OOMs): knees even (53/55) but x1.5 8/8 (7/7 at crouch 0.9) |
| robot_legs_prec29_f075_js20_crouch_s1 | 11-08 | prec16_f075_js20 | seed 15 (prec24 recipe) | replicate | | | | | **half held**: 0 falls on every probe but x1.5 8/8 at 1.0 and 0.9; prec24's margin was the draw; turn R short replicates (−82) |
| robot_legs_prec30_f075_js20_head03 | 11-08 | prec16_f075_js20 | `--w-heading 0.3` | steps5 yaw +38 → < +20 deg, x1.5 kept | | | | | **not held**: steps5 yaw +38 → +28 only; x1.5 3/3 |
| robot_legs_prec31_f075_js30 | 11-08 | prec16_f075_js20 | `--w-joint-speed` 0.2 → 0.3 | time above 180 deg/s −25 %, six directions kept | | | | | **speed held, margin lost**: time above 180 0.52 → 0.30 (lowest yet), knees 72/61 (range 56/45); x1.5 6/8 |
| robot_legs_prec32_f075_js20_crouch_s2t | 11-08 | prec24_f075_js20_crouch | `--feet-phase-s2 0.0008` | the bent-knee gait lifts ≥ 1.8 cm | | | | | teacher only (distil left for tonight): x1.5 6/6 at 1.0, 0/0 at crouch 0.9; median 1.5 cm; back falls 3/8 at 1.0 |
| robot_legs_prec33_f075_js20_mirror_s1 | 11-08 | prec16_f075_js20 | seed 15 (prec20 recipe) | replicate | | | | | |
| robot_legs_prec34_f075_js20_sh35_lg | 11-09 | prec26_f075_js20_sh35 | `--loop-gain-range 1.0,1.6` | x1.5 back to 0/0 with prec26's clearance kept | | | | | |
| robot_legs_prec35_f075_js20_lg | 11-09 | prec16_f075_js20 | `--loop-gain-range 1.0,1.6` | control: x1.5 stays 0/0, x2 improves, no direction lost | | | | | |
| robot_legs_prec36_f075_js20_sh35_s1 | 11-09 | prec16_f075_js20 | seed 15 (prec26 recipe) | replicate | | | | | |
| robot_legs_prec37_f075_js20_sh35_s2t | 11-09 | prec26_f075_js20_sh35 | `--feet-phase-s2 0.0008` | the clearance levers stack: median ≥ 2.0 cm | | | | | |
| robot_legs_prec38_f075_js20_crouch_sh35 | 11-09 | prec24_f075_js20_crouch | `--swing-height 0.035` | bent-knee gait lifts like prec26 | | | | | |
| robot_legs_prec39_f075_js20_sh45 | 11-09 | prec26_f075_js20_sh35 | `--swing-height` 0.035 → 0.045 | median ≥ 2.0 cm, mean knee < 90 deg/s | | | | | |
| robot_legs_prec40_f075_js20_crouch_lg | 11-09 | prec29_f075_js20_crouch_s1 | `--loop-gain-range 1.0,1.6` | repair: x1.5 back to 0/0 at crouch 0.9 and 1.0 | | | | | |
| robot_legs_prec41_f075_js30_lg | 11-09 | prec31_f075_js30 | `--loop-gain-range 1.0,1.6` | repair: x1.5 back to 0/0, time above 180 ≤ 0.33 kept | | | | | |
| robot_legs_prec46_sw35 | 11-10 | prec16_f075_js20 | `--mimic-sway 0.035` | stride1 robot shift ≥ 0.5 both sides, R lift ≥ 2 cm | | | | | **not held**: shift −0.27/+0.28 (prec16 −0.21/+0.16), R lift 1.9 cm; x1.5 0/0, six directions |
| robot_legs_prec47_sw50 | 11-10 | prec16_f075_js20 | `--mimic-sway 0.05` | a bigger sway reaches ≥ 0.6 | | | | | **not held**: −0.24/+0.20; side L 0.34, turn L +97 |
| robot_legs_prec48_s2t_sw35 | 11-10 | prec37_f075_js20_sh35_s2t | `--mimic-sway 0.035` | prec37's lift kept, with the shift | | | | | **not held**: lift 4.9/2.6 kept, shift −0.33/+0.34, x1.5 lost (8/8) |
| robot_legs_prec49_ws2_sw35 | 11-10 | prec43_ws2 | `--mimic-sway 0.035` | the walk-shift kernel gets gradient: ≥ 0.6 | | | | | **not held**: best shift of the line (−0.39/+0.37; steps5 −0.57/+0.58) but side L lost (+0.01) |
| robot_legs_prec50_sw35_s1 | 11-10 | prec16_f075_js20 | seed 15 (prec46 recipe) | replicate | | | | | replicates prec46's null: −0.26/+0.23, R lift 1.4 cm |
| robot_legs_prec51_sw35_lead0 | 11-10 | prec16_f075_js20 | `--mimic-sway-lead 0` (prec46 recipe) | lead 0.785 beats 0 | | | | | **not held**: −0.29/+0.28 = prec46; the lead makes no difference |
| robot_legs_prec52_march | 11-10 | prec16_f075_js20 | march slice, reference on (`--march-mix 0.15 --march-lift 0.08 --mimic-march 0.035` …) | 5 march steps ≥ 3 cm on both feet, shift ≥ 0.6 | | | | | **half held**: marches on the **R foot only** (6.3 cm on every R step, L ≤ 0.75 cm), shift R-up +0.62; 0 falls |
| robot_legs_prec53_march_noref | 11-10 | prec16_f075_js20 | prec52 without `--mimic-march` (control) | lifts, with less shift than prec52 | | | | | no lift at all (0/5 steps): the reference is what finds the march |
| robot_legs_prec54_march6 | 11-10 | prec16_f075_js20 | prec52 with `--march-lift 0.06` (c6 = 0) | c4 alone is enough | | | | | **not held**: no lift at all |
| robot_legs_prec55_march_s1 | 11-10 | prec16_f075_js20 | seed 15 (prec52 recipe) | replicate | | | | | one-sided again, **L only** (8.6 cm), shift L-up −0.64; x1.5 6/6 |
| robot_legs_prec56_march_s2 | 11-10 | prec16_f075_js20 | seed 16 (prec52 recipe) | replicate | | | | | one-sided, **L only** (5.2 cm), shift −0.70 |
| robot_legs_prec57_march_rs | 11-10 | prec16_f075_js20 | prec52 + `--roll-series-k hip=10:25,ankle=16:40` | the march survives the soft roll plant | | | | | never found the march (lift_ok flat at ~1 for 300M); x1.5 8/8 |
| robot_legs_prec58_sw35_rs | 11-10 | prec16_f075_js20 | prec46 + `--roll-series-k` | lift and directions kept on the refit plant | | | | | lift 2.7/2.2, shift −0.26/+0.14; stride1 0.6 refit 4/8 falls |
| robot_legs_prec59_march_rs_s1 | 11-10 | prec16_f075_js20 | seed 15 (prec57 recipe) | replicate | | | | | one-sided, **L only** (6.3 cm), shift −0.76; side L lost (+0.02), stride1 0.6 refit 5/8 falls |
| robot_legs_prec60_march_rs_s2 | 11-10 | prec16_f075_js20 | seed 16 (prec57 recipe) | replicate | | | | | one-sided, **R only** (5.3 cm), shift +0.53; side L lost (+0.02), x1.5 8/8, stride1 0.6 refit 7/8 |
| robot_legs_prec61_mrs_mir (+ _s1, _s2) | 11-11 | prec59_march_rs_s1 | `--w-mirror-loss 1.0` | march5 robot lifts BOTH feet ≥ 3 cm on every step in ≥ 2 of 3 seeds | | | | | never run: moved to queue/held/ on 11-11 (spring plant replaced by the play plant, WHY-1111-spring-march-replaced.txt) |
| robot_legs_prec64_march_rs_mir (+ _s1, _s2) | 11-11 | prec16_f075_js20 | prec57 recipe + `--w-mirror-loss 1.0` | the march is found in 3 of 3 seeds, on both feet | | | | | never run: moved to queue/held/ on 11-11 (spring plant replaced by the play plant, WHY-1111-spring-march-replaced.txt) |
| robot_legs_prec67_mrs_cont (+ _s1, _s2) | 11-11 | prec59_march_rs_s1 | none: +300M on prec59's recipe (control for prec61) | the second foot does not come up with more training alone | | | | | never run: moved to queue/held/ on 11-11 (spring plant replaced by the play plant, WHY-1111-spring-march-replaced.txt) |
| robot_legs_prec68_march_pp (+ _s1) | 11-11 | prec59_march_rs_s1 | roll chain: `--roll-play-deg L_hip=4:10,R_hip=6:10,L_ankle=2:3,R_ankle=0:2` for `--roll-series-k` + lash (P1) | the march still lifts, both feet ≥ 3 cm in ≥ 1 of 2 seeds, torso level within 3 deg | | | | | **not held**: 2/2 **L only** (4.9–5.4 cm, R ≤ 0.1), torso 4.8–5.9 deg over the stance foot; both sidesteps lost (dy ≤ 0.10) on both plants |
| robot_legs_prec69_slow_pp (+ _s1, _s2) | 11-11 | prec59_march_rs_s1 | P1 + `--gait-freq 0.5,0.55 --mimic-sway 0.05` (P2) | more two-sided lifts with a level body than P1 | | | | | **not held**: 3/3 **L only** (4.6–5.0 cm), roll −4.6 to −6.0 deg; sidesteps lost; fwd 0.46–0.65 |
| robot_legs_prec70_slowlift_pp (+ _s1, _s2) | 11-11 | prec59_march_rs_s1 | P2 + `--mimic-march 0.05 --swing-height 0.035` (P3) | the lift rewards fire on both feet | | | | | **not held**: 3/3 **L only**, higher (5.8–6.6 cm); R never; sidesteps lost |
| robot_legs_prec71_mir_pp (+ _s1) | 11-11 | prec59_march_rs_s1 | P1 + `--w-mirror-loss 1.0` (P4; `--num-evals 12`) | a two-sided march where P1 is one-sided | | | | | **not held**: 2/2 the march **flips to R only** (6.4 / 5.0 cm), L stays down; 0 guard trips on march5 (P1–P3 trip 8/8); sidesteps lost |
| robot_legs_prec72_walk_pp (+ _s1, _s2) | 11-12 | prec16_f075_js20 | roll chain only: `--roll-play-deg` (per side) for `--roll-lash-deg 0.5,2.0`, no march | six directions kept and x1.5 0/0 on the play grade plant (prec16 there: 8/8 falls) | | | | | |
| robot_legs_prec68_march_pp_s2 / prec71_mir_pp_s2 | 11-12 | prec59_march_rs_s1 | seed 16 (replicates of P1 / P4) | P1 stays L-only; P4 stays R-only | | | | | |

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

### Night of 2026-10-31 (evaluated 2026-11-01)

All eleven jobs ran (legs 22:00 – 23:56, get-up 23:56 – 04:14; DST ended at
02:00, so w12k's 01:46 → 02:30 is 1 h 44 min) and were refereed on main
`62db430`. Upstream since the last plan: nothing but the planner's own
commits. The plants, servos and envs did not change, so every comparison
below holds. No new moving flight-recorder data (the last session is still
the 10-31 idle ticks), so the fitted legs flags stay.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each; as-built servos):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w11k (`asbuilt` gains) | 8/8 8/8 | 7/8 8/8 | 7/8 8/8 | 46/48 | 14/16 | 3/16 | w9k 48, 16, 3 |
| w12k (+ quantizer, clip within stops) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 14/16 | 5/16 | w11k 46, 14, 3 |
| w11k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 7/8 8/8 | 47/48 | 15/16 | 5/16 | w11k 46, 14, 3 |
| w9k_s128r24 (stock student of w9k) | 1/8 7/8 | 8/8 6/8 | 8/8 2/8 | 32/48 | 6/16 | 0/16 | teacher 48, 16, 3 |

- **The get-up holds on the full deploy contract.** On the robot's factory
  P32 gains (w11k, w11k_s1: 46 and 47/48) and with the quantizer and the
  clip to the stops on top (w12k: 48/48) it loses at most two sweep trials
  and one or two at 15 N. The clip at the hip pitch −120°, knee −130° and
  ankle −40° stops costs nothing. Times are a second faster (supine ph
  22.0 s, prone 33.4 – 35.3 s). 30 N stays at 3 – 5/16, the structural miss.
  **w12k is the deployable get-up teacher** and the new parent.
- **The first get-up student does not survive distillation.** w9k →
  (128, 128) passes its in-job gate at the edge (14/16 at 0.35) but loses
  16 of 48 sweep trials and 10 of 16 at 15 N. It fails in the two places
  the teacher learned beyond the reference: at 0.05 it stands and then
  sinks (end height 0.29 m mean, the post-stand sink that the
  ET-after-latch training removed from the teachers), and at 0.5 and under
  shoves it never stands from prone (end height 0.14 / 0.11 m). Unlike the
  legs students it does not underfit: bc_loss 0.0010, val_mse 0.0162. It
  fits its buffer and does not generalise, which is the signature of too
  little or too narrow data, not too small a net.

**Decision.** Arc step 4 is the get-up's gate now. Two routes, tested
tonight. (1) Better distillation: the stock student of w12k (baseline) and
the same with 2× fresh data per round (`--envs 1024`, the env count the
teacher trains at, so the all-floor contact pool is known to fit the GPU;
4× was not risked). (2) No distillation: train the (128, 128) firmware net
directly with PPO (`--policy-hidden 128,128`, PR #180). The policy is a
0.15 residual around a reference that replays at 43/48 on its own, so a
small net may be enough. The shapes force a fresh init, so w14k (the w12k
recipe from scratch at (512, 256, 128)) is the control that separates net
size from init. Then w12k_s1 and w13k_s1 as replicates. The body question
(every get-up trains on the arms body; the as-built robot has none) is
still Tom's.

#### One-leg stands and short walks (as-built legs-only plant, deploy-first)

| run | parent | the one variable | key rows | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_walk14 | walk13_fit_s1 (init) | init walk12_full → walk13_fit_s1 (second 400M on the fitted plant) | line_1m 3/8, backward 6/8, square_return 7/8, goal_home 7/8, sidestep 7/8 8/8, stand 8/8, stand_off 6/8; walk_steps_L/R **5/8, 3/8** (min peak 2.9 / 2.6 cm) | **79/160**, falls 21 %, 94 W | **held** (≥ 75, R ≥ 3/8): the best deployable teacher |
| robot_legs_walk14_s1 | walk14 | seed 1 | walk_steps 1/8, 1/8; stand 8/8, stand_off 5/8 | 67/160, falls 22 %, 102 W | replicate does not hold: seed spread 79 / 67 |
| walk13_fit_s1_s128r24 | walk13_fit_s1 | (stock distil) | walk_steps 1/8, 0/8; stand 8/8 | 41/160 (teacher 70), 133 W (97) | student |
| walk13_fit_s128r24e24 | walk13_fit_s128r24 | `--epochs` 6 → 24 | walk_steps 0/8, 2/8; stand 7/8 | 46/160 (stock 52), val_mse 0.0066 (0.0071) | **not held**: 4× SGD barely moves the fit |
| walk13_fit_s128r24n2048 | walk13_fit_s128r24 | `--envs` 512 → 2048 | walk_steps 1/8, 1/8; stand 8/8 | 43/160 (stock 52), val_mse **0.0101** | **not held**: 4× data fits worse (the 2M FIFO buffer then holds only ~4 rounds) |
| walk14_s128r24 | walk14 | (stock distil) | walk_steps 1/8, 1/8; stand 8/8, goal_home 1/8 | 37/160 (teacher 79), falls 55 % (21 %), 143 W (94) | student loses 42 |
| walk14_s1_s128r24 | walk14_s1 | (stock distil) | walk_steps 1/8, 1/8 | 41/160 (teacher 67), 154 W | student |

Per-step peaks of the walk_steps episodes (16 per run, the referee's
seeds; `walk_steps_swings` wrapped, as on 10-26):

| run | first step: median, episodes < 3 cm | steady steps: min, median, < 3 cm | stop step: median, episodes < 3 cm |
|---|---|---|---|
| walk13_fit_s1 | 2.5 cm, 10/16 | 0.4, 6.0 cm, 4/75 | 4.6 cm, 3/16 |
| walk14 | 3.0 cm, 8/16 | 3.0, 6.1 cm, 1/74 | 4.6 cm, 1/15 |
| walk14_s1 | 3.2 cm, 7/16 | 0.5, 5.5 cm, 3/76 | 3.5 cm, 6/14 |
| walk14_s128r24 | 2.1 cm, 11/16 | 0.6, 5.5 cm, 2/74 | 4.6 cm, 3/16 |

- **The first step is the walk gate now, not the stop step.** On the fitted
  deploy plant the stop step clears 3 cm in all but 1 of walk14's 15
  episodes (the 10-26 pre-fit runs: 11 – 14 of 16 failed it). The first
  step from standing fails in half the episodes, on both feet (walk14: 4 L,
  4 R). Seed 1's misses are its stop step (6/14), mostly the right foot.
  The right-foot weakness of 10-30 is gone in walk14.
- **The legs students are not limited by the SGD or the data.** 4× epochs
  moved val_mse 0.0071 → 0.0066 and the score 52 → 46. 4× envs made it worse
  (0.0101, 43). Both stay at bc ≈ val, an underfit. In every round about 330
  of 512 student-driven slots fall. What is left is capacity, or the DAgger
  recipe feeding the student its own falls. Tonight's bracket on walk14:
  (256, 256) and the teacher's own (512, 256, 128). If the teacher-sized
  student also loses, the recipe is the problem and capacity is not.
  (256, 256) is ~115k parameters, ~460 KB fp32. Whether the ESP32 holds it
  is a firmware question for Tom and the robot session, and it only matters
  if it wins.

**Decision.** One teacher tonight: walk15 continues walk14 (a third 400M,
since the second added 9 seed-passes on that seed), with its stock student.
The legs night is mostly the capacity bracket. The first-step miss is the
next teacher variable once the student route is known.

#### Tonight (night 12)

Legs first (Tom 2026-10-24 / 10-30), then get-up. Expected: walk15
22:00 – 22:50, three legs distils and two get-up distils to ~23:20, then
four 600M get-up jobs at ~1.75 h each (the fourth starts ~04:35 and is capped
by the guards).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk15 | walk14 (walk14) | init walk13_fit_s1 → walk14 | still converging: ≥ 82/160, first-step lows ≤ 6/16, walk_steps_R ≥ 4/8 |
| 2 | robot_legs_walk14_s256r24 | walk14_s128r24 | distil `--hidden` 128,128 → 256,256 | capacity-bound: val_mse well below 0.005, ≥ 55/160 |
| 3 | robot_legs_walk14_s512r24 | walk14_s128r24 | distil `--hidden` → 512,256,128 (teacher size) | top of the bracket: ≥ 70/160, val_mse < 0.002; if not, the DAgger recipe is the limit |
| 4 | robot_legs_walk15_s128r24 | walk15 | (stock distil) | |
| 5 | robot_getup_w12k_s128r24 | w12k | (stock distil) | baseline student of the deployable teacher (~32/48 like w9k's) |
| 6 | robot_getup_w12k_s128r24n1024 | w12k_s128r24 | distil `--envs` 512 → 1024 | the get-up student overfits: val_mse < 0.008, sweep ≥ 40/48 |
| 7 | robot_getup_w13k | w12k (fresh) | policy (512,256,128) → (128,128) | deployable with no distil: sweep ≥ 44/48, 15 N ≥ 12/16 |
| 8 | robot_getup_w14k | w12k (fresh) | no `--init-from` | control for w13k: sweep ≥ 44/48 |
| 9 | robot_getup_w12k_s1 | w12k | seed 1 | replicate |
| 10 | robot_getup_w13k_s1 | w13k | seed 1 | replicate (capped) |

Preflight: PASS 10/10. Every training job's arg list also parses with
`train_mjx.py`'s own parser on main `6f86441`; preflight does not run that
parser, which matters for a new flag. Fallback: the w12k recipe (the
deploy-contract get-up).

**Tomorrow's evaluation:** legs: the capacity bracket (val_mse, watts,
overall, walk_steps) against walk14_s128r24; walk15 against walk14 with the
per-step table. Get-up: the sweep and the 15 / 30 N grades for w13k, w14k,
w12k_s1 and w13k_s1 against w12k. Read w13k's MJX curve against w14k's
(`eval/episode_recovered`): a small net that stalls early is a different
failure from one that is still climbing. The two w12k students' sweeps go
against the w9k student's 32/48.

### Night of 2026-11-01 (evaluated 2026-11-02)

All fourteen jobs ran (the robot session started the runner at 20:21 and put
its four actuator-DR legs jobs on `policy-stand-rca` first; the night ended
05:45; w13k_s1 was capped at 532M). The planner's jobs trained on main
`dc40c16`. Upstream since the last plan: #192 (floors/actuator fit) and the
on-ESP deploy, fall-guard, flail-guard and `stepresp` PRs. #192 only adds
opt-in flags (`--act-delay-extra`, `--act-lag-scale`, `--servo-kd`,
`--servo-joint-damping`, `--floor-contact`); no referee or env default
changed, and w12k_s1 on the new tree replicates w12k, so the comparisons hold.
A DNS failure at 23:01 skipped one fetch; that job trained on the same
`dc40c16`. No new moving flight-recorder session since the 11-01 stands, which
the RCA (PR #189) already fit.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each; as-built servos):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w12k_s128r24 (stock student) | 4/8 8/8 | 8/8 8/8 | 8/8 8/8 | **44/48** | 4/16 | 1/16 | teacher 48, 14, 5; w9k student 32, 6, 0 |
| w12k_s128r24n1024 (2× data) | 5/8 3/8 | 8/8 6/8 | 8/8 8/8 | 38/48 | 8/16 | 1/16 | stock 44, 4, 1 |
| w13k ((128, 128) PPO, fresh) | 4/8 8/8 | 1/8 8/8 | 2/8 8/8 | 31/48 | 7/16 | 0/16 | w12k 48, 14, 5 |
| w13k_s1 (seed 1, 532M) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 8/16 | 0/16 | w13k 31, 7, 0 |
| w14k ((512, 256, 128), fresh) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 12/16 | 4/16 | w12k 48, 14, 5 |
| w12k_s1 (seed 1) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **15/16** | 2/16 | w12k 48, 14, 5 |

- **The deploy-contract teacher distils much better than w9k did on friction
  (44/48 vs 32/48), but the student does not hold up to shoves (4/16 at 15 N).**
  Its only sweep misses are supine ph at 0.05, where it stands and then sinks
  (end height 0.34 m mean). It is still overfit (bc 0.0011, val 0.014). Twice
  the data per round made the fit worse (val 0.0185) and the sweep worse (38),
  so data volume is not the lever. What the student lacks is the teacher's
  recovery from a shove, i.e. states its buffer never holds.
- **A (128, 128) net trained directly with PPO can do the whole get-up.**
  w13k_s1 sweeps 48/48 with no distillation. Its MJX `episode_recovered`
  curve plateaus by ~160M steps, as fast as the big net's (w14k), so the
  small net is not a slow learner. The other seed (w13k) has the old
  post-stand sink from supine ph: it stands at 21.7 s and ends at 0.29 m in
  7/8 at 0.35. Both small-net seeds lose shoves: 7 and 8/16 at 15 N against
  12 – 15/16 for the big nets.
- **The big net needs no t3k init.** w14k from scratch sweeps 48/48 with
  12/16 at 15 N. w12k_s1 replicates w12k (48/48, 15/16), so the deploy-
  contract recipe holds over two seeds.

**Decision.** For arc step 4 there are two (128, 128) candidates:
w13k_s1 (PPO: 48/48, 8/16 at 15 N) and the w12k student (44/48, 4/16). The
gap to the teacher is now the shove, not the sweep. Tonight tests both
routes. Distillation: noise on the executed actions (`--act-noise 0.3`,
DART-style, which widens the states the buffer sees) and the teacher-sized
student, to tell recipe from capacity. PPO: a (256, 256) net (is the push
loss capacity?), seed 2 of the (128, 128) recipe, and the (128, 128) net at
residual 0.075, to see whether less authority removes w13k's sink. The body
question (every get-up trains on the arms body; the as-built robot has
none) is still Tom's.

#### One-leg stands and short walks

Two lines this morning. (a) The planner's no-dead-time line (walk15, the
capacity bracket). The robot session's 11-01 RCA has made it obsolete for
deploy: on the measured actuator every policy trained without dead time
has ≤ 1.25× loop-gain margin. It still answers the student question.
(b) The robot session's actuator-DR line (`policy-stand-rca`, PR #189):
walk16_act / walk16_actwide, warm-started from walk14_s1_actar.

| run | parent | the one variable | key rows (as-trained CPU referee) | overall | verdict |
|---|---|---|---|---|---|
| robot_legs_walk15 | walk14 (init walk14) | a third 400M | walk_steps L/R 3/8, 1/8 (min peak 2.4 / 1.7 cm); stand 8/8; turn 1/8 | 67/160 (walk14 79), falls 23 % | **not held** (≥ 82): walk14 had converged |
| walk14_s256r24 | walk14_s128r24 | student `--hidden` 256,256 | walk_steps 4/8, 3/8; stand 8/8; val 0.0053 | **62/160** (s128 37) | **held** (≥ 55): capacity is most of the gap |
| walk14_s512r24 | walk14_s128r24 | student = teacher size | walk_steps 2/8, 5/8; stand 8/8; val 0.0041 | **69/160** (teacher 79) | at the edge (≥ 70): capacity closes 32 of 42; the DAgger recipe the last ~10 |
| walk15_s128r24 | walk15 | (stock distil) | walk_steps 2/8, 1/8; stand 7/8 | 28/160 | |
| walk16_act | walk14_s1_actar | +400M actuator DR + action rate 0.3 (seed 0) | stand 8/8 (27 W), walk_steps 0/8 0/8 (min peak 1.0 / 0.5 cm), turn 0/8 | 17/160, falls 78 % | stands, does not walk |
| walk16_actwide | walk14_s1_actar | wider kp/kd/gyro DR (seed 1) | stand 8/8, walk_steps 0/8 0/8 | 20/160, falls 78 % | as walk16_act |
| walk16_act_s128r24 | walk16_act | (stock distil) | stand 6/8 (drift 13 cm, 118 W), walk_steps 0/8 | 12/160 | see the margin table |
| walk16_actwide_s128r24 | walk16_actwide | (stock distil) | stand 7/8 | 17/160 | |

Margin on the measured actuator (`experiments/policy/rca/candidates.py
… fit 8`, `ACT=meas` + `ACT=hard`, branch `policy-stand-rca`; falls of 8):

| policy | measured | ×1.25 | ×1.5 | ×2 | push 0.4 / 0.6 m/s | dead 100 ms, τ 90 | sweep-only fit | gyro ×2.5 |
|---|---|---|---|---|---|---|---|---|
| walk14_s1_s128r24 (deployed 11-01, RCA) | 1/8 | 8/8 | 8/8 | 8/8 | 5/8 / — | — | 8/8 | — |
| walk14_s1_actar_s128r24 (RCA, day) | 0/8 | 0/8 | 0/8 | 8/8 | 0/8 / 8/8 | — | 5/8 | — |
| **walk16_act_s128r24** | 0/8 | 0/8 | **0/8** | 8/8 | 0/8 / 8/8 | 0/8 | **0/8** | 0/8 |
| walk16_actwide_s128r24 | 0/8 | 0/8 | 4/8 | 8/8 | 0/8 / 8/8 | 0/8 | 6/8 | 0/8 |
| walk16_act (teacher) | 0/8 | 0/8 | 0/8 | 8/8 | 0/8 / 1/8 | 0/8 | 2/8 | 0/8 |
| walk16_actwide (teacher) | 0/8 | 0/8 | 5/8 | 8/8 | 0/8 / 5/8 | 0/8 | 4/8 | 0/8 |

- **walk16_act_s128r24 is the best hardware-stand candidate so far.** It
  meets the RCA bar (0/8 at ×1.5). It also passes the pessimistic
  sweep-only actuator fit (0/8, where the actar student had 5/8), 100 ms
  dead time, and gyro ×2.5. It fails ×2, like every other policy. On the
  old no-dead-time referee its stand drifts and burns 118 W, but that
  plant is no longer the robot (see below). The wide DR (actwide) cost
  margin (×1.5 4/8, sweep-only 6/8): dropped.
- **On the actuator-DR plant the walk does not come back.** MJX
  `eval/episode_fell` over 1.1B steps of actuator DR: walk14_s1_act
  1.00 → 0.91, actar → 0.91, walk16_act 0.90 → 0.89, walk16_actwide
  0.91 → 0.88 (walk14 without dead time: 0.25). Every walk_steps peak is
  ≤ 1.2 cm. More steps on the same recipe is not the lever.
  The measured actuator is ~68 ms dead time + ~55 ms rise. At the
  default gait clock (1.25 – 1.75 Hz, scaled up to ×1.4 with commanded
  speed) that leaves roughly 170 – 330 ms per swing. Tonight's two
  teachers each test one cause: the cadence (commanded speed capped at
  0.35 m/s, so the clock runs ≤ 1.0×) and the dead-time tails (DR
  2 – 5 → 3 – 4 ticks, around the measured 68 ms).
- **Student capacity, answered on walk14:** (256, 256) recovers 25 of the
  42 seed-passes the (128, 128) student lost, and the teacher-sized
  student 32. Capacity is the main gap and the DAgger recipe is the rest.
  A (256, 256) student is ~115k parameters (~460 KB fp32), and whether the
  ESP32 firmware can hold it is now a real question for Tom. Tonight
  distils walk17_slow at both sizes.
- **The CPU referee grades the legs on a plant without dead time.** Its
  scorecard and the measured-actuator margins disagree for the
  actuator-DR policies (walk16_act_s128r24: stand_10s 6/8 on the referee,
  0/8 falls on the measured actuator). Until #189 is rebased onto #192 and
  the referee gets the measured actuator, both are reported. For deploy,
  the margin table is the ranking. The SIL card is still missing for every
  legs run (the export refuses: `obs_spec.h` kObsDim 147 vs the run's 165;
  firmware/generated, not touched here).

**Decision.** The stand question is answered in sim:
walk16_act_s128r24 goes to Tom and the robot session as the hardware-stand
candidate (the RCA doc's route b, exported the same way). The walk is now
the open problem on the actuator-DR plant. The no-dead-time line stops:
no more walk14/15 continuations.

**Pre-flight finding:** pre-flight builds an *existing* teacher's distil
env from the night clone on main, whatever the job's `BRANCH=`. A distil of
an existing actuator-DR teacher (`--act-delay-types` is branch-only) fails
pre-flight, and at 22:00 that would block the whole night. It passed last
night only because those teachers did not exist yet. Until #189 merges,
distil only teachers that train on the same night.

#### Tonight (night 13)

Legs first (Tom 2026-10-24 / 10-30), then get-up. Expected: two 400M legs
teachers at ~50 min with three distils (~22:00 – 23:50), two get-up distils
(~8 min), then three 600M get-up jobs at ~1.7 h (the third may be capped).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk17_slow | walk16_act (walk16_act) | `--cmd-v-range` 0.05,0.7 → 0.05,0.35 | the cadence breaks the walk: MJX fell < 0.7 (0.89), walk_steps median peak ≥ 2 cm, student still 0/8 at ×1.5 |
| 2 | robot_legs_walk17_slow_s128r24 | walk17_slow | (stock distil) | |
| 3 | robot_legs_walk17_slow_s256r24 | walk17_slow_s128r24 | `--hidden` 256,256 | the capacity gain transfers to the actuator-DR line, no margin lost |
| 4 | robot_legs_walk17_narrow | walk16_act (walk16_act) | `--act-delay-types` sts3215 2:5 → 3:4 | the dead-time tails break the walk: fell < 0.7, peaks ≥ 2 cm (risk: the 100 ms probe) |
| 5 | robot_legs_walk17_narrow_s128r24 | walk17_narrow | (stock distil) | |
| 6 | robot_getup_w12k_s128r24an3 | w12k_s128r24 | distil `--act-noise` 0 → 0.3 | wider visited states: val < 0.01, 15 N ≥ 8/16, sweep ≥ 44/48 |
| 7 | robot_getup_w12k_s512r24 | w12k_s128r24 | distil `--hidden` 512,256,128 | recipe, not capacity: 15 N stays ≤ 8/16 |
| 8 | robot_getup_w15k | w13k (fresh) | `--policy-hidden` 256,256 | push loss is capacity: 48/48, 15 N ≥ 12/16 |
| 9 | robot_getup_w13k_s2 | w13k | seed 2 | replicate (31 vs 48) |
| 10 | robot_getup_w16k | w13k (fresh) | `--rise-ref-residual` 0.15 → 0.075 | no post-stand sink: sweep ≥ 46/48 (may be capped) |

Pre-flight: PASS 10/10. The five training jobs also parse with
`train_mjx.py`'s own parser on their branches (`policy-stand-rca` c6f8250,
main dc40c16). Fallback: the w13k recipe (the (128, 128) get-up, whose seed
spread is the open question).

**Tomorrow's evaluation:** legs: MJX `eval/episode_fell` and walk_steps
peaks for walk17_slow and walk17_narrow against walk16_act. Grade all three
students with `ACT=meas` + `ACT=hard` candidates.py next to their
scorecards. Compare walk17_slow_s256r24 with its s128 sibling. Get-up: the
sweep plus 15 / 30 N for w15k, w13k_s2 and w16k against w13k / w13k_s1, and
for the two students against w12k_s128r24 (44, 4, 1), with val_mse.

### Night of 2026-11-02 (evaluated 2026-11-03)

All fourteen jobs ran (the night ended 05:41; the robot session put four
shaper and two fast-servo legs jobs ahead of the planner's). Before the night,
the robot session changed walk17_slow / walk17_narrow to the fast-servo
dead time (`sts3215=0:2` and `1:1`), so neither tests its planned variable
against walk16_act. The shaper-train branch had been merged (#203) and
deleted, so `fetch shaper-train` failed and the first job trained on main
`d5ca9b0`, which already holds #203. The get-up trained on main `65c8dd5` –
`1220662`. Upstream since the last plan: #203 (the referee now inherits a
run's firmware shaper from its config.json; no get-up run has one), #205 (SIL
cal), #208, #211 and #212 (firmware tools), power-board docs. No get-up plant,
servo or env default changed, so the get-up comparisons hold. No new moving
flight-recorder session since the 11-02 stands.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35 (CPU referee, 8 seeds each; as-built servos):

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| **w12k_s128r24an3** (distil, action noise 0.3) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 5/16 | stock student 44, 4, 1; teacher 48, 14, 5 |
| w12k_s512r24 (teacher-sized student) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 4/16 | stock 44, 4, 1 |
| w15k ((256, 256) PPO) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 7/16 | 1/16 | w13k_s1 48, 8, 0 |
| w13k_s2 ((128, 128) PPO, seed 2) | 6/8 8/8 | 8/8 8/8 | 8/8 8/8 | 46/48 | 8/16 | 2/16 | w13k 31, 7, 0; w13k_s1 48, 8, 0 |
| w16k ((128, 128) PPO, residual 0.075) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 9/16 | 1/16 | w13k 31, 7, 0 |

Distil losses (round 24): stock bc 0.0011 / val 0.014; an3 0.0072 / 0.022;
s512 0.00036 / 0.0077.

- **Arc step 4 has its deployable get-up: `robot_getup_w12k_s128r24an3`.**
  It is a (128, 128) net, the firmware's size, with distillation noise on the
  executed actions (DART). It sweeps 48/48 and holds 16/16 shoves at 15 N,
  better than its teacher (14/16). The hypothesis held: the stock student's
  buffer never held the post-shove states. Its higher val_mse is measured on
  that noisier buffer, so it does not compare with the stock student's.
- **Capacity alone also closes the gap, so the s512 hypothesis is not
  held.** The teacher-sized student also gets 16/16 at 15 N (val 0.0077). Both
  levers work; at the deployable size, noise is the one to use.
- **PPO small nets plateau on shoves.** Four (128, 128) or (256, 256) PPO
  runs sweep 31 – 48/48 but hold only 7 – 9/16 at 15 N. The (256, 256) net
  (w15k) is no better, so the shove loss is not capacity. Residual 0.075
  (w16k) removes the post-stand sink on its one seed (48/48) but not the
  shove loss. The direct-PPO route for the deployable net stops here: distil
  with noise.

**Found while planning (deploy fidelity, not fixed):** for get-up episodes,
`env_mjx.reset` seeds the act-lag cascade, the dead-time history and the
firmware shaper at the STAND pose (`_sdefault`), not at the lying start.
`walker_env` seeds them the same way, and the cached-first-state redraw always
uses `_sdefault`. Every get-up episode therefore starts with a short yank of
the lagged targets toward standing. The robot will not do this, because the
firmware seeds its shaper at measured q. Training and the referee agree, so
the scores are self-consistent, but before the get-up goes on the robot the
chain should be seeded at the start pose (a flag in both envs, then a
retrain). It matters more with the 2 Hz act-lag stages than with the
10 Hz shaper.

#### One-leg stands and short walks

Grades on the fast measured actuator through the firmware chain
(`experiments/policy/shaper-train/grade.py`, shaper 10, no obsfreeze, flail
guard per tick; falls / guard trips of 8). The as-trained CPU referee card
(`scorecard.md`) is next to it:

| policy | referee overall | stand_10s | walk_steps L / R (min peak) | x1.5 | x2 | push 0.5 | walk 0.3 → stop | stride1 0.6 | squeeze (deg) |
|---|---|---|---|---|---|---|---|---|---|
| walk18_shp10sq_s128r24 (candidate, 11-02) | — | 8/8 | 0/8, 0/8 (1.1 – 1.3 cm) | **0/0** | 8/8 | 6/6 | 0/0 (rate 0.78) | 0/0 (0.73) | **2.04** |
| walk19_shp10sq (+600M, seed 11) | 61/160 | 8/8 | 0/8, 0/8 (1.0 / 1.4) | 8/8 | 8/8 | 1/2 | 0/0 (0.86) | 0/0 (0.83) | 5.63 |
| walk19_shp10sq_s128r24 | 58/160 | 8/8 | 0/8, 1/8 (1.4 / 1.8) | **8/8** | 8/8 | 0/0 | 0/0 (0.86) | 0/0 (0.99) | 5.07 |
| walk19_shp8_12sq (pole U(8, 12), seed 12) | 59/160 | 8/8 | 0/8, 0/8 (1.5 / 1.5) | 8/8 | 8/8 | 0/0 | 0/0 (0.86) | 0/0 (1.01) | 3.21 |
| walk19_shp8_12sq_s128r24 | **64/160** | 8/8 | 0/8, 0/8 (1.3 / 1.3) | **8/8** | 8/8 | 0/0 | 0/0 (0.94) | 0/0 (0.99) | 3.29 |
| walk17_slow_s128r24 (no shaper) | 9/160 | 6/8 | 0/8, 0/8 | 2/2 | 8/8 | 1/3 | 0/7 [pose] | 0/8 [rate, pose] | 4.01 |
| walk17_slow_s256r24 | 11/160 | 5/8 | 0/8, 0/8 | 8/8 | 8/8 | 0/1 | 0/0 | 0/8 [pose] | 3.58 |
| walk17_narrow_s128r24 (no shaper) | 11/160 | 7/8 | 0/8, 0/8 | 8/8 | 8/8 | 1/0 | 0/8 [rate] | 0/8 [pose] | 3.51 |

Every policy passes the slow (factory) servo probe 0/0. Pole sweep of the two
walk19 students (`poles` set): x1.5 is 8/8 at every pole from 8 to 20 Hz. At
20 Hz, shp8_12 trips the guard on chatter in 5/8 walks; walk18 did in 4/8.

- **The +600M continuations lost the hardware margin.** Both walk19
  students fall 8/8 at loop gain x1.5 (walk18_shp10sq_s128r24: 0/8), at every
  pole, and the teachers do too. The stand squeeze rose from 2.0 to 5.1°
  (shp10sq) and 3.3° (shp8_12sq). The pole band did not widen the tolerated
  shaper range. Neither hypothesis held: x2 did not improve, and x1.5 at
  12 / 15 Hz did not hold. Two seeds and two shaper setups lost x1.5 the
  same way, while the ~300M steps walk17_shp10 → walk18_shp10sq kept it.
  The suspect is the budget. Tonight replicates walk18's 300M on a new
  seed to separate the budget from walk18's luck.
- **The distillation gap is closed on the shaper line.** Referee overall,
  teacher → student: 61 → 58 and 59 → 64/160 (on 10-31 the students lost
  50 – 55 %). The pre-shaper walk17 line still loses most of it (26 → 9,
  16 → 11). On the walk17 line the (256, 256) student does not help.
- **The walk itself is unchanged.** Every walk_steps row is 0/8 or 1/8,
  with min peaks of 1.0 – 1.8 cm and medians around 3 cm (about 55 % of
  swings ≥ 3 cm). The low steps are the gate. Tonight's sibling raises the
  per-step clearance penalty (`--w-low-swing` 10 → 30).
- **walk17_slow / walk17_narrow (fast servos, no shaper) are superseded.**
  MJX training falls fell from 0.89 (walk16_act, slow servos) to 0.73 and 0.66,
  so the fast servos help the walk train. But all three students trip the
  flail guard on stride1 and walk → stop. That is the failure the shaper line
  fixes. The no-shaper legs line stops.
- `candidates.py` (`ACT=meas` / `ACT=hard`) grades the pre-11-02 slow actuator
  (68 ms dead time). Since the robot runs fast servos, `grade.py` is the
  deploy grade, and its slow-servo column covers the old actuator (0/0 for
  all). The legs SIL card is still missing (`obs_spec.h` kObsDim 147 vs the
  run's 165).

**Decision.** walk18_shp10sq_s128r24 stays the hardware candidate (the
robot session's §3 recipe in `experiments/policy/shaper-train/2026-11-02-shaper-train.md`).
No walk19 policy is a candidate. Legs fine-tunes go back to 300M from
walk18_shp10sq: one budget replicate, and one sibling for the clearance gate.

#### Tonight (night 14)

Legs first, then get-up. Expected: two 300M legs teachers at ~35 min with
their distils (~22:00 – 23:20), two get-up distils (~8 min), then three 600M
get-up jobs at ~1.7 h (the last may be capped).

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk20_b300 | walk19_shp10sq (walk18_shp10sq) | budget 600M → 300M (seed 13) | the margin loss is the budget: student x1.5 0/8, squeeze ≤ 2.5°, no stride1 / walk → stop trip |
| 2 | robot_legs_walk20_b300_s128r24 | walk20_b300 | (stock distil) | |
| 3 | robot_legs_walk20_lowsw30 | walk20_b300 (walk18_shp10sq) | `--w-low-swing` 10 → 30 | walk_steps min peak ≥ 2 cm, swings ≥ 3 cm ≥ 70 %, x1.5 kept |
| 4 | robot_legs_walk20_lowsw30_s128r24 | walk20_lowsw30 | (stock distil) | |
| 5 | robot_getup_w12k_s1_s128r24an3 | w12k_s128r24an3 | teacher w12k → w12k_s1 | the noise recipe, not one teacher: ≥ 46/48, 15 N ≥ 14/16 |
| 6 | robot_getup_w12k_s128r24an6 | w12k_s128r24an3 | `--act-noise` 0.3 → 0.6 | 30 N above 5/16, sweep ≥ 46/48 |
| 7 | robot_getup_w17k | w12k (t3k) | the measured fast-servo actuator (as the legs since 11-02) | costs nothing: ≥ 46/48, 15 N ≥ 12/16 |
| 8 | robot_getup_w17k_s128r24an3 | w12k_s128r24an3 | teacher → w17k | the deployable student carries over |
| 9 | robot_getup_w18k | w17k (t3k) | + `--fw-shaper 10 --fw-goal-speed` | trains through the deploy shaper: ≥ 46/48, 15 N ≥ 12/16 |
| 10 | robot_getup_w17k_s1 | w17k | seed 1 | replicate (may be capped) |

Pre-flight: PASS 10/10. The five training jobs also parse with
`train_mjx.py`'s own parser on main `1220662`. Fallback: the w12k recipe,
the deploy-contract teacher; each new seed distils to a deployable get-up.

**Tomorrow's evaluation:** legs: `grade.py` (runs set) on both walk20
students against walk18_shp10sq_s128r24, and the walk_steps peaks on their
referee cards. Get-up: the sweep plus 15 / 30 N for all five against
w12k / w12k_s128r24an3. For w18k, the referee inherits the shaper; confirm
that in its card's claim line.

### Night of 2026-11-03 (evaluated 2026-11-04)

All ten jobs ran (the night ended 04:50) on main `71d1e4f`; the last two get-up
jobs ran on `e9bded2` (docs only). Upstream since the last plan: docs, plus #210
(the measured-stops plants), which landed before the night. No get-up plant,
servo or env default changed, so the comparisons hold. One job error: the
w17k_s1 job had no `--xml`, so it trained on the old plant
`bimo_biped_v6ar.xml` while w17k trained on `v6ar_meas`. In within-stops mode the
policy range is the same, so it still counts as a replicate across the plant change.

**Hardware, 2026-11-03 (the robot session, `hw_sessions/2026-11-03/NOTES.md`):**

- walk18_shp10sq_s128r24 stood 60 s on fast servos (jitter 0.018°, 0.88 W): PASS.
- Its first `stride1` (vx 0.6) kicked the right leg fast and high and yawed
  the body −19° in 180 ms. The flail guard tripped on body rate at +0.35 s, and
  the robot pivoted on its left foot and fell.
- Hanging 60° max-speed moves: hip pitch and knee keep 240 – 275 °/s under the
  leg's load. The sim's no-load speed is 231 °/s.

The robot session's diagnosis is PR #214
(`experiments/policy/friction-dr/2026-11-03-friction-dr.md`):

- The friction DR never took the pad–floor μ below 1.0, because MuJoCo
  max-combines the friction of a geom pair.
- The plant's leg swings ~1.6× slower than the robot's.
- An actuator refit (hip pitch / knee at 12 V: stall ×2.0, no-load ×1.325 /
  ×1.281) reproduces the swing. On the refit plant, walk18 trips the guard
  4/8 at μ 1 and 8/8 at μ 0.4.

**I merged #214 this morning (`865793f`) so that the refit-actuator jobs tonight
can be refereed on main.** Every new flag is off by default.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, and the
push-stress grade at 0.35. CPU referee, 8 seeds each; as-built servos.

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N @ 2 % | 30 N @ 2 % | compare |
|---|---|---|---|---|---|---|---|
| w12k_s1_s128r24an3 (DART student, 2nd teacher) | 8/8 8/8 | 8/8 8/8 | 7/8 8/8 | 47/48 | **16/16** | 5/16 | w12k_s128r24an3 48, 16, 5 |
| w12k_s128r24an6 (noise 0.6) | 8/8 8/8 | 8/8 8/8 | 7/8 8/8 | 47/48 | **16/16** | 6/16 | an3 48, 16, 5 |
| **w17k** (measured fast actuator) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 3/16 | w12k 48, 14, 5 |
| w17k_s128r24an3 | 8/8 8/8 | 8/8 8/8 | 7/8 8/8 | 47/48 | **16/16** | 2/16 | w12k_s128r24an3 48, 16, 5 |
| **w18k** (+ firmware shaper 10 Hz) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 5/16 | w17k 48, 16, 3 |
| w17k_s1 (seed 1, old plant) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | 15/16 | 3/16 | w17k 48, 16, 3 |

Every run stands in 21.7 – 23.5 s from supine ph and 33.1 – 34.7 s from prone.

- **The get-up runs on the full deploy chain at no cost.** w17k (the measured
  fast servo) and w18k (plus the firmware shaper and goal speed) both sweep
  48/48 and take 16/16 shoves at 15 N, at least as well as w12k on the
  stand-in act-lag. Both hypotheses held. w18k's referee ran through the
  shaper: eval_precision passes `fw_shaper_hz` from config.json to the env
  (the claim line does not print it).
- **The DART student is a recipe, not one lucky teacher.** On a second
  teacher (w12k_s1) it gives 47/48 and 16/16. On the fast-actuator teacher
  (w17k) it gives 47/48 and 16/16. Both held.
- **More distil noise does not reach 30 N.** Noise 0.6 gives 6/16 against 5/16
  at 0.3, which is within seed noise. 30 N stays at 2 – 6/16 on every policy
  of the line, so it is closed (as on night 9).
- **What is left before the get-up can go on the robot** is not a sim score:
  1. the start-pose seeding (fixed today, below);
  2. a body with arms. The as-built robot has none; Tom's 09-17 decision is
     the two-arm elbow design.

**Start-pose seeding fixed (main `6fac9db`, `--seed-chain-at-q`).** With the
flag, the dead-time ring, act-lag cascade, firmware shaper and slew limiter's
last target start at the episode's start q:

- env_mjx at reset and at the cached-first-state reseed;
- walker_env on the first step.

It is off by default, so past runs are unchanged. The referee inherits it from
config.json, and so does the distil env. `tests/test_seed_chain_at_q.py` pins it.

#### One-leg stands and short walks

Students graded with grade.py on the friction-dr code (now main): the fast
measured actuator through the firmware chain (shaper 10, no obsfreeze, flail
guard per tick), plus #214's low-μ and refit-actuator stride1 probes. Falls /
guard trips of 8. Referee overall is teacher → student.

| policy | referee | walk_steps L / R (min peak) | swings ≥ 3 cm | x1.5 | push 0.5 | squeeze / hold power | stride1 μ 1 / μ 0.4 / μ 0.3 | stride1 refit / refit μ 0.4 |
|---|---|---|---|---|---|---|---|---|
| walk18_shp10sq_s128r24 (candidate) | — | 0/8, 0/8 (1.1 – 1.3) | — | **0/0** | 6/6 | **2.04° / 0.21 W** | 0/0 / 0/0 / 0/0 | 0/4 / 0/8 |
| walk20_fric_s128r24 (contact-μ DR, by day 11-03) | — | 0/8, 0/8 (1.2) | — | 0/0 | 1/8 | 2.77° / 1.35 W | 0/0 / 0/0 / 0/0 | 0/8 / 0/8 |
| walk20_b300_s128r24 (300M, seed 13) | 59 → 53 | 0/8, 0/8 (1.0 / 1.2) | 63 % | **1/1** | 8/8 | 3.36° / 1.53 W | 0/0 / 0/0 / 0/0 | 0/8 / 0/8 |
| walk20_lowsw30_s128r24 (`--w-low-swing` 30) | 55 → 57 | 1/8, 0/8 (1.4 / 1.3) | 71 % | 0/0 | **0/0** | 3.69° / 1.82 W | 0/0 / 0/0 / 0/0 | 0/8 / 0/8 |

- **The budget hypothesis is not held.** The 300M continuation on a new seed
  (walk20_b300) also loses the margin: x1.5 1/8 and squeeze 3.4°. Every
  continuation of walk18 has raised the stand squeeze, whatever the budget:
  walk19 5.1 / 3.3°, walk20_fric 2.8°, walk20_b300 3.4°, walk20_lowsw30 3.7°,
  against walk18's 2.0°. The hold power rose with it, from 0.2 to 1.4 – 1.8 W.
  walk18's quiet stand looks like that seed's luck. The squeeze is
  penalised (`--w-squeeze 1.0`), and the penalty does not hold it there.
- **A heavier low-swing penalty moves the bulk of the steps, not the lowest
  one.** walk20_lowsw30 raises the share of swings ≥ 3 cm from 63 to 71 %
  (median 3.7 → 4.0 cm) and survives push 0.5 (0/0; walk18 6/6). Its lowest
  steps stay at 1.3 – 1.4 cm, so the min peak ≥ 2 cm hypothesis is not held.
  The walk_steps gate is still 0 – 1/8.
- **Distillation gap:** −10 % (b300) and +4 % (lowsw30) on the referee
  overall. The gap stays closed on the shaper line.
- **The refit gate fails for every policy we have.** All students trip on
  rate 8/8 at stride1 on the refit actuator, at μ 1 and at μ 0.4. None of
  them trained on that leg. The low-μ probes on the stock plant pass for all
  of them, so that gate does not separate them.

**Decision.** walk18_shp10sq_s128r24 stays the stand candidate: it stood on
the robot. No policy is a stride1 candidate, because walk18 fell on the
robot's first step. Following the deploy-first rule ("plan sim/plant updates
from the flight recorder before more reward tuning"), the legs line trains on
the refit actuator tonight, then adds Tom's smoothness terms (11-03: "avoid
big jerks, especially when a joint comes to a stop").

#### Tonight (night 15)

Legs first, then get-up. Expected timings:

- two 300M legs teachers at ~40 min each, with their distils: 22:00 – 23:30;
- the w18k distil: ~5 min;
- three 600M get-up teachers at ~1.75 h each, with one distil; the last
  starts ~03:15 and may be capped.

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk21_refit | walk20_fric (walk18_shp10sq), seed 14 | + `--servo-envelope hip_pitch=2.0:1.325,knee=2.0:1.281 --supply-voltage 12` (the refit actuator) | the student passes stride1 refit / refit μ 0.4 (no rate trip; parent 8/8, 8/8), x1.5 0/8 kept |
| 2 | robot_legs_walk21_refit_s128r24 | walk21_refit | (stock distil) | |
| 3 | robot_legs_walk21_fricact | walk21_refit, seed 14 | + `--w-qacc 4e-5 --w-action-acc 1.0` | lower walk_qdd_stop; no stop-phase trip on the refit probes; x1.5 kept |
| 4 | robot_legs_walk21_fricact_s128r24 | walk21_fricact | (stock distil) | |
| 5 | robot_getup_w18k_s128r24an3 | w17k_s128r24an3 | teacher w17k → w18k | the DART student carries through the shaper: ≥ 46/48, 15 N ≥ 14/16 |
| 6 | robot_getup_w19k | w18k (t3k) | + `--seed-chain-at-q` | no reliance on the start yank: ≥ 46/48, 15 N ≥ 14/16 |
| 7 | robot_getup_w19k_s128r24an3 | w18k_s128r24an3 | teacher w18k → w19k | the deploy-faithful student: ≥ 46/48, 15 N ≥ 14/16 |
| 8 | robot_getup_w19k_s1 | w19k | seed 1 | replicate |
| 9 | robot_getup_w20k | w19k (t3k) | + `--w-qacc 4e-5 --w-action-acc 1.0` | lower wobble at no cost (may be capped) |

Pre-flight: PASS 9/9. The five training jobs also parse with `train_mjx.py`'s
own parser on main `6fac9db`, and `build_env_kw` carries the new flags. The
fallback is now the w18k recipe, the full deploy chain.

**Tomorrow's evaluation:**

- Legs: `grade.py` (on main now) for both walk21 students against
  walk20_fric_s128r24 and walk18_shp10sq_s128r24. The refit probes are the
  headline. Also check walk_qdd_stop and the walk_steps peaks on the referee
  cards. The CPU referee grades the refit actuator: `servo_types` with
  explicit envelope pairs passes through config.json.
- Get-up: the sweep plus 15 / 30 N for all five. For w19k, confirm
  `seed_chain_at_q` in its config.json. Grade w20k's wobble against w19k's.

### Night of 2026-11-04 (evaluated 2026-11-05)

All eleven jobs ran (the night ended 05:23) on main `a8008e4` / `fd1c221`. The
queue was not the one planned on 11-04: the robot session merged the motion
limiter (#215) and replaced `walk21_refit` with two limiter jobs,
`walk21_fricact_lim2000` and `_lim3000` (`hw_sessions/2026-11-04/NOTES.md`).
`walk21_refit` stays held: `walk21_fricact` (refit actuator + smoothness)
trained from the same init and seed, so the refit-only control is not worth a
slot.

Upstream since the last plan:

- #215 (the motion limiter, off by default).
- #216: the arm head insert (+1.8 g on each `_arm` body). It also repairs the
  `_meas` plants, which referenced a deleted hip-yoke STL.
- CAD and cleanup.

The get-up jobs trained on `fd1c221`, which includes the insert, and w18k on
the plant before it. At 1.8 g per arm the comparisons hold.

**Two tool failures this morning, both fixed:**

1. **The collection refereed on stale code.** `~/code/robot` could not
   fast-forward (collect log: "could not fast-forward … refereeing on
   `9ce947a`"). Four generated STLs were dirty there and also changed upstream:
   a mira rebuild before #216 made rebuilds byte-stable. On `9ce947a` the
   referee has no `fw_limit`, so **both limiter teachers and their students
   were refereed without the limiter they trained with**.
   - I backed up the four STLs (`/tmp/plan1105/robot_dirty_bak/`), restored
     them, and fast-forwarded to `fd1c221`. The checkout's other uncommitted
     edits are untouched.
   - The four limiter runs are re-refereed on main below. Their morning cards
     (35, 29, 47 and 44/160) are void.
   - The get-up cards and the friction / push grades ran on `9ce947a`. That
     code has no limiter and no insert, so it is the same get-up referee.
2. **`grade.py` on main was broken.** The limiter merge (`bbfc6f9`) moved the
   tick loop into `_episode_loop` but left its yaw / momentum / smoothness
   series behind, so every episode raised `NameError` and graded as an error.
   Fixed on main (`5c543c3`). To grade each student at its own trained
   limiter, I used a wrapper that reads `fw_limit` from config.json
   (`/tmp/plan1105/legs/grade_wrap.py`).

**New referee option (main): `eval_precision.py --contact-mu M [--contact-torsion T]`.**

- It grades on an effective sole friction. The floor, every sole pad and the
  soles are set together, because MuJoCo takes the max of a contact pair and
  the pads are 1.0.
- It writes `scorecard_mu<M>[t<T>]`, and is pinned by
  `tests/test_referee_contact_mu.py`.
- It exists for the 11-04 clamp test: sliding μ ≈ 0.45, yaw grip ≈ 0.1 –
  0.15 N·m, against the plant's ~0.66 N·m.

#### Get-up

Supine ph / prone fall at joint frictionloss 0.05 · 0.35 · 0.5, the
push-stress grade at 0.35, and (new) the sole friction at 0.45 and 0.3
(torsion 0.002). CPU referee, 8 seeds each.

| run | 0.05 | 0.35 | 0.5 | sweep | 15 N | 30 N | sole μ 0.45 / 0.3 | compare |
|---|---|---|---|---|---|---|---|---|
| w18k_s128r24an3 (DART student through the shaper) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 5/16 | 16/16, 16/16 | w17k_s128r24an3 47, 16, 2 |
| **w19k** (+ `--seed-chain-at-q`) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 6/16 | 16/16, 16/16 | w18k 48, 16, 5 |
| **w19k_s128r24an3** (its DART student) | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 3/16 | 16/16, 16/16 | w18k_s128r24an3 48, 16, 5 |
| w19k_s1 | 8/8 8/8 | 8/8 8/8 | 8/8 8/8 | **48/48** | **16/16** | 4/16 | — | w19k |
| w20k (+ smoothness terms) | 8/8 8/8 | 8/8 8/8 | 7/8 8/8 | 47/48 | 15/16 | 5/16 | — | w19k 48, 16, 6 |

Every run stands in 22.0 – 23.0 s from supine ph and in 33.4 – 34.4 s from
prone. Verdicts against the hypotheses:

- **The faithful start costs nothing** (held, two seeds). The get-up never
  leaned on the yank toward the stand pose that the old seeding gave every
  episode. Its DART student is again at the teacher's level.
  **w19k_s128r24an3 is the get-up candidate for the robot**: the full deploy
  chain, (128, 128), 48/48, 15 N 16/16.
- **The get-up does not care about the measured sole friction.** At μ 0.45
  and 0.3, all three policies graded take 16/16, with times within 0.5 s.
  So no friction-DR get-up job is needed.
- **The smoothness terms do not help the get-up** (not held). Wobble fell
  0.40 → 0.37 (supine ph) and 0.43 → 0.41 (prone), which is within card
  noise. Power rose 13 – 19 %, the sweep and the 15 N grade each lost a trial,
  and PPO's KL ran 10× higher. Dropped from the get-up recipe.
- 30 N stays at 3 – 6/16 (closed since night 9).

#### One-leg stands and short walks

Re-refereed on main (`fd1c221`), so the limiter runs are graded with their
limiter: teacher → student overall, plus the student's key rows. `grade.py`
(fixed) runs the fast actuator through the firmware chain at shape 10, each
student at its trained limiter, 8 seeds. Cells are falls / flail-guard trips.

| policy | referee T → S | walk_steps L / R (student, min peak) | turn_180 | x1.5 | push 0.5 | walk 0.3 → stop | squeeze / hold W | stride1 μ 1 / 0.4 / 0.3 | refit / refit μ 0.4 |
|---|---|---|---|---|---|---|---|---|---|
| walk18_shp10sq_s128r24 (stand candidate) | — | 0/8, 0/8 (1.1 – 1.3) | — | **0/0** | 6/6 | 0/0 | **2.05° / 0.20** | 0/0 / 0/0 / 0/0 | 0/6 / 0/8 |
| **walk21_fricact** (refit actuator + smoothness) | 71 → **71** | **5/8** (3.1), 0/8 (1.0) | 0/8 | **0/0** | **0/0** | 0/0 | 4.33° / 2.40 | 0/0 / 0/0 / 0/0 | **0/0** / 0/8 |
| walk21_fricact_lim3000 | 59 → 52 | 4/8 (2.5), 0/8 (0.8) | 2/8 (37°) | **0/0** | **0/0** | 0/8 | 2.42° / 1.53 | 0/0 / 0/2 / 0/1 | 0/8 / 0/8 |
| walk21_fricact_lim2000 | 35 → 36 | 1/8 (2.1), 3/8 (2.3) | 0/8 (175°) | 3/3 | 8/7 | 0/8 | 2.34° / 0.46 | 0/2 / 0/8 / 0/8 | 0/8 / 0/8 |

- **walk21_fricact_s128r24 is the first student to pass the refit stride1**
  (0/0; walk18 trips 6/8 and every student before it 8/8). It is also the
  best walker so far: walk_steps_L 5/8 (min peak 3.1 cm), though R is
  still 0/8 (1.0 cm). Training on the refit actuator did that, since walk21_refit, the
  control without the smoothness terms, did not run.
- **Its smoothness hypothesis did not hold.** The stop-phase peak joint
  acceleration rose rather than fell: 165 rad/s² at μ 1 and 294 on the refit
  actuator, against walk18's 97 and 170. Its stand squeeze doubled, from
  2.05° / 0.20 W to 4.33° / 2.40 W.
- **It fails the realistic floor**: refit μ 0.4, 8/8 rate trips. The
  measured sole is μ ≈ 0.45 with almost no yaw grip. That is tonight's legs
  variable.
- **The motion limiter costs the walk (not held: the gate was 0 trips on the
  refit and low-μ stride1).** At 200,3000 the teacher loses 12/160 on the
  referee and the student 19, and walk → stop trips 8/8 (pose 8, rate 4).
  At 200,2000 the policy barely turns (hErr 164 – 175°) and falls 42 – 45 %.
  **200,3000 is the setting the line continues on.** At least the limited
  students stand quietly: squeeze 2.3 – 2.4°, close to walk18's.
- **Distillation gap:** 0 % (fricact), −12 % (lim3000), +3 % (lim2000).

**Decision:**

- walk18_shp10sq_s128r24 stays the stand candidate.
- No stride1 candidate yet. walk21_fricact_s128r24 passes refit at μ 1 but
  not at the measured μ, and nothing passes refit μ 0.4.
- Tonight's legs line trains on the measured sole friction (`--contact-mu
  0.3,0.7 --contact-torsion 0,0.005`). It runs once without and twice with the
  limiter at 200,3000, the second warm-started from lim3000 to test whether
  the limiter's loss is budget.
- The sole pads still sit at r 0.068 m against the measured ~0.042. Moving
  them is a plant change for the plant pipeline (gen_plant_v6), not
  tonight's queue. The friction DR is the interim stand-in.

#### Tonight (night 16)

Legs first, then get-up. Expected timings:

- three 300M legs teachers at ~38 min each, with their distils: 22:00 – 00:05;
- three 600M get-up teachers at ~1.75 h each, plus one distil; the last
  starts ~03:40.

| order | run | parent (init) | variable | hypothesis |
|---|---|---|---|---|
| 1 | robot_legs_walk22_fmeas | walk21_fricact (walk18_shp10sq, seed 14) | `--contact-mu` 0.35,1.1 → 0.3,0.7, `--contact-torsion` 0.001,0.02 → 0,0.005 (the measured sole) | the student passes stride1 refit μ 0.4 (parent 8/8), keeps refit μ 1 0/0, x1.5 and push 0.4 0/0 |
| 2 | robot_legs_walk22_fmeas_s128r24 | walk22_fmeas | (stock distil) | |
| 3 | robot_legs_walk22_fmeas_lim3000 | walk22_fmeas (walk18_shp10sq) | + `--fw-limit 200,3000` | keeps walk22_fmeas's stride1 grades; walk → stop trips ≤ 2/8 (lim3000 8/8) |
| 4 | robot_legs_walk22_fmeas_lim3000_s128r24 | walk22_fmeas_lim3000 | (stock distil) | |
| 5 | robot_legs_walk22_fmeas_lim3000_ws | walk22_fmeas_lim3000 | init walk18_shp10sq → walk21_fricact_lim3000 (600M on the limiter in all) | referee above lim3000's 59/160, fewer walk → stop trips than its sibling; if not, the limiter costs the walk whatever the budget |
| 6 | robot_legs_walk22_fmeas_lim3000_ws_s128r24 | walk22_fmeas_lim3000_ws | (stock distil) | |
| 7 | robot_getup_w21k | w19k (t3k) | + `--fw-limit 200,3000` | the residual absorbs the 200 °/s cap (the roll reaches 267 – 349 °/s): sweep ≥ 46/48, 15 N ≥ 14/16, up within +2 s |
| 8 | robot_getup_w21k_s128r24an3 | w19k_s128r24an3 | teacher w19k → w21k | the DART student carries through the limiter |
| 9 | robot_getup_w22k | w21k (t3k) | `--fw-limit` → 200,2000 (`limit on`) | no loss vs w21k (the reference peaks at 1170 °/s²) |
| 10 | robot_getup_w21k_s1 | w21k | seed 1 | replicate |

Pre-flight: PASS 10/10. Every training job also parses with `train_mjx.py`'s
own parser, and `build_env_kw` carries `fw_limit`, `contact_mu_range` /
`contact_torsion_range` and `seed_chain_at_q` as intended. The fallback is now
the w19k recipe.

**Tomorrow's evaluation:**

- Legs: `grade.py`, each student at its trained limiter, against
  walk21_fricact_s128r24 and walk18_shp10sq_s128r24. The headline is
  refit μ 0.4 and walk → stop. Also report the referee T → S gap and the
  walk_steps peaks.
- Get-up: the sweep plus 15 / 30 N for all four, the time-to-stand against
  w19k's, and `--contact-mu 0.45` for w21k_s128r24an3.
- Check that `~/code/robot` fast-forwarded in the collect log before trusting
  any card.

### 2026-11-05 (day): STRIDE FIRST, get-up parked

Tom, 08:10: "I want you not to spend any more time on getup until we have
stride working. Always check to make sure we have the latest from github
before starting a training run. Bring in the latest relevant changes." Then:
"I want to get a stride policy we can actually test ASAP. You can use the GPU
at your discretion."

**Changes:**

- **Get-up parked.** w21k, its DART student, w22k and w21k_s1 are in
  `queue/held/`. `fallback.job` is a stride job; the get-up one is
  `fallback.job.getup-held-20261105`. Get-up returns after a student passes
  stride1 on the measured sole and the stride works on the robot.
- **Brought in:**
  - **#219, the measured-sole plant.** It had sat unpushed on a local branch
    since 01:26. It is `bimo_biped_v6legs_meas_sole.xml`: pads μ 0.5, torsion
    0, priority 1, flat at r 0.042 m, 0.15 N·m yaw grip.
  - **#221, `--w-gyro-hinge` / `--gyro-hinge`.** New; see below.
- **#220, harness.** The runner verifies origin's tip before every job and
  writes `SYNC=ok|STALE` into the job's state. The collector stashes only the
  paths that block the fast-forward. `night/claude_fix.sh` gives each failure
  to a headless Claude session, then retries once.
- **The night queue runs by day.** The runner started at 08:55.
  robot_legs_walk22_fmeas launched 08:56 (`SYNC=ok`).

**Baseline on the measured sole** (`grade.py`,
`GRADE_XML=bimo_biped_v6legs_meas_sole.xml`, 8 seeds; cells are falls / guard
trips):

| student | x1.5 | push 0.4 | walk 0.3 → stop | stride1 0.6 / μ0.4 / μ0.3 (fast) | stride1 0.6 refit / μ0.4 refit | 0.4 / 0.3 refit |
|---|---|---|---|---|---|---|
| walk21_fricact_s128r24 | 0/0 | 0/0 | 0/0 | 0/0, 0/0, 0/0 | 0/8, 0/7 | 0/8, 0/8 |
| walk18_shp10sq_s128r24 | 7/7 | 0/0 | 0/0 | 0/0, 0/0, 0/0 | 0/4, 0/7 | 0/8, 0/8 |
| walk21_fricact_lim3000_s128r24 | — | — | — | — | 0/8, 0/8 | 0/8, 0/8 |

- **No student falls on the measured sole.** walk21_fricact's yaw at +180 ms
  is −0.5 to −3° (the robot on 11-03: −19°).
- **What fails is the flail guard on body rate** (EMA |gyro| 1.6 – 1.7 against
  1.5), on the refit actuator only. A slower command does not help (0.4, 0.3).
  The spike is yaw-led: peak |ωz| 1.1 – 1.4, ωy ~0.9, ωx ~0.5.
  `--w-pitch-rate` prices roll and pitch only.
- **Hence #221:** `-w · max(|ω| − 1.2, 0)²` on the guard's own quantity.

**Day queue, in order** (each teacher then its stock distil):

1. walk23_msole_ws: walk21_fricact continued on the measured sole.
2. walk24_msole_gh4.
3. walk24_msole_gh10. Both are 1 + `--w-gyro-hinge` 4 / 10.
4. walk23_msole (init walk18; one variable against walk22_fmeas).
5. walk23_msole_ws_s1.
6. walk23_msole_lim3000_ws.
7. The three walk22_fmeas jobs the planner queued.

Each student is graded on the measured sole as it lands.

**The bar for a robot test:** stride1 0.6 refit and μ0.4 refit with 0 trips,
and x1.5, push 0.4 and walk → stop 0/0.

#### 2026-11-05 day results (stride)

All graded on the measured sole (`GRADE_XML=bimo_biped_v6legs_meas_sole.xml`, 8
seeds). Cells are falls / flail-guard trips; "walks" means walk 0.3 → stop
covers ≥ 0.3 m in 2.5 s.

| student | stride1 refit 0.6 / 0.4 / 0.3 | stride3 / walk→stop refit | fast servo x1.5, push 0.4, walk→stop | walks |
|---|---|---|---|---|
| walk23_msole_ws | 0/6, 0/5, **0/0** | 0/8 (EMA ~1.9 – 2.5) | 0/0 | 0.46 m |
| walk24_msole_gh4 (hinge 4 @ 1.2) | 0/8, 0/5, 1/1 | falls 1 – 2/8 | 0/0 | 0.46 |
| walk24_msole_gh10 | 0/0 | 0/0 | 0/0 | **no** (0.07): stands still |
| walk25_msole_d13 (STS3215 delay 1:3) | 0/6, 0/4, 0/4 | 0/8 (1.8 – 1.9) | 0/0 | 0.59 |
| walk25_msole_d13_gh4 | 0/0 | 0/0 | 0/0 | **no** (0.06) |
| walk25_msole_d23 (delay 2:3) | **0/0 at all speeds** | 0/2 – 0/5 (~1.5) | walk→stop 0/8, x1.5 1/1 | 0.49 |
| walk25_msole_d13_gh2h14 (hinge 2 @ 1.4) | 0/1, **0/0, 0/0** | 0/7 – 0/8 (1.7 – 1.8) | 0/0 | 0.45 |
| **walk25_msole_d13_gh2h14_s1** (seed 15) | **0/0 at all speeds, peak ≤ 1.05** | 0/4 – 0/7 (mean 1.43 – 1.63) | **0/0** | 0.47 |
| walk26_msole_d23_more (+300M) | 0/1 each | 0/8 | x1.5 1/1 | 0.45 |
| walk26_msole_d23_ws13 | 0/0 | 0/8 (2.0 – 2.5) | push 0.4 1/1 | 0.52 |

Lessons:

1. The guard trips at stride onset on the refit actuator because training's
   per-servo STS3215 delay draw (0:2 ticks, mean 20 ms) almost never puts every
   STS3215 at the robot's large-step 40 ms. Centring it (`sts3215=1:3`) is the
   lever.
2. A gyro hinge at 1.2 rad/s with weight ≥ 4 teaches standing still. Gate every
   guard pass on forward travel.
3. A gentle hinge (2 above 1.4) plus the 1:3 delay is the best recipe, and two
   seeds reproduce it.
4. Over several strides on refit, every walking policy runs EMA 1.4 – 2.5 rad/s
   with almost no falls. The guard's 1.5 limit, not falling, is what blocks a
   few strides. **Tom's decision:** raise the guard's rate limit while a walk
   command is active, or keep training.

**Deploy candidates (built, NOT flashed; Tom's go needed):**

- **`deploy/walk25-d13-gh2h14-s1`:** the best.
- `deploy/walk25-d13-gh2h14`.
- `deploy/walk23-msole-ws`.

Link scripts: `stride1_04` and `stride1_03` (0.2 is clamped to a stand by the
walk band).

`fallback.job` = the d13_gh2h14 recipe. Queued: walk27_gh2h14_s1_more and
walk27_gh2h14_s1_gh3. Held: walk22_fmeas (old plant), walk23_msole / msole_ws_s1
(superseded), and d13_gh6 (a stronger hinge collapses).

### Night of 2026-11-05 (evaluated 2026-11-06)

**Stride only (get-up parked, Tom 11-05).** The night ran from 19:30 on 11-05 to 02:29 on 11-06, all
`SYNC=ok`. One fixer run: the referee crashed on a seed that fell before metronome's settle (#231, a
proper fix with a test; no stopgap to finish). `night/fixer.log` shows nothing else.

**Merged / branched:** `train/omni-night` is merged to main (`--dq-obs-window`, `--dq-obs-quantum`,
`--cmd-crouch-range`, `--init-norm-crouch`, the dq-lag probes), because its students are the only ones
that survive the robot probes below. Also new today: `--yaw-kick-range` (off by default, bit-exact),
the stop-timing, `robot`, margin and yaw-kick probes, and `experiments/policy/yaw-gap/`. Not brought in:
`fw/power-lite-rail` (#229) and `cad/shoulder-wire-window` (#228), which don't bear on stride;
`fw/walkrec-tilt-in-grace` is the hardware session's and awaits Tom.

**Upstream:** #230 removed the pre-v6 plants and #227 merged the walk-recovery firmware. Neither changes
the legs plant, so comparisons hold. Collect still can't build SIL numbers for these runs: `obs_spec.h`
on main is the 147-dim walk16 spec and these runs need 165. Deploy branches regenerate it, so this does
not block a deploy, but the card has no SIL number for any of them.

**The robot, 11-06 morning (Tom's go; another session flew it, omni5_dqlag_s128r24 on carpet):**

- stride1_03: stood. Rate trip 3.08 at +904 ms; yaw swung +27 to -17 deg and ended -6.7 (IMU).
- steps5_03: about 4 steps, rate trip 3.07 at +1200 ms, then tipped over at about +2.6 to +3.15 s.
- Flight recorder on for both, so we have the first per-tick data on this policy family.
- **The robot is out of service.** Re-homing for the next test read a ~5 deg lean with every joint at zero;
  Tom found a broken leg link, most likely from the steps5 fall, and is printing a replacement.
  `stride1_03e` was not run, and no robot tests are planned until Tom reports the repair. It stays the next
  test. PR #232 (tilt judged during the walk-recovery grace, the gap this fall exposed) is open for Tom.

**What today's analysis found (`experiments/policy/yaw-gap/README.md`):**

1. **The stride command outlives the clock cycle.** stride1_03 ended at +700 ms, past the stand phase, so
   the coasting clock ran to +1320 ms under the stand command and the robot took a third swing. Grade's
   schedule ended on the tick that lands on the stand phase, so it never saw this. Ending 20 or 40 ms
   late trips all 13 students 8/8; 40 ms early is clean for 7 of 13, including all four dq-lag students. The hardware session has built
   `stride1_03e` (ends 57 ms early) on `deploy/omni5-dqlag-walkrec`, awaiting Tom's go.
2. **The refit actuator is too fast.** Robot peaks: hip pitch 246 - 254, knee 281 - 303 deg/s, in all
   three flight records. The refit sim reaches 360+. The new `robot` probes use 0.87 / 1.0 and 60 ms.
3. **Robot-like dq is the lever that works in sim.** Only the four dq-lag students have 0 falls on all
   six `robot` probes. But the same policy fell on the robot, so this gate is necessary, not sufficient.
4. **The delay margin is thin.** Probes at dq window 5 / 6 are the first to reach the robot's rate
   (omni5 steps5: 3.08 / 3.36, 2/8 falls each, robot 3.07). The register fit says window 2, so this does
   not show the robot has that lag; it shows where the margin ends.
5. **The second swing's yaw is still open.** +42 deg on the robot by +720 ms vs about +9 in the sim at
   the robot's speed. Replaying the robot's joints on the plant does produce it, and contact barely
   matters on the first swing, which corrects 11-05's "the floor gives 2.5x too little" reading. The sim
   is not producing the robot's joint motion. Hypothesis, untested: a late right-foot landing leaves the
   body nearly free while the legs scissor.

**Students** (`grade.py`, measured sole, 8 seeds; falls/trips at the 1.5 line; grades in
`/tmp/plan1106/` on mira; firmware's walking rate limit is now 3.0, so a walking "trip" here is not a
firmware trip). All WALK. Directions f / b / L / R / turn L / turn R; ideals +1.2 / -1.0 / +0.6 / -0.6 /
+115 / -115.

| student | parent, variable | card T→S | stride1 refit 0.6 / 0.4 / 0.3 | x1.5, push, walk→stop | robot: stride1, late40, steps5 | steps5 dq5, dq6, dead80 | directions | verdict |
|---|---|---|---|---|---|---|---|---|
| **omni5_dqlag** | omni1 + dq lag | 39→**51** | 0/5, 0/0, 0/0 | 0/0 ×3 | 0/8, 0/8, 0/8 | 2/8, 2/8, 1/8 | +1.35 -0.88 +0.51 -0.76 +74 -125 | **held; on the robot**, fell on steps5 |
| omni5_dqlag_s1 | seed | 39→42 | 0/0, 0/8, 0/8 | 0/0 ×3 | 0/8, 0/8, 0/8 | **0/8, 1/8, 0/8** | +1.37 -0.94 +0.83 -1.00 +99 -128 | best delay margin and directions; trips stride1 |
| omni6_dqlag_crouch | omni5 + crouch channel | 33→37 | 0/8, 0/8, 0/7 | 2/2, 0/0, 0/6 | **0/0**, 0/8, 0/4 | 1/8, 4/8, 0/8 | +1.48 -0.96 +0.80 -0.59 +84 -116 | lowest rates on `robot`; loses x1.5 |
| omni6_dqlag_crouch_s1 | seed | 34→29 | 0/7, 0/7, 0/3 | 0/0, 0/0, 0/8 | 0/7, 0/8, 0/6 | 3/8, 6/8, 0/8 | +1.35 -1.07 (3 falls) +0.72 -0.49 +8 -141 | lost turn L |
| omni1_d13_sub25 | walk21 + submix 0.25 | 31→32 | 0/0 ×3 | 8/8, 0/0, 0/0 | 4/7, 7/8, 0/8 | 8/8, 8/8, 8/8 | +1.40 -0.96 +0.72 -0.80 +59 -145 | six directions, no margin |
| omni1_d13_sub25_s1 | seed | 36→34 | 0/0 ×3 | 0/0 ×3 | 0/4, 0/8, 0/8 | 8/8, 7/8, 8/8 | +1.46 -1.04 +0.69 -0.82 +80 -131 | passes the 11-05 bar, fails every margin probe |
| omni3_d13_sub30 | submix 0.30 | 37→36 | 0/0, 0/1, 0/0 | 4/7, 0/0, 0/0 | 1/7, 0/8, 2/8 | 8/8, 8/8, 6/8 | +1.15 -1.20 +0.55 -0.59 +65 -127 | no gain over 0.25 |
| omni2_d13_gh2h14_sub25 | omni1 + hinge 2@1.4 | 34→24 | 0/0 ×3 | 0/0 ×3 | 3/4, 3/8, 6/7 | 8/8 ×3 | +1.11 0 0 0 +12 -6 | **hinge erased five directions** |
| walk28_d13_gh3h14 | walk25 + hinge 3@1.4 | 25→32 | 0/0 ×3 | 0/0 ×3 | 0/0, 7/8, 2/8 | 8/8 ×3 | +0.97 0 0 0 +2 -67 | forward only; stride3 0/0 on refit, a first |
| walk28_d13_gh3h14_s1 | seed | 25→30 | 8/8, 5/8, 5/6 | 8/8, 0/0, 0/0 | 3/3, 1/8, 7/8 | 8/8 ×3 | +1.19 0 0 -0.49 +8 -12 | not replicated |
| walk27_gh2h14_s1_more | +300M | 23→24 | 0/0 ×3 | 6/6, 0/0, 0/1 | 8/8, 2/8, 3/8 | 8/8 ×3 | +1.15 0 0 0 +53 -51 | continuation lost x1.5 again |
| walk27_gh2h14_s1_gh3 | hinge 3 | 26→27 | 0/0 ×3 | 7/7, 3/0, 0/0 | 0/3, 0/8, 1/8 | 8/8, 8/8, 1/8 | +1.13 0 0 0 +43 -55 | lost x1.5 |

**Hypotheses:**

- Train on the robot's dq: **held in sim.** Falls on the `robot` probes went to 0 and chatter stayed at
  3.2 or less. On the robot, neither of today's two runs tripped on chatter, but the rate trips and a
  steps5 fall remain.
- Crouch channel: **partly.** At crouch 0.9 omni6 has no falls (trips 6/6 at the 1.5 line) and its seed
  falls 4/6 on stride1; omni5, untrained on the channel, falls 6/6.
- Hinge at 1.4 with omni: **did not hold.** It erases directions.
- Submix 0.30: **no gain.**

**Distil gap** (teacher→student on the card): -10 to +12, median about +1. Not the limit right now.

**Tonight (each followed by its stock distil; BRANCH=main):**

| # | run | parent | variable | hypothesis |
|---|---|---|---|---|
| 1 | omni7_rs | omni5_dqlag | the robot actuator: `--servo-envelope` hip 0.87, knee 1.0; delay `sts3215=2:4` | planned for the joints it has: `robot` steps5 rate and dq5 / dead80 falls down, six directions kept |
| 2 | omni7_rs_s1 | omni7_rs | seed | replicate |
| 3 | omni8_rs_dq25 | omni7_rs | `--dq-obs-window 2,5` | dq5 and dq6 0 falls, rate under 2.5 |
| 4 | omni8_rs_dq25_s1 | omni8_rs_dq25 | seed | replicate |
| 5 | omni9_rs_crouch | omni7_rs | crouch channel | keeps omni6's low rate; grade at crouch 0.9 |
| 6 | omni10_rs_yk | omni7_rs | yaw kicks 0.5 - 2.5 rad/s | stride1 yk3 falls to 0 (omni5 4/8) |
| 7 | omni11_rs_lowswing | omni7_rs | swing height 0.06, lift 0.05 | less rate and yaw, every swing still >= 3 cm |
| 8 | omni12_rs_grace07 | omni7_rs | `--stop-grace-s 0.7` | stride1 late40 trips down |
| 9 | omni13_rs_gh1h20 | omni7_rs | hinge 1 above 2.0 rad/s | less runaway rate, all six directions kept |

Rank tomorrow by the planner's three criteria, then the `robot` and margin probes. No student reaches
hardware on these alone: the robot's result outranks them.

**Fallback:** the omni7_rs recipe.

### Night of 2026-11-06 (evaluated 2026-11-07)

**Precision at reduced speed (Tom 11-06, after omni14_shp8 on the robot: "significant side to side movement
of the feet with not as much forward motion as desired, but the servos are still moving very fast. focus on
the precision of movement with reduced speed").** Stride only; get-up stays parked.

**The runner** ran from 12:32 on 11-06 (daytime GPU is allowed) to 05:13 on 11-07, with no break. The
interactive robot session queued and graded the `prec*` line and held the planner's omni9-13 (1.5 Hz
variants; note `queue/held/WHY-1106220009-omni9..13.txt`). There were 24 teachers and distils, all exit 0
and all on the verified origin tip (`train/gait-freq`, `train/joint-speed` or `main`). The fixer did not
run (`fixer.log` ends 11-06 08:01). `omni8_rs_dq25_s1`'s distil missed the 05:00 cut and runs tonight.

**The robot, 11-06 (`hw_sessions/2026-11-06/NOTES.md`): back in service.** Tom replaced the broken leg
link, and the robot ran with a 2.5 deg lean baseline.
- omni5_dqlag on `stride1_03e` (the command ends 57 ms early): no third swing, which fits the late-stop
  finding. But the rate still tripped (3.00) during the second swing, and on battery (3.18) with a +55 deg
  left spin.
- omni14_rs_shp8 at `shape 8`, battery:
  - Run 1: a normal stride. Then the robot tipped slowly at the stand and the policy folded both hips. That
    tripped the pose guard 2.26 s after the walk, outside the recovery window, and it fell during the hold's
    autohome. Firmware note in NOTES: the autohome should need < 10 deg.
  - Runs 2-3 (shorter post-stride window): **no trip and no fall**. Yaw peaked 33-36 deg and ended at
    +4 / -3 deg with heading hold on.
- Tom's verdict on runs 2-3 launched the prec line. In flight, the knees peak at 281-299 deg/s and the
  hip pitch at 257-275, both at servo top speed. The L/R knee range is 72 / 51 deg.

**The collect cards for prec5-prec16 are STALE.** `~/code/robot` referees on `main`, which had no
`--gait-freq`. Its CPU env drops `gait_freq_range`, so every 1.0 / 0.75 / 0.6 Hz policy was scored on the
old 1.25-1.75 Hz clock. I re-refereed six students on `train/joint-speed`
(`/tmp/plan1107/ref_*.log`):

| student | stale card | correct clock | walk_steps L/R min peak | swings >= 3 cm (median peak) |
|---|---|---|---|---|
| prec16_f075_js20 | 25/160, falls 54 % | 25/160, falls 55 % | 0.3 / 0.4 cm | 4 % (1.2 cm) |
| prec13_f075_l2 | 26, 59 % | 29, 39 % | 0.3 / 0.2 | 8 % (1.4) |
| prec14_f06_l2 | 19, 44 % | 25, 34 % | 0.4 / 0.3 | 9 % (1.3) |
| prec15_f075_cap180 | 18, 75 % | 19, 63 % | 0.4 / 0.5 | 18 % (1.9) |
| prec12_f075_l3 | 24, 63 % | 22, 56 % | 0.5 / 0.5 | 44 % (2.5) |
| prec10_f10_js20 | 33, 54 % | 29, 62 % | 1.6 / 1.3 | 81 % (4.1) |

The family card commands up to 0.7 m/s, and this line trains on 0.05-0.24, so the card is not its
yardstick. grade.py is. Fixed for the future: `train/joint-speed` is merged to main today (below).

**The 2 cm lift is a shuffle.** prec16 reaches a median swing peak of 1.2 cm, and its lowest step peaks
at 0.3 cm. Tom's 10-24 rule is >= 3 cm on every step, and on carpet a 1 cm swing risks stubbing. Tonight's
prec22 tests a 3 cm lift with the joint-speed cost. **Tom: does the 3 cm rule still hold for the
reduced-speed gait?**

**Merged / branched:** I merged `train/joint-speed` (`--gait-freq`, `--w-joint-speed` / `--joint-speed-soft`;
both opt-in, defaults byte-identical, `tests/test_gait_freq.py`) to main. The morning referee then runs
each policy on its own clock, and tonight's prec jobs could run on main. They still name
`BRANCH=train/joint-speed`: the training code is the same, and main only adds #248. `deploy/prec16-f075` (the prec16 student, not
flashed) and `fw/heading-hold` (#247) belong to the hardware session and await Tom. Not brought in:
#228, #229, #232 (they don't bear on stride training) and #206 (the console).

**Upstream:** #248 (gen_obs_spec's dq-lag view off for the obs vectors) changes no training or plant.
Comparisons hold.

**Students** (`grade.py` on `train/joint-speed`, measured sole, 8 seeds, SHAPE_HZ = the trained shaper;
falls / trips at the 1.5 line; `/tmp/plan1107/grade{A,B}.md`). Firmware's walking limit is 3.0, so a walking
"trip" here is not a firmware trip. The hardware session's WalkRecovery grade (`grade_recover.py`)
gave prec16 0 / 0 on every probe. Directions: f / b / L / R / turn L / turn R, ideals +1.2 / -1.0 /
+0.6 / -0.6 / +115 / -115.

| student | parent, variable | x1.5, push, walk→stop | stride1 refit 0.6 / 0.4 / 0.3 | robot: stride1, late40, steps5, back3 | dq5, dq6, dead80 | directions | mean knee L/R deg/s (steps5 0.3) | verdict |
|---|---|---|---|---|---|---|---|---|
| **prec16_f075_js20** | prec13 + js 0.2 @ 120 | 0/0, 0/0, 0/8 | 0/8, 0/6, 0/7 | **0/1, 0/0, 0/0, 0/0** | **0/0, 0/0, 0/0** | +0.92 -0.86 +0.47 -0.49 +114 -106 | **77 / 82** (p95 170) | **held**, the best student, but it shuffles (1.2 cm) |
| prec13_f075_l2 | prec6, 0.75 Hz, 2 cm | 0/0, 0/0, 0/8 | 1/8, 0/8, 0/8 | 0/8, 0/8, 0/0, 0/0 | 0/0 ×3 | +0.46 -0.99 (2 falls) +0.47 -0.47 +133 -128 | 87 / 118 (p95 214) | the slower clock works; fwd short |
| prec14_f06_l2 | prec6, 0.6 Hz | 0/0, 0/0, 0/8 | 0/8 ×3 | 0/8, 0/8, 0/0, 0/8 | 0/5, 0/8, 0/0 | +0.54 -0.76 +0.33 -0.44 +137 -89 | (p95 195-212) | **does NOT walk** on walk→stop (v cap 0.19) |
| prec15_f075_cap180 | prec13 + limit 180,3000 | 0/0, 0/0, 0/8 | 0/7, 0/6, 0/3 | 0/8, 0/8, 0/8, 0/3 | 0/8 ×3 | +0.62 -0.41 (2) +0.32 **-0.03** +79 -112 | (p95 169) | not held: lost side R |
| prec12_f075_l3 | prec6, 0.75 Hz, 3 cm, low-swing 10 | **8/8**, 0/0, 3/8 | 0/8 ×3 | 0/8, 0/8, 0/0, 0/8 | 0/0, 0/1, 0/1 | +0.34 (2) -1.43 (**8**) +0.40 -0.48 +128 -77 | (p95 216) | not held: x1.5 and back |
| prec10_f10_js20 | prec6 + js 0.2, 1.0 Hz | **5/5**, 0/0, 0/8 | 0/8 ×3 | 0/8, 0/8, 0/4, 0/8 | 0/7, 0/8, 0/6 | +1.09 -1.06 (1) +0.49 -0.65 +90 -121 | (knee range 123/82) | lost x1.5 |
| omni14_rs_shp8 | omni7 + shaper 8 | 0/0, 0/0, 0/0 | 0/8 ×3 | 1/4, 0/8, 0/8, 0/8 | 0/8, 1/8, 0/8 | +1.25 -0.80 (**8**) +0.80 -0.44 +107 -118 | 164 / 178 | flown 11-06; loses back |
| omni7_rs | omni5 + robot actuator | 0/0, 0/0, 0/0 | 0/8 ×3 | 0/8, 0/8, 0/8, **8/8** | 0/8, 2/8, 0/8 | +1.20 -0.71 (**6**) +0.59 -1.05 +148 -130 | — | not held: loses back |
| omni7_rs_s1 | seed | 0/0, 0/0, 0/1 | 0/8 ×3 | 0/8 ×4 | 0/8 ×3 | +1.13 -0.86 (**8**) +0.92 -0.78 +150 -129 | — | replicates the back loss |
| omni8_rs_dq25 | omni7 + dq window 2,5 | **3/3**, 0/0, 0/6 | 0/6, 0/8, 0/8 | 0/2, 0/8, 0/3, 0/8 | 0/8, 0/8, 0/6 | +1.18 -0.89 (**8**) +0.64 -0.19 (3) +21 (2) -81 (5) | — | **not held**: lost x1.5 and turns |

Not graded: prec1/3/4 (the 180 deg/s limit at 1.5 Hz stands still: dx 0, walk_steps peak 0.0 cm),
prec2 (lower lift alone: same speed), prec5/6/8/9 (1.0 Hz: knees still at the ceiling, per the hardware
session's speed table), omni14_shp6 (the parked 1.5 Hz line).

**Hypotheses:**

- The robot actuator (omni7): **did not hold.** The robot-probe falls stay at 0, but walking backward falls
  6/8 (seed 8/8). dq window 2,5 (omni8): **did not hold**. It lost x1.5 and the turns.
- Firmware speed limit alone (prec1/3/4 at 1.5 Hz, prec15 at 0.75): **did not hold.** It stands still, or
  loses a direction.
- Slower clock (prec5 / prec13 / prec14): **held, at 0.75 Hz.** At 1.0 Hz the speed doesn't change. At 0.6 Hz
  it no longer travels 0.3 m on walk→stop.
- Soft joint-speed cost on the 0.75 Hz gait (prec16): **held.** Mean knee 77/82 deg/s (was 164/178), and 0
  falls on every probe including the delay margin. Open: knee peaks ~229, yaw drift ~+29 deg per 5 steps,
  L/R knee range 91/50, and the 1.2 cm shuffle.

**Distil gap** (teacher→student on the stale cards): prec16 22→25, prec13 26→26, prec12 18→24, prec10
24→33, omni7 31→34, omni7_s1 29→39, omni8 32→27, omni14_shp8 32→38. Median about +3, so distillation is
not the limit.

**SIL:** collection still builds no SIL number (`libctrl_sil.so` is built for the walk16 spec; these runs
need 165 dims). Deploy branches regenerate it.

**Tonight** (each followed by its stock distil). The hardware session queued the first two. All are
`BRANCH=train/joint-speed` and 300M from prec16 unless noted:

| # | run | parent | variable | hypothesis |
|---|---|---|---|---|
| 0 | distil omni8_rs_dq25_s1 | — | (left over) | — |
| 1 | prec18_f075_js20_s1 | prec13 | seed 15 (prec16's recipe) | prec16's 0 falls / 0 trips are the recipe, not the draw |
| 2 | prec17_f06_js20 | prec16 | 0.6 Hz clock, v 0.05-0.19, 600M | lower mean speed with 0 falls kept (prec14 lost walk→stop travel) |
| 3 | prec19_f075_js20_cap180 | prec16 | `--fw-limit 180,3000` | knee peak < 180 deg/s without prec15's lost side R |
| 4 | prec20_f075_js20_mirror | prec16 | `--w-mirror-loss 1.0` (maps checked on the 165-dim obs) | L/R knee range 91/50 closes to ~15 deg; yaw drift falls |
| 5 | prec21_f075_js40 | prec16 | `--w-joint-speed 0.4` | mean knee < 77/82 with 0 falls; travel >= 0.6 m / 5 steps |
| 6 | prec22_f075_js20_l3 | prec16 | lift 3 cm (`--mimic-lift 0.03 --swing-height 0.035`) | >= 3 cm clearance comes back at mean knee < 100 deg/s |
| 7 | prec23_f075_js20_head | prec16 | `--w-heading 1.0` (was 0) | yaw drift +29 → < 10 deg per 5 steps, turns kept |
| 8 | prec24_f075_js20_crouch | prec16 | crouch channel (`--cmd-crouch-range 0.85,1.0 --init-norm-crouch`) | Tom's knee bend at 0.9 with x1.5 kept (omni6 lost it at 1.5 Hz) |

Rank tomorrow: 0 falls on the robot / margin probes and all six directions first, then mean knee
speed, time above 180, knee symmetry, yaw drift and swing clearance. Grade on main (merged) with
SHAPE_HZ=8.

**Fallback:** the prec16 recipe (was omni7_rs, saved as `fallback.job.omni7rs-20261107`).

### Night of 2026-11-07 (evaluated 2026-11-08)

**Precision at reduced speed, night 2.** Stride only; get-up stays parked. There was no new comment from Tom on
#137, so **the 3 cm question is still open**.

**The runner** ran 22:00 – 04:26. It ran the left-over omni8_rs_dq25_s1 distil, then 8 prec teachers and their distils. All
exited 0 on the verified origin tip (`train/joint-speed` d54fe0f, which is identical to main's training code: main adds only #250's
dead-code cleanup, and `git diff d54fe0f origin/main -- sim/mjx` touches only `eval_stand_arm.py`). The fixer did not
run (`fixer.log` ends 11-06 08:01). The collection cards are on main, which carries `--gait-freq` since 11-07, so they
were scored on the right clock this time. Collection still gives no SIL number: `libctrl_sil.so` is built for the walk16
spec. This is known, and the deploy branches regenerate it.

**Merged / branched:** nothing. No open PR or local branch bears on stride training. `deploy/prec16-f075` (the prec16
student, not flashed), #247 (heading hold) and #232 (tilt in the walk-recovery grace) are firmware/deploy branches for
Tom and the hardware session. Tonight's jobs run on `BRANCH=main`. The training code there is byte-identical to last
night's, and every recipe was parsed with main's `make_parser()` before preflight.

**Students** (`grade.py` on main, measured sole, 8 seeds, SHAPE_HZ=8; falls/trips; `/tmp/plan1108/gradeA.md`, `gradeC09.md`).
Every slow-clock student trips `pose` on walk→stop and stride1 refit: the closing step outlasts the 0.3 s pose hold
(known, see 11-07). The robot probes are stride1 0.3 / late40 / early40, steps5, back3 and walk→stop. Directions are f / b / L / R / turn L / turn R
(ideal +1.2 / −1.0 / +0.6 / −0.6 / +115 / −115).

| student | variable vs parent | x1.5, push, walk→stop | stride1 refit 0.6 / 0.4 / 0.3 | robot probes | dq5, dq6, dead80 | directions (falls) | verdict |
|---|---|---|---|---|---|---|---|
| prec16_f075_js20 (re-graded) | — | 0/0, 0/0, 0/8 | 0/8, 0/6, 0/7 | 0/1, 0/0, 0/0, 0/0, 0/0, 0/8 | 0/0 ×3 | +0.92 −0.86 +0.47 −0.49 +114 −106 (0) | **reproduces** 11-07 |
| prec18_f075_js20_s1 | seed 15 of prec16's recipe | 0/0, 0/0, 0/8 | 0/8 ×3 | **0/8, 0/8, 0/8**, 0/0, 0/0, 0/8 | 0/0, 0/1, 0/0 | +0.91 −0.83 +0.43 −0.54 +122 −97 (0) | **half held**: 0 falls everywhere replicates; prec16's 0 robot trips was the draw (8/8 `pose` at 3.6 s) |
| prec17_f06_js20 | 0.6 Hz, 600M | **8/8**, 0/0, 0/8 | 0/1, 0/2, 0/8 | 0/2 ×3, 0/0, 0/0, 0/8 | 0/0, 0/1, 0/2 | +0.61 −0.67 +0.29 −0.39 +106 −95 (0) | **not held**: lost x1.5, does NOT walk |
| prec19_f075_js20_cap180 | `--fw-limit 180,3000` | 0/0, 0/0, **4/8** | 0/8, 0/4, 0/7 | 0/1 ×3, 0/2, 0/8, 0/7 | 0/8, 0/8, 0/3 | +0.44 −0.60 (1) **+0.02 −0.10** (1) +115 −100 | **not held**: the L knee moves 4 deg (one-legged hop, yaw +74 on steps5) |
| prec20_f075_js20_mirror | `--w-mirror-loss 1.0` | 0/0, 0/0, 0/8 | **5/8**, 0/8, 0/2 | 0/0, 0/2, 0/8, 0/0, 0/6, 0/8 | 0/0, 0/1, 0/0 | +0.86 −0.92 +0.37 −0.46 +108 −107 (0) | **symmetry held** (below); yaw moved, not fixed; stride1 0.6 is out of band |
| prec21_f075_js40 | js 0.4 | 0/0, 0/0, 0/8 | **6/7**, 0/8, 0/8 | 0/4, 0/7, 0/0, 0/0, 0/0, 0/8 | 0/0, 0/2, 0/0 | +0.84 −0.79 **+0.01** −0.40 +109 −143 (0) | **not held**: lost side L |
| prec22_f075_js20_l3 | 3 cm lift | **8/8**, 0/0, **5/8** | **8/8**, 1/7, 0/0 | 0/0 ×3, 0/1, 0/8, 0/8 | 0/8, 0/8, 0/0 | +0.78 −1.01 +0.49 −0.48 +115 −126 (0) | **not held**: lost x1.5 and walk→stop |
| prec23_f075_js20_head | `--w-heading 1.0` | **4/4**, 0/0, 0/8 | 0/8 ×3 | 0/8, 0/8, 0/0, 0/3, 0/0, 0/8 | 0/0 ×3 | +0.73 −0.86 +0.43 −0.39 **+136 −134** (0) | **not held**: lost x1.5; turns overshoot |
| **prec24_f075_js20_crouch** @ 1.0 | crouch channel 0.85-1.0 | 0/0, 0/0, 0/8 | 0/8, 0/8, 0/6 | 0/0, 0/8, 0/1, 0/0, 0/8, 0/8 | 0/1, 0/0, 0/0 | +0.88 −0.84 +0.44 −0.46 +133 **−81** (0) | **held** |
| **prec24** @ crouch 0.9 | (graded with cmd[3] = 0.9) | 0/0, 0/0, 0/8 | 0/7, 0/8, 0/8 | 0/8, 0/8, 0/0, 0/0, 0/8, 0/6 | 0/0 ×3 | +0.82 −0.84 +0.47 −0.45 +110 **−79** (0) | **held**: Tom's knee bend with 0 falls on every probe; turn R short |
| omni8_rs_dq25_s1 (shape 10) | seed of omni8 | 0/0, 0/0, 0/8 | 1/8, 0/8, 0/8 | 0/8 ×6 | 0/8, 0/8, 1/8 | +1.08 **−0.68 (6)** +0.64 −0.44 +111 −107 | **not held**: replicates omni8's back loss |

**Speed table** (new: `experiments/policy/shaper-train/speed.py`, grade.py's episode with the speed stats added. It takes the 50 Hz qvel over the walk window of
`steps5 0.3 robot` (2.5 cycles at 0.3 m/s), 8 seeds. "above 180" is the share of walk ticks where any knee or hip pitch is above 180 deg/s.
This method reads prec16 at 83/68; the hardware session's method read 77/82. Compare students within this table only.)

| student | mean knee L/R | mean hip pitch L/R | knee p95 / peak | above 180 | knee range L/R deg | dx m | yaw deg (steps5 / fwd 4 s) |
|---|---|---|---|---|---|---|---|
| prec16 | 83 / 68 | 66 / 76 | 220 / 229 | 0.52 | 67 / 47 | +0.68 | +38 / +2 |
| prec18 (seed) | 91 / 85 | 67 / 81 | 221 / 229 | 0.60 | 71 / 62 | +0.72 | +34 / +3 |
| prec17 (0.6 Hz) | 75 / 64 | 58 / 56 | 218 / 229 | **0.32** | 73 / 56 | +0.65 | +10 / −4 |
| prec19 (cap 180) | **6** / 79 | 21 / 57 | 220 / 231 | 0.28 | **4** / 65 | +0.34 | +74 / +101 |
| prec20 (mirror) | **73 / 74** | 67 / 67 | 219 / 226 | 0.44 | **57 / 54** | +0.70 | **+12 / −54** |
| prec21 (js 0.4) | 73 / 57 | 59 / 76 | 219 / 227 | **0.26** | 56 / 40 | +0.74 | +22 / +3 |
| prec22 (3 cm) | 85 / 84 | 74 / 85 | 220 / 228 (fwd: 297) | 0.51 | 67 / 58 | +0.82 | +31 / −2 |
| prec23 (heading) | 83 / 76 | 60 / 76 | 223 / 229 | 0.46 | 66 / 52 | +0.67 | **+19 / −20** |
| prec24 (crouch 1.0) | 85 / 79 | 68 / 86 | 220 / 230 | 0.47 | 67 / 55 | +0.74 | +32 / +8 |
| prec24 (crouch 0.9) | 84 / 79 | 70 / 84 | 220 / 229 | 0.43 | 68 / 54 | +0.69 | +29 / +9 |

The knee peak, ~229 deg/s, sits at the robot-speed plant's ceiling in every student. No variable moved it. Yaw is
deterministic per probe (per-seed spread is ±5 deg): prec20 turns −49 to −59 on every fwd seed.

**Clearance** (student scorecards, `walk_steps` L/R min peak, and the share of swings ≥ 3 cm with the median peak): prec16 0.3/0.4 cm, 4 % (1.2 cm).
prec22 (3 cm lift) reaches 0.4/0.4 cm, 17 % (1.8 cm). Every other prec student stays at 0.2 – 0.4 cm and 3 – 7 % (1.1 – 1.5 cm).
**The shuffle survives every variable tried.** Diagnosis: the swing-height kernel is
`exp(-err²/feet_phase_s2)` with s2 = 0.002, so a 1 cm swing against the 2.5 cm target still earns 0.73 of the reward,
and the policy pays almost nothing for shuffling. Tonight tests a tighter kernel (s2 0.0008: 0.46) and a higher
kernel target with the reference unchanged. That is a different lever from prec22, which raised the joint-space reference and
lost margin.

**Hypotheses:**
- Seed (prec18): 0 falls and six directions **held**. prec16's 0 robot-probe trips **did not hold**: they are `pose` trips
  at the closing step, the slow-clock pattern.
- 0.6 Hz with js (prec17): **not held**. It halves the time above 180 but loses x1.5 (8/8) and walk→stop travel. The
  0.6 Hz clock is closed for now.
- Firmware 180 deg/s cap (prec19): **not held**, the third failure (prec1/3/4, prec15, prec19). The left leg
  stops stepping. The cap is closed as a training variable.
- Mirror loss (prec20): **held for knee symmetry** (range 67/47 → 57/54, mean 73/74). It does not fix yaw: the
  drift changes sign by probe (+12 / −54), and stride1 0.6 refit falls 5/8 (0.6 m/s is outside the 0.05-0.24 band).
- js 0.4 (prec21): **not held**. It halves the time above 180 but loses side L (dy +0.01).
- 3 cm lift (prec22): **not held**. Clearance rises a little (median 1.8 cm) at the cost of x1.5 8/8 and walk→stop 5/8 falls.
- Heading 1.0 (prec23): yaw drift halves, but x1.5 is lost (4/4). **Not held** as a candidate. Tonight tries 0.3.
- Crouch channel (prec24): **held**. 0 falls on every probe at crouch 0.9 and 1.0. Speed is unchanged from prec16. Turn R is
  short (−80 vs −115).

**Distil gap** (teacher → student, card seed-passes): prec18 25 → 27, prec17 18 → 22, prec19 17 → 19, prec20 29 → 23,
prec21 26 → 23, prec22 28 → 35, prec23 25 → 29, prec24 27 → 34. Median about +2. Distillation is not the limit.

**Ranking (deploy view):** prec24 (crouch 0.9) ≥ prec16 > prec18 > prec20. None clears 3 cm. If Tom accepts the shuffle,
prec24 is the candidate to put on the robot after prec16. The firmware already carries the crouch command channel
(cmd[3]: `linkproto` watchdog `cmd_crouch_`, default 1.0, and battguard's landing crouch). I have not checked which
operator command sets it to 0.9.

**Tonight** (9 teachers + distils, `BRANCH=main`, 300M warm starts, ~43 min a pair, ends ~04:30):

| # | run | parent | variable | hypothesis |
|---|---|---|---|---|
| 1 | prec25_f075_js20_s2t | prec16 | `--feet-phase-s2 0.0008` | median swing peak ≥ 1.8 cm with prec16's margin |
| 2 | prec26_f075_js20_sh35 | prec16 | `--swing-height 0.035` (reference lift stays 2 cm) | clearance up without prec22's knee-speed cost |
| 3 | prec27_f075_js20_ls2 | prec16 | `--w-low-swing 2` | lowest step 0.3 → ≥ 1 cm, 0 falls kept |
| 4 | prec28_f075_js20_crouch_mirror | prec24 | `--w-mirror-loss 1.0` | prec20's symmetry on the knee-bend policy, 0 falls kept |
| 5 | prec29_f075_js20_crouch_s1 | prec16 | seed 15 (prec24 recipe) | prec24's 0 falls is the recipe |
| 6 | prec30_f075_js20_head03 | prec16 | `--w-heading 0.3` | steps5 yaw < +20 deg with x1.5 kept |
| 7 | prec31_f075_js30 | prec16 | js 0.3 | time above 180 −25 %, six directions kept |
| 8 | prec32_f075_js20_crouch_s2t | prec24 | `--feet-phase-s2 0.0008` | the bent-knee gait lifts ≥ 1.8 cm |
| 9 | prec33_f075_js20_mirror_s1 | prec16 | seed 15 (prec20 recipe) | is prec20's stride1 0.6 / fwd-yaw cost the recipe or the draw |

Rank tomorrow: 0 falls on the robot, margin and direction probes first, then clearance (median swing peak, lowest step),
then speed (the speed table, same method), knee symmetry and yaw. Grade crouch runs at 0.9 as well
(`experiments/policy/shaper-train/grade_crouch.py`, which patches `set_command`'s cmd[3]).

**Fallback:** the prec24 recipe (crouch channel), saved over the prec16 one (`fallback.job.prec16-20261108`).
Preflight passed 18/18.

### Night of 2026-11-08 (evaluated 2026-11-09)

**Precision at reduced speed, night 3.** Stride only; get-up stays parked. Tom has not commented on #137 since 11-04, so
**the 3 cm question is still open**. There was no hardware session (no new `hw_sessions/2026-11-0[7-9]` directory).

**The runner** ran 22:00 – 05:39 on main (`769812d` → `8681f63`; the commits in between are the SIL twin and the firmware
console, with no training code changed). It ran 8 of the 9 teachers and 7 distils. The prec32 distil and the prec33 pair
did not start before 05:00, so they run first tonight. One fetch failed on DNS at 00:06 and the retry succeeded.

**Fixer (4 sessions, all on prec28).** prec28 died with a CUDA OOM in Warp's `convex_narrowphase` scratch (99 MB) four
times. Each time it was at the **same 16th eval**: twice in full runs at 229M steps, and twice in resumes at +55M. Fixer 1 called it
transient. Fixer 2 blamed an ollama model loading onto the GPU and wrote a Warp mempool hold (branch
`fix/warp-mempool-hold`, 61f986f). Fixer 3 saw the third OOM with no ollama model loaded and ran the hold. Fixer 4 saw the hold fail
too and found the pattern: brax derives the eval key from the seed, so eval index 15 replays the same rollout on every
attempt. It re-queued the resume (`--init-from …_oom229M_b --steps 70624000 --num-evals 12`), which finished.
- **Not merged:** `fix/warp-mempool-hold`. The 4th OOM happened with the hold active, so there is no evidence that it helps.
  It stays a pushed branch with no PR.
- **Stopgap, still open:** `--num-evals 12` avoids the eval but does not explain it. Some rollout on prec28's policy with seed
  14 at eval 15 asks Warp for contact scratch beyond the GPU's headroom, probably from a contact blow-up. Nothing tonight
  warm-starts from prec28.
- **prec28 is confounded:** 229M + 70.6M with a fresh optimizer. Treat it as indicative only.
- **collect-sync fixer (07:15):** `~/code/robot` was left on that branch. The fixer switched it back to main (it is now level with
  origin/main) and kept the uncommitted STL / `one_leg.py` edits.

**Merged / branched:** nothing. The open PRs (#254 beacon vbat, #247 heading hold, #232 walk-recovery tilt, #229 power rail,
#228 CAD, #206 console) are firmware, CAD and console work, with no stride-training content. Tonight runs on `BRANCH=main`.
Every recipe was parsed with main's `make_parser()`, and preflight passed 19/19.

**Students** (`grade.py` on main, measured sole, 8 seeds, SHAPE_HZ=8; falls/trips; `/tmp/plan1109/gradeA.md`,
`gradeC09.md`). **Control re-grade this morning, same code:** prec16, prec24 and prec20 reproduce x1.5 0/0 (`gradeP.md`), so
the losses below are real.

| student | variable vs parent | x1.5 | stride1 refit 0.6 / 0.4 / 0.3 falls | robot probes falls | directions (falls) | verdict |
|---|---|---|---|---|---|---|
| prec25_s2t | s2 0.0008 (prec16) | **8/8** | 0 / 0 / 0 | 0 | +0.83 −0.79 +0.37 −0.44 +104 −122 (0) | not held (margin) |
| prec26_sh35 | swing-height 0.035 (prec16) | **6/6** | 0 / 0 / 0 | 0 | +0.91 −0.95 +0.44 −0.52 +102 −101 (0) | clearance held, margin lost |
| prec27_ls2 | w-low-swing 2 (prec16) | 0/0 | 0 / 0 / 0 | 0 | +0.62 **−0.93 (3)** +0.44 −0.45 +107 −109 | not held |
| prec28_crouch_mirror* | mirror 1.0 (prec24) | **8/8**; 7/7 @0.9 | 0 / 0 / 0 | 0 | +0.77 −0.90 +0.50 −0.50 +96 −116 (0) | not held (confounded) |
| prec29_crouch_s1 | seed 15 (prec24 recipe) | **8/8**; 8/8 @0.9 | 0 / 0 / 0 | 0 | +0.94 −0.91 +0.48 −0.39 +104 **−82** (0) | half held |
| prec30_head03 | heading 0.3 (prec16) | **3/3** | 0 / 0 / 0 | 0 | +0.81 −0.89 +0.43 −0.49 +112 −98 (0) | not held |
| prec31_js30 | js 0.3 (prec16) | **6/8** | 0 / 0 / 0 | 0 | +0.91 −0.81 +0.46 −0.44 +101 −121 (0) | speed held, margin lost |
| prec32_crouch_s2t (teacher) | s2 0.0008 (prec24) | 6/6; **0/0 @0.9** | 1 / 0 / 0 | 0 | +1.00 **−0.92 (3)** +0.49 −0.45 +94 −88 | student tonight |

Every student WALKS. The only falls outside x1.5 are prec27's back (3/8) and the prec32 teacher's back (3/8) and stride1 0.6 (1/8). Every
slow-clock student still trips `pose` on walk→stop, the known closing-step pattern.

**Clearance and speed** (student scorecards; `speed.py` on steps5 0.3 robot, the same method as 11-08, so prec16 reads 83/68 again):

| student | median swing peak / ≥ 3 cm / lowest step | walk_steps median L / R | mean knee L/R | above 180 | knee range L/R | steps5 yaw / fwd yaw |
|---|---|---|---|---|---|---|
| prec16 | 1.2 cm / 4 % / 0.3 | 1.1 / 1.1 | 83 / 68 | 0.52 | 67 / 47 | +38 / +2 |
| prec25 (s2t) | 1.5 / 8 % / 0.3 | 1.5 / 1.2 | 80 / 73 | 0.41 | 64 / 50 | +18 / **−66** |
| **prec26 (sh35)** | **1.6 / 12 % / 0.4** | **1.8 / 1.8** | 80 / 76 | 0.43 | **62 / 53** | **+9 / −8** |
| prec27 (ls2) | 1.3 / 1 % / 0.4 | 1.2 / 1.3 | 77 / 69 | 0.37 | 59 / 48 | +35 / +40 |
| prec28 (crouch mirror) | 1.6 / 6 % / 0.3 | 2.1 / 1.4 | 64 / 80 | 0.40 | **53 / 55** | +30 / +2 |
| prec29 (crouch s1) | 1.3 / 3 % / 0.3 | 1.2 / 1.3 | 86 / 69 | 0.38 | 66 / 49 | +20 / −3 |
| prec30 (head 0.3) | 1.3 / 7 % / 0.3 | 1.1 / 1.2 | 79 / 79 | 0.47 | 62 / 55 | +28 / +1 |
| **prec31 (js 0.3)** | 1.3 / 3 % / 0.4 | 1.5 / 1.5 | **72 / 61** | **0.30** | 56 / 45 | +14 / −22 |

**Hypotheses:**
- s2 0.0008 (prec25): a tighter kernel moves the median 1.2 → 1.5 cm. **Not held**: below the 1.8 target, x1.5 lost, fwd yaw −66.
- Swing height 0.035 (prec26): **the best clearance and yaw so far** (median 1.6 cm, walk_steps 1.8/1.8, yaw +9/−8, knees
  more even). It still falls short of the ≥ 1.8 cm overall target and loses x1.5 (6/6). Of the levers tried, the reward
  target moves the shuffle most.
- w-low-swing 2 (prec27): **not held**. It does not move the lowest step (0.4/0.5 cm). It is the only run that keeps x1.5, and it adds back falls.
- Crouch + mirror (prec28): the knees even out (53/55) on the crouch policy too. It loses x1.5, but the resume confounds it.
- Seed of prec24 (prec29): 0 falls on every probe replicates; **the x1.5 margin does not** (8/8 at 1.0 and 0.9). prec16's
  recipe held x1.5 on 2 of 2 seeds (prec16, prec18), and prec24's crouch recipe on 1 of 2.
- Heading 0.3 (prec30): yaw +38 → +28 only, and x1.5 lost. **Not held.**
- js 0.3 (prec31): **the slowest gait yet** (time above 180 0.30; mean knee 72/61), but the knees are less even and x1.5 is 6/8.

**What the night says.** Every variable that moved the gait (clearance, speed, yaw, symmetry) cost the x1.5 stand margin. The
one that moved nothing (ls2) kept it. In the prec line, x1.5 is a fragile property of the particular draw: it is not trained, it is
inherited from prec16. The repo already has the fix in kind: `--loop-gain-range` trains on exactly the grade's x1.5 law (targets =
0.5 s mean + g × deviation). walk18_shp10lg (11-02) tried it on a policy that already passed x1.5 and was "not a candidate"
because its walking body rate rose to 1.2-1.3 rad/s. The slow gait walks at ~1.0, so it has that headroom. **Tonight
tests loop-gain DR as a margin repair** on three policies that lost x1.5 (prec26 clearance, prec29 crouch, prec31 speed),
with a control on prec16. It also stacks the clearance levers on prec26.

**Distil gap** (card seed-passes, teacher → student): prec25 32 → 30, prec26 23 → 26, prec27 28 → 40, prec28 22 → 34, prec29
20 → 39, prec30 32 → 34, prec31 28 → 32. The median is +4, so distillation is not the limit.

**Ranking (deploy view):** prec16 ≥ prec24 (both x1.5 0/0, 0 falls) > prec26 (best clearance and yaw, no margin). The deploy
candidate is unchanged: `deploy/prec16-f075` (not flashed). None of tonight's students is a candidate until it keeps x1.5.

**Tonight** (after the left-over prec32 distil and prec33 pair: 8 teachers + distils, `BRANCH=main`, 300M warm starts, ~44 min a
pair, ends ~04:40):

| # | run | parent | variable | hypothesis |
|---|---|---|---|---|
| 1 | prec34_f075_js20_sh35_lg | prec26 | `--loop-gain-range 1.0,1.6` | x1.5 back to 0/0 with prec26's clearance (median ≥ 1.6 cm) kept |
| 2 | prec35_f075_js20_lg | prec16 | `--loop-gain-range 1.0,1.6` | control: x1.5 stays 0/0, x2 improves, nothing lost |
| 3 | prec36_f075_js20_sh35_s1 | prec16 | seed 15 (prec26 recipe) | prec26's clearance and yaw are the recipe |
| 4 | prec37_f075_js20_sh35_s2t | prec26 | `--feet-phase-s2 0.0008` | the clearance levers stack: median ≥ 2.0 cm |
| 5 | prec38_f075_js20_crouch_sh35 | prec24 | `--swing-height 0.035` | the bent-knee gait lifts like prec26 |
| 6 | prec39_f075_js20_sh45 | prec26 | `--swing-height` 0.035 → 0.045 | the swing-height trend continues: median ≥ 2.0 cm, mean knee < 90 |
| 7 | prec40_f075_js20_crouch_lg | prec29 | `--loop-gain-range 1.0,1.6` | repair 2: x1.5 0/0 at crouch 0.9 and 1.0 |
| 8 | prec41_f075_js30_lg | prec31 | `--loop-gain-range 1.0,1.6` | repair 3: x1.5 0/0 with time above 180 ≤ 0.33 |

Tomorrow: rank by 0 falls on the robot and direction probes plus **x1.5** first, then clearance, then speed, symmetry and yaw.
If loop-gain DR repairs x1.5 in 2 of 3 cases, it joins every prec recipe, and the clearance and speed levers can be stacked
under it.

**Fallback:** back to the prec16 recipe (x1.5 held on 2 of 2 seeds), warm-starting from prec16. The prec24 recipe is saved as
`fallback.job.prec24-20261109`.

### Night of 2026-11-09 (evaluated 2026-11-10)

**Weight transfer, night 1.** Stride only; get-up stays parked. Tom has not commented on #137 since 11-04, so the
3 cm question is still open.

**Hardware 11-09 (interactive session, `hw_sessions/2026-11-09/`).** prec16_f075_js20 on the robot (`deploy/prec16-f075`,
`stride1_02s`). Its joints were much slower than omni14's: mean knee 115/121 deg/s (omni14 171/131). But **the right foot never
left the floor**. The R knee bent 45 deg and the hip 39 deg, while the hip roll moved only 4–8 deg. There was no lateral
weight shift, so the loaded bending leg yawed the body (−7.2 rad/s, ending 46 deg right) and the rate guard tripped
(3.02). **prec16 is no longer the deploy candidate.** The interactive session put the walk-shift reward (branch
`train/walk-shift`) at the head of the queue (prec42–45). It also set the new bar: both feet lift ≥ 2 cm, and the CoM sits
≥ ~0.6 of the way toward the stance foot while the other foot is up (`shift_when_up`, from the job-tmp `grade_recover.py`).

**The runner** ran 22:00–05:17 on main (`28536e6` → `f8516de`) and `train/walk-shift` (`4ae688f`). Every job logged
"origin tip, verified", and the collect log has no STALE REFEREE. It ran the left-over prec32 distil and the prec33 pair, the
four walk-shift pairs (prec42–45), the prec34–37 pairs and the prec38 teacher. The prec38 distil and the prec39–41 pairs
did not start before 05:00, so they stay queued for tonight. **No fixer sessions:** `fixer.log` has nothing after 11-09
02:22. There was no OOM, so there is still no evidence for or against `fix/warp-mempool-hold`, and the `--num-evals 12`
stopgap stays unexplained (nothing warm-starts from prec28).

**Merged / branched:** `train/walk-shift` (4ae688f, opt-in `--w-walk-shift` / `--low-swing-clear`, tests green) and this
morning's `train/mimic-sway` (opt-in `--mimic-sway`, below) are merged to main together (PR linked on #137), so tomorrow's
referee and grade.py on main know both. The other open PRs (#254, #229, #206, #71) are firmware, console and CAD work.
`fix/warp-mempool-hold` stays an unmerged branch (no evidence).

**Students** (grade.py on main through `grade_recover.py`, measured sole, 8 seeds, SHAPE_HZ=8; falls/trips;
`/tmp/plan1110/gradeA.{md,json}`, summary `summ.py`). The control re-grade reproduces: prec16 x1.5 is 0/0, and prec26's row
matches 11-09's exactly (x1.5 6/6; directions +0.91 −0.95 +0.44 −0.52 +102 −101). Every student WALKS. Every student fails
x2 8/8, and every student is 0/0 on push 0.4, on all the robot probes and on walk 0.3 → stop.

| student | variable vs parent | x1.5 | stride1 refit 0.6 / 0.4 / 0.3 falls | directions fwd back sideL sideR turnL turnR (falls) | verdict |
|---|---|---|---|---|---|
| prec42_ws1 | walk-shift 1 + low-swing 10 @ 2 cm (prec16) | **2/2** | 0 / 0 / 0 | +0.81 **−0.27** +0.49 −0.48 +123 −99 | not held |
| prec43_ws2 | walk-shift 2 + low-swing 10 @ 2 cm (prec16) | 0/0 | 0 / 0 / 0 | +0.87 −0.82 +0.36 −0.46 +115 −121 | best shift of the line, far from the bar |
| prec44_sh35_ws1 | walk-shift 1 + low-swing 10 (prec26) | 0/0 | 0 / 0 / 0 | +0.90 −0.91 +0.40 −0.50 +124 −116 | shift not held |
| prec45_ws1_only | walk-shift 1 alone (prec16) | **1/1** | 0 / 0 / 0 | +0.92 −0.90 +0.48 −0.43 +105 −129 | not held |
| prec34_sh35_lg | loop-gain 1.0–1.6 (prec26) | **0/0** (was 6/6) | 0 / 0 / 0 | +0.73 −0.71 +0.33 −0.26 **+52** −94 | margin repaired, clearance and turning lost |
| prec35_lg | loop-gain 1.0–1.6 (prec16, control) | 0/0 | 0 / 0 / 0 | +0.70 −0.64 +0.38 **−0.20 +62 −77** | not held: x2 not better, directions slower |
| prec36_sh35_s1 | seed 15 (prec26 recipe) | 0/0 | 0 / 0 / 0 | +0.67 −1.01 **(2)** +0.38 −0.49 +110 −111 | prec26 not replicated |
| **prec37_sh35_s2t** | s2 0.0008 (prec26) | **0/0** | 0 / 0 / 0 | +0.91 −0.96 +0.42 −0.49 +119 −116 | best lift, margin kept |
| prec32_crouch_s2t | s2 0.0008 (prec24) | 0/0 | **3** / 0 / 0 | +0.83 −0.94 **(2)** +0.60 −0.46 +101 −104 | not held |
| prec33_mirror_s1 | seed 15 (prec20 recipe) | 0/0 | 0 / 0 / 0 | +0.97 −0.90 +0.46 −0.49 +135 −121 | held: symmetry replicates |

**Weight transfer** (the new bar). `stride1 0.3 robot` is the probe closest to the 11-09 hardware run, and `steps5 0.3 robot`
is five steps. Lift is the mean peak per foot (cm, worst seed in brackets). Shift is `shift_when_up` L-up / R-up: the CoM
fraction toward the stance foot, −1 = over the R foot, +1 = over the L foot, and the bar is |shift| ≥ 0.6 on both sides.

| student | stride1 robot lift L / R | stride1 robot shift | steps5 robot shift | knee mean L/R (stride1) | time > 180 | knee range L/R (steps5) |
|---|---|---|---|---|---|---|
| prec16 (control) | 3.1 / 2.1 (1.9) | −0.21 / +0.16 | −0.39 / +0.50 | 73 / 62 | 0.42 | 67 / 48 |
| prec42_ws1 | 2.9 / 1.6 (1.5) | −0.30 / +0.27 | −0.49 / +0.49 | 75 / 49 | 0.34 | |
| prec43_ws2 | 3.4 / 2.3 (2.1) | **−0.35** / +0.24 | −0.53 / **+0.71** | 72 / 59 | 0.36 | |
| prec44_sh35_ws1 | 3.7 / 1.8 (1.7) | −0.38 / +0.23 | −0.52 / +0.63 | 86 / 52 | 0.38 | |
| prec45_ws1_only | 2.8 / 2.0 (2.0) | −0.33 / +0.26 | −0.52 / +0.51 | 72 / 56 | 0.29 | |
| prec34_sh35_lg | 2.7 / 1.5 (1.4) | −0.30 / +0.13 | −0.41 / +0.21 | 62 / 44 | **0.21** | 45 / 41 |
| prec35_lg | 3.0 / 1.3 (1.2) | −0.33 / +0.12 | −0.39 / +0.35 | 70 / 60 | 0.33 | 62 / 42 |
| prec36_sh35_s1 | 3.8 / 2.4 (2.3) | −0.29 / +0.28 | −0.41 / +0.54 | 83 / 51 | 0.39 | |
| **prec37_sh35_s2t** | **4.4 / 3.0 (2.8)** | −0.30 / +0.29 | −0.44 / +0.60 | 58 / 76 | 0.30 | 60 / 51 |
| prec32_crouch_s2t | 3.0 / 2.1 (1.9) | −0.28 / +0.23 | −0.42 / +0.72 | 72 / 60 | 0.36 | |
| prec33_mirror_s1 | 2.9 / 1.8 (1.6) | −0.27 / +0.19 | −0.44 / +0.49 | 56 / 71 | 0.29 | **56 / 51** |

**Hypotheses:**
- **Walk-shift (prec42–45): not held.** The best result is prec43 (walk-shift 2): stride1 shift 0.35/0.24 against the bar's 0.6.
  The training metric shows why. The swing-weighted CoM-to-stance-sole distance (`walk_shift_err / walk_swing_w`) went only
  7.0 → 6.7 cm over 300M steps in all four runs. At d ≈ 6.7 cm a σ = 3 cm kernel pays exp(−5) ≈ 0.007, so it has no gradient.
  And **the imitation reference is against it.** `_mimic_ref` sets the hip and ankle rolls to the standing pose for any forward
  command (roll = default + B·xn, and B is the lateral command), and `w_mimic` 1.5 pays exp(−Σdq²/0.72). A 6 deg sway (hip and ankle
  rolls, both legs) costs ~6 % of that term, ≈ 0.09 a tick, about 20× the ≈ 0.005 a tick the walk-shift kernel pays. The reference is the problem, not the RL.
- **Loop-gain DR (prec34, prec35): margin yes, gait no.** It repaired prec26's x1.5 (6/6 → 0/0), but turning and side R
  slowed on both runs (prec35 turn +62/−77 against prec16's +114/−106), prec34's card clearance fell 1.6 → 0.9 cm, and x2
  did not improve. It is a margin tool that costs directions, so it stays off the recipes until the weight transfer is solved.
- **prec26 replicate (prec36): its clearance and yaw were the draw.** Card median 1.4 (prec26 1.6), steps5 yaw +41 (prec26
  +9), and 2/8 back falls. Its x1.5 0/0 says prec26's x1.5 loss was also the draw.
- **Stacking s2 0.0008 on prec26 (prec37): partly held.** The card median is 1.6 (target 2.0), but it has the best lift on the
  robot probe (4.4/3.0 cm, worst seed 2.8), x1.5 0/0, 0 falls, yaw −2 and the slowest knees. On every criterion before
  11-09 it is the best student, but its shift (0.30/0.29) fails the new bar.
- **prec33 (prec20 recipe, seed 15): held.** Knee range 56/51 (prec20 57/54) replicates the mirror symmetry. It has 0
  stride1 falls and fwd yaw −11, so prec20's falls and yaw were the draw.
- prec32 (crouch + s2t): 3/8 stride1 0.6 falls and 2/8 back falls. **Not held.** prec38's student trains tonight.

**Distil gap** (card seed-passes, teacher → student): prec32 24 → 26, prec33 30 → 36, prec34 19 → 27, prec35 19 → 22,
prec36 27 → 31, prec37 26 → 27, prec42 24 → 26, prec43 24 → 28, prec44 25 → 26, prec45 28 → 36. The median is +3.5;
distillation is not the limit.

**Ranking (deploy view):** nothing meets the weight-transfer bar, so **no robot candidate**. prec16 failed on the robot.
By the old criteria, prec37 is first (lift, margin, yaw, speed). prec43 has the most shift with its margin kept.

**Change of plan: the weight shift goes into the reference.** New opt-in `--mimic-sway M` and `--mimic-sway-lead RAD` (branch
`train/mimic-sway` = `train/walk-shift` + one commit, `tests/test_mimic_sway.py`). While a moving command is active, the
reference moves the torso M metres toward the stance foot. It does this with the hip rolls, the ankle rolls cancel them so the
soles stay level, and the sway leads the swing clock by 0.785 rad by default. The rad/m comes from MuJoCo kinematics
(`walker_env.mimic_sway_coefs`, −3.05 rad/m on the meas-sole plant, so 0.035 m is 6.0 deg and 0.05 m is 8.7 deg), and the CPU
and MJX references agree to < 1e-4. The size comes from the plant: the feet are ±6.05 cm apart, the CoM is 0.255 m high
(ω0 = 6.3 rad/s), and the gait is 0.75 Hz (4.7 rad/s). Putting the ZMP on the stance foot centre then needs ~3.9 cm of CoM
sway, so tonight tests 0.035 and 0.05.

**Tonight** (the prec38 distil first, then 6 sway pairs on `train/mimic-sway`, then the 3 left-over 11-09 pairs on main,
renamed `1110220011-16` so that they run last. 300M warm starts, ~44 min a pair, last start ~04:00):

| # | run | parent | variable | hypothesis |
|---|---|---|---|---|
| 1 | prec46_sw35 | prec16 | `--mimic-sway 0.035` | stride1 robot shift 0.21/0.16 → ≥ 0.5 both sides, R lift ≥ 2 cm, x1.5 and directions kept |
| 2 | prec47_sw50 | prec16 | `--mimic-sway 0.05` | a bigger sway reaches ≥ 0.6; risk: side/turn or x1.5 |
| 3 | prec48_s2t_sw35 | prec37 | `--mimic-sway 0.035` | prec37's lift (≥ 2.8 cm both feet) kept, with the shift |
| 4 | prec49_ws2_sw35 | prec43 | `--mimic-sway 0.035` | with the reference swaying, walk-shift's 3 cm kernel gets gradient: ≥ 0.6 |
| 5 | prec50_sw35_s1 | prec16 | seed 15 (prec46 recipe) | prec46 replicates |
| 6 | prec51_sw35_lead0 | prec16 | `--mimic-sway-lead 0` (prec46 recipe) | lead 0.785 beats 0: with no lead the CoM is between the feet at liftoff |
| 7–9 | prec39_sh45, prec40_crouch_lg, prec41_js30_lg | (11-09 plan) | swing height 0.045; loop-gain on prec29 / prec31 | as planned 11-09; they come after the weight transfer |

Tomorrow: grade every student with `grade_recover.py` and rank by the weight-transfer bar first. That means stride1 0.3
robot and steps5 0.3 robot: both feet ≥ 2 cm and |shift| ≥ 0.6 on both sides, with 0-fall directions and x1.5. Clearance,
speed and yaw come after that. If the sway works, the next step is a hardware candidate for Tom, plus the walk-shift kernel
at a wider σ (6 cm) to sharpen it.

**Fallback:** the prec46 recipe (prec16 + `--mimic-sway 0.035`, warm start prec16). The previous prec16 fallback is saved as
`fallback.job.prec16-20261110`.

(A slip this morning: for about a minute at ~08:20 I switched the shared `~/code/robot` checkout to a branch off
`train/walk-shift`, then switched it straight back to main. The grade's worker pool had already started on main. The prec16
and prec26 control rows reproduce 11-09 exactly.)

### Night of 2026-11-10 (evaluated 2026-11-11)

**Weight transfer, night 2; march, night 1.** Stride only; get-up stays parked. Planned under the 2026-11-10 project review's
rules (planner prompt on main). Note: the wrapper fed this morning's session a stale copy of the prompt
(`robot-mjx/night/planner_prompt.md`, 11-09, without the review rules). I followed main's version and copied it over the night
copy (backup `planner_prompt.md.bak-20261111`). That copy is untracked, so every prompt change on main has to be copied by hand.

**The runner.** Day run 11-10 12:06–14:11 (prec38 distil, prec49/46/48 teachers), then from 18:18 (`day_run.sh`, the 22:00 cron
found the lock) to 03:01: prec47–60, 15 teacher + distil pairs in all. Every job ran on main and logged "origin tip, verified".
There was one transient DNS failure at 21:14, and the retry passed. The collect log has no STALE REFEREE. **Fixers:** one, at
11-10 12:13. `train/mimic-sway` had been merged and deleted, so the fetch failed; the fixer restored the branch and repointed the
sway jobs to `BRANCH=main`. That fix is complete: the branch is in main, and tonight's jobs are on main. prec39–41 are held
(`queue/held/WHY-1110-prec39-41.txt`, the interactive session).

**Merged / branched:** nothing this morning. Open PRs (#254, #229, #206, #71) are firmware, console and CAD. The newest local branches
(`fix/281-benched-hold-result`, `deploy/sway-candidate`, `tools/sway-probe-refit`) belong to the interactive session and do not
touch training. Since the last plan, main gained `--roll-series-k` (PR #280), round 3 of the #259 gate (PR #278, grade.py's opt-in
`GRADE_PLANT_TERMS`) and the scripted-run tooling (#276). **This changes comparisons:** students with the flag train on a softer
roll plant than everything before them.

**Grades** (grade.py on main, measured sole, 8 seeds, SHAPE_HZ=8, 22 probes; `/tmp/plan1111/gradeA.{md,json}`; plant-terms
re-grade `/tmp/plan1111/gradeB.*` with `GRADE_PLANT_TERMS="torso_kg=0.4855,pack=0.083,zoff_La=0,ser_ankle=17.75,ser_hip=11.0"`;
summaries `summA.md` / `summB.md`). The control re-grade reproduces 11-10 (prec16 x1.5 0/0, prec37 lift 4.4/3.0 cm, directions
unchanged). Every student WALKS, is 0/0 on push 0.4 and walk 0.3 → stop, has 0 falls on all the robot probes, and fails x2 8/8.

| student | x1.5 | stride1 refit .6/.4/.3/.4μ.4 falls | directions fwd back sideL sideR turnL turnR | stride1 robot lift L/R (min) | stride1 robot shift | steps5 robot shift | march5 robot |
|---|---|---|---|---|---|---|---|
| prec16 (control) | 0/0 | 0/0/0/0 | +0.92 −0.86 +0.47 −0.49 +114 −106 | 3.1/2.1 (1.9) | −0.21/+0.16 | −0.39/+0.50 | stands (never trained) |
| prec46_sw35 | 0/0 | 0/0/0/0 | +0.92 −0.78 +0.41 −0.47 +120 −122 | 3.0/1.9 (1.8) | −0.27/+0.28 | −0.35/+0.57 | stands |
| prec47_sw50 | 0/0 | 0/0/0/0 | +0.91 −0.86 +0.34 −0.45 +97 −126 | 2.8/1.9 (1.7) | −0.24/+0.20 | −0.35/+0.54 | stands |
| prec48_s2t_sw35 | **8/8** | 0/0/0/0 | +0.87 −0.91 +0.42 −0.48 +138 −96 | **4.9/2.6 (2.5)** | −0.33/+0.34 | −0.44/+0.66 | stands |
| prec49_ws2_sw35 | 0/0 | 0/0/0/0 | +0.86 −0.84 **+0.01** −0.41 +139 −102 | 3.9/2.1 (2.0) | **−0.39/+0.37** | −0.57/+0.58 | stands |
| prec50_sw35_s1 | 0/0 | 0/0/0/0 | +0.84 −0.73 +0.36 −0.48 +95 −121 | 3.5/1.4 (1.3) | −0.26/+0.23 | −0.42/+0.51 | stands |
| prec51_sw35_lead0 | 0/0 | 0/0/0/0 | +0.86 −0.78 +0.49 −0.48 +118 −130 | 3.0/1.7 (1.6) | −0.29/+0.28 | −0.44/+0.37 | stands |
| prec52_march | 0/0 | 0/0/0/0 | +0.86 −0.93 +0.42 −0.42 +121 −118 | 2.9/2.3 (2.1) | −0.27/+0.32 | −0.47/+0.55 | **R only**: 6.3 cm every R step, L ≤ 0.75; shift R-up +0.62 |
| prec53_march_noref | 1/1 | 0/0/0/0 | +0.85 −0.84 +0.44 −0.46 +113 −91 | 3.2/1.6 (1.4) | −0.26/+0.23 | −0.40/+0.49 | no lift |
| prec54_march6 | 0/0 | 0/0/0/0 | +0.77 −0.79 +0.43 −0.43 +92 −112 | 3.7/1.9 (1.8) | −0.23/+0.21 | −0.38/+0.51 | no lift |
| prec55_march_s1 | 6/6 | 0/0/0/0 | +0.84 −0.83 +0.46 −0.45 +131 **−66** | 3.4/1.9 (1.8) | −0.32/+0.16 | −0.44/+0.66 | **L only**: 8.6 cm, shift −0.64 |
| prec56_march_s2 | 1/3 | 0/0/0/0 | +0.91 −0.85 +0.41 −0.37 +98 −109 | 3.3/1.6 (1.5) | −0.27/+0.17 | −0.49/+0.36 | **L only**: 5.2 cm, shift −0.70 |
| prec57_march_rs | 8/8 | 0/0/0/0 | +0.84 −0.80 +0.38 −0.40 +114 −110 | 2.7/2.2 (2.0) | −0.26/+0.21 | −0.45/+0.58 | no lift |
| prec58_sw35_rs | 0/0 | **4**/0/0/0 | +0.86 −0.80 +0.39 −0.44 +132 −115 | 2.7/2.1 (1.9) | −0.26/+0.14 | −0.47/+0.37 | stands |
| prec59_march_rs_s1 | 0/0 | **5**/0/0/0 | +0.64 −0.78 **+0.02** −0.36 +118 −107 | 3.6/2.2 (2.1) | −0.27/+0.20 | −0.40/+0.69 | **L only**: 6.3 cm, shift −0.76 |
| prec60_march_rs_s2 | 8/8 | **7**/0/0/0 | +0.82 −0.72 **+0.02** −0.47 +112 −73 | 3.3/1.9 (1.8) | −0.29/+0.28 | −0.42/+0.58 | **R only**: 5.3 cm, shift +0.53 |

`march5 robot` and `march5 refit` agree on every student. The stride1 robot torso tilt maximum (whole episode, not at +0.5 s) is
9.7–15.9 deg (the robot: 11.3 at +0.5 s, 14.4 at +0.6 s). On the plant-terms plant every shift grows (prec16 stride1 −0.60/+0.48,
steps5 0.7–1.3 for all students) and R lifts drop by ~0.5 cm. The one-sided marches stay one-sided. Under #259 these shifts are a
ranking aid, not evidence.

**Hypotheses:**
- **Sway in the walking reference (prec46–51, 58): not held, on 3 seeds, 2 sizes and 2 leads.** stride1 robot shift is 0.20–0.39
  against prec16's 0.16–0.21, and the bar is 0.6. A bigger sway (prec47) or no lead (prec51) changes nothing. Stacked on prec37
  (prec48) it keeps the lift and loses x1.5; stacked on walk-shift 2 (prec49) it gives the most shift and loses side L.
  The line stops here. Walking shift has to come from something other than a bigger reference.
- **The march (prec52–57, 59, 60): the reference finds a march, but only on one leg.** 5 of the 6 reference-march seeds learned to
  lift a foot high (5–9 cm, well over 3 cm) on every one of that foot's steps, with the CoM over the stance foot (|shift|
  0.53–0.76). The other foot stays down (≤ 0.8 cm). Which side is set by the seed: R for prec52/60, L for prec55/56/59. No seed lifts
  both. The training curves show a step change at 200–250M steps (`lift_ok` ~1 → 12–14) and then a plateau. **Why:** every lift
  term (`w_lift`, `w_knee_high`, `w_com_stance`) is gated by the commanded foot's clearance, so the side that has never lifted gets no
  gradient: the walk-shift lesson again. Without the reference (prec53) or with c4 alone (prec54) no seed found the march at all. The
  hypotheses: A **half held** (one leg), B **held** (the reference is what finds the march), C **not held**.
- **The roll-series flag (52/55/56 against 57/59/60, three seeds each), graded on both plants:** the march was found 3/3 without it
  and 2/3 with it. x1.5 was clean on 1/3 each way. With the flag, 2 of 3 seeds **lost side L** (dy +0.02) and fell on stride1 0.6
  refit (5/8, 7/8); without it, 0 falls and side L 0.41–0.46. On these grade plants the flag shows a cost and no visible gain. It
  stays in every recipe (it is part of the plant: review rule, #259), but Tom should know (issue comment).
- **Distil gap** (card seed-passes teacher → student): 46 23→39, 47 21→23, 48 28→32, 49 36→42, 50 22→28, 51 30→34, 52 22→23,
  53 23→27, 54 27→30, 55 20→29, 56 23→28, 57 26→23, 58 32→32, 59 21→23, 60 20→22. The median is +4; distillation is not the limit.

**Ranking:** no student meets the march bar (both feet ≥ 3 cm on every step, |shift| ≥ 0.6 both sides). Nothing is a hardware
candidate: the #259 gate has not passed, and every march is one-sided. For the walk, prec37 is still first on the old criteria.

**Stop and escalate (rule 4).** The weight-transfer family has no hardware-confirmed gain. It ran walk-shift (11-09) and
sway / march (11-10), and tonight is its **third night**. Tomorrow's planner adds no new variables to it. It writes up the
replicates and asks Tom, unless a robot number arrives first. The walking-sway sub-line is stopped now (above).

**Tonight: three arms × three seeds, all with `--roll-series-k hip=10:25,ankle=16:40`, 300M, BRANCH=main, each with its distil.**
The order is A, B, C, nine pairs, ~44 min a pair, last start ~04:00.

| arm | runs (seeds 14/15/16) | parent | variable | hypothesis |
|---|---|---|---|---|
| A | prec61_mrs_mir, _s1, _s2 | prec59_march_rs_s1 (marches on L only) | `--w-mirror-loss 1.0` | the mirror of an L lift is an R lift: march5 robot lifts both feet ≥ 3 cm on every step in ≥ 2 of 3 seeds, 0 falls; walking no worse than prec59 |
| B | prec64_march_rs_mir, _s1, _s2 | prec16_f075_js20 (prec57 recipe) | `--w-mirror-loss 1.0` | the march is found on both feet in 3 of 3 seeds (control: prec57/59/60, 0 of 3 two-sided) |
| C | prec67_mrs_cont, _s1, _s2 | prec59_march_rs_s1 | none: +300M on prec59's recipe | control for A: more training alone does not bring the second foot up |

The mirror loss is the tool that held symmetry on two seeds (prec20, prec33). I checked its maps on this recipe's env on the CPU:
obs 165, act 12, the permutation is an involution, and c4 (lift) and the gait phase flip as they should. Preflight passed 18/18.
**Fallback:** arm A's recipe (`fallback.job`; the prec46 sway fallback is saved as `fallback.job.prec46-20261111`).

Tomorrow: grade on `march5 robot` first (`march_lifts_pass`, per-foot minimum step, shift both sides), then the walking probes, on
both plants, and write the spread over the three seeds.

### Night of 2026-11-11 (evaluated 2026-11-12)

**March on the play plant: night 4 of the weight-transfer line.** Stride only; get-up stays parked. The 11-10 plan (spring-plant
mirror arms prec61/64/67) never ran: the 11-11 hardware session refitted the roll chain to per-side play beyond the encoders
(gate README Round 5, PR #290) and replaced them with four play-plant arms (prec68–71, `queue/held/WHY-1111-spring-march-replaced.txt`).

**The runner.** `day_run.sh` from 19:12 (the robot was off for the night; the 22:00 cron found the lock) to 03:39: 10 teachers +
10 distils, every job on main with origin verified (SYNC=ok), no STALE REFEREE. **Fixers: two**, both the same crash: prec71_mir_pp
(seed 14) and prec71_mir_pp_s1 (seed 15) died with a Warp out-of-memory (`convex_narrowphase`, 99 MB) in the 16th eval at
229M steps, the signature of prec28 (11-09, 4 attempts). Each fixer reran the job with `--num-evals 12`, which finished. So the
crash is not seed 14: all three hits are `--w-mirror-loss` runs that reach eval 16. **Finished properly as far as the day allows:**
preflight now refuses a mirror-loss job with `--num-evals` ≥ 16 (branch `night/planner-1112`, merged; installed in
`robot-mjx/night/`). The root cause needs a GPU repro (eval 16 of a mirror-loss run, Warp contact buffers), which the planner
may not run by day: it is open.

**Merged / branched:** this morning's branch `night/planner-1112` (grade.py's level-body metric, the preflight check, this entry).
Open PRs #293 / #292 (stepper rounds, the 11-11 record), #282 (sway probe rehearsal, #260), #229, #206, #71 do not touch training.
Since the last plan main gained `--roll-play-deg` per side (#286, #290), the stepper (#289) and the pick-up autohome (#287).
**This changes comparisons:** prec68–71 train on the play plant; everything before them on lash or spring.

**New in grade.py (march5):** `march_roll_{l,r}_deg` / `_max`, the torso roll while only that foot is > 1 cm up, signed toward the
lifted foot (+ = sagging toward the swing foot, − = leaning over the stance foot). The review asks for lifts *with a level body*.

**Grades** (grade.py from the planner worktree = main + the roll metric, measured sole, 8 seeds, SHAPE_HZ=8, 22 probes;
`/tmp/plan1112/gradeA.*`; play plant `gradeB.*` with `GRADE_PLANT_TERMS=play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0`;
tables `python3 /tmp/plan1112/summ.py <json>`). Controls prec16, prec59 (the parent), prec52 reproduce 11-11 on the grade plant.
Every student WALKS, has 0/0 on push 0.4 and walk 0.3 → stop and 0 falls on every robot probe, and falls x2 8/8.

| student | x1.5 grade / play | stride1 0.6 refit falls | fwd / back / side L / side R / turn L / R (grade plant) | side L / R (play) | march5 robot (both plants): lifted foot, min lift cm, shift, roll toward lifted (mean / max deg) | march5 trips |
|---|---|---|---|---|---|---|
| prec16 (control) | 0/0 / **8/8** | 0 | +0.92 −0.86 +0.47 −0.49 +114 −106 | +0.40 / −0.31 | stands | 0 |
| prec59_march_rs_s1 (parent) | 0/0 / 0/0 | 5 | +0.64 −0.78 **+0.02** −0.36 +118 −107 | +0.01 / −0.29 | L, 6.3, −0.76, −7.4 / 12.4 | 8 |
| prec52_march (control) | 0/0 / **8/8** | 0 | +0.86 −0.93 +0.42 −0.42 +121 −118 | +0.30 / −0.28 | R, 6.2, +0.62, −6.5 / 9.9 | 0 |
| P1 prec68_march_pp | 0/0 / 0/0 | 0 | +0.74 −0.75(1) +0.02 −0.04 +104 −102 | +0.01 / −0.04 | L, 5.4, −0.71, −5.8 / 8.2 | 8 |
| P1 prec68_march_pp_s1 | 5/5 / 5/5 | 8 | +0.66 −0.68 +0.04 −0.10 +98 −126 | +0.04 / −0.10 | L, 4.9, −0.64, −4.8 / 7.1 | 8 |
| P2 prec69_slow_pp | 2/2 / 4/4 | 8 | +0.65 −0.60(1) +0.01 −0.04 +106 −90 | +0.01 / −0.04 | L, 4.9, −0.69, −4.7 / 7.6 | 8 |
| P2 prec69_slow_pp_s1 | 0/0 / 0/0 | 0 | +0.46 −0.58 +0.01 −0.05 +82 −86 | +0.01 / −0.05 | L, 4.6, −0.67, −5.6 / 7.1 | 8 |
| P2 prec69_slow_pp_s2 | 0/0 / 0/0 | 0 | +0.59 −0.58 −0.00 −0.11 +91 −81 | −0.00 / −0.11 | L, 4.8, −0.70, −4.7 / 7.4 | 8 |
| P3 prec70_slowlift_pp | 0/0 / 0/0 | 1 | +0.59 −0.59 +0.02 −0.06 +86 −114 | +0.02 / −0.07 | L, 6.4, −0.69, −4.9 / 9.0 | 8 |
| P3 prec70_slowlift_pp_s1 | 0/0 / 0/0 | 8 | +0.61 −0.50 −0.00 −0.06 +112 −101 | −0.01 / −0.06 | L, 6.2, −0.76, −5.0 / 7.3 | 8 |
| P3 prec70_slowlift_pp_s2 | 0/0 / 0/0 | 0 | +0.53 −0.56 −0.01 −0.06 +89 −101 | −0.02 / −0.06 | L, 5.8, −0.64, −5.0 / 7.9 | 8 |
| P4 prec71_mir_pp | 0/0 / 0/0 | 0 | +0.86 −0.83 +0.02 −0.03 +104 −134 | +0.02 / −0.03 | **R**, 6.4, +0.61, −6.2 / 7.7 | 0 |
| P4 prec71_mir_pp_s1 | 0/0 / 0/0 | 6 | +0.73 −0.81 +0.01 −0.08 +91 −120 | +0.01 / −0.08 | **R**, 5.0, +0.56, −4.5 / 5.8 | 0 |

On stride1 0.3 robot the play students lift 2.1–3.9 cm (L) / 1.3–2.4 (R), shift −0.24 to −0.43 / +0.14 to +0.34, about prec16's
(−0.19/+0.18 on the play plant); steps5 robot reaches −0.66/+0.60 at best (prec70_s1). The two plants agree to ~0.1 on every
march number; the play plant's large effect is on the OLD policies' x1.5 margin (prec16, prec52: 0/0 → 8/8 falls).

**Hypotheses:**
- **P1, the play plant under the march: not held (2/2 seeds).** The march survives (L only, as the warm start), but no seed lifts R,
  and the body is not level: in single support the torso leans 4.8–5.9 deg over the stance foot (max 7–9). The robot, with the
  stepper's stance-hip trim, comes level. **Cost:** both sidesteps are gone (|dy| ≤ 0.10, prec16 0.47/0.49), on both plants.
- **P2, the slow clock (0.5–0.55 Hz, sway 5 cm): not held (3/3).** L only, lifts no higher (4.6–5.0), the same lean; forward
  speed drops (0.46–0.65 vs 0.74).
- **P3, more commanded lift: not held (3/3).** The L lift grows (5.8–6.6 cm); R still never leaves the floor (≤ 0.1 cm). More
  target on the side that lifts does nothing for the side that does not: the clearance-gated rewards again (11-11 finding).
- **P4, the mirror loss: not held, and a new fact (2/2).** Both seeds **switched** the marching side from the parent's L to R
  (6.4 / 5.0 cm), L down. The loss makes the policy symmetric in the sense that L and R are interchangeable, not that both happen.
  It is also the only arm with 0 flail-guard trips on march5 (P1–P3 trip 8/8 seeds, no falls).
- **The sidestep loss is the night's main regression**, 10 of 10 students on both plants. The parent prec59 had already lost side L;
  its children lose side R too. Whether the play plant or the march warm start takes it is untested: tonight's arm W answers it.
- **Distil gap** (card seed-passes teacher → student): 68 29→31, 68_s1 23→21, 69 18→22, 69_s1 20→21, 69_s2 18→21, 70 20→19,
  70_s1 21→21, 70_s2 19→18, 71 26→35, 71_s1 35→26. Median +1: not the limit.

**Ranking:** no march candidate (no seed lifts both feet; none is level within 3 deg). No walk candidate either: every play student
lost the sidesteps, and prec16 / prec37 fall x1.5 8/8 on the play plant. The #259 gate has not passed. **No robot candidate.**

**Stop and escalate (review rule 4).** The weight-transfer / march line has now run four nights (walk-shift 11-09, sway + march 11-10,
the play-plant march 11-11) with no hardware-confirmed gain. Across 18 march seeds on three plants it has shown: the reference finds
a march (no reference, no march); every march is one-legged; the seed or the warm start picks the side; the mirror loss swaps the side;
more lift, a slower clock and a sway do not bring up the second foot. It has not shown: a two-sided march, a level body in single
support, or any robot run that moved as the sim said. The structural cause (every lift reward is paid only once the commanded foot is
already up) suggests a reward change, but that is a new variable, and rule 4 says ask first. **Asked Tom on #137** (rule 5).
The hardware side meanwhile walks with the host-side stepper (11-11: 9-10 step walks, turning); "hardware leads, training follows".

**Tonight (2026-11-12): two replicates and one plant arm, 5 teacher + distil pairs, BRANCH=main, ~3.7 h.** No new march variable.

| arm | runs | parent | variable | hypothesis |
|---|---|---|---|---|
| W | prec72_walk_pp, _s1, _s2 (seeds 14/15/16) | prec16_f075_js20 | the roll chain only: `--roll-play-deg` per side for `--roll-lash-deg 0.5,2.0`, no march | six directions kept (sidesteps ≥ 0.3) and x1.5 0/0 back on the play grade plant (prec16 8/8 there): the march, not the plant, took the sidesteps |
| P1 | prec68_march_pp_s2 (seed 16) | prec59_march_rs_s1 | replicate | L only again |
| P4 | prec71_mir_pp_s2 (seed 16, `--num-evals 12`) | prec59_march_rs_s1 | replicate | the mirror loss flips the side again (R only) |

Arm W is not a variable of the march line: it is the plant update the deploy-first rules ask for before more reward tuning, on
the walk line's reference policy. Preflight passed 10/10. **Fallback:** arm W's recipe (`fallback.job`; the spring mirror-march
fallback is saved as `fallback.job.prec61-20261112`).

Tomorrow: arm W on the six directions and x1.5, both plants, against prec16; P1 / P4 third seeds into the spread above; and act on
Tom's answer on #137 (remove his assignment when you do).
