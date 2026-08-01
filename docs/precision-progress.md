# Precision curriculum — progress report

**Updated:** 2026-07-24 early morning (specialists night 2 — the early-start
night: user authorized a 17:52 start and overnight autonomous iteration)

## Round 7 (night 2): loco_v2 record + THE HIP-YAW A/B VERDICT

Six jobs planned, five ran (loco_v3 dropped mid-night as superseded by the
torsion arm), two improvement rounds designed+verified+queued overnight:

| run | result | verdict |
|---|---|---|
| getup_v2 (ratchet) | 0/16, 2–6 W; training converged to −332/ep = the exact sit-still floor | Economics fixed but PPO never FINDS the rise → **getup_v3 armed**: staged-rise reference (tuck→plant→squat→stand, 4 s script, w=2.0), parity-gated |
| skills_v2 (CoM kernel, from scratch) | 9/64; balance clearance 0% — never lifts | **CoM kernel loophole**: paid 0.75/step for leaning with both feet planted; now clearance-gated → **skills_v3 armed** (160M, from scratch) |
| loco_v2 (smoothness fine-tune) | **42/64, watts 44→5.8, stand 0.3 W, CoT 5.7; square_return 3/8 (first ever), turn hErr 97°** | The fine-tune template is proven: loud from-scratch → penalty-weighted polish |
| loco_v3t (8-DOF+torsion+heading) | 36/64; turn_180 1/8 @ 59° | Heading integrator works (172→59°) but 8-DOF turning plateaus far from the bar |
| **loco_v4yaw (10-DOF)** | **53/64; turn_180 7/8 @ 10°; square 5/8 @ 55 cm; circle 2/8 @ 47 cm** | **HIP YAW WINS DECISIVELY** — see hip-yaw-study.md §5a. Print gate passed. |

Tomorrow night queued: getup_v3 → loco_v4yaw_s (yaw + smoothness fine-tune)
→ skills_v3.

## Round 6 (specialists night 1: loco_v1 / skills_v1 / getup_v1 / loco_v1_ctrl,
## 110M steps each, all four completed inside the window)

First night of the specialist-policy split (plan v2 amendment). All jobs
from scratch except skills_v1 (warm-started from precision_v7b); loco/getup
jobs used the FULL action map (the legacy knee cap finding, 2026-07-22).

**Headline: backward walking passed for the first time in six rounds** —
loco_v1 7/8 (0/8 in every prior round), sidestep 7–8/8, line 6/8.

| verdict question | answer |
|---|---|
| imitation vs ablation | **Tied on passes (36 vs 37/56), imitation wins robustness**: falls 2% vs 16%, slip 1.3 vs 2.5 cm/s, line 2.6 s vs 3.4 s. Keep the mimic prior. |
| specialists vs unified | **Loco: decisive win** (the walk-family block broke open). **Skills: regression** — 24/64 vs v7b's home-turf results; balance_L collapsed to 0/8 (54% clearance) while balance_R is 7/8 at 100% — the L/R asymmetry sharpened. The v7b warm-start + legacy map lineage looks tapped out. |
| does the full knee map produce a rise | **No — and the referee found out why.** getup_v1 idles at 1.4–3 W even under nominal conditions (0/16). Two root causes found in code review: (1) the recovery reward paid ~0.33/step of *absolute-height* income for sitting motionless (~165/episode — matches the observed 127 training reward almost exactly; the do-nothing optimum, recovery edition); (2) `recover_start_mix` defaulted to pure ragdoll, so the referee's sit start was never trained. |

Loco caveat: energetically loud — 44–50 W everywhere including stand
(v7b lineage: 3–5 W). Not a preset bug (weights identical); a from-scratch
110M policy simply hasn't been through the calm-down the 1.4B-step lineage
got. Smoothness fine-tune queued.

**User feedback on loco_v1 (2026-07-23) and what landed for it:**

1. *"Raise the foot by bending the knee, keeping the CG static — not by
   sticking out a leg."* Correct diagnosis: lifts were hip-flexion because
   (a) every skills policy trained under the legacy knee cap (−50° of −95°
   reachable — knee-flexion lifts mechanically impossible) and (b) nothing
   charged for CG excursion. Landed: **CoM-over-stance-foot kernel**
   (w_com_stance 0.75, σ=4 cm, active while lifted) in the skills preset;
   the referee's balance scenarios now record the continuous CoM-offset
   metric.
2. *"Work on turning to face a different direction."* Kinematics truth:
   **no hip-yaw joint exists** (hip roll/pitch, knee, ankle per leg) — no
   V-stance, no heel-to-heel; facing changes only via friction-pivot
   stepping on the 90×46 mm silicone pads. New `turn_180` referee scenario
   measured loco_v1 at **8° of a commanded 180°** (hErr 172°, no falls —
   it simply ignores yaw). Root cause: rate kernels forgive chronic
   under-turning. Landed: **heading integrator** (commanded wz integrates
   into a target facing; error accumulates until paid back; restarts at
   current yaw on command resample) → loco_v3 tonight at w_heading 1.0.
   If the reward fix isn't enough, the options go mechanical
   (lower-torsion pads or a hip-yaw servo).

**Fixes landed for night 2 (2026-07-23):** recovery height RATCHET —
while down, only NEW height above the episode best pays (200/m, one-time,
bounded), plus stand bonus and −0.7/step time pressure sized to cancel the
upright+alive income of a settled non-riser. Sitting still now nets
−0.64/step (verified). Mirrored in the CPU referee, parity-gated.
getup_v2 also trains the full start mix (ragdoll/kneel/squat/sit
0.3/0.2/0.2/0.3). loco_v2 = smoothness fine-tune from loco_v1
(w_power 0.008→0.03, w_action_rate 0.15→0.3).

## Round 5 (precision_v5, +280M warm-started overnight; sit curriculum,
## symmetry penalty, widened foot kernel, honest circle minimums)

| scenario | v4 | v5 | note |
|---|---|---|---|
| balance_L/R | 5/8, 5/8 | **6/8, 6/8** | 95–98% clearance held 10 s |
| line_1m | 8/8 @ 2.2 s | **8/8 @ 1.9 s** | faster AND cleaner |
| stand_10s | 8/8 @ 2.9 cm | **8/8 @ 1.3 cm** | best drift yet |
| crouch_hold | hErr 61 mm | **44 mm** | closing on the 35 mm bar |
| circle_air | (bogus passes) | 0/8, traced r=2.0 cm | now honestly graded: real circles forming but under-amplitude (commanded 4 cm) |
| recover_sit | 0/8 (baseline) | 0/8 | sit curriculum's first night; not yet |
| recover_fallen | 0/8 | 1/8 "up in 0.4 s" | a lucky near-upright ragdoll settle, not a true rise — treat as 0 |
| sidestep / backward / turns | 0/8 | 0/8 | still the stubborn block |
| gait asymmetry | (unmeasured) | 56% | first measurement; symmetry penalty just landed |

Overall: 29/112 under the *stricter* criteria (v4's 37 included bogus circle
passes). Falls 17%, wobble 0.43, CoT 3.9.
**Charter:** "We've been trying to run before we walk." Seven user-specified
precision skills, one command-conditioned policy, severe fall penalties, and
honest referee scenarios for each skill. Spec: [precision-curriculum.md](precision-curriculum.md).
**Montage:** `sim/renders/precision_reel_precision_v4.mov` — every maneuver,
PASS/FAIL captioned per take, honest takes (first passing seed, else best
non-fall attempt).

## The robot being trained (as-built truth)

Not the CAD ideal — the robot as printed today, GoPro mounted:
`sim/bimo_biped_v2_asbuilt.xml`: printed 100 mm foot v3 (90×46 mm pad
contact), torso 341.8 g including the tower-top imu_carrier + BNO055 +
gopro_base stack, camera at its true CG (+94.5 mm, reflecting the taller
battery-bay tower). Verified against the printer's own g-code thumbnails
(hip yoke = the new −110° revision; foot = 100 mm v3). Hardware-claim eval
conditions: GoPro 154 g, 4 ms bus latency, 0.7° backlash, model-error DR,
IMU-only observations.

## Round-by-round

| | steps | compute | headline result |
|---|---|---|---|
| **v1** | 150M | RunPod 4090, ~$1.7 | **The do-nothing optimum.** Zero falls in 112 episodes — by never moving. Standing banked 84% of perfect-tracking reward risk-free. Only stand_10s passed. |
| **v2** | 300M | RunPod 4000 Ada, ~$1.9 | **Motion returns** (dense directional progress + a real bug fixed: the yaw kernel had been reading the sidestep channel). line_1m 4/8 @ 2.1 s; attempts everything, falls 30%; sidestep/backward/crouch/turns not tracked. |
| **v3** | 400M | Mira 4070 Ti (electricity) | **Calm + solid walking, skills dodged.** line_1m 5–6/8 @ 2.0 s, stand 8/8 @ 2.1 cm drift, falls 18%, wobble 0.70→0.24 rad/s, CoT 7.9→2.5. But lift/crouch/recover commands answered with calm standing — kernels paid ~4/step for vels=0 under any command. |
| **v4** | 282M (overnight window) | Mira, nightly 23:00–07:00 | **The gate unlocked the skills.** One-leg balance 5/8 with 92–94% real ≥3 cm clearance (v3: 0%), air circles 5–6/8, line 8/8. Overall seed-pass 13 → **37**/104. Recovery now visibly fights (13 W) but doesn't yet stand. |

## Current scorecard (precision_v4, hardware-claim conditions, 8 seeds/scenario)

| scenario | pass | headline | wobble | watts |
|---|---|---|---|---|
| balance_L | **5/8** | clear 92% | 0.59 | 3.5 |
| balance_R | **5/8** | clear 94% | 0.74 | 3.7 |
| circle_air_L | **6/8** | trkErr 165 mm | 0.65 | 4.1 |
| circle_air_R | **5/8** | trkErr 116 mm | 0.79 | 3.9 |
| line_1m | **8/8** | t=2.2 s | 0.46 | 2.6 |
| backward_1m | 0/8 | never reaches | 0.29 | 5.3 |
| sidestep_L/R | 0/8 | never reaches | 0.12/0.61 | 1.6/1.9 |
| square_return | 0/8 | ret 119 cm | 0.82 | 6.1 |
| circle_return | 0/8 | ret 386 cm | 0.84 | 9.3 |
| crouch_hold | 0/8 | hErr 61 mm (v3: 85) | 0.41 | 3.3 |
| recover_fallen | 0/8 | no stand yet (actively trying, 13 W) | 0.47 | 13.2 |
| stand_10s | **8/8** | drift 2.9 cm | 0.06 | 3.0 |

**Overall: 37/104 seed-pass (2 scenarios clean), falls 29%.** Read with the
caveat that falls/wobble rose *because the policy now attempts everything* —
v3's calm came partly from dodging the hard skills.

**Post-audit correction (2026-07-19, user feedback):** the circle_air
"passes" above were too generous — the referee never required the foot to
actually *trace* the circle. Re-graded with real minimums (≥3 cm traced
radius + a full sweep; balance also gains a 10 cm drift bound), the circles
FAIL: the foot traces ~4.5 cm of wander but doesn't complete sweeps.
Training round 5 (tonight) attacks this with a widened foot-target kernel
(3→6 cm — the narrow kernel paid no gradient at v4's foot error), plus a
new gait-symmetry penalty (L/R swing-duration matching) and a reverse
curriculum for recovery (kneel/squat/ragdoll starts in training; the
referee still grades pure ragdoll).

## The smoothness/stability story (user concern 2026-07-18: "very shaky")

Metric panel across rounds (aggregate over all scenarios):

| round | wobble RMS (rad/s) | mean watts | cost of transport | falls |
|---|---|---|---|---|
| v2 | 0.70 | 10.9 | 7.9 | 30% |
| v3 | 0.24 | 2.0 | 2.5 | 18% |
| v4 | 0.53 | 4.7 | 3.6 | 29% |

(v3→v4 wobble/falls rose because v4 *attempts* the risky skills v3 dodged;
compare per-scenario: stand wobble is 0.06 in both, line_1m 0.31→0.46.)

The shakiness collapsed **without a dedicated smoothness round** — the dense
progress economics stopped paying for nervous trembling (a struggling policy
burns watts; a competent one doesn't). A targeted smoothness fine-tune
(raised pitch-rate/action-rate/power penalties at low LR — CLI overrides
ready in `train_mjx.py`) stays queued for after the skills land, judged by
wobble RMS on the *moving* maneuvers.

## What we measure (pass/fail AND continuous)

Every scenario records continuous metrics per seed (endpoint error, tracking
error mm, clearance %, drift RMS, time-to-target/stand) aggregated mean+worst
in `sim/runs/<run>/scorecard.json`; the pass/fail column is a threshold view.
Training optimizes purely continuous signals. Smoothness panel everywhere:
wobble RMS, electrical watts, cost of transport P/(mgv), foot slip, L/R
swing-time symmetry.

## Bugs & findings the referee caught along the way

1. **Do-nothing optimum** (v1) — severe fall penalties + wide kernels ⇒
   safety by inaction. Cure: dense directional progress (pays zero for
   standing, negative for wrong-way motion).
2. **Yaw kernel read the sidestep channel** in the MJX ext mode (2-channel
   unpack survived the 7-channel migration). Pivots were never rewarded in
   round 1. Caught by a parity block with a non-zero fixed command.
3. **Grounded-stance parity "regression"** — actually a test-guard bug:
   engines were compared on post-step contact counts while forces come from
   the pre-step manifold; the silicone-pad sole drop exposed one
   force-carrying mismatched step (MJX keeps a 0.05 mm grazing corner CPU
   omits). Guard fixed; suite back to 1e-11, now 7 gates.
4. **"Balance" was a 2 mm hover** — the ≥3 cm clearance criterion (user)
   revealed lift attempts never actually cleared the ground.
5. **Standing-through-skill-commands** (v3) — tracking kernels paid full
   base under lift/crouch; now gated by compliance (v4).
6. **Joint-limit rows are engine-divergent** like contact manifolds — ragdoll
   poses ride them; the recovery parity gate screens both.

## Infrastructure (all one-command)

- **Nightly Mira window**: cron 23:00 (waits for free VRAM; unloads *idle*
  ollama models after ~10 min of 0% GPU — never touches active inference),
  no new starts after 05:00, hard stop 07:00 (checkpoint-safe SIGTERM).
  `infra/night_arm.sh <train args>` to queue, `infra/night_collect.sh` to
  pull + referee. First armed night validated end-to-end (launched 23:00:01,
  stopped 07:00:01, 282M steps).
- Daytime: `infra/precision_round.sh` (probes Mira, falls back to RunPod
  only if forced). RunPod retired by user policy after round 2 (~$3.5 of
  credit remains).
- Referee: `sim/mjx/eval_precision.py` (13 scenarios); montage:
  `sim/mjx/render_precision_reel.py`. CPU MuJoCo referees every claim —
  MJX numbers are never trusted alone (7-gate parity suite).

## Open items

- **Recovery-from-fallen** is the hardest skill (integrated 2026-07-18:
  15% of episodes start from settled ragdolls, recovery-shaped reward until
  first stand). The as-built 100 mm feet leave a ±20 mm CoM corridor for the
  rise — the 116 mm get-up soles exist in CAD if hardware wants the wider
  margin (print trigger documented in DESIGN.md).
- Turn-while-walking tracking (square/circle return errors ~1–2 m) — turn
  authority is roughly half of commanded; a heading-error term or a
  mirrored-horn leg (mechanical yaw-bias fix) are the candidates.
- Sidestep and backward gaits — attempted, not yet converged.
- Odometry: return-to-start evals use sim ground truth; the hardware has no
  pose estimate yet (known gap, DESIGN §8).

## Found 2026-07-22 (user question about knee usage): the legacy knee cap

All precision rounds trained with the LEGACY action mapping, which cannot
command the knee past -50 deg of its -95 deg range (the symmetric-band bug
the get-up study documented -- and fixed with action_map="full" -- but the
precision lineage never adopted). Two consequences: (1) knee-shy gaits --
foot clearance was being achieved by hip-hike/circumduction because deep
knee flexion was literally unreachable; (2) **recovery episodes could never
command kneeling-depth knee angles (~-93 deg)** -- a strong candidate for
why recovery-to-stand refused to converge across four rounds. Fix: tonight's
from-scratch specialists (loco_v1, loco_v1_ctrl, getup_v1) train with the
full mapping; skills_v1 stays legacy for its v7b warm-start and migrates at
its next from-scratch round.

## Night 3 (2026-07-24→25, early start 17:17): first 10-DOF specialist round

All jobs on the v3yaw plant (user policy 2026-07-24). Queue: getup_v3 →
loco_v4yaw_s (smoothness fine-tune on the A/B-winning loco_v4yaw) →
skills_v3 (clearance-gated CoM kernel). A locale-collation bug in the night
runner's queue sort launched skills before loco; caught at +3 min, swapped
(runner now sorts under LC_ALL=C).

**getup_v3 (staged-rise reference): 0/16 — third consecutive getup failure.**
Referee: recover_sit 0/8, recover_fallen 0/8; falls 0%, watts 1.0 — the
converged behavior is again "hold still". The training curve is damning:
`eval/episode_height` never exceeded ~0.20 m (sitting height) in 110 M
steps; reward improved −1335 → −82 purely by shrinking penalties. Reel:
`sim/renders/precision_reel_getup_v3.mov`.

**Diagnosis.** The v3 reference is time-indexed from episode start, but
(1) the policy has **no clock anywhere in its obs** — it cannot track a
time schedule it cannot see; and (2) the reference is **misaligned with the
reverse-curriculum starts**: a kneel start (weight already on feet) is paid
by the exp kernel to collapse *back into the t=0 tuck*. The exp kernel is
near-zero at ragdoll distances, so from most starts the guidance gradient
is flat and the height ratchet alone has to carry discovery — it doesn't.

**getup_v4 (implemented tonight, parity-gated):** (a) rise-schedule phase
exposed to the policy via command channel c5 (foot_dx — zero on recovery
episodes otherwise): ramps 0..1 over `rise_secs`, holds 1 once recovered;
(b) per-start phase offset `State.rise_t0` — kneel starts enter the
schedule at the plant stage (1/3) instead of being dragged backward; the
squat start already *is* the reference's t=0 tuck (t0=0), ragdoll/sit
must reach it (t0=0); (c) start mix rebalanced toward the rise path:
ragdoll/kneel/squat/sit = 0.2/0.2/0.3/0.3. No obs-layout change — other
families are untouched (phase injects only when `w_rise_ref > 0` on
recovery episodes). Queued for the next free GPU window.

**loco_v4yaw_s (smoothness fine-tune on the A/B winner, +w_power 0.03,
+w_action_rate 0.3, warm-started): 64/72 — new deployable-best on v3yaw.**
Finished in ~2.5 h (warm start). vs loco_v4yaw: pass rate 83→89%, overall
watts **63.6→5.9** (CoT 57.8→5.7, a 10× efficiency gain for free), gait
asym 17→14%. The user's three feedback items now grade: turning —
turn_180 **8/8 @ 7°** (was 7/8 @ 10°), square_return **8/8 @ 7 cm**;
standing wobble — 0.06 rad/s @ 0.8 W (stand_10s 8/8, drift 1.4 cm);
torque-off standing — stand_off 5/8 @ 0.2 W (drift 13.9 cm). Remaining
weak spots: circle_return 4/8 @ 46 cm / 19.3 W (improved from 2/8 @
55.6 W but still the hardest scenario — continuous turn-while-walk), and
stand_off release transients. Reel:
`sim/renders/precision_reel_loco_v4yaw_s.mov`.

**skills_v3 (clearance-gated CoM kernel, 160 M): 26/64 vs skills_v2's 9/64
— the gate killed the leaning loophole.** One-leg balance: balance_L 6/8 @
95% clearance, balance_R 8/8 @ 100% (v2: 0/8 + 0/8 at **0%** clearance —
it leaned onto the stance foot with both feet planted). March 4 clean
lifts/leg (v2: 2). Still failing: circle_air 0/8 (foot-trajectory tracking
r≈1–3 cm), crouch_hold 0/8 (hErr 95 mm), hip_sway 3/8. Finished the full
160 M in 3.6 h (queue-order swap cost nothing). Reel:
`sim/renders/precision_reel_skills_v3.mov`. Next lever for the air skills:
the c5/c6 foot-target channels train on air-circle commands 12% of
episodes — likely needs a dedicated foot-tracking kernel weight bump or a
skills fine-tune with a larger traj share.

**getup_v4 (phase-visible aligned reference, hard-stopped 07:00 at ~50 M of
110 M): 0/16 at the half-run checkpoint, but the failure mode changed** —
watts 1.0→3.8, slip 1.6 cm/s (it moves and works the schedule instead of
holding still), eval height creeping 20→22 cm (v3 *declined* to 17), and
reward still climbing at the cut (−5, vs v3's −82 plateau). Finisher
`getup_v4b` (+60 M warm-start) queued ahead of loco_v5t for night 4.

## Getup feasibility study (2026-07-25 daytime, sim/mjx/scripted_getup.py)

Hand-authored open-loop schedules through the CPU referee, instrumented
with CoM-vs-foot telemetry. Findings: (1) **quasi-static rises are
impossible from the sit** — folding the knees lifts the feet into the air
(foot_z → 0.20 m) while the butt carries the weight; with no arms there is
no static weight-transfer path, so the v1–v4 staged reference was asking
for an infeasible motion. (2) The **deep-tuck crouch is statically
stable** (CoM lands 3–4 cm ahead of the foot centers) — the rise from a
loaded crouch is fine; only the ground→crouch transfer needs momentum.
(3) A human-style momentum rock (roll back, tuck, snap hips) gets upright
over the feet but toppled back in a 27-combo timing grid — open-loop
tuning is the wrong tool; a feedback policy should close it. **getup_v5:**
w_rise_ref cut 2.0→0.5 (hint, not master), new w_up_vel=2.0 (positive
root vertical velocity while down — pays the momentum burst the ratchet
alone under-rewards). skills_v4: new w_foot_under=0.75 (tight horizontal
kernel on the raised foot, clearance-gated) so lifts must come from KNEE
flexion, not a swung-out leg (user feedback). Reels now trim trailing
standstill to ~2 s (user request). Parity 8/8 with both terms gated.

**loco_v5t (turn_emph fine-tune, 60 M): 64/72 — new deployable-best on
robustness.** Ties v4yaw_s on pass count but falls 8%→**0%**, watts
5.9→4.2 (CoT 4.6), wobble 0.35→0.28. circle_return: error 46→37 cm and
19.3→12.3 W but 3/8 passes — the residual failure is return-to-point
accuracy, not turning ability (square_return stays 8/8 @ 8 cm). Verdict:
command-distribution surgery is done; circle-scale return accuracy is a
goal-conditioning problem (the planned next abstraction layer).

**getup_v5 (momentum incentive): 0/16 — the w_up_vel term was farmed by
rocking cycles** (positive vertical velocity pays every up-phase of an
oscillation with no net rise; wobble 0.51, watts 5.3, height plateau
23 cm, late reward oscillating). Removed for v6 — the height ratchet is
the correct "net new height only" signal.

**BUG (found while wiring v6): `getup_start_mix` clobbered the ext-mode
`recover_start_mix` passthrough** — an unconditional default assignment
7 lines below the conditional one. Consequence: **getup_v3, v4, and v5
all trained from PURE RAGDOLL starts**; the reverse-curriculum mix in
their configs never reached the env. Every conclusion about "the
curriculum not helping" was untested until now.

**getup_v6 (queued): the first honest curriculum run** — fixed mix
passthrough; new 5th start type `catch`: 4 harvested mid-momentum-rock
states (qpos+qvel, no settle — the near-catch instant the scripted rocks
kept reaching before toppling backward), 30% of starts; mix
0.15/0.1/0.2/0.25/0.3 rag/kneel/squat/sit/catch; w_rise_ref 0.5 hint;
no w_up_vel. Parity 8/8 with catch starts exercised.

**skills_v4 (w_foot_under): 29/64 — march_in_place 1/8 → 6/8 (6 clean
lifts/leg), balance holds 6/8+8/8 @ 98–100% clearance.** Knee-usage probe
(balance_L, seed 107): lifted-knee mean deviation 0.81 rad — the knee
works hard — but hip-pitch deviates 1.44 rad: the learned lift is a
HIGH-KNEE (thigh up, foot kept under the hip = CG-static, the kernel's
goal) rather than a heel-flick (thigh vertical, shin folds back). A pure
heel-flick puts the foot slightly BEHIND the hip, which w_foot_under
mildly punishes. If the heel-flick look is wanted: add a lifted-leg
hip-pitch deviation penalty (skills_v5 candidate, user's call).
hip_sway regressed 3/8→1/8 (kernel unrelated to sway; likely fine-tune
noise). crouch_hold/circle_air still 0/8.

## Day 5 (2026-07-26): getup_v6 0/16 — but the diagnosis finally lands

**getup_v6 referee: 0/16 (recover_sit 0/8, recover_fallen 0/8), falls 0%,
wobble 0.18 (was 0.51), watts 4.0.** The oscillation farming is gone; the
policy went *passive* instead. Sixth straight zero — so the day went to
per-start-kind probes (scratch `probe_getup_v6.py`) instead of a seventh
reward tweak. Every start kind fails differently, and each failure
exposed a false assumption baked into the whole getup lineage:

| start | v6 policy behavior | root cause found |
|---|---|---|
| kneel | "stands" 3/3 (h 0.28→0.31) | start was ALREADY standing (see below) |
| squat | freezes at h 0.14 forever | pose is a physical trap (see below) |
| sit | freezes at h 0.14–0.17 | no feasible corridor was ever on offer |
| catch | collapses backward to lying | catch target = the squat trap |
| ragdoll | stays flat | ditto |

**Finding 1 — the deep squat is INFEASIBLE, and the reference walked the
policy into it.** Open-loop study: at max hip fold + max knee flexion the
whole-robot CoM sits **9 cm behind the foot centers**; every extension
from the deep squat tips backward (0/4 seeds, and the same backward-tip
kills scripted kneel/fallen finishes). `_rise_ref`'s
tuck→plant→deep-squat schedule — inherited from getup_v3 — was a guided
tour into the trap; v6's freeze at h 0.135 is the policy stopping just
short of the tip-over the reference asks for. The momentum-rock "catch in
a deep squat" strategy (night-4 feasibility study) dies with it: a
perfect catch lands in a pose that cannot hold. 50% of v6's start mix
(squat 20% + catch 30%) trained an unwinnable task.

**Finding 2 — the "kneel" start was standing all along.** Staged starts
settled with `ctrl = default` (the STAND pose), i.e. the servos drove
toward standing during the settle: the kneel start entered the episode at
h 0.277 — 2 mm ABOVE the 0.85·nominal recovered threshold — so every
kneel episode began `recovered=1` and produced **zero recovery training
signal**. The v6 "kneel wins" in the probe are the policy holding a
near-stand it was spawned into. No training episode ever started in a
stable ground pose with the recovery reward active and a feasible gap to
close. That — not reward economics (the 200/unit ratchet + −0.7/step
time pressure were already right) — is why six rounds produced no riser.

**Finding 3 — the high-kneel is the missing rung.** Holding the kneel
POSE (not the stand pose) settles into a statically stable torso-vertical
kneel at **h 0.225 = 69% of standing height**, comfortably below the
recovered threshold. From there the gap to "recovered" is ~5 cm and the
support polygon (shins+insteps) is the one that works. A naive
snap-to-default from the static high-kneel falls forward — the finish
needs feedback — but that is exactly the kind of gap PPO closes when the
start state is honest.

**getup_v7 (queued, 9996): the corridor rebuild.**
- `_rise_ref` rewritten: ball/child's-pose → high-kneel → knees-extend
  (the feasible corridor), replacing tuck→plant→deep-squat.
- Staged starts settle 1.0 s holding their OWN pose (`_settle` grew a
  `ctrl` arg; CPU mirror identical): kneel 0.225, squat 0.136, sit 0.142
  — all honestly DOWN now.
- RSI bank rebuilt (`harvest_rise_states.py`, replaces the 4 rock states):
  48 settled stable rows — 26 ball/child's-pose (t0=0) + 22 jittered
  high-kneels (t0=rise/3) — each carrying its reference phase in a new
  npz `t0` column both envs read. Half-lunge rung attempted and dropped:
  0/60 jitters statically stable (honest negative; the lunge transfer is
  feedback's job).
- Start mix 0.25/0.25/0/0.25/0.25 rag/kneel/–/sit/bank; squat retired.
  w_rise_ref 0.5 → 1.0 (the hint now points somewhere feasible).
- Parity 8/8 PASS after every change (ran twice).

Expectation ladder for v7: bank/kneel starts must stop being 0% —
that's the honest test of the corridor. sit/rag remain hard (the
sit→kneel transition needs a tip-and-roll no reference expresses); if
v7 rises from the kneel rungs but not from sit, the next move is
banking policy rollout states along whatever partial progress v7 shows,
not another reward term.

## Day 5 cont. (2026-07-26): getup_v7 0/16 — the corridor works, the finish doesn't

**getup_v7 referee: 0/16, falls 0%, wobble 0.18, watts 4.2.** But the
per-start probe shows real movement: **ball bank starts CLIMB to the
high-kneel (h 0.09 → 0.22–0.23)** — the rebuilt corridor is learned —
and then every start kind, including the true high-kneel itself,
descends to a common parked pose at h 0.14 and stays. Training telemetry
explains both behaviors at once: **`eval/episode_recovered` ≈ 0 for the
entire 110M steps.** Not one training episode ever reached standing, so
the value function has zero evidence standing pays; the −0.7/step bleed
reads as unavoidable background and the policy optimizes comfort.
Two mechanisms:

1. **The finish is an exploration wall.** The bank's top rung (static
   high-kneel) is still ~1 s of coordinated knee-extend + balance-catch
   from the threshold. PPO exploration never completes it, so the income
   side of the reward stays invisible. (Torque is NOT the wall: the
   commanded stand-drive from the kneel pose demonstrably lifts h
   0.13 → 0.277; only the final balance catch fails open-loop.)
2. **Parking is price-free.** The ratchet is one-way, so once best_h is
   collected, h 0.14 and h 0.23 pay identically — the policy descends
   from the high-kneel because nothing says stay up.

**getup_v8 (queued 15:24, running): finish rows + height-holding income.**
- Bank grew a `snap` rung: 60 rows sampled along commanded rises of the
  jittered kneel poses — a slow servo-ramp band (h 0.225–0.24, small vz)
  plus the ballistic drop-flight sampled every step (h 0.21–0.271, vz up
  to 1.6; the 0.24–0.27 band lasts <50 ms and needed per-step sampling).
  Top rows are 0.05–0.5 s from the recovered threshold with the servos
  already driving standward — close enough for exploration to cross,
  flip `recovered`, and finally light the income signal. Bank now 142
  rows (60 ball / 22 kneel / 60 snap stratified by height), bank share
  raised to 40% of starts.
- **Relative height income while down**: rec_primary += 0.6·h/nominal
  (max 0.5/step, still < the 0.7 bleed → do-nothing stays net-negative;
  unlike getup_v1's absolute-height income this is bounded away from
  farming). Holding the high-kneel now beats parking low.
- Parity 8/8 PASS (2f exercises the new arithmetic).

## Day 5 night (2026-07-26): getup_v8 — the income signal finally lights

Infrastructure first: v8 crashed twice at launch (cuSolver / CUDA OOM) —
both times a race against the night runner's own "migrate ollama back to
GPU" step; the 90% XLA preallocation needs ~10.4 GiB + CUDA context and
the runner's launch gate accepted 10.0 GiB free. Gate raised to 11.5 GiB
and idle ollama models unloaded before relaunch; third attempt ran the
full 5.1 h cleanly.

**Referee: 0/16 (unchanged). Training: transformed.**
`eval/episode_recovered` 2–5 across the whole run (v7: ~0) with an
upward trend, and episode reward crossed POSITIVE (+80 vs v7's −210) —
training episodes reach standing routinely now, so the value function
finally has evidence the stand pays. The probe shows the new frontier:

- **From the top snap rows (h 0.26) the policy reaches standing height
  (hmax 0.34 = 105% nominal) — then falls.** The finish is discovered;
  the HOLD is not yet. Falls after recovery now terminate with fall_cost,
  so the pressure points the right way.
- kneel/sit/squat still park at h 0.14 — the value gradient has not yet
  propagated down the ladder (expected: the income only just lit).

**getup_v8b (queued): pure continuation** — warm-start from v8 params
(`--init-from getup_v8`), identical config, +110M steps. No new
mechanisms; isolates "more training after the income lit" as the sole
variable. If v8b holds the stand from snap/kneel rungs but sit/rag stay
parked, the next bank refresh harvests v8b's own partial rises (real
policy states, not scripted ones) as the mid-ladder.

Also from today's bench thread (docs/bringup-day1.md): all 10 servos
ID'd and chain-verified — but the sim-relevant dynamics numbers
(backdrive friction, true stall, step response) are still unmeasured, so
the actuator model stays datasheet+fit. Backdrive friction is the one to
take before assembly: torque-off standing flips infeasible below
0.25 N·m (sim estimate 0.35).

## Day 6 early (2026-07-27): v8b flat — and the v8 "breakthrough" was a farm

**getup_v8b (warm-start continuation): referee 0/16, probe IDENTICAL to
v8.** Training metrics kept climbing (recovered 4→7, reward +114) while
the deterministic policy didn't move — the classic smell of an exploit.
Rendering the one probe rollout that "stands" (bank row at h 0.26)
settled it in 0.32 s of video: the policy is a PASSENGER. The snap row's
launch momentum coasts it ballistically through standing height (0.337);
recovered flips on the instantaneous height crossing at t=0.02 s; the
ratchet pays ~+15 for altitude the policy didn't earn; the feet slide
out; the fall-after-recover rule terminates the episode at t=0.32 s.
Net-positive income in a third of a second with zero skill — the top
bank rows were a farming loop, and "recovered 4–7" was ballistic flips,
not standing.

**getup_v9 (queued, warm from v8b): two structural rules, no new
rewards.**
1. **recovered requires a HELD stand** — stand_hold_n = 0.5 s of
   consecutive standing steps (new `stand_streak` State field + CPU
   mirror). A ballistic crossing no longer flips anything.
2. **Recovery episodes never terminate on falls** (pre- OR post-recover)
   — the farm's exit door closes: a flop now means living with the
   −0.7/step bleed for the rest of the episode, and every fall is a free
   in-episode retry of the rise. Income flows only while actually
   standing still.

Parity 8/8 PASS (2f exercises the recovery arithmetic on both sides).

## Day 6 morning (2026-07-27): v9 partial honest, v10 fresh

**getup_v9 (66M of 110M, night_stop took it at 07:00): referee 0/16,
recovered honestly 0.00 all run** — the farm is dead (v8b showed 4–7
from ballistic flips). Mean episode height +22% over v8b (the policy
lives higher under the height income), but the deterministic probe is
unchanged: parks at 0.14 from every start.

**The blocker exposed by the training log: entropy ≈ 0.06.** The warm
chain (v8→v8b→v9) carried a nearly-deterministic policy into rules that
now require *discovering* a held stand — with almost no exploration
left to discover it with. Warm-starting preserved the corridor skill
AND the parked habit AND the collapsed entropy.

**getup_v10 (launched 07:01): fresh weights under the honest rules** —
no init-from, entropy 0.005→0.01, all v9 machinery (kneel-corridor
reference, 142-row bank incl. finish rows, relative height income,
held-stand recovered, no-termination recovery episodes). This is the
clean end-to-end test of the rebuilt curriculum. Runner race fixed too:
post-unload recheck 30 s→5 s (a 3 am vision workload was reloading
llama3.2-vision faster than the runner could claim the GPU).

## Day 6 midday (2026-07-27): getup_v10 — first honest held stands (in training)

**getup_v10 (fresh, entropy 0.01, honest rules): referee 0/16, but the
training curve is the first genuine one in the lineage** — recovered
(now = HELD-stand steps only) climbed 0 → 7 → 33 → 52 by 91M steps,
reward −332 → +158. Final eval dipped to 18.6 (std 148 — the metric is
a few full-episode holds among many zeros, so eval noise is huge).

Where the stands actually are (deterministic + stochastic CPU probes):
the policy can sometimes CATCH AND HOLD from top bank rows (h 0.26 with
upward velocity — one CPU draw held 0.28 s; MJX evals hold whole
episodes from these), but the static high-kneel rise is still
undiscovered — every kneel start still parks at 0.14. The MJX-vs-CPU
"gap" was sample size: 40% of 128 MJX eval envs draw bank rows and ~40%
of the bank is the top band; 4 CPU probe seeds rarely land there.

**getup_v10b (running 12:09): consolidation** — warm from v10, entropy
back to 0.005, same everything. The bet: the catch-and-hold at the top
band consolidates deterministic, then value propagates down band by
band (0.24 → 0.22 → static kneel). If v10b consolidates the top but
doesn't propagate, next bank refresh harvests v10b's own successful
hold trajectories (real policy states) as intermediate rungs.

## Day 7 night (2026-07-28): v10b regresses — pivot to two-stage discovery

**getup_v10b (consolidation, entropy 0.005): referee 0/16, and worse —
recovered climbed to 44 by 30M then COLLAPSED to 0 from 71M on.** The
deterministic probe lost even v10's ballistic top-row catch. Reward
stayed +60..130 with zero held stands: ratchet + height income +
sub-0.5 s standing blips pay fine without ever holding, and the hold
itself is fragile enough under full DR that PPO drifted back to the
parked mode. Consolidation consolidated the wrong thing.

**The 11-round pattern, honestly:** discovery keeps failing UNDER FULL
HARDENING — every getup run trained with DR + 154 g payload + latency +
backlash + kicks from step 0, because the referee demands them at eval.
The literature recipe (unified-humanoid-getup, catalogued in the
research pass) splits this: stage 1 discovers the skill in CLEAN
physics, stage 2 warm-starts and layers the hardening back. Discovery
difficulty and robustness were never supposed to be bought together.

**getup_v11 (queued): stage-1 discovery** — new `--discovery` flag in
train_mjx (no DR, no payload, no latency/backlash, no pushes; env code
untouched, parity unaffected), fresh weights, entropy 0.01, all v9/v10
machinery intact. Success test: does clean physics + honest rules +
the corridor curriculum produce a deterministic kneel→stand at all? If
yes → v12 re-hardens from it. If no → the reward/curriculum design is
wrong at a deeper level than conditions, and the next stop is AMP-style
reference imitation (LocoMuJoCo mocap) rather than more shaping.

## Day 7 morning (2026-07-28): getup_v11 — the decisive negative

**Clean-physics discovery run: recovered 0.00 for all 110M steps.** No
DR, no payload, no latency/backlash, no pushes, honest rules, corridor
reference, finish rows, entropy 0.01 — and not one held stand. The
pre-registered decision rule resolves: **the reward/curriculum design
itself does not produce standing; the hardening was never the blocker.**

The v10-vs-v11 comparison adds a sharp irony: hardened v10 reached 52
held-stand steps because push_kick VELOCITY KICKS were tossing the
robot near standing — free exploration the policy never had to learn.
Clean physics removed that accidental scaffold and the parked optimum
was found faster (reward +252 by 20M) and never left.

**Where this leaves the getup effort after 12 rounds / ~1.3B steps:**
shaping + RSI + honest gates found and fixed six real bugs/exploits and
mapped the feasible corridor, but PPO exploration cannot bridge the
final dynamic maneuver (kneel -> balance-catch -> hold) from income
signals alone. The literature answer is reference imitation: a
DeepMimic-style phase-indexed TRACKING objective (pose + velocity,
dominant weight) over a retimed kneel-corridor trajectory, with RSI
along it — the env already has the machinery stubs (w_mimic/_mimic_ref
for gait, w_rise_ref pose kernel, per-row bank t0). That is a Phase-B
build, and a strategy fork worth a human call:
  (a) build getup imitation (env work: mimic-style rise tracking with
      velocity terms; ~a day of build + validation), or
  (b) park getup as "assist to kneel, servo-snap the rest" for now and
      return GPU nights to locomotion/goal-conditioning, where the
      pipeline is demonstrably productive (loco_v5t 64/72 deployable).

**Decision (user, 2026-07-28): getup is PARKED.** Option (b): no
self-recovery for now — assist to kneel by hand on hardware, the servo
stand-drive covers the rest. GPU nights return to locomotion and the
goal-conditioning layer. The imitation build stays documented above as
the revival path if it's ever wanted.

## Day 7 (2026-07-28): goal layer, first evidence — closed-loop homing 2x

Getup parked (user call); task #15 opened: goal-conditioning. First
experiment before any training: a CLOSED-LOOP outer controller (P on
bearing/distance, omnidirectional body-frame creep inside 0.3 m, 5 cm
stand latch) wrapped around the untouched loco_v5t policy, evaluated on
the circle_return task with ground-truth position.

**Result: 6/8 vs the open-loop baseline's 3/8, zero falls; both misses
by <= 3 cm.** Two design facts fall out:
1. Goal-seeking may not need a goal-conditioned POLICY at all — an
   outer command loop + odometry might be the whole layer.
2. The residual is low-speed tracking: `cmd_v_range=(0.3, 1.0)` means
   the policy has NEVER trained below 0.3 m/s; the homing creep
   (0.08–0.25) ran outside the training distribution entirely.

**loco_v6creep (queued for tonight's 22:00 cron): warm from loco_v5t,
`--cmd-v-range 0.05,1.0`** (new train_mjx flag) — teach the creep band,
then re-run the homing eval. Open deployment question for the goal
layer: position source on hardware (IMU odometry drift vs GoPro).

## Day 7 evening (2026-07-28): plant sync for the tower resize

Physical changes today (CAD session): TOWER 42x43.5 -> 56x75 to mount
the General Driver board UPRIGHT (65x65, onboard IMU), pack reseated to
-x, GoPro +31.5 mm, foot rounded toe/heel + solid heel, leg_link cuts.
Sim incorporation, verified end to end:

- **Articulation precursor: check_assembly ALL CLEAR** at every ROM
  extreme with the new shapes (incl. the new horn-rib/idler-hub servo
  mocks). The dressed-cable ROM sweep fails at frame 137
  (R_cable_hip unexportable) -- cable re-sweep issue with the new tower,
  flagged for the CAD session; cables are not in the sim plant.
- Part deltas (STL volume x effective print density): tower +38.3 g,
  everything else sub-gram. Inertia rebuild (with the CAD session, which
  was editing the same files live): torso 341.8 -> 402.3 g, COM z
  +2.9 -> +14.6 mm, total robot 1.152 kg. Battery mock follows
  BATT_SEAT_X to x=-9; board slab upright at +x; IMU site moved onto
  the board (honest accelerometer lever arm); gopro_base mesh +31.5 mm;
  payload_cg_z 0.0945 -> 0.1260 in train_mjx.
- **Parity war story:** the fresh inertials broke gate 2g (airborne
  10-DOF) at dqpos 2e-7. Bisection: feet inertials alone reproduce it;
  canonicalizing the eigh frame (the foot came out as a near-180-degree
  quat, w=0.0006) changed nothing -- values, not representation. Model
  structs bit-identical. Per-step probe under x64: 99 steps at 1e-12,
  ONE step (t=88) at dqvel 5e-5 with several joint torques within
  machine epsilon of the sts3215 envelope/backlash branch -- XLA op
  ordering flips a discontinuous branch. Boundary luck, not divergence.
  Fix: 2g probe phase 0.35 -> 0.353 steps off the knife edge; suite
  passes at full strictness (2g dqpos 2.9e-12). Builder now
  canonicalizes eigen-frames to minimal rotation regardless.
- New plant + meshes rsynced to Mira BEFORE tonight's launch:
  loco_v6creep (warm from loco_v5t, --cmd-v-range 0.05,1.0) now does
  double duty -- creep-band learning AND adaptation to the +100 g,
  taller-COM plant. Referee deltas vs loco_v5t will mix both effects;
  interpret accordingly.

## Day 8 (2026-07-29): goal_home SOLVED in sim — and the new body holds up

**loco_v6creep (warm from loco_v5t, cmd_v_range 0.05-1.0, first run on
the resized-tower plant): 63/72** — statistically the old 64/72 on a
robot that gained 100 g and 12 mm of torso COM. All core gaits 7-8/8;
open-loop circle_return 3/8 -> 5/8 (creep training already helps).

**Closed-loop goal homing: 8/8, zero falls, median return 4.5 cm** (was
3/8 open-loop baseline; 6/8 with the outer loop on the creep-blind
policy). Promoted from scratchpad into the referee as `goal_home`
(circle arc, then P-controller homing with body-frame creep inside
0.3 m; gate: return < 10 cm, no fall; in the loco family + ORDER +
HEADLINE). Referee confirms 3/3 at home 3 cm. The goal layer for flat
ground is: outer command loop + creep-band policy — no goal-conditioned
network needed. Remaining deployment question: position source
(ground truth in sim; hardware needs odometry or the GoPro).

**Regression flag: stand_off 8/8 -> 4/8 (drift 1.9 -> 17.3 cm).** The
heavier, taller torso makes torque-off standing marginal against the
estimated 0.35 N·m backdrive friction. Measuring the REAL backdrive
friction (bring-up checklist) is now decisive for whether idle
torque-off survives the tower resize.

## Day 8 cont. (2026-07-29): terrain + knee enticement (loco_v7knee)

User direction: the knee never moves ("tip-toe" gait), and the perfect
sim floor is suspect — emulate carpet / vary roughness per iteration.
Measured ground truth first: at 0.5 m/s the hip swings 34 deg p2p, the
ankle 27 deg, the KNEE 5.8 deg — jammed at +1 deg against its extension
stop. On a mathematically flat rigid plane, straight-leg + ankle-vault
is optimal, and the gait reference's 31 deg swing-knee ask loses in the
lump-sum imitation kernel.

The build (both envs, parity gate 2h added, 9/9 PASS):
- **Terrain mosaic**: static 12 x 6 m heightfield of 1 m tiles, each
  0/2/5/10/15/20 mm roughness at 0.10-0.20 m feature size (carpet pile
  to door-threshold), generated once (gen_terrain_mosaic.py, seed 0)
  into sim/terrain_mosaic.npz — BOTH envs load the same file, so the
  ground is bit-identical by construction. Per-episode roughness = a
  random spawn draw on the mosaic (no MJX model batching needed; the
  1024 env slots are the diversity population). Height, fall checks and
  foot clearance all measured against LOCAL ground (bilinear lookup,
  the old Stage-2b CPU machinery generalized + ported to jax).
- **Knee-weighted mimic**: per-joint weights in the imitation kernel
  (mimic_knee_w; the knee's 0.55 rad reference swing now carries 4x
  error weight in the recipe).
- New referee scenario **line_rough** (line walk on the mosaic, random
  tile per seed) joins the loco family; flat-plane claims unchanged.
- Known risk, stated up front: terrain gaits died once before on servo
  torque (terrain_v4 wanted ~3 N*m). If knee-bent rough-ground walking
  exceeds the STS3215 envelope we will see it in watts/falls — that is
  a finding about the robot, not a tuning failure.

**loco_v7knee queued**: warm from loco_v6creep, terrain on,
mimic-knee-w 4.0, creep band kept, entropy 0.0075.

## Day 9 (2026-07-30): first rough-ground results — the knee bends when it must

**loco_v7knee partial (71M of 110M; terrain collision costs 5.6x
throughput, so the 07:00 stop took it): referee 72/88.**

The knee finding, measured: on FLAT ground the gait is still tip-toe
(knee 5.5 deg p2p — flat tip-toe remains optimal there, honestly). On a
ROUGH tile the same policy swings the knee 26 deg — 4.5x — with watts
33-49 in the bumps. **The policy learned terrain-conditional knee use**,
which is the mechanically correct answer to "entice the knee": bend it
when the ground demands clearance, not as a style tax on every step.

Scoreboard: goal_home HOLDS 8/8 (home 5 cm); square_return 8/8 @ 10 cm
(best yet); **stand_off recovered 4/8 -> 7/8** (drift 4.2 cm) --
torque-off standing is viable again on the new body. line_rough (new
claim): initial 1/8 was a MEASUREMENT bug -- the line scenario gated on
absolute world x/y, nonsense under mosaic spawns; made spawn-relative
(identical numbers on the flat plane) -> honest **5/8 at t=2.6 s vs
2.5 s flat, zero falls, watts 12.7**. The robot walks 0-20 mm rough
ground at essentially flat-ground pace. turn_180 dipped 8/8 -> 5/8
(partial training; watch the continuation). Continuation queued for the
22:00 window (finish the step budget at terrain throughput).

## Day 9 cont. (2026-07-30): SIL harness live — and it caught the referee

The software-in-the-loop harness (user request; design docs/sil-harness.md,
built by two opus subagents, integrated same day) runs the REAL firmware
control stack — obs assembler, history ring, gait clock, MLP with exported
weights, calibration/tick conversion, bus-ID permutation — against the CPU
referee plant, scored by the standard scenarios. Everything green; policy
through firmware inference matches brax at 6.6e-7; closed loop within one
seed of the python referee.

**Day-one catch: the referee itself had an obs-stacking bug.** The eval
Driver fed [f_t, f_t, f_{t-1}] (duplicated head frame + redrawn IMU noise)
instead of the training stacking. Fixed; both current policies
re-refereed. Shifts are BOTH directions (the old path was a different
distribution, not a handicap): v6creep 72/88 with line_rough 8/8 — the
flat-trained creep policy already walks 0-20 mm bumps clean — and
v7knee 69/88 with flat line_1m dropping to 3/8 and turn_180 3/8: the
partial knee/terrain policy trades flat precision for its terrain
skills. v7knee_b (tonight) gets to argue the trade back. Historical
scorecards predate the fix and carry that caveat.

Also open (SIL finding #2): 8 obs channels have frozen normalizer std
(~1e-6); cmd[3] crouch is frozen at 1.0 but battguard RAMPS it on a
sagging pack — five orders out of distribution at the worst moment.
Next training recipe should vary the crouch command (ties into task #16
encoder-realism work).

## Day 10 (2026-07-31): v7knee_b wins — then the plant moved under everyone

Overnight `loco_v7knee_b` (60M continuation, terrain + knee-mimic 4.0,
entropy 0.005) finished the full budget in 7.7 h. The morning collect
scored it **12/88, falls 85%** — while training eval reward sat at ~1200.
The tell: the only passing scenario was stand_off, the one that ignores
policy actions. That is a referee/plant mismatch signature, not a bad
policy.

**Root cause: night_arm.sh never synced bimo_biped_v3yaw.xml.** Mira's
copy was the 9fccf4c blob (pre-07-30), so the run trained on the OLD
foot while the local referee scored it on the corrected printed foot
(f58a412: 116x52 rounded-corner sole, 8 contact spheres, replacing the
wrong "as-built 90x46 pad" belief). Cross-check: the parent v7knee —
69/88 yesterday — scores **0/16** on the corrected plant. Every
pre-correction policy falls 100% on the corrected foot; the box→sphere
contact change matters far more than the larger support polygon.
night_arm.sh now syncs `bimo_biped_*.xml` + `terrain_mosaic.npz` with
the code (the gap that let plant and trainer drift for two days).

**Scored on the plant it actually trained on, v7knee_b is the best
policy yet: 75/88** (v6creep 72, v7knee 69). The continuation argued
the trade back exactly as hoped: line_1m 3/8→7/8, turn_180 3/8→7/8,
goal_home 8/8 (home 5 cm), stand_10s 8/8, stand_off 7/8, backward_1m
8/8. Soft spots: circle_return 2/8 (ret 67 cm), line_rough 5/8.

**Knee articulation generalized.** Probe (cmd 0.35 m/s, 8 s, 4 seeds):
knee p2p **~28 deg flat AND ~28.5 deg rough** (v7knee: 5.5 flat / 26
rough). The knee is now part of the everyday gait, not a rough-tile
special — directly answering the standing "I don't ever see the knee
joint moving" concern. Caveat: all of this is on the retired plant.

Tonight (`loco_v8foot`, queued): warm-start v7knee_b on the corrected
foot, same recipe. The 75/88 + knee numbers must be re-earned on the
plant that matches the printed robot; expect early reward collapse then
re-adaptation of contact timing.

## Day 11 (2026-08-01): corrected-foot migration done in one night; rotation is the new hole

The extended GPU window (user: until noon) ran the full chain: v8foot
(60M, corrected foot, old mosaic) finished 00:24; the runner's new
queue re-scan launched v9rough one minute later on freshly pulled code
(rough-only mosaic, --quantize, --cmd-crouch-range 0.7,1.0); full 60M
by 07:53. First runs with git_sha stamped in config.json and with the
standing SIL referee column.

**loco_v8foot, python 67/88 / SIL 68/88** -- the migration worked:
every pre-correction policy scored ~0 on this plant; one warm-started
night recovers line_1m 8/8, stand_10s 8/8, goal_home 7/8 (home 8 cm).
**loco_v9rough, python 63/88 / SIL 63/88, 6/11 clean** -- and it is
the first policy trained the way the robot actually senses: encoder-
quantized obs, tick-quantized targets, varied crouch (normalizer
hazard closed: cmd[3] std is now healthy), no smooth ground anywhere.
stand_off 8/8 at 1.7 cm drift is the best torque-off standing ever
recorded here. Knee articulation held through every change: ~29 deg
p2p flat / ~27.5 deg rough on the corrected plant.

**Rotation collapsed on the corrected foot**: turn_180 2/8 -> 1/8,
circle_return 1/8 -> 0/8 (v7knee_b turned 7/8 on the old foot). The
eight-sphere sole grips differently in yaw; nothing in either recipe
emphasized turning. Tonight: v10turn = v9rough recipe + --turn-emph
(the loco_v5t mechanism: 85% turn episodes, |wz| >= 0.25). goal_home
also dipped for v9rough (3/8 python / 5/8 SIL) -- likely the same
heading-control deficit; re-judge after v10turn.

SIL vs python on marginal LONG-horizon scenarios can differ by 2-3
seeds in either direction (goal_home 7v5 for v8foot, 3v5 for v9rough;
opposite signs, per-tick net_err still 6.6e-7): 24 s of closed loop
compounds the half-tick obs difference and the free-running SIL gait
clock. The closed-loop pytest budget is now 2 seeds for goal_home, 1
elsewhere; a real stack bug would miss in ONE direction everywhere.
SIL reference run re-pointed loco_v6creep -> loco_v8foot (goldens
regenerated; firmware 2959 checks green; 31/31 pytest).
