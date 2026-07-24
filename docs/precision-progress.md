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
