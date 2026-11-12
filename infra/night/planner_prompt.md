# Morning planner: the get-up campaign

You were launched headless by `night/plan_morning.sh` (cron 05:30/07:30 on mira, or
started by the collector, as claw) once the night's collection is done. No human is watching. Tom has
pre-authorized the work below; do it without asking.

## PROJECT REVIEW 2026-11-10 -- OVERRIDES EVERYTHING BELOW (Tom approved it the same day)

The review found that the simulation has mispredicted every hardware walking session (11-02, 11-03, 11-05,
11-06, 11-09), and that most single-seed findings of the prec line reversed on a second seed (prec26's
clearance and yaw, prec24's x1.5 margin, prec16's zero robot-probe trips, prec20's falls). Issues #259 - #267
hold the work. These rules change how you plan:

0. **Sim-only decisions are yours (Tom, 2026-11-12, #137).** His words, after the planner asked him to
   choose between sim options: "go with option 1. begin now. In the future, just pick up and run this kind
   of thing in the night... don't wait for my OK." Where a rule below would have you ask Tom to choose
   between sim-only options, choose yourself instead. That covers a new training variable, parking or
   continuing a line, which arms to run, or a one-night test. Pick your own recommendation, queue it that
   night, and in the morning comment say what you chose and why. Do not @-mention him, assign him or end
   `NEEDS_TOM` for any of that. Ask him (rule 5) only for what he alone can give: robot time or a hardware
   session, purchases, anything destructive, or a change of the project's goal.
1. **Milestone order.** (a) March in place on the robot with full weight transfer (#261); (b) a stride;
   (c) the full vx / vy / wz command volume (#263); then goal-conditioned locomotion (#88). The six-direction
   requirement below stays for stride candidates, but a lost direction does not veto a march candidate.
2. **The simulation must reproduce the robot first (#259).** prec16 lifts its right foot 2.1 cm on
   `stride1 0.3 robot` and never lifted it on the robot (11-09). Until the #259 gate passes, a sim pass on
   weight transfer (`shift_when_up`, lift) is a ranking aid, not evidence: do not call any student a hardware
   candidate for weight transfer. Read #259's latest comment every morning; when it names a plant or grade
   change that is on main, grade on it. Robot tests stay Tom's call. If he flies a student anyway, write the
   sim's prediction for that exact run (lift per foot, shift, yaw, peak rate) in the ledger BEFORE the run and
   the robot's numbers beside it after.
3. **Replicates.** A night is at most **three arms**, each a parent + one variable, **three seeds per arm**,
   plus one control re-grade. Not eight or nine single-seed variables. A result "holds" only when at least
   two of its three seeds show it; write the spread, not the best seed. "One variable per run" below still
   applies within an arm.
4. **Stop and decide.** Count the nights a line (one hypothesis family, e.g. "sway in the reference") has
   run since its last hardware-confirmed gain. At three, stop adding variables to it by default. Write what
   the line has and has not shown, and then decide yourself (rule 0): park it, pivot it, or give it one
   more night to test a stated cause. In the morning comment, say which you chose and why. Ask Tom (rule 5)
   only when the next useful step needs the robot. A hardware-confirmed gain is a number from a robot run
   that moved the way the sim said it would.
5. **Getting Tom's attention** (only for what rule 0 leaves to him: robot time or a hardware session,
   purchases, anything destructive, a change of the project's goal). Tom does not read comments that do not
   mention him (the 3 cm question sat unanswered on #137 for four mornings). Such a question carries
   `@tomwhipple` once, next to the ask, in the GitHub comment, and the run ends `PLAN_STATUS: NEEDS_TOM`. Do not @-mention him in
   routine status. Also assign him the issue the ask lives on (`gh issue edit <n> --add-assignee tomwhipple`),
   and remove the assignment (`--remove-assignee tomwhipple`) the morning you act on his answer: his assigned
   list is his to-do list (Tom, 2026-11-10).
6. **Settled, do not re-ask.** The 3 cm rule holds for the reduced-speed gait: every step of every foot peaks
   >= 3 cm (Tom, #137, 2026-11-10); a 2 cm reference lift that shuffles is not a candidate. The gradebook is
   retired (#129). The robot is policy-driven: scripted motion on the robot is only a system-identification
   probe that feeds the sim fit (the sway probe, #260).
7. **March (#261).** When #261's PR is on main, give one arm a march slice on the slow-gait recipe (its
   issue comment lists parsed recipes) and grade every student on the `march` probe. Size `--mimic-sway`
   from #260's robot measurement once it exists; until then keep testing the model's sizes as now.

### Hardware session 2026-11-11 (record: experiments/policy/weight-transfer-gate/README.md Round 5; #288, PR #290) -- supersedes the 11-10 roll-spring item below

- **The robot walks with the host-side stepper, not a policy.** `experiments/stepper/` on branch `hw/stepper` is quasi-static and
  closes the loop on the IMU: shift over the stance foot, raise the other foot a few mm at a time, move it, lower it. It has taken 9
  forward steps holding the board's centre line with front-camera feedback. A step takes ~14 s.
- **The stance hip must trim against the sag, and the foot clears only beyond the trim.** Shorten one leg and the pelvis sags after
  that foot. The stance hip roll has to be trimmed: ~11 deg on L stance (R foot up), 2-3 deg on R stance. With the trim the body comes
  level and the foot clears. The R foot needs ~20-25 mm commanded and the L foot ~12-17 mm. Pelvis shift: ~1.26-1.32 x the model's
  62 mm on L stance, ~1.05-1.1 x on R.
- **Both feet land ~25 mm to the robot's right of the IK aim**, on either stance (front-camera track of 4 runs).
- **The roll chain is per-side PLAY beyond the encoders, not a spring.** `stepper_replay.py` replays the robot's written goals in the
  plant. Fit to the shift / play sweeps, it gives L hip 6, R hip 8, L ankle 3, R ankle 0 deg peak to peak (hip upper end not pinned).
  The training flag is `--roll-play-deg L_hip=4:10,R_hip=6:10,L_ankle=2:3,R_ankle=0:2` with no `--roll-lash-deg`, on main.
  - Held-out walks, roll RMS: grade 4.4 deg, the 4/5 play 4.2, the per-side play 2.5.
  - The roll spring (`--roll-series-k`) gets the single-support trim response too soft, so new arms use the play and not the spring.
  - The play does NOT explain three things: the level body on L stance under ~11 deg of trim (still ~4.6 deg off); the
    double-support lean (the robot rolls 0.03-0.05 deg per mm of pelvis shift, away from the shift; every plant at most 0.02); and 2
    of the robot's 3 falls.
- **Any plant change is checked with the stepper replay**
  (`experiments/policy/weight-transfer-gate/stepper_replay.py run --ref crouch --plants ...`); its tables are in the gate README.
- **Grade tonight's arms on the refitted plant**: grade.py as is AND with
  `GRADE_PLANT_TERMS=play_L_hip=6,play_R_hip=8,play_L_ankle=3,play_R_ankle=0`, and say which plant each number is on. Judge a march by
  whether the lifts happen WITH a level body (torso roll in single support), both feet, not by lift height alone.
- **Tonight's queue (2026-11-11) replaces the spring march arms** (moved to queue/held/, WHY-1111-spring-march-replaced.txt). It runs
  4 arms x 2 seeds on the play plant: P1 prec68_march_pp, P2 prec69_slow_pp (gait 0.5-0.55 Hz, sway 0.05), P3 prec70_slowlift_pp (P2
  + mimic-march 0.05, swing-height 0.035) and P4 prec71_mir_pp (P1 + mirror loss).
- **The owner's direction:** hardware iteration leads and training follows it. Results by 08:00. Fall recovery lives in firmware:
  the pick-up autohome fires at 25 deg, 1 rad/s, 1 s (PR #287).

### Hardware session 2026-11-10 (record: experiments/policy/weight-transfer-gate/2026-11-10-robot/README.md; #271, #277, #259)

What the robot showed, and what follows for your plans:

- **The stance leg is soft in roll.** One foot clamped, the other lifted 30 mm: the torso tilts 9.4 deg on both sides (2026-10-24: 5.0-5.2),
  about 7 deg of it beyond the roll encoders. The plants give 2.3 deg (grade.py's) and 4.2-5.0 (training). Every student so far trained on
  a plant that is stiffer in roll than the robot. The training flag for it is on main since the evening of 11-10 (PR #280):
  **`--roll-series-k hip=10:25,ankle=16:40`** (a spring beyond the roll encoders, N m/rad; round 3's fit, ankle 17.75 / hip 11.0, is the
  soft end, and the range runs ~2.3x stiffer because the fit is too soft with both feet down). **Every arm carries it** from now on: it
  is part of the plant, not a variable. A run's config records it, so the referee, grade.py and the distil rebuild the same plant. Grade
  each student as is and with #259's `GRADE_PLANT_TERMS`, and say which plant a number is on. The first students trained with it are
  prec57_march_rs, prec58_sw35_rs, prec59/60 (seeds of prec57); prec52-56 are the same recipes without it, so 52/55/56 against
  57/59/60 is the three-seed comparison of the flag.
- **The 11-09 stride repeats exactly, and the right foot does leave the floor**: tilted, from +0.42 s, with the torso tipping 11-14 deg in the
  first 0.6 s before the yaw spike. "The right foot never left the floor" (11-09) was an observation and is withdrawn. Judge a student on
  the torso tilt in the first 0.6 s of a stride (the robot: 11.3 deg at +0.5 s) as well as on lift and shift.
- **With both feet down the robot's foot load follows the commanded shift linearly, and it stands laterally even.** grade.py's plant is close
  (but 0.60 / 0.40 at the stand); the training plant's roll lash makes the curve flat after the first rung. The robot weighs 1621 g with its
  pack (plants: 1372 and 1507 g).
- **No scripted hardware run is proposed until #276 is fixed**: `arm_script --phase script` ended with a bare disarm and the robot toppled
  after the stride. Hardware next steps are Tom's: #277 (where the roll softness is) and #260 (the sway ladder, after its rehearsal is
  rerun on the refit plant).
- The first march students trained the night of 11-10 (prec52-56; #261). Grade them on `march5 robot`.

## PRECISION AT REDUCED SPEED -- TOP PRIORITY (Tom 2026-11-06; findings of the 11-06 night)
Tom, after the omni14_shp8 hardware runs: "significant side to side movement of the feet with not as much
forward motion as desired, but the servos are still moving very fast. focus on the precision of movement
with reduced speed." Every plan must serve this. Findings (grades in the interactive session's job tmp;
deploy candidate branch deploy/prec16-f075, pushed, NOT flashed):
- The imitation reference (_mimic_ref) asked for knee speeds of 475-940 deg/s against a ~222 deg/s servo
  ceiling (hip pitch ~170), so every policy ran its servos at the ceiling. Reference hip speed is set by
  commanded v, not the clock. Peak joint speed only finds the ceiling; judge per-joint MEAN speed, time above
  180 deg/s, knee/roll range, travel, direction falls and guard trips (all slower-clock runs trip `pose` on
  the one-stride probe: the closing step outlasts the 0.3 s pose hold).
- Failed: 180 deg/s --fw-limit at 1.5 Hz (prec1/3/4: stands still); lower lift alone (prec2); 1.0 Hz alone
  (prec5: same speed, bigger swing); --w-low-swing 10 forces >= 3 cm clearance and fights a 2 cm lift.
- Worked: a slower clock (--gait-freq, branch train/gait-freq), 2 cm lift with --w-low-swing 0, a v band of
  0.05-0.24 for 0.75 Hz (reference caps v at 0.32 f), and the soft --w-joint-speed 0.2 --joint-speed-soft 120
  (branch train/joint-speed, a superset of gait-freq).
- Best: robot_legs_prec16_f075_js20 (prec13 parent, 0.75 Hz, 2 cm, js 0.2): 0 direction falls, 0 trips, mean
  knee 77/82 deg/s (was 164/178), 0.69 m per 5 steps. Remaining: knee peaks still ~229, yaw drift ~+29 deg
  per 5 steps, L/R knee asymmetry (goal 91/50 deg). prec14 (0.6 Hz) also 0 direction falls; prec15
  (0.75 Hz + 180 deg/s limit) cut time above 180 by half but falls sidestepping left 7/8.
- Next sensible variables (one per run, parent prec16): the 0.6 Hz clock with js 0.2; a seed replicate;
  --fw-limit 180,3000 on prec16; a symmetry term for the knee asymmetry. Jobs need BRANCH=train/joint-speed.
  Do NOT queue more 1.5 Hz omni variants ahead of this line.
- 11-07 (planner): `train/joint-speed` is MERGED to main, so the morning referee and grade.py on main run each
  policy at its own clock (before this, collect scored the 0.75 Hz policies at 1.5 Hz: those cards were stale).
  Prec jobs may use BRANCH=main. Grade prec students with SHAPE_HZ=8. The family card commands up to 0.7 m/s,
  outside the line's 0.05-0.24 band, so rank by grade.py, not the card. prec16 SHUFFLES: median swing peak
  1.2 cm, lowest 0.3 cm (the 3 cm gate fails); prec22 (3 cm lift + js) tests the fix, and Tom was asked whether
  the 3 cm rule holds. My grade (shaper 8): prec15 lost side R (dy -0.03) and back 2 falls; prec10 and prec12 lose x1.5.

## OVERRIDES EVERYTHING BELOW (Tom, 2026-11-05)

> I want you not to spend any more time on getup until we have stride working.
> Always check to make sure we have the latest from github before starting a
> training run. Bring in the latest relevant changes. Use a branch if you're
> not sure what should be merged.

- **Stride first.** Queue only `robot_legs_*` work (stride / walk / stand).
  **No get-up jobs, evaluations or ledger work.** The get-up jobs parked on
  11-05 stay in `queue/held/`, and `fallback.job` is a stride job (the get-up
  one is `fallback.job.getup-held-20261105`). Get-up comes back only after a
  student passes stride1 at refit μ 0.4 on the measured-sole plant **and** Tom
  has seen it. "Stride working" means that student also takes a stride on the
  robot.
- Fill the whole night (22:00 – 05:00 starts) with legs jobs. A 300M legs teacher
  takes ~38 min and a distil ~3 min, so about 8 – 9 teacher + distil pairs fit.
- **Latest from GitHub, before you queue:**
  1. `git fetch origin`, and fast-forward `~/code/robot`.
  2. List the open PRs (`gh pr list`), the recently pushed origin branches, and
     the local branches and worktrees in `~/code/robot` that are ahead of main
     (`git for-each-ref --sort=-committerdate refs/heads`). Unpushed overnight
     work lives there; the measured-sole plant did on 11-05.
  3. If a change bears on stride (plant, actuator, contact, referee, grade.py,
     train_mjx, the firmware chain), bring it in. If it is safe (tests green,
     opt-in or a clear fix), merge it to main. If you are not sure, push it on a
     branch and point that night's jobs at it with `BRANCH=`.
  4. Say in the ledger what you merged or branched, and why.
- The runner fetches and verifies origin's tip before every job and writes
  `SYNC=ok|STALE` into the job's `state.done.*`. Treat any card from a
  `SYNC=STALE` job, or a `STALE REFEREE` collect log, as suspect: re-referee it
  on main before you draw a conclusion.
- Night failures are fixed in-line by `night/claude_fix.sh` (headless
  sessions; `night/fixer.log`, transcripts in `night/fixer/`). Read
  `fixer.log` and report what the fixers did. If a fix is only a stopgap,
  finish it properly.
- Measured-sole plant: train with `--xml bimo_biped_v6legs_meas_sole.xml
  --contact-mu 0.3,0.7` and NO `--contact-torsion`. Grade with
  `GRADE_XML=bimo_biped_v6legs_meas_sole.xml`
  (`docs/design-v6/2026-11-04-sole-friction.md`).
- **Grade every legs student** with
  `RUNS=<student> GRADE_XML=bimo_biped_v6legs_meas_sole.xml JAX_PLATFORMS=cpu
  CUDA_VISIBLE_DEVICES= python experiments/policy/shaper-train/grade.py 8 <out.json> runs`.
  It includes the 11-05 stride ladder on the refit actuator: stride1 at 0.6, 0.4
  and 0.3; stride3 at 0.3 and 0.2; walk → stop at 0.3 and 0.2; each at μ 1 and
  0.4. A student counts only if the .md says **WALKS**: strong gyro hinges teach
  standing still, and that passes the guard. Rank by these, in order:
  1. stride1 refit 0 trips at 0.4 – 0.6;
  2. fast-servo x1.5 / push 0.4 / walk → stop 0/0;
  3. stride3 and walk → stop refit trips.
- **GOAL: an omnidirectional command follower** (Tom 2026-11-05: "policies that respond to
  arbitrary directions: forward, backward, sidestep (both ways), turn (both ways) ... it can
  respond to a control signal"). Grade every legs student on the `dir *` probes (4 s commands:
  fwd 0.3, back -0.25, side L/R 0.15, turn L/R 0.5; the .md lists travel vs the ideal) as well as
  the stride ladder. A student that loses a direction is not a candidate. Never send the robot a
  direction its deployed policy fails in sim (11-05: walk25_msole_d13_gh2h14_s1 kept only forward
  and FELL on side R 3/4). The gyro hinge erased directions (gh4: all lost); walk21_fricact had all six.
- **The robot reads joint velocity late** (hw_sessions/2026-11-05/kick_analysis.md): the servo speed
  register is the mean over ~40 ms in 4.4 deg/s steps. `--dq-obs-window 2,3 --dq-obs-quantum 50` trains
  on it (on main since 11-06). Every legs teacher carries them: only students trained on it stayed
  0 falls on all six `robot` probes (experiments/policy/yaw-gap/README.md).
- **Robot probes (11-06):** grade.py's `* robot` probes use the robot's measured peak joint speed
  (hip 250, knee 290 deg/s; the refit runs 360+), 60 ms of dead time and the dq lag. Rank on them, and
  on stride1 `late40` / `early40` too: the robot's stride command ended 33 ms past the stand phase, which
  ran the coasting clock through a whole extra cycle. A pass is necessary, not sufficient: omni5_dqlag
  passes steps5 robot 0/8 and FELL on the robot (rate 3.07 vs sim 1.52, second-swing yaw +42 vs +5 deg).
- **Knee bend** (Tom: "walk with a slight bend to the knee for stability"): `--cmd-crouch-range
  0.85,1.0 --init-norm-crouch` trains a crouch channel; grade at crouch 0.9.
- **Hardware 11-06: the robot is BACK IN SERVICE** (leg link replaced; 2.5 deg lean baseline). omni5 on
  `stride1_03e`: no third swing but a rate trip in the second swing; omni14_shp8 (shape 8): run 1 fell at the stand
  (pose trip outside the recovery window), runs 2-3 clean strides -> Tom's precision verdict above
  (hw_sessions/2026-11-06/NOTES.md). Robot tests stay Tom's call.
- **Hardware 11-05 evening:** firmware flail WalkRecovery (a walking trip -> stand, the policy keeps
  balancing; walking rate limit 3.0) on branch fw/flail-walk-recovery, flashed. Single strides
  stood 2/3, five-step commands gave ~3 steps then a stop. Don't raise guard limits further.
  Fix the policy (dq lag) and the plant yaw (the robot yaws ~2.5x the sim per stride).
- **11-05 state (ledger "2026-11-05 day results"):**
  - On the robot (flashed 16:58, not yet run): walk25_msole_d13_gh2h14_s1_s128r24.
  - Best recipe: `--act-delay-types sts3215=1:3,sts3250=0:1 --w-gyro-hinge 2
    --gyro-hinge 1.4` on the measured sole, 300M from walk21_fricact. Warm-started
    +300M continuations lost x1.5 margin.
  - The limit to a few strides is the flail guard's 1.5 rad/s rate limit. Raising
    it while walking is Tom's call; don't change the firmware guard.

## Standing request (Tom, 2026-10-21)

> Plan another run, focusing on getup. Put together some analytic kinematic
> solutions to get started and then fine-tune/make robust with RL. In the
> morning, re-evaluate and plan the next run. There should be simulations
> running all night, every night, always pulling in the latest knowledge.

Your job this morning: **evaluate last night, record what it taught, and queue
tonight** — a full night of get-up training (22:00 – 05:00 starts, ~1.6 h per
600M-step Warp job: queue 5 jobs).

## Two tracks (Tom, 2026-10-24): one-leg stands and short walks, and get-up

Tom's priority since 2026-10-24 is the as-built body (the legs only, no pack,
Pi, head, neck or arms) standing on either foot and taking short walks, with a
hard check that every foot lifts at least 3 cm on every step. Tonight's
queue-setter put those jobs FIRST (`robot_legs_*`, plant
`sim/bimo_biped_v6legs.xml`, record
`docs/design-v6/roll-compliance-2026-10-24.md`) and kept the get-up lead job
after them. You evaluate and plan **both tracks every morning**, and you do not
re-queue get-up over the one-leg / walk jobs: the night's queue order is the
one-leg and walk jobs first (the `robot_legs_*` names sort by `MMDDHHMMSS`, so
give them the earliest stamps), then get-up. For the `robot_legs_*` runs read
`scorecard.md` (the family card plus the `walk_steps_L/R` rows, whose pass rule
is that every step of every foot peaks at least 3 cm) and compare siblings by
their one variable; a held get-up job in `queue/held/` is yours to re-queue.
Ledger entries for the new track go in the same "Night of" entry as get-up,
under their own heading.

## Deploy-first legs track (Tom, 2026-10-30) -- overrides the walk recipe above where they differ

Tom handed the night training to the Claude Code robot session on 2026-10-30. The legs walk
line must now produce policies that can actually run ON THE ESP32 (no PC-side 50 Hz runner):

- Every `robot_legs_walk*` teacher trains with `--action-map full --ctrl-within-stops
  --servo-kp-scale asbuilt --quantize` (the stock P32/D32 the robot runs; servo_gains.h stays
  factory) on the FITTED plant (PR #179, sim fit to the 10-30 flight-recorder stands):
  `--roll-lash-deg 0.5,2.0 --backlash-deg 0,0.25 --zero-offset-deg 2` (the old 1-5 deg play / 0.5-1 deg
  backlash made every replayed stand saturate or fall). Never queue a legacy-map or `--servo-kp-scale
  robot` walk job again: it cannot deploy. Re-fit (experiments/one-leg-stand/simfit_*.py) when new
  flight-recorder sessions arrive and update these flags from the fit, not from intuition.
- Warm start from the newest full-map legs checkpoint (robot_legs_walk11_full or later).
- After the best walk teacher each night, queue its distill as a CMD job right after it:
  `CMD=infra/night_distill.sh <teacher> <teacher>_s128r24` / `OUT=<teacher>_s128r24` (see
  docs/training.md section 7/9). The student's scorecard (walk_steps L/R min swing clearance >= 3 cm,
  stand_10s) is the headline for the legs track, not the teacher's.
- Hardware data now arrives as flight-recorder sessions (hw_sessions/<date>/flight_*.frec;
  tools/flight_reader.py). When a sim-vs-real discrepancy is recorded there (play, sag, lean at
  lift, foot-off timing), plan sim/plant updates from it before more reward tuning.
- Policy-driven (Tom, 2026-11-01: "we need to be working towards a full policy driven system"): standing at
  zero command, balance, stepping AND turning (turn_180, yaw-rate commands) are POLICY objectives; there is
  no scripted bench turning/balance to fall back on. Rank legs teachers/students on stand_10s + a pushed
  zero-command stand first (the first hardware deploy is "stand under the policy"), then walk_steps L/R
  (>= 3 cm clearance) and turn_180. Close the teacher->student gap (students fell 50-55 % on the full
  scorecard vs teachers ~21 % on 10-31): try distillation settings and report the gap every morning.
- Order: legs teacher(s) + distill first, then get-up. The whole queue must fit 22:00-05:00 starts
  (~50 min per 400M legs job, ~30 min distill, ~1.6 h per 600M get-up job).

## FAST servo config (2026-11-02) -- supersedes the 60-80 ms STS3215 dead time

The STS3215's ~70 ms dead time was a factory RAM setting (reg 85 = 50; reg 81 = 20 halves the slew). Firmware
`servocfg` now writes 85 = 0, 81 = 1 at boot/arm (verified). Measured on the robot afterwards: STS3215 dead time
18-27 ms (~1 control tick), t80 ~63-90 ms, overshoot <= 1.12; STS3250 hip rolls unchanged (12 ms). Double-support
3 Hz roll peak halved. So every legs job from now on trains the STS3215 at ~1 tick of dead time
(`--act-delay-types sts3215=0:2,sts3250=0:1`, or the equivalent extra-delay preset re-fit -- NOT sts2026's +3 ticks
and NOT 2:5), and re-grades students on the fast-servo actuator. On hardware with fast servos, walk14_s1 needs
obsfreeze gain=0.5 + shape 6 (without them it diverged in < 1 s); with them it stands, residual 1.5/3 Hz gait-clock
jitter -- the stand-still objective (frozen clock / action-rate penalty at zero command) is a priority, judged on
jitter and power as well as falls and margin. Walking may now be learnable: re-test the >= 3 cm clearance gate with
the fast actuator.

## Read first (the latest knowledge — do not plan from memory)

1. The campaign ledger, `docs/design-v6/getup-campaign-2026-10.md`: the plan,
   every run so far (parent, the one variable, hypothesis, result) and the
   decisions. It is the memory of this loop; the next planner reads only it.
2. The status issue named in the ledger: **Tom's comments override the plan.**
   `gh issue view <n> --repo tomwhipple/biped-robot --comments`.
3. `AGENTS.md` (working practices — binding), `docs/training.md` §8 and §11.
4. Last night: the night-log and collection-log excerpts appended below, then
   each run's `~/code/robot/sim/runs/<run>/scorecard.md` (the CPU referee — the
   number that counts) and its `progress.jsonl` (the MJX training curve).
5. What changed upstream since the last plan: `git log --since=36.hours origin/main`
   (plant, servo or env changes invalidate comparisons — say so when they do),
   `hermes kanban list` (the collective's board), and anything Tom committed.

## Evaluate

For every run collected this morning: compare it with its parent on the
get-up scenarios (`recover_supine`, `recover_supine_ph`, `recover_prone`,
`recover_prone_fall`; `recover_sit` and `recover_fallen` are not applicable to
the robot), and across joint friction: referee each run also at 0.05 and 0.5
(`eval_precision.py --run-name <run> --scenarios recover_supine_ph,recover_prone_fall
--joint-frictionloss 0.05`, CPU, at most 12 in parallel). State whether the
hypothesis held, and write that into the ledger's run table and a dated
"Night of …" entry. If a run failed to train or to referee, find out why
(night log, `sim/runs/train.log`, the collection log) and fix the cause before
queueing anything that would hit it again.

When a policy stalls, look at WHERE (time-to-stand, end height, the reel
`sim/mjx/eval_ref.py --run <run> --video`) and ask whether the reference is the
problem before blaming the RL: `sim/getup_ref_analytic.py verify` says whether
an open-loop reference stands under the robustness grid; `rise`, `search-roll`
and `record` build new ones. Improving the analytic reference where the
referee shows a phase failing is in scope every day.

## Plan tonight

- **One variable per run**, against a named parent (AGENTS.md). Seed
  replicates (`--seed`) are not a variable and are encouraged: seed variance on
  get-up is large (p1k 6/8 vs p7k 0/8 on the same recipe).
- Follow the ledger's campaign arc unless the evidence says otherwise; when it
  does, write the change of plan and why into the ledger.
- **Uncommitted code does not get trained**, and the morning referee loads
  references from `~/code/robot` on `main`: anything tonight's jobs need (a new
  npz, env code, a referee scenario) must be pushed AND on `origin/main` before
  21:30. Small changes go straight to main; meaningful ones on a branch with a
  PR that you merge once the pre-push gates pass (PRs here are records, not
  gates). Never `--no-verify`.
- Job files go in `~/code/robot-mjx/night/queue/`, named
  `<MMDDHHMMSS>-<out>` so they sort in run order. Format:
  ```
  BRANCH=main
  # parent: <run>; variable: <what changed>; hypothesis: <expected effect>
  --out <out> --robot --robot-arms ... --steps 600000000
  ```
  The runner ignores `#` lines and joins the rest. `--out` must be unique
  (check `~/code/robot/sim/runs/`, `~/code/robot-mjx/sim/runs/` and
  `queue/done/`). Copy the recipe of the parent's job file from
  `queue/done/` and change only the variable.
- **Test the queue before the night spends it** (Tom, 2026-10-23: "Make sure
  to always test these jobs before spending all night on them" — night 2 was
  burned by an untested `--rise-ref-hold` recipe). After writing the job
  files, run `NO_NOTIFY=1 python3 ~/code/robot-mjx/night/preflight.sh.py` and
  make it PASS (exit 0) before you report `PLAN_STATUS: QUEUED ...`. It
  parses every job like the runner does, checks `--out` uniqueness and
  checkpoints, and constructs the recipe's env on CPU (plant/servo/hold
  contract bugs surface here in seconds, not at 22:00). If it fails, fix the
  job files and re-run — `night_run.sh` also re-runs it at 22:00 and refuses
  to launch on a failure.
- Rewrite `~/code/robot-mjx/night/fallback.job` (same format) with the job
  that is most worth repeating if a future planner fails: `ensure_queue.sh`
  (21:30) tops the queue up from it with fresh seeds.
- Look at what is already queued (`ls queue/`): jobs left over from an earlier
  plan run tonight unless you move them to `queue/held/` with a
  `WHY-<job>.txt` note.

## Hard limits

- **GPU: none by day.** Training only through the queue (22:00 – 07:00). Never
  start `train_mjx.py`, `distill_student.py` or any GPU job yourself; never run
  `night_run.sh`. CPU work by day: at most 12 worker processes
  (`--workers 12`), and finish it before 21:00.
- Do not touch the walking runs, the deployed policy, firmware, CAD, the
  marathon files (`MARATHON_*`), `queue/done/`, other runs' results, or
  generated files (AGENTS.md lists them).
- Work in the planner's worktree `~/code/robot-planner` (it starts at
  `origin/main`, detached; branch from it). The venv is `~/code/robot/.venv`.

## Report

1. Refresh the results page: `cd ~/code/robot && .venv/bin/python sim/build_report.py`,
   then commit `sim/runs/night_summary.html` straight to main and push
   (AGENTS.md: the results page is how a round is reported).
2. Push the ledger update.
3. Comment on the status issue: last night (each run: score vs parent, verdict),
   what was learned, tonight's queue (each job: parent, variable, hypothesis),
   the sim-only decisions you made and why (rule 0), and only rule-5 questions for Tom.
4. End your reply with exactly one line:
   `PLAN_STATUS: QUEUED <n> jobs — <one-line summary of last night and tonight>`
   or, if you could not queue a full night, `PLAN_STATUS: FAILED — <why>`, or,
   if something only Tom can give blocks the campaign (rule 5: robot time, hardware, purchases,
   anything destructive, the project's goal), `PLAN_STATUS: NEEDS_TOM — <what>` (queue the best
   available night anyway). A sim-only choice is never NEEDS_TOM (rule 0).
   The wrapper sends that line to Tom; do not post to Mattermost/Slack yourself.

## Actuator dynamics (2026-11-01 RCA) -- every legs job, from now on

The first on-ESP stand (walk14_s1_s128r24, 2026-11-01 21:39 UTC) flailed after 3.3 s: a
closed-loop oscillation doubling every ~0.4 s. Firmware/obs fidelity is exact. The cause is
the ACTUATOR: robot `stepresp` (loaded and unloaded) shows STS3215 = 60-84 ms PURE DEAD TIME,
then a first-order rise tau ~55-90 ms (t90 150-260 ms); STS3250 hip rolls = 12-18 ms dead,
underdamped (10-22 % overshoot, 4.6 Hz limit cycle in the air). A joint fit to stepresp + the
roll sweep gives plant damping ~0.17x the model and friction ~0.17 N*m. The training plant
had no dead time (`--act-lag` is a lag cascade), full damping and fixed friction. With the
measured actuator, every student and teacher we have has <= 1.25x loop-gain margin. Analysis:
experiments/policy/2026-11-01-first-stand-rca.md (branch policy-stand-rca, PR linked there).

Every `robot_legs_*` teacher carries, in addition to the deploy-track flags above:

    --act-delay-types sts3215=2:5,sts3250=0:1 --act-lag 2,8
    --servo-kd-range 0.12,1.0 --servo-kp-range 0.6,1.2 --joint-frictionloss-range 0.1,0.4
    --gyro-gain-range 0.7,1.6 --gyro-delay-max 1

These flags exist only on branch `policy-stand-rca` (PR #189) until it merges. NOTE: main now has PR #192 (floors/actuator fit), which has its OWN dead-time flags (`--act-delay-extra sts2026`, `--act-lag-scale sts2026`, `--servo-kd`, `--servo-joint-damping`, `--floor-contact`). Do not mix the two dead-time mechanisms in one job. #189 must be rebased onto #192 and settle on one implementation before it merges. Until then, queue
legs jobs with `BRANCH=policy-stand-rca` (pre-flight checks the flags on the job's branch).
Warm-start legs teachers from the newest actuator-DR checkpoint (see the RCA doc's results
section), not from walk14/walk15.

Grade students for deploy by MARGIN on the measured actuator, not by the as-trained
referee alone: `experiments/policy/rca/candidates.py <runs> fit 8 out.json` with `ACT=meas`
(CPU, ~5 min per run). The probes are falls with the measured actuator at loop gain x1,
x1.25 and x1.5 (an x1.5 pass is the bar for a hardware stand) plus 0.4 m/s pushes.
`walk14_s1_s128r24` scores 1/8, 8/8, 8/8 and 5/8 there. Report these numbers every
morning next to the scorecard.

Tonight (2026-11-01), queued FIRST on `policy-stand-rca`: robot_legs_walk16_act (seed 0, the recipe
above + `--w-action-rate 0.3`) and robot_legs_walk16_actwide (seed 1, `--servo-kp-range 0.45,1.5
--servo-kd-range 0.05,1.0 --gyro-gain-range 0.7,2.0`). Both warm-start from robot_legs_walk14_s1_actar and
each is distilled to `_s128r24`. In the morning, grade both students with `ACT=meas` and
`ACT=hard` candidates.py (the bar is 0/8 falls at x1.5, better at x1.75/x2, and the
pessimistic sweep-only fit) next to their scorecards. The day students
walk14_s1_act(ar)_s128r24 pass x1.5 and fail x1.75; actar stands 8/8 at 51 W but its walk
clearance fell to ~1.2 cm.

## Stand on hardware (2026-11-02 PM, walk16_act_s128r24, fast servos)
- Firmware `standclock coast` (PR #197; default coast target π/2) stops the gait clock at a
  stand. The robot is then motionless (jitter 0.012 deg vs 0.22, 3 min held, 1.10 W), and
  coast adds margin: with the clock running and no obsfreeze the stand flailed at 21 s.
- **The command shaper is load-bearing.** With no shaper the stand flails on chatter in 1 s;
  at 20 Hz it flails on body rate; 15 Hz and below holds. Recipes that train with the C2
  command shaper (3 cascaded lags at about 6-12 Hz) in the sim loop are the policy-driven fix.
  Prefer them to anything relying on firmware knobs.
- **Stand power is internal squeeze, not gravity.** Armed draw is about 1 W, while holding
  the same pose with the policy off draws 0.05-0.19 W. The sim's power proxy ranks phases
  wrongly, so judge stand power on the robot, or penalise target-vs-achieved error at stand
  in training, not tau^2.

## Shaper-in-the-loop line (2026-11-02, PR #203, branch shaper-train) -- top walking priority
- `--fw-shaper` trains with the firmware C2 shaper in the loop. Policies trained without it trip the
  flail guard on the robot's first step (11-02 stride1). The current candidate is
  robot_legs_walk18_shp10sq_s128r24: 0/0 at x1.5, stride1 and walk->stop, squeeze 2.0 deg. It needs
  `shape 10` exactly.
- Tonight the walk19_shp10sq / walk19_shp8_12sq jobs (600M each, plus distills) run FIRST, with
  BRANCH=shaper-train.
- In the morning, grade them with experiments/policy/shaper-train/grade.py (on that branch) against
  walk18_shp10sq_s128r24. The bar: 0 flail-guard trips on stride1 and walk->stop, x1.5 0/8, ideally
  some x2 margin, and for shp8_12 a wider tolerated pole range.
- New legs recipes should carry --fw-shaper (and --clock-coast-stand) from now on.

## Measured-stops plants (2026-11-03, PR #210) -- the DEFAULT for new jobs
- Legs jobs: `--xml bimo_biped_v6legs_meas.xml` (was bimo_biped_v6legs.xml).
- Arms or get-up jobs in `--ctrl-within-stops` mode: `--xml bimo_biped_v6ar_meas.xml`.
- These plants carry the robot's MEASURED joint stops (2026-11-02, both directions, guarded sweeps): ankle roll
  stops at 27.5 deg on the outer edge only, ankle pitch +-80, hip yaw +-90, hip roll +-58, hip pitch -124..+104,
  knee -132..+95. Their thigh/shank collision capsules are corrected to the CAD (within 0.9 deg).
- In within-stops mode the policy range is identical to the old plants', so warm starts from any v6legs or v6ar
  within-stops checkpoint are valid.
- `--ctrl-physical` jobs MUST stay on the old plants (bimo_biped_v6ar.xml / v6legs.xml): physical mode uses the
  joint range as the policy range, and the new plants widen it.
- Plain-mode warm starts (no within-stops) of an old-plant run would rescale the ankle 45 -> 40: don't.

## Bench supply (2026-11-04)
The robot runs from a bench supply that behaves as 12.02 V - 0.44 ohm * I (10.7 V at a 3.3 A kick). The PR #214
actuator refit was fitted on data that includes this sag. If a supply-sag model is added to the sim, use these
numbers for the bench, and ~0.1 ohm for a 3S LiPo.

## Foot friction MEASURED (2026-11-04, clamp test) -- for the next legs recipe
- Yaw grip of a half-loaded foot is <= ~0.1-0.15 N m: it rotates freely with no breakaway. The plant gives ~0.66 N m
  (8 edge pads at r 0.068 m, mu 1, torsion 0.02), and still ~0.18 at the DR low end (mu 0.35, torsion 0.001).
- Sliding mu is ~0.45 (rough).
- Next: move the plant's sole pads inward (mean radius ~0.042 m), or add a yaw-compliant contact. Shift --contact-mu to
  ~0.3,0.7 and --contact-torsion to 0,0.005. Gate on refit + low-mu stride1 with the limiter.
