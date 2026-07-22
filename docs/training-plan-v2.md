# Training plan v2 — informed by prior art (2026-07-20)

**Trigger:** "Use these findings to do more research so we're not re-inventing
the wheel. Update our plans. Don't run simulations on techniques we think
will end up changing." Three deep-dives (MuJoCo Playground source extraction;
LocoMuJoCo/AMP + get-up literature; small-biped sim-to-real survey) — full
reports preserved in the session record; key numbers inline below.

## Headline conclusions

1. **Open Duck Mini is our proven twin** — same STS3215 servos, same
   MJX+brax stack, working sim-to-real (ONNX on a Pi Zero 2W). Its gait is
   NOT AMP and NOT mocap: a **procedural walk engine generates reference
   joint trajectories, and the policy earns a phase-locked imitation reward**
   (joint-pos tracking w=15 inside an imitation term, command tracking
   dominant: lin 2.5 / ang 6.0 / alive 20 / imitation 1.0). That is the
   lowest-risk route to natural gait on our morphology.
2. **Our IMU-only observation is the field standard.** None of the four
   surveyed working bipeds estimate linear velocity onboard; all are
   command-conditioned. The "odometry gap" dissolves for locomotion —
   replace with 3–5-frame observation history. Linear velocity/height stay
   as *privileged critic-only* signals (Playground's asymmetric-critic
   pattern — we currently zero them for both actor and critic).
3. **Playground's biped recipes supply exact reward terms we lack:**
   gait-phase clock reward (`exp(-Σ(foot_z − rz(phase))²/0.01)`, w=1.0,
   freq U(1.25,1.75) Hz), feet_slip −0.25, feet_clearance, quadratic
   orientation −1.0 + ang_vel_xy −0.15, pose regularization −1.0,
   dof_pos_limits −1.0, feet_distance −1.0 (principled version of our
   foot-cross guard). PPO: entropy 5e-3 (half ours), nets (512,256,128),
   8192 envs (we fit 1024–2048 on 12 GB), velocity-kick pushes.
4. **Get-up literature (HumanUP, utra unified-getup):** we're missing a
   head-height objective, joint-velocity + action-rate penalties during the
   rise, and the decisive deployability trick — **two-stage: discover a
   fast get-up, then track it temporally stretched (~8 s) under escalating
   smoothness penalties**.
5. **Servo truth transfer:** Open Duck published BAM-identified STS3215@7.4V
   parameters (damping, kp, frictionloss, armature, forcerange). Cross-check
   our fitted kp=12/kd=0.25/damping=0.1; adopt frictionloss + armature (and
   their DR) which we don't model.
6. **What we keep (validated as ahead of the field):** honest torque-speed
   envelope + backlash + sub-step latency + IMU-error DR (richer than all
   surveyed projects), dense directional progress bootstrap, skill-compliance
   gating, as-built plant discipline, CPU-referee standing rule.

## Phases

**Phase A — reward/obs re-architecture (implement first; one overnight run).**
From-scratch `precision_v7`: gait-phase clock obs (cos/sin at gait freq,
replacing episode-fraction phase) + phase reward; feet_slip/clearance;
quadratic orientation + ang_vel_xy; pose + dof-limit regularization;
feet_distance replacing foot_cross; velocity-kick pushes; 3-frame obs
history; frictionloss/armature DR; entropy 5e-3; wider nets. Obs layout
changes → warm-starts intentionally broken. All prior parity gates + new
ones must pass before it trains.

**Phase B — procedural gait imitation + get-up upgrade.**
Reference-trajectory generator for our 8-DOF legs (sinusoidal/Placo-lite,
swept over commands + phase) + Open-Duck-style mimic reward. Get-up:
head-height term, rise jerk penalties, then the HumanUP two-stage recipe.
**The 116 mm feet sit→stand comparison runs here** — after the recovery
reward is literature-informed, so its result stays valid.

**Phase C — as needed.** Asymmetric privileged critic (brax value_obs_key);
AMP via LocoMuJoCo's separable discriminator (~30 lines + expert-data
plumbing, add R1) only if procedural mimic still looks robotic; CrossQ for
get-up if PPO stalls.

**Sim-to-real prep (parallel, no GPU):** study `Open_Duck_Mini_Runtime`
line-by-line (bus timing, IMU calibration offsets, ONNX loop); validate the
ESP32 closes a deterministic 50 Hz loop; per-servo zero-point calibration
procedure; slicer-mass cross-check (already our practice).

## What this retires

- Nightly continuations of the v5/v6 line (hand-tuned gait shaping as the
  primary gait driver) — superseded by Phase A+B.
- The pre-research feet-comparison runs (would have tested sit→stand under
  a recovery reward we now know is missing decisive terms).
- Any onboard linear-velocity estimator work for locomotion.

## Amendment (2026-07-22, progress review): specialist policies

Adopted after the user's progress review: the single do-everything policy
is split into THREE specialists sharing one env/obs format (deployment
switches on command type): **loco** (walk/backward/sidestep/turn/pivot/
stand, with the Phase B procedural-gait mimic reward), **skills**
(balance/circles/march/sway/crouch), **getup** (recovery-only + rise-jerk
penalty). Nights now run 3-4 ~110M diagnostic experiments instead of one
400M shot; scorecards auto-filter to each family's scenarios. precision_v7b
remains the best unified policy (47/128) and warm-starts the skills/getup
specialists. First specialist night: loco_v1 (mimic) vs loco_v1_ctrl
(no-mimic ablation), skills_v1, getup_v1.
