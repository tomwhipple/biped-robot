# v6 body: a biped that can stand on one foot — sim-validated design spec (2026-09-13)

**Status: kinematic design validated in simulation through Gates A–D; awaiting Tom's sign-off before CAD.** No part has been drawn, printed or ordered. Everything below is reproducible from `sim/gen_plant_v6.py`, `sim/design_gates.py` and `sim/static_gait.py`; the raw tables are in `docs/design-v6/`.

Goal (Tom, 2026-09-13): *a new physical robot design able to walk by lifting one foot completely off the ground*; consider a backward-bent knee, different or more servos; no GoPro; room for an onboard Raspberry Pi + camera later.

## 0. The design in one screen

| item | v5 (current, broken) | **v6 (this spec)** |
|---|---|---|
| DOF per leg | 5: hip yaw, hip roll, hip pitch, knee, ankle pitch | **6: + ankle roll** |
| servos | 10 × STS3215 | **6 × STS3215 (hip yaw, hip pitch, ankle pitch) + 6 × STS3250 (hip roll, ankle roll, knee)**; same case, drop-in. Minimum viable: STS3250 at the 4 roll joints only |
| hip separation | 66 mm | **84 mm** (24 mm gap between the feet; a 32 mm battery channel between the yaw servos) |
| thigh / shank | 90 / 90 mm | **110 / 110 mm** |
| ankle | pitch axis 18 mm above the sole | **roll axis 18 mm, pitch axis 50 mm above the sole** (32 mm ankle block) |
| foot | 116 × 52 mm rigid + silicone | **130 × 60 mm, 75 mm toe / 55 mm heel, TPU sole** |
| knee | forward (human) | **forward** — the backward knee was evaluated and gives nothing (§4.6) |
| height | 0.34 m to the deck | **0.41 m to the deck** (~0.43 m with the Pi bay) |
| mass (estimate) | 1.08 kg (1.23 with GoPro) | **≈ 1.25 kg** (servos 0.66 + 0.12 for the six STS3250, print ≈ 0.40, battery 0.10, board + wiring 0.06) |
| standing CoM | 0.18 m | 0.21 m |
| static single-foot stance | **does not exist** (Gate A fail, measured 09-13) | **exists with 30 mm of margin**, reached by a 14° hip/ankle roll shift with both soles flat |
| open-loop walk in the deploy-model plant | 1 cm shuffle, feet never leave the ground | **6–12 steps of 6 cm, swing foot 22–25 mm off the ground for 0.7 s per step, CoM margin ≥ 20 mm, μ 0.3–1.0, ±15 % mass, 3° roll play; arcs at up to 20° of heading per step** |

Cost of the servo change: 6 × STS3250 at ~$25–30 = **≈ $150–180**; the 12 STS3215 on hand cover the other six joints with six spares.

## 1. What the record says, and what it asks of a new body

Read for this design: `lessons-learned-2026-09-13-walking.md`, `design-stage-simulation-gates.md`, `hw_sessions/2026-09-13/notes.md`, `hip-yaw-study.md`, `servo-map.md`, `DESIGN.md` §2/§6 and the dated log, the BOM and camera docs, `cad/dimensions.py`, `sim/bimo_biped_v5body.xml`, `walker_env.py`'s servo model. The requirements that fall out, each traceable to a measurement:

- **R1 — a static single-foot stance must exist** (Gate A). v5 has none: with no ankle roll a planted foot pins the pelvis level, the roll servos slide the feet (loads 88–168) or stall (1.2° short at load 120), and shortening a leg tips the body *toward* the lifted side (8.0° = atan(1.1 cm / 8 cm)). The only regime was a dynamic edge-rock with no margin; the attempt put the robot off the table.
- **R2 — margins measured on the body, before any policy** (process changes §5 of the lessons doc): bench before policy; calibrate the sim where the question lives; dynamic tests need a per-attempt go. This spec is Gates A–D on the kinematics and the deploy servo model; Gate E (a policy) is deliberately last.
- **R3 — the roll chains must be stiff.** Measured on v5: 1–2° of play in the yaw/roll chains, 2–3° free at the hip roll by tilt hysteresis, horn screws that work loose, a right ankle that lurches on load reversal, 2–3° of hardware-only adduction at the stand. Everything lateral rides on these joints.
- **R4 — the actuator must have speed margin under load**: a "200 ms" pose took 600 ms on the bench (peak 130°/s = ⅓ of the no-load speed); Gate B asks ≥ 2× on speed and ≥ 1.5× on torque at 11.1 V.
- **R5 — contact realism made adversarial**: μ swept 0.3–1.0, self-collision on for every pair that can touch, play modelled as free travel, the 2 Hz actuation shaper and 80 ms dead time in the loop.
- **R6 — no GoPro** (154 g at 56 mm over the deck was "the worst item on the machine for balance"); **reserve a Pi + camera volume** on the deck front, mass covered by payload DR (Tom: volume only, no mass budget).
- **R7 — keep what works**: hip yaw (decisive for turning, 7/8 vs 1/8 on turn_180), the pelvis-side yaw/roll/pitch hip stack, the leg_link "servo is the axle" pattern, one-print torso, the 50 Hz ESP32 loop, the General Driver board, the 3S pack.
- **R8 — the robot may grow to ~45 cm and a bigger pack** (Tom).

## 2. Kinematics

Chain per leg, pelvis down (+X forward, +Y left, sim conventions of `bimo_biped_v5body.xml`):

| joint | axis | range | where the servo body sits | servo |
|---|---|---|---|---|
| hip yaw | Z | ±45° | under the deck, horn down (v5 carrier) | STS3215 |
| hip roll | X | 30° adduction / 45° abduction | in the yaw carrier, output +X (v5) | **STS3250** |
| hip pitch | Y | −110 … +90° | thigh top (v5 yoke pair, 50 mm below the roll axis) | STS3215 |
| knee | −Y | −95 … +5° (flexion negative) | shin top (v5 leg_link) | **STS3250** |
| ankle pitch | Y | ±45° | ankle link, case *rising* above the axis between the shin's fork tines | STS3215 |
| ankle roll | X | ±30° | **lying across the foot, output axis fore-aft**, case bottom on the sole plate; the ankle link forks onto horn (front) and idler (rear) | **STS3250** |

Axis stack above the sole bottom: ankle roll 18.4 → ankle pitch 50.4 → knee 160.4 → hip pitch 270.4 → hip roll 320.4 → hip yaw 361.4 → deck top 409.4 mm. Hip separation 84 mm; feet 60 wide → 24 mm between the soles at the neutral stance.

The ankle block reuses the leg pattern: a `leg_link`-like part that grips the ankle-pitch servo case and forks in X onto the ankle-roll servo in the foot; the foot is a re-pocketed v3 foot with the servo turned 90° in yaw. Both roll joints are closed on both sides (horn + idler) — see R3.

**Mass model** (`gen_plant_v6.py`, estimates until CAD): every servo as its real 45.2 × 24.7 × 34.7 mm case at its real place in the chain (this is 55–60 % of the robot and dominates the inertia), printed parts as lumps at the masses today's parts weigh (`cad/PRINT_LIST.md`), battery 100 g in the channel between the yaw servos, board 30 g vertical on the aft wall, 30 g of wiring, Pi bay 65 × 30 × 16 mm at x +28 mm on the deck front with **mass 0** (payload DR covers 0–120 g). Total 1.251 kg, CoM 0.212 m. When the CAD exists, `sim/build_v2_inertia.py` replaces the lumps, as it did for v5.

## 3. Why these numbers (the sweeps)

`design_gates.py --sweep` over hip separation {70, 84, 96} × foot width {56, 64, 72} × leg {90, 100, 110} × knee {fwd, bwd} (`docs/design-v6/gateA_geometry_sweep.txt`):

- **The one-foot CoM margin is exactly half the foot width**, whatever the hips do (the lateral shift is solved to the sole centreline). Fore-aft never limits. So foot width is the margin budget, and its ceiling is `hip_sep − gap`.
- **Narrower hips cut roll torque**: torque margin 3.4× at 70 mm, 2.7× at 84 mm, **1.0× (stalled) at 96 mm** on the STS3215. The required hip roll goes 12° → 14° → 19°.
- **Longer legs buy joint speed**: the swing knee's speed margin on the STS3215 is 1.1× at 90 mm legs, 1.37× at 100, 1.64× at 110 for the same 6 cm step; longer levers need lower joint rates.
- **The gap between the feet is not free**: at 72 mm hips / 60 mm feet (12 mm gap) the walk failed with a 4 cm lift because the hanging swing leg sags 13–15 mm inward and the foot landed *on* the stance foot's inner edge (`L_col_foot × R_col_foot` contacts in the trace). 84 mm gives 24 mm of gap, the same 30 mm margin, and room for the battery low between the yaw servos.

Hence 84 / 60 / 110. A 45 cm robot (R8) would be legs 125–130 mm; the gates scale monotonically in that direction (more speed margin, a little less torque margin) and it was not needed to pass.

## 4. Validation

### 4.1 Gate A — kinematic capability map (no dynamics, no policy)

`design_gates.py` on the default design:

| check | result |
|---|---|
| A1 lateral shift, both soles flat | pelvis 72 mm over, hip roll −14.4° / ankle roll +14.4° (parallelogram), CoM on the left sole centreline with **30.0 mm** margin; double-support margin ≥ 30 mm along the whole path; all joints inside limits; no self-contact |
| A2 lift the right foot 30 mm at that shift, then swing it 60 mm forward | margin 30.0 mm throughout, clearance 30 mm, joints inside limits (knee −68…−73°), no contact; a 50 mm lift also fits (knee −88°) |
| A3 best single-foot margin over a 5-D grid | 30.0 mm = the sole half-width (no better pose exists; the design is margin-limited by foot width, as intended) |
| A4 split stance, feet 6 cm apart fore-aft | hull margin 55 mm |
| A5 level-sole crouch | 62 mm of pelvis drop inside the limits (v5: 40 mm) |

**Verdict: PASS.** Compare v5: "no such path exists".

### 4.2 Gate B — actuator envelope at the design cadence

One step (1.6 s shift, 1.6 s sine-arc swing of 6 cm × 3 cm, 0.5 s landing settle) under the walker_env servo model: PD clamped to the measured STS3215 torque–speed line (2.94 N·m, 4.04 rad/s at 12 V, scaled to 11.1 V) and, for the STS3250, its datasheet (50 kg·cm, 0.133 s/60°) derated by the same 0.86 no-load factor the 3215 measured. Per joint, peak demanded speed and torque against the servo's no-load speed and stall (`docs/design-v6/gateB_mixed.txt`):

| joint | w_pk rad/s | τ_pk N·m | speed margin 3215 | torque margin 3215 | speed / torque margin 3250 |
|---|---|---|---|---|---|
| hip roll (stance) | 0.67 | 0.79 | 5.6× | 3.4× | 9.3× / 5.7× |
| ankle roll (stance) | 0.67 | 0.51 | 5.6× | 5.3× | 9.3× / 8.9× |
| hip pitch (swing) | 1.24 | 0.55 | 3.0× | 4.9× | — |
| **knee (swing)** | **2.44** | 0.88 | **1.5×** | 3.1× | **2.6× / 5.2×** |
| ankle pitch (swing) | 1.42 | 0.59 | 2.6× | 4.6× | — |
| hip yaw | 0.09 | 0.11 | 43× | 24× | — |

No joint saturates at the design cadence. The one joint short of the 2× speed rule on the STS3215 is the swing knee; the STS3250 clears it. **Verdict: PASS with STS3250 knees, FAIL (knee 1.5×) on STS3215 everywhere.** Twice the cadence is not a servo question: a 0.4 s lateral shift of 72 mm is 0.27 g and moves the zero-moment point outside the sole — the quasi-static gait's cadence floor is set by the body, at about 1.2 s per shift (§4.4).

### 4.3 Gate C/D — the open-loop walk under the deploy model, made adversarial

`static_gait.py`: the Gate A path made periodic — shift until the whole-robot CoM (solved on the kinematics with the swing foot at mid-swing) is 5 mm *inboard* of the stance sole centreline, swing the other foot on a sine arc, settle 0.5 s, cross over. Joint targets from the analytic IK at 50 Hz. Servo model as deployed: kp 12 N·m/rad, torque–speed clamp, **2 Hz three-stage shaper + 80 ms dead time, integer goal ticks, 5 ms bus latency, 1° gear backlash, 3° free play on the four roll joints**, μ 0.7. Step 6 cm, lift 4 cm, 37.6 s for six steps (a slow, static gait by construction — 1.6 cm/s).

Final configuration (STS3250 at rolls + knees), worst of 3 seeds per case (`docs/design-v6/gateCD_sweep.txt`):

| case | steps | swing foot off the ground | CoM margin ≥ | stance slip | verdict |
|---|---|---|---|---|---|
| nominal | 6/6 | 25 mm peak, 0.74 s/step | 25.5 mm | 1.3 mm | OK |
| μ 0.3 / 0.5 / 1.0 | 6/6 each | 25 mm | 19.6–26.0 mm | ≤ 1.3 mm | OK |
| no play, no backlash | 6/6 | 31 mm, 0.92 s | 21.8 mm | 0.4 mm | OK |
| mass × 0.85 / × 1.15 | 6/6 | 24–26 mm | 13.5 / 19.6 mm | 1.3 mm | OK |
| payload 60 g on the deck front | 6/6 | 25 mm | 24.2 mm | 1.5 mm | OK |
| payload 120 g, gait unaware of it | fell at step 2 | | | | FAIL |
| payload 120 g, gait solved with it in the model | 6/6 | 24 mm | 25.4 mm | 1.2 mm | OK |
| servos −15 % / −30 % (stall and speed) | 6/6 | 25 mm | 19.7 / 25.5 mm | 1.3 mm | OK |
| floor tilt ±2° fore-aft | 6/6 | 24–25 mm | 19.6–25.4 mm | 1.6 mm | OK |
| 12 steps; 8 steps in place | 12/12; 8/8 | 25 / 26 mm | 26 mm | 1.0 mm | OK |
| cadence 1.2 s shift / 1.2 s swing (30 s per 6 steps) | 6/6 | 22 mm | 25.8 mm | 1.3 mm | OK |
| **arc walk**, 10° / 15° / 20° of heading per step, 8 steps, both directions | 8/8 each | 23–25 mm | 24.9–25.5 mm | ≤ 1.5 mm | OK — heading change 65° / 99° / 134° (§4.7) |
| **play 5° on the rolls** | fell in the first shift | | | | **FAIL** |
| **backlash 2°** | fell after step 1 | | | | **FAIL** |
| cadence 0.8 s shift / 1.0 s swing | fell at step 3 | | | | FAIL (body dynamics) |
| shaper and dead time removed | fell at step 4 | | | | FAIL — the gait is timed for the deployed shaper; re-time before running it on a firmware without the shaper |

**Verdict: PASS** for the deployed chain with the requirements in §5. The boundaries are informative: the walk tolerates the measured 3° of roll play but not 5°, 1° of backlash but not 2°, and a 60 g Pi payload without telling the controller but a 120 g one only with the payload in the model.

Filmstrip (2 × 4 frames across steps two and three; the video is `sim/renders/v6_static_walk.mp4`, gitignored, regenerate with `static_gait.py --render`):

![v6 static walk](../sim/renders/v6_static_walk_strip.png)

### 4.4 What the dynamic plant taught that the kinematics could not

Each of these was a fall in the trace before it was a design decision; all are in `docs/design-v6/gateD_*.txt`.

1. **Roll compliance is the whole game.** Under the fitted 12 N·m/rad (bench: 1.2° short at 0.35 N·m ≈ 17 N·m/rad, so the fit is on the soft side of real) the stance hip roll sags 4° and the ankle roll 2° under 0.65 N·m in single support: the pelvis drifts 25 mm toward the swing side, and when the swing foot lands the closed chain springs it 35 mm the other way. With the STS3215 everywhere the walk survived only at 72 mm hips with a 3 cm lift and zero play (`gateD_lift_matrix_3215.txt`). **With the STS3250 at the four roll joints (≈ 4× the static stiffness, third-party bench: 1.2° under 1.96 N·m vs 2.6° under 0.98 N·m for the 3215) every configuration walks** (`gateD_servo_matrix.txt`). Sensitivity: a 3× stiffness ratio still passes with 3° of play; at 2× only 1° of play passes (`gateD_knee_kp_cadence.txt`). **This is the single assumption to measure before ordering six** (§5).
2. **The swing leg sags inward while it hangs** (12–15 mm at the foot) and would land on the stance foot at a 12 mm gap; hence 24 mm of gap, a 10 mm outward bump at mid-swing and a 12 mm outboard landing offset that relaxes once the foot is loaded (`land_out`).
3. **A step command is a fall.** Commanding the working crouch from the straight stand as a step pitched the body 25° forward: hips and ankles arrive first, the knee (twice the travel) last. Every transition is a min-jerk profile; the start-up eases into the crouch over 1.5 s.
4. **Timing must respect the shaper.** The swing leg lags its command by ~0.3 s through the 2 Hz shaper and dead time; a 0.5 s landing settle before the next shift is what separates 6/6 from a fall at the crossover. A 4 cm commanded lift yields 22–25 mm of real clearance — the rest is sag and lag — which is why the commanded lift is 4 cm for a 2 cm requirement.
5. **Aim the CoM slightly inboard.** With the CoM exactly over the stance ankle-roll axis the body is in neutral equilibrium and any drift is outward, over the sole's edge and away from the other foot. 5 mm inboard makes the failure direction the safe one (toward the incoming foot); the margin cost is 5 of 30 mm.
6. **The shift time scales with distance.** The crossover from one stance to the other is twice the first shift; commanding it in the same 0.8 s put 0.27 g of lateral acceleration into a body on soft roll joints and it overshot by 40 mm. 1.6 s per 60 mm (2 s per crossover) is the design cadence; 1.2 s still passes; 0.8 s does not.

### 4.5 Servo choice

- **STS3215 (on hand, 12)**: passes Gate A trivially (kinematics), Gate B on every joint but the swing knee (1.5× speed), and Gate D only in the no-play, narrow-hip corner. Its softness under load, not its torque, is what fails the walk.
- **STS3250 (drop-in: identical 45.2 × 24.7 × 35 mm case and horn/idler pattern, 74.5 g vs 55 g, 50 kg·cm, 7.9 rad/s no-load, measured backlash 0.43° vs 0.87°, ~$25–30)**: at the four roll joints it turns every Gate D case green; at the knees it clears the Gate B speed rule (2.6×) and adds a few mm of swing clearance. Recommendation: **six STS3250 (rolls + knees), six STS3215 (yaws, hip pitch, ankle pitch)**. Minimum viable: four STS3250 at the rolls and a 1.6–1.8 s swing on STS3215 knees (1.8× speed margin, just short of the rule).
- Mass cost of six STS3250: +117 g (1.25 → ~1.37 kg incl. the heavier feet); Gate D at mass × 1.15 passes with 19.6 mm of margin.
- **A different family (Dynamixel XL430/XC430, ~$50–70 each, new bus and board)** is not needed by any gate and is Tom's last resort; not pursued. **Two more DOF (arms, a waist)** would not change Gates A–D and were left out; the deck keeps M3 bosses for a later arm or head.
- Power: 12 servos on the General Driver's 5 A-rated bus path is the same unresolved item as today (peak 6.8 A measured on 10), worse by two servos; the 3250's 4.2 A stall current says the pads-not-XH inlet fix and the inline fuse in `bom-supplemental.md` become mandatory. Bus timing is fine (12 servos ≈ 3.4 ms of the 20 ms tick).

### 4.6 The backward knee

A knee that bends backward is, in the sagittal plane, the forward knee with time reversed; the only asymmetries are the toe/heel split of the foot, the torso's fore-aft CoM and where the servo cases sit. The evaluation bears that out:

- Gate A: identical margins (30 mm) and crouch depth; **but the forward swing violates the ±45° ankle-pitch range** (45.5° at mid-swing, 51° at the end of a 6 cm step), because flexing a backward knee to lift the foot pitches the shank the other way. It needs a ±55° ankle.
- Gate B: same knee speed demand (2.4 rad/s), higher stance hip-pitch and ankle-pitch torques (torque margins 1.7× / 1.0× at the geometry sweep's tight corners vs 2.7× / 2.1×), more torso pitch under the ideal servos.
- Gate D with the ±55° ankle and STS3250 rolls + knees: walks 6/6 like the forward knee, with 2–3 mm more stance slip, 1.4° more tilt and 1.5–2 cm less progress per six steps (`gateD_knee_kp_cadence.txt`).

There is no capability the backward knee unlocks for this body and one cost (ankle range, and every leg part convention in `cad/` assumes the forward knee). **Decision: forward knee.** Since the knee servo's mechanical travel is ±95° and the direction is a joint-limit plus foot-orientation choice, the same printed legs could be run backward-kneed later if a reason appears; it is not designed in.

### 4.7 Turning: walking in an arc

Tom's follow-up: verify the body can change its bearing by walking in an arc. The generator places each swing foot in a frame rotated by `turn_deg` from the stance foot's yaw, the pelvis heading follows the mean of the two foot yaws, and the leg IK (`v6_kin.pose_world`) solves each leg in its own yawed frame — the hip yaw joint carries ±turn/2 and the soles stay flat and parallel to their own footprint. Same deploy servo model and gait as §4.3 (STS3250 rolls + knees, 3° roll play, 1° lash, 2 Hz shaper + 80 ms), `docs/design-v6/gateD_turning.txt`:

| turn per step | steps | heading commanded → achieved | swing clearance | CoM margin ≥ | slip | verdict |
|---|---|---|---|---|---|---|
| +10° (left) | 8/8 | 75° → 65° | 24 mm | 25.3 mm | 1.5 mm | OK |
| −10° (right) | 8/8 | −75° → −64° | 25 mm | 25.0 mm | 1.7 mm | OK |
| +15° | 8/8 | 113° → 99° | 24 mm | 25.5 mm | 1.4 mm | OK |
| −15° | 8/8 | −113° → −100° | 25 mm | 24.9 mm | 1.3 mm | OK |
| +20° | 8/8 | 150° → 134° | 23 mm | 25.2 mm | 1.5 mm | OK |
| +10°, 12 steps | 12/12 | 115° → 100° | 24 mm | 25.3 mm | 1.5 mm | OK |

The body turns at up to 20° per step (a 134° bearing change in eight steps, radius ≈ 0.17 m) with the same margins as the straight walk; the hip yaws never exceed ±10°. The 10–15 % shortfall in achieved heading is the yaw chain's compliance and lash plus sole pivot under the yaw torque, i.e. the sim's version of the hardware's yaw play; a heading-conditioned controller closes it, and it is the quantity to compare against the bench (Gate D) rather than a design limit. One implementation detail worth keeping: the mid-swing outward bump must be applied in the swing foot's own yawed frame — applied in world y it turned into a fore-aft error past 60° of heading and the 15°/20° arcs fell at steps 7 and 5 (`gateD_turning.txt` history in git).

![v6 arc walk](../sim/renders/v6_arc_walk_strip.png)

## 5. Assumptions to retire on the bench before ordering

In the order the gates doc asks for them ("from the datasheet before purchase, from a bench measurement after"):

1. **STS3250 static stiffness ≥ 3× the STS3215's** (≥ 1.5× with the sag compensator of §8.1). The number in the model (4×) is from a third-party bench, not ours. Buy **two** first, mount one in place of a v5 hip-roll servo (same case), and repeat the 2026-09-13 stance-hip test (8° roll, planted foot): the 3215 stalled 1.2° short at load 120. Pass: ≤ 0.4° short at the same load (≤ 0.8° with the compensator). This single measurement decides whether six are ordered.
2. **Roll-chain play ≤ 3° total per joint (target ≤ 1°), backlash ≤ 1°.** Design rules for the CAD: both roll joints double-supported (horn + idler), metal horns with thread-locked M3s, the ankle link's fork tines sized to the disc faces (`SV_IDLER_CASE_FACE`, not the phantom face), and a tilt-hysteresis play measurement per roll joint on the assembled leg *before* the first walk — the same test that found 2–3° on v5.
3. **Masses.** The plant is 1.25 kg from lumps; the assembled robot with six STS3250 will be ~1.37 kg. Weigh every segment when built and regenerate the inertials (`build_v2_inertia.py`), as `DESIGN.md` §8 has been asking for v5.
4. **The 600 ms "200 ms pose".** Still unexplained on v5 (streamer cap or servo under load). The Gate D script depends on the shaper's timing; measure the streamer against a free servo before trusting any cadence number on hardware.
5. **Sole friction.** μ 0.3–1.0 passes in the model; the bench never measured the pad. A TPU 95A sole is specified for edge compliance; measure μ on the actual pad once printed and re-run `static_gait.py --mu`.

## 6. What changes downstream

- **12 actuators** → the observation frame grows (3 × 12 + 12 = 48 per frame vs 43/49 on v3/v5), `tools/gen_obs_spec.py` re-emits `kNumJoints`/`kServoId`, the SIL C ABI's fixed `[10]` arrays become `[12]`, every existing checkpoint is non-deployable (the yaw precedent: "a parameterization refactor + parity re-run, ~a day"). `walker_env.py` already derives the DOF count from the model; the v6 plant runs in it unchanged (that is how Gate D was run), and its reward roles (`hip_yaw … ankle`) simply do not see the new `ankle_roll` joint until a term is written for it.
- **The gait in `static_gait.py` is the bench script for Gate D on hardware**: the same keyframes, streamed through the firmware's `pose` path, with the robot on the floor and Tom's per-attempt go (the 09-13 rules). It is not a controller; the policy (Gate E) comes after the body has done this open-loop.
- **Charge for every free channel from day one** (gates doc, proposal 1): the ankle roll joins yaw and hip roll in the posture regularizer of the first v6 reward.

## 7. Next steps (after sign-off)

1. Buy 2 × STS3250; run §5.1 on the v5 hip roll (the robot is out of service for walking but a single hip-roll servo swap for a stance test is a bench-only, torque-released job).
2. CAD in `cad/`: `foot` v4 (roll servo across the foot, 130 × 60, TPU sole pocket), `ankle_link` (new: grips the pitch servo, forks in X), `leg_link` at 110 mm (a length parameter), `pelvis` v7 (84 mm hip separation, battery channel, Pi bay on the deck front, aft board recess), unchanged `yaw_carrier` / `yoke_roll` / `yoke_pitch`. Then `check_assembly.py` over the new ROM (ankle roll ±30° vs the shin fork and the foot walls is the new pair to sweep), `check_printability.py`, `run_checks.sh`, fly-in animation, and `build_v2_inertia.py` into the v6 plant.
3. Print, assemble, weigh, measure play per roll joint, re-zero, and run `static_gait.py`'s keyframes over the tether: stand → crouch → shift → one-foot balance → stepping in place → six steps, in that order, each a number, on the floor.
4. Only then Gate E: train on the v6 plant with the measured masses, play and stiffness in it.

## 8. Option study: cost / benefit of the alternatives, and the STS3250 purchase risk

Tom (2026-09-13): *"I wouldn't want to buy a bunch of 3250 servos and then find they're insufficient."* Same tools, same deploy servo model and gait as §4.3 unless stated; rows in `docs/design-v6/study_options.txt` and `study_feedback_3215.txt`.

### 8.1 Is the STS3250 necessary, and what if it under-delivers?

Two controller-side rescues were tried on the all-STS3215 body, because a policy would find them too:

- **IMU-roll feedback** (torso roll → ankle/hip roll targets, P gains ±0.5 … ±1.0): **no help in any sign or gain**. The failure is a lateral pelvis *translation* (the two roll joints of the stance leg sag as a parallelogram, the torso stays within 5° of level), so an attitude sensor does not see it.
- **Joint-position sag compensation** (target += k·(target − measured) on the four roll joints, from the servo's own readback): k = 1 walks the all-STS3215 body **only with ≤ 1° of roll play** (13 mm margin); at the measured 3° it still falls at the crossover; k ≥ 2 is unstable through the 2 Hz shaper + 80 ms dead time.

So the STS3215 route is "1° play *and* a compensator *and* 13 mm of margin", against a hardware record of 2–3° play. Not a design to bet a build on.

The other side of the risk, an STS3250 that is less stiff than the third-party 4×:

| STS3250 stiffness vs 3215 | no compensation | sag compensation k = 1 |
|---|---|---|
| 4× (third-party bench) | walks, 26 mm margin | walks, 14 mm |
| 3× | walks, 25 mm | — |
| 2× | falls after step 1 | **walks, 12 mm** |
| 1.5× | falls after step 1 | **walks, 11 mm** |

**The purchase risk is bounded:** even a 1.5× STS3250 walks with one line of feedback that the controller will have anyway, and at 5° of play nothing walks with any servo. The one thing that cannot be recovered in software is play, which is a printed-chain property, not a servo property. Recommended sequence stands: buy two, measure stiffness and play on the v5 hip roll (§5.1), then six.

### 8.2 The alternatives

| option | what changes | cost (money / build / software) | gates | verdict |
|---|---|---|---|---|
| **v6 serial ankle, STS3250 at rolls + knees** (this spec) | +2 servos, ankle block, foot v4, pelvis v7, longer leg links | ≈ $180 / 4 new or revised parts / 10 → 12 DOF refactor (~1 day) | A 30 mm, B pass, C/D pass incl. arcs, robust to 3° play, 1.5× servo shortfall with feedback | **baseline** |
| v6, STS3215 everywhere | same parts, no purchase | $0 / same / same | A pass, B knee 1.5×, **D fails at measured play**; walks only at ≤ 1° play with a compensator | fallback only if the bench shows the new chains at ≤ 1° |
| **parallel-linkage ankle** (both ankle servos in the shank, ankle DOF through links) | 2 linkages + 4 rod ends per leg, a lever on each servo; foot/ankle subtree 200 → 110 g | +$20–40 / 2 more parts per leg, linkage design + play control (the exact failure class of v5) / same | A 30 mm, **B unchanged** (the swing knee, not the ankle mass, is the speed limit: 1.56× vs 1.5×), D identical to serial per servo class (falls on 3215, walks on 3250) | **no benefit on this servo class**; revisit only for a dynamic gait where swing inertia matters |
| torso-mass shifter / waist roll instead of ankle roll (10 + 1 DOF) | a lateral slide or waist roll joint moving the torso mass | 1 servo / new mechanism / obs change | kinematics: the torso is 26 % of the mass; a 60 mm torso shift moves the CoM 16 mm, the shift needed is ≥ 36 mm; and feet cannot overlap, so no stance exists without a lateral shift | **not viable** at this mass distribution (servos in the legs dominate) |
| arms / waist yaw for balance | +2–4 servos as reaction masses | $50–110 / parts / DOF refactor | static single-foot stance unchanged (they add nothing to Gate A); help only dynamic recovery | later, for a policy; not for the walking capability |
| **scale-up** (130 mm segments, 150 g pack, ~1.35 kg, 0.45 m deck) | longer leg links, bigger bay | +$20 pack, more filament / same parts scaled / same | A 30 mm at 12° roll; B speed 1.82× (3215) / 3.06× (3250); D: **falls with the 110 mm gait on both servo sets**, walks on STS3250 once re-tuned (12 mm margin: heavier body, more sag) | possible but buys speed margin at the cost of lateral margin; not needed to pass |
| Dynamixel XL430-W250 (12 ×) | new bus (Protocol 2.0), U2D2 or board, all brackets, all CAD | ≈ $600 + $50 / everything / firmware bus rewrite | B: knee speed 1.95×, torque 1.67× — short; stiffness unknown | **no** |
| Dynamixel XC430-W240 (12 ×) | same | ≈ $1,100 / everything / rewrite | B pass (2.4× / 2.0×); stiffness unknown, needs the same bench test as the 3250 | last resort only, as Tom said |
| Feetech STS3235 (metal-case STS3215) | drop-in | ≈ $30 each | same torque/speed as the 3215; stiffness and play **unverified** | worth one unit on the same bench test if the 3250 disappoints |

### 8.3 What the study changes

Nothing in the spec; it sharpens the purchase decision: (1) the failure mode is roll-chain compliance and play, which no alternative architecture removes and only servo stiffness, chain design and a position-error compensator address; (2) the STS3250 is sufficient down to 1.5× its claimed stiffness with feedback, and the bench test of two units settles which regime we are in before six are bought; (3) roll-chain play above ~3° defeats every option — the CAD rules in §5.2 and the per-joint play measurement are not optional.

## Files

- `sim/gen_plant_v6.py` — parametric MJCF (all dimensions, masses, ranges); writes `sim/bimo_biped_v6ar.xml`
- `sim/v6_kin.py` — analytic 6-DOF leg IK, support-polygon and CoM geometry
- `sim/design_gates.py` — Gates A and B, the task-space timeline, the geometry sweep
- `sim/static_gait.py` — the quasi-static walk, Gate C/D sweep, rendering
- `tests/test_v6_design_gates.py` — pins the numbers above
- `docs/design-v6/` — every table quoted here, `gateCD_sweep.json`, and the option study (`study_options.txt`, `study_feedback_3215.txt`)
- `sim/renders/v6_static_walk_strip.png`, `sim/renders/v6_arc_walk_strip.png` (+ the gitignored `.mp4`s)
