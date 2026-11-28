# Gear compliance and the give beyond the encoders, from the flight records (2026-11-28)

*A dated record for #6 ("Gear compliance: not started"), with a cross-check of #259 Round 8. Offline, CPU only:
nothing on the robot was touched (it is down since 11-16). Tool: [`experiments/gear-compliance/`](../../experiments/gear-compliance/)
(`gear_compliance.py`; its outputs `servo.json` and `body.json` are next to it). The sim change is opt-in:
per-side keys for the existing `--roll-series-k`. With the keys unused the default path is bit-identical, and so is the
shared spring.*

**In short:** the gears themselves show nothing. Beyond the encoders the left stance side gives about twice as much as
the right. Replaying the 11-16 walks (Round 8's harness) keeps only the left side's springs: hip 10.3 and ankle
11.3 N m/rad, with the right side rigid. That plant brings both sides' weight-shift snaps to the robot's level, and
`--roll-series-k L_hip=10.3,L_ankle=11.3` now expresses it.

## Result

**The gear train's own compliance cannot be measured from these records, and nothing in them calls for modelling it.
The encoder sits on the output shaft, so any flex in the gears is inside the position loop. The servos' static law
is exact, and it accounts for all of goal minus position.** In 2.18 M quasi-static holds (12 leg servos, 49-55
sessions each), the load register is the servo's P law, to the tick:

| servo | duty (register 60) for a position error of e steps (goal − present, joint frame) | dead band | holds where the law is exact |
|---|---|---|---|
| STS3215 (10 leg servos) | 0.8 % × (\|e\| + 1) for \|e\| ≥ 2 | ±1 step (±0.09°) | 98.1-99.7 % |
| STS3250 (the two hip rolls) | 0.8 % × e | none | 97.7-98.0 % |

Register 60 is the controller's output, not a sensed load, so it carries no mechanical information that goal and
position do not already give. The rest of the holds (0.3-2.3 %) read 0.8-4 % duty at e = 0: a goal the servo got
that the record does not show (a balance correction or a write in the same tick). In the sim's units, P 32 is
0.8 % of stall per step. That is 15.3 N m/rad for an STS3215 and 25.6 for an STS3250 at the datasheet's 12 V stall.
In single support, the stance hip roll's duty × datasheet stall is 0.39 / 0.43 N m (L / R). The plant's gravity torque
there is 0.84 / 0.82 N m. So the real torque per unit of duty is about 1.9-2.1× the datasheet's, or friction carries
the difference. That agrees with the `asbuilt` preset's 4.0 × kp 12 = 48 N m/rad on the hip rolls. The knee reads
1.4-1.7×, with friction mixed in. A gear train that flexes ahead of the encoder could only show up as dynamics (a
second resonance): none of this tests that, and nothing in it needs it.

**Beyond the encoder there is a large give, and the left stance side gives about twice as much as the right.** In single support with
the stance foot flat, the IMU's roll is compared with the roll the stance leg's encoders give through the plant's
kinematics. The difference, less the same session's double-support value, is the give beyond the encoders: horn
splines, horns, printed yokes and brackets, the foot. It always tilts the torso toward the lifted foot:

| | sessions | give at the stance hip roll's 0.82-0.85 N m: median over sessions (range) | spring-only fit, k (95 % CI), N m/rad | fit rms / a constant give's rms |
|---|---|---|---|---|
| **left stance** | 15 | **4.65°** (1.70-8.90) | hip **10.3** (8.9-12.3), ankle **11.3** (8.4-28.7) | 1.04° / 1.88° |
| **right stance** | 17 | **2.31°** (1.17-3.96) | hip **19.5** (18.3-20.7), ankle **16.5** (13.6-21.5) | 0.25° / 0.81° |

- **Left exceeds right in all 15 sessions that have both sides:** the ratio is 1.1-4.7, median 1.8, at the same
  hip torque (0.84 against 0.82 N m).
- **It behaves like a spring, not a play.** Within a side, the give follows the stance ankle roll's gravity torque
  across sessions. On the 11-01/02 stands the centre of mass sat inboard of the ankle (−0.3 N m); on the 11-11
  stepper it sat outboard (+0.4 to +0.5 N m). Two springs, at the hip and the ankle, with no offset, explain the
  sessions' spread: rms 0.25° on the right, 1.0° on the left. A constant give (a play, or a fixed preload) leaves
  two to three times that: 0.81° and 1.88°.
- **Hip against ankle is not identified.** The hip roll's torque hardly varies in single support (sd 0.025 N m
  within a session), so its "k" is the part of the give that the ankle torque does not explain. That part could
  equally be a play, a preload, or the foot. Across sessions the ankle split rests on posture differences between
  the stands and the stepper, which also differ in date. The left ankle's CI is wide (8.4-28.7).
- **Pitch: no sagittal give at these torques on the stands** (|pitch| ≤ 0.3° at a stance knee torque of
  0.31-0.38 N m: a knee spring stiffer than about 60 N m/rad). On the track sessions (11-12..11-16) the pitch reads
  0.4-1.8°, with opposite signs per side. That fits the roll give acting about a yawed axis (inferred). Not modelled.

**What the records cannot say:** whether the give sits at the hip, the ankle or the foot, beyond what the ankle
torque's spread shows; whether part of it is a play below the spring; and whether it is still there. The left hip
still had the original, weaker yoke in every session (it broke on 11-16, #326). The reprint may not give the same.

## Cross-check: #259 Round 8 ("a softer left hip")

Round 8 (0571d359, [the record](2026-11-27-left-hip-trim-compliance.md)) found that the left-stance lean fits best
with lower left-hip trim authority: a series spring of k 11-22 N m/rad on the left hip roll, or 0.25-0.5 of the
correction reaching the joint. Every spring on the right hip, or on both hips, killed the left stall response.

- **The records support a softer left side, at Round 8's stiffness.** The left stance gives 1.8× the right, in every
  paired session. Its spring-only hip k is 10.3 (8.9-12.3), the soft end of Round 8's range and the value it called
  best (k 11: left lean 3.01° against the robot's 3.69°).
- **On their own they do not support a rigid right side.** The right stance gives 2.3° (lumped k 19.5). Round 8
  never ran the pair the records give (L ≈ 10, R ≈ 19.5): its "both hips" rows put the same k on both sides (22,
  11). The replay below runs it. The right side's give does not survive there as a spring, which keeps Round 8's
  finding that the right hip must stay stiff for the left stall response. The 11-10 clamp stills (right foot clamped,
  [the clamp record](../../experiments/policy/weight-transfer-gate/2026-11-10-robot/README.md) §4) put most of the
  right side's unseen give between foot and shank: 4.1° there against 1.4° at thigh to torso. So the right
  side's give may sit at the ankle and spare the hip. The flight records cannot tell which.
- **Whether the left give is at the hip is not identified** (above). "Where the stance correction only partly reaches
  the joint" fits a spring beyond the hip-roll encoder, and the left's give that the ankle torque does not explain
  (lumped k 10.3) sits where that would put it. The records cannot tell a hip spring from a left foot or ankle that
  gives at a constant torque.

**Round 8's replay with these springs** settles the right side's question, at least for the open-loop replay
([`round8_replay.md`](../../experiments/gear-compliance/round8_replay.md); the 11-16 evening walks, Round 8's
harness and scores):

| plant (`grade` + series springs, N m/rad) | L snaps (robot 53 %) | L increment (+1.97) | L r | L false tips | L lean (3.69°) | R snaps (29 %) | R short | R slope on the hike (+0.48) |
|---|---|---|---|---|---|---|---|---|
| `grade` | 48 % | +1.96 | 0.93 | 0 | 1.33 | 9.5 % | +0.61 | −0.35 |
| the records' fit, both sides (L hip 10.3, ankle 11.3; R hip 19.5, ankle 16.5) | 0.3 % | +0.36 | 0.56 | 1 | 3.04 | 6.5 % | +0.41 | −0.05 |
| both hips only (L 10.3, R 19.5) | 0 % | +0.71 | 0.85 | 0 | 3.23 | 0 % | +1.39 | −0.03 |
| left hip only (10.3) | 47 % | +2.12 | 0.93 | 0 | 3.16 | 8.9 % | +0.34 | +0.03 |
| **left hip and left ankle (10.3, 11.3), right rigid** | **42 %** | **+1.83** | **0.91** | 1 | **2.69** | **35 %** | **−0.18** | +0.01 |
| the left pair and the right ankle (16.5) | 78 % | +1.39 | 0.85 | 1 | 2.68 | 43 % | −0.40 | −0.05 |

- **The left side's measured pair is the best plant of the set.** It keeps the left stall response (42 % snaps,
  r 0.91, one false tip in 380) and moves the left-stance lean toward the robot's: 2.69° against 3.69°, where
  `grade` has 1.33°. It also brings the right stances to the robot's level: 35 % snaps against 29 %, 0.18° over
  the robot on the stall steps. Round 8's left hip spring alone got the right side to 11 %. The left ankle spring
  is what adds that.
- **Any spring on the right side breaks the left.** With the right hip at 19.5 the left snaps vanish, as Round 8
  found at 22 and 11. With the right ankle at 16.5 they rise to 78 %, against the robot's 53 %. So the right
  stance's quasi-static 2.3° does not behave as a roll spring beyond the encoders while walking. It is somewhere
  the replay's goals do not load the same way: the foot or sole, a play taken up at the stand, or the
  double-support baseline (inferred). The clamp stills point the same way: the right side's give was at foot to
  shank.
- **The slope on the hike stays unexplained** (+0.01 against the robot's +0.48), as in Round 8: closed loop.

So the records and the replay together support a softer **left** side, at the hip and the ankle, with the right
side stiff in the replay. The per-side keys can express that; the shared spring cannot.

## The sim term

`--roll-series-k` (`roll_series_k`, both envs, parity R15) was the shared spring: one k per role, L and R the same.
It now takes per-side keys as well, the way `--roll-play-deg` does: `L_hip=`, `R_hip=`, `L_ankle=`, `R_ankle=`. A side key
overrides its role's range on that side. With any side key, every side draws its own k (MJX: the same
`fold_in 6262`, four draws instead of two). The records' fit is

    --roll-series-k L_hip=10.3,R_hip=19.5,L_ankle=11.3,R_ankle=16.5

and the CIs as a DR range: `L_hip=8.9:12.3,R_hip=18.3:20.7,L_ankle=8.4:28.7,R_ankle=13.6:21.5`. The plant
that the replay above supports is the left side only, `--roll-series-k L_hip=10.3,L_ankle=11.3` (a role left out is
rigid). Grading: `GRADE_ROLL_SERIES=L_hip=10.3,L_ankle=11.3` (or `GRADE_PLANT_TERMS=ser_L_hip=10.3,ser_L_ankle=11.3`).
The gate's `ser_<side>_<role>` terms override `ser_hip` / `ser_ankle` on that side.

**Checks:** `tests/test_roll_series.py`, five new tests:
- the parser and the config.json round trip;
- per-side k and independent draws in both envs;
- per-side values equal to the shared fit give the shared plant, array for array;
- R15 airborne parity with the per-side springs;
- the gate's per-side terms move the plant as the flag does.

`experiments/gear-compliance/bitid.py` compared the branch with a `git archive` of `main` (ea0ea763) over 60 domain-randomised
steps in both envs, float64 and float32, on the default plant and on the shared spring drawn per episode
(`hip=8:16,ankle=12:25`). Every qpos, obs and reward is equal.

**Recommendation for the planner (#137, #259):**
- **No A/B night on these values yet:** they describe the left yoke that broke. The first session on the repaired
  robot re-measures them for free. A one-foot hold of a few seconds per side, with the flight recorder on, is
  enough: re-run `body` (Reproduce, below).
- **Meanwhile, a candidate can be graded on the left-side plant:** `GRADE_ROLL_SERIES=L_hip=10.3,L_ankle=11.3`.
- **If a night is spent on it,** the one-variable arm is B = A with `--roll-series-k L_hip=8.9:12.3,L_ankle=8.4:28.7`
  (the left CIs, the right rigid), against A's shared `hip=10:25,ankle=16:40`.
- **For #259,** the left pair is a better replay plant than Round 8's left hip spring, on both sides' levels. It
  does not explain the right-stance slope.

## Method

**Data.** Every flight record in `hw_sessions/` on mira (`rec.MAIN`, read only; 111 files, 86 with leg-servo feedback, 2026-10-30 to 11-17),
cached per session (`extract`): per tick and leg servo, the goal written, present position, speed, load, voltage, flags,
plus the IMU's fused roll and pitch, the gyro, and each boot's calibration (zero, direction) from its HELLO packets.
The tick's torque bit is not set on the bench pose stream, so a servo counts as driven when its goal-valid and
driven flags are set.

**Holds (`servo`).** A servo holds when its goal is unchanged and its position is within one step over a centred
0.5 s window, its speed register reads 0, and there is no time gap. The error is goal − present in the joint frame,
and the duty is −register 60 × direction / 1000. The register's sign is opposite to the speed's
(`experiments/supply-r/supply_r.py`).

**The body (`bodyextract`, `body`).**
- **Quasi-static ticks:** every leg servo driven, gyro norm below 0.15 rad/s over 0.3 s, upright (g_z > 0.8).
- **Kinematics:** per tick, the plant (`bimo_biped_v6legs.xml`) at the encoders' angles gives the world's up in
  the IMU frame as each foot standing flat would put it. It also gives both feet's heights along the IMU's measured up.
- **Support:** single support is one foot more than 12 mm lower than the other; double support is both within
  3 mm. Sessions with hung or one-leg-rig in the name are left out.
- **The give:** the IMU's roll (ZYX from the fused quaternion, heading-free; not the tick's `up`, which is the body's
  z in the drifting world frame) minus the stance leg's foot-flat roll, less the session's double-support median of
  the same. Signed toward the lifted foot.
- **The torques:** each stance joint's gravity torque is the moment, about the joint axis, of the weight of
  everything on the torso's side of it, with gravity along the IMU's up. It is exact in quasi-static single
  support, and it makes no use of the duty.
- **The fit:** per session the medians. Then `give = τ_hip/k_hip + τ_ankle/k_ankle` across sessions, weighted by
  min(n, 2000), sessions with an odd posture (τ_hip < 0.7 N m) out, and a 1000-sample session bootstrap.
- **Torque feedback:** within a session the give also feeds back into the ankle torque (a tilt moves the centre of
  mass, about 0.064 N m per degree). So the within-session regression is biased (left 4.6 deg/N m by OLS, 3.5 with
  the foot-flat geometry as the instrument). The session medians are used instead.

## Caveats

- **The double-support baseline is taken as zero hip torque.** With the feet under the hips the hip rolls carry
  little in double support, but the two legs can preload each other through the floor. The k values are lumped
  over that unknown baseline.
- **A foot that is not flat counts as give.** The stance foot on its edge in single support would read as give.
  Quasi-static selection keeps the tipping steps out, not a foot resting on its edge.
- **The plant is the as-drawn legs-only body (1.50 kg).** The robot weighed 1.62 kg on 11-10, so the gravity
  torques may be about 8 % low, and the k values with them.
- **Each session is one mechanical state.** Screws were tightened, and a leg link and the right yoke replaced,
  during the period. The spread across sessions is partly that.

## Reproduce

```bash
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES=
.venv/bin/python experiments/gear-compliance/gear_compliance.py extract --workers 6      # -> /tmp/gc/cache (6 min)
.venv/bin/python experiments/gear-compliance/gear_compliance.py servo --json experiments/gear-compliance/servo.json
.venv/bin/python experiments/gear-compliance/gear_compliance.py bodyextract --workers 6  # -> /tmp/gc/body.npz
.venv/bin/python experiments/gear-compliance/gear_compliance.py body --json experiments/gear-compliance/body.json
```
