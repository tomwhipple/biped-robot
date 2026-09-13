# Lessons learned, 2026-09-13: this hardware does not support walking, and why it took two months to say so

Written the evening the robot fell off the table and broke its hip roll joints, at Tom's request: "confront the fact that there is a design flaw in this hardware: it doesn't support walking ... be sure to include why it took us so long to catch this defect." What follows is what the record supports, what I believe, and where I was part of the delay.

## 1. What the hardware can and cannot do

Measured today on the bench with open-loop joint commands, no policy, and cross-checked in the MuJoCo plant with the deploy servo model (all numbers in `hw_sessions/2026-09-13/notes.md`):

- **It can stand, squat and return.** Reference squats to hip −65 / knee −95 / ankle −40 with every joint within 0.5° of command, six for six. The v41rsi_b policy squats on it three for three.
- **It cannot shift its weight onto one foot from rest.** There is no ankle roll joint. With both feet flat, a planted foot pins the pelvis level, so rolling a hip cannot move the centre of mass; the roll servos slide the feet on the pad (torso centroid unmoved on camera at a 12° lean, servo loads 88–168) or stall (stance hip 1.2° short at load 120). Shortening a leg pivots the body about the stance foot's inner edge, which carries the centre of mass *toward* the lifted side: every "lift" tilted the body onto the lifted foot (8.0° for a 1.1 cm shortening, which is atan(1.1 / 8 cm hip width)). The sim sweep finds no lean angle between "falls back onto the other foot" (≤14°) and "tips over sideways" (≥16°).
- **A single step from rest is a prop, not a stride** (Tom's verdict, sim agrees: the sole clears by ~3 mm for ~200 ms while the body tips onto it).
- **The only regime in which this kinematics can unload a foot is dynamic:** a lateral rock on the foot edges. The sim reaches it (a 10–14° hip-roll rock at 0.6–0.8 s sways the torso 13–35° and lifts the feet 1–3 cm) but the band is narrow and bounded by resonance falls, and the one hardware attempt at it put the robot off the table.

So the precise statement is: **the kinematics (5 DOF per leg, no ankle roll, rigid flat 5 cm feet on an 8 cm hip base) permits walking only as a dynamic edge-rocking gait, and the implementation — servo speed under load, 1–2° of play in the yaw/roll chains, sole friction on the pad — leaves no usable margin inside that regime.** In the simulator the same kinematics walks (20 cm steps, metronomic cadence, 2026-08-18), because the simulator's feet rock on their edges without sliding, its joints have no play beyond the modelled backlash, and its servo reaches its measured no-load speed. On the robot the same commands produce the shuffle we have been looking at since 2026-09-08.

## 2. The signals we had, and what we said about each

| date | signal | what we concluded at the time |
|---|---|---|
| 2026-07 | design: 5 DOF/leg, no ankle roll (docs/servo-map.md); hip-yaw redesign work on play | a yaw/roll play problem to fix mechanically, not a walking-capability question |
| 2026-08-02 | plant knee axis had sim and robot bending opposite ways | fixed in sim; no capability review |
| 2026-08-18 | v21sched walks in sim: 100% alternation, 20 cm steps | "cadence is metronomic"; the sim was taken as proof the body can walk |
| 2026-08-30/31 | ground attempts: falls (issue #29) | fall-latch and arming logic; "Stage 0 CoM + friction gap" noted, not pursued |
| 2026-09-02 | zero-command stand: ~1 Hz **roll** rocking and sideways falls on hardware, still in sim; play measured by tilt hysteresis (L hip roll 2–3° step, R knee 3.5°); free-travel play in the twin did **not** reproduce it | attributed to play and a coupling chain; "right ankle closed" after a hands-on lurch fix |
| 2026-09-03 | horn screws loose, re-zeroed; leg_rom lifts one foot **with the other foot clamped** (Tom's instruction) | the clamp was a safety measure; nobody wrote down that it was also the only way a foot came up |
| 2026-09-07 | a held forward pulse steps 1.5× the twin on hardware and falls 2/3; "Stage 0 CoM + friction gap" again | tethered obs dump planned to find a sensing gap |
| 2026-09-08 | the twin's forward step is a **1 cm shuffle**; Tom: "foot lift + feet apart" | treated as a reward problem: swing-height and foot-cross terms (v36swing) |
| 2026-09-11 | v37knee_b "crouch" on hardware = hip-yaw pinch, no descent; twin reproduces | policy behaviour; probe contamination found the same day |
| 2026-09-13 morning | v41rsi_b squats on the robot; toe-in and feet-together are hardware-only (twin shows neither) | a sim-to-real gap on the yaw/roll channels; obs-dump planned |
| 2026-09-13 afternoon | open-loop balance, step, rock: the measurements in §1 | **the capability question, asked directly for the first time, answered in three hours** |

The pattern in the right-hand column is the finding: every hardware failure received a local, plausible, single cause — play, a loose horn, a coupling chain, IMU bias, a reward term, a calibration, a probe bug — and each of those was real. None of them was checked against the simplest question: *can this body take its weight on one foot at all?* That question needs no policy, no referee and no GPU, and it took one afternoon with a serial cable.

## 3. Why it took so long

1. **The simulator walked, and we let that stand in for the body.** Since 2026-08-18 the plant has walked with metronomic cadence, so "the kinematics is fine" was never re-examined. The sim was calibrated where we looked (geometry from CAD and the as-built check, servo no-load speed measured on a free knee, backlash, latency, dead time) and not where it mattered for this question: foot-edge rocking, sole friction on the actual pad, and play downstream of the encoders. The twin inherits the same plant, so "the twin reproduces it" was reassurance about the firmware loop, not about the body.
2. **We went to reinforcement learning before we went to the bench.** The whole pipeline — trainer, referee, distillation, twin, deploy — was built and iterated for seven weeks before anyone commanded a joint by hand and asked what the body does. When the policies shuffled, the natural move inside that pipeline was another reward term.
3. **The roadmap's gates were standing and crouching first.** Sensible for safety, but it meant the walking gate — the one that would have exposed this — was always the next hop. Every stage we passed (stand still, zero at home, squat) is a static skill, and this body is fine at static skills.
4. **Each failure had a true local explanation.** Loose horns were loose. Play was 1–2°. The probes were contaminated. The reward did lack a swing term. Fixing real things felt like progress and each fix was measured to help a little, which is exactly what a wrong model of the problem produces.
5. **The clamp hid it.** On 2026-09-03 single-foot lifts were done with the other foot clamped to the table on Tom's instruction. That was the right call for safety and it worked perfectly, which is why it never occurred to us that without the clamp the same lift is impossible.
6. **My own part.** I stated motion from camera frames twice ("lowered, then back to parallel" on 09-11; "the right sole is off the table" today) and was corrected both times by Tom's eyes. I ran probes with a contaminated setting for three weeks. I attributed the shuffle to the reward with confidence. And this afternoon I ran a dynamic maneuver on the table straight after building it, when the sim had shown the same family falling at neighbouring parameters. Confident local explanations are how a two-month delay is built.

## 4. What this means for the project

- **Everything static stands.** Standing, zeroing, the squat, the deploy pipeline, the referee, the twin as a firmware-loop check, reference-state initialization as a training technique: all of that is real and measured, and today's sim-versus-bench agreement on the negatives is evidence the plant is trustworthy for the questions it was calibrated for.
- **Walking on this body needs a hardware change, not a reward term.** The candidates, in order of how directly they address the mechanism: an ankle roll joint per leg (6 DOF/leg, the standard answer to lateral weight shift); failing that, wider feet with compliant, high-friction soles so the edge-rocking regime has margin, plus taking the yaw/roll play out of the chains; and faster or stronger actuation only helps once one of those is done. A shuffle gait (feet sliding, never lifting) is the other honest option and would need its own sim contact model.
- **Tonight's queue stays** (v42deep, v42deepfast: squat depth and toe-in on a body that squats), because those are static skills the body has.

## 5. Process changes, effective now

1. **Bench before policy.** Any new capability claim about the body (a lift, a step, a turn, a push recovery) is first tried open-loop over the tether with joint readbacks and cameras, and the result is written down, before any training run targets it. `tools/squat_bench.py` is the start of that kit.
2. **Calibrate the sim where the question lives.** Before trusting the plant on a contact-dominated question, measure the thing: sole friction on the pad, what the feet do at a 10° lean, joint speed under body load (the "200 ms" pose that took 600 ms is still unexplained), play downstream of the encoders. Put the measured numbers in the plant or say the plant is not calibrated there.
3. **Dynamic tests need a per-attempt go, on the floor, with a stated sim margin.** A tilt-abort that reads between poses cannot catch a dynamic fall. The robot does not do dynamic experiments on a table again.
4. **Frames are not measurements.** Motion claims come from encoders, IMU, tracked features across frames, or Tom's eyes, in that order of preference for each quantity, and "cannot tell from the camera" is an acceptable answer.
5. **One question per failure:** before a local fix, ask whether the same symptom would appear if the body simply could not do the thing. Today that question was cheaper than any fix we made in the preceding month.

## 6. Open items carried out of today

- Repair of the hip roll joints (Tom); re-zero (`cal zero` / `cal save`) if horns moved; the next hardware session starts with a stand test.
- The servo-speed question: the firmware smooth streamer versus the servo under load.
- The obs-dump test that would split policy-commanded toe-in / adduction from servo sag.
- A decision on the hardware direction in §4 before more walking-oriented training is queued.
