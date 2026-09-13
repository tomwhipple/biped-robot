# Design-stage simulation gates, and the toe-in hypothesis (2026-09-13)

Companion to `lessons-learned-2026-09-13-walking.md`. Two questions from Tom: what can simulation alone do so future designs account for stability and capability before hardware is ordered; and was the toe-in an attempt to put support under the centre of mass, taught by a sim that let the feet overlap.

## Part 1 — what simulation can guarantee before an order

The simulator did not fail us in July; the questions we asked it did. It was asked "does a policy walk?" and answered yes. It was never asked "can this body take its weight on one foot?", which is a question about the body, not the policy, and which the same simulator answered in an afternoon once asked. The gates below are all things the plant can be asked with no policy at all, in order, and every one has a pass/fail number.

### Gate A — kinematic capability map (no dynamics, no policy)
For the candidate design, sweep the pose space and compute, for each pose, the ground projection of the centre of mass against the support polygon of whatever is in contact.
- **Static single-foot stance:** does any pose exist with one foot flat, the other clear of the ground by ≥ 1 cm, and the CoM inside the stance foot with ≥ 1 cm margin? Reachable from the standing pose by a continuous path that keeps the CoM inside the support polygon throughout? *This body: no such path exists (no ankle roll; the planted foot pins the pelvis; the only path is dynamic). That is the finding of 2026-09-13, and it is a five-minute computation on the CAD kinematics.*
- **Split stance:** both feet flat with a 3–6 cm fore-aft offset, CoM inside the hull, every joint inside its limit, sole level.
- **Crouch / recover:** the deepest level-sole pose and the deepest any-sole pose inside the joint limits (this body: 4.0 cm and 5.9 cm of hip drop).
Pass: single-foot stance and split stance exist with margin. Fail: the design cannot walk statically, so it must walk dynamically, which raises the bar on every gate below.

### Gate B — actuator envelope against the motions the design needs
From the datasheet before purchase, from a bench measurement after: torque–speed curve under the expected load, position resolution, backlash, dead time, and the fastest commanded profile the control path can actually deliver (our smooth streamer took 600 ms for a "200 ms" pose; unexplained and it would have failed this gate).
- Compute the joint speeds and torques of the Gate A paths and of a nominal swing (foot 2 cm up, 5 cm forward in 0.4 s). Require ≥ 2× margin on speed and ≥ 1.5× on torque at the intended supply voltage.
- Put the measured envelope in the plant (this plant has the STS3215 torque–speed clamp, backlash deadzone, latency and dead time; the streamer limit was not in it).

### Gate C — contact realism, made adversarial
The failures that hid from us were all at the feet. Before trusting any locomotion result:
- Self-collision ON for everything that can touch: foot–foot, foot–shank, thigh–thigh. (This plant had the feet mutually transparent from 2026-07-30 to 2026-08-14; see Part 2.)
- Sole friction and *edge* behaviour: rigid feet rock on their edges in the sim and slid on the pad in reality. Sweep friction 0.3–1.0 and add compliant-sole and edge-contact variants; a capability that survives only at μ = 1 on a rigid edge is not a capability.
- Joint play downstream of the encoder (1–2° here) as a free-travel model with DR, and the plant's stand must reproduce the hardware's rocking signature (it did not on 2026-09-02; that mismatch was noted and not pursued).

### Gate D — open-loop capability tests in the plant, then the same script on the bench
The same joint scripts that ran on the tether today (`tools/squat_bench.py`: squat, one-foot balance, single step, rock) run on the plant first, then on the robot, and the two must agree on pass/fail before any policy is trained for that capability. Standing → squat → split stance → single-foot stance → single step → push recovery, in that order, each a number.

### Gate E — the policy gate, last
Only after A–D: train, and require the policy's foot clearance, stance width and stride to sit *inside* the margins A–C established, not at their edges. Report the margins on every card.

What this would have cost in July: a day of computation on the CAD kinematics for Gate A and a servo datasheet for Gate B. What it would have said: "this design has no static single-foot stance; walking will be a dynamic edge-rocking gait; the actuator has ~1× speed margin for it." That is the sentence we needed before ordering.

## Part 2 — the toe-in hypothesis: confirmed for the old plant, refuted for the current line

**Hypothesis (Tom):** the toe-in was the policy putting support under the centre of mass, and a sim that let the feet overlap taught it.

**Evidence, plant history.** `sim/bimo_biped_v5body.xml` records that on 2026-07-30 the sole contact geoms were given `conaffinity 0`, so the two feet stopped colliding with each other. On 2026-08-14 the file's `<contact>` block added explicit pairs (sole–sole, sole–opposite shank) with this note: *loco_v18f was measured pivoting with the soles overlapping on 49% of turn steps, up to 46 mm deep — the turn_180 score rode on feet that pass through each other.* On 2026-08-18 a full-footprint pair was added because standing feet were visibly merging on the v21sched reel. So: **confirmed for the policies of 2026-07-30 to 08-14** — the sim allowed overlap and at least one policy exploited it, and this was caught and fixed then.

**Evidence, current line** (walker plant, 2 Hz act lag, 3 seeds × 10 s, crouch hold and 0.35 m/s walking; `hw_sessions/2026-09-13/toein_hyp.py`):

| policy | scenario | sole overlap | min sole gap | yaw pinch | CoM margin, actual yaw | CoM margin, yaw zeroed |
|---|---|---|---|---|---|---|
| v37knee_b student | crouch | 0% | 1.5 cm | +19.9° | +3.9 cm | +3.8 cm |
| v41rsi_b teacher | crouch | 0% | 1.0 cm | +5.9° | +2.7 cm | +2.7 cm |
| v41rsi_b student | crouch | 0% | 0.9 cm | +5.6° | +2.7 cm | +2.6 cm |
| v33home | crouch | 0% | 1.0 cm | +11.8° | +3.9 cm | +3.7 cm |
| all four | walk | 0% | 1.5 cm | −4.5 to −10.9° (toe-*out*) | +3.2 to +4.2 cm | within 0.1 cm |

The current policies never overlap the soles, and zeroing the yaws kinematically changes the centre-of-mass margin by 0.1 cm: **the toe-in does not move support under the centre of mass.** The direct test agrees: with the yaw actions clamped to zero at run time, stand, crouch and walking are unchanged (0 falls in every case; v41rsi_b crouch depth 3.7 → 3.8 cm; walking alternation 77 → 81%, v33home 90 → 81%, v37 68 → 71%). **The hip yaw is a free channel with no support function**; the policies drift into toe-in during a crouch and toe-out while walking because nothing in the reward touched yaw until v41 (weight 3, raised to 10 tonight) and the sim charges nothing for it.

**Why the feet touch on the robot but not in the sim** is a different mechanism: the robot adducts both hips 2–3° at the armed stand and 3–5° at the squat hold (encoders), which the twin and the walker do not show at all (< 1°). That hardware-only adduction, on top of the 7°/side toe-in, closes the 1 cm gap. Whether the closed-loop policy commands it in reaction to real sensing (the +7.4° torso-pitch reading of 09-11 is the suspect) or the servo sags under load is the tethered obs-dump test still pending.

**Verdict:** the overlap half is true history, already fixed; the support-under-CoM half is refuted for the current line; the feet-together on hardware is a sim-to-real gap on the roll channel, not a learned strategy.

### Proposal for future work
1. **Charge for every free channel.** Any joint the reward does not shape gets a small posture regularizer from day one (yaw has one now; roll toward adduction should too — `w_foot_cross` / `foot_cross_sep` exist and were 0 in this recipe). A channel that costs nothing in the sim will be used for nothing on the robot.
2. **Collision on by default, transparency by exception with a written reason and an overlap monitor.** The 07-30 transparency was reasonable engineering and it still let a policy learn through the feet for two weeks; the fix was a measurement (49% of turn steps). Make that measurement a standing referee metric: fraction of ticks with sole boxes overlapping, and minimum sole gap, on every card.
3. **Model the roll channel where the robot disagrees with the twin.** The adduction is measurable and absent from the plant: add roll play / roll compliance under load as DR, and run the obs-dump to learn which it is.
4. **Kinematic re-evaluation as a standard probe.** "Zero this joint and recompute the support margin" and "clamp this action and rerun" took ten minutes and settled the question. Both belong in the probe kit for any suspected compensatory behaviour.
