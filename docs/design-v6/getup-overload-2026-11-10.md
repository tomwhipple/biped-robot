# The get-up against the servo's overload cutoff, on the current references and plant (2026-11-10)

> Dated record for issues #126, #83 and #6: accurate as of its date; the
> current design is [DESIGN.md](../../DESIGN.md). Simulation only. Nothing
> here was measured on a servo. It re-runs
> [getup-overload-2026-09-29.md](getup-overload-2026-09-29.md) (the model and
> the documented constants are described there, §1–2) on what the get-up is
> now.

## 1. What was ported, and what changed since 09-29

The overload model came from branch `sims-overload-prone-current` (`bb9b7f2`,
with its record `2c4a4f8`), cherry-picked onto `main` (#126). The rest of that
branch (the prone study, the Plan B re-run, its DESIGN.md and `docs/wiring.md`
text, the current budget) is superseded or not taken, as #126 sets out.

- `sim/sts_servo_model.py`: the documented constants and `STSProtection`.
  One change: `main`'s `walker_env` now scales each servo's stall and no-load
  speed by a per-servo envelope profile (`servo_types`, `--servo-envelope`:
  the STS3250 hip rolls, the 11-03 refit). The protection reads that profile,
  so a servo's load is a fraction of its own effective maximum, and a trip
  writes 20 % of the base stall back under the profile. Before this change,
  attached to a training env, it measured an STS3250 hip roll against an
  STS3215's stall. The 09-29 runs wrote per-joint arrays themselves, so they
  are unaffected.
- `sim/gate_no3250.py overload` and `sim/getup_v6_overload.py`: as on the
  branch. Re-run on today's `main`, the 09-29 mode reproduces its log within
  about 2 points: the recommended sequence's shoulder 76 % (was 78 %), the
  same no-trip verdicts and the same strength edge
  ([getup_overload_r5_2026-11-10.txt](getup_overload_r5_2026-11-10.txt)).
  So the port did not change the model; the differences below come from the
  get-up and the plant.
- `sim/getup_ref_overload.py` (new): the cutoff on the current references.
  Log: [getup_overload_2026-11-10.txt](getup_overload_2026-11-10.txt).
- `tests/test_sts_overload.py`: the protection on a stub servo (trip at
  2.0 s, the drop to 20 %, measure-only, the two timers, the duty reading,
  release on a new goal, the envelope profile), the supply-current model at
  the datasheet's stall, the profile on a real `BimoWalkerEnv`, and the
  supine reference not tripping at full strength.

| | 2026-09-29 | 2026-11-10 |
|---|---|---|
| get-up | the scripted seat push, shoulder 90 → 0 (`r5_asdrawn_rom120`) | the campaign's analytic references (#137), replayed open loop from their recorded targets: **`getup_ref_v6ar_prone_ikc.npz`** (prone, arms idle; the roll, the seat push with the hands kept down at the crouch, the analytic rise; what the get-up candidate `robot_getup_w19k` tracks) and **`getup_ref_v6ar_ik.npz`** (supine, the reference #83 names) |
| seat push | 90 → 0, elbow −90 → 0 | 60 → 0, elbow −60 → 0, ankle −25 (`getup_ref_analytic.seat_push`; 09-29's second choice) |
| plant | lumped as-drawn robot, 2.10 kg | `bimo_biped_v6ar_meas.xml`: CAD inertials, the measured joint stops, the sole pads, 2.48 kg (w19k's training plant) |
| servos | Plan B: STS3215, P × 4 on rolls and knees | **STS3250 at the hip rolls, STS3215 elsewhere, knees included**; stiffness as built (4× hip rolls, 2.8× the other leg servos, `SERVO_KP_PRESETS["asbuilt"]`), arms 1× |
| envelope | 11.1 V | **11.1 V** (the get-up's training envelope, w19k) and **12 V refit** (#214: 12 V bus, hip pitch and knee stall × 2.0, no-load × 1.325 / × 1.281) |
| conditions | play 3/5°, μ 0.3/0.7/1.0, servos 100/80/65 % | nominal (joint friction 0.35, μ 0.7, play 3°); joint friction 0.05 and 0.5; μ 0.45 (the measured sole, [2026-11-04-sole-friction.md](2026-11-04-sole-friction.md)) and 1.0; play 1°; servos 85 %; mass 110 %; servos 85 % + mass 110 % + μ 1.0 together; servos 65 % |

Each condition runs at the reference's pace and 3× slower. Every run first
only measures. Any run whose longest continuous stretch above 80 % reaches the
2.0 s timer is then re-run with the cutoff enforced (latched) on that reading.
The servo model is the get-up studies' deploy model (`getup_v6.make_env`: 2 Hz
lag, 80 ms dead time, 1° backlash, 3° play on the rolls, integer ticks).

## 2. Results

**Every measuring run stands open loop: 80 of 80** (2 references × 2
envelopes × 2 paces × 10 conditions). That includes the references, which were
recorded on `bimo_biped_v6ar.xml` with STS3250 knees, replayed with STS3215
knees on the measured plant.

**The shoulder at full strength** (the seven conditions with the servos at
100 %), per reference and envelope, at the reference's pace:

| reference, envelope | shoulder peak, nominal | shoulder peak, worst condition | longest above 80 %, torque | longest above 80 %, duty | peak motor current, longest above 2 A |
|---|---|---|---|---|---|
| prone_ikc, 11.1 V | 79 % (2.14 N·m of 2.72) | 82 % (2.23 N·m; joint friction 0.5, μ 1.0, mass 110 %) | 0.02 s | 1.04 s | 2.13 A, 0.02 s |
| supine_ik, 11.1 V | 79 % (2.15 N·m) | 83 % (2.25 N·m; joint friction 0.5) | 0.02 s | 0.78 s | 2.14 A, 0.02 s |
| prone_ikc, 12 V refit | 80 % (2.36 N·m of 2.94) | 83 % (2.45 N·m; mass 110 %) | 0.02 s | 0.94 s | 2.33 A, 0.06 s |
| supine_ik, 12 V refit | 81 % (2.38 N·m) | 84 % (2.46 N·m; joint friction 0.5, μ 1.0, mass 110 %) | 0.02 s | 0.62 s | 2.34 A, 0.44 s |

The shoulder's peak rises with the envelope (2.14 → 2.36 N·m from 11.1 V to
12 V) and its load stays at ~80 %. So it is **torque-limited through the
push**: it runs on the envelope (duty ≈ 100 %) for 0.4–1.0 s of each push. It
holds above 80 % of stall for one 20 ms tick only because the envelope
`stall · (1 − |ω|/ω₀)` caps a moving joint below 80 %.

**At full strength the cutoff never fires**: in no condition, on either
reading, at either pace, on either envelope does any servo hold above 80 % for
the 2.0 s timer. The hottest servo at 11.1 V is not the shoulder but the
STS3215 knee on the duty reading: 1.22 s nominal, up to 1.46 s (μ 1.0), of
the 2.0 s. On the refit envelope (knees × 2.0) it is the shoulder, at
0.42–0.94 s.

**Where it fires** (the enforced re-runs), all with weakened servos:

| condition | 11.1 V | 12 V refit |
|---|---|---|
| servos 85 % alone | no trip (knee 1.50 s on duty) | no trip |
| servos 85 % + mass 110 % + μ 1.0 | **duty: the knees trip (prone 21.5 s, supine 10.3 s), then the shoulders, and the get-up fails** on both references; torque: no trip (shoulder 1.52 / 1.78 s) | no trip (shoulder ≤ 1.22 s) |
| servos 65 % | duty: trips and fails on both references; torque: supine trips the shoulders at 10.6 s and fails, prone stands | no trip (shoulder ≤ 1.28 s) |

**The margin in servo strength.** prone_ikc, nominal condition, every servo's
stall × the scale:

| servo stall | 11.1 V: no cutoff / cutoff on torque / cutoff on duty | 12 V refit: no cutoff / torque / duty |
|---|---|---|
| 100 % | stands / stands / stands | stands / stands / stands |
| 85 % | stands / stands / stands | stands / stands / stands |
| 75 % | stands / stands / stands | stands / stands / stands |
| 65 % | stands / stands / **fails** (hip pitch trips at 7.3 s) | stands / stands / stands |
| 55 % | fails / fails / fails | stands / **fails** / **fails** (shoulders trip at 23.7 s) |
| 45 % | fails | fails |

**3× slower** still buys the most margin at full strength. The shoulder peak
falls to 57–62 % nominal (worst 66–76 %), and its longest stretch above 80 %
duty to ≤ 0.22 s. With weak servos it does not help: at 65 % the slow
replays hold the load longer, and at 11.1 V the prone one trips on both
readings.

## 3. What changed from 2026-09-29

1. **The shoulder carries more of its stall.** On 09-29 the recommended seat
   push peaked at 51 % of stall nominal and 65 % (1.77 N·m) in the worst
   full-strength condition. Today's references peak at **79 % nominal and up
   to 83 % (2.25 N·m) at 11.1 V**, or 80–84 % (2.36–2.46 N·m) on the 12 V
   refit. DESIGN.md §4's 65 % is the old script's figure. Three things
   differ, none isolated here: the references use the 60 → 0 push (09-29's
   second choice, which then peaked at 74–78 %), the plant is 2.48 kg against
   2.10, and the knees are STS3215s at 2.8× instead of Plan B's P × 4.
2. **"No servo ever reaches 80 % on the torque reading" no longer holds.** The
   shoulder and the hip pitch touch it at full strength, for 0.02 s at the
   shoulder and up to 0.24 s at the hip pitch in the prone roll. On the duty
   reading the longest stretch at full strength is now 1.46 s (knee, 11.1 V).
   On 09-29 that was 1.48 s, but only with the servos at 65 %.
3. **"The get-up fails from weakness before the cutoff fires" no longer
   holds.** At 11.1 V, with the cutoff on the duty reading, prone_ikc fails at
   65 % strength, where it stands without the cutoff. The stacked condition
   (85 % servos, +10 % mass, μ 1.0) fails on both references. On the 12 V
   refit, the cutoff fails it at 55 % on either reading, where it stands
   without. **The cutoff now binds before weakness does.** That happens only
   with weakened servos, never at full strength.
4. **Unchanged:** at full strength the cutoff does not fire, and slowing the
   move is the lever with the most margin.

## 4. What it says

- **The shoulder margin (#83) is re-derived.** On the current get-up
  references the shoulder peaks at 79 % of stall nominal (2.14 N·m at 11.1 V)
  and up to 83 % (2.25 N·m) at full strength in the conditions tried. It is
  torque-limited for 0.4–1.0 s of each push. That is under the 2.0 s timer,
  so the cutoff does not fire, but the margin is in time (about half the
  timer) and in strength. With 15 % weaker servos plus 10 % more mass and
  μ 1.0, the duty reading trips the knees and then the shoulders.
- **The bench question has narrowed to two numbers:** which reading the
  servo's protection uses (torque or duty: reg 60 logged through a push
  settles it) and its real stall at the bus voltage. If the protection reads
  duty, the knees at 11.1 V and the shoulder have about 0.5–1.0 s to spare.
- **These are open-loop replays, not the policy.** The w19k family tracks
  prone_ikc with a residual (`--rise-ref-residual 0.15`), so the policy can
  load a joint more or less than the reference does. The referee does not
  run the cutoff. The next step for #6 is to run `STSProtection` in the CPU
  referee's get-up scenarios as a measuring column. With the profile change
  above it attaches to `BimoWalkerEnv` unchanged.

Caveats:

- The refit's stall × 2.0 is an effective low-speed torque given the plant's
  armature, not a datasheet number
  ([actuator-refit](../../experiments/policy/actuator-refit/2026-11-03-actuator-refit.md)).
  So the refit envelope halves the hip pitch's and knee's load fractions
  against the datasheet's stall. The shoulder and elbow are not refit.
- The supply-current model uses the STS3215's torque constant for every
  servo, so its hip-roll currents are not meaningful. The shoulder's
  2.1–2.3 A peaks cross the datasheet's 2 A over-current line for at most
  0.44 s of its 2 s, and never reach the memory table's 3.25 A.

## 5. What the bench still owes (#83)

Unchanged from 09-29 §6:

- read registers 19, 28, 34, 35, 36 and 38 back;
- log Present Load (reg 60) and Present Current (reg 69) through a seat push;
- find how a tripped servo recovers under an unchanged goal re-sent every tick;
- the pull test of a printed upper arm.

## Files

- `sim/sts_servo_model.py`: the documented constants, `STSProtection`, the
  supply-current model.
- `sim/getup_ref_overload.py run` → [getup_overload_2026-11-10.txt](getup_overload_2026-11-10.txt).
- `sim/gate_no3250.py overload` (the 09-29 mode, on today's `main`) →
  [getup_overload_r5_2026-11-10.txt](getup_overload_r5_2026-11-10.txt).
- `tests/test_sts_overload.py`.
