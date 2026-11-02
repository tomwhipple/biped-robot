# 2026-10-25: swing-leg reaction damping of the lift-off rocking, sim study

Question (owner): the stance-leg PID (kp 0.3, ki 1.0, kd 0.03) leaves a 2.0-2.4 Hz lightly damped
roll rocking after lift-off, and raising the stance `kd` goes unstable (0.07). Can the **lifted** leg,
moved as a reaction mass (hip roll / ankle roll), damp it? Arms are out of scope (they do not swing in
the roll plane). Everything below is simulation; nothing was run on the robot.

## 1. Calibration against the 2026-10-25 hardware logs

Hardware (`gyrosign_serial.log`, right stance, as-built shift 1.3x = 80.6 mm, straight 30 mm lift over
4 s, balance on). Sim: same keyframes (`one_leg.keyframes(frac=1.3)`), the deploy servo model of
`static_gait.run_walk` **without** the 2 Hz shaper (the bench streamer has none), as-built plant, the
firmware's stance PID twin (`balance.py`), 25 -> 50 Hz.

**Fitted plant** (`calibrated.json`; random search over 12 knobs, 3 stages, 144 + 54 + 144 plants x 3 kd x
3 seeds, scored on the rate rms windows of kd 0.03 and 0.05, peak err, frequency and "kd 0.07 unstable"):

| knob | fitted | note |
|---|---|---|
| servo P (kp x), hip roll / ankle roll / knee | 2.0x (STS3250 4x * 0.5) / 2.8x / 2.8x | ankle roll and knee = the `robot` preset; the 4x hip-roll credit is unmeasured, 0.5x of it fits better |
| servo D (kd) multiplier | 0.3 | the bench servo is nearly undamped (P 96 / D 0) |
| joint viscous damping | 0.10 N m s/rad | |
| roll-joint extra deadzone / shaft-link play / global backlash | 1.0 deg / 2.0 deg / 0.7 deg (total 3.7 deg on the rolls) | clamped-foot calibration said ~2.6 deg; free standing fits wider |
| servo shaper pole | 3 Hz (first-order cascade) | stand-in for the servo's own lag; the host streamer has none |
| dead time | 1 tick | |
| carpet | sole friction 1.0, pad contact solref (0.04, 0.15) | soft, underdamped |
| **lift-off disturbance (fitted)** | **0.51 N s lateral impulse on the swing foot over 40 ms, outboard, at the tick the foot clears 0.5 mm** | see below |

**Not reproducible by plant knobs: the size of the lift-off burst.** With every plausible knob (play
0-3 deg, roll deadzone 1-4.5 deg, kp 0.5-2.8x, kd 0-1.5x, joint friction, contact softness) the sim's
lift-off is a smooth transfer: rate bursts of 1-3 deg/s against the hardware's +-5..12 deg/s, and the
modal damping is much higher than the robot's. The hardware burst is a stick-slip / carpet-release event
the sim does not have, so I added a lateral impulse at foot clearance and fitted its size. That fixes the
amplitude of window 1, not its physics: treat it as a stand-in. The impulse direction (+ outboard) was
chosen by fit, mirrored with the stance.

**Fit** (right stance, 6 seeds, geometric-mean rate rms of the three 4 s windows, deg/s):

| stance kd | hardware | sim | peak err hw / sim |
|---|---|---|---|
| 0.03 (baseline) | 4.04 / 2.00 / 0.75 | 2.5 / 0.41 / 0.26 | 2.35 / 1.9 deg |
| 0.05 | 3.10 / 2.94 / 0.47 | 3.6 / 2.2 / 0.42 | 2.36 / 1.9 deg |
| 0.07 | 7.1 then unstable, err +12.3 deg (abort) | 5.7 then a growing ~2.5 Hz limit cycle: 3/12 runs diverge (err 49 deg), the rest oscillate +-5..20 deg/s | 12.3 / 49 deg |

- Frequency: sim 2.0-2.6 Hz (median 2.5), hardware 1.7-2.4 Hz (spectral peak of the 25 Hz log is broad).
- kd 0.05 matches (the hardware's window 2 is higher than kd 0.03's, as in the sim) and kd 0.07
  reproduces the instability (a limit cycle at ~2.5 Hz that diverges).
- **Residual:** at kd 0.03 the sim's rocking decays 4-5x faster than the hardware's (windows 2 and 3:
  0.41 / 0.26 against 2.0 / 0.75 deg/s); window 1 is 1.6x low. The hardware rocking decays with
  zeta ~0.015 at kd 0.03; the sim's best-fitting plants are more damped there and no plant I tried
  matches both the kd 0.03 decay and the kd 0.07 instability. The sim is near a stability edge at
  0.05-0.07 like the robot, which is what matters for the design, but it under-states how long
  the baseline rocks. The hardware rocking begins at ~1.6 s, the sim's at ~2.3 s (lift-off in the sim is later:
  clearance appears when the roll compliance is taken up).
- Run-to-run scatter is large near the edge (kd 0.05 window 2 spans 0.5-5 deg/s over seeds); everything
  below uses 6 seeds x 2 stances or 22 cases x 2 stances.

Figure: [2026-10-25-calibration-and-swing-law.png](2026-10-25-calibration-and-swing-law.png).

## 2. The swing-leg law

`swing_sim.SwingHook`, symmetric (one code path, hip/ankle roll joints of the swing leg named from the stance;
the preload and the lift-off impulse are mirrored with the stance, the rate term is odd in the error):

```
x     = sr*rate + se*err + si*W          W' = err - W/tau_w          clip +-lim
swing hip roll   += sh * x      swing ankle roll += sa * x      (hip pitch: sp, not useful, in roll plane)
swing hip roll   += side * sb   swing ankle roll += side * sba   (preload, ramped in over 0.5 s at lift start)
```

`rate` and `err` are the stance PID's own signals (gyro with 0.0075 rad/s noise and one tick of delay, IMU
roll with 0.1 deg noise and one tick). Gains in rad per rad/s (sr), rad per rad (se), `lim` and `sb` in degrees.
Searches: 240 random laws (hip/ankle mix, sr, se, si, tau_w, lim, preload) x kd {0.03, 0.05} x both stances,
then one-term-at-a-time scans (hip and ankle, sr/se/si, both signs, 6 seeds x 2 stances, kd 0.03/0.05/0.07/0.10),
then 5 stronger rate gains and variants.

What the sweeps say (all `final_eval.py` tables in [2026-10-25-final-tables.txt](2026-10-25-final-tables.txt)):

1. **The swing leg's roll play is the first-order effect, not the reaction law.** The hanging leg floats
   in the 3.7 deg roll deadzone and rests against its inboard stop, so a small reaction command does
   nothing until the slack is taken up. Preloading the swing hip roll **3 deg outboard** (and nothing else)
   takes it out: foot clearance +3 mm (11.7 -> 14.7 mm nominal). Across the 46-case robustness set the stance-only
   control passes 33/46 (2 falls: sole friction 0.4, both stances; 11 more fail the 10 mm clearance bar when the
   slack is bigger, min 7.7 mm), the preload passes **46/46** (0 falls, peak err <= 5.2 deg, clearance >= 11.5 mm).
2. **Dynamic reaction terms**: error (`se`) and integral (`si`) terms never beat the preload alone at the
   nominal gains, and larger values destabilise (the swing leg is a competing lateral actuator with
   the stance chain's phase lag). The ankle roll is no better than the hip. The **rate term on the hip roll,
   `sr` = +0.3** (leg swings with the body's roll rate, 0.3 deg per deg/s, clipped at 8 deg), is the only
   term that adds damping measurably; `sr` = 0.6 helps more at high kd but starts to hurt at 0.03/0.05
   (rate rms w3 0.6 vs 0.2), and `sr` >= 1 itself rocks (a 2 Hz loop through the leg).
3. At **nominal gains the rocking is not measurably reduced** by any law beyond seed scatter (rate rms
   w2 0.4 -> 0.5 at kd 0.03; 2.2 -> 1.1 at kd 0.05 is the preload's, within scatter of the 6-seed fit).
   The sim under-states the baseline rocking (residual above), so there is less to remove than on the robot.
4. The swing law **moves the stance-gain stability edge**: stance kd alone, 0.05 fine, 0.07 diverges in
   3/12 (49 deg); with preload + `sr` 0.3, 0.07 and **0.10 stay bounded** (12/12, peak err 4.6 deg; a
   +-10 deg/s limit cycle remains), 0.15 half survives. Scaling kp, ki, kd together: x2.0 falls in 10/12
   without the law and stays bounded (+-9 deg/s limit cycle, peak 2.1 deg) with it; x2.5 falls either way.

## 3. Best law, before / after

Law: **swing hip roll += 3 deg outboard (preload) + 0.3 * roll rate (deg per deg/s), clip 8 deg**
(`sb` 3, `sr` 0.3, `sh` 1, `sa` 0).

| | stance only (kd 0.03) | + preload 3 deg | + preload 3 deg + hip rate 0.3 |
|---|---|---|---|
| robustness set, 46 cases (23 per stance): pass | 33/46 | 46/46 | 46/46 |
| falls / aborts | 2 / 2 | 0 / 0 | 0 / 0 |
| peak err median / max | 2.4 / 52 deg | 2.6 / 5.2 deg | 2.6 / 5.0 deg |
| min foot clearance | 7.7 mm | 11.5 mm | 10.4 mm |
| rate rms median w1/w2/w3 (deg/s) | 2.4 / 0.6 / 0.0 | 2.5 / 0.5 / 0.3 | 2.8 / 0.7 / 0.3 |
| nominal, kd 0.05, rate rms w1/w2/w3 | 3.6 / 2.2 / 0.3 | 3.1 / 1.1 / 0.3 | 2.9 / 1.1 / 0.3 |
| nominal, kd 0.07, rate rms | diverges 3/12 | 5.6 / 18 / 19 limit cycle, 0 falls | 5.5 / 10 / 9 |
| nominal, kd 0.10 | 12/12 fail | 10/12 fail (err > 6) | 0/12 fail, err 4.6 deg |
| gains x2 (kp, ki, kd) | 10/12 fail | n/a | 0/12 fail |

The robustness set is the existing sweep's knobs (torso 0.6-1.4x, deadzone +1.5 deg, backlash +1 deg, 2 Hz lag,
dead time +-, mu 0.4-1.0, keyframes on the design plant, shift bias, CoM +-12 mm, gyro bias +-0.01 rad/s, six combined
worst cases) on the calibrated plant. **Foot-foot contact is not cured**: in every configuration (including the
baseline) the two foot slabs touch (gap ~ 0) for >95 % of the hold once the lifted foot has sagged inboard
(the 2026-10-25 notes saw the same on the robot: "soles nearly alongside"); a larger
preload (hip +8 / ankle -8 deg) raises the median gap by ~1 mm only. Tomorrow's bench should treat foot
contact as already present, not caused by this law.

## 4. Is it worth implementing in firmware?

- **The preload: yes, cheaply.** It is not damping; it is slack take-up. On the robot, add a constant
  outboard offset (3 deg, sign = away from the stance foot) to the swing leg's hip roll goal during `pose ... B<L|R>`
  after the lift starts, ramped in over ~0.5 s. The sim says +3 mm clearance (the 10 mm bench stage did not clear)
  and removes all falls in the robustness set. Cheap to test on the bench: 3 deg of swing hip roll
  and read clearance + `balance log`.
- **The rate term (`sr` 0.3): maybe, as a second step.** It buys stance-gain margin (kd 0.07 -> 0.10, gains x2), not a
  lower rocking at the shipped gains, and the shipped kd 0.03 is already inside the stable range in the sim. The
  change is small in `roll_pi.h`: one more output `sw = clamp(sr * rate, +-8 deg)` added to the swing hip roll
  goal next to the stance mix (ka +1, kh -1). It needs the swing leg's side and the same gyro sample;
  no new state. Only worth it if the owner wants more stance `kd`; without the rocking decay residual
  resolved I cannot promise it lowers the 2 Hz rocking on the robot.
- **Not recommended:** error/integral swing terms, ankle-roll swing terms, `sr` >= 0.6 (see 2).

## 5. Measured vs assumed

- **Measured (2026-10-25 hardware):** the balance logs (rate rms windows, peak err, the kd 0.07 instability).
- **Fitted to those:** the plant knobs and the lift-off impulse of section 1 (12 knobs, 240+ plants; the
  burst amplitude is fitted, not explained; the decay of the kd 0.03 rocking is **not** matched).
- **Assumed:** the roll-plane inertia and servo authority of the swing leg (CAD masses, the STS3250 4x P credit
  halved), the deadzone (play) model, that the real lift-off burst is an event a swing law can affect,
  gyro noise/delay.
- **Not tested:** anything on the robot; foot-foot contact forces (rigid slab contact, no friction fitting);
  the ZV-shaper run (hardware run 2) is not modelled; only the right stance is calibrated (left is its mirror).
