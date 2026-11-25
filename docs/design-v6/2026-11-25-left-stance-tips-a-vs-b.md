# The late left-stance tips: trim drift or a push between the feet? (#297, 2026-11-25)

*A dated record of an offline analysis (2026-11-25, night run, no hardware touched). It follows
[the 2026-11-24 record](2026-11-24-left-stance-tips.md). That record left two candidates for the 12 inward tips of the
11-16 evening, all in late left stances of normal-timing `--cmd` walks:*
- *(A) the firmware's learned stance-hip trim T drifting within a walk;*
- *(B) a push between the feet in double support that grows step by step.*

*It proposed a robot test (frozen trim against control) to separate them. The question here: does the data already on
mira separate A from B, so that the robot test is either not needed or sharper?*

## Result

**Partly. B is right about where the trigger is, but not about its form. A is not the trigger; it may make a tip more
likely once the trigger has happened.**

1. **The trigger is in double support, in the weight shift before the lift-off.** Every one of the 12 tips followed a
   *snap* in the shift onto the left foot (12 / 12):
   - About 1.2 s before the lift-off, at the shift's fastest part, the roll joints fall behind the shift. The L hip roll
     lags its goal by 4.0 deg on snap steps (4.0-5.8 on the tips), against 2.4 deg with no snap.
   - They catch up with a load spike, and the body rolls 3.1-4.4 deg toward the coming swing foot at 65-94 deg/s.
   - It then settles 0.6 deg further toward the left foot than on a clean step, with the hips squeezed: the right hip
     roll is loaded +32 counts against +16 (this is the 11-24 record's swing hip-roll load before the lift-off).
   - By its size and timing it is the "weight-transfer kick" of the README's `--td-pause` section (inferred).

   This lift-off state predicts the swing-back e at 0.9 s on all 365 normal-timing left stances:
   - With walk fixed effects it explains R² 0.46 of the swing-back.
   - Adding T and the stance count changes nothing: T +0.17 ± 0.16 deg per deg, k −0.02 ± 0.03.
2. **It is not a slowly growing preload.** The snap is on or off from step to step:
   - It never happens on the first left stance, and happens on 50 % of the second ones and 60 % of the late ones.
   - Snaps early in a walk and late in a walk are the same size: 3.41 against 3.39 deg; squeeze −25 against −28.
   - What looked like a build-up in the 11-24 medians is the snap share rising and T rising.
3. **The snap needs the shift to start soon after the touchdown.**
   - Left stances whose shift reaches 10 % within 0.5 s of the previous touchdown: 52 % snapped (716 at normal timing).
   - Left stances whose shift starts later: 1 of 257 snapped, and none tipped (all sessions, 11-13 and 11-16).
   - The later start is `--td-pause 0.3`: the 3.47 s recipe, and T 4 with the pause at 4.3 s/step.
   - The step period does not decide it: snaps happen at every period from 3.3 to 6.0 s when the shift starts early.
   - So the fast timing is protected by its touchdown pause, not by its short settle (the 11-24 record's guess). The 4.3 s
     runs (normal settle, 0.3 s pause) had 0 snaps in 106 left stances.
4. **T is not the trigger.**
   - The trim's correction is 0 by the lift-off. The last stance's correction is still fading when the shift starts
     (always, with the early start), and the trim's T values do not predict the snap: R² ≤ 0.001 for T R and for the
     previous stance's hike and error over all sessions; ≤ 0.012 on the evening alone.
   - With the late shift start, T L was 6 deg or more at 34 late left stances (19 of them at normal timing, up to
     about 7 deg): 0 tips.
   - On 11-13, a walk with the trim's learning off (`A_fwd34_learn_off`, T held at 3.4 / 5.2) tipped 3 times, with the
     same squeeze. That is a different timing (3.4 s) and early stances, so it is a weak point.
5. **T may make a snap step tip more often.** On the 166 snap steps (normal L, k ≥ 2; 12 tips):
   - the tips had higher T at the lift-off (AUC 0.78, p 0.001) and more hike in the raise (AUC 0.75);
   - in a logistic fit with the squeeze, T adds (z +3.6, p 0.0002) and the stance count does not (p 0.33).
   - This rests on 12 events, and T climbs within every walk, so it is a lead, not a finding.

**Strength.**
- **That the trigger is in double support: strong.**
  - 12 / 12 tips had a snap, against 0 / 65 late left stances without one.
  - The lift-off state separates tips from clean late stances with AUC 0.13-0.15 (p 0.0002 by permutation). That
    survives a correction for the ~20 features scanned.
  - It explains 46 % of the swing-back's variance across all 365 stances, not only the 12 events.
  - The earlier sessions (11-13, 11-16 09:25-18:26) repeat it: 29 more tips, on both sides, at several timings. The
    squeeze separates them with AUC 0.08 on the left and 0.18 on the right.
- **That the late shift start prevents the snap: strong for the snap, moderate for the tips.**
  - 1 snap in 257 left stances against 377 / 747 (Fisher p ≈ 7e-62).
  - 0 / 257 tips against 22 / 747 (p 0.002).
  - Inferred from runs that differ in other ways too (the 3.47 s recipe, the 4.3 s `tdp03` runs); not an A/B on the
    robot.
- **T's role as an amplifier: weak** (12 events, confounded with the step count).

**What this changes.**
- The first robot test is now the touchdown pause, read per step: the snap share should go from about 55 % to under
  2 %, which a few walks settle without needing tips.
- The frozen-trim test drops to third.
- For steering until then, use `--td-pause 0.3`: the 3.47 s recipe has it, or add it to T 4 (4.3 s/step).

## What each candidate predicted (stated before looking)

| | (A) the trim drifting | (B) a push between the feet that grows |
|---|---|---|
| A1 / B1: what differs before a tip | T and the hike it writes in the raise, at matched stance count | the double support before the lift-off, where the trim writes nothing: the hip-roll loads opposed (feet pushed apart or together), the stance width read by the encoders against the goals |
| A2 / B2: on every step (the continuous swing-back e@0.9) | scales with T; the double-support state adds nothing once T is known | scales with the double-support state; T adds nothing once that is known |
| A3 / B3: over a walk | the walks where T reaches tip values tip, whatever the double support does; a walk with learning off does not tip | the double-support state grows smoothly with the step count in every walk, at both timings |

Against the data (counts and tests below):

| prediction | found | verdict |
|---|---|---|
| A1: T / hike higher on tips at matched k | T at lift-off 6.36 against 6.24 (AUC 0.65, p 0.09); hike at 0.4 s 3.96 against 3.85 (AUC 0.61, p 0.21) | weak at best |
| A2: swing-back scales with T | T alone: R² 0.03; with the lift-off state in the model, T adds nothing (+0.17 ± 0.16) | fails |
| A3: high T tips regardless | late start: 34 late left stances with T median 6.07 (19 with T ≥ 6): 0 tips. Learning off (11-13): 3 tips | fails |
| B1: double support differs on tips | squeeze −36 against −24 (AUC 0.15), hips read 0.27 deg narrower, body 0.6 deg toward the stance side (p ≤ 0.001 each) | holds |
| B2: swing-back scales with the double-support state | R² 0.46 (squeeze) and 0.67 (width deflection, with T L, T R, k); T and k add nothing | holds |
| B3: grows smoothly over a walk, both timings | no: on or off per step, 50 % of 2nd stances and 60 % of late ones, the same size early and late; 1 % at the fast timing | fails as stated |
| B, binary tips: T adds nothing given the state | on snap steps T adds (z +3.6), k does not | fails: T modulates |

## The snap: what happens in the shift

On normal timing:
- the previous stance's foot lands about 1.8 s before the next lift-off;
- the trim's correction fades over 0.6 s (state FADE);
- the shift onto the new stance foot starts at the touchdown. It is min-jerk, so its 10 % point is 0.37 s after the
  touchdown and its 50 % point 0.71 s;
- then a 0.4 s settle.

The medians below are aligned at the left lift-off (s), normal timing:
- roll: body roll, deg (+ = toward the coming swing foot, the right);
- squeeze: (L − R) / 2 of the hip-roll loads, joint frame, STS counts;
- shift: the L hip roll's track nominal, deg (it moves from about +16 to −14.7 over the shift).

| | −1.6 | −1.4 | −1.3 | −1.2 | −1.1 | −1.0 | −0.8 | −0.6 | −0.4 | 0 |
|---|---|---|---|---|---|---|---|---|---|---|
| shift (nominal), tips | 15.8 | 11.7 | 8.6 | 4.7 | 0.4 | −3.8 | −10.6 | −14.1 | −14.7 | −14.7 |
| roll, tips (12) | 0.3 | 0.3 | 0.3 | **3.5** | 2.5 | −0.8 | 0.3 | −1.1 | −0.9 | −0.9 |
| roll, clean normal k ≥ 5 (151) | 0.2 | 0.3 | 0.8 | 0.8 | 0.6 | 0.4 | 0.3 | −0.1 | −0.3 | −0.3 |
| roll, normal k ≤ 4 (202) | −0.2 | 0.1 | 0.4 | 0.5 | 0.4 | 0.4 | 0.4 | 0.1 | −0.1 | −0.1 |
| squeeze, tips | −36 | −38 | −38 | −45 | −36 | −26 | **−46** | −39 | −36 | −36 |
| squeeze, clean normal k ≥ 5 | −34 | −38 | −41 | −36 | −34 | −24 | −25 | −21 | −23 | −24 |
| squeeze, normal k ≤ 4 | −32 | −34 | −33 | −30 | −25 | −21 | −22 | −18 | −20 | −20 |

The clean late medians mix snap and no-snap steps (86 / 65). Every one of the 12 tips shows the roll jump at −1.2 to
−1.1 s (+2.1 to +4.7 deg in the raw traces), then settles toward the stance side.

One step in detail (`capB_L_2` step 11, a tip; record ticks, s from the lift-off). From −1.46 to −1.28 the hip rolls
fall from 1-2 to 5-5.5 deg behind their goals. At −1.24:
- the right hip roll's load is +284 counts, the left ankle roll's −396, the right ankle roll's −448;
- the roll rate is 87 deg/s;
- the roll reaches 4.5 deg at −1.20 and is back at −0.8 deg by −1.00.

On step 9 of the same walk (clean, snap 1.9 deg) the lag peaks at about 3 deg and the loads at 148 / −172 / −252.

Per tip (all left stance, normal timing):
- snap: peak roll toward the swing foot in the double support, deg; and its rate, deg/s;
- end: roll at the lift-off relative to the start of the double support, deg;
- R hip: the right (swing) hip roll's load before the lift-off, joint frame;
- width dev: the hips' encoders minus their goals, L − R, deg;
- t10: the shift's 10 % point after the touchdown, s;
- lag: the L hip roll's lag behind its goal at the shift's fastest part, deg.

| run (MST) | note | k | snap | rate | end | squeeze | R hip | width dev | t10 | lag | T lift | e@0.9 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 18:29:42 | square_and_left | 7 | 4.36 | 89 | −1.90 | −39 | 32 | −0.88 | 0.38 | 5.80 | 5.95 | 4.79 |
| 19:06:57 | ss_side_2 | 5 | 4.09 | 70 | 0.34 | −32 | 24 | −0.70 | 0.43 | 4.13 | 6.31 | 3.09 |
| 19:06:57 | ss_side_2 | 7 | 3.23 | 67 | −0.86 | −24 | 16 | −0.54 | 0.43 | 4.04 | 6.41 | 2.92 |
| 19:09:28 | poshold_march_plain_b | 5 | 3.81 | 82 | −0.80 | −32 | 24 | −0.72 | 0.36 | 4.66 | 6.96 | 3.71 |
| 19:20:12 | rc_side_1 | 5 | 3.48 | 83 | −1.52 | −40 | 40 | −0.89 | 0.46 | 4.48 | 7.18 | 3.63 |
| 19:23:16 | rc_fb_2 | 5 | 3.14 | 65 | −1.11 | −36 | 32 | −0.81 | 0.44 | 3.96 | 6.23 | 4.39 |
| 19:30:52 | rc_fb_4 | 6 | 3.30 | 68 | −0.71 | −32 | 24 | −0.71 | 0.44 | 3.96 | 7.19 | 2.29 |
| 19:48:35 | poshold_march_M_d | 8 | 3.55 | 77 | −2.10 | −36 | 32 | −0.81 | 0.44 | 4.13 | 6.26 | 4.08 |
| 19:48:35 | poshold_march_M_d | 15 | 3.59 | 81 | −1.78 | −33 | 25 | −0.73 | 0.48 | 4.48 | 7.50 | 3.78 |
| 19:48:35 | poshold_march_M_d | 16 | 3.36 | 67 | −2.02 | −36 | 32 | −0.81 | 0.47 | 3.96 | 7.51 | 4.58 |
| 20:03:25 | capB_L_2 | 6 | 3.71 | 87 | −1.98 | −44 | 40 | −0.97 | 0.35 | 4.83 | 6.12 | 5.19 |
| 20:03:25 | capB_L_2 | 7 | 4.04 | 94 | −1.46 | −36 | 32 | −0.79 | 0.36 | 5.45 | 6.12 | 4.96 |
| *clean, normal, k ≥ 5, snap (86)* | | | 3.39 | 67 | −0.60 | −28 | 23 | −0.62 | 0.38 | 4.00 | 6.24 | −0.39 |
| *clean, normal, k ≥ 5, no snap (65)* | | | 1.64 | 29 | −0.31 | −20 | 8 | −0.45 | 0.36 | 2.37 | 6.26 | −2.75 |
| *normal, k ≤ 4 (202)* | | | 1.69 | 31 | −0.02 | −20 | 8 | −0.45 | 0.37 | 2.64 | 5.20 | −2.28 |
| *fast, k ≥ 5 (59)* | | | 1.18 | 26 | −0.32 | −22 | 18 | −0.48 | 0.64 | 2.29 | 5.87 | −2.96 |

The squeeze in plain terms:
- At the lift-off the hips read about 0.3 deg narrower than written: about 0.6 mm at the sole over the 121 mm leg.
- The two soles' encoders differ by −0.18 deg against +0.18 on clean late steps.
- So the feet are held against each other with the right hip roll pushing. The stance ankle roll then carries nothing
  in the raise (+7 counts against −24), as the 11-24 record found.

The 11-24 record's load pattern is this state:
- the swing hip-roll load before the lift-off (−32 there, raw sign) is the squeeze's right half;
- the stance ankle near 0 in the raise follows from it.

## The snap and the tips, by stance count (11-16 evening, 18:26-20:07)

A snap is a peak roll toward the coming swing foot of 2.5 deg or more in the double support, relative to its start.

| | k 1 | k 2 | k 3-4 | k 5-6 | k ≥ 7 |
|---|---|---|---|---|---|
| normal, left: snaps | 0 / 52 | 26 / 52 | 42 / 98 | 48 / 80 | 50 / 83 |
| normal, left: tips | 0 | 0 | 0 | 6 | 6 |
| normal, right: snaps | 14 / 52 | 12 / 52 | 22 / 96 | 24 / 72 | 22 / 65 |
| fast, left: snaps | 0 / 25 | 0 / 25 | 0 / 50 | 0 / 27 | 2 / 32 |
| fast, right: snaps | 1 / 25 | 1 / 25 | 0 / 45 | 3 / 16 | 2 / 28 |

- Late normal left stances: tips 12 / 98 with a snap, 0 / 65 without.
- The swing-back e@0.9 on late normal left stances: median +0.19 with a snap, −2.75 without (tips included).
- Right stances snap less (23-34 %) and never tipped on 11-16 evening.
- Why only the left side tips is not settled here. The roll play differs by side: L ankle 3 deg against R ankle 0;
  L hip 6 deg against R hip 8 (PR #290).
- Early snaps did not tip: 0 / 68 at k 2-4 against 12 / 98 late. What differs between them is T and the hike, not the
  snap (next section).

## The shift's start decides the snap (all sessions: 11-13, 11-16 09:25-20:07)

- t10 is when the stance hip roll's track nominal has done 10 % of its double-support move, s after the touchdown.
- Stance k ≥ 2.
- T lift ≥ 6: the number of late stances with T L at the lift-off of 6 deg or more.

| left stances | stances | snap | tips | late (k ≥ 5) | late tips | T lift ≥ 6 |
|---|---|---|---|---|---|---|
| normal timing, t10 ≥ 0.5 s (the 4.3 s `tdp03` runs, `tdp_mixedA`, `B1_*`) | 106 | 0 % | 0 | 34 | 0 | 19 |
| normal timing, t10 < 0.5 s | 716 | 52 % | 17 | 321 | 15 | 202 |
| fast timing, t10 ≥ 0.5 s (the 3.47 s recipe) | 151 | 1 % | 0 | 58 | 0 | 15 |
| fast timing, t10 < 0.5 s (earlier 3.3-3.6 s recipes) | 31 | 26 % | 5 | 8 | 1 | 2 |

The fastest part of the shift is the same either way, and so is the servos' lag behind it unless a snap happens. The
two starts differ in what is still going on:

| left stances, k ≥ 2 | n | goal rate (deg/s) | lag L hip / R hip / L ankle (deg) | trim still fading at t10 | its hike there |
|---|---|---|---|---|---|
| t10 < 0.5 s, no snap | 370 | 40.5 | 2.37 / 2.90 / 2.90 | 100 % | 0.90 deg |
| t10 < 0.5 s, snap | 377 | 40.5 | 3.96 / 4.31 / 4.57 | 100 % | 0.63 deg |
| t10 ≥ 0.5 s | 257 | 43.3 | 2.20 / 2.81 / 2.81 | 1 % | 0 |

- The early start overlaps two things: the landing's first 0.4 s, and the last stance's trim correction fading out on
  its hip roll (0.6 s, `fade_s`).
- **The data cannot tell which of the two matters.** Both go away with the pause.
- A hint: among early starts, a snap is a little likelier the less of the fade is left when the shift gets going.
  Linear fit with walk fixed effects: −0.22 ± 0.05 per deg of hike left, R² 0.03.
- The trim's T values do not predict the snap: R² 0.001 for T R, for the previous stance's hike at its touchdown, and
  for its error.

## T on the snap steps (A as an amplifier)

Normal left, k ≥ 2, snap steps (166, 12 tips); medians:

| | snap (deg) | squeeze | width dev | T lift | hike 0.4 s | e@0.9 |
|---|---|---|---|---|---|---|
| snap, k 2-4 (68) | 3.41 | −25 | −0.57 | 5.37 | 3.39 | −0.79 |
| snap, k ≥ 5, clean (86) | 3.39 | −28 | −0.62 | 6.24 | 3.86 | −0.39 |
| snap, tips (12) | 3.57 | −36 | −0.80 | 6.36 | 3.96 | +3.93 |

- Logistic fit of tip on the squeeze, T at the lift-off and k (standardised): z −3.1 (squeeze), +3.6 (T), −1.1 (k).
- T adds to squeeze + k: p 0.0002. k adds to squeeze + T: p 0.33.
- Over all sessions (17 tips on 369 snap steps): z −4.3 / +4.3 / −1.1.
- **Inferred, not shown.** A higher learned T, written as more hike in the raise, may be what turns a squeezed lift-off
  into a tip late in a walk:
  - early snaps at T ≈ 5.3 swing back (e@0.9 −0.79);
  - late ones at T ≈ 6.2 barely do (−0.39).
- But T rises with k in every walk. The only walks where T was not learning are not comparable (11-13, 3.4 s), and
  without a snap T up to 7 deg did not tip. The frozen-trim robot arm is what would settle it.

## Confounders

- **Servo temperature.** On the evening's late normal left stances there is no difference: tips against clean have
  AUC 0.44-0.52 for the four roll servos, at 31-33 °C. Over all sessions the tips run 1-2 °C warmer (AUC 0.59-0.63,
  p ≥ 0.07). That follows the time of day: the tips are in the evening, and the servos warm over the session. The swing-back does not follow
  temperature (ρ −0.17 to −0.04).
- **Supply.** The pack read 12.0 V throughout, and the servo-reported voltage (L hip roll) 12.0 V at every lift-off:
  no variation to compare.
- **The operator's `--cmd` inputs.** Late snap steps: 12 tips / 82 in `--cmd` walks, 0 / 16 in fixed walks (Fisher p
  0.21, too few fixed steps to tell). Fixed walks snap as often (`tl_cap_*`, `tl_def_*`: snaps from the 2nd stance on).
  Step types: as the 11-24 record found.
- **Step count.** Covered above: k does not predict the swing-back once the lift-off state is known (+0.01 ± 0.03), and
  does not predict tips on snap steps once T is known.
- **The previous right stance.** T R, its hike at the touchdown and its error at the touchdown each explain 3-7 % of
  the squeeze (walk fixed effects). They move together, so they cannot be told apart; together they explain 9 %.
- **Shift start within the early group.** Tips started a little later than clean late steps (t10 0.43 against 0.37 s,
  AUC 0.69, p 0.02); compare the fade hint above.

## Proposed next steps (mildest first; no firmware behaviour or default changed tonight)

All use existing host flags and CLI commands. Read every arm per left stance with `tip_causes.py`:
- the snap share (≥ 2.5 deg);
- the squeeze before the lift-off;
- e@0.9 on snap steps;
- the tips.

Use the 11-24 record's tipping pattern: normal timing, `--cmd`, three march steps, then sideways left at 3.75 mm/s,
24 steps a walk.

0. **Sim, optional, before the robot.** Replay the 11-16 walks' written goals, as recorded and with every shift moved
   0.3 s later, through the per-side play plant (`footoff_sag.py run`, the PR #290 play terms; CPU). Count the
   mid-shift roll transient and the lag spike.
   - If the plant shows the snap and the pause removes it, compare `fade_s` 0.3 there too.
   - If it does not show the snap (likely while #259's gate fails), skip to 1.
1. **Robot: the touchdown pause against the control.** 2 walks each:
   - C: as on 11-16;
   - P: `--td-pause 0.3`, 4.3 s/step, settle unchanged. This is the configuration of the 4.3 s `tdp03` runs, about 144
     clean steps on 11-16.

   Each walk has about 11 left stances with k ≥ 2.

   Predicted:
   - C: snaps in about 55 % (≈ 12 of 22), 1-3 tips, the squeeze rising on snap steps;
   - P: at most 1 snap in 22, 0 tips, e@0.9 about −2.6.

   Outcome:
   - **Pass** (the snap and the pause are confirmed): P ≤ 2 snaps of 22 and C ≥ 8 of 22. That is Fisher p < 0.01 with no
     tips needed.
   - **Fail**: P snaps like C. Then the shift's start is not it, and the next look is the shift's speed at its middle.
   - **Inconclusive**: C does not snap. The repaired hip changed the plant; re-take the baseline.
2. **Robot: which half of the pause.** Only if 1 passes. No pause, with the trim's fade shortened to 0.3 s so that the
   last stance's correction is gone before the shift gets going. Learning stays on; every other value is the firmware
   default (`trim off`, then `tools/wcli.py 'trim set 0.25 1 15 14 0.3 15 6 8'`, 2 walks, then `trim set default`).
   - Snaps drop as in P: the fade overlapping the shift is the trigger. The firmware candidate is to start the shift
     only after the fade (or a shorter `fade_s`).
   - Snaps stay: it is the landing settling. The candidate is a 0.3 s pause as the normal-timing default.
3. **Robot: the 11-24 frozen-trim test, re-read.** Control timing, so snaps still happen; T frozen at the seeds 5.2 /
   3.4 against learning on.
   - If T amplifies, the snap share is unchanged and frozen snap steps swing back like the early ones (e@0.9 about
     −0.8), with fewer tips.
   - This settles A's remaining role. It matters less once 1 holds, since without the snap T up to 7 deg did not tip.

Only after 1 (and 2) on the robot: a track-player or host default change (a normal-timing touchdown pause, or a shift
that waits for the fade), as its own PR, one change per run.

## Data and method

- The robot side comes from the flight records `hw_sessions/2026-11-09/flight_20261116T211526Z_prec16.frec` (14:15-20:07
  MST), `flight_20261113T172054Z_prec16.frec` (11-13 10:20 to 11-16 14:15) and `flight_20261113T005055Z_prec16.frec`.
- The runs are the `track_*.jsonl` logs of 11-13 and 11-16, as `stance_tips.py` finds them: 206 runs, 2261 single
  supports.
- The 11-16 evening (18:26-20:07) is the primary set; the rest is the replication.
- Tool: `experiments/roll-balance/tip_causes.py`, a sibling of `stance_tips.py` with the same runs, single supports and
  tip rule; tests in `tests/test_tip_causes.py`:

  ```
  python experiments/roll-balance/tip_causes.py extract --day 20261113 --day 20261116     # ~2 min, one pass per record
  python experiments/roll-balance/tip_causes.py table --day 20261116 --from 1415 --to 2010
  python experiments/roll-balance/tip_causes.py table --day 20261113 --day 20261116 --to 1415
  python experiments/roll-balance/tip_causes.py report <evening csv> --since 20261116T1826  # the evening
  python experiments/roll-balance/tip_causes.py report <both csvs>                          # all sessions
  ```

  - The cache goes to `build/tip-causes/` and the per-step CSVs to `experiments/roll-balance/results/tip_causes/`; neither
    is committed.
  - The report prints every number above except the single-step trace of `capB_L_2` step 11 and the per-tip table,
    which come from the same cache.
- The loads are in the joint frame: the STS load follows pos − goal (checked on standing ticks: 8 counts per step of
  deflection). They come in multiples of about 8 counts, so medians land on 8, 16, 24 ...
- The width and tilt numbers are frontal-plane sums of the encoders: hip roll L − R; hip + ankle roll L − R. They are
  not a full forward kinematics; the stance width in mm is that angle times the 121 mm leg.
- About 20 features were scanned. The double-support state's p of 0.0002 survives that; T's p of 0.09 on late stances
  does not. The amplifier result (p 0.0002 within snap steps) came from a second look, so treat it as a lead.
- Verified against inferred:
  - Verified in the record: the snap and its timing, the lag and the load spike, the lift-off state, all counts and fits.
  - Inferred: that the squeeze causes the failed swing-back (it predicts it; nothing was intervened on); that the pause
    is what prevents the snap (the runs with the pause differ in other ways); T as an amplifier.
