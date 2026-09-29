# Get-up from prone with the arms (2026-09-29)

> Dated record for issue #85, with a finding for #84: accurate as of its date;
> the current design is [DESIGN.md](../../DESIGN.md). Simulation only.

Forward falls end prone. The legs-only roll prone → side → supine (design record
§12.1) fails 0/6 on the as-drawn robot with the arms folded up, whatever the
servos ([no3250_getup.txt](no3250_getup.txt)). This study searched for a path
back to standing from prone that uses the arms.

**Result.** It stands, robust 12/12: every robustness condition, at the
searched pace and 3× slower, with the servo overload cutoff enforced. The
path:

1. hold the arms pointing straight back — at the sky while prone;
2. roll onto the back with the legs;
3. fold the arms up before landing;
4. then the recommended seat push.

The study also found a flaw in the scripted get-up from a *backward* fall,
and a fix that is also robust 12/12 (§2).

## 1. Method

The method is getup-decision-2026-09-17.md §4:

- **Search**: a continuous search, never a hand-built grid. A cross-entropy
  (CEM) search runs over five free keyframes. Each keyframe has every leg joint
  and every arm joint, within the as-drawn ranges (DESIGN.md §3), and a move
  and hold time.
- **Physics**: `self_collide=True`, and the deploy servo model under Plan B
  (STS3215 everywhere, P × 4 on the rolls and knees).
- **Plant**: `r5_asdrawn_rom120`, the lumped as-drawn robot with the girdle
  and arms (2.10 kg, hip −120), with the elbow's drawn stops (−100 … +10°).
  The plant otherwise draws ±150.
- **Start**: the state a forward fall leaves. The robot is settled prone, arms
  at the walk's idle pose (shoulder +15°, elbow 0), legs straight.
- **Verification**: every winner is re-run in the six robustness conditions
  (play 3/5°, μ 0.3/0.7/1.0, servos 100/80/65 %), at the searched pace and
  3× slower. The STS overload cutoff is enforced
  ([getup-overload-2026-09-29.md](getup-overload-2026-09-29.md)).

The script is `sim/getup_v6_prone.py`; its docstring lists the families and
modes. The searches ran on mira (12 cores by day, 24 at night) and on the
laptop.

The recorded dead ends were not re-run: a prone push-up to kneel, the one-arm
side-seat push, a third shoulder DOF, and legs-only paths.

## 2. First, a flaw in the scripted seat push: it assumes the arms are already folded

The recommended seat push (DESIGN.md §9) starts with the arms folded up along
the torso (shoulder 180°). Its first keyframe puts them there in 0.5 s.

A real backward fall does not land that way. The robot walks with its arms at
+15°. The shoulder range is −90 … +200°, so every path from +15° to 180°
passes +90°, "straight back". On the back, straight back points into the floor.

The arm sweep levers the robot over its head onto its front, and the get-up
fails **0/6** ([getup_prone_entry_probe.txt](getup_prone_entry_probe.txt)).
Starting with the arms at 0° or −90° fails the same way. Only the start the
script was verified from, arms already at 180°, stands.

![The recommended seat push from a real backward fall: the first keyframe flips it prone](figs/getup_supine_armsidle_flip_sheet.png)

That changes both get-ups:

- **From prone**, the roll has to reach the back with the arms *already*
  folded. So the arms must be at 180° before the body lands, while they still
  swing in free air.
- **From supine** (#84), the seat push needs an entry that starts from the
  idle pose.

**The supine entry, searched.** The search covered the arm path and the hip
flexion through four keyframes (lie, prop, sit up, fold), before the
recommended brace. Every candidate was also scored 3× slower, so that the
winner is quasi-static ([getup_prone_entry_search.txt](getup_prone_entry_search.txt),
[getup_prone_entry_search2.txt](getup_prone_entry_search2.txt)).

The first, 9-parameter round found an entry that stands 6/6 at the searched
pace but **0/6** 3× slower: it throws the torso up. The widened round's
propped-on-the-forearms family passes. Restart 1's first population already
held 7 candidates that stood in all four scored runs, so the search was
stopped there. The candidates were regenerated from the search's own random
stream (seed 1007), and the first one verified.

**Prop up on the elbows** does this:

- the arms go to the idle pose with the elbows fully bent (−100°);
- the upper arms rotate to 85° to prop the torso up on the elbows;
- the robot sits up (hip −76°) on the propped elbows;
- the arms fold up (130° / −78°) while the hips tuck to −118°;
- the recommended brace, tuck, push and rise follow.

It stands **12/12**, six conditions × both paces, with the cutoff enforced and
no trip ([getup_prone_verify_entry.txt](getup_prone_verify_entry.txt)). The
shoulder peaks at 2.02 N·m (74 % of stall) at full strength. With the servos
at 65 %, the shoulder reaches 99 % of its weakened maximum, for 0.9 s.

![Supine with the arms idle → prop on the elbows → the seat push](figs/getup_supine_entry_sheet.png)

Parameters (degrees, seconds):

```
{"l_sh": 5.86, "l_el": -100.0, "l_t": 1.04, "p_hip": 0.0, "p_sh": 85.19, "p_el": -100.0, "p_t": 1.5,
 "su_hip": -75.67, "su_sh": 80.56, "su_el": -88.05, "su_t": 1.25, "f_hip": -117.69, "f_sh": 130.25,
 "f_el": -77.77, "f_t": 0.88, "b_t": 1.43}
```

## 3. Prone → back: the candidates

**A probe first**, to shape the search; not a verdict. The §12.1 legs-only
roll, with the arms held at a grid of poses
([getup_prone_probe.txt](getup_prone_probe.txt)):

- It fails with the arms at the idle pose (+15), at 0, at 90, and folded (180,
  200). That reproduces no3250_getup.txt.
- It **reaches the back with the arms held forward** (−90, or −45 with the
  elbow bent). Forward is down into the floor when prone, so the arms push
  the chest up and the legs roll the body over.

Every family below was searched by CEM: 6 restarts × 40 iterations × 48
candidates. Restart 0 was seeded from the §12.1 roll; the others were
uniformly random. Every restart's winner is then verified, because the
search's own score is a nominal run and does not rank robustness.

| family | what is free | reaches the back (nominal) | verified |
|---|---|---|---|
| **stow** (candidate 1): arms held at one searched pose, legs roll | the stow pose + every leg joint per keyframe | 6 of 6 restarts | ends with the arms idle. The best winner reaches the back 12/12, but chained onto the §2 entry it stands only 1/6 and 0/6: its roll holds the hip roll and pitch at stall long enough to trip them, and the tripped hips fail the seat push ([getup_prone_roll_stow.txt](getup_prone_roll_stow.txt), [getup_prone_verify_stow_entry.txt](getup_prone_verify_stow_entry.txt)) |
| **stow_fold** (candidate 1, ending folded) | the same, and the roll must end with the arms at 180 | 6 of 6 restarts | **restart 0: 12/12 standing**; restart 2: 11/12; the other four 2–7/12 ([getup_prone_verifyall_stow_fold.txt](getup_prone_verifyall_stow_fold.txt)) |
| onearm: the left arm free, the right arm held, legs free | | 6 of 6 restarts | not chained (ends with the arms idle) ([getup_prone_roll_onearm.txt](getup_prone_roll_onearm.txt)) |
| **arm1_fold** (candidate 3): legs straight, one arm rolls the robot | the left arm per keyframe + the right arm's pose | ARM1_RESTARTS | best winner 9/12: 5/6 at pace ×1, 4/6 at ×3. It fails with weak servos: the rolling shoulder runs at 94–100 % of stall ([getup_prone_verify_arm1_fold.txt](getup_prone_verify_arm1_fold.txt)) |

**The verified path (stow_fold, restart 0).** It holds the left arm at 73° and
the right at 90°: straight back, pointing at the sky while prone. The legs
roll the body over its right side, and the arms fold to 180° as it lands on
its back. Then comes the recommended seat push.

- **12/12 standing**: six conditions × both paces, cutoff enforced
  ([getup_prone_verify_stow_fold.txt](getup_prone_verify_stow_fold.txt)).
- **The roll uses the hips hard.** The hip roll and hip yaw reach stall in
  short bursts. The longest time above 80 % of stall is 0.4–0.7 s at full
  strength and 1.7 s with the servos at 65 %.
- **At 3× slower with the servos at 65 %**, the left hip yaw and hip roll
  trip the overload cutoff, and it still stands. That margin is thin.
  ROBUST_PLACEHOLDER

![Prone → roll with the arms held straight back → on the back with the arms folded → the seat push → standing](figs/getup_prone_chain_sheet.png)

Parameters: the `x` line of restart 0 in
[getup_prone_verify_stow_fold.txt](getup_prone_verify_stow_fold.txt).

## 4. Catching a forward fall on the hands (candidate 2)

CATCH_PLACEHOLDER

## 5. What it says

1. **Prone has a sim-robust path.** The arms, stowed straight back (at the sky
   while prone), clear the §12.1 leg roll. Folding them up as the body lands
   gives the seat push its start. The arms that blocked the roll were folded
   *up*, beside the head; held straight back they do not block it.
2. **The scripted seat push only works if the arms are already folded.** From
   a real backward fall they are not, and the fold flips the robot onto its
   front. The propped-on-the-elbows entry fixes it, robust 12/12. This belongs
   in #84's hardware sequence.
3. **A one-arm roll is torque-bound**: 9/12, at stall. It is not the path.
4. **The roll is the hardest part on the hips.** Its hip roll and yaw touch
   stall. The robust-scored search is the one to take to the bench, if it
   holds its margin (§3).

**What the firmware needs** (FIRMWARE track, #81/#84):

- A fall that ends prone is now recoverable. The fall guard currently
  disarms the robot at up_z < 0.4. The recovery would start from the disarmed,
  settled state, with the robot re-armed into the stow pose. That is a
  sequencing decision, not a new control law.
- The sequences are keyframe lists at the shaper's pace, like the seat push.

**What the bench owes:**

- the same "one motion per go, on the floor" rule;
- the seat-push entry first (supine is the more common landing), then the
  prone roll;
- measure the hip roll and yaw load during the roll.

## Files

- `sim/getup_v6_prone.py`:
  - `probe` and `entryprobe`;
  - `rollsearch <family>`, `entrysearch` and `catchsearch`;
  - `verify` and `verifylog`;
  - `render`, which writes an mp4 and a contact sheet (the mp4 is not
    committed).
- Logs: `getup_prone_*.txt`. Figures: `figs/getup_prone_chain_sheet.png`,
  `figs/getup_supine_entry_sheet.png`, `figs/getup_supine_armsidle_flip_sheet.png`.
