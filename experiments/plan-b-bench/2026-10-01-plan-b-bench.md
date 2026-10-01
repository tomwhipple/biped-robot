# Plan B bench, session 2 (2026-10-01): loaded stiffness on the printed rig

Issue #73, [DESIGN.md §13](../../DESIGN.md) step 0, first loaded runs. This
session put the bench STS3215 on the printed rig
([2026-09-30-plan-b-rig.md](2026-09-30-plan-b-rig.md)), with a 100 mm lever
and Gatorade bottles hung from its notch. It measured stiffness at P 32, 128 and
160, then held a rigid inertia (a C-clamp in place of the unprinted salt cup).
Step 1 (stance) was not run: the v5 robot will not be repaired.

**Result.**
- **Stiffness rises with P.** Measured with the gearbox friction cancelled, it is
  **3.5 ± 0.4×** the P 32 stiffness at P 128 and **4.8 ± 0.6×** at P 160.
- **With a rigid inertia on the lever (a C-clamp, run D), P 160 oscillates
  while holding still, whatever the D.** Up to P 128 / D 32 it holds quietly and
  small moves settle. Slow 90° sweeps shake at P 96 and above, where P 32 does not.

How the session ran:

- **Operator:** the Claude session on Mira ran it as `claw`, on Tom's
  instructions. Tom gave every go and hung every bottle.
- **Video:** from 12:36 every hardware command was filmed (`tools/cam_record.sh`,
  `/dev/video0`, wall-clock burn-in), from before the port opened until a few
  seconds after it closed.
- **Raw data:** the per-run JSON, `session.log` and the clips are gitignored.
  They are on Mira under `hw_sessions/2026-10-01/gain_bench_{onebottle,2bottle,4bottle}/`.

## The rig and the loads

- **Rig:** the printed bracket clamped to the desk edge, and the 130 mm stiffness
  lever on the horn. The loads hang from the 100 mm notch on a string.
- **Loads:** 12 fl oz Gatorade bottles, taken as **0.39 kg each, nominal and
  unweighed** (no scale). One bottle at 100 mm is about 0.38 N·m.
  - Every torque and every absolute stiffness below carries the error in that
    mass.
  - The **ratios to P 32 do not**: a common error in the bottle mass scales
    every P's stiffness by the same factor.
  - At most 4 bottles were used (1.53 N·m, about 52 % of the 2.94 N·m stall
    figure in `docs/servo-map.md`). Five or six would be 65–78 %, close to the
    servo's overload cut (80 % for 2 s).
- **Servo:** the same STS3215 as session 1, on the same board (main 98c9ced).
  - It started the day as ID 11.
  - It was renamed to **ID 30** at 12:36, so that `move` is no longer clamped
    to `R_ankle_roll`'s ±25° band.
  - The rename printed `timeout` but landed (scan and register 5 read 30). It
    left the EEPROM unlocked (register 55 = 0), as #101 describes; the next
    gains write re-locked it.
  - It is still ID 30 at the end of the session.
- **"Straight out":** position 2057, taken from the morning's reading with the
  bottle hung and a webcam check. Ticks below are positions minus 2057.
  Positive means the lever is lower: the load pushes toward higher ticks.

## Run A: one bottle, two directions (12:19–12:24)

A one-off script (`2026-10-01/one_bottle.py`), on ID 11, at P 32 / 64 / 96 /
128 / 160 / 32. With one bottle hung the whole time, it held 2057 and
approached it from 40 ticks above and from 40 ticks below, twice each. Values
are ticks from 2057, with the load register in brackets.

| P | from above | from below |
|---|---|---|
| 32 | +12 (104) | 0 (0) |
| 64 | +6 (96) | +2 (32) |
| 96 | +3 (64) | +2 (40) |
| 128 | +2 / +3 (48 / 80) | +2 (48) |
| 160 | +2 (56) | +2 (56) |
| 32, repeat | +12 (104) | 0 (0) |

- At P 32 the from-below approach sat exactly on target with load 0, so friction
  was carrying the bottle on its own.
- The from-above error halved from P 32 to P 64.
- Above P 64 both directions converge on about +2, the dead zone (registers
  26/27 = 1) plus quantization.
- One bottle cannot resolve the higher P values. The script's printed "k"
  assumed the full bottle torque with no friction, and is wrong; it is not used.

## Run B: two bottles (12:39–12:53)

`2026-10-01/two_bottle.py` covered P 32 and 128. At Tom's word ("no need to
lower. just write"), `two_bottle_level.py` covered P 160 with the gains
written level. For each P: bare, 1 bottle, 2 bottles, bottles off.

| P | bare | 1 bottle | 2 bottles | bottles off |
|---|---|---|---|---|
| 32 | +2 (24) | +6 (56) | +8 (72) | +2 (24) |
| 128 | 0 (0) | +3 (80) | +4 (112) | −1 (0) |
| 160 | −1 (0) | +2 (56) | +4 (136) | −1 (0) |

The sag is only 3–4 ticks at P 128 and 160. That cannot separate 3× from 4×,
so the run was repeated with four bottles and probes.

## Run C: four bottles, with probes (13:02–13:18)

`2026-10-01/four_bottle.py`, on ID 30. At each P the order was:

1. Write the gains, with the bare arm released level.
2. On Tom's go: torque on and hold 2057.
3. At each state (bare, 1–4 bottles, bottles off): a static sample, then a probe.
   - The probe moves to 2057 + 23 and returns to 2057 ("from above"), then to
     2057 − 23 and back ("from below").
   - Moves run at 100 steps/s, and each return is sampled for 3 s.
4. After "bottles off": release level.

Values are ticks from 2057 / load register. The band is half the difference
between above and below. The midpoint is their mean.

| P | bottles | static | from above | from below | band ± | midpoint | overshoot above / below | status |
|---|---|---|---|---|---|---|---|---|
| 32 | 0 | −1 / 0 | +1 / 0 | −1 / 0 | 1.0 | +0.0 | 0 / 0 | 0 |
| 32 | 1 | +2 / 24 | +11 / 96 | +0 / 0 | 5.5 | +5.5 | 0 / 0 | 0 |
| 32 | 2 | +5 / 48 | +19 / 160 | +5 / 48 | 7.0 | +12.0 | 0 / 0 | 0 |
| 32 | 3 | +7 / 64 | +28 / 232 | +10 / 88 | 9.0 | +19.0 | 0 / 0 | 0 |
| 32 | 4 | +13 / 112 | +35 / 288 | +24 / 200 | 5.5 | +29.5 | 0 / 0 | 0 |
| 32 | 0, return | +1 / 0 | +2 / 24 | −1 / 0 | 1.5 | +0.5 | 0 / 0 | 0 |
| 128 | 0 | −1 / 0 | −1 / 0 | +1 / 0 | −1.0 | +0.0 | 0 / 0 | 0 |
| 128 | 1 | +2 / 48 | +2 / 48 | +2 / 48 | 0.0 | +2.0 | 0 / 0 | 0 |
| 128 | 2 | +3 / 80 | +4 / 112 | +3 / 80 | 0.5 | +3.5 | 0 / 0 | 0 |
| 128 | 3 | +6 / 176 | +7 / 208 | +4 / 112 | 1.5 | +5.5 | 0 / 0 | 0 |
| 128 | 4 | +6 / 176 | +11 / 336 | +6 / 176 | 2.5 | +8.5 | 0 / 0 | 0 |
| 128 | 0, return | +0 / 0 | +0 / 0 | +0 / 0 | 0.0 | +0.0 | 0 / 0 | 0 |
| 160 | 0 | +0 / 0 | +0 / 0 | +1 / 0 | −0.5 | +0.5 | 0 / 0 | 0 |
| 160 | 1 | +2 / 56 | +2 / 56 | +1 / 0 | 0.5 | +1.5 | 0 / 0 | 0 |
| 160 | 2 | +4 / 136 | +4 / 136 | +4 / 136 | 0.0 | +4.0 | 1 / 0 | 0 |
| 160 | 3 | +6 / 216 | +6 / 216 | +3 / 96 | 1.5 | +4.5 | 0 / 0 | 0 |
| 160 | 4 | +5 / 176 | +8 / 296 | +5 / 176 | 1.5 | +6.5 | 0 / 0 | 0 |
| 160 | 0, return | −1 / 0 | +0 / 0 | −1 / 0 | 0.5 | −0.5 | 0 / 0 | 0 |

The per-state numbers are in `2026-10-01/four_bottle_summary.json`.

### Stiffness

Each fit is a least-squares line with an intercept, deflection on torque, over
the bare and 1–4-bottle states. Torque uses the nominal 0.39 kg per bottle.

| P | k from the midpoints (N·m/rad) | × P 32 | k from static sag (N·m/rad) | × P 32 | load per tick | × P 32 |
|---|---|---|---|---|---|---|
| 32 | 34 ± 3 | 1.00 | 76 | 1.00 | 8.0 | 1.00 |
| 128 | 122 ± 9 | **3.5 ± 0.4** | 139 | 1.83 | 32.0 | 4.00 |
| 160 | 166 ± 17 | **4.8 ± 0.6** | 178 | 2.36 | 40.0 | 5.00 |

- **The midpoint k is the answer.** Friction stops the lever short on the side
  the target came from, so the midpoint of the two approaches cancels it.
- **The static k is not.** Friction carries part of each load at P 32: the
  band there is ±5–9 ticks against ±0–2.5 at P 128 and 160. That inflates
  P 32's static stiffness and compresses every static ratio toward 1.
- **The load register per tick is exactly P/4.** It is a straight line over
  every window with nonzero load. That confirms the position controller's own
  law, not the output torque.
  - Turning it into N·m/rad needs register 1000 = 2.94 N·m, which is not
    verified, so it is not used for the verdict.
- **Resolution.** At P 128 and 160 the deflections are 2–11 ticks, read as
  integers (1 tick = 0.088°). That is where the ± on the ratios comes from.

### Quiet

At every state of every P: the static position range was 0, overshoot on the
probe returns was at most 1 tick, and no status bit was set. The probes return
at 100 steps/s, so this is not a step response at full speed. The loads hang on
a string, so they do not couple to the lever as a rigid inertia.


## Run D: hold with a rigid inertia, a C-clamp (15:30–16:05)

The salt-cup lever was not printed, so a C-clamp went on the stiffness lever near
the 100 mm notch. Script: `2026-10-01/hold_clamp.py`, on ID 30.

**The clamp, as estimated:**
- **Mass:** about 0.8 lb (0.36 kg), Tom's estimate; there is no scale.
- **Torque:** at P 32 the probe midpoint sat 8.5 ticks below level. Against run
  C's P 32 stiffness that is about 0.45 N·m, which puts the clamp's centre of
  mass about 126 mm out.
- **Inertia:** about 0.006 kg·m², roughly 2.3× #73's "leg-like" 0.25 kg at
  0.1 m (0.0025). These are estimates.
- **Off-plane mass:** the clamp's centre of mass sits off the lever's plane. Tom
  observed that part of the shaking comes from that.

**Per P:**
1. With the arm hanging plumb (3081) and released: write the gains.
2. Torque on, then a quiet hold for 5 s and four 2° moves out and back.
3. Raise to level (2057), traced. Quiet hold, two slow probes, four 2° moves.
4. Lower to hanging, traced. Release.

Nothing is ever released with the clamp on unless the arm is within 60 ticks of
plumb.

**Moves.** The first pass used full-speed steps. Tom: "the jerkiness of the
motion is a real problem". After that, every move was acceleration-limited
(register 41 = 2, i.e. 200 steps/s²):
- 2° moves at up to 200 steps/s
- 90° travels at 100 steps/s

Each travel was sampled throughout. "Backward" counts the samples whose reported
speed had the wrong sign for the move.

| pass | P / D | holding still, level: range (ticks), moving samples | holding still, hanging | 2° moves: worst overshoot / residual range, settled? | raise: speed spread, backward samples | lower: speed spread, backward samples |
|---|---|---|---|---|---|---|
| full-speed steps | 32 / 32 | 0, 0/152 | 0 | 1 / 0, yes | 25, 0/308 | 70, 4/265 |
| smooth | 32 / 32 | 0 | 0 | 1 / 0, yes | 29, 0/306 | 80, 5/260 |
| smooth | 128 / 128 | 0 | 0 | 4 / 7, one never settles (hanging) | 140, 41/268 | 193, 84/276 |
| smooth | 160 / 160 | **13, 114/152** | 0 | 7 / 11, none settle | 139, 68/325 | 696, 176/395 |
| smooth | 32 / 32 | 0 | 0 | 1 / 0, yes | 29, 0/308 | 43, 0/290 |
| smooth | 128 / 32 | 0 | 0 | 3 / 0, yes | 201, 78/303 | 173, 60/277 |
| smooth | 160 / 32 | **18, 148/152** | 0 | 10 / 19, none settle | 243, 98/384 | 244, 132/359 |
| smooth, clamp refitted | 32 / 32 | 0 | 0 | 0 / 0, yes | 27, 0/308 | 71, 3/269 |
| smooth, clamp refitted | 96 / 32 | 0 | 0 | 3 / 1, yes | 117, 36/275 | 133, 37/243 |

- **Speed spread** is the standard deviation of the reported speed, in steps/s,
  about a commanded 100.
- **P 160 lowering with D = 160:** the load hit ±1000 (full scale), and the
  move ended 15 ticks over with a 29-tick residual range.
- **No status bit** was set in any pass.
- **Stiffness from the clamp.** Its probe midpoints at level were 7.5–8.5 ticks
  at P 32, 3.0 at P 96 and 2.5 at P 128. That is about 2.8× (P 96) and 3.0×
  (P 128) against P 32. Run C's bottles gave 3.5× at P 128.

Per-pass numbers are in `2026-10-01/hold_clamp_summary.json`. The video is on Mira:
`hw_sessions/2026-10-01/gain_bench_hold/`, `run_attempt1.mp4` 15:29–15:34,
`run2.mp4` 15:37–15:55, `run3.mp4` 16:00–16:04.

**Reading:**
- **P 160** limit-cycles holding still with this inertia, at D 160 and D 32
  alike.
- **D 32 against D = P at P 128:** the small moves settle with D 32 and ring
  with D 128.
- **The 90° sweeps** shake at every P from 96 up: tens of backward samples and
  load swings of ±200–400. P 32 sweeps hardly do (0–5 backward samples). Tom saw
  it most on the downward motion.
- **This inertia is about 2.3× #73's leg-like value,** so P 160 might behave
  differently with a true leg. These runs do not show that either way.

## Against #73

| #73 line | result |
|---|---|
| pass: ≥ 4× the P 32 stiffness, ≤ ±1 count at hold | 4.8 ± 0.6× only at P 160, which **oscillates** holding the clamp |
| conditional pass: 3× and quiet | P 128 / D 32: 3.5 ± 0.4× (bottles), about 3.0× (clamp). Quiet at hold and settles after 2° moves; 90° sweeps shake |
| hold with a leg-like inertia, no limit cycle | clamp at about 2.3× the leg-like inertia: none up to P 128 / D 32; **limit cycle at P 160** (D 160 and D 32) |
| step 1: stance shortfall ≤ 0.3° | **not run**; the v5 robot will not be repaired |

## Incidents

**Drop 1, about 12:24–12:27.**
- Run A ended (12:24:28) with a `release` while the bottle was still hung. The
  script had released between rungs, and the lever had held each time for the
  few seconds until the next torque-on.
- Webcam frames: at 12:19:41 the lever is level with the bottle hung. At 12:27:19
  the lever points steeply down and the bottle is on the floor.
- The servo then read 2803, 746 ticks (66°) down, torque off, status 0.
- There is no footage between those two frames, so the moment of the drop was
  not seen.

**Drop 2, about 12:35.**
- At Tom's request the lever was brought back under torque, from 2803 to 2059
  at 100 steps/s.
- It was then released, at 2065. It read 2988 within 2.5 s.
- The webcam view was blocked, so whether a bottle was on is not recorded.
- Tom's statement: "the level arm only falls with a bottle attached", and
  "friction will keep the bare arm in place". The bare arm did hold, released
  level, at every rung of runs B and C.

**Run D, attempt 1 (15:30–15:33).** While the clamped arm was lowered to hanging
(after the P 32 level block), the C-clamp ran into the clamp holding the stand
to the table. The servo then stopped answering (`id 30: timeout`), and the script
stopped with the torque state unknown. Tom cut power, refitted the clamps and
powered back on. The video is `run_attempt1.mp4`.

**What changed afterwards:**
- Every hardware session is filmed as video, from before the first command to a
  few seconds after the last (Tom).
- Nothing is released while a bottle may be hung; a loaded abort keeps torque on.
- The bare arm is released level; there are no lowering moves.

## Deviations from the tool

- Runs A–D used one-off scripts, kept in `2026-10-01/`, not
  `gain_bench.py`. Run D's `hold_clamp.py` was written there and runs from
  the repo root.
- Run B started on Tom's "go. now" before `gain_bench`'s servo-positioned
  mode had landed. Run C needed probes the tool did not have yet.
- The scripts reuse `gain_bench`'s guarded gains write, reads and step
  statistics. They import it through a path relative to `hw_sessions/`,
  where they ran.
- From this session on, Tom has given the bench scripts to the Mira session to
  own. Probes will move into `gain_bench.py`.

## Open

- **The decision #73 feeds:** whether the STS3250 is still needed, given its cost
  and availability. The bench evidence is above. It has no measurement of the
  STS3250 itself.
- **A true leg-sized inertia:** the salt cup, or about 0.25 kg rigid at 100 mm.
  It would show whether P 160 holds still with the inertia #73 specifies.
- **ID:** the servo is still ID 30. Rename it back to 11 when bench work on it
  is done.
- **Bottle mass:** weigh one bottle, if absolute stiffness is wanted.
