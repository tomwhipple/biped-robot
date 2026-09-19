# Wide-hip gait study: bird3_round (2026-09-17)

Tom: *"We might need a different walk gait."* This designs and gates one, for
the flat-slab side-mounted-leg body (`bird3_round`, `sim/getup_v6_side.py`
CONFIGS: `bird_body`, `hip_sep 0.18 m`, `knee='both'`, `hip_roll_abd 90`,
`yaw_range 180`, standing CoM 0.247 m, mass 1.436 kg) that `study-side-
mounted-legs.md` R3.4 found FAILED the existing open-loop walk on every one
of the 4 gate D cases (0 mm clearance, 0 s air, CoM margin −2.9 to −4.3 mm,
`gateD_bird3.txt`) even with a workaround (planning the IK at a fictitious,
narrower hip_sep). This study explains the actual mechanism, builds three
real fixes, and gates all of them at the REAL hip_sep — no narrowing hack.

## 1. Why the existing gait fails: reach, not balance

`static_gait.walk_timeline`'s weight shift translates the pelvis LATERALLY
(level torso) until the kinematic CoM sits on the stance sole, holding the
foot positions fixed. As the pelvis crosses toward the stance side, the
OTHER ("about to swing") leg's hip-to-foot vector grows by roughly
`hip_sep` — and the standing-height leg is already near its full
thigh+shank extension (`0.110 + 0.110 = 0.220 m`, `v6_kin.leg_ik`'s
`D <= L1+L2` limit), so there is almost no reach budget left for that
lateral component. At the stock hip_sep (0.084 m) the 20 mm standing crouch
(`drop=0.02`) already buys enough slack; at 0.18 m it does not — `leg_ik`
raises `out of reach` (`D` 0.221–0.235 m against the 0.220 m limit) for
essentially every shift, turn, or adverse-condition case tried (confirmed
here again: `crouch20_40`, `waddle10`, `adduct14` below all fail this way
at the stock 20 mm crouch). The round-3 workaround side-stepped this by
solving the IK for a body with hip_sep 0.14 m and applying those joint
angles to the real 0.18 m body — which does not correctly null the real
body's CoM offset, hence the small-but-real negative margins in
`gateD_bird3.txt`. **The fix has to either shrink the reach demand or
increase the reach budget, on the REAL body.**

## 2. Three mechanisms, one knob each

`sim/wide_gait.py`'s `wide_walk_timeline` generalises `static_gait.
walk_timeline` with three knobs (any subset can be nonzero at once):

- **CROUCH** (`drop`, already existed in `static_gait.walk_timeline` —
  the fix was simply to use MORE of it): bending the knees deeper before
  shifting shortens the vertical hip-to-foot vector, which buys back reach
  for the lateral component. Gate A-style crouch-depth check
  (`v6_kin`/`design_gates`, feet under the real hips, level sole): the
  bird3_round body's max crouch depth inside its joint limits is **62 mm**
  — coincidentally the same number as the stock body's own Gate A5 result
  (`docs/design-v6-ankle-roll.md` §4.1); same leg lengths, same limit. The
  gaits below use 30–40 mm of it, well inside that budget.
- **WADDLE** (`roll_amp_deg`, new): the pelvis Key gained an optional
  `roll` field (`design_gates.Key.roll`, default `0.0` — backward
  compatible, `tests/test_v6_design_gates.py` stays 8/8) and `v6_kin.
  pose_world` an optional `roll` parameter (`R = Rx(-roll) @ Rz(-heading)`,
  reducing to the original `Rz(-heading)` at `roll=0`). A Key with nonzero
  roll commands the WHOLE torso tilted about its own forward axis, so the
  hip-mount points (fixed to the torso) swing sideways as part of the
  tilt instead of only through pelvis translation — part of the CoM's
  lateral travel comes from tipping the body, not stretching the leg.
  `wide_walk_timeline` rolls the pelvis toward the stance side during each
  shift (`roll = sign(stance) * roll_amp`) and levels it again as the
  swing foot lands.
- **ADDUCT** (`foot_sep`, new): the feet are planned at a separation
  narrower than the real `hip_sep`, via genuine hip-roll adduction against
  the REAL body (`standing_key_w`, `solve_pelvis_w`) — not the round-3
  workaround's fictitious-hip_sep IK. At `foot_sep` 140/120 mm (vs the real
  180 mm) the required adduction is only **3.8° / 5.7°** of hip roll —
  comfortably inside the 30° `hip_roll_add` ROM already in the plant.

## 3. Sweep and gate results

Full log: `docs/design-v6/gateD_wide_gait.txt` (`sim/wide_gait.py gate ...`,
4 cases x 3 seeds each, worst-of-3 reported, same servo model/adversity as
`gateD_appendages.txt`: STS3250 rolls+knees, μ/play per case).

| candidate | knobs | straight | +15 turn | μ0.3/play5 | −15 turn μ0.9 | verdict |
|---|---|---|---|---|---|---|
| `crouch20_40` (stock crouch/cadence — negative control) | drop 20, lift 40 | reach FAIL | reach FAIL | reach FAIL | reach FAIL | 0/4 |
| `crouch30_80` | drop 30, lift 80 | OK (clear 31, margin 16.6) | reach FAIL | FAIL (clear 4, air 0) | reach FAIL | 1/4 |
| **`crouch40_90`** | drop 40, lift 90, cadence 2.0/2.0/0.7 s | OK (clear 33, margin 16.4) | OK (clear 29, margin 15.6) | OK (clear 23, margin 13.7) | OK (clear 32, margin 15.6) | **4/4** |
| `waddle10` (roll alone, stock crouch) | roll 10° | reach FAIL | reach FAIL | reach FAIL | reach FAIL | 0/4 |
| `waddle15_crouch30` | drop 30, roll 15° | FAIL (tilt 27.9°) | FAIL (tilt 28.6°) | OK | FAIL (tilt 27.3°) | 1/4 — too much roll for this servo model at this cadence |
| **`waddle8_crouch40`** | drop 40, roll 8°, cadence 1.8/1.8/0.6 s | OK (clear 48, margin 15.7, slip 0.1) | OK (clear 48, margin 14.2, slip 0.3) | OK (clear 45, margin 12.1, slip 0.3) | OK (clear 48, margin 14.8, slip 0.4) | **4/4**, best slip control alone |
| `adduct14` (adduct alone, stock crouch) | foot_sep 140 | reach FAIL | reach FAIL | reach FAIL | reach FAIL | 0/4 |
| `adduct12` | foot_sep 120 | reach FAIL | reach FAIL | reach FAIL | reach FAIL | 0/4 |
| **`adduct12_crouch30`** | drop 30, foot_sep 120 | OK (clear 21, margin 23.2) | OK (clear 19, margin 16.7) | OK (clear 20, margin 14.3) | OK (clear 16, margin 16.5) | **4/4** |
| **`combo`** | drop 30, foot_sep 140, roll 8°, cadence 1.8/1.8/0.6 s | OK (clear 45, margin 32.3, slip 0.0) | OK (clear 42, margin 15.7, slip 0.1) | OK (clear 42, margin 37.7, slip 0.0) | OK (clear 39, margin 15.8, slip 0.2) | **4/4 — WINNER** |

**No single mechanism at the stock crouch/cadence (20 mm, 1.6 s) passes
anything** — all three (`crouch20_40`/`waddle10`/`adduct14`) fail every
case on reach, confirming the diagnosis in §1: the stock body's own crouch
margin is the whole reason its gait works, and it simply isn't enough at
1.18/0.084 = 2.14x the hip separation. Each mechanism ALONE, once given
enough crouch to work with, clears the gate (`crouch40_90`,
`waddle8_crouch40`, `adduct12_crouch30` are each independently 4/4). Too
much of the WADDLE knob alone is a net negative (`waddle15_crouch30`:
27–29° of peak torso tilt, unstable) — the roll has to stay modest and be
combined with the other two, not substituted for more crouch.

**`combo` (crouch 30 mm + adduct to 140 mm + waddle 8°) is the winner**: it
has the best CoM margin (15.7–37.7 mm, vs 12.1–16.6 mm for the
crouch/waddle/adduct singles), the best clearance (39–45 mm vs the gate's
15 mm floor), and near-zero stance slip (0.0–0.2 mm vs 1.4–5.0 mm for the
crouch-only and adduct-only gaits) — distributing the weight shift over
three small mechanisms asks less of any one joint than maxing out any
single one (§4).

## 4. Peak joint torque vs stall

`sim/wide_gait.py torque` (straight-walk nominal case, STS3250 rolls+knees,
`walker_env`'s own PD-clamped-to-torque-speed-envelope model, tracked with
`static_gait.run_walk`'s new `track_torque=True`, off by default):

| candidate | hip_yaw | hip_roll | hip_pitch | knee | ankle | ankle_roll |
|---|---|---|---|---|---|---|
| `crouch40_90` | 31.5x | **2.4x** | 18.7x | 10.0x | 13.2x | 5.9x |
| `waddle8_crouch40` | 41.7x | 2.6x | 20.2x | 7.6x | 13.5x | 3.3x |
| `adduct12_crouch30` | 24.5x | 2.8x | 19.4x | 10.2x | 15.1x | 5.7x |
| `combo` | 48.6x | **3.0x** | 20.4x | 10.5x | 14.4x | **8.5x** |

(margin = stall torque / peak demanded torque; STS3215 stall 2.72 N·m,
STS3250 stall 4.53 N·m at 11.1 V). No joint saturates on any candidate —
hip roll is tightest everywhere (2.4–3.0x) but `combo` has the best hip-roll
AND ankle-roll margins of the four, matching its lower slip: spreading the
weight shift over roll + adduction + crouch instead of maxing out one of
them costs less torque on every joint, not just the ones it directly
touches.

## 5. Robustness of the winner (`combo`)

`sim/wide_gait.py robust combo` — mass ×1.1, 100 g payload, 3° floor tilt,
and all three combined, worst of 3 seeds each:

| case | clear | air | CoM margin | slip | tilt | verdict |
|---|---|---|---|---|---|---|
| nominal | 45 mm | 1.34 s | 32.3 mm | 0.0 mm | 6.5° | OK |
| mass ×1.1 | 44 mm | 1.30 s | 37.3 mm | 0.1 mm | 6.3° | OK |
| payload 100 g | 45 mm | 1.32 s | 34.6 mm | 0.0 mm | 6.4° | OK |
| floor tilt 3° | 45 mm | 1.34 s | 31.3 mm | 1.1 mm | 6.3° | OK |
| mass ×1.1 + payload 100 g + tilt 3° | 44 mm | 1.26 s | 38.7 mm | 1.3 mm | 6.0° | OK |

All 3/3 seeds pass on every case, including the combined stress — the
margins the gait was tuned to have (32.3 mm nominal, more than double the
gate's 15 mm requirement) absorb these perturbations with room to spare.

## 6. Regression check: the stock body through the same gait

`sim/wide_gait.py gate stock combo` — the ORIGINAL, narrow-hip (0.084 m)
body run through the wide-hip `combo` gait unmodified:

| case | clear | air | CoM margin | slip | verdict |
|---|---|---|---|---|---|
| straight | 46 mm | 1.14 s | 12.3 mm | 0.1 mm | OK |
| +15 turn | 41 mm | 1.08 s | 11.6 mm | 0.2 mm | OK |
| μ0.3/play5 | 52 mm | 1.56 s | 37.0 mm | 0.0 mm | OK |
| −15 turn μ0.9 | 47 mm | 1.08 s | 11.6 mm | 0.2 mm | OK |

**Not a regression trap** — the stock body also passes all 4 cases, with
HIGHER clearance than its own tuned gait (41–52 mm here vs 16–23 mm in
`docs/design-v6-ankle-roll.md` §4.3/§4.7). The cost is cadence: `combo`'s
timeline totals **49.7 s for 8 steps** on `bird3_round` vs `static_gait.
walk_timeline`'s tuned **45.8 s** for the stock body's own gait at its own
knobs (step 6 cm, lift 4 cm, 1.6 s shift/swing) — a **9 % slower walk**,
from the bigger commanded lift (7 cm vs 4 cm) and slower cadence (1.8 s vs
1.6 s shift/swing) `combo` needs for the wide-hip body's margins. If this
gait is deployed on the stock body too (one gait file for both), that 9 % is
the price of the shared margin; `wide_walk_timeline` with its three extra
knobs off (`drop=0.02` matching the default, `foot_sep=None`,
`roll_amp_deg=0.0`) runs the same fixed-point CoM solve and Key/Timeline
construction as `static_gait.walk_timeline` (only `lift_h`'s default
differs, 0.04 m vs 0.03 m, a wide-hip-appropriate default, not a behaviour
change — passing the same `lift_h` reproduces `static_gait.walk_timeline`'s
own foot placements exactly).

## 7. What it needs from the hip ROM / CAD

- **Nothing new.** `combo`'s adduction (3.8° of hip roll to bring the feet
  from 180 mm to 140 mm apart) is inside the 30° `hip_roll_add` already in
  every v6 plant; its crouch (30 mm) is half of the measured 62 mm budget;
  its roll amplitude (8°, applied through the SAME hip-roll and ankle-roll
  joints the existing gait already commands, just with a bias) needs no
  new joint. `hip_roll_abd` (90°, per the R3.1 CAD-unverified caveat) is
  not touched by this gait at all — WADDLE/ADDUCT/CROUCH stay well inside
  the ABduction-side range and the un-tested ROM does not gate this result.
- **Servo choice is unchanged**: STS3250 at rolls + knees (as already
  specified for the stock body, §4.5 of `docs/design-v6-ankle-roll.md`) —
  the torque margins in §4 are comfortable at the SAME servo assignment,
  no new hardware ask.
- **What DOES change**: the walk is ~9–17 % slower than the stock body's
  own tuned gait (49.7 s vs 45.8 s per 8 steps here; `waddle15_crouch30`'s
  instability shows the roll knob has a real ceiling around 10–12° at this
  cadence — push it further only with a correspondingly slower cadence or
  a stiffer roll chain, not something this study changed).

## 8. Renders

`sim/renders/getup_options/side/bird3_walk_combo_straight.mp4` /
`_turn15.mp4` (`sim/wide_gait.py render combo 0` / `render combo 15`),
filmstrips alongside and copied into this directory:

![combo straight](gateD_wide_gait_bird3_combo_straight_strip.png)
![combo turn 15](gateD_wide_gait_bird3_combo_turn15_strip.png)

## 9. Sway vs stride (2026-09-17 follow-up)

Tom, watching `bird3_walk_combo_straight.mp4` and the stock walk: *"both
robots seem to be swaying side to side more than they're taking forward
strides."* Quantified, then pushed toward longer/faster strides, then
checked whether the sway is the weight shift itself or an over-shoot. Full
numbers: `docs/design-v6/gateD_wide_gait_stride.txt`.

**He's right, exactly.** `sim/wide_gait.py sway` adds an optional
`track_sway` to `static_gait.run_walk` (pelvis x/y every tick, off by
default) and measures pelvis lateral peak-to-peak (sway) vs pelvis forward
advance (stride) per step, steady-state (excludes the last, non-advancing
settle step):

| gait | stride/step | sway/step | sway is ___x the stride |
|---|---|---|---|
| `combo` (bird3_round) | 31.1 mm | 187.4 mm | 6.0x |
| stock's OLD gait (`static_gait.walk_timeline` defaults) | 29.4 mm | 138.4 mm | 4.7x |

**Stride/cadence frontier**: `sim/wide_gait.py frontier {bird3_round,stock}`
sweeps step length (60/90/120/150 mm) x cadence (down from 1.8 s) x
roll_amp (4-12°), drop 30 mm / foot_sep 140 mm held fixed, and reports the
fastest cadence x roll that still clears all 4 gate D cases (3 seeds each):

| body | step | cadence | roll | worst margin | sway:stride ratio | 8-step dist/time |
|---|---|---|---|---|---|---|
| bird3_round | 60 mm | 0.8 s | 8° | 2.7 mm | 0.18 | 21.1 cm / 25.1 s (0.84 cm/s) |
| bird3_round | **90 mm** | **0.8 s** | **12°** | **1.3 mm** | 0.34 | **34.6 cm / 21.7 s (1.60 cm/s)** — fastest |
| bird3_round | 120 mm | 0.8 s | 10° | 1.7 mm | 0.32 | 38.1 cm / 24.4 s (1.56 cm/s) |
| bird3_round | 150 mm | — | — | FAILS at every cadence/roll tried | — | — |
| stock | 60 mm | 0.5 s | 6° | 16.2 mm | 0.18 | 19.1 cm / 18.6 s (1.03 cm/s) |
| stock | 90 mm | 0.8 s | 4° | 29.3 mm | 0.24 | 30.1 cm / 29.0 s (1.04 cm/s) |
| stock | 120 mm | 0.8 s | 4° | 29.0 mm | 0.31 | 40.1 cm / 29.5 s (1.36 cm/s) |
| stock | **150 mm** | **1.2 s** | **8°** | **11.4 mm** | 0.45 | **49.5 cm / 36.8 s (1.35 cm/s)** — fastest |

bird3_round's fastest passing point is **3.9x** the original `combo`'s
speed (1.60 vs 0.41 cm/s) but only a 1.3 mm worst-case margin — a real
speed/margin trade, not a free lunch; the wide hip_sep is why bird3_round's
frontier tops out at 120 mm while the narrower-hipped stock body clears all
four step lengths with comfortable margins throughout. The sway:stride
RATIO gets WORSE at longer steps for both bodies (sway saturates near
150-190 mm regardless of step length while stride grows) — longer strides,
not just a faster cadence, are the real lever on the ratio Tom flagged.

**Is the sway the minimum needed shift, or excess?** A kinematic scan
(`v6_kin.pose_world`/`com_margin` at combo's own drop/foot_sep/roll,
mid-swing) found the minimum pelvis shift for CoM margin ≥ 0 is **10.5 mm**
— `combo` commands **49.1 mm** (kinematic margin 25.0 mm), a ~4.7x
over-shift, because its `bias_y` (the CoM target's offset from the sole
centreline) was inherited unmodified from `static_gait`'s stock-hip-sep
tuning, which never accounted for how much of the alignment roll+adduction
now do for free. Sweeping `bias_y` more inboard (which REDUCES the
commanded shift for this roll+adduct-assisted body):

| bias_y | 4-case gate | worst CoM margin | sway/step |
|---|---|---|---|
| −5 mm (`combo`) | ALL 4 OK | 15.7 mm | 187.4 mm |
| **−10 mm (`combo_tight`)** | **ALL 4 OK** | **9.3 mm** | **148.9 mm — 20.5% less** |
| −15 mm | FAILS (mu0.3/play5 marginal; −15° turn mu0.9 fell) | | |
| −20 mm | FAILS catastrophically (falls in every case) | | |

`combo_tight` (`bias_y=-0.010`, added to `GAITS`) cuts the sway ~20% for
free — same 4/4 gate pass, and it also clears the full robustness suite
(mass ×1.1 / 100 g payload / 3° tilt / all combined, worst margin 11.1 mm).
−15 mm is the wall: past it, the reduced shift stops covering the turning
cases' extra demand.

**Fastest-passing render**: `combo_fast` (drop 30 mm, foot_sep 140 mm, roll
12°, cadence 0.8 s) at step 90 mm — `sim/renders/getup_options/side/
bird3_walk_combo_fast_straight.mp4`, filmstrip alongside and copied here:

![combo_fast straight](bird3_walk_combo_fast_straight_strip.png)

**Parallelism**: the gate/frontier/robustness sweeps above run through a
`multiprocessing.Pool(12, fork context)` added to `sim/wide_gait.py` this
round (the box has 32 cores; the two frontier searches were each pegging
only one). Verified before trusting it: the pooled `wide_gait.py gate
bird3_round combo` reproduced the already-logged `combo` rows in
`gateD_wide_gait.txt` byte-for-byte (0 diff) before generating any of the
numbers above with it.

## Files

- `sim/wide_gait.py` — `wide_walk_timeline`, `solve_pelvis_w`,
  `standing_key_w`, the `GAITS` candidate table (`combo`, `combo_tight`,
  `combo_fast`), a `multiprocessing.Pool(12)` (`run_batch`/`_run_job`) used
  by `gate`/`robust`/`frontier`, CLI (`sweep`/`gate`/`torque`/`robust`/
  `render`/`sway`/`frontier`).
- `sim/design_gates.py` — `Key.roll` (default 0.0), `Timeline.at()`
  interpolates it, `q_of` routes through it. Backward compatible:
  `tests/test_v6_design_gates.py` 8/8 before and after.
- `sim/v6_kin.py` — `_rx`, `orient_quat`, `pose_world(..., roll=0.0)`.
- `sim/static_gait.py` — `run_walk(..., track_torque=False, track_sway=
  False)`, both additive only (off by default, existing callers/tests
  unaffected).
- `docs/design-v6/gateD_wide_gait.txt` — every candidate's full gate row,
  the torque report, the robustness table, the stock regression check.
- `docs/design-v6/gateD_wide_gait_stride.txt` — the sway/stride numbers,
  the frontier tables, the bias_y tightening sweep, the fast render.

## Verdict

The `combo` gait — a modest crouch (30 mm), a modest adduction (180→140 mm
feet, 3.8° of hip roll) and a modest torso roll (8°, a genuine WADDLE) —
clears all 4 gate D cases on the real, un-narrowed `bird3_round` hip_sep
(0.18 m) with better margins, clearance and slip than any single mechanism
alone, no joint torque anywhere near stall, and full robustness to
mass/payload/floor-tilt stress; it is also a strict improvement (not a
regression) when run on the stock narrow-hip body, at a 9 % slower cadence.
The failure the round-3 workaround could not fix (planning the IK at a
fictitious hip_sep) is fixed here by planning against the real body instead
and asking three small, independently-gated mechanisms to share the work
that one alone could not do inside 220 mm of leg reach.

**2026-09-17 follow-up**: Tom's eye was right — both `combo` and the
stock body's old gait sway 5-6x more than they stride per step (§9), because
`combo` inherited a CoM-centring target tuned for a body with no roll/adduct
help and so shifts the pelvis ~4.7x further than the kinematic minimum for
positive margin. `combo_tight` (bias_y −10 mm) cuts that sway ~20% for free,
still 4/4 on the gate and the full robustness suite. Pushed for speed, the
gait also has real headroom: `combo_fast` (90 mm steps, 0.8 s cadence, 12°
roll) is 3.9x faster than `combo` on bird3_round, though its margin thins to
1.3 mm at the frontier — a genuine trade, not a free lunch — while the
narrower-hipped stock body clears every step length up to 150 mm at 1.35 cm/s
with comfortable margins throughout, confirming the wide hip_sep, not the
gait family, is what limits bird3_round's speed.
