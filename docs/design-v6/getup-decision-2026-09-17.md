# Get-up design decision (2026-09-17): two arms with elbows, and the paths not to walk again

Tom, 2026-09-17: *"write up these findings so we don't go down these paths
again. We'll settle on the two arms with elbows."*

This is the record of the 2026-09-14 to 09-17 get-up studies: what was
tried, what the sim measured, why each dead end is dead, and what the chosen
design still owes before CAD. Every number here traces to a log under
`docs/design-v6/` or a section of `design-v6-ankle-roll.md`,
`study-shoulder-arms.md`, `study-side-mounted-legs.md`,
`study-wide-hip-gait.md`. All runs: deploy servo model (2 Hz shaper + 80 ms,
play 3 deg, STS3250 at rolls + knees), whole body colliding with the floor,
and from 09-16 on `self_collide=True` (opt-out self-collision; results
before that could pass a limb through the torso). Videos, local only:
`sim/renders/getup_options/index.html` (25 clips, regenerate with
`sim/renders/getup_options/make_sheet.py`).

## 1. The decision

**v7 body + two 2-DOF arms (shoulder pitch + elbow), 16 cm upper arm + 16 cm
forearm, shoulder at the deck top, mounted 5 cm aft of the torso origin,
hanging straight down at the sides when idle.** Plant config:
`sim/getup_v6_shoulder.py` `CONFIGS["top_elbow_16_16_aft"]`
(`arms=True, arm_z=0.079, arm_len=0.16, arm_elbow=True, arm_fore_len=0.16,
arm_shoulder_x=-0.05`).

Why this one, measured (`study-shoulder-arms.md` §2-§5, logs
`getup_search_shoulder_{seat,elbow,robust}.txt`, `gateD_shoulder.txt`):

- Supine -> standing by the seat push (sit up with the arms folded up
  along the torso, plant the hands behind the hips, tuck, push while the
  ankles dorsiflex): STANDING, pelvis 0.387 m, robust 6/6 (play 5, mu
  0.3-1.0, servos 65-80 %). Peak shoulder 1.29-1.59 N-m, elbow 0.91-1.18
  N-m against the STS3215's 2.72 N-m simulated stall.
- It is the SHORTEST elbow arm that stands: 15+15 (0.30 m reach) 0/12;
  16+16 (0.32 m) stands; 18+18 (0.36 m) stands with more nominal hits.
  The threshold is end-to-end reach ~0.32 m: the hand must reach the floor
  behind the hips at hip level, which is where the tail pushed in the 09-14
  study.
- Idle pose shoulder 0 / elbow 0 (the actuators' rest) hangs the hand 146 mm
  above the floor, 38 mm outboard of the thigh axis. At the DEFAULT mount
  (x = -0.02) the hanging forearm hits the swinging leg 7327 times in an
  8-step walk and the body falls in 2 of 4 gate cases. At x = -0.05 (aft):
  31 contacts, all 4 gate cases OK, margins 30.7 / 12.2 / 35.6 / 12.7 mm
  (bare body 13.0 / 12.2 / 35.2 / 26.6). Wider (+20 mm lateral) barely
  helps (2907 contacts); aft is the fix.
- Cost: 4x STS3215, ~0.294 kg at the deck top (0.466 m above the floor
  standing), +4 obs/action channels, no new actuator type.
- Prone is NOT solved by the arms (push-up 0/122 at every length, §3); it
  stays the legs-only roll prone -> side -> supine of 09-14 (design doc
  §12.1), then the arm seat push.

## 2. Dead ends: do not retry these

Each line: what, on which body, how many, and the measured reason. If a new
idea reduces to one of these, the answer is already known.

### 2.1 Legs only, on the v7 torso (any joint ranges)

- **~150 hand-built sequences + continuous keyframe searches, 0 stood**
  (design doc §11, §11.3, §12.2; `getup_search_{1,2,3,flex130,kneel*,
  sideroll*,legroll*,skid*,lowtorso,birdknee}.txt`). Sit-up works every
  time; child's pose, downward dog, kneel-sit (135/135) work; every path
  dies moving the CoM from the seat or the knees onto the feet.
- **Why (measured):** the legs (0.335 m) are shorter than the torso + head
  (0.55 m); every rotation about a leg contact lands the head, the head is
  the fulcrum on the wrong side of the CoM and it skids. Below pelvis
  ~0.33 m no grounded pose puts the CoM inside a sole. Deep flexion
  (knee 130 / hip 125, now in the CAD) does not change this (0/32); a
  pelvis skid gives a stable SEAT, not a stander (§12.3); a lower hip
  stack (20+20, 10+10 mm) does not either.
- **The pincer** (legs splayed flat, swept together, which stands the flat
  bird body up): on the v7 torso, 12 hill-climb restarts with knee both
  ways, abduction 120, yaw 180: torso reaches fully upright but the rise
  stalls at pelvis 0.211 m on the shins, 176 mm short; the bird's exact
  path jams a hip roll at 4.5 N-m and ends on its side
  (`study-side-mounted-legs.md` R3.7, `getup_search_pincer_stock.txt`).
  Lever: supine CoM sits 0.096 m from the hip line on the v7 torso vs
  0.036 m on the slab.
- **From the kneel** (R3.8, `getup_search_kneel_rise.txt`): foot brace /
  half-kneel 0/192, because when the trailing shin lifts the CoM is 136-148
  mm outside both soles (sole 130 mm); knee pincer / sumo squat 0/96, the
  splayed base is stable but at pelvis 0.05-0.06 m (a 0.22 m leg cannot
  put a sole down from a 0.211 m kneel) and the close-and-rise saturates
  the hip roll at its 4.53 N-m stall; hill-climb from the kneel tops at
  0.211 m. The earlier half-kneel negative (§11 step 4, 0/16) stands.
- **RL shaping** for the get-up: two decisive negatives (1.3 B steps
  2026-07-28; v12 A/B 2026-08-08, 0/16). Kneel achieved, rise never.

### 2.2 Arms and appendages that do not work

- **Short arms at the deck / shoulder height (reach <= 0.30 m)**: 0/N in
  every study (09-14 `getup_search_arms{,2,3}.txt`; 09-16 seat sweep
  incl. a re-run of the 09-14 negative under self-collision). They push on
  the torso, not the floor behind the hips, and pivot the body over the
  head into a headstand. It was never the shoulder HEIGHT, it was the
  reach.
- **A third ab/adduction shoulder DOF**: 0/468 variants (`study-shoulder-
  arms.md` §2); its rest pose keeps the arm pressed to the torso under
  self-collision. Not fully diagnosed, but nothing to gain: 2 DOF stands.
- **One-arm side-seat push**: 0/160. **Prone push-up with any arm**: 0.
- **Arms mounted at the housing bottom** (9 cm above the hips): 0/36
  (design doc §12). Hip-level arms and the 20 cm tail DO work (11/12, 6/6)
  but the tail is rejected as a hypothetical robot (§12.3) and hip-level
  arms are superseded by the top-shoulder elbow arm, which hangs at the
  sides.
- **The elbow arm at the default shoulder x**: walks into its own leg
  (above). Always gate the hanging pose with self-collision on.

### 2.3 Side-mounted / bird layouts

- **Hips raised or outboard on the existing vertical stack** (rounds 1-2,
  incl. the stack tipped horizontal): 0/220, and above hip_z ~100 mm the
  torso belly grounds and removes the sit-up. Not the bird body; do not
  cite these as a bird negative (`study-side-mounted-legs.md` §1-§4,
  R2.1-R2.3).
- **The real flat bird body** (rounded 200 x 140 x 55 mm slab on the hips,
  hips at its sides, knee both ways, abduction 90, hip_sep 0.18, 1.44 kg,
  standing CoM 0.247 m): this one WORKS and is the documented alternative
  if the arm path fails on hardware. Verified (R3.2, R3.6,
  `getup_search_bird3_verify.txt`, `sim/getup_v6_bird3_verify.py`): one
  keyframe path stands it from supine (edge-pitch -> belly -> tuck -> stand,
  quasi-static at 3x slower, robust 7/7, peak 1.5 N-m at a hip roll) and its
  second half from prone (6/6); only the ROUNDED slab (the box tops out at
  0.209 m). Fall census (36 kicks): 12 standing, 24 on an edge, 0 supine vs
  the v7's 8 / 11 supine / 7 prone / 10 side; tip speed 0.66 vs 0.57 m/s.
  Walks on the wide-hip combo gait (below), 4/4 gate.
  **Why not chosen:** a new chassis, hip and gait against 4 servos bolted
  to the existing CAD; the wide hip caps its pace (fastest passing walk
  90 mm steps at 1.3 mm CoM margin, 150 mm steps fail, while the v7 body
  passes every step length to 150 mm on the same gait); 24/36 falls end
  on an EDGE state with no sequence yet; abduction 90-120, yaw 180 and the
  backward knee are sim-only ranges (no CAD hip drawn); the pod mass omits
  the driver, power module and neck. Hand-built sequences found NOTHING on
  it (0/~1100); only the continuous search did.

### 2.4 Walking gait findings that carry over to the v7 body

`sim/wide_gait.py` (`study-wide-hip-gait.md`): the stock static gait
shifts a level pelvis sideways until the CoM is over the stance foot, and
that shift exceeds the 0.220 m leg reach once hips widen. The `combo`
gait (30 mm crouch + feet adducted to 140 mm + 8 deg body roll) passes
Gate D on BOTH bodies; the v7 body passes step lengths 60-150 mm on it.
Both gaits over-shift: sway 138-187 mm per step for 29-31 mm of stride;
the minimum shift for a positive CoM margin is 10.5 mm; `combo_tight`
(bias_y -10 mm) cuts sway 20 % for free. Keep `Key.roll` (the pelvis roll
field added to `design_gates`/`v6_kin`/`static_gait`); the pinned gate
tests are unchanged (8/8).

## 3. What the chosen design still owes before CAD

1. **Fold-up swept volume**: the arm folds UP along the torso (shoulder
   180) during the sit-up; only end poses were checked against the head
   and torso. Sweep the motion under self-collision and against the CAD
   head (`study-shoulder-arms.md` §7 "not measured").
2. **The seat push on the CAD-inertial plant** once the arm parts exist
   (`sim/build_v6_inertia.py` path), incl. the hanging-pose walk gate.
3. **Hand pad**: modelled as a 12 mm rubber sphere; a wider pad was not
   tested.
4. **Fall direction**: backwards falls end supine (the arm path); forward
   falls end prone and go through the legs-only roll first (§12.1, 5/5 with
   the tail's timing; re-verify the roll with the arms hanging, since
   hip-level arms folded along the torso blocked it (0/5) and the top
   shoulder was not checked).
5. **obs/firmware**: +4 position actuators, +4 obs channels; walker_env
   sizes generically (nu = 17 ran unmodified), the firmware/beacon do not.
6. **Never on hardware without the sim margin**, one motion per go, on
   the floor (2026-09-13 rule).

## 4. Method lessons (so the next study is cheaper)

- **Model the actual body, not a proxy.** Two rounds were spent on the
  vertical stack with the hips moved; the flat slab took one. Render
  stills and check them before any sweep.
- **A hand-built negative is not a negative.** The flat body's get-up was
  0/~1100 by hand and found by a 6-node hill-climb in 6 restarts. Run the
  continuous search (`bird3search` style, full joint ranges) before
  declaring a path dead. Then verify the winner at 3x slower (quasi-static
  vs a flip) and across play / mu / servo strength.
- **`self_collide=True` always.** The opt-in pair list cannot see 28 of 62
  geoms; folded arms and splayed legs pass through the torso otherwise.
- **Reach beats height.** For any pusher, compute whether the contact
  can reach the floor at hip level on the CoM side of the pivot first.
- **Use the cores.** Every sweep goes through a 12-worker process pool
  (verified byte-identical to the serial logs); the first runs sat on one
  core of 32.
- **Gate the idle pose.** A part that hangs must be walked with
  self-collision on; the arm's default mount failed only there.

## Files

- `docs/design-v6-ankle-roll.md` §11-§12: 09-14 studies (legs only, deep
  flexion, tail / hip arms, prone roll, skid)
- `docs/design-v6/study-shoulder-arms.md` + `getup_search_shoulder_*.txt`,
  `gateD_shoulder.txt`: the chosen design
- `docs/design-v6/study-side-mounted-legs.md` (§1-§5, R2.x, R3.1-R3.8) +
  `getup_search_side*.txt`, `getup_search_bird3_*.txt`,
  `getup_search_pincer_stock.txt`, `getup_search_kneel_rise.txt`,
  `gateD_side*.txt`, `gateD_bird3.txt`
- `docs/design-v6/study-wide-hip-gait.md` + `gateD_wide_gait*.txt`
- Scripts: `sim/getup_v6.py`, `getup_v6_appendage.py`, `getup_v6_legs.py`,
  `getup_v6_skid.py`, `getup_v6_shoulder.py`, `getup_v6_side.py`,
  `getup_v6_bird3_verify.py`, `wide_gait.py`, `joint_puppet.py`
- Clips (gitignored): `sim/renders/getup_options/{shoulder,side}/*.mp4`,
  sheet `sim/renders/getup_options/index.html`
