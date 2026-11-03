# Foot–floor friction and the stride1 spin (2026-11-03)

*On the robot (2026-11-03 23:00:40 UTC), `robot_legs_walk18_shp10sq_s128r24` stood still,
then ran `stride1` (one gait cycle at vx 0.6). The right leg kicked fast and high. The body
yawed −19° in 180 ms (gyro z −3.9 rad/s peak), the flail guard tripped on body rate at
+350 ms, and the robot pivoted on its left foot and fell. Both hip yaw servos held their
targets. The question: does low foot–floor friction explain the spin?*

***Answer: not on its own.** In the sim, on the firmware path and the fast servo, forcing the
effective contact friction down to μ 0.3 (and even near 0) does not reproduce walk18's spin:*

- *yaw at +180 ms stays −3 to −7° (robot −19°);*
- *peak |gyro z| is 1–3 rad/s (robot 3.9);*
- *no fall, and no guard trip until μ ≤ 0.1 with torsion ≈ 0.*

*The policy's commands match the robot's within ~5° through +140 ms, so the policy does the
same thing. The plant responds differently: the robot's right hip pitch moves ~1.6× faster
than the sim's (−31° vs −19° at +180 ms). The sim reaches the robot's numbers only with
**both** a faster leg (instant servo set point) **and** low friction (μ ≤ 0.4, torsion
≤ 0.005): yaw −16 to −21° at +180 ms, rate trip.*

*Low friction is still a real, unmodelled gap. **walk16_act** (which also spun on the robot,
11-02) spins in the low-μ sim the way walk18 did on the robot: at μ 0.3, torsion 0.005 it
yaws −17° at +180 ms, peaks at 4.2 rad/s, and trips on rate at +0.36 s. Training could never
see that, because:*

- *the friction DR scaled only the floor;*
- *MuJoCo takes the MAX of the two geoms' friction;*
- *the sole pads are 1.0, so the effective μ was always 1.0–1.4.*

Code: `sweep.py` (the μ × torsion stride1 sweep; `TYPED` / `PLANT` env overrides for the
servo and plant), `../shaper-train/grade.py` (`set_contact`, yaw metrics, the low-friction
gate), `sim/mjx/env_mjx.py` + `sim/mjx/train_mjx.py` (`--contact-mu`, `--contact-torsion`),
`tests/test_contact_friction_dr.py`.

## 1. The sim reproduction

`sweep.py` runs grade.py's firmware-path episode, with these settings:

- stride1 at vx 0.6 at t = 2 s, for one 0.667 s gait cycle;
- the C2 shaper at 10 Hz, standclock coast π/2, no obsfreeze;
- the fast STS3215 (dead 20 ms, τ 33 ms) on the joint-fit plant.

`grade.set_contact` sets the sliding μ and the torsional coefficient on the floor, every sole
pad and the reference soles together, so the max-combined pair sees exactly that value.
There are 8 seeds per cell. "yaw @+180 ms" counts from the tick before the first walk goal,
which is the robot's +0 ms.

Robot: −19.2° at +180 ms, |gz| 3.9 rad/s, rate trip +0.35 s, fell.

### walk18_shp10sq_s128r24 and walk16_act_s128r24, μ × torsion (`sweep_walk18.md`)

| run | μ | torsion | falls | trips | peak rate EMA | peak \|gz\| | yaw @+180 ms |
|---|---|---|---|---|---|---|---|
| walk18 | 1.0 | 0.02 (plant) | 0/8 | 0 | 0.73 | 0.96 | −3.0 |
| walk18 | 0.6 | 0.005 | 0/8 | 0 | 0.88 | 1.67 | −4.1 |
| walk18 | 0.4 | 0.005 | 0/8 | 0 | 0.98 | 1.72 | −4.7 |
| walk18 | 0.3 | 0.005 | 0/8 | 0 | 1.08 | 1.66 | −5.8 |
| walk16_act | 1.0 | 0.02 | 0/8 | 8 (pose) | 1.25 | 2.24 | −3.7 |
| walk16_act | 0.4 | 0.005 | 0/8 | 8 (rate 8) | 2.43 | 3.74 | −14.1 |
| walk16_act | 0.3 | 0.005 | 0/8 | 8 (rate 8, first +0.36 s) | 2.75 | 4.24 | −17.2 |

Below μ 0.3 (`sweep_walk18_extreme.md`), walk18 trips on rate only with torsion ≈ 0 and
μ ≤ 0.1, at +0.44–0.62 s. Its yaw at +180 ms never passes −7.7°. The leg's swing momentum
bounds the yaw: even a frictionless foot only lets the body counter-rotate by that much.

### The leg is faster on the robot

The stride1 goals (R knee, deg, per 20 ms tick) are the same:

- robot: −22 −50 −69 −88 −96 −97;
- sim: −23 −53 −74 −90 −96 −96.

The measured R hip pitch / knee (deg) are not:

| | +100 ms | +140 ms | +180 ms | +240 ms | +280 ms |
|---|---|---|---|---|---|
| robot | −11 / −10 | −21 / −20 | −31 / −31 | −44 / −44 | −49 / −43 |
| sim (fast servo, μ 1) | −8 / −9 | −14 / −17 | −19 / −25 | −29 / −36 | −36 / −42 |

The servo's set-point lag is not what limits the sim. With the set point instant
(`TYPED=dead15=0,tau15=0`) the hip is −20° at +180 ms, and raising kp does not speed it up.
So the plant's torque–speed limit under this swing is slower than the robot's. With that
instant set point and low friction (`sweep_fastservo.md`, 4 seeds), the sim reaches the
robot's scale:

| μ | torsion | yaw @+180 ms | peak \|gz\| | rate trips |
|---|---|---|---|---|
| 1.0 | 0.02 | −5.3 | 1.63 | 0/4 |
| 0.4 | 0.005 | −16.0 | 2.49 | 0/4 |
| 0.4 | 0 | −17.0 | 2.59 | 1/4 |
| 0.2 | 0 | −20.8 | 2.63 | 2/4 |

These runs also trip on chatter, and some trip before the command. That comes from the
unfiltered set point, not from the spin, so this sweep only diagnoses; it is not a gate.

## 2. The foot contact in the plant

The settings (`bimo_biped_v6legs*.xml`):

- **Contact geoms.** Each foot touches the floor through 8 spheres of radius 3 mm (`pad`
  class). They sit on the sole's perimeter: x −0.055..0.075, y −0.0235..0.0605 in the foot
  frame, contype 1, friction `1 0.02 0.001`, condim 4.
- **The floor** is `1 0.02 0.001`, condim 4.
- **Non-colliding geoms.** The `L_sole`/`R_sole` box (13 × 8.4 cm) and the TPU sole mesh do
  not collide; walker_env reads the sole box for references and clearance.
- **Training uses `floor_contacts feet`.** Only the pads touch the floor; the grader and
  referee use `all`.
- **The `fallcol` geoms** are μ 0.8. Only those default to 0.8; the pads are 1.0.

This gives four findings:

- **The friction DR never went below 1.0.** MuJoCo, MJX and MuJoCo Warp
  (`collision_core.py`: `wp.max(geom_friction[g1], geom_friction[g2])`) combine a pair's
  friction as the elementwise max. `friction_range 0.4` scaled the floor to 0.6–1.4, so the
  pad–floor μ was 1.0–1.4. `tests/test_contact_friction_dr.py::test_off_by_default…` pins
  this.
- **The torsional term is a large yaw brake.** MuJoCo's torsional coefficient is a length:
  each contact resists spin with up to μ_t·N. At 0.02 m that is a 2 cm patch radius on a
  3 mm sphere, whose real patch is about a millimetre. The yaw grip of a planted foot is
  about (μ · r̄ + μ_t) · N, where r̄ = 0.068 m is the pads' mean distance from their
  centroid:
  - at μ 1.0, torsion adds 29 %;
  - at μ 0.4 it adds 74 %;
  - torsion is not scaled by μ, so at μ 0.02 it alone holds the foot (sweep: −1.2° yaw at
    torsion 0.02 against −7.2° at torsion 0).
- **The perimeter pads overstate the lever arm.** A flat sole under uniform pressure has a
  mean radius of 0.042 m; the pads' is 0.068 m. Even at torsion 0, the sim's foot resists
  yaw about 1.6× more than a uniformly loaded TPU sole at the same μ. When the foot rolls
  onto one edge, only one pad row carries load, which the pads do model.
- **`mimic_sole_level`** (on in walk18's env) solves the imitation reference's ankle pitch so
  the swing sole stays level. It shapes the reference only, not contact.

No condim 6 / rolling friction is used, and none is needed.

**Not measured yet:** the sole's sliding μ and torsional friction on the board. AGENTS.md
"Calibrate the sim where the question lives" applies. Two bench tests would settle it:

- the tilt-board angle at which a loaded foot slides (μ = tan θ);
- the yaw torque at which a standing robot's foot breaks away (a spring scale on a lever).

## 3. The training fix

Two settings in `env_mjx.domain_randomize`:

- **`--contact-mu lo,hi`**: one effective sliding μ per env, U(lo, hi), set on the floor
  AND every sole pad. It replaces the floor-only draw.
- **`--contact-torsion lo,hi`**: one torsional coefficient per env, on the same geoms.

Both are batch-level (one draw per parallel env, like the other model DR) and off by
default; the batched model is unchanged then. The ranges used are μ 0.35–1.1 and
torsion 0.001–0.02:

- the sliding range spans the guessed TPU-on-wood μ 0.4–0.6 and the old 1.0;
- the torsion range spans a realistic small-pad value up to the plant's old 0.02.

No yaw-rate penalty was added. The command tracking already pays for yaw rate
(`w_track_w 2.0`), and the gate below can say whether one is needed.

**The gate** (grade.py): two probes, both stride1 0.6 with the effective contact friction
forced on the floor and the pads. stride1 cells also report yaw at +180 ms.

- `stride1 0.6 mu0.4`: μ 0.4, torsion 0.005;
- `stride1 0.6 mu0.3`: μ 0.3, torsion 0.002.

It separates walk16_act (trips on rate 8/8, yaw −14 / −17°) from walk18 (0/0, −4 / −5°).
It cannot catch walk18's robot spin, because the plant's leg is too slow (§1). Pass means "no
trip, yaw at +180 ms within a few degrees of the μ 1 value".
