# STS3215 hip pitch / knee refit: hanging moves + the stride1 swing (2026-11-03)

*On the robot's first stride1, its leg swung about 1.6× faster than the plant's. The R hip
pitch reached −31° at +180 ms; the plant's reached −19°
([friction-dr](../friction-dr/2026-11-03-friction-dr.md)). On the hanging robot, single 60°
max-speed moves (`moves_fast_large.json`; fast config reg85 0 / reg81 1, other servos limp)
ran at:*

- *237–247°/s mean, 249–277°/s peak;*
- *present load 1000 (full duty) throughout.*

*The plant's law is `cap = stall · (1 − |qd|/w0)`, at 11.1 V with w0 3.74 rad/s (214°/s).
It cannot do this.*

***Refit, at the robot's 12.0 V bus (the flight's `voltage_dv`):** hip pitch and knee stall
×2.0 (5.9 N·m). No-load speed is ×1.325 (5.35 rad/s, 307°/s) for the hip pitch and ×1.281
(5.18 rad/s, 297°/s) for the knee. Under the stride's large goal steps, the fast servo's dead
time is 40 ms in grade.py's chain. That chain is the C2 shaper → TypedServo (dead, τ 33 ms)
→ plant PD.* With these values:

- *the hanging moves fit at 2.5 / 2.7° rms;*
- *the stride1 swing's first 10 ticks fit at 1.25° rms (R hip −29.5°, knee −33° at
  +180 ms; robot −31 / −31).*

Code: `refit.py` (the fixture and the fits), `refit.json` (the chosen point),
`refit_ridge.json` (the grid). The settings are:

- **training:** `--supply-voltage 12 --servo-envelope hip_pitch=2.0:1.325,knee=2.0:1.281`;
- **grader:** `grade.REFIT_PLANT` / `REFIT_SERVO`.

## The fixture

`refit.Hanging` is the plant XML (`bimo_biped_v6legs_meas.xml`) with these changes:

- the torso is welded 0.6 m up;
- the leg joints carry the graded plant's damping, armature (0.028) and frictionloss;
- the MJCF position actuators are zeroed;
- the moved joint runs walker_env's servo law at 2 ms;
- every other joint is limp, as in walker_env's released state: no torque, frictionloss
  0.35.

The zeroed actuators are the `kp 40` "ideal" model; walker_env's sts3215 model zeroes them
too. Left on, they hold the leg near 0°, and the first fit (stall ×2.3, w0 ×2.1) was
fitting against them.

## The moves pin a ridge; the stride picks the point

The fixed-stall fits (rms over 0–420 ms, both sides) lie along a ridge:

| stall × | hip w0 × (°/s) | hip rms | knee w0 × (°/s) | knee rms |
|---|---|---|---|---|
| 1.0 | 1.92 (444) | 3.96 | 1.61 (372) | 3.58 |
| 1.5 | 1.46 (338) | 2.77 | 1.41 (326) | 3.09 |
| 2.0 | 1.33 (307) | 2.47 | 1.28 (297) | 2.69 |
| 3.0 | 1.17 (272) | 2.06 | 1.17 (270) | 2.56 |

The moves alone fit any stall ≥ 1.5× about equally. The higher the stall, the closer w0 sits
to the plateau speed. The trade is the acceleration phase: the robot reaches about 230°/s
within about 60 ms, and that has to accelerate the plant's armature of 0.028 kg·m². So the
"stall" here is an effective low-speed torque *given that armature*, not a datasheet number.

The stride1 swing goes through grade.py's firmware-path episode (walk18_shp10sq_s128r24, μ 1).
Its rms over the first 10 walk ticks:

| stall × | dead 20 ms | 30 | 40 | 50 |
|---|---|---|---|---|
| 1.0 | 2.90 | 2.47 | 2.53 | 3.36 |
| 1.5 | 4.35 | 3.39 | 1.57 | 2.06 |
| 2.0 | 4.93 | 3.40 | **1.25** | 1.71 |
| 3.0 | 5.14 | 3.30 | 1.29 | 1.53 |

×2 / 40 ms is chosen: it is the lowest stall within 0.05° of the best stride rms. A higher
stall buys nothing the data asks for, and it over-strengthens the low-speed (stance) torque.

Not separable here:

- stall from armature;
- the dead time from the shaper's lag;
- the torque–speed *shape*. A flat-top law (full torque up to near w0) is the likelier
  physics; this is the linear law the plant has, refit.

Only hip pitch and knee are refit. The other STS3215s keep the old envelope at 12 V.

## What the refit plant does to walk18_shp10sq (`../friction-dr/sweep_refit.md`, 8 seeds)

| μ | torsion | trips (rate) | first trip (s after cmd) | peak \|gz\| | yaw @+180 ms |
|---|---|---|---|---|---|
| 1.0 | 0.02 | 4/8 | 0.96 | 3.75 | −4.8 |
| 0.5 | 0.005 | 8/8 | 0.90 | 3.00 | −7.7 |
| 0.4 | 0.005 | 8/8 | 0.66 | 3.67 | −8.9 |
| 0.3 | 0.005 | 7/8 | 0.88 | 3.42 | −9.3 |

With the swing matched, walk18 trips the flail guard on body rate, as on the robot:

- it trips even at μ 1, and at almost every seed on a slippery floor;
- peak |gz| is 3–4.4 rad/s (robot 3.9).

But not all of the robot's spin is reproduced:

- the yaw at +180 ms is −5 to −9° (robot −19°);
- the first trip comes later (+0.66–0.96 s; robot +0.35 s);
- nothing falls (the robot fell).

The remaining gap is in the stance foot's yaw. Candidates:

- the sole's real μ and torsion, which are unmeasured;
- the pads' lever arm (1.6× a flat sole's);
- the foot rolling onto an edge.
