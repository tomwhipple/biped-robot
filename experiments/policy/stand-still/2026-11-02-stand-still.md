# Stand still (2026-11-02): the stand jitter is the gait clock

*At the zero command the v6legs student (`walk14_s1_s128r24`) stood stably on the ESP32 with
`obsfreeze gain=0.5` + `shape 6`. It stood for 2 min with factory servos and for 49 s with the
fast servo setting (reg 85 = 0, reg 81 = 1). In both runs every joint goal jittered
0.1-0.5 deg rms, at exactly 1.5 and 3 Hz. That is the gait clock: it keeps running at the
stand command, and the policy reproduces it. The jitter is also where the power goes: the
armed stand draws 0.79 W of motor power, but a static hold of the same pose draws 0.13 W.
In sim, freezing the clock removes the jitter without costing margin. The firmware option for
that is `standclock coast` (hold the phase at π). It is off by default until hardware-tested.
A stand-trained fine-tune is in §5. The 16 late ticks came from one servo missing its
SYNC READ reply, and the read then waiting 18 ms (§6).*

Recordings: `hw_sessions/2026-11-02/flight_20261102T160708Z_policy_stand2*` (factory servos,
armed 16:07:41-16:09:46 UTC) and `flight_20261102T162925Z_policy_stand3_fast*` (fast servos,
armed 16:31:18-16:32:08). Code: `experiments/policy/stand-still/` (CPU only).

## 1. The robot

`jitter.py` takes the written goal (policy frame, 0.5 Hz high-pass) and the body rates from
the 5 s after the arm until the stop. Power is |present current| (6.5 mA/LSB) × 12.0 V,
summed over the 12 servos. That is motor current only; the register is a magnitude.

| stand | goal jitter mean / max (deg) | 1.5 Hz / 3 Hz band (deg) | body rate x / y / z (rad/s) | motor power (W) |
|---|---|---|---|---|
| stand2, factory servos, knobs | 0.275 / 0.466 (L hip pitch) | 0.193 / 0.133 | 0.046 / 0.094 / 0.114 | **0.79** |
| stand3_fast, fast servos, knobs | 0.271 / 0.436 | 0.194 / 0.136 | 0.041 / 0.084 / 0.093 | **0.75** |
| stand2: the soft STOP's hold (same pose, torque on, policy off, 2 s) | — | — | — | **0.13** |
| limp (torque released) | — | — | — | 0.06 (register floor) |

- **Power share.** The policy's motion accounts for (0.79 − 0.13) / (0.79 − 0.06) ≈ **90 %**
  of the above-floor motor power. Holding the pose statically costs 0.07 W. Averaged per
  gait phase, armed power runs from 0.45 to 1.06 W: the clock modulates it.
- **By joint (armed, stand2).** L ankle roll 0.23 W, R ankle roll 0.15, both ankles 0.08,
  L hip pitch 0.08, the rest 0.003-0.04. In the hold, only the ankle rolls draw anything
  (0.09 / 0.03 W).
- **Which joints carry the clock.** In the current spectra, the L ankle roll has 67 % of its
  variance at 1.5 Hz, and the R ankle roll 50 % at 3 Hz.
- **Servo setting.** The fast servos did not change the jitter (it is in the commands, not
  the servo). Without knobs, the fast servos diverged in < 1 s on the robot (flail guard at
  1.04 s).

## 2. Sim repro

`stand_sim.py` runs the firmware arm path from `rca/takeover.py`: history from the first
frame, prev_action 0, the shaper seeded at measured q, and a 1 s ramp from the stand2 arm
pose. It adds the bench knobs (gyro obs x0.5, a 6 Hz C2 shaper) and the measured actuator
(`rca/simh.TypedServo` on the fitted plant, with JF_PLANT stiffness, damping and friction).
The STS3215 response is a parameter:

- **slow**: factory, dead time 68 ms, τ 55 ms (`MEAS_SERVO`);
- **fast**: reg 85 = 0 / reg 81 = 1, dead time 20 ms, τ 33 ms. This matches the re-measured
  loaded steps: dead time 18-27 ms, τ63 30-60 ms (`stepresp_fast_fit.json`).

Each episode runs 20 s at the zero command. Then the goal is frozen for 2 s: that is the
sim's static hold at the same pose. Power is walker_env's proxy `info["power_w"]` (positive
mechanical work + 3.75 W/(N·m)² × τ²). Each row is 8 seeds.

| | goal jitter mean / max | 1.5 / 3 Hz | body rate x / y / z | power / static hold (W) |
|---|---|---|---|---|
| robot stand3_fast (fast, knobs) | 0.271 / 0.436 | 0.194 / 0.136 | 0.041 / 0.084 / 0.093 | 0.75 / — (motor current) |
| **sim, fast, knobs, clock 1.5 Hz** | 0.200 / 0.342 (R hip roll) | 0.160 / 0.084 | 0.028 / 0.016 / 0.010 | 1.70 / 0.61 |
| sim, slow, knobs, clock 1.5 Hz | 0.189 / 0.347 | 0.155 / 0.076 | 0.029 / 0.015 / 0.009 | 1.65 / 0.63 |

**What matches and what doesn't:**

- **Goal jitter: reproduced.** Same frequencies, ~75 % of the size, the same top joints.
- **Body rates: 1.5-9x too small in sim**, most of all in pitch and yaw at 3 Hz. The robot's
  3 Hz body mode is under-modelled. The first-stand RCA saw the same thing in roll (robot
  1.92 vs sim 0.90 rad/rad).
- **Power: the composition differs.** On the robot 90 % is motion. In the sim proxy, 64 % is
  motion and the rest is pose-dependent copper loss. So the proxy ranks static poses too
  harshly; read it as a relative number.
- **Margin: the sim is optimistic.** Fast servos with no knobs stand in sim (0/8 falls) but
  fall 8/8 at loop gain x1.5. The robot diverged at x1. So the robot's loop gain is about
  1.5x the sim's, and a candidate needs **≥ x1.5 in sim** to be a hardware candidate.

## 3. Firmware candidates for the current student (no retraining)

Every candidate keeps prev_action raw. Each cell is falls out of 8. The probes are:

- **stand**: 20 s at the zero command;
- **x1.25 / x1.5 / x2**: loop gain, i.e. the action deviation from its 0.5 s mean amplified
  on the way to the servos;
- **push**: kicks at 3 / 5.5 / 8 s, in m/s;
- **walk → stop**: 4 s at 0.15 m/s, then the stand.

**Fast servos, knobs on** (`coast_fast.md`, `fw_candidates_fast.md`, `knobs_fast.md`):

| candidate | goal jitter mean / max (deg) | 1.5 / 3 Hz (deg) | body rate x / y / z | power / hold (W) | x1.25 | x1.5 | x2 | push 0.4 | push 0.5 | walk → stop | reg-58 dq | slow servo | gyro x1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| as run: clock 1.5 Hz | 0.200 / 0.342 | 0.160 / 0.084 | 0.028 / 0.016 / 0.010 | 1.70 / 0.61 | 0 | 0 | 8 | 0 | 8 | 0 | 0 | 0 | 0 |
| (a) freeze at the stop phase | 0.068 / 0.135 | 0.024 / 0.017 | 0.003 / 0.005 / 0.004 | 1.75 / 1.79 | 0 | 0 | 8 | 0 | 6 | 1 | 0 | 0 | 0 |
| **(a') coast → π** | **0.050 / 0.116** | **0.016 / 0.012** | **0.002 / 0.004 / 0.003** | **1.05** / 0.87 | 0 | 0 | 8 | 0 | 8 | 1 | 0 | 0 | 0 |
| coast → −π/2 | 0.064 / 0.116 | 0.022 / 0.016 | 0.003 / 0.003 / 0.003 | 1.03 / 0.95 | 0 | 0 | 8 | 0 | 8 | 1 | 0 | 0 | 0 |
| (b) ramp clock 1.5 → 0 Hz over 2 s | 0.050 / 0.108 | 0.016 / 0.012 | 0.002 / 0.004 / 0.003 | 1.03 / 0.88 | 0 | 0 | 8 | 0 | 8 | 1 | 0 | 0 | 0 |
| (b) slow clock 0.25 Hz | 0.142 / 0.220 | 0.021 / 0.014 | 0.020 / 0.021 / 0.023 | 1.71 | 0 | 0 | — | 0 | — | 1 | — | — | — |
| (b) slow clock 0.5 Hz | 0.262 / 0.443 | 0.050 / 0.014 | 0.029 / 0.026 / 0.019 | 1.68 | 0 | **4** | — | 0 | — | 0 | — | — | — |
| (c) action low-pass 1 Hz | 0.106 / 0.200 | 0.094 / 0.029 | 0.008 / 0.007 / 0.006 | 1.44 | 0 | 0 | — | 0 | — | 3 | — | — | — |
| (c) action low-pass 2 Hz | 0.148 / 0.272 | 0.129 / 0.050 | 0.017 / 0.010 / 0.008 | 1.55 | 0 | 0 | — | 0 | — | **8** | — | — | — |
| (c) target deadband 1 deg | 0.020 / 0.054 | 0.009 / 0.003 | 0.001 / 0.003 / 0.002 | 1.54 / 1.75 | 0 | 0 | 4 | 0 | 8 | 2 | 0 | 0 | 0 |
| (c) action notch 1.5 + 3 Hz | 0.045 / 0.075 | 0.007 / 0.002 | 0.002 / 0.003 / 0.003 | 1.24 / 1.41 | 0 | 0 | 1 | 0 | 8 | **7** | 0 | 0 | 0 |

**Fast servos, NO knobs** (gyro x1, shape 10; `knobs_fast.md`). On the robot this set-up
diverged in < 1 s:

| candidate | goal jitter mean / max | body rate x / y / z | power | x1.25 | x1.5 | push 0.4 | walk → stop |
|---|---|---|---|---|---|---|---|
| clock 1.5 Hz | 0.229 / 0.366 | 0.033 / 0.021 / 0.014 | 1.82 | 1 | **8** | 0 | 0 |
| coast → π | 0.083 / 0.186 | 0.003 / 0.007 / 0.006 | 1.16 | 0 | 3 | 3 | 3 |
| gyro x0.5 only (shape 10), coast → π | 0.066 / 0.152 | 0.003 / 0.005 / 0.004 | 1.10 | 0 | 0 | 0 | 1 |

The same tables for the slow servos (`*_slow.md`) give the same ranking. The jitter and power
numbers are within 10 %, and the as-run x1.5 is 1/8.

**Findings:**

1. **Stopping the clock removes the jitter (a, a', b).** Goal jitter drops 3-4x and the
   1.5 / 3 Hz bands drop ~10x. Body rates drop 5-10x. Loop-gain margin is unchanged with the
   knobs (x1.5 0/8; x2 8/8 as before) and better without them (x1.5 3/8 vs 8/8). Push
   recovery is unchanged at 0.4 m/s. Holding a constant phase is out of distribution only in
   the history: three equal frames, where training always advanced 0.16-0.22 rad per tick.
   The sim shows no sign that this matters.
2. **The frozen phase sets the power** (`phases_*.md`, freeze at 8 phases). The policy's mean
   pose depends on the phase. Near π and −π/2 the proxy reads 1.0-1.2 W; near 0 and −π/4 it
   reads 2.3-2.8 W, which is more than with the clock running. A plain freeze holds wherever
   the stand began. At the arm that is phase ≈ 0.2, which costs 1.75 W. So the firmware
   **coasts**: the clock keeps the trained rate until the phase reaches the stand phase
   (π, in the middle of the low basin), then holds. The policy never sees the clock jump or
   slow. The 2 s ramp lands near π by luck, and on the robot it would land wherever the
   integral ends.
3. **A slow clock (b) is not a fix.** At 0.25-0.5 Hz the motion becomes a slow sway at the
   same amplitude (0.14-0.26 deg rms), and 0.5 Hz loses margin (x1.5 4/8).
4. **Output filters (c) cost the walk.**
   - The notch is the best filter (0.045 deg, x2 1/8), but it kills the walk → stop
     transition (7/8 falls). The low-pass at 2 Hz does the same (8/8).
   - The 1 deg deadband is the stillest (0.020 deg). But it is a hysteresis in the loop, the
     kind that makes limit cycles on real servos with play, and it is 2/8 on walk → stop.
   - All three only treat the symptom: the clock still drives the policy underneath.

**Recommendation for the current student: `standclock coast` (phase π).** Implemented
(firmware commit `f6e9224`), default **off**. It is plain-stand gated exactly like training's
`clock_stand_freeze`. Under any locomotion command or lift the clock runs as trained.

## 4. Next hardware test

Fast servos (reg 85 = 0 / 81 = 1), `walk14_s1_s128r24`, tethered, hard board, zero command:

```
obsfreeze gain=0.5
shape 6
standclock coast 3.1416
run
```

Expected from sim:

- goal jitter 0.27 → ~0.07 deg rms (the sim's 4x reduction applied to the robot), with no
  1.5 / 3 Hz peaks;
- body rates several times lower;
- motor power toward the 0.13 W static hold, from 0.75 W.

As an A/B in the same session, `standclock run` reproduces today's stand. Keep 10 s stages,
the spotter, and the fall and flail guards. If it is still, `standclock freeze` vs
`coast 3.1416` shows on the robot how much the stand phase matters for power.

## 5. Training fix: stand-trained fine-tune

**Recipe:** robot_legs_walk16_act's recipe (actuator DR, PR #189) plus
`--clock-freeze-stand --w-still 0.1`. The loco track ran 52 jobs with these two flags; the
legs track had dropped them. The STS3215 dead-time draw is widened to
`--act-delay-types sts3215=0:5,sts3250=0:1` (0-100 ms), so the student is valid for both servo
settings. 300M steps, warm-started from walk16_act, then the stock 24-round distill to
s128r24. Run in today's GPU window (`gpu/stand_gpu.sh`, runner lock).

RESULTS_PLACEHOLDER

## 6. Late ticks

Every one of the 16 late ticks in stand2 (worst 29.1 ms) has the same signature (`work_us`,
`fb_read_us` per tick in the recording):

- the feedback read took exactly **20.02-20.03 ms**, against 3.5 ms normally;
- servo id 11 (R ankle roll) had not answered: `flags` 30, while the 11 others answered
  (31);
- the tick took 28.3-28.8 ms (read 20 + policy 7.7 + the rest 0.7). The next tick started
  late, which shows as 28.6-29.2 ms periods.

Nothing else is involved:

- **not Wi-Fi or the recorder:** `net` in `stat` is the policy forward (7.7 ms on every
  tick), there were no ring or net drops, and the ticks around the late ones are 11.8 ms;
- **not the operator stop frames, not retries.**

The misses are irregular, 28-1445 ticks apart, 0.25 % of ticks. Id 11 is the servo that lost
the 11th reply of a 12-servo burst in 2026-10-30's `sread`.

**Cause.** The SYNC READ goes out in chunks of 6. Each chunk's deadline was
`reply_timeout (3 ms) × 6 = 18 ms`. A missed reply in chunk 2 therefore cost 18 ms, even
though id 12, listed after id 11, had already answered.

**Fix** (`scsbus/bus.cpp`, commit `f6e9224`):

- the servos answer in list order, so a burst ends as soon as the last listed servo has
  answered;
- a burst's deadline is 1.5 ms + 0.4 ms per servo: 3.9 ms for 6, against 1.7-2.1 ms
  measured.

With the fix, today's miss costs nothing. A missing *last* servo costs ≤ ~5 ms, so the tick
stays at ≤ ~17 ms. Host tests replay the exact miss pattern. The fast-servo run
(stand3_fast) had 1 late tick in 2445, with the same signature (19.9 ms read, 28.2 ms tick).

## Reproduce

```bash
cd experiments/policy/stand-still           # CPU: CUDA_VISIBLE_DEVICES= JAX_PLATFORMS=cpu
python jitter.py                            # robot baselines -> robot_baseline.json
SERVO=fast HARD=1 python stand_candidates.py 8 coast_fast.json coast
SERVO=fast HARD=1 python stand_candidates.py 8 knobs_fast.json knobs
SERVO=fast ONLY=stand python stand_candidates.py 8 phases_fast.json phases
SERVO=fast python stand_candidates.py 8 fw_candidates_fast.json fw
SERVO=fast HARD=1 RUNS=<run>,<run> python stand_candidates.py 8 students_fast.json students
```
