# Servo temperature and the over-temperature cutoff, from the flight records (2026-11-25)

*A dated record for #6 ("Next 3, thermal derating"). AGENTS.md: a sim term is modelled only after
the bench has measured it. This record measures what the recorded hardware data can, without
touching the robot: what the servos' temperature register did over 40 h of powered robot, a
first-order thermal model fitted to it, and the margin to the STS's 70 °C cutoff at the robot's
paces and at the training's targets. The model is now an opt-in sim term (`--servo-thermal`,
`sim/servo_thermal.py`). Tool: [`experiments/servo-thermal/`](../../experiments/servo-thermal/)
(`servo_thermal.py` surveys and fits, `regrade_thermal.py` grades; the fit's numbers are in
`fit.json` there).*

## Result

| | STS3215 (10 leg servos) | STS3250 (the 2 hip rolls) | how |
|---|---|---|---|
| thermal time constant τ_th | **8 min**, 95 % CI 7–11 | **14 min**, CI 12–16 | fitted, pooled per type, block bootstrap |
| thermal resistance Rth, against the sim's heat K_CU τ² | **16 K/W**, CI 12.7–23.1 | **30 K/W**, CI 24.1–32.9 | same |
| the same, against V × register 69 | 14.5 K/W | 11.6 K/W | same fit, the other heat |
| standby rise (powered, no load) above the room | 6.3–7.3 °C; the ankle rolls 8.8–9.2 °C | 6.2–6.5 °C | the seven cold starts |
| fit residual | 0.29–0.36 °C rms per servo | 0.31 °C | the register's own 1 °C step is 0.29 rms |
| the cutoff | 70 °C (register 13's default: torque off) | the same | servo-map §4; **register 13 not read on the robot** |
| mean heat that reaches 70 °C if held (22 °C room) | 2.56 W = 0.83 N·m rms = **28 % of the 12 V stall** | 1.37 W = 0.81 N·m rms = **16 %** | 41 °C / Rth |

**At the robot's paces: no servo comes near the cutoff.** Over 40 h of recorded robot, the hottest
reading after the spike filter is 36 °C (R ankle roll, the 11-11 stepper session); the 11-16 track
player sessions, the T4 endurance walk among them, peak at 32–33 °C. The worst 10-minute mean heat
of any servo is 0.62 W (R ankle roll, sim heat; 0.72 W as V × reg 69), which held forever settles
at 39 °C. The hip rolls' worst is 0.20 W (35 °C). That is about a 4× heat margin, 2× in torque.

**At the training's targets (CPU referee, the fitted model, a 22 °C room, servos starting at their
29 °C idle equilibrium, 4 seeds):**

| policy, scenario | hottest servo, mean heat | held, it settles at | to 70 °C: nominal / Rth at its CI top / a 28 °C room / both |
|---|---|---|---|
| w19k get-up, prone (45 s) | R hip pitch 1.57 W | 54 °C | never / never / never / 27 min |
| w19k get-up, supine_ph | L shoulder 1.79 W | 58 °C | never / 37 min / never / 15 min |
| prec93 walk, line_1m | **L ankle 2.19 W** | **64 °C** | never / 13 min / 54 min / 9 min |
| prec93 walk, speed_ladder | L ankle 1.88 W | 59 °C | never / 23 min / never / 13 min |
| prec93 walk, march_10s | **L hip roll (STS3250) 1.27 W** | **67 °C** | never / 55 min / 35 min / 25 min |
| prec93 walk, stand_10s | L ankle roll 0.48 W | 37 °C | never |

"Held" means the episode's mean heat sustained for as long as it takes; the get-up rows are that
get-up repeated back to back. One get-up raises the hottest servo 2.2 °C; one 10–22 s walk episode
raises it under 2 °C. No servo cut off in any graded episode.

**So: nothing trips within an episode, and nothing trips at today's paces. Continuous walking at
the training's targets settles the ankle pitch and the hip rolls 3–6 °C under the cutoff, which is
inside the fit's uncertainty and a warm room's shift: within the uncertainty the ankle pitch can
reach the cutoff after 9–13 min of continuous walking and the hip roll after 25–55 min.** That is
the endurance question #6 was kept open for, and it is now answerable per policy (the referee's
thermal block, below) rather than guessed.

**What the sim now models** (opt-in, bit-identical when off): each servo's register-63
temperature as the fitted node, and the cutoff: a servo at 70 °C goes limp for the rest of the
episode. **What it does not model**, because no record measures it: torque derating below the
cutoff, the cutoff's real threshold and recovery on these servos, and the winding's own hot spot.

**Recommendation for the planner (#137):** no thermal A/B night yet. Within a 10–45 s episode a
servo heats a few °C, so the term only acts on a robot that starts warm; the policy cannot see
temperature, so training on it amounts to random joint drop-out near the end of a long session.
Grade instead: a walk candidate's referee card with `servo_thermal` in its config (or
`GRADE_PLANT_TERMS=servo_temp=29`) carries per joint the minutes to 70 °C at that scenario's heat;
gate an endurance walk on it. If a night is spent on it anyway, the one-variable arm is
B = A + `--servo-thermal 60:69` (a robot that has worked long enough to be warm), judged on the
referee with `servo_temp=65`: falls, cut-offs and the thermal block's heat per joint, A against B.

## Method

**Data.** Every flight record in `hw_sessions/` on mira (read only), through the supply-resistance
tool's cache (`experiments/supply-r/supply_r.py extract`, `/tmp/s6r/cache`): in each 50 Hz tick the
firmware reads registers 56–70 of every fitted servo in one sync read, so each tick has per leg
servo the temperature (63, 1 °C steps), voltage (62), load (60, PWM duty) and present current
(69). Ticks where all twelve leg servos answered: 5.67 M, 40.4 h of powered robot, 2026-10-30 to
11-17. The robot was legs-only in all of them (bus 0–11 only, checked in the 11-16 record).

**What else exists, and why it is not used:**
- The stepper's `servos` rows (`hw_sessions/2026-11-11/stepper/*.jsonl`: 35 runs, 960 readings, max
  32 °C, R ankle roll; `2026-11-12/stepper`: 13 runs, 344 readings, max 31 °C, L ankle roll). They
  are a few readings per run, and the flight records hold the same sessions at 50 Hz.
- The track player's host logs (`track_host.py`, `experiments/roll-balance/results/track_*.jsonl`)
  carry no servo registers; they ran with the flight recorder on, which has them. The 11-16 track
  sessions (17:30–18:30Z, `cmd_endurance_T4` at 17:42Z) are in
  `flight_20261113T172054Z_prec16.frec`.
- The arm rig of PR #323 ran on the old controller and has no flight records. The power lite bench
  records (11-19..11-21) carry no servo feedback.

**Cleaning.** Register 63 reads wrong on 0.001–0.11 % of ticks per servo: single-tick values up to
150 °C, the same at any load (bus corruption of that byte). A 51-tick (1 s) running median removes
them; the cleaned register then moves in 1 °C steps. Data are binned to 1 s.

**Model.** One thermal node per servo:

  τ_th dT/dt = T_ss + Rth·P(t) − T,

with P the servo's heat and T_ss its idle equilibrium (the room plus the electronics' standby
heating). T_ss and the starting temperature are free in every chunk (a contiguous recording, cut at
1 h; 72 chunks), so the room's drift and the unknown power-off history drop out; τ_th and Rth are
common to all chunks. Output-error least squares: for each τ_th on a grid (1–60 min), the chunk
terms and Rth are linear. CIs: 60 resamples of whole chunks. The fit is pooled per servo type for
the sim, and per servo in `fit.json`.

**Which heat.** Two regressors were fitted:
- **The sim's own heat**, K_CU τ² (the bridge model's copper term, `sts_servo_model`,
  `current_budget.SERVO_ELEC`), with τ from the logged duty and speed through the sim's envelope as
  `supply_r.currents` prices it (τ = stall(V)·(d − ω/ω0(V)), d = −load/1000). Its Rth pairs with
  what the sim computes, so this is the one the sim uses (as the supply record paired its R with
  the sim's bridge current).
- V × register 69 (the servo's electrical input). Close for the STS3215 (14.5 against 16 K/W). For
  the STS3250 it reads 2.6× more heat than the sim's K_CU τ² (whose K_CU rests on the unverified
  4.2 A stall), so its Rth is 11.6 against 30. The two are the same temperatures; only the heat's
  scale differs.

**How well the load is identified.** Rth comes from within-chunk co-variation of temperature with
load. For the servos that work (hip rolls, knees, ankle rolls) the load term cuts the residual sum
of squares by 30–60 %, and the even and odd chunks fitted separately agree (hip rolls 11.4 / 11.8,
knees 14–18, ankle rolls 9.5–17.5 K/W on V × reg 69). For the hip yaws and pitches and the ankle
pitches the load is too small: their own Rth CIs span 3–49 K/W. A separate (shorter) time constant
for the load path fits no better (best ≥ 10 min), so one node is all the data support.

**The cold starts.** Seven sessions start after ≥ 6 h off (11-01, 11-03, 11-04, 11-10, 11-11, 11-13,
11-16): the servos read the room (17–25 °C) and rise 6–9 °C to idle equilibrium with almost no
load, with a power-on τ of 5–10 min. The ankle rolls run 2–3 °C warmer than the rest at idle in
every session.

## The sim term

`sim/servo_thermal.py`, used by `walker_env` and `env_mjx` (the `supply_sag.py` pattern: one
numpy / jax.numpy kernel):
- per control tick, P = the mean over its sub-steps of min(k·τ², the stall power at the supply),
  k = K_CU of the servo's physical type. Under an envelope pair (`--servo-envelope`, a refit of the
  servo that is there) k is scaled by (type stall / pair stall)², so a given duty heats as the fit's
  data did: full duty is the stall power whatever the envelope says the torque is;
- T' = T + (1 − e^(−dt/τ_th))·(T_idle + Rth·P − T), T_idle = 29 °C;
- a servo at 70 °C is cut off, latched for the episode: its PD output is zeroed from the next tick,
  and it draws nothing from the supply sag.

`--servo-thermal T` or `LO:HI` sets the start temperature, fixed or drawn per episode per servo
(CPU: after every other draw; MJX: `fold_in 7007`). config.json records it and the CPU referee
inherits it. With it on, `eval_precision.py` writes a `thermal` block per scenario into the
scorecard json: per joint the largest mean heat, rise and peak, cut-offs, and the fewest minutes to
70 °C were that heat held. `GRADE_PLANT_TERMS=servo_temp=<°C>` grades any run on it.

**Checks** (`tests/test_servo_thermal.py`; parity R29):
- Off, both envs are bit-identical to `main` d4c01c9: 60 DR steps of random actions in each env,
  float64 and float32 MJX, the default plant and the plant with `supply_sag_ohm` on (the sag's
  scan carry was restructured), compared array for array against a `git archive` of `main`
  (`experiments/servo-thermal/bitid.py`). On
  but below the cutoff, the CPU plant is bit-identical to off.
- R29: the kernel numpy vs jax (heat, step, latch; a refit pair included) to 0; airborne
  arithmetic with half the servos started 0.005 °C under the cutoff (6 of 17 cut off mid-run; qpos
  9.6e-12, qvel 7.3e-10); the temperature step for step (3.8e-6 °C, the MJX state's float32);
  liveness.
- The kernel: exact per-tick step, steady state, minutes-to-cutoff against stepping, the latch,
  the cap, the refit pair; the fit tool recovers a known τ_th and Rth through a 1 °C register;
  the kernel's constants are `fit.json`'s pooled fit, inside its CIs.

## What would settle the rest

None of this needs new logging: the flight recorder already writes registers 60, 62, 63, 65
(status) and 69 of every fitted servo at 50 Hz, and the CLI's read-only `reg <id> <addr>` reads
any register.

1. **The cutoff itself.** `reg <id> 13` (max temperature) and `reg <id> 19` (unload condition: does
   the temperature bit drop torque) on all 17 servos, torque off. Read-only, two minutes, no motion.
2. **Heat at the load the training targets, and derating.** One STS3215 and one STS3250, each with
   its horn pushing a fixed stop (so a cutoff cannot drop a load: bench safety), the torque limit
   (register 48) setting the duty: the STS3215 at 30 % and then 50 % (about 3 W and 8 W; the model
   reaches 65 °C in 12 and 3 min), the STS3250 at 20 % and 30 % (about 2 W and 4.5 W of the sim's
   heat; 13 and 4 min). Flight recorder on (50 Hz), held until register 63 reads 65 °C, then torque
   off and 20 min of cool-down logged. That measures Rth and τ_th at 4–10× the heat on record (the
   fit's main extrapolation), the hot cool-down, and, at constant duty, the stalled torque against
   temperature (a scale under the stop, or reg 69 at constant duty) — the derating this model
   leaves out. Once to 71 °C on the STS3215 shows whether the status bit and the torque-off arrive
   where register 13 says, and at what temperature torque returns.
3. **The robot at a target pace.** Any continuous 15-minute walk or march on the robot with the
   flight recorder on. At the referee's heat (prec93) the model predicts the ankle pitches at
   54–59 °C and the hip rolls at about 54 °C after 15 min (settling at 59–64 and 67 °C); one
   session checks that.

## Caveats

- **Register 63 is a board sensor**, not the winding: the winding runs hotter and faster. The
  cutoff acts on register 63, which is what the model tracks.
- **Extrapolated 4–20×.** The heaviest 10 minutes on record average 0.6 W; the training's targets
  sit at 1.3–2.2 W, and the cutoff needs 1.4–2.6 W held. Convection grows with temperature, so a
  linear Rth errs toward predicting the cutoff early.
- **The room** is not recorded. T_idle = 29 °C assumes the ~22 °C the cold starts read
  (17–25 °C); every degree of room is a degree off the margin.
- **The referee's heat includes falls**: march_10s and speed_ladder pass 0/4 for prec93, so part of
  their heat is pushing on the floor.
- **The duty→torque conversion** uses the datasheet envelope (as `supply_r` does). The robot's
  fast config runs above the datasheet's no-load speed, so moving joints' torque is understated
  there; holding dominates the heat.

## Reproduce

```bash
.venv/bin/python experiments/supply-r/supply_r.py extract --workers 10        # cache -> /tmp/s6r/cache
.venv/bin/python experiments/servo-thermal/servo_thermal.py bins               # -> /tmp/s6th/bins.npz
.venv/bin/python experiments/servo-thermal/servo_thermal.py survey             # survey.txt
.venv/bin/python experiments/servo-thermal/servo_thermal.py fit --heat sim --boot 60 --json experiments/servo-thermal/fit.json
.venv/bin/python experiments/servo-thermal/servo_thermal.py fit --heat reg69 --json experiments/servo-thermal/fit_reg69.json
JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/servo-thermal/regrade_thermal.py \
    --src /tmp/i6n/runs/robot_legs_prec93_march_both_sagdr_dw_s3 --start 29 \
    --scenarios stand_10s,march_10s,line_1m,speed_ladder --episodes 4
```
