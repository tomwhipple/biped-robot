# The servo supply's resistance R, from the flight records (2026-11-22)

*A dated record for #6. PR #339 added the opt-in sag `V = V0 − R·I_bus` (`--supply-sag-ohm`,
`sim/supply_sag.py`) and found that the w19k get-up fails at R = 0.44 Ω (the bench leads) and
holds at 0.15 Ω (a guess). This record estimates R from the recorded hardware data, without
touching the robot, budgets it for the power lite path the robot runs on next, and re-grades
w19k there. Tool: [`experiments/supply-r/`](../../experiments/supply-r/) (`supply_r.py` fits,
`regrade.py` grades; the fit's numbers are in `fit.json` there).*

## Result

| supply path | R (one R for the bus, as the sim uses it) | how |
|---|---|---|
| lab supply on its leads → General Driver H1 (every bench session) | **0.448 Ω**, 95 % CI 0.442–0.454 | measured, 366 548 ticks in 524 blocks, 11 days |
| v5 pack (Tattu 850 mAh 3S 75C, XT30) → General Driver H1 | **0.178 Ω**, 95 % CI 0.172–0.189; the seven events alone 0.167–0.217 | measured, 6 293 ticks in 7 events, 3 sessions |
| OVONIC 4200 mAh 3S pack → power lite (BMS, switch, F1, K1) → J4–J7 | **0.06 / 0.11 / 0.23 Ω** (low / nominal / high) | measured branch part + a datasheet budget for the rest |

**w19k at the power lite's R:** at the nominal 0.11 Ω it holds as it does on a fixed supply
(23/48 against 24/48). The prone get-ups pass 8/8 with no overload trip. supine_ph loses one
seed and shows a 1/8 shoulder trip, the same as at 0.06 and 0.15 Ω. At the high 0.23 Ω it still
passes 23/48 with the overload cutoff measured but not enforced, but the knees would trip the
real cutoff in 3–4 of 8 prone seeds and the shoulders and knees in 3 of 8 supine seeds. The trips
start near 0.20 Ω.

**Recommendation for the planner:** a get-up sag A/B is worth one night. Train with `--supply-sag-ohm 0.06:0.23` (R drawn
per episode over the budget) and grade the A and the B at `supply_ohm=0.11` and `0.23` with
`eval_precision.py --overload-column`. Judge the A/B on the trips at 0.23 Ω. Walks need nothing:
#339 already showed them unaffected at 0.44 Ω. The first power lite sessions with servos replace
this budget with a measurement, by the same fit (below).

## Method

**Data.** Every flight record in `hw_sessions/` on mira (`**/*.frec`, read only, 2026-10-30 to
11-17; the records of 11-20 and 11-21 carry no servo feedback). In each tick the firmware reads
registers 56–70 of every fitted servo in one sync read (`ctrl_task` `syncReadFeedback`): speed
(58), load (60), voltage (62, 0.1 V steps) and present current (69, 6.5 mA steps). Only ticks
where all twelve leg servos answered are used, so the currents summed in a tick were read
together. Ticks that two files share (the m271 and 11-11 bench snapshots) count once. 5.35 M
bench ticks and 0.32 M pack ticks remain.

**Not used:**
- The stepper's `servos` rows (2026-11-11..16). They are read one servo at a time, every 4 steps,
  at ≤ 14 % duty, and the bus only moved 11.9–12.0 V.
- The arm rig of PR #323. It ran on the old controller and has no flight records.
- The power lite bench records (11-19, 11-20, 11-21). None has a load current to go with its
  readings (the largest is K1's coil, 0.17 A).

**Which supply.** The session notes give the configuration, checked against the idle voltage
(`supply_r.py segments`). The lab supply reads 12.02 V idle in every session. A pack falls
steadily while the servos are idle.
- The pack: 11-06 19:25–22:31Z (the `_battery` sessions), and 11-09 23:13Z to 11-11 01:04Z (m271
  NOTES: "battery in … 01:04Z … now on BENCH POWER").
- **11-11 16:37–17:08Z also counts as the pack**, though the bench NOTES say "bench supply". From
  16:39Z ("floor (board), USB unplugged") the idle voltage reads 12.52 V and falls to 11.88 V by
  17:06Z, which is a pack discharging. Its one event fits 0.217 Ω, the highest of the seven.
- Everything else is the lab supply into H1.

Both measured paths run the servo current through the General Driver: H1 (JST XH), the AO4407,
SW1, then H5 (right leg) and H6 (left leg).

**Fit.** V is the mean of the twelve servos' register 62 in a tick. The mean dithers the 0.1 V
steps, because the servos read with different offsets. The model is `V = V0 − R·I_bus`.
- V0 is not assumed. It gets its own intercept in every 20 s block (and at every boot or 2 s
  gap). That absorbs the pack's discharge and the supply's set point, so R comes only from V
  moving with I inside a block.
- Only blocks whose current spans ≥ 0.5 A are used.
- The 95 % CIs are a block bootstrap: 1000 resamples of whole blocks.
- With seven pack blocks, the bootstrap understates the pack's spread, so the seven events' own
  range is given too.

**Which current.** I_bus was estimated three ways, and R fitted against each:

| I_bus from | bench R | pack R |
|---|---|---|
| **register 69, summed** (the servos' own current reading) | **0.448** [0.442, 0.454] | **0.178** [0.172, 0.189] |
| the sim's bridge model (`supply_sag.bus_current`) on the logged load | 0.189 [0.164, 0.223] | 0.085 [0.080, 0.092] |
| current_budget's motor (upper) model on the logged load | 0.054 [0.051, 0.058] | 0.030 [0.025, 0.032] |

**R is the register-69 fit.** The load-based rows are not the answer, for three reasons.
- **How load maps to current.** Register 60 is PWM duty d. The sim's envelope gives the torque
  at duty d and speed ω as τ = stall(V)·(d − ω/ω0(V)) (`sts_servo_model`: duty = τ/stall + ω/ω0).
  Stall and ω0 are each servo's physical type's at 12 V (STS3215 2.94 N·m and 4.04 rad/s,
  STS3250 per `walker_env.SERVO_TYPES`), scaled by the servo's own V/12. That τ and ω then go
  through the sim's current models.
- **Load's sign is the opposite of speed's.** In all 916 records at ≥ 90 % duty while moving
  faster than 1 rad/s, the two registers have opposite signs. So duty is −load in the speed's convention.
- **The models are noisy per tick.** While a joint moves, they price current that register 69
  does not see: the servos run faster than the sim's ω0, so the back-EMF term is off. The noise
  in the regressor biases R low.
- **But the bridge model is on the right scale.** On the ticks where register 69 sums to > 0.5,
  > 1 and > 2 A, it sums to 1.30, 1.20 and 0.79 times the bridge model on the bench (0.82–0.87
  on the pack). On servos holding still at 20–40 % duty they agree (0.227 against 0.223 A
  median); at 40–70 % register 69 reads 1.17 A against the bridge's 0.70 and the motor model's
  1.66. So the register-69 R can be paired with the sim's bridge current as it is.

**Robustness** (register 69):

| change | bench R | pack R |
|---|---|---|
| block 10 s / 60 s | 0.444 / 0.456 | 0.176 / 0.175 |
| span ≥ 0.2 A (1286 / 14 blocks) | 0.446 | 0.180 |
| V one tick after / before I | 0.381 / 0.390 | 0.164 / 0.167 |

The zero lag fits best: the two registers are read in step. On 11-06 to 11-11 the pack's
events read higher R as it discharged: 0.167 at 12.5 V idle, 0.171–0.178 at 12.0–12.1 V,
0.184–0.189 at 11.3–11.7 V. The 11-11 16:41Z event reads 0.217 Ω at 12.4 V. Whether that was the
same pack is not recorded.

**Shared and branch.** The right leg's servos (port A, H5) and the left leg's (port B, H6) are
fitted together, each side with its own intercept per block:

V_A = V0A − R_s·(I_A + I_B) − R_b·I_A,   and the same for B.

| | R_s (supply + leads + the General Driver's inlet, FET, switch) | R_b (one port + its leg chain's leads) |
|---|---|---|
| lab supply | 0.389 [0.384, 0.395] Ω | 0.116 [0.114, 0.119] Ω |
| 850 mAh pack | 0.105 [0.101, 0.114] Ω | 0.145 [0.136, 0.155] Ω |

- The branch part agrees between the two supplies, as it should: the same leg leads and chains.
- R_s is what differs. The bench leads cost about 0.28 Ω more than the 850 mAh pack and its lead.
- R_b is a chain average: a servo at the far end sees more drop than the first one.
- For the sim's one R, a servo's drop is R_s·I_bus + R_b·I_branch. With the two legs sharing the
  bus about evenly, that is R_s + ½·R_b, which is the total R fitted above (0.389 + 0.058, 0.105
  + 0.073).

## The power lite path

The robot runs next on power lite (`pcb/power-lite/README.md`, #207):
- the OVONIC 4200 mAh 3S 130C pack's 12 AWG lead into J1 (XT60);
- the BMS in the negative lead;
- the 20 A main switch on W6/W7;
- F1 (15 A mini blade in Keystone 3568 clips) and K1's contact to SERVO+;
- J4–J7 (Molex 5267) to the same stock servo leads and chains.

The General Driver then carries only the logic, and its H1, AO4407 and SW1 leave the servo path.

| part | low | nominal | high | basis |
|---|---|---|---|---|
| pack, internal | 8 | 15 | 40 | no maker figure. Typical DC resistance of a fresh 4–5 Ah high-C 3S pack; ×2–3 aged, cold or near empty (the 850 mAh pack above rose 13 % from full to 11.3 V) |
| pack lead, 12 AWG, ~2 × 10 cm | 1 | 1 | 2 | 5.2 mΩ/m |
| XT60, mated | 0.5 | 0.5 | 1 | Amass sheet: 0.50 mΩ |
| BMS | 0 | 6 | 12 | 0 with the on-board cut-off the BMS options record recommends (no BMS). Else the fitted CF-3SJH25A-C: two 4-FET banks in series at 5 mΩ typical / 7 max each, plus the shunts (R004 per DS-NA405, R005 on the board received) |
| main switch + its leads | 5 | 10 | 30 | no contact figure for the panel toggle; estimate |
| F1 + its two clips | 5 | 7 | 12 | the fuse's cold resistance is not in the mirrored sheets; typical of a 15 A mini blade, plus a few mΩ of clips (estimate) |
| K1 contact | 1 | 1 | 30 | TE V23134: 10 mV typical, 300 mV max at 10 A |
| board copper, both rails | 1 | 3 | 6 | estimate |
| **R_s, shared** | **21** | **44** | **133** | sum (mΩ) |
| R_b, a leg branch | 110 | 130 | 155 | **measured** above: 0.116 / 0.145 Ω, CIs 0.114–0.155. J4/J6 replace H5/H6 with the same 5264-family contacts (20 mΩ max each) |
| share of the bus on the loaded branch, f | 0.35 | 0.5 | 0.6 | `docs/wiring.md` budget: a leg branch is 0.57 of the bus at the prone peak, an arm branch 0.34 at the supine peak |
| **R = R_s + f·R_b** | **0.06 Ω** | **0.11 Ω** | **0.23 Ω** | |

**The arm branches are not measured.** Their chains are short (2–3 servos), but the shoulder
lead is about 380 mm. The leg branch's R_b stands in for them.

**Cross-check.** Even if the power lite's shared part were no better than the 850 mAh pack
through the General Driver (0.105 Ω), R would be 0.105 + 0.5 × 0.13 = 0.17 Ω, inside the
budget. The high end needs an aged or cold pack and a relay or switch near its worst contact.

## w19k at those R

`robot_getup_w19k` (`/tmp/i6n/runs/robot_getup_w19k`, the copy #339 graded) was copied with
`supply_sag_ohm` set and run on the CPU referee as #339 did:
- `regrade.py`, which runs `eval_precision.py --overload-column`;
- 8 seeds per scenario;
- V0 = the run's own 11.1 V;
- no retraining;
- `main` 27d05a8.

The re-run at 0.44 Ω reproduces #339 exactly (3/8, 4/8, 5/8). The 0.00, 0.15 and 0.44 Ω rows are
#339's.

| R (Ω) | supine_ph | prone | prone_fall | of 48 | would trip (overload measured, not enforced) | supply min, passing scenarios |
|---|---|---|---|---|---|---|
| 0 (fixed 11.1 V) | 8/8 | 8/8 | 8/8 | 24 | none; longest > 80 %: L shoulder 1.76 s | 11.1 V |
| 0.06 (low) | 7/8 | 8/8 | 8/8 | — | supine_ph: shoulders 1/8 each | 10.1 V |
| 0.10 | 7/8 | 8/8 | 8/8 | — | as 0.06 | 9.4 V |
| **0.11 (nominal)** | **7/8** | **8/8** | **8/8** | **23** | supine_ph: shoulders 1/8 each; prone: none (knees 1.72 s > 80 %) | 9.3 V |
| 0.15 | 7/8 | 8/8 | 8/8 | 23 | supine_ph: L shoulder 1/8 | 8.7 V |
| 0.20 | 7/8 | 8/8 | 8/8 | — | prone: knees 1/8; supine_ph: knees 2/8, shoulders 1–2/8 | 8.0 V |
| **0.23 (high)** | **7/8** | **8/8** | **8/8** | **23** | **prone: knees 3–4/8; prone_fall: knees 2–3/8, shoulders 1/8; supine_ph: knees and L shoulder 3/8** | 7.6 V |
| 0.25 | 7/8 | 7/8 | 7/8 | — | knees 4/8, shoulders 1–3/8 | 7.3 V |
| 0.30 | 5/8 | 7/8 | 7/8 | — | knees 4/8, shoulders 1–3/8 | 6.6 V |
| 0.44 (bench) | 3/8 | 4/8 | 5/8 | 12 | knees 7/8, shoulders 4–6/8 | 5.55 V (the floor) |

- The other three get-up scenarios (recover_sit, recover_fallen, recover_supine) fail 0/8 at
  every R, as they do on a fixed supply. They were graded at 0.11 and 0.23 Ω for the "of 48"
  column.
- At the nominal R the bus draws a median 1.1 A, p95 4.4 A, max 16 A, and the supply dips to
  9.3 V at the pushes.
- The 7/8 in supine_ph is the same at every R from 0.06 to 0.25 Ω. That suggests one seed the sag tips
  over, not a trend in R.
- **The cutoff is not enforced in this referee.** A seed that "would trip" passes here, but on the
  robot the servo would drop to 20 % and that push would fail. So the trip column, not the pass
  column, is what changes between nominal and high.

## Caveats

- **Register 69 is the current R is measured against.** What it senses is not documented. It
  agrees with the sim's bridge model within 0.8–1.3× at the bus level. If it under-reads the true
  supply current, the true R is smaller.
- **50 Hz samples.** The fit sees the supply over 20 ms ticks, through whatever filtering the
  servos apply to their voltage and current. A faster sag (bulk capacitance, inductance) is not
  resolved. The sim's sag settles in ~3 ms.
- **Only the legs.** Everything was measured on the twelve-servo legs. The pack data are seven
  stride events of at most 3.3 A. The get-up draws up to 15 A in the sim, about 5× beyond
  anything measured, so the line is extrapolated to there.
- **The power lite's shared part is a datasheet and estimate budget.** It has not been measured.
  Its largest unknowns are the pack's resistance, the switch and K1 at their worst. The BMS's
  FET arrangement (two banks of four, in series) is assumed, not traced on the board.
- **The fix.** The first power lite session with servos gives R directly:
  `supply_r.py extract` and `fit`, with a power lite window added to `PACK_WINDOWS`. Any armed
  motion that draws > 0.5 A in the flight record will do; nothing extra needs instrumenting.

## Reproduce

```bash
.venv/bin/python experiments/supply-r/supply_r.py extract --workers 10   # cache -> /tmp/s6r/cache
.venv/bin/python experiments/supply-r/supply_r.py segments               # which supply, per minute
.venv/bin/python experiments/supply-r/supply_r.py fit --boot 1000 --json experiments/supply-r/fit.json
.venv/bin/python experiments/supply-r/supply_r.py ratio                  # register 69 vs the models
JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/supply-r/regrade.py \
    --src /tmp/i6n/runs/robot_getup_w19k --ohm 0.11 --tag prone --scenarios recover_prone
```
