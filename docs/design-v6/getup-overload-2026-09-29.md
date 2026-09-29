# The get-up against the servo's overload cutoff (2026-09-29)

> Dated record for issues #83 and #6: accurate as of its date; the current
> design is [DESIGN.md](../../DESIGN.md). Simulation only. Nothing here was
> measured on a servo.

The shoulder is the most heavily loaded servo in the scripted get-up. STS
servos protect themselves: a servo whose load stays above a threshold for a
set time drops to a reduced torque. This record puts that documented
behaviour into the get-up simulation. It then measures how close the as-drawn
get-up comes to the cutoff, and searches for a sequence with more margin.

## 1. What the manufacturer documents say

Sources, both Feetech:

- **[MT]** the ST3215 memory table V3.7, the spreadsheet that
  `firmware/components/scsbus/include/scsbus/registers.h` cites
  ([xls](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls));
- **[DS]** the ST-3215-C018 product specification, ed. A/0 2023-07-20,
  `docs/datasheets/st3215/ST-3215-C018-datasheet-12V-30kg.pdf`, §5 and §7-11.

| register | name | default | meaning [MT] |
|---|---|---|---|
| 19 | unloading conditions | 44 = bits 2, 3, 5 | temperature, current and **overload** protection are on |
| 34 | protection torque | 20 (%) | "output torque after entering overload protection … 20 % of the maximum torque" |
| 35 | protection time | 200 (× 10 ms = 2.0 s) | how long "the current load output exceeds the overload torque and remains" |
| 36 | overload torque | 80 (%) | "the maximum torque threshold for starting the overload protection time countdown" |
| 28 | protection current | 500 (× 6.5 mA = 3.25 A) | over-current threshold |
| 38 | over-current protection time | 200 (× 10 ms = 2.0 s) | |
| 60 | present load | — | "the voltage duty cycle of the current control output driving the motor" |

[DS] §7-11 says the same in prose:

- **Overload**: stalled above 80 % of stall for 2 s enters protection. A new
  position command clears the overload flag.
- **Over-current**: above 2 A for 2 s turns the output off. A new position
  command clears it.

The two documents disagree on the over-current threshold: 2 A in [DS], 3.25 A
in [MT]. At 11.1 V the model's stall current is 2.50 A, so only the 2 A line
can be reached.

Two things are **not** documented:

- **How the servo leaves protection**, other than on "a new position
  command". The firmware writes a goal every 20 ms tick. Whether an unchanged
  goal counts as new is a question for the bench.
- **Whether the timer resets** when the load dips below the threshold. The
  wording of reg 35, "exceeds … and remains", reads as a continuous timer.

"Load" is also open to two readings. [DS] speaks of torque ("stalled above
80 % of stall"); [MT] times "the load output", and reg 60 defines load as the
PWM duty. In a DC motor at speed ω, the duty that produces torque τ is
τ/τ_stall + ω/ω₀. So the duty reading is stricter: it also counts a
fast-moving joint as loaded.

## 2. The model

`sim/sts_servo_model.py` holds the documented constants, each with its source,
and `STSProtection`, which sits on top of walker_env's STS3215 envelope at the
50 Hz control tick:

- It measures both readings of load for every servo, as a fraction of that
  servo's own maximum. It reports the peak, the longest continuous time above
  80 %, and the cumulative time above 80 %.
- When a servo's continuous timer reaches 2.0 s, the servo trips. Its
  envelope drops to 20 % of its stall, or to zero for over-current.
- The two undocumented points take their worst case: the trip is **latched**
  for the rest of the run, and the cumulative time is reported alongside the
  continuous one.
- With `enforce=False` it only measures.

It is an option on `getup_v6_shoulder.run_traced` (`protection=`). Mode
`sim/gate_no3250.py overload` runs it on the as-drawn get-up. The log is
[getup_overload.txt](getup_overload.txt).

## 3. The as-drawn get-up with the cutoff on

The setup:

- plant `r5_asdrawn_rom120`, the lumped as-drawn robot with arms (2.10 kg,
  hip ROM −120);
- Plan B servos;
- tuck at hip −120 / knee −130, `self_collide` on;
- both seat-push sequences;
- the six robustness conditions (play 3/5°, μ 0.3/0.7/1.0, servos
  100/80/65 %), at the scripted pace and 3× slower;
- the cutoff enforced on each reading in turn.

Every run stands, 48 of 48. **The cutoff fires in none of them.**

| sequence, pace | reading | hottest servo, peak load | longest above 80 % (of 2.0 s) | total above 80 % |
|---|---|---|---|---|
| **shoulder 90 → 0 (recommended), ×1** | torque | shoulder 78 % | **0 s** (never above 80 %) | 0 s |
| | duty | knee / shoulder 105 % | 1.48 s (knee), 1.28 s (shoulder) | 2.34 s (shoulder) |
| shoulder 90 → 0, ×3 | torque | shoulder 75 % | 0 s | 0 s |
| | duty | shoulder 102 % | 0.24 s | 0.34 s |
| shoulder 60 → 0, ×1 | torque | shoulder 94 % | 1.00 s (shoulder) | 1.02 s |
| | duty | knee 105 % | **1.90 s** (knee) | 2.72 s (shoulder) |
| shoulder 60 → 0, ×3 | torque | shoulder 92 % | 1.12 s | 1.14 s |
| | duty | shoulder 106 % | 1.08 s | 1.66 s |

Every "longest" figure is set by the servos-at-65 % condition. At full
strength the recommended sequence peaks at 51 % of stall (65 % at μ 1.0, the
1.77 N·m of DESIGN.md §4).

**How much weaker could the servos be?** The recommended sequence, play 3°,
μ 0.7, with the servo strength (stall and no-load speed) swept down:

| servo scale | torque reading | duty reading |
|---|---|---|
| 0.65 | stands, never above 80 % | stands, knee 1.48 s above 80 % |
| 0.60 | stands, shoulder 0.04 s above 80 % | stands, knee 1.68 s |
| 0.55 | **fails without the cutoff too** (shoulder 0.28 s) | fails; the knees trip, but it fails with the cutoff off as well |
| 0.45 | fails; the shoulders trip at 23.6 s | fails |

The get-up fails from weakness, at 55 % strength, before the cutoff first
fires on the torque reading (45 %). On the duty reading, the first trip comes
at the same strength at which the sequence already fails without it.

Over-current, in the motor-current model of `sts_servo_model.supply_current`
(|τ|/Kt plus the no-load current, Kt from [DS]): the shoulders peak at
1.68 A on the recommended sequence and 1.91 A on the 60 → 0 one. Both are
under the 2 A line, which is never exceeded in any run.

## 4. A sequence with more margin

`sim/getup_v6_overload.py search` runs a CEM search over the seat push. Its
parameters are the brace and push arm poses, the ankle push, and the sit-up,
tuck, push, hold and rise times. The tuck stays at hip −120 / knee −130. Each
candidate runs all six conditions, with the cutoff enforced on the strictest
reading (duty).

The score is:

- 10 × the conditions standing;
- minus the worst total time above 80 % duty;
- minus half the worst continuous time;
- minus 3 × the worst peak torque load above 60 %;
- minus 0.02 × the sequence length.

It ran 4 restarts × 25 iterations × 24 candidates, restart 0 seeded at the
recommended sequence, on 12 cores of mira. The log is
[getup_overload_search.txt](getup_overload_search.txt).

All four restarts converge on **the recommended poses with a slower tuck and
push**. The best:

| | recommended | search's best |
|---|---|---|
| brace shoulder / elbow | 90 / −90 | 92 / −98 |
| push end shoulder / elbow | 0 / 0 | 3 / 8 |
| ankle push | −25 | −26 |
| tuck / push move time | 1.5 / 2.0 s | **3.7 / 4.3 s** |
| arm after the push | 60 | 5 |
| length | 25.3 s | 30.0 s |
| stands, 6 conditions | 6/6 | 6/6 |
| peak torque load, worst condition | 78 % | **66 %** |
| duty above 80 %: longest / total | 1.48 / 2.34 s | **0.76 / 0.80 s** |
| shoulder peak, nominal condition (μ 0.7) | 51 % | 50 % |
| shoulder peak, worst full-strength condition | 65 % (μ 1.0) | 56 % (μ 0.3) |

The verification is at the end of the same log. The search's best stands 6/6
at ×1 and at ×3, with the cutoff enforced on either reading, and never trips.
At ×3 its peak torque load is 57 % and its time above 80 % duty 0.2 s. At full
strength its shoulder peaks at 50–56 % of stall across μ, **at most 1.52 N·m**.
The recommended sequence's shoulder reaches 51–65 %, up to 1.77 N·m at μ 1.0.

## 5. What it says

1. **The cutoff does not bind the recommended get-up**:
   - on the torque reading, no servo ever reaches 80 % in any of the six
     conditions, at either pace;
   - on the duty reading, the longest stretch above 80 % is 1.48 s of the
     2.0 s timer, in the servos-at-65 % condition;
   - the sequence runs out of strength (55 %) before the cutoff fires.
2. **The 60 → 0 sequence has less margin on every line.** It spends 1.0 s
   above 80 % on the torque reading, and 1.90 s on the duty reading, 0.1 s from
   a trip. It is also the higher-current one
   ([current_budget_v6.txt](current_budget_v6.txt)). It stays the second
   choice.
3. **Slowing the tuck and the push** buys the most margin: the peak torque
   load drops to 66 % and the time above 80 % duty to 0.8 s, at a cost of 5 s.
   The bench should try this timing if the real servo's timer turns out to be
   cumulative, or its threshold lower.
4. **The walk never approaches the cutoff.** No servo in the Gate D walk
   passes 54 % of stall (70 % on the duty reading)
   ([current_budget_v6.txt](current_budget_v6.txt)).

Every sequence here starts with the arms already folded up to 180°. From a
real backward fall they start at the walk's idle pose. See
[getup-prone-2026-09-29.md](getup-prone-2026-09-29.md) §2 for what that changes.

## 6. What the bench still owes (#83)

- **Read registers 19, 28, 34, 35, 36 and 38 back** from every servo; the
  firmware's boot check (register 21) is the place to add them.
- **Measure the seat-push torque.** Log Present Load (reg 60) and Present
  Current (reg 69) through the push on the first scripted get-up. That also
  settles which reading of "load" the protection uses.
- **Find how a tripped servo recovers** when the goal is re-sent unchanged
  every tick.
- **The pull test** of a printed upper arm.

## 7. For the training environments (#6)

`sts_servo_model.STSProtection` needs only `env._servo`, `env._servo_tau` and
`env.data`, so walker_env can use it unchanged. Whether the training
environments need it:

- **The walk does not.** Its loads stay under 54 % of stall, so the cutoff
  cannot fire.
- **A learned get-up or recovery does.** A policy can learn to hold a joint at
  stall indefinitely, which the real servo does not allow. Any task that can
  load a joint above 80 % for seconds should run the cutoff, or at least score
  time above 80 % per servo in the referee.
- **Pack sag and thermal derating** stay open under #6. The current budget
  gives the load profile that sag would need, but nothing about the pack has
  been measured.

## Files

- `sim/sts_servo_model.py`: documented constants, the protection model and the
  supply-current model.
- `sim/gate_no3250.py overload` → [getup_overload.txt](getup_overload.txt).
- `sim/getup_v6_overload.py search` → [getup_overload_search.txt](getup_overload_search.txt).
- `sim/getup_v6_shoulder.py`: `run_traced(protection=, trace=, time_scale=)`.
