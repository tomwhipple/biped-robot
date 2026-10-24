# Power board simulation: results

The suite in `pcb/power-board/pbsim/` (the package is `pbsim`, not `sim`,
so it cannot shadow the robot's repo-root `sim/` — P2-5) against rev A of
`pcb/power-board/`. This record sits next to the code so a reviewer sees
what each test runs *and* what it proves. PR #132, base
`design/power-circuit` (P2-6). Round 1 and round 2 (2026-10-12) answered
Percy's review
(https://github.com/tomwhipple/biped-robot/pull/132#issuecomment-6136458208);
round 3 (2026-10-14) re-checked every round-2 claim against the code and
fixed the ones that did not hold.

**Summary: 67 passed** (macOS, Homebrew ngspice 47, 0.9 s).

```
$ python -m pytest pcb/power-board/pbsim -q
...................................................................      [100%]
67 passed in 0.88s
```

## Where the suite runs as a gate (P2-4)

`make pbsim` runs the suite, and `make pytest` runs it after `tests/`. The
pre-push hook runs `make pytest` as gate 2/9 on every push that is not
docs-only (AGENTS.md, "The gates"). The hook runs each gate as a make target
in the pushed tree, so the gate holds for every clone, including one whose
`core.hooksPath` points at another checkout's hook.

Round 2 added a `[3b/9]` block to this line's hook instead. That was inert.
Tom's laptop points `core.hooksPath` at the main checkout, whose hook runs
make targets, and this line forked before main had a Makefile, so every gate
failed with "No rule to make target". Round 3 merged main into
`design/power-circuit` and from there into this branch, and replaced the
block with the make target.

## Tooling

The suite drives libngspice directly through `ctypes` (`ngsim.py`, the
four entry points it needs); PySpice 1.5's constructor no longer matches
its documentation. Everything else is pure-Python behavioural models of
the BQ76922, the TPS48111 and the pack, wired by `sequencer_sim.py`; the
suite asserts sequences, thresholds and what the ICs and firmware decide,
not waveform shapes. The C4–C6 ngspice runs in the verification record
stand on their own and are not redone here. Dependencies: `pytest`,
`numpy` (in `requirements.txt`).

libngspice (`ngsim.find_lib()`): `$PBSIM_NGSPICE` if set; the vendored
ngspice 42 on linux-x86_64 (an ELF, Ubuntu noble `libngspice0`
42+ds-3build1, byte-identical to the archive's — `vendor/README.md`);
otherwise a native one (macOS: `brew install libngspice`, ngspice 47, the
version C4–C6 used). With none, the spice tests skip and say why. Both
builds give the same numbers (I_d = 10.2 mA at Vgs 1.5 V, 332 A at 3.0 V,
Vds 10 V). One engine per process: a second `ngSpice_Init` after `quit`
is undefined in ngspice's shared mode and SIGTRAPped ngspice 47 once two
spice tests ran in one process.

The TI CSD17556Q5B model is vendored as TI shipped it and as a hand
rewrite ngspice 42 parses (`vendor/README.md`). `rewrite_ti_model.py`
evaluates TI's `.PARAM` expressions and compares the files token by
token; the only differences are LEVEL 7→8 and VERSION 3.2→3.3. Round 3's
audit found the rewrite carried PS = PD = 19.53 on M1 and M2 where TI's
`{2.1*pwidthP}` is 19.635 (junction sidewall capacitance, about 0.5 %);
corrected and re-pinned.

## How green is defined

Every test asserts one of:

- a schematic-extraction match (the components C3 verified, and every
  part a model computes with, parse to the values the record and the
  model use),
- a stated design number against the live simulation ("the rail falls
  within one tick of E-stop", "KILL holds for at least 1 s", "rail rises
  through 90 % in 30–80 ms", "CB latches at 19.8 A in ~10.5 ms"),
- a computed margin against a datasheet or fuse curve ("4.3 A per-branch
  peak is 28 % of F1's rating and 22 % of the CB trip").

## Results per test

### Schematic extraction (`test_schematic_intact.py`)

| test | passes iff | value |
|---|---|---|
| schematic_parses_to_expected_parts | 102 components parse; the key J/U/F/R refs exist; J4–J8 are the right roles; labels include KILL/SERVO_EN/ESTOP_N/ESTOP_RET/RAIL_V/PWR_INT_N/IMON/FLT_N/SDA/SCL; power symbols include BAT+, BAT-, PACK+, SERVO+, GND, +3V3_HOST | 102 components, 14 signal labels, 7 power nets |
| component_values_match_c3 | R24/R25/R26/R30/R31/R34/R35/R36/R38/R39/R12/R11/R32/R33/R18/R21/R22/R20/R23/R17, C16/C17/C19/C21/C22 and D3/D5/D6 16 V parse to C3's table | all match within 1e-6 |
| j11_is_a_two_contact_latching_button | J11 is a 4-pin header | yes |
| j10_is_a_two_pin_momentary_button | J10 is a 2-pin header | yes |
| j12_pin_count_for_control | J12 is an 8-pin header | yes |
| servo_branch_headers_carry_three_pins | J4–J8 carry pins 1–3 | yes |

What is pinned (P3-2): the parts C3 verified, plus every part a model
computes with (`test_rail_switch_protections`: R1, R9, R11, R12, R24–R26,
R30–R35, R38, R39, C19). The other R/C refs (R3–R6 balance filters, R37,
C1–C15 and so on) parse but are asserted nowhere: neither the record nor
any model states a value for them, so a rev-B change there does not fail
this suite.

### ngspice and the TI model (`test_ngsmoke.py`)

| test | passes iff | value |
|---|---|---|
| vendored_hashes_are_pinned | SHA-256 of the three vendored files equal the pins in `fetmodel.py` | all 3 match |
| rewrite_differs_from_ti_only_in_level_and_version | TI's expressions evaluated, the rewrite and TI's file differ only in LEVEL and VERSION | exactly those two |
| ngspice_loads_the_ti_fet_model | the subckt runs a Vgs sweep at Vds 10 V: I_d(1.5 V) < 50 mA, I_d(3.0 V) > 0.5 A, monotone | 10.2 mA, 332 A |
| ti_fet_model_tracks_ti_figure_3 | at Vds 5 V the model reaches 20/100/200 A within 0.1 V of Vgs of TI's 25 °C transfer curve (datasheet Figure 3: 2.37/2.71/2.94 V) | 2.35/2.65/2.87 V |

332 A at Vgs 3 V, Vds 10 V is the device, not an extrapolation of the
rewrite: TI's own Figure 3 passes 200 A at 2.94 V and is still climbing
(round 2, after Percy's review, called it "non-physical, ~330× a sane
plateau"; the figure says otherwise). The part only survives that as a
pulse (I_DM 400 A); the sweep is a DC operating point.

### Servo branches (`test_servo_branches.py`)

| test | passes iff | margin (worst reading) |
|---|---|---|
| every_servo_branch_peak_is_documented | J4–J8 carry harness.md's branch names | matched |
| branch_peak_currents_fit_under_tps48111_cb_trip | J4+J5+J6+J7 at get-up max < 19.8 A | 14.6 A, 26 % margin |
| full_bus_getup_peak_clears_cb_trip_and_f1_slow_band | §2's worst 11.8 A under F1 15 A and CB 19.8 A | 21 % / 40 % |
| two_second_getup_vs_fuse_time_current | 2 s 5–7.6 A and the 3.0/4.3 A branches inside the fuse's no-melt band | 11.8 A is 78 % of rating |
| upper_branch_getup_peaks_against_5264_contact | max(J5, J7) get-up peak ≤ the 5264 contact's 3.0 A | 0 % margin (open item 1) |
| leg_branch_rms_under_contact_rating | J4/J6 walking RMS 1.6 A < 2.5 A | 36 % |
| walking_bus_is_below_fuse_rating_with_margin | 3.05 A walking mean < 25 % of 15 A | 20.3 % |
| full_board_power_up_under_load | soft start closes under 3.05 A + 0.5 A logic; rail ≥ 11.5 V after | 12.58 V at 12.59 V pack |
| rail_soft_start_keeps_inrush_under_design_margin | 5→90 % in 30–80 ms; peak C × dV/dt ≤ 4 A and ≤ 30 % of CB | ~39 ms; 1.20 A |
| per_branch_getup_fuse_charge [3.0, 4.3] | get-up I²t < 50 % of the 135 A²s melt band | 18 / 37 A²s |

The 1.20 A inrush comes from the model's own τ (40 ms / ln 10); the
physical evidence is C4's ngspice runs (1.40 A into 2 mF, 1.99 A into
4 mF), which this suite does not redo.

### Buttons (`test_buttons.py`)

| test | passes iff | reading |
|---|---|---|
| power_button_held_less_than_280_ms_does_not_wake | a 200 ms tap leaves the BQ76922 in SHUTDOWN | 500 ms, never wakes |
| power_button_held_280_ms_wakes | first NORMAL between 270 and 400 ms | 281 ms |
| power_on_sequence_order | PACK+ before rail ON; STARTING→ON in 30–80 ms | 280 ms / 42 ms |
| power_pressed_while_on_runs_orderly_shutdown | PWR_INT_N low → KILL → SHUTDOWN after 1 s, PACK+ 0 | holds |
| kill_before_1s_does_not_shut_down | a 500 ms KILL pulse leaves NORMAL | holds |
| estop_pressed_opens_servo_rail | rail falls next tick; PACK+ up; ESTOP_N high | 1 tick |
| estop_while_rail_starting_recovers_off | press at 0.295 s, inside STARTING (0.281–0.323 s): the ramp aborts and never reaches ON while pressed | STARTING 0.281 s → OFF 0.296 s |
| estop_broken_wire_makes_rail_off | J11 unplugged takes EN to 0 | rail falls in 1 tick |
| estop_stuck_a_contact_means_only_software_can_open_rail | contact A welded: the ESP32 sees ESTOP_N and drops SERVO_EN; INP opens the rail | falls next tick |
| estop_with_welded_q3_kill_opens_main_switch | Q3 welded, contacts working (§5.3): F4 sees ESTOP_N high with RAIL_V up, raises KILL, the BQ76922 opens PACK+ | KILL 1.553 s, SHUTDOWN 2.555 s; rail at 12.55 V until then |
| estop_both_contacts_welded_defeats_the_button | both contacts welded: the press reaches neither EN nor ESTOP_N; the rail stays up | the limit of the one-failure claim |
| estop_contact_b_welded_alone_rail_still_dies_via_contact_a | contact B welded: the ESP32 is blind (ESTOP_N low), contact A still opens the rail | holds |
| power_button_does_not_wake_when_kill_latched | after KILL a 200 ms tap does nothing, a 500 ms press re-wakes | holds |
| jammed_power_button_rewakes_after_every_shutdown | POWER held: 280 ms after each shutdown the board wakes again; released, a shutdown sticks | re-wake 282 ms after each of 4 shutdowns in 8 s |

Round 2's "both contacts welded + Q3 welded" test dropped SERVO_EN at
0.8 s for no reason and raised KILL at 0.85 s, 650 ms before its E-stop
press, so the press took no part. Its two replacements test the design's
claim and its limit.

### ESP32 control (`test_esp32_control.py`)

| test | passes iff | reading |
|---|---|---|
| esp32_unpowered_board_is_inert | SERVO_EN at 0 V via R33; rail stays OFF | 1500 ticks |
| servo_en_low_holds_rail_open | board wakes, rail stays OFF | 1200 ticks |
| servo_en_high_with_estop_pressed_cannot_close_rail | E-stop latched, SERVO_EN 3.3 V: rail stays at 0 V | every pressed tick |
| servo_en_pin_voltage_within_inp_spec | 3.3 V × R33/(R32+R33) ≥ 2 V | 3.267 V |
| rail_v_divider_keeps_gpio34_in_adc_range | R38/R39 keeps GPIO 34 in ADC11's band | 2.21 V at 12.6 V |
| kill_pin_voltage_within_rst_shut_spec | 3.3 V × R11/(R11+R12) ≥ 1.2 V | 3.0 V |
| stuck_on_rail_is_detectable_by_firmware | rail up with SERVO_EN low for 50 ms → KILL → PACK+ drops | holds |
| esp32_unpowered_imu_still_boots | J9's pin map; SDA/SCL labels present (membership, not count — P3-4) | holds |
| control_signals_on_j12_match_design_pin_map | J12 8-pin; harness.md's 7 signals are labels | all present |

### Pack voltage and BQ76922 (`test_pack_voltage.py`)

| test | passes iff | reading |
|---|---|---|
| cuv_at_3_0v_per_cell | 3.1 V/cell clean; 3.0 V/cell trips CUV | holds |
| cov_at_4_2v_per_cell | a 4.25 V cell trips COV after 50 ms; COV blocks the charge FET only | CHG open, DSG closed |
| fresh_full_pack_at_zero_load_powers_on_and_discharges_normally | the board alone on the bench (no logic load): cells at 4.20 V latch COV, PACK+ stays up | COV latched; PACK+ > 9 V throughout |
| cov_hysteresis_releases_latch_once_cells_droop | COV releases 50 mV under the trip | holds |
| ocd_at_20a_with_delay | 4.74 A and 11.8 A × 2 s clean; 25 A trips | holds |
| scd_at_150a_fastest | 100 A × 5 µs clean; 160 A × 50 µs trips | holds |
| sucd_dies_below_the_stack_floor | 9.6 V clean; 8.85 V latches SUCD and shuts down | holds |
| cell_imbalance_fires_cuv_while_stack_voltage_looks_fine | cell 2 at 2.8 V, stack 11.10 V: CUV | holds |
| otp_thresholds_match_schematic_intent | COV 4.20, CUV 3.00, OCD 20, SCD 150, SUCD 9.0; CUV < battguard's 3.3 V | all match |
| wake_consumption_is_microwatt_scale | a year in SHUTDOWN at the model's 1 µA and at the datasheet's 3.1 µA maximum costs < 1 % of the pack | 0.21 % / 0.65 % |
| esp32_is_never_required_for_cell_protection | ESP32 never booted, cell 2 at 1 % SOC: CUV on the first NORMAL tick | holds |

Round 2's zero-load test ran with the board's fixed 0.5 A logic load,
which sags a full pack under COV, so COV never latched and the test also
passed on the round-1 model. `PowerBoard(logic_load_a=0.0)` is the real
bench case; under round 1's "any latch opens discharge" it fails.

### Servo-rail switch (`test_rail_switch_protections.py`)

| test | passes iff | reading |
|---|---|---|
| rail_latches_cb_on_sustained_overcurrent | 25 A latches CB 9–12 ms after the step | 11.0 ms |
| rail_latches_sc_on_hard_short | C5's 69 A peak latches SC next tick | holds |
| rail_latch_releases_on_en_or_inp_toggle [CB, SC] × [EN below ENF, INP low] | a latch held with EN and INP high stays; dropping either releases it and the rail returns to ON (datasheet §8.3.3.2) | 4 cases hold |
| rail_follows_uvlo_with_hysteresis | rail opens below 7.33 V of PACK+, stays open at 7.6 V, closes above 7.79 V | holds |
| servo_lead_short_clears_on_rail_sc_while_bq_scd_stays_out | C5 at its 69 A peak: SC latches; OCD and SCD do not; PACK+ stays up; C5's 66 A pack peak < the lowest SCD trip (150 A − 35 % = 97.5 A) | latched at 0.602 s; pack current ≤ 69.5 A; PACK+ ≥ 11.87 V |
| servo_branch_overcurrent_clears_on_rail_cb | 25 A sustained latches CB | holds |
| sub_cb_overcurrent_does_not_latch | 19.0 A stays ON | holds |
| tps48111_constants_follow_from_the_schematic | from the parsed parts with the datasheet's equations: I_OC (Eq. 6), I_SC (Eq. 11), t_OC (Eq. 10) match the model within 2/2/3 %; UVLO × (R30+R31)/R31 matches C3 within 1 % | 19.77 A, 39.81 A, 10.76 ms; 7.79 / 7.33 V |
| board_model_resistors_match_the_schematic | R30–R33, R38, R39, R11, R12, R9 and R1 equal the model's constants | all match |

The 1 ms tick cannot resolve C5's microsecond race (Q3 off in 5.9–8.3 µs);
that is C5's ngspice result. The test asserts its consequence at the
sequence level.

Round 3 corrected the TPS48111 model against its datasheet (SLUSEE5E):
EN/UVLO turns the gate on above V(UVLOR) 1.18 V and off below V(UVLOF)
1.11 V (the model had 1.0 / 0.3 V, so the rail stayed on with PACK+ down
to 2 V); a latch resets on INP low as well as EN below ENF.

## Deficiencies

Resolved:

- **P1-1** mid-STARTING press at 0.295 s; **P1-2** STARTING honours EN and
  INP; **P1-3** CB/SC/UVLO and the latch releases driven live; **P1-4**
  RESULTS numbers reproduce (inrush 1.20 A; the 332 A is TI's device).
- **P2-1** the §5.3 welded-Q3 chain driven from the E-stop press;
  **P2-2** COV blocks charge only, tested at true zero load; **P2-3**
  contact B alone; **P2-4** `make pbsim`, run by gate 2/9; **P2-5**
  `pbsim`; **P2-6** base branch.
- **P3-1** model constants derived from the schematic; **P3-2** the
  pinned set stated above; **P3-3** `pack.py` claims no IR sweep;
  **P3-4** SDA/SCL by membership; **P3-5** falsy-zero timers in
  `tps48111.py` and `bq76922.py`; **P3-6** `scenarios.py` (used by the
  button and rail tests) and `rewrite_ti_model.py` (an audit, asserted
  by a test); **P3-7** `.so` provenance; **P3-8** jammed POWER pinned.

Open:

1. **`upper_branch_getup_peaks_against_5264_contact` passes at zero
   margin**: J5/J7's 3.0 A get-up peak sits at the 5264 contact's 3.0 A.
   Harness.md defers this to a bench measurement (§10 V8, lead
   temperature).
2. **The OCD delay is 50 ms.** Design §5.5 says "OCD with a delay"
   without a number; the test asserts the intent (get-up peaks do not
   trip), not the choice among the legal delays.
3. **No BQ76922 I²C protocol coverage** (framing, NACK on a stuck bus):
   driver territory.
4. **The TS2 thermistor path is not exercised**: TS2 is the wake pin on
   rev A, the thermistor is on TS1.
5. **The TPS48111's remote temperature diode (Q5 on Q3) is not
   modelled**: C3 set no threshold and no ESP32 input sees it.
6. **Two welded E-stop contacts defeat the stop** and nothing on the
   board can tell (pinned). A jammed POWER button keeps the board from
   staying off; the design's answer is unplugging the pack (§7 step 6).

Any suspected schematic or design error goes here, not into the board
files. None found: every round-3 change is to the models and tests.

## How to reproduce

```bash
# a clone of tomwhipple/biped-robot on sim/power-circuit-tests
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
brew install libngspice      # macOS only; Linux x86-64 uses the vendored one
python -m pytest pcb/power-board/pbsim -q
```

Expected: `67 passed`. Without a loadable libngspice the two tests that
run ngspice skip with the reason.

## What this does not assert

- Waveform agreement with the C4–C6 ngspice runs (thresholds and
  sequences only).
- The physical button parts (still to be chosen per harness.md); the
  POWER button's debounce is the 280 ms wake latency.
- I²C framing, ALERT masking, coulomb counting (the BQ76922's register
  plane; only its protection state machine is modelled).
- Whether the Pi halts on `gpio-poweroff` (#87).
