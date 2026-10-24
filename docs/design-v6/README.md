# Design records and evidence

The dated records behind the current design, and the raw logs and figures its
numbers come from. [DESIGN.md](../../DESIGN.md) is the current design; these
records say how each part of it was decided and measured, as of their dates.
The directory name matches the CAD's (`cad/v6/`); it is the current design,
not a version among others.

## Records

| record | what it decided or measured |
|---|---|
| [2026-09-13-design-record.md](2026-09-13-design-record.md) | the validation record: requirements, kinematics sweeps, Gates A–D, servo choice, body revisions, fall-recovery studies, and **§14 Plan B** (STS3215 everywhere with raised P). Code comments cite its sections |
| [experiments/plan-b-bench/](../../experiments/plan-b-bench/) | the Plan B bench (#73), in its own folder: `gain_bench.py`, the rig (CAD and figures) and its dated sessions -- session 1, the bench servo's registers and the unloaded P ladder; gain-write ack loss (#101) |
| [getup-decision-2026-09-17.md](getup-decision-2026-09-17.md) | the get-up decision (two elbowed arms) and **every dead end with its measured reason**, so none is retried |
| [getup-noappendage-2026-09-14.md](getup-noappendage-2026-09-14.md) | the no-appendage get-up search (kip, bear, crow, bridge, skid, bumper): negative, and the hip −90…−100° crouch finding; figures in `figs/` |
| [study-shoulder-arms.md](study-shoulder-arms.md) | the arm study behind that decision: reach threshold, elbow, mount position |
| [shoulder-girdle.md](shoulder-girdle.md) | shoulders into the torso in the hip plane; assembly order; §11 the get-up re-run on the robot as drawn |
| [arms.md](arms.md) | the arm parts in CAD, their ROM and open items |
| [elbow-servo-upper-arm-2026-10-07.md](elbow-servo-upper-arm-2026-10-07.md) | the elbow servo's case moves from the forearm to the upper arm; both arm links take the leg link's open-U section with end walls; before/after on every gate |
| [hip-yoke-single-print.md](hip-yoke-single-print.md) | the hip roll and pitch clevises as one print; hip-flexion relief to −120° |
| [failure_sweep_2026-10-07.md](failure_sweep_2026-10-07.md) | failure modes of the kinematic walk on the robot plant: 434k random walks; roll play, lift height, actuation lag and swing time decide falls and lift |
| [foot_outline_2026-10-07.md](foot_outline_2026-10-07.md) | the printed foot sat 12 mm outboard of the spec; at the spec the feet touch at 2.1° of mutual hip-roll adduction. Gate D margin vs adduction clearance at three sole placements (open, for Tom) |
| [study-yaw-bearing.md](study-yaw-bearing.md) | hip-yaw bearing options A / C / E and their checks; A selected 2026-10-06 (#75) |
| [2026-10-06-power-circuit.md](2026-10-06-power-circuit.md) | the Power circuit milestone (#106–#109): a custom power board (BQ76922 protection and monitor, TPS48111 servo-rail E-stop switch, servo distribution), the Pi on the General Driver's 5 V, the auto-off sequence, telemetry and the bench checks; proposed, with Tom's 2026-10-06 decisions |
| [2026-10-07-power-board-verification.md](2026-10-07-power-board-verification.md) | power board rev A checked before ordering: TI calculator, ngspice soft-start and short-circuit simulations, logic levels, footprints against makers' drawings (XT60 polarity reversed and fixed), stock, copper resistance; seven design changes |
| [2026-10-07-power-board-quote.md](2026-10-07-power-board-quote.md) | PCBWay and JLCPCB prices for power board rev A at 88 × 56.5 mm, bare and assembled, 2–10 boards; 2 oz versus 1 oz outer copper; quote-only vendors; the through-hole alternative; Digi-Key parts estimate and stock problems |
| [2026-10-24-power-board-full-simulation.md](2026-10-24-power-board-full-simulation.md) | power board rev A simulated whole before ordering: the drawn netlist in ngspice with pin models of the BQ76922 and TPS48111, all 1152 combinations of its discrete inputs (all pass), timed sequences, shorts, temperature and tolerance sweeps; ten findings (F-1–F-10) with simulated fixes |
| [2026-10-24-power-board-pin-audit.md](2026-10-24-power-board-pin-audit.md) | power board rev A, every IC pin and discrete checked against its datasheet: no wiring error; R29 under a harness fault, the missing Q5 capacitor, VC0's tie, the BQ76922 defaults that must change |
| [2026-10-24-power-board-quote.md](2026-10-24-power-board-quote.md) | production quotes for 2, 3 and 5 assembled power boards at 97.1 × 64.6 mm (PCBWay about $203 / $233 / $301 at 1 oz, JLCPCB higher, none firm); the move to 1 oz outer copper that saves about $264; parts stock and the XT60 shortage |

Records here describe what was true on their date; where one disagrees with
DESIGN.md, DESIGN.md is current. Evidence logs for options not taken (legs-only
and tail get-ups, short arms, the split hip yoke, the first arm brackets) were
removed from the tree on 2026-09-29 and remain in git history.

## Evidence

Logs (`*.txt`, `*.json`) are the verbatim output of the scripts named in their
headers; figures (`*_strip.png`) are filmstrips of the runs. `figs/` holds the
other figures the notes and records show, CAD renders among them (a part
render there is copied by hand from `build/v6/renders`). The ones DESIGN.md
leans on:

| evidence | backs |
|---|---|
| `gateA_geometry_sweep.txt`, `gateB_mixed.txt`, `gateAB_v7.txt` | kinematics and actuator envelope (Gates A, B) |
| `gateCD_sweep*.txt`, `gateD_*.txt` | the open-loop walk across the adversity matrix |
| `no3250_{walk,sweep,envelope,arms_walk,getup}.txt`, `no3250_{walk,arms_walk}_plant17.txt`, `no3250_*_elbow_upper.txt` | Plan B: every gate with STS3215s and raised P; the walk on the regenerated 17-actuator plant; the elbow-servo move against main |
| `getup_search_r6_asdrawn*.txt`, `getup_search_r5_asdrawn*.txt`, `getup_search_shoulder_*.txt`, `getup_search_girdle_*.txt`, `getup_search_hipplane_*.txt`, `getup_search_legroll*.txt` | the get-up on the as-drawn robot, the arm choice, the prone roll |
| `cad_rom_check.txt`, `girdle_check_assembly.txt`, `hip_yoke_check_single.txt`, `yaw_bearing_check*.txt`, `yaw_bearing_options_2026-09-29.txt` | the CAD interference gates (`make check-cad` writes each run's output to `build/v6/reports/cad_rom_check.txt`); the A / C / E comparison for #75 (A selected) |
| `deep_flexion_sweep.txt` | knee 130° / hip flexion limits |

The part masses and the print rollup are written by `make cad` to
`build/v6/reports/parts_v6_rollup.txt`.
