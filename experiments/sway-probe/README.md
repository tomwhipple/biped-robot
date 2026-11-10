# Sway probe: size the lateral sway on the robot (issue #260)

`--mimic-sway M` ([PR #257](https://github.com/tomwhipple/biped-robot/pull/257)) puts a torso sway of
M metres toward the stance foot into the imitation reference, hip roll plus a cancelling ankle roll, so
the soles stay level. The model sizes it at ~3.9 cm (pendulum estimate, 0.75 Hz); the robot needed ~1.3x
the model's lateral shift to unload a foot quasi-statically (`hw_sessions/2026-10-25/NOTES.md`), so the
number is probably short. This probe plays the **training reference itself**, open loop, as an amplitude
ladder through the firmware's streamed `pose`, so the amplitude (and lead) at which each foot actually
unloads can be read off the robot and set into `--mimic-sway`.

It is a system-identification probe, one scripted move at a time, with the owner's go per rung. It is not
a gait. The tool and the sim rehearsal are done; **the hardware session is the owner's**
(spotter or catch line, [#267](https://github.com/tomwhipple/biped-robot/issues/267)).

| file | what |
|---|---|
| `sway_ref.py` | the generator: imports `walker_env._mimic_ref` / `mimic_sway_coefs` (the CPU mirror of `sim/mjx/env_mjx.py`) and writes the ladder as 50 Hz joint-target keyframes |
| `sway_sim.py` | the sim rehearsal: open-loop playback on `bimo_biped_v6legs_meas_sole.xml` under the `robot_legs_prec45_ws1_only` actuator chain, over a band of its DR ranges |
| `sway_probe.py` | the robot-side sender: dry run by default, one rung per `--go` |
| `predictions*.json`, `data/` | the sim bands the sender prints (committed; regenerate with `sway_sim.py`) |
| `recipe_prec45_ws1_only.config.json` | a copy of that run's `config.json` (the run is gitignored), so the tool and tests do not need it |
| [`2026-11-10-sim-rehearsal.md`](2026-11-10-sim-rehearsal.md) | the rehearsal's evidence: variants, alternatives, caveats |

## What the sender plays, and what it cannot

The firmware has no 50 Hz keyframe playback. `pose <17 ticks> s<ms>` is a minimum-jerk move of 200 to
20000 ms from the servos' **read-back** positions to one target (`cli.cpp` `cmdPose`, `startSmooth`). The
sender chains one `pose ... s200` every 200 ms, each to the keyframe 200 ms ahead, so the joints follow the
reference as a staircase of rest-to-rest moves. Sim shows what that costs (rehearsal record section 3):

- the swing foot rises **~7 mm** (median; band 1 to 12 mm) on the lift rungs, where the reference's lift is 20 mm (direct 50 Hz playback in the same sim: 19 to 22 mm median from 3.9 cm up);
- 200 ms segments are the best of the lengths tried (300 and 400 ms fall on the 5.0 cm sway rung in 13 and 16 of 24 plants).

So the lift rungs test unloading with an attenuated lift, not the reference's 20 mm. `M` is the commanded
amplitude through this chain; the torso shift it produces is smaller, and the sim predicts it per rung.
The `h<pct>` / `a<n>` tokens of `pose` (servo speed headroom, acceleration) might give more of the reference;
the CPU plant does not model goal speed, so they are not used and the effect is unknown.

## The ladder

Clock 0.75 Hz, lead 0.785 rad (`--mimic-sway-lead`'s default), 4 cycles per rung, 1.5 s minimum-jerk ramp in
and out of the standing pose (8.3 s a rung). Rungs 1 to 6: **sway only**, both feet planted (the reference's
two roll channels, every other joint at stand). Rungs 7 to 12: the same amplitudes with the reference's
lift (2 cm IK lift of the swing sole, stepping in place). The amplitudes bracket the model's 3.9 cm and the
robot's expected 5.1 cm (1.3x).

Sim prediction per rung (`sway_probe.py --table`; chain `pose`, 24 Latin-hypercube plants over the recipe's
DR ranges: floor friction, joint friction, roll play, backlash, torso mass 0.6 to 1.4x and CoM +-15 mm, action lag
2.65 to 8 Hz, dead time, latency, servo stiffness; open loop, no balance loop):

| rung | kind | M cm | hip roll peak deg | sim: stays up | swing-foot clearance mm, L / R (median, band min-max) | CoM over stance sole mm (median) | peak tilt deg (median, max) | roll rate deg/s (median, max) | stop: tilt deg / gyro rad/s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | sway | 2.0 | 3.5 | 24/24 | 1 (0-2) / 1 (0-2) | -8 / -8 | 1.7, 2.0 | 25, 37 | 6 / 1.0 |
| 2 | sway | 3.0 | 5.2 | 24/24 | 3 (1-5) / 3 (0-4) | -1 / -1 | 3.2, 3.6 | 40, 54 | 6 / 1.4 |
| 3 | sway | 3.9 | 6.8 | 24/24 | 5 (2-7) / 5 (2-6) | 7 / 7 | 4.5, 5.0 | 53, 70 | 8 / 1.8 |
| 4 | sway | 4.5 | 7.9 | 24/24 | 6 (3-8) / 6 (3-8) | 12 / 13 | 5.1, 6.1 | 60, 76 | 9 / 2.0 |
| 5 | sway | 5.0 | 8.7 | 24/24 | 7 (4-10) / 7 (5-9) | 20 / 20 | 5.8, 6.9 | 67, 76 | 10 / 2.0 |
| 6 | sway | 5.6 | 9.8 | 22/24 | 8 (5-52) / 9 (6-23) | 28 / 28 | 6.9, 61.1 | 74, 113 | 12 / 3.0 |
| 7 | lift | 2.0 | 3.5 | 24/24 | 7 (1-8) / 7 (1-9) | 0 / 2 | 4.1, 6.1 | 38, 51 | 9 / 1.3 |
| 8 | lift | 3.0 | 5.2 | 24/24 | 7 (3-11) / 7 (3-12) | 3 / 4 | 4.2, 5.2 | 45, 57 | 8 / 1.5 |
| 9 | lift | 3.9 | 6.8 | 24/24 | 7 (3-12) / 8 (4-12) | 9 / 10 | 4.4, 5.7 | 56, 64 | 8 / 1.7 |
| 10 | lift | 4.5 | 7.9 | 24/24 | 7 (3-12) / 8 (4-10) | 14 / 14 | 4.4, 7.2 | 63, 72 | 11 / 1.9 |
| 11 | lift | 5.0 | 8.7 | 24/24 | 7 (4-12) / 8 (3-11) | 17 / 19 | 4.7, 6.4 | 70, 82 | 10 / 2.1 |
| 12 | lift | 5.6 | 9.8 | 24/24 | 7 (5-43) / 8 (3-28) | 22 / 24 | 4.9, 24.0 | 74, 141 | 12 / 3.0 |

Reading it: **clearance** is the swing foot's lowest pad above its standing height. **CoM over stance sole** is
the whole-body CoM's distance inside the stance foot's pad extent, at its best in the swing window: above 0 the
foot that swings *can* be unloaded quasi-statically. It crosses zero between 3.0 and 3.9 cm in the sim. The sim's
foot **load** is not in the table: with both feet planted and position-held, the load split is statically
indeterminate and the swing foot's 100 ms-average load reaches zero at every rung in most plants, driven by hip
roll versus ankle roll timing (in the sim's plant the STS3250 hip roll leads the STS3215 ankle roll by 3 control ticks, 60 ms, the 2026-11-01 actuator fit) and roll play,
not by the CoM. The robot has no foot force sensor either. Do not read "unloaded" from load; read it from
clearance, tilt and roll signature.

## Is open loop safe? (the honest answer)

Not across the whole ladder. In the sim band, open loop stays up on every plant up to **5.0 cm sway-only and
5.6 cm with lift**; the sway-only 5.6 cm rung loses 2 of 24 plants, 6.2 cm loses 11, 7.5 cm loses all, by tipping over the
outer edge of the stance foot (peak tilt in the band's worst plant 60 deg; the median plant is fine). The sender
**refuses `--go` on any rung where a plant of the sim band fell** (rung 6; `--accept-sim-falls` overrides). Alternatives, evaluated in
the rehearsal record: the same ladder with the bench roll PI (`--balance stance`) does not help (sway rungs the same,
lift rungs worse at every M: 22 of 24 plants up at 2.0 to 3.9 cm, then 17, 9, 2 at 4.5, 5.0, 5.6 cm) and the trained `sway`
skill sidesteps (180 to 240 mm drift in 6 s) and has no amplitude knob. **Recommendation: open loop, to the top
of the sim-safe ladder, with the stop criteria below.** Evidence and caveats (the PI is emulated, the sim has no goal
speed) are in the record.

## Session protocol

Before any rung (owner, at the robot):

1. Spotter or catch line in place (#267), robot standing on the floor at the zero pose, hard floor or carpet as for the other sessions, torque on. Operator stop ready: `tools/estop.py --button`.
2. On mira: `tools/flight_session.sh start sway` (collector, UDP 4220). On the robot, `rec <mira's LAN IP>` once if it is not already set (`docs/flight-recorder.md`). The sender refuses to start otherwise: it checks the collector locally and the robot's `rec` line.
3. Film from before the port opens until after it closes (`tools/cam_record.sh`, floor-level camera behind the robot, as in the 10-25 session).
4. Key: `~/.config/bimo/wcli.key` provisioned (`docs/wifi-cli.md`). Check the IMU mount and gyro bias were done for this mounting of the board (the sender only checks upright < 3 deg).

One rung per run, in ladder order, each after a dry run you have read:

```
.venv/bin/python experiments/sway-probe/sway_probe.py --rung 3          # dry run: plan, sim prediction, stops
.venv/bin/python experiments/sway-probe/sway_probe.py --rung 3 --go     # asks for "GO 3"; runs exactly that rung
```

What `--go` does: collector check, `bench`, torque check, `cal show` (all 12 leg joints fitted and unchanged since
the plan), `pose` to the zero stand, upright check, then 42 `pose ... s200` each followed by an `imu` read, then
`home`. Every ending is `home` (smooth to the calibrated zero, hip play take-up, read-back, torque released; `release`
if `home` does not answer), as the one-leg stands ended. Log: `hw_sessions/<date>/sway_probe/<stamp>_rung<N>.log`.

**Stop criteria** (the sender ends the rung, `home`, on the first): tilt past the rung's stop tilt, or |gyro| past its
stop rate (table, last column: 1.5x the worst the sim band reached, tilt 6 to 12 deg, rate 1 to 3 rad/s); a refused or
unanswered `pose`; Ctrl-C. Firmware beyond that: the bench fall guard (30 deg, 100 ms, goes limp). **You stop it** on
anything the sender cannot see: a foot sliding, a leg hitting the other, a servo fault, the spotter's call. Do not
continue the ladder after a stop, a reset-servos, or a fall: bring the log to the sim first.

**Advance rule.** Do rung N+1 only if rung N finished without a stop and its peak tilt and roll rate sit inside the sim
band. Outside it, stop the ladder: the plant is wrong in a way the sim must absorb (#259, #82) before more amplitude.
Run a lift rung only if the sway-only rung at the same M finished. If the foot never clears by rung 12, stop there:
the sim-safe open-loop ladder ends at 5.6 cm, and a larger M needs a safer probe, not a bigger rung.

**Record, per rung**, in a dated record next to this README (`2026-..-sway-probe-session.md`): rung and flags; the
flight-recorder file; the video file names; per side: whether the swing sole was visibly off the floor (a gap seen,
none, cannot tell) and for how many of the 4 cycles, and its height if resolvable; from the flight record: peak torso roll,
peak roll rate, the swing leg's hip roll and ankle roll position error peaks; the sender's per-segment lateness; any stop, and why.
Put the sim band beside each (`predictions.json`). Frames are not measurements: "cannot tell" is a valid entry.

**How the result sets `--mimic-sway`.** For each side, M_clear is the smallest lift-rung M at which that foot's sole is
seen clear on at least 3 of the 4 cycles; the sway-only rungs say whether it is already lightened by the sway alone and
where the CoM crosses (the roll signature). Then: (1) check the measurements against the band; where they sit outside it,
fit the sim plant to this session first (#259, #82: roll play, CoM, lag from the flight record). (2) With the fitted plant,
find the M at which the **training-chain** reference clears the foot (`sway_sim.py --seg-ms 0 --chain prec`; the probe's
chain realizes less of the reference than training's does, so M_clear is an upper bound). (3) `--mimic-sway` = the larger of
the two sides' values; if they differ by more than 0.5 cm that is an asymmetry to record, not to average. `--mimic-sway-lead`:
the lead is only varied if M alone does not clear a foot. The sim says a smaller lead is safer and a larger one clears more
(`predictions_lead0.4.json`, `predictions_lead1.2.json`; the sender takes `--lead 0.4 --predictions ...`). The
issue closes when the record holds, per side, the amplitude and lead with the plant's prediction beside it, and the flag is set from it.

## Regenerating

```
JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES= .venv/bin/python experiments/sway-probe/sway_sim.py --n 24 --chain pose --seg-ms 200 --json experiments/sway-probe/predictions.json
.venv/bin/python experiments/sway-probe/sway_probe.py --table          # the table above
.venv/bin/python -m pytest tests/test_sway_probe_ref.py tests/test_sway_probe_sim.py tests/test_sway_probe_sender.py tests/test_sway_probe_wire.py
```
