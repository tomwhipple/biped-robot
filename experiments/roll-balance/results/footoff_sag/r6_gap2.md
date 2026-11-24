# Round 6 (2026-11-23), gap 2: the foot-off sag plant against the 2026-11-16 walks

Issue #259, item 2: the sag plant (`sag_L=6.4,sag_R=2.9`) is (a) 35-87 % high on the right stance of the two
forward walks, (b) does not tip on the robot's right-stance tips (it tips left stances instead), (c) rings
far less than the robot after the first peak. This record tests one hypothesis per mechanism on the existing
replay (`footoff_sag.py`, grade.py's plant, open loop under the robot's written goals). No new hardware data.

**Split.** Fit side: the three clean runs (152118 march, 150059 fwd 40 mm, 174820 Variant M fwd 20 mm).
Held out: the three runs that tipped (150430 `--ss-s 1.2`, 151416 raise 0.42 s, 152225 raise 0.42 s +
pre-hike). The bar for a training term: better on the held-out tips without new tips or collapses on the
clean runs.

**Metrics** (footoff_sag.py's own, plus one):
- sag, f_ring / zeta: median trace of the clean steps after the first, per stance side (module doc).
- tip: the roll toward the lifted foot peaks >= 7 deg before touchdown (the track's tip line).
- collapse (new, `build/` script, not kept): the roll beyond the stance encoders (delta) peaks >= 9 deg before
  touchdown. The robot's clean steps peak at <= 8.1 deg; each of its tips peaks at 16-24.

**Study terms** added to `footoff_sag.py` (`split_study` / `apply_study`; CPU replay only, no plant default,
the default path is bit-identical: the `sag` row below reproduces summary.md, and the summary.json of a
plain `run` before and after the change is identical):
- `x_kd=<x>`: x the roll servos' kd.
- `x_damp=<N m s/rad>`: the roll joints' viscous damping (plant 0.1).
- `x_ank_L=`, `x_ank_R=<0..1>`: that side's ankle share of the sag preload (plant 0.3).
- `x_k_L=`, `x_k_R=<N m/rad>`: that side's sag springs (plant 60).
- `x_fl_L=`, `x_fl_R=<N m>`: that side's roll dry friction (plant 0.169). gate_common's `fric_hroll` /
  `fric_aroll` set both sides.

```bash
cd experiments/roll-balance
S="grade+sag_L=6.4,sag_R=2.9"
../../.venv/bin/python footoff_sag.py run --out ../../build/r6/fs/g1 --workers 6 --plants \
  "sag=$S;fr08=$S,fric_hroll=0.08,fric_aroll=0.08;fr0=$S,fric_hroll=0,fric_aroll=0;kd0=$S,x_kd=0;dmp02=$S,x_damp=0.02;\
sR17=grade+sag_L=6.4,sag_R=1.7;ankR10=$S,x_ank_R=1.0;ankL015=$S,x_ank_L=0.15;k40=$S,sag_k=40;k90=$S,sag_k=90;..."
# g2: friction x sag_L, per-side friction; g3: + sole_in_R; g4: friction x sag_L x sole_in_R (rows below)
```
Each grid of ~15 plants takes about 40 s on 6 processes.

## Sag per run and stance (deg, L / R)

| plant | march | fwd40 | varM | r042 | r042ph | clean-run tips |
|---|---|---|---|---|---|---|
| **robot** | **5.6 / 3.1** | **5.4 / 1.7** | **5.4 / 1.5** | **5.0 / 2.5** | **5.8 / 2.2** | none |
| sag (6.4 / 2.9) | 5.7 / 2.4 | 5.7 / 2.3 | 6.1 / 2.7 | - / 2.4 | 5.6 / 2.7 | none |
| sag_R 1.7 | 5.8 / 1.8 | 5.7 / 1.8 | 5.9 / 2.2 | - / 1.9 | 5.7 / 2.0 | none |
| sag_R 2.2 | 5.8 / 2.0 | 5.7 / 2.0 | 5.9 / 2.4 | - / 2.1 | 6.8 / 2.2 | none |
| R ankle share 1.0 | 5.5 / 1.8 | 5.2 / 2.2 | 5.4 / 2.5 | - / 2.3 | 5.3 / 2.0 | none |
| sag_k 40 | 6.8 / 2.8 | 6.3 / 2.5 | 6.7 / 2.9 | - / 2.8 | 6.3 / 3.0 | none |
| sag_k 90 | 5.1 / 2.2 | 5.5 / 2.3 | 5.8 / 2.6 | - / 2.3 | - / 2.4 | step 9 |
| fric 0.08, sag_L 5.2, sag_R 2.4 | 4.8 / 2.1 | 5.0 / 1.9 | 5.5 / 2.1 | - / 2.2 | 4.8 / 2.5 | none |

**(a) Not closed.** The robot's right-stance sag depends on the gait: 3.1 (pre-hike march), 2.5 / 2.2 (quick
marches), 1.7 / 1.5 (the forward walks). The plant's barely moves: 2.3-2.7 on every run, whatever the term
(sag_R, R ankle share, sag_k, friction). A per-side constant can match the forward walks (sag_R 1.7: 1.8 /
2.2) or the march (2.9), not both. The training DR range `R=1.45:4.35` already spans what the robot does.
What differs between the gaits is not identified. The two forward walks also differ from the march in
timing (td-pause) and pre-hike, so they do not separate the gait from those.

## Ringing (clean runs, median sag trace, f Hz / zeta; L stance; R stance)

| plant | march | fwd40 | varM | clean-run tips (imu) / collapses |
|---|---|---|---|---|
| **robot** | **1.7 / 0.08; 1.5 / 0.07** | **1.7 / 0.17; 1.6 / 0.00** | **1.6 / 0.05; 1.4 / 0.06** | none / none |
| sag (friction 0.17) | no ring (0.94); 1.1 / 0.23 | 1.7 / 0.58; no ring | 1.9 / 0.58; no ring | none / none |
| kd x0 | 1.6 / 0.79; 1.1 / 0.28 | 1.7 / 0.59; no ring | 1.6 / 0.66; no ring | none |
| viscous 0.02 (was 0.1) | 1.8 / 0.58; 1.2 / 0.29 | 1.8 / 0.52; no ring | 1.6 / 0.73; no ring | none |
| friction 0 | 1.6 / 0.33; 1.7 / 0.11 | 0.9 / 0.56; 1.6 / 0.00 | no fit; 2.2 / 0.00 | L steps 3, 5, 7, 9 tip |
| friction 0.04, sag_L 4.5 | 1.6 / 0.32; 1.6 / 0.17 | 1.6 / 0.38; 1.9 / 0.32 | no ring; 2.2 / 0.00 | none (varM L sag 7.0) |
| friction 0.08, sag_L 5.2 | 1.7 / 0.45; 1.3 / 0.34 | 1.7 / 0.44; no ring | 1.4 / 0.54; 1.3 / 0.00 | none / varM 3L |
| friction 0.10, sag_L 5.4 | 2.2 / 0.55; 1.4 / 0.33 | 1.8 / 0.46; no ring | 1.5 / 0.59; no ring | none / none |
| R friction 0 (L 0.17) | no ring; 1.7 / 0.10 | 1.4 / 0.60; 1.6 / 0.00 | 1.8 / 0.46; 2.0 / 0.15 | none / not scored |

"no ring": the fit sits at its 0.8 Hz lower bound with zeta 0.8-0.95, so there is nothing to fit.

**(c) Narrowed, not closed.** The plant's damping after the first peak is the roll joints' dry friction
(0.169 N m, link side). Taking out the PD's kd or the viscous damping leaves zeta unchanged. Lowering the
friction gives the robot's 1.4-1.7 Hz and lowers zeta, but it also deepens the left-stance sag, which then
has to be refitted (sag_L 4.5-5.4 instead of 6.4). Below about 0.08 N m the clean Variant M run's left
stances collapse or tip. No setting gets zeta to the robot's 0.05-0.17 on the left stance without that:
the best clean one is 0.32-0.55. On the right stance, friction 0 on that side alone rings like the robot
(zeta 0.0-0.15) with no clean-run tip, but the right-side sag stays 2.5-3.1 on every run.

## Tips and collapses on the held-out runs

Robot: tips ss12 2R; r042 3L, 6R, 7L; r042ph 2R, 5L, 6R. Collapses (delta >= 9): the same, plus r042 5L
(9.1). That is 8 collapses, 4 of them on R stances.

| plant | ss12 | r042 | r042ph | tips hit / false / missed | collapses hit / missed / false (held out) | clean false collapses |
|---|---|---|---|---|---|---|
| sag | 1L | 1L, 3L, 5L, 7L | 5L, 7L | 3 / 4 / 4 | 4 / 4 / 4 | 0 |
| sag + sole_in_R=2 | 1L (col +2R) | 1L, 3L, 5L, 7L (col +6R) | 5L, 7L | 3 / 4 / 4 | 6 / 2 / 4 | 1 |
| sag + sole_in_R=1 | 1L (col +2R) | 1L, 3L, 5L, 7L (col +6R) | 5L, 7L (col +6R) | 3 / 4 / 4 | 7 / 1 / 4 | 1 |
| sag + sole_in_R=0 | 1L | 1L, 3L, 5L, 7L | 5L, 7L | 3 / 4 / 4 | every R stance of the quick runs sags ~8 deg, including those the robot held | |
| L ankle share 0.15 | - | 3L, 5L, 7L | - | 2 / 1 / 5 | 4 / 4 / 4 | 1 |
| friction 0.08, sag_L 5.2 | - | 3L, 5L, 7L | - | 2 / 1 / 5 | 4 / 4 / 0 | 1 |
| ... + sole_in_R=2 | - | 3L, 5L, 7L (col +6R) | - | 2 / 1 / 5 | 5 / 3 / 0 | 1 |
| ... + sole_in_R=2, R friction 0.04 | - (col 2R) | 3L, 5L, 7L (col +6R) | - | 2 / 1 / 5 | 6 / 2 / 0 | 2 (and a clean tip, varM 7L) |
| friction 0.10, sag_L 5.4 | - | 3L, 5L, 7L | - | 2 / 1 / 5 | 4 / 4 / 2 | 0 |

Per step (`build/` script; robot vs sag, ss12 / r042 / r042ph):
- **The robot's right-stance tips are a late collapse, not a large first sag.** On the steps that tipped,
  the first peak at 0.3-0.4 s is like that of the R steps that held (delta 3-5 deg). Then the body keeps
  rolling. The roll peaks at 0.82-1.10 s with delta 16-19 deg, while the trim has the stance encoders 7-8 deg
  the other way. That is a tip over the stance sole's edge, not a spring's release. The sag plant's R
  springs (2.9 deg) never get there: delta 3-4 deg on the same steps.
- **The sim's false left tips at step 1** (ss12 and r042: delta 13-15 deg, robot 7) come from too much
  left-stance yield on a quick first lift. A lower left ankle share (0.15) or a lower left sag with less
  friction (5.2 with 0.08 N m) removes them and keeps the hits at r042 3L and 7L.
- **Only an inner edge on the right sole moves the right-stance tips.** With `sole_in_R` at 1-2 mm (the
  right sole's inner support line at the ankle-roll axis), the sim collapses on ss12 2R and r042 6R (and
  r042ph 6R at 1 mm). It holds the R stances the robot held in the same runs (r042 2R and 4R) and all those
  of the clean runs. But the collapse is half as deep (delta 9-10 vs 16-19; roll 4-5 deg vs 9-12, under the
  tip line), and r042ph 2R is missed. The result depends steeply on the edge: at 3 mm nothing happens, and
  at 0 mm every quick-run R stance sags about 8 deg, including those the robot held.

**(b) Narrowed, not closed.** The robot's right-stance tips belong to the static single-support window
on the right foot (the inset-sole finding of 11-12), not to the sag release. A sag-release term cannot
make them: it is per side, and on the right it is small. An R inner edge at about the ankle axis, on top of
the sag, picks out the right steps but does not tip them. That is consistent with the README's warning that
the inset sole and the sag "count the same inner edge twice". On the left, the same sole_in with the sag
tips the clean runs.

## Training flag

**None is supported.** No variant improves the held-out tips without a cost on the clean runs:
- The best tip count is `fric 0.08-0.10 + sag_L 5.2-5.4`: hits 2, false 1, missed 5, against the sag
  plant's hits 3, false 4, missed 4. That is fewer false tips but also fewer hits, and none on the right.
- Every variant that collapses a right stance adds a clean-run collapse, and the varM run is fragile below
  0.10 N m.
- The friction change is a global plant parameter (dof_frictionloss on the rolls). It would also move the
  play and inset plants, which were not re-run here.

As a grading study only: `GRADE_PLANT_TERMS=sag_L=5.4,sag_R=2.9,fric_hroll=0.10,fric_aroll=0.10` keeps
the clean runs clean, drops the false step-1 left tips and rings slightly more (zeta 0.46-0.59 vs
0.58-0.94).

## What would settle it (bench, when the robot is back)

1. **Free decay per stance side.** The robot stands on one foot (lift gate, foot held 15 mm up, both
   sides, 3 times each). Give a 2 deg step in the stance hip-roll trim (or a light tap at the pelvis), and
   record IMU and encoders at the full flight-record rate for 3 s. This gives f and zeta of the
   beyond-encoder roll per side, with the encoders still. Repeat with both feet down. If the both-feet-down
   case rings too, the ring is not in the roll joints.
2. **Roll-joint breakaway torque.** Hang the robot with the feet free, torque on, holding a pose. Push each
   hip-roll and ankle-roll link sideways through its play with a spring scale at a measured lever (e.g. the
   sole edge), 3 times each way. Record the force where the link starts to move and where it stops. The
   plant has 0.17 N m; ringing like the robot's wants about 0.05-0.10.
3. **The right-stance window and edge.** Run a right-stance raise-0.42 march (151416's line) with a side
   camera at sole height on the right foot's inner edge. Does the right sole roll onto its inner edge
   (visible gap at the outer edge) in the 0.4-1.1 s before a tip? That decides edge vs joint yield.
4. **Gait vs session for the R sag.** Run forward 20 mm and march back to back, both at T 4 with the same
   timing flags (no pre-hike, no td-pause), 6 steps each. Is the R sag still 1.5-1.7 forward and 2.5-3.1 in
   the march?
