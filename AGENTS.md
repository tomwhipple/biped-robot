# Working practices for this repo

*For anyone — human or agent — making changes here. The design is
[DESIGN.md](DESIGN.md); how training works is [docs/training.md](docs/training.md).*

## Workflow & status

**Project status lives in GitHub issues** — the current state of the work,
what's planned and what's next; the design documents hold only the current
design. Each issue says enough for someone else (human or AI) to pick it up
cold: the goal, what's done (commits, PRs and records linked), what's next, and
what blocks it. Update the issue when its status changes. Everything being
worked on or planned has one.

## Branching

**Meaningful changes go on a branch with a pull request.** A new capability, a
plant or reward change, a firmware change that will be flashed, a rework of how
something is done — those get a branch, so there is a place to see them whole.

Small ones do not. A typo, a doc touch-up, a report, a regenerated artifact, a
number corrected after a bench session: commit those straight to `main`.

**Most changes here will not be reviewed.** The PR is a record of a substantial
change, not a gate waiting on an approval that is not coming — open it, and do
not sit blocked on it. The real gates are the ones below, and the physical
ones: a part that prints and fits, a bench measurement.

Keep local `main` in sync with `origin/main`: fetch and rebase before working,
push after. `origin/main` moves often — expect to rebase mid-task, and re-check
any claim your change depends on when it does.

Commit at each real milestone without being asked.

## Documentation

The design documents (README, DESIGN.md, `docs/*.md`, the READMEs) describe
**the current design and the current understanding only** — no "was X, now
Y", no version narrative. When a change lands, update the doc it affects to the
new state. History lives in git. Dated records — decision records, studies,
bench-session write-ups, lessons learned — are welcome, but they go in
`docs/design-v6/` (records and evidence for the current design) or
`docs/archive/` (everything older), never inside a design document. An
experiment keeps everything specific to it (its tool, rig CAD and parts,
figures and dated session records) together in `experiments/<name>/`.

## The gates

Run these *before* committing, and report what they said. GitHub Actions CI is
disabled; the **versioned pre-push hook is the gate**. Enable it once per
clone:

```bash
git config core.hooksPath .githooks
```

| gate                                                | covers                                                                                                                                                  | in the hook                                                     |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| `python -m pytest tests/`                           | sim physics parity, link protocol, ROM, referee behaviour, the design gates (plant, IK, Gates A/B)                                                      | hard                                                            |
| `make -C firmware/host check`                       | the firmware's pure modules under ASan/UBSan, the SIL golden check, the consoles                                                                        | hard, on C/C++ pushes (otherwise only the SIL library is built) |
| `python -m pytest sim/sil`                          | the SIL suite: the firmware's control code against the CPU plant, on the deployed run; tests needing that run's gitignored artifacts skip with a reason | hard                                                            |
| cmake configure / build / ctest of `firmware/host`  | CMakeLists.txt and the Makefile agree                                                                                                                   | C/C++ pushes, when cmake is installed                           |
| `run-clang-tidy`, `firmware/host/clang-tidy-gui.sh` | `.clang-tidy`, WarningsAsErrors                                                                                                                         | C/C++ pushes, when clang-tidy is installed                      |
| `firmware/host/cppcheck.sh`                         | whole-program, fails on any finding                                                                                                                     | C/C++ pushes, when cppcheck is installed                        |
| `idf.py -C firmware build`                          | the ESP32 target links                                                                                                                                  | C/C++ pushes, when ESP-IDF is found (`SKIP_ESP32=1` to skip)    |
| `sh cad/run_checks.sh`                              | swept-ROM interference on the armless and `ARMS=1` builds; printability reports                                                                         | not in the hook: run it for any CAD change                      |

A **C/C++ push** changes a C/C++ source or header anywhere, anything under
`firmware/`, or the root `.clang-tidy`. Other pushes skip the C/C++ gates, and
the hook says so. `PREPUSH_ALL=1 git push` forces them, and so does any push the
hook cannot diff. A toolchain-guarded gate prints a SKIP note when its tool is
absent — a skip is visible, never silent. `git push --no-verify` skips
everything; know why.

## Generated files are generated

These are exported from the sim or the CAD and **must not be hand-edited** —
the headers carry a "GENERATED by ... do not edit" line, and host tests pin
them:

| file                                                                  | generator                                                                                                                 |
| --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `firmware/components/obs/include/obs/obs_spec.h`                      | `tools/gen_obs_spec.py`                                                                                                   |
| `firmware/components/policy/include/policy/weights.h`                 | `tools/gen_policy_weights.py`                                                                                             |
| `firmware/host/vectors/protocol_vectors.h`                            | `tools/gen_protocol_vectors.py` (`make -C firmware/host vectors`)                                                         |
| `firmware/host/vectors/obs_vectors.h`, `policy_vectors.h`             | `gen_obs_spec.py` / `gen_policy_weights.py`, both `--run <deployed run>` (`make -C firmware/host deploy-headers RUN=...`) |
| `cad/v6/stl/*`, `cad/v6/step/*`, `docs/design-v6/parts_v6_rollup.txt` | `python cad/v6/parts_v6.py` (`ARMS=1` for the arm set)                                                                    |
| `cad/stl/*`, `cad/step/*`                                             | `python cad/parts.py` (the prototype's parts; the training plant meshes them)                                             |
| `experiments/plan-b-bench/stl/*`, `experiments/plan-b-bench/figs/*`   | `python experiments/plan-b-bench/plan_b_rig.py` (the Plan B bench rig; `plan_b_rig_v3.py` for `stl/plan_b_v3_*`)          |
| `sim/bimo_biped_v6ar.xml`                                             | `sim/build_v6_inertia.py --write` (or `sim/gen_plant_v6.py`)                                                              |

Regenerated binary artifacts (STLs, STEPs) are rebuilt from source rather than
merged. Rendered movies (`.mov`/`.mp4`/`.gif`) are **never committed** — they
are gitignored and regenerated from the render scripts. Trained policies live in
`sim/runs/` and are gitignored; only `night_summary.html` is committed.

## Training

**Uncommitted code does not get trained.** The training box's tree is a git
clone that pulls before every job (`infra/night/night_run.sh`), so anything not
pushed to `main` is invisible to the trainer. This is not a style rule — an
rsync of a hand-curated file list once silently missed a plant XML and trained
against a stale robot for two days.

Training runs at night only (22:00, no new job after 05:00, hard stop 07:00);
days belong to the box's owner.

After every training round, **refresh the visual log**
(`python sim/build_report.py` → `sim/runs/night_summary.html`) — the results
page is how a round is reported.

Never trust MJX numbers alone: a policy is only as good as its CPU-refereed
scorecard, and a referee column is a ranking under a stated plant, not a
prediction of hardware behaviour. Sim scores are gates, not proof. See
[docs/training.md](docs/training.md).

Change one variable per run; a run that changes two cannot say which one
mattered.

## Hardware tests

- **Bench before policy.** A capability claim about the body (a lift, a step, a
  turn, a push recovery, the get-up) is first tried open-loop over the tether,
  with joint readbacks and cameras, and written down — before any training run
  targets it.
- **Calibrate the sim where the question lives.** Before trusting the plant on
  a contact-dominated question, measure the thing (sole friction, speed under
  load, play downstream of the encoders), put it in the plant, or say the plant
  is not calibrated there.
- **Dynamic tests need a stated sim margin and a "go" per attempt**: one motion
  per go, on the floor, never on a table; a spotter in position; torque released
  between runs; reset servos after any fall.
- **Frames are not measurements.** Motion claims come from encoders, the IMU,
  tracked features across frames, or a person's eyes; "cannot tell from the
  camera" is an acceptable answer.
- **One question per failure**: before a local fix, ask whether the same
  symptom would appear if the body simply could not do the thing.
- **Twin first**: before blaming a sensor, run the same command in
  `sim/sil_twin.py` from the same pose and compare.

## Bench safety

The robot is a real machine with torque-producing servos. These rules were each
learned by breaking something:

- **Any bus write is a motion command.** A goal-position write auto-enables
  torque on an STS3215. Never "verify" a calibration against a live robot —
  doing so once broke both feet.
- **Opening the serial port reboots the board**, which eats the first command
  sent after connecting. Drain and verify before any state-writing command; this
  silently saved a wrong calibration once.
- **Check torque before any bench run.** Servos have been found with torque
  dropped overnight, refusing goal writes with no fault flag and no reboot.
- **A reset-servos verdict means *moved*, not "written"** — `homeAll` returns
  `HOME_PENDING` and the firmware reads every joint back before reporting
  `DISARMED_HOME`.
- **Disarming does not lower the robot.** Friction holds the joints and a rigid
  body tips; end a probe with the reset edge while still armed.
- **Gains are registers.** A servo whose position-loop P was raised comes back at
  the factory value after a reset or a swap; read it back before arming.
- **Never release torque with a load on a lever** unless the lever hangs plumb.
  A released, loaded STS3215 back-drives: the Plan B bench dropped a bottle
  this way (2026-10-01, experiments/plan-b-bench). Any abort away from plumb
  keeps torque on.
- **Film every hardware session**, from before the port opens until a few
  seconds after it closes (`tools/cam_record.sh`). Still frames before and
  after could not show how that drop happened.

## Sourcing and documentation

- **Electronics** need **real manufacturer documentation**: connector graphics
  and specs. Rendered web snapshots and self-authored "manuals" do not count as
  a datasheet. The rule is for electronics only: mechanical and passive parts
  (bearings, mirrors, fasteners) can be generic. Measure any dimension the CAD
  depends on when the part arrives, and put the measured value in the design.
- Buy to spec filters, not to listing IDs.
- Deliverables are files in this repo with a path, not cloud artifacts.
- When the bench measures something the sim does not model, model it — but only
  after it is measured. Guessing a domain-randomization range produces a policy
  robust to a thing the robot does not do.
- Print supports are the slicer's job: model printable geometry, never support
  slabs; still teardrop small horizontal bores.

## Local tools are native tools

**No web UI for a local tool. Not a Flask page, not a served MJPEG stream, not
a browser tab — ever.** A tool that drives the sim on this laptop gets a native
window (Qt: `sim/joint_puppet.py`) or a terminal. A browser panel was built
and rejected twice; the detour existed only to dodge a macOS main-thread
problem that Qt solves directly.

- **One window.** A control surface in one place and the picture in another is
  a bug, not a layout.
- **A GUI and MuJoCo's physics must never touch `MjData` at the same instant.**
  Render on one thread, step on another, one lock across both — an unguarded
  read mid-`mj_step` lands in `mj_fwdConstraint` and the OS shows a crash
  dialog ("mjpython quit unexpectedly"); the stack is in
  `~/Library/Logs/DiagnosticReports/mjpython-*.ips`, which is the core file
  for a crash macOS does not dump.
- Renders on the Mac need `MUJOCO_GL=cgl`; scripts default to `egl` (Linux)
  and otherwise skip the render.
