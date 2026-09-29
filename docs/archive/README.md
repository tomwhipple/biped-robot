# Archive

Dated records from the project's history, kept for context: what was tried,
what was measured, and the lessons that shaped the current design. Each one
describes what was true **when it was written** and is not kept up to date —
the current design is [DESIGN.md](../../DESIGN.md), and the current practices are
in [AGENTS.md](../../AGENTS.md). Paths and file names inside these records may
point at files that have since been removed; `git log --all -- <path>` finds
them.

Most of these concern the **10-joint prototype** (5 DOF per leg, 10 ×
STS3215): built, calibrated and walking under its own firmware and a learned
policy in September 2026, and found unable to stand on one foot — which is
what the current design answers.

| record | what it is |
|---|---|
| [2026-09-13-lessons-learned-walking.md](2026-09-13-lessons-learned-walking.md) | **Lessons learned**: why the prototype's hardware could not support walking, why it took two months to see it, and the process changes that followed |
| [2026-09-13-design-stage-simulation-gates.md](2026-09-13-design-stage-simulation-gates.md) | the origin of Gates A–E, and the toe-in hypothesis tested against the prototype plant |
| [2026-07-to-09-prototype-design-log.md](2026-07-to-09-prototype-design-log.md) | the prototype's design document and dated experiment log, from the first CAD to the first walk: plant, actuator model, sensing, training rounds, get-up experiments, printability, knees, ROM |
| [2026-07-precision-progress.md](2026-07-precision-progress.md) | the precision-curriculum training log, including the RL get-up post-mortem |
| [2026-09-04-joystick-roadmap.md](2026-09-04-joystick-roadmap.md) | the staged plan and stage log from "falls in 5 s" to joystick driving on the prototype |
| [2026-08-02-status.md](2026-08-02-status.md) | end-of-day status the day the prototype first stood calibrated |
| [2026-07-26-bringup-day1.md](2026-07-26-bringup-day1.md) | first board power-on and servo ID assignment (on the earlier Servo Driver board) |
| [2026-07-23-hip-yaw-study.md](2026-07-23-hip-yaw-study.md) | the study and A/B that added hip yaw — decisive for turning |
| [2026-07-20-training-prior-art.md](2026-07-20-training-prior-art.md) | the training plan rebuilt on prior art (MuJoCo Playground, LocoMuJoCo, get-up literature) |
| [2026-08-31-camera-options.md](2026-08-31-camera-options.md) | camera options for navigation; led to the Pi + Camera Module 3 |
| [2026-07-11-gopro-vision-input.md](2026-07-11-gopro-vision-input.md) | GoPro MAX as a live vision input (the GoPro is not in the design) |
| [2026-07-10-cad-and-printer-recommendations.md](2026-07-10-cad-and-printer-recommendations.md) | CAD tool and printer choice |
| [2026-07-sheetcut-eval.md](2026-07-sheetcut-eval.md) | whether any printed part should be sheet-cut instead |
| [2026-09-16-study-side-mounted-legs.md](2026-09-16-study-side-mounted-legs.md) | get-up study of side-mounted ("bird") hips — the documented fallback body if the arm get-up fails on hardware |
| [2026-09-17-study-wide-hip-gait.md](2026-09-17-study-wide-hip-gait.md) | a static gait for the wide-hip bird body |

The raw logs, scripts and filmstrips behind the two bird-body studies were
removed from the tree on 2026-09-29 and remain in git history
(`sim/getup_v6_side.py`, `sim/getup_v6_bird3_verify.py`, `sim/wide_gait.py`,
`docs/design-v6/getup_search_{side,bird3}*.txt`, `gateD_{side,bird3,wide_gait}*.txt`).
