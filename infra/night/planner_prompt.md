# Morning planner: the get-up campaign

You were launched headless by `night/plan_morning.sh` (cron 07:30 on mira, as
claw) after the morning collection. No human is watching. Tom has
pre-authorized the work below; do it without asking.

## Standing request (Tom, 2026-10-21)

> Plan another run, focusing on getup. Put together some analytic kinematic
> solutions to get started and then fine-tune/make robust with RL. In the
> morning, re-evaluate and plan the next run. There should be simulations
> running all night, every night, always pulling in the latest knowledge.

Your job this morning: **evaluate last night, record what it taught, and queue
tonight** — a full night of get-up training (22:00 – 05:00 starts, ~1.6 h per
600M-step Warp job: queue 5 jobs).

## Read first (the latest knowledge — do not plan from memory)

1. The campaign ledger, `docs/design-v6/getup-campaign-2026-10.md`: the plan,
   every run so far (parent, the one variable, hypothesis, result) and the
   decisions. It is the memory of this loop; the next planner reads only it.
2. The status issue named in the ledger: **Tom's comments override the plan.**
   `gh issue view <n> --repo tomwhipple/biped-robot --comments`.
3. `AGENTS.md` (working practices — binding), `docs/training.md` §8 and §11.
4. Last night: the night-log and collection-log excerpts appended below, then
   each run's `~/code/robot/sim/runs/<run>/scorecard.md` (the CPU referee — the
   number that counts) and its `progress.jsonl` (the MJX training curve).
5. What changed upstream since the last plan: `git log --since=36.hours origin/main`
   (plant, servo or env changes invalidate comparisons — say so when they do),
   `hermes kanban list` (the collective's board), and anything Tom committed.

## Evaluate

For every run collected this morning: compare it with its parent on the
get-up scenarios (`recover_supine`, `recover_prone`, `recover_prone_fall`;
`recover_sit` and `recover_fallen` are not applicable to the robot), state
whether the hypothesis held, and write that into the ledger's run table and a
dated "Night of …" entry. If a run failed to train or to referee, find out why
(night log, `sim/runs/train.log`, the collection log) and fix the cause before
queueing anything that would hit it again.

When a policy stalls, look at WHERE (time-to-stand, end height, the reel
`sim/mjx/eval_ref.py --run <run> --video`) and ask whether the reference is the
problem before blaming the RL: `sim/getup_ref_analytic.py verify` says whether
an open-loop reference stands under the robustness grid; `rise`, `search-roll`
and `record` build new ones. Improving the analytic reference where the
referee shows a phase failing is in scope every day.

## Plan tonight

- **One variable per run**, against a named parent (AGENTS.md). Seed
  replicates (`--seed`) are not a variable and are encouraged: seed variance on
  get-up is large (p1k 6/8 vs p7k 0/8 on the same recipe).
- Follow the ledger's campaign arc unless the evidence says otherwise; when it
  does, write the change of plan and why into the ledger.
- **Uncommitted code does not get trained**, and the morning referee loads
  references from `~/code/robot` on `main`: anything tonight's jobs need (a new
  npz, env code, a referee scenario) must be pushed AND on `origin/main` before
  21:30. Small changes go straight to main; meaningful ones on a branch with a
  PR that you merge once the pre-push gates pass (PRs here are records, not
  gates). Never `--no-verify`.
- Job files go in `~/code/robot-mjx/night/queue/`, named
  `<MMDDHHMMSS>-<out>` so they sort in run order. Format:
  ```
  BRANCH=main
  # parent: <run>; variable: <what changed>; hypothesis: <expected effect>
  --out <out> --robot --robot-arms ... --steps 600000000
  ```
  The runner ignores `#` lines and joins the rest. `--out` must be unique
  (check `~/code/robot/sim/runs/`, `~/code/robot-mjx/sim/runs/` and
  `queue/done/`). Copy the recipe of the parent's job file from
  `queue/done/` and change only the variable.
- Rewrite `~/code/robot-mjx/night/fallback.job` (same format) with the job
  that is most worth repeating if a future planner fails: `ensure_queue.sh`
  (21:30) tops the queue up from it with fresh seeds.
- Look at what is already queued (`ls queue/`): jobs left over from an earlier
  plan run tonight unless you move them to `queue/held/` with a
  `WHY-<job>.txt` note.

## Hard limits

- **GPU: none by day.** Training only through the queue (22:00 – 07:00). Never
  start `train_mjx.py`, `distill_student.py` or any GPU job yourself; never run
  `night_run.sh`. CPU work by day: at most 12 worker processes
  (`--workers 12`), and finish it before 21:00.
- Do not touch the walking runs, the deployed policy, firmware, CAD, the
  marathon files (`MARATHON_*`), `queue/done/`, other runs' results, or
  generated files (AGENTS.md lists them).
- Work in the planner's worktree `~/code/robot-planner` (it starts at
  `origin/main`, detached; branch from it). The venv is `~/code/robot/.venv`.

## Report

1. Refresh the results page: `cd ~/code/robot && .venv/bin/python sim/build_report.py`,
   then commit `sim/runs/night_summary.html` straight to main and push
   (AGENTS.md: the results page is how a round is reported).
2. Push the ledger update.
3. Comment on the status issue: last night (each run: score vs parent, verdict),
   what was learned, tonight's queue (each job: parent, variable, hypothesis),
   anything Tom should decide.
4. End your reply with exactly one line:
   `PLAN_STATUS: QUEUED <n> jobs — <one-line summary of last night and tonight>`
   or, if you could not queue a full night, `PLAN_STATUS: FAILED — <why>`, or,
   if a decision or hardware action of Tom's blocks the campaign,
   `PLAN_STATUS: NEEDS_TOM — <what>` (queue the best available night anyway).
   The wrapper sends that line to Tom; do not post to Mattermost/Slack yourself.
