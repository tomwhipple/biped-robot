# Gradebook — a Jev-style configurable grader to replace the LLM referee

**Status:** v1 shipped 2026-10-07 (this PR) — design doc + trace capture + metric grader
**Author:** Aime · **Date:** 2026-10-07 (adjudicated Tom 2026-10-07)
**Lives in:** `sim/gradebook/` + this file at `docs/gradebook-design.md`
**Goal:** take over Claude's referee duties with software; emit numerical, per-criterion feedback an RL pipeline can use.

---

## 1. Problem

Today's referee is a conversation, not a pipeline stage. Claude's reads (qualitative verdicts, per-scenario takes, "the movement reads natural") sit *outside* the machine. The machine's own evidence is:

* `RUN/seed_log.jsonl` — terminal JSON per episode, written after the fact, no trace, no per-criterion numbers.
* `RUN/scorecard{,_lag,_sil}.json` — one row per scenario; verdict = `successes == n` plus headline metrics.
* `RUN/reel_index.json` + `.mov` — the reel Claude watches.

What grading must produce that it doesn't today:

1. **Per-criterion verdicts with numbers** — *why* something passed/failed, comparable across runs, plottable, diffable.
2. **Numbers usable in RL** — a graded output that can serve as reward, shaping signal, or offline selector.
3. **Configurable criteria** — grading is a versioned data artifact (YAML), not edits scattered through `eval_precision.py` scenario factories.
4. **A referee you can argue with** — the grade card shows the evidence behind every verdict, so a FAIL can be contested and the grader can't paper over a failure (Claude's honesty rules, enforced by schema instead of by Claude).

## 2. Anchor decision: a Jev-like RETURN interface, judge-model-agnostic

What's Jev-like is the interface **from the grading model back to the referee/RL function**: every criterion returns as a typed answer — scalar or ordered-level index, per-level `probabilities`, a `confidence`, evidence refs — assembled into a versioned grade card. That answer schema is the stable contract; RL reward code reads it and doesn't care what model produced it. Nothing in the reward path changes when the grading model swaps.

The grading model itself is pluggable — deterministic metric evaluators, an LLM judge reading the trace, or a **vision-capable judge watching reel frames/clips**. Jev specifically cannot take visual input, so it's one option among several (claude-consult and the local aux-vision model are, today, the vision-capable judges; §2.1). Where the request side borrows Jev's shape (one episode state in, typed questions out, evaluated in parallel, in isolation), it's our own criteria YAML that builds those questions — a symmetry, not a dependency.

### 2.1 Grader input modality: state data, not pixels

Is the grader watching video or reading a vector? Vector-first everywhere:

* **`metric` criteria read the Trace** — per-episode numeric arrays captured at control dt from the MuJoCo state (qpos/qvel/up_z/height/planar), plus command schedule, contacts, fall/recovery events, servo/power telemetry. Produced by capture-side hooks inside the scenario factories *during the rollout*. Physics state, never pixels — no renderer in the grading path.
* **Judge criteria can take visual input.** Vision-capable judge backends (`claude-consult`, local aux vision) get a `state: {frames: n}` projection — sampled referee-reel frames or clips, already produced by the render path. Jev cannot take images; a Jev-judged criterion falls back to the text projection of the Trace (summary, fall timeline, series stats — `key_frames: n` there are numeric snapshots, not pixels). The criteria YAML's backend table decides which graders are offered which state.
* **v1 still ships metric-only.** Subjective reads start as trace numerics (wobble_rms, up_z variance, foot_slip, power) — precedent: a render has read clean while a numeric check caught real displacement, and vectorized criteria are what make the numbers train-compatible. Visual judging arrives with the judge backends (v2), one criterion at a time, through the trust ladder.

## 3. Architecture

```
run artifacts (scorecard, seed_log, reel)  or  live rollout (CPU referee / MJX)
        │
        ▼
[1] TRACE — one Trace object per episode: state history at control dt
    (qpos/qvel/up_z/height/planar), command schedule, fall/recovery
    events, contact summary, servo/power telemetry, scenario headline.
    One extractor per source kind; capture-side hooks inside the
    scenario factories (see §7.3 risk).
        │
        ▼
[2] CRITERIA SPEC — a versioned YAML artifact (see §4). The only thing
    edited to change what's graded.
        │
        ▼
[3] GRADING ENGINE — compiles spec → typed questions per episode; routes
    each criterion to its backend; evaluates in parallel, in isolation;
    gates/normalizes/weights; aggregates.
        │
        ▼
[4] GRADE CARD — runs/<run>/grade_v{n}.json + .md (the referee read).
    Per-criterion value/level/probs/confidence + evidence refs;
    per-scenario and per-run aggregates.
        │
        ▼
[5] CONSUMERS — nightly champion selection (numerical), RL reward adapter,
    marimo training-inspector notebook, tracking dashboard.
```

The **Trace is the load-bearing interface**: every criterion can only grade what the Trace carries, so the trace schema is designed as carefully as the API. Today verdict logic computes intermediates *during* rollout and drops them; making the factories write evidence into the Trace as they compute it is the critical-path item of the build.

### 3.1 Deliverable slices

| Slice | Ships | Why |
|---|---|---|
| **v1 — gate** | Trace extractor; metric criteria only; grade card JSON+MD; parity vs existing scorecards | No LLM calls, cheap, deterministic; already improves on "successes/n" by grading per criterion |
| **v2 — judges** | Judge backends (`jev`/`claude-consult`/`local`) behind one adapter; rubric criteria for the subjective reads; confidence handling | Covers the judgment calls Claude makes today |
| **v3 — RL** | Episode-level reward adapter; dense-shaping compiler from the same spec | The RL payoff; needs the card stable first |

## 4. Criteria spec (versioned YAML)

```yaml
version: 1
suite: referee_v2                     # names + version survive in every card
judges:                               # backend table; criteria name their backend
  metric: {}
  jev: {model: jev-latest}
  claude: {consult: true}

criteria:
  # ---- metric: deterministic code over the Trace -----------------------
  success:
    type: metric
    source: scenario                  # the scenario's own success test
    weight: 4.0
    score: {normalize: [0, 1]}        # 1.0/0.0 per seed; mean across seeds
  upr:                                # upright quality
    type: metric
    source: trace
    path: up_z
    aggregate: min                    # worst up_z over the episode
    weight: 2.0
    score: {direction: higher_better, normalize: [0.4, 0.95]}
  cot:                                # cost of transport (loco)
    type: metric
    source: shared_metrics
    metric: cot
    weight: 1.0
    score: {direction: lower_better, normalize: [0.4, 2.0]}
  foot_slip:
    type: metric
    source: shared_metrics
    metric: foot_slip
    weight: 1.0
    score: {direction: lower_better, normalize: [0.0, 0.08]}
  wobble:
    type: metric
    source: shared_metrics
    metric: wobble_rms
    weight: 1.0
    score: {direction: lower_better, normalize: [0.0, 1.0]}

  # ---- rubric: judge scores ordered levels ------------------------------
  gait_naturalness:
    type: rubric
    backend: jev                      # or claude / local
    levels: ["rigid, robotic", "adequate", "smooth, biological"]
    state: {trace_summary: brief, key_frames: 8, power_headline: true}
    # key_frames = 8 numeric state snapshots at chosen times, NOT camera
    # frames; a `frames: n` key samples real reel pixels for
    # vision-capable backends only (v2+, see §2.1)
    weight: 1.0

  # ---- check: judge probability of a statement ---------------------------
  reads_stumble:
    type: check
    backend: jev
    statement: "The gait contains a stumble or loss-of-control moment"
    state: {key_frames: 8, up_z_series: true}
    weight: 1.0

defaults:
  weight: 1.0
  min_confidence_in_reward: 0.5      # confidence gating for judge criteria
```

Notes:

* `normalize` windows are calibrated data, not magic — seeded from historical scorecard distributions, re-tuned during shadow week, versioned with the spec.
* Criterion evaluation is **in isolation**: a failure in one criterion errors/labels that one, never poisons the rest.
* The v1 criteria catalog (drawn from what the referee already computes) is Appendix A.

## 5. Grade card (versioned, machine-readable)

```json
{
  "spec_version": 1, "suite": "referee_v2",
  "run": "loco_v29zero", "scenario": "line_1m", "seed": 1,
  "criteria": {
    "success": {"value": 1.0, "level": "pass", "confidence": 1.0},
    "upr":     {"value": 0.93, "normalized": 0.86, "weight": 2.0},
    "cot":     {"value": 0.71, "normalized": 0.64, "weight": 1.0},
    "gait_naturalness": {"value": 1.43, "probs": {"0": 0.11, "1": 0.60, "2": 0.29},
                          "confidence": 0.57, "backend": "jev"}
  },
  "aggregate": {"total": 7.93, "normalized_total": 0.81, "fell": 0},
  "evidence": {"trace": "runs/loco_v29zero/traces/line_1m_s1.npz",
               "reel_chapter": {"scenario": "line_1m", "t": [12.0, 26.0]},
               "headline": "8/8 | wobble 0.41 | 7.2 W"}
}
```

Two card documents per run:

* `grade.json` — machine: nightly selector, RL adapter, dashboards.
* `grade.md` — human: the referee's read. Criteria rows with verdicts + numbers, rubric commentary in its own section, FAIL takes labeled FAIL, evidence links into the reel via `reel_index.json` chapters.

`grade_manifest.json` indexes cards per (spec_version, backend set) so old cards stay retrievable. One manifest row per card: spec_version + suite + one-line summary.

**Aggregation for selector/RL:** per-scenario `total = Σ w_c · norm_c` (mean across seeds); run-level `normalized_total` = weighted mean across scenarios, plus per-family rollups (`loco`/`skills`/`getup`). Scenario-family filtering follows the existing `FAMILY_SCENARIOS` rules.

## 6. RL adapter

* **v1 (episode-level):** `R_ep = Σ_c w_c · norm_c(Trace)` — a proper per-episode scalar. Usable three ways: (a) offline selector — nightly champion = max grade total (ties broken by fall count, then CoT); (b) return labels in offline data; (c) reward-model training data.
* **v2 (dense shaping):** a compiler turns the same criteria YAML into per-step reward components for the training env (this is where `w_track_v`, `w_upright` etc. would move). Same spec, train/eval consistency for free. Explicitly later — get the episode-level number proven first.
* **Confidence gating:** judge-type criteria only enter reward after shadow-week validation; below `min_confidence_in_reward` they're down-weighted or excluded. Judge criteria never silently fabricate a number — schema requires a backend field and errors stay per-criterion.
* **Economies conditioned on completion (from the 2026-10-07 night grading):** CoT/power on early-terminated (fell) episodes are misleading — a policy that falls in 3 s racks up a *better* CoT than one that walks the full 14 s. Economy columns get a `condition: completed` flag in the spec (fall-seeded episodes inherit the worst-case norm), or the selector fallback ordering handles it (fall count before CoT).

## 7. Training inspector (marimo notebook)

Tom's add: it must be easy to *see* what training is doing. A **marimo notebook** ships with the grader (`sim/gradebook/inspect_gradebook.py` — marimo notebooks are plain `.py`, reactive, no hidden cell state) reading only already-versioned artifacts:

* **Grade-card trends:** `normalized_total` and per-criterion scores across successive nightly runs; champion lineage; fall rate & CoT overlays.
* **Per-scenario breakdown:** criteria heatmap (scenario × criterion), seed spread, judge confidences once v2 lands.
* **Trace inspector:** `up_z`/height/planar-vs-t for any (scenario, seed), fall/recovery markers, command schedule — one reactive selector drives every plot.
* **Side-by-side run compare** for champion-vs-candidate; reel clip/filmstrip embed for the selected episode (the render path already exists).

No new capture for the notebook's sake — it's a read-only consumer of grade cards, traces, and reels, so it inherits versioning (`spec_version` selector) instead of fighting it. The reactive marimo model fits the referee loop: change the run selector and every downstream cell recomputes, no stale-notebook trap.

## 8. Migration & integration

* **Additive.** Existing scorecards are frozen and stay comparable (the 144-seed totals). Grade cards are new files (`grade_v{spec}.json/.md`) under `runs/<run>/`. The grader never writes to a scorecard file.
* New scenario = YAML entry; the frozen registry in `eval_precision.py` stays, and each card row declares its mapping to the frozen registry names so old cards keep meaning what they meant.
* Wiring: `eval_precision.py --grade [spec.yaml]`; nightly cron passes the champion's criteria spec explicitly. Default-on only after the parity gate.
* **Claude keeps:** rubric authoring (drafting criteria YAML when a new judgment call appears), contesting grade cards, auditing reels. **Claude loses:** the judging seat — verdicts come from the card.

### 8.1 Trust ladder (before the grader displaces Claude in the nightly loop)

1. **Shadow week:** grader runs alongside Claude's reads; agreement measured on the frozen criteria set; disagreements reviewed, criteria/normalization tuned. (Run-length configurable, not hard-coded.)
2. **Gate+audit:** the automated grader picks the champion; Claude spot-checks cards on the nights he reads anyway.
3. **Automated gate:** Claude only audits on request. Disagreement > threshold (YAML-tunable) still pages for review.

### 8.2 Acceptance criteria

* **A1 parity:** grading a historical run reproduces the frozen scorecard's successes/fall_count/verdict for the same seeds (grader reads the same rollout data; mismatch ⇒ extractor bug).
* **A2 config-as-data:** two criteria-spec variants on the same rollout produce two internally consistent cards, with zero edits to `eval_precision.py`.
* **A3 RL shape:** `R_ep = Σ w_c · norm_c` computes in numpy from a stored card. No model, env, or judge in the loop.
* **A4 honest failure:** missing/invalid judge key ⇒ per-criterion error fields, card marked incomplete; never a fabricated verdict, never a silent fallback.
* **A5 card usability:** `grade.md` renders, evidence links resolve, FAIL takes labeled.
* **A6 immutability:** running the grader leaves every existing scorecard byte-identical.

### 8.3 Risks

* **Trace extractor is the critical path.** Some scenarios compute verdict logic *mid-rollout* (contact fractions, swing keyframes) and don't persist intermediates. **Chosen mitigation: capture-side hooks** — factories write their evidence into the Trace as they compute it (single source of truth), rather than the grader re-deriving verdict logic and risking grader-vs-scenario disagreements.
* **Normalization windows are guesses until validated** — retune in shadow week against historical scorecards.
* **Judge confidence semantics differ per backend** (Jev: real probabilities; Claude: calibrated-ish self-report; local: unreliable). Cards record backend per criterion; confidence is informational until validated.
* **Jev account:** console is Cloudflare-gated from the box (403 on curl and browser) and needs a human identity step Tom hasn't done. The Jev backend stays listed-but-unused until then; the design doesn't depend on it.
* **Two-verdict-files problem** (grade card vs scorecard) until convergence — mitigated by the manifest, A1 parity, and registry-name mapping in every card.
* **Grade-card sprawl** — bounded by manifest + spec-version discipline (variants share rollout data, so don't mint spec versions casually).

## 9. Build slices

1. **Trace capture** (hooks inside the scenario factories) — critical path, enables everything.
2. Grading engine + metric criteria + card writers (JSON/MD).
3. A1 parity run on historical runs.
4. Criteria YAML + suite config + A2 variant proof.
5. RL adapter + nightly champion selection (A3).
6. Judge adapters (`jev`/`claude`/`local`, visual state builders) + shadow-week A/B.
7. `grade.md` polish + reel evidence links.
8. Training inspector notebook (`inspect_gradebook.py`) — trends, heatmap, trace inspector, compare view.

## Appendix A — v1 criteria (as shipped in the prototype, calibrated on the 2026-10-07 night cards)

Literature anchors: IsaacLab velocity-suite reward terms (track-lin/ang exp-kernels, feet-slide, action-rate), CHRL 2024 (velocity/orientation tracking + smoothness/safety/energy blocks), humanoid-velocity RL arXiv:2407.05148 (upright/height/torque/action-rate penalties), probabilistic-gait intervals arXiv:2011.01387. Windows = tonight's observed distributions; retune in shadow week.

| criterion | weight | source | window | direction |
|---|---|---|---|---|
| task_success | 3.0 | successes/n | [0, 1] | higher |
| fall_rate | 2.0 | fall_count/n | [0, 1] | lower |
| stability_wobble | 2.0 | wobble_rms | [0.3, 2.0] | lower |
| cot | 1.5 | CoT (loco) | [2.0, 45.0] | lower |
| power | 0.5 | mean_watts | [20, 200] | lower |
| foot_slip | 1.0 | foot_slip | [0.01, 0.08] | lower |
| symmetry | 1.0 | symmetry | [0.05, 0.5] | lower |
| swing_clearance | 1.0 | swing_clear_frac | [0, 0.4] | higher, optional col |
| lift_quality | 1.0 | lift_frac | [0, 0.5] | higher, optional col |

v2 judge candidates (unchanged): `gait_naturalness`, `reads_stumble`, `contact_sequencing` — rubric-style calls moved into YAML after trace capture makes their inputs cheap.

**Prototype evidence (2026-10-07):** `grade_v1.py` + `criteria_v1.yaml` (scratch `gradebook/`); graded robot_walk_k1 → 0.297 (0.295 with clearance cols), robot_walk_k2 → 0.340 (0.327) — consistent with seed_pass 11/144 vs 20/144; frozen scorecards byte-identical after grading (A6); optional-column path exercised via the referee-clearance worktree cards. CoT-on-falls pathology found → §6 conditioning note.

---

## Open questions (decisions, not blockers)

1. **Criteria catalog:** Appendix A draft — want additions, weight changes, or removals?
2. **Nightly authority during the trust ladder:** grader picks the champion while Claude's reads are non-binding audit (default), or grader stays advisory until full trust?
3. **Jev account:** TypeSafe signup needs the console (403 from this box) + a human identity step. Prep the config so `claude-consult` is the judge backend until an account exists?
4. **Build location:** `sim/gradebook/` module + this doc at `docs/gradebook-design.md` — confirm.