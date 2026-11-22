# 2026-11-22: prompt tuning for qwen3-vl:8b-instruct on a held-out split (#316)

*Dated record, follows [2026-11-21-vlm-qwen3vl-bakeoff.md](2026-11-21-vlm-qwen3vl-bakeoff.md) (set, labels, scoring unchanged).
Raw answers: [vlm-tuning-2026-11-22/](vlm-tuning-2026-11-22/) (`tune/`, `heldout/`, `split.json`).*

## Setup

Same approved install as 11-21, nothing pulled: ollama 0.40.16 user-local CPU-only, the pinned `env -i` launch
(`CUDA_VISIBLE_DEVICES=`, `OLLAMA_VULKAN=0`, `OLLAMA_LLM_LIBRARY=cpu`, `OLLAMA_NO_CLOUD=1`) on 127.0.0.1:11500, store
`~/.cache/ollama-cpu`, qwen3-vl:8b-instruct, `nice 19`, one request at a time, `num_gpu: 0`, `num_thread: 12`, temperature 0.
Log showed `library=cpu`. `nvidia-smi --query-compute-apps` listed only the night training's python, checked after load and
again after the held-out run. Server stopped afterwards (nothing listens on 11500).

## Split

`vlm_bakeoff.py split` (seed 316, `make_split`): frames sorted by id, put in a stratum (person/animal yes; else not
upright; else object; else normal), shuffled per stratum with `random.Random(316)`, dealt alternately. `--split tune|heldout`
on `run` and `score` restricts to one half. Ids: `vlm-tuning-2026-11-22/split.json`.

| half | frames | person/animal yes | not upright | object yes | of which dog frames | frames with a null label |
|---|---|---|---|---|---|---|
| tune | 28 | 11 | 8 | 0 | 5 | 2 |
| heldout | 27 | 10 | 10 | 0 | 3 | 5 |

No object-on-board positive exists in the set, so that label can only show false alarms (none anywhere: 0 of 27 and 0 of 24).

## Variants (tuning half only)

All three put explicit definitions in the prompt (`vlm._DEFS`): "not upright" = lying, on its side or back, seen from
the side with a horizontal or clearly slanted torso, one leg on the board and body tipped, falling, or leaning past a
slight tilt, and "when unsure, answer false for robot_upright"; a person counts from a hand, sleeve or leg; an animal
counts from a tail, legs, head or paw, small, far or partly in view.

- `defs`: describe-first (note, then booleans) + definitions. One call.
- `two-call`: definitions, one short call for person/animal + object (describe, then answer), a second for upright
  (angle and front/side, then answer); answers merged.
- `defs-af`: answer-first (booleans, then a one-line note) + definitions. One call.

Tuning half (28 frames; positive = person/animal yes, not upright; P = precision, R = recall; counts tp/fp/fn/tn):

| prompt | person/animal P / R (tp/fp/fn/tn) | not upright P / R (tp/fp/fn/tn) | median s/frame |
|---|---|---|---|
| answer-first (11-21 prompt) | - / 0.00 | 1.00 / 0.62 | 23 |
| describe-first (11-21 prompt) | 0.88 / 0.64 | 1.00 / 0.12 | 31 |
| defs | 0.83 / 0.91 (10/2/1/14) | 1.00 / 0.38 (3/0/5/20) | 29 |
| two-call | 0.80 / 0.36 (4/1/7/15) | 1.00 / 0.50 (4/0/4/20) | 12 (2 calls) |
| **defs-af** | **0.83 / 0.91 (10/2/1/14)** | **1.00 / 1.00 (8/0/0/20)** | 8 |

Picked `defs-af` (best on both labels). Tuning-half misses: person/animal false alarms `catch_push_163615a`,
`em_lying_213042`, miss `catch_held_163618`. No prompt change after this point.

## Held-out half (scored once, `defs-af`, 27 frames, 0 errors)

| label | P | R | tp / fp / fn / tn |
|---|---|---|---|
| person/animal | 1.00 | **0.80** | 8 / 0 / 2 / 16 |
| not upright | 0.78 | **0.70** | 7 / 2 / 3 / 14 |
| object on board | - (no positive) | - | 0 false alarms in 24 |

Misses: person/animal `em_dog_221405` (the small dog; the other 2 held-out dog frames were found) and
`catch_tip20_back_163616`; not upright `em_lying_210132` (side-view fall), `catch_tip20_down_163615`,
`catch_tip20_back_163616` (the two ~20 deg leans). False "not upright": `em_hand_carry_210424`, `em_hand_lift_220520`
(robot carried by a hand, labelled upright).

## Cost

Held-out: median 25 s/frame (mean 25.1, max 28), one call, ~54 output tokens; the box was running the night training.
On the tuning half the same prompt took 8 s/frame, but that run followed `defs` on the same images (prompt eval 1.4 s
vs 17.9 s on the held-out frames), so the image-embedding / prefix cache very likely helped; the 25 s is the honest
number for a fresh frame, in line with 22-31 s of 11-21. `two-call` was 12 s/frame (two short calls, same caching
caveat); `defs` 29 s.

## Verdict

Target recall >= 0.85 on a held-out split: **not met.** Person/animal 0.80 (8 of 10), not upright 0.70 (7 of 10)
on the held-out half, against 0.91 and 1.00 on the half it was tuned on: the tuning half overstated it (small
halves: one frame is 10 points of recall, so the held-out numbers carry roughly +-0.15). It is a clear gain over
either 11-21 prompt on its own (both 0.67 overall; person 0.29/0.67, not upright 0.67/0.22) at one call per frame
rather than two. Not-upright precision fell to 0.78 (carried-by-hand frames), and the side-view fall and the small
far dog are still missed.

## Recommendation

- Use `defs-af` as the single layer-E prompt (`vlm.PROMPTS["defs-af"]`; it is not yet the default). One call, ~25 s a
  frame, so the 8 key frames of a run cost about 3-4 minutes.
- Still a flag-only hint, never a gate and never "no flag = fine": about 1 in 5 person/animal and 3 in 10 falls are silent.
- Do not merge the parked run-report pass as a safety net on this evidence; as an "extra flag" line beside the IMU / tag
  values it is defensible, if Tom accepts a ~75 % recall hint.
- To reach 0.85 with confidence: more labelled frames (side-view falls, small or far dogs, hand-carried frames) and a
  split of a few hundred, not 55; a second held-out pass only after new frames exist, since this half is now spent.
