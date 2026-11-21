# 2026-11-21: qwen3-vl:8b-instruct on the layer E bake-off (#316)

*Dated record, follows [2026-11-20-vlm-second-opinion-bakeoff.md](2026-11-20-vlm-second-opinion-bakeoff.md) (the set, the
labels, the prompts and the scoring are defined there and unchanged). Raw answers:
[vlm-bakeoff-2026-11-21/](vlm-bakeoff-2026-11-21/) (one JSONL per prompt style).*

## Setup

The baseline model that the 11-20 run could not pull. The pull needed a newer ollama; a dependency review
tonight approved **ollama v0.40.16** as a user-local, CPU-only install (tarball unpacked without its CUDA and
Vulkan backends into `~/.local/opt/ollama-0.40.16/`, nothing on PATH; pins and review in
`~/code/claude-runs/reviews/biped-robot-316-ollama.{pins.txt,review.md}`). Server by hand on 127.0.0.1:11500, the pinned
`env -i` launch (`CUDA_VISIBLE_DEVICES=`, `OLLAMA_VULKAN=0`, `OLLAMA_LLM_LIBRARY=cpu`, `OLLAMA_NO_CLOUD=1`), one model, one
request at a time, `nice 19`. The tarball sha256 and the install tree digest matched the pins; the model blobs
(copied from the review's verified store) matched all six pinned sha256. Log: `OLLAMA_NO_CLOUD:true`,
`OLLAMA_VULKAN:false`, one `inference compute` line with `library=cpu`. After the model loaded, `nvidia-smi
--query-compute-apps` listed only the night training's python, at every check. Requests: `vlm.py` as is
(`num_gpu: 0`, `num_thread: 12`, temperature 0, JSON-schema output), same prompts as 11-20, whole set (55 frames),
both prompt styles. The box was running night training (and, during the first part of the answer-first run,
the push gates).

## Results

Same table format as 11-20 (positive = the frame a person should look at: person/animal yes, upright **no**,
object yes; P = precision, R = recall):

| model (prompt) | frames | person/animal P / R | not upright P / R | object: false alarms | CPU s/frame (median, max) |
|---|---|---|---|---|---|
| **qwen3-vl:8b-instruct (describe-first)** | 55 | 0.93 / 0.67 | 1.00 / 0.22 | 0 of 51 | 31, 52 |
| **qwen3-vl:8b-instruct (answer-first)** | 55 | 1.00 / 0.29 | 1.00 / 0.67 | 0 of 51 | 22, 25 |
| qwen2.5vl:7b (answer-first) | 55 | 1.00 / 0.24 | 1.00 / 0.50 | 1 of 51 | 63, 123 |
| qwen2.5vl:7b (describe-first) | 18 | – / 0.00 | – / 0.00 | 0 of 16 | 62, 72 |
| minicpm-v:8b (describe-first) | 55 | 1.00 / 0.33 | 0.57 / 0.22 | 29 of 51 | 74, 130 |
| granite3.2-vision:2b (describe-first) | 6 | – / 0.00 | – / 0.00 | 0 of 6 | 96, 98 |

Where the misses are (frames found / frames to find):

| model (prompt) | dog: person/animal (7) | catch, pickup, beside: person (14) | fall: not upright (10) | catch, pickup: not upright (8) |
|---|---|---|---|---|
| qwen3-vl:8b-instruct (describe-first) | 4/7 | 10/14 | 1/10 | 3/8 |
| qwen3-vl:8b-instruct (answer-first) | 0/7 | 6/14 | 7/10 | 5/8 |
| qwen2.5vl:7b (answer-first) | 0/7 | 5/14 | 4/10 | 5/8 |
| minicpm-v:8b (describe-first) | 0/7 | 7/14 | 0/10 | 4/8 |

Running both prompts and flagging a frame if either flags it (not run as such; computed from the two answer
files, so 2 x 22-31 s per frame): person/animal 14/21 found with 1 false alarm (P 0.93, R 0.67), not upright
12/18 found with 0 false alarms (P 1.00, R 0.67). The one person/animal false alarm is `catch_push_163615a` (describe-first only, the frame just before the push); every "not upright"
and "object" flag in 55 frames was right (object: no positive frame exists, so false alarms only).

Against the 11-16 GPU test (3 frames, ~1 s/frame): there qwen3-vl read the 30 deg catch correctly (not upright,
hand visible). On the CPU run, describe-first has the hand (person: yes) but says "upright" on that frame;
answer-first says not upright but misses the hand. The ~20 deg lean frames: not upright was found on
`catch_tip20_down_163615` by both prompts, missed on `catch_tip20_back_163616` by both. CPU time is about half of
what the 7-8B models of 11-20 took (22-31 s median, not 60-75 s; the runtime is newer and the load on the box differed, so not a like-for-like timing), so the 8 key frames of a run cost 3-4 minutes per prompt.

Per-frame answers: `tools/vision/vlm_bakeoff.py score --frames-table --answers-dir docs/design-v6/vlm-bakeoff-2026-11-21`.

## Reading

- **It is the first model here that sees the dog**: 4 of 7 dog frames with describe-first (tail, legs behind,
  the small dog on the board missed: `dog_legs_164831`, `em_dog_221405`, `em_dog_221445`), where the others
  found 0 of 7. And the first with **no object false alarms** (minicpm-v: 29 of 51).
- **Precision is the strength, recall is not.** Not one wrong "not upright" in 55 frames and one wrong
  "person" flag; but the two prompts each miss what the other finds: describe-first gets the dog and the hands
  (and says "upright" for 9 of 10 fallen robots, as qwen2.5vl did), answer-first gets 7 of 10 falls but
  none of the dog. Neither finds the "seen from the side, one leg on the board" falls (`em_lying_210132`,
  `em_lying_213042`) with either prompt.
- **Verdict on the criterion of the 11-20 record ("if it finds the dog and the falls"): not met by any one pass.**
  Better than every model before it, and good enough to be a flag-only hint (a flag is nearly always right),
  but a frame it does not flag is not evidence that nothing happened: 7 of 21 person/animal frames and 6 of 18
  not-upright frames are silent even with both prompts, and the dog's two small, far frames are missed.
  So: **do not merge `tools/316-vlm-run-report-pass` as a safety net yet.** It would be defensible as an
  "extra flag" line next to the IMU / tag values (never a gate, never "no flag = fine"), run with **both**
  prompts per key frame (~1 minute per frame on CPU, ~8 minutes per run report).
- **What it would take** to call it good enough: (1) both prompts in the pass (or one prompt that does both:
  the failure is the model's "upright"/"beside the board" reading of the description, which a prompt change might
  fix; tune on a split, the 55 frames are now spent for that); (2) more labelled frames, including staged ones
  with an object on the board (no positive exists), side-view falls, and a small or far dog; (3) a held-out
  re-score with recall >= ~0.85 for the dog and the falls, or the decision that a high-precision, 67 %-recall
  flag is worth 8 minutes of CPU per run (Tom's call); (4) the pass wired as the branch has it: answers beside
  the IMU / tag values, never a gate.

Server stopped after the run (nothing listens on 11500); the install under `~/.local/opt/ollama-0.40.16/` and the
store `~/.cache/ollama-cpu` stay for a re-run.
