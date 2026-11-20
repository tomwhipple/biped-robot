# 2026-11-20: local vision models as a second opinion on key frames (#316, layer E) — bake-off

*Dated record. The tool and how to run it: [tools/vision/README.md](../../tools/vision/README.md), "Layer E".
Raw answers: [vlm-bakeoff-2026-11-20/](vlm-bakeoff-2026-11-20/).*

## Question

Can a local vision-language model, run on the CPU (the GPU belongs to night training and, by day,
to camera-watcher), give a useful coarse second opinion on a run's key frames: *is a person or an
animal at the robot*, *is the robot upright*, *is there an object on the board*? Useful = it flags
the frames a person should look at, at a false-alarm rate a person will tolerate. It never gates.

## The labelled set

55 frames, every one looked at and labelled by hand, listed in
[`tools/vision/vlm_set.json`](../../tools/vision/vlm_set.json) (video, UTC time, frame index, labels, a
one-line note); `vlm_bakeoff.py extract` rebuilds the JPEGs (<= 1280 px wide) from `hw_sessions/`, which is
not in git. 31 frames from the high eMeet (1920x1080), 24 from the low Sony (1024x680, torso often cut
off at the top); 52 from 2026-11-16, 2 from 11-12, 1 from 11-13.

| event | frames | what |
|---|---|---|
| normal | 13 | steps mid-board on both cameras and all three days |
| catch | 9 | the 11-16 push test, 16:36:15-22Z: before, at ~20 deg on the way down and back up, the 30 deg peak with Tom's hand at the torso, after |
| dog | 7 | the dog beside the robot in `cmd_steer_T4_b` (16:48:27-43Z), and a second, small dog walking onto the board at 22:14Z |
| fall | 11 | falling and lying robots, 21:01-22:05Z and the two evening falls (03:07, 03:13Z on 11-17) |
| pickup | 9 | a person stepping up to / reaching for / carrying the fallen robot |
| beside | 3 | a person standing next to the board (not touching); a person seated with feet at the board's edge |
| near_edge | 2 | robot at the left edge of the board |
| empty | 1 | board with no robot |

Labels (rules in `vlm.LABEL_RULES`): *person or animal* 21 yes / 32 no / 2 not scored (hand hidden or
out of frame); *upright* 36 yes / 18 no / 1 not scored (no robot). Upright was checked against the
flight-record IMU tilt at each frame's time wherever a record exists (all but two frames): from the low
Sony view, legs in single support look tilted while the torso is level, and the push-test frames that
look ~20 deg are 1-4 deg on the IMU; the real ~20 deg frames are 15.85 and 16.18 s, either side of the
30 deg peak. *Object on the board*: **no positive frame exists in this footage**: nothing loose ever lay
on the board on camera (tools, a jacket, a C-clamp were on the carpet beside it). 51 no, 4 not scored (a
thick black cable across the near end on 11-16 before 21:05Z whose owner the frame cannot show). So that
question is scored for false alarms only.

Gaps in the footage: the 11-12 and 11-13 falls in the flight record (11-12 20:21, 23:25Z; 11-13 01:09,
02:27, 15:20, 15:40Z) are on no video (those days' clips start minutes after them), nor are the 11-16
17:12Z tag-fitting excursions and the 17:24Z fall; the "~20 deg lean, sleeve at the frame edge" frame from
the 11-16 GPU test was not identified exactly, the two ~20 deg catch frames stand in for it.

## Setup

- **Server:** a separate CPU-only ollama 0.20.2 (`CUDA_VISIBLE_DEVICES=` empty, its own model store,
  127.0.0.1:11500, one model loaded, one request at a time, `nice 19`); every request `num_gpu: 0`,
  `num_thread: 12`, temperature 0, JSON-schema output. `nvidia-smi` showed no ollama process on the GPU at
  any check during the night (only the night training's python). The box was also running night
  training, so the timings are under load.
- **Models.** The baseline, qwen3-vl:8b-instruct, could not be run: this ollama can no longer pull it
  (registry: HTTP 412 "requires a newer version of Ollama"; the same for every qwen3-vl and gemma3 tag),
  and the copy in the system ollama (which runs on the GPU) was off limits. Run instead: qwen2.5vl:7b (the
  other model of the 11-16 GPU test), minicpm-v:8b, granite3.2-vision:2b (llama3.2-vision:11b and
  qwen2.5vl:3b were candidates; the download ran at ~1.6 MB/s, which set the order).
- **Prompts** (`vlm.PROMPTS`): *answer-first* (three booleans, then a note) and *describe-first* (the
  model writes its description of the robot's pose and of every person, hand, animal or object first,
  then the booleans). The prompt states the setup (brown board, the tether and markers are normal) and
  the three questions as written in `vlm.QUESTIONS`; it was written before any model answer was seen
  and not tuned afterwards.
- **Scoring:** the positive class is what a person should look at: person/animal *yes*, upright *no*,
  object *yes*. Precision = how many of the flagged frames were right; recall = how many of the frames
  to flag were flagged. An unparseable answer counts as "nothing to report". Seconds per frame = the
  server's processing time (load excluded after the first frame).

## Results

Positive = the frame a person should look at (person/animal yes, upright **no**, object yes). P =
precision, R = recall. s/frame = the server's CPU time per frame at 12 threads, median and max, with
night training running on the same box (and, for part of each run, the push gates or a second request).

| model (prompt) | frames | person/animal P / R | not upright P / R | object: false alarms | CPU s/frame (median, max) |
|---|---|---|---|---|---|
| qwen2.5vl:7b (answer-first) | 55 | 1.00 / 0.24 | 1.00 / 0.50 | 1 of 51 | 63, 123 |
| qwen2.5vl:7b (describe-first) | 18 | – / 0.00 | – / 0.00 | 0 of 16 | 62, 72 |
| minicpm-v:8b (describe-first) | 55 | 1.00 / 0.33 | 0.57 / 0.22 | 29 of 51 | 74, 130 |
| granite3.2-vision:2b (describe-first) | 6 | – / 0.00 | – / 0.00 | 0 of 6 | 96, 98 |

qwen2.5vl:7b describe-first was run on the dog and fall frames only (18), for lack of time; granite was
stopped after 6 frames (below). minicpm-v and qwen2.5vl answer-first ran the whole set.

Where the misses are (frames found / frames to find):

| model (prompt) | dog: person/animal (7) | catch, pickup, beside: person (14) | fall: not upright (10) | catch, pickup: not upright (8) |
|---|---|---|---|---|
| qwen2.5vl:7b (answer-first) | 0/7 | 5/14 | 4/10 | 5/8 |
| qwen2.5vl:7b (describe-first) | 0/7 | – | 0/10 | – |
| minicpm-v:8b (describe-first) | 0/7 | 7/14 | 0/10 | 4/8 |
| granite3.2-vision:2b (describe-first) | – | 0/3 | – | 0/1 |

For reference, the 2026-11-16 test on the **GPU** (system ollama, three frames, see #316): qwen3-vl:8b-instruct
~1 s per frame, right on the 30 deg catch frame (not upright, hand visible), wrong on the ~20 deg lean with
a sleeve at the frame edge; qwen2.5vl 2.3 s per frame, wrong on the lean.

Per-frame answers of every model: `tools/vision/vlm_bakeoff.py score --frames-table` on the raw answers in
[vlm-bakeoff-2026-11-20/](vlm-bakeoff-2026-11-20/) (one JSONL per model and prompt: the model's raw text, the
parsed answer, timings).

## Reading

**No model that runs here is good enough for a second opinion, not even a coarse one.** The reasons:

- **The dog is invisible to them.** Both full runs answered "no person or animal" on all 7 dog frames,
  including the tail sweeping past the robot's knees and the small dog walking across the board, and
  minicpm-v's own descriptions never mention a dog (qwen2.5vl's do, see the prompt note below). The dog
  was the motivating case of layer E.
- **A fallen robot is "upright".** With nobody near it, minicpm-v called 10 of 10 fallen or falling
  robots upright, qwen2.5vl 6 of 10 (both cameras; the robot lying on its side in plain view). The
  answers read like the prompt's own words ("the robot is upright ... no loose objects other than its own
  cable and printed markers"): the models fill the form from the prompt, not from the picture.
- **What they do see is a big hand.** Person/animal precision is 1.0 for both, but recall comes only from
  frames where a hand or arm fills a large part of the picture (a hand on the torso, someone bending over
  the board). That is the case a person would never miss on the contact sheet.
- **minicpm-v false-alarms on objects** (29 of the 51 frames with nothing loose on the board) and
  calls a robot "not upright" when a hand holds it level; useless for the object question.
- **granite3.2-vision:2b** was stopped after 6 frames: it parroted the prompt on every one (including the
  30 deg catch) at 96 s per frame (it tiles a 1280 px frame into ~5,500 image tokens).
- **CPU cost:** about 60-75 s per frame for the 7-8B models at 12 threads, so the 8 key frames of a run
  would add ~10 minutes to its report. Affordable if the answers were worth it; they are not.

The prompt style did not rescue it, but it shows where the failure is. With describe-first, qwen2.5vl's
own description **names the dog** in 4 of 5 frames of the 11-16 morning ("a dog is partially visible on
the right side of the frame", "a dog is sniffing near the robot") and the fifth says "possibly a person or
an animal", yet every boolean that follows says no person or animal (it seems to read "beside the board"
as not "at the board"). On the falls the description itself is wrong ("the robot is standing upright on
the brown wooden board" for a robot lying on its side, 10 of 10). So the free text is a better dog
detector than the yes/no answer, and neither sees a fallen robot from these camera angles.

**What would change the answer.** In the 11-16 GPU test qwen3-vl:8b read the 30 deg catch correctly
where qwen2.5vl was not asked (and both missed the 20 deg lean): one frame, but the only sign of a model
that might do better. It needs a newer ollama than the
installed 0.20.2 (a system install by Tom, after a dependency review), and on the CPU it would cost
about what qwen2.5vl does. Then: re-run this bake-off with `vlm_bakeoff.py run --model qwen3-vl:8b-instruct`
(the set and the scorer are ready, any `--host`), add a few staged frames with an object on the board
(the set has none), and if it finds the dog and the falls, merge the `run_report --vlm-model` pass from
the branch `tools/316-vlm-run-report-pass` (answers shown next to the IMU / tag values per key frame,
never a gate).

**Found on the way:** `run_report.py` cut a run's clip from the one camd segment that covered most of
the run, so a run crossing a minute boundary lost the rest: on 2026-11-16 21:00:17Z the fall at
21:01:26Z was not in the clip, and the contact sheet's "fall" tiles showed the segment's last frame (the
robot still standing). Fixed in this branch (clips are cut across camd segments).
