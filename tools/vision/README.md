# tools/vision: camera monitoring for hardware runs

**Cameras.** The eMeet C960 on its high mount (/dev/video0, MJPEG 1920x1080) is the MEASUREMENT camera: tags, position, tilt. The Sony (low, tripod) is context video only: from there the board tags are ~55x12 px and nothing decodes. Both can be recorded by two camd processes (`--source emeet`, `--source sony`; separate locks), `where.py` and `calibrate.py` default to the eMeet, and `run_report.py` tracks on a video named `emeet*` and adds a context contact sheet from a `sony*` one.

For an AI operator supervising the biped on its board: a recorder that owns the camera, a
clip cutter, an AprilTag tracker (position, heading, tilt, distance to the board edges), a
per-run report, and a pre-run "where is it / is there room" check. Measurement tools; no
language model is involved (layer E below tested local vision models as a second opinion and
found none good enough). Everything uses only OpenCV (with `cv2.aruco`), numpy, scipy and
PIL from the project venv.

| file | what |
| --- | --- |
| `make_tags.py` | print-ready AprilTag 36h11 sheet (US Letter, 100 %) + placement guide |
| `camd.py` | the camera daemon: one process owns the Sony (or the emeet), rolling 60 s segments |
| `clip.py` | cut a clip from segments by UTC time |
| `calibrate.py` | check the setup from ONE frame: camera pose, f, per-tag reprojection error (run after every re-aim) |
| `tag_track.py` | tag tracker (video / image -> JSONL/CSV, annotated video, calibration report) |
| `run_report.py` | per-run `<run>_vision.json` + contact-sheet JPEG |
| `where.py` | one frame -> position, heading, edge margins; exit code for a travel direction |
| `implausible.py`, `foreign.py` | telemetry-only "somebody helped" check; image-only motion hint |
| `vgeom.py`, `tagdefs.py`, `vsynth.py`, `detect_study.py` | geometry/config, tag ids, synthetic renderer, detectability study |
| `vlm.py` | layer E: three yes/no questions about a frame to a local ollama vision model (CPU by default) |
| `vlm_bakeoff.py`, `vlm_set.json` | layer E: the labelled frame set (rebuilt from footage) and the model bake-off |

## Commands

```
# tags: print apriltags_letter.pdf at 100 % and check the 50 mm bars with a ruler
tools/vision/make_tags.py -o hw_sessions/2026-11-16/vision

# recorder (one owner of the camera; a second start refuses, exit 3). Start it like this:
setsid nohup tools/vision/camd.py --source sony >> ~/cam-segments/camd.log 2>&1 &
tools/vision/camd.py --source emeet --dir ~/cam-segments-emeet     # second stream, /dev/video0
tools/vision/clip.py --from 2026-11-16T16:36:10Z --to 2026-11-16T16:36:25Z -o catch.mp4

# after every re-aim of the eMeet (newest camd frame, or pass a jpg):
tools/vision/calibrate.py --source emeet            # or: calibrate.py frame.jpg --config vision.json

# tags
tools/vision/tag_track.py clip.mp4 --config vision.json -o clip_track.jsonl --annotate clip_ann.mp4
tools/vision/tag_track.py frame.jpg --calibrate --config vision.json       # f, residuals, camera position

# one run
tools/vision/run_report.py track_20261116T093534.jsonl --video-dir hw_sessions/2026-11-16 --frec-dir hw_sessions

# before a run (reads camd's newest frame, never opens the camera); exit 0 ok / 3 too close / 2 not seen / 4 no frame
tools/vision/where.py --direction forward --min-margin 300
# ... and refuse a walk whose END would be within 300 mm of an edge: net displacement in the robot frame
# (forward / left, mm), rotated by the tracker's heading; the predicted end margins are printed, with +-100 mm
# next to them while f comes from the coplanarity estimate
tools/vision/where.py --plan-dx-mm 200 --plan-dy-mm 0 --min-margin 300
```

`tools/sony_record.sh` refuses (exit 3, no file) while camd or any other liveview reader holds
the camera, and deletes its files (exit 1) if no frame arrives within 15 s. With camd running,
use `clip.py` instead of `sony_record.sh`. Segments default to `~/cam-segments` (`$CAMD_DIR`),
6 h retained (`--keep-hours`). camd's single-instance guard is a `flock` on
`DIR/<name>.lock` (not `pgrep -f`, which can match its own command line).

Host tool, not wired in yet: before the track starts, `track_host.py` would call
`subprocess.run([sys.executable, "tools/vision/where.py", "--direction", "forward", "--min-margin", "300"])`
and stop on a return code other than 0 (2 = tags not visible: unknown, not safe).

## Tags as placed (2026-11-16): provisional layout, and the fallback

**Default layout = the placement guide** (`board.measured: false`): tag centres 60 mm from both edges
of their corner (80 mm tag, 20 mm clearance), arrow away from the Sony, ids 0-1 far, 2-3 near, 4-5
mid-edge, corner labels as seen from the Sony (the eMeet sees 2 at image right, 3 at image left: it
looks the other way). `calibrate.py` reports how well that fits: on the real frames the tags 2 and 3
are 704-726 mm apart as seen by the camera vs 692.8 mm in the layout (ratio 1.02-1.05, plane normals
within 2 deg, tag reprojection 0.8-4 px): consistent, not exact. A config file with `board.tags` marks
the layout measured.

Below is the fallback `"layout": "auto_end"`, kept for when the layout is unknown:

Robot: torso BACK = id 11 at 40 mm, torso TOP = id 12 at 60 mm, feet 13/14 at 25 mm (reported as
`foot_L`/`foot_R`, not fused into the body pose), no front tag (the default config has only 11 and 12;
add a `"10"` entry to use one). Board tags are NOT at the nominal corner labels (the eMeet sees ids 2
and 3 at the end where the robot stands), so the default board layout is **`auto_end`**, used until a
measured `board.tags` config is given (which switches to `config` automatically):

* it takes the two most widely separated board tags, assumes they lie in the board plane near one
  end of the board, that the board end is on their centre line (`board.assumed_end_offset_mm`, 0) and
  that the board is symmetric about them (`board.assumed_side_inset_mm`, default (width - spacing)/2);
* f is the one at which the two tags' planar-PnP normals agree best (they are coplanar): a shallow
  minimum, +-20 % (bracket printed). On the real 2026-11-16 eMeet frames: f about 840-1035 px
  (bracket 600-1300), tag spacing measured 704-724 mm (stable over f = 700-1600);
* board frame: origin at the board end corner on the image right, +x along the end toward the image
  left, +y into the board toward the camera, z up (so heading 180 = walking toward the tags);
* accuracy: moving f over its +-20 % bracket moves the robot position by about 130 mm in y and 70 mm
  in x, heading by 5 deg, pitch/roll by 1 deg. Edge margins also depend on where the board edges
  really are relative to the tags. So until the layout is measured and f pinned: position and margins
  to about +-100 mm, heading +-5 deg, tilt +-2 deg. Relative motion within a run is much better.
* what pins f: a measured layout with tags spread over the board (all four in view), or one tag of
  known size held at a measured distance (px = f * size / distance).

`calibrate.py` prints all this (and warns when a robot tag reprojects > 3 px, i.e. the torso mount
offsets are still nominal: on the real frame tags 11 and 12 reproject at ~7.6 px under the nominal
mounts, about 10 mm of mount error).

## Frames and numbers

Board 32 x 48 in = 812.8 x 1219.2 mm (measured). Board frame: origin = near-left corner (as seen from
the Sony), x right, y away from the Sony (toward the eMeet), z up, mm. Heading 0 = along +y, counter-clockwise positive. Pitch > 0 leaning
forward, roll > 0 leaning to the robot's right (`roll_sign` in the config if the firmware
differs; `run_report.py` reports which sign matched the IMU). Edge margins are for the torso
reference point minus `foot_margin_mm`.

## What to measure (a tape measure; defaults are nominal and live in `vgeom.py`)

Put the values in a JSON that overrides the defaults, e.g. `vision.json`:

```json
{"board": {"width_mm": 905, "length_mm": 1198,
           "tags": {"0": {"center": [61, 1141], "size": 79.5}, "...": {}}},
 "robot": {"tags": {"10": {"center": [34, 0, 2], "size": 59.5, "normal": [1,0,0], "up": [0,0,1]}}},
 "camera": {"f_px": null}}
```

1. **Board outline**: total width across both panels, length.
2. **Each board tag (0-5)**: x = distance from the board's LEFT edge to the tag centre, y = distance
   from the NEAR end to the tag centre (as seen from the Sony); and the black square's side length
   (ruler/calipers: the printed size, "size").
3. **Torso**: width, depth, height of the box; for each of tags 10/11/12 the centre offset from the
   torso centre (x forward, y left, z up), which way its arrow points, and its printed size
   (60 mm, or the 40 mm spares). Foot tags 13/14 are optional and reported as foot_L/foot_R.
4. **Camera**: height of the lens above the board and its distance from the near edge. The images
   cannot tell the board's absolute scale (a board 2 % smaller looks the same from a camera 2 %
   closer), so `--calibrate` prints `camera_position_board_mm` and this tape measurement is the check.
5. Leave the lens zoom at the wide end; the focal length estimated from the board tags assumes it
   does not change (the estimate and its bracket are printed by `--calibrate`).

Calibration capture (after the tags are placed): a 5 s capture of the board with the robot out of
the way or standing still; `tag_track.py clip.mp4 --calibrate`. The tracker then holds f and the
board pose for the whole clip (camera assumed fixed; `board_rms_px` per frame shows when it moved).
On the real Sony view the near-corner tags (2, 3) are below the bottom of the frame and the far
tags (0, 1) are small and foreshortened: tags 4 and 5 (mid-edge) are what the Sony calibrates on;
the emeet at the far end sees 0-3.

## What is validated, and how

* **Tag sheet**: rendered, shrunk/blurred/warped, decoded (`detect_study.py`). Minimum outer-edge size
  that decodes every tag at four rotations: 16 px frontal, 24 px with 1.5 px blur, 32 px (nominal,
  before foreshortening) at 60 deg tilt, about 50 px at 70 deg. At the Sony's f = 790 px: 60 mm tag = 95 /
  47 / 32 px at 0.5 / 1.0 / 1.5 m; 80 mm = 126 / 63 / 42; 40 mm = 63 / 32 / 21. (The eMeet at 1080p
  sees tags 47-98 px.)
* **Focal length (Sony 1024x680)**: about 790 px (bracket 720-870) from the board's far end in the
  2026-11-16 and 11-13 footage, now using the measured 812.8 mm width (an earlier quote of 870 px
  assumed 900 mm and was ~10 % high). Not a calibration, and irrelevant for tracking now that the
  eMeet is the measurement camera. At 790 px a 60 mm tag is 95 / 47 / 32 px at 0.5 / 1.0 / 1.5 m.
* **Tracker: synthetic only** (`tests/test_vision_track.py`; no tagged footage exists yet). Camera
  fitted to the real footage (f 870, 475 mm high, 500 mm in front of the board, 20 deg down),
  60 mm torso tags, blur 1 px, noise 4: position within 10 mm, heading within 3 deg, tilt within
  2 deg over a walking sequence. Isolated frames with a single small tag are two-fold ambiguous in
  orientation (planar PnP): 22 of 60 random frames were flagged `ambiguous`, and those had heading
  errors up to 50 deg and pitch up to 24 deg; position was still within 24 mm. A sequence resolves
  them by continuity; the others are flagged, not hidden. Unambiguous frames: heading 0.4 deg rms,
  pitch 1.3, roll 0.1, x 0.4 mm, y (depth) 5 mm. The estimated f was 856 vs 870 true.
* **implausible_recovery**: validated on real data. Push run 2026-11-16 09:35:34 MST
  (`gate_push_march_c`): flagged, 30 deg roll at 16:36:15.95Z, back under 10 deg by 16:36:16.55Z, no
  fall latch (the gate also held at 16:36:15.1Z). The other 10 runs of that day (max tip 4-11 deg)
  produce no flag. The IMU series comes from `flight_20261113T172054Z_prec16.frec` (a session that
  stayed open from 11-13 to 11-16).
* **implausible.py waits** (default 90 s, `--wait-s`) for the flight record to cover the run's end: the
  collector flushes late, so a run checked right after the walk found no record. It now picks the
  session open at the run's start and waits until a decoded tick reaches the run's end; if it never
  does, the result says `partial` (not "no excursion").
* **run_report endings and positions.** The host log has no fall line: a run that fell has no TS line for
  the step it fell in, so `anomalies` lists `short_run` (fewer TS lines than steps decided), `firmware_abort`,
  `stop_rule`, `gate_timeout` (the host summary's `outcome`) and `fall_latch` (the flight record's EV_FALL event or
  TF_FALLEN tick flag); `ending` names them or says `clean`. `position_start` / `position_end` are the median over
  the first / last window of >= 1 s in which the torso tag is stationary (the tag sways +-80 mm with every step;
  `position_method` says how), `position_track_extent` is the min / max x and y over all tagged frames.
* **eMeet frames**: tags decode (ids 2, 3, 11, 12 at 47-77 px on the real 1080p frames); the
  tracker's accuracy on them is synthetic only (`tests/test_vision_track.py::test_emeet_*`): measured
  layout and known f give position within 10 mm and heading within 3 deg; the auto layout gives the
  numbers above.
* **foreign_motion_hints is unreliable** (kept in the report JSON only, not in the printed summary). Foreground against the clip's median frame, robot excluded. At
  its default threshold it fires on most clips, including ones whose only "foreign" thing is the
  person in the chair or a sock at the edge of the board, and on the dog (cmd_steer_T4_b at
  16:48:31-16:48:44) and the catch (gate_push_march_T5_b). It cannot tell a hand from a leg. Treat an
  event as "look at the key frames".

## Layer E: a local vision model as a second opinion (tested, not in use)

Measurement does not see a person or an animal at the robot, a cable snag or a thing on the board
(the dog on 2026-11-16 16:48:33Z showed in no telemetry). Layer E asked whether a local
vision-language model could flag such frames among a run's key frames. **Not yet: no model that
runs here is good enough** (bake-off below), so `run_report.py` has no model pass. The pass that
was built (`--vlm-model`: answers next to the IMU / tag values per key frame, a caption line on the
contact sheet, never a gate) is kept on the branch `tools/316-vlm-run-report-pass` for when a
better model is available.

RESULTS_README

**CPU, not the GPU.** The GPU belongs to night training (22:00-07:00) and by day to
camera-watcher, so models run on a separate CPU-only ollama (its own model store, port 11500),
and `vlm.ask` sends `num_gpu: 0` and `num_thread: 12` with every request (the rest of the box stays
with training):

```
mkdir -p ~/.cache/ollama-cpu
CUDA_VISIBLE_DEVICES= OLLAMA_HOST=127.0.0.1:11500 OLLAMA_MODELS=$HOME/.cache/ollama-cpu \
  OLLAMA_NUM_PARALLEL=1 OLLAMA_MAX_LOADED_MODELS=1 setsid nohup nice -n 19 ollama serve > /tmp/ollama-cpu.log 2>&1 &
OLLAMA_HOST=127.0.0.1:11500 ollama pull MODEL
```

Check after the first load that `nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv`
lists no ollama process, and stop the server when done. The installed ollama (0.20.2) can no
longer pull qwen3-vl or gemma3 (the registry answers 412, "requires a newer version of Ollama").

**The bake-off** (re-runnable against any ollama endpoint): `vlm_set.json` is a labelled set of
real frames (video, UTC time, frame index, labels, a one-line note; label rules in
`vlm.LABEL_RULES`), rebuilt from the footage in `hw_sessions/` (not in git) into
`hw_sessions/vlm_set/`. Two prompt styles (`vlm.PROMPTS`): answer-first, and describe-first (the
model writes its description before the booleans; default).

```
tools/vision/vlm_bakeoff.py extract                                   # frames -> hw_sessions/vlm_set/<id>.jpg
tools/vision/vlm_bakeoff.py run --model MODEL [--host URL] [--prompt describe-first] [--event dog]
                                                                      # raw answers -> hw_sessions/vlm_set/answers/
tools/vision/vlm_bakeoff.py score --md                                # precision / recall per label, s per frame
tools/vision/vlm_bakeoff.py score --frames-table                      # every frame: labels vs each model
tools/vision/vlm_bakeoff.py grab VIDEO --at 2026-11-16T16:48:33Z -o /tmp/look   # frames for choosing new labels
```
