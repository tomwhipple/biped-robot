# tools/vision: camera monitoring for hardware runs

For an AI operator supervising the biped on its board: a recorder that owns the camera, a
clip cutter, an AprilTag tracker (position, heading, tilt, distance to the board edges), a
per-run report, and a pre-run "where is it / is there room" check. Measurement tools; no
language model is involved. Everything uses only OpenCV (with `cv2.aruco`), numpy, scipy and
PIL from the project venv.

| file | what |
| --- | --- |
| `make_tags.py` | print-ready AprilTag 36h11 sheet (US Letter, 100 %) + placement guide |
| `camd.py` | the camera daemon: one process owns the Sony (or the emeet), rolling 60 s segments |
| `clip.py` | cut a clip from segments by UTC time |
| `tag_track.py` | tag tracker (video / image -> JSONL/CSV, annotated video, calibration report) |
| `run_report.py` | per-run `<run>_vision.json` + contact-sheet JPEG |
| `where.py` | one frame -> position, heading, edge margins; exit code for a travel direction |
| `implausible.py`, `foreign.py` | telemetry-only "somebody helped" check; image-only motion hint |
| `vgeom.py`, `tagdefs.py`, `vsynth.py`, `detect_study.py` | geometry/config, tag ids, synthetic renderer, detectability study |

## Commands

```
# tags: print apriltags_letter.pdf at 100 % and check the 50 mm bars with a ruler
tools/vision/make_tags.py -o hw_sessions/2026-11-16/vision

# recorder (one owner of the camera; a second start refuses, exit 3). Start it like this:
setsid nohup tools/vision/camd.py --source sony >> ~/cam-segments/camd.log 2>&1 &
tools/vision/camd.py --source emeet --dir ~/cam-segments-emeet     # second stream, /dev/video0
tools/vision/clip.py --from 2026-11-16T16:36:10Z --to 2026-11-16T16:36:25Z -o catch.mp4

# tags
tools/vision/tag_track.py clip.mp4 --config vision.json -o clip_track.jsonl --annotate clip_ann.mp4
tools/vision/tag_track.py frame.jpg --calibrate --config vision.json       # f, residuals, camera position

# one run
tools/vision/run_report.py track_20261116T093534.jsonl --video-dir hw_sessions/2026-11-16 --frec-dir hw_sessions

# before a run (reads camd's newest frame, never opens the camera); exit 0 ok / 3 too close / 2 not seen / 4 no frame
tools/vision/where.py --direction forward --min-margin 300
```

`tools/sony_record.sh` refuses (exit 3, no file) while camd or any other liveview reader holds
the camera, and deletes its files (exit 1) if no frame arrives within 15 s. With camd running,
use `clip.py` instead of `sony_record.sh`. Segments default to `~/cam-segments` (`$CAMD_DIR`),
6 h retained (`--keep-hours`). camd's single-instance guard is a `flock` on
`DIR/<name>.lock` (not `pgrep -f`, which can match its own command line).

Host tool, not wired in yet: before the track starts, `track_host.py` would call
`subprocess.run([sys.executable, "tools/vision/where.py", "--direction", "forward", "--min-margin", "300"])`
and stop on a return code other than 0 (2 = tags not visible: unknown, not safe).

## Frames and numbers

Board frame: origin = near-left corner (as seen from the Sony), x right, y away from the Sony
(far end), z up, mm. Heading 0 = along +y, counter-clockwise positive. Pitch > 0 leaning
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
  before foreshortening) at 60 deg tilt, about 50 px at 70 deg. At f = 870 px: 60 mm tag = 104 / 52 /
  35 px at 0.5 / 1.0 / 1.5 m; 80 mm = 139 / 70 / 46; 40 mm = 70 / 35 / 23.
* **Focal length (Sony 1024x680)**: about 870 px, bracket 800-960, from the board's far end in the
  2026-11-16 and 11-13 footage (nominal 900 mm wide, 1200 mm long). Not a calibration.
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
* **foreign_motion is unreliable.** Foreground against the clip's median frame, robot excluded. At
  its default threshold it fires on most clips, including ones whose only "foreign" thing is the
  person in the chair or a sock at the edge of the board, and on the dog (cmd_steer_T4_b at
  16:48:31-16:48:44) and the catch (gate_push_march_T5_b). It cannot tell a hand from a leg. Treat an
  event as "look at the key frames".
