#!/usr/bin/env python3
"""One-run vision report: <run>_vision.json + <run>_vision.jpg (key-frame contact sheet).

    tools/vision/run_report.py track_20261116T093534.jsonl --video-dir hw_sessions/2026-11-16
        [--frec-dir hw_sessions] [--out-dir DIR] [--config vision.json] [--no-video]
        [--vlm-model qwen2.5vl:7b [--vlm-host http://127.0.0.1:11500] [--vlm-max-frames 8]]

Takes the roll-balance track log of one run, finds its UTC window (the flight record's
`cal show` command stamps t=0), cuts the video for that window from --video-dir (camd
segments or hand-recorded mp4+.pts), and writes:

  anomalies             every TS step with inward lean >= 8 or outward >= 6 deg, gate hold,
                        track stop, abort/fall line, firmware ABORT step, stop-rule ending, GATE-TIMEOUT,
                        a run with fewer TS lines than steps decided (short_run: the host log has no fall
                        line, a fall shows as a missing last TS), the flight record's fall latch
                        (fall_latch: EV_FALL event / TF_FALLEN tick flag), implausible recovery
  ending                "clean", or the kinds of ending anomalies (fall_latch, firmware_abort, stop_rule,
                        gate_timeout, short_run, abort_or_fall)
  implausible_recovery  TELEMETRY ONLY (flight record IMU): tip > 20 deg and back < 10 deg within
                        6 s without a fall latch = someone helped. Works without tags.
  foreign_motion_hints  image-only hint list (tools/vision/foreign.py); unreliable, fires on the robot itself,
                        so it is kept in the JSON only and left out of the printed summary, see README
  edge_margin_min_mm, edge_margin_min_at, position_start/end, position_track_extent, heading_start/end
                        from the tags; null when no tags are visible. Start / end = median over the first / last
                        window (>= 1 s) in which the torso tag is stationary (the tag swings +-80 mm sideways with
                        every step); position_method says how each was obtained
  tilt_video_vs_imu     max |tag roll/pitch - IMU roll/pitch| (needs tags); null without
  contact sheet         start, end and +-1 s around every anomaly
  vlm_second_opinion    OPTIONAL (--vlm-model, default off): a local vision-language model's yes/no answers on the
                        key frames (person/animal at the robot, upright, object on the board), each shown next to the
                        measured values at that moment (IMU tilt, tag tilt and edge margin) and as a second caption
                        line on the contact sheet. A hint for a person to look, never a measurement, never a gate:
                        nothing above reads it. CPU by default (tools/vision/vlm.py, README "Layer E")

Time of a TS step = the log time of the first `track steps` poll that lists it (up to one poll
period late); gate hold = the first `HOLDING` line; both +- ~1 s.
"""
import argparse
import datetime as dt
import glob
import json
import os
import re
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import clip as C  # noqa: E402
import foreign  # noqa: E402
import implausible as IM  # noqa: E402
import tag_track as T  # noqa: E402
import vgeom as G  # noqa: E402
import vlm as V  # noqa: E402

LEAN_IN_DEG, LEAN_OUT_DEG = 8.0, 6.0
ENDING_KINDS = ("fall_latch", "firmware_abort", "stop_rule", "gate_timeout", "short_run", "abort_or_fall")
STOP_RE = re.compile(r"(?i)aborts\s+(?!\s*none)|\bfell\b|fall latch|\bfallen\b|\babort(ed)?\b(?! or)")


def log_anomalies(rows, t0):
    """-> list of dict(kind, t_utc, detail) from the run log (t0 = UTC of log t=0)."""
    out = []
    summary = next((r for r in reversed(rows) if r.get("summary")), {})
    first_seen = {}
    for r in rows:
        if r.get("cmd") == "track steps":
            # the text can start mid-line, so count the TS lines rather than trust the header
            ns = [int(x) for x in re.findall(r"TS (\d+) ", r.get("text", ""))]
            for n in range(1, max(ns, default=0) + 1):
                first_seen.setdefault(n, r["t"])
    t_end = t0 + max((r["t"] for r in rows if not r.get("summary") and "t" in r), default=0.0)
    for s in summary.get("steps", []):
        t = first_seen.get(s["n"])
        tt = None if t is None else t0 + t
        if s["lean_in"] >= LEAN_IN_DEG or s["lean_out"] >= LEAN_OUT_DEG:
            out.append(dict(kind="ts_lean", t_utc=tt, detail=f"step {s['n']} {s['stance']} in {s['lean_in']} out {s['lean_out']}"))
        if s.get("ok") is False:
            out.append(dict(kind="firmware_abort", t_utc=tt, detail=f"step {s['n']} {s['stance']} ABORT"))
        if s.get("gate_ms"):
            pass                                  # reported by the HOLDING line below (it has the time)
    held = False
    for r in rows:
        tx = r.get("text", "") or ""
        if "HOLDING" in tx and not held:
            held = True
            out.append(dict(kind="gate_hold", t_utc=t0 + r["t"], detail=next(l for l in tx.splitlines() if "HOLDING" in l).strip()[:100]))
        if not r.get("summary") and r.get("cmd", "").startswith("track stop"):
            out.append(dict(kind="stop", t_utc=t0 + r["t"], detail=tx.strip()[:100]))
        for line in tx.splitlines():
            if STOP_RE.search(line) and "accel-reject" not in line and "learning" not in line:
                out.append(dict(kind="abort_or_fall", t_utc=t0 + r["t"], detail=line.strip()[:100]))
    oc = summary.get("outcome") or {}
    if summary.get("gate_timeout"):
        out.append(dict(kind="gate_timeout", t_utc=t_end, detail=str(summary["gate_timeout"])))
    if oc.get("stop_rule"):
        out.append(dict(kind="stop_rule", t_utc=t_end, detail="the host's stop rule ended the track (lean past --stop-lean / --stop-out, or an ABORT)"))
    if oc.get("fell"):
        out.append(dict(kind="fall_latch", t_utc=t_end, detail="the host read the fall latch after the run"))
    planned = (summary.get("args") or {}).get("steps")
    done = max([s["n"] for s in summary.get("steps", [])], default=0)
    if planned and done < planned and not any(o["kind"] in ("gate_timeout", "stop_rule", "firmware_abort", "abort_or_fall") for o in out):
        # the host log has no fall line: a run that fell has no TS line for the step it fell in (2026-11-16 14:01: 16 of 17)
        out.append(dict(kind="short_run", t_utc=t_end, detail=f"{done} TS lines for {planned} steps decided: the track ended early (fall, link or firmware stop)"))
    return out


def find_videos(dirs, t0, t1):
    """[(coverage_s, mp4)] of every mp4 (+ .pts) in dirs overlapping [t0, t1], best first."""
    out = []
    for d in dirs:
        for pts in glob.glob(os.path.join(d, "*.pts")):
            ts = C.read_pts(pts)
            if not ts:
                continue
            cover = min(ts[-1], t1) - max(ts[0], t0)
            if cover > 0 and os.path.exists(pts[:-4] + ".mp4"):
                out.append((cover, pts[:-4] + ".mp4"))
    return sorted(out, reverse=True)


def pick_videos(dirs, t0, t1):
    """(primary, context): the eMeet (measurement camera; its files are named emeet*) is primary,
    the Sony context. Without an eMeet file the best-covering video is primary."""
    vids = find_videos(dirs, t0, t1)
    em = [v for v in vids if os.path.basename(v[1]).startswith("emeet")]
    so = [v for v in vids if os.path.basename(v[1]).startswith("sony")]
    if em:
        return em[0][1], (so[0][1] if so else None)
    if so:
        return so[0][1], None
    return (vids[0][1], None) if vids else (None, None)


def _frame_at(cap_cache, path, ts, t):
    i = int(np.argmin(np.abs(np.asarray(ts) - t)))
    cap = cap_cache.setdefault(path, cv2.VideoCapture(path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ok, f = cap.read()
    return f if ok else None


def contact_sheet(video, moments, out_jpg, cols=4, tw=320, captions=None):
    """moments: [(label, t_utc)] -> JPEG grid of the frames nearest those times. captions: {label: text} drawn as a
    second line at the top of that tile (the VLM's answer)."""
    pts = C.read_pts(os.path.splitext(video)[0] + ".pts")
    caches, tiles = {}, []
    for label, t in moments:
        f = _frame_at(caches, video, pts, t)
        if f is None:
            continue
        f = cv2.resize(f, (tw, int(f.shape[0] * tw / f.shape[1])))
        cv2.rectangle(f, (0, f.shape[0] - 18), (tw, f.shape[0]), (0, 0, 0), -1)
        cv2.putText(f, label[:48], (3, f.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        if captions and captions.get(label):
            cv2.rectangle(f, (0, 0), (tw, 16), (0, 0, 0), -1)
            cv2.putText(f, captions[label][:52], (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)
        tiles.append(f)
    if not tiles:
        return None
    h, w = tiles[0].shape[:2]
    rows = -(-len(tiles) // cols)
    sheet = np.zeros((rows * h, cols * w, 3), np.uint8)
    for k, f in enumerate(tiles):
        sheet[(k // cols) * h:(k // cols + 1) * h, (k % cols) * w:(k % cols + 1) * w] = f
    cv2.imwrite(out_jpg, sheet, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return out_jpg


def _hms(t):
    return None if t is None else dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


STILL_S, STILL_MM = 1.0, 15.0    # a stationary window: >= 1 s with the torso tag within 15 mm (x and y spread)


def still_position(good, last, win_s=STILL_S, spread_mm=STILL_MM):
    """-> ([x, y], how) of the first (last=False) / last (last=True) window of tracker rows >= win_s long whose x and
    y each spread (max - min) <= spread_mm: the robot at rest. The torso tag swings about +-80 mm sideways with every
    step, so a single mid-step sample is up to ~90 mm off the at-rest position. No such window (it never stood still
    that long in view): the median of the first / last 5 % of the rows, and `how` says so."""
    seq = list(reversed(good)) if last else list(good)
    best = None
    for i in range(len(seq)):
        w = [seq[i]]
        for r in seq[i + 1:]:
            w.append(r)
            xs, ys = [q["x_mm"] for q in w], [q["y_mm"] for q in w]
            if max(xs) - min(xs) > spread_mm or max(ys) - min(ys) > spread_mm:
                w.pop()
                break
        if len(w) >= 2 and abs(w[-1]["t"] - w[0]["t"]) >= win_s:
            best = w
            break
    if best is not None:
        how = dict(method="median over a stationary window", window_s=round(abs(best[-1]["t"] - best[0]["t"]), 2),
                   frames=len(best), spread_mm_max=spread_mm,
                   from_utc=_hms(min(q["t"] for q in best)), to_utc=_hms(max(q["t"] for q in best)))
    else:
        n = max(1, len(good) // 20)
        best = seq[:n]
        how = dict(method=f"median of the {'last' if last else 'first'} 5 % of the frames: NO stationary window of "
                          f"{win_s:g} s found, may be mid-step (up to ~90 mm off)", frames=len(best))
    pos = [round(float(np.median([r["x_mm"] for r in best])), 1), round(float(np.median([r["y_mm"] for r in best])), 1)]
    return pos, how


def tag_flags(rows, imu):
    """Flags from tracker rows (+ IMU series for the tilt cross-check)."""
    good = [r for r in rows if r.get("stale") is False]
    if not good:
        return dict(edge_margin_min_mm=None, edge_margin_min_at=None, position_start=None, position_end=None,
                    position_method=None, position_track_extent=None, heading_start=None, heading_end=None, tilt_video_vs_imu=None, tags_visible_frames=0)
    k = min(good, key=lambda r: r["margin_min_mm"])
    n = max(1, len(good) // 20)
    p_start, h_start = still_position(good, False)
    p_end, h_end = still_position(good, True)
    xs, ys = [r["x_mm"] for r in good], [r["y_mm"] for r in good]
    extent = dict(x_mm=[round(min(xs), 1), round(max(xs), 1)], y_mm=[round(min(ys), 1), round(max(ys), 1)],
                  note="all frames with tags, including the +-80 mm per-step sway")
    hd = lambda g: round(float(np.median([r["heading_deg"] for r in g])), 1)
    out = dict(edge_margin_min_mm=round(k["margin_min_mm"], 1), edge_margin_min_at=dict(utc=_hms(k["t"]), edge=k["margin_min_edge"],
               x_mm=round(k["x_mm"], 1), y_mm=round(k["y_mm"], 1)),
               position_start=p_start, position_end=p_end, position_method=dict(start=h_start, end=h_end),
               position_track_extent=extent,
               heading_start=hd(good[:n]), heading_end=hd(good[-n:]), tags_visible_frames=len(good), tilt_video_vs_imu=None)
    if imu is not None and len(imu["t"]):
        sure = [r for r in good if not r.get("ambiguous")]
        if len(sure) >= 5:
            t = np.array([r["t"] for r in sure])
            res = {}
            for key, vk in (("roll", "roll_deg"), ("pitch", "pitch_deg")):
                v = np.array([r[vk] for r in sure])
                i = np.interp(t, imu["t"], imu[f"{key}_deg"])
                # the firmware's sign convention is unconfirmed: report the better of the two
                res[key] = round(float(min(np.max(np.abs(v - i)), np.max(np.abs(v + i)))), 1)
                res[f"{key}_sign"] = +1 if np.max(np.abs(v - i)) <= np.max(np.abs(v + i)) else -1
            res.update(source="flight record IMU roll/pitch", frames=len(sure), excluded_ambiguous=len(good) - len(sure))
            out["tilt_video_vs_imu"] = res
    return out


VLM_MAX_FRAMES = 8


def vlm_moments(moments, max_frames=VLM_MAX_FRAMES, min_gap_s=0.5):
    """The key frames the VLM is asked about: start, end, then each anomaly's own moment (+0 s), then the +-1 s
    frames, at most max_frames, none closer than min_gap_s to one already chosen. Keeps the moments' labels."""
    first = [m for m in moments if m[0] in ("start", "end")]
    zero = [m for m in moments if " +0s " in m[0]]
    rest = [m for m in moments if m not in first and m not in zero]
    out = []
    for m in first + zero + rest:
        if len(out) >= max_frames:
            break
        if all(abs(m[1] - o[1]) >= min_gap_s for o in out):
            out.append(m)
    return sorted(out, key=lambda m: m[1])


def measured_at(t, imu=None, trows=None, tol_s=0.5):
    """The measured values at UTC t, for showing beside the VLM's answer: IMU tilt (max |roll|, |pitch|) from the
    flight record, and the nearest tag-tracker row within tol_s (tag roll/pitch, position, closest edge)."""
    out = {}
    if imu is not None and len(imu["t"]) and imu["t"][0] - tol_s <= t <= imu["t"][-1] + tol_s:
        r, p = (float(np.interp(t, imu["t"], imu[k])) for k in ("roll_deg", "pitch_deg"))
        out["imu_tilt_deg"] = round(max(abs(r), abs(p)), 1)
    good = [r for r in (trows or []) if r.get("stale") is False and abs(r["t"] - t) <= tol_s]
    if good:
        r = min(good, key=lambda q: abs(q["t"] - t))
        out.update(tag_roll_deg=round(r["roll_deg"], 1), tag_pitch_deg=round(r["pitch_deg"], 1),
                   x_mm=round(r["x_mm"], 1), y_mm=round(r["y_mm"], 1), edge_margin_mm=round(r["margin_min_mm"], 1))
    return out


def vlm_caption(ans):
    """Short contact-sheet caption: what the model says, flagged items in capitals."""
    if not ans:
        return "vlm: no answer"
    bits = ["PERSON/ANIMAL" if ans.get("person_or_animal") else "no person",
            "upright" if ans.get("robot_upright") else ("NOT UPRIGHT" if ans.get("robot_upright") is False else "upright ?"),
            "OBJECT" if ans.get("object_on_board") else "no object"]
    return "vlm: " + ", ".join(bits)


def vlm_pass(video, moments, out_dir, name, model, host=V.DEFAULT_HOST, imu=None, trows=None,
             max_frames=VLM_MAX_FRAMES, num_thread=12, ask=V.ask):
    """Ask the VLM about the key frames of one run. -> dict for the report (answers next to measured values).
    Never raises for the model: a failed or unparseable answer is reported as such."""
    pts = C.read_pts(os.path.splitext(video)[0] + ".pts")
    fdir = os.path.join(out_dir, f"{name}_vlm")
    os.makedirs(fdir, exist_ok=True)
    caches, frames = {}, []
    t_all = dt.datetime.now().timestamp()
    for k, (label, t) in enumerate(vlm_moments(moments, max_frames)):
        f = _frame_at(caches, video, pts, t)
        if f is None:
            continue
        h, w = f.shape[:2]
        if w > 1280:
            f = cv2.resize(f, (1280, round(h * 1280 / w)), interpolation=cv2.INTER_AREA)
        jpg = os.path.join(fdir, f"{k:02d}.jpg")
        cv2.imwrite(jpg, f, [cv2.IMWRITE_JPEG_QUALITY, 88])
        r = ask(jpg, model, host, num_thread=num_thread)
        frames.append(dict(moment=label, t_utc=_hms(t), frame=jpg, answer=r.get("answer"), error=r.get("error"),
                           seconds=r.get("total_s"), measured=measured_at(t, imu, trows)))
    for c in caches.values():
        c.release()
    return dict(model=model, host=host, video=video, frames=frames,
                seconds_total=round(dt.datetime.now().timestamp() - t_all, 1),
                reliability="second opinion only: a hint to look at the frame; never a measurement, never a gate "
                            "(bake-off: tools/vision/README.md, Layer E)")


def run_report(track, video_dirs=(), frec_dir="hw_sessions", out_dir=None, cfg_path=None, use_video=True, wait_s=90.0,
               vlm_model=None, vlm_host=V.DEFAULT_HOST, vlm_max_frames=VLM_MAX_FRAMES):
    rows = [json.loads(l) for l in open(track)]
    name = os.path.splitext(os.path.basename(track))[0]
    out_dir = out_dir or os.path.dirname(os.path.abspath(track))
    os.makedirs(out_dir, exist_ok=True)
    imrep = IM.check_run(track, frec_dir=frec_dir, data=True, wait_s=wait_s)
    t0, t1 = imrep["window_utc"]
    imu = imrep.pop("_imu", None)
    imrep.pop("_events", None)
    anomalies = log_anomalies(rows, t0)
    for e in imrep["excursions"]:
        anomalies.append(dict(kind="implausible_recovery" if e["implausible"] else "tip_over", t_utc=e["t_peak"],
                              detail=f"{e['peak_deg']:.0f} deg, fall latch {e['fall_latched']}"))
    for ft in imrep.get("falls", []):
        anomalies.append(dict(kind="fall_latch", t_utc=ft, detail="flight record: fall latch (EV_FALL / TF_FALLEN)"))
    anomalies = [a for a in anomalies if a["t_utc"] is not None]
    anomalies.sort(key=lambda a: a["t_utc"])
    ending = sorted({a["kind"] for a in anomalies if a["kind"] in ENDING_KINDS})
    rep = dict(run=name, window_utc=[_hms(t0), _hms(t1)], flight_record=imrep["source"], ending=ending or "clean",
               implausible_recovery=imrep["implausible_recovery"], max_tip_deg=imrep.get("max_tip_deg"),
               anomalies=[dict(a, t_utc=_hms(a["t_utc"]), t_epoch=a["t_utc"]) for a in anomalies], notes=[])
    if imrep.get("note"):
        rep["notes"].append(imrep["note"])
    if imrep.get("partial"):
        rep["flight_record_partial"] = True
    video, context = pick_videos(video_dirs, t0 - 1, t1 + 1) if use_video else (None, None)
    rep["video"], rep["context_video"] = video, context
    rep.update(foreign_motion_hints=None, tags=None)
    rep.update(edge_margin_min_mm=None, edge_margin_min_at=None, position_start=None, position_end=None,
               position_method=None, position_track_extent=None, heading_start=None, heading_end=None, tilt_video_vs_imu=None)
    if video:
        clip_mp4 = os.path.join(out_dir, f"{name}_clip.mp4")
        # cut with frame accuracy from this one file
        pts = C.read_pts(os.path.splitext(video)[0] + ".pts")
        idx = [i for i, t in enumerate(pts) if t0 - 1 <= t <= t1 + 1]
        if idx:
            clip_pts = [pts[i] for i in idx]
            os.system(f"ffmpeg -loglevel error -y -i '{video}' -vf 'select=between(n\\,{idx[0]}\\,{idx[-1]}),setpts=N/25/TB' "
                      f"-an -c:v libx264 -preset veryfast -pix_fmt yuv420p -r 25 '{clip_mp4}'")
            with open(os.path.splitext(clip_mp4)[0] + ".pts", "w") as f:
                f.write("# timecode format v2\n" + "".join(f"{t * 1000:.3f}\n" for t in clip_pts))
            rep["clip"] = clip_mp4
            fm = foreign.foreign_motion(clip_mp4)
            fm.pop("series", None)
            fm["reliability"] = "hint only: also fires on the person in the chair, socks, the dog (README)"
            fm["events"] = [dict(e, t_start=_hms(e["t_start"]), t_end=_hms(e["t_end"]), t_peak=_hms(e["t_peak"])) for e in fm["events"]]
            rep["foreign_motion_hints"] = fm
            cfg = G.load_config(cfg_path)
            trows, cal = T.track_video(clip_mp4, cfg, stride=2)
            if cal is None:
                rep["tags"] = "no frame shows two board tags: tag-based fields are null"
            else:
                rep["tags"] = dict(calibration=cal, frames=len(trows))
                rep.update(tag_flags(trows, imu))
                with open(os.path.join(out_dir, f"{name}_vision_track.jsonl"), "w") as f:
                    for r in trows:
                        f.write(json.dumps(r) + "\n")
            moments = [("start", t0), ("end", t1)]
            for a in anomalies:
                for dtm in (-1, 0, 1):
                    moments.append((f"{a['kind']} {dtm:+d}s {_hms(a['t_utc'])[:8]}", a["t_utc"] + dtm))
            captions = None
            if vlm_model:
                vp = vlm_pass(clip_mp4, moments, out_dir, name, vlm_model, vlm_host, imu=imu, trows=trows,
                              max_frames=vlm_max_frames)
                rep["vlm_second_opinion"] = vp
                captions = {f["moment"]: vlm_caption(f["answer"]) for f in vp["frames"]}
            rep["contact_sheet"] = contact_sheet(clip_mp4, moments, os.path.join(out_dir, f"{name}_vision.jpg"),
                                                 captions=captions)
            if context:
                rep["context_contact_sheet"] = contact_sheet(context, moments, os.path.join(out_dir, f"{name}_vision_context.jpg"))
    else:
        rep["notes"].append("no video covers this run")
    with open(os.path.join(out_dir, f"{name}_vision.json"), "w") as f:
        json.dump(rep, f, indent=1, default=float)
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track")
    ap.add_argument("--video-dir", action="append", default=[])
    ap.add_argument("--frec-dir", default="hw_sessions")
    ap.add_argument("--out-dir")
    ap.add_argument("--config")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--wait-s", type=float, default=90.0, help="wait for the flight record to cover the run's end")
    ap.add_argument("--vlm-model", help="optional second opinion on the key frames by this ollama vision model "
                                        "(default off; see README, Layer E)")
    ap.add_argument("--vlm-host", default=V.DEFAULT_HOST, help="ollama server (default: the CPU-only one, %(default)s)")
    ap.add_argument("--vlm-max-frames", type=int, default=VLM_MAX_FRAMES, help="key frames asked about (%(default)s)")
    a = ap.parse_args(argv)
    rep = run_report(a.track, a.video_dir, a.frec_dir, a.out_dir, a.config, not a.no_video, a.wait_s,
                     a.vlm_model, a.vlm_host, a.vlm_max_frames)
    # foreign_motion_hints stays in the JSON file only: it fires on the robot itself
    print(json.dumps({k: v for k, v in rep.items() if k not in ("anomalies", "foreign_motion_hints", "vlm_second_opinion")},
                     indent=1, default=float))
    print(f"{len(rep['anomalies'])} anomalies")
    if rep.get("vlm_second_opinion"):
        print(vlm_table(rep["vlm_second_opinion"]))
    return 0


def vlm_table(vp):
    """Plain-text table: per key frame, the measured values and the VLM's answer side by side."""
    lines = [f"second opinion ({vp['model']}, {vp['seconds_total']:.0f} s; a hint, not a measurement):",
             f"  {'moment':34s} {'IMU tilt':>8s} {'tag r/p':>11s} {'edge mm':>7s} | vlm"]
    for f in vp["frames"]:
        m = f["measured"]
        rp = f"{m['tag_roll_deg']:.0f}/{m['tag_pitch_deg']:.0f}" if "tag_roll_deg" in m else "-"
        lines.append(f"  {f['moment'][:34]:34s} {m.get('imu_tilt_deg', '-')!s:>8s} {rp:>11s} {m.get('edge_margin_mm', '-')!s:>7s}"
                     f" | {vlm_caption(f['answer'])[5:] if f['answer'] else 'no answer: ' + str(f['error'])}")
    return "\n".join(lines)


if __name__ == "__main__":
    sys.exit(main())
