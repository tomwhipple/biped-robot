#!/usr/bin/env python3
"""One-run vision report: <run>_vision.json + <run>_vision.jpg (key-frame contact sheet).

    tools/vision/run_report.py track_20261116T093534.jsonl --video-dir hw_sessions/2026-11-16
        [--frec-dir hw_sessions] [--out-dir DIR] [--config vision.json] [--no-video]

Takes the roll-balance track log of one run, finds its UTC window (the flight record's
`cal show` command stamps t=0), cuts the video for that window from --video-dir (camd
segments or hand-recorded mp4+.pts), and writes:

  anomalies             every TS step with inward lean >= 8 or outward >= 6 deg, gate hold,
                        track stop, abort/fall line, flight fall event, implausible recovery
  implausible_recovery  TELEMETRY ONLY (flight record IMU): tip > 20 deg and back < 10 deg within
                        6 s without a fall latch = someone helped. Works without tags.
  foreign_motion        image-only hint (tools/vision/foreign.py); unreliable, see README
  edge_margin_min_mm, edge_margin_min_at, position_start/end, heading_start/end
                        from the tags; null when no tags are visible
  tilt_video_vs_imu     max |tag roll/pitch - IMU roll/pitch| (needs tags); null without
  contact sheet         start, end and +-1 s around every anomaly

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

LEAN_IN_DEG, LEAN_OUT_DEG = 8.0, 6.0
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
    for s in summary.get("steps", []):
        t = first_seen.get(s["n"])
        tt = None if t is None else t0 + t
        if s["lean_in"] >= LEAN_IN_DEG or s["lean_out"] >= LEAN_OUT_DEG:
            out.append(dict(kind="ts_lean", t_utc=tt, detail=f"step {s['n']} {s['stance']} in {s['lean_in']} out {s['lean_out']}"))
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
    if summary.get("gate_timeout"):
        out.append(dict(kind="gate_timeout", t_utc=None, detail=str(summary["gate_timeout"])))
    return out


def find_video(dirs, t0, t1):
    """The mp4 (+ .pts) in any of dirs covering most of [t0, t1] -> path or None."""
    best = None
    for d in dirs:
        for pts in glob.glob(os.path.join(d, "*.pts")):
            ts = C.read_pts(pts)
            if not ts:
                continue
            cover = min(ts[-1], t1) - max(ts[0], t0)
            if cover > 0 and os.path.exists(pts[:-4] + ".mp4") and (best is None or cover > best[0]):
                best = (cover, pts[:-4] + ".mp4")
    return best[1] if best and best[0] > 0.5 * (t1 - t0) else (best[1] if best else None)


def _frame_at(cap_cache, path, ts, t):
    i = int(np.argmin(np.abs(np.asarray(ts) - t)))
    cap = cap_cache.setdefault(path, cv2.VideoCapture(path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ok, f = cap.read()
    return f if ok else None


def contact_sheet(video, moments, out_jpg, cols=4, tw=320):
    """moments: [(label, t_utc)] -> JPEG grid of the frames nearest those times."""
    pts = C.read_pts(os.path.splitext(video)[0] + ".pts")
    caches, tiles = {}, []
    for label, t in moments:
        f = _frame_at(caches, video, pts, t)
        if f is None:
            continue
        f = cv2.resize(f, (tw, int(f.shape[0] * tw / f.shape[1])))
        cv2.rectangle(f, (0, f.shape[0] - 18), (tw, f.shape[0]), (0, 0, 0), -1)
        cv2.putText(f, label[:48], (3, f.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
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


def tag_flags(rows, imu):
    """Flags from tracker rows (+ IMU series for the tilt cross-check)."""
    good = [r for r in rows if r.get("stale") is False]
    if not good:
        return dict(edge_margin_min_mm=None, edge_margin_min_at=None, position_start=None, position_end=None,
                    heading_start=None, heading_end=None, tilt_video_vs_imu=None, tags_visible_frames=0)
    k = min(good, key=lambda r: r["margin_min_mm"])
    n = max(1, len(good) // 20)
    pos = lambda g: [round(float(np.median([r["x_mm"] for r in g])), 1), round(float(np.median([r["y_mm"] for r in g])), 1)]
    hd = lambda g: round(float(np.median([r["heading_deg"] for r in g])), 1)
    out = dict(edge_margin_min_mm=round(k["margin_min_mm"], 1), edge_margin_min_at=dict(utc=_hms(k["t"]), edge=k["margin_min_edge"],
               x_mm=round(k["x_mm"], 1), y_mm=round(k["y_mm"], 1)),
               position_start=pos(good[:n]), position_end=pos(good[-n:]),
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


def run_report(track, video_dirs=(), frec_dir="hw_sessions", out_dir=None, cfg_path=None, use_video=True):
    rows = [json.loads(l) for l in open(track)]
    name = os.path.splitext(os.path.basename(track))[0]
    out_dir = out_dir or os.path.dirname(os.path.abspath(track))
    os.makedirs(out_dir, exist_ok=True)
    imrep = IM.check_run(track, frec_dir=frec_dir, data=True)
    t0, t1 = imrep["window_utc"]
    imu = imrep.pop("_imu", None)
    imrep.pop("_events", None)
    anomalies = log_anomalies(rows, t0)
    for e in imrep["excursions"]:
        anomalies.append(dict(kind="implausible_recovery" if e["implausible"] else "tip_over", t_utc=e["t_peak"],
                              detail=f"{e['peak_deg']:.0f} deg, fall latch {e['fall_latched']}"))
    anomalies = [a for a in anomalies if a["t_utc"] is not None]
    anomalies.sort(key=lambda a: a["t_utc"])
    rep = dict(run=name, window_utc=[_hms(t0), _hms(t1)], flight_record=imrep["source"],
               implausible_recovery=imrep["implausible_recovery"], max_tip_deg=imrep.get("max_tip_deg"),
               anomalies=[dict(a, t_utc=_hms(a["t_utc"]), t_epoch=a["t_utc"]) for a in anomalies], notes=[])
    if imrep["source"] is None:
        rep["notes"].append(imrep.get("note"))
    video = find_video(video_dirs, t0 - 1, t1 + 1) if use_video else None
    rep["video"] = video
    rep.update(foreign_motion=None, tags=None)
    rep.update(edge_margin_min_mm=None, edge_margin_min_at=None, position_start=None, position_end=None,
               heading_start=None, heading_end=None, tilt_video_vs_imu=None)
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
            rep["foreign_motion"] = fm
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
            rep["contact_sheet"] = contact_sheet(clip_mp4, moments, os.path.join(out_dir, f"{name}_vision.jpg"))
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
    a = ap.parse_args(argv)
    rep = run_report(a.track, a.video_dir, a.frec_dir, a.out_dir, a.config, not a.no_video)
    print(json.dumps({k: v for k, v in rep.items() if k not in ("anomalies",)}, indent=1, default=float))
    print(f"{len(rep['anomalies'])} anomalies")
    return 0


if __name__ == "__main__":
    sys.exit(main())
