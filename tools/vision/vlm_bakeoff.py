#!/usr/bin/env python3
"""Layer E bake-off (#316): a labelled frame set from recorded footage, and local vision-language models scored on it.

    # frames for a look (exploring footage, choosing what to label): JPEGs named by UTC time
    tools/vision/vlm_bakeoff.py grab VIDEO --at 2026-11-16T16:48:33Z [--at ...] -o /tmp/look
    # rebuild the labelled set's frames from the footage (hw_sessions/ is not in git)
    tools/vision/vlm_bakeoff.py extract [--manifest M] [--root .] [--out hw_sessions/vlm_set]
    # ask a model about every frame; raw answers + timings appended to a JSONL (resumes: done frames are skipped)
    tools/vision/vlm_bakeoff.py run --model qwen3-vl:8b-instruct [--host http://127.0.0.1:11500] [--num-thread 12]
    # the seeded tuning / held-out split (--split tune|heldout on run and score restricts to one half)
    tools/vision/vlm_bakeoff.py split
    # precision / recall per label and model, seconds per frame
    tools/vision/vlm_bakeoff.py score [--answers-dir DIR] [--md]

The manifest (`vlm_set.json`, beside this file) lists each frame: the video (repo-relative), the UTC time,
the frame index that time resolves to, the three labels and a one-line note. Labels are the answers to the
questions in `vlm.QUESTIONS`; what counts is written down in `vlm.LABEL_RULES`.

A video's frame times come from its `.pts` (epoch ms per frame) or, for the `_top` recordings that only
have a `.start` stamp, start + index / fps. Frames are decoded sequentially (exact, no seek), resized
to <= 1280 px wide and written as JPEG.
"""
import argparse
import datetime as dt
import json
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import clip as C  # noqa: E402
import vlm  # noqa: E402

MANIFEST = os.path.join(HERE, "vlm_set.json")
OUT_DIR = os.path.join(ROOT, "hw_sessions", "vlm_set")
MAX_W = 1280
LABELS = tuple(vlm.QUESTIONS)


def frame_times(video):
    """-> list of UTC epoch seconds, one per frame (from .pts, or .start + index / fps)."""
    base = os.path.splitext(video)[0]
    if os.path.exists(base + ".pts"):
        return C.read_pts(base + ".pts")
    if os.path.exists(base + ".start"):
        t0 = C.parse_time(open(base + ".start").read().strip())
        cap = cv2.VideoCapture(video)
        fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        return [t0 + i / fps for i in range(n)]
    raise FileNotFoundError(f"{video}: no .pts or .start beside it")


def index_at(times, t):
    return min(range(len(times)), key=lambda i: abs(times[i] - t))


def read_frames(video, indices):
    """{index: BGR frame} by sequential decode (exact on the passthrough-timed mp4s)."""
    want = set(indices)
    out, i = {}, 0
    cap = cv2.VideoCapture(video)
    while want - out.keys():
        ok, f = cap.read()
        if not ok:
            break
        if i in want:
            out[i] = f
        i += 1
    cap.release()
    return out


def to_jpeg(frame, path, max_w=MAX_W):
    h, w = frame.shape[:2]
    if w > max_w:
        frame = cv2.resize(frame, (max_w, round(h * max_w / w)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
    return path


def utc(t):
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def load_manifest(path=MANIFEST):
    with open(path) as f:
        return json.load(f)


SPLIT_SEED = 316


def stratum(item):
    """The label stratum a frame is split on: person/animal frames, then not-upright ones, then the rest with an
    object label, then normal frames (nothing to flag)."""
    lb = item["labels"]
    if lb["person_or_animal"]:
        return "person_or_animal"
    if lb["robot_upright"] is False:
        return "not_upright"
    if lb["object_on_board"]:
        return "object"
    return "normal"


def make_split(manifest, seed=SPLIT_SEED):
    """Fixed, seeded split of the manifest frames -> {"tune": [ids], "heldout": [ids]}, stratified by label
    stratum (see `stratum`): each stratum is shuffled with random.Random(seed) after sorting by id, and dealt
    alternately, the larger half to tune. Deterministic: the same manifest and seed give the same split."""
    import random
    rng = random.Random(seed)
    strata = {}
    for it in sorted(manifest["frames"], key=lambda i: i["id"]):
        strata.setdefault(stratum(it), []).append(it["id"])
    out = {"tune": [], "heldout": []}
    flip = 0
    for name in sorted(strata):
        ids = strata[name]
        rng.shuffle(ids)
        for i, fid in enumerate(ids):
            out["tune" if (i + flip) % 2 == 0 else "heldout"].append(fid)
        flip += len(ids) % 2      # an odd stratum gives its spare frame alternately to each half
    return {k: sorted(v) for k, v in out.items()}


def apply_split(manifest, which, seed=SPLIT_SEED):
    """The manifest restricted to one half ("tune" or "heldout")."""
    ids = set(make_split(manifest, seed)[which])
    return dict(manifest, frames=[f for f in manifest["frames"] if f["id"] in ids])


def frame_path(item, out_dir=OUT_DIR):
    return os.path.join(out_dir, item["id"] + ".jpg")


def extract(manifest, root=ROOT, out_dir=OUT_DIR, check=True):
    """Write every manifest frame to out_dir/<id>.jpg. With check, the frame index the UTC time resolves
    to must equal the manifest's (the footage has not changed). -> list of paths."""
    os.makedirs(out_dir, exist_ok=True)
    by_video = {}
    for it in manifest["frames"]:
        by_video.setdefault(it["video"], []).append(it)
    paths = []
    for video, items in sorted(by_video.items()):
        vp = os.path.join(root, video)
        times = frame_times(vp)
        for it in items:
            k = index_at(times, C.parse_time(it["t_utc"]))
            if check and it.get("frame") is not None and k != it["frame"]:
                raise ValueError(f"{it['id']}: {it['t_utc']} resolves to frame {k}, manifest says {it['frame']}")
            it["_k"] = k
        frames = read_frames(vp, [it["_k"] for it in items])
        for it in items:
            f = frames.get(it.pop("_k"))
            if f is None:
                raise ValueError(f"{it['id']}: frame not decodable in {video}")
            paths.append(to_jpeg(f, frame_path(it, out_dir)))
    return paths


def grab(video, ats, out_dir):
    times = frame_times(video)
    idx = {index_at(times, C.parse_time(a)): a for a in ats}
    frames = read_frames(video, list(idx))
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(video))[0]
    for k, f in sorted(frames.items()):
        p = to_jpeg(f, os.path.join(out_dir, f"{stem}_{utc(times[k])[11:23].replace(':', '')}_f{k}.jpg"))
        print(p, utc(times[k]), k)


# ---------------------------------------------------------------- running a model over the set

def answers_path(answers_dir, model, prompt=vlm.DEFAULT_PROMPT):
    return os.path.join(answers_dir, model.replace(":", "_").replace("/", "_") + f"__{prompt}.jsonl")


def run_model(manifest, model, host, answers_dir, frames_dir=OUT_DIR, num_thread=12, timeout_s=600.0, log=print,
              prompt=vlm.DEFAULT_PROMPT):
    """Ask `model` about every frame; one JSON line per frame (raw text, parsed answer, timings). Frames already in
    the file are skipped, so an interrupted run resumes."""
    os.makedirs(answers_dir, exist_ok=True)
    path = answers_path(answers_dir, model, prompt)
    done = set()
    if os.path.exists(path):
        done = {json.loads(l)["id"] for l in open(path) if l.strip()}
    with open(path, "a") as out:
        for it in manifest["frames"]:
            if it["id"] in done:
                continue
            w0 = time.time()
            r = vlm.ask(frame_path(it, frames_dir), model, host, num_thread=num_thread, timeout_s=timeout_s, prompt=prompt)
            r.update(id=it["id"], model=model, prompt=prompt, host=host, wall_s=round(time.time() - w0, 2),
                     num_thread=num_thread)
            out.write(json.dumps(r) + "\n")
            out.flush()
            a = r.get("answer") or {}
            log(f"{model} {it['id']:28s} {r['wall_s']:6.1f}s  " + " ".join(f"{k}={a.get(k)}" for k in LABELS)
                + ("" if r.get("error") is None else f"  ERROR {r['error']}"))


def score(manifest, answers):
    """answers: {id: parsed answer dict or None (asked, no usable answer)}; frames not in answers were not asked and
    are left out. -> per label: tp fp fn tn, precision, recall, accuracy, unanswered.
    'Positive' is the event the second opinion is for: person/animal yes, upright NO (= not upright), object yes."""
    pos = vlm.POSITIVE
    res = {}
    for k in LABELS:
        c = dict(tp=0, fp=0, fn=0, tn=0, unanswered=0)
        for it in manifest["frames"]:
            truth = it["labels"][k]
            if truth is None or it["id"] not in answers:
                continue
            a = (answers.get(it["id"]) or {}).get(k)
            if a is None:
                c["unanswered"] += 1
                a = not pos[k]     # no answer counts as "nothing to report": a miss on a positive frame
            t, p = truth == pos[k], a == pos[k]
            c["tp" if t and p else "fp" if p else "fn" if t else "tn"] += 1
        n = c["tp"] + c["fp"] + c["fn"] + c["tn"]
        c["precision"] = None if c["tp"] + c["fp"] == 0 else round(c["tp"] / (c["tp"] + c["fp"]), 2)
        c["recall"] = None if c["tp"] + c["fn"] == 0 else round(c["tp"] / (c["tp"] + c["fn"]), 2)
        c["accuracy"] = None if n == 0 else round((c["tp"] + c["tn"]) / n, 2)
        res[k] = c
    return res


def load_answers(path):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    return {r["id"]: r for r in rows}


def score_all(manifest, answers_dir):
    out = {}
    for fn in sorted(os.listdir(answers_dir)):
        if not fn.endswith(".jsonl"):
            continue
        rows = load_answers(os.path.join(answers_dir, fn))
        ids = {it["id"] for it in manifest["frames"]}
        rows = {k: v for k, v in rows.items() if k in ids}
        if not rows:
            continue
        r0 = next(iter(rows.values()))
        model = f"{r0['model']} ({r0.get('prompt', 'answer-first')})"
        sec = sorted(r["total_s"] if r.get("total_s") else r["wall_s"] for r in rows.values())   # server time, no queueing
        out[model] = dict(frames=len(rows), of=len(ids), errors=sum(1 for r in rows.values() if r.get("error")),
                          s_per_frame_median=sec[len(sec) // 2], s_per_frame_max=sec[-1],
                          labels=score(manifest, {k: v.get("answer") for k, v in rows.items()}))
    return out


def markdown(scores):
    head = "| model | " + " | ".join(f"{vlm.SHORT[k]} P / R" for k in LABELS) + " | s/frame (median, max) | frames |"
    lines = [head, "|" + "---|" * (len(LABELS) + 3)]
    f = lambda v: "–" if v is None else f"{v:.2f}"
    for m, s in scores.items():
        cells = [f"{f(s['labels'][k]['precision'])} / {f(s['labels'][k]['recall'])}" for k in LABELS]
        lines.append(f"| {m} | " + " | ".join(cells) + f" | {s['s_per_frame_median']:.0f}, {s['s_per_frame_max']:.0f} | "
                     f"{s['frames']}/{s['of']}{'' if not s['errors'] else ', %d errors' % s['errors']} |")
    return "\n".join(lines)


def per_frame(manifest, answers_dir):
    """Markdown: each frame's labels and every model's answers (P person/animal, U upright, O object; lower case =
    no, - = not scored / no answer, * = disagrees with the label)."""
    models = {}
    for fn in sorted(os.listdir(answers_dir)):
        if fn.endswith(".jsonl"):
            rows = load_answers(os.path.join(answers_dir, fn))
            if rows:
                r0 = next(iter(rows.values()))
                models[f"{r0['model']} ({r0.get('prompt', 'answer-first')})"] = rows

    def code(vals, truth=None):
        out = ""
        for k, ch in zip(LABELS, "PUO"):
            v = (vals or {}).get(k)
            c = "-" if v is None else (ch if v else ch.lower())
            if truth is not None and v is not None and truth[k] is not None and v != truth[k]:
                c += "*"
            out += c + " "
        return out.strip()
    lines = ["| frame | event | label | " + " | ".join(models) + " |", "|" + "---|" * (3 + len(models))]
    for it in manifest["frames"]:
        cells = [code((models[m].get(it["id"]) or {}).get("answer"), it["labels"]) for m in models]
        lines.append(f"| {it['id']} | {it.get('event', '')} | {code(it['labels'])} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grab")
    g.add_argument("video")
    g.add_argument("--at", action="append", required=True)
    g.add_argument("-o", "--out", required=True)
    e = sub.add_parser("extract")
    r = sub.add_parser("run")
    s = sub.add_parser("score")
    for p in (e, r, s):
        p.add_argument("--manifest", default=MANIFEST)
        p.add_argument("--frames", default=OUT_DIR, help="frame directory (default hw_sessions/vlm_set)")
    e.add_argument("--root", default=ROOT, help="the footage paths in the manifest are relative to this")
    r.add_argument("--model", action="append", required=True)
    r.add_argument("--host", default=vlm.DEFAULT_HOST)
    r.add_argument("--num-thread", type=int, default=12, help="CPU threads (ollama num_thread); 0 = server default")
    r.add_argument("--prompt", choices=vlm.ALL_PROMPTS, default=vlm.DEFAULT_PROMPT)
    for p in (r, s):
        p.add_argument("--split", choices=("tune", "heldout"), help="only this half of the seeded split (make_split)")
    r.add_argument("--event", action="append", help="only the frames of this event (repeatable); score counts what was asked")
    for p in (r, s):
        p.add_argument("--answers-dir", default=os.path.join(OUT_DIR, "answers"))
    sp = sub.add_parser("split", help="print the seeded split (ids per half and label counts) as JSON")
    sp.add_argument("--manifest", default=MANIFEST)
    s.add_argument("--md", action="store_true", help="print a markdown table instead of JSON")
    s.add_argument("--frames-table", action="store_true", help="markdown: every frame's labels and each model's answers")
    a = ap.parse_args(argv)
    if a.cmd == "grab":
        grab(a.video, a.at, a.out)
        return 0
    m = load_manifest(a.manifest)
    if a.cmd == "split":
        sp = make_split(m)
        by = {i["id"]: stratum(i) for i in m["frames"]}
        print(json.dumps(dict(seed=SPLIT_SEED, **sp, counts={h: {s: sum(by[i] == s for i in ids) for s in sorted(set(by.values()))}
                                                             for h, ids in sp.items()}), indent=1))
        return 0
    if getattr(a, "split", None):
        m = apply_split(m, a.split)
    if a.cmd == "extract":
        for p in extract(m, a.root, a.frames):
            print(p)
    elif a.cmd == "run":
        if a.event:
            m = dict(m, frames=[f for f in m["frames"] if f.get("event") in a.event])
        for model in a.model:
            run_model(m, model, a.host, a.answers_dir, a.frames, a.num_thread or None, prompt=a.prompt)
    else:
        if a.frames_table:
            print(per_frame(m, a.answers_dir))
            return 0
        sc = score_all(m, a.answers_dir)
        print(markdown(sc) if a.md else json.dumps(sc, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
