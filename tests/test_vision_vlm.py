"""Layer E (#316): the VLM client, the bake-off scoring and set extraction; run_report's clip cut across camd segments.
No model and no network: a fake ollama endpoint on 127.0.0.1."""
import base64
import http.server
import json
import os
import sys
import threading

import cv2
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIS = os.path.join(ROOT, "tools", "vision")
sys.path.insert(0, VIS)
import run_report as RR  # noqa: E402
import vlm as V  # noqa: E402
import vlm_bakeoff as B  # noqa: E402

T0 = 1_794_870_000.0      # an epoch second on 2026-11-16


def _video(tmp_path, n=50, fps=25.0, stamp="pts", name="v"):
    """Synthetic mp4 whose frame k shows the number k; .pts (epoch ms) or .start beside it."""
    path = str(tmp_path / f"{name}.mp4")
    w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (320, 240))
    if not w.isOpened():
        pytest.skip("this OpenCV cannot write mp4v")
    for k in range(n):
        f = np.full((240, 320, 3), 40, np.uint8)
        f[:, : 6 * k % 320] = 200            # a bar whose length encodes k
        w.write(f)
    w.release()
    times = [T0 + k / fps for k in range(n)]
    if stamp == "pts":
        with open(str(tmp_path / f"{name}.pts"), "w") as fh:
            fh.write("# timecode format v2\n" + "".join(f"{t * 1000:.3f}\n" for t in times))
    else:
        with open(str(tmp_path / f"{name}.start"), "w") as fh:
            fh.write(B.utc(T0) + "\n")
    return path, times


class _Fake(http.server.BaseHTTPRequestHandler):
    reply = {"person_or_animal": True, "robot_upright": False, "object_on_board": False, "note": "a hand"}
    seen = []

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        _Fake.seen.append((self.path, body))
        out = {"message": {"role": "assistant", "content": json.dumps(_Fake.reply)},
               "total_duration": 2_500_000_000, "load_duration": 0, "prompt_eval_duration": 2_000_000_000,
               "eval_duration": 500_000_000, "prompt_eval_count": 700, "eval_count": 30}
        data = json.dumps(out).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


@pytest.fixture
def fake_ollama():
    _Fake.seen = []
    srv = http.server.HTTPServer(("127.0.0.1", 0), _Fake)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def test_parse_answer_tolerates_fences_strings_and_missing_keys():
    a = V.parse_answer('```json\n{"person_or_animal": "yes", "robot_upright": false, "note": "x"}\n```')
    assert a == {"person_or_animal": True, "robot_upright": False, "object_on_board": None, "note": "x"}
    assert V.parse_answer("no json here") is None
    assert V.parse_answer("") is None
    assert V.parse_answer("{not: json}") is None


def test_ask_sends_cpu_options_schema_and_image(tmp_path, fake_ollama):
    jpg = str(tmp_path / "f.jpg")
    cv2.imwrite(jpg, np.zeros((10, 10, 3), np.uint8))
    r = V.ask(jpg, "m:1", fake_ollama, num_thread=12)
    assert r["error"] is None and r["answer"]["person_or_animal"] is True and r["answer"]["robot_upright"] is False
    assert r["total_s"] == 2.5 and r["prompt_tokens"] == 700
    path, body = _Fake.seen[-1]
    assert path == "/api/chat" and body["model"] == "m:1" and body["stream"] is False
    assert body["options"]["num_gpu"] == 0 and body["options"]["num_thread"] == 12 and body["options"]["temperature"] == 0
    assert body["format"]["required"] == ["note", *V.QUESTIONS]          # describe-first: the note is generated first
    assert body["messages"][0]["content"] == V.PROMPTS["describe-first"][0]
    V.ask(jpg, "m:1", fake_ollama, prompt="answer-first")
    assert _Fake.seen[-1][1]["format"]["required"] == [*V.QUESTIONS, "note"]
    assert base64.b64decode(body["messages"][0]["images"][0]) == open(jpg, "rb").read()
    # cpu=False leaves num_gpu to the server
    V.ask(jpg, "m:1", fake_ollama, num_thread=None, cpu=False)
    assert "num_gpu" not in _Fake.seen[-1][1]["options"] and "num_thread" not in _Fake.seen[-1][1]["options"]


def test_ask_reports_a_dead_host_instead_of_raising(tmp_path):
    jpg = str(tmp_path / "f.jpg")
    cv2.imwrite(jpg, np.zeros((10, 10, 3), np.uint8))
    r = V.ask(jpg, "m", "http://127.0.0.1:9", timeout_s=2)      # port 9 (discard): refused
    assert r["answer"] is None and r["error"]


def test_score_counts_not_upright_as_the_positive_and_skips_null_labels():
    frames = [dict(id=f"f{i}", labels=dict(person_or_animal=p, robot_upright=u, object_on_board=o))
              for i, (p, u, o) in enumerate([(True, True, False), (True, False, None), (False, False, False),
                                             (False, True, False), (None, True, False)])]
    m = {"frames": frames}
    ans = {"f0": dict(person_or_animal=True, robot_upright=True, object_on_board=False),
           "f1": dict(person_or_animal=False, robot_upright=False, object_on_board=True),
           "f2": dict(person_or_animal=True, robot_upright=True, object_on_board=False),
           "f3": None,                                              # no answer: "nothing to report"
           "f4": dict(person_or_animal=True, robot_upright=True, object_on_board=True)}
    s = B.score(m, ans)
    p = s["person_or_animal"]
    assert (p["tp"], p["fp"], p["fn"], p["tn"], p["unanswered"]) == (1, 1, 1, 1, 1)
    u = s["robot_upright"]          # positive = NOT upright: f1, f2 are the positives; f2 missed
    assert (u["tp"], u["fp"], u["fn"], u["tn"]) == (1, 0, 1, 3) and u["precision"] == 1.0 and u["recall"] == 0.5
    o = s["object_on_board"]        # f1 is null: not scored; no positives at all -> recall None
    assert o["fp"] == 1 and o["recall"] is None and o["precision"] == 0.0


def test_extract_resolves_utc_to_the_right_frame_from_pts_and_start(tmp_path):
    for stamp in ("pts", "start"):
        d = tmp_path / stamp
        d.mkdir()
        path, times = _video(d, stamp=stamp)
        m = {"frames": [dict(id="a", video=os.path.relpath(path, str(tmp_path)), t_utc=B.utc(times[7] + 0.01), frame=7),
                        dict(id="b", video=os.path.relpath(path, str(tmp_path)), t_utc=B.utc(times[30]), frame=30)]}
        out = B.extract(m, root=str(tmp_path), out_dir=str(d / "set"))
        assert [os.path.basename(p) for p in out] == ["a.jpg", "b.jpg"]
        bars = [int((cv2.imread(p)[120, :, 0] > 120).sum()) for p in out]
        assert abs(bars[0] - 42) <= 3 and abs(bars[1] - 180) <= 3, bars       # frame 7 and 30
        m["frames"][0]["frame"] = 8
        with pytest.raises(ValueError, match="resolves to frame 7"):
            B.extract(m, root=str(tmp_path), out_dir=str(d / "set"))


def test_run_model_resumes_and_scores(tmp_path, fake_ollama):
    path, times = _video(tmp_path)
    m = {"frames": [dict(id=f"f{k}", video="v.mp4", t_utc=B.utc(times[k]), frame=k,
                         labels=dict(person_or_animal=True, robot_upright=False, object_on_board=False)) for k in (3, 9)]}
    B.extract(m, root=str(tmp_path), out_dir=str(tmp_path / "set"))
    ad = str(tmp_path / "answers")
    B.run_model(m, "fake:1", fake_ollama, ad, frames_dir=str(tmp_path / "set"), log=lambda *a: None)
    B.run_model(m, "fake:1", fake_ollama, ad, frames_dir=str(tmp_path / "set"), log=lambda *a: None)
    assert len(_Fake.seen) == 2                                  # the second run had nothing left to do
    sc = B.score_all(m, ad)["fake:1 (describe-first)"]
    assert sc["frames"] == 2 and sc["labels"]["person_or_animal"]["recall"] == 1.0
    assert sc["labels"]["robot_upright"]["recall"] == 1.0 and sc["s_per_frame_median"] >= 0
    assert "| fake:1 (describe-first) |" in B.markdown({"fake:1 (describe-first)": sc})
    tab = B.per_frame(m, ad)
    assert "| f3 |  | P u o | P u o |" in tab                  # the fake agrees with both labels


def test_labelled_set_manifest_is_well_formed():
    m = B.load_manifest()
    ids = [f["id"] for f in m["frames"]]
    assert len(ids) == len(set(ids)) and 40 <= len(ids) <= 60
    for f in m["frames"]:
        assert f["video"].startswith("hw_sessions/") and isinstance(f["frame"], int) and f["note"]
        assert set(f["labels"]) == set(V.QUESTIONS)
        assert all(v in (True, False, None) for v in f["labels"].values())
    for k in V.QUESTIONS:                        # both classes present where the footage has them
        vals = [f["labels"][k] for f in m["frames"]]
        assert vals.count(False) >= 10
    assert sum(f["labels"]["person_or_animal"] is True for f in m["frames"]) >= 10
    assert sum(f["labels"]["robot_upright"] is False for f in m["frames"]) >= 10


@pytest.mark.skipif(__import__("shutil").which("ffmpeg") is None, reason="ffmpeg not installed")
def test_cut_clip_spans_camd_segments(tmp_path):
    """A run across a minute boundary: the clip must come from both camd segments, not the better-covering one."""
    a, ta = _video(tmp_path, n=50, name="emeet_20261116T230000Z")
    os.rename(str(tmp_path / "emeet_20261116T230000Z.pts"), str(tmp_path / "keep.pts"))
    b, tb = _video(tmp_path, n=50, name="emeet_20261116T230100Z")
    os.rename(str(tmp_path / "keep.pts"), str(tmp_path / "emeet_20261116T230000Z.pts"))
    with open(str(tmp_path / "emeet_20261116T230100Z.pts"), "w") as fh:     # the next minute's segment
        fh.write("# timecode format v2\n" + "".join(f"{(t + 60.0) * 1000:.3f}\n" for t in tb))
    out = str(tmp_path / "clip.mp4")
    n = RR.cut_clip(a, ta[10], tb[10] + 60.0, out)
    kept = B.C.read_pts(str(tmp_path / "clip.pts"))
    assert n == 40 + 11 == len(kept) and kept[0] == pytest.approx(ta[10]) and kept[-1] == pytest.approx(tb[10] + 60.0)


def test_seeded_split_is_stable_and_stratified():
    m = B.load_manifest()
    a, b = B.make_split(m), B.make_split(m)
    assert a == b
    assert sorted(a["tune"] + a["heldout"]) == sorted(i["id"] for i in m["frames"])
    assert not set(a["tune"]) & set(a["heldout"])
    for s in {B.stratum(i) for i in m["frames"]}:
        n = [sum(B.stratum(i) == s for i in m["frames"] if i["id"] in a[h]) for h in a]
        assert abs(n[0] - n[1]) <= 1


def test_two_call_prompt_merges_both_calls_and_defs_af_is_answer_first(tmp_path, fake_ollama):
    jpg = str(tmp_path / "f.jpg")
    cv2.imwrite(jpg, np.zeros((10, 10, 3), np.uint8))
    n = len(_Fake.seen)
    r = V.ask(jpg, "m:1", fake_ollama, prompt="two-call")
    assert len(_Fake.seen) == n + 2 and r["error"] is None and r["total_s"] == 5.0 and r["prompt_tokens"] == 1400
    assert set(r["answer"]) == {*V.QUESTIONS, "note"} and r["answer"]["robot_upright"] is False
    V.ask(jpg, "m:1", fake_ollama, prompt="defs-af")
    assert _Fake.seen[-1][1]["format"]["required"] == [*V.QUESTIONS, "note"]
    assert "NOT upright" in _Fake.seen[-1][1]["messages"][0]["content"]
