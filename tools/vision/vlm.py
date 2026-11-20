"""A local vision-language model as a coarse second opinion on a frame (#316, layer E).

Three yes/no questions per frame, asked of an ollama server (`/api/chat`, JSON-schema output). It is a
hint for a person to look at the frame, never a measurement and never a gate: position, tilt and falls
come from the tags and the flight record.

CPU by default. The GPU is not free (22:00-07:00 night training, by day camera-watcher), so the default
host is a CPU-only ollama on 127.0.0.1:11500, and every request also sends `num_gpu: 0` (no layers on a
GPU even if the host has one) and `num_thread` (default 12, leaving the rest of the box to training).
Start that server like this (the models live in their own store):

    CUDA_VISIBLE_DEVICES= OLLAMA_HOST=127.0.0.1:11500 OLLAMA_MODELS=$HOME/.cache/ollama-cpu \\
        OLLAMA_NUM_PARALLEL=1 OLLAMA_MAX_LOADED_MODELS=1 setsid nohup nice -n 19 ollama serve &

Standard library only (urllib); no network beyond the given host.
"""
import base64
import json
import re
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "http://127.0.0.1:11500"

# label key -> the question as put to the model
QUESTIONS = {
    "person_or_animal": "Is a person or an animal touching the robot, on the board, or standing right at the "
                        "board's edge next to it? (A person sitting in a chair away from the board does not count.)",
    "robot_upright": "Is the robot upright, its torso within about 15 degrees of vertical (not tipping over, "
                     "falling or lying down)?",
    "object_on_board": "Is there a loose object lying on the board other than the robot, its own cable and the "
                       "printed square markers (for example a tool, a tape measure, a cup, clothing, a stray "
                       "cable)?",
}
SHORT = {"person_or_animal": "person/animal", "robot_upright": "upright", "object_on_board": "object"}
# the answer that means "look at this frame": what precision / recall are counted on
POSITIVE = {"person_or_animal": True, "robot_upright": False, "object_on_board": True}

# What the labels in the bake-off set mean (the labeller's rules; vlm_set.json follows them).
LABEL_RULES = {
    "person_or_animal": "yes when any part of a person or an animal touches the robot, is on the board, or stands "
                        "or walks at the board's edge within reach of the robot (the dog, a person stepping up to "
                        "pick it up). No for a person sitting in a chair off the board and not reaching in, even "
                        "with feet near the edge (the usual spectator), and for someone far behind the board. A "
                        "hand that is out of frame or hidden: null (not scored).",
    "robot_upright": "posture only: yes when the torso is within ~15 deg of vertical, mid-step included, and also "
                     "when it is being held level by a hand; no when it is tipping past ~15 deg, falling or lying. "
                     "Checked against the flight-record IMU tilt at the frame time where one exists (the low Sony "
                     "view makes legs in single support look tilted). Robot not in view: null.",
    "object_on_board": "yes when a loose object lies on the board: a tool, tape measure, cup, clothing, a foreign "
                       "cable. Not counted: the robot, its own tether, the printed tags, the board seam, people and "
                       "animals (their own question). A cable whose owner the frame cannot show (the thick black "
                       "cable across the near end on 11-16 before 21:05Z): null.",
}

_SETUP = (
    "This is a frame from a camera watching a small two-legged robot (about 40 cm tall: a box-shaped torso "
    "on two long legs made of servos and brackets) that walks on a brown wooden board on the floor. Its "
    "red/black tether cable and the square black-and-white printed markers on the board and on the robot "
    "are normal. The camera may show only part of the robot.\n"
)

# Two prompt styles (the bake-off ran both, see README "Layer E"):
#   "answer-first": the three booleans, then a one-sentence note;
#   "describe-first": the model first describes the robot's pose and everyone/everything near it, then answers
#   (the JSON keys are generated in schema order, so the description is written before the booleans).
PROMPTS = {
    "answer-first": (
        _SETUP + "Answer each question about this frame with true or false:\n"
        + "".join(f"- {k}: {q}\n" for k, q in QUESTIONS.items())
        + "Also give a one-sentence description in \"note\". Reply with JSON only.",
        {"type": "object",
         "properties": {**{k: {"type": "boolean"} for k in QUESTIONS}, "note": {"type": "string"}},
         "required": [*QUESTIONS, "note"]}),
    "describe-first": (
        _SETUP + "First, in \"note\", describe in one or two sentences what you see: the robot's pose (standing, "
        "stepping, leaning, tipping over, lying on its side, being held) and every person, hand, leg, animal or "
        "loose object on or near the board. Then answer each question with true or false:\n"
        + "".join(f"- {k}: {q}\n" for k, q in QUESTIONS.items())
        + "Reply with JSON only.",
        {"type": "object",
         "properties": {"note": {"type": "string"}, **{k: {"type": "boolean"} for k in QUESTIONS}},
         "required": ["note", *QUESTIONS]}),
}
DEFAULT_PROMPT = "describe-first"
PROMPT, SCHEMA = PROMPTS[DEFAULT_PROMPT]


def _post(url, body, timeout_s):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout_s) as r:
        return json.loads(r.read().decode())


def parse_answer(text):
    """Model text -> {label: bool|None, note: str} or None. Tolerates code fences, extra prose and
    "yes"/"no" strings; a missing key is None."""
    if not text:
        return None
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        return None
    try:
        d = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    out = {}
    for k in QUESTIONS:
        v = d.get(k)
        if isinstance(v, str):
            v = {"true": True, "yes": True, "false": False, "no": False}.get(v.strip().lower())
        out[k] = v if isinstance(v, bool) else None
    out["note"] = str(d.get("note", ""))[:300]
    return out


def ask(image_path, model, host=DEFAULT_HOST, num_thread=12, cpu=True, timeout_s=600.0, prompt=DEFAULT_PROMPT):
    """-> dict(answer={...}|None, raw=str, error=str|None, total_s, load_s, prompt_eval_s, eval_s).
    Never raises on a model or network failure: the error is returned (a second opinion that is down is
    reported as missing, not as a clean frame)."""
    text, schema = PROMPTS[prompt]
    with open(image_path, "rb") as f:
        img = base64.b64encode(f.read()).decode()
    opts = {"temperature": 0}
    if num_thread:
        opts["num_thread"] = int(num_thread)
    if cpu:
        opts["num_gpu"] = 0
    body = dict(model=model, stream=False, format=schema, options=opts, keep_alive="10m",
                messages=[dict(role="user", content=text, images=[img])])
    t = time.time()
    try:
        r = _post(host.rstrip("/") + "/api/chat", body, timeout_s)
    except (urllib.error.URLError, OSError, ValueError) as e:
        return dict(answer=None, raw="", error=f"{type(e).__name__}: {e}", total_s=round(time.time() - t, 2))
    raw = (r.get("message") or {}).get("content", "")
    ns = lambda k: round(r.get(k, 0) / 1e9, 2)
    ans = parse_answer(raw)
    return dict(answer=ans, raw=raw, prompt=prompt, error=None if ans else "unparseable answer", total_s=ns("total_duration"),
                load_s=ns("load_duration"), prompt_eval_s=ns("prompt_eval_duration"), eval_s=ns("eval_duration"),
                prompt_tokens=r.get("prompt_eval_count"), eval_tokens=r.get("eval_count"))
