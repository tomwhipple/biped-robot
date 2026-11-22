"""A local vision-language model as a coarse second opinion on a frame (#316, layer E).

Three yes/no questions per frame, asked of an ollama server (`/api/chat`, JSON-schema output). It is a
hint for a person to look at the frame, never a measurement and never a gate: position, tilt and falls
come from the tags and the flight record.

CPU by default. The GPU is not free (22:00-07:00 night training, by day camera-watcher), so the default
host is a CPU-only ollama on 127.0.0.1:11500, and every request also sends `num_gpu: 0` (no layers on a
GPU even if the host has one) and `num_thread` (default 12, leaving the rest of the box to training).
Start that server like this (the models live in their own store; the binary is the reviewed user-local
CPU-only ollama v0.40.16 unpacked without its GPU backends, see tools/vision/README.md, "Layer E"; the older
`CUDA_VISIBLE_DEVICES=`-only recipe is not safe with 0.40.x, which enables Vulkan by default):

    mkdir -p ~/.cache/ollama-cpu ~/.cache/ollama-cpu-home
    env -i PATH=/usr/bin:/bin HOME=$HOME/.cache/ollama-cpu-home \\
        CUDA_VISIBLE_DEVICES= OLLAMA_VULKAN=0 GGML_VK_VISIBLE_DEVICES=-1 OLLAMA_LLM_LIBRARY=cpu OLLAMA_NO_CLOUD=1 \\
        OLLAMA_HOST=127.0.0.1:11500 OLLAMA_MODELS=$HOME/.cache/ollama-cpu \\
        OLLAMA_NUM_PARALLEL=1 OLLAMA_MAX_LOADED_MODELS=1 \\
        setsid nohup nice -n 19 $HOME/.local/opt/ollama-0.40.16/bin/ollama serve > /tmp/ollama-cpu.log 2>&1 < /dev/null &

CLI calls (pull) always use that binary with OLLAMA_HOST=127.0.0.1:11500 set.

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

# Tuned variants (#316, 2026-11-22, see docs/design-v6/2026-11-22-vlm-prompt-tuning.md): the two failures of the
# first two prompts were "describe-first calls a fallen robot upright" and "answer-first misses the dog".
_DEFS = (
    "Definitions:\n"
    "- NOT upright means any of: lying on the board or floor, on its side or back or front, seen from the side "
    "with the torso horizontal or clearly slanted, one leg on the board and the body tipped or sagging, falling, "
    "or leaning past a slight tilt. Upright means the torso stands vertical on its two legs, including mid-step. "
    "When unsure whether it is tilted, answer false for robot_upright.\n"
    "- A person counts even if only a hand, an arm, a sleeve or a leg is visible at the board or on the robot. "
    "An animal (a dog, a cat; often small, brown, black or white, far away or only partly in view: a tail, "
    "legs, a head or a paw) counts when it is on the board or next to it. A person sitting in a chair away "
    "from the board does not count.\n"
)
_Q = "".join(f"- {k}: {q}\n" for k, q in QUESTIONS.items())
PROMPTS["defs"] = (
    _SETUP + _DEFS + "First, in \"note\", describe in one or two sentences what you see: the robot's pose (standing, "
    "stepping, leaning, tipping over, lying on its side, being held) and every person, hand, leg, animal or loose "
    "object on or near the board. Then answer each question with true or false, using the definitions:\n" + _Q
    + "Reply with JSON only.",
    PROMPTS["describe-first"][1])
# answer-first with the definitions (the booleans are written before any description, which found more falls)
PROMPTS["defs-af"] = (
    _SETUP + _DEFS + "Answer each question about this frame with true or false, using the definitions:\n" + _Q
    + "Also give a one-sentence description in \"note\". Reply with JSON only.",
    PROMPTS["answer-first"][1])
# "two-call": one short call per question group, answers merged (about twice the calls per frame).
TWO_CALL = {
    "two-call": [
        (_SETUP + _DEFS + "First, in \"note\", say in one sentence whether any person, hand, leg, sleeve or animal "
         "is on or near the board and where. Then answer with true or false:\n- person_or_animal: "
         + QUESTIONS["person_or_animal"] + "\n- object_on_board: " + QUESTIONS["object_on_board"]
         + "\nReply with JSON only.",
         {"type": "object", "properties": {"note": {"type": "string"}, "person_or_animal": {"type": "boolean"},
                                           "object_on_board": {"type": "boolean"}},
          "required": ["note", "person_or_animal", "object_on_board"]}),
        (_SETUP + _DEFS + "First, in \"note\", say in one sentence what angle the robot's torso is at (vertical, "
        "slanted, horizontal) and whether it is seen from the front or the side. Then answer with true or "
        "false:\n- robot_upright: " + QUESTIONS["robot_upright"] + "\nReply with JSON only.",
         {"type": "object", "properties": {"note": {"type": "string"}, "robot_upright": {"type": "boolean"}},
          "required": ["note", "robot_upright"]}),
    ],
}
ALL_PROMPTS = sorted([*PROMPTS, *TWO_CALL])
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


def _chat(img, text, schema, model, host, opts, timeout_s):
    body = dict(model=model, stream=False, format=schema, options=opts, keep_alive="10m",
                messages=[dict(role="user", content=text, images=[img])])
    return _post(host.rstrip("/") + "/api/chat", body, timeout_s)


def ask(image_path, model, host=DEFAULT_HOST, num_thread=12, cpu=True, timeout_s=600.0, prompt=DEFAULT_PROMPT):
    """-> dict(answer={...}|None, raw=str, error=str|None, total_s, load_s, prompt_eval_s, eval_s).
    Never raises on a model or network failure: the error is returned (a second opinion that is down is
    reported as missing, not as a clean frame). A TWO_CALL prompt makes one request per entry and merges the
    answers (times are summed; `raw` is the replies joined by a newline)."""
    calls = TWO_CALL.get(prompt) or [PROMPTS[prompt]]
    with open(image_path, "rb") as f:
        img = base64.b64encode(f.read()).decode()
    opts = {"temperature": 0}
    if num_thread:
        opts["num_thread"] = int(num_thread)
    if cpu:
        opts["num_gpu"] = 0
    t = time.time()
    raws, merged, notes = [], {}, []
    tot = dict(total_s=0.0, load_s=0.0, prompt_eval_s=0.0, eval_s=0.0, prompt_tokens=0, eval_tokens=0)
    for text, schema in calls:
        try:
            r = _chat(img, text, schema, model, host, opts, timeout_s)
        except (urllib.error.URLError, OSError, ValueError) as e:
            return dict(answer=None, raw="\n".join(raws), error=f"{type(e).__name__}: {e}", total_s=round(time.time() - t, 2))
        raw = (r.get("message") or {}).get("content", "")
        raws.append(raw)
        part = parse_answer(raw)
        if part:
            notes.append(part.pop("note"))
            merged.update({k: v for k, v in part.items() if k in schema["properties"]})
        for k, src in (("total_s", "total_duration"), ("load_s", "load_duration"), ("prompt_eval_s", "prompt_eval_duration"),
                       ("eval_s", "eval_duration")):
            tot[k] += r.get(src, 0) / 1e9
        tot["prompt_tokens"] += r.get("prompt_eval_count") or 0
        tot["eval_tokens"] += r.get("eval_count") or 0
    ans = {k: merged.get(k) for k in QUESTIONS}
    ans["note"] = " | ".join(notes)[:300]
    ok = len(notes) == len(calls)
    return dict(answer=ans if ok else None, raw="\n".join(raws), prompt=prompt, error=None if ok else "unparseable answer",
                **{k: round(v, 2) if isinstance(v, float) else v for k, v in tot.items()})
