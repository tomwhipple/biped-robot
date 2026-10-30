#!/usr/bin/env python3
"""Night pre-flight: verify tonight's queued jobs before the 22:00 window.

Tom's rule (2026-10-23): "Make sure to always test these jobs before spending
all night on them." Night 2 burned the night on an untested --rise-ref-hold
recipe; the CPU env constructor validates the exact class of bug (plant
mismatch, missing servos, bad hold contract) that killed it, in seconds,
without touching the GPU.

Per queued job, checks (runner-equivalent parsing, then the REAL constructor):
  1. job file parses to an arg list (comments/BRANCH stripped like night_run.sh)
  2. --out unique across queue/, queue/done/ and sim/runs/
  3. --init-from checkpoint exists in sim/runs/
  4. --rise-ref npz exists and resolves as the env resolves it (sim/<name>)
  5. env constructs on CPU with the recipe's full env_kw (robot plant + arms
     held + servo map + rise-ref/hold contract) — constructor raises here, not
     at 22:00, on plant/servo/hold mismatches
CMD= jobs (2026-10-30): a distill job (infra/night_distill.sh <teacher>
[<student>]) is checked for a unique student name (= its OUT=), a teacher
that already exists in the night clone's sim/runs OR is the --out of an
EARLIER queued job (sorted order = run order), a teacher action_map 'full'
(the exporter takes nothing else; DISTILL_ALLOW_LEGACY=1 in the CMD opts
out), the wrapper present on the job's branch, and -- for an existing
teacher -- the distill env constructing on CPU from the teacher's own
config.json.  Any other CMD= job passes with a note (it used to fail as
"no args line").
Exit code: 0 if every job passes, 1 otherwise. NO_NOTIFY/NIGHT_DIR envs for
testing; notifies Tom on failure only (pre-authorized send: the rule is his).
"""
import glob
import json
import os
import subprocess
import sys
import time

BASE = "/home/claw/code/robot"
# The runner trains from the night clone (synced to the job's branch right before pre-flight), so
# env construction, the venv, --init-from and --rise-ref are checked THERE -- the shared checkout
# can lag main (2026-10-30: it lacked #172 and pre-flight refused jobs the clone could run).
CODE = os.environ.get("PREFLIGHT_CODE", "/home/claw/code/robot-mjx")
Q = os.environ.get("NIGHT_DIR", "/home/claw/code/robot-mjx") + "/night/queue"
NOTIFY = os.environ.get("NO_NOTIFY", "") == ""
TOM = "mattermost:xhtnq11nhjn1jpytz1sj4zzimr"
VENV = os.path.join(CODE, ".venv/bin/python")
LOG = os.environ.get("PREFLIGHT_LOG", "/home/claw/code/robot-mjx/night/preflight.log")


def log(msg):
    line = f"[{time.strftime('%F %T')}] {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line)


def notify(text):
    if NOTIFY:
        subprocess.run(["/home/claw/.local/bin/hermes", "send", "-q", "-t", TOM,
                        f"Night pre-flight: {text}"],
                       timeout=60, capture_output=True)


def job_field(path, key):
    for line in open(path):
        if line.startswith(key + "="):
            return line[len(key) + 1:].strip()
    return None


def parse_job(path):
    args = []
    for line in open(path):
        s = line.rstrip("\n")
        if not s.strip() or s.lstrip().startswith("#") or s.startswith(("BRANCH=", "CMD=", "OUT=")):
            continue
        args.extend(s.split())
    return args


def getopt(name, args):
    return args[args.index(name) + 1] if name in args else None


def resolve_out_dir(d, out):
    if not out:
        return None
    return os.path.join(d, out) if d != Q else None


# the env-construction test runs IN the venv (jax/mujoco loaded fresh per job;
# cheap on CPU, ~5-10 s each)
ENV_TEST = r"""
import sys, os, json
sys.path.insert(0, "sim/mjx")
os.chdir("sim/mjx")
os.environ.setdefault("JAX_PLATFORMS", "cpu")
kw = json.loads(sys.argv[1])
import robot_plant
kw_robot = robot_plant.robot_env_kwargs(kw["xml"])
if kw.get("servo_3250"):
    kw_robot["servo_types"] = dict(kw_robot.get("servo_types") or {},
        **{r.strip(): "sts3250" for r in kw["servo_3250"].split(",") if r.strip()})
if kw.get("robot_arms"):
    kw_robot["held_joints"] = {k: v for k, v in (kw_robot.get("held_joints") or {}).items()
                               if robot_plant._role(k) not in ("shoulder", "elbow")}
kw_robot["recover_mix"] = 0.0
kw_robot.update(kw["env_kw"])
kw_robot["xml_path"] = os.path.join(os.getcwd(), "..", kw["xml"])
from env_mjx import BimoMJXEnv
env = BimoMJXEnv(**kw_robot)
print("OK nq=%d rise_secs=%s" % (env.mj_model.nq, getattr(env, "rise_secs", None)))
"""


MJX = "/home/claw/code/robot-mjx"
NIGHT_RUNS = os.path.join(MJX, "sim", "runs")

# the distill env: the teacher's own config.json through distill_student's
# env_kwargs (on the jax impl -- Warp has no CPU path; same plant and kwargs)
DISTILL_ENV_TEST = r"""
import sys, os, json
sys.path.insert(0, "sim/mjx")
os.environ.setdefault("JAX_PLATFORMS", "cpu")
cfg = json.load(open(sys.argv[1]))
from distill_student import env_kwargs
from env_mjx import BimoMJXEnv
try:
    kw = env_kwargs(cfg, "jax", 512)
except TypeError:           # a checkout predating the mjx_impl override
    kw = env_kwargs(cfg)
    kw.pop("mjx_impl", None); kw.pop("warp_envs", None)
env = BimoMJXEnv(**kw)
print("OK obs=%d act=%d map=%s" % (env.obs_size, env.action_size, cfg.get("action_map")))
"""


def check_cmd_job(jf, cmd, seen_outs, queued):
    """CMD= job.  Distill jobs get the real checks; others a parse note."""
    job = os.path.basename(jf)
    e = {"job": job, "ok": False, "errors": []}
    out = job_field(jf, "OUT") or job
    if out in seen_outs:
        e["errors"].append(f"OUT {out} duplicates another queued job")
    seen_outs.add(out)
    for d in (os.path.join(BASE, "sim", "runs"), NIGHT_RUNS):
        if os.path.exists(os.path.join(d, out)):
            e["errors"].append(f"OUT {out} already exists in {d}")
    words = cmd.split()
    if not any(w.endswith("night_distill.sh") for w in words):
        e["env"] = "CMD job (not a distill; command not validated)"
        e["ok"] = not e["errors"]
        return e
    i = next(k for k, w in enumerate(words) if w.endswith("night_distill.sh"))
    pos = [w for w in words[i + 1:] if not w.startswith("-")][:2]
    if not pos:
        e["errors"].append("night_distill.sh needs <teacher> [<student>]")
        return e
    teacher = pos[0]
    student = pos[1] if len(pos) > 1 else teacher + "_s128r24"
    if student != out:
        e["errors"].append(f"student {student} != OUT={out} (the log and the "
                           "collector key on the student name)")
    branch = job_field(jf, "BRANCH") or "main"
    subprocess.run(["git", "-C", MJX, "fetch", "-q", "--filter=blob:none",
                    "origin", branch], capture_output=True, timeout=120)
    r = subprocess.run(["git", "-C", MJX, "cat-file", "-e",
                        f"origin/{branch}:{words[i]}"], capture_output=True)
    if r.returncode != 0:
        e["errors"].append(f"{words[i]} is not on origin/{branch} (the tree the "
                           "runner will check out)")
    allow_legacy = "DISTILL_ALLOW_LEGACY=1" in cmd
    tcfg = os.path.join(NIGHT_RUNS, teacher, "config.json")
    if teacher in queued:
        e["env"] = f"teacher {teacher} trains earlier tonight ({queued[teacher]['job']})"
        amap = queued[teacher]["action_map"]
        if not queued[teacher]["ok"]:
            e["errors"].append(f"teacher job {queued[teacher]['job']} failed pre-flight")
    elif os.path.exists(os.path.join(NIGHT_RUNS, teacher, "params.pkl")) and os.path.exists(tcfg):
        amap = json.load(open(tcfg)).get("action_map", "legacy")
        r = subprocess.run([VENV, "-c", DISTILL_ENV_TEST, tcfg],
                           capture_output=True, text=True, cwd=CODE, timeout=600)
        if "OK obs=" in r.stdout:
            e["env"] = "teacher exists; " + r.stdout.strip().splitlines()[-1]
        else:
            e["errors"].append("distill env CONSTRUCTION FAILED: " +
                               (r.stderr.strip().splitlines()[-1] if r.stderr else r.stdout[-300:]))
    else:
        e["errors"].append(f"teacher {teacher}: no params.pkl in {NIGHT_RUNS} and "
                           "no EARLIER queued job trains it (a distill job must "
                           "sort after its teacher's)")
        return e
    if amap != "full" and not allow_legacy:
        e["errors"].append(f"teacher action_map {amap!r}: the student would not "
                           "be exportable (export_policy_weights.py takes 'full' "
                           "only); train with --action-map full, or put "
                           "DISTILL_ALLOW_LEGACY=1 in the CMD for a referee-only student")
    e["ok"] = not e["errors"]
    return e


def check_job(jf, seen_outs, queued=None):
    queued = {} if queued is None else queued
    job = os.path.basename(jf)
    cmd = job_field(jf, "CMD")
    if cmd:
        return check_cmd_job(jf, cmd, seen_outs, queued)
    e = {"job": job, "ok": False, "errors": []}
    args = parse_job(jf)
    if not args:
        e["errors"].append("no args line")
        return e
    out = getopt("--out", args)
    init = getopt("--init-from", args)
    npz = getopt("--rise-ref", args)
    xml = getopt("--xml", args) or "bimo_biped_v6ar.xml"
    # 2) unique --out (keyed on the --out VALUE, not the queue-file basename —
    #    night_run.sh consumes the file and launches `--out <value>`)
    if out in seen_outs:
        e["errors"].append(f"--out {out} duplicates another queued/earlier job")
    seen_outs.add(out)
    for d in (os.path.join(Q, "done"),
              os.path.join(BASE, "sim", "runs"),
              "/home/claw/code/robot-mjx/sim/runs"):
        p = resolve_out_dir(d, out)
        if out and p and os.path.exists(p):
            e["errors"].append(f"--out {out} already exists in {d}")
    # 3) checkpoint
    if init:
        p = os.path.join(CODE, "sim", "runs", init, "params.pkl")     # train_mjx loads ../runs/<init>
        if not os.path.exists(p):
            e["errors"].append(f"missing --init-from checkpoint {p}")
    # 4) npz as the env resolves it
    if npz:
        p = os.path.join(CODE, "sim", npz)
        if not os.path.exists(p):
            e["errors"].append(f"missing --rise-ref npz {p}")
    # extra flags the env would reject without their dependencies
    if "--rise-ref-hold" in args and not npz:
        e["errors"].append("--rise-ref-hold requires --rise-ref")
    for flag, needs in (("--w-rise-root", "--rise-ref"),
                        ("--rise-ref-residual", "--rise-ref"),
                        ("--rise-ref-s2", "--rise-ref"),
                        ("--rise-ref-et", "--rise-ref")):
        if flag in args and needs not in args:
            e["errors"].append(f"{flag} requires {needs}")
    if e["errors"]:
        return e
    # 5) THE REAL TEST — construct the env on CPU with the recipe's kw
    env_kw = {}

    def fget(flag, conv):
        if flag not in args:
            return None
        try:
            return conv(args[args.index(flag) + 1])
        except (IndexError, ValueError):
            return None

    if "--rise-ref-hold" in args:
        env_kw["rise_ref_hold"] = True
    if "--rise-ref-et-hold" in args:
        env_kw["rise_ref_et_hold"] = True
    if "--clock-freeze-stand" in args:
        env_kw["clock_stand_freeze"] = True
    g = fget("--stop-grace-s", float)
    if g:
        env_kw["stop_grace_s"] = g
    for flag, key, conv in (("--rise-ref", "rise_ref_npz", str),
                            ("--rise-ref-et", "rise_ref_et", float),
                            ("--rise-ref-s2", "rise_ref_s2", float),
                            ("--rise-ref-rsi", "rise_ref_rsi", float),
                            ("--rise-ref-residual", "rise_ref_residual", float)):
        v = fget(flag, conv)
        if v:
            env_kw[key] = v
    for flag, key in (("--w-rise-ref", "w_rise_ref"), ("--w-rise-root", "w_rise_root")):
        v = fget(flag, float)
        if v:
            env_kw[key] = v
    f = fget("--joint-frictionloss", float)
    if f:
        env_kw["joint_frictionloss"] = f
    rng = fget("--joint-frictionloss-range", str)
    if rng:
        env_kw["joint_frictionloss_range"] = tuple(float(x) for x in rng.split(","))
    payload = {"xml": xml, "servo_3250": fget("--servo-3250", str),
               "robot_arms": "--robot-arms" in args, "env_kw": env_kw}
    r = subprocess.run([VENV, "-c", ENV_TEST, json.dumps(payload)],
                       capture_output=True, text=True, cwd=CODE, timeout=600)
    if "OK nq=" in r.stdout:
        e["env"] = r.stdout.strip().splitlines()[-1]
        e["ok"] = not e["errors"]
    else:
        e["errors"].append("env CONSTRUCTION FAILED: " +
                           (r.stderr.strip().splitlines()[-1] if r.stderr else r.stdout[-300:]))
    return e


def main():
    jobs = sorted(glob.glob(os.path.join(Q, "[0-9]*")))
    log(f"=== pre-flight: {len(jobs)} queued job(s) ===")
    seen = set()
    queued = {}     # --out -> {job, action_map, ok}: teachers a later distill may name
    results = []
    for jf in jobs:
        try:
            res = check_job(jf, seen, queued)
            if not job_field(jf, "CMD"):
                a = parse_job(jf)
                if "--out" in a:
                    queued[getopt("--out", a)] = dict(
                        job=os.path.basename(jf), ok=res["ok"],
                        action_map=getopt("--action-map", a) or "legacy")
        except Exception as ex:
            res = {"job": os.path.basename(jf), "ok": False,
                   "errors": [f"preflight error: {ex!r}"]}
        results.append(res)
        log(f"  {res['job']}: {'PASS' if res['ok'] else 'FAIL'}"
            + (f" ({res.get('env')})" if res.get("env") else "")
            + ("; " + "; ".join(res["errors"]) if res["errors"] else ""))
    bad = [r for r in results if not r["ok"]]
    if bad and NOTIFY:
        notify(f"FAILED pre-flight for {len(bad)}/{len(results)} queued job(s): "
               + "; ".join(f"{r['job']}: {r['errors'][0][:160]}" for r in bad)
               + " — fix before 22:00 or the night is wasted.")
    log(f"=== pre-flight {'FAILED' if bad else 'PASSED'}: {len(bad)} bad of {len(results)} ===")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()