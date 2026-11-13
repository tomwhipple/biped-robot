"""infra/night: the planner restarts the night runner when the window is open.

Shell-level tests with a fake clock (NIGHT_NOW) and fake night dir; the real
night_run.sh and train_mjx.py are never started (NIGHT_RUN_CMD is a stub that
drops a file) and nothing here touches the GPU."""
import calendar
import json
import os
import re
import stat
import subprocess
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NIGHT = os.path.join(ROOT, "infra", "night")
LIB = os.path.join(NIGHT, "night_window.sh")
PLAN = os.path.join(NIGHT, "plan_morning.sh")
ENV = {**os.environ, "TZ": "UTC", "LC_ALL": "C"}


def epoch(hhmm, day="2026-11-13"):
    h, m = map(int, hhmm.split(":"))
    y, mo, d = map(int, day.split("-"))
    return calendar.timegm((y, mo, d, h, m, 0))


def decide(nd, hhmm, ok="1", status="PLAN_STATUS: QUEUED 2 jobs - x"):
    r = subprocess.run(
        ["bash", "-c", f'. "{LIB}"; night_continue_decision "$1" "$2" "$3"',
         "_", str(nd), ok, status],
        env={**ENV, "NIGHT_NOW": str(epoch(hhmm))}, capture_output=True, text=True)
    return r.returncode, r.stdout.strip()


@pytest.fixture
def nd(tmp_path):
    (tmp_path / "queue" / "done").mkdir(parents=True)
    for f in (".runner.lock", ".collect.lock", ".planner.lock"):
        (tmp_path / f).touch()
    return tmp_path


def queue(nd, *names):
    for n in names:
        (nd / "queue" / n).write_text("BRANCH=main\n--out x\n")


def test_window_edges():
    def is_open(hhmm):
        r = subprocess.run(["bash", "-c", f'. "{LIB}"; night_window_open'],
                           env={**ENV, "NIGHT_NOW": str(epoch(hhmm))})
        return r.returncode == 0
    assert is_open("22:00") and is_open("23:59") and is_open("00:00")
    assert is_open("03:48") and is_open("06:45")
    assert not is_open("06:46") and not is_open("07:00") and not is_open("07:30")
    assert not is_open("12:00") and not is_open("21:59")


def test_window_matches_night_run():
    src = open(os.path.join(NIGHT, "night_run.sh")).read()
    assert "START_DEADLINE=$((END_HARD - 900))" in src
    assert "END_HARD=$(date -d '07:00' +%s)" in src
    lib = open(LIB).read()
    assert "NIGHT_START_SLACK_S=900" in lib and 'NIGHT_END_HHMM:-07:00' in lib


def test_inside_window_with_jobs_launches(nd):
    queue(nd, "1113220001-a", "1113220002-b")
    rc, out = decide(nd, "03:48")
    assert rc == 0
    assert out == "continuing the night: 2 job(s) queued at 03:48, window open until 07:00"


def test_outside_window_does_nothing(nd):
    queue(nd, "1113220001-a")
    for hhmm in ("06:50", "07:30", "13:00", "21:30"):
        rc, out = decide(nd, hhmm)
        assert rc == 1 and "outside the night window" in out


def test_empty_queue_does_not_launch(nd):
    (nd / "queue" / "held").mkdir()
    (nd / "queue" / "held" / "1113220001-a").write_text("x")
    rc, out = decide(nd, "03:48")
    assert rc == 1 and "no jobs queued" in out


def test_planner_failure_does_not_launch(nd):
    queue(nd, "1113220001-a")
    assert decide(nd, "03:48", ok="0", status="")[0] == 1
    assert decide(nd, "03:48", status="PLAN_STATUS: FAILED - x")[0] == 1
    # NEEDS_TOM WITH jobs queued still continues (the plan says "queue the best night anyway")
    assert decide(nd, "03:48", status="PLAN_STATUS: NEEDS_TOM - x")[0] == 0
    (nd / "queue" / "1113220001-a").unlink()
    assert decide(nd, "03:48", status="PLAN_STATUS: NEEDS_TOM - x")[0] == 1


def test_runner_lock_held_does_not_launch(nd):
    queue(nd, "1113220001-a")
    holder = subprocess.Popen(["flock", str(nd / ".runner.lock"), "sleep", "20"])
    try:
        time.sleep(0.5)
        rc, out = decide(nd, "03:48")
        assert rc == 1 and "runner already holds the lock" in out
    finally:
        holder.kill()
        holder.wait()


# ---- plan_morning.sh end to end with a fake planner and a stub runner ----

@pytest.fixture
def rig(tmp_path):
    n = tmp_path / "night"
    (n / "queue" / "done").mkdir(parents=True)
    for f in (".runner.lock", ".collect.lock"):
        (n / f).touch()
    (n / "planner.log").touch()
    (n / "night.log").touch()
    repo = tmp_path / "repo"
    subprocess.run(["git", "init", "-q", "-b", "main", str(repo)], check=True)
    subprocess.run(["git", "-C", str(repo), "-c", "user.email=a@b", "-c", "user.name=t",
                    "commit", "-q", "--allow-empty", "-m", "i"], check=True)
    subprocess.run(["git", "-C", str(repo), "remote", "add", "origin", str(repo)], check=True)
    claude = tmp_path / "claude"
    claude.write_text(
        '#!/usr/bin/env bash\n'
        'cat > "$RIG/last_prompt"; echo x >> "$RIG/claude_calls"\n'
        'for j in $FAKE_JOBS; do echo "--out $j" > "$NIGHT_DIR/queue/$j"; done\n'
        'printf \'{"result": "ok\\\\n%s"}\' "$FAKE_STATUS"\n'
        'exit "${FAKE_RC:-0}"\n')
    runner = tmp_path / "runner"
    runner.write_text('#!/usr/bin/env bash\necho started >> "$RIG/runner_starts"\n')
    for p in (claude, runner):
        p.chmod(p.stat().st_mode | stat.S_IXUSR)
    return tmp_path, n


def run_plan(rig, hhmm, jobs="", status="PLAN_STATUS: QUEUED 2 jobs - t", rc="0"):
    tmp, n = rig
    env = {**ENV, "RIG": str(tmp), "NIGHT_DIR": str(n), "COLLECT_LOG": str(tmp / "collect.log"),
           "PLAN_WT": str(tmp / "wt"), "PLAN_REPO": str(tmp / "repo"),
           "CLAUDE_BIN": str(tmp / "claude"), "NIGHT_RUN_CMD": str(tmp / "runner"),
           "PLAN_POLL_S": "1", "PLAN_POLL_N": "2", "NIGHT_NOW": str(epoch(hhmm)),
           "FAKE_JOBS": jobs, "FAKE_STATUS": status, "FAKE_RC": rc}
    r = subprocess.run(["bash", PLAN], env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    time.sleep(0.5)    # the stub runner is started detached
    return (n / "planner.log").read_text()


def starts(rig):
    f = rig[0] / "runner_starts"
    return f.read_text().count("started") if f.exists() else 0


def calls(rig):
    f = rig[0] / "claude_calls"
    return f.read_text().count("x") if f.exists() else 0


def test_plan_in_window_restarts_runner_once(rig):
    log = run_plan(rig, "03:48", jobs="1113220001-a 1113220002-b")
    assert "continuing the night: 2 job(s) queued at 03:48, window open until 07:00" in log
    assert starts(rig) == 1
    assert "continuing the night" in (rig[1] / "night.log").read_text()
    assert (rig[1] / ".continued").exists()
    # the stub runner got no inherited lock fds: the planner lock is free again
    assert subprocess.run(["flock", "-n", str(rig[1] / ".planner.lock"), "true"]).returncode == 0
    # a re-trigger with no job launched since: no new plan, no second runner
    log = run_plan(rig, "05:30")
    assert "launched no job" in log and calls(rig) == 1 and starts(rig) == 1
    assert not (rig[1] / ".continued").exists()
    log = run_plan(rig, "07:30")
    assert "already planned today" in log and calls(rig) == 1 and starts(rig) == 1


def test_plan_outside_window_waits_for_22(rig):
    log = run_plan(rig, "12:00", jobs="1113220001-a")
    assert "outside the night window" in log
    assert starts(rig) == 0 and not (rig[1] / ".continued").exists()


def test_plan_failures_do_not_restart(rig):
    run_plan(rig, "03:48", jobs="1113220001-a", status="", rc="1")     # no PLAN_STATUS, no fixer
    assert starts(rig) == 0
    assert "plan_morning failed" in (rig[1] / "planner.log").read_text()
    # (queue left behind by the failed pass) a FAILED status must not start it either
    run_plan(rig, "03:50", status="PLAN_STATUS: FAILED - x")
    assert starts(rig) == 0


def test_plan_empty_queue_no_restart(rig):
    run_plan(rig, "03:48", status="PLAN_STATUS: NEEDS_TOM - pick")
    assert starts(rig) == 0


def test_addendum_pass_after_continued_jobs(rig):
    run_plan(rig, "03:48", jobs="1113220001-a 1113220002-b")
    assert starts(rig) == 1
    time.sleep(0.05)
    q = rig[1] / "queue"
    os.rename(q / "1113220001-a", q / "done" / "1113220001-a")   # the runner launched one
    log = run_plan(rig, "06:50", status="PLAN_STATUS: QUEUED 1 jobs - addendum")
    assert "plan_morning (addendum" in log and calls(rig) == 2
    prompt = (rig[0] / "last_prompt").read_text()
    assert "THIS IS AN ADDENDUM PASS" in prompt and "1113220001-a" in prompt
    # 06:50 is past the last start: the addendum does not relaunch the runner
    assert starts(rig) == 1 and "outside the night window" in log
    assert not (rig[1] / ".continued").exists()
    # and nothing re-plans afterwards
    assert "already planned today" in run_plan(rig, "07:30")
    assert calls(rig) == 2


def test_addendum_inside_window_chains_again(rig):
    run_plan(rig, "01:00", jobs="1113220001-a 1113220002-b")
    time.sleep(0.05)
    q = rig[1] / "queue"
    os.rename(q / "1113220001-a", q / "done" / "1113220001-a")
    log = run_plan(rig, "04:00")
    assert starts(rig) == 2 and (rig[1] / ".continued").exists()
    assert "continuing the night: 1 job(s)" in log
