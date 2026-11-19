"""infra/night: the runners mirror the harness from the repo, re-exec once, and
build their inputs with make.

The REAL night_run.sh / day_run.sh run against a fake HOME: a throwaway bare
`origin`, a clone of it as ~/code/robot-mjx, stub harness helpers (fixer,
pre-flight, vision pause) and PATH shims for nvidia-smi / ollama / docker /
curl / date. Jobs are `CMD=echo ...`. Nothing here touches the GPU, docker, the
real clone or the real crontab."""
import os
import shutil
import stat
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"),
                                reason="infra/night scripts are Linux-only")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL = os.path.join(ROOT, "infra", "night")
RUNNERS = ("night_run.sh", "day_run.sh")
ENV0 = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}

STUBS = {
    # fixer: record the call; FIX_RC=0 means "fixed" (after running FIX_CMD)
    "claude_fix.sh": '''#!/usr/bin/env bash
echo "$1 $2 $3 :: $(cat)" >> "$RIG/fix_calls"
[[ ${FIX_RC:-1} == 0 ]] || exit 1
[[ -n ${FIX_CMD:-} ]] && eval "$FIX_CMD"
exit 0
''',
    "vision_pause.sh": '''#!/usr/bin/env bash
echo "$1 HARNESS_REEXEC=${HARNESS_REEXEC:-}" >> "$RIG/vision_calls"
if [[ $1 == pause && -n ${HOLD_PAUSE:-} ]]; then
  touch "$RIG/paused"
  while [[ ! -f "$RIG/release" ]]; do sleep 0.1; done
fi
exit 0
''',
    "preflight.sh.py": "import sys\nsys.exit(0)\n",
    "planner_prompt.md": "prompt\n",
}
SHIMS = {
    "nvidia-smi": '#!/bin/sh\necho "3, 100, 12282"\n',
    "ollama": "#!/bin/sh\nexit 0\n",
    "docker": "#!/bin/sh\nexit 0\n",
    "curl": "#!/bin/sh\nexit 0\n",
    "systemctl": "#!/bin/sh\nexit 0\n",
    # the runner polls a CMD job with `sleep 60`; a job that ends between two
    # polls must not cost the test a minute
    "sleep": "#!/bin/sh\nexec /usr/bin/sleep 0.2\n",
    # the hard stop is "07:00": move it a day ahead so the test never meets the
    # real clock's start deadline
    "date": '''#!/bin/bash
args=(); for a in "$@"; do [[ $a == 07:00 ]] && a="tomorrow 12:00"; args+=("$a"); done
exec /usr/bin/date "${args[@]}"
''',
}


def sh(path, text, mode=0o755):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(mode)


def git(cwd, *a, check=True):
    env = {**ENV0, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
           "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"}
    return subprocess.run(["git", "-C", str(cwd), *a], check=check, env=env,
                          capture_output=True, text=True)


class World:
    def __init__(self, tmp):
        self.tmp = tmp
        self.home = tmp / "home"
        self.base = self.home / "code" / "robot-mjx"
        self.n = self.base / "night"
        self.rig = tmp / "rig"
        self.rig.mkdir()
        self.shims = tmp / "shims"
        for k, v in SHIMS.items():
            sh(self.shims / k, v)
        # origin/main: Makefile + infra/night (real runners + date lib, stub helpers)
        self.work = tmp / "work"
        self.origin = tmp / "origin.git"
        subprocess.run(["git", "init", "-q", "--bare", "-b", "main", str(self.origin)], check=True)
        self.work.mkdir()
        git(self.work, "init", "-q", "-b", "main")
        git(self.work, "remote", "add", "origin", str(self.origin))
        (self.work / "Makefile").write_text(
            "plant-data:\n\ttest ! -f BROKEN\n\techo built >> build.count\n")
        (self.work / ".gitignore").write_text("/night/\n.venv/\nbuild.count\n")
        inf = self.work / "infra" / "night"
        for f in (*RUNNERS, "night_window.sh"):
            shutil.copy(os.path.join(REAL, f), inf / f if inf.exists() else self._mk(inf, f))
        for k, v in STUBS.items():
            sh(inf / k, v, 0o644 if k.endswith(".md") else 0o755)
        sh(inf / "night_stop.sh", "#!/bin/sh\n")
        (inf / "night_window.sh").chmod(0o644)
        sh(inf / "night_run.sh.bak-1", "#!/bin/sh\necho old backup\n")
        self.push("main", "init")
        subprocess.run(["git", "clone", "-q", str(self.origin), str(self.base)], check=True)
        (self.base / ".venv" / "bin").mkdir(parents=True)
        (self.base / ".venv" / "bin" / "python").symlink_to(sys.executable)
        (self.n / "queue").mkdir(parents=True)
        for f in RUNNERS:                  # the deployed copies, in sync with the repo
            shutil.copy(os.path.join(REAL, f), self.n / f)

    def _mk(self, d, f):
        d.mkdir(parents=True)
        return d / f

    def push(self, branch, msg, files=None, remove=()):
        git(self.work, "checkout", "-q", "-B", branch)
        for name, text in (files or {}).items():
            (self.work / name).write_text(text)
        for name in remove:
            (self.work / name).unlink()
        git(self.work, "add", "-A")
        git(self.work, "commit", "-q", "--allow-empty", "-m", msg)
        git(self.work, "push", "-q", "-f", "origin", branch)
        git(self.work, "checkout", "-q", "main") if branch != "main" else None

    def old_runner(self, runner):
        """$N's copy of the runner, one version behind the repo's."""
        src = open(os.path.join(REAL, runner)).read()
        src = src.replace('PY="$BASE/.venv/bin/python"\n',
                          'PY="$BASE/.venv/bin/python"\necho OLD-VERSION-RAN\n', 1)
        assert "OLD-VERSION-RAN" in src
        sh(self.n / runner, src)

    def job(self, name, body):
        (self.n / "queue" / name).write_text(body)

    def env(self, **kw):
        return {**ENV0, "HOME": str(self.home), "PATH": f"{self.shims}:{os.environ['PATH']}",
                "RIG": str(self.rig), "LC_ALL": "C", "COLLECT_SCRIPT": "/bin/true",
                "DAY_START_BY": "23:59", "DAY_END_BY": "tomorrow 12:00", **kw}

    def run(self, runner, timeout=120, **kw):
        return subprocess.run([str(self.n / runner)], env=self.env(**kw), cwd="/",
                              capture_output=True, text=True, timeout=timeout)

    def log(self):
        f = self.n / "night.log"
        return f.read_text() if f.exists() else ""

    def calls(self, name):
        f = self.rig / name
        return f.read_text().splitlines() if f.exists() else []


@pytest.fixture
def world(tmp_path):
    return World(tmp_path)


def cmd_job(world, name, text, branch="main"):
    world.job(name, f"BRANCH={branch}\nCMD=echo {text}\n")


def ran(world, name):
    f = world.base / "sim" / "runs" / f"cmd_{name}.log"
    return f.read_text().strip() if f.exists() else None


@pytest.mark.parametrize("runner", RUNNERS)
def test_mirror_is_write_then_rename_and_skips_bak(world, runner):
    world.old_runner(runner)
    sh(world.n / "night_stop.sh", "#!/bin/sh\n# old stop\n")
    stop_inode = (world.n / "night_stop.sh").stat().st_ino
    reader = open(world.n / "night_stop.sh")           # a script "still running"
    cmd_job(world, "1119220001-a", "hi")
    r = world.run(runner)
    assert r.returncode == 0, r.stderr
    n = world.n
    assert (n / "night_stop.sh").read_text() == "#!/bin/sh\n"
    assert (n / "night_stop.sh").stat().st_ino != stop_inode     # renamed over, not rewritten
    assert reader.read() == "#!/bin/sh\n# old stop\n"            # the old inode is intact
    reader.close()
    for f in ("night_run.sh", "day_run.sh", "night_stop.sh", "claude_fix.sh", "vision_pause.sh"):
        assert os.access(n / f, os.X_OK), f
    assert not os.access(n / "night_window.sh", os.X_OK) and not os.access(n / "planner_prompt.md", os.X_OK)
    assert not list(n.glob("*.bak-*"))                           # skipped
    assert not [p for p in n.iterdir() if ".new." in p.name]     # no temp files left
    for f in ("night_run.sh", "day_run.sh", "night_window.sh", "preflight.sh.py"):
        assert (n / f).read_bytes() == (world.base / "infra" / "night" / f).read_bytes()
    assert "harness: night_stop.sh updated" in world.log()


@pytest.mark.parametrize("runner", RUNNERS)
def test_reexec_once_when_the_runner_changed(world, runner):
    world.old_runner(runner)
    cmd_job(world, "1119220001-a", "hi")
    r = world.run(runner)
    assert r.returncode == 0, r.stderr
    log = world.log()
    assert log.count("OLD-VERSION-RAN") == 1                     # the old copy stops at the exec
    assert log.count(f"harness: {runner} updated") == 1
    assert log.count("re-exec'ing the new copy") == 1
    assert log.count("re-exec'd as the updated harness; lock held") == 1
    assert log.count("launched CMD job 1119220001-a") == 1        # one night, one version
    assert ran(world, "1119220001-a") == "hi"
    # in sync now: the next start does not re-exec
    cmd_job(world, "1119220002-b", "again")
    assert world.run(runner).returncode == 0
    assert world.log().count("re-exec'ing the new copy") == 1
    assert ran(world, "1119220002-b") == "again"
    # the env flag is not passed on to the helpers the re-exec'd runner starts
    assert all(c.endswith("HARNESS_REEXEC=") for c in world.calls("vision_calls"))


@pytest.mark.parametrize("runner", RUNNERS)
def test_reexec_keeps_the_lock_held(world, runner):
    world.old_runner(runner)
    cmd_job(world, "1119220001-a", "hi")
    p = subprocess.Popen([str(world.n / runner)], env=world.env(HOLD_PAUSE="1"), cwd="/",
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(300):                       # the re-exec'd runner is inside vision_pause
            if (world.rig / "paused").exists():
                break
            time.sleep(0.1)
        assert (world.rig / "paused").exists(), world.log()
        assert "re-exec'd as the updated harness" in world.log()
        # a lock probe from outside, and a second runner, both see the lock held
        assert subprocess.run(["flock", "-n", str(world.n / ".runner.lock"), "true"]).returncode == 1
        r = world.run(runner, timeout=30)
        assert r.returncode == 0
        assert "another runner holds the lock; exiting" in world.log()
        assert world.calls("vision_calls") == ["pause HARNESS_REEXEC="]   # it ran nothing
    finally:
        (world.rig / "release").touch()
        assert p.wait(timeout=60) == 0
    assert ran(world, "1119220001-a") == "hi"


@pytest.mark.parametrize("runner", RUNNERS)
def test_stray_reexec_flag_without_the_lock_does_not_loop(world, runner):
    world.old_runner(runner)
    cmd_job(world, "1119220001-a", "hi")
    r = world.run(runner, HARNESS_REEXEC="1")      # flag set, but fd 9 is not the lock
    assert r.returncode == 0, r.stderr
    log = world.log()
    assert log.count("re-exec'ing the new copy") == 1
    assert log.count("launched CMD job") == 1


@pytest.mark.parametrize("runner", RUNNERS)
def test_failed_job_build_moves_it_to_held_and_continues(world, runner):
    world.push("bad", "broken branch", files={"BROKEN": "x\n"})
    cmd_job(world, "1119220001-bad", "never", branch="bad")
    cmd_job(world, "1119220002-good", "fine")
    r = world.run(runner)
    assert r.returncode == 0, r.stderr
    log = world.log()
    assert "1119220001-bad: make plant-data failed on bad; moved to queue/held/, next job" in log
    held = (world.n / "queue" / "held" / "1119220001-bad").read_text()
    assert held.startswith("BRANCH=bad\nCMD=echo never\n")
    assert "# held by runner" in held and "make plant-data failed on bad" in held
    assert not (world.n / "queue" / "1119220001-bad").exists()
    assert ran(world, "1119220001-bad") is None
    assert ran(world, "1119220002-good") == "fine"                # the night went on
    assert (world.n / "queue" / "done" / "1119220002-good").exists()
    assert "make plant-data ok" in log
    assert "make plant-data" in (world.n / "make.log").read_text()
    assert not (world.base / "BROKEN").exists()                   # tree back on main
    assert world.calls("fix_calls") == []                         # no fixer for a held job


def test_held_jobs_are_not_run_and_do_not_stop_the_scan(world):
    (world.n / "queue" / "held").mkdir()
    (world.n / "queue" / "held" / "1119220000-old").write_text("BRANCH=main\nCMD=echo old\n")
    cmd_job(world, "1119220001-a", "hi")
    assert world.run("night_run.sh").returncode == 0
    assert ran(world, "1119220001-a") == "hi" and ran(world, "1119220000-old") is None


def test_start_build_failure_takes_the_fix_path_then_retries(world):
    world.push("main", "broken main", files={"BROKEN": "x\n"})
    cmd_job(world, "1119220001-a", "hi")
    # the fixer pushes its fix to origin/main (the runner re-syncs before the retry)
    w = world.work
    fix = (f"git -C {w} rm -q BROKEN && git -C {w} -c user.name=t -c user.email=t@t "
           f"commit -qm fix && git -C {w} push -q origin main")
    r = world.run("night_run.sh", FIX_RC="0", FIX_CMD=fix)
    assert r.returncode == 0, r.stderr
    fixes = [c for c in world.calls("fix_calls") if c.startswith("build-inputs")]
    assert len(fixes) == 1 and fixes[0].startswith(f"build-inputs {world.n}/make.log 20 ::")
    log = world.log()
    assert "make plant-data failed" in log and "make plant-data ok" in log
    assert "FAILED; refusing" not in log
    assert ran(world, "1119220001-a") == "hi"


def test_start_build_failure_unfixed_stops_the_night(world):
    world.push("main", "broken main", files={"BROKEN": "x\n"})
    cmd_job(world, "1119220001-a", "hi")
    r = world.run("night_run.sh", FIX_RC="1")
    assert r.returncode == 1
    log = world.log()
    assert len([c for c in world.calls("fix_calls") if c.startswith("build-inputs")]) == 1
    assert "make plant-data FAILED; refusing to launch the queue" in log
    assert (world.n / "queue" / "1119220001-a").exists()          # still queued
    assert ran(world, "1119220001-a") is None
    assert world.calls("vision_calls") == ["resume HARNESS_REEXEC="]   # never paused


def test_no_build_without_a_queue(world):
    r = world.run("night_run.sh")
    assert r.returncode == 0, r.stderr
    assert not (world.base / "build.count").exists()
    assert "make plant-data" not in world.log()


def test_branch_job_never_mirrors_its_harness(world):
    world.push("feat", "feature harness", files={"infra/night/night_stop.sh": "#!/bin/sh\n# FEATURE\n"})
    cmd_job(world, "1119220001-a", "hi", branch="feat")
    assert world.run("night_run.sh").returncode == 0
    assert "FEATURE" not in (world.n / "night_stop.sh").read_text()


# ---- night_arm.sh writes the cron lines for the night/ copies ----

def test_night_arm_cron_lines(tmp_path):
    repo = tmp_path / "repo"
    (repo / "infra").mkdir(parents=True)
    shutil.copy(os.path.join(ROOT, "infra", "night_arm.sh"), repo / "infra" / "night_arm.sh")
    bare = tmp_path / "o.git"
    subprocess.run(["git", "init", "-q", "--bare", "-b", "main", str(bare)], check=True)
    git(repo, "init", "-q", "-b", "main")
    git(repo, "remote", "add", "origin", str(bare))
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "i")
    git(repo, "push", "-q", "origin", "main")
    shims = tmp_path / "shims"
    tab = tmp_path / "tab"
    tab.write_text("0 */4 * * * /home/claw/token.sh\n"
                   "0 22 * * * /home/claw/code/robot-mjx/infra/night/night_run.sh\n"
                   "15 7 * * * /home/claw/code/robot/infra/night_collect_local.sh\n")
    # ssh runs the remote script locally; sudo drops its own flags; the night
    # scripts "exist"; crontab reads/writes a file
    sh(shims / "ssh", '#!/bin/bash\nshift\ncase "$1" in\n*"bash -s") sed "/-x /s#/home/claw/code/robot-mjx/night#$FAKE_NIGHT#g" | bash -s;;\n*tee*) cat > "$FAKE_NIGHT/queue/q";;\n*) ls "$FAKE_NIGHT/queue";;\nesac\n')
    sh(shims / "crontab", '#!/bin/bash\nif [[ ${1:-} == - ]]; then cat > "$FAKE_TAB"; else cat "$FAKE_TAB"; fi\n')
    night = tmp_path / "night"
    (night / "queue").mkdir(parents=True)
    for f in ("night_run", "night_stop", "plan_morning", "ensure_queue"):
        sh(night / f"{f}.sh", "#!/bin/sh\n")
    env = {**ENV0, "PATH": f"{shims}:{os.environ['PATH']}", "FAKE_TAB": str(tab), "FAKE_NIGHT": str(night)}
    # (the ssh shim points only the script's `-x` existence checks at the fake dir)
    r = subprocess.run(["bash", str(repo / "infra" / "night_arm.sh"), "--out", "x"],
                       env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr
    N = "/home/claw/code/robot-mjx/night"
    assert tab.read_text().splitlines() == [
        "0 */4 * * * /home/claw/token.sh",
        f"0 22 * * * {N}/night_run.sh",
        f"0 7 * * * {N}/night_stop.sh",
        "15 7 * * * /home/claw/code/robot/infra/night_collect_local.sh",
        f"30 7 * * * {N}/plan_morning.sh",
        f"30 21 * * * {N}/ensure_queue.sh",
        f"30 5 * * * {N}/plan_morning.sh"]
