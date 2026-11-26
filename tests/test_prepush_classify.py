"""The pre-push hook's ON/SKIP classification: an inert file (docs/, *.md, ...) that a
test reads must turn the Python gates on (2026-11-25: a docs-only edit of the servo map's
port column broke the test that parses it, and main went red); any other doc stays inert
(Tom, 2026-10-08). Drives .githooks/pre-push with PREPUSH_CLASSIFY_ONLY=1 on synthetic
commits in a throwaway clone, so no gate runs. File names are assembled from pieces so
this file does not itself count as a reader of them."""
import os
import subprocess

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HOOK = os.path.join(ROOT, ".githooks", "pre-push")
ZERO = "0" * 40

SERVO_MAP = "docs/servo-" + "map.md"
PORTS_TEST = "tests/test_current_budget_" + "ports.py"
PROMPT = "infra/night/planner_" + "prompt.md"
NIGHT_TEST = "tests/test_night_" + "runner.py"
UNREAD_DOC = "docs/b" + "om.md"
LONE_README = "zz_nobody_mentions/READ" + "ME.md"
TD_README = "READ" + "ME.md"          # test_td_pause reads the root one


# Run from inside the pre-push hook, git exports GIT_DIR (and friends) pointing at the
# pushing worktree; inherited, they would make every command below act on that worktree
# (commit, reset --hard) instead of the throwaway clone. Strip them all.
ENV = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}


def git(cwd, *args):
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                          text=True, env=ENV).stdout.strip()


@pytest.fixture(scope="module")
def clone(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("clone"))
    git(ROOT, "clone", "-q", "--local", "--no-checkout", ROOT, d)
    git(d, "checkout", "-q", "--detach", git(ROOT, "rev-parse", "HEAD"))
    git(d, "config", "user.email", "t@example.com")
    git(d, "config", "user.name", "t")
    assert os.path.realpath(git(d, "rev-parse", "--show-toplevel")) == os.path.realpath(d)
    return d


def classify(clone, path):
    """Commit a one-line edit of <path> on top of HEAD; return the hook's ON/SKIP output."""
    assert os.path.realpath(git(clone, "rev-parse", "--show-toplevel")) == os.path.realpath(clone)
    base = git(clone, "rev-parse", "HEAD")
    full = os.path.join(clone, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "a") as f:
        f.write("\nedit\n")
    git(clone, "add", "-A")
    git(clone, "commit", "-q", "-m", "edit " + path)
    sha = git(clone, "rev-parse", "HEAD")
    try:
        p = subprocess.run(["sh", HOOK], cwd=clone, capture_output=True, text=True,
                           input=f"refs/heads/x {sha} refs/heads/x {base}\n",
                           env={**ENV, "PREPUSH_CLASSIFY_ONLY": "1", "PREPUSH_ALL": "0"})
    finally:
        git(clone, "reset", "-q", "--hard", base)
    assert p.returncode == 0, p.stderr
    return p.stdout + p.stderr


def py_line(out):
    return [ln for ln in out.splitlines() if ln.startswith("Python gates:")][0]


@pytest.mark.parametrize("path,reader", [(SERVO_MAP, PORTS_TEST), (PROMPT, NIGHT_TEST),
                                         (TD_README, "tests/test_td_pause.py")])
def test_doc_read_by_a_test_turns_python_gates_on(clone, path, reader):
    out = classify(clone, path)
    assert py_line(out).startswith("Python gates: ON"), out
    assert reader in out and path in out


@pytest.mark.parametrize("path", [UNREAD_DOC, LONE_README, "docs/img/unread_" + "pic.png"])
def test_doc_no_test_reads_stays_inert(clone, path):
    assert py_line(classify(clone, path)).startswith("Python gates: SKIP")


def test_doc_read_does_not_turn_cpp_gates_on(clone):
    assert "C/C++ gates: SKIP" in classify(clone, SERVO_MAP)
