"""Audit `vendor/csd17556q5b_ngspice.lib` against TI's release.

`vendor/CSD17556Q5B_TI_ORIGINAL.lib` is byte-identical to TI's SLPM066;
`vendor/csd17556q5b_ngspice.lib` is the hand rewrite that Debian
ngspice-42 parses (see vendor/README.md for why). The rewrite may differ
from TI's file in exactly two ways:

  1. TI's `.PARAM` constants are gone and every `{expression}` that used
     them carries the evaluated number instead;
  2. the NMOS model reads `LEVEL = 8 VERSION = 3.3` where TI's reads
     `LEVEL = 7 VERSION = 3.2`.

`rewrite_deltas()` evaluates TI's `.PARAM` expressions, substitutes them,
and compares the two files token by token (comments, whitespace and `+`
continuation breaks ignored; numbers compared as numbers). It returns
every difference; `test_ngsmoke.py` asserts the list is exactly the two
keyword changes. Run as a script, it prints the deltas and both SHA-256s
(of the raw bytes, which is what `fetmodel.py` pins).

It is an audit, not a generator: the rewrite stays a reviewed file.
"""
from __future__ import annotations

import ast
import operator
import re
import sys
from hashlib import sha256
from pathlib import Path

HERE = Path(__file__).resolve().parent
TI_ORIGINAL = HERE / "vendor" / "CSD17556Q5B_TI_ORIGINAL.lib"
REWRITE = HERE / "vendor" / "csd17556q5b_ngspice.lib"

INTENDED = [("LEVEL", "7", "8"), ("VERSION", "3.2", "3.3")]

_OPS = {ast.Add: operator.add, ast.Sub: operator.sub,
        ast.Mult: operator.mul, ast.Div: operator.truediv}


def _eval(expr: str, env: dict[str, float]) -> float:
    """Arithmetic over numbers and known .PARAM names; nothing else."""
    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in env:
            return env[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](ev(node.left), ev(node.right))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -ev(node.operand)
        raise ValueError(f"unsupported .PARAM expression: {expr!r}")
    return ev(ast.parse(expr, mode="eval"))


def _logical_lines(text: str) -> list[str]:
    """Join `+` continuations; drop comments and blank lines."""
    out: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("*"):
            continue
        if line.startswith("+"):
            out[-1] += " " + line[1:]
        else:
            out.append(line)
    return out


def _tokens(line: str) -> list[str]:
    return re.findall(r"\{[^}]*\}|[^\s=(){}]+|[=()]", line)


def _same(a: str, b: str) -> bool:
    if a == b:
        return True
    try:
        fa, fb = float(a), float(b)
    except ValueError:
        return a.upper() == b.upper()
    return abs(fa - fb) <= 1e-9 * max(abs(fa), abs(fb))


def ti_params(ti_text: str) -> dict[str, float]:
    env: dict[str, float] = {}
    for line in _logical_lines(ti_text):
        m = re.match(r"(?i)\.PARAM\s+(\w+)\s+(.+)$", line)
        if m:
            name, val = m.group(1), m.group(2).strip()
            env[name] = _eval(val.strip("{}"), env)
    return env


def rewrite_deltas(ti_text: str | None = None,
                   rw_text: str | None = None) -> list[tuple[str, str, str]]:
    """(context, TI token, rewrite token) for every difference."""
    ti_text = ti_text if ti_text is not None else TI_ORIGINAL.read_text()
    rw_text = rw_text if rw_text is not None else REWRITE.read_text()
    env = ti_params(ti_text)
    ti_lines = [ln for ln in _logical_lines(ti_text)
                if not re.match(r"(?i)\.PARAM\b", ln)]
    rw_lines = _logical_lines(rw_text)
    deltas: list[tuple[str, str, str]] = []
    if len(ti_lines) != len(rw_lines):
        deltas.append(("line count", str(len(ti_lines)), str(len(rw_lines))))
    for ti_line, rw_line in zip(ti_lines, rw_lines):
        ti_tok = [repr(_eval(t[1:-1], env)) if t.startswith("{") else t
                  for t in _tokens(ti_line)]
        rw_tok = _tokens(rw_line)
        if len(ti_tok) != len(rw_tok):
            deltas.append((ti_line[:40], ti_line, rw_line))
            continue
        for i, (a, b) in enumerate(zip(ti_tok, rw_tok)):
            if not _same(a, b):
                key = ti_tok[i - 2] if i >= 2 and ti_tok[i - 1] == "=" \
                    else ti_line[:40]
                deltas.append((key, a, b))
    return deltas


def main() -> int:
    print(f"TI original sha256:  {sha256(TI_ORIGINAL.read_bytes()).hexdigest()}")
    print(f"ngspice rewrite sha: {sha256(REWRITE.read_bytes()).hexdigest()}")
    env = ti_params(TI_ORIGINAL.read_text())
    print(f"TI .PARAM values: {env}")
    deltas = rewrite_deltas()
    print("deltas (context, TI, rewrite):")
    for d in deltas:
        print(f"  {d}")
    unintended = [d for d in deltas if d not in INTENDED]
    if unintended:
        print(f"UNINTENDED: {unintended}")
        return 1
    print("only the intended LEVEL/VERSION change")
    return 0


if __name__ == "__main__":
    sys.exit(main())
