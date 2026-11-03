"""Minimal KiCad S-expression reader/writer."""
import copy


class Q(str):
    """A quoted string atom."""


def parse(text):
    i, n = 0, len(text)
    stack = [[]]
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c == "(":
            stack.append([])
            i += 1
        elif c == ")":
            done = stack.pop()
            stack[-1].append(done)
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j:j + 2])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            stack[-1].append(Q("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            stack[-1].append(text[i:j])
            i = j
    return stack[0]


def fmt_atom(a):
    if isinstance(a, Q):
        s = str(a)
        if "\n" in s or ('"' in s and '\\"' not in s):
            s = s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
        return '"' + s + '"'
    if isinstance(a, float):
        s = f"{a:.4f}".rstrip("0").rstrip(".")
        return s if s not in ("-0", "") else "0"
    return str(a)


def dump(node, depth=0):
    if not isinstance(node, list):
        return fmt_atom(node)
    simple = all(not isinstance(x, list) for x in node)
    if simple:
        return "(" + " ".join(fmt_atom(x) for x in node) + ")"
    pad = "\t" * (depth + 1)
    parts = []
    head = []
    for x in node:
        if isinstance(x, list):
            break
        head.append(fmt_atom(x))
    out = "(" + " ".join(head)
    for x in node[len(head):]:
        out += "\n" + pad + dump(x, depth + 1)
    out += "\n" + "\t" * depth + ")"
    return out


def find(node, key):
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def find_all(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def lib_symbols(path):
    tree = parse(open(path, encoding="utf-8").read())[0]
    return {s[1]: s for s in find_all(tree, "symbol")}


def flatten(syms, name):
    """Resolve (extends ...) by copying the parent and overriding properties."""
    s = copy.deepcopy(syms[name])
    ext = find(s, "extends")
    if not ext:
        return s
    parent = flatten(syms, ext[1])
    pname = parent[1]
    out = copy.deepcopy(parent)
    out[1] = Q(name)
    child_props = {p[1]: p for p in find_all(s, "property")}
    for i, x in enumerate(out):
        if isinstance(x, list) and x and x[0] == "property" and x[1] in child_props:
            out[i] = child_props.pop(x[1])
    insert_at = max(i for i, x in enumerate(out) if isinstance(x, list) and x and x[0] == "property") + 1
    for p in child_props.values():
        out.insert(insert_at, p)
        insert_at += 1
    for x in out:
        if isinstance(x, list) and x and x[0] == "symbol" and x[1].startswith(pname + "_"):
            x[1] = Q(name + x[1][len(pname):])
    return out


def pins(sym):
    """number -> (x, y, angle, type) from every unit."""
    res = {}
    for unit in find_all(sym, "symbol"):
        for p in find_all(unit, "pin"):
            at = find(p, "at")
            num = find(p, "number")[1]
            res.setdefault(str(num), (float(at[1]), float(at[2]), float(at[3]), p[1]))
    return res
