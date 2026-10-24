"""Markdown tables from the campaign's results, for the simulation record:

    python -m pbsim.board.summarize > build/power-board/fullsim/tables.md
"""
from __future__ import annotations

import collections
import json

from .campaign import OUT


def load(suite):
    return {r["name"]: r for r in json.loads((OUT / f"{suite}.json").read_text())}


def truth():
    r = list(load("truth").values())
    n = len(r)
    nfail = sum(1 for x in r if x["fail"])
    print(f"### Truth table: {n} cases, {n - nfail} pass, {nfail} fail\n")
    keys = [("pack_on", "PACK+ / LOGIC_OUT up"), ("v3v3_on", "+3V3_HOST up"),
            ("servo_on", "SERVO+ / RAIL_V up")]
    print("| expected output | cases expecting it up | cases expecting it off |")
    print("|---|---|---|")
    for k, lab in keys:
        up = sum(1 for x in r if x["expected"][k])
        print(f"| {lab} | {up} | {n - up} |")
    for k, lab in (("pwr_int_n", "PWR_INT_N"), ("estop_n", "ESTOP_N")):
        hi = sum(1 for x in r if x["expected"][k] is True)
        lo = sum(1 for x in r if x["expected"][k] is False)
        na = n - hi - lo
        print(f"| {lab} high / low / ESP32 unpowered | {hi} / {lo} | {na} |")
    print()
    on = [x["measured"] for x in r if x["expected"]["servo_on"]]
    pk = [x["measured"] for x in r if x["expected"]["pack_on"]]
    off = [x["measured"] for x in r if not x["expected"]["pack_on"]]
    rng = lambda xs, k: f"{min(m[k] for m in xs):.3f}–{max(m[k] for m in xs):.3f}"  # noqa: E731
    print(f"- PACK+ when up: {rng(pk, 'pack')} V; LOGIC_OUT {rng(pk, 'logic')} V; "
          f"when off: {rng(off, 'pack')} V")
    print(f"- SERVO+ when up: {rng(on, 'servo')} V; RAIL_V {rng(on, 'railv')} V; "
          f"IMON {rng(on, 'imon')} V; Q3 VGS {rng(on, 'q3_vgs')} V; "
          f"BST−SRC {rng(on, 'bst')} V")
    tr = collections.Counter((v["check"]) for x in r for v in x["transients"])
    mx = {}
    for x in r:
        for v in x["transients"]:
            a = mx.setdefault(v["check"], [0, 0, 0])
            a[0] = min(a[0], v["lo"])
            a[1] = max(a[1], v["hi"])
            a[2] = max(a[2], v["outside_s"])
    print("- brief excursions past an absolute maximum (all under 20 µs):")
    for k, c in tr.items():
        lo, hi, d = mx[k]
        print(f"  - {k}: {c} cases, {lo:.2f} V / {hi:.2f} V, at most {d*1e6:.1f} µs")
    print()


def seq():
    r = load("seq")
    print("### Sequences\n")
    print("| sequence | result | failures / notes |")
    print("|---|---|---|")
    for name, x in r.items():
        st = "error" if "error" in x else ("**fail**" if x["fail"] else "pass")
        print(f"| `{name}` | {st} | {'; '.join(x['fail'])[:400]} |")
    print()
    print("#### Measured numbers\n")
    for name, x in r.items():
        m = {k: (round(v, 4) if isinstance(v, float) else v)
             for k, v in x.get("metrics", {}).items()
             if k not in ("abs_max", "events", "cov_events")}
        print(f"- `{name}`: {json.dumps(m, default=str)}")
    print()


def sweeps():
    r = load("sweep")
    print("### Sweeps\n")
    print("#### Switched-off board at temperature: TS2 (V) and LD (V)\n")
    print("| °C | as drawn, typ | as drawn, max | fix, typ | fix, max | LD typ / max |")
    print("|---|---|---|---|---|---|")
    temps = sorted({x["metrics"]["temp_C"] for k, x in r.items() if k.startswith("hot_off_")})
    for t in temps:
        def g(c, f, key="ts2_V"):
            k = f"hot_off_{t}C_{c}{'_fix' if f else ''}"
            return r[k]["metrics"][key] if k in r else float("nan")
        print(f"| {t} | {g('typ', 0):.2f} | {g('max', 0):.2f} | {g('typ', 1):.2f} | "
              f"{g('max', 1):.2f} | {g('typ', 0, 'ld_V'):.2f} / {g('max', 0, 'ld_V'):.2f} |")
    print("\n#### POWER pressed: levels (V)\n")
    print("| °C | TS2, drawn | PWR_INT_N, drawn | TS2, fix | PWR_INT_N, fix |")
    print("|---|---|---|---|---|")
    for t in (-20, 0, 25, 60, 85):
        a = r.get(f"pressed_{t}C", {}).get("metrics", {})
        b = r.get(f"pressed_{t}C_fix", {}).get("metrics", {})
        print(f"| {t} | {a.get('ts2_pressed_V', float('nan')):.3f} | "
              f"{a.get('pwr_int_n_pressed_V', float('nan')):.3f} | "
              f"{b.get('ts2_pressed_V', float('nan')):.3f} | "
              f"{b.get('pwr_int_n_pressed_V', float('nan')):.3f} |")
    print("\n#### Rail UVLO on the drawn divider\n")
    for k in ("uvlo_down", "uvlo_up", "uvlo_down_min", "uvlo_down_max"):
        if k in r:
            m = r[k]["metrics"]
            print(f"- `{k}`: PACK+ {m['pack_at_switch_V']:.3f} V, EN/UVLO {m['en_at_switch_V']:.3f} V")
    print("\n#### ESP32 drive levels\n")
    print("| signal | ESP32 pin (V) | IC pin (V) | result |")
    print("|---|---|---|---|")
    for k, x in sorted(r.items()):
        if k.startswith("drive_"):
            m = x["metrics"]
            pin = m.get("inp_V", m.get("rst_V"))
            res = m.get("rail_on", m.get("shut_down"))
            what = ("rail on" if res else "rail off") if m["signal"] == "SERVO_EN" else \
                ("shutdown started" if res else "no shutdown")
            print(f"| {m['signal']} | {m['esp_V']:.2f} | {pin:.3f} | {what} |")
    print("\n#### Rail current: IMON and the breaker\n")
    print("| branch A | Q3 A | IMON V | V/A | breaker |")
    print("|---|---|---|---|---|")
    for k, x in sorted(r.items(), key=lambda kv: kv[1]["metrics"].get("branch_A", -1)):
        if k.startswith("rail_current_"):
            m = x["metrics"]
            cb = f"trips after {m['cb_after_ms']:.1f} ms" if m["cb_trip"] else "holds"
            print(f"| {m['branch_A']:g} | {m['i_q3_A']:.2f} | {m['imon_V']:.3f} | "
                  f"{m['imon_per_A']:.4f} | {cb} |")
    print("\n#### Who acts first on a rail overload (OCD1 20 mV ± 4 mV × IWRN spread)\n")
    print("| rail A | OCD −4 mV | OCD nominal | OCD +4 mV |")
    print("|---|---|---|---|")
    for a in (16, 17, 18, 19, 21, 25):
        cells = []
        for o in (-4, 0, 4):
            firsts = sorted({x["metrics"]["first"] for k, x in r.items()
                             if k.startswith(f"select_{a}A_ocd{o:+g}_")})
            cells.append(" / ".join(firsts))
        print(f"| {a} | " + " | ".join(cells) + " |")
    print("\n#### What-ifs\n")
    for k, x in r.items():
        if k.startswith("fix_"):
            m = {kk: (round(v, 4) if isinstance(v, float) else v)
                 for kk, v in x["metrics"].items() if kk != "abs_max"}
            print(f"- `{k}`: fail={x['fail']} {json.dumps(m, default=str)}")
    print()


if __name__ == "__main__":
    for f in (truth, seq, sweeps):
        try:
            f()
        except FileNotFoundError as e:
            print(f"(missing: {e})\n")
