"""Continuous inputs swept across their range on the whole board: temperature
(leakage), PACK+ voltage, the ESP32's drive levels, the servo-rail current,
and the protection thresholds' datasheet tolerances.
"""
from __future__ import annotations

from .board import Board, Esp32, Scenario, steps
from .ics import BqLimits, BqOtp, TpsLimits
from .sequences import (SEQ, on, save_trace)

SWEEPS = {}


def _reg(name, fn):
    SWEEPS[name] = fn


# ---- W1: a switched-off board at temperature: does it wake itself? -------

def _hot(t_c, corner):
    def build():
        # wake comparators disabled: the run measures where TS2 and LD settle
        # in SHUTDOWN, which is then compared with DS 6.3's threshold range
        sc = Scenario(name=f"hot_off_{t_c}C_{corner}",
                      t_stop=0.05, temp_c=t_c, leak=corner,
                      bq=BqLimits(v_wake_ts2=-1.0, v_wake_ld=99.0))

        def run(r, b):
            ts2 = float(r.w["ts2"][-1])
            ld = float(r.w["ld"][-1])
            m = {"temp_C": t_c, "corner": corner,
                 "ts2_V": ts2, "ld_V": ld, "pack_V": float(r.w["pack"][-1]),
                 "wakes_if_threshold_1.1V": ts2 < 1.1,
                 "wakes_if_threshold_0.9V": ts2 < 0.9,
                 "wakes_if_threshold_0.7V": ts2 < 0.7,
                 "ld_wakes_if_threshold_0.8V": ld > 0.8}
            return m
        return sc, run
    return build


for _c in ("typ", "max"):
    # 70 °C is left out: ngspice's operating point does not converge there
    for _t in (-20, 0, 10, 25, 40, 50, 60, 65, 75, 80, 85, 95, 105):
        _reg(f"hot_off_{_t}C_{_c}", _hot(_t, _c))


def _pressed_level(t_c):
    """POWER pressed on a switched-off board: TS2's level against the wake
    threshold, at temperature."""
    def build():
        sc = Scenario(name=f"pressed_{t_c}C",
                      t_stop=0.02, temp_c=t_c,
                      power_btn=steps((0, 0), (0.01, 1)),
                      bq=BqLimits(v_wake_ts2=-1.0, v_wake_ld=99.0),
                      usb=steps((0, 1)))

        def run(r, b):
            return {"temp_C": t_c,
                    "ts2_pressed_V": float(r.w["ts2"][-1]),
                    "pwr_int_n_pressed_V": float(r.w["pwr_int_n"][-1]),
                    "ts2_released_V": r.at("ts2", 0.009)}
        return sc, run
    return build


for _t in (-20, 0, 25, 60, 85):
    _reg(f"pressed_{_t}C", _pressed_level(_t))


# ---- W2: PACK+ voltage: the rail's UVLO on the drawn divider ---------------

def _uvlo(direction, tps=None, tag=""):
    def build():
        if direction == "down":
            ramp = [(0, 3.4), (1.0, 3.4), (5.0, 2.2)]
        else:
            ramp = [(0, 2.2), (1.0, 2.2), (5.0, 3.4)]
        # piecewise-linear cells: many small steps
        cells = [(0, ramp[0][1])]
        for k in range(1, 81):
            t = 1.0 + k * 0.05
            v = ramp[1][1] + (ramp[2][1] - ramp[1][1]) * k / 80
            cells.append((t, v))
        otp = BqOtp(cuv_v=1.5, shutdown_stack_v=0.0)
        sc = on(f"uvlo_{direction}{tag}", 5.2, cells=(cells,) * 3, otp=otp,
                tps=tps or TpsLimits(), record_every=2)

        def run(r, b):
            if direction == "down":
                t = r.first("u2_gate", lambda x: x < 0.5, 1.0)
            else:
                t = r.first("u2_gate", lambda x: x > 0.5, 1.0)
            m = {"direction": direction, "tag": tag,
                 "pack_at_switch_V": None if t is None else r.at("pack", t),
                 "en_at_switch_V": None if t is None else r.at("en", t)}
            return m
        return sc, run
    return build


_reg("uvlo_down", _uvlo("down"))
_reg("uvlo_up", _uvlo("up"))
_reg("uvlo_down_min", _uvlo("down", TpsLimits(uvlor=1.16, uvlof=1.10), "_min"))
_reg("uvlo_down_max", _uvlo("down", TpsLimits(uvlor=1.20, uvlof=1.13), "_max"))


# ---- W3: the ESP32's drive levels into INP and RST_SHUT ----------------------

def _drive(sig, frac):
    def build():
        e = Esp32(servo_en=steps((0, frac if sig == "sen" else 1.0)),
                  kill=steps((0, 0), (1.0, frac)) if sig == "kill" else steps((0, 0)))
        sc = on(f"drive_{sig}_{frac:.2f}", 2.3 if sig == "kill" else 1.0, esp=e,
                record_every=4)

        def run(r, b):
            if sig == "sen":
                return {"signal": "SERVO_EN", "esp_V": 3.3 * frac,
                        "inp_V": r.at("inp", 0.95),
                        "rail_on": r.at("servo", 0.95) > 10}
            return {"signal": "KILL", "esp_V": 3.3 * frac,
                    "rst_V": r.at("rst", 1.5),
                    "shut_down": bool(r.event_times("shutdown sequence"))}
        return sc, run
    return build


for _f in (0.30, 0.45, 0.55, 0.65, 0.80, 1.00):
    _reg(f"drive_sen_{_f:.2f}", _drive("sen", _f))
for _f in (0.15, 0.25, 0.35, 0.45, 0.80, 1.00):
    _reg(f"drive_kill_{_f:.2f}", _drive("kill", _f))


# ---- W4: rail current: IMON and the circuit breaker --------------------------

def _rail_i(amps):
    def build():
        sc = on(f"rail_current_{amps:g}A", 1.15,
                branch_a={4: steps((0, 0), (1.0, amps))}, record_every=2)

        def run(r, b):
            return {"branch_A": amps, "i_q3_A": r.at("i_q3", 1.008),
                    "imon_V": r.at("imon", 1.008),
                    "imon_per_A": r.at("imon", 1.008) / max(r.at("i_q3", 1.008), 1e-6),
                    "cb_trip": bool(r.event_times("OC turn-off")),
                    "cb_after_ms": (r.event_times("OC turn-off")[0] - 1.0) * 1e3
                    if r.event_times("OC turn-off") else None}
        return sc, run
    return build


for _a in (0.0, 2.0, 5.0, 10.0, 15.0, 18.5, 19.0, 19.5, 20.0, 20.5, 22.0, 30.0):
    _reg(f"rail_current_{_a:g}A", _rail_i(_a))


# ---- W5: who acts first on a rail overload, over the thresholds' tolerances --

def _select(amps, ocd_tol, iwrn):
    def build():
        sc = on(f"select_{amps:g}A_ocd{ocd_tol:+g}mV_iwrn{iwrn:.3f}", 1.25,
                branch_a={4: steps((0, 0), (1.0, amps))},
                bq=BqLimits(ocd_tol_mv=ocd_tol), tps=TpsLimits(iwrn_ref=iwrn),
                record_every=2)

        def run(r, b):
            cb = r.event_times("OC turn-off")
            ocd = r.event_times("OCD1 trip")
            first = "none"
            if cb or ocd:
                first = "rail breaker" if (cb and (not ocd or cb[0] < ocd[0])) else "BQ76922 OCD (whole robot off)"
            return {"rail_A": amps, "ocd_tol_mV": ocd_tol, "iwrn_ref_V": iwrn,
                    "pack_A": r.at("i_pack", 1.005), "first": first}
        return sc, run
    return build


for _a in (16.0, 17.0, 18.0, 19.0, 21.0, 25.0):
    for _o in (-4.0, 0.0, 4.0):
        for _iw in (1.136, 1.19, 1.224):     # TPS 6.5 V(SNS_WRN) 29.2/30.6/31.5 mV
            _reg(f"select_{_a:g}A_ocd{_o:+g}_iwrn{_iw:.3f}", _select(_a, _o, _iw))


def all_sweeps():
    return list(SWEEPS)


def run_named(name: str) -> dict:
    sc, fn = SWEEPS[name]()
    b = Board(sc)
    r = b.run()
    out = fn(r, b)
    if isinstance(out, tuple):          # a sequence's check (what-ifs)
        fail, m = out
        save_trace(name, r)
    else:
        fail, m = [], out
    return {"name": name, "fail": fail, "metrics": m,
            "transients": b.mon.violations(),
            "events": [(t, e) for t, e in r.events][:80]}
