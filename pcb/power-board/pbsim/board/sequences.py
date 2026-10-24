"""Dynamic tests: each a timed sequence of inputs on the whole board, with
the response the design asks for (design record §5, §7; harness.md) checked
against the simulation.

Every sequence returns its failures (an empty list passes), the measured
numbers the report quotes, and saves its waveforms to
``build/power-board/fullsim/traces/<name>.npz`` for the figures.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from .board import Board, Esp32, Scenario, steps
from .campaign import OUT
from .ics import BqLimits, BqOtp
from .stress import describe, floating

T_WAKE = 0.05                  # POWER tap at 50 ms in every "switched on" run
_trapz = getattr(np, "trapezoid", None) or np.trapz    # numpy < 2 has only trapz
RAIL_UP = 0.80                 # rail settled by then (BST + soft start)
TRACE_KEYS = ("t", "pack", "servo", "logic", "railv", "pwr_int_n", "estop_n",
              "flt_n", "imon", "v3v3", "ts2", "ld", "rst", "en", "inp", "bst",
              "tmr", "q2_vgs", "q3_vgs", "q2_vds", "q3_vds", "i_q2", "i_q3",
              "i_pack", "u1_mode", "u1_dsg", "u2_gate", "u2_latch", "gd_vin")


def on(name, t_stop, **kw) -> Scenario:
    """Switched on by a 50 ms POWER tap; the ESP32 raises SERVO_EN at boot."""
    kw.setdefault("power_btn", steps((0, 0), (T_WAKE, 1), (T_WAKE + 0.05, 0)))
    kw.setdefault("esp", Esp32(servo_en=steps((0, 1))))
    return Scenario(name=name, t_stop=t_stop, **kw)


# ---------------------------------------------------------------- firmware --

class FwShutdown:
    """Design record §7, 'POWER pressed while on': on PWR_INT_N low the ESP32
    drops SERVO_EN (if asked), waits for the landing and the Pi's halt, then
    holds KILL high."""

    def __init__(self, kill_after=0.5, drop_servo_en=True, kill_for=None):
        self.kill_after = kill_after
        self.drop = drop_servo_en
        self.kill_for = kill_for
        self.t_press = None
        self.boot = None

    def __call__(self, t, v, b, lv):
        if b.rail3v3_since != self.boot:      # the ESP32 rebooted: fresh state
            self.boot = b.rail3v3_since
            self.t_press = None
        if self.t_press is None and v.get(b.net("/PWR_INT_N"), 3.3) < 0.8:
            self.t_press = t
            b.events.append((t, "ESP32 sees PWR_INT_N low"))
        if self.t_press is None:
            return lv
        out = dict(lv)
        if self.drop and t >= self.t_press + 0.05:
            out["sen"] = (0.0, False)
        tk = self.t_press + self.kill_after
        if t >= tk and (self.kill_for is None or t < tk + self.kill_for):
            out["kill"] = (1.0, False)
        return out


class FwStuckRail:
    """Design record §5.3: E-stop pressed (ESTOP_N high) but the rail still
    up → KILL. 'Still up' is judged ``window`` s after the press: SERVO+
    (RAIL_V × 57/10) above ``frac`` of the pack voltage (the INA219's reading,
    the General Driver's input)."""

    def __init__(self, window=2.0, frac=0.70):
        self.t_press = None
        self.fired = None
        self.window = window
        self.frac = frac

    def __call__(self, t, v, b, lv):
        out = dict(lv)
        pressed = v.get(b.net("/ESTOP_N"), 0) > 2.48
        if pressed and self.t_press is None:
            self.t_press = t
        if not pressed:
            self.t_press = None
        if (self.t_press is not None and self.fired is None
                and t - self.t_press >= self.window):
            servo = v.get(b.net("/RAIL_V"), 0) * 5.7
            if servo > self.frac * v.get("gd_vin", 0):
                self.fired = t
                b.events.append((t, "ESP32: E-stop pressed but rail up → KILL"))
        if self.fired is not None:
            out["kill"] = (1.0, False)
        return out


# ----------------------------------------------------------------- helpers --

def t_cross(r, key, level, t0=0.0, rising=True):
    return r.first(key, (lambda x: x >= level) if rising
                   else (lambda x: x <= level), t0)


def mode_at(r, t):
    return ["OFF", "SHUTDOWN", "BOOT", "NORMAL", "SOFTSHUT"][int(r.at("u1_mode", t))]


def save_trace(name, r):
    d = OUT / "traces"
    d.mkdir(parents=True, exist_ok=True)
    w = r.w
    n = len(w["t"])
    keep = np.arange(n)
    if n > 20000:     # keep every point near fast events, decimate the rest
        dt = np.diff(w["t"], prepend=w["t"][0])
        keep = np.unique(np.concatenate([np.nonzero(dt < 2e-6)[0],
                                         np.arange(0, n, n // 15000 + 1)]))
    np.savez_compressed(d / f"{name}.npz",
                        **{k: w[k][keep] for k in TRACE_KEYS if k in w})


# --------------------------------------------------------------- sequences --
# Each builder returns (Scenario, check(r, b) -> (fail list, metrics dict)).

SEQ = {}


def seq(fn):
    SEQ[fn.__name__] = fn
    return fn


def _tap(width):
    def build():
        sc = on(f"tap_{width*1e3:g}ms", 0.9,
                power_btn=steps((0, 0), (T_WAKE, 1), (T_WAKE + width, 0)))

        def check(r, b):
            t_up = t_cross(r, "pack", 10.0)
            m = {"t_pack_up_after_press_ms": None if t_up is None
                 else (t_up - T_WAKE) * 1e3}
            fail = []
            if t_up is None:
                fail.append(f"a {width*1e3:g} ms POWER tap did not switch the board on")
            elif not 0.25 < t_up - T_WAKE < 0.40:
                fail.append(f"PACK+ up {t_up - T_WAKE:.3f} s after the press "
                            "(DS §8.4: ~0.28 s)")
            return fail, m
        return sc, check
    return build


for _w in (0.002, 0.02, 0.2, 1.0):
    SEQ[f"power_tap_{_w*1e3:g}ms"] = _tap(_w)


@seq
def power_on_full_sequence():
    """POWER tap → FETs → General Driver → ESP32 → SERVO_EN → rail."""
    sc = on("power_on_full_sequence", 1.2)

    def check(r, b):
        t_pack = t_cross(r, "pack", 10.0)
        t_3v3 = t_cross(r, "v3v3", 3.0)
        t_bst = t_cross(r, "bst", 7.6)
        t_rail = t_cross(r, "servo", 0.9 * r.at("pack", 1.1))
        rail10 = t_cross(r, "servo", 0.1 * r.at("pack", 1.1))
        i_in = float(r.window("i_q3", 0.6, 1.2).max())
        m = {"t_pack_up_s": t_pack, "t_3v3_s": t_3v3, "t_bst_ok_s": t_bst,
             "t_rail_90_s": t_rail,
             "rail_10_90_ms": None if not (t_rail and rail10) else (t_rail - rail10) * 1e3,
             "rail_inrush_peak_A": i_in,
             "q2_turnon_peak_A": float(r.window("i_q2", 0.3, 0.4).max()),
             "pack_V": r.at("pack", 1.15), "servo_V": r.at("servo", 1.15),
             "railv_V": r.at("railv", 1.15), "q2_vgs_V": r.at("q2_vgs", 1.15),
             "q3_vgs_V": r.at("q3_vgs", 1.15), "bst_V": r.at("bst", 1.15)}
        fail = []
        if t_rail is None:
            fail.append("rail never came up")
        if i_in > 4.0:
            fail.append(f"rail inrush {i_in:.1f} A above the 4 A design margin")
        if m["q3_vgs_V"] < 8:
            fail.append("Q3 not fully enhanced")
        return fail, m
    return sc, check


@seq
def kill_pulse_short():
    """KILL high for 0.5 s: a reset only, the board stays on (DS §7.7.6)."""
    sc = on("kill_pulse_short", 2.0,
            esp=Esp32(servo_en=steps((0, 1)), kill=steps((0, 0), (1.0, 1), (1.5, 0))))

    def check(r, b):
        fail = []
        if r.window("pack", 1.0, 2.0).min() < 10:
            fail.append("PACK+ dropped on a 0.5 s KILL pulse")
        return fail, {"pack_min_V": float(r.window("pack", 1.0, 2.0).min())}
    return sc, check


def _shutdown(name, *, drop_servo_en, servos=True, lowv_ohms=None, t_stop=8.0,
              patch=None, power_again=None, extra=None, gd_iq=0.0):
    def build():
        kw = {}
        if not servos:
            kw["branch_n"] = {4: 0, 5: 0, 6: 0, 7: 0}
        btn = [(0, 0), (T_WAKE, 1), (T_WAKE + 0.05, 0), (1.0, 1), (1.1, 0)]
        if power_again:
            btn += [(power_again, 1), (power_again + 0.1, 0)]
        sc = on(name, t_stop, power_btn=steps(*btn),
                esp=Esp32(servo_en=steps((0, 1)),
                          firmware=FwShutdown(kill_after=0.5,
                                              drop_servo_en=drop_servo_en)),
                servo_lowv_ohms=lowv_ohms, patch=patch or {}, record_every=4,
                extra=list(extra or []), gd_iq_off_a=gd_iq, **kw)

        def check(r, b):
            kill = 1.0 + 0.5
            t_off = t_cross(r, "pack", 0.5, kill, rising=False)
            t_sd = r.first("u1_mode", lambda x: x == 1, kill)
            t_soft = r.first("u1_mode", lambda x: x == 4, kill)
            m = {"t_fets_off_after_kill_s": None if t_soft is None else t_soft - kill,
                 "t_pack_below_0.5V_after_kill_s": None if t_off is None else t_off - kill,
                 "t_SHUTDOWN_after_kill_s": None if t_sd is None else t_sd - kill,
                 "servo_V_at_fets_off": None if t_soft is None else r.at("servo", t_soft + 1e-3),
                 "servo_V_end": float(r.w["servo"][-1]),
                 "pack_V_end": float(r.w["pack"][-1]),
                 "mode_end": mode_at(r, t_stop)}
            fail = []
            if t_soft is None or not 0.99 < t_soft - kill < 1.02:
                fail.append("FETs did not open 1 s after KILL")
            if t_sd is None:
                fail.append(f"BQ76922 still not in SHUTDOWN {t_stop - kill:.1f} s "
                            f"after KILL (mode {m['mode_end']}, PACK+ held at "
                            f"{m['pack_V_end']:.2f} V by SERVO+ {m['servo_V_end']:.2f} V)")
            if power_again:
                t_on2 = t_cross(r, "pack", 10.0, power_again)
                m["power_again_switched_on"] = t_on2 is not None
                if t_on2 is None:
                    fail.append(f"POWER at {power_again:.1f} s (after KILL) ignored")
            return fail, m
        return sc, check
    return build


SEQ["shutdown_rail_dropped_first"] = _shutdown(
    "shutdown_rail_dropped_first", drop_servo_en=True)
SEQ["shutdown_rail_on_at_kill"] = _shutdown(
    "shutdown_rail_on_at_kill", drop_servo_en=False)
SEQ["shutdown_no_servos_bench"] = _shutdown(
    "shutdown_no_servos_bench", drop_servo_en=True, servos=False, t_stop=12.0,
    power_again=8.0)
SEQ["shutdown_no_servos_fix_bleed"] = _shutdown(
    "shutdown_no_servos_fix_bleed", drop_servo_en=True, servos=False,
    t_stop=12.0, power_again=8.0, patch={"R38": "4.7k", "R39": "1k"})
SEQ["shutdown_servos_resistive_lowv"] = _shutdown(
    "shutdown_servos_resistive_lowv", drop_servo_en=True, lowv_ohms=2000.0)
SEQ["shutdown_rail_dropped_first_gd_iq100u"] = _shutdown(
    "shutdown_rail_dropped_first_gd_iq100u", drop_servo_en=True, gd_iq=100e-6)


def _pk_bleed(b):
    """What-if: 4.7 kΩ from PACK+ to GND (2.7 mA, 34 mW while on; nothing in
    SHUTDOWN, where PACK+ is at 0 V)."""
    return [f"r_fix_pkbleed {b.net('PACK+')} 0 4.7k"]


BLEED = {"R38": "4.7k", "R39": "1k"}
SEQ["shutdown_no_servos_fix_both"] = _shutdown(
    "shutdown_no_servos_fix_both", drop_servo_en=True, servos=False,
    t_stop=12.0, power_again=8.0, patch=BLEED, extra=[_pk_bleed])
SEQ["shutdown_rail_dropped_first_fix_both"] = _shutdown(
    "shutdown_rail_dropped_first_fix_both", drop_servo_en=True, patch=BLEED,
    extra=[_pk_bleed])
SEQ["shutdown_rail_on_at_kill_fix_both"] = _shutdown(
    "shutdown_rail_on_at_kill_fix_both", drop_servo_en=False, patch=BLEED,
    extra=[_pk_bleed])


@seq
def shutdown_power_still_held():
    """KILL while POWER is still held: soft shutdown until release (TRM §7.5)."""
    sc = on("shutdown_power_still_held", 5.0,
            power_btn=steps((0, 0), (T_WAKE, 1), (T_WAKE + 0.05, 0), (1.0, 1), (3.5, 0)),
            esp=Esp32(servo_en=steps((0, 1)),
                      firmware=FwShutdown(kill_after=0.5)))

    def check(r, b):
        t_sd = r.first("u1_mode", lambda x: x == 1, 1.5)
        m = {"t_SHUTDOWN_s": t_sd, "released_s": 3.5}
        fail = []
        if t_sd is not None and t_sd < 3.5:
            fail.append("entered SHUTDOWN while TS2 was held low")
        if t_cross(r, "pack", 10.0, 2.6) is not None:
            fail.append("board re-woke while POWER was held after KILL")
        return fail, m
    return sc, check


@seq
def power_while_kill_held_usb():
    """ESP32 on USB holding KILL high while the board is off: a POWER press
    must not turn the board on (RST_SHUT holds the BQ76922 in reset)."""
    sc = Scenario(name="power_while_kill_held_usb", t_stop=2.5,
                  usb=steps((0, 1)),
                  esp=Esp32(kill=steps((0, 1)), servo_en=steps((0, 1))),
                  power_btn=steps((0, 0), (0.5, 1), (0.6, 0)))

    def check(r, b):
        fail = []
        pk = float(r.window("pack", 0.5, 2.5).max())
        if pk > 1.0:
            fail.append(f"PACK+ rose to {pk:.1f} V with KILL held")
        return fail, {"pack_max_V": pk, "mode_end": mode_at(r, 2.5)}
    return sc, check


def _estop(name, press, release, t_stop, **kw):
    def build():
        sc = on(name, t_stop, estop_a=steps((0, 1), (press, 0), (release, 1)),
                estop_b=steps((0, 1), (press, 0), (release, 1)),
                fine=[(press, press + 5e-3, 2e-6), (release, release + 1e-3, 5e-6)],
                **kw)

        def check(r, b):
            v0 = r.at("servo", press - 1e-3)
            t_off = t_cross(r, "servo", 0.1 * v0, press, rising=False)
            t_n = t_cross(r, "estop_n", 2.48, press)
            t_back = t_cross(r, "servo", 0.9 * v0, release)
            m = {"rail_V_before": v0,
                 "rail_below_10pct_after_press_ms": None if t_off is None else (t_off - press) * 1e3,
                 "q3_gate_off_after_press_us": None,
                 "estop_n_high_after_press_ms": None if t_n is None else (t_n - press) * 1e3,
                 "rail_90pct_after_release_ms": None if t_back is None else (t_back - release) * 1e3,
                 "pack_min_V": float(r.window("pack", press, t_stop).min()),
                 "v3v3_min_V": float(r.window("v3v3", press, t_stop).min())}
            tg = r.first("u2_gate", lambda x: x < 0.5, press)
            m["q3_gate_off_after_press_us"] = None if tg is None else (tg - press) * 1e6
            fail = []
            if tg is None or tg - press > 5e-3:
                fail.append("gate driver did not drop within 5 ms of the press")
            if t_n is None:
                fail.append("ESTOP_N did not go high")
            if m["pack_min_V"] < 10 or m["v3v3_min_V"] < 3.2:
                fail.append("logic power disturbed by the E-stop")
            if t_back is None:
                fail.append("rail did not return after release")
            return fail, m
        return sc, check
    return build


SEQ["estop_press_release_short"] = _estop("estop_press_release_short", 1.0, 1.5, 2.2)
SEQ["estop_press_release_long"] = _estop("estop_press_release_long", 1.0, 6.0, 6.8,
                                         record_every=4)


@seq
def estop_stuck_rail_bench_no_servos():
    """Bench, no servos: E-stop pressed. Firmware's 'rail still up' rule must
    not fire falsely while SERVO+ bleeds down (design §5.3)."""
    sc = on("estop_stuck_rail_bench_no_servos", 4.5,
            branch_n={4: 0, 5: 0, 6: 0, 7: 0},
            estop_a=steps((0, 1), (1.0, 0)), estop_b=steps((0, 1), (1.0, 0)),
            esp=Esp32(servo_en=steps((0, 1)), firmware=FwStuckRail()),
            record_every=4)

    def check(r, b):
        fired = r.event_times("rail up → KILL")
        t90 = t_cross(r, "servo", 5.0, 1.0, rising=False)
        m = {"false_kill_at_s": fired[0] if fired else None,
             "servo_below_5V_after_press_s": None if t90 is None else t90 - 1.0}
        fail = []
        if fired:
            fail.append(f"firmware's stuck-rail rule fired falsely "
                        f"{fired[0] - 1.0:.2f} s after the press: SERVO+ had "
                        f"not fallen below 70 % of the pack (it bleeds slowly)")
        return fail, m
    return sc, check


@seq
def estop_stuck_rail_bench_fix_bleed():
    sc, check = estop_stuck_rail_bench_no_servos()
    sc = replace(sc, name="estop_stuck_rail_bench_fix_bleed", patch=dict(BLEED),
                 esp=Esp32(servo_en=steps((0, 1)), firmware=FwStuckRail()))
    return sc, check


@seq
def estop_stuck_rail_with_servos():
    """The same rule with the servos plugged in (their electronics load SERVO+)."""
    sc, check = estop_stuck_rail_bench_no_servos()
    sc = replace(sc, name="estop_stuck_rail_with_servos",
                 branch_n={4: 6, 5: 3, 6: 6, 7: 2},
                 esp=Esp32(servo_en=steps((0, 1)), firmware=FwStuckRail()))
    return sc, check


def _overcurrent(name, amps, *, ocd_tol_mv=0.0, expect="cb", t_stop=1.4,
                 otp=None):
    def build():
        sc = on(name, t_stop, branch_a={4: steps((0, 0), (1.0, amps), (1.3, 0))},
                bq=BqLimits(ocd_tol_mv=ocd_tol_mv), otp=otp or BqOtp())

        def check(r, b):
            t_cb = r.event_times("U2 OC turn-off")
            t_ocd = r.event_times("U1 OCD1 trip")
            t_flt = t_cross(r, "flt_n", 0.8, 1.0, rising=False)
            m = {"branch_A": amps, "cb_trip_after_s": t_cb[0] - 1.0 if t_cb else None,
                 "flt_low_after_s": None if t_flt is None else t_flt - 1.0,
                 "ocd_trip_after_s": t_ocd[0] - 1.0 if t_ocd else None,
                 "pack_end_V": float(r.w["pack"][-1]),
                 "servo_end_V": float(r.w["servo"][-1]),
                 "imon_at_load_V": r.at("imon", 1.005)}
            fail = []
            if expect == "cb":
                if not t_cb:
                    fail.append("rail circuit breaker did not trip")
                if t_ocd:
                    fail.append("BQ76922 OCD tripped: the whole robot lost power "
                                "for a servo-rail overcurrent")
            elif expect == "none":
                if t_cb or t_ocd:
                    fail.append("a protection tripped below its threshold")
            return fail, m
        return sc, check
    return build


SEQ["rail_overcurrent_25A"] = _overcurrent("rail_overcurrent_25A", 25.0)
SEQ["rail_load_17A_nominal"] = _overcurrent("rail_load_17A_nominal", 17.0,
                                           expect="none")
SEQ["rail_load_17A_ocd_low_corner"] = _overcurrent(
    "rail_load_17A_ocd_low_corner", 17.0, ocd_tol_mv=-4.0, expect="none")
SEQ["rail_overcurrent_22A_ocd_low_corner"] = _overcurrent(
    "rail_overcurrent_22A_ocd_low_corner", 22.0, ocd_tol_mv=-4.0)


@seq
def rail_cb_latch_reset_by_servo_en():
    """After a CB trip, SERVO_EN low→high restarts the rail (TPS §8.3.3.2)."""
    sc = on("rail_cb_latch_reset_by_servo_en", 2.0,
            branch_a={4: steps((0, 0), (1.0, 25.0), (1.1, 0))},
            esp=Esp32(servo_en=steps((0, 1), (1.3, 0), (1.35, 1))))

    def check(r, b):
        latched = r.at("u2_latch", 1.2) == 1
        back = t_cross(r, "servo", 10.0, 1.35)
        fail = []
        if not latched:
            fail.append("no latch after the overcurrent")
        if back is None:
            fail.append("SERVO_EN toggle did not restart the rail")
        return fail, {"rail_back_after_toggle_ms": None if back is None
                      else (back - 1.35) * 1e3}
    return sc, check


@seq
def rail_ot_latch():
    """Q5 at 160 °C: U2 latches off (TPS §8.3.6); SERVO_EN toggling does not
    clear it; an E-stop press and release does."""
    sc = on("rail_ot_latch", 2.6, diode_sense=True, q5_temp_c=160.0,
            esp=Esp32(servo_en=steps((0, 1), (1.2, 0), (1.25, 1))),
            estop_a=steps((0, 1), (1.8, 0), (1.9, 1)))

    def check(r, b):
        t_ot = r.event_times("OT latch")
        m = {"ot_latch_at_s": t_ot[0] if t_ot else None,
             "diode_temp_read_C": b.u2.t_diode_c,
             "rail_after_servo_en_toggle_V": r.at("servo", 1.7),
             "rail_after_estop_cycle_V": r.at("servo", 2.55)}
        fail = []
        if not t_ot:
            fail.append("no over-temperature latch at 160 °C")
        if m["rail_after_servo_en_toggle_V"] > 1:
            fail.append("SERVO_EN toggle cleared the OT latch (it should not)")
        return fail, m
    return sc, check


@seq
def rail_ot_below_threshold():
    sc = on("rail_ot_below_threshold", 1.2, diode_sense=True, q5_temp_c=140.0)

    def check(r, b):
        fail = []
        if r.event_times("OT latch"):
            fail.append("OT latch at 140 °C (threshold 150 °C)")
        return fail, {"diode_temp_read_C": b.u2.t_diode_c}
    return sc, check


def _short(name, place, t_sc, *, t_stop, r_cell=5e-3, otp=None, dt=10e-9,
           window=200e-6, extra_fine=(), expect=None, max_trips=None, **kw):
    def build():
        fine = [(t_sc - 2e-6, t_sc + window, dt)] + list(extra_fine)
        sc = on(name, t_stop, shorts={place: steps((0, 0), (t_sc, 1))},
                r_cell=r_cell, fine=fine, otp=otp or BqOtp(), **kw)

        def check(r, b):
            w = lambda k: r.window(k, t_sc, t_sc + window)  # noqa: E731
            t_sc_trip = r.event_times("U2 SC trip")
            t_scd = r.event_times("U1 SCD trip")
            t_f = [t for t, e in r.events if "melted" in e]
            tt = r.window("t", t_sc, t_sc + window)
            pk = r.window("pack", t_sc, t_sc + window)
            dvdt = float(np.max(np.abs(np.diff(pk) / np.maximum(np.diff(tt), 1e-12)))) \
                if len(tt) > 2 else None
            m = {"place": place,
                 "u2_sc_trip_after_us": (t_sc_trip[0] - t_sc) * 1e6 if t_sc_trip else None,
                 "u1_scd_trip_after_us": (t_scd[0] - t_sc) * 1e6 if t_scd else None,
                 "fuse_melt": t_f,
                 "q3_peak_A": float(w("i_q3").max()), "q2_peak_A": float(w("i_q2").max()),
                 "pack_peak_A_R1": float(w("i_pack").max()),
                 "q2_vds_peak_V": float(w("q2_vds").max()),
                 "q3_vds_peak_V": float(w("q3_vds").max()),
                 "pack_min_V": float(w("pack").min()),
                 "pack_max_V": float(r.window("pack", t_sc, t_stop).max()),
                 "pack_dvdt_max_V_per_us": None if dvdt is None else dvdt * 1e-6,
                 "pack_end_V": float(r.w["pack"][-1]),
                 "scd_trips_total": len(t_scd),
                 "mode_end": mode_at(r, t_stop)}
            e_q2 = _trapz(np.clip(w("i_q2"), 0, None) * np.clip(w("q2_vds"), 0, None), tt)
            e_q3 = _trapz(np.clip(w("i_q3"), 0, None) * np.clip(w("q3_vds"), 0, None), tt)
            m["q2_energy_mJ"] = float(e_q2 * 1e3)
            m["q3_energy_mJ"] = float(e_q3 * 1e3)
            fail = []
            if expect == "rail":      # design §5.3 / record C5: Q3 clears it alone
                if not t_sc_trip:
                    fail.append("U2 short-circuit trip did not fire")
                if t_scd:
                    fail.append("BQ76922 SCD tripped: a servo-side short took "
                                "the logic down")
                if m["pack_end_V"] < 10:
                    fail.append("PACK+ did not stay up")
            elif expect == "main":    # the BQ76922 must open Q2
                if not t_scd and not t_f:
                    fail.append("nothing cleared a short on PACK+")
                if m["q2_peak_A"] > 400:
                    fail.append(f"Q2 peak {m['q2_peak_A']:.0f} A > 400 A pulsed rating")
                if m["q2_vds_peak_V"] > 30:
                    fail.append(f"Q2 VDS {m['q2_vds_peak_V']:.1f} V > 30 V")
            if max_trips is not None and len(t_scd) > max_trips:
                fail.append(f"the BQ76922 re-closed Q2 into the short "
                            f"{len(t_scd) - 1} times (SCD auto-recovery)")
            return fail, m
        return sc, check
    return build


SEQ["short_servo_lead_J4"] = _short("short_servo_lead_J4", "J4", 1.0,
                                    t_stop=1.05, window=60e-6, expect="rail")
SEQ["short_servo_lead_J4_stiff_pack"] = _short(
    "short_servo_lead_J4_stiff_pack", "J4", 1.0, t_stop=1.05, window=60e-6,
    r_cell=1.0e-3, expect="rail")
SEQ["short_servo_rail_board"] = _short("short_servo_rail_board", "SERVO+", 1.0,
                                       t_stop=1.05, window=60e-6, expect="rail")
SEQ["short_packplus_board"] = _short("short_packplus_board", "PACK+", 1.0,
                                     t_stop=1.05, window=200e-6, expect="main")
SEQ["short_packplus_board_stiff_pack"] = _short(
    "short_packplus_board_stiff_pack", "PACK+", 1.0, t_stop=1.05,
    window=200e-6, expect="main", r_cell=1.7e-3)
SEQ["short_packplus_persistent_default_otp"] = _short(
    "short_packplus_persistent_default_otp", "PACK+", 1.0, t_stop=12.0,
    window=200e-6, expect="main", max_trips=1, fine_on_dsg=400e-6,
    record_every=2)
SEQ["short_packplus_persistent_scdl1"] = _short(
    "short_packplus_persistent_scdl1", "PACK+", 1.0, t_stop=12.0,
    window=200e-6, expect="main", max_trips=1, fine_on_dsg=400e-6,
    otp=BqOtp(scdl_limit=1), record_every=2)
SEQ["short_logic_lead_J3"] = _short("short_logic_lead_J3", "J3", 1.0,
                                    t_stop=1.05, window=400e-6, expect="main")


@seq
def cuv_weak_cell_under_load():
    """One weak cell (40 mΩ, 3.05 V rest) under a 10 A get-up load trips CUV;
    the load goes away with the power; the cell recovers and the BQ76922
    closes Q2 again on its own (TRM: CUV recovery after Recovery:Time)."""
    sc = on("cuv_weak_cell_under_load", 7.0,
            cells=(steps((0, 3.40)), steps((0, 3.15)), steps((0, 3.40))),
            r_cells=(5e-3, 40e-3, 5e-3),
            branch_a={4: steps((0, 0), (1.0, 5.0), (1.6, 0)),
                      6: steps((0, 0), (1.0, 5.0), (1.6, 0))},
            record_every=4)
    return sc, _cuv_check


@seq
def ocd_trip_then_power_press():
    """An OCD trip (17 A rail load at the OCD threshold's low corner), then
    POWER pressed at 2.5 s: the design says POWER turns the robot on."""
    sc = on("ocd_trip_then_power_press", 4.0,
            bq=BqLimits(ocd_tol_mv=-4.0),
            branch_a={4: steps((0, 0), (1.0, 17.0), (1.2, 0))},
            power_btn=steps((0, 0), (T_WAKE, 1), (T_WAKE + 0.05, 0),
                            (2.5, 1), (2.6, 0)), record_every=4)

    def check(r, b):
        t_ocd = r.event_times("OCD1 trip")
        on2 = t_cross(r, "pack", 10.0, 2.5)
        m = {"ocd_trip_s": t_ocd[0] if t_ocd else None,
             "power_press_restored_s": on2, "mode_end": mode_at(r, 4.0)}
        fail = []
        if t_ocd and on2 is None:
            fail.append("after an OCD cut, POWER cannot switch the board back on "
                        "(BQ76922 stays in NORMAL with Q2 open; OCD recovery "
                        "needs charge current; the ESP32 is unpowered)")
        return fail, m
    return sc, check


def _cuv_check(r, b):
    t_cuv = r.event_times("CUV trip")
    t_rec = r.event_times("CUV recovered")
    t_on2 = t_cross(r, "pack", 9.0, (t_rec[0] if t_rec else 99))
    m = {"cuv_trip_s": t_cuv[0] if t_cuv else None,
         "cuv_recovered_s": t_rec[0] if t_rec else None,
         "pack_back_on_s": t_on2}
    fail = []
    if t_on2 is not None:
        fail.append(f"after a CUV cut the board switched itself back on at "
                    f"{t_on2:.2f} s, with no input")
    return fail, m


@seq
def ocd_trip_otp_time_recovery():
    """What-if (OTP): OCD Recovery Threshold −100 mA, so OCD recovers after
    Recovery:Time with no current (TRM 13.6.9.1's note): the board comes
    back by itself 3 s after the cut."""
    sc, check = ocd_trip_then_power_press()
    sc = replace(sc, name="ocd_trip_otp_time_recovery", t_stop=6.0,
                 otp=BqOtp(ocd_recovery_a=-0.1), power_btn=steps(
                     (0, 0), (T_WAKE, 1), (T_WAKE + 0.05, 0)))

    def check2(r, b):
        t_ocd = r.event_times("OCD1 trip")
        back = t_cross(r, "pack", 10.0, (t_ocd[0] + 0.1) if t_ocd else 99)
        m = {"ocd_trip_s": t_ocd[0] if t_ocd else None, "pack_back_s": back}
        fail = []
        if back is not None:
            fail.append(f"board switched itself back on {back - t_ocd[0]:.2f} s "
                        "after the OCD cut, with no input")
        return fail, m
    return sc, check2


@seq
def rail_load_17A_otp_ocd30():
    """What-if (OTP): OCD1 at 30 mV (30 A on R1), at its −4 mV corner: the
    17 A rail overload no longer takes the robot down."""
    sc, check = SEQ["rail_load_17A_ocd_low_corner"]()
    return replace(sc, name="rail_load_17A_otp_ocd30",
                   otp=BqOtp(ocd1_mv=30.0)), check


SEQ["rail_overload_25A_otp_ocd30"] = _overcurrent(
    "rail_overload_25A_otp_ocd30", 25.0, ocd_tol_mv=-4.0, otp=BqOtp(ocd1_mv=30.0))


@seq
def low_battery_ramp():
    """Cells fall from 3.4 V to 2.9 V over 6 s under a 1 A logic load:
    the BQ76922's own backstops (design §7) end in SHUTDOWN."""
    ramp = [(0, 3.4), (1.0, 3.4)] + [(1.0 + k * 0.25, 3.4 - k * 0.5 / 24)
                                      for k in range(1, 25)]
    sc = on("low_battery_ramp", 9.0, cells=(ramp, ramp, ramp), record_every=4)

    def check(r, b):
        t_sd = r.first("u1_mode", lambda x: x == 1, 1.0)
        m = {"t_shutdown_s": t_sd, "events": [e for t, e in r.events if "U1" in e][-6:]}
        fail = [] if t_sd is not None else ["did not reach SHUTDOWN on a flat pack"]
        return fail, m
    return sc, check


@seq
def low_battery_ramp_fix_both():
    sc, check = low_battery_ramp()
    return replace(sc, name="low_battery_ramp_fix_both", t_stop=16.0,
                   patch=dict(BLEED),
                   extra=[_pk_bleed]), check


@seq
def full_pack_cov():
    """A full pack at 4.21 V/cell: COV opens CHG only; PACK+ stays up."""
    c = steps((0, 4.21))
    sc = on("full_pack_cov", 2.0, cells=(c, c, c))

    def check(r, b):
        fail = []
        if r.window("pack", 0.5, 2.0).min() < 12.0:
            fail.append("a full pack lost PACK+")
        return fail, {"cov_events": [e for t, e in r.events if "COV" in e or "CHG" in e]}
    return sc, check


def _hotplug(name, xt, bal, t_stop=0.5, **kw):
    def build():
        sc = Scenario(name=name, t_stop=t_stop, xt60=xt, balance=bal,
                      fine=[(t[0], t[0] + 50e-6, 5e-9) for t in xt + bal
                            if 0 < t[0] < t_stop and t[1] > 0], **kw)

        def check(r, b):
            fail = []
            woke = r.event_times("wake by")
            if woke:
                fail.append(f"plugging in woke the board ({r.events})")
            pk = float(r.window("pack", 0, t_stop).max())
            vgs = float(r.window("q2_vgs", 0, t_stop).max())
            m = {"pack_max_V": pk, "q2_vgs_max_V": vgs,
                 "abs_max": b.mon.violations()}
            if pk > 1.0:
                fail.append(f"PACK+ glitched to {pk:.2f} V on plug-in")
            return fail, m
        return sc, check
    return build


SEQ["hotplug_xt60_after_balance"] = _hotplug(
    "hotplug_xt60_after_balance", steps((0, 0), (0.1, 1)), steps((0, 1)))
SEQ["hotplug_balance_after_xt60"] = _hotplug(
    "hotplug_balance_after_xt60", steps((0, 1)), steps((0, 0), (0.1, 1)))
SEQ["hotplug_both_at_once"] = _hotplug(
    "hotplug_both_at_once", steps((0, 0), (0.1, 1)), steps((0, 0), (0.1, 1)))
SEQ["hotplug_xt60_after_balance_usb"] = _hotplug(
    "hotplug_xt60_after_balance_usb", steps((0, 0), (0.1, 1)), steps((0, 1)),
    usb=steps((0, 1)))


@seq
def unplug_xt60_while_on():
    """Pulling the XT60 with the robot on and the balance lead still in."""
    sc = on("unplug_xt60_while_on", 1.5, xt60=steps((0, 1), (1.0, 0)),
            fine=[(1.0, 1.0002, 5e-9)])

    def check(r, b):
        return [], {"abs_max": b.mon.violations(),
                    "mode_end": mode_at(r, 1.5)}
    return sc, check


@seq
def usb_only_pack_unplugged_outputs_driven():
    """Pack unplugged, ESP32 on USB driving KILL and SERVO_EN high: nothing on
    the board may be back-powered."""
    sc = Scenario(name="usb_only_pack_unplugged_outputs_driven", t_stop=1.0,
                  xt60=steps((0, 0)), balance=steps((0, 0)), usb=steps((0, 1)),
                  esp=Esp32(kill=steps((0, 1)), servo_en=steps((0, 1))))

    def check(r, b):
        pk = float(r.window("pack", 0.5, 1.0).max())
        vs = float(r.window("servo", 0.5, 1.0).max())
        fail = []
        if pk > 0.3 or vs > 0.3:
            fail.append(f"back-powered: PACK+ {pk:.2f} V, SERVO+ {vs:.2f} V")
        return fail, {"pack_max_V": pk, "servo_max_V": vs,
                      "abs_max": b.mon.violations()}
    return sc, check


@seq
def jammed_power_button():
    """POWER stuck pressed: the board wakes; KILL then cannot complete SHUTDOWN
    while TS2 is held low (TRM §7.5), so the robot stays off-but-awake."""
    sc = Scenario(name="jammed_power_button", t_stop=5.0,
                  power_btn=steps((0, 0), (0.05, 1)),
                  esp=Esp32(servo_en=steps((0, 1)),
                            firmware=FwShutdown(kill_after=0.5)))

    def check(r, b):
        m = {"mode_end": mode_at(r, 5.0), "pack_end_V": float(r.w["pack"][-1]),
             "rewakes": len(r.event_times("wake by"))}
        return [], m
    return sc, check


@seq
def getup_peak_brownout():
    """A 12 A get-up peak for 2 s from a half-charged pack: logic stays up."""
    c = steps((0, 3.6))
    sc = on("getup_peak_brownout", 3.2, cells=(c, c, c), r_cell=10e-3,
            branch_a={4: steps((0, 0), (1.0, 4.3)), 6: steps((0, 0), (1.0, 4.3)),
                      5: steps((0, 0), (1.0, 1.7)), 7: steps((0, 0), (1.0, 1.7))},
            record_every=4)

    def check(r, b):
        m = {"pack_min_V": float(r.window("pack", 1.0, 3.0).min()),
             "gd_vin_min_V": float(r.window("gd_vin", 1.0, 3.0).min()),
             "en_min_V": float(r.window("en", 1.0, 3.0).min()),
             "imon_V": r.at("imon", 2.0),
             "i_q3_A": r.at("i_q3", 2.0)}
        fail = []
        if r.event_times("trip") or r.event_times("turn-off"):
            fail.append("a protection tripped during a get-up")
        return fail, m
    return sc, check


def all_sequences():
    return list(SEQ)


def run_named(name: str) -> dict:
    sc, check = SEQ[name]()
    b = Board(sc)
    r = b.run()
    fail, m = check(r, b)
    xt60_out = any(v < 0.5 for _, v in sc.xt60)
    sustained = [v for v in b.mon.violations(min_s=20e-6)
                 if not floating(v, xt60_out)]
    # shorts are the overstress being studied; their pins are reported, not failed
    if not name.startswith("short_"):
        for v in sustained:
            fail.append(describe(v))
    save_trace(name, r)
    return {"name": name, "fail": fail, "metrics": m,
            "transients": b.mon.violations(),
            "events": [(t, e) for t, e in r.events][:120]}
