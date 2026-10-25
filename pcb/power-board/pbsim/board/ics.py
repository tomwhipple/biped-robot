"""Pin-level models of U1 (TI BQ76922) and U2 (TI TPS48111-Q1).

Each IC is split the way the silicon is: its pins' analog behaviour is
ngspice elements on the board's real nets (pull-ups, current sources, gate
drivers, the current-sense amplifier), switched by EXTERNAL control voltages;
its digital behaviour (comparators sampled at accepted time points, timers,
latches, modes) is a Python state machine that reads the pin voltages and
sets those controls. Every number cites its source:

- BQ76922: datasheet SLUSE86A (DS), technical reference manual SLUUCG7 (TRM).
- TPS48111-Q1: datasheet SLUSEE5E (TPS).

Where a document leaves a behaviour unstated the model makes a choice and
says so in ``ASSUMPTIONS``; the report lists them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

ASSUMPTIONS: list[str] = [
    "BQ76922 boots into SHUTDOWN when BAT first rises past VPORA (TI EVM guide "
    "SLVU957A §3 step 7: apply the supply, then press WAKE).",
    "BQ76922 TS2 wake needs TS2 below V(WAKEONTS2) for 1 ms; DS gives no "
    "minimum width.",
    "BQ76922 DSG turn-off pull-down modelled as 30 Ω to VSS while V(DSG) > "
    "V(LD), for 200 µs (DS §8.5 gives the behaviour, not the strength).",
    "BQ76922 charge pump: 11 V above BAT through 40 kΩ into C(CP1), which "
    "reproduces DS §8.2's ~60 ms to 11 V on 470 nF; CHG/DSG on-switches "
    "2.6 kΩ (DS 6.24 t(FET_ON) 21 µs into 20 nF).",
    "BQ76922 SLEEP mode is disabled in OTP (Power Config[SLEEP]=0).",
    "BQ76922 while RST_SHUT is high: FETs hold, boot and protection timers "
    "do not advance (DS §7.7.6 resets 'most of the digital logic').",
    "BQ76922 in soft shutdown (DS §7.8.5, TRM §7.5) keeps the TS2 wake bias "
    "on and returns to boot when LD rises again after falling.",
    "TPS48111 INP / INP_G internal pull-downs taken as 1 MΩ (TPS Table 5-1: "
    "'internal pull-down', no value).",
    "TPS48111 between V(ENF) and V(UVLOR) is active with the gate held off "
    "(UVLO), charge pump running.",
    "TPS48111 IWRN internal scaling (pin carries 0.1 × the sense-amp current, "
    "1.19 V reference) chosen to reproduce TPS Eq. 6 exactly; the pin voltage "
    "itself is not specified.",
    "TPS48111 PD pulls the gate to SRC in shutdown and whenever the gate "
    "driver is not enabled.",
]


def ramp(t: float, t0: float, v0: float, v1: float, tr: float) -> float:
    if t <= t0:
        return v0
    if t >= t0 + tr:
        return v1
    return v0 + (v1 - v0) * (t - t0) / tr


class Ctl:
    """A control voltage that moves linearly over ``tr`` when set."""

    __slots__ = ("v0", "v1", "t0", "tr")

    def __init__(self, v: float = 0.0, tr: float = 1e-6):
        self.v0 = self.v1 = v
        self.t0 = -1.0
        self.tr = tr

    def set(self, t: float, v: float, tr: float | None = None):
        if v == self.v1:
            return
        self.v0 = self.at(t)
        self.v1 = v
        self.t0 = t
        if tr is not None:
            self.tr = tr

    def at(self, t: float) -> float:
        return ramp(t, self.t0, self.v0, self.v1, self.tr)

    def busy_until(self) -> float:
        return self.t0 + self.tr


def _sig(x: str, w: str = "0.02") -> str:
    """Smooth 0→1 step of the expression x (width w volts)."""
    return f"(0.5+0.5*tanh(({x})/{w}))"


# --------------------------------------------------------------------------
# BQ76922
# --------------------------------------------------------------------------

@dataclass
class BqOtp:
    """The OTP image the simulation assumes (TRM §13 names; values in
    engineering units). Defaults: the design record §5.5 and the existing
    pbsim constants where the design names a value, TI's defaults otherwise.
    """
    fet_en: bool = True                 # Mfg Status Init[FET_EN]
    cuv_v: float = 59 * 0.0506          # 2.985 V
    cuv_delay_s: float = 0.0066 + 74 * 0.0033    # TRM default 74 → 250.8 ms
    cuv_hys_v: float = 2 * 0.0506
    cov_v: float = 83 * 0.0506          # 4.200 V (existing pbsim)
    cov_delay_s: float = 0.0066 + 74 * 0.0033
    cov_hys_v: float = 2 * 0.0506
    ocd1_mv: float = 20.0               # 20 A on R1 = 1 mΩ
    ocd1_delay_s: float = 0.0066 + 14 * 0.0033   # 52.8 ms (existing 50 ms)
    ocd_recovery_a: float = 0.200       # TRM default: charge current ≥ 200 mA
    scd_mv: float = 150.0
    scd_delay_setting: int = 1          # 1 = fastest
    scd_recovery_s: float = 5.0         # TRM default
    scdl_limit: int = 0                 # 0 = SCDL disabled (TRM default)
    recovery_s: float = 3.0             # Protections:Recovery:Time default
    shutdown_stack_v: float = 9.0       # design record §7 backstop 1
    shutdown_cell_v: float = 0.0        # disabled (TRM default)
    low_v_shutdown_delay_s: float = 1.0
    fet_off_delay_s: float = 0.0
    shutdown_cmd_delay_s: float = 0.0
    body_diode_a: float = 0.050
    ts2_used: bool = False              # TS2 Config default 0x00 (unused)


@dataclass
class BqLimits:
    """Datasheet limits (DS) with the corner the run uses."""
    v_wake_ts2: float = 0.9     # DS 6.3: 0.7–1.1 V
    v_wake_ld: float = 1.45     # DS 6.3: 0.8 / 1.45 / 2.25 V
    v_pora: float = 3.5         # DS 6.3: 3–4 V rising
    v_pora_hys: float = 0.225
    v_reg18: float = 1.8
    t_boot_fets_s: float = 0.278   # DS §8.4 note: FETs ~278 ms after wake
    ts2_wake_min_s: float = 1e-3
    scd_tol: float = 0.0        # fraction, ±0.35 (DS 6.25 V(SCD_ACC))
    ocd_tol_mv: float = 0.0     # ±4 mV at 20 mV (DS 6.25 V(OC_ACC))


class Bq76922:
    """U1. Node names are the board's nets for each pin number."""

    PREFIX = "u1"

    def __init__(self, pin: dict[int, str], otp: BqOtp, lim: BqLimits,
                 r_sense: float):
        self.p = pin
        self.otp = otp
        self.lim = lim
        self.r_sense = r_sense
        self.mode = "OFF"            # OFF, SHUTDOWN, BOOT, NORMAL, SOFTSHUT
        self.t_mode = 0.0
        self.ctl = {k: Ctl(0.0) for k in (
            "reg18", "ts1", "ts2", "ldpd", "cp", "dsg", "dsgoff", "chg",
            "chgoff", "ibat")}
        self.ctl["cp"].tr = 1e-3
        self.dsg_on = False
        self.chg_on = False
        self.faults: dict[str, float] = {}       # name -> time set
        self.pending: dict[str, float] = {}      # name -> time condition began
        self.recover: dict[str, float] = {}
        self.ts2_low_since: float | None = None
        self.ld_fell = False
        self.rst_high_since: float | None = None
        self.rst_prev = False
        self.lowv_since: float | None = None
        self.shut_seq_at: float | None = None
        self.scdl_count = 0
        self.dsgoff_until = -1.0
        self.next_fet_eval = math.inf
        self.events: list[tuple[float, str]] = []
        self.cells = (0.0, 0.0, 0.0)
        self.i_pack = 0.0            # +discharge, from SRN−SRP
        self.max_scd_mv = 0.0

    # ---------------- analog pins ----------------
    def spice(self) -> list[str]:
        p, x = self.p, "x_u1_"
        vss, bat = p[10], p[1]
        L = [f"* U1 BQ76922 pin model ({__name__})"]
        for k in self.ctl:
            L.append(f"v_u1_{k} {x}{k} 0 dc 0 external")
        L += [
            # BAT supply current, µA from the state machine
            f"b_u1_ibat {bat} {vss} i=v({x}ibat)*1e-6*tanh(max(v({bat},{vss}),0)/0.5)",
            # REG18 LDO 1.8 V when awake; internal load 100 kΩ always
            f"b_u1_reg18 {p[15]} {vss} i=v({x}reg18)*(v({p[15]},{vss})-1.8)/2",
            f"r_u1_reg18ld {p[15]} {vss} 100k",
            # TS1 18 kΩ + 520 Ω pad pull-up from REG18 while measuring
            f"b_u1_ts1 {p[15]} {p[13]} i=v({x}ts1)*v({p[15]},{p[13]})/18.52k",
            # TS2 ≈5 V through 5 MΩ in SHUTDOWN (DS 6.6 VOH, ROH)
            f"b_u1_ts2 {p[14]} {vss} i=v({x}ts2)*(v({p[14]},{vss})"
            f"-min(5,max(v({bat},{vss})-0.5,0)))/5e6",
            # LD: 80 kΩ pull-down in SHUTDOWN (DS 6.7 RPD)
            f"b_u1_ldpd {p[28]} {vss} i=v({x}ldpd)*v({p[28]},{vss})/80k",
            # charge pump: CP1 driven to BAT + 11 V through 40 kΩ
            f"b_u1_cp {p[32]} {vss} i=(v({p[32]},{vss})-(v({bat},{vss})"
            f"+11*v({x}cp)))/40k",
            # DSG on from CP1; off = pull toward VSS while above LD
            f"b_u1_dsgon {p[32]} {p[30]} i=v({x}dsg)*v({p[32]},{p[30]})/2.6k",
            f"b_u1_dsgoff {p[30]} {vss} i=v({x}dsgoff)*v({p[30]},{vss})/30*"
            + _sig(f"v({p[30]},{p[28]})", "0.05"),
            # CHG on from CP1; off = switch to BAT
            f"b_u1_chgon {p[32]} {p[31]} i=v({x}chg)*v({p[32]},{p[31]})/2.6k",
            f"b_u1_chgoff {p[31]} {bat} i=v({x}chgoff)*v({p[31]},{bat})/1k",
            # PACK and LD measurement dividers are 600 kΩ / 2 MΩ only while
            # measured (DS 6.18): left out, below every other load there.
        ]
        # input ESD structure implied by the abs-max table (DS 6.1)
        vc = [p[9], p[8], p[7], p[2]]
        L.append(f"d_u1_esd_vc0 {vss} {vc[0]} d_esd")
        for a, b in zip(vc, vc[1:]):
            L.append(f"d_u1_esd_{b} {a} {b} d_esd")
        for pin in (11, 12, 13, 14, 17, 18, 21, 28, 29):
            L.append(f"d_u1_esd{pin} {vss} {p[pin]} d_esd")
        return L

    def source(self, name: str, t: float) -> float:
        return self.ctl[name[len("v_u1_"):]].at(t)

    def next_event(self, t: float) -> float:
        te = math.inf
        for c in self.ctl.values():
            if c.t0 + c.tr > t:
                te = min(te, c.t0 if c.t0 > t else c.t0 + c.tr)
        cands = [self.next_fet_eval, self.dsgoff_until]
        if self.mode == "BOOT":
            cands.append(self.t_mode + self.lim.t_boot_fets_s)
        if self.rst_high_since is not None:
            cands.append(self.rst_high_since + 1.0)
        for k, t0 in self.pending.items():
            cands.append(t0 + self._delay(k))
        for k, t0 in self.recover.items():
            cands.append(t0 + self._rec_time(k))
        if self.ts2_low_since is not None:
            cands.append(self.ts2_low_since + self.lim.ts2_wake_min_s)
        return min([te] + [c for c in cands if c > t + 1e-12])

    # ---------------- helpers ----------------
    def _log(self, t, s):
        self.events.append((t, s))

    def _set_mode(self, t, m, why=""):
        if m != self.mode:
            self._log(t, f"U1 {self.mode}->{m} {why}".rstrip())
            self.mode = m
            self.t_mode = t

    def _delay(self, k):
        o = self.otp
        return {"CUV": o.cuv_delay_s, "COV": o.cov_delay_s,
                "OCD1": o.ocd1_delay_s,
                "SCD": 0.0 if o.scd_delay_setting <= 1
                else (o.scd_delay_setting - 1) * 15e-6}[k]

    def _rec_time(self, k):
        return self.otp.scd_recovery_s if k == "SCD" else self.otp.recovery_s

    def _drive(self, t):
        """Apply dsg_on/chg_on and mode to the pin controls."""
        c = self.ctl
        awake = self.mode in ("BOOT", "NORMAL", "SOFTSHUT")
        powered = self.mode != "OFF"
        c["reg18"].set(t, 1.0 if awake else 0.0)
        c["ts1"].set(t, 1.0 if self.mode == "NORMAL" else 0.0)
        c["ts2"].set(t, 1.0 if self.mode in ("SHUTDOWN", "SOFTSHUT") else 0.0)
        c["ldpd"].set(t, 1.0 if self.mode == "SHUTDOWN" else 0.0)
        c["cp"].set(t, 1.0 if self.mode in ("BOOT", "NORMAL") else 0.0, 0.5e-3)
        dsg = self.dsg_on and self.mode == "NORMAL"
        chg = self.chg_on and self.mode == "NORMAL"
        if not dsg and c["dsg"].v1 > 0.5:
            self.dsgoff_until = t + 200e-6
            c["dsgoff"].set(t, 1.0)
        c["dsg"].set(t, 1.0 if dsg else 0.0)
        c["chg"].set(t, 1.0 if chg else 0.0)
        c["chgoff"].set(t, 0.0 if chg or not powered else 1.0)
        if t >= self.dsgoff_until:
            c["dsgoff"].set(t, 0.0)
        ib = {"OFF": 0.0, "SHUTDOWN": 1.0, "BOOT": 252.0, "NORMAL": 252.0,
              "SOFTSHUT": 100.0}[self.mode]
        c["ibat"].set(t, ib)

    # ---------------- digital behaviour ----------------
    def step(self, t: float, v: dict[str, float]):
        p, o, lim = self.p, self.otp, self.lim
        g = lambda n: v.get(n, 0.0)  # noqa: E731
        vss = g(p[10])
        vbat = g(p[1]) - vss
        ts2 = g(p[14]) - vss
        ld = g(p[28]) - vss
        rst = g(p[21]) - vss
        reg18 = max(g(p[15]) - vss, 0.0)
        vc0, vc1, vc2, vc5 = (g(p[9]), g(p[8]), g(p[7]), g(p[2]))
        self.cells = (vc1 - vc0, vc2 - vc1, vc5 - vc2)
        srn_srp = g(p[12]) - g(p[11])
        self.i_pack = srn_srp / self.r_sense
        tos = vc5 - vss

        # power-on reset
        if self.mode == "OFF":
            if vbat > lim.v_pora:
                self._set_mode(t, "SHUTDOWN", "POR")
            self._drive(t)
            return
        if vbat < lim.v_pora - lim.v_pora_hys:
            self.dsg_on = self.chg_on = False
            self.faults.clear(); self.pending.clear(); self.recover.clear()
            self._set_mode(t, "OFF", "BAT below VPORA")
            self._drive(t)
            return

        if self.mode == "SHUTDOWN":
            if ts2 < lim.v_wake_ts2:
                if self.ts2_low_since is None:
                    self.ts2_low_since = t
                if t - self.ts2_low_since >= lim.ts2_wake_min_s - 1e-9:
                    self._wake(t, "TS2")
            else:
                self.ts2_low_since = None
            if ld < lim.v_wake_ld:
                self.ld_fell = True
            elif self.ld_fell:
                self._wake(t, "LD")
            self._drive(t)
            return

        if self.mode == "SOFTSHUT":
            if ld < lim.v_wake_ld:
                self.ld_fell = True
            elif self.ld_fell:
                self._wake(t, "LD rose in soft shutdown")
                self._drive(t)
                return
            rst_hi = rst > 0.67 * lim.v_reg18
            if rst_hi and not self.rst_prev:
                self._wake(t, "RST_SHUT edge in soft shutdown")
            elif ts2 >= lim.v_wake_ts2 and ld < lim.v_wake_ld:
                self._set_mode(t, "SHUTDOWN", "soft shutdown complete")
                self.ld_fell = True
                self.ts2_low_since = None
            self.rst_prev = rst_hi
            self._drive(t)
            return

        # BOOT / NORMAL: RST_SHUT
        rst_hi = rst > 0.67 * max(reg18, 1.6)
        if rst_hi:
            if self.rst_high_since is None:
                self.rst_high_since = t
                self._log(t, "U1 RST_SHUT high: digital reset")
            if t - self.rst_high_since >= 1.0 - 1e-9 and self.shut_seq_at is None:
                self._begin_shutdown(t, "RST_SHUT high 1 s")
        else:
            if self.rst_high_since is not None and self.shut_seq_at is None:
                self._log(t, "U1 RST_SHUT released < 1 s: reset only")
            self.rst_high_since = None
        self.rst_prev = rst_hi

        if self.shut_seq_at is not None:
            if t - self.shut_seq_at >= o.fet_off_delay_s:
                self.dsg_on = self.chg_on = False
            if t - self.shut_seq_at >= o.shutdown_cmd_delay_s:
                # FETs off; SHUTDOWN proper waits in SOFTSHUT until LD is
                # below V(WAKEONLD) and TS2 is not held low (TRM §7.5)
                self.shut_seq_at = None
                self.rst_high_since = None
                self.ld_fell = ld < lim.v_wake_ld
                self._set_mode(t, "SOFTSHUT", "shutdown sequence")
                self.faults.clear(); self.pending.clear(); self.recover.clear()
            self._drive(t)
            return

        if rst_hi:
            # digital logic held in reset (DS §7.7.6): FETs keep their
            # state, boot and protections do not advance
            if self.mode == "BOOT":
                self.t_mode = t
            self.pending.clear()
            self.recover.clear()
            self._drive(t)
            return

        if self.mode == "BOOT":
            if t - self.t_mode >= lim.t_boot_fets_s - 1e-9:
                self._set_mode(t, "NORMAL", "boot done")
                # DS §7.8.5: at wake the device makes its first measurements
                # and "evaluate[s] those relative to enabled protections, then
                # enable[s] FETs if conditions allow"
                cmin, cmax = min(self.cells), max(self.cells)
                if cmin < o.cuv_v:
                    self._trip(t, "CUV", cmin)
                if cmax > o.cov_v:
                    self._trip(t, "COV", cmax)
                self._eval_fets(t)
                self.next_fet_eval = t + 0.25
            self._drive(t)
            return

        # NORMAL: protections
        tick = t >= self.next_fet_eval - 1e-9
        self._protect(t, srn_srp * 1e3, tos, tick)
        if tick:
            self._eval_fets(t)
            self.next_fet_eval = t + 0.25
        self._drive(t)

    def _wake(self, t, why):
        self.ts2_low_since = None
        self.ld_fell = False
        self.faults.clear(); self.pending.clear(); self.recover.clear()
        self.dsg_on = self.chg_on = False
        self.rst_high_since = None
        self.shut_seq_at = None
        self.lowv_since = None
        self._set_mode(t, "BOOT", f"wake by {why}")

    def _begin_shutdown(self, t, why):
        self._log(t, f"U1 shutdown sequence: {why}")
        self.shut_seq_at = t

    def _eval_fets(self, t, cells_ok_check=False):
        if not self.otp.fet_en:
            self.dsg_on = self.chg_on = False
            return
        block_dsg = any(k in self.faults for k in ("CUV", "OCD1", "SCD", "SCDL"))
        block_chg = "COV" in self.faults
        if not block_dsg and not self.dsg_on:
            self.dsg_on = True
            self._log(t, "U1 DSG on")
        if not block_chg and not self.chg_on:
            self.chg_on = True
            self._log(t, "U1 CHG on")

    def _trip(self, t, k, val):
        self.faults[k] = t
        self.pending.pop(k, None)
        self._log(t, f"U1 {k} trip ({val:.4g})")
        if k in ("CUV", "OCD1", "SCD", "SCDL") and self.dsg_on:
            self.dsg_on = False
            self._log(t, "U1 DSG off")
        if k == "COV" and self.chg_on:
            self.chg_on = False
            self._log(t, "U1 CHG off")

    def _protect(self, t, scd_mv_meas, tos, tick):
        o, lim = self.otp, self.lim
        cmin, cmax = min(self.cells), max(self.cells)
        ocd_mv = o.ocd1_mv + lim.ocd_tol_mv
        scd_mv = o.scd_mv * (1.0 + lim.scd_tol)
        self.max_scd_mv = max(self.max_scd_mv, scd_mv_meas)
        conds = {"CUV": cmin < o.cuv_v, "COV": cmax > o.cov_v,
                 "OCD1": scd_mv_meas > ocd_mv, "SCD": scd_mv_meas > scd_mv}
        for k, c in conds.items():
            if k in self.faults:
                continue
            if c:
                t0 = self.pending.setdefault(k, t)
                if t - t0 >= self._delay(k) - 1e-12:
                    self._trip(t, k, scd_mv_meas if k in ("OCD1", "SCD")
                               else (cmin if k == "CUV" else cmax))
                    if k == "SCD":
                        self.scdl_count += 1
                        if o.scdl_limit and self.scdl_count >= o.scdl_limit:
                            self.faults["SCDL"] = t
                            self._log(t, "U1 SCDL latch")
            else:
                self.pending.pop(k, None)
        # recoveries (TRM §5.2)
        rec = {"CUV": cmin >= o.cuv_v + o.cuv_hys_v,
               "COV": cmax <= o.cov_v - o.cov_hys_v,
               "OCD1": -self.i_pack >= o.ocd_recovery_a,
               "SCD": scd_mv_meas <= o.scd_mv}
        for k in list(self.faults):
            if k not in rec:
                continue
            if rec[k]:
                t0 = self.recover.setdefault(k, t)
                if t - t0 >= self._rec_time(k) - 1e-9:
                    del self.faults[k]
                    self.recover.pop(k)
                    self._log(t, f"U1 {k} recovered")
            else:
                self.recover.pop(k, None)
        # body diode protection (DS §7.6.5.1, series FETs)
        if self.dsg_on and not self.chg_on and self.i_pack > o.body_diode_a:
            self.chg_on = True
            self._log(t, "U1 CHG on (body-diode protection)")
        elif ("COV" in self.faults and self.chg_on
              and self.i_pack < o.body_diode_a * 0.5):
            self.chg_on = False
            self._log(t, "U1 CHG off (COV, discharge stopped)")
        # low-voltage shutdown (TRM §7.5), judged on the measurement loop's
        # samples, every 250 ms in NORMAL (TRM Table 6-9), unlike the CUV
        # comparator: a 10 µs dip between samples is not seen
        if not tick:
            return
        low = (o.shutdown_stack_v > 0 and tos < o.shutdown_stack_v) or \
              (o.shutdown_cell_v > 0 and cmin < o.shutdown_cell_v)
        if low:
            if self.lowv_since is None:
                self.lowv_since = t
            if t - self.lowv_since >= o.low_v_shutdown_delay_s - 1e-9:
                self.lowv_since = None
                self._begin_shutdown(t, f"low voltage (TOS {tos:.2f} V, "
                                        f"lowest cell {cmin:.3f} V)")
        else:
            self.lowv_since = None


# --------------------------------------------------------------------------
# TPS48111-Q1
# --------------------------------------------------------------------------

@dataclass
class TpsLimits:
    """TPS 6.5 / 6.6 limits; typical by default, corners by override."""
    uvlor: float = 1.18
    uvlof: float = 1.11
    enf: float = 0.7           # 0.3 / 0.7 / 0.9
    en_hys: float = 0.043
    inp_h: float = 1.6         # max 2.0
    inp_l: float = 1.2         # min 0.8
    vs_porr: float = 3.0
    vs_porf: float = 2.9
    bst_uvlor: float = 7.6
    bst_uvlof: float = 6.5
    cp_on: float = 11.7
    cp_off: float = 12.3
    i_cp: float = 100e-6
    i_tmr_cb: float = 82e-6
    i_tmr_slow: float = 2.5e-6
    v_tmr_oc: float = 1.2
    v_tmr_flt: float = 1.1
    v_tmr_low: float = 0.2
    i_iscp: float = 15.6e-6
    t_sc: float = 1.16e-6
    iwrn_ref: float = 1.19
    t_ot_c: float = 150.0
    r_pd: float = 1.0          # 0.69 / 1.34 Ω


class Tps48111:
    """U2."""

    PREFIX = "u2"

    def __init__(self, pin: dict[int, str], lim: TpsLimits, *,
                 diode_sense: bool = False, r_set: float = 100.0):
        self.p = pin
        self.r_set = r_set
        self.lim = lim
        self.ctl = {k: Ctl(0.0) for k in (
            "iq", "act", "cp", "pu", "pd", "flti", "fltt", "tmr", "tmrdis",
            "diode")}
        self.ctl["pd"] = Ctl(1.0)
        self.ctl["diode"] = Ctl(10.0)
        self.state = "OFF"       # OFF (no VS), SHDN, UVLO, ON-ready ...
        self.por = False
        self.active = False
        self.uvlo_ok = False
        self.bst_ok = False
        self.cp_running = False
        self.inp = False
        self.gate = False
        self.latch: str | None = None     # "OC", "SC", "OT"
        self.oc_counting = False
        self.retry_phase: str | None = None
        self.retry_count = 0
        self.sc_since: float | None = None
        self.flt_i = False
        self.diode_sense = diode_sense
        self.diode_phase_t = 0.0
        self.diode_v = {"lo": None, "hi": None}
        self.t_diode_c: float | None = None
        self.events: list[tuple[float, str]] = []

    def spice(self) -> list[str]:
        p, x = self.p, "x_u2_"
        L = [f"* U2 TPS48111-Q1 pin model ({__name__})"]
        for k in self.ctl:
            L.append(f"v_u2_{k} {x}{k} 0 dc 0 external")
        vs, en, inp, inpg = p[20], p[1], p[3], p[2]
        csp, csm, iscp = p[18], p[17], p[19]
        # sense amplifier (TPS Fig. 8-12): it sinks VSNS/RSET at CS+ so that
        # CS+ sits at CS−. Modelled as that current directly, VSNS taken from
        # VS (the top of RSNS and of RSET on this board) to CS−, with a smooth
        # max(·, 0) 10 µV wide (a hard kink stalls ngspice when the sense
        # reverses) and a 10 mA ceiling.
        vsn = f"v({vs},{csm})"
        ics = (f"10m*tanh(0.5*({vsn}+sqrt({vsn}*{vsn}+4e-10))"
               f"/{self.r_set:.6g}/10m)")
        L += [
            f"b_u2_iq {vs} 0 i=v({x}iq)*1e-6*tanh(max(v({vs}),0)/0.5)",
            f"b_u2_en {en} 0 i=61n*tanh(v({en})/0.1)",
            f"r_u2_inp {inp} 0 1meg",
            f"r_u2_inpg {inpg} 0 1meg",
            f"b_u2_flti {p[5]} 0 i=v({x}flti)*v({p[5]})/70",
            f"b_u2_fltt {p[4]} 0 i=v({x}fltt)*v({p[4]})/70",
            # current-sense amplifier: sinks VSNS/RSET at CS+ (TPS §8.3.4)
            f"b_u2_csamp {csp} 0 i=v({x}act)*{ics}",
            f"b_u2_imon 0 {p[7]} i=v({x}act)*0.9*{ics}*"
            + _sig(f"min(v({vs})-0.5,6.5)-v({p[7]})", "0.05"),
            f"b_u2_iwrn 0 {p[8]} i=v({x}act)*0.1*{ics}",
            f"b_u2_csm {csm} 0 i=v({x}act)*183u",
            # short-circuit comparator input: 464 Ω inside the ISCP pin
            f"r_u2_iscp {iscp} u2_iscp_i 464",
            f"b_u2_iscpi u2_iscp_i 0 i=v({x}act)*{self.lim.i_iscp}",
            f"b_u2_tmr 0 {p[9]} i=v({x}tmr)*1e-6",
            f"b_u2_tmrdis {p[9]} 0 i=v({x}tmrdis)*v({p[9]})/100",
            f"b_u2_diode 0 {p[10]} i=v({x}diode)*1e-6*" + _sig(f"3-v({p[10]})", "0.1"),
            # charge pump into BST (TPS §8.3.1)
            f"b_u2_cp 0 {p[12]} i=v({x}cp)*{self.lim.i_cp}*"
            + _sig(f"v({vs})+14-v({p[12]})", "0.2"),
            f"b_u2_pu {p[12]} {p[15]} i=v({x}pu)*v({p[12]},{p[15]})/3.2",
            f"b_u2_pd {p[14]} {p[13]} i=v({x}pd)*v({p[14]},{p[13]})/{self.lim.r_pd}",
            f"r_u2_pdweak {p[14]} {p[13]} 1meg",
            f"r_u2_g {p[11]} {p[13]} 100",
        ]
        for pin in (1, 2, 3, 4, 5, 7, 8, 9, 10):
            L.append(f"d_u2_esd{pin} 0 {p[pin]} d_esd")
        return L

    def source(self, name: str, t: float) -> float:
        return self.ctl[name[len("v_u2_"):]].at(t)

    def _log(self, t, s):
        self.events.append((t, s))

    def next_event(self, t: float) -> float:
        te = math.inf
        for c in self.ctl.values():
            if c.t0 + c.tr > t:
                te = min(te, c.t0 if c.t0 > t else c.t0 + c.tr)
        if self.sc_since is not None:
            te = min(te, self.sc_since + self.lim.t_sc)
        if self.diode_sense and self.active:
            te = min(te, self.diode_phase_t + (1e-3 if self.ctl["diode"].v1 < 50
                                               else 1e-4))
        return te

    def _reset_latch(self, t, why):
        if self.latch or self.retry_phase or self.oc_counting:
            self._log(t, f"U2 latch reset ({why})")
        self.latch = None
        self.retry_phase = None
        self.retry_count = 0
        self.oc_counting = False
        self.flt_i = False

    def step(self, t: float, v: dict[str, float]):
        p, lim, c = self.p, self.lim, self.ctl
        g = lambda n: v.get(n, 0.0)  # noqa: E731
        vs, en, inp = g(p[20]), g(p[1]), g(p[3])
        bst_src = g(p[12]) - g(p[13])
        tmr = g(p[9])
        if self.por:
            if vs < lim.vs_porf:
                self.por = False
                self._reset_latch(t, "VS below POR")
                self._log(t, "U2 VS below POR")
        elif vs > lim.vs_porr:
            self.por = True
            self._log(t, "U2 VS POR")
        was_active = self.active
        if self.active:
            if en < lim.enf or not self.por:
                self.active = False
        elif self.por and en > lim.enf + lim.en_hys:
            self.active = True
        if was_active and not self.active:
            # EN below ENF resets every latch, OT included (TPS §8.3.6)
            self._reset_latch(t, "EN below ENF")
            self._log(t, "U2 shutdown")
        if self.active and not was_active:
            self._log(t, "U2 active")
        self.uvlo_ok = self.active and (en > lim.uvlor or
                                        (self.uvlo_ok and en > lim.uvlof))
        # charge pump hysteresis
        if self.active:
            if bst_src < lim.cp_on:
                self.cp_running = True
            elif bst_src > lim.cp_off:
                self.cp_running = False
        else:
            self.cp_running = False
        self.bst_ok = self.active and (bst_src > lim.bst_uvlor or
                                       (self.bst_ok and bst_src > lim.bst_uvlof))
        new_inp = inp > lim.inp_h or (self.inp and inp > lim.inp_l)
        if self.inp and not new_inp and self.latch in ("OC", "SC"):
            self._reset_latch(t, "INP low")
        if self.inp and not new_inp:
            self.oc_counting = False
        self.inp = new_inp

        # remote temperature (DIODE): ΔV at 10 µA / 160 µA (TPS §8.3.6)
        if self.diode_sense and self.active:
            hi = c["diode"].v1 > 50
            dur = 1e-4 if hi else 9e-4
            if t - self.diode_phase_t >= dur - 1e-9:
                self.diode_v["hi" if hi else "lo"] = g(p[10])
                c["diode"].set(t, 10.0 if hi else 160.0)
                self.diode_phase_t = t
                if self.diode_v["hi"] is not None and self.diode_v["lo"] is not None:
                    dv = self.diode_v["hi"] - self.diode_v["lo"]
                    self.t_diode_c = dv / (8.617333e-5 * math.log(16)) - 273.15
                    if self.t_diode_c >= lim.t_ot_c and self.latch != "OT":
                        self.latch = "OT"
                        self._log(t, f"U2 OT latch ({self.t_diode_c:.1f} °C)")

        # over-current comparator and short circuit
        iwrn = g(p[8])
        oc = self.active and iwrn > lim.iwrn_ref
        sc = self.active and g("u2_iscp_i") > g(p[17])
        if sc and self.latch is None and self.gate:
            if self.sc_since is None:
                self.sc_since = t
            if t - self.sc_since >= lim.t_sc - 1e-12:
                self.latch = "SC"
                self.flt_i = True
                self.retry_phase = "down"
                self._log(t, "U2 SC trip")
        else:
            self.sc_since = None
        if self.latch is None and self.gate and oc and not self.oc_counting:
            self.oc_counting = True
            self._log(t, "U2 OC: timer charging")
        if self.oc_counting:
            if not oc:
                self.oc_counting = False
                self.flt_i = False
            else:
                if tmr >= lim.v_tmr_flt:
                    self.flt_i = True
                if tmr >= lim.v_tmr_oc:
                    self.oc_counting = False
                    self.latch = "OC"
                    self.retry_phase = "down"
                    self._log(t, "U2 OC turn-off (CB)")
        # auto-retry counter (TPS §8.3.3.1); latch-off when it cannot count
        if self.retry_phase == "down" and tmr <= lim.v_tmr_low:
            self.retry_phase = "up"
        elif self.retry_phase == "up" and tmr >= lim.v_tmr_flt:
            self.retry_count += 1
            self.retry_phase = "down"
            if self.retry_count >= 32:
                self._log(t, "U2 auto-retry: FET back on")
                self._reset_latch(t, "retry count")
        # gate
        gate = (self.active and self.uvlo_ok and self.bst_ok and self.inp and
                self.latch is None)
        if gate != self.gate:
            self._log(t, f"U2 gate {'ON' if gate else 'OFF'}")
            self.gate = gate
        c["iq"].set(t, 613.0 if self.active else (1.6 if self.por else 0.0))
        c["act"].set(t, 1.0 if self.active else 0.0)
        c["cp"].set(t, 1.0 if self.cp_running else 0.0, 1e-6)
        c["pu"].set(t, 1.0 if gate else 0.0)
        c["pd"].set(t, 0.0 if gate else 1.0)
        c["flti"].set(t, 1.0 if self.flt_i else 0.0)
        c["fltt"].set(t, 1.0 if self.latch == "OT" else 0.0)
        if self.oc_counting:
            c["tmr"].set(t, lim.i_tmr_cb * 1e6)
            c["tmrdis"].set(t, 0.0)
        elif self.retry_phase == "down":
            c["tmr"].set(t, -lim.i_tmr_slow * 1e6)
            c["tmrdis"].set(t, 0.0)
        elif self.retry_phase == "up":
            c["tmr"].set(t, lim.i_tmr_slow * 1e6)
            c["tmrdis"].set(t, 0.0)
        else:
            c["tmr"].set(t, 0.0)
            c["tmrdis"].set(t, 1.0)
