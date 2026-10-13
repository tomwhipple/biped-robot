"""Behavioural model of the 3S LiPo pack, balance taps included.

The design record (§3) assumes a 3S 4000 mAh pack, and the 2026-10-07
verification record C13 substitutes the actual ordered OVONIC 4200 mAh
130C part with **no maker datasheet** for resistance. So:

- Capacity: 4200 mAh (C13 measured value), but the model is parameterised.
- Internal resistance: a parameter we vary (C13's "depends on the internal
  resistance" finding). The suite sweeps 5/8/15/35 mΩ cases.
- Cell taps: the XHP-4 balance J2 pins (1 B-, 2 c1+, 3 c2+, 4 B+).

This is a behavioural model, not an electrochemical one — no Peukert, no
hysteresis. What it must do and does:

- V_oc(SOC) is the well-known S-curve for Li-ion / LiPo: a linear ramp from
  3.0 V at 0% to 4.2 V at 100% with a knee at ~5%. We use a piecewise
  linear approximation; for the protections we exercise (UV/OV/OC), what
  matters is that V(t) is *monotonic in SOC* at the thresholds. Absolute
  error of ±30 mV across the band is fine for this use.
- Series resistance per cell is added under load: V_term = V_oc − I·R_cell.
  R_pack = 3·R_cell + R_leads (measured, ~1–2 mΩ for the XT60 lead pair).
- The balance taps expose per-cell V terms so a single-cell-imbalance
  test can drive one cell low while the others stay normal.

The model is intentionally tiny: cell SOC is the state, and "simulation"
means numerically integrating I·dt against it. Real simulators (ngspice)
handle the fast electrical transients; this model handles the slow
"chemistry" (what happens over seconds of discharge), where closed-form
behaviour is what the protections actually respond to.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# ------------------ per-cell V_oc(SOC): piecewise-linear, monotone ---------
# Calibration anchors (approximate, standard LiPo): see any HobbyKing/Gens-Ace
# datasheet discharge curve. Anchors picked so dV/dSOC is roughly constant in
# the 20–80 % band where walking lives.

_X = np.array([0.00, 0.02, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 0.90, 0.95, 1.00])
_Y = np.array([2.50, 3.00, 3.30, 3.50, 3.65, 3.74, 3.82, 3.92, 4.05, 4.15, 4.20])


def v_oc(soc: float) -> float:
    """Open-circuit voltage of one cell at `soc` ∈ [0, 1]."""
    s = float(np.clip(soc, 0.0, 1.0))
    return float(np.interp(s, _X, _Y))


@dataclass
class Cell:
    """One 21700-class 4200 mAh Li-ion cell."""

    capacity_mah: float = 4200.0
    internal_resistance_mohm: float = 3.0   # matches a fresh 130C-class cell
    soc: float = 1.0                        # 1.0 = full, 0.0 = empty

    def terminal_v(self, i_a: float = 0.0) -> float:
        """Cell terminal voltage under load (i>0 = discharge)."""
        return v_oc(self.soc) - i_a * self.internal_resistance_mohm * 1e-3

    def step(self, i_a: float, dt_s: float):
        """Integrate: `dt_s` seconds at `i_a` amps changes SOC by the charge
        moved. SOC can exceed 1.0 (over-charge, exercised by the OV tests)
        or drop below 0 (over-discharge).
        """
        mah_moved = i_a * dt_s * 1000.0 / 3600.0
        self.soc -= mah_moved / self.capacity_mah
        return self.soc


@dataclass
class Pack3S:
    """The 3S1P OVONIC 4200 mAh pack; the XT60 discharge port + 4-pin balance
    lead. `cells[0]` is the cell between B− and c1+, `cells[1]` between c1+
    and c2+, `cells[2]` between c2+ and B+.
    """
    cells: list[Cell]
    lead_resistance_mohm: float = 1.5   # XT60 pair, short 12 AWG leads

    @classmethod
    def fresh(cls, r_cell_mohm: float = 3.0, capacity_mah: float = 4200.0):
        return cls(cells=[
            Cell(capacity_mah=capacity_mah,
                 internal_resistance_mohm=r_cell_mohm, soc=1.0)
            for _ in range(3)
        ])

    @property
    def total_internal_mohm(self) -> float:
        return sum(c.internal_resistance_mohm for c in self.cells) \
            + self.lead_resistance_mohm

    @property
    def pack_v_open(self) -> float:
        return sum(v_oc(c.soc) for c in self.cells)

    def terminal_v(self, i_a: float = 0.0) -> float:
        """Pack terminal voltage at the XT60 under load i_a (positive = discharge)."""
        return self.pack_v_open - i_a * self.total_internal_mohm * 1e-3

    def balance_taps(self, i_a: float = 0.0) -> tuple[float, float, float, float]:
        """The four J2 pin voltages w.r.t. B− (pin 1), in pin order 1,2,3,4.

        Cell-imbalance tests drive one cell to a low SOC; the others sit
        at the same SOC as each other. `i_a` applies the per-cell IR drop
        *inside* the cell, so a high series resistance on the bad cell
        makes its tap sag under load while its neighbours stay up — that's
        the signal BQ76922's per-cell UV sees.
        """
        v1 = self.cells[0].terminal_v(i_a)
        v2 = self.cells[1].terminal_v(i_a)
        v3 = self.cells[2].terminal_v(i_a)
        return (0.0, v1, v1 + v2, v1 + v2 + v3)

    def step(self, i_a: float, dt_s: float):
        for c in self.cells:
            c.step(i_a, dt_s)

    def soc_min(self) -> float:
        return min(c.soc for c in self.cells)

    def set_cell_soc(self, idx: int, soc: float):
        assert 0 <= idx < 3
        self.cells[idx].soc = soc


def bq76922_stack_vs_cells(pack: Pack3S, i_a: float = 0.0):
    """What the BQ76922 sees at its VC pins: same as `balance_taps` (both
    sides of the balance input-filters R3–R6 are 20 Ω each, negligible at
    the BQ76922's µA input current)."""
    return pack.balance_taps(i_a)
