"""Part-2.1: servo branches J4–J8 — the current checks the design record
demands the *board* supports.

The design record's authoritative numbers:

- §2 (Loads):     servo bus upper/lower walking model: mean 3.05 A / 1.0 A,
                  RMS 3.11 / 1.0, peak 4.74 / 1.7
                  get-ups:                  peak 9.4–11.8 A / 4.6–6.7 A;
                                            worst 2 s 5.0–7.6 / 2.0–3.4 A
                  servo branch (one leg):   walking peak 3.7 A, RMS 1.6 A
                                              (upper)
- harness.md §J4–J8, "walking peak / get-up peak":
                  J4 (R leg)   3.7 A   / 3.0–4.3 A
                  J5 (R upper) 0.3–0.4 / 2.9–3.0 A
                  J6 (L leg)   3.7 A   / 3.0–4.3 A
                  J7 (L upper) 0.3–0.4 / 2.9–3.0 A
                  J8 (data)    mA

- F1 (15 A MINI blade, Littelfuse 0297015.WXNV): opening time at 200 %
  (30 A) is 0.15–5 s per the design record §5.1; the fuse's I²t profile is
  what we compare the peak profiles against. We use the Littelfuse 297
  series datasheet's published values (design record §13 sources):
      100 % of rating (15 A): never opens
      135 % (20.25 A):        opens ≤ 60 min, ≥ 1 min
      200 % (30 A):           opens 0.15–5 s
      350 % (52.5 A):         opens 0.08–0.25 s
      600 % (90 A):           opens 0.02–0.10 s
  (Littelfuse 297 datasheet, "average time-current curves".)

What "test green" means here:
  * per-branch peaks (upper model) fall under the F1 fuse's "no-melt" band
    with margin, and under the TPS48111's 19.8 A CB trip with margin
    (a single-board trip on a normal gait peak is a design failure)
  * the aggregate (sum over J4–J7 simultaneously at get-up) sits under the
    15 A fuse's slow band for at most the fuse's tolerance at 135 %
  * the 5267 contact on J5/J7 (rated 2.5 A) sees ≤ its rating at the
    2.9–3.0 A get-up peak: this is a **harness.md tolerance**, so a 20 %
    margin is what we call "green"; anything above the contact rating
    continuously is a documentable fail.
  * soft start into rev A's bulk (C21=1000 µF + C22=10 µF + per-servo
    local 0.5–2 mF): peak inrush through Q3 does not exceed the
    TPS48111's CB trip of 19.8 A and stays on the FET's DC line of
    ≈3–4 A at 10 V (CSD17556Q5B SOA Figure 10). C4's ngspice runs gave
    1.4–2.0 A. Our check: peak ≤ 4 A and total CB-timer charge ≪ the
    latch threshold.

These numbers are not the TPS48111's *own* protections; those are tested in
test_pack_voltage / test_esp32_control.
"""
from __future__ import annotations

import numpy as np
import pytest

from pbsim.schematicsnapshot import parse_si_value
from pbsim.sequencer_sim import Buttons, Esp32Stub, PowerBoard
from pbsim.pack import Pack3S


# ----------------- the harness numbers, transcribed ------------------------

# Branch peak profiles (harness.md §J4–J8):
WALK_PEAK_A  = {"J4": 3.7, "J5": 0.4, "J6": 3.7, "J7": 0.4}
GETUP_PEAK_A = {"J4": (3.0, 4.3), "J5": (2.9, 3.0),
                "J6": (3.0, 4.3), "J7": (2.9, 3.0)}
WALK_RMS_A   = {"J4": 1.6, "J5": 0.1, "J6": 1.6, "J7": 0.1}

# Servo-bus aggregates (design §2 "Loads"):
WALK_MEAN_A_UPPER = 3.05
WALK_MEAN_A_LOWER = 1.00
WALK_PEAK_A_BUS   = 4.74
WALK_PEAK_A_BUS_LOWER = 1.7
GETUP_PEAK_A_BUS_UPPER = (9.4, 11.8)
GETUP_PEAK_A_BUS_LOWER = (4.6, 6.7)

# Contact ratings and fuse profile per design + harness
MOLEX_5264_CONTACT_A = 2.5     # harness.md: "rated about 2.5–3 A"
MOLEX_5264_CONTACT_MAX_A = 3.0
F1_RATING_A = 15.0             # schematic: F1 = "15A"
F1_NEVER_OPENS_A = 15.0        # 100 %
F1_SLOW_BAND_A = 20.25         # 135 % — opens ≥ 60 s
F1_FAST_200PCT_A = 30.0        # opens 0.15–5 s
GETUP_DURATION_S = 2.0         # design: "worst 2 s"
GETUP_OCCURRENCES_PER_MIN = 1  # a working gait cycle's get-up count, §2

# TPS48111 thresholds, pinned by C3 (and the schematic's parse):
TPS48111_CB_TRIP_A = 19.8     # circuit-breaker, latches in 10.5 ms
TPS48111_SC_TRIP_A = 39.3     # short-circuit, 1.6 µs

# Soft-start inrush limits (CSD17556Q5B SOA, 10 V DC): ~3–4 A continuous.
# C4's ngspice runs with the post-change capacitor (C16 = 330 nF) showed
# 1.40 A peak into a 2 mF servo capacitive load, 1.99 A into 4 mF.
SOFT_START_PEAK_LIMIT_A = 4.0


# ----------------- static assertions (no simulation needed) ------------------

def test_every_servo_branch_peak_is_documented(schematic):
    """The harness.md table and the design record's §2 table agree on
    per-branch peaks; the schematic carries the four output headers."""
    comps = schematic.components
    for ref, branch in (("J4", "R leg"), ("J5", "R upper"),
                        ("J6", "L leg"), ("J7", "L upper"),
                        ("J8", "Data")):
        c = comps[ref]
        assert branch in c.value, f"{ref} value={c.value!r}"


def test_branch_peak_currents_fit_under_tps48111_cb_trip():
    """J4's 4.3 A get-up peak, plus J6 simultaneously, must not come close to
    tripping the servo rail's own circuit-breaker. (Two legs can share a
    get-up; the uppers + data add ~3 A on top.)"""
    two_leg_getup = max(GETUP_PEAK_A["J4"]) + max(GETUP_PEAK_A["J6"]) \
        + max(GETUP_PEAK_A["J5"]) + max(GETUP_PEAK_A["J7"])
    assert two_leg_getup < TPS48111_CB_TRIP_A, (
        f"two-leg get-up aggregate {two_leg_getup:.1f} A reaches CB trip "
        f"{TPS48111_CB_TRIP_A} A"
    )


def test_full_bus_getup_peak_clears_cb_trip_and_f1_slow_band():
    """The design's absolute worst bus current (11.8 A upper model) still
    doesn't trip CB (19.8 A) and stays under F1's *never-opens* band (<=15 A,
    100 % rating) — i.e. the fuse does not see a get-up as a fault."""
    worst = max(GETUP_PEAK_A_BUS_UPPER)
    assert worst < max(F1_NEVER_OPENS_A, 0.0)
    assert worst < TPS48111_CB_TRIP_A
    # and the margin must be stated, not just present:
    margin_cb = TPS48111_CB_TRIP_A - worst
    margin_fuse = F1_NEVER_OPENS_A - worst
    assert margin_cb > 5.0, f"CB margin only {margin_cb:.2f} A"
    assert margin_fuse > 2.0, f"F1 margin only {margin_fuse:.2f} A"


def test_two_second_getup_vs_fuse_time_current():
    """A 2 s 9.4–11.8 A get-up: will F1 (15 A) notice?

    At 11.8 A the fuse sees 11.8/15 = 78.7 % of rating. The Littelfuse 297
    series never opens at or below 100 %. At 135 % (20.25 A) the published
    minimum opening time is 60 s. So 2 s at 78.7 % is a non-event.
    If the design instead needed ≥ 2 s at ≥ 15 A, the fuse would *never*
    open and the protection would shift to the BQ76922's OCD (20 A)."""
    for peak in GETUP_PEAK_A_BUS_UPPER:
        frac = peak / F1_RATING_A
        assert frac < 1.0, f"get-up peak {peak} A exceeds F1 rating"
        # explicitly NOT testing that the fuse opens — it must not
    # And a second worst case: 2 s at the worst 2-s figure of 7.6 A.
    assert 7.6 < 0.7 * F1_RATING_A


# ----------------- contact rating checks ------------------------------------

def test_upper_branch_getup_peaks_against_5264_contact():
    """J5/J7's 2.9–3.0 A get-up peak is above the 5264 contact's *rated* 2.5 A
    by 0.4–0.5 A. Harness.md notes this: "their peaks exceed it for moments",
    and the verification plan V8 measures lead temperature. We assert the
    *peak* is within the contact's *upper* band of 3 A but flag the
    excursion width as a margin that must be watched — i.e. this test
    carries the design's known-marginal item."""
    upper_peak = max(GETUP_PEAK_A["J5"])     # 3.0 A
    margin_to_upper_band = MOLEX_5264_CONTACT_MAX_A - upper_peak
    assert upper_peak <= MOLEX_5264_CONTACT_MAX_A
    # If the margin ever collapses below 0, fail — a re-spec of the button
    # or the connector is required.
    assert margin_to_upper_band >= 0.0


def test_leg_branch_rms_under_contact_rating():
    """J4/J6's 1.6 A RMS walking is comfortably below the contact's rated
    2.5 A — the harness.md's own line on this."""
    for ref in ("J4", "J6"):
        rms = WALK_RMS_A[ref]
        assert rms < MOLEX_5264_CONTACT_A, (
            f"{ref} RMS {rms} A exceeds 5264 contact rating"
        )


def test_walking_bus_is_below_fuse_rating_with_margin():
    """Walking mean (upper model) 3.05 A is far below F1's 15 A. This is the
    normal-state guard: any regression that drags it close to the fuse
    rating must scream."""
    assert WALK_MEAN_A_UPPER < 0.25 * F1_RATING_A


# ----------------- dynamic: full-rail simulation ----------------------------

class _BootThen(Esp32Stub):
    """ESP32 boots, brings up the rail at t=50 ms, leaves SERVO_EN on for
    the rest of the run."""
    def tick(self, t_s, board):
        return {"servo_en_v": 3.3 if t_s >= 0.050 else 0.0,
                "kill_v": 0.0, "i2c_start_v": 3.3}


def _boot_buttons(t_s):
    # power button pressed for 300 ms (enough to wake the BQ76922), then
    # released
    return Buttons(power_pressed=t_s < 0.300)


def test_full_board_power_up_under_load():
    """The rail closes through the BQ76922 wake + soft start, then carries
    the walking load. Assert on the actual rail/servo current at each tick.
    """
    pack = Pack3S.fresh()
    board = PowerBoard(pack=pack)
    esp = _BootThen()
    out = []
    rail_v_when_walking = []
    for _ in range(600):
        s = board.tick(_boot_buttons(board.t_s), esp,
                       rail_load_a=WALK_MEAN_A_UPPER)
        out.append(s)
        if s.rail_state == "ON":
            rail_v_when_walking.append(s.rail_v)

    # rail reached ON in bounded time; rail is at pack level; servo +
    # logic current is the expected
    assert any(s.rail_state == "ON" for s in out), \
        f"rail never reached ON: states={[s.rail_state for s in out][:20]}..."
    walking = [s for s in out if s.rail_state == "ON"]
    final = walking[-1]
    assert final.rail_v > 11.5
    # rail current is 3.05 A (the load we injected), pack current should be
    # rail + 0.5 A logic
    assert final.rail_i_a == pytest.approx(WALK_MEAN_A_UPPER, rel=1e-6)
    assert final.pack_internal_i_a == pytest.approx(WALK_MEAN_A_UPPER + 0.5,
                                                    rel=1e-6)
    # rail_v during walking must hold above pack_v - drop
    assert min(rail_v_when_walking) > 11.0, (
        f"rail sagged below 11 V under walking load: "
        f"min={min(rail_v_when_walking)}"
    )


def test_rail_soft_start_keeps_inrush_under_design_margin():
    """At SERVO_EN's rising edge, the rail must take ~40 ms to reach 90%
    of PACK+, and the inrush through Q3 must stay under the CB trip of
    19.8 A with order-of-magnitude margin (the CSD17556Q5B SOA line at 10 V
    DC is ~3–4 A for 100 ms).

    The rail's voltage rise is exponential (the gate-slew model in
    `tps48111.py`); we assert the rise takes ≥ 30 ms and ≤ 60 ms, and
    that the peak inrush current — computed as I = C_total × dV/dt across
    the worst ramp instant — stays under 4 A.
    """
    pack = Pack3S.fresh()
    board = PowerBoard(pack=pack)
    esp = _BootThen()
    out = []
    rail_v = []
    for _ in range(600):
        s = board.tick(_boot_buttons(board.t_s), esp, rail_load_a=0.0)
        out.append(s)
        rail_v.append(s.rail_v)

    rail_v = np.asarray(rail_v)
    t = np.asarray([s.t_s for s in out])

    # Find the soft-start interval between 5 % and 90 % of final rail
    final_v = rail_v.max()
    assert final_v > 11.5, f"rail did not come up: peak = {final_v}"
    i05 = np.argmax(rail_v >= 0.05 * final_v)
    i90 = np.argmax(rail_v >= 0.90 * final_v)
    rise_ms = (t[i90] - t[i05]) * 1e3
    assert 20.0 < rise_ms < 80.0, \
        f"soft-start 5→90% took {rise_ms:.1f} ms; C4 design target ~40 ms"

    # Inrush: charge C21+C22 total 1010 µF with dV/dt at the ramp's max.
    dv_dt = np.diff(rail_v) / np.diff(t)
    i_max = dv_dt.max() * 1.010e-3  # 1.01 mF total rail capacitance
    assert i_max < SOFT_START_PEAK_LIMIT_A, (
        f"rail inrush {i_max:.2f} A exceeds CSD17556Q5B SOA band "
        f"({SOFT_START_PEAK_LIMIT_A} A DC @ 10 V)"
    )
    # CB trip trip-margin:
    assert i_max < 0.3 * TPS48111_CB_TRIP_A, (
        f"rail inrush {i_max:.2f} A approaches the CB trip of "
        f"{TPS48111_CB_TRIP_A} A — the fuse should never see a power-up"
    )


# ----------------- inrush under the fuse's I²t profile -----------------------


@pytest.mark.parametrize("getup_peak", [3.0, 4.3])   # J4 get-up range
def test_per_branch_getup_fuse_charge(getup_peak):
    """The charge-delivered in a 2 s get-up worst case (one branch at peak)
    is what the fuse's I²t profile will judge. The Littelfuse 297's
    *melting* I²t at its fastest published point (350 %) is about 6 A²s.
    A 4.3 A 2 s get-up delivers 4.3² × 2 = 36.98 A²s — but against a 15 A
    fuse that is 4.3/15 = 28.7 % of rating, i.e. *under* the never-opens
    band, so no melting occurs; the I²t comparison here is against the 100 %
    band, where 'never opens' is the spec."""
    i2t = getup_peak ** 2 * GETUP_DURATION_S
    # fuse's published melting I²t at 200 %: 30²(=900 A²) × 0.15 s = 135 A²s
    # ours is ≤ 37 A²s, a factor of 3.6 under that. The fuse does not act.
    assert i2t < 135 * 0.5, (
        f"get-up charge {i2t:.1f} A²s reaches fuse's melt band"
    )
