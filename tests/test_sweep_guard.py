"""tools/sweep_guard.py and its use in tools/rom_sweep.py -- against the 2026-11-02 incident,
where a sweep drove the right hip roll inward into the left leg and broke the hip."""
import os
import re
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import sweep_guard as G  # noqa: E402

rom_sweep = pytest.importorskip("rom_sweep")       # needs pyserial


def test_current_sign_bit():
    assert G.current_ma(100) == pytest.approx(650.0)
    assert G.current_ma(0x8000 | 100) == pytest.approx(-650.0)


def test_inward_hip_roll_is_capped_outward_is_not():
    # sim-positive hip roll moves the toe to the robot's LEFT: the right leg's + is inward
    assert G.cap_angle("R_hip_roll", 30.0) == pytest.approx(G.INWARD_CAP_DEG)
    assert G.cap_angle("R_hip_roll", -45.0) == -45.0
    assert G.cap_angle("L_hip_roll", -30.0) == pytest.approx(-G.INWARD_CAP_DEG)
    assert G.cap_angle("L_hip_roll", 45.0) == 45.0
    assert G.cap_angle("R_hip_roll", 30.0, allow_contact=True) == 30.0
    assert G.cap_angle("R_knee", -90.0) == -90.0
    assert G.INWARD_CAP_DEG < 9.0                  # leg-leg contact (mech_envelope.h)


def test_smooth_gravity_ramp_passes():
    g = G.PowerGuard()
    g.baseline([0.30, 0.31, 0.29])
    for w, load in [(0.33, 20), (0.40, 30), (0.48, 40), (0.57, 50), (0.66, 60), (0.75, 70)]:
        assert g.check(w, load) is None


def test_contact_spike_aborts():
    g = G.PowerGuard()
    g.baseline([0.30, 0.30, 0.30])
    assert g.check(0.34, 20) is None
    assert g.check(0.37, 25) is None
    why = g.check(1.4, 140)                        # a step change on top of the trend
    assert why and "spike over trend" in why


def test_rise_and_absolute_ceilings():
    g = G.PowerGuard(rise_w=0.5, step_w=10.0)
    g.baseline([0.3])
    assert "rise over baseline" in g.check(0.9, 10)
    g = G.PowerGuard(rise_w=10.0, step_w=10.0, abs_w=2.0)
    g.baseline([0.3])
    assert "absolute ceiling" in g.check(2.5, 10)


def test_2026_11_02_load_trace_aborts_when_the_part_yields():
    # id 52 |load| per 5 deg inward: 0, 8, 64, 144, then 56 while still following
    g = G.PowerGuard(step_w=10.0)                  # power alone not decisive here
    g.baseline([0.3])
    for load in (0, 8, 64, 144):
        assert g.check(0.4, load) is None
    why = g.check(0.4, 56, following=True)
    assert why and "load fell 144 -> 56" in why
    # the same drop while NOT following (backing off a contact) is not a yield
    g2 = G.PowerGuard(step_w=10.0)
    g2.baseline([0.3])
    for load in (0, 8, 64, 144):
        g2.check(0.4, load)
    assert g2.check(0.4, 56, following=False) is None


# -- a fake bench: one hanging joint, the other leg at a contact -----------------------

class FakeJoint(rom_sweep.Port):
    """The tools' Port, for one servo. Inward (sim angle toward the other leg) past
    `contact_deg` the leg is blocked: position stops there and the current jumps."""

    def __init__(self, sid, zero, d, start_deg=0.0, contact_deg=9.0, inward=+1):
        self.sid, self.zero, self.d = sid, zero, d
        self.tick = zero + round(d * start_deg * rom_sweep.K)
        self.torque, self.tl, self.contact, self.inward = 0, 1000, contact_deg, inward
        self.blocked, self.max_tl_seen, self.log = False, 1000, []

    def angle(self, tick):
        return (tick - self.zero) * self.d / rom_sweep.K

    def cmd(self, line, until=None, timeout=1.0):
        self.log.append(line)
        a = line.split()
        if a[0] == "pos":
            load = 500 if self.blocked else 20
            return f"id {a[1]}  pos  {self.tick}  spd     0  load  {load}  12.0 V  29 C  err 0x00\n"
        if a[0] == "reg":
            reg = int(a[2])
            v = {40: self.torque, 48: self.tl,
                 69: round((900 if self.blocked else 30) / G.MA_PER_LSB)}.get(reg, 0)
            return f"id {a[1]} reg {reg} = {v} (0x{v:04X})\n"
        if a[0] == "regw":
            addr, v = int(a[2]), int(a[3])
            before = self.tl
            self.tl = (self.tl & 0xFF00) | v if addr == 48 else (v << 8) | (self.tl & 0xFF)
            self.max_tl_seen = max(self.max_tl_seen, self.tl)
            return f"regw id {a[1]} reg {addr}: {before} -> wrote {v} (ok) -> reads {self.tl}\n"
        if a[0] == "torque":
            self.torque = 1
            return f"torque {a[1]}: ok\n"
        if a[0] == "release":
            self.torque = 0
            return f"release {a[1]}: ok\n"
        if a[0] == "move":
            tgt = int(a[2])
            ang = self.angle(tgt)
            if ang * self.inward > self.contact:          # blocked by the other leg
                self.blocked = True
                self.tick = self.zero + round(self.d * self.inward * self.contact * rom_sweep.K)
            else:
                self.blocked = False
                self.tick = tgt
            return f"move id {a[1]}: ok\n"
        if a[0] == "cal":
            return "zero <- 2048\n"
        raise AssertionError(f"unexpected command {line!r}")


def _args(**kw):
    a = dict(step_deg=2.0, speed=100000, max_deg=40.0, load_stop=450, rise_w=1.5, step_w=0.35,
             abs_w=4.0, allow_contact=False)
    a.update(kw)
    return types.SimpleNamespace(**a)


def _sweep(fake, tick_dir, **kw):
    msgs = []
    rec = rom_sweep.sweep(fake, fake.sid, tick_dir, _args(**kw), msgs.append, None, None,
                          ("R_hip_roll", fake.zero, fake.d))
    return rec, msgs


def test_rom_sweep_stops_at_the_inward_cap_before_contact():
    # R_hip_roll: id 52, zero 2082, dir -1; inward = sim + = ticks -1
    fake = FakeJoint(52, 2082, -1)
    rec, _ = _sweep(fake, -1)
    assert "inward cap" in rec["stop"]
    assert not fake.blocked
    assert abs(fake.angle(rec["reach"])) <= G.INWARD_CAP_DEG + 0.1


def test_rom_sweep_outward_is_not_capped():
    fake = FakeJoint(52, 2082, -1)
    rec, _ = _sweep(fake, +1, max_deg=20.0)
    assert rec["stop"] == "range cap"
    assert -20.0 <= fake.angle(rec["reach"]) <= -18.0      # last whole 2 deg step inside the cap


def test_rom_sweep_contact_with_allow_contact_aborts_on_power():
    fake = FakeJoint(52, 2082, -1)
    rec, _ = _sweep(fake, -1, allow_contact=True)
    assert rec["stop"].startswith("power ")
    assert fake.angle(rec["reach"]) <= 9.0 + 0.1
    assert max(s["power_w"] for s in rec["steps"]) > 1.0
    # caught on the power, before the old |load| > 450 ceiling would have had to


def test_torque_limit_lowered_then_restored_never_overshooting():
    fake = FakeJoint(52, 2082, -1)
    assert G.set_torque_limit(fake, 52, 400) == 400
    assert fake.max_tl_seen == 1000              # never above the old value on the way down
    fake.max_tl_seen = 400
    assert G.set_torque_limit(fake, 52, 1000) == 1000
    assert fake.max_tl_seen == 1000
    order = [l for l in fake.log if l.startswith("regw")]
    assert re.match(r"regw 52 49 ", order[0])     # lowering: high byte first
    assert re.match(r"regw 52 48 ", order[2])     # raising: low byte first
