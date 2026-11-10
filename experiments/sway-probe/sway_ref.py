"""The sway-probe reference: the TRAINING imitation reference, played as an amplitude ladder.

Nothing here re-derives the reference. `walker_env.BimoWalkerEnv._mimic_ref(cmd, phase, freq)`
(the CPU mirror of sim/mjx/env_mjx.py's; `tests/test_mimic_sway.py` pins the two together) and
`walker_env.mimic_sway_coefs` give the joint targets for a gait-clock phase; this module only
chooses the command, the phase(t) and the envelope, and writes 50 Hz joint-target keyframes.

    rung kind   what it is
    sway        both feet planted: the reference's two ROLL channels (hip roll + the ankle roll that
                cancels it, so the soles stay level), every other joint at the standing pose
    lift        the whole reference at the same phase: the roll sway AND the swing lift
                (`mimic_lift`, the IK lift of the swing sole) of the recipe, stepping in place

The command that makes the reference move without walking is a yaw command just over the
reference's "moving" gate (|c2| > 0.05): the stride term is then 1.4 * 0.028 * 0.051 / f = 0.0027
rad on the hip pitch, 0.15 deg, and the lift and the sway run at full amplitude (the gate is a
step). That is a pivot-in-place command of the recipe's own command space, not a new mode.

Clock: phase(t) = phase0 + 2 pi f t; the left foot swings while sin(phase) > 0 (the env's
convention). The torso sway is -M sin(phase + lead): it reaches the stance side (the foot that
is not swinging) `lead` radians before the swing peaks. phase0 defaults to -lead, so the sway
starts at its zero crossing.

Envelope: q(t) = stand + w(t) * (ref(phase(t)) - stand), w a minimum-jerk ramp 0 -> 1 over
`ramp_s`, 1 for `cycles` clock periods, 1 -> 0 over `ramp_s`. Where w == 1 the keyframe IS the
training reference (tests/test_sway_probe_ref.py).

    .venv/bin/python experiments/sway-probe/sway_ref.py            # the default ladder, one line a rung
    .venv/bin/python experiments/sway-probe/sway_ref.py --npz out.npz
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for p in (os.path.join(ROOT, "sim", "mjx"), os.path.join(ROOT, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

DEFAULT_CONFIG = os.path.join(HERE, "recipe_prec45_ws1_only.config.json")
DEFAULT_XML = os.path.join(ROOT, "sim", "bimo_biped_v6legs_meas_sole.xml")
KEY_HZ = 50.0                      # the control tick the keyframes are written at
CLOCK_HZ = 0.75                    # the recipe's gait clock is U(0.7, 0.8) Hz
LEAD = 0.785                       # rad: train_mjx --mimic-sway-lead's default
# the plant's pendulum estimate of the sway at 0.75 Hz (PR #257) is ~3.9 cm; the hardware needed ~1.3x the model's lateral shift to unload a foot (hw_sessions/2026-10-25/NOTES.md = 5.1 cm): the rungs bracket it
LADDER_M = (0.020, 0.030, 0.039, 0.045, 0.050, 0.056)
CYCLES = 4
RAMP_S = 1.5
# a yaw command just over the reference's moving gate: sway and lift at full strength, ~0.15 deg of stride
CMD_IN_PLACE = np.array([0.0, 0.0, 0.051, 1.0, 0.0, 0.0, 0.0])


def minjerk(s):
    """Quintic minimum-jerk 0 -> 1 on s in [0, 1] (the firmware streamer's profile)."""
    s = np.clip(np.asarray(s, dtype=float), 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


@dataclass
class Rung:
    index: int                    # 1-based, ladder order
    kind: str                     # "sway" | "lift"
    M: float                      # m, the recipe's --mimic-sway
    lead: float                   # rad
    hz: float                     # gait clock
    cycles: int
    ramp_s: float
    t: np.ndarray                 # (n,) s, 50 Hz from 0
    q: np.ndarray                 # (n, 12) rad, env joint order (names)
    w: np.ndarray                 # (n,) envelope, 1 on the plateau
    phase: np.ndarray             # (n,) rad, the gait clock the reference was evaluated at
    names: tuple

    @property
    def label(self):
        return f"{self.index}:{self.kind}@{self.M * 100:.1f}cm"

    def plateau(self):
        return self.w >= 1.0 - 1e-9

    def meta(self):
        return dict(index=self.index, kind=self.kind, M_m=self.M, lead_rad=self.lead, hz=self.hz,
                    cycles=self.cycles, ramp_s=self.ramp_s, duration_s=float(self.t[-1]), frames=len(self.t))


def load_config(path=None):
    with open(path or DEFAULT_CONFIG) as f:
        return json.load(f)


def make_env(cfg=None, xml=None, M=0.04, lead=LEAD, extra=None):
    """The CPU env, built from a recipe's config.json the way the referee builds it (nominal
    plant): the same BimoWalkerEnv class the training reference is mirrored in."""
    import eval_precision as EP
    kw = dict(mimic_sway=M, mimic_sway_lead=lead)
    kw.update(extra or {})
    return EP.make_env(cfg or load_config(), 10.0, True, xml or DEFAULT_XML, extra=kw)


class SwayReference:
    """Evaluates the training reference on a clock. One env, M and lead set per rung."""

    def __init__(self, cfg=None, xml=None):
        self.cfg = cfg or load_config()
        self.env = make_env(self.cfg, xml)
        self.names = tuple(self.env.model.joint(int(j)).name for j in self._joint_ids())
        self.stand = np.asarray(self.env._default, dtype=float).copy()
        if self.env.mimic_lift <= 0.0:
            raise ValueError("the recipe has no mimic_lift: the lift rungs have nothing to play")
        roll = list(self.env._i_roll) + list(self.env._i_aroll)
        self.roll_idx = np.array(sorted(roll))
        self.joint_dof = np.array([self.env.model.jnt_qposadr[int(j)] for j in self._joint_ids()])

    def _joint_ids(self):
        env = self.env
        m = env.model
        # env joint order = qpos7.. : the 12 leg joints, in the env's _jqpos slice
        adr = np.arange(m.nq)[env._jqpos]
        return [j for a in adr for j in range(m.njnt) if m.jnt_qposadr[j] == a]

    def ref(self, phase, M, lead, kind="lift", hz=CLOCK_HZ):
        """The 12 joint targets (rad, env joint order) of the reference at gait-clock `phase`."""
        env = self.env
        env.mimic_sway, env.mimic_sway_lead = float(M), float(lead)
        q = np.asarray(env._mimic_ref(CMD_IN_PLACE, float(phase), hz), dtype=float)
        if kind == "lift":
            return q
        if kind != "sway":
            raise ValueError(f"kind {kind!r}: sway | lift")
        out = self.stand.copy()
        out[self.roll_idx] = q[self.roll_idx]       # the reference's own roll channels, nothing else
        return out

    def rung(self, index, kind, M, lead=LEAD, hz=CLOCK_HZ, cycles=CYCLES, ramp_s=RAMP_S, phase0=None):
        if phase0 is None:
            phase0 = -lead
        dur = 2 * ramp_s + cycles / hz
        n = int(round(dur * KEY_HZ)) + 1
        t = np.arange(n) / KEY_HZ
        w = minjerk(t / ramp_s) * minjerk((dur - t) / ramp_s)
        w[(t >= ramp_s - 1e-9) & (t <= dur - ramp_s + 1e-9)] = 1.0
        phase = phase0 + 2 * math.pi * hz * t
        q = np.empty((n, len(self.stand)))
        for k in range(n):
            q[k] = self.stand + w[k] * (self.ref(phase[k], M, lead, kind, hz) - self.stand)
        return Rung(index, kind, float(M), float(lead), float(hz), int(cycles), float(ramp_s),
                    t, q, w, phase, self.names)

    def ladder(self, Ms=LADDER_M, lead=LEAD, hz=CLOCK_HZ, cycles=CYCLES, ramp_s=RAMP_S):
        """Sway-only rungs at increasing M, then sway-with-lift rungs at the same Ms."""
        rungs = []
        for kind in ("sway", "lift"):
            for M in Ms:
                rungs.append(self.rung(len(rungs) + 1, kind, M, lead, hz, cycles, ramp_s))
        return rungs


def swing_side(phase):
    """The foot that swings at gait-clock phase(s): "L" while sin(phase) > 0, else "R"."""
    return np.where(np.sin(phase) > 0.0, "L", "R")


def parse_Ms(text):
    return tuple(float(x) for x in text.split(","))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", default=None, help="a recipe's config.json (default: the committed prec45 copy)")
    ap.add_argument("--M", type=parse_Ms, default=LADDER_M, help="sway amplitudes, m, comma list")
    ap.add_argument("--lead", type=float, default=LEAD)
    ap.add_argument("--hz", type=float, default=CLOCK_HZ)
    ap.add_argument("--cycles", type=int, default=CYCLES)
    ap.add_argument("--ramp", type=float, default=RAMP_S)
    ap.add_argument("--npz", default=None, help="write every rung's keyframes to this .npz")
    a = ap.parse_args(argv)
    ref = SwayReference(load_config(a.config))
    rungs = ref.ladder(a.M, a.lead, a.hz, a.cycles, a.ramp)
    for r in rungs:
        roll = np.degrees(r.q[:, ref.roll_idx])
        print(f"rung {r.index:2d} {r.kind:5s} M {r.M * 100:4.1f} cm lead {r.lead:.3f} rad  {r.t[-1]:5.2f} s "
              f"{len(r.t)} keyframes  roll joints peak {np.abs(roll).max():5.2f} deg")
    if a.npz:
        np.savez(a.npz, names=np.array(ref.names), **{f"rung{r.index}_{k}": getattr(r, k)
                                                      for r in rungs for k in ("t", "q", "w", "phase")})
        print("wrote", a.npz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
