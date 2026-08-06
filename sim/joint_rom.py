"""THE single source of truth for joint range of motion.

Everything that states a joint limit is downstream of this table:

    sim/joint_rom.py                    <-- you are here, the declaration
      -> sim/bimo_biped_v5body.xml      joint range= and actuator ctrlrange=
      -> firmware/main/mech_envelope.h  kMechLo / kMechHi (bench truth)
      -> firmware/.../obs_spec.h        kJointLo / kJointHi (GENERATED from
                                        the MJCF by tools/gen_obs_spec.py)
      -> cad/dimensions.py ROM          CAD clearance sweeps

tests/test_joint_rom.py asserts every one of those agrees with this file, so
a measurement applied in one place and forgotten in the others fails the
suite instead of silently shipping.

WHY THIS EXISTS. On 2026-08-02 every servo's ROM was measured on the bench.
The numbers reached mech_envelope.h and cad/dimensions.py, and v4rom widened
the MJCF's mechanical *stops* -- but the MJCF's *ctrlrange*, which is what a
policy may actually command, was deliberately deferred to "a separate commit"
that never happened. obs_spec.h is generated from those ctrlranges, so the
firmware faithfully inherited the stale limits too. Net effect: the knee was
measured at +-95 and the policy was still capped at +5 for four days, across
three training nights, and nothing failed. That is the failure mode this
file is meant to make impossible.

CONVENTIONS, because the three consumers do not share one:

  * Angles here are PHYSICAL, per joint, in degrees, in the CAD's +Y-rotation
    sense -- the same convention cad/dimensions.py ROM uses.
  * The MJCF's knee carries axis="0 -1 0", i.e. the sim's knee angle is the
    NEGATED physical angle. `sim_range()` applies that flip; do not hand-flip
    it at call sites.
  * hip_roll is MIRRORED, not symmetric: each leg adducts (inward) 25 and
    abducts (outward) 55. In sim coordinates +roll abducts the LEFT leg and
    -roll abducts the RIGHT, so the per-leg ranges are asymmetric and are
    mirror images of each other. Stating one number for "hip_roll" is what
    made this joint easy to get wrong.
"""

# physical degrees: (inward/negative extreme, outward/positive extreme)
ROM = {
    "hip_yaw":   (-45.0, 45.0),
    "hip_pitch": (-110.0, 90.0),   # -110 forward, +90 backward (verified 08-03)
    "knee":      (-95.0, 95.0),    # +-95 measured 2026-08-02 (servo 3 sweeps)
    "ankle":     (-40.0, 40.0),
}

# joints whose sim angle is the negated physical angle (MJCF axis is -1)
FLIPPED = {"knee"}

# mirrored joints: (adduction, abduction) in physical degrees
HIP_ROLL_ADDUCT = 25.0
HIP_ROLL_ABDUCT = 55.0

JOINT_ORDER = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")


def sim_range(joint, side):
    """(lo, hi) in SIM degrees for `joint` on side 'L' or 'R'."""
    if side not in ("L", "R"):
        raise ValueError(f"side must be L or R, got {side!r}")

    if joint == "hip_roll":
        # abduction is +roll on the left, -roll on the right
        if side == "L":
            return (-HIP_ROLL_ADDUCT, HIP_ROLL_ABDUCT)
        return (-HIP_ROLL_ABDUCT, HIP_ROLL_ADDUCT)

    lo, hi = ROM[joint]
    if joint in FLIPPED:
        return (-hi, -lo)
    return (lo, hi)


def all_sim_ranges():
    """{'L_knee': (lo, hi), ...} in sim degrees, in actuator order."""
    return {f"{side}_{j}": sim_range(j, side)
            for side in ("L", "R") for j in JOINT_ORDER}
