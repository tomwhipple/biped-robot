"""Alias between the no-appendage study's ``skid_h`` field name and the
DesignParams field main actually carries.

When PR #64 (the no-appendage get-up search) was rebased onto main, main
already had its own ``skid`` fields (the round-3 / as-drawn body's rounded
shell — see ``_skid()`` in ``gen_plant_v6.py``). The two parameterisations
describe the SAME physical part with opposite sign conventions:

- main's ``skid_bot``  = height of the skid's bottom REL to the torso origin
  (yaw joint axis); 0 = at hip-yaw level, negative = below the yaw axis.
- PR #64's ``skid_h``  = skid bottom height BELOW the yaw axis (positive).

So ``skid_bot = -skid_h``. Main's capsule plant also defaults ``skid_x=-0.02``
and ``skid_len=0.11`` (vs this study's ``skid_x=-0.02`` / ``skid_len=0.08``
sweep); we keep the call sites self-describing by having them pass the fields
explicitly.

This module is the ONLY place the translation is done — the scripts keep
their original ``skid_h`` keyword and read naturally.
"""
from __future__ import annotations

import dataclasses as dc

from gen_plant_v6 import DesignParams


def replace_skid_h(base: DesignParams, *, skid_h: float, skid_x: float,
                   skid_len: float, skid_w: float = 0.10,
                   skid_mass: float = 0.030) -> DesignParams:
    """Return ``base`` with the passive pelvis skid enabled and configured.

    Translates the no-appendage study's ``skid_h`` into main's ``skid_bot``
    (``= -skid_h``) so the same ``build_xml(p)`` produces the physical part
    the study intends.
    """
    return dc.replace(
        base,
        skid=True,
        skid_bot=-skid_h,
        skid_x=skid_x,
        skid_len=skid_len,
        skid_w=skid_w,
        skid_mass=skid_mass,
    )
