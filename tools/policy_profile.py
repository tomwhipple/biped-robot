"""Policy profiles: which compiled-in policy a firmware build carries.

The DEPLOYED policy's generated files live where they always have:

    firmware/components/obs/include/obs/obs_spec.h
    firmware/components/policy/include/policy/weights.h
    firmware/host/vectors/{obs,policy}_vectors.h
    sim/sil/golden/*.json

A named PROFILE is a second, complete set of the same generated files, for a
plant the deployed policy does not drive (v6legs: the 12-joint legs-only robot,
while the deployed net is the 10-joint v5 prototype's):

    firmware/policies/<profile>/obs/obs_spec.h
    firmware/policies/<profile>/policy/weights.h
    firmware/policies/<profile>/vectors/{obs,policy}_vectors.h
    sim/sil/golden/<profile>/*.json

A build selects one with BIMO_POLICY=<profile> (`make -C firmware/host
POLICY=<profile>`, `idf.py -DBIMO_POLICY=<profile>`, `make firmware
POLICY=<profile>`): the profile directory goes FIRST on the include path and
the build defines BIMO_POLICY_OVERLAY and BIMO_POLICY_PROFILE_<NAME>. Every
generated header carries a guard against the other kind of build (guard_lines
below), so an include-path mistake is a compile error, never the wrong policy.

The generators take --profile; python that reads a spec (sim/sil/harness.py)
follows $BIMO_POLICY.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIRMWARE = os.path.join(ROOT, "firmware")
POLICIES = os.path.join(FIRMWARE, "policies")
GOLDEN = os.path.join(ROOT, "sim", "sil", "golden")

_NAME = re.compile(r"^[a-z0-9_]+$")


def check(profile):
    """None for the deployed set, else a validated profile name."""
    if not profile:
        return None
    if not _NAME.match(profile):
        raise SystemExit(f"policy profile {profile!r}: lower-case letters, "
                         "digits and _ only (it becomes a C macro name)")
    return profile


def from_env():
    return check(os.environ.get("BIMO_POLICY") or None)


def macro(profile):
    return "BIMO_POLICY_PROFILE_" + profile.upper()


def files(profile=None):
    """{spec_h, weights_h, obs_vectors_h, policy_vectors_h, golden_dir}."""
    profile = check(profile)
    if profile is None:
        return dict(
            spec_h=os.path.join(FIRMWARE, "components", "obs", "include",
                                "obs", "obs_spec.h"),
            weights_h=os.path.join(FIRMWARE, "components", "policy",
                                   "include", "policy", "weights.h"),
            obs_vectors_h=os.path.join(FIRMWARE, "host", "vectors",
                                       "obs_vectors.h"),
            policy_vectors_h=os.path.join(FIRMWARE, "host", "vectors",
                                          "policy_vectors.h"),
            golden_dir=GOLDEN)
    d = os.path.join(POLICIES, profile)
    return dict(
        spec_h=os.path.join(d, "obs", "obs_spec.h"),
        weights_h=os.path.join(d, "policy", "weights.h"),
        obs_vectors_h=os.path.join(d, "vectors", "obs_vectors.h"),
        policy_vectors_h=os.path.join(d, "vectors", "policy_vectors.h"),
        golden_dir=os.path.join(GOLDEN, profile))


def list_profiles():
    if not os.path.isdir(POLICIES):
        return []
    return sorted(p for p in os.listdir(POLICIES)
                  if _NAME.match(p)
                  and os.path.exists(files(p)["spec_h"]))


def guard_lines(profile, what):
    """The preprocessor guard every generated header of a set carries.

    The deployed set refuses a profile build (BIMO_POLICY_OVERLAY defined:
    the profile's copy should have been found first); a profile's set refuses
    any build that did not select it."""
    profile = check(profile)
    if profile is None:
        return [
            "#ifdef BIMO_POLICY_OVERLAY",
            '#error "this is the DEPLOYED %s, but the build selected a policy '
            'profile (BIMO_POLICY): its firmware/policies/<profile> directory '
            'must come first on the include path"' % what,
            "#endif",
        ]
    return [
        "#ifndef %s" % macro(profile),
        '#error "this %s is policy profile %s (firmware/policies/%s): build '
        'with BIMO_POLICY=%s (make POLICY=%s, idf.py -DBIMO_POLICY=%s)"'
        % (what, profile, profile, profile, profile, profile),
        "#endif",
    ]
