"""Policy sets: which compiled-in policy a firmware build carries.

What is committed is each set's INPUTS, never its generated files:

    firmware/deployed/policies.txt          the sets: name, run, net format
    firmware/deployed/runs/<run>/           config.json + params.pkl, copied
                                            from sim/runs/<run>/ by `make deploy`
    firmware/deployed/q8/<set>.npz          an int8 set's quantized kernels
                                            (GPTQ on CPU-plant rollouts at
                                            deploy time: the plant moves on,
                                            so the rounding is kept, not redone)
    firmware/deployed/headers.sha256        the bytes of the generated headers
                                            as deployed, which every rebuild
                                            must reproduce

`make policy` generates every set into build/policy/<set>/ (gitignored):

    obs/obs_spec.h                 tools/gen_obs_spec.py
    vectors/obs_vectors.h
    linkproto/cmd_envelope.{h,py}  tools/gen_envelope.py
    vectors/cmd_envelope_vectors.h
    weights/<run>.silw(.json)      tools/export_policy_weights.py
    golden/{obs,policy}_vectors.json
    policy/weights.h               tools/gen_policy_weights.py
    vectors/policy_vectors.h

The set named `deployed` is what a plain build flashes. Any other set is a
policy PROFILE (v6legs, v6legs256: walk policies for the 12-joint legs-only
robot), selected with BIMO_POLICY=<profile> (`make -C firmware/host
POLICY=<profile>`, `idf.py -DBIMO_POLICY=<profile>`, `make firmware
POLICY=<profile>`): its directory goes FIRST on the include path and the
build defines BIMO_POLICY_OVERLAY and BIMO_POLICY_PROFILE_<NAME>. Every
generated header carries a guard against the other kind of build (guard_lines
below), so an include-path mistake is a compile error, never the wrong policy.

The generators take --profile; python that reads a spec (sim/sil/harness.py)
or the command envelope (link/protocol.py) follows $BIMO_POLICY.

    python tools/policy_profile.py gen envelope|net <set>    (what make runs)
    python tools/policy_profile.py deploy --run <run> [--profile <set>] [--quant int8]
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIRMWARE = os.path.join(ROOT, "firmware")
# the committed inputs
INPUTS = os.path.join(FIRMWARE, "deployed")
MANIFEST = os.path.join(INPUTS, "policies.txt")
RUN_INPUTS = os.path.join(INPUTS, "runs")
Q8_INPUTS = os.path.join(INPUTS, "q8")
PINS = os.path.join(INPUTS, "headers.sha256")
# the generated sets
BUILD = os.path.join(ROOT, "build", "policy")
DEPLOYED = "deployed"
RUN_FILES = ("config.json", "params.pkl")

_NAME = re.compile(r"^[a-z0-9_]+$")


def check(profile):
    """None for the deployed set, else a validated profile name."""
    if not profile or profile == DEPLOYED:
        return None
    if not _NAME.match(profile):
        raise SystemExit(f"policy profile {profile!r}: lower-case letters, "
                         "digits and _ only (it becomes a C macro name)")
    return profile


def from_env():
    return check(os.environ.get("BIMO_POLICY") or None)


def macro(profile):
    return "BIMO_POLICY_PROFILE_" + profile.upper()


def set_name(profile=None):
    """The set's name in the manifest and under build/policy/."""
    return check(profile) or DEPLOYED


def set_dir(profile=None):
    return os.path.join(BUILD, set_name(profile))


def files(profile=None):
    """{spec_h, weights_h, obs_vectors_h, policy_vectors_h, golden_dir,
    weights_dir, envelope_h, envelope_py, envelope_vectors_h}: where `make
    policy` writes the set (build/policy/<set>/)."""
    d = set_dir(profile)
    return dict(
        spec_h=os.path.join(d, "obs", "obs_spec.h"),
        weights_h=os.path.join(d, "policy", "weights.h"),
        obs_vectors_h=os.path.join(d, "vectors", "obs_vectors.h"),
        policy_vectors_h=os.path.join(d, "vectors", "policy_vectors.h"),
        envelope_h=os.path.join(d, "linkproto", "cmd_envelope.h"),
        envelope_py=os.path.join(d, "linkproto", "cmd_envelope.py"),
        envelope_vectors_h=os.path.join(d, "vectors", "cmd_envelope_vectors.h"),
        golden_dir=os.path.join(d, "golden"),
        weights_dir=os.path.join(d, "weights"))


def read_manifest(path=MANIFEST):
    """{set: (run, quant)} in file order."""
    out = {}
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.split("#", 1)[0].split()
            if not line:
                continue
            if len(line) != 3 or line[2] not in ("float", "int8"):
                raise SystemExit(f"{path}:{n}: want `<set> <run> float|int8`")
            if line[0] != DEPLOYED:
                check(line[0])
            out[line[0]] = (line[1], line[2])
    return out


def list_profiles():
    """The policy profiles (every set but the deployed one), sorted."""
    return sorted(s for s in read_manifest() if s != DEPLOYED)


def run_inputs(run):
    """firmware/deployed/runs/<run>/: the committed config.json + params.pkl."""
    return os.path.join(RUN_INPUTS, run)


def q8_path(profile):
    return os.path.join(Q8_INPUTS, set_name(profile) + ".npz")


def guard_lines(profile, what):
    """The preprocessor guard every generated header of a set carries.

    Its text still names firmware/policies/, where the sets lived when the
    headers were committed: the deployed headers' bytes are pinned
    (headers.sha256): a change here needs `make deploy` of each set again
    (an int8 set keeps its kernels when redeployed without QUANT=int8).

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


# ---------------------------------------------------------------------
#  generation (make policy) and deployment (make deploy)
# ---------------------------------------------------------------------
def _run(cmd):
    # The goldens are brax forward passes: always on the CPU, never a GPU
    # that a night job may be using.
    env = dict(os.environ, JAX_PLATFORMS="cpu")
    print("+ " + " ".join(os.path.relpath(c, ROOT) if os.path.isabs(c) else c
                          for c in cmd), flush=True)
    subprocess.run(cmd, cwd=ROOT, env=env, check=True)


def _tool(name):
    return [sys.executable, os.path.join(HERE, name)]


# The generated files the firmware compiles in (and link/protocol.py's twin),
# per stage. Their bytes are pinned at deploy time: a rebuild that comes out
# different -- the plant or a generator changed what the run compiles to --
# fails instead of silently flashing something nobody deployed.
PINNED = {"envelope": ("linkproto/cmd_envelope.h", "linkproto/cmd_envelope.py"),
          "net": ("obs/obs_spec.h", "policy/weights.h")}


def _sha256(path):
    import hashlib
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def read_pins(path=PINS):
    """{"<set>/<file>": sha256} (sha256sum's format, paths under build/policy)."""
    out = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                if line.strip() and not line.startswith("#"):
                    h, name = line.split(None, 1)
                    out[name.strip()] = h
    return out


def write_pins(pins, path=PINS):
    with open(path, "w") as f:
        f.write("# sha256 of the generated policy headers as deployed "
                "(tools/policy_profile.py,\n# `make deploy`); `make policy` "
                "checks the files it builds into build/policy/\n# against "
                "these. Never edit by hand.\n")
        for name in sorted(pins):
            f.write("%s  %s\n" % (pins[name], name))


def pin_or_verify(stage, set_, pin):
    pins = read_pins()
    names = ["%s/%s" % (set_, f) for f in PINNED[stage]]
    if pin:
        for n in names:
            pins[n] = _sha256(os.path.join(BUILD, n))
        write_pins(pins)
        return
    bad = []
    for n in names:
        got = _sha256(os.path.join(BUILD, n))
        if pins.get(n) != got:
            bad.append("  build/policy/%s: %s, deployed %s"
                       % (n, got[:16], (pins.get(n) or "(none)")[:16]))
    if bad:
        run = read_manifest()[set_][0]
        raise SystemExit(
            "the generated policy set %r is not the one deployed "
            "(firmware/deployed/headers.sha256):\n%s\nThe run's inputs are "
            "unchanged, so a generator or the plant changed what they "
            "compile to. If that is intended, deploy the run again -- `make "
            "deploy RUN=%s%s` -- and commit firmware/deployed/."
            % (set_, "\n".join(bad), run,
               "" if set_ == DEPLOYED else " POLICY=" + set_))


def gen(stage, set_, q8_save=False, pin=False):
    """Generate one stage of a set from its committed inputs.

    envelope: linkproto/cmd_envelope.{h,py} + its vectors (config.json only;
              what link/protocol.py needs)
    net:      the obs spec, the .silw + goldens, the weights header (needs
              the plant: build/v6/sim)
    q8_save:  (deploy) quantize an int8 set afresh and store its kernels in
              firmware/deployed/q8/<set>.npz instead of reading them there.
    pin:      (deploy) record the PINNED files' bytes instead of checking
              them."""
    sets = read_manifest()
    if set_ not in sets:
        raise SystemExit(f"no set {set_!r} in {os.path.relpath(MANIFEST, ROOT)}"
                         f" (sets: {', '.join(sets)})")
    run, quant = sets[set_]
    rdir = run_inputs(run)
    for f in RUN_FILES:
        if not os.path.exists(os.path.join(rdir, f)):
            raise SystemExit(f"{os.path.relpath(rdir, ROOT)}/{f} missing: "
                             f"`make deploy RUN={run}` copies it from sim/runs")
    prof = [] if set_ == DEPLOYED else ["--profile", set_]
    out = files(None if set_ == DEPLOYED else set_)
    if stage == "envelope":
        _run(_tool("gen_envelope.py") + ["--run", rdir] + prof)
        pin_or_verify(stage, set_, pin)
        return
    if stage != "net":
        raise SystemExit(f"stage {stage!r}: envelope or net")
    silw = os.path.join(out["weights_dir"], run + ".silw")
    _run(_tool("gen_obs_spec.py") + ["--run", rdir] + prof)
    _run(_tool("export_policy_weights.py") + ["--run", rdir, "--out", silw]
         + prof)
    q8 = []
    if quant == "int8":
        q8 = ["--q8-save" if q8_save else "--q8", q8_path(set_)]
    _run(_tool("gen_policy_weights.py") + ["--silw", silw, "--run-dir", rdir,
                                           "--quant", "int8" if quant == "int8"
                                           else "none"] + prof + q8)
    pin_or_verify(stage, set_, pin)


def write_manifest(sets, path=MANIFEST):
    """Rewrite the set lines of the manifest, keeping its comment header."""
    head = []
    with open(path) as f:
        for line in f:
            if line.split("#", 1)[0].strip():
                break
            head.append(line)
    w = max([12] + [len(s) + 2 for s in sets])
    wr = max([32] + [len(r) + 2 for r, _ in sets.values()])
    body = ["%s%s%s\n" % (s.ljust(w), r.ljust(wr), q)
            for s, (r, q) in sets.items()]
    with open(path, "w") as f:
        f.writelines(head + body)


def deploy(run, profile=None, quant=None):
    """Point a set at a run: copy the run's inputs into firmware/deployed/,
    rewrite its manifest line, generate the set (an int8 one quantized on
    today's plant) and pin its headers' bytes. `make deploy` then runs `make
    policy`, which rebuilds it from the committed inputs against the pins."""
    profile = check(profile)
    set_ = set_name(profile)
    src = run if os.sep in run else os.path.join(ROOT, "sim", "runs", run)
    name = os.path.basename(os.path.normpath(src))
    sets = read_manifest()
    # An int8 set is quantized afresh when the deploy says QUANT=int8 or the
    # run changes; re-deploying its run without QUANT keeps the kernels (only
    # the pins are rewritten, e.g. after a generator's text changed).
    requant = quant == "int8" or sets.get(set_, (None,))[0] != name \
        or not os.path.exists(q8_path(set_))
    if quant is None:   # a set that is int8 stays int8
        quant = sets.get(set_, (None, "float"))[1]
    quant = "int8" if quant == "int8" else "float"
    if quant == "int8" and profile is None:
        raise SystemExit("--quant int8 is for a policy PROFILE's net: the "
                         "deployed set stays float")
    dst = run_inputs(name)
    os.makedirs(dst, exist_ok=True)
    for f in RUN_FILES:
        p = os.path.join(src, f)
        if not os.path.exists(p):
            raise SystemExit(f"{p} missing: nothing to deploy")
        q = os.path.join(dst, f)
        if not (os.path.exists(q) and os.path.samefile(p, q)):
            shutil.copy2(p, q)
    sets[set_] = (name, quant)
    if set_ == DEPLOYED:   # the deployed set stays the first line
        sets = {DEPLOYED: sets.pop(DEPLOYED), **sets}
    write_manifest(sets)
    # inputs no set uses any more leave with the deploy
    used = {r for r, _ in sets.values()}
    for r in os.listdir(RUN_INPUTS):
        if r not in used:
            shutil.rmtree(os.path.join(RUN_INPUTS, r))
            print("removed %s" % os.path.relpath(run_inputs(r), ROOT))
    if quant == "float" and os.path.exists(q8_path(set_)):
        os.remove(q8_path(set_))
    pins = {n: h for n, h in read_pins().items() if n.split("/")[0] in sets}
    write_pins(pins)
    print("%s: %s (%s), inputs in %s" % (set_, name, quant,
                                         os.path.relpath(dst, ROOT)))
    gen("envelope", set_, pin=True)
    gen("net", set_, q8_save=quant == "int8" and requant, pin=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen", help="generate a set's stage into build/policy/")
    g.add_argument("stage", choices=("envelope", "net"))
    g.add_argument("set")
    d = sub.add_parser("deploy", help="point a set at a run (make deploy)")
    d.add_argument("--run", required=True,
                   help="a run under sim/runs/, or a run directory")
    d.add_argument("--profile", default=None,
                   help="the set to point (default: the deployed one)")
    d.add_argument("--quant", choices=("float", "none", "int8"), default=None,
                   help="the net format (default: what the set already is)")
    sub.add_parser("sets", help="print the set names, one per line")
    a = ap.parse_args()
    if a.cmd == "gen":
        gen(a.stage, a.set)
    elif a.cmd == "deploy":
        deploy(a.run, a.profile, a.quant)
    else:
        print("\n".join(read_manifest()))


if __name__ == "__main__":
    main()
