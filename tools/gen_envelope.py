#!/usr/bin/env python3
"""Generate the command envelope the robot enforces, from the deployed run.

Run:  .venv/bin/python tools/gen_envelope.py --run <deployed run> [--profile <name>]
      (`make deploy RUN=... [POLICY=<name>]` runs it beside gen_obs_spec.py)

--run is REQUIRED, like gen_obs_spec.py: the envelope is a property of the
policy that is flashed, so it may only change because somebody said which run
to deploy. The run is a name under sim/runs/ or a path to a run directory.

What a run trained on, and where this reads it (docs/training.md section 4):

  forward  vx   U(cmd_v_range)                     config.json cmd_v_range
  backward vx   U(cmd_back_range), <0              code default, NOT in config.json
  lateral  |vy| U(0.1, cmd_vy_range), either sign  code default, NOT in config.json
  turn     |wz| <= cmd_w_range                     config.json cmd_w_range
  crouch   1.0, or cmd_crouch_range, or (for an    config.json cmd_crouch_range,
           ext_mix crouch slice) crouch_range      ext_mix; crouch_range = code default

train_mjx.py has no flag for cmd_back_range, cmd_vy_range or crouch_range, so a
run never records them: they are the defaults of the env's constructor
(sim/walker_env.py BimoWalkerEnv, which sim/mjx/env_mjx.py mirrors; a test pins
that the two agree). They are resolved from those defaults HERE, at generation
time, and recorded with the word "default" in the generated file -- not copied
into this script, where they would drift from the env. The 0.1 m/s floor of the
sidestep draw is a literal in both envs (`uniform(0.1, cmd_vy_range)`); it is
CMD_VY_DRAW_MIN below, and tests/test_cmd_envelope.py greps the env sources so
a change there fails.

Whether a band was trained at all follows the command mix: walk_submix
(backward, sidestep) fractions of the walk commands, ext_cmd for the extended
channels. A band that was never drawn is generated as absent (0, 0): the robot
snaps a command into it to a stand.

Writes, per policy profile (tools/policy_profile.py):

  linkproto/cmd_envelope.h     the firmware's constants (namespace linkproto)
  link/cmd_envelope.py         the Python twin link/protocol.py reads
  vectors/cmd_envelope_vectors.h   the clamp, run through link/protocol.py's
                                   Python on this envelope, for the host test
"""
import argparse
import importlib.util
import inspect
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "link"))
sys.path.insert(0, HERE)

import numpy as np                                     # noqa: E402
import policy_profile                                  # noqa: E402

# env_mjx.py / walker_env.py: `jax.random.uniform(rs[7], minval=0.1, ...)` and
# `rng.uniform(0.1, self.cmd_vy_range)`: the sidestep draw's lower bound.
CMD_VY_DRAW_MIN = 0.1

# The constructor arguments the envelope reads. cfg (config.json) wins; the
# env's own default is used, and recorded as such, when the run did not say.
KEYS = ("ext_cmd", "cmd_v_range", "cmd_w_range", "cmd_back_range",
        "cmd_vy_range", "cmd_crouch_range", "crouch_range", "ext_mix",
        "walk_submix", "sway_vy")


def env_defaults():
    """The default of each KEYS argument of walker_env.BimoWalkerEnv."""
    from walker_env import BimoWalkerEnv
    sig = inspect.signature(BimoWalkerEnv.__init__).parameters
    return {k: sig[k].default for k in KEYS}


def read_source(cfg, defaults=None):
    """{key: value} and {key: "config" | "default"} for the KEYS."""
    defaults = env_defaults() if defaults is None else defaults
    val, where = {}, {}
    for k in KEYS:
        if k in cfg and cfg[k] is not None:
            val[k], where[k] = cfg[k], "config"
        else:
            val[k], where[k] = defaults[k], "default"
        if isinstance(val[k], (list, tuple)):
            val[k] = [float(x) for x in val[k]]
        elif not isinstance(val[k], bool):
            val[k] = float(val[k])
    return val, where


def bands(src):
    """The envelope, a pure function of the resolved training settings.

    Returns {v_fwd_min, v_fwd_max, v_back_min, v_back_max, vy_min, vy_max,
    w_max, crouch_min}: magnitudes, >= 0. A band that was not trained is
    (0, 0)."""
    ext = bool(src["ext_cmd"])
    mix = list(src["ext_mix"]) if ext else []
    p_crouch = mix[1] if len(mix) > 1 else 0.0
    p_sway = mix[6] if len(mix) > 6 else 0.0
    p_back, p_side = src["walk_submix"] if ext else (0.0, 0.0)
    v_lo, v_hi = src["cmd_v_range"]
    out = dict(v_fwd_min=v_lo, v_fwd_max=v_hi, v_back_min=0.0, v_back_max=0.0,
               vy_min=0.0, vy_max=0.0, w_max=src["cmd_w_range"],
               crouch_min=1.0)
    if p_back > 0.0:
        b_lo, b_hi = sorted(abs(x) for x in src["cmd_back_range"])
        out.update(v_back_min=b_lo, v_back_max=b_hi)
    if p_side > 0.0:
        out.update(vy_min=CMD_VY_DRAW_MIN, vy_max=src["cmd_vy_range"])
    if p_sway > 0.0:
        # sway commands an oscillating vy through zero: no hole under it
        out.update(vy_min=0.0, vy_max=max(out["vy_max"], src["sway_vy"]))
    if ext:
        crouch = src["cmd_crouch_range"][0]
        if p_crouch > 0.0:
            crouch = min(crouch, src["crouch_range"][0])
        out["crouch_min"] = crouch
    return out


def load_config(run):
    run_dir = run if os.sep in run else os.path.join(ROOT, "sim", "runs", run)
    path = os.path.join(run_dir, "config.json")
    if not os.path.isfile(path):
        raise SystemExit("no %s: name a run under sim/runs/ or give its "
                         "directory" % path)
    with open(path) as f:
        return os.path.basename(os.path.normpath(run_dir)), json.load(f)


def fl(x):
    s = "%.9g" % float(x)
    if not any(c in s for c in ".eEn"):
        s += ".0"
    return s + "f"


def _fmt(x):
    return "%.9g" % float(x)


def write_header(path, profile, run, band, src, where):
    L = []
    w = L.append
    w("// GENERATED by tools/gen_envelope.py -- do not edit.")
    w("// Source: sim/runs/%s/config.json (+ BimoWalkerEnv defaults where the"
      % run)
    w("// run records nothing: marked `default` below).")
    w("//")
    w("// The command envelope of the deployed policy: the region of commands it")
    w("// was trained on. clampToEnvelope() / clampExtToEnvelope() (protocol.cpp)")
    w("// enforce it; link/cmd_envelope.py is the Python twin. A speed below a")
    w("// band's minimum is a hole in the training distribution, not a slow walk,")
    w("// and is snapped to a stand. A band that was never trained is 0..0.")
    w("#pragma once")
    L.extend(policy_profile.guard_lines(profile, "cmd_envelope.h"))
    w("")
    w("namespace linkproto {")
    w("")
    w('inline constexpr const char* kEnvelopeRun = "%s";' % run)
    w("")
    w("// forward vx in [kVFwdMin, kVFwdMax]       cmd_v_range (%s)"
      % where["cmd_v_range"])
    w("inline constexpr float kVFwdMin = %s;" % fl(band["v_fwd_min"]))
    w("inline constexpr float kVFwdMax = %s;" % fl(band["v_fwd_max"]))
    w("// backward vx in [-kVBackMax, -kVBackMin]  cmd_back_range (%s)"
      % where["cmd_back_range"])
    w("inline constexpr float kVBackMin = %s;" % fl(band["v_back_min"]))
    w("inline constexpr float kVBackMax = %s;" % fl(band["v_back_max"]))
    w("// lateral |vy| in [kVyMin, kVyMax]         cmd_vy_range (%s)"
      % where["cmd_vy_range"])
    w("inline constexpr float kVyMin = %s;" % fl(band["vy_min"]))
    w("inline constexpr float kVyMax = %s;" % fl(band["vy_max"]))
    w("// turn |wz| <= kWMax                       cmd_w_range (%s)"
      % where["cmd_w_range"])
    w("inline constexpr float kWMax = %s;" % fl(band["w_max"]))
    w("// crouch in [kCrouchMin, 1]                cmd_crouch_range (%s)"
      % where["cmd_crouch_range"])
    w("inline constexpr float kCrouchMin = %s;" % fl(band["crouch_min"]))
    w("")
    w("}  // namespace linkproto")
    w("")
    _write(path, L)


def write_py(path, profile, run, band, src, where):
    L = []
    w = L.append
    w("# GENERATED by tools/gen_envelope.py -- do not edit.")
    w("# Source: sim/runs/%s/config.json (+ BimoWalkerEnv defaults where the"
      % run)
    w("# run records nothing: SOURCE_FROM says which).")
    w("#")
    w("# The Python twin of linkproto/cmd_envelope.h: the command envelope of the")
    w("# policy profile %r (\"\" = the deployed set). link/protocol.py reads it;"
      % (profile or ""))
    w("# a profile's twin is chosen by $BIMO_POLICY. Magnitudes, >= 0; a band")
    w("# that was never trained is 0..0.")
    w('PROFILE = "%s"' % (profile or ""))
    w('RUN = "%s"' % run)
    for k in ("v_fwd_min", "v_fwd_max", "v_back_min", "v_back_max",
              "vy_min", "vy_max", "w_max", "crouch_min"):
        w("%s = %r" % (k.upper(), float(band[k])))
    w("")
    w("# The training settings the band was derived from (tools/gen_envelope.py"
      " bands()).")
    w("SOURCE = %r" % (dict(sorted(src.items())),))
    w("SOURCE_FROM = %r" % (dict(sorted(where.items())),))
    w("")
    _write(path, L)


def _write(path, lines):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(lines))


def load_twin(path):
    spec = importlib.util.spec_from_file_location("cmd_envelope_gen", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def f32(x):
    return float(np.float32(x))


def clamp_inputs(band):
    """vx / wz probes: every edge of every band, both sides, the holes, past."""
    vs = [0.0]
    for lo, hi, sgn in ((band["v_fwd_min"], band["v_fwd_max"], 1.0),
                        (band["v_back_min"], band["v_back_max"], -1.0)):
        for m in (lo * 0.5, lo * 0.9, lo, (lo + hi) / 2.0, hi, hi * 1.5,
                  hi + 5.0):
            vs.append(sgn * m)
    vs.extend([9.0, -9.0])
    ws = [0.0, 0.5, band["w_max"], band["w_max"] + 1.0,
          -band["w_max"] - 1.0, -0.7]
    return sorted(set(f32(v) for v in vs)), ws


def ext_inputs(band):
    vys = [0.0]
    lo, hi = band["vy_min"], band["vy_max"]
    for m in (lo * 0.5, lo * 0.9, lo, (lo + hi) / 2.0, hi, hi * 1.5, 5.0):
        vys.extend([m, -m])
    cs = [-1.0, band["crouch_min"] * 0.5, band["crouch_min"],
          (band["crouch_min"] + 1.0) / 2.0, 1.0, 2.0]
    return sorted(set(f32(v) for v in vys)), sorted(set(f32(c) for c in cs))


def write_vectors(path, profile, run, band, env):
    import protocol as P
    L = []
    w = L.append
    w("// GENERATED by tools/gen_envelope.py -- do not edit.")
    w("// link/protocol.py's clamp_to_envelope / clamp_ext_to_envelope on the")
    w("// envelope derived from sim/runs/%s/config.json, run through the Python" % run)
    w("// reference; firmware/host/test_protocol.cpp asserts the C++ port against it.")
    w("#pragma once")
    L.extend(policy_profile.guard_lines(profile, "cmd_envelope_vectors.h"))
    w("#include <stddef.h>")
    w("")
    w("namespace envelope_vectors {")
    w("")
    w("// The bands as derived from the run's training settings. The host test")
    w("// pins linkproto/cmd_envelope.h's constants against these: a hand edit of")
    w("// either file fails it.")
    w("struct Bands { float v_fwd_min, v_fwd_max, v_back_min, v_back_max, "
      "vy_min, vy_max, w_max, crouch_min; };")
    w("inline constexpr Bands kBands = {%s};" % ", ".join(
        fl(band[k]) for k in ("v_fwd_min", "v_fwd_max", "v_back_min",
                              "v_back_max", "vy_min", "vy_max", "w_max",
                              "crouch_min")))
    w("")
    w("struct ClampCase { float vx; float wz; float out_vx; float out_wz; };")
    w("inline const ClampCase kClamp[] = {")
    vs, ws = clamp_inputs(band)
    n = 0
    for vx in vs:
        for wz in ws:
            ov, ow = P.clamp_to_envelope(vx, f32(wz), env)
            w("    {%s, %s, %s, %s}," % (fl(vx), fl(wz), fl(ov), fl(ow)))
            n += 1
    w("};")
    w("inline constexpr size_t kNumClamp = sizeof kClamp / sizeof kClamp[0];")
    w("")
    w("// clamp_ext_to_envelope of the extended channels (vy, crouch, lift, fdx, fdz).")
    w("struct ClampExtCase { float in[5]; float out[5]; };")
    w("inline const ClampExtCase kClampExt[] = {")
    vys, cs = ext_inputs(band)
    rows = [(vy, c, 0.0, 0.0, 0.0) for vy in vys for c in cs]
    # lift and the foot offsets are not from a run's config (protocol.py
    # FOOT_D_MAX), but they ride the same clamp: keep them covered
    rows += [(0.0, 1.0, l, dx, dz) for l, dx, dz in
             ((2.0, 0.2, -0.2), (-2.0, -0.2, 0.2), (0.5, 0.03, 0.045),
              (-0.5, -0.0625, 0.0625))]
    for row in rows:
        row = tuple(f32(v) for v in row)
        d = P.Command(0, 0.0, 0.0, 0, vy=row[0], crouch=row[1], lift=row[2], foot_dx=row[3],
                      foot_dz=row[4])
        out = P.clamp_ext_to_envelope(d, env)
        w("    {{%s}, {%s}}," % (", ".join(fl(v) for v in row),
                                ", ".join(fl(v) for v in out)))
    w("};")
    w("inline constexpr size_t kNumClampExt = "
      "sizeof kClampExt / sizeof kClampExt[0];")
    w("")
    w("}  // namespace envelope_vectors")
    w("")
    _write(path, L)
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", help="run under sim/runs/ (or a run directory) "
                                  "whose config.json defines the envelope")
    ap.add_argument("--profile", default=None,
                    help="write policy profile <name>'s set under "
                         "firmware/policies/<name>/ instead of the deployed "
                         "files (tools/policy_profile.py)")
    args = ap.parse_args()
    profile = policy_profile.check(args.profile)
    out = policy_profile.files(profile)
    if not args.run:
        raise SystemExit(
            "name the run: --run <run under sim/runs/>.  There is no default: "
            "the files this writes ARE the command envelope the flashed policy "
            "is held to (kEnvelopeRun in %s names the run now)."
            % os.path.relpath(out["envelope_h"], ROOT))
    run, cfg = load_config(args.run)
    src, where = read_source(cfg)
    band = bands(src)
    write_header(out["envelope_h"], profile, run, band, src, where)
    write_py(out["envelope_py"], profile, run, band, src, where)
    import protocol as P
    env = P.load_envelope(load_twin(out["envelope_py"]))
    n = write_vectors(out["envelope_vectors_h"], profile, run, band, env)
    print("run       : %s" % run)
    print("profile   : %s" % (profile or "(deployed)"))
    for k, v in band.items():
        print("%-10s: %s" % (k, _fmt(v)))
    print("from      : %s" % ", ".join(
        "%s=%s" % (k, where[k]) for k in ("cmd_v_range", "cmd_back_range",
                                          "cmd_vy_range", "cmd_w_range",
                                          "cmd_crouch_range")))
    print("wrote %s" % out["envelope_h"])
    print("wrote %s" % out["envelope_py"])
    print("wrote %s (%d clamp cases)" % (out["envelope_vectors_h"], n))


if __name__ == "__main__":
    main()
