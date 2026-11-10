"""The command envelope is generated from the deployed run (tools/gen_envelope.py).

Run:  .venv/bin/python -m pytest tests/test_cmd_envelope.py -q

Three things are pinned, for the deployed set and for every policy profile:

  * the generated files agree with each other (header, Python twin, vectors) and
    with what the generator writes from the settings the twin records, so a
    hand edit of any one of them fails;
  * those recorded settings give the recorded bands (bands() is a pure function);
  * when the run's gitignored config.json is on this machine (sim/runs/<run>),
    the envelope is exactly what that config derives -- a retrained or swapped
    run whose headers were not regenerated fails. Without the run this check
    skips with a reason, like the SIL suite's.

The clamp itself is exercised on both sides of every band in
firmware/host/test_protocol.cpp (C++) and tests/test_protocol.py (Python) from
the same vectors (firmware/host/vectors/cmd_envelope_vectors.h, which this
file also pins against the twin).
"""
import importlib.util
import json
import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for sub in ("tools", "link", "sim"):
    sys.path.insert(0, os.path.join(ROOT, sub))

pytest.importorskip("numpy")
import gen_envelope                                    # noqa: E402
import policy_profile                                  # noqa: E402
import protocol                                        # noqa: E402

PROFILES = [None] + policy_profile.list_profiles()
FIELDS = ("v_fwd_min", "v_fwd_max", "v_back_min", "v_back_max", "vy_min",
          "vy_max", "w_max", "crouch_min")
HEADER_NAMES = dict(v_fwd_min="kVFwdMin", v_fwd_max="kVFwdMax",
                    v_back_min="kVBackMin", v_back_max="kVBackMax",
                    vy_min="kVyMin", vy_max="kVyMax", w_max="kWMax",
                    crouch_min="kCrouchMin")


def _twin(profile):
    path = policy_profile.files(profile)["envelope_py"]
    spec = importlib.util.spec_from_file_location("twin_" + str(profile), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _header_values(profile):
    text = open(policy_profile.files(profile)["envelope_h"]).read()
    out = {}
    for k, name in HEADER_NAMES.items():
        m = re.search(r"\b%s = ([-0-9.e+]+)f;" % name, text)
        assert m, name
        out[k] = float(m.group(1))
    return out, re.search(r'kEnvelopeRun = "([^"]+)"', text).group(1)


def _vector_bands(profile):
    text = open(policy_profile.files(profile)["envelope_vectors_h"]).read()
    m = re.search(r"kBands = \{([^}]+)\}", text)
    return dict(zip(FIELDS, (float(x.strip().rstrip("f"))
                             for x in m.group(1).split(","))))


def f32(x):
    import numpy as np
    return float(np.float32(x))


@pytest.mark.parametrize("profile", PROFILES, ids=lambda p: p or "deployed")
def test_generated_files_agree(profile):
    twin = _twin(profile)
    hdr, hdr_run = _header_values(profile)
    vec = _vector_bands(profile)
    assert twin.PROFILE == (profile or "")
    assert hdr_run == twin.RUN
    for k in FIELDS:
        tv = getattr(twin, k.upper())
        assert f32(tv) == f32(hdr[k]), k    # the header holds the float32
        assert f32(tv) == f32(vec[k]), k


@pytest.mark.parametrize("profile", PROFILES, ids=lambda p: p or "deployed")
def test_generated_files_are_what_the_generator_writes(profile, tmp_path):
    """Byte for byte: a hand edit (or a generator change without a
    regeneration) shows up as a diff."""
    twin = _twin(profile)
    out = policy_profile.files(profile)
    band = gen_envelope.bands(twin.SOURCE)
    assert band == {k: getattr(twin, k.upper()) for k in FIELDS}
    h, py = tmp_path / "e.h", tmp_path / "cmd_envelope.py"
    gen_envelope.write_header(str(h), profile, twin.RUN, band, twin.SOURCE,
                              twin.SOURCE_FROM)
    gen_envelope.write_py(str(py), profile, twin.RUN, band, twin.SOURCE,
                          twin.SOURCE_FROM)
    assert h.read_text() == open(out["envelope_h"]).read()
    assert py.read_text() == open(out["envelope_py"]).read()
    vec = tmp_path / "v.h"
    env = protocol.load_envelope(gen_envelope.load_twin(str(py)))
    gen_envelope.write_vectors(str(vec), profile, twin.RUN, band, env)
    assert vec.read_text() == open(out["envelope_vectors_h"]).read()


@pytest.mark.parametrize("profile", PROFILES, ids=lambda p: p or "deployed")
def test_envelope_matches_the_runs_config(profile):
    twin = _twin(profile)
    run_dir = os.path.join(ROOT, "sim", "runs", twin.RUN)
    if not os.path.isfile(os.path.join(run_dir, "config.json")):
        pytest.skip("run %s is not on this machine (sim/runs is gitignored): "
                    "the envelope cannot be re-derived from its config.json"
                    % twin.RUN)
    _, cfg = gen_envelope.load_config(run_dir)
    src, where = gen_envelope.read_source(cfg)
    assert src == twin.SOURCE and where == twin.SOURCE_FROM
    assert gen_envelope.bands(src) == {k: getattr(twin, k.upper())
                                       for k in FIELDS}


def test_the_deployed_envelope_is_the_deployed_runs():
    """obs_spec.h and cmd_envelope.h are generated from the same run."""
    spec = open(policy_profile.files()["spec_h"]).read()
    run = re.search(r'kRunName = "([^"]+)"', spec).group(1)
    assert _twin(None).RUN == run
    for p in policy_profile.list_profiles():
        spec = open(policy_profile.files(p)["spec_h"]).read()
        assert _twin(p).RUN == re.search(r'kRunName = "([^"]+)"', spec).group(1)


# -- bands(): the derivation -------------------------------------------------
PREC = dict(ext_cmd=True, cmd_v_range=[0.05, 0.24], cmd_w_range=1.0,
            cmd_crouch_range=[1.0, 1.0], ext_mix=[0.2, 0, 0, 0, 0.15, 0, 0],
            walk_submix=[0.25, 0.25])


def _bands(cfg):
    src, where = gen_envelope.read_source(cfg)
    return gen_envelope.bands(src), where


def test_prec_run_bands():
    """robot_legs_prec45_ws1_only / prec16_f075_js20: forward 0.05-0.24 from
    config.json, backward -0.4..-0.15 and |vy| 0.1-0.25 from the env's
    defaults (the config records neither)."""
    b, where = _bands(PREC)
    assert b == dict(v_fwd_min=0.05, v_fwd_max=0.24, v_back_min=0.15,
                     v_back_max=0.4, vy_min=0.1, vy_max=0.25, w_max=1.0,
                     crouch_min=1.0)
    assert where["cmd_v_range"] == "config"
    assert where["cmd_back_range"] == where["cmd_vy_range"] == "default"


def test_walk16_run_bands():
    b, _ = _bands(dict(PREC, cmd_v_range=[0.05, 0.7], walk_submix=[0.2, 0.2]))
    assert (b["v_fwd_min"], b["v_fwd_max"]) == (0.05, 0.7)
    assert (b["v_back_min"], b["v_back_max"]) == (0.15, 0.4)


def test_a_config_value_beats_the_default():
    b, where = _bands(dict(PREC, cmd_back_range=[-0.6, -0.2], cmd_vy_range=0.3))
    assert (b["v_back_min"], b["v_back_max"], b["vy_max"]) == (0.2, 0.6, 0.3)
    assert where["cmd_back_range"] == where["cmd_vy_range"] == "config"


def test_an_untrained_band_is_absent():
    b, _ = _bands(dict(PREC, walk_submix=[0.0, 0.0]))
    assert (b["v_back_min"], b["v_back_max"], b["vy_min"], b["vy_max"]) == (0, 0, 0, 0)
    b, _ = _bands(dict(PREC, ext_cmd=False))      # a legacy 2-channel run
    assert (b["v_back_max"], b["vy_max"], b["crouch_min"]) == (0, 0, 1.0)


def test_crouch_range():
    b, _ = _bands(dict(PREC, cmd_crouch_range=[0.8, 1.0]))
    assert b["crouch_min"] == 0.8
    # an ext_mix crouch slice draws crouch_range (env default 0.6-0.9)
    b, _ = _bands(dict(PREC, ext_mix=[0.2, 0.08, 0, 0, 0.15, 0, 0]))
    assert b["crouch_min"] == 0.6


def test_a_sway_trained_run_has_no_lateral_hole():
    b, _ = _bands(dict(PREC, ext_mix=[0.2, 0, 0, 0, 0.15, 0, 0.07]))
    assert b["vy_min"] == 0.0 and b["vy_max"] == 0.25


# -- the env defaults the generator resolves ---------------------------------
def test_env_defaults_match_the_mjx_env():
    """The training env (env_mjx) and the CPU env (walker_env) must agree on
    every default a config.json leaves out."""
    pytest.importorskip("jax")
    import inspect
    from mjx import env_mjx
    sig = inspect.signature(env_mjx.BimoMJXEnv.__init__).parameters
    want = gen_envelope.env_defaults()
    for k in gen_envelope.KEYS:
        if k in sig:
            assert tuple(sig[k].default) == tuple(want[k]) \
                if isinstance(want[k], (tuple, list)) \
                else sig[k].default == want[k], k


def test_the_sidestep_draw_floor_is_still_what_the_generator_says():
    """cmd_vy_range has no recorded lower bound: it is a literal in both envs."""
    mjx = open(os.path.join(ROOT, "sim", "mjx", "env_mjx.py")).read()
    cpu = open(os.path.join(ROOT, "sim", "walker_env.py")).read()
    lo = gen_envelope.CMD_VY_DRAW_MIN
    assert re.search(r"vy_s = side \* jax\.random\.uniform\(rs\[7\], "
                     r"minval=%s,\s+maxval=self\.cmd_vy_range\)" % lo, mjx)
    assert "rng.uniform(%s, self.cmd_vy_range)" % lo in cpu


def test_train_mjx_still_has_no_flag_for_the_unrecorded_bands():
    """If a flag appears the run starts recording the value and the generator
    must read it (it does: config wins); this fails to make someone look."""
    src = open(os.path.join(ROOT, "sim", "mjx", "train_mjx.py")).read()
    for k in ("cmd_back_range", "cmd_vy_range"):
        assert k not in src, k
