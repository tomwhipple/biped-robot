"""tools/gen_obs_spec.py: the obs vectors must be satisfiable by the C assembler for any run."""
import importlib.util
import json
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _gen():
    pytest.importorskip("mujoco")
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    spec = importlib.util.spec_from_file_location(
        "gen_obs_spec", os.path.join(ROOT, "tools", "gen_obs_spec.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_frame_dq_is_the_scripted_input_for_a_dq_lag_run(tmp_path):
    """A --dq-obs-window run reads dq from a position history; the scripted states never fill it,
    so without switching that view off every frame reads dq = 0 and test_obs fails."""
    gen = _gen()
    (tmp_path / "config.json").write_text(json.dumps({
        "xml_path": "bimo_biped_v6legs_meas_sole.xml",
        "dq_obs_window": [2, 3],
        "dq_obs_quantum": 50.0,
        "quantize_ticks": True,
    }))
    _, env = gen.build_env(str(tmp_path))
    assert env.dq_obs_window is None and env.dq_obs_quantum == 0.0
    samples = gen.sample_frames(env, n=3)
    nq = env._nq_act
    for s in samples:
        np.testing.assert_allclose(s["frame"][nq:2 * nq], s["dq"], atol=1e-5)
    assert np.abs(samples[1]["dq"]).max() > 0.1   # the check above is not 0 == 0
