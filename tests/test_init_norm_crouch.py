"""--init-norm-crouch (2026-11-05, knee-bend walking): every walking policy so far trained
with the crouch command frozen at 1.0, so its observation normalizer holds std 1e-6 on the
crouch slots; a warm start with --cmd-crouch-range lo,hi must re-seat those slots or any
crouch below 1 reads as ~1e5 sigma. Pinned: the slots are exactly where env_mjx writes
cmd[3] in every stacked frame, and the re-seat sets mean/std/summed_variance there only."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "mjx"))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

ARGS = ["--robot", "--xml", "bimo_biped_v6legs_meas_sole.xml", "--servo-kp-scale", "asbuilt",
        "--precision", "--mjx-impl", "jax", "--family", "loco", "--action-map", "full",
        "--cmd-crouch-range", "0.85,1.0", "--init-norm-crouch"]


def _env():
    import train_mjx
    from env_mjx import BimoMJXEnv
    a = train_mjx.make_parser().parse_args(ARGS)
    assert a.init_norm_crouch
    return train_mjx, BimoMJXEnv(**train_mjx.build_env_kw(a))


def test_slots_hold_the_crouch_command():
    import jax
    import jax.numpy as jp
    train_mjx, env = _env()
    idx = train_mjx.crouch_obs_slots(env)
    assert len(idx) == env.obs_hist_len and env.obs_size == env.obs_frame * env.obs_hist_len
    seen = set()
    for k in range(6):
        st = env.reset(jax.random.PRNGKey(k))
        for _ in range(3):
            st = env.step(st, jp.zeros(env.action_size))
        c = float(np.asarray(st.cmd)[3])
        o = np.asarray(st.obs)
        assert 0.85 - 1e-6 <= c <= 1.0 + 1e-6
        # the newest frame carries this tick's command at one of the slots
        assert any(abs(float(o[i]) - c) < 1e-5 for i in idx), (o[idx], c)
        seen.add(round(c, 4))
    assert len(seen) > 1                       # the draw really varies the channel


def test_reseat_only_touches_the_crouch_slots():
    import jax.numpy as jp
    from brax.training.acme import running_statistics, specs
    train_mjx, env = _env()
    st = running_statistics.init_state(specs.Array((env.obs_size,), jp.float32))
    obs = np.zeros((4, env.obs_size), np.float32)
    idx = train_mjx.crouch_obs_slots(env)
    obs[:, idx] = 1.0                                     # frozen at 1.0, as trained
    obs[:, 0] = np.arange(4)                               # one ordinary channel
    st = running_statistics.update(st, jp.asarray(obs))
    restore = (st, "policy", "value")
    new, got = train_mjx.reseat_crouch_norm(restore, env, 0.85, 1.0)
    assert got == idx and new[1:] == ("policy", "value")
    m, s = np.asarray(new[0].mean), np.asarray(new[0].std)
    assert np.allclose(m[idx], 0.925) and np.allclose(s[idx], 0.15 / np.sqrt(12.0), rtol=1e-5)
    keep = [i for i in range(env.obs_size) if i not in idx]
    assert np.allclose(m[keep], np.asarray(st.mean)[keep]) and np.allclose(s[keep], np.asarray(st.std)[keep])
    # 0.85 now reads as ~-1.7 sigma, not ~-1e5
    z = (0.85 - m[idx[0]]) / s[idx[0]]
    assert -2.0 < z < -1.5
