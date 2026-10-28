"""rise_ref_et_hold: early termination after the held stand (get-up campaign, 2026-10-28).

The DeepMimic early termination (rise_ref_et) ends a recovery episode whose
pelvis strays from the reference's, but only before the stand latch
(`recovered`). Under rise_ref_hold the reference holds its standing frame
after the latch and the tracking keeps paying, so a stand that sinks into a
~0.27 m crouch is never ended and still earns partial income. Under 15 N
shoves the w2k/w4k get-ups stand, then sink there (2026-10-28 push grade).
rise_ref_et_hold keeps the termination running after the latch.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "sim")
sys.path.insert(0, os.path.join(SIM, "mjx"))
sys.path.insert(0, SIM)
os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("MUJOCO_GL", "disabled")

REF = "getup_ref_v6ar_prone_ikc.npz"


def _env(et_hold):
    import robot_plant
    from env_mjx import BimoMJXEnv
    kw = robot_plant.robot_env_kwargs("bimo_biped_v6ar.xml")
    kw["held_joints"] = {k: v for k, v in (kw.get("held_joints") or {}).items()
                         if robot_plant._role(k) not in ("shoulder", "elbow")}
    kw.update(xml_path=os.path.join(SIM, "bimo_biped_v6ar.xml"), ext_cmd=True, recover_mix=1.0,
              domain_rand=False, rise_ref_npz=REF, rise_ref_hold=True,
              rise_ref_et=0.1, rise_ref_et_hold=et_hold)
    return BimoMJXEnv(**kw)


@pytest.mark.parametrize("et_hold", [False, True])
def test_et_after_the_stand(et_hold):
    """A recovered recovery episode whose pelvis is far from the reference's:
    ended with rise_ref_et_hold, left running without it."""
    import jax
    import jax.numpy as jp
    e = _env(et_hold)
    st = jax.jit(e.reset)(jax.random.PRNGKey(3))
    qpos = st.data.qpos.at[2].set(1.5)          # 1 m+ above any reference frame
    st = st._replace(data=st.data.replace(qpos=qpos),
                     recover_slot=jp.ones(()), recovered=jp.ones(()))
    nxt = jax.jit(e.step)(st, jp.zeros(e.action_size))
    assert float(nxt.done) == (1.0 if et_hold else 0.0)


def test_needs_hold_and_et():
    import robot_plant
    from env_mjx import BimoMJXEnv
    kw = robot_plant.robot_env_kwargs("bimo_biped_v6ar.xml")
    kw.update(xml_path=os.path.join(SIM, "bimo_biped_v6ar.xml"), rise_ref_npz=REF)
    with pytest.raises(ValueError, match="rise_ref_et_hold needs"):
        BimoMJXEnv(**kw, rise_ref_et=0.1, rise_ref_et_hold=True)
    with pytest.raises(ValueError, match="rise_ref_et_hold needs"):
        BimoMJXEnv(**kw, rise_ref_hold=True, rise_ref_et_hold=True)
