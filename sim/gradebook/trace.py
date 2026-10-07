"""Trace capture + loading (design §3 — the load-bearing interface).

One Trace per episode, written by the referee's capture hook
(`eval_precision --trace`) as `runs/<run>/traces/<scenario>_s{seed}.npz`.
Schema: every `Driver.rows` key verbatim (state series at control dt:
t/x/y/yaw/planar/wob2/watts/height/up_z/con_l/con_r/foot_err/foot_clear/
foot_dx/foot_dz/cmd_lift/com_stance/recovered) plus:

  cmd        — (N,7) the command tuple applied at each tick
  fell       — bool; env termination fired
  success    — bool; the scenario's own frozen verdict (so trace consumers
               read the verdict out of the trace instead of re-deriving it)
  meta       — JSON dict (see below)

Capture is ADDITIVE and read-only towards the rollout: the referee's rows,
evaluate() inputs, and scorecard numbers are untouched (acceptance: byte
parity of scorecard outputs with/without --trace).
"""
import json
import os

import numpy as np

SCHEMA_VERSION = 1

# the per-step series key set, in row-dict insertion order (stable since the
# Driver builds each row with one literal dict); mirrored here so loading
# does not depend on eval_precision import order.
ROW_KEYS = ("t", "x", "y", "yaw", "planar", "wob2", "watts", "height",
            "up_z", "con_l", "con_r", "foot_err", "foot_clear", "foot_dx",
            "foot_dz", "cmd_lift", "com_stance", "recovered")
BOOL_KEYS = ("con_l", "con_r")


def save(path, *, scenario, seed, dt, episode_seconds, rows, cmds,
         fell, success, events, swing_summary, shared, headline, metrics,
         movie=None, reel_chapter=None):
    """Write one episode trace. `events` is the scenario's ev-dict minus the
    internal `_setup`/`_env` hooks; non-JSON values degrade to str (the
    event dict is evidence, not a schema)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    meta = dict(schema=SCHEMA_VERSION, scenario=scenario, seed=int(seed),
                dt=float(dt), episode_seconds=float(episode_seconds),
                n_steps=len(rows), success=bool(success), fell=bool(fell),
                headline=headline or "", metrics=metrics or {},
                swing=swing_summary or {}, shared=shared or {},
                movie=movie, reel_chapter=reel_chapter)
    arrays = {}
    for k in ROW_KEYS:
        vals = [r[k] for r in rows]
        dt_ = np.bool_ if k in BOOL_KEYS else np.float64
        arrays[k] = np.asarray(vals, dtype=dt_)
    arrays["cmd"] = np.asarray(
        [list(c) + [0.0] * (7 - len(c)) for c in cmds], dtype=np.float64) \
        if cmds else np.zeros((0, 7))
    # sanitize events for json (numpy scalars won't serialize)
    ev = {}
    for k, v in (events or {}).items():
        if isinstance(v, (np.floating, np.integer)):
            v = v.item()
        try:
            json.dumps(v)
            ev[k] = v
        except TypeError:
            ev[k] = repr(v)
    arrays["meta"] = np.frombuffer(json.dumps(meta).encode(), np.uint8)
    arrays["events"] = np.frombuffer(json.dumps(ev).encode(), np.uint8)
    np.savez_compressed(path, **arrays)
    return path


def load(path):
    """npz -> dict of arrays + decoded meta/events JSON dicts."""
    z = np.load(path, allow_pickle=False)
    out = {k: z[k] for k in z.files if k not in ("meta", "events")}
    out["meta"] = json.loads(bytes(z["meta"].tolist()).decode())
    out["events"] = json.loads(bytes(z["events"].tolist()).decode())
    return out
