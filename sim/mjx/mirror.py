"""Left/right mirror maps for observations and actions.

The gait is asymmetric (22-35% air-time mismatch across every run) and the
swing-mismatch penalty provably does not fix it: w_symmetry 3.0 for 110M+
steps moved the number ~0 (v17sym, and every run since). This module is the
structural alternative: an exact signed permutation that maps a state/action
to its left/right mirror image, so training can penalize the POLICY for
treating the two sides differently (symmetry regularization, cf.
arxiv.org/abs/2403.17320) instead of paying per-touchdown.

Everything is DERIVED from the loaded plant, not hardcoded:

  * joint permutation: name-matched L_*/R_* pairs, in qpos order;
  * joint signs: how a hinge angle transforms under reflection across the
    sagittal (xz) plane -- axis u maps to Mu (M = diag(1,-1,1)) and the
    angle negates, so axes along y keep their sign (pitch/knee/ankle) and
    axes along x or z flip (roll/yaw). Read off jnt_axis per joint.
  * a consistency check at build time: mirrored joints must have mirrored
    ranges (the hip-roll -25..55 vs -55..25 asymmetry is what catches a
    wrong sign here).

The NORMALIZED action mirror equals the joint-angle mirror only because
mirrored actuators have mirrored ctrlranges (center_R = -center_L,
half_R = half_L for flipped joints; identical ranges for unflipped). That
property is asserted, not assumed.

Obs frame layout (env_mjx._obs, one frame):
  q(nj) dq(nj) up(3) linvel(3) gyro(3) prev_action(nj) height(1)
  phase-sin/cos(2) cmd(ncmd)
Under reflection: up_y flips; linvel vy flips; gyro wx,wz flip (angular
velocity is a pseudovector); height keeps; the gait clock convention is
"left swings on sin>0", so the mirror is a half-cycle shift: sin,cos both
negate; cmd (vx,vy,wz,crouch,lift,fx,fz) -> vy,wz,lift negate. History
frames (obs_hist_len > 1) repeat the frame map block-diagonally.
"""
import numpy as np


def joint_perm_signs(model):
    """(perm, sign) over hinge joints in qpos order: q_mirror = sign * q[perm].

    perm swaps each L_<name> with R_<name>; sign is -1 where the hinge axis
    has any x or z component (roll/yaw-like), +1 for pure-y axes.
    """
    import mujoco
    names, axes, ranges = [], [], []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j))
        axes.append(model.jnt_axis[j].copy())
        ranges.append(model.jnt_range[j].copy())
    n = len(names)
    perm = np.zeros(n, dtype=np.int32)
    sign = np.zeros(n)
    for i, nm in enumerate(names):
        if nm.startswith("L_"):
            other = "R_" + nm[2:]
        elif nm.startswith("R_"):
            other = "L_" + nm[2:]
        else:
            raise ValueError(f"joint {nm} has no L_/R_ prefix; mirror "
                             "map undefined")
        k = names.index(other)
        perm[i] = k
        ax = axes[i]
        if not np.allclose(ax, axes[k]):
            raise ValueError(f"{nm}/{other} axes differ; sign rule assumes "
                             "identical axes on both sides")
        # reflection M=diag(1,-1,1): R(u, th) -> R(Mu, -th). With the SAME
        # axis u on both sides, a pure-y axis gives R(-u,-th)=R(u,th) (no
        # flip); any x/z component gives a flip.
        sign[i] = 1.0 if (abs(ax[0]) < 1e-9 and abs(ax[2]) < 1e-9) else -1.0
        # consistency: mirrored joints must have mirrored ranges
        lo_i, hi_i = ranges[i]
        lo_k, hi_k = ranges[k]
        exp = (lo_i, hi_i) if sign[i] > 0 else (-hi_i, -lo_i)
        if not np.allclose((lo_k, hi_k), exp, atol=1e-6):
            raise ValueError(
                f"{nm} sign {sign[i]:+.0f} inconsistent with ranges "
                f"{np.degrees((lo_i, hi_i)).round(1)} vs "
                f"{np.degrees((lo_k, hi_k)).round(1)}")
    return perm, sign


def action_perm_signs(model):
    """Normalized-action mirror. Equals the joint mirror iff mirrored
    actuators have mirrored ctrlranges -- asserted here."""
    perm, sign = joint_perm_signs(model)
    cr = model.actuator_ctrlrange
    for i in range(len(perm)):
        k = perm[i]
        exp = cr[i] if sign[i] > 0 else (-cr[i][1], -cr[i][0])
        if not np.allclose(cr[k], exp, atol=1e-6):
            raise ValueError(f"actuator {i}/{k}: ctrlranges not mirrored; "
                             "normalized-action mirror is NOT a signed "
                             "permutation for this plant")
    return perm, sign


def frame_perm_signs(model, ncmd):
    """(perm, sign) over ONE obs frame (layout in module docstring)."""
    jp_, js = joint_perm_signs(model)
    nj = len(jp_)
    perm, sign = [], []

    def block(p, s, off):
        perm.extend((np.asarray(p) + off).tolist())
        sign.extend(np.asarray(s).tolist())

    off = 0
    block(jp_, js, off); off += nj                       # q
    block(jp_, js, off); off += nj                       # dq
    block([0, 1, 2], [1, -1, 1], off); off += 3          # up: y flips
    block([0, 1, 2], [1, -1, 1], off); off += 3          # linvel: vy flips
    block([0, 1, 2], [-1, 1, -1], off); off += 3         # gyro: wx, wz flip
    block(jp_, js, off); off += nj                       # prev_action
    block([0], [1], off); off += 1                       # height
    block([0, 1], [-1, -1], off); off += 2               # phase: +pi shift
    if ncmd == 7:                                        # vx,vy,wz,crouch,
        block([0, 1, 2, 3, 4, 5, 6],                     # lift,fx,fz
              [1, -1, -1, 1, -1, 1, 1], off); off += 7
    elif ncmd == 2:                                      # vx, wz
        block([0, 1], [1, -1], off); off += 2
    else:
        raise ValueError(f"unknown cmd width {ncmd}")
    return np.asarray(perm, dtype=np.int32), np.asarray(sign)


def obs_perm_signs(model, ncmd, hist_len=1):
    """Full stacked-obs mirror: the frame map repeated per history slot."""
    fp, fs = frame_perm_signs(model, ncmd)
    d = len(fp)
    perm = np.concatenate([fp + h * d for h in range(hist_len)])
    sign = np.concatenate([fs] * hist_len)
    return perm.astype(np.int32), sign


def mirror(x, perm, sign):
    """Apply a signed permutation along the last axis (numpy or jax)."""
    return x[..., perm] * sign


def base_qpos_mirror(qpos):
    """Free-joint root mirror for tests: y, quat-x, quat-z negate."""
    out = np.array(qpos, dtype=float)
    out[1] = -out[1]
    out[4] = -out[4]          # qx
    out[6] = -out[6]          # qz
    return out


def base_qvel_mirror(qvel):
    out = np.array(qvel, dtype=float)
    out[1] = -out[1]          # vy
    out[3] = -out[3]          # wx
    out[5] = -out[5]          # wz
    return out
