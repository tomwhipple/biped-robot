"""Linear-static FEM of the glued hip yoke (pieces fused = perfect glue) under
the hip-roll servo's stall torque, CalculiX via gmsh. Yoke-local frame: roll
axis X through the origin, pitch axis Y through (0, 0, -50).

Load: torque T about +X into the roll HORN arm, as tangential forces on the
four O3.4 bolt-hole walls (BCD 14) of the horn pad, T / (4 * 7) each.
Restraint: the eight O3.4 bolt-hole walls of both pitch pads, all DOF.
Material: PETG, isotropic E 2000 MPa, nu 0.38 (stresses here are what the
part would see if it printed as solid isotropic PETG).

    python fem_yoke.py <tag> [T_Nmm] [clmax]
reads fem/<tag>_union.step, writes fem/<tag>/{mesh.inp, job.inp, job.frd} and
prints the stress summary.
"""
import os
import re
import subprocess
import sys

import numpy as np

CASE = os.environ.get("FEM_CASE", "roll")
BIN = os.environ.get("FREECAD_BIN", "/Applications/FreeCAD.app/Contents/Resources/bin")
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       "build", "experiments", "hip-yoke-strength")
HERE = os.path.join(OUT_DIR, "fem")
R_HOLE, BCD_R = 1.7, 7.0
PZ = -50.0


def mesh(step, out, clmax):
    """Second-order tets, straight-sided (no inverted curved elements). Falls
    back to OCC small-face healing and other 2D algorithms if a sliver face
    defeats the default mesher."""
    base = [f"{BIN}/gmsh", step, "-3", "-order", "2", "-clmax", str(clmax), "-clmin", str(clmax / 4),
            "-clcurv", "12", "-optimize_netgen", "-setnumber", "Mesh.SecondOrderLinear", "1",
            "-nt", str(os.cpu_count()), "-format", "inp", "-o", out]
    heal = ["-setnumber", "Geometry.OCCFixSmallEdges", "1", "-setnumber", "Geometry.OCCFixSmallFaces", "1",
            "-setnumber", "Geometry.OCCSewFaces", "1"]
    for extra in ([], heal, heal + ["-setnumber", "Mesh.Algorithm", "1"],
                  heal + ["-setnumber", "Mesh.Algorithm", "5"], ["-setnumber", "Mesh.Algorithm", "1"]):
        r = subprocess.run(base + extra, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0 and os.path.exists(out):
            print("gmsh ok with", extra or "defaults")
            return
    raise RuntimeError("gmsh failed with every fallback")


def read_inp(path):
    nodes, elems, mode, etype = {}, [], None, None
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s.startswith("*"):
                u = s.upper()
                if u.startswith("*NODE"):
                    mode = "n"
                elif u.startswith("*ELEMENT"):
                    m = re.search(r"TYPE=(\w+)", u)
                    etype = m.group(1) if m else None
                    mode = "e"
                else:
                    mode = None
                continue
            if not s:
                continue
            if mode == "n":
                p = s.split(",")
                nodes[int(p[0])] = [float(v) for v in p[1:4]]
            elif mode == "e" and etype == "C3D10":
                p = [int(v) for v in s.rstrip(",").split(",") if v.strip()]
                elems.append(p[1:])
    used = sorted({n for e in elems for n in e})
    return {n: nodes[n] for n in used}, elems


def hole_nodes(ids, X, axis, centers, span, tol=0.06):
    """Nodes on the wall of a bolt hole: distance to the hole axis ~ R_HOLE,
    and inside `span` along the axis. centers: list of (a, b) in the plane
    normal to the axis. Returns {center: [node ids]}."""
    ax = {"x": 0, "y": 1, "z": 2}[axis]
    oth = [i for i in range(3) if i != ax]
    out = {}
    for c in centers:
        d = np.hypot(X[:, oth[0]] - c[0], X[:, oth[1]] - c[1])
        m = (np.abs(d - R_HOLE) < tol) & (X[:, ax] >= span[0]) & (X[:, ax] <= span[1])
        out[c] = ids[m]
    return out


def run(tag, T=4900.0, clmax=1.0):
    wd = os.path.join(HERE, tag if CASE == "roll" else f"{tag}_{CASE}")
    os.makedirs(wd, exist_ok=True)
    mpath = os.path.join(wd, "mesh.inp")
    mesh(os.path.join(HERE, f"{tag}_union.step"), mpath, clmax)
    nodes, elems = read_inp(mpath)
    ids = np.array(list(nodes.keys()))
    X = np.array(list(nodes.values()))
    rc = [(BCD_R, 0.0), (-BCD_R, 0.0), (0.0, BCD_R), (0.0, -BCD_R)]
    pc = [(BCD_R, PZ), (-BCD_R, PZ), (0.0, PZ + BCD_R), (0.0, PZ - BCD_R)]
    horn_r = hole_nodes(ids, X, "x", rc, (20.0, 25.0))
    idler_r = hole_nodes(ids, X, "x", rc, (-25.0, -16.0))
    horn_p = hole_nodes(ids, X, "y", pc, (20.0, 24.0))
    idler_p = hole_nodes(ids, X, "y", pc, (-21.0, -17.0))
    if CASE == "roll":
        load, fix_h, fix_i = horn_r, horn_p, idler_p
    else:
        load, fix_h, fix_i = (horn_p if CASE == "pitch" else {**horn_p, **{(k[0], k[1], "i"): v for k, v in idler_p.items()}}), horn_r, idler_r
    nl = {c: len(v) for c, v in load.items()}
    nf = sum(len(v) for v in fix_h.values()) + sum(len(v) for v in fix_i.values())
    assert all(n > 6 for n in nl.values()) and nf > 40, (nl, nf)
    lines = ["*HEADING", f"glued hip yoke {tag}: case {CASE}, {T}", "*NODE"]
    lines += [f"{i},{x:.6f},{y:.6f},{z:.6f}" for i, (x, y, z) in nodes.items()]
    lines.append("*ELEMENT, TYPE=C3D10, ELSET=EALL")
    lines += [f"{k + 1}," + ",".join(map(str, e)) for k, e in enumerate(elems)]
    lines += ["*MATERIAL, NAME=PETG", "*ELASTIC", "2000.0, 0.38",
              "*SOLID SECTION, ELSET=EALL, MATERIAL=PETG", "*NSET, NSET=NFIX"]
    fixed = np.concatenate([*fix_h.values(), *fix_i.values()])
    lines += [",".join(map(str, fixed[i:i + 12])) for i in range(0, len(fixed), 12)]
    lines += ["*STEP", "*STATIC", "*BOUNDARY", "NFIX,1,3,0.0", "*CLOAD"]
    y_h, y_i = 21.95, -18.9            # mid-planes of the pitch horn and idler pads
    for key, nn in load.items():
        ca, cb = key[0], key[1]
        idler = len(key) == 3
        if CASE == "roll":         # torque T about +X: X x r = (0, -z, y) / r, 4 holes
            vec = {2: -cb / BCD_R * T / (4 * BCD_R), 3: ca / BCD_R * T / (4 * BCD_R)}
        elif CASE == "pitch":      # torque T about +Y through the pitch axis, 4 holes
            vec = {1: (cb - PZ) / BCD_R * T / (4 * BCD_R), 3: -ca / BCD_R * T / (4 * BCD_R)}
        elif CASE == "yaw":        # torque T about +Z: +x on the horn pad, -x on the idler pad
            vec = {1: (-1 if idler else 1) * T / (y_h - y_i) / 4}
        elif CASE == "fz":         # force T (N) up through the pitch servo, 8 holes
            vec = {3: T / 8}
        elif CASE == "fx":         # force T (N) forward through the pitch servo, 8 holes
            vec = {1: T / 8}
        f = 1.0 / len(nn)
        for n in nn:
            for dof, v in vec.items():
                if abs(v) > 1e-12:
                    lines.append(f"{n},{dof},{f * v:.6f}")
    lines += ["*NODE FILE", "U", "*EL FILE", "S", "*END STEP"]
    with open(os.path.join(wd, "job.inp"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    env = dict(os.environ, OMP_NUM_THREADS=str(os.cpu_count()), CCX_NPROC_EQUATION_SOLVER=str(os.cpu_count()))
    subprocess.run([f"{BIN}/ccx", "-i", "job"], cwd=wd, check=True, env=env, stdout=subprocess.DEVNULL)
    return summarize(tag, wd, ids, X, len(elems))


def read_frd_stress(path):
    S, U, block = {}, {}, None
    with open(path) as f:
        for line in f:
            if line.startswith("    1PSTRESS") or line.startswith(" -4  STRESS"):
                block = "S"
                continue
            if line.startswith(" -4  DISP"):
                block = "U"
                continue
            if line.startswith(" -3"):
                block = None
                continue
            if block and line.startswith(" -1"):
                n = int(line[3:13])
                vals = [float(line[13 + 12 * k:25 + 12 * k]) for k in range((len(line.rstrip()) - 13) // 12)]
                (S if block == "S" else U)[n] = vals
    return S, U


def vm(s):
    sxx, syy, szz, sxy, syz, szx = s[:6]
    return np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))


def summarize(tag, wd, ids, X, ne):
    S, U = read_frd_stress(os.path.join(wd, "job.frd"))
    sid = np.array(sorted(S))
    idx = {n: i for i, n in enumerate(ids)}
    P = X[[idx[n] for n in sid]]
    St = np.array([S[n][:6] for n in sid])
    V = vm(St.T)
    # away from the bolt holes (load / restraint artefacts): > 3.5 mm from every hole axis
    far = np.ones(len(sid), bool)
    for c in [(BCD_R, 0.0), (-BCD_R, 0.0), (0.0, BCD_R), (0.0, -BCD_R)]:
        far &= ~((np.hypot(P[:, 1] - c[0], P[:, 2] - c[1]) < 3.5) & (P[:, 0] > 15))
    for c in [(BCD_R, PZ), (-BCD_R, PZ), (0.0, PZ + BCD_R), (0.0, PZ - BCD_R)]:
        far &= ~(np.hypot(P[:, 0] - c[0], P[:, 2] - c[1]) < 3.5)
    umax = max(np.linalg.norm(u[:3]) for u in U.values())
    out = {"tag": f"{tag} {CASE}", "nodes": len(ids), "elems": ne, "umax_mm": umax,
           "vm_max_far": float(V[far].max())}
    i = np.argmax(np.where(far, V, -1))
    out["vm_max_far_at"] = tuple(np.round(P[i], 2))
    # the failure zone: the roll flange's front overhang + horn arm root
    zone = far & (P[:, 0] > 12) & (P[:, 0] < 24.5) & (P[:, 2] > -24.5) & (P[:, 2] < -8)
    out["vm_max_front_zone"] = float(V[zone].max())
    j = np.argmax(np.where(zone, V, -1))
    out["vm_front_at"] = tuple(np.round(P[j], 2))
    out["vm_p99_front_zone"] = float(np.percentile(V[zone], 99))
    # the glue plane z -20 (the lap): normal stress (peel = tension) and shear
    lap = (np.abs(P[:, 2] + 20.0) < 0.05) & far
    if lap.any():
        out["lap_peel_max"] = float(St[lap, 2].max())
        out["lap_shear_max"] = float(np.hypot(St[lap, 4], St[lap, 5]).max())
    with open(os.path.join(wd, "summary.txt"), "w") as fh:
        for k, v in out.items():
            fh.write(f"{k}: {v}\n")
    np.save(os.path.join(wd, "vm_nodes.npy"), np.column_stack([P, V]))
    np.save(os.path.join(wd, "stress.npy"), np.column_stack([P, St, far]))
    return out


if __name__ == "__main__":
    tag = sys.argv[1]
    T = float(sys.argv[2]) if len(sys.argv) > 2 else {"roll": 4900.0, "pitch": 2940.0, "yaw": 2940.0,
                                                       "fz": 100.0, "fx": 50.0}[CASE]
    cl = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    for k, v in run(tag, T, cl).items():
        print(f"{k:22s} {v}")
