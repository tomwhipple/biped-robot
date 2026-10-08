#!/usr/bin/env python3
"""Slice the v6 parts into OrcaSlicer projects for the FlashForge Adventurer 5M Pro.

For each part, cad/gcode/ (gitignored) gets:

    <part>.3mf    an OrcaSlicer project: the part in its print orientation, the
                  print spec's settings, the plate already sliced. Open it in
                  the GUI, check it, print it from there.
    <part>.gcode  the same slice, for the numbers (time, grams, footprint).

The settings are cad/v6/print_settings.json: the print spec on top of
Flashforge's own presets, one entry per part (orientation, supports,
copies, overrides), and plates: parts printed together, sliced into one
<plate>.3mf + .gcode. If a project is changed in the GUI and saved (Cmd+S),
the next run reads the change back -- settings, orientation, copies -- into
print_settings.json before it slices that part or plate again, so it carries
forward.

    python3 cad/slice.py leg_link_v6
    python3 cad/slice.py leg_link_v6 --copies 4
    python3 cad/slice.py feet            # a plate: both feet, one project
    python3 cad/slice.py --all           # every plate, and every part on no plate
    python3 cad/slice.py --harvest       # only read saved projects back

On the Mac it drives /Applications/OrcaSlicer.app; elsewhere the Flathub
flatpak under xvfb (docs/slicing.md). Standard library only.
"""

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zipfile

import orca_profile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(REPO, "cad", "v6", "stl")
SETTINGS = os.path.join(REPO, "cad", "v6", "print_settings.json")
OUT_DIR = os.path.join(REPO, "cad", "gcode")

FLATPAK_ID = "com.orcaslicer.OrcaSlicer"
# The flatpak sandbox has no network, no devices, and of the repo only
# cad/gcode (docs/slicing.md); everything the slicer reads is staged there.
SANDBOX_ENV = {
    "XDG_RUNTIME_DIR": f"/run/user/{os.getuid()}",
    "DBUS_SESSION_BUS_ADDRESS": f"unix:path=/run/user/{os.getuid()}/bus",
}

# Print orientations: rows are the print axes in model coordinates, so
# print = R @ model. Same constants as the printability audits.
ROTATIONS = {
    "IDENT": ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
    "RX180": ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
    "RY_XUP": ((0, 0, -1), (0, 1, 0), (1, 0, 0)),         # model +X -> print +Z
    "RY_XDOWN": ((0, 0, 1), (0, 1, 0), (-1, 0, 0)),
    "RY_ROLL_WALL": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),   # model +Y -> print +Z
}

# Keys that name presets or describe the file, not the print.
META_KEYS = {
    "inherits", "inherits_group", "different_settings_to_system", "name", "from",
    "version", "setting_id", "instantiation", "type", "renamed_from", "filament_id",
    "print_settings_id", "filament_settings_id", "printer_settings_id",
    "compatible_printers", "compatible_printers_condition", "compatible_prints",
    "compatible_prints_condition", "is_custom_defined", "description",
}
SECRET_KEYS = {"printhost_apikey", "printhost_password", "printhost_user"}
# Project-level keys a part may override (they live in no preset).
PROJECT_KEYS = {"curr_bed_type"}
# Per-object metadata in model_settings.config that is bookkeeping, not settings.
OBJECT_META = {"name", "extruder", "matrix", "source_file", "source_object_id",
               "source_volume_id", "source_offset_x", "source_offset_y",
               "source_offset_z"}
PAINT_ATTRS = (b"paint_supports", b"paint_seam", b"paint_color",
               b"mmu_segmentation", b"fuzzy_skin")


# --------------------------------------------------------------------------
# small 3x3 algebra

def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def snap(a, eps=1e-6):
    def s(x):
        r = round(x)
        return float(r) if abs(x - r) < eps else x
    return tuple(tuple(s(x) for x in row) for row in a)


def orient_matrix(orient):
    if isinstance(orient, str):
        try:
            return ROTATIONS[orient]
        except KeyError:
            raise SystemExit(f"unknown orient {orient!r}; use one of {sorted(ROTATIONS)} "
                             "or a 3x3 matrix") from None
    m = tuple(tuple(float(x) for x in row) for row in orient)
    should_be_eye = matmul(m, transpose(m))
    if len(m) != 3 or any(abs(should_be_eye[i][j] - (i == j)) > 1e-6
                          for i in range(3) for j in range(3)):
        raise SystemExit(f"orient {orient} is not a rotation")
    return m


def orient_name(m):
    for name, r in ROTATIONS.items():
        if all(abs(m[i][j] - r[i][j]) < 1e-6 for i in range(3) for j in range(3)):
            return name
    return [list(row) for row in m]


# --------------------------------------------------------------------------
# STL in, rotated STL out

def read_stl(path):
    """Triangles as (v0, v1, v2) tuples. Binary or ASCII."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            tris = []
            for i in range(n):
                v = struct.unpack_from("<9f", data, 84 + 50 * i + 12)
                tris.append((v[0:3], v[3:6], v[6:9]))
            return tris
    verts = [tuple(float(x) for x in m.groups())
             for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)]
    if not verts or len(verts) % 3:
        raise SystemExit(f"cannot read STL {path}")
    return [tuple(verts[i:i + 3]) for i in range(0, len(verts), 3)]


def write_rotated_stl(tris, rot, path):
    """Write ``tris`` rotated by ``rot``, lifted so the lowest point is z=0.

    Returns the rotated bounding-box size (x, y, z).
    """
    rt = [tuple(tuple(sum(r[k] * v[k] for k in range(3)) for r in rot) for v in t)
          for t in tris]
    lo = [min(v[i] for t in rt for v in t) for i in range(3)]
    hi = [max(v[i] for t in rt for v in t) for i in range(3)]
    out = bytearray(b"cad/slice.py rotated".ljust(80, b" "))
    out += struct.pack("<I", len(rt))
    for t in rt:
        a, b, c = [tuple(v[i] - lo[i] for i in range(3)) for v in t]
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        nrm = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        ln = math.sqrt(sum(x * x for x in nrm)) or 1.0
        out += struct.pack("<12fH", *(x / ln for x in nrm), *a, *b, *c, 0)
    with open(path, "wb") as fh:
        fh.write(out)
    return tuple(hi[i] - lo[i] for i in range(3))


# --------------------------------------------------------------------------
# settings

def load_settings(path=SETTINGS):
    with open(path) as fh:
        return json.load(fh)


def save_settings(spec, path=SETTINGS):
    with open(path, "w") as fh:
        json.dump(spec, fh, indent=2, ensure_ascii=False)
        fh.write("\n")


def part_entry(spec, part):
    try:
        return spec["parts"][part]
    except KeyError:
        raise SystemExit(f"{part}: not in {os.path.relpath(SETTINGS, REPO)} 'parts'") from None


def plate_parts(spec, name):
    """The parts a job puts on the bed: a plate's parts, or the part alone."""
    plate = spec.get("plates", {}).get(name)
    if plate is None:
        part_entry(spec, name)
        return [name]
    for part in plate["parts"]:
        part_entry(spec, part)
    return list(plate["parts"])


def job_entry(spec, name):
    """The entry whose supports, settings and copies slice job ``name``.

    One process slices the whole plate, so a plate's parts must agree on
    supports and settings; its copies are the plate's own.
    """
    plate = spec.get("plates", {}).get(name)
    if plate is None:
        return part_entry(spec, name)
    entries = [part_entry(spec, p) for p in plate["parts"]]
    for key in ("supports", "settings"):
        if len({json.dumps(e.get(key), sort_keys=True) for e in entries}) > 1:
            raise SystemExit(f"plate {name}: its parts disagree on {key!r}, and one "
                             "process slices the whole plate")
    return {"supports": entries[0].get("supports", False),
            "settings": dict(entries[0].get("settings", {})),
            "copies": plate.get("copies", 1)}


def jobs(spec):
    """Every project --all makes: each plate, and each part on no plate."""
    plates = spec.get("plates", {})
    on_plates = {p for plate in plates.values() for p in plate["parts"]}
    return sorted(plates) + sorted(p for p in spec["parts"] if p not in on_plates)


def choose_printer(lib, spec, wanted=None):
    """The GUI's own printer preset for this printer (it holds the print host),
    else the system preset. A project naming the GUI's preset prints from the
    GUI without picking a printer."""
    if wanted:
        lib.get("machine", wanted)
        return wanted
    mine = lib.user_printers(spec["printer"])
    if len(mine) > 1:
        raise SystemExit(f"several GUI printer presets inherit {spec['printer']!r}: "
                         f"{mine}; pick one with --printer")
    return mine[0] if mine else spec["printer"]


def build_presets(lib, spec, entry, printer, with_part=True):
    """(machine, process, filament) dicts, flattened and layered."""
    machine = lib.resolve("machine", printer)
    for k in SECRET_KEYS:
        machine.pop(k, None)
    if printer != spec["printer"]:
        # The CLI checks process/filament compatibility through the printer's
        # system parent; with inherits dropped it rejects the process.
        machine["inherits"] = spec["printer"]
    process = lib.resolve("process", spec["process"]["preset"])
    filament = lib.resolve("filament", spec["filament"]["preset"])
    process.update(spec["process"].get("settings", {}))
    filament.update(spec["filament"].get("settings", {}))
    if entry.get("supports"):
        process.update(spec["supports"])
    else:
        process["enable_support"] = "0"
    if with_part:
        for key, value in entry.get("settings", {}).items():
            route(key, process, filament)[key] = value
    return machine, process, filament


def route(key, process, filament):
    if key in process or key in PROJECT_KEYS:
        return process
    if key in filament:
        return filament
    raise SystemExit(f"setting {key!r} is not a process or filament setting "
                     "(printer settings belong in the GUI's printer preset)")


def different_keys(cfg, system, also=()):
    """Keys whose project value differs from the system preset's.

    The GUI keeps a project's value only for keys listed in
    different_settings_to_system and resets every other key to the system
    preset, so over-listing is harmless and under-listing silently loses a
    setting.
    """
    keys = {k for k in system if k not in META_KEYS and k in cfg and cfg[k] != system[k]}
    keys |= {k for k in also if k in cfg and k not in META_KEYS}
    return ";".join(sorted(keys))


# --------------------------------------------------------------------------
# 3mf

NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02",
      "p": "http://schemas.microsoft.com/3dmanufacturing/production/2015/06"}


def _transform(s):
    """3MF transform string -> (3x3 row-vector rotation part, translation)."""
    v = [float(x) for x in s.split()] if s else [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    return (tuple(v[0:3]), tuple(v[3:6]), tuple(v[6:9])), tuple(v[9:12])


def read_3mf(path):
    """What harvest needs from a project: config, per-object overrides,
    instance rotations, and anything harvest cannot carry."""
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        cfg = json.loads(z.read("Metadata/project_settings.config"))
        model = ET.fromstring(z.read("3D/3dmodel.model"))
        settings_xml = (ET.fromstring(z.read("Metadata/model_settings.config"))
                        if "Metadata/model_settings.config" in names else None)
        painted = any(attr in z.read(n) for n in names
                      if n.startswith("3D/Objects/") for attr in PAINT_ATTRS)
    objects = {}
    for obj in model.iterfind("m:resources/m:object", NS):
        comp = obj.find("m:components/m:component", NS)
        objects[obj.get("id")] = _transform(comp.get("transform") if comp is not None else None)[0]
    overrides, object_names, id_names, extras = {}, set(), {}, []
    if settings_xml is not None:
        for obj in settings_xml.iterfind("object"):
            for md in obj.iterfind("metadata"):
                k, v = md.get("key"), md.get("value")
                if k == "name":
                    object_names.add(v)
                    id_names[obj.get("id")] = v
                elif k not in OBJECT_META:
                    overrides[k] = v
            for part in obj.iterfind("part"):
                if part.get("subtype", "normal_part") != "normal_part":
                    extras.append(f"a {part.get('subtype')} volume")
                for md in part.iterfind("metadata"):
                    k, v = md.get("key"), md.get("value")
                    if k not in OBJECT_META:
                        overrides[k] = v
        if settings_xml.find("object/layer_config_ranges") is not None:
            extras.append("height-range modifiers")
    if painted:
        extras.append("painted supports/seams/colour")
    items = []
    for item in model.iterfind("m:build/m:item", NS):
        item_rot = _transform(item.get("transform"))[0]
        comp_rot = objects.get(item.get("objectid"), ROTATIONS["IDENT"])
        # Row-vector convention: v' = v . C . I, so as a column rotation R = (C I)^T.
        items.append((id_names.get(item.get("objectid")),
                      transpose(matmul(comp_rot, item_rot))))
    return {"config": cfg, "items": items, "rotations": [r for _, r in items],
            "overrides": overrides, "object_names": object_names, "extras": extras}


def finalize_3mf(path, cfg_patch):
    """Rewrite the project's config with ``cfg_patch`` merged in. Returns the config."""
    tmp = path + ".tmp"
    with zipfile.ZipFile(path) as zin, zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
        for info in zin.infolist():
            data = zin.read(info.filename)
            if info.filename == "Metadata/project_settings.config":
                cfg = json.loads(data)
                cfg.update(cfg_patch)
                data = json.dumps(cfg, indent=4).encode()
            zout.writestr(info, data)
    os.replace(tmp, path)
    return cfg


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------------------
# harvest: a project saved from the GUI -> print_settings.json

def state_objects(state, name):
    """{object name: {part, orient, rotation}} from a job's state file. A
    state written before plates existed holds one part's orient + rotation."""
    if "objects" in state:
        return state["objects"]
    return {f"{name}.stl": {"part": name, "orient": state["orient"],
                            "rotation": state["rotation"]}}


def harvest(name, spec, lib, outdir, printer):
    """Fold a GUI-saved project's changes into ``spec``. Returns (changes, problem).

    ``name`` is a part or a plate. A plate's settings are shared, so a
    changed setting goes to every part on it; a part tipped onto another
    face changes that part's orientation only.
    """
    path = os.path.join(outdir, f"{name}.3mf")
    state_path = os.path.join(outdir, ".work", f"{name}.state.json")
    if not (os.path.exists(path) and os.path.exists(state_path)):
        return [], None
    with open(state_path) as fh:
        state = json.load(fh)
    if sha256(path) == state["sha256"]:
        return [], None

    backup_dir = os.path.join(outdir, ".work", "saved")
    os.makedirs(backup_dir, exist_ok=True)
    backup = os.path.join(backup_dir, f"{name}-{time.strftime('%Y%m%d-%H%M%S')}.3mf")
    shutil.copy2(path, backup)
    rel_backup = os.path.relpath(backup, REPO)
    try:
        saved = read_3mf(path)
    except (KeyError, ValueError, zipfile.BadZipFile, ET.ParseError) as exc:
        return [], (f"{name}.3mf changed but cannot be read ({type(exc).__name__}: {exc}); "
                    f"not re-slicing it (copy kept at {rel_backup})")

    objects = state_objects(state, name)
    foreign = saved["object_names"] - set(objects)
    if saved["extras"] or foreign:
        what = saved["extras"] + [f"another object ({', '.join(sorted(foreign))})"] * bool(foreign)
        return [], (f"{name}.3mf was saved with {', '.join(what)}, which slice.py cannot "
                    f"carry over; not re-slicing it (copy kept at {rel_backup}). Encode "
                    "it in print_settings.json, or delete the project to start over.")

    entry = job_entry(spec, name)
    changes = []
    _, base_process, base_filament = build_presets(lib, spec, entry, printer, with_part=False)
    base = {**base_process, **base_filament}
    settings = dict(entry.get("settings", {}))
    found = {k: saved["config"][k] for k, v in state["config"].items()
             if k in saved["config"] and saved["config"][k] != v}
    found.update(saved["overrides"])
    for key, value in sorted(found.items()):
        if key in SECRET_KEYS or key in META_KEYS:
            continue
        if key not in base and key not in PROJECT_KEYS:
            changes.append(f"ignored {key} = {value!r}: not a process or filament setting")
            continue
        old = settings.get(key, base.get(key))
        if base.get(key) == value:
            settings.pop(key, None)
        else:
            settings[key] = value
        changes.append(f"{key}: {old!r} -> {value!r}")
    if settings != entry.get("settings", {}):
        for part in plate_parts(spec, name):
            if settings:
                spec["parts"][part]["settings"] = dict(settings)
            else:
                spec["parts"][part].pop("settings", None)

    counts = {}
    for obj_name, r_saved in saved["items"]:
        if obj_name is None and len(objects) == 1:
            obj_name = next(iter(objects))
        if obj_name not in objects:
            continue
        counts[obj_name] = counts.get(obj_name, 0) + 1
        if counts[obj_name] > 1:
            continue
        # The project's mesh is the part already in its print orientation, so
        # the saved instance rotation acts on top of that. Its bottom row says
        # which way is up; a change there means the part was tipped onto
        # another face, not just spun on the bed. The first copy decides.
        gen = objects[obj_name]
        if any(abs(r_saved[2][i] - gen["rotation"][2][i]) > 1e-4 for i in range(3)):
            new = orient_name(snap(matmul(r_saved, orient_matrix(gen["orient"]))))
            spec["parts"][gen["part"]]["orient"] = new
            label = "orient" if len(objects) == 1 else f"{gen['part']} orient"
            changes.append(f"{label}: {gen['orient']} -> {new}")
    copies = set(counts.values())
    if len(copies) == 1 and copies != {state["copies"]}:
        n = copies.pop()
        owner = spec["plates"][name] if name in spec.get("plates", {}) else spec["parts"][name]
        owner["copies"] = n
        changes.append(f"copies: {state['copies']} -> {n}")
    elif len(copies) > 1:
        changes.append("copies differ between the parts ("
                       + ", ".join(f"{k} x{v}" for k, v in sorted(counts.items()))
                       + "); not carried over")
    if changes:
        changes.append(f"(the saved project is kept at {rel_backup})")
    return changes, None


# --------------------------------------------------------------------------
# slicing

def orca_command():
    if sys.platform == "darwin":
        exe = os.path.join(orca_profile.MAC_APP, "Contents", "MacOS", "OrcaSlicer")
        if not os.path.exists(exe):
            raise SystemExit(f"OrcaSlicer not found at {exe} (set ORCA_APP)")
        return [exe], None
    return ["xvfb-run", "-a", "flatpak", "run", FLATPAK_ID], dict(os.environ, **SANDBOX_ENV)


def slice_part(name, spec, lib, outdir, printer, copies=None):
    """Slice job ``name`` -- a part, or a plate of parts -- into one project."""
    entry = job_entry(spec, name)
    work = os.path.join(outdir, ".work", name)
    shutil.rmtree(os.path.join(work, "out"), ignore_errors=True)
    os.makedirs(os.path.join(work, "out"), exist_ok=True)
    n = copies or entry.get("copies", 1)

    inputs, objects, heights = [], {}, []
    for part in plate_parts(spec, name):
        stl = os.path.join(STL_DIR, f"{part}.stl")
        if not os.path.exists(stl):
            raise SystemExit(f"{part}: no {os.path.relpath(stl, REPO)} (run cad/v6/parts_v6.py)")
        rot = orient_matrix(part_entry(spec, part).get("orient", "IDENT"))
        work_stl = os.path.join(work, f"{part}.stl")
        heights.append(write_rotated_stl(read_stl(stl), rot, work_stl)[2])
        inputs += [work_stl] * n
        objects[f"{part}.stl"] = {"part": part, "orient": orient_name(rot)}

    machine, process, filament = build_presets(lib, spec, entry, printer)
    paths = {}
    for kind, data in (("machine", machine), ("process", process), ("filament", filament)):
        paths[kind] = os.path.join(work, f"{kind}.json")
        with open(paths[kind], "w") as fh:
            json.dump(data, fh, indent=1)

    datadir = os.path.join(outdir, ".work", "datadir", name)
    os.makedirs(datadir, exist_ok=True)     # the CLI only creates the last level
    cmd, env = orca_command()
    cmd = cmd + [
        "--datadir", datadir,
        "--load-settings", f"{paths['machine']};{paths['process']}",
        "--load-filaments", paths["filament"],
        # The GUI slices this printer without G92 E0 in layer_change_gcode;
        # only the CLI's normative check insists on it.
        "--normative-check=0",
        "--slice", "0", "--arrange", "1",
        "--export-3mf", f"{name}.3mf",
        "--outputdir", os.path.join(work, "out"),
        "--logfile", os.path.join(work, "orca.log"), "--debug", "3",
    ] + inputs
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
    gcode = os.path.join(work, "out", "plate_1.gcode")
    project = os.path.join(work, "out", f"{name}.3mf")
    if proc.returncode != 0 or not (os.path.exists(gcode) and os.path.exists(project)):
        detail = (proc.stdout + proc.stderr).strip().splitlines()[-3:]
        raise SystemExit(f"{name}: OrcaSlicer exit {proc.returncode}: {' | '.join(detail)} "
                         f"(log: {os.path.relpath(os.path.join(work, 'orca.log'), REPO)})")

    system_process = lib.resolve("process", spec["process"]["preset"])
    system_filament = lib.resolve("filament", spec["filament"]["preset"])
    system_printer = lib.resolve("machine", spec["printer"])
    cfg = json.loads(zipfile.ZipFile(project).read("Metadata/project_settings.config"))
    overridden = set(spec["process"].get("settings", {})) | set(spec["supports"]) \
        | set(entry.get("settings", {})) | {"enable_support"}
    cfg = finalize_3mf(project, {"different_settings_to_system": [
        different_keys(cfg, system_process, also=overridden & set(process)),
        different_keys(cfg, system_filament, also=overridden & set(filament)),
        # The GUI preset's own keys (print host, serial) are in no system file.
        different_keys(cfg, system_printer, also=set(lib.get("machine", printer))),
    ]})

    final_3mf = os.path.join(outdir, f"{name}.3mf")
    final_gcode = os.path.join(outdir, f"{name}.gcode")
    os.replace(project, final_3mf)
    os.replace(gcode, final_gcode)
    keys = (set(process) | set(filament) | PROJECT_KEYS) - META_KEYS - SECRET_KEYS
    for obj_name, rot in reversed(read_3mf(final_3mf)["items"]):
        if obj_name in objects:
            objects[obj_name]["rotation"] = [list(r) for r in rot]
    missing = [o for o, v in objects.items() if "rotation" not in v]
    if missing:
        raise SystemExit(f"{name}: the project names no object {', '.join(missing)}")
    state = {
        "sha256": sha256(final_3mf),
        "copies": n,
        "objects": objects,
        "config": {k: cfg[k] for k in sorted(keys) if k in cfg},
    }
    with open(os.path.join(outdir, ".work", f"{name}.state.json"), "w") as fh:
        json.dump(state, fh, indent=1)
    stats = gcode_stats(final_gcode)
    stats.update(height=f"{max(heights):.1f}", copies=n, supports=bool(entry.get("supports")))
    return final_3mf, stats


def gcode_stats(path):
    """Pull the header numbers worth printing, plus the real bed footprint."""
    stats = {}
    patterns = {
        "grams": r"^; filament used \[g\] = ([\d.]+)",
        "time": r"^; estimated printing time \(normal mode\) = (.+)",
        "layers": r"^; total layer number: (\d+)",
    }
    xs, ys = [], []
    # machine_start_gcode draws a purge line across X55..X-55 at the front of
    # the bed. It is extrusion, but it is not the part. The custom start/end
    # blocks are wrapped in ";TYPE:Custom", so only measure inside a real role.
    in_part = False
    with open(path, errors="ignore") as fh:
        for line in fh:
            if line.startswith(";TYPE:"):
                in_part = line[len(";TYPE:"):].strip() != "Custom"
            if line.startswith("; "):
                for key, pat in patterns.items():
                    if key not in stats:
                        m = re.match(pat, line)
                        if m:
                            stats[key] = m.group(1).strip()
            elif in_part and line.startswith("G1 ") and " E" in line:
                mx = re.search(r"X(-?\d+\.?\d*)", line)
                my = re.search(r"Y(-?\d+\.?\d*)", line)
                if mx:
                    xs.append(float(mx.group(1)))
                if my:
                    ys.append(float(my.group(1)))
    if xs and ys:
        stats["footprint"] = f"{max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f}"
        # The 5M Pro bed is centre-origin (-110..110); a part centred on (0,0)
        # is what we want to send. See the "always center prints" rule.
        stats["centre"] = f"({(min(xs)+max(xs))/2:+.1f}, {(min(ys)+max(ys))/2:+.1f})"
        stats["in_bounds"] = (min(xs) >= -110 and max(xs) <= 110
                              and min(ys) >= -110 and max(ys) <= 110)
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", metavar="name",
                    help="part names (as in cad/v6/stl) or plate names (print_settings.json 'plates')")
    ap.add_argument("--all", action="store_true",
                    help="every plate, and every part on no plate")
    ap.add_argument("--harvest", action="store_true",
                    help="only read projects saved from the GUI back into print_settings.json")
    ap.add_argument("--copies", type=int, help="copies on the plate, this run only")
    ap.add_argument("--printer", help="GUI printer preset to use (default: the one that "
                                      "inherits the spec's printer and has a print host)")
    ap.add_argument("--outdir", default=OUT_DIR)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    args = ap.parse_args()

    spec = load_settings()
    if args.all:
        names = jobs(spec)
    elif args.names:
        names = args.names
        for n in names:
            job_entry(spec, n)
    elif args.harvest:
        names = sorted(spec.get("plates", {})) + sorted(spec["parts"])
    else:
        ap.error("name at least one part or plate, or pass --all or --harvest")

    lib = orca_profile.Library()
    printer = choose_printer(lib, spec, args.printer)
    os.makedirs(os.path.join(args.outdir, ".work"), exist_ok=True)

    blocked, harvested = set(), False
    for name in names:
        changes, problem = harvest(name, spec, lib, args.outdir, printer)
        if problem:
            print(f"!! {problem}")
            blocked.add(name)
        if changes:
            harvested = True
            print(f"{name}: read back from the saved project")
            for c in changes:
                print(f"    {c}")
    if harvested:
        save_settings(spec)
        print(f"-> {os.path.relpath(SETTINGS, REPO)} updated; commit it\n")
    if args.harvest:
        return 1 if blocked else 0

    todo = [n for n in names if n not in blocked]
    print(f"AD5M Pro | printer preset {printer!r} | {spec['process']['preset']} "
          f"+ spec | {spec['filament']['preset']} + spec")
    failures = len(blocked)
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(slice_part, n, spec, lib, args.outdir, printer, args.copies): n
                   for n in todo}
        results = {}
        for fut in concurrent.futures.as_completed(futures):
            name = futures[fut]
            try:
                results[name] = fut.result()
            except SystemExit as exc:
                results[name] = exc
    for name in todo:
        res = results[name]
        if isinstance(res, SystemExit):
            print(f"  {name:20s} FAILED: {res}")
            failures += 1
            continue
        path, st = res
        flag = "" if st.get("in_bounds", True) else "  !! OFF BED"
        print(f"  {name:20s} x{st['copies']}  {st.get('time', '?'):>10s}  "
              f"{st.get('grams', '?'):>6s} g  {st['height']:>5s} mm tall  "
              f"{'supports' if st['supports'] else 'no supports':11s}  "
              f"{st.get('footprint', '?'):>13s} mm  centre {st.get('centre', '?')}{flag}"
              f"  -> {os.path.relpath(path, REPO)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
