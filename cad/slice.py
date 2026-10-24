#!/usr/bin/env python3
"""Slice the v6 parts into OrcaSlicer projects for the FlashForge Adventurer 5M Pro.

For each part, build/v6/gcode/ (gitignored; `make slice` runs this) gets:

    <part>.3mf    an OrcaSlicer project: the part in its print orientation,
                  every copy square to the bed (walls along X and Y), the
                  print spec's settings, the plate already sliced. Open it in
                  the GUI, check it, print it from there.
    <part>.gcode  the same slice, for the numbers (time, grams, footprint).

The settings are cad/v6/print_settings.json: the print spec on top of
Flashforge's own presets, one entry per part (orientation, supports,
copies, overrides, support-enforcer boxes), and plates: parts printed
together, sliced into one <plate>.3mf + .gcode. If a project is changed in
the GUI and saved (Cmd+S),
the next run reads the change back -- settings, orientation, copies -- into
print_settings.json before it slices that part or plate again, so it carries
forward.

    python3 cad/slice.py leg_link_v6
    python3 cad/slice.py leg_link_v6 --copies 4
    python3 cad/slice.py feet            # a plate: both feet, one project
    python3 cad/slice.py --all           # every plate, and every part on no plate
    python3 cad/slice.py --harvest       # only read saved projects back
    python3 cad/slice.py --variant hip_yoke_glued hip_yoke_glued   # a build variant's plate

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
OUT_DIR = os.path.join(REPO, "build", "v6", "gcode")

FLATPAK_ID = "com.orcaslicer.OrcaSlicer"
# The flatpak sandbox has no network, no devices, and of the repo only
# the output directory (docs/slicing.md); everything the slicer reads is staged there.
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
    "RY_YDOWN": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),       # model +Y -> print -Z
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


def angle_diff(a, b):
    """a - b in degrees, wrapped to (-180, 180]."""
    return -((b - a + 180.0) % 360.0 - 180.0)


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


def rotated_min(tris, rot):
    """The lowest corner of ``tris`` rotated by ``rot``: what write_rotated_stl subtracts."""
    return [min(sum(r[k] * v[k] for k in range(3)) for t in tris for v in t) for r in rot]


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


def with_variants(spec, names):
    """``spec`` with each named entry of its 'variants' added to 'parts' and
    'plates', for this run: parts of a build variant (assembly_v6's
    HIP_YOKE_VARIANT and the like) that the default print table leaves out.
    The entries are the variant's own dicts, so a GUI save that harvest()
    reads back lands in 'variants' when the base spec is saved."""
    if not names:
        return spec
    out = dict(spec, parts=dict(spec["parts"]), plates=dict(spec.get("plates", {})))
    for name in names:
        var = spec.get("variants", {}).get(name)
        if var is None:
            raise SystemExit(f"{name}: not in {os.path.relpath(SETTINGS, REPO)} 'variants'")
        out["parts"].update(var.get("parts", {}))
        out["plates"].update(var.get("plates", {}))
    return out


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


def plate_counts(spec, name, copies=None):
    """{part: build items} for job ``name``: a part alone takes its copies;
    on a plate each part takes the plate's copies times its own (default 1),
    so a plate of a part and its two keys holds one and two."""
    n = copies or job_entry(spec, name).get("copies", 1)
    if name not in spec.get("plates", {}):
        return {name: n}
    return {p: n * part_entry(spec, p).get("copies", 1) for p in plate_parts(spec, name)}


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
                pname = next((md.get("value") for md in part.iterfind("metadata")
                              if md.get("key") == "name"), "")
                if part.get("subtype") == "support_enforcer" and pname.startswith(ENFORCER_NAME):
                    continue        # slice_part's own, from print_settings.json
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


# Support enforcers: boxes in a part's model frame (print_settings.json
# "support_enforcers": [{"box": [x0, x1, y0, y1, z0, z1]}]). slice_part adds
# them to the project as support-enforcer volumes and slices it again; with
# manual supports (support_type normal(manual)) the slicer then supports the
# overhangs inside them and nothing else.
ENFORCER_NAME = "slice.py support enforcer"
BOX_TRIANGLES = ((0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4),
                 (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5))


def enforcer_boxes(entry, rot, lo):
    """``entry``'s enforcer boxes in its rotated STL's frame (rotated by ``rot``,
    less ``lo``), which is the frame of its object in the project."""
    out = []
    for e in entry.get("support_enforcers", []):
        x0, x1, y0, y1, z0, z1 = e["box"]
        pts = [[sum(r[k] * c[k] for k in range(3)) - lo[i] for i, r in enumerate(rot)]
               for c in [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]]
        out.append(tuple((min(p[i] for p in pts), max(p[i] for p in pts)) for i in range(3)))
    return out


def _box_object(oid, box):
    (x0, x1), (y0, y1), (z0, z1) = box
    verts = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>'
                    for z in (z0, z1) for y in (y0, y1) for x in (x0, x1))
    tris = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in BOX_TRIANGLES)
    return (f'<object id="{oid}" type="model"><mesh><vertices>{verts}</vertices>'
            f'<triangles>{tris}</triangles></mesh></object>\n ')


def read_project(path):
    with zipfile.ZipFile(path) as z:
        infos = z.infolist()
        return infos, {i.filename: z.read(i.filename) for i in infos}


def write_project(path, infos, files):
    """Write a project without its sliced gcode, so the slicer slices it again."""
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for info in infos:
            if re.fullmatch(r"Metadata/plate_\d+\.gcode(\.md5)?", info.filename):
                continue
            z.writestr(info, files[info.filename])


def add_enforcers(src, dst, boxes):
    """Copy project ``src`` to ``dst`` with support-enforcer volumes on every
    object named in ``boxes`` ({object name: [box, ...]}, enforcer_boxes'
    frame), and without its sliced gcode."""
    infos, files = read_project(src)
    put_enforcers(files, boxes)
    write_project(dst, infos, files)


# Square to the bed. The CLI's arrange turns copies to arbitrary angles
# (the yaw carrier came out at -25.7 deg, with --allow-rotations off), and a
# straight wall prints best along X or Y. Every copy gets a quarter turn --
# its part's "spin", 0 by default -- and the copies go in rows, LAYOUT_GAP
# apart, centred on the bed.
BED = (-110.0, 110.0)          # printable area, centre origin (docs/slicing.md, Bed geometry)
LAYOUT_GAP = 10.0
LAYOUT_MARGIN = 5.0
QUARTER = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}


def _floats(s):
    return [float(v) for v in s.split()]


def _object_bbox(files, model, oid):
    """An object's xy extent in its own frame: its first part's mesh through
    the component transform."""
    obj = re.search(rf'<object id="{oid}"[^>]*>\s*<components>(.*?)</components>', model, re.S)
    comp = re.search(r"<component [^>]*/>", obj.group(1)).group(0)
    path = re.search(r'p:path="([^"]+)"', comp).group(1).lstrip("/")
    sub = re.search(r'objectid="(\d+)"', comp).group(1)
    t = re.search(r'transform="([^"]+)"', comp)
    m = _floats(t.group(1)) if t else [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    mesh = re.search(rf'<object id="{sub}"[^>]*>(.*?)</object>', files[path].decode(), re.S).group(1)
    v = [tuple(map(float, xyz)) for xyz in re.findall(r'x="([^"]+)" y="([^"]+)" z="([^"]+)"', mesh)]
    lo = [min(p[i] for p in v) for i in range(3)]
    hi = [max(p[i] for p in v) for i in range(3)]
    corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    xs = [x * m[0] + y * m[3] + z * m[6] + m[9] for x, y, z in corners]
    ys = [x * m[1] + y * m[4] + z * m[7] + m[10] for x, y, z in corners]
    return min(xs), max(xs), min(ys), max(ys)


def square_up(files, spins):
    """Re-place every build item square to the bed (``spins``: {object name:
    quarter turn in degrees}). Returns True if anything moved, None if the
    copies do not fit the bed that way (the project is then left alone)."""
    model = files["3D/3dmodel.model"].decode()
    settings = files["Metadata/model_settings.config"].decode()
    names = {oid: re.search(r'<metadata key="name" value="([^"]*)"', body).group(1)
             for oid, body in re.findall(r'<object id="(\d+)">(.*?)</object>', settings, re.S)}
    items, boxes = [], {}
    for tag in re.findall(r"<item [^>]*/>", model):
        oid = re.search(r'objectid="(\d+)"', tag).group(1)
        if oid not in boxes:
            boxes[oid] = _object_bbox(files, model, oid)
        spin = spins.get(names.get(oid), 0)
        x0, x1, y0, y1 = boxes[oid]
        w, h = (x1 - x0, y1 - y0) if spin in (0, 180) else (y1 - y0, x1 - x0)
        items.append(dict(tag=tag, spin=spin, w=w, h=h, centre=((x0 + x1) / 2, (y0 + y1) / 2),
                          old=_floats(re.search(r'transform="([^"]+)"', tag).group(1))))
    usable = BED[1] - BED[0] - 2 * LAYOUT_MARGIN
    rows, row, width = [], [], 0.0
    for it in items:
        if row and width + LAYOUT_GAP + it["w"] > usable:
            rows.append(row)
            row, width = [], 0.0
        width += (LAYOUT_GAP if row else 0.0) + it["w"]
        row.append(it)
    rows.append(row)
    heights = [max(it["h"] for it in r) for r in rows]
    if (sum(heights) + LAYOUT_GAP * (len(rows) - 1) > usable
            or any(it["w"] > usable for it in items)):
        return None
    moved = False
    top = (sum(heights) + LAYOUT_GAP * (len(rows) - 1)) / 2
    for r, h in zip(rows, heights):
        x = -(sum(it["w"] for it in r) + LAYOUT_GAP * (len(r) - 1)) / 2
        for it in r:
            c, s = QUARTER[it["spin"]]
            ox, oy = it["centre"]
            tx = x + it["w"] / 2 - (ox * c - oy * s)
            ty = top - h / 2 - (ox * s + oy * c)
            new = [c, s, 0, -s, c, 0, 0, 0, 1, tx, ty, it["old"][11]]
            if any(abs(a - b) > 1e-3 for a, b in zip(new, it["old"])):
                moved = True
            tf = " ".join(f"{v:.6g}" for v in new)
            model = model.replace(it["tag"], re.sub(r'transform="[^"]+"', f'transform="{tf}"', it["tag"]), 1)
            x += it["w"] + LAYOUT_GAP
        top -= h + LAYOUT_GAP
    files["3D/3dmodel.model"] = model.encode()
    return moved


def put_enforcers(files, boxes):
    model = files["3D/3dmodel.model"].decode()
    settings = files["Metadata/model_settings.config"].decode()
    added = {}                                  # (mesh file, box) -> its object id there
    # object ids are one namespace across every .model file in the project (the
    # CLI numbers a plate's parts 1..2n over their mesh files and the main
    # model): a new enforcer object takes the next id after all of them
    next_id = 1 + max(int(x) for name, data in files.items() if name.endswith(".model")
                      for x in re.findall(rb'<object id="(\d+)"', data))
    for m in reversed(list(re.finditer(r'<object id="(\d+)">(.*?)</object>', settings, re.S))):
        name = re.search(r'<metadata key="name" value="([^"]*)"', m.group(2)).group(1)
        if name not in boxes:
            continue
        comps = re.search(rf'<object id="{m.group(1)}"[^>]*>\s*<components>(.*?)</components>',
                          model, re.S)
        path = re.search(r'p:path="([^"]+)"', comps.group(1)).group(1)
        new_comps, new_parts = "", ""
        for i, box in enumerate(boxes[name]):
            if (path, box) not in added:
                mesh = files[path.lstrip("/")].decode()
                eid, next_id = next_id, next_id + 1
                files[path.lstrip("/")] = mesh.replace(
                    "</resources>", _box_object(eid, box) + "</resources>", 1).encode()
                added[(path, box)] = eid
            eid = added[(path, box)]
            new_comps += (f'\n    <component p:path="{path}" objectid="{eid}" '
                          'transform="1 0 0 0 1 0 0 0 1 0 0 0"/>')
            new_parts += (f'    <part id="{eid}" subtype="support_enforcer">\n'
                          f'      <metadata key="name" value="{ENFORCER_NAME} {i + 1}"/>\n'
                          '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                          '    </part>\n')
        model = model[:comps.end(1)] + new_comps + model[comps.end(1):]
        end = m.end() - len("</object>")
        settings = settings[:end] + new_parts + settings[end:]
    if not added:
        raise SystemExit(f"project has no object named {', '.join(boxes)}")
    files["3D/3dmodel.model"] = model.encode()
    files["Metadata/model_settings.config"] = settings.encode()


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
            continue
        # Not tipped: a spin on the bed. A quarter turn is the part's "spin";
        # any other angle cannot be kept square to the bed.
        angle = math.degrees(math.atan2(r_saved[1][0], r_saved[0][0]))
        if abs(angle_diff(angle, math.degrees(math.atan2(gen["rotation"][1][0],
                                                         gen["rotation"][0][0])))) < 1.0:
            continue
        part = spec["parts"][gen["part"]]
        label = "spin" if len(objects) == 1 else f"{gen['part']} spin"
        quarter = round(angle / 90) % 4 * 90
        if abs(angle_diff(angle, quarter)) < 1.0:
            if quarter != part.get("spin", 0):
                changes.append(f"{label}: {part.get('spin', 0)} -> {quarter}")
                if quarter:
                    part["spin"] = quarter
                else:
                    part.pop("spin", None)
        else:
            changes.append(f"{label}: turned {angle:.1f} deg on the bed, not a quarter turn; "
                           "not carried (copies are laid square to the bed)")
    # a plate's count per part is its copies times the part's own
    own = {o: (spec["parts"][objects[o]["part"]].get("copies", 1) if name in spec.get("plates", {}) else 1)
           for o in counts}
    copies = {c / own[o] for o, c in counts.items()}
    if len(copies) == 1 and next(iter(copies)).is_integer():
        copies = {int(copies.pop())}
    elif len(copies) == 1:
        copies = {0, -1}                    # not a whole multiple: report, do not carry
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
    per_part = plate_counts(spec, name, copies)

    inputs, objects, heights, enforcers = [], {}, [], {}
    for part in plate_parts(spec, name):
        stl = os.path.join(STL_DIR, f"{part}.stl")
        if not os.path.exists(stl):
            raise SystemExit(f"{part}: no {os.path.relpath(stl, REPO)} (run cad/v6/parts_v6.py)")
        rot = orient_matrix(part_entry(spec, part).get("orient", "IDENT"))
        work_stl = os.path.join(work, f"{part}.stl")
        tris = read_stl(stl)
        heights.append(write_rotated_stl(tris, rot, work_stl)[2])
        inputs += [work_stl] * per_part[part]
        objects[f"{part}.stl"] = {"part": part, "orient": orient_name(rot)}
        boxes = enforcer_boxes(part_entry(spec, part), rot, rotated_min(tris, rot))
        if boxes:
            enforcers[f"{part}.stl"] = boxes

    machine, process, filament = build_presets(lib, spec, entry, printer)
    paths = {}
    for kind, data in (("machine", machine), ("process", process), ("filament", filament)):
        paths[kind] = os.path.join(work, f"{kind}.json")
        with open(paths[kind], "w") as fh:
            json.dump(data, fh, indent=1)

    datadir = os.path.join(outdir, ".work", "datadir", name)
    os.makedirs(datadir, exist_ok=True)     # the CLI only creates the last level
    gcode = os.path.join(work, "out", "plate_1.gcode")
    project = os.path.join(work, "out", f"{name}.3mf")

    def orca(args):
        cmd, env = orca_command()
        cmd = cmd + [
            "--datadir", datadir,
            # The GUI slices this printer without G92 E0 in layer_change_gcode;
            # only the CLI's normative check insists on it.
            "--normative-check=0",
            "--slice", "0",
            "--export-3mf", f"{name}.3mf",
            "--outputdir", os.path.join(work, "out"),
            "--logfile", os.path.join(work, "orca.log"), "--debug", "3",
        ] + args
        proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
        if proc.returncode != 0 or not (os.path.exists(gcode) and os.path.exists(project)):
            detail = (proc.stdout + proc.stderr).strip().splitlines()[-3:]
            raise SystemExit(f"{name}: OrcaSlicer exit {proc.returncode}: {' | '.join(detail)} "
                             f"(log: {os.path.relpath(os.path.join(work, 'orca.log'), REPO)})")

    orca(["--load-settings", f"{paths['machine']};{paths['process']}",
          "--load-filaments", paths["filament"], "--arrange", "1"] + inputs)
    # The CLI can neither keep copies square nor take enforcer volumes with
    # STL inputs: rework its own project and slice that again, on the same
    # settings.
    infos, files = read_project(project)
    moved = square_up(files, {f"{p}.stl": part_entry(spec, p).get("spin", 0)
                              for p in plate_parts(spec, name)})
    if moved is None:
        print(f"  {name}: the copies do not fit the bed square to it; "
              "kept the slicer's arrangement", file=sys.stderr)
    if enforcers:
        put_enforcers(files, enforcers)
    if moved or enforcers:
        reworked = os.path.join(work, f"{name}.reworked.3mf")
        write_project(reworked, infos, files)
        os.remove(project)
        os.remove(gcode)
        orca([reworked])

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
    ap.add_argument("--variant", action="append", default=[], metavar="NAME",
                    help="also slice the parts and plates of this print_settings.json "
                         "'variants' entry (repeatable)")
    args = ap.parse_args()

    base = load_settings()
    spec = with_variants(base, args.variant)
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
        save_settings(base)
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
