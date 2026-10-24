"""cad/slice.py: the v6 print table, and the GUI-save round trip.

Everything but the last tests runs without OrcaSlicer: presets come from a
stub library and the projects are built here. The last tests drive the real
slicer and skip where it is not installed.
"""
import json
import os
import re
import struct
import sys
import zipfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))
sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))

import paths  # noqa: E402
import slice as S  # noqa: E402

SPEC = S.load_settings()
ROLLUP = os.path.join(paths.BUILD, "v6", "reports", "parts_v6_rollup.txt")


# ---------------------------------------------------------------- the table

def printed_parts():
    """Parts with qty > 0 in the generated rollup; the soles are cut, not printed."""
    if not os.path.exists(ROLLUP):
        pytest.fail(f"no {os.path.relpath(ROLLUP, paths.ROOT)}: run `make cad`, which writes it")
    out = set()
    for line in open(ROLLUP):
        m = re.match(r"(\S+)\s+(\d+)\s+[\d.]+\s", line)
        if m and int(m.group(2)) > 0 and not m.group(1).startswith("sole_"):
            out.add(m.group(1))
    return out


def test_every_printed_part_has_an_entry_and_an_stl():
    assert printed_parts() == set(SPEC["parts"])
    for part, entry in SPEC["parts"].items():
        assert os.path.exists(os.path.join(S.STL_DIR, f"{part}.stl")), part
        S.orient_matrix(entry["orient"])
        assert isinstance(entry["supports"], bool), part


def test_plates_hold_known_parts_that_share_a_process():
    for name, plate in SPEC.get("plates", {}).items():
        assert name not in SPEC["parts"], name
        assert len(plate["parts"]) > 1 and set(plate["parts"]) <= set(SPEC["parts"]), name
        S.job_entry(SPEC, name)           # raises if the parts disagree
    jobs = S.jobs(SPEC)
    on_plates = [p for plate in SPEC.get("plates", {}).values() for p in plate["parts"]]
    # every printed part is in exactly one --all project
    assert sorted(on_plates + [j for j in jobs if j in SPEC["parts"]]) == sorted(SPEC["parts"])


def test_both_feet_are_one_print():
    assert S.plate_parts(SPEC, "feet") == ["foot_L", "foot_R"]
    jobs = S.jobs(SPEC)
    assert "feet" in jobs and "foot_L" not in jobs and "foot_R" not in jobs


def test_a_plate_whose_parts_disagree_is_refused():
    spec = {"parts": {"a": {"orient": "IDENT", "supports": True},
                      "b": {"orient": "IDENT", "supports": False}},
            "plates": {"ab": {"parts": ["a", "b"]}}}
    with pytest.raises(SystemExit, match="disagree on 'supports'"):
        S.job_entry(spec, "ab")


# part -> (module, constant, key in a dict-valued constant). yaw_carrier_v6
# has no constant of its own; its RX180 is the prototype's, checked by
# test_yaw_carrier_horn_plate_down.
ORIENT_SOURCES = {
    "leg_link_v6": ("leg_link_v6", "PRINT_ORIENT", None),
    "hip_yoke_v6": ("hip_yoke_v6", "PRINT_ORIENT", None),
    "ankle_link": ("ankle_link", "PRINT_ORIENT", None),
    "foot_L": ("foot_v6", "PRINT_ORIENT", None),
    "foot_R": ("foot_v6", "PRINT_ORIENT", None),
    "arm_upper_v6_L": ("arm_v6", "PRINT_ORIENT", "arm_upper_v6"),
    "arm_upper_v6_R": ("arm_v6", "PRINT_ORIENT", "arm_upper_v6"),
    "arm_fore_v6_L": ("arm_v6", "PRINT_ORIENT", "arm_fore_v6"),
    "arm_fore_v6_R": ("arm_v6", "PRINT_ORIENT", "arm_fore_v6"),
    "neck_floor": ("neck_floor", "PRINT_ORIENT", "neck_floor"),
    "shoulder_girdle_v6": ("shoulder_girdle_v6", "PRINT_ORIENT", "shoulder_girdle_v6"),
    "pelvis_v8": ("torso_v8", "PRINT_ORIENT", "pelvis_v8"),
    "torso_v8": ("torso_v8", "PRINT_ORIENT", "torso_v8"),
    "front_cover_v8": ("torso_v8", "PRINT_ORIENT", "front_cover_v8"),
    "gd_plate": ("gd_plate", "PRINT_ORIENT", "gd_plate"),
    "power_cover_v8": ("torso_v8", "PRINT_ORIENT", "power_cover_v8"),
    "battery_door_v8": ("torso_v8", "PRINT_ORIENT", "battery_door_v8"),
    "yaw_bearing_housing": ("yaw_bearing_housing", "PRINT_ORIENT", None),
    "head_shell": ("head", "PRINT_ORIENT", None),
    "head_face": ("head", "PRINT_ORIENT_FACE", None),
}
# head.py's constant is eye(3), but head_face() is built in the assembly
# frame (the plate is 3 mm along X), so eye(3) stands it on its edge. When
# head.py is fixed this xfail turns into a failure: drop it from here.
STALE = {"head_face"}


def module_orient(part):
    import importlib
    mod, attr, key = ORIENT_SOURCES[part]
    value = getattr(importlib.import_module(mod), attr)
    if key is not None:
        value = value[key]
        if len(value) == 2:          # (matrix, note), not the matrix's three rows
            value = value[0]
    if isinstance(value, str):
        value = S.ROTATIONS[value]
    return np.array(value, float)


@pytest.mark.parametrize("part", [
    pytest.param(p, marks=pytest.mark.xfail(strict=True, reason="stale module constant"))
    if p in STALE else p for p in sorted(ORIENT_SOURCES)])
def test_orientation_matches_the_module(part):
    table = np.array(S.orient_matrix(SPEC["parts"][part]["orient"]), float)
    assert np.allclose(table, module_orient(part)), part


@pytest.mark.parametrize("variant", sorted(SPEC["variants"]))
def test_variant_parts_are_sliceable_and_kept_out_of_the_default_table(variant):
    """Each variant is named for its module, which builds its PIECES; its
    one plate (named for it too) holds them, in the module's PRINT_ORIENT."""
    import importlib
    mod = importlib.import_module(variant)
    var = SPEC["variants"][variant]
    assert set(var["parts"]) == set(mod.PIECES)
    assert not set(var["parts"]) & set(SPEC["parts"])
    for part, entry in var["parts"].items():
        assert os.path.exists(os.path.join(S.STL_DIR, f"{part}.stl")), part
        assert entry["supports"] is False, part
        assert np.allclose(np.array(S.orient_matrix(entry["orient"]), float),
                           np.array(mod.PRINT_ORIENT[part], float)), part
    spec = S.with_variants(SPEC, [variant])
    assert S.plate_parts(spec, variant) == list(mod.PIECES)
    S.job_entry(spec, variant)
    # entries are shared, so a harvested GUI save lands in 'variants'
    first = mod.PIECES[0]
    assert spec["parts"][first] is var["parts"][first]


def test_an_unknown_variant_is_refused():
    with pytest.raises(SystemExit, match="not in"):
        S.with_variants(SPEC, ["no_such_variant"])


def test_yaw_carrier_horn_plate_down():
    # RX180 puts the horn plate (the biggest flat face) on the bed.
    tris = S.read_stl(os.path.join(S.STL_DIR, "yaw_carrier_v6.stl"))

    def contact(rot):
        rt = [[tuple(sum(r[k] * v[k] for k in range(3)) for r in rot) for v in t] for t in tris]
        z0 = min(v[2] for t in rt for v in t)
        area = 0.0
        for t in rt:
            if all(abs(v[2] - z0) < 0.01 for v in t):
                a = [t[1][i] - t[0][i] for i in range(3)]
                b = [t[2][i] - t[0][i] for i in range(3)]
                area += 0.5 * abs(a[0] * b[1] - a[1] * b[0])
        return area

    table = S.orient_matrix(SPEC["parts"]["yaw_carrier_v6"]["orient"])
    assert contact(table) > 1500
    assert contact(table) > 5 * contact(S.ROTATIONS["IDENT"])


# ---------------------------------------------------------------- pieces

def test_rotated_stl(tmp_path):
    tri = ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 0.0, 5.0))
    src = tmp_path / "a.stl"
    src.write_bytes(b"x" * 80 + struct.pack("<I", 1)
                    + struct.pack("<12fH", 0, -1, 0, *tri[0], *tri[1], *tri[2], 0))
    out = tmp_path / "b.stl"
    size = S.write_rotated_stl(S.read_stl(str(src)), S.ROTATIONS["RY_XUP"], str(out))
    assert size == pytest.approx((5.0, 0.0, 10.0))     # model +X is now up
    (v0, v1, v2), = S.read_stl(str(out))
    assert min(v[2] for v in (v0, v1, v2)) == pytest.approx(0.0)


def test_enforcer_boxes_follow_the_print_orientation():
    entry = {"support_enforcers": [{"box": [1, 2, 3, 5, -10, -4]}]}
    lo = [-6, -8, 0]             # the rotated part's lowest corner
    (bx, by, bz), = S.enforcer_boxes(entry, S.ROTATIONS["RX180"], lo)
    assert bx == (7, 8) and by == (3, 5) and bz == (4, 10)   # print (x, -y, -z) less lo


def enforcer_project(path, names, extra_part=""):
    """Objects named ``names`` sharing one mesh file, as the CLI exports copies."""
    resources = "".join(
        f'<object id="{2 * i + 2}" type="model">\n   <components>\n    '
        '<component p:path="/3D/Objects/w.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>'
        '\n   </components>\n  </object>' for i in range(len(names)))
    settings = "".join(
        f'<object id="{2 * i + 2}"><metadata key="name" value="{n}"/>'
        f'<part id="1" subtype="normal_part"><metadata key="name" value="{n}"/></part>{extra_part}'
        '</object>' for i, n in enumerate(names))
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("Metadata/project_settings.config", "{}")
        z.writestr("3D/3dmodel.model",
                   '<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
                   'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">'
                   f'<resources>{resources}</resources><build/></model>')
        z.writestr("3D/Objects/w.model", '<model><resources>\n  <object id="1" type="model"><mesh>'
                   '</mesh></object>\n </resources></model>')
        z.writestr("Metadata/model_settings.config", f"<config>{settings}</config>")
        z.writestr("Metadata/plate_1.gcode", "G1 X0")


def test_enforcers_go_on_every_copy_and_harvest_keeps_them(tmp_path):
    src, dst = tmp_path / "a.3mf", tmp_path / "b.3mf"
    enforcer_project(src, ["widget.stl", "widget.stl", "other.stl"])
    box = ((1.0, 2.0), (3.0, 4.0), (5.0, 6.0))
    S.add_enforcers(str(src), str(dst), {"widget.stl": [box]})
    z = zipfile.ZipFile(dst)
    assert "Metadata/plate_1.gcode" not in z.namelist()        # sliced again from here
    mesh = z.read("3D/Objects/w.model").decode()
    # the next id after every object in the project (mesh 1, the copies 2, 4, 6)
    assert mesh.count('<object id="7" type="model">') == 1 and 'x="2.0000" y="4.0000" z="6.0000"' in mesh
    model = z.read("3D/3dmodel.model").decode()
    assert model.count('objectid="7"') == 2                    # both copies, one shared mesh
    settings = z.read("Metadata/model_settings.config").decode()
    assert settings.count('subtype="support_enforcer"') == 2
    assert S.read_3mf(str(dst))["extras"] == []
    # an enforcer drawn in the GUI is still one harvest cannot carry
    enforcer_project(src, ["widget.stl"], '<part id="5" subtype="support_enforcer">'
                     '<metadata key="name" value="Support enforcer"/></part>')
    assert S.read_3mf(str(src))["extras"] == ["a support_enforcer volume"]


def test_enforcer_ids_are_new_across_a_plate_of_different_parts(tmp_path):
    """The CLI numbers a plate's objects in one namespace over all its .model
    files (torso_frame: torso 1/2, pelvis 3/4, door 5/6). An enforcer id taken
    per mesh file collided with them and OrcaSlicer refused to slice."""
    src, dst = tmp_path / "a.3mf", tmp_path / "b.3mf"
    with zipfile.ZipFile(src, "w") as z:
        z.writestr("Metadata/project_settings.config", "{}")
        comps = "".join(
            f'<object id="{2 * i + 2}" type="model">\n   <components>\n    '
            f'<component p:path="/3D/Objects/{n}.model" objectid="{2 * i + 1}" '
            'transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n   </components>\n  </object>'
            for i, n in enumerate(("big", "small")))
        z.writestr("3D/3dmodel.model",
                   '<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
                   'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">'
                   f'<resources>{comps}</resources><build/></model>')
        for i, n in enumerate(("big", "small")):
            z.writestr(f"3D/Objects/{n}.model", f'<model><resources>\n  <object id="{2 * i + 1}" type="model">'
                       '<mesh></mesh></object>\n </resources></model>')
        z.writestr("Metadata/model_settings.config", "<config>" + "".join(
            f'<object id="{2 * i + 2}"><metadata key="name" value="{n}.stl"/>'
            f'<part id="{2 * i + 1}" subtype="normal_part"><metadata key="name" value="{n}.stl"/></part></object>'
            for i, n in enumerate(("big", "small"))) + "</config>")
    boxes = [((0.0, 1.0), (0.0, 1.0), (0.0, float(k + 1))) for k in range(3)]
    S.add_enforcers(str(src), str(dst), {"big.stl": boxes})
    z = zipfile.ZipFile(dst)
    ids = [int(x) for n in z.namelist() if n.endswith(".model")
           for x in re.findall(r'<object id="(\d+)"', z.read(n).decode())]
    assert sorted(ids) == [1, 2, 3, 4, 5, 6, 7]                # 5, 6, 7: the three enforcers, all new


def layout_files(objects):
    """Project files with one box mesh per object ((name, (sx, sy), [(deg, x, y)]))
    and its items turned and placed as an arrange might leave them."""
    resources, build, settings, files = "", "", "", {}
    for i, (name, (sx, sy), items) in enumerate(objects):
        oid = 2 * i + 2
        verts = "".join(f'<vertex x="{x}" y="{y}" z="{z}"/>'
                        for z in (0, 5) for y in (0, sy) for x in (0, sx))
        files[f"3D/Objects/{i}.model"] = (f'<model><resources>\n <object id="1" type="model"><mesh>'
                                         f'<vertices>{verts}</vertices></mesh></object>\n '
                                         '</resources></model>').encode()
        resources += (f'<object id="{oid}" type="model">\n   <components>\n    '
                      f'<component p:path="/3D/Objects/{i}.model" objectid="1" '
                      'transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n   </components>\n  </object>')
        for deg, x, y in items:
            c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
            build += f'<item objectid="{oid}" transform="{c} {s} 0 {-s} {c} 0 0 0 1 {x} {y} 0" printable="1"/>'
        settings += f'<object id="{oid}"><metadata key="name" value="{name}"/></object>'
    files["3D/3dmodel.model"] = (f'<model><resources>{resources}</resources><build>{build}</build></model>').encode()
    files["Metadata/model_settings.config"] = f"<config>{settings}</config>".encode()
    return files


def bed_boxes(files):
    """Each item's footprint on the bed, and its turn in degrees."""
    model = files["3D/3dmodel.model"].decode()
    out = []
    for tag in re.findall(r"<item [^>]*/>", model):
        m = [float(v) for v in re.search(r'transform="([^"]+)"', tag).group(1).split()]
        oid = re.search(r'objectid="(\d+)"', tag).group(1)
        x0, x1, y0, y1 = S._object_bbox(files, model, oid)
        pts = [(x * m[0] + y * m[3] + m[9], x * m[1] + y * m[4] + m[10]) for x in (x0, x1) for y in (y0, y1)]
        out.append((min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts),
                    max(p[1] for p in pts), np.degrees(np.arctan2(m[1], m[0]))))
    return out


def test_copies_are_laid_square_to_the_bed_apart_and_centred():
    files = layout_files([("a.stl", (60, 20), [(-25.7, 3, 4), (140.0, -50, 9)]),
                          ("b.stl", (130, 80), [(12.0, 0, 0)])])
    assert S.square_up(files, {"a.stl": 90}) is True
    boxes = bed_boxes(files)
    assert [round(b[4]) % 360 for b in boxes] == [90, 90, 0]          # quarter turns: a spun, b as cut
    assert boxes[0][1] - boxes[0][0] == pytest.approx(20) and boxes[2][1] - boxes[2][0] == pytest.approx(130)
    for i, p in enumerate(boxes):
        assert S.BED[0] + S.LAYOUT_MARGIN - 1e-6 <= min(p[0], p[2]) and max(p[1], p[3]) <= S.BED[1] - S.LAYOUT_MARGIN + 1e-6
        for q in boxes[i + 1:]:
            gap = max(q[0] - p[1], p[0] - q[1], q[2] - p[3], p[2] - q[3])
            assert gap >= S.LAYOUT_GAP - 1e-6
    xs = [v for b in boxes for v in b[:2]]
    ys = [v for b in boxes for v in b[2:4]]
    assert (min(xs) + max(xs)) / 2 == pytest.approx(0) and (min(ys) + max(ys)) / 2 == pytest.approx(0)
    assert S.square_up(files, {"a.stl": 90}) is False                  # already square: no second pass
    too_big = layout_files([("c.stl", (150, 150), [(0, 0, 0), (0, 0, 0)])])
    assert S.square_up(too_big, {}) is None                            # left to the slicer's arrangement


@pytest.mark.parametrize("deg, spin, change", [(90.0, 90, "spin: 0 -> 90"),
                                               (-90.0, 270, "spin: 0 -> 270"),
                                               (31.0, None, "not a quarter turn")])
def test_a_spin_on_the_bed_is_kept_as_a_quarter_turn(generated, deg, spin, change):
    tmp_path, spec, cfg = generated
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    write_project(tmp_path / "widget.3mf", cfg, [f"{c} {s} 0 {-s} {c} 0 0 0 1"])
    changes, problem = harvest(tmp_path, spec)
    assert problem is None and any(change in ch for ch in changes)
    assert spec["parts"]["widget"].get("spin") == spin
    assert spec["parts"]["widget"]["orient"] == STUB_SPEC["parts"]["widget"]["orient"]


def test_yaw_carrier_window_enforcer_matches_the_cad():
    """The enforcer box sits over the servo-plug window's sill: inside the rear
    wall's outer face and short of the idler seat's detent channel (no sill),
    across the window's height up into its ceiling bar."""
    import dimensions_v6 as V
    D = V.D
    (box,) = [e["box"] for e in SPEC["parts"]["yaw_carrier_v6"]["support_enforcers"]]
    x0, x1, y0, y1, z0, z1 = box
    za = V.CARRIER_ROLL_AXIS
    window_lo, window_hi = za + D.SV_CONN_L[0] - 0.45, za + D.SV_CONN_L[1] + 1.0
    channel_x = D.SV_IDLER_BOSS_Y - D.RIB_RELIEF_DEPTH_CLR
    channel_hw = D.SV_IDLER_BOSS_HW + D.RIB_RELIEF_CLR
    assert -19.95 < x0 < x1 < channel_x
    assert -channel_hw <= y0 < y1 <= channel_hw
    assert z0 < window_lo < z1 <= window_hi and z0 > za + D.BAY_BORE / 2   # into the bar, not past it
    entry = SPEC["parts"]["yaw_carrier_v6"]
    assert entry["supports"] and entry["settings"]["support_type"] == "normal(manual)"


def test_torso_window_enforcers_match_the_cad():
    """The torso's enforcer boxes are torso_v8's: one per window whose flat top
    spans more than a bridge, and the battery door's opening, each inside its
    window's faces. A window moved in the CAD without its box fails here."""
    import torso_v8 as T
    entry = SPEC["parts"]["torso_v8"]
    assert entry["supports"] and entry["settings"]["support_type"] == "normal(manual)"
    assert [e["box"] for e in entry["support_enforcers"]] == T.support_enforcers()
    for cut, enf in T._windows():
        bb = cut.bounding_box()
        assert (enf is not None) == (max(bb.size.X, bb.size.Y) > T.BRIDGE_OK + 1e-9)
        if enf is not None:
            x0, x1, y0, y1, z0, z1 = enf
            assert bb.min.X < x0 < x1 < bb.max.X and bb.min.Y < y0 < y1 < bb.max.Y
            assert z0 == pytest.approx(bb.min.Z - T.ENF_INTO) and z1 == pytest.approx(bb.max.Z)


def test_orient_name_round_trip():
    for name, m in S.ROTATIONS.items():
        assert S.orient_name(m) == name
    odd = S.snap(S.matmul(S.ROTATIONS["RY_XUP"], S.ROTATIONS["RY_ROLL_WALL"]))
    assert S.orient_name(odd) == [list(r) for r in odd]
    with pytest.raises(SystemExit):
        S.orient_matrix([[2, 0, 0], [0, 1, 0], [0, 0, 1]])


def test_different_keys_lists_every_departure():
    system = {"wall_loops": "2", "name": "x", "sparse_infill_density": "15%", "same": "1"}
    cfg = {"wall_loops": "3", "name": "y", "sparse_infill_density": "15%", "same": "1",
           "curr_bed_type": "Textured PEI Plate"}
    # name is metadata; curr_bed_type is in no system preset, so it is passed in.
    assert S.different_keys(cfg, system, also={"curr_bed_type"}) == "curr_bed_type;wall_loops"


# ---------------------------------------------------------------- harvest

class StubLib:
    PRESETS = {
        "machine": {"Printer": {"name": "Printer", "printhost_apikey": "secret",
                                "print_host": "1.2.3.4"}},
        "process": {"Proc": {"name": "Proc", "wall_loops": "2", "sparse_infill_density": "15%",
                             "enable_support": "0", "brim_type": "no_brim"}},
        "filament": {"Fil": {"name": "Fil", "nozzle_temperature": ["250"]}},
    }

    def resolve(self, kind, name):
        return dict(self.PRESETS[kind][name])

    def get(self, kind, name):
        return self.PRESETS[kind][name]


STUB_SPEC = {
    "printer": "Printer",
    "process": {"preset": "Proc", "settings": {"wall_loops": "3"}},
    "filament": {"preset": "Fil", "settings": {}},
    "supports": {"enable_support": "1"},
    "parts": {"widget": {"orient": "RY_XUP", "supports": True}},
}
RZ180_ROW = "-1 0 0 0 -1 0 0 0 1"
RX90_ROW = "1 0 0 0 0 1 0 -1 0"      # row-vector 3MF form of +90 deg about X


def write_project(path, cfg, items, object_meta="", paint=False):
    write_plate(path, cfg, [("widget.stl", items)], object_meta, paint)


def write_plate(path, cfg, objects, object_meta="", paint=False):
    """A project as Orca writes one: ``objects`` is [(name, [item rotation rows])]."""
    resources, build, settings = "", "", ""
    for i, (name, items) in enumerate(objects):
        oid = 2 * i + 2
        resources += (f'<object id="{oid}" type="model"><components>'
                      f'<component p:path="/3D/Objects/o.model" objectid="{oid - 1}" '
                      'transform="1 0 0 0 1 0 0 0 1 5 5 5"/></components></object>')
        build += "".join(f'<item objectid="{oid}" transform="{rot} 50 50 0" printable="1"/>'
                         for rot in items)
        settings += (f'<object id="{oid}"><metadata key="name" value="{name}"/>'
                     f'<metadata key="extruder" value="1"/>{object_meta}'
                     f'<part id="1" subtype="normal_part"><metadata key="name" value="{name}"/>'
                     '</part></object>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
             'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">'
             f'<resources>{resources}</resources><build>{build}</build></model>')
    settings = f'<?xml version="1.0" encoding="UTF-8"?>\n<config>{settings}</config>'
    tri = b'<triangle v1="0" v2="1" v3="2"' + (b' paint_supports="4"' if paint else b"") + b"/>"
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("Metadata/project_settings.config", json.dumps(cfg))
        z.writestr("3D/3dmodel.model", model)
        z.writestr("3D/Objects/o.model", b"<model><mesh>" + tri + b"</mesh></model>")
        z.writestr("Metadata/model_settings.config", settings)


@pytest.fixture
def generated(tmp_path, monkeypatch):
    """A project as slice_part leaves it: the 3mf, its state, a fresh spec."""
    monkeypatch.setattr(S, "REPO", str(tmp_path))
    spec = json.loads(json.dumps(STUB_SPEC))
    cfg = {"wall_loops": "3", "sparse_infill_density": "15%", "enable_support": "1",
           "brim_type": "no_brim", "nozzle_temperature": ["250"], "print_host": "1.2.3.4"}
    path = tmp_path / "widget.3mf"
    write_project(path, cfg, [RZ180_ROW])
    (tmp_path / ".work").mkdir()
    state = {"sha256": S.sha256(str(path)), "copies": 1,
             "objects": {"widget.stl": {
                 "part": "widget", "orient": "RY_XUP",
                 "rotation": [list(r) for r in S.read_3mf(str(path))["rotations"][0]]}},
             "config": {k: v for k, v in cfg.items() if k != "print_host"}}
    (tmp_path / ".work" / "widget.state.json").write_text(json.dumps(state))
    return tmp_path, spec, cfg


def harvest(tmp_path, spec):
    return S.harvest("widget", spec, StubLib(), str(tmp_path), "Printer")


def test_unsaved_project_harvests_nothing(generated):
    tmp_path, spec, _ = generated
    assert harvest(tmp_path, spec) == ([], None)
    assert spec == STUB_SPEC


def test_saved_project_carries_settings_orientation_and_copies(generated):
    tmp_path, spec, cfg = generated
    saved = dict(cfg, wall_loops="4", sparse_infill_density="40%", print_host="9.9.9.9")
    write_project(tmp_path / "widget.3mf", saved, [RX90_ROW, RX90_ROW],
                  object_meta='<metadata key="brim_type" value="outer_only"/>')
    changes, problem = harvest(tmp_path, spec)
    assert problem is None and changes
    entry = spec["parts"]["widget"]
    assert entry["settings"] == {"wall_loops": "4", "sparse_infill_density": "40%",
                                 "brim_type": "outer_only"}
    # the instance rotation acts on the already-oriented mesh
    rx90 = S.transpose(S._transform(RX90_ROW + " 0 0 0")[0])
    assert np.allclose(S.orient_matrix(entry["orient"]),
                       S.matmul(rx90, S.ROTATIONS["RY_XUP"]))
    assert entry["copies"] == 2
    # printer settings never reach the table, and a copy of the save is kept
    assert "print_host" not in json.dumps(spec)
    assert len(list((tmp_path / ".work" / "saved").iterdir())) == 1


def test_spin_on_the_bed_is_not_a_new_orientation(generated):
    tmp_path, spec, cfg = generated
    write_project(tmp_path / "widget.3mf", dict(cfg, wall_loops="4"), ["0 1 0 -1 0 0 0 0 1"])
    harvest(tmp_path, spec)
    assert spec["parts"]["widget"]["orient"] == "RY_XUP"


def test_setting_back_to_the_spec_drops_the_override(generated):
    tmp_path, spec, cfg = generated
    # generated with a 40% override in force ...
    spec["parts"]["widget"]["settings"] = {"sparse_infill_density": "40%"}
    write_project(tmp_path / "widget.3mf", dict(cfg, sparse_infill_density="40%"), [RZ180_ROW])
    state_path = tmp_path / ".work" / "widget.state.json"
    state = json.loads(state_path.read_text())
    state["config"]["sparse_infill_density"] = "40%"
    state["sha256"] = S.sha256(str(tmp_path / "widget.3mf"))
    state_path.write_text(json.dumps(state))
    # ... then set back to the spec's 15% in the GUI and saved
    write_project(tmp_path / "widget.3mf", cfg, [RZ180_ROW])
    harvest(tmp_path, spec)
    assert "settings" not in spec["parts"]["widget"]


def test_painted_project_is_not_overwritten(generated):
    tmp_path, spec, cfg = generated
    write_project(tmp_path / "widget.3mf", dict(cfg, wall_loops="4"), [RZ180_ROW], paint=True)
    changes, problem = harvest(tmp_path, spec)
    assert changes == [] and "painted" in problem
    assert spec == STUB_SPEC


PLATE_SPEC = {
    "printer": "Printer",
    "process": {"preset": "Proc", "settings": {"wall_loops": "3"}},
    "filament": {"preset": "Fil", "settings": {}},
    "supports": {"enable_support": "1"},
    "plates": {"pair": {"parts": ["left", "right"]}},
    "parts": {"left": {"orient": "IDENT", "supports": False},
              "right": {"orient": "IDENT", "supports": False}},
}
IDENT_ROW = "1 0 0 0 1 0 0 0 1"


@pytest.fixture
def generated_plate(tmp_path, monkeypatch):
    """A two-part plate as slice_part leaves it."""
    monkeypatch.setattr(S, "REPO", str(tmp_path))
    spec = json.loads(json.dumps(PLATE_SPEC))
    cfg = {"wall_loops": "3", "sparse_infill_density": "15%", "enable_support": "0",
           "brim_type": "no_brim", "nozzle_temperature": ["250"]}
    path = tmp_path / "pair.3mf"
    write_plate(path, cfg, [("left.stl", [IDENT_ROW]), ("right.stl", [IDENT_ROW])])
    (tmp_path / ".work").mkdir()
    ident = [list(r) for r in S.ROTATIONS["IDENT"]]
    state = {"sha256": S.sha256(str(path)), "copies": 1,
             "objects": {f"{p}.stl": {"part": p, "orient": "IDENT", "rotation": ident}
                         for p in ("left", "right")},
             "config": cfg}
    (tmp_path / ".work" / "pair.state.json").write_text(json.dumps(state))
    return tmp_path, spec, cfg


def harvest_plate(tmp_path, spec):
    return S.harvest("pair", spec, StubLib(), str(tmp_path), "Printer")


def test_saved_plate_shares_settings_and_keeps_each_parts_orientation(generated_plate):
    tmp_path, spec, cfg = generated_plate
    # wall_loops changed for the plate; only the left part tipped onto its side
    write_plate(tmp_path / "pair.3mf", dict(cfg, wall_loops="4"),
                [("left.stl", [RX90_ROW]), ("right.stl", [RZ180_ROW])])
    changes, problem = harvest_plate(tmp_path, spec)
    assert problem is None and changes
    assert spec["parts"]["left"]["settings"] == spec["parts"]["right"]["settings"] \
        == {"wall_loops": "4"}
    rx90 = S.transpose(S._transform(RX90_ROW + " 0 0 0")[0])
    assert np.allclose(S.orient_matrix(spec["parts"]["left"]["orient"]), rx90)
    assert spec["parts"]["right"]["orient"] == "IDENT"
    assert "copies" not in spec["plates"]["pair"]


def test_plate_copies_go_to_the_plate(generated_plate):
    tmp_path, spec, cfg = generated_plate
    write_plate(tmp_path / "pair.3mf", cfg, [("left.stl", [IDENT_ROW] * 2),
                                             ("right.stl", [IDENT_ROW] * 2)])
    harvest_plate(tmp_path, spec)
    assert spec["plates"]["pair"]["copies"] == 2
    assert "copies" not in spec["parts"]["left"]


def test_plate_saved_with_a_stranger_is_not_overwritten(generated_plate):
    tmp_path, spec, cfg = generated_plate
    write_plate(tmp_path / "pair.3mf", cfg, [("left.stl", [IDENT_ROW]), ("right.stl", [IDENT_ROW]),
                                             ("bolt.stl", [IDENT_ROW])])
    changes, problem = harvest_plate(tmp_path, spec)
    assert changes == [] and "bolt.stl" in problem
    assert spec == PLATE_SPEC


def test_state_from_before_plates_still_harvests(generated):
    tmp_path, spec, cfg = generated
    state_path = tmp_path / ".work" / "widget.state.json"
    state = json.loads(state_path.read_text())
    obj = state.pop("objects")["widget.stl"]
    state.update(orient=obj["orient"], rotation=obj["rotation"])
    state_path.write_text(json.dumps(state))
    write_project(tmp_path / "widget.3mf", dict(cfg, wall_loops="4"), [RZ180_ROW])
    changes, problem = harvest(tmp_path, spec)
    assert problem is None and spec["parts"]["widget"]["settings"] == {"wall_loops": "4"}


# ---------------------------------------------------------------- the real slicer

ORCA = os.path.join(S.orca_profile.MAC_APP, "Contents", "MacOS", "OrcaSlicer")


@pytest.mark.skipif(sys.platform != "darwin" or not os.path.exists(ORCA),
                    reason="OrcaSlicer.app not installed")
def test_real_slice_keeps_the_spec_in_the_project(tmp_path):
    lib = S.orca_profile.Library()
    printer = S.choose_printer(lib, SPEC)
    path, stats = S.slice_part("neck_floor", SPEC, lib, str(tmp_path), printer)
    cfg = json.loads(zipfile.ZipFile(path).read("Metadata/project_settings.config"))
    assert cfg["wall_loops"] == "3" and cfg["sparse_infill_density"] == "35%"
    # the GUI resets every key not listed here to the system preset's value
    listed = cfg["different_settings_to_system"][0].split(";")
    assert {"wall_loops", "sparse_infill_density", "curr_bed_type"} <= set(listed)
    assert "printhost_apikey" not in json.dumps(cfg) or cfg.get("printhost_apikey") in (None, "")
    assert stats["in_bounds"] and os.path.exists(os.path.join(tmp_path, "neck_floor.gcode"))


@pytest.mark.skipif(sys.platform != "darwin" or not os.path.exists(ORCA),
                    reason="OrcaSlicer.app not installed")
def test_real_slice_supports_the_yaw_carrier_window_only(tmp_path):
    """The enforcer reaches the slicer: support at the window's print heights
    (its sill to its ceiling bar), none under the lip or in the screw holes."""
    lib = S.orca_profile.Library()
    printer = S.choose_printer(lib, SPEC)
    path, stats = S.slice_part("yaw_carrier_v6", SPEC, lib, str(tmp_path), printer, copies=1)
    project = S.read_3mf(path)
    assert project["extras"] == []
    (_, r), = project["items"]                       # square to the bed, at the part's spin
    assert np.degrees(np.arctan2(r[1][0], r[0][0])) % 360 == pytest.approx(SPEC["parts"]["yaw_carrier_v6"]["spin"])
    assert stats["in_bounds"]
    z, typ, heights = 0.0, "", set()
    for line in open(tmp_path / "yaw_carrier_v6.gcode"):
        if line.startswith(";Z:"):
            z = float(line[3:])
        elif line.startswith(";TYPE:"):
            typ = line[6:].strip()
        elif "upport" in typ and line.startswith("G1") and " E" in line and "E-" not in line:
            heights.add(z)
    assert heights and 27.5 < min(heights) and max(heights) < 34.0


@pytest.mark.skipif(sys.platform != "darwin" or not os.path.exists(ORCA),
                    reason="OrcaSlicer.app not installed")
def test_real_slice_of_the_feet_puts_both_on_one_plate(tmp_path):
    lib = S.orca_profile.Library()
    printer = S.choose_printer(lib, SPEC)
    path, stats = S.slice_part("feet", SPEC, lib, str(tmp_path), printer)
    project = S.read_3mf(path)
    assert sorted(name for name, _ in project["items"]) == ["foot_L.stl", "foot_R.stl"]
    assert stats["in_bounds"] and os.path.exists(os.path.join(tmp_path, "feet.gcode"))
    state = json.loads((tmp_path / ".work" / "feet.state.json").read_text())
    assert set(state["objects"]) == {"foot_L.stl", "foot_R.stl"}
