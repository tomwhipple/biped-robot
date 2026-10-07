"""cad/slice.py: the v6 print table, and the GUI-save round trip.

Everything but the last test runs without OrcaSlicer: presets come from a
stub library and the projects are built here. The last test drives the real
slicer and skips where it is not installed.
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
sys.path.insert(0, os.path.join(HERE, "..", "cad"))
sys.path.insert(0, os.path.join(HERE, "..", "cad", "v6"))

import slice as S  # noqa: E402

SPEC = S.load_settings()
ROLLUP = os.path.join(HERE, "..", "docs", "design-v6", "parts_v6_rollup.txt")


# ---------------------------------------------------------------- the table

def printed_parts():
    """Parts with qty > 0 in the generated rollup; the soles are cut, not printed."""
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
    "pelvis_v7": ("pelvis_v7", "PRINT_ORIENT", None),
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
        value = value[key][0]
    if isinstance(value, str):
        value = S.ROTATIONS[value]
    return np.array(value, float)


@pytest.mark.parametrize("part", [
    pytest.param(p, marks=pytest.mark.xfail(strict=True, reason="stale module constant"))
    if p in STALE else p for p in sorted(ORIENT_SOURCES)])
def test_orientation_matches_the_module(part):
    table = np.array(S.orient_matrix(SPEC["parts"][part]["orient"]), float)
    assert np.allclose(table, module_orient(part)), part


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
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
             'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">'
             '<resources><object id="2" type="model"><components>'
             '<component p:path="/3D/Objects/o.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 5 5 5"/>'
             '</components></object></resources><build>'
             + "".join(f'<item objectid="2" transform="{rot} 50 50 0" printable="1"/>' for rot in items)
             + "</build></model>")
    settings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config><object id="2">'
                '<metadata key="name" value="widget.stl"/><metadata key="extruder" value="1"/>'
                f'{object_meta}<part id="1" subtype="normal_part"><metadata key="name" value="widget.stl"/>'
                '</part></object></config>')
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
    state = {"sha256": S.sha256(str(path)), "orient": "RY_XUP",
             "rotation": [list(r) for r in S.read_3mf(str(path))["rotations"][0]],
             "copies": 1, "config": {k: v for k, v in cfg.items() if k != "print_host"}}
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
