# Hip yoke strength and clearance

The FEM and swept-clearance tools behind
[the 2026-11-02 record](../../docs/design-v6/2026-11-02-hip-yoke-roll-failure.md).
Everything they write goes to `build/experiments/hip-yoke-strength/` (gitignored).
Run them from the repo root (the CAD scripts read the cwd as the checkout to
build from) with the main checkout's `.venv/bin/python`.

FEM needs gmsh and CalculiX (`ccx`) from FreeCAD.app's `bin/`; set
`FREECAD_BIN` if it is not at `/Applications/FreeCAD.app/Contents/Resources/bin`.

| script | what it does |
|---|---|
| `export_glued.py <tag>` | builds the two glued pieces from `cad/v6/hip_yoke_glued.py` in the cwd's checkout and writes `fem/<tag>_union.step` (fused: perfect glue), `_roll.step`, `_pitch.step`, `_union.stl` |
| `fem_yoke.py <tag> [load] [clmax]` | one linear-static case on `fem/<tag>_union.step`: gmsh second-order straight-sided tets (clmax 1.0 mm), CalculiX, isotropic PETG (E 2000 MPa, ν 0.38). `FEM_CASE` picks the case: `roll` (4.9 N·m about X on the roll horn pad's four bolt holes, pitch pads fixed), `pitch` (2.94 N·m about Y on the pitch horn pad, roll pads fixed), `yaw` (2.94 N·m about Z as ±x forces on the two pitch pads, roll pads fixed), `fz` (100 N up through the pitch pads), `fx` (50 N forward). Writes `fem/<tag>[_<case>]/` (`summary.txt`, `stress.npy`, CalculiX files) |
| `run_cases.py <tag>...` | runs `fem_yoke.py` for all five cases of each tag, three at a time; logs in `fem/log_<tag>_<case>.txt` |
| `interlayer.py <tag>...` | resolves each case's stress in each piece's print frame (roll piece: layers normal to Y; pitch piece: normal to X) and on the glue plane (peel, shear); bolt-hole walls excluded; hot spots 5 mm apart |
| `compare.py <tag>...` | the worst values per case and measure, side by side, with locations (the record's table) |
| `fig_interlayer.py <before> <after>` | the record's figure: worst interlayer stress per member over all five cases, two designs, with a `.csv` of the values |
| `probe_space.py` | free-space probe around the roll horn arm: boxes tested against the neighbours that move relative to the yoke (roll servo and carrier over hip roll, thigh and pitch servo over hip pitch), overlap and minimum distance per angle |
| `sweep_maps.py` | occupancy rasters of the swept neighbours around the front corner (the roll horn arm), written as `sweep_maps.png` and `.npz` |
| `sweep_maps2.py` | the same for the idler side; `MAPNAME=sweep_idler` names its output |
| `adduction.py` | first inter-leg contact as one hip adducts toward the other leg: per angle, the other leg's closest piece (`OLD=1` with an older `hip_yoke_glued.py` in `old/` for an A/B) |

```bash
.venv/bin/python experiments/hip-yoke-strength/export_glued.py v6
.venv/bin/python experiments/hip-yoke-strength/run_cases.py v6
.venv/bin/python experiments/hip-yoke-strength/compare.py baseline v6
MAPNAME=sweep_idler .venv/bin/python experiments/hip-yoke-strength/sweep_maps2.py
```

To compare against another design, check it out, run `export_glued.py <tag>`
from there and `run_cases.py <tag>`; `compare.py` takes any tags whose results
are in the output directory. The record's `baseline` is `hip_yoke_glued.py` at
commit `ab92371`.
