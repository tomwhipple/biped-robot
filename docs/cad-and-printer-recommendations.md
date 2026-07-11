# CAD software & 3D printer recommendations

*Compiled 2026-07-10, for building the Bimo-like biped (8× STS3215 servos, ~34 cm tall).
Parts are small — the longest single piece (thigh/shin link) is well under 120 mm — so even a
compact print bed is plenty.*

## CAD software (low-cost / free)

The repo's CAD is **code-based** (`cad/` — build123d/CadQuery Python scripts), so the primary
"CAD software" is already free: a text editor plus a 3D viewer. GUI CAD is still useful for
inspecting STLs, one-off tweaks, and designing around purchased parts.

| Tool | Cost | Best for | Trade-offs |
|---|---|---|---|
| **VS Code + OCP CAD Viewer** (recommended, pairs with this repo) | Free | Editing `cad/*.py` with a live 3D preview of build123d/CadQuery models | Code-first; not a drawing tool |
| **Onshape (free tier)** (recommended GUI) | Free | Full professional parametric CAD in the browser, real assemblies/mates, zero install, runs on anything | Free-tier documents are **public**; fine for an open hobby robot, not for anything private |
| **FreeCAD 1.0+** | Free, open source | Fully local, no cloud, no license risk; imports/exports STEP & STL; huge community | Steeper learning curve, occasional UX roughness vs. commercial tools |
| **Autodesk Fusion (Personal)** | Free for hobbyists | Slickest modeling + built-in CAM/simulation | License limits (10 editable docs, no STEP export on personal tier as of recent changes); Autodesk has repeatedly trimmed the free tier |

**Recommendation:** stay code-first with build123d + OCP viewer for the robot's printed parts
(parametric, diffable in git, regenerates all STLs from one dimensions file), and keep an
**Onshape** free account for assembly sanity checks or quick GUI edits. Choose **FreeCAD**
instead of Onshape if you want everything local/open-source.

For slicing (STL → printer): the slicer comes free with the printer — Bambu Studio, PrusaSlicer,
or the open-source OrcaSlicer (works with nearly every printer, arguably the best of the three).

## 3D printer (reasonably priced)

| Printer | Price (mid-2026) | Build volume | Why / why not |
|---|---|---|---|
| **Bambu Lab A1 mini** ⭐ top pick | **$199–219** | 180×180×180 mm | Best print quality and reliability per dollar; auto-calibration, genuinely plug-and-play; fast (real-world ~500 mm/s class); handles PLA/PETG and TPU (for foot pads). 180 mm bed is ample for every part of this robot. |
| **Bambu Lab A1** | ~$349 | 256×256×256 mm | Same platform, bigger bed — worth it if you want one-piece torso shells or future bigger robots. |
| **Prusa MINI+** | ~$429 kit | 180×180×180 mm | Open-source, fully documented, every spare part purchasable — the repairability/longevity pick. Slower and pricier than the A1 mini. |
| **Creality Ender-3 V3 (KE/SE)** | ~$180–250 | 220×220×240 mm | Cheapest capable option; more tinkering and tuning required — the hours you spend tuning are hours not spent on the robot. |

**Recommendation: Bambu Lab A1 mini ($199–219).** For a robotics project the printer should be a
tool, not a second hobby — the A1 mini is the closest thing to "click print, get part." Add the
AMS Lite (+~$110) only if you want multi-color; it's unnecessary for functional robot parts.

Caveats worth knowing:
- **Bambu ecosystem is partly closed** (cloud-tied firmware; a 2025 firmware-authorization
  controversy). LAN-only mode exists and works. If open-source matters to you on principle,
  that's the case for the Prusa MINI+ despite ~2× the price.
- Whatever you buy, get a **0.4 mm nozzle (stock)**, PLA for structural prototyping, PETG for
  final servo brackets (tougher, slightly heat-resistant near warm servos), and a small roll of
  TPU for foot soles/grip pads.
- Budget ~$25–30 for an M2/M3 screw + heat-set insert assortment; the CAD in `cad/` assumes
  heat-set inserts for anything load-bearing.

## Sources

- [Tom's Hardware — Best 3D Printers 2026](https://www.tomshardware.com/best-picks/best-3d-printers)
- [ADP Industries — Best 3D Printer 2026 (6-printer farm operator)](https://www.adpindustries.com/blog/best-3d-printer-2026/)
- [LayerMath — Prusa vs Bambu 2026, quality/speed/total cost](https://layermath.com/blog/prusa-vs-bambu-2026)
- [BrandChoose — Prusa vs Bambu Lab 2026](https://brandchoose.com/compare/3d-printers/prusa/bambu-lab)
- [Makers101 — Bambu Lab A1 Mini review 2026](https://makers101.com/bambu-lab-a1-mini-worth-it-beginners/)
