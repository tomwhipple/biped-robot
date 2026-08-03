# General Driver for Robots — official documentation index

Waveshare's manufacturer-published documents for this board, all mirrored
here verbatim:

| File | What it is |
|---|---|
| [General_Driver_for_Robots-schematic.pdf](General_Driver_for_Robots-schematic.pdf) | Full circuit schematic |
| [General_Driver_for_Robots-dimensions.pdf](General_Driver_for_Robots-dimensions.pdf) | Dimension drawing (65 × 65 mm, holes 49 × 58 mm, Ø3) |
| [General_Driver_for_Robots-connector-diagram.jpg](General_Driver_for_Robots-connector-diagram.jpg) | Official annotated board photo — every connector numbered 1–27, front and back |
| [JST-XH-connector-datasheet.pdf](JST-XH-connector-datasheet.pdf) | JST XH series datasheet (the power-inlet connector family) |

**Waveshare publishes no user manual for this board.** The connector table
(number → type → function, matching the diagram above), tutorials, and FAQ
exist only on the official wiki:
<https://www.waveshare.com/w/index.php?title=General_Driver_for_Robots&oldid=103968>
(stable permalink, retrieved 2026-08-03). STEP model and demo code are also
linked there. Our schematic-derived connector→GPIO net map is in
[sensor-expansion.md](../../sensor-expansion.md) §1.

## Power input (the battery connection)

Per the official wiki and the connector diagram: power enters at the
**XH2.54 2-pin port** (diagram № 10, bottom-left, silkscreened **− +**),
controlled by the adjacent power switch (№ 12); this input directly powers
the bus servos and motors. Our chain: 3S pack (XT30) → inline switch →
XH 2-pin pigtail (BOM item 22).

⚠️ Open question for Waveshare support: the wiki states 7–13 V (2S or 3S),
but the **rev1.2 silkscreen reads "DC 9-12.6V"** — which would exclude 2S.
Our 11.1 V 3S pack is inside both ranges. Also note the FAQ's **5 A
continuous limit** on the bus-servo path (~5 ST3215s, not stalled
simultaneously) vs our 8 servos / ~10 A transient budget — unresolved; see
BOM item 22.
