#!/bin/sh
# Fabrication outputs for the power-lite board, regenerated from the KiCad sources.
#
#   sh pcb/power-lite/fab.sh     # exit 0 = ERC and DRC clean, outputs written to pcb/power-lite/fab/
#
# ERC and DRC (with schematic parity) run first and stop the export on any violation. Then:
#   fab/power-lite-gerbers.zip   Gerbers (2 copper layers, mask, silk both sides, outline) + Excellon drill,
#                                the file a board house (PCBWay or similar) takes; every part is through-hole
#                                and hand-soldered, so there is no paste layer and no placement file
#   fab/power-lite-bom.csv       bill of materials for ordering the parts, grouped by value / footprint / MPN
#   fab/erc.rpt, fab/drc.rpt     the check reports
set -eu
cd "$(dirname "$0")"
KICAD_CLI=${KICAD_CLI:-kicad-cli}
command -v "$KICAD_CLI" >/dev/null 2>&1 || KICAD_CLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli

rm -rf fab
mkdir -p fab/gerbers
"$KICAD_CLI" sch erc --exit-code-violations --severity-all -o fab/erc.rpt power-lite.kicad_sch
"$KICAD_CLI" pcb drc --exit-code-violations --severity-all --schematic-parity -o fab/drc.rpt power-lite.kicad_pcb
"$KICAD_CLI" pcb export gerbers --check-zones --subtract-soldermask \
    -l F.Cu,B.Cu,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts \
    -o fab/gerbers/ power-lite.kicad_pcb
"$KICAD_CLI" pcb export drill --excellon-separate-th --generate-map --map-format pdf \
    -o fab/gerbers/ power-lite.kicad_pcb
"$KICAD_CLI" sch export bom --fields 'Reference,Value,Footprint,MPN,Datasheet,${QUANTITY}' \
    --labels 'Refs,Value,Footprint,MPN,Datasheet,Qty' --group-by 'Value,Footprint,MPN' --exclude-dnp \
    -o fab/power-lite-bom.csv power-lite.kicad_sch
(cd fab/gerbers && zip -q -X ../power-lite-gerbers.zip ./*)
rm -rf fab/gerbers
"$KICAD_CLI" sch export pdf -o power-lite.pdf power-lite.kicad_sch >/dev/null
echo "fab outputs written to $(pwd)/fab"
