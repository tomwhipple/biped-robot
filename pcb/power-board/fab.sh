#!/bin/sh
# Fabrication outputs for the power board, regenerated from the KiCad sources.
#
#   sh pcb/power-board/fab.sh     # exit 0 = ERC and DRC clean, outputs written to pcb/power-board/fab/
#
# ERC and DRC run first and stop the export on any violation. Then:
#   fab/power-board-gerbers.zip   Gerbers (4 copper layers, mask, paste, silk, outline) + Excellon drill,
#                                 the file a board house (PCBWay or similar) takes
#   fab/power-board-pos.csv       component placement for assembly
#   fab/power-board-bom.csv       bill of materials grouped by value / footprint / MPN
#   fab/erc.rpt, fab/drc.rpt      the check reports
set -eu
cd "$(dirname "$0")"
KICAD_CLI=${KICAD_CLI:-kicad-cli}
command -v "$KICAD_CLI" >/dev/null 2>&1 || KICAD_CLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli

rm -rf fab
mkdir -p fab/gerbers
"$KICAD_CLI" sch erc --exit-code-violations --severity-all -o fab/erc.rpt power-board.kicad_sch
"$KICAD_CLI" pcb drc --exit-code-violations --severity-all -o fab/drc.rpt power-board.kicad_pcb
"$KICAD_CLI" pcb export gerbers --check-zones --subtract-soldermask \
    -l F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts \
    -o fab/gerbers/ power-board.kicad_pcb
"$KICAD_CLI" pcb export drill --excellon-separate-th --generate-map --map-format pdf \
    -o fab/gerbers/ power-board.kicad_pcb
"$KICAD_CLI" pcb export pos --format csv --units mm --side front -o fab/power-board-pos.csv power-board.kicad_pcb
"$KICAD_CLI" sch export bom --fields 'Reference,Value,Footprint,MPN,Datasheet,${QUANTITY}' \
    --labels 'Refs,Value,Footprint,MPN,Datasheet,Qty' --group-by 'Value,Footprint,MPN' \
    -o fab/power-board-bom.csv power-board.kicad_sch
(cd fab/gerbers && zip -q -X -r ../power-board-gerbers.zip .)
rm -rf fab/gerbers
echo "fab outputs in $(pwd)/fab"
